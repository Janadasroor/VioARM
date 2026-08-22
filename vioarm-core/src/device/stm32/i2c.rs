//! The STM32 I2C peripheral (`I2C1`/`I2C2`, plus `I2C3` on the F4), driven by
//! the SVD-generated register tables.
//!
//! Register layout and resets come from the SVD; the table cannot express the
//! bus, so [`I2cBehavior`] runs a master-mode state machine over it. Setting
//! `CR1.START` while enabled and idle generates a start: after
//! [`START_CYCLES`] `SR1.SB` is set (and `CR1.START` self-clears, as on
//! silicon). The firmware then writes the address byte (7-bit address plus
//! read/write) to `DR`; when the byte is shifted out the slave's
//! acknowledge sets `SR1.ADDR`, with `SR2.TRA` reflecting the direction.
//! Reading `SR2` acknowledges `ADDR`, and in master-receive mode streams in
//! the byte the host slave drives, setting `SR1.RXNE` with `DR` filled.
//! Master-transmit data bytes shift out the same way, restoring `SR1.TXE` on
//! completion. `CR1.STOP` ends the transfer after [`BYTE_CYCLES`], releasing
//! the bus (`SR2.BUSY` drops) and clearing the `START`/`STOP` bits.
//!
//! The host injects the connected slave's response through
//! [`Peripheral::drive_inputs`]: the low byte is what the slave returns for
//! every master-read byte. Driving zero makes the slave absent — the address
//! phase is then acknowledged with `SR1.AF` instead of `ADDR`, raising the
//! error interrupt. The master's shifted-out bytes (addresses and data) are
//! captured for the host through [`Peripheral::drain_output`].
//!
//! `CR2.ITEVTEN`/`ITBUFEN` raise the event line (line 0) for `SB`/`ADDR`/
//! `BTF`/`STOPF` and the data-ready `TXE`/`RXNE`; `CR2.ITERREN` raises the
//! error line (line 1) for `AF` and the other `SR1` error flags. Both are
//! level-triggered and mask exactly like the silicon interrupt controller.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

/// Register offsets within the I2C, per the SVD.
const CR1: u32 = 0x00;
const CR2: u32 = 0x04;
const DR: u32 = 0x10;
const SR1: u32 = 0x14;
const SR2: u32 = 0x18;

/// `CR1` bits.
const PE: u32 = 1 << 0;
const START: u32 = 1 << 8;
const STOP: u32 = 1 << 9;
const ACK: u32 = 1 << 10;
const SWRST: u32 = 1 << 15;

/// `CR2` bits.
const ITEVTEN: u32 = 1 << 9;
const ITBUFEN: u32 = 1 << 10;
const ITERREN: u32 = 1 << 8;

/// `SR1` bits.
const SB: u32 = 1 << 0;
const ADDR: u32 = 1 << 1;
const BTF: u32 = 1 << 2;
const STOPF: u32 = 1 << 4;
const RXNE: u32 = 1 << 6;
const TXE: u32 = 1 << 7;
const BERR: u32 = 1 << 8;
const ARLO: u32 = 1 << 9;
const AF: u32 = 1 << 10;
const OVR: u32 = 1 << 11;
const PECERR: u32 = 1 << 12;
const TIMEOUT: u32 = 1 << 14;
const SMBALERT: u32 = 1 << 15;

/// `SR1` flags software clears by writing zero; `TXE`/`RXNE` are hardware-only.
const CLEARABLE: u32 =
    SB | ADDR | BTF | STOPF | BERR | ARLO | AF | OVR | PECERR | TIMEOUT | SMBALERT;

/// `SR2` bits.
const MSL: u32 = 1 << 0;
const BUSY: u32 = 1 << 1;
const TRA: u32 = 1 << 2;

/// Machine cycles a start or stop condition takes.
const START_CYCLES: u64 = 8;
/// Machine cycles a byte (address or data) takes on the wire.
const BYTE_CYCLES: u64 = 8;

/// Where a transfer sits in its sequence.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Phase {
    /// Nothing in flight.
    #[default]
    Idle,
    /// `START` asserted; `SB` is still on its way.
    Start,
    /// `SB` set; the address byte is expected in `DR`.
    Addr,
    /// Address acknowledged; a data byte is moving.
    Data,
    /// `STOP` asserted; the bus is being released.
    Stop,
}

/// The bus the SVD cannot express: a master-mode transfer state machine over
/// the start/address/data/stop sequence.
#[derive(Debug, Default)]
struct I2cBehavior {
    /// The byte the connected slave returns for each master-read byte.
    slave_byte: u8,
    /// The current transfer phase.
    phase: Phase,
    /// Machine cycles until the current phase completes.
    remaining: u64,
    /// Direction of the active data phase (`true` = master receive).
    receiving: bool,
    /// Bytes the master shifted out, for host draining.
    tx: Vec<u8>,
}

impl I2cBehavior {
    /// Begin a start condition, clearing any pending stop.
    fn begin_start(&mut self) {
        self.phase = Phase::Start;
        self.remaining = START_CYCLES;
    }

    /// Clear the transfer state after a start, stop, or soft reset.
    fn reset_transfer(&mut self) {
        self.phase = Phase::Idle;
        self.remaining = 0;
        self.receiving = false;
    }
}

impl Behavior for I2cBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            CR1 => {
                let old = view.get(CR1);
                let new = view.merged(CR1, val, shift);
                if new & SWRST != 0 {
                    // Soft reset aborts the transfer and drops the buffers.
                    self.reset_transfer();
                    self.tx.clear();
                    view.set(SR2, view.get(SR2) & !(BUSY | MSL));
                    view.set(SR1, view.get(SR1) & !(SB | ADDR | BTF | STOPF | RXNE | TXE));
                } else {
                    let start_edge = new & START != 0 && old & START == 0;
                    let stop_edge = new & STOP != 0 && old & STOP == 0;
                    if start_edge
                        && new & PE != 0
                        && matches!(self.phase, Phase::Idle | Phase::Stop)
                    {
                        self.begin_start();
                    } else if stop_edge && matches!(self.phase, Phase::Addr | Phase::Data) {
                        self.phase = Phase::Stop;
                        self.remaining = BYTE_CYCLES;
                    }
                }
                new
            }
            DR => {
                let new = view.merged(DR, val, shift);
                // Only the low byte of the data register is the wire byte.
                if shift == 0 {
                    match self.phase {
                        Phase::Addr => {
                            // The address byte (7-bit address plus read/write)
                            // goes out; SB was the event that led here.
                            self.tx.push(val);
                            self.receiving = val & 1 != 0;
                            view.set(SR1, view.get(SR1) & !SB);
                            self.remaining = BYTE_CYCLES;
                        }
                        Phase::Data if !self.receiving => {
                            self.tx.push(val);
                            view.set(SR1, view.get(SR1) & !TXE);
                            self.remaining = BYTE_CYCLES;
                        }
                        _ => {}
                    }
                }
                new
            }
            SR1 => {
                let new = view.merged(SR1, val, shift);
                // Write-zero-to-clear for the software-clearable flags, leaving
                // the hardware-driven `TXE`/`RXNE` untouched.
                let clear = CLEARABLE & !new;
                view.set(SR1, view.get(SR1) & !clear);
                view.get(SR1)
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            SR2 => {
                // Reading SR1 and then SR2 acknowledges `ADDR`; a master
                // receiver then starts clocking the first data byte.
                if view.get(SR1) & ADDR != 0 {
                    view.set(SR1, view.get(SR1) & !ADDR);
                    if matches!(self.phase, Phase::Data) && self.receiving {
                        self.remaining = BYTE_CYCLES;
                    }
                }
                value
            }
            DR => {
                // Reading the data register acknowledges `RXNE`; with `ACK`
                // still set a master receiver clocks the next byte.
                if view.get(SR1) & RXNE != 0 {
                    view.set(SR1, view.get(SR1) & !RXNE);
                    if matches!(self.phase, Phase::Data)
                        && self.receiving
                        && view.get(CR1) & ACK != 0
                    {
                        self.remaining = BYTE_CYCLES;
                    }
                }
                value
            }
            _ => value,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if self.remaining == 0 {
            return;
        }
        if self.remaining > delta {
            self.remaining -= delta;
            return;
        }
        match self.phase {
            Phase::Start => {
                // The start condition is on the wire: `SB` rises and the
                // `START`/`STOP` bits self-clear, ready for the address byte.
                view.set(SR1, view.get(SR1) | SB);
                view.set(SR2, view.get(SR2) | (BUSY | MSL));
                view.set(CR1, view.get(CR1) & !(START | STOP));
                self.phase = Phase::Addr;
                self.remaining = 0;
            }
            Phase::Addr => {
                // The address byte shifted out and was acknowledged.
                if self.slave_byte == 0 {
                    // No slave on the bus: the address is NACKed and the
                    // transfer aborts with the error flag set.
                    view.set(SR1, (view.get(SR1) & !(SB | ADDR)) | AF);
                    self.reset_transfer();
                } else {
                    view.set(SR1, (view.get(SR1) & !SB) | ADDR | TXE);
                    let sr2 = view.get(SR2) | (BUSY | MSL);
                    view.set(
                        SR2,
                        if self.receiving {
                            sr2 & !TRA
                        } else {
                            sr2 | TRA
                        },
                    );
                    self.phase = Phase::Data;
                    self.remaining = 0;
                }
            }
            Phase::Data if self.receiving => {
                // A byte from the slave landed in the data register.
                view.set(DR, u32::from(self.slave_byte));
                view.set(SR1, view.get(SR1) | RXNE);
                self.remaining = 0;
            }
            Phase::Data => {
                // A transmitted byte completed: the buffer is empty again.
                view.set(SR1, view.get(SR1) | TXE);
                self.remaining = 0;
            }
            Phase::Stop => {
                // The stop condition ends the transfer and releases the bus.
                view.set(CR1, view.get(CR1) & !(START | STOP));
                view.set(SR2, view.get(SR2) & !BUSY);
                self.reset_transfer();
            }
            Phase::Idle => {}
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let cr2 = view.get(CR2);
        let sr1 = view.get(SR1);
        let itevt = cr2 & ITEVTEN != 0;
        let event = itevt && sr1 & (SB | ADDR | BTF | STOPF) != 0;
        let buffer = itevt && cr2 & ITBUFEN != 0 && sr1 & (TXE | RXNE) != 0;
        let error = cr2 & ITERREN != 0 && sr1 & (BERR | ARLO | AF | OVR) != 0;
        u32::from(event || buffer) | (u32::from(error) << 1)
    }

    fn lines(&self) -> u32 {
        3
    }

    fn reset(&mut self) {
        self.reset_transfer();
        self.tx.clear();
    }

    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        // The low byte is the slave's response; zero marks an absent slave.
        self.slave_byte = (value & 0xFF) as u8;
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.tx)
    }
}

/// The `I2C1` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct I2c {
    inner: RegisterFile<I2cBehavior>,
}

impl I2c {
    /// A new `I2C1` at the STM32F1 base address.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `I2C1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        Self::for_device::<stm32f103::Stm32f103>()
    }

    /// A new `I2C1` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `I2C1`
    /// peripheral.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("I2C1").expect("generated tables contain I2C1");
        Self {
            inner: RegisterFile::new(def, I2cBehavior::default()),
        }
    }

    /// A new `I2C2` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `I2C2`
    /// peripheral.
    #[must_use]
    pub fn i2c2_for<D: Device>() -> Self {
        let def = D::peripheral("I2C2").expect("generated tables contain I2C2");
        Self {
            inner: RegisterFile::new(def, I2cBehavior::default()),
        }
    }

    /// A new `I2C3` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `I2C3`
    /// peripheral.
    #[must_use]
    pub fn i2c3_for<D: Device>() -> Self {
        let def = D::peripheral("I2C3").expect("generated tables contain I2C3");
        Self {
            inner: RegisterFile::new(def, I2cBehavior::default()),
        }
    }
}

impl Default for I2c {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for I2c {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }

    fn name(&self) -> &'static str {
        "i2c"
    }

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}

impl crate::device::InterruptSource for I2c {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }

    fn lines(&self) -> u32 {
        self.inner.lines()
    }

    fn exception_for_line(&self, line: u32) -> Option<u8> {
        self.inner.exception_for_line(line)
    }
}

#[cfg(test)]
mod tests {
    use super::{
        I2c, ACK, ADDR, AF, BUSY, BYTE_CYCLES, CR1, CR2, DR, ITBUFEN, ITERREN, ITEVTEN, MSL, PE,
        RXNE, SB, SR1, SR2, START, START_CYCLES, STOP, SWRST, TRA, TXE,
    };
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const BASE: u32 = 0x4000_5400;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(u: &mut I2c, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            u.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(u: &mut I2c, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = u.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn enabled() -> I2c {
        let mut i = I2c::new();
        write_word(&mut i, BASE + CR1, PE);
        i
    }

    fn start(i: &mut I2c) {
        write_word(i, BASE + CR1, PE | START);
        i.tick(START_CYCLES);
    }

    fn send_addr(i: &mut I2c, addr: u8) {
        write_word(i, BASE + DR, u32::from(addr));
        i.tick(BYTE_CYCLES);
    }

    #[test]
    fn a_master_write_shifts_address_then_data() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        assert_eq!(read_word(&mut i, BASE + SR1) & SB, SB);
        // START self-cleared, bus claimed.
        assert_eq!(read_word(&mut i, BASE + CR1) & START, 0);
        assert_eq!(read_word(&mut i, BASE + SR2) & (BUSY | MSL), BUSY | MSL);
        // Address byte: ADDR + TXE, direction = transmit.
        send_addr(&mut i, 0xA0);
        assert_eq!(read_word(&mut i, BASE + SR1) & (ADDR | TXE), ADDR | TXE);
        assert_eq!(read_word(&mut i, BASE + SR2) & TRA, TRA);
        // Clear ADDR, then send data bytes.
        let _ = read_word(&mut i, BASE + SR2);
        for byte in [0x11u32, 0x22, 0x33] {
            write_word(&mut i, BASE + DR, byte);
            assert_eq!(read_word(&mut i, BASE + SR1) & TXE, 0);
            i.tick(BYTE_CYCLES);
            assert_eq!(read_word(&mut i, BASE + SR1) & TXE, TXE);
        }
        assert_eq!(i.drain_output(), [0xA0, 0x11, 0x22, 0x33]);
    }

    #[test]
    fn a_master_receive_latches_the_slave_byte() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        send_addr(&mut i, 0xA1); // address | read
        assert_eq!(read_word(&mut i, BASE + SR2) & TRA, 0);
        // Clearing ADDR streams in the slave's byte.
        let _ = read_word(&mut i, BASE + SR2);
        assert_eq!(read_word(&mut i, BASE + SR1) & RXNE, 0);
        i.tick(BYTE_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & RXNE, RXNE);
        assert_eq!(read_word(&mut i, BASE + DR) & 0xFF, 0xA5);
        assert_eq!(read_word(&mut i, BASE + SR1) & RXNE, 0);
        // ACK is clear: the master NACKs and no further byte arrives.
        i.tick(BYTE_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & RXNE, 0);
    }

    #[test]
    fn ack_streams_the_receive_until_the_last_byte() {
        let mut i = enabled();
        i.drive_inputs(0x5A);
        start(&mut i);
        send_addr(&mut i, 0xA1);
        write_word(&mut i, BASE + CR1, PE | ACK);
        let _ = read_word(&mut i, BASE + SR2);
        for _ in 0..3 {
            i.tick(BYTE_CYCLES);
            assert_eq!(read_word(&mut i, BASE + SR1) & RXNE, RXNE);
            assert_eq!(read_word(&mut i, BASE + DR) & 0xFF, 0x5A);
        }
    }

    #[test]
    fn stop_releases_the_bus_and_clears_the_stop_bit() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        send_addr(&mut i, 0xA0);
        let _ = read_word(&mut i, BASE + SR2);
        write_word(&mut i, BASE + CR1, PE | STOP);
        assert_eq!(read_word(&mut i, BASE + SR2) & BUSY, BUSY);
        i.tick(BYTE_CYCLES);
        assert_eq!(read_word(&mut i, BASE + CR1) & (START | STOP), 0);
        assert_eq!(read_word(&mut i, BASE + SR2) & BUSY, 0);
        // A fresh start is accepted after the stop.
        write_word(&mut i, BASE + CR1, PE | START);
        i.tick(START_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & SB, SB);
    }

    #[test]
    fn an_absent_slave_nacks_the_address() {
        let mut i = enabled();
        i.drive_inputs(0); // no slave on the bus
        start(&mut i);
        send_addr(&mut i, 0xA0);
        assert_eq!(read_word(&mut i, BASE + SR1) & (ADDR | AF), AF);
    }

    #[test]
    fn sr1_writes_clear_only_the_software_flags() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        send_addr(&mut i, 0xA0);
        assert_eq!(read_word(&mut i, BASE + SR1) & (ADDR | TXE), ADDR | TXE);
        // A write-zero-to-clear over the whole register keeps TXE/RXNE.
        write_word(&mut i, BASE + SR1, 0);
        assert_eq!(read_word(&mut i, BASE + SR1) & (ADDR | TXE), TXE);
    }

    #[test]
    fn event_interrupt_is_level_triggered_and_masked() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        assert_eq!(i.pending(), 0);
        // Data-ready flags need ITEVTEN | ITBUFEN.
        write_word(&mut i, BASE + CR2, ITEVTEN | ITBUFEN);
        start(&mut i);
        send_addr(&mut i, 0xA0);
        assert_ne!(i.pending() & 1, 0);
        // Draining DR/TXE isn't enough: the flags must clear.
        let _ = read_word(&mut i, BASE + SR2);
        i.reset();
        assert_eq!(i.pending(), 0);
    }

    #[test]
    fn error_interrupt_tracks_an_absent_slave() {
        let mut i = enabled();
        i.drive_inputs(0);
        write_word(&mut i, BASE + CR2, ITEVTEN | ITBUFEN | ITERREN);
        start(&mut i);
        send_addr(&mut i, 0xA0);
        assert_eq!(i.pending() & 2, 2);
        // A write-zero-to-clear of AF drops the line.
        write_word(&mut i, BASE + SR1, 0);
        assert_eq!(i.pending(), 0);
    }

    #[test]
    fn soft_reset_aborts_the_transfer() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        send_addr(&mut i, 0xA0);
        let _ = read_word(&mut i, BASE + SR2);
        write_word(&mut i, BASE + CR1, PE | SWRST);
        assert_eq!(read_word(&mut i, BASE + SR2) & BUSY, 0);
        assert_eq!(i.drain_output(), b"");
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut i = I2c::new();
        assert!(matches!(i.read(BASE + 0x24), Err(BusError::Reserved)));
        assert!(matches!(i.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}
