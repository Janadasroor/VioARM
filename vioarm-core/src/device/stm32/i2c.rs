//! The STM32 I2C peripheral (`I2C1`/`I2C2`, plus `I2C3` on the F4), driven by
//! the SVD-generated register tables.
//!
//! Register layout and resets come from the SVD; the table cannot express the
//! bus, so `I2cBehavior` runs a master-mode state machine over it. Setting
//! `CR1.START` while enabled and idle generates a start: after
//! `START_CYCLES` `SR1.SB` is set (and `CR1.START` self-clears, as on
//! silicon). The firmware then writes the address byte (7-bit address plus
//! read/write) to `DR`; when the byte is shifted out the slave's
//! acknowledge sets `SR1.ADDR`, with `SR2.TRA` reflecting the direction.
//! Reading `SR2` acknowledges `ADDR`, and in master-receive mode streams in
//! the byte the host slave drives, setting `SR1.RXNE` with `DR` filled.
//! Master-transmit data bytes shift out the same way, restoring `SR1.TXE` on
//! completion. `CR1.STOP` ends the transfer after `BYTE_CYCLES`, releasing
//! the bus (`SR2.BUSY` drops) and clearing the `START`/`STOP` bits.
//!
//! The host injects the connected slave's response through
//! [`Peripheral::drive_inputs`]: the low byte is what the slave returns for
//! every master-read byte. Driving zero makes the slave absent — the address
//! phase is then acknowledged with `SR1.AF` instead of `ADDR`, raising the
//! error interrupt. Bit 8 stretches the in-flight (or next) byte by
//! `STRETCH_CYCLES`, modeling a slow slave holding the clock. The master's
//! shifted-out bytes (addresses and data) are captured for the host through
//! [`Peripheral::drain_output`].
//!
//! Ten-bit addresses ride the same phase machine: a `11110xx0` header byte
//! keeps the address phase open for the second byte (a `11110xx1` header
//! completes at once, covering the read-after-repeated-start form).
//! Repeated start is legal mid-transfer. Byte-transfer-finished (`BTF`)
//! marks completions for `BTF`-polling firmware. Deliberately unmodeled:
//! slave mode (this is a master-only engine), receive overrun (the engine
//! only clocks the next byte after firmware reads `DR`, so it cannot
//! happen — the clock stretches instead, as on silicon), multi-master
//! arbitration (`ARLO`/`BERR`), SMBus packet error (`PECERR`) and
//! timeouts — the flags clear by software but never latch from the bus.
//!
//! `CR2.ITEVTEN`/`ITBUFEN` raise the event line (line 0) for `SB`/`ADDR`/
//! `BTF`/`STOPF` and the data-ready `TXE`/`RXNE`; `CR2.ITERREN` raises the
//! error line (line 1) for `AF` and the other `SR1` error flags. Both are
//! level-triggered and mask exactly like the silicon interrupt controller.
//!
//! New-generation tables (F0/F3/L0/L4: `TIMINGR`/`ISR`/`ICR`/`RXDR`/`TXDR`
//! instead of `SR1`/`SR2`/`DR`) run a second engine over the same phase
//! machine, detected per instance from the register names. Firmware writes
//! `CR2` (`SADD`, `RD_WRN`, `NBYTES`, `START`/`STOP`/`AUTOEND`); the engine
//! raises `ADDR` (+`TXIS` for transmit) or `NACKF` for the address byte,
//! then `TXIS`/`RXNE` per data byte and `TC` at the count, with `STOPF`
//! after the stop. Flags clear through `ICR` (`ADDRCF`/`NACKCF`/`STOPCF`/
//! `TCCF`); `TC` also clears on the next `START`. Byte pacing derives from
//! `TIMINGR` (`PRESC`/`SCLL`/`SCLH`: 9 clocks per byte). The absent-slave
//! contract is shared with the F1 engine: a zero staged byte NACKs the
//! address. Deliberately unmodeled on this path: 10-bit addresses,
//! `RELOAD`, `NACK`-bit generation, SMBus/PEC/wakeup, DMA — the registers
//! latch, the engine ignores them.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, PeripheralDef, RegisterFile, Registers};
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

/// New-generation register offsets (F0/F3/L0/L4). `CR1`/`CR2` share the F1
/// offsets with different fields; `DR`/`SR1`/`SR2` do not exist there
/// (`TIMINGR`/`TIMEOUTR` sit at those offsets instead).
const NG_TIMINGR: u32 = 0x10;
const NG_ISR: u32 = 0x18;
const NG_ICR: u32 = 0x1C;
const NG_RXDR: u32 = 0x24;
const NG_TXDR: u32 = 0x28;

/// New-generation `CR1` interrupt enables.
const NG_TXIE: u32 = 1 << 1;
const NG_RXIE: u32 = 1 << 2;
const NG_ADDRIE: u32 = 1 << 3;
const NG_NACKIE: u32 = 1 << 4;
const NG_STOPIE: u32 = 1 << 5;
const NG_TCIE: u32 = 1 << 6;
const NG_ERRIE: u32 = 1 << 7;

/// New-generation `CR2` fields.
const NG_SADD_MASK: u32 = 0x3FF;
const NG_RD_WRN: u32 = 1 << 10;
const NG_START: u32 = 1 << 13;
const NG_STOP: u32 = 1 << 14;
const NG_NBYTES_SHIFT: u32 = 16;
const NG_AUTOEND: u32 = 1 << 25;

/// New-generation `ISR` flags.
const NG_TXIS: u32 = 1 << 1;
const NG_RXNE: u32 = 1 << 2;
const NG_ADDR: u32 = 1 << 3;
const NG_NACKF: u32 = 1 << 4;
const NG_STOPF: u32 = 1 << 5;
const NG_TC: u32 = 1 << 6;
const NG_BUSY: u32 = 1 << 15;
const NG_DIR: u32 = 1 << 16;
const NG_ADDCODE_SHIFT: u32 = 17;

/// New-generation `ICR` clear bits.
const NG_ADDRCF: u32 = 1 << 3;
const NG_NACKCF: u32 = 1 << 4;
const NG_STOPCF: u32 = 1 << 5;
const NG_TCCF: u32 = 1 << 6;

/// Dynamic `ISR` flags (everything the engine sets or clears).
const NG_DYNAMIC: u32 =
    NG_TXIS | NG_RXNE | NG_ADDR | NG_NACKF | NG_STOPF | NG_TC | NG_BUSY | NG_DIR | 0x7F << 17;

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
/// Extra cycles a stretched byte takes (clock held by the slave).
const STRETCH_CYCLES: u64 = 64;

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
    /// A 10-bit header (`11110xx0`) was sent; the second address byte is
    /// still expected before the address phase completes.
    addr_second: bool,
    /// The next byte transfer is clock-stretched by the slave (set through
    /// the stimulus channel, consumed at the next byte start).
    stretch_pending: bool,
    /// Bytes the master shifted out, for host draining.
    tx: Vec<u8>,
    /// New-generation register layout (F0/F3/L0/L4 `TIMINGR`/`ISR` engine).
    new_gen: bool,
    /// Bytes left in the new-generation transfer (`CR2.NBYTES` count).
    nbytes_left: u8,
    /// Latched `CR2` direction (`true` = master receive) and auto-end.
    ng_receiving: bool,
    ng_autoend: bool,
    /// Latched 7-bit slave address for `ISR.ADDCODE`.
    ng_sadd: u16,
}

impl I2cBehavior {
    /// Whether `def` carries the new-generation register set.
    fn is_new_gen(def: &PeripheralDef) -> bool {
        def.registers.iter().any(|r| r.name == "TIMINGR")
    }

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
        self.addr_second = false;
    }

    /// Cycles a newly started byte takes, consuming a pending stretch.
    fn byte_cycles(&mut self) -> u64 {
        if self.stretch_pending {
            self.stretch_pending = false;
            BYTE_CYCLES + STRETCH_CYCLES
        } else {
            BYTE_CYCLES
        }
    }

    /// New-generation byte time from `TIMINGR`: 9 SCL clocks per byte,
    /// each `(PRESC + 1) * (SCLL + SCLH + 2)` device ticks.
    fn ng_byte_cycles(&mut self, view: &Registers<&mut [u32]>) -> u64 {
        let t = view.get(NG_TIMINGR);
        let presc = (t >> 28) & 0xF;
        let scl = (t & 0xFF) + ((t >> 8) & 0xFF) + 2;
        let base = 9 * u64::from(presc + 1) * u64::from(scl);
        if self.stretch_pending {
            self.stretch_pending = false;
            base + STRETCH_CYCLES
        } else {
            base
        }
    }

    /// Latch a new-generation `CR2` transfer on a `START` edge. Field
    /// snapshot waits for start completion: firmware writes `CR2` in one
    /// word, but the `START` edge (lane 1) lands before `NBYTES`
    /// (lanes 2–3) under byte-lane dispatch.
    fn ng_begin(&mut self, view: &mut Registers<&mut [u32]>) {
        view.set(NG_ISR, view.get(NG_ISR) & !NG_TC);
        self.begin_start();
    }

    /// Advance the new-generation transfer state machine.
    fn ng_tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if self.remaining == 0 {
            return;
        }
        if self.remaining > delta {
            self.remaining -= delta;
            return;
        }
        self.remaining = 0;
        match self.phase {
            Phase::Start => {
                view.set(NG_ISR, view.get(NG_ISR) | NG_BUSY);
                // The word write completed while the start was on the
                // wire: snapshot the transfer fields now.
                let cr2 = view.get(CR2);
                self.ng_sadd = (cr2 & NG_SADD_MASK) as u16;
                self.ng_receiving = cr2 & NG_RD_WRN != 0;
                self.receiving = self.ng_receiving;
                self.ng_autoend = cr2 & NG_AUTOEND != 0;
                self.nbytes_left = ((cr2 >> NG_NBYTES_SHIFT) & 0xFF) as u8;
                self.phase = Phase::Addr;
                self.remaining = self.ng_byte_cycles(view);
            }
            Phase::Addr => {
                // `START` self-clears once the address is on the wire.
                view.set(CR2, view.get(CR2) & !NG_START);
                if self.slave_byte == 0 {
                    // No slave on the bus: NACK, release, abort.
                    view.set(NG_ISR, (view.get(NG_ISR) & !NG_BUSY) | NG_NACKF);
                    self.reset_transfer();
                } else if self.nbytes_left == 0 {
                    // Empty transfer: straight to transfer-complete.
                    view.set(NG_ISR, view.get(NG_ISR) | NG_TC);
                    self.phase = Phase::Data;
                } else {
                    let isr = view.get(NG_ISR)
                        | NG_ADDR
                        | ((u32::from(self.ng_sadd) & 0x7F) << NG_ADDCODE_SHIFT);
                    if self.ng_receiving {
                        view.set(NG_ISR, isr | NG_DIR);
                        self.phase = Phase::Data;
                        self.remaining = self.ng_byte_cycles(view);
                    } else {
                        view.set(NG_ISR, isr & !NG_DIR);
                        self.phase = Phase::Data;
                        view.set(NG_ISR, view.get(NG_ISR) | NG_TXIS);
                    }
                }
            }
            Phase::Data if self.ng_receiving => {
                // A byte from the slave landed; the next clocks after
                // firmware reads `RXDR`. The count completes with the
                // byte itself, so `TC` (and an `AUTOEND` stop) can
                // arrive while `RXNE` is still set, as on silicon.
                view.set(NG_RXDR, u32::from(self.slave_byte));
                self.nbytes_left = self.nbytes_left.saturating_sub(1);
                let mut isr = view.get(NG_ISR) | NG_RXNE;
                if self.nbytes_left == 0 {
                    isr |= NG_TC;
                }
                view.set(NG_ISR, isr);
                if self.nbytes_left == 0 && self.ng_autoend {
                    self.phase = Phase::Stop;
                    self.remaining = START_CYCLES;
                }
            }
            Phase::Data => {
                // A transmitted byte completed.
                if self.nbytes_left == 0 {
                    view.set(NG_ISR, view.get(NG_ISR) & !NG_TXIS);
                    view.set(NG_ISR, view.get(NG_ISR) | NG_TC);
                    if self.ng_autoend {
                        self.phase = Phase::Stop;
                        self.remaining = START_CYCLES;
                    }
                } else {
                    view.set(NG_ISR, view.get(NG_ISR) | NG_TXIS);
                }
            }
            Phase::Stop => {
                // The stop condition ends the transfer and releases the bus.
                view.set(CR2, view.get(CR2) & !(NG_START | NG_STOP));
                view.set(NG_ISR, (view.get(NG_ISR) & !NG_BUSY) | NG_STOPF);
                self.reset_transfer();
            }
            Phase::Idle => {}
        }
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
                if self.new_gen {
                    // No edges live here on this layout (`START`/`STOP`
                    // are `CR2` bits); clearing `PE` aborts the transfer.
                    let new = view.merged(CR1, val, shift);
                    if new & PE == 0 {
                        self.reset_transfer();
                        view.set(NG_ISR, view.get(NG_ISR) & !NG_DYNAMIC);
                    }
                    return new;
                }
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
                    // A start edge begins (or re-begins: repeated start is
                    // legal mid-transfer) when the bus is free or held by
                    // this master.
                    if start_edge
                        && new & PE != 0
                        && matches!(
                            self.phase,
                            Phase::Idle | Phase::Stop | Phase::Addr | Phase::Data
                        )
                    {
                        self.addr_second = false;
                        self.begin_start();
                    } else if stop_edge && matches!(self.phase, Phase::Addr | Phase::Data) {
                        self.phase = Phase::Stop;
                        self.addr_second = false;
                        self.remaining = BYTE_CYCLES;
                    }
                }
                new
            }
            DR => {
                if self.new_gen {
                    // `TIMINGR` on this layout: pure latch.
                    return view.merged(offset, val, shift);
                }
                let new = view.merged(DR, val, shift);
                // Only the low byte of the data register is the wire byte.
                if shift == 0 {
                    match self.phase {
                        Phase::Addr if self.addr_second => {
                            // Second byte of a 10-bit address: completes the
                            // address phase like a 7-bit byte does.
                            self.tx.push(val);
                            self.addr_second = false;
                            view.set(SR1, view.get(SR1) & !SB);
                            self.remaining = self.byte_cycles();
                        }
                        Phase::Addr => {
                            // The address byte (7-bit address plus read/write)
                            // goes out; SB was the event that led here. A
                            // `11110xx0` header opens a 10-bit address: the
                            // second byte is still expected.
                            self.tx.push(val);
                            self.receiving = val & 1 != 0;
                            view.set(SR1, view.get(SR1) & !SB);
                            if val & 0xF8 == 0xF0 && val & 1 == 0 {
                                self.addr_second = true;
                            }
                            self.remaining = self.byte_cycles();
                        }
                        Phase::Data if !self.receiving => {
                            self.tx.push(val);
                            view.set(SR1, view.get(SR1) & !(TXE | BTF));
                            self.remaining = self.byte_cycles();
                        }
                        _ => {}
                    }
                }
                new
            }
            SR1 if self.new_gen => {
                // `TIMEOUTR` on this layout: pure latch.
                view.merged(offset, val, shift)
            }
            SR1 => {
                let new = view.merged(SR1, val, shift);
                // Write-zero-to-clear for the software-clearable flags, leaving
                // the hardware-driven `TXE`/`RXNE` untouched.
                let clear = CLEARABLE & !new;
                view.set(SR1, view.get(SR1) & !clear);
                view.get(SR1)
            }
            CR2 if self.new_gen => {
                let old = view.get(CR2);
                let new = view.merged(CR2, val, shift);
                view.set(CR2, new);
                let start_edge = new & NG_START != 0 && old & NG_START == 0;
                let stop_edge = new & NG_STOP != 0 && old & NG_STOP == 0;
                if start_edge
                    && view.get(CR1) & PE != 0
                    && matches!(self.phase, Phase::Idle | Phase::Stop)
                {
                    self.ng_begin(view);
                } else if stop_edge && matches!(self.phase, Phase::Addr | Phase::Data) {
                    self.phase = Phase::Stop;
                    self.remaining = START_CYCLES;
                }
                new
            }
            NG_TXDR if self.new_gen => {
                // A staged transmit byte shifts out when `TXIS` is set;
                // only the low lane is the wire byte.
                if shift == 0
                    && view.get(NG_ISR) & NG_TXIS != 0
                    && matches!(self.phase, Phase::Data)
                    && !self.ng_receiving
                {
                    self.tx.push(val);
                    self.nbytes_left = self.nbytes_left.saturating_sub(1);
                    view.set(NG_ISR, view.get(NG_ISR) & !NG_TXIS);
                    self.remaining = self.ng_byte_cycles(view);
                }
                view.merged(offset, val, shift)
            }
            NG_ICR if self.new_gen => {
                // Write-1-to-clear for the acknowledged flags.
                let new = view.merged(NG_ICR, val, shift);
                let mut isr = view.get(NG_ISR);
                if new & NG_ADDRCF != 0 {
                    isr &= !NG_ADDR;
                }
                if new & NG_NACKCF != 0 {
                    isr &= !NG_NACKF;
                }
                if new & NG_STOPCF != 0 {
                    isr &= !NG_STOPF;
                }
                if new & NG_TCCF != 0 {
                    isr &= !NG_TC;
                }
                view.set(NG_ISR, isr);
                new
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            NG_ISR if self.new_gen => {
                // Status reads have no side effects on this layout.
                value
            }
            NG_RXDR if self.new_gen => {
                // Reading the receive register acknowledges `RXNE` and
                // clocks the next byte while the count is open.
                if view.get(NG_ISR) & NG_RXNE != 0 {
                    view.set(NG_ISR, view.get(NG_ISR) & !NG_RXNE);
                    if matches!(self.phase, Phase::Data)
                        && self.ng_receiving
                        && self.nbytes_left > 0
                    {
                        self.remaining = self.ng_byte_cycles(view);
                    }
                }
                value
            }
            SR2 => {
                // Reading SR1 and then SR2 acknowledges `ADDR`; a master
                // receiver then starts clocking the first data byte.
                if view.get(SR1) & ADDR != 0 {
                    view.set(SR1, view.get(SR1) & !ADDR);
                    if matches!(self.phase, Phase::Data) && self.receiving {
                        self.remaining = self.byte_cycles();
                    }
                }
                value
            }
            DR => {
                if self.new_gen {
                    // `TIMINGR` on this layout: no read side effects.
                    return value;
                }
                // Reading the data register acknowledges `RXNE` (and the
                // `BTF` that arrived with it); with `ACK` still set a master
                // receiver clocks the next byte.
                if view.get(SR1) & RXNE != 0 {
                    view.set(SR1, view.get(SR1) & !(RXNE | BTF));
                    if matches!(self.phase, Phase::Data)
                        && self.receiving
                        && view.get(CR1) & ACK != 0
                    {
                        self.remaining = self.byte_cycles();
                    }
                }
                value
            }
            _ => value,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if self.new_gen {
            self.ng_tick(view, delta);
            return;
        }
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
                // The address byte shifted out and was acknowledged. A
                // 10-bit header only opens the phase: the flag stays set
                // so the second byte takes the two-byte path, and nothing
                // completes yet.
                if self.addr_second {
                    self.remaining = 0;
                    return;
                }
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
                // A byte from the slave landed in the data register. (No
                // overrun path: the engine only clocks the next byte after
                // firmware reads `DR`, so `RXNE` is always clear here —
                // the peripheral stretches the clock in between, as on
                // silicon. `OVR` stays software-clearable for future DMA.)
                view.set(DR, u32::from(self.slave_byte));
                view.set(SR1, view.get(SR1) | (RXNE | BTF));
                self.remaining = 0;
            }
            Phase::Data => {
                // A transmitted byte completed: the buffer is empty again,
                // with `BTF` marking the finished byte.
                view.set(SR1, view.get(SR1) | (TXE | BTF));
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
        if self.new_gen {
            let cr1 = view.get(CR1);
            let isr = view.get(NG_ISR);
            let ev = (cr1 & NG_TXIE != 0 && isr & NG_TXIS != 0)
                || (cr1 & NG_RXIE != 0 && isr & NG_RXNE != 0)
                || (cr1 & NG_ADDRIE != 0 && isr & NG_ADDR != 0)
                || (cr1 & NG_NACKIE != 0 && isr & NG_NACKF != 0)
                || (cr1 & NG_STOPIE != 0 && isr & NG_STOPF != 0)
                || (cr1 & NG_TCIE != 0 && isr & NG_TC != 0);
            let er = cr1 & NG_ERRIE != 0;
            return u32::from(ev) | (u32::from(er) << 1);
        }
        let cr2 = view.get(CR2);
        let sr1 = view.get(SR1);
        let itevt = cr2 & ITEVTEN != 0;
        let event = itevt && sr1 & (SB | ADDR | BTF | STOPF) != 0;
        let buffer = itevt && cr2 & ITBUFEN != 0 && sr1 & (TXE | RXNE) != 0;
        let error = cr2 & ITERREN != 0 && sr1 & (BERR | ARLO | AF | OVR) != 0;
        u32::from(event || buffer) | (u32::from(error) << 1)
    }

    fn lines(&self) -> u32 {
        if self.new_gen {
            2
        } else {
            3
        }
    }

    fn reset(&mut self) {
        self.reset_transfer();
        self.tx.clear();
    }

    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        // The low byte is the slave's response; zero marks an absent slave.
        // Bit 8 stretches the in-flight (or next) byte: the slave holds the
        // clock, adding `STRETCH_CYCLES` to its transfer.
        self.slave_byte = (value & 0xFF) as u8;
        if value & (1 << 8) != 0 {
            if self.remaining > 0 {
                self.remaining += STRETCH_CYCLES;
            } else {
                self.stretch_pending = true;
            }
        }
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
        let new_gen = I2cBehavior::is_new_gen(def);
        Self {
            inner: RegisterFile::new(
                def,
                I2cBehavior {
                    new_gen,
                    ..Default::default()
                },
            ),
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
        let new_gen = I2cBehavior::is_new_gen(def);
        Self {
            inner: RegisterFile::new(
                def,
                I2cBehavior {
                    new_gen,
                    ..Default::default()
                },
            ),
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
        let new_gen = I2cBehavior::is_new_gen(def);
        Self {
            inner: RegisterFile::new(
                def,
                I2cBehavior {
                    new_gen,
                    ..Default::default()
                },
            ),
        }
    }

    /// A new `I2C4` on another chip's device tables.
    #[must_use]
    pub fn i2c4_for<D: Device>() -> Self {
        let def = D::peripheral("I2C4").expect("generated tables contain I2C4");
        let new_gen = I2cBehavior::is_new_gen(def);
        Self {
            inner: RegisterFile::new(
                def,
                I2cBehavior {
                    new_gen,
                    ..Default::default()
                },
            ),
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

    /// Repeated start mid-transfer reopens the address phase (EEPROM
    /// random-read shape: write address, Sr, read data).
    #[test]
    fn repeated_start_reopens_the_address_phase() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        send_addr(&mut i, 0xA0);
        let _ = read_word(&mut i, BASE + SR2);
        // Data phase is live; a fresh START re-begins addressing.
        write_word(&mut i, BASE + CR1, PE | START);
        i.tick(START_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & SB, SB, "Sr raised SB");
        send_addr(&mut i, 0xA1);
        assert_eq!(read_word(&mut i, BASE + SR1) & ADDR, ADDR);
        assert_eq!(read_word(&mut i, BASE + SR2) & TRA, 0, "now receiving");
    }

    /// A `11110xx0` header keeps the address phase open for the second
    /// byte; a `11110xx1` header completes at once.
    #[test]
    fn ten_bit_header_needs_two_bytes() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        // Header with R/W = 0: ADDR must wait for the second byte.
        write_word(&mut i, BASE + DR, 0xF0u32);
        i.tick(BYTE_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & ADDR, 0, "no ADDR yet");
        write_word(&mut i, BASE + DR, 0x55u32);
        i.tick(BYTE_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & ADDR, ADDR, "ADDR now");
        assert_eq!(i.drain_output(), [0xF0, 0x55]);
        // Header with R/W = 1 completes at once (Sr read form).
        let mut j = enabled();
        j.drive_inputs(0xA5);
        start(&mut j);
        write_word(&mut j, BASE + DR, 0xF1u32);
        j.tick(BYTE_CYCLES);
        assert_eq!(read_word(&mut j, BASE + SR1) & ADDR, ADDR);
        assert_eq!(j.drain_output(), [0xF1]);
    }

    /// `BTF` marks completions for polling firmware and clears on data
    /// access.
    #[test]
    fn btf_marks_completions_and_clears_on_access() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        send_addr(&mut i, 0xA0);
        let _ = read_word(&mut i, BASE + SR2);
        write_word(&mut i, BASE + DR, 0x11u32);
        i.tick(BYTE_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & super::BTF, super::BTF);
        // A fresh data write clears it again until completion.
        write_word(&mut i, BASE + DR, 0x22u32);
        assert_eq!(read_word(&mut i, BASE + SR1) & super::BTF, 0);
    }

    /// A stretched byte takes `BYTE_CYCLES + STRETCH_CYCLES` (cycle-counted
    /// through `tick`, the way firmware observes a slow slave).
    #[test]
    fn stretch_extends_byte_timing() {
        let mut i = enabled();
        i.drive_inputs(0xA5);
        start(&mut i);
        send_addr(&mut i, 0xA0);
        let _ = read_word(&mut i, BASE + SR2);
        // Arm the stretch after addressing so it lands on the data byte.
        i.drive_inputs(0xA5 | (1 << 8)); // slave holds the clock
        write_word(&mut i, BASE + DR, 0x11u32);
        i.tick(BYTE_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & TXE, 0, "still shifting");
        i.tick(super::STRETCH_CYCLES);
        assert_eq!(read_word(&mut i, BASE + SR1) & TXE, TXE, "done now");
    }

    // --- New-generation engine (F0 tables) ---

    use super::{
        NG_ADDR, NG_ADDRCF, NG_ADDRIE, NG_AUTOEND, NG_BUSY, NG_DIR, NG_NACKCF, NG_NACKF,
        NG_NBYTES_SHIFT, NG_RD_WRN, NG_RXDR, NG_RXNE, NG_SADD_MASK, NG_START, NG_STOP, NG_STOPCF,
        NG_STOPF, NG_TC, NG_TCCF, NG_TXDR, NG_TXIS,
    };
    use crate::device::svd::stm32f030::Stm32f030;

    const NG_BASE: u32 = 0x4000_5400;
    const NG_CR2: u32 = 0x04;
    const NG_TIMINGR: u32 = 0x10;
    const NG_ISR: u32 = 0x18;
    const NG_ICR: u32 = 0x1C;

    fn ng_enabled() -> I2c {
        let mut i = I2c::for_device::<Stm32f030>();
        assert!(i.inner.behavior().new_gen, "F0 tables select new-gen");
        write_word(&mut i, NG_BASE + CR1, PE);
        i
    }

    /// `CR2` word for a transfer: 7-bit `sadd`, direction, byte count.
    fn ng_cr2(sadd: u32, read: bool, nbytes: u32, autoend: bool) -> u32 {
        (sadd & NG_SADD_MASK)
            | (u32::from(read) * NG_RD_WRN)
            | (nbytes << NG_NBYTES_SHIFT)
            | (u32::from(autoend) * NG_AUTOEND)
            | NG_START
    }

    /// Run the start condition, then one address byte time.
    fn ng_started(i: &mut I2c) {
        i.tick(8);
        i.tick(18);
    }

    #[test]
    fn ng_nacks_without_slave() {
        let mut i = ng_enabled();
        // No staged byte: nobody answers the address.
        write_word(&mut i, NG_BASE + NG_CR2, ng_cr2(0x50, false, 1, false));
        ng_started(&mut i);
        let isr = read_word(&mut i, NG_BASE + NG_ISR);
        assert_ne!(isr & NG_NACKF, 0, "NACKF latches");
        assert_eq!(isr & NG_BUSY, 0, "bus released");
        // Write-1-to-clear through ICR.
        write_word(&mut i, NG_BASE + NG_ICR, NG_NACKCF);
        assert_eq!(read_word(&mut i, NG_BASE + NG_ISR) & NG_NACKF, 0);
    }

    #[test]
    fn ng_tx_to_staged_slave() {
        let mut i = ng_enabled();
        i.drive_inputs(0x01); // slave present
        write_word(&mut i, NG_BASE + NG_CR2, ng_cr2(0x50, false, 2, false));
        ng_started(&mut i);
        let isr = read_word(&mut i, NG_BASE + NG_ISR);
        assert_ne!(isr & NG_ADDR, 0, "ADDR latches");
        assert_ne!(isr & NG_TXIS, 0, "TXIS invites the first byte");
        assert_eq!(isr & NG_DIR, 0, "transmit direction");
        assert_eq!((isr >> 17) & 0x7F, 0x50, "ADDCODE matches");
        // ADDRCF clears ADDR; two bytes shift out, then TC.
        write_word(&mut i, NG_BASE + NG_ICR, NG_ADDRCF);
        assert_eq!(read_word(&mut i, NG_BASE + NG_ISR) & NG_ADDR, 0);
        write_word(&mut i, NG_BASE + NG_TXDR, 0xAA);
        i.tick(18);
        assert_ne!(read_word(&mut i, NG_BASE + NG_ISR) & NG_TXIS, 0);
        write_word(&mut i, NG_BASE + NG_TXDR, 0xBB);
        i.tick(18);
        let isr = read_word(&mut i, NG_BASE + NG_ISR);
        assert_ne!(isr & NG_TC, 0, "TC after the count");
        assert_eq!(i.drain_output(), [0xAA, 0xBB], "bytes drained");
        // TCCF clears TC; STOP releases with STOPF.
        write_word(&mut i, NG_BASE + NG_ICR, NG_TCCF);
        assert_eq!(read_word(&mut i, NG_BASE + NG_ISR) & NG_TC, 0);
        write_word(&mut i, NG_BASE + NG_CR2, NG_STOP);
        i.tick(8);
        let isr = read_word(&mut i, NG_BASE + NG_ISR);
        assert_ne!(isr & NG_STOPF, 0, "STOPF latches");
        assert_eq!(isr & NG_BUSY, 0, "bus released");
        write_word(&mut i, NG_BASE + NG_ICR, NG_STOPCF);
        assert_eq!(read_word(&mut i, NG_BASE + NG_ISR) & NG_STOPF, 0);
    }

    #[test]
    fn ng_rx_from_slave_with_autoend() {
        let mut i = ng_enabled();
        i.drive_inputs(0x5A);
        write_word(&mut i, NG_BASE + NG_CR2, ng_cr2(0x50, true, 2, true));
        ng_started(&mut i);
        let isr = read_word(&mut i, NG_BASE + NG_ISR);
        assert_ne!(isr & NG_ADDR, 0, "ADDR latches");
        assert_ne!(isr & NG_DIR, 0, "receive direction");
        i.tick(18);
        assert_eq!(
            read_word(&mut i, NG_BASE + NG_RXDR) & 0xFF,
            0x5A,
            "first byte lands"
        );
        // The read cleared RXNE and clocked byte two.
        assert_eq!(read_word(&mut i, NG_BASE + NG_ISR) & NG_RXNE, 0);
        i.tick(18);
        assert_eq!(
            read_word(&mut i, NG_BASE + NG_RXDR) & 0xFF,
            0x5A,
            "second byte"
        );
        // The count completed with the byte: TC set, AUTOEND stop sent.
        let isr = read_word(&mut i, NG_BASE + NG_ISR);
        assert_ne!(isr & NG_TC, 0, "TC with the last byte");
        i.tick(8);
        let isr = read_word(&mut i, NG_BASE + NG_ISR);
        assert_ne!(isr & NG_STOPF, 0, "AUTOEND stop lands");
    }

    #[test]
    fn ng_timingr_paces_bytes() {
        let mut i = ng_enabled();
        // PRESC=15, SCLL=SCLH=0: 9 * 16 * 2 = 288 ticks per byte.
        write_word(&mut i, NG_BASE + NG_TIMINGR, 0xF000_0000);
        assert_eq!(
            read_word(&mut i, NG_BASE + NG_TIMINGR),
            0xF000_0000,
            "TIMINGR latches"
        );
        write_word(&mut i, NG_BASE + NG_CR2, ng_cr2(0x50, false, 1, false));
        i.tick(8);
        i.tick(100);
        assert_eq!(
            read_word(&mut i, NG_BASE + NG_ISR) & (NG_NACKF | NG_ADDR),
            0,
            "address still shifting"
        );
        i.tick(200);
        assert_ne!(
            read_word(&mut i, NG_BASE + NG_ISR) & NG_NACKF,
            0,
            "NACKF after the slow byte"
        );
    }

    #[test]
    fn ng_irq_lines_mask() {
        let mut i = ng_enabled();
        // ADDRIE only: ADDR pends line 0, nothing else does.
        write_word(&mut i, NG_BASE + CR1, PE | NG_ADDRIE);
        i.drive_inputs(0x01);
        write_word(&mut i, NG_BASE + NG_CR2, ng_cr2(0x50, false, 1, false));
        ng_started(&mut i);
        assert_eq!(i.pending(), 1, "ADDR raises line 0");
        write_word(&mut i, NG_BASE + NG_ICR, NG_ADDRCF);
        assert_eq!(i.pending(), 0, "clear drops the line");
    }
}
