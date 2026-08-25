//! The STM32 SPI/I2S peripheral (`SPI1`/`SPI2`/`SPI3`), driven by the
//! SVD-generated register tables.
//!
//! Register layout and resets come from the SVD; the table cannot express the
//! shift register, so [`SpiBehavior`] runs it: with the SPI enabled
//! (`CR1.SPE`) and configured as master (`CR1.MSTR`), writing the data
//! register begins a frame — `CR1.TXE` drops, `SR.BSY` rises, and after a
//! [`SpiBehavior::transfer_cycles`] the byte a connected slave drove on `MISO`
//! lands in `DR` with `SR.RXNE` and `SR.TXE` restored. Reading `DR` clears
//! `RXNE`, exactly as on silicon. `CR2.TXEIE`/`CR2.RXNEIE` raise the SPI
//! interrupt line while `TXE`/`RXNE` are set and the peripheral is enabled.
//! The master's `MOSI` byte is captured for the host through
//! [`Peripheral::drain_output`]; the slave's `MISO` byte is injected through
//! [`Peripheral::drive_inputs`].
//!
//! `CR1.DFF` selects the 8-bit (`DR` byte lane 0) or 16-bit (lane 1) frame the
//! firmware completes with its final data-register write; `CR1.BR` scales the
//! transfer latency like the baud-rate prescaler it is.

use crate::device::stm32::dma::DmaRequests;
use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// Register offsets within the SPI, per the SVD.
const CR1: u32 = 0x00;
const CR2: u32 = 0x04;
const SR: u32 = 0x08;
const DR: u32 = 0x0C;

/// `CR1` bits.
const SPE: u32 = 1 << 6;
const MSTR: u32 = 1 << 2;
const DFF: u32 = 1 << 11;

/// `CR2` bits.
const TXEIE: u32 = 1 << 7;
const RXNEIE: u32 = 1 << 6;
const RXDMAEN: u32 = 1 << 0;
const TXDMAEN: u32 = 1 << 1;

/// `SR` bits.
const RXNE: u32 = 1 << 0;
const TXE: u32 = 1 << 1;
const BSY: u32 = 1 << 7;

/// An in-flight master frame shifting out of the `MOSI` pin.
#[derive(Clone, Copy, Debug)]
struct SpiTransfer {
    tx_byte: u8,
    remaining: u64,
}

/// The shift register the SVD cannot express: a master transfer that shifts a
/// byte out while sampling one from the slave.
#[derive(Debug)]
struct SpiBehavior {
    /// The byte a connected slave will present on the next frame.
    rx_byte: u8,
    /// An in-progress master frame, if the shift register is busy.
    transfer: Option<SpiTransfer>,
    /// Bytes the master shifted out, for host draining.
    mosi: Vec<u8>,
    dma_tx: Option<(Arc<Mutex<DmaRequests>>, u8, u8)>,
    dma_rx: Option<(Arc<Mutex<DmaRequests>>, u8, u8)>,
}

impl Default for SpiBehavior {
    fn default() -> Self {
        Self {
            rx_byte: 0,
            transfer: None,
            mosi: Vec::new(),
            dma_tx: None,
            dma_rx: None,
        }
    }
}

impl SpiBehavior {
    /// The `DR` byte lane whose write completes the frame and starts the shift
    /// (lane 0 for the 8-bit `DFF` mode, lane 1 for 16-bit).
    const fn frame_lane(cr1: u32) -> u32 {
        if cr1 & DFF != 0 {
            8
        } else {
            0
        }
    }

    /// Machine cycles an 8-bit master frame takes at `CR1.BR`: the prescaler
    /// (`/2`, `/4`, ... `/256`) sets the bit period, eight bits per frame.
    const fn transfer_cycles(cr1: u32) -> u64 {
        8 * (2u64 << ((cr1 >> 3) & 0x7))
    }
}

impl Behavior for SpiBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == DR {
            let cr1 = view.get(CR1);
            let word = view.merged(DR, val, shift);
            // A frame starts only in master mode with the peripheral enabled
            // and the transmit buffer empty; writes while busy are dropped.
            let ready = view.get(SR) & TXE != 0 && cr1 & (SPE | MSTR) == (SPE | MSTR);
            if ready && shift == Self::frame_lane(cr1) {
                view.set(SR, (view.get(SR) & !TXE) | BSY);
                self.transfer = Some(SpiTransfer {
                    tx_byte: (word & 0xFF) as u8,
                    remaining: Self::transfer_cycles(cr1),
                });
            }
            return word;
        }
        view.merged(offset, val, shift)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == DR {
            // Reading the data register acknowledges `RXNE`.
            view.set(SR, view.get(SR) & !RXNE);
        }
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        let Some(mut transfer) = self.transfer else {
            return;
        };
        if transfer.remaining > delta {
            transfer.remaining -= delta;
            self.transfer = Some(transfer);
            return;
        }
        // The frame completes: the sampled MISO byte lands in DR.
        self.transfer = None;
        view.set(DR, u32::from(self.rx_byte));
        view.set(SR, (view.get(SR) & !BSY) | RXNE | TXE);
        self.mosi.push(transfer.tx_byte);
        // DMA requests for TXE/RXNE
        let cr2 = view.get(CR2);
        if cr2 & TXDMAEN != 0 {
            if let Some((reqs, ctrl, ch)) = self.dma_tx.as_ref() {
                let reqs = Arc::clone(reqs);
                reqs.lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .request(*ctrl, *ch, true);
            }
        }
        if cr2 & RXDMAEN != 0 {
            if let Some((reqs, ctrl, ch)) = self.dma_rx.as_ref() {
                let reqs = Arc::clone(reqs);
                reqs.lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .request(*ctrl, *ch, false);
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(SR);
        let cr2 = view.get(CR2);
        u32::from(
            view.get(CR1) & SPE != 0
                && ((sr & TXE != 0 && cr2 & TXEIE != 0) || (sr & RXNE != 0 && cr2 & RXNEIE != 0)),
        )
    }

    fn reset(&mut self) {
        self.transfer = None;
        self.mosi.clear();
        self.rx_byte = 0;
    }

    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        // The low byte is the slave's response on the next frame.
        self.rx_byte = (value & 0xFF) as u8;
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.mosi)
    }
}

/// An STM32 SPI/I2S peripheral.
#[derive(Debug)]
pub struct Spi {
    inner: RegisterFile<SpiBehavior>,
    name: &'static str,
}

impl Spi {
    /// The F103 `SPI1` at `0x4001_3000`: IRQ 35.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `SPI1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn spi1() -> Self {
        Self::spi1_for::<stm32f103::Stm32f103>()
    }

    /// An `SPI1` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `SPI1` peripheral.
    #[must_use]
    pub fn spi1_for<D: Device>() -> Self {
        Self::for_device::<D>("SPI1", "spi1")
    }

    /// The F103 `SPI2` at `0x4000_3800`: IRQ 36.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `SPI2`
    /// peripheral.
    #[must_use]
    pub fn spi2() -> Self {
        Self::spi2_for::<stm32f103::Stm32f103>()
    }

    /// An `SPI2` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `SPI2` peripheral.
    #[must_use]
    pub fn spi2_for<D: Device>() -> Self {
        Self::for_device::<D>("SPI2", "spi2")
    }

    /// The F103 `SPI3` at `0x4000_3C00`: IRQ 51.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `SPI3`
    /// peripheral.
    #[must_use]
    pub fn spi3() -> Self {
        Self::spi3_for::<stm32f103::Stm32f103>()
    }

    /// An `SPI3` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `SPI3` peripheral.
    #[must_use]
    pub fn spi3_for<D: Device>() -> Self {
        Self::for_device::<D>("SPI3", "spi3")
    }

    fn for_device<D: Device>(peripheral: &str, name: &'static str) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(def, SpiBehavior::default()),
            name,
        }
    }

    /// Attach DMA TX/RX lines.
    #[must_use]
    pub fn with_dma(
        mut self,
        tx: Option<(Arc<Mutex<DmaRequests>>, u8, u8)>,
        rx: Option<(Arc<Mutex<DmaRequests>>, u8, u8)>,
    ) -> Self {
        self.inner.behavior_mut().dma_tx = tx;
        self.inner.behavior_mut().dma_rx = rx;
        self
    }
}

impl Peripheral for Spi {
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

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }

    fn name(&self) -> &'static str {
        self.name
    }
}

impl InterruptSource for Spi {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32f407::Stm32f407;

    fn read_word(spi: &mut Spi, off: u32) -> u32 {
        let base = spi.ranges()[0].base;
        let mut word = 0;
        for i in 0..4 {
            let byte = spi.read(base + off + i).unwrap();
            word |= u32::from(byte) << (8 * i);
        }
        word
    }

    fn write_word(spi: &mut Spi, off: u32, val: u32) {
        let base = spi.ranges()[0].base;
        for i in 0..4 {
            spi.write(base + off + i, ((val >> (8 * i)) & 0xFF) as u8)
                .unwrap();
        }
    }

    fn enabled_master() -> Spi {
        let mut spi = Spi::spi1();
        write_word(&mut spi, CR1, SPE | MSTR);
        spi
    }

    #[test]
    fn txe_is_set_on_reset_and_cleared_by_a_frame() {
        let mut spi = Spi::spi1();
        assert_eq!(read_word(&mut spi, SR) & TXE, TXE);
        let mut spi = enabled_master();
        write_word(&mut spi, DR, 0x55);
        let sr = read_word(&mut spi, SR);
        assert_eq!(sr & TXE, 0);
        assert!(sr & BSY != 0);
    }

    #[test]
    fn a_frame_exchanges_bytes_with_the_slave() {
        let mut spi = enabled_master();
        spi.drive_inputs(0xAB);
        write_word(&mut spi, DR, 0x55);
        assert_eq!(read_word(&mut spi, SR) & RXNE, 0); // not done yet
        spi.tick(16);
        let sr = read_word(&mut spi, SR);
        assert!(sr & RXNE != 0 && sr & TXE != 0 && sr & BSY == 0);
        assert_eq!(read_word(&mut spi, DR), 0xAB);
        // Reading the result clears RXNE.
        assert_eq!(read_word(&mut spi, SR) & RXNE, 0);
        assert_eq!(spi.drain_output(), [0x55]);
    }

    #[test]
    fn rx_interrupt_is_level_triggered_and_masked() {
        let mut spi = Spi::spi1();
        spi.drive_inputs(0xAB);
        write_word(&mut spi, CR1, SPE | MSTR);
        write_word(&mut spi, DR, 0x55);
        spi.tick(16);
        // RXNE is set but no RXNEIE: the line stays down.
        assert_eq!(spi.pending(), 0);
        write_word(&mut spi, CR2, RXNEIE);
        assert_eq!(spi.pending(), 1);
        // Level-triggered: acknowledging does not release the line.
        spi.acknowledge(1);
        assert_eq!(spi.pending(), 1);
        // Reading the result does.
        assert_eq!(read_word(&mut spi, DR), 0xAB);
        assert_eq!(spi.pending(), 0);
    }

    #[test]
    fn tx_interrupt_follows_txe() {
        let mut spi = Spi::spi1();
        write_word(&mut spi, CR2, TXEIE);
        // Disabled at reset: the pending TXE does not assert the line.
        assert_eq!(spi.pending(), 0);
        write_word(&mut spi, CR1, SPE | MSTR);
        // Enabled with the transmit buffer empty: the line asserts.
        assert_eq!(spi.pending(), 1);
        // A write empties the buffer: the line drops while busy.
        write_word(&mut spi, DR, 0x55);
        assert_eq!(spi.pending(), 0);
        // The frame completes: TXE returns and the line asserts again.
        spi.tick(16);
        assert_eq!(spi.pending(), 1);
    }

    #[test]
    fn frames_are_gated_by_spe_and_master_mode() {
        let mut spi = Spi::spi1();
        spi.drive_inputs(0xAB);
        // Slave mode (no MSTR): the write only loads the shift register.
        write_word(&mut spi, CR1, SPE);
        write_word(&mut spi, DR, 0x55);
        spi.tick(16);
        assert_eq!(read_word(&mut spi, SR) & RXNE, 0);
        // TXE was never consumed: the lane is still ready.
        assert_eq!(read_word(&mut spi, SR) & TXE, TXE);
        // Disabled: the write is dropped too.
        write_word(&mut spi, CR1, MSTR);
        write_word(&mut spi, DR, 0x55);
        spi.tick(16);
        assert_eq!(read_word(&mut spi, SR) & RXNE, 0);
    }

    #[test]
    fn writes_while_busy_are_ignored() {
        let mut spi = enabled_master();
        spi.drive_inputs(0xAB);
        write_word(&mut spi, DR, 0x55);
        // A second write while TXE is clear does not start another frame.
        write_word(&mut spi, DR, 0x66);
        spi.tick(16);
        assert_eq!(read_word(&mut spi, DR), 0xAB);
        assert_eq!(spi.drain_output(), [0x55]);
    }

    #[test]
    fn reset_clears_status_and_captured_output() {
        let mut spi = enabled_master();
        spi.drive_inputs(0xAB);
        write_word(&mut spi, DR, 0x55);
        spi.tick(16);
        assert!(read_word(&mut spi, SR) & RXNE != 0);
        spi.reset();
        assert_eq!(read_word(&mut spi, SR) & RXNE, 0);
        assert_eq!(read_word(&mut spi, SR) & TXE, TXE); // SVD reset value
        assert_eq!(spi.drain_output(), []);
    }

    #[test]
    fn transfer_time_scales_with_the_baud_prescaler() {
        let mut spi = Spi::spi1();
        spi.drive_inputs(0xAB);
        // BR[2:0] = 1 (PCLK/4): a frame takes 32 cycles, not 16.
        write_word(&mut spi, CR1, SPE | MSTR | (1 << 3));
        write_word(&mut spi, DR, 0x55);
        spi.tick(16);
        assert_eq!(read_word(&mut spi, SR) & RXNE, 0);
        spi.tick(16);
        assert!(read_word(&mut spi, SR) & RXNE != 0);
    }

    #[test]
    fn tables_mount_the_same_register_file() {
        let mut spi = Spi::spi1_for::<Stm32f407>();
        assert_eq!(spi.ranges()[0].base, 0x4001_3000);
        assert_eq!(spi.exception_number(), Some(51)); // IRQ 35 -> exception 51
        spi.drive_inputs(0xAB);
        write_word(&mut spi, CR1, SPE | MSTR);
        write_word(&mut spi, DR, 0x55);
        spi.tick(16);
        assert_eq!(read_word(&mut spi, DR), 0xAB);
        assert_eq!(spi.drain_output(), [0x55]);
    }
}
