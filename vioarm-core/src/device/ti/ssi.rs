//! TI Tiva C synchronous serial interfaces (`SSI0`-`SSI3`).
//!
//! Master-mode Freescale SPI follows the TM4C123GH6PM datasheet
//! (SPMS376E §15): 8-deep FIFOs, `SR` synthesis (`TFE`/`TNF`/`RNE`/`RFF`/
//! `BSY`), `CR0` clock math (`SysClk / (CPSDVSR × (1+SCR))`, `CPSDVSR`
//! even-enforced), `DSS` data widths 4–16 bit (reserved `0x0-0x2` latch but
//! read as 16-bit width, p970), `TXRIS` at half-empty-or-less
//! (`EOT` empties), `RXRIS` at half-full-or-more, `IM`/`MIS` with `ICR`
//! W1C (`RTIC`/`RORIC`), overrun `RORRIS`, `SSICC` system/`PIOSC` select,
//! `CPSR[31:8]` `RO` with `CPSDVSR` LSB hard-0 (p976), `DR` 16-bit
//! right-justified (p973).
//!
//! Like the repo's SPI pairs and STM32 USART, TX completes into the host
//! mailbox instantly — no bit timing is modeled. `BSY` is a 1-tick shifter
//! hold: set on a `DR` write, cleared on the next `tick` once the FIFO is
//! empty and the shifter is done (p974: `1` while Tx/Rx *or* TX FIFO
//! non-empty). RX bytes arrive through [`Ssi::inject_rx`]
//! (host/test stimulus); overflow sets sticky `RORRIS`. `LBM` loopback
//! routes TX into RX in order. Slave mode (`MS=1`), TI/Microwire formats,
//! and the receive-timeout interrupt stay latched/unfired (documented).
//! Firmware always word-stores `DR` (like TivaWare); byte/halfword stores
//! assemble lanes but only a full-word store pushes a frame, and sub-word
//! data must be right-justified (p973). `PeriphID`/`CellID` read as Table
//! 15-2 p967 even though the SVD omits them (custom bus shim).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;

const CR0_OFF: u32 = 0x000;
const CR1_OFF: u32 = 0x004;
const DR_OFF: u32 = 0x008;
const SR_OFF: u32 = 0x00C;
const CPSR_OFF: u32 = 0x010;
const IM_OFF: u32 = 0x014;
const RIS_OFF: u32 = 0x018;
const MIS_OFF: u32 = 0x01C;
const ICR_OFF: u32 = 0x020;
const CC_OFF: u32 = 0xFC8;

/// `SR` bits: `BSY` 4, `RFF` 3, `RNE` 2, `TNF` 1, `TFE` 0.
const SR_BSY: u32 = 1 << 4;
const SR_RFF: u32 = 1 << 3;
const SR_RNE: u32 = 1 << 2;
const SR_TNF: u32 = 1 << 1;
const SR_TFE: u32 = 1 << 0;
/// `CR1` bits: `EOT` 4, `MS` 2, `SSE` 1, `LBM` 0.
const CR1_EOT: u32 = 1 << 4;
const CR1_MS: u32 = 1 << 2;
const CR1_SSE: u32 = 1 << 1;
const CR1_LBM: u32 = 1 << 0;
/// `CR0` fields: `SCR` 15:8, `FRF` 5:4, `DSS` 3:0.
const CR0_SCR_SHIFT: u32 = 8;
const CR0_FRF_SHIFT: u32 = 4;
const CR0_FRF_MASK: u32 = 0x03;
const CR0_DSS_MASK: u32 = 0x0F;
/// `RIS`/`IM`/`ICR` bits: `TX` 3, `RX` 2, `RT` 1, `ROR` 0.
const RIS_TX: u32 = 1 << 3;
const RIS_RX: u32 = 1 << 2;
const RIS_ROR: u32 = 1 << 0;
const ICR_RTIC: u32 = 1 << 1;
const ICR_RORIC: u32 = 1 << 0;
/// `CC.CS`: `0x0` system clock, `0x5` PIOSC.
const CC_PIOSC: u32 = 0x05;
/// FIFO depth per direction (half-full 4 of 8).
const FIFO_DEPTH: usize = 8;
const FIFO_HALF: usize = 4;

/// Nonzero datasheet resets stamped over the SVD zeros.
/// Table 15-2 p967: `SR=0x03`, `RIS=0x08`; `PeriphID`/`CellID` are handled
/// by the bus shim below (SVD omits them).
const RESETS: &[(u32, u32)] = &[(SR_OFF, 0x0000_0003), (RIS_OFF, 0x0000_0008)];

/// Identification words missing from the SVD (Table 15-2 p967, all `RO`).
/// `PeriphID4-7` read 0.
const ID_WORDS: &[(u32, u32)] = &[
    (0xFE0, 0x0000_0022), // PeriphID0
    (0xFE4, 0x0000_0000), // PeriphID1
    (0xFE8, 0x0000_0018), // PeriphID2
    (0xFEC, 0x0000_0001), // PeriphID3
    (0xFF0, 0x0000_000D), // CellID0
    (0xFF4, 0x0000_00F0), // CellID1
    (0xFF8, 0x0000_0005), // CellID2
    (0xFFC, 0x0000_00B1), // CellID3
];

/// SSI bit rate in Hz: `SysClk / (CPSDVSR × (1 + SCR))`, 0 when unprogrammed.
#[must_use]
pub fn bitrate_hz_for(cpsdvsr: u32, scr: u32, sysclk_hz: u32) -> u32 {
    let cps = cpsdvsr & 0xFE;
    if cps < 2 || sysclk_hz == 0 {
        return 0;
    }
    u32::try_from(u64::from(sysclk_hz) / (u64::from(cps) * (u64::from(scr) + 1)))
        .unwrap_or(u32::MAX)
}

/// Behavior behind one SSI window (master Freescale SPI).
#[derive(Debug, Default)]
pub struct SsiBehavior {
    tx: VecDeque<u16>,
    rx: VecDeque<u16>,
    popped: u16,
    /// Frames firmware transmitted, for the host to drain.
    tx_out: Vec<u16>,
    /// Sticky receive-overrun flag.
    ror: bool,
    /// `BSY` shifter hold: set on a `DR` write, cleared on the next `tick`
    /// once the FIFO is empty and the shifter is done (p974). Instant TX
    /// drain keeps `tx` empty, so `SR.BSY` reads this hold.
    busy: bool,
    /// Last popped frame (lane-1 `DR` reads fetch its high byte).
    /// Cached `(RIS & IM) != 0` (`InterruptSource::pending` is `&self`).
    pending: bool,
}

impl SsiBehavior {
    fn enabled(&self, view: &Registers<&mut [u32]>) -> bool {
        view.get(CR1_OFF) & CR1_SSE != 0
    }

    fn master(&self, view: &Registers<&mut [u32]>) -> bool {
        view.get(CR1_OFF) & CR1_MS == 0
    }

    /// Data width in bits from `DSS` (4–16); reserved `0x0-0x2` latch but
    /// are treated as 16-bit width (p970, documented approximation).
    fn width(&self, view: &Registers<&mut [u32]>) -> u32 {
        let dss = view.get(CR0_OFF) & CR0_DSS_MASK;
        if (0x03..=0x0F).contains(&dss) {
            dss + 1
        } else {
            16
        }
    }

    fn mask(&self, view: &Registers<&mut [u32]>) -> u16 {
        let w = self.width(view);
        if w >= 16 {
            0xFFFF
        } else {
            (1u16 << w) - 1
        }
    }

    fn ris(&self, view: &Registers<&mut [u32]>) -> u32 {
        let mut ris = 0u32;
        let eot = view.get(CR1_OFF) & CR1_EOT != 0;
        if (eot && self.tx.is_empty()) || (!eot && self.tx.len() <= FIFO_HALF) {
            ris |= RIS_TX;
        }
        if self.rx.len() >= FIFO_HALF {
            ris |= RIS_RX;
        }
        if self.ror {
            ris |= RIS_ROR;
        }
        ris
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        self.pending = self.ris(view) & view.get(IM_OFF) != 0;
    }
}

impl Behavior for SsiBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn tick(&mut self, _view: &mut Registers<&mut [u32]>, _delta: u64) {
        // 1-tick shifter hold: the instant-drained FIFO is empty, so clear
        // the `BSY` latch on the next tick (p974).
        self.busy = false;
    }

    fn reset(&mut self) {
        *self = Self::default();
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            SR_OFF => {
                let mut sr = 0u32;
                if self.rx.len() >= FIFO_DEPTH {
                    sr |= SR_RFF;
                }
                if !self.rx.is_empty() {
                    sr |= SR_RNE;
                }
                if self.tx.len() < FIFO_DEPTH {
                    sr |= SR_TNF;
                }
                if self.tx.is_empty() {
                    sr |= SR_TFE;
                }
                // p974: `1` while Tx/Rx *or* TX FIFO non-empty (shifter hold
                // after FIFO empty). Instant drain keeps `tx` empty, so the
                // 1-tick `busy` hold is the shifter.
                if self.busy || !self.tx.is_empty() {
                    sr |= SR_BSY;
                }
                sr
            }
            RIS_OFF => self.ris(view),
            MIS_OFF => self.ris(view) & view.get(IM_OFF),
            DR_OFF => value,
            _ => value,
        };
        self.refresh(view);
        out
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            DR_OFF => {
                // `DR` is 16-bit right-justified (p973): firmware must
                // right-justify sub-16-bit data, the transmit logic ignores
                // unused top bits, and receive data is auto-right-justified.
                // Lane-assembled 16-bit frame; only full-word stores push
                // (firmware always word-stores `DR`, like TivaWare, so
                // byte/halfword stores assemble but never push — documented).
                let cur = view.get(offset);
                let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
                view.set(offset, word);
                if shift == 8 && self.enabled(view) && self.master(view) {
                    let frame = (word & 0xFFFF) as u16 & self.mask(view);
                    if self.tx.len() < FIFO_DEPTH {
                        self.tx.push_back(frame);
                    }
                    // Shifter hold for `BSY` (cleared on next `tick`).
                    self.busy = true;
                    // Instant completion into the host mailbox (no bit
                    // timing); loopback routes TX into RX in order.
                    while let Some(f) = self.tx.pop_front() {
                        self.tx_out.push(f);
                        if view.get(CR1_OFF) & CR1_LBM != 0 && self.rx.len() < FIFO_DEPTH {
                            self.rx.push_back(f);
                        }
                    }
                }
                word
            }
            SR_OFF | RIS_OFF | MIS_OFF => view.get(offset),
            CPSR_OFF => {
                // p976: `31:8` `RO` 0, only `7:0 CPSDVSR`, LSB hard-0
                // (reads back even). Upper-lane writes sink.
                if shift != 0 {
                    view.get(offset)
                } else {
                    let lo = val & 0xFE;
                    view.set(offset, u32::from(lo));
                    u32::from(lo)
                }
            }
            ICR_OFF => {
                let word = view.merged(offset, val, shift);
                if word & (ICR_RTIC | ICR_RORIC) & ICR_RORIC != 0 {
                    self.ror = false;
                }
                self.refresh(view);
                view.get(offset)
            }
            _ => {
                let word = view.merged(offset, val, shift);
                view.set(offset, word);
                self.refresh(view);
                word
            }
        }
    }
}

/// A Tiva C SSI (master Freescale SPI).
pub struct Ssi {
    inner: RegisterFile<SsiBehavior>,
}

impl Ssi {
    /// A new SSI on `D`'s tables (`"SSI0"`…`"SSI3"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain SSI");
        let mut ssi = Self {
            inner: RegisterFile::new(def, SsiBehavior::default()),
        };
        ssi.stamp_resets();
        ssi
    }

    /// Stamp the nonzero datasheet resets over the SVD zeros.
    fn stamp_resets(&mut self) {
        let def = self.inner.def();
        let stamped: Vec<(usize, u32)> = RESETS
            .iter()
            .filter_map(|&(offset, value)| {
                def.registers
                    .iter()
                    .position(|r| r.offset == offset)
                    .map(|idx| (idx, value))
            })
            .collect();
        let words = self.inner.regs_mut();
        for (idx, value) in stamped {
            words[idx] = value;
        }
    }

    /// Bit rate in Hz over an explicit system-clock rate.
    #[must_use]
    pub fn bitrate_hz_over(&mut self, sysclk_hz: u32) -> u32 {
        self.inner.with_view(|view, _| {
            let cps = view.get(CPSR_OFF) & 0xFF;
            let scr = (view.get(CR0_OFF) >> CR0_SCR_SHIFT) & 0xFF;
            bitrate_hz_for(cps, scr, sysclk_hz)
        })
    }

    /// Frames firmware transmitted, for the host to drain.
    pub fn drain_output(&mut self) -> Vec<u16> {
        self.inner.with_view(|_, b| std::mem::take(&mut b.tx_out))
    }

    /// Feed received frames (host/test stimulus). Overflow sets `RORRIS`.
    pub fn inject_rx(&mut self, frames: &[u16]) {
        self.inner.with_view(|view, b| {
            let mask = b.mask(view);
            for &frame in frames {
                if b.rx.len() < FIFO_DEPTH {
                    b.rx.push_back(frame & mask);
                } else {
                    b.ror = true;
                }
            }
            b.refresh(view);
        });
    }

    /// Pop one received frame (test hook; firmware reads `DR`).
    pub fn pop_rx(&mut self) -> Option<u16> {
        self.inner.with_view(|view, b| {
            let frame = b.rx.pop_front();
            b.refresh(view);
            frame
        })
    }
}

impl Peripheral for Ssi {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        // `DR` pops a frame on the lane-0 read (firmware word-reads `DR`
        // ascending, like the TI UART hook); upper lanes read the frame
        // tail, like silicon's 16-bit data word.
        let (base, len) = {
            let r = self.inner.ranges()[0];
            (r.base, r.len)
        };
        let off = addr
            .checked_sub(base)
            .filter(|o| *o < len)
            .ok_or(BusError::Reserved)?;
        if (DR_OFF..DR_OFF + 4).contains(&off) {
            let lane = off - DR_OFF;
            if lane == 0 {
                let frame = self.inner.with_view(|view, b| {
                    let enabled =
                        view.get(CR1_OFF) & CR1_SSE != 0 && view.get(CR1_OFF) & CR1_MS == 0;
                    let popped = if enabled { b.rx.pop_front() } else { None };
                    b.popped = popped.unwrap_or(0);
                    b.refresh(view);
                    b.popped
                });
                return Ok((frame & 0xFF) as u8);
            }
            if lane == 1 {
                // High byte of the frame popped by the lane-0 read
                // (firmware word-reads `DR` ascending).
                return Ok(self.inner.with_view(|_, b| (b.popped >> 8) as u8));
            }
            return Ok(0);
        }
        // Table 15-2 p967 identification words (SVD omits them, all `RO`).
        if off >= 0xFD0 && off < 0x1000 {
            if off >= 0xFE0 {
                for &(base, word) in ID_WORDS {
                    if off >= base && off < base + 4 {
                        let shift = 8 * (off - base);
                        return Ok(((word >> shift) & 0xFF) as u8);
                    }
                }
            }
            // `PeriphID4-7` reset 0.
            return Ok(0);
        }
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let (base, len) = {
            let r = self.inner.ranges()[0];
            (r.base, r.len)
        };
        let off = addr
            .checked_sub(base)
            .filter(|o| *o < len)
            .ok_or(BusError::Reserved)?;
        // Identification words are `RO`: writes sink (Table 15-2 p967).
        if off >= 0xFD0 && off < 0x1000 {
            return Ok(());
        }
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
        self.stamp_resets();
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
}

impl InterruptSource for Ssi {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        u32::from(self.inner.behavior().pending)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const SSI0_BASE: u32 = 0x4000_8000;

    fn ssi0() -> Ssi {
        Ssi::for_device::<Tm4c123gh6pm>("SSI0")
    }

    fn read_word(ssi: &mut Ssi, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(ssi.read(SSI0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(ssi: &mut Ssi, offset: u32, value: u32) {
        for lane in 0..4 {
            ssi.write(
                SSI0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Master enable: `SSE` set, `MS` clear.
    fn enable(ssi: &mut Ssi) {
        write_word(ssi, CR1_OFF, CR1_SSE);
    }

    /// Datasheet resets: `SR` shows empty TX, `RIS` shows `TXRIS`.
    /// Table 15-2 p967 `PeriphID`/`CellID` read through the bus shim.
    #[test]
    fn datasheet_resets() {
        let mut ssi = ssi0();
        assert_eq!(read_word(&mut ssi, SR_OFF), 0x03);
        assert_eq!(read_word(&mut ssi, RIS_OFF), RIS_TX);
        assert_eq!(read_word(&mut ssi, CR0_OFF), 0);
        assert_eq!(read_word(&mut ssi, 0xFE0), 0x22);
        assert_eq!(read_word(&mut ssi, 0xFE8), 0x18);
        assert_eq!(read_word(&mut ssi, 0xFEC), 0x01);
        assert_eq!(read_word(&mut ssi, 0xFF0), 0x0D);
        assert_eq!(read_word(&mut ssi, 0xFF4), 0xF0);
        assert_eq!(read_word(&mut ssi, 0xFF8), 0x05);
        assert_eq!(read_word(&mut ssi, 0xFFC), 0xB1);
        assert_eq!(read_word(&mut ssi, 0xFD0), 0);
        // Identification words are `RO`.
        write_word(&mut ssi, 0xFE0, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut ssi, 0xFE0), 0x22);
    }

    /// Bit rate math: 16 MHz / (2 × (1 + 7)) = 1 MHz. Odd `CPSDVSR`
    /// reads back even (LSB hard-wired), so 3 behaves as 2.
    #[test]
    fn bitrate_math() {
        assert_eq!(bitrate_hz_for(2, 7, 16_000_000), 1_000_000);
        assert_eq!(bitrate_hz_for(3, 0, 16_000_000), 8_000_000);
        let mut ssi = ssi0();
        write_word(&mut ssi, CPSR_OFF, 2);
        write_word(&mut ssi, CR0_OFF, 0x07 << CR0_SCR_SHIFT | 0x07);
        assert_eq!(ssi.bitrate_hz_over(16_000_000), 1_000_000);
    }

    /// `CPSDVSR` LSB is hard-wired to zero; `CPSR[31:8]` are `RO` 0 (p976).
    #[test]
    fn cpsr_lsb_forced_zero() {
        let mut ssi = ssi0();
        write_word(&mut ssi, CPSR_OFF, 0x03);
        assert_eq!(read_word(&mut ssi, CPSR_OFF) & 0xFF, 0x02);
        write_word(&mut ssi, CPSR_OFF, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut ssi, CPSR_OFF), 0xFE);
    }

    /// Reserved `DSS 0x0-0x2` latch but read as 16-bit width (p970).
    #[test]
    fn dss_reserved_reads_as_16bit() {
        let mut ssi = ssi0();
        enable(&mut ssi);
        write_word(&mut ssi, CR0_OFF, 0x02);
        assert_eq!(read_word(&mut ssi, CR0_OFF) & CR0_DSS_MASK, 0x02);
        write_word(&mut ssi, CR1_OFF, CR1_SSE | CR1_LBM);
        write_word(&mut ssi, DR_OFF, 0xABCD);
        assert_eq!(read_word(&mut ssi, DR_OFF) & 0xFFFF, 0xABCD);
    }

    /// `BSY` is a 1-tick shifter hold (p974): set on the `DR` write, clear
    /// on the next `tick` once the FIFO is empty.
    #[test]
    fn bsy_is_one_tick_shifter_hold() {
        let mut ssi = ssi0();
        enable(&mut ssi);
        write_word(&mut ssi, CR0_OFF, 0x07);
        assert_eq!(read_word(&mut ssi, SR_OFF) & SR_BSY, 0);
        write_word(&mut ssi, DR_OFF, 0xA5);
        assert_eq!(read_word(&mut ssi, SR_OFF) & SR_BSY, SR_BSY);
        ssi.tick(1);
        assert_eq!(read_word(&mut ssi, SR_OFF) & SR_BSY, 0);
    }

    /// `DR` is 16-bit right-justified (p973): only full-word stores push a
    /// frame (TivaWare word-store assumption); byte stores assemble lanes
    /// without pushing.
    #[test]
    fn dr_word_store_pushes_byte_store_does_not() {
        let mut ssi = ssi0();
        enable(&mut ssi);
        write_word(&mut ssi, CR0_OFF, 0x07); // 8-bit frames
        ssi.write(SSI0_BASE + DR_OFF, 0xA5).unwrap();
        assert_eq!(ssi.drain_output(), []);
        write_word(&mut ssi, DR_OFF, 0xA5);
        assert_eq!(ssi.drain_output(), [0xA5]);
    }

    /// TX frames drain to the mailbox; `SR` tracks empty/full.
    #[test]
    fn tx_drains_to_mailbox() {
        let mut ssi = ssi0();
        enable(&mut ssi);
        write_word(&mut ssi, CR0_OFF, 0x07); // 8-bit frames
        write_word(&mut ssi, DR_OFF, 0xA5);
        assert_eq!(ssi.drain_output(), [0xA5]);
        assert_eq!(
            read_word(&mut ssi, SR_OFF) & (SR_TFE | SR_TNF),
            SR_TFE | SR_TNF
        );
    }

    /// `LBM` loopback routes TX into RX with the `DSS` width applied.
    #[test]
    fn loopback_routes_tx_to_rx() {
        let mut ssi = ssi0();
        enable(&mut ssi);
        write_word(&mut ssi, CR0_OFF, 0x07); // 8-bit frames
        write_word(&mut ssi, CR1_OFF, CR1_SSE | CR1_LBM);
        write_word(&mut ssi, IM_OFF, RIS_RX);
        write_word(&mut ssi, DR_OFF, 0x1A5); // masked to 0xA5
        assert_eq!(read_word(&mut ssi, SR_OFF) & SR_RNE, SR_RNE);
        assert_eq!(ssi.pending(), 0); // 1 of 8 < half-full trigger
        assert_eq!(ssi.exception_number(), Some(23));
        assert_eq!(read_word(&mut ssi, DR_OFF) & 0xFFFF, 0xA5);
    }

    /// Injected RX frames raise `RXRIS` at half-full; reading below it
    /// clears; overflow sets sticky `RORRIS`, cleared by `RORIC`.
    #[test]
    fn rx_interrupt_and_overrun() {
        let mut ssi = ssi0();
        enable(&mut ssi);
        write_word(&mut ssi, IM_OFF, RIS_RX | RIS_ROR);
        ssi.inject_rx(&[1, 2, 3]);
        assert_eq!(read_word(&mut ssi, RIS_OFF) & RIS_RX, 0);
        ssi.inject_rx(&[4]);
        assert_eq!(read_word(&mut ssi, RIS_OFF) & RIS_RX, RIS_RX);
        assert_eq!(read_word(&mut ssi, MIS_OFF) & RIS_RX, RIS_RX);
        assert_eq!(ssi.pending(), 1);
        assert_eq!(ssi.pop_rx(), Some(1));
        assert_eq!(read_word(&mut ssi, RIS_OFF) & RIS_RX, 0);
        ssi.inject_rx(&[5, 6, 7, 8, 9, 10]);
        assert_eq!(read_word(&mut ssi, RIS_OFF) & RIS_ROR, RIS_ROR);
        write_word(&mut ssi, ICR_OFF, ICR_RORIC);
        assert_eq!(read_word(&mut ssi, RIS_OFF) & RIS_ROR, 0);
    }

    /// `SR`/`RIS`/`MIS` ignore writes.
    #[test]
    fn ro_words_ignore_writes() {
        let mut ssi = ssi0();
        write_word(&mut ssi, SR_OFF, 0xFFFFFFFF);
        assert_eq!(read_word(&mut ssi, SR_OFF), 0x03);
        write_word(&mut ssi, RIS_OFF, 0xFFFFFFFF);
        assert_eq!(read_word(&mut ssi, RIS_OFF) & RIS_RX, 0);
    }
}
