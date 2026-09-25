//! TI Tiva C UARTs (`UART0`-`UART7`).
//!
//! Register map, resets, and FIFO/interrupt rules follow the TM4C123GH6PM
//! datasheet (SPMS376E §14, Table 14-3 for resets): 16-deep FIFOs (1-deep
//! holding registers with `FEN=0`), `FR` synthesis (`TXFE`/`TXFF`/`RXFE`/
//! `RXFF`/`BUSY`), `IBRD`/`FBRD` baud math (`clk / (ClkDiv × (IBRD+FBRD/64))`,
//! `ClkDiv=16 HSE=0 / 8 HSE=1` per §14.3.2 p896, `UARTCC` selects system clock
//! or `PIOSC`), `IFLS` trigger levels, `RIS`/`MIS` with `ICR` W1C clear (clears
//! `RIS` only), sticky `RSR` errors cleared by any `ECR` write (separate latch
//! from `RIS` per p908 vs p933), `DR` error-flag lanes on read (per-char flags
//! in `DR[11:8]`, `RSR` reflects the prior `DR` per p906-907).
//!
//! Like the repo's STM32 USART, TX completes into the host mailbox the
//! instant firmware writes it — no baud timing is modeled. `BUSY` is a 1-tick
//! shifter hold: set on a TX write, cleared on the next `tick` once the FIFO is
//! empty and the shifter is done (p911-912). RX bytes arrive through
//! [`Uart::inject_rx`] (host/test stimulus); overflow sets sticky `OE` in both
//! the `RSR` and `RIS` latches. `LBE` loopback routes TX into RX in silicon
//! order. Transition-triggered interrupts are approximated by levels (`TXRIS`
//! while `TX ≤ trigger`, `RXRIS` while `RX ≥ trigger`); `TXIC`/`RXIC` suppress
//! their level bit until the FIFO level moves again. The receive-timeout, 9-bit,
//! and CTS interrupts never fire (documented). `UART9BITAMASK` resets to
//! `0x00FF`; `PeriphID`/`CellID` read as Table 14-3 p904 even though the SVD
//! omits them (custom bus shim).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;

const DR_OFF: u32 = 0x000;
const RSR_OFF: u32 = 0x004;
const FR_OFF: u32 = 0x018;
const IBRD_OFF: u32 = 0x024;
const FBRD_OFF: u32 = 0x028;
const LCRH_OFF: u32 = 0x02C;
const CTL_OFF: u32 = 0x030;
const IFLS_OFF: u32 = 0x034;
const IM_OFF: u32 = 0x038;
const RIS_OFF: u32 = 0x03C;
const MIS_OFF: u32 = 0x040;
const ICR_OFF: u32 = 0x044;
const CC_OFF: u32 = 0xFC8;

/// `FR` bits.
const FR_TXFE: u32 = 1 << 7;
const FR_RXFF: u32 = 1 << 6;
const FR_TXFF: u32 = 1 << 5;
const FR_RXFE: u32 = 1 << 4;
const FR_BUSY: u32 = 1 << 3;
/// `LCRH.FEN` bit 4.
const LCRH_FEN: u32 = 1 << 4;
/// `CTL` bits: `UARTEN` 0, `HSE` 5, `LBE` 7, `TXE` 8, `RXE` 9.
const CTL_UARTEN: u32 = 1 << 0;
const CTL_HSE: u32 = 1 << 5;
const CTL_LBE: u32 = 1 << 7;
const CTL_TXE: u32 = 1 << 8;
const CTL_RXE: u32 = 1 << 9;
/// `RIS`/`ICR` bits: `TX` 5, `RX` 4, `OE` 10, `BE` 9, `PE` 8, `FE` 7.
const RIS_TX: u32 = 1 << 5;
const RIS_RX: u32 = 1 << 4;
const RIS_OE: u32 = 1 << 10;
const RIS_BE: u32 = 1 << 9;
const RIS_PE: u32 = 1 << 8;
const RIS_FE: u32 = 1 << 7;
/// `DR` error-flag lanes: `OE` 11, `BE` 10, `PE` 9, `FE` 8.
const DR_OE: u32 = 1 << 11;
const DR_BE: u32 = 1 << 10;
const DR_PE: u32 = 1 << 9;
const DR_FE: u32 = 1 << 8;
/// `RSR` error lanes: `OE` 3, `BE` 2, `PE` 1, `FE` 0.
const RSR_OE: u32 = 1 << 3;
const RSR_BE: u32 = 1 << 2;
const RSR_PE: u32 = 1 << 1;
const RSR_FE: u32 = 1 << 0;
/// `UARTCC.CS`: `0x0` system clock, `0x5` PIOSC.
const CC_PIOSC: u32 = 0x05;
/// `PIOSC` rate in Hz (datasheet §5.2.5.1).
const PIOSC_HZ: u32 = 16_000_000;
/// FIFO depth with `FEN=1` (datasheet IFLS page: half is 8 of 16).
const FIFO_DEPTH: usize = 16;
/// `RXIFLSEL`/`TXIFLSEL` level tables (entries at/above, free at/below).
const RX_TRIGGER: &[usize] = &[2, 4, 8, 12, 14];
const TX_TRIGGER: &[usize] = &[2, 4, 8, 12, 14];

/// Nonzero datasheet resets the behavior stamps over the SVD zeros.
/// Table 14-3 p904: `9BITAMASK=0x00FF`; `PeriphID`/`CellID` are handled by
/// the bus shim below (SVD omits them).
const RESETS: &[(u32, u32)] = &[
    (FR_OFF, 0x0000_0090),
    (CTL_OFF, 0x0000_0300),
    (IFLS_OFF, 0x0000_0012),
    (0x0A8, 0x0000_00FF), // UART9BITAMASK
    (0xFC0, 0x0000_0003), // UARTPP
];

/// Identification words missing from the SVD (Table 14-3 p904, all `RO`).
/// `PeriphID4-7` read 0.
const ID_WORDS: &[(u32, u32)] = &[
    (0xFE0, 0x0000_0060), // PeriphID0
    (0xFE4, 0x0000_0000), // PeriphID1
    (0xFE8, 0x0000_0018), // PeriphID2
    (0xFEC, 0x0000_0001), // PeriphID3
    (0xFF0, 0x0000_000D), // CellID0
    (0xFF4, 0x0000_00F0), // CellID1
    (0xFF8, 0x0000_0005), // CellID2
    (0xFFC, 0x0000_00B1), // CellID3
];

/// Baud rate in Hz from the divisor words, the baud-clock rate, and `HSE`:
/// `clk / (ClkDiv × (IBRD + FBRD/64))` with `ClkDiv=16 HSE=0 / 8 HSE=1`
/// (§14.3.2 p896), 0 when unprogrammed.
#[must_use]
pub fn baud_hz_for(ibrd: u32, fbrd: u32, uartclk_hz: u32, hse: bool) -> u32 {
    let div = u64::from(ibrd) * 64 + u64::from(fbrd & 0x3F);
    if div == 0 || uartclk_hz == 0 {
        return 0;
    }
    let clkdiv = if hse { 8 } else { 16 };
    u32::try_from(u64::from(uartclk_hz) * 64 / (clkdiv * div)).unwrap_or(u32::MAX)
}

/// Behavior behind one UART window.
#[derive(Debug, Default)]
pub struct UartBehavior {
    tx: VecDeque<u8>,
    /// RX entries as `(byte, error-nibble)`; the nibble holds the popped-char
    /// `OE/BE/PE/FE` flags for `DR[11:8]` (p906-907). `BE/PE/FE` are never
    /// generated here (no parity/break modeling), so the nibble is 0 except
    /// for documentation; `OE` overrun is sticky (see `rsr`/`ris_err`).
    rx: VecDeque<(u8, u8)>,
    /// Bytes firmware transmitted, for the host to drain.
    tx_out: Vec<u8>,
    /// `RSR` latch `(oe, be, pe, fe)`: set on receive errors, cleared by any
    /// `ECR` write only (p908-910).
    rsr: (bool, bool, bool, bool),
    /// `RIS` error latch `(oe, be, pe, fe)`: set on the same events, cleared
    /// by `ICR` W1C only (p927-933). Separate from `rsr`.
    ris_err: (bool, bool, bool, bool),
    /// Error nibble latched with the last popped RX byte (`DR[11:8]` lane 1,
    /// low nibble; upper nibble 0).
    popped_flags: u8,
    /// `BUSY` shifter hold: set on a TX write, cleared on the next `tick`
    /// once the FIFO is empty and the shifter (incl. stop bits) is done
    /// (p911-912). Instant TX drain keeps `tx` empty, so `FR.BUSY` reads this.
    busy: bool,
    /// `TXIC`/`RXIC` suppression levels (interrupt stays clear until the
    /// FIFO level moves past the cleared point).
    tx_cleared_at: Option<usize>,
    rx_cleared_at: Option<usize>,
    /// Cached `(RIS & IM) != 0` (`InterruptSource::pending` is `&self`).
    pending: bool,
}

impl UartBehavior {
    fn enabled(&self, view: &Registers<&mut [u32]>) -> bool {
        view.get(CTL_OFF) & CTL_UARTEN != 0
    }

    fn depth(&self, view: &Registers<&mut [u32]>) -> usize {
        if view.get(LCRH_OFF) & LCRH_FEN != 0 {
            FIFO_DEPTH
        } else {
            1
        }
    }

    fn rx_trigger(&self, view: &Registers<&mut [u32]>) -> usize {
        let sel = ((view.get(IFLS_OFF) >> 3) & 0x07) as usize;
        RX_TRIGGER.get(sel).copied().unwrap_or(FIFO_DEPTH)
    }

    fn tx_trigger(&self, view: &Registers<&mut [u32]>) -> usize {
        let sel = (view.get(IFLS_OFF) & 0x07) as usize;
        TX_TRIGGER.get(sel).copied().unwrap_or(0)
    }

    /// Raw interrupt word from FIFO levels, sticky errors, and clears.
    fn ris(&self, view: &Registers<&mut [u32]>) -> u32 {
        let mut ris = 0u32;
        let tx_len = self.tx.len();
        let rx_len = self.rx.len();
        let tx_suppressed = self.tx_cleared_at.is_some_and(|at| tx_len == at);
        let rx_suppressed = self.rx_cleared_at.is_some_and(|at| rx_len == at);
        if !tx_suppressed && tx_len <= self.tx_trigger(view) {
            ris |= RIS_TX;
        }
        if !rx_suppressed && rx_len >= self.rx_trigger(view) && rx_len > 0 {
            ris |= RIS_RX;
        }
        let (oe, be, pe, fe) = self.ris_err;
        if oe {
            ris |= RIS_OE;
        }
        if be {
            ris |= RIS_BE;
        }
        if pe {
            ris |= RIS_PE;
        }
        if fe {
            ris |= RIS_FE;
        }
        ris
    }

    fn flags_word(&self) -> u32 {
        let (oe, be, pe, fe) = self.rsr;
        (u32::from(oe) << 3) | (u32::from(be) << 2) | (u32::from(pe) << 1) | u32::from(fe)
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        self.pending = self.ris(view) & view.get(IM_OFF) != 0;
    }
}

impl Behavior for UartBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn tick(&mut self, _view: &mut Registers<&mut [u32]>, _delta: u64) {
        // 1-tick shifter hold: the instant-drained FIFO is empty, so clear
        // the `BUSY` latch on the next tick.
        self.busy = false;
    }

    fn reset(&mut self) {
        *self = Self::default();
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            DR_OFF => value,
            RSR_OFF => self.flags_word(),
            FR_OFF => {
                let mut fr = 0u32;
                if self.tx.is_empty() {
                    fr |= FR_TXFE;
                }
                if self.tx.len() >= self.depth(view) {
                    fr |= FR_TXFF;
                }
                if self.rx.len() >= self.depth(view) {
                    fr |= FR_RXFF;
                }
                if self.rx.is_empty() {
                    fr |= FR_RXFE;
                }
                // p911-912: set on TX write, clear only when FIFO empty
                // *and* the shifter (incl. stop bits) is done. Instant drain
                // keeps `tx` empty, so the 1-tick `busy` hold is the shifter.
                if self.busy || !self.tx.is_empty() {
                    fr |= FR_BUSY;
                }
                fr
            }
            RIS_OFF => self.ris(view),
            MIS_OFF => self.ris(view) & view.get(IM_OFF),
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
        let out = match offset {
            DR_OFF => {
                // Only lane 0 carries `DATA`; error-flag lanes are RO.
                if shift == 0 && self.enabled(view) && view.get(CTL_OFF) & CTL_TXE != 0 {
                    let depth = self.depth(view);
                    if self.tx.len() < depth {
                        self.tx.push_back(val);
                    }
                    // Shifter hold for `BUSY` (cleared on next `tick`).
                    self.busy = true;
                    // Instant completion into the host mailbox (no baud
                    // timing, same convention as the STM32 USART).
                    while let Some(b) = self.tx.pop_front() {
                        self.tx_out.push(b);
                        // Internal loopback routes TX into RX in order.
                        if view.get(CTL_OFF) & CTL_LBE != 0
                            && view.get(CTL_OFF) & CTL_RXE != 0
                            && self.rx.len() < self.depth(view)
                        {
                            self.rx.push_back((b, 0));
                        }
                    }
                    self.tx_cleared_at = None;
                }
                view.get(offset)
            }
            RSR_OFF => {
                // Any `ECR` write clears the `RSR` latch only; `RIS` keeps
                // its own latch until `ICR` (p908 vs p933). `RSR` is RO.
                self.rsr = (false, false, false, false);
                view.get(offset)
            }
            FR_OFF | RIS_OFF | MIS_OFF => view.get(offset),
            ICR_OFF => {
                let word = view.merged(offset, val, shift);
                // W1C clears the `RIS` latch only; `RSR` is untouched.
                if word & RIS_OE != 0 {
                    self.ris_err.0 = false;
                }
                if word & RIS_BE != 0 {
                    self.ris_err.1 = false;
                }
                if word & RIS_PE != 0 {
                    self.ris_err.2 = false;
                }
                if word & RIS_FE != 0 {
                    self.ris_err.3 = false;
                }
                if word & RIS_TX != 0 {
                    self.tx_cleared_at = Some(self.tx.len());
                }
                if word & RIS_RX != 0 {
                    self.rx_cleared_at = Some(self.rx.len());
                }
                view.get(offset)
            }
            _ => view.merged(offset, val, shift),
        };
        self.refresh(view);
        out
    }
}

/// A Tiva C UART.
pub struct Uart {
    inner: RegisterFile<UartBehavior>,
}

impl Uart {
    /// A new UART on `D`'s tables (`"UART0"`…`"UART7"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain UART");
        let mut uart = Self {
            inner: RegisterFile::new(def, UartBehavior::default()),
        };
        uart.stamp_resets();
        uart
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

    /// Baud rate in Hz over an explicit baud-clock rate (system clock or
    /// `PIOSC`, per `UARTCC.CS`). `HSE` in `UARTCTL` selects `ClkDiv` 8/16.
    #[must_use]
    pub fn baud_hz_over(&mut self, uartclk_hz: u32) -> u32 {
        self.inner.with_view(|view, _| {
            let hse = view.get(CTL_OFF) & CTL_HSE != 0;
            baud_hz_for(view.get(IBRD_OFF), view.get(FBRD_OFF), uartclk_hz, hse)
        })
    }

    /// Bytes firmware transmitted, for the host to drain.
    pub fn drain_output(&mut self) -> Vec<u8> {
        self.inner.with_view(|_, b| std::mem::take(&mut b.tx_out))
    }

    /// Feed received bytes (host/test stimulus). Overflow sets sticky `OE`
    /// in both the `RSR` and `RIS` latches.
    pub fn inject_rx(&mut self, bytes: &[u8]) {
        self.inner.with_view(|view, b| {
            let depth = if view.get(LCRH_OFF) & LCRH_FEN != 0 {
                FIFO_DEPTH
            } else {
                1
            };
            for &byte in bytes {
                if b.rx.len() < depth {
                    // No parity/break modeling: per-char error nibble is 0.
                    b.rx.push_back((byte, 0));
                } else {
                    b.rsr.0 = true;
                    b.ris_err.0 = true;
                }
            }
            b.rx_cleared_at = None;
        });
    }

    /// Pop one received byte (test hook; firmware reads `DR`).
    pub fn pop_rx(&mut self) -> Option<u8> {
        self.inner
            .with_view(|_, b| b.rx.pop_front().map(|(byte, _)| byte))
    }
}

impl Peripheral for Uart {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        // `DR` pops on lane 0 and latches that byte's error nibble for the
        // lane-1 read (`DR[11:8]`, low nibble; p906-907); lanes 2-3 read 0.
        // `RSR` (p908) reflects the sticky latch for the prior `DR`.
        let off = addr
            .checked_sub(self.inner.ranges()[0].base)
            .filter(|o| *o < self.inner.ranges()[0].len)
            .ok_or(BusError::Reserved)?;
        if off < 4 {
            let lane = off;
            if lane == 0 {
                let byte = self.inner.with_view(|view, b| {
                    let enabled =
                        view.get(CTL_OFF) & CTL_UARTEN != 0 && view.get(CTL_OFF) & CTL_RXE != 0;
                    let popped = if enabled { b.rx.pop_front() } else { None };
                    b.popped_flags = popped.map_or(0, |(_, flags)| flags & 0x0F);
                    if popped.is_some() {
                        b.rx_cleared_at = None;
                    }
                    popped.map_or(0, |(byte, _)| byte)
                });
                return Ok(byte);
            }
            if lane == 1 {
                return Ok(self.inner.with_view(|_, b| b.popped_flags & 0x0F));
            }
            return Ok(0);
        }
        // Table 14-3 p904 identification words (SVD omits them, all `RO`).
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
        let off = addr
            .checked_sub(self.inner.ranges()[0].base)
            .filter(|o| *o < self.inner.ranges()[0].len)
            .ok_or(BusError::Reserved)?;
        // Identification words are `RO`: writes sink.
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

    fn name(&self) -> &'static str {
        self.inner.def().name
    }

    fn drain_output(&mut self) -> Vec<u8> {
        Uart::drain_output(self)
    }

    fn drive_inputs(&mut self, value: u32) {
        self.inject_rx(&[value as u8]);
    }
}

impl InterruptSource for Uart {
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

    const UART0_BASE: u32 = 0x4000_C000;

    fn uart0() -> Uart {
        Uart::for_device::<Tm4c123gh6pm>("UART0")
    }

    fn read_word(uart: &mut Uart, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(uart.read(UART0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(uart: &mut Uart, offset: u32, value: u32) {
        for lane in 0..4 {
            uart.write(
                UART0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Enable TX (and RX where needed): `UARTEN | TXE | RXE`.
    fn enable(uart: &mut Uart) {
        write_word(uart, CTL_OFF, CTL_UARTEN | CTL_TXE | CTL_RXE);
    }

    /// Datasheet Table 14-3 resets land on the bus.
    #[test]
    fn datasheet_resets() {
        let mut uart = uart0();
        assert_eq!(read_word(&mut uart, FR_OFF), 0x90);
        assert_eq!(read_word(&mut uart, CTL_OFF), 0x300);
        assert_eq!(read_word(&mut uart, IFLS_OFF), 0x12);
        assert_eq!(read_word(&mut uart, 0xFC0), 0x03);
        assert_eq!(read_word(&mut uart, IBRD_OFF), 0);
        // Table 14-3 p904: `9BITAMASK=0x00FF`, IDs even though SVD omits them.
        assert_eq!(read_word(&mut uart, 0x0A8), 0xFF);
        assert_eq!(read_word(&mut uart, 0xFE0), 0x60);
        assert_eq!(read_word(&mut uart, 0xFE8), 0x18);
        assert_eq!(read_word(&mut uart, 0xFEC), 0x01);
        assert_eq!(read_word(&mut uart, 0xFF0), 0x0D);
        assert_eq!(read_word(&mut uart, 0xFF4), 0xF0);
        assert_eq!(read_word(&mut uart, 0xFF8), 0x05);
        assert_eq!(read_word(&mut uart, 0xFFC), 0xB1);
        assert_eq!(read_word(&mut uart, 0xFD0), 0);
        // Identification words are `RO`.
        write_word(&mut uart, 0xFE0, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut uart, 0xFE0), 0x60);
    }

    /// The datasheet worked example: 20 MHz / (16 × 115200) → `IBRD=10`,
    /// `FBRD=54` reads back 115273 baud (+0.06%, inside UART tolerance).
    /// With `HSE` set the divider is 8, doubling the rate (§14.3.2 p896).
    #[test]
    fn baud_math_matches_worked_example() {
        assert_eq!(baud_hz_for(10, 54, 20_000_000, false), 115_273);
        assert_eq!(baud_hz_for(10, 54, 20_000_000, true), 230_547);
        assert_eq!(baud_hz_for(0, 0, 20_000_000, false), 0);
        let mut uart = uart0();
        write_word(&mut uart, IBRD_OFF, 10);
        write_word(&mut uart, FBRD_OFF, 54);
        assert_eq!(uart.baud_hz_over(20_000_000), 115_273);
        write_word(&mut uart, CTL_OFF, CTL_UARTEN | CTL_TXE | CTL_RXE | CTL_HSE);
        assert_eq!(uart.baud_hz_over(20_000_000), 230_547);
    }

    /// TX completes into the host mailbox; `BUSY` is a 1-tick shifter hold
    /// (p911-912): set on the TX write, clear on the next `tick`.
    #[test]
    fn tx_drains_to_mailbox() {
        let mut uart = uart0();
        enable(&mut uart);
        assert_eq!(read_word(&mut uart, FR_OFF) & FR_TXFE, FR_TXFE);
        assert_eq!(read_word(&mut uart, FR_OFF) & FR_BUSY, 0);
        uart.write(UART0_BASE + DR_OFF, b'H').unwrap();
        uart.write(UART0_BASE + DR_OFF, b'i').unwrap();
        assert_eq!(uart.drain_output(), b"Hi");
        assert_eq!(read_word(&mut uart, FR_OFF) & FR_BUSY, FR_BUSY);
        uart.tick(1);
        assert_eq!(read_word(&mut uart, FR_OFF) & (FR_TXFE | FR_BUSY), FR_TXFE);
    }

    /// `LBE` loopback routes TX into RX in order, raising `RXRIS`.
    #[test]
    fn loopback_routes_tx_to_rx() {
        let mut uart = uart0();
        enable(&mut uart);
        // FIFO mode so the looped bytes queue; RX trigger ⅛ (2 bytes).
        write_word(&mut uart, LCRH_OFF, LCRH_FEN);
        write_word(&mut uart, IFLS_OFF, 0x02);
        write_word(&mut uart, CTL_OFF, CTL_UARTEN | CTL_TXE | CTL_RXE | CTL_LBE);
        write_word(&mut uart, IM_OFF, RIS_RX);
        uart.write(UART0_BASE + DR_OFF, 0xA5).unwrap();
        uart.write(UART0_BASE + DR_OFF, 0x5A).unwrap();
        assert_eq!(read_word(&mut uart, FR_OFF) & FR_RXFE, 0);
        assert_eq!(uart.pending(), 1);
        assert_eq!(uart.exception_number(), Some(21));
        assert_eq!(read_word(&mut uart, DR_OFF) & 0xFF, 0xA5);
        assert_eq!(read_word(&mut uart, DR_OFF) & 0xFF, 0x5A);
        assert_eq!(read_word(&mut uart, FR_OFF) & FR_RXFE, FR_RXFE);
    }

    /// Injected RX bytes raise `RXRIS` at the trigger level; reading below
    /// it clears; `TXRIS` fires while the TX FIFO sits at/below trigger.
    #[test]
    fn rx_interrupt_flow() {
        let mut uart = uart0();
        enable(&mut uart);
        write_word(&mut uart, LCRH_OFF, LCRH_FEN);
        write_word(&mut uart, IM_OFF, RIS_RX | RIS_TX);
        // Default RX trigger is half (8 of 16): 7 bytes stay quiet.
        uart.inject_rx(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]);
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_RX, 0);
        uart.inject_rx(&[0x88]);
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_RX, RIS_RX);
        assert_eq!(read_word(&mut uart, MIS_OFF) & RIS_RX, RIS_RX);
        // TX idle: `TXRIS` set (nothing to send).
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_TX, RIS_TX);
        // Reading back below the trigger clears `RXRIS`.
        assert_eq!(read_word(&mut uart, DR_OFF) & 0xFF, 0x11);
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_RX, 0);
    }

    /// RX overflow sets sticky `OE` in both latches (p908 vs p927-933):
    /// `RSR` clears by any `ECR` write only, `RIS` by `ICR` W1C only.
    #[test]
    fn overrun_sets_sticky_oe() {
        let mut uart = uart0();
        enable(&mut uart);
        write_word(&mut uart, LCRH_OFF, LCRH_FEN);
        uart.inject_rx(&[0x55; 16]);
        uart.inject_rx(&[0xAA]);
        assert_eq!(read_word(&mut uart, RSR_OFF) & RSR_OE, RSR_OE);
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_OE, RIS_OE);
        // `ECR` clears `RSR` only; `RIS` stays.
        write_word(&mut uart, RSR_OFF, 0xFFFFFFFF);
        assert_eq!(read_word(&mut uart, RSR_OFF), 0);
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_OE, RIS_OE);
        // `ICR` clears `RIS` only; `RSR` stays clear here (already clear).
        write_word(&mut uart, ICR_OFF, RIS_OE);
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_OE, 0);
        assert_eq!(read_word(&mut uart, RSR_OFF), 0);
    }

    /// `ICR` error clears leave `RSR` set (separate latches, p908 vs p933).
    #[test]
    fn icr_does_not_clear_rsr() {
        let mut uart = uart0();
        enable(&mut uart);
        write_word(&mut uart, LCRH_OFF, LCRH_FEN);
        uart.inject_rx(&[0x55; 16]);
        uart.inject_rx(&[0xAA]);
        write_word(&mut uart, ICR_OFF, RIS_OE);
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_OE, 0);
        assert_eq!(read_word(&mut uart, RSR_OFF) & RSR_OE, RSR_OE);
        write_word(&mut uart, RSR_OFF, 0);
        assert_eq!(read_word(&mut uart, RSR_OFF), 0);
    }

    /// `DR[11:8]` lane 1 carries the popped char's error nibble (here 0:
    /// no parity/break modeling); `RSR` reflects the prior `DR`'s sticky
    /// latch (p906-907).
    #[test]
    fn dr_error_lanes_carry_popped_flags() {
        let mut uart = uart0();
        enable(&mut uart);
        write_word(&mut uart, LCRH_OFF, LCRH_FEN);
        uart.inject_rx(&[0xA5]);
        assert_eq!(uart.read(UART0_BASE + DR_OFF + 1).unwrap(), 0);
        assert_eq!(uart.read(UART0_BASE + DR_OFF).unwrap(), 0xA5);
        assert_eq!(uart.read(UART0_BASE + DR_OFF + 1).unwrap() & 0x0F, 0);
        assert_eq!(uart.read(UART0_BASE + DR_OFF + 2).unwrap(), 0);
        assert_eq!(uart.read(UART0_BASE + DR_OFF + 3).unwrap(), 0);
        assert_eq!(read_word(&mut uart, RSR_OFF), 0);
    }

    /// FIFO-disabled mode is 1-deep: the second RX byte overruns.
    #[test]
    fn fifo_disabled_is_one_deep() {
        let mut uart = uart0();
        enable(&mut uart);
        assert_eq!(read_word(&mut uart, LCRH_OFF) & LCRH_FEN, 0);
        uart.inject_rx(&[0x41]);
        uart.inject_rx(&[0x42]);
        assert_eq!(read_word(&mut uart, RSR_OFF) & RSR_OE, RSR_OE);
        assert_eq!(read_word(&mut uart, DR_OFF) & 0xFF, 0x41);
    }

    /// `FR`/`RIS`/`MIS` ignore writes (Table 14-3 `RO`).
    #[test]
    fn ro_words_ignore_writes() {
        let mut uart = uart0();
        write_word(&mut uart, FR_OFF, 0xFFFFFFFF);
        assert_eq!(read_word(&mut uart, FR_OFF), 0x90);
        // `RXRIS` stays clear on an idle receiver; `TXRIS` reads set
        // (empty TX FIFO) but is not stored by the write.
        write_word(&mut uart, RIS_OFF, 0xFFFFFFFF);
        assert_eq!(read_word(&mut uart, RIS_OFF) & RIS_RX, 0);
    }
}
