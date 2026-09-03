//! The STM32F1 reset and clock control unit (`RCC`, `0x4002_1000`), driven by
//! the SVD-generated register tables.
//!
//! Every clock enable and status bit from the SVD is latched and readable; the
//! internal HSI oscillator reads as always on and ready (so
//! `while (!(RCC->CR & RCC_CR_HSIRDY))` terminates), and the PLL locks
//! instantly once `PLLON` is set — both expressed as a [`Behavior`] hook on
//! the otherwise table-driven [`RegisterFile`]. Clock gating of the downstream
//! peripherals is not enforced here — see the `stm32` module docs.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// HSION bit in `CR`.
const HSION: u32 = 1 << 0;
/// HSIRDY bit in `CR` — F1 is bit 1, H7 is bit 2.
const HSIRDY_F1: u32 = 1 << 1;
const HSIRDY_H7: u32 = 1 << 2;
/// HSEON/HSERDY bits (H7 `CR` 16/17).
const HSEON: u32 = 1 << 16;
const HSERDY: u32 = 1 << 17;
/// PLLON / PLL1ON bit in `CR`.
const PLLON: u32 = 1 << 24;
/// PLLRDY / PLL1RDY bit in `CR`.
const PLLRDY: u32 = 1 << 25;
/// H7 PLL2/PLL3.
const PLL2ON: u32 = 1 << 26;
const PLL2RDY: u32 = 1 << 27;
const PLL3ON: u32 = 1 << 28;
const PLL3RDY: u32 = 1 << 29;
/// CSI comparable (H7 CSION 7 / CSIRDY 8).
const CSION: u32 = 1 << 7;
const CSIRDY: u32 = 1 << 8;

/// `CR` lives at register offset `0x00`.
const CR_OFFSET: u32 = 0x00;
/// `CFGR` lives at register offset `0x04`.
const CFGR_OFFSET: u32 = 0x04;
/// H7 `D1CFGR`/`D2CFGR` live at `0x18`/`0x1C` (HPRE/D1CPRE/D1PPRE, D2PPRE1/2).
const D1CFGR_OFFSET: u32 = 0x18;
const D2CFGR_OFFSET: u32 = 0x1C;

/// AHB/APB prescaler state decoded from RCC clock-configuration registers,
/// shared with the bus so timer tick scaling tracks firmware's `CFGR` writes.
///
/// Dividers are stored, not frequencies: the model has no absolute clock, so
/// peripherals scale relative to core cycles. On F1/F4-family parts the core
/// runs `HCLK` and `HPRE` cancels out of the timer ratio; on H7 the core runs
/// `cpu_ck = sys_ck/D1CPRE` while timers run off `HCLK`, hence `d1cpre`.
///
/// Not modeled (documented follow-ups): H7 `TIMPRE` (timers ×4 when set with
/// `PPRE >= 4`; modeled as ×2), `SysTick` `CLKSOURCE`, and PLL frequency math
/// (PLL locks instantly, so dividers are the only observable clock state).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockState {
    /// AHB divider from `HPRE` (F1/F4 `CFGR`, H7 `D1CFGR`).
    pub hpre: u32,
    /// H7 CPU divider from `D1CPRE`; always 1 off-H7.
    pub d1cpre: u32,
    /// APB1 divider (`PPRE1`, H7 `D2PPRE1`).
    pub ppre1: u32,
    /// APB2 divider (`PPRE2`, H7 `D2PPRE2`).
    pub ppre2: u32,
    h7: bool,
}

impl ClockState {
    /// Reset state: every prescaler divides by 1, matching SVD reset words.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            hpre: 1,
            d1cpre: 1,
            ppre1: 1,
            ppre2: 1,
            h7: false,
        }
    }

    /// A fresh shared handle for one board: the `Rcc` writes it, the bus
    /// reads it. `Send` so it can live on the bus next to the other mailboxes.
    #[must_use]
    pub fn shared() -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(Self::new()))
    }

    /// Restore divider-neutral state, keeping the family flag: mirrors the
    /// SVD reset words, which reset every prescaler to /1.
    pub fn reset_dividers(&mut self) {
        let h7 = self.h7;
        *self = Self::new();
        self.h7 = h7;
    }

    /// `(numerator, denominator)` of the APB `apb` (1 or 2) timer-kernel
    /// clock relative to core cycles, including the STM32 ×2 rule (timers
    /// run at twice `PCLKx` when the APB prescaler divides).
    #[must_use]
    pub fn tim_factor(&self, apb: u8) -> (u64, u64) {
        let ppre = if apb == 2 { self.ppre2 } else { self.ppre1 };
        let k = if ppre <= 1 { 1u64 } else { 2u64 };
        if self.h7 {
            (
                u64::from(self.d1cpre) * k,
                u64::from(self.hpre) * u64::from(ppre),
            )
        } else {
            (k, u64::from(ppre))
        }
    }
}

impl Default for ClockState {
    fn default() -> Self {
        Self::new()
    }
}

/// `HPRE`-style 4-bit divider encoding (F1/F4 `CFGR`, H7 `D1CFGR`/`D1CPRE`).
fn decode_hpre(v: u32) -> u32 {
    match v {
        0..=7 => 1,
        8 => 2,
        9 => 4,
        10 => 8,
        11 => 16,
        12 => 64,
        13 => 128,
        14 => 256,
        _ => 512,
    }
}

/// `PPRE`-style 3-bit divider encoding (`PPRE1/2`, H7 `D1PPRE`/`D2PPREx`).
/// Single-APB families (F0/G0) have no `PPRE2`; those bits read 0 → /1, which
/// is exactly their undivided APB2.
fn decode_ppre(v: u32) -> u32 {
    match v {
        0..=3 => 1,
        4 => 2,
        5 => 4,
        6 => 8,
        _ => 16,
    }
}

/// RCC's synthesis: clock-ready bits in `CR` + CFGR SW/SWS.
#[derive(Clone, Debug)]
struct RccBehavior {
    hsi_rdy: u32,
    h7: bool,
    clock: Option<Arc<Mutex<ClockState>>>,
}

impl Default for RccBehavior {
    fn default() -> Self {
        Self {
            hsi_rdy: HSIRDY_F1,
            h7: false,
            clock: None,
        }
    }
}

impl RccBehavior {
    /// Push one decoded register word into the shared clock handle.
    fn note_cfgr(&mut self, word: u32) {
        let Some(clock) = &self.clock else {
            return;
        };
        let mut clock = clock.lock().unwrap_or_else(|e| e.into_inner());
        clock.hpre = decode_hpre((word >> 4) & 0xF);
        clock.ppre1 = decode_ppre((word >> 8) & 0x7);
        clock.ppre2 = decode_ppre((word >> 11) & 0x7);
    }

    /// Push a decoded H7 `D1CFGR` word (`HPRE`/`D1CPRE`; `D1PPRE` feeds no
    /// general timer and is intentionally ignored).
    fn note_d1cfgr(&mut self, word: u32) {
        let Some(clock) = &self.clock else {
            return;
        };
        let mut clock = clock.lock().unwrap_or_else(|e| e.into_inner());
        clock.hpre = decode_hpre(word & 0xF);
        clock.d1cpre = decode_hpre((word >> 8) & 0xF);
    }

    /// Push a decoded H7 `D2CFGR` word (`D2PPRE1/2`).
    fn note_d2cfgr(&mut self, word: u32) {
        let Some(clock) = &self.clock else {
            return;
        };
        let mut clock = clock.lock().unwrap_or_else(|e| e.into_inner());
        clock.ppre1 = decode_ppre((word >> 4) & 0x7);
        clock.ppre2 = decode_ppre((word >> 8) & 0x7);
    }
}

impl Behavior for RccBehavior {
    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == CR_OFFSET {
            let hse = if value & HSEON != 0 { HSERDY } else { 0 };
            let pll = if value & PLLON != 0 { PLLRDY } else { 0 };
            let pll2 = if value & PLL2ON != 0 { PLL2RDY } else { 0 };
            let pll3 = if value & PLL3ON != 0 { PLL3RDY } else { 0 };
            let csi = if value & CSION != 0 { CSIRDY } else { 0 };
            value | HSION | self.hsi_rdy | hse | pll | pll2 | pll3 | csi
        } else if offset == CFGR_OFFSET {
            // SW (1:0) selects SYSCLK; SWS (3:2) reports it.
            // HSI always ready, so SWS follows SW instantly.
            let sw = value & 0x3;
            (value & !0xC) | (sw << 2)
        } else {
            value
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        // Decode prescalers from the merged word (byte-lane writes merge
        // with the stored lanes first) so the shared clock tracks firmware.
        if offset == CFGR_OFFSET && !self.h7 {
            let word = view.merged(offset, val, shift);
            self.note_cfgr(word);
            // CFGR is fully writable; SW/SWS handling is in on_read.
            return word;
        }
        if self.h7 && offset == D1CFGR_OFFSET {
            let word = view.merged(offset, val, shift);
            self.note_d1cfgr(word);
            return word;
        }
        if self.h7 && offset == D2CFGR_OFFSET {
            let word = view.merged(offset, val, shift);
            self.note_d2cfgr(word);
            return word;
        }
        view.merged(offset, val, shift)
    }
}

/// The `RCC` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct Rcc {
    inner: RegisterFile<RccBehavior>,
}

impl Rcc {
    /// A new `RCC` at the STM32F1 base address.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `RCC`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        Self::for_device::<stm32f103::Stm32f103>()
    }

    /// A new `RCC` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `RCC` peripheral.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("RCC").expect("generated tables contain RCC");
        // H7 RCC lives at 0x58024400 with HSIRDY at bit2; F1/F4/etc at
        // 0x4002xxxx use bit1. Derive the synthesized bit from the base so
        // we don't OR both shims on every read.
        let h7 = def.base == 0x5802_4400;
        let hsi_rdy = if h7 { HSIRDY_H7 } else { HSIRDY_F1 };
        Self {
            inner: RegisterFile::new(
                def,
                RccBehavior {
                    hsi_rdy,
                    h7,
                    clock: None,
                },
            ),
        }
    }

    /// Share prescaler state with the bus: `CFGR`/`D1CFGR`/`D2CFGR` writes
    /// update `clock`, which the bus reads to scale timer ticks. Boards call
    /// this once and hand the same handle to `SystemMemory::set_clock`.
    #[must_use]
    pub fn with_clock(mut self, clock: Arc<Mutex<ClockState>>) -> Self {
        let h7 = self.inner.behavior().h7;
        self.inner.behavior_mut().clock = Some(clock.clone());
        // The SVD reset words are divider-neutral (/1 everywhere), matching a
        // fresh handle; stamp the family flag in case the handle is reused.
        clock.lock().unwrap_or_else(|e| e.into_inner()).h7 = h7;
        self
    }
}

impl Default for Rcc {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Rcc {
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
        // SVD reset words are divider-neutral; keep the shared handle in
        // lockstep so a rebooted board reboots its clocks too.
        if let Some(clock) = &self.inner.behavior().clock {
            clock
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .reset_dividers();
        }
    }

    fn name(&self) -> &'static str {
        "rcc"
    }
}

impl InterruptSource for Rcc {
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
    use super::{Rcc, HSION, HSIRDY_F1 as HSIRDY, PLLON, PLLRDY};
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4002_1000;
    const CR: u32 = 0x00;
    const APB2ENR: u32 = 0x18;
    const APB1ENR: u32 = 0x1C;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(r: &mut Rcc, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            r.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(r: &mut Rcc, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = r.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn hsi_is_always_on_and_ready() {
        let mut r = Rcc::new();
        assert_eq!(
            read_word(&mut r, BASE + CR) & (HSION | HSIRDY),
            HSION | HSIRDY
        );
    }

    #[test]
    fn pll_locks_instantly_when_enabled() {
        let mut r = Rcc::new();
        write_word(&mut r, BASE + CR, PLLON);
        assert_eq!(read_word(&mut r, BASE + CR) & PLLRDY, PLLRDY);
    }

    #[test]
    fn enable_bits_roundtrip() {
        let mut r = Rcc::new();
        write_word(&mut r, BASE + APB2ENR, 0x0000_4014);
        write_word(&mut r, BASE + APB1ENR, 0x0000_0001);
        assert_eq!(read_word(&mut r, BASE + APB2ENR), 0x0000_4014);
        assert_eq!(read_word(&mut r, BASE + APB1ENR), 0x0000_0001);
    }

    #[test]
    fn reset_restores_svd_values() {
        let mut r = Rcc::new();
        write_word(&mut r, BASE + CR, PLLON);
        write_word(&mut r, BASE + APB2ENR, 0xFFFF_FFFF);
        r.reset();
        // SVD CR reset is 0x83 (HSION|HSIRDY_F1| bit7); H7 would synthesize
        // HSIRDY_H7 (0x84) instead, so F1 reads 0x83.
        assert_eq!(read_word(&mut r, BASE + CR) & 0xFF, 0x83);
        assert_eq!(read_word(&mut r, BASE + APB2ENR), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut r = Rcc::new();
        assert!(matches!(r.read(BASE + 0x28), Err(BusError::Reserved)));
        assert!(matches!(r.write(BASE - 4, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn cfgr_prescalers_decode_into_shared_clock() {
        use super::ClockState;
        let clock = ClockState::shared();
        let mut r = Rcc::new().with_clock(clock.clone());
        // HPRE=/2, PPRE1=/4, PPRE2=/2: (8<<4)|(5<<8)|(4<<11).
        write_word(&mut r, BASE + super::CFGR_OFFSET, 0x2580);
        {
            let clock = clock.lock().unwrap();
            assert_eq!(
                (clock.hpre, clock.d1cpre, clock.ppre1, clock.ppre2),
                (2, 1, 4, 2)
            );
            // Timer ratios relative to core: APB1 2/4, APB2 2/2 (×2 rule).
            assert_eq!(clock.tim_factor(1), (2, 4));
            assert_eq!(clock.tim_factor(2), (2, 2));
        }
        // PPRE1=/1: no ×2 rule, full rate.
        write_word(&mut r, BASE + super::CFGR_OFFSET, 0x80);
        assert_eq!(clock.lock().unwrap().tim_factor(1), (1, 1));
    }

    #[test]
    fn reset_restores_divider_neutral_clocks() {
        use super::ClockState;
        let clock = ClockState::shared();
        let mut r = Rcc::new().with_clock(clock.clone());
        write_word(&mut r, BASE + super::CFGR_OFFSET, 0xFFFF_FFFF);
        assert_ne!(clock.lock().unwrap().ppre1, 1);
        r.reset();
        // NOTE: bind one guard — two `lock()`s in one expression deadlocks
        // a non-reentrant `Mutex`.
        {
            let clock = clock.lock().unwrap();
            assert_eq!((clock.hpre, clock.ppre1), (1, 1));
        }
    }

    #[test]
    fn h7_decodes_d1cfgr_d2cfgr_not_cfgr() {
        use super::ClockState;
        use crate::device::svd::stm32h743::Stm32h743;
        let clock = ClockState::shared();
        let mut r = Rcc::for_device::<Stm32h743>().with_clock(clock.clone());
        // CFGR@0x04 writes must not touch prescalers on H7.
        write_word(&mut r, 0x5802_4400 + super::CFGR_OFFSET, 0xFFFF_FFFF);
        {
            let clock = clock.lock().unwrap();
            assert_eq!((clock.hpre, clock.ppre1), (1, 1));
        }
        // D1CFGR@0x18: HPRE=/2 (nibble 8), D1CPRE=/4 (nibble 9).
        // D2CFGR@0x1C: D2PPRE1=/2, D2PPRE2=/4.
        write_word(&mut r, 0x5802_4400 + super::D1CFGR_OFFSET, 0x908);
        write_word(&mut r, 0x5802_4400 + super::D2CFGR_OFFSET, 0x540);
        let clock = clock.lock().unwrap();
        assert_eq!(
            (clock.hpre, clock.d1cpre, clock.ppre1, clock.ppre2),
            (2, 4, 2, 4)
        );
        // H7 APB1 timer ratio: (d1cpre*2)/(hpre*ppre1) = 8/4.
        assert_eq!(clock.tim_factor(1), (8, 4));
    }
}
