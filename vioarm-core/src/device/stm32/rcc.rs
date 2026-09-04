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

/// `CR` lives at register offset `0x00` on every family.
const CR_OFFSET: u32 = 0x00;
/// F1-family `CFGR` (SW/HPRE/PPRE1/PPRE2/PLLSRC/PLLMUL) at `0x04`.
const CFGR_F1_OFFSET: u32 = 0x04;
/// F4-family `CFGR` (SW/HPRE/PPRE1/PPRE2) at `0x08`; `0x04` is `PLLCFGR`.
const CFGR_F4_OFFSET: u32 = 0x08;
/// F4-family `PLLCFGR` (PLLM/PLLN/PLLP/PLLSRC) at `0x04`.
const PLLCFGR_F4_OFFSET: u32 = 0x04;
/// H7 `CFGR` (SW/SWS/`TIMPRE`) at `0x10`.
const CFGR_H7_OFFSET: u32 = 0x10;
/// G0 `CFGR` (3-bit SW/SWS, `HPRE` at 11:8, single `PPRE` at 14:12) at `0x08`,
/// plus `PLLSYSCFGR` (PLLSRC/PLLM/PLLN/PLLR) at `0x0C`.
const CFGR_G0_OFFSET: u32 = 0x08;
const PLLSYSCFGR_OFFSET: u32 = 0x0C;
/// H7 `D1CFGR`/`D2CFGR` live at `0x18`/`0x1C` (HPRE/D1CPRE/D1PPRE, D2PPRE1/2).
const D1CFGR_OFFSET: u32 = 0x18;
const D2CFGR_OFFSET: u32 = 0x1C;
/// H7 `PLLCKSELR` (PLLSRC/DIVM1) and `PLL1DIVR` (DIVN1/DIVP1).
const PLLCKSELR_OFFSET: u32 = 0x28;
const PLL1DIVR_OFFSET: u32 = 0x30;

/// Fixed oscillator rates: the simulator has no crystal model, so `HSE` is
/// the common 8 MHz part on F-families (F0/F1/F3/F4/G4/L4) and the usual
/// 25 MHz crystal on H7; `CSI` is the H7 4 MHz RC. `HSI` is per-family.
/// Firmware that needs another crystal rate is outside the model.
pub const HSE_HZ: u32 = 8_000_000;
pub const HSE_H7_HZ: u32 = 25_000_000;
pub const CSI_HZ: u32 = 4_000_000;
pub const HSI_F1_HZ: u32 = 8_000_000;
pub const HSI_F4_HZ: u32 = 16_000_000;
pub const HSI_H7_HZ: u32 = 64_000_000;

/// Which `CFGR`/`PLL` register layout (and `HSI` rate) an `RCC` decodes.
/// Detected from the mounted tables in [`Rcc::for_device`]: H7 by base,
/// F4 by the `PLLCFGR` window, G4/L4 by `AHB2ENR` (F4 positions, 16 MHz
/// `HSI`), G0 by `PLLSYSCFGR` (own layout, 16 MHz `HSI`), everything else
/// (F0/F1/F3) by the F1 layout at 8 MHz.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RccFamily {
    /// F0/F1/F3: `CFGR` at `0x04`, `HSI` 8 MHz.
    #[default]
    F1,
    /// F4/G4/L4: `CFGR` at `0x08` + `PLLCFGR`, `HSI` 16 MHz.
    F4,
    /// H7: `CFGR` at `0x10` + `D1CFGR`/`D2CFGR` + PLL1, `HSI` 64 MHz.
    H7,
    /// G0: `CFGR` at `0x08` (HPRE at 11:8, single PPRE) + `PLLSYSCFGR`
    /// (the `R` output drives `SYSCLK`), `HSI` 16 MHz.
    G0,
}

/// AHB/APB prescaler and frequency state decoded from RCC clock-configuration
/// registers, shared with the bus so timer tick scaling tracks firmware's
/// `CFGR` writes.
///
/// On F1/F4-family parts the core runs `HCLK` and `HPRE` cancels out of the
/// timer ratio; on H7 the core runs `cpu_ck = sys_ck/D1CPRE` while timers run
/// off `HCLK`, hence `d1cpre`. Absolute frequencies come from the `SW` source
/// selection plus PLL math over the fixed oscillator rates above; the PLL
/// itself still locks instantly.
///
/// Clock gating of downstream peripherals is deliberately not enforced (see
/// `Known limitations`): every HAL enables clocks before touching a
/// peripheral, and faulting unclocked accesses would churn the 500-board
/// suite for no firmware-visible gain.
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
    /// H7 `CFGR.TIMPRE`: with divided `PPRE`, timers run 4x `PCLK`.
    pub timpre: bool,
    /// `SYSCLK` in Hz, from `SW` + PLL math over the fixed oscillators.
    pub sysclk_hz: u32,
    /// `HSI` rate of the family this handle was stamped for.
    pub hsi_hz: u32,
    h7: bool,
}

impl ClockState {
    /// Reset state: every prescaler divides by 1 and `SW` selects `HSI`,
    /// matching SVD reset words.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            hpre: 1,
            d1cpre: 1,
            ppre1: 1,
            ppre2: 1,
            timpre: false,
            sysclk_hz: HSI_F1_HZ,
            hsi_hz: HSI_F1_HZ,
            h7: false,
        }
    }

    /// A fresh shared handle for one board: the `Rcc` writes it, the bus
    /// reads it. `Send` so it can live on the bus next to the other mailboxes.
    #[must_use]
    pub fn shared() -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(Self::new()))
    }

    /// Restore divider-neutral state, keeping the family stamp: mirrors the
    /// SVD reset words, which reset every prescaler to /1 and `SW` to `HSI`.
    pub fn reset_dividers(&mut self) {
        let (h7, hsi_hz) = (self.h7, self.hsi_hz);
        *self = Self::new();
        self.h7 = h7;
        self.hsi_hz = hsi_hz;
        self.sysclk_hz = hsi_hz;
    }

    /// `(numerator, denominator)` of the APB `apb` (1 or 2) timer-kernel
    /// clock relative to core cycles, including the STM32 ×2 rule (timers
    /// run at twice `PCLKx` when the APB prescaler divides) and H7
    /// `CFGR.TIMPRE` (RM0433: set with a divided `PPRE`, timers run at
    /// `HCLK` for /1-/2-/4, else 4x `PCLK`).
    #[must_use]
    pub fn tim_factor(&self, apb: u8) -> (u64, u64) {
        let ppre = if apb == 2 { self.ppre2 } else { self.ppre1 };
        let d1cpre = u64::from(self.d1cpre.max(1));
        let hpre = u64::from(self.hpre.max(1));
        let ppre = u64::from(ppre.max(1));
        if self.h7 && self.timpre {
            if ppre <= 4 {
                return (d1cpre, hpre);
            }
            return (4 * d1cpre, hpre * ppre);
        }
        let k = if ppre <= 1 { 1u64 } else { 2u64 };
        if self.h7 {
            (d1cpre * k, hpre * ppre)
        } else {
            (k, ppre)
        }
    }

    /// Core clock in Hz (`HCLK` off-H7, `cpu_ck` on H7).
    #[must_use]
    pub fn cpu_hz(&self) -> u32 {
        if self.h7 {
            self.sysclk_hz / self.d1cpre.max(1)
        } else {
            self.hclk_hz()
        }
    }

    /// AHB clock in Hz.
    #[must_use]
    pub fn hclk_hz(&self) -> u32 {
        self.sysclk_hz / self.hpre.max(1)
    }

    /// APB1 clock in Hz.
    #[must_use]
    pub fn pclk1_hz(&self) -> u32 {
        self.hclk_hz() / self.ppre1.max(1)
    }

    /// APB2 clock in Hz.
    #[must_use]
    pub fn pclk2_hz(&self) -> u32 {
        self.hclk_hz() / self.ppre2.max(1)
    }

    /// `(numerator, denominator)` of the `SysTick` rate relative to core
    /// cycles: the processor clock (`HCLK`, `d1cpre`x core on H7), or that
    /// divided by 8 when firmware clears `CLKSOURCE` (`external`).
    #[must_use]
    pub fn systick_factor(&self, external: bool) -> (u64, u64) {
        let num = u64::from(if self.h7 { self.d1cpre.max(1) } else { 1 });
        (num, if external { 8 } else { 1 })
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

/// RCC's synthesis: clock-ready bits in `CR`, `CFGR` SW/SWS, prescaler and
/// PLL decoding into the shared [`ClockState`].
#[derive(Clone, Debug)]
struct RccBehavior {
    hsi_rdy: u32,
    h7: bool,
    family: RccFamily,
    clock: Option<Arc<Mutex<ClockState>>>,
    /// Cached `SW` source selection (F1/F4 2 bits, H7 3 bits).
    sw: u32,
    /// F1-family PLL inputs: `PLLSRC`, `PLLXTPRE`, `PLLMUL`.
    pll_f1: (bool, bool, u32),
    /// F4-family PLL inputs: `PLLM`, `PLLN`, `PLLP` divider, `PLLSRC`.
    pll_f4: (u32, u32, u32, bool),
    /// H7 PLL1 inputs: `PLLSRC`, `DIVM1`, `DIVN1`, `DIVP1`.
    pll_h7: (u32, u32, u32, u32),
    /// G0 PLL inputs: `PLLSRC`, `PLLM` (factor), `PLLN` (factor), `PLLR`
    /// (factor driving `SYSCLK`).
    pll_g0: (u32, u32, u32, u32),
}

impl Default for RccBehavior {
    fn default() -> Self {
        Self {
            hsi_rdy: HSIRDY_F1,
            h7: false,
            family: RccFamily::F1,
            clock: None,
            sw: 0,
            pll_f1: (false, false, 0),
            pll_f4: (16, 192, 2, false),
            pll_h7: (0, 32, 128, 1),
            pll_g0: (1, 1, 16, 1),
        }
    }
}

/// F1 `PLLMUL` field → multiplication factor (×(v+2), ×16 at the top).
fn decode_pllmul(v: u32) -> u32 {
    if v >= 14 {
        16
    } else {
        v + 2
    }
}

impl RccBehavior {
    fn push(&self, f: impl FnOnce(&mut ClockState)) {
        if let Some(clock) = &self.clock {
            f(&mut clock.lock().unwrap_or_else(|e| e.into_inner()));
        }
    }

    /// Recompute `SYSCLK` from the cached source selection + PLL inputs.
    /// Invalid PLL programming (zero dividers, unselected source) falls back
    /// to `HSI` rather than dividing by zero: silicon would misbehave, the
    /// simulator stays observable.
    fn push_sysclk(&self) {
        let sysclk = match self.family {
            RccFamily::F1 => {
                let (pllsrc, pllxtpre, pllmul) = self.pll_f1;
                let hse = HSE_HZ >> u32::from(pllxtpre);
                match self.sw {
                    // F0 PREDIV is assumed /1, so its output is the HSE rate.
                    1 => HSE_HZ,
                    2 => {
                        let src = if pllsrc { hse } else { HSI_F1_HZ / 2 };
                        src.saturating_mul(decode_pllmul(pllmul))
                    }
                    _ => HSI_F1_HZ,
                }
            }
            RccFamily::F4 => {
                let (pllm, plln, pllp, pllsrc) = self.pll_f4;
                match self.sw {
                    1 => HSE_HZ,
                    2 => {
                        let src = if pllsrc { HSE_HZ } else { HSI_F4_HZ };
                        let ref_ck = src / pllm.max(1);
                        ref_ck.saturating_mul(plln) / pllp.max(1)
                    }
                    _ => HSI_F4_HZ,
                }
            }
            RccFamily::H7 => match self.sw {
                // H7 PLL DIVM/DIVN/DIVP hold the factor directly (min 1).
                1 => CSI_HZ,
                2 => HSE_H7_HZ,
                3 => {
                    let (pllsrc, divm, divn, divp) = self.pll_h7;
                    let src = match pllsrc {
                        1 => CSI_HZ,
                        2 => HSE_H7_HZ,
                        _ => HSI_H7_HZ,
                    };
                    let ref_ck = src / divm.max(1);
                    ref_ck.saturating_mul(divn.max(1)) / divp.max(1)
                }
                _ => HSI_H7_HZ,
            },
            RccFamily::G0 => match self.sw {
                // G0 `SYSCLK` runs the PLL `R` output; M/P/R are +1 encoded,
                // N direct (RM0444). LSI fallback is outside the model.
                1 => HSE_HZ,
                2 => {
                    let (pllsrc, m, n, r) = self.pll_g0;
                    let src = match pllsrc {
                        2 => HSE_HZ,
                        _ => HSI_F4_HZ,
                    };
                    let vco = src / m.max(1) * n.max(1);
                    vco / r.max(1)
                }
                _ => HSI_F4_HZ,
            },
        };
        self.push(|clock| clock.sysclk_hz = sysclk);
    }

    /// F1-family `CFGR` at `0x04`: SW/HPRE/PPRE1/PPRE2 + PLL inputs.
    fn note_cfgr_f1(&mut self, word: u32) {
        self.sw = word & 0x3;
        let pll = (
            (word >> 16) & 1 != 0,
            (word >> 17) & 1 != 0,
            (word >> 18) & 0xF,
        );
        self.pll_f1 = pll;
        self.push(|clock| {
            clock.hpre = decode_hpre((word >> 4) & 0xF);
            clock.ppre1 = decode_ppre((word >> 8) & 0x7);
            clock.ppre2 = decode_ppre((word >> 11) & 0x7);
        });
        self.push_sysclk();
    }

    /// F4-family `CFGR` at `0x08`: SW/HPRE/PPRE1/PPRE2 (F4 bit positions).
    fn note_cfgr_f4(&mut self, word: u32) {
        self.sw = word & 0x3;
        self.push(|clock| {
            clock.hpre = decode_hpre((word >> 4) & 0xF);
            clock.ppre1 = decode_ppre((word >> 10) & 0x7);
            clock.ppre2 = decode_ppre((word >> 13) & 0x7);
        });
        self.push_sysclk();
    }

    /// F4-family `PLLCFGR` at `0x04`: PLLM/PLLN/PLLP/PLLSRC. (This offset is
    /// `CFGR` on F1: decoding by family, not offset alone, is what keeps F4
    /// `PLLCFGR` writes from corrupting the prescalers.)
    fn note_pllcfgr_f4(&mut self, word: u32) {
        let pllp = [2, 4, 6, 8][((word >> 16) & 0x3) as usize];
        self.pll_f4 = (
            word & 0x3F,
            (word >> 6) & 0x1FF,
            pllp,
            (word >> 22) & 1 != 0,
        );
        self.push_sysclk();
    }

    /// H7 `CFGR` at `0x10`: 3-bit SW + `TIMPRE`.
    fn note_cfgr_h7(&mut self, word: u32) {
        self.sw = word & 0x7;
        let timpre = (word >> 15) & 1 != 0;
        self.push(|clock| clock.timpre = timpre);
        self.push_sysclk();
    }

    /// G0 `CFGR` at `0x08`: 3-bit SW, `HPRE` at 11:8, one `PPRE` at 14:12
    /// driving both APB domains (G0 has a single APB for timers).
    fn note_cfgr_g0(&mut self, word: u32) {
        self.sw = word & 0x7;
        let ppre = decode_ppre((word >> 12) & 0x7);
        self.push(|clock| {
            clock.hpre = decode_hpre((word >> 8) & 0xF);
            clock.ppre1 = ppre;
            clock.ppre2 = ppre;
        });
        self.push_sysclk();
    }

    /// G0 `PLLSYSCFGR` at `0x0C`: PLLSRC/PLLM(+1)/PLLN/PLLR(+1).
    fn note_pllsyscfgr(&mut self, word: u32) {
        self.pll_g0 = (
            word & 0x3,
            ((word >> 4) & 0x7) + 1,
            (word >> 8) & 0x7F,
            ((word >> 29) & 0x7) + 1,
        );
        self.push_sysclk();
    }

    /// H7 `PLLCKSELR` at `0x28`: PLLSRC/DIVM1.
    fn note_pllckselr(&mut self, word: u32) {
        let (_, _, divn, divp) = self.pll_h7;
        self.pll_h7 = (word & 0x3, (word >> 4) & 0x3F, divn, divp);
        self.push_sysclk();
    }

    /// H7 `PLL1DIVR` at `0x30`: DIVN1/DIVP1.
    fn note_pll1divr(&mut self, word: u32) {
        let (pllsrc, divm, _, _) = self.pll_h7;
        self.pll_h7 = (pllsrc, divm, word & 0x1FF, (word >> 9) & 0x7F);
        self.push_sysclk();
    }

    /// Push a decoded H7 `D1CFGR` word (`HPRE`/`D1CPRE`; `D1PPRE` feeds no
    /// general timer and is intentionally ignored).
    fn note_d1cfgr(&mut self, word: u32) {
        self.push(|clock| {
            clock.hpre = decode_hpre(word & 0xF);
            clock.d1cpre = decode_hpre((word >> 8) & 0xF);
        });
    }

    /// Push a decoded H7 `D2CFGR` word (`D2PPRE1/2`).
    fn note_d2cfgr(&mut self, word: u32) {
        self.push(|clock| {
            clock.ppre1 = decode_ppre((word >> 4) & 0x7);
            clock.ppre2 = decode_ppre((word >> 8) & 0x7);
        });
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
        } else if offset == self.cfgr_offset() {
            // SW selects SYSCLK; SWS reports it. Oscillators/PLL lock
            // instantly, so SWS follows SW on the next read.
            match self.family {
                RccFamily::H7 | RccFamily::G0 => {
                    let sw = value & 0x7;
                    (value & !0x38) | (sw << 3)
                }
                _ => {
                    let sw = value & 0x3;
                    (value & !0xC) | (sw << 2)
                }
            }
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
        // CFGR is fully writable; SW/SWS handling is in on_read.
        let word = view.merged(offset, val, shift);
        match (self.family, offset) {
            (RccFamily::F1, CFGR_F1_OFFSET) => self.note_cfgr_f1(word),
            (RccFamily::F4, CFGR_F4_OFFSET) => self.note_cfgr_f4(word),
            (RccFamily::F4, PLLCFGR_F4_OFFSET) => self.note_pllcfgr_f4(word),
            (RccFamily::H7, CFGR_H7_OFFSET) => self.note_cfgr_h7(word),
            (RccFamily::H7, D1CFGR_OFFSET) => self.note_d1cfgr(word),
            (RccFamily::H7, D2CFGR_OFFSET) => self.note_d2cfgr(word),
            (RccFamily::H7, PLLCKSELR_OFFSET) => self.note_pllckselr(word),
            (RccFamily::H7, PLL1DIVR_OFFSET) => self.note_pll1divr(word),
            (RccFamily::G0, CFGR_G0_OFFSET) => self.note_cfgr_g0(word),
            (RccFamily::G0, PLLSYSCFGR_OFFSET) => self.note_pllsyscfgr(word),
            _ => {}
        }
        word
    }
}

impl RccBehavior {
    /// The `CFGR` offset for this family (also keyed in `on_read`).
    const fn cfgr_offset(&self) -> u32 {
        match self.family {
            RccFamily::F1 => CFGR_F1_OFFSET,
            RccFamily::F4 => CFGR_F4_OFFSET,
            RccFamily::H7 => CFGR_H7_OFFSET,
            RccFamily::G0 => CFGR_G0_OFFSET,
        }
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
        // `CFGR` layout follows the family: H7 by base, F4 by its `PLLCFGR`
        // window, G4/L4 by `AHB2ENR` (F4 positions, 16 MHz `HSI`), G0 by
        // `PLLSYSCFGR` (own layout, 16 MHz `HSI`), the rest (F0/F1/F3) by
        // the F1 layout at 8 MHz.
        let has = |name: &str| def.registers.iter().any(|r| r.name == name);
        let family = if h7 {
            RccFamily::H7
        } else if has("PLLSYSCFGR") {
            RccFamily::G0
        } else if has("PLLCFGR") || has("AHB2ENR") {
            RccFamily::F4
        } else {
            RccFamily::F1
        };
        Self {
            inner: RegisterFile::new(
                def,
                RccBehavior {
                    hsi_rdy,
                    h7,
                    family,
                    ..RccBehavior::default()
                },
            ),
        }
    }

    /// Share prescaler state with the bus: `CFGR`/`D1CFGR`/`D2CFGR`/PLL writes
    /// update `clock`, which the bus reads to scale timer ticks. Boards call
    /// this once and hand the same handle to `SystemMemory::set_clock`.
    #[must_use]
    pub fn with_clock(mut self, clock: Arc<Mutex<ClockState>>) -> Self {
        let behavior = self.inner.behavior();
        let (h7, family) = (behavior.h7, behavior.family);
        self.inner.behavior_mut().clock = Some(clock.clone());
        // The SVD reset words are divider-neutral (/1 everywhere, `SW` =
        // `HSI`), matching a fresh handle; stamp the family in case the
        // handle is reused.
        let hsi_hz = match family {
            RccFamily::F1 => HSI_F1_HZ,
            RccFamily::F4 | RccFamily::G0 => HSI_F4_HZ,
            RccFamily::H7 => HSI_H7_HZ,
        };
        let mut clock = clock.lock().unwrap_or_else(|e| e.into_inner());
        clock.h7 = h7;
        clock.hsi_hz = hsi_hz;
        clock.sysclk_hz = hsi_hz;
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
        write_word(&mut r, BASE + super::CFGR_F1_OFFSET, 0x2580);
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
        write_word(&mut r, BASE + super::CFGR_F1_OFFSET, 0x80);
        assert_eq!(clock.lock().unwrap().tim_factor(1), (1, 1));
    }

    #[test]
    fn reset_restores_divider_neutral_clocks() {
        use super::ClockState;
        let clock = ClockState::shared();
        let mut r = Rcc::new().with_clock(clock.clone());
        write_word(&mut r, BASE + super::CFGR_F1_OFFSET, 0xFFFF_FFFF);
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
        write_word(&mut r, 0x5802_4400 + super::CFGR_F1_OFFSET, 0xFFFF_FFFF);
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

    #[test]
    fn f1_pll_programs_72mhz() {
        use super::ClockState;
        let clock = ClockState::shared();
        let mut r = Rcc::new().with_clock(clock.clone());
        // HSE on, PLL x9 (PLLMUL=7), SW=PLL.
        write_word(&mut r, BASE + CR, PLLON);
        assert_eq!(read_word(&mut r, BASE + CR) & PLLRDY, PLLRDY);
        write_word(&mut r, BASE + 0x04, 0x001D_0002);
        let c = clock.lock().unwrap();
        assert_eq!(c.sysclk_hz, 72_000_000);
        assert_eq!(c.cpu_hz(), 72_000_000);
        assert_eq!(
            (c.hclk_hz(), c.pclk1_hz(), c.pclk2_hz()),
            (72_000_000, 72_000_000, 72_000_000)
        );
    }

    #[test]
    fn f4_pll_programs_168mhz_and_pllcfgr_does_not_touch_dividers() {
        use super::ClockState;
        use crate::device::svd::stm32f407::Stm32f407;
        let clock = ClockState::shared();
        let mut r = Rcc::for_device::<Stm32f407>().with_clock(clock.clone());
        const F4: u32 = 0x4002_3800;
        // SW=PLL, PPRE1=/2 via the real F4 CFGR@0x08.
        write_word(&mut r, F4 + 0x08, 0x0000_1002);
        // PLLCFGR@0x04: PLLM=8, PLLN=336, PLLP=/2, PLLSRC=HSE.
        write_word(&mut r, F4 + 0x04, 8 | (336 << 6) | (1 << 22));
        let c = clock.lock().unwrap();
        // The PLLCFGR low bits must not decode as HPRE/PPRE (old offset bug).
        assert_eq!((c.hpre, c.ppre1, c.ppre2), (1, 2, 1));
        assert_eq!(c.sysclk_hz, 168_000_000);
        assert_eq!(c.pclk1_hz(), 84_000_000);
        // Timer kernel on divided APB1 runs at 2x PCLK1 = SYSCLK here.
        assert_eq!(c.tim_factor(1), (2, 2));
    }

    #[test]
    fn h7_timpre_selects_x4_and_sws_follows_sw() {
        use super::ClockState;
        use crate::device::svd::stm32h743::Stm32h743;
        const H7: u32 = 0x5802_4400;
        let clock = ClockState::shared();
        let mut r = Rcc::for_device::<Stm32h743>().with_clock(clock.clone());
        // D2PPRE1=/8, D1 dividers /1: TIMPRE=0 -> 2/8, TIMPRE=1 -> 4/8.
        write_word(&mut r, H7 + super::D2CFGR_OFFSET, 0x60);
        write_word(&mut r, H7 + super::CFGR_H7_OFFSET, 0);
        assert_eq!(clock.lock().unwrap().tim_factor(1), (2, 8));
        write_word(&mut r, H7 + super::CFGR_H7_OFFSET, 1 << 15);
        assert_eq!(clock.lock().unwrap().tim_factor(1), (4, 8));
        // SW=PLL1 reports back through SWS.
        write_word(&mut r, H7 + super::CFGR_H7_OFFSET, (1 << 15) | 3);
        assert_eq!(read_word(&mut r, H7 + super::CFGR_H7_OFFSET) & 0x38, 0x18);
    }

    #[test]
    fn h7_pll1_p_programs_sysclk() {
        use super::ClockState;
        use crate::device::svd::stm32h743::Stm32h743;
        const H7: u32 = 0x5802_4400;
        let clock = ClockState::shared();
        let mut r = Rcc::for_device::<Stm32h743>().with_clock(clock.clone());
        // HSE 25 MHz / DIVM1=25 = 1 MHz ref, x DIVN1=240, / DIVP1=1.
        write_word(&mut r, H7 + super::PLLCKSELR_OFFSET, 2 | (25 << 4));
        write_word(&mut r, H7 + super::PLL1DIVR_OFFSET, 240);
        write_word(&mut r, H7 + super::CFGR_H7_OFFSET, 3);
        assert_eq!(clock.lock().unwrap().sysclk_hz, 240_000_000);
    }

    #[test]
    fn g0_pll_r_programs_sysclk() {
        use super::ClockState;
        use crate::device::svd::stm32g081::Stm32g081;
        const G0: u32 = 0x4002_1000;
        let clock = ClockState::shared();
        let mut r = Rcc::for_device::<Stm32g081>().with_clock(clock.clone());
        assert_eq!(clock.lock().unwrap().sysclk_hz, super::HSI_F4_HZ);
        // HSI 16 MHz / M=1 x N=16 / R=2 = 128 MHz. SW=PLL.
        write_word(&mut r, G0 + 0x0C, 1 | (16 << 8) | (1 << 29));
        write_word(&mut r, G0 + 0x08, 2);
        let c = clock.lock().unwrap();
        assert_eq!(c.sysclk_hz, 128_000_000);
        assert_eq!(
            (c.cpu_hz(), c.pclk1_hz(), c.pclk2_hz()),
            (128_000_000, 128_000_000, 128_000_000)
        );
    }

    #[test]
    fn reset_returns_to_hsi() {
        use super::ClockState;
        let clock = ClockState::shared();
        let mut r = Rcc::new().with_clock(clock.clone());
        write_word(&mut r, BASE + 0x04, 0x001D_0002);
        assert_eq!(clock.lock().unwrap().sysclk_hz, 72_000_000);
        r.reset();
        assert_eq!(clock.lock().unwrap().sysclk_hz, super::HSI_F1_HZ);
    }
}
