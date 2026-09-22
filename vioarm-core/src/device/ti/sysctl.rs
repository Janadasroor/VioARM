//! TI Tiva C System Control and clocks (`SYSCTL`, `0x400F_E000`).
//!
//! Register map, resets, and clock math follow the TM4C123GH6PM datasheet
//! (SPMS376E §5, Table 5-7 for resets; Tables 5-4/5-5/5-6 for the PLL
//! divisor math). The SVD carries no `<resetValue>` and marks every window
//! `ReadWrite`, so this behavior stamps the datasheet resets at construction
//! and enforces the Table 5-7 RO/RW types on write.
//!
//! Modeled: `RCC`/`RCC2` (PLL math → `sysclk_hz`, pushed into the shared
//! [`ClockState`]), PLL lock after [`PLL_LOCK_TICKS`] device ticks
//! (`RIS.PLLLRIS` bit 6 + `PLLSTAT.LOCK` bit 0; the millisecond-scale analog
//! lock is unobservable to polling firmware, same convention as
//! [`crate::device::stm32::rcc`]), `RCGCn` gating with `PRn` ready synthesis
//! (`PR = PP & RCGC`, immediate — real silicon lags a few clocks, TivaWare
//! only polls), RO ID/capability constants (`DID0` modeled `0x1805_0001`:
//! `VER=1`, `CLASS=0x05` per datasheet, die revision A1; `DID1`/`DC0-DC9`
//! from Table 5-7), `RESC` power-on (`POR` set), `MISC` W1C latch.
//! Deliberate approximations: `MOSC` ready is immediate once `MOSCDIS`
//! clears; `LFIOSC` reads nominal 30 kHz (datasheet: wide variations);
//! unknown `XTAL` encodings fall back to 16 MHz; `SYSDIV` below the
//! `MINSYSDIV` floor is applied as-written (reserved use on silicon).

use crate::device::stm32::rcc::ClockState;
use crate::device::svd::tm4c123gh6pm;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// `SYSCTL` base address.
pub const SYSCTL_BASE: u32 = 0x400F_E000;
/// Register offsets.
const DID0_OFF: u32 = 0x000;
const DID1_OFF: u32 = 0x004;
const RIS_OFF: u32 = 0x050;
const IMC_OFF: u32 = 0x054;
const MISC_OFF: u32 = 0x058;
const RESC_OFF: u32 = 0x05C;
const RCC_OFF: u32 = 0x060;
const GPIOHBCTL_OFF: u32 = 0x06C;
const RCC2_OFF: u32 = 0x070;
const MOSCCTL_OFF: u32 = 0x07C;
const DSLPCLKCFG_OFF: u32 = 0x144;
const PIOSCCAL_OFF: u32 = 0x150;
const PLLSTAT_OFF: u32 = 0x168;
const SLPPWRCFG_OFF: u32 = 0x188;
const DSLPPWRCFG_OFF: u32 = 0x18C;
const LDOSPCTL_OFF: u32 = 0x1B4;
const LDODPCTL_OFF: u32 = 0x1BC;
const PP_BASE_OFF: u32 = 0x300;
const SR_BASE_OFF: u32 = 0x500;
const RCGC_BASE_OFF: u32 = 0x600;
const SCGC_BASE_OFF: u32 = 0x700;
const DCGC_BASE_OFF: u32 = 0x800;
const PR_BASE_OFF: u32 = 0xA00;
/// Module-group offsets within each `PP`/`SR`/`RCGC`/`SCGC`/`DCGC`/`PR`
/// block (the blocks are not contiguous: USB/CAN/EEPROM groups leave gaps).
/// Order: WD, TIMER, GPIO, DMA, HIB, UART, SSI, I2C, USB, CAN, ADC, ACMP,
/// PWM, QEI, EEPROM, WTIMER.
const GROUP_OFFS: &[u32] = &[
    0x00, 0x04, 0x08, 0x0C, 0x14, 0x18, 0x1C, 0x20, 0x28, 0x34, 0x38, 0x3C, 0x40, 0x44, 0x58, 0x5C,
];
/// Group indices into [`GROUP_OFFS`].
const GROUP_GPIO: usize = 2;
const GROUP_UART: usize = 5;

/// `RCC`: `USESYSDIV` bit 22, `BYPASS` bit 11, `PWRDN` bit 13.
const RCC_USESYSDIV: u32 = 1 << 22;
const RCC_BYPASS: u32 = 1 << 11;
const RCC_PWRDN: u32 = 1 << 13;
/// `RCC`: `SYSDIV` bits 26:23, `OSCSRC` bits 5:4, `XTAL` bits 10:6.
const RCC_SYSDIV_SHIFT: u32 = 23;
const RCC_SYSDIV_MASK: u32 = 0x0F;
const RCC_OSCSRC_SHIFT: u32 = 4;
const RCC_OSCSRC_MASK: u32 = 0x03;
const RCC_XTAL_SHIFT: u32 = 6;
const RCC_XTAL_MASK: u32 = 0x1F;
/// `RCC2`: `USERCC2` bit 31, `DIV400` bit 30, `SYSDIV2LSB` bit 22.
const RCC2_USERCC2: u32 = 1 << 31;
const RCC2_DIV400: u32 = 1 << 30;
const RCC2_SYSDIV2LSB: u32 = 1 << 22;
/// `RCC`/`RCC2`: shared bit positions (`BYPASS` 11, `PWRDN` 13,
/// `SYSDIV`/`SYSDIV2` LSB 23, `OSCSRC`/`OSCSRC2` LSB 4).
const BYPASS_BIT: u32 = 1 << 11;
const PWRDN_BIT: u32 = 1 << 13;
const SYSDIV_LSB: u32 = 23;
const RCC2_SYSDIV2_MASK: u32 = 0x3F;
const OSCSRC_SHIFT: u32 = 4;
const RCC_MOSCDIS: u32 = 1 << 0;
/// `RIS.PLLLRIS` bit 6; `PLLSTAT.LOCK` bit 0.
const RIS_PLLLRIS: u32 = 1 << 6;
const PLLSTAT_LOCK: u32 = 1 << 0;
/// `RESC.POR` bit 1 (power-on reset cause, sticky).
const RESC_POR: u32 = 1 << 1;
/// Modeled PLL lock delay in device ticks (same convention as STM32 `Rcc`).
const PLL_LOCK_TICKS: u32 = 128;
/// `PIOSC` rate in Hz (datasheet §5.2.5.1: 16 MHz).
pub const PIOSC_HZ: u32 = 16_000_000;
/// Nominal `LFIOSC` rate in Hz (datasheet: wide variations; TivaWare nominal).
pub const LFIOSC_HZ: u32 = 30_000;
/// 32.768 kHz Hibernate-module oscillator in Hz.
pub const HIB_OSC_HZ: u32 = 32_768;
/// PLL VCO frequency in Hz (datasheet §5.2.5.3: 400 MHz, predivided by 2
/// to 200 MHz unless `DIV400` selects the 400 MHz output directly).
const PLL_VCO_HZ: u64 = 400_000_000;
/// Maximum TM4C123 system clock in Hz (80 MHz part).
const SYSCLK_MAX_HZ: u32 = 80_000_000;

/// `DID0` model: `VER=1`, `CLASS=0x05` (datasheet), die revision A1
/// (`MAJOR=0`, `MINOR=1` — a modeled choice, silicon varies by lot).
const DID0_MODEL: u32 = 0x1805_0001;
/// `DID1` reset (Table 5-7).
const DID1_RESET: u32 = 0x10A1_606E;
/// `RCC`/`RCC2` resets (Table 5-7).
const RCC_RESET: u32 = 0x078E_3AD1;
const RCC2_RESET: u32 = 0x07C0_6810;
/// `GPIOHBCTL` reset (Table 5-7: ports K-N/P-Q AHB-only bits).
const GPIOHBCTL_RESET: u32 = 0x0000_7E00;
/// `DSLPCLKCFG` reset (Table 5-7).
const DSLPCLKCFG_RESET: u32 = 0x0780_0000;

/// Crystal frequency in Hz for each `RCC.XTAL` encoding (datasheet RCC
/// register page). Encodings not listed (reserved) fall back to 16 MHz.
const XTAL_HZ: &[(u32, u32)] = &[
    (0x06, 4_000_000),
    (0x07, 4_096_000),
    (0x08, 4_915_200),
    (0x09, 5_000_000),
    (0x0A, 5_120_000),
    (0x0B, 6_000_000),
    (0x0C, 6_144_000),
    (0x0D, 7_372_800),
    (0x0E, 8_000_000),
    (0x0F, 8_192_000),
    (0x10, 10_000_000),
    (0x11, 12_000_000),
    (0x12, 12_288_000),
    (0x13, 13_560_000),
    (0x14, 14_318_180),
    (0x15, 16_000_000),
    (0x16, 16_384_000),
    (0x17, 18_000_000),
    (0x18, 20_000_000),
    (0x19, 24_000_000),
    (0x1A, 25_000_000),
];

/// `(offset, datasheet reset)` for every `SYSCTL` register the behavior
/// stamps at construction. RO ID/capability words come from Table 5-7;
/// `PRn`/`RIS`/`PLLSTAT`/`MISC` are synthesized (stamped 0 here).
const RESETS: &[(u32, u32)] = &[
    (DID0_OFF, DID0_MODEL),
    (DID1_OFF, DID1_RESET),
    (0x008, 0x007F_007F), // DC0
    (0x010, 0x1333_2FFF), // DC1
    (0x014, 0x030F_F337), // DC2
    (0x018, 0xBFFF_8FFF), // DC3
    (0x01C, 0x0004_F03F), // DC4
    (0x020, 0x0130_00FF), // DC5
    (0x024, 0x0000_0013), // DC6
    (0x028, 0xFFFF_FFFF), // DC7
    (0x02C, 0x0FFF_0FFF), // DC8
    (0x030, 0x0000_7FFF), // PBORCTL
    (RIS_OFF, 0),
    (IMC_OFF, 0),
    (MISC_OFF, 0),
    (RESC_OFF, RESC_POR),
    (RCC_OFF, RCC_RESET),
    (GPIOHBCTL_OFF, GPIOHBCTL_RESET),
    (RCC2_OFF, RCC2_RESET),
    (MOSCCTL_OFF, 0),
    (0x100, 0x0000_0040), // RCGC0 (legacy RO)
    (0x104, 0),           // RCGC1 (legacy RO)
    (0x108, 0),           // RCGC2 (legacy RO)
    (0x110, 0x0000_0040), // SCGC0 (legacy RO)
    (0x114, 0),           // SCGC1 (legacy RO)
    (0x118, 0),           // SCGC2 (legacy RO)
    (0x120, 0x0000_0040), // DCGC0 (legacy RO)
    (0x124, 0),           // DCGC1 (legacy RO)
    (0x128, 0),           // DCGC2 (legacy RO)
    (DSLPCLKCFG_OFF, DSLPCLKCFG_RESET),
    (0x14C, 0x0000_1D31), // SYSPROP
    (PIOSCCAL_OFF, 0),
    (0x154, 0x0000_0040), // PIOSCSTAT
    (0x160, 0x0000_0032), // PLLFREQ0
    (0x164, 0x0000_0001), // PLLFREQ1
    (PLLSTAT_OFF, 0),
    (SLPPWRCFG_OFF, 0),
    (DSLPPWRCFG_OFF, 0),
    (0x190, 0x00FF_00FF), // DC9
    (0x1A0, 0x0000_0001), // NVMSTAT
    (LDOSPCTL_OFF, 0x0000_0018),
    (0x1B8, 0x0000_1818), // LDOSPCAL
    (LDODPCTL_OFF, 0x0000_0012),
    (0x1C0, 0x0000_1212), // LDODPCAL
    (0x1CC, 0),           // SDPMST
    (0x300, 0x0000_0003), // PPWD
    (0x304, 0x0000_003F), // PPTIMER
    (0x308, 0x0000_003F), // PPGPIO
    (0x30C, 0x0000_0001), // PPDMA
    (0x314, 0x0000_0001), // PPHIB
    (0x318, 0x0000_00FF), // PPUART
    (0x31C, 0x0000_000F), // PPSSI
    (0x320, 0x0000_000F), // PPI2C
    (0x328, 0x0000_0001), // PPUSB
    (0x334, 0x0000_0003), // PPCAN
    (0x338, 0x0000_0003), // PPADC
    (0x33C, 0x0000_0001), // PPACMP
    (0x340, 0x0000_0003), // PPPWM
    (0x344, 0x0000_0003), // PPQEI
    (0x358, 0x0000_0001), // PPEEPROM
    (0x35C, 0x0000_003F), // PPWTIMER
    (0x614, 0x0000_0001), // RCGCHIB (HIB clocked out of reset)
    (0x714, 0x0000_0001), // SCGCHIB
    (0x814, 0x0000_0001), // DCGCHIB
];

/// Returns `true` for offsets firmware may write (Table 5-7 `RW` type).
/// Everything else (`RO` ID/capability/status, `W1C` MISC handled
/// separately) ignores writes, like silicon.
fn is_writable(offset: u32) -> bool {
    // Gating, reset, and clock-config registers are RW.
    if offset == IMC_OFF
        || offset == RESC_OFF
        || offset == RCC_OFF
        || offset == GPIOHBCTL_OFF
        || offset == RCC2_OFF
        || offset == MOSCCTL_OFF
        || offset == DSLPCLKCFG_OFF
        || offset == PIOSCCAL_OFF
        || offset == SLPPWRCFG_OFF
        || offset == DSLPPWRCFG_OFF
        || offset == LDOSPCTL_OFF
        || offset == LDODPCTL_OFF
        || offset == 0x030
    {
        return true;
    }
    // `SRn`/`RCGCn`/`SCGCn`/`DCGCn` groups are RW words.
    for base in [SR_BASE_OFF, RCGC_BASE_OFF, SCGC_BASE_OFF, DCGC_BASE_OFF] {
        if GROUP_OFFS.iter().any(|g| base + g == offset) {
            return true;
        }
    }
    false
}

/// Compute `SYSCLK` in Hz from the `RCC`/`RCC2` words (datasheet §5.2.5.3,
/// Tables 5-4/5-5/5-6). `locked` gates the PLL path: an unpowered or
/// not-yet-locked PLL cannot source the clock, so the raw oscillator feeds
/// through (firmware always bypasses first, per the 5-step procedure).
#[must_use]
pub fn sysclk_hz_for(rcc: u32, rcc2: u32, locked: bool) -> u32 {
    let usercc2 = rcc2 & RCC2_USERCC2 != 0;
    let cfg = if usercc2 { rcc2 } else { rcc };
    let bypass = cfg & BYPASS_BIT != 0;
    let pwrdn = cfg & PWRDN_BIT != 0;
    let usesysdiv = rcc & RCC_USESYSDIV != 0;
    let div400 = usercc2 && (rcc2 & RCC2_DIV400 != 0);
    let sysdiv = if div400 {
        let s2 = (rcc2 >> SYSDIV_LSB) & RCC2_SYSDIV2_MASK;
        let lsb = (rcc2 & RCC2_SYSDIV2LSB) >> 22;
        (s2 << 1) | lsb
    } else if usercc2 {
        (rcc2 >> SYSDIV_LSB) & RCC2_SYSDIV2_MASK
    } else {
        (rcc >> RCC_SYSDIV_SHIFT) & RCC_SYSDIV_MASK
    };
    let divisor = u64::from(sysdiv) + 1;
    let pll_selected = !bypass && !pwrdn && locked;
    if pll_selected {
        // The divider is forced on when the PLL sources the clock.
        let base = if div400 { PLL_VCO_HZ } else { PLL_VCO_HZ / 2 };
        let hz = base / divisor;
        return u32::try_from(hz.min(u64::from(SYSCLK_MAX_HZ))).unwrap_or(SYSCLK_MAX_HZ);
    }
    let oscsrc_mask = if usercc2 { 0x07 } else { 0x03 };
    let oscsrc = (cfg >> OSCSRC_SHIFT) & oscsrc_mask;
    let xtal = (rcc >> RCC_XTAL_SHIFT) & RCC_XTAL_MASK;
    let src = match oscsrc {
        0 => XTAL_HZ
            .iter()
            .find(|(enc, _)| *enc == xtal)
            .map_or(PIOSC_HZ, |(_, hz)| *hz),
        1 => PIOSC_HZ,
        2 => PIOSC_HZ / 4,
        3 => LFIOSC_HZ,
        _ => HIB_OSC_HZ,
    };
    if usesysdiv {
        u32::try_from(u64::from(src) / divisor).unwrap_or(u32::MAX)
    } else {
        src
    }
}

/// Behavior behind the `SYSCTL` window: datasheet resets, RO enforcement,
/// PLL lock timing, `PR` synthesis, and `sysclk_hz` pushes.
#[derive(Debug)]
pub struct SysctlBehavior {
    clock: Option<Arc<Mutex<ClockState>>>,
    pll_lockdown: u32,
    locked: bool,
    sysclk: u32,
}

impl Default for SysctlBehavior {
    fn default() -> Self {
        Self {
            clock: None,
            pll_lockdown: 0,
            locked: false,
            sysclk: PIOSC_HZ,
        }
    }
}

impl SysctlBehavior {
    /// Recompute lock state and `sysclk` from the current `RCC`/`RCC2` words.
    fn recompute(&mut self, view: &mut Registers<&mut [u32]>) {
        let rcc = view.get(RCC_OFF);
        let rcc2 = view.get(RCC2_OFF);
        let usercc2 = rcc2 & RCC2_USERCC2 != 0;
        let cfg = if usercc2 { rcc2 } else { rcc };
        let powered = cfg & PWRDN_BIT == 0;
        if !powered {
            self.locked = false;
            self.pll_lockdown = 0;
        } else if !self.locked && self.pll_lockdown == 0 {
            self.pll_lockdown = PLL_LOCK_TICKS;
        }
        self.sysclk = sysclk_hz_for(rcc, rcc2, self.locked);
        self.push_clock();
    }

    fn push_clock(&self) {
        if let Some(clock) = &self.clock {
            if let Ok(mut c) = clock.lock() {
                c.sysclk_hz = self.sysclk;
                c.hsi_hz = PIOSC_HZ;
            }
        }
    }

    /// `PR` group word: present-and-clocked modules (`PP & RCGC`).
    fn pr_word(view: &Registers<&mut [u32]>, group: usize) -> u32 {
        let off = GROUP_OFFS[group];
        let pp = view.get(PP_BASE_OFF + off);
        let rcgc = view.get(RCGC_BASE_OFF + off);
        pp & rcgc
    }
}

impl Behavior for SysctlBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if self.pll_lockdown == 0 {
            return;
        }
        let dt = u32::try_from(delta).unwrap_or(u32::MAX);
        self.pll_lockdown = self.pll_lockdown.saturating_sub(dt);
        if self.pll_lockdown == 0 {
            self.locked = true;
            let rcc = view.get(RCC_OFF);
            let rcc2 = view.get(RCC2_OFF);
            self.sysclk = sysclk_hz_for(rcc, rcc2, true);
            self.push_clock();
        }
    }

    fn reset(&mut self) {
        self.pll_lockdown = 0;
        self.locked = false;
        self.sysclk = PIOSC_HZ;
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == RIS_OFF {
            return if self.locked {
                value | RIS_PLLLRIS
            } else {
                value & !RIS_PLLLRIS
            };
        }
        if offset == PLLSTAT_OFF {
            return if self.locked {
                value | PLLSTAT_LOCK
            } else {
                value & !PLLSTAT_LOCK
            };
        }
        if offset == MISC_OFF {
            return 0;
        }
        if PR_BASE_OFF <= offset {
            if let Some(group) = GROUP_OFFS.iter().position(|g| PR_BASE_OFF + g == offset) {
                return Self::pr_word(view, group);
            }
        }
        value
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == MISC_OFF {
            // Write-1-to-clear interrupt status; nothing latches here (no
            // BOR events in simulation), so the write sinks.
            return view.get(offset);
        }
        if !is_writable(offset) {
            return view.get(offset);
        }
        let word = view.merged(offset, val, shift);
        if offset == RCC_OFF || offset == RCC2_OFF {
            // Recompute against the about-to-land word: stage it, refresh
            // lock/sysclk, then report the staged word.
            view.set(offset, word);
            self.recompute(view);
        }
        word
    }
}

/// The Tiva C `SYSCTL` peripheral.
pub struct Sysctl {
    inner: RegisterFile<SysctlBehavior>,
}

impl Sysctl {
    /// A new `SYSCTL` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain `SYSCTL`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("SYSCTL").expect("generated tables contain SYSCTL");
        let mut sys = Self {
            inner: RegisterFile::new(def, SysctlBehavior::default()),
        };
        sys.stamp_resets();
        sys
    }

    /// Stamp the datasheet (Table 5-7) reset words over the SVD zeros.
    fn stamp_resets(&mut self) {
        let def = self.inner.def();
        let idx_of = |offset: u32| def.registers.iter().position(|r| r.offset == offset);
        let rcc_idx = idx_of(RCC_OFF).expect("SYSCTL has RCC");
        let rcc2_idx = idx_of(RCC2_OFF).expect("SYSCTL has RCC2");
        let stamped: Vec<(usize, u32)> = RESETS
            .iter()
            .filter_map(|&(offset, value)| idx_of(offset).map(|idx| (idx, value)))
            .collect();
        let words = self.inner.regs_mut();
        for (idx, value) in stamped {
            words[idx] = value;
        }
        let (rcc, rcc2) = (words[rcc_idx], words[rcc2_idx]);
        let behavior = self.inner.behavior_mut();
        behavior.sysclk = sysclk_hz_for(rcc, rcc2, false);
    }

    /// Current `SYSCLK` in Hz, from the last `RCC`/`RCC2` write or lock event.
    #[must_use]
    pub fn sysclk_hz(&self) -> u32 {
        self.inner.behavior().sysclk
    }

    /// Share clock state with the bus: `RCC`/`RCC2` writes and lock events
    /// update `sysclk_hz`, stamped for the Tiva `PIOSC` source.
    #[must_use]
    pub fn with_clock(mut self, clock: Arc<Mutex<ClockState>>) -> Self {
        self.inner.behavior_mut().clock = Some(clock.clone());
        if let Ok(mut c) = clock.lock() {
            c.hsi_hz = PIOSC_HZ;
            c.sysclk_hz = self.inner.behavior().sysclk;
        }
        self
    }
}

impl Default for Sysctl {
    fn default() -> Self {
        Self::for_device::<tm4c123gh6pm::Tm4c123gh6pm>()
    }
}

impl Peripheral for Sysctl {
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
        // SVD zeros would wipe the datasheet words: reload, then re-stamp.
        self.inner.reset();
        self.stamp_resets();
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
}

impl InterruptSource for Sysctl {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sysctl() -> Sysctl {
        Sysctl::for_device::<tm4c123gh6pm::Tm4c123gh6pm>()
    }

    fn word(sys: &mut Sysctl, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(sys.read(SYSCTL_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(sys: &mut Sysctl, offset: u32, value: u32) {
        for lane in 0..4 {
            sys.write(
                SYSCTL_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Datasheet Table 5-7 reset words land on the bus.
    #[test]
    fn datasheet_resets() {
        let mut sys = sysctl();
        assert_eq!(word(&mut sys, RCC_OFF), RCC_RESET);
        assert_eq!(word(&mut sys, RCC2_OFF), RCC2_RESET);
        assert_eq!(word(&mut sys, DID1_OFF), DID1_RESET);
        assert_eq!(word(&mut sys, DID0_OFF), DID0_MODEL);
        assert_eq!(word(&mut sys, GPIOHBCTL_OFF), GPIOHBCTL_RESET);
        assert_eq!(word(&mut sys, RESC_OFF), RESC_POR);
        // `PRn` read 0 with no clocks (except always-ready `PRHIB`).
        assert_eq!(word(&mut sys, PR_BASE_OFF + GROUP_OFFS[GROUP_GPIO]), 0);
        assert_eq!(word(&mut sys, PR_BASE_OFF + GROUP_OFFS[4]), 1);
    }

    /// RO ID/capability words ignore writes (Table 5-7 types).
    #[test]
    fn ro_words_ignore_writes() {
        let mut sys = sysctl();
        write_word(&mut sys, DID1_OFF, 0xFFFF_FFFF);
        assert_eq!(word(&mut sys, DID1_OFF), DID1_RESET);
        write_word(&mut sys, 0x308, 0); // PPGPIO
        assert_eq!(word(&mut sys, 0x308), 0x0000_003F);
        write_word(&mut sys, RIS_OFF, 0xFFFF_FFFF);
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
    }

    /// Power-on reset runs on `PIOSC`: 16 MHz undivided (`BYPASS=1`,
    /// `USESYSDIV=0` out of reset).
    #[test]
    fn reset_clock_is_piosc_16mhz() {
        let sys = sysctl();
        assert_eq!(sys.sysclk_hz(), 16_000_000);
    }

    /// Datasheet Table 5-4 spot: `SYSDIV=0x2` → 200 MHz / 3.
    #[test]
    fn sysdiv_0x2_is_66mhz() {
        // RCC path: BYPASS=0, PWRDN=0, USESYSDIV=1, SYSDIV=0x2.
        let rcc = (0x02 << RCC_SYSDIV_SHIFT) | RCC_USESYSDIV;
        assert_eq!(sysclk_hz_for(rcc, 0, true), 66_666_666);
    }

    /// The classic 80 MHz TivaWare setup (`DIV400`, `SYSDIV2=0x2`,
    /// 16 MHz crystal) divides 400 MHz by 5.
    #[test]
    fn div400_sysdiv2_0x2_is_80mhz() {
        let rcc = RCC_USESYSDIV | (0x15 << RCC_XTAL_SHIFT); // MOSC 16 MHz
        let rcc2 = RCC2_USERCC2 | RCC2_DIV400 | (0x02 << SYSDIV_LSB);
        assert_eq!(sysclk_hz_for(rcc, rcc2, true), 80_000_000);
    }

    /// Bypassed `PIOSC` divided by 16 (`SYSDIV=0xF`) is 1 MHz.
    #[test]
    fn bypass_piosc_div16_is_1mhz() {
        let rcc =
            RCC_BYPASS | RCC_USESYSDIV | (0x0F << RCC_SYSDIV_SHIFT) | (0x01 << RCC_OSCSRC_SHIFT);
        assert_eq!(sysclk_hz_for(rcc, rcc2_reset_word(), false), 1_000_000);
    }

    fn rcc2_reset_word() -> u32 {
        RCC2_RESET
    }

    /// `RCGC` enables `PR` (`PP & RCGC`); the lock lands after the modeled
    /// delay and raises `RIS.PLLLRIS` + `PLLSTAT.LOCK`.
    #[test]
    fn gating_and_lock() {
        let mut sys = sysctl();
        // Enable GPIOA + UART0 clocks: PR follows immediately.
        write_word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_GPIO], 0x01);
        assert_eq!(word(&mut sys, PR_BASE_OFF + GROUP_OFFS[GROUP_GPIO]), 0x01);
        write_word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_UART], 0x01);
        assert_eq!(word(&mut sys, PR_BASE_OFF + GROUP_OFFS[GROUP_UART]), 0x01);
        // Power the PLL (RCC path): lock pending, then lands on tick.
        let rcc = RCC_USESYSDIV | (0x02 << RCC_SYSDIV_SHIFT);
        write_word(&mut sys, RCC_OFF, rcc);
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
        assert_eq!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        sys.tick(u64::from(PLL_LOCK_TICKS - 1));
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
        sys.tick(1);
        assert_ne!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
        assert_ne!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        assert_eq!(sys.sysclk_hz(), 66_666_666);
    }

    /// Powering the PLL back down clears the lock indications.
    #[test]
    fn pwrdn_clears_lock() {
        let mut sys = sysctl();
        let rcc = RCC_USESYSDIV | (0x02 << RCC_SYSDIV_SHIFT);
        write_word(&mut sys, RCC_OFF, rcc);
        sys.tick(u64::from(PLL_LOCK_TICKS));
        assert_ne!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        write_word(&mut sys, RCC_OFF, rcc | RCC_PWRDN);
        assert_eq!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
    }
}
