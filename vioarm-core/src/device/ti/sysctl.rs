//! TI Tiva C System Control and clocks (`SYSCTL`, `0x400F_E000`).
//!
//! Register map, resets, and clock math follow the TM4C123GH6PM datasheet
//! (SPMS376E §5, Table 5-7 for resets; Tables 5-4/5-5/5-6 for the PLL
//! divisor math). The SVD carries no `<resetValue>` and marks every window
//! `ReadWrite`, so this behavior stamps the datasheet resets at construction
//! and enforces the Table 5-7 RO/RW types on write.
//!
//! Modeled: `RCC`/`RCC2` (PLL math → `sysclk_hz`, pushed into the shared
//! [`ClockState`]), PLL lock with TREADY counts (§5.2.5.8: `0x200` on
//! power-up, `0xC0` on XTAL/M-N change; HW holds the oscillator clock until
//! met, same hold modeled here via `locked=false` → oscillator path),
//! `RCGCn` gating with `PRn` ready latches (cleared on `RCGC` change or `SR`
//! `0→1`, set only when `PP & RCGC & !SR` plus dependent-clock checks, after
//! [`PR_READY_TICKS`] ticks modeling the §5.2.6 3-clock delay as a small tick
//! count), legacy `0x100–0x128` aliasing (§5.2.6 Important: legacy write also
//! writes peripheral-specific bits; peripheral-specific write does not reflect
//! back; reads return their own latch), `RIS`/`MISC` RW1C (`MISC` reads
//! `RIS & IMC`, W1C clears `RIS` including `PLLLMIS→PLLLRIS`; unmodeled
//! BOR/MOSCPUP/USBPLL/MOF bits stay latched 0), `RESC` sticky write-0-clear
//! (POR clears rest), MOSC gating (`MOSCDIS` clear + `NOXTAL` clear +
//! stabilize via [`MOSC_STABLE_TICKS`] ticks before select, §5.2.5.2),
//! dependent-clock rules (ADC needs `SysClk>=16MHz`, USB needs MOSC source +
//! `SysClk>=20MHz`, EEPROM `EEDONE` freeze via `set_eeprom_busy`), RO
//! ID/capability constants (`DID0` modeled `0x1805_0001`: `VER=1`,
//! `CLASS=0x05` per datasheet, die revision A1; `DID1`/`DC0-DC9` from
//! Table 5-7), `RESC` power-on (`POR` set).
//! Deliberate approximations: `LFIOSC` nominal 33 kHz (Table 24-16:
//! 10/33/90 kHz); unknown `XTAL` encodings fall back to 16 MHz (reserved/
//! undefined per RCC p256 — cosmetic, fallback kept with note); `OSCSRC`
//! unreachable fallbacks return `PIOSC` with reserved note (cosmetic);
//! `DIV400` reserved divisors `/2–/4` substitute `/5` (80 MHz) and `/7`
//! substitutes `/8` (50 MHz) per Table 5-6 (nearest valid, documented);
//! `PR` 3-clock delay modeled as [`PR_READY_TICKS`]=3 ticks (audit allows
//! immediate-with-ordering or small tick count); MOSC stabilize modeled as
//! [`MOSC_STABLE_TICKS`]=64 ticks (analog settle unobservable, same
//! convention as [`crate::device::stm32::rcc`]); USB PLL lock, BOR, MOF
//! analog events never fire in simulation.

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
/// Legacy gating offsets (§5.2.6: `0x100–0x128`).
const LEGACY_RCGC0_OFF: u32 = 0x100;
const LEGACY_RCGC1_OFF: u32 = 0x104;
const LEGACY_RCGC2_OFF: u32 = 0x108;
const LEGACY_SCGC0_OFF: u32 = 0x110;
const LEGACY_SCGC1_OFF: u32 = 0x114;
const LEGACY_SCGC2_OFF: u32 = 0x118;
const LEGACY_DCGC0_OFF: u32 = 0x120;
const LEGACY_DCGC1_OFF: u32 = 0x124;
const LEGACY_DCGC2_OFF: u32 = 0x128;
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
const GROUP_USB: usize = 8;
const GROUP_ADC: usize = 10;

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
/// `MOSCCTL` bits: `NOXTAL` 2, `MOSCIM` 1, `CVAL` 0 (p263).
const MOSCCTL_NOXTAL: u32 = 1 << 2;
/// `RIS.PLLLRIS` bit 6; `RIS.MOSCPUPRIS` bit 8; `PLLSTAT.LOCK` bit 0.
const RIS_PLLLRIS: u32 = 1 << 6;
const RIS_MOSCPUPRIS: u32 = 1 << 8;
const PLLSTAT_LOCK: u32 = 1 << 0;
/// `RESC.POR` bit 1 (power-on reset cause, sticky).
const RESC_POR: u32 = 1 << 1;
/// TREADY counts (§5.2.5.8): `0x200` on power-up, `0xC0` on XTAL (M/N) change.
/// HW holds the oscillator clock until met (modeled via `locked=false`).
const PLL_LOCK_TICKS_POWERUP: u32 = 0x200;
const PLL_LOCK_TICKS_XTAL: u32 = 0xC0;
/// MOSC stabilize ticks (analog settle unobservable; small-tick model).
const MOSC_STABLE_TICKS: u32 = 64;
/// `PR` ready delay modeling the §5.2.6 3-clock delay as a small tick count
/// (audit allows immediate-with-ordering or small tick count).
const PR_READY_TICKS: u32 = 3;
/// `MINSYSDIV` from `DC1` p427 (`0x2`: 80 MHz CPU clock, PLL divider 2.5).
/// RCC p254: if `SYSDIV<MINSYSDIV` and PLL used, `MINSYSDIV` is substituted.
const MINSYSDIV: u32 = 0x2;
/// `PIOSC` rate in Hz (datasheet §5.2.5.1: 16 MHz).
pub const PIOSC_HZ: u32 = 16_000_000;
/// Nominal `LFIOSC` rate in Hz (Table 24-16: Min 10 kHz, Nom 33 kHz,
/// Max 90 kHz).
pub const LFIOSC_HZ: u32 = 33_000;
/// 32.768 kHz Hibernate-module oscillator in Hz.
pub const HIB_OSC_HZ: u32 = 32_768;
/// PLL VCO frequency in Hz (datasheet §5.2.5.3: 400 MHz, predivided by 2
/// to 200 MHz unless `DIV400` selects the 400 MHz output directly).
const PLL_VCO_HZ: u64 = 400_000_000;
/// Maximum TM4C123 system clock in Hz (80 MHz part).
const SYSCLK_MAX_HZ: u32 = 80_000_000;
/// Minimum valid `SysClk` for ADC (Fig 5-5 note: ADC needs PLL or 16 MHz;
/// if not using PIOSC directly, `SysClk` must be at least 16 MHz).
const ADC_MIN_SYSCLK_HZ: u32 = 16_000_000;
/// Minimum valid `SysClk` for USB (Fig 5-5 note: MOSC source + >=20 MHz).
const USB_MIN_SYSCLK_HZ: u32 = 20_000_000;

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
/// register page). Encodings not listed (`0x00–0x05`, `0x1B+`) are
/// reserved/undefined (RCC p256) — cosmetic: fallback to 16 MHz kept with
/// this note.
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
    (0x100, 0x0000_0040), // RCGC0 (legacy)
    (0x104, 0),           // RCGC1 (legacy)
    (0x108, 0),           // RCGC2 (legacy)
    (0x110, 0x0000_0040), // SCGC0 (legacy)
    (0x114, 0),           // SCGC1 (legacy)
    (0x118, 0),           // SCGC2 (legacy)
    (0x120, 0x0000_0040), // DCGC0 (legacy)
    (0x124, 0),           // DCGC1 (legacy)
    (0x128, 0),           // DCGC2 (legacy)
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

/// Legacy `RCGC0`/`SCGC0`/`DCGC0` bit → (group, peripheral bit).
/// From RCGC0/SCGC0/DCGC0 pages (WDT/CAN/PWM/ADC/HIB).
const LEGACY0_MAP: &[(u32, usize, u32)] = &[
    (28, 0, 1),  // WDT1 → WD R1
    (25, 9, 1),  // CAN1 → CAN R1
    (24, 9, 0),  // CAN0 → CAN R0
    (20, 12, 0), // PWM0 → PWM R0 (PWM1 only via peripheral-specific)
    (17, 10, 1), // ADC1 → ADC R1
    (16, 10, 0), // ADC0 → ADC R0
    (6, 4, 0),   // HIB → HIB R0
    (3, 0, 0),   // WDT0 → WD R0
];
/// Legacy `RCGC1`/`SCGC1`/`DCGC1` bit → (group, peripheral bit).
/// From RCGC1 page (COMP/TIMER/I2C/QEI/SSI/UART).
const LEGACY1_MAP: &[(u32, usize, u32)] = &[
    (25, 11, 0), // COMP1 → ACMP R0 (single-bit module, OR)
    (24, 11, 0), // COMP0 → ACMP R0
    (19, 1, 3),  // TIMER3 → TIMER R3
    (18, 1, 2),  // TIMER2 → TIMER R2
    (17, 1, 1),  // TIMER1 → TIMER R1
    (16, 1, 0),  // TIMER0 → TIMER R0
    (14, 7, 1),  // I2C1 → I2C R1
    (12, 7, 0),  // I2C0 → I2C R0
    (9, 13, 1),  // QEI1 → QEI R1
    (8, 13, 0),  // QEI0 → QEI R0
    (5, 6, 1),   // SSI1 → SSI R1
    (4, 6, 0),   // SSI0 → SSI R0
    (2, 5, 2),   // UART2 → UART R2
    (1, 5, 1),   // UART1 → UART R1
    (0, 5, 0),   // UART0 → UART R0
];
/// Legacy `RCGC2`/`SCGC2`/`DCGC2` bit → (group, peripheral bit).
/// From RCGC2 page (USB/uDMA/GPIO).
const LEGACY2_MAP: &[(u32, usize, u32)] = &[
    (16, 8, 0), // USB0 → USB R0
    (13, 3, 0), // UDMA → DMA R0
    (5, 2, 5),  // GPIOF → GPIO R5
    (4, 2, 4),  // GPIOE → GPIO R4
    (3, 2, 3),  // GPIOD → GPIO R3
    (2, 2, 2),  // GPIOC → GPIO R2
    (1, 2, 1),  // GPIOB → GPIO R1
    (0, 2, 0),  // GPIOA → GPIO R0
];

/// Returns `true` for offsets firmware may write (Table 5-7 `RW` type plus
/// legacy `0x100–0x128` which alias despite `RO` type, §5.2.6).
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
    // Legacy gating registers alias (§5.2.6): writable for the alias effect.
    if matches!(
        offset,
        LEGACY_RCGC0_OFF
            | LEGACY_RCGC1_OFF
            | LEGACY_RCGC2_OFF
            | LEGACY_SCGC0_OFF
            | LEGACY_SCGC1_OFF
            | LEGACY_SCGC2_OFF
            | LEGACY_DCGC0_OFF
            | LEGACY_DCGC1_OFF
            | LEGACY_DCGC2_OFF
    ) {
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

/// Crystal rate for an `XTAL` encoding, falling back to `PIOSC` for
/// reserved/undefined encodings (RCC p256 — cosmetic, fallback kept).
fn xtal_hz(xtal: u32) -> u32 {
    XTAL_HZ
        .iter()
        .find(|(enc, _)| *enc == xtal)
        .map_or(PIOSC_HZ, |(_, hz)| *hz)
}

/// Compute `SYSCLK` in Hz from the `RCC`/`RCC2` words (datasheet §5.2.5.3,
/// Tables 5-4/5-5/5-6). `locked` gates the PLL path: an unpowered or
/// not-yet-locked PLL cannot source the clock, so the raw oscillator feeds
/// through (firmware always bypasses first, per the 5-step procedure; HW
/// holds the oscillator until TREADY, §5.2.5.8). `mosc_ready` gates the
/// MOSC path (§5.2.5.2: `MOSCDIS` clear + stabilize before select;
/// otherwise `PIOSC` feeds through).
///
/// Reserved handling: `SYSDIV` `0x0/0x1` + PLL are `reserved` (Table 5-4)
/// and `SYSDIV<MINSYSDIV` (`DC1=0x2`, RCC p254) substitutes `MINSYSDIV`
/// when the PLL is used — the two items coincide (`0x0/0x1` are the only
/// values below the floor), so both model as `0x2` (200 MHz/3 = 66.67 MHz).
/// `SYSDIV2` `0x00/0x01` + PLL likewise substitute `0x02` (Table 5-5).
/// `DIV400` reserved divisors `/2–/4` substitute `/5` (80 MHz) and `/7`
/// substitutes `/8` (50 MHz) per Table 5-6 (nearest valid). `OSCSRC2`
/// `0x4–0x6` are `Reserved` (RCC2 p262) and fall back to `PIOSC` (not HIB).
#[must_use]
pub fn sysclk_hz_for(rcc: u32, rcc2: u32, locked: bool, mosc_ready: bool) -> u32 {
    let usercc2 = rcc2 & RCC2_USERCC2 != 0;
    let cfg = if usercc2 { rcc2 } else { rcc };
    let bypass = cfg & BYPASS_BIT != 0;
    let pwrdn = cfg & PWRDN_BIT != 0;
    let usesysdiv = rcc & RCC_USESYSDIV != 0;
    let div400 = usercc2 && (rcc2 & RCC2_DIV400 != 0);
    let raw_sysdiv = if div400 {
        let s2 = (rcc2 >> SYSDIV_LSB) & RCC2_SYSDIV2_MASK;
        let lsb = (rcc2 & RCC2_SYSDIV2LSB) >> 22;
        (s2 << 1) | lsb
    } else if usercc2 {
        (rcc2 >> SYSDIV_LSB) & RCC2_SYSDIV2_MASK
    } else {
        (rcc >> RCC_SYSDIV_SHIFT) & RCC_SYSDIV_MASK
    };
    // Apply MINSYSDIV floor + reserved substitution on the PLL path only
    // (RCC p254: only when the PLL is being used; DIV400 note: only valid
    // when BYPASS2=0).
    let pll_in_use = !bypass && !pwrdn;
    let sysdiv = if div400 {
        if !pll_in_use {
            raw_sysdiv
        } else {
            let divisor = raw_sysdiv + 1;
            if divisor <= 4 {
                // Reserved `/2–/4` (Table 5-6): nearest valid `/5`.
                4
            } else if divisor == 7 {
                // Reserved `/7` (Table 5-6): nearest valid `/8`.
                7
            } else {
                raw_sysdiv
            }
        }
    } else if pll_in_use && raw_sysdiv < MINSYSDIV {
        // Reserved `0x0/0x1` (Tables 5-4/5-5) and MINSYSDIV floor (RCC p254).
        MINSYSDIV
    } else {
        raw_sysdiv
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
    let mosc = xtal_hz(xtal);
    let src = if usercc2 {
        match oscsrc {
            // MOSC gated: needs enable + stabilize (§5.2.5.2).
            0 => {
                if mosc_ready {
                    mosc
                } else {
                    PIOSC_HZ
                }
            }
            1 => PIOSC_HZ,
            2 => PIOSC_HZ / 4,
            3 => LFIOSC_HZ,
            // `0x4–0x6 Reserved` (RCC2 p262): fall back to PIOSC, not HIB.
            4..=6 => PIOSC_HZ,
            _ => HIB_OSC_HZ,
        }
    } else {
        match oscsrc {
            0 => {
                if mosc_ready {
                    mosc
                } else {
                    PIOSC_HZ
                }
            }
            1 => PIOSC_HZ,
            2 => PIOSC_HZ / 4,
            3 => LFIOSC_HZ,
            // Unreachable 2-bit value — cosmetic PIOSC fallback with note.
            _ => PIOSC_HZ,
        }
    };
    if usesysdiv {
        u32::try_from(u64::from(src) / divisor).unwrap_or(u32::MAX)
    } else {
        src
    }
}

/// Whether the current `RCC`/`RCC2` selects MOSC as the oscillator source.
fn selects_mosc(rcc: u32, rcc2: u32) -> bool {
    let usercc2 = rcc2 & RCC2_USERCC2 != 0;
    let cfg = if usercc2 { rcc2 } else { rcc };
    let mask = if usercc2 { 0x07 } else { 0x03 };
    (cfg >> OSCSRC_SHIFT) & mask == 0
}

/// Behavior behind the `SYSCTL` window: datasheet resets, RO enforcement,
/// PLL lock timing, MOSC stabilize, `PR` latches with delay, legacy
/// aliasing, `RIS`/`MISC` RW1C, `RESC` W0C, and `sysclk_hz` pushes.
#[derive(Debug)]
pub struct SysctlBehavior {
    clock: Option<Arc<Mutex<ClockState>>>,
    pll_lockdown: u32,
    locked: bool,
    sysclk: u32,
    last_xtal: u32,
    mosc_lockdown: u32,
    mosc_stable: bool,
    pr: [u32; 16],
    pr_pending: [u32; 16],
    pr_delay: u32,
    eeprom_busy: bool,
}

impl Default for SysctlBehavior {
    fn default() -> Self {
        Self {
            clock: None,
            pll_lockdown: 0,
            locked: false,
            sysclk: PIOSC_HZ,
            last_xtal: (RCC_RESET >> RCC_XTAL_SHIFT) & RCC_XTAL_MASK,
            mosc_lockdown: 0,
            mosc_stable: false,
            pr: [0; 16],
            pr_pending: [0; 16],
            pr_delay: 0,
            eeprom_busy: false,
        }
    }
}

impl SysctlBehavior {
    /// Whether MOSC is enabled (`MOSCDIS` clear + `NOXTAL` clear).
    fn mosc_enabled(view: &Registers<&mut [u32]>) -> bool {
        let rcc = view.get(RCC_OFF);
        let moscctl = view.get(MOSCCTL_OFF);
        rcc & RCC_MOSCDIS == 0 && moscctl & MOSCCTL_NOXTAL == 0
    }

    /// Whether MOSC is ready (enabled + stabilized).
    fn mosc_ready(&self, view: &Registers<&mut [u32]>) -> bool {
        Self::mosc_enabled(view) && self.mosc_stable
    }

    /// Desired `PR` bits for a group (`PP & RCGC & !SR`).
    fn desired_pr(view: &Registers<&mut [u32]>, group: usize) -> u32 {
        let off = GROUP_OFFS[group];
        let pp = view.get(PP_BASE_OFF + off);
        let rcgc = view.get(RCGC_BASE_OFF + off);
        let sr = view.get(SR_BASE_OFF + off);
        pp & rcgc & !sr
    }

    /// Clear `PR` for a group and schedule the desired value after
    /// [`PR_READY_TICKS`] (models clear-on-change + 3-clock set delay).
    fn pend_group(&mut self, view: &mut Registers<&mut [u32]>, group: usize) {
        self.pr[group] = 0;
        self.pr_pending[group] = Self::desired_pr(view, group);
        self.pr_delay = PR_READY_TICKS;
    }

    /// Recompute lock state and `sysclk` from the current `RCC`/`RCC2` words.
    /// Handles TREADY counts (`0x200` power-up, `0xC0` XTAL change) and
    /// holds the oscillator clock while relocking.
    fn recompute(&mut self, view: &mut Registers<&mut [u32]>) {
        // EEPROM freeze: sysclk config cannot change during EEDONE WORKING.
        if self.eeprom_busy {
            return;
        }
        let rcc = view.get(RCC_OFF);
        let rcc2 = view.get(RCC2_OFF);
        let usercc2 = rcc2 & RCC2_USERCC2 != 0;
        let cfg = if usercc2 { rcc2 } else { rcc };
        let powered = cfg & PWRDN_BIT == 0;
        let xtal = (rcc >> RCC_XTAL_SHIFT) & RCC_XTAL_MASK;
        if !powered {
            if self.locked || self.pll_lockdown != 0 {
                self.locked = false;
                self.pll_lockdown = 0;
                // Power-down clears the lock indication (test-visible).
                let ris = view.get(RIS_OFF) & !RIS_PLLLRIS;
                view.set(RIS_OFF, ris);
            }
        } else if xtal != self.last_xtal {
            // XTAL (M/N) change relocks with `0xC0`; same-value writes do not.
            // Already-locked stays locked only when XTAL is unchanged.
            self.locked = false;
            self.pll_lockdown = PLL_LOCK_TICKS_XTAL;
            let ris = view.get(RIS_OFF) & !RIS_PLLLRIS;
            view.set(RIS_OFF, ris);
        } else if !self.locked && self.pll_lockdown == 0 {
            self.pll_lockdown = PLL_LOCK_TICKS_POWERUP;
        }
        self.last_xtal = xtal;
        let ready = self.mosc_ready(view);
        self.sysclk = sysclk_hz_for(rcc, rcc2, self.locked, ready);
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

    /// Dependent-clock gate for a `PR` group (Fig 5-5 notes).
    /// ADC needs `SysClk>=16MHz`; USB needs MOSC source + `SysClk>=20MHz`.
    fn dependent_ok(&self, view: &Registers<&mut [u32]>, group: usize) -> bool {
        if group == GROUP_ADC {
            self.sysclk >= ADC_MIN_SYSCLK_HZ
        } else if group == GROUP_USB {
            let rcc = view.get(RCC_OFF);
            let rcc2 = view.get(RCC2_OFF);
            selects_mosc(rcc, rcc2) && self.mosc_ready(view) && self.sysclk >= USB_MIN_SYSCLK_HZ
        } else {
            true
        }
    }

    /// `PR` group word: latched ready masked by dependent-clock rules.
    fn pr_word(&self, view: &Registers<&mut [u32]>, group: usize) -> u32 {
        if self.dependent_ok(view, group) {
            self.pr[group]
        } else {
            0
        }
    }

    /// Apply a legacy gating write to the peripheral-specific words (§5.2.6).
    /// Returns the list of affected groups for `PR` handling.
    fn apply_legacy(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        legacy: u32,
        word: u32,
    ) -> Vec<usize> {
        let (base, map) = match legacy {
            LEGACY_RCGC0_OFF => (RCGC_BASE_OFF, LEGACY0_MAP),
            LEGACY_RCGC1_OFF => (RCGC_BASE_OFF, LEGACY1_MAP),
            LEGACY_RCGC2_OFF => (RCGC_BASE_OFF, LEGACY2_MAP),
            LEGACY_SCGC0_OFF => (SCGC_BASE_OFF, LEGACY0_MAP),
            LEGACY_SCGC1_OFF => (SCGC_BASE_OFF, LEGACY1_MAP),
            LEGACY_SCGC2_OFF => (SCGC_BASE_OFF, LEGACY2_MAP),
            LEGACY_DCGC0_OFF => (DCGC_BASE_OFF, LEGACY0_MAP),
            LEGACY_DCGC1_OFF => (DCGC_BASE_OFF, LEGACY1_MAP),
            LEGACY_DCGC2_OFF => (DCGC_BASE_OFF, LEGACY2_MAP),
            _ => return Vec::new(),
        };
        let mut groups = Vec::new();
        for &(leg_bit, group, per_bit) in map {
            let off = base + GROUP_OFFS[group];
            let mut per = view.get(off);
            let want = word & (1 << leg_bit) != 0;
            let have = per & (1 << per_bit) != 0;
            if want != have {
                if want {
                    per |= 1 << per_bit;
                } else {
                    per &= !(1 << per_bit);
                }
                view.set(off, per);
                if !groups.contains(&group) {
                    groups.push(group);
                }
            }
        }
        groups
    }
}

impl Behavior for SysctlBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        let dt = u32::try_from(delta).unwrap_or(u32::MAX);
        if self.pll_lockdown != 0 {
            self.pll_lockdown = self.pll_lockdown.saturating_sub(dt);
            if self.pll_lockdown == 0 {
                self.locked = true;
                let ris = view.get(RIS_OFF) | RIS_PLLLRIS;
                view.set(RIS_OFF, ris);
                let rcc = view.get(RCC_OFF);
                let rcc2 = view.get(RCC2_OFF);
                let ready = self.mosc_ready(view);
                self.sysclk = sysclk_hz_for(rcc, rcc2, true, ready);
                self.push_clock();
            }
        }
        if self.mosc_lockdown != 0 {
            self.mosc_lockdown = self.mosc_lockdown.saturating_sub(dt);
            if self.mosc_lockdown == 0 {
                self.mosc_stable = true;
                let ris = view.get(RIS_OFF) | RIS_MOSCPUPRIS;
                view.set(RIS_OFF, ris);
                // MOSC now selectable: refresh sysclk (may switch from PIOSC).
                let rcc = view.get(RCC_OFF);
                let rcc2 = view.get(RCC2_OFF);
                self.sysclk = sysclk_hz_for(rcc, rcc2, self.locked, true);
                self.push_clock();
            }
        }
        if self.pr_delay != 0 {
            self.pr_delay = self.pr_delay.saturating_sub(dt);
            if self.pr_delay == 0 {
                self.pr = self.pr_pending;
            }
        }
    }

    fn reset(&mut self) {
        self.pll_lockdown = 0;
        self.locked = false;
        self.sysclk = PIOSC_HZ;
        self.last_xtal = (RCC_RESET >> RCC_XTAL_SHIFT) & RCC_XTAL_MASK;
        self.mosc_lockdown = 0;
        self.mosc_stable = false;
        self.pr = [0; 16];
        self.pr_pending = [0; 16];
        self.pr_delay = 0;
        self.eeprom_busy = false;
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let ris = view.get(RIS_OFF);
        // Synthesize lock/MOSC bits that are still converging as 0; stored
        // RIS holds the latched indications (cleared via MISC W1C).
        let mut full = ris;
        if !self.locked {
            full &= !RIS_PLLLRIS;
        }
        if !self.mosc_stable {
            full &= !RIS_MOSCPUPRIS;
        }
        u32::from(full & view.get(IMC_OFF) != 0)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == RIS_OFF {
            // Stored latch masked by live status (power-down clears the
            // latch; unready reads 0 even if a stale bit somehow remains).
            let mut out = view.get(RIS_OFF);
            if !self.locked {
                out &= !RIS_PLLLRIS;
            }
            if !self.mosc_stable {
                out &= !RIS_MOSCPUPRIS;
            }
            return out;
        }
        if offset == PLLSTAT_OFF {
            return if self.locked {
                value | PLLSTAT_LOCK
            } else {
                value & !PLLSTAT_LOCK
            };
        }
        if offset == MISC_OFF {
            // MISC reads `RIS & IMC` (p249).
            let mut ris = view.get(RIS_OFF);
            if !self.locked {
                ris &= !RIS_PLLLRIS;
            }
            if !self.mosc_stable {
                ris &= !RIS_MOSCPUPRIS;
            }
            return ris & view.get(IMC_OFF);
        }
        if PR_BASE_OFF <= offset {
            if let Some(group) = GROUP_OFFS.iter().position(|g| PR_BASE_OFF + g == offset) {
                return self.pr_word(view, group);
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
            // RW1C: writing 1 clears the corresponding RIS bit (p249);
            // writing 0 has no effect. Other (unmodeled) bits stay latched 0.
            let w1c = u32::from(val) << shift;
            let ris = view.get(RIS_OFF) & !w1c;
            view.set(RIS_OFF, ris);
            return view.get(offset);
        }
        if offset == RESC_OFF {
            // Sticky write-0-clear (p252–253): writing 0 clears, writing 1
            // cannot set. POR clears the rest (handled in `stamp_resets`).
            let old = view.get(offset);
            let merged = view.merged(offset, val, shift);
            return old & merged;
        }
        if !is_writable(offset) {
            return view.get(offset);
        }
        // Legacy aliasing (§5.2.6).
        if matches!(
            offset,
            LEGACY_RCGC0_OFF
                | LEGACY_RCGC1_OFF
                | LEGACY_RCGC2_OFF
                | LEGACY_SCGC0_OFF
                | LEGACY_SCGC1_OFF
                | LEGACY_SCGC2_OFF
                | LEGACY_DCGC0_OFF
                | LEGACY_DCGC1_OFF
                | LEGACY_DCGC2_OFF
        ) {
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            let groups = self.apply_legacy(view, offset, word);
            // RCGC legacy writes drive PR like peripheral-specific ones.
            if matches!(
                offset,
                LEGACY_RCGC0_OFF | LEGACY_RCGC1_OFF | LEGACY_RCGC2_OFF
            ) {
                for g in groups {
                    self.pend_group(view, g);
                }
            }
            return word;
        }
        // Peripheral-specific RCGC: PR cleared immediately, set after delay.
        if let Some(group) = GROUP_OFFS.iter().position(|g| RCGC_BASE_OFF + g == offset) {
            let old = view.get(offset);
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            if old != word {
                self.pend_group(view, group);
            }
            return word;
        }
        // Peripheral-specific SR: `0→1` clears PR (reset start); `1→0`
        // schedules ready (reset done). Any `0→1` bit clears.
        if let Some(group) = GROUP_OFFS.iter().position(|g| SR_BASE_OFF + g == offset) {
            let old = view.get(offset);
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            let started = (!old & word) != 0;
            let finished = (old & !word) != 0;
            if started {
                self.pr[group] = 0;
                self.pr_pending[group] = 0;
                self.pr_delay = PR_READY_TICKS;
            } else if finished {
                self.pend_group(view, group);
            }
            return word;
        }
        let word = view.merged(offset, val, shift);
        if offset == RCC_OFF || offset == RCC2_OFF || offset == MOSCCTL_OFF {
            // EEPROM freeze: clock config cannot change during EEDONE WORKING.
            if self.eeprom_busy && (offset == RCC_OFF || offset == RCC2_OFF) {
                return view.get(offset);
            }
            // Recompute against the about-to-land word: stage it, refresh
            // lock/sysclk, then report the staged word.
            view.set(offset, word);
            // MOSC stabilize: starting the countdown when newly enabled.
            if Self::mosc_enabled(view) && !self.mosc_stable && self.mosc_lockdown == 0 {
                // Crystal path needs settle; single-ended (PWRDN set) would
                // be immediate, but the board crystal is fixed 25 MHz, so
                // require the full sequence (audit) and count ticks.
                self.mosc_lockdown = MOSC_STABLE_TICKS;
            } else if !Self::mosc_enabled(view) {
                self.mosc_stable = false;
                self.mosc_lockdown = 0;
                let ris = view.get(RIS_OFF) & !RIS_MOSCPUPRIS;
                view.set(RIS_OFF, ris);
            }
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
    /// `POR` clears all other `RESC` bits; `PR` starts ready where
    /// `PP & RCGC & !SR` (HIB out of reset).
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
        behavior.sysclk = sysclk_hz_for(rcc, rcc2, false, false);
        behavior.last_xtal = (rcc >> RCC_XTAL_SHIFT) & RCC_XTAL_MASK;
        behavior.locked = false;
        behavior.pll_lockdown = 0;
        behavior.mosc_stable = false;
        behavior.mosc_lockdown = 0;
        behavior.pr_delay = 0;
        // Initial PR: HIB (and any other PP&RCGC) ready out of reset.
        self.inner.with_view(|view, b| {
            for group in 0..GROUP_OFFS.len() {
                let want = SysctlBehavior::desired_pr(view, group);
                // Dependent gating for the initial read is applied on read;
                // store the raw latch here.
                b.pr[group] = want;
                b.pr_pending[group] = want;
            }
        });
    }

    /// Current `SYSCLK` in Hz, from the last `RCC`/`RCC2` write or lock event.
    #[must_use]
    pub fn sysclk_hz(&self) -> u32 {
        self.inner.behavior().sysclk
    }

    /// Mark EEPROM busy (`EEDONE.WORKING` set): `RCC`/`RCC2` writes freeze
    /// until clear (Fig 5-5 note). Cross-peripheral wiring lands separately;
    /// this latch exists so the rule is testable within `SYSCTL`.
    pub fn set_eeprom_busy(&mut self, busy: bool) {
        self.inner.behavior_mut().eeprom_busy = busy;
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
        // POR clears all RESC bits except POR (sticky otherwise).
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
        // `PRn` ready out of reset where `PP & RCGC` (HIB).
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
        assert_eq!(sysclk_hz_for(rcc, 0, true, false), 66_666_666);
    }

    /// The classic 80 MHz TivaWare setup (`DIV400`, `SYSDIV2=0x2`,
    /// 16 MHz crystal) divides 400 MHz by 5.
    #[test]
    fn div400_sysdiv2_0x2_is_80mhz() {
        let rcc = RCC_USESYSDIV | (0x15 << RCC_XTAL_SHIFT); // MOSC 16 MHz
        let rcc2 = RCC2_USERCC2 | RCC2_DIV400 | (0x02 << SYSDIV_LSB);
        assert_eq!(sysclk_hz_for(rcc, rcc2, true, true), 80_000_000);
    }

    /// Bypassed `PIOSC` divided by 16 (`SYSDIV=0xF`) is 1 MHz.
    #[test]
    fn bypass_piosc_div16_is_1mhz() {
        let rcc =
            RCC_BYPASS | RCC_USESYSDIV | (0x0F << RCC_SYSDIV_SHIFT) | (0x01 << RCC_OSCSRC_SHIFT);
        assert_eq!(
            sysclk_hz_for(rcc, rcc2_reset_word(), false, false),
            1_000_000
        );
    }

    fn rcc2_reset_word() -> u32 {
        RCC2_RESET
    }

    /// TREADY power-up uses `0x200` ticks and holds the oscillator clock
    /// (§5.2.5.8). `RCGC` enables `PR` after the 3-tick delay.
    #[test]
    fn tready_powerup_uses_0x200_and_holds_oscillator() {
        let mut sys = sysctl();
        // Enable GPIOA clock: PR stays 0 until the 3-tick delay passes.
        write_word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_GPIO], 0x01);
        assert_eq!(word(&mut sys, PR_BASE_OFF + GROUP_OFFS[GROUP_GPIO]), 0);
        sys.tick(u64::from(PR_READY_TICKS));
        assert_eq!(word(&mut sys, PR_BASE_OFF + GROUP_OFFS[GROUP_GPIO]), 0x01);
        // Power the PLL (RCC path, no USESYSDIV so the hold clock is the
        // undivided oscillator; same XTAL as reset so this is a power-up
        // `0x200` relock, not an XTAL-change `0xC0`): lock pending, held.
        let rcc = (0x02 << RCC_SYSDIV_SHIFT) | (0x0B << RCC_XTAL_SHIFT);
        write_word(&mut sys, RCC_OFF, rcc);
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
        assert_eq!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        assert_eq!(sys.sysclk_hz(), 16_000_000);
        sys.tick(u64::from(PLL_LOCK_TICKS_POWERUP - 1));
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
        sys.tick(1);
        assert_ne!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
        assert_ne!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        assert_eq!(sys.sysclk_hz(), 66_666_666);
    }

    /// XTAL change relocks with `0xC0`; same-value writes do not relock.
    #[test]
    fn xtal_change_relocks_with_0xc0() {
        let mut sys = sysctl();
        let rcc = RCC_USESYSDIV | (0x02 << RCC_SYSDIV_SHIFT);
        write_word(&mut sys, RCC_OFF, rcc);
        sys.tick(u64::from(PLL_LOCK_TICKS_POWERUP));
        assert_ne!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        // Same XTAL value: stays locked.
        write_word(&mut sys, RCC_OFF, rcc);
        assert_ne!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        // Different XTAL: relock with 0xC0, oscillator held meanwhile.
        let rcc_xtal = rcc | (0x15 << RCC_XTAL_SHIFT);
        write_word(&mut sys, RCC_OFF, rcc_xtal);
        assert_eq!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
        sys.tick(u64::from(PLL_LOCK_TICKS_XTAL - 1));
        assert_eq!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        sys.tick(1);
        assert_ne!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
    }

    /// `SYSDIV<MINSYSDIV` (`DC1=0x2`) substitutes `MINSYSDIV` when the PLL
    /// is used (RCC p254).
    #[test]
    fn minsysdiv_substitution() {
        // SYSDIV=0x1 + PLL would be /2 (100 MHz) as-written; floor gives /3.
        let rcc = RCC_USESYSDIV | (0x01 << RCC_SYSDIV_SHIFT);
        assert_eq!(sysclk_hz_for(rcc, 0, true, false), 66_666_666);
    }

    /// `SYSDIV` `0x0/0x1` + PLL are `reserved` (Table 5-4): not
    /// 200/100 MHz clamped to 80 MHz, but the `MINSYSDIV` substitution.
    #[test]
    fn sysdiv_reserved_0x0_0x1_not_clamped() {
        for sysdiv in [0x0, 0x1] {
            let rcc = RCC_USESYSDIV | (sysdiv << RCC_SYSDIV_SHIFT);
            assert_eq!(sysclk_hz_for(rcc, 0, true, false), 66_666_666);
        }
        // Bypass path stays valid: source/1 and source/2.
        let rcc0 = RCC_BYPASS | (0x01 << RCC_OSCSRC_SHIFT);
        assert_eq!(sysclk_hz_for(rcc0, 0, false, false), 16_000_000);
        let rcc1 =
            RCC_BYPASS | RCC_USESYSDIV | (0x01 << RCC_SYSDIV_SHIFT) | (0x01 << RCC_OSCSRC_SHIFT);
        assert_eq!(sysclk_hz_for(rcc1, 0, false, false), 8_000_000);
    }

    /// `OSCSRC2` `0x4–0x6` are `Reserved` (RCC2 p262): fall back to `PIOSC`,
    /// only `0x7` is the 32.768 kHz source.
    #[test]
    fn oscsrc2_reserved_falls_back_to_piosc() {
        let rcc = RCC_USESYSDIV;
        for src in [0x4, 0x5, 0x6] {
            let rcc2 = RCC2_USERCC2 | RCC_BYPASS | (src << OSCSRC_SHIFT);
            assert_eq!(sysclk_hz_for(rcc, rcc2, false, false), PIOSC_HZ);
        }
        let hib = RCC2_USERCC2 | RCC_BYPASS | (0x7 << OSCSRC_SHIFT);
        assert_eq!(sysclk_hz_for(rcc, hib, false, false), HIB_OSC_HZ);
    }

    /// `LFIOSC` nominal is 33 kHz (Table 24-16: 10/33/90).
    #[test]
    fn lfiosc_is_33khz() {
        assert_eq!(LFIOSC_HZ, 33_000);
        let rcc = RCC_BYPASS | (0x03 << RCC_OSCSRC_SHIFT);
        assert_eq!(sysclk_hz_for(rcc, 0, false, false), 33_000);
    }

    /// `PR` synthesis: cleared on `RCGC` change or `SR` `0→1`, set only when
    /// `PP & RCGC & !SR` after 3 ticks (small-tick model of the §5.2.6 delay).
    #[test]
    fn pr_clears_on_rcgc_and_sr_with_delay() {
        let mut sys = sysctl();
        let gpio = RCGC_BASE_OFF + GROUP_OFFS[GROUP_GPIO];
        let sr_gpio = SR_BASE_OFF + GROUP_OFFS[GROUP_GPIO];
        let pr_gpio = PR_BASE_OFF + GROUP_OFFS[GROUP_GPIO];
        write_word(&mut sys, gpio, 0x01);
        sys.tick(u64::from(PR_READY_TICKS));
        assert_eq!(word(&mut sys, pr_gpio), 0x01);
        // RCGC change clears immediately.
        write_word(&mut sys, gpio, 0x03);
        assert_eq!(word(&mut sys, pr_gpio), 0);
        sys.tick(u64::from(PR_READY_TICKS));
        assert_eq!(word(&mut sys, pr_gpio), 0x03 & word(&mut sys, 0x308));
        // SR 0→1 clears (reset start).
        write_word(&mut sys, sr_gpio, 0x01);
        assert_eq!(word(&mut sys, pr_gpio), 0);
        // SR 1→0 schedules ready again after the delay.
        write_word(&mut sys, sr_gpio, 0x00);
        assert_eq!(word(&mut sys, pr_gpio), 0);
        sys.tick(u64::from(PR_READY_TICKS));
        assert_eq!(word(&mut sys, pr_gpio), 0x03 & word(&mut sys, 0x308));
    }

    /// Legacy `0x100–0x128` aliasing (§5.2.6 Important): legacy write also
    /// writes peripheral-specific bits; peripheral-specific write does not
    /// reflect back; each reads back its own latch.
    #[test]
    fn legacy_aliasing() {
        let mut sys = sysctl();
        // Legacy RCGC2 GPIOA (bit0) sets RCGCGPIO bit0.
        write_word(&mut sys, LEGACY_RCGC2_OFF, 0x01);
        assert_eq!(word(&mut sys, LEGACY_RCGC2_OFF) & 0x01, 0x01);
        assert_eq!(
            word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_GPIO]) & 0x01,
            0x01
        );
        // Peripheral-specific write for a legacy peripheral does not reflect.
        write_word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_GPIO], 0x00);
        assert_eq!(word(&mut sys, LEGACY_RCGC2_OFF) & 0x01, 0x01);
        assert_eq!(
            word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_GPIO]) & 0x01,
            0x00
        );
        // Legacy UART0 (RCGC1 bit0) aliases to RCGCUART bit0.
        write_word(&mut sys, LEGACY_RCGC1_OFF, 0x01);
        assert_eq!(
            word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_UART]) & 0x01,
            0x01
        );
    }

    /// `RIS`/`MISC` RW1C: `MISC` reads `RIS & IMC`, W1C clears `RIS`
    /// (`PLLLMIS` clears `PLLLRIS`); writing 0 has no effect.
    #[test]
    fn misc_rw1c_clears_ris() {
        let mut sys = sysctl();
        write_word(&mut sys, IMC_OFF, RIS_PLLLRIS);
        let rcc = RCC_USESYSDIV | (0x02 << RCC_SYSDIV_SHIFT);
        write_word(&mut sys, RCC_OFF, rcc);
        sys.tick(u64::from(PLL_LOCK_TICKS_POWERUP));
        assert_eq!(word(&mut sys, MISC_OFF) & RIS_PLLLRIS, RIS_PLLLRIS);
        assert_eq!(sys.pending(), 1);
        // Write 0: no effect.
        write_word(&mut sys, MISC_OFF, 0);
        assert_eq!(word(&mut sys, MISC_OFF) & RIS_PLLLRIS, RIS_PLLLRIS);
        // W1C clears RIS and the pending line.
        write_word(&mut sys, MISC_OFF, RIS_PLLLRIS);
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
        assert_eq!(word(&mut sys, MISC_OFF), 0);
        assert_eq!(sys.pending(), 0);
    }

    /// `RESC` sticky write-0-clear; `POR` clears the rest.
    #[test]
    fn resc_write0_clear_and_por() {
        let mut sys = sysctl();
        assert_eq!(word(&mut sys, RESC_OFF), RESC_POR);
        // Software cannot set bits (write 1 has no effect beyond POR).
        write_word(&mut sys, RESC_OFF, 0xFFFF_FFFF);
        assert_eq!(word(&mut sys, RESC_OFF), RESC_POR);
        // Writing 0 clears (including POR).
        write_word(&mut sys, RESC_OFF, 0);
        assert_eq!(word(&mut sys, RESC_OFF), 0);
        // POR (reset) clears the rest and sets POR.
        sys.reset();
        assert_eq!(word(&mut sys, RESC_OFF), RESC_POR);
    }

    /// MOSC gating: `MOSCDIS` clear + stabilize before select (§5.2.5.2).
    #[test]
    fn mosc_gating_requires_enable_and_stabilize() {
        let mut sys = sysctl();
        // XTAL 16 MHz but MOSC disabled out of reset: MOSC source falls back.
        let rcc = (0x15 << RCC_XTAL_SHIFT) | (0x00 << RCC_OSCSRC_SHIFT);
        write_word(&mut sys, RCC_OFF, rcc);
        assert_eq!(sys.sysclk_hz(), PIOSC_HZ);
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_MOSCPUPRIS, 0);
        // Enable: clear MOSCDIS (bit0). Still PIOSC until stabilize ticks.
        let rcc_en = rcc & !RCC_MOSCDIS;
        write_word(&mut sys, RCC_OFF, rcc_en);
        assert_eq!(sys.sysclk_hz(), PIOSC_HZ);
        sys.tick(u64::from(MOSC_STABLE_TICKS));
        assert_ne!(word(&mut sys, RIS_OFF) & RIS_MOSCPUPRIS, 0);
        assert_eq!(sys.sysclk_hz(), 16_000_000);
    }

    /// Dependent-clock rules (Fig 5-5 notes): ADC needs `SysClk>=16MHz`,
    /// USB needs MOSC + `SysClk>=20MHz`, EEPROM freezes `RCC` during busy.
    #[test]
    fn dependent_clocks_adc_usb_eeprom() {
        let mut sys = sysctl();
        // Slow bypass clock (PIOSC/16 = 1 MHz): ADC PR gated off.
        let slow =
            RCC_BYPASS | RCC_USESYSDIV | (0x0F << RCC_SYSDIV_SHIFT) | (0x01 << RCC_OSCSRC_SHIFT);
        write_word(&mut sys, RCC_OFF, slow);
        write_word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_ADC], 0x03);
        sys.tick(u64::from(PR_READY_TICKS));
        assert_eq!(word(&mut sys, PR_BASE_OFF + GROUP_OFFS[GROUP_ADC]), 0);
        // Fast PLL clock: ADC PR ready.
        let fast = RCC_USESYSDIV | (0x02 << RCC_SYSDIV_SHIFT);
        write_word(&mut sys, RCC_OFF, fast);
        sys.tick(u64::from(PLL_LOCK_TICKS_POWERUP));
        sys.tick(u64::from(PR_READY_TICKS));
        assert_ne!(
            word(&mut sys, PR_BASE_OFF + GROUP_OFFS[GROUP_ADC]) & 0x01,
            0
        );
        // USB with PIOSC source (not MOSC): PR gated off even when fast.
        // Keep the PLL fast but switch the oscillator source to PIOSC.
        let fast_piosc = RCC_USESYSDIV | (0x02 << RCC_SYSDIV_SHIFT) | (0x01 << RCC_OSCSRC_SHIFT);
        write_word(&mut sys, RCC_OFF, fast_piosc);
        write_word(&mut sys, RCGC_BASE_OFF + GROUP_OFFS[GROUP_USB], 0x01);
        sys.tick(u64::from(PR_READY_TICKS));
        assert_eq!(word(&mut sys, PR_BASE_OFF + GROUP_OFFS[GROUP_USB]), 0);
        // EEPROM freeze: RCC writes ignored while busy.
        sys.set_eeprom_busy(true);
        let before = word(&mut sys, RCC_OFF);
        write_word(&mut sys, RCC_OFF, RCC_BYPASS);
        assert_eq!(word(&mut sys, RCC_OFF), before);
        sys.set_eeprom_busy(false);
        write_word(&mut sys, RCC_OFF, RCC_BYPASS);
        assert_eq!(word(&mut sys, RCC_OFF) & RCC_BYPASS, RCC_BYPASS);
    }

    /// `XTAL` reserved (`0x00–0x05`, `0x1B+`) falls back to 16 MHz
    /// (RCC p256 — cosmetic, fallback kept with note).
    #[test]
    fn xtal_reserved_falls_back_to_piosc() {
        for xtal in [0x00, 0x05, 0x1B, 0x1F] {
            let rcc = RCC_BYPASS | (xtal << RCC_XTAL_SHIFT) | (0x00 << RCC_OSCSRC_SHIFT);
            // MOSC path would use XTAL, but reserved → PIOSC fallback for the
            // crystal value itself; with MOSC not ready the source is PIOSC
            // either way — assert the helper maps reserved to PIOSC.
            assert_eq!(xtal_hz(xtal), PIOSC_HZ);
            assert_eq!(sysclk_hz_for(rcc, 0, false, true), PIOSC_HZ);
        }
    }

    /// Powering the PLL back down clears the lock indications.
    #[test]
    fn pwrdn_clears_lock() {
        let mut sys = sysctl();
        let rcc = RCC_USESYSDIV | (0x02 << RCC_SYSDIV_SHIFT);
        write_word(&mut sys, RCC_OFF, rcc);
        sys.tick(u64::from(PLL_LOCK_TICKS_POWERUP));
        assert_ne!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        write_word(&mut sys, RCC_OFF, rcc | RCC_PWRDN);
        assert_eq!(word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        assert_eq!(word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
    }
}
