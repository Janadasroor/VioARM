//! TI Tiva C 129-family System Control and clocks (`SYSCTL`, `0x400F_E000`).
//!
//! Register map, resets, and clock math follow the TM4C1294NCPDT datasheet
//! (SPMS433B §5; Table 5-11 for resets; RSCLKCFG/PLLFREQ pages for the PLL
//! math). The 129 SYSCTL is a different generation from the 123 one (no
//! `RCC`/`RCC2`; `RSCLKCFG` + `PLLFREQ0`/`1` instead), so this is a native
//! model, not a parameterization of [`crate::device::ti::sysctl`]. The SVD
//! carries no `<resetValue>` and marks every window `ReadWrite`, so this
//! behavior stamps the datasheet resets at construction and enforces the
//! Table 5-11 RO/RW types on write.
//!
//! Modeled: `RSCLKCFG`/`PLLFREQ0`/`PLLFREQ1` (PLL math → `sysclk_hz`, pushed
//! into the shared [`ClockState`]), PLL lock after [`PLL_LOCK_TICKS`]
//! device ticks (`RIS.PLLLRIS` bit 6 + `PLLSTAT.LOCK` bit 0; the analog lock
//! is unobservable to polling firmware, same convention as the 123 model),
//! `RCGCn`/`SCGCn`/`DCGCn`/`SRn` latch words (instant-apply; the 129 family
//! has no `PRn` ready registers — firmware waits 3 clocks, unobservable
//! here), RO ID/capability constants (`DID0` modeled `0x100A_0001`:
//! `VER=1`, `CLASS=0x0A` Snowflake per datasheet, die revision A1;
//! `DID1` `0x101F_C06E`; `PPn` from Table 5-11, including the genuine
//! `PPWTIMER=0` quirk), `RESC` power-on (`POR` set), `MISC` W1C sink.
//! Deliberate approximations: `MOSC` is a modeled 25 MHz crystal (the
//! Connected LaunchPad crystal; frequency is not a register and firmware
//! "knows" its board); `MOSC` ready is immediate; `LFIOSC` reads nominal
//! 30 kHz; `MOSCPUPRIS`/`MOFRIS`/`BORRIS` never fire (no analog events in
//! simulation, same rationale as the 123 `MISC` handling).

use crate::device::stm32::rcc::ClockState;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// `SYSCTL` base address.
pub const SYSCTL_BASE: u32 = 0x400F_E000;

const DID0_OFF: u32 = 0x000;
const DID1_OFF: u32 = 0x004;
const RIS_OFF: u32 = 0x050;
const IMC_OFF: u32 = 0x054;
const MISC_OFF: u32 = 0x058;
const RESC_OFF: u32 = 0x05C;
const RSCLKCFG_OFF: u32 = 0x0B0;
const PLLFREQ0_OFF: u32 = 0x160;
const PLLFREQ1_OFF: u32 = 0x164;
const PLLSTAT_OFF: u32 = 0x168;

/// `RSCLKCFG` fields.
const RS_USEPLL: u32 = 1 << 28;
const RS_PLLSRC_SHIFT: u32 = 24;
const RS_OSCSRC_SHIFT: u32 = 20;
const RS_OSYSDIV_SHIFT: u32 = 10;
const RS_OSYSDIV_MASK: u32 = 0x3FF;
const RS_PSYSDIV_MASK: u32 = 0x3FF;
const RS_NEWFREQ: u32 = 1 << 30;
/// Oscillator sources.
const OSCSRC_PIOSC: u32 = 0x0;
const OSCSRC_LFIOSC: u32 = 0x2;
const OSCSRC_MOSC: u32 = 0x3;
const OSCSRC_RTC: u32 = 0x4;
/// `PLLFREQ0` fields.
const FREQ0_PLLPWR: u32 = 1 << 23;
const FREQ0_MFRAC_SHIFT: u32 = 10;
const FREQ0_MFRAC_MASK: u32 = 0x3FF;
const FREQ0_MINT_MASK: u32 = 0x3FF;
/// `PLLFREQ1` fields.
const FREQ1_Q_SHIFT: u32 = 8;
const FREQ1_Q_MASK: u32 = 0x1F;
const FREQ1_N_MASK: u32 = 0x1F;
/// `PLLSTAT.LOCK` bit 0; `RIS` interrupt bits.
const PLLSTAT_LOCK: u32 = 1 << 0;
const RIS_PLLLRIS: u32 = 1 << 6;

/// `SRn`/`RCGCn`/`SCGCn`/`DCGCn` group base offsets.
const SR_BASE_OFF: u32 = 0x500;
const RCGC_BASE_OFF: u32 = 0x600;
const SCGC_BASE_OFF: u32 = 0x700;
const DCGC_BASE_OFF: u32 = 0x800;
/// Group strides present on TM4C1294NCPDT (SVD).
const GROUP_OFFS: &[u32] = &[
    0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x28, 0x30, 0x34, 0x38, 0x3C, 0x40, 0x44,
    0x58, 0x74, 0x9C,
];

const PLL_LOCK_TICKS: u32 = 128;
/// `PIOSC` rate in Hz (datasheet §5.2.5.1: 16 MHz).
pub const PIOSC_HZ: u32 = 16_000_000;
/// Nominal `LFIOSC` rate in Hz (datasheet: wide variations; TivaWare nominal).
pub const LFIOSC_HZ: u32 = 30_000;
/// Hibernate RTC oscillator in Hz.
pub const RTC_OSC_HZ: u32 = 32_768;
/// Modeled MOSC crystal in Hz (EK-TM4C1294XL 25 MHz crystal; the frequency
/// is board knowledge, not a register).
pub const MOSC_HZ: u32 = 25_000_000;
/// Maximum TM4C129 system clock in Hz (120 MHz part).
const SYSCLK_MAX_HZ: u32 = 120_000_000;

/// `DID0` model: `VER=1`, `CLASS=0x0A` Snowflake (datasheet), die revision
/// A1 (`MAJOR=0`, `MINOR=1` — a modeled choice, silicon varies by lot).
const DID0_MODEL: u32 = 0x100A_0001;
/// `DID1` reset (Table 5-11: `PARTNO=0x1F`, 128-pin TQFP).
const DID1_RESET: u32 = 0x101F_C06E;

/// `(offset, datasheet reset)` for every `SYSCTL` register the behavior
/// stamps at construction (Table 5-11). `RIS`/`PLLSTAT` are synthesized
/// (stamped 0 here).
const RESETS: &[(u32, u32)] = &[
    (DID0_OFF, DID0_MODEL),
    (DID1_OFF, DID1_RESET),
    (0x038, 0x0000_0003),    // PTBOCTL
    (RESC_OFF, 0x0000_0002), // RESC (POR)
    (0x07C, 0x0000_000C),    // MOSCCTL (PWRDN + NOXTAL)
    (0x0C0, 0x0030_0030),    // MEMTIM0
    (0x14C, 0x0003_1F31),    // SYSPROP
    (0x154, 0x0040_0040),    // PIOSCSTAT
    (0x1A0, 0x0000_0001),    // NVMSTAT
    (0x1B4, 0x0000_0018),    // LDOSPCTL
    (0x1B8, 0x0000_1818),    // LDOSPCAL
    (0x1BC, 0x0000_0012),    // LDODPCTL
    (0x1C0, 0x0000_1212),    // LDODPCAL
    (0x1D8, 0xFFFF_FFFF),    // RESBEHAVCTL
    (0x280, 0x0000_003F),    // USBPDS
    (0x284, 0x0000_0003),    // USBMPC
    (0x288, 0x0000_003F),    // EMACPDS
    (0x28C, 0x0000_0003),    // EMACMPC
    (0x300, 0x0000_0003),    // PPWD
    (0x304, 0x0000_00FF),    // PPTIMER
    (0x308, 0x0000_7FFF),    // PPGPIO
    (0x30C, 0x0000_0001),    // PPDMA
    (0x310, 0x0000_0001),    // PPEPI
    (0x314, 0x0000_0001),    // PPHIB
    (0x318, 0x0000_00FF),    // PPUART
    (0x31C, 0x0000_000F),    // PPSSI
    (0x320, 0x0000_03FF),    // PPI2C
    (0x328, 0x0000_0001),    // PPUSB
    (0x330, 0x0000_0001),    // PPEPHY
    (0x334, 0x0000_0003),    // PPCAN
    (0x338, 0x0000_0003),    // PPADC
    (0x33C, 0x0000_0001),    // PPACMP
    (0x340, 0x0000_0001),    // PPPWM
    (0x344, 0x0000_0001),    // PPQEI
    (0x348, 0x0000_0000),    // PPLPC
    (0x350, 0x0000_0000),    // PPPECI
    (0x354, 0x0000_0000),    // PPFAN
    (0x358, 0x0000_0001),    // PPEEPROM
    (0x35C, 0x0000_0000),    // PPWTIMER (genuine die quirk)
    (0x370, 0x0000_0000),    // PPRTS
    (0x374, 0x0000_0001),    // PPCCM
    (0x390, 0x0000_0000),    // PPLCD
    (0x398, 0x0000_0000),    // PPOWIRE
    (0x39C, 0x0000_0001),    // PPEMAC
    (0x3A0, 0x0000_0000),    // PPPRB
];

/// Oscillator rate in Hz for an `OSCSRC`/`PLLSRC` encoding.
fn osc_hz(src: u32) -> u32 {
    match src {
        OSCSRC_PIOSC => PIOSC_HZ,
        OSCSRC_LFIOSC => LFIOSC_HZ,
        OSCSRC_MOSC => MOSC_HZ,
        OSCSRC_RTC => RTC_OSC_HZ,
        _ => PIOSC_HZ,
    }
}

/// Compute `SYSCLK` in Hz from the `RSCLKCFG`/`PLLFREQ0`/`PLLFREQ1` words
/// (datasheet RSCLKCFG + PLLFREQ pages). `locked` gates the PLL path: an
/// unpowered or not-yet-locked PLL cannot source the clock, so the raw
/// oscillator feeds through (firmware programs the PLL before setting
/// `USEPLL`, per the datasheet procedure).
#[must_use]
pub fn sysclk_hz_for_129(rsclkcfg: u32, pllfreq0: u32, pllfreq1: u32, locked: bool) -> u32 {
    let usepll = rsclkcfg & RS_USEPLL != 0;
    let powered = pllfreq0 & FREQ0_PLLPWR != 0;
    let oscsrc = (rsclkcfg >> RS_OSCSRC_SHIFT) & 0xF;
    if usepll && powered && locked {
        let pllsrc = (rsclkcfg >> RS_PLLSRC_SHIFT) & 0xF;
        let fin = u64::from(osc_hz(pllsrc));
        let mint = u64::from(pllfreq0 & FREQ0_MINT_MASK);
        let mfrac = u64::from((pllfreq0 >> FREQ0_MFRAC_SHIFT) & FREQ0_MFRAC_MASK);
        let q = u64::from((pllfreq1 >> FREQ1_Q_SHIFT) & FREQ1_Q_MASK);
        let n = u64::from(pllfreq1 & FREQ1_N_MASK);
        let mdiv_num = mint * 1024 + mfrac;
        let psysdiv = u64::from(rsclkcfg & RS_PSYSDIV_MASK);
        let fvco = fin * mdiv_num / (1024 * (q + 1) * (n + 1));
        u32::try_from(fvco / (psysdiv + 1)).unwrap_or(SYSCLK_MAX_HZ)
    } else {
        let osysdiv = u64::from((rsclkcfg >> RS_OSYSDIV_SHIFT) & RS_OSYSDIV_MASK);
        u32::try_from(u64::from(osc_hz(oscsrc)) / (osysdiv + 1)).unwrap_or(PIOSC_HZ)
    }
}

/// Whether `offset` is a writable (latch or W1C) register; everything else
/// (IDs, `PPn`, capability, status, power-domain) is RO.
fn is_writable(offset: u32) -> bool {
    if offset == DID0_OFF
        || offset == DID1_OFF
        || offset == RIS_OFF
        || offset == PLLSTAT_OFF
        || (0x14C..=0x154).contains(&offset) && offset != 0x150
        || offset == 0x1A0
        || offset == 0x1B8
        || offset == 0x1C0
        || offset == 0x280
        || offset == 0x288
        || (0x298..=0x2A0).contains(&offset) && offset % 4 == 0
        || (0x300..=0x3A4).contains(&offset)
    {
        return false;
    }
    // `SRn`/`RCGCn`/`SCGCn`/`DCGCn` groups are RW words.
    for base in [SR_BASE_OFF, RCGC_BASE_OFF, SCGC_BASE_OFF, DCGC_BASE_OFF] {
        if GROUP_OFFS.iter().any(|g| base + g == offset) {
            return true;
        }
    }
    // `MISC` is W1C (handled in `on_write`); the rest latch by default.
    true
}

/// PLL lock timing and `sysclk_hz` pushes.
#[derive(Debug)]
pub struct Sysctl129Behavior {
    clock: Option<Arc<Mutex<ClockState>>>,
    pll_lockdown: u32,
    locked: bool,
    sysclk: u32,
}

impl Default for Sysctl129Behavior {
    fn default() -> Self {
        Self {
            clock: None,
            pll_lockdown: 0,
            locked: false,
            sysclk: PIOSC_HZ,
        }
    }
}

impl Sysctl129Behavior {
    /// Recompute lock state and `sysclk` from the current clock words.
    fn recompute(&mut self, view: &mut Registers<&mut [u32]>) {
        let rsclkcfg = view.get(RSCLKCFG_OFF);
        let pllfreq0 = view.get(PLLFREQ0_OFF);
        let powered = pllfreq0 & FREQ0_PLLPWR != 0;
        let usepll = rsclkcfg & RS_USEPLL != 0;
        if !powered || !usepll {
            self.locked = false;
            self.pll_lockdown = 0;
        } else if !self.locked && self.pll_lockdown == 0 {
            self.pll_lockdown = PLL_LOCK_TICKS;
        }
        let pllfreq1 = view.get(PLLFREQ1_OFF);
        self.sysclk = sysclk_hz_for_129(rsclkcfg, pllfreq0, pllfreq1, self.locked);
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
}

impl Behavior for Sysctl129Behavior {
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
            let rsclkcfg = view.get(RSCLKCFG_OFF);
            let pllfreq0 = view.get(PLLFREQ0_OFF);
            let pllfreq1 = view.get(PLLFREQ1_OFF);
            self.sysclk = sysclk_hz_for_129(rsclkcfg, pllfreq0, pllfreq1, true);
            self.push_clock();
        }
    }

    fn reset(&mut self) {
        self.pll_lockdown = 0;
        self.locked = false;
        self.sysclk = PIOSC_HZ;
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let ris = if self.locked {
            view.get(RIS_OFF) | RIS_PLLLRIS
        } else {
            view.get(RIS_OFF) & !RIS_PLLLRIS
        };
        u32::from(ris & view.get(IMC_OFF) != 0)
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
            // analog events in simulation), so the write sinks.
            return view.get(offset);
        }
        if !is_writable(offset) {
            return view.get(offset);
        }
        let word = view.merged(offset, val, shift);
        if offset == RSCLKCFG_OFF || offset == PLLFREQ0_OFF || offset == PLLFREQ1_OFF {
            // Recompute against the about-to-land word: stage it, refresh
            // lock/sysclk, then report the staged word.
            view.set(offset, word);
            self.recompute(view);
        }
        word
    }
}

/// A Tiva C 129-family system controller.
pub struct Sysctl129 {
    inner: RegisterFile<Sysctl129Behavior>,
}

impl Sysctl129 {
    /// A new 129 SYSCTL on `D`'s tables (`"SYSCTL"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("SYSCTL").expect("generated tables contain SYSCTL");
        let mut sys = Self {
            inner: RegisterFile::new(def, Sysctl129Behavior::default()),
        };
        sys.stamp_resets();
        sys
    }

    /// Stamp the datasheet (Table 5-11) reset words over the SVD zeros.
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

    /// Current `SYSCLK` in Hz (last recompute).
    #[must_use]
    pub fn sysclk_hz(&self) -> u32 {
        self.inner.behavior().sysclk
    }

    /// Attach the shared clock state (the 129 board wires this).
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

impl Peripheral for Sysctl129 {
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

impl InterruptSource for Sysctl129 {
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
    use crate::device::svd::tm4c1294ncpdt::Tm4c1294ncpdt;

    const SYSCTL_BASE_129: u32 = 0x400F_E000;

    fn sys129() -> Sysctl129 {
        Sysctl129::for_device::<Tm4c1294ncpdt>()
    }

    fn read_word(sys: &mut Sysctl129, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(sys.read(SYSCTL_BASE_129 + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(sys: &mut Sysctl129, offset: u32, value: u32) {
        for lane in 0..4 {
            sys.write(
                SYSCTL_BASE_129 + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Datasheet Table 5-11 resets land on the bus.
    #[test]
    fn datasheet_resets() {
        let mut sys = sys129();
        assert_eq!(read_word(&mut sys, DID0_OFF), DID0_MODEL);
        assert_eq!(read_word(&mut sys, DID1_OFF), DID1_RESET);
        assert_eq!(read_word(&mut sys, 0x14C), 0x0003_1F31); // SYSPROP
        assert_eq!(read_word(&mut sys, 0x1A0), 0x0000_0001); // NVMSTAT
        assert_eq!(read_word(&mut sys, 0x318), 0x0000_00FF); // PPUART
        assert_eq!(read_word(&mut sys, 0x35C), 0x0000_0000); // PPWTIMER quirk
        assert_eq!(read_word(&mut sys, RESC_OFF), 0x0000_0002); // POR
        assert_eq!(read_word(&mut sys, 0x07C), 0x0000_000C); // MOSCCTL
        assert_eq!(read_word(&mut sys, 0x0C0), 0x0030_0030); // MEMTIM0
        assert_eq!(sys.exception_number(), Some(44)); // SYSCTL = IRQ 28.
    }

    /// RO ID/capability words reject writes.
    #[test]
    fn ro_enforcement() {
        let mut sys = sys129();
        write_word(&mut sys, DID1_OFF, 0xDEAD_BEEF);
        assert_eq!(read_word(&mut sys, DID1_OFF), DID1_RESET);
        write_word(&mut sys, 0x318, 0x00);
        assert_eq!(read_word(&mut sys, 0x318), 0x0000_00FF);
        write_word(&mut sys, 0x168, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut sys, 0x168) & PLLSTAT_LOCK, 0);
    }

    /// Bypass clock math: PIOSC through `OSYSDIV`.
    #[test]
    fn bypass_math() {
        // Reset: USEPLL=0, OSCSRC=PIOSC, OSYSDIV=0 → 16 MHz.
        assert_eq!(sysclk_hz_for_129(0, 0, 0, false), 16_000_000);
        // PIOSC / 4.
        let cfg = 3 << RS_OSYSDIV_SHIFT;
        assert_eq!(sysclk_hz_for_129(cfg, 0, 0, false), 4_000_000);
        // LFIOSC source.
        let cfg = OSCSRC_LFIOSC << RS_OSCSRC_SHIFT;
        assert_eq!(sysclk_hz_for_129(cfg, 0, 0, false), 30_000);
    }

    /// The LaunchPad 120 MHz configuration: 25 MHz MOSC → 480 MHz VCO → /4.
    #[test]
    fn pll_120mhz() {
        // MINT=96, MFRAC=0, N=4, Q=0; PSYSDIV=3; PLLSRC=MOSC; USEPLL.
        let freq0 = FREQ0_PLLPWR | 96;
        let freq1 = 4; // N=4, Q=0.
        let cfg = RS_USEPLL | (OSCSRC_MOSC << RS_PLLSRC_SHIFT) | 3;
        assert_eq!(sysclk_hz_for_129(cfg, freq0, freq1, true), 120_000_000);
        // Same words, unlocked: the raw oscillator feeds through.
        assert_eq!(sysclk_hz_for_129(cfg, freq0, freq1, false), 16_000_000);
    }

    /// `NEWFREQ` + 128 ticks lock the PLL (`PLLSTAT.LOCK`, `RIS.PLLLRIS`).
    #[test]
    fn newfreq_lock_flow() {
        let mut sys = sys129();
        let freq0 = FREQ0_PLLPWR | 96;
        write_word(&mut sys, PLLFREQ1_OFF, 4);
        write_word(&mut sys, PLLFREQ0_OFF, freq0);
        let cfg = RS_USEPLL | (OSCSRC_MOSC << RS_PLLSRC_SHIFT) | 3;
        write_word(&mut sys, RSCLKCFG_OFF, cfg | RS_NEWFREQ);
        assert_eq!(read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        sys.tick(127);
        assert_eq!(read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        sys.tick(1);
        assert_eq!(
            read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK,
            PLLSTAT_LOCK
        );
        assert_eq!(read_word(&mut sys, RIS_OFF) & RIS_PLLLRIS, RIS_PLLLRIS);
        assert_eq!(sys.sysclk_hz(), 120_000_000);
    }

    /// `RCGC` words latch; `PP` reads stay.
    #[test]
    fn rcgc_latch() {
        let mut sys = sys129();
        write_word(&mut sys, 0x618, 0x01); // RCGCUART
        assert_eq!(read_word(&mut sys, 0x618), 0x01);
        write_word(&mut sys, 0x518, 0xFFFF_FFFF); // SRUART sinks? no: SR latches.
        assert_eq!(read_word(&mut sys, 0x518), 0xFFFF_FFFF);
    }

    /// `MISC` W1C sinks; `IMC` gates the line.
    #[test]
    fn misc_imc_flow() {
        let mut sys = sys129();
        write_word(&mut sys, IMC_OFF, 1 << 6); // PLLLMIM
        write_word(&mut sys, PLLFREQ1_OFF, 4);
        write_word(&mut sys, PLLFREQ0_OFF, FREQ0_PLLPWR | 96);
        let cfg = RS_USEPLL | (OSCSRC_MOSC << RS_PLLSRC_SHIFT) | 3;
        write_word(&mut sys, RSCLKCFG_OFF, cfg | RS_NEWFREQ);
        sys.tick(128);
        assert_eq!(sys.pending(), 1);
        write_word(&mut sys, MISC_OFF, 1 << 6);
        assert_eq!(read_word(&mut sys, MISC_OFF), 0);
    }
}
