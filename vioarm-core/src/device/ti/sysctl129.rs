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
//! into the shared [`ClockState`]), `PLLFREQ` deferred until `NEWFREQ=1`
//! (`R0/W`, auto-clear; change only while powered-down / not sourcing,
//! p236–238), `MEMTIM0` applied only on `MEMTIMU=1` (`R0/W` auto-clear,
//! p277; wait-state values unmodeled — the applied value is latched),
//! `RSCLKCFG` bits 31/30 `R0/W` read-0 auto-clear, PLL lock after
//! [`PLL_LOCK_TICKS`] device ticks (`RIS.PLLLRIS` bit 6 + `PLLSTAT.LOCK`
//! bit 0; the analog lock is unobservable to polling firmware, same
//! convention as the 123 model), MOSC enable sequence (`MOSCCTL` reset
//! `PWRDN=1+NOXTAL=1`, p273–274; clear `PWRDN`/`NOXTAL` and stabilize via
//! [`MOSC_STABLE_TICKS`] ticks before select; board crystal fixed 25 MHz),
//! `RCGCn`/`SCGCn`/`DCGCn`/`SRn` latch words, `CAN0MPC` RW reset
//! `0x0000.0003` (Table 5-11), `PWRTC` RW1C, `RIS`/`MISC` RW1C (`MISC` reads
//! `RIS & IMC`, W1C clears `RIS`; `pending()` clears with `MISC`), RO
//! ID/capability constants (`DID0` modeled `0x100A_0001`: `VER=1`,
//! `CLASS=0x0A` Snowflake per datasheet, die revision A1; `DID1`
//! `0x101F_C06E`; `PPn` from Table 5-11, including the genuine `PPWTIMER=0`
//! quirk), `RESC` power-on (`POR` set).
//! Deliberate approximations: `LFIOSC` nominal 33 kHz (§5.2.5.1/Table 5-3
//! note: 33/10/90); `PLLSRC` `0x1–0x2` reserved → `PIOSC` fallback with note
//! (p276: only `0x0`=PIOSC, `0x3`=MOSC); `OSCSRC` `0x1/0x5+` reserved →
//! `PIOSC` fallback with note (p276 — cosmetic, fallback kept);
//! `MOSCPUPRIS`/`MOFRIS`/`BORRIS` analog events never fire except via the
//! modeled MOSC/PLL paths (same rationale as the 123 `MISC` handling).

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
const PWRTC_OFF: u32 = 0x060;
const MOSCCTL_OFF: u32 = 0x07C;
const RSCLKCFG_OFF: u32 = 0x0B0;
const MEMTIM0_OFF: u32 = 0x0C0;
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
const RS_MEMTIMU: u32 = 1 << 31;
/// Oscillator sources (RSCLKCFG p276).
const OSCSRC_PIOSC: u32 = 0x0;
const OSCSRC_LFIOSC: u32 = 0x2;
const OSCSRC_MOSC: u32 = 0x3;
const OSCSRC_RTC: u32 = 0x4;
/// `MOSCCTL` bits (p273–274): `OSCRNG` 4, `PWRDN` 3, `NOXTAL` 2.
const MOSCCTL_PWRDN: u32 = 1 << 3;
const MOSCCTL_NOXTAL: u32 = 1 << 2;
/// `PLLFREQ0` fields.
const FREQ0_PLLPWR: u32 = 1 << 23;
const FREQ0_MFRAC_SHIFT: u32 = 10;
const FREQ0_MFRAC_MASK: u32 = 0x3FF;
const FREQ0_MINT_MASK: u32 = 0x3FF;
/// `PLLFREQ1` fields.
const FREQ1_Q_SHIFT: u32 = 8;
const FREQ1_Q_MASK: u32 = 0x1F;
const FREQ1_N_MASK: u32 = 0x1F;
/// `PLLSTAT.LOCK` bit 0; `RIS` interrupt bits (p261).
const PLLSTAT_LOCK: u32 = 1 << 0;
const RIS_PLLLRIS: u32 = 1 << 6;
const RIS_MOSCPUPRIS: u32 = 1 << 8;

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
/// MOSC stabilize ticks (analog settle unobservable; small-tick model).
const MOSC_STABLE_TICKS: u32 = 64;
/// `PIOSC` rate in Hz (datasheet §5.2.5.1: 16 MHz).
pub const PIOSC_HZ: u32 = 16_000_000;
/// Nominal `LFIOSC` rate in Hz (§5.2.5.1/Table 5-3 note: 33 kHz nom, 10/90).
pub const LFIOSC_HZ: u32 = 33_000;
/// Hibernate RTC oscillator in Hz.
pub const RTC_OSC_HZ: u32 = 32_768;
/// Modeled MOSC crystal in Hz (EK-TM4C1294XL 25 MHz crystal; the frequency
/// is board knowledge, not a register).
pub const MOSC_HZ: u32 = 25_000_000;
/// Maximum TM4C129 system clock in Hz (120 MHz part).
const SYSCLK_MAX_HZ: u32 = 120_000_000;
/// `MEMTIM0` reset (Table 5-11).
const MEMTIM0_RESET: u32 = 0x0030_0030;

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
    (0x0C0, MEMTIM0_RESET),  // MEMTIM0
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
    (0x298, 0x0000_003F),    // CAN0PDS
    (0x29C, 0x0000_0003),    // CAN0MPC (RW)
    (0x2A0, 0x0000_003F),    // CAN1PDS
    (0x2A4, 0x0000_0003),    // CAN1MPC
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

/// Oscillator rate for an `OSCSRC` encoding with MOSC gating.
/// Reserved `0x1/0x5+` fall back to `PIOSC` with note (p276 — cosmetic).
fn osc_hz(src: u32, mosc_ready: bool) -> u32 {
    match src {
        OSCSRC_PIOSC => PIOSC_HZ,
        OSCSRC_LFIOSC => LFIOSC_HZ,
        OSCSRC_MOSC => {
            if mosc_ready {
                MOSC_HZ
            } else {
                PIOSC_HZ
            }
        }
        OSCSRC_RTC => RTC_OSC_HZ,
        // `0x1` and `0x5–0xFF` reserved (p276): PIOSC fallback, cosmetic.
        _ => PIOSC_HZ,
    }
}

/// PLL source rate for a `PLLSRC` encoding with MOSC gating.
/// Only `0x0`=PIOSC and `0x3`=MOSC are valid (p276); `0x1–0x2` reserved →
/// `PIOSC` fallback with note; `0x4+` reserved likewise.
fn pll_src_hz(src: u32, mosc_ready: bool) -> u32 {
    match src {
        0x0 => PIOSC_HZ,
        0x3 => {
            if mosc_ready {
                MOSC_HZ
            } else {
                PIOSC_HZ
            }
        }
        // `0x1–0x2` reserved (p276): PIOSC fallback.
        // `0x4–0xFF` reserved: PIOSC fallback, cosmetic.
        _ => PIOSC_HZ,
    }
}

/// Compute `SYSCLK` in Hz from the `RSCLKCFG`/`PLLFREQ0`/`PLLFREQ1` words
/// (datasheet RSCLKCFG + PLLFREQ pages). `locked` gates the PLL path: an
/// unpowered or not-yet-locked PLL cannot source the clock, so the raw
/// oscillator feeds through (firmware programs the PLL before setting
/// `USEPLL`, per the datasheet procedure). `mosc_ready` gates MOSC
/// (enable sequence + stabilize before select).
#[must_use]
pub fn sysclk_hz_for_129(
    rsclkcfg: u32,
    pllfreq0: u32,
    pllfreq1: u32,
    locked: bool,
    mosc_ready: bool,
) -> u32 {
    let usepll = rsclkcfg & RS_USEPLL != 0;
    let powered = pllfreq0 & FREQ0_PLLPWR != 0;
    let oscsrc = (rsclkcfg >> RS_OSCSRC_SHIFT) & 0xF;
    if usepll && powered && locked {
        let pllsrc = (rsclkcfg >> RS_PLLSRC_SHIFT) & 0xF;
        let fin = u64::from(pll_src_hz(pllsrc, mosc_ready));
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
        u32::try_from(u64::from(osc_hz(oscsrc, mosc_ready)) / (osysdiv + 1)).unwrap_or(PIOSC_HZ)
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
        || offset == 0x298
        || offset == 0x2A0
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
    // `MISC`/`PWRTC` are W1C (handled in `on_write`); `CAN0MPC`/`CAN1MPC`
    // are RW (Table 5-11); the rest latch by default.
    true
}

/// PLL lock timing, MOSC stabilize, NEWFREQ/MEMTIMU defer, and `sysclk_hz`
/// pushes.
#[derive(Debug)]
pub struct Sysctl129Behavior {
    clock: Option<Arc<Mutex<ClockState>>>,
    pll_lockdown: u32,
    locked: bool,
    sysclk: u32,
    active_freq0: u32,
    active_freq1: u32,
    memtim_active: u32,
    memtim_pending: u32,
    mosc_lockdown: u32,
    mosc_stable: bool,
}

impl Default for Sysctl129Behavior {
    fn default() -> Self {
        Self {
            clock: None,
            pll_lockdown: 0,
            locked: false,
            sysclk: PIOSC_HZ,
            active_freq0: 0,
            active_freq1: 0,
            memtim_active: MEMTIM0_RESET,
            memtim_pending: MEMTIM0_RESET,
            mosc_lockdown: 0,
            mosc_stable: false,
        }
    }
}

impl Sysctl129Behavior {
    /// Whether MOSC is enabled for crystal mode (`NOXTAL` clear;
    /// `PWRDN` clear for crystal, set for single-ended — the board crystal
    /// needs both clear).
    fn mosc_enabled(view: &Registers<&mut [u32]>) -> bool {
        let moscctl = view.get(MOSCCTL_OFF);
        moscctl & MOSCCTL_NOXTAL == 0
    }

    /// Whether MOSC is ready (enabled + crystal stabilized).
    /// Single-ended (`PWRDN` set) is immediate; crystal (`PWRDN` clear)
    /// needs [`MOSC_STABLE_TICKS`] (board crystal fixed 25 MHz).
    fn mosc_ready(&self, view: &Registers<&mut [u32]>) -> bool {
        let moscctl = view.get(MOSCCTL_OFF);
        if moscctl & MOSCCTL_NOXTAL != 0 {
            return false;
        }
        if moscctl & MOSCCTL_PWRDN != 0 {
            // Single-ended: ready once NOXTAL clears (no settle).
            return true;
        }
        self.mosc_stable
    }

    /// Recompute lock state and `sysclk` from the current clock words.
    /// Uses the applied (`NEWFREQ`/`MEMTIMU`-gated) `PLLFREQ` values.
    fn recompute(&mut self, view: &mut Registers<&mut [u32]>) {
        let rsclkcfg = view.get(RSCLKCFG_OFF) & !(RS_MEMTIMU | RS_NEWFREQ);
        let powered = self.active_freq0 & FREQ0_PLLPWR != 0;
        let usepll = rsclkcfg & RS_USEPLL != 0;
        if !powered || !usepll {
            self.locked = false;
            self.pll_lockdown = 0;
            // Power-down/bypass clears the lock indication.
            let ris = view.get(RIS_OFF) & !RIS_PLLLRIS;
            view.set(RIS_OFF, ris);
        } else if !self.locked && self.pll_lockdown == 0 {
            self.pll_lockdown = PLL_LOCK_TICKS;
        }
        let ready = self.mosc_ready(view);
        self.sysclk = sysclk_hz_for_129(
            rsclkcfg,
            self.active_freq0,
            self.active_freq1,
            self.locked,
            ready,
        );
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
        let dt = u32::try_from(delta).unwrap_or(u32::MAX);
        if self.pll_lockdown != 0 {
            self.pll_lockdown = self.pll_lockdown.saturating_sub(dt);
            if self.pll_lockdown == 0 {
                self.locked = true;
                let ris = view.get(RIS_OFF) | RIS_PLLLRIS;
                view.set(RIS_OFF, ris);
                let rsclkcfg = view.get(RSCLKCFG_OFF) & !(RS_MEMTIMU | RS_NEWFREQ);
                let ready = self.mosc_ready(view);
                self.sysclk =
                    sysclk_hz_for_129(rsclkcfg, self.active_freq0, self.active_freq1, true, ready);
                self.push_clock();
            }
        }
        if self.mosc_lockdown != 0 {
            self.mosc_lockdown = self.mosc_lockdown.saturating_sub(dt);
            if self.mosc_lockdown == 0 {
                self.mosc_stable = true;
                let ris = view.get(RIS_OFF) | RIS_MOSCPUPRIS;
                view.set(RIS_OFF, ris);
                self.recompute(view);
            }
        }
    }

    fn reset(&mut self) {
        self.pll_lockdown = 0;
        self.locked = false;
        self.sysclk = PIOSC_HZ;
        self.active_freq0 = 0;
        self.active_freq1 = 0;
        self.memtim_active = MEMTIM0_RESET;
        self.memtim_pending = MEMTIM0_RESET;
        self.mosc_lockdown = 0;
        self.mosc_stable = false;
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        // Stored latch masked by live lock/stability, gated by `IMC`.
        let mut full = view.get(RIS_OFF);
        if !self.locked {
            full &= !RIS_PLLLRIS;
        }
        if !self.mosc_stable {
            // Crystal path needs settle; single-ended (`PWRDN` set, `NOXTAL`
            // clear) keeps the latch (set immediately on enable).
            let mc = view.get(MOSCCTL_OFF);
            if mc & MOSCCTL_PWRDN == 0 || mc & MOSCCTL_NOXTAL != 0 {
                full &= !RIS_MOSCPUPRIS;
            }
        }
        u32::from(full & view.get(IMC_OFF) != 0)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == RIS_OFF {
            let mut out = view.get(RIS_OFF);
            if !self.locked {
                out &= !RIS_PLLLRIS;
            }
            if !self.mosc_stable {
                let mc = view.get(MOSCCTL_OFF);
                // Crystal path needs settle; single-ended keeps the latch.
                if mc & MOSCCTL_PWRDN == 0 || mc & MOSCCTL_NOXTAL != 0 {
                    out &= !RIS_MOSCPUPRIS;
                }
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
            // `MISC` reads `RIS & IMC` (p265).
            let ris = self.on_read(view, RIS_OFF, view.get(RIS_OFF));
            return ris & view.get(IMC_OFF);
        }
        if offset == RSCLKCFG_OFF {
            // Bits 31/30 are `R0/W`: read 0, auto-clear (p275).
            return value & !(RS_MEMTIMU | RS_NEWFREQ);
        }
        if offset == MEMTIM0_OFF {
            // Applied value only (writes stage until `MEMTIMU`).
            return self.memtim_active;
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
            // RW1C: writing 1 clears the corresponding `RIS` bit (p265).
            let w1c = u32::from(val) << shift;
            let ris = view.get(RIS_OFF) & !w1c;
            view.set(RIS_OFF, ris);
            return view.get(offset);
        }
        if offset == PWRTC_OFF {
            // RW1C (p270): writing 1 clears, writing 0 has no effect;
            // reserved bits stay 0 (cannot be set in simulation).
            let w1c = u32::from(val) << shift;
            let old = view.get(offset);
            let cleared = old & !w1c;
            // Only the two status bits are writable; mask the rest.
            let masked = cleared & 0x0000_0011;
            view.set(offset, masked);
            return masked;
        }
        if !is_writable(offset) {
            return view.get(offset);
        }
        if offset == MEMTIM0_OFF {
            // Stage until `MEMTIMU`; reads show the applied value.
            // Merge into the pending latch (not the register file, which
            // holds the applied value) so multi-byte writes accumulate.
            let mask = 0xFFu32 << shift;
            let staged = (self.memtim_pending & !mask) | (u32::from(val) << shift);
            self.memtim_pending = staged;
            return self.memtim_active;
        }
        if offset == PLLFREQ0_OFF || offset == PLLFREQ1_OFF {
            // Deferred until `NEWFREQ=1` (p292–293): latch the staged word,
            // do not touch the applied PLL yet.
            let staged = view.merged(offset, val, shift);
            view.set(offset, staged);
            return staged;
        }
        if offset == RSCLKCFG_OFF {
            let old = view.get(offset) & !(RS_MEMTIMU | RS_NEWFREQ);
            let merged = view.merged(offset, val, shift);
            let want_memtimu = merged & RS_MEMTIMU != 0;
            let want_newfreq = merged & RS_NEWFREQ != 0;
            let stored = merged & !(RS_MEMTIMU | RS_NEWFREQ);
            view.set(offset, stored);
            if want_memtimu {
                // `MEMTIMU`: apply `MEMTIM0`, auto-clear (p275–277).
                // Wait-state values stay unmodeled — latch the applied value.
                self.memtim_active = self.memtim_pending;
                view.set(MEMTIM0_OFF, self.memtim_active);
            }
            if want_newfreq {
                // `NEWFREQ`: apply `PLLFREQ0/1` only while powered-down and
                // not sourcing (p292–293 + §5.2.5.5); auto-clear regardless.
                let active_pwr = self.active_freq0 & FREQ0_PLLPWR != 0;
                let old_usepll = old & RS_USEPLL != 0;
                if !active_pwr && !old_usepll {
                    self.active_freq0 = view.get(PLLFREQ0_OFF);
                    self.active_freq1 = view.get(PLLFREQ1_OFF);
                }
            }
            self.recompute(view);
            return stored;
        }
        if offset == MOSCCTL_OFF {
            let old = view.get(offset);
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            let was_enabled = old & MOSCCTL_NOXTAL == 0;
            let now_enabled = word & MOSCCTL_NOXTAL == 0;
            let crystal = word & MOSCCTL_PWRDN == 0;
            if now_enabled && !crystal {
                // Single-ended: ready immediately, set the latch.
                self.mosc_stable = true;
                self.mosc_lockdown = 0;
                let ris = view.get(RIS_OFF) | RIS_MOSCPUPRIS;
                view.set(RIS_OFF, ris);
            } else if now_enabled && crystal && !self.mosc_stable && self.mosc_lockdown == 0 {
                // Crystal: needs settle after the enable sequence.
                if !was_enabled || old & MOSCCTL_PWRDN != 0 {
                    self.mosc_lockdown = MOSC_STABLE_TICKS;
                }
            } else if !now_enabled {
                self.mosc_stable = false;
                self.mosc_lockdown = 0;
                let ris = view.get(RIS_OFF) & !RIS_MOSCPUPRIS;
                view.set(RIS_OFF, ris);
            }
            self.recompute(view);
            return word;
        }
        // `SRn`/`RCGCn`/`SCGCn`/`DCGCn` and remaining RW words latch.
        view.merged(offset, val, shift)
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
        let behavior = self.inner.behavior_mut();
        behavior.active_freq0 = 0;
        behavior.active_freq1 = 0;
        behavior.memtim_active = MEMTIM0_RESET;
        behavior.memtim_pending = MEMTIM0_RESET;
        behavior.mosc_stable = false;
        behavior.mosc_lockdown = 0;
        behavior.locked = false;
        behavior.pll_lockdown = 0;
        behavior.sysclk = PIOSC_HZ;
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
        assert_eq!(sysclk_hz_for_129(0, 0, 0, false, false), 16_000_000);
        // PIOSC / 4.
        let cfg = 3 << RS_OSYSDIV_SHIFT;
        assert_eq!(sysclk_hz_for_129(cfg, 0, 0, false, false), 4_000_000);
        // LFIOSC source (33 kHz nom).
        let cfg = OSCSRC_LFIOSC << RS_OSCSRC_SHIFT;
        assert_eq!(sysclk_hz_for_129(cfg, 0, 0, false, false), 33_000);
    }

    /// The LaunchPad 120 MHz configuration: 25 MHz MOSC → 480 MHz VCO → /4.
    #[test]
    fn pll_120mhz() {
        // MINT=96, MFRAC=0, N=4, Q=0; PSYSDIV=3; PLLSRC=MOSC; USEPLL.
        let freq0 = FREQ0_PLLPWR | 96;
        let freq1 = 4; // N=4, Q=0.
        let cfg = RS_USEPLL | (OSCSRC_MOSC << RS_PLLSRC_SHIFT) | 3;
        assert_eq!(
            sysclk_hz_for_129(cfg, freq0, freq1, true, true),
            120_000_000
        );
        // Same words, unlocked: the raw oscillator feeds through.
        assert_eq!(
            sysclk_hz_for_129(cfg, freq0, freq1, false, true),
            16_000_000
        );
    }

    /// `NEWFREQ` defers `PLLFREQ` until set, auto-clears, and only applies
    /// while powered-down / not sourcing; then 128 ticks lock the PLL.
    #[test]
    fn newfreq_defers_and_autoclears_powered_down_only() {
        let mut sys = sys129();
        // Stage PLLFREQ without NEWFREQ: no lock starts, sysclk stays PIOSC.
        write_word(&mut sys, PLLFREQ1_OFF, 4);
        write_word(&mut sys, PLLFREQ0_OFF, FREQ0_PLLPWR | 96);
        assert_eq!(read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        assert_eq!(sys.sysclk_hz(), 16_000_000);
        sys.tick(200);
        assert_eq!(read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        // NEWFREQ applies (powered-down before, not sourcing) + auto-clears.
        let cfg = RS_USEPLL | (OSCSRC_MOSC << RS_PLLSRC_SHIFT) | 3;
        // Enable MOSC crystal first (board 25 MHz): clear PWRDN+NOXTAL.
        write_word(&mut sys, MOSCCTL_OFF, 0x0000_0000);
        sys.tick(u64::from(MOSC_STABLE_TICKS));
        write_word(&mut sys, RSCLKCFG_OFF, cfg | RS_NEWFREQ);
        assert_eq!(read_word(&mut sys, RSCLKCFG_OFF) & RS_NEWFREQ, 0);
        assert_eq!(read_word(&mut sys, RSCLKCFG_OFF) & RS_MEMTIMU, 0);
        assert_eq!(read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        sys.tick(127);
        assert_eq!(read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK, 0);
        sys.tick(1);
        assert_eq!(
            read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK,
            PLLSTAT_LOCK
        );
        assert_eq!(read_word(&mut sys, RIS_OFF) & RIS_PLLLRIS, RIS_PLLLRIS);
        // Second NEWFREQ while powered + sourcing is ignored (stays locked
        // config, no relock): change staged values, NEWFREQ clears, no drop.
        write_word(&mut sys, PLLFREQ0_OFF, FREQ0_PLLPWR | 32);
        write_word(&mut sys, RSCLKCFG_OFF, cfg | RS_NEWFREQ);
        assert_eq!(read_word(&mut sys, RSCLKCFG_OFF) & RS_NEWFREQ, 0);
        assert_eq!(
            read_word(&mut sys, PLLSTAT_OFF) & PLLSTAT_LOCK,
            PLLSTAT_LOCK
        );
    }

    /// Legacy `newfreq_lock_flow` sequence still locks (powered-down start).
    #[test]
    fn newfreq_lock_flow() {
        let mut sys = sys129();
        let freq0 = FREQ0_PLLPWR | 96;
        write_word(&mut sys, PLLFREQ1_OFF, 4);
        write_word(&mut sys, PLLFREQ0_OFF, freq0);
        write_word(&mut sys, MOSCCTL_OFF, 0);
        sys.tick(u64::from(MOSC_STABLE_TICKS));
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

    /// `MEMTIM0` stages until `MEMTIMU`, which auto-clears (p277).
    #[test]
    fn memtim0_applies_on_memtimu_autoclears() {
        let mut sys = sys129();
        assert_eq!(read_word(&mut sys, MEMTIM0_OFF), MEMTIM0_RESET);
        write_word(&mut sys, MEMTIM0_OFF, 0x0000_1111);
        // Still applied value (staged, not yet applied).
        assert_eq!(read_word(&mut sys, MEMTIM0_OFF), MEMTIM0_RESET);
        write_word(&mut sys, RSCLKCFG_OFF, RS_MEMTIMU);
        assert_eq!(read_word(&mut sys, RSCLKCFG_OFF) & RS_MEMTIMU, 0);
        assert_eq!(read_word(&mut sys, MEMTIM0_OFF), 0x0000_1111);
    }

    /// `RSCLKCFG` bits 31/30 are `R0/W`: read 0, auto-clear (p275).
    #[test]
    fn rsclkcfg_bits_31_30_read0_autoclear() {
        let mut sys = sys129();
        write_word(&mut sys, RSCLKCFG_OFF, RS_MEMTIMU | RS_NEWFREQ | RS_USEPLL);
        let got = read_word(&mut sys, RSCLKCFG_OFF);
        assert_eq!(got & (RS_MEMTIMU | RS_NEWFREQ), 0);
        assert_ne!(got & RS_USEPLL, 0);
    }

    /// `LFIOSC` nominal is 33 kHz (Table 5-3 note).
    #[test]
    fn lfiosc_is_33khz() {
        assert_eq!(LFIOSC_HZ, 33_000);
        let cfg = OSCSRC_LFIOSC << RS_OSCSRC_SHIFT;
        assert_eq!(sysclk_hz_for_129(cfg, 0, 0, false, false), 33_000);
    }

    /// MOSC reset `PWRDN=1+NOXTAL=1` with stabilize path (board 25 MHz).
    #[test]
    fn mosc_reset_and_stabilize_path() {
        let mut sys = sys129();
        assert_eq!(read_word(&mut sys, MOSCCTL_OFF), 0x0000_000C);
        // MOSC source without enable falls back to PIOSC.
        let cfg = OSCSRC_MOSC << RS_OSCSRC_SHIFT;
        assert_eq!(sysclk_hz_for_129(cfg, 0, 0, false, false), PIOSC_HZ);
        // Single-ended (NOXTAL clear, PWRDN set): immediate ready.
        write_word(&mut sys, MOSCCTL_OFF, 0x0000_0008);
        assert_eq!(
            read_word(&mut sys, RIS_OFF) & RIS_MOSCPUPRIS,
            RIS_MOSCPUPRIS
        );
        // Crystal (both clear): needs settle ticks.
        let mut sys2 = sys129();
        write_word(&mut sys2, MOSCCTL_OFF, 0x0000_0000);
        assert_eq!(read_word(&mut sys2, RIS_OFF) & RIS_MOSCPUPRIS, 0);
        sys2.tick(u64::from(MOSC_STABLE_TICKS) - 1);
        assert_eq!(read_word(&mut sys2, RIS_OFF) & RIS_MOSCPUPRIS, 0);
        sys2.tick(1);
        assert_ne!(read_word(&mut sys2, RIS_OFF) & RIS_MOSCPUPRIS, 0);
        // Enabled crystal gives the board 25 MHz.
        let cfg = OSCSRC_MOSC << RS_OSCSRC_SHIFT;
        assert_eq!(sysclk_hz_for_129(cfg, 0, 0, false, true), MOSC_HZ);
    }

    /// `PLLSRC` `0x1–0x2` reserved (p276): fall back to `PIOSC`.
    #[test]
    fn pllsrc_reserved_falls_back() {
        let freq0 = FREQ0_PLLPWR | 96;
        let freq1 = 4;
        for src in [0x1, 0x2] {
            let cfg = RS_USEPLL | (src << RS_PLLSRC_SHIFT) | 3;
            // Reserved → PIOSC reference (16 MHz * 96 / 5 / 4 = 76.8 MHz),
            // not the MOSC-derived 120 MHz.
            let hz = sysclk_hz_for_129(cfg, freq0, freq1, true, true);
            assert_eq!(hz, 76_800_000);
        }
        // Valid MOSC source gives 120 MHz.
        let cfg = RS_USEPLL | (0x3 << RS_PLLSRC_SHIFT) | 3;
        assert_eq!(
            sysclk_hz_for_129(cfg, freq0, freq1, true, true),
            120_000_000
        );
    }

    /// `OSCSRC` `0x1/0x5+` reserved (p276 — cosmetic, PIOSC fallback kept).
    #[test]
    fn oscsrc_reserved_falls_back() {
        for src in [0x1, 0x5, 0x6, 0xF] {
            let cfg = src << RS_OSCSRC_SHIFT;
            assert_eq!(sysclk_hz_for_129(cfg, 0, 0, false, false), PIOSC_HZ);
        }
    }

    /// `CAN0MPC` RW reset `0x0000.0003` (Table 5-11).
    /// Note: the SVD tables lack `0x298–0x2A4`, so the bus reports
    /// `Reserved` there today (SVD fix lands separately); this test asserts
    /// the behavior-side contract (writable + reset) that takes effect once
    /// the tables expose the register.
    #[test]
    fn can0mpc_rw_reset_0003() {
        assert!(is_writable(0x29C));
        assert!(!is_writable(0x298));
        assert!(!is_writable(0x2A0));
        assert!(RESETS.iter().any(|&(o, v)| o == 0x29C && v == 0x0000_0003));
        // Power-domain status stays RO.
        assert!(!is_writable(0x298));
    }

    /// `PWRTC` RW1C (p270).
    #[test]
    fn pwrtc_rw1c() {
        let mut sys = sys129();
        assert_eq!(read_word(&mut sys, PWRTC_OFF), 0);
        // Writing 1 to a clear bit has no effect (cannot set in simulation).
        write_word(&mut sys, PWRTC_OFF, 0x0000_0011);
        assert_eq!(read_word(&mut sys, PWRTC_OFF), 0);
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

    /// `MISC` RW1C reads `RIS & IMC`; W1C clears `RIS` and `pending()`.
    #[test]
    fn misc_imc_flow() {
        let mut sys = sys129();
        write_word(&mut sys, IMC_OFF, 1 << 6); // PLLLMIM
                                               // Stage PLL then NEWFREQ (powered-down start) with MOSC ready.
        write_word(&mut sys, MOSCCTL_OFF, 0);
        sys.tick(u64::from(MOSC_STABLE_TICKS));
        write_word(&mut sys, PLLFREQ1_OFF, 4);
        write_word(&mut sys, PLLFREQ0_OFF, FREQ0_PLLPWR | 96);
        let cfg = RS_USEPLL | (OSCSRC_MOSC << RS_PLLSRC_SHIFT) | 3;
        write_word(&mut sys, RSCLKCFG_OFF, cfg | RS_NEWFREQ);
        sys.tick(128);
        assert_eq!(sys.pending(), 1);
        assert_eq!(read_word(&mut sys, MISC_OFF), 1 << 6);
        // Write 0: no effect.
        write_word(&mut sys, MISC_OFF, 0);
        assert_eq!(sys.pending(), 1);
        write_word(&mut sys, MISC_OFF, 1 << 6);
        assert_eq!(read_word(&mut sys, MISC_OFF), 0);
        assert_eq!(sys.pending(), 0);
        assert_eq!(read_word(&mut sys, RIS_OFF) & RIS_PLLLRIS, 0);
    }
}
