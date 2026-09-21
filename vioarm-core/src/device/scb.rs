//! The ARMv7-M System Control Block (subset).
//!
//! Models the `ICSR` `PendSV` set/clear bits, the `SHPR2`/`SHPR3` configurable
//! exception priorities (`SVCall`, `PendSV`, `SysTick`), the `VTOR` vector
//! table offset, the `SHCSR` system handler control/state register, the
//! `CFSR`/`HFSR` fault status registers, the `CPACR` coprocessor access-control
//! register, and the reset/control registers `AIRCR`, `SCR`, and `CCR`. The
//! `CPACR` value is stored but does not gate VFP execution (the core implements
//! FPv4-SP unconditionally); the window exists so firmware that enables the FPU
//! (as every Cortex-M4 image does) does not fault. `AIRCR.SYSRESETREQ` latches
//! a reset request the machine honors; `SCR.SLEEPONEXIT` and
//! `CCR.STKALIGN`/`DIV_0_TRP` drive the core's exception-return and
//! exception-entry behavior.
//!
//! The ARMv7-M MPU is modeled here too: the `MPU_TYPE`/`MPU_CTRL`/`MPU_RNR`/
//! `MPU_RBAR`/`MPU_RASR` registers (plus their `_A1`/`_A2`/`_A3` aliases —
//! PMSAv7 indexes `MPU_RNR`, PMSAv8 overrides `MPU_RNR[1:0]` via
//! `mpu_v8_region_index`) with eight unified regions,
//! subregion disable, `AP`/`XN` permissions, and the `PRIVDEFENA` background
//! region. [`Scb::mpu_check`] answers permission queries from the core's
//! fetch/data paths, and [`Scb::record_mpu_fault`] latches `MMFSR`, `MMFAR`,
//! and `HFSR.FORCED` when a violation escalates to HardFault. The MPU is
//! bypassed inside `NMI`/`HardFault` unless `HFNMIENA` is set.
//!
//! `VTOR` sets the base address of the exception vector table the core consults
//! on exception entry. `SHCSR` stores the fault-enable and pending bits;
//! `SVCALLPENDED` is wired into the interrupt controller so a written pend is
//! taken like a pended SVC. Its active-status (`*ACT`) bits read zero: the SCB
//! is not told which exception is running, so they are only correct in Thread
//! mode. `CFSR` and `HFSR` are write-1-to-clear fault status; the core records
//! the folding fault causes (`UNDEFINSTR`, `DIVBYZERO`, `PRECISERR`) and the
//! `FORCED` escalation bit through [`crate::mem::bus::PeripheralBus::record_fault`].
//!
//! The `FPCCR` (`0xE000_EF34`) and `FPCAR` (`0xE000_EF38`) windows expose the
//! M-profile lazy-FP-stacking control to firmware. Their state lives in the
//! shared [`FpuCtx`], which the core also holds, so a firmware write is
//! immediately visible to the exception-entry/FP logic and vice versa.
//!
//! Priority encoding follows the architecture: lower value == higher
//! priority, with the `SVCall` byte in `SHPR2` and the `PendSV`/`SysTick`
//! bytes in `SHPR3` (CMSIS `SCB->SHP[7]`, `SHP[10]`, `SHP[11]`). Priority
//! bytes are top-aligned like the NVIC `IPR`: four bits implemented
//! (`0xF0`), unimplemented low bits read zero (see `nvic::PRIO_MASK`).

use crate::cpu::FpuCtx;
use crate::device::nvic::PRIO_MASK;
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::{MpuAccess, MpuLookup, Region, SauHit};
use std::sync::{Arc, Mutex};

const ICSR: u32 = 0xE000_ED04;
const VTOR: u32 = 0xE000_ED08;
const AIRCR: u32 = 0xE000_ED0C;
const SCR: u32 = 0xE000_ED10;
const CCR: u32 = 0xE000_ED14;
const SHPR1: u32 = 0xE000_ED18;
const SHPR2: u32 = 0xE000_ED1C;
const SHPR3: u32 = 0xE000_ED20;
const SHCSR: u32 = 0xE000_ED24;
const CFSR: u32 = 0xE000_ED28;
const HFSR: u32 = 0xE000_ED2C;
const MMFAR: u32 = 0xE000_ED34;
const BFAR: u32 = 0xE000_ED38;
const CPACR: u32 = 0xE000_ED88;
const FPCCR: u32 = 0xE000_EF34;
const FPCAR: u32 = 0xE000_EF38;
const FPDSCR: u32 = 0xE000_EF3C;
/// `DEMCR` (`0xE000_EDFC`): only `TRCENA` (bit 24) is latched. The DWT
/// counters run on their own `CTRL.CYCCNTENA` (matching long-established
/// VioARM behavior and every existing test), so `TRCENA` is storage that
/// lets CMSIS-style `DEMCR |= TRCENA` sequences — and their read-backs —
/// behave like silicon instead of faulting.
const DEMCR: u32 = 0xE000_EDFC;
/// `DEMCR` writable bits: `TRCENA` only. `MON_EN`/`MON_PEND`/`MON_STEP`
/// belong to the unmodeled DebugMonitor exception and stay RAZ/WI.
const DEMCR_WRITABLE: u32 = 1 << 24;

/// `S_VTOR` (`0xE002_ED08`): the ARMv8-M Secure vector table offset register,
/// Secure-only alias of `VTOR`. Firmware relocates the Secure table here when
/// the reset value (address 0) is not loadable; a Non-secure access is
/// RAZ/WI.
const S_VTOR: u32 = 0xE002_ED08;

/// `SFSR` (`0xE002_ED28`): the ARMv8-M `SecureFault` status register. The
/// `INVIS` and `INVER` bits are modelable on the baseline; write-1-to-clear,
/// Non-secure access RAZ/WI.
const SFSR: u32 = 0xE002_ED28;

/// `SFSR.INVIS` (bit 1, QEMU `R_V7M_SFSR_INVIS_MASK`): the callee-saves
/// integrity signature in the stacked exception frame did not match on a
/// Secure exception return, so the `SecureFault` was taken instead.
const SFSR_INVIS: u32 = 0x0000_0002;

/// `SFSR.INVER` (bit 2, QEMU `R_V7M_SFSR_INVER_MASK`): a Non-secure
/// exception return failed the `ES`/`DCRS` validity check, so a `SecureFault`
/// was taken instead.
const SFSR_INVER: u32 = 0x0000_0004;

/// The `SFSR` bits this model can record (`INVIS` and `INVER`).
const SFSR_MASK: u32 = SFSR_INVIS | SFSR_INVER;

/// The ARMv8-M Security Attribution Unit window (`SAU_CTRL`/`SAU_TYPE`/
/// `SAU_RNR`/`SAU_RBAR`/`SAU_RLAR`) with the architectural offsets: the M33
/// conformance tests program the SAU to make the IDAU Non-secure regions
/// effective, and the core's `TT`-family lookup applies the programmed
/// regions through [`Scb::sau_hit`].
const SAU: u32 = 0xE000_EDD0;

/// The MPU register window. One 4-byte-aligned window covers `MPU_TYPE` ..=
/// `MPU_RASR_A3`, dispatched by byte offset; the aliases `_A1`/`_A2`/`_A3`
/// index `MPU_RNR` on PMSAv7 and override `MPU_RNR[1:0]` on PMSAv8 via
/// `mpu_v8_region_index`, matching the QEMU oracle.
const MPU: u32 = 0xE000_ED90;

// MMFSR fault-status bits (CFSR bits 7:0). `IACCVIOL`/`DACCVIOL` are the
// instruction/data access violations the MPU raises; `MMARVALID` marks
// `MMFAR` as holding the faulting address.
const MMFSR_IACCVIOL: u32 = 0x0000_0001;
const MMFSR_DACCVIOL: u32 = 0x0000_0002;
const MMFSR_MMARVALID: u32 = 0x0000_0080;

// BFSR fault-status bits (CFSR bits 15:8). `PRECISERR` marks a precise
// data bus error; `BFARVALID` marks `BFAR` as holding the faulting address.
const BFSR_PRECISERR: u32 = 0x0000_0200;
const BFSR_BFARVALID: u32 = 0x0000_8000;

const PENDSVSET: u8 = 0x10; // ICSR byte 3, bit 28: write 1 to pend.
const PENDSVCLR: u8 = 0x08; // ICSR byte 3, bit 27: write 1 to clear.

// AIRCR: bits 31:16 carry the write `VECTKEY` (0x05FA) and read back as
// `VECTKEYSTAT` (0xFA05); bit 15 `ENDIANNESS` is read-only zero.
const AIRCR_VECTKEY: u32 = 0x05FA_0000;
const AIRCR_PRIGROUP: u32 = 0x0000_0700;
const AIRCR_SYSRESETREQ: u32 = 0x0000_0004;
// AIRCR.BFHFNMINS (bit 1, v8-M): escalated faults target Non-secure
// HardFault instead of Secure. Readable from either state; written only with
// the `VECTKEY`.
const AIRCR_BFHFNMINS: u32 = 0x0000_0002;

// SCR: SEVONPEND (4), SLEEPONEXIT (1). The deep-sleep bits (SLEEPDEEP 2,
// SLEEPDEEPS 3) are RAZ/WI: neither this model nor the QEMU oracle
// implements deep sleep, which the architecture permits.
const SCR_MASK: u32 = 0x0000_0012;

// CCR: STKALIGN (9), BFHFNMIGN (8), DIV_0_TRP (4), UNALIGN_TRP (3),
// USERSETMPEND (1), NONBASETHRDENA (0). STKALIGN is set at reset.
const CCR_MASK: u32 = 0x0000_033B;
const CCR_RESET: u32 = 0x0000_0200;

// SHCSR: the writable bits are the three fault enables (USGFAULTENA 18,
// BUSFAULTENA 17, MEMFAULTENA 16) and the four pend bits (SVCALLPENDED 15,
// BUSFAULTPENDED 14, MEMFAULTPENDED 13, USGFAULTPENDED 12). The active-status
// bits (SYSTICKACT 11, PENDSVACT 10, ...) are read-only and read zero here:
// the SCB is not told which exception is running, so they are only correct in
// Thread mode.
const SHCSR_WRITABLE: u32 = 0x0007_F000;
const SHCSR_SVCALLPENDED: u32 = 0x0000_8000;
const SHCSR_BUSFAULTPENDED: u32 = 0x0000_4000;
const SHCSR_MEMFAULTPENDED: u32 = 0x0000_2000;
const SHCSR_USGFAULTPENDED: u32 = 0x0000_1000;
const SHCSR_BUSFAULTENA: u32 = 0x0002_0000;
const SHCSR_MEMFAULTENA: u32 = 0x0001_0000;
const SHCSR_USGFAULTENA: u32 = 0x0004_0000;

// CFSR combines MMFSR (bits 7:0), BFSR (15:8), and UFSR (31:16); HFSR carries
// FORCED (31) and VECTTBL (1). Both are write-1-to-clear: firmware writes 1s
// to clear the corresponding status, and 0s are ignored.
const CFSR_MASK: u32 = 0xFFFF_FFFF;
// HFSR.FORCED (W1C) is reported at bit 30 to match the QEMU oracle, which
// stores it there instead of the architectural bit 31 (see cpu/mod.rs).
const HFSR_MASK: u32 = 0x4000_0002;
const HFSR_FORCED: u32 = 0x4000_0000;

/// One MPU region, holding the raw `RBAR`/`RASR` words exactly as firmware
/// wrote them. The base is kept in `RBAR` bits `31:5` (firmware's `REGION`/
/// `VALID` bits are consumed on the word write); the region size, subregion
/// disable, and access permissions decode from `RASR`.
#[derive(Debug, Clone, Copy, Default)]
struct MpuRegion {
    rbar: u32,
    rasr: u32,
}

impl MpuRegion {
    /// Whether the region is enabled: `RASR.ENABLE` set and a valid size
    /// field (`SIZE >= 7`, i.e. at least 256 bytes; a smaller `SIZE` makes
    /// `ENABLE` read as zero per the architecture).
    #[must_use]
    const fn enabled(&self) -> bool {
        self.rasr & 0x1 != 0 && (self.rasr >> 1) & 0x1F >= 7
    }

    /// The region size in bytes: `2^(SIZE+1)`.
    #[must_use]
    const fn size(&self) -> u64 {
        1u64 << (((self.rasr >> 1) & 0x1F) + 1)
    }

    /// Whether `addr` falls inside the (base-aligned) region range.
    #[must_use]
    fn contains(&self, addr: u32) -> bool {
        let base = u64::from(self.rbar & !0x1F);
        let a = u64::from(addr);
        a >= base && a < base + self.size()
    }

    /// Whether `addr` falls in a subregion that `RASR.SRD` disables. A
    /// disabled subregion means that address range is not covered by this
    /// region, so lower-numbered regions and the background region apply.
    #[must_use]
    fn subregion_disabled(&self, addr: u32) -> bool {
        let base = u64::from(self.rbar & !0x1F);
        let sub = self.size() / 8;
        let idx = ((u64::from(addr) - base) / sub) as u32;
        (self.rasr >> 8) & (1 << idx) != 0
    }

    /// Whether the region's `AP`/`XN` fields permit `access` for the given
    /// privilege. The caller has already established that `addr` is covered by
    /// this region (and not a disabled subregion), so a denial here is final.
    #[must_use]
    fn permits(&self, access: MpuAccess, privileged: bool) -> bool {
        // XN (RASR bit 28) forbids instruction fetches.
        if access == MpuAccess::Fetch && self.rasr & 0x1000_0000 != 0 {
            return false;
        }
        let ap = (self.rasr >> 24) & 0x7;
        match access {
            MpuAccess::Fetch | MpuAccess::Read => Self::ap_read(ap, privileged),
            MpuAccess::Write => Self::ap_write(ap, privileged),
        }
    }

    /// The AP read-permission matrix (ARMv7-M, PMSAv7): `true` when a read
    /// (or instruction fetch) is allowed in the given privilege.
    const fn ap_read(ap: u32, privileged: bool) -> bool {
        match ap {
            0 => false,      // no access
            1 => privileged, // PRIV rw / UNPRIV none
            2 | 3 => true,   // PRIV rw/UNPRIV ro, rw/rw
            4 => false,      // reserved
            5 => privileged, // PRIV ro / UNPRIV none
            6 | 7 => true,   // ro/ro, PRIV ro / UNPRIV rw
            _ => false,
        }
    }

    /// The AP write-permission matrix: `true` when a store is allowed.
    const fn ap_write(ap: u32, privileged: bool) -> bool {
        match ap {
            1 => privileged,  // PRIV rw / UNPRIV none
            2 => privileged,  // PRIV rw / UNPRIV ro
            3 => true,        // rw/rw
            7 => !privileged, // PRIV ro / UNPRIV rw (per the matrix)
            _ => false,       // AP 0/4/5/6 are reserved: no access either way
        }
    }
}

/// The MPU configuration: control bits, the region-number register, and the
/// eight unified regions.
#[derive(Debug, Clone, Default)]
struct Mpu {
    /// `MPU_CTRL`: `ENABLE` (0), `HFNMIENA` (1), `PRIVDEFENA` (2).
    ctrl: u32,
    /// `MPU_RNR.REGION`: the region indexed by the base `RBAR`/`RASR` writes,
    /// masked to the three bits the eight regions need.
    rnr: u32,
    regions: [MpuRegion; 8],
}

/// The PMSAv8 `AP` permission matrix (QEMU's `simple_ap_to_rw_prot`): the raw
/// read/write bits for the two-bit `AP` field in `MPU_RBAR`, in `privileged`
/// mode.
const fn ap_rw_prot(ap: u32, privileged: bool) -> u8 {
    match ap {
        0 => {
            if privileged {
                crate::mem::bus::PROT_READ | crate::mem::bus::PROT_WRITE
            } else {
                0
            }
        }
        1 => crate::mem::bus::PROT_READ | crate::mem::bus::PROT_WRITE,
        2 => {
            if privileged {
                crate::mem::bus::PROT_READ
            } else {
                0
            }
        }
        _ => crate::mem::bus::PROT_READ,
    }
}

/// The PMSAv8 (v8-M) MPU register file: the control bits and eight regions,
/// banked per security state (index 0 = Non-secure, 1 = Secure).
#[derive(Debug, Clone, Default)]
struct MpuV8 {
    /// `MPU_CTRL` per security state (`ENABLE`/`HFNMIENA`/`PRIVDEFENA`).
    ctrl: [u32; 2],
    /// `MPU_RNR.REGION` per security state.
    rnr: [u32; 2],
    /// `MPU_RBAR` per security state: `XN` (bit 0) and `AP` (bits `2:1`) in
    /// the low byte, the base in bits `31:5`.
    rbar: [[u32; 8]; 2],
    /// `MPU_RLAR` per security state: `ENABLE` (bit 0) and the inclusive
    /// limit in bits `31:5` (32-byte granularity).
    rlar: [[u32; 8]; 2],
}

/// The System Control Block: interrupt control state, exception priorities,
/// the reset/control registers (`AIRCR`/`SCR`/`CCR`), the coprocessor
/// access-control register, and the FP context control.
#[derive(Debug, Clone)]
pub struct Scb {
    ranges: [Region; 22],
    pendsv: bool,
    /// `SHPR1`: configurable priorities for the fault exceptions
    /// (`MemManage` byte 0, `BusFault` byte 1, `UsageFault` byte 2).
    shpr1: u32,
    shpr2: u32,
    shpr3: u32,
    cpacr: u32,
    /// `VTOR`: the vector table base, masked to the 128-byte alignment.
    vtor: u32,
    /// `S_VTOR` (v8-M): the Secure vector table base, masked to the 128-byte
    /// alignment. Resets to zero, so the Secure table lives at address 0
    /// until firmware relocates it.
    s_vtor: u32,
    /// `SFSR` (v8-M): the `SecureFault` status register; the `INVIS` and
    /// `INVER` bits are modelable on the baseline, write-1-to-clear.
    sfsr: u32,
    /// `SHCSR`: fault enables and pend bits; the `*ACT` bits read zero.
    shcsr: u32,
    /// `CFSR`: the combined fault status (MMFSR/BFSR/UFSR), write-1-to-clear.
    cfsr: u32,
    /// `HFSR`: `FORCED`/`VECTTBL`, write-1-to-clear.
    hfsr: u32,
    /// `MMFAR`: the faulting address latched by an MPU access violation. The
    /// core writes it through [`Scb::record_mpu_fault`]; firmware reads it.
    mmfar: u32,
    /// `BFAR`: the faulting address latched by a precise data bus error. The
    /// core writes it through [`Scb::record_bus_fault`]; firmware reads it.
    bfar: u32,
    /// The MPU configuration and region registers.
    mpu: Mpu,
    /// The PMSAv8 (v8-M) MPU register file, banked per security state.
    mpu_v8: MpuV8,
    /// Whether the MPU window and lookups use the PMSAv8 (v8-M) model instead
    /// of the v7-M PMSAv7 model. Disabled by default.
    pmsav8: bool,
    /// The core's current security state, mirrored from the core so the
    /// banked v8-M MPU registers resolve the accessing security state.
    secure: bool,
    /// The in-flight `MPU_RNR` word, so the v8-M out-of-range (`>= 8`) write
    /// gate sees the whole word before deciding.
    mpu_rnr_pending: u32,
    /// The in-flight `MPU_RBAR` word. `RBAR` consumes its whole 32-bit value
    /// on write (`VALID`/`REGION` redirect the store and update `MPU_RNR`),
    /// so the four byte-writes of a word store are accumulated here and
    /// applied when the last byte arrives.
    rbar_pending: u32,
    /// The last `AIRCR` word written (key bytes included), so a 32-bit store's
    /// `VECTKEY` is recognized across its four byte-writes. Reads always
    /// report `VECTKEYSTAT`, never the stored key.
    aircr: u32,
    /// `SCR` (`SEVONPEND`/`SLEEPDEEP`/`SLEEPONEXIT`), written by firmware.
    scr: u32,
    /// `CCR` (`STKALIGN`, `DIV_0_TRP`, ...), written by firmware.
    ccr: u32,
    /// `DEMCR.TRACENA` latch (see `DEMCR`); reset-cleared.
    demcr: u32,
    /// Latched by `AIRCR.SYSRESETREQ`; the machine performs the reset.
    reset_requested: bool,
    /// `SAU_CTRL` (`ENABLE`/`ALLNS`). Stored here and applied to the `TT`
    /// family's security attribution through [`Scb::sau_hit`] and the
    /// `ALLNS`/`ENABLE` bits read by the core.
    sau_ctrl: u32,
    /// `SAU_RNR`: the region index for `SAU_RBAR`/`SAU_RLAR` accesses.
    sau_rnr: u32,
    /// `SAU_RBAR`: the 8 region bases (bits 31:8).
    sau_rbar: [u32; 8],
    /// `SAU_RLAR`: the 8 region limits/attributes (`LIMIT`/`NSC`/`ENABLE`).
    sau_rlar: [u32; 8],
    /// Shared `FPCCR`/`FPCAR`/`FPDSCR` state; the core holds the same `Rc`.
    fpu_ctx: Arc<Mutex<FpuCtx>>,
}

impl Scb {
    /// A new `Scb`.
    #[must_use]
    pub fn new() -> Self {
        Self {
            ranges: [
                Region::new(ICSR, 4),
                Region::new(VTOR, 4),
                Region::new(AIRCR, 4),
                Region::new(SCR, 4),
                Region::new(CCR, 4),
                Region::new(SHPR1, 4),
                Region::new(SHPR2, 4),
                Region::new(SHPR3, 4),
                Region::new(SHCSR, 4),
                Region::new(CFSR, 4),
                Region::new(HFSR, 4),
                Region::new(MMFAR, 4),
                Region::new(BFAR, 4),
                Region::new(CPACR, 4),
                Region::new(MPU, 0x2C),
                Region::new(SAU, 0x18),
                Region::new(FPCCR, 4),
                Region::new(FPCAR, 4),
                Region::new(FPDSCR, 4),
                Region::new(S_VTOR, 4),
                Region::new(SFSR, 4),
                Region::new(DEMCR, 4),
            ],
            pendsv: false,
            shpr1: 0,
            shpr2: 0,
            shpr3: 0,
            cpacr: 0,
            vtor: 0,
            s_vtor: 0,
            sfsr: 0,
            shcsr: 0,
            cfsr: 0,
            hfsr: 0,
            mmfar: 0,
            bfar: 0,
            mpu: Mpu::default(),
            mpu_v8: MpuV8::default(),
            pmsav8: false,
            secure: true,
            mpu_rnr_pending: 0,
            rbar_pending: 0,
            aircr: 0,
            scr: 0,
            ccr: CCR_RESET,
            demcr: 0,
            reset_requested: false,
            sau_ctrl: 0,
            sau_rnr: 0,
            sau_rbar: [0; 8],
            sau_rlar: [0; 8],
            fpu_ctx: Arc::new(Mutex::new(FpuCtx::new())),
        }
    }

    /// Share the `FPCCR`/`FPCAR` state with the core. The core created the
    /// `Rc`; adopting it here makes firmware writes and core logic agree.
    pub fn set_fpu_ctx(&mut self, fpu_ctx: Arc<Mutex<FpuCtx>>) {
        self.fpu_ctx = fpu_ctx;
    }

    /// Switch the MPU window and lookups to the PMSAv8 (v8-M) model, banked
    /// per security state. The default is the v7-M PMSAv7 model.
    pub fn set_pmsav8(&mut self) {
        self.pmsav8 = true;
    }

    /// Mirror the core's current security state, so the banked v8-M MPU
    /// registers resolve the accessing security state.
    pub fn set_secure(&mut self, secure: bool) {
        self.secure = secure;
    }

    /// Whether the `PendSV` line is latched pending.
    #[must_use]
    pub const fn pendsv_pending(&self) -> bool {
        self.pendsv
    }

    /// The configurable priority byte for system exception `n` (`4`, `5`,
    /// `6`, `11`, `14`, `15`), lower meaning higher priority. Unrelated
    /// numbers read zero.
    #[must_use]
    pub const fn priority(&self, n: u8) -> u8 {
        match n {
            4 => (self.shpr1 & 0xFF) as u8,
            5 => ((self.shpr1 >> 8) & 0xFF) as u8,
            6 => ((self.shpr1 >> 16) & 0xFF) as u8,
            11 => ((self.shpr2 >> 24) & 0xFF) as u8,
            14 => ((self.shpr3 >> 16) & 0xFF) as u8,
            15 => ((self.shpr3 >> 24) & 0xFF) as u8,
            _ => 0,
        }
    }

    /// `AIRCR.PRIGROUP` (0..7): the number of subpriority bits the interrupt
    /// controller uses to split each priority byte into group and
    /// subpriority parts. Only visible after a write that carried the
    /// `VECTKEY`.
    #[must_use]
    pub const fn prig(&self) -> u8 {
        if self.aircr & 0xFFFF_0000 == AIRCR_VECTKEY {
            ((self.aircr & AIRCR_PRIGROUP) >> 8) as u8
        } else {
            0
        }
    }

    /// `SCR`: `SEVONPEND`, `SLEEPDEEP`, and `SLEEPONEXIT`.
    #[must_use]
    pub const fn scr(&self) -> u32 {
        self.scr
    }

    /// `CCR`: `STKALIGN`, `BFHFNMIGN`, `DIV_0_TRP`, `UNALIGN_TRP`,
    /// `USERSETMPEND`, and `NONBASETHRDENA`.
    #[must_use]
    pub const fn ccr(&self) -> u32 {
        self.ccr
    }

    /// Whether `AIRCR.SYSRESETREQ` has been written since the last reset.
    #[must_use]
    pub const fn reset_requested(&self) -> bool {
        self.reset_requested
    }

    /// `VTOR`: the vector table base (bits `31:7`, 128-byte aligned).
    #[must_use]
    pub const fn vtor(&self) -> u32 {
        self.vtor
    }

    /// The vector table base for the `secure` bank: `S_VTOR` when this is a
    /// v8-M machine (`PMSAv8`) and the request is for Secure state, else the
    /// plain `VTOR` (the only table on pre-v8-M).
    #[must_use]
    pub const fn vector_table_base(&self, secure: bool) -> u32 {
        if self.pmsav8 && secure {
            self.s_vtor
        } else {
            self.vtor
        }
    }

    /// Whether `AIRCR.BFHFNMINS` is set (v8-M): escalated faults target
    /// `Non-secure HardFault` instead of `Secure`. Only a store carrying the
    /// `VECTKEY` is honored.
    #[must_use]
    pub const fn bfhfnmins(&self) -> bool {
        self.aircr & 0xFFFF_0000 == AIRCR_VECTKEY && self.aircr & AIRCR_BFHFNMINS != 0
    }

    /// Whether `SHCSR.SVCALLPENDED` is set: an SVC is pending and should be
    /// taken by the interrupt controller.
    #[must_use]
    pub const fn svcall_pending(&self) -> bool {
        self.shcsr & SHCSR_SVCALLPENDED != 0
    }

    /// Clear `SHCSR.SVCALLPENDED`, as the interrupt controller does when an
    /// SVC is taken.
    pub fn acknowledge_svcall(&mut self) {
        self.shcsr &= !SHCSR_SVCALLPENDED;
    }

    /// Whether `SHCSR.MEMFAULTENA` is set: `MemManage` is taken as a distinct
    /// exception rather than escalated to `HardFault`.
    #[must_use]
    pub const fn memfault_enabled(&self) -> bool {
        self.shcsr & SHCSR_MEMFAULTENA != 0
    }

    /// Whether `SHCSR.MEMFAULTPENDED` is set: a `MemManage` fault is pended.
    #[must_use]
    pub const fn memfault_pending(&self) -> bool {
        self.shcsr & SHCSR_MEMFAULTPENDED != 0
    }

    /// Clear `SHCSR.MEMFAULTPENDED`, as the interrupt controller does when a
    /// `MemManage` fault is taken.
    pub fn acknowledge_memfault(&mut self) {
        self.shcsr &= !SHCSR_MEMFAULTPENDED;
    }

    /// Whether `SHCSR.BUSFAULTENA` is set: `BusFault` is taken as a distinct
    /// exception rather than escalated to `HardFault`.
    #[must_use]
    pub const fn busfault_enabled(&self) -> bool {
        self.shcsr & SHCSR_BUSFAULTENA != 0
    }

    /// Whether `SHCSR.BUSFAULTPENDED` is set: a `BusFault` is pended.
    #[must_use]
    pub const fn busfault_pending(&self) -> bool {
        self.shcsr & SHCSR_BUSFAULTPENDED != 0
    }

    /// Clear `SHCSR.BUSFAULTPENDED`, as the interrupt controller does when a
    /// `BusFault` is taken.
    pub fn acknowledge_busfault(&mut self) {
        self.shcsr &= !SHCSR_BUSFAULTPENDED;
    }

    /// Whether `SHCSR.USGFAULTENA` is set: `UsageFault` is taken as a distinct
    /// exception rather than escalated to `HardFault`.
    #[must_use]
    pub const fn usage_fault_enabled(&self) -> bool {
        self.shcsr & SHCSR_USGFAULTENA != 0
    }

    /// Whether `SHCSR.USGFAULTPENDED` is set: a `UsageFault` is pended and
    /// should be taken by the interrupt controller once the exception is
    /// enabled and preempts the running context.
    #[must_use]
    pub const fn usage_fault_pending(&self) -> bool {
        self.shcsr & SHCSR_USGFAULTPENDED != 0
    }

    /// Clear `SHCSR.USGFAULTPENDED`, as the interrupt controller does when a
    /// `UsageFault` is taken.
    pub fn acknowledge_usage_fault(&mut self) {
        self.shcsr &= !SHCSR_USGFAULTPENDED;
    }

    /// Record escalated-fault status into `CFSR`/`HFSR`. The core calls this
    /// on the paths where it folds a fault into `HardFault`; firmware reads the
    /// cause through the memory-mapped windows and clears bits write-1-to-clear.
    pub fn record_fault(&mut self, cfsr_bits: u32, hfsr_bits: u32) {
        self.cfsr |= cfsr_bits & CFSR_MASK;
        self.hfsr |= hfsr_bits & HFSR_MASK;
    }

    /// Record a `SecureFault` cause into `SFSR` (`INVIS` and/or `INVER`). The
    /// core calls this when a v8-M exception return fails its integrity
    /// signature check (`INVIS`) or its `ES`/`DCRS` validity check (`INVER`)
    /// and a `SecureFault` is taken instead.
    pub fn record_secure_fault(&mut self, bits: u32) {
        self.sfsr |= bits & SFSR_MASK;
    }

    /// Record an MPU access violation at `addr` (an instruction fetch when
    /// `fetch`): latch `MMFSR.IACCVIOL`/`DACCVIOL` and `MMARVALID` in `CFSR`,
    /// set `HFSR.FORCED`, and store `addr` in `MMFAR`. The core calls this
    /// before escalating to `HardFault`.
    pub fn record_mpu_fault(&mut self, addr: u32, fetch: bool) {
        let cfsr_bits = if fetch {
            MMFSR_IACCVIOL | MMFSR_MMARVALID
        } else {
            MMFSR_DACCVIOL | MMFSR_MMARVALID
        };
        self.cfsr |= cfsr_bits & CFSR_MASK;
        self.hfsr |= HFSR_FORCED & HFSR_MASK;
        self.mmfar = addr;
    }

    /// Record an MPU access violation taken as a distinct `MemManage` fault:
    /// the `MMFSR`/`MMFAR` recording without `HFSR.FORCED`.
    pub fn record_mpu_violation(&mut self, addr: u32, fetch: bool) {
        let cfsr_bits = if fetch {
            MMFSR_IACCVIOL | MMFSR_MMARVALID
        } else {
            MMFSR_DACCVIOL | MMFSR_MMARVALID
        };
        self.cfsr |= cfsr_bits & CFSR_MASK;
        self.mmfar = addr;
    }

    /// Record a precise data bus fault at `addr`: latch `BFSR.PRECISERR` +
    /// `BFARVALID`, store `addr` in `BFAR`, and OR `hfsr_bits` into `HFSR`
    /// (`HFSR.FORCED` when the fault folds into `HardFault`). The core calls
    /// this on the data-access bus-error escalation paths.
    pub fn record_bus_fault(&mut self, addr: u32, hfsr_bits: u32) {
        self.cfsr |= (BFSR_PRECISERR | BFSR_BFARVALID) & CFSR_MASK;
        self.hfsr |= hfsr_bits & HFSR_MASK;
        self.bfar = addr;
    }

    /// Whether the MPU permits the core's `access` to `addr` in the given
    /// privilege and exception context. Disabled by reset, so plain execution
    /// is unaffected until firmware enables it.
    #[must_use]
    pub fn mpu_check(&self, addr: u32, access: MpuAccess, privileged: bool, exception: u8) -> bool {
        if self.pmsav8 {
            return self.mpu_check_v8(addr, access, privileged, exception);
        }
        // `MPU_CTRL.ENABLE` clear: the MPU is off, every access is allowed.
        if self.mpu.ctrl & 0x1 == 0 {
            return true;
        }
        // The private peripheral bus (`0xE000_0000`..`0xE00F_FFFF`) bypasses
        // the MPU and always uses the default map (QEMU's `m_is_ppb_region`).
        if addr & 0xFFF0_0000 == 0xE000_0000 {
            return true;
        }
        // Without `HFNMIENA` the MPU is bypassed inside NMI (2) and HardFault
        // (3), so their handlers always run.
        if self.mpu.ctrl & 0x2 == 0 && (exception == 2 || exception == 3) {
            return true;
        }
        // The highest-numbered region that covers the address decides. A
        // disabled subregion leaves that range uncovered, so lower regions
        // (and the background) still apply.
        for region in self.mpu.regions.iter().rev() {
            if region.enabled() && region.contains(addr) && !region.subregion_disabled(addr) {
                return region.permits(access, privileged);
            }
        }
        // No region matches: `PRIVDEFENA` grants privileged access to the
        // background (the default memory map); unprivileged accesses fault.
        self.mpu.ctrl & 0x4 != 0 && privileged
    }

    /// The `PMSAv8` (v8-M) permission gate. `MPU_CTRL.ENABLE` clear bypasses the
    /// MPU entirely; without `HFNMIENA` the NMI/HardFault handlers bypass it
    /// too. Otherwise the [`Self::mpu_lookup_v8`] result for the current
    /// security state decides, so a no-hit/background fault denies everything.
    /// When the MPU is disabled the lookup reports the default map, whose
    /// execute-never ranges still reject fetches.
    #[must_use]
    fn mpu_check_v8(&self, addr: u32, access: MpuAccess, privileged: bool, exception: u8) -> bool {
        let ctrl = self.mpu_v8.ctrl[usize::from(self.secure)];
        if ctrl & 0x2 == 0 && (exception == 2 || exception == 3) {
            return true;
        }
        let want = match access {
            MpuAccess::Fetch => crate::mem::bus::PROT_EXEC,
            MpuAccess::Read => crate::mem::bus::PROT_READ,
            MpuAccess::Write => crate::mem::bus::PROT_WRITE,
        };
        self.mpu_lookup_v8(addr, self.secure, privileged).prot & want != 0
    }

    /// The `PMSAv8` region lookup for `addr` (QEMU's `pmsav8_mpu_lookup` region
    /// search, in the `secure` MPU bank and `privileged` mode): the permission
    /// bits and matched region. An enabled region hit carries its `AP`/`XN`
    /// permissions; the default map applies when the MPU is disabled or
    /// `PRIVDEFENA` grants the privileged background. A multiple-region hit
    /// and a no-hit-without-background both report no permission and no region
    /// (QEMU's permission/background fault).
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn mpu_lookup_v8(&self, addr: u32, secure: bool, privileged: bool) -> MpuLookup {
        let sec = usize::from(secure);
        let ctrl = self.mpu_v8.ctrl[sec];
        let mut matched = None;
        let mut hit = false;
        if ctrl & 0x1 == 0 {
            // MPU disabled: the default map decides.
            hit = true;
        } else if addr & 0xFFF0_0000 == 0xE000_0000 {
            // The PPB (`0xE000_0000`..`0xE00F_FFFF`) bypasses the MPU.
            hit = true;
        } else {
            // `PRIVDEFENA` grants privileged access to the default map.
            if privileged && ctrl & 0x4 != 0 {
                hit = true;
            }
            for n in (0..8).rev() {
                let rlar = self.mpu_v8.rlar[sec][n];
                if rlar & 1 == 0 {
                    continue;
                }
                let base = self.mpu_v8.rbar[sec][n] & !0x1F;
                let limit = rlar | 0x1F;
                if addr < base || addr > limit {
                    continue;
                }
                if matched.is_some() {
                    // Two or more enabled regions overlap: the lookup fails
                    // (unlike PMSAv7 there is no highest-numbered winner).
                    return MpuLookup {
                        prot: 0,
                        region: None,
                    };
                }
                matched = Some(n);
                hit = true;
            }
        }
        if !hit {
            // No region and no background: a background fault, so nothing is
            // permitted and no region is valid.
            return MpuLookup {
                prot: 0,
                region: None,
            };
        }
        matched.map_or_else(
            || MpuLookup {
                prot: MpuLookup::background_prot(addr),
                region: None,
            },
            |n| {
                let rbar = self.mpu_v8.rbar[sec][n];
                let ap = (rbar >> 1) & 0x3;
                let mut prot = ap_rw_prot(ap, privileged);
                // `XN` (RBAR bit 0) forbids fetches, and the system space
                // (`0xE000_0000`..) is always execute-never; otherwise an
                // accessible region carries execute (QEMU's `prot |= PAGE_EXEC`).
                if prot != 0 && (rbar & 1 == 0 && addr < 0xE000_0000) {
                    prot |= crate::mem::bus::PROT_EXEC;
                }
                MpuLookup {
                    prot,
                    region: Some(n as u8),
                }
            },
        )
    }

    /// Read the MPU register word at byte offset `reg_off` (0, 4, 8, ..) from
    /// the `0xE000_ED90` window base.
    const fn mpu_read(&self, reg_off: u32) -> u32 {
        if self.pmsav8 {
            return self.mpu_v8_read(reg_off);
        }
        match reg_off {
            0x00 => 0x0000_0800, // MPU_TYPE: 8 unified regions
            0x04 => self.mpu.ctrl,
            0x08 => self.mpu.rnr,
            // RBAR and aliases (0xC, 0x14, 0x1C, 0x24): the base in bits
            // [31:5], `VALID` reads zero, and bits [3:0] report the current
            // `MPU_RNR.REGION` (QEMU-consistent).
            0x0C | 0x14 | 0x1C | 0x24 => {
                let r = self.mpu_region_index();
                (self.mpu.regions[r].rbar & !0x1F) | (self.mpu.rnr & 0xF)
            }
            // RASR and aliases (0x10, 0x18, 0x20, 0x28).
            0x10 | 0x18 | 0x20 | 0x28 => {
                let r = self.mpu_region_index();
                self.mpu.regions[r].rasr
            }
            _ => 0,
        }
    }

    /// Write one byte of the MPU register at window offset `reg_off` (the
    /// register word start) and `byte` (0..3 within that word).
    fn mpu_write(&mut self, reg_off: u32, byte: u32, val: u8) {
        if self.pmsav8 {
            self.mpu_v8_write(reg_off, byte, val);
            return;
        }
        let shift = 8 * byte;
        match reg_off {
            // MPU_TYPE is read-only (falls through to the `_` no-op arm).
            0x04 => {
                self.mpu.ctrl = (self.mpu.ctrl & !(0xFF << shift)) | (u32::from(val) << shift);
                self.mpu.ctrl &= 0x7;
            }
            0x08 => {
                self.mpu.rnr = (self.mpu.rnr & !(0xFF << shift)) | (u32::from(val) << shift);
                // Eight regions: only the low three bits are meaningful.
                self.mpu.rnr &= 0x7;
            }
            // RBAR and aliases. A `VALID` set updates `MPU_RNR.REGION` to the
            // word's `REGION` field and selects it; otherwise the current
            // `MPU_RNR.REGION` is used. A `REGION` beyond the implemented
            // eight drops the write, matching the oracle.
            0x0C | 0x14 | 0x1C | 0x24 => {
                self.rbar_pending =
                    (self.rbar_pending & !(0xFF << shift)) | (u32::from(val) << shift);
                if byte == 3 {
                    let w = self.rbar_pending;
                    self.rbar_pending = 0;
                    let region = if w & 0x10 != 0 {
                        let r = w & 0x0F;
                        if r < 8 {
                            self.mpu.rnr = r;
                            r as usize
                        } else {
                            return;
                        }
                    } else {
                        self.mpu_region_index()
                    };
                    self.mpu.regions[region].rbar = w & !0x1F;
                }
            }
            // RASR and aliases: plain read/write storage.
            0x10 | 0x18 | 0x20 | 0x28 => {
                let r = self.mpu_region_index();
                self.mpu.regions[r].rasr =
                    (self.mpu.regions[r].rasr & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            _ => {}
        }
    }

    /// The `MPU_RNR.REGION` index (0..8) used by the base `RBAR`/`RASR`
    /// accesses.
    const fn mpu_region_index(&self) -> usize {
        (self.mpu.rnr & 0x7) as usize
    }

    /// The `PMSAv8` MPU register read at word offset `reg_off` (0, 4, 8, ..),
    /// banked per security state. The `RBAR`/`RLAR` aliases `A1`..`A3` (at
    /// every 8-byte offset) override the low two bits of `MPU_RNR.REGION`,
    /// and there is no `VALID`/`REGION` field in `RBAR` (unlike `PMSAv7`).
    const fn mpu_v8_read(&self, reg_off: u32) -> u32 {
        match reg_off {
            0x00 => 0x0000_0800, // MPU_TYPE: 8 unified regions
            0x04 => self.mpu_v8.ctrl[self.secure as usize],
            0x08 => self.mpu_v8.rnr[self.secure as usize],
            0x0C | 0x14 | 0x1C | 0x24 => {
                let r = self.mpu_v8_region_index(reg_off, 0x0C);
                if r >= 8 {
                    0
                } else {
                    self.mpu_v8.rbar[self.secure as usize][r]
                }
            }
            0x10 | 0x18 | 0x20 | 0x28 => {
                let r = self.mpu_v8_region_index(reg_off, 0x10);
                if r >= 8 {
                    0
                } else {
                    self.mpu_v8.rlar[self.secure as usize][r]
                }
            }
            _ => 0,
        }
    }

    /// Write one byte of a `PMSAv8` MPU register at word offset `reg_off` and
    /// byte `byte`, banked per security state. `MPU_RNR` ignores a
    /// full-word value of at least 8 (an out-of-range region); `RBAR`/`RLAR`
    /// store their raw values, with the 32-byte alignment applied only at
    /// lookup time.
    fn mpu_v8_write(&mut self, reg_off: u32, byte: u32, val: u8) {
        let shift = 8 * byte;
        let sec = usize::from(self.secure);
        match reg_off {
            0x04 => {
                self.mpu_v8.ctrl[sec] =
                    (self.mpu_v8.ctrl[sec] & !(0xFF << shift)) | (u32::from(val) << shift);
                self.mpu_v8.ctrl[sec] &= 0x7;
            }
            0x08 => {
                self.mpu_rnr_pending =
                    (self.mpu_rnr_pending & !(0xFF << shift)) | (u32::from(val) << shift);
                if byte == 3 {
                    let w = self.mpu_rnr_pending;
                    self.mpu_rnr_pending = 0;
                    if w < 8 {
                        self.mpu_v8.rnr[sec] = w;
                    }
                }
            }
            0x0C | 0x14 | 0x1C | 0x24 => {
                let r = self.mpu_v8_region_index(reg_off, 0x0C);
                if r < 8 {
                    self.mpu_v8.rbar[sec][r] =
                        (self.mpu_v8.rbar[sec][r] & !(0xFF << shift)) | (u32::from(val) << shift);
                }
            }
            0x10 | 0x18 | 0x20 | 0x28 => {
                let r = self.mpu_v8_region_index(reg_off, 0x10);
                if r < 8 {
                    self.mpu_v8.rlar[sec][r] =
                        (self.mpu_v8.rlar[sec][r] & !(0xFF << shift)) | (u32::from(val) << shift);
                }
            }
            _ => {}
        }
    }

    /// The v8-M region index for `reg_off`: the base registers use the full
    /// `MPU_RNR.REGION`; the `RBAR`/`RLAR` aliases `A1`..`A3` override the low
    /// two bits of the region number.
    const fn mpu_v8_region_index(&self, reg_off: u32, base: u32) -> usize {
        let rnr = self.mpu_v8.rnr[self.secure as usize];
        let alias = (reg_off - base) / 8;
        let region = if alias == 0 {
            rnr
        } else {
            (rnr & !0x3) | alias
        };
        region as usize
    }

    const fn sau_region_index(&self) -> usize {
        (self.sau_rnr & 0x7) as usize
    }

    /// The SAU region lookup for `addr` (QEMU's `v8m_security_lookup` region
    /// loop, consulted when `SAU_CTRL.ENABLE == 1`): the enabled region
    /// covering `addr`, or `Multiple` when two or more enabled regions
    /// overlap (which collapses to Secure with no valid region number).
    /// [`SauHit::None`] when the SAU is disabled or nothing covers `addr`.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn sau_hit(&self, addr: u32) -> SauHit {
        if self.sau_ctrl & 1 == 0 {
            return SauHit::None;
        }
        let mut hit = SauHit::None;
        for (r, rlar) in self.sau_rlar.iter().enumerate() {
            if rlar & 1 == 0 {
                continue;
            }
            let base = self.sau_rbar[r] & !0x1F;
            let limit = rlar | 0x1F;
            if base <= addr && limit >= addr {
                if hit != SauHit::None {
                    return SauHit::Multiple;
                }
                hit = SauHit::Region {
                    index: r as u8,
                    nsc: rlar & 2 != 0,
                };
            }
        }
        hit
    }

    /// Byte-write to the SAU window at `offset` (from `SAU_CTRL`), using the
    /// architectural layout: `SAU_CTRL`/`SAU_TYPE`/`SAU_RNR`/`SAU_RBAR`/
    /// `SAU_RLAR` at offsets `0`/`4`/`8`/`12`/`16`.
    fn sau_write(&mut self, offset: u32, val: u8) {
        let shift = 8 * (offset & 0x3);
        match offset & !0x3 {
            // Only `ENABLE` and `ALLNS` are implemented.
            0 => {
                self.sau_ctrl = (self.sau_ctrl & !(0xFF << shift)) | (u32::from(val) << shift);
                self.sau_ctrl &= 0x3;
            }
            // `SAU_RNR`: the region index, masked to the eight regions.
            // (`SAU_TYPE` at offset 4 is read-only.)
            8 => {
                self.sau_rnr = (self.sau_rnr & !(0xFF << shift)) | (u32::from(val) << shift);
                self.sau_rnr &= 0x7;
            }
            12 => {
                let idx = self.sau_region_index();
                self.sau_rbar[idx] =
                    (self.sau_rbar[idx] & !(0xFF << shift)) | (u32::from(val) << shift);
                // `RBAR[7:0]` are reserved (bases are 256-byte aligned).
                self.sau_rbar[idx] &= !0xFF;
            }
            16 => {
                let idx = self.sau_region_index();
                self.sau_rlar[idx] =
                    (self.sau_rlar[idx] & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            _ => {}
        }
    }

    fn read_reg(&self, base: u32, offset: u32) -> u8 {
        let byte = offset & 0x3;
        let reg = if base == MPU {
            self.mpu_read(offset & !0x3)
        } else {
            match base {
                ICSR => u32::from(self.pendsv) << 28,
                VTOR => self.vtor,
                // The read reports VECTKEYSTAT; the stored key bytes are never
                // echoed back. `BFHFNMINS` (bit 1) is R/O from Non-secure
                // state and stored from Secure writes, so it always echoes.
                AIRCR => {
                    0xFA05_0000 | (u32::from(self.prig()) << 8) | (u32::from(self.bfhfnmins()) << 1)
                }
                SCR => self.scr,
                CCR => self.ccr,
                SHPR1 => self.shpr1,
                SHPR2 => self.shpr2,
                SHPR3 => self.shpr3,
                SHCSR => self.shcsr,
                CFSR => self.cfsr,
                HFSR => self.hfsr,
                MMFAR => self.mmfar,
                BFAR => self.bfar,
                CPACR => self.cpacr,
                DEMCR => self.demcr, // `S_VTOR`/`SFSR` live in the Secure system window: reads from
                // Non-secure state are RAZ.
                S_VTOR => {
                    if self.secure {
                        self.s_vtor
                    } else {
                        0
                    }
                }
                SFSR => {
                    if self.secure {
                        self.sfsr
                    } else {
                        0
                    }
                }
                SAU => match offset & !0x3 {
                    // `SAU_CTRL` reads back the stored `ENABLE`/`ALLNS` bits.
                    0 => self.sau_ctrl,
                    // `SAU_TYPE`: eight regions on the Cortex-M33.
                    4 => 0x0000_0008,
                    8 => self.sau_rnr,
                    12 => self.sau_rbar[self.sau_region_index()],
                    16 => self.sau_rlar[self.sau_region_index()],
                    _ => 0,
                },
                FPCCR => self
                    .fpu_ctx
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .fpccr(),
                FPCAR => self.fpu_ctx.lock().unwrap_or_else(|e| e.into_inner()).fpcar,
                FPDSCR => {
                    self.fpu_ctx
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .fpdscr
                }
                _ => 0,
            }
        };
        ((reg >> (8 * byte)) & 0xFF) as u8
    }

    fn write_reg(&mut self, base: u32, offset: u32, val: u8) {
        if base == MPU {
            self.mpu_write(offset & !0x3, offset & 0x3, val);
            return;
        }
        match base {
            ICSR => {
                if offset == 3 {
                    // Only the PendSV set/clear bits are modeled.
                    if val & PENDSVSET != 0 {
                        self.pendsv = true;
                    } else if val & PENDSVCLR != 0 {
                        self.pendsv = false;
                    }
                }
            }
            VTOR => {
                let shift = 8 * offset;
                self.vtor = (self.vtor & !(0xFF << shift)) | (u32::from(val) << shift);
                self.vtor &= !0x7F;
            }
            S_VTOR => {
                // Secure-only alias: a Non-secure write is WI.
                if self.secure {
                    let shift = 8 * offset;
                    self.s_vtor = (self.s_vtor & !(0xFF << shift)) | (u32::from(val) << shift);
                    self.s_vtor &= !0x7F;
                }
            }
            SFSR => {
                // Write-1-to-clear, Secure-only (Non-secure writes are WI).
                if self.secure {
                    self.sfsr &= !(u32::from(val) << (8 * offset));
                }
            }
            AIRCR => {
                // Only a store whose bits 31:16 carry `VECTKEY` (0x05FA) is
                // honored. The key bytes may arrive in any of the store's
                // byte-writes, so the last word written (key included) is
                // remembered; reads always report `VECTKEYSTAT`.
                let shift = 8 * offset;
                self.aircr = (self.aircr & !(0xFF << shift)) | (u32::from(val) << shift);
                if self.aircr & 0xFFFF_0000 == AIRCR_VECTKEY {
                    // `VECTRESET` (bit 0) is reserved on the M3/M4 and
                    // `VECTCLRACTIVE` (bit 1) is only meaningful inside a fault
                    // handler; neither is modeled.
                    if self.aircr & AIRCR_SYSRESETREQ != 0 {
                        self.reset_requested = true;
                    }
                }
            }
            SCR => {
                let shift = 8 * offset;
                self.scr = (self.scr & !(0xFF << shift)) | (u32::from(val) << shift);
                self.scr &= SCR_MASK;
            }
            CCR => {
                let shift = 8 * offset;
                self.ccr = (self.ccr & !(0xFF << shift)) | (u32::from(val) << shift);
                self.ccr &= CCR_MASK;
            }
            SHPR1 => {
                let shift = 8 * offset;
                self.shpr1 =
                    (self.shpr1 & !(0xFF << shift)) | (u32::from(val & PRIO_MASK) << shift);
            }
            SHPR2 => {
                let shift = 8 * offset;
                self.shpr2 =
                    (self.shpr2 & !(0xFF << shift)) | (u32::from(val & PRIO_MASK) << shift);
            }
            SHPR3 => {
                let shift = 8 * offset;
                self.shpr3 =
                    (self.shpr3 & !(0xFF << shift)) | (u32::from(val & PRIO_MASK) << shift);
            }
            SHCSR => {
                // The enable and pend bits are plain read/write storage (matching
                // QEMU's storage of `SHCSR`); the read-only active-status bits are
                // not writable and read zero.
                let shift = 8 * offset;
                self.shcsr = (self.shcsr & !(0xFF << shift)) | (u32::from(val) << shift);
                self.shcsr &= SHCSR_WRITABLE;
            }
            CFSR => {
                // Write-1-to-clear: a written 1 clears that fault status bit.
                self.cfsr &= !(u32::from(val) << (8 * offset));
            }
            HFSR => {
                self.hfsr &= !(u32::from(val) << (8 * offset));
            }
            CPACR => {
                let shift = 8 * offset;
                self.cpacr = (self.cpacr & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            DEMCR => {
                // `TRCENA` latches; all other bits (unmodeled
                // DebugMonitor controls) are RAZ/WI.
                let shift = 8 * offset;
                let cur = (self.demcr & !(0xFF << shift)) | (u32::from(val) << shift);
                self.demcr = cur & DEMCR_WRITABLE;
            }
            SAU => self.sau_write(offset, val),
            FPCCR => {
                // Only the ASPEN (bit 7 of byte 3) and LSPEN (bit 6 of byte 3)
                // are software-writable; the ready bits and LSPACT are
                // read-only.
                if offset == 3 {
                    let mut ctx = self.fpu_ctx.lock().unwrap_or_else(|e| e.into_inner());
                    ctx.write_fpccr(u32::from(val) << 24);
                }
            }
            FPCAR => {
                let shift = 8 * offset;
                let mut ctx = self.fpu_ctx.lock().unwrap_or_else(|e| e.into_inner());
                ctx.fpcar = (ctx.fpcar & !(0xFF << shift)) | (u32::from(val) << shift) & !0x7;
            }
            FPDSCR => {
                let shift = 8 * offset;
                let mut ctx = self.fpu_ctx.lock().unwrap_or_else(|e| e.into_inner());
                let raw = (ctx.fpdscr & !(0xFF << shift)) | (u32::from(val) << shift);
                // FPDSCR trap-enable bits (FPSCR bits [12:8] and [15]) are RAZ/WI
                // on the Cortex-M4's FPv4-SP (VFPv3/VFPv4, not U); only AHP/DN/FZ/
                // RMode are writable. Match the QEMU oracle and keep FPSCR's VMSR
                // trap-bit RAZ/WI consistent with the reset value copied on lazy
                // stacking (`fpscr = fpdscr`).
                const FPDSCR_WRITABLE: u32 = (1 << 26) | (1 << 25) | (1 << 24) | (3 << 22);
                ctx.fpdscr = raw & FPDSCR_WRITABLE;
            }
            _ => {}
        }
    }
}

impl Default for Scb {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Scb {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        for r in &self.ranges {
            if r.contains(addr) {
                return Ok(self.read_reg(r.base, addr - r.base));
            }
        }
        Err(BusError::Reserved)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        for r in &self.ranges {
            if r.contains(addr) {
                self.write_reg(r.base, addr - r.base, val);
                return Ok(());
            }
        }
        Err(BusError::Reserved)
    }

    fn reset(&mut self) {
        self.pendsv = false;
        self.shpr1 = 0;
        self.shpr2 = 0;
        self.shpr3 = 0;
        self.cpacr = 0;
        self.vtor = 0;
        self.s_vtor = 0;
        self.sfsr = 0;
        self.shcsr = 0;
        self.cfsr = 0;
        self.hfsr = 0;
        self.mmfar = 0;
        self.mpu = Mpu::default();
        self.mpu_v8 = MpuV8::default();
        self.secure = true;
        self.mpu_rnr_pending = 0;
        self.rbar_pending = 0;
        self.aircr = 0;
        self.scr = 0;
        self.ccr = CCR_RESET;
        self.demcr = 0;
        self.reset_requested = false;
        self.sau_ctrl = 0;
        self.sau_rnr = 0;
        self.sau_rbar = [0; 8];
        self.sau_rlar = [0; 8];
        *self.fpu_ctx.lock().unwrap_or_else(|e| e.into_inner()) = FpuCtx::new();
    }

    fn name(&self) -> &'static str {
        "scb"
    }
}

impl InterruptSource for Scb {
    fn exception_number(&self) -> Option<u8> {
        Some(14)
    }

    fn pending(&self) -> u32 {
        u32::from(self.pendsv)
    }

    fn acknowledge(&mut self, _lines: u32) {
        self.pendsv = false;
    }
}

#[cfg(test)]
mod tests {
    use super::Scb;
    use crate::cpu::FpuCtx;
    use crate::device::Peripheral;
    use crate::mem::bus::{MpuAccess, MpuLookup, SauHit};
    use std::sync::{Arc, Mutex};

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(s: &mut Scb, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            s.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(s: &mut Scb, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = s.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn icsr_pendsv_set_clear_roundtrip() {
        let mut s = Scb::new();
        assert!(!s.pendsv_pending());
        // ICSR byte 3, bit 4 (PENDSVSET).
        write_word(&mut s, 0xE000_ED04, 0x1000_0000);
        assert!(s.pendsv_pending());
        assert_eq!(read_word(&mut s, 0xE000_ED04), 0x1000_0000);
        // ICSR byte 3, bit 3 (PENDSVCLR).
        write_word(&mut s, 0xE000_ED04, 0x0800_0000);
        assert!(!s.pendsv_pending());
    }

    #[test]
    fn shpr_priorities_stored_per_exception() {
        let mut s = Scb::new();
        // SHPR2 byte 3 -> SVCall; SHPR3 byte 3 -> SysTick; SHPR3 byte 2 -> PendSV.
        s.write(0xE000_ED1F, 0x40).unwrap();
        s.write(0xE000_ED23, 0x20).unwrap();
        s.write(0xE000_ED22, 0x80).unwrap();
        assert_eq!(s.priority(11), 0x40);
        assert_eq!(s.priority(15), 0x20);
        assert_eq!(s.priority(14), 0x80);
        assert_eq!(read_word(&mut s, 0xE000_ED1C), 0x4000_0000);
        assert_eq!(read_word(&mut s, 0xE000_ED20), 0x2080_0000);
    }

    #[test]
    fn reset_clears_pend_and_priorities() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED04, 0x1000_0000);
        write_word(&mut s, 0xE000_ED1F, 0x10);
        s.reset();
        assert!(!s.pendsv_pending());
        assert_eq!(s.priority(11), 0);
    }

    #[test]
    fn shpr1_stores_usage_fault_priority() {
        let mut s = Scb::new();
        // SHPR1 byte 2 -> UsageFault (bits 23:16); byte 0 -> MemManage.
        s.write(0xE000_ED1A, 0x60).unwrap();
        assert_eq!(s.priority(6), 0x60);
        assert_eq!(read_word(&mut s, 0xE000_ED18), 0x0060_0000);
        s.write(0xE000_ED18, 0x50).unwrap();
        assert_eq!(s.priority(4), 0x50);
        assert_eq!(read_word(&mut s, 0xE000_ED18), 0x0060_0050);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED18), 0);
        assert_eq!(s.priority(6), 0);
        assert_eq!(s.priority(4), 0);
    }

    #[test]
    fn shcsr_usage_fault_enable_and_pend() {
        let mut s = Scb::new();
        assert!(!s.usage_fault_enabled());
        assert!(!s.usage_fault_pending());
        // Pend and enable through the SHCSR window.
        write_word(&mut s, 0xE000_ED24, 0x0004_1000);
        assert!(s.usage_fault_pending());
        assert!(s.usage_fault_enabled());
        // Acknowledge (the interrupt controller taking the exception) clears
        // the pend but leaves the enable.
        s.acknowledge_usage_fault();
        assert!(!s.usage_fault_pending());
        assert!(s.usage_fault_enabled());
        s.reset();
        assert!(!s.usage_fault_pending());
        assert!(!s.usage_fault_enabled());
    }

    #[test]
    fn cpacr_stores_and_resets() {
        let mut s = Scb::new();
        // Full access to CP10/CP11, as firmware enabling the FPU writes.
        write_word(&mut s, 0xE000_ED88, 0x00F0_0000);
        assert_eq!(read_word(&mut s, 0xE000_ED88), 0x00F0_0000);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED88), 0);
    }

    #[test]
    fn demcr_trcena_latches_and_resets() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_EDFC), 0);
        // The CMSIS `DEMCR |= TRCENA` sequence plus a DebugMonitor bit.
        write_word(&mut s, 0xE000_EDFC, 0x0101_0000);
        assert_eq!(
            read_word(&mut s, 0xE000_EDFC),
            0x0100_0000,
            "TRCENA latches, MON_EN stays RAZ/WI"
        );
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_EDFC), 0);
    }

    #[test]
    fn fpccr_resets_with_ready_bits_and_lazy_on() {
        let mut s = Scb::new();
        // HFRDY|BFRDY|MMRDY|MONRDY|UFRDY plus ASPEN|LSPEN = 0xC7800004.
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0xC780_0004);
        // FPCAR resets to zero.
        assert_eq!(read_word(&mut s, 0xE000_EF38), 0);
    }

    #[test]
    fn fpccr_aspen_lspen_are_software_writable() {
        let mut s = Scb::new();
        // Clear LSPEN (bit 30): ASPEN stays on.
        write_word(&mut s, 0xE000_EF34, 0x8780_0004);
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x8780_0004);
        // Clear ASPEN too: only the ready bits remain.
        write_word(&mut s, 0xE000_EF34, 0x0780_0004);
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x0780_0004);
        // LSPACT (bit 0) is read-only: firmware writes to the low bytes are
        // ignored and never appear in the read value.
        write_word(&mut s, 0xE000_EF34, 0x0000_0001);
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x0780_0004);
    }

    #[test]
    fn fpcar_aligns_and_stores() {
        let mut s = Scb::new();
        // FPCAR holds bits [31:3]; the low three bits read back as zero.
        write_word(&mut s, 0xE000_EF38, 0x2000_083F);
        assert_eq!(read_word(&mut s, 0xE000_EF38), 0x2000_0838);
    }

    #[test]
    fn fpdscr_stores_and_resets() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0);
        // Round-towards-zero: FPSCR RMODE bits 23:22 = 0b11.
        write_word(&mut s, 0xE000_EF3C, 0x00C0_0000);
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0x00C0_0000);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0);
    }

    #[test]
    fn fpdscr_trap_bits_are_raz_wi() {
        let mut s = Scb::new();
        // FPSCR trap enables (bits 12:8 and 15) are RAZ/WI on FPv4-SP.
        write_word(&mut s, 0xE000_EF3C, 0x0000_1F00 | 0x0000_8000);
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0, "trap bits RAZ/WI");
        write_word(&mut s, 0xE000_EF3C, 0x00C0_0000 | 0x0000_1F00 | 0x0000_8000);
        assert_eq!(
            read_word(&mut s, 0xE000_EF3C),
            0x00C0_0000,
            "RMODE kept, traps dropped"
        );
        // AHP/DN/FZ are writable; DN+FZ round-trips.
        write_word(&mut s, 0xE000_EF3C, 0x0300_0000);
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0x0300_0000);
        write_word(&mut s, 0xE000_EF3C, 0x0400_0000);
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0x0400_0000, "AHP");
    }

    #[test]
    fn fpu_ctx_is_shared_with_the_core() {
        let fpu_ctx = Arc::new(Mutex::new(FpuCtx::new()));
        let mut s = Scb::new();
        s.set_fpu_ctx(fpu_ctx.clone());
        // Firmware clears LSPEN through the FPCCR window; the core sees it.
        write_word(&mut s, 0xE000_EF34, 0x8780_0004);
        assert!(!fpu_ctx.lock().unwrap_or_else(|e| e.into_inner()).lspen);
        assert!(fpu_ctx.lock().unwrap_or_else(|e| e.into_inner()).aspen);
        // Hardware sets LSPACT; the FPCCR read reflects it.
        fpu_ctx.lock().unwrap_or_else(|e| e.into_inner()).lspact = true;
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x8780_0005);
        // And the FPCCR write survives reset only until `reset()`.
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0xC780_0004);
        assert!(fpu_ctx.lock().unwrap_or_else(|e| e.into_inner()).lspen);
    }

    #[test]
    fn aircr_requires_vectkey_and_stores_prigroup() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
        assert_eq!(s.prig(), 0);
        // A write without VECTKEY is ignored entirely.
        write_word(&mut s, 0xE000_ED0C, 0x0000_0700);
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
        assert_eq!(s.prig(), 0);
        // With VECTKEY the PRIGROUP field is stored; reads show VECTKEYSTAT.
        write_word(&mut s, 0xE000_ED0C, 0x05FA_0700);
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0700);
        assert_eq!(s.prig(), 7);
    }

    #[test]
    fn aircr_sysresetreq_latches_only_under_vectkey() {
        let mut s = Scb::new();
        // No key: the request never latches.
        write_word(&mut s, 0xE000_ED0C, 0x0000_0004);
        assert!(!s.reset_requested());
        // Properly keyed: SYSRESETREQ latches a reset request.
        write_word(&mut s, 0xE000_ED0C, 0x05FA_0004);
        assert!(s.reset_requested());
        // `reset()` clears it, along with the stored PRIGROUP.
        s.reset();
        assert!(!s.reset_requested());
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
    }

    #[test]
    fn scr_stores_masked_and_resets() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_ED10), 0);
        // SEVONPEND (4) and SLEEPONEXIT (1) are the implemented bits.
        write_word(&mut s, 0xE000_ED10, 0x0000_0012);
        assert_eq!(read_word(&mut s, 0xE000_ED10), 0x0000_0012);
        assert_eq!(s.scr(), 0x0000_0012);
        // The deep-sleep bits (SLEEPDEEP 2, SLEEPDEEPS 3) are RAZ/WI.
        write_word(&mut s, 0xE000_ED10, 0x0000_001E);
        assert_eq!(read_word(&mut s, 0xE000_ED10), 0x0000_0012);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED10), 0);
    }

    #[test]
    fn ccr_resets_with_stkalign_and_masks_writes() {
        let mut s = Scb::new();
        // STKALIGN is set at reset.
        assert_eq!(read_word(&mut s, 0xE000_ED14), 0x0000_0200);
        assert_eq!(s.ccr(), 0x0000_0200);
        // Write DIV_0_TRP only: STKALIGN is cleared, DIV_0_TRP sticks.
        write_word(&mut s, 0xE000_ED14, 0x0000_0010);
        assert_eq!(read_word(&mut s, 0xE000_ED14), 0x0000_0010);
        assert_eq!(s.ccr(), 0x0000_0010);
        // Reserved bits (here bit 7) are dropped on write.
        write_word(&mut s, 0xE000_ED14, 0x0000_0090);
        assert_eq!(read_word(&mut s, 0xE000_ED14), 0x0000_0010);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED14), 0x0000_0200);
    }

    #[test]
    fn vtor_aligns_and_stores() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_ED08), 0);
        // The base must be 128-byte aligned: the low seven bits are masked.
        write_word(&mut s, 0xE000_ED08, 0x2000_00B5);
        assert_eq!(read_word(&mut s, 0xE000_ED08), 0x2000_0080);
        assert_eq!(s.vtor(), 0x2000_0080);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED08), 0);
    }

    #[test]
    fn svtor_sfsr_are_secure_gated_windows() {
        let mut s = Scb::new();
        // Reset: both read zero.
        assert_eq!(read_word(&mut s, 0xE002_ED08), 0);
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0);
        // Secure access (reset state): S_VTOR stores 128-byte aligned.
        write_word(&mut s, 0xE002_ED08, 0x1000_00F3);
        assert_eq!(read_word(&mut s, 0xE002_ED08), 0x1000_0080);
        // SFSR is write-1-to-clear like the other fault-status registers: a
        // firmware write can only clear, never set.
        write_word(&mut s, 0xE002_ED28, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0);
        // The core records the INVER cause; it reads back until cleared.
        s.record_secure_fault(0x0000_0004);
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0x0000_0004);
        write_word(&mut s, 0xE002_ED28, 0x0000_0004);
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0);
        // The INVIS cause (callee-saves signature mismatch) is also latched,
        // and the two bits combine.
        s.record_secure_fault(0x0000_0002);
        s.record_secure_fault(0x0000_0004);
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0x0000_0006);
        write_word(&mut s, 0xE002_ED28, 0x0000_0006);
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0);
        // Non-secure access is RAZ/WI: writes are ignored, reads read zero.
        s.record_secure_fault(0x0000_0004);
        s.set_secure(false);
        write_word(&mut s, 0xE002_ED08, 0x2000_0000);
        write_word(&mut s, 0xE002_ED28, 0xFFFF_FFFF);
        assert_eq!(
            read_word(&mut s, 0xE002_ED08),
            0,
            "S_VTOR reads zero from NS"
        );
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0, "SFSR reads zero from NS");
        // The stored values are preserved through the Non-secure accesses.
        s.set_secure(true);
        assert_eq!(read_word(&mut s, 0xE002_ED08), 0x1000_0080);
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0x0000_0004);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE002_ED08), 0);
        assert_eq!(read_word(&mut s, 0xE002_ED28), 0);
    }

    #[test]
    fn vector_table_base_selects_the_banked_vtor() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED08, 0x2000_0000);
        write_word(&mut s, 0xE002_ED08, 0x1000_0000);
        // Pre-v8-M (PMSAv8 off): there is only one table, so the Secure bank
        // answers VTOR like the Non-secure one.
        assert_eq!(s.vector_table_base(false), 0x2000_0000);
        assert_eq!(s.vector_table_base(true), 0x2000_0000);
        // v8-M: the Secure bank uses S_VTOR.
        s.set_pmsav8();
        assert_eq!(s.vector_table_base(false), 0x2000_0000);
        assert_eq!(s.vector_table_base(true), 0x1000_0000);
    }

    #[test]
    fn aircr_bfhfnmins_is_vectkey_gated_and_echoed() {
        let mut s = Scb::new();
        assert!(!s.bfhfnmins());
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
        // A store without the VECTKEY does not set BFHFNMINS.
        write_word(&mut s, 0xE000_ED0C, 0x0000_0002);
        assert!(!s.bfhfnmins());
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
        // With the VECTKEY the bit is honored and echoes in the read.
        write_word(&mut s, 0xE000_ED0C, 0x05FA_0002);
        assert!(s.bfhfnmins());
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0002);
        // Clearing it again (VECTKEY carried) drops it from the read.
        write_word(&mut s, 0xE000_ED0C, 0x05FA_0000);
        assert!(!s.bfhfnmins());
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
        s.reset();
        assert!(!s.bfhfnmins());
    }

    #[test]
    fn shcsr_stores_enables_and_pends_svcall() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_ED24), 0);
        assert!(!s.svcall_pending());
        // Enable the folded fault exceptions and pend an SVC.
        write_word(&mut s, 0xE000_ED24, 0x0007_8000);
        assert_eq!(read_word(&mut s, 0xE000_ED24), 0x0007_8000);
        assert!(s.svcall_pending());
        // The active-status bits are read-only and read zero: writing
        // SYSTICKACT (bit 11, 0x800) is ignored, the pend/enable bits stay.
        write_word(&mut s, 0xE000_ED24, 0x0007_C800);
        assert_eq!(read_word(&mut s, 0xE000_ED24), 0x0007_C000);
        // Clearing SVCALLPENDED drops the pend (the controller does this on
        // acknowledge; firmware may also write it).
        write_word(&mut s, 0xE000_ED24, 0x0007_0000);
        assert!(!s.svcall_pending());
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED24), 0);
    }

    #[test]
    fn cfsr_and_hfsr_are_write_one_to_clear() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED2C), 0);
        // Firmware cannot set a status bit by writing: writes only clear.
        write_word(&mut s, 0xE000_ED28, 0xFFFF_FFFF);
        write_word(&mut s, 0xE000_ED2C, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED2C), 0);
        // The core records a folded fault; the windows then report it.
        s.record_fault(0x0200_0000, 0x4000_0000); // DIVBYZERO | FORCED
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0x0200_0000);
        assert_eq!(read_word(&mut s, 0xE000_ED2C), 0x4000_0000);
        // Clearing is write-1-to-clear; the untouched bits survive.
        s.record_fault(0x0000_0100, 0); // also PRECISERR
        s.write(0xE000_ED2B, 0x02).unwrap(); // clear DIVBYZERO (bit 25)
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0x0000_0100);
        s.write(0xE000_ED2F, 0x40).unwrap(); // clear FORCED (bit 30)
        assert_eq!(read_word(&mut s, 0xE000_ED2C), 0);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED2C), 0);
    }

    #[test]
    fn mpu_type_reports_eight_unified_regions() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_ED90), 0x0000_0800);
        // MPU_TYPE is read-only.
        write_word(&mut s, 0xE000_ED90, 0);
        assert_eq!(read_word(&mut s, 0xE000_ED90), 0x0000_0800);
    }

    #[test]
    fn mpu_rbar_valid_redirects_and_updates_rnr() {
        let mut s = Scb::new();
        // RNR.REGION is three bits wide for eight regions: 0xA masks to 0x2.
        write_word(&mut s, 0xE000_ED98, 0x0A);
        assert_eq!(read_word(&mut s, 0xE000_ED98), 0x2);
        // An RBAR write with VALID selects the REGION field and updates RNR.
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000 | 0x10 | 0x1);
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0x2000_0000 | 0x1);
        assert_eq!(read_word(&mut s, 0xE000_ED98), 0x1);
        // A REGION field beyond the eight is dropped: RNR and the target
        // region's base stay untouched.
        write_word(&mut s, 0xE000_ED9C, 0x1000_0000 | 0x10 | 0x8);
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0x2000_0000 | 0x1);
        assert_eq!(read_word(&mut s, 0xE000_ED98), 0x1);
    }

    #[test]
    fn mpu_aliases_index_the_rnr_region_like_the_base_registers() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED98, 0x3);
        write_word(&mut s, 0xE000_EDAC, 0x4000_0000); // RBAR_A2, region 3
        write_word(&mut s, 0xE000_EDB0, 0x0300_0013); // RASR_A2, region 3
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0x4000_0000 | 0x3);
        assert_eq!(read_word(&mut s, 0xE000_EDA0), 0x0300_0013);
    }

    #[test]
    fn mpu_disabled_allows_everything() {
        let mut s = Scb::new();
        // Region 0 configured but the MPU left disabled: every access passes.
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000);
        write_word(&mut s, 0xE000_EDA0, 0x0300_0013);
        assert!(s.mpu_check(0x1000_0000, MpuAccess::Read, true, 0));
        assert!(s.mpu_check(0x1000_0000, MpuAccess::Write, false, 0));
        assert!(s.mpu_check(0x1000_0000, MpuAccess::Fetch, true, 0));
    }

    #[test]
    fn mpu_region_grants_within_and_denies_without() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED94, 0x1); // ENABLE, no PRIVDEFENA
                                              // Region 0: 0x2000_0000, 1 KiB (SIZE 9), rw/rw (AP 011).
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000);
        write_word(&mut s, 0xE000_EDA0, 0x0300_0013);
        assert!(s.mpu_check(0x2000_0004, MpuAccess::Read, true, 0));
        assert!(s.mpu_check(0x2000_0004, MpuAccess::Write, false, 0));
        // Past the region end, with no background, everything faults.
        assert!(!s.mpu_check(0x2000_1000, MpuAccess::Read, true, 0));
        assert!(!s.mpu_check(0x1000_0000, MpuAccess::Read, true, 0));
        assert!(!s.mpu_check(0x1000_0000, MpuAccess::Write, false, 0));
        // Disabling the MPU re-opens the map.
        write_word(&mut s, 0xE000_ED94, 0);
        assert!(s.mpu_check(0x1000_0000, MpuAccess::Read, true, 0));
    }

    #[test]
    fn mpu_ap_xn_gate_privileged_and_fetch_access() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED94, 0x1);
        // Region 0: AP 001 (PRIV rw / UNPRIV none) plus XN (no fetch).
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000);
        write_word(&mut s, 0xE000_EDA0, 0x1100_0013);
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Read, true, 0));
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Write, true, 0));
        assert!(!s.mpu_check(0x2000_0000, MpuAccess::Read, false, 0));
        assert!(!s.mpu_check(0x2000_0000, MpuAccess::Write, false, 0));
        assert!(!s.mpu_check(0x2000_0000, MpuAccess::Fetch, true, 0));
    }

    #[test]
    fn mpu_subregions_and_background_region() {
        let mut s = Scb::new();
        // ENABLE | PRIVDEFENA; no region yet.
        write_word(&mut s, 0xE000_ED94, 0x5);
        // Region 0: 1 KiB, SRD disables subregion 0 (0x2000_0000..0x7F).
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000);
        write_word(&mut s, 0xE000_EDA0, 0x0300_0113);
        // A disabled subregion falls through to the background region:
        // privileged allowed, unprivileged denied.
        assert!(s.mpu_check(0x2000_0040, MpuAccess::Read, true, 0));
        assert!(!s.mpu_check(0x2000_0040, MpuAccess::Read, false, 0));
        // A covered subregion uses the region's rw/rw permissions.
        assert!(s.mpu_check(0x2000_0080, MpuAccess::Read, true, 0));
        assert!(s.mpu_check(0x2000_0080, MpuAccess::Write, false, 0));
        // Without PRIVDEFENA the background denies even privileged access.
        write_word(&mut s, 0xE000_ED94, 0x1);
        assert!(!s.mpu_check(0x1000_0000, MpuAccess::Read, true, 0));
    }

    #[test]
    fn mpu_bypassed_in_nmi_and_hardfault_unless_hfnmiena() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED94, 0x1); // ENABLE only
                                              // Unmapped address faults in Thread mode but is bypassed in NMI (2)
                                              // and HardFault (3).
        assert!(!s.mpu_check(0x1000_0000, MpuAccess::Read, true, 0));
        assert!(s.mpu_check(0x1000_0000, MpuAccess::Read, true, 2));
        assert!(s.mpu_check(0x1000_0000, MpuAccess::Read, true, 3));
        // HFNMIENA set: the bypass is gone.
        write_word(&mut s, 0xE000_ED94, 0x3);
        assert!(!s.mpu_check(0x1000_0000, MpuAccess::Read, true, 3));
    }

    #[test]
    fn mpu_fault_records_mmfar_and_mmfsr() {
        let mut s = Scb::new();
        s.record_mpu_fault(0x2000_0040, false);
        assert_eq!(read_word(&mut s, 0xE000_ED34), 0x2000_0040);
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0x0000_0082); // DACCVIOL|MMARVALID
        assert_eq!(read_word(&mut s, 0xE000_ED2C), 0x4000_0000); // FORCED
        s.record_mpu_fault(0x0800_0000, true);
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0x0000_0083); // + IACCVIOL
        assert_eq!(read_word(&mut s, 0xE000_ED34), 0x0800_0000);
        // Both MMFSR bits clear write-1-to-clear through the CFSR window.
        s.write(0xE000_ED28, 0x83).unwrap();
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0);
    }

    #[test]
    fn v7m_mpu_bypasses_ppb() {
        // With the MPU enabled and no region covering `0xE000_0000`..,
        // private-peripheral-bus accesses must still go through: the MPU never
        // applies to them (QEMU's `m_is_ppb_region` -> default map), so a
        // fault handler can always read `CFSR`/`HFSR`/`MMFAR`.
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000); // RBAR: region 0
        write_word(&mut s, 0xE000_EDA0, 0x0300_0015); // RASR: 2 KiB rw/rw
        write_word(&mut s, 0xE000_ED94, 0x1); // MPU_CTRL: ENABLE
        assert!(
            s.mpu_check(0xE000_ED28, MpuAccess::Read, true, 0),
            "PPB read bypasses the MPU"
        );
        assert!(
            !s.mpu_check(0x2000_1000, MpuAccess::Read, true, 0),
            "a normal unmapped access is still denied"
        );
    }

    #[test]
    fn mpu_resets_to_disabled() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED94, 0x7);
        write_word(&mut s, 0xE000_ED98, 0x3);
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000 | 0x10);
        s.record_mpu_fault(0x2000_0040, false);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED90), 0x0000_0800); // TYPE unchanged
        assert_eq!(read_word(&mut s, 0xE000_ED94), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED98), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED34), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED28), 0);
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Read, true, 0));
    }

    #[test]
    fn sau_registers_store_and_read_back() {
        let mut s = Scb::new();
        // Reset: CTRL clear, TYPE reads eight regions, RNR/RBAR/RLAR read zero.
        assert_eq!(read_word(&mut s, 0xE000_EDD0), 0);
        assert_eq!(read_word(&mut s, 0xE000_EDD4), 0x0000_0008);
        assert_eq!(read_word(&mut s, 0xE000_EDD8), 0);
        assert_eq!(read_word(&mut s, 0xE000_EDDC), 0);
        assert_eq!(read_word(&mut s, 0xE000_EDE0), 0);
        // Select region 2 and store its base/limit/attributes.
        write_word(&mut s, 0xE000_EDD8, 2);
        write_word(&mut s, 0xE000_EDDC, 0x8000_0000);
        write_word(&mut s, 0xE000_EDE0, 0x8FFF_FFFD); // NS, ENABLE, no NSC
        assert_eq!(read_word(&mut s, 0xE000_EDD8), 2);
        assert_eq!(read_word(&mut s, 0xE000_EDDC), 0x8000_0000);
        assert_eq!(read_word(&mut s, 0xE000_EDE0), 0x8FFF_FFFD);
        // The other regions are untouched.
        write_word(&mut s, 0xE000_EDD8, 0);
        assert_eq!(read_word(&mut s, 0xE000_EDDC), 0);
        // `RBAR[7:0]` are reserved: a write of a non-aligned base is masked.
        write_word(&mut s, 0xE000_EDD8, 3);
        write_word(&mut s, 0xE000_EDDC, 0x2000_00FF);
        assert_eq!(read_word(&mut s, 0xE000_EDDC), 0x2000_0000);
        // Only `ENABLE`/`ALLNS` of `SAU_CTRL` are implemented.
        write_word(&mut s, 0xE000_EDD0, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut s, 0xE000_EDD0), 0x0000_0003);
        // Reset clears the whole unit (TYPE stays read-only).
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_EDD0), 0);
        assert_eq!(read_word(&mut s, 0xE000_EDDC), 0);
    }

    #[test]
    fn sau_hit_matches_enabled_regions() {
        let mut s = Scb::new();
        // Disabled: never matches.
        assert_eq!(s.sau_hit(0x8000_0000), SauHit::None);
        // Region 0: NS region covering 0x0000_0000..0x0000_001F.
        write_word(&mut s, 0xE000_EDD8, 0);
        write_word(&mut s, 0xE000_EDDC, 0x0000_0000);
        write_word(&mut s, 0xE000_EDE0, 0x0000_0001); // NS, ENABLE
                                                      // Region 1: NSC region covering 0x1000_0000..0x1000_001F.
        write_word(&mut s, 0xE000_EDD8, 1);
        write_word(&mut s, 0xE000_EDDC, 0x1000_0000);
        write_word(&mut s, 0xE000_EDE0, 0x1000_0003); // NSC, ENABLE
                                                      // Region 2: NS region covering 0x8000_0000..0x8000_001F.
        write_word(&mut s, 0xE000_EDD8, 2);
        write_word(&mut s, 0xE000_EDDC, 0x8000_0000);
        write_word(&mut s, 0xE000_EDE0, 0x8000_0001); // NS, ENABLE
                                                      // Region 3: a disabled region (ignored).
        write_word(&mut s, 0xE000_EDD8, 3);
        write_word(&mut s, 0xE000_EDDC, 0x0000_0000);
        write_word(&mut s, 0xE000_EDE0, 0xFFFF_FF00); // ENABLE clear

        // Still `SAU_CTRL.ENABLE == 0`: no matches.
        assert_eq!(s.sau_hit(0x8000_0000), SauHit::None);
        write_word(&mut s, 0xE000_EDD0, 0x1); // ENABLE

        assert_eq!(
            s.sau_hit(0x8000_0000),
            SauHit::Region {
                index: 2,
                nsc: false
            }
        );
        assert_eq!(
            s.sau_hit(0x8000_001F),
            SauHit::Region {
                index: 2,
                nsc: false
            }
        );
        assert_eq!(s.sau_hit(0x8000_0020), SauHit::None, "just past the limit");
        assert_eq!(
            s.sau_hit(0x1000_0000),
            SauHit::Region {
                index: 1,
                nsc: true
            }
        );
        assert_eq!(
            s.sau_hit(0x0000_0000),
            SauHit::Region {
                index: 0,
                nsc: false
            }
        );
        assert_eq!(s.sau_hit(0x2000_0000), SauHit::None, "outside every region");
        // A second enabled region overlapping region 0 collapses to Multiple.
        write_word(&mut s, 0xE000_EDD8, 4);
        write_word(&mut s, 0xE000_EDDC, 0x0000_0000);
        write_word(&mut s, 0xE000_EDE0, 0x0000_0001);
        assert_eq!(s.sau_hit(0x0000_0000), SauHit::Multiple);
    }

    #[test]
    fn mpu_v8_type_reports_eight_regions_and_registers_store_raw() {
        let mut s = Scb::new();
        s.set_pmsav8();
        // TYPE is read-only and reports eight unified regions.
        write_word(&mut s, 0xE000_ED90, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut s, 0xE000_ED90), 0x0000_0800);
        // CTRL stores only the three implemented bits.
        write_word(&mut s, 0xE000_ED94, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut s, 0xE000_ED94), 0x7);
        // RBAR/RLAR store the raw word (the 32-byte alignment is applied only
        // at lookup time), unlike the v7-M window's `REGION`/`VALID` field.
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000 | 0x1F | 0x7);
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0x2000_0000 | 0x1F | 0x7);
        write_word(&mut s, 0xE000_EDA0, 0x2000_001F | 0x1);
        assert_eq!(read_word(&mut s, 0xE000_EDA0), 0x2000_001F | 0x1);
        // RNR stores the region number.
        write_word(&mut s, 0xE000_ED98, 0x3);
        assert_eq!(read_word(&mut s, 0xE000_ED98), 0x3);
        // Reset clears the unit (TYPE stays read-only) and returns to the
        // secure bank.
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED94), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0);
        assert_eq!(read_word(&mut s, 0xE000_EDA0), 0);
    }

    #[test]
    fn mpu_v8_rnr_ignores_out_of_range_regions() {
        let mut s = Scb::new();
        s.set_pmsav8();
        // A full-word value of 8 or more is ignored, like QEMU.
        write_word(&mut s, 0xE000_ED98, 8);
        assert_eq!(read_word(&mut s, 0xE000_ED98), 0);
        write_word(&mut s, 0xE000_ED98, 7);
        assert_eq!(read_word(&mut s, 0xE000_ED98), 7);
        write_word(&mut s, 0xE000_ED98, 8);
        assert_eq!(read_word(&mut s, 0xE000_ED98), 7, "stays at the last valid");
    }

    #[test]
    fn mpu_v8_aliases_override_the_rnr_low_bits() {
        let mut s = Scb::new();
        s.set_pmsav8();
        // RNR = 0: the base registers index region 0, the aliases index
        // 1/2/3.
        write_word(&mut s, 0xE000_ED98, 0);
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000); // base -> region 0
        write_word(&mut s, 0xE000_EDA4, 0x2000_0020); // A1  -> region 1
        write_word(&mut s, 0xE000_EDAC, 0x2000_0040); // A2  -> region 2
        write_word(&mut s, 0xE000_EDB4, 0x2000_0060); // A3  -> region 3
        write_word(&mut s, 0xE000_EDA0, 0x2000_001F | 1); // RLAR base -> region 0
        write_word(&mut s, 0xE000_EDA8, 0x2000_003F | 1); // RLAR A1  -> region 1
        write_word(&mut s, 0xE000_EDB0, 0x2000_005F | 1); // RLAR A2  -> region 2
        write_word(&mut s, 0xE000_EDB8, 0x2000_007F | 1); // RLAR A3  -> region 3
                                                          // Read each back through its alias.
        write_word(&mut s, 0xE000_ED98, 0);
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0x2000_0000);
        assert_eq!(read_word(&mut s, 0xE000_EDA0), 0x2000_001F | 1);
        write_word(&mut s, 0xE000_ED98, 1);
        assert_eq!(read_word(&mut s, 0xE000_EDA4), 0x2000_0020);
        assert_eq!(read_word(&mut s, 0xE000_EDA8), 0x2000_003F | 1);
        write_word(&mut s, 0xE000_ED98, 3);
        assert_eq!(read_word(&mut s, 0xE000_EDB4), 0x2000_0060);
        assert_eq!(read_word(&mut s, 0xE000_EDB8), 0x2000_007F | 1);
        // The base registers use the full RNR region number (not masked), so
        // RNR = 3 reaches region 3 through the base registers too.
        write_word(&mut s, 0xE000_ED98, 3);
        write_word(&mut s, 0xE000_ED9C, 0x3000_0000); // base -> region 3
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0x3000_0000);
        // With RNR = 2 the aliases hit the low two bits of the group: A1
        // indexes region 1, leaving region 3 untouched.
        write_word(&mut s, 0xE000_ED98, 2);
        write_word(&mut s, 0xE000_EDA4, 0x4000_0000); // A1 -> region 1
        assert_eq!(
            read_word(&mut s, 0xE000_EDB4),
            0x3000_0000,
            "region 3 stays"
        );
        write_word(&mut s, 0xE000_ED98, 1);
        assert_eq!(read_word(&mut s, 0xE000_EDA4), 0x4000_0000);
    }

    #[test]
    fn mpu_v8_registers_are_banked_per_security_state() {
        let mut s = Scb::new();
        s.set_pmsav8();
        // Boot secure: CTRL and RBAR land in the secure bank.
        write_word(&mut s, 0xE000_ED94, 0x1);
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000);
        assert_eq!(read_word(&mut s, 0xE000_ED94), 0x1);
        // Non-secure: a fresh bank reads back zeros.
        s.set_secure(false);
        assert_eq!(read_word(&mut s, 0xE000_ED94), 0);
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0);
        write_word(&mut s, 0xE000_ED94, 0x5); // PRIVDEFENA
        write_word(&mut s, 0xE000_ED9C, 0x8000_0000);
        assert_eq!(read_word(&mut s, 0xE000_ED94), 0x5);
        // Back in secure: the secure values are untouched.
        s.set_secure(true);
        assert_eq!(read_word(&mut s, 0xE000_ED94), 0x1);
        assert_eq!(read_word(&mut s, 0xE000_ED9C), 0x2000_0000);
    }

    #[test]
    fn mpu_v8_disabled_uses_the_default_map() {
        let mut s = Scb::new();
        s.set_pmsav8();
        // Disabled: every lookup reports the background map, no region.
        assert_eq!(
            s.mpu_lookup_v8(0x2000_0000, true, true),
            MpuLookup {
                prot: crate::mem::bus::PROT_READ
                    | crate::mem::bus::PROT_WRITE
                    | crate::mem::bus::PROT_EXEC,
                region: None,
            }
        );
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Read, true, 0));
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Write, true, 0));
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Fetch, true, 0));
        // The peripheral range is read/write only.
        assert_eq!(
            s.mpu_lookup_v8(0x4000_0000, true, false).prot,
            crate::mem::bus::PROT_READ | crate::mem::bus::PROT_WRITE,
        );
        assert!(!s.mpu_check(0x4000_0000, MpuAccess::Fetch, true, 0));
    }

    #[test]
    fn mpu_v8_region_ap_grants_and_denies() {
        let mut s = Scb::new();
        s.set_pmsav8();
        write_word(&mut s, 0xE000_ED94, 0x1); // ENABLE
                                              // Region 0: 0x2000_0000..0x2000_001F, AP=1 (read/write), no XN.
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000 | 0x2);
        write_word(&mut s, 0xE000_EDA0, 0x2000_001F | 1);
        let lk = s.mpu_lookup_v8(0x2000_0000, true, true);
        assert_eq!(lk.region, Some(0));
        assert_eq!(lk.prot, 0x7);
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Read, true, 0));
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Write, true, 0));
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Fetch, true, 0));
        assert_eq!(
            s.mpu_lookup_v8(0x2000_0020, true, true).prot,
            0,
            "past the limit: no region, no background"
        );
        assert!(!s.mpu_check(0x2000_0020, MpuAccess::Read, true, 0));
        // AP=0: read/write for privileged, nothing for unprivileged.
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000);
        assert_eq!(s.mpu_lookup_v8(0x2000_0000, true, true).prot, 0x7);
        assert_eq!(s.mpu_lookup_v8(0x2000_0000, true, false).prot, 0x0);
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Read, true, 0));
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Write, true, 0));
        // AP=2: read-only for privileged, nothing for unprivileged.
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000 | (0x2 << 1));
        assert_eq!(s.mpu_lookup_v8(0x2000_0000, true, true).prot, 0x5);
        assert_eq!(s.mpu_lookup_v8(0x2000_0000, true, false).prot, 0x0);
        // AP=3: read-only for everyone.
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000 | (0x3 << 1));
        assert_eq!(s.mpu_lookup_v8(0x2000_0000, true, false).prot, 0x5);
        assert!(!s.mpu_check(0x2000_0000, MpuAccess::Write, true, 0));
    }

    #[test]
    fn mpu_v8_xn_and_system_space_remove_execute() {
        let mut s = Scb::new();
        s.set_pmsav8();
        write_word(&mut s, 0xE000_ED94, 0x1); // ENABLE
                                              // Region 1: 0x2000_0000..0x2000_001F, AP=1, XN set.
        write_word(&mut s, 0xE000_EDA4, 0x2000_0000 | 0x2 | 0x1);
        write_word(&mut s, 0xE000_EDA8, 0x2000_001F | 1);
        let lk = s.mpu_lookup_v8(0x2000_0000, true, true);
        assert_eq!(lk.region, Some(1));
        assert_eq!(lk.prot, 0x3, "XN clears only the execute bit");
        assert!(!s.mpu_check(0x2000_0000, MpuAccess::Fetch, true, 0));
        assert!(s.mpu_check(0x2000_0000, MpuAccess::Read, true, 0));
        // A region in the system space is always execute-never, even with XN
        // clear: 0xE010_0000 (outside the PPB bypass window).
        write_word(&mut s, 0xE000_ED9C, 0xE010_0000 | 0x2);
        write_word(&mut s, 0xE000_EDA0, 0xE010_001F | 1);
        assert_eq!(
            s.mpu_lookup_v8(0xE010_0000, true, true).prot,
            crate::mem::bus::PROT_READ | crate::mem::bus::PROT_WRITE,
        );
        assert!(!s.mpu_check(0xE010_0000, MpuAccess::Fetch, true, 0));
    }

    #[test]
    fn mpu_v8_ppb_bypasses_and_multiple_hits_fail() {
        let mut s = Scb::new();
        s.set_pmsav8();
        write_word(&mut s, 0xE000_ED94, 0x1); // ENABLE
                                              // The PPB (0xE000_0000..0xE00F_FFFF) bypasses the MPU even when a
                                              // region covers it: the lookup reports the background map.
        write_word(&mut s, 0xE000_ED9C, 0xE000_0000 | 0x2);
        write_word(&mut s, 0xE000_EDA0, 0xE00F_FFFF | 1);
        assert_eq!(
            s.mpu_lookup_v8(0xE000_ED90, true, true),
            MpuLookup {
                prot: crate::mem::bus::PROT_READ | crate::mem::bus::PROT_WRITE,
                region: None,
            }
        );
        assert!(s.mpu_check(0xE000_ED90, MpuAccess::Read, true, 0));
        // Two enabled regions covering the same address fail the lookup.
        write_word(&mut s, 0xE000_ED9C, 0x2000_0000 | 0x2); // region 0
        write_word(&mut s, 0xE000_EDA0, 0x2000_001F | 1);
        write_word(&mut s, 0xE000_EDB4, 0x2000_0000 | 0x2); // region 3 (RNR & !3 = 0)
        write_word(&mut s, 0xE000_EDB8, 0x2000_001F | 1);
        assert_eq!(
            s.mpu_lookup_v8(0x2000_0000, true, true),
            MpuLookup {
                prot: 0,
                region: None
            },
        );
        assert!(!s.mpu_check(0x2000_0000, MpuAccess::Read, true, 0));
    }

    #[test]
    fn mpu_v8_privefena_background_and_bypass() {
        let mut s = Scb::new();
        s.set_pmsav8();
        write_word(&mut s, 0xE000_ED94, 0x5); // ENABLE | PRIVDEFENA
                                              // No region covers 0x3000_0000: privileged gets the background map,
                                              // unprivileged gets nothing.
        assert_eq!(
            s.mpu_lookup_v8(0x3000_0000, true, true),
            MpuLookup {
                prot: crate::mem::bus::PROT_READ
                    | crate::mem::bus::PROT_WRITE
                    | crate::mem::bus::PROT_EXEC,
                region: None,
            }
        );
        assert_eq!(s.mpu_lookup_v8(0x3000_0000, true, false).prot, 0);
        assert!(s.mpu_check(0x3000_0000, MpuAccess::Fetch, true, 0));
        assert!(!s.mpu_check(0x3000_0000, MpuAccess::Fetch, false, 0));
        // No PRIVDEFENA and no match: even privileged fails.
        write_word(&mut s, 0xE000_ED94, 0x1);
        assert_eq!(s.mpu_lookup_v8(0x3000_0000, true, true).prot, 0);
        assert!(!s.mpu_check(0x3000_0000, MpuAccess::Read, true, 0));
        // Without HFNMIENA the NMI (2) and HardFault (3) handlers bypass the
        // MPU; with it set, they do not.
        assert!(s.mpu_check(0x3000_0000, MpuAccess::Read, true, 2));
        assert!(s.mpu_check(0x3000_0000, MpuAccess::Read, true, 3));
        assert!(!s.mpu_check(0x3000_0000, MpuAccess::Read, true, 0));
        write_word(&mut s, 0xE000_ED94, 0x3); // ENABLE | HFNMIENA
        assert!(!s.mpu_check(0x3000_0000, MpuAccess::Read, true, 2));
    }
}
