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
//! `MPU_RBAR`/`MPU_RASR` registers (plus their v7M `_A1`/`_A2`/`_A3` aliases,
//! which index `MPU_RNR` like the base registers) with eight unified regions,
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
//! `FORCED` escalation bit through [`PeripheralBus::record_fault`].
//!
//! The `FPCCR` (`0xE000_EF34`) and `FPCAR` (`0xE000_EF38`) windows expose the
//! M-profile lazy-FP-stacking control to firmware. Their state lives in the
//! shared [`FpuCtx`], which the core also holds, so a firmware write is
//! immediately visible to the exception-entry/FP logic and vice versa.
//!
//! Priority encoding follows the architecture: lower value == higher
//! priority, with the `SVCall` byte in `SHPR2` and the `PendSV`/`SysTick`
//! bytes in `SHPR3` (CMSIS `SCB->SHP[7]`, `SHP[10]`, `SHP[11]`).

use crate::cpu::FpuCtx;
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::{MpuAccess, Region};
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
const CPACR: u32 = 0xE000_ED88;
const FPCCR: u32 = 0xE000_EF34;
const FPCAR: u32 = 0xE000_EF38;
const FPDSCR: u32 = 0xE000_EF3C;

/// The ARMv8-M Security Attribution Unit window (`SAU_CTRL`/`SAU_TYPE`/
/// `SAU_RNR`/`SAU_RBAR`/`SAU_RLAR`). The register values are stored for
/// firmware compatibility — the M33 conformance tests program the SAU to make
/// the IDAU Non-secure regions effective — but the security lookup itself is
/// not yet SAU-aware (it stays in the SAU-disabled, IDAU-only case).
const SAU: u32 = 0xE000_EDD0;

/// The MPU register window. One 4-byte-aligned window covers `MPU_TYPE` ..=
/// `MPU_RASR_A3`, dispatched by byte offset; the aliases `_A1`/`_A2`/`_A3`
/// behave exactly like the base registers on ARMv7-M (they are indexed by
/// `MPU_RNR`, unlike the v8M PMSAv8 alias-region override), matching the QEMU
/// oracle.
const MPU: u32 = 0xE000_ED90;

// MMFSR fault-status bits (CFSR bits 7:0). `IACCVIOL`/`DACCVIOL` are the
// instruction/data access violations the MPU raises; `MMARVALID` marks
// `MMFAR` as holding the faulting address.
const MMFSR_IACCVIOL: u32 = 0x0000_0001;
const MMFSR_DACCVIOL: u32 = 0x0000_0002;
const MMFSR_MMARVALID: u32 = 0x0000_0080;

const PENDSVSET: u8 = 0x10; // ICSR byte 3, bit 28: write 1 to pend.
const PENDSVCLR: u8 = 0x08; // ICSR byte 3, bit 27: write 1 to clear.

// AIRCR: bits 31:16 carry the write `VECTKEY` (0x05FA) and read back as
// `VECTKEYSTAT` (0xFA05); bit 15 `ENDIANNESS` is read-only zero.
const AIRCR_VECTKEY: u32 = 0x05FA_0000;
const AIRCR_PRIGROUP: u32 = 0x0000_0700;
const AIRCR_SYSRESETREQ: u32 = 0x0000_0004;

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
// the SCB is not told which exception is running, so they are only accurate in
// Thread mode.
const SHCSR_WRITABLE: u32 = 0x0007_F000;
const SHCSR_SVCALLPENDED: u32 = 0x0000_8000;
const SHCSR_USGFAULTPENDED: u32 = 0x0000_1000;
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
/// wrote them. The base is kept in `RBAR` bits [31:5] (firmware's `REGION`/
/// `VALID` bits are consumed on the word write); the region size, subregion
/// disable, and access permissions decode from `RASR`.
#[derive(Debug, Clone, Copy)]
struct MpuRegion {
    rbar: u32,
    rasr: u32,
}

impl Default for MpuRegion {
    fn default() -> Self {
        Self { rbar: 0, rasr: 0 }
    }
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
            0 | 4 | 5 | 6 | _ => false,
        }
    }
}

/// The MPU configuration: control bits, the region-number register, and the
/// eight unified regions.
#[derive(Debug, Clone)]
struct Mpu {
    /// `MPU_CTRL`: `ENABLE` (0), `HFNMIENA` (1), `PRIVDEFENA` (2).
    ctrl: u32,
    /// `MPU_RNR.REGION`: the region indexed by the base `RBAR`/`RASR` writes,
    /// masked to the three bits the eight regions need.
    rnr: u32,
    regions: [MpuRegion; 8],
}

impl Default for Mpu {
    fn default() -> Self {
        Self {
            ctrl: 0,
            rnr: 0,
            regions: [MpuRegion::default(); 8],
        }
    }
}

/// The System Control Block: interrupt control state, exception priorities,
/// the reset/control registers (`AIRCR`/`SCR`/`CCR`), the coprocessor
/// access-control register, and the FP context control.
#[derive(Debug, Clone)]
pub struct Scb {
    ranges: [Region; 18],
    pendsv: bool,
    /// `SHPR1`: configurable priorities for the fault exceptions. Only the
    /// `UsageFault` byte (bits 15:8) is modeled; MemManage/BusFault are not
    /// distinct exceptions in this simulator.
    shpr1: u32,
    shpr2: u32,
    shpr3: u32,
    cpacr: u32,
    /// `VTOR`: the vector table base, masked to the 128-byte alignment.
    vtor: u32,
    /// `SHCSR`: fault enables and pend bits; the `*ACT` bits read zero.
    shcsr: u32,
    /// `CFSR`: the combined fault status (MMFSR/BFSR/UFSR), write-1-to-clear.
    cfsr: u32,
    /// `HFSR`: `FORCED`/`VECTTBL`, write-1-to-clear.
    hfsr: u32,
    /// `MMFAR`: the faulting address latched by an MPU access violation. The
    /// core writes it through [`Scb::record_mpu_fault`]; firmware reads it.
    mmfar: u32,
    /// The MPU configuration and region registers.
    mpu: Mpu,
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
    /// Latched by `AIRCR.SYSRESETREQ`; the machine performs the reset.
    reset_requested: bool,
    /// `SAU_CTRL` (`ENABLE`/`ALLNS`). Stored for firmware compatibility; not
    /// yet applied to the security attribution.
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
                Region::new(CPACR, 4),
                Region::new(MPU, 0x2C),
                Region::new(SAU, 0x10),
                Region::new(FPCCR, 4),
                Region::new(FPCAR, 4),
                Region::new(FPDSCR, 4),
            ],
            pendsv: false,
            shpr1: 0,
            shpr2: 0,
            shpr3: 0,
            cpacr: 0,
            vtor: 0,
            shcsr: 0,
            cfsr: 0,
            hfsr: 0,
            mmfar: 0,
            mpu: Mpu::default(),
            rbar_pending: 0,
            aircr: 0,
            scr: 0,
            ccr: CCR_RESET,
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

    /// Whether the `PendSV` line is latched pending.
    #[must_use]
    pub const fn pendsv_pending(&self) -> bool {
        self.pendsv
    }

    /// The configurable priority byte for system exception `n` (`6`, `11`,
    /// `14`, `15`), lower meaning higher priority. Unrelated numbers read
    /// zero.
    #[must_use]
    pub const fn priority(&self, n: u8) -> u8 {
        match n {
            6 => ((self.shpr1 >> 8) & 0xFF) as u8,
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

    /// `VTOR`: the vector table base (bits [31:7], 128-byte aligned).
    #[must_use]
    pub const fn vtor(&self) -> u32 {
        self.vtor
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

    /// Whether the MPU permits the core's `access` to `addr` in the given
    /// privilege and exception context. Disabled by reset, so plain execution
    /// is unaffected until firmware enables it.
    #[must_use]
    pub fn mpu_check(&self, addr: u32, access: MpuAccess, privileged: bool, exception: u8) -> bool {
        // `MPU_CTRL.ENABLE` clear: the MPU is off, every access is allowed.
        if self.mpu.ctrl & 0x1 == 0 {
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

    /// Read the MPU register word at byte offset `reg_off` (0, 4, 8, ..) from
    /// the `0xE000_ED90` window base.
    const fn mpu_read(&self, reg_off: u32) -> u32 {
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

    const fn sau_region_index(&self) -> usize {
        (self.sau_rnr & 0x7) as usize
    }

    /// Byte-write to the SAU window at `offset` (from `SAU_CTRL`). The
    /// register values are stored for firmware compatibility; the security
    /// lookup does not yet apply them.
    fn sau_write(&mut self, offset: u32, val: u8) {
        let shift = 8 * (offset & 0x3);
        match offset & !0x3 {
            // Only `ENABLE` and `ALLNS` are implemented.
            0 => {
                self.sau_ctrl = (self.sau_ctrl & !(0xFF << shift)) | (u32::from(val) << shift);
                self.sau_ctrl &= 0x3;
            }
            // The write side of the shared address is `SAU_RNR`; the region
            // index is masked to the eight regions.
            4 => {
                self.sau_rnr = (self.sau_rnr & !(0xFF << shift)) | (u32::from(val) << shift);
                self.sau_rnr &= 0x7;
            }
            8 => {
                let idx = self.sau_region_index();
                self.sau_rbar[idx] =
                    (self.sau_rbar[idx] & !(0xFF << shift)) | (u32::from(val) << shift);
                // `RBAR[7:0]` are reserved (bases are 256-byte aligned).
                self.sau_rbar[idx] &= !0xFF;
            }
            12 => {
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
                // echoed back.
                AIRCR => 0xFA05_0000 | (u32::from(self.prig()) << 8),
                SCR => self.scr,
                CCR => self.ccr,
                SHPR1 => self.shpr1,
                SHPR2 => self.shpr2,
                SHPR3 => self.shpr3,
                SHCSR => self.shcsr,
                CFSR => self.cfsr,
                HFSR => self.hfsr,
                MMFAR => self.mmfar,
                CPACR => self.cpacr,
                SAU => match offset & !0x3 {
                    // `SAU_CTRL` reads back the stored `ENABLE`/`ALLNS` bits.
                    0 => self.sau_ctrl,
                    // `SAU_TYPE` (the read side of the `SAU_RNR` address):
                    // eight regions on the Cortex-M33.
                    4 => 0x0000_0008,
                    8 => self.sau_rbar[self.sau_region_index()],
                    12 => self.sau_rlar[self.sau_region_index()],
                    _ => 0,
                },
                FPCCR => self.fpu_ctx.lock().unwrap().fpccr(),
                FPCAR => self.fpu_ctx.lock().unwrap().fpcar,
                FPDSCR => self.fpu_ctx.lock().unwrap().fpdscr,
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
                self.shpr1 = (self.shpr1 & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            SHPR2 => {
                let shift = 8 * offset;
                self.shpr2 = (self.shpr2 & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            SHPR3 => {
                let shift = 8 * offset;
                self.shpr3 = (self.shpr3 & !(0xFF << shift)) | (u32::from(val) << shift);
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
            SAU => self.sau_write(offset, val),
            FPCCR => {
                // Only the ASPEN (bit 7 of byte 3) and LSPEN (bit 6 of byte 3)
                // are software-writable; the ready bits and LSPACT are
                // read-only.
                if offset == 3 {
                    let mut ctx = self.fpu_ctx.lock().unwrap();
                    ctx.write_fpccr(u32::from(val) << 24);
                }
            }
            FPCAR => {
                let shift = 8 * offset;
                let mut ctx = self.fpu_ctx.lock().unwrap();
                ctx.fpcar = (ctx.fpcar & !(0xFF << shift)) | (u32::from(val) << shift) & !0x7;
            }
            FPDSCR => {
                let shift = 8 * offset;
                let mut ctx = self.fpu_ctx.lock().unwrap();
                ctx.fpdscr = (ctx.fpdscr & !(0xFF << shift)) | (u32::from(val) << shift);
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
        self.shcsr = 0;
        self.cfsr = 0;
        self.hfsr = 0;
        self.mmfar = 0;
        self.mpu = Mpu::default();
        self.rbar_pending = 0;
        self.aircr = 0;
        self.scr = 0;
        self.ccr = CCR_RESET;
        self.reset_requested = false;
        self.sau_ctrl = 0;
        self.sau_rnr = 0;
        self.sau_rbar = [0; 8];
        self.sau_rlar = [0; 8];
        *self.fpu_ctx.lock().unwrap() = FpuCtx::new();
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
    use crate::mem::bus::MpuAccess;
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
        // SHPR1 byte 1 -> UsageFault (bits 15:8).
        s.write(0xE000_ED19, 0x60).unwrap();
        assert_eq!(s.priority(6), 0x60);
        assert_eq!(read_word(&mut s, 0xE000_ED18), 0x0000_6000);
        // The MemManage/BusFault bytes (SHPR1 bytes 3 and 2) are unmodeled
        // and expose no priority.
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED18), 0);
        assert_eq!(s.priority(6), 0);
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
    fn fpu_ctx_is_shared_with_the_core() {
        let fpu_ctx = Arc::new(Mutex::new(FpuCtx::new()));
        let mut s = Scb::new();
        s.set_fpu_ctx(fpu_ctx.clone());
        // Firmware clears LSPEN through the FPCCR window; the core sees it.
        write_word(&mut s, 0xE000_EF34, 0x8780_0004);
        assert!(!fpu_ctx.lock().unwrap().lspen);
        assert!(fpu_ctx.lock().unwrap().aspen);
        // Hardware sets LSPACT; the FPCCR read reflects it.
        fpu_ctx.lock().unwrap().lspact = true;
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x8780_0005);
        // And the FPCCR write survives reset only until `reset()`.
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0xC780_0004);
        assert!(fpu_ctx.lock().unwrap().lspen);
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
        // Reset: CTRL clear, TYPE reads eight regions, RBAR/RLAR read zero.
        assert_eq!(read_word(&mut s, 0xE000_EDD0), 0);
        assert_eq!(read_word(&mut s, 0xE000_EDD4), 0x0000_0008);
        assert_eq!(read_word(&mut s, 0xE000_EDD8), 0);
        assert_eq!(read_word(&mut s, 0xE000_EDDC), 0);
        // Select region 2 and store its base/limit/attributes.
        write_word(&mut s, 0xE000_EDD4, 2);
        write_word(&mut s, 0xE000_EDD8, 0x8000_0000);
        write_word(&mut s, 0xE000_EDDC, 0x8FFF_FFFD); // NS, ENABLE, no NSC
        assert_eq!(read_word(&mut s, 0xE000_EDD8), 0x8000_0000);
        assert_eq!(read_word(&mut s, 0xE000_EDDC), 0x8FFF_FFFD);
        // The other regions are untouched.
        write_word(&mut s, 0xE000_EDD4, 0);
        assert_eq!(read_word(&mut s, 0xE000_EDD8), 0);
        // `RBAR[7:0]` are reserved: a write of a non-aligned base is masked.
        write_word(&mut s, 0xE000_EDD4, 3);
        write_word(&mut s, 0xE000_EDD8, 0x2000_00FF);
        assert_eq!(read_word(&mut s, 0xE000_EDD8), 0x2000_0000);
        // Only `ENABLE`/`ALLNS` of `SAU_CTRL` are implemented.
        write_word(&mut s, 0xE000_EDD0, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut s, 0xE000_EDD0), 0x0000_0003);
        // Reset clears the whole unit (TYPE stays read-only).
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_EDD0), 0);
        assert_eq!(read_word(&mut s, 0xE000_EDD8), 0);
    }
}
