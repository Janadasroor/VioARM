//! The ARMv6-M CPU core: fetch, decode, execute, and the (typed) halt states.
//!
//! The executor is the *only* place that interprets semantics. It is written
//! against `Bus`, so swapping the memory behind the CPU does not change
//! instruction behavior.

pub mod decode;
pub mod decode16;
pub mod decode32;
pub mod fpu;
pub mod registers;

use crate::error::{Exception, HaltCause};
use crate::mem::{MpuAccess, PeripheralBus, SauHit};
use decode16::{
    decode16, decode16_v7m, AluOp, ExtendKind, Hint, Inst16, LsImm, LsReg, ReverseKind, Shift,
    ShiftImm,
};
use decode32::{
    decode32, decode32_v7m, decode32_v8m, is_thumb2, is_thumb2_v7m, thumb_expand_imm, BitfieldOp,
    DpOp, DspMulOp,
    DualMulOp, ExSize, ExtendAddOp, Inst32, MemOp, MulOp, ParOp, ParSat, QaddOp, SatOp, ShiftType,
    SysReg, TtVariant, UnaryOp, Vfp2Op, Vfp3Op, VrintOp,
};
use registers::{Registers, Xpsr, APSR_FLAGS, APSR_GE, APSR_Q, APSR_QFLAGS, XPSR_T};

/// Which Thumb architecture the core executes.
///
/// ARMv6-M (Cortex-M0/M0+) is a 16-bit ISA plus `bl`/`mrs`/`msr`/barriers;
/// ARMv7-M (Cortex-M3/M4) adds the full Thumb-2 set, IT blocks, and a
/// programmable exception-priority model. ARMv8-M baseline (Cortex-M23) is a
/// strict superset of ARMv6-M and a strict subset of ARMv7-M (and of the
/// ARMv8-M mainline, whose Cortex-M33 is the conformance oracle): it adds
/// IT blocks, the `b.w` T4 branch, `sdiv`/`udiv`, `movw`/`movt`, the
/// exclusive `ldrex`/`strex` family and `clrex`, unprivileged state
/// (`CONTROL.nPRIV`), and the TrustZone security extension â€” but no BASEPRI,
/// FAULTMASK, or UsageFault, and none of the ARMv7-M Thumb-2 data-processing,
/// load/store, bitfield, or DSP encodings. The decoder and executor select
/// behavior on this field so a single core type serves all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Isa {
    Armv6M,
    Armv7M,
    Armv8MBaseline,
}

/// `LR` value loaded on exception entry, causing `bx lr`/`pop {pc}` to return
/// to Handler mode (re-entering the handler that was interrupted).
const EXC_RETURN_HANDLER: u32 = 0xFFFF_FFF1;
/// Return to Thread mode, restoring the main stack pointer.
const EXC_RETURN_THREAD_MSP: u32 = 0xFFFF_FFF9;
/// Return to Thread mode, restoring the process stack pointer.
const EXC_RETURN_THREAD_PSP: u32 = 0xFFFF_FFFD;
/// The same returns with the extended (FP) frame: `EXC_RETURN.FType == 0`.
const EXC_RETURN_HANDLER_FP: u32 = 0xFFFF_FFE1;
const EXC_RETURN_THREAD_MSP_FP: u32 = 0xFFFF_FFE9;
const EXC_RETURN_THREAD_PSP_FP: u32 = 0xFFFF_FFED;

/// Lowest possible ARMv8-M `EXC_RETURN` value: a value at or above
/// `0xFF000000` with the fixed `1` pattern is an `EXC_RETURN`, below it (but
/// at or above `FNC_RETURN_MIN_MAGIC`) a `FNC_RETURN` (QEMU
/// `EXC_RETURN_MIN_MAGIC`/`FNC_RETURN_MIN_MAGIC`).
const EXC_RETURN_MIN_MAGIC: u32 = 0xFF00_0000;
/// Lowest possible ARMv8-M `FNC_RETURN` value (`0xFEFFFFFF` is the lowest
/// Non-secure call-return magic; QEMU uses `0xFEFFFFFE`).
const FNC_RETURN_MIN_MAGIC: u32 = 0xFEFF_FFFE;
/// The `FNC_RETURN` value `BLXNS` loads into `LR` (and `BXNS` returns on) to
/// mark a Non-secure function call.
const FNC_RETURN: u32 = 0xFEFF_FFFF;

/// `EXC_RETURN` field masks (ARMv8-M; also used to build the value on
/// exception entry). `RES1` is bits[31:7], `S` the target-security bit,
/// `DCRS` the callee-saves stacking bit, `FTYPE` the FP-frame bit, `MODE` the
/// thread/handler bit, `SPSEL` the target stack bit, `RES0` a reserved bit
/// and `ES` the exception-security bit.
const EXC_RETURN_RES1: u32 = 0xFFFF_FF80;
const EXC_RETURN_S: u32 = 0x40;
const EXC_RETURN_DCRS: u32 = 0x20;
const EXC_RETURN_FTYPE: u32 = 0x10;
const EXC_RETURN_MODE: u32 = 0x08;
const EXC_RETURN_SPSEL: u32 = 0x04;
const EXC_RETURN_RES0: u32 = 0x02;
const EXC_RETURN_ES: u32 = 0x01;

/// The integrity signature written as the first word of the ARMv8-M
/// callee-saves stack block (`0xFEFA125A` with bit 0 set because the baseline
/// always stacks a basic frame, so `EXC_RETURN.FTYPE` is set; QEMU
/// `v7m_integrity_sig`). Its value is checked on the Secure return that pops
/// the block; a mismatch is `SFSR.INVIS`.
const CALLEE_SAVES_SIG: u32 = 0xFEFA_125B;

/// The ARMv8-M callee-saves stack block, in bytes: the integrity signature
/// word, a reserved word, and `r4`-`r11` (QEMU `v7m_push_callee_stack`).
const CALLEE_SAVES_BYTES: u32 = 0x28;

/// xPSR `ALIGN` bit (bit 9): set on the stacked xPSR when the interrupted
/// stack was 8-byte-aligned by an exception-entry adjustment.
const XPSR_ALIGN: u32 = 1 << 9;

/// The eight-word ARMv6-M exception frame, in bytes.
const FRAME_BYTES: u32 = 32;
/// The FP extension of the exception frame (16 single-precision registers +
/// FPSCR), in bytes, added when `CONTROL.FPCA` is set at exception entry.
const FP_FRAME_BYTES: u32 = 0x48;
/// The full extended frame, in bytes.
const EXTENDED_FRAME_BYTES: u32 = FRAME_BYTES + FP_FRAME_BYTES;

/// What an executed instruction does with the program counter.
enum Next {
    /// Continue at `address`.
    Advance(u32),
    /// The machine halted; propagate the cause.
    Stop(HaltCause),
}

/// FPSCR bit positions (architectural A32/A32VFP layout).
pub mod fpscr {
    /// Invalid-operation cumulative exception flag (bit 0).
    pub const IOC: u32 = 1 << 0;
    /// Divide-by-zero cumulative exception flag (bit 1).
    pub const DZC: u32 = 1 << 1;
    /// Overflow cumulative exception flag (bit 2).
    pub const OFC: u32 = 1 << 2;
    /// Underflow cumulative exception flag (bit 3).
    pub const UFC: u32 = 1 << 3;
    /// Inexact cumulative exception flag (bit 4).
    pub const IXC: u32 = 1 << 4;
    /// Input-denormal cumulative exception flag (bit 7).
    pub const IDC: u32 = 1 << 7;
    /// The six cumulative exception flags.
    pub const CEXC: u32 = IOC | DZC | OFC | UFC | IXC | IDC;
    /// Rounding-mode field, bits [23:22] (`0`=nearest-even, `1`=+inf,
    /// `2`=-inf, `3`=towards zero).
    pub const RMODE: u32 = 3 << 22;
    /// Flush-to-zero enable (bit 24): denormal results are flushed.
    pub const FZ: u32 = 1 << 24;
    /// Default-NaN enable (bit 25).
    pub const DN: u32 = 1 << 25;
    /// The condition flags, bits [31:28], written by VFP comparisons.
    pub const NZCV: u32 = 0xF << 28;
}

/// True if the raw float32 bits denote a NaN (exponent all ones, fraction
/// non-zero), matching QEMU's `float32_is_any_nan`.
#[inline]
#[must_use]
fn is_f32_nan(raw: u32) -> bool {
    raw & 0x7f80_0000 == 0x7f80_0000 && raw & 0x007f_ffff != 0
}

/// VFP (FPv4-SP) architectural state: the S0-S31 single-precision register
/// bank and FPSCR. D0-D15 alias the S pairs (`Dn` == `S{2n}:S{2n+1}`) for the
/// 64-bit move and load/store instructions.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VfpState {
    pub s: [u32; 32],
    pub fpscr: u32,
}

impl VfpState {
    /// Read single-precision register `Sd`.
    #[inline]
    #[must_use]
    pub fn read_s(&self, d: u8) -> u32 {
        self.s[d as usize]
    }

    /// Write single-precision register `Sd`.
    #[inline]
    pub fn write_s(&mut self, d: u8, v: u32) {
        self.s[d as usize] = v;
    }

    /// Read doubleword register `Dd` (the `S{2d}:S{2d+1}` pair).
    #[inline]
    #[must_use]
    pub fn read_d(&self, d: u8) -> u64 {
        let lo = u64::from(self.s[(2 * d) as usize]);
        let hi = u64::from(self.s[(2 * d + 1) as usize]);
        lo | (hi << 32)
    }

    /// Write doubleword register `Dd` (the `S{2d}:S{2d+1}` pair).
    #[inline]
    pub fn write_d(&mut self, d: u8, v: u64) {
        self.s[(2 * d) as usize] = v as u32;
        self.s[(2 * d + 1) as usize] = (v >> 32) as u32;
    }

    /// Rounding mode encoded in FPSCR bits [23:22], as a `RoundMode`.
    #[inline]
    #[must_use]
    pub fn rounding_mode(&self) -> fpu::RoundMode {
        match (self.fpscr >> 22) & 0b11 {
            0b01 => fpu::RoundMode::Up,
            0b10 => fpu::RoundMode::Down,
            0b11 => fpu::RoundMode::ToZero,
            _ => fpu::RoundMode::NearestEven,
        }
    }

    /// Build the softfloat status word from the control bits of FPSCR.
    #[inline]
    #[must_use]
    pub fn float_status(&self) -> fpu::FloatStatus {
        let mut status = fpu::FloatStatus::new(self.rounding_mode());
        status.flush_to_zero = self.fpscr & fpscr::FZ != 0;
        status.flush_inputs_to_zero = self.fpscr & fpscr::FZ != 0;
        status.default_nan_mode = self.fpscr & fpscr::DN != 0;
        status
    }

    /// Fold a softfloat exception word into FPSCR's cumulative exception
    /// flags. `flags` is a `FLAG_*` bitmask from [`fpu`].
    pub fn fold_flags(&mut self, flags: u32) {
        let mut bits = 0;
        if flags & fpu::FLAG_INVALID != 0 {
            bits |= fpscr::IOC;
        }
        if flags & fpu::FLAG_DIVBYZERO != 0 {
            bits |= fpscr::DZC;
        }
        if flags & fpu::FLAG_OVERFLOW != 0 {
            bits |= fpscr::OFC;
        }
        if flags & (fpu::FLAG_UNDERFLOW | fpu::FLAG_OUTPUT_DENORMAL_FLUSHED) != 0 {
            bits |= fpscr::UFC;
        }
        if flags & fpu::FLAG_INEXACT != 0 {
            bits |= fpscr::IXC;
        }
        if flags & fpu::FLAG_INPUT_DENORMAL_FLUSHED != 0 {
            bits |= fpscr::IDC;
        }
        self.fpscr |= bits;
    }
}

/// The M-profile floating-point context-control state: the `ASPEN`, `LSPEN`
/// and `LSPACT` bits of `FPCCR`, `FPCAR`, and the default FP control register
/// `FPDSCR`.
///
/// `ASPEN` enables automatic context switching (`CONTROL.FPCA`); `LSPEN`
/// enables lazy stacking of `S0`-`S15`/FPSCR on exception entry (the frame is
/// only reserved and written on the first FP instruction the handler runs);
/// `LSPACT` marks an outstanding lazy frame whose address is `FPCAR`.
/// `FPDSCR` supplies the control bits (`DN`, `FZ`, `RMode`, and the trap
/// enables) that a freshly-created FP context's FPSCR is loaded from.
///
/// The state is shared between the core (which consumes it during exception
/// entry/return and FP execution) and the `SCB` device (which exposes it to
/// firmware as `FPCCR` at `0xE000_EF34`, `FPCAR` at `0xE000_EF38`, and
/// `FPDSCR` at `0xE000_EF3C`). It resets to lazy stacking enabled, matching
/// the architecture and QEMU.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FpuCtx {
    /// FPCCR[31]: automatic FP context switching (set `CONTROL.FPCA` on the
    /// first FP instruction of a new context).
    pub aspen: bool,
    /// FPCCR[30]: lazy stacking of FP registers on exception entry.
    pub lspen: bool,
    /// FPCCR[0]: an FP frame is reserved on the stack at `FPCAR` but not yet
    /// written. Set by hardware; the first FP instruction or exception return
    /// resolves it.
    pub lspact: bool,
    /// The address of the outstanding lazy FP frame (bits [31:3]).
    pub fpcar: u32,
    /// FPDSCR: the FPSCR control bits (DN/FZ/RMode/trap enables) loaded into a
    /// new FP context on its first FP instruction.
    pub fpdscr: u32,
}

impl FpuCtx {
    /// Reset state: automatic context switching on, lazy stacking on, the
    /// default FP control bits (round-to-nearest, no traps) all clear.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            aspen: true,
            lspen: true,
            lspact: false,
            fpcar: 0,
            fpdscr: 0,
        }
    }

    /// The modeled `FPCCR` read value: the `ASPEN`/`LSPEN`/`LSPACT` bits plus
    /// the ready bits that are statically set at reset (`HFRDY`, `BFRDY`,
    /// `MMRDY`, `MONRDY`, `UFRDY`).
    #[must_use]
    pub const fn fpccr(&self) -> u32 {
        let mut v = 0x0780_0004; // HFRDY | BFRDY | MMRDY | MONRDY | UFRDY.
        if self.aspen {
            v |= 1 << 31;
        }
        if self.lspen {
            v |= 1 << 30;
        }
        if self.lspact {
            v |= 1 << 0;
        }
        v
    }

    /// Apply a firmware write to `FPCCR`. Only `ASPEN` and `LSPEN` are
    /// software-controllable; `LSPACT` is set and cleared by hardware and the
    /// ready bits are read-only.
    pub fn write_fpccr(&mut self, v: u32) {
        self.aspen = v & (1 << 31) != 0;
        self.lspen = v & (1 << 30) != 0;
    }
}

/// `CONTROL` bit 2: FP extension active. Set by the first FP instruction of a
/// new context (when `FPCCR.ASPEN` is set) and cleared at exception entry.
const CONTROL_FPCA: u32 = 1 << 2;

/// `SCR.SLEEPONEXIT` (bit 1): on exception return to Thread mode the core
/// re-enters sleep instead of resuming the thread, until an exception wakes it.
const SCR_ADDR: u32 = 0xE000_ED10;
const SCR_SLEEPONEXIT: u32 = 0x0000_0002;

/// `CCR` fields the exception paths consult through the bus. `STKALIGN` gates
/// the 8-byte stack alignment on exception entry; `DIV_0_TRP` traps a
/// divide-by-zero. The architectural reset value (`STKALIGN` set) is the
/// fallback when no `SCB` is present on the bus.
const CCR_ADDR: u32 = 0xE000_ED14;
const CCR_STKALIGN: u32 = 0x0000_0200;
const CCR_DIV0_TRP: u32 = 0x0000_0010;
const CCR_RESET: u32 = 0x0000_0200;

/// `AIRCR` (`0xE000_ED0C`): read through the memory window so the
/// exception-escalation logic can see `BFHFNMINS` (bit 1) without a direct
/// SCB handle.
const AIRCR_ADDR: u32 = 0xE000_ED0C;

/// `CFSR`/`HFSR` fault-status bits the exception paths record on the
/// escalation paths that fold a fault into HardFault (see
/// `PeripheralBus::record_fault`). A zero divisor with `DIV_0_TRP` is a
/// UsageFault `DIVBYZERO`; an undecodable instruction is `UNDEFINSTR`; an
/// access to a disabled coprocessor is `NOCP`; a faulting data access is a
/// BusFault `PRECISERR`. Every escalation sets `HFSR.FORCED`.
const UFSR_UNDEFINSTR: u32 = 0x0001_0000; // UFSR bit 0 -> combined bit 16
const UFSR_NOCP: u32 = 0x0002_0000; // UFSR bit 1 -> combined bit 17
const UFSR_DIVBYZERO: u32 = 0x0200_0000; // UFSR bit 9 -> combined bit 25
const UFSR_INVPC: u32 = 0x0008_0000; // UFSR bit 3 -> combined bit 19
const BFSR_PRECISERR: u32 = 0x0000_0100; // BFSR bit 0 -> combined bit 8
// `SFSR` (v8-M `SecureFault`) causes the core records on the exception-return
// failure paths, written through `PeripheralBus::record_secure_fault`.
const SFSR_INVIS: u32 = 0x0000_0002; // callee-saves integrity signature mismatch
const SFSR_INVER: u32 = 0x0000_0004; // invalid Non-secure exception return (ES/DCRS)
                                         // QEMU reports HFSR.FORCED at bit 30 (0x4000_0000) rather than the ARMv7-M
                                         // architectural bit 31; the differential oracle is the arbiter here.
const HFSR_FORCED: u32 = 0x4000_0000;

/// `SHCSR.USGFAULTENA` (bit 18): a distinct `UsageFault` exception, enabled by
/// firmware through the SCB window.
const SHCSR_ADDR: u32 = 0xE000_ED24;
const SHCSR_USGFAULTENA: u32 = 0x0004_0000;

/// `SHPR1` (bits 15:8) holds the configurable `UsageFault` priority byte;
/// `SHPR2`/`SHPR3` hold the SVCall/PendSV/SysTick bytes.
const SHPR1_ADDR: u32 = 0xE000_ED18;
const SHPR2_ADDR: u32 = 0xE000_ED1C;
const SHPR3_ADDR: u32 = 0xE000_ED20;

/// The NVIC `IPR` base: the configurable priority byte for IRQ `n` lives at
/// `IPR + n` (byte-addressable, like the M3/M4).
const NVIC_IPR: u32 = 0xE000_E400;

/// `CPACR` CP10/CP11 (bits 21:20, 23:22): both `0b11` is full FPU access.
/// With no SCB on the bus the fallback is full access, so a bare core keeps
/// the unconditional FPU of the previous model.
const CPACR_ADDR: u32 = 0xE000_ED88;
const CPACR_FULL: u32 = 0x00F0_0000;

/// `SAU_CTRL` (bits 0 `ENABLE`, 1 `ALLNS`), read through the bus from the SCB.
/// With no SCB the fallback is the architectural reset state (disabled).
const SAU_CTRL_ADDR: u32 = 0xE000_EDD0;

/// The SAU/IDAU security attribution for one address, as reported to the
/// `TT` instruction family. The field layout mirrors QEMU's
/// `V8M_SAttributes`, so result bits map one-to-one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SAttrs {
    /// Non-secure attribution.
    ns: bool,
    /// Non-secure-callable attribution.
    nsc: bool,
    /// Matching SAU region number.
    sregion: u8,
    /// Whether an SAU region matched.
    srvalid: bool,
    /// IDAU region number.
    iregion: u8,
    /// Whether an IDAU is present to attribute the address.
    irvalid: bool,
}

/// The core CPU state plus its bus.
pub struct Core<B: PeripheralBus> {
    pub regs: Registers,
    pub xpsr: Xpsr,
    /// VFP (FPv4-SP) registers and FPSCR.
    pub vfp: VfpState,
    /// The ISA this core was constructed for. Immutable after construction.
    pub isa: Isa,
    /// PRIMASK (interrupt masking). Only bit `i` of `CPSIE/CPSID` touches it
    /// on ARMv6-M.
    pub primask: bool,
    /// BASEPRI (ARMv7-M): masks exceptions with priority >= this value.
    pub basepri: u8,
    /// FAULTMASK (ARMv7-M): masks every configurable-priority exception.
    pub faultmask: bool,
    /// CONTROL register: bit0 nPRIV, bit1 SPSEL, bit2 FPCA.
    pub control: u32,
    /// Current exception number: 0 in Thread mode, non-zero in Handler mode.
    pub ipsr: u8,
    /// IT block state (ARMv7-M): bits [3:0] condition, bits [7:4] remaining
    /// conditional instructions. Zero outside an IT block.
    pub itstate: u8,
    /// The local exclusive monitor: the address tagged by the last `LDREX*`.
    /// `None` means open; `STREX*` succeeds only while it matches, and any
    /// exception entry or return clears it.
    pub exmon: Option<u32>,
    /// Whether the core is asleep. Set by `WFI`, `WFE` (when no event is
    /// pending), and by `SCR.SLEEPONEXIT` on exception return to Thread mode. A
    /// sleeping core does not execute until an exception wakes it or (for
    /// `WFE`) an event is signaled; peripherals keep ticking.
    pub sleeping: bool,
    /// The 1-bit event latch used by `WFE`/`SEV`. `SEV` sets it; `WFE` clears
    /// it and, if it was set, does not sleep. Also set when an exception is
    /// taken (an interrupt is itself a wakeup event).
    pub event: bool,
    /// Set by `exception_return`: the last instruction was an `EXC_RETURN`, so
    /// the architecture guarantees the restored instruction executes before a
    /// new exception is taken. The `Machine` consumes it (via `take`) at the
    /// start of the next step to skip that step's exception pre-check.
    pub(crate) just_returned: bool,
    pub cycles: u64,
    /// Shared M-profile FP context-control state (`FPCCR`/`FPCAR`). The `SCB`
    /// device holds the same `Rc` so firmware writes through the bus and the
    /// core's exception/FP logic agree on one value.
    pub fpu_ctx: std::sync::Arc<std::sync::Mutex<FpuCtx>>,
    /// Base address of the reset vector table. Boots on 0x1000_0000-located
    /// flash by default; machines that place flash elsewhere (ARMv8-M boards
    /// at `0x0000_0000`) set this before resetting so the vector table is read
    /// from the right region.
    vector_base: u32,
    /// ARMv8-M baseline stack-limit register for the main stack (`MSPLIM`).
    /// Zero (the reset value) disables the limit. RAZ/WI on ARMv6-M/ARMv7-M.
    msp_lim: u32,
    /// ARMv8-M baseline stack-limit register for the process stack (`PSPLIM`).
    psp_lim: u32,
    /// ARMv8-M TrustZone: the other security state's banked main stack pointer.
    /// The *current* state's SPs live in `regs.msp`/`regs.psp` (one mirroring
    /// the active `regs.sp`); `switch_security_state` swaps the two banks, so
    /// the active SP of whichever state is current is always reachable through
    /// `regs`. ARMv6-M/ARMv7-M have no security extension and never touch these.
    pub(crate) other_ss_msp: u32,
    /// ARMv8-M TrustZone: the other security state's banked process stack
    /// pointer (see `other_ss_msp`).
    pub(crate) other_ss_psp: u32,
    /// ARMv8-M TrustZone: the other security state's `CONTROL` register.
    /// `self.control` holds the current state's `CONTROL`; MRS/MSR of the NS
    /// encodings and the exception-return paths read and write this bank.
    pub(crate) other_ss_control: u32,
    /// ARMv8-M TrustZone: the other security state's `MSPLIM`.
    pub(crate) other_ss_msp_lim: u32,
    /// ARMv8-M TrustZone: the other security state's `PSPLIM`.
    pub(crate) other_ss_psp_lim: u32,
    /// Whether the core is executing in the Secure security state (ARMv8-M
    /// TrustZone). ARMv8-M baseline resets Secure. ARMv6-M/ARMv7-M have no
    /// security extension and keep this false; the `TT` family does not decode
    /// there.
    pub secure: bool,
    mem: B,
}

impl<B: PeripheralBus> Core<B> {
    /// Create an uninitialized ARMv6-M core over `mem`. Register state is zero
    /// until `reset_from_vector_table()` or host code sets it.
    pub fn new(mem: B) -> Self {
        Self::new_with_isa(mem, Isa::Armv6M)
    }

    /// Create an uninitialized ARMv7-M core over `mem`.
    pub fn armv7m(mem: B) -> Self {
        Self::new_with_isa(mem, Isa::Armv7M)
    }

    /// Create an uninitialized ARMv8-M baseline core over `mem`.
    pub fn armv8m(mem: B) -> Self {
        Self::new_with_isa(mem, Isa::Armv8MBaseline)
    }

    /// An ARMv7-M core whose FP context-control state is shared with the `SCB`
    /// device (the firmware-visible `FPCCR`/`FPCAR`). `mem` must be a
    /// `SystemMemory` built with the same `fpu_ctx`; core-only uses keep the
    /// default (architectural reset) state.
    #[must_use]
    pub fn armv7m_with_fpu(mem: B, fpu_ctx: std::sync::Arc<std::sync::Mutex<FpuCtx>>) -> Self {
        let mut core = Self::new_with_isa(mem, Isa::Armv7M);
        core.fpu_ctx = fpu_ctx;
        core
    }

    fn new_with_isa(mem: B, isa: Isa) -> Self {
        Self {
            regs: Registers::default(),
            xpsr: Xpsr::new(XPSR_T),
            vfp: VfpState::default(),
            isa,
            primask: false,
            basepri: 0,
            faultmask: false,
            control: 0,
            ipsr: 0,
            itstate: 0,
            exmon: None,
            sleeping: false,
            event: false,
            just_returned: false,
            cycles: 0,
            fpu_ctx: std::sync::Arc::new(std::sync::Mutex::new(FpuCtx::new())),
            vector_base: 0,
            msp_lim: 0,
            psp_lim: 0,
            other_ss_msp: 0,
            other_ss_psp: 0,
            other_ss_control: 0,
            other_ss_msp_lim: 0,
            other_ss_psp_lim: 0,
            secure: isa == Isa::Armv8MBaseline,
            mem,
        }
    }

    /// Borrow the bus.
    pub fn mem(&mut self) -> &mut B {
        &mut self.mem
    }

    /// Set the base address of the reset vector table (see the `vector_base`
    /// field). Defaults to `0x0000_0000`.
    pub fn set_vector_base(&mut self, base: u32) {
        self.vector_base = base;
    }

    /// Fetch and execute a single instruction. On success the machine has
    /// advanced exactly one instruction; on error it has *not*. Interrupts
    /// are taken by the `Machine`, not here.
    pub fn step(&mut self) -> Result<(), HaltCause> {
        // A sleeping core (`SCR.SLEEPONEXIT`) does not execute; the machine
        // still advances peripherals and re-checks for a waking exception
        // each cycle.
        if self.sleeping {
            return Ok(());
        }
        let pc = self.regs.pc;
        let next = match self.isa {
            Isa::Armv6M => self.step_v6m(pc)?,
            Isa::Armv7M => self.step_v7m(pc)?,
            Isa::Armv8MBaseline => self.step_v8m(pc)?,
        };
        match next {
            Next::Advance(a) => self.regs.pc = a,
            Next::Stop(cause) => return Err(cause),
        }
        self.cycles += 1;
        Ok(())
    }

    /// Fetch and execute a single 16-bit-centric ARMv6-M instruction.
    fn step_v6m(&mut self, pc: u32) -> Result<Next, HaltCause> {
        if !self.mpu_allows(pc, MpuAccess::Fetch) {
            return self.mpu_fault(pc, pc, true);
        }
        let hw = self.mem.read_half(pc).map_err(|_| HaltCause::FetchAbort)?;
        if is_thumb2(hw) {
            let lo = self
                .mem
                .read_half(pc.wrapping_add(2))
                .map_err(|_| HaltCause::FetchAbort)?;
            match decode32(hw, lo) {
                Ok(inst) => self.exec32(pc, inst),
                Err(_) => self.hardfault_on_undefined(pc, 4),
            }
        } else {
            match decode16(hw) {
                Ok(inst) => self.exec(pc, inst),
                Err(_) => self.hardfault_on_undefined(pc, 2),
            }
        }
    }

    /// Fetch and execute a single ARMv7-M instruction.
    ///
    /// ARMv7-M freely intermixes 16-bit and 32-bit instructions at any even
    /// address, so a 32-bit instruction can start on the second halfword of an
    /// aligned word (`pc & 2 == 2`). The two halfwords are therefore read
    /// from `pc` and `pc + 2` directly; the parallel aligned-word fetch would
    /// mispair the halves for that case.
    fn step_v7m(&mut self, pc: u32) -> Result<Next, HaltCause> {
        if pc & 1 != 0 {
            return Err(HaltCause::FetchAbort);
        }
        if !self.mpu_allows(pc, MpuAccess::Fetch) {
            return self.mpu_fault(pc, pc, true);
        }
        let h0 = self.mem.read_half(pc).map_err(|_| HaltCause::FetchAbort)?;
        let h1 = self
            .mem
            .read_half(pc.wrapping_add(2))
            .map_err(|_| HaltCause::FetchAbort)?;
        if is_thumb2_v7m(h0) {
            match decode32_v7m(h0, h1) {
                Ok(inst) => self.exec_it_step(pc, 4, |s| s.exec32(pc, inst)),
                Err(_) => self.hardfault_on_undefined(pc, 4),
            }
        } else {
            match decode16_v7m(h0) {
                Ok(inst) => self.exec_it_step(pc, 2, |s| s.exec(pc, inst)),
                Err(_) => self.hardfault_on_undefined(pc, 2),
            }
        }
    }

    /// Fetch and execute a single ARMv8-M baseline instruction.
    ///
    /// ARMv8-M baseline mixes 16-bit and 32-bit instructions like ARMv7-M and
    /// supports IT blocks, so the fetch and 16-bit path mirror `step_v7m`.
    /// The 32-bit path is the ARMv7-M Thumb-2 decoder filtered down to the
    /// baseline subset (`sdiv`/`udiv`, `movw`/`movt`, `b.w`, the exclusive
    /// family, `clrex`, and the shared `bl`/`mrs`/`msr`/barriers); every
    /// other Thumb-2 encoding faults as undefined, matching Cortex-M23.
    fn step_v8m(&mut self, pc: u32) -> Result<Next, HaltCause> {
        if pc & 1 != 0 {
            return Err(HaltCause::FetchAbort);
        }
        if !self.mpu_allows(pc, MpuAccess::Fetch) {
            return self.mpu_fault(pc, pc, true);
        }
        let h0 = self.mem.read_half(pc).map_err(|_| HaltCause::FetchAbort)?;
        let h1 = self
            .mem
            .read_half(pc.wrapping_add(2))
            .map_err(|_| HaltCause::FetchAbort)?;
        if is_thumb2_v7m(h0) {
            match decode32_v8m(h0, h1) {
                // `SG` is unconditional even inside an IT block (QEMU's
                // `thumb_insn_is_unconditional` treats `0xE97FE97F` as
                // unconditional): it executes and clears ITSTATE regardless of
                // the pending condition, so it must bypass `exec_it_step`.
                Ok(inst @ Inst32::Sg) => self.exec32(pc, inst),
                Ok(inst) => self.exec_it_step(pc, 4, |s| s.exec32(pc, inst)),
                Err(_) => self.hardfault_on_undefined(pc, 4),
            }
        } else {
            match decode16_v7m(h0) {
                Ok(inst) => self.exec_it_step(pc, 2, |s| s.exec(pc, inst)),
                Err(_) => self.hardfault_on_undefined(pc, 2),
            }
        }
    }

    /// ARMv7-M conditional-execution wrapper.
    ///
    /// When `itstate` is nonzero the current instruction sits inside an IT
    /// block. Its condition is `itstate >> 4`; if it fails the instruction is
    /// suppressed (no side effects) but ITSTATE still advances and the PC
    /// moves on by `step`. When it passes, `exec` runs and ITSTATE advances
    /// on a normal fall-through. A taken branch clears ITSTATE in its own arm
    /// and skips the advance here, and an exception entry/stop discards it.
    fn exec_it_step(
        &mut self,
        pc: u32,
        step: u32,
        exec: impl FnOnce(&mut Self) -> Result<Next, HaltCause>,
    ) -> Result<Next, HaltCause> {
        if self.itstate == 0 {
            return exec(self);
        }
        let cond = self.itstate >> 4;
        if !self.cond_passes(cond) {
            self.it_advance();
            return Ok(Next::Advance(pc + step));
        }
        let next = exec(self)?;
        match next {
            Next::Advance(a) if a == pc + step => {
                self.it_advance();
                Ok(next)
            }
            // Branch taken or exception/halt: the branch path clears ITSTATE,
            // exception entry clears it, and stops discard it.
            _ => Ok(next),
        }
    }

    /// Advance the ITSTATE to the next conditional instruction, per the
    /// architecture's `ITAdvance()`: if the block has one instruction left
    /// (`ITSTATE<2:0> == 000`) it clears; otherwise the low five bits shift
    /// left so the next instruction's condition bit lands in `ITSTATE<4>`.
    fn it_advance(&mut self) {
        if self.itstate.trailing_zeros() >= 3 {
            self.itstate = 0;
        } else {
            self.itstate = (self.itstate & 0xE0) | (((self.itstate & 0x1F) << 1) & 0x1F);
        }
    }

    /// Execute until the machine halts of its own accord, or `budget`
    /// instructions have been executed (whichever comes first).
    pub fn run(&mut self, budget: u64) -> Result<(), HaltCause> {
        for _ in 0..budget {
            self.step()?;
        }
        Err(HaltCause::CycleBudgetExhausted)
    }

    /// Bootstrap the core from the reset vector table: `sp = [vector_base]`,
    /// `pc = [vector_base + 4] & ~1`. Returns `NonThumbTarget` if the reset
    /// vector is not a Thumb address.
    pub fn reset_from_vector_table(&mut self) -> Result<(), HaltCause> {
        let sp = self.read_word_halt(self.vector_base)?;
        let pc = self.read_word_halt(self.vector_base.wrapping_add(4))?;
        if pc & 1 == 0 {
            return Err(HaltCause::NonThumbTarget);
        }
        self.regs.sp = sp;
        self.regs.msp = sp;
        self.regs.psp = 0;
        self.regs.pc = pc & !1;
        self.xpsr = Xpsr::new(XPSR_T);
        self.primask = false;
        self.basepri = 0;
        self.faultmask = false;
        self.control = 0;
        self.ipsr = 0;
        self.itstate = 0;
        self.sleeping = false;
        self.event = false;
        self.just_returned = false;
        // ARMv8-M: the banked other-state SPs, CONTROL and stack limits all
        // reset to zero (the architectural reset state).
        self.other_ss_msp = 0;
        self.other_ss_psp = 0;
        self.other_ss_control = 0;
        self.other_ss_msp_lim = 0;
        self.other_ss_psp_lim = 0;
        Ok(())
    }

    /// Point-in-time register snapshot for host tooling (cli, debugger).
    pub const fn snapshot(&self) -> (Registers, Xpsr, u64) {
        (self.regs, self.xpsr, self.cycles)
    }

    fn read_word_halt(&mut self, addr: u32) -> Result<u32, HaltCause> {
        self.mem
            .read_word(addr)
            .map_err(|_| HaltCause::BusFault { address: addr })
    }

    fn write_word_halt(&mut self, addr: u32, v: u32) -> Result<(), HaltCause> {
        self.mem
            .write_word(addr, v)
            .map_err(|_| HaltCause::BusFault { address: addr })
    }

    /// Whether the active stack pointer is the process SP (`SPSEL == 1`).
    /// In Handler mode the active SP is always the main stack pointer, so
    /// `SPSEL` is only consulted for thread-mode accesses.
    const fn sp_is_process(&self) -> bool {
        self.ipsr == 0 && self.control & 0x2 != 0
    }

    /// Deposit the given thread/process selection into the `SPSEL` bit of the
    /// given security state's `CONTROL`, mirroring QEMU's
    /// `write_v7m_control_spsel_for_secstate`. Only a change to the *current*
    /// state's `SPSEL` can move the active stack pointer, and then only when
    /// it is done in Thread mode (`v7m_using_psp` is false in Handler mode),
    /// so the active/inactive SPs swap only in that case.
    fn write_control_spsel_for_secstate(&mut self, secstate: bool, new_spsel: bool) {
        if secstate == self.secure {
            let old_is_psp = self.sp_is_process();
            self.control = (self.control & !0x2) | u32::from(new_spsel) << 1;
            let new_is_psp = self.sp_is_process();
            if old_is_psp != new_is_psp {
                let r = &mut self.regs;
                if new_is_psp {
                    r.msp = r.sp;
                    r.sp = r.psp;
                } else {
                    r.psp = r.sp;
                    r.sp = r.msp;
                }
            }
        } else {
            self.other_ss_control =
                (self.other_ss_control & !0x2) | u32::from(new_spsel) << 1;
        }
    }

    /// Switch the running security state to `new_secstate`, swapping the two
    /// banked SPs (and, since the current `CONTROL` and stack-limit fields hold
    /// the current state's values, those banks too). Mirrors QEMU's
    /// `switch_v7m_security_state`: the current state's active and inactive SPs
    /// are stashed into the other state's bank, then the other state's bank is
    /// loaded with the active SP selected by the new state's `CONTROL.SPSEL`
    /// and the (unbanked, still-current) thread/handler mode.
    fn switch_security_state(&mut self, new_secstate: bool) {
        if new_secstate == self.secure {
            return;
        }
        let new_ss_msp = self.other_ss_msp;
        let new_ss_psp = self.other_ss_psp;
        // Save the current state's bank into the other-state slots.
        if self.sp_is_process() {
            self.other_ss_psp = self.regs.sp;
            self.other_ss_msp = self.regs.msp;
        } else {
            self.other_ss_msp = self.regs.sp;
            self.other_ss_psp = self.regs.psp;
        }
        self.secure = new_secstate;
        self.mem.set_secure(new_secstate);
        // The single current-state CONTROL/limit fields now hold the new
        // state's values.
        std::mem::swap(&mut self.control, &mut self.other_ss_control);
        std::mem::swap(&mut self.msp_lim, &mut self.other_ss_msp_lim);
        std::mem::swap(&mut self.psp_lim, &mut self.other_ss_psp_lim);
        // Load the new state's bank, honoring its SPSEL in thread mode.
        if self.sp_is_process() {
            self.regs.psp = new_ss_psp;
            self.regs.msp = new_ss_msp;
            self.regs.sp = new_ss_psp;
        } else {
            self.regs.msp = new_ss_msp;
            self.regs.psp = new_ss_psp;
            self.regs.sp = new_ss_msp;
        }
    }

    /// The SAU/IDAU security attribution for `addr` in the security state
    /// `targetsec` (the state the `TT` family *probes*), matching QEMU's
    /// `v8m_security_lookup`. The SAU registers live in the SCB and are read
    /// through the bus: with no SCB (or the reset state) `SAU_CTRL` is
    /// `ENABLE == 0, ALLNS == 0`, attribution case 0 where the IDAU decides
    /// (and cannot downgrade, so everything defaults Secure). The SAU/IDAU
    /// exempt system aliases are attributed `!targetsec`; an `ENABLE == 1`
    /// region match sets the region/validity fields and the NS/NSC attribute;
    /// and the IDAU can only strengthen a Non-secure attribution to Secure,
    /// never downgrade to Non-secure.
    fn security_attribution(&mut self, addr: u32, targetsec: bool) -> SAttrs {
        let idau = self.mem.idau_attribution(addr);
        let mut s = SAttrs {
            ns: false,
            nsc: false,
            sregion: 0,
            srvalid: false,
            iregion: 0,
            irvalid: false,
        };
        // The architectural SAU-exempt system aliases (subsumed by the ARMSSE
        // IDAU's own exempt range, but kept for a no-IDAU machine).
        let sau_exempt = matches!(
            addr,
            0xE000_0000..=0xE000_2FFF
                | 0xE000_E000..=0xE000_EFFF
                | 0xE002_E000..=0xE002_EFFF
                | 0xE004_0000..=0xE004_1FFF
                | 0xE00F_F000..=0xE00F_FFFF
        );
        if sau_exempt || matches!(idau, Some(i) if i.exempt) {
            s.ns = !targetsec;
            return s;
        }
        if let Some(idau) = idau {
            s.irvalid = true;
            s.iregion = idau.region;
        }
        // `SAU_CTRL`, read through the bus (0 with no SCB on the bus: the
        // architectural reset state).
        let sau_ctrl = self.mem.read_word(SAU_CTRL_ADDR).unwrap_or(0) & 0x3;
        match sau_ctrl {
            // `SAU.ENABLE == 0, SAU.ALLNS == 0`: the IDAU decides (default
            // Secure; the override below can only upgrade).
            0 => {}
            // `SAU.ENABLE == 0, SAU.ALLNS == 1`: everything defaults
            // Non-secure, subject to the IDAU upgrade below.
            2 => s.ns = true,
            // `SAU.ENABLE == 1`: the enabled regions decide. Matching QEMU,
            // a no-hit address stays Secure (the ARM's default-NS is not what
            // the oracle does) and a multi-hit collapses to Secure with no
            // valid region number.
            _ => match self.mem.sau_hit(addr) {
                SauHit::None | SauHit::Multiple => {}
                SauHit::Region { index, nsc } => {
                    if nsc {
                        s.nsc = true;
                    } else {
                        s.ns = true;
                    }
                    s.srvalid = true;
                    s.sregion = index;
                }
            },
        }
        // The IDAU overrides the SAU with higher security (QEMU's final
        // block): an IDAU-Secure region upgrades an NS/NSC attribution, but
        // never downgrades.
        if let Some(idau) = idau {
            if !idau.ns && (s.ns || (!idau.nsc && s.nsc)) {
                s.ns = false;
                s.nsc = idau.nsc;
            }
        }
        s
    }

    /// Read a system register for `mrs`.
    fn sysreg_read(&self, sys: SysReg) -> Result<u32, HaltCause> {
        // In an unprivileged (Thread, nPRIV=1) context only the xPSR subfields
        // and CONTROL are readable; reads of the remaining special-purpose
        // registers return 0 (ARMv7-M ARM; matches QEMU's `v7m_mrs`).
        let unprivileged = self.ipsr == 0 && self.control & 1 != 0;
        let v = match sys {
            // Reads return the whole APSR; the `_g`/`_nzcvqg` qualifiers only
            // constrain the *write* mask.
            SysReg::Apsr | SysReg::ApsrG | SysReg::ApsrNzvCqg => self.xpsr.get() & APSR_QFLAGS,
            SysReg::Xpsr => {
                // xPSR also carries ITSTATE at bits [15:10] and [26:25].
                let mut v = self.xpsr.get() & (APSR_QFLAGS | XPSR_T);
                let it = u32::from(self.itstate);
                v |= (it & 0xFC) << 8;
                v |= (it & 0x03) << 25;
                v
            }
            SysReg::Ipsr => u32::from(self.ipsr),
            SysReg::Epsr => XPSR_T,
            // ARMv6-M (Cortex-M0/M0+) and ARMv8-M baseline (Cortex-M23) have
            // no BASEPRI or FAULTMASK register: reads of their sysreg
            // encodings are UNPREDICTABLE on silicon, and the QEMU M0/M33
            // references RAZ them.
            SysReg::Basepri | SysReg::BasepriMax | SysReg::Faultmask
                if self.isa != Isa::Armv7M =>
            {
                0
            }
            // The ARMv8-M stack-limit registers only exist on the baseline
            // here; their encodings RAZ on ARMv6-M/ARMv7-M like BASEPRI.
            SysReg::MspLim | SysReg::PspLim if self.isa != Isa::Armv8MBaseline => 0,
            // `CONTROL_NS` is readable by unprivileged *Secure* code (QEMU
            // `v7m_mrs` case 0x94); from the Non-secure state it reads 0. The
            // S-bank `FPCA` is folded into the NS value; the baseline never
            // sets it, so this is just the NS bank here.
            SysReg::ControlNs if !self.secure => 0,
            SysReg::ControlNs => {
                self.other_ss_control | (self.control & CONTROL_FPCA)
            }
            // Unprivileged reads of these special-purpose registers are RAZ.
            SysReg::Msp
            | SysReg::Psp
            | SysReg::Primask
            | SysReg::Basepri
            | SysReg::BasepriMax
            | SysReg::Faultmask
            | SysReg::MspLim
            | SysReg::PspLim
            | SysReg::MspNs
            | SysReg::PspNs
            | SysReg::MsplimNs
            | SysReg::PsplimNs
            | SysReg::SpNs
                if unprivileged =>
            {
                0
            }
            // The Non-secure banked encodings exist only on ARMv8-M (the
            // `secure` state is always false on ARMv6-M/ARMv7-M, so the next
            // arm's `!secure` gate already RAZs them there; kept explicit for
            // clarity).
            SysReg::MspNs
            | SysReg::PspNs
            | SysReg::MsplimNs
            | SysReg::PsplimNs
            | SysReg::SpNs
                if self.isa != Isa::Armv8MBaseline =>
            {
                0
            }
            // From the Non-secure state the NS encodings read as 0 (QEMU
            // `v7m_mrs`: `if (!env->v7m.secure) return 0`).
            SysReg::MspNs
            | SysReg::PspNs
            | SysReg::MsplimNs
            | SysReg::PsplimNs
            | SysReg::SpNs
                if !self.secure =>
            {
                0
            }
            SysReg::Msp => {
                if self.sp_is_process() {
                    self.regs.msp
                } else {
                    self.regs.sp
                }
            }
            SysReg::Psp => {
                if self.sp_is_process() {
                    self.regs.sp
                } else {
                    self.regs.psp
                }
            }
            SysReg::Primask => u32::from(self.primask),
            SysReg::MspLim => self.msp_lim,
            SysReg::PspLim => self.psp_lim,
            SysReg::Basepri | SysReg::BasepriMax => u32::from(self.basepri),
            SysReg::Faultmask => u32::from(self.faultmask),
            SysReg::Control => self.control,
            SysReg::MspNs => self.other_ss_msp,
            SysReg::PspNs => self.other_ss_psp,
            SysReg::MsplimNs => self.other_ss_msp_lim,
            SysReg::PsplimNs => self.other_ss_psp_lim,
            // `SP_NS` selects between the NS main and process SPs by the NS
            // `CONTROL.SPSEL` and the current thread/handler mode (QEMU
            // `v7m_mrs` case 0x98).
            SysReg::SpNs => {
                if self.ipsr == 0 && self.other_ss_control & 0x2 != 0 {
                    self.other_ss_psp
                } else {
                    self.other_ss_msp
                }
            }
            SysReg::Other(_) => {
                return Err(HaltCause::NotImplemented {
                    what: "mrs of reserved system register",
                })
            }
        };
        Ok(v)
    }

    /// Write a system register for `msr`.
    fn sysreg_write(&mut self, sys: SysReg, v: u32) -> Result<(), HaltCause> {
        match sys {
            SysReg::Apsr => {
                let cur = self.xpsr.get();
                self.xpsr.set((cur & !APSR_FLAGS) | (v & APSR_FLAGS));
                self.xpsr.set_q(v & APSR_Q != 0);
            }
            SysReg::ApsrG => {
                let cur = self.xpsr.get();
                self.xpsr.set((cur & !APSR_GE) | (v & APSR_GE));
            }
            SysReg::ApsrNzvCqg => {
                self.xpsr
                    .set((self.xpsr.get() & !APSR_QFLAGS) | (v & APSR_QFLAGS));
            }
            SysReg::Msp => {
                let v = v & !3;
                if self.sp_is_process() {
                    self.regs.msp = v;
                } else {
                    self.regs.sp = v;
                }
            }
            SysReg::Psp => {
                let v = v & !3;
                if self.sp_is_process() {
                    self.regs.sp = v;
                } else {
                    self.regs.psp = v;
                }
            }
            SysReg::Primask => self.primask = v & 1 != 0,
            // ARMv6-M and ARMv8-M baseline have no BASEPRI/FAULTMASK: writes
            // are UNPREDICTABLE on silicon and ignored by the QEMU references.
            SysReg::Basepri | SysReg::BasepriMax | SysReg::Faultmask
                if self.isa != Isa::Armv7M =>
            {
            }
            // The stack-limit registers only exist on the ARMv8-M baseline
            // here; their encodings are RAZ/WI on ARMv6-M/ARMv7-M.
            SysReg::MspLim | SysReg::PspLim if self.isa != Isa::Armv8MBaseline => {} 
            SysReg::MspLim => self.msp_lim = v,
            SysReg::PspLim => self.psp_lim = v,
            SysReg::Basepri => self.basepri = v as u8,
            SysReg::BasepriMax => {
                // Raises the mask (moves toward higher priority / lower
                // number); writes that would reduce it are ignored, and a
                // zero never re-enables a set mask.
                let new = v as u8;
                if new != 0 && (self.basepri == 0 || new < self.basepri) {
                    self.basepri = new;
                }
            }
            SysReg::Faultmask => self.faultmask = v & 1 != 0,
            SysReg::Control if self.isa == Isa::Armv6M => {
                // ARMv6-M (Cortex-M0/M0+): CONTROL.nPRIV and CONTROL.FPCA are
                // RAZ/WI â€” there is no unprivileged state and no FPU â€” so only
                // SPSEL is writable, and only in Thread mode (Handler mode
                // always uses MSP). Matches QEMU, which gates nPRIV on
                // ARM_FEATURE_M_MAIN and SPSEL on thread mode.
                if self.ipsr == 0 {
                    let new = v & 0x2;
                    let want_process = new != 0;
                    let was_process = self.sp_is_process();
                    if want_process != was_process {
                        let r = &mut self.regs;
                        if want_process {
                            r.msp = r.sp;
                            r.sp = r.psp;
                        } else {
                            r.psp = r.sp;
                            r.sp = r.msp;
                        }
                    }
                    self.control = new;
                }
            }
            SysReg::Control => {
                // MSR CONTROL is only honored in a privileged context: Handler
                // mode, or Thread mode while nPRIV is still clear. Once the
                // thread drops to unprivileged (nPRIV=1), further writes are
                // ignored, so an unprivileged thread cannot re-privilege
                // itself. ARMv8-M baseline has no FPU, so CONTROL.FPCA is
                // RAZ/WI there (bit 2 masked off); ARMv7-M keeps it. The SPSEL
                // write only switches the active SP when it actually changes
                // the SP in use (`sp_is_process`); in Handler mode both sides
                // are false, so no swap happens there (QEMU
                // `write_v7m_control_spsel`).
                if self.ipsr != 0 || self.control & 1 == 0 {
                    let mask = if self.isa == Isa::Armv7M { 0x7 } else { 0x3 };
                    let new = v & mask;
                    let old_is_psp = self.sp_is_process();
                    self.control = new;
                    let new_is_psp = self.sp_is_process();
                    if old_is_psp != new_is_psp {
                        let r = &mut self.regs;
                        if new_is_psp {
                            r.msp = r.sp;
                            r.sp = r.psp;
                        } else {
                            r.psp = r.sp;
                            r.sp = r.msp;
                        }
                    }
                }
            }
            // The ARMv8-M Non-secure banked registers. All are RAZ/WI from the
            // Non-secure state (unreachable anyway: `secure` is false on
            // ARMv6-M/ARMv7-M) and ignored by unprivileged Secure code, so the
            // whole family is guarded on "privileged Secure".
            SysReg::MspNs
            | SysReg::PspNs
            | SysReg::MsplimNs
            | SysReg::PsplimNs
            | SysReg::ControlNs
            | SysReg::SpNs
                if !(self.secure && (self.ipsr != 0 || self.control & 1 == 0)) =>
            {
            }
            SysReg::MspNs => self.other_ss_msp = v & !3,
            SysReg::PspNs => self.other_ss_psp = v & !3,
            SysReg::MsplimNs => self.other_ss_msp_lim = v & !7,
            SysReg::PsplimNs => self.other_ss_psp_lim = v & !7,
            // `CONTROL_NS` (0x94): deposit SPSEL into the NS bank and write
            // nPRIV (matching the M_MAIN M33 oracle; the baseline's own
            // CONTROL keeps the same M_MAIN-like behavior for parity).
            SysReg::ControlNs => {
                self.other_ss_control = (self.other_ss_control & !0x3) | (v & 0x3);
            }
            // `SP_NS` (0x98): writes the NS SP selected by the NS
            // `CONTROL.SPSEL` and the current mode. QEMU raises a StackFault
            // when the value falls below the NS limit; VioARM skips that
            // immediate check (documented divergence; unreachable in the
            // conformance suite).
            SysReg::SpNs => {
                let val = v & !3;
                if self.ipsr == 0 && self.other_ss_control & 0x2 != 0 {
                    self.other_ss_psp = val;
                } else {
                    self.other_ss_msp = val;
                }
            }
            SysReg::Xpsr | SysReg::Ipsr | SysReg::Epsr => {
                return Err(HaltCause::NotImplemented {
                    what: "msr to xpsr/ipsr/epsr",
                })
            }
            SysReg::Other(_) => {
                return Err(HaltCause::NotImplemented {
                    what: "msr of reserved system register",
                })
            }
        }
        Ok(())
    }

    /// Evaluate an ARM condition code against the current flags.
    fn cond_passes(&self, cond: u8) -> bool {
        let (n, z, c, v) = (self.xpsr.n(), self.xpsr.z(), self.xpsr.c(), self.xpsr.v());
        match cond {
            0x0 => z,            // EQ
            0x1 => !z,           // NE
            0x2 => c,            // CS/HS
            0x3 => !c,           // CC/LO
            0x4 => n,            // MI
            0x5 => !n,           // PL
            0x6 => v,            // VS
            0x7 => !v,           // VC
            0x8 => c && !z,      // HI
            0x9 => !c || z,      // LS
            0xA => n == v,       // GE
            0xB => n != v,       // LT
            0xC => !z && n == v, // GT
            0xD => z || n != v,  // LE
            _ => unreachable!("cond {cond:#x}"),
        }
    }

    /// Sign-extend the bottom (`top == false`) or top (`top == true`) halfword
    /// of `v` to a signed 32-bit value.
    const fn half_signed(&self, v: u32, top: bool) -> i32 {
        let hw = if top { (v >> 16) as u16 } else { v as u16 };
        hw as i16 as i32
    }

    /// The two halfword products of `n` and `m` (with `m`'s halves swapped
    /// first when `m_swap`), each as a signed 32-bit value. A 16-by-16 signed
    /// product always fits, so no overflow is possible.
    const fn smul_dual(&self, n: u32, m: u32, m_swap: bool) -> (i32, i32) {
        let ms = if m_swap { (m >> 16) | (m << 16) } else { m };
        let p_lo = self
            .half_signed(n, false)
            .wrapping_mul(self.half_signed(ms, false));
        let p_hi = self
            .half_signed(n, true)
            .wrapping_mul(self.half_signed(ms, true));
        (p_lo, p_hi)
    }

    /// Wrapping signed 32-bit addition that sets the sticky `Q` flag on
    /// overflow, mirroring QEMU's `add_setq` helper.
    fn add_setq(&mut self, a: i32, b: i32) -> i32 {
        let (result, overflow) = a.overflowing_add(b);
        if overflow {
            self.xpsr.set_q(true);
        }
        result
    }

    /// Sum the absolute differences of the four byte lanes of `n` and `m`.
    const fn sum_abs_diff(&self, n: u32, m: u32) -> i32 {
        let d0 = ((n & 0xFF) as i32) - ((m & 0xFF) as i32);
        let d1 = (((n >> 8) & 0xFF) as i32) - (((m >> 8) & 0xFF) as i32);
        let d2 = (((n >> 16) & 0xFF) as i32) - (((m >> 16) & 0xFF) as i32);
        let d3 = (((n >> 24) & 0xFF) as i32) - (((m >> 24) & 0xFF) as i32);
        d0.abs() + d1.abs() + d2.abs() + d3.abs()
    }

    /// Execute `inst` at address `pc`. Semantic meaning only; no memory
    /// acquisition happens here.
    fn exec(&mut self, pc: u32, inst: Inst16) -> Result<Next, HaltCause> {
        use Inst16 as I;
        match inst {
            I::ShiftImm {
                rd,
                rm,
                kind,
                amount,
            } => {
                let v = self.regs.read(rm);
                let (res, c) = shift_imm(kind, v, u32::from(amount));
                self.write_shift_flags_it(res, c);
                self.regs.write(rd, res);
                Ok(Next::Advance(pc + 2))
            }
            I::ShiftReg { rd, rn, rm, kind } => {
                let v = self.regs.read(rn);
                let count = self.regs.read(rm) & 0xFF;
                let (res, c) = shift_reg(kind, v, count);
                self.write_shift_flags_it(res, c);
                self.regs.write(rd, res);
                Ok(Next::Advance(pc + 2))
            }
            I::AddReg { rd, rn, rm } => {
                let (res, c, v) = add_flags(self.regs.read(rn), self.regs.read(rm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubReg { rd, rn, rm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), self.regs.read(rm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::AddImm3 { rd, rn, imm } => {
                let (res, c, v) = add_flags(self.regs.read(rn), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubImm3 { rd, rn, imm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::MovImm { rd, imm } => {
                let res = u32::from(imm);
                self.regs.write(rd, res);
                self.set_nz_it(res, false);
                Ok(Next::Advance(pc + 2))
            }
            I::CmpImm { rn, imm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), u32::from(imm), false);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::AddImm8 { rd, imm } => {
                let (res, c, v) = add_flags(self.regs.read(rd), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubImm8 { rd, imm } => {
                let (res, c, v) = sub_flags(self.regs.read(rd), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::Alu { rd, rm, op } => {
                self.exec_alu(rd, rm, op, self.itstate == 0);
                Ok(Next::Advance(pc + 2))
            }
            I::AddSpImm8 { imm } => {
                self.regs.sp = self.regs.sp.wrapping_add(imm as u32);
                Ok(Next::Advance(pc + 2))
            }
            I::SubSpImm8 { imm } => {
                self.regs.sp = self.regs.sp.wrapping_sub(imm as u32);
                Ok(Next::Advance(pc + 2))
            }
            I::SpecialAdd { rd, rm } => {
                let b = self.regs.read(rm);
                if rd == 15 {
                    return self.branch(pc.wrapping_add(4).wrapping_add(b));
                }
                self.regs.write(rd, self.regs.read(rd) + b);
                Ok(Next::Advance(pc + 2))
            }
            I::SpecialCmp { rn, rm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), self.regs.read(rm), false);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SpecialMov { rd, rm } => {
                let b = self.regs.read(rm);
                if rd == 15 {
                    return self.branch(b);
                }
                self.regs.write(rd, b);
                Ok(Next::Advance(pc + 2))
            }
            I::Bx { rm } => self.branch(self.regs.read(rm)),
            I::Blx { rm } => {
                let target = self.regs.read(rm);
                self.itstate = 0;
                self.regs.lr = (pc + 2) | 1;
                Ok(Self::check_branch(target))
            }
            I::BxNs { rm } => self.exec_bxns(self.regs.read(rm)),
            I::BlxNs { rm } => self.exec_blxns(pc, self.regs.read(rm)),
            I::LdrLiteral { rt, imm } => {
                let addr = ((pc + 4) & !3) + imm as u32;
                if !self.mpu_allows(addr, MpuAccess::Read) {
                    return self.mpu_fault(addr, pc, false);
                }
                let v = match self.mem.read_word(addr) {
                    Ok(v) => v,
                    Err(_) => return self.data_fault(addr, pc),
                };
                self.write_rt(rt, v, addr, pc)
            }
            I::LsReg { rt, rn, rm, kind } => {
                let addr = self.regs.read(rn).wrapping_add(self.regs.read(rm));
                self.exec_ls(addr, rt, kind.into(), pc)
            }
            I::LsImm { rt, rn, imm, kind } => {
                let addr = self.regs.read(rn).wrapping_add(imm as u32);
                self.exec_ls(addr, rt, kind.into(), pc)
            }
            I::LsSp { rt, imm, is_load } => {
                let addr = self.regs.sp.wrapping_add(imm as u32);
                self.exec_ls(addr, rt, LsAccess::Word { load: is_load }, pc)
            }
            I::AddrPc { rd, imm } => {
                let v = ((pc + 4) & !3) + imm as u32;
                self.regs.write(rd, v);
                Ok(Next::Advance(pc + 2))
            }
            I::AddrSp { rd, imm } => {
                self.regs.write(rd, self.regs.sp.wrapping_add(imm as u32));
                Ok(Next::Advance(pc + 2))
            }
            I::BranchCond { cond, off } => {
                if self.cond_passes(cond) {
                    self.itstate = 0;
                    Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
                } else {
                    Ok(Next::Advance(pc + 2))
                }
            }
            I::Branch { off } => {
                self.itstate = 0;
                Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
            }
            I::CbzCbnz { rn, off, nonzero } => {
                let is_zero = self.regs.read(rn) == 0;
                let taken = if nonzero { !is_zero } else { is_zero };
                if taken {
                    self.itstate = 0;
                    Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
                } else {
                    Ok(Next::Advance(pc + 2))
                }
            }
            I::Svc { .. } => {
                // ARMv7-M: a 16-bit SVC takes the SVCall exception; the
                // stacked return address is the instruction after the SVC so
                // the handler can continue with `bx lr`. The immediate is
                // service data for software only.
                //
                // A SVC executed while a handler is already active cannot
                // complete: it has priority 0 (or higher) while any handler in
                // the set we model runs at priority <= 0, so taking it would
                // not preempt. Likewise, a SVC masked by PRIMASK (or FAULTMASK)
                // is a synchronous exception that cannot be taken, so it can
                // never simply drain from the nesting stack. Real hardware
                // escalates both cases to a HardFault faulting at the SVC
                // itself. Matching that here keeps the handler from
                // re-entering itself recursively (or the thread from
                // launching a SVC that must be deferred forever).
                if self.ipsr != 0 || self.primask || self.faultmask {
                    // Escalate to HardFault. The stacked return address keeps
                    // the *normal* SVC semantics (the instruction after the
                    // SVC): the SVC itself executes, only its exception entry
                    // cannot be honored.
                    //
                    // FAULTMASK is special: it raises the current priority to
                    // -1, so the escalated HardFault (also priority -1) cannot
                    // activate at all and the core locks up. The SVC still
                    // executes (the PC advances past it) before the lockup,
                    // matching QEMU's halted frame at svc+2.
                    if self.faultmask {
                        self.regs.pc = pc.wrapping_add(2);
                        return Err(HaltCause::UnhandledException);
                    }
                    let targets_secure = self.hardfault_target_secure();
                    self.take_exception_target(Exception::HardFault, pc + 2, targets_secure)?;
                    return Ok(Next::Advance(self.regs.pc));
                }
                self.take_exception(Exception::SvCall, pc + 2)?;
                Ok(Next::Advance(self.regs.pc))
            }
            // A BKPT when debug is not enabled takes the same path as an
            // undefined instruction: it escalates to HardFault stacking the
            // BKPT's own address (so the handler can skip it), matching QEMU.
            I::Bkpt { .. } => self.hardfault_on_undefined(pc, 2),
            I::Hint { kind } => {
                // `WFI` sleeps until an exception pends (the `Machine` peeks
                // every cycle and wakes the core through `take_exception`).
                // `WFE` sleeps only when no event is pending: `SEV` and any
                // exception entry set the event latch, and `WFE` consumes it.
                match kind {
                    Hint::Wfi => self.sleeping = true,
                    Hint::Wfe => {
                        if self.event {
                            self.event = false;
                        } else {
                            self.sleeping = true;
                        }
                    }
                    Hint::Sev => {
                        self.event = true;
                        // SEV signals an event to this core, waking a WFE.
                        self.sleeping = false;
                    }
                    Hint::Nop | Hint::Yield => {}
                }
                Ok(Next::Advance(pc + 2))
            }
            I::Cps { disable, im } => {
                // The `i` bit (bit1) controls PRIMASK. On ARMv7-M the `f` bit
                // (bit0) is a FAULTMASK alias; ARMv6-M has neither `f` nor `a`.
                if im & 0b010 != 0 {
                    self.primask = disable;
                }
                if self.isa == Isa::Armv7M && im & 0b001 != 0 {
                    self.faultmask = disable;
                }
                Ok(Next::Advance(pc + 2))
            }
            I::It { cond, mask } => {
                // ITSTATE = firstcond:mask. `exec_it_step` never gates this
                // instruction itself because an IT inside an IT block is
                // UNPREDICTABLE; the fresh value simply replaces any stale
                // state.
                self.itstate = (cond << 4) | mask;
                Ok(Next::Advance(pc + 2))
            }
            I::Extend { rd, rm, kind } => {
                let v = self.regs.read(rm);
                // The signed forms sign-extend the selected low field; the
                // unsigned forms zero it. No flags are affected.
                let result = match kind {
                    ExtendKind::Sxth => ((v & 0xFFFF) as i16) as i32 as u32,
                    ExtendKind::Sxtb => ((v & 0xFF) as i8) as i32 as u32,
                    ExtendKind::Uxth => v & 0xFFFF,
                    ExtendKind::Uxtb => v & 0xFF,
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 2))
            }
            I::Reverse { rd, rm, kind } => {
                let v = self.regs.read(rm);
                // `REV` flips the whole word's byte order, `REV16` flips each
                // halfword, and `REVSH` flips the low halfword's bytes before
                // sign-extending it to 32 bits. No flags are affected.
                let result = match kind {
                    ReverseKind::Rev => v.swap_bytes(),
                    ReverseKind::Rev16 => ((v & 0x00FF_00FF) << 8) | ((v >> 8) & 0x00FF_00FF),
                    ReverseKind::Revsh => {
                        let half = ((v & 0xFF) << 8) | ((v >> 8) & 0xFF);
                        if half & 0x8000 != 0 {
                            half | 0xFFFF_0000
                        } else {
                            half
                        }
                    }
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 2))
            }
            I::Push { regs, include_lr } => {
                let count = regs.count_ones() + u32::from(include_lr);
                let new_sp = self.regs.sp.wrapping_sub(4 * count);
                // ARMv8-M: a PUSH that would move the current SP below its
                // stack limit raises a StackFault (HardFault on the baseline)
                // and the SP is not updated.
                if self.stack_limit_violation(new_sp) {
                    return self.stack_fault(pc);
                }
                let mut addr = new_sp;
                if !self.mpu_allows(addr, MpuAccess::Write) {
                    return self.mpu_fault(addr, pc, false);
                }
                for r in 0..8 {
                    if regs & (1 << r) != 0 {
                        let v = self.regs.read(r);
                        self.write_word_halt(addr, v)?;
                        addr = addr.wrapping_add(4);
                    }
                }
                if include_lr {
                    self.write_word_halt(addr, self.regs.lr)?;
                }
                self.regs.sp = new_sp;
                Ok(Next::Advance(pc + 2))
            }
            I::Pop { regs, load_pc } => {
                let mut addr = self.regs.sp;
                if !self.mpu_allows(addr, MpuAccess::Read) {
                    return self.mpu_fault(addr, pc, false);
                }
                for r in 0..8 {
                    if regs & (1 << r) != 0 {
                        let v = self.read_word_halt(addr)?;
                        self.regs.write(r, v);
                        addr = addr.wrapping_add(4);
                    }
                }
                let pc_word = if load_pc {
                    Some(self.read_word_halt(addr)?)
                } else {
                    None
                };
                let count = regs.count_ones() + u32::from(load_pc);
                self.regs.sp = self.regs.sp.wrapping_add(4 * count);
                if let Some(target) = pc_word {
                    return self.branch(target);
                }
                Ok(Next::Advance(pc + 2))
            }
            I::LdmStm { is_load, rn, regs } => {
                let base = self.regs.read(rn);
                // Thumb-1 LDM/STM require the base to be word-aligned; an
                // unaligned address is an alignment UsageFault (escalates to
                // HardFault here, per ARMv6-M folding), not a silent transfer.
                if base & 3 != 0 {
                    return self.data_fault(base, pc);
                }
                let mut addr = base;
                if is_load {
                    if !self.mpu_allows(base, MpuAccess::Read) {
                        return self.mpu_fault(base, pc, false);
                    }
                    for r in 0..8 {
                        if regs & (1 << r) != 0 {
                            let v = self.read_word_halt(addr)?;
                            self.regs.write(r, v);
                            addr = addr.wrapping_add(4);
                        }
                    }
                    // Writeback; if rn was loaded, the loaded value wins.
                    if regs & (1 << rn) == 0 {
                        self.regs
                            .write(rn, base.wrapping_add(4 * regs.count_ones()));
                    }
                } else {
                    // A store to SP (STMDB sp!...) is a PUSH.W; the ARMv8-M
                    // stack-limit check applies to the new SP value.
                    if rn == 13 {
                        let new_sp = base.wrapping_sub(4 * regs.count_ones());
                        if self.stack_limit_violation(new_sp) {
                            return self.stack_fault(pc);
                        }
                    }
                    if !self.mpu_allows(base, MpuAccess::Write) {
                        return self.mpu_fault(base, pc, false);
                    }
                    for r in 0..8 {
                        if regs & (1 << r) != 0 {
                            let v = self.regs.read(r);
                            self.write_word_halt(addr, v)?;
                            addr = addr.wrapping_add(4);
                        }
                    }
                    self.regs
                        .write(rn, base.wrapping_add(4 * regs.count_ones()));
                }
                Ok(Next::Advance(pc + 2))
            }
        }
    }

    /// Whether `inst` is a VFP/FP instruction. M-profile FP context mechanics
    /// (lazy preserve, `CONTROL.FPCA` set) run before these execute.
    fn inst_is_vfp(inst: &Inst32) -> bool {
        use Inst32 as I;
        matches!(
            inst,
            I::Vfp3 { .. }
                | I::Vfp2 { .. }
                | I::VfpCmp { .. }
                | I::VfpVrint { .. }
                | I::VfpCvtInt { .. }
                | I::VfpCvtFix { .. }
                | I::VfpMovImm { .. }
                | I::VfpMov { .. }
                | I::VfpMov64 { .. }
                | I::VfpMrsVmsr { .. }
                | I::VfpLdSt { .. }
                | I::VfpLdStD { .. }
                | I::VfpLdStMult { .. }
                | I::VfpLdStMultD { .. }
        )
    }

    /// M-profile FP context mechanics run before any FP instruction, matching
    /// the `PreserveFPState()`/`ExecuteFPCheck()` pseudocode and QEMU:
    ///
    /// 1. If `FPCCR.LSPACT` is set, an exception left an FP frame reserved at
    ///    `FPCAR`; store `S0`-`S15` and FPSCR there and clear `LSPACT`.
    /// 2. If `FPCCR.ASPEN` is set and `CONTROL.FPCA` is clear, create a new FP
    ///    context: set `FPCA` and load FPSCR from `FPDSCR`.
    fn fp_context(&mut self) -> Result<(), HaltCause> {
        let ctx = *self.fpu_ctx.lock().unwrap();
        if ctx.lspact {
            for i in 0..16u32 {
                let v = self.vfp.read_s(i as u8);
                self.write_word_halt(ctx.fpcar.wrapping_add(i * 4), v)?;
            }
            self.write_word_halt(ctx.fpcar.wrapping_add(0x40), self.vfp.fpscr)?;
            self.fpu_ctx.lock().unwrap().lspact = false;
        }
        if ctx.aspen && self.control & CONTROL_FPCA == 0 {
            self.control |= CONTROL_FPCA;
            self.vfp.fpscr = ctx.fpdscr;
        }
        Ok(())
    }

    /// Execute a 32-bit ARMv6-M instruction. Every 32-bit instruction is
    /// exactly 4 bytes, so a non-branching one simply advances `pc + 4`.
    fn exec32(&mut self, pc: u32, inst: Inst32) -> Result<Next, HaltCause> {
        use Inst32 as I;
        // M-profile FP context mechanics run before every FP instruction,
        // exactly as the PreserveFPState()/ExecuteFPCheck() pseudocode does.
        // CPACR CP10/CP11 gate access first: denied access is a NOCP
        // UsageFault taken before any FP-context activation.
        if Self::inst_is_vfp(&inst) {
            if self.mem.read_word(CPACR_ADDR).unwrap_or(CPACR_FULL) & CPACR_FULL != CPACR_FULL {
                return self.usage_fault(pc, UFSR_NOCP);
            }
            self.fp_context()?;
        }
        match inst {
            I::Bl { off } => {
                self.itstate = 0;
                self.regs.lr = (pc.wrapping_add(4)) | 1;
                Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
            }
            I::BlxImm { off } => {
                self.itstate = 0;
                self.regs.lr = (pc.wrapping_add(4)) | 1;
                // BLX targets are word-aligned, so bit 0 is always clear and
                // `check_branch` stops at the non-Thumb (ARM) target.
                Ok(Self::check_branch(
                    pc.wrapping_add(4).wrapping_add_signed(off),
                ))
            }
            I::BranchWide { off } => {
                self.itstate = 0;
                Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
            }
            I::BranchCondWide { cond, off } => {
                if self.cond_passes(cond) {
                    self.itstate = 0;
                    Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
                } else {
                    Ok(Next::Advance(pc + 4))
                }
            }
            I::Mrs { rd, sys } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented { what: "mrs to pc" });
                }
                let v = self.sysreg_read(sys)?;
                self.regs.write(rd, v);
                Ok(Next::Advance(pc + 4))
            }
            I::Msr { rn, sys } => {
                let v = self.regs.read(rn);
                // ARMv8-M stack-limit check precedes the write: an `MSR MSP`/
                // `MSR PSP` that would move the *current* stack pointer below
                // its limit is ignored and raises a StackFault (HardFault on
                // the baseline, which has no StackFault exception).
                let cur_sp_write = match sys {
                    SysReg::Msp if !self.sp_is_process() => Some(v & !3),
                    SysReg::Psp if self.sp_is_process() => Some(v & !3),
                    _ => None,
                };
                if let Some(new_sp) = cur_sp_write {
                    if self.stack_limit_violation(new_sp) {
                        return self.stack_fault(pc);
                    }
                }
                self.sysreg_write(sys, v)?;
                Ok(Next::Advance(pc + 4))
            }
            I::Barrier { .. } => Ok(Next::Advance(pc + 4)),
            I::Ldrex { rt, rn, size, off } => {
                let addr = self.regs.read(rn).wrapping_add(off);
                if !self.mpu_allows(addr, MpuAccess::Read) {
                    return self.mpu_fault(addr, pc, false);
                }
                let v = match size {
                    ExSize::Word => match self.mem.read_word(addr) {
                        Ok(v) => v,
                        Err(_) => return self.data_fault(addr, pc),
                    },
                    ExSize::Half => match self.mem.read_half(addr) {
                        Ok(hw) => u32::from(hw),
                        Err(_) => return self.data_fault(addr, pc),
                    },
                    ExSize::Byte => match self.mem.read(addr) {
                        Ok(b) => u32::from(b),
                        Err(_) => return self.data_fault(addr, pc),
                    },
                };
                self.regs.write(rt, v);
                self.exmon = Some(addr);
                Ok(Next::Advance(pc + 4))
            }
            I::Strex {
                rd,
                rt,
                rn,
                size,
                off,
            } => {
                let addr = self.regs.read(rn).wrapping_add(off);
                // The local monitor is cleared whether the store succeeds or
                // not; it only lets a store through while it matches.
                let ok = self.exmon == Some(addr);
                self.exmon = None;
                if ok {
                    if !self.mpu_allows(addr, MpuAccess::Write) {
                        return self.mpu_fault(addr, pc, false);
                    }
                    let result = match size {
                        ExSize::Word => self.mem.write_word(addr, self.regs.read(rt)),
                        ExSize::Half => self.mem.write_half(addr, self.regs.read(rt) as u16),
                        ExSize::Byte => self.mem.write(addr, self.regs.read(rt) as u8),
                    };
                    if result.is_err() {
                        return self.data_fault(addr, pc);
                    }
                    self.regs.write(rd, 0);
                } else {
                    self.regs.write(rd, 1);
                }
                Ok(Next::Advance(pc + 4))
            }
            I::Clrex => {
                self.exmon = None;
                Ok(Next::Advance(pc + 4))
            }
            I::Lda {
                rt,
                rn,
                size,
                exclusive,
            } => {
                // Load-acquire: the load result is observable; the acquire
                // ordering is a no-op on the in-order single-core model.
                let addr = self.regs.read(rn);
                if !self.mpu_allows(addr, MpuAccess::Read) {
                    return self.mpu_fault(addr, pc, false);
                }
                let v = match size {
                    ExSize::Word => match self.mem.read_word(addr) {
                        Ok(v) => v,
                        Err(_) => return self.data_fault(addr, pc),
                    },
                    ExSize::Half => match self.mem.read_half(addr) {
                        Ok(hw) => u32::from(hw),
                        Err(_) => return self.data_fault(addr, pc),
                    },
                    ExSize::Byte => match self.mem.read(addr) {
                        Ok(b) => u32::from(b),
                        Err(_) => return self.data_fault(addr, pc),
                    },
                };
                self.regs.write(rt, v);
                if exclusive {
                    self.exmon = Some(addr);
                }
                Ok(Next::Advance(pc + 4))
            }
            I::Stl {
                rd,
                rt,
                rn,
                size,
                exclusive,
            } => {
                // Store-release: the store result is observable; the release
                // ordering is a no-op on the in-order single-core model.
                let addr = self.regs.read(rn);
                if exclusive {
                    // STLEX: like STREX, the local monitor is cleared whether
                    // the store succeeds or not; it only lets a store through
                    // while it matches. `rd` is written only for STLEX (the
                    // plain STL encoding keeps the `0b1111` sentinel).
                    let ok = self.exmon == Some(addr);
                    self.exmon = None;
                    if ok {
                        if !self.mpu_allows(addr, MpuAccess::Write) {
                            return self.mpu_fault(addr, pc, false);
                        }
                        let result = match size {
                            ExSize::Word => self.mem.write_word(addr, self.regs.read(rt)),
                            ExSize::Half => self.mem.write_half(addr, self.regs.read(rt) as u16),
                            ExSize::Byte => self.mem.write(addr, self.regs.read(rt) as u8),
                        };
                        if result.is_err() {
                            return self.data_fault(addr, pc);
                        }
                        self.regs.write(rd, 0);
                    } else {
                        self.regs.write(rd, 1);
                    }
                } else {
                    if !self.mpu_allows(addr, MpuAccess::Write) {
                        return self.mpu_fault(addr, pc, false);
                    }
                    let result = match size {
                        ExSize::Word => self.mem.write_word(addr, self.regs.read(rt)),
                        ExSize::Half => self.mem.write_half(addr, self.regs.read(rt) as u16),
                        ExSize::Byte => self.mem.write(addr, self.regs.read(rt) as u8),
                    };
                    if result.is_err() {
                        return self.data_fault(addr, pc);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
            // ARMv8-M `SG`: clear ITSTATE in the Secure state, NOP in the
            // Non-secure state (ARMv8-M ARM; matches QEMU's trans_SG, which
            // also bypasses the IT state).
            I::Sg => {
                if self.secure {
                    self.itstate = 0;
                }
                Ok(Next::Advance(pc + 4))
            }
            I::Tt { variant, rd, rn } => {
                use TtVariant as V;
                // `TTA`/`TTAT` probe the opposite security state; the `TTTT`/
                // `TTAT` unprivileged-force only feeds the (disabled) MPU
                // lookup, so it does not change the permission bits here.
                let alt = matches!(variant, V::Tta | V::Ttat);
                if alt && !self.secure {
                    // TTA/TTAT read the opposite security state; from
                    // Non-secure they are UNPREDICTABLE and QEMU UNDEFs them.
                    return self.hardfault_on_undefined(pc, 4);
                }
                let addr = self.regs.read(rn);
                // The MPU gate and target privilege follow `arm_current_el`:
                // a privileged thread is EL 1, unprivileged thread EL 0.
                let current_el = u8::from(self.ipsr != 0 || self.control & 1 == 0);
                let targetsec = self.secure != alt;
                // MPU region info is only reported for a privileged current
                // state or when probing the opposite security state. The
                // PMSAv8 lookup then runs in the target security state's MPU
                // bank and target privilege (`TTTT`/`TTAT` force
                // unprivileged, otherwise the target state's `CONTROL.nPRIV`;
                // the target privilege feeds the `AP` matrix's user column).
                let (r, rw, mrvalid, mregion) = if current_el != 0 || alt {
                    let forceunpriv = matches!(variant, V::Tttt | V::Ttat);
                    let targetpriv = if forceunpriv {
                        false
                    } else {
                        let nprive = if targetsec == self.secure {
                            self.control
                        } else {
                            self.other_ss_control
                        };
                        self.ipsr != 0 || nprive & 1 == 0
                    };
                    let lk = self.mem.mpu_lookup_v8(addr, targetsec, targetpriv);
                    (
                        lk.prot & crate::mem::bus::PROT_READ != 0,
                        lk.prot & crate::mem::bus::PROT_WRITE != 0,
                        lk.region.is_some(),
                        lk.region.unwrap_or(0),
                    )
                } else {
                    (false, false, false, 0)
                };
                let s = if self.secure {
                    self.security_attribution(addr, targetsec)
                } else {
                    // A Non-secure read of the security state reports
                    // Non-secure with no SAU/IDAU attribution at all.
                    SAttrs {
                        ns: true,
                        nsc: false,
                        sregion: 0,
                        srvalid: false,
                        iregion: 0,
                        irvalid: false,
                    }
                };
                let (nsr, nsrw) = if self.secure {
                    (s.ns && r, s.ns && rw)
                } else {
                    (false, false)
                };
                let value = (u32::from(s.iregion) << 24)
                    | (u32::from(s.irvalid) << 23)
                    | (u32::from(!s.ns) << 22)
                    | (u32::from(nsrw) << 21)
                    | (u32::from(nsr) << 20)
                    | (u32::from(rw) << 19)
                    | (u32::from(r) << 18)
                    | (u32::from(s.srvalid) << 17)
                    | (u32::from(mrvalid) << 16)
                    | (u32::from(s.sregion) << 8)
                    | u32::from(mregion);
                self.regs.write(rd, value);
                Ok(Next::Advance(pc + 4))
            }
            I::DpReg {
                op,
                rd,
                rn,
                rm,
                shift,
                amount,
                set_flags,
            } => {
                let rm_v = self.regs.read(rm);
                let (op2, shift_carry) = match (shift, amount) {
                    (ShiftType::Ror, 0) => {
                        // RRX: rotate one bit right through the old carry.
                        let carry_out = rm_v & 1 == 1;
                        let rotated = (rm_v >> 1) | (u32::from(self.xpsr.c()) << 31);
                        (rotated, Some(carry_out))
                    }
                    (ShiftType::Ror, n) => {
                        let carry = (rm_v >> (n - 1)) & 1 == 1;
                        (rm_v.rotate_right(u32::from(n)), Some(carry))
                    }
                    (kind, n) => {
                        let kind = match kind {
                            ShiftType::Lsl => ShiftImm::Lsl,
                            ShiftType::Lsr => ShiftImm::Lsr,
                            ShiftType::Asr => ShiftImm::Asr,
                            ShiftType::Ror => unreachable!(),
                        };
                        shift_imm(kind, rm_v, u32::from(n))
                    }
                };
                self.exec_dp32(op, rd, rn, op2, set_flags, shift_carry)?;
                Ok(Next::Advance(pc + 4))
            }
            I::DpImm {
                op,
                rd,
                rn,
                imm12,
                expand,
                set_flags,
            } => {
                let (op2, expand_carry) = if expand {
                    let imm32 = thumb_expand_imm(imm12);
                    // A flag-setting logical op folds bit31 of a *rotated*
                    // modified immediate into C; an unrotated one (imm8 placed
                    // directly, i:imm3 == 0) leaves C alone.
                    let imm_c = imm12 & 0xC00 != 0;
                    (imm32, imm_c.then_some(imm32 >> 31 & 1 != 0))
                } else {
                    (u32::from(imm12), None)
                };
                self.exec_dp32(op, rd, rn, op2, set_flags, expand_carry)?;
                Ok(Next::Advance(pc + 4))
            }
            I::MovWide { rd, imm, high } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "movw/movt to pc",
                    });
                }
                let cur = self.regs.read(rd);
                // MOVW clears the upper half; MOVT preserves it and fills
                // bits 31:16.
                let new = if high {
                    (cur & 0xFFFF) | (u32::from(imm) << 16)
                } else {
                    u32::from(imm)
                };
                self.regs.write(rd, new);
                Ok(Next::Advance(pc + 4))
            }
            I::Mem {
                access,
                rt,
                rn,
                off,
                rm,
                shift,
                add,
                index,
                writeback,
                unprivileged: _,
            } => {
                // Rn == 15 is the literal form: the word-aligned `pc + 4`.
                let base = if rn == 15 {
                    (pc.wrapping_add(4)) & !3
                } else {
                    self.regs.read(rn)
                };
                // A register offset adds `Rv(Rm) << shift`; the immediate
                // forms add or subtract `off`.
                let offset_addr = if let Some(rm) = rm {
                    base.wrapping_add(self.regs.read(rm) << shift)
                } else if add {
                    base.wrapping_add(off)
                } else {
                    base.wrapping_sub(off)
                };
                // Post-indexed forms access the base and then step it; the
                // other two access the stepped address.
                let address = if index { offset_addr } else { base };
                let next = self.exec_ls_at(address, rt, access.into(), pc, 4)?;
                if writeback {
                    self.regs.write(rn, offset_addr);
                }
                Ok(next)
            }
            I::LdmStm {
                is_load,
                rn,
                regs,
                writeback,
                dec_before,
            } => {
                let base = self.regs.read(rn);
                let words = regs.count_ones();
                // DB stores the lowest-numbered register at `base - 4n`; IA
                // at `base`. Either way registers land at ascending addresses.
                let wb_addr = if dec_before {
                    base.wrapping_sub(4 * words)
                } else {
                    base.wrapping_add(4 * words)
                };
                let mut addr = if dec_before { wb_addr } else { base };
                // T32 LDM/STM require a word-aligned address; the
                // least-significant two bits of the base are ignored for the
                // transfer, but a non-zero value is an alignment fault.
                if addr & 3 != 0 {
                    return self.data_fault(addr, pc);
                }
                if !self.mpu_allows(
                    addr,
                    if is_load {
                        MpuAccess::Read
                    } else {
                        MpuAccess::Write
                    },
                ) {
                    return self.mpu_fault(addr, pc, false);
                }
                if is_load {
                    let mut pc_target = None;
                    for r in 0..16 {
                        if regs & (1u16 << r) != 0 {
                            let v = self.read_word_halt(addr)?;
                            if r == 15 {
                                pc_target = Some(v);
                            } else {
                                self.regs.write(r, v);
                            }
                            addr = addr.wrapping_add(4);
                        }
                    }
                    // A loaded rn wins over the write-back address.
                    if writeback && regs & (1u16 << rn) == 0 {
                        self.regs.write(rn, wb_addr);
                    }
                    if let Some(target) = pc_target {
                        return self.branch(target);
                    }
                } else {
                    for r in 0..16 {
                        if regs & (1u16 << r) != 0 {
                            let v = self.regs.read(r);
                            self.write_word_halt(addr, v)?;
                            addr = addr.wrapping_add(4);
                        }
                    }
                    if writeback {
                        self.regs.write(rn, wb_addr);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
            I::LdrdStrd {
                is_load,
                rt,
                rt2,
                rn,
                imm,
                add,
                index,
                writeback,
            } => {
                // `Rn == 15` is the `LDRD` literal form: the word-aligned
                // `pc + 4`. `index` selects the offset- (pre-) indexed
                // address over the post-indexed base.
                let base = if rn == 15 {
                    (pc.wrapping_add(4)) & !3
                } else {
                    self.regs.read(rn)
                };
                let offset_addr = if add {
                    base.wrapping_add(imm)
                } else {
                    base.wrapping_sub(imm)
                };
                let address = if index { offset_addr } else { base };
                // LDRD/STRD require a word-aligned address (the literal form's
                // `(pc+4) & !3` base is always aligned); an unaligned access
                // is an alignment fault.
                if address & 3 != 0 {
                    return self.data_fault(address, pc);
                }
                if !self.mpu_allows(
                    address,
                    if is_load {
                        MpuAccess::Read
                    } else {
                        MpuAccess::Write
                    },
                ) {
                    return self.mpu_fault(address, pc, false);
                }
                if is_load {
                    // The low word lands in `rt`, the high word in `rt2`;
                    // the decoder guarantees they are distinct registers.
                    let first = self.read_word_halt(address)?;
                    let second = self.read_word_halt(address.wrapping_add(4))?;
                    self.regs.write(rt, first);
                    self.regs.write(rt2, second);
                } else {
                    self.write_word_halt(address, self.regs.read(rt))?;
                    self.write_word_halt(address.wrapping_add(4), self.regs.read(rt2))?;
                }
                if writeback {
                    self.regs.write(rn, offset_addr);
                }
                Ok(Next::Advance(pc + 4))
            }
            I::TblBranch { rn, rm, half } => {
                // index = Mem[Rn + Rm (or Rn + Rm<<1) as byte/halfword];
                // target = (pc + 4) + 2 * index, a simple Thumb branch. A PC
                // base reads the address of the following instruction.
                let base = if rn == 15 {
                    pc.wrapping_add(4)
                } else {
                    self.regs.read(rn)
                };
                let index_reg = self.regs.read(rm);
                let table_addr = if half {
                    base.wrapping_add(index_reg << 1)
                } else {
                    base.wrapping_add(index_reg)
                };
                if !self.mpu_allows(table_addr, MpuAccess::Read) {
                    return self.mpu_fault(table_addr, pc, false);
                }
                let index: u32 =
                    if half {
                        u32::from(self.mem.read_half(table_addr).map_err(|_| {
                            HaltCause::BusFault {
                                address: table_addr,
                            }
                        })?)
                    } else {
                        u32::from(self.mem.read(table_addr).map_err(|_| HaltCause::BusFault {
                            address: table_addr,
                        })?)
                    };
                let target = pc.wrapping_add(4).wrapping_add(index << 1);
                self.itstate = 0;
                Ok(Next::Advance(target & !1))
            }
            #[allow(clippy::cast_sign_loss, clippy::cast_possible_wrap)]
            I::Div { signed, rd, rn, rm } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "divide writes to pc",
                    });
                }
                // A zero divisor yields zero unless `CCR.DIV_0_TRP` traps it
                // to a UsageFault, folded into HardFault exactly like an
                // undefined instruction. Signed `MIN / -1` saturates to MIN
                // without trapping, as the architecture requires.
                let num = self.regs.read(rn);
                let den = self.regs.read(rm);
                if den == 0 {
                    if self.mem.read_word(CCR_ADDR).unwrap_or(CCR_RESET) & CCR_DIV0_TRP != 0 {
                        return self.usage_fault(pc, UFSR_DIVBYZERO);
                    }
                    self.regs.write(rd, 0);
                    return Ok(Next::Advance(pc + 4));
                }
                let result = if signed {
                    i32::wrapping_div(num as i32, den as i32) as u32
                } else {
                    u32::wrapping_div(num, den)
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::Mul { op, rd, rn, rm, ra } => {
                match op {
                    MulOp::Mul | MulOp::Mla | MulOp::Mls => {
                        if rd == 15 {
                            return Err(HaltCause::NotImplemented {
                                what: "multiply writes to pc",
                            });
                        }
                        // The 32-bit multiply leaves the low word of the
                        // product; there is no flag setting on ARMv7-M.
                        let n = self.regs.read(rn);
                        let m = self.regs.read(rm);
                        let product = n.wrapping_mul(m);
                        let result = match op {
                            MulOp::Mul => product,
                            MulOp::Mla => product.wrapping_add(self.regs.read(ra)),
                            MulOp::Mls => self.regs.read(ra).wrapping_sub(product),
                            MulOp::Smull | MulOp::Umull | MulOp::Smlal | MulOp::Umlal => {
                                unreachable!()
                            }
                        };
                        self.regs.write(rd, result);
                    }
                    MulOp::Smull | MulOp::Umull | MulOp::Smlal | MulOp::Umlal => {
                        if rd == 15 || ra == 15 {
                            return Err(HaltCause::NotImplemented {
                                what: "long multiply writes to pc",
                            });
                        }
                        let n = self.regs.read(rn);
                        let m = self.regs.read(rm);
                        let product = if matches!(op, MulOp::Smull | MulOp::Smlal) {
                            i64::from(n as i32).wrapping_mul(i64::from(m as i32))
                        } else {
                            i64::from(n).wrapping_mul(i64::from(m))
                        };
                        // `ra` (bits15:12) is `RdLo` and `rd` (bits11:8) is
                        // `RdHi`, so the accumulator is `RdHi:RdLo`.
                        let accumulator = if matches!(op, MulOp::Smlal | MulOp::Umlal) {
                            (i64::from(self.regs.read(rd)) << 32) | i64::from(self.regs.read(ra))
                        } else {
                            0
                        };
                        let total = product.wrapping_add(accumulator);
                        let lo = total as u32;
                        let hi = (total >> 32) as u32;
                        self.regs.write(ra, lo);
                        self.regs.write(rd, hi);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::DspMul { op, rd, rn, rm, ra } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "DSP multiply writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                let result = match op {
                    DspMulOp::Smulbb | DspMulOp::Smulbt | DspMulOp::Smultb | DspMulOp::Smultt => {
                        let n_hi = matches!(op, DspMulOp::Smultb | DspMulOp::Smultt);
                        let m_hi = matches!(op, DspMulOp::Smulbt | DspMulOp::Smultt);
                        self.half_signed(n, n_hi)
                            .wrapping_mul(self.half_signed(m, m_hi))
                    }
                    DspMulOp::Smulwb | DspMulOp::Smulwt => {
                        let m_hi = op == DspMulOp::Smulwt;
                        let product = i64::from(n as i32) * i64::from(self.half_signed(m, m_hi));
                        (product >> 16) as i32
                    }
                    DspMulOp::Smmul | DspMulOp::Smmulr => {
                        let product = i64::from(n as i32) * i64::from(m as i32);
                        let rounded = if op == DspMulOp::Smmulr {
                            product.wrapping_add(0x8000_0000)
                        } else {
                            product
                        };
                        (rounded >> 32) as i32
                    }
                    DspMulOp::Usad8 | DspMulOp::Usada8 => {
                        let sum = self.sum_abs_diff(n, m);
                        if op == DspMulOp::Usada8 {
                            sum.wrapping_add(self.regs.read(ra) as i32)
                        } else {
                            sum
                        }
                    }
                };
                self.regs.write(rd, result as u32);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::DualMul { op, rd, rn, rm, ra } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "dual multiply writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                let m_swap = matches!(
                    op,
                    DualMulOp::Smladx | DualMulOp::Smlsdx | DualMulOp::Smlaldx | DualMulOp::Smlsldx
                );
                match op {
                    // Word forms write the single `Rd`.
                    DualMulOp::Smlabb
                    | DualMulOp::Smlabt
                    | DualMulOp::Smlatb
                    | DualMulOp::Smlatt
                    | DualMulOp::Smlawb
                    | DualMulOp::Smlawt
                    | DualMulOp::Smlad
                    | DualMulOp::Smladx
                    | DualMulOp::Smlsd
                    | DualMulOp::Smlsdx
                    | DualMulOp::Smmla
                    | DualMulOp::Smmlar
                    | DualMulOp::Smmls
                    | DualMulOp::Smmlsr => {
                        let result = match op {
                            // Halfword accumulate: `Ra + Rn[x] * Rm[y]`, with
                            // the wrapping add sticky-setting `Q`.
                            DualMulOp::Smlabb
                            | DualMulOp::Smlabt
                            | DualMulOp::Smlatb
                            | DualMulOp::Smlatt => {
                                let nt = matches!(op, DualMulOp::Smlatb | DualMulOp::Smlatt);
                                let mt = matches!(op, DualMulOp::Smlabt | DualMulOp::Smlatt);
                                let product = self
                                    .half_signed(n, nt)
                                    .wrapping_mul(self.half_signed(m, mt));
                                self.add_setq(product, self.regs.read(ra) as i32)
                            }
                            // Word accumulate of the halfword product's top
                            // bits; `add_setq` on the truncated sum.
                            DualMulOp::Smlawb | DualMulOp::Smlawt => {
                                let mt = op == DualMulOp::Smlawt;
                                let shifted = if mt { m & 0xFFFF_0000 } else { m << 16 };
                                let product = i64::from(n as i32) * i64::from(shifted as i32);
                                let high = (product >> 32) as i32;
                                self.add_setq(high, self.regs.read(ra) as i32)
                            }
                            // `SMLAD`/`SMLADX`, and `SMUAD`/`SMUSD` when
                            // `Ra == 15`. The accumulate add is 64-bit so the
                            // `Q` check sees the true overflow.
                            DualMulOp::Smlad
                            | DualMulOp::Smladx
                            | DualMulOp::Smlsd
                            | DualMulOp::Smlsdx => {
                                let sub = matches!(op, DualMulOp::Smlsd | DualMulOp::Smlsdx);
                                let (p_lo, p_hi) = self.smul_dual(n, m, m_swap);
                                if sub {
                                    // `p_lo - p_hi` cannot overflow.
                                    let diff = p_lo.wrapping_sub(p_hi);
                                    if ra == 15 {
                                        diff
                                    } else {
                                        self.add_setq(diff, self.regs.read(ra) as i32)
                                    }
                                } else if ra == 15 {
                                    self.add_setq(p_lo, p_hi)
                                } else {
                                    let sum = i64::from(p_lo)
                                        + i64::from(p_hi)
                                        + i64::from(self.regs.read(ra) as i32);
                                    let lo = sum as i32;
                                    if (sum >> 32) as i32 != lo >> 31 {
                                        self.xpsr.set_q(true);
                                    }
                                    lo
                                }
                            }
                            // Top word of the 32-by-32 product (no `Q`); the
                            // `SMMLS` subtract borrows into the high word.
                            DualMulOp::Smmla
                            | DualMulOp::Smmlar
                            | DualMulOp::Smmls
                            | DualMulOp::Smmlsr => {
                                let product = i64::from(n as i32) * i64::from(m as i32);
                                let sub = matches!(op, DualMulOp::Smmls | DualMulOp::Smmlsr);
                                let round = matches!(op, DualMulOp::Smmlar | DualMulOp::Smmlsr);
                                if sub {
                                    let acc = u64::from(self.regs.read(ra)) << 32;
                                    let result = acc.wrapping_sub(product as u64);
                                    let mut high = (result >> 32) as u32;
                                    if round {
                                        high = high.wrapping_add(result as u32 >> 31);
                                    }
                                    high as i32
                                } else {
                                    let high = (product >> 32) as u32;
                                    let mut result = high.wrapping_add(self.regs.read(ra));
                                    if round {
                                        result = result.wrapping_add(product as u32 >> 31);
                                    }
                                    result as i32
                                }
                            }
                            _ => unreachable!(),
                        };
                        self.regs.write(rd, result as u32);
                    }
                    // Long forms write `RdLo` (`ra`, bits15:12) and `RdHi`
                    // (`rd`, bits11:8). None of them touch `Q`.
                    DualMulOp::Smlalbb
                    | DualMulOp::Smlalbt
                    | DualMulOp::Smlaltb
                    | DualMulOp::Smlaltt
                    | DualMulOp::Smlald
                    | DualMulOp::Smlaldx
                    | DualMulOp::Smlsld
                    | DualMulOp::Smlsldx
                    | DualMulOp::Umaal => {
                        if ra == 15 {
                            return Err(HaltCause::NotImplemented {
                                what: "dual long multiply writes to pc",
                            });
                        }
                        let acc = (i64::from(self.regs.read(rd) as i32) << 32)
                            | i64::from(self.regs.read(ra));
                        let total = match op {
                            // 16-by-16 products sign-extended into the pair.
                            DualMulOp::Smlalbb
                            | DualMulOp::Smlalbt
                            | DualMulOp::Smlaltb
                            | DualMulOp::Smlaltt => {
                                let nt = matches!(op, DualMulOp::Smlaltb | DualMulOp::Smlaltt);
                                let mt = matches!(op, DualMulOp::Smlalbt | DualMulOp::Smlaltt);
                                let product = i64::from(self.half_signed(n, nt))
                                    * i64::from(self.half_signed(m, mt));
                                acc.wrapping_add(product)
                            }
                            // Dual products summed (or subtracted) and added
                            // to the accumulator.
                            DualMulOp::Smlald
                            | DualMulOp::Smlaldx
                            | DualMulOp::Smlsld
                            | DualMulOp::Smlsldx => {
                                let sub = matches!(op, DualMulOp::Smlsld | DualMulOp::Smlsldx);
                                let (p_lo, p_hi) = self.smul_dual(n, m, m_swap);
                                let pair = if sub {
                                    i64::from(p_lo) - i64::from(p_hi)
                                } else {
                                    i64::from(p_lo) + i64::from(p_hi)
                                };
                                acc.wrapping_add(pair)
                            }
                            // Unsigned product plus both halves of the pair.
                            DualMulOp::Umaal => {
                                let mut total = u64::from(n).wrapping_mul(u64::from(m));
                                total = total.wrapping_add(u64::from(self.regs.read(ra)));
                                total = total.wrapping_add(u64::from(self.regs.read(rd)));
                                let lo = total as u32;
                                let hi = (total >> 32) as u32;
                                self.regs.write(ra, lo);
                                self.regs.write(rd, hi);
                                return Ok(Next::Advance(pc + 4));
                            }
                            _ => unreachable!(),
                        };
                        let lo = total as u32;
                        let hi = (total >> 32) as u32;
                        self.regs.write(ra, lo);
                        self.regs.write(rd, hi);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
            I::Unary { op, rd, rm } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "unary data-processing writes to pc",
                    });
                }
                let v = self.regs.read(rm);
                let result = match op {
                    UnaryOp::Rev => v.swap_bytes(),
                    UnaryOp::Rev16 => ((v & 0x00FF_00FF) << 8) | ((v >> 8) & 0x00FF_00FF),
                    UnaryOp::Revsh => {
                        let half = ((v & 0xFF) << 8) | ((v >> 8) & 0xFF);
                        if half & 0x8000 != 0 {
                            half | 0xFFFF_0000
                        } else {
                            half
                        }
                    }
                    UnaryOp::Rbit => rbit(v),
                    UnaryOp::Clz => v.leading_zeros(),
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::Qadd { op, rd, rn, rm } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "saturating add writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                // The ARM syntax `Q{D}ADD/Q{D}SUB Rd, Rm, Rn` maps `Rm` to the
                // low-nibble register (`rm`) and `Rn` to the high-nibble one
                // (`rn`); the doubling always applies to `Rn`. Everything is
                // widened to `i64` so the sum of two 32-bit values cannot wrap
                // before the clamp sees it.
                let (result, sat) = match op {
                    QaddOp::Qadd => {
                        signed_sat(i64::from(m as i32).wrapping_add(i64::from(n as i32)), 32)
                    }
                    QaddOp::Qsub => {
                        signed_sat(i64::from(m as i32).wrapping_sub(i64::from(n as i32)), 32)
                    }
                    QaddOp::Qdadd => {
                        // `Rd = SAT(Rm + SAT(Rn*2))`.
                        let (d, double_sat) = signed_sat(i64::from(n as i32).wrapping_mul(2), 32);
                        let (r, add_sat) =
                            signed_sat(i64::from(m as i32).wrapping_add(i64::from(d)), 32);
                        (r, double_sat || add_sat)
                    }
                    QaddOp::Qdsub => {
                        // `Rd = SAT(Rm - SAT(Rn*2))`.
                        let (d, double_sat) = signed_sat(i64::from(n as i32).wrapping_mul(2), 32);
                        let (r, sub_sat) =
                            signed_sat(i64::from(m as i32).wrapping_sub(i64::from(d)), 32);
                        (r, double_sat || sub_sat)
                    }
                };
                if sat {
                    self.xpsr.set_q(true);
                }
                self.regs.write(rd, result as u32);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_possible_wrap
            )]
            I::Par {
                op,
                sat,
                rd,
                rn,
                rm,
            } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "parallel add/subtract writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                let (result, ge) = match op {
                    ParOp::Add8 | ParOp::Sub8 => {
                        let add = op == ParOp::Add8;
                        let mut res = 0u32;
                        let mut ge = 0u32;
                        for lane in 0..4 {
                            let (v, g) = par_lane(
                                n >> (lane * 8) & 0xFF,
                                m >> (lane * 8) & 0xFF,
                                add,
                                sat,
                                8,
                            );
                            res |= v << (lane * 8);
                            ge |= u32::from(g) << lane;
                        }
                        (res, ge)
                    }
                    ParOp::Add16 | ParOp::Sub16 => {
                        let add = op == ParOp::Add16;
                        let mut res = 0u32;
                        let mut ge = 0u32;
                        for lane in 0..2 {
                            let (v, g) = par_lane(
                                n >> (lane * 16) & 0xFFFF,
                                m >> (lane * 16) & 0xFFFF,
                                add,
                                sat,
                                16,
                            );
                            res |= v << (lane * 16);
                            ge |= (u32::from(g) * 0b11) << (lane * 2);
                        }
                        (res, ge)
                    }
                    ParOp::Asx | ParOp::Sax => {
                        // `ASX` subtracts the swapped high half from the low
                        // lane and adds the swapped low half to the high lane;
                        // `SAX` mirrors the operations.
                        let add_lo = op == ParOp::Sax;
                        let (lo_v, lo_ge) = par_lane(n & 0xFFFF, m >> 16, add_lo, sat, 16);
                        let (hi_v, hi_ge) = par_lane(n >> 16, m & 0xFFFF, !add_lo, sat, 16);
                        let ge = (u32::from(lo_ge) * 0b11) | ((u32::from(hi_ge) * 0b11) << 2);
                        ((lo_v & 0xFFFF) | (hi_v << 16), ge)
                    }
                };
                if matches!(sat, ParSat::S | ParSat::U) {
                    self.xpsr.set_ge(ge);
                }
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            I::Sel { rd, rn, rm } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "sel writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                let ge = self.xpsr.ge();
                let mut mask = 0u32;
                for lane in 0..4 {
                    if ge & (1 << lane) != 0 {
                        mask |= 0xFF << (lane * 8);
                    }
                }
                self.regs.write(rd, (n & mask) | (m & !mask));
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::ExtendAdd {
                op,
                rd,
                rn,
                rm,
                rot,
            } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "register extend writes to pc",
                    });
                }
                let m = self.regs.read(rm).rotate_right(u32::from(rot) * 8);
                let ext = match op {
                    ExtendAddOp::Sxtah => (m as u16 as i16) as u32,
                    ExtendAddOp::Uxtah => m & 0xFFFF,
                    ExtendAddOp::Sxtab16 => {
                        let lo = (m & 0xFF) as u8 as i8 as u32 & 0xFFFF;
                        let hi = ((m >> 16) & 0xFF) as u8 as i8 as u32 & 0xFFFF;
                        lo | (hi << 16)
                    }
                    ExtendAddOp::Uxtab16 => (m & 0xFF) | ((m >> 16) & 0xFF) << 16,
                    ExtendAddOp::Sxtab => (m & 0xFF) as u8 as i8 as u32,
                    ExtendAddOp::Uxtab => m & 0xFF,
                };
                // `rn == 15` is the two-operand alias: extend only, no addend.
                let result = if rn == 15 {
                    ext
                } else {
                    let n = self.regs.read(rn);
                    match op {
                        ExtendAddOp::Sxtab16 | ExtendAddOp::Uxtab16 => {
                            let lo = (n & 0xFFFF).wrapping_add(ext & 0xFFFF) & 0xFFFF;
                            let hi = (n >> 16).wrapping_add(ext >> 16) & 0xFFFF;
                            lo | (hi << 16)
                        }
                        _ => n.wrapping_add(ext),
                    }
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::Sat {
                op,
                rd,
                rn,
                sat_n,
                sh,
                shift,
            } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "saturate writes to pc",
                    });
                }
                let v = self.regs.read(rn);
                let (result, sat) = match op {
                    SatOp::Ssat | SatOp::Usat => {
                        // `sh` selects LSL/ASR; the decoder folds the
                        // zero-shift `ASR` alias into the 16-bit forms, so a
                        // nonzero shift is guaranteed here.
                        let shifted = if sh {
                            (v as i32) >> shift
                        } else {
                            v.wrapping_shl(u32::from(shift)) as i32
                        };
                        let signed = op == SatOp::Ssat;
                        let (r, s) = if signed {
                            signed_sat(i64::from(shifted), sat_n)
                        } else {
                            unsigned_sat(i64::from(shifted), sat_n)
                        };
                        (r as u32, s)
                    }
                    SatOp::Ssat16 | SatOp::Usat16 => {
                        let signed = op == SatOp::Ssat16;
                        let lane = |hw: u32| {
                            let s = i64::from(hw as u16 as i16);
                            if signed {
                                signed_sat(s, sat_n)
                            } else {
                                // `USAT16` saturates a *signed* halfword into
                                // the unsigned range `[0, 2^sat_n - 1]`, so a
                                // negative halfword clamps to zero.
                                unsigned_sat(s, sat_n)
                            }
                        };
                        let (lo, sat_lo) = lane(v);
                        let (hi, sat_hi) = lane(v >> 16);
                        (
                            (u32::from(hi as u16) << 16) | u32::from(lo as u16),
                            sat_lo || sat_hi,
                        )
                    }
                };
                if sat {
                    self.xpsr.set_q(true);
                }
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            I::Bitfield {
                op,
                rd,
                rn,
                lsb,
                field,
            } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "bitfield writes to pc",
                    });
                }
                match op {
                    BitfieldOp::Sbfx | BitfieldOp::Ubfx => {
                        // `width = field + 1` bits are shifted down from
                        // `lsb` and sign- or zero-extended. The decoder
                        // guarantees `lsb + field <= 31`.
                        let width = u32::from(field) + 1;
                        let mask = if width == 32 {
                            u32::MAX
                        } else {
                            (1u32 << width) - 1
                        };
                        let extracted = self.regs.read(rn) >> u32::from(lsb) & mask;
                        let result =
                            if op == BitfieldOp::Sbfx && extracted & (1u32 << (width - 1)) != 0 {
                                extracted | !mask
                            } else {
                                extracted
                            };
                        self.regs.write(rd, result);
                    }
                    BitfieldOp::Bfi | BitfieldOp::Bfc => {
                        // Deposit `Rn`'s low `width` bits into `Rd[field:lsb]`
                        // (or clear the window, for `BFC`). The decoder
                        // guarantees `field >= lsb`.
                        let width = u32::from(field) - u32::from(lsb) + 1;
                        let mask = if width == 32 {
                            u32::MAX
                        } else {
                            ((1u32 << width) - 1) << u32::from(lsb)
                        };
                        let base = self.regs.read(rd) & !mask;
                        let inserted = if op == BitfieldOp::Bfc {
                            0
                        } else {
                            self.regs.read(rn) << u32::from(lsb) & mask
                        };
                        self.regs.write(rd, base | inserted);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
            // ---- VFP (FPv4-SP) ----
            I::Vfp3 { op, vd, vn, vm } => {
                let mut st = self.vfp.float_status();
                let a = self.vfp.read_s(vn);
                let b = self.vfp.read_s(vm);
                let c = self.vfp.read_s(vd);
                // The VMLA-family forms are non-fused: the product rounds
                // before the add/subtract. The fused forms negate an input by
                // flipping its sign bit before the fused multiply-add, exactly
                // as hardware does for a NaN input too.
                let result = match op {
                    Vfp3Op::Vmla => {
                        let prod = fpu::mul(a, b, &mut st);
                        fpu::add(c, prod, &mut st)
                    }
                    Vfp3Op::Vmls => {
                        let prod = fpu::mul(a, b, &mut st);
                        fpu::add(c, prod ^ 0x8000_0000, &mut st)
                    }
                    Vfp3Op::Vnmls => {
                        let prod = fpu::mul(a, b, &mut st);
                        fpu::add(c ^ 0x8000_0000, prod, &mut st)
                    }
                    Vfp3Op::Vnmla => {
                        let prod = fpu::mul(a, b, &mut st);
                        fpu::add(c ^ 0x8000_0000, prod ^ 0x8000_0000, &mut st)
                    }
                    Vfp3Op::Vmul => fpu::mul(a, b, &mut st),
                    Vfp3Op::Vnmul => fpu::mul(a, b, &mut st) ^ 0x8000_0000,
                    Vfp3Op::Vadd => fpu::add(a, b, &mut st),
                    Vfp3Op::Vsub => fpu::sub(a, b, &mut st),
                    Vfp3Op::Vdiv => fpu::div(a, b, &mut st),
                    Vfp3Op::Vfma => fpu::muladd(a, b, c, &mut st),
                    Vfp3Op::Vfms => fpu::muladd(a ^ 0x8000_0000, b, c, &mut st),
                    // VFNMA negates the final result of a fused multiply-add,
                    // VFNMS the result of a fused multiply-subtract:
                    //   VFNMA = -(fd + fn*fm)   VFNMS = fn*fm - fd.
                    Vfp3Op::Vfnma => fpu::muladd(a ^ 0x8000_0000, b, c ^ 0x8000_0000, &mut st),
                    Vfp3Op::Vfnms => fpu::muladd(a, b, c ^ 0x8000_0000, &mut st),
                };
                self.vfp.write_s(vd, result);
                self.vfp.fold_flags(st.flags);
                Ok(Next::Advance(pc + 4))
            }
            I::Vfp2 { op, vd, vm } => {
                let m = self.vfp.read_s(vm);
                // VMOV/VABS/VNEG are plain register moves that neither
                // consume denormals nor raise exceptions.
                let result = match op {
                    Vfp2Op::Vmov => m,
                    Vfp2Op::Vabs => m & 0x7fff_ffff,
                    Vfp2Op::Vneg => m ^ 0x8000_0000,
                    Vfp2Op::Vsqrt => {
                        let mut st = self.vfp.float_status();
                        let r = fpu::sqrt(m, &mut st);
                        self.vfp.fold_flags(st.flags);
                        r
                    }
                };
                self.vfp.write_s(vd, result);
                Ok(Next::Advance(pc + 4))
            }
            I::VfpCmp { vd, vm, e, z } => {
                // VCMP compares `S{vd}` against `S{vm}` (or `+0.0` for the
                // `z` form) and writes FPSCR bits[31:28]. `e` selects VCMPE,
                // the signaling compare that raises Invalid on any NaN.
                let a = self.vfp.read_s(vd);
                let b = if z { 0x0000_0000 } else { self.vfp.read_s(vm) };
                let mut st = self.vfp.float_status();
                let rel = if e {
                    fpu::compare(a, b, &mut st)
                } else {
                    fpu::compare_signal(a, b, &mut st)
                };
                self.vfp.fold_flags(st.flags);
                let nzcv = match rel {
                    fpu::FloatRelation::Equal => 0b0110,
                    fpu::FloatRelation::Less => 0b1000,
                    fpu::FloatRelation::Greater => 0b0010,
                    fpu::FloatRelation::Unordered => 0b0011,
                };
                self.vfp.fpscr = (self.vfp.fpscr & !fpscr::NZCV) | (nzcv << 28);
                Ok(Next::Advance(pc + 4))
            }
            I::VfpVrint { op, vd, vm } => {
                let a = self.vfp.read_s(vm);
                let mut st = self.vfp.float_status();
                let result = match op {
                    VrintOp::Z => {
                        let prev = st.rounding_mode;
                        st.rounding_mode = fpu::RoundMode::ToZero;
                        let r = fpu::round_to_int(a, &mut st);
                        st.rounding_mode = prev;
                        r
                    }
                    VrintOp::R | VrintOp::X => fpu::round_to_int(a, &mut st),
                };
                self.vfp.write_s(vd, result);
                self.vfp.fold_flags(st.flags);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::VfpCvtInt {
                to_float,
                signed,
                rz,
                vd,
                vm,
            } => {
                let a = self.vfp.read_s(vm);
                let mut st = self.vfp.float_status();
                let result = if to_float {
                    if signed {
                        fpu::int32_to_float32(a as i32, &mut st)
                    } else {
                        fpu::uint32_to_float32(a, &mut st)
                    }
                } else {
                    // NaN converts to zero with Invalid Operation.
                    if is_f32_nan(a) {
                        st.flags |= fpu::FLAG_INVALID;
                        0
                    } else if signed {
                        if rz {
                            fpu::to_int32_round_to_zero(a, &mut st) as u32
                        } else {
                            fpu::to_int32(a, &mut st) as u32
                        }
                    } else if rz {
                        fpu::to_uint32_round_to_zero(a, &mut st)
                    } else {
                        fpu::to_uint32(a, &mut st)
                    }
                };
                self.vfp.write_s(vd, result);
                self.vfp.fold_flags(st.flags);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::VfpCvtFix { opc, frac, vd } => {
                // The convert is in place: source and destination are both
                // `S{vd}`. `opc` = bit18<<2 | bit16<<1 | bit7: bit18 = `op`
                // (1 = float-to-int), bit16 = `u` (0 = signed), bit7 = `sx`
                // (0 = 16-bit).
                let a = self.vfp.read_s(vd);
                let mut st = self.vfp.float_status();
                let to_int = opc & 4 != 0;
                let is_16 = opc & 1 == 0;
                let signed = opc & 2 == 0;
                let result = if to_int {
                    if is_f32_nan(a) {
                        st.flags |= fpu::FLAG_INVALID;
                        0
                    } else if signed {
                        if is_16 {
                            fpu::to_int16_scalbn(a, i32::from(frac), &mut st) as u32
                        } else {
                            fpu::to_int32_scalbn(a, i32::from(frac), &mut st) as u32
                        }
                    } else if is_16 {
                        u32::from(fpu::to_uint16_scalbn(a, i32::from(frac), &mut st))
                    } else {
                        fpu::to_uint32_scalbn(a, i32::from(frac), &mut st)
                    }
                } else {
                    // Fixed-point to float rounds to nearest even regardless
                    // of the FPSCR rounding mode.
                    let prev = st.rounding_mode;
                    st.rounding_mode = fpu::RoundMode::NearestEven;
                    let r = if signed {
                        if is_16 {
                            fpu::int16_to_float32_scalbn(a as u16 as i16, i32::from(frac), &mut st)
                        } else {
                            fpu::int32_to_float32_scalbn(a as i32, i32::from(frac), &mut st)
                        }
                    } else if is_16 {
                        fpu::uint16_to_float32_scalbn(a as u16, i32::from(frac), &mut st)
                    } else {
                        fpu::uint32_to_float32_scalbn(a, i32::from(frac), &mut st)
                    };
                    st.rounding_mode = prev;
                    r
                };
                self.vfp.write_s(vd, result);
                self.vfp.fold_flags(st.flags);
                Ok(Next::Advance(pc + 4))
            }
            I::VfpMovImm { vd, imm } => {
                self.vfp.write_s(vd, imm);
                Ok(Next::Advance(pc + 4))
            }
            I::VfpMov { to_gp, vd, rt } => {
                if to_gp {
                    if rt == 15 {
                        // VMOV APSR_nzcv, Sd: copy the FPSCR condition flags.
                        let nzcv = (self.vfp.fpscr >> 28) & 0xF;
                        self.xpsr.set_nzcv(
                            nzcv & 0b1000 != 0,
                            nzcv & 0b0100 != 0,
                            nzcv & 0b0010 != 0,
                            nzcv & 0b0001 != 0,
                        );
                    } else {
                        self.regs.write(rt, self.vfp.read_s(vd));
                    }
                } else {
                    self.vfp.write_s(vd, self.regs.read(rt));
                }
                Ok(Next::Advance(pc + 4))
            }
            I::VfpMov64 {
                to_gp,
                vm,
                rt,
                rt2,
                is_dp,
            } => {
                // The single-register form moves `S{vm}`/`S{vm+1}`; the
                // double-precision form moves `S{2*vm}`/`S{2*vm+1}`.
                let lo = if is_dp {
                    self.vfp.read_s(2 * vm)
                } else {
                    self.vfp.read_s(vm)
                };
                let hi = if is_dp {
                    self.vfp.read_s(2 * vm + 1)
                } else {
                    self.vfp.read_s(vm + 1)
                };
                if to_gp {
                    self.regs.write(rt, lo);
                    self.regs.write(rt2, hi);
                } else {
                    self.vfp
                        .write_s(if is_dp { 2 * vm } else { vm }, self.regs.read(rt));
                    self.vfp
                        .write_s(if is_dp { 2 * vm + 1 } else { vm + 1 }, self.regs.read(rt2));
                }
                Ok(Next::Advance(pc + 4))
            }
            I::VfpMrsVmsr { is_read, rt } => {
                if is_read {
                    if rt == 15 {
                        // VMRS APSR_nzcv, FPSCR: copy the condition flags only.
                        let nzcv = (self.vfp.fpscr >> 28) & 0xF;
                        self.xpsr.set_nzcv(
                            nzcv & 0b1000 != 0,
                            nzcv & 0b0100 != 0,
                            nzcv & 0b0010 != 0,
                            nzcv & 0b0001 != 0,
                        );
                    } else {
                        self.regs.write(rt, self.vfp.fpscr);
                    }
                } else {
                    if rt == 15 {
                        // VMSR FPSCR, R15 is undefined.
                        return self.hardfault_on_undefined(pc, 4);
                    }
                    // The condition flags are read-only to VMSR; the write
                    // updates the control and cumulative-exception bits.
                    let v = self.regs.read(rt);
                    self.vfp.fpscr = (self.vfp.fpscr & fpscr::NZCV)
                        | (v & (fpscr::CEXC | fpscr::RMODE | fpscr::FZ | fpscr::DN));
                }
                Ok(Next::Advance(pc + 4))
            }
            I::VfpLdSt {
                is_load,
                vd,
                rn,
                add,
                imm,
            } => {
                // VLDR/VSTR (single). `Rn == 15` is the literal form, whose
                // base is the word-aligned `pc + 4`.
                let base = if rn == 15 {
                    (pc.wrapping_add(4)) & !3
                } else {
                    self.regs.read(rn)
                };
                let addr = if add {
                    base.wrapping_add(imm)
                } else {
                    base.wrapping_sub(imm)
                };
                if addr & 3 != 0 {
                    return self.data_fault(addr, pc);
                }
                if !self.mpu_allows(
                    addr,
                    if is_load {
                        MpuAccess::Read
                    } else {
                        MpuAccess::Write
                    },
                ) {
                    return self.mpu_fault(addr, pc, false);
                }
                if is_load {
                    let v = self.read_word_halt(addr)?;
                    self.vfp.write_s(vd, v);
                } else {
                    self.write_word_halt(addr, self.vfp.read_s(vd))?;
                }
                Ok(Next::Advance(pc + 4))
            }
            I::VfpLdStD {
                is_load,
                vd,
                rn,
                add,
                imm,
            } => {
                let base = if rn == 15 {
                    (pc.wrapping_add(4)) & !3
                } else {
                    self.regs.read(rn)
                };
                let addr = if add {
                    base.wrapping_add(imm)
                } else {
                    base.wrapping_sub(imm)
                };
                if addr & 7 != 0 {
                    return self.data_fault(addr, pc);
                }
                if !self.mpu_allows(
                    addr,
                    if is_load {
                        MpuAccess::Read
                    } else {
                        MpuAccess::Write
                    },
                ) {
                    return self.mpu_fault(addr, pc, false);
                }
                if is_load {
                    let lo = self.read_word_halt(addr)?;
                    let hi = self.read_word_halt(addr.wrapping_add(4))?;
                    self.vfp.write_s(2 * vd, lo);
                    self.vfp.write_s(2 * vd + 1, hi);
                } else {
                    self.write_word_halt(addr, self.vfp.read_s(2 * vd))?;
                    self.write_word_halt(addr.wrapping_add(4), self.vfp.read_s(2 * vd + 1))?;
                }
                Ok(Next::Advance(pc + 4))
            }
            #[allow(clippy::cast_possible_truncation)]
            I::VfpLdStMult {
                is_load,
                vd,
                rn,
                n,
                writeback,
                dec_before,
            } => {
                // VLDM/VSTM with `Rn == PC` and write-back is UNPREDICTABLE.
                if writeback && rn == 15 {
                    return self.hardfault_on_undefined(pc, 4);
                }
                let base = if rn == 15 {
                    pc.wrapping_add(4)
                } else {
                    self.regs.read(rn)
                };
                let count = u32::from(n) * 4;
                let start = if dec_before {
                    base.wrapping_sub(count)
                } else {
                    base
                };
                if start & 3 != 0 {
                    return self.data_fault(start, pc);
                }
                if !self.mpu_allows(
                    start,
                    if is_load {
                        MpuAccess::Read
                    } else {
                        MpuAccess::Write
                    },
                ) {
                    return self.mpu_fault(start, pc, false);
                }
                let mut addr = start;
                for i in 0..u32::from(n) {
                    let s = vd.wrapping_add(i as u8);
                    if is_load {
                        let v = self.read_word_halt(addr)?;
                        self.vfp.write_s(s, v);
                    } else {
                        self.write_word_halt(addr, self.vfp.read_s(s))?;
                    }
                    addr = addr.wrapping_add(4);
                }
                if writeback {
                    let wb = if dec_before {
                        start
                    } else {
                        base.wrapping_add(count)
                    };
                    self.regs.write(rn, wb);
                }
                Ok(Next::Advance(pc + 4))
            }
            #[allow(clippy::cast_possible_truncation)]
            I::VfpLdStMultD {
                is_load,
                vd,
                rn,
                n,
                writeback,
                dec_before,
            } => {
                if writeback && rn == 15 {
                    return self.hardfault_on_undefined(pc, 4);
                }
                let base = if rn == 15 {
                    pc.wrapping_add(4)
                } else {
                    self.regs.read(rn)
                };
                let count = u32::from(n) * 8;
                let start = if dec_before {
                    base.wrapping_sub(count)
                } else {
                    base
                };
                if start & 7 != 0 {
                    return self.data_fault(start, pc);
                }
                if !self.mpu_allows(
                    start,
                    if is_load {
                        MpuAccess::Read
                    } else {
                        MpuAccess::Write
                    },
                ) {
                    return self.mpu_fault(start, pc, false);
                }
                let mut addr = start;
                for i in 0..u32::from(n) {
                    let d = vd.wrapping_add(i as u8);
                    if is_load {
                        let lo = self.read_word_halt(addr)?;
                        let hi = self.read_word_halt(addr.wrapping_add(4))?;
                        self.vfp.write_s(2 * d, lo);
                        self.vfp.write_s(2 * d + 1, hi);
                    } else {
                        self.write_word_halt(addr, self.vfp.read_s(2 * d))?;
                        self.write_word_halt(addr.wrapping_add(4), self.vfp.read_s(2 * d + 1))?;
                    }
                    addr = addr.wrapping_add(8);
                }
                if writeback {
                    let wb = if dec_before {
                        start
                    } else {
                        base.wrapping_add(count)
                    };
                    self.regs.write(rn, wb);
                }
                Ok(Next::Advance(pc + 4))
            }
        }
    }

    /// Execute the common ARMv7-M data-processing core over an already
    /// shifted (`op2`) operand. `shift_carry` is the carry the shifted
    /// register produced; only the flag-setting logical operations consume it
    /// (arithmetic computes `C`/`V` from the operation itself).
    fn exec_dp32(
        &mut self,
        op: DpOp,
        rd: u8,
        rn: u8,
        op2: u32,
        set_flags: bool,
        shift_carry: Option<bool>,
    ) -> Result<(), HaltCause> {
        let writes_rd = !matches!(op, DpOp::Tst | DpOp::Teq | DpOp::Cmp | DpOp::Cmn);
        if writes_rd && rd == 15 {
            return Err(HaltCause::NotImplemented {
                what: "data-processing writes to pc",
            });
        }
        let rn_v = self.regs.read(rn);
        let (res, c, v) = match op {
            DpOp::And | DpOp::Tst => {
                let c = self.shift_carry_or_current(shift_carry);
                (rn_v & op2, c, self.xpsr.v())
            }
            DpOp::Eor | DpOp::Teq => {
                let c = self.shift_carry_or_current(shift_carry);
                (rn_v ^ op2, c, self.xpsr.v())
            }
            DpOp::Bic => {
                let c = self.shift_carry_or_current(shift_carry);
                (rn_v & !op2, c, self.xpsr.v())
            }
            DpOp::Orr => {
                let c = self.shift_carry_or_current(shift_carry);
                (rn_v | op2, c, self.xpsr.v())
            }
            DpOp::Orn => {
                let c = self.shift_carry_or_current(shift_carry);
                (op2 | !rn_v, c, self.xpsr.v())
            }
            DpOp::Mov => {
                let c = self.shift_carry_or_current(shift_carry);
                (op2, c, self.xpsr.v())
            }
            DpOp::Mvn => {
                let c = self.shift_carry_or_current(shift_carry);
                (!op2, c, self.xpsr.v())
            }
            DpOp::Adc => add_flags(rn_v, op2, self.xpsr.c()),
            DpOp::Add | DpOp::Cmn => add_flags(rn_v, op2, false),
            DpOp::Sub | DpOp::Cmp => sub_flags(rn_v, op2, false),
            DpOp::Sbc => sub_flags(rn_v, op2, !self.xpsr.c()),
            DpOp::Rsb => sub_flags(op2, rn_v, false),
        };
        if set_flags {
            self.set_nzcv(res, c, v);
        }
        if writes_rd {
            self.regs.write(rd, res);
        }
        Ok(())
    }

    /// The carry an ARMv7-M data-processing logical op produces, when it sets
    /// flags: the shifted-register carry if there was one, else the old `C`.
    fn shift_carry_or_current(&self, shift_carry: Option<bool>) -> bool {
        shift_carry.unwrap_or_else(|| self.xpsr.c())
    }

    /// Execute a 16-bit ALU (data-processing) instruction. `set_flags` is
    /// false only inside an IT block, where the S-forms stop updating the
    /// flags (CMP/CMN/TST always update them).
    fn exec_alu(&mut self, rd: u8, rm: u8, op: AluOp, set_flags: bool) {
        let a = self.regs.read(rd);
        let b = self.regs.read(rm);
        match op {
            AluOp::And => {
                let res = a & b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
            }
            AluOp::Eor => {
                let res = a ^ b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
            }
            AluOp::Lsl => {
                let (res, c) = shift_reg(Shift::Lsl, a, b & 0xFF);
                self.regs.write(rd, res);
                if set_flags {
                    self.write_shift_flags(res, c);
                }
            }
            AluOp::Lsr => {
                let (res, c) = shift_reg(Shift::Lsr, a, b & 0xFF);
                self.regs.write(rd, res);
                if set_flags {
                    self.write_shift_flags(res, c);
                }
            }
            AluOp::Asr => {
                let (res, c) = shift_reg(Shift::Asr, a, b & 0xFF);
                self.regs.write(rd, res);
                if set_flags {
                    self.write_shift_flags(res, c);
                }
            }
            AluOp::Adc => {
                let (res, c, v) = add_flags(a, b, self.xpsr.c());
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nzcv(res, c, v);
                }
            }
            AluOp::Sbc => {
                let (res, c, v) = sub_flags(a, b, !self.xpsr.c());
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nzcv(res, c, v);
                }
            }
            AluOp::Ror => {
                let (res, c) = shift_reg(Shift::Ror, a, b & 0xFF);
                self.regs.write(rd, res);
                if set_flags {
                    self.write_shift_flags(res, c);
                }
            }
            AluOp::Tst => {
                let res = a & b;
                self.set_nz(res, false);
            }
            AluOp::Neg => {
                let (res, c, v) = sub_flags(0, b, false);
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nzcv(res, c, v);
                }
            }
            AluOp::Cmp => {
                let (res, c, v) = sub_flags(a, b, false);
                self.set_nzcv(res, c, v);
            }
            AluOp::Cmn => {
                let (res, c, v) = add_flags(a, b, false);
                self.set_nzcv(res, c, v);
            }
            AluOp::Orr => {
                let res = a | b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
            }
            AluOp::Mul => {
                let res = a.wrapping_mul(b);
                self.regs.write(rd, res);
                if set_flags {
                    // MULS on ARMv7-M sets N/Z but leaves C alone.
                    self.set_nz(res, false);
                }
            }
            AluOp::Bic => {
                let res = a & !b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
            }
            AluOp::Mvn => {
                let res = !b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
            }
        }
    }

    /// A load or store with a fully-computed `addr`.
    fn exec_ls(&mut self, addr: u32, rt: u8, access: LsAccess, pc: u32) -> Result<Next, HaltCause> {
        self.exec_ls_at(addr, rt, access, pc, 2)
    }

    /// `exec_ls`, but the fall-through (and the `rt == 15` branch's `LR`
    /// base) advance by `step` instead of 2, for the 32-bit load/stores.
    fn exec_ls_at(
        &mut self,
        addr: u32,
        rt: u8,
        access: LsAccess,
        pc: u32,
        step: u32,
    ) -> Result<Next, HaltCause> {
        let is_load = matches!(
            access,
            LsAccess::Word { load: true }
                | LsAccess::Half { load: true }
                | LsAccess::Byte { load: true }
                | LsAccess::SextByte
                | LsAccess::SextHalf
        );
        if !self.mpu_allows(
            addr,
            if is_load {
                MpuAccess::Read
            } else {
                MpuAccess::Write
            },
        ) {
            return self.mpu_fault(addr, pc, false);
        }
        if is_load {
            let v = match access {
                LsAccess::Word { load: _ } => match self.mem.read_word(addr) {
                    Ok(v) => v,
                    Err(_) => return self.data_fault(addr, pc),
                },
                LsAccess::Half { load: true } => match self.mem.read_half(addr) {
                    Ok(hw) => u32::from(hw),
                    Err(_) => return self.data_fault(addr, pc),
                },
                LsAccess::Byte { load: true } => match self.mem.read(addr) {
                    Ok(b) => u32::from(b),
                    Err(_) => return self.data_fault(addr, pc),
                },
                LsAccess::SextByte => match self.mem.read(addr) {
                    Ok(b) => (b as i8) as u32,
                    Err(_) => return self.data_fault(addr, pc),
                },
                LsAccess::SextHalf => match self.mem.read_half(addr) {
                    Ok(hw) => (hw as i16) as u32,
                    Err(_) => return self.data_fault(addr, pc),
                },
                _ => unreachable!(),
            };
            self.write_rt_at(rt, v, addr, pc, step)
        } else {
            match access {
                LsAccess::Word { load: false } => {
                    let v = self.regs.read(rt);
                    match self.mem.write_word(addr, v) {
                        Ok(()) => {}
                        Err(_) => return self.data_fault(addr, pc),
                    }
                }
                LsAccess::Half { load: false } => {
                    let v = self.regs.read(rt) as u16;
                    match self.mem.write_half(addr, v) {
                        Ok(()) => {}
                        Err(_) => return self.data_fault(addr, pc),
                    }
                }
                LsAccess::Byte { load: false } => {
                    let v = self.regs.read(rt) as u8;
                    match self.mem.write(addr, v) {
                        Ok(()) => {}
                        Err(_) => return self.data_fault(addr, pc),
                    }
                }
                _ => unreachable!(),
            }
            Ok(Next::Advance(pc + step))
        }
    }

    /// Write a loaded value to `rt`. If `rt == 15` the load is a branch
    /// (Thumb bit checked, EXC_RETURN honored).
    fn write_rt(&mut self, rt: u8, v: u32, addr: u32, pc: u32) -> Result<Next, HaltCause> {
        self.write_rt_at(rt, v, addr, pc, 2)
    }

    /// `write_rt` with a 32-bit fall-through step.
    fn write_rt_at(
        &mut self,
        rt: u8,
        v: u32,
        addr: u32,
        pc: u32,
        step: u32,
    ) -> Result<Next, HaltCause> {
        let _ = addr;
        if rt == 15 {
            return self.branch(v);
        }
        self.regs.write(rt, v);
        Ok(Next::Advance(pc + step))
    }

    /// Turn a Thumb target into a `Next`, or a `NonThumbTarget` stop.
    fn check_branch(target: u32) -> Next {
        if target & 1 == 0 {
            return Next::Stop(HaltCause::NonThumbTarget);
        }
        Next::Advance(target & !1)
    }

    /// Take a branch to `target`, unless it is an `EXC_RETURN` while a
    /// handler is active, in which case perform the exception return.
    ///
    /// Any taken branch leaves the current IT block, so ITSTATE is cleared
    /// (the exception-return path overwrites it from the stacked xPSR).
    ///
    /// On ARMv8-M the Security Extension turns a branch to a `FNC_RETURN`
    /// (`>= 0xFEFFFFFE`) or `EXC_RETURN` (`>= 0xFF000000`) magic value into a
    /// secure function return or exception return (QEMU's
    /// `gen_bx_excret`/`gen_bx_excret_final_code` routing).
    fn branch(&mut self, target: u32) -> Result<Next, HaltCause> {
        self.itstate = 0;
        if self.isa == Isa::Armv8MBaseline && target >= FNC_RETURN_MIN_MAGIC {
            return self.return_via_magic(target);
        }
        if self.ipsr != 0 && is_exc_return(target) {
            return self.exception_return(target);
        }
        Ok(Self::check_branch(target))
    }

    /// ARMv8-M magic-branch dispatch: `target` is at or above
    /// `FNC_RETURN_MIN_MAGIC`. Values below `EXC_RETURN_MIN_MAGIC` are secure
    /// function returns; the rest are exception returns (QEMU routes both to
    /// `EXCP_EXCEPTION_EXIT` and splits them on `EXC_RETURN_MIN_MAGIC`).
    fn return_via_magic(&mut self, target: u32) -> Result<Next, HaltCause> {
        if target < EXC_RETURN_MIN_MAGIC {
            return self.function_return(target);
        }
        if self.ipsr == 0 {
            // Exception-return magic in Thread mode has no magic behaviour
            // (QEMU's `do_v7m_exception_exit` returns early); the branch is a
            // plain `BX` to the even target, which in VioARM stops as a
            // Non-Thumb target.
            return Ok(Self::check_branch(target));
        }
        self.exception_return_v8m(target)
    }

    /// ARMv8-M secure function return (`FNC_RETURN` magic reached by
    /// `BXNS`/`BLXNS`): pop `{newpc, newpsr}` from the *Secure* stack bank
    /// (while running Non-secure the Secure bank lives in the `other_ss_*`
    /// fields), check the IPSR consistency, then switch to Secure and
    /// continue at `newpc`. Mirrors QEMU's `do_v7m_function_return`; the
    /// SFPA bits are FPU-only and skipped.
    fn function_return(&mut self, _target: u32) -> Result<Next, HaltCause> {
        let threadmode = self.ipsr == 0;
        let spsel = self.other_ss_control & 0x2 != 0;
        let frameptr = if threadmode && spsel {
            self.other_ss_psp
        } else {
            self.other_ss_msp
        };
        let newpc = match self.read_word_halt(frameptr) {
            Ok(v) => v,
            Err(e) => return Err(e),
        };
        let newpsr = match self.read_word_halt(frameptr.wrapping_add(4)) {
            Ok(v) => v,
            Err(e) => return Err(e),
        };
        // Consistency check on the new IPSR: returning from thread with a
        // nonzero stacked IPSR (or from the BLXNS dummy handler with a zero
        // one) is an INVPC UsageFault (QEMU's check in
        // `do_v7m_function_return`).
        let newpsr_exc = (newpsr & 0xFF) as u8;
        if !((self.ipsr == 0 && newpsr_exc == 0) || (self.ipsr == 1 && newpsr_exc != 0)) {
            return self.usage_fault(self.regs.pc, UFSR_INVPC);
        }
        if threadmode && spsel {
            self.other_ss_psp = frameptr.wrapping_add(8);
        } else {
            self.other_ss_msp = frameptr.wrapping_add(8);
        }
        self.switch_security_state(true);
        self.ipsr = newpsr_exc;
        self.itstate = 0;
        Ok(Self::check_branch(newpc))
    }

    /// ARMv8-M exception return (any `EXC_RETURN` magic from a Handler-mode
    /// branch), mirroring QEMU's `do_v7m_exception_exit` for the baseline
    /// (no FP, no tailchaining): validate, deposit the return `SPSEL` into
    /// the target state, switch security state, unstack the eight-word frame
    /// from the target stack, then restore xPSR/IPSR/ITSTATE.
    fn exception_return_v8m(&mut self, value: u32) -> Result<Next, HaltCause> {
        let return_to_handler = value & EXC_RETURN_MODE == 0;
        let return_to_sp_process = value & EXC_RETURN_SPSEL != 0;
        let return_to_secure = value & EXC_RETURN_S != 0;
        // The `ES`/`DCRS` validity check (QEMU's R_SMFL): returning to
        // Non-secure state with `ES` set, or without `DCRS`, is a SecureFault
        // (stacking/integrity failure on the frame the return was about to
        // unstack). The ES bit is then treated as 0 for everything else
        // (R_HXSR). This precedes the `RES0` check: a return that violates
        // both takes the SecureFault, not the UsageFault.
        let mut exc_secure = value & EXC_RETURN_ES;
        let sfault = !self.secure
            && (value & EXC_RETURN_ES != 0 || value & EXC_RETURN_DCRS == 0);
        if sfault {
            exc_secure = 0;
        }
        if !sfault && value & EXC_RETURN_RES0 != 0 {
            return self.usage_fault(self.regs.pc, UFSR_INVPC);
        }
        // Depositing `SPSEL` for the exception being deactivated (selected by
        // `ES`, not by the return target `S` — QEMU writes the bank
        // `control[exc_secure]`) cannot move the active SP while still in
        // Handler mode.
        self.write_control_spsel_for_secstate(exc_secure != 0, return_to_sp_process);
        if sfault {
            // The aborted return cannot be completed: take a SecureFault
            // directly, tailchaining off the frame the failed return was
            // going to pop (no unstack/stack cycle).
            self.mem.record_secure_fault(SFSR_INVER);
            return self.take_exception_tailchain(Exception::SecureFault, value);
        }
        self.switch_security_state(return_to_secure);
        // The frame lives on the target state's stack, selected by its
        // `CONTROL.SPSEL` in Thread mode.
        let mut frameptr = if !return_to_handler && self.control & EXC_RETURN_SPSEL != 0 {
            self.regs.psp
        } else {
            self.regs.msp
        };
        // A return to Secure whose frame carries the callee-saves block
        // (`ES` clear: the popped exception targeted Non-secure from a Secure
        // background; or `DCRS` clear: the block was left by the tailchain
        // that cleared it) pops the block first. The integrity signature must
        // match; a mismatch is `SFSR.INVIS` and the `SecureFault` tailchains
        // off the frame (QEMU `do_v7m_exception_exit`).
        if return_to_secure && (value & EXC_RETURN_ES == 0 || value & EXC_RETURN_DCRS == 0) {
            let sig = self.read_word_halt(frameptr)?;
            if sig != CALLEE_SAVES_SIG {
                self.mem.record_secure_fault(SFSR_INVIS);
                return self.take_exception_tailchain(Exception::SecureFault, value);
            }
            let mut coff = 8u32;
            for r in 4u8..=11 {
                let v = self.read_word_halt(frameptr.wrapping_add(coff))?;
                self.regs.write(r, v);
                coff = coff.wrapping_add(4);
            }
            frameptr = frameptr.wrapping_add(CALLEE_SAVES_BYTES);
        }
        let mut frame = [0u32; 8];
        for (i, slot) in frame.iter_mut().enumerate() {
            let addr = frameptr.wrapping_add((i as u32) * 4);
            *slot = match self.read_word_halt(addr) {
                Ok(v) => v,
                Err(e) => return Err(e),
            };
        }
        let xpsr = frame[7];
        // The stacked xPSR exception field must agree with `EXC_RETURN.MODE`
        // (QEMU's v8M consistency check; a mismatch is an INVPC UsageFault).
        if return_to_handler != (xpsr & 0xFF != 0) {
            return self.usage_fault(self.regs.pc, UFSR_INVPC);
        }
        // Restore the caller-saved registers and LR from the basic frame.
        self.regs.write(0, frame[0]);
        self.regs.write(1, frame[1]);
        self.regs.write(2, frame[2]);
        self.regs.write(3, frame[3]);
        self.regs.write(12, frame[4]);
        self.regs.lr = frame[5];
        // Consume the frame, undoing any entry-time stack alignment.
        let new_sp = frameptr.wrapping_add(0x20);
        let new_sp = if xpsr & XPSR_ALIGN != 0 { new_sp | 4 } else { new_sp };
        if !return_to_handler && self.control & EXC_RETURN_SPSEL != 0 {
            self.regs.psp = new_sp;
            self.regs.sp = new_sp;
        } else {
            self.regs.msp = new_sp;
            self.regs.sp = new_sp;
        }
        self.xpsr.set(xpsr & (APSR_QFLAGS | XPSR_T));
        self.ipsr = (xpsr & 0xFF) as u8;
        self.itstate = ((xpsr >> 8) & 0xFC) as u8 | ((xpsr >> 25) & 0x03) as u8;
        // SCR.SLEEPONEXIT: returning to Thread mode re-enters sleep.
        if !return_to_handler
            && self.mem.read_word(SCR_ADDR).unwrap_or(0) & SCR_SLEEPONEXIT != 0
        {
            self.sleeping = true;
        }
        self.just_returned = true;
        let pc = frame[6] & !1;
        self.regs.pc = pc;
        Ok(Next::Advance(pc))
    }

    /// ARMv8-M `BXNS Rm`: branch to `Rm`, switching security state on the
    /// LSB of the target (even -> Non-secure, odd -> Secure). A target that
    /// is a `FNC_RETURN`/`EXC_RETURN` magic value performs the function or
    /// exception return instead. Undefined from the Non-secure state or on
    /// any pre-ARMv8-M ISA (QEMU's `trans_BXNS` gates on `v8m_secure`).
    fn exec_bxns(&mut self, target: u32) -> Result<Next, HaltCause> {
        if self.isa != Isa::Armv8MBaseline || !self.secure {
            return self.hardfault_on_undefined(self.regs.pc, 2);
        }
        self.itstate = 0;
        if target >= FNC_RETURN_MIN_MAGIC {
            return self.return_via_magic(target);
        }
        self.switch_security_state(target & 1 != 0);
        Ok(Next::Advance(target & !1))
    }

    /// ARMv8-M `BLXNS Rm`: link to `Rm`. An odd target (Secure) is a plain
    /// `BLX` with `LR = next instruction`; an even target (Non-secure) is a
    /// secure function call that pushes `{next instruction, saved IPSR}` onto
    /// the current (Secure) stack, sets `LR = FNC_RETURN` and branches to the
    /// Non-secure target. Mirrors QEMU's `v7m_blxns`. Undefined from the
    /// Non-secure state or on any pre-ARMv8-M ISA.
    fn exec_blxns(&mut self, pc: u32, target: u32) -> Result<Next, HaltCause> {
        if self.isa != Isa::Armv8MBaseline || !self.secure {
            return self.hardfault_on_undefined(self.regs.pc, 2);
        }
        self.itstate = 0;
        let next = pc.wrapping_add(2) | 1;
        if target & 1 != 0 {
            // Target is Secure: an ordinary BLX; the LSB is a security
            // selector, not a Thumb indicator.
            self.regs.lr = next;
            return Ok(Next::Advance(target & !1));
        }
        // Target is Non-secure: push a return frame (return address + the
        // interrupted IPSR) onto the Secure stack and set `LR` to the
        // `FNC_RETURN` magic, which `BXNS LR` (or any BX to `0xFEFFFFFF`)
        // pops to come back here.
        let sp = self.regs.sp.wrapping_sub(8);
        self.write_word_halt(sp, next)?;
        self.write_word_halt(sp.wrapping_add(4), u32::from(self.ipsr))?;
        self.regs.sp = sp;
        self.regs.lr = FNC_RETURN;
        if self.ipsr != 0 {
            // In a handler: the `FNC_RETURN` consistency check requires a
            // nonzero stacked IPSR (QEMU sets `exception = 1`).
            self.ipsr = 1;
        }
        self.switch_security_state(false);
        Ok(Next::Advance(target & !1))
    }

    /// Take a UsageFault for an undecodable instruction at `pc`. With
    /// `SHCSR.USGFAULTENA` set and a UsageFault that outranks the running
    /// context, a distinct UsageFault (vector 6) is taken; otherwise the fault
    /// escalates to HardFault. Either way the stacked return address is the
    /// faulting instruction itself, so the handler can repair or skip the bad
    /// opcode (it must advance `pc` itself). Records a UsageFault
    /// `UNDEFINSTR` and, on escalation, the `FORCED` bit.
    fn hardfault_on_undefined(&mut self, pc: u32, _size: u32) -> Result<Next, HaltCause> {
        self.usage_fault(pc, UFSR_UNDEFINSTR)
    }

/// Whether an escalated HardFault targets `Secure` state, mirroring QEMU's
    /// `exc_targets_secure` on the derived-exception escalation path: Secure
    /// when the fault already targeted Secure, or when `AIRCR.BFHFNMINS` is
    /// clear - the reset state, which disables `Non-secure HardFault` entirely
    /// so every escalated fault lands in the Secure bank. On pre-v8-M the
    /// single `HardFault` is never the Secure version.
    fn hardfault_target_secure(&mut self) -> bool {
        self.isa == Isa::Armv8MBaseline && (self.secure || !self.bfhfnmins())
    }

    /// `AIRCR.BFHFNMINS`, read through the memory window (the core has no
    /// direct SCB handle; the SCB echoes the bit in the `AIRCR` read).
    fn bfhfnmins(&mut self) -> bool {
        self.mem.read_word(AIRCR_ADDR).unwrap_or(0) & 0x2 != 0
    }

    /// Take a UsageFault (recorded as `ufsr_bit` in `CFSR`): a distinct
    /// exception when `SHCSR.USGFAULTENA` is set and its priority outranks the
    /// running context, otherwise escalated to HardFault with `HFSR.FORCED`.
    /// The stacked return address is the faulting instruction. With no SCB on
    /// the bus the architectural reset state applies (disabled), so faults
    /// fold to HardFault exactly as before.
    ///
    /// ARMv6-M (Cortex-M0/M0+) has no UsageFault exception at all: the fold
    /// to HardFault is unconditional there.
    fn usage_fault(&mut self, pc: u32, ufsr_bit: u32) -> Result<Next, HaltCause> {
        let enabled = self.isa == Isa::Armv7M
            && self.mem.read_word(SHCSR_ADDR).unwrap_or(0) & SHCSR_USGFAULTENA != 0;
        // FAULTMASK raises the current priority to -1, below which nothing can
        // preempt, so a distinct UsageFault cannot be taken under it.
        let current = if self.faultmask {
            0
        } else {
            self.current_priority()
        };
        if enabled && self.priority_byte(6) < current {
            self.mem.record_fault(ufsr_bit, 0);
            self.take_exception(Exception::UsageFault, pc)?;
        } else {
            self.mem.record_fault(ufsr_bit, HFSR_FORCED);
            let targets_secure = self.hardfault_target_secure();
            self.take_exception_target(Exception::HardFault, pc, targets_secure)?;
        }
        Ok(Next::Advance(self.regs.pc))
    }

    /// The priority of the currently running context: `u8::MAX` in Thread
    /// mode, HardFault at -1 (encoded as the highest priority), and otherwise
    /// the running exception's configurable priority byte.
    fn current_priority(&mut self) -> u8 {
        match self.ipsr {
            0 => u8::MAX,
            // SecureFault (v8-M) has a fixed priority above any configurable
            // exception; encoded like HardFault at the top of the range.
            3 | 7 => 0,
            6 | 11 | 14 | 15 => self.priority_byte(self.ipsr),
            n => self
                .mem
                .read(NVIC_IPR.wrapping_add(u32::from(n - 16)))
                .unwrap_or(0),
        }
    }

    /// The configurable priority byte for system exception `n` (`6`, `11`,
    /// `14`, `15`), lower meaning higher priority.
    fn priority_byte(&mut self, n: u8) -> u8 {
        let (addr, shift) = match n {
            6 => (SHPR1_ADDR, 8),
            11 => (SHPR2_ADDR, 24),
            14 => (SHPR3_ADDR, 16),
            15 => (SHPR3_ADDR, 24),
            _ => return 0,
        };
        ((self.mem.read_word(addr).unwrap_or(0) >> shift) & 0xFF) as u8
    }

    /// Escalate a faulting data access at `addr` (from instruction at `pc`)
    /// to a HardFault, per ARMv6-M (no separate bus-fault exception exists).
    /// The stacked return address is the faulting instruction so the handler
    /// can fix the cause and retry it. Records a BusFault `PRECISERR` and the
    /// `FORCED` escalation bit. Continues into the handler.
    fn data_fault(&mut self, addr: u32, pc: u32) -> Result<Next, HaltCause> {
        let _ = addr;
        self.mem.record_fault(BFSR_PRECISERR, HFSR_FORCED);
        let targets_secure = self.hardfault_target_secure();
        self.take_exception_target(Exception::HardFault, pc, targets_secure)?;
        Ok(Next::Advance(self.regs.pc))
    }

    /// Whether the MPU permits this core's `access` to `addr` right now.
    /// The privilege comes from `CONTROL.nPRIV` in Thread mode; a bus with no
    /// MPU (or one still disabled) always answers true.
    fn mpu_allows(&mut self, addr: u32, access: MpuAccess) -> bool {
        let privileged = !(self.ipsr == 0 && self.control & 1 != 0);
        self.mem.mpu_check(addr, access, privileged, self.ipsr)
    }

    /// Escalate an MPU access violation at `addr` (from the instruction at
    /// `pc`) to HardFault. `fetch` distinguishes an instruction access
    /// violation from a data access violation in the recorded `MMFSR`; the
    /// faulting address is latched in `MMFAR`. Continues into the handler.
    fn mpu_fault(&mut self, addr: u32, pc: u32, fetch: bool) -> Result<Next, HaltCause> {
        self.mem.mpu_fault(addr, fetch);
        let targets_secure = self.hardfault_target_secure();
        self.take_exception_target(Exception::HardFault, pc, targets_secure)?;
        Ok(Next::Advance(self.regs.pc))
    }

    /// ARMv8-M stack-limit check: whether updating the *currently active* SP
    /// to `new_sp` would place it below that stack's limit register
    /// (`MSPLIM`/`PSPLIM`). The limits are architecturally zero (disabled) on
    /// ARMv6-M/ARMv7-M, so the check never fires there.
    fn stack_limit_violation(&self, new_sp: u32) -> bool {
        let limit = if self.sp_is_process() {
            self.psp_lim
        } else {
            self.msp_lim
        };
        self.isa == Isa::Armv8MBaseline && new_sp < limit
    }

    /// A stack-limit violation. The baseline has no StackFault exception, so
    /// it escalates to HardFault with `HFSR.FORCED` set; the stacked return
    /// address is the faulting instruction so the handler can grow the stack
    /// and retry. (A StackFault raised while stacking the fault frame would
    /// itself STKOF on silicon, locking the core up; that escalation is not
    /// modelled here.)
    fn stack_fault(&mut self, pc: u32) -> Result<Next, HaltCause> {
        self.mem.record_fault(0, HFSR_FORCED);
        let targets_secure = self.hardfault_target_secure();
        self.take_exception_target(Exception::HardFault, pc, targets_secure)?;
        Ok(Next::Advance(self.regs.pc))
    }

    /// Enter Handler mode for `ex`, pushing the eight-word frame onto the
    /// interrupted context's stack and switching to the main stack pointer.
    /// `return_pc` is where execution resumes on exception return. The
    /// exception targets the security state it was raised in.
    fn take_exception(&mut self, ex: Exception, return_pc: u32) -> Result<(), HaltCause> {
        self.take_exception_target(ex, return_pc, self.secure)
    }

    /// Like [`take_exception`](Self::take_exception), but the exception
    /// targets the given security state, which may differ from the running
    /// one on the v8-M baseline (a Non-secure fault escalating to Secure
    /// `HardFault`). The frame is still pushed onto the interrupted context's
    /// stack; the handler then runs in `targets_secure`.
    #[allow(clippy::too_many_lines)]
    fn take_exception_target(
        &mut self,
        ex: Exception,
        return_pc: u32,
        targets_secure: bool,
    ) -> Result<(), HaltCause> {
        // A fault taken while a fault handler is already active is a double
        // fault, which locks the core up. HardFault (priority -1) may however
        // always preempt a non-fault handler (SVC, PendSV, SysTick, an IRQ:
        // all priority >= 0), stacking a second frame and returning through
        // it. The interrupt controller only offers exceptions that outrank
        // the currently running one. On the v8-M baseline the two HardFaults
        // are banked, so a Secure HardFault preempts a Non-secure one (and
        // vice versa) unless it targets the bank that is already active.
        //
        // FAULTMASK raises the current priority to -1, so with it set even
        // HardFault (priority -1) cannot activate: an escalated fault then
        // locks the core up (QEMU: "Lockup: can't escalate 3 to HardFault
        // (current priority -1)"). PRIMASK only raises it to 0, so HardFault
        // still preempts under PRIMASK.
        if ex == Exception::HardFault
            && (self.faultmask
                || (self.ipsr == Exception::HardFault.number() && targets_secure == self.secure))
        {
            return Err(HaltCause::UnhandledException);
        }
        // Exception entry clears the local exclusive monitor and wakes the core
        // from a `WFI`/`WFE`/`SCR.SLEEPONEXIT` sleep. An interrupt is itself a
        // wakeup event, so it also sets the event latch consumed by `WFE`.
        self.exmon = None;
        self.sleeping = false;
        self.event = true;
        // The vector table lives at the (banked) `VTOR` for the exception's
        // target security state, defaulting to address zero when no `SCB` is
        // on the bus. Exception `n`'s entry is the `n`-th word of that table.
        let vtor = self.mem.vector_table_base(targets_secure);
        let vector_addr = vtor.wrapping_add(u32::from(ex.number()) * 4);
        let Ok(vector) = self.read_word_halt(vector_addr) else {
            return Err(HaltCause::UnhandledException);
        };
        if vector & 1 == 0 {
            return Err(HaltCause::NonThumbTarget);
        }
        // When the interrupted context owns the FP context (`CONTROL.FPCA`),
        // the frame is extended by the FP registers. With lazy stacking
        // (`FPCCR.LSPEN`) the S0-S15/FPSCR block is only reserved (its address
        // goes to FPCAR and LSPACT is set); otherwise it is stored now and the
        // handler starts with the FP state cleared.
        let fpu_active = self.control & CONTROL_FPCA != 0;
        let fpu_ctx = *self.fpu_ctx.lock().unwrap();
        let framesize = if fpu_active {
            EXTENDED_FRAME_BYTES
        } else {
            FRAME_BYTES
        };
        // The frame is pushed onto the stack the interrupted context was
        // using; the handler then runs on the main stack pointer.
        let use_process = self.sp_is_process();
        let background_secure = self.secure;
        let mut sp = self.regs.sp;
        let mut sxpsr = self.xpsr.get() & (APSR_QFLAGS | XPSR_T) | u32::from(self.ipsr);
        // ITSTATE occupies xPSR bits [15:10] (IT[7:2]) and [26:25] (IT[1:0]).
        // It is captured in the frame and cleared for the handler.
        let it = u32::from(self.itstate);
        sxpsr |= (it & 0xFC) << 8;
        sxpsr |= (it & 0x03) << 25;
        if sp & 0x4 != 0 {
            // CCR.STKALIGN (set at reset) enables the 8-byte alignment of the
            // frame base on exception entry; cleared, a 4-aligned-but-not-8
            // SP is pushed unadjusted (ARMv7-M; matches QEMU). Without an SCB
            // on the bus the architectural reset value (STKALIGN set) applies.
            let stkalign = self.mem.read_word(CCR_ADDR).unwrap_or(CCR_RESET) & CCR_STKALIGN != 0;
            if stkalign {
                sp = sp.wrapping_sub(4);
                sxpsr |= XPSR_ALIGN;
            }
        }
        let frame = sp.wrapping_sub(framesize);
        let mut off = 0u32;
        for r in [0u8, 1, 2, 3, 12] {
            self.write_word_halt(frame.wrapping_add(off), self.regs.read(r))?;
            off = off.wrapping_add(4);
        }
        self.write_word_halt(frame.wrapping_add(20), self.regs.lr)?;
        // The frame PC is a plain address; the interrupted Thumb state lives
        // in the stacked xPSR's T bit, matching QEMU and real Cortex-M frames.
        self.write_word_halt(frame.wrapping_add(24), return_pc)?;
        self.write_word_halt(frame.wrapping_add(28), sxpsr)?;
        if fpu_active {
            if fpu_ctx.lspen {
                // Lazy stacking: reserve the FP block for the first FP
                // instruction the handler runs.
                let mut ctx = self.fpu_ctx.lock().unwrap();
                ctx.fpcar = frame.wrapping_add(0x20) & !7;
                ctx.lspact = true;
            } else {
                // Non-lazy stacking: store the FP registers now.
                for i in 0..16u32 {
                    let v = self.vfp.read_s(i as u8);
                    self.write_word_halt(frame.wrapping_add(0x20).wrapping_add(i * 4), v)?;
                }
                self.write_word_halt(frame.wrapping_add(0x60), self.vfp.fpscr)?;
                self.vfp.s[0..16].fill(0);
                self.vfp.fpscr = 0;
            }
        }
        // ARMv8-M: an exception taken from a Secure background into the
        // Non-secure state stacks the callee-saves block (integrity signature
        // + r4..r11) below the basic frame, because the Non-secure handler
        // runs from a different register bank. `EXC_RETURN.DCRS` stays set
        // and the Secure return pops the block (QEMU `v7m_push_callee_stack`
        // in `v7m_exception_taken`). Entries that stay in Secure, and
        // Non-secure backgrounds, push no callee-saves block.
        let callee_block = self.isa == Isa::Armv8MBaseline && background_secure && !targets_secure;
        let frame = if callee_block {
            let cbase = frame.wrapping_sub(CALLEE_SAVES_BYTES);
            self.write_word_halt(cbase, CALLEE_SAVES_SIG)?;
            let mut coff = 8u32;
            for r in 4u8..=11 {
                self.write_word_halt(cbase.wrapping_add(coff), self.regs.read(r))?;
                coff = coff.wrapping_add(4);
            }
            cbase
        } else {
            frame
        };
        self.regs.lr = if self.isa == Isa::Armv8MBaseline {
            // ARMv8-M builds `EXC_RETURN` from `RES1|DCRS|FTYPE` (the
            // baseline has no FPU so the frame is always basic), `S` set from
            // the interrupted (background) state, `ES` from the target state,
            // `MODE` from the current thread/handler mode, and `SPSEL` from
            // the *target* state's `CONTROL.SPSEL` (QEMU
            // `v7m_exception_taken`). When target and background differ (a
            // Secure-targeting exception taken from Non-secure code) the
            // value carries `ES` without `S`, and the eventual return is a
            // plain return to Non-secure.
            let mut lr = EXC_RETURN_RES1 | EXC_RETURN_DCRS | EXC_RETURN_FTYPE;
            if self.secure {
                lr |= EXC_RETURN_S;
            }
            if targets_secure {
                lr |= EXC_RETURN_ES;
            }
            let target_control = if targets_secure == self.secure {
                self.control
            } else {
                self.other_ss_control
            };
            if target_control & EXC_RETURN_SPSEL != 0 {
                lr |= EXC_RETURN_SPSEL;
            }
            if self.ipsr == 0 {
                lr |= EXC_RETURN_MODE;
            }
            lr
        } else if fpu_active {
            if self.ipsr != 0 {
                EXC_RETURN_HANDLER_FP
            } else if use_process {
                EXC_RETURN_THREAD_PSP_FP
            } else {
                EXC_RETURN_THREAD_MSP_FP
            }
        } else if self.ipsr != 0 {
            EXC_RETURN_HANDLER
        } else if use_process {
            EXC_RETURN_THREAD_PSP
        } else {
            EXC_RETURN_THREAD_MSP
        };
        if use_process {
            self.regs.psp = frame;
            self.regs.sp = self.regs.msp;
        } else {
            self.regs.msp = frame;
            self.regs.sp = frame;
        }
        // A v8-M exception targeting the other security state switches the
        // handler into it: the frame stays on the interrupted state's stack
        // (now the inactive bank) and the handler runs with the target
        // state's bank, mirroring QEMU's `switch_v7m_security_state` in
        // `v7m_exception_taken`.
        if self.isa == Isa::Armv8MBaseline && targets_secure != self.secure {
            self.switch_security_state(targets_secure);
        }
        self.regs.pc = vector & !1;
        self.ipsr = ex.number();
        self.itstate = 0;
        // ARMv8-M v8.0M: an exception targeting the Non-secure state clears the
        // caller-saved registers (r0-r3, r12) that were pushed in the frame
        // above, and - when the interrupted context was Secure - the
        // callee-saves registers r4-r11 too (they were stacked in the block
        // pushed above), so Non-secure code cannot observe Secure values;
        // EAPSR (NZCV|Q|GE|IT) is cleared. Exceptions targeting Secure leave
        // the registers untouched on v8.0M (QEMU `v7m_exception_taken`).
        if self.isa == Isa::Armv8MBaseline && !targets_secure {
            for r in 0u8..=12 {
                if !(4..=11).contains(&r) || background_secure {
                    self.regs.write(r, 0);
                }
            }
            self.xpsr.set(self.xpsr.get() & !(APSR_FLAGS | APSR_Q));
            self.itstate = 0;
        }
        // The handler starts without an FP context (the interrupted one was
        // stacked or reserved above).
        self.control &= !CONTROL_FPCA;
        Ok(())
    }

    /// ARMv8-M tailchain: take `ex` in Handler mode without pushing a frame,
    /// chaining off the stack frame the failed exception return (`value`) was
    /// about to unstack. Mirrors QEMU's `v7m_exception_taken` with
    /// `dotailchain = true`; the `SecureFault` that results from a failed
    /// return always targets Secure state. The new `EXC_RETURN` keeps the
    /// aborted return's `S`/`MODE` (the interrupted context) and `DCRS`,
    /// sets `ES` for the Secure target, and takes `SPSEL` from the Secure
    /// `CONTROL` bank; `DCRS` is cleared when chaining off an exception that
    /// targeted Non-secure, because that entry stacked the callee-saves block
    /// which this Secure handler's eventual return will pop.
    fn take_exception_tailchain(&mut self, ex: Exception, value: u32) -> Result<Next, HaltCause> {
        let targets_secure = true;
        let vtor = self.mem.vector_table_base(targets_secure);
        let vector_addr = vtor.wrapping_add(u32::from(ex.number()) * 4);
        let Ok(vector) = self.read_word_halt(vector_addr) else {
            return Err(HaltCause::UnhandledException);
        };
        if vector & 1 == 0 {
            return Err(HaltCause::NonThumbTarget);
        }
        let mut lr = value & (EXC_RETURN_S | EXC_RETURN_MODE | EXC_RETURN_DCRS);
        // `RES1` carries the all-ones prefix; the baseline always pushes a
        // basic frame, so `FTYPE` is set.
        lr |= EXC_RETURN_RES1 | EXC_RETURN_FTYPE;
        // `ES` follows the target; `SPSEL` the target bank's `CONTROL`.
        lr &= !EXC_RETURN_ES;
        if targets_secure {
            lr |= EXC_RETURN_ES;
        }
        lr &= !EXC_RETURN_SPSEL;
        let target_control = if targets_secure == self.secure {
            self.control
        } else {
            self.other_ss_control
        };
        if target_control & EXC_RETURN_SPSEL != 0 {
            lr |= EXC_RETURN_SPSEL;
        }
        // Tailchaining off an exception that targeted Non-secure (`ES`
        // clear), from a Secure background, clears `DCRS`: the callee-saves
        // block belongs to the frame this handler will unstack on return.
        if lr & EXC_RETURN_S != 0 && (value & EXC_RETURN_ES) == 0 {
            lr &= !EXC_RETURN_DCRS;
        }
        // Exception entry wakes the core and clears the exclusive monitor.
        self.exmon = None;
        self.sleeping = false;
        self.event = true;
        self.switch_security_state(targets_secure);
        // QEMU writes `SPSEL = 0` and clears FPCA/IT for the handler.
        self.control &= !EXC_RETURN_SPSEL;
        self.control &= !CONTROL_FPCA;
        self.itstate = 0;
        self.regs.lr = lr;
        self.regs.pc = vector & !1;
        self.ipsr = ex.number();
        Ok(Next::Advance(self.regs.pc))
    }

    /// Enter Handler mode for exception number `n`, as decided by the
    /// machine's interrupt controller. The return address is the instruction
    /// the core is currently pointing at (the next one to execute on return).
    pub fn enter_exception(&mut self, n: u8) -> Result<(), HaltCause> {
        let ex = match n {
            3 => Exception::HardFault,
            6 => Exception::UsageFault,
            7 => Exception::SecureFault,
            11 => Exception::SvCall,
            14 => Exception::PendSV,
            15 => Exception::SysTick,
            16..=111 => Exception::Irq(n - 16),
            _ => return Err(HaltCause::UnhandledException),
        };
        // An external interrupt targets the security state its `ITNS` line
        // selects (reset: Non-secure, the ARMSSE default), so on the v8-M
        // baseline one taken from a Secure background is a Secure-to-Non-secure
        // entry: it stacks the callee-saves block and switches the handler into
        // the Non-secure state (QEMU `v7m_exception_taken`). The single-bank
        // system exceptions and every pre-v8-M machine target the current
        // state.
        let targets_secure = if n >= 16 && self.isa == Isa::Armv8MBaseline {
            self.mem.nvic_irq_targets_secure(u32::from(n - 16))
        } else {
            self.secure
        };
        self.take_exception_target(ex, self.regs.pc, targets_secure)
    }

    /// Unstack the exception frame and leave Handler mode, as directed by an
    /// `EXC_RETURN` value loaded into the PC. Returns the branch to the
    /// restored program counter. An extended return (`EXC_RETURN.FType == 0`,
    /// used when the interrupted context owned the FP context) also unstacks
    /// `S0`-`S15` and FPSCR Ã¢â‚¬â€ unless the frame was left lazily stacked
    /// (`FPCCR.LSPACT`), in which case the FPU still holds valid state and the
    /// reservation is simply abandoned.
    fn exception_return(&mut self, value: u32) -> Result<Next, HaltCause> {
        let to_handler = value == EXC_RETURN_HANDLER || value == EXC_RETURN_HANDLER_FP;
        let to_process = value == EXC_RETURN_THREAD_PSP || value == EXC_RETURN_THREAD_PSP_FP;
        let to_main = value == EXC_RETURN_THREAD_MSP || value == EXC_RETURN_THREAD_MSP_FP;
        // FType == 0 (bit 4 clear) marks an extended frame with FP state.
        let ftype = value & 0x10 != 0;
        if !(to_handler || to_process || to_main) {
            return Err(HaltCause::UnhandledException);
        }
        // In Handler mode the live stack pointer is the main SP; process
        // returns unstack the frame from the banked process SP.
        let frame = if to_process {
            self.regs.psp
        } else {
            self.regs.sp
        };
        let r0 = self.read_word_halt(frame.wrapping_add(0))?;
        let r1 = self.read_word_halt(frame.wrapping_add(4))?;
        let r2 = self.read_word_halt(frame.wrapping_add(8))?;
        let r3 = self.read_word_halt(frame.wrapping_add(12))?;
        let r12 = self.read_word_halt(frame.wrapping_add(16))?;
        let lr = self.read_word_halt(frame.wrapping_add(20))?;
        let pc_ret = self.read_word_halt(frame.wrapping_add(24))?;
        let sxpsr = self.read_word_halt(frame.wrapping_add(28))?;
        // An extended return restores the FP context that was stacked at
        // entry, unless lazy stacking left it reserved (LSPACT): then the FPU
        // still holds the pre-exception values and we just clear LSPACT.
        if !ftype {
            let ctx = *self.fpu_ctx.lock().unwrap();
            if ctx.lspact {
                self.fpu_ctx.lock().unwrap().lspact = false;
            } else {
                for i in 0..16u32 {
                    let v = self.read_word_halt(frame.wrapping_add(0x20).wrapping_add(i * 4))?;
                    self.vfp.write_s(i as u8, v);
                }
                self.vfp.fpscr = self.read_word_halt(frame.wrapping_add(0x60))?;
            }
        }
        // Exception return clears the local exclusive monitor.
        self.exmon = None;
        // A return-to-process hands the thread the handler's final main SP,
        // captured before the registers below switch the active pointer.
        let handler_msp = self.regs.sp;
        let align = sxpsr & XPSR_ALIGN != 0;
        let mut sp =
            frame
                .wrapping_add(FRAME_BYTES)
                .wrapping_add(if ftype { 0 } else { FP_FRAME_BYTES });
        if align {
            sp |= 4;
        }
        self.regs.write(0, r0);
        self.regs.write(1, r1);
        self.regs.write(2, r2);
        self.regs.write(3, r3);
        self.regs.write(12, r12);
        self.regs.lr = lr;
        // The stacked PC is a plain address; Thumb state is restored from the
        // stacked xPSR's T bit below, not from the PC's LSB.
        self.regs.pc = pc_ret;
        self.xpsr.set(sxpsr & (APSR_QFLAGS | XPSR_T));
        // Restore ITSTATE from its xPSR[15:10] / [26:25] slots.
        self.itstate = ((sxpsr >> 8) & 0xFC) as u8 | ((sxpsr >> 25) & 0x03) as u8;
        // The returning context owns the FP context iff the frame was extended.
        if ftype {
            self.control &= !CONTROL_FPCA;
        } else {
            self.control |= CONTROL_FPCA;
        }
        if to_process {
            self.regs.msp = handler_msp;
            self.regs.psp = sp;
            self.regs.sp = sp;
            self.control |= 0x2;
            self.ipsr = 0;
        } else if to_main {
            self.regs.sp = sp;
            self.regs.msp = sp;
            self.control &= !0x2;
            self.ipsr = 0;
        } else {
            // Return into the interrupted handler: IPSR comes from the
            // stacked xPSR's IPSR field.
            self.regs.sp = sp;
            self.regs.msp = sp;
            self.ipsr = (sxpsr & 0xFF) as u8;
        }
        // SCR.SLEEPONEXIT: returning to Thread mode re-enters sleep instead
        // of resuming the thread, until a pending exception wakes it. Fault
        // returns into another handler do not sleep.
        if (to_process || to_main)
            && self.mem.read_word(SCR_ADDR).unwrap_or(0) & SCR_SLEEPONEXIT != 0
        {
            self.sleeping = true;
        }
        self.just_returned = true;
        Ok(Next::Advance(self.regs.pc))
    }

    fn set_nz(&mut self, res: u32, v: bool) {
        self.xpsr
            .set_nzcv(res >> 31 == 1, res == 0, self.xpsr.c(), v);
    }

    fn set_nzcv(&mut self, res: u32, c: bool, v: bool) {
        self.xpsr.set_nzcv(res >> 31 == 1, res == 0, c, v);
    }

    fn write_shift_flags(&mut self, res: u32, c: Option<bool>) {
        let c = c.unwrap_or_else(|| self.xpsr.c());
        self.xpsr.set_nzcv(res >> 31 == 1, res == 0, c, false);
    }

    /// `set_nzcv` gated on IT-block state: the 16-bit data-processing
    /// instructions that normally set flags do not do so when they execute
    /// inside an IT block (CMP, CMN and TST are exempt and keep using the
    /// ungated writers). The gate works because `itstate` is still nonzero
    /// for the *whole* of an IT-block instruction's execution.
    fn set_nzcv_it(&mut self, res: u32, c: bool, v: bool) {
        if self.itstate == 0 {
            self.set_nzcv(res, c, v);
        }
    }

    /// `set_nz` gated on IT-block state, see `set_nzcv_it`.
    fn set_nz_it(&mut self, res: u32, v: bool) {
        if self.itstate == 0 {
            self.set_nz(res, v);
        }
    }

    /// `write_shift_flags` gated on IT-block state, see `set_nzcv_it`.
    fn write_shift_flags_it(&mut self, res: u32, c: Option<bool>) {
        if self.itstate == 0 {
            self.write_shift_flags(res, c);
        }
    }
}

/// Whether `v` is one of the ARMv6-M `EXC_RETURN` values (both the basic
/// `F1/F9/FD` set and the FP-carrying `E1/E9/ED` set).
const fn is_exc_return(v: u32) -> bool {
    matches!(
        v,
        EXC_RETURN_HANDLER
            | EXC_RETURN_THREAD_MSP
            | EXC_RETURN_THREAD_PSP
            | EXC_RETURN_HANDLER_FP
            | EXC_RETURN_THREAD_MSP_FP
            | EXC_RETURN_THREAD_PSP_FP
    )
}

/// A fully-resolved load/store (word/half/byte Ãƒâ€” load/store, plus
/// sign-extending loads).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LsAccess {
    Word { load: bool },
    Half { load: bool },
    Byte { load: bool },
    SextByte,
    SextHalf,
}

impl From<LsReg> for LsAccess {
    fn from(k: LsReg) -> Self {
        match k {
            LsReg::Str => Self::Word { load: false },
            LsReg::Strh => Self::Half { load: false },
            LsReg::Strb => Self::Byte { load: false },
            LsReg::Ldr => Self::Word { load: true },
            LsReg::Ldrh => Self::Half { load: true },
            LsReg::Ldrb => Self::Byte { load: true },
            LsReg::Ldrsb => Self::SextByte,
            LsReg::Ldrsh => Self::SextHalf,
        }
    }
}

impl From<LsImm> for LsAccess {
    fn from(k: LsImm) -> Self {
        match k {
            LsImm::Str => Self::Word { load: false },
            LsImm::Ldr => Self::Word { load: true },
            LsImm::Strb => Self::Byte { load: false },
            LsImm::Ldrb => Self::Byte { load: true },
            LsImm::Strh => Self::Half { load: false },
            LsImm::Ldrh => Self::Half { load: true },
        }
    }
}

impl From<MemOp> for LsAccess {
    fn from(m: MemOp) -> Self {
        match m {
            MemOp::Str => Self::Word { load: false },
            MemOp::Ldr => Self::Word { load: true },
            MemOp::Strb => Self::Byte { load: false },
            MemOp::Ldrb => Self::Byte { load: true },
            MemOp::Strh => Self::Half { load: false },
            MemOp::Ldrh => Self::Half { load: true },
            MemOp::Ldrsb => Self::SextByte,
            MemOp::Ldrsh => Self::SextHalf,
        }
    }
}

/// `a + b + carry_in`, returning `(result, carry_out, overflow)`.
fn add_flags(a: u32, b: u32, carry_in: bool) -> (u32, bool, bool) {
    let sum = (u64::from(a)) + (u64::from(b)) + u64::from(carry_in);
    let res = sum as u32;
    let carry = sum >> 32 != 0;
    let overflow = ((a ^ res) & (b ^ res)) & 0x8000_0000 != 0;
    (res, carry, overflow)
}

/// `a - b - borrow_in`, returning `(result, carry_out, overflow)`.
/// `carry_out` is the C flag: borrow complements it (C=1 means no borrow).
fn sub_flags(a: u32, b: u32, borrow_in: bool) -> (u32, bool, bool) {
    let res = a.wrapping_sub(b).wrapping_sub(u32::from(borrow_in));
    // C = 1 if the subtraction didn't need a borrow at the top.
    let carry = u64::from(a) >= u64::from(b) + u64::from(borrow_in);
    let overflow = ((a ^ b) & (a ^ res)) & 0x8000_0000 != 0;
    (res, carry, overflow)
}

/// Reverse the 32 bits of `v` (`RBIT`).
///
/// Swap the bytes (reversing the byte order *and* dropping each byte into the
/// mirror position), then finish reversing the bits within each byte so the
/// whole word is mirrored end to end.
#[must_use]
const fn rbit(v: u32) -> u32 {
    let v = v.swap_bytes();
    let v = (v & 0x0F0F_0F0F) << 4 | (v & 0xF0F0_F0F0) >> 4;
    let v = (v & 0x3333_3333) << 2 | (v & 0xCCCC_CCCC) >> 2;
    (v & 0x5555_5555) << 1 | (v & 0xAAAA_AAAA) >> 1
}

/// Clamp `v` to the signed `n`-bit range `[-2^(n-1), 2^(n-1) - 1]`, reporting
/// whether the value changed. `n == 32` (the `QADD` family and `SSAT #32`)
/// clamps to the whole signed 32-bit range.
#[allow(clippy::cast_possible_truncation)]
const fn signed_sat(v: i64, n: u8) -> (i32, bool) {
    if n >= 32 {
        if v > i32::MAX as i64 {
            (i32::MAX, true)
        } else if v < i32::MIN as i64 {
            (i32::MIN, true)
        } else {
            (v as i32, false)
        }
    } else {
        let max = (1i64 << (n as u32 - 1)) - 1;
        let min = -(1i64 << (n as u32 - 1));
        if v > max {
            (max as i32, true)
        } else if v < min {
            (min as i32, true)
        } else {
            (v as i32, false)
        }
    }
}

/// Clamp `v` to the unsigned `n`-bit range `[0, 2^n - 1]`, reporting whether
/// the value changed. `n == 32` (USAT #32) clamps only negative values.
#[allow(clippy::cast_possible_truncation)]
const fn unsigned_sat(v: i64, n: u8) -> (i32, bool) {
    if n >= 32 {
        if v < 0 {
            (0, true)
        } else {
            (v as i32, false)
        }
    } else {
        let max = (1i64 << (n as u32)) - 1;
        if v > max {
            (max as i32, true)
        } else if v < 0 {
            (0, true)
        } else {
            (v as i32, false)
        }
    }
}

/// Sign-extend an `8`- or `16`-bit lane to `i64`.
#[allow(clippy::cast_lossless)]
#[must_use]
const fn sign_ext_lane(v: u32, bits: u8) -> i64 {
    let v = (v & if bits == 8 { 0xFF } else { 0xFFFF }) as i64;
    let sign = 1i64 << (bits - 1);
    if v & sign != 0 {
        v - (sign << 1)
    } else {
        v
    }
}

/// Compute one lane of a parallel add/subtract instruction.
///
/// `a`/`b` are already lane-extracted operands; `add` selects add over
/// subtract; `bits` is the lane width (`8` or `16`). Returns the packed lane
/// result and whether the lane's GE flag is set. The `S` form sets GE when the
/// exact signed lane result is non-negative; the `U` form sets GE on the add's
/// carry out (or on the absence of a borrow for subtract). The halving forms
/// shift the exact result right by one; the saturating forms clamp each lane
/// to the `bits`-bit signed/unsigned range without touching the sticky Q flag.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]
const fn par_lane(a: u32, b: u32, add: bool, sat: ParSat, bits: u8) -> (u32, bool) {
    let mask = if bits == 8 { 0xFF } else { 0xFFFF };
    let a = a & mask;
    let b = b & mask;
    let (sa, sb) = match sat {
        ParSat::S | ParSat::Q | ParSat::Sh => (sign_ext_lane(a, bits), sign_ext_lane(b, bits)),
        _ => (a as i64, b as i64),
    };
    let exact = if add { sa + sb } else { sa - sb };
    let (ge, result) = match sat {
        ParSat::S => (exact >= 0, exact as u32 & mask),
        ParSat::U => {
            let carry = if add { exact > mask as i64 } else { exact >= 0 };
            (carry, exact as u32 & mask)
        }
        ParSat::Sh | ParSat::Uh => (false, (exact >> 1) as u32 & mask),
        ParSat::Q => {
            let (r, _) = signed_sat(exact, bits);
            (false, r as u32 & mask)
        }
        ParSat::Uq => {
            let (r, _) = unsigned_sat(exact, bits);
            (false, r as u32 & mask)
        }
    };
    (result, ge)
}

/// Shift `v` by an immediate 5-bit `amount` (LSL/LSR/ASR #n). Returns
/// `(result, new_carry)`. A `None` carry means the flag is unchanged (shift by
/// zero).
const fn shift_imm(kind: ShiftImm, v: u32, amount: u32) -> (u32, Option<bool>) {
    let amount = amount & 0x1F;
    shift_by_count(kind, v, amount)
}

/// Shift `v` by a register `count` (LSL/LSR/ASR/ROR by register).
fn shift_reg(kind: Shift, v: u32, count: u32) -> (u32, Option<bool>) {
    match kind {
        Shift::Ror => {
            // A zero amount leaves the C flag alone; a nonzero *multiple* of
            // 32 rotates to no effect but still reports C = bit31.
            if count == 0 {
                (v, None)
            } else if count & 0x1F == 0 {
                (v, Some(v >> 31 & 1 != 0))
            } else {
                let n = count & 0x1F;
                let carry = (v >> (n - 1)) & 1 != 0;
                (v.rotate_right(n), Some(carry))
            }
        }
        Shift::Lsl | Shift::Lsr | Shift::Asr => {
            let kind = match kind {
                Shift::Lsl => ShiftImm::Lsl,
                Shift::Lsr => ShiftImm::Lsr,
                Shift::Asr => ShiftImm::Asr,
                Shift::Ror => unreachable!(),
            };
            shift_by_count(kind, v, count)
        }
    }
}

/// Shift by an arbitrary count (may exceed 31 for register shifts).
const fn shift_by_count(kind: ShiftImm, v: u32, count: u32) -> (u32, Option<bool>) {
    match kind {
        ShiftImm::Lsl => {
            if count == 0 {
                (v, None)
            } else if count < 32 {
                let carry = (v >> (32 - count)) & 1 != 0;
                (v << count, Some(carry))
            } else if count == 32 {
                (0, Some(v & 1 != 0))
            } else {
                (0, Some(false))
            }
        }
        ShiftImm::Lsr => {
            if count == 0 {
                (v, None)
            } else if count < 32 {
                let carry = (v >> (count - 1)) & 1 != 0;
                (v >> count, Some(carry))
            } else if count == 32 {
                (0, Some(v >> 31 & 1 != 0))
            } else {
                (0, Some(false))
            }
        }
        ShiftImm::Asr => {
            if count == 0 {
                (v, None)
            } else if count < 32 {
                let carry = (v >> (count - 1)) & 1 != 0;
                (((v as i32) >> count) as u32, Some(carry))
            } else {
                // Arithmetic shift of >=32: sign-fill; C = sign bit.
                let sign = v >> 31 & 1 != 0;
                (if sign { u32::MAX } else { 0 }, Some(sign))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mem::bus::Bus;

    #[test]
    fn add_flags_basic() {
        let (r, c, v) = add_flags(0xFFFF_FFFF, 1, false);
        assert_eq!(r, 0);
        assert!(c);
        assert!(!v);
    }

    #[test]
    fn add_flags_overflow() {
        let (_, _, v) = add_flags(0x7FFF_FFFF, 1, false);
        assert!(v);
    }

    #[test]
    fn sub_flags_borrow() {
        let (r, c, v) = sub_flags(0, 1, false);
        assert_eq!(r, u32::MAX);
        assert!(!c);
        assert!(!v);
    }

    #[test]
    fn sub_flags_no_borrow() {
        let (r, c, _) = sub_flags(5, 3, false);
        assert_eq!(r, 2);
        assert!(c);
    }

    #[test]
    fn lsl_flags() {
        let (r, c) = shift_imm(ShiftImm::Lsl, 0x0000_0001, 31);
        assert_eq!(r, 0x8000_0000);
        assert_eq!(c, Some(false));
    }

    fn core() -> Core<crate::mem::flat::FlatMemory> {
        Core::new(crate::mem::flat::FlatMemory::with_layout(
            &[0x2000_0000],
            &[0x1000],
        ))
    }

    fn armv7m_core() -> Core<crate::mem::flat::FlatMemory> {
        Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x2000_0000],
            &[0x1000],
        ))
    }

    fn armv8m_core() -> Core<crate::mem::flat::FlatMemory> {
        Core::armv8m(crate::mem::flat::FlatMemory::with_layout(
            &[0x2000_0000],
            &[0x1000],
        ))
    }

    /// An ARMv8-M baseline core over a `SystemMemory` (SCB + the ARMSSE IDAU),
    /// so the TT family's region attribution and fault folding behave like the
    /// conformance machine.
    fn armv8m_sys_core() -> Core<crate::mem::system::SystemMemory<crate::mem::flat::FlatMemory>> {
        let backing = crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        );
        let mut mem = crate::mem::system::SystemMemory::new(backing);
        mem.enable_pmsav8();
        Core::armv8m(mem)
    }

    /// A Cortex-M4-style core with a shared FP context-control state, plus
    /// both a low vector region and SRAM.
    fn armv7m_fpu_core() -> (
        Core<crate::mem::flat::FlatMemory>,
        std::sync::Arc<std::sync::Mutex<FpuCtx>>,
    ) {
        let fpu_ctx = std::sync::Arc::new(std::sync::Mutex::new(FpuCtx::new()));
        let c = Core::armv7m_with_fpu(
            crate::mem::flat::FlatMemory::with_layout(
                &[0x0000_0000, 0x2000_0000],
                &[0x1000, 0x1000],
            ),
            fpu_ctx.clone(),
        );
        (c, fpu_ctx)
    }

    /// Place a byte sequence (an instruction stream) at `addr`.
    fn load_inst<M: crate::mem::PeripheralBus>(c: &mut Core<M>, addr: u32, bytes: &[u8]) {
        for (i, b) in bytes.iter().enumerate() {
            c.mem().write(addr + i as u32, *b).unwrap();
        }
    }

    #[test]
    fn cond_codes_against_flags() {
        let mut c = core();
        // N=1, Z=0, C=0, V=1.
        c.xpsr.set_nzcv(true, false, false, true);
        assert!(c.cond_passes(0x4)); // MI
        assert!(!c.cond_passes(0x5)); // PL
        assert!(!c.cond_passes(0x0)); // EQ
        assert!(c.cond_passes(0x1)); // NE
        assert!(!c.cond_passes(0x8)); // HI needs C && !Z
        assert!(c.cond_passes(0x9)); // LS is !C || Z
        assert!(c.cond_passes(0xA)); // GE: N == V
        assert!(!c.cond_passes(0xB)); // LT
    }

    #[test]
    fn sysreg_sp_bank_switch() {
        let mut c = core();
        c.regs.sp = 0x100;
        c.regs.msp = 0x100;
        c.regs.psp = 0x200;
        // CONTROL.SPSEL = 1 -> process SP becomes active.
        c.sysreg_write(SysReg::Control, 0x2).unwrap();
        assert_eq!(c.regs.sp, 0x200);
        assert_eq!(c.regs.msp, 0x100);
        assert_eq!(c.sysreg_read(SysReg::Msp).unwrap(), 0x100);
        assert_eq!(c.sysreg_read(SysReg::Psp).unwrap(), 0x200);
        // Writing PSP while process SP is active updates the live pointer.
        c.sysreg_write(SysReg::Psp, 0x300).unwrap();
        assert_eq!(c.regs.sp, 0x300);
        // Back to main SP; the process pointer is preserved.
        c.sysreg_write(SysReg::Control, 0x0).unwrap();
        assert_eq!(c.regs.sp, 0x100);
        assert_eq!(c.sysreg_read(SysReg::Psp).unwrap(), 0x300);
    }

    #[test]
    fn sysreg_primask_and_control() {
        let mut c = core();
        c.sysreg_write(SysReg::Primask, 1).unwrap();
        assert!(c.primask);
        assert_eq!(c.sysreg_read(SysReg::Primask).unwrap(), 1);
        assert_eq!(c.sysreg_read(SysReg::Control).unwrap(), 0);
        // On ARMv6-M (the default `Core::new` ISA) nPRIV/FPCA are RAZ/WI, so
        // only SPSEL survives a write.
        c.sysreg_write(SysReg::Control, 0x7).unwrap();
        assert_eq!(c.sysreg_read(SysReg::Control).unwrap(), 0x2);
        // Reserved sysreg reads/writes are typed errors, not silent.
        assert!(matches!(
            c.sysreg_read(SysReg::Other(0x7f)),
            Err(HaltCause::NotImplemented { .. })
        ));
        assert!(matches!(
            c.sysreg_write(SysReg::Other(0x7f), 0),
            Err(HaltCause::NotImplemented { .. })
        ));
    }

    #[test]
    fn sysreg_masks_basepri_and_faultmask() {
        let mut c = armv7m_core();
        // msr basepri, r0 -> f380 8011; mrs r0, basepri -> f3ef 8011.
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x80);
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xf3, 0x11, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x80);
        load_inst(&mut c, 0x2000_0004, &[0xef, 0xf3, 0x11, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x80);
        // BASEPRI_MAX (f380 8012) raises the mask but never lowers it.
        c.regs.write(0, 0x90);
        load_inst(&mut c, 0x2000_0008, &[0x80, 0xf3, 0x12, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x80);
        c.regs.write(0, 0x40);
        load_inst(&mut c, 0x2000_000c, &[0x80, 0xf3, 0x12, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x40);
        c.regs.write(0, 0);
        load_inst(&mut c, 0x2000_0010, &[0x80, 0xf3, 0x12, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x40);
        // A plain MSR BASEPRI writes freely in either direction.
        c.regs.write(0, 0x20);
        load_inst(&mut c, 0x2000_0014, &[0x80, 0xf3, 0x11, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x20);
        // FAULTMASK (f380 8013 / f3ef 8013) is a single mask bit.
        c.regs.write(0, 1);
        load_inst(&mut c, 0x2000_0018, &[0x80, 0xf3, 0x13, 0x80]);
        c.step().unwrap();
        assert!(c.faultmask);
        load_inst(&mut c, 0x2000_001c, &[0xef, 0xf3, 0x13, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 1);
    }

    #[test]
    fn cps_primask_and_faultmask() {
        // cpsid i -> b672; cpsie i -> b662; cpsid f -> b671; cpsie f -> b661.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x71, 0xb6]); // cpsid f
        c.step().unwrap();
        assert!(c.faultmask);
        load_inst(&mut c, 0x2000_0002, &[0x61, 0xb6]); // cpsie f
        c.step().unwrap();
        assert!(!c.faultmask);
        load_inst(&mut c, 0x2000_0004, &[0x72, 0xb6]); // cpsid i
        c.step().unwrap();
        assert!(c.primask);
        // On ARMv6-M the `f` bit is not an alias: it must leave FAULTMASK
        // untouched (the register does not exist there).
        let mut m0 = core();
        m0.regs.pc = 0x2000_0000;
        load_inst(&mut m0, 0x2000_0000, &[0x71, 0xb6]); // cpsid f
        m0.step().unwrap();
        assert!(!m0.faultmask);
        assert!(!m0.primask);
    }

    #[test]
    fn armv6m_has_no_basepri_or_faultmask() {
        // The ARMv6-M sysreg encodings for BASEPRI/BASEPRI_MAX/FAULTMASK do
        // not exist on Cortex-M0/M0+: MRS reads them as 0 and MSR is ignored,
        // matching the QEMU M0 reference (RAZ/WI) instead of silently
        // modeling the ARMv7-M registers.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x80);
        // msr basepri, r0 -> f380 8011.
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xf3, 0x11, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0);
        // mrs r0, basepri -> f3ef 8011 reads 0.
        c.regs.write(0, 0xDEAD);
        load_inst(&mut c, 0x2000_0004, &[0xef, 0xf3, 0x11, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        // msr faultmask, r0 -> f380 8013 is ignored.
        c.regs.write(0, 1);
        load_inst(&mut c, 0x2000_0008, &[0x80, 0xf3, 0x13, 0x80]);
        c.step().unwrap();
        assert!(!c.faultmask);
        // mrs faultmask, r0 -> f3ef 8013 reads 0.
        c.regs.write(0, 0xBAD);
        load_inst(&mut c, 0x2000_000c, &[0xef, 0xf3, 0x13, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        // msr basepri_max, r0 -> f380 8012 is ignored too.
        c.regs.write(0, 0x40);
        load_inst(&mut c, 0x2000_0010, &[0x80, 0xf3, 0x12, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0);
    }

    #[test]
    fn armv6m_undefined_faults_fold_to_hardfault() {
        // An undecodable instruction on ARMv6-M folds to HardFault: there is
        // no UsageFault exception on Cortex-M0/M0+. The SHCSR.USGFAULTENA
        // enable bit must not produce a distinct vector-6 entry even if a
        // (non-architectural) SCB write attempted it.
        let mut c = Core::new(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0040;
        load_inst(&mut c, 0x2000_0000, &[0x00, 0xde]); // udf
                                                       // Vector table: SP=0x2000_0040, PC=0x2000_0060, HardFault=0x2000_0080.
        load_inst(
            &mut c,
            0x0000_0000,
            &[
                0x40, 0x00, 0x00, 0x20, 0x61, 0x00, 0x00, 0x20, 0, 0, 0, 0, 0x81, 0x00, 0x00, 0x20,
            ],
        );
        c.step().unwrap();
        assert_eq!(c.ipsr, 3, "ARMv6-M must fold the fault to HardFault");
    }

    #[test]
    fn v8m_baseline_undefined_faults_fold_to_hardfault() {
        // ARMv8-M baseline has no UsageFault, so an undecodable 32-bit
        // encoding folds straight to HardFault even though the same bytes
        // are a valid ARMv7-M instruction.
        let mut c = Core::armv8m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0040;
        load_inst(&mut c, 0x2000_0000, &[0x09, 0xea, 0x0a, 0x08]); // and.w r8, r9, r10
        load_inst(
            &mut c,
            0x0000_0000,
            &[
                0x40, 0x00, 0x00, 0x20, 0x61, 0x00, 0x00, 0x20, 0, 0, 0, 0, 0x81, 0x00, 0x00, 0x20,
            ],
        );
        c.step().unwrap();
        assert_eq!(c.ipsr, 3, "baseline must fold the undefined encoding to HardFault");
    }

    #[test]
    fn v8m_baseline_v7m_only_executes_on_v7m() {
        // The same wide data-processing instruction is fine on ARMv7-M.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(8, 0x0F);
        c.regs.write(9, 0xFF);
        c.regs.write(10, 0x0F);
        load_inst(&mut c, 0x2000_0000, &[0x09, 0xea, 0x0a, 0x08]); // and.w r8, r9, r10
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 0x0F);
        assert_eq!(c.ipsr, 0);
    }

    #[test]
    fn v8m_baseline_tt_reads_security_state() {
        // Secure privileged thread (reset state): TT reads the attribution.
        let mut c = armv8m_sys_core();
        c.mem().write_word(3 * 4, 0x2000_0401).unwrap(); // HardFault vector
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0040;
        // Non-exempt address: IDAU region Addr[31:28], irvalid, Secure.
        c.regs.write(0, 0x2000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x40, 0xe8, 0x00, 0xf4]); // tt r4, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x02CC_0000);
        // System alias: exempt, no region/validity, still Secure.
        c.regs.write(0, 0xE000_0000);
        load_inst(&mut c, 0x2000_0004, &[0x40, 0xe8, 0x00, 0xf4]); // tt r4, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x004C_0000);
        // Region 15, not exempt.
        c.regs.write(0, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0008, &[0x40, 0xe8, 0x00, 0xf4]); // tt r4, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x0FCC_0000);
        // TTTT: force an unprivileged target; without an MPU the permission
        // bits are unchanged.
        c.regs.write(0, 0x2000_0000);
        load_inst(&mut c, 0x2000_000c, &[0x40, 0xe8, 0x40, 0xf7]); // tttt r7, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(7), 0x02CC_0000);
        // TTA: probe the opposite (Non-secure) state. The exempt alias flips
        // Non-secure (S=0, NSR/NSRW set)...
        c.regs.write(0, 0xE000_0000);
        load_inst(&mut c, 0x2000_0010, &[0x40, 0xe8, 0x80, 0xf5]); // tta r5, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(5), 0x003C_0000);
        // ...but a non-exempt address stays Secure while the SAU is disabled.
        c.regs.write(0, 0x2000_0000);
        load_inst(&mut c, 0x2000_0014, &[0x40, 0xe8, 0x80, 0xf5]); // tta r5, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(5), 0x02CC_0000);
        // TTAT on the exempt alias: unprivileged Non-secure.
        c.regs.write(0, 0xE000_0000);
        load_inst(&mut c, 0x2000_0018, &[0x40, 0xe8, 0xc0, 0xf6]); // ttat r6, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(6), 0x003C_0000);
        assert_eq!(c.ipsr, 0, "TT must not fault");
    }

    #[test]
    fn v8m_baseline_tt_reports_mpu_permissions() {
        // `TT`'s `r`/`rw` bits and the `mrvalid`/`mregion` field come from the
        // PMSAv8 MPU lookup in the target security/privilege state (QEMU's
        // `pmsav8_mpu_lookup`), not a hardcoded read/write.
        let mut c = armv8m_sys_core();
        c.mem().write_word(3 * 4, 0x2000_0401).unwrap(); // HardFault vector
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0040;
        // Region 0: 0x2000_0000..0x2000_001F, AP=1 (read/write), no XN.
        c.mem().write_word(0xE000_ED9C, 0x2000_0000 | 0x2).unwrap();
        c.mem().write_word(0xE000_EDA0, 0x2000_001F | 0x1).unwrap();
        c.mem().write_word(0xE000_ED94, 0x1).unwrap(); // MPU ENABLE
        // A region hit reports R/W plus MRVALID=1, MREGION=0 (the IDAU adds
        // IREGION/IRVALID/S like the baseline test).
        c.regs.write(0, 0x2000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x40, 0xe8, 0x00, 0xf4]); // tt r4, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x02CD_0000);
        // Just past the region with no `PRIVDEFENA`: a background fault
        // reports no permission and no region.
        c.regs.write(0, 0x2000_0020);
        load_inst(&mut c, 0x2000_0004, &[0x40, 0xe8, 0x00, 0xf4]); // tt r4, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x02C0_0000);
        // AP=3 (read-only for everyone): W clears, R and the region stay.
        c.mem().write_word(0xE000_ED9C, 0x2000_0000 | 0x6).unwrap();
        c.regs.write(0, 0x2000_0000);
        load_inst(&mut c, 0x2000_0008, &[0x40, 0xe8, 0x00, 0xf4]); // tt r4, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x02C5_0000);
        assert_eq!(c.ipsr, 0, "TT must not fault");
        // The same region gates real data accesses: a store to the read-only
        // region raises an MPU fault and escalates to HardFault.
        c.regs.write(0, 0x2000_0000);
        load_inst(&mut c, 0x2000_000c, &[0x00, 0x60]); // str r0, [r0]
        c.step().unwrap();
        assert_eq!(c.ipsr, 3, "MPU-denied store must escalate to HardFault");
    }

    #[test]
    fn v8m_baseline_tt_from_nonsecure() {
        let mk = || {
            let mut c = armv8m_sys_core();
            c.mem().write_word(3 * 4, 0x2000_0401).unwrap(); // HardFault vector
            c.secure = false;
            c.regs.pc = 0x2000_0000;
            c.regs.sp = 0x2000_0040;
            c.regs.write(0, 0x2000_0000);
            c
        };
        // A Non-secure TT reports Non-secure with no SAU/IDAU attribution.
        let mut c = mk();
        load_inst(&mut c, 0x2000_0000, &[0x40, 0xe8, 0x00, 0xf4]); // tt r4, r0
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x000C_0000);
        // TTA/TTAT read the opposite security state; from Non-secure they are
        // UNPREDICTABLE and QEMU UNDEFs them (HardFault on the baseline).
        let mut c = mk();
        load_inst(&mut c, 0x2000_0000, &[0x40, 0xe8, 0x80, 0xf5]); // tta r5, r0
        c.step().unwrap();
        assert_eq!(c.ipsr, 3, "TTA from Non-secure must UNDEF to HardFault");
        let mut c = mk();
        load_inst(&mut c, 0x2000_0000, &[0x40, 0xe8, 0xc0, 0xf6]); // ttat r6, r0
        c.step().unwrap();
        assert_eq!(c.ipsr, 3, "TTAT from Non-secure must UNDEF to HardFault");
    }

    #[test]
    fn v8m_baseline_tt_undef_forms() {
        // The constrained-unpredictable decode rejections are also caught at
        // execution: a non-zero low halfword (the mirrored-Rm form) faults.
        let mut c = armv8m_sys_core();
        c.mem().write_word(3 * 4, 0x2000_0401).unwrap(); // HardFault vector
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0040;
        c.regs.write(0, 0x2000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x40, 0xe8, 0x01, 0xf4]); // tt r4, r0 mirror-Rm
        c.step().unwrap();
        assert_eq!(c.ipsr, 3, "mirror-Rm TT must UNDEF to HardFault");
    }

    #[test]
    fn v8m_baseline_tt_undef_on_v6m_and_v7m() {
        // TT is not an ARMv6-M/ARMv7-M instruction: the encoding faults.
        for isa in [Isa::Armv6M, Isa::Armv7M] {
            let mut c = Core::new_with_isa(
                crate::mem::flat::FlatMemory::with_layout(
                    &[0x0000_0000, 0x2000_0000],
                    &[0x1000, 0x1000],
                ),
                isa,
            );
            c.mem().write_word(3 * 4, 0x2000_0081).unwrap(); // HardFault vector
            c.regs.pc = 0x2000_0000;
            c.regs.sp = 0x2000_0040;
            c.regs.write(0, 0x2000_0000);
            load_inst(&mut c, 0x2000_0000, &[0x40, 0xe8, 0x00, 0xf4]); // tt r4, r0
            c.step().unwrap();
            assert_eq!(c.ipsr, 3, "{isa:?} must UNDEF TT to HardFault");
        }
    }

    #[test]
    fn v8m_baseline_bxns_blxns_undef_on_v6m_and_v7m() {
        // BXNS/BLXNS are ARMv8-M-only: on ARMv6-M/ARMv7-M the encoding faults.
        for isa in [Isa::Armv6M, Isa::Armv7M] {
            for (bytes, name) in [(&[0x04, 0x47][..], "bxns"), (&[0x84, 0x47][..], "blxns")] {
                let mut c = Core::new_with_isa(
                    crate::mem::flat::FlatMemory::with_layout(
                        &[0x0000_0000, 0x2000_0000],
                        &[0x1000, 0x1000],
                    ),
                    isa,
                );
                c.mem().write_word(3 * 4, 0x2000_0081).unwrap(); // HardFault vector
                c.regs.pc = 0x2000_0000;
                c.regs.sp = 0x2000_0040;
                c.regs.write(0, 0x2000_0000);
                load_inst(&mut c, 0x2000_0000, bytes);
                c.step().unwrap();
                assert_eq!(c.ipsr, 3, "{isa:?} must UNDEF {name} to HardFault");
            }
        }
    }

    #[test]
    fn v8m_baseline_bxns_blxns_undef_from_nonsecure() {
        // From the Non-secure state both are UNDEF (QEMU's `trans_BXNS` gates
        // on `v8m_secure`), folding to HardFault with the SHCSR disabled.
        for (bytes, name) in [(&[0x04, 0x47][..], "bxns"), (&[0x84, 0x47][..], "blxns")] {
            let mut c = armv8m_sys_core();
            c.mem().write_word(3 * 4, 0x2000_0401).unwrap(); // HardFault vector
            c.secure = false;
            c.regs.pc = 0x2000_0000;
            c.regs.sp = 0x2000_0040;
            c.regs.write(0, 0x2000_0000);
            load_inst(&mut c, 0x2000_0000, bytes);
            c.step().unwrap();
            assert_eq!(c.ipsr, 3, "{name} from Non-secure must UNDEF to HardFault");
        }
    }

    #[test]
    fn v8m_baseline_blxns_to_nonsecure_frames_secure_stack() {
        let mut c = armv8m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0040;
        c.regs.write(0, 0x2000_0010); // even target -> Non-secure function call
        load_inst(&mut c, 0x2000_0000, &[0x84, 0x47]); // blxns r0
        c.step().unwrap();
        assert!(!c.secure);
        assert_eq!(c.regs.lr, FNC_RETURN);
        assert_eq!(c.regs.pc, 0x2000_0010);
        // The {return address, saved IPSR} frame went to the Secure stack bank.
        assert_eq!(c.other_ss_msp, 0x2000_0038);
        assert_eq!(c.mem().read_word(0x2000_0038).unwrap(), 0x2000_0003);
        assert_eq!(c.mem().read_word(0x2000_003C).unwrap(), 0);
        // The active SP is now the (empty) Non-secure bank.
        assert_eq!(c.regs.sp, 0);
    }

    #[test]
    fn v8m_baseline_blxns_odd_target_is_plain_blx() {
        let mut c = armv8m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0040;
        c.regs.write(0, 0x2000_0011); // odd target -> Secure, ordinary BLX
        load_inst(&mut c, 0x2000_0000, &[0x84, 0x47]); // blxns r0
        c.step().unwrap();
        assert!(c.secure);
        assert_eq!(c.regs.lr, 0x2000_0003);
        assert_eq!(c.regs.sp, 0x2000_0040, "no frame pushed for a Secure target");
        assert_eq!(c.regs.pc, 0x2000_0010);
    }

    #[test]
    fn v8m_baseline_bxns_switches_on_target_lsb() {
        let mut c = armv8m_core();
        c.regs.sp = 0x2000_0040;
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x2000_0011); // odd -> Secure
        load_inst(&mut c, 0x2000_0000, &[0x04, 0x47]); // bxns r0
        c.step().unwrap();
        assert!(c.secure);
        assert_eq!(c.regs.pc, 0x2000_0010);
        c.regs.write(0, 0x2000_0020); // even -> Non-secure
        load_inst(&mut c, 0x2000_0010, &[0x04, 0x47]); // bxns r0
        c.step().unwrap();
        assert!(!c.secure);
        assert_eq!(c.other_ss_msp, 0x2000_0040, "Secure bank stashed on the switch");
        assert_eq!(c.regs.pc, 0x2000_0020);
    }

    #[test]
    fn v8m_baseline_fncreturn_pops_secure_bank() {
        let mut c = armv8m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0040;
        c.regs.write(0, 0x2000_0010);
        load_inst(&mut c, 0x2000_0000, &[0x84, 0x47]); // blxns r0 -> Non-secure
        c.step().unwrap();
        // The Non-secure function returns with a plain `BX` to the FNC_RETURN
        // magic (`BXNS` is UNDEF in the Non-secure state), which VioARM's
        // `branch()` routes to the secure function return.
        c.regs.write(0, FNC_RETURN);
        load_inst(&mut c, 0x2000_0010, &[0x00, 0x47]); // bx r0
        c.step().unwrap();
        assert!(c.secure);
        assert_eq!(c.regs.sp, 0x2000_0040, "Secure SP restored past the frame");
        assert_eq!(c.regs.pc, 0x2000_0002);
        assert_eq!(c.ipsr, 0, "thread-mode return must not set IPSR");
    }

    #[test]
    fn v8m_baseline_sg_clears_itstate_only_in_secure() {
        // SG clears ITSTATE in the Secure state and is a NOP in the
        // Non-secure state, and it bypasses the IT block either way.
        let mut c = armv8m_core();
        c.itstate = 0x34;
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x7f, 0xe9, 0x7f, 0xe9]); // sg
        c.step().unwrap();
        assert_eq!(c.itstate, 0, "SG in Secure must clear ITSTATE");
        assert_eq!(c.regs.pc, 0x2000_0004);
        let mut c = armv8m_core();
        c.secure = false;
        c.itstate = 0x34;
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x7f, 0xe9, 0x7f, 0xe9]); // sg
        c.step().unwrap();
        assert_eq!(c.itstate, 0x34, "SG in Non-secure is a NOP");
    }

    #[test]
    fn v8m_baseline_switch_security_state_banks_sp() {
        let mut c = armv8m_core();
        c.regs.msp = 0x2000_0100;
        c.regs.psp = 0x2000_0200;
        c.regs.sp = 0x2000_0100; // Thread, SPSEL clear -> MSP
        c.other_ss_msp = 0x2000_0300;
        c.other_ss_psp = 0x2000_0400;
        c.other_ss_control = 0x2; // NS SPSEL set -> PSP in Thread mode
        c.switch_security_state(false);
        assert!(!c.secure);
        // The Secure bank is stashed, the NS bank loaded per its SPSEL.
        assert_eq!(c.other_ss_msp, 0x2000_0100);
        assert_eq!(c.other_ss_psp, 0x2000_0200);
        assert_eq!(c.regs.msp, 0x2000_0300);
        assert_eq!(c.regs.psp, 0x2000_0400);
        assert_eq!(c.regs.sp, 0x2000_0400);
        // CONTROL/limits swapped with the state.
        assert_eq!(c.control, 0x2);
        assert_eq!(c.other_ss_control, 0);
    }

    #[test]
    fn v8m_baseline_control_spsel_deposit() {
        // A thread-mode write of the *current* state's SPSEL swaps the active
        // SP between MSP and PSP...
        let mut c = armv8m_core();
        c.regs.msp = 0x2000_0100;
        c.regs.psp = 0x2000_0200;
        c.regs.sp = 0x2000_0100;
        c.write_control_spsel_for_secstate(true, true);
        assert_eq!(c.control, 0x2);
        assert_eq!(c.regs.sp, 0x2000_0200, "SPSEL set moves to the process stack");
        assert_eq!(c.regs.msp, 0x2000_0100);
        c.write_control_spsel_for_secstate(true, false);
        assert_eq!(c.control, 0);
        assert_eq!(c.regs.sp, 0x2000_0100, "SPSEL cleared moves back to MSP");
        // ...but in Handler mode SPSEL never selects the process stack, so the
        // bit changes without moving the active SP.
        let mut c = armv8m_core();
        c.ipsr = 3;
        c.regs.msp = 0x2000_0100;
        c.regs.psp = 0x2000_0200;
        c.regs.sp = 0x2000_0100;
        c.control = 0x2;
        c.write_control_spsel_for_secstate(true, false);
        assert_eq!(c.control, 0);
        assert_eq!(c.regs.sp, 0x2000_0100, "handler-mode SPSEL must not swap");
        // The inactive (Non-secure) state only gets the bit deposited.
        let mut c = armv8m_core();
        c.regs.sp = 0x2000_0100;
        c.write_control_spsel_for_secstate(false, true);
        assert_eq!(c.other_ss_control, 0x2);
        assert_eq!(c.regs.sp, 0x2000_0100, "inactive-state SPSEL must not swap");
    }

    #[test]
    fn v8m_baseline_ns_banked_sysreg_gating() {
        // From the Non-secure state the NS-banked encodings read as 0 and
        // writes are ignored (QEMU's `v7m_mrs`/`v7m_msr` NS guards).
        let mut c = armv8m_core();
        c.secure = false;
        c.other_ss_msp = 0x2000_0300;
        assert_eq!(c.sysreg_read(SysReg::MspNs).unwrap(), 0);
        assert_eq!(c.sysreg_read(SysReg::ControlNs).unwrap(), 0);
        assert_eq!(c.sysreg_read(SysReg::SpNs).unwrap(), 0);
        c.sysreg_write(SysReg::MspNs, 0x2000_0900).unwrap();
        c.sysreg_write(SysReg::ControlNs, 0x3).unwrap();
        c.sysreg_write(SysReg::SpNs, 0x2000_0A00).unwrap();
        assert_eq!(c.other_ss_msp, 0x2000_0300, "NS-bank writes ignored from NS");
        assert_eq!(c.other_ss_control, 0);
        // Unprivileged Secure code is also ignored for the NS bank...
        let mut c = armv8m_core();
        c.control = 0x1; // nPRIV
        c.sysreg_write(SysReg::MspNs, 0x2000_0900).unwrap();
        c.sysreg_write(SysReg::PspNs, 0x2000_0A00).unwrap();
        c.sysreg_write(SysReg::ControlNs, 0x3).unwrap();
        assert_eq!(c.other_ss_msp, 0);
        assert_eq!(c.other_ss_psp, 0);
        assert_eq!(c.other_ss_control, 0);
        // ...but privileged Secure code writes the bank, and SP_NS follows the
        // NS CONTROL.SPSEL.
        let mut c = armv8m_core();
        c.sysreg_write(SysReg::MspNs, 0x2000_0900).unwrap();
        c.sysreg_write(SysReg::PspNs, 0x2000_0A00).unwrap();
        c.sysreg_write(SysReg::ControlNs, 0x3).unwrap();
        c.sysreg_write(SysReg::SpNs, 0x2000_0B00).unwrap();
        assert_eq!(c.other_ss_msp, 0x2000_0900);
        assert_eq!(c.other_ss_psp, 0x2000_0B00, "SPSEL set selects the NS PSP");
        assert_eq!(c.other_ss_control, 0x3);
    }

    #[test]
    fn v8m_baseline_exception_return_secure_thread() {
        // EXC_RETURN to the Secure thread (0xFFFF_FFF9): unstack the eight
        // words from the Secure MSP and restore xPSR/IPSR/PC.
        let mut c = armv8m_core();
        c.ipsr = 3; // running the HardFault handler
        c.regs.msp = 0x2000_0100;
        c.regs.sp = 0x2000_0100;
        let frame = [0, 0, 0, 0, 0, 0xFFFF_FFF9, 0x2000_0001, 0x0100_0000];
        for (i, w) in frame.iter().enumerate() {
            c.mem().write_word(0x2000_0100 + (i as u32) * 4, *w).unwrap();
        }
        c.exception_return_v8m(EXC_RETURN_THREAD_MSP).unwrap();
        assert!(c.secure);
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.pc, 0x2000_0000);
        assert_eq!(c.regs.sp, 0x2000_0120, "frame consumed");
    }

    #[test]
    fn v8m_baseline_exception_return_to_nonsecure() {
        // EXC_RETURN to the Non-secure thread-MSP (S bit clear): the frame is
        // popped from the NS bank and the Secure bank is preserved.
        let mut c = armv8m_core();
        c.ipsr = 3;
        c.regs.msp = 0x2000_0100;
        c.regs.sp = 0x2000_0100;
        c.other_ss_msp = 0x2000_0500;
        let frame = [0, 0, 0, 0, 0, 0xFFFF_FFB9, 0x2000_0001, 0x0100_0000];
        for (i, w) in frame.iter().enumerate() {
            c.mem().write_word(0x2000_0500 + (i as u32) * 4, *w).unwrap();
        }
        c.exception_return_v8m(0xFFFF_FFB9).unwrap();
        assert!(!c.secure);
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.pc, 0x2000_0000);
        assert_eq!(c.regs.sp, 0x2000_0520);
        assert_eq!(c.other_ss_msp, 0x2000_0100, "Secure bank preserved");
    }

    /// An ARMv8-M baseline core over a `SystemMemory` with a loadable vector
    /// region at `0x1000_0000` (where the banked `S_VTOR` table lives) plus
    /// SRAM.
    fn armv8m_sys_core_tables(
    ) -> Core<crate::mem::system::SystemMemory<crate::mem::flat::FlatMemory>> {
        let backing = crate::mem::flat::FlatMemory::with_layout(
            &[0x1000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        );
        let mut mem = crate::mem::system::SystemMemory::new(backing);
        mem.enable_pmsav8();
        Core::armv8m(mem)
    }

/// A core positioned in the `Non-secure HardFault` handler, with `S_VTOR`
    /// relocated to `0x1000_0000` (the `SecureFault` vector set at its entry 7)
    /// and the `Secure MSP` at `0x2000_0900` - the setup for the `SecureFault`
    /// exception-return tests.
    fn ns_hardfault_core() -> Core<crate::mem::system::SystemMemory<crate::mem::flat::FlatMemory>> {
        let mut c = armv8m_sys_core_tables();
        c.mem().write_word(0xE002_ED08, 0x1000_0000).unwrap(); // S_VTOR
        c.mem().write_word(0x1000_0000 + 7 * 4, 0x2000_0401).unwrap(); // SecureFault vector
        c.regs.msp = 0x2000_0900; // the Secure bank's MSP
        c.regs.sp = 0x2000_0900;
        c.switch_security_state(false);
        c.regs.msp = 0x2000_0100;
        c.regs.sp = 0x2000_0100;
        c.ipsr = 3; // Non-secure HardFault handler running
        c
    }

    #[test]
    fn v8m_baseline_secure_fault_targets_secure_bank() {
        // A fault raised in Secure state escalates to the *Secure* HardFault:
        // the handler comes from S_VTOR and the EXC_RETURN carries S|ES.
        let mut c = armv8m_sys_core_tables();
        c.mem().write_word(0xE002_ED08, 0x1000_0000).unwrap(); // S_VTOR
        c.mem().write_word(0x1000_0000 + 3 * 4, 0x2000_0401).unwrap(); // Secure HF vector
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0100;
        load_inst(&mut c, 0x2000_0000, &[0x09, 0xea, 0x0a, 0x08]); // and.w (baseline undef)
        c.step().unwrap();
        assert_eq!(c.ipsr, 3);
        assert!(c.secure);
        assert_eq!(c.regs.lr, 0xFFFF_FFF9, "Secure thread-MSP EXC_RETURN");
        assert_eq!(c.regs.pc, 0x2000_0400, "handler from the Secure table");
    }

    #[test]
    fn v8m_baseline_ns_fault_escalates_to_secure_hardfault() {
        // `BFHFNMINS` resets clear: a Non-secure fault still escalates to the
        // *Secure* HardFault. The EXC_RETURN carries ES without S (returning
        // to Non-secure), the handler runs in Secure state, and the Secure
        // target leaves the caller-saved registers untouched (v8.0M).
        let mut c = armv8m_sys_core_tables();
        c.mem().write_word(0xE002_ED08, 0x1000_0000).unwrap(); // S_VTOR
        c.mem().write_word(0x1000_0000 + 3 * 4, 0x2000_0401).unwrap(); // Secure HF vector
        c.switch_security_state(false); // Non-secure background
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0100;
        c.regs.write(0, 0xDEAD_BEEF);
        load_inst(&mut c, 0x2000_0000, &[0x09, 0xea, 0x0a, 0x08]); // and.w (baseline undef)
        c.step().unwrap();
        assert_eq!(c.ipsr, 3);
        assert!(c.secure, "Secure HardFault handler runs Secure");
        assert_eq!(c.regs.lr, 0xFFFF_FFB9, "ES set, S clear: returns to NS thread");
        assert_eq!(c.regs.pc, 0x2000_0400, "handler from the Secure table");
        assert_eq!(c.regs.read(0), 0xDEAD_BEEF, "Secure target keeps r0");
    }

    #[test]
    fn v8m_baseline_ns_fault_targets_ns_hardfault_with_bfhfnmins() {
        // `AIRCR.BFHFNMINS` routes escalated faults to the Non-secure
        // HardFault: the handler comes from VTOR, stays Non-secure, and the
        // caller-saved registers are cleared for the Non-secure handler.
        let mut c = armv8m_sys_core_tables();
        c.mem().write_word(0xE000_ED08, 0x1000_0000).unwrap(); // NS VTOR
        c.mem().write_word(0xE000_ED0C, 0x05FA_0002).unwrap(); // BFHFNMINS
        c.mem().write_word(0x1000_0000 + 3 * 4, 0x2000_0401).unwrap(); // HF vector
        c.switch_security_state(false); // Non-secure background
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0100;
        c.regs.write(0, 0xDEAD_BEEF);
        load_inst(&mut c, 0x2000_0000, &[0x09, 0xea, 0x0a, 0x08]); // and.w (baseline undef)
        c.step().unwrap();
        assert_eq!(c.ipsr, 3);
        assert!(!c.secure, "Non-secure HardFault handler stays Non-secure");
        assert_eq!(c.regs.lr, 0xFFFF_FFB8, "S and ES clear: NS thread MSP");
        assert_eq!(c.regs.pc, 0x2000_0400, "handler from the Non-secure table");
        assert_eq!(c.regs.read(0), 0, "caller-saved regs cleared for NS handler");
    }

    #[test]
    fn v8m_baseline_failed_ns_return_takes_secure_fault() {
        // An EXC_RETURN carrying ES back into Non-secure state fails the
        // validity check: a SecureFault (vector 7) tailchains off the aborted
        // frame, records `SFSR.INVER`, and runs in Secure state.
        let mut c = ns_hardfault_core();
        c.exception_return_v8m(0xFFFF_FFB9).unwrap();
        assert!(c.secure, "SecureFault handler runs Secure");
        assert_eq!(c.ipsr, 7);
        assert_eq!(c.regs.pc, 0x2000_0400, "handler from the Secure table");
        assert_eq!(c.regs.lr, 0xFFFF_FFB9, "returns through the same frame");
        assert_eq!(c.regs.sp, 0x2000_0900, "no new frame pushed, Secure MSP");
        assert_eq!(
            c.mem().read_word(0xE002_ED28).unwrap(),
            0x4,
            "SFSR.INVER recorded"
        );
    }

    #[test]
    fn v8m_baseline_securefault_dcrs_zero_invalidates_return() {
        // EXC_RETURN with DCRS clear (no callee-saves block in the frame) is
        // equally invalid for a Non-secure return.
        let mut c = ns_hardfault_core();
        c.exception_return_v8m(0xFFFF_FF98).unwrap();
        assert!(c.secure);
        assert_eq!(c.ipsr, 7);
        assert_eq!(c.regs.pc, 0x2000_0400);
        assert_eq!(c.mem().read_word(0xE002_ED28).unwrap(), 0x4);
    }

    /// A core in Secure thread mode with banked `S_VTOR`/`VTOR` tables at
    /// `0x1000_0000` and the `HardFault` and `SecureFault` vectors set,
    /// positioned for an exception-return test.
    fn secure_thread_v8m_core() -> Core<crate::mem::system::SystemMemory<crate::mem::flat::FlatMemory>>
    {
        let mut c = armv8m_sys_core_tables();
        c.mem().write_word(0xE002_ED08, 0x1000_0000).unwrap(); // S_VTOR
        c.mem().write_word(0xE000_ED08, 0x1000_0000).unwrap(); // NS VTOR
        c.mem().write_word(0x1000_0000 + 3 * 4, 0x2000_0401).unwrap(); // HF vector
        c.mem().write_word(0x1000_0000 + 7 * 4, 0x2000_0401).unwrap(); // SecureFault vector
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0100;
        c.regs.msp = 0x2000_0100;
        for r in 0u8..=12 {
            c.regs.write(r, 0x4000_0000 + u32::from(r) * 0x1000);
        }
        c
    }

    #[test]
    fn v8m_baseline_secure_to_ns_entry_stacks_callee_saves() {
        // An exception targeting Non-secure taken from a Secure background
        // stacks the callee-saves block (integrity signature + r4..r11) below
        // the basic frame, `EXC_RETURN` keeps `DCRS` set, and the Non-secure
        // handler starts with r0-r12 cleared (QEMU `v7m_push_callee_stack`).
        let mut c = secure_thread_v8m_core();
        c.take_exception_target(Exception::HardFault, 0x2000_0000, false)
            .unwrap();
        assert!(!c.secure, "handler switched to Non-secure");
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.lr, 0xFFFF_FFF8, "S|DCRS|FTYPE|MODE, ES clear");
        let fb = c.other_ss_msp;
        assert_eq!(fb, 0x2000_00B8, "frame base below the callee block");
        assert_eq!(
            c.mem().read_word(fb).unwrap(),
            CALLEE_SAVES_SIG,
            "integrity signature"
        );
        assert_eq!(c.mem().read_word(fb + 0x04).unwrap(), 0, "reserved word");
        for r in 4u8..=11 {
            let stacked = c.mem().read_word(fb + 8 + u32::from(r - 4) * 4).unwrap();
            assert_eq!(stacked, 0x4000_0000 + u32::from(r) * 0x1000, "stacked r{r}");
        }
        for r in 0u8..=12 {
            assert_eq!(c.regs.read(r), 0, "r{r} cleared for the NS handler");
        }
        // The basic frame still holds the original caller-saved values.
        assert_eq!(
            c.mem().read_word(fb + 0x28).unwrap(),
            0x4000_0000,
            "r0 stacked before clearing"
        );
    }

    #[test]
    fn v8m_baseline_secure_to_ns_return_restores_callee_saves() {
        // Returning to Secure from the Non-secure handler pops the callee-saves
        // block and restores r4-r11 along with the basic frame (QEMU
        // `do_v7m_exception_exit`).
        let mut c = secure_thread_v8m_core();
        c.take_exception_target(Exception::HardFault, 0x2000_0000, false)
            .unwrap();
        c.exception_return_v8m(0xFFFF_FFF8).unwrap();
        assert!(c.secure, "returned to Secure");
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.pc, 0x2000_0000);
        for r in 0u8..=12 {
            assert_eq!(
                c.regs.read(r),
                0x4000_0000 + u32::from(r) * 0x1000,
                "r{r} restored"
            );
        }
        assert_eq!(c.regs.sp, 0x2000_0100, "both blocks consumed");
    }

    #[test]
    fn v8m_baseline_secure_to_ns_bad_signature_takes_invis_fault() {
        // A corrupted callee-saves signature on the Secure return fails the
        // integrity check: a `SecureFault` tailchains off the frame and
        // records `SFSR.INVIS`. `EXC_RETURN.DCRS` is cleared so the eventual
        // return pops the block.
        let mut c = secure_thread_v8m_core();
        c.take_exception_target(Exception::HardFault, 0x2000_0000, false)
            .unwrap();
        let fb = c.other_ss_msp;
        c.mem().write_word(fb, 0xDEAD_BEEF).unwrap(); // corrupt the signature
        c.exception_return_v8m(0xFFFF_FFF8).unwrap();
        assert!(c.secure, "SecureFault handler runs Secure");
        assert_eq!(c.ipsr, 7);
        assert_eq!(c.regs.pc, 0x2000_0400, "handler from the Secure table");
        assert_eq!(c.regs.lr, 0xFFFF_FFD9, "tailchained, DCRS cleared");
        assert_eq!(
            c.mem().read_word(0xE002_ED28).unwrap(),
            0x2,
            "SFSR.INVIS recorded"
        );
    }

    #[test]
    fn v8m_baseline_ns_to_ns_entry_has_no_callee_block() {
        // A Non-secure background targeting Non-secure pushes only the basic
        // frame: no callee-saves block, and r4-r11 survive (only the
        // caller-saved registers are cleared for the Non-secure handler).
        let mut c = armv8m_sys_core_tables();
        c.mem().write_word(0xE000_ED08, 0x1000_0000).unwrap(); // NS VTOR
        c.mem().write_word(0x1000_0000 + 3 * 4, 0x2000_0401).unwrap();
        c.switch_security_state(false); // Non-secure background
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0100;
        c.regs.msp = 0x2000_0100;
        for r in 0u8..=12 {
            c.regs.write(r, 0x4000_0000 + u32::from(r) * 0x1000);
        }
        c.take_exception_target(Exception::HardFault, 0x2000_0000, false)
            .unwrap();
        assert!(!c.secure);
        assert_eq!(c.regs.lr, 0xFFFF_FFB8, "S and ES clear");
        assert_eq!(c.regs.sp, 0x2000_00E0, "basic frame only, no callee block");
        for r in 0u8..=3 {
            assert_eq!(c.regs.read(r), 0, "r{r} cleared for NS handler");
        }
        assert_eq!(c.regs.read(12), 0, "r12 cleared for NS handler");
        for r in 4u8..=11 {
            assert_eq!(
                c.regs.read(r),
                0x4000_0000 + u32::from(r) * 0x1000,
                "r{r} kept (background was Non-secure)"
            );
        }
    }

    #[test]
    fn v8m_baseline_secure_to_secure_entry_has_no_callee_block() {
        // A Secure background targeting Secure pushes only the basic frame
        // (the interrupt state change the callee-saves block exists for never
        // happens), and r4-r11 are preserved.
        let mut c = secure_thread_v8m_core();
        c.take_exception_target(Exception::HardFault, 0x2000_0000, true)
            .unwrap();
        assert!(c.secure);
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.lr, 0xFFFF_FFF9, "S|ES|DCRS|FTYPE|MODE");
        assert_eq!(c.regs.sp, 0x2000_00E0, "basic frame only, no callee block");
        for r in 4u8..=11 {
            assert_eq!(
                c.regs.read(r),
                0x4000_0000 + u32::from(r) * 0x1000,
                "r{r} kept for the Secure handler"
            );
        }
    }

    #[test]
    fn v8m_baseline_irq_from_secure_targets_nonsecure() {
        // A software-pended external interrupt taken from a Secure thread: the
        // ARMSSE-default `ITNS` routes every line to the Non-secure state, so
        // `enter_exception` delivers it through `take_exception_target` as a
        // Secure-to-Non-secure entry: the callee-saves block is stacked on the
        // Secure MSP, the handler runs Non-secure with r0-r12 cleared, and its
        // return pops the block and restores r4-r11 along with the basic frame.
        let mut c = secure_thread_v8m_core();
        c.mem().write_word(0x1000_0000 + 16 * 4, 0x2000_0501).unwrap(); // IRQ0 vector
        c.enter_exception(16).unwrap();
        assert!(!c.secure, "default ITNS routes the IRQ to Non-secure");
        assert_eq!(c.ipsr, 16);
        assert_eq!(c.regs.lr, 0xFFFF_FFF8, "S|DCRS|FTYPE|MODE, ES clear");
        let fb = c.other_ss_msp;
        assert_eq!(
            c.mem().read_word(fb).unwrap(),
            CALLEE_SAVES_SIG,
            "callee-saves block on the Secure stack"
        );
        for r in 0u8..=12 {
            assert_eq!(c.regs.read(r), 0, "r{r} cleared for the NS handler");
        }
        c.exception_return_v8m(0xFFFF_FFF8).unwrap();
        assert!(c.secure, "returned to Secure");
        assert_eq!(c.ipsr, 0);
        for r in 0u8..=12 {
            assert_eq!(
                c.regs.read(r),
                0x4000_0000 + u32::from(r) * 0x1000,
                "r{r} restored from the blocks"
            );
        }
        assert_eq!(c.regs.sp, 0x2000_0100, "both blocks consumed");
    }

    #[test]
    fn v8m_baseline_irq_secure_line_stays_secure() {
        // Clearing an `ITNS` line claims the interrupt for the Secure state:
        // the same IRQ is then a Secure-to-Secure entry with only the basic
        // frame, no callee-saves block, and r4-r11 preserved.
        let mut c = secure_thread_v8m_core();
        c.mem().write_word(0xE000_E380, 0xFFFF_FFFE).unwrap(); // ITNS0: line 0 Secure
        c.mem().write_word(0x1000_0000 + 16 * 4, 0x2000_0501).unwrap(); // IRQ0 vector
        c.enter_exception(16).unwrap();
        assert!(c.secure, "secure-targeted line stays Secure");
        assert_eq!(c.ipsr, 16);
        assert_eq!(c.regs.lr, 0xFFFF_FFF9, "S|ES|DCRS|FTYPE|MODE");
        assert_eq!(c.regs.sp, 0x2000_00E0, "basic frame only, no callee block");
        for r in 0u8..=12 {
            assert_eq!(
                c.regs.read(r),
                0x4000_0000 + u32::from(r) * 0x1000,
                "r{r} kept for the Secure handler"
            );
        }
    }

    #[test]
    fn armv7m_irq_has_no_security_switch() {
        // A pre-v8-M core has a single security state: `enter_exception` never
        // routes an IRQ across the (nonexistent) TrustZone boundary, regardless
        // of the bus's `ITNS`.
        let mut c = armv7m_sys_core();
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0100;
        c.regs.msp = 0x2000_0100;
        c.mem().write_word(0x40, 0x2000_0601).unwrap(); // IRQ0 vector (VTOR = 0)
        c.enter_exception(16).unwrap();
        assert!(!c.secure, "v7-M has no secure state to leave");
        assert_eq!(c.ipsr, 16);
        assert_eq!(c.regs.lr, EXC_RETURN_THREAD_MSP);
        for r in 4u8..=11 {
            assert_eq!(c.regs.read(r), 0, "r{r} stays zero in the single state");
        }
        assert_eq!(c.regs.sp, 0x2000_00E0, "basic frame only");
    }

    #[test]
    fn v8m_baseline_sau_attribution() {
        // The `TT` security attribution consults the SAU registers in the SCB
        // (read through the bus), mirroring QEMU's `v8m_security_lookup`.
        let mut c = armv8m_sys_core();
        // Reset (SAU disabled, ALLNS clear): the IDAU decides, and since it
        // cannot downgrade, everything is Secure.
        assert!(!c.security_attribution(0x8000_0000, true).ns);
        // `ALLNS = 1`: everything defaults Non-secure, but the IDAU-Secure
        // regions (1, 3, ...) are upgraded back to Secure.
        c.mem().write_word(0xE000_EDD0, 0x2).unwrap();
        assert!(c.security_attribution(0x8000_0000, true).ns);
        assert!(!c.security_attribution(0x1000_0000, true).ns);
        // `ENABLE = 1` with region 0 = 0x8000_0000..0x1F (NS) and region 1 =
        // 0x0000_0000..0x1F (NSC).
        c.mem().write_word(0xE000_EDD0, 0x0).unwrap();
        c.mem().write_word(0xE000_EDD8, 0x0).unwrap(); // RNR = 0
        c.mem().write_word(0xE000_EDDC, 0x8000_0000).unwrap(); // RBAR
        c.mem().write_word(0xE000_EDE0, 0x8000_0001).unwrap(); // RLAR NS, ENABLE
        c.mem().write_word(0xE000_EDD8, 0x1).unwrap(); // RNR = 1
        c.mem().write_word(0xE000_EDDC, 0x0000_0000).unwrap(); // RBAR
        c.mem().write_word(0xE000_EDE0, 0x0000_0003).unwrap(); // RLAR NSC, ENABLE
        c.mem().write_word(0xE000_EDD0, 0x1).unwrap(); // ENABLE
        // Region 0 hit: Non-secure with SRVALID, SREGION = 0.
        let s = c.security_attribution(0x8000_0000, true);
        assert!(s.ns && !s.nsc && s.srvalid && s.sregion == 0);
        // Just past region 0, IDAU-NS region 8 with no SAU hit: matching QEMU,
        // the address stays Secure (not the architectural default-NS).
        let s = c.security_attribution(0x8000_0020, true);
        assert!(!s.ns && !s.srvalid);
        // NSC region 1: Secure-attributed, NSC with SRVALID, SREGION = 1.
        let s = c.security_attribution(0x0000_0000, true);
        assert!(!s.ns && s.nsc && s.srvalid && s.sregion == 1);
        // IDAU-Secure region 1 with no SAU hit: Secure, IRVALID/IREGION = 1.
        let s = c.security_attribution(0x1000_0000, true);
        assert!(!s.ns && !s.srvalid && s.irvalid && s.iregion == 1);
        // The probe of the Non-secure state sees the same attribution (only
        // the exempt aliases flip with `targetsec`).
        let s = c.security_attribution(0x8000_0000, false);
        assert!(s.ns && s.srvalid && s.sregion == 0);
        let s = c.security_attribution(0xE000_0000, false);
        assert!(s.ns && !s.irvalid, "exempt alias is Non-secure probing NS");
    }

    #[test]
    fn v8m_baseline_sysregs() {
        // BASEPRI and FAULTMASK do not exist on the baseline: writes are
        // ignored, reads RAZ.
        let mut c = armv8m_core();
        c.sysreg_write(SysReg::Basepri, 0x80).unwrap();
        c.sysreg_write(SysReg::BasepriMax, 0x40).unwrap();
        c.sysreg_write(SysReg::Faultmask, 1).unwrap();
        assert_eq!(c.basepri, 0);
        assert!(!c.faultmask);
        assert_eq!(c.sysreg_read(SysReg::Basepri).unwrap(), 0);
        assert_eq!(c.sysreg_read(SysReg::BasepriMax).unwrap(), 0);
        assert_eq!(c.sysreg_read(SysReg::Faultmask).unwrap(), 0);
        // Unlike ARMv6-M the baseline has an unprivileged state: CONTROL.nPRIV
        // and SPSEL are both writable, and FPCA (bit 2, no FPU) is RAZ/WI.
        c.sysreg_write(SysReg::Control, 0x7).unwrap();
        assert_eq!(c.sysreg_read(SysReg::Control).unwrap(), 0x3);
        // With nPRIV set the special-purpose register reads are RAZ.
        assert_eq!(c.sysreg_read(SysReg::Primask).unwrap(), 0);
        assert_eq!(c.sysreg_read(SysReg::Msp).unwrap(), 0);
    }

    #[test]
    fn v8m_baseline_msplim_psplim() {
        // MSPLIM/PSPLIM are readable/writable on the ARMv8-M baseline (their
        // encodings RAZ on ARMv6-M/ARMv7-M). msr msplim, r0 = f380 880a;
        // mrs r0, msplim = f3ef 800a; the psplim pair uses sys byte 0x0b.
        let mut c = armv8m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x2000_1000);
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xf3, 0x0a, 0x88]); // msr msplim, r0
        c.step().unwrap();
        assert_eq!(c.msp_lim, 0x2000_1000);
        load_inst(&mut c, 0x2000_0004, &[0xef, 0xf3, 0x0a, 0x80]); // mrs r0, msplim
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x2000_1000);
        c.regs.write(0, 0x2000_1800);
        load_inst(&mut c, 0x2000_0008, &[0x80, 0xf3, 0x0b, 0x88]); // msr psplim, r0
        c.step().unwrap();
        assert_eq!(c.psp_lim, 0x2000_1800);
        load_inst(&mut c, 0x2000_000c, &[0xef, 0xf3, 0x0b, 0x80]); // mrs r0, psplim
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x2000_1800);
        // A write to the *non-current* stack's limit never trips the check,
        // even when it lands below the active SP: PSPLIM is writable while
        // the main stack is in use.
        c.regs.write(0, 0x2000_0050);
        load_inst(&mut c, 0x2000_0010, &[0x80, 0xf3, 0x0b, 0x88]); // msr psplim, r0
        c.step().unwrap();
        assert_eq!(c.psp_lim, 0x2000_0050);
        assert_eq!(c.ipsr, 0);
        // Unprivileged reads of the limit registers are RAZ.
        c.sysreg_write(SysReg::Control, 0x1).unwrap();
        assert_eq!(c.sysreg_read(SysReg::MspLim).unwrap(), 0);
        assert_eq!(c.sysreg_read(SysReg::PspLim).unwrap(), 0);
    }

    #[test]
    fn msplim_raz_wi_on_v6m_and_v7m() {
        let mut c = core(); // ARMv6-M
        c.sysreg_write(SysReg::MspLim, 0x2000_1000).unwrap();
        c.sysreg_write(SysReg::PspLim, 0x2000_1000).unwrap();
        assert_eq!(c.sysreg_read(SysReg::MspLim).unwrap(), 0);
        assert_eq!(c.sysreg_read(SysReg::PspLim).unwrap(), 0);
        let mut c = armv7m_core();
        c.sysreg_write(SysReg::MspLim, 0x2000_1000).unwrap();
        assert_eq!(c.sysreg_read(SysReg::MspLim).unwrap(), 0);
    }

    #[test]
    fn msplim_push_violation_hardfaults() {
        const HANDLER: u32 = 0x2000_0400;
        let mut c = Core::armv8m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        c.mem().write_word(0xc, HANDLER | 1).unwrap(); // HardFault vector
        c.regs.sp = 0x2000_0800;
        c.regs.msp = 0x2000_0800;
        c.sysreg_write(SysReg::MspLim, 0x2000_07f0).unwrap();
        // push {r0-r4} (b41f, no LR): new SP 0x7ec < limit 0x7f0 -> HardFault.
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xDEAD_BEEF);
        load_inst(&mut c, 0x2000_0000, &[0x1f, 0xb4]);
        c.step().unwrap();
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HANDLER);
        assert_eq!(c.regs.sp, 0x2000_0800 - FRAME_BYTES);
        // The PUSH store did not complete: the frame's LR slot (SP-4) holds
        // the stacked EXC_RETURN, not the register value the push would have
        // written.
        assert_ne!(c.mem().read_word(0x2000_0800 - 4).unwrap(), 0xDEAD_BEEF);
    }

    #[test]
    fn msplim_push_within_limit_succeeds() {
        let mut c = armv8m_core();
        c.regs.sp = 0x2000_0800;
        c.regs.msp = 0x2000_0800;
        c.sysreg_write(SysReg::MspLim, 0x2000_07f0).unwrap();
        c.regs.pc = 0x2000_0000;
        // push {r0-r3} (b40f, no LR): new SP 0x7f0 is not below the limit.
        c.regs.write(1, 0xAABBCCDD);
        load_inst(&mut c, 0x2000_0000, &[0x0f, 0xb4]);
        c.step().unwrap();
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.sp, 0x2000_07f0);
        assert_eq!(c.mem().read_word(0x2000_07f4).unwrap(), 0xAABBCCDD);
    }

    #[test]
    fn msr_msp_below_limit_hardfaults() {
        const HANDLER: u32 = 0x2000_0400;
        let mut c = Core::armv8m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        c.mem().write_word(0xc, HANDLER | 1).unwrap(); // HardFault vector
        c.regs.sp = 0x2000_0800;
        c.regs.msp = 0x2000_0800;
        c.sysreg_write(SysReg::MspLim, 0x2000_0800).unwrap();
        // msr msp, r0 (f380 8808) with r0 below the limit: ignored + fault.
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x2000_0700);
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xf3, 0x08, 0x88]);
        c.step().unwrap();
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HANDLER);
        // The write was suppressed: the stacked frame still holds the old SP.
        assert_eq!(c.regs.sp, 0x2000_0800 - FRAME_BYTES);
        assert_eq!(c.sysreg_read(SysReg::Msp).unwrap(), 0x2000_0800 - FRAME_BYTES);
        // Writing the *non-current* process stack below PSPLIM is allowed.
        c.sysreg_write(SysReg::PspLim, 0x2000_0800).unwrap();
        c.sysreg_write(SysReg::Psp, 0x2000_0050).unwrap();
        assert_eq!(c.ipsr, 3, "non-current stack write must not fault");
    }

    #[test]
    fn v8m_baseline_it_block() {
        // IT blocks are part of the baseline (unlike ARMv6-M). An IT EQ move
        // executes when Z is set and is suppressed when it is clear; the NOP
        // that follows the single-instruction block always runs.
        // it eq (bf08), moveq r0, #1 (2001), nop (bf00).
        let mut c = armv8m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(false, true, false, false); // Z set
        load_inst(&mut c, 0x2000_0000, &[0x08, 0xbf, 0x01, 0x20, 0x00, 0xbf]);
        for _ in 0..3 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.r[0], 1, "gated move must execute when Z is set");
        assert_eq!(c.ipsr, 0);

        let mut c = armv8m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(false, false, false, false); // Z clear
        load_inst(&mut c, 0x2000_0000, &[0x08, 0xbf, 0x01, 0x20, 0x00, 0xbf]);
        for _ in 0..3 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.r[0], 0, "gated move must be suppressed when Z is clear");
        assert_eq!(c.ipsr, 0);
    }

    #[test]
    fn msr_apsr_write_masks() {
        // msr apsr, r0 -> f380 8000 writes nzcvq only; the `_g` (8001) and
        // `_nzcvqg` (8002) forms expand the mask to GE.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        let v = 0x8000_0000 | 0x2000_0000 | 0x0800_0000 | 0x000F_0000;
        c.regs.write(0, v);
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xf3, 0x00, 0x80]); // msr apsr, r0
        c.step().unwrap();
        assert!(c.xpsr.n() && c.xpsr.c() && c.xpsr.q());
        assert_eq!(c.xpsr.ge(), 0);
        load_inst(&mut c, 0x2000_0004, &[0x80, 0xf3, 0x01, 0x80]); // msr apsr_g, r0
        c.step().unwrap();
        assert_eq!(c.xpsr.ge(), 0xF);
        load_inst(&mut c, 0x2000_0008, &[0x80, 0xf3, 0x02, 0x80]); // msr apsr_nzcvqg, r0
        c.step().unwrap();
        assert!(c.xpsr.n() && c.xpsr.c() && c.xpsr.q());
        assert_eq!(c.xpsr.ge(), 0xF);
    }

    #[test]
    fn msr_sp_writes_word_align() {
        // A non-word-aligned MSP/PSP write is silently word-aligned: the low
        // two bits of the written value are dropped, matching QEMU.
        let mut c = armv7m_core();
        c.regs.write(0, 0x2000_0002);
        c.sysreg_write(SysReg::Msp, c.regs.read(0)).unwrap();
        assert_eq!(c.sysreg_read(SysReg::Msp).unwrap(), 0x2000_0000);
        c.regs.write(0, 0x2000_0003);
        c.sysreg_write(SysReg::Psp, c.regs.read(0)).unwrap();
        assert_eq!(c.sysreg_read(SysReg::Psp).unwrap(), 0x2000_0000);
        // Word-aligned (but not 8-aligned) values pass through untouched.
        c.regs.write(0, 0x2000_0004);
        c.sysreg_write(SysReg::Msp, c.regs.read(0)).unwrap();
        assert_eq!(c.sysreg_read(SysReg::Msp).unwrap(), 0x2000_0004);
    }

    #[test]
    fn svc_takes_svcall_exception_and_returns() {
        const SVC_ADDR: u32 = 0x2000_0100;
        const HANDLER: u32 = 0x2000_0400;
        const SP: u32 = 0x2000_0800;
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        // SVCall is exception 11: its vector sits at 0x2c in the vector table.
        c.mem().write_word(0x2c, HANDLER | 1).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        load_inst(&mut c, SVC_ADDR, &[0x99, 0xdf]); // svc #153
        load_inst(&mut c, HANDLER, &[0x70, 0x47]); // bx lr
        c.regs.pc = SVC_ADDR;
        c.step().unwrap();
        // The SVCall handler is entered: ipsr = 11, pc = handler, lr holds the
        // EXC_RETURN through which the handler returns.
        assert_eq!(c.ipsr, 11);
        assert_eq!(c.regs.pc, HANDLER);
        assert_eq!(c.regs.lr, EXC_RETURN_THREAD_MSP);
        // The task frame lies on the main SP: the stacked PC is one halfword
        // past the SVC, and the stacked xPSR carries Thumb with the
        // pre-exception IPSR (0 in Thread mode).
        assert_eq!(c.regs.sp, SP - FRAME_BYTES);
        assert_eq!(
            c.mem().read_word(SP - FRAME_BYTES + 24).unwrap(),
            SVC_ADDR + 2
        );
        assert_eq!(c.mem().read_word(SP - FRAME_BYTES + 28).unwrap(), XPSR_T);
        c.step().unwrap();
        // `bx lr` performs the exception return: thread resumes at the
        // instruction after the SVC with its SP intact.
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.pc, SVC_ADDR + 2);
        assert_eq!(c.regs.sp, SP);
    }

    #[test]
    fn sleeponexit_sleeps_until_exception_wakes() {
        use crate::mem::flat::FlatMemory;
        use crate::mem::system::SystemMemory;
        const SVC_ADDR: u32 = 0x2000_0100;
        const HANDLER: u32 = 0x2000_0400;
        const SP: u32 = 0x2000_0800;
        let backing = FlatMemory::with_layout(&[0x0000_0000, 0x2000_0000], &[0x1000, 0x1000]);
        let mut c = Core::armv7m(SystemMemory::new(backing));
        // SVCall vector 11 at 0x2c, PendSV vector 14 at 0x38: both point at a
        // `bx lr` handler.
        c.mem().write_word(0x2c, HANDLER | 1).unwrap();
        c.mem().write_word(0x38, HANDLER | 1).unwrap();
        c.mem().write_word(SVC_ADDR, 0xdf00).unwrap(); // svc #0
        c.mem().write_word(HANDLER, 0x4770).unwrap(); // bx lr
        c.regs.sp = SP;
        c.regs.msp = SP;
        // SCR.SLEEPONEXIT: return to Thread mode goes straight back to sleep.
        c.mem().write_word(0xE000_ED10, 0x0000_0002).unwrap();
        c.regs.pc = SVC_ADDR;
        c.step().unwrap(); // take SVC
        assert_eq!(c.ipsr, 11);
        c.step().unwrap(); // bx lr -> exception return, then sleep
        assert_eq!(c.ipsr, 0);
        assert!(c.sleeping);
        assert_eq!(c.regs.pc, SVC_ADDR + 2);
        // A sleeping core does not execute.
        let pc = c.regs.pc;
        c.step().unwrap();
        assert_eq!(c.regs.pc, pc);
        assert_eq!(c.cycles, 2);
        // A pending exception wakes it on entry.
        c.mem().write_word(0xE000_ED04, 0x1000_0000).unwrap(); // pend PendSV
        c.enter_exception(14).unwrap();
        assert!(!c.sleeping);
        assert_eq!(c.ipsr, 14);
    }

    #[test]
    fn wfi_wfe_sleep_and_sev_generates_an_event() {
        use crate::mem::flat::FlatMemory;
        use crate::mem::system::SystemMemory;
        const WFI_ADDR: u32 = 0x2000_0100;
        const WFE_ADDR: u32 = 0x2000_0200;
        const SEV_ADDR: u32 = 0x2000_0300;
        const NOP_ADDR: u32 = 0x2000_0400;
        const SP: u32 = 0x2000_0800;
        let backing = FlatMemory::with_layout(&[0x0000_0000, 0x2000_0000], &[0x1000, 0x1000]);
        let mut c = Core::armv7m(SystemMemory::new(backing));
        c.regs.sp = SP;
        c.regs.msp = SP;

        // WFI sleeps and a sleeping core does not advance.
        load_inst(&mut c, WFI_ADDR, &[0x30, 0xbf]); // wfi
        c.regs.pc = WFI_ADDR;
        c.step().unwrap();
        assert!(c.sleeping);
        assert_eq!(c.regs.pc, WFI_ADDR + 2);
        let pc = c.regs.pc;
        c.step().unwrap();
        assert!(c.sleeping);
        assert_eq!(c.regs.pc, pc, "a sleeping core must not execute");

        // WFE with no event pending sleeps too.
        c.sleeping = false;
        load_inst(&mut c, WFE_ADDR, &[0x20, 0xbf]); // wfe
        c.regs.pc = WFE_ADDR;
        c.step().unwrap();
        assert!(c.sleeping);

        // SEV sets the event latch without sleeping.
        c.sleeping = false;
        load_inst(&mut c, SEV_ADDR, &[0x40, 0xbf]); // sev
        c.regs.pc = SEV_ADDR;
        c.step().unwrap();
        assert!(c.event);
        assert!(!c.sleeping);

        // WFE with an event pending does not sleep and consumes the event.
        load_inst(&mut c, WFE_ADDR, &[0x20, 0xbf]);
        c.regs.pc = WFE_ADDR;
        c.step().unwrap();
        assert!(!c.sleeping, "WFE with a pending event must not sleep");
        assert!(!c.event, "WFE consumes the event");

        // Exception entry wakes the core and is itself a wakeup event.
        c.mem().write_word(0x3C, 0x2000_0500 | 1).unwrap(); // SysTick vector
        c.enter_exception(15).unwrap();
        assert!(!c.sleeping);
        assert!(c.event);

        // A plain NOP hint stays awake.
        c.sleeping = false;
        c.event = false;
        load_inst(&mut c, NOP_ADDR, &[0x00, 0xbf]); // nop
        c.regs.pc = NOP_ADDR;
        c.step().unwrap();
        assert!(!c.sleeping);
        assert_eq!(c.regs.pc, NOP_ADDR + 2);
    }

    #[test]
    fn hardfault_preempts_svc_and_double_fault_halts() {
        const SVC_ADDR: u32 = 0x2000_0100;
        const SVC_HANDLER: u32 = 0x2000_0400;
        const HF_HANDLER: u32 = 0x2000_0600;
        const SP: u32 = 0x2000_0800;
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        // SVCall vector 11 at 0x2c; HardFault vector 3 at 0xc.
        c.mem().write_word(0x2c, SVC_HANDLER | 1).unwrap();
        c.mem().write_word(0xc, HF_HANDLER | 1).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        load_inst(&mut c, SVC_ADDR, &[0x00, 0xdf]); // svc #0
        load_inst(&mut c, SVC_HANDLER, &[0x00, 0xde]); // udf (0xDE00) -> HardFault
        load_inst(&mut c, SVC_HANDLER + 2, &[0x70, 0x47]); // bx lr
                                                           // HardFault handler: bx lr (returns to SVC).
        load_inst(&mut c, HF_HANDLER, &[0x70, 0x47]); // bx lr
        c.regs.pc = SVC_ADDR;
        c.step().unwrap(); // take SVC
        assert_eq!(c.ipsr, 11);
        c.step().unwrap(); // the UDEF in SVC takes a HardFault preempting it
                           // HardFault nests on top: ipsr = 3, a second frame is stacked (32
                           // bytes), and LR points back into Handler mode via EXC_RETURN_HANDLER.
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HF_HANDLER);
        assert_eq!(c.regs.lr, EXC_RETURN_HANDLER);
        assert_eq!(c.regs.sp, SP - FRAME_BYTES * 2);
        // Patch the stacked SVC-handler PC so the fault handler's return skips
        // the UDEF (the way firmware would repair the fault).
        let hf_sp = c.regs.sp;
        c.mem().write_word(hf_sp + 24, SVC_HANDLER + 2).unwrap();
        c.step().unwrap(); // bx lr -> return to SVC
        assert_eq!(c.ipsr, 11);
        assert_eq!(c.regs.sp, SP - FRAME_BYTES);
        assert_eq!(c.regs.pc, SVC_HANDLER + 2);
        c.step().unwrap(); // bx lr -> SVC returns to Thread mode
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.pc, SVC_ADDR + 2);
        assert_eq!(c.regs.sp, SP);
        // A fault inside the HardFault handler itself is a double fault: the
        // core must halt rather than stack a third frame.
        let mut d = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        d.mem().write_word(0xc, HF_HANDLER | 1).unwrap();
        d.mem().write_word(0x2c, SVC_HANDLER | 1).unwrap();
        d.regs.sp = SP;
        d.regs.msp = SP;
        load_inst(&mut d, SVC_ADDR, &[0x00, 0xdf]); // svc #0
        load_inst(&mut d, SVC_HANDLER, &[0x00, 0xde]); // udf -> HardFault
        load_inst(&mut d, HF_HANDLER, &[0x00, 0xde]); // udf inside HardFault
        d.regs.pc = SVC_ADDR;
        d.step().unwrap(); // SVC
        d.step().unwrap(); // HardFault (preempts SVC)
        assert_eq!(d.ipsr, 3);
        assert_eq!(d.step(), Err(HaltCause::UnhandledException));
    }

    #[test]
    fn svc_inside_svc_handler_escalates_to_hardfault() {
        const SVC_ADDR: u32 = 0x2000_0100;
        const SVC_HANDLER: u32 = 0x2000_0400;
        const HF_HANDLER: u32 = 0x2000_0600;
        const SP: u32 = 0x2000_0800;
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        // SVCall vector 11 at 0x2c; HardFault vector 3 at 0xc.
        c.mem().write_word(0x2c, SVC_HANDLER | 1).unwrap();
        c.mem().write_word(0xc, HF_HANDLER | 1).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        // svc #0; svc #0 again in the handler; the handler's bx lr (with the
        // escalated HardFault handler's bx lr returning into the SVC frame).
        load_inst(&mut c, SVC_ADDR, &[0x00, 0xdf]);
        load_inst(&mut c, SVC_HANDLER, &[0x00, 0xdf]); // nesting SVC
        load_inst(&mut c, SVC_HANDLER + 2, &[0x70, 0x47]); // bx lr
        load_inst(&mut c, HF_HANDLER, &[0x70, 0x47]); // bx lr (fault skipped)
        c.regs.pc = SVC_ADDR;
        c.step().unwrap(); // take SVC
        assert_eq!(c.ipsr, 11);
        c.step().unwrap(); // SVC in the handler: cannot preempt -> HardFault
                           // HardFault nests on top of the SVC frame; the stacked return address
                           // keeps SVC semantics (instruction *after* the offending SVC), so a
                           // bare `bx lr` returns straight into the SVC handler.
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HF_HANDLER);
        assert_eq!(c.regs.lr, EXC_RETURN_HANDLER);
        assert_eq!(c.regs.sp, SP - FRAME_BYTES * 2);
        assert_eq!(
            c.mem().read_word(SP - FRAME_BYTES * 2 + 24).unwrap(),
            SVC_HANDLER + 2
        );
        c.step().unwrap(); // bx lr -> back into the SVC handler
        assert_eq!(c.ipsr, 11);
        assert_eq!(c.regs.pc, SVC_HANDLER + 2);
        c.step().unwrap(); // bx lr -> SVC returns to Thread mode
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.pc, SVC_ADDR + 2);
        assert_eq!(c.regs.sp, SP);
    }

    #[test]
    fn svc_masked_by_primask_escalates_to_hardfault() {
        const SVC_ADDR: u32 = 0x2000_0100;
        const SVC_HANDLER: u32 = 0x2000_0400;
        const HF_HANDLER: u32 = 0x2000_0600;
        const SP: u32 = 0x2000_0800;
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        c.mem().write_word(0x2c, SVC_HANDLER | 1).unwrap();
        c.mem().write_word(0xc, HF_HANDLER | 1).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        c.sysreg_write(SysReg::Primask, 1).unwrap();
        load_inst(&mut c, SVC_ADDR, &[0x00, 0xdf]); // svc #0
        load_inst(&mut c, HF_HANDLER, &[0x70, 0x47]); // bx lr (fault skipped)
        c.regs.pc = SVC_ADDR;
        c.step().unwrap(); // masked SVC cannot be taken -> HardFault
                           // Priority -1 HardFault still preempts Thread mode; the stacked return
                           // address keeps SVC semantics (instruction *after* the offending SVC).
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HF_HANDLER);
        assert_eq!(c.regs.lr, EXC_RETURN_THREAD_MSP);
        assert_eq!(c.regs.sp, SP - FRAME_BYTES);
        assert_eq!(
            c.mem().read_word(SP - FRAME_BYTES + 24).unwrap(),
            SVC_ADDR + 2
        );
        c.step().unwrap(); // bx lr -> return to Thread mode, past the SVC
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.pc, SVC_ADDR + 2);
        assert_eq!(c.regs.sp, SP);
    }

    #[test]
    fn svc_locked_up_by_faultmask() {
        const SVC_ADDR: u32 = 0x2000_0100;
        const SVC_HANDLER: u32 = 0x2000_0400;
        const HF_HANDLER: u32 = 0x2000_0600;
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        c.mem().write_word(0x2c, SVC_HANDLER | 1).unwrap();
        c.mem().write_word(0xc, HF_HANDLER | 1).unwrap();
        c.regs.sp = 0x2000_0800;
        c.regs.msp = 0x2000_0800;
        c.sysreg_write(SysReg::Faultmask, 1).unwrap();
        load_inst(&mut c, SVC_ADDR, &[0x00, 0xdf]); // svc #0
        c.regs.pc = SVC_ADDR;
        // FAULTMASK raises the current priority to -1, so the escalated
        // HardFault (also -1) cannot activate: the core locks up. The SVC
        // itself still executes, so the halted PC sits past it.
        assert_eq!(c.step(), Err(HaltCause::UnhandledException));
        assert_eq!(c.regs.pc, SVC_ADDR + 2);
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.sp, 0x2000_0800); // no frame was pushed
    }

    #[test]
    fn mem_imm_word_roundtrip() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xCAFE_BABE);
        c.regs.write(2, 0x2000_0400);
        // str.w r1, [r2, #4] f8c2 1004; ldr.w r3, [r2, #4] f8d2 3004.
        load_inst(&mut c, 0x2000_0000, &[0xc2, 0xf8, 0x04, 0x10]);
        load_inst(&mut c, 0x2000_0004, &[0xd2, 0xf8, 0x04, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert_eq!(c.mem().read_word(0x2000_0404).unwrap(), 0xCAFE_BABE);
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0xCAFE_BABE);
        assert_eq!(c.regs.pc, 0x2000_0008);
    }

    #[test]
    fn mem_imm_byte_half_store() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xAABB_CCDD);
        c.regs.write(1, 0x1122_3344);
        c.regs.write(2, 0x2000_0400);
        // strb.w r0, [r2, #0] f882 0000; strh.w r1, [r2, #2] f8a2 1002.
        load_inst(&mut c, 0x2000_0000, &[0x82, 0xf8, 0x00, 0x00]);
        load_inst(&mut c, 0x2000_0004, &[0xa2, 0xf8, 0x02, 0x10]);
        c.step().unwrap();
        assert_eq!(c.mem().read(0x2000_0400).unwrap(), 0xDD);
        c.step().unwrap();
        assert_eq!(c.mem().read(0x2000_0402).unwrap(), 0x44);
        assert_eq!(c.mem().read(0x2000_0403).unwrap(), 0x33);
    }

    #[test]
    fn mem_imm_signed_loads() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.mem().write(0x2000_0410, 0x80).unwrap();
        c.mem().write_half(0x2000_0420, 0x8000).unwrap();
        // ldrsb.w r0, [r2, #0x10] f992 0010; ldrsh.w r1, [r2, #0x20] f9b2 1020.
        load_inst(&mut c, 0x2000_0000, &[0x92, 0xf9, 0x10, 0x00]);
        load_inst(&mut c, 0x2000_0004, &[0xb2, 0xf9, 0x20, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0xFFFF_FF80);
        c.step().unwrap();
        assert_eq!(c.regs.read(1), 0xFFFF_8000);
    }

    #[test]
    fn mem_imm_pre_post_index() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xDEAD_0000);
        c.regs.write(2, 0x2000_0400);
        // str.w r0, [r2, #8]! f842 0f08 (pre-indexed write-back).
        load_inst(&mut c, 0x2000_0000, &[0x42, 0xf8, 0x08, 0x0f]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_0408).unwrap(), 0xDEAD_0000);
        assert_eq!(c.regs.read(2), 0x2000_0408);
        assert_eq!(c.regs.pc, 0x2000_0004);
        // ldr.w r3, [r2], #8 f852 3b08 (post-indexed write-back).
        load_inst(&mut c, 0x2000_0004, &[0x52, 0xf8, 0x08, 0x3b]);
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0xDEAD_0000);
        assert_eq!(c.regs.read(2), 0x2000_0410);
        assert_eq!(c.regs.pc, 0x2000_0008);
    }

    #[test]
    fn mem_unprivileged_ldrt_strt() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x1122_3344);
        c.regs.write(2, 0x2000_0400);
        // strt r1, [r2, #4] f842 1e04 (unprivileged store, no write-back).
        load_inst(&mut c, 0x2000_0000, &[0x42, 0xf8, 0x04, 0x1e]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_0404).unwrap(), 0x1122_3344);
        assert_eq!(c.regs.read(2), 0x2000_0400);
        assert_eq!(c.regs.pc, 0x2000_0004);
        // ldrt r0, [r2, #8] f852 0e08 (unprivileged load, no write-back).
        c.mem().write_word(0x2000_0408, 0xAABB_CCDD).unwrap();
        load_inst(&mut c, 0x2000_0004, &[0x52, 0xf8, 0x08, 0x0e]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0xAABB_CCDD);
        assert_eq!(c.regs.read(2), 0x2000_0400);
        assert_eq!(c.regs.pc, 0x2000_0008);
    }

    #[test]
    fn exclusive_ldrex_strex_roundtrip() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.regs.write(1, 0xAABB_CCDD);
        c.mem().write_word(0x2000_0400, 0x1111_1111).unwrap();
        // ldrex r0, [r2] e852 0f00; strex r3, r1, [r2] e842 1300.
        load_inst(&mut c, 0x2000_0000, &[0x52, 0xe8, 0x00, 0x0f]);
        load_inst(&mut c, 0x2000_0004, &[0x42, 0xe8, 0x00, 0x13]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x1111_1111);
        assert_eq!(c.exmon, Some(0x2000_0400));
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0);
        assert_eq!(c.mem().read_word(0x2000_0400).unwrap(), 0xAABB_CCDD);
        assert_eq!(c.exmon, None);
    }

    #[test]
    fn exclusive_strex_fails_without_match() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.regs.write(0, 0xAABB_CCDD);
        c.mem().write_word(0x2000_0400, 0x1111_1111).unwrap();
        // A lone STREX with an open monitor fails and leaves memory alone.
        load_inst(&mut c, 0x2000_0000, &[0x42, 0xe8, 0x00, 0x03]); // strex r3, r0, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
        assert_eq!(c.mem().read_word(0x2000_0400).unwrap(), 0x1111_1111);
        // A STREX to a different address than the LDREX tagged also fails.
        c.regs.write(2, 0x2000_0404);
        load_inst(&mut c, 0x2000_0004, &[0x52, 0xe8, 0x00, 0x0f]); // ldrex r0, [r2]
        c.step().unwrap();
        assert_eq!(c.exmon, Some(0x2000_0404));
        c.regs.write(2, 0x2000_0400);
        load_inst(&mut c, 0x2000_0008, &[0x42, 0xe8, 0x00, 0x03]); // strex r3, r0, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
        assert_eq!(c.mem().read_word(0x2000_0400).unwrap(), 0x1111_1111);
        // The mismatched STREX still cleared the monitor, so a retry at the
        // tagged address now fails too.
        c.regs.write(2, 0x2000_0404);
        load_inst(&mut c, 0x2000_000c, &[0x42, 0xe8, 0x00, 0x03]); // strex r3, r0, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
    }

    #[test]
    fn exclusive_clrex_and_byte_half_forms() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.regs.write(1, 0xAABB_CCDD);
        c.mem().write(0x2000_0400, 0x11).unwrap();
        c.mem().write_half(0x2000_0410, 0x1122).unwrap();
        // ldrexb r0, [r2] e8d2 0f4f; strexb r3, r1, [r2] e8c2 1f43.
        load_inst(&mut c, 0x2000_0000, &[0xd2, 0xe8, 0x4f, 0x0f]);
        load_inst(&mut c, 0x2000_0004, &[0xc2, 0xe8, 0x43, 0x1f]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x11);
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0);
        assert_eq!(c.mem().read(0x2000_0400).unwrap(), 0xDD);
        // ldrexh r0, [r2] e8d2 0f5f at a fresh address, then CLREX
        // makes the following STREXH fail.
        load_inst(&mut c, 0x2000_0008, &[0xd2, 0xe8, 0x5f, 0x0f]); // ldrexh r0, [r2]
        c.regs.write(2, 0x2000_0410);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x1122);
        load_inst(&mut c, 0x2000_000c, &[0xbf, 0xf3, 0x2f, 0x8f]); // clrex
        c.step().unwrap();
        assert_eq!(c.exmon, None);
        load_inst(&mut c, 0x2000_0010, &[0xc2, 0xe8, 0x53, 0x1f]); // strexh r3, r1, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
        assert_eq!(c.mem().read_half(0x2000_0410).unwrap(), 0x1122);
    }

    #[test]
    fn exclusive_word_offset_and_exception_clears_monitor() {
        const HANDLER: u32 = 0x2000_0400;
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        // SVCall is exception 11: its vector sits at 0x2c in the vector table.
        c.mem().write_word(0x2c, HANDLER | 1).unwrap();
        c.regs.sp = 0x2000_0800;
        c.regs.msp = 0x2000_0800;
        c.regs.write(2, 0x2000_0000);
        c.mem().write_word(0x2000_0004, 0xCAFE_F00D).unwrap();
        c.regs.write(0, 0xDEAD_BEEF);
        // ldrex r0, [r2, #4] e852 f001: the monitor is tagged with the
        // effective address.
        load_inst(&mut c, 0x2000_0100, &[0x52, 0xe8, 0x01, 0x0f]);
        c.regs.pc = 0x2000_0100;
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0xCAFE_F00D);
        assert_eq!(c.exmon, Some(0x2000_0004));
        // svc #10 df0a: entering the handler clears the local monitor, so a
        // STREX in Thread mode afterwards must fail.
        load_inst(&mut c, 0x2000_0104, &[0x0a, 0xdf]);
        c.step().unwrap();
        assert_eq!(c.exmon, None);
        assert_eq!(c.ipsr, 11);
        // Handler returns via bx lr; the monitor stays clear across the
        // exception return as well.
        load_inst(&mut c, HANDLER, &[0x70, 0x47]);
        c.step().unwrap();
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.exmon, None);
        c.regs.pc = 0x2000_0108;
        c.regs.write(2, 0x2000_0004);
        load_inst(&mut c, 0x2000_0108, &[0x42, 0xe8, 0x00, 0x03]); // strex r3, r0, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
    }

    #[test]
    fn mem_reg_offset_roundtrip() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.regs.write(3, 0x10);
        c.regs.write(4, 0xDEAD_BEEF);
        // str.w r4, [r2, r3] f842 4003: offset Rm = 0x10, no write-back.
        load_inst(&mut c, 0x2000_0000, &[0x42, 0xf8, 0x03, 0x40]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_0410).unwrap(), 0xDEAD_BEEF);
        assert_eq!(c.regs.read(2), 0x2000_0400);
        assert_eq!(c.regs.pc, 0x2000_0004);
        // ldr.w r5, [r2, r3, lsl #2] f852 5023: offset Rm << 2 = 0x40.
        load_inst(&mut c, 0x2000_0004, &[0x52, 0xf8, 0x23, 0x50]);
        c.mem().write_word(0x2000_0440, 0x1234_ABCD).unwrap();
        c.step().unwrap();
        assert_eq!(c.regs.read(5), 0x1234_ABCD);
        assert_eq!(c.regs.read(2), 0x2000_0400);
        assert_eq!(c.regs.pc, 0x2000_0008);
    }

    #[test]
    fn mem_ldm_stm_push_pop() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_1000;
        c.regs.write(0, 0x1111_1111);
        c.regs.write(1, 0x2222_2222);
        c.regs.write(2, 0x3333_3333);
        c.regs.lr = 0xFFFF_FFF5;
        // push.w {r0-r2, lr} e92d 4007: STMDB sp!, {r0-r2, lr}.
        load_inst(&mut c, 0x2000_0000, &[0x2d, 0xe9, 0x07, 0x40]);
        c.step().unwrap();
        assert_eq!(c.regs.sp, 0x2000_0FF0);
        assert_eq!(c.mem().read_word(0x2000_0FF0).unwrap(), 0x1111_1111);
        assert_eq!(c.mem().read_word(0x2000_0FF4).unwrap(), 0x2222_2222);
        assert_eq!(c.mem().read_word(0x2000_0FF8).unwrap(), 0x3333_3333);
        assert_eq!(c.mem().read_word(0x2000_0FFC).unwrap(), 0xFFFF_FFF5);
    }

    #[test]
    fn mem_ldm_stm_load_writeback_and_pc() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0004;
        c.regs.sp = 0x2000_0800;
        c.mem().write_word(0x2000_0800, 0xCAFE_F00D).unwrap();
        c.mem().write_word(0x2000_0804, 0x2000_0008 | 1).unwrap();
        // pop.w {r0, pc} e8bd 8001: the loaded pc (with its Thumb bit) wins.
        load_inst(&mut c, 0x2000_0004, &[0xbd, 0xe8, 0x01, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0xCAFE_F00D);
        assert_eq!(c.regs.sp, 0x2000_0808);
        assert_eq!(c.regs.pc, 0x2000_0008);

        // ldmdb r2!, {r3, r4}: rn loaded last, rn in the list is the lowest.
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.mem().write_word(0x2000_03F8, 0xAAA0).unwrap();
        c.mem().write_word(0x2000_03FC, 0xBBB0).unwrap();
        load_inst(&mut c, 0x2000_0000, &[0x32, 0xe9, 0x18, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0xAAA0);
        assert_eq!(c.regs.read(4), 0xBBB0);
        assert_eq!(c.regs.read(2), 0x2000_03F8);
    }

    #[test]
    fn mem_ldm_stm_ia_no_writeback() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x2000_0200);
        c.regs.write(1, 0xC0FF_EE00);
        c.regs.write(2, 0x00FF_00FF);
        // stmia r0, {r1, r2} e880 0006: no write-back.
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xe8, 0x06, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x2000_0200);
        assert_eq!(c.mem().read_word(0x2000_0200).unwrap(), 0xC0FF_EE00);
        assert_eq!(c.mem().read_word(0x2000_0204).unwrap(), 0x00FF_00FF);
    }

    #[test]
    fn mem_unaligned_ldm_stm_faults() {
        use crate::mem::flat::FlatMemory;
        // A misaligned multiple transfer is an alignment fault, not a silent
        // transfer. Hard-fault paths for Thumb-1 LDM/STM, T32 LDM/STM, and
        // LDRD/STRD, plus aligned controls that must not fault.
        const HF: u32 = 0x2000_0100; // HardFault handler
        let new = || {
            let mut c = Core::armv7m(FlatMemory::with_layout(
                &[0x0000_0000, 0x2000_0000],
                &[0x1000, 0x1000],
            ));
            c.mem().write_word(3 * 4, HF | 1).unwrap(); // vector table in flash
            c.regs.sp = 0x2000_0800;
            c.regs.msp = 0x2000_0800;
            c
        };
        let expect_hardfault = |c: &mut Core<FlatMemory>| {
            assert_eq!(c.ipsr, 3);
            assert_eq!(c.regs.pc, HF);
            assert_eq!(c.regs.sp, 0x2000_0800 - 0x20);
        };

        // Thumb-1 stmia r4!, {r0, r1, r3} (c403) at an unaligned base.
        let mut c = new();
        c.regs.pc = 0x2000_0000;
        c.regs.write(4, 0x2000_0002);
        load_inst(&mut c, 0x2000_0000, &[0x03, 0xc4]);
        c.step().unwrap();
        expect_hardfault(&mut c);

        // T32 stm r4, {r0, r1} (e884 0003) at an unaligned base.
        let mut c = new();
        c.regs.pc = 0x2000_0000;
        c.regs.write(4, 0x2000_0002);
        load_inst(&mut c, 0x2000_0000, &[0x84, 0xe8, 0x03, 0x00]);
        c.step().unwrap();
        expect_hardfault(&mut c);

        // T32 strd r0, r1, [r4, #4] (e9c4 0101) at an unaligned base.
        let mut c = new();
        c.regs.pc = 0x2000_0000;
        c.regs.write(4, 0x2000_0002);
        load_inst(&mut c, 0x2000_0000, &[0xc4, 0xe9, 0x01, 0x01]);
        c.step().unwrap();
        expect_hardfault(&mut c);

        // Aligned controls must not fault: T32 stm r4, {r0, r1}.
        let mut c = new();
        c.regs.pc = 0x2000_0000;
        c.regs.write(4, 0x2000_0000);
        c.regs.write(0, 0xAABB_CCDD);
        c.regs.write(1, 0x1122_3344);
        load_inst(&mut c, 0x2000_0000, &[0x84, 0xe8, 0x03, 0x00]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_0000).unwrap(), 0xAABB_CCDD);
        assert_eq!(c.mem().read_word(0x2000_0004).unwrap(), 0x1122_3344);
    }

    #[test]
    fn mem_imm_pc_literal() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        // ldr.w r4, [pc, #8] f8df 4008: base Align(pc+4) = 0x20000004, so the
        // word is fetched from 0x2000000c.
        load_inst(&mut c, 0x2000_0000, &[0xdf, 0xf8, 0x08, 0x40]);
        c.mem().write_word(0x2000_000c, 0x1234_5678).unwrap();
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x1234_5678);
        assert_eq!(c.regs.pc, 0x2000_0004);
    }

    #[test]
    fn mem_imm_load_to_pc_branches() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x2000_0400);
        c.mem().write_word(0x2000_0400, 0x2000_0501).unwrap();
        // ldr.w pc, [r1, #0] f8d1 f000.
        load_inst(&mut c, 0x2000_0000, &[0xd1, 0xf8, 0x00, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0500);
    }

    #[test]
    fn branch_cbz_cbnz() {
        // cbz r0, +6 (b118): r0 == 0 so it branches to pc + 4 + 6.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0);
        load_inst(&mut c, 0x2000_0000, &[0x18, 0xb1]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_000a);

        // cbz r0, +6: r0 != 0 falls through.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 1);
        load_inst(&mut c, 0x2000_0000, &[0x18, 0xb1]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0002);

        // cbnz r3, +8 (b923): r3 != 0 branches; offset 8.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 3);
        load_inst(&mut c, 0x2000_0000, &[0x23, 0xb9]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_000c);
    }

    #[test]
    fn branch_tbb_tbh() {
        // tbb [r0, r1] (e8d0 f001): byte index at r0+r1. pc = 0x2000, so the
        // base pc+4 = 0x2004; index byte = 6 -> target 0x2004 + 12 = 0x2010.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x2000_0200);
        c.regs.write(1, 4);
        c.mem().write(0x2000_0204, 6).unwrap();
        load_inst(&mut c, 0x2000_0000, &[0xd0, 0xe8, 0x01, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0010);

        // tbh [r1, r2] (e8d1 f012): halfword index at r1 + 2*r2. Index = 0x100
        // -> +0x200 = target 0x2000 + 4 + 0x200.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x2000_0200);
        c.regs.write(2, 4);
        c.mem().write_half(0x2000_0208, 0x100).unwrap();
        load_inst(&mut c, 0x2000_0000, &[0xd1, 0xe8, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0204);
    }

    #[test]
    fn divide_signed_unsigned() {
        // udiv r2, r4, r5 (fbb4 f2f5): 100 / 4 = 25, flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(4, 100);
        c.regs.write(5, 4);
        load_inst(&mut c, 0x2000_0000, &[0xb4, 0xfb, 0xf5, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 25);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // sdiv r1, r0, r3 (fb90 f1f3): -100 / 7 truncates toward zero.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xFFFF_FF9C);
        c.regs.write(3, 7);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfb, 0xf3, 0xf1]);
        c.step().unwrap();
        assert_eq!(c.regs.r[1], 0xFFFF_FFF2);

        // sdiv r0, r0, r1 (fb90 f0f1): MIN / -1 saturates to MIN.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x8000_0000);
        c.regs.write(1, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfb, 0xf1, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);

        // A zero divisor yields zero for both flavors.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(4, 100);
        c.regs.write(5, 0);
        load_inst(&mut c, 0x2000_0000, &[0xb4, 0xfb, 0xf5, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xFFFF_FF9C);
        c.regs.write(1, 0);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfb, 0xf1, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
    }

    #[test]
    fn multiply_ops() {
        // mul r4, r5, r6 (fb05 f406): 10 * 5 = 50, flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(5, 10);
        c.regs.write(6, 5);
        load_inst(&mut c, 0x2000_0000, &[0x05, 0xfb, 0x06, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 50);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // mla r0, r1, r2, r3 (fb01 3002): 10 * 20 + 100 = 300.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 10);
        c.regs.write(2, 20);
        c.regs.write(3, 100);
        load_inst(&mut c, 0x2000_0000, &[0x01, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 300);

        // mls r5, r6, r7, r8 (fb06 8517): 100 - 7 * 6 = 58.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(6, 7);
        c.regs.write(7, 6);
        c.regs.write(8, 100);
        load_inst(&mut c, 0x2000_0000, &[0x06, 0xfb, 0x17, 0x85]);
        c.step().unwrap();
        assert_eq!(c.regs.r[5], 58);

        // A 32-bit multiply truncates to the low word and writes nothing else.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x8000_0001);
        c.regs.write(6, 2);
        load_inst(&mut c, 0x2000_0000, &[0x05, 0xfb, 0x06, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 2);
    }

    #[test]
    fn long_multiply_ops() {
        // smull r0, r1, r2, r3 (fb82 0103): -7 * 5 = -35 -> r1:r0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0xFFFF_FFF9);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x82, 0xfb, 0x03, 0x01]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xFFFF_FFDD);
        assert_eq!(c.regs.r[1], 0xFFFF_FFFF);

        // umull r0, r1, r9, r2 (fba9 0102): 0x10000 * 0x10000 = 2^32 -> r1 = 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x0001_0000);
        c.regs.write(2, 0x0001_0000);
        load_inst(&mut c, 0x2000_0000, &[0xa9, 0xfb, 0x02, 0x01]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        assert_eq!(c.regs.r[1], 1);

        // smlal r2, r1, r3, r4 (fbc3 2104): acc r1:r2 = 0x1_0000_0010,
        // product -5 * 4 = -20 -> total = 0xFFFF_FFFC, r1 cleared.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0xFFFF_FFFB);
        c.regs.write(4, 4);
        c.regs.write(1, 1);
        c.regs.write(2, 16);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0x04, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xFFFF_FFFC);
        assert_eq!(c.regs.r[1], 0);

        // umlal r2, r1, r3, r4 (fbe3 2104): acc r1:r2 = 0x2_0000_0064,
        // product 1000 * 10 = 10000 -> total = 0x2_0000_2774.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 1000);
        c.regs.write(4, 10);
        c.regs.write(1, 2);
        c.regs.write(2, 100);
        load_inst(&mut c, 0x2000_0000, &[0xe3, 0xfb, 0x04, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x0000_2774);
        assert_eq!(c.regs.r[1], 2);

        // Writing the low result register `RdLo` (a PC) is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 3);
        c.regs.write(3, 4);
        load_inst(&mut c, 0x2000_0000, &[0xa2, 0xfb, 0x03, 0xf1]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn dsp_mul_ops() {
        // smulbb r0, r1, r2 (fb11 f002): low*low, both sign-extended. With
        // r1 = 0xffff0003 the low half is 3, r2 low half is 4 -> 12.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_0003);
        c.regs.write(2, 0x0000_0004);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 12);

        // smulbt r0, r1, r2 (fb11 f012): n low * m high. r1 low = 3,
        // r2 = 0xffff0004 high half = -1 -> -3.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_0003);
        c.regs.write(2, 0xffff_0004);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_fffd);

        // smultb r0, r1, r2 (fb11 f022): n high * m low. r1 high = -1,
        // r2 low = 4 -> -4.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_0003);
        c.regs.write(2, 0x0000_0004);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x22, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_fffc);

        // smultt r0, r1, r2 (fb11 f032): high*high. Both high halves -1 -> 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_0003);
        c.regs.write(2, 0xffff_0004);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x32, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 1);

        // smulwb r0, r1, r2 (fb31 f002): r1 is a full signed word,
        // r2 high half -32768. 32766 * 32768 = 0x3fff0000, >> 16 = 0x3fff.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_8002);
        c.regs.write(2, 0x0000_8000);
        load_inst(&mut c, 0x2000_0000, &[0x31, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0000_3fff);

        // smulwt r0, r1, r2 (fb31 f012): -32766 * 3 = -98298; the arithmetic
        // shift of the 48-bit product rounds toward negative infinity to -2.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_8002);
        c.regs.write(2, 0x0003_0000);
        load_inst(&mut c, 0x2000_0000, &[0x31, 0xfb, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_fffe);

        // smmul r0, r1, r2 (fb51 f002): 0x80000000 * -1 = 0x80000000, and the
        // top word of that is 0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x51, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);

        // smmulr r0, r1, r2 (fb51 f012): the same product, plus 0x80000000
        // before the shift wraps it into the next word -> 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x51, 0xfb, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 1);

        // usad8 r0, r1, r2 (fb71 f002): each byte differs by 255, sum = 1020.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xff00_ff00);
        c.regs.write(2, 0x00ff_00ff);
        load_inst(&mut c, 0x2000_0000, &[0x71, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x3fc);

        // usada8 r0, r1, r2, r3 (fb71 3002): 1020 + 0x12345678.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xff00_ff00);
        c.regs.write(2, 0x00ff_00ff);
        c.regs.write(3, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x71, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x1234_5a74);

        // A DSP multiply that targets the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x02, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn dual_mul_ops() {
        // smlad r0, r1, r2, r3 (fb21 3002): p_lo + p_hi + Ra. With both halves
        // -32768 the products are 2^30 each; Ra = -1 makes the 64-bit sum
        // 0x7FFFFFFF, so Q is NOT set (a two-step add would wrongly set it).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        c.regs.write(3, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7FFF_FFFF);
        assert!(!c.xpsr.q());

        // The same products with Ra = 0x40000000 overflow the 32-bit result,
        // and the 64-bit check sees it: Q is set.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        c.regs.write(3, 0x4000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xC000_0000);
        assert!(c.xpsr.q());

        // smuad r0, r1, r2 (fb21 f002): the sentinel Ra drops the accumulate,
        // and the single 32-bit add wraps with Q set.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);
        assert!(c.xpsr.q());

        // smladx r0, r1, r2, r3 (fb21 3012) swaps Rm's halves: without the
        // swap the products cancel, with it r0 = 7 * 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_0007);
        c.regs.write(2, 0x0001_0000);
        c.regs.write(3, 0);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x12, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 7);

        // smlsd r0, r1, r2, r3 (fb41 3002): p_lo - p_hi + Ra, no overflow.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x41, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 5);
        assert!(!c.xpsr.q());

        // smusd r0, r1, r2 (fb41 f002): the difference alone, no Q.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        load_inst(&mut c, 0x2000_0000, &[0x41, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        assert!(!c.xpsr.q());

        // smlabb r0, r1, r2, r3 (fb11 3002): Ra + halfword product.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_0003);
        c.regs.write(2, 0x0000_0004);
        c.regs.write(3, 100);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 112);

        // smlabt r0, r1, r2, r3 (fb11 3012): the accumulate add overflows and
        // wraps, sticky-setting Q.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_7fff);
        c.regs.write(2, 0x7fff_0000);
        c.regs.write(3, 0x7fff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x12, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xbfff_0000);
        assert!(c.xpsr.q());

        // smlawb r0, r1, r2, r3 (fb31 3002): (Rn * Rm[15:0]) >> 16 + Ra.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_1000);
        c.regs.write(2, 0x0000_ffff);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x31, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 4);

        // smmla r0, r1, r2, r3 (fb51 3002): top word of the product + Ra.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0x8000_0000);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x51, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x4000_0005);

        // smmlar r0, r1, r2, r3 (fb51 3012): adds the low word's bit 31.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_ffff);
        c.regs.write(2, 0x8000_0000);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x51, 0xfb, 0x12, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 6);

        // smmls r0, r1, r2, r3 (fb61 3002): (Ra << 32) - product, borrow
        // included, so a tiny product of 6 with Ra = 0 gives -1, not 0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        c.regs.write(3, 0);
        load_inst(&mut c, 0x2000_0000, &[0x61, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_ffff);

        // smmlsr r0, r1, r2, r3 (fb61 3012): the rounding bit borrows back.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        c.regs.write(3, 0);
        load_inst(&mut c, 0x2000_0000, &[0x61, 0xfb, 0x12, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);

        // smlald r2, r1, r3, r4 (fbc3 21c4): the dual sum accumulates into
        // RdHi:RdLo and never touches Q.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x8000_8000);
        c.regs.write(4, 0x8000_8000);
        c.regs.write(1, 1);
        c.regs.write(2, 0);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0xc4, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x8000_0000);
        assert_eq!(c.regs.r[1], 1);
        assert!(!c.xpsr.q());

        // smlsld r2, r1, r3, r4 (fbd3 21c4): identical products cancel.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x8000_8000);
        c.regs.write(4, 0x8000_8000);
        c.regs.write(1, 0x1234_5678);
        c.regs.write(2, 0x9abc_def0);
        load_inst(&mut c, 0x2000_0000, &[0xd3, 0xfb, 0xc4, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x9abc_def0);
        assert_eq!(c.regs.r[1], 0x1234_5678);

        // smlalbb r2, r1, r3, r4 (fbc3 2184): a sign-extended halfword product
        // added to the pair.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x0000_0003);
        c.regs.write(4, 0x0000_0004);
        c.regs.write(1, 1);
        c.regs.write(2, 100);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0x84, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x0000_0070);
        assert_eq!(c.regs.r[1], 1);

        // umaal r2, r1, r3, r4 (fbe3 2164): unsigned product plus both halves
        // of the pair, low to RdLo and high to RdHi.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0xffff_ffff);
        c.regs.write(4, 0xffff_ffff);
        c.regs.write(1, 1);
        c.regs.write(2, 1);
        load_inst(&mut c, 0x2000_0000, &[0xe3, 0xfb, 0x64, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 3);
        assert_eq!(c.regs.r[1], 0xffff_fffe);

        // A word form targeting the PC and a long form targeting RdLo=PC are
        // both rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x02, 0x3f]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 2);
        c.regs.write(4, 3);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0xc4, 0xf1]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn qadd_family_saturates() {
        // qadd r0, r1, r2 (fa81 f082) without saturation: 2 + 3 = 5, Q stays
        // clear.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 5);
        assert!(!c.xpsr.q());

        // qadd r0, r1, r2 with r1 = INT32_MAX, r2 = 1: the sum clamps to
        // INT32_MAX and the Q flag latches.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x7fff_ffff);
        c.regs.write(2, 1);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7fff_ffff);
        assert!(c.xpsr.q());

        // qadd r0, r1, r2 with r1 = INT32_MIN, r2 = -1: clamps downward and
        // the Q flag latches again (it is sticky, not re-cleared).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);
        assert!(c.xpsr.q());

        // qsub r0, r1, r2 (fa81 f0a2): `Rd = SAT(Rm - Rn)` where the low-nibble
        // register is Rm (`r2`) and the high-nibble one is Rn (`r1`). With
        // r2 = INT32_MIN and r1 = 1 the result INT32_MIN - 1 clamps to
        // INT32_MIN.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 1);
        c.regs.write(2, 0x8000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0xa2, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);
        assert!(c.xpsr.q());

        // qdadd r0, r1, r2 (fa81 f192): `Rd = SAT(Rm + SAT(Rn*2))` with Rm
        // (`r2`) = INT32_MAX and Rn (`r1`) = 1; 1*2 + INT32_MAX clamps the
        // sum.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 1);
        c.regs.write(2, 0x7fff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x92, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7fff_ffff);
        assert!(c.xpsr.q());

        // qdadd r0, r1, r2 with Rn (`r1`) = 0 and Rm (`r2`) = INT32_MAX: the
        // *doubling* of Rn is 0, so no saturation.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0);
        c.regs.write(2, 0x7fff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x92, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7fff_ffff);
        assert!(!c.xpsr.q());

        // qdadd r0, r1, r2 with Rn (`r1`) = INT32_MAX and Rm (`r2`) = 0: the
        // doubling saturates to INT32_MAX, setting Q.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x7fff_ffff);
        c.regs.write(2, 0);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x92, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7fff_ffff);
        assert!(c.xpsr.q());

        // qdsub r0, r1, r2 (fa81 f1b2): `Rd = SAT(Rm - SAT(Rn*2))` with Rm
        // (`r2`) = 5 and Rn (`r1`) = 2: 5 - doubled 2 = 1, no saturation.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 5);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0xb2, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 1);
        assert!(!c.xpsr.q());

        // qdsub r0, r1, r2 with Rn (`r1`) = 0x40000000 and Rm (`r2`) =
        // INT32_MIN: the doubling of Rn clamps to INT32_MAX (Q), and
        // INT32_MIN - that clamps to INT32_MIN (Q stays latched).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x4000_0000);
        c.regs.write(2, 0x8000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0xb2, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);
        assert!(c.xpsr.q());

        // A saturating add targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x82, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn parallel_add_sub_executes() {
        // sadd8 r0, r1, r2 (fa81 f002): per-byte signed add with GE written per
        // lane on non-negative results.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_00FF);
        c.regs.write(2, 0x0000_0200);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8001_02FF);
        assert_eq!(c.xpsr.ge(), 0b0110);

        // ssub8 r0, r1, r2 (fac1 f002): lane 0 borrows, the rest are zero.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0102_0304);
        c.regs.write(2, 0x0102_0305);
        load_inst(&mut c, 0x2000_0000, &[0xc1, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0000_00FF);
        assert_eq!(c.xpsr.ge(), 0b1110);

        // sadd16 r0, r1, r2 (fa91 f002): each halfword lane sets two GE bits.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_0002);
        c.regs.write(2, 0x0001_0002);
        load_inst(&mut c, 0x2000_0000, &[0x91, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8002_0004);
        assert_eq!(c.xpsr.ge(), 0b0011);

        // uadd8 r0, r1, r2 (fa81 f042): GE follows the per-byte carry out.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0102_80FF);
        c.regs.write(2, 0x0102_80FF);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x42, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0204_00FE);
        assert_eq!(c.xpsr.ge(), 0b0011);

        // qadd8 r0, r1, r2 (fa81 f012): the saturated lane clamps, and neither
        // the Q flag nor GE is touched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0102_7F80);
        c.regs.write(2, 0x0000_0101);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0102_7F81);
        assert!(!c.xpsr.q());
        assert_eq!(c.xpsr.ge(), 0);

        // uqsub8 r0, r1, r2 (fac1 f052): the underflowing lane clamps to zero.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_0010);
        c.regs.write(2, 0x0000_0020);
        load_inst(&mut c, 0x2000_0000, &[0xc1, 0xfa, 0x52, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        assert!(!c.xpsr.q());

        // shadd16 r0, r1, r2 (fa91 f022): each halfword lane halves the exact
        // signed result (negative lanes round away from zero).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_7FFF);
        c.regs.write(2, 0x0000_0001);
        load_inst(&mut c, 0x2000_0000, &[0x91, 0xfa, 0x22, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xC000_4000);

        // uhsub8 r0, r1, r2 (fac1 f062): 0x10 - 0x20 wraps to 0xF8 after the
        // halving shift, as on the hardware.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_0010);
        c.regs.write(2, 0x0000_0020);
        load_inst(&mut c, 0x2000_0000, &[0xc1, 0xfa, 0x62, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0000_00F8);

        // sasx r0, r1, r2 (faa1 f002): lo = Rn_lo - Rm_hi, hi = Rn_hi + Rm_lo.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0001_0002);
        c.regs.write(2, 0x0004_0008);
        load_inst(&mut c, 0x2000_0000, &[0xa1, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0009_FFFE);
        assert_eq!(c.xpsr.ge(), 0b1100);

        // ssax r0, r1, r2 (fae1 f002): lo = Rn_lo + Rm_hi, hi = Rn_hi - Rm_lo.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0001_0002);
        c.regs.write(2, 0x0004_0008);
        load_inst(&mut c, 0x2000_0000, &[0xe1, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xFFF9_0006);
        assert_eq!(c.xpsr.ge(), 0b0011);

        // A parallel op targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x02, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn sel_selects_bytes_by_ge() {
        // sel r0, r1, r2 (faa1 f082): byte i follows GE flag i.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x1122_3344);
        c.regs.write(2, 0xAABB_CCDD);
        c.xpsr.set_ge(0b0101);
        load_inst(&mut c, 0x2000_0000, &[0xa1, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xAA22_CC44);

        // Chained after a parallel op: sadd8 then sel, so SEL reads the GE the
        // parallel op just wrote.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_00FF);
        c.regs.write(2, 0x0000_0200);
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x81, 0xfa, 0x02, 0xf0, 0xa1, 0xfa, 0x82, 0xf3],
        );
        c.step().unwrap();
        c.step().unwrap();
        // sadd8 wrote GE = 0b0110: bytes 1 and 2 follow r1, the others r2.
        assert_eq!(c.regs.r[3], 0x0001_0000);

        // sel targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0xa1, 0xfa, 0x82, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn extend_add_executes() {
        // sxth r0, r1 (fa0f f081): the two-operand alias sign-extends the low
        // halfword without any addend.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_ff80);
        load_inst(&mut c, 0x2000_0000, &[0x0f, 0xfa, 0x81, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_ff80);

        // sxtab r2, r3, r4 (fa43 f284): 1 + sign-extended byte 0x80.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 1);
        c.regs.write(4, 0x0000_0080);
        load_inst(&mut c, 0x2000_0000, &[0x43, 0xfa, 0x84, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xffff_ff81);

        // uxtah r5, r6, r7 (fa16 f587): 0x100 + zero-extended halfword 0xff00.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(6, 0x100);
        c.regs.write(7, 0x0001_ff00);
        load_inst(&mut c, 0x2000_0000, &[0x16, 0xfa, 0x87, 0xf5]);
        c.step().unwrap();
        assert_eq!(c.regs.r[5], 0x0001_0000);

        // sxtab16 r8, r9, r10 (fa29 fa88): per-halfword signed byte extends.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x0001_0002);
        c.regs.write(10, 0x80ff_0080);
        load_inst(&mut c, 0x2000_0000, &[0x29, 0xfa, 0x8a, 0xf8]);
        c.step().unwrap();
        // lo = 2 + sext(0x80) = 0xffffff82 & 0xffff = 0xff82; hi = 1 + sext(0xff)
        // = 0x0000...00, so the result is 0x0000_ff82.
        assert_eq!(c.regs.r[8], 0x0000_ff82);

        // uxtab16 r0, r1, r2 (fa31 f002): per-halfword add wraps each lane.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_ffff);
        c.regs.write(2, 0x0000_0001);
        load_inst(&mut c, 0x2000_0000, &[0x31, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        // lo = 0xffff + 1 -> 0x0000; hi = 0xffff + 0 -> 0xffff.
        assert_eq!(c.regs.r[0], 0xffff_0000);

        // sxtab r0, r1, r2, ror #8 (fa41 f082): the byte that feeds the extend
        // comes from bit 8 after the rotation.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 1);
        c.regs.write(2, 0x0000_8000);
        load_inst(&mut c, 0x2000_0000, &[0x41, 0xfa, 0x92, 0xf0]);
        c.step().unwrap();
        // ror #8 turns 0x00008000 into 0x00000080: extend that byte and add 1.
        assert_eq!(c.regs.r[0], 0xffff_ff81);

        // sxtb16 r0, r1 (fa2f f081): both bytes sign-extend into halfwords.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x80ff_007f);
        load_inst(&mut c, 0x2000_0000, &[0x2f, 0xfa, 0x81, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_007f);

        // A register extend targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x0f, 0xfa, 0x81, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn saturate_ssat_usat() {
        // ssat r0, #8, r1, lsl #4 (f301 1007): 0x01 << 4 = 16 fits the signed
        // 8-bit range, so no Q flag.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x01);
        load_inst(&mut c, 0x2000_0000, &[0x01, 0xf3, 0x07, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 16);
        assert!(!c.xpsr.q());

        // ssat r0, #8, r1, lsl #4: 0x08 << 4 = 128 clamps to 127, Q latches.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x08);
        load_inst(&mut c, 0x2000_0000, &[0x01, 0xf3, 0x07, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7f);
        assert!(c.xpsr.q());

        // ssat r0, #8, r1, asr #1 (f321 0047): -1 stays -1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xf3, 0x47, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_ffff);
        assert!(!c.xpsr.q());

        // ssat r0, #8, r1, asr #1: 0x80000000 >> 1 clamps to -128.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xf3, 0x47, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_ff80);
        assert!(c.xpsr.q());

        // usat r2, #8, r3, lsl #2 (f383 0287): 0x10 << 2 = 64 fits [0, 255].
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x10);
        load_inst(&mut c, 0x2000_0000, &[0x83, 0xf3, 0x87, 0x02]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 64);
        assert!(!c.xpsr.q());

        // usat r2, #8, r3, lsl #2: 0x40 << 2 = 256 clamps to 255, Q latches.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x40);
        load_inst(&mut c, 0x2000_0000, &[0x83, 0xf3, 0x88, 0x02]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xff);
        assert!(c.xpsr.q());

        // usat r2, #8, r3 (f383 2208): a negative value clamps to 0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x83, 0xf3, 0x08, 0x22]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0);
        assert!(c.xpsr.q());

        // ssat16 r0, #8, r1 (f321 0007): hi half 0x8001 clamps to -128, the
        // low half 0x007f fits. Both halves are written and Q latches.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_007f);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xf3, 0x07, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xff80_007f);
        assert!(c.xpsr.q());

        // usat16 r0, #4, r1 (f3a1 0004): hi half 0x0010 clamps to 15, the
        // low half 0x000f fits.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0010_000f);
        load_inst(&mut c, 0x2000_0000, &[0xa1, 0xf3, 0x04, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x000f_000f);
        assert!(c.xpsr.q());

        // A saturate targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x01, 0xf3, 0x07, 0x1f]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn sysreg_reads_and_writes_q_flag() {
        // `msr apsr, r1` writes the Q flag; `mrs r0, apsr` reads it back.
        // msr apsr, r1 -> f381 8000; mrs r0, apsr -> f3ef 8000.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0800_0000);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xf3, 0x00, 0x80]);
        c.step().unwrap();
        assert!(c.xpsr.q());
        load_inst(&mut c, 0x2000_0004, &[0xef, 0xf3, 0x00, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0] & 0x0800_0000, 0x0800_0000);
        assert_eq!(c.regs.r[0] & 0x07f0_0000, 0);
    }

    #[test]
    fn unary_rev_and_flags() {
        // rev r0, r1 (fa91 f081): byte-reverse a word, flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(1, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x91, 0xfa, 0x81, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7856_3412);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // rev16 r2, r3 (fa93 f293): swap bytes within each halfword.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x93, 0xfa, 0x93, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x3412_7856);

        // revsh r4, r5 (fa95 f4b5): byte-swap the low half, sign-extend.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_1234);
        load_inst(&mut c, 0x2000_0000, &[0x95, 0xfa, 0xb5, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 0x0000_3412);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_80FF);
        load_inst(&mut c, 0x2000_0000, &[0x95, 0xfa, 0xb5, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 0xFFFF_FF80);
    }

    #[test]
    fn unary_rbit_clz() {
        // rbit r6, r7 (fa97 f6a7): mirror all 32 bits.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(7, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x97, 0xfa, 0xa7, 0xf6]);
        c.step().unwrap();
        assert_eq!(c.regs.r[6], 0x1E6A_2C48);

        // clz r8, r9 (fab9 f889).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x000F_0000);
        load_inst(&mut c, 0x2000_0000, &[0xb9, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 12);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x8000_0000);
        load_inst(&mut c, 0x2000_0000, &[0xb9, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 0);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0);
        load_inst(&mut c, 0x2000_0000, &[0xb9, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 32);

        // Writing the destination register 15 is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfa, 0x80, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn bitfield_extract() {
        // ubfx r0, r1, #4, #8 (f3c1 1007): pull the middle byte out,
        // zero-extended, with the flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(1, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0xc1, 0xf3, 0x07, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x67);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // sbfx r2, r3, #8, #8 (f343 2207): a negative source byte
        // sign-extends.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x0000_80FF);
        load_inst(&mut c, 0x2000_0000, &[0x43, 0xf3, 0x07, 0x22]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xFFFF_FF80);

        // ... while a positive byte stays small.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x0000_1278);
        load_inst(&mut c, 0x2000_0000, &[0x43, 0xf3, 0x07, 0x22]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x12);
    }

    #[test]
    fn bitfield_insert_clear() {
        // bfi r4, r5, #8, #8 (f365 240f): drop r5's low byte into r4's
        // [15:8] window, leaving the rest alone.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(4, 0xFFFF_FFFF);
        c.regs.write(5, 0x12);
        load_inst(&mut c, 0x2000_0000, &[0x65, 0xf3, 0x0f, 0x24]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 0xFFFF_12FF);

        // bfc r6, #8, #8 (f36f 260f): clear the same window.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(6, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0000, &[0x6f, 0xf3, 0x0f, 0x26]);
        c.step().unwrap();
        assert_eq!(c.regs.r[6], 0xFFFF_00FF);

        // Writing the destination register 15 is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x60, 0xf3, 0x0f, 0x2f]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn extend_16bit() {
        // sxth r3, r5 (b22b): sign-extend the low halfword.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(5, 0x0000_FF80);
        load_inst(&mut c, 0x2000_0000, &[0x2b, 0xb2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0xFFFF_FF80);
        assert_eq!(c.regs.pc, 0x2000_0002);
        // The flags from the setup survive untouched.
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // sxtb r3, r5 (b26b): sign-extend the low byte.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_0080);
        load_inst(&mut c, 0x2000_0000, &[0x6b, 0xb2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0xFFFF_FF80);

        // uxth r3, r5 (b2ab): zero the top halfword.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0000, &[0xab, 0xb2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0xFFFF);

        // uxtb r3, r5 (b2eb): zero the top three bytes.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0xeb, 0xb2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0x78);
    }

    #[test]
    fn reverse_16bit() {
        // rev r3, r5 (ba2b): full word byte reversal.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(5, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x2b, 0xba]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0x7856_3412);
        assert_eq!(c.regs.pc, 0x2000_0002);
        // The flags from the setup survive untouched.
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // rev16 r3, r5 (ba6b): byte-reverse each halfword.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x6b, 0xba]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0x3412_7856);

        // revsh r3, r5 (baeb): reverse the low halfword and sign-extend.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_12F0);
        load_inst(&mut c, 0x2000_0000, &[0xeb, 0xba]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0xFFFF_F012);

        // A positive `revsh` keeps the value sign-bit-free.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_1234);
        load_inst(&mut c, 0x2000_0000, &[0xeb, 0xba]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0x3412);
    }

    #[test]
    fn it_block_conditional_execution() {
        // movs r0, #1 (Z=0) ; ite eq ; adds r1, r1, #1 (suppressed) ;
        // adds r1, r1, #2 (taken). ARMv7-M only.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x01, 0x20, 0x0c, 0xbf, 0x01, 0x31, 0x02, 0x31],
        );
        for _ in 0..4 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.pc, 0x2000_0008);
        assert_eq!(c.regs.r[0], 1);
        assert_eq!(c.regs.r[1], 2);
        // The in-block 16-bit ADDS forms left `movs`'s flags alone.
        assert!(!c.xpsr.n() && !c.xpsr.z() && !c.xpsr.c() && !c.xpsr.v());
        assert_eq!(c.itstate, 0);
    }

    #[test]
    fn it_block_full_advance() {
        // movs r0, #5 (Z=0) ; ittt ne ; adds r1..#3 (all three arms run).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x05, 0x20, 0x1e, 0xbf, 0x01, 0x31, 0x02, 0x32, 0x03, 0x33],
        );
        for _ in 0..5 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.pc, 0x2000_000a);
        assert_eq!(c.regs.r[1], 1);
        assert_eq!(c.regs.r[2], 2);
        assert_eq!(c.regs.r[3], 3);
        assert_eq!(c.itstate, 0);
    }

    #[test]
    fn it_block_failed_condition_skips() {
        // movs r0, #5 (Z=0) ; ite eq ; adds r1, r1, #1 (skipped) ;
        // adds r1, r1, #7 (taken).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x05, 0x20, 0x0c, 0xbf, 0x01, 0x31, 0x07, 0x31],
        );
        for _ in 0..4 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.pc, 0x2000_0008);
        assert_eq!(c.regs.r[1], 7);
        assert_eq!(c.itstate, 0);
    }

    #[test]
    fn it_block_branch_clears_state() {
        // movs r0, #5 (Z=0) ; itt ne ; adds r1, r1, #1 ; bne +2 (taken,
        // clearing the IT block).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x05, 0x20, 0x1c, 0xbf, 0x01, 0x31, 0x01, 0xd1],
        );
        for _ in 0..4 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.pc, 0x2000_000c);
        assert_eq!(c.regs.r[1], 1);
        assert_eq!(c.itstate, 0);
    }

    #[test]
    fn itstate_saved_in_exception_frame() {
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0400;
        c.regs.msp = 0x2000_0400;
        c.itstate = 0x0C; // `ite eq`
        c.mem().write_word(11 * 4, 0x2000_0011).unwrap();
        c.take_exception(Exception::SvCall, 0x2000_0002).unwrap();
        assert_eq!(c.itstate, 0);
        // ITSTATE[7:2] sits in the stacked xPSR at [15:10] (here 0x0C << 8).
        let frame = c.regs.sp;
        let saved = c.mem().read_word(frame + 28).unwrap();
        assert_eq!(saved & 0xFC00, 0x0C00);
        // Returning pops the frame and revives the block.
        c.exception_return(EXC_RETURN_THREAD_MSP).unwrap();
        assert_eq!(c.itstate, 0x0C);
    }

    #[test]
    fn vfp_arith_exec() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        // vadd.f32 s0, s1, s2: 1.0 + 2.0 = 3.0.
        c.vfp.write_s(1, 0x3f80_0000);
        c.vfp.write_s(2, 0x4000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x30, 0xee, 0x81, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x4040_0000);
        // vmul.f32 s0, s1, s2: 2.0 * 3.0 = 6.0.
        c.vfp.write_s(1, 0x4000_0000);
        c.vfp.write_s(2, 0x4040_0000);
        load_inst(&mut c, 0x2000_0004, &[0x20, 0xee, 0x81, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x40c0_0000);
        // vsub.f32 s0, s1, s2: 2.0 - 3.0 = -1.0.
        load_inst(&mut c, 0x2000_0008, &[0x30, 0xee, 0xc1, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0xbf80_0000);
        // vmla.f32 s0, s1, s2 (non-fused): 2.0 * 3.0 + 4.0 = 10.0.
        c.vfp.write_s(0, 0x4080_0000);
        c.vfp.write_s(1, 0x4000_0000);
        c.vfp.write_s(2, 0x4040_0000);
        load_inst(&mut c, 0x2000_000c, &[0x00, 0xee, 0x81, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x4120_0000);
        // vfma.f32 s0, s1, s2 (fused): vn*vm + vd = 2.0*3.0 + 4.0 = 10.0, and
        // NOT vd*vm + vn (which would be 14.0).
        c.vfp.write_s(0, 0x4080_0000);
        c.vfp.write_s(1, 0x4000_0000);
        c.vfp.write_s(2, 0x4040_0000);
        load_inst(&mut c, 0x2000_0010, &[0xa0, 0xee, 0x81, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x4120_0000);
        // vfnma.f32 s0, s1, s2: -(vn*vm + vd) = -(2.0*3.0 + 4.0) = -10.0.
        c.vfp.write_s(0, 0x4080_0000);
        load_inst(&mut c, 0x2000_0014, &[0x90, 0xee, 0xc1, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0xc120_0000);
        // vfnms.f32 s0, s1, s2: vn*vm - vd = 2.0*3.0 - 4.0 = 2.0.
        c.vfp.write_s(0, 0x4080_0000);
        load_inst(&mut c, 0x2000_0018, &[0x90, 0xee, 0x81, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x4000_0000);
    }

    #[test]
    fn vfp_two_op_exec() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        // vneg.f32 s0, s1.
        c.vfp.write_s(1, 0x4000_0000); // 2.0
        load_inst(&mut c, 0x2000_0000, &[0xb1, 0xee, 0x60, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0xc000_0000);
        // vabs.f32 s0, s1.
        load_inst(&mut c, 0x2000_0004, &[0xb0, 0xee, 0x60, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x4000_0000);
        // vsqrt.f32 s0, s1: sqrt(4.0) = 2.0.
        c.vfp.write_s(1, 0x4080_0000);
        load_inst(&mut c, 0x2000_0008, &[0xb1, 0xee, 0xe0, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x4000_0000);
    }

    #[test]
    fn vfp_move_imm_exec() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        // vmov.f32 s1, #1.0.
        load_inst(&mut c, 0x2000_0000, &[0xf7, 0xee, 0x00, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(1), 0x3f80_0000);
        // vmov s1, r0 (gp -> vfp).
        c.regs.write(0, 0x1122_3344);
        load_inst(&mut c, 0x2000_0004, &[0x00, 0xee, 0x90, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(1), 0x1122_3344);
        // vmov r6, s1 (vfp -> gp).
        load_inst(&mut c, 0x2000_0008, &[0x10, 0xee, 0x90, 0x6a]);
        c.step().unwrap();
        assert_eq!(c.regs.read(6), 0x1122_3344);
        // vmov r0, r1, s1 (two single regs -> gp): S1 -> r0, S2 -> r1.
        c.vfp.write_s(1, 0x1122_3344);
        c.vfp.write_s(2, 0xaabb_ccdd);
        load_inst(&mut c, 0x2000_000c, &[0x51, 0xec, 0x30, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x1122_3344);
        assert_eq!(c.regs.read(1), 0xaabb_ccdd);
    }

    #[test]
    fn vfp_convert_exec() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        // vcvt.f32.s32 s1, s2: int -3 -> -3.0.
        c.vfp.write_s(2, 0xffff_fffd);
        load_inst(&mut c, 0x2000_0000, &[0xf8, 0xee, 0xc1, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(1), 0xc040_0000);
        // vcvt.s32.f32 s1, s2: 1.75 -> 2 (FPSCR round-to-nearest).
        c.vfp.write_s(2, 0x3fe0_0000);
        load_inst(&mut c, 0x2000_0004, &[0xfd, 0xee, 0x41, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(1), 2);
        // vcvt.u32.f32 s1, s2: 1.5 -> 2.
        c.vfp.write_s(2, 0x3fc0_0000);
        load_inst(&mut c, 0x2000_0008, &[0xfc, 0xee, 0x41, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(1), 2);
        // vcvtr.s32.f32 s1, s2: 1.75 -> 1 (round toward zero).
        c.vfp.write_s(2, 0x3fe0_0000);
        load_inst(&mut c, 0x2000_000c, &[0xfd, 0xee, 0xc1, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(1), 1);
        // NaN -> 0 with the Invalid Operation flag set.
        c.vfp.fpscr = 0;
        c.vfp.write_s(2, 0x7fc0_0000);
        load_inst(&mut c, 0x2000_0010, &[0xfd, 0xee, 0x41, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(1), 0);
        assert_ne!(c.vfp.fpscr & fpscr::IOC, 0);
    }

    #[test]
    fn vfp_compare_exec() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        // vcmp.f32 s0, s0: equal -> NZCV = 0110 (Z, C).
        c.vfp.write_s(0, 0x3f80_0000);
        c.vfp.fpscr = 0;
        load_inst(&mut c, 0x2000_0000, &[0xb4, 0xee, 0x40, 0x0a]);
        c.step().unwrap();
        assert_eq!((c.vfp.fpscr >> 28) & 0xF, 0b0110);
        // vcmp.f32 s0, s1: 1.0 vs 2.0 -> less than (NZCV = 1000).
        c.vfp.write_s(0, 0x3f80_0000);
        c.vfp.write_s(1, 0x4000_0000);
        c.vfp.fpscr = 0;
        load_inst(&mut c, 0x2000_0004, &[0xb4, 0xee, 0x60, 0x0a]);
        c.step().unwrap();
        assert_eq!((c.vfp.fpscr >> 28) & 0xF, 0b1000);
        // vcmp.f32 s0, #0.0: 1.0 vs +0 -> greater than (NZCV = 0010).
        c.vfp.write_s(0, 0x3f80_0000);
        c.vfp.fpscr = 0;
        load_inst(&mut c, 0x2000_0008, &[0xb5, 0xee, 0x40, 0x0a]);
        c.step().unwrap();
        assert_eq!((c.vfp.fpscr >> 28) & 0xF, 0b0010);
        // vcmp.f32 s0, s0 with a quiet NaN -> unordered (NZCV = 0011), and a
        // quiet compare does not raise Invalid.
        c.vfp.write_s(0, 0x7fc0_0000);
        c.vfp.fpscr = 0;
        load_inst(&mut c, 0x2000_000c, &[0xb4, 0xee, 0x40, 0x0a]);
        c.step().unwrap();
        assert_eq!((c.vfp.fpscr >> 28) & 0xF, 0b0011);
        assert_eq!(c.vfp.fpscr & fpscr::IOC, 0);
    }

    #[test]
    fn vfp_load_store_exec() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x2000_0800);
        c.mem().write_word(0x2000_0800, 0x3f80_0000).unwrap();
        // vldr s0, [r0].
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xed, 0x00, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x3f80_0000);
        // vstr s0, [r0, #8].
        load_inst(&mut c, 0x2000_0004, &[0x80, 0xed, 0x02, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_0808).unwrap(), 0x3f80_0000);
        // vpush {s0-s3} at SP.
        c.regs.sp = 0x2000_0400;
        c.vfp.write_s(0, 0x4000_0000);
        c.vfp.write_s(3, 0x4040_0000);
        load_inst(&mut c, 0x2000_0008, &[0x2d, 0xed, 0x04, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_03f0).unwrap(), 0x4000_0000);
        assert_eq!(c.mem().read_word(0x2000_03fc).unwrap(), 0x4040_0000);
        assert_eq!(c.regs.sp, 0x2000_03f0);
        // vpop {s0-s3} back into s0-s3.
        c.vfp.write_s(0, 0);
        c.vfp.write_s(3, 0);
        load_inst(&mut c, 0x2000_000c, &[0xbd, 0xec, 0x04, 0x0a]);
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x4000_0000);
        assert_eq!(c.vfp.read_s(3), 0x4040_0000);
        assert_eq!(c.regs.sp, 0x2000_0400);
    }

    #[test]
    fn fpu_ctx_preserve_on_first_fp_insn() {
        let (mut c, fpu_ctx) = armv7m_fpu_core();
        c.regs.pc = 0x2000_0000;
        // A prior exception left a lazy frame reserved (LSPACT at FPCAR); the
        // first FP instruction must write S0-S15 + FPSCR there and clear it.
        fpu_ctx.lock().unwrap().lspact = true;
        fpu_ctx.lock().unwrap().fpcar = 0x2000_0800;
        c.vfp.write_s(0, 0x3F80_0000); // s0 = 1.0
        c.vfp.write_s(1, 0x4000_0000); // s1 = 2.0
        c.vfp.write_s(2, 0x4040_0000); // s2 = 3.0
        c.vfp.write_s(15, 0x40A0_0000); // s15 = 5.0
        c.vfp.fpscr = 0x0000_0001; // IOC set
        load_inst(&mut c, 0x2000_0000, &[0x30, 0xee, 0x81, 0x0a]); // vadd.f32 s0, s1, s2
        c.step().unwrap();
        assert_eq!(c.vfp.read_s(0), 0x40A0_0000); // 2.0 + 3.0 = 5.0
        assert!(!fpu_ctx.lock().unwrap().lspact);
        // The pre-instruction bank and FPSCR were preserved into the frame.
        assert_eq!(c.mem().read_word(0x2000_0800).unwrap(), 0x3F80_0000);
        assert_eq!(c.mem().read_word(0x2000_0804).unwrap(), 0x4000_0000);
        assert_eq!(c.mem().read_word(0x2000_083c).unwrap(), 0x40A0_0000);
        assert_eq!(c.mem().read_word(0x2000_0840).unwrap(), 0x0000_0001);
        // A new context took over: FPCA set and FPSCR reloaded from FPDSCR (0).
        assert_ne!(c.control & CONTROL_FPCA, 0);
        assert_eq!(c.vfp.fpscr, 0);
    }

    #[test]
    fn fpu_ctx_context_create_on_first_fp_insn() {
        let (mut c, _) = armv7m_fpu_core();
        c.regs.pc = 0x2000_0000;
        assert_eq!(c.control & CONTROL_FPCA, 0);
        load_inst(&mut c, 0x2000_0000, &[0x30, 0xee, 0x81, 0x0a]); // vadd.f32 s0, s1, s2
        c.step().unwrap();
        // The first FP instruction of a context sets FPCA and resets FPSCR.
        assert_ne!(c.control & CONTROL_FPCA, 0);
        assert_eq!(c.vfp.fpscr, 0);
    }

    #[test]
    fn fpu_ctx_exception_lazy_reserve() {
        let (mut c, fpu_ctx) = armv7m_fpu_core();
        const SVC_HANDLER: u32 = 0x2000_0400;
        const SP: u32 = 0x2000_0800;
        c.mem().write_word(0x2c, SVC_HANDLER | 1).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        c.regs.pc = 0x2000_0100;
        // The interrupted thread owns the FP context.
        c.control |= CONTROL_FPCA;
        c.vfp.write_s(0, 0x3F80_0000);
        c.vfp.write_s(15, 0x40A0_0000);
        c.vfp.fpscr = 0x0000_0001;
        c.take_exception(Exception::SvCall, 0x2000_0102).unwrap();
        // The frame is extended to 0x68 and lazily reserved.
        assert_eq!(c.ipsr, 11);
        assert_eq!(c.regs.sp, SP - EXTENDED_FRAME_BYTES);
        assert_eq!(c.regs.pc, SVC_HANDLER);
        assert_eq!(c.regs.lr, EXC_RETURN_THREAD_MSP_FP);
        assert_eq!(c.control & CONTROL_FPCA, 0); // handler starts context-less
        let ctx = *fpu_ctx.lock().unwrap();
        assert_eq!(ctx.fpcar, SP - EXTENDED_FRAME_BYTES + 0x20);
        assert!(ctx.lspact);
        // Lazy: the registers were not touched.
        assert_eq!(c.vfp.read_s(0), 0x3F80_0000);
        assert_eq!(c.vfp.fpscr, 0x0000_0001);
    }

    #[test]
    fn fpu_ctx_exception_nonlazy_store() {
        let (mut c, fpu_ctx) = armv7m_fpu_core();
        const SVC_HANDLER: u32 = 0x2000_0400;
        const SP: u32 = 0x2000_0800;
        c.mem().write_word(0x2c, SVC_HANDLER | 1).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        c.regs.pc = 0x2000_0100;
        c.control |= CONTROL_FPCA;
        c.vfp.write_s(0, 0x3F80_0000);
        c.vfp.write_s(15, 0x40A0_0000);
        c.vfp.fpscr = 0x0000_0001;
        fpu_ctx.lock().unwrap().lspen = false;
        c.take_exception(Exception::SvCall, 0x2000_0102).unwrap();
        let frame = SP - EXTENDED_FRAME_BYTES;
        assert_eq!(c.regs.sp, frame);
        assert_eq!(c.regs.lr, EXC_RETURN_THREAD_MSP_FP);
        assert_eq!(c.control & CONTROL_FPCA, 0);
        // Non-lazy: S0-S15 + FPSCR stored now and the registers cleared.
        assert_eq!(c.mem().read_word(frame + 0x20).unwrap(), 0x3F80_0000);
        assert_eq!(c.mem().read_word(frame + 0x5c).unwrap(), 0x40A0_0000);
        assert_eq!(c.mem().read_word(frame + 0x60).unwrap(), 0x0000_0001);
        assert_eq!(c.vfp.read_s(0), 0);
        assert_eq!(c.vfp.read_s(15), 0);
        assert_eq!(c.vfp.fpscr, 0);
        let ctx = *fpu_ctx.lock().unwrap();
        assert!(!ctx.lspact);
        assert_eq!(ctx.fpcar, 0);
    }

    #[test]
    fn fpu_ctx_return_restores() {
        let (mut c, _) = armv7m_fpu_core();
        const SP: u32 = 0x2000_0800;
        let frame = SP - EXTENDED_FRAME_BYTES;
        // A pre-stacked extended frame with FP contents.
        c.mem().write_word(frame + 0x00, 0x1111_1111).unwrap();
        c.mem().write_word(frame + 0x04, 0x2222_2222).unwrap();
        c.mem().write_word(frame + 0x08, 0x3333_3333).unwrap();
        c.mem().write_word(frame + 0x0c, 0x4444_4444).unwrap();
        c.mem().write_word(frame + 0x10, 0x5555_5555).unwrap();
        c.mem().write_word(frame + 0x14, 0x6666_6666).unwrap();
        c.mem().write_word(frame + 0x18, 0x2000_0102).unwrap();
        c.mem().write_word(frame + 0x1c, XPSR_T).unwrap();
        for (i, w) in [0x3F80_0000u32, 0x4000_0000, 0x4040_0000, 0x4080_0000]
            .into_iter()
            .enumerate()
        {
            c.mem().write_word(frame + 0x20 + i as u32 * 4, w).unwrap();
        }
        c.mem().write_word(frame + 0x60, 0x0000_0001).unwrap();
        c.regs.sp = frame;
        c.regs.msp = frame;
        c.regs.pc = 0x2000_0400; // handler code
        c.ipsr = 11;
        c.control &= !CONTROL_FPCA;
        c.exception_return(EXC_RETURN_THREAD_MSP_FP).unwrap();
        // S0-S15 + FPSCR came back from the frame, FPCA is re-owned, and the
        // extended frame (0x68) was fully popped.
        assert_eq!(c.vfp.read_s(0), 0x3F80_0000);
        assert_eq!(c.vfp.read_s(1), 0x4000_0000);
        assert_eq!(c.vfp.read_s(3), 0x4080_0000);
        assert_eq!(c.vfp.fpscr, 0x0000_0001);
        assert_ne!(c.control & CONTROL_FPCA, 0);
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.sp, SP);
    }

    #[test]
    fn fpu_ctx_return_abandons_lazy() {
        let (mut c, fpu_ctx) = armv7m_fpu_core();
        const SP: u32 = 0x2000_0800;
        let frame = SP - EXTENDED_FRAME_BYTES;
        c.mem().write_word(frame + 0x1c, XPSR_T).unwrap();
        c.regs.sp = frame;
        c.regs.msp = frame;
        c.regs.pc = 0x2000_0400; // handler code
        c.ipsr = 11;
        c.control &= !CONTROL_FPCA;
        c.vfp.write_s(0, 0x40A0_0000);
        c.vfp.fpscr = 0x0000_0001;
        {
            let mut ctx = fpu_ctx.lock().unwrap();
            ctx.lspact = true;
            ctx.fpcar = frame + 0x20;
        }
        c.exception_return(EXC_RETURN_THREAD_MSP_FP).unwrap();
        // The lazy frame is abandoned: LSPACT cleared, the FPU still holds the
        // pre-exception values, and the returning thread re-owns the context.
        assert!(!fpu_ctx.lock().unwrap().lspact);
        assert_eq!(c.vfp.read_s(0), 0x40A0_0000);
        assert_eq!(c.vfp.fpscr, 0x0000_0001);
        assert_ne!(c.control & CONTROL_FPCA, 0);
    }

    /// An ARMv7-M core over a `SystemMemory` with the SCB present, so `SHCSR`
    /// and `CPACR` windows gate fault and FP behavior.
    fn armv7m_sys_core() -> Core<crate::mem::system::SystemMemory<crate::mem::flat::FlatMemory>> {
        let backing = crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        );
        Core::armv7m(crate::mem::system::SystemMemory::new(backing))
    }

    #[test]
    fn usage_fault_taken_as_distinct_exception_when_enabled() {
        const CODE: u32 = 0x2000_0100;
        const UF_HANDLER: u32 = 0x2000_0400;
        const SP: u32 = 0x2000_0800;
        let mut c = armv7m_sys_core();
        // UsageFault vector 6 at 0x18; HardFault vector 3 at 0xc.
        c.mem().write_word(6 * 4, UF_HANDLER | 1).unwrap();
        c.mem().write_word(3 * 4, UF_HANDLER | 1).unwrap();
        // SHCSR.USGFAULTENA (bit 18) enables a distinct UsageFault.
        c.mem().write_word(0xE000_ED24, 0x0004_0000).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        load_inst(&mut c, CODE, &[0x00, 0xde]); // udf -> UsageFault
        load_inst(&mut c, UF_HANDLER, &[0x70, 0x47]); // bx lr
        c.regs.pc = CODE;
        c.step().unwrap();
        // The distinct UsageFault is taken: ipsr 6, its own vector, and the
        // standard EXC_RETURN. CFSR records UNDEFINSTR without FORCED.
        assert_eq!(c.ipsr, 6);
        assert_eq!(c.regs.pc, UF_HANDLER);
        assert_eq!(c.regs.lr, EXC_RETURN_THREAD_MSP);
        assert_eq!(c.mem().read_word(0xE000_ED28).unwrap(), 0x0001_0000);
        assert_eq!(c.mem().read_word(0xE000_ED2C).unwrap(), 0);
        // The stacked return address is the faulting instruction.
        assert_eq!(c.mem().read_word(SP - FRAME_BYTES + 24).unwrap(), CODE);
    }

    #[test]
    fn usage_fault_disabled_escalates_to_hardfault() {
        const CODE: u32 = 0x2000_0100;
        const HF_HANDLER: u32 = 0x2000_0600;
        const SP: u32 = 0x2000_0800;
        let mut c = armv7m_sys_core();
        c.mem().write_word(3 * 4, HF_HANDLER | 1).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        load_inst(&mut c, CODE, &[0x00, 0xde]); // udf -> HardFault (USGFAULTENA=0)
        c.regs.pc = CODE;
        c.step().unwrap();
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HF_HANDLER);
        // The fold records UNDEFINSTR plus the FORCED escalation bit.
        assert_eq!(c.mem().read_word(0xE000_ED28).unwrap(), 0x0001_0000);
        assert_eq!(c.mem().read_word(0xE000_ED2C).unwrap(), 0x4000_0000);
    }

    #[test]
    fn usage_fault_in_handler_escalates_to_hardfault() {
        const CODE: u32 = 0x2000_0100;
        const UF_HANDLER: u32 = 0x2000_0400;
        const HF_HANDLER: u32 = 0x2000_0600;
        const SP: u32 = 0x2000_0800;
        let mut c = armv7m_sys_core();
        c.mem().write_word(6 * 4, UF_HANDLER | 1).unwrap();
        c.mem().write_word(3 * 4, HF_HANDLER | 1).unwrap();
        c.mem().write_word(0xE000_ED24, 0x0004_0000).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        load_inst(&mut c, CODE, &[0x00, 0xde]); // udf -> UsageFault
        load_inst(&mut c, UF_HANDLER, &[0x00, 0xde]); // udf inside the handler
        load_inst(&mut c, HF_HANDLER, &[0x70, 0x47]); // bx lr
        c.regs.pc = CODE;
        c.step().unwrap(); // take UsageFault
        assert_eq!(c.ipsr, 6);
        c.step().unwrap(); // the UDEF in the handler escalates to HardFault
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HF_HANDLER);
    }

    #[test]
    fn cpacr_denied_vfp_raises_nocp_usage_fault() {
        const CODE: u32 = 0x2000_0100;
        const UF_HANDLER: u32 = 0x2000_0400;
        const SP: u32 = 0x2000_0800;
        let mut c = armv7m_sys_core();
        c.mem().write_word(6 * 4, UF_HANDLER | 1).unwrap();
        c.mem().write_word(0xE000_ED24, 0x0004_0000).unwrap();
        // CPACR resets to zero: FP access is denied until firmware enables it.
        c.regs.sp = SP;
        c.regs.msp = SP;
        c.regs.pc = CODE;
        load_inst(&mut c, CODE, &[0x00, 0xee, 0x90, 0x0a]); // vmov s1, r0
        load_inst(&mut c, UF_HANDLER, &[0x70, 0x47]); // bx lr
        c.step().unwrap();
        assert_eq!(c.ipsr, 6);
        assert_eq!(c.regs.pc, UF_HANDLER);
        // NOCP is the recorded UsageFault cause.
        assert_eq!(c.mem().read_word(0xE000_ED28).unwrap(), 0x0002_0000);
        // The FP context never activates on the denied access.
        assert_eq!(c.control & CONTROL_FPCA, 0);
    }

    #[test]
    fn cpacr_full_access_executes_vfp() {
        const CODE: u32 = 0x2000_0100;
        let mut c = armv7m_sys_core();
        c.mem().write_word(6 * 4, CODE | 1).unwrap();
        // Full access to CP10/CP11, as firmware enabling the FPU writes.
        c.mem().write_word(0xE000_ED88, 0x00F0_0000).unwrap();
        c.regs.write(0, 0x1122_3344);
        c.regs.pc = CODE;
        load_inst(&mut c, CODE, &[0x00, 0xee, 0x90, 0x0a]); // vmov s1, r0
        c.step().unwrap();
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.vfp.read_s(1), 0x1122_3344);
    }

    /// Configure region 0 to cover `0x2000_0000` for 2 KiB with rw/rw access
    /// (`RASR = 0x0300_0015`) and enable the MPU, leaving everything above the
    /// region (and below the background) unmapped.
    fn enable_mpu_region0(
        mem: &mut crate::mem::system::SystemMemory<crate::mem::flat::FlatMemory>,
    ) {
        mem.write_word(0xE000_ED94, 0x1).unwrap(); // MPU_CTRL: ENABLE
        mem.write_word(0xE000_ED98, 0).unwrap(); // MPU_RNR: region 0
        mem.write_word(0xE000_ED9C, 0x2000_0000).unwrap(); // MPU_RBAR
        mem.write_word(0xE000_EDA0, 0x0300_0015).unwrap(); // MPU_RASR
    }

    #[test]
    fn mpu_data_violation_escalates_to_hardfault() {
        const CODE: u32 = 0x2000_0100;
        const DATA: u32 = 0x2000_0800; // in RAM, outside region 0
        const HF_HANDLER: u32 = 0x2000_0600;
        const SP: u32 = 0x2000_0C00;
        let mut c = armv7m_sys_core();
        c.mem().write_word(3 * 4, HF_HANDLER | 1).unwrap();
        enable_mpu_region0(c.mem());
        c.regs.sp = SP;
        c.regs.msp = SP;
        c.regs.write(1, DATA);
        c.regs.pc = CODE;
        load_inst(&mut c, CODE, &[0x08, 0x68]); // ldr r0, [r1]
        load_inst(&mut c, HF_HANDLER, &[0x70, 0x47]); // bx lr
        c.step().unwrap();
        // The data access to the unmapped address is a DACCVIOL escalated to
        // HardFault with the faulting address in MMFAR.
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HF_HANDLER);
        assert_eq!(c.mem().read_word(0xE000_ED28).unwrap(), 0x0000_0082); // DACCVIOL|MMARVALID
        assert_eq!(c.mem().read_word(0xE000_ED2C).unwrap(), 0x4000_0000); // FORCED
        assert_eq!(c.mem().read_word(0xE000_ED34).unwrap(), DATA);
        // The stacked return address is the faulting instruction.
        assert_eq!(c.mem().read_word(SP - FRAME_BYTES + 24).unwrap(), CODE);
    }

    #[test]
    fn mpu_fetch_violation_escalates_to_hardfault() {
        const CODE: u32 = 0x2000_0800; // outside region 0
        const HF_HANDLER: u32 = 0x2000_0600;
        const SP: u32 = 0x2000_0C00;
        let mut c = armv7m_sys_core();
        c.mem().write_word(3 * 4, HF_HANDLER | 1).unwrap();
        enable_mpu_region0(c.mem());
        c.regs.sp = SP;
        c.regs.msp = SP;
        c.regs.pc = CODE;
        load_inst(&mut c, CODE, &[0x70, 0x47]); // bx lr
        load_inst(&mut c, HF_HANDLER, &[0x70, 0x47]); // bx lr
        c.step().unwrap();
        assert_eq!(c.ipsr, 3);
        assert_eq!(c.regs.pc, HF_HANDLER);
        // An instruction access violation: IACCVIOL|MMARVALID, MMFAR = CODE.
        assert_eq!(c.mem().read_word(0xE000_ED28).unwrap(), 0x0000_0081);
        assert_eq!(c.mem().read_word(0xE000_ED34).unwrap(), CODE);
    }

    #[test]
    fn mpu_disabled_does_not_disturb_execution() {
        const CODE: u32 = 0x2000_0100;
        let mut c = armv7m_sys_core();
        c.regs.write(0, 7);
        c.regs.pc = CODE;
        load_inst(&mut c, CODE, &[0x00, 0x68]); // ldr r0, [r0]
        c.step().unwrap();
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.mem().read_word(0xE000_ED28).unwrap(), 0);
    }
}
