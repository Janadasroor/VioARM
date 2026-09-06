//! Emitter skeleton (P2-1): execution context, register map,
//! prologue/epilogue, exit codes, and the callout ABI.
//!
//! # Execution model
//!
//! Guest state is NEVER copied: emitted code reads and writes the live
//! `Core` state in place (one source of truth — differential tests compare
//! `Core` directly, and there is no writeback path to forget on side
//! exits). Addresses are computed in safe Rust (`offset_of!`, `repr(C)`
//! context, `repr(transparent)` flags) and baked in as immediates; the
//! machine code itself only dereferences them.
//!
//! # Register map (fixed, no coloring — P2-1 decision)
//!
//! A fixed subset of guest registers is pinned in callee-saved host
//! registers across the block. Callee-saved is load-bearing: callouts are
//! ordinary C calls, so pinned values survive them without flushing
//! (proven by the callout test). The rest lives in `Core` and is accessed
//! directly.
//!
//! | guest | x86-64 | AArch64 | notes |
//! |---|---|---|
//! | r0 | rbx | x19 | Thumb workhorses |
//! | r1 | r12 | x20 | |
//! | r2 | r13 | x21 | |
//! | r3 | r14 | x22 | |
//! | xPSR word | r15 | x23 | full word; templates mask NZCV |
//!
//! `rbp` is deliberately NOT pinned (unwind-friendly frames); `r4`–`r12`,
//! `sp`, `lr`, `pc` are memory-backed in P2-1 (sp/lr/pc get template rules
//! in P2-2). Scratch: `rax`/`rcx`/`rdx`/`rsi` (x64), `x9`–`x15` (AArch64) —
//! caller-saved on both ABIs, free for templates to clobber.
//!
//! # Frame discipline
//!
//! The prologue saves the pinned set plus a context spill slot; sizes are
//! chosen so `rsp`/`sp` is 16-aligned before any `call` (x64: 5 pushes +
//! 16 slot = 56; AArch64: 64-byte frame). Templates
//! (P2-2) MUST keep the spill slot at `[rsp]`/`[sp,#48]` intact and the
//! stack balanced at every exit: the epilogue and callout sequences
//! address the slot absolutely.
//!
//! # Exit codes
//!
//! Epilogues return `u64 = (reason << 32) | next_pc` (`RAX`/`X0`).
//! P2-1 implements `FALL`; `BRANCH` and the reserved P2-2 reasons share
//! the packing.
//!
//! # Callout ABI
//!
//! Callouts are `extern "C" fn`s taking plain `u64`s (the context pointer
//! reloaded from the spill slot when state access is needed) and returning
//! `u64`. Defining them, taking their address (`as usize`, then `as i64`
//! for the mov-imm), and emitting `call` are all safe operations — no
//! `unsafe` enters `vioarm-core`; only `vioarm-jit-call` jumps. Two rules
//! for P2-2 callout authors:
//!
//! 1. **Pinned regs are NOT flushed around callouts** (they survive in
//!    callee-saved registers); flush only when the callout must observe a
//!    pinned value through `Core` memory — then reload after.
//! 2. **Callouts must not unwind** (no panic across a JIT frame: no unwind
//!    tables cover emitted code). P2-2 enforces this at the callout
//!    boundary.
//!
//! 64-bit hosts only (pointer-sized immediates throughout).

use std::sync::Arc;

use dynasmrt::{dynasm, AssemblyOffset, DynasmApi, ExecutableBuffer};

use crate::cpu::registers::{Registers, Xpsr};

/// Exit reason: fell off the block end (next block chainable at `next_pc`).
pub const EXIT_FALL: u32 = 0;
/// Exit reason: control flow diverged inside the block (branch taken,
/// call, return — `next_pc` is the true PC).
pub const EXIT_BRANCH: u32 = 1;
// Reserved for P2-2: CALLOUT_RESULT = 2, SLEEP = 3, MMIO_TOUCH = 4.

/// Pack an epilogue exit code.
#[must_use]
pub const fn exit_code(reason: u32, next_pc: u32) -> u64 {
    ((reason as u64) << 32) | next_pc as u64
}

/// Unpack an exit reason.
#[must_use]
pub const fn exit_reason(code: u64) -> u32 {
    (code >> 32) as u32
}

/// Unpack the exit PC.
#[must_use]
pub const fn exit_pc(code: u64) -> u32 {
    code as u32
}

/// Entry context for emitted code, passed as the single `call1` argument.
/// `repr(C)`: the emitter reads these offsets directly.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct NativeCtx {
    /// Live guest registers (zero-copy source of truth).
    pub regs: *mut Registers,
    /// Live flags word (`repr(transparent)` u32).
    pub xpsr: *mut Xpsr,
}

/// A committed native block: executable code plus its entry offset.
/// `Clone` shares the mapping (code is immutable after `finalize`; cache
/// eviction drops the last `Arc` and unmaps).
#[derive(Clone, Debug)]
pub struct NativeBlock {
    code: Arc<ExecutableBuffer>,
    entry: AssemblyOffset,
}

impl NativeBlock {
    /// Wrap a committed buffer (entry at `entry`).
    #[must_use]
    pub fn new(code: ExecutableBuffer, entry: AssemblyOffset) -> Self {
        Self {
            code: Arc::new(code),
            entry,
        }
    }

    /// Raw entry pointer for `vioarm-jit-call` (see its SAFETY CASE: only
    ///_entry points produced here, with the matching ABI, are ever called).
    #[must_use]
    pub fn entry_ptr(&self) -> *const u8 {
        self.code.ptr(self.entry)
    }

    /// Backing mapping size (proves commitment in tests).
    #[must_use]
    pub fn size(&self) -> usize {
        self.code.size()
    }

    /// Live references to the mapping (cache-pressure observability for
    /// P2-2, and the leak probe in the cache tests: eviction must return
    /// a held clone's count to 1).
    #[must_use]
    pub fn strong_count(&self) -> usize {
        Arc::strong_count(&self.code)
    }
}

/// Byte offset of `regs.r[i]` from the regs base (array contiguity is
/// guaranteed; the field offset itself comes from `offset_of!`, so no
/// `repr(C)` assumption on `Registers`).
fn reg_offset(i: usize) -> i32 {
    (core::mem::offset_of!(Registers, r) + i * 4) as i32
}

#[cfg(target_arch = "x86_64")]
fn emit_prologue(ops: &mut dynasmrt::x64::Assembler) {
    let ctx_regs = core::mem::offset_of!(NativeCtx, regs) as i32;
    let ctx_xpsr = core::mem::offset_of!(NativeCtx, xpsr) as i32;
    let (o0, o1, o2, o3) = (reg_offset(0), reg_offset(1), reg_offset(2), reg_offset(3));
    dynasm!(ops
        ; .arch x64
        // Save pinned set (5 pushes: entry rsp%16==8 -> aligned).
        ; push rbx
        ; push r12
        ; push r13
        ; push r14
        ; push r15
        // Context spill slot: 16 bytes (8 used + 8 pad) so the frame is
        // 56 total and rsp stays 16-aligned for calls (5 pushes = 40).
        ; sub rsp, 16
        ; mov [rsp], rdi
        // Pin r0-r3 (32-bit loads zero-extend for free).
        ; mov rax, [rdi + ctx_regs]
        ; mov ebx, [rax + o0]
        ; mov r12d, [rax + o1]
        ; mov r13d, [rax + o2]
        ; mov r14d, [rax + o3]
        // Pin flags word.
        ; mov rcx, [rdi + ctx_xpsr]
        ; mov r15d, [rcx]
    );
}

#[cfg(target_arch = "x86_64")]
fn emit_epilogue(ops: &mut dynasmrt::x64::Assembler, reason: u32, next_pc: u32) {
    let ctx_regs = core::mem::offset_of!(NativeCtx, regs) as i32;
    let ctx_xpsr = core::mem::offset_of!(NativeCtx, xpsr) as i32;
    let (o0, o1, o2, o3) = (reg_offset(0), reg_offset(1), reg_offset(2), reg_offset(3));
    let code = exit_code(reason, next_pc) as i64;
    dynasm!(ops
        ; .arch x64
        // Write pinned set back.
        ; mov rax, [rsp]
        ; mov rcx, [rax + ctx_regs]
        ; mov [rcx + o0], ebx
        ; mov [rcx + o1], r12d
        ; mov [rcx + o2], r13d
        ; mov [rcx + o3], r14d
        ; mov rcx, [rax + ctx_xpsr]
        ; mov [rcx], r15d
        // Exit code + teardown.
        ; mov rax, QWORD code
        ; add rsp, 16
        ; pop r15
        ; pop r14
        ; pop r13
        ; pop r12
        ; pop rbx
        ; ret
    );
}

/// Assemble a trivial block (prologue + `FALL` epilogue): the P2-1
/// end-to-end proof that pinning round-trips `Core` state untouched.
#[cfg(target_arch = "x86_64")]
pub(crate) fn assemble_trivial(
    reason: u32,
    next_pc: u32,
) -> Result<NativeBlock, crate::jit::EmitError> {
    let mut ops = dynasmrt::x64::Assembler::new()?;
    emit_prologue(&mut ops);
    emit_epilogue(&mut ops, reason, next_pc);
    ops.finalize()
        .map(|buf| NativeBlock::new(buf, AssemblyOffset(0)))
        .map_err(|_| crate::jit::EmitError::Finalize)
}

#[cfg(target_arch = "aarch64")]
fn emit_prologue(ops: &mut dynasmrt::aarch64::Assembler) {
    // AArch64 displacements take `u32` (x64 takes `i32`): convert once here.
    let ctx_regs = core::mem::offset_of!(NativeCtx, regs) as u32;
    let ctx_xpsr = core::mem::offset_of!(NativeCtx, xpsr) as u32;
    let (o0, o1, o2, o3) = (
        reg_offset(0) as u32,
        reg_offset(1) as u32,
        reg_offset(2) as u32,
        reg_offset(3) as u32,
    );
    dynasm!(ops
        ; .arch aarch64
        // 64-byte frame (16-aligned): pinned saves + ctx spill at [sp,#48].
        ; sub sp, sp, #64
        ; stp x19, x20, [sp]
        ; stp x21, x22, [sp, #16]
        ; stp x23, x24, [sp, #32]
        ; str x0, [sp, #48]
        // Pin r0-r3 + flags (x0 = ctx).
        ; ldr x9, [x0, ctx_regs]
        ; ldr w19, [x9, o0]
        ; ldr w20, [x9, o1]
        ; ldr w21, [x9, o2]
        ; ldr w22, [x9, o3]
        ; ldr x9, [x0, ctx_xpsr]
        ; ldr w23, [x9]
    );
}

#[cfg(target_arch = "aarch64")]
fn emit_epilogue(ops: &mut dynasmrt::aarch64::Assembler, reason: u32, next_pc: u32) {
    let ctx_regs = core::mem::offset_of!(NativeCtx, regs) as u32;
    let ctx_xpsr = core::mem::offset_of!(NativeCtx, xpsr) as u32;
    let (o0, o1, o2, o3) = (
        reg_offset(0) as u32,
        reg_offset(1) as u32,
        reg_offset(2) as u32,
        reg_offset(3) as u32,
    );
    let code = exit_code(reason, next_pc);
    let (c0, c1, c2, c3) = (
        (code & 0xFFFF) as u32,
        ((code >> 16) & 0xFFFF) as u32,
        ((code >> 32) & 0xFFFF) as u32,
        ((code >> 48) & 0xFFFF) as u32,
    );
    dynasm!(ops
        ; .arch aarch64
        // Write pinned set back.
        ; ldr x9, [sp, #48]
        ; ldr x10, [x9, ctx_regs]
        ; str w19, [x10, o0]
        ; str w20, [x10, o1]
        ; str w21, [x10, o2]
        ; str w22, [x10, o3]
        ; ldr x10, [x9, ctx_xpsr]
        ; str w23, [x10]
        // Exit code + teardown.
        ; movz x0, c0, lsl #0
        ; movk x0, c1, lsl #16
        ; movk x0, c2, lsl #32
        ; movk x0, c3, lsl #48
        ; ldp x23, x24, [sp, #32]
        ; ldp x21, x22, [sp, #16]
        ; ldp x19, x20, [sp]
        ; add sp, sp, #64
        ; ret
    );
}

/// Assemble a trivial block (prologue + epilogue): the P2-1 end-to-end
/// proof that pinning round-trips `Core` state untouched.
#[cfg(target_arch = "aarch64")]
pub(crate) fn assemble_trivial(
    reason: u32,
    next_pc: u32,
) -> Result<NativeBlock, crate::jit::EmitError> {
    let mut ops = dynasmrt::aarch64::Assembler::new()?;
    emit_prologue(&mut ops);
    emit_epilogue(&mut ops, reason, next_pc);
    ops.finalize()
        .map(|buf| NativeBlock::new(buf, AssemblyOffset(0)))
        .map_err(|_| crate::jit::EmitError::Finalize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::registers::Xpsr;

    #[test]
    fn exit_code_packing_round_trips() {
        assert_eq!(exit_code(EXIT_FALL, 0x1000), 0x1000);
        assert_eq!(exit_code(EXIT_BRANCH, 0x2000), (1 << 32) | 0x2000);
        assert_eq!(exit_reason(exit_code(EXIT_BRANCH, 0x2000)), EXIT_BRANCH);
        assert_eq!(exit_pc(exit_code(EXIT_BRANCH, 0x2000)), 0x2000);
    }

    #[test]
    fn reg_offsets_match_layout() {
        // Soundness spot-check: computed offsets address the right words.
        // Pure address arithmetic (no dereference) — no `unsafe` needed.
        let regs = Registers {
            r: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
            sp: 0,
            msp: 0,
            psp: 0,
            lr: 0,
            pc: 0,
        };
        let base = &regs as *const Registers as usize;
        for i in 0..13 {
            let expect = &regs.r[i] as *const u32 as usize;
            assert_eq!(base + reg_offset(i) as usize, expect, "r{i} offset");
        }
    }

    /// Distinctive register/flag values for round-trip tests.
    fn patterned_regs() -> Registers {
        Registers {
            r: [
                0xDEAD_BEEF,
                0xCAFE_0001,
                0x1234_5678,
                0xABCD_EF01,
                0x0BAD_F00D,
                0xFEED_FACE,
                0x1337_1337,
                0x8000_0001,
                0x7FFF_FFFE,
                0x0102_0304,
                0x1020_3040,
                0xFFF0_0FFF,
                0x0000_0000,
            ],
            sp: 0x2000_1000,
            msp: 0x2000_1000,
            psp: 0x2000_2000,
            lr: 0xFFFF_FFF9,
            pc: 0x0800_1000,
        }
    }

    #[test]
    fn trivial_block_round_trips_state_and_exit_code() {
        let mut regs = patterned_regs();
        let before = regs;
        let mut xpsr = Xpsr::default();
        xpsr.set(0xF000_0000); // NZCV set
        let mut ctx = Box::new(NativeCtx {
            regs: &mut regs,
            xpsr: &mut xpsr,
        });
        let block = assemble_trivial(EXIT_FALL, 0x1000).expect("trivial block");
        assert!(block.size() > 0);
        let code = vioarm_jit_call::call1(block.entry_ptr(), &mut *ctx as *mut NativeCtx as u64);
        assert_eq!(exit_reason(code), EXIT_FALL);
        assert_eq!(exit_pc(code), 0x1000);
        // Pinning round-tripped the hot state untouched (any offset or
        // save/restore bug corrupts these); the rest was never touched.
        assert_eq!(regs.r[0], before.r[0], "r0");
        assert_eq!(regs.r[1], before.r[1], "r1");
        assert_eq!(regs.r[2], before.r[2], "r2");
        assert_eq!(regs.r[3], before.r[3], "r3");
        assert_eq!(regs, before, "full regs");
    }

    static RECORDED: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

    extern "C" fn probe(arg: u64) -> u64 {
        RECORDED.store(arg, core::sync::atomic::Ordering::SeqCst);
        arg
    }

    /// Emit prologue + one callout (ctx pointer as the argument) + `FALL`
    /// epilogue: proves call mechanics and callee-saved pin survival
    /// across a real compiled-Rust call.
    #[cfg(target_arch = "x86_64")]
    fn assemble_callout_probe() -> Result<NativeBlock, crate::jit::EmitError> {
        let mut ops = dynasmrt::x64::Assembler::new()?;
        emit_prologue(&mut ops);
        let addr = probe as *const () as usize as i64;
        dynasm!(ops
            ; .arch x64
            ; mov rdi, [rsp]
            ; mov rax, QWORD addr
            ; call rax
        );
        emit_epilogue(&mut ops, EXIT_FALL, 0x2000);
        ops.finalize()
            .map(|buf| NativeBlock::new(buf, AssemblyOffset(0)))
            .map_err(|_| crate::jit::EmitError::Finalize)
    }

    /// Emit prologue + one callout (ctx pointer as the argument) + `FALL`
    /// epilogue: proves call mechanics and callee-saved pin survival
    /// across a real compiled-Rust call.
    #[cfg(target_arch = "aarch64")]
    fn assemble_callout_probe() -> Result<NativeBlock, crate::jit::EmitError> {
        let mut ops = dynasmrt::aarch64::Assembler::new()?;
        emit_prologue(&mut ops);
        let addr = probe as *const () as usize as u64;
        let (a0, a1, a2, a3) = (
            (addr & 0xFFFF) as u32,
            ((addr >> 16) & 0xFFFF) as u32,
            ((addr >> 32) & 0xFFFF) as u32,
            ((addr >> 48) & 0xFFFF) as u32,
        );
        dynasm!(ops
            ; .arch aarch64
            ; ldr x0, [sp, #48]
            ; movz x9, a0, lsl #0
            ; movk x9, a1, lsl #16
            ; movk x9, a2, lsl #32
            ; movk x9, a3, lsl #48
            ; blr x9
        );
        emit_epilogue(&mut ops, EXIT_FALL, 0x2000);
        ops.finalize()
            .map(|buf| NativeBlock::new(buf, AssemblyOffset(0)))
            .map_err(|_| crate::jit::EmitError::Finalize)
    }

    #[test]
    fn callout_receives_ctx_and_preserves_pins() {
        use core::sync::atomic::Ordering::SeqCst;
        let mut regs = patterned_regs();
        let before = regs;
        let mut xpsr = Xpsr::default();
        xpsr.set(0xF000_0000);
        let mut ctx = Box::new(NativeCtx {
            regs: &mut regs,
            xpsr: &mut xpsr,
        });
        let want = &mut *ctx as *mut NativeCtx as u64;
        RECORDED.store(0, SeqCst);
        let block = assemble_callout_probe().expect("callout block");
        let code = vioarm_jit_call::call1(block.entry_ptr(), &mut *ctx as *mut NativeCtx as u64);
        assert_eq!(RECORDED.load(SeqCst), want, "ctx pointer delivered");
        assert_eq!(exit_reason(code), EXIT_FALL);
        assert_eq!(exit_pc(code), 0x2000);
        // The callout is a real Rust call (clobbers caller-saved): intact
        // pins prove the callee-saved strategy.
        assert_eq!(regs, before, "pinned regs survive callout");
    }
}
