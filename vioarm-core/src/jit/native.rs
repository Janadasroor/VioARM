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
//! 16 slot = 56; AArch64: 64-byte frame). Templates (P2-2) MUST keep the
//! spill slot at `[rsp]`/`[sp,#48]` intact and the stack balanced at every
//! exit: the epilogue and callout sequences address the slot absolutely.
//! The remaining pad bytes (`[rsp+8]` / `[sp,#56]`) are template scratch
//! (e.g. shift big-case value spills), clobbered freely between ops.
//!
//! Template-author rule (load-bearing): memory-path loads/stores reload
//! the base through `rsi`/`x9`, killing any value carried in the low half.
//! Never carry values in `esi`/`w9` across a mem-path access — recompute
//! after, or spill to the pad slot first. Terminator templates (P2-2)
//! must additionally preserve the pinned set: scratch is `eax`/`ecx`/
//! `edx`/`esi` (`w0`/`w1`/`w9`–`w11`) only, and dynamic exits deliver the
//! target PC in `edx`/`w0` for the dynamic epilogue. Pointers (ctx, TLB,
//! epoch, host bases) ALWAYS use 64-bit moves (`rdi`, never `edi` —
//! truncating a 48-bit heap address faults; caught by differential).
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

// Label definition/binding for the chaining stubs (per-arch emitters).
#[cfg(any(target_arch = "x86_64", target_arch = "aarch64"))]
use dynasmrt::DynasmLabelApi;

use super::tlb::Tlb;
use crate::cpu::registers::{Registers, Xpsr};

/// Exit reason: fell off the block end (next block chainable at `next_pc`).
pub const EXIT_FALL: u32 = 0;
/// Exit reason: control flow diverged inside the block (branch taken,
/// call, return — `next_pc` is the true PC).
pub const EXIT_BRANCH: u32 = 1;
/// Exit reason: memory side exit (guard/TLB miss, page cross, unaligned-
/// with-trap, bitband): `next_pc` carries the faulting op's PC. The
/// driver re-executes that op through the interpreter (identical MMIO and
/// fault behavior) and attempts a TLB fill from the reported address.
/// P2-4 generalizes the driver path (not the code): complex-tail branches
/// (`BX`/`BLX` to magic targets) also exit here with their op PC; the
/// driver re-executes them with no fill (`mem_access` is `None` there).
/// Reserved: CALLOUT_RESULT = 3, SLEEP = 4 (P2-6).
pub const EXIT_MMIO: u32 = 2;
/// Exit reason: architected halt inside native code (P2-4: `BX`/`BLX` to
/// an even target — `check_branch` stops with `NonThumbTarget`). The
/// driver flushes chained accounting, then reports
/// `Err(HaltCause::NonThumbTarget)` exactly like the stepper.
pub const EXIT_HALT: u32 = 5;
/// Exit reason: native multi-load to a magic target (P2-5 `POP {pc}` /
/// `LDM pc`): the transfers already happened natively, so the driver
/// only runs the branch dispatch (`branch_native` — no re-execution, no
/// TLB fill). `next_pc` carries the op PC; `fault_addr` carries the
/// loaded target.
pub const EXIT_POP_BRANCH: u32 = 6;

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
    /// Page translation cache (runner-owned, stable address).
    pub tlb: *mut Tlb,
    /// Effective privilege for TLB tags this execution (M2: 0/1).
    pub privileged: u32,
    /// Bus write-generation counter (bumped by fast-path stores).
    /// Always valid when a native block containing stores executes
    /// (untracked buses keep such blocks interpreted).
    pub write_gen: *mut u64,
    /// Faulting guest address, stored by templates on side exits (the
    /// driver fills the TLB from it; size/direction come from the op).
    /// Pop-branch exits (P2-5) reuse it for the loaded branch target.
    pub fault_addr: u32,
    /// Live IT state (P2-5, S4): predicated blocks store their statically
    /// simulated exit value here. Always valid on native entry (the driver
    /// points it at the core field; the harness uses scratch).
    pub itstate: *mut u8,
    /// Chaining budgets, set by the driver per `call1` (P2-4): each
    /// chained-block entry subtracts its block size and stops the chain
    /// (`FALL` to its own PC) at zero. Bounds native execution within one
    /// call so quanta stay preemptible (no unbounded native loops).
    pub budget_ops: i64,
    /// Chaining block budget, set alongside `budget_ops` (same stop).
    pub budget_blocks: i64,
    /// Link-epoch value for this call (copied from the runner): chained
    /// edges compare their slot's stamped epoch against this.
    pub link_epoch: u64,
    /// Chained accounting accumulators (P2-4): each chained entry adds its
    /// block's exact driver-equivalent contribution; the driver flushes
    /// them into `Core` after the call returns (all exit reasons).
    pub acc_ops: u64,
    /// Retired-op cycles from chained blocks (`cycles += insns` each).
    pub acc_cycles: u64,
    /// `cyccnt` estimates from chained blocks (per-block D7 each).
    pub acc_cyccnt: u64,
    /// Chained blocks executed (beyond the entered block).
    pub acc_blocks: u64,
}

impl Default for NativeCtx {
    fn default() -> Self {
        Self {
            regs: core::ptr::null_mut(),
            xpsr: core::ptr::null_mut(),
            tlb: core::ptr::null_mut(),
            privileged: 0,
            write_gen: core::ptr::null_mut(),
            fault_addr: 0,
            itstate: core::ptr::null_mut(),
            // Fail-safe: a stray chain entry with zero budgets breaks out
            // immediately instead of running away.
            budget_ops: 0,
            budget_blocks: 0,
            link_epoch: 0,
            acc_ops: 0,
            acc_cycles: 0,
            acc_cyccnt: 0,
            acc_blocks: 0,
        }
    }
}

/// One chained edge (P2-4, T6): runner-owned, plain writable memory (no
/// self-modifying code — emitted code only READS these). The address is
/// baked into the emitting block; the box lives in the runner's link map
/// (stable heap address across map operations) and is reset when its
/// source block is rebuilt. `entry == 0` means unlinked (exit to driver).
#[repr(C)]
#[derive(Debug)]
pub struct LinkSlot {
    /// Static edges: the fixed target (recorded for driver mapping);
    /// dynamic edges (`BX`/`BLX`): the linked target PC (compared at emit).
    pub target_pc: u32,
    /// Linked target's chain-entry address (`0` = unlinked).
    pub entry: u64,
    /// Keepalive for the linked mapping (code is immutable; eviction must
    /// not unmap under a live edge — bytes are unchanged by eviction, so
    /// surviving links stay sound).
    pub target: Option<NativeBlock>,
    /// Bus write generation at link time (any bus write breaks the edge).
    pub gen: u64,
    /// Runner link epoch at link time (any invalidation/purge breaks all
    /// edges globally — cheap blanket unlinking for rare events).
    pub epoch: u64,
}

impl LinkSlot {
    /// A fresh unlinked edge to `target_pc` (dynamic edges pass the same
    /// value as a placeholder until first linked).
    #[must_use]
    pub const fn unlinked(target_pc: u32) -> Self {
        Self {
            target_pc,
            entry: 0,
            target: None,
            gen: 0,
            epoch: 0,
        }
    }
}

/// A committed native block: executable code plus its entry offsets.
/// `Clone` shares the mapping (code is immutable after `finalize`; cache
/// eviction drops the last `Arc` and unmaps).
#[derive(Clone, Debug)]
pub struct NativeBlock {
    code: Arc<ExecutableBuffer>,
    entry: AssemblyOffset,
    /// Chained entry (P2-4): offset of the budget/accumulator stub just
    /// past the prologue. Linked edges jump here with pins live in host
    /// registers and the driver's frame active (`[rsp]`/`[sp,#48]` still
    /// holds the context pointer).
    chain_entry: AssemblyOffset,
    /// Static exit targets in slot order (P2-4): the driver maps an exit
    /// PC back to its edge slot by position. Empty for dynamic-only
    /// (`BX`/`BLX`) blocks.
    static_exits: Box<[u32]>,
    /// Whether the block ends in a dynamic (`BX`/`BLX`) edge (its slot is
    /// the one past the static exits).
    has_dynamic: bool,
    /// Load-use tracker value stepping this block would leave behind
    /// (`Some(rt)` when its last op is a single load into `rt < 13`).
    /// Timing-only state, set by the emitter (P2-3).
    last_load: Option<u8>,
}

impl NativeBlock {
    /// Wrap a committed buffer (driver entry at `entry`, chained entry at
    /// `chain_entry`, exit shape from the emitter).
    #[must_use]
    pub fn new(
        code: ExecutableBuffer,
        entry: AssemblyOffset,
        chain_entry: AssemblyOffset,
        static_exits: Box<[u32]>,
        has_dynamic: bool,
        last_load: Option<u8>,
    ) -> Self {
        Self {
            code: Arc::new(code),
            entry,
            chain_entry,
            static_exits,
            has_dynamic,
            last_load,
        }
    }

    /// Raw entry pointer for `vioarm-jit-call` (see its SAFETY CASE: only
    ///_entry points produced here, with the matching ABI, are ever called).
    #[must_use]
    pub fn entry_ptr(&self) -> *const u8 {
        self.code.ptr(self.entry)
    }

    /// Raw chained-entry pointer (linked edges jump here; same SAFETY CASE
    /// — pins live, driver frame active, budgets set).
    #[must_use]
    pub fn chain_entry_ptr(&self) -> *const u8 {
        self.code.ptr(self.chain_entry)
    }

    /// Static exit targets in slot order (driver edge mapping).
    #[must_use]
    pub fn static_exits(&self) -> &[u32] {
        &self.static_exits
    }

    /// Whether the block ends in a dynamic (`BX`/`BLX`) edge.
    #[must_use]
    pub const fn has_dynamic(&self) -> bool {
        self.has_dynamic
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

    /// Load-use tracker value for the block exit (see the field docs).
    #[must_use]
    pub const fn last_load(&self) -> Option<u8> {
        self.last_load
    }
}

/// Byte offset of `regs.r[i]` from the regs base (array contiguity is
/// guaranteed; the field offset itself comes from `offset_of!`, so no
/// `repr(C)` assumption on `Registers`).
fn reg_offset(i: usize) -> i32 {
    (core::mem::offset_of!(Registers, r) + i * 4) as i32
}

#[cfg(target_arch = "x86_64")]
pub(crate) fn emit_prologue(ops: &mut dynasmrt::x64::Assembler) {
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

/// Chained-entry stub (P2-4, T6): budgets gate chain length (signed
/// subtract-and-branch: zero still runs — exact-fit is allowed),
/// accumulators collect the driver-equivalent accounting. Entered by
/// linked edges with pins live and the driver frame active; falls through
/// to the body. Returns the break label: the caller binds it after the
/// body with a `FALL`-to-`block_pc` epilogue (the driver re-runs the block
/// with fresh budgets — progress, never a skip).
#[cfg(target_arch = "x86_64")]
pub(crate) fn emit_chain_entry(
    ops: &mut dynasmrt::x64::Assembler,
    insns: u32,
    estimate: u64,
) -> dynasmrt::DynamicLabel {
    let budg_ops = core::mem::offset_of!(NativeCtx, budget_ops) as i32;
    let budg_blk = core::mem::offset_of!(NativeCtx, budget_blocks) as i32;
    let acc_ops = core::mem::offset_of!(NativeCtx, acc_ops) as i32;
    let acc_cyc = core::mem::offset_of!(NativeCtx, acc_cycles) as i32;
    let acc_cc = core::mem::offset_of!(NativeCtx, acc_cyccnt) as i32;
    let acc_blk = core::mem::offset_of!(NativeCtx, acc_blocks) as i32;
    // Estimates are `1 + touches` per op (`<= 2 * MAX_TB_INSNS`): imm32
    // always suffices (loud in debug if that ever changes).
    debug_assert!(estimate <= u64::from(u32::MAX));
    let (insns, est) = (insns as i32, estimate as i32);
    let brk = ops.new_dynamic_label();
    dynasm!(ops
        ; .arch x64
        ; mov rax, [rsp]
        ; sub QWORD [rax + budg_ops], insns
        ; js =>brk
        ; sub QWORD [rax + budg_blk], 1
        ; js =>brk
        ; add QWORD [rax + acc_ops], insns
        ; add QWORD [rax + acc_cyc], insns
        ; add QWORD [rax + acc_cc], est
        ; add QWORD [rax + acc_blk], 1
    );
    brk
}

/// Store the statically simulated ITSTATE exit value (P2-5, S4) through
/// the context pointer. Emitted once per IT-involving block, before the
/// body, so every exit (edges + budget-break) observes it.
#[cfg(target_arch = "x86_64")]
pub(crate) fn emit_itstate_store(ops: &mut dynasmrt::x64::Assembler, it: u8) {
    let c_it = core::mem::offset_of!(NativeCtx, itstate) as i32;
    let it = it as i8;
    dynasm!(ops
        ; .arch x64
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + c_it]
        ; mov BYTE [rsi], it
    );
}

/// One chained static edge (P2-4): `slot` is the baked [`LinkSlot`]
/// address. Linked (entry stamped with matching generation + epoch) jumps
/// straight to the target's chained entry — pins stay live, no driver
/// round-trip. Anything else falls to the unlinked epilogue (pins
/// writeback + return; the driver links the edge for next time).
/// Scratch is `rax`/`rsi`/`rdi`/`rdx` only (pins + `r15d` untouched; host
/// flags may clobber — guest flags live in `r15d`).
#[cfg(target_arch = "x86_64")]
pub(crate) fn emit_static_edge(
    ops: &mut dynasmrt::x64::Assembler,
    reason: u32,
    target: u32,
    slot: u64,
) {
    let unlinked = ops.new_dynamic_label();
    let slot = slot as i64;
    let e_off = core::mem::offset_of!(LinkSlot, entry) as i32;
    let g_off = core::mem::offset_of!(LinkSlot, gen) as i32;
    let x_off = core::mem::offset_of!(LinkSlot, epoch) as i32;
    let c_epoch = core::mem::offset_of!(NativeCtx, link_epoch) as i32;
    let c_gen = core::mem::offset_of!(NativeCtx, write_gen) as i32;
    dynasm!(ops
        ; .arch x64
        ; mov rax, QWORD slot
        ; mov rsi, [rax + e_off]
        ; test rsi, rsi
        ; jz =>unlinked
        ; mov rdi, [rsp]
        ; mov rdx, [rdi + c_epoch]
        ; cmp rdx, [rax + x_off]
        ; jne =>unlinked
        ; mov rdi, [rdi + c_gen]
        ; mov rdi, [rdi]
        ; cmp rdi, [rax + g_off]
        ; jne =>unlinked
        ; jmp rsi
        ; =>unlinked
    );
    emit_epilogue(ops, reason, target);
}

/// One chained dynamic edge (P2-4): the target arrives in `edx` (low 32
/// bits) and is compared against the slot's linked PC first. Scratch is
/// `rax`/`rsi`/`rdi` only — `edx` survives the gate for the miss
/// epilogue. Misses exit through the dynamic `BRANCH` epilogue.
#[cfg(target_arch = "x86_64")]
pub(crate) fn emit_dynamic_edge(ops: &mut dynasmrt::x64::Assembler, slot: u64) {
    let miss = ops.new_dynamic_label();
    let slot = slot as i64;
    let pc_off = core::mem::offset_of!(LinkSlot, target_pc) as i32;
    let e_off = core::mem::offset_of!(LinkSlot, entry) as i32;
    let g_off = core::mem::offset_of!(LinkSlot, gen) as i32;
    let x_off = core::mem::offset_of!(LinkSlot, epoch) as i32;
    let c_epoch = core::mem::offset_of!(NativeCtx, link_epoch) as i32;
    let c_gen = core::mem::offset_of!(NativeCtx, write_gen) as i32;
    dynasm!(ops
        ; .arch x64
        ; mov rax, QWORD slot
        ; cmp edx, [rax + pc_off]
        ; jne =>miss
        ; mov rsi, [rax + e_off]
        ; test rsi, rsi
        ; jz =>miss
        ; mov rdi, [rsp]
        ; mov rdi, [rdi + c_epoch]
        ; cmp rdi, [rax + x_off]
        ; jne =>miss
        ; mov rdi, [rsp]
        ; mov rdi, [rdi + c_gen]
        ; mov rdi, [rdi]
        ; cmp rdi, [rax + g_off]
        ; jne =>miss
        ; jmp rsi
        ; =>miss
    );
    emit_epilogue_dynamic(ops, EXIT_BRANCH);
}

/// Dynamic epilogue: `next_pc` arrives in `edx` (low 32 bits; producers
/// use 32-bit writes, which zero-extend), composed with `reason` and torn
/// down like [`emit_epilogue`]. For terminators whose target resolves at
/// runtime (conditional branches). `eax`/`ecx` are scratch as usual.
#[cfg(target_arch = "x86_64")]
pub(crate) fn emit_epilogue_dynamic(ops: &mut dynasmrt::x64::Assembler, reason: u32) {
    let ctx_regs = core::mem::offset_of!(NativeCtx, regs) as i32;
    let ctx_xpsr = core::mem::offset_of!(NativeCtx, xpsr) as i32;
    let (o0, o1, o2, o3) = (reg_offset(0), reg_offset(1), reg_offset(2), reg_offset(3));
    let reason = reason as i32;
    dynasm!(ops
        ; .arch x64
        // Write pinned set back (same as the static epilogue).
        ; mov rax, [rsp]
        ; mov rcx, [rax + ctx_regs]
        ; mov [rcx + o0], ebx
        ; mov [rcx + o1], r12d
        ; mov [rcx + o2], r13d
        ; mov [rcx + o3], r14d
        ; mov rcx, [rax + ctx_xpsr]
        ; mov [rcx], r15d
        // Compose (reason << 32) | next_pc from edx.
        ; mov eax, edx
        ; mov ecx, reason
        ; shl rcx, 32
        ; or rax, rcx
        ; add rsp, 16
        ; pop r15
        ; pop r14
        ; pop r13
        ; pop r12
        ; pop rbx
        ; ret
    );
}

#[cfg(target_arch = "x86_64")]
pub(crate) fn emit_epilogue(ops: &mut dynasmrt::x64::Assembler, reason: u32, next_pc: u32) {
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
        .map(|buf| {
            NativeBlock::new(
                buf,
                AssemblyOffset(0),
                AssemblyOffset(0),
                Box::new([]),
                false,
                None,
            )
        })
        .map_err(|_| crate::jit::EmitError::Finalize)
}

#[cfg(target_arch = "aarch64")]
pub(crate) fn emit_prologue(ops: &mut dynasmrt::aarch64::Assembler) {
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
pub(crate) fn emit_epilogue(ops: &mut dynasmrt::aarch64::Assembler, reason: u32, next_pc: u32) {
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

/// Store the statically simulated ITSTATE exit value (P2-5, S4,
/// AArch64): same placement contract as the x64 form.
#[cfg(target_arch = "aarch64")]
pub(crate) fn emit_itstate_store(ops: &mut dynasmrt::aarch64::Assembler, it: u8) {
    let c_it = core::mem::offset_of!(NativeCtx, itstate) as u32;
    let it = u32::from(it);
    dynasm!(ops
        ; .arch aarch64
        ; ldr x9, [sp, #48]
        ; ldr x9, [x9, c_it]
        ; movz w10, it
        ; strb w10, [x9]
    );
}

/// Absolute 64-bit immediate into `x9` (slot addresses, P2-4 edges).
#[cfg(target_arch = "aarch64")]
fn emit_movabs_x9(ops: &mut dynasmrt::aarch64::Assembler, addr: u64) {
    let (a0, a1, a2, a3) = (
        (addr & 0xFFFF) as u32,
        ((addr >> 16) & 0xFFFF) as u32,
        ((addr >> 32) & 0xFFFF) as u32,
        ((addr >> 48) & 0xFFFF) as u32,
    );
    dynasm!(ops
        ; .arch aarch64
        ; movz x9, a0, lsl #0
        ; movk x9, a1, lsl #16
        ; movk x9, a2, lsl #32
        ; movk x9, a3, lsl #48
    );
}

/// Chained-entry stub (P2-4, AArch64 mirror of the x64 stub): signed
/// budget checks (`subs` + `b.mi`), accumulators, fall through to the
/// body. Returns the break label (caller binds it after the body with a
/// `FALL`-to-`block_pc` epilogue). Immediates fit imm12 by construction
/// (`insns <= MAX_TB_INSNS`, estimates `<= 2 * MAX_TB_INSNS`).
#[cfg(target_arch = "aarch64")]
pub(crate) fn emit_chain_entry(
    ops: &mut dynasmrt::aarch64::Assembler,
    insns: u32,
    estimate: u64,
) -> dynasmrt::DynamicLabel {
    let budg_ops = core::mem::offset_of!(NativeCtx, budget_ops) as u32;
    let budg_blk = core::mem::offset_of!(NativeCtx, budget_blocks) as u32;
    let acc_ops = core::mem::offset_of!(NativeCtx, acc_ops) as u32;
    let acc_cyc = core::mem::offset_of!(NativeCtx, acc_cycles) as u32;
    let acc_cc = core::mem::offset_of!(NativeCtx, acc_cyccnt) as u32;
    let acc_blk = core::mem::offset_of!(NativeCtx, acc_blocks) as u32;
    debug_assert!(insns <= 64);
    debug_assert!(estimate <= 4095);
    let (insns, est) = (insns, estimate as u32);
    let brk = ops.new_dynamic_label();
    dynasm!(ops
        ; .arch aarch64
        ; ldr x9, [sp, #48]
        ; ldr x10, [x9, budg_ops]
        ; subs xzr, x10, #insns
        ; b.mi =>brk
        ; sub x10, x10, #insns
        ; str x10, [x9, budg_ops]
        ; ldr x10, [x9, budg_blk]
        ; subs xzr, x10, #1
        ; b.mi =>brk
        ; sub x10, x10, #1
        ; str x10, [x9, budg_blk]
        ; ldr x10, [x9, acc_ops]
        ; add x10, x10, #insns
        ; str x10, [x9, acc_ops]
        ; ldr x10, [x9, acc_cyc]
        ; add x10, x10, #insns
        ; str x10, [x9, acc_cyc]
        ; ldr x10, [x9, acc_cc]
        ; add x10, x10, est
        ; str x10, [x9, acc_cc]
        ; ldr x10, [x9, acc_blk]
        ; add x10, x10, #1
        ; str x10, [x9, acc_blk]
    );
    brk
}

/// One chained static edge (P2-4, AArch64): slot base stays in `x9`
/// throughout (scratch is `x9`–`x11`; `w0` is free — static targets bake
/// in). Linked jumps to the target's chained entry via `br`.
#[cfg(target_arch = "aarch64")]
pub(crate) fn emit_static_edge(
    ops: &mut dynasmrt::aarch64::Assembler,
    reason: u32,
    target: u32,
    slot: u64,
) {
    let unlinked = ops.new_dynamic_label();
    let e_off = core::mem::offset_of!(LinkSlot, entry) as u32;
    let g_off = core::mem::offset_of!(LinkSlot, gen) as u32;
    let x_off = core::mem::offset_of!(LinkSlot, epoch) as u32;
    let c_epoch = core::mem::offset_of!(NativeCtx, link_epoch) as u32;
    let c_gen = core::mem::offset_of!(NativeCtx, write_gen) as u32;
    emit_movabs_x9(ops, slot);
    dynasm!(ops
        ; .arch aarch64
        ; ldr x10, [x9, e_off]
        ; cbz x10, =>unlinked
        ; ldr x10, [sp, #48]
        ; ldr x11, [x10, c_epoch]
        ; ldr x10, [x9, x_off]
        ; cmp x11, x10
        ; b.ne =>unlinked
        ; ldr x10, [sp, #48]
        ; ldr x10, [x10, c_gen]
        ; ldr x10, [x10]
        ; ldr x11, [x9, g_off]
        ; cmp x10, x11
        ; b.ne =>unlinked
        ; ldr x10, [x9, e_off]
        ; br x10
        ; =>unlinked
    );
    emit_epilogue(ops, reason, target);
}

/// One chained dynamic edge (P2-4, AArch64): target arrives in `w0` and
/// survives the gate (scratch is `x9`–`x11` only). Misses exit through
/// the dynamic `BRANCH` epilogue.
#[cfg(target_arch = "aarch64")]
pub(crate) fn emit_dynamic_edge(ops: &mut dynasmrt::aarch64::Assembler, slot: u64) {
    let miss = ops.new_dynamic_label();
    let pc_off = core::mem::offset_of!(LinkSlot, target_pc) as u32;
    let e_off = core::mem::offset_of!(LinkSlot, entry) as u32;
    let g_off = core::mem::offset_of!(LinkSlot, gen) as u32;
    let x_off = core::mem::offset_of!(LinkSlot, epoch) as u32;
    let c_epoch = core::mem::offset_of!(NativeCtx, link_epoch) as u32;
    let c_gen = core::mem::offset_of!(NativeCtx, write_gen) as u32;
    emit_movabs_x9(ops, slot);
    dynasm!(ops
        ; .arch aarch64
        ; ldr w10, [x9, pc_off]
        ; cmp w0, w10
        ; b.ne =>miss
        ; ldr x10, [x9, e_off]
        ; cbz x10, =>miss
        ; ldr x10, [sp, #48]
        ; ldr x11, [x10, c_epoch]
        ; ldr x10, [x9, x_off]
        ; cmp x11, x10
        ; b.ne =>miss
        ; ldr x10, [sp, #48]
        ; ldr x10, [x10, c_gen]
        ; ldr x10, [x10]
        ; ldr x11, [x9, g_off]
        ; cmp x10, x11
        ; b.ne =>miss
        ; ldr x10, [x9, e_off]
        ; br x10
        ; =>miss
    );
    emit_epilogue_dynamic(ops, EXIT_BRANCH);
}

/// Dynamic epilogue: `next_pc` arrives in `w0`, composed with `reason`
/// and torn down like [`emit_epilogue`]. Terminator templates must
/// preserve the pinned set (scratch is `w0`/`w1`/`w9`–`w11` only).
#[cfg(target_arch = "aarch64")]
pub(crate) fn emit_epilogue_dynamic(ops: &mut dynasmrt::aarch64::Assembler, reason: u32) {
    let ctx_regs = core::mem::offset_of!(NativeCtx, regs) as u32;
    let ctx_xpsr = core::mem::offset_of!(NativeCtx, xpsr) as u32;
    let (o0, o1, o2, o3) = (
        reg_offset(0) as u32,
        reg_offset(1) as u32,
        reg_offset(2) as u32,
        reg_offset(3) as u32,
    );
    let r = reason & 0xFFFF;
    dynasm!(ops
        ; .arch aarch64
        ; ldr x9, [sp, #48]
        ; ldr x10, [x9, ctx_regs]
        ; str w19, [x10, o0]
        ; str w20, [x10, o1]
        ; str w21, [x10, o2]
        ; str w22, [x10, o3]
        ; ldr x10, [x9, ctx_xpsr]
        ; str w23, [x10]
        ; movz x1, r
        ; lsl x1, x1, #32
        ; orr x0, x0, x1
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
        .map(|buf| {
            NativeBlock::new(
                buf,
                AssemblyOffset(0),
                AssemblyOffset(0),
                Box::new([]),
                false,
                None,
            )
        })
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
            ..Default::default()
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
            .map(|buf| {
                NativeBlock::new(
                    buf,
                    AssemblyOffset(0),
                    AssemblyOffset(0),
                    Box::new([]),
                    false,
                    None,
                )
            })
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
            .map(|buf| {
                NativeBlock::new(
                    buf,
                    AssemblyOffset(0),
                    AssemblyOffset(0),
                    Box::new([]),
                    false,
                    None,
                )
            })
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
            ..Default::default()
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
