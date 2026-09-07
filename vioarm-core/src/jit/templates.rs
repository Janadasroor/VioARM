//! Template coverage + differential harness (P2-2, extended P2-3/P2-4).
//!
//! [`is_templated`] is the single coverage decision shared by both hosts
//! (CI parity: an op is native everywhere or interpreted everywhere).
//! The `harness` module executes one template natively over scratch state
//! and through the interpreter over identical state, comparing full
//! registers + flags word; every template test below is random-input
//! differential with directed edges. The harness serves P2-2 through P2-5
//! unchanged: later phases only add op classes and arch emitters.
//!
//! P2-4 adds [`exit_shape`] (the single source mapping a block body to its
//! chained-edge slots, shared by slot allocation and emission) and the
//! [`SlotTape`] cursor that hands baked slot addresses to terminator
//! templates in slot order.

use crate::cpu::decode16::Inst16;
use crate::jit::builder::{op_len, TbBody, TbEnd, TbOp};
use crate::jit::native::LinkSlot;

#[cfg(target_arch = "aarch64")]
use super::templates_aarch64 as arch;
#[cfg(target_arch = "x86_64")]
use super::templates_x64 as arch;
#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
compile_error!("jit templates support x86-64 and AArch64 hosts only");

/// Whether `op` has a native template on every host. S0 (P2-2): plain
/// 16-bit Thumb data-processing with no PC involvement (`r15` anywhere
/// bails — the guest PC is stale mid-block by construction; branches end
/// blocks and go through the driver instead).
pub fn is_templated(op: &TbOp) -> bool {
    match *op {
        TbOp::V16(inst) => match inst {
            Inst16::ShiftImm { rd, rm, .. } => rd != 15 && rm != 15,
            Inst16::ShiftReg { rd, rn, rm, .. } => rd != 15 && rn != 15 && rm != 15,
            Inst16::AddReg { rd, rn, rm } => rd != 15 && rn != 15 && rm != 15,
            Inst16::SubReg { rd, rn, rm } => rd != 15 && rn != 15 && rm != 15,
            Inst16::AddImm3 { rd, rn, .. } => rd != 15 && rn != 15,
            Inst16::SubImm3 { rd, rn, .. } => rd != 15 && rn != 15,
            Inst16::MovImm { rd, .. } => rd != 15,
            Inst16::CmpImm { rn, .. } => rn != 15,
            Inst16::AddImm8 { rd, .. } => rd != 15,
            Inst16::SubImm8 { rd, .. } => rd != 15,
            Inst16::Alu { rd, rm, .. } => rd != 15 && rm != 15,
            Inst16::SpecialCmp { rn, rm } => rn != 15 && rm != 15,
            // P2-2 terminators: 16-bit static branches (targets resolve
            // without guest state; reg/pc forms stay interpreted).
            Inst16::Branch { .. } => true,
            Inst16::BranchCond { cond, .. } => cond <= 13,
            Inst16::CbzCbnz { rn, .. } => rn != 15,
            // P2-4 terminators: register-indirect branches resolve at
            // runtime (fast path for plain Thumb targets; magic/even
            // targets side-exit to the interpreter). `rm == 15` reads the
            // stale guest PC — stays interpreted.
            Inst16::Bx { rm } => rm != 15,
            Inst16::Blx { rm } => rm != 15,
            // P2-5 (S3): stack + multiple transfers. `PUSH`/`POP`/`LDM`/
            // `STM` never touch `r15` (low regs, `sp`/`lr`/`pc` only);
            // `POP {pc}`/`LDM pc` use the pop-branch protocol, never the
            // stale PC.
            Inst16::Push { .. } => true,
            Inst16::Pop { .. } => true,
            Inst16::LdmStm { .. } => true,
            // P2-5 (S5-adjacent): pure extends/reversals, no flags.
            Inst16::Extend { rd, rm, .. } => rd != 15 && rm != 15,
            Inst16::Reverse { rd, rm, .. } => rd != 15 && rm != 15,
            // P2-5 (S4): the `IT` setter is a no-op template (the exit
            // store materializes the set); nested `IT` never emits (the
            // builder breaks before it).
            Inst16::It { .. } => true,
            // P2-3 single transfers (S1): no PC involvement (`r15`
            // anywhere bails — the guest PC is stale mid-block; `rn ==
            // 15` is allowed only for the integer-literal base form,
            // whose address is static). Dual/multiple/exclusive/VFP
            // forms stay interpreted (P2-5/P2-6).
            Inst16::LsReg { rt, rn, rm, .. } => rt != 15 && rn != 15 && rm != 15,
            Inst16::LsImm { rt, rn, .. } => rt != 15 && rn != 15,
            Inst16::LsSp { rt, .. } => rt != 15,
            Inst16::LdrLiteral { rt, .. } => rt != 15,
            _ => false,
        },
        TbOp::V32(inst) => match inst {
            crate::cpu::decode32::Inst32::Mem {
                access,
                rt,
                rn,
                rm,
                writeback,
                ..
            } => {
                use crate::cpu::decode32::MemOp as M;
                let single = matches!(
                    access,
                    M::Ldr | M::Str | M::Ldrb | M::Strb | M::Ldrh | M::Strh | M::Ldrsb | M::Ldrsh
                );
                single
                    && rt != 15
                    && (rn != 15 || (!writeback && is_mem_load(access)))
                    && rm.is_none_or(|r| r != 15)
            }
            // P2-4 terminators: static 32-bit branches (`BL` links `LR`
            // natively). `BLX`-immediate always faults (ARM-state target)
            // and stays interpreted.
            crate::cpu::decode32::Inst32::Bl { .. } => true,
            crate::cpu::decode32::Inst32::BranchWide { .. } => true,
            crate::cpu::decode32::Inst32::BranchCondWide { cond, .. } => cond <= 13,
            // P2-5 (S3): dual + multiple transfers. `rn == 15` is the
            // literal base (loads only, static address); register lists
            // with `pc` use the pop-branch protocol on loads and bail on
            // stores (stale-PC read).
            crate::cpu::decode32::Inst32::LdrdStrd {
                is_load,
                rt,
                rt2,
                rn,
                writeback,
                ..
            } => rt != 15 && rt2 != 15 && (rn != 15 || (is_load && !writeback)),
            crate::cpu::decode32::Inst32::LdmStm {
                is_load, rn, regs, ..
            } => rn != 15 && regs != 0 && (is_load || regs & 0x8000 == 0),
            // P2-5: table branch (guarded table read + dynamic edge).
            // `rm == 15` reads the stale PC; `rn == 15` is the static
            // `pc + 4` base.
            crate::cpu::decode32::Inst32::TblBranch { rm, .. } => rm != 15,
            // P2-5 (S5): 32-bit data processing. `rn`/`rm` read live
            // registers only (never the stale PC); writers avoid `r15`
            // (the interpreter faults those); test forms ignore `rd`.
            crate::cpu::decode32::Inst32::DpReg {
                op,
                rd,
                rn,
                rm,
                amount,
                ..
            } => rn != 15 && rm != 15 && amount <= 31 && (!dp_writes_rd(op) || rd != 15),
            crate::cpu::decode32::Inst32::DpImm { op, rd, rn, .. } => {
                rn != 15 && (!dp_writes_rd(op) || rd != 15)
            }
            crate::cpu::decode32::Inst32::MovWide { rd, .. } => rd != 15,
            crate::cpu::decode32::Inst32::Div { rd, rn, rm, .. } => {
                rd != 15 && rn != 15 && rm != 15
            }
            crate::cpu::decode32::Inst32::Mul { op, rd, rn, rm, ra } => {
                use crate::cpu::decode32::MulOp as O;
                matches!(op, O::Mul | O::Mla | O::Mls)
                    && rd != 15
                    && rn != 15
                    && rm != 15
                    && (matches!(op, O::Mul) || ra != 15)
            }
            _ => false,
        },
    }
}

/// Whether a 32-bit DP op writes `Rd` (test forms don't — mirrors
/// `exec_dp32`'s `writes_rd`, shared by coverage and emission).
pub(crate) fn dp_writes_rd(op: crate::cpu::decode32::DpOp) -> bool {
    use crate::cpu::decode32::DpOp as O;
    !matches!(op, O::Tst | O::Teq | O::Cmp | O::Cmn)
}

/// 16- and 32-bit control flow is `pc + 4`-relative.
fn branch_target(pc: u32, off: i32) -> u32 {
    pc.wrapping_add(4).wrapping_add_signed(off)
}

/// Whether `op` ends its block with control flow (P2-5: `POP {pc}`,
/// `LDM pc`, and table branches join the P2-4 set). Used to exclude
/// terminators from predicated bodies (a taken branch clears ITSTATE,
/// which straight-line templates never model).
pub(crate) fn is_terminator(op: &TbOp) -> bool {
    match *op {
        TbOp::V16(inst) => match inst {
            Inst16::Branch { .. }
            | Inst16::BranchCond { .. }
            | Inst16::CbzCbnz { .. }
            | Inst16::Bx { .. }
            | Inst16::Blx { .. } => true,
            Inst16::Pop { load_pc, .. } => load_pc,
            _ => false,
        },
        TbOp::V32(inst) => match inst {
            crate::cpu::decode32::Inst32::Bl { .. }
            | crate::cpu::decode32::Inst32::BranchWide { .. }
            | crate::cpu::decode32::Inst32::BranchCondWide { .. }
            | crate::cpu::decode32::Inst32::TblBranch { .. } => true,
            crate::cpu::decode32::Inst32::LdmStm { is_load, regs, .. } => {
                is_load && regs & 0x8000 != 0
            }
            _ => false,
        },
    }
}

/// Whether `op` may execute predicated inside an IT body (P2-5, S4):
/// templated ALU-only forms (16-bit S0 set + extends/reversals + 32-bit
/// DP). Memory ops (faults interact with suppression), terminators (a
/// taken branch clears ITSTATE), and `IT` itself (unpredictable nesting)
/// stay interpreted in predicated position.
pub(crate) fn is_predicable(op: &TbOp) -> bool {
    if !is_templated(op) || is_terminator(op) {
        return false;
    }
    match *op {
        TbOp::V16(inst) => matches!(
            inst,
            Inst16::ShiftImm { .. }
                | Inst16::ShiftReg { .. }
                | Inst16::AddReg { .. }
                | Inst16::SubReg { .. }
                | Inst16::AddImm3 { .. }
                | Inst16::SubImm3 { .. }
                | Inst16::MovImm { .. }
                | Inst16::CmpImm { .. }
                | Inst16::AddImm8 { .. }
                | Inst16::SubImm8 { .. }
                | Inst16::Alu { .. }
                | Inst16::SpecialCmp { .. }
                | Inst16::Extend { .. }
                | Inst16::Reverse { .. }
        ),
        TbOp::V32(inst) => matches!(
            inst,
            crate::cpu::decode32::Inst32::DpReg { .. }
                | crate::cpu::decode32::Inst32::DpImm { .. }
                | crate::cpu::decode32::Inst32::MovWide { .. }
                | crate::cpu::decode32::Inst32::Div { .. }
                | crate::cpu::decode32::Inst32::Mul { .. }
        ),
    }
}

/// `ITAdvance()` (P2-5, S4): mirrors `Core::it_advance` (same expression
/// over the same type — the emitter simulates ITSTATE statically over
/// predicated blocks; trajectories are pinned by unit test).
pub(crate) fn it_advance(it: u8) -> u8 {
    if it.trailing_zeros() >= 3 {
        0
    } else {
        (it & 0xE0) | (((it & 0x1F) << 1) & 0x1F)
    }
}

/// The ITSTATE a block exits with (P2-5, S4): the entry value advanced
/// over predicated ops, replaced by any `IT` set. Pure (unit-tested);
/// the emitter stores it once before the edges, so intermediate values
/// are never materialized (nothing native observes them: ALU-only bodies
/// have no exits, hooks are entry-gated).
pub(crate) fn exit_itstate(body: &TbBody) -> u8 {
    let mut it = body.it_entry;
    for op in &body.ops {
        if let TbOp::V16(Inst16::It { cond, mask }) = *op {
            it = (cond << 4) | mask;
        } else if it != 0 {
            it = it_advance(it);
        }
    }
    it
}

/// Exit shape of a block body (P2-4): static exit targets in slot order,
/// plus whether the block ends in a dynamic (`BX`/`BLX`) edge (its slot
/// trails the static ones). Single source for slot allocation (runner)
/// and emission (arch backends): `emit_block` asserts the emitter consumed
/// exactly this shape, so drift fails loud instead of mislinking.
///
/// Conditional forms list taken first (emitters select taken before fall).
/// Anything else (plain tails, store seals, untemplated tails) falls off
/// the end through one `FALL` edge.
pub fn exit_shape(body: &TbBody) -> (Vec<u32>, bool) {
    let n = body.ops.len();
    let last = body.ops.last();
    // The builder seals after the first terminator, so only the last op
    // can be one; `end_pc - len` is its address. Simulate ITSTATE to the
    // last op (predicated/`IT` tails fall through — see `exit_shape_for`).
    let mut it = body.it_entry;
    for op in body.ops.iter().take(n.saturating_sub(1)) {
        if let TbOp::V16(Inst16::It { cond, mask }) = *op {
            it = (cond << 4) | mask;
        } else if it != 0 {
            it = it_advance(it);
        }
    }
    let last_pc = last.map(|op| body.end_pc.wrapping_sub(op_len(op)));
    exit_shape_for(last, last_pc, body.end_pc, it)
}

/// [`exit_shape`] over an explicit tail (the harness sizes single-op
/// scratch slots without building a body). `it_at_last` is the simulated
/// ITSTATE at the last op: predicated/`IT` tails always fall through
/// (their single `FALL` edge), terminators are impossible there (the
/// builder breaks first — and the emitter errors closed if one arrives).
pub(crate) fn exit_shape_for(
    last: Option<&TbOp>,
    last_pc: Option<u32>,
    end_pc: u32,
    it_at_last: u8,
) -> (Vec<u32>, bool) {
    use crate::cpu::decode16::Inst16 as I16;
    use crate::cpu::decode32::Inst32 as I32;

    let fall = vec![end_pc];
    let (Some(last), Some(last_pc)) = (last, last_pc) else {
        return (fall, false);
    };
    if it_at_last != 0 || matches!(*last, TbOp::V16(I16::It { .. })) {
        return (fall, false);
    }
    match *last {
        TbOp::V16(I16::Branch { off }) => (vec![branch_target(last_pc, off)], false),
        TbOp::V16(I16::BranchCond { cond, off }) if cond <= 13 => (
            vec![branch_target(last_pc, off), last_pc.wrapping_add(2)],
            false,
        ),
        TbOp::V16(I16::CbzCbnz { off, .. }) => (
            vec![branch_target(last_pc, off), last_pc.wrapping_add(2)],
            false,
        ),
        TbOp::V16(I16::Bx { .. }) | TbOp::V16(I16::Blx { .. }) => (vec![], true),
        TbOp::V16(I16::Pop { load_pc: true, .. }) => (vec![], true),
        TbOp::V32(I32::TblBranch { .. }) => (vec![], true),
        TbOp::V32(I32::LdmStm { is_load, regs, .. }) if is_load && regs & 0x8000 != 0 => {
            (vec![], true)
        }
        TbOp::V32(I32::Bl { off }) => (vec![branch_target(last_pc, off)], false),
        TbOp::V32(I32::BranchWide { off }) => (vec![branch_target(last_pc, off)], false),
        TbOp::V32(I32::BranchCondWide { cond, off }) if cond <= 13 => (
            vec![branch_target(last_pc, off), last_pc.wrapping_add(4)],
            false,
        ),
        _ => (fall, false),
    }
}

/// Cursor handing baked [`LinkSlot`] addresses to terminator templates in
/// slot order (P2-4). Overruns panic: an emitter consuming more edges than
/// [`exit_shape`] allocated means the two drifted, and baking a wrong
/// address would mislink (silence is unsound here — fail loud).
pub(crate) struct SlotTape<'a> {
    slots: &'a [LinkSlot],
    next: usize,
}

impl<'a> SlotTape<'a> {
    pub(crate) fn new(slots: &'a [LinkSlot]) -> Self {
        Self { slots, next: 0 }
    }

    pub(crate) fn take(&mut self) -> u64 {
        let slot = self
            .slots
            .get(self.next)
            .expect("slot tape overrun: exit_shape drifted from emitter");
        self.next += 1;
        (slot as *const LinkSlot) as u64
    }

    pub(crate) fn done(&self) -> bool {
        self.next == self.slots.len()
    }
}

/// Whether a 32-bit `Mem` access loads (vs stores).
fn is_mem_load(access: crate::cpu::decode32::MemOp) -> bool {
    use crate::cpu::decode32::MemOp as M;
    matches!(access, M::Ldr | M::Ldrb | M::Ldrh | M::Ldrsb | M::Ldrsh)
}

/// Static memory-access descriptor shared by template emission and the
/// side-exit fill path (single source: `(byte size, is_write)`).
/// `None` for non-memory or deferred (dual/multiple/exclusive) ops.
pub(crate) fn mem_access(op: &TbOp) -> Option<(u32, bool)> {
    match *op {
        TbOp::V16(inst) => {
            use crate::cpu::decode16::{LsImm, LsReg};
            match inst {
                Inst16::LsReg { kind, .. } => match kind {
                    LsReg::Str => Some((4, true)),
                    LsReg::Ldr => Some((4, false)),
                    LsReg::Strh => Some((2, true)),
                    LsReg::Ldrh => Some((2, false)),
                    LsReg::Strb => Some((1, true)),
                    LsReg::Ldrb | LsReg::Ldrsb => Some((1, false)),
                    LsReg::Ldrsh => Some((2, false)),
                },
                Inst16::LsImm { kind, .. } => match kind {
                    LsImm::Str => Some((4, true)),
                    LsImm::Ldr => Some((4, false)),
                    LsImm::Strh => Some((2, true)),
                    LsImm::Ldrh => Some((2, false)),
                    LsImm::Strb => Some((1, true)),
                    LsImm::Ldrb => Some((1, false)),
                },
                Inst16::LsSp { is_load, .. } => Some((4, !is_load)),
                Inst16::LdrLiteral { .. } => Some((4, false)),
                // P2-5 multi-word (word multiples; the fill path only
                // needs size class + direction for the faulting page).
                Inst16::Push { .. } => Some((4, true)),
                Inst16::Pop { .. } => Some((4, false)),
                Inst16::LdmStm { is_load, .. } => Some((4, !is_load)),
                _ => None,
            }
        }
        TbOp::V32(inst) => match inst {
            crate::cpu::decode32::Inst32::Mem { access, .. } => {
                use crate::cpu::decode32::MemOp as M;
                match access {
                    M::Str => Some((4, true)),
                    M::Ldr => Some((4, false)),
                    M::Strh => Some((2, true)),
                    M::Ldrh => Some((2, false)),
                    M::Strb => Some((1, true)),
                    M::Ldrb | M::Ldrsb => Some((1, false)),
                    M::Ldrsh => Some((2, false)),
                }
            }
            crate::cpu::decode32::Inst32::LdrdStrd { is_load, .. } => Some((4, !is_load)),
            crate::cpu::decode32::Inst32::LdmStm { is_load, .. } => Some((4, !is_load)),
            // Table reads fill like loads (byte vs halfword is static).
            crate::cpu::decode32::Inst32::TblBranch { half, .. } => {
                Some((u32::from(half) + 1, false))
            }
            _ => None,
        },
    }
}

/// Emit one templated op at guest `pc` into an arch assembler. Returns
/// `false` for untemplated ops (the block then stays interpreted).
pub fn emit_op(ops: &mut arch::Assembler, op: &TbOp, pc: u32, unalign_trap: bool) -> bool {
    if !is_templated(op) {
        return false;
    }
    arch::emit_op(ops, op, pc, unalign_trap)
}

/// Assemble prologue + one template + edges for harness runs. The op is
/// wrapped in a one-op body so emission shares the block path exactly
/// (including slot consumption — the harness passes scratch slots, which
/// stay unlinked so every edge exits to the driver like before).
pub(crate) fn assemble_single(
    op: &TbOp,
    pc: u32,
    unalign_trap: bool,
    slots: &[LinkSlot],
) -> Result<super::native::NativeBlock, super::EmitError> {
    use crate::cpu::Isa;

    let body = TbBody {
        pc,
        secure: false,
        isa: Isa::Armv7M,
        ops: vec![*op],
        end_pc: pc.wrapping_add(op_len(op)),
        end: TbEnd::MaxInsns,
        start_page: pc >> super::tb::PAGE_SHIFT,
        page_count: 1,
        cycle_estimate: 0,
        store_mask: 0,
        it_entry: 0,
    };
    emit_block(&body, unalign_trap, slots)
}

/// Emit a whole block natively: prologue, the chained-entry stub (P2-4),
/// one template per op in order, then the terminator's own edges — or,
/// when the block ends in plain ops, a `FALL` edge to the fallthrough
/// address. Bails with [`EmitError::Untemplated`] on the first untemplated
/// op (the block then stays interpreted); the builder guarantees
/// terminators only appear last, so anything after one would be
/// unreachable anyway. Asserts the emitter consumed exactly the slots
/// [`exit_shape`] allocated (drift fails loud — see [`SlotTape`]).
pub fn emit_block(
    body: &super::TbBody,
    unalign_trap: bool,
    slots: &[LinkSlot],
) -> Result<super::native::NativeBlock, super::EmitError> {
    use super::native::{emit_chain_entry, emit_epilogue, emit_prologue, NativeBlock, EXIT_FALL};
    use dynasmrt::AssemblyOffset;

    let mut ops = arch::Assembler::new()?;
    emit_prologue(&mut ops);
    // Driver entry skips the chained-entry stub (budgets/accumulators are
    // a chaining resource — the entered block must not consume them).
    let body_lbl = arch::emit_entry_jump(&mut ops);
    // Marker label: its resolved offset is the chained entry (linked edges
    // jump here with pins live and the driver frame active).
    let chain_lbl = ops.new_dynamic_label();
    arch::mark_label(&mut ops, chain_lbl);
    let chain_entry = ops
        .labels()
        .resolve_dynamic(chain_lbl)
        .map_err(|_| super::EmitError::Finalize)?;
    let brk = emit_chain_entry(&mut ops, body.ops.len() as u32, body.cycle_estimate);
    arch::mark_label(&mut ops, body_lbl);
    // ITSTATE exit store first (P2-5, S4): the value is fully static, and
    // nothing native reads the field mid-block (conditions are static,
    // guards read flags) — so it can land before the body and cover every
    // exit (edges + budget-break). Skipped when unchanged, which is every
    // pre-S4 block (zero added bytes there).
    let exit_it = exit_itstate(body);
    if exit_it != body.it_entry {
        super::native::emit_itstate_store(&mut ops, exit_it);
    }
    let mut tape = SlotTape::new(slots);
    let mut pc = body.pc;
    let mut it = body.it_entry;
    let mut edged = false;
    let n = body.ops.len();
    for (i, op) in body.ops.iter().enumerate() {
        if let TbOp::V16(Inst16::It { cond, mask }) = *op {
            // No-op template (the exit store materializes the set).
            // Nested `IT` never reaches emission (builder breaks first).
            it = (cond << 4) | mask;
        } else if it != 0 {
            // Predicated body op (S4): the builder guarantees predicable
            // (arch double-gates via `emit_op` failing closed).
            if !arch::emit_predicated(&mut ops, op, pc, unalign_trap, it >> 4) {
                return Err(super::EmitError::Untemplated);
            }
            it = it_advance(it);
        } else if i + 1 == n {
            if emit_op(&mut ops, op, pc, unalign_trap) {
                arch::emit_fall_edge(&mut ops, body.end_pc, tape.take());
            } else if !arch::emit_terminator(&mut ops, op, pc, unalign_trap, &mut tape) {
                return Err(super::EmitError::Untemplated);
            }
            edged = true;
        } else if !emit_op(&mut ops, op, pc, unalign_trap) {
            return Err(super::EmitError::Untemplated);
        }
        pc = pc.wrapping_add(op_len(op));
    }
    if !edged {
        // Predicated or `IT` tail: no terminator emitted its own edge.
        arch::emit_fall_edge(&mut ops, body.end_pc, tape.take());
    }
    assert!(
        tape.done(),
        "slot tape underrun: exit_shape over-allocated for this body"
    );
    // Budget-break exit: pins writeback + `FALL` to the block's own PC
    // (the driver re-runs it with fresh budgets).
    arch::mark_label(&mut ops, brk);
    emit_epilogue(&mut ops, EXIT_FALL, body.pc);
    let last_load = last_op_load(body.ops.last());
    let (statics, has_dynamic) = exit_shape(body);
    ops.finalize()
        .map(|buf| {
            NativeBlock::new(
                buf,
                AssemblyOffset(0),
                chain_entry,
                statics.into_boxed_slice(),
                has_dynamic,
                last_load,
            )
        })
        .map_err(|_| super::EmitError::Finalize)
}

/// Load-use tracker value stepping `op` (as a block tail) would leave
/// behind, mirroring `base_cycles`: `Some(rt)` for single loads into
/// `rt < 13`, else `None`. Timing-only state.
fn last_op_load(op: Option<&TbOp>) -> Option<u8> {
    use crate::cpu::decode16::{LsImm, LsReg};
    use crate::cpu::decode32::MemOp;
    let op = op?;
    let rt = match *op {
        TbOp::V16(crate::cpu::decode16::Inst16::LdrLiteral { rt, .. }) => rt,
        TbOp::V16(crate::cpu::decode16::Inst16::LsSp {
            rt, is_load: true, ..
        }) => rt,
        TbOp::V16(crate::cpu::decode16::Inst16::LsReg {
            rt,
            kind: LsReg::Ldr | LsReg::Ldrb | LsReg::Ldrh | LsReg::Ldrsb | LsReg::Ldrsh,
            ..
        }) => rt,
        TbOp::V16(crate::cpu::decode16::Inst16::LsImm {
            rt,
            kind: LsImm::Ldr | LsImm::Ldrb | LsImm::Ldrh,
            ..
        }) => rt,
        TbOp::V32(crate::cpu::decode32::Inst32::Mem {
            rt,
            access: MemOp::Ldr | MemOp::Ldrb | MemOp::Ldrh | MemOp::Ldrsb | MemOp::Ldrsh,
            ..
        }) => rt,
        _ => return None,
    };
    if rt < 13 {
        Some(rt)
    } else {
        None
    }
}

#[cfg(test)]
pub(crate) mod harness {
    use crate::cpu::registers::{Registers, Xpsr};
    use crate::cpu::Core;
    use crate::error::HaltCause;
    use crate::jit::native::{exit_reason, NativeCtx};
    use crate::mem::flat::FlatMemory;

    use super::*;

    /// Deterministic xorshift64* (fixed seed: CI-reproducible trials).
    pub struct Rng(u64);

    impl Rng {
        pub fn new() -> Self {
            Self(0x1234_5678_9ABC_DEF1)
        }

        pub fn next_u32(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            self.0 = x;
            (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32
        }
    }

    /// Directed edge values sprinkled into operand regs each trial.
    pub const EDGES: &[u32] = &[
        0x0000_0000,
        0x0000_0001,
        0x0000_0002,
        0x0000_001F,
        0x0000_0020,
        0x0000_0021,
        0x7FFF_FFFF,
        0x8000_0000,
        0x8000_0001,
        0xFFFF_FFFE,
        0xFFFF_FFFF,
        0x00FF_00FF,
        0xFF00_FF00,
        0x5555_5555,
        0xAAAA_AAAA,
    ];

    /// Scratch register file: random plus two edge values at random lanes.
    fn scratch_regs(rng: &mut Rng, trial: u32) -> Registers {
        let mut r = [0u32; 13];
        for slot in r.iter_mut() {
            *slot = rng.next_u32();
        }
        let e = EDGES.len() as u32;
        r[rng.next_u32() as usize % 13] = EDGES[trial as usize % e as usize];
        r[rng.next_u32() as usize % 13] = EDGES[(trial as usize * 7 + 1) % e as usize];
        Registers {
            r,
            sp: rng.next_u32(),
            msp: 0,
            psp: 0,
            lr: rng.next_u32(),
            pc: 0x0800_1000,
        }
    }

    /// Scratch flags: random NZCV/Q/GE lanes, Thumb set, IT zero (native
    /// blocks always run with `itstate == 0`).
    fn scratch_xpsr(rng: &mut Rng) -> Xpsr {
        let mut x = Xpsr::default();
        x.set(0x0100_0000 | (rng.next_u32() & 0xF89F_0000));
        x
    }

    /// Scratch edge slots for one op (stay unlinked: harness executions
    /// always exit to the driver, exactly like an unlinked block).
    fn scratch_slots(op: &TbOp, pc: u32) -> Vec<LinkSlot> {
        let end = pc.wrapping_add(op_len(op));
        let (statics, has_dynamic) = exit_shape_for(Some(op), Some(pc), end, 0);
        let mut slots: Vec<LinkSlot> = statics.into_iter().map(LinkSlot::unlinked).collect();
        if has_dynamic {
            slots.push(LinkSlot::unlinked(u32::MAX));
        }
        slots
    }

    /// Raw emitted bytes for one template (byte-level debugging).
    #[allow(dead_code)]
    pub fn dump_native(op: &TbOp) -> Vec<u8> {
        use super::super::native::{emit_epilogue, emit_prologue, EXIT_FALL};
        let mut ops = super::arch::Assembler::new().unwrap();
        emit_prologue(&mut ops);
        assert!(super::emit_op(&mut ops, op, 0x1000, false));
        emit_epilogue(&mut ops, EXIT_FALL, 0);
        let buf = ops.finalize().unwrap();
        buf.to_vec()
    }

    /// Execute one template natively over scratch state at guest `pc`.
    /// Returns the end state plus the raw exit code (callers assert the
    /// reason: S0 ops always `FALL`, terminators `BRANCH`).
    pub fn run_native(
        op: &TbOp,
        regs: &Registers,
        xpsr: &Xpsr,
        pc: u32,
        unalign_trap: bool,
    ) -> ((Registers, Xpsr), u64) {
        let mut r = *regs;
        let mut x = *xpsr;
        let mut itstate = 0u8;
        let mut ctx = Box::new(NativeCtx {
            regs: &mut r,
            xpsr: &mut x,
            itstate: &mut itstate,
            // Unbounded budgets: harness blocks never chain (slots stay
            // unlinked), but the chained-entry stub still gates the body
            // on them — zero would break out before running anything.
            budget_ops: i64::MAX,
            budget_blocks: i64::MAX,
            ..Default::default()
        });
        // Slots live in this frame across the call (the emitted code reads
        // them while running — baking a shorter-lived address would be
        // unsound even unlinked).
        let slots = scratch_slots(op, pc);
        let block = assemble_single(op, pc, unalign_trap, &slots).expect("harness op is templated");
        let code = vioarm_jit_call::call1(block.entry_ptr(), &mut *ctx as *mut NativeCtx as u64);
        ((r, x), code)
    }

    /// Execute one op through the interpreter over identical scratch state
    /// (no bus traffic: ALU ops never fetch or touch memory; `exec` takes
    /// the decoded op directly). Covers 16-bit S0 and the 32-bit DP
    /// subset (both fall through — callers assert the exit).
    pub fn run_interp(op: &TbOp, regs: &Registers, xpsr: &Xpsr) -> (Registers, Xpsr) {
        let mem = FlatMemory::with_layout(&[], &[]);
        let mut core = Core::armv7m(mem);
        core.regs = *regs;
        core.xpsr = *xpsr;
        core.itstate = 0;
        match *op {
            TbOp::V16(inst) => {
                core.exec_tb16(0x1000, inst).unwrap();
            }
            TbOp::V32(inst) => {
                core.exec_tb32(0x1000, inst).unwrap();
            }
        }
        (core.regs, core.xpsr)
    }

    /// Randomized differential: `trials` states per op, each with random
    /// regs/flags plus directed edges; `force` pins `(reg, values)` cycled
    /// by trial (for count operands like register shifts). S0 ops always
    /// fall through (`FALL` exit asserted here).
    pub fn check_one(op: &TbOp, trials: u32, force: &[(u8, &[u32])]) {
        use crate::jit::native::EXIT_FALL;

        assert!(is_templated(op), "harness checks templated ops");
        let mut rng = Rng::new();
        for t in 0..trials {
            let mut regs = scratch_regs(&mut rng, t);
            for (reg, vals) in force {
                regs.r[*reg as usize] = vals[t as usize % vals.len()];
            }
            let xpsr = scratch_xpsr(&mut rng);
            let ((nr, nx), code) = run_native(op, &regs, &xpsr, 0x1000, false);
            assert_eq!(exit_reason(code), EXIT_FALL, "S0 always falls");
            let (ir, ix) = run_interp(op, &regs, &xpsr);
            assert_eq!((nr, nx), (ir, ix), "trial {t} for {op:?}");
        }
    }

    /// Convenience: no forced regs.
    pub fn check(op: &TbOp, trials: u32) {
        check_one(op, trials, &[]);
    }

    // ---- Memory-template harness (P2-3) ----
    //
    // Single ops execute against a real `FlatMemory` (vectors + stack for
    // fault delivery, RAM for data) with a real TLB: the first run
    // side-exits (cold TLB), the harness fills via the shared `fill_tlb`
    // exactly like the driver, and the second run takes the fast path.
    // The interpreter reference runs the same op over an identically
    // initialized bus. Compared: regs, flags, and RAM bytes (fault tests
    // additionally compare exception state: same `exec` path both sides).

    use crate::jit::tlb::{fill_tlb, Tlb};
    use crate::mem::bus::Bus;

    pub(crate) const MEM_RAM: u32 = 0x2000_0000;
    const MEM_VEC: u32 = 0x0000_0000;
    pub(crate) const MEM_SP: u32 = 0x2000_0200;
    const MEM_HANDLER: u32 = 0x2000_0300;

    /// Deterministic bus image: vectors (all faults → handler), stack
    /// area, and patterned RAM.
    pub fn mem_bus() -> FlatMemory {
        let mut bus = FlatMemory::with_layout(&[MEM_VEC, MEM_RAM], &[0x400, 0x1000]);
        bus.write_word(MEM_VEC, MEM_SP).unwrap();
        for vec in [0x08, 0x0C, 0x10, 0x18, 0x2C] {
            bus.write_word(MEM_VEC + vec, MEM_HANDLER | 1).unwrap();
        }
        for i in 0..0x1000u32 {
            let b = (i.wrapping_mul(0x9E37).wrapping_add(i >> 3) & 0xFF) as u8;
            bus.write(MEM_RAM + i, b).unwrap();
        }
        bus
    }

    /// Snapshotted RAM window for comparison.
    pub fn mem_snap(bus: &mut FlatMemory) -> Vec<u8> {
        (0..0x100u32)
            .map(|i| bus.read(MEM_RAM + i).unwrap())
            .collect()
    }

    /// Scratch regs pointed at the harness RAM (sp valid for stacking).
    pub fn mem_regs() -> Registers {
        Registers {
            r: [0u32; 13],
            sp: MEM_SP,
            msp: MEM_SP,
            psp: 0,
            lr: 0,
            pc: MEM_RAM,
        }
    }

    /// Run one memory op natively: cold TLB first (must side-exit with
    /// `MMIO`), fill from the reported address exactly like the driver,
    /// re-run (fast path or second exit). Returns end state + RAM +
    /// `fault_addr` + both exit codes. Miss paths are effect-free by
    /// template construction (scratch-only prefix), so both passes start
    /// from the same initial state.
    pub fn run_mem(
        op: &TbOp,
        regs: &Registers,
        xpsr: &Xpsr,
        pc: u32,
        trap: bool,
        bus: &mut FlatMemory,
        tlb: &mut Tlb,
        gen: &mut u64,
    ) -> (
        (Registers, Xpsr, Vec<u8>),
        u64,
        u32,
        (Registers, Xpsr, Vec<u8>),
        u64,
    ) {
        use super::super::native::{exit_reason, NativeCtx, EXIT_MMIO};
        use super::mem_access;

        let slots = scratch_slots(op, pc);
        let run_once = |bus: &mut FlatMemory, tlb: &mut Tlb, gen: &mut u64| {
            let mut r = *regs;
            let mut x = *xpsr;
            let mut itstate = 0u8;
            let mut ctx = Box::new(NativeCtx {
                regs: &mut r,
                xpsr: &mut x,
                tlb: &mut *tlb as *mut Tlb,
                privileged: 1,
                write_gen: &mut *gen as *mut u64,
                itstate: &mut itstate,
                budget_ops: i64::MAX,
                budget_blocks: i64::MAX,
                ..Default::default()
            });
            let block = assemble_single(op, pc, trap, &slots).expect("harness op is templated");
            let code =
                vioarm_jit_call::call1(block.entry_ptr(), &mut *ctx as *mut NativeCtx as u64);
            let fault = ctx.fault_addr;
            ((r, x, mem_snap(bus)), code, fault)
        };
        let (st1, code1, fault) = run_once(bus, tlb, gen);
        assert_eq!(exit_reason(code1), EXIT_MMIO, "cold TLB must side-exit");
        if let Some((size, is_write)) = mem_access(op) {
            fill_tlb(tlb, bus, fault, size, is_write, true, 0);
        }
        let (st2, code2, _) = run_once(bus, tlb, gen);
        (st1, code1, fault, st2, code2)
    }

    /// Interpreter reference for one memory op over a twin bus.
    pub fn run_mem_interp(
        op: &TbOp,
        regs: &Registers,
        xpsr: &Xpsr,
        pc: u32,
        bus: FlatMemory,
    ) -> ((Registers, Xpsr, Vec<u8>, u8), Option<HaltCause>) {
        use crate::cpu::Next;

        let mut core = Core::armv7m(bus);
        core.regs = *regs;
        core.xpsr = *xpsr;
        core.itstate = 0;
        let outcome = match *op {
            TbOp::V16(inst) => core.exec_tb16(pc, inst),
            TbOp::V32(inst) => core.exec_tb32(pc, inst),
        };
        let err = match outcome {
            Ok(Next::Advance(a)) => {
                // Commit exactly like the stepper tail (cycles irrelevant
                // here; compared state is_regs/flags/RAM).
                core.regs.pc = a;
                None
            }
            Ok(Next::Stop(c)) | Err(c) => Some(c),
        };
        let ram = mem_snap(core.mem_mut());
        ((core.regs, core.xpsr, ram, core.ipsr), err)
    }
}

#[cfg(test)]
mod tests {
    use super::harness::*;
    use super::*;
    use crate::cpu::registers::{Registers, Xpsr};

    /// Low regs (pinned) + general regs (memory path) + high regs.
    const POOL: &[u8] = &[0, 1, 2, 3, 4, 5, 7, 8, 12];

    fn pool(i: usize) -> u8 {
        POOL[i % POOL.len()]
    }

    #[test]
    fn mov_imm_matches() {
        for (k, rd) in POOL.iter().enumerate() {
            for imm in [0u8, 1, 0x7F, 0x80, 0xFF] {
                let op = TbOp::V16(Inst16::MovImm { rd: *rd, imm });
                check(&op, 64 + k as u32 * 8);
            }
        }
    }

    #[test]
    fn add_sub_reg_match() {
        for (k, rd) in POOL.iter().enumerate() {
            let rn = pool(k + 1);
            let rm = pool(k + 2);
            check(&TbOp::V16(Inst16::AddReg { rd: *rd, rn, rm }), 128);
            check(&TbOp::V16(Inst16::SubReg { rd: *rd, rn, rm }), 128);
        }
    }

    #[test]
    fn add_sub_imm_match() {
        for (k, rd) in POOL.iter().enumerate() {
            let rn = pool(k + 3);
            for imm in [0u8, 1, 7, 0xFF] {
                check(
                    &TbOp::V16(Inst16::AddImm3 {
                        rd: *rd,
                        rn,
                        imm: imm & 7,
                    }),
                    48,
                );
                check(
                    &TbOp::V16(Inst16::SubImm3 {
                        rd: *rd,
                        rn,
                        imm: imm & 7,
                    }),
                    48,
                );
                check(&TbOp::V16(Inst16::AddImm8 { rd: *rd, imm }), 48);
                check(&TbOp::V16(Inst16::SubImm8 { rd: *rd, imm }), 48);
            }
        }
        check(&TbOp::V16(Inst16::CmpImm { rn: 2, imm: 0x5A }), 256);
    }

    #[test]
    fn shift_imm_matches_all_amounts() {
        use crate::cpu::decode16::ShiftImm as K;
        for kind in [K::Lsl, K::Lsr, K::Asr] {
            for amount in 0..32u8 {
                let op = TbOp::V16(Inst16::ShiftImm {
                    rd: 1,
                    rm: 4,
                    kind,
                    amount,
                });
                check(&op, 48);
            }
        }
        // Pinned-destination variant at the edge amounts.
        for amount in [0u8, 1, 31] {
            let op = TbOp::V16(Inst16::ShiftImm {
                rd: 0,
                rm: 2,
                kind: crate::cpu::decode16::ShiftImm::Lsl,
                amount,
            });
            check(&op, 128);
        }
    }

    /// Register-shift counts covering every interpreter class.
    const COUNTS: &[u32] = &[0, 1, 2, 7, 8, 15, 16, 31, 32, 33, 63, 64, 65, 127, 128, 255];

    #[test]
    fn shift_reg_matches() {
        use crate::cpu::decode16::Shift as K;
        for kind in [K::Lsl, K::Lsr, K::Asr, K::Ror] {
            // Value in r4 (memory path), count forced into r5 lane by lane.
            let op = TbOp::V16(Inst16::ShiftReg {
                rd: 1,
                rn: 4,
                rm: 5,
                kind,
            });
            check_one(&op, 64, &[(5, COUNTS)]);
            // Pinned-lane variant.
            let op = TbOp::V16(Inst16::ShiftReg {
                rd: 0,
                rn: 2,
                rm: 3,
                kind,
            });
            check_one(&op, 64, &[(3, COUNTS)]);
        }
    }

    #[test]
    fn alu_ops_match() {
        use crate::cpu::decode16::AluOp as O;
        let ops = [
            O::And,
            O::Eor,
            O::Orr,
            O::Bic,
            O::Tst,
            O::Cmp,
            O::Cmn,
            O::Mul,
            O::Mvn,
            O::Neg,
            O::Adc,
            O::Sbc,
            O::Lsl,
            O::Lsr,
            O::Asr,
            O::Ror,
        ];
        for (k, op) in ops.iter().enumerate() {
            let rd = pool(k);
            let rm = pool(k + 4);
            let inst = TbOp::V16(Inst16::Alu { rd, rm, op: *op });
            // Shift/rotate-by-register ops take counts from Rm: force the
            // class edges into the count lane.
            let force: &[(u8, &[u32])] = match op {
                O::Lsl | O::Lsr | O::Asr | O::Ror => &[(rm, COUNTS)],
                _ => &[],
            };
            check_one(&inst, 128, force);
        }
    }

    #[test]
    fn special_cmp_matches() {
        let op = TbOp::V16(Inst16::SpecialCmp { rn: 8, rm: 12 });
        check(&op, 256);
    }

    #[test]
    fn untemplated_ops_bail() {
        // System ops, PC forms, unpredictable register branches, secure
        // branches, always-faulting `BLX`-immediate, long/exclusive/DSP
        // multiplies, and the remaining 32-bit ops have no template: the
        // emitter reports false and the block stays interpreted.
        // (`POP {pc}`, table branches, and the S3/S5 sets went native in
        // P2-5 — they must NOT appear here.)
        let cases = [
            TbOp::V16(Inst16::Svc { imm: 0 }),
            TbOp::V16(Inst16::Bkpt { imm: 0 }),
            TbOp::V16(Inst16::MovImm { rd: 15, imm: 1 }),
            TbOp::V16(Inst16::AddReg {
                rd: 0,
                rn: 15,
                rm: 1,
            }),
            TbOp::V16(Inst16::SpecialCmp { rn: 15, rm: 0 }),
            TbOp::V16(Inst16::BranchCond { cond: 14, off: 4 }),
            TbOp::V16(Inst16::CbzCbnz {
                rn: 15,
                off: 4,
                nonzero: false,
            }),
            TbOp::V16(Inst16::Bx { rm: 15 }),
            TbOp::V16(Inst16::Blx { rm: 15 }),
            TbOp::V16(Inst16::BxNs { rm: 2 }),
            TbOp::V32(crate::cpu::decode32::Inst32::BlxImm { off: 4 }),
            TbOp::V32(crate::cpu::decode32::Inst32::Mul {
                op: crate::cpu::decode32::MulOp::Smull,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            }),
            TbOp::V32(crate::cpu::decode32::Inst32::Div {
                signed: true,
                rd: 15,
                rn: 1,
                rm: 2,
            }),
            TbOp::V32(crate::cpu::decode32::Inst32::LdmStm {
                is_load: true,
                rn: 15,
                regs: 1,
                writeback: false,
                dec_before: false,
            }),
            TbOp::V32(crate::cpu::decode32::Inst32::LdmStm {
                is_load: false,
                rn: 1,
                regs: 0x8000,
                writeback: true,
                dec_before: false,
            }),
        ];
        for op in &cases {
            assert!(!is_templated(op), "{op:?} must bail");
        }
    }

    /// Branches touch no state: end state must equal input state exactly,
    /// and the exit must carry `BRANCH` + the resolved PC.
    fn check_branch(op: &TbOp, regs: &Registers, xpsr: Xpsr, pc: u32, want_pc: u32) {
        use crate::jit::native::{exit_pc, exit_reason, EXIT_BRANCH};

        assert!(is_templated(op));
        let ((nr, nx), code) = run_native(op, regs, &xpsr, pc, false);
        assert_eq!(exit_reason(code), EXIT_BRANCH);
        assert_eq!(exit_pc(code), want_pc, "{op:?}");
        assert_eq!((nr, nx), (*regs, xpsr), "branches are stateless");
    }

    fn blank_regs() -> Registers {
        Registers {
            r: [
                0x1111_1111,
                0x2222_2222,
                0x3333_3333,
                0x4444_4444,
                0x5555_5555,
                0x6666_6666,
                0x7777_7777,
                0x8888_8888,
                0x9999_9999,
                0xAAAA_AAAA,
                0xBBBB_BBBB,
                0xCCCC_CCCC,
                0xDDDD_DDDD,
            ],
            sp: 0x2000_1000,
            msp: 0x2000_1000,
            psp: 0x2000_2000,
            lr: 0x0800_2001,
            pc: 0x0800_1000,
        }
    }

    fn flags_nzcv(n: bool, z: bool, c: bool, v: bool) -> Xpsr {
        let mut x = Xpsr::default();
        x.set(
            0x0100_0000
                | (u32::from(n) << 31)
                | (u32::from(z) << 30)
                | (u32::from(c) << 29)
                | (u32::from(v) << 28),
        );
        x
    }

    #[test]
    fn branch_unconditional_matches() {
        let regs = blank_regs();
        let xpsr = flags_nzcv(true, false, true, false);
        // B self at 0x1000 (off -4) and B forward (off +6 -> 0x100A).
        check_branch(
            &TbOp::V16(Inst16::Branch { off: -4 }),
            &regs,
            xpsr,
            0x1000,
            0x1000,
        );
        check_branch(
            &TbOp::V16(Inst16::Branch { off: 6 }),
            &regs,
            xpsr,
            0x1000,
            0x100A,
        );
    }

    #[test]
    fn branch_cond_matches_all_conds_both_ways() {
        // (cond, take-flags, fall-flags) at pc 0x1000, off +6.
        let cases: &[(u8, Xpsr, Xpsr)] = &[
            (
                0,
                flags_nzcv(false, true, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                1,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, true, false, false),
            ),
            (
                2,
                flags_nzcv(false, false, true, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                3,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, false, true, false),
            ),
            (
                4,
                flags_nzcv(true, false, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                5,
                flags_nzcv(false, false, false, false),
                flags_nzcv(true, false, false, false),
            ),
            (
                6,
                flags_nzcv(false, false, false, true),
                flags_nzcv(false, false, false, false),
            ),
            (
                7,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, false, false, true),
            ),
            (
                8,
                flags_nzcv(false, false, true, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                8,
                flags_nzcv(false, false, true, false),
                flags_nzcv(false, true, true, false),
            ),
            (
                9,
                flags_nzcv(false, true, true, false),
                flags_nzcv(false, false, true, false),
            ),
            (
                10,
                flags_nzcv(false, false, false, false),
                flags_nzcv(true, false, false, false),
            ),
            (
                11,
                flags_nzcv(true, false, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                11,
                flags_nzcv(false, false, false, true),
                flags_nzcv(true, false, false, true),
            ),
            (
                12,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, true, false, false),
            ),
            (
                13,
                flags_nzcv(false, true, false, false),
                flags_nzcv(false, false, false, false),
            ),
        ];
        let regs = blank_regs();
        for (cond, take, fall) in cases {
            let op = TbOp::V16(Inst16::BranchCond {
                cond: *cond,
                off: 6,
            });
            check_branch(&op, &regs, *take, 0x1000, 0x100A);
            check_branch(&op, &regs, *fall, 0x1000, 0x1002);
        }
    }

    // ---- Memory templates (P2-3) ----

    use super::harness::{mem_bus, mem_regs, run_mem, run_mem_interp, MEM_RAM};
    use crate::jit::native::{exit_reason, EXIT_FALL, EXIT_MMIO};
    use crate::jit::tlb::Tlb;
    use crate::mem::bus::Bus;
    use crate::mem::flat::FlatMemory;

    /// Fresh harness triple per case (TLB + generation start cold/zero).
    fn mem_setup() -> (FlatMemory, Tlb, u64) {
        (mem_bus(), Tlb::new(), 0)
    }

    #[test]
    fn mem_word_imm_fast_path() {
        // STR then LDR word, imm offsets; fault_addr + values checked.
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x100;
        regs.r[2] = 0xDEAD_BEEF;
        let xpsr = Xpsr::default();
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let str_op = TbOp::V16(Inst16::LsImm {
            rt: 2,
            rn: 1,
            imm: 12,
            kind: crate::cpu::decode16::LsImm::Str,
        });
        let (st1, c1, fault, _, c2) = run_mem(
            &str_op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(fault, MEM_RAM + 0x10C);
        assert_eq!((st1.0, st1.1), (regs, xpsr), "miss is effect-free");
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert!(gen > 0, "fast store bumps the generation");
        let ldr_op = TbOp::V16(Inst16::LsImm {
            rt: 0,
            rn: 1,
            imm: 12,
            kind: crate::cpu::decode16::LsImm::Ldr,
        });
        // Fresh TLB (the STR above warmed the shared one — same page).
        let mut tlb2 = Tlb::new();
        let (_, _, _, (nr2, _, _), c4) = run_mem(
            &ldr_op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb2, &mut gen,
        );
        assert_eq!(exit_reason(c4), EXIT_FALL);
        assert_eq!(nr2.r[0], 0xDEAD_BEEF, "stored word reads back");
        // Interpreter parity on a twin bus (fast path must equal it).
        let (mut bus2, _, _) = mem_setup();
        bus2.write_word(MEM_RAM + 0x10C, 0xDEAD_BEEF).unwrap();
        let ((ir, _, _, _), err) = run_mem_interp(&ldr_op, &regs, &xpsr, 0x1000, bus2);
        assert!(err.is_none());
        assert_eq!(ir.r[0], nr2.r[0]);
    }

    #[test]
    fn mem_unmapped_side_exits_and_faults_identically() {
        // Unmapped data address: native side-exits (fill refuses), the
        // interpreter takes the fault; miss paths must be effect-free.
        let mut regs = mem_regs();
        regs.r[1] = 0x4000_0000;
        let xpsr = Xpsr::default();
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let op = TbOp::V16(Inst16::LsImm {
            rt: 0,
            rn: 1,
            imm: 0,
            kind: crate::cpu::decode16::LsImm::Ldr,
        });
        let (st1, c1, fault, st2, c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(fault, 0x4000_0000);
        assert_eq!(exit_reason(c2), EXIT_MMIO, "unfillable stays exited");
        assert_eq!((st1.0, st1.1), (regs, xpsr), "miss is effect-free");
        assert_eq!((st2.0, st2.1), (regs, xpsr), "miss is effect-free");
        let (bus2, _, _) = mem_setup();
        let ((ir, ix, _, ipsr), err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
        assert!(err.is_none(), "fault delivers via Advance, not Err");
        assert_eq!(ipsr, 3, "HardFault taken");
        assert_eq!(ix, xpsr, "fault preserves flags word shape");
    }

    /// Fast-path differential for one memory op: miss→fill→hit must equal
    /// the interpreter over a twin bus (regs, flags, RAM). `seed` prepares
    /// both buses identically (test data beyond the image pattern).
    fn check_mem_fast(
        op: &TbOp,
        regs: &Registers,
        pc: u32,
        trap: bool,
        seed: impl Fn(&mut FlatMemory),
    ) {
        use crate::jit::native::{exit_reason, EXIT_FALL, EXIT_MMIO};

        let xpsr = Xpsr::default();
        let (mut bus, mut tlb, mut gen) = mem_setup();
        seed(&mut bus);
        let (_, c1, _, st2, c2) = run_mem(op, regs, &xpsr, pc, trap, &mut bus, &mut tlb, &mut gen);
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(exit_reason(c2), EXIT_FALL, "{op:?} must go fast");
        let (mut bus2, _, _) = mem_setup();
        seed(&mut bus2);
        let ((ir, ix, ram_i, _), err) = run_mem_interp(op, regs, &xpsr, pc, bus2);
        assert!(err.is_none(), "{op:?} interpreter must not fault");
        // `pc` is driver-managed (no template reads `regs.pc`; literals
        // use the emit-time pc), so normalize it out of the comparison.
        let (mut nr, nx, nram) = st2;
        nr.pc = ir.pc;
        assert_eq!((nr, nx, nram), (ir, ix, ram_i), "{op:?}");
    }

    #[test]
    fn mem_sizes_and_sign_extension() {
        use crate::cpu::decode16::{LsImm, LsReg};
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x200;
        regs.r[2] = 0xA5A5_F00D;
        // Stores then loads across widths (STR/STRB/STRH + LDR/LDRB/LDRH).
        for (rt, imm, kind) in [
            (2u8, 0x00u16, LsImm::Str),
            (2u8, 0x10u16, LsImm::Strb),
            (2u8, 0x20u16, LsImm::Strh),
        ] {
            let op = TbOp::V16(Inst16::LsImm {
                rt,
                rn: 1,
                imm: imm as usize,
                kind,
            });
            check_mem_fast(&op, &regs, 0x1000, false, |_| {});
        }
        for (rt, imm, kind, want) in [
            (0u8, 0x00u16, LsImm::Ldr, 0xA5A5_F00Du32),
            (0u8, 0x10u16, LsImm::Ldrb, 0x0000_000Du32),
            (0u8, 0x20u16, LsImm::Ldrh, 0x0000_F00Du32),
        ] {
            // Seed the slot the matching store above wrote.
            let seed = move |bus: &mut FlatMemory| {
                bus.write_word(MEM_RAM + 0x200 + imm as u32, 0xA5A5_F00D)
                    .unwrap();
            };
            let op = TbOp::V16(Inst16::LsImm {
                rt,
                rn: 1,
                imm: imm as usize,
                kind,
            });
            check_mem_fast(&op, &regs, 0x1000, false, seed);
            // Spot-check the decoded value, not just parity.
            let (mut bus, mut tlb, mut gen) = mem_setup();
            seed(&mut bus);
            let xpsr = Xpsr::default();
            let (_, _, _, (nr, _, _), _) = run_mem(
                &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
            );
            assert_eq!(nr.r[rt as usize], want, "{op:?}");
        }
        // Register-offset + sign-extending loads.
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x300;
        regs.r[2] = 8;
        for kind in [LsReg::Ldrsb, LsReg::Ldrsh, LsReg::Ldrb] {
            let op = TbOp::V16(Inst16::LsReg {
                rt: 0,
                rn: 1,
                rm: 2,
                kind,
            });
            // Seed 0x80/0xFF80 at the effective address (base + 8).
            check_mem_fast(&op, &regs, 0x1000, false, |bus| {
                bus.write(MEM_RAM + 0x308, 0x80).unwrap();
                bus.write_word(MEM_RAM + 0x308, 0xFF80).unwrap();
            });
        }
        // Sign-extension value assertions (parity above already proves
        // equality; these pin the architected values).
        let op = TbOp::V16(Inst16::LsReg {
            rt: 0,
            rn: 1,
            rm: 2,
            kind: LsReg::Ldrsb,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write(MEM_RAM + 0x308, 0x80).unwrap();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), _) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(nr.r[0], 0xFFFF_FF80, "sign extends");
    }

    #[test]
    fn mem_sp_and_literal() {
        use crate::cpu::decode16::LsImm;
        // SP-relative store/load roundtrip (SP is never pinned: always the
        // memory path, always fresh).
        let mut regs = mem_regs();
        regs.sp = MEM_RAM + 0x400;
        regs.msp = MEM_RAM + 0x400;
        regs.r[0] = 0x1234_5678;
        let str_op = TbOp::V16(Inst16::LsSp {
            rt: 0,
            imm: 16,
            is_load: false,
        });
        check_mem_fast(&str_op, &regs, 0x1000, false, |_| {});
        let ldr_op = TbOp::V16(Inst16::LsSp {
            rt: 1,
            imm: 16,
            is_load: true,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write_word(MEM_RAM + 0x410, 0x1234_5678).unwrap();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), c2) = run_mem(
            &ldr_op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert_eq!(nr.r[1], 0x1234_5678);
        let _ = LsImm::Ldr;
        // PC-relative literal load (static base from the emit-time pc,
        // placed in RAM so the page maps).
        let lit_pc = MEM_RAM + 0x100;
        let lit_op = TbOp::V16(Inst16::LdrLiteral { rt: 0, imm: 8 });
        check_mem_fast(&lit_op, &regs, lit_pc, false, |bus| {
            // Base = Align(lit_pc + 4, 4); literal at +8.
            let base = (lit_pc.wrapping_add(4)) & !3;
            bus.write_word(base + 8, 0xCAFEBABE).unwrap();
        });
        // Spot-check the decoded literal value.
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let base = (lit_pc.wrapping_add(4)) & !3;
        bus.write_word(base + 8, 0xCAFEBABE).unwrap();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), c2) = run_mem(
            &lit_op, &regs, &xpsr, lit_pc, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert_eq!(nr.r[0], 0xCAFEBABE);
    }

    #[test]
    fn mem_pre_post_writeback_and_aliasing() {
        use crate::cpu::decode32::{Inst32, MemOp};
        // Post-indexed store with writeback, then rt==rn aliasing load:
        // access-then-writeback order must match the interpreter (the
        // loaded value lands first, the base update overwrites it).
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x500;
        regs.r[2] = 0x1111_2222;
        let post = TbOp::V32(Inst32::Mem {
            access: MemOp::Str,
            rt: 2,
            rn: 1,
            off: 4,
            rm: None,
            shift: 0,
            add: true,
            index: false,
            writeback: true,
            unprivileged: false,
        });
        check_mem_fast(&post, &regs, 0x1000, false, |_| {});
        // Verify the writeback actually advanced r1 (not just parity).
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), _) = run_mem(
            &post, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(nr.r[1], MEM_RAM + 0x504);
        // rt == rn aliasing load with writeback.
        let mut regs = mem_regs();
        regs.r[0] = MEM_RAM + 0x600;
        let alias = TbOp::V32(Inst32::Mem {
            access: MemOp::Ldr,
            rt: 0,
            rn: 0,
            off: 4,
            rm: None,
            shift: 0,
            add: true,
            index: true,
            writeback: true,
            unprivileged: false,
        });
        check_mem_fast(&alias, &regs, 0x1000, false, |_| {});
        // Pre-indexed with register offset + shift, unprivileged flag set
        // (ignored exactly like the interpreter — fidelity match).
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x700;
        regs.r[2] = 0x10;
        let regoff = TbOp::V32(Inst32::Mem {
            access: MemOp::Ldr,
            rt: 0,
            rn: 1,
            off: 0,
            rm: Some(2),
            shift: 2,
            add: true,
            index: true,
            writeback: true,
            unprivileged: true,
        });
        check_mem_fast(&regoff, &regs, 0x1000, false, |_| {});
    }

    #[test]
    fn mem_cross_page_side_exits() {
        use crate::cpu::decode16::LsImm;
        // Word straddling a page boundary (both pages mapped): the fast
        // path must still refuse (host adjacency unproven) and the
        // interpreter reference must agree on values.
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0xFFD;
        regs.r[2] = 0xAABB_CCDD;
        let xpsr = Xpsr::default();
        let op = TbOp::V16(Inst16::LsImm {
            rt: 2,
            rn: 1,
            imm: 0,
            kind: LsImm::Str,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let (_, c1, _, _, c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(exit_reason(c2), EXIT_MMIO, "page-crossing never goes fast");
        // The interpreter performs it fine — same bytes both sides would
        // see (proves the exit, not a fault).
        let (bus2, _, _) = mem_setup();
        let (_, err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
        assert!(err.is_none(), "cross-page store succeeds interpreted");
    }

    #[test]
    fn mem_unaligned_trap_modeled() {
        use crate::cpu::decode16::LsImm;
        // Halfword at an odd address: with the trap baked, side exit;
        // without, the x86 host performs it natively (AArch64 always
        // exits — conservative but value-identical via the interpreter).
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x101;
        regs.r[2] = 0xBEEF;
        let op = TbOp::V16(Inst16::LsImm {
            rt: 2,
            rn: 1,
            imm: 0,
            kind: LsImm::Strh,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let xpsr = Xpsr::default();
        let (_, c1, _, _, c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, true, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(exit_reason(c2), EXIT_MMIO, "trapped unaligned exits");
        #[cfg(target_arch = "x86_64")]
        {
            let (mut bus, mut tlb, mut gen) = mem_setup();
            let (_, c1, _, (nr, _, _), c2) = run_mem(
                &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
            );
            assert_eq!(exit_reason(c1), EXIT_MMIO);
            assert_eq!(exit_reason(c2), EXIT_FALL, "x64 performs in-page unaligned");
            let (bus2, _, _) = mem_setup();
            let ((ir, _, _, _), err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
            assert!(err.is_none());
            assert_eq!(nr.r[2], ir.r[2]);
        }
    }

    // ---- 32-bit + register-indirect branches (P2-4) ----

    use crate::cpu::decode32::Inst32 as I32;
    use crate::jit::native::EXIT_HALT;

    #[test]
    fn wide_branches_match() {
        let regs = blank_regs();
        let xpsr = flags_nzcv(true, false, true, false);
        // B.W self at 0x1000 (off -4) and forward (off +8 -> 0x100C).
        check_branch(
            &TbOp::V32(I32::BranchWide { off: -4 }),
            &regs,
            xpsr,
            0x1000,
            0x1000,
        );
        check_branch(
            &TbOp::V32(I32::BranchWide { off: 8 }),
            &regs,
            xpsr,
            0x1000,
            0x100C,
        );
        // Bcond.W reuses the cond matrix shape (taken/fall at pc+4 base).
        let regs = blank_regs();
        for (cond, take, fall) in [
            (
                0u8,
                flags_nzcv(false, true, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                1u8,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, true, false, false),
            ),
            (
                12u8,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, true, false, false),
            ),
            (
                13u8,
                flags_nzcv(false, true, false, false),
                flags_nzcv(false, false, false, false),
            ),
        ] {
            let op = TbOp::V32(I32::BranchCondWide { cond, off: 8 });
            check_branch(&op, &regs, take, 0x1000, 0x100C);
            check_branch(&op, &regs, fall, 0x1000, 0x1004);
        }
        // Full 14-condition sweep on Bcond.W (same flags as the 16-bit
        // matrix — the select is shared, the base differs).
        let cases: &[(u8, Xpsr, Xpsr)] = &[
            (
                0,
                flags_nzcv(false, true, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                2,
                flags_nzcv(false, false, true, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                4,
                flags_nzcv(true, false, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                6,
                flags_nzcv(false, false, false, true),
                flags_nzcv(false, false, false, false),
            ),
            (
                10,
                flags_nzcv(false, false, false, false),
                flags_nzcv(true, false, false, false),
            ),
        ];
        for (cond, take, fall) in cases {
            let op = TbOp::V32(I32::BranchCondWide {
                cond: *cond,
                off: -4,
            });
            check_branch(&op, &regs, *take, 0x1000, 0x1000);
            check_branch(&op, &regs, *fall, 0x1000, 0x1004);
        }
    }

    #[test]
    fn bl_links_lr_and_branches() {
        use crate::jit::native::{exit_pc, exit_reason, EXIT_BRANCH};

        let regs = blank_regs();
        let xpsr = flags_nzcv(false, false, false, false);
        // BL +0x100 at 0x1000: LR = 0x1005, target 0x1104.
        let op = TbOp::V32(I32::Bl { off: 0x100 });
        assert!(is_templated(&op));
        let ((nr, nx), code) = run_native(&op, &regs, &xpsr, 0x1000, false);
        assert_eq!(exit_reason(code), EXIT_BRANCH);
        assert_eq!(exit_pc(code), 0x1104);
        assert_eq!(nr.lr, 0x1005, "LR = (pc + 4) | 1");
        let mut want = regs;
        want.lr = 0x1005;
        assert_eq!((nr, nx), (want, xpsr), "only LR changes");
        // Interpreter parity (LR + target, not just shape).
        let (ir, _) = run_mem_interp_peek(&op, &regs, &xpsr);
        assert_eq!(ir.lr, 0x1005);
    }

    /// Interpreter peek for a single non-memory op (regs + flags only;
    /// memory ops use `run_mem_interp`).
    fn run_mem_interp_peek(op: &TbOp, regs: &Registers, xpsr: &Xpsr) -> (Registers, Xpsr) {
        use crate::cpu::Core;

        let mem = FlatMemory::with_layout(&[], &[]);
        let mut core = Core::armv7m(mem);
        core.regs = *regs;
        core.xpsr = *xpsr;
        core.itstate = 0;
        match *op {
            TbOp::V16(inst) => {
                core.exec_tb16(0x1000, inst).unwrap();
            }
            TbOp::V32(inst) => {
                core.exec_tb32(0x1000, inst).unwrap();
            }
        }
        (core.regs, core.xpsr)
    }

    /// Dynamic-branch check: `BRANCH` exit to the Thumb-cleared target
    /// with end state equal to input (BX) or LR-only delta (BLX). The
    /// target value is planted in the op's actual `Rm` lane (pinned lanes
    /// included — that path must work too).
    fn check_bx(op: &TbOp, rm_val: u32, pc: u32, want_pc: u32, want_lr: Option<u32>) {
        use crate::jit::native::{exit_pc, exit_reason, EXIT_BRANCH};

        assert!(is_templated(op));
        let rm = match *op {
            TbOp::V16(Inst16::Bx { rm }) | TbOp::V16(Inst16::Blx { rm }) => rm,
            _ => panic!("check_bx takes Bx/Blx only"),
        };
        let mut regs = blank_regs();
        if rm == 13 {
            regs.sp = rm_val;
        } else if rm == 14 {
            regs.lr = rm_val;
        } else {
            regs.r[rm as usize] = rm_val;
        }
        let xpsr = flags_nzcv(true, false, true, false);
        let ((nr, nx), code) = run_native(op, &regs, &xpsr, pc, false);
        assert_eq!(exit_reason(code), EXIT_BRANCH);
        assert_eq!(exit_pc(code), want_pc, "{op:?} with Rm = {rm_val:#X}");
        let mut want = regs;
        if let Some(lr) = want_lr {
            want.lr = lr;
        }
        assert_eq!(
            (nr, nx),
            (want, xpsr),
            "BX is stateless; BLX writes LR only"
        );
    }

    #[test]
    fn bx_fast_path_matches() {
        // BX r2 across Thumb targets (odd incl. the largest odd below
        // the magic floor, Thumb-bit cleared on exit); BLX additionally
        // links LR = (pc + 2) | 1. (0xFFFF_FFF9 and friends sit above
        // the floor — covered by the magic side-exit test.)
        for target in [0x0800_1001, 0x2000_0001, 0xFEFF_FFFD, 0x0000_0001] {
            check_bx(
                &TbOp::V16(Inst16::Bx { rm: 2 }),
                target,
                0x1000,
                target & !1,
                None,
            );
            check_bx(
                &TbOp::V16(Inst16::Blx { rm: 2 }),
                target,
                0x1000,
                target & !1,
                Some(0x1003),
            );
        }
        // Pinned register lane.
        check_bx(
            &TbOp::V16(Inst16::Bx { rm: 0 }),
            0x0800_2001,
            0x1000,
            0x0800_2000,
            None,
        );
        // LR lane (regression: the memory-path offset for r14 must be
        // `lr`, not the adjacent `msp` — `BX lr` is the common return).
        check_bx(
            &TbOp::V16(Inst16::Bx { rm: 14 }),
            0x0800_3001,
            0x1000,
            0x0800_3000,
            None,
        );
        check_bx(
            &TbOp::V16(Inst16::Blx { rm: 14 }),
            0x0800_3001,
            0x1000,
            0x0800_3000,
            Some(0x1003),
        );
    }

    #[test]
    fn high_reg_lanes_match() {
        // r13/r14 operand lanes use the `sp`/`lr` fields (same `mem_reg_off`
        // regression as `BX lr`): ALU destinations/sources plus a load with
        // an `lr` base must equal the interpreter exactly.
        check(&TbOp::V16(Inst16::MovImm { rd: 14, imm: 0x5A }), 128);
        check(
            &TbOp::V16(Inst16::AddReg {
                rd: 14,
                rn: 2,
                rm: 14,
            }),
            128,
        );
        check(
            &TbOp::V16(Inst16::AddReg {
                rd: 4,
                rn: 14,
                rm: 13,
            }),
            128,
        );
        let mut regs = mem_regs();
        regs.r[0] = MEM_RAM + 0x600;
        regs.lr = MEM_RAM + 0x600;
        let op = TbOp::V16(Inst16::LsImm {
            rt: 0,
            rn: 14,
            imm: 0,
            kind: crate::cpu::decode16::LsImm::Ldr,
        });
        check_mem_fast(&op, &regs, 0x1000, false, |_| {});
    }

    #[test]
    fn bx_even_target_halts_stateless() {
        use crate::jit::native::{exit_pc, exit_reason};

        // Even target: HALT exit carrying the op PC, no state touched
        // (the driver commits the prefix and reports NonThumbTarget).
        for (mk, target) in [
            (0u8, 0x0800_1000u32),
            (0u8, 0x0000_0000u32),
            (1u8, 0x2000_0002u32),
        ] {
            let op = if mk == 0 {
                TbOp::V16(Inst16::Bx { rm: 2 })
            } else {
                TbOp::V16(Inst16::Blx { rm: 2 })
            };
            let mut regs = blank_regs();
            regs.r[2] = target;
            let xpsr = flags_nzcv(false, true, false, true);
            let ((nr, nx), code) = run_native(&op, &regs, &xpsr, 0x1000, false);
            assert_eq!(exit_reason(code), EXIT_HALT, "{op:?} even target");
            assert_eq!(exit_pc(code), 0x1000, "HALT carries the op PC");
            assert_eq!((nr, nx), (regs, xpsr), "halt is effect-free");
        }
    }

    #[test]
    fn bx_magic_side_exits_effect_free() {
        use crate::jit::native::{exit_pc, exit_reason};

        // Magic targets (EXC_RETURN/FNC floor and above): MMIO-side exit
        // with the op PC, no state touched (the driver re-executes through
        // the interpreter, which owns the magic dispatch).
        for target in [0xFFFF_FFF9, 0xFFFF_FFFD, 0xFEFF_FFFE, 0xFF00_0000] {
            for mk in [0u8, 1u8] {
                let op = if mk == 0 {
                    TbOp::V16(Inst16::Bx { rm: 2 })
                } else {
                    TbOp::V16(Inst16::Blx { rm: 2 })
                };
                let mut regs = blank_regs();
                regs.r[2] = target;
                let xpsr = flags_nzcv(true, true, false, false);
                let ((nr, nx), code) = run_native(&op, &regs, &xpsr, 0x1000, false);
                assert_eq!(exit_reason(code), EXIT_MMIO, "{op:?} magic {target:#X}");
                assert_eq!(exit_pc(code), 0x1000, "side exit carries the op PC");
                assert_eq!((nr, nx), (regs, xpsr), "miss is effect-free");
            }
        }
    }

    // ---- Stack/multiple transfers (P2-5, S3) ----

    /// Regs with distinctive lane values plus a valid stack pointer.
    fn stack_regs() -> Registers {
        let mut regs = mem_regs();
        for (i, slot) in regs.r.iter_mut().enumerate() {
            *slot = 0x1111_1111u32.wrapping_add((i as u32).wrapping_mul(0x0101_0101));
        }
        regs.sp = MEM_SP;
        regs.msp = MEM_SP;
        regs.lr = 0x0800_3001;
        regs
    }

    #[test]
    fn push_pop_roundtrip() {
        // PUSH {r0-r3, lr} then POP {r4-r7}: sp round-trips, values land,
        // generation bumps on the store side only.
        let regs = stack_regs();
        let push = TbOp::V16(Inst16::Push {
            regs: 0x0F,
            include_lr: true,
        });
        check_mem_fast(&push, &regs, 0x1000, false, |_| {});
        // The pushed words read back through a POP (fresh TLB each op —
        // `check_mem_fast` builds its own).
        let mut regs = stack_regs();
        regs.r[4] = 0;
        regs.r[5] = 0;
        regs.r[6] = 0;
        regs.r[7] = 0;
        let pop = TbOp::V16(Inst16::Pop {
            regs: 0xF0,
            load_pc: false,
        });
        let seed = |bus: &mut FlatMemory| {
            // Simulate a PUSH {r0-r3}: words at sp-16..sp-4 hold r0-r3
            // of `stack_regs` (sp comes back to MEM_SP after the POP).
            let base = MEM_SP - 16;
            for (i, v) in [regs.r[0], regs.r[1], regs.r[2], regs.r[3]]
                .iter()
                .enumerate()
            {
                bus.write_word(base + 4 * i as u32, *v).unwrap();
            }
        };
        // POP reads from the pushed-down sp.
        let mut regs = stack_regs();
        regs.sp = MEM_SP - 16;
        regs.msp = MEM_SP - 16;
        regs.r[4] = 0;
        regs.r[5] = 0;
        regs.r[6] = 0;
        regs.r[7] = 0;
        check_mem_fast(&pop, &regs, 0x1000, false, seed);
        // Spot-check: r4-r7 got r0-r3's values and sp came back.
        let (mut bus, mut tlb, mut gen) = mem_setup();
        seed(&mut bus);
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), c2) = run_mem(
            &pop, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert_eq!(
            (nr.r[4], nr.r[5], nr.r[6], nr.r[7]),
            (nr.r[0], nr.r[1], nr.r[2], nr.r[3])
        );
        assert_eq!(nr.sp, MEM_SP);
        assert_eq!(gen, 0, "loads never bump the generation");
    }

    #[test]
    fn push_bumps_generation() {
        let regs = stack_regs();
        let push = TbOp::V16(Inst16::Push {
            regs: 0x03,
            include_lr: false,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let xpsr = Xpsr::default();
        let (_, c1, _, _, c2) = run_mem(
            &push, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert!(gen > 0, "fast multi-store bumps once per op");
    }

    #[test]
    fn pop_pc_branches_and_returns() {
        use crate::jit::native::{exit_pc, EXIT_BRANCH};
        // POP {r4, pc}: r4 loads, pc-word chains (dynamic edge, unlinked
        // here → driver exit with the Thumb-cleared target).
        let mut regs = stack_regs();
        regs.sp = MEM_SP - 8;
        regs.msp = MEM_SP - 8;
        regs.r[4] = 0;
        let op = TbOp::V16(Inst16::Pop {
            regs: 0x10,
            load_pc: true,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write_word(MEM_SP - 8, 0xAAAAAAAA).unwrap();
        bus.write_word(MEM_SP - 4, 0x0800_1001).unwrap();
        let xpsr = Xpsr::default();
        let (_, c1, _, (nr, _, _), c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO, "cold TLB side-exits");
        assert_eq!(exit_reason(c2), EXIT_BRANCH);
        assert_eq!(exit_pc(c2), 0x0800_1000, "Thumb bit cleared");
        assert_eq!(nr.r[4], 0xAAAAAAAA);
        assert_eq!(nr.sp, MEM_SP, "sp updates before the branch");
        // Interpreter parity on a twin bus (normalize pc: the driver owns
        // it for branch exits).
        let (mut bus2, _, _) = mem_setup();
        bus2.write_word(MEM_SP - 8, 0xAAAAAAAA).unwrap();
        bus2.write_word(MEM_SP - 4, 0x0800_1001).unwrap();
        let ((mut ir, _, _, _), err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
        assert!(err.is_none());
        ir.pc = nr.pc;
        let mut nr_fix = nr;
        nr_fix.pc = ir.pc;
        assert_eq!(nr_fix.r, ir.r);
        assert_eq!(nr_fix.sp, ir.sp);
    }

    #[test]
    fn ldm_stm_16_writeback_rules() {
        // STM {r0,r1} writes back rn even though rn is stored (mandatory
        // v6-M writeback); LDM with rn in the list keeps the loaded value.
        let mut regs = stack_regs();
        regs.r[0] = MEM_RAM + 0x300;
        regs.r[1] = 0x1111_1111;
        regs.r[2] = 0x2222_2222;
        let stm = TbOp::V16(Inst16::LdmStm {
            is_load: false,
            rn: 0,
            regs: 0x06,
        });
        check_mem_fast(&stm, &regs, 0x1000, false, |_| {});
        // LDM {r0 = base, r1}: r0 keeps the LOADED word, not base + 8.
        let mut regs = stack_regs();
        regs.r[0] = MEM_RAM + 0x400;
        let ldm = TbOp::V16(Inst16::LdmStm {
            is_load: true,
            rn: 0,
            regs: 0x03,
        });
        check_mem_fast(&ldm, &regs, 0x1000, false, |bus| {
            bus.write_word(MEM_RAM + 0x400, 0xDEAD_BEEF).unwrap();
            bus.write_word(MEM_RAM + 0x404, 0xCAFEBABE).unwrap();
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write_word(MEM_RAM + 0x400, 0xDEAD_BEEF).unwrap();
        bus.write_word(MEM_RAM + 0x404, 0xCAFEBABE).unwrap();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), _) = run_mem(
            &ldm, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(nr.r[0], 0xDEAD_BEEF, "loaded rn wins over writeback");
        assert_eq!(nr.r[1], 0xCAFEBABE);
    }

    // ---- 32-bit multiples, dual, table branch (P2-5) ----

    use crate::cpu::decode32::Inst32 as I32b;

    #[test]
    fn ldm_stm_32_ia_db_writeback() {
        // STMIA {r0,r1} + LDMIA back, then the DB forms (words below the
        // base, writeback re-derives symmetrically).
        let mut regs = stack_regs();
        regs.r[3] = MEM_RAM + 0x500;
        regs.r[0] = 0x1111_1111;
        regs.r[1] = 0x2222_2222;
        let stm = TbOp::V32(I32b::LdmStm {
            is_load: false,
            rn: 3,
            regs: 0x0003,
            writeback: true,
            dec_before: false,
        });
        check_mem_fast(&stm, &regs, 0x1000, false, |_| {});
        let ldm = TbOp::V32(I32b::LdmStm {
            is_load: true,
            rn: 3,
            regs: 0x0003,
            writeback: true,
            dec_before: false,
        });
        let mut regs = stack_regs();
        regs.r[3] = MEM_RAM + 0x500;
        check_mem_fast(&ldm, &regs, 0x1000, false, |bus| {
            bus.write_word(MEM_RAM + 0x500, 0x1111_1111).unwrap();
            bus.write_word(MEM_RAM + 0x504, 0x2222_2222).unwrap();
        });
        // DB store: words land below the base, `rn` steps down.
        let mut regs = stack_regs();
        regs.r[3] = MEM_RAM + 0x600;
        let stmdb = TbOp::V32(I32b::LdmStm {
            is_load: false,
            rn: 3,
            regs: 0x0003,
            writeback: true,
            dec_before: true,
        });
        check_mem_fast(&stmdb, &regs, 0x1000, false, |_| {});
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), c2) = run_mem(
            &stmdb, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert_eq!(nr.r[3], MEM_RAM + 0x600 - 8, "DB writeback steps down");
        // `regs` here are the stack defaults (r0 = 0x11111111, r1 laps to
        // 0x12121212): ascending stores land below the base in order.
        assert_eq!(bus.read_word(MEM_RAM + 0x5F8).unwrap(), 0x1111_1111);
        assert_eq!(bus.read_word(MEM_RAM + 0x5FC).unwrap(), 0x1212_1212);
        // DB load with `rn` in the list: the loaded value wins.
        let mut regs = stack_regs();
        regs.r[3] = MEM_RAM + 0x600;
        let ldmdb = TbOp::V32(I32b::LdmStm {
            is_load: true,
            rn: 3,
            regs: 0x000B,
            writeback: true,
            dec_before: true,
        });
        check_mem_fast(&ldmdb, &regs, 0x1000, false, |bus| {
            bus.write_word(MEM_RAM + 0x5F4, 0xAAAAAAAA).unwrap();
            bus.write_word(MEM_RAM + 0x5F8, 0xBBBBBBBB).unwrap();
            bus.write_word(MEM_RAM + 0x5FC, 0xCCCCCCCC).unwrap();
        });
    }

    #[test]
    fn ldm_pc_pop_branches() {
        use crate::jit::native::{exit_pc, EXIT_BRANCH};
        // LDMIA {r0, pc}: loads transfer, the `pc` word chains.
        let mut regs = stack_regs();
        regs.r[3] = MEM_RAM + 0x700;
        regs.r[0] = 0;
        let op = TbOp::V32(I32b::LdmStm {
            is_load: true,
            rn: 3,
            regs: 0x8001,
            writeback: true,
            dec_before: false,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write_word(MEM_RAM + 0x700, 0x1234_5678).unwrap();
        bus.write_word(MEM_RAM + 0x704, 0x0800_2001).unwrap();
        let xpsr = Xpsr::default();
        let (_, c1, _, (nr, nx, _), c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO, "cold TLB side-exits");
        assert_eq!(exit_reason(c2), EXIT_BRANCH);
        assert_eq!(exit_pc(c2), 0x0800_2000);
        assert_eq!(nr.r[0], 0x1234_5678);
        assert_eq!(
            nr.r[3],
            MEM_RAM + 0x708,
            "writeback lands before the branch"
        );
        // Interpreter parity on an identically seeded twin bus (normalize
        // pc: branch exits are driver-owned).
        let (mut bus2, _, _) = mem_setup();
        bus2.write_word(MEM_RAM + 0x700, 0x1234_5678).unwrap();
        bus2.write_word(MEM_RAM + 0x704, 0x0800_2001).unwrap();
        let ((ir, ix, _, _), err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
        assert!(err.is_none());
        let mut nr_fix = nr;
        nr_fix.pc = ir.pc;
        assert_eq!((nr_fix, nx), (ir, ix), "regs + flags match");
    }

    #[test]
    fn ldrd_strd_roundtrip() {
        // STRD post-indexed with writeback, then LDRD offset back.
        let mut regs = stack_regs();
        regs.r[1] = MEM_RAM + 0x800;
        regs.r[2] = 0xAAAAAAAA;
        regs.r[3] = 0xBBBBBBBB;
        let strd = TbOp::V32(I32b::LdrdStrd {
            is_load: false,
            rt: 2,
            rt2: 3,
            rn: 1,
            imm: 8,
            add: true,
            index: false,
            writeback: true,
        });
        check_mem_fast(&strd, &regs, 0x1000, false, |_| {});
        // Post-indexed: words at base, `rn` stepped by 8.
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), c2) = run_mem(
            &strd, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert_eq!(nr.r[1], MEM_RAM + 0x808);
        assert_eq!(bus.read_word(MEM_RAM + 0x800).unwrap(), 0xAAAAAAAA);
        assert_eq!(bus.read_word(MEM_RAM + 0x804).unwrap(), 0xBBBBBBBB);
        // LDRD literal pair (static base, no writeback).
        let lit_pc = MEM_RAM + 0x100;
        let ldrd = TbOp::V32(I32b::LdrdStrd {
            is_load: true,
            rt: 0,
            rt2: 1,
            rn: 15,
            imm: 8,
            add: true,
            index: true,
            writeback: false,
        });
        check_mem_fast(&ldrd, &regs, lit_pc, false, |bus| {
            let base = (lit_pc.wrapping_add(4)) & !3;
            bus.write_word(base + 8, 0x1234_5678).unwrap();
            bus.write_word(base + 12, 0x9ABC_DEF0).unwrap();
        });
    }

    #[test]
    fn tblbranch_targets() {
        use crate::jit::native::{exit_pc, EXIT_BRANCH};
        // TBB [r1, r2]: target = (pc + 4) + 2 * table[r1 + r2].
        let mut regs = stack_regs();
        regs.r[1] = MEM_RAM + 0x900;
        regs.r[2] = 4;
        let tbb = TbOp::V32(I32b::TblBranch {
            rn: 1,
            rm: 2,
            half: false,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write(MEM_RAM + 0x904, 0x10).unwrap();
        let xpsr = Xpsr::default();
        let (st1, c1, _, (nr, nx, _), c2) = run_mem(
            &tbb, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO, "cold table side-exits");
        assert_eq!(exit_reason(c2), EXIT_BRANCH);
        assert_eq!(exit_pc(c2), 0x1004 + 2 * 0x10, "target = (pc+4) + 2*index");
        assert_eq!((nr, nx), (regs, xpsr), "table branches touch no regs");
        assert_eq!((st1.0, st1.1), (regs, xpsr), "miss is effect-free");
        // TBH [r1, r2]: halfword index, scaled by 2 in the address.
        let mut regs = stack_regs();
        regs.r[1] = MEM_RAM + 0x900;
        regs.r[2] = 6;
        let tbh = TbOp::V32(I32b::TblBranch {
            rn: 1,
            rm: 2,
            half: true,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write_word(MEM_RAM + 0x906, 0x0020).unwrap();
        let (_, _, _, (_, _, _), c2) = run_mem(
            &tbh, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c2), EXIT_BRANCH);
        // Interpreter parity on an identically seeded twin bus: same branch
        // target (normalize pc), untouched regs.
        let mut regs_tb = stack_regs();
        regs_tb.r[1] = MEM_RAM + 0x900;
        regs_tb.r[2] = 4;
        let (mut bus2, _, _) = mem_setup();
        bus2.write(MEM_RAM + 0x904, 0x10).unwrap();
        let ((ir, ix, _, _), err) = run_mem_interp(&tbb, &regs_tb, &xpsr, 0x1000, bus2);
        assert!(err.is_none());
        assert_eq!(ir.pc, 0x1004 + 2 * 0x10, "interpreter agrees on target");
        assert_eq!((ir.r, ix), (regs_tb.r, xpsr));
    }

    #[test]
    fn multi_second_word_unmapped_stays_exited() {
        // Two-word LDM with the second word unmapped: phase-1 probes fail
        // before any transfer — effect-free on both sides, and the
        // interpreter faults identically (HardFault, nothing transferred).
        let mut regs = stack_regs();
        regs.r[0] = MEM_RAM + 0xFFC;
        let xpsr = Xpsr::default();
        let op = TbOp::V16(Inst16::LdmStm {
            is_load: true,
            rn: 0,
            regs: 0x03,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        // Bytes at the (mapped) first word: any partial transfer lands here.
        let bytes_at = |bus: &mut FlatMemory| {
            (0..4u32)
                .map(|k| bus.read(MEM_RAM + 0xFFC + k).unwrap())
                .collect::<Vec<_>>()
        };
        let before = bytes_at(&mut bus);
        let (st1, c1, fault, st2, c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(fault, MEM_RAM + 0xFFC, "first probe misses cold");
        assert_eq!(exit_reason(c2), EXIT_MMIO, "unmapped word never goes fast");
        assert_eq!((st1.0, st1.1), (regs, xpsr), "miss is effect-free");
        assert_eq!((st2.0, st2.1), (regs, xpsr), "miss is effect-free");
        assert_eq!(bytes_at(&mut bus), before, "no partial transfer");
        let (mut bus2, _, _) = mem_setup();
        // Word0's patterned content (loads before the word1 fault).
        let w0 = bus2.read_word(MEM_RAM + 0xFFC).unwrap();
        let ((ir, ix, _, ipsr), err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
        // The interpreter loads word0, then faults on word1 (entry or
        // error — bus-dependent): the native side must match that exact
        // shape when the driver re-executes (it does — same pre-op state
        // in, same fault out). What must NEVER happen is a *fast* transfer.
        assert!(err.is_some() || ipsr != 0, "interpreter faults too");
        assert_eq!(ir.r[0], w0, "word0 loads before the fault");
        assert_eq!(ir.r[1], regs.r[1], "word1 never lands");
        assert_eq!(ix, xpsr);
    }

    // ---- 32-bit data processing + extends/reversals (P2-5, S5) ----

    use crate::cpu::decode32::{DpOp as Dp, ShiftType as ST};

    /// All 16 DP ops over fixed lanes, both flag settings, rotated and
    /// unrotated immediates (carry paths), plus zero immediates.
    #[test]
    fn dp_imm_matches() {
        use crate::cpu::decode32::Inst32 as I32c;
        let ops = [
            Dp::And,
            Dp::Eor,
            Dp::Bic,
            Dp::Orr,
            Dp::Orn,
            Dp::Mov,
            Dp::Mvn,
            Dp::Add,
            Dp::Adc,
            Dp::Sbc,
            Dp::Sub,
            Dp::Rsb,
            Dp::Tst,
            Dp::Teq,
            Dp::Cmp,
            Dp::Cmn,
        ];
        for (k, op) in ops.iter().enumerate() {
            for (rd, rn) in [(1u8, 4u8), (0u8, 2u8)] {
                for (imm12, expand) in [(0x123u16, true), (0xABC, true), (0x0FF, false), (0, true)]
                {
                    for set_flags in [true, false] {
                        let inst = TbOp::V32(I32c::DpImm {
                            op: *op,
                            rd,
                            rn,
                            imm12,
                            expand,
                            set_flags,
                        });
                        check(&inst, 48 + k as u32);
                    }
                }
            }
        }
    }

    /// Register-shifted op2: every shift kind at telling amounts (0, 1,
    /// 31) plus RRX, crossed with logical + arithmetic ops, flags on/off.
    #[test]
    fn dp_reg_matches() {
        use crate::cpu::decode32::Inst32 as I32c;
        for (shift, amount) in [
            (ST::Lsl, 0u8),
            (ST::Lsl, 5),
            (ST::Lsl, 31),
            (ST::Lsr, 1),
            (ST::Lsr, 31),
            (ST::Asr, 3),
            (ST::Asr, 31),
            (ST::Ror, 4),
            (ST::Ror, 0),
        ] {
            for op in [Dp::Add, Dp::And, Dp::Orr, Dp::Sub, Dp::Mov, Dp::Cmp] {
                for set_flags in [true, false] {
                    let inst = TbOp::V32(I32c::DpReg {
                        op,
                        rd: 1,
                        rn: 4,
                        rm: 2,
                        shift,
                        amount,
                        set_flags,
                    });
                    check(&inst, 48);
                    // Pinned-lane variant.
                    let inst = TbOp::V32(I32c::DpReg {
                        op,
                        rd: 0,
                        rn: 2,
                        rm: 3,
                        shift,
                        amount,
                        set_flags,
                    });
                    check(&inst, 48);
                }
            }
        }
    }

    #[test]
    fn movwide_mul_div_extend_reverse_match() {
        use crate::cpu::decode16::{ExtendKind as E, ReverseKind as R};
        use crate::cpu::decode32::Inst32 as I32c;
        use crate::cpu::decode32::MulOp as M;
        // MOVW clears, MOVT preserves-then-fills.
        for (rd, imm, high) in [(0u8, 0u16, false), (1, 0xFFFF, false), (2, 0x1234, true)] {
            check(&TbOp::V32(I32c::MovWide { rd, imm, high }), 128);
        }
        // MUL/MLA/MLS over edge products.
        for op in [M::Mul, M::Mla, M::Mls] {
            let inst = TbOp::V32(I32c::Mul {
                op,
                rd: 1,
                rn: 4,
                rm: 2,
                ra: 3,
            });
            check(&inst, 256);
        }
        // DIV signed/unsigned over sign/edge divisors (zero divisor is a
        // side exit — covered separately below).
        for signed in [true, false] {
            let inst = TbOp::V32(I32c::Div {
                signed,
                rd: 1,
                rn: 4,
                rm: 2,
            });
            check_one(
                &inst,
                128,
                &[
                    (4, &[0, 1, 7, 100, 0xFFFF_FFFF, 0x8000_0000]),
                    (2, &[1, 2, 3, 7, 0xFFFF_FFFF]),
                ],
            );
        }
        // Signed MIN / -1 saturates (x86 would fault — pin it).
        let inst = TbOp::V32(I32c::Div {
            signed: true,
            rd: 0,
            rn: 2,
            rm: 3,
        });
        check_one(&inst, 16, &[(2, &[0x8000_0000]), (3, &[0xFFFF_FFFF])]);
        let mut regs = blank_regs();
        regs.r[2] = 0x8000_0000;
        regs.r[3] = 0xFFFF_FFFF;
        let ((nr, _), code) = run_native(&inst, &regs, &Xpsr::default(), 0x1000, false);
        assert_eq!(exit_reason(code), EXIT_FALL);
        assert_eq!(nr.r[0], 0x8000_0000, "MIN / -1 saturates");
        // Extends/reversals over sign/byte edges.
        for kind in [E::Sxth, E::Sxtb, E::Uxth, E::Uxtb] {
            check(&TbOp::V16(Inst16::Extend { rd: 1, rm: 4, kind }), 128);
        }
        for kind in [R::Rev, R::Rev16, R::Revsh] {
            check(&TbOp::V16(Inst16::Reverse { rd: 1, rm: 4, kind }), 256);
        }
    }

    #[test]
    fn div_by_zero_side_exits() {
        use crate::cpu::decode32::Inst32 as I32c;
        use crate::jit::native::EXIT_MMIO;
        // Zero divisor: MMIO-side exit, effect-free; the interpreter owns
        // `DIV_0_TRP`-vs-zero (here: zero result, like the empty bus).
        for signed in [true, false] {
            let op = TbOp::V32(I32c::Div {
                signed,
                rd: 1,
                rn: 4,
                rm: 2,
            });
            let mut regs = blank_regs();
            regs.r[4] = 100;
            regs.r[2] = 0;
            let xpsr = flags_nzcv(true, false, true, false);
            let ((nr, nx), code) = run_native(&op, &regs, &xpsr, 0x1000, false);
            assert_eq!(exit_reason(code), EXIT_MMIO);
            assert_eq!((nr, nx), (regs, xpsr), "miss is effect-free");
            let (ir, _) = run_interp(&op, &regs, &xpsr);
            assert_eq!(ir.r[1], 0, "empty-bus CCR leaves TRP clear");
        }
    }

    // ---- IT blocks (P2-5, S4) ----

    #[test]
    fn it_advance_matches_interpreter() {
        use crate::cpu::Core;
        // Drive real `IT EQ` (0xBF08) + two predicated steps through the
        // stepper; the static simulation must match its ITSTATE trajectory
        // exactly (independent implementation — the step path).
        let mut bus = FlatMemory::with_layout(&[0x1000], &[0x100]);
        bus.write(0x1000, 0x08).unwrap();
        bus.write(0x1001, 0xBF).unwrap();
        bus.write(0x1002, 0x01).unwrap();
        bus.write(0x1003, 0x20).unwrap(); // MOVS r0, #1
        bus.write(0x1004, 0x02).unwrap();
        bus.write(0x1005, 0x20).unwrap(); // MOVS r0, #2
        let mut core = Core::armv7m(bus);
        core.regs.pc = 0x1000;
        core.xpsr.set(0x0100_0000); // Z clear → EQ fails (suppressed)
        core.step().unwrap();
        let it0 = core.itstate;
        assert_ne!(it0, 0, "IT sets state");
        core.step().unwrap();
        assert_eq!(core.itstate, it_advance(it0), "advance 1 matches");
        let it1 = core.itstate;
        core.step().unwrap();
        assert_eq!(core.itstate, it_advance(it1), "advance 2 matches");
        // Drain trajectories pin the endpoints too.
        assert_eq!(it_advance(0x08), 0x00, "single-THEN drains");
        assert_eq!(it_advance(0), 0, "zero stays zero");
    }

    #[test]
    fn it_setter_stores_state() {
        // Single `IT` template: FALL exit, state untouched, scratch holds
        // the set value (the exit store covers it in blocks).
        let op = TbOp::V16(Inst16::It {
            cond: 2,
            mask: 0x1C,
        });
        let regs = blank_regs();
        let xpsr = flags_nzcv(true, false, true, false);
        let ((nr, nx), code) = run_native(&op, &regs, &xpsr, 0x1000, false);
        assert_eq!(exit_reason(code), EXIT_FALL);
        assert_eq!((nr, nx), (regs, xpsr), "IT touches no state itself");
        // The block-level store is covered by `predicated_block_emission`
        // below; here pin the pure assignment value.
        assert_eq!((2u8 << 4) | 0x1C, 0x3C);
    }

    #[test]
    fn predicated_block_emission() {
        use crate::jit::native::{exit_reason, NativeCtx, EXIT_FALL};
        use crate::jit::tlb::Tlb;
        // Block [ADDS r0, r1] entered under EQ-with-one-to-go (0x04):
        // taken (Z=1) updates Rd with flags suppressed; skipped (Z=0)
        // leaves everything; scratch ITSTATE advances either way.
        let body = TbBody {
            pc: 0x1000,
            secure: false,
            isa: crate::cpu::Isa::Armv7M,
            ops: vec![TbOp::V16(Inst16::AddReg {
                rd: 0,
                rn: 0,
                rm: 1,
            })],
            end_pc: 0x1002,
            end: TbEnd::MaxInsns,
            start_page: 1,
            page_count: 1,
            cycle_estimate: 1,
            store_mask: 0,
            it_entry: 0x04,
        };
        let (statics, has_dynamic) = exit_shape(&body);
        assert_eq!((statics.clone(), has_dynamic), (vec![0x1002], false));
        let slots: Vec<LinkSlot> = statics.into_iter().map(LinkSlot::unlinked).collect();
        let block = emit_block(&body, false, &slots).expect("predicated block emits");
        let run = |regs: Registers, xpsr: Xpsr| {
            let mut r = regs;
            let mut x = xpsr;
            let mut it = 0u8;
            let mut tlb = Tlb::new();
            let mut gen = 0u64;
            let mut ctx = Box::new(NativeCtx {
                regs: &mut r,
                xpsr: &mut x,
                tlb: &mut tlb as *mut Tlb,
                privileged: 1,
                write_gen: &mut gen as *mut u64,
                itstate: &mut it,
                budget_ops: i64::MAX,
                budget_blocks: i64::MAX,
                ..Default::default()
            });
            let code =
                vioarm_jit_call::call1(block.entry_ptr(), &mut *ctx as *mut NativeCtx as u64);
            ((r, x, it), code)
        };
        // Taken: EQ holds (Z=1) — Rd updates, flags suppressed, state advances.
        let mut regs = blank_regs();
        regs.r[0] = 10;
        regs.r[1] = 5;
        let xpsr = flags_nzcv(false, true, true, false);
        let ((nr, nx, it), code) = run(regs, xpsr);
        assert_eq!(exit_reason(code), EXIT_FALL);
        assert_eq!(nr.r[0], 15);
        assert_eq!(nx, xpsr, "predicated S-forms suppress the merge");
        assert_eq!(it, it_advance(0x04));
        // Skipped: EQ fails (Z=0) — nothing moves except ITSTATE.
        let xpsr = flags_nzcv(false, false, true, false);
        let ((nr, nx, it), code) = run(regs, xpsr);
        assert_eq!(exit_reason(code), EXIT_FALL);
        assert_eq!((nr, nx), (regs, xpsr), "suppressed means suppressed");
        assert_eq!(it, it_advance(0x04));
    }

    #[test]
    fn exit_itstate_simulation() {
        // Pure simulation pins: plain (0), setter, drain, entry-advance.
        let plain = TbBody {
            pc: 0x1000,
            secure: false,
            isa: crate::cpu::Isa::Armv7M,
            ops: vec![TbOp::V16(Inst16::MovImm { rd: 0, imm: 1 })],
            end_pc: 0x1002,
            end: TbEnd::MaxInsns,
            start_page: 1,
            page_count: 1,
            cycle_estimate: 1,
            store_mask: 0,
            it_entry: 0,
        };
        assert_eq!(exit_itstate(&plain), 0);
        let set = TbBody {
            ops: vec![TbOp::V16(Inst16::It { cond: 1, mask: 8 })],
            ..plain.clone()
        };
        assert_eq!(exit_itstate(&set), (1 << 4) | 8);
        let drain = TbBody {
            ops: vec![
                TbOp::V16(Inst16::AddReg {
                    rd: 0,
                    rn: 0,
                    rm: 1,
                }),
                TbOp::V16(Inst16::AddReg {
                    rd: 0,
                    rn: 0,
                    rm: 1,
                }),
            ],
            it_entry: 0x08,
            ..plain.clone()
        };
        assert_eq!(exit_itstate(&drain), it_advance(it_advance(0x08)));
    }

    #[test]
    fn ldm_stm_16_unaligned_base_faults() {
        // Thumb-1 multiples fault on unaligned bases (trap-independent):
        // side exit, effect-free, interpreter agrees.
        let mut regs = stack_regs();
        regs.r[0] = MEM_RAM + 0x301;
        let xpsr = Xpsr::default();
        let op = TbOp::V16(Inst16::LdmStm {
            is_load: true,
            rn: 0,
            regs: 0x01,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let (st1, c1, _, st2, c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(exit_reason(c2), EXIT_MMIO, "alignment never goes fast");
        assert_eq!((st1.0, st1.1), (regs, xpsr), "miss is effect-free");
        assert_eq!((st2.0, st2.1), (regs, xpsr), "miss is effect-free");
    }

    #[test]
    fn cbz_cbnz_match_both_ways() {
        let regs = blank_regs();
        let xpsr = flags_nzcv(false, false, true, true);
        // CBZ r2 (r2 != 0 in blank_regs): falls.
        check_branch(
            &TbOp::V16(Inst16::CbzCbnz {
                rn: 2,
                off: 8,
                nonzero: false,
            }),
            &regs,
            xpsr,
            0x1000,
            0x1002,
        );
        // CBNZ r2: takes.
        check_branch(
            &TbOp::V16(Inst16::CbzCbnz {
                rn: 2,
                off: 8,
                nonzero: true,
            }),
            &regs,
            xpsr,
            0x1000,
            0x100C,
        );
        // CBZ r12 == 0 in blank_regs? r12 = 0xDDDD_DDDD (nonzero). Use r0
        // via a zeroed copy for the taken-CBZ direction.
        let mut zeroed = regs;
        zeroed.r[0] = 0;
        check_branch(
            &TbOp::V16(Inst16::CbzCbnz {
                rn: 0,
                off: 8,
                nonzero: false,
            }),
            &zeroed,
            xpsr,
            0x1000,
            0x100C,
        );
    }
}
