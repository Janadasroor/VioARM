//! x86-64 integer ALU templates (S0, P2-2).
//!
//! Each template emits straight-line code for one [`TbOp`]: load operands
//! (pinned `r0`–`r3` from `rbx`/`r12d`–`r14d`, the rest from `Core` via
//! the spill-slot context reload), compute with host flags, store the
//! result, then merge NZCV into the pinned flags word (`r15d`). The merge
//! helpers encode the interpreter's exact flag contracts (`cpu/mod.rs`):
//! `merge_nzcv` replaces all four flags (ADD/SUB/ADC/SBC/CMP/CMN/RSB and
//! shifts, whose V is always false); `merge_nz` replaces N/Z, preserves C,
//! and clears V (MOV/MVN/AND/ORR/EOR/MUL/TST and zero-count shifts).
//!
//! Ordering rule (load-bearing): the result is stored BEFORE `lahf`
//! (`lahf` overwrites `ah`, i.e. result bits 15:8), and `seto dl` runs
//! before any flag-clobbering instruction after the compute. `mov`
//! never touches flags, so stores/loads are freely interleaved.
//!
//! Clobbers per template: `eax` (result/op1), `ecx` (op2), `esi`/`edx`
//! inside merges (operands dead by then). `rdi`/`rsi` reused as scratch
//! only after their operand role ends.

use dynasmrt::{dynasm, DynasmApi, DynasmLabelApi};

use crate::cpu::decode16::ShiftImm;
use crate::cpu::registers::Registers;
use crate::jit::builder::TbOp;

/// Concrete assembler for this host (re-exported as `arch::Assembler`).
pub type Assembler = dynasmrt::x64::Assembler;

type A = Assembler;

/// Define a marker label at the current position (slot/chain plumbing).
pub fn mark_label(ops: &mut A, lbl: dynasmrt::DynamicLabel) {
    dynasm!(ops ; .arch x64 ; =>lbl);
}

/// Driver-entry jump over the chained-entry stub (the stub is a chaining
/// resource: the entered block must not consume budgets/accumulators).
/// Returns the body label for binding after the stub.
pub fn emit_entry_jump(ops: &mut A) -> dynasmrt::DynamicLabel {
    let body = ops.new_dynamic_label();
    dynasm!(ops ; .arch x64 ; jmp =>body);
    body
}

/// `FALL` edge to `end_pc` through `slot` (sequential chaining).
pub fn emit_fall_edge(ops: &mut A, end_pc: u32, slot: u64) {
    super::native::emit_static_edge(ops, super::native::EXIT_FALL, end_pc, slot, 0);
}

fn regs_off() -> i32 {
    core::mem::offset_of!(crate::jit::NativeCtx, regs) as i32
}

fn reg_off(i: usize) -> i32 {
    (core::mem::offset_of!(Registers, r) + i * 4) as i32
}

/// Memory-path offset for an architectural register number, mirroring
/// [`Registers::read`]/`write`: 13 is the active `sp`, 14 is `lr`
/// (NOT the adjacent `msp` — raw array indexing ends at `r[12]`; the
/// `BX lr` return caught this: `reg_off(14)` read `msp`).
fn mem_reg_off(guest: u8) -> i32 {
    match guest {
        13 => core::mem::offset_of!(Registers, sp) as i32,
        14 => core::mem::offset_of!(Registers, lr) as i32,
        _ => reg_off(guest as usize),
    }
}

/// Load guest `r[guest]` into `eax` (`true`) or `ecx` (`false`).
fn load(ops: &mut A, guest: u8, into_eax: bool) {
    let off = mem_reg_off(guest);
    match (guest, into_eax) {
        (0, true) => dynasm!(ops ; .arch x64 ; mov eax, ebx),
        (1, true) => dynasm!(ops ; .arch x64 ; mov eax, r12d),
        (2, true) => dynasm!(ops ; .arch x64 ; mov eax, r13d),
        (3, true) => dynasm!(ops ; .arch x64 ; mov eax, r14d),
        (0, false) => dynasm!(ops ; .arch x64 ; mov ecx, ebx),
        (1, false) => dynasm!(ops ; .arch x64 ; mov ecx, r12d),
        (2, false) => dynasm!(ops ; .arch x64 ; mov ecx, r13d),
        (3, false) => dynasm!(ops ; .arch x64 ; mov ecx, r14d),
        _ => {
            let ctx = regs_off();
            if into_eax {
                dynasm!(ops
                    ; .arch x64
                    ; mov rsi, [rsp]
                    ; mov rsi, [rsi + ctx]
                    ; mov eax, [rsi + off]
                );
            } else {
                dynasm!(ops
                    ; .arch x64
                    ; mov rsi, [rsp]
                    ; mov rsi, [rsi + ctx]
                    ; mov ecx, [rsi + off]
                );
            }
        }
    }
}

/// Store `eax` into guest `r[guest]`.
fn store(ops: &mut A, guest: u8) {
    let off = mem_reg_off(guest);
    match guest {
        0 => dynasm!(ops ; .arch x64 ; mov ebx, eax),
        1 => dynasm!(ops ; .arch x64 ; mov r12d, eax),
        2 => dynasm!(ops ; .arch x64 ; mov r13d, eax),
        3 => dynasm!(ops ; .arch x64 ; mov r14d, eax),
        _ => {
            let ctx = regs_off();
            dynasm!(ops
                ; .arch x64
                ; mov rsi, [rsp]
                ; mov rsi, [rsi + ctx]
                ; mov [rsi + off], eax
            );
        }
    }
}

/// Merge full NZCV from `ah` ( LAHF image) + `dl` (V) into `r15d`.
/// Clobbers `ecx`, `esi`; preserves `eax`.
fn merge_nzcv(ops: &mut A) {
    dynasm!(ops
        ; .arch x64
        ; movzx ecx, ah
        ; mov esi, ecx
        ; and esi, 0x80
        ; shl esi, 24
        ; and ecx, 0x40
        ; shl ecx, 24
        ; or esi, ecx
        ; movzx ecx, ah
        ; and ecx, 1
        ; shl ecx, 29
        ; or esi, ecx
        ; movzx ecx, dl
        ; shl ecx, 28
        ; or esi, ecx
        ; mov ecx, r15d
        ; and ecx, 0x0FFFFFFF
        ; or ecx, esi
        ; mov r15d, ecx
    );
}

/// Merge N/Z from `ah`, preserve C, clear V (matches `set_nz(res, false)`).
/// Clobbers `ecx`, `esi`.
fn merge_nz(ops: &mut A) {
    dynasm!(ops
        ; .arch x64
        ; movzx ecx, ah
        ; mov esi, ecx
        ; and esi, 0x80
        ; shl esi, 24
        ; and ecx, 0x40
        ; shl ecx, 24
        ; or esi, ecx
        ; mov ecx, r15d
        ; and ecx, 0x2FFFFFFF
        ; or ecx, esi
        ; mov r15d, ecx
    );
}

/// Emit one templated op at guest `pc`. Returns `false` when `op` has no
/// template (the block then stays interpreted — see `is_templated`).
/// Plain ops only: terminators go through [`emit_terminator`] (they
/// consume edge slots). `unalign_trap` is the emit-time CCR bit (P2-3:
/// stable across the block by store-seal + config purge).
pub fn emit_op(ops: &mut A, op: &TbOp, pc: u32, unalign_trap: bool, merge: bool) -> bool {
    match *op {
        TbOp::V16(inst) => emit16(ops, &inst, pc, unalign_trap, false, merge),
        TbOp::V32(inst) => emit32(ops, &inst, pc, unalign_trap),
    }
}

/// Emit one predicated op under IT condition `cond` (P2-5, S4): evaluate
/// the condition against the pinned flags, skip the whole template on
/// failure (suppressed ops have no effects — faults included, since side
/// exits sit inside the skipped region). 16-bit S-forms additionally
/// suppress their flag merge (`pred` — the interpreter's `_it` variants);
/// test forms, flagless ops, and 32-bit DP (S-bit honored in IT) merge as
/// usual. Returns `false` for non-plain ops (archy double-gate: the
/// templates-level `is_predicable` decides, this fails closed).
pub fn emit_predicated(
    ops: &mut A,
    op: &TbOp,
    pc: u32,
    unalign_trap: bool,
    cond: u8,
    merge: bool,
) -> bool {
    use super::templates::is_predicable;
    // Single gate (coverage + cond range): the templates-level simulation
    // only sends predicable ops, this fails closed on anything else
    // (memory ops must never predicate — their side exits skip the IT
    // advance the interpreter performs on re-execution).
    if cond > 13 || !is_predicable(op) {
        return false;
    }
    // Condition bit into `sil` + host ZF (shares `cond_bit` with the
    // branch pairs — taken-ness, not the target select).
    cond_bit(ops, cond);
    let skip = ops.new_dynamic_label();
    dynasm!(ops ; .arch x64 ; jz =>skip);
    let ok = match *op {
        TbOp::V16(inst) => emit16(ops, &inst, pc, unalign_trap, true, merge),
        // 32-bit predicated forms (S-bit honored in IT — no suppression).
        TbOp::V32(inst) => emit32(ops, &inst, pc, unalign_trap),
    };
    dynasm!(ops ; .arch x64 ; =>skip);
    ok
}

/// Emit a block terminator (P2-2 static branches, P2-4 register-indirect
/// and 32-bit branches). Consumes its edge slots from `tape` in
/// [`exit_shape`](super::templates::exit_shape) order and returns `false`
/// for untemplated forms (the block then stays interpreted).
pub fn emit_terminator(
    ops: &mut A,
    op: &TbOp,
    pc: u32,
    unalign_trap: bool,
    tape: &mut super::templates::SlotTape<'_>,
) -> bool {
    // `unalign_trap` feeds table-branch sites; other terminators ignore it.
    match *op {
        TbOp::V16(inst) => emit16_term(ops, &inst, pc, tape),
        TbOp::V32(inst) => emit32_term(ops, &inst, pc, unalign_trap, tape),
    }
}

/// NativeCtx field offsets (all `i32` immediates below).
fn tlb_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, tlb) as i32
}

fn priv_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, privileged) as i32
}

fn epoch_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, write_gen) as i32
}

fn fault_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, fault_addr) as i32
}

/// 16-bit control flow is `pc + 4`-relative.
fn b_target(pc: u32, off: i32) -> u32 {
    pc.wrapping_add(4).wrapping_add_signed(off)
}

fn lr_off() -> i32 {
    core::mem::offset_of!(Registers, lr) as i32
}

/// `pred` suppresses flag merges for 16-bit S-forms executing under
/// ITSTATE (P2-5, S4 — the interpreter's `_it` variants); test forms,
/// flagless ops, and 32-bit DP ignore it.
fn emit16(
    ops: &mut A,
    inst: &crate::cpu::decode16::Inst16,
    pc: u32,
    unalign_trap: bool,
    pred: bool,
    merge: bool,
) -> bool {
    use crate::cpu::decode16::Inst16 as I;
    match *inst {
        I::MovImm { rd, imm } => {
            if rd == 15 {
                return false;
            }
            let imm = i32::from(imm);
            dynasm!(ops ; .arch x64 ; mov eax, imm);
            store(ops, rd);
            if !pred {
                dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
                merge_nz(ops);
            }
            true
        }
        I::AddReg { rd, rn, rm } => alu_rr(ops, rd, rn, rm, AluKind::Add, pred, merge),
        I::SubReg { rd, rn, rm } => alu_rr(ops, rd, rn, rm, AluKind::Sub, pred, merge),
        I::AddImm3 { rd, rn, imm } => {
            alu_ri(ops, rd, rn, i32::from(imm), AluKind::Add, pred, merge)
        }
        I::SubImm3 { rd, rn, imm } => {
            alu_ri(ops, rd, rn, i32::from(imm), AluKind::Sub, pred, merge)
        }
        I::AddImm8 { rd, imm } => alu_ri(ops, rd, rd, i32::from(imm), AluKind::Add, pred, merge),
        I::SubImm8 { rd, imm } => alu_ri(ops, rd, rd, i32::from(imm), AluKind::Sub, pred, merge),
        I::CmpImm { rn, imm } => cmp_ri(ops, rn, i32::from(imm), merge),
        I::ShiftImm {
            rd,
            rm,
            kind,
            amount,
        } => emit_shift_imm(ops, rd, rm, kind, amount, pred, merge),
        I::ShiftReg { rd, rn, rm, kind } => {
            if rd == 15 || rn == 15 || rm == 15 {
                return false;
            }
            load(ops, rn, true);
            load(ops, rm, false);
            shift_reg_seq(ops, rd, kind, pred)
        }
        I::Alu { rd, rm, op } => emit_alu(ops, rd, rm, op, pred, merge),
        I::SpecialCmp { rn, rm } => {
            if rn == 15 || rm == 15 {
                return false;
            }
            // Pure flags (G3 flag traffic): vanish entirely when dead.
            if !merge {
                return true;
            }
            load(ops, rn, true);
            load(ops, rm, false);
            dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc ; seto dl ; lahf);
            merge_nzcv(ops);
            true
        }
        // --- High-reg add/move + SP arithmetic (split-generation
        // gap-fillers): plain ALU, never any flags.
        I::SpecialAdd { rd, rm } => {
            if rd == 15 || rm == 15 {
                return false;
            }
            load(ops, rd, true);
            load(ops, rm, false);
            dynasm!(ops ; .arch x64 ; add eax, ecx);
            store(ops, rd);
            true
        }
        I::SpecialMov { rd, rm } => {
            if rd == 15 || rm == 15 {
                return false;
            }
            load(ops, rm, true);
            store(ops, rd);
            true
        }
        I::AddSpImm8 { imm } => {
            load(ops, 13, true);
            let imm = imm as i32;
            dynasm!(ops ; .arch x64 ; add eax, imm);
            store(ops, 13);
            true
        }
        I::SubSpImm8 { imm } => {
            load(ops, 13, true);
            let imm = imm as i32;
            dynasm!(ops ; .arch x64 ; sub eax, imm);
            store(ops, 13);
            true
        }
        I::AddrSp { rd, imm } => {
            load(ops, 13, true);
            let imm = imm as i32;
            dynasm!(ops ; .arch x64 ; add eax, imm);
            store(ops, rd);
            true
        }
        I::AddrPc { rd, imm } => {
            let v = pc.wrapping_add(4) & !3;
            let v = v.wrapping_add(imm as u32) as i32;
            dynasm!(ops ; .arch x64 ; mov eax, v);
            store(ops, rd);
            true
        }
        // --- Loads/stores (P2-3 S1).
        I::LsReg { rt, rn, rm, kind } => emit_lsreg(ops, rt, rn, rm, kind, pc, unalign_trap),
        I::LsImm { rt, rn, imm, kind } => emit_lsimm(ops, rt, rn, imm, kind, pc, unalign_trap),
        I::LsSp { rt, imm, is_load } => emit_lssp(ops, rt, imm, is_load, pc, unalign_trap),
        I::LdrLiteral { rt, imm } => emit_ldrliteral(ops, rt, imm, pc, unalign_trap),
        // --- Stack/multiple (P2-5 S3): two-phase guarded transfers
        // (probe all words effect-free, then transfer). `Push`/`Pop` go
        // through `sp`; `POP {pc}` is a terminator (see `emit16_term`).
        I::Push { regs, include_lr } => emit_push(ops, regs, include_lr, pc),
        I::Pop {
            regs,
            load_pc: false,
        } => emit_pop(ops, regs, pc),
        I::LdmStm { is_load, rn, regs } => emit_ldmstm16(ops, is_load, rn, regs, pc),
        // --- Extends/reversals (P2-5): pure ALU, never any flags.
        I::Extend { rd, rm, kind } => emit_extend(ops, rd, rm, kind),
        I::Reverse { rd, rm, kind } => emit_reverse(ops, rd, rm, kind),
        // --- System hints (P2-6, S6): inline field ops.
        I::Hint { kind } => emit_hint(ops, kind),
        // Terminators live in `emit_terminator` (slot-consuming edges).
        // Anything else templated (CPS/SVC/BKPT) is a flush + driver
        // re-execution tail (P2-6): the fallthrough owns it, so no per-op
        // arm can go stale. Terminators must NOT fall through (their
        // edges live in the terminator emitter).
        _ => {
            let op = TbOp::V16(*inst);
            if !super::templates::is_terminator(&op) && super::templates::is_templated(&op) {
                mem_side_exit(ops, pc);
                true
            } else {
                false
            }
        }
    }
}

/// 16-bit terminators (P2-2 static forms with P2-4 chained edges, P2-4
/// register-indirect forms). Slot order is taken-then-fall.
fn emit16_term(
    ops: &mut A,
    inst: &crate::cpu::decode16::Inst16,
    pc: u32,
    tape: &mut super::templates::SlotTape<'_>,
) -> bool {
    use super::native::{emit_static_edge, EXIT_BRANCH};
    use crate::cpu::decode16::Inst16 as I;
    match *inst {
        I::Branch { off } => {
            emit_static_edge(ops, EXIT_BRANCH, b_target(pc, off), tape.take(), 0);
            true
        }
        I::BranchCond { cond, off } => {
            if cond > 13 {
                return false;
            }
            emit_cond_pair(ops, cond, b_target(pc, off), pc.wrapping_add(2), tape);
            true
        }
        I::CbzCbnz { rn, off, nonzero } => {
            if rn == 15 {
                return false;
            }
            load(ops, rn, true);
            // Taken edge first (inverted jump): slot order matches
            // `exit_shape` (taken, fall).
            let fall = ops.new_dynamic_label();
            if nonzero {
                dynasm!(ops ; .arch x64 ; test eax, eax ; jz =>fall);
            } else {
                dynasm!(ops ; .arch x64 ; test eax, eax ; jnz =>fall);
            }
            emit_static_edge(
                ops,
                EXIT_BRANCH,
                b_target(pc, off),
                tape.take(),
                super::native::BRANCH_TAKEN_EXTRA,
            );
            dynasm!(ops ; .arch x64 ; =>fall);
            emit_static_edge(ops, EXIT_BRANCH, pc.wrapping_add(2), tape.take(), 0);
            true
        }
        I::Bx { rm } => emit_bx(ops, rm, pc, false, tape),
        I::Blx { rm } => emit_bx(ops, rm, pc, true, tape),
        I::Pop {
            regs,
            load_pc: true,
        } => emit_pop_pc(ops, regs, pc, tape),
        _ => false,
    }
}

/// 32-bit terminators (P2-4): `BL` links `LR` natively, wide branches
/// chain like their 16-bit cousins (32-bit forms are `pc + 4`-relative).
fn emit32_term(
    ops: &mut A,
    inst: &crate::cpu::decode32::Inst32,
    pc: u32,
    trap: bool,
    tape: &mut super::templates::SlotTape<'_>,
) -> bool {
    use super::native::{emit_static_edge, EXIT_BRANCH};
    use crate::cpu::decode32::Inst32 as I;
    match *inst {
        I::Bl { off } => {
            let lr = pc.wrapping_add(4) | 1;
            dynasm!(ops ; .arch x64 ; mov eax, lr as i32);
            store_lr(ops);
            emit_static_edge(ops, EXIT_BRANCH, b_target(pc, off), tape.take(), 0);
            true
        }
        I::BranchWide { off } => {
            emit_static_edge(ops, EXIT_BRANCH, b_target(pc, off), tape.take(), 0);
            true
        }
        I::BranchCondWide { cond, off } => {
            if cond > 13 {
                return false;
            }
            emit_cond_pair(ops, cond, b_target(pc, off), pc.wrapping_add(4), tape);
            true
        }
        I::LdmStm {
            is_load,
            rn,
            regs,
            writeback,
            dec_before,
        } => emit_ldmstm32(
            ops,
            is_load,
            rn,
            regs,
            writeback,
            dec_before,
            pc,
            Some(tape),
        ),
        I::TblBranch { rn, rm, half } => emit_tblbranch(ops, rn, rm, half, pc, trap, tape),
        _ => false,
    }
}

/// Store `eax` to the guest `LR` (memory path — `lr` is never pinned).
/// Clobbers `rsi`; preserves `edx` (the dynamic-edge target register).
fn store_lr(ops: &mut A) {
    let ctx = regs_off();
    let off = lr_off();
    dynasm!(ops
        ; .arch x64
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + ctx]
        ; mov [rsi + off], eax
    );
}

/// Register-indirect branch (P2-4): fast path for plain Thumb targets
/// (dynamic chained edge), side exits otherwise. `link` selects `BLX`
/// (`LR = (pc + 2) | 1` on the fast path only — slow paths leave `LR`
/// untouched so the interpreter's re-execution owns the write). Targets
/// at or above the FNC magic floor side-exit FIRST (`EXIT_MMIO` with the
/// op PC — the driver re-executes with no TLB fill, covering
/// secure/exception returns on every ISA/mode without baking any config
/// here); the interpreter dispatches magic before `check_branch`, so an
/// even magic value must not halt. Remaining even targets halt
/// (`EXIT_HALT`, mirroring `check_branch`).
fn emit_bx(
    ops: &mut A,
    rm: u8,
    pc: u32,
    link: bool,
    tape: &mut super::templates::SlotTape<'_>,
) -> bool {
    use super::native::{emit_dynamic_edge, emit_epilogue, EXIT_HALT};
    use crate::cpu::FNC_RETURN_MIN_MAGIC;
    if rm == 15 {
        return false;
    }
    load(ops, rm, true);
    let halt = ops.new_dynamic_label();
    let side = ops.new_dynamic_label();
    let magic = FNC_RETURN_MIN_MAGIC as i32;
    dynasm!(ops
        ; .arch x64
        ; mov edx, eax
        ; cmp edx, magic
        ; jae =>side
        ; test dl, 1
        ; jz =>halt
    );
    if link {
        let lr = pc.wrapping_add(2) | 1;
        dynasm!(ops ; .arch x64 ; mov eax, lr as i32);
        store_lr(ops);
    }
    dynasm!(ops ; .arch x64 ; and edx, 0xFFFFFFFEu32 as i32);
    emit_dynamic_edge(ops, tape.take());
    dynasm!(ops ; .arch x64 ; =>halt);
    emit_epilogue(ops, EXIT_HALT, pc);
    dynasm!(ops ; .arch x64 ; =>side);
    // `fault_addr` is unread on this path (`mem_access` is `None` for
    // branches): still zero the contract register for determinism.
    dynasm!(ops ; .arch x64 ; xor r10d, r10d);
    mem_side_exit(ops, pc);
    true
}

/// Context field offsets for S6 templates (bus-independent — `NativeCtx`
/// is `repr(C)` concrete; never `Core<B>` offsets, which vary by bus).
fn primask_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, primask) as i32
}

fn ipsr_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, ipsr) as i32
}

fn control_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, control) as i32
}

fn event_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, event) as i32
}

fn sleeping_off() -> i32 {
    core::mem::offset_of!(super::native::NativeCtx, sleeping) as i32
}

/// Hint execution (P2-6, S6): `NOP`/`YIELD` assemble to nothing; `SEV`
/// sets the event latch and wakes (`sleeping = false`); `WFI` sleeps;
/// `WFE` sleeps unless an event is latched (consuming it). All
/// field-local (no faults, no regs) — plain FALL-shape ops.
fn emit_hint(ops: &mut A, kind: crate::cpu::decode16::Hint) -> bool {
    use crate::cpu::decode16::Hint as H;
    match kind {
        H::Nop | H::Yield => true,
        H::Sev => {
            dynasm!(ops
                ; .arch x64
                ; mov rsi, [rsp]
                ; mov rsi, [rsi + event_off()]
                ; mov BYTE [rsi], 1
                ; mov rsi, [rsp]
                ; mov rsi, [rsi + sleeping_off()]
                ; mov BYTE [rsi], 0
            );
            true
        }
        H::Wfi => {
            dynasm!(ops
                ; .arch x64
                ; mov rsi, [rsp]
                ; mov rsi, [rsi + sleeping_off()]
                ; mov BYTE [rsi], 1
            );
            true
        }
        H::Wfe => {
            let sleep = ops.new_dynamic_label();
            let done = ops.new_dynamic_label();
            dynasm!(ops
                ; .arch x64
                ; mov rsi, [rsp]
                ; mov rsi, [rsi + event_off()]
                ; movzx eax, BYTE [rsi]
                ; test eax, eax
                ; jz =>sleep
                ; mov BYTE [rsi], 0
                ; jmp =>done
                ; =>sleep
            );
            dynasm!(ops
                ; .arch x64
                ; mov rsi, [rsp]
                ; mov rsi, [rsi + sleeping_off()]
                ; mov BYTE [rsi], 1
                ; =>done
            );
            true
        }
    }
}

/// `MRS` (P2-6, S6): field-only forms inline (`rd` gated); everything
/// else side-exits (banking already covered below needs no gate beyond
/// `rd`; the TrustZone/limit/FPSR/reserved forms re-execute).
fn emit_mrs(ops: &mut A, rd: u8, sys: crate::cpu::decode32::SysReg, pc: u32) -> bool {
    use crate::cpu::decode32::SysReg as S;
    use crate::cpu::registers::{APSR_QFLAGS, XPSR_T};
    if rd == 15 {
        return false;
    }
    match sys {
        S::Apsr | S::ApsrG | S::ApsrNzvCqg => {
            // Reads return the whole APSR regardless of qualifier.
            let mask = APSR_QFLAGS as i32;
            dynasm!(ops ; .arch x64 ; mov eax, r15d ; and eax, mask);
            store(ops, rd);
            true
        }
        S::Xpsr => {
            // ITSTATE bits read zero: native `MRS` never runs predicated
            // (non-predicable → `ItBreak` → interpreter), and `IT` seals
            // its own block — so the field is always 0 here, exactly like
            // an unpredicated stepper state.
            let mask = (APSR_QFLAGS | XPSR_T) as i32;
            let t = XPSR_T as i32;
            dynasm!(ops
                ; .arch x64
                ; mov eax, r15d
                ; and eax, mask
                ; or eax, t
            );
            store(ops, rd);
            true
        }
        S::Epsr => {
            let t = XPSR_T as i32;
            dynasm!(ops ; .arch x64 ; mov eax, t);
            store(ops, rd);
            true
        }
        S::Ipsr => {
            mrs_unpriv(ops, |ops| {
                dynasm!(ops
                    ; .arch x64
                    ; mov rsi, [rsp]
                    ; mov rsi, [rsi + ipsr_off()]
                    ; movzx eax, BYTE [rsi]
                );
            });
            store(ops, rd);
            true
        }
        S::Msp => {
            mrs_unpriv(ops, |ops| {
                mrs_sp_bank(ops, true);
            });
            store(ops, rd);
            true
        }
        S::Psp => {
            mrs_unpriv(ops, |ops| {
                mrs_sp_bank(ops, false);
            });
            store(ops, rd);
            true
        }
        S::Primask => {
            mrs_unpriv(ops, |ops| {
                dynasm!(ops
                    ; .arch x64
                    ; mov rsi, [rsp]
                    ; mov rsi, [rsi + primask_off()]
                    ; movzx eax, BYTE [rsi]
                );
            });
            store(ops, rd);
            true
        }
        S::Control => {
            dynasm!(ops
                ; .arch x64
                ; mov rsi, [rsp]
                ; mov rsi, [rsi + control_off()]
                ; mov eax, [rsi]
            );
            store(ops, rd);
            true
        }
        _ => {
            mem_side_exit(ops, pc);
            true
        }
    }
}

/// Unprivileged gate for `MRS` (P2-6): Thread mode (`ipsr == 0`) with
/// nPRIV set (`control & 1`) reads 0; otherwise `present` computes the
/// value into `eax`. Mirrors the interpreter's rule exactly.
fn mrs_unpriv(ops: &mut A, present: impl FnOnce(&mut A)) {
    let present_lbl = ops.new_dynamic_label();
    let done = ops.new_dynamic_label();
    dynasm!(ops
        ; .arch x64
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + ipsr_off()]
        ; movzx eax, BYTE [rsi]
        ; test eax, eax
        ; jnz =>present_lbl
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + control_off()]
        ; mov eax, [rsi]
        ; test eax, 1
        ; jz =>present_lbl
        ; mov eax, 0
        ; jmp =>done
        ; =>present_lbl
    );
    present(ops);
    dynasm!(ops ; .arch x64 ; =>done);
}

/// Active-stack read for `MRS MSP/PSP` (P2-6): `main = process ? msp :
/// sp`, `other = process ? sp : psp` (mirrors `sysreg_read`; all three
/// are concrete `Registers` fields — bus-independent offsets). Result in
/// `eax`. `process = (control & 2) != 0 && ipsr == 0` (matches
/// `sp_is_process`).
fn mrs_sp_bank(ops: &mut A, main: bool) {
    let use_sp = ops.new_dynamic_label();
    let done = ops.new_dynamic_label();
    let ctx = regs_off();
    let sp_off = core::mem::offset_of!(Registers, sp) as i32;
    let msp = core::mem::offset_of!(Registers, msp) as i32;
    let psp = core::mem::offset_of!(Registers, psp) as i32;
    dynasm!(ops
        ; .arch x64
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + control_off()]
        ; mov eax, [rsi]
        ; test eax, 2
        ; jz =>use_sp
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + ipsr_off()]
        ; movzx eax, BYTE [rsi]
        ; test eax, eax
        ; jnz =>use_sp
    );
    // Process mode.
    if main {
        dynasm!(ops
            ; .arch x64
            ; mov rsi, [rsp]
            ; mov rsi, [rsi + ctx]
            ; mov eax, [rsi + msp]
            ; jmp =>done
        );
    } else {
        dynasm!(ops
            ; .arch x64
            ; mov rsi, [rsp]
            ; mov rsi, [rsi + ctx]
            ; mov eax, [rsi + sp_off]
            ; jmp =>done
        );
    }
    // Handler mode and unprivileged-thread SPSEL: the active `sp` side.
    dynasm!(ops ; .arch x64 ; =>use_sp);
    if main {
        dynasm!(ops
            ; .arch x64
            ; mov rsi, [rsp]
            ; mov rsi, [rsi + ctx]
            ; mov eax, [rsi + sp_off]
        );
    } else {
        dynasm!(ops
            ; .arch x64
            ; mov rsi, [rsp]
            ; mov rsi, [rsi + ctx]
            ; mov eax, [rsi + psp]
        );
    }
    dynasm!(ops ; .arch x64 ; =>done);
}

/// `MSR` (P2-6, S6): flag merges plus the PRIMASK byte store inline;
/// everything else side-exits (ISA gates, banking, limits, TrustZone,
/// FPSCR — all owned by the interpreter).
fn emit_msr(ops: &mut A, rn: u8, sys: crate::cpu::decode32::SysReg, pc: u32) -> bool {
    use crate::cpu::decode32::SysReg as S;
    use crate::cpu::registers::{APSR_FLAGS, APSR_GE, APSR_Q, APSR_QFLAGS};
    if rn == 15 {
        return false;
    }
    load(ops, rn, true);
    match sys {
        S::Apsr => {
            let keep = !(APSR_FLAGS | APSR_Q) as i32;
            let take = (APSR_FLAGS | APSR_Q) as i32;
            dynasm!(ops
                ; .arch x64
                ; mov ecx, r15d
                ; and ecx, keep
                ; and eax, take
                ; or eax, ecx
                ; mov r15d, eax
            );
            true
        }
        S::ApsrG => {
            let keep = !APSR_GE as i32;
            let take = APSR_GE as i32;
            dynasm!(ops
                ; .arch x64
                ; mov ecx, r15d
                ; and ecx, keep
                ; and eax, take
                ; or eax, ecx
                ; mov r15d, eax
            );
            true
        }
        S::ApsrNzvCqg => {
            let keep = !APSR_QFLAGS as i32;
            let take = APSR_QFLAGS as i32;
            dynasm!(ops
                ; .arch x64
                ; mov ecx, r15d
                ; and ecx, keep
                ; and eax, take
                ; or eax, ecx
                ; mov r15d, eax
            );
            true
        }
        S::Primask => {
            dynasm!(ops
                ; .arch x64
                ; and eax, 1
                ; mov rsi, [rsp]
                ; mov rsi, [rsi + primask_off()]
                ; mov [rsi], al
            );
            true
        }
        _ => {
            mem_side_exit(ops, pc);
            true
        }
    }
}

/// Conditional pair: evaluate `cond` (0-13) and take the chained taken
/// edge or fall edge (slot order taken-then-fall, matching `exit_shape`).
fn emit_cond_pair(
    ops: &mut A,
    cond: u8,
    taken: u32,
    fall: u32,
    tape: &mut super::templates::SlotTape<'_>,
) {
    use super::native::{emit_static_edge, EXIT_BRANCH};
    cond_bit(ops, cond);
    let fall_lbl = ops.new_dynamic_label();
    dynasm!(ops ; .arch x64 ; jz =>fall_lbl);
    emit_static_edge(
        ops,
        EXIT_BRANCH,
        taken,
        tape.take(),
        super::native::BRANCH_TAKEN_EXTRA,
    );
    dynasm!(ops ; .arch x64 ; =>fall_lbl);
    emit_static_edge(ops, EXIT_BRANCH, fall, tape.take(), 0);
}

/// Condition select: evaluate `cond` (0-13) against the pinned NZCV
/// nibble, leaving the taken bit in `sil` with host Z set accordingly
/// (so callers branch on it directly). Nibble layout after `shr ecx, 28`:
/// N = 8, Z = 4, C = 2, V = 1. `edx` starts zeroed so every path yields a
/// clean 0/1 taken bit. Multi-condition paths leave their scratch in
/// `dh`, which must not vote (`sil` needs no REX conflict here — `ah` is
/// long dead by exit).
fn cond_bit(ops: &mut A, cond: u8) {
    dynasm!(ops
        ; .arch x64
        ; mov ecx, r15d
        ; shr ecx, 28
        ; xor edx, edx
    );
    match cond {
        0 => dynasm!(ops ; .arch x64 ; test ecx, 4 ; setnz dl), // EQ: Z
        1 => dynasm!(ops ; .arch x64 ; test ecx, 4 ; setz dl),  // NE: !Z
        2 => dynasm!(ops ; .arch x64 ; test ecx, 2 ; setnz dl), // CS: C
        3 => dynasm!(ops ; .arch x64 ; test ecx, 2 ; setz dl),  // CC: !C
        4 => dynasm!(ops ; .arch x64 ; test ecx, 8 ; setnz dl), // MI: N
        5 => dynasm!(ops ; .arch x64 ; test ecx, 8 ; setz dl),  // PL: !N
        6 => dynasm!(ops ; .arch x64 ; test ecx, 1 ; setnz dl), // VS: V
        7 => dynasm!(ops ; .arch x64 ; test ecx, 1 ; setz dl),  // VC: !V
        8 => dynasm!(ops // HI: C & !Z
            ; .arch x64
            ; test ecx, 2
            ; setnz dl
            ; test ecx, 4
            ; setz dh
            ; and dl, dh),
        9 => dynasm!(ops // LS: !C | Z
            ; .arch x64
            ; test ecx, 2
            ; setz dl
            ; test ecx, 4
            ; setnz dh
            ; or dl, dh),
        10 => dynasm!(ops // GE: N == V
            ; .arch x64
            ; mov edx, ecx
            ; shr edx, 3
            ; xor edx, ecx
            ; and edx, 1
            ; xor edx, 1),
        11 => dynasm!(ops // LT: N != V
            ; .arch x64
            ; mov edx, ecx
            ; shr edx, 3
            ; xor edx, ecx
            ; and edx, 1),
        12 => dynasm!(ops // GT: !Z & N == V (`dh` computed LAST: the
            // 32-bit `mov` below would wipe it)
            ; .arch x64
            ; mov edx, ecx
            ; shr edx, 3
            ; xor edx, ecx
            ; and edx, 1
            ; xor edx, 1
            ; test ecx, 4
            ; setz dh
            ; and dl, dh),
        _ => dynasm!(ops // LE: Z | N != V (same ordering constraint)
            ; .arch x64
            ; mov edx, ecx
            ; shr edx, 3
            ; xor edx, ecx
            ; and edx, 1
            ; test ecx, 4
            ; setnz dh
            ; or dl, dh),
    }
    dynasm!(ops
        ; .arch x64
        ; mov esi, edx
        ; test sil, sil
    );
}

#[derive(Clone, Copy)]
enum AluKind {
    Add,
    Sub,
}

/// `Rd = Rn +/- Rm` with full NZCV (`pred` skips the merge — S4).
fn alu_rr(ops: &mut A, rd: u8, rn: u8, rm: u8, kind: AluKind, pred: bool, merge: bool) -> bool {
    if rd == 15 || rn == 15 || rm == 15 {
        return false;
    }
    load(ops, rn, true);
    load(ops, rm, false);
    match kind {
        AluKind::Add => dynasm!(ops ; .arch x64 ; add eax, ecx),
        // x86 SUB leaves *borrow* in CF (inverted vs ARM C); `cmc`
        // complements CF only, so N/Z/V survive for the merge.
        AluKind::Sub => dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc),
    }
    store(ops, rd);
    // G3 flag traffic: skip the extraction + merge when a later full
    // producer covers the next observation (see `flag_merge_bits`).
    if !pred && merge {
        dynasm!(ops ; .arch x64 ; seto dl ; lahf);
        merge_nzcv(ops);
    }
    true
}

/// `Rd = Rn +/- imm` with full NZCV (`pred` skips the merge — S4).
fn alu_ri(ops: &mut A, rd: u8, rn: u8, imm: i32, kind: AluKind, pred: bool, merge: bool) -> bool {
    if rd == 15 || rn == 15 {
        return false;
    }
    load(ops, rn, true);
    dynasm!(ops ; .arch x64 ; mov ecx, imm);
    match kind {
        AluKind::Add => dynasm!(ops ; .arch x64 ; add eax, ecx),
        AluKind::Sub => dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc),
    }
    store(ops, rd);
    // G3 flag traffic: skip the extraction + merge when a later full
    // producer covers the next observation (see `flag_merge_bits`).
    if !pred && merge {
        dynasm!(ops ; .arch x64 ; seto dl ; lahf);
        merge_nzcv(ops);
    }
    true
}

/// `CMP Rn, #imm`: compare and discard, full NZCV. Pure flags: with
/// `merge` false the whole op vanishes (no other effects).
fn cmp_ri(ops: &mut A, rn: u8, imm: i32, merge: bool) -> bool {
    if rn == 15 {
        return false;
    }
    if !merge {
        return true;
    }
    load(ops, rn, true);
    dynasm!(ops ; .arch x64 ; mov ecx, imm);
    dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc ; seto dl ; lahf);
    merge_nzcv(ops);
    true
}

/// Memory guard + fast path (P2-3). Input `ECX` = guest address.
/// Falls through with `RAX` = host address; misses jump to `side`.
/// Clobbers `EAX`/`EDX`/`ESI`/`EDI`; preserves `R8D`–`R11D` and pins.
/// `trap` bakes CCR.UNALIGN_TRP (x64 needs no alignment check otherwise:
/// the host tolerates in-page unaligned accesses natively).
/// A primary miss probes the victim slot (P3-6) before giving up: the
/// fast path pays one extra `jmp` over the probe; colliding hot pages
/// settle instead of side-exiting every access.
fn mem_guard(ops: &mut A, size: u32, is_write: bool, trap: bool, side: dynasmrt::DynamicLabel) {
    if size > 1 {
        // Page-cross check (host pages need not be adjacent).
        let limit = (0x1000 - size) as i32;
        dynasm!(ops
            ; .arch x64
            ; mov edx, ecx
            ; and edx, 0xFFF
            ; cmp edx, limit
            ; ja =>side
        );
        if trap {
            let mask = (size - 1) as i32;
            dynasm!(ops
                ; .arch x64
                ; test ecx, mask
                ; jnz =>side
            );
        }
    }
    let tlb = tlb_off();
    let priv_off = priv_off();
    let perm = if is_write { 2i8 } else { 1i8 };
    let victim = ops.new_dynamic_label();
    let done = ops.new_dynamic_label();
    let vtag = super::tlb::TLB_VICTIM_OFF as i32;
    let vperm = vtag + 4;
    let vptr = vtag + 8;
    dynasm!(ops
        ; .arch x64
        ; mov r10d, ecx
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + tlb]
        ; mov eax, r10d
        ; shr eax, 12
        ; mov edx, eax
        ; and edx, 63
        ; shl edx, 4
        ; shl eax, 1
        ; mov rdi, [rsp]
        ; or eax, [rdi + priv_off]
        ; cmp DWORD [rsi + rdx], eax
        ; jne =>victim
        ; test BYTE [rsi + rdx + 4], perm
        ; jz =>victim
        ; mov rdx, [rsi + rdx + 8]
        ; mov eax, r10d
        ; and eax, 0xFFF
        ; add rax, rdx
        ; jmp =>done
        ; =>victim
        ; cmp DWORD [rsi + vtag], eax
        ; jne =>side
        ; test BYTE [rsi + vperm], perm
        ; jz =>side
        ; mov rdx, [rsi + vptr]
        ; mov eax, r10d
        ; and eax, 0xFFF
        ; add rax, rdx
        ; =>done
    );
}

/// Side-exit handler for a memory miss: flush pins (the driver's interp
/// re-execution observes `Core`), record the faulting guest address
/// (saved in `R10D` by the guard contract) and exit with `(MMIO, op_pc)`
/// for driver handling (interp re-execution + TLB fill attempt).
fn mem_side_exit(ops: &mut A, op_pc: u32) {
    use super::native::{emit_epilogue, emit_pin_flush, EXIT_MMIO};
    let fault = fault_off();
    emit_pin_flush(ops);
    dynasm!(ops
        ; .arch x64
        ; mov rdi, [rsp]
        ; mov [rdi + fault], r10d
    );
    emit_epilogue(ops, EXIT_MMIO, op_pc);
}

/// One guarded memory site: address already in `ECX`, then `fast`
/// performs the access (host in `RAX` after the guard), optional
/// writeback, and the epoch bump for stores. Skips the miss handler on
/// hits. Value/base spills live in `R8D`/`R9D` (guard-safe).
fn mem_site(
    ops: &mut A,
    size: u32,
    is_write: bool,
    trap: bool,
    op_pc: u32,
    fast: impl FnOnce(&mut A),
) {
    let side = ops.new_dynamic_label();
    let end = ops.new_dynamic_label();
    mem_guard(ops, size, is_write, trap, side);
    // Stores decide the generation bump here (per code page); loads
    // never bump (a load invalidating links would be pure pessimism).
    if is_write {
        bump_if_code(ops);
    }
    fast(ops);
    dynasm!(ops ; .arch x64 ; jmp =>end);
    dynasm!(ops ; .arch x64 ; =>side);
    mem_side_exit(ops, op_pc);
    dynasm!(ops ; .arch x64 ; =>end);
}

/// Store `r8d` to the guest `sp` field (memory path — `sp` is never
/// pinned). Clobbers `rsi`.
fn store_sp_r8(ops: &mut A) {
    let ctx = regs_off();
    let off = core::mem::offset_of!(Registers, sp) as i32;
    dynasm!(ops
        ; .arch x64
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + ctx]
        ; mov [rsi + off], r8d
    );
}

/// One multi-word transfer slot (P2-5, S3): word index from the cursor
/// base in `R8D` plus the phase-2 action. `Store` preloads `R[reg]` to
/// `R9D` per word; `Load` writes the fetched word to `R[reg]`; `LoadTarget`
/// parks the fetched word in `R9D` for the pop-branch protocol.
#[derive(Clone, Copy)]
enum WordSlot {
    Store(u8),
    Load(u8),
    LoadTarget,
}

/// Probe one word for a multi-word transfer (phase 1): TLB check only, no
/// transfer — misses jump to `side` while no guest byte has moved, so the
/// driver's interp re-execution never double-effects. Address in `ECX`.
fn probe_word(ops: &mut A, is_write: bool, side: dynasmrt::DynamicLabel) {
    mem_guard(ops, 4, is_write, false, side);
}

/// Shared tail for multi-word emitters: `R8D` holds the post-transfer
/// base cursor. Generation bumps happen per word at their transfer
/// guards (split generations — code pages only), so the tail just
/// binds `side` to the effect-free side exit.
fn multi_tail(ops: &mut A, op_pc: u32, side: dynasmrt::DynamicLabel) {
    let done = ops.new_dynamic_label();
    dynasm!(ops ; .arch x64 ; jmp =>done);
    dynasm!(ops ; .arch x64 ; =>side);
    mem_side_exit(ops, op_pc);
    dynasm!(ops ; .arch x64 ; =>done);
}

/// Advance the write generation after a fast store, but only through
/// code pages (split generations): the guest address is still in `R10D`
/// per the guard contract, so re-derive its TLB index and test the code
/// bit. Data-only stores skip the bump — links and validated entries
/// survive them (store loops chain) — while code patches still fence
/// (the insert path marks live entries; fills stamp from the page
/// index). Clobbers `EDX`/`RSI`/`RDI` (all guard-clobberable);
/// preserves `RAX` (the host address stores use next), pins,
/// `R8D`–`R11D`, and `R10D`.
fn bump_if_code(ops: &mut A) {
    let tlb = tlb_off();
    let epoch = epoch_off();
    let code = super::tlb::TLB_CODE as i8;
    let bump = ops.new_dynamic_label();
    dynasm!(ops
        ; .arch x64
        ; mov edx, r10d
        ; shr edx, 12
        ; and edx, 63
        ; shl edx, 4
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + tlb]
        ; test BYTE [rsi + rdx + 4], code
        ; jz =>bump
        ; mov rdi, [rsp]
        ; mov rdi, [rdi + epoch]
        ; inc QWORD [rdi]
        ; =>bump
    );
}

/// `PUSH {regs, lr?}` (P2-5, S3): `new_sp = sp - 4n`, ascending stores
/// from `new_sp`, then `sp = new_sp` — mirroring the interpreter exactly
/// (ascending MPU pre-checks become ascending phase-1 probes, so the
/// first violator faults identically). The ARMv8-M stack-limit check is
/// runner-gated (v8-M keeps PUSH interpreted); unaligned `new_sp` also
/// side-exits (the bus owns unaligned word semantics).
fn emit_push(ops: &mut A, regs: u8, include_lr: bool, pc: u32) -> bool {
    let mut slots = Vec::new();
    for r in 0..8u8 {
        if regs & (1 << r) != 0 {
            slots.push((slots.len() as i32 * 4, WordSlot::Store(r)));
        }
    }
    if include_lr {
        slots.push((slots.len() as i32 * 4, WordSlot::Store(14)));
    }
    if slots.is_empty() {
        return true; // No words, `sp` untouched — a pure no-op like interp.
    }
    let side = ops.new_dynamic_label();
    let n = slots.len() as i32;
    // Cursor: `R8D = sp - 4n`.
    load(ops, 13, false);
    dynasm!(ops ; .arch x64 ; mov r8d, ecx ; sub r8d, 4 * n);
    dynasm!(ops ; .arch x64 ; test r8d, 3 ; jnz =>side);
    // Phase 1: probe every word effect-free, ascending.
    for (off, _) in &slots {
        let off = *off;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        probe_word(ops, true, side);
    }
    // Phase 2: transfer (proven hits — cannot side-exit).
    for (off, slot) in &slots {
        let WordSlot::Store(reg) = *slot else {
            unreachable!("push slots are stores")
        };
        let off = *off;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        load(ops, reg, true);
        dynasm!(ops ; .arch x64 ; mov r9d, eax);
        mem_guard(ops, 4, true, false, side);
        bump_if_code(ops);
        dynasm!(ops ; .arch x64 ; mov [rax], r9d);
    }
    store_sp_r8(ops);
    multi_tail(ops, pc, side);
    true
}

/// `POP {regs}` without `pc` (P2-5, S3): ascending loads from `sp`, then
/// `sp += 4n`. The `pc` form is a terminator (see `emit_pop_pc`).
fn emit_pop(ops: &mut A, regs: u8, pc: u32) -> bool {
    let n = regs.count_ones() as i32;
    if n == 0 {
        return true; // No words, `sp` untouched — like interp.
    }
    let side = ops.new_dynamic_label();
    // Cursor: `R8D = sp`.
    load(ops, 13, false);
    dynasm!(ops ; .arch x64 ; mov r8d, ecx ; test r8d, 3 ; jnz =>side);
    let mut idx = 0i32;
    for r in 0..8u8 {
        if regs & (1 << r) == 0 {
            continue;
        }
        let off = idx * 4;
        idx += 1;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        probe_word(ops, false, side);
    }
    idx = 0;
    for r in 0..8u8 {
        if regs & (1 << r) == 0 {
            continue;
        }
        let off = idx * 4;
        idx += 1;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        mem_guard(ops, 4, false, false, side);
        dynasm!(ops ; .arch x64 ; mov eax, [rax]);
        store(ops, r);
    }
    dynasm!(ops ; .arch x64 ; lea r8d, [r8 + 4 * n]);
    store_sp_r8(ops);
    multi_tail(ops, pc, side);
    true
}

/// `POP {regs, pc}` (P2-5, S3 terminator): register words transfer like
/// `emit_pop`, the `pc` word parks in `R9D`, `sp` updates, then the loaded
/// word classifies — plain Thumb targets take the dynamic chained edge,
/// anything else exits `POP_BRANCH` (the driver runs only the branch
/// dispatch: no transfer re-execution, no double reads).
fn emit_pop_pc(ops: &mut A, regs: u8, pc: u32, tape: &mut super::templates::SlotTape<'_>) -> bool {
    use super::native::{emit_dynamic_edge, emit_epilogue, EXIT_POP_BRANCH};
    use crate::cpu::FNC_RETURN_MIN_MAGIC;
    let n = regs.count_ones() as i32;
    let side = ops.new_dynamic_label();
    let pb = ops.new_dynamic_label();
    load(ops, 13, false);
    dynasm!(ops ; .arch x64 ; mov r8d, ecx ; test r8d, 3 ; jnz =>side);
    let mut idx = 0i32;
    for r in 0..8u8 {
        if regs & (1 << r) == 0 {
            continue;
        }
        let off = idx * 4;
        idx += 1;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        probe_word(ops, false, side);
    }
    // The `pc` word probes last (ascending, like interp reads it last).
    let pc_off = idx * 4;
    dynasm!(ops ; .arch x64 ; lea ecx, [r8 + pc_off]);
    probe_word(ops, false, side);
    idx = 0;
    for r in 0..8u8 {
        if regs & (1 << r) == 0 {
            continue;
        }
        let off = idx * 4;
        idx += 1;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        mem_guard(ops, 4, false, false, side);
        dynasm!(ops ; .arch x64 ; mov eax, [rax]);
        store(ops, r);
    }
    dynasm!(ops ; .arch x64 ; lea ecx, [r8 + pc_off]);
    mem_guard(ops, 4, false, false, side);
    dynasm!(ops ; .arch x64 ; mov r9d, [rax]);
    // `sp` updates before the branch (interp order).
    dynasm!(ops ; .arch x64 ; lea r8d, [r8 + 4 * (n + 1)]);
    store_sp_r8(ops);
    // Classify: fast (plain Thumb) or pop-branch (even + magic alike —
    // the driver dispatches both identically to the interpreter).
    let magic = FNC_RETURN_MIN_MAGIC as i32;
    dynasm!(ops
        ; .arch x64
        ; mov edx, r9d
        ; cmp edx, magic
        ; jae =>pb
        ; test dl, 1
        ; jz =>pb
        ; and edx, 0xFFFFFFFEu32 as i32
    );
    emit_dynamic_edge(ops, tape.take());
    dynasm!(ops ; .arch x64 ; =>pb);
    // `fault_addr` carries the loaded target (no fault involved).
    let fault = fault_off();
    dynasm!(ops
        ; .arch x64
        ; mov rdi, [rsp]
        ; mov [rdi + fault], r9d
    );
    emit_epilogue(ops, EXIT_POP_BRANCH, pc);
    // The shared side block still needs binding (probes jump here).
    let done = ops.new_dynamic_label();
    dynasm!(ops ; .arch x64 ; jmp =>done);
    dynasm!(ops ; .arch x64 ; =>side);
    mem_side_exit(ops, pc);
    dynasm!(ops ; .arch x64 ; =>done);
    true
}

/// 16-bit `LDMIA`/`STMIA` (P2-5, S3): ascending words from the `rn` base
/// (`rn` is 3-bit — never `sp`/`lr`/`pc`), then mandatory writeback
/// (`rn += 4n`, always — even empty; a loaded `rn` wins over it, decided
/// statically from the mask, exactly like the interpreter).
fn emit_ldmstm16(ops: &mut A, is_load: bool, rn: u8, regs: u8, pc: u32) -> bool {
    let side = ops.new_dynamic_label();
    // Cursor: `R8D = Rn`.
    load(ops, rn, false);
    // Thumb-1 multiples fault on unaligned bases (not trap-gated).
    dynasm!(ops ; .arch x64 ; mov r8d, ecx ; test r8d, 3 ; jnz =>side);
    let mut idx = 0i32;
    for r in 0..8u8 {
        if regs & (1 << r) == 0 {
            continue;
        }
        let off = idx * 4;
        idx += 1;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        probe_word(ops, !is_load, side);
    }
    idx = 0;
    for r in 0..8u8 {
        if regs & (1 << r) == 0 {
            continue;
        }
        let off = idx * 4;
        idx += 1;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        if !is_load {
            // Value first: the guard returns the host address in `rax`,
            // and loading after it would clobber exactly that (caught by
            // differential — segfault on the value-as-pointer).
            load(ops, r, true);
            dynasm!(ops ; .arch x64 ; mov r9d, eax);
        }
        mem_guard(ops, 4, !is_load, false, side);
        if is_load {
            dynasm!(ops ; .arch x64 ; mov eax, [rax]);
            store(ops, r);
        } else {
            bump_if_code(ops);
            dynasm!(ops ; .arch x64 ; mov [rax], r9d);
        }
    }
    // Writeback (loaded `rn` wins — static).
    let n = regs.count_ones() as i32;
    if !is_load || regs & (1 << rn) == 0 {
        let bytes = 4 * n;
        dynasm!(ops ; .arch x64 ; lea eax, [r8 + bytes]);
        store(ops, rn);
    }
    multi_tail(ops, pc, side);
    true
}

/// 16-bit single-transfer emitters (S1). Conventions: guest address ends
/// in `ECX`; store values are preloaded to `R9D`; writeback effects wait
/// in `R8D`; the guard preserves all three. `pc` feeds literal bases and
/// side-exit PCs; `trap` bakes CCR.UNALIGN_TRP.
fn emit_lsreg(
    ops: &mut A,
    rt: u8,
    rn: u8,
    rm: u8,
    kind: crate::cpu::decode16::LsReg,
    pc: u32,
    trap: bool,
) -> bool {
    use crate::cpu::decode16::LsReg as K;
    if rt == 15 || rn == 15 || rm == 15 {
        return false;
    }
    let (size, is_write, sext) = match kind {
        K::Str => (4, true, false),
        K::Ldr => (4, false, false),
        K::Strh => (2, true, false),
        K::Ldrh => (2, false, false),
        K::Strb => (1, true, false),
        K::Ldrb => (1, false, false),
        K::Ldrsb => (1, false, true),
        K::Ldrsh => (2, false, true),
    };
    load(ops, rn, false);
    load(ops, rm, true);
    dynasm!(ops ; .arch x64 ; add ecx, eax);
    if is_write {
        load(ops, rt, true);
        dynasm!(ops ; .arch x64 ; mov r9d, eax);
    }
    mem_site(ops, size, is_write, trap, pc, |ops| {
        emit_access(ops, size, is_write, sext, rt, None);
    });
    true
}

fn emit_lsimm(
    ops: &mut A,
    rt: u8,
    rn: u8,
    imm: usize,
    kind: crate::cpu::decode16::LsImm,
    pc: u32,
    trap: bool,
) -> bool {
    use crate::cpu::decode16::LsImm as K;
    if rt == 15 || rn == 15 {
        return false;
    }
    let (size, is_write, sext) = match kind {
        K::Str => (4, true, false),
        K::Ldr => (4, false, false),
        K::Strh => (2, true, false),
        K::Ldrh => (2, false, false),
        K::Strb => (1, true, false),
        K::Ldrb => (1, false, false),
    };
    let imm = imm as i32;
    load(ops, rn, false);
    dynasm!(ops ; .arch x64 ; add ecx, imm);
    if is_write {
        load(ops, rt, true);
        dynasm!(ops ; .arch x64 ; mov r9d, eax);
    }
    mem_site(ops, size, is_write, trap, pc, |ops| {
        emit_access(ops, size, is_write, sext, rt, None);
    });
    true
}

fn emit_lssp(ops: &mut A, rt: u8, imm: usize, is_load: bool, pc: u32, trap: bool) -> bool {
    if rt == 15 {
        return false;
    }
    let imm = imm as i32;
    // SP is never pinned: always the memory path (fresh every access).
    load(ops, 13, false);
    dynasm!(ops ; .arch x64 ; add ecx, imm);
    if !is_load {
        load(ops, rt, true);
        dynasm!(ops ; .arch x64 ; mov r9d, eax);
    }
    mem_site(ops, 4, !is_load, trap, pc, |ops| {
        emit_access(ops, 4, !is_load, false, rt, None);
    });
    true
}

fn emit_ldrliteral(ops: &mut A, rt: u8, imm: usize, pc: u32, trap: bool) -> bool {
    if rt == 15 {
        return false;
    }
    // Static base: word-aligned `pc + 4` plus the literal offset.
    let base = ((pc.wrapping_add(4)) & !3).wrapping_add(imm as u32) as i32;
    dynasm!(ops ; .arch x64 ; mov ecx, base);
    mem_site(ops, 4, false, trap, pc, |ops| {
        emit_access(ops, 4, false, false, rt, None);
    });
    true
}

/// 32-bit single transfer (S1): full addressing modes (register offset
/// with shift, ±immediate, offset/pre/post-indexed, writeback). Matches
/// the interpreter's access-then-writeback order exactly (including
/// `rt == rn` aliasing: the loaded value lands first, the base update
/// overwrites it). `unprivileged` is ignored to match the interpreter
/// (documented fidelity gap, not a template decision). Coverage gated
/// upstream by `is_templated` (no `r15` dynamics, no dual forms).
fn emit_mem32(
    ops: &mut A,
    access: crate::cpu::decode32::MemOp,
    rt: u8,
    rn: u8,
    off: u32,
    rm: Option<u8>,
    shift: u8,
    add: bool,
    index: bool,
    writeback: bool,
    pc: u32,
    trap: bool,
) -> bool {
    use crate::cpu::decode32::MemOp as M;
    let (size, is_write, sext) = match access {
        M::Str => (4, true, false),
        M::Ldr => (4, false, false),
        M::Strh => (2, true, false),
        M::Ldrh => (2, false, false),
        M::Strb => (1, true, false),
        M::Ldrb => (1, false, false),
        M::Ldrsb => (1, false, true),
        M::Ldrsh => (2, false, true),
    };
    if rt == 15 {
        return false;
    }
    if rn == 15 {
        // Literal base (loads only, never writeback — gated upstream):
        // static address, no base register traffic.
        let base = ((pc.wrapping_add(4)) & !3) as i32;
        dynasm!(ops ; .arch x64 ; mov ecx, base);
    } else {
        if rm == Some(15) {
            return false;
        }
        load(ops, rn, false);
    }
    // Offset into the address; writeback effects wait in R8D.
    let mut wb = None;
    if let Some(r) = rm {
        load(ops, r, true);
        let shift = shift as i8;
        dynasm!(ops ; .arch x64 ; shl eax, shift);
        if index {
            dynasm!(ops ; .arch x64 ; add ecx, eax);
            if writeback {
                dynasm!(ops ; .arch x64 ; mov r8d, ecx);
                wb = Some(rn);
            }
        } else {
            // Post-indexed: address stays base; effect steps it.
            dynasm!(ops ; .arch x64 ; mov r8d, ecx ; add r8d, eax);
            wb = Some(rn);
        }
    } else {
        let off = off as i32;
        if index {
            if add {
                dynasm!(ops ; .arch x64 ; add ecx, off);
            } else {
                dynasm!(ops ; .arch x64 ; sub ecx, off);
            }
            if writeback {
                dynasm!(ops ; .arch x64 ; mov r8d, ecx);
                wb = Some(rn);
            }
        } else {
            // Post-indexed immediate.
            dynasm!(ops ; .arch x64 ; mov r8d, ecx);
            if add {
                dynasm!(ops ; .arch x64 ; add r8d, off);
            } else {
                dynasm!(ops ; .arch x64 ; sub r8d, off);
            }
            wb = Some(rn);
        }
    }
    if is_write {
        load(ops, rt, true);
        dynasm!(ops ; .arch x64 ; mov r9d, eax);
    }
    mem_site(ops, size, is_write, trap, pc, |ops| {
        emit_access(ops, size, is_write, sext, rt, wb);
    });
    true
}

/// Shared access writer: fast load/store at host `RAX`, optional
/// writeback of `R8D` to `rn`. Stores read `R9D`; their generation
/// bump already happened at the site guard (`mem_site`).
/// `rt` names the transfer register (loads).
fn emit_access(ops: &mut A, size: u32, is_write: bool, sext: bool, rt: u8, wb_rn: Option<u8>) {
    if is_write {
        match size {
            1 => dynasm!(ops ; .arch x64 ; mov [rax], r9b),
            2 => dynasm!(ops ; .arch x64 ; mov [rax], r9w),
            _ => dynasm!(ops ; .arch x64 ; mov [rax], r9d),
        }
    } else {
        match (size, sext) {
            (1, false) => dynasm!(ops ; .arch x64 ; movzx eax, BYTE [rax]),
            (1, true) => dynasm!(ops ; .arch x64 ; movsx eax, BYTE [rax]),
            (2, false) => dynasm!(ops ; .arch x64 ; movzx eax, WORD [rax]),
            (2, true) => dynasm!(ops ; .arch x64 ; movsx eax, WORD [rax]),
            _ => dynasm!(ops ; .arch x64 ; mov eax, [rax]),
        }
        store(ops, rt);
    }
    // Writeback after the access (matches `exec_ls_at` + post-access
    // write: on success the base update lands last, so rt==rn aliasing
    // resolves identically).
    if let Some(rn) = wb_rn {
        dynasm!(ops ; .arch x64 ; mov eax, r8d);
        store(ops, rn);
    }
}

/// 32-bit `LDM`/`STM` (P2-5, S3): `rn`-relative IA/DB with optional
/// writeback, ascending transfers. Register-15 loads use the pop-branch
/// protocol (needs `tape`; `None` fails closed so plain emission never
/// takes them). Loaded-`rn`-wins and the `writeback` flag are static.
/// `dec_before` folds into the cursor (`R8D` starts at the transfer base;
/// writeback re-derives from it), so IA and DB share the loops.
fn emit_ldmstm32(
    ops: &mut A,
    is_load: bool,
    rn: u8,
    regs: u16,
    writeback: bool,
    dec_before: bool,
    pc: u32,
    tape: Option<&mut super::templates::SlotTape<'_>>,
) -> bool {
    use super::native::{emit_dynamic_edge, emit_epilogue, EXIT_POP_BRANCH};
    use crate::cpu::FNC_RETURN_MIN_MAGIC;
    if rn == 15 {
        return false;
    }
    let has_pc = is_load && regs & 0x8000 != 0;
    let mut tape = match (has_pc, tape) {
        (false, _) => None,
        (true, Some(t)) => Some(t),
        (true, None) => return false,
    };
    let n = regs.count_ones() as i32;
    let side = ops.new_dynamic_label();
    // Cursor: `R8D = base (- 4n for DB)`.
    load(ops, rn, false);
    dynasm!(ops ; .arch x64 ; mov r8d, ecx);
    if dec_before {
        let bytes = 4 * n;
        dynasm!(ops ; .arch x64 ; sub r8d, bytes);
    }
    // Word-aligned transfer base (least-significant bits ignored nowhere
    // — a nonzero value faults, trap-independent).
    dynasm!(ops ; .arch x64 ; test r8d, 3 ; jnz =>side);
    let mut idx = 0i32;
    for r in 0..16u8 {
        if regs & (1u16 << r) == 0 {
            continue;
        }
        let off = idx * 4;
        idx += 1;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        probe_word(ops, !is_load, side);
    }
    idx = 0;
    for r in 0..16u8 {
        if regs & (1u16 << r) == 0 {
            continue;
        }
        let off = idx * 4;
        idx += 1;
        dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        if r != 15 && !is_load {
            // Value before the guard (see `emit_ldmstm16`: the guard owns
            // `rax` after it runs).
            load(ops, r, true);
            dynasm!(ops ; .arch x64 ; mov r9d, eax);
        }
        mem_guard(ops, 4, !is_load, false, side);
        if r == 15 {
            dynasm!(ops ; .arch x64 ; mov r9d, [rax]);
        } else if is_load {
            dynasm!(ops ; .arch x64 ; mov eax, [rax]);
            store(ops, r);
        } else {
            bump_if_code(ops);
            dynasm!(ops ; .arch x64 ; mov [rax], r9d);
        }
    }
    // Writeback re-derives from the cursor (`R8D` is the transfer base
    // for DB, `base` for IA — symmetric by construction).
    if writeback && (!is_load || regs & (1u16 << rn) == 0) {
        if !dec_before {
            let bytes = 4 * n;
            dynasm!(ops ; .arch x64 ; add r8d, bytes);
        }
        dynasm!(ops ; .arch x64 ; mov eax, r8d);
        store(ops, rn);
    }
    if !has_pc {
        multi_tail(ops, pc, side);
        return true;
    }
    // Pop-branch protocol (loads only — no generation bump): classify the
    // parked `pc` word, chain fast targets, exit the rest.
    let tape = tape.take().expect("pc slot checked above");
    let pb = ops.new_dynamic_label();
    let magic = FNC_RETURN_MIN_MAGIC as i32;
    dynasm!(ops
        ; .arch x64
        ; mov edx, r9d
        ; cmp edx, magic
        ; jae =>pb
        ; test dl, 1
        ; jz =>pb
        ; and edx, 0xFFFFFFFEu32 as i32
    );
    emit_dynamic_edge(ops, tape.take());
    dynasm!(ops ; .arch x64 ; =>pb);
    let fault = fault_off();
    dynasm!(ops
        ; .arch x64
        ; mov rdi, [rsp]
        ; mov [rdi + fault], r9d
    );
    emit_epilogue(ops, EXIT_POP_BRANCH, pc);
    let done = ops.new_dynamic_label();
    dynasm!(ops ; .arch x64 ; jmp =>done);
    dynasm!(ops ; .arch x64 ; =>side);
    mem_side_exit(ops, pc);
    dynasm!(ops ; .arch x64 ; =>done);
    true
}

/// `LDRD`/`STRD` (P2-5, S3): two ascending words at the computed address
/// (literal/static, indexed, or post-indexed + writeback), then the
/// writeback. Cursor in `R8D`, the writeback effect parked in `R11D`
/// (both guard-safe); unaligned addresses fault trap-independently.
fn emit_ldrdstrd(
    ops: &mut A,
    is_load: bool,
    rt: u8,
    rt2: u8,
    rn: u8,
    imm: u32,
    add: bool,
    index: bool,
    writeback: bool,
    pc: u32,
) -> bool {
    if rt == 15 || rt2 == 15 {
        return false;
    }
    if rn == 15 && (!is_load || writeback) {
        return false;
    }
    let side = ops.new_dynamic_label();
    // Address + effect: `R8D` = transfer address, `R11D` = writeback value.
    if rn == 15 {
        let base = ((pc.wrapping_add(4)) & !3) as i32;
        dynasm!(ops ; .arch x64 ; mov ecx, base);
    } else {
        load(ops, rn, false);
    }
    let imm = imm as i32;
    if add {
        dynasm!(ops ; .arch x64 ; mov r11d, ecx ; add r11d, imm);
    } else {
        dynasm!(ops ; .arch x64 ; mov r11d, ecx ; sub r11d, imm);
    }
    if index {
        dynasm!(ops ; .arch x64 ; mov r8d, r11d);
    } else {
        dynasm!(ops ; .arch x64 ; mov r8d, ecx);
    }
    dynasm!(ops ; .arch x64 ; mov ecx, r8d ; test ecx, 3 ; jnz =>side);
    // Phase 1: probe both words (low, high).
    dynasm!(ops ; .arch x64 ; mov ecx, r8d);
    probe_word(ops, !is_load, side);
    dynasm!(ops ; .arch x64 ; lea ecx, [r8 + 4]);
    probe_word(ops, !is_load, side);
    // Phase 2: transfer low then high (interp order).
    for (rt, off) in [(rt, 0i32), (rt2, 4i32)] {
        if off == 0 {
            dynasm!(ops ; .arch x64 ; mov ecx, r8d);
        } else {
            dynasm!(ops ; .arch x64 ; lea ecx, [r8 + off]);
        }
        if !is_load {
            // Value before the guard (guard owns `rax` after it runs).
            load(ops, rt, true);
            dynasm!(ops ; .arch x64 ; mov r9d, eax);
        }
        mem_guard(ops, 4, !is_load, false, side);
        if is_load {
            dynasm!(ops ; .arch x64 ; mov eax, [rax]);
            store(ops, rt);
        } else {
            bump_if_code(ops);
            dynasm!(ops ; .arch x64 ; mov [rax], r9d);
        }
    }
    if writeback {
        dynasm!(ops ; .arch x64 ; mov eax, r11d);
        store(ops, rn);
    }
    multi_tail(ops, pc, side);
    true
}

/// `TBB`/`TBH` (P2-5 terminator): guarded table read (trap-baked like
/// P2-3 — `read_half` shares the bus tolerance), then `target =
/// (pc + 4) + 2 * index` (always even, so `& !1` is a no-op — mirrored
/// exactly) into the dynamic chained edge. `itstate` is already 0 in
/// native blocks. The target survives `mem_site` in `edx` on the hit path
/// (misses exit inside the site).
fn emit_tblbranch(
    ops: &mut A,
    rn: u8,
    rm: u8,
    half: bool,
    pc: u32,
    trap: bool,
    tape: &mut super::templates::SlotTape<'_>,
) -> bool {
    use super::native::emit_dynamic_edge;
    if rm == 15 {
        return false;
    }
    // `table_addr = base + Rm (<< 1 for half)` into `ECX` first.
    if rn == 15 {
        // Static `pc + 4` base (deliberately NOT word-aligned — mirrors
        // the interpreter, unlike literal loads).
        let base = pc.wrapping_add(4) as i32;
        dynasm!(ops ; .arch x64 ; mov ecx, base);
    } else {
        load(ops, rn, false);
    }
    load(ops, rm, true);
    if half {
        dynasm!(ops ; .arch x64 ; shl eax, 1);
    }
    dynasm!(ops ; .arch x64 ; add ecx, eax);
    let size = if half { 2 } else { 1 };
    mem_site(ops, size, false, trap, pc, |ops| {
        // Host table byte/halfword in `RAX` (zero-extended `index`).
        if half {
            dynasm!(ops ; .arch x64 ; movzx eax, WORD [rax]);
        } else {
            dynasm!(ops ; .arch x64 ; movzx eax, BYTE [rax]);
        }
        // `edx = (pc + 4) + 2 * index` (scale-2 lea; `pc + 4` wraps
        // as disp32 exactly like the interpreter's wrapping add).
        let base = pc.wrapping_add(4) as i32;
        dynasm!(ops ; .arch x64 ; lea edx, [rax * 2 + base]);
    });
    emit_dynamic_edge(ops, tape.take());
    true
}

/// Carry source for flag-setting logical ops (P2-5, S5): mirrors
/// `shift_carry_or_current` — an explicit bit (modified-immediate
/// rotation) or the live C (unrotated immediates, zero shifts), or a bit
/// saved in `R8B` by the register-shift prologue.
#[derive(Clone, Copy)]
enum CarrySrc {
    Static(bool),
    Current,
    SavedR8,
}

/// Inject the carry into host CF after the result test (P2-5, S5):
/// `bt` preserves SF/ZF (only CF changes), `clc`/`stc` likewise — so the
/// following `lahf` sees N/Z from the test and C from here.
fn inject_carry(ops: &mut A, carry: CarrySrc) {
    match carry {
        CarrySrc::Static(true) => dynasm!(ops ; .arch x64 ; stc),
        CarrySrc::Static(false) => dynasm!(ops ; .arch x64 ; clc),
        CarrySrc::Current => dynasm!(ops ; .arch x64 ; bt r15d, 29),
        CarrySrc::SavedR8 => dynasm!(ops ; .arch x64 ; bt r8d, 0),
    }
}

/// Shift `Rm` into `ECX` for 32-bit DP (P2-5, S5): mirrors the
/// interpreter's `(Ror, 0) → RRX` split plus the `shift_imm` table
/// (`amount <= 31`, gated upstream). Returns the carry for flag-setting
/// logical ops (`Current` = live C; arithmetic ops discard it — they set
/// their own). x86 shift/rotate CF matches ARM's last-out for 1..=31
/// (P2-2 established); the carry-out parks in `R8B` (dead in DP
/// templates, guard-safe).
fn shift_rm_to_ecx(
    ops: &mut A,
    rm: u8,
    shift: crate::cpu::decode32::ShiftType,
    amount: u8,
) -> CarrySrc {
    use crate::cpu::decode32::ShiftType as S;
    load(ops, rm, true);
    let n = amount as i8;
    match (shift, amount) {
        (S::Ror, 0) => {
            // RRX: `op2 = (rm >> 1) | (C << 31)`, carry = bit0(rm).
            dynasm!(ops
                ; .arch x64
                ; mov r8d, eax
                ; and r8d, 1
                ; shr eax, 1
                ; bt r15d, 29
                ; setc cl
                ; shl ecx, 31
                ; or eax, ecx
                ; mov ecx, eax
            );
            CarrySrc::SavedR8
        }
        (S::Ror, _) => {
            dynasm!(ops ; .arch x64 ; ror eax, n ; setc r8b ; mov ecx, eax);
            CarrySrc::SavedR8
        }
        (S::Lsl, 0) | (S::Lsr, 0) | (S::Asr, 0) => {
            dynasm!(ops ; .arch x64 ; mov ecx, eax);
            CarrySrc::Current
        }
        (S::Lsl, _) => {
            dynasm!(ops ; .arch x64 ; shl eax, n ; setc r8b ; mov ecx, eax);
            CarrySrc::SavedR8
        }
        (S::Lsr, _) => {
            dynasm!(ops ; .arch x64 ; shr eax, n ; setc r8b ; mov ecx, eax);
            CarrySrc::SavedR8
        }
        (S::Asr, _) => {
            dynasm!(ops ; .arch x64 ; sar eax, n ; setc r8b ; mov ecx, eax);
            CarrySrc::SavedR8
        }
    }
}

/// Flag-setting logical tail (P2-5, S5): result in `eax` (stored by the
/// caller, or discarded for tests), N/Z re-tested, C injected, V preserved
/// from the pinned word (logical ops never compute V — mirrors
/// `exec_dp32`). `dl` is seeded with the live V up front by the caller;
/// the compute never touches it.
fn logical_tail(ops: &mut A, carry: CarrySrc) {
    dynasm!(ops ; .arch x64 ; test eax, eax);
    inject_carry(ops, carry);
    dynasm!(ops ; .arch x64 ; lahf);
    merge_nzcv(ops);
}

/// 32-bit DP compute+merge core (P2-5, S5): `eax` = Rn value, `ecx` =
/// op2. Logical ops take C from `carry` (V preserved); arithmetic ops set
/// their own (host add/sub + `cmc`/`seto`, P2-2 patterns). `set_flags =
/// false` computes + stores with zero flag traffic.
fn exec_dp_op(
    ops: &mut A,
    op: crate::cpu::decode32::DpOp,
    rd: u8,
    set_flags: bool,
    carry: CarrySrc,
) {
    use crate::cpu::decode32::DpOp as O;
    if set_flags && !matches!(op, O::Cmp | O::Cmn) {
        // Live V into `dl` before the compute (compute never touches it).
        // Only the compare forms skip the seed (they overwrite `dl` via
        // `seto`); `Tst`/`Teq` need it (they preserve V through
        // `merge_nzcv` — caught by differential).
        dynasm!(ops ; .arch x64 ; mov edx, r15d ; shr edx, 28 ; and edx, 1);
    }
    let writes = super::templates::dp_writes_rd(op);
    match op {
        O::And => {
            dynasm!(ops ; .arch x64 ; and eax, ecx);
            store(ops, rd);
            if set_flags {
                logical_tail(ops, carry);
            }
        }
        O::Eor | O::Teq => {
            dynasm!(ops ; .arch x64 ; xor eax, ecx);
            if writes {
                store(ops, rd);
            }
            if set_flags {
                logical_tail(ops, carry);
            }
        }
        O::Bic => {
            dynasm!(ops ; .arch x64 ; not ecx ; and eax, ecx);
            store(ops, rd);
            if set_flags {
                logical_tail(ops, carry);
            }
        }
        O::Orr => {
            dynasm!(ops ; .arch x64 ; or eax, ecx);
            store(ops, rd);
            if set_flags {
                logical_tail(ops, carry);
            }
        }
        O::Orn => {
            dynasm!(ops ; .arch x64 ; not ecx ; or eax, ecx);
            store(ops, rd);
            if set_flags {
                logical_tail(ops, carry);
            }
        }
        O::Mov => {
            dynasm!(ops ; .arch x64 ; mov eax, ecx);
            store(ops, rd);
            if set_flags {
                logical_tail(ops, carry);
            }
        }
        O::Mvn => {
            dynasm!(ops ; .arch x64 ; mov eax, ecx ; not eax);
            store(ops, rd);
            if set_flags {
                logical_tail(ops, carry);
            }
        }
        O::Add | O::Cmn => {
            dynasm!(ops ; .arch x64 ; add eax, ecx);
            if writes {
                store(ops, rd);
            }
            if set_flags {
                dynasm!(ops ; .arch x64 ; seto dl ; lahf);
                merge_nzcv(ops);
            }
        }
        O::Adc => {
            dynasm!(ops ; .arch x64 ; bt r15d, 29);
            dynasm!(ops ; .arch x64 ; adc eax, ecx);
            store(ops, rd);
            if set_flags {
                dynasm!(ops ; .arch x64 ; seto dl ; lahf);
                merge_nzcv(ops);
            }
        }
        O::Sub | O::Cmp => {
            dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc);
            if writes {
                store(ops, rd);
            }
            if set_flags {
                dynasm!(ops ; .arch x64 ; seto dl ; lahf);
                merge_nzcv(ops);
            }
        }
        O::Sbc => {
            // Borrow-in is !C: seed CF from the pinned word first (entry
            // CF is garbage — the prologue only moves — then complement
            // twice around the subtract, P2-2 pattern).
            dynasm!(ops ; .arch x64 ; bt r15d, 29);
            dynasm!(ops ; .arch x64 ; cmc ; sbb eax, ecx ; cmc);
            store(ops, rd);
            if set_flags {
                dynasm!(ops ; .arch x64 ; seto dl ; lahf);
                merge_nzcv(ops);
            }
        }
        O::Rsb => {
            // `res = op2 - Rn` (in `ecx`, then moved for the store — `mov`
            // preserves the flags the merge reads).
            dynasm!(ops ; .arch x64 ; sub ecx, eax ; mov eax, ecx ; cmc);
            store(ops, rd);
            if set_flags {
                dynasm!(ops ; .arch x64 ; seto dl ; lahf);
                merge_nzcv(ops);
            }
        }
        O::Tst => {
            // Gated on `set_flags` (unlike 16-bit `exec_alu`, `exec_dp32`
            // does not always-update); when set, N/Z from `rn & op2`, C
            // from the shift/expand carry, V preserved. The test runs here
            // (the shared tail re-tests `eax`, which for non-storing test
            // forms does NOT hold the result — caught by differential).
            dynasm!(ops ; .arch x64 ; test eax, ecx);
            if set_flags {
                inject_carry(ops, carry);
                dynasm!(ops ; .arch x64 ; lahf);
                merge_nzcv(ops);
            }
        }
    }
}

/// `DpImm` (P2-5, S5): static op2 (`ThumbExpandImm` or raw imm12) with
/// the interpreter's expand-carry rule (rotated → bit31, else live C).
fn emit_dpimm(
    ops: &mut A,
    op: crate::cpu::decode32::DpOp,
    rd: u8,
    rn: u8,
    imm12: u16,
    expand: bool,
    set_flags: bool,
) -> bool {
    use crate::cpu::decode32::thumb_expand_imm;
    if rn == 15 {
        return false;
    }
    if super::templates::dp_writes_rd(op) && rd == 15 {
        return false;
    }
    let op2 = if expand {
        thumb_expand_imm(imm12)
    } else {
        u32::from(imm12)
    };
    let carry = if expand && imm12 & 0xC00 != 0 {
        CarrySrc::Static(op2 >> 31 & 1 != 0)
    } else {
        CarrySrc::Current
    };
    load(ops, rn, true);
    dynasm!(ops ; .arch x64 ; mov ecx, op2 as i32);
    exec_dp_op(ops, op, rd, set_flags, carry);
    true
}

/// `DpReg` (P2-5, S5): shifted-`Rm` op2 plus the `DpImm` core.
fn emit_dpreg(
    ops: &mut A,
    op: crate::cpu::decode32::DpOp,
    rd: u8,
    rn: u8,
    rm: u8,
    shift: crate::cpu::decode32::ShiftType,
    amount: u8,
    set_flags: bool,
) -> bool {
    // `MOV`/`MVN` never read `Rn` (the ROR-immediate shifted-move
    // forms use `rn == 15` as a no-Rn marker): the stale-PC load is
    // skipped entirely, not just tolerated.
    use crate::cpu::decode32::DpOp as O;
    if rm == 15 || (rn == 15 && !matches!(op, O::Mov | O::Mvn)) {
        return false;
    }
    if super::templates::dp_writes_rd(op) && rd == 15 {
        return false;
    }
    let shifted = shift_rm_to_ecx(ops, rm, shift, amount);
    let carry = match shifted {
        CarrySrc::SavedR8 => CarrySrc::SavedR8,
        CarrySrc::Current => CarrySrc::Current,
        CarrySrc::Static(_) => unreachable!("shifts never produce static carry"),
    };
    load(ops, rn, true);
    exec_dp_op(ops, op, rd, set_flags, carry);
    true
}

/// `MOVW`/`MOVT` (P2-5, S5): pure immediate materialization, no flags.
fn emit_movwide(ops: &mut A, rd: u8, imm: u16, high: bool) -> bool {
    if rd == 15 {
        return false;
    }
    if high {
        load(ops, rd, true);
        let top = (u32::from(imm) << 16) as i32;
        dynasm!(ops
            ; .arch x64
            ; mov ecx, top
            ; and eax, 0xFFFF
            ; or eax, ecx
        );
        store(ops, rd);
    } else {
        let v = u32::from(imm) as i32;
        dynasm!(ops ; .arch x64 ; mov eax, v);
        store(ops, rd);
    }
    true
}

/// `MUL`/`MLA`/`MLS` (P2-5, S5): low word only, never any flags.
fn emit_mul(ops: &mut A, op: crate::cpu::decode32::MulOp, rd: u8, rn: u8, rm: u8, ra: u8) -> bool {
    use crate::cpu::decode32::MulOp as O;
    if rd == 15 || rn == 15 || rm == 15 {
        return false;
    }
    if !matches!(op, O::Mul) && ra == 15 {
        return false;
    }
    load(ops, rn, true);
    load(ops, rm, false);
    dynasm!(ops ; .arch x64 ; imul eax, ecx);
    match op {
        O::Mul => {}
        O::Mla => {
            load(ops, ra, false);
            dynasm!(ops ; .arch x64 ; add eax, ecx);
        }
        O::Mls => {
            load(ops, ra, false);
            dynasm!(ops ; .arch x64 ; sub ecx, eax ; mov eax, ecx);
        }
        _ => return false,
    }
    store(ops, rd);
    true
}

/// `SDIV`/`UDIV` (P2-5, S5): never any flags. Zero divisors side-exit
/// (the interpreter owns `DIV_0_TRP`-vs-zero); signed `MIN / -1`
/// saturates natively (x86 `IDIV` would fault).
fn emit_div(ops: &mut A, signed: bool, rd: u8, rn: u8, rm: u8, pc: u32) -> bool {
    if rd == 15 || rn == 15 || rm == 15 {
        return false;
    }
    let side = ops.new_dynamic_label();
    let done = ops.new_dynamic_label();
    load(ops, rm, false);
    dynasm!(ops ; .arch x64 ; test ecx, ecx ; jz =>side);
    load(ops, rn, true);
    if signed {
        // `MIN / -1` saturates (x86 `IDIV` would fault — hence the split:
        // the saturated value must skip the divider entirely).
        let neg1 = 0xFFFF_FFFFu32 as i32;
        let min = 0x8000_0000u32 as i32;
        let divlbl = ops.new_dynamic_label();
        let stored = ops.new_dynamic_label();
        dynasm!(ops
            ; .arch x64
            ; cmp ecx, neg1
            ; jne =>divlbl
            ; cmp eax, min
            ; jne =>divlbl
            ; mov eax, min
            ; jmp =>stored
            ; =>divlbl
            ; cdq
            ; idiv ecx
            ; =>stored
        );
    } else {
        dynasm!(ops ; .arch x64 ; xor edx, edx ; div ecx);
    }
    store(ops, rd);
    dynasm!(ops ; .arch x64 ; jmp =>done);
    dynasm!(ops ; .arch x64 ; =>side);
    mem_side_exit(ops, pc);
    dynasm!(ops ; .arch x64 ; =>done);
    true
}

/// `SXTH`/`SXTB`/`UXTH`/`UXTB` (P2-5): pure extends, never any flags.
fn emit_extend(ops: &mut A, rd: u8, rm: u8, kind: crate::cpu::decode16::ExtendKind) -> bool {
    use crate::cpu::decode16::ExtendKind as K;
    if rd == 15 || rm == 15 {
        return false;
    }
    load(ops, rm, true);
    match kind {
        K::Sxth => dynasm!(ops ; .arch x64 ; movsx eax, ax),
        K::Sxtb => dynasm!(ops ; .arch x64 ; movsx eax, al),
        K::Uxth => dynasm!(ops ; .arch x64 ; movzx eax, ax),
        K::Uxtb => dynasm!(ops ; .arch x64 ; movzx eax, al),
    }
    store(ops, rd);
    true
}

/// `REV`/`REV16`/`REVSH` (P2-5): pure reversals, never any flags
/// (`REV16` = byte-swap within each halfword: `bswap` + half-swap).
fn emit_reverse(ops: &mut A, rd: u8, rm: u8, kind: crate::cpu::decode16::ReverseKind) -> bool {
    use crate::cpu::decode16::ReverseKind as K;
    if rd == 15 || rm == 15 {
        return false;
    }
    load(ops, rm, true);
    match kind {
        K::Rev => dynasm!(ops ; .arch x64 ; bswap eax),
        // `REV16` swaps within each halfword: byte-swap everything, then
        // swap the halves back (`xchg`+`ror` reads as rev16 but computes a
        // full `bswap` — caught by differential).
        K::Rev16 => dynasm!(ops ; .arch x64 ; bswap eax ; ror eax, 16),
        K::Revsh => dynasm!(ops
            ; .arch x64
            ; xchg al, ah
            ; movsx eax, ax
        ),
    }
    store(ops, rd);
    true
}

fn emit32(ops: &mut A, inst: &crate::cpu::decode32::Inst32, pc: u32, unalign_trap: bool) -> bool {
    use crate::cpu::decode32::Inst32 as I;
    match *inst {
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
            ..
        } => emit_mem32(
            ops,
            access,
            rt,
            rn,
            off,
            rm,
            shift,
            add,
            index,
            writeback,
            pc,
            unalign_trap,
        ),
        // P2-5 (S3): dual + multiple (plain forms; `pc` loads need a slot
        // — see `emit32_term`).
        I::LdrdStrd {
            is_load,
            rt,
            rt2,
            rn,
            imm,
            add,
            index,
            writeback,
        } => emit_ldrdstrd(ops, is_load, rt, rt2, rn, imm, add, index, writeback, pc),
        I::LdmStm {
            is_load,
            rn,
            regs,
            writeback,
            dec_before,
        } => emit_ldmstm32(ops, is_load, rn, regs, writeback, dec_before, pc, None),
        // P2-5 (S5): 32-bit data processing.
        I::DpReg {
            op,
            rd,
            rn,
            rm,
            shift,
            amount,
            set_flags,
        } => emit_dpreg(ops, op, rd, rn, rm, shift, amount, set_flags),
        I::DpImm {
            op,
            rd,
            rn,
            imm12,
            expand,
            set_flags,
        } => emit_dpimm(ops, op, rd, rn, imm12, expand, set_flags),
        I::MovWide { rd, imm, high } => emit_movwide(ops, rd, imm, high),
        I::Div { signed, rd, rn, rm } => emit_div(ops, signed, rd, rn, rm, pc),
        I::Mul { op, rd, rn, rm, ra } => emit_mul(ops, op, rd, rn, rm, ra),
        // P2-6 (S6): barriers are silent fallthroughs.
        I::Barrier { .. } => true,
        // P2-6 (S6): MRS/MSR split inline vs side-exit.
        I::Mrs { rd, sys } => emit_mrs(ops, rd, sys, pc),
        I::Msr { rn, sys } => emit_msr(ops, rn, sys, pc),
        // Anything else templated (exclusives, DSP, FP, TT, SG,
        // hard MRS/MSR) is a flush + driver re-execution tail (P2-6):
        // the fallthrough owns it so coverage can never go stale here
        // when `is_templated` grows. Terminators are excluded (their
        // edges live in the terminator emitter).
        _ => {
            let op = TbOp::V32(*inst);
            if !super::templates::is_terminator(&op) && super::templates::is_templated(&op) {
                mem_side_exit(ops, pc);
                true
            } else {
                false
            }
        }
    }
}

/// Merge a precomposed NZCV word from `esi` (N/Z/C/V at ARM positions).
fn merge_reg(ops: &mut A) {
    dynasm!(ops
        ; .arch x64
        ; mov ecx, r15d
        ; and ecx, 0x0FFFFFFF
        ; or ecx, esi
        ; mov r15d, ecx
    );
}

/// Register-controlled shift sequence. Entry: `eax` = value, `ecx` = raw
/// count. Masks to a byte (matching `& 0xFF`), saves the value for
/// big-case carry, stores the result, merges per the interpreter's
/// `shift_reg`/`shift_by_count` truth tables. Labels are dynamic (fresh
/// per emission), so multi-op blocks never collide.
fn shift_reg_seq(ops: &mut A, rd: u8, kind: crate::cpu::decode16::Shift, pred: bool) -> bool {
    use crate::cpu::decode16::Shift as S;
    if kind == S::Ror {
        return ror_reg_seq(ops, rd, pred);
    }
    let big = ops.new_dynamic_label();
    let copy = ops.new_dynamic_label();
    let end = ops.new_dynamic_label();
    dynasm!(ops
        ; .arch x64
        ; and ecx, 0xFF
        ; mov esi, eax
        ; cmp ecx, 32
        ; jae =>big
        ; test ecx, ecx
        ; jz =>copy
    );
    match kind {
        S::Lsl => dynasm!(ops ; .arch x64 ; shl eax, cl),
        S::Lsr => dynasm!(ops ; .arch x64 ; shr eax, cl),
        S::Asr => dynasm!(ops ; .arch x64 ; sar eax, cl),
        S::Ror => unreachable!("dispatched above"),
    }
    store(ops, rd);
    dynasm!(ops ; .arch x64 ; lahf ; xor edx, edx);
    // S4: predicated S-forms skip the merge (extracts touch dead scratch
    // only — the merge is the sole architectural flag write).
    if !pred {
        merge_nzcv(ops);
    }
    dynasm!(ops ; .arch x64 ; jmp =>end);
    // Zero count: copy through with N/Z set, C preserved, V cleared
    // (matches `write_shift_flags(res, None)` — only C is conditional).
    dynasm!(ops ; .arch x64 ; =>copy);
    store(ops, rd);
    dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
    if !pred {
        merge_nz(ops);
    }
    dynasm!(ops ; .arch x64 ; jmp =>end);
    // Big counts per kind. NOTE: mem-path stores reload `rsi`, killing
    // `esi` — so `v` is spilled to the frame pad slot `[rsp + 8]` on entry
    // and reloaded after the store (caught by differential: pinned stores
    // don't touch `rsi`, which is why this only failed for mem lanes).
    dynasm!(ops ; .arch x64 ; =>big);
    match kind {
        S::Lsl => {
            // Result 0; C = (count == 32) & bit0(v); N = 0, Z = 1, V = 0.
            dynasm!(ops
                ; .arch x64
                ; mov [rsp + 8], eax
                ; xor eax, eax
            );
            store(ops, rd);
            dynasm!(ops
                ; .arch x64
                ; mov esi, [rsp + 8]
                ; cmp ecx, 32
                ; sete dl
                ; and esi, 1
                ; and sil, dl
                ; shl esi, 29
                ; or esi, 0x40000000
            );
            if !pred {
                merge_reg(ops);
            }
        }
        S::Lsr => {
            // Result 0; C = (count == 32) & bit31(v); N = 0, Z = 1, V = 0.
            dynasm!(ops
                ; .arch x64
                ; mov [rsp + 8], eax
                ; xor eax, eax
            );
            store(ops, rd);
            dynasm!(ops
                ; .arch x64
                ; mov esi, [rsp + 8]
                ; cmp ecx, 32
                ; sete dl
                ; shr esi, 31
                ; and sil, dl
                ; shl esi, 29
                ; or esi, 0x40000000
            );
            if !pred {
                merge_reg(ops);
            }
        }
        S::Asr => {
            // Result signfill; C = N = sign(v); Z = !sign; V = 0.
            dynasm!(ops
                ; .arch x64
                ; mov [rsp + 8], eax
                ; sar eax, 31
            );
            store(ops, rd);
            dynasm!(ops
                ; .arch x64
                ; mov esi, [rsp + 8]
                ; shr esi, 31
                ; mov edx, esi
                ; shl edx, 29
                ; shl esi, 31
                ; or edx, esi
                ; test esi, esi
                ; setz cl
                ; shl ecx, 30
                ; or edx, ecx
                ; mov esi, edx
            );
            if !pred {
                merge_reg(ops);
            }
        }
        S::Ror => unreachable!("dispatched above"),
    }
    dynasm!(ops ; .arch x64 ; =>end);
    true
}

/// Register-controlled rotate. Entry: `eax` = value, `ecx` = raw count.
/// Classes: zero (unchanged, C preserved), nonzero multiple of 32
/// (unchanged, C = bit31), else rotate with CF carry. Dynamic labels.
fn ror_reg_seq(ops: &mut A, rd: u8, pred: bool) -> bool {
    let mult32 = ops.new_dynamic_label();
    let copy = ops.new_dynamic_label();
    let end = ops.new_dynamic_label();
    dynasm!(ops
        ; .arch x64
        ; and ecx, 0xFF
        ; test ecx, ecx
        ; jz =>copy
        ; and ecx, 0x1F
        ; jz =>mult32
        // NB: x86 ROR sets CF but leaves SF/ZF undefined, so N/Z come
        // from an explicit test and C is spliced into the LAHF image
        // (`or` runs after `lahf`: nothing reads host flags past it).
        // `cl` (count, dead) holds C: `ah` cannot pair with REX byte
        // regs like `sil`.
        ; ror eax, cl
        ; setc cl
    );
    store(ops, rd);
    dynasm!(ops
        ; .arch x64
        ; test eax, eax
        ; lahf
        ; or ah, cl
        ; xor edx, edx
    );
    if !pred {
        merge_nzcv(ops);
    }
    dynasm!(ops ; .arch x64 ; jmp =>end);
    // Zero count: copy through with N/Z set, C preserved, V cleared.
    dynasm!(ops ; .arch x64 ; =>copy);
    store(ops, rd);
    dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
    if !pred {
        merge_nz(ops);
    }
    dynasm!(ops ; .arch x64 ; jmp =>end);
    // Nonzero multiple of 32: unchanged, C = bit31, V = 0.
    dynasm!(ops ; .arch x64 ; =>mult32);
    store(ops, rd);
    dynasm!(ops
        ; .arch x64
        ; mov esi, eax
        ; shr esi, 31
        ; mov edx, esi
        ; shl edx, 29
        ; shl esi, 31
        ; or edx, esi
        ; test eax, eax
        ; setz cl
        ; shl ecx, 30
        ; or edx, ecx
        ; mov esi, edx
    );
    if !pred {
        merge_reg(ops);
    }
    dynasm!(ops ; .arch x64 ; =>end);
    true
}

/// ALU-op dispatcher (16-bit `Alu` family; `set_flags` is always true in
/// native blocks since `itstate == 0` throughout).
fn emit_alu(
    ops: &mut A,
    rd: u8,
    rm: u8,
    op: crate::cpu::decode16::AluOp,
    pred: bool,
    merge: bool,
) -> bool {
    use crate::cpu::decode16::AluOp as O;
    if rd == 15 || rm == 15 {
        return false;
    }
    match op {
        O::Ror | O::Lsl | O::Lsr | O::Asr => {
            // Register shifts read the count from Rm (not Rd): load the
            // value from Rd and the count from Rm, then share the sequence.
            load(ops, rd, true);
            load(ops, rm, false);
            let kind = match op {
                O::Lsl => crate::cpu::decode16::Shift::Lsl,
                O::Lsr => crate::cpu::decode16::Shift::Lsr,
                O::Asr => crate::cpu::decode16::Shift::Asr,
                _ => crate::cpu::decode16::Shift::Ror,
            };
            return shift_reg_seq(ops, rd, kind, pred);
        }
        O::Adc | O::Sbc => {
            load(ops, rd, true);
            load(ops, rm, false);
            // Carry in from the pinned flags word (`bt` sets CF = bit 29).
            dynasm!(ops ; .arch x64 ; bt r15d, 29);
            match op {
                O::Adc => dynasm!(ops ; .arch x64 ; adc eax, ecx),
                // Borrow in is !C (`cmc` complements CF only); borrow OUT
                // is likewise inverted vs ARM C, hence the second `cmc`.
                _ => dynasm!(ops ; .arch x64 ; cmc ; sbb eax, ecx ; cmc),
            }
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; seto dl ; lahf);
            // G3 flag traffic (Adc/Sbc read carry via `bt` above — the
            // read stays unconditional; only the merge gates).
            if !pred && merge {
                merge_nzcv(ops);
            }
            return true;
        }
        _ => {}
    }
    // Plain two-operand ops: value in eax, operand in ecx.
    let (load_b, compute) = match op {
        O::And => (true, "and"),
        O::Eor => (true, "eor"),
        O::Orr => (true, "orr"),
        O::Bic => (true, "bic-not"),
        O::Tst => (true, "test"),
        O::Cmp => (true, "cmp"),
        O::Cmn => (true, "add-nostore"),
        O::Mul => (true, "mul"),
        O::Mvn => (false, "mvn"),
        O::Neg => (false, "neg"),
        _ => unreachable!("shifts/adc/sbc handled above"),
    };
    load(ops, rd, true);
    if load_b {
        load(ops, rm, false);
    } else if matches!(op, O::Neg | O::Mvn) {
        load(ops, rm, true);
    }
    match compute {
        "and" => {
            dynasm!(ops ; .arch x64 ; and eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; lahf);
            if !pred {
                merge_nz(ops);
            }
        }
        "eor" => {
            dynasm!(ops ; .arch x64 ; xor eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; lahf);
            if !pred {
                merge_nz(ops);
            }
        }
        "orr" => {
            dynasm!(ops ; .arch x64 ; or eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; lahf);
            if !pred {
                merge_nz(ops);
            }
        }
        "bic-not" => {
            dynasm!(ops ; .arch x64 ; not ecx ; and eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; lahf);
            if !pred {
                merge_nz(ops);
            }
        }
        "test" => {
            dynasm!(ops ; .arch x64 ; test eax, ecx ; lahf);
            merge_nz(ops);
        }
        "cmp" => {
            // Pure flags: vanish entirely when dead.
            if merge {
                dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc ; seto dl ; lahf);
                merge_nzcv(ops);
            }
        }
        "add-nostore" => {
            // Pure flags: vanish entirely when dead.
            if merge {
                dynasm!(ops ; .arch x64 ; add eax, ecx ; seto dl ; lahf);
                merge_nzcv(ops);
            }
        }
        "mul" => {
            // 2-operand IMUL leaves SF/ZF undefined: re-test for N/Z.
            dynasm!(ops ; .arch x64 ; imul eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
            if !pred {
                merge_nz(ops);
            }
        }
        "mvn" => {
            dynasm!(ops ; .arch x64 ; not eax);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
            if !pred {
                merge_nz(ops);
            }
        }
        "neg" => {
            // RSB: res = 0 - Rm; x86 NEG leaves inverted carry vs ARM.
            // Store BEFORE flag extraction: `lahf` overwrites ah, i.e.
            // result bits 15:8 (caught by differential trial 0).
            dynasm!(ops ; .arch x64 ; neg eax ; cmc);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; seto dl ; lahf);
            if !pred && merge {
                merge_nzcv(ops);
            }
        }
        _ => unreachable!("covered above"),
    }
    true
}

/// Shift-immediate kinds after the interpreter's `shift_imm` truth table.
/// `merge` gates only the nonzero-amount full merge (G3 flag traffic);
/// the zero-amount C-preserving merge always runs (preservation reads
/// must see fresh C — same rule as `merge_nz` everywhere).
fn emit_shift_imm(
    ops: &mut A,
    rd: u8,
    rm: u8,
    kind: ShiftImm,
    amount: u8,
    pred: bool,
    merge: bool,
) -> bool {
    if rd == 15 || rm == 15 {
        return false;
    }
    load(ops, rm, true);
    let n = amount as i8;
    if n == 0 {
        // Copy with C preserved, V cleared (matches `shift_imm` + `set_nz`).
        store(ops, rd);
        dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
        if !pred {
            merge_nz(ops);
        }
        return true;
    }
    match kind {
        ShiftImm::Lsl => dynasm!(ops ; .arch x64 ; shl eax, n),
        ShiftImm::Lsr => dynasm!(ops ; .arch x64 ; shr eax, n),
        ShiftImm::Asr => dynasm!(ops ; .arch x64 ; sar eax, n),
    }
    store(ops, rd);
    // CF holds the last bit out (x86 shift semantics match ARM here for
    // 1..=31); V is always false for shifts.
    dynasm!(ops ; .arch x64 ; lahf ; xor edx, edx);
    if !pred && merge {
        merge_nzcv(ops);
    }
    true
}
