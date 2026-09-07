//! AArch64 integer ALU templates (S0, P2-2).
//!
//! Mirrors `templates_x64` op-for-op (same `is_templated` set — CI parity).
//! AArch64 is kinder to flags: `adds`/`subs`/`adcs`/`sbcs` set NZCV
//! natively with ARM semantics (no `cmc` fixups), read back via `cset`
//! (no `mrs` sysreg access anywhere — zero ISA-encoding risk) and merged
//! into the pinned flags word (`w23`). Shifts set no flags, so carry is
//! computed explicitly (like the x64 big-cases, but on every shift).
//! Carry-in for `adcs`/`sbcs` comes from a C-seeding compare pair (two
//! dynamic labels); only C matters downstream.
//!
//! Scratch: `w0` (result/op1), `w1` (op2), `w9`–`w11` (merge/scratch),
//! `x9` for addresses. `w0` doubles as the exit register (epilogue
//! overwrites it, so templates may clobber it freely).

use dynasmrt::{dynasm, DynasmApi, DynasmLabelApi};

use crate::cpu::decode16::ShiftImm;
use crate::cpu::registers::Registers;
use crate::jit::builder::TbOp;

/// Concrete assembler for this host (re-exported as `arch::Assembler`).
pub type Assembler = dynasmrt::aarch64::Assembler;

type A = Assembler;

/// NativeCtx field offsets (`u32` immediates below).
fn tlb_off() -> u32 {
    core::mem::offset_of!(super::native::NativeCtx, tlb) as u32
}

fn priv_off() -> u32 {
    core::mem::offset_of!(super::native::NativeCtx, privileged) as u32
}

fn epoch_off() -> u32 {
    core::mem::offset_of!(super::native::NativeCtx, write_gen) as u32
}

fn fault_off() -> u32 {
    core::mem::offset_of!(super::native::NativeCtx, fault_addr) as u32
}

fn regs_off() -> u32 {
    core::mem::offset_of!(crate::jit::NativeCtx, regs) as u32
}

fn reg_off(i: usize) -> u32 {
    (core::mem::offset_of!(Registers, r) + i * 4) as u32
}

/// Memory-path offset mirroring [`Registers::read`]/`write` (see the x64
/// `mem_reg_off`: 14 is `lr`, never the adjacent `msp`).
fn mem_reg_off(guest: u8) -> u32 {
    match guest {
        13 => core::mem::offset_of!(Registers, sp) as u32,
        14 => core::mem::offset_of!(Registers, lr) as u32,
        _ => reg_off(guest as usize),
    }
}

/// Load guest `r[guest]` into `w0` (`first`) or `w1`.
fn load(ops: &mut A, guest: u8, first: bool) {
    let off = mem_reg_off(guest);
    match (guest, first) {
        (0, true) => dynasm!(ops ; .arch aarch64 ; mov w0, w19),
        (1, true) => dynasm!(ops ; .arch aarch64 ; mov w0, w20),
        (2, true) => dynasm!(ops ; .arch aarch64 ; mov w0, w21),
        (3, true) => dynasm!(ops ; .arch aarch64 ; mov w0, w22),
        (0, false) => dynasm!(ops ; .arch aarch64 ; mov w1, w19),
        (1, false) => dynasm!(ops ; .arch aarch64 ; mov w1, w20),
        (2, false) => dynasm!(ops ; .arch aarch64 ; mov w1, w21),
        (3, false) => dynasm!(ops ; .arch aarch64 ; mov w1, w22),
        _ => {
            let ctx = regs_off();
            if first {
                dynasm!(ops
                    ; .arch aarch64
                    ; ldr x9, [sp, #48]
                    ; ldr x9, [x9, ctx]
                    ; ldr w0, [x9, off]
                );
            } else {
                dynasm!(ops
                    ; .arch aarch64
                    ; ldr x9, [sp, #48]
                    ; ldr x9, [x9, ctx]
                    ; ldr w1, [x9, off]
                );
            }
        }
    }
}

/// Store `w0` into guest `r[guest]`.
fn store(ops: &mut A, guest: u8) {
    let off = mem_reg_off(guest);
    match guest {
        0 => dynasm!(ops ; .arch aarch64 ; mov w19, w0),
        1 => dynasm!(ops ; .arch aarch64 ; mov w20, w0),
        2 => dynasm!(ops ; .arch aarch64 ; mov w21, w0),
        3 => dynasm!(ops ; .arch aarch64 ; mov w22, w0),
        _ => {
            let ctx = regs_off();
            dynasm!(ops
                ; .arch aarch64
                ; ldr x9, [sp, #48]
                ; ldr x9, [x9, ctx]
                ; str w0, [x9, off]
            );
        }
    }
}

/// Write back an NZCV word sitting in `x9` (top nibble) to the pinned word.
fn writeback(ops: &mut A) {
    dynasm!(ops
        ; .arch aarch64
        ; lsr w9, w9, #28
        ; bfi w23, w9, #28, #4
    );
}

/// Merge full NZCV from the live condition flags (read immediately after
/// the flag-setting op + stores only — `cset` reads flags without
/// touching them, so no `mrs` sysreg access is needed anywhere).
fn merge_nzcv(ops: &mut A) {
    dynasm!(ops
        ; .arch aarch64
        ; cset w9, mi
        ; cset w10, eq
        ; cset w11, cs
        ; cset w12, vs
        ; lsl w9, w9, #31
        ; lsl w10, w10, #30
        ; lsl w11, w11, #29
        ; lsl w12, w12, #28
        ; orr w9, w9, w10
        ; orr w9, w9, w11
        ; orr w9, w9, w12
    );
    writeback(ops);
}

/// Merge N/Z from the live flags; preserve C, clear V.
fn merge_nz(ops: &mut A) {
    dynasm!(ops
        ; .arch aarch64
        ; cset w9, mi
        ; cset w10, eq
        ; lsl w9, w9, #31
        ; lsl w10, w10, #30
        ; orr w9, w9, w10
        ; bfi w23, w9, #30, #2
        ; bfc w23, #28, #1
    );
}

/// Emit one S0 op. Returns `false` when `op` has no template.
pub fn emit_op(ops: &mut A, op: &TbOp, pc: u32, unalign_trap: bool) -> bool {
    match *op {
        TbOp::V16(inst) => emit16(ops, &inst, pc, unalign_trap),
        TbOp::V32(inst) => emit32(ops, &inst, pc, unalign_trap),
    }
}

/// Define a marker label at the current position (slot/chain plumbing).
pub fn mark_label(ops: &mut A, lbl: dynasmrt::DynamicLabel) {
    dynasm!(ops ; .arch aarch64 ; =>lbl);
}

/// Driver-entry jump over the chained-entry stub (mirrors x64).
/// Returns the body label for binding after the stub.
pub fn emit_entry_jump(ops: &mut A) -> dynasmrt::DynamicLabel {
    let body = ops.new_dynamic_label();
    dynasm!(ops ; .arch aarch64 ; b =>body);
    body
}

/// `FALL` edge to `end_pc` through `slot` (sequential chaining).
pub fn emit_fall_edge(ops: &mut A, end_pc: u32, slot: u64) {
    super::native::emit_static_edge(ops, super::native::EXIT_FALL, end_pc, slot);
}

/// Emit a block terminator (P2-2 static branches with P2-4 chained edges,
/// P2-4 register-indirect and 32-bit branches). Consumes its edge slots
/// from `tape` in [`exit_shape`](super::templates::exit_shape) order.
pub fn emit_terminator(
    ops: &mut A,
    op: &TbOp,
    pc: u32,
    unalign_trap: bool,
    tape: &mut super::templates::SlotTape<'_>,
) -> bool {
    let _ = unalign_trap;
    match *op {
        TbOp::V16(inst) => emit16_term(ops, &inst, pc, tape),
        TbOp::V32(inst) => emit32_term(ops, &inst, pc, tape),
    }
}

fn emit16(ops: &mut A, inst: &crate::cpu::decode16::Inst16, pc: u32, unalign_trap: bool) -> bool {
    use crate::cpu::decode16::Inst16 as I;
    match *inst {
        I::MovImm { rd, imm } => {
            if rd == 15 {
                return false;
            }
            let imm = u32::from(imm);
            dynasm!(ops ; .arch aarch64 ; mov w0, imm);
            store(ops, rd);
            dynasm!(ops ; .arch aarch64 ; tst w0, w0);
            merge_nz(ops);
            true
        }
        I::AddReg { rd, rn, rm } => alu_rr(ops, rd, rn, rm, false),
        I::SubReg { rd, rn, rm } => alu_rr(ops, rd, rn, rm, true),
        I::AddImm3 { rd, rn, imm } => alu_ri(ops, rd, rn, u32::from(imm), false),
        I::SubImm3 { rd, rn, imm } => alu_ri(ops, rd, rn, u32::from(imm), true),
        I::AddImm8 { rd, imm } => alu_ri(ops, rd, rd, u32::from(imm), false),
        I::SubImm8 { rd, imm } => alu_ri(ops, rd, rd, u32::from(imm), true),
        I::CmpImm { rn, imm } => {
            if rn == 15 {
                return false;
            }
            load(ops, rn, true);
            let imm = u32::from(imm);
            // Compare and discard (`subs` to xzr sets flags natively).
            dynasm!(ops ; .arch aarch64 ; subs w9, w0, imm);
            merge_nzcv(ops);
            true
        }
        I::ShiftImm {
            rd,
            rm,
            kind,
            amount,
        } => emit_shift_imm(ops, rd, rm, kind, amount),
        I::ShiftReg { rd, rn, rm, kind } => {
            if rd == 15 || rn == 15 || rm == 15 {
                return false;
            }
            load(ops, rn, true);
            load(ops, rm, false);
            shift_reg_seq(ops, rd, kind)
        }
        I::Alu { rd, rm, op } => emit_alu(ops, rd, rm, op),
        I::SpecialCmp { rn, rm } => {
            if rn == 15 || rm == 15 {
                return false;
            }
            load(ops, rn, true);
            load(ops, rm, false);
            dynasm!(ops ; .arch aarch64 ; subs w9, w0, w1);
            merge_nzcv(ops);
            true
        }
        // --- Loads/stores (P2-3 S1, mirroring x64).
        I::LsReg { rt, rn, rm, kind } => emit_lsreg(ops, rt, rn, rm, kind, pc, unalign_trap),
        I::LsImm { rt, rn, imm, kind } => emit_lsimm(ops, rt, rn, imm, kind, pc, unalign_trap),
        I::LsSp { rt, imm, is_load } => emit_lssp(ops, rt, imm, is_load, pc, unalign_trap),
        I::LdrLiteral { rt, imm } => emit_ldrliteral(ops, rt, imm, pc, unalign_trap),
        // Terminators live in `emit_terminator` (slot-consuming edges).
        _ => false,
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
            emit_static_edge(ops, EXIT_BRANCH, b_target(pc, off), tape.take());
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
            // Taken edge first (inverted test): slot order matches
            // `exit_shape` (taken, fall).
            let fall = ops.new_dynamic_label();
            if nonzero {
                dynasm!(ops ; .arch aarch64 ; cbz w0, =>fall);
            } else {
                dynasm!(ops ; .arch aarch64 ; cbnz w0, =>fall);
            }
            emit_static_edge(ops, EXIT_BRANCH, b_target(pc, off), tape.take());
            dynasm!(ops ; .arch aarch64 ; =>fall);
            emit_static_edge(ops, EXIT_BRANCH, pc.wrapping_add(2), tape.take());
            true
        }
        I::Bx { rm } => emit_bx(ops, rm, pc, false, tape),
        I::Blx { rm } => emit_bx(ops, rm, pc, true, tape),
        _ => false,
    }
}

/// 32-bit terminators (P2-4, mirroring x64).
fn emit32_term(
    ops: &mut A,
    inst: &crate::cpu::decode32::Inst32,
    pc: u32,
    tape: &mut super::templates::SlotTape<'_>,
) -> bool {
    use super::native::{emit_static_edge, EXIT_BRANCH};
    use crate::cpu::decode32::Inst32 as I;
    match *inst {
        I::Bl { off } => {
            store_lr_imm(ops, pc.wrapping_add(4) | 1);
            emit_static_edge(ops, EXIT_BRANCH, b_target(pc, off), tape.take());
            true
        }
        I::BranchWide { off } => {
            emit_static_edge(ops, EXIT_BRANCH, b_target(pc, off), tape.take());
            true
        }
        I::BranchCondWide { cond, off } => {
            if cond > 13 {
                return false;
            }
            emit_cond_pair(ops, cond, b_target(pc, off), pc.wrapping_add(4), tape);
            true
        }
        _ => false,
    }
}

/// Store the 32-bit immediate `v` to the guest `LR` (memory path — `lr`
/// is never pinned). Preserves `w0` (the dynamic-edge target register).
fn store_lr_imm(ops: &mut A, v: u32) {
    let ctx = regs_off();
    let off = lr_off();
    let (lo, hi) = (v & 0xFFFF, (v >> 16) & 0xFFFF);
    dynasm!(ops
        ; .arch aarch64
        ; ldr x9, [sp, #48]
        ; ldr x9, [x9, ctx]
        ; movz w10, lo
        ; movk w10, hi, lsl #16
        ; str w10, [x9, off]
    );
}

fn lr_off() -> u32 {
    core::mem::offset_of!(Registers, lr) as u32
}

/// Register-indirect branch (P2-4, AArch64 mirror): magic-range side exit
/// first (the interpreter dispatches magic before `check_branch` — an
/// even magic value must not halt), then the even-target halt, then the
/// fast dynamic edge. The floor needs two moves (no imm12 form); `w10`
/// is free here (target still in `w0`).
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
    let (m_lo, m_hi) = (
        (FNC_RETURN_MIN_MAGIC & 0xFFFF) as u32,
        ((FNC_RETURN_MIN_MAGIC >> 16) & 0xFFFF) as u32,
    );
    dynasm!(ops
        ; .arch aarch64
        ; movz w10, m_lo
        ; movk w10, m_hi, lsl #16
        ; cmp w0, w10
        ; b.hs =>side
        ; tbz w0, #0, =>halt
    );
    if link {
        store_lr_imm(ops, pc.wrapping_add(2) | 1);
    }
    // Clear the Thumb bit (set on this path — even targets halted out;
    // shift pair avoids logical-immediate encoding questions entirely).
    dynasm!(ops
        ; .arch aarch64
        ; lsl w0, w0, #1
        ; lsr w0, w0, #1
    );
    emit_dynamic_edge(ops, tape.take());
    dynasm!(ops ; .arch aarch64 ; =>halt);
    emit_epilogue(ops, EXIT_HALT, pc);
    dynasm!(ops ; .arch aarch64 ; =>side);
    // `fault_addr` is unread on this path (`mem_access` is `None` for
    // branches): still zero the contract register for determinism (the
    // AArch64 guard contract carries it in `w1`).
    dynasm!(ops ; .arch aarch64 ; mov w1, #0);
    mem_side_exit(ops, pc);
    true
}

/// 16-bit control flow is `pc + 4`-relative.
fn b_target(pc: u32, off: i32) -> u32 {
    pc.wrapping_add(4).wrapping_add_signed(off)
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
    dynasm!(ops ; .arch aarch64 ; b.eq =>fall_lbl);
    emit_static_edge(ops, EXIT_BRANCH, taken, tape.take());
    dynasm!(ops ; .arch aarch64 ; =>fall_lbl);
    emit_static_edge(ops, EXIT_BRANCH, fall, tape.take());
}

/// Condition select: evaluate `cond` (0-13) against the pinned NZCV
/// nibble, ending with `cmp w9, #0` (host Z reflects taken-ness for the
/// caller's branch). Nibble layout: N = 8, Z = 4, C = 2, V = 1. Only
/// flag-clean insns run between the nibble tests and their `cset`s.
fn cond_bit(ops: &mut A, cond: u8) {
    dynasm!(ops
        ; .arch aarch64
        ; lsr w9, w23, #28
        ; and w9, w9, #0xF
    );
    match cond {
        0 => dynasm!(ops ; .arch aarch64 ; tst w9, #4 ; cset w9, ne), // EQ: Z
        1 => dynasm!(ops ; .arch aarch64 ; tst w9, #4 ; cset w9, eq), // NE: !Z
        2 => dynasm!(ops ; .arch aarch64 ; tst w9, #2 ; cset w9, ne), // CS: C
        3 => dynasm!(ops ; .arch aarch64 ; tst w9, #2 ; cset w9, eq), // CC: !C
        4 => dynasm!(ops ; .arch aarch64 ; tst w9, #8 ; cset w9, ne), // MI: N
        5 => dynasm!(ops ; .arch aarch64 ; tst w9, #8 ; cset w9, eq), // PL: !N
        6 => dynasm!(ops ; .arch aarch64 ; tst w9, #1 ; cset w9, ne), // VS: V
        7 => dynasm!(ops ; .arch aarch64 ; tst w9, #1 ; cset w9, eq), // VC: !V
        8 => dynasm!(ops // HI: C & !Z
            ; .arch aarch64
            ; tst w9, #2
            ; cset w10, ne
            ; tst w9, #4
            ; cset w11, eq
            ; and w9, w10, w11),
        9 => dynasm!(ops // LS: !C | Z
            ; .arch aarch64
            ; tst w9, #2
            ; cset w10, eq
            ; tst w9, #4
            ; cset w11, ne
            ; orr w9, w10, w11),
        10 => dynasm!(ops // GE: N == V
            ; .arch aarch64
            ; lsr w10, w9, #3
            ; eor w10, w10, w9
            ; and w10, w10, #1
            ; eor w9, w10, #1),
        11 => dynasm!(ops // LT: N != V
            ; .arch aarch64
            ; lsr w10, w9, #3
            ; eor w10, w10, w9
            ; and w9, w10, #1),
        12 => dynasm!(ops // GT: !Z & N == V
            ; .arch aarch64
            ; tst w9, #4
            ; cset w10, eq
            ; lsr w11, w9, #3
            ; eor w11, w11, w9
            ; and w11, w11, #1
            ; eor w11, w11, #1
            ; and w9, w11, w10),
        _ => dynasm!(ops // LE: Z | N != V
            ; .arch aarch64
            ; tst w9, #4
            ; cset w10, ne
            ; lsr w11, w9, #3
            ; eor w11, w11, w9
            ; and w11, w11, #1
            ; orr w9, w11, w10),
    }
    dynasm!(ops ; .arch aarch64 ; cmp w9, #0);
}

/// `Wd = Wn +/- Wm` with full NZCV (`subs`/`adds` match the interpreter
/// with no fixups: C is NOT-borrow natively).
fn alu_rr(ops: &mut A, rd: u8, rn: u8, rm: u8, sub: bool) -> bool {
    if rd == 15 || rn == 15 || rm == 15 {
        return false;
    }
    load(ops, rn, true);
    load(ops, rm, false);
    if sub {
        dynasm!(ops ; .arch aarch64 ; subs w0, w0, w1);
    } else {
        dynasm!(ops ; .arch aarch64 ; adds w0, w0, w1);
    }
    store(ops, rd);
    merge_nzcv(ops);
    true
}

/// `Wd = Wn +/- imm` with full NZCV.
fn alu_ri(ops: &mut A, rd: u8, rn: u8, imm: u32, sub: bool) -> bool {
    if rd == 15 || rn == 15 {
        return false;
    }
    load(ops, rn, true);
    if sub {
        dynasm!(ops ; .arch aarch64 ; subs w0, w0, imm);
    } else {
        dynasm!(ops ; .arch aarch64 ; adds w0, w0, imm);
    }
    store(ops, rd);
    merge_nzcv(ops);
    true
}

/// Register-controlled shift sequence. Entry: `w0` = value, `w1` = raw
/// count. Saves the value in `w10`, stores the result, merges per the
/// interpreter's `shift_reg`/`shift_by_count` truth tables. Labels are
/// dynamic (fresh per emission), so multi-op blocks never collide.
fn shift_reg_seq(ops: &mut A, rd: u8, kind: crate::cpu::decode16::Shift) -> bool {
    use crate::cpu::decode16::Shift as S;
    if kind == S::Ror {
        return ror_reg_seq(ops, rd);
    }
    let big = ops.new_dynamic_label();
    let copy = ops.new_dynamic_label();
    let end = ops.new_dynamic_label();
    dynasm!(ops
        ; .arch aarch64
        ; and w1, w1, #0xFF
        ; mov w10, w0
        ; cmp w1, #32
        ; b.hs =>big
        ; cbz w1, =>copy
    );
    match kind {
        S::Lsl => dynasm!(ops ; .arch aarch64 ; lsl w0, w0, w1),
        S::Lsr => dynasm!(ops ; .arch aarch64 ; lsr w0, w0, w1),
        S::Asr => dynasm!(ops ; .arch aarch64 ; asr w0, w0, w1),
        S::Ror => unreachable!("dispatched above"),
    }
    store(ops, rd);
    // Carry from the saved value: bit(32-c) for LSL, bit(c-1) otherwise
    // (`w1` still holds the 1..=31 count; only flag-clean insns so far).
    match kind {
        S::Lsl => {
            dynasm!(ops
                ; .arch aarch64
                ; mov w11, #32
                ; sub w11, w11, w1
                ; and w11, w11, #31
                ; lsr w1, w10, w11
                ; and w1, w1, #1
            );
        }
        S::Lsr | S::Asr => {
            dynasm!(ops
                ; .arch aarch64
                ; sub w11, w1, #1
                ; lsr w1, w10, w11
                ; and w1, w1, #1
            );
        }
        S::Ror => unreachable!("dispatched above"),
    }
    carry_in_w1(ops);
    dynasm!(ops ; .arch aarch64 ; b =>end);
    // Zero count: copy through with N/Z set, C preserved, V cleared.
    dynasm!(ops ; .arch aarch64 ; =>copy);
    store(ops, rd);
    dynasm!(ops ; .arch aarch64 ; tst w0, w0);
    merge_nz(ops);
    dynasm!(ops ; .arch aarch64 ; b =>end);
    // Big counts per kind.
    dynasm!(ops ; .arch aarch64 ; =>big);
    match kind {
        S::Lsl => {
            // Result 0; C = (count == 32) & bit0(v); N = 0, Z = 1, V = 0.
            dynasm!(ops ; .arch aarch64 ; movz w0, #0);
            store(ops, rd);
            dynasm!(ops
                ; .arch aarch64
                ; cmp w1, #32
                ; cset w11, eq
                ; and w10, w10, #1
                ; and w10, w10, w11
                ; lsl w10, w10, #29
                ; orr w10, w10, #0x40000000
                ; mov w9, w10
            );
            writeback(ops);
        }
        S::Lsr => {
            // Result 0; C = (count == 32) & bit31(v); N = 0, Z = 1, V = 0.
            dynasm!(ops ; .arch aarch64 ; movz w0, #0);
            store(ops, rd);
            dynasm!(ops
                ; .arch aarch64
                ; cmp w1, #32
                ; cset w11, eq
                ; lsr w10, w10, #31
                ; and w10, w10, w11
                ; lsl w10, w10, #29
                ; orr w10, w10, #0x40000000
                ; mov w9, w10
            );
            writeback(ops);
        }
        S::Asr => {
            // Result signfill; C = N = sign(v); Z = !sign; V = 0.
            dynasm!(ops ; .arch aarch64 ; asr w0, w10, #31);
            store(ops, rd);
            dynasm!(ops
                ; .arch aarch64
                ; lsr w10, w10, #31
                ; lsl w11, w10, #29
                ; lsl w9, w10, #31
                ; orr w11, w11, w9
                ; eor w9, w10, #1
                ; lsl w9, w9, #30
                ; orr w11, w11, w9
                ; mov w9, w11
            );
            writeback(ops);
        }
        S::Ror => unreachable!("dispatched above"),
    }
    dynasm!(ops ; .arch aarch64 ; =>end);
    true
}

/// Register-controlled rotate. Classes: zero (unchanged, C preserved),
/// nonzero multiple of 32 (unchanged, C = bit31), else rotate with
/// result-bit31 carry. Dynamic labels.
fn ror_reg_seq(ops: &mut A, rd: u8) -> bool {
    let mult32 = ops.new_dynamic_label();
    let copy = ops.new_dynamic_label();
    let end = ops.new_dynamic_label();
    dynasm!(ops
        ; .arch aarch64
        ; and w1, w1, #0xFF
        ; mov w10, w0
        ; cbz w1, =>copy
        ; and w1, w1, #31
        ; cbz w1, =>mult32
        ; ror w0, w0, w1
    );
    store(ops, rd);
    dynasm!(ops
        ; .arch aarch64
        ; lsr w1, w0, #31
        ; and w1, w1, #1
    );
    carry_in_w1(ops);
    dynasm!(ops ; .arch aarch64 ; b =>end);
    // Zero count: copy through with N/Z set, C preserved, V cleared.
    dynasm!(ops ; .arch aarch64 ; =>copy);
    store(ops, rd);
    dynasm!(ops ; .arch aarch64 ; tst w0, w0);
    merge_nz(ops);
    dynasm!(ops ; .arch aarch64 ; b =>end);
    // Nonzero multiple of 32: unchanged, C = bit31, V = 0.
    dynasm!(ops ; .arch aarch64 ; =>mult32);
    store(ops, rd);
    dynasm!(ops
        ; .arch aarch64
        ; lsr w1, w10, #31
        ; and w1, w1, #1
    );
    carry_in_w1(ops);
    dynasm!(ops ; .arch aarch64 ; =>end);
    true
}

/// Memory guard + fast path (P2-3, mirroring x64): input `W1` = guest
/// address; falls through with `X0` = host address; misses jump to
/// `side`. Clobbers `W2`–`W6`/`X9`/`X10`; preserves `W11`/`W12` and pins.
/// AArch64 always checks alignment for multi-byte accesses (the host may
/// fault on unaligned `ldr`/`str` regardless of the guest TRP bit — the
/// values still match via the interpreter on side exits).
fn mem_guard(ops: &mut A, size: u32, is_write: bool, side: dynasmrt::DynamicLabel) {
    // Cross-page check (size > 1 only).
    if size > 1 {
        let limit = 0x1000 - size;
        dynasm!(ops
            ; .arch aarch64
            ; and w6, w1, #0xFFF
            ; cmp w6, limit
            ; b.hi =>side
        );
        // Alignment check (always on AArch64; TRP modeled implicitly —
        // unaligned always exits, exactly like a trapped access would).
        let mask = size - 1;
        dynasm!(ops
            ; .arch aarch64
            ; tst w1, mask
            ; b.ne =>side
        );
    }
    let tlb = tlb_off();
    let priv_off = priv_off();
    let perm: u32 = if is_write { 2 } else { 1 };
    dynasm!(ops
        ; .arch aarch64
        ; mov w10, w1
        ; ldr x9, [sp, #48]
        ; ldr x9, [x9, tlb]
        ; lsr w2, w10, #12
        ; and x3, x2, #63
        ; lsl x3, x3, #4
        ; lsl w2, w2, #1
        ; ldr x10, [sp, #48]
        ; ldr w4, [x10, priv_off]
        ; orr w2, w2, w4
        ; ldr w4, [x9, x3]
        ; cmp w4, w2
        ; b.ne =>side
        ; add x3, x3, #4
        ; ldrb w4, [x9, x3]
        ; tst w4, perm
        ; b.eq =>side
        ; add x3, x3, #4
        ; ldr x5, [x9, x3]
        ; and w6, w1, #0xFFF
        ; add x0, x5, x6
    );
}

/// Side-exit handler: record the faulting guest address (still in `W1`
/// per the guard contract) and exit with `(MMIO, op_pc)`.
fn mem_side_exit(ops: &mut A, op_pc: u32) {
    use super::native::{emit_epilogue, EXIT_MMIO};
    let fault = fault_off();
    dynasm!(ops
        ; .arch aarch64
        ; ldr x9, [sp, #48]
        ; str w1, [x9, fault]
    );
    emit_epilogue(ops, EXIT_MMIO, op_pc);
}

/// One guarded memory site: address already in `W1`, then `fast`
/// performs the access (host in `X0`), optional writeback, and the epoch
/// bump for stores. Value/base spills live in `W11`/`W12` (guard-safe).
fn mem_site(ops: &mut A, size: u32, is_write: bool, op_pc: u32, fast: impl FnOnce(&mut A)) {
    let side = ops.new_dynamic_label();
    let end = ops.new_dynamic_label();
    // AArch64 folds the trap decision into the guard (always strict).
    mem_guard(ops, size, is_write, side);
    fast(ops);
    dynasm!(ops ; .arch aarch64 ; b =>end);
    dynasm!(ops ; .arch aarch64 ; =>side);
    mem_side_exit(ops, op_pc);
    dynasm!(ops ; .arch aarch64 ; =>end);
}

/// Shared access writer: fast load/store at host `X0`, optional
/// writeback of `W12` to `rn`, epoch bump for stores.
fn emit_access(ops: &mut A, size: u32, is_write: bool, sext: bool, rt: u8, wb_rn: Option<u8>) {
    if is_write {
        match size {
            1 => dynasm!(ops ; .arch aarch64 ; strb w11, [x0]),
            2 => dynasm!(ops ; .arch aarch64 ; strh w11, [x0]),
            _ => dynasm!(ops ; .arch aarch64 ; str w11, [x0]),
        }
        bump_epoch(ops);
    } else {
        match (size, sext) {
            (1, false) => dynasm!(ops ; .arch aarch64 ; ldrb w0, [x0]),
            (1, true) => dynasm!(ops ; .arch aarch64 ; ldrsb w0, [x0]),
            (2, false) => dynasm!(ops ; .arch aarch64 ; ldrh w0, [x0]),
            (2, true) => dynasm!(ops ; .arch aarch64 ; ldrsh w0, [x0]),
            _ => dynasm!(ops ; .arch aarch64 ; ldr w0, [x0]),
        }
        store(ops, rt);
    }
    if let Some(rn) = wb_rn {
        dynasm!(ops ; .arch aarch64 ; mov w0, w12);
        store(ops, rn);
    }
}

/// Bump the bus write generation after a fast store.
fn bump_epoch(ops: &mut A) {
    let epoch = epoch_off();
    dynasm!(ops
        ; .arch aarch64
        ; ldr x9, [sp, #48]
        ; ldr x9, [x9, epoch]
        ; ldr x10, [x9]
        ; add x10, x10, #1
        ; str x10, [x9]
    );
}

/// 16-bit single-transfer emitters (S1). Conventions: guest address ends
/// in `W1`; store values are preloaded to `W11`; writeback effects wait
/// in `W12`; the guard preserves all three.
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
    dynasm!(ops ; .arch aarch64 ; add w1, w1, w0);
    if is_write {
        load(ops, rt, true);
        dynasm!(ops ; .arch aarch64 ; mov w11, w0);
    }
    let _ = trap;
    mem_site(ops, size, is_write, pc, |ops| {
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
    let imm = imm as u32;
    load(ops, rn, false);
    dynasm!(ops ; .arch aarch64 ; add w1, w1, imm);
    if is_write {
        load(ops, rt, true);
        dynasm!(ops ; .arch aarch64 ; mov w11, w0);
    }
    let _ = trap;
    mem_site(ops, size, is_write, pc, |ops| {
        emit_access(ops, size, is_write, sext, rt, None);
    });
    true
}

fn emit_lssp(ops: &mut A, rt: u8, imm: usize, is_load: bool, pc: u32, trap: bool) -> bool {
    if rt == 15 {
        return false;
    }
    let imm = imm as u32;
    load(ops, 13, false);
    dynasm!(ops ; .arch aarch64 ; add w1, w1, imm);
    if !is_load {
        load(ops, rt, true);
        dynasm!(ops ; .arch aarch64 ; mov w11, w0);
    }
    let _ = trap;
    mem_site(ops, 4, !is_load, pc, |ops| {
        emit_access(ops, 4, !is_load, false, rt, None);
    });
    true
}

fn emit_ldrliteral(ops: &mut A, rt: u8, imm: usize, pc: u32, trap: bool) -> bool {
    if rt == 15 {
        return false;
    }
    let base = (pc.wrapping_add(4)) & !3;
    let addr = base.wrapping_add(imm as u32);
    let (lo, hi) = (addr & 0xFFFF, (addr >> 16) & 0xFFFF);
    dynasm!(ops
        ; .arch aarch64
        ; movz w1, lo
        ; movk w1, hi, lsl #16
    );
    let _ = trap;
    mem_site(ops, 4, false, pc, |ops| {
        emit_access(ops, 4, false, false, rt, None);
    });
    true
}

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
        let base = (pc.wrapping_add(4)) & !3;
        let (lo, hi) = (base & 0xFFFF, (base >> 16) & 0xFFFF);
        dynasm!(ops
            ; .arch aarch64
            ; movz w1, lo
            ; movk w1, hi, lsl #16
        );
    } else {
        if rm == Some(15) {
            return false;
        }
        load(ops, rn, false);
    }
    let mut wb = None;
    if let Some(r) = rm {
        load(ops, r, true);
        let shift = shift as u32;
        dynasm!(ops ; .arch aarch64 ; lsl w0, w0, shift);
        if index {
            dynasm!(ops ; .arch aarch64 ; add w1, w1, w0);
            if writeback {
                dynasm!(ops ; .arch aarch64 ; mov w12, w1);
                wb = Some(rn);
            }
        } else {
            dynasm!(ops ; .arch aarch64 ; mov w12, w1 ; add w12, w12, w0);
            wb = Some(rn);
        }
    } else if index {
        if add {
            dynasm!(ops ; .arch aarch64 ; add w1, w1, off);
        } else {
            dynasm!(ops ; .arch aarch64 ; sub w1, w1, off);
        }
        if writeback {
            dynasm!(ops ; .arch aarch64 ; mov w12, w1);
            wb = Some(rn);
        }
    } else {
        dynasm!(ops ; .arch aarch64 ; mov w12, w1);
        if add {
            dynasm!(ops ; .arch aarch64 ; add w12, w12, off);
        } else {
            dynasm!(ops ; .arch aarch64 ; sub w12, w12, off);
        }
        wb = Some(rn);
    }
    if is_write {
        load(ops, rt, true);
        dynasm!(ops ; .arch aarch64 ; mov w11, w0);
    }
    let _ = trap;
    mem_site(ops, size, is_write, pc, |ops| {
        emit_access(ops, size, is_write, sext, rt, wb);
    });
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
        _ => false,
    }
}
/// ALU-op dispatcher (16-bit `Alu` family; `itstate == 0` throughout
/// native blocks, so flags always update).
fn emit_alu(ops: &mut A, rd: u8, rm: u8, op: crate::cpu::decode16::AluOp) -> bool {
    use crate::cpu::decode16::AluOp as O;
    if rd == 15 || rm == 15 {
        return false;
    }
    match op {
        O::Ror | O::Lsl | O::Lsr | O::Asr => {
            // Register shifts read the count from Rm (not Rd).
            load(ops, rd, true);
            load(ops, rm, false);
            let kind = match op {
                O::Lsl => crate::cpu::decode16::Shift::Lsl,
                O::Lsr => crate::cpu::decode16::Shift::Lsr,
                O::Asr => crate::cpu::decode16::Shift::Asr,
                _ => crate::cpu::decode16::Shift::Ror,
            };
            return shift_reg_seq(ops, rd, kind);
        }
        O::Adc | O::Sbc => {
            load(ops, rd, true);
            load(ops, rm, false);
            // Seed C from the pinned word without `msr`: branchless is
            // impossible (only flags carry into `adcs`), so two dynamic
            // labels select a C-setting compare. Only C matters downstream
            // (`adcs`/`sbcs` overwrite the rest).
            let c0 = ops.new_dynamic_label();
            let c1 = ops.new_dynamic_label();
            dynasm!(ops
                ; .arch aarch64
                ; lsr w9, w23, #29
                ; and w9, w9, #1
                ; cbz w9, =>c0
                ; cmp w9, w9
                ; b =>c1
                ; =>c0
                ; subs w9, w9, #2
                ; =>c1
            );
            match op {
                O::Adc => dynasm!(ops ; .arch aarch64 ; adcs w0, w0, w1),
                _ => dynasm!(ops ; .arch aarch64 ; sbcs w0, w0, w1),
            }
            store(ops, rd);
            merge_nzcv(ops);
            return true;
        }
        _ => {}
    }
    load(ops, rd, true);
    match op {
        O::Tst | O::Cmp | O::Cmn => {
            load(ops, rm, false);
        }
        O::Mvn | O::Neg => {
            load(ops, rm, true);
        }
        _ => {
            load(ops, rm, false);
        }
    }
    match op {
        O::And => {
            dynasm!(ops ; .arch aarch64 ; and w0, w0, w1);
            store(ops, rd);
            dynasm!(ops ; .arch aarch64 ; tst w0, w0);
            merge_nz(ops);
        }
        O::Eor => {
            dynasm!(ops ; .arch aarch64 ; eor w0, w0, w1);
            store(ops, rd);
            dynasm!(ops ; .arch aarch64 ; tst w0, w0);
            merge_nz(ops);
        }
        O::Orr => {
            dynasm!(ops ; .arch aarch64 ; orr w0, w0, w1);
            store(ops, rd);
            dynasm!(ops ; .arch aarch64 ; tst w0, w0);
            merge_nz(ops);
        }
        O::Bic => {
            dynasm!(ops ; .arch aarch64 ; bic w0, w0, w1);
            store(ops, rd);
            dynasm!(ops ; .arch aarch64 ; tst w0, w0);
            merge_nz(ops);
        }
        O::Tst => {
            dynasm!(ops ; .arch aarch64 ; tst w0, w1);
            merge_nz(ops);
        }
        O::Cmp => {
            dynasm!(ops ; .arch aarch64 ; subs w9, w0, w1);
            merge_nzcv(ops);
        }
        O::Cmn => {
            dynasm!(ops ; .arch aarch64 ; adds w9, w0, w1);
            merge_nzcv(ops);
        }
        O::Mul => {
            dynasm!(ops ; .arch aarch64 ; mul w0, w0, w1);
            store(ops, rd);
            dynasm!(ops ; .arch aarch64 ; tst w0, w0);
            merge_nz(ops);
        }
        O::Mvn => {
            dynasm!(ops ; .arch aarch64 ; mvn w0, w1);
            store(ops, rd);
            dynasm!(ops ; .arch aarch64 ; tst w0, w0);
            merge_nz(ops);
        }
        O::Neg => {
            // RSB: `neg` (sub-from-zero alias) carries natively, result in
            // place for the store (no x86-style fixup needed).
            dynasm!(ops ; .arch aarch64 ; neg w0, w1);
            store(ops, rd);
            merge_nzcv(ops);
        }
        _ => unreachable!("shifts/adc/sbc handled above"),
    }
    true
}

fn emit_shift_imm(ops: &mut A, rd: u8, rm: u8, kind: ShiftImm, amount: u8) -> bool {
    if rd == 15 || rm == 15 {
        return false;
    }
    load(ops, rm, true);
    let n = u32::from(amount);
    if n == 0 {
        store(ops, rd);
        dynasm!(ops ; .arch aarch64 ; tst w0, w0);
        merge_nz(ops);
        return true;
    }
    // Save the original for carry (shifts set no flags here): C is
    // bit(32-n) for LSL, bit(n-1) otherwise. Only non-flag-setting
    // insns run between the shift and the NZ-establishing `tst`.
    dynasm!(ops ; .arch aarch64 ; mov w10, w0);
    match kind {
        ShiftImm::Lsl => {
            let c = 32 - n;
            dynasm!(ops ; .arch aarch64 ; lsl w0, w0, n);
            store(ops, rd);
            dynasm!(ops
                ; .arch aarch64
                ; lsr w1, w10, c
                ; and w1, w1, #1
            );
        }
        ShiftImm::Lsr => {
            let c = n - 1;
            dynasm!(ops ; .arch aarch64 ; lsr w0, w0, n);
            store(ops, rd);
            dynasm!(ops
                ; .arch aarch64
                ; lsr w1, w10, c
                ; and w1, w1, #1
            );
        }
        ShiftImm::Asr => {
            let c = n - 1;
            dynasm!(ops ; .arch aarch64 ; asr w0, w0, n);
            store(ops, rd);
            dynasm!(ops
                ; .arch aarch64
                ; lsr w1, w10, c
                ; and w1, w1, #1
            );
        }
    }
    carry_in_w1(ops);
    true
}

/// Finish a shift with an explicitly computed carry in `w1`: N/Z from a
/// fresh `tst` of the result, C from `w1`, V cleared. (`tst` N/Z feed the
/// `cset`s; the carry never touches flags.)
fn carry_in_w1(ops: &mut A) {
    dynasm!(ops
        ; .arch aarch64
        ; tst w0, w0
        ; cset w9, mi
        ; cset w10, eq
        ; lsl w9, w9, #31
        ; lsl w10, w10, #30
        ; lsl w11, w1, #29
        ; orr w9, w9, w10
        ; orr w9, w9, w11
    );
    writeback(ops);
}
