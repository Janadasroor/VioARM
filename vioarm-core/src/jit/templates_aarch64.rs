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

fn regs_off() -> u32 {
    core::mem::offset_of!(crate::jit::NativeCtx, regs) as u32
}

fn reg_off(i: usize) -> u32 {
    (core::mem::offset_of!(Registers, r) + i * 4) as u32
}

/// Load guest `r[guest]` into `w0` (`first`) or `w1`.
fn load(ops: &mut A, guest: u8, first: bool) {
    let off = reg_off(guest as usize);
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
    let off = reg_off(guest as usize);
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
pub fn emit_op(ops: &mut A, op: &TbOp, pc: u32) -> bool {
    match *op {
        TbOp::V16(inst) => emit16(ops, &inst, pc),
        TbOp::V32(_) => false,
    }
}

/// Assemble prologue + one template + `FALL` epilogue.
pub fn assemble_single(op: &TbOp, pc: u32) -> Result<super::native::NativeBlock, super::EmitError> {
    use super::native::{emit_epilogue, emit_prologue};
    use dynasmrt::AssemblyOffset;

    let mut ops = Assembler::new()?;
    emit_prologue(&mut ops);
    if !emit_op(&mut ops, op, pc) {
        return Err(super::EmitError::Untemplated);
    }
    emit_epilogue(&mut ops, super::native::EXIT_FALL, 0);
    ops.finalize()
        .map(|buf| super::native::NativeBlock::new(buf, AssemblyOffset(0)))
        .map_err(|_| super::EmitError::Finalize)
}

fn emit16(ops: &mut A, inst: &crate::cpu::decode16::Inst16, pc: u32) -> bool {
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
        // --- Terminators (P2-2 minimal set, mirroring x64).
        I::Branch { off } => {
            super::native::emit_epilogue(ops, super::native::EXIT_BRANCH, b_target(pc, off));
            true
        }
        I::BranchCond { cond, off } => {
            if cond > 13 {
                return false;
            }
            emit_cond_branch(ops, cond, b_target(pc, off), pc.wrapping_add(2));
            true
        }
        I::CbzCbnz { rn, off, nonzero } => {
            if rn == 15 {
                return false;
            }
            load(ops, rn, true);
            let taken = ops.new_dynamic_label();
            if nonzero {
                dynasm!(ops ; .arch aarch64 ; cbnz w0, =>taken);
            } else {
                dynasm!(ops ; .arch aarch64 ; cbz w0, =>taken);
            }
            super::native::emit_epilogue(ops, super::native::EXIT_BRANCH, pc.wrapping_add(2));
            dynasm!(ops ; .arch aarch64 ; =>taken);
            super::native::emit_epilogue(ops, super::native::EXIT_BRANCH, b_target(pc, off));
            true
        }
        _ => false,
    }
}

/// 16-bit control flow is `pc + 4`-relative.
fn b_target(pc: u32, off: i32) -> u32 {
    pc.wrapping_add(4).wrapping_add_signed(off)
}

/// Conditional branch: evaluate `cond` (0-13) against the pinned NZCV
/// nibble, select target/fallthrough into `w0`, dynamic exit. Nibble
/// layout: N = 8, Z = 4, C = 2, V = 1. Only flag-clean insns run between
/// the nibble tests and their `cset`s.
fn emit_cond_branch(ops: &mut A, cond: u8, target: u32, fall: u32) {
    use super::native::{emit_epilogue_dynamic, EXIT_BRANCH};
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
    let (t_lo, t_hi) = (target & 0xFFFF, (target >> 16) & 0xFFFF);
    let (f_lo, f_hi) = (fall & 0xFFFF, (fall >> 16) & 0xFFFF);
    dynasm!(ops
        ; .arch aarch64
        ; movz w0, f_lo
        ; movk w0, f_hi, lsl #16
        ; movz w1, t_lo
        ; movk w1, t_hi, lsl #16
        ; cmp w9, #0
        ; csel w0, w1, w0, ne
    );
    emit_epilogue_dynamic(ops, EXIT_BRANCH);
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
