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

/// Assemble prologue + one template + `FALL` epilogue (harness + block
/// emission share this shape).
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

fn regs_off() -> i32 {
    core::mem::offset_of!(crate::jit::NativeCtx, regs) as i32
}

fn reg_off(i: usize) -> i32 {
    (core::mem::offset_of!(Registers, r) + i * 4) as i32
}

/// Load guest `r[guest]` into `eax` (`true`) or `ecx` (`false`).
fn load(ops: &mut A, guest: u8, into_eax: bool) {
    let off = reg_off(guest as usize);
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
    let off = reg_off(guest as usize);
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
/// Terminator arms emit their epilogue directly (P2-2: exits via driver;
/// direct chaining is P2-4).
pub fn emit_op(ops: &mut A, op: &TbOp, pc: u32) -> bool {
    match *op {
        TbOp::V16(inst) => emit16(ops, &inst, pc),
        TbOp::V32(_) => false,
    }
}

/// 16-bit control flow is `pc + 4`-relative.
fn b_target(pc: u32, off: i32) -> u32 {
    pc.wrapping_add(4).wrapping_add_signed(off)
}

fn lr_off() -> i32 {
    core::mem::offset_of!(Registers, lr) as i32
}

fn emit16(ops: &mut A, inst: &crate::cpu::decode16::Inst16, pc: u32) -> bool {
    use super::native::{emit_epilogue, EXIT_BRANCH};
    use crate::cpu::decode16::Inst16 as I;
    match *inst {
        I::MovImm { rd, imm } => {
            if rd == 15 {
                return false;
            }
            let imm = i32::from(imm);
            dynasm!(ops ; .arch x64 ; mov eax, imm);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
            merge_nz(ops);
            true
        }
        I::AddReg { rd, rn, rm } => alu_rr(ops, rd, rn, rm, AluKind::Add),
        I::SubReg { rd, rn, rm } => alu_rr(ops, rd, rn, rm, AluKind::Sub),
        I::AddImm3 { rd, rn, imm } => alu_ri(ops, rd, rn, i32::from(imm), AluKind::Add),
        I::SubImm3 { rd, rn, imm } => alu_ri(ops, rd, rn, i32::from(imm), AluKind::Sub),
        I::AddImm8 { rd, imm } => alu_ri(ops, rd, rd, i32::from(imm), AluKind::Add),
        I::SubImm8 { rd, imm } => alu_ri(ops, rd, rd, i32::from(imm), AluKind::Sub),
        I::CmpImm { rn, imm } => cmp_ri(ops, rn, i32::from(imm)),
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
            dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc ; seto dl ; lahf);
            merge_nzcv(ops);
            true
        }
        // --- Terminators (P2-2 minimal set): static targets exit via the
        // static epilogue; conditions select at runtime into edx for the
        // dynamic epilogue. BlxImm/registers/table branches stay
        // interpreted (P2-4).
        I::Branch { off } => {
            emit_epilogue(ops, EXIT_BRANCH, b_target(pc, off));
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
                dynasm!(ops ; .arch x64 ; test eax, eax ; jnz =>taken);
            } else {
                dynasm!(ops ; .arch x64 ; test eax, eax ; jz =>taken);
            }
            emit_epilogue(ops, EXIT_BRANCH, pc.wrapping_add(2));
            dynasm!(ops ; .arch x64 ; =>taken);
            emit_epilogue(ops, EXIT_BRANCH, b_target(pc, off));
            true
        }
        _ => false,
    }
}

/// Conditional branch: evaluate `cond` (0-13) against the pinned NZCV
/// nibble, select target/fallthrough into `edx`, dynamic exit. Nibble
/// layout after `shr ecx, 28`: N = 8, Z = 4, C = 2, V = 1. `edx` starts
/// zeroed so every path yields a clean 0/1 taken bit; the select then
/// overwrites it wholesale.
fn emit_cond_branch(ops: &mut A, cond: u8, target: u32, fall: u32) {
    use super::native::{emit_epilogue_dynamic, EXIT_BRANCH};
    let target = target as i32;
    let fall = fall as i32;
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
        ; mov edx, fall
        ; mov ecx, target
        // Test the taken BYTE only: multi-condition paths leave their
        // scratch in dh, which must not vote (`sil` needs no REX
        // conflict here — `ah` is long dead by exit).
        ; test sil, sil
        ; cmovnz edx, ecx
    );
    emit_epilogue_dynamic(ops, EXIT_BRANCH);
}

#[derive(Clone, Copy)]
enum AluKind {
    Add,
    Sub,
}

/// `Rd = Rn +/- Rm` with full NZCV.
fn alu_rr(ops: &mut A, rd: u8, rn: u8, rm: u8, kind: AluKind) -> bool {
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
    dynasm!(ops ; .arch x64 ; seto dl ; lahf);
    merge_nzcv(ops);
    true
}

/// `Rd = Rn +/- imm` with full NZCV.
fn alu_ri(ops: &mut A, rd: u8, rn: u8, imm: i32, kind: AluKind) -> bool {
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
    dynasm!(ops ; .arch x64 ; seto dl ; lahf);
    merge_nzcv(ops);
    true
}

/// `CMP Rn, #imm`: compare and discard, full NZCV.
fn cmp_ri(ops: &mut A, rn: u8, imm: i32) -> bool {
    if rn == 15 {
        return false;
    }
    load(ops, rn, true);
    dynasm!(ops ; .arch x64 ; mov ecx, imm);
    dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc ; seto dl ; lahf);
    merge_nzcv(ops);
    true
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
fn shift_reg_seq(ops: &mut A, rd: u8, kind: crate::cpu::decode16::Shift) -> bool {
    use crate::cpu::decode16::Shift as S;
    if kind == S::Ror {
        return ror_reg_seq(ops, rd);
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
    merge_nzcv(ops);
    dynasm!(ops ; .arch x64 ; jmp =>end);
    // Zero count: copy through with N/Z set, C preserved, V cleared
    // (matches `write_shift_flags(res, None)` — only C is conditional).
    dynasm!(ops ; .arch x64 ; =>copy);
    store(ops, rd);
    dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
    merge_nz(ops);
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
            merge_reg(ops);
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
            merge_reg(ops);
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
            merge_reg(ops);
        }
        S::Ror => unreachable!("dispatched above"),
    }
    dynasm!(ops ; .arch x64 ; =>end);
    true
}

/// Register-controlled rotate. Entry: `eax` = value, `ecx` = raw count.
/// Classes: zero (unchanged, C preserved), nonzero multiple of 32
/// (unchanged, C = bit31), else rotate with CF carry. Dynamic labels.
fn ror_reg_seq(ops: &mut A, rd: u8) -> bool {
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
    merge_nzcv(ops);
    dynasm!(ops ; .arch x64 ; jmp =>end);
    // Zero count: copy through with N/Z set, C preserved, V cleared.
    dynasm!(ops ; .arch x64 ; =>copy);
    store(ops, rd);
    dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
    merge_nz(ops);
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
    merge_reg(ops);
    dynasm!(ops ; .arch x64 ; =>end);
    true
}

/// ALU-op dispatcher (16-bit `Alu` family; `set_flags` is always true in
/// native blocks since `itstate == 0` throughout).
fn emit_alu(ops: &mut A, rd: u8, rm: u8, op: crate::cpu::decode16::AluOp) -> bool {
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
            return shift_reg_seq(ops, rd, kind);
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
            merge_nzcv(ops);
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
            merge_nz(ops);
        }
        "eor" => {
            dynasm!(ops ; .arch x64 ; xor eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; lahf);
            merge_nz(ops);
        }
        "orr" => {
            dynasm!(ops ; .arch x64 ; or eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; lahf);
            merge_nz(ops);
        }
        "bic-not" => {
            dynasm!(ops ; .arch x64 ; not ecx ; and eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; lahf);
            merge_nz(ops);
        }
        "test" => {
            dynasm!(ops ; .arch x64 ; test eax, ecx ; lahf);
            merge_nz(ops);
        }
        "cmp" => {
            dynasm!(ops ; .arch x64 ; sub eax, ecx ; cmc ; seto dl ; lahf);
            merge_nzcv(ops);
        }
        "add-nostore" => {
            dynasm!(ops ; .arch x64 ; add eax, ecx ; seto dl ; lahf);
            merge_nzcv(ops);
        }
        "mul" => {
            // 2-operand IMUL leaves SF/ZF undefined: re-test for N/Z.
            dynasm!(ops ; .arch x64 ; imul eax, ecx);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
            merge_nz(ops);
        }
        "mvn" => {
            dynasm!(ops ; .arch x64 ; not eax);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
            merge_nz(ops);
        }
        "neg" => {
            // RSB: res = 0 - Rm; x86 NEG leaves inverted carry vs ARM.
            // Store BEFORE flag extraction: `lahf` overwrites ah, i.e.
            // result bits 15:8 (caught by differential trial 0).
            dynasm!(ops ; .arch x64 ; neg eax ; cmc);
            store(ops, rd);
            dynasm!(ops ; .arch x64 ; seto dl ; lahf);
            merge_nzcv(ops);
        }
        _ => unreachable!("covered above"),
    }
    true
}

/// Shift-immediate kinds after the interpreter's `shift_imm` truth table.
fn emit_shift_imm(ops: &mut A, rd: u8, rm: u8, kind: ShiftImm, amount: u8) -> bool {
    if rd == 15 || rm == 15 {
        return false;
    }
    load(ops, rm, true);
    let n = amount as i8;
    if n == 0 {
        // Copy with C preserved, V cleared (matches `shift_imm` + `set_nz`).
        store(ops, rd);
        dynasm!(ops ; .arch x64 ; test eax, eax ; lahf);
        merge_nz(ops);
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
    merge_nzcv(ops);
    true
}
