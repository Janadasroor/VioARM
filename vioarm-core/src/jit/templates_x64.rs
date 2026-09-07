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
    super::native::emit_static_edge(ops, super::native::EXIT_FALL, end_pc, slot);
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
pub fn emit_op(ops: &mut A, op: &TbOp, pc: u32, unalign_trap: bool) -> bool {
    match *op {
        TbOp::V16(inst) => emit16(ops, &inst, pc, unalign_trap),
        TbOp::V32(inst) => emit32(ops, &inst, pc, unalign_trap),
    }
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
    // `unalign_trap` only affects memory sites; branches take it to keep
    // the signature uniform (terminators never touch memory).
    let _ = unalign_trap;
    match *op {
        TbOp::V16(inst) => emit16_term(ops, &inst, pc, tape),
        TbOp::V32(inst) => emit32_term(ops, &inst, pc, tape),
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

fn emit16(ops: &mut A, inst: &crate::cpu::decode16::Inst16, pc: u32, unalign_trap: bool) -> bool {
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
        // --- Loads/stores (P2-3 S1).
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
            // Taken edge first (inverted jump): slot order matches
            // `exit_shape` (taken, fall).
            let fall = ops.new_dynamic_label();
            if nonzero {
                dynasm!(ops ; .arch x64 ; test eax, eax ; jz =>fall);
            } else {
                dynasm!(ops ; .arch x64 ; test eax, eax ; jnz =>fall);
            }
            emit_static_edge(ops, EXIT_BRANCH, b_target(pc, off), tape.take());
            dynasm!(ops ; .arch x64 ; =>fall);
            emit_static_edge(ops, EXIT_BRANCH, pc.wrapping_add(2), tape.take());
            true
        }
        I::Bx { rm } => emit_bx(ops, rm, pc, false, tape),
        I::Blx { rm } => emit_bx(ops, rm, pc, true, tape),
        _ => false,
    }
}

/// 32-bit terminators (P2-4): `BL` links `LR` natively, wide branches
/// chain like their 16-bit cousins (32-bit forms are `pc + 4`-relative).
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
            let lr = pc.wrapping_add(4) | 1;
            dynasm!(ops ; .arch x64 ; mov eax, lr as i32);
            store_lr(ops);
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
    emit_static_edge(ops, EXIT_BRANCH, taken, tape.take());
    dynasm!(ops ; .arch x64 ; =>fall_lbl);
    emit_static_edge(ops, EXIT_BRANCH, fall, tape.take());
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

/// Memory guard + fast path (P2-3). Input `ECX` = guest address.
/// Falls through with `RAX` = host address; misses jump to `side`.
/// Clobbers `EAX`/`EDX`/`ESI`/`EDI`; preserves `R8D`–`R11D` and pins.
/// `trap` bakes CCR.UNALIGN_TRP (x64 needs no alignment check otherwise:
/// the host tolerates in-page unaligned accesses natively).
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
        ; jne =>side
        ; test BYTE [rsi + rdx + 4], perm
        ; jz =>side
        ; mov rdx, [rsi + rdx + 8]
        ; mov eax, r10d
        ; and eax, 0xFFF
        ; add rax, rdx
    );
}

/// Side-exit handler for a memory miss: record the faulting guest address
/// (saved in `R10D` by the guard contract) and exit with `(MMIO, op_pc)`
/// for driver handling (interp re-execution + TLB fill attempt).
fn mem_side_exit(ops: &mut A, op_pc: u32) {
    use super::native::{emit_epilogue, EXIT_MMIO};
    let fault = fault_off();
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
    fast(ops);
    dynasm!(ops ; .arch x64 ; jmp =>end);
    dynasm!(ops ; .arch x64 ; =>side);
    mem_side_exit(ops, op_pc);
    dynasm!(ops ; .arch x64 ; =>end);
}

/// Bump the bus write generation after a fast store (keeps the
/// entry-gate invariant absolute: every bus write advances it).
fn bump_epoch(ops: &mut A) {
    let epoch = epoch_off();
    dynasm!(ops
        ; .arch x64
        ; mov rsi, [rsp]
        ; mov rsi, [rsi + epoch]
        ; inc QWORD [rsi]
    );
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
/// writeback of `R8D` to `rn`, epoch bump for stores. `rt` names the
/// transfer register (loads) or is unused (stores read `R9D`).
fn emit_access(ops: &mut A, size: u32, is_write: bool, sext: bool, rt: u8, wb_rn: Option<u8>) {
    if is_write {
        match size {
            1 => dynasm!(ops ; .arch x64 ; mov [rax], r9b),
            2 => dynasm!(ops ; .arch x64 ; mov [rax], r9w),
            _ => dynasm!(ops ; .arch x64 ; mov [rax], r9d),
        }
        bump_epoch(ops);
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
