//! A tiny byte-level Thumb/Thumb-2 assembler for building conformance test
//! images.
//!
//! Every encoder in this file is the inverse of a decoder whose encoding is
//! verified against `llvm-mc`/`clang` in `vioarm-core`'s own test suite; the
//! QEMU differential harness is the second, cross-checking oracle: a wrong
//! encoding here shows up as a step-indexed divergence that the diff reports.

use std::collections::HashMap;

/// Where the image's code starts (the reset vector points here). Must be a
/// multiple of 4 so Thumb-2 instructions and the word-aligned vector table
/// stay aligned.
pub const CODE_BASE: u32 = 0x100;

/// The initial stack pointer written into the vector table.
pub const SP: u32 = 0x2000_2000;

#[derive(Clone)]
enum Item {
    Label(String),
    H2(u16),
    H4(u32),
    /// Unconditional `B.W` (always the 4-byte wide form).
    B { target: String },
    Bl { target: String },
    /// Narrow conditional branch `B<c> imm8`, ±256 bytes.
    Bc { cc: u8, target: String },
    /// Wide conditional branch `B<c>.W imm24`, ±1 MB.
    BcWide { cc: u8, target: String },
    /// Narrow `CBZ`/`CBNZ`, forward only.
    Cbz { rn: u8, nonzero: bool, target: String },
    /// `LDR Rt, [PC, #imm]` against the literal at `target`.
    LdrLit { rt: u8, target: String },
    /// `LDRD Rt, Rt2, [PC, #imm]` against the literal pool at `target`.
    LdrdLit { rt: u8, rt2: u8, target: String },
    /// Pad with `NOP` (2 bytes each) up to a 4-byte boundary.
    PadTo4,
    DcU32(u32),
}

fn size_of(item: &Item) -> u32 {
    match item {
        Item::Label(_) => 0,
        Item::H2(_) => 2,
        Item::H4(_) => 4,
        Item::B { .. } | Item::Bl { .. } | Item::BcWide { .. } | Item::LdrdLit { .. } => 4,
        Item::Bc { .. } | Item::Cbz { .. } | Item::LdrLit { .. } => 2,
        Item::PadTo4 | Item::DcU32(_) => 4,
    }
}

/// The assembled image: raw bytes plus the resolved label table.
pub struct Image {
    pub bytes: Vec<u8>,
    pub labels: HashMap<String, u32>,
}

impl Image {
    /// The absolute address of a code label.
    #[must_use]
    pub fn addr(&self, name: &str) -> Option<u32> {
        self.labels.get(name).copied()
    }
}

/// A program under construction.
#[derive(Clone, Default)]
pub struct Asm {
    items: Vec<Item>,
    /// Exception vector-table entries (exception number, handler label).
    /// `0` and `1` (MSP / reset) are always present; this covers the rest.
    vectors: Vec<(u32, String)>,
}

impl Asm {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    fn emit(&mut self, item: Item) {
        self.items.push(item);
    }

    // ------------------------------ layout ------------------------------

    /// Mark the current position with `name`.
    pub fn label(&mut self, name: impl Into<String>) {
        self.emit(Item::Label(name.into()));
    }

    /// Pad with `NOP`s up to the next 4-byte boundary.
    pub fn align4(&mut self) {
        self.emit(Item::PadTo4);
    }

    /// Install `handler` as the vector-table entry for exception number `exc`.
    /// The handler address is written with the Thumb bit set, exactly as the
    /// reset vector is.
    pub fn vec(&mut self, exc: u32, handler: impl Into<String>) {
        self.vectors.push((exc, handler.into()));
    }

    /// Emit a raw 32-bit data word (for literal pools).
    pub fn dc_u32(&mut self, v: u32) {
        self.emit(Item::DcU32(v));
    }

    /// Emit a raw 16-bit instruction halfword (for anything not yet covered
    /// by a named encoder).
    pub fn raw(&mut self, hw: u16) {
        self.emit(Item::H2(hw));
    }

    /// Emit a raw 32-bit Thumb-2 instruction (`hi` in the low half).
    pub fn raw32(&mut self, w: u32) {
        self.emit(Item::H4(w));
    }

    /// Run `n` `NOP`s.
    pub fn pad_nops(&mut self, n: usize) {
        for _ in 0..n {
            self.raw(0xBF00);
        }
    }

    // ------------------------------- moves ------------------------------

    pub fn movs_imm8(&mut self, rd: u8, imm: u8) {
        self.raw(0x2000 | (u16::from(rd) & 7) << 8 | u16::from(imm));
    }

    pub fn movw(&mut self, rd: u8, imm: u16) {
        self.movwide(0xF240, rd, imm);
    }

    pub fn movt(&mut self, rd: u8, imm: u16) {
        self.movwide(0xF2C0, rd, imm);
    }

    fn movwide(&mut self, base: u16, rd: u8, imm: u16) {
        let hi = base | (u16::from((imm >> 11) & 1)) << 10 | (imm >> 12);
        let lo = (u16::from((imm >> 8) & 7)) << 12 | (u16::from(rd) << 8) | (imm & 0xFF);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// Load any 32-bit constant into `rd` with `MOVW`/`MOVT` (pairs).
    pub fn mov_imm32(&mut self, rd: u8, imm: u32) {
        self.movw(rd, (imm & 0xFFFF) as u16);
        let hi = (imm >> 16) as u16;
        if hi != 0 {
            self.movt(rd, hi);
        }
    }

    /// `MOV Rd, Rm` (special form; handles the high registers, no flags).
    pub fn mov_reg(&mut self, rd: u8, rm: u8) {
        let h = u16::from(rd >> 3);
        self.raw(0x4600 | h << 7 | (u16::from(rm) & 0xF) << 3 | u16::from(rd) & 7);
    }

    // ---------------------------- add / subtract ----------------------------

    pub fn adds_reg(&mut self, rd: u8, rn: u8, rm: u8) {
        self.raw(0x1800 | (u16::from(rm) & 7) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn subs_reg(&mut self, rd: u8, rn: u8, rm: u8) {
        self.raw(0x1A00 | (u16::from(rm) & 7) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn adds_imm3(&mut self, rd: u8, rn: u8, imm: u8) {
        self.raw(0x1C00 | (u16::from(imm) & 7) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn subs_imm3(&mut self, rd: u8, rn: u8, imm: u8) {
        self.raw(0x1E00 | (u16::from(imm) & 7) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn adds_imm8(&mut self, rd: u8, imm: u8) {
        self.raw(0x3000 | (u16::from(rd) & 7) << 8 | u16::from(imm));
    }

    pub fn subs_imm8(&mut self, rd: u8, imm: u8) {
        self.raw(0x3800 | (u16::from(rd) & 7) << 8 | u16::from(imm));
    }

    pub fn cmp_imm8(&mut self, rn: u8, imm: u8) {
        self.raw(0x2800 | (u16::from(rn) & 7) << 8 | u16::from(imm));
    }

    /// `ADD Rd, Rm` high-register add (no flags).
    pub fn add_high(&mut self, rd: u8, rm: u8) {
        let h = u16::from(rd >> 3);
        self.raw(0x4400 | h << 7 | (u16::from(rm) & 0xF) << 3 | u16::from(rd) & 7);
    }

    /// `CMP Rd, Rm` high-register compare (no write).
    pub fn cmp_high(&mut self, rn: u8, rm: u8) {
        let h = u16::from(rn >> 3);
        self.raw(0x4500 | h << 7 | (u16::from(rm) & 0xF) << 3 | u16::from(rn) & 7);
    }

    /// `ADDW Rd, Rn, #imm12` (raw 12-bit immediate).
    pub fn addw(&mut self, rd: u8, rn: u8, imm12: u16) {
        self.dp_plain(0, rd, rn, imm12, false);
    }

    /// `SUBW Rd, Rn, #imm12`.
    pub fn subw(&mut self, rd: u8, rn: u8, imm12: u16) {
        self.dp_plain(0b101, rd, rn, imm12, false);
    }

    fn dp_plain(&mut self, op: u16, rd: u8, rn: u8, imm12: u16, set_flags: bool) {
        let i = u16::from((imm12 >> 11) & 1);
        let imm3 = u16::from((imm12 >> 8) & 7);
        let imm8 = imm12 & 0xFF;
        let hi = 0xF200 | i << 10 | op << 5 | (u16::from(set_flags)) << 4 | u16::from(rn);
        let lo = imm3 << 12 | u16::from(rd) << 8 | imm8;
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// Thumb-2 data-processing, register operand with `LSL #amount` shift.
    #[allow(clippy::too_many_arguments)]
    fn dp_reg(
        &mut self,
        add_family: bool,
        op: u16,
        rd: u8,
        rn: u8,
        rm: u8,
        shift: u16,
        amount: u8,
        set_flags: bool,
    ) {
        let hi = 0xEA00 | u16::from(add_family) << 8 | op << 5 | u16::from(set_flags) << 4 | u16::from(rn);
        let imm3 = u16::from(amount >> 2);
        let imm2 = u16::from(amount & 3);
        let lo = imm3 << 12 | u16::from(rd) << 8 | imm2 << 6 | (shift & 3) << 4 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `ADDS Rd, Rn, Rm` (wide, sets flags).
    pub fn adds_wide(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dp_reg(true, 0, rd, rn, rm, 0, 0, true);
    }

    /// `SUBS Rd, Rn, Rm` (wide, sets flags).
    pub fn subs_wide(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dp_reg(true, 5, rd, rn, rm, 0, 0, true);
    }

    /// `ANDS Rd, Rn, Rm` (wide, sets flags).
    pub fn ands_wide(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dp_reg(false, 0, rd, rn, rm, 0, 0, true);
    }

    /// `ORR Rd, Rn, Rm` (wide).
    pub fn orr_wide(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dp_reg(false, 2, rd, rn, rm, 0, 0, false);
    }

    /// `EOR Rd, Rn, Rm` (wide).
    pub fn eor_wide(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dp_reg(false, 4, rd, rn, rm, 0, 0, false);
    }

    /// `BIC Rd, Rn, Rm` (wide).
    pub fn bic_wide(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dp_reg(false, 1, rd, rn, rm, 0, 0, false);
    }

    /// `LSLS Rd, Rm, #amount` (wide, sets flags).
    pub fn lsls_wide(&mut self, rd: u8, rn: u8, rm: u8, amount: u8) {
        self.dp_reg(false, 0, rd, rn, rm, 0, amount, true);
    }

    /// `ASR Rd, Rm, #amount` (wide).
    pub fn asr_wide(&mut self, rd: u8, rn: u8, rm: u8, amount: u8) {
        self.dp_reg(false, 0, rd, rn, rm, 2, amount, false);
    }

    /// `MVN Rd, Rm` (wide).
    pub fn mvn_wide(&mut self, rd: u8, rm: u8) {
        self.dp_reg(false, 3, rd, 15, rm, 0, 0, false);
    }

    /// `MOV Rd, Rm` (wide, flags may be clear).
    pub fn mov_wide(&mut self, rd: u8, rm: u8) {
        self.dp_reg(false, 2, rd, 15, rm, 0, 0, false);
    }

    // ------------------------------- shifts -------------------------------

    pub fn lsls_imm(&mut self, rd: u8, rm: u8, amount: u8) {
        self.raw(0x0000 | (u16::from(amount) & 0x1F) << 6 | (u16::from(rm) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn lsrs_imm(&mut self, rd: u8, rm: u8, amount: u8) {
        self.raw(0x0800 | (u16::from(amount) & 0x1F) << 6 | (u16::from(rm) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn asrs_imm(&mut self, rd: u8, rm: u8, amount: u8) {
        self.raw(0x1000 | (u16::from(amount) & 0x1F) << 6 | (u16::from(rm) & 7) << 3 | u16::from(rd) & 7);
    }

    // ---------------------------- ALU (0x4000) ----------------------------

    /// `ANDS`..`MVNS` from the 16-bit ALU space; `opcode` is the 4-bit field.
    fn alu(&mut self, opcode: u8, rd: u8, rm: u8) {
        self.raw(0x4000 | (u16::from(opcode) & 0xF) << 6 | (u16::from(rm) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn ands(&mut self, rd: u8, rm: u8) {
        self.alu(0, rd, rm);
    }
    pub fn eors(&mut self, rd: u8, rm: u8) {
        self.alu(1, rd, rm);
    }
    pub fn lsls_reg(&mut self, rd: u8, rm: u8) {
        self.alu(2, rd, rm);
    }
    pub fn lsrs_reg(&mut self, rd: u8, rm: u8) {
        self.alu(3, rd, rm);
    }
    pub fn asrs_reg(&mut self, rd: u8, rm: u8) {
        self.alu(4, rd, rm);
    }
    pub fn adcs(&mut self, rd: u8, rm: u8) {
        self.alu(5, rd, rm);
    }
    pub fn sbcs(&mut self, rd: u8, rm: u8) {
        self.alu(6, rd, rm);
    }
    pub fn rors_reg(&mut self, rd: u8, rm: u8) {
        self.alu(7, rd, rm);
    }
    pub fn tst(&mut self, rd: u8, rm: u8) {
        self.alu(8, rd, rm);
    }
    pub fn negs(&mut self, rd: u8, rm: u8) {
        self.alu(9, rd, rm);
    }
    pub fn cmp_reg(&mut self, rd: u8, rm: u8) {
        self.alu(10, rd, rm);
    }
    pub fn cmn(&mut self, rd: u8, rm: u8) {
        self.alu(11, rd, rm);
    }
    pub fn orrs(&mut self, rd: u8, rm: u8) {
        self.alu(12, rd, rm);
    }
    pub fn muls(&mut self, rd: u8, rm: u8) {
        self.alu(13, rd, rm);
    }
    pub fn bics(&mut self, rd: u8, rm: u8) {
        self.alu(14, rd, rm);
    }
    pub fn mvns(&mut self, rd: u8, rm: u8) {
        self.alu(15, rd, rm);
    }

    // ------------------------------- branch -------------------------------

    pub fn b(&mut self, target: impl Into<String>) {
        self.emit(Item::B {
            target: target.into(),
        });
    }

    pub fn bc(&mut self, cc: u8, target: impl Into<String>) {
        self.emit(Item::Bc {
            cc,
            target: target.into(),
        });
    }

    /// Wide conditional branch `B<c>.W` (T3), ±1 MB.
    pub fn bc_wide(&mut self, cc: u8, target: impl Into<String>) {
        self.emit(Item::BcWide {
            cc,
            target: target.into(),
        });
    }

    /// `TBB [Rn, Rm]`: a byte-indexed table branch. The table is read from
    /// `Align(pc + 4, 4) + Rm`; `pc` here is the address of the `TBB` itself.
    pub fn tb(&mut self, rn: u8, rm: u8) {
        self.raw32((0xE8D0 | u16::from(rn)) as u32 | ((0xF000 | u16::from(rm)) as u32) << 16);
    }

    /// `TBH [Rn, Rm]`: a halfword-indexed table branch.
    pub fn tbh(&mut self, rn: u8, rm: u8) {
        self.raw32((0xE8D0 | u16::from(rn)) as u32 | ((0xF010 | u16::from(rm)) as u32) << 16);
    }

    pub fn bl(&mut self, target: impl Into<String>) {
        self.emit(Item::Bl {
            target: target.into(),
        });
    }

    pub fn cbz(&mut self, rn: u8, target: impl Into<String>) {
        self.emit(Item::Cbz {
            rn,
            nonzero: false,
            target: target.into(),
        });
    }

    pub fn cbnz(&mut self, rn: u8, target: impl Into<String>) {
        self.emit(Item::Cbz {
            rn,
            nonzero: true,
            target: target.into(),
        });
    }

    pub fn bx(&mut self, rm: u8) {
        self.raw(0x4700 | u16::from(rm) << 3);
    }

    pub fn blx(&mut self, rm: u8) {
        self.raw(0x4780 | u16::from(rm) << 3);
    }

    /// `NOP` (the 16-bit hint, `0xBF00`).
    pub fn nop(&mut self) {
        self.raw(0xBF00);
    }

    // ------------------------------ IT block ------------------------------

    pub fn it(&mut self, firstcond: u8, mask: u8) {
        self.raw(0xBF00 | u16::from(firstcond) << 4 | u16::from(mask));
    }

    // ---------------------------- load / store ----------------------------

    pub fn str_imm(&mut self, rt: u8, rn: u8, imm: u8) {
        self.raw(0x6000 | u16::from(imm / 4) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rt) & 7);
    }
    pub fn ldr_imm(&mut self, rt: u8, rn: u8, imm: u8) {
        self.raw(0x6800 | u16::from(imm / 4) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rt) & 7);
    }
    pub fn strb_imm(&mut self, rt: u8, rn: u8, imm: u8) {
        self.raw(0x7000 | u16::from(imm) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rt) & 7);
    }
    pub fn ldrb_imm(&mut self, rt: u8, rn: u8, imm: u8) {
        self.raw(0x7800 | u16::from(imm) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rt) & 7);
    }
    pub fn strh_imm(&mut self, rt: u8, rn: u8, imm: u8) {
        self.raw(0x8000 | u16::from(imm / 2) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rt) & 7);
    }
    pub fn ldrh_imm(&mut self, rt: u8, rn: u8, imm: u8) {
        self.raw(0x8800 | u16::from(imm / 2) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rt) & 7);
    }

    /// `LDR Rt, [PC, #imm_bytes]` (imm is in bytes, 4-aligned, forward).
    pub fn ldr_lit(&mut self, rt: u8, target: impl Into<String>) {
        self.emit(Item::LdrLit {
            rt,
            target: target.into(),
        });
    }

    /// `LDRD Rt, Rt2, [PC, #imm]` against the literal pool at `target`.
    pub fn ldrd_lit(&mut self, rt: u8, rt2: u8, target: impl Into<String>) {
        self.emit(Item::LdrdLit {
            rt,
            rt2,
            target: target.into(),
        });
    }

    /// Register-offset load/store; `kind` is the 3-bit `LsReg` selector
    /// (`STR=0, STRH=1, STRB=2, LDRSB=3, LDR=4, LDRH=5, LDRB=6, LDRSH=7`).
    pub fn ls_reg(&mut self, kind: u8, rt: u8, rn: u8, rm: u8) {
        self.raw(0x5000 | (u16::from(kind) & 7) << 9 | (u16::from(rm) & 7) << 6 | (u16::from(rn) & 7) << 3 | u16::from(rt) & 7);
    }

    pub fn str_sp(&mut self, rt: u8, imm: u8) {
        self.raw(0x9000 | (u16::from(rt) & 7) << 8 | u16::from(imm / 4));
    }
    pub fn ldr_sp(&mut self, rt: u8, imm: u8) {
        self.raw(0x9800 | (u16::from(rt) & 7) << 8 | u16::from(imm / 4));
    }

    /// `ADR Rd, label` PC-relative add with a raw byte offset (the base is
    /// `Align(pc + 4, 4)`). Use `adr_pc_off` directly; only an explicit byte
    /// offset is supported here.
    pub fn adr_pc_off(&mut self, rd: u8, imm: u8) {
        self.raw(0xA000 | (u16::from(rd) & 7) << 8 | u16::from(imm / 4));
    }

    pub fn addr_sp(&mut self, rd: u8, imm: u8) {
        self.raw(0xA800 | (u16::from(rd) & 7) << 8 | u16::from(imm / 4));
    }

    pub fn add_sp(&mut self, imm: u8) {
        self.raw(0xB000 | u16::from(imm / 4));
    }
    pub fn sub_sp(&mut self, imm: u8) {
        self.raw(0xB080 | u16::from(imm / 4));
    }

    /// `PUSH {r0..r7}` list mask plus LR.
    pub fn push(&mut self, regs: u8, include_lr: bool) {
        self.raw(0xB400 | u16::from(include_lr) << 8 | u16::from(regs));
    }

    /// `POP {r0..r7}` list mask plus PC.
    pub fn pop(&mut self, regs: u8, load_pc: bool) {
        self.raw(0xBC00 | u16::from(load_pc) << 8 | u16::from(regs));
    }

    pub fn stmia(&mut self, rn: u8, list: u8) {
        self.raw(0xC000 | (u16::from(rn) & 7) << 8 | u16::from(list));
    }
    pub fn ldmia(&mut self, rn: u8, list: u8) {
        self.raw(0xC800 | (u16::from(rn) & 7) << 8 | u16::from(list));
    }

    // --------------------------- Thumb-2 memory ---------------------------

    /// The `hi` prefix nibble for each load/store op, in the *non-wide*
    /// (`imm8`/register-offset) form. The `imm12` form adds bit 8 (`0x0080`).
    fn mem_hi(access: MemAccess) -> u16 {
        match access {
            MemAccess::Str => 0xF840,
            MemAccess::Ldr => 0xF850,
            MemAccess::Strb => 0xF800,
            MemAccess::Ldrb => 0xF810,
            MemAccess::Strh => 0xF820,
            MemAccess::Ldrh => 0xF830,
            MemAccess::Ldrsb => 0xF910,
            MemAccess::Ldrsh => 0xF930,
        }
    }

    fn mem_imm12(&mut self, access: MemAccess, rt: u8, rn: u8, imm12: u16) {
        let hi = Self::mem_hi(access) | 0x0080 | u16::from(rn);
        let lo = u16::from(rt) << 12 | imm12 & 0xFFF;
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `STR.W Rt, [Rn, #imm12]`.
    pub fn str_wide(&mut self, rt: u8, rn: u8, off: u16) {
        self.mem_imm12(MemAccess::Str, rt, rn, off);
    }
    /// `LDR.W Rt, [Rn, #imm12]`.
    pub fn ldr_wide(&mut self, rt: u8, rn: u8, off: u16) {
        self.mem_imm12(MemAccess::Ldr, rt, rn, off);
    }
    pub fn ldrb_wide(&mut self, rt: u8, rn: u8, off: u16) {
        self.mem_imm12(MemAccess::Ldrb, rt, rn, off);
    }
    pub fn strb_wide(&mut self, rt: u8, rn: u8, off: u16) {
        self.mem_imm12(MemAccess::Strb, rt, rn, off);
    }
    pub fn strh_wide(&mut self, rt: u8, rn: u8, off: u16) {
        self.mem_imm12(MemAccess::Strh, rt, rn, off);
    }
    pub fn ldrh_wide(&mut self, rt: u8, rn: u8, off: u16) {
        self.mem_imm12(MemAccess::Ldrh, rt, rn, off);
    }
    pub fn ldrsb_wide(&mut self, rt: u8, rn: u8, off: u16) {
        self.mem_imm12(MemAccess::Ldrsb, rt, rn, off);
    }
    pub fn ldrsh_wide(&mut self, rt: u8, rn: u8, off: u16) {
        self.mem_imm12(MemAccess::Ldrsh, rt, rn, off);
    }

    /// The imm8 form flags: `PUW` in decode order (`P` bit10, `U` bit9,
    /// `W` bit8), plus the mandatory selector bit 11.
    fn mem_imm8(&mut self, access: MemAccess, rt: u8, rn: u8, puw: u16, imm8: u8) {
        let hi = Self::mem_hi(access) | u16::from(rn);
        let lo = u16::from(rt) << 12 | 0x0800 | puw << 8 | u16::from(imm8);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `LDR Rt, [Rn, #imm8]!` (pre-indexed, P=1 U=1 W=1).
    pub fn ldr_pre(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldr, rt, rn, 0b111, imm8);
    }
    /// `STR Rt, [Rn, #imm8]!` (pre-indexed, P=1 U=1 W=1).
    pub fn str_pre(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Str, rt, rn, 0b111, imm8);
    }
    /// `LDR Rt, [Rn], #imm8` (post-indexed, P=0 U=1 W=1).
    pub fn ldr_post(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldr, rt, rn, 0b011, imm8);
    }
    /// `STR Rt, [Rn], #imm8` (post-indexed, P=0 U=1 W=1).
    pub fn str_post(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Str, rt, rn, 0b011, imm8);
    }
    /// `STRT Rt, [Rn, #imm8]` (unprivileged, P=1 U=1 W=0).
    pub fn strt(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Str, rt, rn, 0b110, imm8);
    }
    /// `LDRT Rt, [Rn, #imm8]`.
    pub fn ldrt(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldr, rt, rn, 0b110, imm8);
    }

    /// Byte pre-indexed: `LDRB Rt, [Rn, #imm8]!`.
    pub fn ldrb_pre(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrb, rt, rn, 0b111, imm8);
    }
    /// Byte pre-indexed: `STRB Rt, [Rn, #imm8]!`.
    pub fn strb_pre(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Strb, rt, rn, 0b111, imm8);
    }
    /// Byte post-indexed: `LDRB Rt, [Rn], #imm8`.
    pub fn ldrb_post(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrb, rt, rn, 0b011, imm8);
    }
    /// Byte post-indexed: `STRB Rt, [Rn], #imm8`.
    pub fn strb_post(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Strb, rt, rn, 0b011, imm8);
    }
    /// Unprivileged byte: `LDRBT Rt, [Rn, #imm8]`.
    pub fn ldrbt(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrb, rt, rn, 0b110, imm8);
    }
    /// Unprivileged byte: `STRBT Rt, [Rn, #imm8]`.
    pub fn strbt(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Strb, rt, rn, 0b110, imm8);
    }

    /// Halfword pre-indexed: `LDRH Rt, [Rn, #imm8]!`.
    pub fn ldrh_pre(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrh, rt, rn, 0b111, imm8);
    }
    /// Halfword pre-indexed: `STRH Rt, [Rn, #imm8]!`.
    pub fn strh_pre(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Strh, rt, rn, 0b111, imm8);
    }
    /// Halfword post-indexed: `LDRH Rt, [Rn], #imm8`.
    pub fn ldrh_post(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrh, rt, rn, 0b011, imm8);
    }
    /// Halfword post-indexed: `STRH Rt, [Rn], #imm8`.
    pub fn strh_post(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Strh, rt, rn, 0b011, imm8);
    }
    /// Unprivileged halfword: `LDRHT Rt, [Rn, #imm8]`.
    pub fn ldrht(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrh, rt, rn, 0b110, imm8);
    }
    /// Unprivileged halfword: `STRHT Rt, [Rn, #imm8]`.
    pub fn strht(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Strh, rt, rn, 0b110, imm8);
    }

    /// Signed byte pre-indexed: `LDRSB Rt, [Rn, #imm8]!`.
    pub fn ldrsb_pre(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrsb, rt, rn, 0b111, imm8);
    }
    /// Signed byte post-indexed: `LDRSB Rt, [Rn], #imm8`.
    pub fn ldrsb_post(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrsb, rt, rn, 0b011, imm8);
    }
    /// Unprivileged signed byte: `LDRSBT Rt, [Rn, #imm8]`.
    pub fn ldrsbt(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrsb, rt, rn, 0b110, imm8);
    }
    /// Signed halfword pre-indexed: `LDRSH Rt, [Rn, #imm8]!`.
    pub fn ldrsh_pre(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrsh, rt, rn, 0b111, imm8);
    }
    /// Signed halfword post-indexed: `LDRSH Rt, [Rn], #imm8`.
    pub fn ldrsh_post(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrsh, rt, rn, 0b011, imm8);
    }
    /// Unprivileged signed halfword: `LDRSHT Rt, [Rn, #imm8]`.
    pub fn ldrsht(&mut self, rt: u8, rn: u8, imm8: u8) {
        self.mem_imm8(MemAccess::Ldrsh, rt, rn, 0b110, imm8);
    }

    /// Register-offset wide form: `LDR.W Rt, [Rn, Rm, LSL #shift]`.
    pub fn ldr_reg_wide(&mut self, rt: u8, rn: u8, rm: u8, shift_lsl: u8) {
        let hi = Self::mem_hi(MemAccess::Ldr) | u16::from(rn);
        // Second halfword: `Rt 000000 shimm Rm` (shimm = the shift itself).
        let lo = u16::from(rt) << 12 | u16::from(shift_lsl) << 4 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `LDMIA rn!, {list}` / `STMIA rn!, {list}` and the DB forms.
    pub fn ldm_wide(&mut self, rn: u8, writeback: bool, dec_before: bool, list: u16) {
        let hi = if dec_before { 0xE910 } else { 0xE890 };
        let hi = hi | u16::from(writeback) << 5 | u16::from(rn);
        self.raw32(hi as u32 | (list as u32) << 16);
    }

    pub fn stm_wide(&mut self, rn: u8, writeback: bool, dec_before: bool, list: u16) {
        let hi = if dec_before { 0xE900 } else { 0xE880 };
        let hi = hi | u16::from(writeback) << 5 | u16::from(rn);
        self.raw32(hi as u32 | (list as u32) << 16);
    }

    /// `STRD Rt, Rt2, [Rn, #imm]` / `LDRD` — the imm8 T1 forms. `load`
    /// selects `LDRD`, `index` the pre-indexed (writeback) over the plain
    /// offset form, `post` the post-indexed form; `imm` is in bytes.
    fn dual_mem(&mut self, load: bool, rt: u8, rt2: u8, rn: u8, index: bool, post: bool, imm: u8) {
        let p = if post { 0 } else { 1 };
        let w = if post || index { 1 } else { 0 };
        let hi = 0xE840 | p << 8 | 0x80 | w << 5 | u16::from(load) << 4 | u16::from(rn);
        let lo = u16::from(rt) << 12 | u16::from(rt2) << 8 | u16::from(imm >> 2);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `STRD Rt, Rt2, [Rn, #imm]` (offset form, no writeback).
    pub fn strd(&mut self, rt: u8, rt2: u8, rn: u8, imm: u8) {
        self.dual_mem(false, rt, rt2, rn, false, false, imm);
    }
    /// `LDRD Rt, Rt2, [Rn, #imm]` (offset form; `Rn == 15` is the literal).
    pub fn ldrd(&mut self, rt: u8, rt2: u8, rn: u8, imm: u8) {
        self.dual_mem(true, rt, rt2, rn, false, false, imm);
    }
    /// `STRD Rt, Rt2, [Rn, #imm]!` (pre-indexed with writeback).
    pub fn strd_pre(&mut self, rt: u8, rt2: u8, rn: u8, imm: u8) {
        self.dual_mem(false, rt, rt2, rn, true, false, imm);
    }
    /// `LDRD Rt, Rt2, [Rn, #imm]!` (pre-indexed with writeback).
    pub fn ldrd_pre(&mut self, rt: u8, rt2: u8, rn: u8, imm: u8) {
        self.dual_mem(true, rt, rt2, rn, true, false, imm);
    }
    /// `STRD Rt, Rt2, [Rn], #imm` (post-indexed).
    pub fn strd_post(&mut self, rt: u8, rt2: u8, rn: u8, imm: u8) {
        self.dual_mem(false, rt, rt2, rn, false, true, imm);
    }
    /// `LDRD Rt, Rt2, [Rn], #imm` (post-indexed).
    pub fn ldrd_post(&mut self, rt: u8, rt2: u8, rn: u8, imm: u8) {
        self.dual_mem(true, rt, rt2, rn, false, true, imm);
    }

    pub fn dmb(&mut self) {
        self.raw32(0xF3BF_8F5F);
    }
    pub fn dsb(&mut self) {
        self.raw32(0xF3BF_8F4F);
    }
    pub fn isb(&mut self) {
        self.raw32(0xF3BF_8F6F);
    }

    // ------------------------------ exclusive ------------------------------

    /// `LDREX.W Rt, [Rn, #off]` (word, `off` in bytes, 4-aligned).
    pub fn ldrex(&mut self, rt: u8, rn: u8, off: u8) {
        let hi = 0xE850 | u16::from(rn);
        let lo = u16::from(rt) << 12 | 0xF00 | u16::from(off / 4);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }
    /// `STREX.W Rd, Rt, [Rn, #off]` (word); `Rd` receives the success bit.
    pub fn strex(&mut self, rd: u8, rt: u8, rn: u8, off: u8) {
        let hi = 0xE840 | u16::from(rn);
        let lo = u16::from(rt) << 12 | u16::from(rd) << 8 | u16::from(off / 4);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }
    /// `LDREXB Rt, [Rn]`.
    pub fn ldrexb(&mut self, rt: u8, rn: u8) {
        let hi = 0xE8D0 | u16::from(rn);
        let lo = u16::from(rt) << 12 | 0xF4F;
        self.raw32(hi as u32 | (lo as u32) << 16);
    }
    /// `LDREXH Rt, [Rn]`.
    pub fn ldrexh(&mut self, rt: u8, rn: u8) {
        let hi = 0xE8D0 | u16::from(rn);
        let lo = u16::from(rt) << 12 | 0xF5F;
        self.raw32(hi as u32 | (lo as u32) << 16);
    }
    /// `STREXB Rd, Rt, [Rn]`.
    pub fn strexb(&mut self, rd: u8, rt: u8, rn: u8) {
        let hi = 0xE8C0 | u16::from(rn);
        let lo = u16::from(rt) << 12 | 0xF40 | u16::from(rd);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }
    /// `STREXH Rd, Rt, [Rn]`.
    pub fn strexh(&mut self, rd: u8, rt: u8, rn: u8) {
        let hi = 0xE8C0 | u16::from(rn);
        let lo = u16::from(rt) << 12 | 0xF50 | u16::from(rd);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    // ------------------------- system registers ---------------------------

    /// `MRS Rd, <sysreg>` (`sys` is the `SYSm` 8-bit field, see `SysReg`).
    pub fn mrs(&mut self, rd: u8, sys: u8) {
        let lo = 0x8000 | u16::from(rd) << 8 | u16::from(sys);
        self.raw32(0xF3EF as u32 | (lo as u32) << 16);
    }
    /// `MSR <sysreg>, Rn`.
    pub fn msr(&mut self, sys: u8, rn: u8) {
        let hi = 0xF380 | u16::from(rn);
        let lo = 0x8000 | u16::from(sys);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `SVC #imm` (16-bit).
    pub fn svc(&mut self, imm: u8) {
        self.raw(0xDF00 | u16::from(imm));
    }

    // ---------------------------- divide / multiply ----------------------------

    pub fn sdiv(&mut self, rd: u8, rn: u8, rm: u8) {
        self.divide(0xFB90, rd, rn, rm);
    }
    pub fn udiv(&mut self, rd: u8, rn: u8, rm: u8) {
        self.divide(0xFBB0, rd, rn, rm);
    }

    fn divide(&mut self, base: u16, rd: u8, rn: u8, rm: u8) {
        let hi = base | u16::from(rn);
        let lo = 0xF000 | u16::from(rd) << 8 | 0x00F0 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    pub fn mul_wide(&mut self, rd: u8, rn: u8, rm: u8) {
        self.mul(0xFB00, 0xF, rd, rn, rm);
    }
    /// `MLA Rd, Rn, Rm, Ra`.
    pub fn mla(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.mul(0xFB00, ra, rd, rn, rm);
    }
    /// `MLS Rd, Rn, Rm, Ra`.
    pub fn mls(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        let hi = 0xFB00 | u16::from(rn);
        let lo = u16::from(ra) << 12 | u16::from(rd) << 8 | 0x0010 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }
    fn mul(&mut self, base: u16, ra: u8, rd: u8, rn: u8, rm: u8) {
        let hi = base | u16::from(rn);
        let lo = u16::from(ra) << 12 | u16::from(rd) << 8 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `SMULL RdLo, RdHi, Rn, Rm`.
    pub fn smull(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.long_mul(0xFB80, rdlo, rdhi, rn, rm);
    }
    pub fn umull(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.long_mul(0xFBA0, rdlo, rdhi, rn, rm);
    }
    pub fn smlal(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.long_mul(0xFBC0, rdlo, rdhi, rn, rm);
    }
    pub fn umlal(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.long_mul(0xFBE0, rdlo, rdhi, rn, rm);
    }
    fn long_mul(&mut self, base: u16, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        let hi = base | u16::from(rn);
        let lo = u16::from(rdlo) << 12 | u16::from(rdhi) << 8 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    // --------------------------- DSP / dual multiply ---------------------------

    /// The ARMv7E-M DSP multiply family: `SMULBB..SMULTT`, `SMULWB/WT`,
    /// `SMMUL/R`, `USAD8/USADA8`. `hi_zone` is the `0xFB00`-relative prefix
    /// nibble (`1`/`3`/`5`/`7`), `sel` the second-halfword selector nibble,
    /// and `ra` the accumulate register (`0xF` for the no-accumulate forms).
    fn dsp_mul(&mut self, hi_zone: u16, sel: u16, rd: u8, rn: u8, rm: u8, ra: u8) {
        let hi = 0xFB00 | hi_zone << 4 | u16::from(rn);
        let lo = u16::from(ra) << 12 | u16::from(rd) << 8 | (sel & 0xF) << 4 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    pub fn smulbb(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(1, 0, rd, rn, rm, 0xF);
    }
    pub fn smulbt(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(1, 1, rd, rn, rm, 0xF);
    }
    pub fn smultb(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(1, 2, rd, rn, rm, 0xF);
    }
    pub fn smultt(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(1, 3, rd, rn, rm, 0xF);
    }
    pub fn smulwb(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(3, 0, rd, rn, rm, 0xF);
    }
    pub fn smulwt(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(3, 1, rd, rn, rm, 0xF);
    }
    pub fn smmul(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(5, 0, rd, rn, rm, 0xF);
    }
    pub fn smmulr(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(5, 1, rd, rn, rm, 0xF);
    }
    pub fn usad8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.dsp_mul(7, 0, rd, rn, rm, 0xF);
    }
    pub fn usada8(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dsp_mul(7, 0, rd, rn, rm, ra);
    }

    /// The ARMv7E-M dual multiply family: `SMLABB..`, `SMLAD/X`, `SMLAWB/WT`,
    /// `SMLSD/X`, `SMMLA/R`, `SMMLS/R`, the long `SMLALBB..`, `SMLALD/X`,
    /// `SMLSLD/X`, and `UMAAL`. `prefix` is the full first-halfword prefix
    /// (`0xFB10`..`0xFBE0`), `sel` the `lo` selector nibble; long forms pass
    /// `rdlo`/`rdhi` as `rd`/`ra`.
    fn dual_mul(&mut self, prefix: u16, sel: u16, rd: u8, rn: u8, rm: u8, ra: u8) {
        let hi = prefix | u16::from(rn);
        let lo = u16::from(ra) << 12 | u16::from(rd) << 8 | (sel & 0xF) << 4 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    pub fn smlabb(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB10, 0, rd, rn, rm, ra);
    }
    pub fn smlabt(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB10, 1, rd, rn, rm, ra);
    }
    pub fn smlatb(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB10, 2, rd, rn, rm, ra);
    }
    pub fn smlatt(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB10, 3, rd, rn, rm, ra);
    }
    pub fn smlad(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB20, 0, rd, rn, rm, ra);
    }
    pub fn smladx(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB20, 1, rd, rn, rm, ra);
    }
    pub fn smlawb(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB30, 0, rd, rn, rm, ra);
    }
    pub fn smlawt(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB30, 1, rd, rn, rm, ra);
    }
    pub fn smlsd(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB40, 0, rd, rn, rm, ra);
    }
    pub fn smlsdx(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB40, 1, rd, rn, rm, ra);
    }
    pub fn smmla(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB50, 0, rd, rn, rm, ra);
    }
    pub fn smmlar(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB50, 1, rd, rn, rm, ra);
    }
    pub fn smmls(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB60, 0, rd, rn, rm, ra);
    }
    pub fn smmlsr(&mut self, rd: u8, rn: u8, rm: u8, ra: u8) {
        self.dual_mul(0xFB60, 1, rd, rn, rm, ra);
    }
    pub fn smlalbb(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBC0, 8, rdlo, rn, rm, rdhi);
    }
    pub fn smlalbt(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBC0, 9, rdlo, rn, rm, rdhi);
    }
    pub fn smlaltb(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBC0, 0xA, rdlo, rn, rm, rdhi);
    }
    pub fn smlaltt(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBC0, 0xB, rdlo, rn, rm, rdhi);
    }
    pub fn smlald(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBC0, 0xC, rdlo, rn, rm, rdhi);
    }
    pub fn smlaldx(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBC0, 0xD, rdlo, rn, rm, rdhi);
    }
    pub fn smlsld(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBD0, 0xC, rdlo, rn, rm, rdhi);
    }
    pub fn smlsldx(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBD0, 0xD, rdlo, rn, rm, rdhi);
    }
    pub fn umaal(&mut self, rdlo: u8, rdhi: u8, rn: u8, rm: u8) {
        self.dual_mul(0xFBE0, 6, rdlo, rn, rm, rdhi);
    }

    // ------------------------------ bitfield ------------------------------

    fn bitfield(&mut self, base: u16, rd: u8, rn: u8, lsb: u8, field: u8) {
        let hi = base | u16::from(rn);
        let lo = u16::from(lsb >> 2) << 12 | u16::from(rd) << 8 | u16::from(lsb & 3) << 6 | u16::from(field) & 0x1F;
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `SBFX Rd, Rn, #lsb, #width`.
    pub fn sbfx(&mut self, rd: u8, rn: u8, lsb: u8, width: u8) {
        self.bitfield(0xF340, rd, rn, lsb, width - 1);
    }
    /// `UBFX Rd, Rn, #lsb, #width`.
    pub fn ubfx(&mut self, rd: u8, rn: u8, lsb: u8, width: u8) {
        self.bitfield(0xF3C0, rd, rn, lsb, width - 1);
    }
    /// `BFI Rd, Rn, #lsb, #width`.
    pub fn bfi(&mut self, rd: u8, rn: u8, lsb: u8, width: u8) {
        self.bitfield(0xF360, rd, rn, lsb, lsb + width - 1);
    }
    /// `BFC Rd, #lsb, #width`.
    pub fn bfc(&mut self, rd: u8, lsb: u8, width: u8) {
        self.bitfield(0xF36F, rd, 15, lsb, lsb + width - 1);
    }

    // -------------------------------- saturate --------------------------------

    /// `SSAT/USAT` shared encode. `field` is the raw 5-bit `sat_imm`; `SSAT`
    /// (imm 1..=32) stores `imm - 1`, `USAT` (imm 0..=31) stores it directly.
    fn sat(&mut self, base: u16, rd: u8, rn: u8, field: u8, shift_amt: u8, sh: bool) {
        let hi = base | (u16::from(sh)) << 5 | u16::from(rn);
        let lo = u16::from(shift_amt >> 2) << 12 | u16::from(rd) << 8 | u16::from(shift_amt & 3) << 6 | u16::from(field) & 0x1F;
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    /// `SSAT Rd, #n, Rn`.
    pub fn ssat(&mut self, rd: u8, rn: u8, n: u8, shift_amt: u8) {
        self.sat(0xF300, rd, rn, n - 1, shift_amt, false);
    }
    /// `USAT Rd, #n, Rn`.
    pub fn usat(&mut self, rd: u8, rn: u8, n: u8, shift_amt: u8) {
        self.sat(0xF380, rd, rn, n, shift_amt, false);
    }
    /// `SSAT Rd, #n, Rn, ASR #shift_amt`.
    pub fn ssat_asr(&mut self, rd: u8, rn: u8, n: u8, shift_amt: u8) {
        self.sat(0xF300, rd, rn, n - 1, shift_amt, true);
    }
    /// `USAT Rd, #n, Rn, ASR #shift_amt`.
    pub fn usat_asr(&mut self, rd: u8, rn: u8, n: u8, shift_amt: u8) {
        self.sat(0xF380, rd, rn, n, shift_amt, true);
    }
    /// `SSAT16 Rd, #n, Rn` (per-halfword signed saturate, no shift).
    pub fn ssat16(&mut self, rd: u8, rn: u8, n: u8) {
        self.sat(0xF300, rd, rn, n - 1, 0, true);
    }
    /// `USAT16 Rd, #n, Rn` (per-halfword unsigned saturate, no shift).
    pub fn usat16(&mut self, rd: u8, rn: u8, n: u8) {
        self.sat(0xF380, rd, rn, n, 0, true);
    }

    // ------------------------------ qadd / qsub ------------------------------

    fn qop(&mut self, sel: u16, rd: u8, rn: u8, rm: u8) {
        let hi = 0xFA80 | u16::from(rn);
        let lo = 0xF000 | u16::from(rd) << 8 | sel | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    pub fn qadd(&mut self, rd: u8, rn: u8, rm: u8) {
        self.qop(0x0080, rd, rn, rm);
    }
    pub fn qdadd(&mut self, rd: u8, rn: u8, rm: u8) {
        self.qop(0x0090, rd, rn, rm);
    }
    pub fn qsub(&mut self, rd: u8, rn: u8, rm: u8) {
        self.qop(0x00A0, rd, rn, rm);
    }
    pub fn qdsub(&mut self, rd: u8, rn: u8, rm: u8) {
        self.qop(0x00B0, rd, rn, rm);
    }

    // -------------------------------- unary --------------------------------

    /// Thumb-2 unary ops (`REV`, `REV16`, `RBIT`, `REVSH`) at `0xFA90` and
    /// `CLZ` at `0xFAB0`; `sel` is the second-halfword selector nibble.
    fn unary(&mut self, base: u16, sel: u16, rd: u8, rm: u8) {
        let hi = base | u16::from(rm); // `Rm2` low nibble must copy `Rm`
        let lo = 0xF000 | u16::from(rd) << 8 | sel | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    pub fn rev_wide(&mut self, rd: u8, rm: u8) {
        self.unary(0xFA90, 0x0080, rd, rm);
    }
    pub fn rev16_wide(&mut self, rd: u8, rm: u8) {
        self.unary(0xFA90, 0x0090, rd, rm);
    }
    pub fn rbit(&mut self, rd: u8, rm: u8) {
        self.unary(0xFA90, 0x00A0, rd, rm);
    }
    pub fn revsh_wide(&mut self, rd: u8, rm: u8) {
        self.unary(0xFA90, 0x00B0, rd, rm);
    }
    pub fn clz(&mut self, rd: u8, rm: u8) {
        self.unary(0xFAB0, 0x0080, rd, rm);
    }

    // ------------------------- narrow extend / reverse -------------------------

    /// Thumb-1 `SXTH/SXTB/UXTH/UXTB Rd, Rm`; `kind` is the two-bit
    /// signedness/size selector (`0` SXTH, `1` SXTB, `2` UXTH, `3` UXTB).
    fn narrow_extend(&mut self, kind: u8, rd: u8, rm: u8) {
        self.raw(0xB200 | (u16::from(kind) & 3) << 6 | (u16::from(rm) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn sxth(&mut self, rd: u8, rm: u8) {
        self.narrow_extend(0, rd, rm);
    }
    pub fn sxtb(&mut self, rd: u8, rm: u8) {
        self.narrow_extend(1, rd, rm);
    }
    pub fn uxth(&mut self, rd: u8, rm: u8) {
        self.narrow_extend(2, rd, rm);
    }
    pub fn uxtb(&mut self, rd: u8, rm: u8) {
        self.narrow_extend(3, rd, rm);
    }

    /// Thumb-1 `REV/REV16/REVSH Rd, Rm`; `kind` is the two-bit selector
    /// (`0` REV, `1` REV16, `3` REVSH).
    fn narrow_reverse(&mut self, kind: u8, rd: u8, rm: u8) {
        self.raw(0xBA00 | (u16::from(kind) & 3) << 6 | (u16::from(rm) & 7) << 3 | u16::from(rd) & 7);
    }

    pub fn rev(&mut self, rd: u8, rm: u8) {
        self.narrow_reverse(0, rd, rm);
    }
    pub fn rev16(&mut self, rd: u8, rm: u8) {
        self.narrow_reverse(1, rd, rm);
    }
    pub fn revsh(&mut self, rd: u8, rm: u8) {
        self.narrow_reverse(3, rd, rm);
    }

    // ------------------------- signed extend + add -------------------------

    /// Register-extend family with `rotate` in eighths (`ROR #rot*8`).
    fn extend_add(&mut self, op: u16, rd: u8, rn: u8, rm: u8, rot: u8) {
        let hi = 0xFA00 | op << 4 | u16::from(rn);
        let lo = 0xF080 | u16::from(rd) << 8 | (u16::from(rot) & 3) << 4 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    pub fn sxtah(&mut self, rd: u8, rn: u8, rm: u8, rot: u8) {
        self.extend_add(0, rd, rn, rm, rot);
    }
    pub fn uxtah(&mut self, rd: u8, rn: u8, rm: u8, rot: u8) {
        self.extend_add(1, rd, rn, rm, rot);
    }
    pub fn sxtab16(&mut self, rd: u8, rn: u8, rm: u8, rot: u8) {
        self.extend_add(2, rd, rn, rm, rot);
    }
    pub fn uxtab16(&mut self, rd: u8, rn: u8, rm: u8, rot: u8) {
        self.extend_add(3, rd, rn, rm, rot);
    }
    pub fn sxtab(&mut self, rd: u8, rn: u8, rm: u8, rot: u8) {
        self.extend_add(4, rd, rn, rm, rot);
    }
    pub fn uxtab(&mut self, rd: u8, rn: u8, rm: u8, rot: u8) {
        self.extend_add(5, rd, rn, rm, rot);
    }

    // --------------------------- parallel add/sub ---------------------------

    fn par(&mut self, fam: u16, sel: u16, rd: u8, rn: u8, rm: u8) {
        let hi = 0xFA00 | fam << 4 | u16::from(rn);
        // The selector nibble (`S/Q/SH/U/UQ/UH`, `1000` for `SEL`) sits in
        // bits[7:4] of the second halfword; `Rm` owns the low nibble.
        let lo = 0xF000 | u16::from(rd) << 8 | (sel & 0xF) << 4 | u16::from(rm);
        self.raw32(hi as u32 | (lo as u32) << 16);
    }

    pub fn sadd8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x8, 0x0, rd, rn, rm);
    }
    pub fn qadd8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x8, 0x1, rd, rn, rm);
    }
    pub fn shadd8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x8, 0x2, rd, rn, rm);
    }
    pub fn uadd8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x8, 0x4, rd, rn, rm);
    }
    pub fn uqadd8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x8, 0x5, rd, rn, rm);
    }
    pub fn uhadd8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x8, 0x6, rd, rn, rm);
    }
    pub fn sadd16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x9, 0x0, rd, rn, rm);
    }
    pub fn sasx(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xA, 0x0, rd, rn, rm);
    }
    pub fn ssax(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xE, 0x0, rd, rn, rm);
    }

    /// `SEL Rd, Rn, Rm` (GE-driven per-byte select).
    pub fn sel(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xA, 0x8, rd, rn, rm);
    }

    // 16-bit add, ASX/SAX, and byte/halfword sub families: Q/SH/U/UQ/UH are the
    // relabeled `S` selector modes of the same encoding.
    pub fn qadd16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x9, 0x1, rd, rn, rm);
    }
    pub fn shadd16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x9, 0x2, rd, rn, rm);
    }
    pub fn uadd16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x9, 0x4, rd, rn, rm);
    }
    pub fn uqadd16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x9, 0x5, rd, rn, rm);
    }
    pub fn uhadd16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0x9, 0x6, rd, rn, rm);
    }
    pub fn qasx(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xA, 0x1, rd, rn, rm);
    }
    pub fn shasx(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xA, 0x2, rd, rn, rm);
    }
    pub fn uasx(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xA, 0x4, rd, rn, rm);
    }
    pub fn uqasx(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xA, 0x5, rd, rn, rm);
    }
    pub fn uhasx(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xA, 0x6, rd, rn, rm);
    }
    pub fn ssub8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xC, 0x0, rd, rn, rm);
    }
    pub fn qsub8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xC, 0x1, rd, rn, rm);
    }
    pub fn shsub8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xC, 0x2, rd, rn, rm);
    }
    pub fn usub8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xC, 0x4, rd, rn, rm);
    }
    pub fn uqsub8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xC, 0x5, rd, rn, rm);
    }
    pub fn uhsub8(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xC, 0x6, rd, rn, rm);
    }
    pub fn ssub16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xD, 0x0, rd, rn, rm);
    }
    pub fn qsub16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xD, 0x1, rd, rn, rm);
    }
    pub fn shsub16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xD, 0x2, rd, rn, rm);
    }
    pub fn usub16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xD, 0x4, rd, rn, rm);
    }
    pub fn uqsub16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xD, 0x5, rd, rn, rm);
    }
    pub fn uhsub16(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xD, 0x6, rd, rn, rm);
    }
    pub fn qsax(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xE, 0x1, rd, rn, rm);
    }
    pub fn shsax(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xE, 0x2, rd, rn, rm);
    }
    pub fn usax(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xE, 0x4, rd, rn, rm);
    }
    pub fn uqsax(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xE, 0x5, rd, rn, rm);
    }
    pub fn uhsax(&mut self, rd: u8, rn: u8, rm: u8) {
        self.par(0xE, 0x6, rd, rn, rm);
    }

    // -------------------------------- finish --------------------------------

    /// Lay out the program, resolve all labels and branch/literal immediates,
    /// and produce the loadable image (vector table + code).
    ///
    /// # Errors
    ///
    /// Returns an error string for an undefined label or an out-of-range
    /// narrow branch/literal.
    pub fn finish(self, sp: u32) -> Result<Image, String> {
        // First pass: sizes are fixed, so compute every label's address.
        let mut pc = CODE_BASE;
        let mut labels = HashMap::new();
        for item in &self.items {
            match item {
                Item::Label(name) => {
                    labels.insert(name.clone(), pc);
                }
                Item::PadTo4 => {
                    pc += (4 - (pc % 4)) % 4;
                }
                other => pc += size_of(other),
            }
        }

        // Second pass: emit bytes, resolving targets now that labels exist.
        let mut code: Vec<u8> = Vec::with_capacity((pc - CODE_BASE) as usize);
        let mut cur = CODE_BASE;
        let mut emit = |cur: &mut u32, bytes: &[u8]| {
            let idx = (*cur - CODE_BASE) as usize;
            debug_assert_eq!(idx, code.len());
            code.extend_from_slice(bytes);
            *cur += bytes.len() as u32;
        };
        for item in &self.items {
            match item {
                Item::Label(_) => {}
                Item::H2(hw) => emit(&mut cur, &hw.to_le_bytes()),
                Item::H4(w) => {
                    let bytes = [
                        (w & 0xFFFF) as u8,
                        (w >> 8) as u8,
                        (w >> 16) as u8,
                        (w >> 24) as u8,
                    ];
                    emit(&mut cur, &bytes);
                }
                Item::B { target } | Item::Bl { target } => {
                    let t = labels
                        .get(target)
                        .copied()
                        .ok_or_else(|| format!("undefined label {target:?}"))?;
                    let off = i64::from(t) - i64::from(cur + 4);
                    let (hi, lo) = match item {
                        Item::B { .. } => {
                            let (hi, lo) = encode_branch(off, 0x9000);
                            (hi, lo)
                        }
                        _ => encode_branch(off, 0xD000),
                    };
                    let bytes = [
                        hi as u8,
                        (hi >> 8) as u8,
                        lo as u8,
                        (lo >> 8) as u8,
                    ];
                    emit(&mut cur, &bytes);
                }
                Item::Bc { cc, target } => {
                    let t = labels
                        .get(target)
                        .copied()
                        .ok_or_else(|| format!("undefined label {target:?}"))?;
                    let off = i64::from(t) - i64::from(cur + 4);
                    if off < -256 || off > 254 || off % 2 != 0 {
                        return Err(format!("conditional branch to {target:?} out of ±256-byte range"));
                    }
                    let imm8 = ((off >> 1) as i16) as u8;
                    emit(
                        &mut cur,
                        &(0xD000 | u16::from(*cc) << 8 | u16::from(imm8)).to_le_bytes(),
                    );
                }
                Item::BcWide { cc, target } => {
                    let t = labels
                        .get(target)
                        .copied()
                        .ok_or_else(|| format!("undefined label {target:?}"))?;
                    let off = i64::from(t) - i64::from(cur + 4);
                    if off < -(1 << 20) || off > (1 << 20) - 2 || off % 2 != 0 {
                        return Err(format!("conditional wide branch to {target:?} out of ±1 MB range"));
                    }
                    let u = (off as i32) as u32;
                    let s = (u >> 20) & 1;
                    let i1 = (u >> 13) & 1;
                    let i2 = (u >> 12) & 1;
                    let j1 = (1 - (i1 ^ s)) as u16;
                    let j2 = (1 - (i2 ^ s)) as u16;
                    let hi = 0xF000 | (s << 10) as u16 | u16::from(*cc) << 6 | ((u >> 1) & 0x3F) as u16;
                    let lo = 0x8000 | j1 << 13 | j2 << 11 | ((u >> 1) & 0x7FF) as u16;
                    let bytes = [hi as u8, (hi >> 8) as u8, lo as u8, (lo >> 8) as u8];
                    emit(&mut cur, &bytes);
                }
                Item::Cbz {
                    rn,
                    nonzero,
                    target,
                } => {
                    let t = labels
                        .get(target)
                        .copied()
                        .ok_or_else(|| format!("undefined label {target:?}"))?;
                    let off = i64::from(t) - i64::from(cur + 4);
                    if off < 2 || off > 126 || off % 2 != 0 {
                        return Err(format!("cbz/cbnz to {target:?} out of +2..126-byte range"));
                    }
                    let b = (off >> 1) as u16; // 1..=63
                    let i = (b >> 5) & 1;
                    let imm5 = b & 0x1F;
                    let hw = 0xB100 | u16::from(*nonzero) << 11 | i << 9 | imm5 << 3 | u16::from(*rn);
                    emit(&mut cur, &hw.to_le_bytes());
                }
                Item::LdrLit { rt, target } => {
                    let t = labels
                        .get(target)
                        .copied()
                        .ok_or_else(|| format!("undefined label {target:?}"))?;
                    let base = (cur + 4) & !3;
                    let off = i64::from(t) - i64::from(base);
                    if off < 0 || off > 1020 || off % 4 != 0 {
                        return Err(format!("ldr literal to {target:?} out of range"));
                    }
                    let hw = 0x4800 | (u16::from(*rt) & 7) << 8 | (off / 4) as u16;
                    emit(&mut cur, &hw.to_le_bytes());
                }
                Item::LdrdLit { rt, rt2, target } => {
                    let t = labels
                        .get(target)
                        .copied()
                        .ok_or_else(|| format!("undefined label {target:?}"))?;
                    let base = (cur + 4) & !3;
                    let off = i64::from(t) - i64::from(base);
                    if off < 0 || off > 1020 || off % 4 != 0 {
                        return Err(format!("ldrd literal to {target:?} out of range"));
                    }
                    let hi = 0xE9DF; // LDRD Rt,Rt2,[pc,#imm], P=1 U=1 L=1 W=0, rn=15
                    let lo = u16::from(*rt) << 12 | u16::from(*rt2) << 8 | (off / 4) as u16;
                    let bytes = [hi as u8, (hi >> 8) as u8, lo as u8, (lo >> 8) as u8];
                    emit(&mut cur, &bytes);
                }
                Item::PadTo4 => {
                    let pad = (4 - (cur % 4)) % 4;
                    for _ in 0..pad / 2 {
                        emit(&mut cur, &0xBF00u16.to_le_bytes());
                    }
                }
                Item::DcU32(w) => emit(&mut cur, &w.to_le_bytes()),
            }
        }

        let mut bytes = vec![0u8; CODE_BASE as usize];
        bytes[0..4].copy_from_slice(&sp.to_le_bytes());
        bytes[4..8].copy_from_slice(&(CODE_BASE | 1).to_le_bytes());
        for (exc, name) in &self.vectors {
            let handler = labels
                .get(name)
                .ok_or_else(|| format!("undefined handler label {name:?}"))?;
            let idx = (exc * 4) as usize;
            if idx + 4 > CODE_BASE as usize {
                return Err(format!("vector {exc} beyond the 0x{CODE_BASE:x} table"));
            }
            bytes[idx..idx + 4].copy_from_slice(&(handler | 1).to_le_bytes());
        }
        bytes.extend_from_slice(&code);
        Ok(Image { bytes, labels })
    }
}

/// Encode the 25-bit signed branch offset (from `pc + 4`, in bytes) as a
/// `(hi, lo)` halfword pair. The decoder reconstructs the byte offset as
/// `S:imm10:I1:I2:imm11:0` (see `branch_off25`), so the fields are sliced from
/// the *byte* offset directly: `imm11` is bits[11:1], `imm10` bits[23:14], and
/// the single-bit `I1`/`I2` fall at bits 13/12. `lo_prefix` is `0x9000` for
/// `B.W` (T4) and `0xD000` for `BL` (T1).
#[must_use]
fn encode_branch(off_bytes: i64, lo_prefix: u16) -> (u16, u16) {
    let u = off_bytes as u32;
    let s = (u >> 24) & 1;
    let i1 = (u >> 13) & 1;
    let i2 = (u >> 12) & 1;
    let imm10 = ((u >> 14) & 0x3FF) as u16;
    let imm11 = ((u >> 1) & 0x7FF) as u16;
    // The instruction XORs in `S` before the inverted bit is stored.
    let j1 = (1 - (i1 ^ s)) as u16;
    let j2 = (1 - (i2 ^ s)) as u16;
    let hi = (0xF000 | ((s << 10) as u16) | imm10) as u16;
    let lo = (lo_prefix | (j1 << 13) | (j2 << 11) | imm11) as u16;
    (hi, lo)
}

/// Thumb-2 memory access op names for the `mem_hi` selector.
#[derive(Clone, Copy)]
enum MemAccess {
    Str,
    Ldr,
    Strb,
    Ldrb,
    Strh,
    Ldrh,
    Ldrsb,
    Ldrsh,
}