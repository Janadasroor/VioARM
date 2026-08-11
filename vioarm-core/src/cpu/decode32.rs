//! Decoding of the 32-bit Thumb-2 instructions.
//!
//! ARMv6-M is a 16-bit-only ISA apart from a tiny Thumb-2 subset: `bl`
//! (immediate), `mrs`/`msr`, and `dmb`/`dsb`/`isb`. There is no `blx`
//! immediate and no other Thumb-2 instruction on Cortex-M0. Every encoding is
//! verified against `llvm-mc -triple=thumbv6m-none-eabi`.
//!
//! ARMv7-M (Cortex-M3/M4) adds the full Thumb-2 set; this file carries the
//! data-processing group (`MOVW`/`MOVT`, shifted-register operations, and the
//! `ThumbExpandImm` plus raw-12-bit immediate forms), the single-data-transfer
//! family (all `LDR`/`STR` immediate sizes and the signed loads), and the
//! divide, multiply, unary (`REV`/`REV16`/`REVSH`/`RBIT`/`CLZ`), and bitfield
//! (`SBFX`/`UBFX`/`BFI`/`BFC`) families, verified against
//! `clang --target=thumbv7m`. Encoding ground truth lives in the test suite.

use super::decode::DecodeError;
use crate::error::HaltCause;

/// The system register (sysreg) id in `mrs`/`msr`. Values are the raw
/// 8-bit fields LLVM emits on `thumbv6m`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SysReg {
    Apsr,
    Xpsr,
    Ipsr,
    Epsr,
    Msp,
    Psp,
    Primask,
    Control,
    /// A reserved/unknown system-register id.
    Other(u8),
}

impl From<u8> for SysReg {
    fn from(id: u8) -> Self {
        match id {
            0x00 => Self::Apsr,
            0x03 => Self::Xpsr,
            0x05 => Self::Ipsr,
            0x06 => Self::Epsr,
            0x08 => Self::Msp,
            0x09 => Self::Psp,
            0x10 => Self::Primask,
            0x14 => Self::Control,
            other => Self::Other(other),
        }
    }
}

/// Memory barrier kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Barrier {
    Dmb,
    Dsb,
    Isb,
}

/// ARMv7-M data-processing opcode, canonical across the register and immediate
/// forms. The test aliases (`TST`/`TEQ`/`CMP`/`CMN`) never write `Rd`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DpOp {
    And,
    Eor,
    Bic,
    Orr,
    Orn,
    Mov,
    Mvn,
    Add,
    Adc,
    Sbc,
    Sub,
    Rsb,
    Tst,
    Teq,
    Cmp,
    Cmn,
}

/// Load/store access size in the ARMv7-M single-data-transfer family.
///
/// The `Ldr*` members are the unsigned loads; `Ldrsb`/`Ldrsh` sign-extend the
/// loaded byte/halfword. Which of the two `Ldr`/`Str` halfword forms (imm12
/// vs imm8) was encoded is carried separately by the decoder, so the enum
/// only names the memory op.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemOp {
    Str,
    Ldr,
    Strb,
    Ldrb,
    Strh,
    Ldrh,
    Ldrsb,
    Ldrsh,
}

/// Shift applied to `Rm` in an ARMv7-M data-processing (register)
/// instruction. Which variant the encoder intended (LSL/LSR/ASR/ROR #n, or
/// RRX) is resolved from the raw shift type plus its 5-bit amount.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShiftType {
    Lsl,
    Lsr,
    Asr,
    Ror,
}

/// Access width of an ARMv7-M exclusive load/store. The doubleword forms
/// (`LDREXD`/`STREXD`) are not valid on M-profile and are not modeled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExSize {
    Byte,
    Half,
    Word,
}

/// A decoded 32-bit instruction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Inst32 {
    /// `BL {label}`. `off` is the already-sign-extended byte offset from
    /// `pc + 4`. LR is set to `pc + 4 | 1` by the executor.
    Bl {
        off: i32,
    },
    /// `B.W {label}` (unconditional T4). `off` is the already-sign-extended
    /// byte offset from `pc + 4`.
    BranchWide {
        off: i32,
    },
    /// `B<c>.W {label}` (conditional T3). `off` is the already-sign-extended
    /// byte offset from `pc + 4`.
    BranchCondWide {
        cond: u8,
        off: i32,
    },
    /// `BLX {label}` (immediate). `off` is the already-sign-extended byte
    /// offset from `pc + 4`; targets are word-aligned, so the branch always
    /// selects the ARM state, which faults on ARMv7-M. LR is set to
    /// `pc + 4 | 1` by the executor.
    BlxImm {
        off: i32,
    },
    Mrs {
        rd: u8,
        sys: SysReg,
    },
    Msr {
        rn: u8,
        sys: SysReg,
    },
    Barrier {
        kind: Barrier,
    },
    /// `LDREX{B,H}` (word `LDREX`). `off` is the word-form `imm8 << 2`
    /// offset (0 for the byte/halfword forms). The executor marks the
    /// effective address with the exclusive monitor.
    Ldrex {
        rt: u8,
        rn: u8,
        size: ExSize,
        off: u32,
    },
    /// `STREX{B,H}` (word `STREX`). `rd` receives 0 on success and 1 on
    /// failure; `off` is the word-form `imm8 << 2` offset.
    Strex {
        rd: u8,
        rt: u8,
        rn: u8,
        size: ExSize,
        off: u32,
    },
    /// `CLREX`: clear the local exclusive monitor.
    Clrex,
    /// ARMv7-M data-processing with a shifted-register operand:
    /// `Rd = Rn <op> Shift(Rm)`. `rn == 15` marks the `MOV`/`MVN` aliases,
    /// which do not read `Rn`. `amount` is the 5-bit shift amount; a `Ror`
    /// with `amount == 0` is the RRX form.
    DpReg {
        op: DpOp,
        rd: u8,
        rn: u8,
        rm: u8,
        shift: ShiftType,
        amount: u8,
        set_flags: bool,
    },
    /// ARMv7-M data-processing with a 12-bit immediate. `imm12` is either
    /// fed through `ThumbExpandImm` (`expand`) or used as a raw 12-bit value
    /// (the `addw`/`subw` forms).
    DpImm {
        op: DpOp,
        rd: u8,
        rn: u8,
        imm12: u16,
        expand: bool,
        set_flags: bool,
    },
    /// ARMv7-M `MOVW`/`MOVT`. `imm` is the 16-bit literal; `high` selects
    /// `MOVT`.
    MovWide {
        rd: u8,
        imm: u16,
        high: bool,
    },
    /// ARMv7-M single-data-transfer with an immediate or register offset:
    /// `LDR`/`STR` word, byte, halfword, and the sign-extending loads. `off`
    /// is the immediate offset magnitude (0-4095 imm12 or 0-255 imm8) when
    /// `rm` is `None`; otherwise the offset is `Rv(Rm) << shift`, where
    /// `shift` is an LSL amount (0, 2, 4, or 6). `add` selects `+`/`-` (the
    /// register forms always add). `index`/`writeback` give the three
    /// addressing forms: offset (`index`, no writeback), pre-indexed (both),
    /// post-indexed (writeback only). `rn == 15` marks the PC-relative
    /// literal form (loads only), whose base is `Align(pc + 4, 4)`.
    Mem {
        access: MemOp,
        rt: u8,
        rn: u8,
        off: u32,
        rm: Option<u8>,
        shift: u8,
        add: bool,
        index: bool,
        writeback: bool,
    },
    /// ARMv7-M load/store dual: `LDRD`/`STRD` with an immediate offset.
    /// `imm` is the byte offset (`imm8 << 2`), `add` selects `+`/`-`,
    /// `index`/`writeback` give the three addressing forms as in `Mem`.
    /// `rn == 15` marks the `LDRD` literal form (loads only), whose base is
    /// `Align(pc + 4, 4)`; the low word goes to `rt` and the high word to
    /// `rt2`. Flags are not affected.
    LdrdStrd {
        is_load: bool,
        rt: u8,
        rt2: u8,
        rn: u8,
        imm: u32,
        add: bool,
        index: bool,
        writeback: bool,
    },
    /// ARMv7-M load/store multiple: `STMIA`/`LDMIA`/`STMDB`/`LDMDB
    /// rn{!}, {list}`. `regs` is the 16-bit register mask; `writeback` is the
    /// `!`, `dec_before` selects the decrement-before (DB) forms over IA.
    LdmStm {
        is_load: bool,
        rn: u8,
        regs: u16,
        writeback: bool,
        dec_before: bool,
    },
    /// ARMv7-M table branch: `TBB [Rn, Rm]` (`half == false`) or
    /// `TBH [Rn, Rm]` (`half == true`).
    TblBranch {
        rn: u8,
        rm: u8,
        half: bool,
    },
    /// ARMv7-M signed (`signed == true`) or unsigned divide:
    /// `div Rd, Rn, Rm`. Flags are not affected.
    Div {
        signed: bool,
        rd: u8,
        rn: u8,
        rm: u8,
    },
    /// ARMv7-M multiply family. `MUL` writes the low 32 bits of `Rn * Rm`;
    /// `MLA`/`MLS` add/subtract `Ra`, and the long forms write the 64-bit
    /// product (or accumulator, for the *AL forms) to `RdHi:RdLo`, which
    /// decode as `rd` (bits11:8) and `ra` (bits15:12) respectively. Flags
    /// are never affected, so `MUL` and `MLA` need no `S` bit.
    Mul {
        op: MulOp,
        rd: u8,
        rn: u8,
        rm: u8,
        /// `MLA`/`MLS` accumulator, or the `RdLo` half of a long multiply
        /// (bits15:12 of the encoding). For `MUL` this is the all-ones
        /// encoding sentinel.
        ra: u8,
    },
    /// ARMv7-M DSP multiply family. `ra` is the `USADA8` accumulator (the
    /// all-ones sentinel for every other form). Flags are never affected.
    DspMul {
        op: DspMulOp,
        rd: u8,
        rn: u8,
        rm: u8,
        ra: u8,
    },
    /// ARMv7-M dual multiply family. `ra` is the accumulate operand (the
    /// all-ones sentinel for `SMUAD`/`SMUSD`), or the `RdLo` half of the long
    /// forms, which decode as `rd` (bits11:8) = `RdHi` and `ra` (bits15:12) =
    /// `RdLo`, mirroring `Mul`. Flags are never touched, though `Q` may be
    /// sticky-set.
    DualMul {
        op: DualMulOp,
        rd: u8,
        rn: u8,
        rm: u8,
        ra: u8,
    },
    /// ARMv7-M unary register data-processing: `REV`/`REV16`/`REVSH`,
    /// `RBIT`, and `CLZ`. Flags are not affected.
    Unary {
        op: UnaryOp,
        rd: u8,
        rm: u8,
    },
    /// ARMv7-M saturating add/subtract: `QADD`/`QSUB` and the doubling
    /// `QDADD`/`QDSUB`. The result is clamped to the signed 32-bit range and
    /// the sticky APSR.Q flag is set when clamping happens. Flags are
    /// otherwise not affected.
    Qadd {
        op: QaddOp,
        rd: u8,
        rn: u8,
        rm: u8,
    },
    /// ARMv7-M saturate: `SSAT`/`USAT` (optionally shifted) and their 16-bit
    /// `SSAT16`/`USAT16` forms. `sat_n` is the `sat_imm + 1` bit width of the
    /// target range; `shift`/`sh` apply only to the full-word forms. Sets
    /// APSR.Q on saturation. Flags are otherwise not affected.
    Sat {
        op: SatOp,
        rd: u8,
        rn: u8,
        sat_n: u8,
        sh: bool,
        shift: u8,
    },
    /// ARMv7-M bitfield extract/insert: `SBFX`/`UBFX`/`BFI`/`BFC`. `field`
    /// is the `widthm1` for the extracts (width `field + 1`) or the `msb`
    /// for the inserts; `rn` is the source register, or `15` for the `BFC`
    /// clear form. Flags are not affected.
    Bitfield {
        op: BitfieldOp,
        rd: u8,
        rn: u8,
        lsb: u8,
        field: u8,
    },
    /// ARMv7-M parallel add/subtract: the `S`/`Q`/`SH`/`U`/`UQ`/`UH` forms of
    /// `ADD8`/`ADD16`/`SUB8`/`SUB16`/`ASX`/`SAX`. Each lane of `Rd` is the
    /// wrapped, halved, or saturated lane result; only the `S` and `U` forms
    /// write the per-lane GE flags. The per-lane saturating forms never touch
    /// the sticky `APSR.Q` flag.
    Par {
        op: ParOp,
        sat: ParSat,
        rd: u8,
        rn: u8,
        rm: u8,
    },
    /// ARMv7-M `SEL Rd, Rn, Rm`: byte lane `i` of `Rd` comes from `Rn` when
    /// GE flag `i` is set, and from `Rm` otherwise. Flags are not affected.
    Sel {
        rd: u8,
        rn: u8,
        rm: u8,
    },
    /// ARMv7-M register-extend: `SXTAH`/`UXTAH`/`SXTAB16`/`UXTAB16`/`SXTAB`/
    /// `UXTAB`. `rot` is the `ROR #rot*8` amount; `rn == 15` selects the
    /// two-operand `SXTH`/`UXTH`/`SXTB16`/`UXTB16`/`SXTB`/`UXTB` forms, which
    /// the executor folds by skipping the addend. Flags are not affected.
    ExtendAdd {
        op: ExtendAddOp,
        rd: u8,
        rn: u8,
        rm: u8,
        rot: u8,
    },
}

/// ARMv7-M multiply-family opcode. The long forms read their `RdHi:RdLo`
/// pair as the accumulator and write the 64-bit result back to the same pair;
/// the plain forms keep the low 32 bits of the product.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MulOp {
    /// `MUL Rd, Rn, Rm`: `Rd = Rn * Rm`.
    Mul,
    /// `MLA Rd, Rn, Rm, Ra`: `Rd = Rn * Rm + Ra`.
    Mla,
    /// `MLS Rd, Rn, Rm, Ra`: `Rd = Ra - Rn * Rm`.
    Mls,
    /// `SMULL RdLo, RdHi, Rn, Rm`: signed 64-bit product.
    Smull,
    /// `UMULL RdLo, RdHi, Rn, Rm`: unsigned 64-bit product.
    Umull,
    /// `SMLAL RdLo, RdHi, Rn, Rm`: signed accumulate.
    Smlal,
    /// `UMLAL RdLo, RdHi, Rn, Rm`: unsigned accumulate.
    Umlal,
}

/// ARMv7-M DSP multiply-family opcode. The halfword forms select the 16-bit
/// halves of `Rn` (`x`) and `Rm` (`y`); `SMULWB`/`SMULWT` multiply the full
/// signed 32-bit `Rn` by the selected halfword of `Rm` and keep the top 32
/// bits of the 48-bit product; `SMMUL`/`SMMULR` keep the top word of the full
/// 32-by-32 product; `USAD8`/`USADA8` sum the absolute byte differences (with
/// an accumulate operand for the `A8` form). None of them touch the flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DspMulOp {
    /// `SMULBB Rd, Rn, Rm`: `Rn[15:0] * Rm[15:0]`.
    Smulbb,
    /// `SMULBT Rd, Rn, Rm`: `Rn[15:0] * Rm[31:16]`.
    Smulbt,
    /// `SMULTB Rd, Rn, Rm`: `Rn[31:16] * Rm[15:0]`.
    Smultb,
    /// `SMULTT Rd, Rn, Rm`: `Rn[31:16] * Rm[31:16]`.
    Smultt,
    /// `SMULWB Rd, Rn, Rm`: `(Rn * Rm[15:0]) >> 16`.
    Smulwb,
    /// `SMULWT Rd, Rn, Rm`: `(Rn * Rm[31:16]) >> 16`.
    Smulwt,
    /// `SMMUL Rd, Rn, Rm`: `(Rn * Rm) >> 32`.
    Smmul,
    /// `SMMULR Rd, Rn, Rm`: `SMMUL` rounded by adding `0x8000_0000`.
    Smmulr,
    /// `USAD8 Rd, Rn, Rm`: sum of the four absolute byte differences.
    Usad8,
    /// `USADA8 Rd, Rn, Rm, Ra`: `USAD8` plus `Ra`.
    Usada8,
}

/// ARMv7-M dual multiply family, from the `1111 1011` multiply zone.
///
/// `SMLAD`/`SMLSD` (and the `X` variants, which swap `Rm`'s halves) form the
/// two halfword products and add or subtract them, then add `Ra`; with `Ra`
/// all-ones they become `SMUAD`/`SMUSD` and drop the accumulate. `SMLABB`..
/// `SMLATT` and `SMLAWB`/`SMLAWT` accumulate a single 16-bit (or 16-bit-of-
/// 32-bit) product with `Ra`. `SMMLA`/`SMMLS` keep the top word of a 32-by-32
/// product (subtracting it from `Ra << 32` for `SMMLS`). The long forms
/// `SMLALBB`..`SMLALTT`, `SMLALD`, `SMLSLD` and `UMAAL` write `RdLo:RdHi`
/// (bits15:12 / bits11:8) and add their products to a 64-bit accumulator.
/// `SMLAD`/`SMLSD`/`SMLABB`/`SMLAWB`/`SMMLA` set the sticky `Q` flag on
/// overflow; the long forms and `UMAAL` never touch it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DualMulOp {
    /// `SMLABB Rd, Rn, Rm, Ra`: `Ra + Rn[15:0] * Rm[15:0]`.
    Smlabb,
    /// `SMLABT Rd, Rn, Rm, Ra`: `Ra + Rn[15:0] * Rm[31:16]`.
    Smlabt,
    /// `SMLATB Rd, Rn, Rm, Ra`: `Ra + Rn[31:16] * Rm[15:0]`.
    Smlatb,
    /// `SMLATT Rd, Rn, Rm, Ra`: `Ra + Rn[31:16] * Rm[31:16]`.
    Smlatt,
    /// `SMLAWB Rd, Rn, Rm, Ra`: `Ra + (Rn * Rm[15:0]) >> 16`.
    Smlawb,
    /// `SMLAWT Rd, Rn, Rm, Ra`: `Ra + (Rn * Rm[31:16]) >> 16`.
    Smlawt,
    /// `SMLAD Rd, Rn, Rm, Ra`: `Ra + p1 + p2` (also `SMUAD` when `Ra == 15`).
    Smlad,
    /// `SMLADX Rd, Rn, Rm, Ra`: `SMLAD` with `Rm`'s halves swapped.
    Smladx,
    /// `SMLSD Rd, Rn, Rm, Ra`: `Ra + p1 - p2` (also `SMUSD` when `Ra == 15`).
    Smlsd,
    /// `SMLSDX Rd, Rn, Rm, Ra`: `SMLSD` with `Rm`'s halves swapped.
    Smlsdx,
    /// `SMMLA Rd, Rn, Rm, Ra`: `Ra + (Rn * Rm) >> 32`.
    Smmla,
    /// `SMMLAR Rd, Rn, Rm, Ra`: `SMMLA` rounded by the `Rm * Rn` low bit 31.
    Smmlar,
    /// `SMMLS Rd, Rn, Rm, Ra`: `(Ra << 32 - Rn * Rm) >> 32`.
    Smmls,
    /// `SMMLSR Rd, Rn, Rm, Ra`: `SMMLS` rounded by the product low bit 31.
    Smmlsr,
    /// `SMLALD RdLo, RdHi, Rn, Rm`: `p1 + p2` accumulated, signed 64-bit.
    Smlald,
    /// `SMLALDX RdLo, RdHi, Rn, Rm`: `SMLALD` with `Rm`'s halves swapped.
    Smlaldx,
    /// `SMLSLD RdLo, RdHi, Rn, Rm`: `p1 - p2` accumulated, signed 64-bit.
    Smlsld,
    /// `SMLSLDX RdLo, RdHi, Rn, Rm`: `SMLSLD` with `Rm`'s halves swapped.
    Smlsldx,
    /// `SMLALBB RdLo, RdHi, Rn, Rm`: accumulate `Rn[15:0] * Rm[15:0]`.
    Smlalbb,
    /// `SMLALBT RdLo, RdHi, Rn, Rm`: accumulate `Rn[15:0] * Rm[31:16]`.
    Smlalbt,
    /// `SMLALTB RdLo, RdHi, Rn, Rm`: accumulate `Rn[31:16] * Rm[15:0]`.
    Smlaltb,
    /// `SMLALTT RdLo, RdHi, Rn, Rm`: accumulate `Rn[31:16] * Rm[31:16]`.
    Smlaltt,
    /// `UMAAL RdLo, RdHi, Rn, Rm`: unsigned product plus `RdHi` and `RdLo`.
    Umaal,
}

/// ARMv7-M unary register-only data-processing opcode, from the `1111 1010`
/// misc group. Each reads `Rm` and writes `Rd`; the flags are never touched.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UnaryOp {
    /// `REV Rd, Rm`: reverse the byte order of a word.
    Rev,
    /// `REV16 Rd, Rm`: reverse the byte order of each halfword.
    Rev16,
    /// `REVSH Rd, Rm`: reverse the low halfword's bytes and sign-extend.
    Revsh,
    /// `RBIT Rd, Rm`: reverse the bit order of a word.
    Rbit,
    /// `CLZ Rd, Rm`: count leading zeros (`32` when `Rm == 0`).
    Clz,
}

/// ARMv7-M saturating add/subtract opcode (`QADD`/`QSUB`/`QDADD`/`QDSUB`).
///
/// `QADD`/`QSUB` clamp the plain sum or difference; `QDADD`/`QDSUB` clamp the
/// doubled second operand first and then clamp the sum/difference with it. The
/// sticky Q flag is set if either clamp engages.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QaddOp {
    /// `QADD Rd, Rn, Rm`: `Rd = Sat(Rn + Rm)`.
    Qadd,
    /// `QDADD Rd, Rn, Rm`: `Rd = Sat(Rn + Sat(Rm * 2))`.
    Qdadd,
    /// `QSUB Rd, Rn, Rm`: `Rd = Sat(Rn - Rm)`.
    Qsub,
    /// `QDSUB Rd, Rn, Rm`: `Rd = Sat(Rn - Sat(Rm * 2))`.
    Qdsub,
}

/// ARMv7-M saturate opcode (`SSAT`/`USAT`/`SSAT16`/`USAT16`).
///
/// `SSAT`/`USAT` saturate a (shifted) 32-bit word to a signed/unsigned
/// `sat_n`-bit range; `SSAT16`/`USAT16` saturate each halfword independently.
/// Saturation sets the sticky Q flag.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SatOp {
    /// `SSAT Rd, #sat, Rn {, shift}`: signed saturate.
    Ssat,
    /// `USAT Rd, #sat, Rn {, shift}`: unsigned saturate.
    Usat,
    /// `SSAT16 Rd, #sat, Rn`: per-halfword signed saturate.
    Ssat16,
    /// `USAT16 Rd, #sat, Rn`: per-halfword unsigned saturate.
    Usat16,
}

/// ARMv7-M bitfield-family opcode. `SBFX`/`UBFX` extract `field + 1` bits of
/// `Rn` from `lsb` (sign- or zero-extending the result); `BFI` deposits `Rn`'s
/// low `field - lsb + 1` bits into `Rd[field:lsb]`; `BFC` (the `Rn == 15`
/// form of `BFI`) clears that same window. None of them touch the flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitfieldOp {
    /// `SBFX Rd, Rn, #lsb, #width`: signed bitfield extract.
    Sbfx,
    /// `UBFX Rd, Rn, #lsb, #width`: unsigned bitfield extract.
    Ubfx,
    /// `BFI Rd, Rn, #lsb, #width`: bitfield insert.
    Bfi,
    /// `BFC Rd, #lsb, #width`: bitfield clear.
    Bfc,
}

/// ARMv7-M parallel add/subtract families: `ADD8`, `ADD16`, `SUB8`, `SUB16`,
/// `ASX`, and `SAX`.
///
/// The byte/halfword families work lane-wise over `Rn` and `Rm`; `ASX`/`SAX`
/// exchange the second operand's halves (`ASX` subtracts the swapped high half
/// from the low lane and adds the swapped low half to the high lane; `SAX`
/// mirrors the operations).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParOp {
    /// `ADD8 Rd, Rn, Rm`: per-byte add.
    Add8,
    /// `ADD16 Rd, Rn, Rm`: per-halfword add.
    Add16,
    /// `SUB8 Rd, Rn, Rm`: per-byte subtract.
    Sub8,
    /// `SUB16 Rd, Rn, Rm`: per-halfword subtract.
    Sub16,
    /// `ASX Rd, Rn, Rm`: halfword subtract/add with the operands exchanged.
    Asx,
    /// `SAX Rd, Rn, Rm`: halfword add/subtract with the operands exchanged.
    Sax,
}

/// ARMv7-M parallel add/subtract saturation selector (`S`/`Q`/`SH`/`U`/`UQ`/`UH`).
///
/// `S`/`U` wrap silently and write the per-lane GE flags; `Q`/`UQ` saturate
/// each lane to the signed/unsigned `bits`-bit range; `SH`/`UH` halve each
/// lane's exact result. The saturating forms do not set the sticky Q flag.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParSat {
    /// Signed wrapping, writes GE.
    S,
    /// Signed saturating.
    Q,
    /// Signed halving.
    Sh,
    /// Unsigned wrapping, writes GE.
    U,
    /// Unsigned saturating.
    Uq,
    /// Unsigned halving.
    Uh,
}

/// ARMv7-M register-extend family: `SXTAH`/`UXTAH`/`SXTAB16`/`UXTAB16`/
/// `SXTAB`/`UXTAB`.
///
/// Each rotates `Rm` right by `rot * 8` bits, extends the selected byte (both
/// bytes for the `16` forms) or halfword, and adds `Rn`. Flags are not
/// affected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtendAddOp {
    /// `SXTAH Rd, Rn, Rm {, ROR #rot}`: sign-extend the halfword, add `Rn`.
    Sxtah,
    /// `UXTAH Rd, Rn, Rm {, ROR #rot}`: zero-extend the halfword, add `Rn`.
    Uxtah,
    /// `SXTAB16 Rd, Rn, Rm {, ROR #rot}`: sign-extend each byte, per-halfword
    /// add `Rn`.
    Sxtab16,
    /// `UXTAB16 Rd, Rn, Rm {, ROR #rot}`: zero-extend each byte,
    /// per-halfword add `Rn`.
    Uxtab16,
    /// `SXTAB Rd, Rn, Rm {, ROR #rot}`: sign-extend the byte, add `Rn`.
    Sxtab,
    /// `UXTAB Rd, Rn, Rm {, ROR #rot}`: zero-extend the byte, add `Rn`.
    Uxtab,
}

/// True when `hw` is the first halfword of a 32-bit ARMv6-M instruction.
/// The first halfword always starts `11110` (0xF000-0xF7FF), which no 16-bit
/// encoding uses.
#[inline]
#[must_use]
pub const fn is_thumb2(hw: u16) -> bool {
    hw & 0xF800 == 0xF000
}

/// True when `hw` is the first halfword of a 32-bit ARMv7-M instruction.
/// The full Thumb-2 (T32) encoding space covers 0xE800-0xFFFF; the 16-bit
/// encodings stop below it.
#[inline]
#[must_use]
pub const fn is_thumb2_v7m(hw: u16) -> bool {
    hw >= 0xE800
}

/// Expand a Thumb-2 modified-immediate `imm12` (`i:imm3:imm8`) into its
/// 32-bit constant, per `ThumbExpandImm()`.
///
/// The unrotated cases produce byte-replication shapes; the rotated ones place
/// a value with bit 7 set at any byte-aligned position. The executor folds the
/// rotated cases' bit-31 carry into `C` when a flag-setting logical op writes
/// the flags.
#[must_use]
pub fn thumb_expand_imm(imm12: u16) -> u32 {
    let imm12 = imm12 & 0xFFF;
    let imm8 = u32::from(imm12 & 0xFF);
    if imm12 & 0xC00 == 0 {
        // imm12<11:10> == 00: no rotation; imm8 is placed in one of four
        // byte positions (or repeated).
        match (imm12 >> 8) & 0b11 {
            0b00 => imm8,
            0b01 => (imm8 << 16) | imm8,
            0b10 => (imm8 << 24) | (imm8 << 8),
            _ => imm8 * 0x0101_0101,
        }
    } else {
        // Rotate a value with bit 7 set by imm12<11:7> (0..=31): the low byte
        // loses its top bit, which is implied to be 1, so an 8-bit pattern
        // starting with a 1 can be rotated to any byte-aligned position. A
        // rotation by zero is the identity.
        let unrotated = (1u32 << 7) | u32::from(imm12 & 0x7F);
        unrotated.rotate_right(u32::from((imm12 >> 7) & 0x1F))
    }
}

/// Decode the two halfwords of a 32-bit ARMv6-M instruction.
///
/// # Errors
///
/// Returns [`DecodeError`] when the halfwords do not form a known ARMv6-M
/// 32-bit instruction.
pub fn decode32(hi: u16, lo: u16) -> Result<Inst32, DecodeError> {
    decode_mrs(hi, lo)
        .or_else(|| decode_msr(hi, lo))
        .or_else(|| decode_barrier(hi, lo))
        .or_else(|| decode_bl(hi, lo))
        .unwrap_or_else(|| Err(err_for(hi)))
}

/// Decode the two halfwords of a 32-bit ARMv7-M instruction.
///
/// The ARMv6-M subset is preserved; `MOVW`/`MOVT`, the Thumb-2
/// data-processing set (shifted register, `ThumbExpandImm`, and the raw
/// `addw`/`subw` immediates), the immediate-offset load/store family, and the
/// `B.W`/`B<c>.W`/`BLX` immediate branches are layered on top. The memory
/// decoders are consulted before the data-processing set: their `F8xx`/`F9xx`
/// first halfwords overlap the modified-immediate mask (`hi & 0xF600`) and
/// must win the match.
///
/// # Errors
///
/// Returns [`DecodeError`] when the halfwords do not form a known ARMv7-M
/// 32-bit instruction.
pub fn decode32_v7m(hi: u16, lo: u16) -> Result<Inst32, DecodeError> {
    decode_mrs(hi, lo)
        .or_else(|| decode_msr(hi, lo))
        .or_else(|| decode_clrex(hi, lo))
        .or_else(|| decode_barrier(hi, lo))
        .or_else(|| decode_branch(hi, lo))
        .or_else(|| decode_movwide(hi, lo))
        .or_else(|| decode_mem(hi, lo))
        .or_else(|| decode_ldm_stm(hi, lo))
        .or_else(|| decode_exclusive(hi, lo))
        .or_else(|| decode_tbranch(hi, lo))
        .or_else(|| decode_ldrd_strd(hi, lo))
        .or_else(|| decode_dp_plain(hi, lo))
        .or_else(|| decode_dp_modified(hi, lo))
        .or_else(|| decode_dp_reg(hi, lo))
        .or_else(|| decode_div(hi, lo))
        .or_else(|| decode_mul(hi, lo))
        .or_else(|| decode_dsp_mul(hi, lo))
        .or_else(|| decode_dual_mul(hi, lo))
        .or_else(|| decode_extend(hi, lo))
        .or_else(|| decode_par_addsub(hi, lo))
        .or_else(|| decode_qadd(hi, lo))
        .or_else(|| decode_unary(hi, lo))
        .or_else(|| decode_bitfield(hi, lo))
        .or_else(|| decode_sat(hi, lo))
        .unwrap_or_else(|| Err(err_for(hi)))
}

/// Build the typed undefined-instruction error for a first halfword.
const fn err_for(hi: u16) -> DecodeError {
    DecodeError {
        opcode: hi,
        cause: HaltCause::UndefinedInstruction,
    }
}

/// Decode the ARMv6-M `BL` immediate only.
fn decode_bl(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // BL: 11110 S imm10 / 11 J1 1 J2 imm11.
    if hi & 0xF800 != 0xF000 {
        return None;
    }
    if lo & 0xD000 != 0xD000 {
        // The `11110` first-halfword space is shared with the Thumb-2
        // data-processing set; only a `11 J1 1 J2`-shaped second halfword
        // (bits 15, 14, 12 set) is BL.
        return None;
    }
    let s = i32::from((hi >> 10) & 1);
    let imm10 = i32::from(hi & 0x3FF);
    let imm11 = i32::from(lo & 0x7FF);
    Some(Ok(Inst32::Bl {
        off: branch_off25(s, imm10, branch_i(lo, 13, s), branch_i(lo, 11, s), imm11),
    }))
}

/// Decode the shared ARMv7-M branch family of the `11110` first-halfword
/// space. The second halfword splits it: `B.W` (T4) and `BL` (T1) share the
/// 25-bit `imm10:imm11` field and differ only in bit 14; `BLX` (T2) swaps it
/// for the word-aligned `imm10H:imm10L`; `B<c>.W` (T3) uses a 21-bit
/// `imm6:imm11` offset with the condition in the first halfword.
fn decode_branch(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    if hi & 0xF800 != 0xF000 {
        return None;
    }
    let s = i32::from((hi >> 10) & 1);
    let i1 = branch_i(lo, 13, s);
    let i2 = branch_i(lo, 11, s);
    match lo & 0xD000 {
        // B.W (T4): 11110 S imm10 / 10 J1 1 J2 imm11.
        0x9000 => {
            let off = branch_off25(s, i32::from(hi & 0x3FF), i1, i2, i32::from(lo & 0x7FF));
            Some(Ok(Inst32::BranchWide { off }))
        }
        // BL (T1): 11110 S imm10 / 11 J1 1 J2 imm11.
        0xD000 => {
            let off = branch_off25(s, i32::from(hi & 0x3FF), i1, i2, i32::from(lo & 0x7FF));
            Some(Ok(Inst32::Bl { off }))
        }
        // BLX (T2): 11110 S imm10H / 11 J1 0 J2 imm10L.
        0xC000 => {
            let off = branch_off25(
                s,
                i32::from(hi & 0x3FF),
                i1,
                i2,
                i32::from((lo >> 1) & 0x3FF) << 1,
            );
            Some(Ok(Inst32::BlxImm { off }))
        }
        // B<c>.W (T3): 11110 S cond imm6 / 10 J1 0 J2 imm11. The `cond == 1111`
        // encoding is UNPREDICTABLE.
        0x8000 => {
            let cond = ((hi >> 6) & 0xF) as u8;
            if cond == 0xF {
                return Some(Err(err_for(hi)));
            }
            let off = branch_off21(s, i1, i2, i32::from(hi & 0x3F), i32::from(lo & 0x7FF));
            Some(Ok(Inst32::BranchCondWide { cond, off }))
        }
        _ => None,
    }
}

/// `I1`/`I2` for a branch: `NOT(J EOR S)`, where `J` is bit `bit` of the
/// second halfword.
#[inline]
fn branch_i(lo: u16, bit: u8, s: i32) -> i32 {
    1 - ((i32::from((lo >> bit) & 1)) ^ s)
}

/// Reconstruct the 25-bit signed byte offset `S:imm10:I1:I2:imm11:0`,
/// shift-sign-extended from bit 24 (shifts wrap, so this is well-defined for
/// the whole range).
#[inline]
const fn branch_off25(s: i32, imm10: i32, i1: i32, i2: i32, imm11: i32) -> i32 {
    ((s << 24) | (imm10 << 14) | (i1 << 13) | (i2 << 12) | (imm11 << 1)) << 7 >> 7
}

/// Reconstruct the 21-bit signed byte offset `S:I1:I2:imm6:imm11:0`,
/// shift-sign-extended from bit 20.
#[inline]
const fn branch_off21(s: i32, i1: i32, i2: i32, imm6: i32, imm11: i32) -> i32 {
    ((s << 20) | (i1 << 19) | (i2 << 18) | (imm6 << 12) | (imm11 << 1)) << 11 >> 11
}

/// decode the ARMv6/7-M `MRS`.
fn decode_mrs(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // MRS: 1111 0011 1110 1111 1000 Rd sys.
    if hi != 0xF3EF {
        return None;
    }
    if lo & 0xF000 != 0x8000 {
        return Some(Err(err_for(hi)));
    }
    let rd = ((lo >> 8) & 0xF) as u8;
    Some(Ok(Inst32::Mrs {
        rd,
        sys: SysReg::from((lo & 0xFF) as u8),
    }))
}

/// decode the ARMv6/7-M `MSR`.
fn decode_msr(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // MSR: 1111 0011 1000 Rn 1000 sys.
    if hi & 0xFFF0 != 0xF380 {
        return None;
    }
    // The `1111 0011 10..` zone also hosts `USAT`, whose second halfword
    // starts with a fixed zero (bit 15). MSR needs `1000` there, so a clear
    // bit 15 is not ours to reject.
    if lo & 0x8000 == 0 {
        return None;
    }
    if lo & 0xF000 != 0x8000 {
        return Some(Err(err_for(hi)));
    }
    let rn = (hi & 0xF) as u8;
    Some(Ok(Inst32::Msr {
        rn,
        sys: SysReg::from((lo & 0xFF) as u8),
    }))
}

/// decode the ARMv6/7-M `DMB`/`DSB`/`ISB`.
fn decode_barrier(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // DMB/DSB/ISB: 1111 0011 1011 1111 1000 1111 op option.
    if hi != 0xF3BF {
        return None;
    }
    // `CLREX` shares the first halfword; it is claimed by `decode_clrex` on
    // the ARMv7-M path and must fall through (to undefined) on ARMv6-M.
    if lo == 0x8F2F {
        return None;
    }
    if lo & 0xF000 != 0x8000 {
        return Some(Err(err_for(hi)));
    }
    let kind = match (lo >> 4) & 0xF {
        0x4 => Barrier::Dsb,
        0x5 => Barrier::Dmb,
        0x6 => Barrier::Isb,
        _ => return Some(Err(err_for(hi))),
    };
    Some(Ok(Inst32::Barrier { kind }))
}

/// decode the ARMv7-M `CLREX`. Its first halfword (`0xF3BF`) is shared with
/// the barrier family; this decoder runs first on the ARMv7-M path.
fn decode_clrex(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    if hi == 0xF3BF && lo == 0x8F2F {
        Some(Ok(Inst32::Clrex))
    } else {
        None
    }
}

/// decode the ARMv7-M exclusive load/store family: the word `LDREX`/`STREX`
/// and the `LDREX{B,H}`/`STREX{B,H}` forms.
///
/// First-halfword bits [7:4] name the instruction: `0100` the word `STREX`,
/// `0101` the word `LDREX`, `1100` the `STREX{B,H}`, `1101` the
/// `LDREX{B,H}`. The word forms carry `Rt Rd imm8` / `Rt 1111 imm8` in the
/// second halfword (offset `imm8 << 2`); the byte/halfword forms fix
/// second-halfword bits [11:8] = `1111` and put the width selector in bits
/// [7:4] (`0100` byte, `0101` halfword, `0111` doubleword — the doubleword
/// forms are not valid on M-profile). The `0xE8C0`/`0xE8D0` spaces are
/// shared with `STRD`/`LDRD` (bits [11:8] = `Rt2`) and `TBB`/`TBH`, so a
/// second halfword without the `1111` tag is handed back to those decoders.
fn decode_exclusive(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let rn = (hi & 0xF) as u8;
    let rt = ((lo >> 12) & 0xF) as u8;
    let field = ((lo >> 8) & 0xF) as u8;
    let err = || Some(Err(err_for(hi)));
    match hi & 0xFFF0 {
        0xE840 => {
            // STREX (word): Rt Rd imm8. `Rt == 1111` is the ARMv8-M `TT`
            // family, not valid here.
            if rt == 15 || field == 15 || rn == 15 {
                return err();
            }
            Some(Ok(Inst32::Strex {
                rd: field,
                rt,
                rn,
                size: ExSize::Word,
                off: ((lo & 0xFF) as u32) << 2,
            }))
        }
        0xE850 => {
            // LDREX (word): Rt 1111 imm8.
            if rt == 15 || field != 15 || rn == 15 {
                return err();
            }
            Some(Ok(Inst32::Ldrex {
                rt,
                rn,
                size: ExSize::Word,
                off: ((lo & 0xFF) as u32) << 2,
            }))
        }
        0xE8C0 | 0xE8D0 => {
            let is_load = hi & 0x10 != 0;
            // Without the `1111` tag this second halfword is a `STRD`
            // (stores) or `LDRD` (loads) register pair.
            if field != 15 {
                return None;
            }
            let size = match (lo >> 4) & 0xF {
                0x4 => ExSize::Byte,
                0x5 => ExSize::Half,
                // The `0111` doubleword forms (and the ARMv8 acquire /
                // release family) are not valid on M-profile.
                _ => return err(),
            };
            if is_load {
                if lo & 0xF != 0xF || rt == 15 || rn == 15 {
                    return err();
                }
                Some(Ok(Inst32::Ldrex {
                    rt,
                    rn,
                    size,
                    off: 0,
                }))
            } else {
                let rd = (lo & 0xF) as u8;
                if rd == 15 || rt == 15 || rn == 15 {
                    return err();
                }
                Some(Ok(Inst32::Strex {
                    rd,
                    rt,
                    rn,
                    size,
                    off: 0,
                }))
            }
        }
        _ => None,
    }
}

/// decode the ARMv7-M `MOVW`/`MOVT`.
fn decode_movwide(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // MOVW/MOVT: 1111 0 i 1 0 0 1 0 0 insn / 0 imm4 Rd imm3 imm8, where bit7
    // of the first halfword is 0 for MOVW and 1 for MOVT.
    if hi & 0xFB70 != 0xF240 {
        return None;
    }
    if lo & 0x8000 != 0 {
        return Some(Err(err_for(hi)));
    }
    let i = (hi >> 10) & 1;
    let high = hi & 0x80 != 0;
    let rd = ((lo >> 8) & 0xF) as u8;
    let imm = (hi & 0xF) << 12 | i << 11 | ((lo >> 12) & 0x7) << 8 | lo & 0xFF;
    Some(Ok(Inst32::MovWide { rd, imm, high }))
}

/// decode the ARMv7-M single-data-transfer family.
///
/// Three halfword layouts share the `1111 100x` first halfword: the unsigned
/// forms live at `F8xx`, the sign-extending loads at `F9xx` (`bit8` is the
/// sign bit). Each size/op has an imm12 form (offset only, second-halfword
/// `Rt imm12`), an imm8 form (off-, pre-, post-indexed, second-halfword
/// `Rt 1 P U W imm8`), and a shifted-register form (second-halfword
/// `Rt 000000 shimm Rm`). `Rn == 15` in any form is the PC-relative literal
/// (loads only).
fn decode_mem(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let (access, wide) = match hi & 0xFFF0 {
        0xF8C0 => (MemOp::Str, true),
        0xF8D0 => (MemOp::Ldr, true),
        0xF880 => (MemOp::Strb, true),
        0xF890 => (MemOp::Ldrb, true),
        0xF8A0 => (MemOp::Strh, true),
        0xF8B0 => (MemOp::Ldrh, true),
        0xF990 => (MemOp::Ldrsb, true),
        0xF9B0 => (MemOp::Ldrsh, true),
        0xF840 => (MemOp::Str, false),
        0xF850 => (MemOp::Ldr, false),
        0xF800 => (MemOp::Strb, false),
        0xF810 => (MemOp::Ldrb, false),
        0xF820 => (MemOp::Strh, false),
        0xF830 => (MemOp::Ldrh, false),
        0xF910 => (MemOp::Ldrsb, false),
        0xF930 => (MemOp::Ldrsh, false),
        _ => return None,
    };
    let rn = (hi & 0xF) as u8;
    let is_load = matches!(
        access,
        MemOp::Ldr | MemOp::Ldrb | MemOp::Ldrh | MemOp::Ldrsb | MemOp::Ldrsh
    );
    let err = || Some(Err(err_for(hi)));
    // The literal load patterns (`Rn == 15`) win the whole single-load space
    // regardless of the second halfword, so a PC base always means the imm12
    // literal form. Stores with a PC base are UNPREDICTABLE.
    if rn == 15 {
        if !is_load {
            return err();
        }
        let rt = (lo >> 12) as u8;
        return Some(Ok(Inst32::Mem {
            access,
            rt,
            rn: 15,
            off: u32::from(lo & 0xFFF),
            rm: None,
            shift: 0,
            add: true,
            index: true,
            writeback: false,
        }));
    }
    if wide {
        let rt = (lo >> 12) as u8;
        if !is_load && rt == 15 {
            // STR to PC, or with a PC base, is architecturally UNPREDICTABLE.
            return err();
        }
        Some(Ok(Inst32::Mem {
            access,
            rt,
            rn,
            off: u32::from(lo & 0xFFF),
            rm: None,
            shift: 0,
            add: true,
            index: true,
            writeback: false,
        }))
    } else if lo & 0x0800 != 0 {
        // The immediate form's second halfword is `Rt 1 P U W imm8`: bit 11
        // is the fixed one that selects it over the register form below.
        let p = lo & 0x0400 != 0;
        let u = lo & 0x0200 != 0;
        let w = lo & 0x0100 != 0;
        let rt = (lo >> 12) as u8;
        let off = u32::from(lo & 0xFF);
        if !is_load && rt == 15 {
            // STR to PC is architecturally UNPREDICTABLE.
            return err();
        }
        if p && u && !w {
            // The unprivileged LDRT/STRT form is UNPREDICTABLE on ARMv7-M.
            return err();
        }
        if !p && !w {
            return err();
        }
        let (index, writeback) = match (p, w) {
            (true, true) => (true, true),
            (false, true) => (false, true),
            (true, false) => (true, false),
            _ => unreachable!(),
        };
        Some(Ok(Inst32::Mem {
            access,
            rt,
            rn,
            off,
            rm: None,
            shift: 0,
            add: u,
            index,
            writeback,
        }))
    } else if lo & 0x0FC0 == 0 {
        // The register-offset form's second halfword is `Rt 000000 shimm Rm`:
        // bits 11..6 are all zero, and the offset is `Rm << (shimm * 2)`
        // (LSL only, 0/2/4/6). It always adds (`U == 1`) with no write-back.
        let rt = (lo >> 12) as u8;
        let rm = (lo & 0xF) as u8;
        let shift = (((lo >> 4) & 0x3) as u8) * 2;
        if !is_load && rt == 15 {
            return err();
        }
        Some(Ok(Inst32::Mem {
            access,
            rt,
            rn,
            off: 0,
            rm: Some(rm),
            shift,
            add: true,
            index: true,
            writeback: false,
        }))
    } else {
        // A second halfword with bit 11 clear but bits 11..6 non-zero is
        // not a valid form of these encodings.
        err()
    }
}

/// decode the ARMv7-M load/store multiple family: `STMIA`, `LDMIA`,
/// `STMDB`, `LDMDB`, from the `1110 1000 10wX rn` (IA) and
/// `1110 1001 00wX rn` (DB) first-halfword spaces, where `X` is the load bit
/// and `w` the write-back. The second halfword is the 16-bit register list.
fn decode_ldm_stm(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let (is_load, dec_before) = match hi & 0xFFD0 {
        0xE880 => (false, false),
        0xE890 => (true, false),
        0xE900 => (false, true),
        0xE910 => (true, true),
        _ => return None,
    };
    let writeback = hi & 0x20 != 0;
    let rn = (hi & 0xF) as u8;
    let list = lo;
    let err = || Some(Err(err_for(hi)));
    if list == 0 {
        // An empty register list is UNPREDICTABLE.
        return err();
    }
    if !is_load {
        // STM with a PC base, or storing to PC, is UNPREDICTABLE.
        if rn == 15 || list & 0x8000 != 0 {
            return err();
        }
    } else if rn == 15 {
        // LDM with a PC base is UNDEFINED on M-profile.
        return err();
    } else if writeback && list & (1u16 << rn) != 0 && u16::from(rn) != list.trailing_zeros() as u16
    {
        // A write-back with rn in the list is UNPREDICTABLE unless rn is the
        // lowest-numbered register in the list (the loaded value wins).
        return err();
    }
    Some(Ok(Inst32::LdmStm {
        is_load,
        rn,
        regs: list,
        writeback,
        dec_before,
    }))
}

/// Map the ARMv7-M data-processing opcode field, plus the `Rd`/`Rn` aliases
/// that turn AND/ORR etc. into the flag-only or `MOV`/`MVN` forms.
const fn dp_op(family_add: bool, op3: u8, rn: u8, rd: u8, set_flags: bool) -> Option<DpOp> {
    let test = set_flags && rd == 15;
    if family_add {
        match op3 {
            0 => Some(if test { DpOp::Cmn } else { DpOp::Add }),
            2 => Some(DpOp::Adc),
            3 => Some(DpOp::Sbc),
            5 => Some(if test { DpOp::Cmp } else { DpOp::Sub }),
            6 => Some(DpOp::Rsb),
            _ => None,
        }
    } else {
        match op3 {
            0 => Some(if test { DpOp::Tst } else { DpOp::And }),
            1 => Some(DpOp::Bic),
            2 => Some(if rn == 15 { DpOp::Mov } else { DpOp::Orr }),
            3 => Some(if rn == 15 { DpOp::Mvn } else { DpOp::Orn }),
            4 => Some(if test { DpOp::Teq } else { DpOp::Eor }),
            _ => None,
        }
    }
}

/// Assemble the ARMv7-M data-processing `imm12` (`i:imm3:imm8`).
const fn dp_imm12(hi: u16, lo: u16) -> u16 {
    ((hi >> 10) & 1) << 11 | ((lo >> 12) & 0x7) << 8 | lo & 0xFF
}

/// decode the ARMv7-M plain-binary-immediate `addw`/`subw`.
fn decode_dp_plain(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // 1111 0 i 1 0 op S Rn / 0 imm3 Rd imm8, op = 000 (ADD) or 101 (SUB),
    // imm12 used as a raw 12-bit value.
    if hi & 0xFB40 != 0xF200 {
        return None;
    }
    let op = match (hi >> 5) & 0x7 {
        0 => DpOp::Add,
        5 => DpOp::Sub,
        _ => return Some(Err(err_for(hi))),
    };
    if lo & 0x8000 != 0 {
        return Some(Err(err_for(hi)));
    }
    Some(Ok(Inst32::DpImm {
        op,
        rd: ((lo >> 8) & 0xF) as u8,
        rn: (hi & 0xF) as u8,
        imm12: dp_imm12(hi, lo),
        expand: false,
        set_flags: hi & 0x10 != 0,
    }))
}

/// decode the ARMv7-M modified-immediate data-processing set.
fn decode_dp_modified(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // 1111 0 i 0 0 C op S Rn / 0 imm3 Rd imm8, C = bit8 selects the
    // add/sub family, `imm12` expanded by `ThumbExpandImm`.
    if hi & 0xF600 != 0xF000 {
        return None;
    }
    if lo & 0x8000 != 0 {
        return Some(Err(err_for(hi)));
    }
    let set_flags = hi & 0x10 != 0;
    let rn = (hi & 0xF) as u8;
    let rd = ((lo >> 8) & 0xF) as u8;
    let Some(op) = dp_op(hi & 0x100 != 0, ((hi >> 5) & 0x7) as u8, rn, rd, set_flags) else {
        return Some(Err(err_for(hi)));
    };
    Some(Ok(Inst32::DpImm {
        op,
        rd,
        rn,
        imm12: dp_imm12(hi, lo),
        expand: true,
        set_flags,
    }))
}

/// decode the ARMv7-M data-processing (register) set, with its shifted
/// Decode the ARMv7-M table branch instructions `TBB`/`TBH`.
///
/// `TBB [Rn, Rm]` is `1110 1000 1101 Rn / 1111 0000 0000 Rm` and
/// `TBH [Rn, Rm]` is `1110 1000 1101 Rn / 1111 0000 0001 Rm`; the second
/// halfword's bit 4 selects the byte table over the halfword table.
const fn decode_tbranch(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    if hi & 0xFFF0 != 0xE8D0 {
        return None;
    }
    let rn = (hi & 0xF) as u8;
    let half = match lo & 0xFFF0 {
        0xF000 => false,
        0xF010 => true,
        _ => return Some(Err(err_for(hi))),
    };
    let rm = (lo & 0xF) as u8;
    Some(Ok(Inst32::TblBranch { rn, rm, half }))
}

/// decode the ARMv7-M load/store dual family: `LDRD`/`STRD` with an
/// immediate offset.
///
/// The `1110 100P U1WL Rn` first-halfword space carries `P` (pre-indexed),
/// `U` (add), `W` (writeback), `L` (load), and `Rn`; the second halfword
/// packs `Rt Rt2 imm8`, where the byte offset is `imm8 << 2`. `LDRD` with
/// `Rn == 15` is the PC-relative literal form. The `(P, W) == (0, 0)`
/// combos of this space belong to `TBB`/`TBH` (consulted first) or are
/// UNDEFINED; register pairs touching the PC, a doubled register, a
/// writeback base that overlaps the pair, a `STRD` PC base, or writeback to
/// the PC are all UNPREDICTABLE and decode as UNDEFINED.
const fn decode_ldrd_strd(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    if hi & 0xFE40 != 0xE840 {
        return None;
    }
    let rn = (hi & 0xF) as u8;
    let is_load = hi & 0x10 != 0;
    let add = hi & 0x80 != 0;
    let index = hi & 0x100 != 0;
    let writeback = hi & 0x20 != 0;
    let rt = ((lo >> 12) & 0xF) as u8;
    let rt2 = ((lo >> 8) & 0xF) as u8;
    let err = Some(Err(err_for(hi)));
    if !index && !writeback {
        // The post-indexed forms always write back.
        return err;
    }
    if rt == 15 || rt2 == 15 || rt == rt2 {
        return err;
    }
    if !is_load && rn == 15 {
        // STRD has no PC-relative literal form.
        return err;
    }
    if rn == 15 && writeback {
        // Only the literal `LDRD` (no writeback) may use a PC base.
        return err;
    }
    if writeback && (rn == rt || rn == rt2) {
        return err;
    }
    Some(Ok(Inst32::LdrdStrd {
        is_load,
        rt,
        rt2,
        rn,
        imm: ((lo & 0xFF) as u32) << 2,
        add,
        index,
        writeback,
    }))
}

/// register operand.
fn decode_dp_reg(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // 1110 101 1 C op S Rn / 0 imm3 Rd imm2 type Rm. The shift type field is
    // 00 LSL, 01 LSR, 10 ASR, 11 ROR (ROR #0 is the RRX form).
    if hi & 0xFE00 != 0xEA00 {
        return None;
    }
    if lo & 0x8000 != 0 {
        return Some(Err(err_for(hi)));
    }
    let set_flags = hi & 0x10 != 0;
    let rn = (hi & 0xF) as u8;
    let rd = ((lo >> 8) & 0xF) as u8;
    let Some(op) = dp_op(hi & 0x100 != 0, ((hi >> 5) & 0x7) as u8, rn, rd, set_flags) else {
        return Some(Err(err_for(hi)));
    };
    let shift = match (lo >> 4) & 0x3 {
        0 => ShiftType::Lsl,
        1 => ShiftType::Lsr,
        2 => ShiftType::Asr,
        _ => ShiftType::Ror,
    };
    let amount = ((((lo >> 12) & 0x7) as u8) << 2) | (((lo >> 6) & 0x3) as u8);
    Some(Ok(Inst32::DpReg {
        op,
        rd,
        rn,
        rm: (lo & 0xF) as u8,
        shift,
        amount,
        set_flags,
    }))
}

/// decode the ARMv7-M divide instruction family.
///
/// `SDIV Rd, Rn, Rm` is `1111 1011 1001 Rn 1111 Rd 1111 Rm` and
/// `UDIV Rd, Rn, Rm` is `1111 1011 1011 Rn 1111 Rd 1111 Rm`; the first
/// halfword's low nibble is `Rn` and the second halfword packs `Rd` in the
/// middle. The `0xF000` upper nibbles of both halfwords make the pair
/// unmistakable, and there is no 16-bit form.
const fn decode_div(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let selected = hi & 0xFFF0;
    if selected != 0xFB90 && selected != 0xFBB0 {
        return None;
    }
    if lo & 0xF0F0 != 0xF0F0 {
        return Some(Err(err_for(hi)));
    }
    let rd = ((lo >> 8) & 0xF) as u8;
    let rn = (hi & 0xF) as u8;
    let rm = (lo & 0xF) as u8;
    Some(Ok(Inst32::Div {
        signed: selected == 0xFB90,
        rd,
        rn,
        rm,
    }))
}

/// decode the ARMv7-M multiply and long-multiply family.
///
/// `MUL`/`MLA`/`MLS` all start `1111 1011 0000 Rn` (Rn in the first
/// halfword's low nibble); the second halfword packs `Ra:Rd:sel:Rm`, where
/// `Ra == 1111` marks `MUL`, and `sel` bit 4 picks `MLS` over `MLA`. The long
/// forms `SMULL`/`UMULL`/`SMLAL`/`UMLAL` use the `1000`/`1010`/`1100`/`1110`
/// prefixes and pack `RdHi:RdLo:0000:Rm`. All seven ignore the flags.
const fn decode_mul(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let rn = (hi & 0xF) as u8;
    let rd = ((lo >> 8) & 0xF) as u8;
    let ra = ((lo >> 12) & 0xF) as u8;
    let rm = (lo & 0xF) as u8;
    let (op, is_long) = match hi & 0xFFF0 {
        0xFB00 => match lo & 0x00F0 {
            0x0000 => (if ra == 0xF { MulOp::Mul } else { MulOp::Mla }, false),
            0x0010 => (MulOp::Mls, false),
            _ => return Some(Err(err_for(hi))),
        },
        0xFB80 => (MulOp::Smull, true),
        0xFBA0 => (MulOp::Umull, true),
        0xFBC0 => (MulOp::Smlal, true),
        0xFBE0 => (MulOp::Umlal, true),
        _ => return None,
    };
    // The `0xFBC0`/`0xFBE0` prefixes carry `SMLALBB`..`SMLALTT`, `SMLALD`,
    // `SMLSLD` and `UMAAL` under other selectors, which `decode_dual_mul`
    // claims.
    if is_long && lo & 0x00F0 != 0 {
        return None;
    }
    Some(Ok(Inst32::Mul { op, rd, rn, rm, ra }))
}

/// Decode the ARMv7-M DSP multiply family: the halfword `SMUL`, the word
/// `SMMUL`/`SMMULR`, and `USAD8`/`USADA8`.
///
/// They share the `1111 1011` multiply zone with `decode_mul`; the prefix
/// nibble `op22_20` picks the family and the second halfword's `Ra:Rd:op:Rm`
/// fields pick the form. `SMULx` and `SMMULx` reuse the `Ra == 1111` sentinel
/// of the plain multiply: a nonzero `Ra` selects the accumulate variants
/// (`SMLAB`/`SMLAWB`/`SMMLA`), which this decoder does not implement yet.
const fn decode_dsp_mul(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let rn = (hi & 0xF) as u8;
    let rd = ((lo >> 8) & 0xF) as u8;
    let ra = ((lo >> 12) & 0xF) as u8;
    let rm = (lo & 0xF) as u8;
    let op = match hi & 0xFFF0 {
        0xFB10 => {
            if ra != 0xF {
                // `SMLABB`/`SMLABT`/`SMLATB`/`SMLATT` claim this prefix.
                return None;
            }
            match lo & 0x00F0 {
                0x0000 => DspMulOp::Smulbb,
                0x0010 => DspMulOp::Smulbt,
                0x0020 => DspMulOp::Smultb,
                0x0030 => DspMulOp::Smultt,
                _ => return Some(Err(err_for(hi))),
            }
        }
        0xFB30 => {
            if ra != 0xF {
                // `SMLAWB`/`SMLAWT` claim this prefix.
                return None;
            }
            match lo & 0x00F0 {
                0x0000 => DspMulOp::Smulwb,
                0x0010 => DspMulOp::Smulwt,
                _ => return Some(Err(err_for(hi))),
            }
        }
        0xFB50 => {
            if ra != 0xF {
                // `SMMLA`/`SMMLAR` claim this prefix.
                return None;
            }
            match lo & 0x00F0 {
                0x0000 => DspMulOp::Smmul,
                0x0010 => DspMulOp::Smmulr,
                _ => return Some(Err(err_for(hi))),
            }
        }
        0xFB70 => match lo & 0x00F0 {
            0x0000 => {
                if ra == 0xF {
                    DspMulOp::Usad8
                } else {
                    DspMulOp::Usada8
                }
            }
            _ => return Some(Err(err_for(hi))),
        },
        _ => return None,
    };
    Some(Ok(Inst32::DspMul { op, rd, rn, rm, ra }))
}

/// Decode the ARMv7-M dual multiply family: `SMLAD`/`SMLSD`, `SMLABB`..`SMLATT`,
/// `SMLAWB`/`SMLAWT`, `SMMLA`/`SMMLS`, the long forms `SMLALBB`..`SMLALTT`,
/// `SMLALD`/`SMLSLD`, and `UMAAL`.
///
/// They share the `1111 1011` multiply zone; the first-halfword nibble `op22_20`
/// picks the family and the second halfword's `Ra:Rd:sel:Rm` fields the form.
/// The accumulate forms (a nonzero `Ra` on the `0xFB10`/`0xFB30`/`0xFB50`
/// prefixes) and the `0xFB20`/`0xFB40`/`0xFB60`/`0xFBC0`/`0xFBD0`/`0xFBE0`
/// prefixes never match `decode_mul`/`decode_dsp_mul`, so this decoder is only
/// consulted in the multiply zone. `SMLAD`/`SMLSD` keep their `Ra == 1111`
/// `SMUAD`/`SMUSD` meaning. Flags are never affected, though `Q` may be set.
const fn decode_dual_mul(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let rn = (hi & 0xF) as u8;
    let rd = ((lo >> 8) & 0xF) as u8;
    let ra = ((lo >> 12) & 0xF) as u8;
    let rm = (lo & 0xF) as u8;
    let op = match hi & 0xFFF0 {
        0xFB10 => match lo & 0x00F0 {
            0x0000 => DualMulOp::Smlabb,
            0x0010 => DualMulOp::Smlabt,
            0x0020 => DualMulOp::Smlatb,
            0x0030 => DualMulOp::Smlatt,
            _ => return Some(Err(err_for(hi))),
        },
        0xFB20 => match lo & 0x00F0 {
            0x0000 => DualMulOp::Smlad,
            0x0010 => DualMulOp::Smladx,
            _ => return Some(Err(err_for(hi))),
        },
        0xFB30 => match lo & 0x00F0 {
            0x0000 => DualMulOp::Smlawb,
            0x0010 => DualMulOp::Smlawt,
            _ => return Some(Err(err_for(hi))),
        },
        0xFB40 => match lo & 0x00F0 {
            0x0000 => DualMulOp::Smlsd,
            0x0010 => DualMulOp::Smlsdx,
            _ => return Some(Err(err_for(hi))),
        },
        0xFB50 => match lo & 0x00F0 {
            0x0000 => DualMulOp::Smmla,
            0x0010 => DualMulOp::Smmlar,
            _ => return Some(Err(err_for(hi))),
        },
        0xFB60 => match lo & 0x00F0 {
            // `SMMLS` has no `Ra == 1111` form, so that case is UNDEFINED.
            0x0000 if ra != 0xF => DualMulOp::Smmls,
            0x0010 if ra != 0xF => DualMulOp::Smmlsr,
            _ => return Some(Err(err_for(hi))),
        },
        0xFBC0 => match lo & 0x00F0 {
            0x0080 => DualMulOp::Smlalbb,
            0x0090 => DualMulOp::Smlalbt,
            0x00A0 => DualMulOp::Smlaltb,
            0x00B0 => DualMulOp::Smlaltt,
            0x00C0 => DualMulOp::Smlald,
            0x00D0 => DualMulOp::Smlaldx,
            _ => return Some(Err(err_for(hi))),
        },
        0xFBD0 => match lo & 0x00F0 {
            0x00C0 => DualMulOp::Smlsld,
            0x00D0 => DualMulOp::Smlsldx,
            _ => return Some(Err(err_for(hi))),
        },
        0xFBE0 => match lo & 0x00F0 {
            0x0060 => DualMulOp::Umaal,
            _ => return Some(Err(err_for(hi))),
        },
        _ => return None,
    };
    Some(Ok(Inst32::DualMul { op, rd, rn, rm, ra }))
}

/// decode the ARMv7-M unary data-processing family: `REV`, `REV16`, `REVSH`,
/// `RBIT`, and `CLZ`.
///
/// They live in the `1111 1010` misc zone: `REV`/`REV16`/`REVSH`/`RBIT` share
/// the `1001` first-halfword selector (`0xFA90`, `RBIT`/`REV` selector `1010`
/// at second-halfword bit 4) and `CLZ` the `1011` selector (`0xFAB0`). The
/// register number is encoded twice — in the second halfword's `Rm` and the
/// first halfword's low nibble `Rm2` — and must agree. The `QADD` family
/// (selector `1000`, `0xFA80`) is handled by [`decode_qadd`].
const fn decode_unary(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // `Rm2` (first halfword low nibble) must copy `Rm`; otherwise the
    // encoding is UNPREDICTABLE, which we treat as UNDEFINED.
    if hi & 0xF != lo & 0xF {
        return None;
    }
    let op = match (hi & 0xFFF0, lo & 0xF0F0) {
        (0xFA90, 0xF080) => UnaryOp::Rev,
        (0xFA90, 0xF090) => UnaryOp::Rev16,
        (0xFA90, 0xF0A0) => UnaryOp::Rbit,
        (0xFA90, 0xF0B0) => UnaryOp::Revsh,
        (0xFAB0, 0xF080) => UnaryOp::Clz,
        _ => return None,
    };
    Some(Ok(Inst32::Unary {
        op,
        rd: ((lo >> 8) & 0xF) as u8,
        rm: (lo & 0xF) as u8,
    }))
}

/// Decode the ARMv7-M register-extend family (`SXTAH`/`UXTAH`/`SXTAB16`/
/// `UXTAB16`/`SXTAB`/`UXTAB`).
///
/// The first halfword is `1111 1010 <op> Rn` (`0xFA00 | op<<4 | rn`); the
/// second is `1111 Rd 10 <rot> Rm`. `Rn == 15` is the two-operand alias
/// (`SXTH`/`UXTB16`/...), which the executor folds by skipping the addend.
/// Only the `op` nibbles `0`-`5` are extend instructions; the `0xFA80`+
/// parallel, `QADD`, and unary families (and the reserved `0xFA60`/`0xFA70`
/// space) are left for the other decoders.
const fn decode_extend(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    if hi & 0xFF00 != 0xFA00 {
        return None;
    }
    let op_nibble = (hi >> 4) & 0xF;
    if op_nibble > 0x5 || lo & 0xF0C0 != 0xF080 {
        return None;
    }
    let op = match op_nibble {
        0x0 => ExtendAddOp::Sxtah,
        0x1 => ExtendAddOp::Uxtah,
        0x2 => ExtendAddOp::Sxtab16,
        0x3 => ExtendAddOp::Uxtab16,
        0x4 => ExtendAddOp::Sxtab,
        _ => ExtendAddOp::Uxtab,
    };
    Some(Ok(Inst32::ExtendAdd {
        op,
        rd: ((lo >> 8) & 0xF) as u8,
        rn: (hi & 0xF) as u8,
        rm: (lo & 0xF) as u8,
        rot: ((lo >> 4) & 0x3) as u8,
    }))
}

/// Decode the ARMv7-M parallel add/subtract family (`SADD8`/`QADD16`/... and
/// their `SH`/`U`/`UQ`/`UH` forms) plus the `SEL` instruction.
///
/// The first halfword is `1111 1010 fam Rn`; `fam` is a nibble from the
/// `ADD8`/`ADD16`/`ASX`/`SUB8`/`SUB16`/`SAX` set, and `Rn` is the first
/// operand. The second halfword is `1111 Rd <sel> Rm`, where the selector
/// nibble picks the saturation mode. `SEL` reuses the `ASX` family with
/// `sel == 1000`; the `sel` values that belong to `QADD` (family `ADD8`) and
/// the unary ops (families `ADD16`/`ASX`-adjacent `0x9`/`0xB`) are returned
/// as `None` so the other decoders claim them.
const fn decode_par_addsub(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    if hi & 0xFF00 != 0xFA00 {
        return None;
    }
    let fam = (hi >> 4) & 0xF;
    let sel = (lo >> 4) & 0xF;
    let op = match fam {
        0x8 => ParOp::Add8,
        0x9 => ParOp::Add16,
        0xA => ParOp::Asx,
        0xC => ParOp::Sub8,
        0xD => ParOp::Sub16,
        0xE => ParOp::Sax,
        _ => return None,
    };
    if fam == 0xA && sel == 0x8 {
        return Some(Ok(Inst32::Sel {
            rd: ((lo >> 8) & 0xF) as u8,
            rn: (hi & 0xF) as u8,
            rm: (lo & 0xF) as u8,
        }));
    }
    let sat = match sel {
        0x0 => ParSat::S,
        0x1 => ParSat::Q,
        0x2 => ParSat::Sh,
        0x4 => ParSat::U,
        0x5 => ParSat::Uq,
        0x6 => ParSat::Uh,
        _ => return None,
    };
    Some(Ok(Inst32::Par {
        op,
        sat,
        rd: ((lo >> 8) & 0xF) as u8,
        rn: (hi & 0xF) as u8,
        rm: (lo & 0xF) as u8,
    }))
}

/// decode the ARMv7-M saturating add/subtract family: `QADD`, `QDADD`, `QSUB`,
/// and `QDSUB`.
///
/// All four share the `1111 1010 1000 Rn` first halfword (`0xFA80`); the
/// second halfword's selector nibble picks the opcode. Each reads `Rn` and
/// `Rm`, writes `Rd`, and clamps to the signed 32-bit range, setting the
/// sticky Q flag on saturation.
const fn decode_qadd(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    if hi & 0xFFF0 != 0xFA80 {
        return None;
    }
    // The second halfword is `1111 Rd <sel> Rm`; bit 15 is the fixed `1`.
    let op = match (lo & 0xF000, lo & 0x00F0) {
        (0xF000, 0x0080) => QaddOp::Qadd,
        (0xF000, 0x0090) => QaddOp::Qdadd,
        (0xF000, 0x00A0) => QaddOp::Qsub,
        (0xF000, 0x00B0) => QaddOp::Qdsub,
        _ => return Some(Err(err_for(hi))),
    };
    Some(Ok(Inst32::Qadd {
        op,
        rd: ((lo >> 8) & 0xF) as u8,
        rn: (hi & 0xF) as u8,
        rm: (lo & 0xF) as u8,
    }))
}

/// decode the ARMv7-M saturate family: `SSAT`, `USAT`, `SSAT16`, and `USAT16`.
///
/// `SSAT`/`USAT` use `1111 0011 0/10 sh Rn` first halfwords (`0xF300`/`0xF380`
/// with `sh` at bit 5); the second halfword packs `0 imm3 Rd imm2 0 sat_imm`,
/// where `imm3:imm2` is the shift amount. `sh == 1` with a zero shift is the
/// `SSAT16`/`USAT16` alias, which saturates each halfword independently and
/// never shifts; the fixed zero bits (second-halfword bit 15 and bit 5) guard
/// the encoding. The bitfield family's `0xF340`/`0xF360`/`0xF3C0` prefixes
/// are disjoint, and `decode_msr` (also `0xF380`) is disambiguated by the
/// second halfword's bit 15.
const fn decode_sat(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let sh = hi & (1 << 5) != 0;
    // Mask off `sh` (bit 5) and `Rn` (low nibble) but keep the fixed zero at
    // bit 4: both `SSAT`/`USAT` share the `1111 0011 00/10 0` prefix.
    let op = match hi & 0xFFD0 {
        0xF300 => SatOp::Ssat,
        0xF380 => SatOp::Usat,
        _ => return None,
    };
    // The second halfword is `0 imm3 Rd imm2 0 sat_imm`: bit 15 and bit 5
    // are fixed zeros.
    if lo & 0x8020 != 0 {
        return Some(Err(err_for(hi)));
    }
    let rd = ((lo >> 8) & 0xF) as u8;
    let rn = (hi & 0xF) as u8;
    let shift = ((((lo >> 12) & 0x7) as u8) << 2) | (((lo >> 6) & 0x3) as u8);
    let sat_n = (lo & 0x1F) as u8 + 1;
    // `sh == 1` with `imm3:imm2 == 0` is the SEE alias to the 16-bit form,
    // which never shifts.
    if sh && shift == 0 {
        return Some(Ok(Inst32::Sat {
            op: match op {
                SatOp::Ssat => SatOp::Ssat16,
                SatOp::Usat => SatOp::Usat16,
                _ => unreachable!(),
            },
            rd,
            rn,
            sat_n,
            sh: false,
            shift: 0,
        }));
    }
    Some(Ok(Inst32::Sat {
        op,
        rd,
        rn,
        sat_n,
        sh,
        shift,
    }))
}

/// decode the ARMv7-M bitfield family: `SBFX`, `UBFX`, `BFI`, and `BFC`.
///
/// The `1111 0011 010/110/011 0` first-halfword prefixes select
/// `SBFX`/`UBFX`/`BFI`, with the source `Rn` in the low nibble (`Rn == 15`
/// turns `BFI` into `BFC`). The second halfword packs `0 imm3 Rd imm2 0
/// field`, where `imm3:imm2` builds the `lsb` position (the same composition
/// as the data-processing shift amounts) and `field` is `widthm1` for the
/// extracts or `msb` for the inserts. The extracts UNDEFINE when the field
/// runs past bit 31; the inserts UNDEFINE when `msb < lsb`.
const fn decode_bitfield(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    let rn = (hi & 0xF) as u8;
    let op = match hi & 0xFFF0 {
        0xF340 => BitfieldOp::Sbfx,
        0xF3C0 => BitfieldOp::Ubfx,
        0xF360 => {
            if rn == 0xF {
                BitfieldOp::Bfc
            } else {
                BitfieldOp::Bfi
            }
        }
        _ => return None,
    };
    // The second halfword's bit 15 and bit 5 are fixed zero.
    if lo & 0x8020 != 0 {
        return Some(Err(err_for(hi)));
    }
    let rd = ((lo >> 8) & 0xF) as u8;
    let lsb = ((((lo >> 12) & 0x7) as u8) << 2) | (((lo >> 6) & 0x3) as u8);
    let field = (lo & 0x1F) as u8;
    if matches!(op, BitfieldOp::Sbfx | BitfieldOp::Ubfx) {
        if lsb + field > 31 {
            return Some(Err(err_for(hi)));
        }
    } else if field < lsb {
        return Some(Err(err_for(hi)));
    }
    Some(Ok(Inst32::Bitfield {
        op,
        rd,
        rn,
        lsb,
        field,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Decode a little-endian byte pair as produced by llvm-mc.
    fn decode32_bytes(bytes: [u8; 4]) -> Result<Inst32, DecodeError> {
        let hi = u16::from_le_bytes([bytes[0], bytes[1]]);
        let lo = u16::from_le_bytes([bytes[2], bytes[3]]);
        decode32(hi, lo)
    }

    /// Decode a byte quadruple (little-endian halfword pair) as an ARMv7-M
    /// 32-bit instruction.
    fn v7m_bytes(bytes: [u8; 4]) -> Result<Inst32, DecodeError> {
        let hi = u16::from_le_bytes([bytes[0], bytes[1]]);
        let lo = u16::from_le_bytes([bytes[2], bytes[3]]);
        decode32_v7m(hi, lo)
    }

    fn reg(
        op: DpOp,
        rd: u8,
        rn: u8,
        rm: u8,
        shift: ShiftType,
        amount: u8,
        set_flags: bool,
    ) -> Inst32 {
        Inst32::DpReg {
            op,
            rd,
            rn,
            rm,
            shift,
            amount,
            set_flags,
        }
    }

    fn imm(op: DpOp, rd: u8, rn: u8, imm12: u16, expand: bool, set_flags: bool) -> Inst32 {
        Inst32::DpImm {
            op,
            rd,
            rn,
            imm12,
            expand,
            set_flags,
        }
    }

    fn mem(
        access: MemOp,
        rt: u8,
        rn: u8,
        off: u32,
        add: bool,
        index: bool,
        writeback: bool,
    ) -> Inst32 {
        Inst32::Mem {
            access,
            rt,
            rn,
            off,
            rm: None,
            shift: 0,
            add,
            index,
            writeback,
        }
    }

    /// The register-offset form: `R[n] ± R[rm] << shift`, always add, index
    /// only, no write-back.
    fn mem_reg(access: MemOp, rt: u8, rn: u8, rm: u8, shift: u8) -> Inst32 {
        Inst32::Mem {
            access,
            rt,
            rn,
            off: 0,
            rm: Some(rm),
            shift,
            add: true,
            index: true,
            writeback: false,
        }
    }

    #[test]
    fn is_thumb2_v7m_full_space() {
        assert!(is_thumb2_v7m(0xE800)); // first 32-bit T32 halfword
        assert!(is_thumb2_v7m(0xEA09)); // and.w register
        assert!(is_thumb2_v7m(0xF04F)); // mov.w
        assert!(is_thumb2_v7m(0xFFFF));
        assert!(!is_thumb2_v7m(0xE7FF)); // last 16-bit halfword
        assert!(!is_thumb2_v7m(0x4770)); // bx lr
    }

    #[test]
    fn thumb_expand_imm_cases() {
        // Every constant clang encodes for `mov.w` in probe38, plus the
        // probe31 variants, must round-trip through ThumbExpandImm.
        let cases: &[(u16, u32)] = &[
            (0x001, 0x1),
            (0x0FF, 0xFF),
            (0xF80, 0x100),
            (0xC7F, 0xFF00),
            (0x87F, 0xFF_0000),
            (0x47F, 0xFF00_0000),
            (0xC00, 0x8000),
            (0x00A, 0xA),
            (0x82F, 0xAF_0000),
            (0x2FF, 0xFF00_FF00),
            (0x1FF, 0xFF_00FF),
            (0x670, 0x0F00_0000),
            (0x470, 0xF000_0000),
            (0x301, 0x0101_0101),
            (0x380, 0x8080_8080),
            (0xFFF, 0x1FE),
            (0xF7F, 0x3FC),
            (0x4FF, 0x7F80_0000),
            (0x480, 0x4000_0000),
            (0xCFF, 0x7F80),
            (0xC7E, 0xFE00),
            (0xC80, 0x4000),
            (0xF9F, 0x13E),
        ];
        for &(imm12, want) in cases {
            assert_eq!(thumb_expand_imm(imm12), want, "imm12 = {imm12:#x}");
        }
    }

    #[test]
    fn v7m_modified_immediate() {
        // mov.w r0, #0x8 -> f04f 0008.
        assert_eq!(
            v7m_bytes([0x4f, 0xf0, 0x08, 0x00]),
            Ok(imm(DpOp::Mov, 0, 15, 0x008, true, false))
        );
        // movs.w r0, #0x8 -> f05f 0008.
        assert_eq!(
            v7m_bytes([0x5f, 0xf0, 0x08, 0x00]),
            Ok(imm(DpOp::Mov, 0, 15, 0x008, true, true))
        );
        // mvn r0, #0x8 -> f06f 0008.
        assert_eq!(
            v7m_bytes([0x6f, 0xf0, 0x08, 0x00]),
            Ok(imm(DpOp::Mvn, 0, 15, 0x008, true, false))
        );
        // rsb.w r0, r1, #0x8 -> f1c1 0008.
        assert_eq!(
            v7m_bytes([0xc1, 0xf1, 0x08, 0x00]),
            Ok(imm(DpOp::Rsb, 0, 1, 0x008, true, false))
        );
        // cmp.w r0, #0xff000000 -> f1b0 4f7f.
        assert_eq!(
            v7m_bytes([0xb0, 0xf1, 0x7f, 0x4f]),
            Ok(imm(DpOp::Cmp, 15, 0, 0x47F, true, true))
        );
        // tst.w r1, #0xff -> f011 0fff.
        assert_eq!(
            v7m_bytes([0x11, 0xf0, 0xff, 0x0f]),
            Ok(imm(DpOp::Tst, 15, 1, 0x0FF, true, true))
        );
        // eor r0, r1, #0xf000000 -> f081 6070 (imm12 0x670).
        assert_eq!(
            v7m_bytes([0x81, 0xf0, 0x70, 0x60]),
            Ok(imm(DpOp::Eor, 0, 1, 0x670, true, false))
        );
        // add.w sp, sp, #0x8 -> f10d 0d08.
        assert_eq!(
            v7m_bytes([0x0d, 0xf1, 0x08, 0x0d]),
            Ok(imm(DpOp::Add, 13, 13, 0x008, true, false))
        );
        // sub.w sp, sp, #0x8 -> f1ad 0d08.
        assert_eq!(
            v7m_bytes([0xad, 0xf1, 0x08, 0x0d]),
            Ok(imm(DpOp::Sub, 13, 13, 0x008, true, false))
        );
        // mov.w r0, #0x80808080 -> f04f 3080.
        assert_eq!(
            v7m_bytes([0x4f, 0xf0, 0x80, 0x30]),
            Ok(imm(DpOp::Mov, 0, 15, 0x380, true, false))
        );
    }

    #[test]
    fn v7m_plain_binary_immediate() {
        // addw r0, r1, #0xff -> f201 00ff.
        assert_eq!(
            v7m_bytes([0x01, 0xf2, 0xff, 0x00]),
            Ok(imm(DpOp::Add, 0, 1, 0x0FF, false, false))
        );
        // addw r1, r2, #0x123 -> f202 1123.
        assert_eq!(
            v7m_bytes([0x02, 0xf2, 0x23, 0x11]),
            Ok(imm(DpOp::Add, 1, 2, 0x123, false, false))
        );
        // subw r3, r4, #0xfff -> f6a4 73ff.
        assert_eq!(
            v7m_bytes([0xa4, 0xf6, 0xff, 0x73]),
            Ok(imm(DpOp::Sub, 3, 4, 0xFFF, false, false))
        );
    }

    #[test]
    fn v7m_register_data_processing() {
        // add.w r8, r9, r10 -> eb09 080a.
        assert_eq!(
            v7m_bytes([0x09, 0xeb, 0x0a, 0x08]),
            Ok(reg(DpOp::Add, 8, 9, 10, ShiftType::Lsl, 0, false))
        );
        // and.w r8, r9, r10 -> ea09 080a.
        assert_eq!(
            v7m_bytes([0x09, 0xea, 0x0a, 0x08]),
            Ok(reg(DpOp::And, 8, 9, 10, ShiftType::Lsl, 0, false))
        );
        // adc.w r8, r9, r10 -> eb49 080a.
        assert_eq!(
            v7m_bytes([0x49, 0xeb, 0x0a, 0x08]),
            Ok(reg(DpOp::Adc, 8, 9, 10, ShiftType::Lsl, 0, false))
        );
        // sbc.w r0, r1, r2 -> eb61 0002.
        assert_eq!(
            v7m_bytes([0x61, 0xeb, 0x02, 0x00]),
            Ok(reg(DpOp::Sbc, 0, 1, 2, ShiftType::Lsl, 0, false))
        );
        // rsb r4, r5, r6, lsl #8 -> ebc5 2406.
        assert_eq!(
            v7m_bytes([0xc5, 0xeb, 0x06, 0x24]),
            Ok(reg(DpOp::Rsb, 4, 5, 6, ShiftType::Lsl, 8, false))
        );
        // rsb r4, r5, r6, asr #8 -> ebc5 2426.
        assert_eq!(
            v7m_bytes([0xc5, 0xeb, 0x26, 0x24]),
            Ok(reg(DpOp::Rsb, 4, 5, 6, ShiftType::Asr, 8, false))
        );
        // sub.w r0, r1, r2 -> eba1 0002.
        assert_eq!(
            v7m_bytes([0xa1, 0xeb, 0x02, 0x00]),
            Ok(reg(DpOp::Sub, 0, 1, 2, ShiftType::Lsl, 0, false))
        );
        // subs.w r0, r1, r2 -> ebb1 0002.
        assert_eq!(
            v7m_bytes([0xb1, 0xeb, 0x02, 0x00]),
            Ok(reg(DpOp::Sub, 0, 1, 2, ShiftType::Lsl, 0, true))
        );
        // mvn.w r0, r1 -> ea6f 0001.
        assert_eq!(
            v7m_bytes([0x6f, 0xea, 0x01, 0x00]),
            Ok(reg(DpOp::Mvn, 0, 15, 1, ShiftType::Lsl, 0, false))
        );
        // mvn.w r0, r1, lsl #4 -> ea6f 1001.
        assert_eq!(
            v7m_bytes([0x6f, 0xea, 0x01, 0x10]),
            Ok(reg(DpOp::Mvn, 0, 15, 1, ShiftType::Lsl, 4, false))
        );
        // lsls.w r0, r1, #3 -> ea5f 00c1.
        assert_eq!(
            v7m_bytes([0x5f, 0xea, 0xc1, 0x00]),
            Ok(reg(DpOp::Mov, 0, 15, 1, ShiftType::Lsl, 3, true))
        );
        // lsrs.w r0, r1, #3 -> ea5f 00d1.
        assert_eq!(
            v7m_bytes([0x5f, 0xea, 0xd1, 0x00]),
            Ok(reg(DpOp::Mov, 0, 15, 1, ShiftType::Lsr, 3, true))
        );
        // asrs.w r0, r1, #3 -> ea5f 00e1.
        assert_eq!(
            v7m_bytes([0x5f, 0xea, 0xe1, 0x00]),
            Ok(reg(DpOp::Mov, 0, 15, 1, ShiftType::Asr, 3, true))
        );
        // add.w r0, r1, r2, lsl #4 -> eb01 1002.
        assert_eq!(
            v7m_bytes([0x01, 0xeb, 0x02, 0x10]),
            Ok(reg(DpOp::Add, 0, 1, 2, ShiftType::Lsl, 4, false))
        );
        // add.w r8, r9, r10, asr #7 -> eb09 18ea.
        assert_eq!(
            v7m_bytes([0x09, 0xeb, 0xea, 0x18]),
            Ok(reg(DpOp::Add, 8, 9, 10, ShiftType::Asr, 7, false))
        );
        // add.w r8, r9, r10, ror #3 -> eb09 08fa.
        assert_eq!(
            v7m_bytes([0x09, 0xeb, 0xfa, 0x08]),
            Ok(reg(DpOp::Add, 8, 9, 10, ShiftType::Ror, 3, false))
        );
        // add.w r8, r9, r10, rrx -> eb09 083a.
        assert_eq!(
            v7m_bytes([0x09, 0xeb, 0x3a, 0x08]),
            Ok(reg(DpOp::Add, 8, 9, 10, ShiftType::Ror, 0, false))
        );
        // sub.w r8, r9, r10, lsr #31 -> eba9 78da.
        assert_eq!(
            v7m_bytes([0xa9, 0xeb, 0xda, 0x78]),
            Ok(reg(DpOp::Sub, 8, 9, 10, ShiftType::Lsr, 31, false))
        );
        // tst.w r8, r9 -> ea18 0f09.
        assert_eq!(
            v7m_bytes([0x18, 0xea, 0x09, 0x0f]),
            Ok(reg(DpOp::Tst, 15, 8, 9, ShiftType::Lsl, 0, true))
        );
        // teq.w r8, r9 -> ea98 0f09.
        assert_eq!(
            v7m_bytes([0x98, 0xea, 0x09, 0x0f]),
            Ok(reg(DpOp::Teq, 15, 8, 9, ShiftType::Lsl, 0, true))
        );
        // cmn.w r8, r9 -> eb18 0f09.
        assert_eq!(
            v7m_bytes([0x18, 0xeb, 0x09, 0x0f]),
            Ok(reg(DpOp::Cmn, 15, 8, 9, ShiftType::Lsl, 0, true))
        );
        // cmp.w r8, r9 -> ebb8 0f09.
        assert_eq!(
            v7m_bytes([0xb8, 0xeb, 0x09, 0x0f]),
            Ok(reg(DpOp::Cmp, 15, 8, 9, ShiftType::Lsl, 0, true))
        );
    }

    #[test]
    fn v7m_movw_movt() {
        // movw r9, #0x1234 -> f241 2934.
        assert_eq!(
            v7m_bytes([0x41, 0xf2, 0x34, 0x29]),
            Ok(Inst32::MovWide {
                rd: 9,
                imm: 0x1234,
                high: false
            })
        );
        // movt r9, #0xabcd -> f6ca 39cd.
        assert_eq!(
            v7m_bytes([0xca, 0xf6, 0xcd, 0x39]),
            Ok(Inst32::MovWide {
                rd: 9,
                imm: 0xABCD,
                high: true
            })
        );
        // movw r12, #0x1234 -> f241 2c34.
        assert_eq!(
            v7m_bytes([0x41, 0xf2, 0x34, 0x2c]),
            Ok(Inst32::MovWide {
                rd: 12,
                imm: 0x1234,
                high: false
            })
        );
        // movw r0, #0xfedc -> f64f 60dc.
        assert_eq!(
            v7m_bytes([0x4f, 0xf6, 0xdc, 0x60]),
            Ok(Inst32::MovWide {
                rd: 0,
                imm: 0xFEDC,
                high: false
            })
        );
    }

    #[test]
    fn v7m_load_store_immediate() {
        // ldr.w r0, [r1, #4] -> f8d1 0004 (imm12 offset).
        assert_eq!(
            v7m_bytes([0xd1, 0xf8, 0x04, 0x00]),
            Ok(mem(MemOp::Ldr, 0, 1, 4, true, true, false))
        );
        // str.w r0, [r1, #4] -> f8c1 0004.
        assert_eq!(
            v7m_bytes([0xc1, 0xf8, 0x04, 0x00]),
            Ok(mem(MemOp::Str, 0, 1, 4, true, true, false))
        );
        // ldrb.w r12, [r3, #0xff] -> f893 c0ff.
        assert_eq!(
            v7m_bytes([0x93, 0xf8, 0xff, 0xc0]),
            Ok(mem(MemOp::Ldrb, 12, 3, 0xFF, true, true, false))
        );
        // strb.w r0, [r1, #4] -> f881 0004.
        assert_eq!(
            v7m_bytes([0x81, 0xf8, 0x04, 0x00]),
            Ok(mem(MemOp::Strb, 0, 1, 4, true, true, false))
        );
        // ldrh.w r0, [r1, #4] -> f8b1 0004.
        assert_eq!(
            v7m_bytes([0xb1, 0xf8, 0x04, 0x00]),
            Ok(mem(MemOp::Ldrh, 0, 1, 4, true, true, false))
        );
        // strh.w r0, [r1, #4] -> f8a1 0004.
        assert_eq!(
            v7m_bytes([0xa1, 0xf8, 0x04, 0x00]),
            Ok(mem(MemOp::Strh, 0, 1, 4, true, true, false))
        );
        // ldrsb.w r0, [r1, #4] -> f991 0004.
        assert_eq!(
            v7m_bytes([0x91, 0xf9, 0x04, 0x00]),
            Ok(mem(MemOp::Ldrsb, 0, 1, 4, true, true, false))
        );
        // ldrsh.w r0, [r1, #4] -> f9b1 0004.
        assert_eq!(
            v7m_bytes([0xb1, 0xf9, 0x04, 0x00]),
            Ok(mem(MemOp::Ldrsh, 0, 1, 4, true, true, false))
        );
        // ldr.w r12, [r3, #0x20] -> f8d3 c020 (imm12 offset).
        assert_eq!(
            v7m_bytes([0xd3, 0xf8, 0x20, 0xc0]),
            Ok(mem(MemOp::Ldr, 12, 3, 0x20, true, true, false))
        );
        // ldr.w r10, [r1, #-4] -> f851 ac04 (imm8 offset, Rt in 15..12).
        assert_eq!(
            v7m_bytes([0x51, 0xf8, 0x04, 0xac]),
            Ok(mem(MemOp::Ldr, 10, 1, 4, false, true, false))
        );
        // ldr.w r0, [r1, #-8] -> f851 0c08 (imm8, offset, subtract).
        assert_eq!(
            v7m_bytes([0x51, 0xf8, 0x08, 0x0c]),
            Ok(mem(MemOp::Ldr, 0, 1, 8, false, true, false))
        );
        // ldr.w r0, [r1, #8]! -> f851 0f08 (imm8, pre-indexed).
        assert_eq!(
            v7m_bytes([0x51, 0xf8, 0x08, 0x0f]),
            Ok(mem(MemOp::Ldr, 0, 1, 8, true, true, true))
        );
        // ldr.w r0, [r1], #8 -> f851 0b08 (imm8, post-indexed).
        assert_eq!(
            v7m_bytes([0x51, 0xf8, 0x08, 0x0b]),
            Ok(mem(MemOp::Ldr, 0, 1, 8, true, false, true))
        );
        // ldr.w r4, [pc, #8] -> f8df 4008 (PC-relative literal, imm12).
        assert_eq!(
            v7m_bytes([0xdf, 0xf8, 0x08, 0x40]),
            Ok(mem(MemOp::Ldr, 4, 15, 8, true, true, false))
        );
        // ldrh.w r5, [pc, #4] -> f8bf 5004 (PC-relative literal, imm12).
        assert_eq!(
            v7m_bytes([0xbf, 0xf8, 0x04, 0x50]),
            Ok(mem(MemOp::Ldrh, 5, 15, 4, true, true, false))
        );
        // ldrsb.w r9, [r6, #-1] -> f916 9c01 (imm8 offset, subtract).
        assert_eq!(
            v7m_bytes([0x16, 0xf9, 0x01, 0x9c]),
            Ok(mem(MemOp::Ldrsb, 9, 6, 1, false, true, false))
        );
    }

    #[test]
    fn v7m_load_store_register() {
        // The register-offset second halfword is `Rt 000000 shimm Rm`, where
        // the offset is `Rm << (shimm * 2)` (LSL only).
        // ldr.w r0, [r2, r3] -> f852 0003.
        assert_eq!(
            v7m_bytes([0x52, 0xf8, 0x03, 0x00]),
            Ok(mem_reg(MemOp::Ldr, 0, 2, 3, 0))
        );
        // ldr.w r2, [r4, r8] -> f854 2008.
        assert_eq!(
            v7m_bytes([0x54, 0xf8, 0x08, 0x20]),
            Ok(mem_reg(MemOp::Ldr, 2, 4, 8, 0))
        );
        // str.w r5, [r6, r7, lsl #2] -> f846 5017.
        assert_eq!(
            v7m_bytes([0x46, 0xf8, 0x17, 0x50]),
            Ok(mem_reg(MemOp::Str, 5, 6, 7, 2))
        );
        // strb.w r0, [r1, r2] -> f801 0002.
        assert_eq!(
            v7m_bytes([0x01, 0xf8, 0x02, 0x00]),
            Ok(mem_reg(MemOp::Strb, 0, 1, 2, 0))
        );
        // ldrb.w r9, [r3, r4] -> f813 9004.
        assert_eq!(
            v7m_bytes([0x13, 0xf8, 0x04, 0x90]),
            Ok(mem_reg(MemOp::Ldrb, 9, 3, 4, 0))
        );
        // strh.w r6, [r12, r2, lsl #2] -> f82c 6012.
        assert_eq!(
            v7m_bytes([0x2c, 0xf8, 0x12, 0x60]),
            Ok(mem_reg(MemOp::Strh, 6, 12, 2, 2))
        );
        // ldrh.w r3, [r10, r4] -> f83a 3004.
        assert_eq!(
            v7m_bytes([0x3a, 0xf8, 0x04, 0x30]),
            Ok(mem_reg(MemOp::Ldrh, 3, 10, 4, 0))
        );
        // ldrsb.w r7, [r8, r11, lsl #6] -> f918 703b.
        assert_eq!(
            v7m_bytes([0x18, 0xf9, 0x3b, 0x70]),
            Ok(mem_reg(MemOp::Ldrsb, 7, 8, 11, 6))
        );
        // ldrsh.w r0, [r2, r3, lsl #4] -> f932 0023.
        assert_eq!(
            v7m_bytes([0x32, 0xf9, 0x23, 0x00]),
            Ok(mem_reg(MemOp::Ldrsh, 0, 2, 3, 4))
        );
    }

    #[test]
    fn v7m_load_store_undefined_cases() {
        // A store through the PC base is UNPREDICTABLE.
        assert!(v7m_bytes([0xcf, 0xf8, 0x04, 0x00]).is_err());
        // A store to PC is UNPREDICTABLE.
        assert!(v7m_bytes([0xc1, 0xf8, 0x04, 0xf0]).is_err());
        // A register-offset store through the PC base is UNPREDICTABLE.
        assert!(v7m_bytes([0x4f, 0xf8, 0x01, 0x00]).is_err());
        // P == 0 && W == 0 in the imm8 form is UNDEFINED.
        assert!(v7m_bytes([0x51, 0xf8, 0x08, 0x0a]).is_err());
        // The unprivileged (LDRT/STRT) pattern is UNPREDICTABLE.
        assert!(v7m_bytes([0x51, 0xf8, 0x00, 0x0e]).is_err());
        // A second halfword with bit 11 clear but bits 11..6 non-zero is
        // not a valid immediate or register form.
        assert!(v7m_bytes([0x50, 0xf8, 0x40, 0x00]).is_err());
        // The ARMv6-M decoder must not swallow the ARMv7-M memory set.
        assert!(decode32_bytes([0x51, 0xf8, 0x08, 0x0b]).is_err());
    }

    #[test]
    fn v6m_rejects_armv7m_data_processing() {
        // The ARMv6-M decoder must not swallow the ARMv7-M set.
        assert!(decode32_bytes([0x4f, 0xf0, 0x08, 0x00]).is_err()); // mov.w
        assert!(decode32_bytes([0x09, 0xeb, 0x0a, 0x08]).is_err()); // add.w reg
        assert!(decode32_bytes([0x41, 0xf2, 0x34, 0x29]).is_err()); // movw
    }

    #[test]
    fn is_thumb2_detects_prefixes() {
        assert!(is_thumb2(0xF000)); // bl
        assert!(is_thumb2(0xF7FF)); // bl far
        assert!(is_thumb2(0xF3EF)); // mrs
        assert!(is_thumb2(0xF380)); // msr
        assert!(is_thumb2(0xF3BF)); // dmb/dsb/isb
        assert!(!is_thumb2(0xE7FE)); // 16-bit b
        assert!(!is_thumb2(0xD0FE)); // 16-bit b<cond>
    }

    #[test]
    fn bl_offsets() {
        // bl . (target self) -> [0x00, 0xf0, 0x00, 0xf8]: offset 0.
        assert_eq!(
            decode32_bytes([0x00, 0xf0, 0x00, 0xf8]),
            Ok(Inst32::Bl { off: 0 })
        );
        // bl +4 from 0x6 (encodes as f000 f802) -> off 4.
        assert_eq!(
            decode32_bytes([0x00, 0xf0, 0x02, 0xf8]),
            Ok(Inst32::Bl { off: 4 })
        );
        // bl -4 from 0xa (f7ff fffe) -> off -4.
        assert_eq!(
            decode32_bytes([0xff, 0xf7, 0xfe, 0xff]),
            Ok(Inst32::Bl { off: -4 })
        );
    }

    #[test]
    fn bw_offsets() {
        // b.w +10 from 0x6 (f000 b805) -> off 10.
        assert_eq!(
            v7m_bytes([0x00, 0xf0, 0x05, 0xb8]),
            Ok(Inst32::BranchWide { off: 10 })
        );
        // b.w -12 from 0xa (f7ff bffa) -> off -12.
        assert_eq!(
            v7m_bytes([0xff, 0xf7, 0xfa, 0xbf]),
            Ok(Inst32::BranchWide { off: -12 })
        );
        // b.w +16384 with a nonzero imm10 (f001 b800) -> off 0x4000.
        assert_eq!(
            v7m_bytes([0x01, 0xf0, 0x00, 0xb8]),
            Ok(Inst32::BranchWide { off: 0x4000 })
        );
        // b.w +16380 with imm11 = 0x7fe (f000 97fe) -> off 0x3ffc.
        assert_eq!(
            v7m_bytes([0x00, 0xf0, 0xfe, 0x97]),
            Ok(Inst32::BranchWide { off: 0x3ffc })
        );
    }

    #[test]
    fn bcc_wide_offsets() {
        // beq.w +8 from 0x6 (f000 a804) -> cond EQ, off 8.
        assert_eq!(
            v7m_bytes([0x00, 0xf0, 0x04, 0xa8]),
            Ok(Inst32::BranchCondWide { cond: 0, off: 8 })
        );
        // bmi.w -183396 (f513 a9ce) -> cond MI, off -183396.
        assert_eq!(
            v7m_bytes([0x13, 0xf5, 0xce, 0xa9]),
            Ok(Inst32::BranchCondWide {
                cond: 4,
                off: -183_396
            })
        );
        // b.al.w (cond == 0xf) is UNPREDICTABLE -> UndefinedInstruction.
        assert_eq!(
            v7m_bytes([0xc0, 0xf3, 0x00, 0xa8]),
            Err(DecodeError {
                opcode: 0xf3c0,
                cause: HaltCause::UndefinedInstruction
            })
        );
    }

    #[test]
    fn blx_immediate_offsets() {
        // blx . (f000 e800) -> off 0.
        assert_eq!(
            v7m_bytes([0x00, 0xf0, 0x00, 0xe8]),
            Ok(Inst32::BlxImm { off: 0 })
        );
        // blx +8 (f000 e804) -> off 8.
        assert_eq!(
            v7m_bytes([0x00, 0xf0, 0x04, 0xe8]),
            Ok(Inst32::BlxImm { off: 8 })
        );
        // blx +8192 with I1 = 1 (f000 c800) -> off 0x2000.
        assert_eq!(
            v7m_bytes([0x00, 0xf0, 0x00, 0xc8]),
            Ok(Inst32::BlxImm { off: 0x2000 })
        );
        // blx -4 (f7ff efff) -> off -4.
        assert_eq!(
            v7m_bytes([0xff, 0xf7, 0xff, 0xef]),
            Ok(Inst32::BlxImm { off: -4 })
        );
    }

    #[test]
    fn bl_accepts_j1_j2_ne_one() {
        // bl +4104 with J2 = 0 (f000 f004) -> off 4104. The old `0xf800`
        // second-halfword mask rejected this; the `0xd000` mask accepts it.
        assert_eq!(
            v7m_bytes([0x00, 0xf0, 0x04, 0xf0]),
            Ok(Inst32::Bl { off: 4104 })
        );
    }

    #[test]
    fn mrs_msr() {
        // mrs r1, msp -> f3ef 8108.
        assert_eq!(
            decode32_bytes([0xef, 0xf3, 0x08, 0x81]),
            Ok(Inst32::Mrs {
                rd: 1,
                sys: SysReg::Msp
            })
        );
        // mrs r0, primask -> f3ef 8010.
        assert_eq!(
            decode32_bytes([0xef, 0xf3, 0x10, 0x80]),
            Ok(Inst32::Mrs {
                rd: 0,
                sys: SysReg::Primask
            })
        );
        // msr control, r0 -> f380 8814.
        assert_eq!(
            decode32_bytes([0x80, 0xf3, 0x14, 0x88]),
            Ok(Inst32::Msr {
                rn: 0,
                sys: SysReg::Control
            })
        );
        // msr msp, r6 -> f386 8808.
        assert_eq!(
            decode32_bytes([0x86, 0xf3, 0x08, 0x88]),
            Ok(Inst32::Msr {
                rn: 6,
                sys: SysReg::Msp
            })
        );
    }

    #[test]
    fn barriers() {
        // dmb sy -> f3bf 8f5f.
        assert_eq!(
            decode32_bytes([0xbf, 0xf3, 0x5f, 0x8f]),
            Ok(Inst32::Barrier { kind: Barrier::Dmb })
        );
        // dsb sy -> f3bf 8f4f.
        assert_eq!(
            decode32_bytes([0xbf, 0xf3, 0x4f, 0x8f]),
            Ok(Inst32::Barrier { kind: Barrier::Dsb })
        );
        // isb sy -> f3bf 8f6f.
        assert_eq!(
            decode32_bytes([0xbf, 0xf3, 0x6f, 0x8f]),
            Ok(Inst32::Barrier { kind: Barrier::Isb })
        );
    }

    #[test]
    fn unknown_sysreg_decodes_to_other() {
        // mrs r0, #0x7f has no ARMv6-M meaning; decoded verbatim so the
        // executor can reject it with a typed error.
        assert_eq!(
            decode32_bytes([0xef, 0xf3, 0x7f, 0x80]),
            Ok(Inst32::Mrs {
                rd: 0,
                sys: SysReg::Other(0x7f)
            })
        );
    }

    #[test]
    fn v7m_ldm_stm() {
        // push.w {r0-r3, lr} -> e92d 400f (STMDB sp!, write-back).
        assert_eq!(
            v7m_bytes([0x2d, 0xe9, 0x0f, 0x40]),
            Ok(Inst32::LdmStm {
                is_load: false,
                rn: 13,
                regs: 0x400F,
                writeback: true,
                dec_before: true
            })
        );
        // pop.w {r0-r3, pc} -> e8bd 800f (LDMIA sp!, write-back).
        assert_eq!(
            v7m_bytes([0xbd, 0xe8, 0x0f, 0x80]),
            Ok(Inst32::LdmStm {
                is_load: true,
                rn: 13,
                regs: 0x800F,
                writeback: true,
                dec_before: false
            })
        );
        // stmia r0, {r1, r2} (no write-back) -> e880 0006.
        assert_eq!(
            v7m_bytes([0x80, 0xe8, 0x06, 0x00]),
            Ok(Inst32::LdmStm {
                is_load: false,
                rn: 0,
                regs: 0x0006,
                writeback: false,
                dec_before: false
            })
        );
        // ldmdb r2!, {r3, r4, r5} -> e932 0038.
        assert_eq!(
            v7m_bytes([0x32, 0xe9, 0x38, 0x00]),
            Ok(Inst32::LdmStm {
                is_load: true,
                rn: 2,
                regs: 0x0038,
                writeback: true,
                dec_before: true
            })
        );
        // ldmia r0!, {r0, r1} (rn in the list, but the lowest) -> e8b0 0003.
        assert_eq!(
            v7m_bytes([0xb0, 0xe8, 0x03, 0x00]),
            Ok(Inst32::LdmStm {
                is_load: true,
                rn: 0,
                regs: 0x0003,
                writeback: true,
                dec_before: false
            })
        );
    }

    #[test]
    fn v7m_ldm_stm_undefined() {
        // STMIA through the PC base is UNPREDICTABLE.
        assert!(v7m_bytes([0x8f, 0xe8, 0x0f, 0x00]).is_err());
        // STM storing PC is UNPREDICTABLE.
        assert!(v7m_bytes([0x80, 0xe8, 0x0f, 0x80]).is_err());
        // LDM with a PC base is UNDEFINED.
        assert!(v7m_bytes([0x9f, 0xe8, 0x0f, 0x00]).is_err());
        // An empty register list is UNPREDICTABLE.
        assert!(v7m_bytes([0x80, 0xe8, 0x00, 0x00]).is_err());
        // Write-back with rn not the lowest register in the list is
        // UNPREDICTABLE (ldmia r1!, {r0, r1, r2}).
        assert!(v7m_bytes([0xb1, 0xe8, 0x07, 0x00]).is_err());
    }

    #[test]
    fn v7m_tbranch() {
        // tbb [r2, r5] -> e8d2 f005.
        assert_eq!(
            v7m_bytes([0xd2, 0xe8, 0x05, 0xf0]),
            Ok(Inst32::TblBranch {
                rn: 2,
                rm: 5,
                half: false
            })
        );
        // tbh [r0, r3] -> e8d0 f013.
        assert_eq!(
            v7m_bytes([0xd0, 0xe8, 0x13, 0xf0]),
            Ok(Inst32::TblBranch {
                rn: 0,
                rm: 3,
                half: true
            })
        );
    }

    #[test]
    fn v7m_divide() {
        // sdiv r0, r0, r1 -> fb90 f0f1.
        assert_eq!(
            v7m_bytes([0x90, 0xfb, 0xf1, 0xf0]),
            Ok(Inst32::Div {
                signed: true,
                rd: 0,
                rn: 0,
                rm: 1
            })
        );
        // udiv r2, r4, r5 -> fbb4 f2f5.
        assert_eq!(
            v7m_bytes([0xb4, 0xfb, 0xf5, 0xf2]),
            Ok(Inst32::Div {
                signed: false,
                rd: 2,
                rn: 4,
                rm: 5
            })
        );
        // sdiv r3, r9, r8 -> fb99 f3f8.
        assert_eq!(
            v7m_bytes([0x99, 0xfb, 0xf8, 0xf3]),
            Ok(Inst32::Div {
                signed: true,
                rd: 3,
                rn: 9,
                rm: 8
            })
        );
        // A malformed second halfword (bits 7:4 of the lo not `1111`) is
        // UNDEFINED, and the ARMv6-M decoder must not swallow the family.
        assert!(v7m_bytes([0x90, 0xfb, 0xe0, 0xf0]).is_err());
        assert!(decode32_bytes([0x90, 0xfb, 0xf1, 0xf0]).is_err());
        assert!(decode32_bytes([0xb4, 0xfb, 0xf5, 0xf2]).is_err());
    }

    #[test]
    fn v7m_multiply() {
        // mul r3, r9, r8 -> fb09 f308: the `Ra` field is all ones, which is
        // the `MUL` marker over `MLA`.
        assert_eq!(
            v7m_bytes([0x09, 0xfb, 0x08, 0xf3]),
            Ok(Inst32::Mul {
                op: MulOp::Mul,
                rd: 3,
                rn: 9,
                rm: 8,
                ra: 0xF,
            })
        );
        // mla r0, r1, r2, r3 -> fb01 3002: `Ra` in the second halfword top.
        assert_eq!(
            v7m_bytes([0x01, 0xfb, 0x02, 0x30]),
            Ok(Inst32::Mul {
                op: MulOp::Mla,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        // mls r1, r2, r3, r4 -> fb02 4113: bit 4 of the lo selects `MLS`.
        assert_eq!(
            v7m_bytes([0x02, 0xfb, 0x13, 0x41]),
            Ok(Inst32::Mul {
                op: MulOp::Mls,
                rd: 1,
                rn: 2,
                rm: 3,
                ra: 4,
            })
        );
        // smull r0, r1, r2, r3 -> fb82 0103, with RdHi=rd (bits11:8) and
        // RdLo=ra (bits15:12) in the lo.
        assert_eq!(
            v7m_bytes([0x82, 0xfb, 0x03, 0x01]),
            Ok(Inst32::Mul {
                op: MulOp::Smull,
                rd: 1,
                rn: 2,
                rm: 3,
                ra: 0,
            })
        );
        // umull r0, r1, r2, r3 -> fba2 0103.
        assert_eq!(
            v7m_bytes([0xa2, 0xfb, 0x03, 0x01]),
            Ok(Inst32::Mul {
                op: MulOp::Umull,
                rd: 1,
                rn: 2,
                rm: 3,
                ra: 0,
            })
        );
        // smlal r2, r3, r4, r5 -> fbc4 2305, accumulating into r3:r2.
        assert_eq!(
            v7m_bytes([0xc4, 0xfb, 0x05, 0x23]),
            Ok(Inst32::Mul {
                op: MulOp::Smlal,
                rd: 3,
                rn: 4,
                rm: 5,
                ra: 2,
            })
        );
        // umlal r2, r3, r4, r5 -> fbe4 2305.
        assert_eq!(
            v7m_bytes([0xe4, 0xfb, 0x05, 0x23]),
            Ok(Inst32::Mul {
                op: MulOp::Umlal,
                rd: 3,
                rn: 4,
                rm: 5,
                ra: 2,
            })
        );
        // A malformed selector is UNDEFINED, and ARMv6-M never decodes the
        // multiply space even when the second halfword looks plausible.
        assert!(v7m_bytes([0x00, 0xfb, 0x20, 0xf0]).is_err());
        assert!(decode32_bytes([0x01, 0xfb, 0x02, 0x30]).is_err());
        assert!(decode32_bytes([0x82, 0xfb, 0x03, 0x10]).is_err());
    }

    #[test]
    fn v7m_dsp_mul() {
        // smulbb r0, r1, r2 -> fb11 f002; the b/t suffix flips the low nibble
        // of the lo halfword (bit 4 selects the m half, bit 5 the n half).
        assert_eq!(
            v7m_bytes([0x11, 0xfb, 0x02, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smulbb,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        assert_eq!(
            v7m_bytes([0x11, 0xfb, 0x12, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smulbt,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        assert_eq!(
            v7m_bytes([0x11, 0xfb, 0x22, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smultb,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        assert_eq!(
            v7m_bytes([0x11, 0xfb, 0x32, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smultt,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        // smulwb r0, r1, r2 -> fb31 f002; smulwt r0, r1, r2 -> fb31 f012.
        assert_eq!(
            v7m_bytes([0x31, 0xfb, 0x02, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smulwb,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        assert_eq!(
            v7m_bytes([0x31, 0xfb, 0x12, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smulwt,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        // smmul r0, r1, r2 -> fb51 f002; smmulr r0, r1, r2 -> fb51 f012.
        assert_eq!(
            v7m_bytes([0x51, 0xfb, 0x02, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smmul,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        assert_eq!(
            v7m_bytes([0x51, 0xfb, 0x12, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smmulr,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        // usad8 r0, r1, r2 -> fb71 f002; usada8 r0, r1, r2, r3 -> fb71 3002
        // with the accumulation register in bits 15:12 of the lo halfword.
        assert_eq!(
            v7m_bytes([0x71, 0xfb, 0x02, 0xf0]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Usad8,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        assert_eq!(
            v7m_bytes([0x71, 0xfb, 0x02, 0x30]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Usada8,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        // A couple of non-zero register combinations: smulbb r5, r6, r7 and
        // smulwt r8, r9, r10.
        assert_eq!(
            v7m_bytes([0x16, 0xfb, 0x07, 0xf5]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smulbb,
                rd: 5,
                rn: 6,
                rm: 7,
                ra: 0xf,
            })
        );
        assert_eq!(
            v7m_bytes([0x39, 0xfb, 0x1a, 0xf8]),
            Ok(Inst32::DspMul {
                op: DspMulOp::Smulwt,
                rd: 8,
                rn: 9,
                rm: 10,
                ra: 0xf,
            })
        );
        // The reserved selector (lo bits 6:4 = 0b101) is UNDEFINED, and the
        // DSP zone never decodes on ARMv6-M.
        assert!(v7m_bytes([0x11, 0xfb, 0x52, 0xf0]).is_err());
        assert!(decode32_bytes([0x11, 0xfb, 0x02, 0xf0]).is_err());
    }

    #[test]
    fn v7m_dual_mul() {
        // smlad r0, r1, r2, r3 -> fb21 3002; the X form flips lo bit 4.
        assert_eq!(
            v7m_bytes([0x21, 0xfb, 0x02, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlad,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        assert_eq!(
            v7m_bytes([0x21, 0xfb, 0x12, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smladx,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        // smuad r4, r5, r6 -> fb25 6004: the all-ones Ra is the sentinel.
        assert_eq!(
            v7m_bytes([0x25, 0xfb, 0x06, 0xf4]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlad,
                rd: 4,
                rn: 5,
                rm: 6,
                ra: 0xf,
            })
        );
        // smlsd r0, r1, r2, r3 -> fb41 3002, and smusd via the sentinel.
        assert_eq!(
            v7m_bytes([0x41, 0xfb, 0x02, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlsd,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        assert_eq!(
            v7m_bytes([0x41, 0xfb, 0x02, 0xf0]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlsd,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 0xf,
            })
        );
        // smlabb/smlabt/smlatb/smlatt r0, r1, r2, r3 -> fb11 3002 + lo bit 4.
        for (lo, op) in [
            (0x02, DualMulOp::Smlabb),
            (0x12, DualMulOp::Smlabt),
            (0x22, DualMulOp::Smlatb),
            (0x32, DualMulOp::Smlatt),
        ] {
            assert_eq!(
                v7m_bytes([0x11, 0xfb, lo, 0x30]),
                Ok(Inst32::DualMul {
                    op,
                    rd: 0,
                    rn: 1,
                    rm: 2,
                    ra: 3,
                })
            );
        }
        // smlawb/smlawt r0, r1, r2, r3 -> fb31 3002 / fb31 3012.
        assert_eq!(
            v7m_bytes([0x31, 0xfb, 0x02, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlawb,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        assert_eq!(
            v7m_bytes([0x31, 0xfb, 0x12, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlawt,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        // smmla/smmlar/smmls/smmlsr r0, r1, r2, r3.
        assert_eq!(
            v7m_bytes([0x51, 0xfb, 0x02, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smmla,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        assert_eq!(
            v7m_bytes([0x51, 0xfb, 0x12, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smmlar,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        assert_eq!(
            v7m_bytes([0x61, 0xfb, 0x02, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smmls,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        assert_eq!(
            v7m_bytes([0x61, 0xfb, 0x12, 0x30]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smmlsr,
                rd: 0,
                rn: 1,
                rm: 2,
                ra: 3,
            })
        );
        // The long forms pack RdLo=ra (bits15:12) and RdHi=rd (bits11:8).
        // smlald r2, r1, r3, r4 -> fbc3 21c4; smlaldx flips lo bit 4.
        assert_eq!(
            v7m_bytes([0xc3, 0xfb, 0xc4, 0x21]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlald,
                rd: 1,
                rn: 3,
                rm: 4,
                ra: 2,
            })
        );
        assert_eq!(
            v7m_bytes([0xc3, 0xfb, 0xd4, 0x21]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlaldx,
                rd: 1,
                rn: 3,
                rm: 4,
                ra: 2,
            })
        );
        // smlsld r2, r1, r3, r4 -> fbd3 21c4.
        assert_eq!(
            v7m_bytes([0xd3, 0xfb, 0xc4, 0x21]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlsld,
                rd: 1,
                rn: 3,
                rm: 4,
                ra: 2,
            })
        );
        assert_eq!(
            v7m_bytes([0xd3, 0xfb, 0xd4, 0x21]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Smlsldx,
                rd: 1,
                rn: 3,
                rm: 4,
                ra: 2,
            })
        );
        // smlalbb/smlalbt/smlaltb/smlaltt r2, r1, r3, r4 -> fbc3 2184 + sel.
        for (lo, op) in [
            (0x84, DualMulOp::Smlalbb),
            (0x94, DualMulOp::Smlalbt),
            (0xa4, DualMulOp::Smlaltb),
            (0xb4, DualMulOp::Smlaltt),
        ] {
            assert_eq!(
                v7m_bytes([0xc3, 0xfb, lo, 0x21]),
                Ok(Inst32::DualMul {
                    op,
                    rd: 1,
                    rn: 3,
                    rm: 4,
                    ra: 2,
                })
            );
        }
        // umaal r2, r1, r3, r4 -> fbe3 2164.
        assert_eq!(
            v7m_bytes([0xe3, 0xfb, 0x64, 0x21]),
            Ok(Inst32::DualMul {
                op: DualMulOp::Umaal,
                rd: 1,
                rn: 3,
                rm: 4,
                ra: 2,
            })
        );
        // SMMLS has no `Ra == 1111` form, so that combination is UNDEFINED;
        // the dual zone never decodes on ARMv6-M.
        assert!(v7m_bytes([0x61, 0xfb, 0x02, 0xf0]).is_err());
        assert!(v7m_bytes([0x21, 0xfb, 0x42, 0x30]).is_err());
        assert!(decode32_bytes([0x20, 0xfb, 0x02, 0x30]).is_err());
    }

    #[test]
    fn v7m_unary() {
        // rev r0, r1 -> fa91 f081; rev16 r2, r3 -> fa93 f293.
        assert_eq!(
            v7m_bytes([0x91, 0xfa, 0x81, 0xf0]),
            Ok(Inst32::Unary {
                op: UnaryOp::Rev,
                rd: 0,
                rm: 1,
            })
        );
        assert_eq!(
            v7m_bytes([0x93, 0xfa, 0x93, 0xf2]),
            Ok(Inst32::Unary {
                op: UnaryOp::Rev16,
                rd: 2,
                rm: 3,
            })
        );
        // revsh r4, r5 -> fa95 f4b5; rbit r6, r7 -> fa97 f6a7.
        assert_eq!(
            v7m_bytes([0x95, 0xfa, 0xb5, 0xf4]),
            Ok(Inst32::Unary {
                op: UnaryOp::Revsh,
                rd: 4,
                rm: 5,
            })
        );
        assert_eq!(
            v7m_bytes([0x97, 0xfa, 0xa7, 0xf6]),
            Ok(Inst32::Unary {
                op: UnaryOp::Rbit,
                rd: 6,
                rm: 7,
            })
        );
        // clz r8, r9 -> fab9 f889.
        assert_eq!(
            v7m_bytes([0xb9, 0xfa, 0x89, 0xf8]),
            Ok(Inst32::Unary {
                op: UnaryOp::Clz,
                rd: 8,
                rm: 9,
            })
        );
        // The `Rm2` field (first halfword's low nibble) must copy `Rm`; a
        // mismatch is architecturally UNPREDICTABLE and stays UNDEFINED.
        assert!(v7m_bytes([0x90, 0xfa, 0xb5, 0xf4]).is_err());
        // A reserved selector combination stays UNDEFINED, and the ARMv6-M
        // decoder rejects the whole `1111 1010` unary zone.
        assert!(v7m_bytes([0x90, 0xfa, 0x70, 0xf0]).is_err());
        assert!(decode32_bytes([0x9a, 0xfa, 0x06, 0xf0]).is_err());
        assert!(decode32_bytes([0xba, 0xfa, 0x08, 0xf0]).is_err());
    }

    #[test]
    fn v7m_qadd() {
        // qadd r0, r1, r2 -> fa81 f082.
        assert_eq!(
            v7m_bytes([0x81, 0xfa, 0x82, 0xf0]),
            Ok(Inst32::Qadd {
                op: QaddOp::Qadd,
                rd: 0,
                rn: 1,
                rm: 2,
            })
        );
        // qsub r3, r4, r5 -> fa84 f2a5.
        assert_eq!(
            v7m_bytes([0x84, 0xfa, 0xa5, 0xf3]),
            Ok(Inst32::Qadd {
                op: QaddOp::Qsub,
                rd: 3,
                rn: 4,
                rm: 5,
            })
        );
        // qdadd r6, r7, r8 -> fa87 f198.
        assert_eq!(
            v7m_bytes([0x87, 0xfa, 0x98, 0xf6]),
            Ok(Inst32::Qadd {
                op: QaddOp::Qdadd,
                rd: 6,
                rn: 7,
                rm: 8,
            })
        );
        // qdsub r9, r10, r11 -> fa8a f3bb.
        assert_eq!(
            v7m_bytes([0x8a, 0xfa, 0xbb, 0xf9]),
            Ok(Inst32::Qadd {
                op: QaddOp::Qdsub,
                rd: 9,
                rn: 10,
                rm: 11,
            })
        );
        // The `1111 Rd` top nibble of the second halfword is fixed; a bogus
        // selector or top nibble stays UNDEFINED. A zero selector is now a
        // valid `SADD8`, so the bogus cases use the reserved `0b011` nibble.
        assert!(v7m_bytes([0x81, 0xfa, 0xc2, 0xf0]).is_err());
        assert!(v7m_bytes([0x81, 0xfa, 0x72, 0xf0]).is_err());
    }

    #[test]
    fn v7m_parallel_add_sub() {
        // sadd8 r0, r1, r2 -> fa81 f002.
        assert_eq!(
            v7m_bytes([0x81, 0xfa, 0x02, 0xf0]),
            Ok(Inst32::Par {
                op: ParOp::Add8,
                sat: ParSat::S,
                rd: 0,
                rn: 1,
                rm: 2,
            })
        );
        // qadd16 r3, r4, r5 -> fa94 f315.
        assert_eq!(
            v7m_bytes([0x94, 0xfa, 0x15, 0xf3]),
            Ok(Inst32::Par {
                op: ParOp::Add16,
                sat: ParSat::Q,
                rd: 3,
                rn: 4,
                rm: 5,
            })
        );
        // shsub8 r6, r7, r8 -> fac7 f628.
        assert_eq!(
            v7m_bytes([0xc7, 0xfa, 0x28, 0xf6]),
            Ok(Inst32::Par {
                op: ParOp::Sub8,
                sat: ParSat::Sh,
                rd: 6,
                rn: 7,
                rm: 8,
            })
        );
        // uadd16 r9, r10, r11 -> fa9a f94b.
        assert_eq!(
            v7m_bytes([0x9a, 0xfa, 0x4b, 0xf9]),
            Ok(Inst32::Par {
                op: ParOp::Add16,
                sat: ParSat::U,
                rd: 9,
                rn: 10,
                rm: 11,
            })
        );
        // uqsub8 r12, r13, r14 -> facd fc5e.
        assert_eq!(
            v7m_bytes([0xcd, 0xfa, 0x5e, 0xfc]),
            Ok(Inst32::Par {
                op: ParOp::Sub8,
                sat: ParSat::Uq,
                rd: 12,
                rn: 13,
                rm: 14,
            })
        );
        // uhadd16 r0, r1, r2 -> fa91 f062.
        assert_eq!(
            v7m_bytes([0x91, 0xfa, 0x62, 0xf0]),
            Ok(Inst32::Par {
                op: ParOp::Add16,
                sat: ParSat::Uh,
                rd: 0,
                rn: 1,
                rm: 2,
            })
        );
        // sasx r2, r3, r4 -> faa3 f204.
        assert_eq!(
            v7m_bytes([0xa3, 0xfa, 0x04, 0xf2]),
            Ok(Inst32::Par {
                op: ParOp::Asx,
                sat: ParSat::S,
                rd: 2,
                rn: 3,
                rm: 4,
            })
        );
        // ssax r5, r6, r7 -> fae6 f507.
        assert_eq!(
            v7m_bytes([0xe6, 0xfa, 0x07, 0xf5]),
            Ok(Inst32::Par {
                op: ParOp::Sax,
                sat: ParSat::S,
                rd: 5,
                rn: 6,
                rm: 7,
            })
        );
        // sel r0, r1, r2 -> faa1 f082; sel r3, r4, r5 -> faa4 f385.
        assert_eq!(
            v7m_bytes([0xa1, 0xfa, 0x82, 0xf0]),
            Ok(Inst32::Sel {
                rd: 0,
                rn: 1,
                rm: 2,
            })
        );
        assert_eq!(
            v7m_bytes([0xa4, 0xfa, 0x85, 0xf3]),
            Ok(Inst32::Sel {
                rd: 3,
                rn: 4,
                rm: 5,
            })
        );
        // The first halfword's low nibble is `Rn`, not the `Rm2` copy the
        // unary ops require: sadd16 r1, r0, r15 -> fa90 f10f decodes anyway.
        assert_eq!(
            v7m_bytes([0x90, 0xfa, 0x0f, 0xf1]),
            Ok(Inst32::Par {
                op: ParOp::Add16,
                sat: ParSat::S,
                rd: 1,
                rn: 0,
                rm: 15,
            })
        );
        // The `SEL` selector is only valid on the ASX family: the same `sel`
        // on the ADD16 family stays reserved for the unary ops.
        assert_eq!(
            v7m_bytes([0x91, 0xfa, 0x81, 0xf0]),
            Ok(Inst32::Unary {
                op: UnaryOp::Rev,
                rd: 0,
                rm: 1,
            })
        );
        // A reserved selector nibble (0b011) is UNDEFINED.
        assert!(v7m_bytes([0x81, 0xfa, 0x72, 0xf0]).is_err());
        // The parallel family never decodes on ARMv6-M.
        assert!(decode32_bytes([0x81, 0xfa, 0x02, 0xf0]).is_err());
    }

    #[test]
    fn v7m_extend_add() {
        // sxth r0, r1 (sxtah with rn == 15) -> fa0f f081.
        assert_eq!(
            v7m_bytes([0x0f, 0xfa, 0x81, 0xf0]),
            Ok(Inst32::ExtendAdd {
                op: ExtendAddOp::Sxtah,
                rd: 0,
                rn: 15,
                rm: 1,
                rot: 0,
            })
        );
        // uxtah r2, r3, r4 -> fa13 f284.
        assert_eq!(
            v7m_bytes([0x13, 0xfa, 0x84, 0xf2]),
            Ok(Inst32::ExtendAdd {
                op: ExtendAddOp::Uxtah,
                rd: 2,
                rn: 3,
                rm: 4,
                rot: 0,
            })
        );
        // sxtab16 r5, r6, r7, ror #8 -> fa26 f597.
        assert_eq!(
            v7m_bytes([0x26, 0xfa, 0x97, 0xf5]),
            Ok(Inst32::ExtendAdd {
                op: ExtendAddOp::Sxtab16,
                rd: 5,
                rn: 6,
                rm: 7,
                rot: 1,
            })
        );
        // uxtab16 r8, r9, r10 -> fa39 fa8a.
        assert_eq!(
            v7m_bytes([0x39, 0xfa, 0x8a, 0xf8]),
            Ok(Inst32::ExtendAdd {
                op: ExtendAddOp::Uxtab16,
                rd: 8,
                rn: 9,
                rm: 10,
                rot: 0,
            })
        );
        // sxtab r1, r2, r3, ror #24 -> fa42 f1b3.
        assert_eq!(
            v7m_bytes([0x42, 0xfa, 0xb3, 0xf1]),
            Ok(Inst32::ExtendAdd {
                op: ExtendAddOp::Sxtab,
                rd: 1,
                rn: 2,
                rm: 3,
                rot: 3,
            })
        );
        // uxtb r4, r5 (uxtab with rn == 15) -> fa5f f485.
        assert_eq!(
            v7m_bytes([0x5f, 0xfa, 0x85, 0xf4]),
            Ok(Inst32::ExtendAdd {
                op: ExtendAddOp::Uxtab,
                rd: 4,
                rn: 15,
                rm: 5,
                rot: 0,
            })
        );
        // A reserved op nibble (`0xFA6x`) and a second-halfword shape with
        // bit 6 set stay UNDEFINED; ARMv6-M rejects the whole family.
        assert!(v7m_bytes([0x60, 0xfa, 0x80, 0xf0]).is_err());
        assert!(v7m_bytes([0x01, 0xfa, 0xc1, 0xf0]).is_err());
        assert!(decode32_bytes([0x0f, 0xfa, 0x81, 0xf0]).is_err());
    }

    #[test]
    fn v7m_saturate() {
        // ssat r0, #8, r1, lsl #4 -> f301 1007: `sh` clear, LSL shift
        // `imm3:imm2` = 4, `sat_imm` = 7 (8 bits).
        assert_eq!(
            v7m_bytes([0x01, 0xf3, 0x07, 0x10]),
            Ok(Inst32::Sat {
                op: SatOp::Ssat,
                rd: 0,
                rn: 1,
                sat_n: 8,
                sh: false,
                shift: 4,
            })
        );
        // ssat r0, #16, r1, asr #2 -> f321 008f: `sh` set, ASR #2.
        assert_eq!(
            v7m_bytes([0x21, 0xf3, 0x8f, 0x00]),
            Ok(Inst32::Sat {
                op: SatOp::Ssat,
                rd: 0,
                rn: 1,
                sat_n: 16,
                sh: true,
                shift: 2,
            })
        );
        // usat r2, #8, r3, lsl #2 -> f383 0287.
        assert_eq!(
            v7m_bytes([0x83, 0xf3, 0x87, 0x02]),
            Ok(Inst32::Sat {
                op: SatOp::Usat,
                rd: 2,
                rn: 3,
                sat_n: 8,
                sh: false,
                shift: 2,
            })
        );
        // ssat16 r4, #8, r5 -> f325 0407: the ASR-with-zero-shift alias.
        assert_eq!(
            v7m_bytes([0x25, 0xf3, 0x07, 0x04]),
            Ok(Inst32::Sat {
                op: SatOp::Ssat16,
                rd: 4,
                rn: 5,
                sat_n: 8,
                sh: false,
                shift: 0,
            })
        );
        // usat16 r6, #4, r7 -> f3a7 0603.
        assert_eq!(
            v7m_bytes([0xa7, 0xf3, 0x03, 0x06]),
            Ok(Inst32::Sat {
                op: SatOp::Usat16,
                rd: 6,
                rn: 7,
                sat_n: 4,
                sh: false,
                shift: 0,
            })
        );
        // The second halfword's fixed bit 5 is enforced, and the `1111 0011`
        // first-halfword bit 4 must be clear (otherwise the encoding belongs
        // to neither SAT nor the bitfield family). A second halfword with bit
        // 15 set is claimed by the `B<c>.W` branch decoder first, so real
        // SATs always arrive here with bit 15 clear.
        assert!(v7m_bytes([0x01, 0xf3, 0x27, 0x00]).is_err());
        assert!(v7m_bytes([0x11, 0xf3, 0x07, 0x00]).is_err());
    }

    #[test]
    fn v7m_bitfield() {
        // sbfx r0, r1, #8, #8 -> f341 2007: `Rn` in the first halfword's low
        // nibble, `imm3:imm2` (= 2:0) building the 8-bit `lsb` in the second.
        assert_eq!(
            v7m_bytes([0x41, 0xf3, 0x07, 0x20]),
            Ok(Inst32::Bitfield {
                op: BitfieldOp::Sbfx,
                rd: 0,
                rn: 1,
                lsb: 8,
                field: 7,
            })
        );
        // sbfx r2, r3, #8, #8 -> f343 2207.
        assert_eq!(
            v7m_bytes([0x43, 0xf3, 0x07, 0x22]),
            Ok(Inst32::Bitfield {
                op: BitfieldOp::Sbfx,
                rd: 2,
                rn: 3,
                lsb: 8,
                field: 7,
            })
        );
        // ubfx r0, r1, #4, #8 -> f3c1 1007: `imm3` = 1, `imm2` = 0.
        assert_eq!(
            v7m_bytes([0xc1, 0xf3, 0x07, 0x10]),
            Ok(Inst32::Bitfield {
                op: BitfieldOp::Ubfx,
                rd: 0,
                rn: 1,
                lsb: 4,
                field: 7,
            })
        );
        // bfi r4, r5, #8, #8 -> f365 240f: `field` is the `msb` (15).
        assert_eq!(
            v7m_bytes([0x65, 0xf3, 0x0f, 0x24]),
            Ok(Inst32::Bitfield {
                op: BitfieldOp::Bfi,
                rd: 4,
                rn: 5,
                lsb: 8,
                field: 0xF,
            })
        );
        // bfc r6, #8, #8 -> f36f 260f: `Rn == 15` marks the clear form.
        assert_eq!(
            v7m_bytes([0x6f, 0xf3, 0x0f, 0x26]),
            Ok(Inst32::Bitfield {
                op: BitfieldOp::Bfc,
                rd: 6,
                rn: 0xF,
                lsb: 8,
                field: 0xF,
            })
        );
        // An extract whose field runs off the top of the word (lsb 24, width
        // 16), a `BFI` with `msb < lsb`, and a second halfword with a
        // reserved fixed bit set are all UNDEFINED; the ARMv6-M decoder
        // rejects the whole `1111 0011` bitfield space.
        assert!(v7m_bytes([0xc1, 0xf3, 0x0f, 0x60]).is_err());
        assert!(v7m_bytes([0x61, 0xf3, 0x0f, 0x40]).is_err());
        assert!(v7m_bytes([0x41, 0xf3, 0x27, 0x20]).is_err());
        assert!(decode32_bytes([0x41, 0xf3, 0x07, 0x20]).is_err());
    }

    #[test]
    fn v7m_ldrd_strd() {
        // ldrd r2, r3, [r0, #8] -> e9d0 2302: `P U W L` in the first
        // halfword, `Rt Rt2 imm8` (byte offset `imm8 << 2`) in the second.
        assert_eq!(
            v7m_bytes([0xd0, 0xe9, 0x02, 0x23]),
            Ok(Inst32::LdrdStrd {
                is_load: true,
                rt: 2,
                rt2: 3,
                rn: 0,
                imm: 8,
                add: true,
                index: true,
                writeback: false,
            })
        );
        // strd r4, r5, [r1, #-4] -> e941 4501: a negative offset clears `U`.
        assert_eq!(
            v7m_bytes([0x41, 0xe9, 0x01, 0x45]),
            Ok(Inst32::LdrdStrd {
                is_load: false,
                rt: 4,
                rt2: 5,
                rn: 1,
                imm: 4,
                add: false,
                index: true,
                writeback: false,
            })
        );
        // ldrd r0, r1, [r2], #8 -> e8f2 0102: the post-indexed form clears
        // `P` and sets `W`.
        assert_eq!(
            v7m_bytes([0xf2, 0xe8, 0x02, 0x01]),
            Ok(Inst32::LdrdStrd {
                is_load: true,
                rt: 0,
                rt2: 1,
                rn: 2,
                imm: 8,
                add: true,
                index: false,
                writeback: true,
            })
        );
        // strd r6, r7, [r3, #16]! -> e9e3 6704: pre-indexed with writeback.
        assert_eq!(
            v7m_bytes([0xe3, 0xe9, 0x04, 0x67]),
            Ok(Inst32::LdrdStrd {
                is_load: false,
                rt: 6,
                rt2: 7,
                rn: 3,
                imm: 16,
                add: true,
                index: true,
                writeback: true,
            })
        );
        // ldrd r0, r1, [pc, #8] -> e9df 0102: `Rn == 15` is the literal form.
        assert_eq!(
            v7m_bytes([0xdf, 0xe9, 0x02, 0x01]),
            Ok(Inst32::LdrdStrd {
                is_load: true,
                rt: 0,
                rt2: 1,
                rn: 0xF,
                imm: 8,
                add: true,
                index: true,
                writeback: false,
            })
        );
        // A doubled or PC destination register, a `STRD` PC base, writeback
        // onto a loaded register, and the `TBB`-adjacent `(P, W) == (0, 0)`
        // space are all UNPREDICTABLE and decode as UNDEFINED; the ARMv6-M
        // decoder rejects the whole `1110 100P U1WL` dual space.
        assert!(v7m_bytes([0xd1, 0xe9, 0x01, 0x00]).is_err()); // rt == rt2
        assert!(v7m_bytes([0xd0, 0xe9, 0x01, 0x1f]).is_err()); // rt == 15
        assert!(v7m_bytes([0x4f, 0xe9, 0x01, 0x01]).is_err()); // STRD [pc]
        assert!(v7m_bytes([0xf0, 0xe9, 0x01, 0x01]).is_err()); // wb over r0
                                                               // The `(P, W) == (0, 0)` load side of this space is UNDEFINED: the
                                                               // `0xE840`/`0xE850` first halfwords now belong to `STREX`/`LDREX`
                                                               // (see `v7m_exclusive`), so probe it with a non-exclusive pair.
        assert!(v7m_bytes([0x50, 0xe8, 0x01, 0x02]).is_err()); // LDRD P == W == 0
        assert!(decode32_bytes([0xd0, 0xe9, 0x02, 0x23]).is_err());
    }

    #[test]
    fn v7m_exclusive() {
        // strex r1, r0, [r2] -> e842 0100.
        assert_eq!(
            v7m_bytes([0x42, 0xe8, 0x00, 0x01]),
            Ok(Inst32::Strex {
                rd: 1,
                rt: 0,
                rn: 2,
                size: ExSize::Word,
                off: 0,
            })
        );
        // ldrex r0, [r2] -> e852 f000.
        assert_eq!(
            v7m_bytes([0x52, 0xe8, 0x00, 0x0f]),
            Ok(Inst32::Ldrex {
                rt: 0,
                rn: 2,
                size: ExSize::Word,
                off: 0,
            })
        );
        // The word forms carry an `imm8 << 2` offset: strex r1, r0, [r2, #8].
        assert_eq!(
            v7m_bytes([0x42, 0xe8, 0x02, 0x01]),
            Ok(Inst32::Strex {
                rd: 1,
                rt: 0,
                rn: 2,
                size: ExSize::Word,
                off: 8,
            })
        );
        // strexb r1, r0, [r2] -> e8c2 0f41: Rd sits in bits [3:0].
        assert_eq!(
            v7m_bytes([0xc2, 0xe8, 0x41, 0x0f]),
            Ok(Inst32::Strex {
                rd: 1,
                rt: 0,
                rn: 2,
                size: ExSize::Byte,
                off: 0,
            })
        );
        // strexh r1, r0, [r2] -> e8c2 0f51.
        assert_eq!(
            v7m_bytes([0xc2, 0xe8, 0x51, 0x0f]),
            Ok(Inst32::Strex {
                rd: 1,
                rt: 0,
                rn: 2,
                size: ExSize::Half,
                off: 0,
            })
        );
        // ldrexb r0, [r2] -> e8d2 0f4f.
        assert_eq!(
            v7m_bytes([0xd2, 0xe8, 0x4f, 0x0f]),
            Ok(Inst32::Ldrex {
                rt: 0,
                rn: 2,
                size: ExSize::Byte,
                off: 0,
            })
        );
        // ldrexh r0, [r2] -> e8d2 0f5f.
        assert_eq!(
            v7m_bytes([0xd2, 0xe8, 0x5f, 0x0f]),
            Ok(Inst32::Ldrex {
                rt: 0,
                rn: 2,
                size: ExSize::Half,
                off: 0,
            })
        );
        // clrex -> f3bf 8f2f, indistinguishable from a barrier until the
        // second halfword's `1111` tag.
        assert_eq!(v7m_bytes([0xbf, 0xf3, 0x2f, 0x8f]), Ok(Inst32::Clrex));
        // A `TBB` sharing the `0xE8D0` space still decodes (bits [11:8] = 0).
        assert_eq!(
            v7m_bytes([0xd2, 0xe8, 0x03, 0xf0]),
            Ok(Inst32::TblBranch {
                rn: 2,
                rm: 3,
                half: false,
            })
        );
        // Register constraints and the ARMv7-A/R-only doubleword forms are
        // UNDEFINED, and the whole family (plus CLREX) stays UNDEFINED on
        // ARMv6-M.
        assert!(v7m_bytes([0x42, 0xe8, 0x00, 0xf0]).is_err()); // STREX Rd == 15
        assert!(v7m_bytes([0x52, 0xe8, 0x00, 0xff]).is_err()); // LDREX Rt == 15
        assert!(v7m_bytes([0xc2, 0xe8, 0x4f, 0x0f]).is_err()); // STREXB Rd == 15
        assert!(v7m_bytes([0xd2, 0xe8, 0xcf, 0x0f]).is_err()); // LDAEXB (v8 only)
        assert!(v7m_bytes([0xc2, 0xe8, 0x70, 0x01]).is_err()); // STREXD
        assert!(decode32_bytes([0xc2, 0xe8, 0x41, 0x0f]).is_err());
        assert!(decode32_bytes([0xbf, 0xf3, 0x2f, 0x8f]).is_err()); // no CLREX on v6-M
    }
}
