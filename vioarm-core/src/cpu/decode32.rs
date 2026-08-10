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
//! family (all `LDR`/`STR` immediate sizes and the signed loads), verified
//! against `clang --target=thumbv7m`. Encoding ground truth lives in the test
//! suite.

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

/// A decoded 32-bit instruction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Inst32 {
    /// `BL {label}`. `off` is the already-sign-extended byte offset from
    /// `pc + 4`. LR is set to `pc + 4 | 1` by the executor.
    Bl {
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
    /// ARMv7-M single-data-transfer with an immediate offset: `LDR`/`STR`
    /// word, byte, halfword, and the sign-extending loads. `off` is the
    /// offset magnitude (0-4095 imm12 or 0-255 imm8); `add` selects `+`/`-`.
    /// `index`/`writeback` give the three addressing forms: offset
    /// (`index`, no writeback), pre-indexed (both), post-indexed (writeback
    /// only). `rn == 15` marks the PC-relative literal form (loads only),
    /// whose base is `Align(pc + 4, 4)`.
    Mem {
        access: MemOp,
        rt: u8,
        rn: u8,
        off: u32,
        add: bool,
        index: bool,
        writeback: bool,
    },
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
/// `addw`/`subw` immediates), and the immediate-offset load/store family are
/// layered on top. The memory decoders are consulted before the
/// data-processing set: their `F8xx`/`F9xx` first halfwords overlap the
/// modified-immediate mask (`hi & 0xF600`) and must win the match.
///
/// # Errors
///
/// Returns [`DecodeError`] when the halfwords do not form a known ARMv7-M
/// 32-bit instruction.
pub fn decode32_v7m(hi: u16, lo: u16) -> Result<Inst32, DecodeError> {
    decode_mrs(hi, lo)
        .or_else(|| decode_msr(hi, lo))
        .or_else(|| decode_barrier(hi, lo))
        .or_else(|| decode_bl(hi, lo))
        .or_else(|| decode_movwide(hi, lo))
        .or_else(|| decode_mem(hi, lo))
        .or_else(|| decode_dp_plain(hi, lo))
        .or_else(|| decode_dp_modified(hi, lo))
        .or_else(|| decode_dp_reg(hi, lo))
        .unwrap_or_else(|| Err(err_for(hi)))
}

/// Build the typed undefined-instruction error for a first halfword.
const fn err_for(hi: u16) -> DecodeError {
    DecodeError {
        opcode: hi,
        cause: HaltCause::UndefinedInstruction,
    }
}

/// decode the ARMv6/7-M `BL` immediate.
fn decode_bl(hi: u16, lo: u16) -> Option<Result<Inst32, DecodeError>> {
    // BL: 1111 0 S imm10 / 1111 1 J1 J2 imm11.
    if hi & 0xF800 != 0xF000 {
        return None;
    }
    if lo & 0xF800 != 0xF800 {
        // The `11110` first-halfword space is shared with the ARMv7-M
        // data-processing set; only a `11111`-shaped second halfword is BL.
        return None;
    }
    let s = i32::from((hi >> 10) & 1);
    let imm10 = i32::from(hi & 0x3FF);
    let j1 = i32::from((lo >> 13) & 1);
    let j2 = i32::from((lo >> 11) & 1);
    let imm11 = i32::from(lo & 0x7FF);
    let i1 = 1 - (j1 ^ s);
    let i2 = 1 - (j2 ^ s);
    // 25-bit signed field S:imm10:i1:i2:imm11:0, shift-sign-extended from
    // bit 24 (shifts wrap, so this is well-defined for the whole range).
    let off = ((s << 24) | (imm10 << 14) | (i1 << 13) | (i2 << 12) | (imm11 << 1)) << 7 >> 7;
    Some(Ok(Inst32::Bl { off }))
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

/// decode the ARMv7-M immediate-offset single-data-transfer family.
///
/// Two halfword layouts share the `1111 100x` first halfword:
/// the unsigned forms live at `F8xx`, the sign-extending loads at `F9xx`
/// (`bit8` is the sign bit). Each size/op has an imm12 form (offset only,
/// second-halfword `Rt imm12`) and an imm8 form (off-, pre-, post-indexed,
/// second-halfword `1 P U W Rt imm8`). `Rn == 15` in either form is the
/// PC-relative literal (loads only).
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
    if wide {
        let rt = (lo >> 12) as u8;
        if !is_load && (rn == 15 || rt == 15) {
            // STR to PC, or with a PC base, is architecturally UNPREDICTABLE.
            return err();
        }
        Some(Ok(Inst32::Mem {
            access,
            rt,
            rn,
            off: u32::from(lo & 0xFFF),
            add: true,
            index: true,
            writeback: false,
        }))
    } else {
        let p = lo & 0x4000 != 0;
        let u = lo & 0x2000 != 0;
        let w = lo & 0x1000 != 0;
        let rt = ((lo >> 8) & 0xF) as u8;
        let off = u32::from(lo & 0xFF);
        if !is_load && (rn == 15 || rt == 15) {
            return err();
        }
        if rn == 15 && w {
            // A PC-relative literal with write-back is UNPREDICTABLE.
            return err();
        }
        if rn != 15 && p && u && !w {
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
            add: u,
            index,
            writeback,
        }))
    }
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
    let Some(op) = dp_op(hi & 0x100 != 0, ((hi >> 5) & 0x7) as u8, rn, rd, set_flags)
    else {
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
    let Some(op) = dp_op(hi & 0x100 != 0, ((hi >> 5) & 0x7) as u8, rn, rd, set_flags)
    else {
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

    fn imm(
        op: DpOp,
        rd: u8,
        rn: u8,
        imm12: u16,
        expand: bool,
        set_flags: bool,
    ) -> Inst32 {
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
            add,
            index,
            writeback,
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
        // ldr.w r0, [r1, #-8] -> f851 c008 (imm8, offset).
        assert_eq!(
            v7m_bytes([0x51, 0xf8, 0x08, 0xc0]),
            Ok(mem(MemOp::Ldr, 0, 1, 8, false, true, false))
        );
        // ldr.w r0, [r1, #8]! -> f851 f008 (imm8, pre-indexed).
        assert_eq!(
            v7m_bytes([0x51, 0xf8, 0x08, 0xf0]),
            Ok(mem(MemOp::Ldr, 0, 1, 8, true, true, true))
        );
        // ldr.w r0, [r1], #8 -> f851 b008 (imm8, post-indexed).
        assert_eq!(
            v7m_bytes([0x51, 0xf8, 0x08, 0xb0]),
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
        // ldr.w r5, [pc, #4] -> f85f e504 (PC-relative literal, imm8).
        assert_eq!(
            v7m_bytes([0x5f, 0xf8, 0x04, 0xe5]),
            Ok(mem(MemOp::Ldr, 5, 15, 4, true, true, false))
        );
        // ldrsb.w r9, [r6, #-1] -> f916 c901 (imm8 offset, subtract).
        assert_eq!(
            v7m_bytes([0x16, 0xf9, 0x01, 0xc9]),
            Ok(mem(MemOp::Ldrsb, 9, 6, 1, false, true, false))
        );
    }

    #[test]
    fn v7m_load_store_undefined_cases() {
        // A store through the PC base is UNPREDICTABLE.
        assert!(v7m_bytes([0xcf, 0xf8, 0x04, 0x00]).is_err());
        // A store to PC is UNPREDICTABLE.
        assert!(v7m_bytes([0xc1, 0xf8, 0x04, 0xf0]).is_err());
        // P == 0 && W == 0 in the imm8 form is UNDEFINED.
        assert!(v7m_bytes([0x51, 0xf8, 0x00, 0x80]).is_err());
        // The unprivileged (LDRT/STRT) pattern is UNPREDICTABLE.
        assert!(v7m_bytes([0x51, 0xf8, 0x00, 0xe0]).is_err());
        // A PC-relative literal with write-back is UNPREDICTABLE.
        assert!(v7m_bytes([0x5f, 0xf8, 0x04, 0xf5]).is_err());
        // The ARMv6-M decoder must not swallow the ARMv7-M memory set.
        assert!(decode32_bytes([0x51, 0xf8, 0x08, 0xb0]).is_err());
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
}
