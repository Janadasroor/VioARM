//! Decoding of 16-bit ARMv6-M (Thumb) instructions.
//!
//! Every encoding in this file is verified against
//! `llvm-mc -triple=thumbv6m-none-eabi` in the test suite. ARMv6-M has no IT
//! block, no `cbz`/`cbnz` (ARMv8-M+), and no immediate `ldrsb`/`ldrsh` (those
//! are Thumb-2). The 32-bit ARMv6-M instructions (`bl`, `mrs`/`msr`, and the
//! barriers) live in `super::decode32`.

use super::decode::DecodeError;
use crate::error::HaltCause;

/// Load/store access type for register-offset addressing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LsReg {
    Str,
    Strh,
    Strb,
    Ldrsb,
    Ldr,
    Ldrh,
    Ldrb,
    Ldrsh,
}

/// Load/store access type for immediate-offset addressing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LsImm {
    Str,
    Ldr,
    Strb,
    Ldrb,
    Strh,
    Ldrh,
}

/// ALU register data-processing ops (0x4000 space).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AluOp {
    And,
    Eor,
    Lsl,
    Lsr,
    Asr,
    Adc,
    Sbc,
    Ror,
    Tst,
    Neg,
    Cmp,
    Cmn,
    Orr,
    Mul,
    Bic,
    Mvn,
}

/// Shift kinds for register-by-register shifts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Shift {
    Lsl,
    Lsr,
    Asr,
    Ror,
}

/// Shift with immediate amount (LSL/LSR/ASR #imm).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShiftImm {
    Lsl,
    Lsr,
    Asr,
}

/// Architecture hint instructions (`HINT #imm8`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hint {
    Nop,
    Yield,
    Wfe,
    Wfi,
    Sev,
}

/// Sign/zero extension (`SXTH`/`SXTB`/`UXTH`/`UXTB`). `half` selects the
/// 16-bit (`T*H`) form over the 8-bit (`T*B`) form.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtendKind {
    Sxth,
    Sxtb,
    Uxth,
    Uxtb,
}

/// Byte-reversal (`REV`/`REV16`/`REVSH`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReverseKind {
    Rev,
    Rev16,
    Revsh,
}

/// A decoded 16-bit instruction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Inst16 {
    ShiftImm {
        rd: u8,
        rm: u8,
        kind: ShiftImm,
        amount: u8,
    },
    ShiftReg {
        rd: u8,
        rn: u8,
        rm: u8,
        kind: Shift,
    },
    AddReg {
        rd: u8,
        rn: u8,
        rm: u8,
    },
    SubReg {
        rd: u8,
        rn: u8,
        rm: u8,
    },
    AddImm3 {
        rd: u8,
        rn: u8,
        imm: u8,
    },
    SubImm3 {
        rd: u8,
        rn: u8,
        imm: u8,
    },
    MovImm {
        rd: u8,
        imm: u8,
    },
    CmpImm {
        rn: u8,
        imm: u8,
    },
    AddImm8 {
        rd: u8,
        imm: u8,
    },
    SubImm8 {
        rd: u8,
        imm: u8,
    },
    Alu {
        rd: u8,
        rm: u8,
        op: AluOp,
    },
    AddSpImm8 {
        imm: usize,
    },
    SubSpImm8 {
        imm: usize,
    },
    /// High/Low ADD (no flags).
    SpecialAdd {
        rd: u8,
        rm: u8,
    },
    SpecialCmp {
        rn: u8,
        rm: u8,
    },
    SpecialMov {
        rd: u8,
        rm: u8,
    },
    Bx {
        rm: u8,
    },
    Blx {
        rm: u8,
    },
    LdrLiteral {
        rt: u8,
        imm: usize,
    },
    LsReg {
        rt: u8,
        rn: u8,
        rm: u8,
        kind: LsReg,
    },
    LsImm {
        rt: u8,
        rn: u8,
        imm: usize,
        kind: LsImm,
    },
    LsSp {
        rt: u8,
        imm: usize,
        is_load: bool,
    },
    AddrPc {
        rd: u8,
        imm: usize,
    },
    AddrSp {
        rd: u8,
        imm: usize,
    },
    /// Conditional branch `B<cond> imm8` (imm8 sign-extended to the PC
    /// offset in bytes).
    BranchCond {
        cond: u8,
        off: i32,
    },
    /// Unconditional branch `B imm11`.
    Branch {
        off: i32,
    },
    /// `CBZ`/`CBNZ`: compare `rn` against zero and take the branch when the
    /// condition matches. `off` is the byte offset from `pc + 4`.
    CbzCbnz {
        rn: u8,
        off: i32,
        nonzero: bool,
    },
    Svc {
        imm: u8,
    },
    Bkpt {
        imm: u8,
    },
    Hint {
        kind: Hint,
    },
    /// `CPSIE/CPSID im`; only the `i` bit (PRIMASK) is architecturally
    /// meaningful on ARMv6-M.
    Cps {
        disable: bool,
        im: u8,
    },
    /// `SXTH`/`SXTB`/`UXTH`/`UXTB Rd, Rm`.
    Extend {
        rd: u8,
        rm: u8,
        kind: ExtendKind,
    },
    /// `REV`/`REV16`/`REVSH Rd, Rm`.
    Reverse {
        rd: u8,
        rm: u8,
        kind: ReverseKind,
    },
    /// `PUSH {r0-r7}` plus `lr` when `include_lr`.
    Push {
        regs: u8,
        include_lr: bool,
    },
    /// `POP {r0-r7}` plus `pc` when `load_pc`.
    Pop {
        regs: u8,
        load_pc: bool,
    },
    /// `STMIA rn!, {list}` / `LDMIA rn!, {list}`. Writeback is mandatory in
    /// ARMv6-M.
    LdmStm {
        is_load: bool,
        rn: u8,
        regs: u8,
    },
}

pub fn decode16(op: u16) -> Result<Inst16, DecodeError> {
    let err = || DecodeError {
        opcode: op,
        cause: HaltCause::UndefinedInstruction,
    };

    // Shift (immediate): 000 00 (LSL) / 000 01 (LSR) / 000 10 (ASR)
    if op & 0xF800 == 0x0000 || op & 0xF800 == 0x0800 || op & 0xF800 == 0x1000 {
        let rd = (op & 0x7) as u8;
        let rm = ((op >> 3) & 0x7) as u8;
        let amount = ((op >> 6) & 0x1F) as u8;
        let kind = match op & 0x1800 {
            0x0000 => ShiftImm::Lsl,
            0x0800 => ShiftImm::Lsr,
            0x1000 => ShiftImm::Asr,
            _ => return Err(err()),
        };
        return Ok(Inst16::ShiftImm {
            rd,
            rm,
            kind,
            amount,
        });
    }

    // ADD/SUB (reg or imm3): 0001 1
    if op & 0xF800 == 0x1800 {
        let is_imm = op & 0x0400 != 0;
        let is_sub = op & 0x0200 != 0;
        let rd = (op & 0x7) as u8;
        let rn = ((op >> 3) & 0x7) as u8;
        let other = ((op >> 6) & 0x7) as u8;
        return Ok(match (is_imm, is_sub) {
            (false, false) => Inst16::AddReg { rd, rn, rm: other },
            (false, true) => Inst16::SubReg { rd, rn, rm: other },
            (true, false) => Inst16::AddImm3 { rd, rn, imm: other },
            (true, true) => Inst16::SubImm3 { rd, rn, imm: other },
        });
    }

    // MOVS/CMP/ADDS/SUBS imm8: 001 0/1 0/1
    if op & 0xF800 == 0x2000 {
        let rd = ((op >> 8) & 0x7) as u8;
        let imm = (op & 0xFF) as u8;
        return Ok(Inst16::MovImm { rd, imm });
    }
    if op & 0xF800 == 0x2800 {
        let rn = ((op >> 8) & 0x7) as u8;
        let imm = (op & 0xFF) as u8;
        return Ok(Inst16::CmpImm { rn, imm });
    }
    if op & 0xF800 == 0x3000 {
        let rd = ((op >> 8) & 0x7) as u8;
        let imm = (op & 0xFF) as u8;
        return Ok(Inst16::AddImm8 { rd, imm });
    }
    if op & 0xF800 == 0x3800 {
        let rd = ((op >> 8) & 0x7) as u8;
        let imm = (op & 0xFF) as u8;
        return Ok(Inst16::SubImm8 { rd, imm });
    }

    // ALU data-processing: 0100 00
    if op & 0xFC00 == 0x4000 {
        let rd = (op & 0x7) as u8;
        let rm = ((op >> 3) & 0x7) as u8;
        let opcode = (op >> 6) & 0xF;
        let alu = match opcode {
            0x0 => AluOp::And,
            0x1 => AluOp::Eor,
            0x2 => AluOp::Lsl,
            0x3 => AluOp::Lsr,
            0x4 => AluOp::Asr,
            0x5 => AluOp::Adc,
            0x6 => AluOp::Sbc,
            0x7 => AluOp::Ror,
            0x8 => AluOp::Tst,
            0x9 => AluOp::Neg,
            0xA => AluOp::Cmp,
            0xB => AluOp::Cmn,
            0xC => AluOp::Orr,
            0xD => AluOp::Mul,
            0xE => AluOp::Bic,
            0xF => AluOp::Mvn,
            _ => return Err(err()),
        };
        return Ok(Inst16::Alu { rd, rm, op: alu });
    }

    // Special data / branch-exchange: 0100 01
    if op & 0xFC00 == 0x4400 {
        let h = (op & 0x80) != 0;
        let rm = ((op >> 3) & 0xF) as u8;
        let rd = (op & 0x7) as u8 | if h { 0x8 } else { 0 };
        let sel = (op >> 8) & 0x3;
        return match sel {
            0b00 => Ok(Inst16::SpecialAdd { rd, rm }),
            0b01 => Ok(Inst16::SpecialCmp { rn: rd, rm }),
            0b10 => Ok(Inst16::SpecialMov { rd, rm }),
            0b11 => {
                // BX if bit7==0, BLX if bit7==1.
                if op & 0x80 != 0 {
                    Ok(Inst16::Blx { rm })
                } else {
                    Ok(Inst16::Bx { rm })
                }
            }
            _ => Err(err()),
        };
    }

    // LDR literal: 0100 1 Rt imm8
    if op & 0xF800 == 0x4800 {
        let rt = ((op >> 8) & 0x7) as u8;
        let imm = (op & 0xFF) as usize * 4;
        return Ok(Inst16::LdrLiteral { rt, imm });
    }

    // Load/store register offset: 0101 0 Lx B
    if op & 0xF000 == 0x5000 {
        let rt = (op & 0x7) as u8;
        let rn = ((op >> 3) & 0x7) as u8;
        let rm = ((op >> 6) & 0x7) as u8;
        let kind = match (op >> 9) & 0x7 {
            0b000 => LsReg::Str,
            0b001 => LsReg::Strh,
            0b010 => LsReg::Strb,
            0b011 => LsReg::Ldrsb,
            0b100 => LsReg::Ldr,
            0b101 => LsReg::Ldrh,
            0b110 => LsReg::Ldrb,
            0b111 => LsReg::Ldrsh,
            _ => return Err(err()),
        };
        return Ok(Inst16::LsReg { rt, rn, rm, kind });
    }

    // Load/store immediate word: 0110 0/1
    if op & 0xF800 == 0x6000 || op & 0xF800 == 0x6800 {
        let rt = (op & 0x7) as u8;
        let rn = ((op >> 3) & 0x7) as u8;
        let imm = ((op >> 6) & 0x1F) as usize * 4;
        let kind = if op & 0x0800 != 0 {
            LsImm::Ldr
        } else {
            LsImm::Str
        };
        return Ok(Inst16::LsImm { rt, rn, imm, kind });
    }

    // Load/store byte: 0111 0/1
    if op & 0xF800 == 0x7000 || op & 0xF800 == 0x7800 {
        let rt = (op & 0x7) as u8;
        let rn = ((op >> 3) & 0x7) as u8;
        let imm = ((op >> 6) & 0x1F) as usize;
        let kind = if op & 0x0800 != 0 {
            LsImm::Ldrb
        } else {
            LsImm::Strb
        };
        return Ok(Inst16::LsImm { rt, rn, imm, kind });
    }

    // Load/store halfword: 1000 0/1
    if op & 0xF800 == 0x8000 || op & 0xF800 == 0x8800 {
        let rt = (op & 0x7) as u8;
        let rn = ((op >> 3) & 0x7) as u8;
        let imm = ((op >> 6) & 0x1F) as usize * 2;
        let kind = if op & 0x0800 != 0 {
            LsImm::Ldrh
        } else {
            LsImm::Strh
        };
        return Ok(Inst16::LsImm { rt, rn, imm, kind });
    }

    // STR/LDR SP-relative: 1001 0/1 Rt imm8
    if op & 0xF800 == 0x9000 || op & 0xF800 == 0x9800 {
        let rt = ((op >> 8) & 0x7) as u8;
        let imm = (op & 0xFF) as usize * 4;
        let is_load = op & 0x0800 != 0;
        return Ok(Inst16::LsSp { rt, imm, is_load });
    }

    // ADD Rd, PC: 1010 0
    if op & 0xF800 == 0xA000 {
        let rd = ((op >> 8) & 0x7) as u8;
        let imm = (op & 0xFF) as usize * 4;
        return Ok(Inst16::AddrPc { rd, imm });
    }
    // ADD Rd, SP: 1010 1
    if op & 0xF800 == 0xA800 {
        let rd = ((op >> 8) & 0x7) as u8;
        let imm = (op & 0xFF) as usize * 4;
        return Ok(Inst16::AddrSp { rd, imm });
    }

    // ADD/SUB SP: 1011 0000
    if op & 0xFF00 == 0xB000 {
        let is_sub = op & 0x80 != 0;
        let imm = (op & 0x7F) as usize * 4;
        return if is_sub {
            Ok(Inst16::SubSpImm8 { imm })
        } else {
            Ok(Inst16::AddSpImm8 { imm })
        };
    }

    // PUSH: 1011 010 M rlist (M = lr). Only rlist, no high-register option.
    if op & 0xFE00 == 0xB400 {
        return Ok(Inst16::Push {
            regs: (op & 0xFF) as u8,
            include_lr: op & 0x100 != 0,
        });
    }

    // POP: 1011 110 P rlist (P = pc).
    if op & 0xFE00 == 0xBC00 {
        return Ok(Inst16::Pop {
            regs: (op & 0xFF) as u8,
            load_pc: op & 0x100 != 0,
        });
    }

    // BKPT: 1011 1110 imm8.
    if op & 0xFF00 == 0xBE00 {
        return Ok(Inst16::Bkpt {
            imm: (op & 0xFF) as u8,
        });
    }

    // HINT: 1011 1111 0000 imm4, imm4 in {0..=4} (bits[7:4]).
    if op & 0xFF0F == 0xBF00 {
        let kind = match (op >> 4) & 0xF {
            0 => Hint::Nop,
            1 => Hint::Yield,
            2 => Hint::Wfe,
            3 => Hint::Wfi,
            4 => Hint::Sev,
            _ => return Err(err()),
        };
        return Ok(Inst16::Hint { kind });
    }

    // CPSIE/CPSID: 1011 0110 0110 D im.
    if op & 0xFFE0 == 0xB660 {
        return Ok(Inst16::Cps {
            disable: op & 0x10 != 0,
            im: (op & 0x7) as u8,
        });
    }

    // EXTEND: 1011 0010 01 op Rm Rd. The two-bit `op` (bits[7:6]) picks the
    // signed/unsigned halfword/byte form; `Rm` sits in bits[6:3] and `Rd` in
    // bits[2:0], so the low six bits never overlap the discriminator.
    if op & 0xFF00 == 0xB200 {
        let kind = match op & 0xFFC0 {
            0xB200 => ExtendKind::Sxth,
            0xB240 => ExtendKind::Sxtb,
            0xB280 => ExtendKind::Uxth,
            0xB2C0 => ExtendKind::Uxtb,
            _ => return Err(err()),
        };
        return Ok(Inst16::Extend {
            rd: (op & 0x7) as u8,
            rm: ((op >> 3) & 0x7) as u8,
            kind,
        });
    }

    // REVERSE: 1011 1010 op Rm Rd. `op` (bits[7:6]) selects `REV` (00),
    // `REV16` (01), or `REVSH` (11); the `10` middle space is unallocated.
    if op & 0xFF00 == 0xBA00 {
        let kind = match op & 0xFFC0 {
            0xBA00 => ReverseKind::Rev,
            0xBA40 => ReverseKind::Rev16,
            0xBAC0 => ReverseKind::Revsh,
            _ => return Err(err()),
        };
        return Ok(Inst16::Reverse {
            rd: (op & 0x7) as u8,
            rm: ((op >> 3) & 0x7) as u8,
            kind,
        });
    }

    // STMIA/LDMIA: 1100 rn rlist (bit11 = load).
    if op & 0xF800 == 0xC000 || op & 0xF800 == 0xC800 {
        return Ok(Inst16::LdmStm {
            is_load: op & 0x0800 != 0,
            rn: ((op >> 8) & 0x7) as u8,
            regs: (op & 0xFF) as u8,
        });
    }

    // SVC: 1101 1111 imm8 (cond == 0xF in the conditional-branch space).
    if op & 0xFF00 == 0xDF00 {
        return Ok(Inst16::Svc {
            imm: (op & 0xFF) as u8,
        });
    }

    // B<cond>: 1101 cond imm8, cond != 0xF.
    if op & 0xF000 == 0xD000 && op & 0x0F00 != 0x0F00 {
        let cond = ((op >> 8) & 0xF) as u8;
        let off = i32::from(op & 0xFF) << 1;
        // Sign-extend the 9-bit offset.
        let off = (off << 23) >> 23;
        return Ok(Inst16::BranchCond { cond, off });
    }

    // CBZ/CBNZ: 1011 P 0 i 1 imm5 rn. Bit 11 selects CBZ (0) / CBNZ (1),
    // bit 9 is the i bit, bits[7:3] the imm5, and the offset is the unsigned
    // (forward-only) `i:imm5:0`.
    if op & 0xF500 == 0xB100 {
        let rn = (op & 0x7) as u8;
        let i = i32::from((op >> 9) & 1);
        let imm5 = i32::from((op >> 3) & 0x1F);
        let off = (i << 6) | (imm5 << 1);
        return Ok(Inst16::CbzCbnz {
            rn,
            off,
            nonzero: op & 0x0800 != 0,
        });
    }

    // B (unconditional): 1110 imm11.
    if op & 0xF800 == 0xE000 {
        let off = i32::from(op & 0x7FF) << 1;
        // Sign-extend the 12-bit offset.
        let off = (off << 20) >> 20;
        return Ok(Inst16::Branch { off });
    }

    Err(err())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Decode a byte sequence produced by llvm-mc (little-endian halfword).
    fn decode_bytes(bytes: [u8; 2]) -> Result<Inst16, DecodeError> {
        let op = u16::from_le_bytes(bytes);
        decode16(op)
    }

    #[test]
    fn shift_imm() {
        // lsls r3, r4, #2  -> [0xa3, 0x00]
        assert_eq!(
            decode_bytes([0xa3, 0x00]),
            Ok(Inst16::ShiftImm {
                rd: 3,
                rm: 4,
                kind: ShiftImm::Lsl,
                amount: 2
            })
        );
    }

    #[test]
    fn add_sub_imm3() {
        // subs r5, r5, #1 -> [0x6d, 0x1e]
        assert_eq!(
            decode_bytes([0x6d, 0x1e]),
            Ok(Inst16::SubImm3 {
                rd: 5,
                rn: 5,
                imm: 1
            })
        );
    }

    #[test]
    fn mov_cmp_add_sub_imm8() {
        // movs r2, #5 -> [0x05, 0x22]
        assert_eq!(
            decode_bytes([0x05, 0x22]),
            Ok(Inst16::MovImm { rd: 2, imm: 5 })
        );
        // adds r0, r0, r2 -> [0x80, 0x18]
        assert_eq!(
            decode_bytes([0x80, 0x18]),
            Ok(Inst16::AddReg {
                rd: 0,
                rn: 0,
                rm: 2
            })
        );
    }

    #[test]
    fn alu() {
        // eors r0, r1 -> [0x48, 0x40]
        assert_eq!(
            decode_bytes([0x48, 0x40]),
            Ok(Inst16::Alu {
                rd: 0,
                rm: 1,
                op: AluOp::Eor
            })
        );
        // muls r2, r3 -> [0x5a, 0x43]
        assert_eq!(
            decode_bytes([0x5a, 0x43]),
            Ok(Inst16::Alu {
                rd: 2,
                rm: 3,
                op: AluOp::Mul
            })
        );
    }

    #[test]
    fn special() {
        // mov r2, r3 -> [0x1a, 0x46]
        assert_eq!(
            decode_bytes([0x1a, 0x46]),
            Ok(Inst16::SpecialMov { rd: 2, rm: 3 })
        );
        // mov lr, r1 -> [0x8e, 0x46]
        assert_eq!(
            decode_bytes([0x8e, 0x46]),
            Ok(Inst16::SpecialMov { rd: 14, rm: 1 })
        );
        // bx lr -> [0x70, 0x47]
        assert_eq!(decode_bytes([0x70, 0x47]), Ok(Inst16::Bx { rm: 14 }));
        // blx r3 -> [0x98, 0x47]
        assert_eq!(decode_bytes([0x98, 0x47]), Ok(Inst16::Blx { rm: 3 }));
    }

    #[test]
    fn literal() {
        // ldr r0, [pc, #8] -> 0x4802 (imm=2*4=8)
        assert_eq!(
            decode_bytes([0x02, 0x48]),
            Ok(Inst16::LdrLiteral { rt: 0, imm: 8 })
        );
    }

    #[test]
    fn ls_reg() {
        // ldrsb r4, [r2, r5] -> [0x54, 0x57]
        assert_eq!(
            decode_bytes([0x54, 0x57]),
            Ok(Inst16::LsReg {
                rt: 4,
                rn: 2,
                rm: 5,
                kind: LsReg::Ldrsb
            })
        );
    }

    #[test]
    fn ls_imm() {
        // strh r2, [r1, #4] -> [0x8a, 0x80]
        assert_eq!(
            decode_bytes([0x8a, 0x80]),
            Ok(Inst16::LsImm {
                rt: 2,
                rn: 1,
                imm: 4,
                kind: LsImm::Strh
            })
        );
    }

    #[test]
    fn sp_relative() {
        // str r6, [sp, #8] -> [0x02, 0x96]
        assert_eq!(
            decode_bytes([0x02, 0x96]),
            Ok(Inst16::LsSp {
                rt: 6,
                imm: 8,
                is_load: false
            })
        );
        // ldr r7, [sp, #12] -> [0x03, 0x9f]
        assert_eq!(
            decode_bytes([0x03, 0x9f]),
            Ok(Inst16::LsSp {
                rt: 7,
                imm: 12,
                is_load: true
            })
        );
    }

    #[test]
    fn add_sp() {
        // sub sp, #8 -> [0x82, 0xb0]
        assert_eq!(decode_bytes([0x82, 0xb0]), Ok(Inst16::SubSpImm8 { imm: 8 }));
    }

    #[test]
    fn undefined_is_error() {
        // 0xBFFF has a reserved HINT imm4 and is not a valid ARMv6-M opcode.
        assert!(matches!(
            decode_bytes([0xff, 0xbf]),
            Err(DecodeError { .. })
        ));
    }

    #[test]
    fn branches() {
        // bne . -> [0xfe, 0xd1] (cond NE, imm8=0xFE => offset -4).
        assert_eq!(
            decode_bytes([0xfe, 0xd1]),
            Ok(Inst16::BranchCond { cond: 0x1, off: -4 })
        );
        // bal . -> [0xfe, 0xe7] (unconditional, imm11=0x7FE => offset -4).
        assert_eq!(decode_bytes([0xfe, 0xe7]), Ok(Inst16::Branch { off: -4 }));
        // beq +2 -> [0x01, 0xd0].
        assert_eq!(
            decode_bytes([0x01, 0xd0]),
            Ok(Inst16::BranchCond { cond: 0x0, off: 2 })
        );
    }

    #[test]
    fn cbz_cbnz() {
        // cbz r0, +2 -> [0x08, 0xb1] (imm5=1 => offset 2).
        assert_eq!(
            decode_bytes([0x08, 0xb1]),
            Ok(Inst16::CbzCbnz {
                rn: 0,
                off: 2,
                nonzero: false
            })
        );
        // cbnz r3, +8 -> [0x23, 0xb9] (P=1, i=0, imm5=4 => offset 8).
        assert_eq!(
            decode_bytes([0x23, 0xb9]),
            Ok(Inst16::CbzCbnz {
                rn: 3,
                off: 8,
                nonzero: true
            })
        );
        // cbz r0, +64 -> [0x00, 0xb3] (i=1, imm5=0 => offset 64).
        assert_eq!(
            decode_bytes([0x00, 0xb3]),
            Ok(Inst16::CbzCbnz {
                rn: 0,
                off: 64,
                nonzero: false
            })
        );
    }

    #[test]
    fn svc_bkpt() {
        assert_eq!(decode_bytes([0x01, 0xdf]), Ok(Inst16::Svc { imm: 1 }));
        assert_eq!(decode_bytes([0x34, 0xbe]), Ok(Inst16::Bkpt { imm: 0x34 }));
    }

    #[test]
    fn hints_and_cps() {
        assert_eq!(
            decode_bytes([0x00, 0xbf]),
            Ok(Inst16::Hint { kind: Hint::Nop })
        );
        assert_eq!(
            decode_bytes([0x40, 0xbf]),
            Ok(Inst16::Hint { kind: Hint::Sev })
        );
        assert_eq!(
            decode_bytes([0x62, 0xb6]),
            Ok(Inst16::Cps {
                disable: false,
                im: 0x2
            })
        );
        assert_eq!(
            decode_bytes([0x72, 0xb6]),
            Ok(Inst16::Cps {
                disable: true,
                im: 0x2
            })
        );
    }

    #[test]
    fn push_pop_ldm_stm() {
        // push {r4, r7, lr} -> [0x90, 0xb5].
        assert_eq!(
            decode_bytes([0x90, 0xb5]),
            Ok(Inst16::Push {
                regs: 0x90,
                include_lr: true
            })
        );
        // pop {r4, r7, pc} -> [0x90, 0xbd].
        assert_eq!(
            decode_bytes([0x90, 0xbd]),
            Ok(Inst16::Pop {
                regs: 0x90,
                load_pc: true
            })
        );
        // stm r0!, {r1, r2, r3} -> [0x0e, 0xc0].
        assert_eq!(
            decode_bytes([0x0e, 0xc0]),
            Ok(Inst16::LdmStm {
                is_load: false,
                rn: 0,
                regs: 0x0e
            })
        );
        // ldm r2!, {r5, r6} -> [0x60, 0xca].
        assert_eq!(
            decode_bytes([0x60, 0xca]),
            Ok(Inst16::LdmStm {
                is_load: true,
                rn: 2,
                regs: 0x60
            })
        );
    }

    #[test]
    fn extend() {
        // sxth r3, r5 -> [0x2b, 0xb2].
        assert_eq!(
            decode_bytes([0x2b, 0xb2]),
            Ok(Inst16::Extend {
                rd: 3,
                rm: 5,
                kind: ExtendKind::Sxth
            })
        );
        // sxtb r2, r4 -> [0x62, 0xb2].
        assert_eq!(
            decode_bytes([0x62, 0xb2]),
            Ok(Inst16::Extend {
                rd: 2,
                rm: 4,
                kind: ExtendKind::Sxtb
            })
        );
        // uxth r1, r6 -> [0xb1, 0xb2].
        assert_eq!(
            decode_bytes([0xb1, 0xb2]),
            Ok(Inst16::Extend {
                rd: 1,
                rm: 6,
                kind: ExtendKind::Uxth
            })
        );
        // uxtb r0, r7 -> [0xf8, 0xb2].
        assert_eq!(
            decode_bytes([0xf8, 0xb2]),
            Ok(Inst16::Extend {
                rd: 0,
                rm: 7,
                kind: ExtendKind::Uxtb
            })
        );
    }

    #[test]
    fn reverse() {
        // rev r3, r5 -> [0x2b, 0xba].
        assert_eq!(
            decode_bytes([0x2b, 0xba]),
            Ok(Inst16::Reverse {
                rd: 3,
                rm: 5,
                kind: ReverseKind::Rev
            })
        );
        // rev16 r2, r4 -> [0x62, 0xba].
        assert_eq!(
            decode_bytes([0x62, 0xba]),
            Ok(Inst16::Reverse {
                rd: 2,
                rm: 4,
                kind: ReverseKind::Rev16
            })
        );
        // revsh r1, r6 -> [0xf1, 0xba].
        assert_eq!(
            decode_bytes([0xf1, 0xba]),
            Ok(Inst16::Reverse {
                rd: 1,
                rm: 6,
                kind: ReverseKind::Revsh
            })
        );
        // The `1011 1010 10` middle space is unallocated.
        assert!(decode_bytes([0x80, 0xba]).is_err());
    }
}
