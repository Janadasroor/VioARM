//! Decoding of the 32-bit ARMv6-M (Thumb-2) instructions.
//!
//! ARMv6-M is a 16-bit-only ISA apart from a tiny Thumb-2 subset: `bl`
//! (immediate), `mrs`/`msr`, and `dmb`/`dsb`/`isb`. There is no `blx`
//! immediate and no other Thumb-2 instruction on Cortex-M0. Every encoding is
//! verified against `llvm-mc -triple=thumbv6m-none-eabi`.

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
}

/// True when `hw` is the first halfword of a 32-bit ARMv6-M instruction.
/// The first halfword always starts `11110` (0xF000-0xF7FF), which no 16-bit
/// encoding uses.
#[inline]
#[must_use]
pub const fn is_thumb2(hw: u16) -> bool {
    hw & 0xF800 == 0xF000
}

/// Decode the two halfwords of a 32-bit ARMv6-M instruction.
pub fn decode32(hi: u16, lo: u16) -> Result<Inst32, DecodeError> {
    let err = || DecodeError {
        opcode: hi,
        cause: HaltCause::UndefinedInstruction,
    };

    // MRS: 1111 0011 1110 1111 1000 Rd sys.
    if hi == 0xF3EF {
        if lo & 0xF000 != 0x8000 {
            return Err(err());
        }
        let rd = ((lo >> 8) & 0xF) as u8;
        return Ok(Inst32::Mrs {
            rd,
            sys: SysReg::from((lo & 0xFF) as u8),
        });
    }

    // MSR: 1111 0011 1000 Rn 1000 sys.
    if hi & 0xFFF0 == 0xF380 {
        if lo & 0xF000 != 0x8000 {
            return Err(err());
        }
        let rn = (hi & 0xF) as u8;
        return Ok(Inst32::Msr {
            rn,
            sys: SysReg::from((lo & 0xFF) as u8),
        });
    }

    // DMB/DSB/ISB: 1111 0011 1011 1111 1000 1111 op option.
    if hi == 0xF3BF {
        if lo & 0xF000 != 0x8000 {
            return Err(err());
        }
        let kind = match (lo >> 4) & 0xF {
            0x4 => Barrier::Dsb,
            0x5 => Barrier::Dmb,
            0x6 => Barrier::Isb,
            _ => return Err(err()),
        };
        return Ok(Inst32::Barrier { kind });
    }

    // BL: 1111 0 S imm10 / 1111 1 J1 J2 imm11.
    if hi & 0xF800 == 0xF000 {
        if lo & 0xF800 != 0xF800 {
            return Err(err());
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
        return Ok(Inst32::Bl { off });
    }

    Err(err())
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
