//! Text disassembly over the shared decoders (REPL `disas`).
//!
//! The decoders (`cpu::decode16`/`decode32`) are the single source of
//! truth: this module only renders their output as conventional
//! lower-case UAL text (`movs r0, #0x5a`, `b 0x0000001c`). Width detection
//! and ISA gating match the stepper (`is_thumb2` on ARMv6-M,
//! `is_thumb2_v7m` elsewhere, per-ISA `decode32_*`), so anything the
//! machine retires disassembles at the same width. Undecodable halfwords
//! render as `.hword 0x....` — never an error, never a guess.
//!
//! Branch targets print absolute (`pc + 4 + off`, the architectural base
//! for T1/T3/T4 branches and literal loads); every other immediate prints
//! as decoded (all decoder immediates are pre-scaled to bytes).

use crate::cpu::decode16::{decode16, decode16_v7m};
use crate::cpu::decode16::{AluOp, ExtendKind, Hint, Inst16, LsImm, LsReg, ReverseKind, ShiftImm};
use crate::cpu::decode32::{
    decode32_v7em, decode32_v7m, decode32_v8m, decode32_v8m_mainline, is_thumb2, is_thumb2_v7m,
    thumb_expand_imm, BitfieldOp, DpOp, DspMulOp, DualMulOp, ExSize, Inst32, MemOp, MulOp, ParOp,
    ParSat, QaddOp, SatOp, ShiftType, SysReg, TtVariant, UnaryOp, VrintOp,
};
use crate::cpu::Isa;

/// Condition-code suffixes (`0xE` prints `al`; `0xF` is not a branch cond).
#[must_use]
pub const fn cond_name(cond: u8) -> &'static str {
    match cond {
        0x0 => "eq",
        0x1 => "ne",
        0x2 => "cs",
        0x3 => "cc",
        0x4 => "mi",
        0x5 => "pl",
        0x6 => "vs",
        0x7 => "vc",
        0x8 => "hi",
        0x9 => "ls",
        0xA => "ge",
        0xB => "lt",
        0xC => "gt",
        0xD => "le",
        0xE => "al",
        _ => "nv",
    }
}

/// Architectural register names (`13/14/15` are `sp`/`lr`/`pc`).
#[must_use]
pub fn reg_name(reg: u8) -> String {
    match reg {
        13 => "sp".to_owned(),
        14 => "lr".to_owned(),
        15 => "pc".to_owned(),
        _ => format!("r{reg}"),
    }
}

/// `{r0-r3, lr}`-style list over an 8-bit low-register mask plus one
/// optional high register (`lr` for push, `pc` for pop).
fn reg_list8(regs: u8, high: Option<&str>) -> String {
    let mut parts: Vec<String> = Vec::new();
    let mut run: Option<(u8, u8)> = None;
    let flush = |run: &mut Option<(u8, u8)>, parts: &mut Vec<String>| {
        if let Some((lo, hi)) = run.take() {
            if lo == hi {
                parts.push(format!("r{lo}"));
            } else {
                parts.push(format!("r{lo}-r{hi}"));
            }
        }
    };
    for bit in 0..8u8 {
        if regs & (1 << bit) != 0 {
            match &mut run {
                Some((_, hi)) if *hi + 1 == bit => *hi = bit,
                _ => {
                    flush(&mut run, &mut parts);
                    run = Some((bit, bit));
                }
            }
        } else {
            flush(&mut run, &mut parts);
        }
    }
    flush(&mut run, &mut parts);
    if let Some(name) = high {
        parts.push(name.to_owned());
    }
    format!("{{{}}}", parts.join(", "))
}

/// `{r0-r3, lr}`-style list over a 16-bit mask (`sp`/`lr`/`pc` named).
fn reg_list16(regs: u16) -> String {
    let mut parts: Vec<String> = Vec::new();
    let mut bit = 0u16;
    while bit < 16 {
        if regs & (1 << bit) != 0 {
            let mut hi = bit;
            while hi + 1 < 16 && regs & (1 << (hi + 1)) != 0 {
                hi += 1;
            }
            if hi == bit {
                parts.push(reg_name(bit as u8));
            } else if bit < 13 && hi < 13 {
                parts.push(format!("r{bit}-r{hi}"));
            } else {
                for b in bit..=hi {
                    parts.push(reg_name(b as u8));
                }
            }
            bit = hi + 1;
        } else {
            bit += 1;
        }
    }
    format!("{{{}}}", parts.join(", "))
}

/// Absolute branch target (`pc + 4 + off`, wrapping).
fn target(pc: u32, off: i32) -> u32 {
    pc.wrapping_add(4).wrapping_add(off as u32)
}

/// `IT` suffix letters mirroring the executor's shift machine
/// (`Core::it_advance` over `ITSTATE = (cond << 4) | mask`): the first
/// letter is always `t`; each following letter is `t` when the shifted
/// condition bit matches `cond<0>`, else `e`. Masks are cond-dependent
/// (e.g. 0100 is `itt eq` but `ite ne`); spot-checked against the
/// assembler (`itte` = 0110, `itet` = 1010, `itttt` = 0001, and the
/// cond-dependent 0100/1100 pairs).
fn it_letters(cond: u8, mask: u8) -> String {
    let mut letters = String::from("it");
    let mut state: u8 = (cond << 4) | (mask & 0xF);
    loop {
        if state.trailing_zeros() >= 3 {
            break;
        }
        state = (state & 0xE0) | (((state & 0x1F) << 1) & 0x1F);
        letters.push(if (state >> 4) & 1 == cond & 1 {
            't'
        } else {
            'e'
        });
    }
    letters
}

fn shift_suffix(shift: ShiftType, amount: u8) -> String {
    match (shift, amount) {
        (ShiftType::Lsl, 0) => String::new(),
        (ShiftType::Lsl, n) => format!(", lsl #{n}"),
        (ShiftType::Lsr, 0) => ", lsr #32".to_owned(),
        (ShiftType::Lsr, n) => format!(", lsr #{n}"),
        (ShiftType::Asr, 0) => ", asr #32".to_owned(),
        (ShiftType::Asr, n) => format!(", asr #{n}"),
        (ShiftType::Ror, 0) => ", rrx".to_owned(),
        (ShiftType::Ror, n) => format!(", ror #{n}"),
    }
}

fn sysreg_name(sys: SysReg) -> String {
    match sys {
        SysReg::Apsr => "APSR".to_owned(),
        SysReg::ApsrG => "APSR_g".to_owned(),
        SysReg::ApsrNzvCqg => "APSR_nzcvq".to_owned(),
        SysReg::Xpsr => "xPSR".to_owned(),
        SysReg::Ipsr => "IPSR".to_owned(),
        SysReg::Epsr => "EPSR".to_owned(),
        SysReg::Msp => "MSP".to_owned(),
        SysReg::Psp => "PSP".to_owned(),
        SysReg::MspLim => "MSPLIM".to_owned(),
        SysReg::PspLim => "PSPLIM".to_owned(),
        SysReg::Primask => "PRIMASK".to_owned(),
        SysReg::Basepri => "BASEPRI".to_owned(),
        SysReg::BasepriMax => "BASEPRI_MAX".to_owned(),
        SysReg::Faultmask => "FAULTMASK".to_owned(),
        SysReg::Control => "CONTROL".to_owned(),
        SysReg::MspNs => "MSP_NS".to_owned(),
        SysReg::PspNs => "PSP_NS".to_owned(),
        SysReg::MsplimNs => "MSPLIM_NS".to_owned(),
        SysReg::PsplimNs => "PSPLIM_NS".to_owned(),
        SysReg::ControlNs => "CONTROL_NS".to_owned(),
        SysReg::SpNs => "SP_NS".to_owned(),
        SysReg::Other(enc) => format!("SYS_{enc:#x}"),
    }
}

/// Render one decoded 16-bit instruction (`pc` anchors branch targets).
#[must_use]
pub fn fmt16(inst: &Inst16, pc: u32) -> String {
    match *inst {
        Inst16::ShiftImm {
            rd,
            rm,
            kind,
            amount,
        } => {
            // `lsls rd, rm, #0` is the `movs` alias.
            if kind == ShiftImm::Lsl && amount == 0 {
                return format!("movs {}, {}", reg_name(rd), reg_name(rm));
            }
            let op = match kind {
                ShiftImm::Lsl => "lsls",
                ShiftImm::Lsr => "lsrs",
                ShiftImm::Asr => "asrs",
            };
            format!("{op} {}, {}, #{amount}", reg_name(rd), reg_name(rm))
        }
        Inst16::ShiftReg { rd, rn, rm, kind } => {
            let op = match kind {
                crate::cpu::decode16::Shift::Lsl => "lsls",
                crate::cpu::decode16::Shift::Lsr => "lsrs",
                crate::cpu::decode16::Shift::Asr => "asrs",
                crate::cpu::decode16::Shift::Ror => "rors",
            };
            format!("{op} {}, {}, {}", reg_name(rd), reg_name(rn), reg_name(rm))
        }
        Inst16::AddReg { rd, rn, rm } => {
            format!("adds {}, {}, {}", reg_name(rd), reg_name(rn), reg_name(rm))
        }
        Inst16::SubReg { rd, rn, rm } => {
            format!("subs {}, {}, {}", reg_name(rd), reg_name(rn), reg_name(rm))
        }
        Inst16::AddImm3 { rd, rn, imm } => {
            format!("adds {}, {}, #{imm}", reg_name(rd), reg_name(rn))
        }
        Inst16::SubImm3 { rd, rn, imm } => {
            format!("subs {}, {}, #{imm}", reg_name(rd), reg_name(rn))
        }
        Inst16::MovImm { rd, imm } => format!("movs {}, #{imm}", reg_name(rd)),
        Inst16::CmpImm { rn, imm } => format!("cmp {}, #{imm}", reg_name(rn)),
        Inst16::AddImm8 { rd, imm } => format!("adds {}, #{imm}", reg_name(rd)),
        Inst16::SubImm8 { rd, imm } => format!("subs {}, #{imm}", reg_name(rd)),
        Inst16::Alu { rd, rm, op } => {
            let (rd_s, rm_s) = (reg_name(rd), reg_name(rm));
            match op {
                AluOp::And => format!("ands {rd_s}, {rm_s}"),
                AluOp::Eor => format!("eors {rd_s}, {rm_s}"),
                AluOp::Lsl => format!("lsls {rd_s}, {rm_s}"),
                AluOp::Lsr => format!("lsrs {rd_s}, {rm_s}"),
                AluOp::Asr => format!("asrs {rd_s}, {rm_s}"),
                AluOp::Adc => format!("adcs {rd_s}, {rm_s}"),
                AluOp::Sbc => format!("sbcs {rd_s}, {rm_s}"),
                AluOp::Ror => format!("rors {rd_s}, {rm_s}"),
                AluOp::Tst => format!("tst {rd_s}, {rm_s}"),
                // `neg rd, rm` is `rsbs rd, rm, #0`.
                AluOp::Neg => format!("rsbs {rd_s}, {rm_s}, #0"),
                AluOp::Cmp => format!("cmp {rd_s}, {rm_s}"),
                AluOp::Cmn => format!("cmn {rd_s}, {rm_s}"),
                AluOp::Orr => format!("orrs {rd_s}, {rm_s}"),
                AluOp::Mul => format!("muls {rd_s}, {rm_s}"),
                AluOp::Bic => format!("bics {rd_s}, {rm_s}"),
                AluOp::Mvn => format!("mvns {rd_s}, {rm_s}"),
            }
        }
        Inst16::AddSpImm8 { imm } => format!("add sp, #{imm}"),
        Inst16::SubSpImm8 { imm } => format!("sub sp, #{imm}"),
        Inst16::SpecialAdd { rd, rm } => {
            format!("add {}, {}", reg_name(rd), reg_name(rm))
        }
        Inst16::SpecialCmp { rn, rm } => {
            format!("cmp {}, {}", reg_name(rn), reg_name(rm))
        }
        Inst16::SpecialMov { rd, rm } => {
            format!("mov {}, {}", reg_name(rd), reg_name(rm))
        }
        Inst16::Bx { rm } => format!("bx {}", reg_name(rm)),
        Inst16::Blx { rm } => format!("blx {}", reg_name(rm)),
        Inst16::BxNs { rm } => format!("bxns {}", reg_name(rm)),
        Inst16::BlxNs { rm } => format!("blxns {}", reg_name(rm)),
        Inst16::LdrLiteral { rt, imm } => {
            let base = (pc.wrapping_add(4)) & !3;
            let addr = base.wrapping_add(imm as u32);
            format!("ldr {}, [pc, #{imm}] ; {addr:#010x}", reg_name(rt))
        }
        Inst16::LsReg { rt, rn, rm, kind } => {
            let op = match kind {
                LsReg::Str => "str",
                LsReg::Strh => "strh",
                LsReg::Strb => "strb",
                LsReg::Ldrsb => "ldrsb",
                LsReg::Ldr => "ldr",
                LsReg::Ldrh => "ldrh",
                LsReg::Ldrb => "ldrb",
                LsReg::Ldrsh => "ldrsh",
            };
            format!(
                "{op} {}, [{}, {}]",
                reg_name(rt),
                reg_name(rn),
                reg_name(rm)
            )
        }
        Inst16::LsImm { rt, rn, imm, kind } => {
            let op = match kind {
                LsImm::Str => "str",
                LsImm::Ldr => "ldr",
                LsImm::Strb => "strb",
                LsImm::Ldrb => "ldrb",
                LsImm::Strh => "strh",
                LsImm::Ldrh => "ldrh",
            };
            format!("{op} {}, [{}, #{imm}]", reg_name(rt), reg_name(rn))
        }
        Inst16::LsSp { rt, imm, is_load } => {
            let op = if is_load { "ldr" } else { "str" };
            format!("{op} {}, [sp, #{imm}]", reg_name(rt))
        }
        Inst16::AddrPc { rd, imm } => {
            let base = (pc.wrapping_add(4)) & !3;
            let addr = base.wrapping_add(imm as u32);
            format!("adr {}, [pc, #{imm}] ; {addr:#010x}", reg_name(rd))
        }
        Inst16::AddrSp { rd, imm } => format!("add {}, sp, #{imm}", reg_name(rd)),
        Inst16::BranchCond { cond, off } => {
            format!("b{} {:#010x}", cond_name(cond), target(pc, off))
        }
        Inst16::Branch { off } => format!("b {:#010x}", target(pc, off)),
        Inst16::CbzCbnz { rn, off, nonzero } => {
            let op = if nonzero { "cbnz" } else { "cbz" };
            format!("{op} {}, {:#010x}", reg_name(rn), target(pc, off))
        }
        Inst16::Svc { imm } => format!("svc #{imm}"),
        Inst16::Bkpt { imm } => format!("bkpt #{imm}"),
        Inst16::Hint { kind } => match kind {
            Hint::Nop => "nop".to_owned(),
            Hint::Yield => "yield".to_owned(),
            Hint::Wfe => "wfe".to_owned(),
            Hint::Wfi => "wfi".to_owned(),
            Hint::Sev => "sev".to_owned(),
        },
        Inst16::Cps { disable, im } => {
            // `im` is the `a:i:f` flag set (bit1 = `i`, bit0 = `f`).
            let gate = match im & 0x3 {
                0x2 => "i",
                0x1 => "f",
                0x3 => "if",
                _ => "i",
            };
            if disable {
                format!("cpsid {gate}")
            } else {
                format!("cpsie {gate}")
            }
        }
        Inst16::Extend { rd, rm, kind } => {
            let op = match kind {
                ExtendKind::Sxth => "sxth",
                ExtendKind::Sxtb => "sxtb",
                ExtendKind::Uxth => "uxth",
                ExtendKind::Uxtb => "uxtb",
            };
            format!("{op} {}, {}", reg_name(rd), reg_name(rm))
        }
        Inst16::Reverse { rd, rm, kind } => {
            let op = match kind {
                ReverseKind::Rev => "rev",
                ReverseKind::Rev16 => "rev16",
                ReverseKind::Revsh => "revsh",
            };
            format!("{op} {}, {}", reg_name(rd), reg_name(rm))
        }
        Inst16::Push { regs, include_lr } => {
            format!("push {}", reg_list8(regs, include_lr.then_some("lr")))
        }
        Inst16::Pop { regs, load_pc } => {
            format!("pop {}", reg_list8(regs, load_pc.then_some("pc")))
        }
        Inst16::LdmStm { is_load, rn, regs } => {
            let op = if is_load { "ldmia" } else { "stmia" };
            format!("{op} {}!, {}", reg_name(rn), reg_list8(regs, None))
        }
        Inst16::It { cond, mask } => {
            format!("{} {}", it_letters(cond, mask), cond_name(cond))
        }
    }
}

fn dp_name(op: DpOp) -> &'static str {
    match op {
        DpOp::And => "and",
        DpOp::Eor => "eor",
        DpOp::Bic => "bic",
        DpOp::Orr => "orr",
        DpOp::Orn => "orn",
        DpOp::Mov => "mov",
        DpOp::Mvn => "mvn",
        DpOp::Add => "add",
        DpOp::Adc => "adc",
        DpOp::Sbc => "sbc",
        DpOp::Sub => "sub",
        DpOp::Rsb => "rsb",
        DpOp::Tst => "tst",
        DpOp::Teq => "teq",
        DpOp::Cmp => "cmp",
        DpOp::Cmn => "cmn",
    }
}

/// `true` for the flag-reading data-processing ops (`tst`/`teq`/`cmp`/`cmn`:
/// no destination, never an `s` suffix).
const fn dp_is_test(op: DpOp) -> bool {
    matches!(op, DpOp::Tst | DpOp::Teq | DpOp::Cmp | DpOp::Cmn)
}

fn mem_name(access: MemOp, unprivileged: bool) -> String {
    let base = match access {
        MemOp::Str => "str",
        MemOp::Ldr => "ldr",
        MemOp::Strb => "strb",
        MemOp::Ldrb => "ldrb",
        MemOp::Strh => "strh",
        MemOp::Ldrh => "ldrh",
        MemOp::Ldrsb => "ldrsb",
        MemOp::Ldrsh => "ldrsh",
    };
    if unprivileged {
        // `ldrt`/`strt`: no `.w` (matches the assembler spelling).
        format!("{base}t")
    } else {
        format!("{base}.w")
    }
}

/// `[rn, #off]`-style address shared by the `Mem`/`LdrdStrd` forms.
fn mem_addr(
    rt: u8,
    rn: u8,
    off: u32,
    rm: Option<u8>,
    shift: u8,
    add: bool,
    index: bool,
    writeback: bool,
) -> String {
    let rt_s = reg_name(rt);
    let rn_s = reg_name(rn);
    if let Some(rm) = rm {
        let mut addr = format!("{rt_s}, [{rn_s}, {}", reg_name(rm));
        if shift != 0 {
            addr.push_str(&format!(", lsl #{shift}"));
        }
        addr.push(']');
        return addr;
    }
    if off == 0 && add && index && !writeback {
        return format!("{rt_s}, [{rn_s}]");
    }
    let off_s = if add {
        format!("#{off}")
    } else {
        format!("#-{off}")
    };
    if index && writeback {
        format!("{rt_s}, [{rn_s}, {off_s}]!")
    } else if !index && writeback {
        format!("{rt_s}, [{rn_s}], {off_s}")
    } else {
        format!("{rt_s}, [{rn_s}, {off_s}]")
    }
}

/// Render one decoded 32-bit instruction (`pc` anchors branch targets).
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn fmt32(inst: &Inst32, pc: u32) -> String {
    match *inst {
        Inst32::Bl { off } => format!("bl {:#010x}", target(pc, off)),
        Inst32::BranchWide { off } => format!("b.w {:#010x}", target(pc, off)),
        Inst32::BranchCondWide { cond, off } => {
            format!("b{}.w {:#010x}", cond_name(cond), target(pc, off))
        }
        Inst32::BlxImm { off } => format!("blx {:#010x}", target(pc, off)),
        Inst32::Mrs { rd, sys } => format!("mrs {}, {}", reg_name(rd), sysreg_name(sys)),
        Inst32::Msr { rn, sys } => format!("msr {}, {}", sysreg_name(sys), reg_name(rn)),
        Inst32::Barrier { kind } => format!("{kind:?}").to_lowercase(),
        Inst32::Ldrex { rt, rn, size, off } => {
            let op = match size {
                ExSize::Byte => "ldrexb",
                ExSize::Half => "ldrexh",
                ExSize::Word => "ldrex",
            };
            let addr = if off == 0 {
                format!("[{}]", reg_name(rn))
            } else {
                format!("[{}, #{off}]", reg_name(rn))
            };
            format!("{op} {}, {addr}", reg_name(rt))
        }
        Inst32::Strex {
            rd,
            rt,
            rn,
            size,
            off,
        } => {
            let op = match size {
                ExSize::Byte => "strexb",
                ExSize::Half => "strexh",
                ExSize::Word => "strex",
            };
            let addr = if off == 0 {
                format!("[{}]", reg_name(rn))
            } else {
                format!("[{}, #{off}]", reg_name(rn))
            };
            format!("{op} {}, {}, {addr}", reg_name(rd), reg_name(rt))
        }
        Inst32::Clrex => "clrex".to_owned(),
        Inst32::Lda {
            rt,
            rn,
            size,
            exclusive,
        } => {
            let op = match (size, exclusive) {
                (ExSize::Byte, false) => "ldab",
                (ExSize::Half, false) => "ldah",
                (ExSize::Word, false) => "lda",
                (ExSize::Byte, true) => "ldaexb",
                (ExSize::Half, true) => "ldaexh",
                (ExSize::Word, true) => "ldaex",
            };
            format!("{op} {}, [{}]", reg_name(rt), reg_name(rn))
        }
        Inst32::Stl {
            rd,
            rt,
            rn,
            size,
            exclusive,
        } => {
            let op = match (size, exclusive) {
                (ExSize::Byte, false) => "stlb",
                (ExSize::Half, false) => "stlh",
                (ExSize::Word, false) => "stl",
                (ExSize::Byte, true) => "stlexb",
                (ExSize::Half, true) => "stlexh",
                (ExSize::Word, true) => "stlex",
            };
            if exclusive {
                format!(
                    "{op} {}, {}, [{}]",
                    reg_name(rd),
                    reg_name(rt),
                    reg_name(rn)
                )
            } else {
                format!("{op} {}, [{}]", reg_name(rt), reg_name(rn))
            }
        }
        Inst32::DpReg {
            op,
            rd,
            rn,
            rm,
            shift,
            amount,
            set_flags,
        } => {
            // Wide forms carry `.w` (a narrow twin exists for every one of
            // these mnemonics); flag-setting adds `s` (tests never take it).
            let s = if set_flags && !dp_is_test(op) {
                "s"
            } else {
                ""
            };
            let op2 = format!("{}{}", reg_name(rm), shift_suffix(shift, amount));
            if dp_is_test(op) {
                format!("{}.w {}, {op2}", dp_name(op), reg_name(rn))
            } else if matches!(op, DpOp::Mov | DpOp::Mvn)
                && rn == 15
                && shift_suffix(shift, amount).is_empty()
            {
                format!("{}.w{s} {}, {}", dp_name(op), reg_name(rd), reg_name(rm))
            } else {
                format!(
                    "{}.w{s} {}, {}, {op2}",
                    dp_name(op),
                    reg_name(rd),
                    reg_name(rn)
                )
            }
        }
        Inst32::DpImm {
            op,
            rd,
            rn,
            imm12,
            expand,
            set_flags,
        } => {
            let value = if expand {
                thumb_expand_imm(imm12)
            } else {
                u32::from(imm12)
            };
            let s = if set_flags && !dp_is_test(op) {
                "s"
            } else {
                ""
            };
            if dp_is_test(op) {
                format!("{}.w {}, #{value}", dp_name(op), reg_name(rn))
            } else if matches!(op, DpOp::Mov | DpOp::Mvn) {
                format!("{}.w{s} {}, #{value}", dp_name(op), reg_name(rd))
            } else {
                format!(
                    "{}.w{s} {}, {}, #{value}",
                    dp_name(op),
                    reg_name(rd),
                    reg_name(rn)
                )
            }
        }
        Inst32::MovWide { rd, imm, high } => {
            let op = if high { "movt" } else { "movw" };
            format!("{op} {}, #{imm}", reg_name(rd))
        }
        Inst32::Mem {
            access,
            rt,
            rn,
            off,
            rm,
            shift,
            add,
            index,
            writeback,
            unprivileged,
        } => {
            // PC-relative literals read `[Align(pc + 4, 4) + off]`.
            if rn == 15 && rm.is_none() && !matches!(access, MemOp::Str | MemOp::Strb | MemOp::Strh)
            {
                let base = (pc.wrapping_add(4)) & !3;
                let addr = base.wrapping_add(off);
                return format!(
                    "{} {}, [pc, #{off}] ; {addr:#010x}",
                    mem_name(access, unprivileged),
                    reg_name(rt)
                );
            }
            format!(
                "{} {}",
                mem_name(access, unprivileged),
                mem_addr(rt, rn, off, rm, shift, add, index, writeback)
            )
        }
        Inst32::LdrdStrd {
            is_load,
            rt,
            rt2,
            rn,
            imm,
            add,
            index,
            writeback,
        } => {
            let op = if is_load { "ldrd" } else { "strd" };
            let pair = format!("{}, {}", reg_name(rt), reg_name(rt2));
            if rn == 15 && is_load {
                let base = (pc.wrapping_add(4)) & !3;
                let addr = base.wrapping_add(imm);
                return format!("{op} {pair}, [pc, #{imm}] ; {addr:#010x}");
            }
            let off_s = if add {
                format!("#{imm}")
            } else {
                format!("#-{imm}")
            };
            if index && writeback {
                format!("{op} {pair}, [{}, {off_s}]!", reg_name(rn))
            } else if !index && writeback {
                format!("{op} {pair}, [{}], {off_s}", reg_name(rn))
            } else {
                format!("{op} {pair}, [{}, {off_s}]", reg_name(rn))
            }
        }
        Inst32::LdmStm {
            is_load,
            rn,
            regs,
            writeback,
            dec_before,
        } => {
            let op = match (is_load, dec_before) {
                (true, false) => "ldmia",
                (false, false) => "stmia",
                (true, true) => "ldmdb",
                (false, true) => "stmdb",
            };
            let wb = if writeback { "!" } else { "" };
            format!("{op} {}{wb}, {}", reg_name(rn), reg_list16(regs))
        }
        Inst32::TblBranch { rn, rm, half } => {
            let op = if half { "tbh" } else { "tbb" };
            format!("{op} [{}, {}]", reg_name(rn), reg_name(rm))
        }
        Inst32::Div { signed, rd, rn, rm } => {
            let op = if signed { "sdiv" } else { "udiv" };
            format!("{op} {}, {}, {}", reg_name(rd), reg_name(rn), reg_name(rm))
        }
        Inst32::Mul { op, rd, rn, rm, ra } => {
            let (rd_s, rn_s, rm_s) = (reg_name(rd), reg_name(rn), reg_name(rm));
            match op {
                MulOp::Mul => format!("mul {rd_s}, {rn_s}, {rm_s}"),
                MulOp::Mla => format!("mla {rd_s}, {rn_s}, {rm_s}, {}", reg_name(ra)),
                MulOp::Mls => format!("mls {rd_s}, {rn_s}, {rm_s}, {}", reg_name(ra)),
                // Long forms: `rd` is RdHi, `ra` is RdLo.
                MulOp::Smull => format!("smull {}, {rd_s}, {rn_s}, {rm_s}", reg_name(ra)),
                MulOp::Umull => format!("umull {}, {rd_s}, {rn_s}, {rm_s}", reg_name(ra)),
                MulOp::Smlal => format!("smlal {}, {rd_s}, {rn_s}, {rm_s}", reg_name(ra)),
                MulOp::Umlal => format!("umlal {}, {rd_s}, {rn_s}, {rm_s}", reg_name(ra)),
            }
        }
        Inst32::DspMul { op, rd, rn, rm, ra } => {
            let name = format!("{op:?}").to_lowercase();
            let (rd_s, rn_s, rm_s) = (reg_name(rd), reg_name(rn), reg_name(rm));
            if matches!(op, DspMulOp::Usada8) {
                format!("{name} {rd_s}, {rn_s}, {rm_s}, {}", reg_name(ra))
            } else {
                format!("{name} {rd_s}, {rn_s}, {rm_s}")
            }
        }
        Inst32::DualMul { op, rd, rn, rm, ra } => {
            let name = format!("{op:?}").to_lowercase();
            let (rd_s, rn_s, rm_s, ra_s) = (reg_name(rd), reg_name(rn), reg_name(rm), reg_name(ra));
            // Only the rounding `SMMUL` family takes no accumulator; every
            // other dual form reads `ra` (accumulator or RdLo).
            if matches!(
                op,
                DualMulOp::Smmla | DualMulOp::Smmlar | DualMulOp::Smmls | DualMulOp::Smmlsr
            ) {
                format!("{name} {rd_s}, {rn_s}, {rm_s}")
            } else {
                format!("{name} {rd_s}, {rn_s}, {rm_s}, {ra_s}")
            }
        }
        Inst32::Unary { op, rd, rm } => {
            let name = match op {
                UnaryOp::Rev => "rev",
                UnaryOp::Rev16 => "rev16",
                UnaryOp::Revsh => "revsh",
                UnaryOp::Rbit => "rbit",
                UnaryOp::Clz => "clz",
            };
            format!("{name} {}, {}", reg_name(rd), reg_name(rm))
        }
        Inst32::Qadd { op, rd, rn, rm } => {
            let name = match op {
                QaddOp::Qadd => "qadd",
                QaddOp::Qdadd => "qdadd",
                QaddOp::Qsub => "qsub",
                QaddOp::Qdsub => "qdsub",
            };
            // ARM syntax is `Rd, Rm, Rn` (the doubling applies to `Rn`).
            format!(
                "{name} {}, {}, {}",
                reg_name(rd),
                reg_name(rm),
                reg_name(rn)
            )
        }
        Inst32::Sat {
            op,
            rd,
            rn,
            sat_n,
            sh,
            shift,
        } => {
            let name = match op {
                SatOp::Ssat => "ssat",
                SatOp::Usat => "usat",
                SatOp::Ssat16 => "ssat16",
                SatOp::Usat16 => "usat16",
            };
            if matches!(op, SatOp::Ssat16 | SatOp::Usat16) || shift == 0 {
                format!("{name} {}, #{sat_n}, {}", reg_name(rd), reg_name(rn))
            } else if sh {
                format!(
                    "{name} {}, #{sat_n}, {}, asr #{shift}",
                    reg_name(rd),
                    reg_name(rn)
                )
            } else {
                format!(
                    "{name} {}, #{sat_n}, {}, lsl #{shift}",
                    reg_name(rd),
                    reg_name(rn)
                )
            }
        }
        Inst32::Bitfield {
            op,
            rd,
            rn,
            lsb,
            field,
        } => match op {
            BitfieldOp::Sbfx => format!(
                "sbfx {}, {}, #{lsb}, #{}",
                reg_name(rd),
                reg_name(rn),
                field + 1
            ),
            BitfieldOp::Ubfx => format!(
                "ubfx {}, {}, #{lsb}, #{}",
                reg_name(rd),
                reg_name(rn),
                field + 1
            ),
            // Inserts: `field` is the msb, width is `msb - lsb + 1`.
            BitfieldOp::Bfi => format!(
                "bfi {}, {}, #{lsb}, #{}",
                reg_name(rd),
                reg_name(rn),
                field + 1 - lsb
            ),
            BitfieldOp::Bfc => format!("bfc {}, #{lsb}, #{}", reg_name(rd), field + 1 - lsb),
        },
        Inst32::Par {
            op,
            sat,
            rd,
            rn,
            rm,
        } => {
            let prefix = match sat {
                ParSat::S => "s",
                ParSat::Q => "q",
                ParSat::Sh => "sh",
                ParSat::U => "u",
                ParSat::Uq => "uq",
                ParSat::Uh => "uh",
            };
            let base = match op {
                ParOp::Add8 => "add8",
                ParOp::Add16 => "add16",
                ParOp::Sub8 => "sub8",
                ParOp::Sub16 => "sub16",
                ParOp::Asx => "asx",
                ParOp::Sax => "sax",
            };
            format!(
                "{prefix}{base} {}, {}, {}",
                reg_name(rd),
                reg_name(rn),
                reg_name(rm)
            )
        }
        Inst32::Sel { rd, rn, rm } => {
            format!("sel {}, {}, {}", reg_name(rd), reg_name(rn), reg_name(rm))
        }
        Inst32::ExtendAdd {
            op,
            rd,
            rn,
            rm,
            rot,
        } => {
            let name = format!("{op:?}").to_lowercase();
            let rot_s = if rot == 0 {
                String::new()
            } else {
                format!(", ror #{}", rot * 8)
            };
            // `rn == 15` selects the two-operand `SXTH`-style fold.
            if rn == 15 && rot == 0 {
                let short = name.trim_end_matches("a").trim_end_matches("16");
                format!("{short} {}, {}{rot_s}", reg_name(rd), reg_name(rm))
            } else {
                format!(
                    "{name} {}, {}, {}{rot_s}",
                    reg_name(rd),
                    reg_name(rn),
                    reg_name(rm)
                )
            }
        }
        Inst32::Vfp3 {
            op,
            vd,
            vn,
            vm,
            is_dp,
        } => {
            let name = format!("{op:?}").to_lowercase();
            let (w, d, n, m) = fp3_regs(vd, vn, vm, is_dp);
            format!("{name}.{w} {d}, {n}, {m}")
        }
        Inst32::Vfp2 { op, vd, vm, is_dp } => {
            let name = format!("{op:?}").to_lowercase();
            let (w, d, m) = fp2_regs(vd, vm, is_dp);
            format!("{name}.{w} {d}, {m}")
        }
        Inst32::VfpCmp {
            vd,
            vm,
            e,
            z,
            is_dp,
        } => {
            let name = if e { "vcmpe" } else { "vcmp" };
            let (w, d, m) = fp2_regs(vd, vm, is_dp);
            if z {
                format!("{name}.{w} {d}, #0")
            } else {
                format!("{name}.{w} {d}, {m}")
            }
        }
        Inst32::VfpVrint { op, vd, vm, is_dp } => {
            let name = match op {
                VrintOp::R => "vrintr",
                VrintOp::Z => "vrintz",
                VrintOp::X => "vrintx",
            };
            let (w, d, m) = fp2_regs(vd, vm, is_dp);
            format!("{name}.{w} {d}, {m}")
        }
        Inst32::VfpCvtInt {
            to_float,
            signed,
            rz,
            vd,
            vm,
            is_dp,
        } => {
            let float = if is_dp { "f64" } else { "f32" };
            let int = if signed { "s32" } else { "u32" };
            if to_float {
                let (dst, src) = if is_dp {
                    (format!("d{vd}"), format!("r{vm}"))
                } else {
                    (format!("s{vd}"), format!("r{vm}"))
                };
                format!("vcvt.{float}.{int} {dst}, {src}")
            } else {
                let r = if rz { "r" } else { "" };
                let (dst, src) = if is_dp {
                    (format!("r{vd}"), format!("d{vm}"))
                } else {
                    (format!("r{vd}"), format!("s{vm}"))
                };
                format!("vcvt{r}.{int}.{float} {dst}, {src}")
            }
        }
        Inst32::VfpCvtFix {
            opc,
            frac,
            vd,
            is_dp,
        } => {
            // `opc` = `sx:op:u` (bit18, bit16, bit7): `op` selects the
            // float-to-int direction, `u` the signedness, `sx` the width.
            let to_int = opc & 4 != 0;
            let bits16 = opc & 1 == 0;
            let sign = if opc & 2 == 0 { "s" } else { "u" };
            let width = if bits16 { "16" } else { "32" };
            let float = if is_dp { "f64" } else { "f32" };
            let reg = if is_dp {
                format!("d{vd}")
            } else {
                format!("s{vd}")
            };
            if to_int {
                format!("vcvt.{sign}{width}.{float} {reg}, {reg}, #{frac}")
            } else {
                format!("vcvt.{float}.{sign}{width} {reg}, {reg}, #{frac}")
            }
        }
        Inst32::VfpCvt { to_double, vd, vm } => {
            if to_double {
                format!("vcvt.f64.f32 d{vd}, s{vm}")
            } else {
                format!("vcvt.f32.f64 s{vd}, d{vm}")
            }
        }
        Inst32::VfpMovImm { vd, imm, is_dp } => {
            if is_dp {
                format!("vmov.f64 d{vd}, #{}", f64::from_bits(imm))
            } else {
                format!("vmov.f32 s{vd}, #{}", f32::from_bits(imm as u32))
            }
        }
        Inst32::VfpMov { to_gp, vd, rt } => {
            if to_gp {
                format!("vmov {}, s{vd}", reg_name(rt))
            } else {
                format!("vmov s{vd}, {}", reg_name(rt))
            }
        }
        Inst32::VfpMov64 {
            to_gp,
            vm,
            rt,
            rt2,
            is_dp,
        } => {
            if is_dp {
                if to_gp {
                    format!("vmov {}, {}, d{vm}", reg_name(rt), reg_name(rt2))
                } else {
                    format!("vmov d{vm}, {}, {}", reg_name(rt), reg_name(rt2))
                }
            } else if to_gp {
                format!(
                    "vmov {}, {}, s{}, s{}",
                    reg_name(rt),
                    reg_name(rt2),
                    2 * vm,
                    2 * vm + 1
                )
            } else {
                format!(
                    "vmov s{}, s{}, {}, {}",
                    2 * vm,
                    2 * vm + 1,
                    reg_name(rt),
                    reg_name(rt2)
                )
            }
        }
        Inst32::VfpMrsVmsr { is_read, rt } => {
            if is_read {
                format!("vmrs {}, fpscr", reg_name(rt))
            } else {
                format!("vmsr fpscr, {}", reg_name(rt))
            }
        }
        Inst32::VfpLdSt {
            is_load,
            vd,
            rn,
            add,
            imm,
        } => {
            let op = if is_load { "vldr" } else { "vstr" };
            let sign = if add { "" } else { "-" };
            format!("{op} s{vd}, [{}, #{sign}{imm}]", reg_name(rn))
        }
        Inst32::VfpLdStD {
            is_load,
            vd,
            rn,
            add,
            imm,
        } => {
            let op = if is_load { "vldr" } else { "vstr" };
            let sign = if add { "" } else { "-" };
            format!("{op} d{vd}, [{}, #{sign}{imm}]", reg_name(rn))
        }
        Inst32::VfpLdStMult {
            is_load,
            vd,
            rn,
            n,
            writeback,
            dec_before,
        } => {
            let last = vd + n - 1;
            let list = format!("{{s{vd}-s{last}}}");
            if rn == 15 {
                let op = if is_load { "vpop" } else { "vpush" };
                return format!("{op} {list}");
            }
            let op = match (is_load, dec_before) {
                (true, false) => "vldm",
                (false, false) => "vstm",
                (true, true) => "vldmdb",
                (false, true) => "vstmdb",
            };
            let wb = if writeback { "!" } else { "" };
            format!("{op} {}{wb}, {list}", reg_name(rn))
        }
        Inst32::VfpLdStMultD {
            is_load,
            vd,
            rn,
            n,
            writeback,
            dec_before,
        } => {
            let last = vd + n - 1;
            let list = format!("{{d{vd}-d{last}}}");
            if rn == 15 {
                let op = if is_load { "vpop" } else { "vpush" };
                return format!("{op} {list}");
            }
            let op = match (is_load, dec_before) {
                (true, false) => "vldm",
                (false, false) => "vstm",
                (true, true) => "vldmdb",
                (false, true) => "vstmdb",
            };
            let wb = if writeback { "!" } else { "" };
            format!("{op} {}{wb}, {list}", reg_name(rn))
        }
        Inst32::Tt { variant, rd, rn } => {
            let name = match variant {
                TtVariant::Tt => "tt",
                TtVariant::Tttt => "ttt",
                TtVariant::Tta => "tta",
                TtVariant::Ttat => "ttat",
            };
            format!("{name} {}, {}", reg_name(rd), reg_name(rn))
        }
        Inst32::Sg => "sg".to_owned(),
    }
}

fn fp3_regs(vd: u8, vn: u8, vm: u8, is_dp: bool) -> (&'static str, String, String, String) {
    if is_dp {
        ("f64", format!("d{vd}"), format!("d{vn}"), format!("d{vm}"))
    } else {
        ("f32", format!("s{vd}"), format!("s{vn}"), format!("s{vm}"))
    }
}

fn fp2_regs(vd: u8, vm: u8, is_dp: bool) -> (&'static str, String, String) {
    if is_dp {
        ("f64", format!("d{vd}"), format!("d{vm}"))
    } else {
        ("f32", format!("s{vd}"), format!("s{vm}"))
    }
}

/// Disassemble the instruction at `pc`: `h0` always read, `h1` (at
/// `pc + 2`) only consumed when `h0` is a 32-bit prefix for `isa`.
/// Returns the text plus the instruction width in bytes (2 or 4).
/// Undecodable or unreadable halfwords render as `.hword 0x....`.
#[must_use]
pub fn disassemble(isa: Isa, pc: u32, h0: u16, h1: Option<u16>) -> (String, u32) {
    let wide = match isa {
        Isa::Armv6M => is_thumb2(h0),
        _ => is_thumb2_v7m(h0),
    };
    if !wide {
        let decoded = match isa {
            Isa::Armv6M => decode16(h0),
            _ => decode16_v7m(h0),
        };
        let text = match decoded {
            Ok(inst) => fmt16(&inst, pc),
            Err(_) => format!(".hword {h0:#06x}"),
        };
        return (text, 2);
    }
    let Some(h1) = h1 else {
        return (format!(".hword {h0:#06x}"), 2);
    };
    let decoded = match isa {
        Isa::Armv6M => None,
        Isa::Armv7M => decode32_v7m(h0, h1).ok(),
        Isa::Armv7EM => decode32_v7em(h0, h1).ok(),
        Isa::Armv8MBaseline => decode32_v8m(h0, h1).ok(),
        Isa::Armv8MMainline => decode32_v8m_mainline(h0, h1).ok(),
    };
    match decoded {
        Some(inst) => (fmt32(&inst, pc), 4),
        None => (format!(".hword {h0:#06x}"), 2),
    }
}

#[cfg(test)]
mod tests {
    use super::{cond_name, disassemble, reg_name};
    use crate::cpu::Isa;

    fn text(isa: Isa, pc: u32, h0: u16, h1: Option<u16>) -> String {
        disassemble(isa, pc, h0, h1).0
    }

    fn narrow(pc: u32, h0: u16) -> String {
        let (text, width) = disassemble(Isa::Armv7M, pc, h0, None);
        assert_eq!(width, 2);
        text
    }

    #[test]
    fn narrow_data_processing() {
        assert_eq!(narrow(0, 0x215A), "movs r1, #90");
        assert_eq!(narrow(0, 0x1888), "adds r0, r1, r2");
        assert_eq!(narrow(0, 0x1FE3), "subs r3, r4, #7");
        assert_eq!(narrow(0, 0x00B5), "lsls r5, r6, #2");
        // `lsls rd, rm, #0` is the `movs` alias.
        assert_eq!(narrow(0, 0x0007), "movs r7, r0");
        assert_eq!(narrow(0, 0x290A), "cmp r1, #10");
        assert_eq!(narrow(0, 0x4008), "ands r0, r1");
        // `neg rd, rm` is `rsbs rd, rm, #0`.
        assert_eq!(narrow(0, 0x425A), "rsbs r2, r3, #0");
        assert_eq!(narrow(0, 0x4348), "muls r0, r1");
        assert_eq!(narrow(0, 0xB004), "add sp, #16");
        assert_eq!(narrow(0, 0xB082), "sub sp, #8");
        assert_eq!(narrow(0, 0xA901), "add r1, sp, #4");
        assert_eq!(narrow(0, 0xB208), "sxth r0, r1");
        assert_eq!(narrow(0, 0xBA1A), "rev r2, r3");
    }

    #[test]
    fn narrow_memory_and_lists() {
        assert_eq!(narrow(0x1A, 0x68C8), "ldr r0, [r1, #12]");
        assert_eq!(narrow(0, 0x715A), "strb r2, [r3, #5]");
        assert_eq!(narrow(0, 0x5BAC), "ldrh r4, [r5, r6]");
        assert_eq!(narrow(0, 0x9F05), "ldr r7, [sp, #20]");
        assert_eq!(narrow(0, 0xB5F0), "push {r4-r7, lr}");
        assert_eq!(narrow(0, 0xBDF0), "pop {r4-r7, pc}");
        assert_eq!(narrow(0, 0xC00E), "stmia r0!, {r1-r3}");
        assert_eq!(narrow(0, 0xCA50), "ldmia r2!, {r4, r6}");
        assert_eq!(narrow(0, 0x4801), "ldr r0, [pc, #4] ; 0x00000008");
    }

    #[test]
    fn narrow_branches_print_absolute_targets() {
        // `beq` at 0x2A with off +0xFC lands on 0x12A.
        assert_eq!(narrow(0x2A, 0xD07E), "beq 0x0000012a");
        // `b .` at 0x2C is off -4 back onto itself.
        assert_eq!(narrow(0x2C, 0xE7FE), "b 0x0000002c");
        assert_eq!(narrow(0x2E, 0xB171), "cbz r1, 0x0000004e");
        assert_eq!(narrow(0x30, 0xB982), "cbnz r2, 0x00000054");
    }

    #[test]
    fn narrow_system() {
        assert_eq!(narrow(0, 0xDF03), "svc #3");
        assert_eq!(narrow(0, 0xBEAB), "bkpt #171");
        assert_eq!(narrow(0, 0xBF00), "nop");
        assert_eq!(narrow(0, 0xBF30), "wfi");
        assert_eq!(narrow(0, 0xB672), "cpsid i");
        assert_eq!(narrow(0, 0x47A8), "blx r5");
        assert_eq!(narrow(0, 0x4770), "bx lr");
    }

    #[test]
    fn it_letters_follow_the_assembler_masks() {
        // Masks are cond-dependent (gas-verified): 0100 is `itt eq` but
        // `ite ne`; 1100 is `ite eq` but `itt ne`.
        assert_eq!(narrow(0, 0xBF04), "itt eq");
        assert_eq!(narrow(0, 0xBF0C), "ite eq");
        assert_eq!(narrow(0, 0xBF1C), "itt ne");
        assert_eq!(narrow(0, 0xBF14), "ite ne");
        assert_eq!(narrow(0, 0xBF06), "itte eq");
        assert_eq!(narrow(0, 0xBFAA), "itet ge");
        assert_eq!(narrow(0, 0xBF01), "itttt eq");
        // ARMv6-M has no IT: the halfword falls back, never guesses.
        assert_eq!(text(Isa::Armv6M, 0, 0xBF0C, None), ".hword 0xbf0c");
    }

    #[test]
    fn wide_forms_carry_dot_w() {
        let wide = |pc: u32, h0: u16, h1: u16| {
            let (text, width) = disassemble(Isa::Armv7M, pc, h0, Some(h1));
            assert_eq!(width, 4);
            text
        };
        assert_eq!(wide(0x4A, 0xF241, 0x2034), "movw r0, #4660");
        assert_eq!(wide(0x4E, 0xF2C0, 0x01AB), "movt r1, #171");
        assert_eq!(wide(0x52, 0xEB03, 0x0284), "add.w r2, r3, r4, lsl #2");
        assert_eq!(wide(0x56, 0xF2A6, 0x15FF), "sub.w r5, r6, #511");
        assert_eq!(wide(0x60, 0xF843, 0x2C08), "str.w r2, [r3, #-8]");
        assert_eq!(wide(0x66, 0xF855, 0x4026), "ldr.w r4, [r5, r6, lsl #2]");
        assert_eq!(wide(0x6A, 0xE9D0, 0x8904), "ldrd r8, r9, [r0, #16]");
        assert_eq!(wide(0x6E, 0xE921, 0x00F0), "stmdb r1!, {r4-r7}");
        assert_eq!(wide(0x7A, 0xF000, 0xFA7E), "bl 0x0000057a");
        assert_eq!(wide(0x72, 0xF000, 0xB9FE), "b.w 0x00000472");
        assert_eq!(wide(0x7E, 0xFB91, 0xF0F2), "sdiv r0, r1, r2");
        assert_eq!(wide(0x82, 0xFB04, 0x6305), "mla r3, r4, r5, r6");
        assert_eq!(wide(0x86, 0xFB84, 0x2305), "smull r2, r3, r4, r5");
        assert_eq!(wide(0x8A, 0xFAB1, 0xF081), "clz r0, r1");
        assert_eq!(wide(0x8E, 0xFA93, 0xF2A3), "rbit r2, r3");
        assert_eq!(wide(0x92, 0xF341, 0x1007), "sbfx r0, r1, #4, #8");
        assert_eq!(wide(0x96, 0xF363, 0x220F), "bfi r2, r3, #8, #8");
        assert_eq!(wide(0x9A, 0xF36F, 0x0403), "bfc r4, #0, #4");
        assert_eq!(wide(0x9E, 0xF301, 0x2007), "ssat r0, #8, r1, lsl #8");
        assert_eq!(wide(0xA2, 0xF383, 0x020C), "usat r2, #12, r3");
        // ARM syntax is `Rd, Rm, Rn` (the doubling applies to `Rn`).
        assert_eq!(wide(0x7A, 0xFA82, 0xF091), "qdadd r0, r1, r2");
        assert_eq!(wide(0xAA, 0xFA84, 0xF305), "sadd8 r3, r4, r5");
        assert_eq!(wide(0xAE, 0xFAA1, 0xF082), "sel r0, r1, r2");
        assert_eq!(wide(0xB2, 0xFA44, 0xF385), "sxtab r3, r4, r5");
        assert_eq!(wide(0xB8, 0xF3EF, 0x8000), "mrs r0, APSR");
        assert_eq!(wide(0xBC, 0xF381, 0x8811), "msr BASEPRI, r1");
        assert_eq!(wide(0xC0, 0xF3BF, 0x8F5F), "dmb");
        assert_eq!(wide(0xC4, 0xE851, 0x0F00), "ldrex r0, [r1]");
        assert_eq!(wide(0xC8, 0xE844, 0x3200), "strex r2, r3, [r4]");
        assert_eq!(wide(0xCC, 0xE8D5, 0xF006), "tbb [r5, r6]");
        assert_eq!(wide(0xD0, 0xFBB8, 0xF7F9), "udiv r7, r8, r9");
        assert_eq!(wide(0xD4, 0xEEB0, 0x0A60), "vmov.f32 s0, s1");
        assert_eq!(wide(0xD8, 0xEE31, 0x1A82), "vadd.f32 s2, s3, s4");
        assert_eq!(wide(0xDC, 0xEEF1, 0x0A10), "vmrs r0, fpscr");
    }

    #[test]
    fn wide_branches_use_direct_t3_offsets() {
        // `beq.w .+0x404` is F000 8200 (gas); the T3 I-bits read directly.
        assert_eq!(
            text(Isa::Armv7M, 0x76, 0xF000, Some(0x8200)),
            "beq.w 0x0000047a"
        );
        // `beq.w .+8` is F000 8004 (gas).
        assert_eq!(
            text(Isa::Armv7M, 0x00, 0xF000, Some(0x8004)),
            "beq.w 0x0000000c"
        );
        // Backward: `beq.w back` is F43F AFFE (gas), off -4.
        assert_eq!(
            text(Isa::Armv7M, 0x10, 0xF43F, Some(0xAFFE)),
            "beq.w 0x00000010"
        );
    }

    #[test]
    fn undecodable_renders_hword() {
        // 0xBA80 sits in unallocated space (repo decode test).
        assert_eq!(narrow(0x100, 0xBA80), ".hword 0xba80");
        // Truncated 32-bit prefix: width stays 2, no guessing.
        let (text, width) = disassemble(Isa::Armv7M, 0x100, 0xF000, None);
        assert_eq!((text.as_str(), width), (".hword 0xf000", 2));
    }

    #[test]
    fn names_cover_regs_and_conds() {
        assert_eq!(reg_name(0), "r0");
        assert_eq!(reg_name(12), "r12");
        assert_eq!(reg_name(13), "sp");
        assert_eq!(reg_name(14), "lr");
        assert_eq!(reg_name(15), "pc");
        assert_eq!(cond_name(0), "eq");
        assert_eq!(cond_name(0xC), "gt");
        assert_eq!(cond_name(0xE), "al");
    }
}
