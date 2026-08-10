//! The ARMv6-M CPU core: fetch, decode, execute, and the (typed) halt states.
//!
//! The executor is the *only* place that interprets semantics. It is written
//! against `Bus`, so swapping the memory behind the CPU does not change
//! instruction behavior.

pub mod decode;
pub mod decode16;
pub mod decode32;
pub mod registers;

use crate::error::{Exception, HaltCause};
use crate::mem::PeripheralBus;
use decode16::{decode16, AluOp, Inst16, LsImm, LsReg, Shift, ShiftImm};
use decode32::{decode32, is_thumb2, Inst32, SysReg};
use registers::{Registers, Xpsr, APSR_FLAGS, XPSR_T};

/// Which Thumb architecture the core executes.
///
/// ARMv6-M (Cortex-M0/M0+) is a 16-bit ISA plus `bl`/`mrs`/`msr`/barriers;
/// ARMv7-M (Cortex-M3/M4) adds the full Thumb-2 set, IT blocks, and a
/// programmable exception-priority model. The decoder and executor select
/// behavior on this field so a single core type serves both.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Isa {
    Armv6M,
    Armv7M,
}

/// `LR` value loaded on exception entry, causing `bx lr`/`pop {pc}` to return
/// to Handler mode (re-entering the handler that was interrupted).
const EXC_RETURN_HANDLER: u32 = 0xFFFF_FFF1;
/// Return to Thread mode, restoring the main stack pointer.
const EXC_RETURN_THREAD_MSP: u32 = 0xFFFF_FFF9;
/// Return to Thread mode, restoring the process stack pointer.
const EXC_RETURN_THREAD_PSP: u32 = 0xFFFF_FFFD;

/// xPSR `ALIGN` bit (bit 9): set on the stacked xPSR when the interrupted
/// stack was 8-byte-aligned by an exception-entry adjustment.
const XPSR_ALIGN: u32 = 1 << 9;

/// The eight-word ARMv6-M exception frame, in bytes.
const FRAME_BYTES: u32 = 32;

/// What an executed instruction does with the program counter.
enum Next {
    /// Continue at `address`.
    Advance(u32),
    /// The machine halted; propagate the cause.
    Stop(HaltCause),
}

/// The core CPU state plus its bus.
pub struct Core<B: PeripheralBus> {
    pub regs: Registers,
    pub xpsr: Xpsr,
    /// The ISA this core was constructed for. Immutable after construction.
    pub isa: Isa,
    /// PRIMASK (interrupt masking). Only bit `i` of `CPSIE/CPSID` touches it
    /// on ARMv6-M.
    pub primask: bool,
    /// BASEPRI (ARMv7-M): masks exceptions with priority >= this value.
    pub basepri: u8,
    /// FAULTMASK (ARMv7-M): masks every configurable-priority exception.
    pub faultmask: bool,
    /// CONTROL register: bit0 nPRIV, bit1 SPSEL, bit2 FPCA.
    pub control: u32,
    /// Current exception number: 0 in Thread mode, non-zero in Handler mode.
    pub ipsr: u8,
    /// IT block state (ARMv7-M): bits [3:0] condition, bits [7:4] remaining
    /// conditional instructions. Zero outside an IT block.
    pub itstate: u8,
    pub cycles: u64,
    mem: B,
}

impl<B: PeripheralBus> Core<B> {
    /// Create an uninitialized ARMv6-M core over `mem`. Register state is zero
    /// until `reset_from_vector_table()` or host code sets it.
    pub fn new(mem: B) -> Self {
        Self::new_with_isa(mem, Isa::Armv6M)
    }

    /// Create an uninitialized ARMv7-M core over `mem`.
    pub fn armv7m(mem: B) -> Self {
        Self::new_with_isa(mem, Isa::Armv7M)
    }

    fn new_with_isa(mem: B, isa: Isa) -> Self {
        Self {
            regs: Registers::default(),
            xpsr: Xpsr::new(XPSR_T),
            isa,
            primask: false,
            basepri: 0,
            faultmask: false,
            control: 0,
            ipsr: 0,
            itstate: 0,
            cycles: 0,
            mem,
        }
    }

    /// Borrow the bus.
    pub fn mem(&mut self) -> &mut B {
        &mut self.mem
    }

    /// Fetch and execute a single instruction. On success the machine has
    /// advanced exactly one instruction; on error it has *not*. Interrupts
    /// are taken by the `Machine`, not here.
    pub fn step(&mut self) -> Result<(), HaltCause> {
        let pc = self.regs.pc;
        let next = match self.isa {
            Isa::Armv6M => self.step_v6m(pc)?,
            Isa::Armv7M => self.step_v7m(pc)?,
        };
        match next {
            Next::Advance(a) => self.regs.pc = a,
            Next::Stop(cause) => return Err(cause),
        }
        self.cycles += 1;
        Ok(())
    }

    /// Fetch and execute a single 16-bit-centric ARMv6-M instruction.
    fn step_v6m(&mut self, pc: u32) -> Result<Next, HaltCause> {
        let hw = self.mem.read_half(pc).map_err(|_| HaltCause::FetchAbort)?;
        if is_thumb2(hw) {
            let lo = self
                .mem
                .read_half(pc.wrapping_add(2))
                .map_err(|_| HaltCause::FetchAbort)?;
            match decode32(hw, lo) {
                Ok(inst) => self.exec32(pc, inst),
                Err(_) => self.hardfault_on_undefined(pc, 4),
            }
        } else {
            match decode16(hw) {
                Ok(inst) => self.exec(pc, inst),
                Err(_) => self.hardfault_on_undefined(pc, 2),
            }
        }
    }

    /// Fetch and execute a single ARMv7-M instruction.
    ///
    /// ARMv7-M fetches 32-bit instructions only from 4-byte-aligned addresses.
    /// The fetch always reads the aligned halfword pair `[pc & ~3, pc + 2]`;
    /// which halfword the instruction starts at is decided by `pc & 2`.
    fn step_v7m(&mut self, pc: u32) -> Result<Next, HaltCause> {
        if pc & 1 != 0 {
            return Err(HaltCause::FetchAbort);
        }
        let aligned = pc & !3;
        let h0 = self.mem.read_half(aligned).map_err(|_| HaltCause::FetchAbort)?;
        let h1 = self
            .mem
            .read_half(aligned.wrapping_add(2))
            .map_err(|_| HaltCause::FetchAbort)?;
        // The halfword at `pc` is the start of the instruction: `h0` for an
        // even 4-byte slot, `h1` when the PC sits on the second halfword.
        let (first, second) = if pc & 2 == 0 { (h0, h1) } else { (h1, h0) };
        if is_thumb2(first) {
            match decode32(first, second) {
                Ok(inst) => self.exec32(pc, inst),
                Err(_) => self.hardfault_on_undefined(pc, 4),
            }
        } else {
            match decode16(first) {
                Ok(inst) => self.exec(pc, inst),
                Err(_) => self.hardfault_on_undefined(pc, 2),
            }
        }
    }

    /// Execute until the machine halts of its own accord, or `budget`
    /// instructions have been executed (whichever comes first).
    pub fn run(&mut self, budget: u64) -> Result<(), HaltCause> {
        for _ in 0..budget {
            self.step()?;
        }
        Err(HaltCause::CycleBudgetExhausted)
    }

    /// Bootstrap the core from the reset vector table: `sp = [0x0000_0000]`,
    /// `pc = [0x0000_0004] & ~1`. Returns `NonThumbTarget` if the reset
    /// vector is not a Thumb address.
    pub fn reset_from_vector_table(&mut self) -> Result<(), HaltCause> {
        let sp = self.read_word_halt(0)?;
        let pc = self.read_word_halt(4)?;
        if pc & 1 == 0 {
            return Err(HaltCause::NonThumbTarget);
        }
        self.regs.sp = sp;
        self.regs.msp = sp;
        self.regs.psp = 0;
        self.regs.pc = pc & !1;
        self.xpsr = Xpsr::new(XPSR_T);
        self.primask = false;
        self.basepri = 0;
        self.faultmask = false;
        self.control = 0;
        self.ipsr = 0;
        self.itstate = 0;
        Ok(())
    }

    /// Point-in-time register snapshot for host tooling (cli, debugger).
    pub const fn snapshot(&self) -> (Registers, Xpsr, u64) {
        (self.regs, self.xpsr, self.cycles)
    }

    fn read_word_halt(&mut self, addr: u32) -> Result<u32, HaltCause> {
        self.mem
            .read_word(addr)
            .map_err(|_| HaltCause::BusFault { address: addr })
    }

    fn write_word_halt(&mut self, addr: u32, v: u32) -> Result<(), HaltCause> {
        self.mem
            .write_word(addr, v)
            .map_err(|_| HaltCause::BusFault { address: addr })
    }

    /// Whether the active stack pointer is the process SP (`SPSEL == 1`).
    /// In Handler mode the active SP is always the main stack pointer, so
    /// `SPSEL` is only consulted for thread-mode accesses.
    const fn sp_is_process(&self) -> bool {
        self.ipsr == 0 && self.control & 0x2 != 0
    }

    /// Read a system register for `mrs`.
    fn sysreg_read(&self, sys: SysReg) -> Result<u32, HaltCause> {
        let v = match sys {
            SysReg::Apsr => self.xpsr.get() & APSR_FLAGS,
            SysReg::Xpsr => self.xpsr.get() & (APSR_FLAGS | XPSR_T),
            SysReg::Ipsr => u32::from(self.ipsr),
            SysReg::Epsr => XPSR_T,
            SysReg::Msp => {
                if self.sp_is_process() {
                    self.regs.msp
                } else {
                    self.regs.sp
                }
            }
            SysReg::Psp => {
                if self.sp_is_process() {
                    self.regs.sp
                } else {
                    self.regs.psp
                }
            }
            SysReg::Primask => u32::from(self.primask),
            SysReg::Control => self.control,
            SysReg::Other(_) => {
                return Err(HaltCause::NotImplemented {
                    what: "mrs of reserved system register",
                })
            }
        };
        Ok(v)
    }

    /// Write a system register for `msr`.
    fn sysreg_write(&mut self, sys: SysReg, v: u32) -> Result<(), HaltCause> {
        match sys {
            SysReg::Apsr => {
                let cur = self.xpsr.get();
                self.xpsr.set((cur & !APSR_FLAGS) | (v & APSR_FLAGS));
            }
            SysReg::Msp => {
                if self.sp_is_process() {
                    self.regs.msp = v;
                } else {
                    self.regs.sp = v;
                }
            }
            SysReg::Psp => {
                if self.sp_is_process() {
                    self.regs.sp = v;
                } else {
                    self.regs.psp = v;
                }
            }
            SysReg::Primask => self.primask = v & 1 != 0,
            SysReg::Control => {
                let new = v & 0x7;
                let want_process = new & 0x2 != 0;
                let was_process = self.sp_is_process();
                if want_process != was_process {
                    let r = &mut self.regs;
                    if want_process {
                        r.msp = r.sp;
                        r.sp = r.psp;
                    } else {
                        r.psp = r.sp;
                        r.sp = r.msp;
                    }
                }
                self.control = new;
            }
            SysReg::Xpsr | SysReg::Ipsr | SysReg::Epsr => {
                return Err(HaltCause::NotImplemented {
                    what: "msr to xpsr/ipsr/epsr",
                })
            }
            SysReg::Other(_) => {
                return Err(HaltCause::NotImplemented {
                    what: "msr of reserved system register",
                })
            }
        }
        Ok(())
    }

    /// Evaluate an ARM condition code against the current flags.
    fn cond_passes(&self, cond: u8) -> bool {
        let (n, z, c, v) = (self.xpsr.n(), self.xpsr.z(), self.xpsr.c(), self.xpsr.v());
        match cond {
            0x0 => z,            // EQ
            0x1 => !z,           // NE
            0x2 => c,            // CS/HS
            0x3 => !c,           // CC/LO
            0x4 => n,            // MI
            0x5 => !n,           // PL
            0x6 => v,            // VS
            0x7 => !v,           // VC
            0x8 => c && !z,      // HI
            0x9 => !c || z,      // LS
            0xA => n == v,       // GE
            0xB => n != v,       // LT
            0xC => !z && n == v, // GT
            0xD => z || n != v,  // LE
            _ => unreachable!("cond {cond:#x}"),
        }
    }

    /// Execute `inst` at address `pc`. Semantic meaning only; no memory
    /// acquisition happens here.
    fn exec(&mut self, pc: u32, inst: Inst16) -> Result<Next, HaltCause> {
        use Inst16 as I;
        match inst {
            I::ShiftImm {
                rd,
                rm,
                kind,
                amount,
            } => {
                let v = self.regs.read(rm);
                let (res, c) = shift_imm(kind, v, u32::from(amount));
                self.write_shift_flags(res, c);
                self.regs.write(rd, res);
                Ok(Next::Advance(pc + 2))
            }
            I::ShiftReg { rd, rn, rm, kind } => {
                let v = self.regs.read(rn);
                let count = self.regs.read(rm) & 0xFF;
                let (res, c) = shift_reg(kind, v, count);
                self.write_shift_flags(res, c);
                self.regs.write(rd, res);
                Ok(Next::Advance(pc + 2))
            }
            I::AddReg { rd, rn, rm } => {
                let (res, c, v) = add_flags(self.regs.read(rn), self.regs.read(rm), false);
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubReg { rd, rn, rm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), self.regs.read(rm), false);
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::AddImm3 { rd, rn, imm } => {
                let (res, c, v) = add_flags(self.regs.read(rn), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubImm3 { rd, rn, imm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::MovImm { rd, imm } => {
                let res = u32::from(imm);
                self.regs.write(rd, res);
                self.set_nz(res, false);
                Ok(Next::Advance(pc + 2))
            }
            I::CmpImm { rn, imm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), u32::from(imm), false);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::AddImm8 { rd, imm } => {
                let (res, c, v) = add_flags(self.regs.read(rd), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubImm8 { rd, imm } => {
                let (res, c, v) = sub_flags(self.regs.read(rd), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::Alu { rd, rm, op } => {
                self.exec_alu(rd, rm, op);
                Ok(Next::Advance(pc + 2))
            }
            I::AddSpImm8 { imm } => {
                self.regs.sp = self.regs.sp.wrapping_add(imm as u32);
                Ok(Next::Advance(pc + 2))
            }
            I::SubSpImm8 { imm } => {
                self.regs.sp = self.regs.sp.wrapping_sub(imm as u32);
                Ok(Next::Advance(pc + 2))
            }
            I::SpecialAdd { rd, rm } => {
                let b = self.regs.read(rm);
                if rd == 15 {
                    return self.branch(pc.wrapping_add(4).wrapping_add(b));
                }
                self.regs.write(rd, self.regs.read(rd) + b);
                Ok(Next::Advance(pc + 2))
            }
            I::SpecialCmp { rn, rm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), self.regs.read(rm), false);
                self.set_nzcv(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SpecialMov { rd, rm } => {
                let b = self.regs.read(rm);
                if rd == 15 {
                    return self.branch(b);
                }
                self.regs.write(rd, b);
                Ok(Next::Advance(pc + 2))
            }
            I::Bx { rm } => self.branch(self.regs.read(rm)),
            I::Blx { rm } => {
                let target = self.regs.read(rm);
                self.regs.lr = (pc + 2) | 1;
                Ok(Self::check_branch(target))
            }
            I::LdrLiteral { rt, imm } => {
                let addr = ((pc + 4) & !3) + imm as u32;
                let v = match self.mem.read_word(addr) {
                    Ok(v) => v,
                    Err(_) => return self.data_fault(addr, pc),
                };
                self.write_rt(rt, v, addr, pc)
            }
            I::LsReg { rt, rn, rm, kind } => {
                let addr = self.regs.read(rn).wrapping_add(self.regs.read(rm));
                self.exec_ls(addr, rt, kind.into(), pc)
            }
            I::LsImm { rt, rn, imm, kind } => {
                let addr = self.regs.read(rn).wrapping_add(imm as u32);
                self.exec_ls(addr, rt, kind.into(), pc)
            }
            I::LsSp { rt, imm, is_load } => {
                let addr = self.regs.sp.wrapping_add(imm as u32);
                self.exec_ls(addr, rt, LsAccess::Word { load: is_load }, pc)
            }
            I::AddrPc { rd, imm } => {
                let v = ((pc + 4) & !3) + imm as u32;
                self.regs.write(rd, v);
                Ok(Next::Advance(pc + 2))
            }
            I::AddrSp { rd, imm } => {
                self.regs.write(rd, self.regs.sp.wrapping_add(imm as u32));
                Ok(Next::Advance(pc + 2))
            }
            I::BranchCond { cond, off } => {
                if self.cond_passes(cond) {
                    Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
                } else {
                    Ok(Next::Advance(pc + 2))
                }
            }
            I::Branch { off } => Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off))),
            I::Svc { imm } => Ok(Next::Stop(HaltCause::Svc(imm))),
            I::Bkpt { .. } => Ok(Next::Stop(HaltCause::Breakpoint)),
            I::Hint { .. } => Ok(Next::Advance(pc + 2)),
            I::Cps { disable, im } => {
                // Only the `i` bit (bit1) has an architectural effect on
                // ARMv6-M: it controls PRIMASK. `f`/`a` are aliases that do
                // not exist on the M0.
                if im & 0b010 != 0 {
                    self.primask = disable;
                }
                Ok(Next::Advance(pc + 2))
            }
            I::Push { regs, include_lr } => {
                let count = regs.count_ones() + u32::from(include_lr);
                let mut addr = self.regs.sp.wrapping_sub(4 * count);
                for r in 0..8 {
                    if regs & (1 << r) != 0 {
                        let v = self.regs.read(r);
                        self.write_word_halt(addr, v)?;
                        addr = addr.wrapping_add(4);
                    }
                }
                if include_lr {
                    self.write_word_halt(addr, self.regs.lr)?;
                }
                self.regs.sp = self.regs.sp.wrapping_sub(4 * count);
                Ok(Next::Advance(pc + 2))
            }
            I::Pop { regs, load_pc } => {
                let mut addr = self.regs.sp;
                for r in 0..8 {
                    if regs & (1 << r) != 0 {
                        let v = self.read_word_halt(addr)?;
                        self.regs.write(r, v);
                        addr = addr.wrapping_add(4);
                    }
                }
                let pc_word = if load_pc {
                    Some(self.read_word_halt(addr)?)
                } else {
                    None
                };
                let count = regs.count_ones() + u32::from(load_pc);
                self.regs.sp = self.regs.sp.wrapping_add(4 * count);
                if let Some(target) = pc_word {
                    return self.branch(target);
                }
                Ok(Next::Advance(pc + 2))
            }
            I::LdmStm { is_load, rn, regs } => {
                let base = self.regs.read(rn);
                let mut addr = base;
                if is_load {
                    for r in 0..8 {
                        if regs & (1 << r) != 0 {
                            let v = self.read_word_halt(addr)?;
                            self.regs.write(r, v);
                            addr = addr.wrapping_add(4);
                        }
                    }
                    // Writeback; if rn was loaded, the loaded value wins.
                    if regs & (1 << rn) == 0 {
                        self.regs
                            .write(rn, base.wrapping_add(4 * regs.count_ones()));
                    }
                } else {
                    for r in 0..8 {
                        if regs & (1 << r) != 0 {
                            let v = self.regs.read(r);
                            self.write_word_halt(addr, v)?;
                            addr = addr.wrapping_add(4);
                        }
                    }
                    self.regs
                        .write(rn, base.wrapping_add(4 * regs.count_ones()));
                }
                Ok(Next::Advance(pc + 2))
            }
        }
    }

    /// Execute a 32-bit ARMv6-M instruction. Every 32-bit instruction is
    /// exactly 4 bytes, so a non-branching one simply advances `pc + 4`.
    fn exec32(&mut self, pc: u32, inst: Inst32) -> Result<Next, HaltCause> {
        use Inst32 as I;
        match inst {
            I::Bl { off } => {
                self.regs.lr = (pc.wrapping_add(4)) | 1;
                Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
            }
            I::Mrs { rd, sys } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented { what: "mrs to pc" });
                }
                let v = self.sysreg_read(sys)?;
                self.regs.write(rd, v);
                Ok(Next::Advance(pc + 4))
            }
            I::Msr { rn, sys } => {
                let v = self.regs.read(rn);
                self.sysreg_write(sys, v)?;
                Ok(Next::Advance(pc + 4))
            }
            I::Barrier { .. } => Ok(Next::Advance(pc + 4)),
        }
    }

    fn exec_alu(&mut self, rd: u8, rm: u8, op: AluOp) {
        let a = self.regs.read(rd);
        let b = self.regs.read(rm);
        match op {
            AluOp::And => {
                let res = a & b;
                self.regs.write(rd, res);
                self.set_nz(res, false);
            }
            AluOp::Eor => {
                let res = a ^ b;
                self.regs.write(rd, res);
                self.set_nz(res, false);
            }
            AluOp::Lsl => {
                let (res, c) = shift_reg(Shift::Lsl, a, b & 0xFF);
                self.regs.write(rd, res);
                self.write_shift_flags(res, c);
            }
            AluOp::Lsr => {
                let (res, c) = shift_reg(Shift::Lsr, a, b & 0xFF);
                self.regs.write(rd, res);
                self.write_shift_flags(res, c);
            }
            AluOp::Asr => {
                let (res, c) = shift_reg(Shift::Asr, a, b & 0xFF);
                self.regs.write(rd, res);
                self.write_shift_flags(res, c);
            }
            AluOp::Adc => {
                let (res, c, v) = add_flags(a, b, self.xpsr.c());
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
            }
            AluOp::Sbc => {
                let (res, c, v) = sub_flags(a, b, !self.xpsr.c());
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
            }
            AluOp::Ror => {
                let (res, c) = shift_reg(Shift::Ror, a, b & 0xFF);
                self.regs.write(rd, res);
                self.write_shift_flags(res, c);
            }
            AluOp::Tst => {
                let res = a & b;
                self.set_nz(res, false);
            }
            AluOp::Neg => {
                let (res, c, v) = sub_flags(0, a, false);
                self.regs.write(rd, res);
                self.set_nzcv(res, c, v);
            }
            AluOp::Cmp => {
                let (res, c, v) = sub_flags(a, b, false);
                self.set_nzcv(res, c, v);
            }
            AluOp::Cmn => {
                let (res, c, v) = add_flags(a, b, false);
                self.set_nzcv(res, c, v);
            }
            AluOp::Orr => {
                let res = a | b;
                self.regs.write(rd, res);
                self.set_nz(res, false);
            }
            AluOp::Mul => {
                let res = a.wrapping_mul(b);
                self.regs.write(rd, res);
                self.set_nzcv(res, false, false);
            }
            AluOp::Bic => {
                let res = a & !b;
                self.regs.write(rd, res);
                self.set_nz(res, false);
            }
            AluOp::Mvn => {
                let res = !a;
                self.regs.write(rd, res);
                self.set_nz(res, false);
            }
        }
    }

    /// A load or store with a fully-computed `addr`.
    fn exec_ls(&mut self, addr: u32, rt: u8, access: LsAccess, pc: u32) -> Result<Next, HaltCause> {
        let is_load = matches!(
            access,
            LsAccess::Word { load: true }
                | LsAccess::Half { load: true }
                | LsAccess::Byte { load: true }
                | LsAccess::SextByte
                | LsAccess::SextHalf
        );
        if is_load {
            let v = match access {
                LsAccess::Word { load: _ } => match self.mem.read_word(addr) {
                    Ok(v) => v,
                    Err(_) => return self.data_fault(addr, pc),
                },
                LsAccess::Half { load: true } => match self.mem.read_half(addr) {
                    Ok(hw) => u32::from(hw),
                    Err(_) => return self.data_fault(addr, pc),
                },
                LsAccess::Byte { load: true } => match self.mem.read(addr) {
                    Ok(b) => u32::from(b),
                    Err(_) => return self.data_fault(addr, pc),
                },
                LsAccess::SextByte => match self.mem.read(addr) {
                    Ok(b) => (b as i8) as u32,
                    Err(_) => return self.data_fault(addr, pc),
                },
                LsAccess::SextHalf => match self.mem.read_half(addr) {
                    Ok(hw) => (hw as i16) as u32,
                    Err(_) => return self.data_fault(addr, pc),
                },
                _ => unreachable!(),
            };
            self.write_rt(rt, v, addr, pc)
        } else {
            match access {
                LsAccess::Word { load: false } => {
                    let v = self.regs.read(rt);
                    match self.mem.write_word(addr, v) {
                        Ok(()) => {}
                        Err(_) => return self.data_fault(addr, pc),
                    }
                }
                LsAccess::Half { load: false } => {
                    let v = self.regs.read(rt) as u16;
                    match self.mem.write_half(addr, v) {
                        Ok(()) => {}
                        Err(_) => return self.data_fault(addr, pc),
                    }
                }
                LsAccess::Byte { load: false } => {
                    let v = self.regs.read(rt) as u8;
                    match self.mem.write(addr, v) {
                        Ok(()) => {}
                        Err(_) => return self.data_fault(addr, pc),
                    }
                }
                _ => unreachable!(),
            }
            Ok(Next::Advance(pc + 2))
        }
    }

    /// Write a loaded value to `rt`. If `rt == 15` the load is a branch
    /// (Thumb bit checked, EXC_RETURN honored).
    fn write_rt(&mut self, rt: u8, v: u32, addr: u32, pc: u32) -> Result<Next, HaltCause> {
        let _ = addr;
        if rt == 15 {
            return self.branch(v);
        }
        self.regs.write(rt, v);
        Ok(Next::Advance(pc + 2))
    }

    /// Turn a Thumb target into a `Next`, or a `NonThumbTarget` stop.
    fn check_branch(target: u32) -> Next {
        if target & 1 == 0 {
            return Next::Stop(HaltCause::NonThumbTarget);
        }
        Next::Advance(target & !1)
    }

    /// Take a branch to `target`, unless it is an `EXC_RETURN` while a
    /// handler is active, in which case perform the exception return.
    fn branch(&mut self, target: u32) -> Result<Next, HaltCause> {
        if self.ipsr != 0 && is_exc_return(target) {
            return self.exception_return(target);
        }
        Ok(Self::check_branch(target))
    }

    /// Raise a HardFault for an undecodable instruction at `pc`. The stacked
    /// return address is the following instruction so the handler can skip
    /// the bad opcode and continue. Returns the branch to the handler entry.
    fn hardfault_on_undefined(&mut self, pc: u32, size: u32) -> Result<Next, HaltCause> {
        self.take_exception(Exception::HardFault, pc.wrapping_add(size))?;
        Ok(Next::Advance(self.regs.pc))
    }

    /// Escalate a faulting data access at `addr` (from instruction at `pc`)
    /// to a HardFault, per ARMv6-M (no separate bus-fault exception exists).
    /// The stacked return address is the faulting instruction so the handler
    /// can fix the cause and retry it. Continues into the handler.
    fn data_fault(&mut self, addr: u32, pc: u32) -> Result<Next, HaltCause> {
        let _ = addr;
        self.take_exception(Exception::HardFault, pc)?;
        Ok(Next::Advance(self.regs.pc))
    }

    /// Enter Handler mode for `ex`, pushing the eight-word frame onto the
    /// interrupted context's stack and switching to the main stack pointer.
    /// `return_pc` is where execution resumes on exception return.
    fn take_exception(&mut self, ex: Exception, return_pc: u32) -> Result<(), HaltCause> {
        // ARMv6-M faults do not nest in the set we model: taking a HardFault
        // while a handler is already active is a double fault, which halts.
        // Interrupts preempt legitimately and are always legal here; the
        // interrupt controller only offers exceptions that outrank the
        // currently running one.
        if ex == Exception::HardFault && self.ipsr != 0 {
            return Err(HaltCause::UnhandledException);
        }
        let vector_addr = u32::from(ex.number()) * 4;
        let vector = match self.read_word_halt(vector_addr) {
            Ok(v) => v,
            Err(_) => return Err(HaltCause::UnhandledException),
        };
        if vector & 1 == 0 {
            return Err(HaltCause::NonThumbTarget);
        }
        // The frame is pushed onto the stack the interrupted context was
        // using; the handler then runs on the main stack pointer.
        let use_process = self.sp_is_process();
        let mut sp = self.regs.sp;
        let mut sxpsr = self.xpsr.get() & (APSR_FLAGS | XPSR_T) | u32::from(self.ipsr);
        if sp & 0x4 != 0 {
            sp = sp.wrapping_sub(4);
            sxpsr |= XPSR_ALIGN;
        }
        let frame = sp.wrapping_sub(FRAME_BYTES);
        for (i, r) in [0u8, 1, 2, 3, 12].into_iter().enumerate() {
            self.write_word_halt(frame.wrapping_add((i as u32) * 4), self.regs.read(r))?;
        }
        self.write_word_halt(frame.wrapping_add(20), self.regs.lr)?;
        self.write_word_halt(frame.wrapping_add(24), return_pc | 1)?;
        self.write_word_halt(frame.wrapping_add(28), sxpsr)?;
        self.regs.lr = if self.ipsr != 0 {
            EXC_RETURN_HANDLER
        } else if use_process {
            EXC_RETURN_THREAD_PSP
        } else {
            EXC_RETURN_THREAD_MSP
        };
        if use_process {
            self.regs.psp = frame;
            self.regs.sp = self.regs.msp;
        } else {
            self.regs.msp = frame;
            self.regs.sp = frame;
        }
        self.regs.pc = vector & !1;
        self.ipsr = ex.number();
        Ok(())
    }

    /// Enter Handler mode for exception number `n`, as decided by the
    /// machine's interrupt controller. The return address is the instruction
    /// the core is currently pointing at (the next one to execute on return).
    pub fn enter_exception(&mut self, n: u8) -> Result<(), HaltCause> {
        let ex = match n {
            3 => Exception::HardFault,
            15 => Exception::SysTick,
            16..=79 => Exception::Irq(n - 16),
            _ => return Err(HaltCause::UnhandledException),
        };
        self.take_exception(ex, self.regs.pc)
    }

    /// Unstack the eight-word frame and leave Handler mode, as directed by an
    /// `EXC_RETURN` value loaded into the PC. Returns the branch to the
    /// restored program counter.
    fn exception_return(&mut self, value: u32) -> Result<Next, HaltCause> {
        let to_handler = value == EXC_RETURN_HANDLER;
        let to_process = value == EXC_RETURN_THREAD_PSP;
        let to_main = value == EXC_RETURN_THREAD_MSP;
        if !(to_handler || to_process || to_main) {
            return Err(HaltCause::UnhandledException);
        }
        // In Handler mode the live stack pointer is the main SP; process
        // returns unstack the frame from the banked process SP.
        let frame = if to_process {
            self.regs.psp
        } else {
            self.regs.sp
        };
        let r0 = self.read_word_halt(frame.wrapping_add(0))?;
        let r1 = self.read_word_halt(frame.wrapping_add(4))?;
        let r2 = self.read_word_halt(frame.wrapping_add(8))?;
        let r3 = self.read_word_halt(frame.wrapping_add(12))?;
        let r12 = self.read_word_halt(frame.wrapping_add(16))?;
        let lr = self.read_word_halt(frame.wrapping_add(20))?;
        let pc_ret = self.read_word_halt(frame.wrapping_add(24))?;
        let sxpsr = self.read_word_halt(frame.wrapping_add(28))?;
        if pc_ret & 1 == 0 {
            return Err(HaltCause::NonThumbTarget);
        }
        // A return-to-process hands the thread the handler's final main SP,
        // captured before the registers below switch the active pointer.
        let handler_msp = self.regs.sp;
        let align = sxpsr & XPSR_ALIGN != 0;
        let sp = frame
            .wrapping_add(FRAME_BYTES)
            .wrapping_add(u32::from(align) * 4);
        self.regs.write(0, r0);
        self.regs.write(1, r1);
        self.regs.write(2, r2);
        self.regs.write(3, r3);
        self.regs.write(12, r12);
        self.regs.lr = lr;
        self.regs.pc = pc_ret & !1;
        self.xpsr.set(sxpsr & (APSR_FLAGS | XPSR_T));
        if to_process {
            self.regs.msp = handler_msp;
            self.regs.psp = sp;
            self.regs.sp = sp;
            self.control |= 0x2;
            self.ipsr = 0;
        } else if to_main {
            self.regs.sp = sp;
            self.regs.msp = sp;
            self.control &= !0x2;
            self.ipsr = 0;
        } else {
            // Return into the interrupted handler: IPSR comes from the
            // stacked xPSR's IPSR field.
            self.regs.sp = sp;
            self.regs.msp = sp;
            self.ipsr = (sxpsr & 0xFF) as u8;
        }
        Ok(Next::Advance(self.regs.pc))
    }

    fn set_nz(&mut self, res: u32, v: bool) {
        self.xpsr
            .set_nzcv(res >> 31 == 1, res == 0, self.xpsr.c(), v);
    }

    fn set_nzcv(&mut self, res: u32, c: bool, v: bool) {
        self.xpsr.set_nzcv(res >> 31 == 1, res == 0, c, v);
    }

    fn write_shift_flags(&mut self, res: u32, c: Option<bool>) {
        let c = c.unwrap_or_else(|| self.xpsr.c());
        self.xpsr.set_nzcv(res >> 31 == 1, res == 0, c, false);
    }
}

/// Whether `v` is one of the ARMv6-M `EXC_RETURN` values.
const fn is_exc_return(v: u32) -> bool {
    matches!(
        v,
        EXC_RETURN_HANDLER | EXC_RETURN_THREAD_MSP | EXC_RETURN_THREAD_PSP
    )
}

/// A fully-resolved load/store (word/half/byte × load/store, plus
/// sign-extending loads).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LsAccess {
    Word { load: bool },
    Half { load: bool },
    Byte { load: bool },
    SextByte,
    SextHalf,
}

impl From<LsReg> for LsAccess {
    fn from(k: LsReg) -> Self {
        match k {
            LsReg::Str => Self::Word { load: false },
            LsReg::Strh => Self::Half { load: false },
            LsReg::Strb => Self::Byte { load: false },
            LsReg::Ldr => Self::Word { load: true },
            LsReg::Ldrh => Self::Half { load: true },
            LsReg::Ldrb => Self::Byte { load: true },
            LsReg::Ldrsb => Self::SextByte,
            LsReg::Ldrsh => Self::SextHalf,
        }
    }
}

impl From<LsImm> for LsAccess {
    fn from(k: LsImm) -> Self {
        match k {
            LsImm::Str => Self::Word { load: false },
            LsImm::Ldr => Self::Word { load: true },
            LsImm::Strb => Self::Byte { load: false },
            LsImm::Ldrb => Self::Byte { load: true },
            LsImm::Strh => Self::Half { load: false },
            LsImm::Ldrh => Self::Half { load: true },
        }
    }
}

/// `a + b + carry_in`, returning `(result, carry_out, overflow)`.
fn add_flags(a: u32, b: u32, carry_in: bool) -> (u32, bool, bool) {
    let sum = (u64::from(a)) + (u64::from(b)) + u64::from(carry_in);
    let res = sum as u32;
    let carry = sum >> 32 != 0;
    let overflow = ((a ^ res) & (b ^ res)) & 0x8000_0000 != 0;
    (res, carry, overflow)
}

/// `a - b - borrow_in`, returning `(result, carry_out, overflow)`.
/// `carry_out` is the C flag: borrow complements it (C=1 means no borrow).
fn sub_flags(a: u32, b: u32, borrow_in: bool) -> (u32, bool, bool) {
    let res = a.wrapping_sub(b).wrapping_sub(u32::from(borrow_in));
    // C = 1 if the subtraction didn't need a borrow at the top.
    let carry = u64::from(a) >= u64::from(b) + u64::from(borrow_in);
    let overflow = ((a ^ b) & (a ^ res)) & 0x8000_0000 != 0;
    (res, carry, overflow)
}

/// Shift `v` by an immediate 5-bit `amount` (LSL/LSR/ASR #n). Returns
/// `(result, new_carry)`. A `None` carry means the flag is unchanged (shift by
/// zero).
const fn shift_imm(kind: ShiftImm, v: u32, amount: u32) -> (u32, Option<bool>) {
    let amount = amount & 0x1F;
    shift_by_count(kind, v, amount)
}

/// Shift `v` by a register `count` (LSL/LSR/ASR/ROR by register).
fn shift_reg(kind: Shift, v: u32, count: u32) -> (u32, Option<bool>) {
    match kind {
        Shift::Ror => {
            let n = count & 0x1F;
            if n == 0 {
                (v, None)
            } else {
                let carry = (v >> (n - 1)) & 1 != 0;
                (v.rotate_right(n), Some(carry))
            }
        }
        Shift::Lsl | Shift::Lsr | Shift::Asr => {
            let kind = match kind {
                Shift::Lsl => ShiftImm::Lsl,
                Shift::Lsr => ShiftImm::Lsr,
                Shift::Asr => ShiftImm::Asr,
                Shift::Ror => unreachable!(),
            };
            shift_by_count(kind, v, count)
        }
    }
}

/// Shift by an arbitrary count (may exceed 31 for register shifts).
const fn shift_by_count(kind: ShiftImm, v: u32, count: u32) -> (u32, Option<bool>) {
    match kind {
        ShiftImm::Lsl => {
            if count == 0 {
                (v, None)
            } else if count < 32 {
                let carry = (v >> (32 - count)) & 1 != 0;
                (v << count, Some(carry))
            } else if count == 32 {
                (0, Some(v & 1 != 0))
            } else {
                (0, Some(false))
            }
        }
        ShiftImm::Lsr => {
            if count == 0 {
                (v, None)
            } else if count < 32 {
                let carry = (v >> (count - 1)) & 1 != 0;
                (v >> count, Some(carry))
            } else if count == 32 {
                (0, Some(v >> 31 & 1 != 0))
            } else {
                (0, Some(false))
            }
        }
        ShiftImm::Asr => {
            if count == 0 {
                (v, None)
            } else if count < 32 {
                let carry = (v >> (count - 1)) & 1 != 0;
                (((v as i32) >> count) as u32, Some(carry))
            } else {
                // Arithmetic shift of >=32: sign-fill; C = sign bit.
                let sign = v >> 31 & 1 != 0;
                (if sign { u32::MAX } else { 0 }, Some(sign))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_flags_basic() {
        let (r, c, v) = add_flags(0xFFFF_FFFF, 1, false);
        assert_eq!(r, 0);
        assert!(c);
        assert!(!v);
    }

    #[test]
    fn add_flags_overflow() {
        let (_, _, v) = add_flags(0x7FFF_FFFF, 1, false);
        assert!(v);
    }

    #[test]
    fn sub_flags_borrow() {
        let (r, c, v) = sub_flags(0, 1, false);
        assert_eq!(r, u32::MAX);
        assert!(!c);
        assert!(!v);
    }

    #[test]
    fn sub_flags_no_borrow() {
        let (r, c, _) = sub_flags(5, 3, false);
        assert_eq!(r, 2);
        assert!(c);
    }

    #[test]
    fn lsl_flags() {
        let (r, c) = shift_imm(ShiftImm::Lsl, 0x0000_0001, 31);
        assert_eq!(r, 0x8000_0000);
        assert_eq!(c, Some(false));
    }

    fn core() -> Core<crate::mem::flat::FlatMemory> {
        Core::new(crate::mem::flat::FlatMemory::with_layout(
            &[0x2000_0000],
            &[0x1000],
        ))
    }

    #[test]
    fn cond_codes_against_flags() {
        let mut c = core();
        // N=1, Z=0, C=0, V=1.
        c.xpsr.set_nzcv(true, false, false, true);
        assert!(c.cond_passes(0x4)); // MI
        assert!(!c.cond_passes(0x5)); // PL
        assert!(!c.cond_passes(0x0)); // EQ
        assert!(c.cond_passes(0x1)); // NE
        assert!(!c.cond_passes(0x8)); // HI needs C && !Z
        assert!(c.cond_passes(0x9)); // LS is !C || Z
        assert!(c.cond_passes(0xA)); // GE: N == V
        assert!(!c.cond_passes(0xB)); // LT
    }

    #[test]
    fn sysreg_sp_bank_switch() {
        let mut c = core();
        c.regs.sp = 0x100;
        c.regs.msp = 0x100;
        c.regs.psp = 0x200;
        // CONTROL.SPSEL = 1 -> process SP becomes active.
        c.sysreg_write(SysReg::Control, 0x2).unwrap();
        assert_eq!(c.regs.sp, 0x200);
        assert_eq!(c.regs.msp, 0x100);
        assert_eq!(c.sysreg_read(SysReg::Msp).unwrap(), 0x100);
        assert_eq!(c.sysreg_read(SysReg::Psp).unwrap(), 0x200);
        // Writing PSP while process SP is active updates the live pointer.
        c.sysreg_write(SysReg::Psp, 0x300).unwrap();
        assert_eq!(c.regs.sp, 0x300);
        // Back to main SP; the process pointer is preserved.
        c.sysreg_write(SysReg::Control, 0x0).unwrap();
        assert_eq!(c.regs.sp, 0x100);
        assert_eq!(c.sysreg_read(SysReg::Psp).unwrap(), 0x300);
    }

    #[test]
    fn sysreg_primask_and_control() {
        let mut c = core();
        c.sysreg_write(SysReg::Primask, 1).unwrap();
        assert!(c.primask);
        assert_eq!(c.sysreg_read(SysReg::Primask).unwrap(), 1);
        assert_eq!(c.sysreg_read(SysReg::Control).unwrap(), 0);
        // MSR CONTROL preserves the nPRIV/FPCA bits we model.
        c.sysreg_write(SysReg::Control, 0x5).unwrap();
        assert_eq!(c.sysreg_read(SysReg::Control).unwrap(), 0x5);
        // Reserved sysreg reads/writes are typed errors, not silent.
        assert!(matches!(
            c.sysreg_read(SysReg::Other(0x7f)),
            Err(HaltCause::NotImplemented { .. })
        ));
        assert!(matches!(
            c.sysreg_write(SysReg::Other(0x7f), 0),
            Err(HaltCause::NotImplemented { .. })
        ));
    }
}
