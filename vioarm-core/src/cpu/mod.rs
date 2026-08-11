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
use decode16::{decode16, decode16_v7m, AluOp, ExtendKind, Inst16, LsImm, LsReg, ReverseKind, Shift, ShiftImm};
use decode32::{
    decode32, decode32_v7m, is_thumb2, is_thumb2_v7m, thumb_expand_imm, BitfieldOp, DpOp, Inst32,
    MemOp, MulOp, ShiftType, SysReg, UnaryOp,
};
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
    /// ARMv7-M freely intermixes 16-bit and 32-bit instructions at any even
    /// address, so a 32-bit instruction can start on the second halfword of an
    /// aligned word (`pc & 2 == 2`). The two halfwords are therefore read
    /// from `pc` and `pc + 2` directly; the parallel aligned-word fetch would
    /// mispair the halves for that case.
    fn step_v7m(&mut self, pc: u32) -> Result<Next, HaltCause> {
        if pc & 1 != 0 {
            return Err(HaltCause::FetchAbort);
        }
        let h0 = self.mem.read_half(pc).map_err(|_| HaltCause::FetchAbort)?;
        let h1 = self
            .mem
            .read_half(pc.wrapping_add(2))
            .map_err(|_| HaltCause::FetchAbort)?;
        if is_thumb2_v7m(h0) {
            match decode32_v7m(h0, h1) {
                Ok(inst) => self.exec_it_step(pc, 4, |s| s.exec32(pc, inst)),
                Err(_) => self.hardfault_on_undefined(pc, 4),
            }
        } else {
            match decode16_v7m(h0) {
                Ok(inst) => self.exec_it_step(pc, 2, |s| s.exec(pc, inst)),
                Err(_) => self.hardfault_on_undefined(pc, 2),
            }
        }
    }

    /// ARMv7-M conditional-execution wrapper.
    ///
    /// When `itstate` is nonzero the current instruction sits inside an IT
    /// block. Its condition is `itstate >> 4`; if it fails the instruction is
    /// suppressed (no side effects) but ITSTATE still advances and the PC
    /// moves on by `step`. When it passes, `exec` runs and ITSTATE advances
    /// on a normal fall-through. A taken branch clears ITSTATE in its own arm
    /// and skips the advance here, and an exception entry/stop discards it.
    fn exec_it_step(
        &mut self,
        pc: u32,
        step: u32,
        exec: impl FnOnce(&mut Self) -> Result<Next, HaltCause>,
    ) -> Result<Next, HaltCause> {
        if self.itstate == 0 {
            return exec(self);
        }
        let cond = self.itstate >> 4;
        if !self.cond_passes(cond) {
            self.it_advance();
            return Ok(Next::Advance(pc + step));
        }
        let next = exec(self)?;
        match next {
            Next::Advance(a) if a == pc + step => {
                self.it_advance();
                Ok(next)
            }
            // Branch taken or exception/halt: the branch path clears ITSTATE,
            // exception entry clears it, and stops discard it.
            _ => Ok(next),
        }
    }

    /// Advance the ITSTATE to the next conditional instruction, per the
    /// architecture's `ITAdvance()`: if the block has one instruction left
    /// (`ITSTATE<2:0> == 000`) it clears; otherwise the low five bits shift
    /// left so the next instruction's condition bit lands in `ITSTATE<4>`.
    fn it_advance(&mut self) {
        if self.itstate.trailing_zeros() >= 3 {
            self.itstate = 0;
        } else {
            self.itstate = (self.itstate & 0xE0) | (((self.itstate & 0x1F) << 1) & 0x1F);
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
            SysReg::Xpsr => {
                // xPSR also carries ITSTATE at bits [15:10] and [26:25].
                let mut v = self.xpsr.get() & (APSR_FLAGS | XPSR_T);
                let it = u32::from(self.itstate);
                v |= (it & 0xFC) << 8;
                v |= (it & 0x03) << 25;
                v
            }
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
                self.write_shift_flags_it(res, c);
                self.regs.write(rd, res);
                Ok(Next::Advance(pc + 2))
            }
            I::ShiftReg { rd, rn, rm, kind } => {
                let v = self.regs.read(rn);
                let count = self.regs.read(rm) & 0xFF;
                let (res, c) = shift_reg(kind, v, count);
                self.write_shift_flags_it(res, c);
                self.regs.write(rd, res);
                Ok(Next::Advance(pc + 2))
            }
            I::AddReg { rd, rn, rm } => {
                let (res, c, v) = add_flags(self.regs.read(rn), self.regs.read(rm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubReg { rd, rn, rm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), self.regs.read(rm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::AddImm3 { rd, rn, imm } => {
                let (res, c, v) = add_flags(self.regs.read(rn), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubImm3 { rd, rn, imm } => {
                let (res, c, v) = sub_flags(self.regs.read(rn), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::MovImm { rd, imm } => {
                let res = u32::from(imm);
                self.regs.write(rd, res);
                self.set_nz_it(res, false);
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
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::SubImm8 { rd, imm } => {
                let (res, c, v) = sub_flags(self.regs.read(rd), u32::from(imm), false);
                self.regs.write(rd, res);
                self.set_nzcv_it(res, c, v);
                Ok(Next::Advance(pc + 2))
            }
            I::Alu { rd, rm, op } => {
                self.exec_alu(rd, rm, op, self.itstate == 0);
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
                self.itstate = 0;
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
                    self.itstate = 0;
                    Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
                } else {
                    Ok(Next::Advance(pc + 2))
                }
            }
            I::Branch { off } => {
                self.itstate = 0;
                Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
            }
            I::CbzCbnz { rn, off, nonzero } => {
                let is_zero = self.regs.read(rn) == 0;
                let taken = if nonzero { !is_zero } else { is_zero };
                if taken {
                    self.itstate = 0;
                    Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
                } else {
                    Ok(Next::Advance(pc + 2))
                }
            }
            I::Svc { .. } => {
                // ARMv7-M: a 16-bit SVC takes the SVCall exception; the
                // stacked return address is the instruction after the SVC so
                // the handler can continue with `bx lr`. The immediate is
                // service data for software only.
                self.take_exception(Exception::SvCall, pc + 2)?;
                Ok(Next::Advance(self.regs.pc))
            }
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
            I::It { cond, mask } => {
                // ITSTATE = firstcond:mask. `exec_it_step` never gates this
                // instruction itself because an IT inside an IT block is
                // UNPREDICTABLE; the fresh value simply replaces any stale
                // state.
                self.itstate = (cond << 4) | mask;
                Ok(Next::Advance(pc + 2))
            }
            I::Extend { rd, rm, kind } => {
                let v = self.regs.read(rm);
                // The signed forms sign-extend the selected low field; the
                // unsigned forms zero it. No flags are affected.
                let result = match kind {
                    ExtendKind::Sxth => ((v & 0xFFFF) as i16) as i32 as u32,
                    ExtendKind::Sxtb => ((v & 0xFF) as i8) as i32 as u32,
                    ExtendKind::Uxth => v & 0xFFFF,
                    ExtendKind::Uxtb => v & 0xFF,
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 2))
            }
            I::Reverse { rd, rm, kind } => {
                let v = self.regs.read(rm);
                // `REV` flips the whole word's byte order, `REV16` flips each
                // halfword, and `REVSH` flips the low halfword's bytes before
                // sign-extending it to 32 bits. No flags are affected.
                let result = match kind {
                    ReverseKind::Rev => v.swap_bytes(),
                    ReverseKind::Rev16 => ((v & 0x00FF_00FF) << 8) | ((v >> 8) & 0x00FF_00FF),
                    ReverseKind::Revsh => {
                        let half = ((v & 0xFF) << 8) | ((v >> 8) & 0xFF);
                        if half & 0x8000 != 0 {
                            half | 0xFFFF_0000
                        } else {
                            half
                        }
                    }
                };
                self.regs.write(rd, result);
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
                self.itstate = 0;
                self.regs.lr = (pc.wrapping_add(4)) | 1;
                Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
            }
            I::BlxImm { off } => {
                self.itstate = 0;
                self.regs.lr = (pc.wrapping_add(4)) | 1;
                // BLX targets are word-aligned, so bit 0 is always clear and
                // `check_branch` stops at the non-Thumb (ARM) target.
                Ok(Self::check_branch(pc.wrapping_add(4).wrapping_add_signed(off)))
            }
            I::BranchWide { off } => {
                self.itstate = 0;
                Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
            }
            I::BranchCondWide { cond, off } => {
                if self.cond_passes(cond) {
                    self.itstate = 0;
                    Ok(Next::Advance(pc.wrapping_add(4).wrapping_add_signed(off)))
                } else {
                    Ok(Next::Advance(pc + 4))
                }
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
            I::DpReg {
                op,
                rd,
                rn,
                rm,
                shift,
                amount,
                set_flags,
            } => {
                let rm_v = self.regs.read(rm);
                let (op2, shift_carry) = match (shift, amount) {
                    (ShiftType::Ror, 0) => {
                        // RRX: rotate one bit right through the old carry.
                        let carry_out = rm_v & 1 == 1;
                        let rotated = (rm_v >> 1) | (u32::from(self.xpsr.c()) << 31);
                        (rotated, Some(carry_out))
                    }
                    (ShiftType::Ror, n) => {
                        let carry = (rm_v >> (n - 1)) & 1 == 1;
                        (rm_v.rotate_right(u32::from(n)), Some(carry))
                    }
                    (kind, n) => {
                        let kind = match kind {
                            ShiftType::Lsl => ShiftImm::Lsl,
                            ShiftType::Lsr => ShiftImm::Lsr,
                            ShiftType::Asr => ShiftImm::Asr,
                            ShiftType::Ror => unreachable!(),
                        };
                        shift_imm(kind, rm_v, u32::from(n))
                    }
                };
                self.exec_dp32(op, rd, rn, op2, set_flags, shift_carry)?;
                Ok(Next::Advance(pc + 4))
            }
            I::DpImm {
                op,
                rd,
                rn,
                imm12,
                expand,
                set_flags,
            } => {
                let (op2, expand_carry) = if expand {
                    let imm32 = thumb_expand_imm(imm12);
                    // A flag-setting logical op folds bit31 of a *rotated*
                    // modified immediate into C; an unrotated one (imm8 placed
                    // directly, i:imm3 == 0) leaves C alone.
                    let imm_c = imm12 & 0xC00 != 0;
                    (imm32, imm_c.then_some(imm32 >> 31 & 1 != 0))
                } else {
                    (u32::from(imm12), None)
                };
                self.exec_dp32(op, rd, rn, op2, set_flags, expand_carry)?;
                Ok(Next::Advance(pc + 4))
            }
            I::MovWide { rd, imm, high } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "movw/movt to pc",
                    });
                }
                let cur = self.regs.read(rd);
                let new = if high {
                    (cur & 0xFFFF) | (u32::from(imm) << 16)
                } else {
                    (cur & 0xFFFF_0000) | u32::from(imm)
                };
                self.regs.write(rd, new);
                Ok(Next::Advance(pc + 4))
            }
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
            } => {
                // Rn == 15 is the literal form: the word-aligned `pc + 4`.
                let base = if rn == 15 {
                    (pc.wrapping_add(4)) & !3
                } else {
                    self.regs.read(rn)
                };
                // A register offset adds `Rv(Rm) << shift`; the immediate
                // forms add or subtract `off`.
                let offset_addr = if let Some(rm) = rm {
                    base.wrapping_add(self.regs.read(rm) << shift)
                } else if add {
                    base.wrapping_add(off)
                } else {
                    base.wrapping_sub(off)
                };
                // Post-indexed forms access the base and then step it; the
                // other two access the stepped address.
                let address = if index { offset_addr } else { base };
                let next = self.exec_ls_at(address, rt, access.into(), pc, 4)?;
                if writeback {
                    self.regs.write(rn, offset_addr);
                }
                Ok(next)
            }
            I::LdmStm {
                is_load,
                rn,
                regs,
                writeback,
                dec_before,
            } => {
                let base = self.regs.read(rn);
                let words = regs.count_ones();
                // DB stores the lowest-numbered register at `base - 4n`; IA
                // at `base`. Either way registers land at ascending addresses.
                let wb_addr = if dec_before {
                    base.wrapping_sub(4 * words)
                } else {
                    base.wrapping_add(4 * words)
                };
                let mut addr = if dec_before { wb_addr } else { base };
                if is_load {
                    let mut pc_target = None;
                    for r in 0..16 {
                        if regs & (1u16 << r) != 0 {
                            let v = self.read_word_halt(addr)?;
                            if r == 15 {
                                pc_target = Some(v);
                            } else {
                                self.regs.write(r, v);
                            }
                            addr = addr.wrapping_add(4);
                        }
                    }
                    // A loaded rn wins over the write-back address.
                    if writeback && regs & (1u16 << rn) == 0 {
                        self.regs.write(rn, wb_addr);
                    }
                    if let Some(target) = pc_target {
                        return self.branch(target);
                    }
                } else {
                    for r in 0..16 {
                        if regs & (1u16 << r) != 0 {
                            let v = self.regs.read(r);
                            self.write_word_halt(addr, v)?;
                            addr = addr.wrapping_add(4);
                        }
                    }
                    if writeback {
                        self.regs.write(rn, wb_addr);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
            I::LdrdStrd {
                is_load,
                rt,
                rt2,
                rn,
                imm,
                add,
                index,
                writeback,
            } => {
                // `Rn == 15` is the `LDRD` literal form: the word-aligned
                // `pc + 4`. `index` selects the offset- (pre-) indexed
                // address over the post-indexed base.
                let base = if rn == 15 {
                    (pc.wrapping_add(4)) & !3
                } else {
                    self.regs.read(rn)
                };
                let offset_addr = if add {
                    base.wrapping_add(imm)
                } else {
                    base.wrapping_sub(imm)
                };
                let address = if index { offset_addr } else { base };
                if is_load {
                    // The low word lands in `rt`, the high word in `rt2`;
                    // the decoder guarantees they are distinct registers.
                    let first = self.read_word_halt(address)?;
                    let second = self.read_word_halt(address.wrapping_add(4))?;
                    self.regs.write(rt, first);
                    self.regs.write(rt2, second);
                } else {
                    self.write_word_halt(address, self.regs.read(rt))?;
                    self.write_word_halt(address.wrapping_add(4), self.regs.read(rt2))?;
                }
                if writeback {
                    self.regs.write(rn, offset_addr);
                }
                Ok(Next::Advance(pc + 4))
            }
            I::TblBranch { rn, rm, half } => {
                // index = Mem[Rn + Rm (or Rn + Rm<<1) as byte/halfword];
                // target = (pc + 4) + 2 * index, a simple Thumb branch. A PC
                // base reads the address of the following instruction.
                let base = if rn == 15 {
                    pc.wrapping_add(4)
                } else {
                    self.regs.read(rn)
                };
                let index_reg = self.regs.read(rm);
                let table_addr = if half {
                    base.wrapping_add(index_reg << 1)
                } else {
                    base.wrapping_add(index_reg)
                };
                let index: u32 =
                    if half {
                        u32::from(self.mem.read_half(table_addr).map_err(|_| {
                            HaltCause::BusFault {
                                address: table_addr,
                            }
                        })?)
                    } else {
                        u32::from(self.mem.read(table_addr).map_err(|_| HaltCause::BusFault {
                            address: table_addr,
                        })?)
                    };
                let target = pc.wrapping_add(4).wrapping_add(index << 1);
                self.itstate = 0;
                Ok(Next::Advance(target & !1))
            }
            #[allow(clippy::cast_sign_loss, clippy::cast_possible_wrap)]
            I::Div { signed, rd, rn, rm } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "divide writes to pc",
                    });
                }
                // A zero divisor yields zero (no fault; `DIV_0_TRP` is off at
                // reset and not modeled). Signed `MIN / -1` saturates to MIN
                // without trapping, exactly as the architecture requires.
                let num = self.regs.read(rn);
                let den = self.regs.read(rm);
                let result = if den == 0 {
                    0
                } else if signed {
                    i32::wrapping_div(num as i32, den as i32) as u32
                } else {
                    u32::wrapping_div(num, den)
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::Mul { op, rd, rn, rm, ra } => {
                match op {
                    MulOp::Mul | MulOp::Mla | MulOp::Mls => {
                        if rd == 15 {
                            return Err(HaltCause::NotImplemented {
                                what: "multiply writes to pc",
                            });
                        }
                        // The 32-bit multiply leaves the low word of the
                        // product; there is no flag setting on ARMv7-M.
                        let n = self.regs.read(rn);
                        let m = self.regs.read(rm);
                        let product = n.wrapping_mul(m);
                        let result = match op {
                            MulOp::Mul => product,
                            MulOp::Mla => product.wrapping_add(self.regs.read(ra)),
                            MulOp::Mls => self.regs.read(ra).wrapping_sub(product),
                            MulOp::Smull | MulOp::Umull | MulOp::Smlal | MulOp::Umlal => {
                                unreachable!()
                            }
                        };
                        self.regs.write(rd, result);
                    }
                    MulOp::Smull | MulOp::Umull | MulOp::Smlal | MulOp::Umlal => {
                        if rd == 15 || ra == 15 {
                            return Err(HaltCause::NotImplemented {
                                what: "long multiply writes to pc",
                            });
                        }
                        let n = self.regs.read(rn);
                        let m = self.regs.read(rm);
                        let product = if matches!(op, MulOp::Smull | MulOp::Smlal) {
                            i64::from(n as i32).wrapping_mul(i64::from(m as i32))
                        } else {
                            i64::from(n).wrapping_mul(i64::from(m))
                        };
                        let accumulator = if matches!(op, MulOp::Smlal | MulOp::Umlal) {
                            (i64::from(self.regs.read(ra)) << 32) | i64::from(self.regs.read(rd))
                        } else {
                            0
                        };
                        let total = product.wrapping_add(accumulator);
                        let lo = total as u32;
                        let hi = (total >> 32) as u32;
                        self.regs.write(rd, lo);
                        self.regs.write(ra, hi);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
            I::Unary { op, rd, rm } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "unary data-processing writes to pc",
                    });
                }
                let v = self.regs.read(rm);
                let result = match op {
                    UnaryOp::Rev => v.swap_bytes(),
                    UnaryOp::Rev16 => ((v & 0x00FF_00FF) << 8) | ((v >> 8) & 0x00FF_00FF),
                    UnaryOp::Revsh => {
                        let half = ((v & 0xFF) << 8) | ((v >> 8) & 0xFF);
                        if half & 0x8000 != 0 {
                            half | 0xFFFF_0000
                        } else {
                            half
                        }
                    }
                    UnaryOp::Rbit => rbit(v),
                    UnaryOp::Clz => v.leading_zeros(),
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            I::Bitfield {
                op,
                rd,
                rn,
                lsb,
                field,
            } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "bitfield writes to pc",
                    });
                }
                match op {
                    BitfieldOp::Sbfx | BitfieldOp::Ubfx => {
                        // `width = field + 1` bits are shifted down from
                        // `lsb` and sign- or zero-extended. The decoder
                        // guarantees `lsb + field <= 31`.
                        let width = u32::from(field) + 1;
                        let mask = if width == 32 {
                            u32::MAX
                        } else {
                            (1u32 << width) - 1
                        };
                        let extracted = self.regs.read(rn) >> u32::from(lsb) & mask;
                        let result =
                            if op == BitfieldOp::Sbfx && extracted & (1u32 << (width - 1)) != 0 {
                                extracted | !mask
                            } else {
                                extracted
                            };
                        self.regs.write(rd, result);
                    }
                    BitfieldOp::Bfi | BitfieldOp::Bfc => {
                        // Deposit `Rn`'s low `width` bits into `Rd[field:lsb]`
                        // (or clear the window, for `BFC`). The decoder
                        // guarantees `field >= lsb`.
                        let width = u32::from(field) - u32::from(lsb) + 1;
                        let mask = if width == 32 {
                            u32::MAX
                        } else {
                            ((1u32 << width) - 1) << u32::from(lsb)
                        };
                        let base = self.regs.read(rd) & !mask;
                        let inserted = if op == BitfieldOp::Bfc {
                            0
                        } else {
                            self.regs.read(rn) << u32::from(lsb) & mask
                        };
                        self.regs.write(rd, base | inserted);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
        }
    }

    /// Execute the common ARMv7-M data-processing core over an already
    /// shifted (`op2`) operand. `shift_carry` is the carry the shifted
    /// register produced; only the flag-setting logical operations consume it
    /// (arithmetic computes `C`/`V` from the operation itself).
    fn exec_dp32(
        &mut self,
        op: DpOp,
        rd: u8,
        rn: u8,
        op2: u32,
        set_flags: bool,
        shift_carry: Option<bool>,
    ) -> Result<(), HaltCause> {
        let writes_rd = !matches!(op, DpOp::Tst | DpOp::Teq | DpOp::Cmp | DpOp::Cmn);
        if writes_rd && rd == 15 {
            return Err(HaltCause::NotImplemented {
                what: "data-processing writes to pc",
            });
        }
        let rn_v = self.regs.read(rn);
        let (res, c, v) = match op {
            DpOp::And | DpOp::Tst => {
                let c = self.shift_carry_or_current(shift_carry);
                (rn_v & op2, c, self.xpsr.v())
            }
            DpOp::Eor | DpOp::Teq => {
                let c = self.shift_carry_or_current(shift_carry);
                (rn_v ^ op2, c, self.xpsr.v())
            }
            DpOp::Bic => {
                let c = self.shift_carry_or_current(shift_carry);
                (rn_v & !op2, c, self.xpsr.v())
            }
            DpOp::Orr => {
                let c = self.shift_carry_or_current(shift_carry);
                (rn_v | op2, c, self.xpsr.v())
            }
            DpOp::Orn => {
                let c = self.shift_carry_or_current(shift_carry);
                (op2 | !rn_v, c, self.xpsr.v())
            }
            DpOp::Mov => {
                let c = self.shift_carry_or_current(shift_carry);
                (op2, c, self.xpsr.v())
            }
            DpOp::Mvn => {
                let c = self.shift_carry_or_current(shift_carry);
                (!op2, c, self.xpsr.v())
            }
            DpOp::Adc => add_flags(rn_v, op2, self.xpsr.c()),
            DpOp::Add | DpOp::Cmn => add_flags(rn_v, op2, false),
            DpOp::Sub | DpOp::Cmp => sub_flags(rn_v, op2, false),
            DpOp::Sbc => sub_flags(rn_v, op2, !self.xpsr.c()),
            DpOp::Rsb => sub_flags(op2, rn_v, false),
        };
        if set_flags {
            self.set_nzcv(res, c, v);
        }
        if writes_rd {
            self.regs.write(rd, res);
        }
        Ok(())
    }

    /// The carry an ARMv7-M data-processing logical op produces, when it sets
    /// flags: the shifted-register carry if there was one, else the old `C`.
    fn shift_carry_or_current(&self, shift_carry: Option<bool>) -> bool {
        shift_carry.unwrap_or_else(|| self.xpsr.c())
    }

    /// Execute a 16-bit ALU (data-processing) instruction. `set_flags` is
    /// false only inside an IT block, where the S-forms stop updating the
    /// flags (CMP/CMN/TST always update them).
    fn exec_alu(&mut self, rd: u8, rm: u8, op: AluOp, set_flags: bool) {
        let a = self.regs.read(rd);
        let b = self.regs.read(rm);
        match op {
            AluOp::And => {
                let res = a & b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
            }
            AluOp::Eor => {
                let res = a ^ b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
            }
            AluOp::Lsl => {
                let (res, c) = shift_reg(Shift::Lsl, a, b & 0xFF);
                self.regs.write(rd, res);
                if set_flags {
                    self.write_shift_flags(res, c);
                }
            }
            AluOp::Lsr => {
                let (res, c) = shift_reg(Shift::Lsr, a, b & 0xFF);
                self.regs.write(rd, res);
                if set_flags {
                    self.write_shift_flags(res, c);
                }
            }
            AluOp::Asr => {
                let (res, c) = shift_reg(Shift::Asr, a, b & 0xFF);
                self.regs.write(rd, res);
                if set_flags {
                    self.write_shift_flags(res, c);
                }
            }
            AluOp::Adc => {
                let (res, c, v) = add_flags(a, b, self.xpsr.c());
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nzcv(res, c, v);
                }
            }
            AluOp::Sbc => {
                let (res, c, v) = sub_flags(a, b, !self.xpsr.c());
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nzcv(res, c, v);
                }
            }
            AluOp::Ror => {
                let (res, c) = shift_reg(Shift::Ror, a, b & 0xFF);
                self.regs.write(rd, res);
                if set_flags {
                    self.write_shift_flags(res, c);
                }
            }
            AluOp::Tst => {
                let res = a & b;
                self.set_nz(res, false);
            }
            AluOp::Neg => {
                let (res, c, v) = sub_flags(0, a, false);
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nzcv(res, c, v);
                }
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
                if set_flags {
                    self.set_nz(res, false);
                }
            }
            AluOp::Mul => {
                let res = a.wrapping_mul(b);
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nzcv(res, false, false);
                }
            }
            AluOp::Bic => {
                let res = a & !b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
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
        self.exec_ls_at(addr, rt, access, pc, 2)
    }

    /// `exec_ls`, but the fall-through (and the `rt == 15` branch's `LR`
    /// base) advance by `step` instead of 2, for the 32-bit load/stores.
    fn exec_ls_at(
        &mut self,
        addr: u32,
        rt: u8,
        access: LsAccess,
        pc: u32,
        step: u32,
    ) -> Result<Next, HaltCause> {
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
            self.write_rt_at(rt, v, addr, pc, step)
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
            Ok(Next::Advance(pc + step))
        }
    }

    /// Write a loaded value to `rt`. If `rt == 15` the load is a branch
    /// (Thumb bit checked, EXC_RETURN honored).
    fn write_rt(&mut self, rt: u8, v: u32, addr: u32, pc: u32) -> Result<Next, HaltCause> {
        self.write_rt_at(rt, v, addr, pc, 2)
    }

    /// `write_rt` with a 32-bit fall-through step.
    fn write_rt_at(
        &mut self,
        rt: u8,
        v: u32,
        addr: u32,
        pc: u32,
        step: u32,
    ) -> Result<Next, HaltCause> {
        let _ = addr;
        if rt == 15 {
            return self.branch(v);
        }
        self.regs.write(rt, v);
        Ok(Next::Advance(pc + step))
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
    ///
    /// Any taken branch leaves the current IT block, so ITSTATE is cleared
    /// (the exception-return path overwrites it from the stacked xPSR).
    fn branch(&mut self, target: u32) -> Result<Next, HaltCause> {
        self.itstate = 0;
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
        // ITSTATE occupies xPSR bits [15:10] (IT[7:2]) and [26:25] (IT[1:0]).
        // It is captured in the frame and cleared for the handler.
        let it = u32::from(self.itstate);
        sxpsr |= (it & 0xFC) << 8;
        sxpsr |= (it & 0x03) << 25;
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
        self.itstate = 0;
        Ok(())
    }

    /// Enter Handler mode for exception number `n`, as decided by the
    /// machine's interrupt controller. The return address is the instruction
    /// the core is currently pointing at (the next one to execute on return).
    pub fn enter_exception(&mut self, n: u8) -> Result<(), HaltCause> {
        let ex = match n {
            3 => Exception::HardFault,
            11 => Exception::SvCall,
            14 => Exception::PendSV,
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
        // Restore ITSTATE from its xPSR[15:10] / [26:25] slots.
        self.itstate =
            ((sxpsr >> 8) & 0xFC) as u8 | ((sxpsr >> 25) & 0x03) as u8;
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

    /// `set_nzcv` gated on IT-block state: the 16-bit data-processing
    /// instructions that normally set flags do not do so when they execute
    /// inside an IT block (CMP, CMN and TST are exempt and keep using the
    /// ungated writers). The gate works because `itstate` is still nonzero
    /// for the *whole* of an IT-block instruction's execution.
    fn set_nzcv_it(&mut self, res: u32, c: bool, v: bool) {
        if self.itstate == 0 {
            self.set_nzcv(res, c, v);
        }
    }

    /// `set_nz` gated on IT-block state, see `set_nzcv_it`.
    fn set_nz_it(&mut self, res: u32, v: bool) {
        if self.itstate == 0 {
            self.set_nz(res, v);
        }
    }

    /// `write_shift_flags` gated on IT-block state, see `set_nzcv_it`.
    fn write_shift_flags_it(&mut self, res: u32, c: Option<bool>) {
        if self.itstate == 0 {
            self.write_shift_flags(res, c);
        }
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

impl From<MemOp> for LsAccess {
    fn from(m: MemOp) -> Self {
        match m {
            MemOp::Str => Self::Word { load: false },
            MemOp::Ldr => Self::Word { load: true },
            MemOp::Strb => Self::Byte { load: false },
            MemOp::Ldrb => Self::Byte { load: true },
            MemOp::Strh => Self::Half { load: false },
            MemOp::Ldrh => Self::Half { load: true },
            MemOp::Ldrsb => Self::SextByte,
            MemOp::Ldrsh => Self::SextHalf,
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

/// Reverse the 32 bits of `v` (`RBIT`).
///
/// Swap the bytes (reversing the byte order *and* dropping each byte into the
/// mirror position), then finish reversing the bits within each byte so the
/// whole word is mirrored end to end.
#[must_use]
const fn rbit(v: u32) -> u32 {
    let v = v.swap_bytes();
    let v = (v & 0x0F0F_0F0F) << 4 | (v & 0xF0F0_F0F0) >> 4;
    let v = (v & 0x3333_3333) << 2 | (v & 0xCCCC_CCCC) >> 2;
    (v & 0x5555_5555) << 1 | (v & 0xAAAA_AAAA) >> 1
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
    use crate::mem::bus::Bus;

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

    fn armv7m_core() -> Core<crate::mem::flat::FlatMemory> {
        Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x2000_0000],
            &[0x1000],
        ))
    }

    /// Place a byte sequence (an instruction stream) at `addr`.
    fn load_inst(c: &mut Core<crate::mem::flat::FlatMemory>, addr: u32, bytes: &[u8]) {
        for (i, b) in bytes.iter().enumerate() {
            c.mem().write(addr + i as u32, *b).unwrap();
        }
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

    #[test]
    fn svc_takes_svcall_exception_and_returns() {
        const SVC_ADDR: u32 = 0x2000_0100;
        const HANDLER: u32 = 0x2000_0400;
        const SP: u32 = 0x2000_0800;
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        // SVCall is exception 11: its vector sits at 0x2c in the vector table.
        c.mem().write_word(0x2c, HANDLER | 1).unwrap();
        c.regs.sp = SP;
        c.regs.msp = SP;
        load_inst(&mut c, SVC_ADDR, &[0x99, 0xdf]); // svc #153
        load_inst(&mut c, HANDLER, &[0x70, 0x47]); // bx lr
        c.regs.pc = SVC_ADDR;
        c.step().unwrap();
        // The SVCall handler is entered: ipsr = 11, pc = handler, lr holds the
        // EXC_RETURN through which the handler returns.
        assert_eq!(c.ipsr, 11);
        assert_eq!(c.regs.pc, HANDLER);
        assert_eq!(c.regs.lr, EXC_RETURN_THREAD_MSP);
        // The task frame lies on the main SP: the stacked PC is one halfword
        // past the SVC, and the stacked xPSR carries Thumb with the
        // pre-exception IPSR (0 in Thread mode).
        assert_eq!(c.regs.sp, SP - FRAME_BYTES);
        assert_eq!(
            c.mem().read_word(SP - FRAME_BYTES + 24).unwrap(),
            (SVC_ADDR + 2) | 1
        );
        assert_eq!(c.mem().read_word(SP - FRAME_BYTES + 28).unwrap(), XPSR_T);
        c.step().unwrap();
        // `bx lr` performs the exception return: thread resumes at the
        // instruction after the SVC with its SP intact.
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.regs.pc, SVC_ADDR + 2);
        assert_eq!(c.regs.sp, SP);
    }

    #[test]
    fn mem_imm_word_roundtrip() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xCAFE_BABE);
        c.regs.write(2, 0x2000_0400);
        // str.w r1, [r2, #4] f8c2 1004; ldr.w r3, [r2, #4] f8d2 3004.
        load_inst(&mut c, 0x2000_0000, &[0xc2, 0xf8, 0x04, 0x10]);
        load_inst(&mut c, 0x2000_0004, &[0xd2, 0xf8, 0x04, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert_eq!(c.mem().read_word(0x2000_0404).unwrap(), 0xCAFE_BABE);
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0xCAFE_BABE);
        assert_eq!(c.regs.pc, 0x2000_0008);
    }

    #[test]
    fn mem_imm_byte_half_store() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xAABB_CCDD);
        c.regs.write(1, 0x1122_3344);
        c.regs.write(2, 0x2000_0400);
        // strb.w r0, [r2, #0] f882 0000; strh.w r1, [r2, #2] f8a2 1002.
        load_inst(&mut c, 0x2000_0000, &[0x82, 0xf8, 0x00, 0x00]);
        load_inst(&mut c, 0x2000_0004, &[0xa2, 0xf8, 0x02, 0x10]);
        c.step().unwrap();
        assert_eq!(c.mem().read(0x2000_0400).unwrap(), 0xDD);
        c.step().unwrap();
        assert_eq!(c.mem().read(0x2000_0402).unwrap(), 0x44);
        assert_eq!(c.mem().read(0x2000_0403).unwrap(), 0x33);
    }

    #[test]
    fn mem_imm_signed_loads() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.mem().write(0x2000_0410, 0x80).unwrap();
        c.mem().write_half(0x2000_0420, 0x8000).unwrap();
        // ldrsb.w r0, [r2, #0x10] f992 0010; ldrsh.w r1, [r2, #0x20] f9b2 1020.
        load_inst(&mut c, 0x2000_0000, &[0x92, 0xf9, 0x10, 0x00]);
        load_inst(&mut c, 0x2000_0004, &[0xb2, 0xf9, 0x20, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0xFFFF_FF80);
        c.step().unwrap();
        assert_eq!(c.regs.read(1), 0xFFFF_8000);
    }

    #[test]
    fn mem_imm_pre_post_index() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xDEAD_0000);
        c.regs.write(2, 0x2000_0400);
        // str.w r0, [r2, #8]! f842 0f08 (pre-indexed write-back).
        load_inst(&mut c, 0x2000_0000, &[0x42, 0xf8, 0x08, 0x0f]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_0408).unwrap(), 0xDEAD_0000);
        assert_eq!(c.regs.read(2), 0x2000_0408);
        assert_eq!(c.regs.pc, 0x2000_0004);
        // ldr.w r3, [r2], #8 f852 3b08 (post-indexed write-back).
        load_inst(&mut c, 0x2000_0004, &[0x52, 0xf8, 0x08, 0x3b]);
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0xDEAD_0000);
        assert_eq!(c.regs.read(2), 0x2000_0410);
        assert_eq!(c.regs.pc, 0x2000_0008);
    }

    #[test]
    fn mem_reg_offset_roundtrip() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.regs.write(3, 0x10);
        c.regs.write(4, 0xDEAD_BEEF);
        // str.w r4, [r2, r3] f842 4003: offset Rm = 0x10, no write-back.
        load_inst(&mut c, 0x2000_0000, &[0x42, 0xf8, 0x03, 0x40]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_0410).unwrap(), 0xDEAD_BEEF);
        assert_eq!(c.regs.read(2), 0x2000_0400);
        assert_eq!(c.regs.pc, 0x2000_0004);
        // ldr.w r5, [r2, r3, lsl #2] f852 5013: offset Rm << 2 = 0x40.
        load_inst(&mut c, 0x2000_0004, &[0x52, 0xf8, 0x13, 0x50]);
        c.mem().write_word(0x2000_0440, 0x1234_ABCD).unwrap();
        c.step().unwrap();
        assert_eq!(c.regs.read(5), 0x1234_ABCD);
        assert_eq!(c.regs.read(2), 0x2000_0400);
        assert_eq!(c.regs.pc, 0x2000_0008);
    }

    #[test]
    fn mem_ldm_stm_push_pop() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_1000;
        c.regs.write(0, 0x1111_1111);
        c.regs.write(1, 0x2222_2222);
        c.regs.write(2, 0x3333_3333);
        c.regs.lr = 0xFFFF_FFF5;
        // push.w {r0-r2, lr} e92d 4007: STMDB sp!, {r0-r2, lr}.
        load_inst(&mut c, 0x2000_0000, &[0x2d, 0xe9, 0x07, 0x40]);
        c.step().unwrap();
        assert_eq!(c.regs.sp, 0x2000_0FF0);
        assert_eq!(c.mem().read_word(0x2000_0FF0).unwrap(), 0x1111_1111);
        assert_eq!(c.mem().read_word(0x2000_0FF4).unwrap(), 0x2222_2222);
        assert_eq!(c.mem().read_word(0x2000_0FF8).unwrap(), 0x3333_3333);
        assert_eq!(c.mem().read_word(0x2000_0FFC).unwrap(), 0xFFFF_FFF5);
    }

    #[test]
    fn mem_ldm_stm_load_writeback_and_pc() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0004;
        c.regs.sp = 0x2000_0800;
        c.mem().write_word(0x2000_0800, 0xCAFE_F00D).unwrap();
        c.mem().write_word(0x2000_0804, 0x2000_0008 | 1).unwrap();
        // pop.w {r0, pc} e8bd 8001: the loaded pc (with its Thumb bit) wins.
        load_inst(&mut c, 0x2000_0004, &[0xbd, 0xe8, 0x01, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0xCAFE_F00D);
        assert_eq!(c.regs.sp, 0x2000_0808);
        assert_eq!(c.regs.pc, 0x2000_0008);

        // ldmdb r2!, {r3, r4}: rn loaded last, rn in the list is the lowest.
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.mem().write_word(0x2000_03F8, 0xAAA0).unwrap();
        c.mem().write_word(0x2000_03FC, 0xBBB0).unwrap();
        load_inst(&mut c, 0x2000_0000, &[0x32, 0xe9, 0x18, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0xAAA0);
        assert_eq!(c.regs.read(4), 0xBBB0);
        assert_eq!(c.regs.read(2), 0x2000_03F8);
    }

    #[test]
    fn mem_ldm_stm_ia_no_writeback() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x2000_0200);
        c.regs.write(1, 0xC0FF_EE00);
        c.regs.write(2, 0x00FF_00FF);
        // stmia r0, {r1, r2} e880 0006: no write-back.
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xe8, 0x06, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x2000_0200);
        assert_eq!(c.mem().read_word(0x2000_0200).unwrap(), 0xC0FF_EE00);
        assert_eq!(c.mem().read_word(0x2000_0204).unwrap(), 0x00FF_00FF);
    }

    #[test]
    fn mem_imm_pc_literal() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        // ldr.w r4, [pc, #8] f8df 4008: base Align(pc+4) = 0x20000004, so the
        // word is fetched from 0x2000000c.
        load_inst(&mut c, 0x2000_0000, &[0xdf, 0xf8, 0x08, 0x40]);
        c.mem().write_word(0x2000_000c, 0x1234_5678).unwrap();
        c.step().unwrap();
        assert_eq!(c.regs.read(4), 0x1234_5678);
        assert_eq!(c.regs.pc, 0x2000_0004);
    }

    #[test]
    fn mem_imm_load_to_pc_branches() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x2000_0400);
        c.mem().write_word(0x2000_0400, 0x2000_0501).unwrap();
        // ldr.w pc, [r1, #0] f8d1 f000.
        load_inst(&mut c, 0x2000_0000, &[0xd1, 0xf8, 0x00, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0500);
    }

    #[test]
    fn branch_cbz_cbnz() {
        // cbz r0, +6 (b118): r0 == 0 so it branches to pc + 4 + 6.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0);
        load_inst(&mut c, 0x2000_0000, &[0x18, 0xb1]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_000a);

        // cbz r0, +6: r0 != 0 falls through.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 1);
        load_inst(&mut c, 0x2000_0000, &[0x18, 0xb1]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0002);

        // cbnz r3, +8 (b923): r3 != 0 branches; offset 8.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 3);
        load_inst(&mut c, 0x2000_0000, &[0x23, 0xb9]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_000c);
    }

    #[test]
    fn branch_tbb_tbh() {
        // tbb [r0, r1] (e8d0 f001): byte index at r0+r1. pc = 0x2000, so the
        // base pc+4 = 0x2004; index byte = 6 -> target 0x2004 + 12 = 0x2010.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x2000_0200);
        c.regs.write(1, 4);
        c.mem().write(0x2000_0204, 6).unwrap();
        load_inst(&mut c, 0x2000_0000, &[0xd0, 0xe8, 0x01, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0010);

        // tbh [r1, r2] (e8d1 f012): halfword index at r1 + 2*r2. Index = 0x100
        // -> +0x200 = target 0x2000 + 4 + 0x200.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x2000_0200);
        c.regs.write(2, 4);
        c.mem().write_half(0x2000_0208, 0x100).unwrap();
        load_inst(&mut c, 0x2000_0000, &[0xd1, 0xe8, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.pc, 0x2000_0204);
    }

    #[test]
    fn divide_signed_unsigned() {
        // udiv r2, r4, r5 (fbb4 f2f5): 100 / 4 = 25, flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(4, 100);
        c.regs.write(5, 4);
        load_inst(&mut c, 0x2000_0000, &[0xb4, 0xfb, 0xf5, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 25);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // sdiv r1, r0, r3 (fb90 f1f3): -100 / 7 truncates toward zero.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xFFFF_FF9C);
        c.regs.write(3, 7);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfb, 0xf3, 0xf1]);
        c.step().unwrap();
        assert_eq!(c.regs.r[1], 0xFFFF_FFF2);

        // sdiv r0, r0, r1 (fb90 f0f1): MIN / -1 saturates to MIN.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x8000_0000);
        c.regs.write(1, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfb, 0xf1, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);

        // A zero divisor yields zero for both flavors.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(4, 100);
        c.regs.write(5, 0);
        load_inst(&mut c, 0x2000_0000, &[0xb4, 0xfb, 0xf5, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0xFFFF_FF9C);
        c.regs.write(1, 0);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfb, 0xf1, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
    }

    #[test]
    fn multiply_ops() {
        // mul r4, r5, r6 (fb05 f406): 10 * 5 = 50, flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(5, 10);
        c.regs.write(6, 5);
        load_inst(&mut c, 0x2000_0000, &[0x05, 0xfb, 0x06, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 50);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // mla r0, r1, r2, r3 (fb01 3002): 10 * 20 + 100 = 300.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 10);
        c.regs.write(2, 20);
        c.regs.write(3, 100);
        load_inst(&mut c, 0x2000_0000, &[0x01, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 300);

        // mls r5, r6, r7, r8 (fb06 8517): 100 - 7 * 6 = 58.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(6, 7);
        c.regs.write(7, 6);
        c.regs.write(8, 100);
        load_inst(&mut c, 0x2000_0000, &[0x06, 0xfb, 0x17, 0x85]);
        c.step().unwrap();
        assert_eq!(c.regs.r[5], 58);

        // A 32-bit multiply truncates to the low word and writes nothing else.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x8000_0001);
        c.regs.write(6, 2);
        load_inst(&mut c, 0x2000_0000, &[0x05, 0xfb, 0x06, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 2);
    }

    #[test]
    fn long_multiply_ops() {
        // smull r0, r1, r2, r3 (fb82 1003): -7 * 5 = -35 -> r1:r0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0xFFFF_FFF9);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x82, 0xfb, 0x03, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xFFFF_FFDD);
        assert_eq!(c.regs.r[1], 0xFFFF_FFFF);

        // umull r0, r1, r9, r2 (fba9 1002): 0x10000 * 0x10000 = 2^32 -> r1 = 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x0001_0000);
        c.regs.write(2, 0x0001_0000);
        load_inst(&mut c, 0x2000_0000, &[0xa9, 0xfb, 0x02, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        assert_eq!(c.regs.r[1], 1);

        // smlal r2, r1, r3, r4 (fbc3 1204): acc r1:r2 = 0x1_0000_0010,
        // product -5 * 4 = -20 -> total = 0xFFFF_FFFC, r1 cleared.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0xFFFF_FFFB);
        c.regs.write(4, 4);
        c.regs.write(1, 1);
        c.regs.write(2, 16);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0x04, 0x12]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xFFFF_FFFC);
        assert_eq!(c.regs.r[1], 0);

        // umlal r2, r1, r3, r4 (fbe3 1204): acc r1:r2 = 0x2_0000_0064,
        // product 1000 * 10 = 10000 -> total = 0x2_0000_2774.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 1000);
        c.regs.write(4, 10);
        c.regs.write(1, 2);
        c.regs.write(2, 100);
        load_inst(&mut c, 0x2000_0000, &[0xe3, 0xfb, 0x04, 0x12]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x0000_2774);
        assert_eq!(c.regs.r[1], 2);

        // Writing the low result register (a PC) is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 3);
        c.regs.write(3, 4);
        load_inst(&mut c, 0x2000_0000, &[0xa2, 0xfb, 0x03, 0x1f]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn unary_rev_and_flags() {
        // rev r0, r1 (fa80 f081): byte-reverse a word, flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(1, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xfa, 0x81, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7856_3412);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // rev16 r2, r3 (fa80 f293): swap bytes within each halfword.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xfa, 0x93, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x3412_7856);

        // revsh r4, r5 (fa90 f4b5): byte-swap the low half, sign-extend.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_1234);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfa, 0xb5, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 0x0000_3412);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_80FF);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfa, 0xb5, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 0xFFFF_FF80);
    }

    #[test]
    fn unary_rbit_clz() {
        // rbit r6, r7 (fa90 f6a7): mirror all 32 bits.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(7, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfa, 0xa7, 0xf6]);
        c.step().unwrap();
        assert_eq!(c.regs.r[6], 0x1E6A_2C48);

        // clz r8, r9 (fab0 f889).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x000F_0000);
        load_inst(&mut c, 0x2000_0000, &[0xb0, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 12);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x8000_0000);
        load_inst(&mut c, 0x2000_0000, &[0xb0, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 0);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0);
        load_inst(&mut c, 0x2000_0000, &[0xb0, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 32);

        // Writing the destination register 15 is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xfa, 0x80, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn bitfield_extract() {
        // ubfx r0, r1, #4, #8 (f3c1 1007): pull the middle byte out,
        // zero-extended, with the flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(1, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0xc1, 0xf3, 0x07, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x67);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // sbfx r2, r3, #8, #8 (f343 2207): a negative source byte
        // sign-extends.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x0000_80FF);
        load_inst(&mut c, 0x2000_0000, &[0x43, 0xf3, 0x07, 0x22]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xFFFF_FF80);

        // ... while a positive byte stays small.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x0000_1278);
        load_inst(&mut c, 0x2000_0000, &[0x43, 0xf3, 0x07, 0x22]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x12);
    }

    #[test]
    fn bitfield_insert_clear() {
        // bfi r4, r5, #8, #8 (f365 240f): drop r5's low byte into r4's
        // [15:8] window, leaving the rest alone.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(4, 0xFFFF_FFFF);
        c.regs.write(5, 0x12);
        load_inst(&mut c, 0x2000_0000, &[0x65, 0xf3, 0x0f, 0x24]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 0xFFFF_12FF);

        // bfc r6, #8, #8 (f36f 260f): clear the same window.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(6, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0000, &[0x6f, 0xf3, 0x0f, 0x26]);
        c.step().unwrap();
        assert_eq!(c.regs.r[6], 0xFFFF_00FF);

        // Writing the destination register 15 is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x60, 0xf3, 0x0f, 0x2f]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn extend_16bit() {
        // sxth r3, r5 (b22b): sign-extend the low halfword.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(5, 0x0000_FF80);
        load_inst(&mut c, 0x2000_0000, &[0x2b, 0xb2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0xFFFF_FF80);
        assert_eq!(c.regs.pc, 0x2000_0002);
        // The flags from the setup survive untouched.
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // sxtb r3, r5 (b26b): sign-extend the low byte.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_0080);
        load_inst(&mut c, 0x2000_0000, &[0x6b, 0xb2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0xFFFF_FF80);

        // uxth r3, r5 (b2ab): zero the top halfword.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0000, &[0xab, 0xb2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0xFFFF);

        // uxtb r3, r5 (b2eb): zero the top three bytes.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0xeb, 0xb2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0x78);
    }

    #[test]
    fn reverse_16bit() {
        // rev r3, r5 (ba2b): full word byte reversal.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(5, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x2b, 0xba]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0x7856_3412);
        assert_eq!(c.regs.pc, 0x2000_0002);
        // The flags from the setup survive untouched.
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // rev16 r3, r5 (ba6b): byte-reverse each halfword.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x6b, 0xba]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0x3412_7856);

        // revsh r3, r5 (baeb): reverse the low halfword and sign-extend.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_12F0);
        load_inst(&mut c, 0x2000_0000, &[0xeb, 0xba]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0xFFFF_F012);

        // A positive `revsh` keeps the value sign-bit-free.
        let mut c = core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_1234);
        load_inst(&mut c, 0x2000_0000, &[0xeb, 0xba]);
        c.step().unwrap();
        assert_eq!(c.regs.r[3], 0x3412);
    }

    #[test]
    fn it_block_conditional_execution() {
        // movs r0, #1 (Z=0) ; ite eq ; adds r1, r1, #1 (suppressed) ;
        // adds r1, r1, #2 (taken). ARMv7-M only.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x01, 0x20, 0x0c, 0xbf, 0x01, 0x31, 0x02, 0x31],
        );
        for _ in 0..4 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.pc, 0x2000_0008);
        assert_eq!(c.regs.r[0], 1);
        assert_eq!(c.regs.r[1], 2);
        // The in-block 16-bit ADDS forms left `movs`'s flags alone.
        assert!(!c.xpsr.n() && !c.xpsr.z() && !c.xpsr.c() && !c.xpsr.v());
        assert_eq!(c.itstate, 0);
    }

    #[test]
    fn it_block_full_advance() {
        // movs r0, #5 (Z=0) ; ittt ne ; adds r1..#3 (all three arms run).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x05, 0x20, 0x1e, 0xbf, 0x01, 0x31, 0x02, 0x32, 0x03, 0x33],
        );
        for _ in 0..5 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.pc, 0x2000_000a);
        assert_eq!(c.regs.r[1], 1);
        assert_eq!(c.regs.r[2], 2);
        assert_eq!(c.regs.r[3], 3);
        assert_eq!(c.itstate, 0);
    }

    #[test]
    fn it_block_failed_condition_skips() {
        // movs r0, #5 (Z=0) ; ite eq ; adds r1, r1, #1 (skipped) ;
        // adds r1, r1, #7 (taken).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x05, 0x20, 0x0c, 0xbf, 0x01, 0x31, 0x07, 0x31],
        );
        for _ in 0..4 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.pc, 0x2000_0008);
        assert_eq!(c.regs.r[1], 7);
        assert_eq!(c.itstate, 0);
    }

    #[test]
    fn it_block_branch_clears_state() {
        // movs r0, #5 (Z=0) ; itt ne ; adds r1, r1, #1 ; bne +2 (taken,
        // clearing the IT block).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x05, 0x20, 0x1c, 0xbf, 0x01, 0x31, 0x01, 0xd1],
        );
        for _ in 0..4 {
            c.step().unwrap();
        }
        assert_eq!(c.regs.pc, 0x2000_000c);
        assert_eq!(c.regs.r[1], 1);
        assert_eq!(c.itstate, 0);
    }

    #[test]
    fn itstate_saved_in_exception_frame() {
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        c.regs.pc = 0x2000_0000;
        c.regs.sp = 0x2000_0400;
        c.regs.msp = 0x2000_0400;
        c.itstate = 0x0C; // `ite eq`
        c.mem().write_word(11 * 4, 0x2000_0011).unwrap();
        c.take_exception(Exception::SvCall, 0x2000_0002).unwrap();
        assert_eq!(c.itstate, 0);
        // ITSTATE[7:2] sits in the stacked xPSR at [15:10] (here 0x0C << 8).
        let frame = c.regs.sp;
        let saved = c.mem().read_word(frame + 28).unwrap();
        assert_eq!(saved & 0xFC00, 0x0C00);
        // Returning pops the frame and revives the block.
        c.exception_return(EXC_RETURN_THREAD_MSP).unwrap();
        assert_eq!(c.itstate, 0x0C);
    }
}
