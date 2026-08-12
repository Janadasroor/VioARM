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
use decode16::{
    decode16, decode16_v7m, AluOp, ExtendKind, Inst16, LsImm, LsReg, ReverseKind, Shift, ShiftImm,
};
use decode32::{
    decode32, decode32_v7m, is_thumb2, is_thumb2_v7m, thumb_expand_imm, BitfieldOp, DpOp, DspMulOp,
    DualMulOp, ExSize, ExtendAddOp, Inst32, MemOp, MulOp, ParOp, ParSat, QaddOp, SatOp, ShiftType,
    SysReg, UnaryOp,
};
use registers::{Registers, Xpsr, APSR_FLAGS, APSR_GE, APSR_Q, APSR_QFLAGS, XPSR_T};

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
    /// The local exclusive monitor: the address tagged by the last `LDREX*`.
    /// `None` means open; `STREX*` succeeds only while it matches, and any
    /// exception entry or return clears it.
    pub exmon: Option<u32>,
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
            exmon: None,
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
        // In an unprivileged (Thread, nPRIV=1) context only the xPSR subfields
        // and CONTROL are readable; reads of the remaining special-purpose
        // registers return 0 (ARMv7-M ARM; matches QEMU's `v7m_mrs`).
        let unprivileged = self.ipsr == 0 && self.control & 1 != 0;
        let v = match sys {
            // Reads return the whole APSR; the `_g`/`_nzcvqg` qualifiers only
            // constrain the *write* mask.
            SysReg::Apsr | SysReg::ApsrG | SysReg::ApsrNzvCqg => self.xpsr.get() & APSR_QFLAGS,
            SysReg::Xpsr => {
                // xPSR also carries ITSTATE at bits [15:10] and [26:25].
                let mut v = self.xpsr.get() & (APSR_QFLAGS | XPSR_T);
                let it = u32::from(self.itstate);
                v |= (it & 0xFC) << 8;
                v |= (it & 0x03) << 25;
                v
            }
            SysReg::Ipsr => u32::from(self.ipsr),
            SysReg::Epsr => XPSR_T,
            // Unprivileged reads of these special-purpose registers are RAZ.
            SysReg::Msp
            | SysReg::Psp
            | SysReg::Primask
            | SysReg::Basepri
            | SysReg::BasepriMax
            | SysReg::Faultmask
                if unprivileged =>
            {
                0
            }
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
            SysReg::Basepri | SysReg::BasepriMax => u32::from(self.basepri),
            SysReg::Faultmask => u32::from(self.faultmask),
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
                self.xpsr.set_q(v & APSR_Q != 0);
            }
            SysReg::ApsrG => {
                let cur = self.xpsr.get();
                self.xpsr.set((cur & !APSR_GE) | (v & APSR_GE));
            }
            SysReg::ApsrNzvCqg => {
                self.xpsr
                    .set((self.xpsr.get() & !APSR_QFLAGS) | (v & APSR_QFLAGS));
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
            SysReg::Basepri => self.basepri = v as u8,
            SysReg::BasepriMax => {
                // Raises the mask (moves toward higher priority / lower
                // number); writes that would reduce it are ignored, and a
                // zero never re-enables a set mask.
                let new = v as u8;
                if new != 0 && (self.basepri == 0 || new < self.basepri) {
                    self.basepri = new;
                }
            }
            SysReg::Faultmask => self.faultmask = v & 1 != 0,
            SysReg::Control => {
                // MSR CONTROL is only honored in a privileged context: Handler
                // mode, or Thread mode while nPRIV is still clear. Once the
                // thread drops to unprivileged (nPRIV=1), further writes are
                // ignored, so an unprivileged thread cannot re-privilege
                // itself.
                if self.ipsr != 0 || self.control & 1 == 0 {
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

    /// Sign-extend the bottom (`top == false`) or top (`top == true`) halfword
    /// of `v` to a signed 32-bit value.
    const fn half_signed(&self, v: u32, top: bool) -> i32 {
        let hw = if top { (v >> 16) as u16 } else { v as u16 };
        hw as i16 as i32
    }

    /// The two halfword products of `n` and `m` (with `m`'s halves swapped
    /// first when `m_swap`), each as a signed 32-bit value. A 16-by-16 signed
    /// product always fits, so no overflow is possible.
    const fn smul_dual(&self, n: u32, m: u32, m_swap: bool) -> (i32, i32) {
        let ms = if m_swap { (m >> 16) | (m << 16) } else { m };
        let p_lo = self
            .half_signed(n, false)
            .wrapping_mul(self.half_signed(ms, false));
        let p_hi = self
            .half_signed(n, true)
            .wrapping_mul(self.half_signed(ms, true));
        (p_lo, p_hi)
    }

    /// Wrapping signed 32-bit addition that sets the sticky `Q` flag on
    /// overflow, mirroring QEMU's `add_setq` helper.
    fn add_setq(&mut self, a: i32, b: i32) -> i32 {
        let (result, overflow) = a.overflowing_add(b);
        if overflow {
            self.xpsr.set_q(true);
        }
        result
    }

    /// Sum the absolute differences of the four byte lanes of `n` and `m`.
    const fn sum_abs_diff(&self, n: u32, m: u32) -> i32 {
        let d0 = ((n & 0xFF) as i32) - ((m & 0xFF) as i32);
        let d1 = (((n >> 8) & 0xFF) as i32) - (((m >> 8) & 0xFF) as i32);
        let d2 = (((n >> 16) & 0xFF) as i32) - (((m >> 16) & 0xFF) as i32);
        let d3 = (((n >> 24) & 0xFF) as i32) - (((m >> 24) & 0xFF) as i32);
        d0.abs() + d1.abs() + d2.abs() + d3.abs()
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
                // The `i` bit (bit1) controls PRIMASK. On ARMv7-M the `f` bit
                // (bit0) is a FAULTMASK alias; ARMv6-M has neither `f` nor `a`.
                if im & 0b010 != 0 {
                    self.primask = disable;
                }
                if self.isa == Isa::Armv7M && im & 0b001 != 0 {
                    self.faultmask = disable;
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
                Ok(Self::check_branch(
                    pc.wrapping_add(4).wrapping_add_signed(off),
                ))
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
            I::Ldrex { rt, rn, size, off } => {
                let addr = self.regs.read(rn).wrapping_add(off);
                let v = match size {
                    ExSize::Word => match self.mem.read_word(addr) {
                        Ok(v) => v,
                        Err(_) => return self.data_fault(addr, pc),
                    },
                    ExSize::Half => match self.mem.read_half(addr) {
                        Ok(hw) => u32::from(hw),
                        Err(_) => return self.data_fault(addr, pc),
                    },
                    ExSize::Byte => match self.mem.read(addr) {
                        Ok(b) => u32::from(b),
                        Err(_) => return self.data_fault(addr, pc),
                    },
                };
                self.regs.write(rt, v);
                self.exmon = Some(addr);
                Ok(Next::Advance(pc + 4))
            }
            I::Strex {
                rd,
                rt,
                rn,
                size,
                off,
            } => {
                let addr = self.regs.read(rn).wrapping_add(off);
                // The local monitor is cleared whether the store succeeds or
                // not; it only lets a store through while it matches.
                let ok = self.exmon == Some(addr);
                self.exmon = None;
                if ok {
                    let result = match size {
                        ExSize::Word => self.mem.write_word(addr, self.regs.read(rt)),
                        ExSize::Half => self.mem.write_half(addr, self.regs.read(rt) as u16),
                        ExSize::Byte => self.mem.write(addr, self.regs.read(rt) as u8),
                    };
                    if result.is_err() {
                        return self.data_fault(addr, pc);
                    }
                    self.regs.write(rd, 0);
                } else {
                    self.regs.write(rd, 1);
                }
                Ok(Next::Advance(pc + 4))
            }
            I::Clrex => {
                self.exmon = None;
                Ok(Next::Advance(pc + 4))
            }
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
                // MOVW clears the upper half; MOVT preserves it and fills
                // bits 31:16.
                let new = if high {
                    (cur & 0xFFFF) | (u32::from(imm) << 16)
                } else {
                    u32::from(imm)
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
                unprivileged: _,
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
                        // `ra` (bits15:12) is `RdLo` and `rd` (bits11:8) is
                        // `RdHi`, so the accumulator is `RdHi:RdLo`.
                        let accumulator = if matches!(op, MulOp::Smlal | MulOp::Umlal) {
                            (i64::from(self.regs.read(rd)) << 32) | i64::from(self.regs.read(ra))
                        } else {
                            0
                        };
                        let total = product.wrapping_add(accumulator);
                        let lo = total as u32;
                        let hi = (total >> 32) as u32;
                        self.regs.write(ra, lo);
                        self.regs.write(rd, hi);
                    }
                }
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::DspMul { op, rd, rn, rm, ra } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "DSP multiply writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                let result = match op {
                    DspMulOp::Smulbb | DspMulOp::Smulbt | DspMulOp::Smultb | DspMulOp::Smultt => {
                        let n_hi = matches!(op, DspMulOp::Smultb | DspMulOp::Smultt);
                        let m_hi = matches!(op, DspMulOp::Smulbt | DspMulOp::Smultt);
                        self.half_signed(n, n_hi)
                            .wrapping_mul(self.half_signed(m, m_hi))
                    }
                    DspMulOp::Smulwb | DspMulOp::Smulwt => {
                        let m_hi = op == DspMulOp::Smulwt;
                        let product = i64::from(n as i32) * i64::from(self.half_signed(m, m_hi));
                        (product >> 16) as i32
                    }
                    DspMulOp::Smmul | DspMulOp::Smmulr => {
                        let product = i64::from(n as i32) * i64::from(m as i32);
                        let rounded = if op == DspMulOp::Smmulr {
                            product.wrapping_add(0x8000_0000)
                        } else {
                            product
                        };
                        (rounded >> 32) as i32
                    }
                    DspMulOp::Usad8 | DspMulOp::Usada8 => {
                        let sum = self.sum_abs_diff(n, m);
                        if op == DspMulOp::Usada8 {
                            sum.wrapping_add(self.regs.read(ra) as i32)
                        } else {
                            sum
                        }
                    }
                };
                self.regs.write(rd, result as u32);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::DualMul { op, rd, rn, rm, ra } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "dual multiply writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                let m_swap = matches!(op, DualMulOp::Smladx | DualMulOp::Smlsdx);
                match op {
                    // Word forms write the single `Rd`.
                    DualMulOp::Smlabb
                    | DualMulOp::Smlabt
                    | DualMulOp::Smlatb
                    | DualMulOp::Smlatt
                    | DualMulOp::Smlawb
                    | DualMulOp::Smlawt
                    | DualMulOp::Smlad
                    | DualMulOp::Smladx
                    | DualMulOp::Smlsd
                    | DualMulOp::Smlsdx
                    | DualMulOp::Smmla
                    | DualMulOp::Smmlar
                    | DualMulOp::Smmls
                    | DualMulOp::Smmlsr => {
                        let result = match op {
                            // Halfword accumulate: `Ra + Rn[x] * Rm[y]`, with
                            // the wrapping add sticky-setting `Q`.
                            DualMulOp::Smlabb
                            | DualMulOp::Smlabt
                            | DualMulOp::Smlatb
                            | DualMulOp::Smlatt => {
                                let nt = matches!(op, DualMulOp::Smlatb | DualMulOp::Smlatt);
                                let mt = matches!(op, DualMulOp::Smlabt | DualMulOp::Smlatt);
                                let product = self
                                    .half_signed(n, nt)
                                    .wrapping_mul(self.half_signed(m, mt));
                                self.add_setq(product, self.regs.read(ra) as i32)
                            }
                            // Word accumulate of the halfword product's top
                            // bits; `add_setq` on the truncated sum.
                            DualMulOp::Smlawb | DualMulOp::Smlawt => {
                                let mt = op == DualMulOp::Smlawt;
                                let shifted = if mt { m & 0xFFFF_0000 } else { m << 16 };
                                let product = i64::from(n as i32) * i64::from(shifted as i32);
                                let high = (product >> 32) as i32;
                                self.add_setq(high, self.regs.read(ra) as i32)
                            }
                            // `SMLAD`/`SMLADX`, and `SMUAD`/`SMUSD` when
                            // `Ra == 15`. The accumulate add is 64-bit so the
                            // `Q` check sees the true overflow.
                            DualMulOp::Smlad
                            | DualMulOp::Smladx
                            | DualMulOp::Smlsd
                            | DualMulOp::Smlsdx => {
                                let sub = matches!(op, DualMulOp::Smlsd | DualMulOp::Smlsdx);
                                let (p_lo, p_hi) = self.smul_dual(n, m, m_swap);
                                if sub {
                                    // `p_lo - p_hi` cannot overflow.
                                    let diff = p_lo.wrapping_sub(p_hi);
                                    if ra == 15 {
                                        diff
                                    } else {
                                        self.add_setq(diff, self.regs.read(ra) as i32)
                                    }
                                } else if ra == 15 {
                                    self.add_setq(p_lo, p_hi)
                                } else {
                                    let sum = i64::from(p_lo)
                                        + i64::from(p_hi)
                                        + i64::from(self.regs.read(ra) as i32);
                                    let lo = sum as i32;
                                    if (sum >> 32) as i32 != lo >> 31 {
                                        self.xpsr.set_q(true);
                                    }
                                    lo
                                }
                            }
                            // Top word of the 32-by-32 product (no `Q`); the
                            // `SMMLS` subtract borrows into the high word.
                            DualMulOp::Smmla
                            | DualMulOp::Smmlar
                            | DualMulOp::Smmls
                            | DualMulOp::Smmlsr => {
                                let product = i64::from(n as i32) * i64::from(m as i32);
                                let sub = matches!(op, DualMulOp::Smmls | DualMulOp::Smmlsr);
                                let round = matches!(op, DualMulOp::Smmlar | DualMulOp::Smmlsr);
                                if sub {
                                    let acc = u64::from(self.regs.read(ra)) << 32;
                                    let result = acc.wrapping_sub(product as u64);
                                    let mut high = (result >> 32) as u32;
                                    if round {
                                        high = high.wrapping_add(result as u32 >> 31);
                                    }
                                    high as i32
                                } else {
                                    let high = (product >> 32) as u32;
                                    let mut result = high.wrapping_add(self.regs.read(ra));
                                    if round {
                                        result = result.wrapping_add(product as u32 >> 31);
                                    }
                                    result as i32
                                }
                            }
                            _ => unreachable!(),
                        };
                        self.regs.write(rd, result as u32);
                    }
                    // Long forms write `RdLo` (`ra`, bits15:12) and `RdHi`
                    // (`rd`, bits11:8). None of them touch `Q`.
                    DualMulOp::Smlalbb
                    | DualMulOp::Smlalbt
                    | DualMulOp::Smlaltb
                    | DualMulOp::Smlaltt
                    | DualMulOp::Smlald
                    | DualMulOp::Smlaldx
                    | DualMulOp::Smlsld
                    | DualMulOp::Smlsldx
                    | DualMulOp::Umaal => {
                        if ra == 15 {
                            return Err(HaltCause::NotImplemented {
                                what: "dual long multiply writes to pc",
                            });
                        }
                        let acc = (i64::from(self.regs.read(rd) as i32) << 32)
                            | i64::from(self.regs.read(ra));
                        let total = match op {
                            // 16-by-16 products sign-extended into the pair.
                            DualMulOp::Smlalbb
                            | DualMulOp::Smlalbt
                            | DualMulOp::Smlaltb
                            | DualMulOp::Smlaltt => {
                                let nt = matches!(op, DualMulOp::Smlaltb | DualMulOp::Smlaltt);
                                let mt = matches!(op, DualMulOp::Smlalbt | DualMulOp::Smlaltt);
                                let product = i64::from(self.half_signed(n, nt))
                                    * i64::from(self.half_signed(m, mt));
                                acc.wrapping_add(product)
                            }
                            // Dual products summed (or subtracted) and added
                            // to the accumulator.
                            DualMulOp::Smlald
                            | DualMulOp::Smlaldx
                            | DualMulOp::Smlsld
                            | DualMulOp::Smlsldx => {
                                let sub = matches!(op, DualMulOp::Smlsld | DualMulOp::Smlsldx);
                                let (p_lo, p_hi) = self.smul_dual(n, m, m_swap);
                                let pair = if sub {
                                    i64::from(p_lo) - i64::from(p_hi)
                                } else {
                                    i64::from(p_lo) + i64::from(p_hi)
                                };
                                acc.wrapping_add(pair)
                            }
                            // Unsigned product plus both halves of the pair.
                            DualMulOp::Umaal => {
                                let mut total = u64::from(n).wrapping_mul(u64::from(m));
                                total = total.wrapping_add(u64::from(self.regs.read(ra)));
                                total = total.wrapping_add(u64::from(self.regs.read(rd)));
                                let lo = total as u32;
                                let hi = (total >> 32) as u32;
                                self.regs.write(ra, lo);
                                self.regs.write(rd, hi);
                                return Ok(Next::Advance(pc + 4));
                            }
                            _ => unreachable!(),
                        };
                        let lo = total as u32;
                        let hi = (total >> 32) as u32;
                        self.regs.write(ra, lo);
                        self.regs.write(rd, hi);
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
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::Qadd { op, rd, rn, rm } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "saturating add writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                // Everything is widened to `i64` so the sum of two 32-bit
                // values cannot wrap before the clamp sees it.
                let (result, sat) = match op {
                    QaddOp::Qadd => {
                        signed_sat(i64::from(n as i32).wrapping_add(i64::from(m as i32)), 32)
                    }
                    QaddOp::Qsub => {
                        signed_sat(i64::from(n as i32).wrapping_sub(i64::from(m as i32)), 32)
                    }
                    QaddOp::Qdadd => {
                        // The doubled operand is clamped to the signed 32-bit
                        // range before it feeds the addition.
                        let (d, double_sat) = signed_sat(i64::from(m as i32).wrapping_mul(2), 32);
                        let (r, add_sat) =
                            signed_sat(i64::from(n as i32).wrapping_add(i64::from(d)), 32);
                        (r, double_sat || add_sat)
                    }
                    QaddOp::Qdsub => {
                        let (d, double_sat) = signed_sat(i64::from(m as i32).wrapping_mul(2), 32);
                        let (r, sub_sat) =
                            signed_sat(i64::from(n as i32).wrapping_sub(i64::from(d)), 32);
                        (r, double_sat || sub_sat)
                    }
                };
                if sat {
                    self.xpsr.set_q(true);
                }
                self.regs.write(rd, result as u32);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_possible_wrap
            )]
            I::Par {
                op,
                sat,
                rd,
                rn,
                rm,
            } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "parallel add/subtract writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                let (result, ge) = match op {
                    ParOp::Add8 | ParOp::Sub8 => {
                        let add = op == ParOp::Add8;
                        let mut res = 0u32;
                        let mut ge = 0u32;
                        for lane in 0..4 {
                            let (v, g) = par_lane(
                                n >> (lane * 8) & 0xFF,
                                m >> (lane * 8) & 0xFF,
                                add,
                                sat,
                                8,
                            );
                            res |= v << (lane * 8);
                            ge |= u32::from(g) << lane;
                        }
                        (res, ge)
                    }
                    ParOp::Add16 | ParOp::Sub16 => {
                        let add = op == ParOp::Add16;
                        let mut res = 0u32;
                        let mut ge = 0u32;
                        for lane in 0..2 {
                            let (v, g) = par_lane(
                                n >> (lane * 16) & 0xFFFF,
                                m >> (lane * 16) & 0xFFFF,
                                add,
                                sat,
                                16,
                            );
                            res |= v << (lane * 16);
                            ge |= (u32::from(g) * 0b11) << (lane * 2);
                        }
                        (res, ge)
                    }
                    ParOp::Asx | ParOp::Sax => {
                        // `ASX` subtracts the swapped high half from the low
                        // lane and adds the swapped low half to the high lane;
                        // `SAX` mirrors the operations.
                        let add_lo = op == ParOp::Sax;
                        let (lo_v, lo_ge) = par_lane(n & 0xFFFF, m >> 16, add_lo, sat, 16);
                        let (hi_v, hi_ge) = par_lane(n >> 16, m & 0xFFFF, !add_lo, sat, 16);
                        let ge = (u32::from(lo_ge) * 0b11) | ((u32::from(hi_ge) * 0b11) << 2);
                        ((lo_v & 0xFFFF) | (hi_v << 16), ge)
                    }
                };
                if matches!(sat, ParSat::S | ParSat::U) {
                    self.xpsr.set_ge(ge);
                }
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            I::Sel { rd, rn, rm } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "sel writes to pc",
                    });
                }
                let n = self.regs.read(rn);
                let m = self.regs.read(rm);
                let ge = self.xpsr.ge();
                let mut mask = 0u32;
                for lane in 0..4 {
                    if ge & (1 << lane) != 0 {
                        mask |= 0xFF << (lane * 8);
                    }
                }
                self.regs.write(rd, (n & mask) | (m & !mask));
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::ExtendAdd {
                op,
                rd,
                rn,
                rm,
                rot,
            } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "register extend writes to pc",
                    });
                }
                let m = self.regs.read(rm).rotate_right(u32::from(rot) * 8);
                let ext = match op {
                    ExtendAddOp::Sxtah => (m as u16 as i16) as u32,
                    ExtendAddOp::Uxtah => m & 0xFFFF,
                    ExtendAddOp::Sxtab16 => {
                        let lo = (m & 0xFF) as u8 as i8 as u32 & 0xFFFF;
                        let hi = ((m >> 16) & 0xFF) as u8 as i8 as u32 & 0xFFFF;
                        lo | (hi << 16)
                    }
                    ExtendAddOp::Uxtab16 => (m & 0xFF) | ((m >> 16) & 0xFF) << 16,
                    ExtendAddOp::Sxtab => (m & 0xFF) as u8 as i8 as u32,
                    ExtendAddOp::Uxtab => m & 0xFF,
                };
                // `rn == 15` is the two-operand alias: extend only, no addend.
                let result = if rn == 15 {
                    ext
                } else {
                    let n = self.regs.read(rn);
                    match op {
                        ExtendAddOp::Sxtab16 | ExtendAddOp::Uxtab16 => {
                            let lo = (n & 0xFFFF).wrapping_add(ext & 0xFFFF) & 0xFFFF;
                            let hi = (n >> 16).wrapping_add(ext >> 16) & 0xFFFF;
                            lo | (hi << 16)
                        }
                        _ => n.wrapping_add(ext),
                    }
                };
                self.regs.write(rd, result);
                Ok(Next::Advance(pc + 4))
            }
            #[allow(
                clippy::cast_possible_wrap,
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss
            )]
            I::Sat {
                op,
                rd,
                rn,
                sat_n,
                sh,
                shift,
            } => {
                if rd == 15 {
                    return Err(HaltCause::NotImplemented {
                        what: "saturate writes to pc",
                    });
                }
                let v = self.regs.read(rn);
                let (result, sat) = match op {
                    SatOp::Ssat | SatOp::Usat => {
                        // `sh` selects LSL/ASR; the decoder folds the
                        // zero-shift `ASR` alias into the 16-bit forms, so a
                        // nonzero shift is guaranteed here.
                        let shifted = if sh {
                            (v as i32) >> shift
                        } else {
                            v.wrapping_shl(u32::from(shift)) as i32
                        };
                        let signed = op == SatOp::Ssat;
                        let (r, s) = if signed {
                            signed_sat(i64::from(shifted), sat_n)
                        } else {
                            unsigned_sat(i64::from(shifted), sat_n)
                        };
                        (r as u32, s)
                    }
                    SatOp::Ssat16 | SatOp::Usat16 => {
                        let signed = op == SatOp::Ssat16;
                        let lane = |hw: u32| {
                            if signed {
                                signed_sat(i64::from(hw as u16 as i16), sat_n)
                            } else {
                                unsigned_sat(i64::from(hw & 0xFFFF), sat_n)
                            }
                        };
                        let (lo, sat_lo) = lane(v);
                        let (hi, sat_hi) = lane(v >> 16);
                        (
                            (u32::from(hi as u16) << 16) | u32::from(lo as u16),
                            sat_lo || sat_hi,
                        )
                    }
                };
                if sat {
                    self.xpsr.set_q(true);
                }
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
                let (res, c, v) = sub_flags(0, b, false);
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
                    // MULS on ARMv7-M sets N/Z but leaves C alone.
                    self.set_nz(res, false);
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
                let res = !b;
                self.regs.write(rd, res);
                if set_flags {
                    self.set_nz(res, false);
                }
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
        // Exception entry clears the local exclusive monitor.
        self.exmon = None;
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
        let mut sxpsr = self.xpsr.get() & (APSR_QFLAGS | XPSR_T) | u32::from(self.ipsr);
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
        // Exception return clears the local exclusive monitor.
        self.exmon = None;
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
        self.xpsr.set(sxpsr & (APSR_QFLAGS | XPSR_T));
        // Restore ITSTATE from its xPSR[15:10] / [26:25] slots.
        self.itstate = ((sxpsr >> 8) & 0xFC) as u8 | ((sxpsr >> 25) & 0x03) as u8;
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

/// Clamp `v` to the signed `n`-bit range `[-2^(n-1), 2^(n-1) - 1]`, reporting
/// whether the value changed. `n == 32` (the `QADD` family and `SSAT #32`)
/// clamps to the whole signed 32-bit range.
#[allow(clippy::cast_possible_truncation)]
const fn signed_sat(v: i64, n: u8) -> (i32, bool) {
    if n >= 32 {
        if v > i32::MAX as i64 {
            (i32::MAX, true)
        } else if v < i32::MIN as i64 {
            (i32::MIN, true)
        } else {
            (v as i32, false)
        }
    } else {
        let max = (1i64 << (n as u32 - 1)) - 1;
        let min = -(1i64 << (n as u32 - 1));
        if v > max {
            (max as i32, true)
        } else if v < min {
            (min as i32, true)
        } else {
            (v as i32, false)
        }
    }
}

/// Clamp `v` to the unsigned `n`-bit range `[0, 2^n - 1]`, reporting whether
/// the value changed. `n == 32` (USAT #32) clamps only negative values.
#[allow(clippy::cast_possible_truncation)]
const fn unsigned_sat(v: i64, n: u8) -> (i32, bool) {
    if n >= 32 {
        if v < 0 {
            (0, true)
        } else {
            (v as i32, false)
        }
    } else {
        let max = (1i64 << (n as u32)) - 1;
        if v > max {
            (max as i32, true)
        } else if v < 0 {
            (0, true)
        } else {
            (v as i32, false)
        }
    }
}

/// Sign-extend an `8`- or `16`-bit lane to `i64`.
#[allow(clippy::cast_lossless)]
#[must_use]
const fn sign_ext_lane(v: u32, bits: u8) -> i64 {
    let v = (v & if bits == 8 { 0xFF } else { 0xFFFF }) as i64;
    let sign = 1i64 << (bits - 1);
    if v & sign != 0 {
        v - (sign << 1)
    } else {
        v
    }
}

/// Compute one lane of a parallel add/subtract instruction.
///
/// `a`/`b` are already lane-extracted operands; `add` selects add over
/// subtract; `bits` is the lane width (`8` or `16`). Returns the packed lane
/// result and whether the lane's GE flag is set. The `S` form sets GE when the
/// exact signed lane result is non-negative; the `U` form sets GE on the add's
/// carry out (or on the absence of a borrow for subtract). The halving forms
/// shift the exact result right by one; the saturating forms clamp each lane
/// to the `bits`-bit signed/unsigned range without touching the sticky Q flag.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]
const fn par_lane(a: u32, b: u32, add: bool, sat: ParSat, bits: u8) -> (u32, bool) {
    let mask = if bits == 8 { 0xFF } else { 0xFFFF };
    let a = a & mask;
    let b = b & mask;
    let (sa, sb) = match sat {
        ParSat::S | ParSat::Q | ParSat::Sh => (sign_ext_lane(a, bits), sign_ext_lane(b, bits)),
        _ => (a as i64, b as i64),
    };
    let exact = if add { sa + sb } else { sa - sb };
    let (ge, result) = match sat {
        ParSat::S => (exact >= 0, exact as u32 & mask),
        ParSat::U => {
            let carry = if add { exact > mask as i64 } else { exact >= 0 };
            (carry, exact as u32 & mask)
        }
        ParSat::Sh | ParSat::Uh => (false, (exact >> 1) as u32 & mask),
        ParSat::Q => {
            let (r, _) = signed_sat(exact, bits);
            (false, r as u32 & mask)
        }
        ParSat::Uq => {
            let (r, _) = unsigned_sat(exact, bits);
            (false, r as u32 & mask)
        }
    };
    (result, ge)
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
            // A zero amount leaves the C flag alone; a nonzero *multiple* of
            // 32 rotates to no effect but still reports C = bit31.
            if count == 0 {
                (v, None)
            } else if count & 0x1F == 0 {
                (v, Some(v >> 31 & 1 != 0))
            } else {
                let n = count & 0x1F;
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
    fn sysreg_masks_basepri_and_faultmask() {
        let mut c = armv7m_core();
        // msr basepri, r0 -> f380 8011; mrs r0, basepri -> f3ef 8011.
        c.regs.pc = 0x2000_0000;
        c.regs.write(0, 0x80);
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xf3, 0x11, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x80);
        load_inst(&mut c, 0x2000_0004, &[0xef, 0xf3, 0x11, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x80);
        // BASEPRI_MAX (f380 8012) raises the mask but never lowers it.
        c.regs.write(0, 0x90);
        load_inst(&mut c, 0x2000_0008, &[0x80, 0xf3, 0x12, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x80);
        c.regs.write(0, 0x40);
        load_inst(&mut c, 0x2000_000c, &[0x80, 0xf3, 0x12, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x40);
        c.regs.write(0, 0);
        load_inst(&mut c, 0x2000_0010, &[0x80, 0xf3, 0x12, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x40);
        // A plain MSR BASEPRI writes freely in either direction.
        c.regs.write(0, 0x20);
        load_inst(&mut c, 0x2000_0014, &[0x80, 0xf3, 0x11, 0x80]);
        c.step().unwrap();
        assert_eq!(c.basepri, 0x20);
        // FAULTMASK (f380 8013 / f3ef 8013) is a single mask bit.
        c.regs.write(0, 1);
        load_inst(&mut c, 0x2000_0018, &[0x80, 0xf3, 0x13, 0x80]);
        c.step().unwrap();
        assert!(c.faultmask);
        load_inst(&mut c, 0x2000_001c, &[0xef, 0xf3, 0x13, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 1);
    }

    #[test]
    fn cps_primask_and_faultmask() {
        // cpsid i -> b672; cpsie i -> b662; cpsid f -> b671; cpsie f -> b661.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x71, 0xb6]); // cpsid f
        c.step().unwrap();
        assert!(c.faultmask);
        load_inst(&mut c, 0x2000_0002, &[0x61, 0xb6]); // cpsie f
        c.step().unwrap();
        assert!(!c.faultmask);
        load_inst(&mut c, 0x2000_0004, &[0x72, 0xb6]); // cpsid i
        c.step().unwrap();
        assert!(c.primask);
        // On ARMv6-M the `f` bit is not an alias: it must leave FAULTMASK
        // untouched (the register does not exist there).
        let mut m0 = core();
        m0.regs.pc = 0x2000_0000;
        load_inst(&mut m0, 0x2000_0000, &[0x71, 0xb6]); // cpsid f
        m0.step().unwrap();
        assert!(!m0.faultmask);
        assert!(!m0.primask);
    }

    #[test]
    fn msr_apsr_write_masks() {
        // msr apsr, r0 -> f380 8000 writes nzcvq only; the `_g` (8001) and
        // `_nzcvqg` (8002) forms expand the mask to GE.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        let v = 0x8000_0000 | 0x2000_0000 | 0x0800_0000 | 0x000F_0000;
        c.regs.write(0, v);
        load_inst(&mut c, 0x2000_0000, &[0x80, 0xf3, 0x00, 0x80]); // msr apsr, r0
        c.step().unwrap();
        assert!(c.xpsr.n() && c.xpsr.c() && c.xpsr.q());
        assert_eq!(c.xpsr.ge(), 0);
        load_inst(&mut c, 0x2000_0004, &[0x80, 0xf3, 0x01, 0x80]); // msr apsr_g, r0
        c.step().unwrap();
        assert_eq!(c.xpsr.ge(), 0xF);
        load_inst(&mut c, 0x2000_0008, &[0x80, 0xf3, 0x02, 0x80]); // msr apsr_nzcvqg, r0
        c.step().unwrap();
        assert!(c.xpsr.n() && c.xpsr.c() && c.xpsr.q());
        assert_eq!(c.xpsr.ge(), 0xF);
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
    fn mem_unprivileged_ldrt_strt() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x1122_3344);
        c.regs.write(2, 0x2000_0400);
        // strt r1, [r2, #4] f842 1e04 (unprivileged store, no write-back).
        load_inst(&mut c, 0x2000_0000, &[0x42, 0xf8, 0x04, 0x1e]);
        c.step().unwrap();
        assert_eq!(c.mem().read_word(0x2000_0404).unwrap(), 0x1122_3344);
        assert_eq!(c.regs.read(2), 0x2000_0400);
        assert_eq!(c.regs.pc, 0x2000_0004);
        // ldrt r0, [r2, #8] f852 0e08 (unprivileged load, no write-back).
        c.mem().write_word(0x2000_0408, 0xAABB_CCDD).unwrap();
        load_inst(&mut c, 0x2000_0004, &[0x52, 0xf8, 0x08, 0x0e]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0xAABB_CCDD);
        assert_eq!(c.regs.read(2), 0x2000_0400);
        assert_eq!(c.regs.pc, 0x2000_0008);
    }

    #[test]
    fn exclusive_ldrex_strex_roundtrip() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.regs.write(1, 0xAABB_CCDD);
        c.mem().write_word(0x2000_0400, 0x1111_1111).unwrap();
        // ldrex r0, [r2] e852 0f00; strex r3, r1, [r2] e842 1300.
        load_inst(&mut c, 0x2000_0000, &[0x52, 0xe8, 0x00, 0x0f]);
        load_inst(&mut c, 0x2000_0004, &[0x42, 0xe8, 0x00, 0x13]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x1111_1111);
        assert_eq!(c.exmon, Some(0x2000_0400));
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0);
        assert_eq!(c.mem().read_word(0x2000_0400).unwrap(), 0xAABB_CCDD);
        assert_eq!(c.exmon, None);
    }

    #[test]
    fn exclusive_strex_fails_without_match() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.regs.write(0, 0xAABB_CCDD);
        c.mem().write_word(0x2000_0400, 0x1111_1111).unwrap();
        // A lone STREX with an open monitor fails and leaves memory alone.
        load_inst(&mut c, 0x2000_0000, &[0x42, 0xe8, 0x00, 0x03]); // strex r3, r0, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
        assert_eq!(c.mem().read_word(0x2000_0400).unwrap(), 0x1111_1111);
        // A STREX to a different address than the LDREX tagged also fails.
        c.regs.write(2, 0x2000_0404);
        load_inst(&mut c, 0x2000_0004, &[0x52, 0xe8, 0x00, 0x0f]); // ldrex r0, [r2]
        c.step().unwrap();
        assert_eq!(c.exmon, Some(0x2000_0404));
        c.regs.write(2, 0x2000_0400);
        load_inst(&mut c, 0x2000_0008, &[0x42, 0xe8, 0x00, 0x03]); // strex r3, r0, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
        assert_eq!(c.mem().read_word(0x2000_0400).unwrap(), 0x1111_1111);
        // The mismatched STREX still cleared the monitor, so a retry at the
        // tagged address now fails too.
        c.regs.write(2, 0x2000_0404);
        load_inst(&mut c, 0x2000_000c, &[0x42, 0xe8, 0x00, 0x03]); // strex r3, r0, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
    }

    #[test]
    fn exclusive_clrex_and_byte_half_forms() {
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0x2000_0400);
        c.regs.write(1, 0xAABB_CCDD);
        c.mem().write(0x2000_0400, 0x11).unwrap();
        c.mem().write_half(0x2000_0410, 0x1122).unwrap();
        // ldrexb r0, [r2] e8d2 0f4f; strexb r3, r1, [r2] e8c2 1f43.
        load_inst(&mut c, 0x2000_0000, &[0xd2, 0xe8, 0x4f, 0x0f]);
        load_inst(&mut c, 0x2000_0004, &[0xc2, 0xe8, 0x43, 0x1f]);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x11);
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 0);
        assert_eq!(c.mem().read(0x2000_0400).unwrap(), 0xDD);
        // ldrexh r0, [r2] e8d2 0f5f at a fresh address, then CLREX
        // makes the following STREXH fail.
        load_inst(&mut c, 0x2000_0008, &[0xd2, 0xe8, 0x5f, 0x0f]); // ldrexh r0, [r2]
        c.regs.write(2, 0x2000_0410);
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0x1122);
        load_inst(&mut c, 0x2000_000c, &[0xbf, 0xf3, 0x2f, 0x8f]); // clrex
        c.step().unwrap();
        assert_eq!(c.exmon, None);
        load_inst(&mut c, 0x2000_0010, &[0xc2, 0xe8, 0x53, 0x1f]); // strexh r3, r1, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
        assert_eq!(c.mem().read_half(0x2000_0410).unwrap(), 0x1122);
    }

    #[test]
    fn exclusive_word_offset_and_exception_clears_monitor() {
        const HANDLER: u32 = 0x2000_0400;
        let mut c = Core::armv7m(crate::mem::flat::FlatMemory::with_layout(
            &[0x0000_0000, 0x2000_0000],
            &[0x1000, 0x1000],
        ));
        // SVCall is exception 11: its vector sits at 0x2c in the vector table.
        c.mem().write_word(0x2c, HANDLER | 1).unwrap();
        c.regs.sp = 0x2000_0800;
        c.regs.msp = 0x2000_0800;
        c.regs.write(2, 0x2000_0000);
        c.mem().write_word(0x2000_0004, 0xCAFE_F00D).unwrap();
        c.regs.write(0, 0xDEAD_BEEF);
        // ldrex r0, [r2, #4] e852 f001: the monitor is tagged with the
        // effective address.
        load_inst(&mut c, 0x2000_0100, &[0x52, 0xe8, 0x01, 0x0f]);
        c.regs.pc = 0x2000_0100;
        c.step().unwrap();
        assert_eq!(c.regs.read(0), 0xCAFE_F00D);
        assert_eq!(c.exmon, Some(0x2000_0004));
        // svc #10 df0a: entering the handler clears the local monitor, so a
        // STREX in Thread mode afterwards must fail.
        load_inst(&mut c, 0x2000_0104, &[0x0a, 0xdf]);
        c.step().unwrap();
        assert_eq!(c.exmon, None);
        assert_eq!(c.ipsr, 11);
        // Handler returns via bx lr; the monitor stays clear across the
        // exception return as well.
        load_inst(&mut c, HANDLER, &[0x70, 0x47]);
        c.step().unwrap();
        assert_eq!(c.ipsr, 0);
        assert_eq!(c.exmon, None);
        c.regs.pc = 0x2000_0108;
        c.regs.write(2, 0x2000_0004);
        load_inst(&mut c, 0x2000_0108, &[0x42, 0xe8, 0x00, 0x03]); // strex r3, r0, [r2]
        c.step().unwrap();
        assert_eq!(c.regs.read(3), 1);
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
        // smull r0, r1, r2, r3 (fb82 0103): -7 * 5 = -35 -> r1:r0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 0xFFFF_FFF9);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x82, 0xfb, 0x03, 0x01]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xFFFF_FFDD);
        assert_eq!(c.regs.r[1], 0xFFFF_FFFF);

        // umull r0, r1, r9, r2 (fba9 0102): 0x10000 * 0x10000 = 2^32 -> r1 = 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x0001_0000);
        c.regs.write(2, 0x0001_0000);
        load_inst(&mut c, 0x2000_0000, &[0xa9, 0xfb, 0x02, 0x01]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        assert_eq!(c.regs.r[1], 1);

        // smlal r2, r1, r3, r4 (fbc3 2104): acc r1:r2 = 0x1_0000_0010,
        // product -5 * 4 = -20 -> total = 0xFFFF_FFFC, r1 cleared.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0xFFFF_FFFB);
        c.regs.write(4, 4);
        c.regs.write(1, 1);
        c.regs.write(2, 16);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0x04, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xFFFF_FFFC);
        assert_eq!(c.regs.r[1], 0);

        // umlal r2, r1, r3, r4 (fbe3 2104): acc r1:r2 = 0x2_0000_0064,
        // product 1000 * 10 = 10000 -> total = 0x2_0000_2774.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 1000);
        c.regs.write(4, 10);
        c.regs.write(1, 2);
        c.regs.write(2, 100);
        load_inst(&mut c, 0x2000_0000, &[0xe3, 0xfb, 0x04, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x0000_2774);
        assert_eq!(c.regs.r[1], 2);

        // Writing the low result register `RdLo` (a PC) is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(2, 3);
        c.regs.write(3, 4);
        load_inst(&mut c, 0x2000_0000, &[0xa2, 0xfb, 0x03, 0xf1]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn dsp_mul_ops() {
        // smulbb r0, r1, r2 (fb11 f002): low*low, both sign-extended. With
        // r1 = 0xffff0003 the low half is 3, r2 low half is 4 -> 12.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_0003);
        c.regs.write(2, 0x0000_0004);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 12);

        // smulbt r0, r1, r2 (fb11 f012): n low * m high. r1 low = 3,
        // r2 = 0xffff0004 high half = -1 -> -3.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_0003);
        c.regs.write(2, 0xffff_0004);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_fffd);

        // smultb r0, r1, r2 (fb11 f022): n high * m low. r1 high = -1,
        // r2 low = 4 -> -4.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_0003);
        c.regs.write(2, 0x0000_0004);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x22, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_fffc);

        // smultt r0, r1, r2 (fb11 f032): high*high. Both high halves -1 -> 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_0003);
        c.regs.write(2, 0xffff_0004);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x32, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 1);

        // smulwb r0, r1, r2 (fb31 f002): r1 is a full signed word,
        // r2 high half -32768. 32766 * 32768 = 0x3fff0000, >> 16 = 0x3fff.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_8002);
        c.regs.write(2, 0x0000_8000);
        load_inst(&mut c, 0x2000_0000, &[0x31, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0000_3fff);

        // smulwt r0, r1, r2 (fb31 f012): -32766 * 3 = -98298; the arithmetic
        // shift of the 48-bit product rounds toward negative infinity to -2.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_8002);
        c.regs.write(2, 0x0003_0000);
        load_inst(&mut c, 0x2000_0000, &[0x31, 0xfb, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_fffe);

        // smmul r0, r1, r2 (fb51 f002): 0x80000000 * -1 = 0x80000000, and the
        // top word of that is 0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x51, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);

        // smmulr r0, r1, r2 (fb51 f012): the same product, plus 0x80000000
        // before the shift wraps it into the next word -> 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x51, 0xfb, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 1);

        // usad8 r0, r1, r2 (fb71 f002): each byte differs by 255, sum = 1020.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xff00_ff00);
        c.regs.write(2, 0x00ff_00ff);
        load_inst(&mut c, 0x2000_0000, &[0x71, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x3fc);

        // usada8 r0, r1, r2, r3 (fb71 3002): 1020 + 0x12345678.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xff00_ff00);
        c.regs.write(2, 0x00ff_00ff);
        c.regs.write(3, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x71, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x1234_5a74);

        // A DSP multiply that targets the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x02, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn dual_mul_ops() {
        // smlad r0, r1, r2, r3 (fb21 3002): p_lo + p_hi + Ra. With both halves
        // -32768 the products are 2^30 each; Ra = -1 makes the 64-bit sum
        // 0x7FFFFFFF, so Q is NOT set (a two-step add would wrongly set it).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        c.regs.write(3, 0xFFFF_FFFF);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7FFF_FFFF);
        assert!(!c.xpsr.q());

        // The same products with Ra = 0x40000000 overflow the 32-bit result,
        // and the 64-bit check sees it: Q is set.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        c.regs.write(3, 0x4000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xC000_0000);
        assert!(c.xpsr.q());

        // smuad r0, r1, r2 (fb21 f002): the sentinel Ra drops the accumulate,
        // and the single 32-bit add wraps with Q set.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);
        assert!(c.xpsr.q());

        // smladx r0, r1, r2, r3 (fb21 3012) swaps Rm's halves: without the
        // swap the products cancel, with it r0 = 7 * 1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_0007);
        c.regs.write(2, 0x0001_0000);
        c.regs.write(3, 0);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x12, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 7);

        // smlsd r0, r1, r2, r3 (fb41 3002): p_lo - p_hi + Ra, no overflow.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x41, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 5);
        assert!(!c.xpsr.q());

        // smusd r0, r1, r2 (fb41 f002): the difference alone, no Q.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_8000);
        c.regs.write(2, 0x8000_8000);
        load_inst(&mut c, 0x2000_0000, &[0x41, 0xfb, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        assert!(!c.xpsr.q());

        // smlabb r0, r1, r2, r3 (fb11 3002): Ra + halfword product.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_0003);
        c.regs.write(2, 0x0000_0004);
        c.regs.write(3, 100);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 112);

        // smlabt r0, r1, r2, r3 (fb11 3012): the accumulate add overflows and
        // wraps, sticky-setting Q.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_7fff);
        c.regs.write(2, 0x7fff_0000);
        c.regs.write(3, 0x7fff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x11, 0xfb, 0x12, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xbfff_0000);
        assert!(c.xpsr.q());

        // smlawb r0, r1, r2, r3 (fb31 3002): (Rn * Rm[15:0]) >> 16 + Ra.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_1000);
        c.regs.write(2, 0x0000_ffff);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x31, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 4);

        // smmla r0, r1, r2, r3 (fb51 3002): top word of the product + Ra.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0x8000_0000);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x51, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x4000_0005);

        // smmlar r0, r1, r2, r3 (fb51 3012): adds the low word's bit 31.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_ffff);
        c.regs.write(2, 0x8000_0000);
        c.regs.write(3, 5);
        load_inst(&mut c, 0x2000_0000, &[0x51, 0xfb, 0x12, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 6);

        // smmls r0, r1, r2, r3 (fb61 3002): (Ra << 32) - product, borrow
        // included, so a tiny product of 6 with Ra = 0 gives -1, not 0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        c.regs.write(3, 0);
        load_inst(&mut c, 0x2000_0000, &[0x61, 0xfb, 0x02, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_ffff);

        // smmlsr r0, r1, r2, r3 (fb61 3012): the rounding bit borrows back.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        c.regs.write(3, 0);
        load_inst(&mut c, 0x2000_0000, &[0x61, 0xfb, 0x12, 0x30]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);

        // smlald r2, r1, r3, r4 (fbc3 21c4): the dual sum accumulates into
        // RdHi:RdLo and never touches Q.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x8000_8000);
        c.regs.write(4, 0x8000_8000);
        c.regs.write(1, 1);
        c.regs.write(2, 0);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0xc4, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x8000_0000);
        assert_eq!(c.regs.r[1], 1);
        assert!(!c.xpsr.q());

        // smlsld r2, r1, r3, r4 (fbd3 21c4): identical products cancel.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x8000_8000);
        c.regs.write(4, 0x8000_8000);
        c.regs.write(1, 0x1234_5678);
        c.regs.write(2, 0x9abc_def0);
        load_inst(&mut c, 0x2000_0000, &[0xd3, 0xfb, 0xc4, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x9abc_def0);
        assert_eq!(c.regs.r[1], 0x1234_5678);

        // smlalbb r2, r1, r3, r4 (fbc3 2184): a sign-extended halfword product
        // added to the pair.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x0000_0003);
        c.regs.write(4, 0x0000_0004);
        c.regs.write(1, 1);
        c.regs.write(2, 100);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0x84, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x0000_0070);
        assert_eq!(c.regs.r[1], 1);

        // umaal r2, r1, r3, r4 (fbe3 2164): unsigned product plus both halves
        // of the pair, low to RdLo and high to RdHi.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0xffff_ffff);
        c.regs.write(4, 0xffff_ffff);
        c.regs.write(1, 1);
        c.regs.write(2, 1);
        load_inst(&mut c, 0x2000_0000, &[0xe3, 0xfb, 0x64, 0x21]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 3);
        assert_eq!(c.regs.r[1], 0xffff_fffe);

        // A word form targeting the PC and a long form targeting RdLo=PC are
        // both rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xfb, 0x02, 0x3f]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 2);
        c.regs.write(4, 3);
        load_inst(&mut c, 0x2000_0000, &[0xc3, 0xfb, 0xc4, 0xf1]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn qadd_family_saturates() {
        // qadd r0, r1, r2 (fa81 f082) without saturation: 2 + 3 = 5, Q stays
        // clear.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 2);
        c.regs.write(2, 3);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 5);
        assert!(!c.xpsr.q());

        // qadd r0, r1, r2 with r1 = INT32_MAX, r2 = 1: the sum clamps to
        // INT32_MAX and the Q flag latches.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x7fff_ffff);
        c.regs.write(2, 1);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7fff_ffff);
        assert!(c.xpsr.q());

        // qadd r0, r1, r2 with r1 = INT32_MIN, r2 = -1: clamps downward and
        // the Q flag latches again (it is sticky, not re-cleared).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);
        assert!(c.xpsr.q());

        // qsub r0, r1, r2 (fa81 f0a2): INT32_MIN - 1 clamps to INT32_MIN.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 1);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0xa2, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);
        assert!(c.xpsr.q());

        // qdadd r0, r1, r2 (fa81 f192): INT32_MAX + doubled 1 clamps the sum.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x7fff_ffff);
        c.regs.write(2, 1);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x92, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7fff_ffff);
        assert!(c.xpsr.q());

        // qdadd r0, r1, r2 with r1 = 0, r2 = INT32_MAX: the *doubling*
        // saturates to INT32_MAX, which also sets Q.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0);
        c.regs.write(2, 0x7fff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x92, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7fff_ffff);
        assert!(c.xpsr.q());

        // qdsub r0, r1, r2 (fa81 f1b2): 5 - doubled 2 = 1, no saturation
        // anywhere.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 5);
        c.regs.write(2, 2);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0xb2, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 1);
        assert!(!c.xpsr.q());

        // qdsub r0, r1, r2 with r1 = INT32_MIN, r2 = 0x40000000: the doubling
        // clamps to INT32_MAX (Q), and INT32_MIN - that clamps to INT32_MIN
        // (Q stays latched).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        c.regs.write(2, 0x4000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0xb2, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8000_0000);
        assert!(c.xpsr.q());

        // A saturating add targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x82, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn parallel_add_sub_executes() {
        // sadd8 r0, r1, r2 (fa81 f002): per-byte signed add with GE written per
        // lane on non-negative results.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_00FF);
        c.regs.write(2, 0x0000_0200);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8001_02FF);
        assert_eq!(c.xpsr.ge(), 0b0110);

        // ssub8 r0, r1, r2 (fac1 f002): lane 0 borrows, the rest are zero.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0102_0304);
        c.regs.write(2, 0x0102_0305);
        load_inst(&mut c, 0x2000_0000, &[0xc1, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0000_00FF);
        assert_eq!(c.xpsr.ge(), 0b1110);

        // sadd16 r0, r1, r2 (fa91 f002): each halfword lane sets two GE bits.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_0002);
        c.regs.write(2, 0x0001_0002);
        load_inst(&mut c, 0x2000_0000, &[0x91, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x8002_0004);
        assert_eq!(c.xpsr.ge(), 0b0011);

        // uadd8 r0, r1, r2 (fa81 f042): GE follows the per-byte carry out.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0102_80FF);
        c.regs.write(2, 0x0102_80FF);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x42, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0204_00FE);
        assert_eq!(c.xpsr.ge(), 0b0011);

        // qadd8 r0, r1, r2 (fa81 f012): the saturated lane clamps, and neither
        // the Q flag nor GE is touched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0102_7F80);
        c.regs.write(2, 0x0000_0101);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x12, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0102_7F81);
        assert!(!c.xpsr.q());
        assert_eq!(c.xpsr.ge(), 0);

        // uqsub8 r0, r1, r2 (fac1 f052): the underflowing lane clamps to zero.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_0010);
        c.regs.write(2, 0x0000_0020);
        load_inst(&mut c, 0x2000_0000, &[0xc1, 0xfa, 0x52, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0);
        assert!(!c.xpsr.q());

        // shadd16 r0, r1, r2 (fa91 f022): each halfword lane halves the exact
        // signed result (negative lanes round away from zero).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_7FFF);
        c.regs.write(2, 0x0000_0001);
        load_inst(&mut c, 0x2000_0000, &[0x91, 0xfa, 0x22, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xC000_4000);

        // uhsub8 r0, r1, r2 (fac1 f062): 0x10 - 0x20 wraps to 0xF8 after the
        // halving shift, as on the hardware.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_0010);
        c.regs.write(2, 0x0000_0020);
        load_inst(&mut c, 0x2000_0000, &[0xc1, 0xfa, 0x62, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0000_00F8);

        // sasx r0, r1, r2 (faa1 f002): lo = Rn_lo - Rm_hi, hi = Rn_hi + Rm_lo.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0001_0002);
        c.regs.write(2, 0x0004_0008);
        load_inst(&mut c, 0x2000_0000, &[0xa1, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x0009_FFFE);
        assert_eq!(c.xpsr.ge(), 0b1100);

        // ssax r0, r1, r2 (fae1 f002): lo = Rn_lo + Rm_hi, hi = Rn_hi - Rm_lo.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0001_0002);
        c.regs.write(2, 0x0004_0008);
        load_inst(&mut c, 0x2000_0000, &[0xe1, 0xfa, 0x02, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xFFF9_0006);
        assert_eq!(c.xpsr.ge(), 0b0011);

        // A parallel op targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xfa, 0x02, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn sel_selects_bytes_by_ge() {
        // sel r0, r1, r2 (faa1 f082): byte i follows GE flag i.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x1122_3344);
        c.regs.write(2, 0xAABB_CCDD);
        c.xpsr.set_ge(0b0101);
        load_inst(&mut c, 0x2000_0000, &[0xa1, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xAA22_CC44);

        // Chained after a parallel op: sadd8 then sel, so SEL reads the GE the
        // parallel op just wrote.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_00FF);
        c.regs.write(2, 0x0000_0200);
        load_inst(
            &mut c,
            0x2000_0000,
            &[0x81, 0xfa, 0x02, 0xf0, 0xa1, 0xfa, 0x82, 0xf3],
        );
        c.step().unwrap();
        c.step().unwrap();
        // sadd8 wrote GE = 0b0110: bytes 1 and 2 follow r1, the others r2.
        assert_eq!(c.regs.r[3], 0x0001_0000);

        // sel targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0xa1, 0xfa, 0x82, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn extend_add_executes() {
        // sxth r0, r1 (fa0f f081): the two-operand alias sign-extends the low
        // halfword without any addend.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0000_ff80);
        load_inst(&mut c, 0x2000_0000, &[0x0f, 0xfa, 0x81, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_ff80);

        // sxtab r2, r3, r4 (fa43 f284): 1 + sign-extended byte 0x80.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 1);
        c.regs.write(4, 0x0000_0080);
        load_inst(&mut c, 0x2000_0000, &[0x43, 0xfa, 0x84, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xffff_ff81);

        // uxtah r5, r6, r7 (fa16 f587): 0x100 + zero-extended halfword 0xff00.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(6, 0x100);
        c.regs.write(7, 0x0001_ff00);
        load_inst(&mut c, 0x2000_0000, &[0x16, 0xfa, 0x87, 0xf5]);
        c.step().unwrap();
        assert_eq!(c.regs.r[5], 0x0001_0000);

        // sxtab16 r8, r9, r10 (fa29 fa88): per-halfword signed byte extends.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x0001_0002);
        c.regs.write(10, 0x80ff_0080);
        load_inst(&mut c, 0x2000_0000, &[0x29, 0xfa, 0x8a, 0xf8]);
        c.step().unwrap();
        // lo = 2 + sext(0x80) = 0xffffff82 & 0xffff = 0xff82; hi = 1 + sext(0xff)
        // = 0x0000...00, so the result is 0x0000_ff82.
        assert_eq!(c.regs.r[8], 0x0000_ff82);

        // uxtab16 r0, r1, r2 (fa31 f002): per-halfword add wraps each lane.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_ffff);
        c.regs.write(2, 0x0000_0001);
        load_inst(&mut c, 0x2000_0000, &[0x31, 0xfa, 0x82, 0xf0]);
        c.step().unwrap();
        // lo = 0xffff + 1 -> 0x0000; hi = 0xffff + 0 -> 0xffff.
        assert_eq!(c.regs.r[0], 0xffff_0000);

        // sxtab r0, r1, r2, ror #8 (fa41 f082): the byte that feeds the extend
        // comes from bit 8 after the rotation.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 1);
        c.regs.write(2, 0x0000_8000);
        load_inst(&mut c, 0x2000_0000, &[0x41, 0xfa, 0x92, 0xf0]);
        c.step().unwrap();
        // ror #8 turns 0x00008000 into 0x00000080: extend that byte and add 1.
        assert_eq!(c.regs.r[0], 0xffff_ff81);

        // sxtb16 r0, r1 (fa2f f081): both bytes sign-extend into halfwords.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x80ff_007f);
        load_inst(&mut c, 0x2000_0000, &[0x2f, 0xfa, 0x81, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_007f);

        // A register extend targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x0f, 0xfa, 0x81, 0xff]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn saturate_ssat_usat() {
        // ssat r0, #8, r1, lsl #4 (f301 1007): 0x01 << 4 = 16 fits the signed
        // 8-bit range, so no Q flag.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x01);
        load_inst(&mut c, 0x2000_0000, &[0x01, 0xf3, 0x07, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 16);
        assert!(!c.xpsr.q());

        // ssat r0, #8, r1, lsl #4: 0x08 << 4 = 128 clamps to 127, Q latches.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x08);
        load_inst(&mut c, 0x2000_0000, &[0x01, 0xf3, 0x07, 0x10]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7f);
        assert!(c.xpsr.q());

        // ssat r0, #8, r1, asr #1 (f321 0047): -1 stays -1.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xf3, 0x47, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_ffff);
        assert!(!c.xpsr.q());

        // ssat r0, #8, r1, asr #1: 0x80000000 >> 1 clamps to -128.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8000_0000);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xf3, 0x47, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xffff_ff80);
        assert!(c.xpsr.q());

        // usat r2, #8, r3, lsl #2 (f383 0287): 0x10 << 2 = 64 fits [0, 255].
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x10);
        load_inst(&mut c, 0x2000_0000, &[0x83, 0xf3, 0x87, 0x02]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 64);
        assert!(!c.xpsr.q());

        // usat r2, #8, r3, lsl #2: 0x40 << 2 = 256 clamps to 255, Q latches.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x40);
        load_inst(&mut c, 0x2000_0000, &[0x83, 0xf3, 0x87, 0x02]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0xff);
        assert!(c.xpsr.q());

        // usat r2, #8, r3 (f383 2007): a negative value clamps to 0.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0xffff_ffff);
        load_inst(&mut c, 0x2000_0000, &[0x83, 0xf3, 0x07, 0x20]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0);
        assert!(c.xpsr.q());

        // ssat16 r0, #8, r1 (f321 0007): hi half 0x8001 clamps to -128, the
        // low half 0x007f fits. Both halves are written and Q latches.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x8001_007f);
        load_inst(&mut c, 0x2000_0000, &[0x21, 0xf3, 0x07, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0xff80_007f);
        assert!(c.xpsr.q());

        // usat16 r0, #4, r1 (f3a1 0003): hi half 0x0010 clamps to 15, the
        // low half 0x000f fits.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0010_000f);
        load_inst(&mut c, 0x2000_0000, &[0xa1, 0xf3, 0x03, 0x00]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x000f_000f);
        assert!(c.xpsr.q());

        // A saturate targeting the PC is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x01, 0xf3, 0x07, 0x1f]);
        assert!(matches!(c.step(), Err(HaltCause::NotImplemented { .. })));
    }

    #[test]
    fn sysreg_reads_and_writes_q_flag() {
        // `msr apsr, r1` writes the Q flag; `mrs r0, apsr` reads it back.
        // msr apsr, r1 -> f381 8000; mrs r0, apsr -> f3ef 8000.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(1, 0x0800_0000);
        load_inst(&mut c, 0x2000_0000, &[0x81, 0xf3, 0x00, 0x80]);
        c.step().unwrap();
        assert!(c.xpsr.q());
        load_inst(&mut c, 0x2000_0004, &[0xef, 0xf3, 0x00, 0x80]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0] & 0x0800_0000, 0x0800_0000);
        assert_eq!(c.regs.r[0] & 0x07f0_0000, 0);
    }

    #[test]
    fn unary_rev_and_flags() {
        // rev r0, r1 (fa91 f081): byte-reverse a word, flags untouched.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.xpsr.set_nzcv(true, true, true, true);
        c.regs.write(1, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x91, 0xfa, 0x81, 0xf0]);
        c.step().unwrap();
        assert_eq!(c.regs.r[0], 0x7856_3412);
        assert_eq!(c.regs.pc, 0x2000_0004);
        assert!(c.xpsr.n() && c.xpsr.z() && c.xpsr.c() && c.xpsr.v());

        // rev16 r2, r3 (fa93 f293): swap bytes within each halfword.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(3, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x93, 0xfa, 0x93, 0xf2]);
        c.step().unwrap();
        assert_eq!(c.regs.r[2], 0x3412_7856);

        // revsh r4, r5 (fa95 f4b5): byte-swap the low half, sign-extend.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_1234);
        load_inst(&mut c, 0x2000_0000, &[0x95, 0xfa, 0xb5, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 0x0000_3412);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(5, 0x0000_80FF);
        load_inst(&mut c, 0x2000_0000, &[0x95, 0xfa, 0xb5, 0xf4]);
        c.step().unwrap();
        assert_eq!(c.regs.r[4], 0xFFFF_FF80);
    }

    #[test]
    fn unary_rbit_clz() {
        // rbit r6, r7 (fa97 f6a7): mirror all 32 bits.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(7, 0x1234_5678);
        load_inst(&mut c, 0x2000_0000, &[0x97, 0xfa, 0xa7, 0xf6]);
        c.step().unwrap();
        assert_eq!(c.regs.r[6], 0x1E6A_2C48);

        // clz r8, r9 (fab9 f889).
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x000F_0000);
        load_inst(&mut c, 0x2000_0000, &[0xb9, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 12);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0x8000_0000);
        load_inst(&mut c, 0x2000_0000, &[0xb9, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 0);

        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        c.regs.write(9, 0);
        load_inst(&mut c, 0x2000_0000, &[0xb9, 0xfa, 0x89, 0xf8]);
        c.step().unwrap();
        assert_eq!(c.regs.r[8], 32);

        // Writing the destination register 15 is rejected.
        let mut c = armv7m_core();
        c.regs.pc = 0x2000_0000;
        load_inst(&mut c, 0x2000_0000, &[0x90, 0xfa, 0x80, 0xff]);
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
