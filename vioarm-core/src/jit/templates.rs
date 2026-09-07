//! Template coverage + differential harness (P2-2).
//!
//! [`is_templated`] is the single coverage decision shared by both hosts
//! (CI parity: an op is native everywhere or interpreted everywhere).
//! The `harness` module executes one template natively over scratch state
//! and through the interpreter over identical state, comparing full
//! registers + flags word; every template test below is random-input
//! differential with directed edges. The harness serves P2-2 through P2-5
//! unchanged: later phases only add op classes and arch emitters.

use crate::cpu::decode16::Inst16;
use crate::jit::builder::TbOp;

#[cfg(target_arch = "aarch64")]
use super::templates_aarch64 as arch;
#[cfg(target_arch = "x86_64")]
use super::templates_x64 as arch;
#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
compile_error!("jit templates support x86-64 and AArch64 hosts only");

/// Whether `op` has a native template on every host. S0 (P2-2): plain
/// 16-bit Thumb data-processing with no PC involvement (`r15` anywhere
/// bails — the guest PC is stale mid-block by construction; branches end
/// blocks and go through the driver instead).
pub fn is_templated(op: &TbOp) -> bool {
    match *op {
        TbOp::V16(inst) => match inst {
            Inst16::ShiftImm { rd, rm, .. } => rd != 15 && rm != 15,
            Inst16::ShiftReg { rd, rn, rm, .. } => rd != 15 && rn != 15 && rm != 15,
            Inst16::AddReg { rd, rn, rm } => rd != 15 && rn != 15 && rm != 15,
            Inst16::SubReg { rd, rn, rm } => rd != 15 && rn != 15 && rm != 15,
            Inst16::AddImm3 { rd, rn, .. } => rd != 15 && rn != 15,
            Inst16::SubImm3 { rd, rn, .. } => rd != 15 && rn != 15,
            Inst16::MovImm { rd, .. } => rd != 15,
            Inst16::CmpImm { rn, .. } => rn != 15,
            Inst16::AddImm8 { rd, .. } => rd != 15,
            Inst16::SubImm8 { rd, .. } => rd != 15,
            Inst16::Alu { rd, rm, .. } => rd != 15 && rm != 15,
            Inst16::SpecialCmp { rn, rm } => rn != 15 && rm != 15,
            // P2-2 terminators: 16-bit static branches (targets resolve
            // without guest state; reg/pc forms stay interpreted).
            Inst16::Branch { .. } => true,
            Inst16::BranchCond { cond, .. } => cond <= 13,
            Inst16::CbzCbnz { rn, .. } => rn != 15,
            // P2-3 single transfers (S1): no PC involvement (`r15`
            // anywhere bails — the guest PC is stale mid-block; `rn ==
            // 15` is allowed only for the integer-literal base form,
            // whose address is static). Dual/multiple/exclusive/VFP
            // forms stay interpreted (P2-5/P2-6).
            Inst16::LsReg { rt, rn, rm, .. } => rt != 15 && rn != 15 && rm != 15,
            Inst16::LsImm { rt, rn, .. } => rt != 15 && rn != 15,
            Inst16::LsSp { rt, .. } => rt != 15,
            Inst16::LdrLiteral { rt, .. } => rt != 15,
            _ => false,
        },
        TbOp::V32(inst) => match inst {
            crate::cpu::decode32::Inst32::Mem {
                access,
                rt,
                rn,
                rm,
                writeback,
                ..
            } => {
                use crate::cpu::decode32::MemOp as M;
                let single = matches!(
                    access,
                    M::Ldr | M::Str | M::Ldrb | M::Strb | M::Ldrh | M::Strh | M::Ldrsb | M::Ldrsh
                );
                single
                    && rt != 15
                    && (rn != 15 || (!writeback && is_mem_load(access)))
                    && rm.is_none_or(|r| r != 15)
            }
            _ => false,
        },
    }
}

/// Whether a 32-bit `Mem` access loads (vs stores).
fn is_mem_load(access: crate::cpu::decode32::MemOp) -> bool {
    use crate::cpu::decode32::MemOp as M;
    matches!(access, M::Ldr | M::Ldrb | M::Ldrh | M::Ldrsb | M::Ldrsh)
}

/// Static memory-access descriptor shared by template emission and the
/// side-exit fill path (single source: `(byte size, is_write)`).
/// `None` for non-memory or deferred (dual/multiple/exclusive) ops.
pub(crate) fn mem_access(op: &TbOp) -> Option<(u32, bool)> {
    match *op {
        TbOp::V16(inst) => {
            use crate::cpu::decode16::{LsImm, LsReg};
            match inst {
                Inst16::LsReg { kind, .. } => match kind {
                    LsReg::Str => Some((4, true)),
                    LsReg::Ldr => Some((4, false)),
                    LsReg::Strh => Some((2, true)),
                    LsReg::Ldrh => Some((2, false)),
                    LsReg::Strb => Some((1, true)),
                    LsReg::Ldrb | LsReg::Ldrsb => Some((1, false)),
                    LsReg::Ldrsh => Some((2, false)),
                },
                Inst16::LsImm { kind, .. } => match kind {
                    LsImm::Str => Some((4, true)),
                    LsImm::Ldr => Some((4, false)),
                    LsImm::Strh => Some((2, true)),
                    LsImm::Ldrh => Some((2, false)),
                    LsImm::Strb => Some((1, true)),
                    LsImm::Ldrb => Some((1, false)),
                },
                Inst16::LsSp { is_load, .. } => Some((4, !is_load)),
                Inst16::LdrLiteral { .. } => Some((4, false)),
                _ => None,
            }
        }
        TbOp::V32(inst) => match inst {
            crate::cpu::decode32::Inst32::Mem { access, .. } => {
                use crate::cpu::decode32::MemOp as M;
                match access {
                    M::Str => Some((4, true)),
                    M::Ldr => Some((4, false)),
                    M::Strh => Some((2, true)),
                    M::Ldrh => Some((2, false)),
                    M::Strb => Some((1, true)),
                    M::Ldrb | M::Ldrsb => Some((1, false)),
                    M::Ldrsh => Some((2, false)),
                }
            }
            _ => None,
        },
    }
}

/// Emit one templated op at guest `pc` into an arch assembler. Returns
/// `false` for untemplated ops (the block then stays interpreted).
pub fn emit_op(ops: &mut arch::Assembler, op: &TbOp, pc: u32, unalign_trap: bool) -> bool {
    if !is_templated(op) {
        return false;
    }
    arch::emit_op(ops, op, pc, unalign_trap)
}

/// Assemble prologue + one template + epilogue for harness runs. S0 ops
/// fall through (`FALL`); terminators exit with their branch code.
pub(crate) fn assemble_single(
    op: &TbOp,
    pc: u32,
    unalign_trap: bool,
) -> Result<super::native::NativeBlock, super::EmitError> {
    arch::assemble_single(op, pc, unalign_trap)
}

/// Emit a whole block natively: prologue, one template per op in order,
/// then the terminator's own epilogue — or, when the block ends in plain
/// ops, a `FALL` epilogue to the fallthrough address. Bails with
/// [`EmitError::Untemplated`] on the first untemplated op (the block then
/// stays interpreted); the builder guarantees terminators only appear
/// last, so anything after one would be unreachable anyway.
pub fn emit_block(
    body: &super::TbBody,
    unalign_trap: bool,
) -> Result<super::native::NativeBlock, super::EmitError> {
    use super::builder::op_len;
    use super::native::{emit_epilogue, emit_prologue, NativeBlock, EXIT_FALL};
    use dynasmrt::AssemblyOffset;

    let mut ops = arch::Assembler::new()?;
    emit_prologue(&mut ops);
    let mut pc = body.pc;
    let mut terminated = false;
    for op in &body.ops {
        if !emit_op(&mut ops, op, pc, unalign_trap) {
            return Err(super::EmitError::Untemplated);
        }
        terminated = matches!(
            op,
            TbOp::V16(
                crate::cpu::decode16::Inst16::Branch { .. }
                    | crate::cpu::decode16::Inst16::BranchCond { .. }
                    | crate::cpu::decode16::Inst16::CbzCbnz { .. }
            )
        );
        pc = pc.wrapping_add(op_len(op));
    }
    if !terminated {
        emit_epilogue(&mut ops, EXIT_FALL, body.end_pc);
    }
    // A terminator already returned: emitting another epilogue would only
    // append dead bytes (its `ret` ends the stream).
    let last_load = last_op_load(body.ops.last());
    ops.finalize()
        .map(|buf| NativeBlock::new(buf, AssemblyOffset(0), last_load))
        .map_err(|_| super::EmitError::Finalize)
}

/// Load-use tracker value stepping `op` (as a block tail) would leave
/// behind, mirroring `base_cycles`: `Some(rt)` for single loads into
/// `rt < 13`, else `None`. Timing-only state.
fn last_op_load(op: Option<&TbOp>) -> Option<u8> {
    use crate::cpu::decode16::{LsImm, LsReg};
    use crate::cpu::decode32::MemOp;
    let op = op?;
    let rt = match *op {
        TbOp::V16(crate::cpu::decode16::Inst16::LdrLiteral { rt, .. }) => rt,
        TbOp::V16(crate::cpu::decode16::Inst16::LsSp {
            rt, is_load: true, ..
        }) => rt,
        TbOp::V16(crate::cpu::decode16::Inst16::LsReg {
            rt,
            kind: LsReg::Ldr | LsReg::Ldrb | LsReg::Ldrh | LsReg::Ldrsb | LsReg::Ldrsh,
            ..
        }) => rt,
        TbOp::V16(crate::cpu::decode16::Inst16::LsImm {
            rt,
            kind: LsImm::Ldr | LsImm::Ldrb | LsImm::Ldrh,
            ..
        }) => rt,
        TbOp::V32(crate::cpu::decode32::Inst32::Mem {
            rt,
            access: MemOp::Ldr | MemOp::Ldrb | MemOp::Ldrh | MemOp::Ldrsb | MemOp::Ldrsh,
            ..
        }) => rt,
        _ => return None,
    };
    if rt < 13 {
        Some(rt)
    } else {
        None
    }
}

#[cfg(test)]
pub(crate) mod harness {
    use crate::cpu::registers::{Registers, Xpsr};
    use crate::cpu::Core;
    use crate::error::HaltCause;
    use crate::jit::native::{exit_reason, NativeCtx};
    use crate::mem::flat::FlatMemory;

    use super::*;

    /// Deterministic xorshift64* (fixed seed: CI-reproducible trials).
    pub struct Rng(u64);

    impl Rng {
        pub fn new() -> Self {
            Self(0x1234_5678_9ABC_DEF1)
        }

        pub fn next_u32(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            self.0 = x;
            (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32
        }
    }

    /// Directed edge values sprinkled into operand regs each trial.
    pub const EDGES: &[u32] = &[
        0x0000_0000,
        0x0000_0001,
        0x0000_0002,
        0x0000_001F,
        0x0000_0020,
        0x0000_0021,
        0x7FFF_FFFF,
        0x8000_0000,
        0x8000_0001,
        0xFFFF_FFFE,
        0xFFFF_FFFF,
        0x00FF_00FF,
        0xFF00_FF00,
        0x5555_5555,
        0xAAAA_AAAA,
    ];

    /// Scratch register file: random plus two edge values at random lanes.
    fn scratch_regs(rng: &mut Rng, trial: u32) -> Registers {
        let mut r = [0u32; 13];
        for slot in r.iter_mut() {
            *slot = rng.next_u32();
        }
        let e = EDGES.len() as u32;
        r[rng.next_u32() as usize % 13] = EDGES[trial as usize % e as usize];
        r[rng.next_u32() as usize % 13] = EDGES[(trial as usize * 7 + 1) % e as usize];
        Registers {
            r,
            sp: rng.next_u32(),
            msp: 0,
            psp: 0,
            lr: rng.next_u32(),
            pc: 0x0800_1000,
        }
    }

    /// Scratch flags: random NZCV/Q/GE lanes, Thumb set, IT zero (native
    /// blocks always run with `itstate == 0`).
    fn scratch_xpsr(rng: &mut Rng) -> Xpsr {
        let mut x = Xpsr::default();
        x.set(0x0100_0000 | (rng.next_u32() & 0xF89F_0000));
        x
    }

    /// Raw emitted bytes for one template (byte-level debugging).
    #[allow(dead_code)]
    pub fn dump_native(op: &TbOp) -> Vec<u8> {
        use super::super::native::{emit_epilogue, emit_prologue, EXIT_FALL};
        let mut ops = super::arch::Assembler::new().unwrap();
        emit_prologue(&mut ops);
        assert!(super::emit_op(&mut ops, op, 0x1000, false));
        emit_epilogue(&mut ops, EXIT_FALL, 0);
        let buf = ops.finalize().unwrap();
        buf.to_vec()
    }

    /// Execute one template natively over scratch state at guest `pc`.
    /// Returns the end state plus the raw exit code (callers assert the
    /// reason: S0 ops always `FALL`, terminators `BRANCH`).
    pub fn run_native(
        op: &TbOp,
        regs: &Registers,
        xpsr: &Xpsr,
        pc: u32,
        unalign_trap: bool,
    ) -> ((Registers, Xpsr), u64) {
        let mut r = *regs;
        let mut x = *xpsr;
        let mut ctx = Box::new(NativeCtx {
            regs: &mut r,
            xpsr: &mut x,
            ..Default::default()
        });
        let block = assemble_single(op, pc, unalign_trap).expect("harness op is templated");
        let code = vioarm_jit_call::call1(block.entry_ptr(), &mut *ctx as *mut NativeCtx as u64);
        ((r, x), code)
    }

    /// Execute one op through the interpreter over identical scratch state
    /// (no bus traffic: S0 ops never fetch or touch memory; `exec` takes
    /// the decoded op directly).
    pub fn run_interp(op: &TbOp, regs: &Registers, xpsr: &Xpsr) -> (Registers, Xpsr) {
        let mem = FlatMemory::with_layout(&[], &[]);
        let mut core = Core::armv7m(mem);
        core.regs = *regs;
        core.xpsr = *xpsr;
        core.itstate = 0;
        match *op {
            TbOp::V16(inst) => {
                core.exec_tb16(0x1000, inst).unwrap();
            }
            TbOp::V32(_) => panic!("S0 harness covers 16-bit ops only"),
        }
        (core.regs, core.xpsr)
    }

    /// Randomized differential: `trials` states per op, each with random
    /// regs/flags plus directed edges; `force` pins `(reg, values)` cycled
    /// by trial (for count operands like register shifts). S0 ops always
    /// fall through (`FALL` exit asserted here).
    pub fn check_one(op: &TbOp, trials: u32, force: &[(u8, &[u32])]) {
        use crate::jit::native::EXIT_FALL;

        assert!(is_templated(op), "harness checks templated ops");
        let mut rng = Rng::new();
        for t in 0..trials {
            let mut regs = scratch_regs(&mut rng, t);
            for (reg, vals) in force {
                regs.r[*reg as usize] = vals[t as usize % vals.len()];
            }
            let xpsr = scratch_xpsr(&mut rng);
            let ((nr, nx), code) = run_native(op, &regs, &xpsr, 0x1000, false);
            assert_eq!(exit_reason(code), EXIT_FALL, "S0 always falls");
            let (ir, ix) = run_interp(op, &regs, &xpsr);
            assert_eq!((nr, nx), (ir, ix), "trial {t} for {op:?}");
        }
    }

    /// Convenience: no forced regs.
    pub fn check(op: &TbOp, trials: u32) {
        check_one(op, trials, &[]);
    }

    // ---- Memory-template harness (P2-3) ----
    //
    // Single ops execute against a real `FlatMemory` (vectors + stack for
    // fault delivery, RAM for data) with a real TLB: the first run
    // side-exits (cold TLB), the harness fills via the shared `fill_tlb`
    // exactly like the driver, and the second run takes the fast path.
    // The interpreter reference runs the same op over an identically
    // initialized bus. Compared: regs, flags, and RAM bytes (fault tests
    // additionally compare exception state: same `exec` path both sides).

    use crate::jit::tlb::{fill_tlb, Tlb};
    use crate::mem::bus::Bus;

    pub(crate) const MEM_RAM: u32 = 0x2000_0000;
    const MEM_VEC: u32 = 0x0000_0000;
    const MEM_SP: u32 = 0x2000_0200;
    const MEM_HANDLER: u32 = 0x2000_0300;

    /// Deterministic bus image: vectors (all faults → handler), stack
    /// area, and patterned RAM.
    pub fn mem_bus() -> FlatMemory {
        let mut bus = FlatMemory::with_layout(&[MEM_VEC, MEM_RAM], &[0x400, 0x1000]);
        bus.write_word(MEM_VEC, MEM_SP).unwrap();
        for vec in [0x08, 0x0C, 0x10, 0x18, 0x2C] {
            bus.write_word(MEM_VEC + vec, MEM_HANDLER | 1).unwrap();
        }
        for i in 0..0x1000u32 {
            let b = (i.wrapping_mul(0x9E37).wrapping_add(i >> 3) & 0xFF) as u8;
            bus.write(MEM_RAM + i, b).unwrap();
        }
        bus
    }

    /// Snapshotted RAM window for comparison.
    pub fn mem_snap(bus: &mut FlatMemory) -> Vec<u8> {
        (0..0x100u32)
            .map(|i| bus.read(MEM_RAM + i).unwrap())
            .collect()
    }

    /// Scratch regs pointed at the harness RAM (sp valid for stacking).
    pub fn mem_regs() -> Registers {
        Registers {
            r: [0u32; 13],
            sp: MEM_SP,
            msp: MEM_SP,
            psp: 0,
            lr: 0,
            pc: MEM_RAM,
        }
    }

    /// Run one memory op natively: cold TLB first (must side-exit with
    /// `MMIO`), fill from the reported address exactly like the driver,
    /// re-run (fast path or second exit). Returns end state + RAM +
    /// `fault_addr` + both exit codes. Miss paths are effect-free by
    /// template construction (scratch-only prefix), so both passes start
    /// from the same initial state.
    pub fn run_mem(
        op: &TbOp,
        regs: &Registers,
        xpsr: &Xpsr,
        pc: u32,
        trap: bool,
        bus: &mut FlatMemory,
        tlb: &mut Tlb,
        gen: &mut u64,
    ) -> (
        (Registers, Xpsr, Vec<u8>),
        u64,
        u32,
        (Registers, Xpsr, Vec<u8>),
        u64,
    ) {
        use super::super::native::{exit_reason, NativeCtx, EXIT_MMIO};
        use super::mem_access;

        let run_once = |bus: &mut FlatMemory, tlb: &mut Tlb, gen: &mut u64| {
            let mut r = *regs;
            let mut x = *xpsr;
            let mut ctx = Box::new(NativeCtx {
                regs: &mut r,
                xpsr: &mut x,
                tlb: &mut *tlb as *mut Tlb,
                privileged: 1,
                write_gen: &mut *gen as *mut u64,
                ..Default::default()
            });
            let block = assemble_single(op, pc, trap).expect("harness op is templated");
            let code =
                vioarm_jit_call::call1(block.entry_ptr(), &mut *ctx as *mut NativeCtx as u64);
            let fault = ctx.fault_addr;
            ((r, x, mem_snap(bus)), code, fault)
        };
        let (st1, code1, fault) = run_once(bus, tlb, gen);
        assert_eq!(exit_reason(code1), EXIT_MMIO, "cold TLB must side-exit");
        if let Some((size, is_write)) = mem_access(op) {
            fill_tlb(tlb, bus, fault, size, is_write, true, 0);
        }
        let (st2, code2, _) = run_once(bus, tlb, gen);
        (st1, code1, fault, st2, code2)
    }

    /// Interpreter reference for one memory op over a twin bus.
    pub fn run_mem_interp(
        op: &TbOp,
        regs: &Registers,
        xpsr: &Xpsr,
        pc: u32,
        bus: FlatMemory,
    ) -> ((Registers, Xpsr, Vec<u8>, u8), Option<HaltCause>) {
        use crate::cpu::Next;

        let mut core = Core::armv7m(bus);
        core.regs = *regs;
        core.xpsr = *xpsr;
        core.itstate = 0;
        let outcome = match *op {
            TbOp::V16(inst) => core.exec_tb16(pc, inst),
            TbOp::V32(inst) => core.exec_tb32(pc, inst),
        };
        let err = match outcome {
            Ok(Next::Advance(a)) => {
                // Commit exactly like the stepper tail (cycles irrelevant
                // here; compared state is_regs/flags/RAM).
                core.regs.pc = a;
                None
            }
            Ok(Next::Stop(c)) | Err(c) => Some(c),
        };
        let ram = mem_snap(core.mem_mut());
        ((core.regs, core.xpsr, ram, core.ipsr), err)
    }
}

#[cfg(test)]
mod tests {
    use super::harness::*;
    use super::*;
    use crate::cpu::registers::{Registers, Xpsr};

    /// Low regs (pinned) + general regs (memory path) + high regs.
    const POOL: &[u8] = &[0, 1, 2, 3, 4, 5, 7, 8, 12];

    fn pool(i: usize) -> u8 {
        POOL[i % POOL.len()]
    }

    #[test]
    fn mov_imm_matches() {
        for (k, rd) in POOL.iter().enumerate() {
            for imm in [0u8, 1, 0x7F, 0x80, 0xFF] {
                let op = TbOp::V16(Inst16::MovImm { rd: *rd, imm });
                check(&op, 64 + k as u32 * 8);
            }
        }
    }

    #[test]
    fn add_sub_reg_match() {
        for (k, rd) in POOL.iter().enumerate() {
            let rn = pool(k + 1);
            let rm = pool(k + 2);
            check(&TbOp::V16(Inst16::AddReg { rd: *rd, rn, rm }), 128);
            check(&TbOp::V16(Inst16::SubReg { rd: *rd, rn, rm }), 128);
        }
    }

    #[test]
    fn add_sub_imm_match() {
        for (k, rd) in POOL.iter().enumerate() {
            let rn = pool(k + 3);
            for imm in [0u8, 1, 7, 0xFF] {
                check(
                    &TbOp::V16(Inst16::AddImm3 {
                        rd: *rd,
                        rn,
                        imm: imm & 7,
                    }),
                    48,
                );
                check(
                    &TbOp::V16(Inst16::SubImm3 {
                        rd: *rd,
                        rn,
                        imm: imm & 7,
                    }),
                    48,
                );
                check(&TbOp::V16(Inst16::AddImm8 { rd: *rd, imm }), 48);
                check(&TbOp::V16(Inst16::SubImm8 { rd: *rd, imm }), 48);
            }
        }
        check(&TbOp::V16(Inst16::CmpImm { rn: 2, imm: 0x5A }), 256);
    }

    #[test]
    fn shift_imm_matches_all_amounts() {
        use crate::cpu::decode16::ShiftImm as K;
        for kind in [K::Lsl, K::Lsr, K::Asr] {
            for amount in 0..32u8 {
                let op = TbOp::V16(Inst16::ShiftImm {
                    rd: 1,
                    rm: 4,
                    kind,
                    amount,
                });
                check(&op, 48);
            }
        }
        // Pinned-destination variant at the edge amounts.
        for amount in [0u8, 1, 31] {
            let op = TbOp::V16(Inst16::ShiftImm {
                rd: 0,
                rm: 2,
                kind: crate::cpu::decode16::ShiftImm::Lsl,
                amount,
            });
            check(&op, 128);
        }
    }

    /// Register-shift counts covering every interpreter class.
    const COUNTS: &[u32] = &[0, 1, 2, 7, 8, 15, 16, 31, 32, 33, 63, 64, 65, 127, 128, 255];

    #[test]
    fn shift_reg_matches() {
        use crate::cpu::decode16::Shift as K;
        for kind in [K::Lsl, K::Lsr, K::Asr, K::Ror] {
            // Value in r4 (memory path), count forced into r5 lane by lane.
            let op = TbOp::V16(Inst16::ShiftReg {
                rd: 1,
                rn: 4,
                rm: 5,
                kind,
            });
            check_one(&op, 64, &[(5, COUNTS)]);
            // Pinned-lane variant.
            let op = TbOp::V16(Inst16::ShiftReg {
                rd: 0,
                rn: 2,
                rm: 3,
                kind,
            });
            check_one(&op, 64, &[(3, COUNTS)]);
        }
    }

    #[test]
    fn alu_ops_match() {
        use crate::cpu::decode16::AluOp as O;
        let ops = [
            O::And,
            O::Eor,
            O::Orr,
            O::Bic,
            O::Tst,
            O::Cmp,
            O::Cmn,
            O::Mul,
            O::Mvn,
            O::Neg,
            O::Adc,
            O::Sbc,
            O::Lsl,
            O::Lsr,
            O::Asr,
            O::Ror,
        ];
        for (k, op) in ops.iter().enumerate() {
            let rd = pool(k);
            let rm = pool(k + 4);
            let inst = TbOp::V16(Inst16::Alu { rd, rm, op: *op });
            // Shift/rotate-by-register ops take counts from Rm: force the
            // class edges into the count lane.
            let force: &[(u8, &[u32])] = match op {
                O::Lsl | O::Lsr | O::Asr | O::Ror => &[(rm, COUNTS)],
                _ => &[],
            };
            check_one(&inst, 128, force);
        }
    }

    #[test]
    fn special_cmp_matches() {
        let op = TbOp::V16(Inst16::SpecialCmp { rn: 8, rm: 12 });
        check(&op, 256);
    }

    #[test]
    fn untemplated_ops_bail() {
        // System ops, PC forms, register/table/32-bit branches, and 32-bit
        // ops have no template: the emitter reports false and the block
        // stays interpreted.
        let cases = [
            TbOp::V16(Inst16::Svc { imm: 0 }),
            TbOp::V16(Inst16::Bkpt { imm: 0 }),
            TbOp::V16(Inst16::MovImm { rd: 15, imm: 1 }),
            TbOp::V16(Inst16::AddReg {
                rd: 0,
                rn: 15,
                rm: 1,
            }),
            TbOp::V16(Inst16::SpecialCmp { rn: 15, rm: 0 }),
            TbOp::V16(Inst16::BranchCond { cond: 14, off: 4 }),
            TbOp::V16(Inst16::CbzCbnz {
                rn: 15,
                off: 4,
                nonzero: false,
            }),
            TbOp::V16(Inst16::Bx { rm: 14 }),
            TbOp::V16(Inst16::Pop {
                regs: 0,
                load_pc: true,
            }),
            TbOp::V32(crate::cpu::decode32::Inst32::Bl { off: 4 }),
        ];
        for op in &cases {
            assert!(!is_templated(op), "{op:?} must bail");
        }
    }

    /// Branches touch no state: end state must equal input state exactly,
    /// and the exit must carry `BRANCH` + the resolved PC.
    fn check_branch(op: &TbOp, regs: &Registers, xpsr: Xpsr, pc: u32, want_pc: u32) {
        use crate::jit::native::{exit_pc, exit_reason, EXIT_BRANCH};

        assert!(is_templated(op));
        let ((nr, nx), code) = run_native(op, regs, &xpsr, pc, false);
        assert_eq!(exit_reason(code), EXIT_BRANCH);
        assert_eq!(exit_pc(code), want_pc, "{op:?}");
        assert_eq!((nr, nx), (*regs, xpsr), "branches are stateless");
    }

    fn blank_regs() -> Registers {
        Registers {
            r: [
                0x1111_1111,
                0x2222_2222,
                0x3333_3333,
                0x4444_4444,
                0x5555_5555,
                0x6666_6666,
                0x7777_7777,
                0x8888_8888,
                0x9999_9999,
                0xAAAA_AAAA,
                0xBBBB_BBBB,
                0xCCCC_CCCC,
                0xDDDD_DDDD,
            ],
            sp: 0x2000_1000,
            msp: 0x2000_1000,
            psp: 0x2000_2000,
            lr: 0x0800_2001,
            pc: 0x0800_1000,
        }
    }

    fn flags_nzcv(n: bool, z: bool, c: bool, v: bool) -> Xpsr {
        let mut x = Xpsr::default();
        x.set(
            0x0100_0000
                | (u32::from(n) << 31)
                | (u32::from(z) << 30)
                | (u32::from(c) << 29)
                | (u32::from(v) << 28),
        );
        x
    }

    #[test]
    fn branch_unconditional_matches() {
        let regs = blank_regs();
        let xpsr = flags_nzcv(true, false, true, false);
        // B self at 0x1000 (off -4) and B forward (off +6 -> 0x100A).
        check_branch(
            &TbOp::V16(Inst16::Branch { off: -4 }),
            &regs,
            xpsr,
            0x1000,
            0x1000,
        );
        check_branch(
            &TbOp::V16(Inst16::Branch { off: 6 }),
            &regs,
            xpsr,
            0x1000,
            0x100A,
        );
    }

    #[test]
    fn branch_cond_matches_all_conds_both_ways() {
        // (cond, take-flags, fall-flags) at pc 0x1000, off +6.
        let cases: &[(u8, Xpsr, Xpsr)] = &[
            (
                0,
                flags_nzcv(false, true, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                1,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, true, false, false),
            ),
            (
                2,
                flags_nzcv(false, false, true, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                3,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, false, true, false),
            ),
            (
                4,
                flags_nzcv(true, false, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                5,
                flags_nzcv(false, false, false, false),
                flags_nzcv(true, false, false, false),
            ),
            (
                6,
                flags_nzcv(false, false, false, true),
                flags_nzcv(false, false, false, false),
            ),
            (
                7,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, false, false, true),
            ),
            (
                8,
                flags_nzcv(false, false, true, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                8,
                flags_nzcv(false, false, true, false),
                flags_nzcv(false, true, true, false),
            ),
            (
                9,
                flags_nzcv(false, true, true, false),
                flags_nzcv(false, false, true, false),
            ),
            (
                10,
                flags_nzcv(false, false, false, false),
                flags_nzcv(true, false, false, false),
            ),
            (
                11,
                flags_nzcv(true, false, false, false),
                flags_nzcv(false, false, false, false),
            ),
            (
                11,
                flags_nzcv(false, false, false, true),
                flags_nzcv(true, false, false, true),
            ),
            (
                12,
                flags_nzcv(false, false, false, false),
                flags_nzcv(false, true, false, false),
            ),
            (
                13,
                flags_nzcv(false, true, false, false),
                flags_nzcv(false, false, false, false),
            ),
        ];
        let regs = blank_regs();
        for (cond, take, fall) in cases {
            let op = TbOp::V16(Inst16::BranchCond {
                cond: *cond,
                off: 6,
            });
            check_branch(&op, &regs, *take, 0x1000, 0x100A);
            check_branch(&op, &regs, *fall, 0x1000, 0x1002);
        }
    }

    // ---- Memory templates (P2-3) ----

    use super::harness::{mem_bus, mem_regs, run_mem, run_mem_interp, MEM_RAM};
    use crate::jit::native::{exit_reason, EXIT_FALL, EXIT_MMIO};
    use crate::jit::tlb::Tlb;
    use crate::mem::bus::Bus;
    use crate::mem::flat::FlatMemory;

    /// Fresh harness triple per case (TLB + generation start cold/zero).
    fn mem_setup() -> (FlatMemory, Tlb, u64) {
        (mem_bus(), Tlb::new(), 0)
    }

    #[test]
    fn mem_word_imm_fast_path() {
        // STR then LDR word, imm offsets; fault_addr + values checked.
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x100;
        regs.r[2] = 0xDEAD_BEEF;
        let xpsr = Xpsr::default();
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let str_op = TbOp::V16(Inst16::LsImm {
            rt: 2,
            rn: 1,
            imm: 12,
            kind: crate::cpu::decode16::LsImm::Str,
        });
        let (st1, c1, fault, _, c2) = run_mem(
            &str_op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(fault, MEM_RAM + 0x10C);
        assert_eq!((st1.0, st1.1), (regs, xpsr), "miss is effect-free");
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert!(gen > 0, "fast store bumps the generation");
        let ldr_op = TbOp::V16(Inst16::LsImm {
            rt: 0,
            rn: 1,
            imm: 12,
            kind: crate::cpu::decode16::LsImm::Ldr,
        });
        // Fresh TLB (the STR above warmed the shared one — same page).
        let mut tlb2 = Tlb::new();
        let (_, _, _, (nr2, _, _), c4) = run_mem(
            &ldr_op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb2, &mut gen,
        );
        assert_eq!(exit_reason(c4), EXIT_FALL);
        assert_eq!(nr2.r[0], 0xDEAD_BEEF, "stored word reads back");
        // Interpreter parity on a twin bus (fast path must equal it).
        let (mut bus2, _, _) = mem_setup();
        bus2.write_word(MEM_RAM + 0x10C, 0xDEAD_BEEF).unwrap();
        let ((ir, _, _, _), err) = run_mem_interp(&ldr_op, &regs, &xpsr, 0x1000, bus2);
        assert!(err.is_none());
        assert_eq!(ir.r[0], nr2.r[0]);
    }

    #[test]
    fn mem_unmapped_side_exits_and_faults_identically() {
        // Unmapped data address: native side-exits (fill refuses), the
        // interpreter takes the fault; miss paths must be effect-free.
        let mut regs = mem_regs();
        regs.r[1] = 0x4000_0000;
        let xpsr = Xpsr::default();
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let op = TbOp::V16(Inst16::LsImm {
            rt: 0,
            rn: 1,
            imm: 0,
            kind: crate::cpu::decode16::LsImm::Ldr,
        });
        let (st1, c1, fault, st2, c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(fault, 0x4000_0000);
        assert_eq!(exit_reason(c2), EXIT_MMIO, "unfillable stays exited");
        assert_eq!((st1.0, st1.1), (regs, xpsr), "miss is effect-free");
        assert_eq!((st2.0, st2.1), (regs, xpsr), "miss is effect-free");
        let (bus2, _, _) = mem_setup();
        let ((ir, ix, _, ipsr), err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
        assert!(err.is_none(), "fault delivers via Advance, not Err");
        assert_eq!(ipsr, 3, "HardFault taken");
        assert_eq!(ix, xpsr, "fault preserves flags word shape");
    }

    /// Fast-path differential for one memory op: miss→fill→hit must equal
    /// the interpreter over a twin bus (regs, flags, RAM). `seed` prepares
    /// both buses identically (test data beyond the image pattern).
    fn check_mem_fast(
        op: &TbOp,
        regs: &Registers,
        pc: u32,
        trap: bool,
        seed: impl Fn(&mut FlatMemory),
    ) {
        use crate::jit::native::{exit_reason, EXIT_FALL, EXIT_MMIO};

        let xpsr = Xpsr::default();
        let (mut bus, mut tlb, mut gen) = mem_setup();
        seed(&mut bus);
        let (_, c1, _, st2, c2) = run_mem(op, regs, &xpsr, pc, trap, &mut bus, &mut tlb, &mut gen);
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(exit_reason(c2), EXIT_FALL, "{op:?} must go fast");
        let (mut bus2, _, _) = mem_setup();
        seed(&mut bus2);
        let ((ir, ix, ram_i, _), err) = run_mem_interp(op, regs, &xpsr, pc, bus2);
        assert!(err.is_none(), "{op:?} interpreter must not fault");
        // `pc` is driver-managed (no template reads `regs.pc`; literals
        // use the emit-time pc), so normalize it out of the comparison.
        let (mut nr, nx, nram) = st2;
        nr.pc = ir.pc;
        assert_eq!((nr, nx, nram), (ir, ix, ram_i), "{op:?}");
    }

    #[test]
    fn mem_sizes_and_sign_extension() {
        use crate::cpu::decode16::{LsImm, LsReg};
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x200;
        regs.r[2] = 0xA5A5_F00D;
        // Stores then loads across widths (STR/STRB/STRH + LDR/LDRB/LDRH).
        for (rt, imm, kind) in [
            (2u8, 0x00u16, LsImm::Str),
            (2u8, 0x10u16, LsImm::Strb),
            (2u8, 0x20u16, LsImm::Strh),
        ] {
            let op = TbOp::V16(Inst16::LsImm {
                rt,
                rn: 1,
                imm: imm as usize,
                kind,
            });
            check_mem_fast(&op, &regs, 0x1000, false, |_| {});
        }
        for (rt, imm, kind, want) in [
            (0u8, 0x00u16, LsImm::Ldr, 0xA5A5_F00Du32),
            (0u8, 0x10u16, LsImm::Ldrb, 0x0000_000Du32),
            (0u8, 0x20u16, LsImm::Ldrh, 0x0000_F00Du32),
        ] {
            // Seed the slot the matching store above wrote.
            let seed = move |bus: &mut FlatMemory| {
                bus.write_word(MEM_RAM + 0x200 + imm as u32, 0xA5A5_F00D)
                    .unwrap();
            };
            let op = TbOp::V16(Inst16::LsImm {
                rt,
                rn: 1,
                imm: imm as usize,
                kind,
            });
            check_mem_fast(&op, &regs, 0x1000, false, seed);
            // Spot-check the decoded value, not just parity.
            let (mut bus, mut tlb, mut gen) = mem_setup();
            seed(&mut bus);
            let xpsr = Xpsr::default();
            let (_, _, _, (nr, _, _), _) = run_mem(
                &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
            );
            assert_eq!(nr.r[rt as usize], want, "{op:?}");
        }
        // Register-offset + sign-extending loads.
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x300;
        regs.r[2] = 8;
        for kind in [LsReg::Ldrsb, LsReg::Ldrsh, LsReg::Ldrb] {
            let op = TbOp::V16(Inst16::LsReg {
                rt: 0,
                rn: 1,
                rm: 2,
                kind,
            });
            // Seed 0x80/0xFF80 at the effective address (base + 8).
            check_mem_fast(&op, &regs, 0x1000, false, |bus| {
                bus.write(MEM_RAM + 0x308, 0x80).unwrap();
                bus.write_word(MEM_RAM + 0x308, 0xFF80).unwrap();
            });
        }
        // Sign-extension value assertions (parity above already proves
        // equality; these pin the architected values).
        let op = TbOp::V16(Inst16::LsReg {
            rt: 0,
            rn: 1,
            rm: 2,
            kind: LsReg::Ldrsb,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write(MEM_RAM + 0x308, 0x80).unwrap();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), _) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(nr.r[0], 0xFFFF_FF80, "sign extends");
    }

    #[test]
    fn mem_sp_and_literal() {
        use crate::cpu::decode16::LsImm;
        // SP-relative store/load roundtrip (SP is never pinned: always the
        // memory path, always fresh).
        let mut regs = mem_regs();
        regs.sp = MEM_RAM + 0x400;
        regs.msp = MEM_RAM + 0x400;
        regs.r[0] = 0x1234_5678;
        let str_op = TbOp::V16(Inst16::LsSp {
            rt: 0,
            imm: 16,
            is_load: false,
        });
        check_mem_fast(&str_op, &regs, 0x1000, false, |_| {});
        let ldr_op = TbOp::V16(Inst16::LsSp {
            rt: 1,
            imm: 16,
            is_load: true,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        bus.write_word(MEM_RAM + 0x410, 0x1234_5678).unwrap();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), c2) = run_mem(
            &ldr_op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert_eq!(nr.r[1], 0x1234_5678);
        let _ = LsImm::Ldr;
        // PC-relative literal load (static base from the emit-time pc,
        // placed in RAM so the page maps).
        let lit_pc = MEM_RAM + 0x100;
        let lit_op = TbOp::V16(Inst16::LdrLiteral { rt: 0, imm: 8 });
        check_mem_fast(&lit_op, &regs, lit_pc, false, |bus| {
            // Base = Align(lit_pc + 4, 4); literal at +8.
            let base = (lit_pc.wrapping_add(4)) & !3;
            bus.write_word(base + 8, 0xCAFEBABE).unwrap();
        });
        // Spot-check the decoded literal value.
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let base = (lit_pc.wrapping_add(4)) & !3;
        bus.write_word(base + 8, 0xCAFEBABE).unwrap();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), c2) = run_mem(
            &lit_op, &regs, &xpsr, lit_pc, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c2), EXIT_FALL);
        assert_eq!(nr.r[0], 0xCAFEBABE);
    }

    #[test]
    fn mem_pre_post_writeback_and_aliasing() {
        use crate::cpu::decode32::{Inst32, MemOp};
        // Post-indexed store with writeback, then rt==rn aliasing load:
        // access-then-writeback order must match the interpreter (the
        // loaded value lands first, the base update overwrites it).
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x500;
        regs.r[2] = 0x1111_2222;
        let post = TbOp::V32(Inst32::Mem {
            access: MemOp::Str,
            rt: 2,
            rn: 1,
            off: 4,
            rm: None,
            shift: 0,
            add: true,
            index: false,
            writeback: true,
            unprivileged: false,
        });
        check_mem_fast(&post, &regs, 0x1000, false, |_| {});
        // Verify the writeback actually advanced r1 (not just parity).
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let xpsr = Xpsr::default();
        let (_, _, _, (nr, _, _), _) = run_mem(
            &post, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(nr.r[1], MEM_RAM + 0x504);
        // rt == rn aliasing load with writeback.
        let mut regs = mem_regs();
        regs.r[0] = MEM_RAM + 0x600;
        let alias = TbOp::V32(Inst32::Mem {
            access: MemOp::Ldr,
            rt: 0,
            rn: 0,
            off: 4,
            rm: None,
            shift: 0,
            add: true,
            index: true,
            writeback: true,
            unprivileged: false,
        });
        check_mem_fast(&alias, &regs, 0x1000, false, |_| {});
        // Pre-indexed with register offset + shift, unprivileged flag set
        // (ignored exactly like the interpreter — fidelity match).
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x700;
        regs.r[2] = 0x10;
        let regoff = TbOp::V32(Inst32::Mem {
            access: MemOp::Ldr,
            rt: 0,
            rn: 1,
            off: 0,
            rm: Some(2),
            shift: 2,
            add: true,
            index: true,
            writeback: true,
            unprivileged: true,
        });
        check_mem_fast(&regoff, &regs, 0x1000, false, |_| {});
    }

    #[test]
    fn mem_cross_page_side_exits() {
        use crate::cpu::decode16::LsImm;
        // Word straddling a page boundary (both pages mapped): the fast
        // path must still refuse (host adjacency unproven) and the
        // interpreter reference must agree on values.
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0xFFD;
        regs.r[2] = 0xAABB_CCDD;
        let xpsr = Xpsr::default();
        let op = TbOp::V16(Inst16::LsImm {
            rt: 2,
            rn: 1,
            imm: 0,
            kind: LsImm::Str,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let (_, c1, _, _, c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(exit_reason(c2), EXIT_MMIO, "page-crossing never goes fast");
        // The interpreter performs it fine — same bytes both sides would
        // see (proves the exit, not a fault).
        let (bus2, _, _) = mem_setup();
        let (_, err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
        assert!(err.is_none(), "cross-page store succeeds interpreted");
    }

    #[test]
    fn mem_unaligned_trap_modeled() {
        use crate::cpu::decode16::LsImm;
        // Halfword at an odd address: with the trap baked, side exit;
        // without, the x86 host performs it natively (AArch64 always
        // exits — conservative but value-identical via the interpreter).
        let mut regs = mem_regs();
        regs.r[1] = MEM_RAM + 0x101;
        regs.r[2] = 0xBEEF;
        let op = TbOp::V16(Inst16::LsImm {
            rt: 2,
            rn: 1,
            imm: 0,
            kind: LsImm::Strh,
        });
        let (mut bus, mut tlb, mut gen) = mem_setup();
        let xpsr = Xpsr::default();
        let (_, c1, _, _, c2) = run_mem(
            &op, &regs, &xpsr, 0x1000, true, &mut bus, &mut tlb, &mut gen,
        );
        assert_eq!(exit_reason(c1), EXIT_MMIO);
        assert_eq!(exit_reason(c2), EXIT_MMIO, "trapped unaligned exits");
        #[cfg(target_arch = "x86_64")]
        {
            let (mut bus, mut tlb, mut gen) = mem_setup();
            let (_, c1, _, (nr, _, _), c2) = run_mem(
                &op, &regs, &xpsr, 0x1000, false, &mut bus, &mut tlb, &mut gen,
            );
            assert_eq!(exit_reason(c1), EXIT_MMIO);
            assert_eq!(exit_reason(c2), EXIT_FALL, "x64 performs in-page unaligned");
            let (bus2, _, _) = mem_setup();
            let ((ir, _, _, _), err) = run_mem_interp(&op, &regs, &xpsr, 0x1000, bus2);
            assert!(err.is_none());
            assert_eq!(nr.r[2], ir.r[2]);
        }
    }

    #[test]
    fn cbz_cbnz_match_both_ways() {
        let regs = blank_regs();
        let xpsr = flags_nzcv(false, false, true, true);
        // CBZ r2 (r2 != 0 in blank_regs): falls.
        check_branch(
            &TbOp::V16(Inst16::CbzCbnz {
                rn: 2,
                off: 8,
                nonzero: false,
            }),
            &regs,
            xpsr,
            0x1000,
            0x1002,
        );
        // CBNZ r2: takes.
        check_branch(
            &TbOp::V16(Inst16::CbzCbnz {
                rn: 2,
                off: 8,
                nonzero: true,
            }),
            &regs,
            xpsr,
            0x1000,
            0x100C,
        );
        // CBZ r12 == 0 in blank_regs? r12 = 0xDDDD_DDDD (nonzero). Use r0
        // via a zeroed copy for the taken-CBZ direction.
        let mut zeroed = regs;
        zeroed.r[0] = 0;
        check_branch(
            &TbOp::V16(Inst16::CbzCbnz {
                rn: 0,
                off: 8,
                nonzero: false,
            }),
            &zeroed,
            xpsr,
            0x1000,
            0x100C,
        );
    }
}
