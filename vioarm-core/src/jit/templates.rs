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
            _ => false,
        },
        TbOp::V32(_) => false,
    }
}

/// Emit one templated op at guest `pc` into an arch assembler. Returns
/// `false` for untemplated ops (the block then stays interpreted).
pub fn emit_op(ops: &mut arch::Assembler, op: &TbOp, pc: u32) -> bool {
    if !is_templated(op) {
        return false;
    }
    arch::emit_op(ops, op, pc)
}

/// Assemble prologue + one template + epilogue for harness runs. S0 ops
/// fall through (`FALL`); terminators exit with their branch code.
pub(crate) fn assemble_single(
    op: &TbOp,
    pc: u32,
) -> Result<super::native::NativeBlock, super::EmitError> {
    arch::assemble_single(op, pc)
}

/// Emit a whole block natively: prologue, one template per op in order,
/// then the terminator's own epilogue — or, when the block ends in plain
/// ops, a `FALL` epilogue to the fallthrough address. Bails with
/// [`EmitError::Untemplated`] on the first untemplated op (the block then
/// stays interpreted); the builder guarantees terminators only appear
/// last, so anything after one would be unreachable anyway.
pub fn emit_block(body: &super::TbBody) -> Result<super::native::NativeBlock, super::EmitError> {
    use super::builder::op_len;
    use super::native::{emit_epilogue, emit_prologue, NativeBlock, EXIT_FALL};
    use dynasmrt::AssemblyOffset;

    let mut ops = arch::Assembler::new()?;
    emit_prologue(&mut ops);
    let mut pc = body.pc;
    let mut terminated = false;
    for op in &body.ops {
        if !emit_op(&mut ops, op, pc) {
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
    ops.finalize()
        .map(|buf| NativeBlock::new(buf, AssemblyOffset(0)))
        .map_err(|_| super::EmitError::Finalize)
}

#[cfg(test)]
pub(crate) mod harness {
    use crate::cpu::registers::{Registers, Xpsr};
    use crate::cpu::Core;
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
        assert!(super::emit_op(&mut ops, op, 0x1000));
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
    ) -> ((Registers, Xpsr), u64) {
        let mut r = *regs;
        let mut x = *xpsr;
        let mut ctx = Box::new(NativeCtx {
            regs: &mut r,
            xpsr: &mut x,
        });
        let block = assemble_single(op, pc).expect("harness op is templated");
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
            let ((nr, nx), code) = run_native(op, &regs, &xpsr, 0x1000);
            assert_eq!(exit_reason(code), EXIT_FALL, "S0 always falls");
            let (ir, ix) = run_interp(op, &regs, &xpsr);
            assert_eq!((nr, nx), (ir, ix), "trial {t} for {op:?}");
        }
    }

    /// Convenience: no forced regs.
    pub fn check(op: &TbOp, trials: u32) {
        check_one(op, trials, &[]);
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
        let ((nr, nx), code) = run_native(op, regs, &xpsr, pc);
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
