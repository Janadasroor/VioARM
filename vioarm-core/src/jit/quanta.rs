//! Quanta driver (P1-5): synchronized time model over TB execution.
//!
//! [`Machine::run_tb_quantum`] runs translated blocks up to an op budget
//! (E2, default [`DEFAULT_QUANTUM_OPS`]), then performs ONE boundary: a
//! single device flush for the whole quantum plus a single pending-IRQ
//! delivery through the existing `enter_exception` path (E1). This is the
//! same never-early contract as M11-coarse (scans see pre-flush latch
//! state; lag `<=` quantum): timing-sensitive assertions belong on the
//! accurate stepper, never here.
//!
//! Phase-1 mappings of the roadmap's exit list (E1):
//! - (a) "next TB missing": the builder only refuses where the stepper owns
//!   the semantics (ITSTATE bodies, odd PC, unmapped holes), so a refusal
//!   runs exactly one accurate `step` and the quantum continues.
//! - (b) budget spent: blocks are atomic, so a quantum may overshoot by at
//!   most one block (`< MAX_TB_INSNS` ops).
//! - (c) side exits: exception entry/return divert the PC, which ends blocks
//!   by construction (E4 — reused, never reimplemented); `WFI`/`WFE` sleep
//!   (E3) naps one accurate sleeping cycle per quantum and exits: no
//!   spinning translated idle loops.
//! - (d) MMIO touch: needs no exit in Phase 1 — ops execute through the
//!   shared executor with identical side effects at identical op boundaries,
//!   so MMIO values are exact. (d) becomes a real exit with the emitter in
//!   Phase 2, where translated code can no longer call into devices.
//! - Pending interrupts are never polled mid-burst (E5): same as
//!   back-to-back steps, masked lines stay latched, and `just_returned`
//!   skips the boundary scan so the restored instruction runs first.
//! - `cyccnt` stays EXACT in Phase 1 (`commit_step` per op); the D7
//!   approximation arrives with the emitter (E6 is Phase-2 work).
//!
//! Compatibility: the driver needs the accurate time model —
//! `fast_quantum` must be `0` (debug-asserted); combining both batching
//! schemes would double-tick devices. A halted quantum still flushes its
//! consumed cycles first (mirrors `run`'s settle-before-`Err` contract), but
//! takes no boundary exception. Mid-quantum `SYSRESETREQ` is served by the
//! fallback `step`'s own reset check, or at the next quantum start.

use super::runner::TbRunner;
use crate::error::HaltCause;
use crate::machine::Machine;
use crate::mem::PeripheralBus;

/// Default quantum: retired-op budget per [`Machine::run_tb_quantum`] (E2).
pub const DEFAULT_QUANTUM_OPS: u64 = 1024;

/// Why a quantum stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QuantumEnd {
    /// Op budget spent (modulo one atomic trailing block).
    Budget,
    /// The core is asleep with nothing pending (one sleep cycle counted).
    Sleep,
}

/// What one [`Machine::run_tb_quantum`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QuantumOutcome {
    /// Ops retired from blocks plus one per fallback step.
    pub ops: u64,
    /// `cyccnt` consumed (== device cycles flushed).
    pub cycles: u64,
    /// Why the quantum stopped.
    pub end: QuantumEnd,
}

impl<M: PeripheralBus> Machine<M> {
    /// Run translated blocks to `budget` retired ops (the JIT analogue
    /// of [`Machine::run`], X6): quanta of [`DEFAULT_QUANTUM_OPS`] until
    /// the budget retires or a halt propagates (flushed first, like
    /// `run`'s settle-before-`Err`). Always ends with
    /// [`HaltCause::CycleBudgetExhausted`] when nothing halts first, so
    /// CLI/reporting code matches on the same halt contract.
    ///
    /// # Errors
    ///
    /// Propagates halts exactly as the stepper reports them.
    pub fn run_tb(&mut self, runner: &mut TbRunner, budget: u64) -> Result<(), HaltCause> {
        let mut ops = 0u64;
        while ops < budget {
            // Cap each quantum at the remainder so small budgets don't
            // overshoot by a full 1024-op quantum (blocks stay atomic —
            // overshoot is at most one trailing block, same as before).
            let cap = (budget - ops).clamp(1, DEFAULT_QUANTUM_OPS);
            ops += self.run_tb_quantum(runner, cap)?.ops;
        }
        Err(HaltCause::CycleBudgetExhausted)
    }

    /// Step one translation block and report every retired instruction's
    /// PC (X2: GDB single-step in JIT mode steps translated blocks but
    /// reports per-insn stops). Falls back to one accurate step where no
    /// block runs (sleeping core, MPU refusal, builder refusal, hooked
    /// ranges — all sound single-step owners). The PCs decode forward
    /// from the entry PC over the retired op count (P2-6 exit rules make
    /// that walk exact: stores and branches seal, side exits commit the
    /// prefix plus the faulting op). Never empty on success.
    ///
    /// # Errors
    ///
    /// Propagates halts exactly as the stepper reports them.
    pub fn step_tb_block(&mut self, runner: &mut TbRunner) -> Result<Vec<u32>, HaltCause> {
        let start = self.core.regs.pc;
        let Some(outcome) = runner.run_block(&mut self.core)? else {
            self.step()?;
            return Ok(vec![start]);
        };
        let mut pcs = Vec::with_capacity(outcome.ops);
        let mut pc = start;
        let isa = self.core.isa;
        for _ in 0..outcome.ops {
            pcs.push(pc);
            // Validated bytes decode (entry validation + store seals);
            // break defensively rather than over-report past a hole.
            let Some((_, len)) = super::builder::fetch_op(self.core.mem_mut(), pc, isa) else {
                break;
            };
            pc = pc.wrapping_add(len);
        }
        if pcs.is_empty() {
            pcs.push(start);
        }
        Ok(pcs)
    }

    /// Run translated blocks up to `max_ops` retired ops, then one boundary
    /// (device flush + single IRQ delivery). See the module docs for the
    /// time model. Requires `fast_quantum() == 0`.
    ///
    /// # Errors
    ///
    /// Propagates halts exactly as the stepper reports them (after flushing
    /// the cycles consumed so far).
    pub fn run_tb_quantum(
        &mut self,
        runner: &mut TbRunner,
        max_ops: u64,
    ) -> Result<QuantumOutcome, HaltCause> {
        debug_assert_eq!(
            self.fast_quantum(),
            0,
            "JIT quanta needs the accurate time model (fast_quantum == 0)"
        );
        if self.core.mem().reset_requested() {
            self.reset()?;
        }
        let start = self.core.cyccnt;
        let mut ops = 0u64;
        let end = self.burst_tb(runner, max_ops, &mut ops);
        let errored = end.is_err();
        // Boundary (E1/E5): one delivery max, skipped when halted and when
        // an exception return still owes its restored instruction.
        if !errored {
            if std::mem::take(&mut self.core.just_returned) {
                // The restored instruction runs first next quantum.
            } else if !self.core.sleeping {
                if let Some(n) = self.scan_pending() {
                    self.deliver(n)?;
                }
            }
        }
        let cycles = self.core.cyccnt.wrapping_sub(start);
        self.core.mem().tick_by(cycles);
        end.map(|end| QuantumOutcome { ops, cycles, end })
    }

    /// The execution burst: chained blocks, accurate fallback steps, and
    /// sleep naps until the budget spends or sleep persists.
    fn burst_tb(
        &mut self,
        runner: &mut TbRunner,
        max_ops: u64,
        ops: &mut u64,
    ) -> Result<QuantumEnd, HaltCause> {
        loop {
            if *ops >= max_ops {
                return Ok(QuantumEnd::Budget);
            }
            if self.core.sleeping {
                match self.scan_pending() {
                    // Woke: deliver and continue the burst (the entry is not
                    // a retired op, so the budget does not advance — forward
                    // progress is guaranteed by acknowledge consuming it).
                    Some(n) => self.deliver(n)?,
                    None => {
                        // Naps one accurate sleeping cycle, mirroring
                        // `run_instruction`'s asleep accounting (E3).
                        self.core.cycles += 1;
                        self.core.cyccnt += 1;
                        return Ok(QuantumEnd::Sleep);
                    }
                }
                continue;
            }
            // Native chaining (P2-4) stops exactly at the remainder: the
            // quantum overshoot stays one entered block (same atomicity as
            // before — chains never cross the budget line).
            runner.set_chain_budgets(max_ops.saturating_sub(*ops).max(1), max_ops.max(1));
            match runner.run_block(&mut self.core)? {
                Some(outcome) => *ops += outcome.ops as u64,
                // Refusal: one accurate step (faults, hooks, sleep handling
                // identical to the stepper), then continue the burst.
                None => {
                    self.step()?;
                    *ops += 1;
                }
            }
        }
    }

    /// The pending-exception scan with the core's live masks.
    fn scan_pending(&mut self) -> Option<u8> {
        let (primask, faultmask, basepri, ipsr) = (
            self.core.primask,
            self.core.faultmask,
            self.core.basepri,
            self.core.ipsr,
        );
        self.core
            .mem()
            .next_exception(primask, faultmask, basepri, ipsr)
    }

    /// Deliver one exception through the existing path (E1): entry,
    /// acknowledge, and the entry cycle — all shared with `step`.
    fn deliver(&mut self, n: u8) -> Result<(), HaltCause> {
        self.core.enter_exception(n)?;
        self.core.mem().acknowledge_exception(n);
        self.core.cycles += 1;
        self.core.cyccnt += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::error::BusError;
    use crate::jit::runner::TbRunner;
    use crate::mem::bus::{Bus, MpuAccess};

    use super::*;

    /// Test bus: vector RAM at `0x0`, code RAM at `0x1000`, stack RAM at
    /// `0x2000_0000`, a raisable IRQ line, and a device-tick counter (so
    /// flush exactness is observable).
    struct TestBus {
        vectors: Vec<u8>,
        code: Vec<u8>,
        stack: Vec<u8>,
        pending: Option<u8>,
        ticks: u64,
        writes: u64,
    }

    impl TestBus {
        fn new(code_halves: &[u16]) -> Self {
            let mut bus = Self {
                vectors: vec![0; 0x400],
                code: vec![0; 0x1000],
                stack: vec![0; 0x100],
                pending: None,
                ticks: 0,
                writes: 0,
            };
            for (i, half) in code_halves.iter().enumerate() {
                let [lo, hi] = half.to_le_bytes();
                bus.code[i * 2] = lo;
                bus.code[i * 2 + 1] = hi;
            }
            // Initial SP + all-zero vectors; tests poke what they need.
            bus.write_word(0x0, 0x2000_0100).unwrap();
            bus
        }

        fn raise(&mut self, n: u8) {
            self.pending = Some(n);
        }
    }

    impl Bus for TestBus {
        fn read(&mut self, addr: u32) -> Result<u8, BusError> {
            let (mem, base) = match addr {
                0x0000_0000..=0x0000_03FF => (&self.vectors, 0x0000_0000),
                0x0000_1000..=0x0000_1FFF => (&self.code, 0x0000_1000),
                0x2000_0000..=0x2000_00FF => (&self.stack, 0x2000_0000),
                _ => return Err(BusError::Reserved),
            };
            Ok(mem[(addr - base) as usize])
        }

        fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
            let (mem, base) = match addr {
                0x0000_0000..=0x0000_03FF => (&mut self.vectors, 0x0000_0000),
                0x0000_1000..=0x0000_1FFF => (&mut self.code, 0x0000_1000),
                0x2000_0000..=0x2000_00FF => (&mut self.stack, 0x2000_0000),
                _ => return Err(BusError::Reserved),
            };
            mem[(addr - base) as usize] = val;
            self.writes = self.writes.wrapping_add(1);
            Ok(())
        }
    }

    impl PeripheralBus for TestBus {
        fn next_exception(
            &mut self,
            primask: bool,
            faultmask: bool,
            _basepri: u8,
            _current_ipsr: u8,
        ) -> Option<u8> {
            // Simplified mask model (Thread-mode tests only): a masked line
            // stays latched. The real NVIC honors full priority grouping.
            if primask || faultmask {
                None
            } else {
                self.pending
            }
        }

        fn acknowledge_exception(&mut self, _n: u8) {
            self.pending = None;
        }

        fn tick_cycle(&mut self) {
            self.ticks += 1;
        }

        fn mpu_check(
            &mut self,
            _addr: u32,
            _access: MpuAccess,
            _privileged: bool,
            _exception: u8,
        ) -> bool {
            true
        }

        fn config_epoch(&self) -> Option<u64> {
            Some(0)
        }

        fn write_epoch(&self) -> Option<u64> {
            Some(self.writes)
        }
    }

    const NOP: u16 = 0xBF00;
    const B_SELF: u16 = 0xE7FE;
    const WFI: u16 = 0xBF30;

    fn machine_at(code: &[u16]) -> Machine<TestBus> {
        let mut m = Machine::new(TestBus::new(code));
        m.core.regs.pc = 0x1000;
        // Bypassed reset leaves `sp` zero: set the active stack pointer
        // (and its bank) explicitly so exception stacking lands in RAM.
        m.core.regs.sp = 0x2000_0100;
        m.core.regs.sp = 0x2000_0100;
        m.core.regs.msp = 0x2000_0100;
        m
    }

    #[test]
    fn quantum_runs_blocks_to_budget() {
        // 10 NOPs then `B .`: one 11-op block, then 1-op self-loop hits.
        let mut code = vec![NOP; 10];
        code.push(B_SELF);
        let mut m = machine_at(&code);
        let mut runner = TbRunner::new();
        let out = m.run_tb_quantum(&mut runner, 100).unwrap();
        // 11 + 89 self-loop hits: chaining + cache hits inside quanta.
        assert_eq!(out.end, QuantumEnd::Budget);
        assert_eq!(out.ops, 100);
        assert_eq!(m.core.regs.pc, 0x1014);
        // Flush exactness: devices saw exactly the consumed cycles.
        assert_eq!(m.core.mem().ticks, out.cycles);
        assert!(out.cycles >= 100);
    }

    #[test]
    fn irq_delivers_at_boundary_not_mid_burst() {
        // Handler vector points at UNMAPPED 0x3000: delivery anywhere but
        // the boundary would fault the burst (builder refusal -> step ->
        // fetch abort). Boundary delivery ends the quantum cleanly instead.
        let mut m = machine_at(&[NOP; 8]);
        m.core.mem().write_word(0x48, 0x3001).unwrap(); // IRQ18 vector
        m.core.mem().raise(18);
        let mut runner = TbRunner::new();
        let out = m.run_tb_quantum(&mut runner, 4).unwrap();
        assert_eq!(m.core.ipsr, 18);
        assert_eq!(m.core.regs.pc, 0x3000);
        assert_eq!(m.core.mem().pending, None);
        // The burst ran first (8 NOPs plus zero-padding `LSLs` fill one
        // 64-op MaxInsns block, overshooting the budget of 4 atomically).
        assert_eq!(out.ops, 64);
        assert_eq!(out.end, QuantumEnd::Budget);
    }

    #[test]
    fn masked_irq_stays_latched_past_boundary() {
        let mut m = machine_at(&[NOP; 4]);
        m.core.mem().write_word(0x48, 0x3001).unwrap();
        m.core.mem().raise(18);
        m.core.primask = true;
        let mut runner = TbRunner::new();
        let out = m.run_tb_quantum(&mut runner, 4).unwrap();
        // Masked: no delivery, still latched, burst unaffected (E5).
        assert_eq!(m.core.ipsr, 0);
        assert_eq!(m.core.mem().pending, Some(18));
        assert_eq!(out.end, QuantumEnd::Budget);
    }

    #[test]
    fn wfi_naps_one_cycle_and_sleeps() {
        let mut m = machine_at(&[WFI, NOP]);
        let mut runner = TbRunner::new();
        let out = m.run_tb_quantum(&mut runner, 64).unwrap();
        assert_eq!(out.end, QuantumEnd::Sleep);
        assert_eq!(out.ops, 1);
        assert!(m.core.sleeping);
        // One sleeping cycle counted and flushed (E3 accounting).
        assert_eq!(m.core.cycles, 2);
        assert_eq!(m.core.mem().ticks, out.cycles);
    }

    #[test]
    fn sleeping_wakes_on_boundary_irq() {
        // Handler vector points at 0x1008 (past the WFI): waking must
        // continue there, never re-execute the WFI.
        let mut code = vec![WFI, NOP, NOP, NOP, NOP, NOP, NOP, NOP];
        code.extend_from_slice(&[NOP, NOP, B_SELF]);
        let mut m = machine_at(&code);
        m.core.mem().write_word(0x48, 0x1009).unwrap(); // IRQ18 -> code
        let mut runner = TbRunner::new();
        // First quantum: executes WFI, naps, sleeps.
        let out = m.run_tb_quantum(&mut runner, 64).unwrap();
        assert_eq!(out.end, QuantumEnd::Sleep);
        assert_eq!(m.core.regs.pc, 0x1002);
        // Raise while asleep: the next quantum's sleep-scan wakes the core
        // and continues the burst at the handler.
        m.core.mem().raise(18);
        let out = m.run_tb_quantum(&mut runner, 64).unwrap();
        assert!(!m.core.sleeping);
        assert_eq!(m.core.ipsr, 18);
        assert_eq!(out.end, QuantumEnd::Budget);
        assert!(out.ops >= 7);
    }

    #[test]
    fn it_body_falls_back_then_resumes_blocks() {
        // v7-M: `IT EQ` seals a 1-op block; the predicated body runs
        // interpreted (builder refusal -> accurate fallback step) until
        // ITSTATE drains, then blocks resume. Z == 0 so the body is
        // skipped: state-observable only via continued execution.
        let mut m = Machine::armv7m(TestBus::new(&[0xBF08, NOP, NOP, B_SELF]));
        m.core.regs.pc = 0x1000;
        m.core.regs.sp = 0x2000_0100;
        m.core.regs.msp = 0x2000_0100;
        let mut runner = TbRunner::new();
        let out = m.run_tb_quantum(&mut runner, 16).unwrap();
        assert_eq!(m.core.itstate, 0);
        assert_eq!(out.end, QuantumEnd::Budget);
        assert!(out.ops >= 4);
        assert!(runner.stats().fallbacks >= 1);
    }

    #[test]
    fn just_returned_skips_one_boundary_scan() {
        // A pending IRQ with just_returned set must NOT deliver at the
        // boundary (the restored instruction runs first); the flag is
        // consumed, so the next boundary delivers normally. The vector
        // points at unmapped memory, so any early delivery faults loudly.
        let mut m = machine_at(&[NOP, NOP, NOP, NOP]);
        m.core.mem().write_word(0x48, 0x3001).unwrap();
        m.core.mem().raise(18);
        m.core.just_returned = true;
        let mut runner = TbRunner::new();
        let out = m.run_tb_quantum(&mut runner, 4).unwrap();
        assert_eq!(m.core.ipsr, 0, "no delivery while just_returned");
        assert!(!m.core.just_returned, "flag consumed");
        assert!(out.ops > 0);
        // Next quantum: the scan runs and delivers.
        let out = m.run_tb_quantum(&mut runner, 4).unwrap();
        assert_eq!(m.core.ipsr, 18);
        assert_eq!(m.core.regs.pc, 0x3000);
        assert_eq!(out.end, QuantumEnd::Budget);
    }

    #[test]
    fn default_quantum_is_1024() {
        assert_eq!(DEFAULT_QUANTUM_OPS, 1024);
    }
}
