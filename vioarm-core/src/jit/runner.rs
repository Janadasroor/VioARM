//! Interpreted translation-block execution loop (P1-3).
//!
//! [`TbRunner::run_block`] looks a block up in the [`TbCache`], builds and
//! inserts it on a cold miss (T5), then executes its ops one by one through
//! the existing executor (`exec_tb16`/`exec_tb32`) with the exact per-op
//! cycle accounting of [`Core::step`]. [`TbRunner::run_chain`] follows
//! `end_pc` from block to block under a caller-supplied budget (the
//! interpreted analogue of T6 chaining).
//!
//! Soundness rules (each with a test below unless noted):
//! - Block entry is gated on MPU fetch permission, mirroring the stepper's
//!   pre-fetch check; a forbidden PC falls back so the stepper faults
//!   identically (P1-2 deferred requirement, closed here).
//! - A sleeping core never enters a block (the stepper no-ops while
//!   asleep); an op that sleeps (`WFI`/`WFE`) ends the block immediately.
//! - After every op the runner compares the PC against the fallthrough
//!   address: any divergence (taken branch, exception entry, fault) ends
//!   the block at the true PC. Halts propagate as `Err` exactly like the
//!   stepper reports them.
//! - Pending-interrupt delivery is NOT polled between chained blocks —
//!   identical to back-to-back `step` calls, which also leave delivery to
//!   the machine. Responsiveness quanta are P1-5's job.
//! - Cached blocks with an empty op payload are treated as misses and
//!   rebuilt (defensive: every insert path stores the payload).
//! - Verify mode (T5 debug): every stored op is re-decoded from the bus
//!   before execution and must match, catching stale-decode/store bugs.
//!   Full state-differential testing is P1-6's job.

use crate::cpu::{Core, Next};
use crate::error::HaltCause;
use crate::mem::PeripheralBus;

use super::builder::{build_tb, fetch_op, TbOp};
use super::tb::{TbCache, TbKey};

/// Per-runner execution counters (hit/miss shape for G1 measurement).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TbStats {
    /// Cache hits that executed.
    pub hits: u64,
    /// Cold misses that built, inserted, and executed.
    pub misses: u64,
    /// Builder/MPU/sleep refusals (caller steps normally).
    pub fallbacks: u64,
    /// Blocks executed (hits + misses).
    pub blocks: u64,
    /// Total ops executed.
    pub ops: u64,
}

/// What one [`TbRunner::run_block`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlockOutcome {
    /// Ops executed from the block.
    pub ops: usize,
    /// True PC after the last executed op (fallthrough or divergence).
    pub end_pc: u32,
    /// Ran every op and fell off the end (the next block is chainable).
    pub fell_through: bool,
    /// Served from cache (`false` = cold miss, just built).
    pub cached: bool,
}

/// Why [`TbRunner::run_chain`] stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChainEnd {
    /// Block budget spent (tight loops chain within one call: always bound).
    Budget,
    /// A block refused (builder/MPU/sleep): caller steps interpreted once,
    /// then re-chains.
    Fallback,
    /// The core fell asleep mid-chain.
    Sleep,
}

/// What one [`TbRunner::run_chain`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChainOutcome {
    /// Blocks executed.
    pub blocks: usize,
    /// Total ops executed.
    pub ops: usize,
    /// True PC when chaining stopped.
    pub end_pc: u32,
    /// Why chaining stopped.
    pub end: ChainEnd,
}

/// Interpreted-TB execution driver over a [`TbCache`].
#[derive(Debug)]
pub struct TbRunner {
    cache: TbCache,
    verify_decodes: bool,
    stats: TbStats,
}

impl TbRunner {
    /// A runner with a default cache and verify mode off.
    #[must_use]
    pub fn new() -> Self {
        Self {
            cache: TbCache::new(),
            verify_decodes: false,
            stats: TbStats::default(),
        }
    }

    /// A runner that re-decodes every stored op from the bus before
    /// executing it (T5 debug verify mode).
    #[must_use]
    pub fn with_verify() -> Self {
        Self {
            cache: TbCache::new(),
            verify_decodes: true,
            stats: TbStats::default(),
        }
    }

    /// The block cache (for P1-4 invalidation hooks and inspection).
    #[must_use]
    pub fn cache(&self) -> &TbCache {
        &self.cache
    }

    /// Mutable block cache (P1-4 invalidation hooks).
    pub fn cache_mut(&mut self) -> &mut TbCache {
        &mut self.cache
    }

    /// Execution counters.
    #[must_use]
    pub const fn stats(&self) -> &TbStats {
        &self.stats
    }

    /// Execute one translation block at the core's PC.
    ///
    /// Returns `Ok(None)` when no block runs here (sleeping core, MPU
    /// fetch refusal, builder refusal): the caller must `step` normally.
    /// `Err` propagates halts exactly as the stepper reports them.
    pub fn run_block<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
    ) -> Result<Option<BlockOutcome>, HaltCause> {
        if core.sleeping {
            self.stats.fallbacks += 1;
            return Ok(None);
        }
        let pc = core.regs.pc;
        let (isa, secure) = (core.isa, core.secure);
        if !core.fetch_allowed(pc) {
            self.stats.fallbacks += 1;
            return Ok(None);
        }
        let key = TbKey {
            pc,
            secure,
            isa,
            mpu_epoch: self.cache.epoch(),
        };
        // Two steps: the lookup's mutable borrow must end before the
        // payload probe (`cached_ops`) re-borrows the cache.
        let hit = self.cache.lookup(pc, secure, isa).is_some();
        let cached = if hit && !self.cached_ops(&key).is_empty() {
            true
        } else {
            let itstate = core.itstate;
            let Some(body) = build_tb(core.mem_mut(), pc, secure, isa, itstate) else {
                self.stats.fallbacks += 1;
                return Ok(None);
            };
            let meta = body.to_meta();
            self.cache.insert(meta, body.ops);
            false
        };
        if cached {
            self.stats.hits += 1;
        } else {
            self.stats.misses += 1;
        }
        if self.verify_decodes {
            self.verify_block(core, &key);
        }
        let outcome = self.exec_cached(core, &key, cached)?;
        self.stats.blocks += 1;
        self.stats.ops += outcome.ops as u64;
        Ok(Some(outcome))
    }

    /// Execute chained blocks starting at the core's PC, up to `max_blocks`
    /// blocks. Follows `end_pc` across taken branches; stops on budget,
    /// fallback, sleep, or halt (`Err`).
    pub fn run_chain<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
        max_blocks: usize,
    ) -> Result<ChainOutcome, HaltCause> {
        let mut blocks = 0;
        let mut ops = 0;
        let end = loop {
            if core.sleeping {
                break ChainEnd::Sleep;
            }
            if blocks >= max_blocks {
                break ChainEnd::Budget;
            }
            match self.run_block(core)? {
                None => break ChainEnd::Fallback,
                Some(outcome) => {
                    blocks += 1;
                    ops += outcome.ops;
                }
            }
        };
        Ok(ChainOutcome {
            blocks,
            ops,
            end_pc: core.regs.pc,
            end,
        })
    }

    /// Stored ops for a looked-up key (empty when the key just missed).
    fn cached_ops(&self, key: &TbKey) -> &[TbOp] {
        self.cache.ops(key).unwrap_or(&[])
    }

    /// T5 debug verify: re-decode every stored op from the bus; any
    /// mismatch is a stale-cache bug and panics in debug *and* release
    /// (verify mode is opt-in, so a mismatch must be loud).
    fn verify_block<B: PeripheralBus>(&self, core: &mut Core<B>, key: &TbKey) {
        let mut cur = key.pc;
        for op in self.cached_ops(key) {
            let (fresh, len) = fetch_op(core.mem_mut(), cur, key.isa)
                .unwrap_or_else(|| panic!("jit verify: bus changed under block @ {cur:#X}"));
            assert_eq!(
                *op, fresh,
                "jit verify: stale decode @ {cur:#X} (TB from pc {:#X})",
                key.pc
            );
            cur = cur.wrapping_add(len);
        }
    }

    /// Execute the cached block's ops with per-op accounting and
    /// PC-continuity checks. The ops slice borrows `self` immutably while
    /// only `core` is mutated, so no per-op re-lookup is needed.
    fn exec_cached<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
        key: &TbKey,
        cached: bool,
    ) -> Result<BlockOutcome, HaltCause> {
        let ops: &[TbOp] = self.cached_ops(key);
        let mut cur = key.pc;
        let mut done = 0;
        let mut fell_through = true;
        for op in ops {
            let fall = cur.wrapping_add(op_len(op));
            let npc = match *op {
                TbOp::V16(inst) => core.exec_tb16(cur, inst),
                TbOp::V32(inst) => core.exec_tb32(cur, inst),
            };
            let npc = match npc {
                Ok(Next::Advance(addr)) => addr,
                Ok(Next::Stop(cause)) | Err(cause) => return Err(cause),
            };
            core.commit_step(cur, npc);
            done += 1;
            // A sleeping op (WFI/WFE) ends the block even on fallthrough:
            // the stepper would no-op from here.
            if core.sleeping {
                cur = npc;
                fell_through = false;
                break;
            }
            if npc != fall {
                cur = npc;
                fell_through = false;
                break;
            }
            cur = fall;
        }
        Ok(BlockOutcome {
            ops: done,
            end_pc: cur,
            fell_through: fell_through && done == ops.len(),
            cached,
        })
    }
}

impl Default for TbRunner {
    fn default() -> Self {
        Self::new()
    }
}

/// Byte length of one op.
const fn op_len(op: &TbOp) -> u32 {
    match op {
        TbOp::V16(_) => 2,
        TbOp::V32(_) => 4,
    }
}

#[cfg(test)]
mod tests {
    use crate::error::BusError;
    use crate::mem::bus::{Bus, MpuAccess};

    use super::*;

    /// Stub bus: byte RAM plus an MPU-fetch gate switch.
    #[derive(Clone, Debug)]
    struct StubBus {
        base: u32,
        data: Vec<u8>,
        deny_fetch: bool,
    }

    impl StubBus {
        fn with_halves(base: u32, halves: &[u16]) -> Self {
            let mut bus = Self {
                base,
                data: vec![0; halves.len() * 2],
                deny_fetch: false,
            };
            for (i, half) in halves.iter().enumerate() {
                let [lo, hi] = half.to_le_bytes();
                bus.data[i * 2] = lo;
                bus.data[i * 2 + 1] = hi;
            }
            bus
        }
    }

    impl Bus for StubBus {
        fn read(&mut self, addr: u32) -> Result<u8, BusError> {
            self.data
                .get(addr.wrapping_sub(self.base) as usize)
                .copied()
                .ok_or(BusError::Reserved)
        }

        fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
            let i = addr.wrapping_sub(self.base) as usize;
            if i < self.data.len() {
                self.data[i] = val;
                Ok(())
            } else {
                Err(BusError::Reserved)
            }
        }
    }

    impl PeripheralBus for StubBus {
        fn next_exception(
            &mut self,
            _primask: bool,
            _faultmask: bool,
            _basepri: u8,
            _current_ipsr: u8,
        ) -> Option<u8> {
            None
        }

        fn acknowledge_exception(&mut self, _n: u8) {}

        fn mpu_check(
            &mut self,
            addr: u32,
            access: MpuAccess,
            _privileged: bool,
            _exception: u8,
        ) -> bool {
            !(self.deny_fetch && access == MpuAccess::Fetch && addr < 0x2000_0000)
        }
    }

    type StubCore = Core<StubBus>;

    fn core_at(halves: &[u16]) -> StubCore {
        let mut core = StubCore::new(StubBus::with_halves(0x1000, halves));
        core.regs.pc = 0x1000;
        core
    }

    const NOP: u16 = 0xBF00;
    const MOVS_R0_1: u16 = 0x2001;
    const MOVS_R1_2: u16 = 0x2102;
    const B_SELF: u16 = 0xE7FE;
    /// `B +0`: target is `pc + 4`, skipping one halfword.
    const B_NEXT: u16 = 0xE000;
    const WFI: u16 = 0xBF30;

    #[test]
    fn cold_miss_builds_inserts_and_executes() {
        let mut core = core_at(&[MOVS_R0_1, NOP]);
        let mut runner = TbRunner::new();
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(
            outcome,
            BlockOutcome {
                ops: 2,
                end_pc: 0x1004,
                fell_through: true,
                cached: false,
            }
        );
        assert_eq!(core.regs.r[0], 1);
        assert_eq!(core.regs.pc, 0x1004);
        assert_eq!(core.cycles, 2);
        assert_eq!(runner.stats().misses, 1);
        assert_eq!(runner.cache().len(), 1);
    }

    #[test]
    fn second_run_hits_cache() {
        let mut core = core_at(&[MOVS_R0_1, NOP]);
        let mut runner = TbRunner::new();
        core.regs.pc = 0x1000;
        let _ = runner.run_block(&mut core).unwrap().unwrap();
        // Reset architectural state, keep the cache: must hit now.
        core.regs.pc = 0x1000;
        core.regs.r[0] = 0;
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert!(outcome.cached);
        assert_eq!(core.regs.r[0], 1);
        assert_eq!(runner.stats().hits, 1);
        assert_eq!(runner.cache().len(), 1);
    }

    #[test]
    fn taken_branch_diverges_at_true_pc() {
        let mut core = core_at(&[NOP, B_SELF]);
        let mut runner = TbRunner::new();
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        // NOP falls to 0x1002; `B .` at 0x1002 targets itself.
        assert_eq!(outcome.ops, 2);
        assert_eq!(outcome.end_pc, 0x1002);
        assert!(!outcome.fell_through);
        assert_eq!(core.regs.pc, 0x1002);
    }

    #[test]
    fn chain_follows_branches_under_budget() {
        // 0x1000: B -> 0x1004. 0x1004: MOVS r1,#2; B . (self at 0x1006).
        let mut core = core_at(&[B_NEXT, NOP, MOVS_R1_2, B_SELF]);
        let mut runner = TbRunner::new();
        let chain = runner.run_chain(&mut core, 4).unwrap();
        assert_eq!(
            chain,
            ChainOutcome {
                blocks: 4,
                ops: 1 + 2 + 1 + 1,
                end_pc: 0x1006,
                end: ChainEnd::Budget,
            }
        );
        assert_eq!(core.regs.r[1], 2);
        // Block shapes: [B](diverge) [MOVS,B](diverge) [B](hit) [B](hit).
        assert_eq!(runner.stats().misses, 3);
        assert_eq!(runner.stats().hits, 1);
    }

    #[test]
    fn fallback_paths_return_none() {
        let mut core = core_at(&[NOP]);
        let mut runner = TbRunner::new();
        // Odd PC: builder refuses.
        core.regs.pc = 0x1001;
        assert!(runner.run_block(&mut core).unwrap().is_none());
        // ITSTATE set: builder refuses.
        core.regs.pc = 0x1000;
        core.itstate = 1;
        assert!(runner.run_block(&mut core).unwrap().is_none());
        core.itstate = 0;
        // Sleeping core: never enters.
        core.sleeping = true;
        assert!(runner.run_block(&mut core).unwrap().is_none());
        core.sleeping = false;
        assert_eq!(runner.stats().fallbacks, 3);
    }

    #[test]
    fn mpu_fetch_refusal_falls_back() {
        let mut core = core_at(&[MOVS_R0_1, NOP]);
        core.mem_mut().deny_fetch = true;
        let mut runner = TbRunner::new();
        assert!(runner.run_block(&mut core).unwrap().is_none());
        assert_eq!(core.regs.pc, 0x1000);
        assert_eq!(core.regs.r[0], 0);
        // Gate open again: the same PC builds and runs.
        core.mem_mut().deny_fetch = false;
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(outcome.ops, 2);
        assert_eq!(core.regs.r[0], 1);
    }

    #[test]
    fn wfi_sleeps_and_stops_the_chain() {
        let mut core = core_at(&[WFI, NOP]);
        let mut runner = TbRunner::new();
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(outcome.ops, 1);
        assert!(core.sleeping);
        let chain = runner.run_chain(&mut core, 8).unwrap();
        assert_eq!(
            chain,
            ChainOutcome {
                blocks: 0,
                ops: 0,
                end_pc: core.regs.pc,
                end: ChainEnd::Sleep,
            }
        );
    }

    #[test]
    fn verify_mode_accepts_clean_blocks() {
        let mut core = core_at(&[MOVS_R0_1, NOP]);
        let mut runner = TbRunner::with_verify();
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(outcome.ops, 2);
        assert_eq!(core.regs.r[0], 1);
    }

    #[test]
    fn runner_matches_stepper_cycle_for_cycle() {
        // One block of four ops (MOVS, LDR-literal, NOP, B .): the runner
        // must match four stepper steps in pc, counters, and registers.
        let halves = [MOVS_R0_1, 0x4800, NOP, B_SELF];
        let mut ref_core = core_at(&halves);
        for _ in 0..4 {
            ref_core.step().unwrap();
        }
        let mut core = core_at(&halves);
        let mut runner = TbRunner::new();
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(outcome.ops, 4);
        assert!(!outcome.fell_through);
        assert_eq!(core.regs.pc, ref_core.regs.pc);
        assert_eq!(core.cycles, ref_core.cycles);
        assert_eq!(core.cyccnt, ref_core.cyccnt);
        assert_eq!(core.regs.r[0], ref_core.regs.r[0]);
    }
}
