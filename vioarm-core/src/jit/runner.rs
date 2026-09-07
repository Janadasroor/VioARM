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
//! - Block entry purges on bus config-generation change, and write ops
//!   revalidate content + config right after executing (P1-6): an MPU/SAU
//!   reconfiguration or code patch (by CPU store, DMA drain, or debugger
//!   write — all funnel through the bus `write` chokepoint) exits the
//!   running block at the true PC, even when the patch target is a later
//!   op in the same block. Loads/ALU/branches need no checks: they cannot
//!   mutate code or config (M5/T7).
//! - Cache hits re-hash the code bytes (FNV-1a taken at insert) and rebuild
//!   on mismatch, so self-modifying code from ANY source can never execute
//!   stale — source-agnostic by construction, no per-site hook audit (M5).
//!   The M1 page-guard map proper ships with the emitter (P2-3); checksums
//!   are its Phase-1 safety equivalent. Hook-armed ranges (X1) arrive with
//!   P3-3 host surfaces.
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

use crate::cpu::{Core, Isa, Next};
use crate::error::HaltCause;
use crate::mem::PeripheralBus;

use super::builder::{build_tb, fetch_op, op_len, TbOp};
use super::native::NativeBlock;
use super::tb::{Got, TbCache, TbKey, TbMeta};

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
    /// Blocks executed natively (subset of `blocks`; G2 measurement).
    pub native: u64,
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
    /// Page translation cache for native fast paths (runner-owned box:
    /// stable address across calls; cleared with the TB cache on config
    /// changes).
    tlb: Box<super::tlb::Tlb>,
    verify_decodes: bool,
    /// Execute fully-templated blocks natively (default on). Off forces
    /// the interpreted loop everywhere: cycle-exact `cyccnt`, for the
    /// P1-6 differential (native mode commits block estimates per D7).
    use_native: bool,
    /// Last bus config generation seen (`None` until the first poll, or
    /// permanently when the bus is untracked — which polls as changed).
    last_seen: Option<u64>,
    stats: TbStats,
}

impl TbRunner {
    /// A runner with a default cache and verify mode off.
    #[must_use]
    pub fn new() -> Self {
        Self {
            cache: TbCache::new(),
            tlb: Box::default(),
            verify_decodes: false,
            use_native: true,
            last_seen: None,
            stats: TbStats::default(),
        }
    }

    /// A runner that re-decodes every stored op from the bus before
    /// executing it (T5 debug verify mode).
    #[must_use]
    pub fn with_verify() -> Self {
        Self {
            cache: TbCache::new(),
            tlb: Box::default(),
            verify_decodes: true,
            use_native: true,
            last_seen: None,
            stats: TbStats::default(),
        }
    }

    /// Force interpreted (`false`) or native-where-coverable (`true`)
    /// execution. Interpreted mode is cycle-exact; native mode commits
    /// block-level `cyccnt` estimates (D7) while keeping retired-op
    /// `cycles` and all architectural state exact.
    pub fn set_native(&mut self, on: bool) {
        self.use_native = on;
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
        // Predicated bodies never enter blocks (hits or misses): plain
        // `exec` does not predicate (the stepper wraps it in `exec_it_step`
        // for that), so a block cached under `itstate == 0` must not run
        // under `itstate != 0` — e.g. a branch landing inside someone
        // else's IT body. The fallback stepper owns predicated execution
        // until the IT state drains (Phase-1 IT policy; S4 goes native).
        if core.itstate != 0 {
            self.stats.fallbacks += 1;
            return Ok(None);
        }
        // Armed hooks run interpreted (P3-3 refines to per-range bypass):
        // native blocks neither fire per-insn hooks nor check watchpoints,
        // and fast paths skip hook registries — so any armed hook forces
        // the accurate path, which fires. Peripheral-only hooks need no
        // gate (fast paths never touch devices; side exits go interpreted).
        if let Some(hooks) = core.mem_mut().hooks() {
            if hooks.has_before() || hooks.has_after() || hooks.has_read() || hooks.has_write() {
                self.stats.fallbacks += 1;
                return Ok(None);
            }
        }
        let pc = core.regs.pc;
        let (isa, secure) = (core.isa, core.secure);
        if !core.fetch_allowed(pc) {
            self.stats.fallbacks += 1;
            return Ok(None);
        }
        // Entry config poll (T7/M5): blocks translated under an older
        // fetch-affecting configuration are purged before lookup.
        self.poll_config(core.mem_mut().config_epoch());
        let key = TbKey {
            pc,
            secure,
            isa,
            mpu_epoch: self.cache.epoch(),
        };
        // One probe returns owned copies (the borrow ends before content
        // probes re-borrow the cache; no second lookup). A hit
        // additionally requires validated code bytes (gated on the write
        // generation: a matching generation means no byte could have
        // changed since the last validation). Any mismatch rebuilds.
        let probed: Option<(TbMeta, Option<NativeBlock>, Vec<TbOp>)> =
            self.cache.get(pc, secure, isa).map(|hit| match hit {
                Got::Native(meta, native) => (*meta, Some(native.clone()), Vec::new()),
                Got::Interp(meta, ops) => (*meta, None, ops.to_vec()),
            });
        let (meta, native, ops, cached) = match probed {
            Some((meta, Some(native), _))
                if self.use_native && self.validated(core, &key, &meta) =>
            {
                self.stats.hits += 1;
                (meta, Some(native), Vec::new(), true)
            }
            Some((meta, None, ops)) if !ops.is_empty() && self.validated(core, &key, &meta) => {
                self.stats.hits += 1;
                (meta, None, ops, true)
            }
            Some((meta, _, _)) => {
                self.cache.invalidate_range(meta.key.pc, meta.len_bytes);
                match self.miss(core, pc, secure, isa) {
                    Some((meta, native, ops)) => (meta, native, ops, false),
                    None => return Ok(None),
                }
            }
            None => match self.miss(core, pc, secure, isa) {
                Some((meta, native, ops)) => (meta, native, ops, false),
                None => return Ok(None),
            },
        };
        if self.verify_decodes {
            self.verify_block(core, &key);
        }
        let outcome = match native {
            Some(native) => {
                let outcome = self.exec_native(core, &meta, &native, cached)?;
                self.stats.native += 1;
                outcome
            }
            None => self.exec_cached(core, &key, &meta, &ops, cached)?,
        };
        self.stats.blocks += 1;
        self.stats.ops += outcome.ops as u64;
        Ok(Some(outcome))
    }

    /// Execute a native block: enter through the trampoline, decode the
    /// exit, commit block-level accounting. S0+terminator blocks run every
    /// op exactly once with no faults, divergence inside, or sleep (no
    /// memory/system ops by template construction), so the outcome shape
    /// follows directly. Cycle accounting is per-block (D7): `cycles`
    /// counts retired ops exactly, `cyccnt` takes the block estimate, and
    /// the load-use tracker resets (no loads occur in native S0 blocks —
    /// exactly what stepping them would leave behind).
    fn exec_native<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
        meta: &TbMeta,
        native: &NativeBlock,
        cached: bool,
    ) -> Result<BlockOutcome, HaltCause> {
        use super::native::{exit_pc, exit_reason, NativeCtx, EXIT_FALL, EXIT_MMIO};

        // Whole-core reads first (each borrow ends before the field
        // borrows below begin).
        let privileged = u32::from(core.is_privileged());
        let write_gen = core
            .mem_mut()
            .write_epoch_ptr()
            .unwrap_or(core::ptr::null_mut());
        let mut ctx = Box::new(NativeCtx {
            regs: &mut core.regs,
            xpsr: &mut core.xpsr,
            tlb: self.tlb.as_mut() as *mut super::tlb::Tlb,
            privileged,
            write_gen,
            ..Default::default()
        });
        let code = vioarm_jit_call::call1(native.entry_ptr(), &mut *ctx as *mut NativeCtx as u64);
        let reason = exit_reason(code);
        let npc = exit_pc(code);
        if reason == EXIT_MMIO {
            return self.native_mmio_exit(core, meta, npc, ctx.fault_addr);
        }
        let end_pc = npc;
        let fell_through = reason == EXIT_FALL;
        let ops = meta.insns as u64;
        core.cycles += ops;
        core.cyccnt += meta.cycle_estimate;
        core.set_load_tracker(native.last_load());
        core.regs.pc = end_pc;
        Ok(BlockOutcome {
            ops: ops as usize,
            end_pc,
            fell_through,
            cached,
        })
    }

    /// A native memory side exit at `op_pc` (guard/TLB miss, page cross,
    /// unaligned-with-trap, bitband): commit the native prefix op-by-op
    /// (exact — every prefix op fell through, else an earlier exit would
    /// have fired), re-execute the faulting op through the interpreter
    /// (identical MMIO and precise-fault behavior), and attempt a TLB fill
    /// from the reported address for next time. Re-fetching the ops is
    /// sound: stores seal blocks, so only effect-free ALU ops precede any
    /// op in a block and the bytes cannot have changed since entry
    /// validation.
    fn native_mmio_exit<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
        meta: &TbMeta,
        op_pc: u32,
        fault_addr: u32,
    ) -> Result<BlockOutcome, HaltCause> {
        use super::templates::mem_access;
        use super::tlb::fill_tlb;

        let isa = core.isa;
        let mut pc = meta.key.pc;
        let mut ops = 0usize;
        while pc != op_pc {
            let Some((_, len)) = fetch_op(core.mem_mut(), pc, isa) else {
                return Err(HaltCause::FetchAbort);
            };
            let next = pc.wrapping_add(len);
            core.commit_step(pc, next);
            ops += 1;
            pc = next;
        }
        let Some((op, len)) = fetch_op(core.mem_mut(), op_pc, isa) else {
            return Err(HaltCause::FetchAbort);
        };
        let npc = match op {
            TbOp::V16(inst) => core.exec_tb16(op_pc, inst),
            TbOp::V32(inst) => core.exec_tb32(op_pc, inst),
        };
        let npc = match npc {
            Ok(Next::Advance(addr)) => addr,
            Ok(Next::Stop(cause)) | Err(cause) => return Err(cause),
        };
        core.commit_step(op_pc, npc);
        ops += 1;
        if let Some((size, is_write)) = mem_access(&op) {
            // Whole-core reads first (each borrow ends before `mem_mut`).
            let privileged = core.is_privileged();
            let ipsr = core.ipsr;
            fill_tlb(
                &mut self.tlb,
                core.mem_mut(),
                fault_addr,
                size,
                is_write,
                privileged,
                ipsr,
            );
        }
        let fall = op_pc.wrapping_add(len);
        Ok(BlockOutcome {
            ops,
            end_pc: npc,
            fell_through: npc == fall,
            cached: true,
        })
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

    /// Cold-miss path: build, hash, then go native when every op has a
    /// template (otherwise interpreted). `None` on builder refusal or on
    /// bytes vanishing between build and hash (fall back so the stepper
    /// faults identically). Emission failures (including resource
    /// exhaustion) degrade gracefully to interpreted execution.
    fn miss<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
        pc: u32,
        secure: bool,
        isa: Isa,
    ) -> Option<(TbMeta, Option<NativeBlock>, Vec<TbOp>)> {
        use super::templates::emit_block;

        // Explicit `else` (not `?`): every refusal counts a fallback.
        let itstate = core.itstate;
        let Some(body) = build_tb(core.mem_mut(), pc, secure, isa, itstate) else {
            self.stats.fallbacks += 1;
            return None;
        };
        let mut meta = body.to_meta();
        let Some(check) = Self::hash_code(core, &meta) else {
            self.stats.fallbacks += 1;
            return None;
        };
        meta.check = check;
        // Stamp this entry's generation: the hash just established its
        // baseline (untracked buses stamp 0 and always revalidate).
        // Native store templates bump the write generation on the fast
        // path; untracked buses (no counter) keep store-containing blocks
        // interpreted, where post-store revalidation covers them. Loads-only
        // blocks go native regardless (entry revalidation suffices: no
        // mid-block stores exist to track). The CCR unaligned-trap bit is
        // sampled here (stable across the block by store-seal + purge).
        meta.write_gen = core.mem_mut().write_epoch().unwrap_or(0);
        let tracked = core.mem_mut().write_epoch_ptr().is_some();
        let unalign_trap = core
            .mem_mut()
            .read_word(0xE000_ED14)
            .map(|ccr| ccr & 8 != 0)
            .unwrap_or(false);
        if self.use_native && (body.store_mask == 0 || tracked) {
            if let Ok(native) = emit_block(&body, unalign_trap) {
                self.cache.insert_native(meta, native.clone());
                self.stats.misses += 1;
                return Some((meta, Some(native), Vec::new()));
            }
        }
        let ops = body.ops.clone();
        self.cache.insert(meta, body.ops);
        self.stats.misses += 1;
        Some((meta, None, ops))
    }

    /// Entry content gate (P2-3): re-hash only when the bus reports writes
    /// since THIS entry validated — an unchanged generation proves no byte
    /// could have changed, whatever the source. The generation is tracked
    /// per cache entry (a global baseline is unsound: a store in block B
    /// must not mark block A's stale bytes fresh). Untracked buses
    /// (`None`) always re-hash. True when content is confirmed (and the
    /// entry's generation is refreshed on a match).
    fn validated<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
        key: &TbKey,
        meta: &TbMeta,
    ) -> bool {
        if let Some(g) = core.mem_mut().write_epoch() {
            if g == meta.write_gen {
                return true;
            }
        }
        if Self::code_matches(core, meta) {
            if let Some(g) = core.mem_mut().write_epoch() {
                self.cache.note_validated(key, g);
            }
            true
        } else {
            false
        }
    }

    /// Re-check content + config after a write op (P1-6): true when the
    /// block went stale and execution must stop at the current PC. A
    /// matching revalidation refreshes the generation baseline.
    fn revalidate<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
        key: &TbKey,
        meta: &TbMeta,
    ) -> bool {
        if self.poll_config(core.mem_mut().config_epoch()) {
            return true;
        }
        if !Self::code_matches(core, meta) {
            self.cache.invalidate_range(meta.key.pc, meta.len_bytes);
            return true;
        }
        if let Some(g) = core.mem_mut().write_epoch() {
            self.cache.note_validated(key, g);
        }
        false
    }

    /// Poll the bus config generation; on change purge blocks translated
    /// under the older configuration (T7) plus the whole TLB (mappings
    /// may have moved). Returns whether it changed, in which case block
    /// execution must stop at the current PC. Called at block entry and
    /// after write ops only (never per-op: only stores can change config
    /// mid-block). An untracked bus (`None`) always reports change: entry
    /// purges every time (safe; rebuilds each block).
    fn poll_config(&mut self, bus_epoch: Option<u64>) -> bool {
        let changed = match bus_epoch {
            None => true,
            Some(seen) => self.last_seen != Some(seen),
        };
        if changed {
            self.cache.bump_epoch();
            self.tlb.clear();
            self.last_seen = bus_epoch;
        }
        changed
    }

    /// FNV-1a over the block's code bytes; `None` when a byte is unreadable.
    /// The range is bounded by construction (`<= 64` ops, `<= 256` bytes).
    fn hash_code<B: PeripheralBus>(core: &mut Core<B>, meta: &TbMeta) -> Option<u64> {
        let bus = core.mem_mut();
        let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
        for offset in 0..meta.len_bytes {
            let byte = bus.read(meta.key.pc.wrapping_add(offset)).ok()?;
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x0100_0000_01b3);
        }
        Some(hash)
    }

    /// Whether the bus bytes still match the checksum taken at insert.
    fn code_matches<B: PeripheralBus>(core: &mut Core<B>, meta: &TbMeta) -> bool {
        Self::hash_code(core, meta) == Some(meta.check)
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
    /// PC-continuity checks. Ops are copied out first so revalidation
    /// (which mutates the cache) never fights the borrow checker; the copy
    /// is `<= 64` small `Copy` ops. Revalidation runs only after bus-write
    /// ops (`store_mask`): loads/ALU/branches cannot mutate code or config
    /// mid-block (see `op_writes_memory`), so hot loops pay no per-op
    /// polling (P1-6). Sleeping is checked once at block end: only
    /// `WFI`/`WFE` set it and both always seal their block, so sleep can
    /// only ever engage on the last op.
    fn exec_cached<B: PeripheralBus>(
        &mut self,
        core: &mut Core<B>,
        key: &TbKey,
        meta: &TbMeta,
        ops: &[TbOp],
        cached: bool,
    ) -> Result<BlockOutcome, HaltCause> {
        let mask = meta.store_mask;
        let mut cur = key.pc;
        let mut done = 0;
        let mut fell_through = true;
        for (i, op) in ops.iter().enumerate() {
            // Invariant: sleeping cores never enter blocks (run_block
            // refuses) and only sealing last-ops can sleep.
            debug_assert!(!core.sleeping, "sleep engaged mid-block");
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
            // A write op may have patched code (even later ops in this same
            // block) or reprogrammed the MPU: revalidate and exit at the
            // true PC on any change; the next block rebuilds fresh.
            if mask & (1_u64 << i) != 0 && self.revalidate(core, key, meta) {
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
        if core.sleeping {
            fell_through = false;
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

#[cfg(test)]
mod tests {
    use crate::error::BusError;
    use crate::mem::bus::{Bus, MpuAccess};

    use super::*;

    /// Stub bus: byte RAM plus an MPU-fetch gate switch and a controllable
    /// config generation (the test analog of the system-control window:
    /// writes to `MAGIC_CONFIG_ADDR` sink and bump it).
    #[derive(Clone, Debug)]
    struct StubBus {
        base: u32,
        data: Vec<u8>,
        deny_fetch: bool,
        untracked: bool,
        config: u64,
        writes: u64,
    }

    /// Writes here succeed without backing and bump the config generation.
    const MAGIC_CONFIG_ADDR: u32 = 0x2000;

    impl StubBus {
        fn with_halves(base: u32, halves: &[u16]) -> Self {
            let mut bus = Self {
                base,
                data: vec![0; halves.len() * 2],
                deny_fetch: false,
                untracked: false,
                config: 0,
                writes: 0,
            };
            for (i, half) in halves.iter().enumerate() {
                bus.poke_half(base + (i as u32) * 2, *half);
            }
            bus
        }

        fn poke_half(&mut self, addr: u32, half: u16) {
            let [lo, hi] = half.to_le_bytes();
            let i = addr.wrapping_sub(self.base) as usize;
            self.data[i] = lo;
            self.data[i + 1] = hi;
            // Backdoor writes bump too: ANY byte mutation must be visible
            // to the validation gate, without exception.
            self.writes = self.writes.wrapping_add(1);
        }

        fn bump_config(&mut self) {
            self.config = self.config.wrapping_add(1);
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
            if addr == MAGIC_CONFIG_ADDR {
                self.bump_config();
                self.writes = self.writes.wrapping_add(1);
                return Ok(());
            }
            let i = addr.wrapping_sub(self.base) as usize;
            if i < self.data.len() {
                self.data[i] = val;
                self.writes = self.writes.wrapping_add(1);
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

        fn config_epoch(&self) -> Option<u64> {
            if self.untracked {
                None
            } else {
                Some(self.config)
            }
        }

        fn write_epoch(&self) -> Option<u64> {
            if self.untracked {
                None
            } else {
                Some(self.writes)
            }
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
    fn coverable_block_goes_native() {
        // S0-pure block (no terminator): prologue + templates + FALL.
        let mut core = core_at(&[MOVS_R0_1, 0x2102]);
        let mut runner = TbRunner::new();
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(outcome.ops, 2);
        assert!(outcome.fell_through);
        assert!(!outcome.cached);
        assert_eq!(runner.stats().native, 1);
        assert_eq!(core.regs.r[0], 1);
        assert_eq!(core.regs.r[1], 2);
        // Second run hits the native block (no rebuild).
        core.regs.pc = 0x1000;
        core.regs.r[0] = 0;
        core.regs.r[1] = 0;
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert!(outcome.cached);
        assert_eq!(runner.stats().native, 2);
        assert_eq!(runner.stats().hits, 1);
        assert_eq!((core.regs.r[0], core.regs.r[1]), (1, 2));
    }

    #[test]
    fn interp_mode_never_goes_native() {
        let mut core = core_at(&[MOVS_R0_1, 0x2102]);
        let mut runner = TbRunner::new();
        runner.set_native(false);
        let _ = runner.run_block(&mut core).unwrap().unwrap();
        core.regs.pc = 0x1000;
        let _ = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(runner.stats().native, 0);
        assert_eq!(runner.stats().hits, 1);
        assert_eq!((core.regs.r[0], core.regs.r[1]), (1, 2));
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
    fn itstate_hit_refuses_despite_cache() {
        // The hole: a block cached under itstate==0 must not execute (even
        // partially) under itstate!=0, where plain `exec` would ignore
        // predication. A branch landing inside someone else's IT body hits
        // this; the fallback stepper predicates correctly.
        let mut core = core_at(&[NOP, NOP]);
        let mut runner = TbRunner::new();
        let _ = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(runner.cache().len(), 1);
        core.regs.pc = 0x1000;
        core.itstate = 1;
        assert!(runner.run_block(&mut core).unwrap().is_none());
        assert_eq!(runner.stats().fallbacks, 1);
        // Drained again: the cached block runs.
        core.itstate = 0;
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert!(outcome.cached);
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
    fn self_modifying_code_rebuilds() {
        let mut core = core_at(&[MOVS_R0_1, NOP]);
        let mut runner = TbRunner::new();
        let _ = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(core.regs.r[0], 1);
        // Rewrite the first instruction in place (any width, any source:
        // the checksum is source-agnostic by construction).
        core.regs.pc = 0x1000;
        core.mem_mut().poke_half(0x1000, 0x2002);
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert!(!outcome.cached);
        assert_eq!(core.regs.r[0], 2);
        assert_eq!(runner.stats().misses, 2);
        assert_eq!(runner.stats().hits, 0);
        // Same through the word-width write path.
        core.regs.pc = 0x1000;
        core.regs.r[0] = 0;
        core.mem_mut().write_word(0x1000, 0xBF00_2003).unwrap();
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert!(!outcome.cached);
        assert_eq!(core.regs.r[0], 3);
    }

    #[test]
    fn config_bump_purges_and_rebuilds() {
        let mut core = core_at(&[MOVS_R0_1, NOP]);
        let mut runner = TbRunner::new();
        let _ = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(runner.cache().len(), 1);
        // An MPU/SAU reconfiguration between blocks purges everything.
        core.mem_mut().bump_config();
        core.regs.pc = 0x1000;
        core.regs.r[0] = 0;
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert!(!outcome.cached);
        assert_eq!(core.regs.r[0], 1);
        assert_eq!(runner.stats().misses, 2);
        assert_eq!(runner.cache().len(), 1);
    }

    #[test]
    fn mid_block_config_change_exits_at_true_pc() {
        // `STRB r0, [r1]` (0x7008, a continuing plain store) targets the
        // config-magic address with r1 = 0x2000: the store succeeds, bumps
        // the generation, and the runner must exit before the NOP. (Byte
        // store: a word store would spill past the single magic address.)
        let mut core = core_at(&[0x7008, NOP]);
        core.regs.r[0] = 0xAA;
        core.regs.r[1] = MAGIC_CONFIG_ADDR;
        let mut runner = TbRunner::new();
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(outcome.ops, 1);
        assert_eq!(outcome.end_pc, 0x1002);
        assert!(!outcome.fell_through);
        // The purge already dropped the block that just ran.
        assert!(runner.cache().is_empty());
        // Re-execution rebuilds (and exits again: the store always bumps).
        core.regs.pc = 0x1000;
        let outcome = runner.run_block(&mut core).unwrap().unwrap();
        assert!(!outcome.cached);
        assert_eq!(runner.stats().misses, 2);
    }

    #[test]
    fn untracked_bus_rebuilds_every_block() {
        // `None` generation = assume change at every entry: the cache is
        // purged before each lookup, so every block is a cold miss — but
        // full blocks still execute (revalidation only fires after writes).
        let mut core = core_at(&[MOVS_R0_1, NOP]);
        core.mem_mut().untracked = true;
        let mut runner = TbRunner::new();
        let first = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(first.ops, 2);
        assert!(first.fell_through);
        core.regs.pc = 0x1000;
        let second = runner.run_block(&mut core).unwrap().unwrap();
        assert_eq!(second.ops, 2);
        assert_eq!(runner.stats().hits, 0);
        assert_eq!(runner.stats().misses, 2);
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
