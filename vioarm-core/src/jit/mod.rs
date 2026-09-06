//! Translated fast engine: translation-block (TB) cache skeleton.
//!
//! Phase 1 of `jit-roadmap.md`: TB data structures only — key (T1),
//! metadata (T3), LRU storage with capacity cap (T4), page-overlap index
//! for invalidation (M6), and the MPU-epoch source of truth (T7). No
//! decoder calls, no emitter, no dynasm dependency: the block builder
//! (P1-2) and the interpreted execution loop (P1-3) build on this.
//!
//! Key invariants (see `jit-roadmap.md` §3–§4):
//! - The cache stamps the *current* MPU epoch into every key at insert
//!   time; lookups always use the current epoch, so a bumped epoch hides
//!   stale blocks without a scan. `bump_epoch` additionally purges them so
//!   dead entries never occupy capacity.
//! - `PRIMASK`/`BASEPRI`/`FAULTMASK` are deliberately NOT in the key: they
//!   affect delivery at quanta boundaries, never mid-block content.
//! - Blocks must never cover bit-band alias regions (M4) and must stop at
//!   MMIO touches (T2): the builder enforces that; the cache trusts the
//!   `start_page`/`page_count` it is given and invalidates on overlap.

mod builder;
mod emit;
mod native;
mod quanta;
mod runner;
mod tb;

pub use builder::{build_tb, TbBody, TbEnd, TbOp};
pub use emit::{assemble_smoke, EmitError};
pub use native::{exit_code, exit_pc, exit_reason, NativeBlock, NativeCtx, EXIT_BRANCH, EXIT_FALL};
pub use quanta::{QuantumEnd, QuantumOutcome, DEFAULT_QUANTUM_OPS};
pub use runner::{BlockOutcome, ChainEnd, ChainOutcome, TbRunner, TbStats};
pub use tb::BlockPayload;
pub use tb::{
    MpuEpoch, TbCache, TbKey, TbMeta, DEFAULT_CAPACITY, MAX_TB_INSNS, PAGE_SHIFT, PAGE_SIZE,
};
