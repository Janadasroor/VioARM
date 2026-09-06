//! Translation-block key, metadata, LRU cache, and page-overlap index.
//!
//! Implements `jit-roadmap.md` T1 (key), T3 (metadata), T4 (LRU + cap),
//! M6 (page-indexed invalidation), and T7 (MPU epoch). See `super` for the
//! invariant list.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

use super::builder::TbOp;
use crate::cpu::Isa;

/// Default TB-cache capacity: 8192 blocks (T4). Sized so whole soak-loop
/// working sets fit while bounding memory on huge firmware (R6).
pub const DEFAULT_CAPACITY: usize = 8_192;

/// Maximum guest instructions per translation block (T2 length cap). Stops
/// pathological mega-blocks; the builder (P1-2) enforces it.
pub const MAX_TB_INSNS: u16 = 64;

/// Guest-page shift for the invalidation index: 4 KiB pages (M6).
pub const PAGE_SHIFT: u32 = 12;

/// Guest-page size for the invalidation index: 4 KiB (M6).
pub const PAGE_SIZE: u32 = 1 << PAGE_SHIFT;

/// MPU-generation counter (T7). Every MPU/SCB write that can change region
/// attribution, permissions, or enablement must call
/// [`TbCache::bump_epoch`]; stale blocks become invisible to lookups
/// immediately and are purged by the bump itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MpuEpoch(u64);

impl MpuEpoch {
    /// The reset generation.
    #[must_use]
    pub const fn initial() -> Self {
        Self(0)
    }

    /// Advance to the next generation, invalidating all cached blocks.
    pub fn bump(&mut self) {
        self.0 = self.0.wrapping_add(1);
    }

    /// The raw generation value (for logging and golden metadata).
    #[must_use]
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Translation-block cache key (T1): `(PC, security-state, MPU-epoch,
/// ISA-variant)`.
///
/// `secure` is load-bearing: the same virtual address maps different
/// bytes/devices per security state on ARMv8-M. `isa` covers the
/// v6M/v7M/v8M decode differences. Interrupt masks are excluded by design
/// (see `super`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TbKey {
    /// Guest PC of the block's first instruction (Thumb: bit 0 clear).
    pub pc: u32,
    /// ARMv8-M security state the block was translated under.
    pub secure: bool,
    /// ISA variant the block was decoded under.
    pub isa: Isa,
    /// MPU generation the block was translated under (stamped by the cache).
    pub mpu_epoch: MpuEpoch,
}

/// Per-block metadata (T3): guest extent, touched code pages (for
/// invalidation), and the cycle estimate feeding quanta accounting (D7).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TbMeta {
    /// The block's key (epoch stamped by the cache at insert).
    pub key: TbKey,
    /// Guest instruction count in the block (`<= MAX_TB_INSNS`).
    pub insns: u16,
    /// Guest byte length of the block.
    pub len_bytes: u32,
    /// First 4 KiB code page the block spans.
    pub start_page: u32,
    /// Contiguous 4 KiB code pages spanned (`>= 1`). Code is contiguous, so
    /// a start page plus a count covers the footprint exactly.
    pub page_count: u32,
    /// Approximate cycle cost of the block (D7 accounting; exactness is
    /// explicitly NOT guaranteed in JIT mode).
    pub cycle_estimate: u64,
    /// Content checksum (FNV-1a over the code bytes) taken when the block
    /// was inserted. The runner re-hashes on every cache hit and rebuilds
    /// on mismatch, so writes from ANY source — CPU stores, DMA/mailbox
    /// drains, debugger writes, flash programming — can never execute
    /// stale (M5). Stamped by the runner, not the builder.
    pub check: u64,
    /// Bit `i` set when op `i` performs a bus write (stamped by the
    /// builder): the runner revalidates only after write ops, never after
    /// pure ALU/branch/load ops (P1-6).
    pub store_mask: u64,
}

impl TbMeta {
    /// First code page *past* the block's footprint (exclusive end).
    #[must_use]
    pub const fn end_page(self) -> u32 {
        self.start_page.saturating_add(self.page_count)
    }

    /// Whether the block's code footprint covers `page`.
    #[must_use]
    pub const fn covers_page(self, page: u32) -> bool {
        page >= self.start_page && page < self.end_page()
    }
}

/// One cached block: metadata, the decoded ops to execute (Phase 1;
/// native code replaces them in Phase 2), plus LRU bookkeeping.
#[derive(Clone, Debug)]
struct Entry {
    meta: TbMeta,
    ops: Vec<TbOp>,
    /// Monotonic last-use tick; the eviction heap orders by it (lazy LRU).
    last_used: u64,
}

/// Translation-block cache: LRU storage (T4) plus a page-overlap index so
/// invalidation never scans the block set (M6).
#[derive(Clone, Debug)]
pub struct TbCache {
    epoch: MpuEpoch,
    capacity: usize,
    blocks: HashMap<TbKey, Entry>,
    /// Page number -> keys of blocks whose code footprint covers it.
    pages: HashMap<u32, HashSet<TbKey>>,
    /// Min-heap of `(last_used, key)`; stale tops are skipped lazily.
    evict: BinaryHeap<Reverse<(u64, TbKey)>>,
    tick: u64,
}

impl TbCache {
    /// An empty cache with [`DEFAULT_CAPACITY`] and the initial epoch.
    #[must_use]
    pub fn new() -> Self {
        Self::with_capacity(DEFAULT_CAPACITY)
    }

    /// An empty cache with `capacity` blocks and the initial epoch.
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            epoch: MpuEpoch::initial(),
            capacity,
            blocks: HashMap::new(),
            pages: HashMap::new(),
            evict: BinaryHeap::new(),
            tick: 0,
        }
    }

    /// Number of cached blocks.
    #[must_use]
    pub fn len(&self) -> usize {
        self.blocks.len()
    }

    /// Whether the cache holds no blocks.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.blocks.is_empty()
    }

    /// Maximum blocks retained before LRU eviction.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        self.capacity
    }

    /// The current MPU generation (stamped into keys at insert).
    #[must_use]
    pub const fn epoch(&self) -> MpuEpoch {
        self.epoch
    }

    /// Look up the block starting at `pc` for this security state and ISA
    /// under the current epoch. A hit refreshes LRU recency.
    pub fn lookup(&mut self, pc: u32, secure: bool, isa: Isa) -> Option<&TbMeta> {
        let key = TbKey {
            pc,
            secure,
            isa,
            mpu_epoch: self.epoch,
        };
        let tick = self.tick.wrapping_add(1);
        self.tick = tick;
        if let Some(entry) = self.blocks.get_mut(&key) {
            entry.last_used = tick;
            self.evict.push(Reverse((tick, key)));
            return Some(&self.blocks[&key].meta);
        }
        None
    }

    /// Insert a block with its decoded ops, stamping the current epoch into
    /// its key. Overwrites any block with the same `(pc, secure, isa)` and
    /// evicts least-recently used blocks past capacity (T4). Chained jump
    /// patches to evicted blocks must be dropped by the chaining layer (T6).
    pub fn insert(&mut self, mut meta: TbMeta, ops: Vec<TbOp>) {
        meta.key.mpu_epoch = self.epoch;
        let key = meta.key;
        // Refresh path: drop the old page registration before re-registering.
        if let Some(old) = self.blocks.get(&key) {
            Self::unregister_pages_static(&mut self.pages, old.meta);
        }
        self.tick = self.tick.wrapping_add(1);
        self.blocks.insert(
            key,
            Entry {
                meta,
                ops,
                last_used: self.tick,
            },
        );
        for page in meta.start_page..meta.end_page() {
            self.pages.entry(page).or_default().insert(key);
        }
        self.evict.push(Reverse((self.tick, key)));
        self.evict_past_capacity();
    }

    /// The decoded ops stored for `key`, if the block is cached.
    #[must_use]
    pub fn ops(&self, key: &TbKey) -> Option<&[TbOp]> {
        self.blocks.get(key).map(|entry| entry.ops.as_slice())
    }

    /// Look up a block with LRU refresh, returning metadata and payload
    /// together: one hash probe for the runner's hot path (P1-6).
    pub fn get(&mut self, pc: u32, secure: bool, isa: Isa) -> Option<(&TbMeta, &[TbOp])> {
        let key = TbKey {
            pc,
            secure,
            isa,
            mpu_epoch: self.epoch,
        };
        let tick = self.tick.wrapping_add(1);
        self.tick = tick;
        let entry = self.blocks.get_mut(&key)?;
        entry.last_used = tick;
        self.evict.push(Reverse((tick, key)));
        Some((&entry.meta, entry.ops.as_slice()))
    }

    /// Bump the MPU generation (T7): all cached blocks become stale and are
    /// purged now. Returns the number of blocks dropped.
    pub fn bump_epoch(&mut self) -> usize {
        self.epoch.bump();
        let stale: Vec<TbKey> = self
            .blocks
            .iter()
            .filter(|(key, _)| key.mpu_epoch != self.epoch)
            .map(|(key, _)| *key)
            .collect();
        let dropped = stale.len();
        for key in stale {
            self.remove(&key);
        }
        // The heap is now all-stale; rebuild it instead of leaking memory.
        self.rebuild_heap();
        dropped
    }

    /// Drop every block whose code footprint covers `page` (M6). Returns the
    /// number of blocks dropped.
    pub fn invalidate_page(&mut self, page: u32) -> usize {
        let keys: Vec<TbKey> = self
            .pages
            .remove(&page)
            .map_or_else(Vec::new, |set| set.into_iter().collect());
        let mut dropped = 0;
        for key in keys {
            if self.remove(&key) {
                dropped += 1;
            }
        }
        dropped
    }

    /// Drop every block overlapping the guest byte range
    /// `[start_addr, start_addr + len_bytes)` (M6). Returns the number of
    /// blocks dropped. An empty range drops nothing.
    pub fn invalidate_range(&mut self, start_addr: u32, len_bytes: u32) -> usize {
        if len_bytes == 0 {
            return 0;
        }
        let first = start_addr >> PAGE_SHIFT;
        let last = start_addr.saturating_add(len_bytes.saturating_sub(1)) >> PAGE_SHIFT;
        // A block spanning pages appears in several buckets: dedupe first so
        // each block is removed (and counted) exactly once.
        let mut keys = HashSet::new();
        for page in first..=last {
            if let Some(set) = self.pages.get(&page) {
                keys.extend(set.iter().copied());
            }
        }
        let mut dropped = 0;
        for key in keys {
            if self.remove(&key) {
                dropped += 1;
            }
        }
        dropped
    }

    /// Remove one block by key (unregisters all its pages). Stale heap
    /// entries are left for lazy cleanup.
    fn remove(&mut self, key: &TbKey) -> bool {
        if let Some(entry) = self.blocks.remove(key) {
            Self::unregister_pages_static(&mut self.pages, entry.meta);
            true
        } else {
            false
        }
    }

    /// Drop a block's key from every page bucket it occupies; empty buckets
    /// are removed so the index never grows stale.
    fn unregister_pages_static(pages: &mut HashMap<u32, HashSet<TbKey>>, meta: TbMeta) {
        for page in meta.start_page..meta.end_page() {
            let empty = if let Some(set) = pages.get_mut(&page) {
                set.remove(&meta.key);
                set.is_empty()
            } else {
                false
            };
            if empty {
                pages.remove(&page);
            }
        }
    }

    /// Evict least-recently used blocks until within capacity. Stale heap
    /// tops (refreshed or removed keys) are skipped.
    fn evict_past_capacity(&mut self) {
        while self.blocks.len() > self.capacity {
            let Some(Reverse((tick, key))) = self.evict.pop() else {
                break;
            };
            let stale = self
                .blocks
                .get(&key)
                .is_none_or(|entry| entry.last_used != tick);
            if stale {
                continue;
            }
            self.remove(&key);
        }
        // Bound heap memory: rebuild once stale entries dominate.
        if self.evict.len() > self.blocks.len().saturating_mul(4).max(64) {
            self.rebuild_heap();
        }
    }

    /// Rebuild the eviction heap from live blocks (drops all stale tops).
    fn rebuild_heap(&mut self) {
        self.evict = self
            .blocks
            .iter()
            .map(|(key, entry)| Reverse((entry.last_used, *key)))
            .collect();
    }
}

impl Default for TbCache {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn meta(pc: u32, page_count: u32) -> TbMeta {
        TbMeta {
            key: TbKey {
                pc,
                secure: false,
                isa: Isa::Armv7M,
                mpu_epoch: MpuEpoch::initial(),
            },
            insns: 4,
            len_bytes: 8,
            start_page: pc >> PAGE_SHIFT,
            page_count,
            cycle_estimate: 4,
            check: 0,
            store_mask: 0,
        }
    }

    #[test]
    fn key_axes_are_distinct() {
        let mut cache = TbCache::new();
        cache.insert(meta(0x1000, 1), Vec::new());
        // Same PC, Secure state: a different block, not an overwrite.
        let mut secure = meta(0x1000, 1);
        secure.key.secure = true;
        cache.insert(secure, Vec::new());
        // Same PC/state, v6-M decode: a different block.
        let mut v6 = meta(0x1000, 1);
        v6.key.isa = Isa::Armv6M;
        cache.insert(v6, Vec::new());
        assert_eq!(cache.len(), 3);
        assert!(cache.lookup(0x1000, false, Isa::Armv7M).is_some());
        assert!(cache.lookup(0x1000, true, Isa::Armv7M).is_some());
        assert!(cache.lookup(0x1000, false, Isa::Armv6M).is_some());
        // Untranslated axis combination misses.
        assert!(cache.lookup(0x1000, true, Isa::Armv6M).is_none());
        assert!(cache.lookup(0x2000, false, Isa::Armv7M).is_none());
    }

    #[test]
    fn same_key_overwrites_without_growth() {
        let mut cache = TbCache::new();
        cache.insert(meta(0x1000, 1), Vec::new());
        let mut again = meta(0x1000, 1);
        again.cycle_estimate = 9;
        cache.insert(again, Vec::new());
        assert_eq!(cache.len(), 1);
        assert_eq!(
            cache
                .lookup(0x1000, false, Isa::Armv7M)
                .unwrap()
                .cycle_estimate,
            9
        );
    }

    #[test]
    fn lru_evicts_cold_block_first() {
        let mut cache = TbCache::with_capacity(2);
        cache.insert(meta(0x1000, 1), Vec::new()); // cold: never re-touched
        cache.insert(meta(0x2000, 1), Vec::new());
        // Refresh 0x2000 so 0x1000 is least-recently used.
        assert!(cache.lookup(0x2000, false, Isa::Armv7M).is_some());
        cache.insert(meta(0x3000, 1), Vec::new());
        assert_eq!(cache.len(), 2);
        assert!(cache.lookup(0x1000, false, Isa::Armv7M).is_none());
        assert!(cache.lookup(0x2000, false, Isa::Armv7M).is_some());
        assert!(cache.lookup(0x3000, false, Isa::Armv7M).is_some());
    }

    #[test]
    fn epoch_bump_hides_and_purges_stale_blocks() {
        let mut cache = TbCache::new();
        cache.insert(meta(0x1000, 1), Vec::new());
        let dropped = cache.bump_epoch();
        assert_eq!(dropped, 1);
        assert!(cache.is_empty());
        assert!(cache.lookup(0x1000, false, Isa::Armv7M).is_none());
        assert_eq!(cache.epoch().get(), 1);
        // Re-translation under the new epoch is a fresh block.
        cache.insert(meta(0x1000, 1), Vec::new());
        assert!(cache.lookup(0x1000, false, Isa::Armv7M).is_some());
    }

    #[test]
    fn page_invalidation_hits_only_overlapping_blocks() {
        let mut cache = TbCache::new();
        cache.insert(meta(0x1000, 1), Vec::new()); // page 1
        cache.insert(meta(0x2000, 1), Vec::new()); // page 2
                                                   // Block spanning pages 3-4.
        let mut wide = meta(0x3000, 2);
        wide.len_bytes = 0x1000;
        cache.insert(wide, Vec::new());
        // A disjoint page touches nothing; the block's second page kills it.
        assert_eq!(cache.invalidate_page(0x5000 >> PAGE_SHIFT), 0);
        assert_eq!(cache.invalidate_page(0x3800 >> PAGE_SHIFT), 1);
        assert_eq!(cache.len(), 2);
        assert!(cache.lookup(0x1000, false, Isa::Armv7M).is_some());
        assert!(cache.lookup(0x2000, false, Isa::Armv7M).is_some());
    }

    #[test]
    fn range_invalidation_dedupes_spanning_blocks() {
        let mut cache = TbCache::new();
        let mut wide = meta(0x1000, 3); // pages 1-3
        wide.len_bytes = 0x2FFF;
        cache.insert(wide, Vec::new());
        cache.insert(meta(0x9000, 1), Vec::new()); // page 9, untouched
                                                   // Range covers pages 1-3: the wide block is counted exactly once.
        assert_eq!(cache.invalidate_range(0x1000, 0x3000), 1);
        assert_eq!(cache.len(), 1);
        assert!(cache.lookup(0x9000, false, Isa::Armv7M).is_some());
        // Empty ranges drop nothing.
        assert_eq!(cache.invalidate_range(0x9000, 0), 0);
        assert_eq!(cache.len(), 1);
    }

    #[test]
    fn meta_page_coverage_math() {
        let m = meta(0x1F00, 2);
        assert!(m.covers_page(1));
        assert!(m.covers_page(2));
        assert!(!m.covers_page(0));
        assert!(!m.covers_page(3));
        assert_eq!(m.end_page(), 3);
    }
}
