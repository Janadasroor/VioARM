//! Guest-page translation cache (M1, P2-3): a small direct-mapped TLB
//! letting emitted loads/stores reach RAM/flash without a callout.
//!
//! Each entry maps one 4 KiB guest page (plus the effective privilege the
//! permissions were checked under, per M2) to a host page base. Entries
//! are filled on side-exit misses by [`fill_tlb`] (full bus translation +
//! live MPU check) and flushed wholesale on config changes (the runner
//! clears alongside the TB purge — no per-entry epochs needed). Zeroed
//! entries deny by construction (`perms == 0` fails every check), so no
//! sentinel value is required.
//!
//! A single victim slot (P3-6) holds the most-recently-evicted entry.
//! Two hot pages sharing one index (e.g. flash page 0 and RAM page
//! `0x20000`, both index 0) would otherwise evict each other on every
//! access and side-exit every lap forever; with the victim both pages
//! stay resident after the two cold fills (one primary, one victim).
//! Three or more live pages on one index still rotate — documented,
//! strictly better than before.
//!
//! Soundness notes:
//! - The host pointer is only dereferenced after a tag + permission hit
//!   in emitted code; mappings are stable (banks never reallocate;
//!   reconfiguration flushes).
//! - Entries cover exactly one page: accesses crossing a page boundary
//!   always side-exit (templates enforce this for multi-byte accesses),
//!   so a hit can never alias a differently-mapped neighbor page.
//! - Fill verifies the access fits the reported range AND the page.

use crate::mem::bus::MpuAccess;
use crate::mem::PeripheralBus;

/// TLB size (direct-mapped, `vpn & mask`).
pub const TLB_SIZE: usize = 64;
/// TLB index mask.
pub const TLB_MASK: u32 = (TLB_SIZE as u32) - 1;
/// Byte offset of the victim slot from the entries base, for emitted
/// guards (they address it directly). Keep in sync with the `TlbEntry`
/// layout (stride 16 — pinned by the assert below).
pub const TLB_VICTIM_OFF: u32 = TLB_SIZE as u32 * 16;
/// Permission bit: reads allowed.
pub const TLB_READ: u32 = 0x1;
/// Permission bit: writes allowed.
pub const TLB_WRITE: u32 = 0x2;
/// Code-page bit for split generations: set when a cached translation
/// block covers the page. Native store fast paths advance the write
/// generation only through code pages — data-only stores skip the bump,
/// so links and validated entries survive them (store loops chain).
/// Packed into `perms` (bit 2 is free): no layout/stride change, and the
/// existing tag+permission gate (which masks 0x1/0x2) is unaffected.
pub const TLB_CODE: u32 = 0x4;

/// One cached page translation. `repr(C)`: emitted code addresses these
/// fields directly (`tag` at +0, `perms` at +4, `ptr` at +8).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct TlbEntry {
    /// `(vpn << 1) | effective_privilege` (M2: mode changes never flush).
    pub tag: u32,
    /// [`TLB_READ`] / [`TLB_WRITE`] bits.
    pub perms: u32,
    /// Host page base (valid only when `perms != 0`).
    pub ptr: u64,
}

/// Emitted guards stride entries by 16 (`shl 4`); the victim offset
/// above assumes the same stride.
const _: () = assert!(core::mem::size_of::<TlbEntry>() == 16);

/// Direct-mapped guest-page translation cache, boxed by the runner for a
/// stable address across calls.
#[derive(Clone, Debug)]
pub struct Tlb {
    /// Entries indexed by `vpn & TLB_MASK`.
    pub entries: [TlbEntry; TLB_SIZE],
    /// Most-recently-evicted entry (P3-6): emitted guards probe it after
    /// a primary miss, so two colliding hot pages both stay resident.
    /// The code bit rides along on eviction (stale-set is safe: extra
    /// generation bumps, never missed ones — same rationale as SPLIT).
    pub victim: TlbEntry,
}

impl Tlb {
    /// Zeroed (all-denying) table.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entries: [TlbEntry {
                tag: 0,
                perms: 0,
                ptr: 0,
            }; TLB_SIZE],
            victim: TlbEntry {
                tag: 0,
                perms: 0,
                ptr: 0,
            },
        }
    }

    /// Drop every entry (config change: mappings may have moved).
    pub fn clear(&mut self) {
        *self = Self::new();
    }

    /// Mark the live entry for `vpn` as code-covered (split generations):
    /// a translation block was just cached over the page. Only touches
    /// the entry when its tag already names this `vpn` (either privilege
    /// — the tag's low bit is the mode, stripped for the compare);
    /// collided or empty entries are left alone (the next fill stamps the
    /// bit fresh from the page index). Stale-set bits are safe (extra
    /// bumps, never missed ones); stale-clear is impossible — inserts
    /// always mark.
    pub fn mark_code_page(&mut self, vpn: u32) {
        let e = &mut self.entries[(vpn & TLB_MASK) as usize];
        if e.tag >> 1 == vpn {
            e.perms |= TLB_CODE;
        }
    }
}

impl Default for Tlb {
    fn default() -> Self {
        Self::new()
    }
}

/// Fill the entry for `addr` (write side when `is_write`): full bus
/// translation plus live MPU checks for BOTH directions (permissions are
/// cached per direction, so one fill serves later reads and writes).
/// `is_code` stamps the code bit when a cached block already covers the
/// page (the runner's page index); later inserts mark live entries, so
/// both orders are covered. Returns whether an entry was installed
/// (`false`: unmapped, denied, or spanning — keep side-exiting and let
/// the interpreter own the access, faults included).
pub fn fill_tlb<B: PeripheralBus>(
    tlb: &mut Tlb,
    bus: &mut B,
    addr: u32,
    size: u32,
    is_write: bool,
    privileged: bool,
    ipsr: u8,
    is_code: bool,
) -> bool {
    use crate::mem::DataTranslate;

    if size == 0 || size > 4 {
        return false;
    }
    let vpn = addr >> 12;
    let offset = addr & 0xFFF;
    // Single-page fit (the template enforces this too; defense in depth).
    if offset + size > 0x1000 {
        return false;
    }
    let access = if is_write {
        MpuAccess::Write
    } else {
        MpuAccess::Read
    };
    let (ptr, len) = match bus.translate(addr, access) {
        DataTranslate::Direct { ptr, len } => (ptr as u64, len),
        DataTranslate::Indirect => return false,
    };
    if offset + size > len {
        return false;
    }
    // Whole-page coverage only: entries claim the full page, so a
    // partially-mapped page must never install (a later same-page access
    // past `len` would hit a stale tag and escape the mapping).
    if len < 0x1000 {
        return false;
    }
    // Permissions for both directions under the CURRENT mode (M2: the tag
    // records the effective privilege, so later mode switches hit their
    // own entries instead of stale ones).
    let mut perms = 0;
    if bus.mpu_check(addr, MpuAccess::Read, privileged, ipsr) {
        perms |= TLB_READ;
    }
    if bus.mpu_check(addr, MpuAccess::Write, privileged, ipsr) {
        perms |= TLB_WRITE;
    }
    let want = if is_write { TLB_WRITE } else { TLB_READ };
    if perms & want == 0 {
        // Denied: do not cache (the interpreter faults identically on
        // every such access; faults are cold).
        return false;
    }
    // `translate` already returns the page-relative host base (host
    // address of the guest page start) — store it directly (an earlier
    // revision subtracted the offset twice, landing a page low; caught
    // by differential).
    if is_code {
        perms |= TLB_CODE;
    }
    // The evicted entry becomes the victim (even when empty: a denying
    // victim just misses, which is correct). No swap-back on victim hits
    // — the asymmetric steady state is stable and keeps the fast path
    // branch-free.
    tlb.victim = tlb.entries[(vpn & TLB_MASK) as usize];
    tlb.entries[(vpn & TLB_MASK) as usize] = TlbEntry {
        tag: (vpn << 1) | u32::from(privileged),
        perms,
        ptr,
    };
    true
}

#[cfg(test)]
mod tests {
    use crate::mem::flat::FlatMemory;

    use super::*;

    fn bus() -> FlatMemory {
        FlatMemory::with_layout(&[0x2000_0000], &[0x2000])
    }

    #[test]
    fn fill_installs_and_clear_drops() {
        let mut tlb = Tlb::new();
        let mut mem = bus();
        assert!(fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0100,
            4,
            false,
            true,
            0,
            false
        ));
        let e = &tlb.entries[(0x2000_0100 >> 12) as usize & TLB_MASK as usize];
        assert_eq!(e.tag, (0x2000_0100 >> 12) << 1 | 1);
        assert_eq!(e.perms, TLB_READ | TLB_WRITE);
        assert_ne!(e.ptr, 0);
        // Privilege tags differ (M2).
        assert!(fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0100,
            4,
            false,
            false,
            0,
            false
        ));
        let e2 = &tlb.entries[(0x2000_0100 >> 12) as usize & TLB_MASK as usize];
        assert_eq!(e2.tag, (0x2000_0100 >> 12) << 1);
        tlb.clear();
        assert_eq!(tlb.entries[0].perms, 0);
    }

    #[test]
    fn fill_refuses_unmapped_and_spanning() {
        let mut tlb = Tlb::new();
        let mut mem = bus();
        assert!(!fill_tlb(
            &mut tlb,
            &mut mem,
            0x4000_0000,
            4,
            false,
            true,
            0,
            false
        ));
        assert!(!fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0FFF,
            4,
            false,
            true,
            0,
            false
        ));
        assert!(!fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0100,
            0,
            false,
            true,
            0,
            false
        ));
        assert!(!fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0100,
            8,
            false,
            true,
            0,
            false
        ));
    }

    #[test]
    fn fill_evicts_prior_entry_to_victim() {
        // vpn 0x20000 and 0x20040 share index 0: the colliding pair.
        let mut tlb = Tlb::new();
        let mut mem = FlatMemory::with_layout(&[0x2000_0000], &[0x50000]);
        assert!(fill_tlb(
            &mut tlb, &mut mem, 0x2000_0080, 4, false, true, 0, true
        ));
        assert_eq!(tlb.entries[0].tag, (0x20000 << 1) | 1);
        assert_eq!(tlb.entries[0].perms, TLB_READ | TLB_WRITE | TLB_CODE);
        assert_eq!(tlb.victim.perms, 0);
        assert!(fill_tlb(
            &mut tlb, &mut mem, 0x2004_0080, 4, true, true, 0, false
        ));
        assert_eq!(tlb.entries[0].tag, (0x20040 << 1) | 1);
        // The evicted entry (code bit included) now lives in the victim.
        assert_eq!(tlb.victim.tag, (0x20000 << 1) | 1);
        assert_eq!(tlb.victim.perms, TLB_READ | TLB_WRITE | TLB_CODE);
        assert_ne!(tlb.victim.ptr, 0);
        // Ping-pong is stable: re-filling the first page parks the second.
        assert!(fill_tlb(
            &mut tlb, &mut mem, 0x2000_0080, 4, false, true, 0, false
        ));
        assert_eq!(tlb.entries[0].tag, (0x20000 << 1) | 1);
        assert_eq!(tlb.victim.tag, (0x20040 << 1) | 1);
    }

    #[test]
    fn zeroed_entries_deny() {
        let tlb = Tlb::new();
        assert_eq!(tlb.entries[0].perms, 0);
        assert_eq!(tlb.victim.perms, 0);
    }

    #[test]
    fn code_bit_rides_fill_and_mark() {
        let mut tlb = Tlb::new();
        let mut mem = bus();
        let vpn = 0x2000_0100 >> 12;
        let idx = (vpn & TLB_MASK) as usize;
        // Plain fill: no code bit; the permission gate still masks it out.
        assert!(fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0100,
            4,
            false,
            true,
            0,
            false
        ));
        assert_eq!(tlb.entries[idx].perms, TLB_READ | TLB_WRITE);
        // Fill with coverage stamps the bit (perm checks unaffected).
        assert!(fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0100,
            4,
            false,
            true,
            0,
            true
        ));
        assert_eq!(tlb.entries[idx].perms, TLB_READ | TLB_WRITE | TLB_CODE);
        // Marking sets it on a tag-matching live entry (either mode).
        let mut tlb2 = Tlb::new();
        assert!(fill_tlb(
            &mut tlb2,
            &mut mem,
            0x2000_0100,
            4,
            false,
            false,
            0,
            false
        ));
        tlb2.mark_code_page(vpn);
        assert_eq!(
            tlb2.entries[(vpn & TLB_MASK) as usize].perms,
            TLB_READ | TLB_WRITE | TLB_CODE
        );
        // ...but never on a foreign or empty slot.
        let mut tlb3 = Tlb::new();
        tlb3.mark_code_page(vpn);
        assert_eq!(tlb3.entries[(vpn & TLB_MASK) as usize].perms, 0);
    }
}
