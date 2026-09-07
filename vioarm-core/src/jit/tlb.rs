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
/// Permission bit: reads allowed.
pub const TLB_READ: u32 = 0x1;
/// Permission bit: writes allowed.
pub const TLB_WRITE: u32 = 0x2;

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

/// Direct-mapped guest-page translation cache, boxed by the runner for a
/// stable address across calls.
#[derive(Clone, Debug)]
pub struct Tlb {
    /// Entries indexed by `vpn & TLB_MASK`.
    pub entries: [TlbEntry; TLB_SIZE],
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
        }
    }

    /// Drop every entry (config change: mappings may have moved).
    pub fn clear(&mut self) {
        *self = Self::new();
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
/// Returns whether an entry was installed (`false`: unmapped, denied, or
/// spanning — keep side-exiting and let the interpreter own the access,
/// faults included).
pub fn fill_tlb<B: PeripheralBus>(
    tlb: &mut Tlb,
    bus: &mut B,
    addr: u32,
    size: u32,
    is_write: bool,
    privileged: bool,
    ipsr: u8,
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
        assert!(fill_tlb(&mut tlb, &mut mem, 0x2000_0100, 4, false, true, 0));
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
            0
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
            0
        ));
        assert!(!fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0FFF,
            4,
            false,
            true,
            0
        ));
        assert!(!fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0100,
            0,
            false,
            true,
            0
        ));
        assert!(!fill_tlb(
            &mut tlb,
            &mut mem,
            0x2000_0100,
            8,
            false,
            true,
            0
        ));
    }

    #[test]
    fn zeroed_entries_deny() {
        let tlb = Tlb::new();
        assert_eq!(tlb.entries[0].perms, 0);
    }
}
