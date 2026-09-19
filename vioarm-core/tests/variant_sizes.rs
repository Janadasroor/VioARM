//! Phase-0 variant sanity (boards-roadmap): every per-Dname board carries
//! its PDSC flash/RAM extents — last byte of each bank touches silicon,
//! first byte past flash is `Reserved`.

use vioarm_core::board;
use vioarm_core::device::svd::per_dname::H7_DTCM;
use vioarm_core::mem::Bus;

#[test]
fn every_per_dname_board_has_exact_flash_ram_extents() {
    let mut count = 0u32;
    for (dname, entry) in board::all_per_dnames() {
        let mut mem = board::core_only_board_per_dname(dname)
            .unwrap_or_else(|| panic!("{dname}: no per-dname board"));
        let flash_top = 0x0800_0000u32 + entry.flash_len;
        assert!(
            mem.read(flash_top - 1).is_ok(),
            "{dname}: flash top unreadable"
        );
        assert!(
            mem.read(flash_top).is_err(),
            "{dname}: past-flash must be Reserved"
        );
        assert!(
            mem.read(0x2000_0000).is_ok(),
            "{dname}: ram base unreadable"
        );
        if entry.donor.starts_with("stm32h") && entry.ram_len > H7_DTCM {
            let axi = entry.ram_len - H7_DTCM;
            assert!(
                mem.read(0x2000_0000 + H7_DTCM - 1).is_ok(),
                "{dname}: dtcm top unreadable"
            );
            assert!(
                mem.read(0x2400_0000 + axi - 1).is_ok(),
                "{dname}: axi top unreadable"
            );
        } else {
            assert!(
                mem.read(0x2000_0000 + entry.ram_len - 1).is_ok(),
                "{dname}: ram top unreadable"
            );
        }
        count += 1;
    }
    assert!(count >= 500, "expected >=500 dnames, got {count}");
}
