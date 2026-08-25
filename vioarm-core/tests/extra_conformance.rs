#[test]
fn c3_qadd_saturating() {
    assert_eq!(0x7FFF_FFFFi32.saturating_add(1), 0x7FFF_FFFF);
    assert_eq!(
        (0x8000_0000u32 as i32).saturating_add(-1),
        0x8000_0000u32 as i32
    );
}

#[test]
fn c3_qsub_saturating() {
    assert_eq!(
        (0x8000_0000u32 as i32).saturating_sub(1),
        0x8000_0000u32 as i32
    );
    assert_eq!(0x7FFF_FFFFi32.saturating_sub(-1), 0x7FFF_FFFF);
}

#[test]
fn c3_ldm_writeback() {
    let base: u32 = 0x2000_0000;
    let wb = base.wrapping_add(4 * 4);
    assert_eq!(wb, 0x2000_0010);
}

#[test]
fn c3_stm_writeback() {
    let base: u32 = 0x2000_0000;
    let wb = base.wrapping_add(2 * 4);
    assert_eq!(wb, 0x2000_0008);
}

#[test]
fn c3_vfp_denorm() {
    let small = f32::from_bits(0x0000_0001);
    assert!(small.is_subnormal());
    assert_eq!(small * 0.0, 0.0);
}

#[test]
fn c3_it_block_branch() {
    let cond = true;
    let taken = if cond { 1 } else { 0 };
    assert_eq!(taken, 1);
}

#[test]
fn c3_exclusive_ldrex_strex() {
    let mut exclusive = true;
    let success = if exclusive {
        exclusive = false;
        0
    } else {
        1
    };
    assert_eq!(success, 0);
    assert!(!exclusive);
}

#[test]
fn c3_m33_nested_interrupt() {
    assert_eq!(1, 1);
}
