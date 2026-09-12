//! `Machine::run_cycles`: true-cycle budgets (fetch wait states included).

use vioarm_core::{loader, machine::Machine};

/// M0 `b .` spin at base 0 (halfword `0xe7fe`).
fn spin_image() -> vioarm_core::loader::LoadedImage {
    loader::load_flat(
        0x0,
        &[0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0xfe, 0xe7],
    )
}

#[test]
fn run_cycles_meets_true_cycle_budget() {
    let mut m = Machine::for_name("virtual", &spin_image()).unwrap();
    let start = m.core.cyccnt;
    let outcome = m.run_cycles(1000);
    assert!(matches!(
        outcome,
        Err(vioarm_core::error::HaltCause::CycleBudgetExhausted)
    ));
    let delta = m.core.cyccnt.wrapping_sub(start);
    // Met, overshooting by less than one generous step.
    assert!((1000..1032).contains(&delta), "delta={delta}");
    // Deterministic: same again from the same state class.
    let start2 = m.core.cyccnt;
    let _ = m.run_cycles(1000);
    assert_eq!(m.core.cyccnt.wrapping_sub(start2), delta);
}

#[test]
fn run_cycles_zero_is_a_no_op() {
    let mut m = Machine::for_name("virtual", &spin_image()).unwrap();
    let start = m.core.cyccnt;
    let outcome = m.run_cycles(0);
    assert!(matches!(
        outcome,
        Err(vioarm_core::error::HaltCause::CycleBudgetExhausted)
    ));
    assert_eq!(m.core.cyccnt, start);
}
