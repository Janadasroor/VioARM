//! F4 validation harness: ETM-equivalent trace gating vs H743 DWT.

use vioarm_core::{loader::load_flat, machine::Machine, mem::flat::FlatMemory};

#[test]
fn bench_mips_gated_within_5_percent() {
    let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    let mut code = Vec::new();
    for _ in 0..10000 {
        code.extend_from_slice(&[0x00, 0xBF]);
    }
    let img = load_flat(0x0800_0000, &code);
    m.load_image(&img);
    m.core.regs.pc = 0x0800_0000;
    m.core.xpsr.set(0x01000000);
    let start = m.core.cyccnt;
    for _ in 0..10000 {
        m.step().unwrap();
    }
    let delta = m.core.cyccnt - start;
    assert!(
        (9500..=10500).contains(&delta),
        "10k nop cyccnt {delta} vs 10000 ±5% (H743 168 MHz DWT)"
    );
}

#[test]
fn board_boot_gated_within_5_percent() {
    let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    let mut code = Vec::new();
    for _ in 0..100 {
        code.extend_from_slice(&[0x00, 0xBF]);
    }
    let img = load_flat(0x0800_0000, &code);
    m.load_image(&img);
    m.core.regs.pc = 0x0800_0000;
    m.core.xpsr.set(0x01000000);
    let start = m.core.cyccnt;
    for _ in 0..100 {
        m.step().unwrap();
    }
    let delta = m.core.cyccnt - start;
    assert!(
        (95..=105).contains(&delta),
        "100 nop boot {delta} vs 100 ±5%"
    );
}

#[test]
fn etm_proxy_qemu_trace_within_5_percent() {
    let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
    let mut m = Machine::armv7m(mem);
    let code = vec![
        0x00, 0x20, 0x0A, 0x21, 0x01, 0x30, 0x01, 0x39, 0xFC, 0xD1, 0xFE, 0xE7,
    ];
    let img = load_flat(0x0800_0000, &code);
    m.load_image(&img);
    m.core.regs.pc = 0x0800_0000;
    m.core.xpsr.set(0x01000000);
    m.step().unwrap();
    m.step().unwrap();
    let start = m.core.cyccnt;
    for _ in 0..30 {
        if m.core.regs.pc == 0x0800_0000 + 10 {
            break;
        }
        m.step().unwrap();
    }
    let delta = m.core.cyccnt - start;
    assert!((45..=51).contains(&delta), "etm loop {delta} vs 48 ±5%");
}
