//! Machine-driven quantum tests (WP-R5): every MUST in
//! `docs/src/coupling-contract.md` has a test here.
//!
//! Firmware blobs are copied verbatim from the proven C-ABI suites
//! (`vioarm-cosim/tests/pins.rs` echo, `analog_bridge.rs` echo) so no new
//! Thumb encodings are invented in this file.

use vioarm_core::board::BoardMemory;
use vioarm_core::couple::{cyccnt_to_seconds, seconds_to_cycles, CoupledEventKind};
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;

fn boot_virtual(fw: &[u8]) -> Machine<BoardMemory> {
    let mut m =
        Machine::for_name_bare("virtual", &load_flat(0x0, fw)).expect("virtual board exists");
    m.set_vector_base(0x0);
    m.reset().expect("reset succeeds");
    m
}

/// `b .` spin at the reset target (from `vioarm-cosim/tests/smoke.rs`).
fn spin_firmware() -> [u8; 10] {
    [0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0xfe, 0xe7]
}

/// DIR pin0+pin2 output, OUT pin0 high, then `OUT[2] = IN[1]` forever
/// (from `vioarm-cosim/tests/pins.rs`).
fn echo_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x0A, 0x48, 0x05, 0x21, 0x01, 0x60, 0x0A,
        0x48, 0x01, 0x21, 0x01, 0x60, 0x09, 0x48, 0x0A, 0x49, 0x02, 0x68, 0x02, 0x23, 0x1A, 0x42,
        0x04, 0xD0, 0x0A, 0x68, 0x04, 0x23, 0x1A, 0x43, 0x0A, 0x60, 0xF6, 0xE7, 0x0A, 0x68, 0x04,
        0x23, 0x9A, 0x43, 0x0A, 0x60, 0xF1, 0xE7,
    ];
    for word in [0x4000_0000u32, 0x4000_0004, 0x4000_0008, 0x4000_0004] {
        fw.extend_from_slice(&word.to_le_bytes());
    }
    fw
}

/// Enable the `apin` window, then forever `TXMV = RXMV; TXGO` (from
/// `vioarm-cosim/tests/analog_bridge.rs`).
fn analog_echo_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x08, 0x48, 0x01, 0x21, 0x01, 0x60, 0x08,
        0x48, 0x08, 0x49, 0x01, 0x60, 0x08, 0x48, 0x01, 0x21, 0x01, 0x60, 0x08, 0x48, 0x08, 0x49,
        0x09, 0x4A, 0x03, 0x68, 0x0B, 0x60, 0x01, 0x23, 0x13, 0x60, 0xFA, 0xE7, 0x00, 0x00,
    ];
    for word in [
        0x4000_2010u32,
        0x4000_2000,
        3300,
        0x4000_2004,
        0x4000_2008,
        0x4000_2000,
        0x4000_2004,
    ] {
        fw.extend_from_slice(&word.to_le_bytes());
    }
    fw
}

#[test]
fn quantum_never_exceeds_budget() {
    let mut m = boot_virtual(&spin_firmware());
    let outcome = m.run_quantum(8000, &[]);
    assert!(outcome.halt.is_none());
    assert!(outcome.steps > 0);
    let consumed = outcome.consumed_cycles();
    assert!(consumed >= 8000, "budget met, got {consumed}");
    assert!(consumed - 8000 <= 16, "one-step overshoot, got {consumed}");
}

#[test]
fn zero_quantum_retires_nothing() {
    let mut m = boot_virtual(&spin_firmware());
    let start = m.cyccnt();
    let outcome = m.run_quantum(0, &[(0, 0)]);
    assert_eq!(outcome.steps, 0);
    assert!(outcome.events.is_empty());
    assert!(outcome.halt.is_none());
    assert_eq!(outcome.start_cyccnt, start);
    assert_eq!(outcome.end_cyccnt, start);
}

#[test]
fn stamps_within_quantum_bounds() {
    let mut m = boot_virtual(&echo_firmware());
    m.gpio_drive(0, 1 << 1);
    let outcome = m.run_quantum(8000, &[(0, 0), (0, 2)]);
    assert!(!outcome.events.is_empty());
    for event in &outcome.events {
        assert!(event.cyccnt > outcome.start_cyccnt);
        assert!(event.cyccnt <= outcome.end_cyccnt);
    }
}

#[test]
fn pin_toggle_emits_change_only_events() {
    let fw = echo_firmware();
    let mut m = boot_virtual(&fw);
    m.gpio_drive(0, 1 << 1);
    let outcome = m.run_quantum(16_000, &[(0, 2)]);
    let pin2: Vec<_> = outcome
        .events
        .iter()
        .filter_map(|event| match event.kind {
            CoupledEventKind::Pin { pin: 2, level, .. } => Some((event.cyccnt, level)),
            _ => None,
        })
        .collect();
    assert!(!pin2.is_empty(), "driven input copies to pin 2");
    assert!(pin2.iter().any(|(_, level)| *level), "pin 2 ends high");
    m.gpio_drive(0, 0);
    let outcome = m.run_quantum(16_000, &[(0, 2)]);
    let pin2: Vec<_> = outcome
        .events
        .iter()
        .filter_map(|event| match event.kind {
            CoupledEventKind::Pin { pin: 2, level, .. } => Some(level),
            _ => None,
        })
        .collect();
    assert!(pin2.contains(&false), "release copies back low");
    for pair in outcome.events.windows(2) {
        if let (
            CoupledEventKind::Pin {
                port: a,
                pin: b,
                level: c,
                driving: d,
            },
            CoupledEventKind::Pin {
                port: e,
                pin: f,
                level: g,
                driving: h,
            },
        ) = (pair[0].kind, pair[1].kind)
        {
            assert!(
                (a, b, c, d) != (e, f, g, h),
                "no duplicate consecutive pin rows"
            );
        }
    }
}

#[test]
fn analog_strobe_emits_one_event_per_word() {
    let mut m = boot_virtual(&analog_echo_firmware());
    let outcome = m.run_quantum(4000, &[]);
    let staged: Vec<u32> = outcome
        .events
        .iter()
        .filter_map(|event| match event.kind {
            CoupledEventKind::Analog { mv } => Some(mv),
            _ => None,
        })
        .collect();
    assert!(
        staged.contains(&3300),
        "first strobe publishes, got {staged:?}"
    );
    assert!(m.apin_inject_mv(1500), "window enabled by firmware");
    let outcome = m.run_quantum(16_000, &[]);
    let echoed: Vec<u32> = outcome
        .events
        .iter()
        .filter_map(|event| match event.kind {
            CoupledEventKind::Analog { mv } => Some(mv),
            _ => None,
        })
        .collect();
    assert!(
        echoed.contains(&1500),
        "injected word echoes back, got {echoed:?}"
    );
}

#[test]
fn stamps_monotone_and_ordered() {
    let mut m = boot_virtual(&echo_firmware());
    m.gpio_drive(0, 1 << 1);
    let outcome = m.run_quantum(16_000, &[(0, 0), (0, 1), (0, 2)]);
    assert!(outcome.events.len() > 1);
    for pair in outcome.events.windows(2) {
        assert!(
            pair[0].cyccnt <= pair[1].cyccnt,
            "stamps never reorder: {} > {}",
            pair[0].cyccnt,
            pair[1].cyccnt
        );
    }
}

#[test]
fn rerun_is_byte_identical() {
    let run = || {
        let mut m = boot_virtual(&echo_firmware());
        m.gpio_drive(0, 1 << 1);
        let first = m.run_quantum(8000, &[(0, 0), (0, 2)]);
        let second = m.run_quantum(8000, &[(0, 0), (0, 2)]);
        let mut events = first.events;
        events.extend(second.events);
        (events, m.core.regs.pc, m.cyccnt())
    };
    let (a_events, a_pc, a_cyccnt) = run();
    let (b_events, b_pc, b_cyccnt) = run();
    assert_eq!((a_pc, a_cyccnt), (b_pc, b_cyccnt));
    assert_eq!(a_events, b_events);
}

#[test]
fn quanta_match_run_cycles_gpio() {
    let drive = 1u32 << 1;
    let mut single = boot_virtual(&echo_firmware());
    single.gpio_drive(0, drive);
    let halt = single.run_cycles(16_000);
    assert!(matches!(
        halt,
        Err(vioarm_core::error::HaltCause::CycleBudgetExhausted)
    ));
    let (single_pc, single_cyccnt) = (single.core.regs.pc, single.cyccnt());
    let single_pin = single.gpio_pin(0, 2);

    let mut quanta = boot_virtual(&echo_firmware());
    quanta.gpio_drive(0, drive);
    for _ in 0..4 {
        let outcome = quanta.run_quantum(4000, &[(0, 2)]);
        assert!(outcome.halt.is_none());
    }
    assert_eq!(quanta.core.regs.pc, single_pc, "same final pc");
    assert_eq!(quanta.cyccnt(), single_cyccnt, "same final cyccnt");
    assert_eq!(quanta.gpio_pin(0, 2), single_pin, "same final pin");
}

#[test]
fn quanta_match_run_cycles_analog() {
    let mut single = boot_virtual(&analog_echo_firmware());
    let _ = single.run_cycles(4000);
    single.apin_inject_mv(1500);
    let halt = single.run_cycles(16_000);
    assert!(matches!(
        halt,
        Err(vioarm_core::error::HaltCause::CycleBudgetExhausted)
    ));
    let single_tail = single.apin_drain_mv();

    let mut quanta = boot_virtual(&analog_echo_firmware());
    quanta.run_quantum(4000, &[]);
    quanta.apin_inject_mv(1500);
    let mut echoed = Vec::new();
    for _ in 0..4 {
        let outcome = quanta.run_quantum(4000, &[]);
        assert!(outcome.halt.is_none());
        echoed.extend(outcome.events.iter().filter_map(|event| match event.kind {
            CoupledEventKind::Analog { mv } => Some(mv),
            _ => None,
        }));
    }
    assert_eq!(quanta.core.regs.pc, single.core.regs.pc, "same final pc");
    assert_eq!(quanta.cyccnt(), single.cyccnt(), "same final cyccnt");
    assert_eq!(
        echoed.last(),
        single_tail.as_ref(),
        "same final analog word"
    );
}

#[test]
fn cycle_time_uses_live_cpu_hz() {
    let mut m = boot_virtual(&spin_firmware());
    assert_eq!(m.cpu_hz(), 8_000_000, "virtual board runs HSI 8 MHz");
    assert_eq!(seconds_to_cycles(0.001, m.cpu_hz()), 8000);
    let outcome = m
        .run_quantum_seconds(0.001, &[])
        .expect("finite non-negative budget");
    assert!(outcome.consumed_cycles() >= 8000);
    let seconds = cyccnt_to_seconds(outcome.consumed_cycles(), 8_000_000);
    assert!((seconds - 0.001).abs() < 1e-5, "got {seconds}");
    assert!(m.run_quantum_seconds(-1.0, &[]).is_none());
    assert!(m.run_quantum_seconds(f64::NAN, &[]).is_none());
}
