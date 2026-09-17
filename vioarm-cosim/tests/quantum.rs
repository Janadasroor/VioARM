//! Machine-driven quantum C ABI tests (WP-R5): `vioarm_cosim_run_quantum`
//! over spin / gpio-echo / analog-echo firmware, plus truncation and
//! fail-closed misuse probes.
//!
//! Firmware blobs are copied verbatim from the proven suites (`smoke.rs`
//! spin, `pins.rs` echo, `analog_bridge.rs` echo) so no new Thumb
//! encodings are invented here.

use std::ffi::{CStr, CString};
use std::ptr;
use vioarm_cosim::{
    vioarm_cosim_create, vioarm_cosim_destroy, vioarm_cosim_pin_set, vioarm_cosim_run_quantum,
    vioarm_cosim_set_external_voltage, vioarm_cosim_step_duration, VioArmEvent, VIOARM_ERR_BAD_ARG,
    VIOARM_ERR_NULL_HANDLE, VIOARM_EVENT_ANALOG, VIOARM_EVENT_PIN, VIOARM_FW_BIN,
    VIOARM_STEP_HALTED, VIOARM_STEP_OK,
};

static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn serial() -> std::sync::MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|poison| poison.into_inner())
}

fn last_error() -> String {
    // SAFETY: valid NUL-terminated pointer until the next failing call on
    // this thread; copied immediately.
    unsafe { CStr::from_ptr(vioarm_cosim::vioarm_cosim_last_error()) }
        .to_string_lossy()
        .into_owned()
}

fn create(board: &str, fw: &[u8], base: u32) -> *mut vioarm_cosim::VioArmHandle {
    let name = CString::new(board).unwrap();
    let handle = vioarm_cosim_create(name.as_ptr(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, base);
    assert!(!handle.is_null(), "create failed: {}", last_error());
    handle
}

fn blank_event() -> VioArmEvent {
    VioArmEvent {
        cyccnt: 0,
        kind: 0,
        port: 0,
        pin: 0,
        level: 0,
        driving: 0,
        mv: 0,
    }
}

/// Minimal M0 firmware at base `0x0`: SP + reset vector, then `b .`
/// (from `smoke.rs`).
fn spin_firmware() -> [u8; 10] {
    [0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0xfe, 0xe7]
}

/// DIR pin0+pin2 output, OUT pin0 high, then `OUT[2] = IN[1]` forever
/// (from `pins.rs`).
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
/// `analog_bridge.rs`).
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

fn run_quantum(
    handle: *mut vioarm_cosim::VioArmHandle,
    seconds: f64,
    cap: usize,
) -> (i32, Vec<VioArmEvent>, usize) {
    let mut buf = vec![blank_event(); cap];
    let mut out_len = usize::MAX;
    let code = vioarm_cosim_run_quantum(
        handle,
        seconds,
        if cap == 0 {
            ptr::null_mut()
        } else {
            buf.as_mut_ptr()
        },
        cap,
        &mut out_len,
    );
    buf.truncate(out_len.min(cap));
    (code, buf, out_len)
}

#[test]
fn spin_quantum_ok_no_events() {
    let _serial = serial();
    let fw = spin_firmware();
    let handle = create("virtual", &fw, 0x0);
    let (code, events, total) = run_quantum(handle, 0.001, 64);
    assert_eq!(code, VIOARM_STEP_OK, "spin never halts: {}", last_error());
    assert_eq!(total, 0, "steady spin emits nothing, got {events:?}");
    assert!(events.is_empty());
    vioarm_cosim_destroy(handle);
}

#[test]
fn pin_echo_quantum_monotone() {
    let _serial = serial();
    let fw = echo_firmware();
    let handle = create("virtual", &fw, 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    assert_eq!(vioarm_cosim_pin_set(handle, 0, 1, 1), 0);
    let (code, events, total) = run_quantum(handle, 0.001, 256);
    assert_eq!(code, VIOARM_STEP_OK, "echo never halts: {}", last_error());
    assert_eq!(total, events.len());
    assert!(!events.is_empty(), "driven input copies to pin 2");
    for window in events.windows(2) {
        assert!(
            window[0].cyccnt <= window[1].cyccnt,
            "stamps monotone: {} > {}",
            window[0].cyccnt,
            window[1].cyccnt
        );
    }
    for event in &events {
        assert!(
            event.kind == VIOARM_EVENT_PIN || event.kind == VIOARM_EVENT_ANALOG,
            "known kind, got {}",
            event.kind
        );
        if event.kind == VIOARM_EVENT_PIN {
            assert!(event.port < 4, "watched ports A-D, got {}", event.port);
            assert!(event.pin < 16, "pin range, got {}", event.pin);
            assert!(event.level <= 1 && event.driving <= 1);
        }
    }
    assert!(
        events.iter().any(|event| event.kind == VIOARM_EVENT_PIN
            && event.port == 0
            && event.pin == 2
            && event.level == 1),
        "pin 2 echoes high: {events:?}"
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn analog_echo_quantum_publishes() {
    let _serial = serial();
    let fw = analog_echo_firmware();
    let handle = create("virtual", &fw, 0x0);
    // No `step_duration` first: the SPICE-master path leaves staged words
    // pending as most-recent (DAC semantics), so a mixed-path preamble
    // would supersede the 3300 strobe with loop echoes of 0. Start
    // machine-driven from reset, like `couple_quantum` does.
    let (code, events, _) = run_quantum(handle, 0.000_5, 1024);
    assert_eq!(
        code,
        VIOARM_STEP_OK,
        "analog echo never halts: {}",
        last_error()
    );
    assert!(
        events
            .iter()
            .any(|event| event.kind == VIOARM_EVENT_ANALOG && event.mv == 3300),
        "first strobe publishes 3300: {events:?}"
    );
    assert_eq!(
        vioarm_cosim_set_external_voltage(handle, 0, 1.5),
        0,
        "{}",
        last_error()
    );
    let (code, events, _) = run_quantum(handle, 0.001, 4096);
    assert_eq!(code, VIOARM_STEP_OK, "{}", last_error());
    assert!(
        events
            .iter()
            .any(|event| event.kind == VIOARM_EVENT_ANALOG && event.mv == 1500),
        "injected word echoes: {events:?}"
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn truncation_reports_total() {
    let _serial = serial();
    let fw = analog_echo_firmware();
    let handle = create("virtual", &fw, 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    let (_, _, total) = run_quantum(handle, 0.001, 4096);
    assert!(total > 1, "soak emits dense analog words, got {total}");
    vioarm_cosim_destroy(handle);

    let handle = create("virtual", &fw, 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    let (code, events, truncated_total) = run_quantum(handle, 0.001, 1);
    assert_eq!(
        code,
        VIOARM_STEP_OK,
        "truncation still retires: {}",
        last_error()
    );
    assert!(
        truncated_total > 1,
        "total reported past cap, got {truncated_total}"
    );
    assert_eq!(events.len(), 1, "buffer holds first event in retire order");
    assert!(
        last_error().contains("truncated"),
        "truncation noted: {}",
        last_error()
    );
    // Zero-cap probe: null buffer, total still reported.
    vioarm_cosim_destroy(handle);

    let handle = create("virtual", &fw, 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    let mut out_len = usize::MAX;
    let code = vioarm_cosim_run_quantum(handle, 0.001, ptr::null_mut(), 0, &mut out_len);
    assert_eq!(code, VIOARM_STEP_OK, "{}", last_error());
    assert!(out_len > 0, "zero-cap probe reports total");
    vioarm_cosim_destroy(handle);
}

#[test]
fn quantum_bad_args_fail_closed() {
    let _serial = serial();
    let fw = spin_firmware();
    let handle = create("virtual", &fw, 0x0);
    let mut buf = vec![blank_event(); 8];
    let mut out_len = 0usize;
    assert_eq!(
        vioarm_cosim_run_quantum(ptr::null_mut(), 0.001, buf.as_mut_ptr(), 8, &mut out_len),
        VIOARM_ERR_NULL_HANDLE
    );
    assert_eq!(
        vioarm_cosim_run_quantum(handle, 0.001, buf.as_mut_ptr(), 8, ptr::null_mut()),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_run_quantum(handle, 0.001, ptr::null_mut(), 8, &mut out_len),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_run_quantum(handle, -1.0, buf.as_mut_ptr(), 8, &mut out_len),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_run_quantum(handle, f64::NAN, buf.as_mut_ptr(), 8, &mut out_len),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_run_quantum(handle, f64::INFINITY, buf.as_mut_ptr(), 8, &mut out_len),
        VIOARM_ERR_BAD_ARG
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn quantum_halt_returns_halted() {
    let _serial = serial();
    let fw = [0xFFu8; 16];
    let handle = create("virtual", &fw, 0x0);
    let (code, _, _) = run_quantum(handle, 0.001, 64);
    assert_eq!(code, VIOARM_STEP_HALTED);
    assert!(last_error().contains("halted"), "{}", last_error());
    vioarm_cosim_destroy(handle);
}

#[test]
fn quantum_rerun_byte_identical() {
    let _serial = serial();
    let run = || {
        let fw = echo_firmware();
        let handle = create("virtual", &fw, 0x0);
        assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
        assert_eq!(vioarm_cosim_pin_set(handle, 0, 1, 1), 0);
        let (code, events, _) = run_quantum(handle, 0.001, 1024);
        assert_eq!(code, VIOARM_STEP_OK);
        vioarm_cosim_destroy(handle);
        events
    };
    assert_eq!(run(), run(), "same firmware + quanta rerun identically");
}
