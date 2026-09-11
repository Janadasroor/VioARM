//! Slice-2 analog tests: SPICE-volts in, DAC-volts out through the C ABI
//! on the `virtual` board (`apin` window at `0x4000_2000`).
//!
//! Firmware is hand-assembled Thumb: enable the window (`CR.EN`), then
//! either stage one DAC word and park, or echo `RXMV -> TXMV` forever.

use std::ffi::{CStr, CString};
use std::ptr;
use vioarm_cosim::{
    vioarm_cosim_create, vioarm_cosim_destroy, vioarm_cosim_get_dac_voltage,
    vioarm_cosim_set_external_voltage, vioarm_cosim_step_duration, VIOARM_FW_BIN, VIOARM_STEP_OK,
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

fn create(fw: &[u8]) -> *mut vioarm_cosim::VioArmHandle {
    let name = CString::new("virtual").unwrap();
    let handle = vioarm_cosim_create(name.as_ptr(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, 0x0);
    assert!(!handle.is_null(), "create failed: {}", last_error());
    handle
}

fn dac_volts(handle: *mut vioarm_cosim::VioArmHandle) -> f64 {
    let mut volts = -1.0;
    let code = vioarm_cosim_get_dac_voltage(handle, 0, &mut volts);
    assert_eq!(code, 0, "get_dac_voltage failed: {}", last_error());
    volts
}

/// Enable the window, stage 3300 mV, strobe once, park.
fn dac_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, // SP
        0x09, 0x00, 0x00, 0x00, // reset -> 0x08 thumb
        0x04, 0x48, // 08: ldr r0, =CR
        0x01, 0x21, // 0A: movs r1, #1 (EN)
        0x01, 0x60, // 0C: str r1, [r0]
        0x04, 0x48, // 0E: ldr r0, =TXMV
        0x04, 0x49, // 10: ldr r1, =3300
        0x01, 0x60, // 12: str r1, [r0]
        0x04, 0x48, // 14: ldr r0, =TXGO
        0x01, 0x21, // 16: movs r1, #1
        0x01, 0x60, // 18: str r1, [r0]
        0xFE, 0xE7, // 1A: b .
    ];
    for word in [0x4000_2010u32, 0x4000_2000, 3300, 0x4000_2004] {
        fw.extend_from_slice(&word.to_le_bytes());
    }
    fw
}

/// Enable the window, then forever `TXMV = RXMV; TXGO` (ADC echo).
fn echo_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, // SP
        0x09, 0x00, 0x00, 0x00, // reset -> 0x08 thumb
        0x08, 0x48, // 08: ldr r0, =CR
        0x01, 0x21, // 0A: movs r1, #1 (EN)
        0x01, 0x60, // 0C: str r1, [r0]
        0x08, 0x48, // 0E: ldr r0, =TXMV
        0x08, 0x49, // 10: ldr r1, =3300
        0x01, 0x60, // 12: str r1, [r0]
        0x08, 0x48, // 14: ldr r0, =TXGO
        0x01, 0x21, // 16: movs r1, #1
        0x01, 0x60, // 18: str r1, [r0]
        0x08, 0x48, // 1A: ldr r0, =RXMV
        0x08, 0x49, // 1C: ldr r1, =TXMV
        0x09, 0x4A, // 1E: ldr r2, =TXGO
        0x03, 0x68, // 20: loop: ldr r3, [r0]
        0x0B, 0x60, // 22: str r3, [r1]
        0x01, 0x23, // 24: movs r3, #1
        0x13, 0x60, // 26: str r3, [r2]
        0xFA, 0xE7, // 28: b loop
        0x00, 0x00, // 2A: pad (never executed) so the pool starts at 0x2C
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
fn dac_staged_word_reads_back() {
    let _serial = serial();
    let fw = dac_firmware();
    let handle = create(&fw);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    assert!((dac_volts(handle) - 3.3).abs() < 1e-9);
    vioarm_cosim_destroy(handle);
}

#[test]
fn inject_echo_round_trip() {
    let _serial = serial();
    let fw = echo_firmware();
    let handle = create(&fw);
    // Run once first: the window's `EN` comes from firmware (like an ADC
    // enable), and injection into a disabled window is dropped by design.
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    assert_eq!(
        vioarm_cosim_set_external_voltage(handle, 0, 1.5),
        0,
        "{}",
        last_error()
    );
    assert_eq!(vioarm_cosim_step_duration(handle, 0.001), VIOARM_STEP_OK);
    assert!((dac_volts(handle) - 1.5).abs() < 1e-9);
    // Negative and NaN saturate to 0 mV.
    assert_eq!(vioarm_cosim_set_external_voltage(handle, 0, -1.0), 0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.001), VIOARM_STEP_OK);
    assert!((dac_volts(handle)).abs() < 1e-9);
    assert_eq!(vioarm_cosim_set_external_voltage(handle, 0, f64::NAN), 0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.001), VIOARM_STEP_OK);
    assert!((dac_volts(handle)).abs() < 1e-9);
    vioarm_cosim_destroy(handle);
}

#[test]
fn analog_errors_fail_closed() {
    let _serial = serial();
    let fw = dac_firmware();
    let handle = create(&fw);
    assert_ne!(vioarm_cosim_set_external_voltage(handle, 1, 1.5), 0);
    assert_ne!(
        vioarm_cosim_set_external_voltage(handle, 0, f64::INFINITY),
        0
    );
    assert_ne!(
        vioarm_cosim_set_external_voltage(ptr::null_mut(), 0, 1.5),
        0
    );
    assert_ne!(vioarm_cosim_get_dac_voltage(handle, 1, &mut 0.0), 0);
    assert_ne!(vioarm_cosim_get_dac_voltage(handle, 0, ptr::null_mut()), 0);
    vioarm_cosim_destroy(handle);
}
