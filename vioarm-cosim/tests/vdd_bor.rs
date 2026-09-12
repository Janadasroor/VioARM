//! Supply-rail tests: operating voltage validation, brown-out reboot,
//! and recovery — all through the C ABI on the `virtual` board (whose
//! bus carries the default-wired rail; brown-out is evaluated rail-side,
//! so no PWR device is needed for the reset itself).

use std::ffi::{CStr, CString};
use vioarm_cosim::{
    vioarm_cosim_cpu_hz, vioarm_cosim_create, vioarm_cosim_destroy, vioarm_cosim_pin_get,
    vioarm_cosim_set_operating_voltage, vioarm_cosim_step_duration, VIOARM_ERR_BAD_ARG,
    VIOARM_FW_BIN, VIOARM_MODE_INPUT_FLOATING, VIOARM_MODE_OUTPUT_PUSH_PULL, VIOARM_STEP_OK,
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

/// DIR pin0 out, OUT pin0 high, spin. Generic GPIO at `0x4000_0000`.
/// Literals: DIR@0x18 (imm 3 from 0x08), OUT@0x1C (imm 3 from 0x0E).
fn latch_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, // SP
        0x09, 0x00, 0x00, 0x00, // reset -> 0x08 thumb
        0x03, 0x48, // 08: ldr r0, =DIR
        0x01, 0x21, // 0A: movs r1, #1
        0x01, 0x60, // 0C: str r1, [r0]
        0x03, 0x48, // 0E: ldr r0, =OUT
        0x01, 0x21, // 10: movs r1, #1
        0x01, 0x60, // 12: str r1, [r0]
        0xFE, 0xE7, // 14: b .
        0x00, 0x00, // 16: pad (pool must stay word-aligned)
    ];
    for word in [0x4000_0000u32, 0x4000_0004] {
        fw.extend_from_slice(&word.to_le_bytes());
    }
    fw
}

fn create() -> *mut vioarm_cosim::VioArmHandle {
    let board = CString::new("virtual").unwrap();
    let fw = latch_firmware();
    let handle = vioarm_cosim_create(board.as_ptr(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, 0x0);
    assert!(!handle.is_null(), "create failed: {}", last_error());
    handle
}

fn pin_level(handle: *mut vioarm_cosim::VioArmHandle) -> (u8, u8, u32) {
    let (mut level, mut driving, mut mode) = (0u8, 0u8, 0u32);
    let code = vioarm_cosim_pin_get(handle, 0, 0, &mut level, &mut driving, &mut mode);
    assert_eq!(code, 0, "pin_get failed: {}", last_error());
    (level, driving, mode)
}

#[test]
fn operating_voltage_rejects_garbage() {
    let _serial = serial();
    let handle = create();
    assert_eq!(
        vioarm_cosim_set_operating_voltage(handle, f64::INFINITY),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_set_operating_voltage(std::ptr::null_mut(), 3.3),
        vioarm_cosim::VIOARM_ERR_NULL_HANDLE
    );
    // Negative saturates to 0 mV (accepted, asserts brown-out).
    assert_eq!(vioarm_cosim_set_operating_voltage(handle, -1.0), 0);
    vioarm_cosim_destroy(handle);
}

#[test]
fn brownout_reboots_and_recovers() {
    let _serial = serial();
    let handle = create();
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    assert_eq!(pin_level(handle), (1, 1, VIOARM_MODE_OUTPUT_PUSH_PULL));
    // Rail dip under BOR: the next step reboots. One core cycle runs
    // reset + a single setup instruction — GPIO is back to reset-state
    // inputs, which is itself the reboot proof (it was an output).
    assert_eq!(vioarm_cosim_set_operating_voltage(handle, 1.0), 0);
    assert_eq!(vioarm_cosim_step_duration(handle, 125e-9), VIOARM_STEP_OK);
    assert_eq!(pin_level(handle), (0, 0, VIOARM_MODE_INPUT_FLOATING));
    // Still held while the rail stays low.
    assert_eq!(vioarm_cosim_step_duration(handle, 125e-9), VIOARM_STEP_OK);
    assert_eq!(pin_level(handle), (0, 0, VIOARM_MODE_INPUT_FLOATING));
    // Rail back: firmware re-runs from the vector table, latch set again.
    assert_eq!(vioarm_cosim_set_operating_voltage(handle, 3.3), 0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.001), VIOARM_STEP_OK);
    assert_eq!(pin_level(handle), (1, 1, VIOARM_MODE_OUTPUT_PUSH_PULL));
    // cpu_hz is unaffected by rail state (clocks don't need volts here).
    assert_eq!(vioarm_cosim_cpu_hz(handle), 8_000_000);
    vioarm_cosim_destroy(handle);
}
