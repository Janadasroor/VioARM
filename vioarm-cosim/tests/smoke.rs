//! Slice-1 integration tests: handle lifecycle, error paths, and a live
//! `step_duration` loopback on the `virtual` board (hand-assembled M0
//! branch-to-self, no toolchain needed).

use std::ffi::{CStr, CString};
use std::ptr;
use vioarm_cosim::{
    vioarm_cosim_cpu_hz, vioarm_cosim_create, vioarm_cosim_destroy, vioarm_cosim_get_active_handle,
    vioarm_cosim_last_error, vioarm_cosim_step_duration, VIOARM_ERR_BAD_ARG,
    VIOARM_ERR_NULL_HANDLE, VIOARM_FW_BIN, VIOARM_STEP_OK,
};

fn last_error() -> String {
    // SAFETY: the contract guarantees a valid NUL-terminated pointer until
    // the next failing call on this thread; we copy immediately.
    unsafe { CStr::from_ptr(vioarm_cosim_last_error()) }
        .to_string_lossy()
        .into_owned()
}

// The process-global active handle is shared by every test in this binary;
// serialize so lifecycle assertions stay deterministic (each test is ms).
static SERIAL: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn serial() -> std::sync::MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|poison| poison.into_inner())
}

/// Minimal M0 firmware at base `0x0`: SP + reset vector, then `b .`
/// (halfword `0xe7fe`, bytes `fe e7` little-endian) at the reset target.
fn spin_firmware() -> [u8; 10] {
    [0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0xfe, 0xe7]
}

fn create_virtual() -> *mut vioarm_cosim::VioArmHandle {
    let board = CString::new("virtual").unwrap();
    let fw = spin_firmware();
    let handle = vioarm_cosim_create(board.as_ptr(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, 0x0);
    assert!(!handle.is_null(), "create failed: {}", last_error());
    handle
}

#[test]
fn unknown_board_fails_closed() {
    let _serial = serial();
    let board = CString::new("stm32nope").unwrap();
    let fw = spin_firmware();
    let handle = vioarm_cosim_create(board.as_ptr(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, 0x0);
    assert!(handle.is_null());
    assert!(last_error().contains("unknown board"));
}

#[test]
fn null_and_empty_inputs_fail_closed() {
    let _serial = serial();
    let board = CString::new("virtual").unwrap();
    let fw = spin_firmware();
    assert!(vioarm_cosim_create(ptr::null(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, 0x0).is_null());
    assert!(
        vioarm_cosim_create(board.as_ptr(), ptr::null(), fw.len(), VIOARM_FW_BIN, 0x0).is_null()
    );
    assert!(vioarm_cosim_create(board.as_ptr(), fw.as_ptr(), 0, VIOARM_FW_BIN, 0x0).is_null());
    assert_eq!(
        vioarm_cosim_create(board.as_ptr(), fw.as_ptr(), fw.len(), 99, 0x0),
        ptr::null_mut()
    );
    assert!(last_error().contains("unknown firmware format"));
}

#[test]
fn garbage_firmware_halts_on_first_step() {
    let _serial = serial();
    // `0xFF` words decode to a faulting instruction, but reset itself only
    // loads SP/PC — so create succeeds and the halt surfaces on step.
    let board = CString::new("virtual").unwrap();
    let fw = [0xFFu8; 16];
    let handle = vioarm_cosim_create(board.as_ptr(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, 0x0);
    assert!(!handle.is_null(), "create failed: {}", last_error());
    assert_eq!(
        vioarm_cosim_step_duration(handle, 0.001),
        vioarm_cosim::VIOARM_STEP_HALTED
    );
    assert!(last_error().contains("halted"));
    vioarm_cosim_destroy(handle);
}

#[test]
fn lifecycle_and_active_handle() {
    let _serial = serial();
    assert_eq!(vioarm_cosim_get_active_handle(), ptr::null_mut());
    let handle = create_virtual();
    assert_eq!(vioarm_cosim_get_active_handle(), handle);
    vioarm_cosim_destroy(ptr::null_mut());
    vioarm_cosim_destroy(handle);
    assert_eq!(vioarm_cosim_get_active_handle(), ptr::null_mut());
}

#[test]
fn cpu_hz_matches_virtual_board() {
    let _serial = serial();
    let handle = create_virtual();
    let hz = vioarm_cosim_cpu_hz(handle);
    assert_eq!(hz, 8_000_000, "virtual board runs HSI 8 MHz");
    assert_eq!(vioarm_cosim_cpu_hz(ptr::null_mut()), 0);
    vioarm_cosim_destroy(handle);
}

#[test]
fn step_duration_runs_without_halting() {
    let _serial = serial();
    let handle = create_virtual();
    // 1 ms at 8 MHz = 8000 branch-to-self iterations: no halt expected.
    assert_eq!(vioarm_cosim_step_duration(handle, 0.001), VIOARM_STEP_OK);
    assert_eq!(
        vioarm_cosim_step_duration(ptr::null_mut(), 0.001),
        VIOARM_ERR_NULL_HANDLE
    );
    assert_eq!(vioarm_cosim_step_duration(handle, -1.0), VIOARM_ERR_BAD_ARG);
    assert_eq!(
        vioarm_cosim_step_duration(handle, f64::NAN),
        VIOARM_ERR_BAD_ARG
    );
    vioarm_cosim_destroy(handle);
}
