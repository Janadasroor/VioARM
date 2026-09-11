//! Slice-2 pin tests: closed-loop digital through the C ABI on the
//! `virtual` board (generic DIR/OUT/IN GPIO at `0x4000_0000`), plus the
//! STM32F1 nibble decode on `stm32f103`.
//!
//! All firmware is hand-assembled Thumb (no toolchain needed); encodings
//! were verified against the CLI (`run --budget`) during development.

use std::ffi::{CStr, CString};
use std::ptr;
use vioarm_cosim::{
    vioarm_cosim_create, vioarm_cosim_destroy, vioarm_cosim_pin_get, vioarm_cosim_pin_set,
    vioarm_cosim_step_duration, VIOARM_FW_BIN, VIOARM_MODE_INPUT_FLOATING,
    VIOARM_MODE_OUTPUT_PUSH_PULL,
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

fn pin_get(handle: *mut vioarm_cosim::VioArmHandle, port: u32, pin: u32) -> (u8, u8, u32) {
    let (mut level, mut driving, mut mode) = (0u8, 0u8, 0u32);
    let code = vioarm_cosim_pin_get(handle, port, pin, &mut level, &mut driving, &mut mode);
    assert_eq!(code, 0, "pin_get failed: {}", last_error());
    (level, driving, mode)
}

/// Firmware: DIR pin0+pin2 output, OUT pin0 high, then loop `OUT[2] =
/// IN[1]` forever. Literals: DIR/OUT/IN/OUT word addresses.
fn echo_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, // SP
        0x09, 0x00, 0x00, 0x00, // reset -> 0x08 thumb
        0x0A, 0x48, // 08: ldr r0, =DIR
        0x05, 0x21, // 0A: movs r1, #5
        0x01, 0x60, // 0C: str r1, [r0]
        0x0A, 0x48, // 0E: ldr r0, =OUT
        0x01, 0x21, // 10: movs r1, #1
        0x01, 0x60, // 12: str r1, [r0]
        0x09, 0x48, // 14: ldr r0, =IN
        0x0A, 0x49, // 16: ldr r1, =OUT
        0x02, 0x68, // 18: loop: ldr r2, [r0]
        0x02, 0x23, // 1A: movs r3, #2
        0x1A, 0x42, // 1C: tst r2, r3
        0x04, 0xD0, // 1E: beq clear
        0x0A, 0x68, // 20: ldr r2, [r1]
        0x04, 0x23, // 22: movs r3, #4
        0x1A, 0x43, // 24: orrs r2, r3
        0x0A, 0x60, // 26: str r2, [r1]
        0xF6, 0xE7, // 28: b loop
        0x0A, 0x68, // 2A: clear: ldr r2, [r1]
        0x04, 0x23, // 2C: movs r3, #4
        0x9A, 0x43, // 2E: bics r2, r3
        0x0A, 0x60, // 30: str r2, [r1]
        0xF1, 0xE7, // 32: b loop (offset -15: (0x18-0x36)/2)
    ];
    for word in [0x4000_0000u32, 0x4000_0004, 0x4000_0008, 0x4000_0004] {
        fw.extend_from_slice(&word.to_le_bytes());
    }
    fw
}

#[test]
fn echo_closed_loop() {
    let _serial = serial();
    let fw = echo_firmware();
    let handle = create("virtual", &fw, 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), 0);
    // Pin 0: firmware output, high.
    assert_eq!(pin_get(handle, 0, 0), (1, 1, VIOARM_MODE_OUTPUT_PUSH_PULL));
    // Pin 1: floating input, host-low.
    assert_eq!(pin_get(handle, 0, 1), (0, 0, VIOARM_MODE_INPUT_FLOATING));
    // Drive pin 1 high: firmware copies it to pin 2.
    assert_eq!(vioarm_cosim_pin_set(handle, 0, 1, 1), 0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.001), 0);
    assert_eq!(pin_get(handle, 0, 1), (1, 0, VIOARM_MODE_INPUT_FLOATING));
    assert_eq!(pin_get(handle, 0, 2), (1, 1, VIOARM_MODE_OUTPUT_PUSH_PULL));
    // Release: pin 2 follows back low.
    assert_eq!(vioarm_cosim_pin_set(handle, 0, 1, 0), 0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.001), 0);
    assert_eq!(pin_get(handle, 0, 2), (0, 1, VIOARM_MODE_OUTPUT_PUSH_PULL));
    vioarm_cosim_destroy(handle);
}

#[test]
fn pin_errors_fail_closed() {
    let _serial = serial();
    let fw = echo_firmware();
    let handle = create("virtual", &fw, 0x0);
    let (mut level, mut driving, mut mode) = (0u8, 0u8, 0u32);
    // Null out-pointer.
    assert_ne!(
        vioarm_cosim_pin_get(handle, 0, 0, ptr::null_mut(), &mut driving, &mut mode),
        0
    );
    // Missing port on the single-port virtual board.
    assert_ne!(
        vioarm_cosim_pin_get(handle, 1, 0, &mut level, &mut driving, &mut mode),
        0
    );
    // Pin out of range.
    assert_ne!(
        vioarm_cosim_pin_get(handle, 0, 32, &mut level, &mut driving, &mut mode),
        0
    );
    assert_ne!(vioarm_cosim_pin_set(handle, 1, 0, 1), 0);
    assert_ne!(vioarm_cosim_pin_set(handle, 0, 0, 2), 0);
    assert_ne!(vioarm_cosim_pin_set(ptr::null_mut(), 0, 0, 1), 0);
    vioarm_cosim_destroy(handle);
}

/// Firmware for `stm32f103`: enable `IOPAEN`, set CRL pin0 nibble to
/// `0x3` (output push-pull), ODR pin0 high, then park. RCC APB2ENR
/// `0x4002_1018`, GPIOA CRL `0x4001_0800`, ODR `0x4001_080C`.
fn f1_output_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, // SP
        0x09, 0x00, 0x00, 0x00, // reset -> 0x08 thumb
        0x07, 0x48, // 08: ldr r0, =RCC_EN
        0x01, 0x68, // 0A: ldr r1, [r0]
        0x04, 0x22, // 0C: movs r2, #4 (IOPAEN)
        0x11, 0x43, // 0E: orrs r1, r2
        0x01, 0x60, // 10: str r1, [r0]
        0x06, 0x48, // 12: ldr r0, =CRL
        0x01, 0x68, // 14: ldr r1, [r0]
        0x0F, 0x22, // 16: movs r2, #0xF
        0x91, 0x43, // 18: bics r1, r2
        0x03, 0x22, // 1A: movs r2, #3
        0x11, 0x43, // 1C: orrs r1, r2
        0x01, 0x60, // 1E: str r1, [r0]
        0x03, 0x48, // 20: ldr r0, =ODR
        0x01, 0x21, // 22: movs r1, #1
        0x01, 0x60, // 24: str r1, [r0]
        0xFE, 0xE7, // 26: b .
    ];
    for word in [0x4002_1018u32, 0x4001_0800, 0x4001_080C] {
        fw.extend_from_slice(&word.to_le_bytes());
    }
    fw
}

#[test]
fn f1_nibble_decode_reports_output() {
    let _serial = serial();
    let fw = f1_output_firmware();
    // Boot-alias convention (shared with `vioarm-qemu` and the golden
    // tests): STM32 firmware loads at the `0x0` flash alias, because
    // stores to the gated `0x0800_0000` main-flash window are dropped
    // while the FLASH controller is locked.
    let handle = create("stm32f103", &fw, 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), 0);
    // Pin 0: CRL nibble `0x3` (MODE=50MHz, CNF=PP) + ODR high.
    assert_eq!(pin_get(handle, 0, 0), (1, 1, VIOARM_MODE_OUTPUT_PUSH_PULL));
    // Pin 1: reset nibble `0x4` (input floating).
    assert_eq!(pin_get(handle, 0, 1), (0, 0, VIOARM_MODE_INPUT_FLOATING));
    vioarm_cosim_destroy(handle);
}
