//! Stdin harness for the `vioarm-cosim` C ABI entry surface (P1-5).
//!
//! Input layout (all little-endian, lengths capped so one input cannot
//! hang the harness):
//! ```text
//! [board_len:u8][board bytes][format:u8][fw_len:u16le][fw bytes][ops...]
//! op = [tag:u8][payload]
//!   0x00 step_duration(f64)   — clamped to 0..10ms (huge budgets would
//!                               only burn cycles, not find bugs)
//!   0x01 pin_get(port:u8, pin:u8)
//!   0x02 pin_set(port:u8, pin:u8, level:u8)
//!   0x03 cpu_hz
//!   0x04 set_external_voltage(channel:i8, volts:f64)
//!   0x05 get_dac_voltage(channel:i8)
//!   0x06 last_error (drain the message)
//! ```
//! Handles come only from successful `create` (destroyed once at the end —
//! double-destroy is a caller contract violation, not fuzzable here), and
//! every call goes through valid pointers (raw-pointer misuse by foreign
//! callers is out of scope for an in-process harness). Any nonzero exit
//! is a finding: the entries must all fail closed, never panic.
//!
//! A few fixed nasties (NaN/inf/negative durations, wild channels) run
//! unconditionally so they are covered even by empty inputs.

use std::ffi::CString;
use vioarm_cosim::{
    vioarm_cosim_cpu_hz, vioarm_cosim_create, vioarm_cosim_destroy, vioarm_cosim_get_active_handle,
    vioarm_cosim_get_dac_voltage, vioarm_cosim_last_error, vioarm_cosim_pin_get,
    vioarm_cosim_pin_set, vioarm_cosim_set_external_voltage, vioarm_cosim_step_duration,
    VioArmHandle,
};

const BOARDS: [&str; 4] = ["virtual", "stm32f103", "stm32f407", "nope"];

fn take<'a>(input: &mut &'a [u8], n: usize) -> &'a [u8] {
    let (head, tail) = input.split_at(n.min(input.len()));
    *input = tail;
    head
}

fn take_u8(input: &mut &[u8]) -> u8 {
    take(input, 1).first().copied().unwrap_or(0)
}

fn take_f64(input: &mut &[u8]) -> f64 {
    let bytes = take(input, 8);
    let mut raw = [0u8; 8];
    raw[..bytes.len()].copy_from_slice(bytes);
    f64::from_le_bytes(raw)
}

fn exercise(handle: *mut VioArmHandle, input: &mut &[u8]) {
    // Fixed nasties first (covered on every input, even empty ones).
    for seconds in [f64::NAN, f64::INFINITY, -1.0, 0.0] {
        vioarm_cosim_step_duration(handle, seconds);
    }
    for channel in [-1, 0, 1, 127] {
        vioarm_cosim_set_external_voltage(handle, channel, 1.5);
        let mut volts = 0.0;
        vioarm_cosim_get_dac_voltage(handle, channel, &mut volts);
    }
    let mut level = 0u8;
    let mut driving = 0u8;
    let mut mode = 0u32;
    // Scripted ops, bounded so one input stays quick.
    for _ in 0..64 {
        if input.is_empty() {
            break;
        }
        match take_u8(input) {
            0x00 => {
                // Clamp: full-range durations only burn cycles.
                let raw = take_f64(input);
                let seconds = if raw.is_finite() && raw >= 0.0 {
                    (raw % 0.011).max(0.0)
                } else {
                    raw
                };
                vioarm_cosim_step_duration(handle, seconds);
            }
            0x01 => {
                let port = u32::from(take_u8(input));
                let pin = u32::from(take_u8(input));
                vioarm_cosim_pin_get(handle, port, pin, &mut level, &mut driving, &mut mode);
            }
            0x02 => {
                let port = u32::from(take_u8(input));
                let pin = u32::from(take_u8(input));
                let level = take_u8(input);
                vioarm_cosim_pin_set(handle, port, pin, level);
            }
            0x03 => {
                vioarm_cosim_cpu_hz(handle);
            }
            0x04 => {
                let channel = take_u8(input) as i8 as i32;
                vioarm_cosim_set_external_voltage(handle, channel, take_f64(input));
            }
            0x05 => {
                let channel = take_u8(input) as i8 as i32;
                let mut volts = 0.0;
                vioarm_cosim_get_dac_voltage(handle, channel, &mut volts);
            }
            _ => {
                let _ = vioarm_cosim_last_error();
            }
        }
    }
    std::hint::black_box((level, driving, mode));
}

fn main() {
    let mut buf = Vec::new();
    use std::io::Read;
    let _ = std::io::stdin().read_to_end(&mut buf);
    let mut input = buf.as_slice();

    // Board: fuzzed index with fallback to raw bytes (exercises UTF-8 and
    // unknown-board paths alike).
    let board_len = take_u8(&mut input) as usize;
    let board_bytes = take(&mut input, board_len.min(32)).to_vec();
    let board_name = String::from_utf8(board_bytes.clone())
        .unwrap_or_else(|_| BOARDS[board_len % BOARDS.len()].to_owned());
    let board = CString::new(board_name)
        .unwrap_or_else(|_| CString::new(BOARDS[board_len % BOARDS.len()]).unwrap());

    let format = take_u8(&mut input) % 3;
    let len_bytes = take(&mut input, 2);
    let mut len_raw = [0u8; 2];
    len_raw[..len_bytes.len()].copy_from_slice(len_bytes);
    let fw_len = u16::from_le_bytes(len_raw) as usize;
    let fw = take(&mut input, fw_len.min(4096)).to_vec();

    // Null/empty edge calls (fail-closed paths) on every input.
    vioarm_cosim_create(std::ptr::null(), fw.as_ptr(), fw.len(), format.into(), 0);
    vioarm_cosim_cpu_hz(std::ptr::null_mut());
    vioarm_cosim_step_duration(std::ptr::null_mut(), 0.001);
    vioarm_cosim_destroy(std::ptr::null_mut());
    assert!(vioarm_cosim_get_active_handle().is_null());

    let handle = vioarm_cosim_create(board.as_ptr(), fw.as_ptr(), fw.len(), format.into(), 0);
    if !handle.is_null() {
        exercise(handle, &mut input);
        vioarm_cosim_destroy(handle);
    }
    assert!(vioarm_cosim_get_active_handle().is_null());
    std::hint::black_box(buf);
}
