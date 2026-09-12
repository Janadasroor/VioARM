//! C ABI embedding surface for VioARM: the host side of co-simulation.
//!
//! The XSPICE shim (`vioarm_cosim` shared library, built from C++ against
//! this crate's `cdylib`, mirroring VioAVR's `avr_cosim_shim.cpp`) drives
//! simulations through the `extern "C"` functions below: create a machine
//! for a named board, advance it by SPICE seconds, and query clocks. Pin
//! and analog bridges (P1-3/P1-4) build on the same handle.
//!
//! # Why this crate exists
//!
//! `workspace.lints.rust.unsafe_code = "forbid"` applies to all of
//! `vioarm-core`, and `forbid` — unlike `deny` — cannot be weakened by a
//! targeted `#[allow]` anywhere in the crate. A C ABI boundary is
//! inherently unsafe (dereferencing caller-owned raw pointers, `Box` round
//! trips, `CStr`/slice construction), so it cannot live in `vioarm-core`.
//! This micro-crate encapsulates exactly that, following the
//! `vioarm-jit-call` precedent: every `unsafe` block is marked below and
//! justified in the SAFETY CASE. All simulation logic stays in
//! `vioarm-core` behind safe signatures.
//!
//! # SAFETY CASE (cosim-roadmap P1)
//!
//! Soundness of each `unsafe` block, in order of appearance:
//!
//! 1. `CStr::from_ptr` (create): the caller lends a NUL-terminated name.
//!    The block copies it into an owned `String` immediately and never
//!    retains the pointer; a null pointer is rejected before the block.
//!    A lying caller (unterminated) can only cause an over-read, never
//!    write-side UB — and `to_str` validates UTF-8 before use.
//! 2. `slice::from_raw_parts` (create): same lending discipline — null or
//!    zero-length firmware is rejected first, the bytes are consumed
//!    synchronously by the loader, nothing is retained.
//! 3. `Box::from_raw` (destroy): every live handle originates from
//!    `Box::into_raw` in create (a safe function); destroy consumes it
//!    exactly once. Reusing a consumed pointer would be a caller bug this
//!    API cannot cause by itself, and every entry null-checks.
//! 4. `&mut *handle` (entry guard): the pointer is non-null (checked) and
//!    designates a live `Box<VioArmHandle>` by leg 3. Aliasing across
//!    threads is the caller's responsibility, documented on each entry:
//!    ngspice's `d_cosim` steps a single instance on one thread.
//! 5. Out-pointer writes (`pin_get`, `get_dac_voltage`): each pointer is
//!    null-checked first, written exactly once, and never retained.
//!
//! Panics never cross the boundary: every entry wraps its body in
//! `catch_unwind` and fails closed (null / negative status, message in
//! `vioarm_cosim_last_error`). Interior NULs in messages are dropped, not
//! passed through.
//!
//! # Handle discipline (for the shim author)
//!
//! - Handles are single-threaded: one thread per handle, no sharing.
//! - `vioarm_cosim_destroy` on null is a no-op; on a live handle it runs
//!   once. The process-global active handle (for bridge `dlsym`) tracks
//!   the most recently created live handle and clears on its destroy.

#![deny(missing_docs)]

use std::cell::RefCell;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_double};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;
use std::sync::atomic::{AtomicPtr, Ordering};

use vioarm_core::error::HaltCause;
use vioarm_core::loader;
use vioarm_core::machine::Machine;

pub mod handle;
pub use handle::VioArmHandle;

/// `step_duration` consumed its whole budget without halting.
pub const VIOARM_STEP_OK: i32 = 0;
/// `step_duration` stopped early: the core halted (reset, fault, WFI, ...).
pub const VIOARM_STEP_HALTED: i32 = 1;
/// Any entry: null handle.
pub const VIOARM_ERR_NULL_HANDLE: i32 = -1;
/// Any entry: an internal panic was caught (fails closed; see last error).
pub const VIOARM_ERR_PANIC: i32 = -2;
/// Any entry: bad argument (null pointer, unknown board/format, bad time).
pub const VIOARM_ERR_BAD_ARG: i32 = -3;

/// Firmware image formats for [`vioarm_cosim_create`]: ELF (`fw_format` 0).
pub const VIOARM_FW_ELF: u32 = 0;
/// Firmware image formats for [`vioarm_cosim_create`]: Intel HEX (`fw_format` 1).
pub const VIOARM_FW_HEX: u32 = 1;
/// Firmware image formats for [`vioarm_cosim_create`]: raw binary at `bin_base` (`fw_format` 2).
pub const VIOARM_FW_BIN: u32 = 2;

/// Pin modes for [`vioarm_cosim_pin_get`] (`mode_out`): digital input, no pull.
pub const VIOARM_MODE_INPUT_FLOATING: u32 = 0;
/// Pin modes for [`vioarm_cosim_pin_get`] (`mode_out`): digital input with pull-up.
pub const VIOARM_MODE_INPUT_PULL_UP: u32 = 1;
/// Pin modes for [`vioarm_cosim_pin_get`] (`mode_out`): digital input with pull-down.
pub const VIOARM_MODE_INPUT_PULL_DOWN: u32 = 2;
/// Pin modes for [`vioarm_cosim_pin_get`] (`mode_out`): analog, never digitally driven.
pub const VIOARM_MODE_ANALOG: u32 = 3;
/// Pin modes for [`vioarm_cosim_pin_get`] (`mode_out`): push-pull output.
pub const VIOARM_MODE_OUTPUT_PUSH_PULL: u32 = 4;
/// Pin modes for [`vioarm_cosim_pin_get`] (`mode_out`): open-drain output.
pub const VIOARM_MODE_OUTPUT_OPEN_DRAIN: u32 = 5;
/// Pin modes for [`vioarm_cosim_pin_get`] (`mode_out`): alternate-function push-pull.
pub const VIOARM_MODE_ALTERNATE_PUSH_PULL: u32 = 6;
/// Pin modes for [`vioarm_cosim_pin_get`] (`mode_out`): alternate-function open-drain.
pub const VIOARM_MODE_ALTERNATE_OPEN_DRAIN: u32 = 7;

/// Process-global active handle for bridge `dlsym` (mirrors
/// `vioavr_cosim_get_active_handle`): the most recently created live
/// handle, cleared when it is destroyed. Plain atomic operations only —
/// no `unsafe` needed.
static ACTIVE_HANDLE: AtomicPtr<VioArmHandle> = AtomicPtr::new(ptr::null_mut());

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

fn set_error(message: String) {
    if let Ok(detail) = CString::new(message) {
        LAST_ERROR.with(|slot| *slot.borrow_mut() = Some(detail));
    }
}

/// Last error message for this thread (empty string if none). The pointer
/// stays valid until the next failing call on the same thread; copy it
/// before calling back in.
#[must_use]
#[no_mangle]
pub extern "C" fn vioarm_cosim_last_error() -> *const c_char {
    static EMPTY: c_char = 0;
    LAST_ERROR.with(|slot| {
        slot.borrow()
            .as_ref()
            .map_or(&EMPTY as *const c_char, |detail| detail.as_ptr())
    })
}

/// Look up a live handle or fail closed with [`VIOARM_ERR_NULL_HANDLE`].
fn with_handle<T>(
    raw: *mut VioArmHandle,
    body: impl FnOnce(&mut VioArmHandle) -> T,
) -> Result<T, i32> {
    if raw.is_null() {
        set_error("null handle".to_owned());
        return Err(VIOARM_ERR_NULL_HANDLE);
    }
    // SAFETY CASE leg 4: non-null (checked) + live (leg 3) + documented
    // single-thread discipline.
    Ok(body(unsafe { &mut *raw }))
}

/// Run `body` without letting panics cross the FFI boundary.
fn guard(body: impl FnOnce() -> i32) -> i32 {
    match catch_unwind(AssertUnwindSafe(body)) {
        Ok(code) => code,
        Err(_) => {
            set_error("internal error (panic caught at the FFI boundary)".to_owned());
            VIOARM_ERR_PANIC
        }
    }
}

/// Create a machine for `board` (an `all_boards` short name such as
/// `"stm32f103"` or `"virtual"`), load `fw` (`fw_len` bytes in `fw_format`:
/// [`VIOARM_FW_ELF`] / [`VIOARM_FW_HEX`] / [`VIOARM_FW_BIN`], the latter at
/// `bin_base`), reset it, and return its handle — null on error, with the
/// reason in [`vioarm_cosim_last_error`]. The new handle becomes the
/// process-global active handle.
///
/// Loading follows the host convention shared with `vioarm-qemu` and the
/// golden tests: firmware goes to the board's boot alias (`0x0` on STM32),
/// because stores to the gated `0x0800_0000` main-flash window are dropped
/// while the FLASH controller is locked. The shim must rebase STM32 images
/// to the alias before calling (raw binaries via `bin_base`; ELF/HEX
/// segments must already target it).
#[no_mangle]
pub extern "C" fn vioarm_cosim_create(
    board: *const c_char,
    fw: *const u8,
    fw_len: usize,
    fw_format: u32,
    bin_base: u32,
) -> *mut VioArmHandle {
    match catch_unwind(AssertUnwindSafe(|| {
        create_inner(board, fw, fw_len, fw_format, bin_base)
    })) {
        Ok(Ok(raw)) => raw,
        Ok(Err(_)) => ptr::null_mut(),
        Err(_) => {
            set_error("internal error (panic caught at the FFI boundary)".to_owned());
            ptr::null_mut()
        }
    }
}

fn create_inner(
    board: *const c_char,
    fw: *const u8,
    fw_len: usize,
    fw_format: u32,
    bin_base: u32,
) -> Result<*mut VioArmHandle, i32> {
    if board.is_null() || fw.is_null() || fw_len == 0 {
        set_error("null board name or empty firmware".to_owned());
        return Err(VIOARM_ERR_BAD_ARG);
    }
    // SAFETY CASE leg 1: null rejected above; copied out immediately.
    let name = unsafe { CStr::from_ptr(board) }.to_str().map_err(|_| {
        set_error("board name is not valid UTF-8".to_owned());
        VIOARM_ERR_BAD_ARG
    })?;
    // SAFETY CASE leg 2: null/empty rejected above; consumed synchronously.
    let image = unsafe { std::slice::from_raw_parts(fw, fw_len) };
    let loaded = match fw_format {
        VIOARM_FW_ELF => loader::load_elf(image),
        VIOARM_FW_HEX => loader::load_hex(image),
        VIOARM_FW_BIN => Ok(loader::load_flat(bin_base, image)),
        other => {
            set_error(format!("unknown firmware format {other}"));
            return Err(VIOARM_ERR_BAD_ARG);
        }
    }
    .map_err(|cause| {
        set_error(format!("firmware rejected: {cause}"));
        VIOARM_ERR_BAD_ARG
    })?;
    let machine = if fw_format == VIOARM_FW_BIN {
        // Raw binaries carry no vector-table address: build unreset, boot
        // from the load base, then reset (ELF/HEX keep the default table,
        // matching the CLI exactly).
        let mut bare = Machine::for_name_bare(name, &loaded).map_err(|cause| {
            set_error(cause.to_string());
            VIOARM_ERR_BAD_ARG
        })?;
        bare.set_vector_base(bin_base);
        bare.reset().map_err(|cause| {
            set_error(format!("reset failed: {cause}"));
            VIOARM_ERR_BAD_ARG
        })?;
        bare
    } else {
        Machine::for_name(name, &loaded).map_err(|cause| {
            set_error(cause.to_string());
            VIOARM_ERR_BAD_ARG
        })?
    };
    let raw = Box::into_raw(Box::new(VioArmHandle::new(machine)));
    ACTIVE_HANDLE.store(raw, Ordering::SeqCst);
    Ok(raw)
}

/// Destroy `handle` (no-op on null). Clears the process-global active
/// handle when it designates this handle.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately: the
/// lint wants `unsafe fn`, but unsafety is unenforceable across a C ABI
/// (the caller is C++) and the null check makes this entry total — the
/// same reasoning as `vioarm-jit-call`'s `call0`/`call1`.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_destroy(handle: *mut VioArmHandle) {
    if handle.is_null() {
        return;
    }
    let _ = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY CASE leg 3: sole owner round-trips its own `Box::into_raw`.
        let _boxed = unsafe { Box::from_raw(handle) };
        let _ = ACTIVE_HANDLE.compare_exchange(
            handle,
            ptr::null_mut(),
            Ordering::SeqCst,
            Ordering::SeqCst,
        );
    }));
}

/// The process-global active handle (null when none is live): what the
/// analog bridges resolve with `dlsym`.
#[must_use]
#[no_mangle]
pub extern "C" fn vioarm_cosim_get_active_handle() -> *mut VioArmHandle {
    ACTIVE_HANDLE.load(Ordering::SeqCst)
}

/// Advance `handle` by `seconds` of SPICE virtual time, converted to
/// true core cycles at the live CPU clock (`run_cycles`: fetch wait
/// states included, so SPICE time and core time agree absolutely —
/// driving by retired-instruction count instead would run ahead wherever
/// flash wait states apply). Returns [`VIOARM_STEP_OK`] when the whole
/// budget retired, [`VIOARM_STEP_HALTED`] on an early halt (detail in
/// [`vioarm_cosim_last_error`]), or a negative error code.
#[no_mangle]
pub extern "C" fn vioarm_cosim_step_duration(handle: *mut VioArmHandle, seconds: c_double) -> i32 {
    guard(|| {
        with_handle(handle, |harness| {
            let machine = harness.machine_mut();
            if !seconds.is_finite() || seconds < 0.0 {
                set_error(format!("bad step duration {seconds}"));
                return VIOARM_ERR_BAD_ARG;
            }
            let hz = machine.cpu_hz();
            #[allow(clippy::cast_possible_truncation)]
            let cycles = (seconds * f64::from(hz)) as u64;
            match machine.run_cycles(cycles) {
                Err(HaltCause::CycleBudgetExhausted) => VIOARM_STEP_OK,
                Err(cause) => {
                    set_error(format!("halted: {cause}"));
                    VIOARM_STEP_HALTED
                }
                Ok(()) => VIOARM_STEP_OK,
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// The live CPU clock in Hz (`0` on a null handle).
#[no_mangle]
pub extern "C" fn vioarm_cosim_cpu_hz(handle: *mut VioArmHandle) -> u32 {
    match catch_unwind(AssertUnwindSafe(|| {
        with_handle(handle, |harness| harness.machine_mut().cpu_hz())
    })) {
        Ok(Ok(hz)) => hz,
        Ok(Err(_)) | Err(_) => 0,
    }
}

/// Read GPIO `pin` on port `port` (0 = A): `level_out` gets 0/1,
/// `driving_out` gets 1 when firmware drives the pin (output/alternate —
/// the shim models these `{level, STRONG}`, inputs `{level, HI-Z}`),
/// `mode_out` gets a `VIOARM_MODE_*` code. Returns 0 on success or a
/// negative error code. All three out-pointers must be non-null.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same as
/// [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI and
/// the null checks make this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_pin_get(
    handle: *mut VioArmHandle,
    port: u32,
    pin: u32,
    level_out: *mut u8,
    driving_out: *mut u8,
    mode_out: *mut u32,
) -> i32 {
    guard(|| {
        if level_out.is_null() || driving_out.is_null() || mode_out.is_null() {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        let (port, pin) = match checked_pin(port, pin) {
            Ok(pair) => pair,
            Err(code) => return code,
        };
        with_handle(handle, |harness| {
            let machine = harness.machine_mut();
            match machine.gpio_pin(port, pin) {
                Some((mode, level)) => {
                    // SAFETY CASE leg 5: out-pointers validated non-null
                    // above, written exactly once, never retained.
                    unsafe {
                        level_out.write(u8::from(level));
                        driving_out.write(u8::from(mode.is_driven()));
                        mode_out.write(mode as u32);
                    }
                    0
                }
                None => {
                    set_error(format!("no gpio port {port} pin {pin}"));
                    VIOARM_ERR_BAD_ARG
                }
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Drive GPIO `pin` on port `port` (0 = A) to `level` (0/1) from the host
/// side (input pins follow it; driven pins ignore it, like a net the MCU
/// already owns). Returns 0 on success or a negative error code.
#[no_mangle]
pub extern "C" fn vioarm_cosim_pin_set(
    handle: *mut VioArmHandle,
    port: u32,
    pin: u32,
    level: u8,
) -> i32 {
    guard(|| {
        if level > 1 {
            set_error(format!("bad pin level {level}"));
            return VIOARM_ERR_BAD_ARG;
        }
        let (port, pin) = match checked_pin(port, pin) {
            Ok(pair) => pair,
            Err(code) => return code,
        };
        with_handle(handle, |harness| {
            let index = usize::from(port);
            let mut mask = harness.driven(index);
            if level == 1 {
                mask |= 1 << pin;
            } else {
                mask &= !(1 << pin);
            }
            if harness.machine_mut().gpio_drive(port, mask) {
                harness.set_driven(index, mask);
                0
            } else {
                set_error(format!("no gpio port {port}"));
                VIOARM_ERR_BAD_ARG
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Inject an analog voltage (volts) into ADC `channel`, as the ADC bridge
/// resolves it each SPICE step. Only channel 0 exists (the single N3 `apin`
/// window); anything else is [`VIOARM_ERR_BAD_ARG`]. The window's `EN` bit
/// must be set by firmware first — like enabling an ADC. Negative/NaN
/// inputs saturate to 0 mV, huge ones to `u32::MAX` mV.
#[no_mangle]
pub extern "C" fn vioarm_cosim_set_external_voltage(
    handle: *mut VioArmHandle,
    channel: i32,
    volts: c_double,
) -> i32 {
    guard(|| {
        if channel != 0 {
            set_error(format!("no analog channel {channel} (only 0 exists)"));
            return VIOARM_ERR_BAD_ARG;
        }
        if volts.is_infinite() {
            set_error(format!("bad voltage {volts}"));
            return VIOARM_ERR_BAD_ARG;
        }
        // Saturation is the intent (`as` saturates float-to-int by
        // definition): negatives and NaN land on 0 mV.
        #[allow(clippy::cast_possible_truncation)]
        let mv = (volts * 1000.0) as u32;
        with_handle(handle, |harness| {
            if harness.machine_mut().apin_inject_mv(mv) {
                0
            } else {
                set_error("no apin device attached".to_owned());
                VIOARM_ERR_BAD_ARG
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Read the DAC `channel` output (volts): the most recently transmitted
/// `apin` word, if firmware staged one (`TXMV` + `TXGO`). Only channel 0
/// exists. `volts_out` must be non-null. With nothing staged, the output
/// is left untouched and the return is still 0 — the shim keeps its last
/// value, like a held DAC register.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same as
/// [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI and
/// the null check makes this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_get_dac_voltage(
    handle: *mut VioArmHandle,
    channel: i32,
    volts_out: *mut c_double,
) -> i32 {
    guard(|| {
        if volts_out.is_null() {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        if channel != 0 {
            set_error(format!("no analog channel {channel} (only 0 exists)"));
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            if let Some(mv) = harness.machine_mut().apin_drain_mv() {
                // `u32` fits `f64` exactly; no rounding possible.
                let volts = f64::from(mv) / 1000.0;
                // SAFETY CASE leg 5: validated above, written once.
                unsafe {
                    volts_out.write(volts);
                }
            }
            0
        })
        .unwrap_or_else(|code| code)
    })
}

/// Validate a `(port, pin)` pair from the ABI: ports 0..=15, pins 0..=31
/// (devices narrow further — STM32 ports carry 16 pins).
fn checked_pin(port: u32, pin: u32) -> Result<(u8, u8), i32> {
    let port = u8::try_from(port).map_err(|_| {
        set_error(format!("bad gpio port {port}"));
        VIOARM_ERR_BAD_ARG
    })?;
    if port >= 16 {
        set_error(format!("bad gpio port {port}"));
        return Err(VIOARM_ERR_BAD_ARG);
    }
    let pin = u8::try_from(pin).map_err(|_| {
        set_error(format!("bad gpio pin {pin}"));
        VIOARM_ERR_BAD_ARG
    })?;
    if pin >= 32 {
        set_error(format!("bad gpio pin {pin}"));
        return Err(VIOARM_ERR_BAD_ARG);
    }
    Ok((port, pin))
}
