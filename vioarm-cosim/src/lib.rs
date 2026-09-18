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
//! 2. `slice::from_raw_parts` (create/load/inject): same lending
//!    discipline — null or zero-length buffers are rejected first, the
//!    bytes are consumed synchronously by the loader (firmware) or the
//!    device (injection), nothing is retained.
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
//! 6. Event-buffer writes (`run_quantum`): `out` is null-checked whenever
//!    `cap > 0`, elements are written with `ptr::addr_of_mut!`‑free plain
//!    `add(i).write` inside the buffer the caller sized, `out_len` is
//!    null-checked always and written exactly once; nothing is retained.
//! 7. Snapshot-buffer writes (`snapshot_save`): same discipline as leg 6 —
//!    `out` validated for `cap` bytes, `out_len` always written once; the
//!    core serializer refuses short buffers before touching anything.
//! 8. String-out writes (`device_name`, `board_name`, `board_pin`):
//!    `out`/`cap` validated (`cap` must hold bytes plus NUL), written
//!    in-bounds with a terminating NUL, never retained; name lookups copy
//!    out immediately (`CStr::from_ptr` board/pin inputs, same as leg 1).
//! 9. Snapshot-restore reads (`snapshot_restore`): `buf` null-checked
//!    whenever `len > 0`, consumed synchronously into owned state via
//!    `slice::from_raw_parts` (same lending discipline as leg 2); the core
//!    validator (magic/version/exact length) runs before anything is
//!    touched, so a rejected buffer leaves the machine exactly as it was.
//!
//! Panics never cross the boundary: every entry wraps its body in
//! `catch_unwind` and fails closed (null / negative status, message in
//! `vioarm_cosim_last_error`). Interior NULs in messages are dropped, not
//! passed through.
//!
//! Host event callbacks (`set_event_callback`) run synchronously inside
//! stepping calls with the machine borrow released: the event pointer
//! borrows from a caller-visible owned buffer that outlives the call,
//! and each invocation is `catch_unwind`-guarded like an entry. Calling
//! back in on the same handle trips the reentrancy guard
//! (`VIOARM_ERR_REENTRY`) instead of aliasing the machine.
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
use std::os::raw::{c_char, c_double, c_void};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;
use std::sync::atomic::{AtomicPtr, Ordering};

use vioarm_core::board;
use vioarm_core::error::HaltCause;
use vioarm_core::loader;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;
use vioarm_core::snapshot::SnapshotError;

pub mod handle;
use handle::HaltOutcome;
pub use handle::VioArmHandle;

/// `step_duration` consumed its whole budget without halting.
pub const VIOARM_STEP_OK: i32 = 0;
/// `step_duration` stopped early: the core halted (reset, fault, WFI, ...).
pub const VIOARM_STEP_HALTED: i32 = 1;
/// A stepping call stopped at a host breakpoint before the step at the
/// reported `pc` (see [`vioarm_cosim_halt_reason`]): the machine is
/// parked, nothing retired past the breakpoint.
pub const VIOARM_STEP_BREAKPOINT: i32 = 2;
/// Any entry: null handle.
pub const VIOARM_ERR_NULL_HANDLE: i32 = -1;
/// Any entry: an internal panic was caught (fails closed; see last error).
pub const VIOARM_ERR_PANIC: i32 = -2;
/// Any entry: bad argument (null pointer, unknown board/format, bad time).
pub const VIOARM_ERR_BAD_ARG: i32 = -3;
/// Data-space access hit unmapped or device-claimed-but-unreadable memory
/// ([`vioarm_cosim_read_data`] / [`vioarm_cosim_write_data`]): the access
/// was refused, nothing was touched.
pub const VIOARM_ERR_BUS: i32 = -4;
/// Snapshot frozen under an incompatible version
/// ([`vioarm_cosim_snapshot_restore`]): layout version or API major
/// mismatch — fails closed, the machine is untouched. Anything else about
/// the buffer (short, bad magic, span mismatch) stays
/// [`VIOARM_ERR_BAD_ARG`].
pub const VIOARM_ERR_VERSION: i32 = -5;
/// A host event callback called back into `vioarm_cosim_*` on the same
/// handle: callbacks are observe-only, the nested call is refused and the
/// outer call continues once the callback returns.
pub const VIOARM_ERR_REENTRY: i32 = -6;

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
/// Machine-driven quantum event kinds for [`VioArmEvent::kind`]: pin change.
pub const VIOARM_EVENT_PIN: u32 = 0;
/// Machine-driven quantum event kinds for [`VioArmEvent::kind`]: analog word.
pub const VIOARM_EVENT_ANALOG: u32 = 1;

/// Halt reasons for [`vioarm_cosim_halt_reason`]: the last stepping call
/// retired its whole budget.
pub const VIOARM_HALT_COMPLETED: u32 = 0;
/// Halt reasons: a host breakpoint fired (parked before the step at the
/// reported `pc`; see [`VIOARM_STEP_BREAKPOINT`]).
pub const VIOARM_HALT_BREAKPOINT: u32 = 1;
/// Halt reasons: the core halted early (fault, reset request, ... — detail
/// in [`vioarm_cosim_last_error`]).
pub const VIOARM_HALT_FAULT: u32 = 2;

/// Host event callback: one synchronous call per observed event, in retire
/// order, with the host context from
/// [`vioarm_cosim_set_event_callback`]. The event pointer is valid for the
/// call only. Observe-only: calling back into `vioarm_cosim_*` on the same
/// handle fails with [`VIOARM_ERR_REENTRY`]; a panicking (Rust) callback
/// aborts the stepping call with [`VIOARM_ERR_PANIC`].
pub type VioArmEventCallback = Option<extern "C" fn(event: *const VioArmEvent, ctx: *mut c_void)>;

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
/// A call that arrives while the handle's own event callback runs fails
/// with [`VIOARM_ERR_REENTRY`] instead: callbacks are observe-only.
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
    let harness = unsafe { &mut *raw };
    if harness.in_callback() {
        set_error("reentrant call from a host event callback".to_owned());
        return Err(VIOARM_ERR_REENTRY);
    }
    Ok(body(harness))
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
    let machine = load_machine(name, image, fw_format, bin_base)?;
    let raw = Box::into_raw(Box::new(VioArmHandle::new(machine, name.to_owned())));
    ACTIVE_HANDLE.store(raw, Ordering::SeqCst);
    Ok(raw)
}

/// Load `fw` (`fw_format`, `bin_base`: same convention as
/// [`vioarm_cosim_create`]) into a reset machine: the shared boot path
/// behind both `create` and [`vioarm_cosim_load`], so the CLI and the
/// co-simulation path cannot diverge.
fn load_machine(
    name: &str,
    fw: &[u8],
    fw_format: u32,
    bin_base: u32,
) -> Result<Machine<board::BoardMemory>, i32> {
    let loaded = match fw_format {
        VIOARM_FW_ELF => loader::load_elf(fw),
        VIOARM_FW_HEX => loader::load_hex(fw),
        VIOARM_FW_BIN => Ok(loader::load_flat(bin_base, fw)),
        other => {
            set_error(format!("unknown firmware format {other}"));
            return Err(VIOARM_ERR_BAD_ARG);
        }
    }
    .map_err(|cause| {
        set_error(format!("firmware rejected: {cause}"));
        VIOARM_ERR_BAD_ARG
    })?;
    if fw_format == VIOARM_FW_BIN {
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
        Ok(bare)
    } else {
        Machine::for_name(name, &loaded).map_err(|cause| {
            set_error(cause.to_string());
            VIOARM_ERR_BAD_ARG
        })
    }
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
/// [`vioarm_cosim_last_error`]), [`VIOARM_STEP_BREAKPOINT`] when a host
/// breakpoint fired (parked before its step; see
/// [`vioarm_cosim_halt_reason`]), or a negative error code.
///
/// With no breakpoints armed and no event callback set this is one
/// single-call quantum; otherwise stepping retires through an evented
/// loop (same primitives, same order — an armed-but-never-hit run is
/// bit-identical to the single-call path) and host callbacks fire per
/// observed event before the return.
#[no_mangle]
pub extern "C" fn vioarm_cosim_step_duration(handle: *mut VioArmHandle, seconds: c_double) -> i32 {
    guard(|| {
        if !seconds.is_finite() || seconds < 0.0 {
            set_error(format!("bad step duration {seconds}"));
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            if !harness.breakpoints().is_empty() || harness.callback().0.is_some() {
                return step_duration_evented(harness, seconds);
            }
            let (code, outcome) = {
                let machine = harness.machine_mut();
                let hz = machine.cpu_hz();
                #[allow(clippy::cast_possible_truncation)]
                let cycles = (seconds * f64::from(hz)) as u64;
                match machine.run_cycles(cycles) {
                    Err(HaltCause::CycleBudgetExhausted) => {
                        (VIOARM_STEP_OK, HaltOutcome::Completed)
                    }
                    Err(cause) => {
                        set_error(format!("halted: {cause}"));
                        (VIOARM_STEP_HALTED, HaltOutcome::Fault)
                    }
                    Ok(()) => (VIOARM_STEP_OK, HaltOutcome::Completed),
                }
            };
            harness.set_last_halt(outcome);
            code
        })
        .unwrap_or_else(|code| code)
    })
}

/// Evented `step_duration` (breakpoints armed and/or callback set): retire
/// through [`step_evented`], deliver callbacks, map the outcome. Collects
/// (and, with a callback, drains analog words for) events only when a
/// callback observes them, so an armed-but-callback-free run leaves
/// device state exactly as the single-call path would.
fn step_duration_evented(harness: &mut VioArmHandle, seconds: c_double) -> i32 {
    let hz = harness.machine_mut().cpu_hz();
    #[allow(clippy::cast_possible_truncation)]
    let cycles = (seconds * f64::from(hz)) as u64;
    let collect = harness.callback().0.is_some();
    let outcome = step_evented(harness, cycles, collect);
    if fire_callback(harness, &outcome.events).is_err() {
        harness.set_last_halt(HaltOutcome::Fault);
        return VIOARM_ERR_PANIC;
    }
    finish_step(harness, outcome.halt, outcome.hit_bp)
}

/// Outcome of one evented stepping run (breakpoints armed and/or host
/// callback set).
struct EventedOutcome {
    halt: Option<HaltCause>,
    hit_bp: Option<u32>,
    events: Vec<VioArmEvent>,
}

/// Step `max_cycles` of true core cycles one step at a time, checking host
/// breakpoints before every step and — when `collect` — buffering
/// change-only pin events (ports A–D, same watch as `run_quantum`) plus,
/// with `drain_analog`, every staged `apin` word. The same observation the
/// core `run_quantum` makes, through the same primitives in the same
/// order: an armed-but-never-hit run is bit-identical to the single-call
/// path (a dedicated regression test pins this).
fn step_evented(harness: &mut VioArmHandle, max_cycles: u64, drain_analog: bool) -> EventedOutcome {
    let breakpoints: Vec<u32> = harness.breakpoints().to_vec();
    let machine = harness.machine_mut();
    let start = machine.core.cyccnt;
    // Parked on a breakpoint: report the hit with zero steps, like the
    // CLI's check-then-step loop.
    if breakpoints.contains(&machine.core.regs.pc) {
        return EventedOutcome {
            halt: None,
            hit_bp: Some(machine.core.regs.pc),
            events: Vec::new(),
        };
    }
    let mut watch = Vec::with_capacity(64);
    for port in 0..4u8 {
        for pin in 0..16u8 {
            watch.push((port, pin));
        }
    }
    let mut last: Vec<Option<(bool, bool)>> = watch
        .iter()
        .map(|(port, pin)| {
            machine
                .gpio_pin(*port, *pin)
                .map(|(mode, level)| (level, mode.is_driven()))
        })
        .collect();
    let mut events = Vec::new();
    let mut halt = None;
    let mut hit_bp = None;
    while machine.core.cyccnt.wrapping_sub(start) < max_cycles {
        if breakpoints.contains(&machine.core.regs.pc) {
            hit_bp = Some(machine.core.regs.pc);
            break;
        }
        if let Err(cause) = machine.step() {
            machine.settle_fast();
            halt = Some(cause);
            break;
        }
        let stamp = machine.core.cyccnt;
        for (i, (port, pin)) in watch.iter().enumerate() {
            if let Some((mode, level)) = machine.gpio_pin(*port, *pin) {
                let state = (level, mode.is_driven());
                if last[i] != Some(state) {
                    last[i] = Some(state);
                    events.push(pin_event(stamp, *port, *pin, level, state.1));
                }
            }
        }
        if drain_analog {
            if let Some(mv) = machine.apin_drain_mv() {
                events.push(analog_event(stamp, mv));
            }
        }
    }
    if halt.is_none() {
        machine.settle_fast();
    }
    EventedOutcome {
        halt,
        hit_bp,
        events,
    }
}

/// Build a pin-change event (same layout as the `run_quantum` buffer).
const fn pin_event(cyccnt: u64, port: u8, pin: u8, level: bool, driving: bool) -> VioArmEvent {
    VioArmEvent {
        cyccnt,
        kind: VIOARM_EVENT_PIN,
        port: port as u32,
        pin: pin as u32,
        level: level as u32,
        driving: driving as u32,
        mv: 0,
    }
}

/// Build an analog-word event (same layout as the `run_quantum` buffer).
const fn analog_event(cyccnt: u64, mv: u32) -> VioArmEvent {
    VioArmEvent {
        cyccnt,
        kind: VIOARM_EVENT_ANALOG,
        port: 0,
        pin: 0,
        level: 0,
        driving: 0,
        mv,
    }
}

/// Deliver buffered events to the host callback (if set), in retire order.
/// A (Rust) callback that unwinds aborts with [`VIOARM_ERR_PANIC`];
/// reentrant entries fail inside `with_handle` with
/// [`VIOARM_ERR_REENTRY`].
fn fire_callback(harness: &mut VioArmHandle, events: &[VioArmEvent]) -> Result<(), i32> {
    let (callback, ctx) = harness.callback();
    let Some(callback) = callback else {
        return Ok(());
    };
    harness.set_in_callback(true);
    let outcome = catch_unwind(AssertUnwindSafe(|| {
        for event in events {
            callback(event as *const VioArmEvent, ctx);
        }
    }));
    harness.set_in_callback(false);
    match outcome {
        Ok(()) => Ok(()),
        Err(_) => {
            set_error("host event callback panicked".to_owned());
            Err(VIOARM_ERR_PANIC)
        }
    }
}

/// Map an evented outcome to a status code, recording it for
/// [`vioarm_cosim_halt_reason`].
fn finish_step(harness: &mut VioArmHandle, halt: Option<HaltCause>, hit_bp: Option<u32>) -> i32 {
    match (hit_bp, halt) {
        (Some(pc), _) => {
            harness.set_last_halt(HaltOutcome::Breakpoint(pc));
            set_error(format!("breakpoint at {pc:#010x}"));
            VIOARM_STEP_BREAKPOINT
        }
        (None, Some(cause)) => {
            harness.set_last_halt(HaltOutcome::Fault);
            set_error(format!("halted: {cause}"));
            VIOARM_STEP_HALTED
        }
        (None, None) => {
            harness.set_last_halt(HaltOutcome::Completed);
            VIOARM_STEP_OK
        }
    }
}

/// One machine-driven quantum event (see `docs/src/coupling-contract.md`).
///
/// `kind` is [`VIOARM_EVENT_PIN`] (pin fields valid) or
/// [`VIOARM_EVENT_ANALOG`] (`mv` valid). `cyccnt` is the exact stamp after
/// the retiring step; convert to seconds with `cyccnt / cpu_hz`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct VioArmEvent {
    /// `DWT CYCCNT` after the retiring step.
    pub cyccnt: u64,
    /// [`VIOARM_EVENT_PIN`] or [`VIOARM_EVENT_ANALOG`].
    pub kind: u32,
    /// GPIO port (0 = A; pin events only).
    pub port: u32,
    /// Pin index (pin events only).
    pub pin: u32,
    /// Sampled level (pin events only, 0/1).
    pub level: u32,
    /// Firmware drives the pin (pin events only, 0/1).
    pub driving: u32,
    /// Millivolts of the staged word (analog events only).
    pub mv: u32,
}

/// Machine-driven bounded run: retire at most `seconds` of true core
/// cycles at the live CPU clock, buffering change-only pin events (ports
/// A–D, absent ports never emit) plus every staged `apin` word into the
/// caller-sized `out[cap]` buffer.
///
/// `out_len` (always required) reports the total event count even when it
/// exceeds `cap` — the buffer holds the first `cap` in retire order, so
/// `*out_len > cap` means truncation (drain with a bigger buffer next
/// quantum; the machine already retired, events are not replayed). `out`
/// is required when `cap > 0`. Returns [`VIOARM_STEP_OK`] on a full
/// quantum, [`VIOARM_STEP_HALTED`] on an early halt,
/// [`VIOARM_STEP_BREAKPOINT`] on a host breakpoint, or a negative error
/// code. The SPICE-master [`vioarm_cosim_step_duration`] path is
/// untouched.
///
/// With breakpoints armed and/or an event callback set, stepping retires
/// through the evented loop (same primitives, same order) and callbacks
/// fire per event, for every observed event even when the fixed buffer
/// truncates. See [`vioarm_cosim_step_duration`] for the exact
/// armed-but-never-hit identity guarantee.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI
/// and the null checks make this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_run_quantum(
    handle: *mut VioArmHandle,
    seconds: c_double,
    out: *mut VioArmEvent,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    guard(|| {
        if out_len.is_null() || (out.is_null() && cap > 0) {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        if !seconds.is_finite() || seconds < 0.0 {
            set_error(format!("bad quantum duration {seconds}"));
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            if !harness.breakpoints().is_empty() || harness.callback().0.is_some() {
                return run_quantum_evented(harness, seconds, out, cap, out_len);
            }
            let machine = harness.machine_mut();
            let mut watch = Vec::with_capacity(64);
            for port in 0..4u8 {
                for pin in 0..16u8 {
                    watch.push((port, pin));
                }
            }
            let Some(outcome) = machine.run_quantum_seconds(seconds, &watch) else {
                set_error(format!("bad quantum duration {seconds}"));
                return VIOARM_ERR_BAD_ARG;
            };
            let total = outcome.events.len();
            let stored = total.min(cap);
            // SAFETY CASE leg 6: `out` validated for `cap` elements above,
            // written in-bounds only, never retained.
            unsafe {
                for (i, event) in outcome.events.iter().take(stored).enumerate() {
                    let (kind, port, pin, level, driving, mv) = match event.kind {
                        vioarm_core::couple::CoupledEventKind::Pin {
                            port,
                            pin,
                            level,
                            driving,
                        } => (
                            VIOARM_EVENT_PIN,
                            u32::from(port),
                            u32::from(pin),
                            u32::from(level),
                            u32::from(driving),
                            0,
                        ),
                        vioarm_core::couple::CoupledEventKind::Analog { mv } => {
                            (VIOARM_EVENT_ANALOG, 0, 0, 0, 0, mv)
                        }
                    };
                    out.add(i).write(VioArmEvent {
                        cyccnt: event.cyccnt,
                        kind,
                        port,
                        pin,
                        level,
                        driving,
                        mv,
                    });
                }
                out_len.write(total);
            }
            if total > cap {
                set_error(format!("event buffer truncated ({total} > {cap})"));
            }
            let (code, halt_outcome) = match outcome.halt {
                None => (VIOARM_STEP_OK, HaltOutcome::Completed),
                Some(cause) => {
                    set_error(format!("halted: {cause}"));
                    (VIOARM_STEP_HALTED, HaltOutcome::Fault)
                }
            };
            harness.set_last_halt(halt_outcome);
            code
        })
        .unwrap_or_else(|code| code)
    })
}

/// Evented `run_quantum` (breakpoints armed and/or callback set): retire
/// through [`step_evented`] (always collecting and draining, exactly like
/// the core quantum), fill the caller buffer with the same truncation
/// semantics, deliver callbacks for every observed event, map the
/// outcome.
fn run_quantum_evented(
    harness: &mut VioArmHandle,
    seconds: c_double,
    out: *mut VioArmEvent,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    let hz = harness.machine_mut().cpu_hz();
    #[allow(clippy::cast_possible_truncation)]
    let max_cycles = (seconds * f64::from(hz)) as u64;
    let outcome = step_evented(harness, max_cycles, true);
    let total = outcome.events.len();
    let stored = total.min(cap);
    // SAFETY CASE leg 6: `out` validated for `cap` elements by the entry
    // contract (null iff `cap == 0`); `out_len` always required.
    unsafe {
        for (i, event) in outcome.events.iter().take(stored).enumerate() {
            out.add(i).write(*event);
        }
        out_len.write(total);
    }
    if total > cap {
        set_error(format!("event buffer truncated ({total} > {cap})"));
    }
    if fire_callback(harness, &outcome.events).is_err() {
        harness.set_last_halt(HaltOutcome::Fault);
        return VIOARM_ERR_PANIC;
    }
    finish_step(harness, outcome.halt, outcome.hit_bp)
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

/// Drive the supply rail to `volts` (the `vcc_bridge` path): brown-out
/// is evaluated with hysteresis, so a rail held under ~2.0V holds the
/// machine in reset like silicon. Negative/NaN saturate to 0 mV, huge
/// values to `u32::MAX` mV, infinities are rejected. Returns 0 on
/// success or a negative error code.
#[no_mangle]
pub extern "C" fn vioarm_cosim_set_operating_voltage(
    handle: *mut VioArmHandle,
    volts: c_double,
) -> i32 {
    guard(|| {
        if volts.is_infinite() {
            set_error(format!("bad voltage {volts}"));
            return VIOARM_ERR_BAD_ARG;
        }
        // Saturation is the intent (`as` saturates float-to-int by
        // definition): negatives and NaN land on 0 mV.
        #[allow(clippy::cast_possible_truncation)]
        let mv = (volts * 1000.0) as u32;
        with_handle(handle, |harness| {
            harness.machine_mut().set_operating_millivolts(mv);
            0
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

// ---------------------------------------------------------------------------
// WP-R7 embedding API completion (additive: existing symbols untouched).
// ---------------------------------------------------------------------------

/// Embedding API version (`major.minor.patch`, from the crate version).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct VioArmApiVersion {
    /// Major version (breaking ABI changes).
    pub major: u32,
    /// Minor version (additive entries).
    pub minor: u32,
    /// Patch version (fixes, no surface change).
    pub patch: u32,
}

/// The embedding API version (by value, infallible — no handle needed).
#[must_use]
#[no_mangle]
pub extern "C" fn vioarm_cosim_api_version() -> VioArmApiVersion {
    VioArmApiVersion {
        major: std::env!("CARGO_PKG_VERSION_MAJOR").parse().unwrap_or(0),
        minor: std::env!("CARGO_PKG_VERSION_MINOR").parse().unwrap_or(0),
        patch: std::env!("CARGO_PKG_VERSION_PATCH").parse().unwrap_or(0),
    }
}

/// Snapshot byte size for `handle` (`0` on a null handle): the exact `cap`
/// [`vioarm_cosim_snapshot_save`] needs.
#[no_mangle]
pub extern "C" fn vioarm_cosim_snapshot_size(handle: *mut VioArmHandle) -> usize {
    match catch_unwind(AssertUnwindSafe(|| {
        with_handle(handle, |harness| harness.machine_mut().snapshot_len())
    })) {
        Ok(Ok(len)) => len,
        Ok(Err(_)) | Err(_) => 0,
    }
}

/// Freeze integer CPU state plus the `record --snap` default RAM span
/// (`0x20000000:0x1000`) into the caller-sized `out[cap]` buffer.
/// `out_len` (always required) reports the snapshot size even when `cap`
/// is short: `cap < need` fails closed with [`VIOARM_ERR_BAD_ARG`]
/// (machine untouched), otherwise 0 with the bytes stored. `out` is
/// required when `cap > 0`. See `docs/src/embedding.md` for what
/// snapshots cover and what they deliberately do not.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI
/// and the null checks make this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_snapshot_save(
    handle: *mut VioArmHandle,
    out: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    guard(|| {
        if out_len.is_null() || (out.is_null() && cap > 0) {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            let machine = harness.machine_mut();
            let need = machine.snapshot_len();
            if cap < need {
                // SAFETY CASE leg 7: `out_len` validated non-null above.
                unsafe {
                    out_len.write(need);
                }
                set_error(format!("snapshot needs {need} bytes, got {cap}"));
                return VIOARM_ERR_BAD_ARG;
            }
            // SAFETY CASE leg 7: `out` validated for `cap >= need` bytes.
            let slot = unsafe { std::slice::from_raw_parts_mut(out, need) };
            match machine.snapshot_save(slot) {
                Ok(len) => {
                    unsafe {
                        out_len.write(len);
                    }
                    0
                }
                Err(cause) => {
                    unsafe {
                        out_len.write(need);
                    }
                    set_error(format!("snapshot save failed: {cause}"));
                    VIOARM_ERR_BAD_ARG
                }
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Restore state frozen by [`vioarm_cosim_snapshot_save`]: `buf[len]`
/// must be exactly one snapshot image (magic, layout version, API major,
/// and exact length are validated before anything is touched — a rejected
/// buffer leaves the machine exactly as it was). Returns 0,
/// [`VIOARM_ERR_VERSION`] on a layout/API-major mismatch, or another
/// negative error code.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`].
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_snapshot_restore(
    handle: *mut VioArmHandle,
    buf: *const u8,
    len: usize,
) -> i32 {
    guard(|| {
        if buf.is_null() && len > 0 {
            set_error("null snapshot buffer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            // SAFETY CASE leg 9: null/empty rejected above; consumed
            // synchronously, never retained.
            let image = unsafe {
                if len == 0 {
                    &[][..]
                } else {
                    std::slice::from_raw_parts(buf, len)
                }
            };
            match harness.machine_mut().snapshot_restore(image) {
                Ok(()) => 0,
                Err(
                    cause @ (SnapshotError::BadVersion(_) | SnapshotError::BadApiVersion { .. }),
                ) => {
                    set_error(format!("snapshot restore failed: {cause}"));
                    VIOARM_ERR_VERSION
                }
                Err(cause) => {
                    set_error(format!("snapshot restore failed: {cause}"));
                    VIOARM_ERR_BAD_ARG
                }
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Program counter of `handle` (`0` on a null handle).
#[no_mangle]
pub extern "C" fn vioarm_cosim_get_pc(handle: *mut VioArmHandle) -> u32 {
    match catch_unwind(AssertUnwindSafe(|| {
        with_handle(handle, |harness| harness.machine_mut().core.regs.pc)
    })) {
        Ok(Ok(pc)) => pc,
        Ok(Err(_)) | Err(_) => 0,
    }
}

/// Active stack pointer of `handle` (`0` on a null handle).
#[no_mangle]
pub extern "C" fn vioarm_cosim_get_sp(handle: *mut VioArmHandle) -> u32 {
    match catch_unwind(AssertUnwindSafe(|| {
        with_handle(handle, |harness| harness.machine_mut().core.regs.sp)
    })) {
        Ok(Ok(sp)) => sp,
        Ok(Err(_)) | Err(_) => 0,
    }
}

/// True core cycles (`DWT CYCCNT`, the coupling timebase) of `handle`
/// (`0` on a null handle).
#[no_mangle]
pub extern "C" fn vioarm_cosim_get_cycles(handle: *mut VioArmHandle) -> u64 {
    match catch_unwind(AssertUnwindSafe(|| {
        with_handle(handle, |harness| harness.machine_mut().core.cyccnt)
    })) {
        Ok(Ok(cyccnt)) => cyccnt,
        Ok(Err(_)) | Err(_) => 0,
    }
}

/// Retired instructions (1/step `cycles`, the `run` budget unit) of
/// `handle` (`0` on a null handle).
#[no_mangle]
pub extern "C" fn vioarm_cosim_get_instructions(handle: *mut VioArmHandle) -> u64 {
    match catch_unwind(AssertUnwindSafe(|| {
        with_handle(handle, |harness| harness.machine_mut().cycles())
    })) {
        Ok(Ok(cycles)) => cycles,
        Ok(Err(_)) | Err(_) => 0,
    }
}

/// Read one data-space byte at `addr` into `val_out`: 0 on success,
/// [`VIOARM_ERR_BUS`] when the bus refuses the address (nothing read),
/// or a negative error code. `val_out` must be non-null.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`].
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_read_data(
    handle: *mut VioArmHandle,
    addr: u32,
    val_out: *mut u8,
) -> i32 {
    guard(|| {
        if val_out.is_null() {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            match harness.machine_mut().core.mem().read(addr) {
                Ok(val) => {
                    // SAFETY CASE leg 5: validated non-null above.
                    unsafe {
                        val_out.write(val);
                    }
                    0
                }
                Err(cause) => {
                    set_error(format!("read {addr:#010x}: {cause}"));
                    VIOARM_ERR_BUS
                }
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Write one data-space byte `val` at `addr`: 0 on success,
/// [`VIOARM_ERR_BUS`] when the bus refuses the address (nothing written),
/// or a negative error code.
#[no_mangle]
pub extern "C" fn vioarm_cosim_write_data(handle: *mut VioArmHandle, addr: u32, val: u8) -> i32 {
    guard(|| {
        with_handle(handle, |harness| {
            match harness.machine_mut().core.mem().write(addr, val) {
                Ok(()) => 0,
                Err(cause) => {
                    set_error(format!("write {addr:#010x}: {cause}"));
                    VIOARM_ERR_BUS
                }
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Number of attached devices on `handle` (`0` on a null handle).
#[no_mangle]
pub extern "C" fn vioarm_cosim_device_count(handle: *mut VioArmHandle) -> usize {
    match catch_unwind(AssertUnwindSafe(|| {
        with_handle(handle, |harness| {
            harness.machine_mut().core.mem().devices_mut().len()
        })
    })) {
        Ok(Ok(len)) => len,
        Ok(Err(_)) | Err(_) => 0,
    }
}

/// Copy the `index`-th attached device name (NUL-terminated) into
/// `out[cap]`: 0 on success, [`VIOARM_ERR_BAD_ARG`] on a null/out-of-range
/// argument or a short buffer (the last error notes the needed size).
/// `out` is required when `cap > 0`.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`].
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_device_name(
    handle: *mut VioArmHandle,
    index: usize,
    out: *mut c_char,
    cap: usize,
) -> i32 {
    guard(|| {
        if out.is_null() && cap > 0 {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            let machine = harness.machine_mut();
            let devices = machine.core.mem().devices_mut();
            let Some(device) = devices.get(index) else {
                set_error(format!("no device index {index}"));
                return VIOARM_ERR_BAD_ARG;
            };
            write_str_out(device.name(), out, cap)
        })
        .unwrap_or_else(|code| code)
    })
}

/// Number of known boards (`board::all_boards`).
#[must_use]
#[no_mangle]
pub extern "C" fn vioarm_cosim_board_count() -> usize {
    board::all_boards().len()
}

/// Copy the `index`-th board short name (NUL-terminated) into `out[cap]`:
/// 0 on success, [`VIOARM_ERR_BAD_ARG`] on an out-of-range index or a
/// short buffer (the last error notes the needed size). `out` is required
/// when `cap > 0`.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`].
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_board_name(index: usize, out: *mut c_char, cap: usize) -> i32 {
    guard(|| {
        if out.is_null() && cap > 0 {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        match board::all_boards().get(index) {
            Some((name, _)) => write_str_out(name, out, cap),
            None => {
                set_error(format!("no board index {index}"));
                VIOARM_ERR_BAD_ARG
            }
        }
    })
}

/// Live CPU clock in Hz for `board` (short name, e.g. `"stm32f103"`):
/// builds the board memory and reads its clock handle — no firmware
/// needed, clocks are static configuration. 0 on success (with `hz_out`
/// written) or a negative error code. `board` and `hz_out` must be
/// non-null.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`].
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_board_clock(board: *const c_char, hz_out: *mut u32) -> i32 {
    guard(|| {
        if board.is_null() || hz_out.is_null() {
            set_error("null pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        // SAFETY CASE leg 8: null rejected above; copied out immediately.
        let name = unsafe { CStr::from_ptr(board) }
            .to_str()
            .unwrap_or_default()
            .to_owned();
        let builder = if name == "virtual" {
            None
        } else {
            board::all_boards()
                .into_iter()
                .find(|(entry, _)| *entry == name)
                .map(|(_, build)| build)
        };
        if name != "virtual" && builder.is_none() {
            set_error(format!("unknown board '{name}'"));
            return VIOARM_ERR_BAD_ARG;
        }
        let mem = match builder {
            Some(build) => build(),
            None => board::virtual_board(),
        };
        let mut machine = Machine::for_mem(&name, mem);
        let hz = machine.cpu_hz();
        // SAFETY CASE leg 5: validated non-null above, written once.
        unsafe {
            hz_out.write(hz);
        }
        0
    })
}

/// Copy the default-AF label of `pin` (e.g. `"PA9"`, case-insensitive) on
/// `board` (R6 `pins::pin_labels`, NUL-terminated) into `out[cap]`: 0 on
/// success, [`VIOARM_ERR_BAD_ARG`] on an unknown board/pin or a short
/// buffer (the last error notes the need). All three pointers must be
/// non-null when used (`out` required when `cap > 0`).
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`].
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_board_pin(
    board: *const c_char,
    pin: *const c_char,
    out: *mut c_char,
    cap: usize,
) -> i32 {
    guard(|| {
        if board.is_null() || pin.is_null() || (out.is_null() && cap > 0) {
            set_error("null pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        // SAFETY CASE leg 8: nulls rejected above; copied out immediately.
        let board_name = unsafe { CStr::from_ptr(board) }
            .to_str()
            .unwrap_or_default()
            .to_owned();
        let pin_name = unsafe { CStr::from_ptr(pin) }
            .to_str()
            .unwrap_or_default()
            .to_owned();
        let Some(table) = vioarm_core::pins::pin_labels(&board_name) else {
            set_error(format!(
                "unknown board '{board_name}' — valid: {}",
                vioarm_core::pins::PIN_BOARDS.join(", ")
            ));
            return VIOARM_ERR_BAD_ARG;
        };
        match table
            .iter()
            .find(|(name, _)| name.eq_ignore_ascii_case(&pin_name))
        {
            Some((_, function)) => write_str_out(function, out, cap),
            None => {
                set_error(format!("no pin '{pin_name}' on '{board_name}'"));
                VIOARM_ERR_BAD_ARG
            }
        }
    })
}

/// Reboot `handle` from its vector table without reloading firmware
/// (same [`Machine::reset`] the CLI boots through: peripherals reset,
/// semihost/RTT session buffers clear, backtrace ring restarts, image
/// stays). Host-driven pin levels are forgotten (a fresh boot owns
/// nothing). Returns 0, or [`VIOARM_STEP_HALTED`] when the reset itself
/// faults — then reload or destroy the handle.
#[no_mangle]
pub extern "C" fn vioarm_cosim_reset(handle: *mut VioArmHandle) -> i32 {
    guard(|| {
        with_handle(handle, |harness| {
            let outcome = harness.machine_mut().reset();
            match outcome {
                Ok(()) => {
                    harness.clear_driven();
                    harness.set_last_halt(HaltOutcome::Completed);
                    0
                }
                Err(cause) => {
                    harness.set_last_halt(HaltOutcome::Fault);
                    set_error(format!("reset failed: {cause}"));
                    VIOARM_STEP_HALTED
                }
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Load new firmware into the live `handle` (same board): `fw`/`fw_len`
/// in `fw_format` at `bin_base`, exactly the [`vioarm_cosim_create`]
/// convention, then reset. Breakpoints and the event callback survive
/// (they match PCs, which a new image reuses); host-driven pin levels
/// are forgotten. Returns 0 or a negative error code (the old image
/// keeps running when the load is rejected).
#[no_mangle]
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI
/// and the null checks make this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn vioarm_cosim_load(
    handle: *mut VioArmHandle,
    fw: *const u8,
    fw_len: usize,
    fw_format: u32,
    bin_base: u32,
) -> i32 {
    guard(|| {
        if fw.is_null() || fw_len == 0 {
            set_error("null or empty firmware".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            // SAFETY CASE leg 2: null/empty rejected above; consumed
            // synchronously by the loader, never retained.
            let image = unsafe { std::slice::from_raw_parts(fw, fw_len) };
            let board = harness.board().to_owned();
            match load_machine(&board, image, fw_format, bin_base) {
                Ok(machine) => {
                    *harness.machine_mut() = machine;
                    harness.clear_driven();
                    harness.set_last_halt(HaltOutcome::Completed);
                    0
                }
                Err(code) => code,
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Arm an execution breakpoint at `pc`: stepping calls park before the
/// step there and report [`VIOARM_STEP_BREAKPOINT`] (see
/// [`vioarm_cosim_halt_reason`]). At most 256 (bounds the per-step scan);
/// arming the same `pc` twice is [`VIOARM_ERR_BAD_ARG`]. Breakpoints
/// survive `reset`/`load` (they match PCs, not images).
#[no_mangle]
pub extern "C" fn vioarm_cosim_break_set(handle: *mut VioArmHandle, pc: u32) -> i32 {
    guard(|| {
        with_handle(handle, |harness| {
            if harness.break_add(pc) {
                0
            } else {
                set_error(format!("breakpoint {pc:#010x} already armed or table full"));
                VIOARM_ERR_BAD_ARG
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Disarm the breakpoint at `pc` (idempotent: always 0 on a live handle).
#[no_mangle]
pub extern "C" fn vioarm_cosim_break_clear(handle: *mut VioArmHandle, pc: u32) -> i32 {
    guard(|| {
        with_handle(handle, |harness| {
            harness.break_remove(pc);
            0
        })
        .unwrap_or_else(|code| code)
    })
}

/// List armed breakpoint PCs into caller-sized `out[cap]` (`out_len`
/// reports the total; truncation is noted, never an error — same
/// discipline as the event buffer). `out` is required when `cap > 0`.
#[no_mangle]
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI
/// and the null checks make this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn vioarm_cosim_break_list(
    handle: *mut VioArmHandle,
    out: *mut u32,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    guard(|| {
        if out_len.is_null() || (out.is_null() && cap > 0) {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            let armed = harness.breakpoints().to_vec();
            let total = armed.len();
            let stored = total.min(cap);
            if stored > 0 {
                // Caller-sized for `stored` words (checked above).
                unsafe {
                    ptr::copy_nonoverlapping(armed.as_ptr(), out, stored);
                    out_len.write(total);
                }
            } else {
                // SAFETY: `out_len` validated non-null above.
                unsafe {
                    out_len.write(total);
                }
            }
            if total > cap {
                set_error(format!("breakpoint list truncated ({total} > {cap})"));
            }
            0
        })
        .unwrap_or_else(|code| code)
    })
}

/// How the last stepping call ended: [`VIOARM_HALT_COMPLETED`] (0, also
/// before any stepping call and on a null handle),
/// [`VIOARM_HALT_BREAKPOINT`] (1, parked — [`vioarm_cosim_get_pc`] names
/// the breakpoint), or [`VIOARM_HALT_FAULT`] (2, detail in
/// [`vioarm_cosim_last_error`]). Infallible.
#[must_use]
#[no_mangle]
pub extern "C" fn vioarm_cosim_halt_reason(handle: *mut VioArmHandle) -> u32 {
    match catch_unwind(AssertUnwindSafe(|| {
        with_handle(handle, |harness| harness.last_halt())
    })) {
        Ok(Ok(HaltOutcome::Completed)) => VIOARM_HALT_COMPLETED,
        Ok(Ok(HaltOutcome::Breakpoint(_))) => VIOARM_HALT_BREAKPOINT,
        Ok(Ok(HaltOutcome::Fault)) => VIOARM_HALT_FAULT,
        Ok(Err(_)) | Err(_) => VIOARM_HALT_COMPLETED,
    }
}

/// Read architectural register `reg` (`0..=12` = `r0`–`r12`, `13` = `sp`,
/// `14` = `lr`, `15` = `pc`, `16` = `xpsr` read-only) into `val_out`:
/// 0 on success or a negative error code (`val_out` must be non-null).
#[no_mangle]
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI
/// and the null checks make this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn vioarm_cosim_reg_read(
    handle: *mut VioArmHandle,
    reg: u32,
    val_out: *mut u32,
) -> i32 {
    guard(|| {
        if val_out.is_null() {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        if reg > 16 {
            set_error(format!("no register {reg}"));
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            let machine = harness.machine_mut();
            #[allow(clippy::cast_possible_truncation)]
            let value = if reg == 16 {
                machine.core.xpsr.get()
            } else {
                machine.core.regs.read(reg as u8)
            };
            // Validated non-null above, written once.
            unsafe {
                val_out.write(value);
            }
            0
        })
        .unwrap_or_else(|code| code)
    })
}

/// Write architectural register `reg` (`0..=12` = `r0`–`r12`, `13` = `sp`,
/// `14` = `lr`, `15` = `pc`; `16`/`xpsr` and above are
/// [`VIOARM_ERR_BAD_ARG`]): 0 on success or a negative error code.
#[no_mangle]
pub extern "C" fn vioarm_cosim_reg_write(handle: *mut VioArmHandle, reg: u32, val: u32) -> i32 {
    guard(|| {
        if reg > 15 {
            set_error(format!("register {reg} is read-only or absent"));
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            #[allow(clippy::cast_possible_truncation)]
            harness.machine_mut().core.regs.write(reg as u8, val);
            0
        })
        .unwrap_or_else(|code| code)
    })
}

/// One backtrace-ring entry (see `Machine::backtrace`): the `pc` an
/// instruction retired from, its code halfwords, and its exact post-retire
/// stamp. `op1`/`has_op1` carry the second halfword whenever readable
/// (the consumer decides the encoding width from the opcode, like the
/// CLI's `format_backtrace_op`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct VioArmBacktraceEntry {
    /// Retired-from `pc`.
    pub pc: u32,
    /// Halfword at `pc` (`0` when unreadable).
    pub op0: u16,
    /// Halfword at `pc + 2` (`0` when unreadable).
    pub op1: u16,
    /// Whether `op1` read.
    pub has_op1: u32,
    /// `DWT CYCCNT` after the retiring step.
    pub cyccnt: u64,
}

/// Copy up to `cap` backtrace entries (most recent last) into caller-sized
/// `out[cap]` (`out_len` reports the ring total; truncation is noted,
/// never an error). `out` is required when `cap > 0`. Returns 0 or a
/// negative error code.
#[no_mangle]
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI
/// and the null checks make this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn vioarm_cosim_backtrace(
    handle: *mut VioArmHandle,
    out: *mut VioArmBacktraceEntry,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    guard(|| {
        if out_len.is_null() || (out.is_null() && cap > 0) {
            set_error("null out-pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        with_handle(handle, |harness| {
            let entries = harness.machine_mut().backtrace(cap);
            let total = harness.machine_mut().backtrace(usize::MAX).len();
            let stored = entries.len().min(cap);
            for (i, entry) in entries.iter().take(stored).enumerate() {
                let record = VioArmBacktraceEntry {
                    pc: entry.pc,
                    op0: entry.h0.unwrap_or(0),
                    op1: entry.h1.unwrap_or(0),
                    has_op1: u32::from(entry.h1.is_some()),
                    cyccnt: entry.cyccnt,
                };
                // Caller-sized for `stored` records (checked above).
                unsafe {
                    out.add(i).write(record);
                }
            }
            // SAFETY: `out_len` validated non-null above.
            unsafe {
                out_len.write(total);
            }
            if total > cap {
                set_error(format!("backtrace truncated ({total} > {cap})"));
            }
            0
        })
        .unwrap_or_else(|code| code)
    })
}

/// Drain a host-output channel since the last drain into caller-sized
/// `out[cap]` (`out_len` reports the total; truncation is noted, never an
/// error). `channel` is `"itm"`, `"semihost"` (firmware `printf`
/// retargeting), or an attached device name from
/// [`vioarm_cosim_device_name`] (`"usart1"` drains its TX bytes, ...);
/// anything else is [`VIOARM_ERR_BAD_ARG`]. `out` is required when
/// `cap > 0`. Returns 0 or a negative error code.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`].
#[allow(clippy::not_unsafe_ptr_arg_deref)]
#[no_mangle]
pub extern "C" fn vioarm_cosim_drain(
    handle: *mut VioArmHandle,
    channel: *const c_char,
    out: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    guard(|| {
        if channel.is_null() || out_len.is_null() || (out.is_null() && cap > 0) {
            set_error("null pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        // SAFETY: null rejected above; copied out immediately.
        let name = unsafe { CStr::from_ptr(channel) }
            .to_str()
            .unwrap_or_default()
            .to_owned();
        with_handle(handle, |harness| {
            let machine = harness.machine_mut();
            let bytes = if name == "semihost" {
                machine.semihost_output()
            } else if name == "itm" {
                machine.drain_channel("itm")
            } else {
                let known = machine
                    .core
                    .mem()
                    .devices_mut()
                    .iter()
                    .any(|d| d.name() == name);
                if !known {
                    set_error(format!("unknown channel '{name}'"));
                    return VIOARM_ERR_BAD_ARG;
                }
                machine.drain_device(&name)
            };
            let total = bytes.len();
            let stored = total.min(cap);
            if stored > 0 {
                // Caller-sized for `stored` bytes (checked above).
                unsafe {
                    ptr::copy_nonoverlapping(bytes.as_ptr(), out, stored);
                    out_len.write(total);
                }
            } else {
                // SAFETY: `out_len` validated non-null above.
                unsafe {
                    out_len.write(total);
                }
            }
            if total > cap {
                set_error(format!("drain truncated ({total} > {cap})"));
            }
            0
        })
        .unwrap_or_else(|code| code)
    })
}

/// Inject host bytes into an input channel: `"semihost"` queues for
/// firmware `READC`, otherwise the named device's host-input channel
/// (`"usart1"` feeds its RX, ...). Unknown channels are
/// [`VIOARM_ERR_BAD_ARG`]; `len == 0` is a no-op (then `buf` may be
/// null). Returns 0 or a negative error code.
#[no_mangle]
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately, same
/// as [`vioarm_cosim_destroy`]: unsafety is unenforceable across a C ABI
/// and the null checks make this entry total.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn vioarm_cosim_inject(
    handle: *mut VioArmHandle,
    channel: *const c_char,
    buf: *const u8,
    len: usize,
) -> i32 {
    guard(|| {
        if channel.is_null() || (buf.is_null() && len > 0) {
            set_error("null pointer".to_owned());
            return VIOARM_ERR_BAD_ARG;
        }
        // SAFETY: null rejected above; copied out immediately.
        let name = unsafe { CStr::from_ptr(channel) }
            .to_str()
            .unwrap_or_default()
            .to_owned();
        // Same lending discipline as firmware buffers: consumed
        // synchronously, never retained.
        let bytes = unsafe {
            if len == 0 {
                &[][..]
            } else {
                std::slice::from_raw_parts(buf, len)
            }
        };
        with_handle(handle, |harness| {
            let machine = harness.machine_mut();
            if name == "semihost" {
                machine.semihost_input(bytes);
                0
            } else if machine.drive_device(&name, bytes) {
                0
            } else {
                set_error(format!("unknown channel '{name}'"));
                VIOARM_ERR_BAD_ARG
            }
        })
        .unwrap_or_else(|code| code)
    })
}

/// Install the host event callback fired once per observed event, in
/// retire order, at the end of stepping calls ([`vioarm_cosim_step_duration`]
/// with a callback armed observes the same A–D pin watch as
/// [`vioarm_cosim_run_quantum`]; [`vioarm_cosim_run_quantum`] additionally
/// fills its buffer). `None` disables. `ctx` is opaque (may be null).
/// Callbacks are observe-only: reentrant entries fail with
/// [`VIOARM_ERR_REENTRY`]. Returns 0 or [`VIOARM_ERR_NULL_HANDLE`].
#[no_mangle]
pub extern "C" fn vioarm_cosim_set_event_callback(
    handle: *mut VioArmHandle,
    callback: VioArmEventCallback,
    ctx: *mut c_void,
) -> i32 {
    guard(|| {
        with_handle(handle, |harness| {
            harness.set_callback(callback, ctx);
            0
        })
        .unwrap_or_else(|code| code)
    })
}

/// Copy `text` plus NUL into `out[cap]`; [`VIOARM_ERR_BAD_ARG`] when `cap`
/// is short (the last error notes the needed size).
fn write_str_out(text: &str, out: *mut c_char, cap: usize) -> i32 {
    let need = text.len() + 1;
    if cap < need {
        set_error(format!("name needs {need} bytes, got {cap}"));
        return VIOARM_ERR_BAD_ARG;
    }
    if need == 1 {
        // Empty text with `cap == 0` and null `out` is legal (zero-length
        // names never occur, but the discipline stays total).
        return 0;
    }
    // SAFETY CASE leg 8: caller-sized for `need` bytes (checked above).
    unsafe {
        ptr::copy_nonoverlapping(text.as_ptr().cast::<c_char>(), out, text.len());
        out.add(text.len()).write(0);
    }
    0
}
