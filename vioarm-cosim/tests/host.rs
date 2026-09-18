//! E1 host-control tests: reset/load, breakpoints + halt reason, register
//! and backtrace observation, channel drain/inject, and observe-only event
//! callbacks — all through the C ABI.
//!
//! Firmware blobs stay hand-assembled 16-bit Thumb (the `virtual` board is
//! ARMv6-M): the proven `ldr/movs/str/b .` shapes, a GPIO toggle loop on
//! the virtual `Gpio` (`DIR`@+0/`OUT`@+4 at `0x4000_0000`), a UART-TX byte
//! store (`DR` lane 0 at `0x4000_1000`), and an unmapped load that faults.

use std::ffi::{c_void, CStr, CString};
use std::ptr;
use vioarm_cosim::{
    vioarm_cosim_backtrace, vioarm_cosim_break_clear, vioarm_cosim_break_list,
    vioarm_cosim_break_set, vioarm_cosim_create, vioarm_cosim_destroy, vioarm_cosim_drain,
    vioarm_cosim_get_cycles, vioarm_cosim_get_pc, vioarm_cosim_halt_reason, vioarm_cosim_inject,
    vioarm_cosim_load, vioarm_cosim_read_data, vioarm_cosim_reg_read, vioarm_cosim_reg_write,
    vioarm_cosim_reset, vioarm_cosim_run_quantum, vioarm_cosim_set_event_callback,
    vioarm_cosim_step_duration, vioarm_cosim_write_data, VioArmBacktraceEntry, VioArmEvent,
    VIOARM_ERR_BAD_ARG, VIOARM_ERR_NULL_HANDLE, VIOARM_ERR_REENTRY, VIOARM_FW_BIN,
    VIOARM_HALT_BREAKPOINT, VIOARM_HALT_COMPLETED, VIOARM_HALT_FAULT, VIOARM_STEP_BREAKPOINT,
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

fn cstring(text: &str) -> CString {
    CString::new(text).unwrap()
}

fn create(board: &str, fw: &[u8], base: u32) -> *mut vioarm_cosim::VioArmHandle {
    let name = CString::new(board).unwrap();
    let handle = vioarm_cosim_create(name.as_ptr(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, base);
    assert!(!handle.is_null(), "create failed: {}", last_error());
    handle
}

/// `ldr r0, =0x20000000; movs r1, #0x5A; str r1, [r0]; b .` at `0x08`
/// (`ldr r0, [pc, #4]`: fetch pc reads `0x0C`, pool sits at `0x10`).
fn store_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x01, 0x48, 0x5A, 0x21, 0x01, 0x60, 0xFE,
        0xE7,
    ];
    fw.extend_from_slice(&0x2000_0000u32.to_le_bytes());
    fw
}

/// `movs r0, #0xA5; str` target swapped: stores `0xA5` at `0x2000_0100`,
/// then spins. Same vector shape as [`store_firmware`].
fn store_a5_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x01, 0x48, 0xA5, 0x21, 0x01, 0x60, 0xFE,
        0xE7,
    ];
    fw.extend_from_slice(&0x2000_0100u32.to_le_bytes());
    fw
}

/// Virtual-GPIO toggle loop: `DIR = 1`, then `OUT ^= 1` forever.
/// Each `str` to `OUT` is one pin event on port 0 pin 0.
/// (Bytes are little-endian halfwords.)
fn toggle_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, // vector table
        0x05, 0x48, // 0x08: ldr r0, [pc, #20]  (gpio base)
        0x01, 0x21, // 0x0A: movs r1, #1
        0x01, 0x60, // 0x0C: str r1, [r0]       (DIR: pin 0 output)
        0x01, 0x22, // 0x0E: movs r2, #1
        0x41, 0x68, // 0x10: ldr r1, [r0, #4]   (OUT)
        0x51, 0x40, // 0x12: eors r1, r2
        0x41, 0x60, // 0x14: str r1, [r0, #4]   (toggle!)
        0xFB, 0xE7, // 0x16: b 0x10
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // pad to 0x20
    ];
    fw.extend_from_slice(&0x4000_0000u32.to_le_bytes());
    fw
}

/// Virtual-UART TX: stores `'H'` to `DR` lane 0, then spins.
fn uart_tx_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, // vector table
        0x02, 0x48, // 0x08: ldr r0, [pc, #8]   (uart base)
        0x48, 0x21, // 0x0A: movs r1, #'H'
        0x01, 0x70, // 0x0C: strb r1, [r0]
        0xFE, 0xE7, // 0x0E: b .
        0x00, 0x00, 0x00, 0x00, // pad to 0x14
    ];
    fw.extend_from_slice(&0x4000_1000u32.to_le_bytes());
    fw
}

/// Unmapped load: `ldr r0, [r0]` with `r0 = 0x5000_0000` faults
/// (BusFault escalates; the table has no HardFault vector, so the halt
/// is a fault either way).
fn fault_firmware() -> Vec<u8> {
    let mut fw = vec![
        0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, // vector table
        0x01, 0x48, // 0x08: ldr r0, [pc, #4]   (pool at 0x10)
        0x00, 0x68, // 0x0A: ldr r0, [r0]       (faults)
        0x00, 0x00, 0x00, 0x00, // pad to 0x10
    ];
    fw.extend_from_slice(&0x5000_0000u32.to_le_bytes());
    fw
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

fn blank_backtrace() -> VioArmBacktraceEntry {
    VioArmBacktraceEntry {
        pc: 0,
        op0: 0,
        op1: 0,
        has_op1: 0,
        cyccnt: 0,
    }
}

struct CbCtx {
    events: Vec<VioArmEvent>,
}

extern "C" fn record_cb(event: *const VioArmEvent, ctx: *mut c_void) {
    // SAFETY: `ctx` is the live `CbCtx` the test lent; the event pointer
    // is valid for the call.
    unsafe {
        let ctx = &mut *(ctx as *mut CbCtx);
        ctx.events.push(*event);
    }
}

struct ReentryCtx {
    events: usize,
    handle: *mut vioarm_cosim::VioArmHandle,
    nested: Vec<i32>,
}

extern "C" fn reentry_cb(_event: *const VioArmEvent, ctx: *mut c_void) {
    // SAFETY: same lending discipline as `record_cb`.
    unsafe {
        let ctx = &mut *(ctx as *mut ReentryCtx);
        ctx.events += 1;
        ctx.nested.push(vioarm_cosim_step_duration(ctx.handle, 0.0));
    }
}

#[test]
fn reset_reboots_without_reload() {
    let _serial = serial();
    let fw = store_firmware();
    let handle = create("virtual", &fw, 0x0);
    let pc0 = vioarm_cosim_get_pc(handle);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    assert_eq!(vioarm_cosim_halt_reason(handle), VIOARM_HALT_COMPLETED);
    // Scratch RAM, then reset: the store re-executes from the boot vector.
    assert_eq!(vioarm_cosim_write_data(handle, 0x2000_0100, 0xAB), 0);
    assert_eq!(vioarm_cosim_reset(handle), 0, "{}", last_error());
    assert_eq!(vioarm_cosim_get_pc(handle), pc0, "back at the reset vector");
    assert_eq!(vioarm_cosim_halt_reason(handle), VIOARM_HALT_COMPLETED);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    let mut val = 0u8;
    assert_eq!(vioarm_cosim_read_data(handle, 0x2000_0000, &mut val), 0);
    assert_eq!(val, 0x5A, "firmware store re-ran after reset");
    vioarm_cosim_destroy(handle);
}

#[test]
fn load_swaps_image_on_a_live_handle() {
    let _serial = serial();
    let handle = create("virtual", &store_firmware(), 0x0);
    let pc0 = vioarm_cosim_get_pc(handle);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    // Swap in the A5 image: the old store is gone, the new one runs.
    let fw2 = store_a5_firmware();
    assert_eq!(
        vioarm_cosim_load(handle, fw2.as_ptr(), fw2.len(), VIOARM_FW_BIN, 0x0),
        0,
        "{}",
        last_error()
    );
    assert_eq!(vioarm_cosim_get_pc(handle), pc0, "same entry, new image");
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    let mut val = 0u8;
    assert_eq!(vioarm_cosim_read_data(handle, 0x2000_0100, &mut val), 0);
    assert_eq!(val, 0xA5, "new image stored");
    // A rejected load keeps the old image running.
    let pc_before = vioarm_cosim_get_pc(handle);
    assert_eq!(
        vioarm_cosim_load(handle, fw2.as_ptr(), fw2.len(), 99, 0x0),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_get_pc(handle),
        pc_before,
        "rejected load touches nothing"
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn breakpoints_park_and_report() {
    let _serial = serial();
    let handle = create("virtual", &store_firmware(), 0x0);
    // Parked on the reset vector: immediate hit, zero steps.
    let cycles0 = vioarm_cosim_get_cycles(handle);
    assert_eq!(vioarm_cosim_break_set(handle, 0x08), 0);
    assert_eq!(
        vioarm_cosim_step_duration(handle, 0.001),
        VIOARM_STEP_BREAKPOINT
    );
    assert_eq!(vioarm_cosim_halt_reason(handle), VIOARM_HALT_BREAKPOINT);
    assert_eq!(vioarm_cosim_get_pc(handle), 0x08, "parked before the step");
    assert_eq!(
        vioarm_cosim_get_cycles(handle),
        cycles0,
        "parked hit retires nothing"
    );
    // Duplicate arming fails closed.
    assert_eq!(vioarm_cosim_break_set(handle, 0x08), VIOARM_ERR_BAD_ARG);
    // Breakpoint list round-trips.
    let mut pcs = vec![0u32; 4];
    let mut out_len = 0usize;
    assert_eq!(
        vioarm_cosim_break_list(handle, pcs.as_mut_ptr(), pcs.len(), &mut out_len),
        0
    );
    assert_eq!((out_len, pcs[0]), (1, 0x08));
    // Clear and run through: the store retires, halt reason completes.
    assert_eq!(vioarm_cosim_break_clear(handle, 0x08), 0);
    assert_eq!(
        vioarm_cosim_break_clear(handle, 0x08),
        0,
        "clear is idempotent"
    );
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    assert_eq!(vioarm_cosim_halt_reason(handle), VIOARM_HALT_COMPLETED);
    let mut val = 0u8;
    assert_eq!(vioarm_cosim_read_data(handle, 0x2000_0000, &mut val), 0);
    assert_eq!(val, 0x5A);
    vioarm_cosim_destroy(handle);
}

#[test]
fn halt_reason_reports_faults() {
    let _serial = serial();
    let handle = create("virtual", &fault_firmware(), 0x0);
    assert_eq!(vioarm_cosim_halt_reason(handle), VIOARM_HALT_COMPLETED);
    assert_eq!(
        vioarm_cosim_step_duration(handle, 0.001),
        VIOARM_STEP_HALTED
    );
    assert_eq!(vioarm_cosim_halt_reason(handle), VIOARM_HALT_FAULT);
    assert!(!last_error().is_empty(), "fault names itself");
    vioarm_cosim_destroy(handle);
}

#[test]
fn registers_read_and_write() {
    let _serial = serial();
    let handle = create("virtual", &store_firmware(), 0x0);
    let mut val = 0u32;
    assert_eq!(vioarm_cosim_reg_read(handle, 0, &mut val), 0);
    assert_eq!(val, 0, "r0 starts zeroed");
    assert_eq!(vioarm_cosim_reg_write(handle, 1, 0x1234), 0);
    assert_eq!(vioarm_cosim_reg_read(handle, 1, &mut val), 0);
    assert_eq!(val, 0x1234);
    // `pc` aliases `get_pc`; `xpsr` (16) reads with T set on this core.
    assert_eq!(vioarm_cosim_reg_read(handle, 15, &mut val), 0);
    assert_eq!(val, vioarm_cosim_get_pc(handle));
    assert_eq!(vioarm_cosim_reg_read(handle, 16, &mut val), 0);
    assert_ne!(val & 0x0100_0000, 0, "Thumb bit set");
    // Out of range, read-only xpsr writes, and null outs fail closed.
    // Status-returning entries report null handles (the 0-on-null
    // convention is for direct value getters like `get_pc`).
    assert_eq!(
        vioarm_cosim_reg_read(handle, 17, &mut val),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(vioarm_cosim_reg_write(handle, 16, 0), VIOARM_ERR_BAD_ARG);
    assert_eq!(vioarm_cosim_reg_write(handle, 99, 0), VIOARM_ERR_BAD_ARG);
    assert_eq!(
        vioarm_cosim_reg_read(handle, 0, ptr::null_mut()),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_reg_read(ptr::null_mut(), 0, &mut val),
        VIOARM_ERR_NULL_HANDLE
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn backtrace_lists_retired_with_stamps() {
    let _serial = serial();
    let handle = create("virtual", &store_firmware(), 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    let mut entries = vec![blank_backtrace(); 8];
    let mut out_len = 0usize;
    assert_eq!(
        vioarm_cosim_backtrace(handle, entries.as_mut_ptr(), entries.len(), &mut out_len),
        0,
        "{}",
        last_error()
    );
    assert!(out_len >= 1, "ring recorded retires");
    for entry in entries.iter().take(out_len.min(entries.len())) {
        assert!(
            (0x08..=0x0E).contains(&entry.pc),
            "entry inside firmware: {:#010x}",
            entry.pc
        );
        assert_ne!(entry.op0, 0, "opcode halfword captured");
        assert!(entry.cyccnt > 0, "stamps are true cyccnt");
    }
    // Truncation reports the total without failing.
    let mut one = vec![blank_backtrace(); 1];
    assert_eq!(
        vioarm_cosim_backtrace(handle, one.as_mut_ptr(), one.len(), &mut out_len),
        0
    );
    assert!(out_len >= 1);
    // Null handle reads zero entries.
    assert_eq!(
        vioarm_cosim_backtrace(ptr::null_mut(), one.as_mut_ptr(), one.len(), &mut out_len),
        VIOARM_ERR_NULL_HANDLE
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn drain_moves_uart_bytes_and_rejects_unknown_channels() {
    let _serial = serial();
    let handle = create("virtual", &uart_tx_firmware(), 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    let mut buf = vec![0u8; 16];
    let mut out_len = usize::MAX;
    assert_eq!(
        vioarm_cosim_drain(
            handle,
            cstring("uart").as_ptr(),
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len
        ),
        0,
        "{}",
        last_error()
    );
    assert_eq!(out_len, 1, "one TX byte staged");
    assert_eq!(buf[0], b'H', "firmware byte drained");
    // Drained twice: second drain is empty, still success.
    assert_eq!(
        vioarm_cosim_drain(
            handle,
            cstring("uart").as_ptr(),
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len
        ),
        0
    );
    assert_eq!(out_len, 0);
    // Quiet channels drain empty; unknown channels fail closed.
    assert_eq!(
        vioarm_cosim_drain(
            handle,
            cstring("itm").as_ptr(),
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len
        ),
        0
    );
    assert_eq!(out_len, 0);
    assert_eq!(
        vioarm_cosim_drain(
            handle,
            cstring("nope").as_ptr(),
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len
        ),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_drain(
            handle,
            ptr::null(),
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len
        ),
        VIOARM_ERR_BAD_ARG
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn inject_queues_semihost_and_rejects_unknown_channels() {
    let _serial = serial();
    let handle = create("virtual", &store_firmware(), 0x0);
    let bytes = b"hi";
    assert_eq!(
        vioarm_cosim_inject(
            handle,
            cstring("semihost").as_ptr(),
            bytes.as_ptr(),
            bytes.len()
        ),
        0,
        "{}",
        last_error()
    );
    assert_eq!(
        vioarm_cosim_inject(
            handle,
            cstring("nope").as_ptr(),
            bytes.as_ptr(),
            bytes.len()
        ),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_inject(handle, cstring("semihost").as_ptr(), ptr::null(), 4),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_inject(handle, ptr::null(), bytes.as_ptr(), bytes.len()),
        VIOARM_ERR_BAD_ARG
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn callbacks_fire_per_event_in_retire_order() {
    let _serial = serial();
    let handle = create("virtual", &toggle_firmware(), 0x0);
    let mut ctx = CbCtx { events: Vec::new() };
    assert_eq!(
        vioarm_cosim_set_event_callback(handle, Some(record_cb), &mut ctx as *mut _ as *mut c_void),
        0
    );
    let mut evts = vec![blank_event(); 64];
    let mut out_len = 0usize;
    assert_eq!(
        vioarm_cosim_run_quantum(handle, 0.001, evts.as_mut_ptr(), evts.len(), &mut out_len),
        VIOARM_STEP_OK,
        "{}",
        last_error()
    );
    assert!(out_len > 2, "toggle loop emits pin events");
    assert_eq!(
        ctx.events.len(),
        out_len,
        "callback sees every buffered event"
    );
    // Same stream, same order: buffer and callback agree exactly.
    for (buffered, pushed) in evts.iter().zip(ctx.events.iter()) {
        assert_eq!(buffered, pushed);
    }
    // First edge is the DIR write (mode change at level 0), then the OUT
    // bit toggles strictly every edge.
    assert_eq!(ctx.events[0].level, 0);
    assert_eq!(ctx.events[1].level, 1);
    let mut last_cyccnt = 0u64;
    for pair in ctx.events.windows(2) {
        assert_eq!((pair[0].kind, pair[0].port, pair[0].pin), (0, 0, 0));
        assert_ne!(pair[0].level, pair[1].level, "strict alternation");
        assert!(pair[0].cyccnt >= last_cyccnt, "stamps are monotone");
        last_cyccnt = pair[0].cyccnt;
    }
    // Uninstall: stepping goes quiet.
    assert_eq!(
        vioarm_cosim_set_event_callback(handle, None, ptr::null_mut()),
        0
    );
    ctx.events.clear();
    assert_eq!(
        vioarm_cosim_run_quantum(handle, 0.001, evts.as_mut_ptr(), evts.len(), &mut out_len),
        VIOARM_STEP_OK
    );
    assert!(ctx.events.is_empty());
    vioarm_cosim_destroy(handle);
}

#[test]
fn callback_reentry_fails_closed() {
    let _serial = serial();
    let handle = create("virtual", &toggle_firmware(), 0x0);
    let mut ctx = ReentryCtx {
        events: 0,
        handle,
        nested: Vec::new(),
    };
    assert_eq!(
        vioarm_cosim_set_event_callback(
            handle,
            Some(reentry_cb),
            &mut ctx as *mut _ as *mut c_void
        ),
        0
    );
    let mut evts = vec![blank_event(); 64];
    let mut out_len = 0usize;
    // The outer call succeeds; every nested step is refused while the
    // callback runs.
    assert_eq!(
        vioarm_cosim_run_quantum(handle, 0.001, evts.as_mut_ptr(), evts.len(), &mut out_len),
        VIOARM_STEP_OK,
        "{}",
        last_error()
    );
    assert!(ctx.events > 0, "callback ran");
    assert!(
        ctx.nested.iter().all(|code| *code == VIOARM_ERR_REENTRY),
        "nested calls refused: {:?}",
        ctx.nested
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn armed_but_never_hit_matches_the_fast_path() {
    let _serial = serial();
    // Same firmware, same quantum, twice: once plain, once with an
    // unreachable breakpoint armed (plus a callback counting silently).
    // Bit-identical end state proves the evented loop changes nothing
    // when it never fires.
    let run_plain = || {
        let handle = create("virtual", &toggle_firmware(), 0x0);
        let mut evts = vec![blank_event(); 64];
        let mut out_len = 0usize;
        let code =
            vioarm_cosim_run_quantum(handle, 0.001, evts.as_mut_ptr(), evts.len(), &mut out_len);
        let tail = (
            code,
            out_len,
            vioarm_cosim_get_pc(handle),
            vioarm_cosim_get_cycles(handle),
        );
        vioarm_cosim_destroy(handle);
        (tail, evts)
    };
    let run_armed = || {
        let handle = create("virtual", &toggle_firmware(), 0x0);
        assert_eq!(vioarm_cosim_break_set(handle, 0xDEAD_0000), 0);
        let mut ctx = CbCtx { events: Vec::new() };
        assert_eq!(
            vioarm_cosim_set_event_callback(
                handle,
                Some(record_cb),
                &mut ctx as *mut _ as *mut c_void
            ),
            0
        );
        let mut evts = vec![blank_event(); 64];
        let mut out_len = 0usize;
        let code =
            vioarm_cosim_run_quantum(handle, 0.001, evts.as_mut_ptr(), evts.len(), &mut out_len);
        let tail = (
            code,
            out_len,
            vioarm_cosim_get_pc(handle),
            vioarm_cosim_get_cycles(handle),
        );
        vioarm_cosim_destroy(handle);
        (tail, evts, ctx.events)
    };
    let (plain_tail, plain_evts) = run_plain();
    let (armed_tail, armed_evts, pushed) = run_armed();
    assert_eq!(plain_tail, armed_tail, "same code, count, pc, cyccnt");
    assert_eq!(plain_evts, armed_evts, "same event bytes");
    assert_eq!(pushed.len(), plain_tail.1, "callback saw them all");
}
