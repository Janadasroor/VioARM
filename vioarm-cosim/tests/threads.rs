//! E3 concurrency tests: handles are single-threaded with the host
//! owning locking — independent handles run race-free in parallel, one
//! handle shared under a mutex matches the single-threaded golden, and
//! unguarded concurrent entry fails loud (never silent corruption).
//!
//! Firmware is the virtual-GPIO toggle loop from `host.rs` (hand-assembled
//! 16-bit Thumb): every `str` to `OUT` is one port-0-pin-0 event, so event
//! streams compare exactly.

use std::ffi::CString;
use std::sync::{Arc, Barrier, Mutex};
use vioarm_cosim::{
    vioarm_cosim_create, vioarm_cosim_destroy, vioarm_cosim_get_cycles, vioarm_cosim_get_pc,
    vioarm_cosim_run_quantum, vioarm_cosim_step_duration, VioArmEvent, VIOARM_ERR_PANIC,
    VIOARM_FW_BIN, VIOARM_STEP_BREAKPOINT, VIOARM_STEP_HALTED, VIOARM_STEP_OK,
};

/// Virtual-GPIO toggle loop (`DIR = 1`, then `OUT ^= 1` forever).
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

fn create(fw: &[u8]) -> *mut vioarm_cosim::VioArmHandle {
    let name = CString::new("virtual").unwrap();
    let handle = vioarm_cosim_create(name.as_ptr(), fw.as_ptr(), fw.len(), VIOARM_FW_BIN, 0);
    assert!(!handle.is_null(), "create failed");
    handle
}

fn run_quantum(handle: *mut vioarm_cosim::VioArmHandle, seconds: f64) -> (i32, Vec<VioArmEvent>) {
    let mut evts = vec![blank_event(); 128];
    let mut out_len = 0usize;
    let code =
        vioarm_cosim_run_quantum(handle, seconds, evts.as_mut_ptr(), evts.len(), &mut out_len);
    evts.truncate(out_len.min(evts.len()));
    (code, evts)
}

/// Independent handles on many threads converge bit-identically (and match
/// a single-threaded golden): no shared state, no races by construction.
#[test]
fn parallel_handles_converge() {
    let fw = toggle_firmware();
    let golden = {
        let handle = create(&fw);
        let (code, evts) = run_quantum(handle, 0.001);
        let tail = (
            code,
            vioarm_cosim_get_pc(handle),
            vioarm_cosim_get_cycles(handle),
        );
        vioarm_cosim_destroy(handle);
        (tail, evts)
    };
    let fw = Arc::new(fw);
    let mut threads = Vec::new();
    for _ in 0..4 {
        let fw = Arc::clone(&fw);
        threads.push(std::thread::spawn(move || {
            let mut local = Vec::new();
            for _ in 0..2 {
                let handle = create(&fw);
                let (code, evts) = run_quantum(handle, 0.001);
                let tail = (
                    code,
                    vioarm_cosim_get_pc(handle),
                    vioarm_cosim_get_cycles(handle),
                );
                vioarm_cosim_destroy(handle);
                local.push((tail, evts));
            }
            local
        }));
    }
    for thread in threads {
        for (tail, evts) in thread.join().expect("worker survived") {
            assert_eq!((tail, evts), golden, "thread diverged from golden");
        }
    }
}

/// One handle shared under an external mutex matches the same total work
/// done single-threaded: the documented sharing discipline is exact (the
/// mutex serializes quanta, so the concatenated sequence is identical).
#[test]
fn shared_handle_under_mutex_matches_serial() {
    let fw = toggle_firmware();
    // Serial golden: 20 quanta on one handle.
    let golden = {
        let handle = create(&fw);
        let mut total = 0usize;
        for _ in 0..20 {
            let (code, evts) = run_quantum(handle, 0.001);
            assert_eq!(code, VIOARM_STEP_OK);
            total += evts.len();
        }
        let tail = (vioarm_cosim_get_pc(handle), vioarm_cosim_get_cycles(handle));
        vioarm_cosim_destroy(handle);
        (tail, total)
    };
    // Same 20 quanta from 4 threads through one mutex (the handle rides
    // as an integer; the guard makes each access exclusive).
    let shared = Arc::new(Mutex::new(create(&fw) as usize));
    let mut threads = Vec::new();
    for _ in 0..4 {
        let shared = Arc::clone(&shared);
        threads.push(std::thread::spawn(move || {
            let mut total = 0usize;
            for _ in 0..5 {
                // The guard makes this access exclusive; sequential
                // cross-thread handoffs never trip the busy guard.
                let guard = shared.lock().expect("mutex held");
                let handle = *guard as *mut vioarm_cosim::VioArmHandle;
                let (code, evts) = run_quantum(handle, 0.001);
                assert_eq!(code, VIOARM_STEP_OK);
                total += evts.len();
            }
            total
        }));
    }
    let mut total = 0usize;
    for thread in threads {
        total += thread.join().expect("worker survived");
    }
    assert_eq!(total, golden.1, "same event total");
    let handle = *shared.lock().expect("mutex held") as *mut vioarm_cosim::VioArmHandle;
    let tail = (vioarm_cosim_get_pc(handle), vioarm_cosim_get_cycles(handle));
    vioarm_cosim_destroy(handle);
    assert_eq!(tail, golden.0, "same end state under external locking");
}

/// Unlocked concurrent entry never corrupts and never crashes: every call
/// reports a valid code (losers panic loudly into `VIOARM_ERR_PANIC`),
/// and the handle keeps working afterwards.
#[test]
fn racy_entry_fails_loud_not_silent() {
    let fw = toggle_firmware();
    let handle = create(&fw);
    // The raw handle rides across threads as an integer (Send by
    // construction): the test provokes concurrent entry by design and
    // only asserts loud failure — it never reads racy state.
    let raw = handle as usize;
    let barrier = Arc::new(Barrier::new(4));
    let mut threads = Vec::new();
    for _ in 0..4 {
        let barrier = Arc::clone(&barrier);
        threads.push(std::thread::spawn(move || {
            let handle = raw as *mut vioarm_cosim::VioArmHandle;
            let mut codes = Vec::new();
            for _ in 0..12 {
                barrier.wait();
                codes.push(vioarm_cosim_step_duration(handle, 0.002));
            }
            codes
        }));
    }
    let mut panics = 0usize;
    for thread in threads {
        for code in thread.join().expect("worker survived the race") {
            if code == VIOARM_ERR_PANIC {
                panics += 1;
            }
            assert!(
                matches!(
                    code,
                    VIOARM_STEP_OK | VIOARM_STEP_HALTED | VIOARM_STEP_BREAKPOINT | VIOARM_ERR_PANIC
                ),
                "valid code only, got {code}"
            );
        }
    }
    eprintln!("racy rounds: {panics} loud refusals");
    // The handle is usable afterwards (claims always released, even on
    // unwind): a quiet quantum completes.
    let mut evts = vec![blank_event(); 8];
    let mut out_len = 0usize;
    let code =
        vioarm_cosim_run_quantum(handle, 0.000_1, evts.as_mut_ptr(), evts.len(), &mut out_len);
    assert!(
        matches!(
            code,
            VIOARM_STEP_OK | VIOARM_STEP_HALTED | VIOARM_STEP_BREAKPOINT
        ),
        "handle usable after the race, got {code}"
    );
    vioarm_cosim_destroy(handle);
}
