//! Embedding API tests (WP-R7): version, snapshots, introspection, memory
//! peeks, and device/board catalog over the C ABI — the five verbs the
//! pre-R7 surface lacked.
//!
//! Firmware blobs stay hand-assembled Thumb (no new encodings invented
//! beyond the proven `ldr/movs/str/b .` shape): store `0x5A` to the
//! snapshot span, then spin.

use std::ffi::{CStr, CString};
use std::ptr;
use vioarm_cosim::{
    vioarm_cosim_api_version, vioarm_cosim_board_clock, vioarm_cosim_board_count,
    vioarm_cosim_board_name, vioarm_cosim_board_pin, vioarm_cosim_cpu_hz, vioarm_cosim_create,
    vioarm_cosim_destroy, vioarm_cosim_device_count, vioarm_cosim_device_name,
    vioarm_cosim_get_cycles, vioarm_cosim_get_dac_voltage, vioarm_cosim_get_instructions,
    vioarm_cosim_get_pc, vioarm_cosim_get_sp, vioarm_cosim_pin_get, vioarm_cosim_pin_set,
    vioarm_cosim_read_data, vioarm_cosim_run_quantum, vioarm_cosim_set_external_voltage,
    vioarm_cosim_set_operating_voltage, vioarm_cosim_snapshot_restore, vioarm_cosim_snapshot_save,
    vioarm_cosim_snapshot_size, vioarm_cosim_step_duration, vioarm_cosim_write_data,
    VIOARM_ERR_BAD_ARG, VIOARM_ERR_BUS, VIOARM_ERR_NULL_HANDLE, VIOARM_ERR_VERSION, VIOARM_FW_BIN,
    VIOARM_STEP_OK,
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

fn cstring(text: &str) -> CString {
    CString::new(text).unwrap()
}

fn read_byte(handle: *mut vioarm_cosim::VioArmHandle, addr: u32) -> u8 {
    let mut val = 0xFFu8;
    assert_eq!(
        vioarm_cosim_read_data(handle, addr, &mut val),
        0,
        "read {addr:#010x}: {}",
        last_error()
    );
    val
}

fn get_string(mut probe: impl FnMut(*mut i8, usize) -> i32) -> String {
    let mut buf = vec![0i8; 64];
    assert_eq!(probe(buf.as_mut_ptr(), buf.len()), 0, "{}", last_error());
    // SAFETY: written NUL-terminated by the callee; read back immediately.
    unsafe { CStr::from_ptr(buf.as_ptr()) }
        .to_string_lossy()
        .into_owned()
}

#[test]
fn api_version_matches_crate() {
    let _serial = serial();
    let version = vioarm_cosim_api_version();
    let pkg = env!("CARGO_PKG_VERSION");
    let mut parts = pkg.split('.');
    let major: u32 = parts.next().unwrap().parse().unwrap();
    let minor: u32 = parts.next().unwrap().parse().unwrap();
    let patch: u32 = parts.next().unwrap().parse().unwrap();
    assert_eq!(
        (version.major, version.minor, version.patch),
        (major, minor, patch),
        "ABI version tracks the crate version"
    );
}

#[test]
fn snapshot_steps_and_restores_pc_and_ram() {
    let _serial = serial();
    let fw = store_firmware();
    let handle = create("virtual", &fw, 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    assert_eq!(read_byte(handle, 0x2000_0000), 0x5A, "firmware stored");
    let pc = vioarm_cosim_get_pc(handle);
    let cyccnt = vioarm_cosim_get_cycles(handle);
    let insns = vioarm_cosim_get_instructions(handle);
    assert!(insns > 0, "steps retired");
    assert!(cyccnt > 0, "core cycles elapsed");
    assert_ne!(vioarm_cosim_get_sp(handle), 0, "reset sets a stack");

    // Save, then mutate past the snapshot (scratch byte the firmware
    // never touches, plus forward motion).
    let need = vioarm_cosim_snapshot_size(handle);
    assert!(need > 0x1000, "regs + 4 KiB span, got {need}");
    let mut buf = vec![0u8; need];
    let mut out_len = usize::MAX;
    assert_eq!(
        vioarm_cosim_snapshot_save(handle, buf.as_mut_ptr(), buf.len(), &mut out_len),
        0,
        "{}",
        last_error()
    );
    assert_eq!(out_len, need);
    const SCRATCH: u32 = 0x2000_0100;
    assert_eq!(
        vioarm_cosim_write_data(handle, SCRATCH, 0xAB),
        0,
        "{}",
        last_error()
    );
    assert_eq!(vioarm_cosim_step_duration(handle, 0.001), VIOARM_STEP_OK);
    assert!(vioarm_cosim_get_cycles(handle) > cyccnt, "moved forward");
    assert_eq!(read_byte(handle, SCRATCH), 0xAB);

    // Rewind: pc, counters, and RAM all match the snapshot.
    assert_eq!(
        vioarm_cosim_snapshot_restore(handle, buf.as_ptr(), buf.len()),
        0,
        "{}",
        last_error()
    );
    assert_eq!(vioarm_cosim_get_pc(handle), pc, "pc rewound");
    assert_eq!(vioarm_cosim_get_cycles(handle), cyccnt, "cyccnt rewound");
    assert_eq!(
        vioarm_cosim_get_instructions(handle),
        insns,
        "retired count rewound"
    );
    assert_eq!(read_byte(handle, SCRATCH), 0x00, "RAM rewound");
    assert_eq!(read_byte(handle, 0x2000_0000), 0x5A, "store survives");
    vioarm_cosim_destroy(handle);
}

#[test]
fn embedding_misuse_fails_closed() {
    let _serial = serial();
    let fw = store_firmware();
    let handle = create("virtual", &fw, 0x0);
    let need = vioarm_cosim_snapshot_size(handle);
    let mut buf = vec![0u8; need];
    let mut out_len = 0usize;
    assert_eq!(
        vioarm_cosim_snapshot_save(handle, buf.as_mut_ptr(), buf.len(), &mut out_len),
        0
    );

    // Null handles fail closed (0 for value queries, codes for actions).
    assert_eq!(vioarm_cosim_snapshot_size(ptr::null_mut()), 0);
    assert_eq!(vioarm_cosim_get_pc(ptr::null_mut()), 0);
    assert_eq!(vioarm_cosim_get_sp(ptr::null_mut()), 0);
    assert_eq!(vioarm_cosim_get_cycles(ptr::null_mut()), 0);
    assert_eq!(vioarm_cosim_get_instructions(ptr::null_mut()), 0);
    assert_eq!(vioarm_cosim_device_count(ptr::null_mut()), 0);
    assert_eq!(
        vioarm_cosim_snapshot_save(ptr::null_mut(), buf.as_mut_ptr(), buf.len(), &mut out_len),
        VIOARM_ERR_NULL_HANDLE
    );
    assert_eq!(
        vioarm_cosim_snapshot_restore(ptr::null_mut(), buf.as_ptr(), buf.len()),
        VIOARM_ERR_NULL_HANDLE
    );
    let mut val = 0u8;
    assert_eq!(
        vioarm_cosim_read_data(ptr::null_mut(), 0x2000_0000, &mut val),
        VIOARM_ERR_NULL_HANDLE
    );
    assert_eq!(
        vioarm_cosim_write_data(ptr::null_mut(), 0x2000_0000, 1),
        VIOARM_ERR_NULL_HANDLE
    );
    let mut name = vec![0i8; 32];
    assert_eq!(
        vioarm_cosim_device_name(ptr::null_mut(), 0, name.as_mut_ptr(), name.len()),
        VIOARM_ERR_NULL_HANDLE
    );

    // Bad pointers and short buffers fail closed (and short saves still
    // report the need, without touching the machine).
    let pc = vioarm_cosim_get_pc(handle);
    assert_eq!(
        vioarm_cosim_snapshot_save(handle, buf.as_mut_ptr(), buf.len(), ptr::null_mut()),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_snapshot_save(handle, ptr::null_mut(), 8, &mut out_len),
        VIOARM_ERR_BAD_ARG
    );
    let mut tiny = vec![0u8; 16];
    let mut tiny_len = 0usize;
    assert_eq!(
        vioarm_cosim_snapshot_save(handle, tiny.as_mut_ptr(), tiny.len(), &mut tiny_len),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(tiny_len, need, "short save reports the need");
    assert_eq!(
        vioarm_cosim_get_pc(handle),
        pc,
        "rejected save touches nothing"
    );
    assert_eq!(
        vioarm_cosim_snapshot_restore(handle, ptr::null(), 8),
        VIOARM_ERR_BAD_ARG
    );
    let mut bad = buf.clone();
    bad[0] ^= 0xFF;
    assert_eq!(
        vioarm_cosim_snapshot_restore(handle, bad.as_ptr(), bad.len()),
        VIOARM_ERR_BAD_ARG,
        "{}",
        last_error()
    );
    assert_eq!(
        vioarm_cosim_get_pc(handle),
        pc,
        "rejected restore touches nothing"
    );
    assert_eq!(
        vioarm_cosim_snapshot_restore(handle, buf.as_ptr(), buf.len() - 1),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_read_data(handle, 0x2000_0000, ptr::null_mut()),
        VIOARM_ERR_BAD_ARG
    );

    // Unmapped data-space bytes refuse with `VIOARM_ERR_BUS`.
    assert_eq!(
        vioarm_cosim_read_data(handle, 0x5000_0000, &mut val),
        VIOARM_ERR_BUS,
        "{}",
        last_error()
    );
    assert_eq!(
        vioarm_cosim_write_data(handle, 0x5000_0000, 1),
        VIOARM_ERR_BUS,
        "{}",
        last_error()
    );

    // Catalog misuse: OOB indices, unknown boards/pins, short buffers.
    assert_eq!(
        vioarm_cosim_board_name(usize::MAX, name.as_mut_ptr(), name.len()),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_board_name(0, ptr::null_mut(), 8),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_device_name(handle, usize::MAX, name.as_mut_ptr(), name.len()),
        VIOARM_ERR_BAD_ARG
    );
    let mut hz = 0u32;
    assert_eq!(
        vioarm_cosim_board_clock(ptr::null(), &mut hz),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_board_clock(cstring("virtual").as_ptr(), ptr::null_mut()),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_board_clock(cstring("stm32f999").as_ptr(), &mut hz),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_board_pin(
            cstring("stm32f999").as_ptr(),
            cstring("PA9").as_ptr(),
            name.as_mut_ptr(),
            name.len()
        ),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_board_pin(
            cstring("stm32f103").as_ptr(),
            cstring("PZ99").as_ptr(),
            name.as_mut_ptr(),
            name.len()
        ),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_board_pin(
            cstring("stm32f103").as_ptr(),
            cstring("PA9").as_ptr(),
            name.as_mut_ptr(),
            4
        ),
        VIOARM_ERR_BAD_ARG,
        "short pin buffer fails, need noted: {}",
        last_error()
    );
    vioarm_cosim_destroy(handle);
}

#[test]
fn catalog_lists_boards_devices_clocks_and_pins() {
    let _serial = serial();
    assert_eq!(vioarm_cosim_board_count(), 506, "frozen board list");
    let first = get_string(|out, cap| vioarm_cosim_board_name(0, out, cap));
    assert!(!first.is_empty(), "board 0 named");
    let last =
        get_string(|out, cap| vioarm_cosim_board_name(vioarm_cosim_board_count() - 1, out, cap));
    assert!(!last.is_empty(), "last board named");

    let fw = store_firmware();
    let handle = create("virtual", &fw, 0x0);
    let devices = vioarm_cosim_device_count(handle);
    assert!(devices > 0, "virtual board attaches devices");
    let dev0 = get_string(|out, cap| vioarm_cosim_device_name(handle, 0, out, cap));
    assert!(!dev0.is_empty(), "device 0 named");
    vioarm_cosim_destroy(handle);

    let mut hz = 0u32;
    assert_eq!(
        vioarm_cosim_board_clock(cstring("stm32f103").as_ptr(), &mut hz),
        0,
        "{}",
        last_error()
    );
    assert_eq!(hz, 8_000_000, "F1 boots on the 8 MHz HSI");
    assert_eq!(
        vioarm_cosim_board_clock(cstring("virtual").as_ptr(), &mut hz),
        0
    );
    assert_eq!(hz, 8_000_000);

    let pa9 = get_string(|out, cap| {
        vioarm_cosim_board_pin(
            cstring("stm32f103").as_ptr(),
            cstring("PA9").as_ptr(),
            out,
            cap,
        )
    });
    assert_eq!(pa9, "USART1_TX");
    // Pin lookup is case-insensitive, like the core catalog.
    let pa9_low = get_string(|out, cap| {
        vioarm_cosim_board_pin(
            cstring("STM32H743").as_ptr(),
            cstring("pa13").as_ptr(),
            out,
            cap,
        )
    });
    assert_eq!(pa9_low, "SWDIO");
}

/// E0 exit: every handle-taking entry fails closed on a null handle (0 for
/// value queries, `VIOARM_ERR_NULL_HANDLE` for actions), `destroy(null)`
/// is a silent no-op, and bad `create` arguments return null with the
/// reason in the last error.
#[test]
fn embedding_null_handles_fail_closed() {
    let _serial = serial();
    let null = ptr::null_mut();
    // Value queries read 0.
    assert_eq!(vioarm_cosim_snapshot_size(null), 0);
    assert_eq!(vioarm_cosim_get_pc(null), 0);
    assert_eq!(vioarm_cosim_get_sp(null), 0);
    assert_eq!(vioarm_cosim_get_cycles(null), 0);
    assert_eq!(vioarm_cosim_get_instructions(null), 0);
    assert_eq!(vioarm_cosim_device_count(null), 0);
    assert_eq!(vioarm_cosim_cpu_hz(null), 0);
    // Actions report `VIOARM_ERR_NULL_HANDLE` (valid sibling args, so the
    // null handle is what fails).
    assert_eq!(
        vioarm_cosim_step_duration(null, 0.001),
        VIOARM_ERR_NULL_HANDLE
    );
    let mut out_len = 0usize;
    assert_eq!(
        vioarm_cosim_run_quantum(null, 0.001, ptr::null_mut(), 0, &mut out_len),
        VIOARM_ERR_NULL_HANDLE
    );
    let (mut level, mut driving) = (0u8, 0u8);
    let mut mode = 0u32;
    assert_eq!(
        vioarm_cosim_pin_get(null, 0, 0, &mut level, &mut driving, &mut mode),
        VIOARM_ERR_NULL_HANDLE
    );
    assert_eq!(vioarm_cosim_pin_set(null, 0, 0, 1), VIOARM_ERR_NULL_HANDLE);
    assert_eq!(
        vioarm_cosim_set_external_voltage(null, 0, 1.0),
        VIOARM_ERR_NULL_HANDLE
    );
    assert_eq!(
        vioarm_cosim_set_operating_voltage(null, 1.0),
        VIOARM_ERR_NULL_HANDLE
    );
    let mut volts = 0.0f64;
    assert_eq!(
        vioarm_cosim_get_dac_voltage(null, 0, &mut volts),
        VIOARM_ERR_NULL_HANDLE
    );
    let mut buf = vec![0u8; 64];
    assert_eq!(
        vioarm_cosim_snapshot_save(null, buf.as_mut_ptr(), buf.len(), &mut out_len),
        VIOARM_ERR_NULL_HANDLE
    );
    assert_eq!(
        vioarm_cosim_snapshot_restore(null, buf.as_ptr(), buf.len()),
        VIOARM_ERR_NULL_HANDLE
    );
    let mut val = 0u8;
    assert_eq!(
        vioarm_cosim_read_data(null, 0x2000_0000, &mut val),
        VIOARM_ERR_NULL_HANDLE
    );
    assert_eq!(
        vioarm_cosim_write_data(null, 0x2000_0000, 1),
        VIOARM_ERR_NULL_HANDLE
    );
    let mut name = vec![0i8; 32];
    assert_eq!(
        vioarm_cosim_device_name(null, 0, name.as_mut_ptr(), name.len()),
        VIOARM_ERR_NULL_HANDLE
    );
    // Destroying null is a silent no-op (no crash, no error latched).
    vioarm_cosim_destroy(null);
    // Bad creates return null with a reason.
    assert!(vioarm_cosim_create(ptr::null(), buf.as_ptr(), buf.len(), VIOARM_FW_BIN, 0).is_null());
    assert!(!last_error().is_empty(), "null board names the reason");
    assert!(vioarm_cosim_create(
        cstring("virtual").as_ptr(),
        ptr::null(),
        0,
        VIOARM_FW_BIN,
        0
    )
    .is_null());
    assert!(vioarm_cosim_create(
        cstring("stm32f999").as_ptr(),
        buf.as_ptr(),
        buf.len(),
        VIOARM_FW_BIN,
        0
    )
    .is_null());
    assert!(
        vioarm_cosim_create(cstring("virtual").as_ptr(), buf.as_ptr(), buf.len(), 99, 0).is_null()
    );
    // Null board/pin names fail closed.
    assert_eq!(
        vioarm_cosim_board_pin(
            ptr::null(),
            cstring("PA9").as_ptr(),
            name.as_mut_ptr(),
            name.len()
        ),
        VIOARM_ERR_BAD_ARG
    );
    assert_eq!(
        vioarm_cosim_board_pin(
            cstring("stm32f103").as_ptr(),
            ptr::null(),
            name.as_mut_ptr(),
            name.len()
        ),
        VIOARM_ERR_BAD_ARG
    );
}

/// E0 exit: snapshot version gates fail closed with `VIOARM_ERR_VERSION`
/// (layout or API major), while content corruption stays
/// `VIOARM_ERR_BAD_ARG` — and rejected buffers never touch the machine.
#[test]
fn embedding_snapshot_versions_fail_closed() {
    let _serial = serial();
    let fw = store_firmware();
    let handle = create("virtual", &fw, 0x0);
    assert_eq!(vioarm_cosim_step_duration(handle, 0.000_01), VIOARM_STEP_OK);
    let pc = vioarm_cosim_get_pc(handle);
    let need = vioarm_cosim_snapshot_size(handle);
    let mut buf = vec![0u8; need];
    let mut out_len = 0usize;
    assert_eq!(
        vioarm_cosim_snapshot_save(handle, buf.as_mut_ptr(), buf.len(), &mut out_len),
        0,
        "{}",
        last_error()
    );
    // Bad magic: malformed buffer, not a version problem.
    let mut bad_magic = buf.clone();
    bad_magic[0] ^= 0xFF;
    assert_eq!(
        vioarm_cosim_snapshot_restore(handle, bad_magic.as_ptr(), bad_magic.len()),
        VIOARM_ERR_BAD_ARG
    );
    // Bad layout version: `VIOARM_ERR_VERSION`.
    let mut bad_ver = buf.clone();
    let ver = u32::from_le_bytes(bad_ver[4..8].try_into().unwrap()).wrapping_add(1);
    bad_ver[4..8].copy_from_slice(&ver.to_le_bytes());
    assert_eq!(
        vioarm_cosim_snapshot_restore(handle, bad_ver.as_ptr(), bad_ver.len()),
        VIOARM_ERR_VERSION,
        "{}",
        last_error()
    );
    assert!(!last_error().is_empty(), "version failure names itself");
    // Bad API major (header tail at 140..144): `VIOARM_ERR_VERSION`.
    let mut bad_api = buf.clone();
    let major = u32::from_le_bytes(bad_api[140..144].try_into().unwrap()).wrapping_add(1);
    bad_api[140..144].copy_from_slice(&major.to_le_bytes());
    assert_eq!(
        vioarm_cosim_snapshot_restore(handle, bad_api.as_ptr(), bad_api.len()),
        VIOARM_ERR_VERSION
    );
    assert_eq!(
        vioarm_cosim_get_pc(handle),
        pc,
        "rejected restores touch nothing"
    );
    // A RAM-only corruption still restores (content, not version).
    let mut dirty_ram = buf.clone();
    dirty_ram[200] ^= 0xFF;
    assert_eq!(
        vioarm_cosim_snapshot_restore(handle, dirty_ram.as_ptr(), dirty_ram.len()),
        0,
        "{}",
        last_error()
    );
    assert_eq!(read_byte(handle, 0x2000_0000 + 200 - 152), dirty_ram[200]);
    vioarm_cosim_destroy(handle);
}
