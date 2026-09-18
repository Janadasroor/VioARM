# Embedding

```rust
use vioarm_core::{board, Machine, loader::load_elf};
let mut m = Machine::armv7m(board::stm32f103());
let img = load_elf(&elf_bytes).unwrap();
m.load_image(&img);
m.reset().unwrap();
m.run(1_000_000);
```

Implement `Peripheral` trait for custom devices and `attach` to `SystemMemory`.

## ABI contract (frozen as of E0)

The C ABI (`vioarm-cosim`) is the stability surface — the Rust API is
explicitly *not* stable. These rules are frozen; changing any of them is a
major version bump:

- **Opaque handles.** Hosts never see inside `VioArmHandle`. A handle is
  single-threaded: one thread per handle, no sharing (the library never
  locks; misuse is a host bug).
- **No panics cross FFI.** Every entry catches unwind and fails closed;
  `vioarm_cosim_last_error()` gives the reason as a thread-local string
  that stays valid until the next failing call on the same thread — copy
  it before calling back in.
- **Caller-owned buffers.** Every out-pointer has an explicit `cap`; short
  buffers fail closed and report the need (`out_len`, or the last error
  for strings). A null `out` with `cap > 0` is always an error; `cap == 0`
  with null `out` is legal size-probing.
- **Status codes.** `VIOARM_STEP_OK` (0), `VIOARM_STEP_HALTED` (1),
  `VIOARM_ERR_NULL_HANDLE` (-1), `VIOARM_ERR_PANIC` (-2),
  `VIOARM_ERR_BAD_ARG` (-3), `VIOARM_ERR_BUS` (-4, data-space access
  refused), `VIOARM_ERR_VERSION` (-5, snapshot layout/API-major
  mismatch). There is no budget-exhausted code: stepping entries take
  seconds and run to completion or halt. Value-returning getters
  (`get_pc`, `cpu_hz`, `*_count`, `*_size`) return `0` on a null handle;
  `destroy` on null is a no-op; `create` returns null with the reason in
  the last error.
- **Version policy.** `vioarm_cosim_api_version()` tracks the crate
  version. Major bumps may break anything (signatures, semantics,
  snapshot layout); minor bumps only add entries; patch bumps change no
  surface. Snapshots stamp `(magic b"VASN", layout version, API
  major/minor/patch)`; restore fails closed unless magic, layout version,
  *and* API major match (minor/patch are diagnostic only).
- **Snapshots rewind short deterministic CPU+RAM sections** (coverage
  below); they are not full-machine checkpoints, and device/peripheral
  state is deliberately outside them — a host that needs more must
  quiesce peripherals itself before saving.

## C ABI (`vioarm-cosim`, WP-R7)

The pre-R7 surface (`create`/`destroy`/`step`/pins/analog/voltage/clock)
grew five verbs, additive only — existing symbols are untouched. Every
entry keeps the fail-closed discipline (null-checks, `catch_unwind`,
last-error string); status codes are `VIOARM_STEP_OK` (0),
`VIOARM_STEP_HALTED` (1), `VIOARM_ERR_NULL_HANDLE` (-1),
`VIOARM_ERR_PANIC` (-2), `VIOARM_ERR_BAD_ARG` (-3),
`VIOARM_ERR_BUS` (-4, data-space access refused), and
`VIOARM_ERR_VERSION` (-5, snapshot layout/API-major mismatch).

- **Version**: `vioarm_cosim_api_version()` returns the
  `VioArmApiVersion{major, minor, patch}` by value — always in sync with
  the crate version. No handle needed.
- **Snapshots**: `vioarm_cosim_snapshot_size(handle)` is the exact byte
need; `vioarm_cosim_snapshot_save(handle, out, cap, out_len)` freezes
integer CPU state plus the `record --snap` default RAM span
(`0x20000000:0x1000`, so snapshots and goldens agree) into the
caller-sized buffer (`cap < need` fails closed and still reports the
need via `out_len`); `vioarm_cosim_snapshot_restore(handle, buf, len)`
validates magic, layout version, API major, and exact length before
touching anything (layout/API-major mismatch is `VIOARM_ERR_VERSION`,
anything else malformed is `VIOARM_ERR_BAD_ARG`).
  Covered: `r0-r12/sp/msp/psp/lr/pc`, `xpsr`, `control/primask/basepri/
  faultmask/ipsr/itstate/sleeping/event`, `cycles`/`cyccnt`, and the RAM
  span. Deliberately NOT covered: FPU banks, ARMv8-M security banks,
  exclusive monitor, NVIC pending, peripheral/device state,
  semihosting/RTT buffers, backtrace ring, load-use tracker
  (timing-only). Snapshots rewind short deterministic CPU+RAM sections;
  they are not full-machine checkpoints.
- **Introspection**: `vioarm_cosim_get_pc/get_sp` (`u32`),
  `vioarm_cosim_get_cycles` (`DWT CYCCNT`, the coupling timebase),
  `vioarm_cosim_get_instructions` (retired count, the `run` budget
  unit). All return `0` on a null handle.
- **Memory**: `vioarm_cosim_read_data(handle, addr, val_out)` /
  `vioarm_cosim_write_data(handle, addr, val)` move one data-space byte
  with bus-error status (`VIOARM_ERR_BUS` on unmapped addresses —
  never panicking).
- **Catalog**: `vioarm_cosim_device_count/device_name` (attached
  devices), `vioarm_cosim_board_count/board_name` (the frozen 506),
  `vioarm_cosim_board_clock(board, hz_out)` (static clock config, no
  firmware needed), and `vioarm_cosim_board_pin(board, pin, out, cap)`
  over the R6 pin tables (case-insensitive). String outs are
  NUL-terminated; short buffers fail with the need noted in the last
  error.

Proven by `vioarm-cosim/tests/embedding.rs` (version match,
snapshot/step/restore with `pc`+RAM equality, misuse probes, catalog
spot-checks) plus `vioarm-core` snapshot unit tests.
