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
  with null `out` is legal size-probing. Exception: `drain` is a byte
  stream, not a snapshot — it returns up to `cap` bytes with `out_len`
  reporting this call's count and parks the rest, so small buffers lose
  nothing (drain till empty).
- **Status codes.** `VIOARM_STEP_OK` (0), `VIOARM_STEP_HALTED` (1),
  `VIOARM_STEP_BREAKPOINT` (2, parked before the step at the reported
  `pc`), `VIOARM_ERR_NULL_HANDLE` (-1), `VIOARM_ERR_PANIC` (-2),
  `VIOARM_ERR_BAD_ARG` (-3), `VIOARM_ERR_BUS` (-4, data-space access
  refused), `VIOARM_ERR_VERSION` (-5, snapshot layout/API-major
  mismatch), `VIOARM_ERR_REENTRY` (-6, a host callback called back in).
  There is no budget-exhausted code: stepping entries take seconds and
  run to completion, halt, or breakpoint. Direct value getters
  (`get_pc`, `cpu_hz`, `*_count`, `*_size`) return `0` on a null handle;
  status-returning entries report `VIOARM_ERR_NULL_HANDLE` instead;
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

## Host control (E1)

Execution control, observation, channels, and callbacks — additive entries
(minor version 1.10.0), existing symbols untouched:

- **Execution**: `vioarm_cosim_reset(handle)` reboots from the vector
  table without reloading (image stays; host-driven pin levels are
  forgotten); `vioarm_cosim_load(handle, fw, len, format, base)` swaps in
  a new image on the same board and resets (breakpoints and the callback
  survive; a rejected load keeps the old image running).
- **Breakpoints**: `vioarm_cosim_break_set/clear/list` manage up to 256
  `pc` stops (clear is idempotent). A stepping call parks *before* the
  step at a hit and reports `VIOARM_STEP_BREAKPOINT`;
  `vioarm_cosim_halt_reason(handle)` tells `COMPLETED` (0), `BREAKPOINT`
  (1, `get_pc` names the stop), or `FAULT` (2, detail in the last
  error) for the last stepping call. With nothing armed the stepping
  paths are untouched single-call quanta; armed-but-never-hit runs are
  bit-identical to them (regression-pinned).
- **Observation**: `vioarm_cosim_reg_read/write` (`0..=12` = `r0`–`r12`,
  `13` = `sp`, `14` = `lr`, `15` = `pc`, `16` = `xpsr` read-only);
  `vioarm_cosim_backtrace(handle, out, cap, out_len)` copies the
  fault-triage ring (`pc`, opcode halfwords, post-retire `cyccnt`) most
  recent last, with the usual truncation discipline.
- **Channels**: `vioarm_cosim_drain(handle, channel, out, cap, out_len)`
  (`"itm"`, `"semihost"`, or an attached device name like `"usart1"`)
  returns up to `cap` bytes with `out_len` reporting this call's count:
  channels are byte streams (POSIX-read semantics), so a small buffer
  never loses data — the remainder parks for the next drain; drain till
  `out_len == 0`. `vioarm_cosim_inject(handle, channel, buf, len)`
  queues into `"semihost"` (`READC`) or a device input channel
  (`"usart1"` feeds RX); `len == 0` is a no-op.
- **Callbacks**: `vioarm_cosim_set_event_callback(handle, cb, ctx)`
  (`None` disables) fires once per observed event, in retire order, at
  the end of stepping calls — every event even when the fixed `run_quantum`
  buffer truncates. Callbacks are observe-only: reentrant entries fail
  with `VIOARM_ERR_REENTRY`, and an unwinding (Rust) callback aborts the
  call with `VIOARM_ERR_PANIC`.

Proven by `vioarm-cosim/tests/host.rs` (reset/load round-trips, parked
and listed breakpoints, fault halt reasons, register aliases incl.
read-only `xpsr`, backtrace stamps, UART TX drain, semihost inject,
per-event callback order vs the buffer, reentry refusal, and the
armed-never-hit identity proof).

## Packaging (E2)

- **Artifacts**: `cdylib` (`libvioarm_cosim.so`, soname
  `libvioarm_cosim.so.1`) + `staticlib` from the same crate.
- **Header**: `vioarm-cosim/include/vioarm_cosim.h`, generated by cbindgen
  from `src/lib.rs` on every build; a regression test diffs the fresh
  output against the checked-in copy so drift fails the build. The C++
  co-sim shim (`cosim/`) compiles against the generated copy — the old
  hand-written header is deleted.
- **Discovery**: `packaging/FindVioARM.cmake` (`find_package(VioARM)` →
  `VioARM::vioarm_cosim`) and `packaging/vioarm-cosim.pc` (version pinned
  to the crate by a regression test).
- **Bindings**: `bindings/python/vioarm.py` (ctypes, stdlib only, full ABI
  coverage with a `Machine` context manager).
- **Examples**: `packaging/examples/host_minimal.c` (CMake, <60 lines)
  and `host_minimal.py` — boot → step 1 ms → read a pin — both run in
  `pytest/test_packaging.py`, which also configures the CMake example
  against a staged install to prove `find_package` end to end.
- Install recipe: `packaging/README.md`.

## Validation (E4)

Four suites, green on every push (the threads suite from E3 makes four
with the three below):

- **C conformance** (`pytest/abi_conformance.c`, driven by
  `pytest/test_abi_conformance.py`): every function, every status code,
  null probes, short buffers, version gates, snapshot round-trips — from
  C, against the installed-style header + library. `_Static_assert`s pin
  struct sizes, field offsets, and code values, catching the drift class
  Rust-side tests cannot (it caught a wrong `sizeof` in its own first
  draft).
- **ABI fuzz** (`pytest/fuzz_abi.py`, driven by
  `pytest/test_abi_fuzz.py`): seeded PRNG, random call sequences with
  contract-honest arguments, run in a subprocess (a segfault fails the
  suite instead of killing pytest) with a wall-time backstop and an RSS
  tripwire. Deterministic seeds, so any failure reproduces exactly.
- **Goldens** (`pytest/test_abi_golden.py`): CLI↔ABI parity on real
  firmware — VCD pin-edge stamps vs `run_quantum` event stamps compared
  cyccnt-for-cyccnt, UART drains compared byte-for-byte. Plus
  `pytest/test_coremark_abi.py` (opt-in via `COREMARK_BIN`): a full
  CoreMark through the ABI diffed against the CLI on CRC lines and total
  ticks, with the published known-good asserted for 2K-performance
  profiles.

## Threading and time (E3)

- **One rule: a handle is single-threaded; the host owns locking.**
  Independent handles run race-free on any threads. One handle shared
  under an external mutex behaves exactly like the same quanta run
  serially (regression-pinned). Unguarded concurrent entry trips an
  atomic busy guard and panics loudly — caught at the boundary with the
  offending message in the last error — instead of aliasing the machine.
  (Sequential cross-thread handoffs never trip it; only true overlap
  does.) Callbacks observe the same rule one level down: reentrant
  entries fail with `VIOARM_ERR_REENTRY`.
- **Time model.** Virtual time advances only through stepping calls;
  the library reads no clock and does no networking, ever. A quantum is
  the unit: at most one step's overshoot past its end, exact post-retire
  stamps, monotone within and across quanta. Same image + same input
  quanta (pin drives, injects, voltages) = bit-identical output — across
  threads, processes, and runs.
- **Real-time pacing is host-side.** `packaging/examples/rate_limited.py`
  steps fixed quanta and sleeps the remainder to hold a wall-clock scale;
  when the sim outruns the budget it reports lag instead of spiraling
  (never sleeps negative, never steps extra). `pytest/test_packaging.py`
  pins both regimes: a holdable cadence stays within ±250 ms drift, an
  unholdable one still completes with honest lag.
