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

## C ABI (`vioarm-cosim`, WP-R7)

The pre-R7 surface (`create`/`destroy`/`step`/pins/analog/voltage/clock)
grew five verbs, additive only — existing symbols are untouched. Every
entry keeps the fail-closed discipline (null-checks, `catch_unwind`,
last-error string); status codes are `VIOARM_STEP_OK` (0),
`VIOARM_STEP_HALTED` (1), `VIOARM_ERR_NULL_HANDLE` (-1),
`VIOARM_ERR_PANIC` (-2), `VIOARM_ERR_BAD_ARG` (-3), and the new
`VIOARM_ERR_BUS` (-4, data-space access refused).

- **Version**: `vioarm_cosim_api_version()` returns the
  `VioArmApiVersion{major, minor, patch}` by value — always in sync with
  the crate version. No handle needed.
- **Snapshots**: `vioarm_cosim_snapshot_size(handle)` is the exact byte
  need; `vioarm_cosim_snapshot_save(handle, out, cap, out_len)` freezes
  integer CPU state plus the `record --snap` default RAM span
  (`0x20000000:0x1000`, so snapshots and goldens agree) into the
  caller-sized buffer (`cap < need` fails closed and still reports the
  need via `out_len`); `vioarm_cosim_snapshot_restore(handle, buf, len)`
  validates magic/version/exact length before touching anything.
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
