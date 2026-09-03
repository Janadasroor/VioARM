# Architecture

- `Machine` + `SystemMemory` + `FlatMemory` + `Peripheral` trait
- `RegisterFile` + `Behavior` + SVD tables (`vioarm-svd`)
- `DWT` `cyccnt`, `SCB` `MPU` `TrustZone`, `FPU` softfloat
- Determinism: single-threaded, `BTreeMap`, no `HashMap` in state
- Clocks: `RCC` `HPRE/PPRE` decode → shared `ClockState` → `APB` timer-kernel ratios (×2 rule, exact fractional carry); `FLASH_ACR` wait states + `AXI/FMC` fetch latency priced in core `cyccnt`
- Per-Dname model: 21 donor `PERIPHERALS` + removal deltas (`per_dname.rs`, 505 Dnames), filtered at runtime — no per-chip table copies
