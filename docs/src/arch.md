# Architecture

- `Machine` + `SystemMemory` + `FlatMemory` + `Peripheral` trait
- `RegisterFile` + `Behavior` + SVD tables (`vioarm-svd`)
- `DWT` `cyccnt`, `SCB` `MPU` `TrustZone`, `FPU` softfloat; `M33` mainline = full `Thumb-2` + `DSP` + `FPv5-SP`, diffed vs `mps2-an505`
- Determinism: single-threaded, `BTreeMap`, no `HashMap` in state
- Clocks: family-aware `RCC` (`F1`/`F4`/`H7` layouts, `SW`+PLL → `SYSCLK` Hz, `H7` `TIMPRE`) → shared `ClockState` → `APB` timer-kernel ratios (×2/×4 rules, exact fractional carry) + `SysTick` `CLKSOURCE` (`HCLK` or `HCLK`/8, `D1CPRE` on `H7`); `FLASH_ACR` wait states + `AXI/FMC` fetch latency priced in core `cyccnt`. `HSE` assumed `8M` (`25M` `H7`); downstream clock gating deliberately unenforced
- Per-Dname model: 32 donor `PERIPHERALS` + removal deltas (`per_dname.rs`, 505 Dnames), filtered at runtime — no per-chip table copies; `F2`/`F7`/`G0` donors are own-`SVD` generation tables (residual per-window diffs inventoried in `Known limitations`)
