# Architecture

- `Machine` + `SystemMemory` + `FlatMemory` + `Peripheral` trait
- `RegisterFile` + `Behavior` + SVD tables (`vioarm-svd`)
- `DWT` `cyccnt`, `SCB` `MPU` `TrustZone`, `FPU` softfloat; `M33` mainline = full `Thumb-2` + `DSP` + `FPv5-SP`, diffed vs `mps2-an505`
- Determinism: single-threaded, `BTreeMap`, no `HashMap` in state
- Clocks: family-aware `RCC` (`F1`/`F4`/`H7` layouts, `SW`+PLL → `SYSCLK` Hz, `H7` `TIMPRE`) → shared `ClockState` → `APB` timer-kernel ratios (×2/×4 rules, exact fractional carry) + `SysTick` `CLKSOURCE` (`HCLK` or `HCLK`/8, `D1CPRE` on `H7`); `FLASH_ACR` wait states + `AXI/FMC` fetch latency priced in core `cyccnt`. `HSE` assumed `8M` (`25M` `H7`); downstream clock gating deliberately unenforced
- Per-Dname model: 32 donor `PERIPHERALS` + removal deltas (`per_dname.rs`, 505 Dnames), filtered at runtime — no per-chip table copies; `F2`/`F7`/`G0` donors are own-`SVD` generation tables (residual per-window diffs inventoried in `Known limitations`)
- DMA request lines: `F1` channels (`DmaRequests` mailbox — `ADC1`→ch1, `TIM2_UP`→ch2, `SPI1` TX/RX→ch3/ch2, `USART` baked routes, wired on `F103`+`F107`) vs `F4` streams (one burst per asserted request, memory-to-memory stays optimistic; wired on every `F4`/`F2`/`F7`/`H7` board under one stream convention — `USART1` on `DMA2 S0/S1`, `USART2/3` on `DMA1 S2/S3`, `UART4` on `DMA1 S4/S5`, `UART5` on `DMA2 S6/S7`, `USART6` on `DMA2 S2/S3`, `TIM1/2/4` update on `DMA1 S6`, `ADC` on `DMA1 S7`, `SPI1` on `DMA1 S0/S1`, `SPI2/3` on `DMA2 S4/S5`; `CHSEL` remapping and the `H7` `DMAMUX` are not modeled)
- `LPTIM` (compare/autoreload matches, prescaler, one-shot, external-edge counting; encoder/capture latch-only) on `F2`/`F7`/`G0`/`G4`/`L4`/`H7`/`M33` boards; `FDCAN` full `M_CAN` (`FDCAN1` on `H743` against verified `SRAMCAN`) + cut-down (`G0`/`G4`/`M33` on verified CAN RAM) + stub elsewhere, loopback-proven board-level (`fdcan_boards.rs`)
