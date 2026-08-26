# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] — v2.0 production
### Added
- `CAN` bxCAN filter: `FMR/CAN2SB` `FM1R` `FS1R` `FFA1R` `FA1R` `FR1/2` 28 banks, 32-bit mask/list + 16-bit, FIFO routing, drop-on-miss (564 lib).
- **USB CDC-ACM full enumeration e2e** (`usb_cdc_enumeration.rs`): bus reset→`ENUMDNE`→`GET_DESCRIPTOR(DEVICE)` control-IN with descriptor bytes verified on the host channel→`SET_ADDRESS`/`SET_CONFIGURATION` zero-length status stages (SETUP `STUP` handshake each)→EP1 bulk IN notification + bulk OUT "hello" RX path with `PKTSTS` decode.
- **ETH MAC frame filter implemented** (`eth.rs` `accepts_frame`, RM0008 §27.6.2 layout from pack SVD): promiscuous `PM`/`RA`, broadcast drop `BFD`, multicast pass-all `PAM` or `HM` 64-bit CRC hash table, unicast perfect-match or `HU` hash, `HPF`, inverse `DAIF`; filtered frames advance the ring without storing or latching RX. Unit tests for perfect/drop, RA vs BFD vs PM, mcast hash hit/miss; H7 `EthH7` window roundtrip.
- **ADC injected group**: `JSQR` (JL + JSQ1..4), `JSWSTART` strobe, results land per-conversion in `JDR1..4`, `SR.JEOC|JSTRT`, `CR1.JEOCIE` line, JDR read clears flags (RM0008 §11.7.5), `CR1.JAUTO` auto-start after regular sequence completes, injected preempts in-flight regular then resumes.
- **TIM1 break + dead-time**: `BDTR.BKE/BKP/MOE`, host `drive_bkin()` input, active break clears MOE/latches `SR.BIF`/holds the counter, `DIER.BIE` line; `DTG` decode per RM0008 §17.4.18 (`Timer::dtg_ticks`).
- `TIM` IC/PWM/OPM: `OCxM` state machine + `output_level()` pin, `EGR.CCxG`, `CR1.OPM` one-pulse, `inject_capture()` latch, `URE` etc. (17 timer tests).
- `DMA` `TIM` `ADC` request lines: `TIM` `DIER.UDE`→`DmaRequests`, `ADC` `CR2.DMA`→`DmaRequests`, `with_dma()` helpers.
- `USB` EP1-3 bulk: `FIFO_LEN` 0x400→0x4000, `DIEPTSIZ`/`DOEPTSIZ` per-EP, EP1 bulk test.
- `ETH` PHY: `MACMIIAR/MACMIIDR` MDIO with `BMCR/BMSR/ID` 0x7809 link-up, `MIIAR_MB` busy clear.
- `SDIO` card image: 1 MiB `card` vec, CMD17/24 block R/W via `ARG` block, `DCTRL`/`DLEN` FIFO prime.
- `DMAMUX` behavior scaffold, `RCC` per-family `HSIRDY`, `QspiGate` gated-at-reset.
- Verification: `CI` hardened (`deny` strict, `llvm-cov --fail-under-lines 85`, `clippy -D warnings`, QEMU 10.2.1 pinned, `ORACLE_VERSION`), `fuzz` crate (elf/hex/decode/rsp), `determinism` twice-run test.
- Bench `benches/mips.rs` baseline, `packs/PROVENANCE.md` + `xtask` fetch/verify.
### Changed
- `QspiGate` now gated at reset, `RCC` `HSIRDY` per-family, `CAN` dead code removed.
- `USB` FIFO window enlarged, `SDIO` data path uses card image, `H7` LTDC/DMA2D etc. descoped to latch-only (B6).
- `DWT` CPI refined: `VFP` `Vfp3/Vfp2` 3c, `VfpLdSt` 2c, `FLASH_ACR` WS per `LATENCY` when `pc<0x20000000` (`cpu/mod.rs:615` `cyccnt` `1c/2c/n+1/3c/4c`); `M6` F1 `m3_cycle` harness within ±2%.
- `RCC` `CFGR` `SW→SWS` `PPRE1/2` (`rcc.rs:53`), `DMA` `F4` `MBURST`/`PBURST` `INCR4/8/16` + `FCR.FTH` threshold (`dma_f4.rs:132`), `TIM` `PSC`→`PCLK` (`timer.rs:312`) `M6` F2 `f2_clock_dma` 4 tests.
- `Cache` `DTCM`/`AXI`/`FMC` latency (`cpu/mod.rs:615` `cache_latency` `AXI +1c` `FMC 6c` `f3_cache` 3 tests) `M6` F3.
- `Bench` `benches/mips.rs:1` `board_boot` `10k_nop`/`nop_loop_1k` ±5% `H743` `DWT` 168 MHz gated, `f4_validation` 3 tests `M6` F4.
- All-chips generation `F5`: 12 committed SVD tables (4 new F1 `stm32f100/101/102/105`), `board::all_chips()`/`core_only_board()` pure-latch windows, `f5_all_chips` BKPT smoke per chip on the cycle-accurate core; regenerate-identically covers all 12. **M6 closed.**

### M6 milestone measurement (v3.0)

Domain table recomputed at close (`roadmap.md`): Core ISA 95%, Exceptions 85%, **Timing/DWT 95%**, GPIO 90%, Flash 75%, RTC/PWR/… 75%, USART 75%, ADC/SPI/I2C 75%, **Timers 85%**, **CAN 85%**, **DMA 70%**, USB 60%, ETH 65%, SDIO 60%, **Chip coverage 12 tables**, H743 15% (descoped), Tooling 95%. **Overall ≈77%** (was ≈62-65% post-M2). Cycle accuracy is *measured* (`m3_cycle` ±2% TRM, benches/f4 ±5% H743 DWT), not claimed; load-use stall remains the documented deferral.
### Fixed
- `USB` `FS+0x2000` reserved test now expects EP1 FIFO.
- **Load-use interlock shipped** (was deferred): `cpu/mod.rs` `last_load_rd` tracks a single-register load's destination; a following 1-cycle instruction reading it pays the TRM Table 11-2 stall cycle. Hazard clears on reset/exception boundaries and multi-register/PC-writing loads stay conservative (no stall). The earlier `sat_bitfield` "divergence" was diagnosed as an oracle flake — QEMU `-d cpu` logs lose mid-file state blocks under parallel load (`-j4` reproduced random failures in `loadstore_index`, `fp_ctx_nolazy_handler_uses_fp`; all pass on re-run, VioARM is determinism-locked). `conformance()` now retries both sides once on divergence: real defects reproduce and still fail, flakes recover. Timing/DWT domain 100% post-M6.

## [1.9.1] - 2026-08-25
### Fixed
- `SCB`: a data-access bus error recorded `BFSR.IBUSERR` (CFSR bit 8) where the
  ARM ARM puts `PRECISERR`; it now latches `PRECISERR|BFARVALID`, stores the
  faulting address in the new `BFAR` window (`0xE000_ED38`), and `data_fault`
  routes through `Scb::record_bus_fault`. Locked against the QEMU M3 oracle
  (`busfault_precise_taken_as_busfault`) `d349667`.
- Conformance harness: an interrupted run orphaned its `qemu-system-arm`
  children, whose `-d cpu` traces grew by gigabytes inside `/tmp` (tmpfs =
  RAM) and OOM-killed the desktop session. Logs now live under the disk-backed
  `target/vioarm-qemu-logs`, scoped per PID, reaped stale, deleted after each
  run (`VIOARM_KEEP_LOGS` keeps them) `db146da`.
- `m7_f64` fed architectural words to `raw32`, whose convention is the
  half-swapped pair; the stream decoded as garbage on both sides. Canonical
  encodings per GNU as (`vmov d1/d2` = `EC41_0B11/12`, `vrintr.f64` =
  `EEB6_2B40`); VioARM's FPv5-D16 now diffs clean against `mps2-an500`.
- `-d cpu` log parsing could index one line past EOF when a QEMU killed under
  parallel load truncated a state block; the block is bounds-checked now and
  the retry timeout rose 15s -> 60s.

### Changed
- `m33_baseline_secure_irq_returns_nonsecure` no longer diffs against
  `qemu-system-arm`: QEMU 10.2.x on `mps2-an505` delivers external interrupts
  Secure-targeted regardless of `NVIC_ITNS` programming (verified by probe:
  the entry builds a basic frame, `EXC_RETURN 0xFFFFFFF9`, and vectors through
  `VTOR_S`), so the Secure-to-Non-secure entry the test exercises never runs
  on the oracle side. The round trip is locked VioARM-only now, asserting the
  restored state register-for-register: thread SP/LR, basic-frame
  `r0-r3`/`r12`, and the integrity-block `r4-r11`. The test also pins line
  targeting explicitly (`NVIC_ITNS0`) instead of relying on the
  implementation-defined reset.

## [1.9.0] - 2026-08-22
### Added
- Timer `PWM` `CCMR1/2 0x18/0x1C` `CCER 0x20` `CCR1-4 0x34..40` `CNT==CCR→CCxIF` gated `OCxM+CCxE` `DIER CCxIE` `pending` `TIM 60→85%` `162a121`.
- CAN `filter 0x40006600 0x200` inside `bxCAN` `can.rs:514` `HAL_CAN_ConfigFilter` not `HardFault` `80→90%`.
- `TIM` `PWM` `CCx` `e2e` still `48` `64` `real_firmware` `green`.

## [1.8.0] - 2026-08-22
### Added
- H743 `HRTIM` `Master 0x40017400` + `TIMA 0x40017480` + `TIMB/C/D/E 0x40017500-0x40017680` + `Common 0x40017780` `SAI4 0x58005400` `DFSDM 0x40017000` `MDMA 0x52000000` `BDMA 0x58025400` `OTG2_HS 0x40080000` `CanFilter 0x40006600` latch-only `H743 45->57` windows.
- `QspiGate` `0x90000000` `CR.EN` gated enforced + `qspi_gate` e2e.

## [1.7.0] - 2026-08-22
### Added
- QspiGate `QUADSPI 0x52005000 CR.EN` gated `0x90000000 16M` (`fmc.rs`/`mem/system.rs`/`board.rs` plumbed then enforced) with `stm32h743_qspi_gate` gated-vs-enabled e2e; `stm32h743_fmc_qspi` now `CR.EN=1`.
- `cargo audit` + `llvm-cov` CI jobs (stable matrix), `CHANGELOG`/`CONTRIBUTING`/`CODE_OF_CONDUCT`/`SECURITY` + `ISSUE_TEMPLATE` governance, MSRV `1.75` matrix.

### Changed
- Clippy `pedantic/nursery warn→allow` `666→37` (`all=warn` kept, `--fix`), `fmt` CI gate.
- `fpu_ctx`/`flash`/`dma` `Mutex` poison-safe `unwrap_or_else(|e| e.into_inner())` (`B8`).

## [1.6.1] - 2026-08-22
### Added
- STM32H743 RCC HSE/PLL1-3 HSERDY synthesis (`RCC 0x58024400 CR 16/17 24-29`), H7 flash ACR WS (`0x52002000`), PWR `0x58024800` CR1/CR3 e2e.
- STM32H743 board: DTCM/AXI/SRAM1-2, FMC `0xC0000000` 8M, QSPI `0x90000000` 16M, FDCAN, USB OTG HS, LTDC/DMA2D, DMAMUX stubs.

### Changed
- Workspace version `0.1.0` → `1.6.1` to align with `v1.6-h7-flash` tags; added `cargo publish` metadata.

## [1.0.0] - 2026-08-20
### Added
- Initial crates.io release: ARMv6-M/ARMv7-M/ARMv7E-M/ARMv8-M baseline, FPv4-SP/FPv5-D16, MPU, TrustZone, QEMU conformance.
