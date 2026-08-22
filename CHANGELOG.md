# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
