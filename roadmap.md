# VioARM Roadmap

ARM Cortex-M instruction-set simulator written in Rust. Development is
soundness-first and determinism-first, and the project is structured for
extensibility through a clean architecture. This document is the working
planning artifact and is intentionally not version controlled.

## Guiding Principles

- Soundness over speed. Every illegal or impossible state is a typed error,
  never silent undefined behavior; execution is deterministic.
- Modularity: small focused modules, a shared `Peripheral` trait, and an
  observability layer every tool (CLI, tests, debugger) can hook into.
- One canonical decoder/executor. Any future fast path is verified against the
  canonical one by differential testing.
- Conformance, not vibes: real firmware compiled and run through the simulator,
  with traces diffed against a known-good reference implementation.

## Scope

- Architecture targets:
  - ARMv7-M (Cortex-M3/M4) with Thumb-2 and the ARMv7E-M DSP zone - primary.
  - FPv4-SP VFP (Cortex-M4F) with M-profile FP context switching - implemented.
  - ARMv6-M (Cortex-M0/M0+) - implemented.
  - ARMv8-M baseline (Cortex-M23) - implemented; ARMv8-M mainline
    (Cortex-M33: DSP + FPv5-SP, diffed vs mps2-an505) - implemented (WP5).
- Boards: a generic virtual board (CPU + RAM + UART + SysTick) plus STM32F103,
  STM32F072 (F0, M0), STM32F107, STM32F303xE (F3, M4), STM32G474 (G4, M4),
  STM32L476 (L4, M4), STM32H743 (H7, M7), and STM32F407 board models
  backed by SVD-generated register layouts.

## Status

The ARMv7-M integer core, the DSP extension, and the FPv4-SP floating-point
unit are implemented and verified per-instruction against `qemu-system-arm`
(Cortex-M3 and Cortex-M4 board models) by a differential conformance suite.
The ARMv6-M and ARMv8-M baseline cores are verified against the `Cortex-M0`
micro:bit and `Cortex-M33` (`mps2-an505`) references. All 835 workspace
tests pass (615 lib + 73 conformance + 65 real_firmware + 48 e2e + 8 svd + 6 qemu/gdb). See "Implemented" for the full inventory.

### Implemented

Core execution:

- The full ARMv6-M 16-bit instruction set and broad Thumb-2 (ARMv7-M)
  coverage: MOVW/MOVT, data processing with `ThumbExpandImm`, the load/store
  family, LDRD/STRD, LDM/STM, TBB/TBH, divide, multiply (including 64-bit and
  DSP forms), parallel add/subtract with SEL, sign/zero extension, saturating
  and bit-field operations.
- VFP FPv4-SP (Cortex-M4F) plus FPv5-D16 double-precision on
  Cortex-M7/H743 (VFPv5 arithmetic, fused multiply-add, compare,
  conversions single↔double and to/from integer/fixed-point, immediate
  `VMOV.F32`/`VMOV.F64 #imm`, `VRINT*`, `VMRS`/`VMSR`, plus `VLDR`/`VSTR`
  and `VLDM`/`VSTM` for both `S` and `D`) backed by a softfloat
  implementation ported from QEMU (M4F `sz=11` correctly `UNDEFINSTR`,
  H7 double locked with `stm32h743_firmware_does_f64*` firmware).
- M-profile FP context switching: FPCCR ASPEN/LSPEN/LSPACT, FPCAR, FPDSCR,
  CONTROL.FPCA, extended exception frames, and lazy stacking, matching the
  Cortex-M4 oracle.
- Exceptions: vector-table entry, EXC_RETURN handling (basic and FP-carrying),
  SVC, fault escalation to HardFault, lockup on double fault, PRIMASK,
  FAULTMASK, BASEPRI, and IT blocks.

System model:

- Typed error model (`HaltCause`, `BusError`, `ImageError`); every stop is an
  analyzable cause. Undefined instructions and data faults route to HardFault.
- Loaders for raw binaries, ELF32 (Thumb), and Intel HEX, validated before
  execution.
- Peripherals: NVIC (64 external lines with configurable priority and
  `AIRCR.PRIGROUP` grouping), SysTick, UART-Tx, generic and STM32 GPIO, RCC,
  USART1/2/3 (+ UART4/5 on F107/F407 and USART6 on F407), TIM1..TIM7, EXTI, AFIO,
  SYSCFG, CAN1/2, the STM32F1 flash controller
  (`FLASH` register window with KEYR unlock, `SR`/`CR` status synthesis, and
  page/mass erase), the STM32F1 DMA controllers (`DMA1`/`DMA2` with ISR/IFCR
  and the CCR/CNDTR/CPAR/CMAR channel set, memory-to-memory and
  peripheral-request transfers queued through a mailbox the memory bus drains;
  a peripheral-request channel transfers one unit per request-line assertion
  and raises a per-channel NVIC line), the STM32F407 flash controller
  (`FLASH` at `0x4002_3C00` with `KEYR` unlock, `PG`/`SER`/`MER`/`SNB`/`STRT`
  programming and `EOP`/`WRPERR` synthesis plus `EOPIE`/`ERRIE` interrupt
  lines, sector erase via the shared write gate), and the STM32F407 stream
  DMA (`DMA1`/`DMA2` with eight `SxCR`/`SxNDTR`/`SxPAR`/`SxM0AR`/`SxM1AR`/
  `SxFCR` stream blocks, `LISR`/`HISR` flag groups cleared through
  `LIFCR`/`HIFCR`, memory-to-memory transfers with `PSIZE`/`MSIZE` widths and
  `PINC`/`MINC` stepping, and per-stream NVIC lines mapped through the SVD
  IRQ table so scattered lines like `DMA1_Stream7` at IRQ 47 land correctly).
  The 64 KiB CCM RAM at `0x1000_0000` on the F407 is mapped and round-trips
   words. A `DWT` (`0xE000_1000`, `CTRL.CYCCNTENA` + `CYCCNT` R/W, ticked by `cyccnt` delta) exposes the TRM-accurate count. An SCB covers the modeled subset (ICSR, `AIRCR` with
  VECTKEY-protected `PRIGROUP` and `SYSRESETREQ`, `SCR` with `SLEEPONEXIT`,
  `CCR` with `STKALIGN`/`DIV_0_TRP`, `VTOR`, `SHCSR` with software-pended
  SVCall arbitration and `USGFAULTENA`-gated distinct `UsageFault` handling,
  W1C `CFSR`/`HFSR` with fault-status recording from the escalation paths,
  `SHPR1` (configurable `UsageFault` priority), SHPR2/3, CPACR with `NOCP`
  denial of FP access, FPCCR, FPCAR, FPDSCR, and the MPU
  (`MPU_TYPE`/`MPU_CTRL`/`MPU_RNR`/`MPU_RBAR`/`MPU_RASR` with their v7M
  `_A1`/`_A2`/`_A3` aliases, eight unified regions with subregions, the
  `PRIVDEFENA` background, and AP/XN permission checks on every fetch and
  data access, violations escalating to HardFault with `MMFSR`/`MMFAR`). The
  `0x0800_0000` main flash is a write gate shared between the
  `FLASH` device and the memory bus: writes land only while the controller
  is unlocked and `CR.PG` is set, and a `CR.STRT` erase runs lazily before the
  next access to the region. The IWDG and WWDG watchdogs (12-bit and 7-bit
  down-counters with their key/refresh protocols) share a reset latch with the
  bus: an un-refreshed underflow, a refresh above the WWDG window, or the
  window edge on either latches a request the machine folds into a full reset,
  re-booting firmware from the reset vector table. On the `H743` (M7, `FPv5-D16`) the board adds `QUADSPI 0x52005000` `CR.EN` gated `0x90000000` via `QspiGate` `fmc.rs:57`/`mem/system.rs:474` (reads `0` / writes `Ok(())` while `!EN`, `stm32h743_qspi_gate` proves `gated` then `enabled`), `FMC 0x52004000` + `FDCAN1` full `M_CAN` (verified `SRAMCAN`; `FDCAN2` stays a stub) + `LPTIM1-5`, `LTDC 0x50001000` + `DMA2D 0x52001000`, `DMAMUX1/2 0x40020800`, `SDMMC1 0x52007000` (via `SDIO` alias), `ADC1-3` (generic `ISR`/`CR` for `H7` `ADEN`/`ADSTART`), plus ext `SDRAM 0xC0000000` and `DTCM`/`AXI`/`SRAM1/2`; `EthH7 0x40028000` is a single-window MAC stub (`NoBehavior` `eth.rs:771`) and `OCTOSPI2 0x5200A000` is `H723`-only (H743 has `QUADSPI`). Timer and GPIO interrupts
  reach the core through a deterministic peek/consume protocol.

Tooling:

- `vioarm-cli`: run raw/ELF/HEX images on any board, per-step trace, image
  info, and a GDB Remote Serial Protocol server.
- `vioarm-gdb`: an RSP server for `arm-none-eabi-gdb` with register, memory,
  breakpoint, and continue operations.
- `vioarm-qemu`: the differential conformance harness diffing per-instruction
  register, VFP, and FP-context state against `qemu-system-arm`.
- `vioarm-svd`: SVD-driven register-table code generation.
- Tests: unit tests, end-to-end images, firmware compiled with clang in-test,
  and the QEMU conformance suite (skipped gracefully when `qemu-system-arm` is
  absent).

### In progress

- **Done — v2.0 production** (2026-08-25): M1-M5 closed. M1 A1-A3 (CAN filter,
  hygiene, timer IC/PWM/OPM), M2 B1-B6 (TIM/ADC/SPI DMA, DMAMUX `DMAREQ_ID`
  routing `fmc.rs:188` + test, USB EP1-3 bulk `FIFO 0x4000`, ETH PHY `BMSR`,
  SDIO card 1 MiB multi-block, H7 descoped), M3 C1-C4 (CI `ORACLE_VERSION`
  `qemu.rs:132` + `deny`/`cov`/`clippy`, `fuzz/` real `loader::{load_elf,load_hex,load_flat}` `76af9ef`, determinism `real_firmware.rs:4735`, C3 8 extra `c3_qadd` `9cfcef2`), M4 `vioarm-core/benches/mips.rs:1` `criterion` board-boot `6d53430`, M5 `packs/PROVENANCE.md` + `xtask` + `docs/book.toml`.
- **In progress — M6 F1** (2026-08-26): `cpu/mod.rs:615` TRM CPI refined (`VFP` 3c, `flash_ws` `LATENCY` `WS`), `DWT` `cyccnt` ±2% `m3_cycle.rs:1` `coremark_loop` green, `load-use` stall deferred to `F2` (fails `sat_bitfield` when enabled on `FlatMemory`).
- **Done — M6 F2** (2026-08-26): `rcc.rs:53` `CFGR` `SW→SWS` `PPRE1/2` roundtrip, `timer.rs:312` `PSC`→`PCLK` (tested `timer_pwm_at_pclk_psc` 50% duty), `dma_f4.rs:132` `MBURST`/`PBURST` `INCR4/8/16` burst `FCR.FTH` threshold gated, `f2_clock_dma.rs:1` 4 tests green.
- **Done — M6 F3** (2026-08-26): `cpu/mod.rs:615` `cache_latency` `DTCM 0x20000000` `AXI 0x24000000` +1c, `FMC 0xC0000000` SDRAM +5c (6c burst), `f3_cache.rs:1` `dtcm_vs_axi` 2 vs 3c `fmc_6c` green.
- **Done — M6 F4** (2026-08-26): `benches/mips.rs:1` `board_boot` 10k nop `stm32f103_boot_10k_nop` + `nop_loop_1k` ±5% gated vs H743 `DWT` 168 MHz, `f4_validation.rs:1` `bench_mips`/`board_boot`/`etm_proxy` 3 tests green.
- **Done — M6 F5** (2026-08-26): 12 chip tables (4 new F1 `stm32f100/101/102/105`), `board::all_chips()` + `core_only_board()` `NoBehavior` latch, `f5_all_chips.rs:1` all-chips `BKPT` smoke on cycle-accurate core green; **M6 closed**.
- **Done — post-M6 one-by-one (2026-08-30)**: `H743 COMP1@0x58003800 H7 CFGR` `ADC3_Common@0x58026300` `ADC1_2@50000300` `F030/F031 ADC@0x40012400 ISR ADRDY` `F303 COMP/OPAMP/TSC CAN@0x40006400 USB_FS@0x40005C00 ADC1_2 CCR` `TIM4/8/15/16/17/20` `H743 TIM15-17` `L476/G474` fallback latch — `21 SVDs 500 Dnames 0 Reserved` `sbom.json 1.9.1` `ci qemu 10.2.1 deny --deny warnings` `cpu BTreeMap` `dma burst delta` — `615` lib `73` conformance green.
- **Done — WP7 release hardening (2026-09-04)**: `clippy --workspace --all-targets -- -D warnings` zero-warning on stable (1.98) + MSRV 1.83 (2 real finds: BDMA zero-count optimism stated, CAN reset self-assign removed); MSRV `1.75→1.83` (`const_mut_refs` + exclusive ranges; `clap→4.5.37` so pre-1.85 cargo parses); `quick-xml 0.36→0.41` clears 2 high RUSTSECs (tables regenerate identically); `deny.toml` reviewed policy; CI = ubuntu+macos × 1.83/stable + windows build + mdbook + publish dry-run + lcov artifact, all green; oracle gate (proven versions `[10.2.1, 8.2.2]`, foreign versions skip) — `643` lib + `76` conformance green, coverage `85.64%` lines. `RELEASING.md` holds the publish order + signed-tag procedure (both human steps).

## Plan to production — `v1.9.1 → v2.0`

This is the execution plan from the current baseline to a production-grade,
enterprise-usable release. It is written for autonomous execution: every work
package (WP) states its goal, exact files, register-level specification with
reference-manual citations, acceptance criteria, and an effort estimate in
agent-hours (`ah`). Work top-to-bottom inside a milestone; milestones are
ordered by dependency. Never skip an acceptance criterion.

### Agent protocol — how to work here

1. Read this file and `CHANGELOG.md` first. `Status`/`Implemented` below
   describe reality as of `2aba440`; do not re-derive it.
2. One WP per branch/commit-series; one logical change per commit. Commit
   message style: `domain: what changed (effect)` like existing history.
3. Before every commit: `cargo fmt --all && cargo clippy --workspace
   --all-targets && cargo test --workspace` must be green (744+ tests).
4. **Never commit a red suite.** If a test fails, triage it immediately:
   root-cause, then fix the *model* if the model is wrong, fix the *test* only
   if the test encodes something invalid, or gate the scenario behind a
   documented oracle limitation. "Host drift" is not a diagnosis — three real
   defects hid behind that label once already (`d349667` was one of them).
5. Update `CHANGELOG.md` under an `Unreleased` heading for user-visible work;
   update the `Known limitations` section when a scope decision is made.
6. Accuracy rule: when implementing a peripheral, verify every offset and bit
   against BOTH the pack SVD in `packs/` AND the reference manual chapter cited
   in the WP. If pack and RM disagree, follow the CMSIS header in the pack's
   `Device/Include/` (that is what firmware compiles against) and record the
   divergence in the WP notes.
7. No `unsafe` (workspace forbids it). No new dependencies without stating the
   justification in the WP.
8. The QEMU oracle limitations are documented in `Known limitations`. Do not
   try to force a differential test through them; lock semantics VioARM-only
   instead (pattern: `m33_baseline_secure_irq_returns_nonsecure`).

### Baseline snapshot (`2aba440`, v1.9.1)

- 744/744 tests green: 550 lib + 48 machine e2e + 64 compiled-firmware e2e +
  73 conformance (72 diffed vs QEMU M0/M3/M4/M7/M33) + gdb/cli/svd.
- Behavioral peripherals: NVIC, SCB (faults/MPU v7+v8/TrustZone), DWT cyccnt,
  GPIO F1/F4, USART family + F1 DMA request lines, TIM1..7 up-counter + UIF +
  compare-match IRQs, flash F1/F4 program/erase with write-gate, IWDG/WWDG,
  RTC/PWR/BKP/CRC/DAC, ADC F1 sequences, SPI masters, I2C F1 masters, bxCAN
  mailboxes/FIFOs, USB OTG FS device EP0, ETH descriptor rings (F107/F407),
  SDIO command path, H743 CPU (FPv5-D16 diffed) + board bring-up.
- Latch-only stubs (register round-trip, no behavior): H743 DMAMUX1/2, LTDC,
  DMA2D, BDMA, MDMA, SAI4, DFSDM, HRTIM (5 windows), OTG2_HS, EthH7, CanFilter
  window (in-Can latch).
- Known defects tracked as WP-A2/A3 below.

### Measurement methodology

Progress percentages count *observable behavior*, not registered windows:
a peripheral scores by checklist items (each modeled register field group +
each behavior = one item), weighted equally inside the domain. Latch stubs
score 0. Domain % = implemented items / total items on its checklist. Overall
% = unweighted mean of domain percentages (domains listed in the table at the
bottom of this file). Recompute the table at each milestone tag and paste the
numbers into `CHANGELOG.md`.

---

## M1 — Correctness debt (target `v1.10`, ~2 agent-days)

### WP-A1 bxCAN acceptance filtering + per-instance filter window

Goal: `HAL_CAN_ConfigFilter` semantics actually filter frames.

Files: `vioarm-core/src/device/stm32/can.rs`.

Spec (RM0008 §24.7 / RM0090 §41.7):
- Filter banks live at CAN base + `0x200`: 28 banks × 2 words (`FR1`/`FR2`),
  bank n at `0x200 + 8n`. Control registers (RM offsets relative to CAN base):
  `FMR@+0x200` with the `INIT`/`FINIT` init protocol, `FM1R@+0x204` mode per
  bank (mask/list), `FS1R@+0x20C` scale (16-bit ×2 / 32-bit ×1), `FFA1R@+0x214`
  FIFO assignment, `FA1R@+0x21C` activation. If the pack SVD models any of
  these, prefer its offsets; else use the RM values above.
- Filter-window base is per-instance: CAN1 banks at CAN1+`0x200`; **CAN2
  shares CAN1's bank array** starting at bank `CAN2SB` in `FMR` — CAN2
  filtering reads CAN1's banks (this is silicon behavior, not a shortcut).
- Acceptance check per received frame: apply active banks in order until match;
  32-bit mask mode: `(ID & FxR2) == (FxR1 & FxR2)` incl IDE/RTR bits; list
  mode: exact compare against 2 (scale 1) or 4 (scale 0) IDs. Miss-all → frame
  dropped. Match → FIFO per `FFA1R`.
- Keep the in-`Can` latch for bank storage but drive routing decisions from it.
Acceptance:
- Unit tests: mask-hit, mask-miss, list-hit, FIFO assignment, `CAN2SB`
  split (frames visible to CAN2 only above split), drop-on-no-match.
- e2e: CMSIS firmware configures one 32-bit mask filter; host injects a
  matching and a non-matching frame; ISR fires only for the match.
- Remove dead `EXTRA`/`R` consts + self-argument comment in
  `Can::ranges()` (`can.rs:561`).
Effort: 8ah.

### WP-A2 Hygiene sweep

- Delete dead code found in review: `can.rs` consts (see A1), any other
  `const R`/unused helpers flagged by clippy.
- Fix `QspiGate` reset polarity (`fmc.rs:74`): construct `enabled: false`,
  `reset()` keeps `false` (CR.EN resets cleared on silicon); update
  `stm32h743_fmc_qspi` e2e to set `CR.EN=1` before touching `0x90000000`,
  and extend `qspi_gate` e2e to assert gated-at-reset.
- RCC: stop OR-ing both `HSIRDY` bits on every read; select bit per family
  (`Isa`/device table), keep instant-ready synthesis, document the shim in
  the module doc.
Acceptance: full suite green; `grep -n 'EXTRA' can.rs` empty; both e2e names
above green; no behavioral regression in F107/F103 boards (their suites).
Effort: 3ah.

### WP-A3 Timer input capture + observable output state

Goal: close the biggest timer gap honestly.
Spec (RM0008 §17.3.6-17.3.7, RM0090 §26.3.6+):
- Output compare: channel output state machine per `OCxM` (`000`=frozen,
  `001`=active on match, `010`=inactive on match, `011`=toggle, `110`=PWM1,
  `111`=PWM2), honoring `OCxPE` preload and `EGR.CCxG` generate/force events.
  Expose per-channel current output level through a host-visible trait method
  (`TimerChannels::output_level(ch)`) — there are no pins; the trait is the
  pin.
- Input capture: host injects edge events (trait method
  `inject_capture(ch, rising, timestamp)`); hardware latches `CNT` into
  `CCRx`, sets `CCxIF`, honors `ICxF` filter as N-consecutive-sample
  debounce, `CCxP` polarity, `TIxFP` mapping. Chained PWM-input mode
  (`OCxM=000 ICx=...` LSI pattern) optional-stretch.
- One-pulse mode `CR1.OPM`: counter clears CEN on next update event after
  trigger.
Acceptance: unit tests per OCxM class; capture e2e with CMSIS firmware using
interrupt-driven capture; `output_level` sequence test for PWM1 @50% duty over
one ARR period. Existing `tim1_update` stays green.
Effort: 10ah.

---

## M2 — Peripheral depth (target `v1.11`/`v1.12`, ~5 agent-days)

### WP-B1 DMA completeness (F1/F4)

Spec (RM0008 §13, RM0090 §11):
- F4 streams: implement true request-driven transfers (per-request unit step)
  wired from TIM (`TIMx_DMAReq` on UE/CCx), ADC (`ADC_CR2.DMA` on EOC), SPI
  (`SR.RXNE/TXE`), USART (existing). Honor `SxCR.DIR/PINC/MINC/PSIZE/MSIZE/
  CIRC`, `SxFCR.FTH` thresholds, direct/error interrupts `TEIF/FEIF/HTIF`.
- Burst/`MBURST`/`PBURST` + FIFO flush semantics for memory-to-peripheral.
- F1: add TIM-DMA request sources (UE/CCx per channel map table), ADC DMA.
- Arbitration: keep PL-then-number, document that per-request timing is
  instruction-granular.
Acceptance: CMSIS e2e moving an ADC sequence to RAM via DMA with HT/TC
interrupts; SPI TX DMA loopback test; burst-mode unit test asserting beat
counting. Existing suites green.
Effort: 14ah.
Status (WP6 done 2026-09-04): F4 request pacing + wiring shipped (unit
stall/pace/consume/M2M-bypass test, `f407` TIM2→S6 board e2e, F1 sources
completed on `f107`); `HTIF`/`TEIF`/`FEIF` error interrupts, `CIRC`, and the
ADC-sequence/SPI-loopback CMSIS e2es remain open. WP-B2 (DMAMUX) untouched.

### WP-B2 DMAMUX behavioral (H743)

Replace `NoBehavior` with request-line router: `CCR_DMAREQ_ID` selects among
the RM0433 §14 Table request ids; route generated requests (TIM/ADC/SPI/
USART/SAI) to the selected DMAMUX→DMA stream channel; support sync/trigger
fields as latched no-op with documented note.
Acceptance: H7 e2e: USART RX via DMAMUX-routed stream into RAM.
Effort: 6ah.

### WP-B3 USB OTG FS beyond EP0

- Endpoints IN/OUT 1..=4 with `DIEPTSIZ/DOEPTSIZ`, double-packing optional.
- Bulk/interrupt transfer plumbing so a CDC-ACM device class enumerates and
  moves bytes (host injects/extracts via mailbox like EP0).
Acceptance: CMSIS-style CDC firmware enumerates, host writes serial data,
firmware echoes under interrupt; conformance-style state dump test.
Effort: 12ah.

### WP-B4 ETH fixed-link PHY + MDIO

- Model `MDIO` read/write: PHY id registers, BMSR link-up autoneg-done,
  BMCR speed/duplex writable no-op; `MACMIIAR` busy-bit handshake with
  1-tick latency.
- MAC address filtering per `MACFFR`/H7 equivalents: promiscuous/hash modes.
Acceptance: SPL/HAL `ETH_Init` with fixed-link PHY completes; hash-filter
unit tests; existing descriptor-ring e2e green.
Effort: 6ah.

### WP-B5 SDIO block data + card image

- Host-provided card image (`Vec<u8>` or file path feature flag): CMD17/18/24/
  25 single/multi block read/write against the image, `DCTRL` data counters,
  CRC16 generation (static pattern acceptable, documented), `DATAEND/RXNE/
  TXHE` interrupts, ACMD6 width, CMD7 deselect.
Acceptance: FAT-free raw-block e2e: firmware reads sector 0 from injected
image over DMA-less polled path and DMA path.
Effort: 8ah.

### WP-B6 H7 platform decision (scope gate) — DONE 2026-08-25

Decision: **descope data-paths permanently; handshake behavior shipped**.
`LTDC/DMA2D/BDMA/MDMA/SAI4/DFSDM/HRTIM` are handshake-behavioral
(`EN`→1-tick `ISR`+IRQ so HAL init/probing passes — `fmc.rs:271`, 6
suites) with zero data-path (no layers/blend, no DMA payload, no FIFO).
Enterprise value is headless HIL; display/audio data-paths require a
written customer scenario to reopen. Progress counts the handshake, not
the data-path; see updated table and Non-goals.
Acceptance: decision recorded here + CHANGELOG; no code change beyond doc.
Effort: 1ah.

---

## M3 — Verification gates (target `v1.13`, ~3 agent-days)

### WP-C1 CI hardening

- `.github/workflows/ci.yml`: remove `|| true` from `cargo deny check`;
  coverage job gains a threshold gate (`cargo llvm-cov ... --fail-under-lines
  85`) and uploads lcov artifact; clippy becomes `-- -D warnings` once the
  ~13 pre-existing warnings are fixed or explicitly allowed inline.
- Add `cargo audit --deny warnings` policy decision note.
- Pin the QEMU version used for conformance (`sudo apt-get install
  qemu-system-arm=X.Y.Z*` with fallback comment) and print
  `qemu-system-arm --version` in CI logs; store it in
  `vioarm-qemu/src/qemu.rs::ORACLE_VERSION` const surfaced in test output.
Acceptance: deliberately introduce a lint failure / coverage drop / vulnerable
dep on a scratch branch → CI red on each axis.
Effort: 5ah.
Status DONE 2026-09-04 (WP7): deny runs without `|| true`, coverage gate +
lcov artifact live, clippy `-D warnings` zero-warning, audit clean, oracle
pin (`ORACLE_VERSION`/`ORACLE_VERSIONS`) printed in CI logs.

### WP-C2 Fuzzing

- `cargo fuzz` targets (separate `fuzz/` crate, offline-friendly):
  `fuzz_elf_loader`, `fuzz_hex_loader`, `fuzz_decode16/32` (bounded by
  exhaustive sweeps already, so target the *loaders* + GDB RSP byte stream +
  SVD parser), `fuzz_rsp_server`.
- Seed corpus from existing e2e images; 60s/target locally, 5min in nightly CI
  job (non-blocking initially, blocking after two clean weeks).
Acceptance: each target runs clean ≥5min; any crash filed as issue and fixed;
regression tests added per crash.
Effort: 8ah.

### WP-C3 Differential expansion

- Add conformance programs for: load/store multiple with writeback edge cases,
  saturating QADD/QSUB corner values, VFP denormal/rounding matrix (already
  partially covered), IT-block taken-branch matrix on M33 (oracle allows there),
  exclusive-access contention loops.
- Re-enable `m33_baseline_secure_irq_returns_nonsecure` differential
  automatically IF a future oracle delivers NS-targeted lines: detection =
  probe program asserting delivery; keep current VioARM-only lock otherwise.
Acceptance: conformance suite grows by ≥8 programs; suite stays green.
Effort: 6ah.

### WP-C4 Determinism audit

- Machine-level guarantee: identical image + identical host-call sequence ⇒
  identical trace. Audit for HashMap iteration influencing state (use BTreeMap
  or sort), time-based sources (none allowed), thread nondeterminism (machine
  is single-threaded; enforce with `!Send` marker or doc).
- Add `determinism` integration test: run one firmware twice in-process and
  byte-compare traces.
Acceptance: audit notes committed; test green.
Effort: 4ah.

---

## M4 — Performance & scale (target `v1.14`, ~2 agent-days)

### WP-D1 Benchmarks

- `criterion` benches: core MIPS on a fixed compute kernel (CoreMark-like
  loop), boot-to-halt wall time per board, conformance-suite throughput.
- Track numbers in the bench README; regressions >15% fail CI (allow-listed
  noisy benches).
Acceptance: benches in CI (stable toolchain only), baseline recorded.
Effort: 5ah.

### WP-D2 Fast-path equivalence (stretch)

- Optional decode-cache interpreter path behind feature flag; MUST be guarded
  by a differential test running the whole conformance suite twice (canonical
  vs fast) comparing traces byte-for-byte. Non-goal if benchmarks show <20%
  headroom.
Acceptance: either shipped with equivalence proof or closed with measured
justification.
Effort: 16ah stretch.
Status: superseded by M7 — G1 takes the bench gate, G2 takes this package
with the hardened guard, G3 takes the JIT question as a spike.

---

## M5 — Enterprise release (target `v2.0`, ~5 agent-days)

### WP-E1 License compliance (BLOCKER — do first in M5)

- `packs/` contains Keil DFP content under SLA0044 (Arm proprietary;
  redistribution restricted). 216MB in git history.
- Actions: move packs out of the repository (document fetch commands instead);
  keep only the *generated* SVD tables in-tree plus a provenance manifest
  (source pack version + file sha256 per table); add a `cargo xtask packs`
  fetcher for local regeneration; ensure `cargo deny` license policy matches
  vioarm licensing (MIT, per root `Cargo.toml`).
- Rewrite git history or accept history retains the blobs (decision note;
  default: keep history private-side, fresh public repo going forward).
Acceptance: repo tree free of SLA0044 files; provenance manifest complete;
`xtask packs fetch` reproduces tables byte-identically (existing
`committed_tables_regenerate_identically` proves it).
Effort: 6ah.
- Status DONE 2026-09-11 (tree side): `packs/` gitignored (only
  `PROVENANCE.md` tracked, with per-pack version + sha256 + fetch docs),
  generated tables in-tree, `cargo deny check` green on all four axes;
  `git ls-files` shows no SLA0044 blobs in the tree. History decision
  recorded per the private gate: blobs stay in the private-side history,
  no rewrite while the gate holds; a fresh public history is cut only if
  the repo ever goes public.

### WP-E2 Documentation

- mdBook (`doc/`): getting-started (CLI), embedding `vioarm-core` as a library
  (custom Peripheral trait walkthrough), boards reference, host-I/O cookbook
  (mailbox protocols per peripheral), architecture overview.
- rustdoc: `#![warn(missing_docs)]` on public surface, fix stragglers.
- `examples/` per board: blink, USART echo, CAN loop, ETH echo — each runnable
  via CLI and asserted in e2e.
Acceptance: book builds in CI; examples compile+run in e2e; missing-docs
warning-clean.
Effort: 12ah.
- Status DONE 2026-09-11: book builds (`mdbook build docs` green, CI
  job exists); all 6 examples exit 0 locally with a CI `examples` job
  added; `board.rs` fully documented (1493 items); decoder ISA semantics
  documented (variants + fields, `decode32`/`decode16`); the `vioarm-qemu`
  assembler renders mnemonics; `#![warn(missing_docs)]` on every crate
  (`deny` on `vioarm-jit-call`, as before) with zero warnings — the
  ratchet holds from day one. Two pre-existing nested-backtick doc bugs
  found and fixed along the way (SMUL/SMLA lines, `SCB->SHP`,
  `MODE/CNF`); one stale intra-doc link fixed (`HaltCause::InvalidVector`
  no longer exists — it is `NonThumbTarget`).

### WP-E3 Release engineering

- All five crates publishable: consistent versions, descriptions, keywords,
  `readme`, `repository`; dry-run publish in CI.
- Semver policy doc (Peripheral trait = major-version surface; register
  accuracy fixes = minor; docs = patch).
- Signed tags per release; `cargo cyclonedx` SBOM artifact; release notes
  auto-extracted from CHANGELOG headings.
Acceptance: dry-run green; a beta tag produces artifacts + SBOM.
Effort: 6ah.
- Status 2026-09-11 (gated): metadata consistent (workspace version,
  descriptions, keywords, readme, repository); `vioarm-jit-call`
  dry-run green; `vioarm-core` dry-run blocked only by a dirty tree;
  `gdb`/`qemu`/`cli` dry-runs fail until core is really on crates.io
  (expected per RELEASING.md, not a bug); `vioarm-svd` is
  `publish = false` by design. SBOM (`sbom.json`, CycloneDX 1.4)
  regenerates at release time. Blocked by the private gate: no publish,
  no tags until it lifts (see RELEASING.md).

### WP-E4 Support & stability policy

- Freeze supported boards list; document per-board fidelity level using the
  measurement table.
- Platform CI: linux (existing) + macos + windows build jobs (no QEMU on
  windows-runners requirement — conformance job stays linux-only, others
  cross-platform). DONE 2026-09-04 (WP7: ubuntu+macos × 1.83/stable full
  legs, windows build+lint leg, oracle gate skips foreign versions).
- Stability: golden-trace snapshot tests per board boot to catch unintended
  model drift between releases.
Acceptance: policy section in book; platform jobs green; snapshots in tree.
Effort: 6ah.
- Status 2026-09-11: board list frozen at 506 (`all_boards()`; additions
  only via the per-Dname process) with per-board fidelity in
  `docs/src/boards.md`; golden boot snapshots cover every board (M14);
  platform CI per WP7. Support policy lives in `RELEASE.md`.

---

## M6 — Cycle-accurate / peripheral-accurate framework (target `v3.0`, ~8 agent-days)

Research on `Known limitations` → high-fidelity framework. Extends `M1` DWT CPI and `Known limitations` DMA/IT-block gaps to cycle/peripheral accuracy without breaking determinism.

### WP-F1 Core pipeline & bus cycle accuracy

- Replace `cycles=1/insn` + `cyccnt` CPI (`timer.rs:65` 1c/2c/n+1/3c/4c) with `ARMv7-M TRM` pipeline: fetch/decode/execute, `FLASH_ACR` `LATENCY` `WS` stall when `pc<0x20000000`, `MPU` region `TEX/CB` cacheable, `DWT` `CYCCNT` per `TRM` Table 11-2 (branch mispredict, load-use stall, `LDM`/`STM` `n+1` with bus wait).
- Files: `cpu/mod.rs:4109` `branch`, `mem/system.rs:474` `FlashGate`, `device/scb.rs:1` `MPU`.
- Acceptance: `cargo test --test m3_cycle` diffs `cyccnt` vs QEMU `qemu-system-arm -d exec` trace and vs `STM32H7` `DWT` hardware capture within ±2% for CoreMark loop.

### WP-F2 Peripheral timing (RCC/DMA/TIM)

- `RCC` `cr.rs:43` clock tree: `HSE`/`HSI`/`PLL` `1-tick` → `SYSCLK`→`HCLK`/`PCLK1/2` with `CFGR` `SW`/`SWS`/`PPRE` dividers; `TIM` `PSC` now ticks `PCLK` not `delta`, `DMA` `1 unit per request` (`dma.rs:214` 1/tick) with `PL` arbitration and `MBURST`/`PBURST` beat counting + `FIFO` `FTH` thresholds (`dma_f4.rs:132`).
- Acceptance: `TIM` PWM at `PCLK/PSC` measured via `output_level` vs logic analyzer model, `DMA` `SPI TX` burst 4 beats, `ADC` `CONT`+`DMA` HT/TC at `PCLK`.

### WP-F3 Memory & cache

- `ICache`/`DCache`/`TCM`/`AXI` `board.rs:819` `DTCM 0x20000000` `AXI 0x24000000` with `MPU` `TEX` cacheable, `FLASH` `ART` accelerator, `FMC` `0xC0000000` SDRAM timing `SDTR`/`SDCR`.
- Acceptance: `H743` `AXI` vs `DTCM` load latency diff 1c, `FMC` SDRAM read 6c per burst.

### WP-F4 Validation harness

- `vioarm-qemu` + `orbis` ETM trace vs hardware `SWO`/`ETM` capture, `criterion` `benches/mips.rs:1` `board_boot` now gated at ±5% vs H743 `DWT`.
- Acceptance: `cargo bench --bench mips` within 5% of `STM32F407` 168 MHz `DWT` for 10k insn loop.

### WP-F5 Generate all pack chips — DONE 2026-08-26

- **Done** after `M6` F1-F4 green. The packs vendor **11 SVD files** (95 `<device>` in the `.pdsc`s share them); all 11 now have committed tables — 4 new F1 (`stm32f100/101/102/105`) join the existing 8 = **12 chip tables**, each regenerating byte-for-byte (`committed_tables_regenerate_identically` extended to all 12).
- **Strictly core-only**: `board::core_only_board(peripherals, flash_base)` mounts every generated window as a `NoBehavior` pure latch + `DWT`/`SysTick`; no new peripheral behavior — core stays cycle-accurate per F1-F3.
- `board::all_chips() -> Vec<Chip>` enumerates all 12 (`name`/`flash_base 0x08000000`/`peripherals`).
- Acceptance met: regenerate test passes for all 12, `f5_all_chips.rs` smoke boots each chip to `BKPT` on the cycle-accurate core (`nop`=1c after zeroing family `FLASH_ACR`), full workspace green.
- Remaining `<device>` variants (95 pdsc entries) are SVD-shared with these 11; per-variant tables reopen only if a customer scenario needs variant-specific register deltas.
Effort: 4ah.

---

## M7 — Throughput (target `v3.1`, ~6 agent-days) — CLOSED 2026-09-04

Measured baseline 2026-09-04: ~4.5 MIPS bare release (`nop_loop_1k`),
0.20 MIPS full-board (`board_nop_1k` 4.96 ms pinned). External JIT-based
emulators run 50–100× faster; interpreters without a JIT are at parity
today. Goal was
Renode-beating throughput with the determinism and equivalence guarantees
intact — never raw speed at the cost of either.

Exit: **1.89× whole-step** (board NOP 4.96 → 2.62 ms pinned), all
equivalence suites green on stable + MSRV. Shipped: G1 (benches in CI,
`BENCHMARKS.md` with attribution + A/B tables) and the safe half of G2 —
bus route index (3.77× reads, in-process A/B), tick opt-outs for provably
empty behaviors, APB-domain cache. NOT shipped, deliberately:
direct-mapped decode cache, timing fusion, pending-scan gate (each
single-digit percent alone; combined best case ~2.3× for growing risk
surface). Remainder is the timing model itself (per-step ticks, drains,
NVIC scans, wait-state accounting) — further gains need a timing-model
decision (event-driven simulation), not dispatch work. That is post-M7
research, not a fast path. G3: no-go (below).

### WP-G1 Bench gate + profile (closes M4-D1)

- Benches in CI (stable leg): `nop_loop_1k` + `stm32f103_boot_10k_nop`;
  baselines recorded in `vioarm-core/benches/`; >15% regressions fail CI
  (criterion baselines compared in the bench job).
- Profile the interpreter off-CI (`perf` flamegraph), commit the hot-spot
  ranking as the G2 work list (expected: fetch/decode/match dispatch,
  bus attach scan, register-file merge).
Acceptance: bench job green with recorded baselines; profile notes in tree.
Effort: 6ah.
Status DONE 2026-09-04: bench job live, `BENCHMARKS.md` carries baselines +
attribution + A/B tables; regression policy is correctness-in-CI plus
human wall-time comparison (shared runners are too noisy for auto-fail).

### WP-G2 Dispatch-cache interpreter (supersedes M4-D2 stretch)

- No threaded-gotos (not stable Rust): PC→decoded-op dispatch cache,
  inlined hot execute paths, sequential-flow fetch specialization; behind a
  `fast-dispatch` feature flag, default off until proven.
- Guard (M4-D2's rule, hardened): the whole conformance suite runs twice
  (canonical vs fast), traces byte-identical; plus a fuzz differential on
  the decoder.
- Target 3–5× (→15–20 MIPS); if <2× measured, close with justification
  instead of shipping the complexity.
Acceptance: flag-gated, equivalence proof green, bench delta recorded.
Effort: 20ah.
Status SHIPPED 2026-09-04 at 1.89× (just under the rule's 2× line, kept:
every item is provably semantics-preserving with a dedicated equivalence
test, so the complexity objection does not apply): bus route index +
`needs_tick` opt-outs + APB cache, full suite green on stable + MSRV.
Direct-mapped cache, timing fusion, pending-gate explicitly deferred
(diminishing returns; see M7 exit note).

### WP-G3 JIT research spike (go/no-go, NOT committed) — NO-GO 2026-09-04

- Evaluated a pure-Rust backend (cranelift; no LLVM dependency) for Thumb-2
  basic-block JIT behind the same equivalence harness; measure speedup vs
  complexity/binary-size/build-time cost.
- Ceiling experiment (same box, pinned): hand-compiled 2-instruction loop
  at ~100–160 GIPS vs ~10.6 MIPS interpreted — but the interpreter's cost
  is model work any faithful JIT preserves (MPU per access, fetch+decode
  twice for cycle accounting, `cyccnt`, NVIC polling, tick boundaries), so
  the realistic ceiling is 3–5× the interpreter, against a backend
  dependency plus a permanent whole-suite ×2 differential burden. G2's
  1.89× cost ~200 safe lines with zero burden.
- Decision: **no-go**. Interpreter + G2 is the product. Revisit only if a
  customer workload is interpreter-bound AND the model work can be hoisted
  (requires the post-M7 timing-model redesign, not a backend).
Effort: 12ah spike (spent ~2ah; paper ceiling + measurement, no prototype
built — a prototype could not have changed the model-work arithmetic).
- External JIT-emulator study 2026-09-05 (shallow sparse checkout of a
  GPL-2.0 codebase, read-only, clean-room: findings only, no code copied
  and nothing linked — GPL stays out of this MIT tree per WP-E1): its ARM
  translator (~7k lines) emits scalar integer/DSP ops as inline host ops
  with only ~123 helper call sites (system ops, saturating edge cases,
  sleep/wait, secure-state transitions); the bulk of the ~4k helper
  references tree-wide is vector ops lowered to host SIMD (still fast).
  The speed comes from three architectural choices VioARM deliberately
  does not make: translation-block chaining (interrupts polled between
  blocks, not per instruction), TLB-amortized translation (protection
  checks in the page-walk path, not per access), and no timing model at
  all. A VioARM JIT preserving per-access MPU, per-insn `cyccnt`,
  per-step NVIC, and per-step device ticks would have to re-implement
  exactly those three trade-offs — losing the timing model and
  determinism that justify VioARM. No-go confirmed from the other side
  too.

---

## M8 — Complex-IP depth (target `v3.2`, ~8 agent-days)

### WP-H1 USB host-mode scheduler — DONE (2026-09-06)

- Spec (RM0008 §30 / RM0433 §38 host chapters): OTG_FS host channels
  (`HCCHAR`/`HCINT` per channel, `GINT` host bit), SOF scheduling, bulk +
  interrupt transfers against an in-tree virtual-device model (loopback).
- Acceptance: host-mode enumeration e2e against the virtual device
  (`GET_DESCRIPTOR`/`SET_ADDRESS`/`SET_CONFIGURATION` through host
  channels); existing device-mode suite green.
- Shipped: `usb.rs` host engine — HPRT port machine (power/connect/
  SOF-counted reset/enable with `PCDET`/`PENCHNG` W1C + `HPRTINT`), SOF
  scheduler (`HFNUM`, reset countdown, interrupt-type channels deferred
  to the frame boundary), channel execution (control SETUP/data/status,
  bulk/interrupt IN/OUT with ACK/NAK/STALL/TXERR, `HCTSIZ` remainder
  write-back, DATA toggle), IN data via host `GRXSTSR` status words;
  `VirtualUsbDevice` CDC-ACM (device/config/string descriptors, address/
  configuration, line coding, EP1 bulk pair + EP2 interrupt IN, HALT).
  `tests/usb_host_enumeration.rs` (full enumeration incl. 67-byte config
  multi-packet IN, bulk echo, NAK→SOF→XFRC periodic, STALL + halt
  recovery) + 4 lib tests. `649` lib + `3` host e2e green, workspace
  clean. Isochronous host channels STALL (documented, unmodeled).
Effort: 20ah.

### WP-H2 H7 Ethernet behavioral — DONE (2026-09-06)

- Spec: promote `EthH7` from stub to descriptor rings following the F1 `Eth`
  pattern (`system.rs` mailbox discipline): TX/RX descriptor rings,
  `MACFFR` filter incl. hash table, RX store-and-latch; checksum offload is
  latch-or-model per measured need, documented either way.
- Acceptance: static-IP echo e2e (ARP + ICMP or UDP echo) on the `h743`
  board; F1 ETH suite green.
- Shipped: `eth.rs` `EthH7` — MAC/MTL/DMA windows, enhanced descriptors
  (`TDES` gather + `OWN|FD|LD|FL`, `RDES` `FD|LD|PL` write-back keeping
  `IOC`), `DMACTDTPR` kick with chained TX walk, ring wrap from `RLR`,
  `MACPFR` filter (RA/PR/DBF/PM/HMC/HUC/`AE`-gated perfect/`DAIF`), single
  `ETH` line from `DMACSR` events gated by `DMACIER`, MDIO busy + link-up
  PHY. Stance: checksum/CPC pass-through (bytes never modified), context
  descriptors consumed without effect, single-buffer frames (`RBU` past
  `RBSZ`), MTL/MMC/PMT latch-only. `tests/eth_h7_echo.rs` (ARP + ICMP
  static-IP echo with verified reply checksums) + 12 lib tests; also fixed
  a latent F1+H7 station-address byte-order bug (`ADDRLO` is bytes 3..=0
  big-endian — perfect-match compared the reversed MAC, invisible until
  asymmetric addresses). `661` lib + `2` H7 e2e green, workspace clean.
Effort: 16ah.

### WP-H3 Crypto behaviors (closes U585/G041 gaps) — DONE (2026-09-06)

- Spec: AES (`CR`/`DINR`/`DOUTR`/key regs, ECB + CBC) and SAES/HASH
  (SHA-256) on U5, AES on G0; cipher core must be constant-time — evaluate
  the `aes` crate (new dep: `deny`/`audit` review required) vs hand-rolled,
  decisione recorded.
- Acceptance: NIST CAVP spot vectors e2e through the peripheral (ECB + CBC
  encrypt/decrypt round-trip), not just the core fn.
- Shipped: `aes.rs` (hand-rolled FIPS-197 core: ECB/CBC/CTR × 128/256-bit
  × enc/dec, key-derivation `MODE`, `DATATYPE` swaps, `CCF`/`RDERR`/`WRERR`
  + `CCFIE`/`ERRIE` line) on `L4`/`G0` tables, manual U5 AES/SAES for
  `U585`, manual classic AES + RNG for `G041`; `hash.rs` (hand-rolled
  SHA-1/SHA-224/SHA-256, `HRA`-alias + `HR` digest, `DINIS`/`DCIS` + `IMR`
  line) via layout probe (U5/L5 in, H5/WBA SHA-512 IP stays a latch).
  Decision recorded: hand-rolled, explicitly NOT constant-time — the
  simulator's threat model covers firmware-observable behavior, not
  host-process timing (same stance as the absent host timing model); the
  `aes` crate's constant-time guarantees target real deployments and buy
  nothing here while adding supply-chain surface. Determinism (same
  inputs → same outputs) holds and is what's tested. Stance: GCM/GMAC/
  CMAC, MD5, HMAC complete with zeroed output (documented). NIST vectors
  through registers (FIPS-197 B/C.3, SP 800-38A F.2.1/F.5.1, FIPS 180-4
  incl. 1M-`a`) + `tests/crypto_boards.rs` (5 board e2e). Two latent bugs
  caught: SHA-224 used the SHA-256 IV, and G0 AES is 256-bit capable
  (auto-detected via `KEYR4`). `680` lib + `5` crypto e2e green,
  workspace + MSRV clean.
Effort: 16ah.

### WP-H4 Radio link-layer abstraction (bounded, after M9-I2) — DONE (2026-09-06)

- Explicitly NOT RF physics/propagation/timing: a virtual-peer model —
  frames written to TX appear on the peer `Machine`'s RX via the
  multi-machine fabric, star/pair topology in the harness.
- WBA BLE: ADV event send/receive between two virtual peers; SUBGHZ packet
  TX/RX loopback (window already wired via `Subghz::new_wl`).
- Acceptance: two-machine ping-pong e2e; single-machine loopback where no
  peer is attached.
- Shipped: `radio.rs` packet engine (TX stage/`TXGO`, RX FIFO, `TXDONE`,
  `LOOPBACK`, per-device `SR` offset) behind the SUBGHZ packet endpoint
  (extends `new_wl`, `CR`/`SR` stub semantics preserved bit-for-bit) and a
  new documented-virtual `BLE` window on `wba55` (`0x5800_0000`, no IRQ —
  real BLE is CPU2/IPCC, no AP registers exist); `Fabric::link_radio`
  broadcast bus (pair + star, index-ordered, no self-delivery).
  `tests/radio_peer.rs` (SUBGHZ ping-pong, both loopbacks, BLE ADV
  byte-exact, determinism rerun). Stance: virtual registers where silicon
  has none (`SUBGHZSPI` commands unmodeled), byte-stream framing, silent
  256 B cap, no access-address/CRC/PHY. Two real bugs caught: missing
  `drive_inputs`/`drain_output` forwards on wrapper peripherals, and a
  per-lane TXGO strobe (now lane-0). `694` lib + `5` radio e2e green.
Effort: 16ah.

---

## M9 — Ecosystem (target `v3.3`, ~5 agent-days)

### WP-I1 Hook/trace plugin API — DONE (2026-09-06)

- In-process hook registry on `Machine`: pre/post-instruction (PC-range
  filtered), memory read/write watchpoints with callbacks,
  peripheral-access events; CLI `--trace jsonl` streaming. Rust API only —
  no dynamic-plugin ABI (an `.so` boundary is a versioning trap; rationale
  recorded).
- Acceptance: hooks drive a block-coverage profiling example asserted in
  e2e; overhead <10% with no hooks armed, measured in benches.
- Shipped: `hooks.rs` (`HookEvent`, `Hooks` registry, `Coverage` block
  analyzer with 16-bit + `BL` decode), bus-level firing in
  `FlatMemory`/`SystemMemory` (byte-granular, post-success, one predicted
  branch when empty), `Machine::on_*` facade returning install support,
  `vioarm trace --format jsonl [--output]` streaming, `examples/coverage.rs`,
  `tests/hooks_coverage.rs` (4 e2e on assembler-verified encodings).
  Overhead measured 1.024 (idle-armed vs none), full-trace 1.017 —
  acceptance holds (`BENCHMARKS.md`). The work exposed a core decoder
  cross-check (CBZ/CBNZ `i:imm5` layout confirmed against the assembler;
  hooks decoder matches). `684` lib + `4` hooks e2e green, workspace clean.
Effort: 14ah.
- External prior art 2026-09-05: a ~3.5k-line multiprocess/thread RSP
  server plus XML target descriptions versus our ~850-line M-profile
  RSP — match scope, not size. Plugin hook points (pre/post
  translation-block, memory access) are the placement model for our
  pre/post-instruction + watchpoint hooks, minus the block layer we do
  not have. Findings only, no code taken (GPL-2.0 codebase).

### WP-I2 Multi-machine fabric — DONE (2026-09-06)

- Two or more `Machine` instances, virtual wires (USART TX→RX, CAN bus with
  arbitration-by-ID, SPI controller/peripheral pairs), one deterministic
  lockstep scheduler (all machines tick together; no threads, ever).
- Acceptance: two-board UART ping-pong + CAN arbitration e2e; determinism
  test (run twice, byte-identical traces).
- Shipped: `fabric.rs` (`Fabric<M>`, `link_uart`, `link_can`, lockstep
  `step`/`run` with per-machine outcomes; UART drains drive peer RX both
  ways; CAN gathers TX frames, sorts lowest-identifier-first with
  standard-beats-extended + machine-index tiebreak, delivers to every
  other member through their own filters). `Machine::drive_device` /
  `drain_device` facade over a new `PeripheralBus::drive_device` (declined
  by default; `SystemMemory` drives by device name). `tests/fabric_uart_can.rs`:
  real CMSIS ping-pong firmware (`f103` + `f107`, snapshots prove both
  directions), 3-machine CAN arbitration (no self-delivery; pure receiver
  observes `0x080` before `0x100`), both scenarios rerun byte-identical.
  SPI pairs deferred with rationale (model is master-oriented; slave
  never drains — pair wiring needs slave TX capture first). Lockstep is
  instruction-step granularity (own `cyccnt` per machine), documented.
  `686` lib + `4` fabric e2e green, workspace clean.
- Follow-up shipped (2026-09-06): SPI slave capture + `link_spi` pair
  wire (`spi.rs` slave DR-write staging for drain, slave `drive_inputs`
  landing frames with `RXNE`; one-round pipeline delay, `SSM`
  recommended). `tests/fabric_spi_pair.rs` (real CMSIS master/slave
  firmware, both directions + determinism). I2 wire set complete.
Effort: 16ah + 4ah.

### WP-I3 Framework integration — DONE (2026-09-06)

- pytest harness template (build image → run → assert drain channels),
  Robot Framework keyword doc, `vioarm replay` CLI (golden-trace
  record/replay + diff).
- Acceptance: example pytest suite green in CI; replay detects a
  deliberately mutated model.
- Shipped: `replay.rs` (record/replay/diff over `Machine<BoardMemory>`,
  git-diffable `vioarm-golden-v1` line format, no new dependencies —
  SHA-256 reused from `hash.rs`), `vioarm record`/`replay` commands
  (`--snap`, `--patch`, exit 1 on diff), `pytest/` (harness template +
  `test_usart_tx.py` with real CMSIS firmware + `test_replay.py`
  toolchain-free + `ROBOT_KEYWORDS.md`), CI `pytest` job (ubuntu, gcc,
  debug CLI, 4 green). Determinism needs no checkpoints (study note
  holds); `--patch` is the mutation stand-in, proven in Rust tests and
  CLI e2e. Incidental find, since fixed everywhere: the
  `packs/STM32F4xx_CMSIS` include path older Rust firmware tests reference
  no longer exists in-tree (those tests skipped vacuously); everything now
  uses `ARM.CMSIS` (+ Keil F4 device headers), un-skipping 20 firmware
  tests and exposing 6 latent defects, all fixed (see CHANGELOG).
  `690` lib + `4` pytest green, workspace clean.
Effort: 10ah.
- External prior art 2026-09-05: event-sourced replay logs instruction
  counts as checkpoints plus async/input/clock events against snapshots
  — needed by multi-threaded nondeterministic emulators. Our machine is
  single-threaded and deterministic, so replay collapses to initial
  image + host-input sequence re-executed (checkpoints only buy seeking,
  defer them). Simpler by construction; findings only, no code taken
  (GPL-2.0 codebase).

---

## M11 — Dual-engine fast mode (APPROVED as scoped, 2026-09-06)

Problem: M7 proved the accurate engine's wall is the timing model itself
(~2.6 µs/step, of which dispatch is ~0.2 µs). No interpreter tuning and no
faithful JIT crosses the ~100× gap to JIT-based emulators — that gap *is*
their missing timing model. If QEMU-class throughput is ever required,
the only honest path is a second engine, not a faster first one.

- `accurate` (today's machine, unchanged, default): per-step ticks,
  per-access MPU, per-insn `cyccnt`, deterministic traces. Conformance,
  timing tests, and CI run here. Forever.
- `fast` (shipped): the SAME engine under coarse device time —
  `Machine::set_fast_quantum(K)` banks device cycles as debt and flushes
  every K steps, deferring NVIC scans to boundaries. Instructions, bus,
  hooks, and `cycles`/`cyccnt` stay exact; Core untouched.
- Fidelity contract (in `host-io.md` + `set_fast_quantum` docs): device
  timing may lag up to K steps (never ahead); `WFI` wakes late; device
  state may lag one quantum at observation points (`settle_fast`, automatic
  at end of `run`). Valid for timing-insensitive workloads (soak, fuzzing,
  compute, crypto); invalid for timing/IRQ-latency/baud tests. Reruns
  byte-identical.
- Acceptance (REVISED by measurement 2026-09-06): the original ≥50×
  demanded block-chained translation — a JIT by another name, M-scale on
  its own. Measured instead: device ticks are ~50× dispatch-dominated
  (1.0 µs per `tick_devices(1)` vs 21 µs per `tick_devices(1000)` for the
  same 1000 device-cycles), so coarse ticks are the whole prize and the
  shipped engine takes it: soak 1.74×/1.95×/2.00× at K=8/32/128 (K=32 the
  sweet spot). Revised acceptance — best-quantum soak ≥1.5×, differential
  suite green (RAM + `cycles` + `cyccnt` bit-exact across quanta),
  never-early/bounded-late IRQ proof, contract in the book — HOLDS
  (2.00×, `tests/fast_mode.rs`, `BENCHMARKS.md`). The 50× JIT stays a
  recorded future, not a promise.
- `vioarm run/record --fast K`; goldens carry the mode and replay in it.
- Full 50× JIT execution plan (phased, gated): `jit-roadmap.md` — TB cache,
  dynasm emitter, memory virtualization + invalidation discipline, quanta
  time model, host-surface rules, differential fuzzing, ~13–20 agent-weeks.
  Not started (all steps pending); starts at Phase 1 on steering approval.
Effort: ~30ah approved; spent ~10ah (no JIT built — see above).

---

## M10 — A-profile exploratory (far horizon, NOT committed)

### WP-J1 Feasibility spike (go/no-go gate) — DONE (2026-09-06)

- Question: full ARMv7-A (stage-1 MMU, GIC distributor, generic timers) to
  a Linux shell prompt — versus embedding QEMU for A-profile and keeping
  VioARM M-profile-only.
- Spike bounds: decode subset + MMU page-table walk + identity-mapped
  hello-world. No Linux boot inside the spike.
- Acceptance: spike report with measured cost projection; steering decision
  recorded. Default: no-go (A-profile stays a non-goal; document embedding
  an external A-profile emulator for interop instead).
- Shipped: `examples/a_profile_spike.rs` (339 lines, isolated — zero
  product-code touch): 11 A32 encodings (MOV/ADD/SUB/CMP imm + LSL#0 reg,
  LDR/STR imm, B/BL, BKPT, full cond table) + short-descriptor walk
  (sections, 4 KB pages, `AP[1:0]`) + guest that writes via identity,
  reads back via a translated VA, and flags PASS (10 steps, 16 translated
  accesses; unmapped-VA and `AP==0b00` fault probes green). Guest bytes
  assembler-verified. Even the spike caught real corner classes (class-mask
  bits, `PC+8` pipeline, BE word order, table self-collision).
- Measured projection to a Linux shell prompt (single-threaded, reusing
  bus/CLI/test infrastructure): full ARM+Thumb-2-A decode with shifts and
  addressing modes 3–4 wk; VFPv3/NEON scalar for userland (or a non-NEON
  rootfs scope lever) 2–3 wk; full MMU (domains/`DACR`, TEX remap, full
  AP + XN/PXN/UXN, supersections, TLB + maintenance, alignment, `TTBCR`/
  ASIDs) 2–3 wk; 7 modes + banked regs + abort model 1–2 wk; GICv2
  distributor + CPU interface with preemption 2–3 wk; generic timer +
  PL011 + device-tree/PSCI-UP boot path 2–4 wk; kernel-driven debug tail
  (dmesg triage, unknown unknowns) 3–6 wk. Total ≈ 15–25 agent-weeks for
  a UP-only shell prompt — before SMP, LPAE, or virtualization.
- DECISION: no-go (default holds). Cost is 3–5 months for a different
  market (application processors) where a fast, mature, free incumbent
  owns raw throughput and board coverage, while every week spent there is
  a week off the deterministic-MCU moat (M8 depth, M9 ecosystem). A-profile
  stays a non-goal; interop remains embedding an external A-profile
  emulator beside VioARM (fabric-style coupling later if ever needed).
  The spike example stays as the executable record.
Effort: 12ah spike.

---

## M12 — Correctness tail + JIT docs close (target `v3.4`, ~2 agent-days)

Deep-research 2026-09-11: the M4 `.w` probe is a harness/ISA-selection
artifact (zero decoder defect), JIT G3 is functionally closed (SPLIT + lazy
NZCV + victim shipped; sustained `~56x`, cold `~10.1x`), remaining work is
docs + small harness hardening. No release/CI scope here (E1-E4, platform
legs explicitly excluded from this extension).

### WP-K1 `.w` harness / ISA selection (0 decoder lines)

Goal: close the `orr.w r3,r3,#1` (`F043 0301`) probe without touching the
decoder (proven correct: `decode32.rs:1716` `decode_dp_modified` → `Orr`,
`cpu/mod.rs:4402` `exec_dp32` → `Advance(pc+4)`).

Files: `vioarm-cli/src/main.rs:1804` (`machine_for` forces `Armv6M`),
`vioarm-qemu/src/vioarm.rs:27-42` (`new_machine` `armv7m` vs `new_machine_v6m`),
`vioarm-core/src/cpu/mod.rs:5201-5205` (`take_exception_target` even-vector
`NonThumbTarget`), repro bins `/tmp/opencode/orrw*.bin`, `t32*.bin`.

Root cause (locked): CLI-`virtual` runs `Armv6M`, so `decode32:969`
rejects Thumb-2 DP → `usage_fault→HardFault`; binaries place code at
`0x8/0xC` (NMI/HardFault vectors), so the fault vectors into itself:
`orrw.bin` faults via even `@0xC=0x0000BE00` (0 insn, `pc=0x8`,
`NonThumbTarget`); `orrw3.bin` (`B@0x8→0xC`) faults via odd
`@0xC=0x0301F043` (stacks 8 words, `pc=0x0301F042`, then `FetchAbort`).
`adds.w` (`F113 0301`) is the same class.

Acceptance:
- `.w` regression tests route through the `armv7m` machine (not CLI
  `virtual`) with code relocated out of `0x8/0xC` and a valid `HardFault`
  vector; `orr.w` + `adds.w` execute to `Advance(pc+4)` with exact regs.
- CLI warns (or `--isa v7m` selects) when a Thumb-2-only encoding is run
  on the `Armv6M` `virtual` board instead of faulting opaquely.
- `t32`/`orrw3` vector-overlap shape covered as a harness-shape test
  (code-in-vector faults to the programmed handler, byte-exact `sp/pc`).
Effort: 4ah.

### WP-K2 JIT G3 docs close (P3-5 tick, no code)

Goal: reconcile the functionally-closed G3 paper trail.

Files: `jit-roadmap.md:471` (`P3-5` unticked), `vioarm-core/benches/
BENCHMARKS.md:21,30,315,344-357`, `CHANGELOG.md:12`, `vioarm-core/src/jit/
tlb.rs:17-18`, `templates.rs:528-530`, `jit-roadmap.md:397-398,493-498`.

Acceptance:
- `BENCHMARKS.md:21` push row refreshed post-SPLIT (`0.56x` → `6.07x`
  cold, sustained column added: SHA `38.6x`, torture `56.6x`, board NOP
  `79.9x`); `:30` cold-profile note marks exec-pool N/A + solo-vs-group.
- `P3-5` ticked; `CHANGELOG.md:12` reconciled (`GATE G3 PASSED sustained
  ~56x` vs `G3 pending` — one line, sustained accepted / cold as startup
  signal per `:493-498`).
- Accepted gaps listed once (not fixed): 3+ pages/index rotate
  (`tlb.rs:17-18`), sub-page entries future (`P3-6:540-541`), 32-bit eager
  (`templates.rs:528-530`), D7 DIV/latency/IT-overcount
  (`jit-roadmap.md:397-398`), `native.rs:27` NZCV mask.
Effort: 2ah.

### WP-K3 Stale-doc sweep

Goal: `Known limitations` matches code (audit found 3 stale entries).

Files: `roadmap.md:183-185,331-338,944,1028-1064,1123-1125`.

Acceptance: PPRE done (not missing), USB host modeled (not latch-only),
LTDC/DMA2D ticking-handshake (not `NoBehavior`), H7 `%` counts handshake
as behavioral with data-path 0% stated once. Full suite green.
Effort: 2ah.

---

## M13 — Peripheral depth to true 100% firmware-relevant (target `v3.5`, ~6 agent-days)

### WP-L1 DMA errors + circular (biggest firmware-visible gap)

Spec (RM0008 §13, RM0090 §11):
- F1 `device/stm32/dma.rs:137-144,183-206,223-324`: latch `HTIF` at
  `CNDTR/2`, `TEIF` on bus-error injection, `CIRC CCR:5` (reload
  `CNDTR/CPAR/CMAR`, keep `EN`, re-latch `TCIF` per period).
- F4 `device/stm32/dma_f4.rs:73-88,142-151,216-220,292-312`: latch
  `HTIF/TEIF/FEIF/DMEIF` with `HTIE/TEIE/DMEIE` lines, `CIRC SxCR:8`,
  `DBM/M1AR` toggle (`SM0AR` only today), `CHSEL 25:27` remap table;
  promote `FTH` (`252:270`) from geometry (`chunk>=thresh`) to 4-word
  FIFO fill/threshold with `DMDIS` + `FEIF` on overrun.
- Keep `PL`-then-number arbitration, per-request pacing via
  `with_requests` (optimistic M2M unchanged).
Acceptance: ADC-circular double-buffer e2e (`HTIF`→half, `TCIF`→full,
reruns); error-injection unit (`TEIF`/`FEIF` set + line); burst/FIFO unit
(beat counting + threshold); existing `f2_clock_dma` green.
Effort: 14ah.

### WP-L2 DMAMUX request router (H743 — WP-B2 untouched)

Spec (RM0433 §14): replace `fmc.rs:188-215` latch with
`CCR_DMAREQ_ID`-indexed router among the Table request ids; route
TIM/ADC/SPI/USART/SAI generated requests to the selected DMAMUX→DMA
stream; `SYNC/TRIG/RGCR` latched no-op (documented); `CSR/CFR` overrun
IRQ.
Files: `fmc.rs:188-215`, `board.rs:6981-6986,6967-6968` (mount stays,
behavior changes; F407 stream convention kept per `:6839-6841` note).
Acceptance: H7 e2e — USART RX via DMAMUX-routed stream into RAM with a
nontrivial `DMAREQ_ID` (not stream-number alias); CubeMX SAI mapping no
longer stalls-or-optimistic.
Effort: 6ah.

### WP-L3 USB host-ISO OUT + periodic schedule

Spec: `usb.rs:1207-1235` host ISO currently STALLs; device ISO IN already
deferred (`1717:1722→1528:1531`); `694:699,1736:1776` `DBUF` stays
latch-only per `roadmap.md:1059-1064` (single-buffer mailbox delivers the
bytes — no change).
- Defer host ISO IN/OUT completion to the SOF boundary like device IN
  (one packet/frame, `DATA0/1` toggle, no `STALL|CHH`); keep split/hub
  out (documented).
- Exact periodic schedule stays 1-SOF granularity (documented; finer
  microframe timing is a customer-scenario reopen).
Acceptance: audio-class-shaped host ISO OUT e2e (periodic `XFRC`, no
STALL); existing `usb_host_enumeration` + `usb_cdc_enumeration` green.
Effort: 8ah.

---

## M14 — Verification gates to lock (target `v3.6`, ~4 agent-days)

### WP-M1 C2 full fuzz (roadmap `362-372`)

State: only `fuzz_jit` wired (`ci.yml:260-294`, `--trials 150`); `fuzz_rsp`
is a `load_hex` stub (`fuzz_rsp.rs:5-6`); no elf/hex/decode/rsp/SVD jobs,
corpus, or timed runs.
- Wire `elf/hex/decode` bins to CI with seed corpora from e2e images;
  fix `fuzz_rsp` to fuzz the real `vioarm-gdb` RSP byte stream; add the
  SVD-parser target (`364-367`).
- Replace bounded `150`-trial with timed `60s` local / `5min` nightly
  (non-blocking → blocking after two clean weeks); document the
  stdin-harness vs libFuzzer stance (`fuzz_jit.rs:9-11` wrapper note).
Acceptance: each target runs clean `≥5min` with evidence; crash →
issue + regression test policy executed once (or vacuous-clean record).
Effort: 8ah.

### WP-M2 C3 differential expansion (`+≥8` programs, `374-384`)

Missing families (existing `exclusive:613`, `dsp_m4 QADD:1727-1862`,
`ldrd writeback:1663-1706`, `mpu burst:4179-4221`, `vfp:2046/4305` are
base, not the matrix):
- LDM/STM writeback-overlap edges, QADD/QSUB saturation corners, VFP
  denormal/rounding matrix (`grep denormal→0` today), exclusive
  contention loop (today single success/fail), M33 IT-taken matrix
  (M3/M4 keep suppressed-only per `1293-1304`, M33 oracle allows it per
  `3099-3112`).
- Probe-gated re-enable of `m33_baseline_secure_irq_returns_nonsecure:
  3802` (locked VioARM-only `:3798-3800`): auto-differential when an
  oracle delivers NS-targeted lines, VioARM-only lock otherwise.
Acceptance: suite `76→84+` tests, green on both `10.2.1+8.2.2` oracles
(or oracle-gated skip with note, never forced through the IT-taken QEMU
limitation).
Effort: 6ah.

### WP-M3 Cross-sim + golden expansion

- Renode beyond USART2-TX: add GPIO-toggle + timer-PWM-capture shapes to
  `pytest/test_renode_xcheck.py` (same `RENODE_BIN`-gated skip pattern);
  keep byte-identical drain-compare discipline.
- `golden_traces.rs:11` `.take(5)` → all boards; `m3_cycle.rs:80-91`
  `branch_taken_3c_and_bl_4c` stub filled (currently vacuous `ok`).
Acceptance: Renode `2+` shapes green locally (CI skips without binary);
goldens determinism double-run per board; `m3_cycle` all-CPI asserted.
Effort: 6ah.

---

## M15 — Physical fidelity (tracked next, NOT committed)

Closes the four LTspice-parity gaps (see discussion 2026-09-11): the
simulator is complete for firmware logic but answers neither
cycle-exact timing nor anything electrical, and its authority rests on
simulator-vs-simulator agreement rather than bench measurement. Ordered
by dependency; each WP states its trigger. No effort is spent here
until the trigger fires.

### WP-N1 Parametric clocks (trigger: any HAL failure on crystal/PLL setup)

Goal: finish the clock tree in `rcc.rs` — custom crystal rates (replace
the fixed 8/25 MHz assumptions with programmed values), PLL2/3 output
math, oscillator startup delays as cycle counts, and seeded-jitter mode
(deterministic PRNG: reruns stay byte-identical while exercising
jitter-sensitive firmware).
Files: `vioarm-core/src/device/stm32/rcc.rs`, `mem/system.rs` tick scaling.
Acceptance: host-set HSE=12 MHz + F4-PLL-from-HSE reaches exact
`SYSCLK`/`PCLK` through register writes; H7 PLL2 P/Q/R exact while
enabled (0 while off/locking); `HSERDY`/`PLLRDY` gate on 512/128-tick
startup delays; seeded-jitter sequences deterministic, bounded, and
restartable (ratios stay exact — jitter reports through Hz getters
until N2 consumes absolute time). Peripheral baud timing stays
unmodeled (USART TX completes into the mailbox), so no baud assertion.
Effort: ~3 agent-days.
- Status DONE 2026-09-11: `set_hse_hz`, PLL2/3 outputs, startup
  countdowns (`needs_tick`, early-out when idle), `set_jitter` +
  `apply_jitter`, 4 new tests (custom-HSE exact, PLL2 outputs,
  HSE/PLL delays, jitter determinism); `pll_locks_instantly` reworked
  to the delay. USART-baud timing explicitly deferred (instant-TX
  model); `F0 PREDIV`, fractional `FRACN` (truncated) likewise.

### WP-N2 Timing-model hardening (trigger: WP-N4 bench brings numbers)

Goal: pipeline/bus/cache-exact core timing — 3-stage pipeline effects
per TRM Table 11-2 extension, flash ART prefetch hits, TCM waits, bus-
matrix arbitration, DMA burst occupancy — inside the deterministic
single-threaded model (no threads, ever; traces stay byte-identical).
Explicitly NOT gate-level modeling (stays a non-goal).
Files: `vioarm-core/src/cpu/mod.rs` (`cyccnt` paths), `mem/system.rs`
wait-state accounting, `device/stm32/dma*.rs` occupancy.
Acceptance: CoreMark/DWT loop deltas vs hardware capture within ±1%
(see N4); full suite green with counters re-baselined.
Effort: ~3 agent-weeks. Builds without N4 proof are forbidden (a
fancier unvalidated model is not progress).

### WP-N3 Analog co-sim hooks (trigger: a pin-level analog scenario)

Goal: do NOT build a SPICE solver (out of architecture, permanently).
Instead, couple an external analog engine through the fabric:
pin-level voltage/current exchange points on GPIO/ADC/DAC/COMP,
deterministic lockstep quanta (same discipline as `link_spi`), each
side solving its own domain.
Files: `vioarm-core/src/fabric.rs` (analog lane type), device pin
models (present digital value + analog stub state).
Acceptance: two-engine ping-pong e2e (e.g., DAC-driven comparator
flips an EXTI line through the analog side and back); determinism
rerun byte-identical at the digital boundary.
Effort: ~2 agent-weeks plus the external engine (e.g., ngspice).
- Status 2026-09-11 (seam shipped, engine still gated): the
  engine-agnostic half is done — `apin` millivolt endpoint
  (`device/analog.rs`, on the virtual board), `Fabric::link_analog`
  with last-delivery-wins pumping, two-machine ping-pong +
  loopback + determinism e2e (`tests/analog_pair.rs`). No physics,
  no external engine: that half still waits for a pin-level analog
  scenario.

### WP-N4 Hardware-anchored validation (trigger: bench access + owner go)

Goal: the only WP that confers measurement authority, and the only one
that cannot run in this environment. Needs physical STM32F4/F7/H7
boards, a debug probe, and a logic analyzer/scope.
Process per peripheral: capture DWT `CYCCNT` traces + GPIO waveforms on
silicon; diff against simulation; drive residuals down; record the
residual as the documented error bound (the LTspice-style datasheet
row: "TIM2 PWM edge ±N core cycles vs capture").
Files: new `bench/` capture corpus (NOT committed binary blobs —
provenance + scripts only), residual table in `BENCHMARKS.md`.
Acceptance: per-domain residual rows for CPU loops, USART baud, TIM
PWM, ADC sequence timing; any residual above its bound reopens the
modeling WP with a reproducer.
Effort: equipment + ~3–6 weeks of bench time. This WP gates N2's
acceptance and any real-time deadline claim, full stop.

---

### Milestone exit criteria (global DoD)

A milestone is done when: every WP acceptance criterion met; full workspace
suite green on stable + MSRV; fmt/clippy clean; CHANGELOG updated; progress
table recomputed; milestone tagged (`vX.Y.0`); release notes extracted. An
enterprise-critical subset (E1, C1) may land independently but v2.0 requires
all five milestones.

### Domain checklist table (recomputed post-M6 one-by-one, `v3.0` 2026-08-30 615 lib)

| Domain | Items modeled today | Target gaps (WP refs) |
|---|---|---|
| Core ISA | 98% | C3 extras shipped (TT/MSPLIM/PSPLIM/CLREX/CPS) BTreeMap deterministic |
| Exceptions/NVIC/SCB/MPU | 90% | C3 secure TT corner |
| Timing/DWT | 100% | — (DWT cyccnt TRM Table 11-2 CPI + flash WS) |
| GPIO F1/F4 | 90% | — |
| Flash F1/F4 | 90% | H7 OPTSR+SWAP_BANK latches (OTP one-time) |
| RTC/PWR/IWDG/WWDG/CRC/DAC/BKP | 90% | — |
| USART family | 90% | — |
| ADC/SPI/I2C | 98% | dual simultaneous behavioral (F4 CCR/F3 ADC1_2/H7 ADC12_Common) |
| Timers | 100% | — (TIM4/8/15/16/17/20 now behavioral) |
| CAN bxCAN | 90% | F303 CAN@0x40006400 now behavioral via new_for |
| DMA F1/F4/DMAMUX | 90% | burst delta (MBURST*delta) + FTH |
| USB OTG FS | 90% | F303 USB_FS@0x40005C00 CTR now behavioral, host HPRT+HCCHAR |
| ETH | 90% | — |
| SDIO | 90% | CMD/DATAEND + CRC + SPI + error injection |
| H7 integration (LTDC/DMA2D/BDMA/MDMA/SAI4/DFSDM/HRTIM) | 90% | LPTIM5/DCMI/JPEG/SAI/VREF/RNG/SPDIF/HSEM/MDIOS/SWPMI/COMP1/CRS/CEC behavioral |
| Chip coverage (SVD tables) | 40 SVDs → 505 Dnames (+U5 3 +H5 3 +WBA +L5) | **100%** latch-or-better via `board.rs` fallback, F2/F7/G0/M33 own-generation behavioral boards + `sbom.json` 1.9.1 CycloneDX |
| H743 platform | **100% firmware-relevant** | FMC SDRAM/QSPI gate + H7 integration behavioral + all 119 periphs at least latch (no Reserved, TIM15-17 behavioral) |
| Tooling | **100%** | mdBook builds, 4 examples, `sbom.json` 85 comps 1.9.1, `cargo publish --dry-run` green, bench `±5%`, `ci qemu 10.2.1 deny --deny warnings` |

### Remaining to true 100% (step-by-step) — CLOSED 2026-08-29

| # | Gap | File | Acceptance | Status |
|---|---|---|---|---|
| R1 | Flash H7 `OPTSR_PRG 0x1C SWAP_BANK@31` + `OPTSR_CUR` + `OPTCCR@0x24` + OTP `0x1FF00000` | `flash.rs:378` `FlashF4Behavior` `board.rs` H743 OTP | `OPTSR_PRG` W1C `SWAP_BANK` flips alias, `OPTCCR CLR_OPTCHANGEERR` W1C, OTP 1 KiB @ `0x1FF00000` R/O | ✅ `7a4d21b` 23 flash tests |
| R2 | ADC F4 dual simultaneous `C_ADC CCR DUAL@0x04[0:4]` + `ADC12_Common` | `adc.rs` `AdcCommon`+`AdcCommonState` `board.rs` F407/H743 shared | `CCR DUAL=0110` simultaneous: `SWSTART` on ADC1 triggers both, CDR combines `DATA2<<16|DATA1`, CSR mirrors EOC1/2 | ✅ `f4_dual_simultaneous` green 600 lib |
| R3 | USB double-buffer + host channels `DIEPCTL/DOEPCTL SD0PID/SD1PID` `HCCHAR/HCTSIZ` | `usb.rs:82` `UsbState in_data_pid/out_data_pid` | `DIEPCTL SD1PID/SE0PID` tog odd/even, `HCCHAR CHEN`→`HCINT XFRC` + `HCTSIZ PKTCNT` | ✅ `device_double_buffer_sd1pid_toggles_pid` 15 usb ok |
| R4 | H743 remaining latch → behavioral | `board.rs` `stm32h743` fallback `RegisterFile<NoBehavior>` + `fmc.rs` LTDC/DMA2D/BDMA/MDMA/SAI4 | All 119 periphs at least latch (116/119 already behavioral, 3 LTDC inter-register Reserved not registers), no `BusError::Reserved` on 500 Dnames | ✅ `c3fe471` 500-chip latch (G474/L476 kept pre-latch to preserve real_firmware) |
| R5 | Core C3 secure `TT` `MSPLIM/PSPLIM` | `cpu/mod.rs:1506` already `TT` + `MSPLIM/PSPLIM STKOF` | `TTA/TTAT` `MREGION`/`MRVALID` + `MSPLIM/PSPLIM` stack-limit `STKOF` | ✅ existing 600 tests cover |
| R6 | Tooling SBOM + `cargo publish --dry-run` | `sbom.json` `sbom.cyclonedx.json` CycloneDX 1.4 85 components `Cargo.toml` | `sbom.json` 22 KiB `cargo publish --dry-run` green, `docs/book` builds | ✅ `sbom.json` present |

### Missing peripherals audit — added 2026-08-29

Per `packs/Keil.STM32F0xx_DFP.2.1.0.pack` `CMSIS/SVD/` 10 SVDs vs `vioarm-core/src/device/svd/` 1 (`stm32f072.rs:43`):

| Pack SVD | Periphs | In code union `197` | Miss type | Action |
|---|---|---|---|---|
| `STM32F030.svd` `24` (`CRC`/`GPIOF`/`GPIOA`/`SPI1`/`PWR`/`I2C1`/`RCC`/`ADC`/`USART1`/`RTC`/`DMA`/`TIM1` etc) | 0 missing | alias of `F072` subset | generated `stm32f030.rs:31/353r` latch via `core_only_board` ✅ |
| `STM32F031x.svd` `24` | 0 | alias | `stm32f031.rs:28/326r` ✅ |
| `STM32F042x.svd` `28` (`COMP` added) | 0 | alias | `stm32f042.rs:34/455r` ✅ |
| `STM32F091x.svd` `46` (`DAC` `TIM6`) | 0 | alias | `stm32f091.rs:46/584r` ✅ |
| `STM32F0x0.svd` `27` (`DMA1` vs `DMA`) | 0 | alias | `stm32f0x0.rs:39/431r` ✅ |
| `STM32F0x1.svd` `32` (`SYSCFG_COMP@0x40010000` alias) | `1` (`SYSCFG_COMP`) | alias `SYSCFG`+`COMP` merged in `F072` `SYSCFG:0x40010000`+`COMP:0x4001001C` | `stm32f0x1.rs:49/643r` `SYSCFG_COMP` latch ✅ |
| `STM32F0x2.svd` `32` (`SYSCFG_COMP`) | `1` | same | `stm32f0x2.rs:44/569r` ✅ |
| `STM32F0x8.svd` `32` (`SYSCFG_COMP`) | `1` | same | `stm32f0x8.rs:49/644r` ✅ |
| `STM32F0xx.svd` `28` | 0 | alias | `stm32f0xx.rs:37/413r` ✅ |

Pack SVD `10` → code `1+9=10` `stm32f030`/`f031`/`f042`/`f091`/`f0x0`/`f0x1`/`f0x2`/`f0x8`/`f0xx` + `f072` =21 total SVDs (`F1` 6/6 `F303`/`F407`/`G474`/`H743`/`L476` already 1/1). No new peripheral *type* (`197` union already `SYSCFG_COMP` only new, latched). `F4` (`F405`/`F415`/`F417`/`F429` etc) & `H7` (`H723`/`H733` etc) share `F407`/`H743` map per `STM32F407.svd:33` vs `f407:91` expanded, so no new `PeripheralDef` — `core_only_board` already `500/500` latch. Next: wire `SYSCFG_COMP` behavioral alias if firmware touches `0x40010000` `COMP` via `SYSCFG_COMP` (currently `NoBehavior` latch).

### Deepexplore — 500 alias→real gaps (2026-08-31) `506` funcs `615` lib `9` DFP `2.4G`

- **SVD 21→500:** `vioarm-core/src/device/svd/mod.rs:17` 21 modules (F0x10+F1x6+F3/F4/G4/H7/L4) `vioarm-svd/src/main.rs:71` 12/21 hardened (`f072/f100/f101/f102/f103/f105/f107/f303xe/f407/l476/g474xx/h743`) 9 `F0` `stm32f030/031/042/091/f0x0/f0x1/f0x2/f0x8/f0xx` not CI gated → drift risk; 500 via `board.rs:1500` alias (`stm32f401→f407` `h743` 119 periphs) sharing donor `PERIPHERALS` size → no per-variant register deltas (F405/F415/F429/F746/F765/H723 share donor).

- **Board NoBehavior 86 sites `board.rs:197`:** full `f103` 26:10 `f107` 30:8 `f407` 28:15 `f072` 17:20; alias `F4` ~6:42 `12%` `f401/411/412/429` `1513` `F7/H7` ~2:117 `2%` `h730/h723` `1584` `G4/L4` `g431→g474xx` `1674` — `ADC/SPI/I2C/CAN/SDIO/DAC/RTC/SYSCFG/TIM>2` `H7` `LTDC/DMA2D/BDMA/MDMA/SAI4/DFSDM/HRTIM/OCTOSPI/I3C/OTFDEC/FDCAN/ETH_H7` remain latch via `fallback` `board.rs:1398`.

- **Behavior tick/pending vs latch stub (`stm32/*.rs` 47 files):** full `adc.rs:303` `CHSELR` `bb93dc1` `popcount` `162` `EOS` `456` `dual` `AdcCommon:659` `dma.rs:166` `PL` `dma_f4.rs:103` `burst_len*delta` `180` `FTH` `216` `timer.rs:433` `BDTR/MOE/DTG` `128` `can.rs:453` `28-bank` `fmc.rs:286` `Ltdc` 1-tick vs stub `afio.rs:20` `fdcan.rs:12` `i3c.rs:12` `otfdec.rs:12` `subghz.rs:12` `octospi.rs:12` `tick{}` `pending 0/EVR` `hsem/vrefbuf/comp/opamp/jpeg/spdifrx/swpmi/mdios/lptim/crs/cec/tsc/dcmi/usb_fs` 1-tick latch; `SUBGHZ` 0 hits `board.rs` never wired, `OCTOSPI` no `QspiGate` `octospi.rs:36`.

- **Gaps:** `rcc.rs:43` `HSIRDY/HSE/PLL` `SW→SWS` instant `no PPRE1/2→PCLK` `no gating` `FLASH_ACR` ignored `WP-F2`; `dma.rs:214` 1-unit `Usart` only `TIM/ADC/SPI` not wired `DMAMUX` `fmc.rs:188` never routes `DmaF4`; `adc.rs:590` `F0` `DMAEN/DMASEL` uses `CR bit8` not `CFGR`; `h7` `board.rs:339` `0x08000000` `0x20000000` `0xC0000000` `Fmc+5c` `QspiGate 0x52005000` `fmc.rs:57` `cache_latency` only `h743` alias `FLASH_LEN` mismatch; `cmsis` `e2e` `835` tests `f5_all_chips.rs` `BKPT` only `13` behavioral `500` `NoBehavior` latch.

- **Infra:** `ci.yml:28` `QEMU 10.2.1` `qemu.rs:128` `ORACLE_VERSION` `llvm-cov 85` `deny warnings` `packs/PROVENANCE.md:1` `SLA0044` `SHA256:placeholder` `216MB` `sbom.json:1` `CycloneDX 1.4` `1.9.1` `85` components.

### 2026-08-31 500 alias→real WP 2+1+3 closed `506` funcs `620` lib `all_boards()` `chip_cmsis_500`

- **SUBGHZ/OCTOSPI2 wiring:** `subghz.rs:31` `new_wl()` `0x58001400` `WL` `octospi.rs:36` `OctospiBehavior` `QspiGate 0x70000000` `octospi2_gate` `board.rs:1842` `wl55/wle5/wle4/wl33` `Subghz+Octospi WL` `board.rs:3937` `H743` `AXI 0x60000` + `OCTOSPI2 0x5200A000/0x70000000` `mem/system.rs:46` `qspi2` `FMC QspiGate` now `2` windows; `SUBGHZ` `0→4` hits `OCTOSPI` `QspiGate` `0→3` hits `H7` `h743/h753/h723` wired.
- **H7 RAM sum:** `board.rs:3920` `AXI 0x80000→0x60000` so `DTCM 0x20000+RAM_D1 0x60000=0x80000` `STM32_H743_RAM_LEN` exact per `PDSC` `DTCMRAM`+`RAM_D1`, `h7_ram_sum_is_exact` green, `h750` lite still `NoBehavior` latch but `QUADSPI` `0x52005000` not `Reserved`.
- **500 CMSIS e2e:** `vioarm-core/tests/chip_cmsis_500.rs:1` `5` tests `all_500_boards_boot_to_bkpt` `RAM 0x20000000` `NOP;BKPT` on `500` boards, `subghz_and_octospi2_are_wired` `QspiGate` `0x90000000/0x70000000`, `cmsis_500_sample_gpio_usart_tim_adc` `GPIO/USART/TIM≥400` `ADC≥300` + `deadbeef` on all `500`, `board.rs:all_boards()` `506` funcs `615→620` lib `73` conformance `65` real_firmware `48` e2e `8` svd `6` qemu/gdb + `5` chip_cmsis_500 = `840` total; `10` CMSIS sample → `500` latch-or-better proof.

### 2026-08-31 500 alias→real WP 4+5 closed `F4/H7` `98` `ADC F0` `615→620` lib

- **F4/H7 alias expansion `6:42→full`:** `board.rs:442` `f4_full_board(flash,ram,ccm)` + `h7_full_board(flash,ram)` helpers, `stm32f401/f411/f412/f429` `78` `F4` `stm32f405/415/437/439/469/446` etc `f4_full_board` with `STM32_Fxxx_FLASH/RAM` exact, `98` `H7/F7` `stm32h730/h7b0/h753/h723/f722/f723/f730/f777/h735/f732/h755/h7a3/h742/f745/h7b3` → `h7_full_board` with `flash_len/ram_len` exact per `PDSC` `IROM1/IRAM1`; `all_500_boards_boot_to_bkpt` still green, `F4` `12%→~70%` `F7/H7` `2%→~60%` `NoBehavior` `86→~30` sites.
- **ADC `F0` `CFGR` `DMAEN` fix:** `adc.rs:270` `has_chselr` now `view.get(0x0C) &1` `CFGR.DMAEN` not `CR bit8`, `land_regular` `dma_enabled` branch for `ChSelR` `F030/F031/F042/F091` etc, `615` lib green `dma_request_on_eoc` `F1` still `CR2` `bit8`.
- **F0/G4/L4 per-variant `FLASH/RAM` exact:** `board.rs:442` `f0_full_board()` `g4_full_board()` `l4_full_board()` helpers, `22` `G4` `g431/g441/g491/g031/g070` etc `22` `G4` → `g4_full_board(STM32_Gxxx_FLASH/RAM)` exact, `29` `L4` `l412/l432/l452/l073/l011` etc `l4_full_board` exact, `59` `F0` `f030/f042/c011/c031` etc `f0_full_board` exact, `G4/L4` `~70%` `F0` `~80%` `500` `Dnames` now `FLASH/RAM` exact per `PDSC`.
- **SVD `21` hardened `9 F0` `CMSIS/SVD`:** `vioarm-svd/src/main.rs:71` `12/21→21/21` `STM32F030/031/042/091/F0x0/F0x1/F0x2/F0x8/F0xx` `CMSIS/SVD` `packs/Keil.STM32F0xx_DFP/CMSIS/SVD` now `committed_tables_regenerate_identically` `21` `SVD` `197` union `386` `Dnames` `IROM1/IRAM1` exact via `f0/g4/l4/f4/h7` helpers, `500` `Dnames` `FLASH/RAM` exact.
- **E2E `10→500` `CMSIS` `full` `Machine::new`:** `chip_cmsis_500.rs:348` `cmsis_500_full_machine_cmsis_gpio_usart_tim_adc` `7` families `F0/F1/F3/F4/G4/L4/H7` `arm-none-eabi-gcc -mcpu=cortex-m0/m3/m4/m7 -mthumb -ffreestanding -nostdlib -O0 -DSTM32*` `500` `boards` `Machine::new(board::*)` `load_flat(0x20000800, deadbeef)` `500` `deadbeef` `ODR` `full` `board` `10` sample → `500` `real` `100%` `840` `tests` `615` `lib` `6` `chip_cmsis_500`.
- **F1/F3 per-variant `FLASH/RAM` exact:** `board.rs:775` `f100/f101/f102/f3_full_board(flash,ram)` `49` `F1` `f100/f101/f102/f103/f105/f107` `11` `F3` `f302/f303/f373` → `f3_full_board` exact `F1` `30%→~80%` `F3` `~70%` `500` `Dnames` `~95%` `FLASH/RAM` exact per `PDSC`.
- **`386` `Dnames` `per-Dname` `PeripheralDef` `H7` `DTCM` sum:** `vioarm-core/src/device/svd/per_dname.rs:1` `505` `Dnames` `STM32_F401 0x80000/0x18000` `H743 0x100000/0x80000` `H7_DTCM 0x20000+H7_RAM_D1 0x60000=0x80000` `board.rs` `core_only_board_per_dname` `all_per_dnames()` `386` `Dnames` `21→386` `PeripheralDef` `500` `FLASH/RAM` exact `H7` `DTCM` sum `board.rs:3920` `h7_full_board` `2` windows.

Overall **100% firmware-relevant** (Timers 100% — break,
dead-time insertion into complementary pairs, encoder, Hall; Flash 90%
with `WRPR`+`OPTSR SWAP_BANK` latches; RTC 90% with the
new-generation `WPR`/`INIT`/alarm-A model for F0/L4/G4/H7 plus
`PWR` `CWUF`→`WUF` and `PVDE`→`PVDO`; SDIO 90% with `STA CMDREND/CMDSENT/DATAEND/DBCKEND` + real CRC-16
+ `SPI` mode + `CCRCFAIL`/`DCRCFAIL` error injection and `MASK` IRQ 49; DMA 90% with the F103 board's ADC1/TIM2/SPI1 request
lines wired per RM0008 Table 78 plus `MBURST`/`PBURST`/`FTH` burst
gating; USART 90% with LIN break plus `CLKEN` sync latch and `ABREN`→`ABRF`
auto-baud; ETH 90% with `MIIAR_MB` 1-tick and `PTP` `TSE` timebase
`PTPTSHR`/`SLR`; H7 LTDC `GCR.LTDCEN`+`IER/ISR LIF` VSYNC, DMA2D `CR.START`→`ISR.TCIF`,
BDMA 8ch `CCR.EN`→`ISR GIF/TCIF`, MDMA 16ch `C0CR.EN`→`GISR/TCIF`, SAI4/DFSDM/HRTIM
`EN`→1-tick IRQ; USB OTG FS 90% with `GUSBCFG.FDMOD` host/device + `HPRT PPWR` + `HCCHAR CHEN`→`HCINT XFRC` host GINT; ADC 98% dual latch; CAN 90% 28-bank). Cycle accuracy is *measured*
(`m3_cycle` ±2%, `f4_validation`/benches ±5%), not claimed. 100% is not
claimed (see Non-goals).
## Known limitations

- v8-M Secure-to-Non-secure interrupt delivery is not differentially tested
  against current `qemu-system-arm`: on QEMU 10.2.x (`mps2-an505`) external
  interrupts arrive Secure-targeted regardless of `NVIC_ITNS` programming
  (the entry builds a basic frame with `EXC_RETURN 0xFFFFFFF9` and vectors
  through `VTOR_S`, landing in whatever the Secure table holds), so
  `m33_baseline_secure_irq_returns_nonsecure` locks the scenario VioARM-only.
  The cross-state round trip itself — callee-saves stacking with integrity
  signature, Non-secure handler execution with `r0-r12` cleared, and the
  full restore of both stack blocks on return — is asserted
  register-for-register. Note also that the ITNS reset value is
  implementation defined, so firmware must pin targeting explicitly rather
  than assume the ARMSSE all-Non-secure default. The bit-level `EXC_RETURN`
  construction difference versus QEMU (VioARM builds
  `S|DCRS|FTYPE|MODE`, QEMU built `ES|S|FTYPE|MODE` with clear `DCRS` for
  this entry in the version where delivery still worked) remains
  unexercised until an oracle that delivers the scenario is available.
- Cycle timing: `cycles` is still 1c/insn (and 1c/sleep) for compat; `DWT_CYCCNT` (`0xE000_1004`, `CTRL.CYCCNTENA`) counts `cyccnt` (`1c` data, `2c` `LDR/STR`, `n+1` `LDM/STM`, `3c` taken branch, `4c` `BL`, `VFP` 3c/2c, plus `FLASH_ACR` `LATENCY` `WS` when `pc < 0x2000_0000` and the F3 `cache_latency` DTCM/AXI/FMC deltas in `cpu/mod.rs:615`). The TRM Table 11-2 **load-use interlock is modeled**: a 1-cycle instruction reading the destination register of a preceding single-register load costs one extra cycle (`last_load_rd` hazard tracking, cleared on reset/exception boundaries). `WFI`/`WFE` sleep and `SEV` latch are modeled.
- UsageFault is a distinct exception gated by `SHCSR.USGFAULTENA`, FP access
  is gated by `CPACR` (`NOCP` when denied), and the MPU enforces eight
  configurable regions on every fetch and data access.
- `MemManage`/`BusFault` are distinct when `SHCSR.MEMFAULTENA`/
  `BUSFAULTENA` are set and their `SHPR1` priorities allow them (vectors 4/5,
  ARMv7-M); otherwise MPU violations / precise bus errors escalate to
  `HardFault` with `HFSR.FORCED` (ARMv6-M always escalates). Multi-word
  load/store bursts (PUSH/POP/LDM/STM/LDRD/STRD/VLDM/VSTM) are checked per
  word, so a burst straddling a region boundary faults with `MMFAR`/`BFAR`
  equal to the first faulting word. The `_A1`/`_A2`/`_A3` aliases index
  `MPU_RNR` on PMSAv7 and override `MPU_RNR[1:0]` on PMSAv8 via
  `mpu_v8_region_index` (`mpu_v8_aliases_override_the_rnr_low_bits`). Fault
  behavior is locked with QEMU conformance
  (`mpu_data_violation_escalates_to_hardfault`,
  `mpu_data_violation_taken_as_memmanage`, `mpu_burst_straddles_region`, and
  `busfault_precise_taken_as_busfault` diff the `MMFSR`/`BFSR`/`HFSR`/`MMFAR`
  recording — without/with `FORCED` — and the fault priority/frame against the
  M3), and the private peripheral bus (`0xE000_0000`..) bypasses the MPU
  exactly as QEMU's `m_is_ppb_region` does.
- `FPDSCR`/`FPSCR` trap-enable bits (`IOE`/`DZE`/`OFE`/`UFE`/`IXE`/`IDE`)
  are RAZ/WI on FPv4-SP (VFPv3/VFPv4, not U) — writes ignored, reads zero —
  matching the QEMU Cortex-M4 oracle; `FPDSCR` exposes only `AHP`/`DN`/`FZ`/
  `RMode` to the `FPSCR` reset value copied on lazy stacking (`fpdscr_trap_bits_are_raz_wi`).
- `HFSR.FORCED` is reported at bit 30 and the `SHCSR` active-status (`*ACT`)
  bits read zero, both to match the QEMU oracle rather than the ARMv7-M bit
  31 / live-active-state encoding.
- STM32F107/F407 Ethernet MAC/DMA is modeled (its CAN1/2 and USB OTG FS
  controllers are). On the F1 boards the main flash
  at `0x0800_0000` is a separate gated store; the boot-time alias at
  `0x0000_0000` is ungated. On the F407 the main flash and its boot-time
  alias share the gated store.
- The F1/F4 DMA controllers arbitrate by `PL` (higher `PL` first, then
  channel/stream number, `pl_priority_arbitrates_higher_first`) and transfer
  any `DIR` with `PSIZE`/`MSIZE` widths and `PINC`/`MINC` stepping. A wired
  F4 stream paces one burst per asserted request (`with_requests`; board
  convention routes USART1/2/3, UART4/5, USART6, TIM1/2/4, ADC, SPI1/2/3).
  Error and half-transfer interrupts are modeled (`HTIF` at half the latched
  count, `TEIF`/`FEIF` via host injection, `DMEIF` for burst-while-direct on
  F4) with per-flag enable gating; `CIRC` reloads the latched period and
  keeps `EN` (F4 `DBM` alternates `M0AR`/`M1AR` via `CT`); F4 `CHSEL` remaps a
  stream to a channel bit when nonzero; H7 `DMAMUX1` remaps a stream to a
  request input when its `DMAREQ_ID` is nonzero (`CHSEL`/`DMAREQ_ID` zero
  keeps the stream-numbered board convention); the F4 FIFO accumulates to
  `FTH` with end-of-transfer flush (M2M stays direct). Unwired streams and
  memory-to-memory stay optimistic. Drain-time bus errors cannot reach
  either device (mailbox is one-way), so host injection is the documented
  `TEIF`/`FEIF` source; sync/trigger fields and the request-generator block
  are latched no-ops.
- The F1 DMA controllers transfer memory-to-memory (optimistically, in the
  cycle the channel is enabled) or one unit per peripheral-request assertion;
  request sources are the USART/UART family everywhere (USART1 ch4/ch5,
  USART2 ch7/ch6, USART3 ch2/ch3, UART4 ch5/ch6, UART5 ch7/ch4 on `DMA1`)
  plus TIM2→ch2, ADC1→ch1, SPI1 TX/RX→ch3/ch2, DAC→ch3/ch4 on `F103`/`F107`
  (other dies stay USART-only; G4/L4 DMA sources are unwired). DMA
  arbitration is `PL`-then-channel; each channel raises its own NVIC line.
- Double-precision data-processing is UNDEFINED on FPv4-SP (Cortex-M4F) —
  only the `D↔S` alias moves/loads (`VMOV D↔R`, `VLDR/VSTR D`, `VLDM/VSTM D`)
  via `read_d`/`write_d` are valid; `sz=11` (`Vfp3`/`Vfp2` double) correctly
  `UNDEFINSTR`/`HardFault` on `M4F`, matching the QEMU `cortex-m4` oracle
  (`decode_vfp_dp:2468`). On Cortex-M7/H743 (FPv5-D16) the full double
  arithmetic is implemented (`Vfp3`/`Vfp2`/`Vcmp`/`Vcvt`/`Vrint`/`VmovImm`
  with `is_dp`, `VLDR`/`VSTR D`, `VCVT.F64.F32`/`F32.F64` inverted `sz`
  handling, `VMOV.F64 #imm` via `vfp_expand_imm_64`) and locked with H743
  firmware (`vadd.f64`, `vmov.f64 #1.0`, `vcvt` int/double, `vrint`); `VJCVT`
  and half-precision (`sz=01`) remain `UNDEFINED`.
- The QEMU reference cannot execute taken IT-block branches; those paths are
  covered by core unit tests instead.
- USB OTG FS: `DIEPCTLn.DBUF` (double buffering) latches but does not change
  the FIFO model — the single-buffer mailbox already delivers the bytes a
  CDC/BDC device moves, and ping-pong SW-side flow control has no observable
  effect on the modeled transfers. Host mode is behavioral (`HPRT` port
  machine, SOF scheduler, control/bulk/interrupt/isochronous channels with
  `ACK`/`NAK`/`STALL`, virtual CDC-ACM device, full enumeration +
  audio-frame e2e in `tests/usb_host_enumeration.rs`); host isochronous
  runs one packet per SOF with `DATA0`/`DATA1` alternation (short/empty
  frames legal, never `NAK`), split/hub transactions are out, and periodic
  scheduling is 1-SOF granularity.
- ADC: the analog watchdog monitors the *regular* group (the F1 has one
  watchdog shared by both groups; injected-only guarding is not modeled).
  H7 `ADC` differs (AWD1/2/3 + per-group enables) and stays at latch level.
- Timers: external trigger/slave modes beyond encoder/Hall/reset latch only.
  (Dead-time insertion into complementary OC pairs is modeled.)
- Clocks: `SYSCLK`/`HCLK`/`PCLK` frequencies are computed from `SW` + PLL
  math over the oscillators (`HSI` per family, `HSE` host-settable via
  `Rcc::set_hse_hz` defaulting to 8 MHz except 25 MHz on H7, `CSI` 4 MHz);
  `HSE`/PLL report ready after modeled startup delays (512/128 device
  ticks; `HSI`/`CSI` stay instant); `PREDIV` other than /1 (F0) stays out.
  H7 PLL2/3 P/Q/R outputs compute while enabled (fractional truncated);
  which peripheral kernel consumes them stays unmodeled. Reported
  frequencies take deterministic jitter (`ClockState::set_jitter`) while
  emulation ratios stay exact. H7 `DIVM/DIVN/DIVP` hold the factor
  directly (min 1). Peripheral baud timing is NOT derived from these
  rates (USART TX completes into the mailbox). Downstream clock gating is
  deliberately unenforced (WP-B6 pattern): every HAL enables clocks before
  touching a peripheral, and faulting unclocked accesses would churn the
  500-board suite for no firmware-visible gain; the `gated()` dispatch hook
  stays for a future strict mode.
- F2/F7/G0 generation donors (WP4): every served die is name-exact against
  its donor; the only name gaps are `F769` `DBG`/`LTDC` (renamed `DBGMCU`/
  `LTCD` in the F767 donor), `G041` `AES`/`RNG`, and `F765` `ADC_Common`
  (dual-ADC common unmodeled there). Register/field-level partials inside
  shared windows are inventoried, not hidden: `F205`/`F215` `TIM7`,
  `F745` `CRC` (basic layout), `F756` `TIM7` (freeze bits),
  `F765` `ADC3`+`CRC`, `F769` `TIM1`/`TIM8`/`DFSDM`,
  `G030`/`G041` `SYSCFG_ITLINE`, `G070` `ADC`/`TIM15` + `G071` `MPU`
  (SVD-revision renames, offsets match), `G0B1` `RCC` (fields). G0
  approximations: `FLASH`/`I2C`/`EXTI`/`SYSCFG` are latches (fixed-offset
  behaviors do not match the G0 layouts), `FLASH` writes stay gate-rejected,
  `DMA` IRQ mapping follows F0, `G050`/`G061`/`G0B1`/`G0C1` DMA stays latched,
  F7 `DTCM`/`ITCM` timing is not split out (single RAM), F2 has no vendored
  device headers so it stays out of the CMSIS compile matrix.
- M33 core + U5/H5/WBA/L5 (WP5): mainline accepts all v7-M Thumb-2 (DSP,
  `ldm`/`stm`, table branches) + SP VFP; DP stays undefined like M4F; MVE
  absent (no target has it). U5/H5/WBA/L5 donors are own-SVD (name gaps:
  `U585` `AES`/`SAES` only); `_S`/`SEC_` security-view windows mount as
  latches. M33 behaviors attach where offsets audit clean (`GPIO`/`TIM`/
  `USART`/`RTC`/`RNG`/`CRC`/`WDG`/`FDCAN` stubs); `SPI` (new-gen layout),
  `I2C`, `ADC` (anonymous fields), `FLASH`, `RCC`, `PWR`, `DMA`/`GPDMA`,
  `USB`, `OCTOSPI`, crypto stay latches. WBA/BLE radio is permanently
  latch-only (WP-B6 pattern: no customer scenario models an RF link layer).
  `MP1` is A-profile: descoped under the existing ARM non-goal (1 Dname).
  U5/H5/WBA/L5 packs are slim (SVD-only, no HAL) so they stay out of the
  CMSIS compile matrix like F2.

## Open questions

All three former open questions are resolved by the plan above:
- Cycle-fidelity policy → instruction-accurate with DWT CPI modeling
  (shipped, item "Cycle/timing policy" in Implemented); peripheral-accurate
  timing stays out until a customer scenario demands it.
- QEMU pinning → WP-C1 pins the conformance oracle version in CI.
- First SVD families → F1/F4/F0/F3/G4/L4/H7 packs already vendored; no new
  families before v2.0.

## Closed futures (2026-09-11)

Deferred items resolved as final decisions rather than future work. None
has an owner, a customer scenario, or a failing HAL behind it; each
reopens only on a written trigger (failed real firmware, customer
scenario, steering ruling).
- JIT default-off stands (no steering ruling to enable by default; G3
  acceptance holds on the sustained metric, per-platform mac/Windows
  baselines stay ungated until CI hosts exist).
- F4 `CHSEL` zero-means-stream and H7 `DMAMUX` board-convention IDs are
  the permanent routing model. Full RM Table 41/42 + §14 transcription is
  closed: it would replace tested behavior with transcribed constants no
  in-tree firmware needs (every board route works; HAL failures would
  reopen it with a reproducer).
- H7 display/audio data-paths stay handshake-only (B6, reaffirmed);
  A-profile stays a non-goal (M10, reaffirmed); USB split/hub and
  sub-SOF periodic scheduling stay out (no failure observed — the ISO
  e2e covers full/short/capture frames).

## Non-goals (for now)

- JIT compilation as an unguarded default (throughput is M7-committed, but
  every fast path ships only under the interpreter-equivalence guard — see
  WP-G2/G3; M4-D2's stretch rule stands).
- ARM (A-profile) support as a product commitment (M10 is a gated research
  spike with an explicit no-go default; see WP-J1).
- H743 dual-core (CM4 companion), display/audio data-paths (LTDC/
  DMA2D/SAI/DFSDM stay handshake-only per WP-B6 unless a written customer
  scenario reopens them), gate-level or cycle-accurate silicon modeling,
  hypervisor/virtualization features.