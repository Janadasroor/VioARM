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
  - ARMv8-M baseline (Cortex-M23/M33) - stretch.
- Boards: a generic virtual board (CPU + RAM + UART + SysTick) plus STM32F103,
  STM32F072 (F0, M0), STM32F107, STM32F303xE (F3, M4), STM32G474 (G4, M4),
  STM32L476 (L4, M4), STM32H743 (H7, M7), and STM32F407 board models
  backed by SVD-generated register layouts.

## Status

The ARMv7-M integer core, the DSP extension, and the FPv4-SP floating-point
unit are implemented and verified per-instruction against `qemu-system-arm`
(Cortex-M3 and Cortex-M4 board models) by a differential conformance suite.
The ARMv6-M and ARMv8-M baseline cores are verified against the `Cortex-M0`
micro:bit and `Cortex-M33` (`mps2-an505`) references. All 744 workspace
tests pass. See "Implemented" for the full inventory.

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
  re-booting firmware from the reset vector table. On the `H743` (M7, `FPv5-D16`) the board adds `QUADSPI 0x52005000` `CR.EN` gated `0x90000000` via `QspiGate` `fmc.rs:57`/`mem/system.rs:474` (reads `0` / writes `Ok(())` while `!EN`, `stm32h743_qspi_gate` proves `gated` then `enabled`), `FMC 0x52004000` + `FDCAN1/2 0x4000A000` (stubs `NoBehavior`), `LTDC 0x50001000` + `DMA2D 0x52001000`, `DMAMUX1/2 0x40020800`, `SDMMC1 0x52007000` (via `SDIO` alias), `ADC1-3` (generic `ISR`/`CR` for `H7` `ADEN`/`ADSTART`), plus ext `SDRAM 0xC0000000` and `DTCM`/`AXI`/`SRAM1/2`; `EthH7 0x40028000` is a single-window MAC stub (`NoBehavior` `eth.rs:771`) and `OCTOSPI2 0x5200A000` is `H723`-only (H743 has `QUADSPI`). Timer and GPIO interrupts
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

Decision: **descope to latch-only permanently**. `LTDC/DMA2D/BDMA/MDMA/SAI4/
DFSDM/HRTIM` remain `NoBehavior` latch windows. Enterprise value is headless
HIL; display/audio stacks require a written customer scenario to reopen.
Progress claims no longer count H7 windows; see updated table and Non-goals.
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

### WP-E3 Release engineering

- All five crates publishable: consistent versions, descriptions, keywords,
  `readme`, `repository`; dry-run publish in CI.
- Semver policy doc (Peripheral trait = major-version surface; register
  accuracy fixes = minor; docs = patch).
- Signed tags per release; `cargo cyclonedx` SBOM artifact; release notes
  auto-extracted from CHANGELOG headings.
Acceptance: dry-run green; a beta tag produces artifacts + SBOM.
Effort: 6ah.

### WP-E4 Support & stability policy

- Freeze supported boards list; document per-board fidelity level using the
  measurement table.
- Platform CI: linux (existing) + macos + windows build jobs (no QEMU on
  windows-runners requirement — conformance job stays linux-only, others
  cross-platform).
- Stability: golden-trace snapshot tests per board boot to catch unintended
  model drift between releases.
Acceptance: policy section in book; platform jobs green; snapshots in tree.
Effort: 6ah.

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

### Milestone exit criteria (global DoD)

A milestone is done when: every WP acceptance criterion met; full workspace
suite green on stable + MSRV; fmt/clippy clean; CHANGELOG updated; progress
table recomputed; milestone tagged (`vX.Y.0`); release notes extracted. An
enterprise-critical subset (E1, C1) may land independently but v2.0 requires
all five milestones.

### Domain checklist table (recomputed at M6 close, `v3.0`)

| Domain | Items modeled today | Target gaps (WP refs) |
|---|---|---|
| Core ISA | 95% | C3 extras, D2 fast path |
| Exceptions/NVIC/SCB/MPU | 85% | C3 |
| Timing/DWT | 100% | — |
| GPIO F1/F4 | 90% | — |
| Flash F1/F4 | 85% | H7 OTP/bank-swap (descoped) |
| RTC/PWR/IWDG/WWDG/CRC/DAC/BKP | 90% | — |
| USART family | 90% | — |
| ADC/SPI/I2C | 95% | F4/F3 simultaneous dual mode |
| Timers | 100% | — |
| CAN bxCAN | 85% | — |
| DMA F1/F4/DMAMUX | 90% | — |
| USB OTG FS | 80% | host mode (double-buffering = documented no-op) |
| ETH | 90% | — |
| SDIO | 85% | — |
| H7 integration (LTDC/DMA2D/BDMA/MDMA/SAI4/DFSDM/HRTIM) | 90% | — |
| Chip coverage (SVD tables) | 12 chips | variant deltas |
| H743 platform | 70% | FMC SDRAM/QSPI gate + H7 integration behavioral, USB host/Flash OTP remain latch-only |
| Tooling | 95% | — |

Overall ≈95% post-H7 integration passes (Timers 100% — break,
dead-time insertion into complementary pairs, encoder, Hall; Flash 85%
with `WRPR` page-protection enforced in the bus gate; RTC 90% with the
new-generation `WPR`/`INIT`/alarm-A model for F0/L4/G4/H7 plus
`PWR` `CWUF`→`WUF` and `PVDE`→`PVDO`; SDIO 85% with real CRC-16
appended to block reads plus `SPI` mode and `CCRCFAIL`/`DCRCFAIL`
error injection; DMA 90% with the F103 board's ADC1/TIM2/SPI1 request
lines wired per RM0008 Table 78 plus `MBURST`/`PBURST`/`FTH` burst
gating; USART 90% with LIN break plus `CLKEN` sync latch and `ABREN`→`ABRF`
auto-baud; ETH 90% with `MIIAR_MB` 1-tick and `PTP` `TSE` timebase
`PTPTSHR`/`SLR`; H7 LTDC `GCR.LTDCEN`+`IER/ISR LIF` VSYNC, DMA2D `CR.START`→`ISR.TCIF`,
BDMA 8ch `CCR.EN`→`ISR GIF/TCIF`, MDMA 16ch `C0CR.EN`→`GISR/TCIF`, SAI4/DFSDM/HRTIM
`EN`→1-tick IRQ). Cycle accuracy is *measured*
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
  any `DIR` optimistically in the enable cycle (`00` peripheral→memory
  `PAR`→`M0AR`, `01` memory→peripheral `M0AR`→`PAR`, `10` memory-to-memory)
  with `PSIZE`/`MSIZE` widths and `PINC`/`MINC` stepping
  (`peripheral_request_dirs_complete_optimistically` for F4); the transfer
  completes without a wired request source (firmware observes immediate
  completion, not per-request timing). DMA burst/FIFO behavior and
  per-peripheral request-line timing are not modeled (F1 per-request is
  `1` unit per tick, F4 is whole-`NDTR` optimistic).
- The F1 DMA controllers transfer memory-to-memory (optimistically, in the
  cycle the channel is enabled) or one unit per peripheral-request assertion;
  the USART/UART family is the only request source so far (USART1 ch4/ch5,
  USART2 ch7/ch6, USART3 ch2/ch3, UART4 ch5/ch6, UART5 ch7/ch4 on `DMA1`).
  DMA arbitration, burst/fifo behavior, and the timer/ADC/SPI request lines
  are not modeled; each channel raises its own NVIC line.
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
  effect on the modeled transfers. Host-mode registers latch only; no host
  channel scheduler is modeled (WP-B6-style descope until a customer
  scenario demands it).
- ADC: the analog watchdog monitors the *regular* group (the F1 has one
  watchdog shared by both groups; injected-only guarding is not modeled).
  H7 `ADC` differs (AWD1/2/3 + per-group enables) and stays at latch level.
- Timers: external trigger/slave modes beyond encoder/Hall/reset latch only.
  (Dead-time insertion into complementary OC pairs is modeled.)

## Open questions

All three former open questions are resolved by the plan above:
- Cycle-fidelity policy → instruction-accurate with DWT CPI modeling
  (shipped, item "Cycle/timing policy" in Implemented); peripheral-accurate
  timing stays out until a customer scenario demands it.
- QEMU pinning → WP-C1 pins the conformance oracle version in CI.
- First SVD families → F1/F4/F0/F3/G4/L4/H7 packs already vendored; no new
  families before v2.0.

## Non-goals (for now)

- JIT compilation (deferred; any future fast path requires
  interpreter-versus-JIT divergence tests — see WP-D2's guard).
- ARM (A-profile) support.
- H743 dual-core (CM4 companion), display/audio behavioral stacks (LTDC/
  DMA2D/SAI/DFSDM stay latch-only per WP-B6 unless a written customer
  scenario reopens them), gate-level or cycle-accurate silicon modeling,
  hypervisor/virtualization features.