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
micro:bit and `Cortex-M33` (`mps2-an505`) references. All 722 workspace tests
pass. See "Implemented" for the full inventory.

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

- None. Cycle-accurate timing hardening (item 6) is complete.

### Gaps to a real STM32 simulator

The instruction core is done and QEMU-verified; what separates the project from
a faithful STM32 SoC simulator is the checklist below, in priority order.
"Real firmware" means code that runs on silicon: HAL/Standard-Peripheral
projects, RTOSes, and bare-metal drivers. Measured `2026-08-22` `df266a7`
`v1.7.0` `64` `real_firmware` `550` `lib` `29` clippy:

| Domain | Silicon `denom` | Modeled `num` | Reach | File |
|---|---|---|---|---|
| Core ISA `Armv6M/7M/7EM/8M-baseline` `Thumb-2+DSP+FPv5-D16` | `QEMU m0/3/4/33/7` `73` `conformance` | `≈95%` | `cpu/mod.rs:41` `decode32.rs:991` |
| System `NVIC/SCB/MPU/TrustZone` `DWT cyccnt` `FLASH WS` | `M3` `HFSR bit30` | `≈85%` | `cpu/mod.rs:714` `scb.rs:1059` |
| Clock/Power `HSE/HSI/CSI/PLL1-3` `PWR CR1/CR3` | `Cube HAL SystemClock 25MHz` | `≈30%` | `rcc.rs:43` `pwr.rs:13` `d238b56` |
| GPIO `CRL/CRH→MODER/BSRR` | `HAL GPIO_Init` | `≈90%` Full | `gpio.rs:28` `gpio_f4.rs:28` |
| USART/UART `1-3+4/5/6` `TXE/RXNE` `DMA ch` | `USART1 92 regs` | `≈85%` Full | `usart.rs:67` `board.rs:914` |
| TIM1..7 `PSC/ARR/UIF` `TIM5 32b` | `HAL_TIM` | `≈60%` up-counter | `timer.rs` |
| DMA `F1 ch / F4 streams+DMAMUX` `MEM2MEM` `FIFO/burst` | `Cube BURST` | `≈40%` `1-unit/tick` `USART` only | `dma.rs:139` `dma_f4.rs:139` |
| FLASH `KEYR/PG/PER/MER` `F1/F4` sector | `HAL FLASH_Program` | `≈80%` | `flash.rs:17` |
| ADC/SPI/I2C `SQR/BR/START` | `HAL_ADC` | `≈75%` | `adc.rs` `spi.rs` `i2c.rs` |
| CAN/RTC/CRC/DAC/IWDG/SDIO | `SDIO CMD0/2/3/7` stub `0xFF` | `≈80%` | `sdio.rs:34` |
| USB `FS/HS` ETH `MAC/DMA` `RDES/TDES` | `HAL USB ETH` | `≈60%` `EthH7` stub `eth.rs:771` | `fmc.rs:13` |
| H7 `QUADSPI/FMC/LTDC/DMA2D/FDCAN` `BDMA/MDMA` | `119 H743 SVD →45 38%` | `≈38%` `QspiGate dd60d89` gated | `board.rs:817` `fmc.rs:57` |

Overall `weighted` `≈58-62%` to `HAL/RTOS no-mod` (`F103 66%→75% HAL`, `F107 100% windows 80% HAL`, `F407 51%→55%`, `H743 38%→35%` H7 `SystemClock HSE CR.EN 1` now `1`).

### Path to 100% — `58% → 100%` (priority, `effort`, `target %`)

1. **H7 `QSPI XIP` + `FMC SDRAM` init** `FMC SDCR/SDTR` `QUADSPI CCR/DLR` `1d` `H743 38→55%`.
2. **DMA `burst/FIFO` + `DMAMUX` routing + `timer/ADC/SPI` request lines** `1w` `DMA 40→85%` `TIM` `60→85%`.
3. **Clock tree `PLLSRC/DIVM/DIVN/DIVP/Q/R` `RCC 0x58024428` `PLLCFGR` `CFGR/D1CFGR`** `3d` `Clock 30→80%` `HSE 25MHz →480MHz` `SystemClock` `WS 5`.
4. **TIM `PWM/OC/IC` `CCMR/CCR` `DMA burst` `ADC triggered`** `1w` `TIM 60→90%`.
5. **CAN `filter-bank 28×` `29-bit ext` `FIFO0/1` `RFLM`** `3d` `CAN 80→95%`.
6. **USB `endpoints 1-3` `IN/OUT` `FIFO` `OTG HS2 0x40080000`** `1w` `USB 60→90%` `eth.rs:771` `EthH7` `DMAMR` `1w` `ETH 60→90%`.
7. **H7 `LTDC layer/PFC` `DMA2D PFC` `BDMA 0x58025400` `MDMA` `SAI4` `DFSDM`** `2w` `H7 55→75%`.
8. **Quality `llvm-cov 90%` + `cargo fuzz` decoder + `cargo audit/deny` strict** already `ci.yml:51` `1d` `R2-4` `prod` gate.
9. **`no_std` + `benches/cyccnt`** `R1/R6` `3d` reuse.
10. **Tag `v1.8-v2.0` per domain `CHANGELOG` `crates.io` `cargo publish`** `1d` `100%` when `HAL` `CubeMX` `RTOS` boots unmodified (`≈6-8w` total).

1. **Distinct UsageFault.** **Done.** A real exception with its own vector,
   `UFSR` fault-status recording, `SHCSR.USGFAULTENA` gating, and escalation
   to HardFault when disabled or un-preemptable (locked with QEMU conformance).
2. **CPACR gating of VFP.** **Done.** FPU access checks CP10/CP11; a disabled
   coprocessor raises `UFSR.NOCP` UsageFault (locked with QEMU conformance).
3. **MPU.** **Done.** `MPU_TYPE`/`MPU_CTRL`/`MPU_RNR`/`MPU_RBAR`/`MPU_RASR`
   (with the v7M `_A1`/`_A2`/`_A3` aliases indexing `RNR`, matching the QEMU
   oracle), eight unified regions with subregions, the `PRIVDEFENA` background,
   and AP/XN permission checks on every fetch and data access. Violations
   escalate to HardFault with `MMFSR.IACCVIOL`/`DACCVIOL`/`MMARVALID` and the
   address in `MMFAR`; the NMI/HardFault bypass and `HFNMIENA` are honored.
   Locked in with unit tests and a compiled-firmware e2e.
4. **Peripheral-request DMA.** **Done.** F1 DMA now handles request-driven
   transfers: the USART asserts TX/RX request lines (keyed by controller and
   hardware channel, TX on `DMA1` ch4, RX on ch5) and each enabled channel
   transfers one unit per assertion, counting down `CNDTR`, self-clearing
   `EN`, and latching `TCIF`. Per-channel NVIC lines replace the single
   combined line. Locked in with unit tests and CMSIS firmware that transmits
   and receives over `DMA1` under per-channel interrupts.
5. **F407 target completeness.** **Done.** The F407 flash controller
   (`0x4002_3C00`) is modeled with KEYR unlock, `PG`/`SER`/`MER`/`SNB`/`STRT`
   programming and sector erase (sharing the main-flash write gate), and
   `EOP`/`WRPERR` status plus `EOPIE`/`ERRIE` interrupts. The stream-based DMA
   (`DMA1`/`DMA2` with `SxCR`/`SxNDTR`/`SxPAR`/`SxM0AR`/`SxM1AR`/`SxFCR`)
   runs memory-to-memory transfers with per-stream NVIC lines routed through
   the SVD IRQ table (including the scattered `DMA1_Stream7` IRQ 47), and the
   64 KiB CCM RAM at `0x1000_0000` is mapped. All three are locked in with
   CMSIS firmware compiled in test.
6. **Cycle/timing policy.** **Done.** One cycle per retired instruction (or per
   sleeping cycle), and `WFI`/`WFE` now actually sleep the core: the machine
   keeps ticking peripherals and peeking for pending exceptions while asleep,
   so `SysTick` fires, wakes the core through exception entry, and the thread
   resumes the instruction after the `WFI`. `SEV`/`WFE` implement the 1-bit
   event latch (an interrupt is itself a wakeup event; `WFE` consumes a pending
   event without sleeping), and the machine's cycle counter stays monotonic
   across sleep. **TRM-accurate hardening done:** `DWT_CYCCNT` (`0xE000_1000`,
   `CTRL.CYCCNTENA`) counts `Core::cyccnt` with per-opcode CPI (data `1c`,
   `LDR/STR 2c`, `LDM/STM n+1`, taken branch `3c`, `BL 4c`, variable `DIV`
   `2-12c` via divisor leading zeros, `MUL 1c`/`MLA 2c`/long `3c`) plus
   `FLASH_ACR LATENCY` wait-states on fetch (`0x4002_2000` F1 /
   `0x4002_3C00` F4); the machine ticks peripherals by the instruction's
   cycle delta. Locked in with CPU unit tests, machine e2e tests, CMSIS
   firmware that `WFI`-loops until `SysTick` wakes it, and the e2e flash-wait
   lock (`cyccnt_incudes_flash_wait_states`).
7. **Peripheral breadth.** Only GPIO, USART1, TIM2/3, EXTI, RCC, FLASH, DMA are
    behaviorally modeled. **IWDG/WWDG watchdog done:** the 12-bit/7-bit
    down-counters implement their key/refresh protocols, prescalers, the WWDG
    window, `EWIF`/early-wakeup interrupt, and a shared reset latch that
    reboots the whole machine on underflow (locked in with unit tests and CMSIS
    firmware that lets the watchdog reset it). **ADC done:** ADC1/2/3 with
    software-start sequence conversion, fixed 14-cycle latency, `EOC`/`OVR`
    interrupts, `CONT`/`SCAN` re-arm, 12-bit `ALIGN`, per-channel analog input
    drive, and the F1 second-`ADON` restart quirk (locked in with unit tests, a
    machine e2e interrupt dispatch, and CMSIS firmware sampling channels 0/1).
    **SPI done:** SPI1/2/3 master shift registers with 8/16-bit `DFF` frames,
    `BR` prescaler-timed transfers, `TXE`/`RXNE` level interrupts, host-driven
    slave byte input and `MOSI` capture (locked in with unit tests, a machine
    e2e interrupt dispatch, and CMSIS firmware running polled and
    interrupt-driven frames). **I2C done:** I2C1/2 (F103/F107) and I2C3 (F407)
    master transfers — `START`/`STOP`/`SWRST`, 7-bit addressing with `ADD10`,
    polled and EV/ER level interrupts (address, buffer, and error events), and
    a host-driven slave byte that NACKs an absent address (locked in with unit
    tests, a machine e2e interrupt dispatch, and CMSIS firmware running polled
    and interrupt-driven master reads). This also fixed a machine modeling gap:
    an exception return now guarantees the restored instruction executes before
    a still-pending level-sensitive line re-enters the handler, so a hot line
    no longer starves the thread. **USART done:** the single USART1 model is
    generalized to the whole USART/UART family — USART2/USART3 on every line,
    UART4/UART5 on the connectivity and F4 lines, and USART6 on the F4 — each
    table-driven (base + IRQ) with the byte-oriented `TXE`/`TC`/`RXNE`
    behavior and the F1 DMA request routing per instance (USART1 ch4/ch5,
    USART2 ch7/ch6, USART3 ch2/ch3, UART4 ch5/ch6, UART5 ch7/ch4; locked in
    with unit tests, a machine e2e interrupt dispatch for USART2, and CMSIS
    firmware transmitting and interrupt-receiving over USART2 at its real
    `0x4000_4400` registers). **Timers done:** the TIM2/TIM3 counter model is
    generalized to the whole TIM1..TIM7 family — TIM4/TIM6/TIM7 (16-bit) and
    TIM5 (32-bit) on every line, plus the advanced TIM1, whose SVD table lists
    the break interrupt first so the update event routes through the *second*
    table line (`TIM1_UP`, IRQ 25) via a per-instance IRQ index; the boards
    stay faithful to real silicon (the F103 keeps its TIM1/2/3/4/6/7, the
    F107/F407 add TIM5). This also fixed a counter-model bug: the overflow
    comparison now reads the *unmasked* count, so a 16-bit wrap past
    `0xFFFF` still raises `UIF` instead of silently resetting to zero
    (locked in with a full-range regression test). Locked in with unit tests,
    an SVD-layout conformance test, a machine e2e dispatch through TIM1's UP
    line, and CMSIS firmware arming the basic TIM6 at its real `0x4000_1000`
    registers. **CAN done:** the STM32 `bxCAN` controllers (`CAN1`/`CAN2` on
    the connectivity and F4 lines, base `0x4000_6400`/`0x4000_6800`) with the
    init handshake (`INRQ`↔`INAK`), three transmit mailboxes with
    write-one-to-clear status, abort, and `BTR`-derived wire latency, and two
    receive FIFOs with `FMP`/`FULL`/`FOVR` and the `RFLM` lock (the SVD tables
    carry no filter-bank data registers, so received frames always land in
    FIFO0). The four interrupt lines (TX, RX0, RX1, SCE) map through the SVD
    IRQ table, and the host injects or drains frames as 16-byte mailbox-word
    descriptors. Locked in with unit tests, an SVD-layout conformance test, a
    machine e2e dispatch through the `CAN1_RX0` line, and CMSIS firmware
    (`STM32F10X_CL`) that receives on `CAN1` and transmits back. **USB done:**
    the STM32 USB OTG FS controller in device mode (`USB_OTG_FS` at
    `0x5000_0000` on the F107, `OTG_FS`/`OTG_HS` on the F407), driven through
    the SVD register tables: the `FDMOD` soft-reset handshake, the
    receive-FIFO pop path (`GRXSTSR`/`GRXSTSP`), the `USBRST`/`ENUMDNE`/`RXFLVL`
    interrupt bits, and the EP0 IN/OUT state machines with `STUP`/`XFRC` and
    the TX FIFO. Both NVIC lines dispatch their ISR, and the host injects or
    drains bytes as 8-byte descriptor words plus a 4-byte status word. Locked
    in with unit tests, an SVD-layout conformance test, a machine e2e dispatch
    through the `OTG_FS` line, and CMSIS firmware (`STM32F10X_CL`) that
    soft-resets the core, enumerates, and answers a GET_DESCRIPTOR request from
    RAM. **Ethernet done:** the STM32 Ethernet (MAC/MMC/PTP/DMA windows on the
    F107 at `0x4002_8000` and the F407 at `0x4002_8000` with the MAC subset
    window) — a mailbox-driven DMA model: the host pushes an RX frame as a
    `[CMD_RX, len_lo, len_hi, payload...]` word and drains a transmitted frame
    as `[EVT_TX, ...]`; the model walks the two self-chained descriptor rings
    (`RDES`/`TDES` with `OWN`, `RBS1`/`TBS1`, `RCH`/`TCH`, `FS`/`LS`),
    synthesizes `DMASR`/`DMAIER` flags (`NIS`/`RS`/`TS`/`RBUS`/`TPSS`, write-one
    to-clear), honors `MACCR` `RE`/`TE` plus `DMAOMR` `SR`/`ST` as the RX/TX
    enable gates and `DMATPDR` as the TX poll, and raises the `ETH` (IRQ 61 →
    vector 77) and `ETH_WKUP` (IRQ 62 → vector 78) lines so both ISRs dispatch
    on the F107. The F407 shares the model with the MAC window difference (`0x61`
    with no `MACRWUFFR`). Locked in with unit tests, an SVD-layout conformance
    test, a machine e2e dispatch through the `ETH` line, and CMSIS firmware
    (`STM32F10X_CL`) that drives the descriptor rings directly and
    receive-echoes a frame under interrupts. **CRC done:** the STM32 CRC
    calculation unit (`CRC` at `0x4002_3000`) with `DR`/`IDR`/`CR` and
    polynomial `0x04C1_1DB7` (init `0xFFFF_FFFF`, `CR.RESET` reloads; locked in
    with unit tests `crc_resets_to_all_ones_and_computes`/`idr_is_plain_storage`
    and mounted on the F103/F107/F407 boards).     **DAC done:** the STM32 DAC
    (`DAC` at `0x4000_7400`) with `CR` (`EN`/`TEN`/`TSEL`/`DMAEN`), `SWTRIGR`
    software trigger, `DHR12R1/L1/8R1`→`DOR1` and dual `DHR12RD` (immediate when
    `TEN==0`/`DMAEN==0`, latched on `SWTRIGR` when `TEN==1`/`TSEL==0`,
    `DMAEN` asserts `DMA1 Ch3/4` `to_periph` request on `SWTRIGR` for `F103`;
    `dac_immediate_…`/`dac_triggered_…`; mounted on F103/F107/F407, `F103` `DAC`
    wired to `DMA1`).
    **RTC/BKP done:** the backup domain (`RTC` at `0x4000_2800`, `BKP` at
    `0x4000_6C00` on F1) — `RTC` now accurate prescaler/counter/alarm
    (`PRL`→`DIV`→`CNT` 32-bit, `SECF`/`ALRF`/`OWF`, `CRH` enables, `CNF`/`RTOFF`
    gating, `RTCAlarm` IRQ 41, `rtc_prescaler_ticks_counter_and_alarm`; `BKP`
    remains `DR1..10` latches, `RTC` on all lines, `BKP` only on F1/F107).
    **SDIO done:** the SD-card host (`SDIO` at `0x4001_8000` on F1 / `0x4001_2C00`
    on F4) with `POWER`/`CLKCR`/`ARG`/`CMD`/`RESP`/`STA`/`ICR`/`MASK`/`DLEN`/
    `DCTRL`/`FIFO` CMD+data+card-accurate (`CPSMEN` latches `RESPCMD`/`RESP1`,
    `STA.CMDSENT`/`CMDREND`/`DATAEND`/`DBCKEND` + `MASK` → IRQ 49, `FIFO`
    `RXDAVL` queue, card `CMD0/2/3/7/8/55/41` `OCR`/`RCA`/`CID`, `sdio_card_init…`;
    mounted on F103/F107/F407 so `SD_Init` HAL completes — `CSD`/`card image`
    remain stubbed as `0xFF` blocks). **PWR done:** the STM32 PWR (`PWR` at `0x4000_7000`) with
    `CR`/`CSR` (`DBP`/`PVDE`/`WUF`) latch (`pwr_registers_do_not_fault`;
    mounted on F103/F107/F407, `DBP` gates backup domain on silicon but `RTC`/
    `BKP` latches ignore it as HAL always sets `DBP` first).
8. **ARMv6-M (Cortex-M0/M0+)** and instruction-table/SVD codegen hardening.
   **Done.** The ARMv6-M exception model is now distinct from ARMv7-M: `MRS`/
   `MSR` of `BASEPRI`/`BASEPRI_MAX`/`FAULTMASK` read zero and ignore writes
   (the QEMU M0 RAZ/WI behavior), a `UsageFault` no longer exists as a
   distinct exception — the fold to `HardFault` is unconditional — and
   `CONTROL.nPRIV`/`CONTROL.FPCA` are RAZ/WI with only `SPSEL` writable in
   Thread mode (verified by `armv6m_has_no_basepri_or_faultmask`,
   `armv6m_undefined_faults_fold_to_hardfault`, and the updated
   `sysreg_primask_and_control`). A `cortex_m0()` board exposes the generic
   peripheral set under the ARMv6-M core, and real-firmware locking includes
   `cortex_m0_rejects_thumb2_firmware`, which compiles `MOVW` for a
   Cortex-M4 and proves the v6-m core escalates it to `HardFault` before any
   side effect. **QEMU M0 differential conformance added:** the harness now
   runs the v6-m machine against a `Cortex-M0` micro:bit reference (same
   per-instruction trace diff as the M3/M4 suite), covered by
   `m0_sysreg` (MSP/PSP/PRIMASK round-trips, the BASEPRI/FAULTMASK RAZ/WI
   gates, and the SPSEL-only CONTROL), `m0_alu_thumb1` (the Thumb-1
   data-processing forms), `m0_loadstore_thumb1` (word/byte/halfword,
   register/SP-relative offsets, literal loads, push/pop), `m0_branch_bl`
   (16-bit conditional branches, `CBZ`/`CBNZ`, `BL`/`BX`), and the exception
   pair `m0_exception_svc`/`m0_hardfault_undef` (SVCall round-trips and the
   undefined-instruction fold to HardFault). Codegen hardening: `decode16` is
   verified by exhaustive sweeps of all 65536 16-bit encodings
   (`armv6m_decoder_is_subset_of_armv7m` asserts every v6-m acceptance is a
   v7-m acceptance and every v7-m rejection is a v6-m rejection;
   `it_block_encodings_are_v7m_only` proves `IT` blocks are never ARMv6-M),
   and `vioarm-svd` regenerates all three committed tables byte-for-byte from
   their packs (`committed_tables_regenerate_identically`).

### Done

9. **ARMv8-M baseline (Cortex-M23).** **Done.** `Isa::Armv8MBaseline` is a
   superset of ARMv6-M plus the baseline Thumb-2 subset: hardware divide
   (`SDIV`/`UDIV`), 32-bit branches (`B.W`), exclusive access
   (`LDREX`/`STREX`/`CLREX` word/byte/halfword), `MOVW`/`MOVT`,
   load-acquire/store-release (`LDA*`/`STL*` including the exclusive forms),
   IT blocks, the TrustZone security extension (`SG`, `TT`/`TTT`/`TTA`/`TTAT`,
   `BXNS`/`BLXNS`/`FNC_RETURN`, banked `MSP`/`PSP`/`CONTROL`), stack-limit
   registers (`MSPLIM`/`PSPLIM`), the PMSAv8 MPU programmer's model with
   `SAU` attribution, and the v8-M exception model (Secure HardFault vs
   Non-secure HardFault escalation gated by `AIRCR.BFHFNMINS`, `SecureFault`
   exception 7 with the `ES`/`DCRS` exception-return validity check and
   `SFSR.INVER`, the callee-saves (`DCRS`) block stacked on Secure-to-Non-secure
exception entries with the `SFSR.INVIS` integrity-signature check on return,
    the `ITNS`-targeted delivery of external interrupts (the ARMSSE default
    routes every line to the Non-secure state, so an IRQ taken from a Secure
    background is a Secure-to-Non-secure entry that switches the handler into
    Non-secure with `r0-r12` cleared),
    banked `S_VTOR`/`VTOR` vector tables, and the `S`/`ES`/`SPSEL`
    `EXC_RETURN` construction). Verified two ways:
   - Differential conformance against QEMU `cortex-m33` (`mps2-an505`) as a
     *superset* oracle (`m33_baseline_*` suite: sysreg, additions,
lda_stl, splim, tt/sau_tt/mpu_tt, alu_thumb1, sg, blxns/bxns,
      secure_undef_hardfault, secure_udf_return_restores_r0r3,
      secure_irq_returns_nonsecure, nonsecure_udf_returns_through_secure_handler),
      plus the v6-m
      suite against `cortex-m0`.
   - Unit tests pinning the *boundary*: `v8m_baseline_rejects_v7m_only` proves
     the full Thumb-2 DSP/bitfield/saturate/VFP zones, wide data-processing,
     conditional branches, `blx` immediate, `tbb`/`tbh`, `ldm`, and VFP
     encodings decode-fail under `Armv8MBaseline` while still decoding on
     ARMv7-M.
   Documented baseline simplification kept: `SecureFault` is taken
   unconditionally (`SFSR` causes are recorded but no `SF` software
   configurable/fault handler enable is modeled).

## Known limitations

- Cycle timing: `cycles` is still 1c/insn (and 1c/sleep) for compat; `DWT_CYCCNT` (`0xE000_1004`, `CTRL.CYCCNTENA`) counts `cyccnt` (`1c` data, `2c` `LDR/STR`, `n+1` `LDM/STM`, `3c` taken branch, `4c` `BL`, plus `FLASH_ACR` `LATENCY` `WS` when `pc < 0x2000_0000`). `WFI`/`WFE` sleep and `SEV` latch are modeled. See item 6 hardening.
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

## Open questions

- Cycle-fidelity policy: peripheral-accurate versus instruction-accurate
  timing once a timing milestone is scheduled.
- Whether the conformance suite should pin QEMU at a specific version.
- Which chip families receive the first generated SVD descriptors.

## Non-goals (for now)

- JIT compilation (deferred; any future fast path requires
  interpreter-versus-JIT divergence tests).
- ARM (A-profile) support.