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
  STM32F107, and STM32F407 board models backed by SVD-generated register
  layouts.

## Status

The ARMv7-M integer core, the DSP extension, and the FPv4-SP floating-point
unit are implemented and verified per-instruction against `qemu-system-arm`
(Cortex-M3 and Cortex-M4 board models) by a differential conformance suite.
The ARMv6-M and ARMv8-M baseline cores are verified against the `Cortex-M0`
micro:bit and `Cortex-M33` (`mps2-an505`) references. All 663 workspace tests
pass. See "Implemented" for the full inventory.

### Implemented

Core execution:

- The full ARMv6-M 16-bit instruction set and broad Thumb-2 (ARMv7-M)
  coverage: MOVW/MOVT, data processing with `ThumbExpandImm`, the load/store
  family, LDRD/STRD, LDM/STM, TBB/TBH, divide, multiply (including 64-bit and
  DSP forms), parallel add/subtract with SEL, sign/zero extension, saturating
  and bit-field operations.
- VFP FPv4-SP data processing (arithmetic, fused multiply-add, compare,
  conversions, immediate and register moves, VMRS/VMSR) plus VLDR/VSTR and
  VLDM/VSTM, backed by a softfloat implementation ported from QEMU.
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
  words. An SCB covers the modeled subset (ICSR, `AIRCR` with
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
  re-booting firmware from the reset vector table. Timer and GPIO interrupts
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

- None. The ARMv8-M baseline (item 9 below) is complete.

### Gaps to a real STM32 simulator

The instruction core is done and QEMU-verified; what separates the project from
a faithful STM32 SoC simulator is the checklist below, in priority order.
"Real firmware" means code that runs on silicon: HAL/Standard-Peripheral
projects, RTOSes, and bare-metal drivers.

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
   across sleep. Locked in with CPU unit tests, machine e2e tests, and CMSIS
   firmware that `WFI`-loops until `SysTick` wakes it.
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
    receive-echoes a frame under interrupts.
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

- No cycle-accurate timing: one cycle per retired instruction (and one per
  sleeping cycle), `WFI`/`WFE` sleep until an exception wakes them, and
  `SEV`/`WFE` share a 1-bit event latch. Time-sensitive firmware that needs
  real cycle counts (baud rates, peripheral delays) still does not reproduce
  silicon timing. See the cycle/timing gap (item 6).
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
  decode-fails to `UNDEFINSTR`/`HardFault`, matching the QEMU Cortex-M4
  oracle (`decode_vfp_dp:2468`). A double-arithmetic FPU would be a new ISA
  (FPv5-D16 / Cortex-M7).
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