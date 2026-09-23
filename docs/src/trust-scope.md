# Trust scope

Version: Phase C complete (`aa333b4`), 2026-09-21. This page states exactly
what VioARM claims about its 21 SVD boards and what it does not. It is
versioned with `boards-roadmap.md`: any change to the claims lands here in
the same commit.

## How to read the claims

- **T2 (behavioral)**: a hand-written device is mounted; every register
  answers, interrupts route, and the documented dynamics run. Proven by
  lib tests and board-level direct-poke proofs (`*_closeout.rs`).
- **T3 (proven)**: T2 plus real firmware evidence on that board
  (scoreboard programs, fabric pairs, vectors through the registers).
- **T1 (latch)**: a pure register file. Only two remain across 1,046 bus
  windows, both documented below — everything else is T2+.
- **Audit**: `cargo run -p vioarm-core --example board_audit`
  (address-coverage matching). Current matrix: **1,045 behavioral, 1 latch,
  60 core-owned**, in `boards.md`.

## Per-board claims (21/21 closed)

| Board | Windows | Behavioral | Latch | Firmware-proven (T3) on this board |
|---|---|---|---|---|
| stm32f030 | 30 | 30 | 0 | GPIO/USART/TIM/ADC (`chip_cmsis_500`) |
| stm32f031 | 27 | 27 | 0 | — |
| stm32f042 | 33 | 33 | 0 | COMP hysteresis |
| stm32f072 | 42 | 42 | 0 | CRC, DAC, GPIO, SPI |
| stm32f091 | 45 | 45 | 0 | — |
| stm32f0x0 | 36 | 36 | 0 | — (generic donor) |
| stm32f0x1 | 46 | 46 | 0 | — (generic donor) |
| stm32f0x2 | 41 | 41 | 0 | — (generic donor) |
| stm32f0x8 | 46 | 46 | 0 | — (generic donor) |
| stm32f0xx | 36 | 36 | 0 | — (generic donor) |
| stm32f100 | 46 | 46 | 0 | — |
| stm32f101 | 44 | 44 | 0 | — |
| stm32f102 | 23 | 23 | 0 | — |
| stm32f103 | 51 | 51 | 0 | Dhrystone, CoreMark, FreeRTOS, Embench, GPIO/USART1/TIM2, fabric pairs, CAN arbitration |
| stm32f105 | 43 | 43 | 0 | CAN1 fabric arbitration |
| stm32f107 | 47 | 47 | 0 | Embench-IoT, TFLM `hello_world`, ETH/USB ISRs, SPI pairs |
| tm4c123gh6pm | 58 | 56 | 2 | Dhrystone 9.124, CoreMark valid (400 it), UART/GPIO/Timer e2e (`scripts/kings/ports/tm4c`) |
| tm4c1294ncpdt | 64 | 59 | 5 | Dhrystone 9.124, CoreMark valid (400 it), UART/GPIO/Timer e2e (`scripts/kings/ports/tm4c` 129 builds) |
| tm4c129xnczad | 72 | 70 | 2 | Dhrystone 9.124, CoreMark valid (400 it), UART/GPIO/Timer e2e (same 129 ELFs, tick-identical) |
| stm32f303xe | 56 | 56 | 0 | CRC, DAC1/2, GPIO, chip_cmsis legs |
| stm32f407 | 82 | 82 | 0 | GPIO/USART/TIM2, ADC IRQ, Flash, DMA, USB host |
| stm32g474xx | 82 | 82 | 0 | — |
| stm32h743 | 119 | 118 | 1 | ETH echo, USB host enumeration, FDCAN loopback |
| stm32l476 | 71 | 71 | 0 | — |

## Explicit non-claims

These are true of the simulator and out of scope by design:

- **`stm32h743` `AXI` latch**: interconnect issuing/QoS/arbtration tuning
  with no firmware-observable dynamics in a functional simulator. Reset
  values are the working configuration. T2 under the contract-driver rule
  (pure-latch drivers count T2); the audit labels it T1. If a workload
  ever observes AXI shaping, that workload files the model request.
- **Radio physics**: SUBGHZ/BLE windows move link-layer frames between
  virtual peers (`radio_peer.rs`); access-address/CRC/PHY on-air behavior
  and RF propagation are not modeled.
- **Timing**: cycle counts are estimates (`DWT_CYCCNT` model, `m3_cycle`
  harness ±2% on F1). The silicon comparison rig (Phase D) has not run;
  no number here is calibrated against hardware.
- **Analog**: one ideal bus voltage shared by ADC/COMP/GPIO/EXTI/TIM;
  no noise, impedance, temperature, or supply ramp.
- **Crypto**: AES ECB/CBC/CTR, GCM (96-bit IVs, NIST 800-38D), GMAC,
  CMAC (800-38B), suspend/resume, and SHA-1/224/256 + HMAC (RFC 2202/4231)
  are behavioral with register-driven vectors, where attached (L4/G0/U5
  tables, G041 classic, U585 AES/SAES+RNG). Out of scope by design:
  MD5/HMAC-MD5 (zeroed digest), non-96-bit GCM IVs, empty CMAC,
  classic-table suspend, DMA requests, device-unique key hiding.
- **USB**: FS-device PMA transfers unmodeled; isochronous host channels
  STALL; HS high-bandwidth paths handshake-only.
- **Ethernet**: checksum/CPC pass-through (bytes never modified); MTL/MMC/
  PMT latch-only; single-buffer frames.
- **FDCAN**: bit-timing, error frames, TX-event FIFO, cancellation,
  timestamping unmodeled (both tiers).
- **Flash**: option-byte reload, WPSN/PCROP protection enforcement, CRC/ECC
  engines, OTP unmodeled. H7 bank 2 stays locked (SVD marks `KEYR2`
  read-only).
- **Gating**: peripheral clock gating formally descoped — everything is
  clocked.
- **JIT**: default-off translation engine; no timing or behavior claim
  changes when enabled (differential suites pin parity).

Per-driver limits beyond this list live in the module docs (`device/stm32/*.rs`).

## What voids these claims

- A failing `cargo test --workspace`, a dirty `clippy`, or an audit run
  whose numbers differ from the table above.
- Firmware evidence (T3 column) lapsing: re-runs must pass without sim
  changes (acceptance gate).
- Silicon-rig divergence, once measured, replaces the timing estimates —
  this page then records calibrated numbers.

## Re-verify

```sh
cargo fmt --all && cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo run -p vioarm-core --example board_audit
cargo test -p vioarm-core --test f5_all_chips
```
