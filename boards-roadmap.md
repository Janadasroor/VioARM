# VioARM Boards Roadmap (full peripheral support)

Take all 21 SVD boards from "every register exists" to "every peripheral
works": each peripheral on each board climbs from latch-only stub to a
proven behavioral model, in order of what real firmware actually uses.

Relation to the other roadmaps: `roadmap.md` stays authoritative for the
core/milestone plan; `cosim-roadmap.md` owns SPICE coupling;
`jit-roadmap.md` owns the translation engine; `roadmap-final.md` owned
host embedding. This file owns only peripheral completeness
(`vioarm-core/src/device/`, `vioarm-core/src/board.rs`,
`docs/src/boards.md`, `docs/src/arch.md`).

Agent protocol (same as everywhere): one work package per branch/commit
series; `cargo fmt --all && cargo clippy --workspace --all-targets -- -D
warnings && cargo test --workspace` green before every commit;
`CHANGELOG.md` updated under `Unreleased` for user-visible work.

## Support tiers (the definition of "fully supported")

| Tier | Meaning | Example |
|---|---|---|
| T0 | Register map exists (SVD-generated address, nothing behind it) | — (all 21 boards start here or above) |
| T1 | Latch-only: reads/writes don't fault, values latch, no behavior | HRTIM/SAI4/DFSDM/MDMA on `h743` |
| T2 | Behavioral: the peripheral actually does its job in the common modes | USART TX, timers, AES ECB/CBC |
| T3 | Proven: register-level tests + proof (vectors, real firmware, silicon) | AES/HASH (NIST vectors), USB enum e2e |

A board is **fully supported** when every peripheral on its SVD table is
T2 or higher, and every peripheral real firmware touches is T3.

## Starting point (already shipped)

- 21 SVD tables (`vioarm boards`: 25–119 peripherals each, ~1,050 slots
  total), 506 named chip variants + `virtual` (507 `--board` values),
  every variant smoke-booted to `BKPT` (`f5_all_chips`).
- 13 headline boards with pin-label tables (`vioarm boards --pins`).
- ~50 shared driver files in `vioarm-core/src/device/stm32/` (+ `uart`,
  `gpio`, `dwt`, `systick`, `nvic`, `scb`, `analog`): one driver fix
  upgrades every board that mounts it.
- T3 proofs already banked: AES/HASH (NIST), USB device enum, H7
  Ethernet echo, FDCAN loopback, Dhrystone/CoreMark/FreeRTOS/mbedTLS/
  ThreadX/Embench/TFLM scoreboard (`docs/src/scoreboard.md`).

## Phase 0 — baseline audit (do this first; ~1 week)

No box can be checked without knowing which boxes are unchecked. For
each of the 21 boards: enumerate its SVD table, mark every peripheral's
current tier (T1/T2/T3), and record it as a matrix in
`docs/src/boards.md`. Deliverables:

- [x] Audit script (`vioarm-core/examples/board_audit.rs` + `Peripheral::behavior_kind` hook): builds all 21 family boards, classifies all 1,046 bus windows — **454 behavioral**, 592 latch, +60 core-owned. Run: `cargo run -p vioarm-core --example board_audit`.
- [x] Published matrix: `docs/src/boards.md` "Phase 0 audit" section (summary table + per-board lists); every Phase A/B/C item links to these cells.
- [x] Pin tables for the 7 real boards missing them (`f031/f042/f091/f100/f101/f102/f105`; `vioarm boards --pins` now 20 tables).
- [x] Variant sanity (`vioarm-core/tests/variant_sizes.rs`): all 505 per-Dname boards carry exact PDSC flash/RAM extents (flash top readable, past-flash `Reserved`, RAM bank tops incl. H7 DTCM/AXI split).

## Phase A — shared-driver T2 push (all boards benefit at once)

Work is per unique IP, not per board. Each item: model → lib tests →
board e2e with real firmware → `arch.md` + CHANGELOG.

- [x] **USART RX + full duplex**: RX error model (`ORE` on overrun, `FE`/`PE` stimulus bits, F1 `SR`-then-`DR` sequence, new-gen `ICR`, error IRQs via `EIE`/`PEIE`); `RE` gating deliberately unenforced (documented). Proof: 5 lib tests + loopback firmware on `f103` + `h743` (`tests/usart_loopback.rs`).
- [x] **SPI slave + full duplex master**: `OVR` (old byte kept via `rx_data` shadow) + `UDR` (unstaged frames) + `SR`-then-`DR` clear sequence + `ERRIE` IRQ; `SSOE` output and slave `TXE` consume deliberately unmodeled (documented). Proof: 5 lib tests + real C-firmware pair over `Fabric::link_spi` (`f103`+`f107`, both directions, `UDR`/`OVR` asserted — `tests/spi_pair_hand.rs`). The pair wire itself was already proven (`fabric_spi_pair.rs`).
- [x] **I2C full transaction set**: repeated `START` mid-transfer, 10-bit headers, `BTF` completion flags, clock-stretch stimulus bit; receive-`OVR` dropped as provably unreachable, slave mode / `ARLO` / `PECERR` deliberately unmodeled (documented). Proof: 4 lib tests + real C-firmware EEPROM random-read on `f103` (`Sr`, `BTF`, 10-bit, wire drain — `tests/i2c_eeprom.rs`).
- [x] **ADC full modes**: scan/continuous/injected/watchdog/DMA already covered (19 lib tests); added external triggers (`EXTTRIG`/`JEXTTRIG`, `EXTEN`/`JEXTEN`) + per-channel stimulus + fixed the `ADON` byte-lane restart artifact. Proof: 4 lib tests + real C-firmware scan/watchdog/injected on `f103` (`tests/adc_scan.rs`).
- [x] **DAC + COMP/OPAMP chains**: triangle/noise waveforms (`MAMP` mask, LFSR documented) + DAC→bus publish (wired on all mounts) + `COMP` bus on `f042`/`h743`; timer-triggered waves, wave+DMA, `F0` `EXTI` routing, `OPAMP` gain deliberately unmodeled (documented). Proof: 3 lib tests + C firmware triangle + `DAC→ADC` on `f103` + hysteresis hold-in-band on `f042` (`tests/dac_wave.rs`, `tests/comp_hyst.rs`).
- [x] **RTC + backup domain**: backup registers survive system reset (found wiped — fixed), tamper (`TEF`/wipe/`TPIE`→`TAMPER` IRQ 2, `CTEF` clears); `PWR.DB`, `RCC.BDCR`, wakeup timer, alarm B deliberately unmodeled (documented). Proof: 3 lib tests + C firmware `RTCAlarm` interrupt through the NVIC + backup magic across host reset on `f103` (`tests/rtc_alarm.rs`).
- [x] **Watchdogs**: `WWDG` counter loads from `T` on enable/refresh (below-edge enable resets at once) and counts on `PCLK/4096` — the base prescaler the first e2e proved load-bearing (without it, `EWI` and underflow land in one step and reset preempts the handler); `IWDG` keys/prescaler/reload already covered. Proof: 1 lib test + C firmware `EWI` through the NVIC + reboot counters for `WWDG` underflow and `IWDG` expiry on `f103` (`tests/wdg_reset.rs`).
- [x] **CRC full**: programmable `POL`/`POLYSIZE`/`REV_IN`/`REV_OUT`/`INIT` over a transaction-grouped MSB-first engine (byte/halfword/word paths); fixed units bit-identical; `RESET` keeps config now. Proof: 6 exact check values + C firmware ISO-HDLC/CCITT-FALSE on `g474` (`tests/crc_vectors.rs`).
- [x] **RNG startup + health**: rewritten with resolved offsets, `CECS`/`SECS` halt+`CEIS`/`SEIS`+IRQ, W0C/disable recovery, `CED` switch; fixed missing `drive_inputs` forwarding and stale-return clobbers. Proof: 5 lib tests (smoke + flags + recovery) + board smoke on `h743` (`tests/rng_smoke.rs`).
- [x] **General DMA completion**: no driver changes — pacing, FIFO/`DBM`/`M2M` IRQs, request wiring already proven (52 lib tests + `f2_clock_dma.rs`); added the missing firmware proof: C `MEM2MEM` copy + circular reload on `f103` (`tests/dma_flow.rs`). **Phase A complete (10/10).**
- [ ] **SPI slave + full duplex master**: master TX exists; add slave
      capture both directions, NSS handling, IRQ/DMA paths. Proof:
      fabric master/slave ping-pong both directions.
- [ ] **I2C full transaction set**: START/STOP, ACK/NACK, clock stretch,
      10-bit + IRQ paths on F1/F4. Proof: EEPROM-style firmware e2e.
- [ ] **ADC full modes**: scan/continuous/injected + analog watchdog +
      DMA on F1/F4/H7 (basic single-shot is T2). Proof: multi-channel
      ramp vs expected codes.
- [ ] **DAC + COMP/OPAMP chains**: waveform output, comparator IRQs with
      hysteresis (bus side exists). Proof: triangle-out + threshold IRQ.
- [ ] **RTC + backup domain**: calendar, alarms, tamper, BKP persistence
      across reset. Proof: alarm-IRQ firmware, persistence check.
- [ ] **Watchdogs (IWDG/WWDG)**: reset on expiry, window check, EWI.
      Proof: reset-observed e2e.
- [ ] **CRC full**: polynomial/size/mode paths on F1/F4/H7. Proof:
      known-answer vectors.
- [ ] **RNG startup + health**: DRDY, error flags, reseeding behavior.
      Proof: statistical smoke + flag checks.
- [ ] **General DMA completion**: request pacing exists (F4/H7/F1
      sources); add FIFO/threshold errors, double-buffer, MEM2MEM IRQ
      paths. Proof: TIM→DMA→USART chain e2e.

## Phase B — complex peripherals (one work package each)

- [ ] **USB host**: HPRT/SOF scheduler + channels exist behaviorally;
      close split/hub/isochronous gaps. Proof: virtual CDC-ACM + HID
      device traffic.
- [ ] **Ethernet full (H7)**: echo exists; add checksum offload,
      multi-buffer frames, MMC/PMT counters. Proof: large-payload echo
      with verified checksums.
- [ ] **CAN/FDCAN timing**: loopback exists; add bit-timing, error
      frames, TX-event FIFO, cancellation. Proof: error-injection e2e.
- [ ] **SDIO/SDMMC full speed paths**: card image exists; add 4-bit,
      high-speed, error paths. Proof: multi-block read/write compare.
- [ ] **Crypto completion**: GCM/GMAC/CMAC, HMAC, suspend/resume banks
      (currently zeroed/latch). Proof: NIST vectors per mode.
- [ ] **Display/camera (LTDC/DMA2D/DSI/DCMI/JPEG)**: descoped in B6;
      handshake behavior first, then data paths. Proof: frame-IRQ e2e.
- [ ] **Analog extras (DFSDM/SAI/SPDIFRX/OPAMP maps)**: currently
      latch-only on H7/G4; promote in firmware-demand order.
- [ ] **Radio honesty (SUBGHZ/BLE)**: keep documented-virtual stance;
      close CRC/address filtering on the link layer only. No RF physics
      (non-goal, stays documented).

## Phase C — per-board close-out (21 checklists)

For each board below: enumerate its SVD table, mark every peripheral
T1/T2/T3, promote T1→T2 in Phase A/B order, attach pin table if headline,
record result in `docs/src/boards.md`. All pending.

- [ ] `stm32f030` (31) — F0 baseline close-out
- [ ] `stm32f031` (28)
- [ ] `stm32f042` (34)
- [ ] `stm32f072` (43)
- [ ] `stm32f091` (46)
- [ ] `stm32f0x0` (39)
- [ ] `stm32f0x1` (49)
- [ ] `stm32f0x2` (44)
- [ ] `stm32f0x8` (49)
- [ ] `stm32f0xx` (37)
- [ ] `stm32f100` (48)
- [ ] `stm32f101` (46)
- [ ] `stm32f102` (25)
- [ ] `stm32f103` (53) — scoreboard board; close first, it's the reference
- [ ] `stm32f105` (45)
- [ ] `stm32f107` (49) — scoreboard + fabric board; close second
- [ ] `stm32f303xe` (59)
- [ ] `stm32f407` (91) — DMA/FPU reference board
- [ ] `stm32g474xx` (91)
- [ ] `stm32h743` (119) — largest table; close last, complex IPs land here
- [ ] `stm32l476` (80)

## Phase D — proof upgrade (T2 → T3 where firmware demands)

- [ ] Silicon comparison rig: same binaries on real Blue Pill (F103)
      + F107 board; diff UART logs and `DWT_CYCCNT` vs VioARM. Converts
      cycle estimates into calibrated numbers.
- [ ] New famous firmware per quarter (Zephyr sample, MicroPython port,
      or similar): each either passes (evidence) or files the next work
      item with a reproducer.
- [ ] Trust-scope page (`docs/src/trust-scope.md`): claimed vs
      non-claimed, versioned with this roadmap.

## Final acceptance gate (the definition of done)

The roadmap is finished when ALL of these hold; until then it stays open
no matter how many boxes are checked:

- [ ] Audit matrix (Phase 0) shows zero T1 cells on all 21 boards —
      every peripheral is T2+ and every firmware-touched path is T3.
- [ ] Full suite green: `cargo fmt --all && cargo clippy --workspace
      --all-targets -- -D warnings && cargo test --workspace`, plus the
      `f5_all_chips` 506-variant boot smoke.
- [ ] Scoreboard green end-to-end (Dhrystone, CoreMark, FreeRTOS,
      mbedTLS, ThreadX, Embench 19/19, TFLM) with no new sim fixes
      required by the final runs — the suite passes on a simulator that
      no longer needs to change for it.
- [ ] Silicon rig (Phase D) concurs on UART logs; `DWT_CYCCNT`
      divergence vs real F103/F107 documented per benchmark (calibrated
      numbers replace estimates).
- [ ] `CHANGELOG.md` Unreleased section drained into a release entry;
      `docs/src/boards.md` + `arch.md` describe the finished state.

## Estimates (Fermi, one focused engineer)

- Phase A: ~10 items × 3–7 days ≈ **6–10 weeks**.
- Phase B: ~8 items × 1–3 weeks ≈ **8–16 weeks**.
- Phase C: mostly falls out of A/B (shared drivers); per-board tail
  (pins, close-out, docs) ≈ **2–4 weeks**.
- Phase D: rig + firmware ≈ **2–4 weeks**, ongoing.
- **Full scope: ~9–15 months. Headline-13 + used-peripherals cut: ~3–6
  months.** Proof (tests/vectors/silicon) is ~half of every item.

## Standing rule (demand-driven)

A peripheral jumps the queue the moment real firmware needs it: run the
firmware → pass (bank as evidence) or fail (work item with reproducer
attached, regression test named before the fix lands). The backlog burns
in order of usefulness, never alphabetically.
