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

## Phase B — complex peripherals (one work package each)

- [x] **USB host**: split start/complete sequencing via `HCSPLT` (`OTG_HS` only) + virtual HID boot-keyboard (descriptors, idle/protocol/report/LED, interrupt-IN reports); hub topology + hub-class stay documented non-goals. Proof: HID enumeration + key report + LED on `f407` `OTG_FS`, bulk OUT/IN split pairs on `OTG_HS` (`tests/usb_host_hid_split.rs`).
- [x] **Ethernet full (H7)**: TX `FD`..`LD` chaining + RX scatter, checksum insertion (`TDES3.CIC`) + validation (`MACCR.IPC`, `IPCB`/`IPHE`/`IPCE`+`ES`), MMC TX-good/unicast-RX counters; VLAN/TSO, MTL, MMC control, PMT wakeup stay documented non-goals. Proof: 1400-byte UDP echo with wire-verified checksums + scatter + error-flag + MMC on `h743` (`tests/eth_h7_checksum_mmc.rs`).
- [x] **CAN/FDCAN timing**: bxCAN `TEC`/`REC` + `EWGF`/`EPVF`/`BOFF` + sticky `LEC` + `TERR` + bus-off freeze/recovery (timing stays `BTR`-derived, proven tick-exact); FDCAN `ECR`/`PSR` (destructive reads), error-destroyed RX, `DAR` abort/retry, TX event FIFO, `TXBCR` cancellation. Proof: error-injection suites in `can.rs`/`fdcan.rs` lib tests (escalation to bus-off and recovery both controllers).
- [x] **SDIO/SDMMC full speed paths**: paced data path (`CLKCR` divisor/width, `CMD6` HS doubling, tick-exact), `CMD18`/`CMD25` multi-block with `CMD12` end, per-block CRCs, `DTIMER` deadline (`DTIMEOUT`), `CTIMEOUT` on clash, `CMD13` polling. Proof: 4-block write→read byte-compare + pacing + timeout/clash lib tests (`sdio.rs`).
- [x] **Crypto completion**: AES GCM/GMAC/CMAC (GHASH, phased `GCMPH`, subkeys, `SUSPR`), HASH HMAC-SHA1/224/256 (two-step `DCAL`, `LKEY`) + `CSR` suspend/resume; MD5/empty-CMAC/non-96-bit-IV stay documented non-goals. Proof: NIST vectors per mode + board proofs on `u585`/`u575` (`aes.rs`/`hash.rs` lib tests, `crypto_boards.rs`).
- [x] **Display/camera (LTDC/DMA2D/DSI/DCMI/JPEG)**: LTDC layers + reload + line IRQ, DMA2D fills/copies with format conversion over a bus mailbox, DCMI snapshot bytes + frame flag, JPEG handshake; pixel pipelines/blending, DSI (no SVD/panel), DMA capture, JPEG codec stay documented non-goals. Proof: board frame-IRQ e2e on `h743` (`display_frame_irq.rs`).
- [x] **Analog extras (DFSDM/SAI/SPDIFRX/OPAMP maps)**: SAI FIFOs + frame rhythm + mute/overrun, DFSDM bus conversions + watchdog + per-filter lines, OPAMP bus-follow modes/gains, SPDIFRX sync + data + parity; slots/DMA/breaks/cal/VP_SEL/decode stay documented non-goals. Proof: DAC→OPAMP→ADC→DFSDM chain + SAI/SPDIFRX legs on `h743` (`analog_extras.rs`).
- [x] **Radio honesty (SUBGHZ/BLE)**: documented-virtual stance kept (no RF physics); link-layer address filtering (sync-word network address, silent drops) + CRC-16 verify (`CRCERR`, `DROPSTAT`) over length-prefixed air frames. Proof: loopback + addressed 3-machine delivery with drop counts (`radio_peer.rs`, `radio.rs` lib tests).

## Phase C — per-board close-out (21 checklists)

For each board below: enumerate its SVD table, mark every peripheral
T1/T2/T3, promote T1→T2 in Phase A/B order, attach pin table if headline,
record result in `docs/src/boards.md`. All pending.

- [x] `stm32f030` (31) — F0 baseline closed: 30/30 behavioral, zero latch
      (new `DBGMCU` + I2C new-gen engine; record in `docs/src/boards.md`).
- [x] `stm32f031` (28) — closed: 27/27 behavioral, zero latch
      (shared-driver mounts + direct-poke proofs; record in `docs/src/boards.md`).
- [x] `stm32f042` (34) — closed: 33/33 behavioral, zero latch
      (shared-driver mounts + 13 direct-poke proofs; record in `docs/src/boards.md`).
- [x] `stm32f072` (43) — closed: 42/42 behavioral, zero latch
      (new `usart4_for` + shared-driver mounts + 10 proofs; record in `docs/src/boards.md`).
- [x] `stm32f091` (46) — closed: 45/45 behavioral, zero latch
      (new `usart7/8_for` + shared-driver mounts + 11 proofs; record in `docs/src/boards.md`).
- [x] `stm32f0x0` (39) — generic donor closed: 36/36 behavioral, zero latch
      (new `usart5_for` + shared-driver mounts + 13 proofs; record in `docs/src/boards.md`).
- [x] `stm32f0x1` (49) — generic donor closed: 46/46 behavioral, zero latch
      (`Syscfg` fallback for `SYSCFG_COMP` + 19 proofs; record in `docs/src/boards.md`).
- [x] `stm32f0x2` (44) — generic donor closed: 41/41 behavioral, zero latch
      (shared-driver mounts + 19 proofs; record in `docs/src/boards.md`).
- [x] `stm32f0x8` (49) — generic donor closed: 46/46 behavioral, zero latch
      (layout identical to `f0x1`; record in `docs/src/boards.md`).
- [x] `stm32f0xx` (37) — generic donor closed: 36/36 behavioral, zero latch
      (shared-driver mounts + 17 proofs; record in `docs/src/boards.md`).
- [ ] `stm32f100` (48)
- [ ] `stm32f101` (46)
- [ ] `stm32f102` (25)
- [x] `stm32f103` (53) — scoreboard board; closed first: 51/51 behavioral, zero latch
      (`f103_closeout.rs` 11 tests + `fsmc` lib tests; record in `docs/src/boards.md`).
- [ ] `stm32f105` (45)
- [x] `stm32f107` (49) — scoreboard + fabric board; closed second: 47/47 behavioral, zero latch
      (audit-only close-out; record in `docs/src/boards.md`).
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
