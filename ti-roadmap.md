# TI Roadmap — Tiva C Series (TM4C123x, TM4C129x)

New vendor track, not an extension: the Cortex-M4F core transfers
directly, every peripheral is a new model. Method follows the STM32
playbook (SVD tables → hand-written behaviors → closeout proofs →
famous firmware), and the kings harness ports in days.

## Scope

- **TM4C123x first**: TM4C123GH6PM + EK-TM4C123GXL LaunchPad class board.
  Cortex-M4F @ 80 MHz, 256 KiB flash / 32 KiB RAM.
- **TM4C129x second**: TM4C1294 + EK-TM4C1294XL class board. Superset +
  Ethernet MAC/PHY; exercises ETH modeling against a second vendor IP.
- **Out of scope**: MSP430/CC25xx (non-ARM cores), CC13xx/CC26xx (value
  is the radio; RF/baseband out of scope by design), Hercules TMS570.

## Packs (Phase P0)

- `Keil::TM4C_DFP@1.1.0` (`Keil.TM4C_DFP.1.1.0.pack`, keil.arm.com):
  71 Tiva C devices, SVD + CMSIS headers + board examples. Same `.pack`
  zip convention as the STM32 DFPs — add one entry to
  `scripts/fetch_packs.py` (with SHA256 from the downloaded file) and a
  row in `packs/PROVENANCE.md`.
- TivaWare (BSD SDK: driver source + register headers) as the behavioral
  oracle, the role STM32 HAL/LL plays in verification. Never vendored;
  fetched at use time like all upstream sources.
- Acceptance: `fetch_packs.py --list` shows the entry; the `.pack`
  hash-verifies; SVDs extract for both target dice.

## Phase P1 — Ingestion (3–5 days)

- SVD → donor tables via the existing generator; board constructors:
  flash/RAM/memory map, boot at `0x00000000` (Tiva VTOR reset, no STM32
  BOOT alias), M4F core (already modeled).
- **Donor finding (verified 2026-09-23, pack SVD diff): the 51 TM4C123x
  dice are NOT one die.** 18 peripheral-name clusters; 238
  shared-peripheral register diffs (GPIO pin counts, UART/ADC instances,
  SYSCTL/FLASH_CTRL fields). 50 true own-SVD donors needed for full
  coverage. P1 wires exactly one die — TM4C123GH6PM (LaunchPad MCU),
  true own-SVD donor + board. The other 50 dice stay unwired (no
  wrong-donor aliases; cf. the WP4 F7/G0 lesson) until P2/P3 mints
  their donors one cluster at a time.
- **123 breadth (done 2026-09-23): 14 subset Dnames wired** (540
  total, anchor 38880): 14 dice proved GH6PM-subsets, wired via
  `removed` with inventoried residuals; 15/51 covered, 36 need new
  windows (`GPIOG`/`H`/`J`, `I2C4`/`5`).
- **123 second donor + sweep (done 2026-09-23): GH6PZ** (67 periphs,
  audit 65/67; 568 Dnames, anchor 40555): G/H/J/K rows, I2C4/5,
  data-driven 3rd comparator, 27 subsets wired; 43/51 covered, 8 BGA
  dice left.
- **123 third donor + family complete (done 2026-09-23): GH6ZRB** (72
  periphs, audit 70/72; 577 Dnames, anchor 41180): GPIO L/M/N/P/Q,
  8 BGA subsets wired; 51/51 TM4C123x + 20/20 TM4C129x = full Tiva C
  family enumerated (71 dice).
- **123 donors runnable + proven (done 2026-09-23)**: PZ/ZRB on the CLI
  `--board` list; same `tm4c` ELFs tick-identical (e2e 4/4, Dhry 9.124
  byte-identical, CM40 exactly 19397092 ticks, CM400 valid);
  `tiva123pz`/`tiva123zrb` gates + ZRB smoke test.
- `board_audit.rs` lists the new windows; pins entry.
- Acceptance: audit shows `tm4c123gh6pm` 0/58 behavioral, 58 latch,
  zero Reserved.
- **TM4C129 P1 (done 2026-09-22, pack SVD diff): the 20 TM4C129x dice
  form 12 peripheral-name clusters with only 10 within-cluster
  register diffs (flash-bank counts, EPHY presence) + 9
  varying-presence IPs (`EMAC0`, `LCD0`, `AES`/`DES`/`SHAMD5`,
  `ONEWIRE0`, `GPIOR`/`S`/`T`) — far cleaner than the 123 family.
  TM4C1294NCPDT (Connected LaunchPad MCU) minted as the first true
  own-SVD donor (64 periphs/2196 regs/7366 fields; 1 MB flash, 256 KB
  SRAM per PDSC) + wired Dname + latch board: audit 0/64 behavioral,
  zero Reserved. The other 19 dice stay unwired pending own-SVD
  donors. New 129-only windows (`EMAC0`, `EPI0`, `CCM0`, `I2C4`–`I2C9`,
  `TIMER6`/`7`, extra GPIO) are P2 modeling work; note the 129
  `SYSCTL` DID constants differ from the 123 set the shared model
  stamps today.
- **129 P2a (done 2026-09-22): native 129 SYSCTL** (`device/ti/
  sysctl129.rs`, 7 lib tests, mounted, audit 1/64): Table 5-11 stamps,
  129 PLL math with the 25 MHz LaunchPad MOSC modeled, 120 MHz proven.
- **129 P2b (done 2026-09-22): 129 GPIO** (reset-table parameter, 15
  ports mounted, audit 16/64).
- **129 P2c (done 2026-09-22): compatible fleet** (42 windows, audit
  58/64): UART/TIMER/SSI/I2C/WDT/ADC/PWM/QEI/CAN/USB/EEPROM/HIB/COMP
  mount unchanged; this die has no `WTIMER` windows; latch loop moved
  after behaviors (first-wins routing).
- **129 P2d (done 2026-09-22): native 129 flash** (`device/ti/
  flash129.rs`, 7 lib tests, mounted with the EEPROM link, audit
  59/64): `FMC2` buffered program, 16× protection, `PP`/`SSIZE` ID.
- **129 P3 (done 2026-09-22): closeout** (`vioarm-core/tests/
  tm4c129_closeout.rs`, 18/18): 129 natives + fleet on the real board.
- **129 T3 (done 2026-09-22): firmware proof** (`scripts/kings/ports/
  tm4c/` 129 builds): e2e 4/4, Dhrystone 9.124, CoreMark-40 golden CRCs
  (tick-identical to the 123 run), CoreMark-400 valid (12 s). Pins +
  gates in `expected.sh`/`run_all.sh` (weekly CI).
- **129X T3 (done 2026-09-23): second-board kings**: same 129 ELFs
  tick-identical on `tm4c129xnczad` (e2e/dhry/cm40/cm400 valid).
- **Second 129 donor (done 2026-09-22): TM4C129XNCZAD** (72 periphs,
  audit 62/72): maximal superset + full 129 mounts; 508 Dnames.
- **Full 129 family (done 2026-09-22): 20/20 Dnames** (526 total):
  every remaining die proved a peripheral-subset of XNCZAD and wired
  via `removed` (PDSC sizes); register residuals inventoried in
  `per_dname.rs` (Known limitations).
- **129 crypto 1/2 (done 2026-09-23): CCM CRC-32 + SHA-256** (3+3
  lib tests, XNCZAD audit 64/72): NIST-vectored; other types latch.
- **129 crypto 2/2 (done 2026-09-23): AES + DES** (5+4 lib tests,
  XNCZAD audit 66/72): FIPS/NIST-vectored ECB/CBC; advanced modes latch.
- **129 AES advanced modes (done 2026-09-24): CTR + GCM/GMAC +
  CBC-MAC** (9 lib tests, audit still 66/72 — no new windows): TI
  driverlib `AES_CFG_MODE_*` decode (`CTR 0x40` + widths,
  `CBCMAC 0x20008000`, `GCM 0x20030040`); GHASH via the NIST-proven
  STM32 primitives; byte-FIFO + 16 B staging so firmware chunking
  (incl. mid-stream AAD/data straddles) can't misalign blocks;
  `C_LENGTH`/`AUTH_length` partition streams, `TAG_OUT` finalizes.
  Python-oracle vectors (CTR-128/256, GCM zero + GMAC + straddle +
  trailing-partial, raw CBC-MAC). ICM/CFB/XTS/F8/F9/CCM still
  deferred.
- **129 AES CCM (done 2026-09-24): SP 800-38C/RFC 3610 framing** (6
  lib tests incl. RFC 3610 Packet Vector #1, audit still 66/72):
  `CCM 0x20040040` + `CCM_L_*`/`CCM_M_*`, `B0`/AAD-format/CTR/CBC-MAC
  state machine over `L` = 2-8, `M` = 4-16 (MAC covers PLAINTEXT per
  RFC §2.2, unlike GCM); malformed configs inert. Only ICM/CFB/XTS/
  F8/F9/DMA/`SAVE_CONTEXT` still latch.
- **129 AES residual modes (done 2026-09-24): ICM + CFB + XTS** (10
  lib tests, audit still 66/72 — no new windows): TivaWare driverlib
  `AES_CFG_MODE_*` values fetched at runtime (never vendored: `ICM
  0x200`, `CFB 0x400`, XTS `TWEAKJL 0x800`/`K2IJL 0x1000`/`K2ILJ0
  0x1800`, `F8 0x2000`, `F9 0x20004000`); ICM reuses the proven CTR
  primitive full-width (first keystream == ECB(IV) checked); full-block
  CFB with cross-completion chaining; XTS as one continuous tweak chain
  over all three load variants (128/256-bit, tweak from `IV_IN`, KEY2
  same interleave as KEY1) — the oracle caught the alpha-step byte
  order (block 1 matched, block 2 decided little-endian). Frozen oracle
  vectors (`aes_resid_vecs.json`: ICM/CFB/XTS enc/dec, carry wrap,
  variant agreement, XTS-192 inert). F8/F9 permanently latched (verdict
  2026-09-24: no TI-published construction — no AES-engine chapter in the
  datasheet, TivaWare names modes + key slots only; no oracle obtainable
  — no TivaWare examples/vectors, UMTS f8/f9 are KASUMI-based per TS
  35.201 and do not apply, LTE single-key shapes do not match
  KEY1/KEY2/KEY3; zero firmware demand; wrong crypto worse than
  latched); XTS-192/steal-tails/continuation reloads documented. Only
  F8/F9/DMA/`SAVE_CONTEXT` still latch.
- **129 LCD (done 2026-09-23): raster timing + done IRQs** (3 lib
  tests, XNCZAD audit 67/72): PL110-compatible frames, no pixels.
- **129 1-Wire (done 2026-09-23): master transactions** (4 lib tests,
  XNCZAD audit 68/72): presence + instant `WR`/`RD`/`WRRD`.
- **129 EPI (done 2026-09-23): external bus configs + FIFO** (3 lib
  tests, XNCZAD audit 69/72): no transfer engine (FSMC stance).
- **129 EMAC package 1 (done 2026-09-23): MAC shell + MII** (2 lib
  tests, XNCZAD audit 70/72): DMA/checksum/MMC/PTP deferred.
- **129 EMAC package 2 (done 2026-09-23): descriptor-ring DMA** (6
  lib tests + board TX proof): TX/RX rings via the shared mailbox;
  checksum/MMC/PTP still deferred.
- **129 EMAC package 3 (done 2026-09-24): checksum offload + MMC**
  (13 lib tests + 2 board proofs, audit still 70/72 — no new
  windows): TX `CIC` insertion (IPv4/TCP/UDP/ICMPv4/ICMPv6, `CIC=2`
  folds the input field, `CIC=3` full pseudo-header) with `IHE`/`ES`
  write-back; RX Table 20-9 status under `EMACCFG.IPC`; all eight
  32-bit MMC counters with `CNTRST`/`CNTPRST`/`CNTFREEZ`/`RSTONRD`,
   half/full interrupts + masks into `EMACRIS`. PTP/magic-packet/
   `CRCR`/`VLIC` still deferred.
- **129 EMAC package 4 (done 2026-09-24): PTP time + magic-packet wake
  + `CRCR`/`VLIC`** (16 lib tests + 1 board proof, audit still 70/72
  — no new windows): system-time generator (coarse `SSINC`/fine
  `TSAR`, `TSINIT`/`TSUPDT`/`ADDSUB`, digital/binary rollover,
  target-time `TT` IRQ + `TIMSTAT`), TX snapshots (`TTSE` →
  `TDES6/7` + `TTSS`) and RX snapshots (`ALLF`/PTP-select →
  `RDES6/7` + `RDES4` info + `RDES0[7]`), magic-packet + global-unicast
  wake with power-down gating (`WUPRX`/`MGKPRX` read-clear,
  `DMARIS.PMT` + `EMACRIS.PMT`), TX `DC`/`CRCR` FCS append/replace +
  `DP` runt padding (IEEE CRC-32), `VLIC` strip/insert/replace via
  `EMACVLNINCREP` (`VF` status, RX VLAN-presence bit). Frozen oracle
  vectors (`emac_vecs.json`: CRC-32 check, FCS append/replace, VLAN
  edits, magic pattern, PTP Sync). Still deferred: remote wake-up
  pattern CRC match (bank mechanics only), second-buffer scatter, DMA
  burst arbitration + `DMAIM` masking, IPv6 extension headers, `UCDBC`,
  PPS output, RX FCS validation.

## Phase P2 — Peripheral models (~2–3 weeks, the bulk)

New drivers, one per IP, each with lib tests + vectors where applicable:

- SYSCTL clocking (PLL 80 MHz math, peripheral ready bits).
- GPIO (commit control, drive strength, alternate functions).
- UART (baud divisor, FIFO, TX/RX interrupts).
- SSI (SPI modes, FIFO).
- I2C (master modes).
- GPTM (32/64-bit timers, PWM, capture).
- ADC sequencers (sample sequencers 0–3, FIFO, interrupts).
- Watchdog (WDT0/1, reset vs interrupt).
- USB OTG device (enumerates; reuse the USB-host lessons, device side).
- CAN (message objects; arbitration lessons transfer).
- Flash/EEPROM + μDMA (request-driven pacing per the F4 convention).
- TM4C129x extension: Ethernet MAC/PHY + larger memory map.

Each model lands with the standard treatment; TivaWare is the oracle,
QEMU has no Tiva machine — second opinions come from TivaWare examples
run as golden pairs where feasible, else VioARM-only locked tests.

Status 2026-09-22: P2 core set done — SYSCTL, GPIO, UART, GPTM, SSI,
I2C, WDT, ADC, PWM, QEI, CAN, USB, EEPROM, FLASH_CTRL, HIB, COMP
(audit 56/58 behavioral). `UDMA` stays latch by decision (a real engine
needs bus-master reads; behaviors own their window only) and `SYSEXC`
stays a proven latch (zero-correct: `sysexc_stays_quiet` on both family
closeouts — mask programs, clear takes writes, no faults occur).
- **TI UDMA (done 2026-09-24): μDMA engine** (12 lib tests + 1 board
  proof, all 5 boards +1 behavioral — `SYSEXC` is the last latch):
  TivaWare `hw_udma.h`/`udma.c`/`udma.h` oracle fetched at runtime
  (never vendored); 32 channels over the SRAM control table
  (`BASIC`/`AUTO` run to completion, `PINGPONG` alternates with
  `ALTSET` flipping, `STOP`/scatter-gather inert), end-pointer/N/INC
  arithmetic, `STOP`+`N=0` write-back, per-channel `CHIS` (pack SVD
  omits it — synthesized at the edge, write-mask clears) + `ERRCLR`
  behind the done/error NVIC lines, priority service order, instant
  transfers through the shared DMA mailbox (F4 request convention:
  `SWREQ` triggers). Frozen oracle vectors (`udma_vecs.json`).
  Deferred: scatter-gather task lists, arbitration pacing/`USEBURST`,
  `CHMAP`/`CHASGN` routing, `REQMASK` (SW bypasses), mid-transfer
  residuals, size mismatch (source width wins).

## Phase P3 — Boards + closeout (~1 week)

- 1–2 boards per family, audit to zero latch, `tiva_closeout.rs`-style
  direct-poke proofs per board. DONE 2026-09-22 for `tm4c123gh6pm`
  (`vioarm-core/tests/tiva_closeout.rs`, 17 tests, 56/58 behavioral).
- T3 firmware: CoreMark + Dhrystone retargets via `scripts/kings/`
  (new port dir, same method), plus a UART/GPIO/Timer e2e. DONE
  2026-09-22 (`scripts/kings/ports/tm4c/`): e2e 4/4 markers, Dhrystone
  22 self-checks + DMIPS 9.124 (= STM32 pin), CoreMark-40 golden CRCs
  `e714/1fd7/8e3a` at 2.062/MHz (= STM32 pin), CoreMark-400 valid run
  (`Correct operation validated`, 12 s >= 10 s floor). Tiva pins live
  in `expected.sh` (`DHRY_DMIPS_TM4C`, `COREMARK_MHZ_TM4C`) and ride
  the weekly kings CI via `run_all.sh` (P4).
- `boards.md` + `trust-scope.md` rows for the new boards.

## Phase P4 — Continuous proof

- New expected tables in `scripts/kings/`; the weekly CI job covers the
  Tiva binaries once pinned.
- `CHANGELOG.md` release entry when the track closes.

## Acceptance gate (definition of done)

Closed 2026-09-24 as `v1.11.0` (`CHANGELOG.md` TI entries drained into
the release). `UDMA`/`SYSEXC` latch by standing decision on every Tiva
board — a real μDMA engine needs bus-master reads (behaviors own their
window only) and no faults occur in simulation — so "zero latch" below
means zero *undecided* latch.

- [x] TM4C_DFP in `fetch_packs.py`, hash-verified, PROVENANCE row added.
- [x] TM4C123x board: audit 70/72 behavioral, closeout proofs green.
- [x] TM4C129x board: audit 70/72 behavioral, closeout proofs green.
- [x] CoreMark/Dhrystone pass on Tiva boards (T3): all five boards,
  tick-identical ELFs.
- [x] Full suite green + kings CI extended; trust-scope + boards docs updated.

## Estimates (Fermi, one focused engineer)

- P0–P1: ~1 week. P2: ~2–3 weeks. P3: ~1 week. P4: ongoing.
- **Total: ~4–5 weeks** to a closed Tiva track.
