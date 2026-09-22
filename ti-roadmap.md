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
stays latch (zero-correct: no faults occur in simulation).

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

- [ ] TM4C_DFP in `fetch_packs.py`, hash-verified, PROVENANCE row added.
- [ ] TM4C123x board: audit zero latch, closeout proofs green.
- [ ] TM4C129x board: audit zero latch, closeout proofs green.
- [ ] CoreMark/Dhrystone pass on at least one Tiva board (T3).
- [ ] Full suite green + kings CI extended; trust-scope + boards docs updated.

## Estimates (Fermi, one focused engineer)

- P0–P1: ~1 week. P2: ~2–3 weeks. P3: ~1 week. P4: ongoing.
- **Total: ~4–5 weeks** to a closed Tiva track.
