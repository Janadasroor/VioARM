# TI audit: 129-native blocks (SPMS433B §§8–12, §20, uDMA ch.)

Source PDF: `datasheets/spms433b.pdf`.
Date: 2026-09-25. Status legend: `[ ]` open, `[x]` fixed + regression-tested.

## EMAC (`vioarm-core/src/device/ti/emac.rs`, §20)

- [x] `emac.rs:1104` TDES0 IC[30]/TI: always set `TI`; Table 20-2 `TI` only if
      `IC` set — **wrong-behavior**
- [x] `emac.rs:1222` RDES1 DIC[31]/RI: always set `RI`; Table 20-10 DIC
      suppresses `RI` — **wrong-behavior**
- [x] `emac.rs:131` TDES0 IPE[12]: no `IPE` const/never set; Table 20-2 `IPE`
      on payload-len mismatch, ORed into `ES` — **missing-latch-needed**
- [x] `emac.rs:1009` TDES1 SAIC[31:29]: ignored; Table 20-3 SA insert/replace
      + auto-CRC — **missing-latch-needed**
- [x] `emac.rs:1098,1203` TDES6/7,RDES6/7/ATDS: write extended words
      unconditionally; §20.3.2.5 DES4-7 valid only if `ATDS=1` —
      **wrong-behavior**
- [x] `emac.rs:1139` RDES0 DE[14]/FL[29:16]: truncate to `RBS1` but `FL`=full
      len, never `DE`; Table 20-8 `DE` on truncation, `FL` valid only if
      `DE/OE` clear — **wrong-behavior**
- [x] `emac.rs:527` RDES0 Table 20-9: emit `(FT=1,HERR=1,PERR=0)` on
      bad-header/good-payload; Table 20-9 has no such row (reserved) —
      **wrong-behavior**

## CRC/CCM (`vioarm-core/src/device/ti/ccm.rs`, §12)

- [x] `ccm.rs:139` CRCCTRL INIT[14:13]: latch only, never self-clear on
      `CRCDIN`; Register 1 INIT self-clears with first `CRCDIN` write —
      **missing-latch-needed**

## Flash129 (`vioarm-core/src/device/ti/flash129.rs`, §8)

- [x] `flash129.rs:143` FMC2/FWBVAL: protected `WRBUF` sets `ARIS` but keeps
      `FWBVAL`; Register 8 violation also clears `FWBVAL` — **wrong-behavior**
- [x] `flash129.rs:55` FMC/FMC2 WRKEY[31:16]: accept only `0xA442`; FMC/FMC2
      allow `FLPEKEY` when `BOOTCFG.KEY=0` — **missing-latch-needed**
- [x] `flash129.rs:143` FMC2 FMA alignment: no `FMA[6:0]==0` check; §8.2.3.11
      requires 32-word aligned `FMA` — **missing-latch-needed**

## uDMA (`vioarm-core/src/device/ti/udma.rs`, ch. 9)

- [x] `udma.rs:66` CHIS `0x504`: synthesized per-channel done register; Table
      9-13 `0x504` reserved gap (`0x500` CHASGN → `0x510` CHMAP0), completion
      via `dma_done`/vectors Table 9-6, error via `ERRCLR` — **wrong-behavior**
- [x] `udma.rs:277` DMAENASET auto-clear: `ena` left set after
      BASIC/AUTO/PINGPONG; §9.2.5/§9.3.2.3 auto-clears on completion —
      **missing-latch-needed**
- [x] `udma.rs:223` BASIC/SWREQ/ARBSIZE: always full `N=XFERSIZE+1`; §9.2.6.2
      BASIC on SW request transfers only `ARBSIZE` items — **wrong-behavior**
- [x] `udma.rs:177` DMAALTBASE `0x00C`: writable fallback `CTLBASE+0x200`;
      Register 7 `RO` reset `0x200` — **wrong-behavior**
- [x] `udma.rs:83` CHCTL writeback `0x3FFF`: clears `NXTUSEBURST[3]` with
      mode+N; §9.2.5 updates only `XFERSIZE+XFERMODE` — **cosmetic**
- [x] `udma.rs:276` XFERMODE STOP: inert, leaves channel enabled; §9.2.6.1
      Stop disables channel if enabled — **missing-latch-needed**
- [x] `udma.rs:348` DMAWAITSTAT `0x010`: returns `0`; Register 8 `RO` reset
      `0x03C3CF00` — **cosmetic**

## GPIO 129 deltas (`vioarm-core/src/device/ti/gpio.rs`, §10)

- [x] `gpio.rs:85` Table cite: was Table 10-7; the specials table is Table
      10-6 p756 — **cosmetic** (cite fixed; values unchanged)
- [x] ~~`gpio.rs:106` GPIOR/S/T defaults: extra ports~~ — INVALID FINDING:
      `R`/`S`/`T` are real xnczad-class peripherals (own SVD tables in
      `tm4c129xnczad.rs`, mounted by that donor); §10.5 A–Q covers the
      1294 die only. Kept, comment clarified.
