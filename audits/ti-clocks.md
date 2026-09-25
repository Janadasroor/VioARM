# TI audit: clocks / System Control (SPMS376E §5, SPMS433B §5)

Source PDFs: `datasheets/spms376e.pdf`, `datasheets/spms433b.pdf`.
Date: 2026-09-25. Status legend: `[ ]` open, `[x]` fixed + regression-tested.

Verified correct (no action): 123 Tables 5-4/5-5/5-6/5-7 citations; DID1
`0x10A1.606E`, DC0–DC9, RCC/RCC2/GPIOHBCTL/DSLPCLKCFG/SYSPROP/PIOSCSTAT/PLLFREQ
resets; DID0 MINOR=A1 is the documented model (datasheet `-` varies).

## TM4C123 (`vioarm-core/src/device/ti/sysctl.rs`)

- [x] `sysctl.rs:97` RIS.PLLLRIS/PLLSTAT lock timing — we count 128 ticks for
      every power-up; §5.2.5.8: TREADY counter `0x200` on power-up, `0xC0` on
      M/N (=XTAL) change, HW holds old clock until met — **wrong-behavior**
- [x] `sysctl.rs:321` PLL relock trigger — we only countdown on power-down→up;
      XTAL-value change also relocks, already-locked stays locked in our
      code — **wrong-behavior**
- [x] `sysctl.rs:254` SYSDIV < MINSYSDIV — we apply as-written (comment
      admits); RCC p254: if `SYSDIV<MINSYSDIV` (DC1 p427 = `0x2`) and PLL
      used, MINSYSDIV is substituted — **wrong-behavior**
- [x] `sysctl.rs:270` SYSDIV `0x0/0x1` + PLL — we give 200/100MHz clamped to
      80MHz; Table 5-4: `reserved` — **wrong-behavior**
- [x] `sysctl.rs:278` OSCSRC2 `0x4–0x6` — we map to HIB 32.768kHz; RCC2 p262:
      `0x4–0x6 Reserved` — **wrong-behavior**
- [x] `sysctl.rs:125` XTAL reserved (`0x00–0x05`, `0x1B+`) — we fall back
      16MHz; RCC p256: reserved/undefined — **cosmetic**
- [x] `sysctl.rs:101` LFIOSC — we 30,000Hz nominal; Table 24-16: Nom 33kHz,
      10–90kHz — **wrong-behavior**
- [x] `sysctl.rs:347` PR synthesis — we `PP & RCGC` immediate; PR pages p403ff:
      cleared on RCGC *or* SR `0→1` change, set only after powered/enabled/
      reset, plus §5.2.6 3-clock delay — **missing-latch-needed**
- [x] `sysctl.rs:222` legacy `0x100–0x128` aliasing — we treat RO/ignore;
      §5.2.6 Important: legacy write also writes peripheral-specific bits
      (and vice-versa read rules) — **missing-latch-needed**
- [x] `sysctl.rs:382/414` RIS/MISC — we synthesize only PLLLRIS bit 6, MISC
      read always 0 and writes sink; p244/249: MISC is RW1C, reads `RIS&IMC`,
      W1C clears RIS (PLLLMIS clears PLLLRIS); BOR/MOSCPUP/USBPLLL/MOF bits
      unmodeled — **wrong-behavior**
- [x] `sysctl.rs:225` RESC — we normal RW latch (software can set); p252–253:
      sticky, cleared by writing `0`, POR clears all others — **wrong-behavior**
- [x] `sysctl.rs:254` MOSC gating — we use XTAL freq whenever OSCSRC=0
      regardless of MOSCDIS/MOSCCTL; §5.2.5.2: MOSCDIS must clear + MOSC
      stabilize before select — **wrong-behavior**
- [x] `sysctl.rs:254` dependent-clock rules — we ignore: ADC needs PLL or
      16MHz, USB needs MOSC and SysClk ≥20MHz, EEPROM freeze during EEDONE
      (Fig 5-5 notes) — **wrong-behavior**

## TM4C129 (`vioarm-core/src/device/ti/sysctl129.rs`)

- [x] `sysctl129.rs:244/56` NEWFREQ — `RS_NEWFREQ` defined but never used; we
      lock on any RSCLKCFG/PLLFREQ write with USEPLL&PWR; p236–238: PLLFREQ0/1
      deferred until NEWFREQ=1 (R0/W, auto-clear), change only while
      powered-down / not sourcing SysClk — **missing-latch-needed**
- [x] `sysctl129.rs:218/114` MEMTIM0/MEMTIMU — we latch MEMTIM0 immediately;
      p277 + RSCLKCFG bit31: applied only on MEMTIMU=1 (R0/W auto-clear),
      FWS/EWS/FBCHT/EBCHT wait states (Table 5-12) never modeled —
      **missing-latch-needed**
- [x] `sysctl129.rs:56/218` RSCLKCFG bits 31/30 — we sticky-latch MEMTIMU/
      NEWFREQ via `merged`; datasheet: R0/W, HW auto-clears, read 0 —
      **wrong-behavior**
- [x] `sysctl129.rs:90` LFIOSC — we 30,000Hz; §5.2.5.1/Table 5-3 note: 33kHz
      nom (10/90) — **wrong-behavior**
- [x] `sysctl129.rs:95/157` MOSC — we fixed 25MHz always ready; p273–274:
      reset PWRDN=1+NOXTAL=1, OSCRNG range, must clear PWRDN/NOXTAL and
      stabilize before select; crystal can be 5–25 (PLL) / 4–25 (bypass),
      board-dependent — **wrong-behavior**
- [x] `sysctl129.rs:157` PLLSRC — we map `0x2`→LFIOSC via shared `osc_hz`;
      RSCLKCFG p276: PLLSRC only `0x0`=PIOSC, `0x3`=MOSC, `0x1–0x2`
      reserved — **wrong-behavior**
- [x] `sysctl129.rs:157` OSCSRC `0x1/0x5+` — we fall back PIOSC; RSCLKCFG
      p276: reserved — **cosmetic**
- [x] `sysctl129.rs:207` CAN0MPC `0x29C` — we RO via
      `(0x298..=0x2A0)&&%4==0`; Table 5-11: CAN0MPC RW `0x0000.0003` —
      **wrong-behavior**
- [x] `sysctl129.rs:218` PWRTC `0x060` — we normal latch; Table 5-11: RW1C —
      **wrong-behavior**
- [x] `sysctl129.rs:307/334` RIS/MISC — same as 123: only PLLLRIS
      synthesized, MISC sinks, `pending()` stays set after MISC clear —
      **wrong-behavior**
