# Kings end-to-end re-verification

Rebuilds the famous-firmware set from upstream sources and runs it on
release VioARM, checking functional gates (hard fail) and pinned cycle
counts (5% hard fail). This is the automation behind the trust-scope
rule that T3 claims lapse if re-runs stop passing.

## Layout

- `common.sh` — `KINGSWORK` scratch env (default `/tmp/kings`), compiler
  flags, `kings_link` (VioARM boots the 0x0 flash alias) and `kings_run` /
  `kings_uart` helpers. Sourced, never executed.
- `fetch.sh` — pins + clones all upstream sources (idempotent). Pins:
  Embench-IoT `09c2ed8`, mbedTLS `v4.2.0`, flatbuffers `v25.9.23`;
  CoreMark/FreeRTOS/ThreadX/TFLM/gemmlowp/ruy roll HEAD; Dhrystone is the
  netlib shar page, hash-verified on extract.
- `harness/` — clean-room board support (mine): `startup.s`, `board.h/c`
  (USART1 115200 + DWT CYCCNT + newlib syscalls), `board_tm4c.c`
  (UART0 115200 + DWT CYCCNT for tm4c123gh6pm @ 16 MHz PIOSC),
  `mem_f103.ld`, `mem_f107.ld`, `mem_tm4c.ld`. No upstream code. Two
  rules documented in `board.c`: `board_init()` before any
  output/timing, tuned newlib strings.
- `ports/<fw>/` — per-firmware port glue (mine) + `build.sh`. Upstream
  sources stay pristine; patches are build-time scripts with rationale.
  `ports/tm4c/` holds the Tiva retargets (e2e + Dhrystone + CoreMark
  40/400; the valid CoreMark run is 400 iterations — >=10 s at 16 MHz).
  A second board argument (`tm4c129`) retargets the same firmware to
  `tm4c1294ncpdt` (AHB-only GPIO in `board_tm4c129.c`, 1 MB/256 KB in
  `mem_tm4c129.ld`). `run_all.sh` re-runs the same ELFs tick-identical
  on all five Tiva boards (`tiva`/`tiva123pz`/`tiva123zrb` share the
  `tm4c` ELFs; `tiva129`/`tiva129x` share the `tm4c129` ELFs).
- `expected.sh`, `expected_embench.tsv` — pinned tables. Re-pin only
  with a full manual re-verification, never to silence CI.
- `run_all.sh` — builds everything, runs, checks, exits nonzero on any
  gate failure. Slow (~2h); weekly cron + manual dispatch, not per-commit.

## What lives outside this repo (and why)

Upstream firmware (GPL/MIT/Apache/BSD mixes) is fetched at run time,
never vendored — same rule as `packs/`. QEMU second opinions stay
manual (need machine-specific images + a debugger for triage).
