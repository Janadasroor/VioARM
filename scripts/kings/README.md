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
  (USART1 115200 + DWT CYCCNT + newlib syscalls), `mem_f103.ld`,
  `mem_f107.ld`. No upstream code. Two rules documented in `board.c`:
  `board_init()` before any output/timing, tuned newlib strings.
- `ports/<fw>/` — per-firmware port glue (mine) + `build.sh`. Upstream
  sources stay pristine; patches are build-time scripts with rationale.
- `expected.sh`, `expected_embench.tsv` — pinned tables. Re-pin only
  with a full manual re-verification, never to silence CI.
- `run_all.sh` — builds everything, runs, checks, exits nonzero on any
  gate failure. Slow (~2h); weekly cron + manual dispatch, not per-commit.

## What lives outside this repo (and why)

Upstream firmware (GPL/MIT/Apache/BSD mixes) is fetched at run time,
never vendored — same rule as `packs/`. QEMU second opinions stay
manual (need machine-specific images + a debugger for triage).
