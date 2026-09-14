# VioARM Roadmap Map

A reader's map over the full planning surface plus the forward work packages.
`roadmap.md` remains the authoritative plan; `cosim-roadmap.md` owns the
SPICE-coupling phases; `jit-roadmap.md` owns the translation engine; the
mdBook under `docs/src/` owns the user narrative. This file answers two
questions: where we stand, and what comes next for host-facing workflows —
waveforms, fault triage, field measurement, analog coupling, and embedding.

Agent protocol, measurement methodology, and milestone exit criteria live in
`roadmap.md` and are not duplicated here. One work package per
branch/commit-series; `cargo fmt --all && cargo clippy --workspace
--all-targets && cargo test --workspace` green before every commit;
`CHANGELOG.md` updated under `Unreleased` for user-visible work.

## Part A — where we stand (map of `roadmap.md`)

- M1 correctness debt (CAN filtering, hygiene, timer capture/compare): shipped.
- M2 peripheral depth (DMA completeness, USB beyond EP0, ETH PHY, SDIO blocks,
  H7 scope gate): handshake behavior shipped per B6; L1–L3 close the remainder
  (see M13).
- M3 verification gates (CI hardening, fuzzing, differential expansion,
  determinism audit): CI/fuzz/differential/determinism live; M14 locks them.
- M4 performance and scale (benches, fast-path equivalence): superseded by M7.
- M5 enterprise release (license tree-side, book, publish metadata, board
  freeze at 506 with golden snapshots): tree side done; publish gated.
- M6 cycle/peripheral framework (TRM CPI in `DWT CYCCNT`, RCC/TIM/DMA timing,
  cache latency, bench gates, all-chip tables): closed.
- M7 throughput (bench gate, dispatch tuning at 1.89x whole-step,
  translation-engine no-go turned go under `jit/`): closed; engine default-off.
- M8 complex-IP depth (USB host scheduler, H7 Ethernet rings, crypto
  behaviors, link-layer abstraction): shipped.
- M9 ecosystem (hook/trace API, multi-machine fabric, pytest + replay):
  shipped. Hooks live in `vioarm-core/src/hooks.rs`; fabric in
  `vioarm-core/src/fabric.rs`; goldens in `vioarm-core/src/replay.rs`.
- M10 A-profile exploratory: no-go recorded; spike example stays as the record.
- M11 dual-engine fast mode (`Machine::set_fast_quantum`, coarse device time
  with exact instructions/bus/counters): shipped with fidelity contract.
- M12 correctness tail + translation-engine docs close: CLI boots the correct
  core per family; `.w` shape locked by `tests/thumb2_w.rs`.
- M13 peripheral depth to firmware-relevant 100% (DMA errors/circular/FIFO,
  request router, host-ISO): shipped.
- M14 verification gates to lock (full fuzz, +8 differential programs,
  cross-sim + golden expansion): shipped per CHANGELOG.
- M15 physical fidelity (parametric clocks N1 shipped; timing hardening N2
  gated on bench proof; analog seam N3 shipped with full SPICE-master path in
  `cosim/` + `vioarm-cosim`; hardware-anchored validation N4 needs a bench).

Terminal surface today: `vioarm run/record/replay/trace/info/boards/docs/
debug/gdb` (`vioarm-cli/src/main.rs:1603`), `trace --format text/jsonl`
(`main.rs:18`), `--pin-log` pin sampling (`main.rs:1626`), `docs <topic>`
(`main.rs:2077`), interactive `debug` REPL (`main.rs:2103`).

## Part B — forward work (M16–M18)

### M16 — Observability: waveforms, backtraces, field measurement

#### WP-R1 Waveform export (`trace --format vcd`)

Goal: hardware engineers open simulation output in a waveform viewer without
a conversion script. Today `--pin-log` prints `t=<s> <pin>=<0|1>` sample
lines and `trace --format jsonl` streams retired instructions; neither is a
timed value-change file.

Files: `vioarm-cli/src/main.rs:18` (`TraceFormat`), `main.rs:2005/2030`
(`run_traced_jsonl`/`run_traced`), `vioarm-core/src/hooks.rs:21`
(`HookEvent`), `vioarm-core/src/machine.rs` (`step`/`run`).

Spec: add `TraceFormat::Vcd`; reuse the hook registry (instruction retire +
GPIO/USART/TIM peripheral events) and the `--pin-log` sampler as the signal
source; emit a minimal value-change dump (header with signal names/widths,
monotone timestamps from retired-step counts mapped through the board clock,
one entry per change — no duplicate rows). `--output` writes the file;
stdout stays text/jsonl-only so pipes keep working.

Acceptance: `vioarm trace fw.elf --format vcd --pin-log c13 --output out.vcd`
opens in a viewer with the LED toggle visible at the sampled steps; reruns
are byte-identical; empty-signal runs emit a valid header with zero rows;
full suite green.

Effort: 6ah.

#### WP-R2 Fault backtrace ring + `debug history`

Goal: cut HardFault triage from "re-run under GDB and guess" to "the halt
message shows the last N retired instructions". Today triage means re-running
with `--trace` and grepping; a deeply embedded fault loop floods stdout.

Files: `vioarm-core/src/machine.rs` (`step`, fault paths),
`vioarm-core/src/hooks.rs` (observe-only precedent),
`vioarm-cli/src/main.rs:2103` (`debug_repl`), `main.rs:2030` (`run_traced`).

Spec: bounded ring (default 256 entries, `const` capacity, no alloc per
step beyond the overwrite) of `(pc, opcode-halfwords, cyccnt)` recorded on
every retired instruction inside `Machine`, independent of hooks (zero-cost
when disabled behind a `Machine::set_backtrace(len)` opt-in; default on at
256 — one small copy per step, measured in R3). On any halt, `run`/`trace`/
`debug` print the last K entries (K = min(ring, 32)) as `pc opcode cyccnt`
lines after the halt cause. `debug` gains a `history [N]` command dumping
the ring without re-running.

Acceptance: a faulting firmware's `vioarm run` output ends with a 32-deep
backtrace whose final pc equals the faulting pc and whose predecessors
disassemble to the true path (asserted in e2e with an assembler-verified
branch-into-fault image); overhead with the ring on measured <2% in benches;
`history` in `debug` matches `trace` output for the same image.

Effort: 5ah.

#### WP-R3 Field benchmark subcommand (`vioarm benchmark`)

Goal: measure throughput on the machine in front of you, not in criterion
output. Benches live in `vioarm-core/benches/mips.rs` with baselines in
`BENCHMARKS.md`; field engineers and CI-on-new-hardware need one command.

Files: `vioarm-cli/src/main.rs:1603` (`Commands`), `vioarm-core/benches/
mips.rs` (kernels), `vioarm-core/benches/BENCHMARKS.md` (baselines/method).

Spec: `vioarm benchmark [--cycles N] [--board stm32f103]` runs the same
kernels as the bench group (bare NOP loop, board boot-to-halt, USART-TX
soak) through the release `Machine` (no dev-dependencies at runtime —
reimplement the two tightest kernels inline in the CLI, ~60 lines, driven
by `Machine::run`), prints per-kernel MIPS + wall time + `cyccnt`, and exits
nonzero on `--fail-under <mips>` for CI floors. Numbers go to stdout as
plain `kernel mips wall-ms` rows (scripts parse them; humans read them).

Acceptance: `vioarm benchmark --cycles 1000000` on the reference box lands
within 15% of `BENCHMARKS.md` stepper rows; `--fail-under` trips on a
deliberately throttled (`--fast 0` vs `--fast 32` is not throttling — use an
absurd floor) run; docs page added under `docs/src/`.

Effort: 4ah.

### M17 — Analog and live coupling

#### WP-R4 Shared analog bus (one voltage, many consumers)

Goal: one host-driven voltage must move every consumer coherently. Today
`Apin` (`vioarm-core/src/device/analog.rs:51`) is a point-to-point
millivolt endpoint: a value staged in `TXMV` reaches one peer `RXMV` over
`Fabric::link_analog`. Firmware that samples the same node through ADC
while a comparator watches it and a GPIO thresholds it sees three
independent stores, not one node.

Files: `vioarm-core/src/device/analog.rs`, `vioarm-core/src/device/stm32/
adc.rs`, `device/stm32/gpio.rs` (+ `gpio_f4.rs`), `device/stm32/exti.rs`,
`device/stm32/timer.rs` (external-clock path), `vioarm-core/src/fabric.rs`
(`link_analog`).

Spec: add a host-owned shared bus object (normalized millivolts, exact `u32`
integers, no physics — same stance as `Apin` docs): ADC channel mux reads
the bus value at conversion-complete, the comparator derives its output from
the bus pair with its hysteresis band, GPIO/EXTI apply the existing
Schmitt-threshold path against the bus, and the timer external-clock input
observes edges through the GPIO bit it already samples. Single-driver per
net by convention (documented, matching the fabric's lowest-index-wins
discipline). `Apin` stays as the transport endpoint; the bus is what
multiple `Apin`-fed consumers read.

Acceptance: e2e with real CMSIS firmware — host ramps one bus value, ADC
result tracks it, comparator output flips at the threshold, GPIO input bit
follows, all three observed in one run and rerun byte-identical; existing
`analog_pair` loopback/determinism tests green.

Effort: 10ah.

#### WP-R5 Live-coupling contract + machine-driven bridge mode

Goal: write down the timing contract so external engines couple without
reading our source, and support the direction we lack. Today coupling is
SPICE-master only (`cosim/` shim + `vioarm.cm` bridges, `cosim-roadmap.md`
P0–P5): the analog side advances time and pulls the machine. Long digital
soaks with sparse analog events want the reverse — the machine runs ahead
in bounded quanta and publishes timestamped pin changes.

Files: `cosim/`, `vioarm-cosim/src/lib.rs`, `vioarm-cosim/src/handle.rs`,
`vioarm-core/src/fabric.rs`, new `docs/src/coupling-contract.md`.

Spec: (1) Contract page: cycle-to-time mapping (board clock from `rcc.rs`),
quantum definition (max machine steps before a forced sync point),
timestamp rule (every pin/analog event carries the exact `cyccnt` of its
retiring step), drift rule (events inside a quantum are never reordered;
the consumer interpolates or steps to the stamp — its choice, documented).
(2) Machine-driven mode: `step_duration(seconds)` gains a bounded-run
variant that retires up to the quantum, buffers pin/analog events with
stamps, and returns the event list with the halt cause; the existing
SPICE-master path is untouched. Single-producer (machine) /
single-consumer (bridge) queue; machine stays single-threaded, no threads
ever.

Acceptance: contract page reviewed against the implementation (every MUST
in the page has a test); machine-driven soak delivers the same final state
as the SPICE-master run for the `gpio_toggle` + `adc_dc` scenarios with
event stamps monotone; reruns byte-identical.

Effort: 12ah.

### M18 — Host surface: pins, catalog, embedding API

#### WP-R6 Pin-label catalog (`boards --pins`)

Goal: HIL wiring needs human pin names, not just SVD windows. `vioarm
boards` lists 21 tables with peripheral counts (`main.rs:2409`); nothing
maps `PA9` to `USART1_TX` on a given die, so every lab setup rebuilds that
table by hand from headers.

Files: `vioarm-core/src/board.rs` (`all_boards`, `all_chip_tables`),
`vioarm-cli/src/main.rs:2409` (`Boards`), `main.rs:2077` (`docs_topic`),
`docs/src/boards.md`.

Spec: static per-board pin-label table for the 13 full boards first
(`PA9: USART1_TX`, `PC13: GPIO_O`, …; GPIO alternate-function subset only —
the tables record the default AF mapping firmware actually uses, with the
full AF matrix explicitly out of scope and stated). `vioarm boards --pins
stm32f103` prints the table; `vioarm docs boards` points at it. Data is a
`const` table beside the board constructors (no new dependency, no SVD
schema change).

Acceptance: all 13 full boards carry tables; spot-checked against the
in-tree CMSIS headers (USART1_TX/RX, SWDIO/SWCLK, LED pins used by
`examples/`); unknown board errors with the valid list; book table updated.

Effort: 8ah.

#### WP-R7 Embedding API completion (snapshots, introspection, catalog)

Goal: embedders need five verbs the current C surface lacks. Today
`vioarm-cosim` exposes create/destroy/step/pins/analog/voltage/clock with
integer status codes (`lib.rs:73`) over `VioArmHandle` (`handle.rs:12`);
missing: API version, snapshot save/restore, CPU introspection, memory
peeks, and device/board enumeration.

Files: `vioarm-cosim/src/lib.rs`, `vioarm-cosim/src/handle.rs`.

Spec (additive only, existing symbols untouched): `vioarm_cosim_api_version`
(major/minor/patch struct), `snapshot_save`/`snapshot_restore` (opaque
sized buffer over core regs + RAM spans — same spans `record --snap`
digests, so snapshots and goldens agree), `get_pc`/`get_sp`/`get_cycles`/
`get_instructions`, `read_data`/`write_data` (data-space byte with bus-error
status, never panicking), `device_count`/`device_name` and `board_count`/
`board_name`/`board_clock`/`board_pin` (backed by `board::all_boards` +
R6 tables). Every entry keeps the crate's fail-closed discipline
(null-checks, `catch_unwind`, last-error string).

Acceptance: C test program (in `vioarm-cosim/tests/`, compiled in CI)
creates a board, snapshots, steps, restores, and asserts the restored pc/
RAM equal the snapshot; misuse probes (null handle, bad board name, OOB
read) return error codes instead of aborting; docs in `docs/src/embedding.md`.

Effort: 10ah.

#### WP-R8 Translation-engine execution proof on 64-bit ARM hosts

Goal: the translation backend must execute somewhere other than x86-64
before anyone trusts it there. The emitter is compile-checked for the
second host ISA but CI never runs it (no hosted runner), so backend bugs
can only be found by x86 developers reading ARM assembly.

Files: `vioarm-core/src/jit/*` (emitter + `tlb.rs` + `templates.rs`),
`vioarm-core/tests/jit_*.rs`, `.github/workflows/ci.yml`.

Spec: run the existing JIT differential + lib suites under user-mode
emulation on linux CI (`qemu-aarch64` + static target, no sysroot needed
for static binaries): full lib suite plus `jit_differential`,
`jit_flags`, `jit_tlb_thrash` must go green there. No behavior change is
expected — this WP is a test-leg addition plus whatever latent emitter
defects it exposes (file each as its own fix with a regression test, per
the agent protocol's triage rule). If the leg proves too flaky for gating,
it lands non-blocking with a tracking note, never silently skipped.

Acceptance: ARM-execution leg green in CI (or non-blocking with a dated
tracking issue and a local-repro recipe); any defect found ships with a
regression test; x86-64 results unchanged.

Effort: 6ah + per-defect fixes.
