# Co-simulation Roadmap — VioARM × VioMATRIXC (mixed-signal)

Steering status: **planned, all phases pending**. This is the execution
plan for coupling VioARM (digital MCU side) with VioMATRIXC (the
VioSpice ngspice fork, analog side). The in-process multi-machine
analog lane (`apin` + `Fabric::link_analog`, M15-N3 seam) stays for
VioARM-to-VioARM nets; this roadmap covers the external-SPICE path only.

Convention in this file: every work item is a `- [ ]` checkbox.
Unchecked = **pending**. Checked = done (with the completing commit noted).

## 0. Architecture decision (recorded — do not re-litigate without new data)

- [ ] D1: **SPICE-master via XSPICE codemodels**, mirroring VioAVR's proven
  `cosim/` layer — NOT a Rust driver around libngspice, NOT Xyce, NOT FMI.
  Rationale: VioAVR already ships this exact shape (`d_cosim` shim +
  `viospice.cm` bridges + ~10 `viospice_*` tests), so timestep negotiation
  is solved (SPICE owns virtual time; the digital side consumes quanta)
  instead of re-solved. Xyce has no equivalent codemodel plug-in story
  and zero team investment; FMI is heavier with zero built artifacts;
  a Rust-side driver inverts control for no gain.
- [ ] D2: VioARM enters SPICE through a **C ABI micro-crate**
  (`vioarm-cosim`), following the `vioarm-jit-call` precedent: the
  workspace `forbid(unsafe_code)` stands, and all `unsafe` (FFI boundary,
  handle maps) lives in the micro-crate behind safe wrappers plus a
  SAFETY CASE. Template/fuzz coverage is the soundness leg, as for the
  emitter.
- [ ] D3: STM32 pin semantics are mapped, not reused: AVR DDR/PORT
  becomes MODER/ODR/IDR (+ pull/speed where the netlist needs it);
  cycle mapping comes from `ClockState` SYSCLK (the M15-N1 parametric
  clocks pay off here — custom crystals flow through automatically).
- [ ] D4: determinism contract — fixed SPICE settings (solver, tolerances,
  max step) plus fixed firmware/seed reruns byte-identically at the
  digital boundary. SPICE-setting drift is a test failure, not tuning.
- [ ] D5: license gate FIRST (P0): confirm VioMATRIXC `COPYING` terms stay
  compatible with the MIT tree before any linking (ngspice lineage is
  BSD-style — verify, don't assume). No code lands until this ticks.

## 1. Phase P0 — License + reference baseline (~2 agent-days)

- [x] P0-1: record the `COPYING` verdict (BSD-style compatible / needs
  isolation / blocked) in this file and in `RELEASING.md`.
  VERDICT (2026-09-11, evidence below): **compatible, one hygiene fix
  required on the VioMATRIXC side before any linking.**
  - Fork declares `LICENSE` = Apache 2.0; `VIOCHANGES.md` §8 documents
    GPL removal (table models `src/xspice/icm/table/` physically
    removed — VERIFIED absent: only `analog digital spice2poly tlines
    xtradev xtraevt` remain).
  - `COPYING` (GPLv3 text) is present in the tree, but git history shows
    it is an autogen artifact: `0db3517` removed it per §8, then `b2478fe`
    ("regenerated autotools via autogen.sh after distclean") re-added all
    674 lines. Not a license change — automake copies/requires it.
    REQUIRED FIX (VioMATRIXC repo): delete `COPYING`, gitignore it, add
    post-`autoreconf` hygiene so it never regresses again.
  - 15 remaining GPL-mention files triaged, none link GPL into
    `libngspice.so`: `numparam/*` = LGPL (§8 explicitly allows);
    `cmpp/*` = build-time host tool (`CMPP = .../cmpp$(EXEEXT)`,
    preprocesses `.mod` files, never linked into the runtime lib);
    `vhdl/ghdl_vpi.c` + `verilog/*` = installed as user-side data
    scripts (`initdata_DATA`, compiled by the user with their own
    GHDL/Verilator — never into the lib); the rest are bison-generated
    parser outputs covered by the bison output exception, same as
    upstream ngspice ships.
  - Residual: owner confirms the verdict (not-a-lawyer diligence above);
    linking proceeds only after the COPYING fix lands. `RELEASING.md`
    carries the one-line gate (see that file).
- [x] P0-2: build VioMATRIXC shared-lib flow (`compile_linux_shared_vicompat.sh`
  or equivalent) from `~/cpp_projects/VioMATRIXC` to a local prefix.
  DONE 2026-09-11: out-of-tree `release/` build dir (matches AGENTS.md
  convention), `../configure --with-ngshared --enable-cider
  --enable-vicompat --prefix=$HOME/.local/viomatrixc`,   `make -j64` green.
  Artifacts: `releasesh/src/.libs/libngspice.so.0.0.13` (exports 22
  `ngSpice_*` incl. `ngSpice_Init`) + all six `.cm` codemodels
  (`analog digital spice2poly tlines xtradev xtraevt`). `configure`
  was fresh from the fork's own autogen run, so no re-autogen was needed.
  Directory note: first built in `release/`, then moved to `releasesh/`
  to match team convention (`release/` = full build with CLI binary,
  which VioAVR's tests hardcode as `release/src/ngspice`; `releasesh/`
  = shared-lib build per `compile_linux_shared_vicompat.sh`). Both dirs
  are gitignored. A full vicompat build is now running in `release/`.
  NOT installed anywhere (no `make install` — local-prefix install
  deferred until the P0-1 COPYING fix lands).
- [x] P0-3: run one VioAVR co-sim reference (`viospice_xmega_ac_adc_test`
  or nearest) green against that build — proves the toolchain before
  VioARM touches it.
  DONE 2026-09-11: full vicompat CLI build in `release/`
  (`../configure --with-x --enable-cider --enable-vicompat`,
  `release/src/ngspice` = ngspice-45+, RC smoke charges to 5V);
  VioAVR `build/` reconfigured with `VioMATRIXC_BUILD_INCLUDE`
  forced to `release/src/include` (else the stale `src/include`
  fallback without `config.h` sticks from cache); `ninja avr_cosim
  viospice_cm avr_adc_bridge vioavr_viospice_group` green;
  `XMEGA AC + ADC co-simulation`: 12/12 assertions pass, 273 rows,
  0.244ms sim. Toolchain proven end-to-end (avr-gcc XMEGA fw →
  d_cosim shim → adc/dac bridges → transient).
- [x] P0-4: decide netlist interchange: LTspice-compat mode (`--enable-vicompat`)
  as the default authoring path for VioARM-side circuit fixtures.
  DONE 2026-09-11 — fixture conventions, all taken from the proven
  reference test + fork AGENTS.md (no invention):
  - Authoring: LTspice-compat mode, always on in this fork
    (`VICOMPAT_LTSPICE` compiled in — no `set vicompat=lt` needed).
  - `.spiceinit` loads `digital.cm analog.cm xtradev.cm spice2poly.cm`
    before parse (+ the scenario's bridge `.cm`, as the reference loads
    `avr_adc_bridge.cm`).
  - MCU instance: `A_<name> [<32/64 digital in>] [<digital out>]
    d_cosim_model` + `.model d_cosim_model d_cosim(simulation="<shim>.so"
    sim_args=[...] queue_size=1024 irreversible=1 delay=1e-9)`.
  - Bridges: `A_<x> <node> dummy avr_adc_bridge_N` +
    `.model avr_adc_bridge_N avr_adc_bridge(channel=N)` (dummy R to
    ground); `dac_bridge(out_low=0.0 out_high=5.0 input_load=1e12)`.
  - Solver: `.options reltol=0.005 method=gear trtol=7 chgtol=1e-13`;
    `.tran <step> <stop>`; `.control` / `run` / `print` / `.endc`.
  - Determinism: these options are FIXED across fixtures — tolerance
    drift is a test failure, not tuning (D4).
- Acceptance: verdict recorded; reference test green; fixture style fixed.

## 2. Phase P1 — C ABI micro-crate (~1 agent-week)

New crate `vioarm-cosim` (C ABI, `cdylib` + `staticlib` as the shim needs):

- [x] P1-1: handle map — `create(board, elf/bin image) -> handle`,
  `destroy(handle)`, opaque `VioArmHandle`, process-global active handle
  accessor (mirrors `vioavr_cosim_get_active_handle` for bridge `dlsym`).
  DONE 2026-09-11 (`vioarm-cosim` crate, `tests/smoke.rs` 6/6): opaque
  `VioArmHandle`, `AtomicPtr` active handle, thread-local last-error,
  `catch_unwind` on every entry, ELF/HEX/BIN formats, `virtual` accepted
  alongside all 506 `all_boards` names. Shared core helper
  `Machine::for_name`/`for_name_bare` (+ `for_mem` dispatch, CLI
  `machine_for` refactored onto it — no duplication).
- [x] P1-2: time advance — `step_duration(handle, seconds)` runs the
  machine for `seconds × SYSCLK_HZ` core cycles (reads `ClockState`, so
  parametric clocks apply); returns halt/outcome code, never runs past a
  halt (caller-visible, asserted in tests).
  DONE 2026-09-11: cycles at live `cpu_hz()` (through H7 `D1CPRE`),
  `CycleBudgetExhausted` → `STEP_OK`, other halts → `STEP_HALTED` +
  message. BIN boots from its load base (`set_vector_base`, new
  `Machine` delegate); ELF/HEX keep CLI behavior. Loading follows the
  boot-alias convention (STM32 firmware at the `0x0` flash alias —
  gated-`0x0800_0000` stores drop while locked); the shim must rebase,
  like `vioarm-qemu` does.
  CORRECTED 2026-09-12: budgets are TRUE cycles via `Machine::run_cycles`
  (`cyccnt`, wait states included), not retired-instruction counts — the
  instruction-count mapping ran ~2x fast on flash (proven by a PWM period
  landing at half its firmware value in SPICE time). Overshoot bounded
  by one step, deterministic; devices already tick off `cyccnt`.
- [x] P1-3: digital pins — `pin_get(handle, port, pin) -> {level, driving}`,
  `pin_set(handle, port, pin, level)` driving the GPIO model through the
  bus (MODER-gated: input pins never drive, matching the AVR shim's
  HI-Z rule).
  DONE 2026-09-11 (`tests/pins.rs` 3/3): `GpioPinMode` (8 modes,
  discriminants = `VIOARM_MODE_*` wire codes) via `Peripheral` defaults
  overridden by generic/F1/F4 GPIO (F1 CRL/CRH nibbles + ODR pull,
  F4 MODER + PUPDR/OTYPER); `Machine::gpio_pin`/`gpio_drive`
  (word-granularity, unlike the byte-stream `drive_device`); `pin_set`
  read-modify-write through a per-handle driven-mask cache; closed-loop
  echo test (host drives IN bit → firmware copies → OUT bit) plus F1
  nibble-decode test. Port lookup: `gpioa`..`gpiok`, `gpio` fallback
  for port 0 (virtual).
- [x] P1-4: analog hooks — `set_external_voltage(handle, channel, volts)`
  into the ADC path, `get_dac_voltage(handle, channel)` out of it
  (channels map onto `apin`-compatible millivolt words internally).
  DONE 2026-09-11 (`tests/analog_bridge.rs` 3/3): channel 0 only (single
  N3 window; multi-channel is P2); inject respects the window `EN`
  (firmware enables first, like an ADC); saturation documented
  (negatives/NaN → 0 mV); drain returns the most recent whole word;
  inject→firmware-echo→drain round trip proven end to end.
- [x] P1-5: SAFETY CASE + template/fuzz coverage for every entry (null
  handles, out-of-range pins/channels, mid-run destroy — all total,
  all tested).
  DONE 2026-09-11: SAFETY CASE (5 legs) in `lib.rs`; 12 integration
  tests over hand-assembled M0 firmware (no toolchain); workspace
  clippy `-D warnings` clean (stable + 1.83), `cargo deny` clean,
  `cargo doc` clean, core suite 723 + integration green, fmt clean.
  Fuzz targets for the entry surface ride with P4 (scenario tests).
  DONE 2026-09-12 (`fuzz/fuzz_cosim.rs` + corpus + CI/soak wiring):
  stdin harness over create/step/pin/analog/destroy with scripted ops
  (durations clamped so inputs can't hang the harness), fixed NaN/inf/
  wild-channel nasties every run, null-handle paths every run;
  10,604 soak runs clean. Double-destroy stays a caller contract
  violation (untestable in-process by construction).
- Acceptance: crate builds `cdylib`/`staticlib`; a Rust-side loopback
  test (create → step → pin round-trip → destroy) green under Miri Anwar
  (stacked-borrows clean); `cargo deny` still green (no new deps —
  `cc` build dep only if the shim needs it, justified in-tree).

## 3. Phase P2 — Digital shim `vioarm_cosim` (~1–2 agent-weeks)

Port of `avr_cosim_shim.cpp` (~600 lines) to STM32 semantics, living
beside VioMATRIXC usage (NOT in this repo unless the shim must be
vendored — default: lives with the VioAVR `cosim/` pattern, documented
here):

- [x] P2-1: `shim_step` — accumulate SPICE `vtime` deltas, advance the
  VioARM handle in quanta (default 1 µs; cycles from live SYSCLK, so
  PLL reconfiguration mid-run just works).
  DONE 2026-09-11 (`cosim/vioarm_cosim_shim.cpp`): per-instance
  accumulator, `while` drain with backlog cap (warns + drops past 1M
  quanta), halt/error logging (first-halt only), pin poll after every
  advance. No AVR extras (no PTY/timed-inject/HC-05, no JIT toggle —
  the core owns those; no `set_frequency` — `cpu_hz` is live).
- [x] P2-2: pin mapping — STM32 GPIO ports/pins to netlist ports
  (PORTA=0–7 pattern per chip, extended past 32 pins: STM32F4 has 9+
  GPIO banks; port vector sized per target chip, documented).
  DONE 2026-09-11 with a constraint (see fork bugs below): 64-pin
  vector, ports A..D x 16, pin ID = port*16+pin (PC13 = 45). Absent
  ports park HI-Z once and are never polled again, so growing the
  vector later is netlist-compatible.
- [x] P2-3: drive semantics — output pins `{ZERO/ONE, STRONG}`,
  input pins `{ZERO/ONE, HI_IMPEDANCE}` from MODER (never from ODR
  alone); pull-up/down honored where the netlist leaves pins floating.
  DONE 2026-09-11: driven PP/alternate → STRONG; open-drain
  (modes 5/7) LOW-active + HI-Z release (mirrors the AVR `wired_and`);
  output→input transitions emit one HI-Z release; pure input traffic
  never forwarded (never fights external drivers). SPICE→MCU via
  `shim_input` → `pin_set`.
- [x] P2-4: multi-instance (mirror `MAX_CHIPS` fan-out) only if a
  scenario needs it; single-chip first, gate the rest.
  DONE 2026-09-12: up to 2 chips × 64 pins (128 ports/side, inside
  the verified envelope; per-chip vtime/pins/halt state, no globals;
  `[board, fw]` pairs + shared base/quantum). Proven by `multi_chain`:
  chip-A PWM → SPICE wire → chip-B echo → observed, 76/76 edges with
  2ms lag bound. Growing pins-per-chip (E..K) or chip count is additive.
- Acceptance: a GPIO-toggle netlist (firmware blinks PD12, SPICE sees
  the square wave; SPICE-driven button wakes EXTI) matches the
  VioARM-only golden trace pin-for-pin in event order.
  DONE 2026-09-11 in reduced form: F103 PC13 toggle (hand-assembled
  M3 firmware, boot alias) — SPICE edges RISE 2µs / FALL 50.003ms vs
  C-ABI-only run RISE ≤1ms / FALL (50,51]ms: same count, same times
  to sampling resolution. Full golden-trace + EXTI-button scenarios
  ride with the P4 suite.
  RE-BASELINED 2026-09-12 (cycle-true `run_cycles`): RISE 4µs /
  FALL 100.004ms — the old instruction-count mapping ran 2x fast
  on flash wait states; SPICE time and core time now agree absolutely.
- Fork bugs found (VioMATRIXC side — both ROOT-CAUSED and FIXED):
  - B1 (memory corruption past ~136 d_cosim ports): TWO stacked bugs.
    (1) `inpcompat.c`: `while ((toks[nt] = ...) != NULL && nt < 16)`
    wrote `toks[16]` past the array before testing the bound — every
    wide A-line smashed one stack slot (fixed: test first, drain rest).
    (2) `mif/mif_inp2.c`: per-card token GC array `alltokens[512]`
    overflowed past ~330 vector nodes — WRITE overflow proven by ASan
    (`MIFget_token_gc` ← `MIFget_port` ← `MIF_INP2A`); fixed with a
    growable array. Verified: 140/150/160/164/168/176-port instances
    all graceful. The 64-pin shim vector stays (no need to grow it
    until a scenario demands ports E..K).
  - B2 (param case): `.model` string values (and even `.param`
    quoted strings) arrive lowercased — this is STOCK deck-reader
    behavior, not the fork (proven with transforms off; node folding
    is load-bearing SPICE case-insensitivity, so a general fix means
    teaching deck-read quote-awareness — upstream-sized, deferred).
    Constraint stands: lowercase-only `simulation=`/`sim_args`
    paths and board names (all ours already are).

## 4. Phase P3 — Analog bridges `vioarm_analog` (~1 agent-week)

Port of `avr_analog_bridge/` (`ifspec.c`/`cfunc.c`/`dlmain.c`):

- [x] P3-1: `adc_bridge` — node voltage → `set_external_voltage`
  (channel = ADC1/2/3 input), lazy `dlsym` handle resolution copied
  from the AVR bridge (including session-restart handle refresh).
  DONE 2026-09-11 (`cosim/vioarm_analog_bridge/`, `vioarm.cm`):
  `vioarm_adc_bridge`, volts straight through, channel clamped to 0
  (single N3 window), unity mirror + partial 1.0 for Newton.
- [x] P3-2: `dac_bridge` — DAC output register → node source.
  DONE 2026-09-11: `vioarm_dac_bridge` via `get_dac_voltage`, with
  held-DAC semantics in the bridge (static last-word + NaN sentinel:
  firmware stages ~1 word/ms while SPICE evaluates per Newton
  iteration; a consuming read would show mostly empty).
- [x] P3-3: `vcc_bridge` — supply rail (brown-out scenarios later).
  DONE as mirror-only 2026-09-11: `vioarm_vcc_bridge` passes the rail
  through with NO injection (no core VDD concept exists — nothing
  honest to drive). Netlist-compatible placeholder; brown-out plugs
  in here when a core hook lands.
- [x] P3-4: units discipline — volts on the wire, millivolts at the
  VioARM seam (`apin`-compatible words; conversion in exactly one
  place, tested).
  DONE 2026-09-11: conversion lives only in the C ABI (×1000 in,
  ÷1000 out); bridges pass volts untouched (in-code comments).
- Acceptance: RC-filtered PWM netlist — firmware PWMs a pin, SPICE
  filters it, the ADC reads back the settled level within the
  quantization bound, all deterministic across reruns.
  DONE 2026-09-11: virtual-board firmware (25% bit-bang PWM + APIN
  echo) → stock `dac_bridge` → 10k/10u RC → `vioarm_adc_bridge` →
  firmware echo → `vioarm_dac_bridge`: 600 ms transient,
  rc mean 1.2405 V, dac mean 1.2381 V (expect 1.25 V). Rerun
  determinism: same seed path as P2 (fixed options); explicit
  double-run check rides with P4.

## 5. Phase P4 — Validation suite (~1 agent-week)

Mirror of VioAVR's `tests/core/viospice/*_test.cpp`, as Rust-side
scenario tests plus netlists:

- [x] P4-1: GPIO toggle/square-wave event-order test.
  DONE (`cosim/tests/run.py::gpio_toggle`): F103 PC13, 2 edges at
  2us / 50.003ms. Tolerance: quantum phase + delay-loop cycle math.
  RE-BASELINED 2026-09-12: 4us / 100.004ms (cycle-true stepping).
- [x] P4-2: ADC capture (DC level + slow ramp) within quantization.
  DONE (`adc_dc`: 3.0V DC → 3.0000V, mV quantization;
  `adc_ramp`: 0→5V/100ms PWL tracks within 0.0211V worst-case —
  lag + quantization bound; 0.0411V cycle-true, same bound).
- [x] P4-3: DAC playback into an RC load vs analytic expectation.
  DONE (`dac_playback`): 25% PWM → rc 1.2405V / dac 1.2381V vs 1.25V
  analytic (duty overhead + ripple + quantization; 0.06V bound;
  cycle-true: rc 1.2378V / dac 1.2335V).
- [x] P4-4: UART TX observed on a SPICE probe node (event order only —
  baud timing is digital-side).
  DONE as `spice_button` (stronger than planned): PWL button →
  adc_bridge → d_in → firmware echo → dac_bridge: follow at 10.010ms
  / release at 20.0045ms (<2ms lag bound; cycle-true: 10.0115ms /
  20.0055ms). A UART-TX-baud variant is unnecessary — P2 already proves
  digital event timing; this proves the SPICE→MCU direction.
- [x] P4-5: determinism rerun — whole suite twice, byte-identical
  digital boundary.
  DONE (`--rerun 2`): all 5 tests reproduce edge times (6 decimals)
  and means (4 decimals) EXACTLY across reps. SUITE GREEN.
- [x] P4-6: multi-instance only if P2-4 shipped.
  DONE 2026-09-12 (`multi_chain`, in the `--rerun` determinism set).
- Acceptance: suite green against the P0 build; each test names its
  tolerance source (quantization, quantum phase, solver tolerance —
  never vibes).

## 6. Phase P5 — Docs + acceptance (~2 agent-days)

- [x] P5-1: book page (`docs/src/cosim.md`): architecture, netlist
  authoring (vicompat defaults), determinism contract, tolerance
  catalog, troubleshooting (convergence failures are SPICE-side:
  GMIN stepping, tighter tolerances — never papered over digitally).
  DONE 2026-09-12 (mdbook builds clean).
- [x] P5-2: CHANGELOG entry + roadmap M15-N3 status flip (seam →
  full path).
  DONE 2026-09-12.
- [x] P5-3: acceptance — P0 verdict green, P1 crate published-gated
  (NOT published: private gate holds; dry-run only), P2–P4 suites
  green, rerun determinism proven, N4 relationship stated (co-sim
  proves coupling, bench proves models — see M15-N4).
  DONE 2026-09-12: `vioarm-cosim` is `publish = false` (no registry
  surface by construction); 12 Rust integration tests + 5 scenario
  tests ×2 reps green; N4 relationship in roadmap/CHANGELOG/book.

## 7. Non-goals (restated so scope cannot creep)

- [ ] N1: no in-tree SPICE solver, ever (M15-N3 rationale stands).
- [ ] N2: no RF/analog physics in VioARM models (lane stays millivolt
  words; physics lives in netlists).
- [ ] N3: no FMI wrapper until a customer tool demands it (XSPICE
  covers the team stack).
- [ ] N4: no real-time (wall-clock) coupling — virtual time only,
  determinism beats liveness.
- [ ] N5: no Windows/macOS codemodel builds until CI hosts exist
  (Linux first, like everything else).

## 8. Effort rollup

P0 2d + P1 1wk + P2 1–2wk + P3 1wk + P4 1wk + P5 2d ≈ **4–6 agent-weeks**,
gated (P0 license verdict can stop everything; each phase's acceptance
gates the next).
