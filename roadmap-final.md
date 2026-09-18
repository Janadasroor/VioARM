# VioARM Embedding Roadmap (final)

Take `vioarm-cosim` from "works" to "embeddable product": any app — test
harness, classroom GUI, SPICE front-end, visualizer — drives the simulator
through the C ABI without touching Rust.

Relation to the other roadmaps: `roadmap.md` stays authoritative for the
core/milestone plan; `cosim-roadmap.md` owns the SPICE-coupling phases;
`jit-roadmap.md` owns the translation engine. This file owns only the
host-embedding surface (`vioarm-cosim/src/lib.rs`, `docs/src/embedding.md`).

Agent protocol (same as everywhere): one work package per branch/commit
series; `cargo fmt --all && cargo clippy --workspace --all-targets -- -D
warnings && cargo test --workspace` green before every commit;
`CHANGELOG.md` updated under `Unreleased` for user-visible work.

## Starting point (already shipped)

- Opaque `VioArmHandle`: create/destroy, `step_duration`, `run_quantum`
  (≤1-step overshoot), `cpu_hz`, pin get/set, external/operating voltage,
  DAC voltage read, `api_version`, snapshot size/save/restore, memory and
  catalog accessors, `last_error`.
- Proofs: `vioarm-cosim/tests/` suites (spin/pin/analog echo, truncation,
  bad-arg fail-closed, halt, rerun identity), CoreMark-via-CLI parity
  reference (2.067 CoreMark/MHz, CRCs e714/1fd7/8e3a).

## E0 — Lock the ABI contract (1–2 days)

- Freeze the rules in `docs/src/embedding.md` + SAFETY CASE: opaque handles
  only, no panics cross FFI, caller-owned out-buffers with explicit lengths,
  `VioArmApiVersion` bumped on *any* signature change.
- Finish the error taxonomy: distinct codes for ok, halted, bad-handle,
  bad-arg, bus-error, budget-exhausted, version-mismatch (+ `last_error()`
  string). No bare `-1`.
- Snapshot versioning: stamp `(api_version, core_version)`; `restore` fails
  closed on mismatch.
- Exit: rules doc merged; a test calls every function with a null handle
  and gets a clean error, never a crash.

## E1 — Complete the host-side API (1–2 weeks)

In priority order; each is additive, none breaks E0:

1. **Event drain** — pins/analog/UART output per quantum (internals exist
   via `run_quantum`; expose the drain).
2. **Input injection** — drive pins/analog/UART-RX between quanta (RX and
   analog counterparts to the existing `pin_set`, documented).
3. **Execution control** — reset-without-recreate, load-image-into-live
   handle, breakpoint set/clear + halt-reason query
   (breakpoint/fault/budget/watchpoint).
4. **Observation** — register read/write, memory read/write (align/fault
   behavior documented), cycle/instruction counters, backtrace fetch.
5. **Callbacks, observe-only** — host event callbacks with the same
   no-reentry discipline as the internal hooks; documented and enforced
   where cheap.
- Exit: the Phase-4 conformance suite (below) covers every function, and
  the two host examples from E2 build against the new surface.

## E2 — Packaging (1 week)

- Artifacts: `cdylib` + `staticlib` from the same crate; C header generated
  by `cbindgen` in CI, build fails on drift.
- Versioned soname (`libvioarm.so.1`); semver tied to `VioArmApiVersion`.
- Bindings: hand-written `ctypes`/`cffi` Python wrapper first (zero new
  deps, proves the ABI from a second language); PyO3 only on demand.
- `FindVioARM.cmake` + pkg-config file for C/C++ hosts.
- One minimal host example per language (C, Python): boot → step 1 ms →
  read pin → exit. If an example needs more than ~60 lines, the API is
  wrong — fix the API, not the example.
- Exit: clean-room build (container) produces lib + header + examples that
  run unmodified against the installed files.

## E3 — Concurrency and time contracts (3–5 days)

- **One rule:** a handle is single-threaded; the host owns locking.
  Document it; add a debug-mode owner-thread check that fails loudly on
  violation (never silently corrupts).
- **Time model doc:** quanta are the unit; same image + same input quanta
  = bit-identical output, across threads and processes.
- **Real-time guidance:** host-side rate-limiter example; no wall-clock or
  networking logic inside the library, ever.
- Exit: the multithread stress test (Phase 4) passes; the rate-limiter
  example holds a steady cadence against both a fast and a slow workload.

## E4 — Embedder-grade validation (1 week, run alongside E1)

- **C conformance suite** (gcc, CI): every function, every error code,
  null-handle probes, buffer-too-small, version mismatch, snapshot
  round-trip across versions.
- **ABI fuzz target**: random call sequences + random bytes; no
  crash/hang/leak under ASan + timeout.
- **Thread test**: N threads × M handles, then N threads on one handle
  under a mutex — must match single-threaded goldens.
- **Golden through the C ABI**: CoreMark-via-ABI diffed against the CLI
  result (CRCs e714/1fd7/8e3a, score within determinism tolerance). If they
  ever disagree, the embedding layer lied — stop and fix.
- Exit: all four suites green in CI on every push.

## Explicit non-goals

No GUI, no callbacks into JIT internals, no stable Rust API (the C ABI is
the stability surface), no wall-clock or networking inside the library.

## Order and sizing

E0 → E1 → E4 (validate as you extend) → E2 → E3. Roughly 4–6 weeks for one
person. Suggested commit series: `E0 abi-contract`, `E1 events+inject`,
`E1 exec-control`, `E1 observe+callbacks`, `E4 conformance+fuzz`, `E2
packaging+examples`, `E3 threading+time`.
