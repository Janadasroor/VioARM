# Translated-block engine (JIT)

- Opt-in `jit` cargo feature (default off; non-JIT builds never pull
  `dynasm`): `dynasm-rs` x86-64 + AArch64 emitter behind a safe API
  (`vioarm-jit-call` owns the only `unsafe`, `forbid(unsafe_code)`
  elsewhere). CLI: `vioarm run/record/gdb --jit` (needs `--fast 0`).
- Translation blocks keyed `(PC, secure, ISA, MPU-epoch)` (≤64 ops,
  page-contained): linear scan over the shared decoders with seal rules
  (branches/calls/returns, system/exclusive/FP/DSP ops, bus writes,
  page crossings). Content checksums + per-entry write generations +
  config-epoch purges (self-modifying code, DMA, debugger writes, MPU
  reconfig all funnel through the bus chokepoints).
- Native execution: pinned regs (`r0`–`r3` + `xPSR` in callee-saved
  host regs, zero-copy `Core`), 64-entry direct-mapped TLB
  (`(vpn<<1)|priv` tags, runner-owned), guard fast paths with
  driver-re-executed side exits (MMIO, faults, complex tails — no
  double effects), static + single-target dynamic chained edges with
  per-call op/block budgets, emission groups (one finalize per static
  successor group).
- Contract (D7): `cyccnt` is approximate (per-TB TRM estimates +
  live flash wait states + taken-branch deltas; monotonic, device ticks
  stay sane), retired-op `cycles` and all architectural state exact.
  Quanta time model mirrors fast-coarse (run ~1024 ops, one device
  flush + one IRQ delivery, never early). ITSTATE-gated entry,
  sleeping cores never enter, hooks/GDB/replay/fabric surfaces below.
- Host surfaces: pre/post-instruction hooks + watchpoints bypass
  per-range (traces byte-identical incl. `cycles`); GDB breakpoints
  bypass their pages, single-step reports per-insn stops from TB
  metadata, continue runs quanta; semihosting/`BKPT` are callout tails;
  `Coverage` works unchanged; replay goldens carry a `jit` marker and
  compare state-only under JIT; fabric lockstep admits JIT members
  (`step_mixed`) with byte-identical reruns.
- When NOT to use it: timing-sensitive assertions or cycle-exact
  `cyccnt` reads (use accurate mode); store-heavy loops (PUSH/POP,
  SHA-256 schedule traffic run at/below stepper speed until split
  code/data write generations land); short runs (each cold block pays
  ~17µs translation — needs ~10k+ ops to amortize); IT-predicated,
  DSP, or FP-heavy code (mostly interpreted by design); hosts outside
  x86-64/AArch64 (no emitter; everything degrades to interpreted);
  minimal/MSRV/embedded builds (feature off, zero cost).
- Measure: `cargo bench -p vioarm-core --bench mips --features jit`
  (`tb/*` cold-4k pairs, board NOP 1M, SHA-256, ALU torture —
  `BENCHMARKS.md` holds the tables and the regression policy).
- Soundness trail: `jit-roadmap.md` (phases D1–D8/S0–S6/T1–T7, gates
  G1–G3 with stop clauses), differential suites (`jit_differential`,
  `jit_cycles`, `jit_fuzz` 600 seeded trials + `fuzz/fuzz_jit.rs` CI
  target, `jit_fp` 8280 FP edge comparisons, `jit_hooks`,
  `jit_coverage`, `jit_semihost`, `jit_fabric`).
