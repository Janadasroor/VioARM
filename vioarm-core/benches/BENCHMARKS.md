# Benchmarks (M7-G1)

Method: `cargo bench -p vioarm-core --bench mips` (criterion, release).
Reference machine: Intel Xeon Gold 6130 @ 2.10GHz, `rustc 1.98.1`.
Step benches exclude setup (`iter_batched`); the means below divide into
per-step costs. Cycle assertions in each bench pin the simulated behavior,
so a perf change that alters timing fails loudly instead of silently.

## JIT summary (P3-5 consolidated table, x86-64)

Cold per-iteration means (fresh runner + machine; translation included)
unless noted. Detail sections below hold the history.

| Soak | Stepper | JIT | Ratio |
|---|---|---|---|
| ALU loop RAM 4k (`tb/*_ram_4k`) | 345–350 µs | ~34 µs | **~10** |
| ALU loop flash 4k (`tb/*_flash_4k`) | ~420 µs | ~35 µs | **~12** |
| mem-mix RAM 4k | ~377 µs | ~66 µs | **~5.7** |
| mem-mix flash 4k | ~440 µs | ~68 µs | **~6.5** |
| call/ret RAM 4k | ~503 µs | ~61 µs | **~8.3** |
| push/pop RAM 4k | ~500 µs | ~890 µs | **0.56** |
| board NOP 1M | 2.6476 s | 33.132 ms | **79.9** |
| SHA-256 loop RAM 4k | 838.42 µs | 1154.2 µs | **0.73** |
| ALU torture RAM 4k | 416.82 µs | 84.259 µs | **4.95** |
| ALU loop sustained (warm, per op) | 135 ns | 3.0 ns | **~45** |

Sub-unity soaks are store-fenced chaining gaps (one driver trip per
block; see the push note and the P3-4 section), not missed bugs — the
fix is split code/data write generations (G3 critical path). G3
acceptance: geometric mean ≥50× over board NOP 1M, SHA-256, ALU
torture on x86-64 release.

## Baselines (2026-09-04, pre-G2)

| Bench | Window | Mean | Per-op |
|---|---|---|---|
| `attrib/bare_nop_1k_miss` | 1000 bare steps, all decode misses | 195 µs | 195 ns/step |
| `attrib/bare_loop_100_hit` | 201 bare steps, cache-hot loop | 18.8 µs | 94 ns/step |
| `attrib/board_nop_1k` | 1000 f103 steps | 2.46 ms | 2.46 µs/step (~0.41 MIPS) |
| `attrib/bus_flash_read_1k` | 1000 flash halfword reads (f103) | 166 µs | 166 ns/read |
| `attrib/bus_h743_flash_read_1k` | 1000 flash halfword reads (h743) | 267 µs | 267 ns/read |
| `attrib/bus_flat_read_1k` | 1000 halfword reads, no bus | 3.4 µs | 3.4 ns/read |
| `attrib/bus_tick_1k` | 1000 `tick_devices(1)` | 1.24 ms | 1.24 µs/tick |
| `attrib/bus_next_exception_1k` | 1000 NVIC peeks | 164 µs | 164 ns/peek |
| `attrib/core_nop_1k` | 1000 core steps, no Machine wrapper | 213 µs | 213 ns/step |
| `mips/nop_loop_1k` | legacy aggregate | — | ~4.5 MIPS bare |

History: `board_nop_1k` was 4.96 ms pinned pre-G2 (worktree A/B) and the
flash reads 607 ns (f103) / 1.43 µs (h743) before the G2 bus index
(sorted windows + bounded walk, attach-order-first incl. overlaps); the
first index revision showed zero gain because overlaps took a linear
fallback — overlaps are common (shared SRAM windows), so the fallback
*was* the path. Lesson recorded: measure the fallback rate, not just
the happy path.

In-process A/B (same binary, flag flips per setup — immune to machine
drift), h743 flash reads, pinned cores:

| Path | Mean/1000 | Per read |
|---|---|---|
| index | 291 µs | 291 ns |
| linear scan | 1096 µs | 1096 ns |

Index effect: **3.77×** on reads. Whole-step effect (f103 board NOP):
4.96 ms → 2.62 ms pinned, **1.89×**. Remainder is the timing model
itself (per-step device ticks, drains, NVIC scans, wait-state accounting),
not dispatch — see G2 exit note in the roadmap. `set_route_force_linear`
stays as the A/B hook for future perf work.

G3 ceiling micro-experiment (2026-09-04, same box): the 2-instruction
loop hand-compiled to native Rust runs at ~100–160 GIPS vs ~10.6 MIPS
interpreted (cache-hot bare core) — but the interpreter's cost is model
work a JIT must preserve (MPU per access, fetch+decode twice for timing,
cyccnt, NVIC polling, tick boundaries), so a model-faithful JIT keeps an
estimated 70–80% of per-instruction cost: realistic ceiling 3–5× the
interpreter, against a backend dependency plus a permanent ×2
differential burden. G2 delivered 1.89× for ~200 safe lines. JIT: no-go
(see WP-G3).

## Attribution (board NOP step ≈ 4.47 µs)

| # | Cost | Size | Nature |
|---|---|---|---|
| 1 | `tick_devices` loop (~150 devices) | ~1.24 µs (28%) | semantic (timing model) |
| 2 | bus linear scan (fetch + timing re-reads) | ~1.1–1.6 µs (30%) | mechanical, precedence-preserving fast-path possible |
| 3 | machine-step fixed overhead (drains, clock mutex, 2 NVIC peeks, reset checks) | ~1 µs (22%) | partially mechanical |
| 4 | double decode + execute + MPU | ~0.3 µs (7%) | mechanical (cache/fusion) |
| 5 | decode miss vs hit delta | ~100 ns | already cached; BTreeMap lookup itself is next |

## Ranking for G2 (ROI × safety)

1. Bus range fast-path (sorted device-range table, first-match precedence
   kept) — biggest safe win.
2. `Behavior::tick_is_noop` opt-out — skip provably empty ticks (default
   stays ticking; each opt-out cites the empty body).
3. Direct-mapped decode cache (replaces the `BTreeMap`; same
   halfword-validation safety, ~15× cheaper lookup).
4. Timing fusion (feed the cached decode into `base_cycles`, killing the
   second fetch+decode) — most invasive, last.
5. NOT in scope: tick coalescing / event-driven simulation. Skipping ticks
   shifts interrupt timing visibly (traces change), so it is a timing-model
   decision, not a fast path. Documented for post-M7.

## Hook overhead (M9-I1, 2026-09-06)

Same 1000-step NOP window, `hooks` group (`cargo bench --bench mips -- hooks`):

| Bench | Window | Mean | Ratio vs none |
|---|---|---|---|
| `hooks/steps_1k_none` | no hooks | 219.7 µs | 1.00 |
| `hooks/steps_1k_armed_idle` | hooks armed, never-matching ranges | 225.0 µs | 1.024 |
| `hooks/steps_1k_trace_all` | full-range tracing hook + atomic count | 223.5 µs | 1.017 |

Acceptance (`idle / none < 1.10`) holds at 1.024: with no hooks armed the
cost is one predicted branch per step/access. (Full-range tracing costs
no more than idle-armed here — one registry entry plus an atomic
increment; real trace sinks like file I/O dominate, as expected.)

## Fast mode (M11, 2026-09-06)

Same 1000-step f103 window, `fast` group (`cargo bench --bench mips -- fast/`):

| Bench | Quantum | Mean | Ratio vs accurate |
|---|---|---|---|
| `fast/board_nop_1k_q0` | accurate | 2.84 ms | 1.00 |
| `fast/board_nop_1k_q8` | 8 | 1.63 ms | 1.74 |
| `fast/board_nop_1k_q32` | 32 | 1.46 ms | 1.95 |
| `fast/board_nop_1k_q128` | 128 | 1.42 ms | 2.00 |

Acceptance (best / accurate ≥ 1.5) holds at 2.00 (K=128); K=32 at 1.95
is the documented sweet spot (±32-step event accuracy). The floor (~1.4 ms)
is core+bus+memory — device dispatch was the prize, and coarse ticks took
it (see the tickexp probe notes in the M11 entry).

## JIT Phase 1 (P1-6/G1, 2026-09-07)

4096-op ALU soak loop, `tb` group with the `jit` feature
(`cargo bench -p vioarm-core --features jit --bench mips -- tb`);
cold cache per iteration (includes block builds):

| Bench | Mean | Ratio vs stepper |
|---|---|---|
| `tb/step_ram_4k` | 439.9 µs | 1.00 |
| `tb/quanta_ram_4k` | 345.7 µs | **1.27** |
| `tb/step_flash_4k` | 521.2 µs | 1.00 |
| `tb/quanta_flash_4k` | 455.5 µs | 1.14 |

G1 acceptance (≥1.2× on the soak) holds at 1.27 on the RAM loop — the
mechanism's target (the stepper has no decode cache off-flash, so TBs
remove a re-decode per op there). Flash shows 1.14: the stepper's
decode cache already covers re-decode, leaving dispatch + fetch as
the remaining prize (the emitter's target in Phase 2). Enabling runs:
mask-driven revalidation (P1-6), write-generation-gated entry checks,
single-probe lookup.

## JIT Phase 2 checkpoint (P2-2, 2026-09-07)

Same `tb` soak (ALU loop, now S0 + branch templates natively; n=30):

| Bench | Mean | Ratio vs stepper |
|---|---|---|
| `tb/step_ram_4k` | 397.0 µs | 1.00 |
| `tb/quanta_ram_4k` | 151.1 µs | **2.63** |
| `tb/step_flash_4k` | 499.7 µs | 1.00 |
| `tb/quanta_flash_4k` | 153.1 µs | **3.26** |

Not yet G2 (≥10× at end of Phase 2): remaining prizes are Mem
templates (P2-3), direct chaining (P2-4, removes per-block prologue /
epilogue + driver lookup on hot edges), and wider coverage (P2-5).
Trajectory 0.64 → 0.86 → 1.27 → 2.63/3.26 supports continuing.

## JIT Phase 2 checkpoint (P2-3, 2026-09-07)

New `tb_mem` soak (array-sum loop: native LDR/STR/branches; n=10):

| Bench | Mean | Ratio vs stepper |
|---|---|---|
| `tb/step_mem_ram_4k` | 452.7 µs | 1.00 |
| `tb/quanta_mem_ram_4k` | 142.1 µs | **3.19** |
| `tb/step_mem_flash_4k` | 556.8 µs | 1.00 |
| `tb/quanta_mem_flash_4k` | 149.1 µs | **3.73** |

ALU soak holds (`tb/quanta_ram_4k` 165.8 µs ≈ 2.47×,
`tb/quanta_flash_4k` 159.6 µs ≈ 3.10×; small move from store-seal
block splitting is within run noise — the loop is store-free).
Memory-heavy code gains most (interp loads pay full fetch + decode +
dispatch per access). Still not G2: chaining (P2-4) is the next prize.

## JIT Phase 2 checkpoint (P2-4, 2026-09-07)

Chained edges (static + FALL + single-target dynamic) with per-call
budgets. New `tb_call` soak (BL/BX lr ping-pong, n=10); ALU/mem soaks
re-run (loop back-edges now chain — driver trips nearly gone):

| Bench | Mean | Ratio vs stepper |
|---|---|---|
| `tb/step_ram_4k` | 432.5 µs | 1.00 |
| `tb/quanta_ram_4k` | 49.7 µs | **8.70** |
| `tb/step_flash_4k` | 537.0 µs | 1.00 |
| `tb/quanta_flash_4k` | 50.6 µs | **10.61** |
| `tb/step_mem_ram_4k` | 431.8 µs | 1.00 |
| `tb/quanta_mem_ram_4k` | 54.2 µs | **7.97** |
| `tb/step_mem_flash_4k` | 535.8 µs | 1.00 |
| `tb/quanta_mem_flash_4k` | 56.4 µs | **9.50** |
| `tb/step_call_ram_4k` | 620.7 µs | 1.00 |
| `tb/quanta_call_ram_4k` | 111.6 µs | **5.56** |
| `tb/step_call_flash_4k` | 641.1 µs | 1.00 |
| `tb/quanta_call_flash_4k` | 112.3 µs | **5.71** |

Flash already clears 10× on loop soaks; RAM close behind. Call-heavy
code gains least (two polymorphic dynamic edges per round trip still
exit to the driver on first sight + LR/memory traffic). Remaining path
to a robust G2: wider coverage (P2-5: PUSH/POP/LDM/STM keep more code
native between edges) — the per-edge cost is now tiny, so every edge
that stays native compounds.

## JIT Phase 2 checkpoint (P2-5, 2026-09-07)

New `tb_push` soak (PUSH/POP pairs with real stack traffic, n=10):

| Bench | Mean | Ratio vs stepper |
|---|---|---|
| `tb/step_push_ram_4k` | 542.3 µs | 1.00 |
| `tb/quanta_push_ram_4k` | 850.0 µs | **0.64** |
| `tb/step_push_flash_4k` | 657.2 µs | 1.00 |
| `tb/quanta_push_flash_4k` | 865.5 µs | **0.76** |

The sub-unity push ratio is understood, not a missed bug: a store
intervenes between every link and its next traversal, so no edge in a
store loop ever observes a matching generation — every block pays the
full driver trip (probe + entry rehash + `call1` + prologue/epilogue)
plus two guards per word (probe + transfer). The same loop under
interpreted blocks runs 0.29× (per-op re-decode dominates), so native
multis are still 2×+ better than their real fallback; and cold
call-site spills (the common case) pay no chaining penalty at all.
Unlocking store-loop chaining needs split code/data write generations
(a data-only loop must not invalidate code freshness) — recorded as
Phase-2 follow-up work, since it changes the bus write chokepoint.

## JIT G2 work (2026-09-08, x86-64)

Two legs. (1) Template fuzz (`jit_fuzz`, 600 deterministic trials):
class-uniform random programs (ALU/mem/multi/branch/sys/VFP × 16/32)
run natively vs the stepper with full state compare — green, and it
caught a real product panic on the way (`DIV` with `rm >= 13` indexed
`regs.r[]` in cycle accounting; now `regs.read`, matching exec).
(2) Emission groups: a miss now translates static successors
(taken-first, depth ≤ 2, ≤ 4 bodies, ≤ 12 ops) into the same
assembler/finalize — one ~17µs mapping tax per group instead of per
block. Same-session A/B (groups on/off):

| Soak (RAM, cold-4k) | Groups off | Groups on |
|---|---|---|
| ALU (`quanta_ram_4k`) | 44.4 µs (8.2×) | 33.9 µs (**10.3×**) |
| call (`quanta_call_ram_4k`) | 107.7 µs (4.7×) | 60.5 µs (**8.3×**) |
| mem (`quanta_mem_ram_4k`) | 51.7 µs (7.4×) | 66.2 µs (5.7×) |
| push (`quanta_push_ram_4k`) | 0.57× | 0.57× |

Loop/call shapes win big (their successors run hot — the saved
finalizes dominate); the mem soak's fallthrough successor never runs in
the 4k window, so speculation there is pure (bounded) waste plus
second-order mapping effects. Groups stay: net strongly positive, and
the waste is capped (no 64-op zero monsters, one page per group).
Sustained (warm) ALU throughput is ~3.0ns/op ≈ **45×** the stepper —
the emitter approach the gate questions is healthy; the cold-4k ALU
number sits at the 10× line with run variance (±3%), so the gate
verdict below stays with the owner. Flash loop soaks clear 10×
(`quanta_flash_4k` 12.1×).

## P3-4 per-platform baselines (x86-64 Linux)

Soak workloads for G3 acceptance (gate numbers on x86-64; mac ARM and
Windows smoke pending CI runs on those hosts):

| Soak (cold, per-iteration fresh runner) | Stepper | JIT | Ratio |
|---|---|---|---|
| board NOP 1M (`step_board_nop_1M`) | 2.6476 s | 33.132 ms | **79.9** |
| SHA-256 loop (`step_sha_ram_4k`) | 838.42 µs | 1154.2 µs | **0.73** |
| ALU torture (`step_torture_ram_4k`) | 416.82 µs | 84.259 µs | **4.95** |

- Board NOP: stm32f103 flash sled (WS realism via ACR), 1M ops/iter.
  Translation amortizes to nothing; sustained rate dominates.
- SHA-256: gcc-compiled compression firmware (`benches/data/sha256.c`,
  exact build in `scripts/gen-sha256-blob.sh`), proven against the NIST
  "abc" vector in `tests/sha_blob.rs`. The 0.73× is understood, not a
  missed bug: volatile-RAM schedule traffic fences every chained edge
  (a store intervenes between every link and its traversal — the same
  mechanism as the push soak), so blocks run solo with full driver
  trips. Fixing it is the split code/data write generations follow-up,
  now the critical path to G3's 50× geomean (with SHA below unity no
  geomean over these three can reach 50×).
- ALU torture: every 16-bit DP form (assembler-verified halves) plus
  MOVW/SDIV/CBNZ in a 36-op loop. Flag-dense templates cost ~2× the
  lean ALU soak per op; templates are fuzz-proven, so this is rate,
  not correctness.

## Regression policy

Wall-clock gating on shared CI runners is flake-prone, so CI runs the
benches as correctness (cycle assertions must hold) and humans compare
wall time against this table. A >15% wall-time regression on a quiet
runner is investigated, not auto-failed.
