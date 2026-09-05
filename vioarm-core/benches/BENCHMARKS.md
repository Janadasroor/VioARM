# Benchmarks (M7-G1)

Method: `cargo bench -p vioarm-core --bench mips` (criterion, release).
Reference machine: Intel Xeon Gold 6130 @ 2.10GHz, `rustc 1.98.1`.
Step benches exclude setup (`iter_batched`); the means below divide into
per-step costs. Cycle assertions in each bench pin the simulated behavior,
so a perf change that alters timing fails loudly instead of silently.

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

## Regression policy

Wall-clock gating on shared CI runners is flake-prone, so CI runs the
benches as correctness (cycle assertions must hold) and humans compare
wall time against this table. A >15% wall-time regression on a quiet
runner is investigated, not auto-failed.
