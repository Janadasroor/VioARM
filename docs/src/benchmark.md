# Field benchmark

Measure throughput on the machine in front of you — not in criterion
output. `vioarm benchmark` runs the same kernel shapes as the
criterion group in `vioarm-core/benches/mips.rs` through the release
`Machine`, with no dev-dependencies at runtime:

```sh
cargo run --release -p vioarm-cli -- benchmark --cycles 1000000
```

Example output (this box, release):

```text
bare-nop 8.245 121.3 1000000
board-boot 0.448 2233.2 1000000
usart-tx 0.455 2197.1 1994408 # tx=330200B
```

## Reading the rows

Each stdout row is `kernel mips wall-ms cyccnt [# note]` — plain
numbers scripts parse; the trailing comment is human context:

- `mips` — retired instructions per wall microsecond (host speed, not
  simulated speed; `cyccnt` is the simulated cycle count).
- `wall-ms` — stepping time only. Kernel setup and per-repeat reboots
  stay outside the timer, mirroring criterion's `iter_batched`.
- `cyccnt` — simulated `DWT CYCCNT` delta across the timed steps
  (timing-model output, host-independent).

Always compare release builds: debug builds measure ~10× slower and
their numbers mean nothing across boxes.

## The kernels

| Kernel | Shape | Bench twin |
|---|---|---|
| `bare-nop` | 1000 NOPs on bare `FlatMemory` (ARMv7-M) | `mips/nop_loop_1k` (~4.5 MIPS) |
| `board-boot` | Vector-table boot at the flash alias, 10k NOPs per window, family-correct core | `board/stm32f103_boot_10k_nop` |
| `usart-tx` | F1-map firmware (`CR1=UE\|TE`, then 200 `STR` to `DR` per window); `# tx=` counts drained bytes | New (no criterion twin) |

`--cycles N` retires roughly N steps per kernel (fixed windows repeat
to it; small N still runs one full window). `--board` switches the
board kernels (default `stm32f103`); `bare-nop` is board-independent.
`usart-tx` resolves USART1's base from the board's SVD table and needs
the F1 register map (F1/F2/F4/F7) — anything else fails a host-side TX
probe and skips with a stderr note instead of timing a dead store
(the virtual board always skips: it carries a `uart`, not a `usart1`).

## Against the book numbers

On the reference machine (see `vioarm-core/benches/BENCHMARKS.md`),
expect the stepper rows within ~15%: `bare-nop` near the
`mips/nop_loop_1k` row, `board-boot` near `attrib/board_nop_1k`
(~0.41 MIPS). On other hosts compare against your own criterion runs
(`cargo bench -p vioarm-core --bench mips`) — the load-bearing shape
is the ratio: board kernels land about an order of magnitude below
`bare-nop` (bus scan + device ticks dominate), and `usart-tx` tracks
`board-boot` closely.

## CI floors

`--fail-under <mips>` exits 1 when any kernel lands below the floor,
after printing all rows (parseable stdout stays intact; the failure
goes to stderr):

```sh
vioarm benchmark --cycles 1000000 --fail-under 0.3 || echo "perf regression"
```

Pick the floor from your own release baselines with generous headroom —
host noise is real (20%+ swings run to run on shared, thermally-limited,
or frequency-scaling hosts were observed during development). For
tracking over time, take best-of-3 and compare identical kernels.
