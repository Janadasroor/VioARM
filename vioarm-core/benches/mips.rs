use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use vioarm_core::{
    board,
    loader::load_flat,
    machine::Machine,
    mem::{flat::FlatMemory, Bus, PeripheralBus},
};

fn bench_board_boot(c: &mut Criterion) {
    let mut group = c.benchmark_group("board");
    group.bench_function("stm32f103_boot_10k_nop", |b| {
        b.iter(|| {
            let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
            let mut m = Machine::armv7m(mem);
            let mut code = Vec::new();
            for _ in 0..10000 {
                code.extend_from_slice(&[0x00, 0xBF]);
            }
            code.extend_from_slice(&[0xFE, 0xE7]);
            let img = load_flat(0x0800_0000, &code);
            m.load_image(&img);
            m.core.regs.pc = 0x0800_0000;
            m.core.xpsr.set(0x01000000);
            for _ in 0..10000 {
                let _ = black_box(m.step());
            }
            black_box(m.core.cyccnt)
        })
    });
    group.finish();
}

fn bench_mips(c: &mut Criterion) {
    let mut g = c.benchmark_group("mips");
    g.bench_function("nop_loop_1k", |b| {
        b.iter(|| {
            let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
            let mut m = Machine::armv7m(mem);
            let mut code = Vec::new();
            for _ in 0..1000 {
                code.extend_from_slice(&[0x00, 0xBF]);
            }
            let img = load_flat(0x0800_0000, &code);
            m.load_image(&img);
            m.core.regs.pc = 0x0800_0000;
            m.core.xpsr.set(0x01000000);
            let start = m.core.cyccnt;
            for _ in 0..1000 {
                let _ = m.step();
            }
            let delta = m.core.cyccnt - start;
            assert!((950..=1050).contains(&delta), "mips gated {delta} vs 1000");
            black_box(delta)
        })
    });
    g.finish();
}

/// Attribution benches (M7-G1): same 1000-step window, setup excluded via
/// `iter_batched`, so the means divide into per-step costs.
fn bench_attribution(c: &mut Criterion) {
    let mut g = c.benchmark_group("attrib");

    // Bare core, sequential NOPs: every fetch is a decode-cache miss.
    g.bench_function("bare_nop_1k_miss", |b| {
        b.iter_batched(
            || {
                let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
                let mut m = Machine::armv7m(mem);
                let mut code = Vec::new();
                for _ in 0..1000 {
                    code.extend_from_slice(&[0x00, 0xBF]);
                }
                let img = load_flat(0x0800_0000, &code);
                m.load_image(&img);
                m.core.regs.pc = 0x0800_0000;
                m.core.xpsr.set(0x01000000);
                m
            },
            |mut m| {
                for _ in 0..1000 {
                    let _ = m.step();
                }
                black_box(m.core.cyccnt)
            },
            BatchSize::SmallInput,
        );
    });

    // Bare core, tight 2-instruction loop: steady-state decode-cache hits.
    g.bench_function("bare_loop_100_hit", |b| {
        b.iter_batched(
            || {
                let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
                let mut m = Machine::armv7m(mem);
                // movs r0,#100; loop: subs r0,#1; bne loop
                let code = [0x64u8, 0x20, 0x01, 0x38, 0xFE, 0xD1];
                let img = load_flat(0x0800_0000, &code);
                m.load_image(&img);
                m.core.regs.pc = 0x0800_0000;
                m.core.xpsr.set(0x01000000);
                m
            },
            |mut m| {
                for _ in 0..201 {
                    let _ = m.step();
                }
                black_box(m.core.cyccnt)
            },
            BatchSize::SmallInput,
        );
    });

    // Bare core without the Machine wrapper: separates core cost from
    // machine-step overhead (NVIC peeks, tick dispatch, reset checks).
    g.bench_function("core_nop_1k", |b| {
        b.iter_batched(
            || {
                let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
                let mut core = vioarm_core::cpu::Core::armv7m(mem);
                let mut code = Vec::new();
                for _ in 0..1000 {
                    code.extend_from_slice(&[0x00, 0xBF]);
                }
                let img = load_flat(0x0800_0000, &code);
                for (addr, bytes) in &img.segments {
                    for (i, &byte) in bytes.iter().enumerate() {
                        core.mem().write(addr.wrapping_add(i as u32), byte).unwrap();
                    }
                }
                core.regs.pc = 0x0800_0000;
                core.xpsr.set(0x01000000);
                core
            },
            |mut core| {
                for _ in 0..1000 {
                    let _ = core.step();
                }
                black_box(core.cyccnt)
            },
            BatchSize::SmallInput,
        );
    });

    // Full f103 board, sequential NOPs: adds the bus scan, NVIC scan, and
    // per-step device ticks to the bare cost above. Flash is unlocked the
    // way firmware does it (F1 KEYR sequence), and the landing is asserted
    // so a gating change fails loudly instead of timing fault paths.
    g.bench_function("board_nop_1k", |b| {
        b.iter_batched(
            || {
                let mem = board::stm32f103();
                let mut m = Machine::armv7m(mem);
                for (i, b) in 0x4567_0123u32.to_le_bytes().iter().enumerate() {
                    m.core.mem().write(0x4002_2004 + i as u32, *b).unwrap();
                }
                for (i, b) in 0xCDEF_89ABu32.to_le_bytes().iter().enumerate() {
                    m.core.mem().write(0x4002_2004 + i as u32, *b).unwrap();
                }
                // CR.PG: programming mode so the image writes land.
                m.core.mem().write(0x4002_2010, 1).unwrap();
                let mut code = Vec::new();
                for _ in 0..1000 {
                    code.extend_from_slice(&[0x00, 0xBF]);
                }
                code.extend_from_slice(&[0xFE, 0xE7]);
                let img = load_flat(0x0800_0000, &code);
                m.load_image(&img);
                assert_eq!(m.core.mem().read_half(0x0800_0000).unwrap(), 0xBF00);
                m.core.regs.pc = 0x0800_0000;
                m.core.xpsr.set(0x01000000);
                m
            },
            |mut m| {
                for _ in 0..1000 {
                    let _ = m.step();
                }
                black_box(m.core.cyccnt)
            },
            BatchSize::SmallInput,
        );
    });

    // Raw bus cost: one flash halfword read on the full board (scan +
    // backing, no core).
    g.bench_function("bus_flash_read_1k", |b| {
        b.iter_batched(
            board::stm32f103,
            |mut mem| {
                let mut acc = 0u16;
                for i in 0..1000u32 {
                    acc = acc.wrapping_add(mem.read_half(0x0800_0000 + i * 2).unwrap_or(0));
                }
                black_box(acc)
            },
            BatchSize::SmallInput,
        );
    });

    // Raw backing cost with no bus at all (baseline for the two below).
    g.bench_function("bus_flat_read_1k", |b| {
        b.iter_batched(
            || FlatMemory::with_layout(&[0x0800_0000], &[0x10000]),
            |mut mem| {
                let mut acc = 0u16;
                for i in 0..1000u32 {
                    acc = acc.wrapping_add(mem.read_half(0x0800_0000 + i * 2).unwrap_or(0));
                }
                black_box(acc)
            },
            BatchSize::SmallInput,
        );
    });

    // Raw bus cost on the big H7 board (100+ windows): where the index
    // should win over the linear scan.
    g.bench_function("bus_h743_flash_read_1k", |b| {
        b.iter_batched(
            board::stm32h743,
            |mut mem| {
                let mut acc = 0u16;
                for i in 0..1000u32 {
                    acc = acc.wrapping_add(mem.read_half(0x0800_0000 + i * 2).unwrap_or(0));
                }
                black_box(acc)
            },
            BatchSize::SmallInput,
        );
    });

    // In-process A/B of the route index: same binary, same process, the
    // flag flips per setup, so machine drift cannot fake the delta.
    for (name, linear) in [("index", false), ("linear", true)] {
        let bench = format!("bus_h743_route_{name}_1k");
        g.bench_function(&bench, |b| {
            b.iter_batched(
                || {
                    let mut mem = board::stm32h743();
                    mem.set_route_force_linear(linear);
                    mem
                },
                |mut mem| {
                    let mut acc = 0u32;
                    for i in 0..1000u32 {
                        acc = acc
                            .wrapping_add(mem.read_half(0x0800_0000 + i * 2).unwrap_or(0) as u32);
                    }
                    black_box(acc)
                },
                BatchSize::SmallInput,
            );
        });
    }

    // Per-step peripheral time advance on the full board (all device ticks
    // + NVIC latch + mailbox drains, no core).
    g.bench_function("bus_tick_1k", |b| {
        b.iter_batched(
            board::stm32f103,
            |mut mem| {
                for _ in 0..1000 {
                    mem.tick_devices(1);
                }
                black_box(());
            },
            BatchSize::SmallInput,
        );
    });

    // NVIC exception peek on the full board (per-step interrupt scan).
    g.bench_function("bus_next_exception_1k", |b| {
        b.iter_batched(
            board::stm32f103,
            |mut mem| {
                let mut acc = 0u32;
                for _ in 0..1000 {
                    acc = acc
                        .wrapping_add(mem.next_exception(false, false, 0, 0).unwrap_or(0) as u32);
                }
                black_box(acc)
            },
            BatchSize::SmallInput,
        );
    });

    g.finish();
}

/// Hook overhead (M9-I1): the same 1000-step NOP window with no hooks,
/// with hooks armed on never-matching ranges (check cost), and with a
/// full-range tracing hook (fire cost). Acceptance is `idle / none < 1.10`.
fn bench_hooks(c: &mut Criterion) {
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;
    fn nop_machine() -> Machine<FlatMemory> {
        let mem = FlatMemory::with_layout(&[0x0800_0000], &[0x10000]);
        let mut m = Machine::armv7m(mem);
        let mut code = Vec::new();
        for _ in 0..1000 {
            code.extend_from_slice(&[0x00, 0xBF]);
        }
        let img = load_flat(0x0800_0000, &code);
        m.load_image(&img);
        m.core.regs.pc = 0x0800_0000;
        m.core.xpsr.set(0x01000000);
        m
    }
    let mut g = c.benchmark_group("hooks");
    g.bench_function("steps_1k_none", |b| {
        b.iter_batched(
            nop_machine,
            |mut m| {
                for _ in 0..1000 {
                    let _ = m.step();
                }
                black_box(m.core.cyccnt)
            },
            BatchSize::SmallInput,
        );
    });
    g.bench_function("steps_1k_armed_idle", |b| {
        b.iter_batched(
            || {
                let mut m = nop_machine();
                // Never-matching ranges: pure check cost, no callback runs.
                m.on_instruction(true, 0x3000_0000, 0x3000_1000, |_| {});
                m.on_write(0x3000_0000, 0x3000_1000, |_| {});
                m
            },
            |mut m| {
                for _ in 0..1000 {
                    let _ = m.step();
                }
                black_box(m.core.cyccnt)
            },
            BatchSize::SmallInput,
        );
    });
    g.bench_function("steps_1k_trace_all", |b| {
        b.iter_batched(
            || {
                let mut m = nop_machine();
                let n = Arc::new(AtomicU64::new(0));
                let sink = n.clone();
                m.on_instruction(true, 0, u32::MAX, move |_| {
                    sink.fetch_add(1, Ordering::Relaxed);
                });
                (m, n)
            },
            |(mut m, n)| {
                for _ in 0..1000 {
                    let _ = m.step();
                }
                black_box((m.core.cyccnt, n.load(Ordering::Relaxed)))
            },
            BatchSize::SmallInput,
        );
    });
    g.finish();
}

/// Fast-mode scaling (M11): the same 1000-step board window in accurate
/// mode and at several quanta. Acceptance is best-quantum / accurate ≥ 1.5.
fn bench_fast(c: &mut Criterion) {
    let mut g = c.benchmark_group("fast");
    for quantum in [0, 8, 32, 128] {
        let bench = format!("board_nop_1k_q{quantum}");
        g.bench_function(&bench, |b| {
            b.iter_batched(
                || {
                    let mem = board::stm32f103();
                    let mut m = Machine::armv7m(mem);
                    for (i, b) in 0x4567_0123u32.to_le_bytes().iter().enumerate() {
                        m.core.mem().write(0x4002_2004 + i as u32, *b).unwrap();
                    }
                    for (i, b) in 0xCDEF_89ABu32.to_le_bytes().iter().enumerate() {
                        m.core.mem().write(0x4002_2004 + i as u32, *b).unwrap();
                    }
                    m.core.mem().write(0x4002_2010, 1).unwrap();
                    let mut code = Vec::new();
                    for _ in 0..1000 {
                        code.extend_from_slice(&[0x00, 0xBF]);
                    }
                    code.extend_from_slice(&[0xFE, 0xE7]);
                    let img = load_flat(0x0800_0000, &code);
                    m.load_image(&img);
                    m.core.regs.pc = 0x0800_0000;
                    m.core.xpsr.set(0x01000000);
                    m.set_fast_quantum(quantum);
                    m
                },
                |mut m| {
                    for _ in 0..1000 {
                        let _ = m.step();
                    }
                    m.settle_fast();
                    black_box(m.core.cyccnt)
                },
                BatchSize::SmallInput,
            );
        });
    }
    g.finish();
}

criterion_group!(
    benches,
    bench_board_boot,
    bench_mips,
    bench_attribution,
    bench_hooks,
    bench_fast
);
criterion_main!(benches);
