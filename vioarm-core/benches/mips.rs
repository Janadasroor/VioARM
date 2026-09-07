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

/// G1 gate measurement (Phase 1 JIT): TB-quanta vs stepper wall time for
/// the same retired-op count. The loop lives in RAM (no stepper decode
/// cache off-flash: the TB win) and in flash (stepper cached: parity
/// expectation) for context. Cold cache per iteration: honest short-run
/// cost including block builds.
#[cfg(feature = "jit")]
fn bench_tb_quanta(c: &mut Criterion) {
    use vioarm_core::jit::TbRunner;
    // top: MOVS r0,#0; MOVS r1,#1; MOVS r2,#200;
    // loop: ADDS r0,r0,r1; ADDS r1,#1; SUBS r2,#1; BNE loop; B top.
    // BNE @base+12 -> base+6: off = -10 -> 0xD1FB. B @base+14 -> base: -18 -> 0xE7F7.
    let prog = vec![
        0x00, 0x20, 0x01, 0x21, 0xC8, 0x23, 0x40, 0x18, 0x49, 0x31, 0x91, 0x3B, 0xFB, 0xD1, 0xF7,
        0xE7,
    ];
    // Memory-mix soak (P2-3): fixed-trip array-sum loop exercising
    // native loads/stores/branches (LDR/ADDS/ADDS/SUBS/BNE + STR tail).
    // Loop body at ram_base: LDR r3,[r1]; ADDS r0,r0,r3; ADDS r1,#4;
    // SUBS r2,#1; BNE loop. BNE @+8 -> +0: off -12 -> 0xD1FA.
    let mem_prog = vec![
        0x0B, 0x68, 0xC0, 0x18, 0x04, 0x31, 0x01, 0x3A, 0xFA, 0xD1, 0x08, 0x60,
    ];
    let mem_setup = |base: u32| {
        // Array clear of the code (RAM-hosted code starts at the bank).
        let array = if base == 0x2000_0000 {
            0x2000_1000
        } else {
            0x2000_0000
        };
        let mem = FlatMemory::with_layout(&[base, 0x2000_0000], &[0x10000, 0x10000]);
        let mut m = Machine::armv7m(mem);
        m.load_image(&load_flat(base, &mem_prog));
        for i in 0..4096u32 {
            let v = i.wrapping_mul(2654435761).wrapping_add(0x9E37);
            for (j, b) in v.to_le_bytes().iter().enumerate() {
                m.core.mem().write(array + 4 * i + j as u32, *b).unwrap();
            }
        }
        m.core.regs.pc = base;
        m.core.regs.r[0] = 0;
        m.core.regs.r[1] = array;
        m.core.regs.r[2] = 100000;
        m.core.xpsr.set(0x01000000);
        m
    };
    // Call/return soak (P2-4): BL + BX lr pairs chaining across blocks.
    // Layout at base: MOVS r2,#200; loop: BL func(+6); SUBS r2,#1;
    // BNE loop(-10); B top(-14); func: ADDS r0,r0,r1; BX lr.
    // BL #6 = (F000,F803); BNE -10 = D1FB; B -14 = E7F9.
    let call_prog = vec![
        0xC8, 0x22, 0x00, 0xF0, 0x03, 0xF8, 0x01, 0x3A, 0xFB, 0xD1, 0xF9, 0xE7, 0x40, 0x18, 0x70,
        0x47,
    ];
    let call_setup = |base: u32| {
        let mem = FlatMemory::with_layout(&[base], &[0x10000]);
        let mut m = Machine::armv7m(mem);
        m.load_image(&load_flat(base, &call_prog));
        m.core.regs.pc = base;
        m.core.regs.r[0] = 0;
        m.core.regs.r[1] = 1;
        m.core.xpsr.set(0x01000000);
        m
    };
    let mut g = c.benchmark_group("tb");
    for (name, base) in [("ram", 0x2000_0000u32), ("flash", 0x0800_0000u32)] {
        g.bench_function(format!("step_{name}_4k"), |b| {
            b.iter_batched(
                || {
                    let mem = FlatMemory::with_layout(&[base], &[0x10000]);
                    let mut m = Machine::armv7m(mem);
                    m.load_image(&load_flat(base, &prog));
                    m.core.regs.pc = base;
                    m.core.xpsr.set(0x01000000);
                    m
                },
                |mut m| {
                    for _ in 0..4096 {
                        let _ = m.step();
                    }
                    black_box(m.core.cyccnt)
                },
                BatchSize::SmallInput,
            );
        });
        g.bench_function(format!("quanta_{name}_4k"), |b| {
            b.iter_batched(
                || {
                    let mem = FlatMemory::with_layout(&[base], &[0x10000]);
                    let mut m = Machine::armv7m(mem);
                    m.load_image(&load_flat(base, &prog));
                    m.core.regs.pc = base;
                    m.core.xpsr.set(0x01000000);
                    (m, TbRunner::new())
                },
                |(mut m, mut runner)| {
                    let mut ops = 0u64;
                    while ops < 4096 {
                        ops += m.run_tb_quantum(&mut runner, 1024).unwrap().ops;
                    }
                    black_box(m.core.cyccnt)
                },
                BatchSize::SmallInput,
            );
        });
        g.bench_function(format!("step_mem_{name}_4k"), |b| {
            b.iter_batched(
                || mem_setup(base),
                |mut m| {
                    for _ in 0..4096 {
                        let _ = m.step();
                    }
                    black_box(m.core.cyccnt)
                },
                BatchSize::SmallInput,
            );
        });
        g.bench_function(format!("quanta_mem_{name}_4k"), |b| {
            b.iter_batched(
                || (mem_setup(base), TbRunner::new()),
                |(mut m, mut runner)| {
                    let mut ops = 0u64;
                    while ops < 4096 {
                        ops += m.run_tb_quantum(&mut runner, 1024).unwrap().ops;
                    }
                    black_box(m.core.cyccnt)
                },
                BatchSize::SmallInput,
            );
        });
        g.bench_function(format!("step_call_{name}_4k"), |b| {
            b.iter_batched(
                || call_setup(base),
                |mut m| {
                    for _ in 0..4096 {
                        let _ = m.step();
                    }
                    black_box(m.core.cyccnt)
                },
                BatchSize::SmallInput,
            );
        });
        g.bench_function(format!("quanta_call_{name}_4k"), |b| {
            b.iter_batched(
                || (call_setup(base), TbRunner::new()),
                |(mut m, mut runner)| {
                    let mut ops = 0u64;
                    while ops < 4096 {
                        ops += m.run_tb_quantum(&mut runner, 1024).unwrap().ops;
                    }
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
#[cfg(feature = "jit")]
criterion_group!(benches_jit, bench_tb_quanta);
#[cfg(not(feature = "jit"))]
criterion_main!(benches);
#[cfg(feature = "jit")]
criterion_main!(benches, benches_jit);
