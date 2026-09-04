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

criterion_group!(benches, bench_board_boot, bench_mips, bench_attribution);
criterion_main!(benches);
