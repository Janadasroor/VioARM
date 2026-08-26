use criterion::{black_box, criterion_group, criterion_main, Criterion};
use vioarm_core::{loader::load_flat, machine::Machine, mem::flat::FlatMemory};

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

criterion_group!(benches, bench_board_boot, bench_mips);
criterion_main!(benches);
