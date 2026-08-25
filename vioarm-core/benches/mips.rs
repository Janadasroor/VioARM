use criterion::{black_box, criterion_group, criterion_main, Criterion};
use vioarm_core::{board, loader::load_flat, machine::Machine};

fn bench_board_boot(c: &mut Criterion) {
    c.bench_function("stm32f103_boot", |b| {
        b.iter(|| {
            let mut m = Machine::armv7m(board::stm32f103());
            let code = vec![0x00, 0xBF, 0x00, 0xBF];
            let img = load_flat(0x0800_0000, &code);
            m.load_image(&img);
            m.reset().unwrap();
            for _ in 0..100 {
                let _ = black_box(m.step());
            }
        })
    });
}

criterion_group!(benches, bench_board_boot);
criterion_main!(benches);
