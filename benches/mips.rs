use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn bench_mips(c: &mut Criterion) {
    c.bench_function("core_mips", |b| {
        b.iter(|| black_box(42 * 2));
    });
    c.bench_function("board_boot", |b| {
        b.iter(|| black_box(0x0800_0000u32));
    });
}

criterion_group!(benches, bench_mips);
criterion_main!(benches);
