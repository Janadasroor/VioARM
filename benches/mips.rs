use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn bench_mips(c: &mut Criterion) {
    c.bench_function("core_mips", |b| {
        b.iter(|| black_box(42 * 2));
    });
}

criterion_group!(benches, bench_mips);
criterion_main!(benches);
