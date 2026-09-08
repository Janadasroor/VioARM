//! GATE G2 template fuzz (deterministic leg): `trials()` seeded trials
//! through the shared [`vioarm_core::jit::fuzz`] core, plus the coverage
//! obligation (every template family fires substantially). See the
//! module docs for method and boundaries; `fuzz/fuzz_jit.rs` drives the
//! same core from raw bytes for nightly fuzzing.

#![cfg(feature = "jit")]

use vioarm_core::cpu::decode16::{decode16_v7m, Inst16};
use vioarm_core::jit::fuzz::{self, BX_LR};

#[test]
fn template_fuzz_matches_stepper() {
    // `BX lr` sanity (hand-placed in call shapes).
    assert!(matches!(decode16_v7m(BX_LR), Ok(Inst16::Bx { rm: 14 })));
    let total = fuzz::trials();
    let range: Vec<u64> = match fuzz::only() {
        Some(t) => vec![t],
        None => (0..total).collect(),
    };
    let mut cover = [0u64; 10];
    for trial in range {
        let stats = fuzz::check_trial(trial).unwrap_or_else(|m| panic!("{m}"));
        for (b, c) in cover.iter_mut().zip(stats.buckets.iter()) {
            *b += *c;
        }
    }
    eprintln!("fuzz coverage buckets: {cover:?}");
    // The G2 obligation: every family fired substantially. (VFP tallies
    // only when the stream draws v7-M FP forms plus the deterministic
    // pool; system-barrier forms are rare by encoding — thresholds
    // reflect draw reality, not importance.)
    for (i, c) in cover.iter().enumerate() {
        let min = match i {
            9 => 5,  // VFP: sparse in Thumb-2 space
            4 => 10, // 16-bit system: small subspace
            8 => 10, // 32-bit sys/barrier: small subspace
            _ => 50,
        };
        assert!(
            *c >= min,
            "bucket {i} under-covered ({c} < {min}): {cover:?}"
        );
    }
}
