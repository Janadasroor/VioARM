//! TB-vs-interpreter differential fuzzer (P3-2).
//!
//! Two modes sharing `vioarm_core::jit::fuzz` with `tests/jit_fuzz.rs`:
//!
//! - `--trials N`: run deterministic trials `0..N` (CI bounded runs and
//!   nightly throughput). Any mismatch prints and exits nonzero.
//! - `seedfile ...`: run each file as one raw differential input
//!   ([`fuzz::trial_input`] format; trailing bytes ignored). This is the
//!   libFuzzer-compatible shape: point `cargo-fuzz` at a thin
//!   `fuzz_target!(|data: &[u8]| ...)` wrapper calling
//!   `fuzz::check_bytes` when that infrastructure lands.
//! - `--dump-trial K`: write trial `K`'s input bytes to stdout (corpus
//!   seed generation: `fuzz_jit --dump-trial K > corpus/fuzz_jit/K`).
//!
//! Corpus seeds live in `fuzz/corpus/fuzz_jit/` (deterministic trials
//! covering all four shapes). Exit codes: 0 pass, 1 mismatch, 2 usage
//! or I/O error.

use std::io::Read;
use vioarm_core::jit::fuzz;

fn usage() -> ! {
    eprintln!("usage: fuzz_jit [--trials N] [--dump-trial K] [seedfile ...]");
    std::process::exit(2);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut trials: Option<u64> = None;
    let mut dump: Option<u64> = None;
    let mut seeds: Vec<String> = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--trials" => {
                i += 1;
                trials = Some(args.get(i).unwrap_or_else(|| usage()).parse().unwrap_or_else(|_| usage()));
            }
            "--dump-trial" => {
                i += 1;
                dump = Some(args.get(i).unwrap_or_else(|| usage()).parse().unwrap_or_else(|_| usage()));
            }
            s if s.starts_with('-') => usage(),
            s => seeds.push(s.to_string()),
        }
        i += 1;
    }
    if let Some(k) = dump {
        let bytes = fuzz::trial_input(k);
        use std::io::Write;
        std::io::stdout()
            .write_all(&bytes)
            .unwrap_or_else(|e| {
                eprintln!("write: {e}");
                std::process::exit(2);
            });
        return;
    }
    if !seeds.is_empty() {
        let mut n = 0u64;
        for path in &seeds {
            let mut buf = Vec::new();
            std::fs::File::open(path)
                .and_then(|mut f| f.read_to_end(&mut buf))
                .unwrap_or_else(|e| {
                    eprintln!("{path}: {e}");
                    std::process::exit(2);
                });
            match fuzz::check_bytes(&buf) {
                None => {
                    eprintln!("{path}: rejected (degenerate), skipped");
                    continue;
                }
                Some(Ok(_)) => n += 1,
                Some(Err(m)) => {
                    eprintln!("{path}: MISMATCH\n{m}");
                    std::process::exit(1);
                }
            }
        }
        eprintln!("seeds ok: {n}");
        return;
    }
    let n = trials
        .or_else(|| {
            std::env::var("FUZZ_TRIALS")
                .ok()
                .and_then(|v| v.parse().ok())
        })
        .unwrap_or(600);
    let mut cover = [0u64; 10];
    for t in 0..n {
        match fuzz::check_trial(t) {
            Ok(stats) => {
                for (b, c) in cover.iter_mut().zip(stats.buckets.iter()) {
                    *b += *c;
                }
            }
            Err(m) => {
                eprintln!("MISMATCH\n{m}");
                std::process::exit(1);
            }
        }
    }
    eprintln!("trials ok: {n} buckets: {cover:?}");
}
