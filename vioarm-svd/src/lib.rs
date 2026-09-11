#![warn(missing_docs)]
//! `vioarm-svd` as a library: the CMSIS-SVD parser (`svd`) and the
//! register-table emitter (`emit`) behind the `vioarm-svd` binary.
//!
//! The binary (`src/main.rs`) is a thin CLI over these modules; the fuzz
//! harness (`fuzz/src/bin/fuzz_svd.rs`) drives `svd::parse` directly.

pub mod emit;
pub mod svd;
