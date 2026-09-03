//! `VioARM` core library.
//!
//! A soundness-first ARM Cortex-M (ARMv6-M first) simulator. The design goal
//! is that every possible stop of the machine is a typed, analyzable event and
//! that no state can be silently undefined.

#![forbid(unsafe_code)]

pub mod board;
pub mod cpu;
pub mod device;
pub mod error;
pub mod loader;
pub mod machine;
pub mod mem;
pub mod semihost;
