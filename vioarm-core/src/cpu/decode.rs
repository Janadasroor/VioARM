//! One canonical Thumb (ARMv6-M) decoder.
//!
//! The decode step has a single entry point: `decode(halfword) -> Result<Inst,
//! DecodeError>`. All executors (interpreter, future fast paths) must call this
//! exact function so a future JIT or dispatch cache cannot drift from the
//! canonical semantics.

use core::fmt;

use crate::error::HaltCause;

/// An instruction that the decoder does not recognize within ARMv6-M.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DecodeError {
    /// Offending halfword.
    pub opcode: u16,
    /// Why it cannot decode.
    pub cause: HaltCause,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:04x}: {}", self.opcode, self.cause)
    }
}
