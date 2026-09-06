//! dynasm smoke assembler (P2-0 dependency gate).
//!
//! Proves the `dynasm`/`dynasmrt` pair (pinned synced, see `Cargo.toml`)
//! assembles and commits executable memory on this host, using only the
//! crates' safe API — so `workspace.lints.rust.unsafe_code = "forbid"`
//! stays intact in `vioarm-core`. Execution of the buffer goes through
//! `vioarm-jit-call` (see its SAFETY CASE); the unit test below is the
//! full vertical slice (assemble → call → value) with zero `unsafe` in
//! this crate.
//!
//! Host coverage: x86-64 and AArch64 (the CI legs). Any other host that
//! enables `jit` fails fast here with a clear message instead of a
//! half-working emitter.

use dynasmrt::{dynasm, ExecutableBuffer};

// Only the x64 encoding path needs the trait in scope (the AArch64
// instructions used here don't); keep the import arch-gated so neither
// host warns.
#[cfg(target_arch = "x86_64")]
use dynasmrt::DynasmApi;

/// Assemble a stub returning the constant 42 (`u64`, C ABI) and commit it
/// as executable memory.
///
/// # Errors
///
/// Returns [`EmitError`] when the executable mapping cannot be allocated
/// or finalization fails.
#[cfg(target_arch = "x86_64")]
pub fn assemble_smoke() -> Result<ExecutableBuffer, EmitError> {
    let mut ops = dynasmrt::x64::Assembler::new()?;
    dynasm!(ops
        ; .arch x64
        ; mov eax, 42i32
        ; ret
    );
    ops.finalize().map_err(|_| EmitError::Finalize)
}

/// Assemble a stub returning the constant 42 (`u64`, C ABI) and commit it
/// as executable memory.
///
/// # Errors
///
/// Returns [`EmitError`] when the executable mapping cannot be allocated
/// or finalization fails.
#[cfg(target_arch = "aarch64")]
pub fn assemble_smoke() -> Result<ExecutableBuffer, EmitError> {
    let mut ops = dynasmrt::aarch64::Assembler::new()?;
    dynasm!(ops
        ; .arch aarch64
        ; mov x0, #42
        ; ret
    );
    ops.finalize().map_err(|_| EmitError::Finalize)
}

#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
compile_error!("jit feature: dynasm smoke supports x86-64 and AArch64 hosts only (P2-0 gate)");

/// Assembly failure.
#[derive(Debug)]
pub enum EmitError {
    /// The executable mapping could not be allocated.
    Alloc(std::io::Error),
    /// Finalization (relocation/commit) failed.
    Finalize,
    /// The op has no native template (the block stays interpreted).
    Untemplated,
}

impl From<std::io::Error> for EmitError {
    fn from(err: std::io::Error) -> Self {
        Self::Alloc(err)
    }
}

impl core::fmt::Display for EmitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Alloc(err) => write!(f, "executable mapping failed: {err}"),
            Self::Finalize => write!(f, "assembly finalization failed"),
            Self::Untemplated => write!(f, "op has no native template"),
        }
    }
}

impl std::error::Error for EmitError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Alloc(err) => Some(err),
            Self::Finalize | Self::Untemplated => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use dynasmrt::AssemblyOffset;

    use super::*;

    #[test]
    fn smoke_assembles_executable_buffer() {
        let buf = assemble_smoke().expect("dynasm smoke must assemble");
        assert!(buf.size() > 0, "committed executable mapping");
        assert!(!buf.ptr(AssemblyOffset(0)).is_null());
    }

    #[test]
    fn smoke_calls_through_trampoline() {
        let buf = assemble_smoke().expect("dynasm smoke must assemble");
        let value = vioarm_jit_call::call0(buf.ptr(AssemblyOffset(0)));
        assert_eq!(value, 42);
    }
}
