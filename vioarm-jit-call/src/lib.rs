//! Call trampolines for JIT-emitted machine code.
//!
//! # Why this crate exists
//!
//! `workspace.lints.rust.unsafe_code = "forbid"` applies to all of
//! `vioarm-core`, and `forbid` — unlike `deny` — cannot be weakened by a
//! targeted `#[allow]` anywhere in the crate. Calling emitted machine code
//! (transmuting an entry pointer to a function pointer and jumping to it)
//! is inherently unsafe, so the call site cannot live in `vioarm-core`.
//! This micro-crate encapsulates exactly that: two tiny wrappers whose
//! audit surface is Three Lines of Unsafe (marked below), called by the
//! Phase-2 emitter through safe signatures.
//!
//! # SAFETY CASE (jit-roadmap P2-0)
//!
//! A `safe` wrapper around an indirect machine-code jump is sound only if
//! every reachable entry point is valid emitted code with the matching ABI.
//! That rests on three legs, all enforced outside this crate's lines:
//!
//! 1. **Single producer.** The only producer of entry pointers is the
//!    `vioarm-core` JIT emitter (`jit/emit.rs` and its Phase-2 templates),
//!    which hands out pointers solely to committed `dynasmrt`
//!    `ExecutableBuffer`s it owns (executable, alive, same-process).
//! 2. **ABI match.** `call0`/`call1` use the C ABI with `u64` in/out; every
//!    template ends its prologue/epilogue honoring it (covered by the
//!    P2-2 template tests, which execute through these wrappers).
//! 3. **No arbitrary entry.** Nothing else in the tree calls these
//!    functions (enforced by review: `grep call0/call1` must show only the
//!    emitter driver and these tests).
//!
//! Residual risk (emitter miscompilation jumping somewhere unintended) is
//! the general JIT-correctness risk, covered by template unit tests (P2-2)
//! and differential fuzzing (P3-2) — not by this crate's lines, which do
//! nothing but jump.
//!
//! # Hygiene
//!
//! This crate does not inherit `workspace.lints` (which would forbid
//! `unsafe_code` with no override): its own `[lints]` table allows
//! `unsafe_code` for exactly the two marked blocks and keeps everything
//! else at the workspace's levels. Any new `unsafe` here must extend the
//! SAFETY CASE above.

#![deny(missing_docs)]

/// Call JIT-emitted code at `entry` with no argument, returning its `u64`
/// result (C ABI).
///
/// # Caller obligations (see SAFETY CASE above)
///
/// `entry` must point to committed, live, executable emitter output using
/// the C ABI with a `u64` return. The emitter upholds this for every
/// pointer it hands out; do not call with any other pointer.
///
/// `clippy::not_unsafe_ptr_arg_deref` is allowed here deliberately: the
/// lint is right in general, but marking these `unsafe fn` would push the
/// `unsafe {}` block into `vioarm-core`, where `forbid(unsafe_code)` bans
/// it. Encapsulating the jump here — with the obligations above upheld by
/// the emitter legs — keeps the audit surface at two marked blocks.
#[must_use]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn call0(entry: *const u8) -> u64 {
    // THREE LINES OF UNSAFE (1/2): transmute the entry pointer and jump.
    // `extern "C" fn() -> u64`: no arguments, `u64` in RAX/X0.
    let target: extern "C" fn() -> u64 = unsafe { core::mem::transmute(entry) };
    target()
}

/// Call JIT-emitted code at `entry` with one `u64` argument, returning its
/// `u64` result (C ABI).
///
/// # Caller obligations (see SAFETY CASE above)
///
/// As in [`call0`], plus: the emitted entry must take exactly one `u64`
/// argument (RDI/X0) and return `u64` (same deliberate lint allow).
#[must_use]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn call1(entry: *const u8, arg: u64) -> u64 {
    // THREE LINES OF UNSAFE (2/2): transmute the entry pointer and jump.
    let target: extern "C" fn(u64) -> u64 = unsafe { core::mem::transmute(entry) };
    target(arg)
}

#[cfg(test)]
mod tests {
    use dynasmrt::{dynasm, AssemblyOffset};

    // Only the x64 encoding path needs the trait in scope.
    #[cfg(target_arch = "x86_64")]
    use dynasmrt::DynasmApi;

    use super::*;

    #[cfg(target_arch = "x86_64")]
    fn assemble_ret42() -> dynasmrt::ExecutableBuffer {
        let mut ops = dynasmrt::x64::Assembler::new().unwrap();
        dynasm!(ops
            ; .arch x64
            ; mov eax, 42i32
            ; ret
        );
        ops.finalize().unwrap()
    }

    #[cfg(target_arch = "aarch64")]
    fn assemble_ret42() -> dynasmrt::ExecutableBuffer {
        let mut ops = dynasmrt::aarch64::Assembler::new().unwrap();
        dynasm!(ops
            ; .arch aarch64
            ; mov x0, #42
            ; ret
        );
        ops.finalize().unwrap()
    }

    #[cfg(target_arch = "x86_64")]
    fn assemble_add1() -> dynasmrt::ExecutableBuffer {
        let mut ops = dynasmrt::x64::Assembler::new().unwrap();
        dynasm!(ops
            ; .arch x64
            ; mov rax, rdi
            ; add rax, 1i8
            ; ret
        );
        ops.finalize().unwrap()
    }

    #[cfg(target_arch = "aarch64")]
    fn assemble_add1() -> dynasmrt::ExecutableBuffer {
        let mut ops = dynasmrt::aarch64::Assembler::new().unwrap();
        dynasm!(ops
            ; .arch aarch64
            ; add x0, x0, #1
            ; ret
        );
        ops.finalize().unwrap()
    }

    #[test]
    fn call0_returns_emitted_value() {
        let buf = assemble_ret42();
        assert_eq!(call0(buf.ptr(AssemblyOffset(0))), 42);
    }

    #[test]
    fn call1_passes_argument() {
        let buf = assemble_add1();
        assert_eq!(call1(buf.ptr(AssemblyOffset(0)), 41), 42);
    }
}
