//! ARM semihosting and SEGGER RTT host channels.
//!
//! Both channels are opt-in and deterministic: with identical firmware and an
//! identical host-input sequence the machine trace is bit-identical, because
//! every host observable (clocks, input bytes, exit codes) is a pure function
//! of machine state plus the queued input.
//!
//! Semihosting traps `BKPT 0xAB` (ARM DUI 0476). When disabled — the default,
//! preserving the QEMU-oracle parity the conformance suite diffs against — a
//! `BKPT` keeps its old meaning and faults. Enable per machine with
//! [`Machine::semihost_enable`](crate::machine::Machine::semihost_enable).
//!
//! Implemented calls: `WRITEC`, `WRITE0`, `WRITE` (fd 1/2 → host output),
//! `READC` (non-blocking: `u32::MAX` when the input queue is empty),
//! `CLOCK`/`TIME`/`ELAPSED`/`TICKFREQ` on a virtual 1 MHz timebase derived
//! from `cyccnt`, `ISTTY`, and `EXIT`. Anything else returns `-1` without
//! faulting, so unsupported newlib paths degrade instead of crashing.
//!
//! Guest-memory accesses use the normal data path: a bad guest pointer fails
//! the call (`-1` / truncated transfer) rather than faulting the machine.
//!
//! RTT drains target channel 0 by scanning guest RAM for the `SEGGER RTT`
//! control block; see [`RttState::poll`].

use crate::mem::Bus;
use std::collections::VecDeque;

/// `BKPT` immediate that traps into semihosting when enabled.
pub const SEMIHOST_BKPT: u8 = 0xAB;
/// `SYS_WRITEC`: `r1` points at the character to write (ARM DUI 0476).
pub const SYS_WRITEC: u32 = 0x03;
/// `SYS_WRITE0`: `r1` points at a NUL-terminated string.
pub const SYS_WRITE0: u32 = 0x04;
/// `SYS_WRITE`: `r1` points at `[fd, ptr, len]`; returns bytes not written.
pub const SYS_WRITE: u32 = 0x05;
/// `SYS_READC`: returns one input byte, or `u32::MAX` when empty.
pub const SYS_READC: u32 = 0x07;
/// `SYS_ISTTY`: `r1` is a file handle; 1 for fd 1/2, else 0.
pub const SYS_ISTTY: u32 = 0x09;
/// `SYS_CLOCK`: virtual centiseconds.
pub const SYS_CLOCK: u32 = 0x10;
/// `SYS_TIME`: virtual epoch seconds (always 0).
pub const SYS_TIME: u32 = 0x11;
/// `SYS_EXIT`: `r1` points at `[reason, subcode]`.
pub const SYS_EXIT: u32 = 0x18;
/// `SYS_ELAPSED`: virtual ticks since boot.
pub const SYS_ELAPSED: u32 = 0x30;
/// `SYS_TICKFREQ`: ticks per second of the virtual timebase.
pub const SYS_TICKFREQ: u32 = 0x31;
/// `ADP_Stopped_ApplicationExit`: the only exit reason that maps to code 0.
pub const ADP_EXIT: u32 = 0x20026;
/// Virtual timebase: one `cyccnt` tick is one microsecond.
pub const VIRTUAL_TICKFREQ: u32 = 1_000_000;
/// Longest string/block a single call transfers; bounds a hostile guest.
const MAX_TRANSFER: usize = 65536;

/// What a semihosting call decided.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SemihostOutcome {
    /// Continue execution with `r0` set to this value.
    Return(u32),
    /// Stop the machine with this process exit code.
    Exit(i32),
}

/// Semihosting session state: configuration plus host-channel buffers.
#[derive(Debug, Default)]
pub struct SemihostState {
    enabled: bool,
    output: Vec<u8>,
    input: VecDeque<u8>,
    exit: Option<i32>,
    pub(crate) rtt: RttState,
}

impl SemihostState {
    /// Enable or disable the `BKPT 0xAB` trap. Disabled by default.
    pub fn set_enabled(&mut self, on: bool) {
        self.enabled = on;
    }

    /// Whether the trap is armed.
    #[must_use]
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Queue host bytes for `READC` / RTT down-channel use.
    pub fn push_input(&mut self, bytes: &[u8]) {
        self.input.extend(bytes);
    }

    /// Take everything the firmware emitted since the last drain.
    pub fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.output)
    }

    /// Take a latched `EXIT` code, if firmware requested one.
    pub fn take_exit(&mut self) -> Option<i32> {
        self.exit.take()
    }

    /// Forget session data after a machine reset. The enabled flag survives:
    /// it is host configuration, not guest state.
    pub fn reset_session(&mut self) {
        self.output.clear();
        self.input.clear();
        self.exit = None;
        self.rtt.forget();
    }

    /// Serve one trapped call. `cyccnt` drives the virtual timebase.
    pub fn handle<B: Bus>(
        &mut self,
        mem: &mut B,
        op: u32,
        param: u32,
        cyccnt: u64,
    ) -> SemihostOutcome {
        match op {
            SYS_WRITEC => {
                // Spec pointer semantics (QEMU parity): `r1` addresses the
                // character. A bad pointer fails the call (`-1`) with no
                // output, like every other guest-memory access here.
                match mem.read(param) {
                    Ok(b) => {
                        self.output.push(b);
                        SemihostOutcome::Return(0)
                    }
                    Err(_) => SemihostOutcome::Return(u32::MAX),
                }
            }
            SYS_WRITE0 => {
                self.output.extend(read_cstring(mem, param));
                SemihostOutcome::Return(0)
            }
            SYS_WRITE => {
                let mut block = [0u32; 3];
                for (i, slot) in block.iter_mut().enumerate() {
                    *slot = mem.read_word(param.wrapping_add(i as u32 * 4)).unwrap_or(0);
                }
                let [fd, ptr, len] = block;
                let want = len.min(MAX_TRANSFER as u32) as usize;
                if fd == 1 || fd == 2 {
                    let mut got = 0;
                    for i in 0..want {
                        match mem.read(ptr.wrapping_add(i as u32)) {
                            Ok(b) => {
                                self.output.push(b);
                                got += 1;
                            }
                            Err(_) => break,
                        }
                    }
                    SemihostOutcome::Return(len.wrapping_sub(got as u32))
                } else {
                    SemihostOutcome::Return(len)
                }
            }
            SYS_READC => {
                SemihostOutcome::Return(self.input.pop_front().map_or(u32::MAX, u32::from))
            }
            SYS_ISTTY => SemihostOutcome::Return(u32::from(param == 1 || param == 2)),
            SYS_CLOCK => SemihostOutcome::Return((cyccnt / 10_000) as u32),
            SYS_TIME => SemihostOutcome::Return(0),
            SYS_ELAPSED => SemihostOutcome::Return(cyccnt as u32),
            SYS_TICKFREQ => SemihostOutcome::Return(VIRTUAL_TICKFREQ),
            SYS_EXIT => {
                let reason = mem.read_word(param).unwrap_or(0);
                let code = if reason == ADP_EXIT { 0 } else { 1 };
                self.exit = Some(code);
                SemihostOutcome::Exit(code)
            }
            _ => SemihostOutcome::Return(u32::MAX),
        }
    }
}

/// Read a NUL-terminated guest string, stopping at the first fault.
fn read_cstring<B: Bus>(mem: &mut B, mut ptr: u32) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..MAX_TRANSFER {
        match mem.read(ptr) {
            Ok(0) => break,
            Ok(b) => out.push(b),
            Err(_) => break,
        }
        ptr = ptr.wrapping_add(1);
    }
    out
}

/// SEGGER RTT control-block magic: `SEGGER RTT` plus six zero bytes.
const RTT_MAGIC: [u8; 16] = [
    b'S', b'E', b'G', b'G', b'E', b'R', b' ', b'R', b'T', b'T', 0, 0, 0, 0, 0, 0,
];
/// Largest ring accepted; rejects false-positive magic hits.
const RTT_MAX_RING: u32 = 0x10_0000;

/// Drain/inject state for RTT channel 0. The control-block address is found
/// by scanning guest RAM once and cached; [`RttState::forget`] drops it.
#[derive(Clone, Copy, Debug, Default)]
pub struct RttState {
    control: Option<u32>,
}

impl RttState {
    /// Drop the cached control-block address (machine reset).
    pub fn forget(&mut self) {
        self.control = None;
    }

    fn find<B: Bus>(&mut self, mem: &mut B, base: u32, len: u32) -> Option<u32> {
        if let Some(addr) = self.control {
            if ring_valid(mem, addr) {
                return Some(addr);
            }
            self.control = None;
        }
        let end = base.saturating_add(len.min(MAX_TRANSFER as u32));
        let mut addr = base;
        while addr < end.saturating_sub(16) {
            let mut hit = true;
            for (i, &m) in RTT_MAGIC.iter().enumerate() {
                if mem.read(addr.wrapping_add(i as u32)).unwrap_or(!m) != m {
                    hit = false;
                    break;
                }
            }
            if hit && ring_valid(mem, addr) {
                self.control = Some(addr);
                return Some(addr);
            }
            addr = addr.wrapping_add(4);
        }
        None
    }

    /// Move new target→host bytes on up-channel 0 into the returned buffer,
    /// advancing its read offset past them.
    pub fn poll<B: Bus>(&mut self, mem: &mut B, base: u32, len: u32) -> Vec<u8> {
        let Some(ctrl) = self.find(mem, base, len) else {
            return Vec::new();
        };
        let desc = ctrl.wrapping_add(24);
        let (buf, size, wr, rd) = channel(mem, desc);
        let Some(mask) = ring_mask(size) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        let mut rd = rd;
        while rd != wr && out.len() < MAX_TRANSFER {
            if let Ok(b) = mem.read(buf.wrapping_add(rd & mask)) {
                out.push(b);
            }
            rd = rd.wrapping_add(1) & mask;
        }
        let _ = mem.write_word(desc.wrapping_add(16), rd);
        out
    }

    /// Move host bytes into down-channel 0, advancing its write offset.
    /// Returns the number of bytes accepted (ring may be fuller than input).
    pub fn inject<B: Bus>(&mut self, mem: &mut B, base: u32, len: u32, bytes: &[u8]) -> usize {
        let Some(ctrl) = self.find(mem, base, len) else {
            return 0;
        };
        let max_up = mem.read_word(ctrl.wrapping_add(16)).unwrap_or(0);
        let desc = ctrl
            .wrapping_add(24)
            .wrapping_add(max_up.saturating_mul(24));
        let (buf, size, wr, rd) = channel(mem, desc);
        let Some(mask) = ring_mask(size) else {
            return 0;
        };
        let free = rd.wrapping_sub(wr).wrapping_sub(1) & mask;
        let n = (free as usize).min(bytes.len());
        let mut wr = wr;
        for &b in &bytes[..n] {
            let _ = mem.write(buf.wrapping_add(wr & mask), b);
            wr = wr.wrapping_add(1) & mask;
        }
        let _ = mem.write_word(desc.wrapping_add(12), wr);
        n
    }
}

/// Up-channel 0 descriptor: buffer, size, write offset, read offset.
fn channel<B: Bus>(mem: &mut B, desc: u32) -> (u32, u32, u32, u32) {
    (
        mem.read_word(desc.wrapping_add(4)).unwrap_or(0),
        mem.read_word(desc.wrapping_add(8)).unwrap_or(0),
        mem.read_word(desc.wrapping_add(12)).unwrap_or(0),
        mem.read_word(desc.wrapping_add(16)).unwrap_or(0),
    )
}

/// A cached or candidate control block is usable: at least one up-channel
/// whose ring 0 is a power-of-two buffer inside the address space.
fn ring_valid<B: Bus>(mem: &mut B, ctrl: u32) -> bool {
    if mem.read_word(ctrl.wrapping_add(16)).unwrap_or(0) < 1 {
        return false;
    }
    let (buf, size, _, _) = channel(mem, ctrl.wrapping_add(24));
    buf != 0 && ring_mask(size).is_some()
}

/// Ring index mask, or `None` when the size is not a usable power of two.
fn ring_mask(size: u32) -> Option<u32> {
    if (8..=RTT_MAX_RING).contains(&size) && size.is_power_of_two() {
        Some(size - 1)
    } else {
        None
    }
}
