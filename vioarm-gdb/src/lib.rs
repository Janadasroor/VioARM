#![warn(missing_docs)]
//! A GDB Remote Serial Protocol (RSP) server over TCP for a `vioarm` machine.
//!
//! Exposes one `Machine` to an `arm-none-eabi-gdb` `target remote` session:
//! register and memory access, single-step, continue (with interruptible
//! stop), and software breakpoints. The register layout is delivered through
//! a target description (see [`serve`]) so GDB shows the Cortex-M register
//! names (`r0`..`pc`, `xpsr`, `msp`, `psp`, `primask`, `basepri`,
//! `faultmask`, `control`).
//!
//! The protocol core (`Session`) is transport-independent and unit-tested; the
//! TCP framing and socket loop live in [`serve`].

use std::collections::{HashMap, VecDeque};
use std::fmt::Write as _;
use std::io::{self, ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};

use vioarm_core::error::HaltCause;
use vioarm_core::jit::TbRunner;
use vioarm_core::machine::Machine;
use vioarm_core::mem::PeripheralBus;

/// The number of registers reported by `g` and addressed by `p`/`P`:
/// `r0`..`r15` (0..=15), `xpsr` (16), `msp` (17), `psp` (18), `primask` (19),
/// `basepri` (20), `faultmask` (21), `control` (22).
const REG_COUNT: u8 = 23;

/// The byte length of a full `g` packet (23 32-bit registers, one `G` entry).
const G_PACKET_BYTES: usize = REG_COUNT as usize * 4;

/// The Thumb `bkpt #0` encoding swapped in for software breakpoints.
const BKPT: u16 = 0xBE00;

/// The register names GDB shows after fetching this description.
const TARGET_XML: &str = r#"<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target>
  <architecture>armv7-m</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32"/>
    <reg name="r1" bitsize="32"/>
    <reg name="r2" bitsize="32"/>
    <reg name="r3" bitsize="32"/>
    <reg name="r4" bitsize="32"/>
    <reg name="r5" bitsize="32"/>
    <reg name="r6" bitsize="32"/>
    <reg name="r7" bitsize="32"/>
    <reg name="r8" bitsize="32"/>
    <reg name="r9" bitsize="32"/>
    <reg name="r10" bitsize="32"/>
    <reg name="r11" bitsize="32"/>
    <reg name="r12" bitsize="32"/>
    <reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32"/>
    <reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32"/>
    <reg name="msp" bitsize="32"/>
    <reg name="psp" bitsize="32"/>
    <reg name="primask" bitsize="32"/>
    <reg name="basepri" bitsize="32"/>
    <reg name="faultmask" bitsize="32"/>
    <reg name="control" bitsize="32"/>
  </feature>
</target>
"#;

/// Map a machine halt cause to the GDB stop-signal number (from
/// `<signal.h>`): 2 SIGINT, 4 SIGILL, 5 SIGTRAP, 6 SIGABRT, 11 SIGSEGV.
const fn gdb_signal(cause: HaltCause) -> u8 {
    match cause {
        HaltCause::Breakpoint | HaltCause::CycleBudgetExhausted => 5,
        HaltCause::UndefinedInstruction
        | HaltCause::NonThumbTarget
        | HaltCause::NotImplemented { .. } => 4,
        HaltCause::FetchAbort | HaltCause::BusFault { .. } | HaltCause::StackMisaligned { .. } => {
            11
        }
        HaltCause::UnhandledException => 6,
        // Exits report through the `W` packet, not a signal; the stop-signal
        // fallback is only for contexts that cannot carry an exit.
        HaltCause::SemihostExit { .. } => 5,
    }
}

/// A semihosting exit code clamped to the one byte RSP carries.
fn exit_byte(code: i32) -> u8 {
    u8::try_from(code).unwrap_or(1)
}

/// The value of a single hex nibble.
const fn hex_nibble(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// Parse up to eight hex digits as a `u32`.
fn hex_u32(s: &str) -> Option<u32> {
    let mut value = 0u32;
    for &c in s.as_bytes().iter().take(8) {
        value = value.checked_shl(4)?;
        value |= u32::from(hex_nibble(c)?);
    }
    Some(value)
}

/// Parse a one- or two-digit hex quantity as a `u8`.
fn hex_u8(s: &str) -> Option<u8> {
    u8::try_from(hex_u32(s)?).ok()
}

/// Parse `<addr>,<len>` (the tail of the `m`/`M`/`Z`/`z` packets).
fn mem_range(s: &str) -> Option<(u32, u32)> {
    let (addr, len) = s.split_once(',')?;
    Some((hex_u32(addr)?, hex_u32(len)?))
}

/// Append `value` to `out` as the hex bytes of its little-endian form, the
/// RSP wire order for a register value.
fn push_le_hex(out: &mut String, value: u32) {
    for b in value.to_le_bytes() {
        let _ = write!(out, "{b:02x}");
    }
}

/// The RSP packet checksum: the sum of the payload bytes.
fn checksum(payload: &[u8]) -> u8 {
    payload.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

/// What a request directed the server to do.
enum Outcome {
    /// Reply with this payload.
    Reply(String),
    /// Resume the machine; the machine has not moved yet.
    Resume { step: bool, addr: Option<u32> },
    /// End the session.
    Quit,
}

/// Why a resumed machine came back to the debugger.
enum Stop {
    Signal(u8),
    /// Firmware exited through semihosting; the byte is the exit code.
    Exited(u8),
    /// The client closed the connection while the machine was running.
    Disconnect,
}

/// A transport-independent RSP session against one machine.
pub struct Session<'a, M: PeripheralBus> {
    machine: &'a mut Machine<M>,
    /// Translated-block engine for JIT-mode stepping/continue (X2).
    runner: TbRunner,
    /// JIT mode: single-steps run translated blocks (per-insn stops from
    /// TB metadata) and continues run quanta; breakpoint pages bypass
    /// translation through X1 hook markers (see [`Session::set_jit`]).
    jit: bool,
    /// Queued per-insn stop PCs from the in-flight translated block.
    pending: VecDeque<u32>,
    /// Last reported stop PC (the user-visible PC while stops are queued;
    /// live regs already ran the block ahead).
    stop_pc: Option<u32>,
    /// The last stop signal, reported by a later `?`.
    last_signal: u8,
    /// `QStartNoAckMode` negotiated.
    no_ack: bool,
    /// Live software breakpoints: the address of the `bkpt` and the original
    /// halfword it replaced. The breakpoint bytes themselves sit in machine
    /// memory.
    bps: HashMap<u32, u16>,
}

impl<'a, M: PeripheralBus> Session<'a, M> {
    /// A session over `machine`, starting in the "stopped for debugging"
    /// state (a fresh connection behaves like an immediate SIGTRAP stop).
    #[must_use]
    pub fn new(machine: &'a mut Machine<M>) -> Self {
        Self {
            machine,
            runner: TbRunner::new(),
            jit: false,
            pending: VecDeque::new(),
            stop_pc: None,
            last_signal: 5,
            no_ack: false,
            bps: HashMap::new(),
        }
    }

    /// Enable or disable JIT-mode stepping/continue (X2). Switching
    /// modes flushes queued per-insn stops (they belong to already
    /// retired code under the other engine).
    pub fn set_jit(&mut self, on: bool) {
        self.jit = on;
        self.pending.clear();
    }

    /// Drop queued per-insn stops (live state changed underneath: memory
    /// or register writes, breakpoint updates, mode switches).
    fn flush_pending(&mut self) {
        self.pending.clear();
        self.stop_pc = None;
    }

    /// Read architectural register number `reg` (see [`REG_COUNT`]).
    fn read_reg(&self, reg: u8) -> Option<u32> {
        // While per-insn stops are queued, the user-visible PC is the
        // last reported stop (live regs already ran the block ahead).
        if reg == 15 {
            if let Some(pc) = self.stop_pc {
                return Some(pc);
            }
        }
        if reg < 16 {
            return Some(self.machine.core.regs.read(reg));
        }
        match reg {
            16 => Some(self.machine.core.xpsr.get()),
            17 => Some(self.machine.core.regs.msp),
            18 => Some(self.machine.core.regs.psp),
            19 => Some(u32::from(self.machine.core.primask)),
            20 => Some(u32::from(self.machine.core.basepri)),
            21 => Some(u32::from(self.machine.core.faultmask)),
            22 => Some(self.machine.core.control),
            _ => None,
        }
    }

    /// Write architectural register number `reg`.
    fn write_reg(&mut self, reg: u8, value: u32) -> Option<()> {
        if reg < 16 {
            self.machine.core.regs.write(reg, value);
            return Some(());
        }
        match reg {
            16 => {
                self.machine.core.xpsr.set(value);
                Some(())
            }
            17 => {
                self.machine.core.regs.msp = value;
                Some(())
            }
            18 => {
                self.machine.core.regs.psp = value;
                Some(())
            }
            19 => {
                self.machine.core.primask = value & 1 != 0;
                Some(())
            }
            20 => {
                self.machine.core.basepri = u8::try_from(value & 0xFF).unwrap_or(0);
                Some(())
            }
            21 => {
                self.machine.core.faultmask = value & 1 != 0;
                Some(())
            }
            22 => {
                self.machine.core.control = value;
                Some(())
            }
            _ => None,
        }
    }

    /// Read one byte of guest-visible memory.
    fn read_mem_byte(&mut self, addr: u32) -> Result<u8, HaltCause> {
        self.machine
            .core
            .mem()
            .read(addr)
            .map_err(|_| HaltCause::BusFault { address: addr })
    }

    /// Write one byte of guest-visible memory (flushes queued per-insn
    /// stops: patched bytes invalidate already-retired reports).
    fn write_mem_byte(&mut self, addr: u32, value: u8) -> Result<(), HaltCause> {
        self.flush_pending();
        self.machine
            .core
            .mem()
            .write(addr, value)
            .map_err(|_| HaltCause::BusFault { address: addr })
    }

    /// Handle one request and decide what the loop should send next.
    fn handle(&mut self, req: &str) -> Outcome {
        match req.as_bytes().first() {
            Some(b'g') => Outcome::Reply(self.packet_g()),
            Some(b'G') => match Self::parse_g(&req[1..]) {
                Some(regs) => {
                    for (i, value) in regs.into_iter().enumerate() {
                        let reg = u8::try_from(i).unwrap_or(0);
                        if self.write_reg(reg, value).is_none() {
                            return Outcome::Reply("E01".to_owned());
                        }
                    }
                    self.flush_pending();
                    Outcome::Reply("OK".to_owned())
                }
                None => Outcome::Reply("E01".to_owned()),
            },
            Some(b'p' | b'P') => self.handle_p(req),
            Some(b'm') => self.handle_m(req),
            Some(b'M') => self.handle_m_write(req),
            Some(b'c' | b's') => Outcome::Resume {
                step: req.as_bytes()[0] == b's',
                addr: if req.len() > 1 {
                    hex_u32(&req[1..])
                } else {
                    None
                },
            },
            Some(b'Z' | b'z') => self.handle_breakpoint(req),
            Some(b'H') => Outcome::Reply("OK".to_owned()),
            Some(b'D' | b'k') => Outcome::Quit,
            Some(b'?') => Outcome::Reply(format!("S{:02x}", self.last_signal)),
            Some(b'q') => Self::handle_query(req),
            Some(b'Q') => {
                if req == "QStartNoAckMode" {
                    self.no_ack = true;
                    Outcome::Reply("OK".to_owned())
                } else {
                    Outcome::Reply(String::new())
                }
            }
            _ => Outcome::Reply(String::new()),
        }
    }

    /// The `g` reply: every register in declaration order, little-endian.
    fn packet_g(&self) -> String {
        let mut out = String::with_capacity(G_PACKET_BYTES * 2);
        for reg in 0..REG_COUNT {
            push_le_hex(&mut out, self.read_reg(reg).unwrap_or(0));
        }
        out
    }

    /// Parse the little-endian register lanes of a `G` request.
    fn parse_g(data: &str) -> Option<Vec<u32>> {
        if !data.is_ascii() || data.len() < G_PACKET_BYTES * 2 {
            return None;
        }
        let mut regs = Vec::with_capacity(usize::from(REG_COUNT));
        for i in 0..usize::from(REG_COUNT) {
            let off = i * 8;
            let chunk = data.get(off..off + 8)?;
            let mut value = [0u8; 4];
            for (j, byte) in value.iter_mut().enumerate() {
                *byte = hex_nibble(chunk.as_bytes()[j * 2])? << 4
                    | hex_nibble(chunk.as_bytes()[j * 2 + 1])?;
            }
            regs.push(u32::from_le_bytes(value));
        }
        Some(regs)
    }

    /// `p <reg>` / `P <reg>=<value>`.
    fn handle_p(&mut self, req: &str) -> Outcome {
        let (prefix, body) = req.split_at(1);
        let value_hex = if prefix == "P" {
            body.split_once('=').map(|(_, v)| v)
        } else {
            None
        };
        let reg_hex = if prefix == "P" {
            body.split_once('=').map_or(body, |(r, _)| r)
        } else {
            body
        };
        let Some(reg) = hex_u8(reg_hex) else {
            return Outcome::Reply("E01".to_owned());
        };
        if let Some(value_hex) = value_hex {
            let Some(value) = hex_u32(value_hex) else {
                return Outcome::Reply("E01".to_owned());
            };
            if self.write_reg(reg, value).is_some() {
                self.flush_pending();
                Outcome::Reply("OK".to_owned())
            } else {
                Outcome::Reply("E01".to_owned())
            }
        } else {
            self.read_reg(reg).map_or_else(
                || Outcome::Reply("E01".to_owned()),
                |value| {
                    let mut out = String::with_capacity(8);
                    push_le_hex(&mut out, value);
                    Outcome::Reply(out)
                },
            )
        }
    }

    /// `m <addr>,<len>`.
    fn handle_m(&mut self, req: &str) -> Outcome {
        let Some((addr, len)) = mem_range(&req[1..]) else {
            return Outcome::Reply("E01".to_owned());
        };
        let len = usize::try_from(len).unwrap_or(0);
        if len > 8192 {
            return Outcome::Reply("E01".to_owned());
        }
        let mut out = String::with_capacity(len * 2);
        for i in 0..len {
            let off = u32::try_from(i).unwrap_or(0);
            match self.read_mem_byte(addr.wrapping_add(off)) {
                Ok(b) => {
                    let _ = write!(out, "{b:02x}");
                }
                Err(_) => return Outcome::Reply("E01".to_owned()),
            }
        }
        Outcome::Reply(out)
    }

    /// `M <addr>,<len>:<hexbytes>`.
    fn handle_m_write(&mut self, req: &str) -> Outcome {
        let Some(colon) = req.find(':') else {
            return Outcome::Reply("E01".to_owned());
        };
        let Some((addr, len)) = mem_range(&req[1..colon]) else {
            return Outcome::Reply("E01".to_owned());
        };
        let data = &req[colon + 1..];
        if !data.is_ascii() {
            return Outcome::Reply("E01".to_owned());
        }
        for i in 0..len {
            let off = usize::try_from(i).unwrap_or(0) * 2;
            let Some(hi) = hex_nibble(data.as_bytes()[off]) else {
                return Outcome::Reply("E01".to_owned());
            };
            let Some(lo) = hex_nibble(data.as_bytes()[off + 1]) else {
                return Outcome::Reply("E01".to_owned());
            };
            if self
                .write_mem_byte(addr.wrapping_add(i), hi << 4 | lo)
                .is_err()
            {
                return Outcome::Reply("E01".to_owned());
            }
        }
        Outcome::Reply("OK".to_owned())
    }

    /// `Z0 <addr>,2` inserts a software breakpoint; `z0` removes it. Other
    /// breakpoint/watchpoint types are unsupported.
    fn handle_breakpoint(&mut self, req: &str) -> Outcome {
        if req.get(1..2) != Some("0") {
            return Outcome::Reply(String::new());
        }
        // `req.get`, not slicing: short packets like `Z0` must reply, never
        // panic (found by `fuzz_rsp`: `$Z0#...` crashed the server).
        let Some((addr, len)) = req.get(3..).and_then(mem_range) else {
            return Outcome::Reply("E01".to_owned());
        };
        if len != 2 {
            return Outcome::Reply(String::new());
        }
        if req.starts_with('Z') {
            self.insert_breakpoint(addr)
        } else {
            self.remove_breakpoint(addr)
        }
    }

    fn insert_breakpoint(&mut self, addr: u32) -> Outcome {
        if self.bps.contains_key(&addr) {
            return Outcome::Reply("OK".to_owned());
        }
        let Ok(original) = self.machine.core.mem().read_half(addr) else {
            return Outcome::Reply("E01".to_owned());
        };
        self.bps.insert(addr, original);
        if self.machine.core.mem().write_half(addr, BKPT).is_err() {
            self.bps.remove(&addr);
            return Outcome::Reply("E01".to_owned());
        }
        // X2: breakpoint pages bypass translation through the X1 hook
        // mechanism (a marker pre-hook over the patched halfword). The
        // bus write already invalidated covering blocks; the marker keeps
        // them interpreted so the swapped `bkpt` always traps in the
        // stepper path with identical stops.
        self.machine
            .on_instruction(true, addr, addr.wrapping_add(2), |_| {});
        self.flush_pending();
        Outcome::Reply("OK".to_owned())
    }

    fn remove_breakpoint(&mut self, addr: u32) -> Outcome {
        let Some(original) = self.bps.remove(&addr) else {
            return Outcome::Reply("OK".to_owned());
        };
        if self.machine.core.mem().write_half(addr, original).is_err() {
            return Outcome::Reply("E01".to_owned());
        }
        self.machine
            .remove_instruction(true, addr, addr.wrapping_add(2));
        self.flush_pending();
        Outcome::Reply("OK".to_owned())
    }

    fn handle_query(req: &str) -> Outcome {
        if req.starts_with("qSupported") {
            return Outcome::Reply(
                "PacketSize=400;QStartNoAckMode+;qXfer:features:read+".to_owned(),
            );
        }
        if let Some(target) = req.strip_prefix("qXfer:features:read:") {
            return Self::xfer_features(target);
        }
        match req {
            "qAttached" => Outcome::Reply("0".to_owned()),
            "qC" => Outcome::Reply("QC1".to_owned()),
            "qfThreadInfo" => Outcome::Reply("m1".to_owned()),
            "qsThreadInfo" => Outcome::Reply("l".to_owned()),
            _ => Outcome::Reply(String::new()),
        }
    }

    /// Serve a `qXfer:features:read:<annex>:<offset>,<length>` request from
    /// the target-description blob, returning the whole unread chunk in one
    /// `l` (last) packet.
    fn xfer_features(spec: &str) -> Outcome {
        let (annex, range) = spec.split_once(':').map_or((spec, ""), |(a, r)| (a, r));
        if annex != "target.xml" {
            return Outcome::Reply("E00".to_owned());
        }
        let (offset, _len) = range.split_once(',').map_or((0, 0), |(o, l)| {
            (hex_u32(o).unwrap_or(0), hex_u32(l).unwrap_or(0))
        });
        let bytes = TARGET_XML.as_bytes();
        let mut start = usize::try_from(offset).unwrap_or(0);
        if start > bytes.len() {
            start = bytes.len();
        }
        let chunk = &bytes[start..];
        let mut out = String::with_capacity(chunk.len() * 2);
        for b in chunk {
            let _ = write!(out, "{b:02x}");
        }
        Outcome::Reply(format!("l{out}"))
    }

    /// Resume execution: apply an optional new PC, step over a software
    /// breakpoint sitting at the current PC, then run until a halt, a
    /// breakpoint, or a Ctrl-C interrupt arrives on `stream`.
    fn resume(&mut self, stream: &TcpStream, step: bool, addr: Option<u32>) -> io::Result<Stop> {
        if let Some(addr) = addr {
            self.machine.core.regs.write(15, addr);
            self.flush_pending();
        }
        if self.bps.contains_key(&self.machine.core.regs.pc) {
            if let Some(stop) = self.step_over(step) {
                return Ok(stop);
            }
        }
        stream.set_nonblocking(true)?;
        let outcome = self.run_until_stop(stream, step);
        stream.set_nonblocking(false)?;
        outcome
    }

    /// If the current PC holds a live software breakpoint, restore the
    /// original halfword, execute one step, and re-arm the breakpoint. This is
    /// what lets a debugger "continue from the breakpoint" without the BKPT
    /// immediately re-hitting. Returns `Some(stop)` when a machine fault was
    /// hit during the step, or when the single step *is* the requested step;
    /// returns `None` when the machine should keep running.
    fn step_over(&mut self, step: bool) -> Option<Stop> {
        self.flush_pending();
        let pc = self.machine.core.regs.pc;
        let original = self.bps.remove(&pc)?;
        let _ = self.write_mem_byte(pc, u8::try_from(original & 0xFF).unwrap_or(0));
        let _ = self.write_mem_byte(pc + 1, u8::try_from(original >> 8).unwrap_or(0));
        let result = self.machine.step();
        self.bps.insert(pc, original);
        let _ = self.write_mem_byte(pc, u8::try_from(BKPT & 0xFF).unwrap_or(0));
        let _ = self.write_mem_byte(pc + 1, u8::try_from(BKPT >> 8).unwrap_or(0));
        match result {
            Ok(()) => {
                if step {
                    self.last_signal = 5;
                    Some(Stop::Signal(self.last_signal))
                } else {
                    None
                }
            }
            Err(HaltCause::SemihostExit { code }) => Some(Stop::Exited(exit_byte(code))),
            Err(cause) => {
                self.last_signal = gdb_signal(cause);
                Some(Stop::Signal(self.last_signal))
            }
        }
    }

    /// Map a halt to a stop reply: software breakpoints (ours, swapped
    /// into the target) report SIGTRAP even when the core escalated them
    /// to a fault; everything else maps by cause. Shared by the stepper
    /// and JIT arms.
    fn map_halt(&mut self, cause: HaltCause) -> Stop {
        if let HaltCause::SemihostExit { code } = cause {
            return Stop::Exited(exit_byte(code));
        }
        if self.bps.contains_key(&self.machine.core.regs.pc) {
            self.last_signal = 5;
        } else {
            self.last_signal = gdb_signal(cause);
        }
        Stop::Signal(self.last_signal)
    }

    /// One JIT single-step (X2): drain a queued per-insn stop if present,
    /// else run one translated block and queue its stops (the executing
    /// step reports the first). `step_tb_block` decodes the retired op
    /// PCs from TB metadata; breakpoint-overlapping blocks never arrive
    /// here (X1 markers force the exact single-step path instead).
    fn jit_step(&mut self) -> Result<(), HaltCause> {
        if self.pending.is_empty() {
            let pcs = self.machine.step_tb_block(&mut self.runner)?;
            let end = self.machine.core.regs.pc;
            // pcs[0] is the address stepped FROM (already reported);
            // queue the landing PCs plus the final fallthrough.
            self.pending.extend(pcs.into_iter().skip(1));
            self.pending.push_back(end);
        }
        // The popped stop is what the user now sees (read_reg shadows it
        // until the queue drains onto the live PC).
        self.stop_pc = self.pending.pop_front();
        self.last_signal = 5;
        Ok(())
    }

    fn run_until_stop(&mut self, stream: &TcpStream, step: bool) -> io::Result<Stop> {
        loop {
            let mut peek = [0u8; 1];
            match TcpStream::peek(stream, &mut peek) {
                Ok(0) => return Ok(Stop::Disconnect),
                Ok(_) if peek[0] == 0x03 => return Ok(Stop::Signal(2)),
                Ok(_) => {}
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(e) => return Err(e),
            }
            if self.jit {
                if step {
                    match self.jit_step() {
                        Ok(()) => return Ok(Stop::Signal(self.last_signal)),
                        Err(cause) => return Ok(self.map_halt(cause)),
                    }
                }
                // Continue: quanta until halt/breakpoint/Ctrl-C (breakpoint
                // pages bypass via X1 markers, so swapped bkpts trap in
                // the stepper path with identical stops).
                match self.machine.run_tb_quantum(&mut self.runner, 1024) {
                    Ok(_) => {}
                    Err(cause) => return Ok(self.map_halt(cause)),
                }
                continue;
            }
            match self.machine.step() {
                Ok(()) => {
                    if step {
                        self.last_signal = 5;
                        return Ok(Stop::Signal(self.last_signal));
                    }
                }
                Err(HaltCause::SemihostExit { code }) => {
                    return Ok(Stop::Exited(exit_byte(code)));
                }
                Err(cause) => {
                    // A software breakpoint is the bkpt instruction swapped
                    // into the target. With core debug disabled that bkpt
                    // escalates to a fault (matching QEMU), but a debugger
                    // must see a breakpoint stop, not the underlying fault.
                    if self.bps.contains_key(&self.machine.core.regs.pc) {
                        self.last_signal = 5;
                    } else {
                        self.last_signal = gdb_signal(cause);
                    }
                    return Ok(Stop::Signal(self.last_signal));
                }
            }
        }
    }
}

/// Read one request packet from `stream`, skipping client acks and stray
/// bytes, and verifying the checksum before acknowledging. Returns `None`
/// when the connection closes.
fn read_packet(stream: &mut TcpStream, no_ack: bool) -> io::Result<Option<String>> {
    let mut byte = [0u8; 1];
    loop {
        let n = stream.read(&mut byte)?;
        if n == 0 {
            return Ok(None);
        }
        if byte[0] != b'$' {
            continue;
        }
        let mut payload = Vec::new();
        loop {
            stream.read_exact(&mut byte)?;
            if byte[0] == b'#' {
                break;
            }
            payload.push(byte[0]);
        }
        let mut check = [0u8; 2];
        stream.read_exact(&mut check)?;
        let expected = hex_nibble(check[0]).unwrap_or(0) << 4 | hex_nibble(check[1]).unwrap_or(0);
        if expected != checksum(&payload) {
            stream.write_all(b"-")?;
            continue;
        }
        if !no_ack {
            stream.write_all(b"+")?;
        }
        return Ok(Some(String::from_utf8_lossy(&payload).into_owned()));
    }
}

/// Send one reply packet on `stream`.
fn send_packet(stream: &mut TcpStream, payload: &str) -> io::Result<()> {
    let sum = checksum(payload.as_bytes());
    let mut frame = String::with_capacity(payload.len() + 4);
    let _ = write!(frame, "${payload}#{sum:02x}");
    stream.write_all(frame.as_bytes())
}

/// Accept one connection on `listener` and run a full debug session against
/// `machine`, reading requests until GDB detaches, kills, or disconnects.
///
/// # Errors
///
/// Returns the first socket I/O error that breaks the session, or the accept
/// error if the listener fails.
pub fn serve<M: PeripheralBus>(machine: &mut Machine<M>, listener: &TcpListener) -> io::Result<()> {
    serve_jit(machine, listener, false)
}

/// Accept one connection and run a full debug session, optionally in JIT
/// mode (X2: translated-block stepping with per-insn stops, quanta
/// continue, breakpoint pages bypassing translation).
///
/// # Errors
///
/// Returns the first socket I/O error that breaks the session, or the accept
/// error if the listener fails.
pub fn serve_jit<M: PeripheralBus>(
    machine: &mut Machine<M>,
    listener: &TcpListener,
    jit: bool,
) -> io::Result<()> {
    let (mut stream, _peer) = listener.accept()?;
    let mut session = Session::new(machine);
    session.set_jit(jit);
    loop {
        let Some(request) = read_packet(&mut stream, session.no_ack)? else {
            return Ok(());
        };
        match session.handle(&request) {
            Outcome::Reply(reply) => send_packet(&mut stream, &reply)?,
            Outcome::Resume { step, addr } => {
                let stop = session.resume(&stream, step, addr)?;
                match stop {
                    Stop::Signal(sig) => send_packet(&mut stream, &format!("S{sig:02x}"))?,
                    Stop::Exited(code) => {
                        send_packet(&mut stream, &format!("W{code:02x}"))?;
                        return Ok(());
                    }
                    Stop::Disconnect => return Ok(()),
                }
            }
            Outcome::Quit => {
                send_packet(&mut stream, "OK")?;
                return Ok(());
            }
        }
    }
}

/// Feed raw RSP-stream bytes through packet framing and request dispatch,
/// discarding replies. This is the fuzz-harness entry (`fuzz/fuzz_rsp`):
/// checksum failures drop the packet exactly like the socket path, `c`/`s`
/// resume packets retire at most one instruction each (total work bounded
/// by the input length), and `D`/`k` end the stream. Any panic on malformed
/// input is a fuzzer finding — the parse paths must be total.
pub fn fuzz_stream(data: &[u8]) {
    use vioarm_core::mem::flat::FlatMemory;
    let mem = FlatMemory::with_layout(&[0, 0x2000_0000], &[0x2000, 0x4000]);
    let mut machine = Machine::new(mem);
    machine.core.regs.pc = 0x100;
    machine.core.regs.sp = 0x2000_3F00;
    let mut session = Session::new(&mut machine);
    let mut i = 0;
    while i < data.len() {
        // Frame like `read_packet`: skip to `$`, payload to `#`, then two
        // checksum bytes; a bad checksum drops the packet.
        if data[i] != b'$' {
            i += 1;
            continue;
        }
        let mut j = i + 1;
        while j < data.len() && data[j] != b'#' {
            j += 1;
        }
        if j + 2 >= data.len() {
            break;
        }
        let payload = &data[i + 1..j];
        let expected =
            hex_nibble(data[j + 1]).unwrap_or(0) << 4 | hex_nibble(data[j + 2]).unwrap_or(0);
        i = j + 3;
        if expected != checksum(payload) {
            continue;
        }
        let request = String::from_utf8_lossy(payload).into_owned();
        match session.handle(&request) {
            Outcome::Reply(_) => {}
            Outcome::Resume { .. } => {
                let _ = session.machine.step();
            }
            Outcome::Quit => break,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::time::Duration;

    use vioarm_core::machine::Machine;
    use vioarm_core::mem::flat::FlatMemory;
    use vioarm_core::mem::Bus as _;

    use super::{checksum, gdb_signal, hex_u32, mem_range, serve, serve_jit, HaltCause};

    /// Send one request packet (with correct framing) and read the reply.
    fn exch(stream: &mut std::net::TcpStream, payload: &str) -> String {
        let sum = checksum(payload.as_bytes());
        stream
            .write_all(format!("${payload}#{sum:02x}").as_bytes())
            .unwrap();
        read_reply(stream)
    }

    /// Read one reply packet, skipping acks.
    fn read_reply(stream: &mut std::net::TcpStream) -> String {
        let mut buf = Vec::new();
        let mut byte = [0u8; 1];
        loop {
            stream.read_exact(&mut byte).unwrap();
            if byte[0] == b'$' {
                break;
            }
        }
        loop {
            stream.read_exact(&mut byte).unwrap();
            if byte[0] == b'#' {
                break;
            }
            buf.push(byte[0]);
        }
        let mut check = [0u8; 2];
        stream.read_exact(&mut check).unwrap();
        String::from_utf8(buf).unwrap()
    }

    /// A machine with a 2 KiB flash and 16 KiB RAM, pc parked at `0x100`.
    fn test_machine() -> Machine<FlatMemory> {
        let mem = FlatMemory::with_layout(&[0, 0x2000_0000], &[0x2000, 0x4000]);
        let mut machine = Machine::new(mem);
        machine.core.regs.pc = 0x100;
        machine.core.regs.sp = 0x2000_3F00;
        machine
    }

    /// Some CI/sandboxed environments forbid loopback `connect()` (Windows
    /// returns `10013 WSAEACCES`). The socket tests would otherwise hang on
    /// `accept()` after the client thread panics, so probe loopback first and
    /// skip when it is unavailable.
    fn loopback_available() -> bool {
        let Ok(listener) = TcpListener::bind(("127.0.0.1", 0)) else {
            return false;
        };
        let addr = listener.local_addr().unwrap();
        std::net::TcpStream::connect(addr).is_ok()
    }

    fn write_half(machine: &mut Machine<FlatMemory>, addr: u32, op: u16) {
        machine.core.mem().write_half(addr, op).unwrap();
    }

    #[test]
    fn hex_parsing() {
        assert_eq!(hex_u32("0"), Some(0));
        assert_eq!(hex_u32("3ff"), Some(0x3FF));
        assert_eq!(hex_u32("20000000"), Some(0x2000_0000));
        assert_eq!(hex_u32("zz"), None);
        assert_eq!(hex_u32(""), Some(0));
        assert_eq!(mem_range("100,4"), Some((0x100, 4)));
        assert_eq!(mem_range("100"), None);
        // The packet checksum is the byte sum: 'O'(0x4f) + 'K'(0x4b) = 0x9a.
        assert_eq!(checksum(b"OK"), 0x9A);
        assert_eq!(gdb_signal(HaltCause::Breakpoint), 5);
        assert_eq!(gdb_signal(HaltCause::BusFault { address: 0 }), 11);
        assert_eq!(gdb_signal(HaltCause::UndefinedInstruction), 4);
    }

    /// The fuzz entry never panics: garbage bytes, truncated frames, bad
    /// checksums, and a well-formed mixed session all run to completion.
    #[test]
    fn fuzz_stream_never_panics() {
        use super::fuzz_stream;
        fuzz_stream(b"");
        fuzz_stream(b"\xff\x00\x80$$$$$$$$");
        fuzz_stream(b"$g#");
        fuzz_stream(b"$g#67$");
        // Regression: a bare `$Z0` crashed `handle_breakpoint` on `&req[3..]`
        // (found by `fuzz_rsp` 5-minute soak). Short `z0` and truncated
        // breakpoint forms reply instead of panicking.
        fuzz_stream(b"$Z0#8a");
        fuzz_stream(b"$z0#aa");
        fuzz_stream(b"$Z0,100#47");
        fuzz_stream(b"$m0,0#00$M0,0:#00$qX#00$?#00$c#00$s#00$D#00$k#00");
        // A well-formed session: g, m, M, Z/z breakpoints, step, query.
        let mut session = Vec::new();
        for payload in [
            "g",
            "m100,4",
            "M100,4:deadbeef",
            "Z0,100,2",
            "z0,100,2",
            "p0",
            "P0=12345678",
            "qSupported",
            "?",
            "s",
            "c",
            "Hc0",
            "D",
        ] {
            session.push(b'$');
            session.extend_from_slice(payload.as_bytes());
            session.push(b'#');
            let sum: u8 = payload.bytes().fold(0u8, |a, b| a.wrapping_add(b));
            session.extend_from_slice(format!("{sum:02x}").as_bytes());
        }
        fuzz_stream(&session);
    }

    /// A scripted GDB session over a loopback TCP socket: register and memory
    /// access, a single step, a software breakpoint hit on continue (both from
    /// the breakpoint itself and from a fresh PC), and a detach. The guest is
    /// an infinite `movs r0,#1 ; b .` loop rewritten in place.
    #[test]
    fn loopback_session_steps_and_hits_breakpoint() {
        if !loopback_available() {
            eprintln!("skipping: loopback connect unavailable in this environment");
            return;
        }
        let mut machine = test_machine();
        write_half(&mut machine, 0x100, 0x2001); // movs r0, #1
        write_half(&mut machine, 0x102, 0xE7FD); // b 0x100

        let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
        let thread = std::thread::spawn(move || {
            let mut c = std::net::TcpStream::connect(addr).unwrap();
            assert!(exch(&mut c, "qSupported").starts_with("PacketSize="));
            let desc = exch(&mut c, "qXfer:features:read:target.xml:0,ffff");
            assert!(desc.starts_with('l'));
            assert!(desc.to_ascii_lowercase().contains("78707372")); // "xpsr"
            assert_eq!(exch(&mut c, "?"), "S05");
            // The `g` reply holds 23 little-endian 32-bit registers.
            let g = exch(&mut c, "g");
            assert_eq!(g.len(), 23 * 8);
            assert_eq!(&g[15 * 8..15 * 8 + 8], "00010000"); // pc == 0x100
            assert_eq!(exch(&mut c, "m100,2"), "0120"); // movs r0, #1
            assert_eq!(exch(&mut c, "s"), "S05");
            assert_eq!(&exch(&mut c, "g")[15 * 8..15 * 8 + 8], "02010000"); // pc == 0x102
            assert_eq!(exch(&mut c, "Z0,102,2"), "OK");
            assert_eq!(exch(&mut c, "m102,2"), "00be"); // bkpt #0, little-endian
            assert_eq!(exch(&mut c, "c"), "S05");
            assert_eq!(&exch(&mut c, "g")[15 * 8..15 * 8 + 8], "02010000"); // back at the bp
                                                                            // Resume from the instruction *before* the breakpoint so the
                                                                            // continue genuinely runs onto it.
            assert_eq!(exch(&mut c, "P0f=00000100"), "OK");
            assert_eq!(exch(&mut c, "c"), "S05");
            let g = exch(&mut c, "g");
            assert_eq!(&g[15 * 8..15 * 8 + 8], "02010000"); // pc == 0x102
            assert_eq!(&g[..8], "01000000"); // r0 == 1 from movs r0, #1
            assert_eq!(exch(&mut c, "z0,102,2"), "OK");
            assert_eq!(exch(&mut c, "m102,2"), "fde7"); // b 0x100 restored
            assert_eq!(exch(&mut c, "D"), "OK");
        });
        serve(&mut machine, &listener).unwrap();
        thread.join().unwrap();
        assert_eq!(machine.core.regs.pc, 0x102);
        assert_eq!(machine.core.regs.read(0), 1);
    }

    /// A JIT session (`serve_jit(.., true)`): single-steps run translated
    /// blocks but report per-insn stops (with the user-visible PC tracking
    /// the reported stop), and continues run quanta onto breakpoints
    /// whose pages bypass translation.
    #[test]
    fn jit_session_steps_blocks_and_hits_breakpoints() {
        if !loopback_available() {
            eprintln!("skipping: loopback connect unavailable in this environment");
            return;
        }
        let mut machine = test_machine();
        write_half(&mut machine, 0x100, 0x2001); // movs r0, #1
        write_half(&mut machine, 0x102, 0x2102); // movs r1, #2
        write_half(&mut machine, 0x104, 0xE7FC); // b 0x100
        let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
        let thread = std::thread::spawn(move || {
            let mut c = std::net::TcpStream::connect(addr).unwrap();
            // Three retiring ops in one translated block: three stops.
            assert_eq!(exch(&mut c, "s"), "S05");
            assert_eq!(&exch(&mut c, "g")[15 * 8..15 * 8 + 8], "02010000"); // 0x102
            assert_eq!(exch(&mut c, "s"), "S05");
            assert_eq!(&exch(&mut c, "g")[15 * 8..15 * 8 + 8], "04010000"); // 0x104
            assert_eq!(exch(&mut c, "s"), "S05");
            let g = exch(&mut c, "g");
            assert_eq!(&g[15 * 8..15 * 8 + 8], "00010000"); // back at 0x100
            assert_eq!(&g[..8], "01000000"); // r0 == 1 went native
            assert_eq!(&g[8..16], "02000000"); // r1 == 2
                                               // Breakpoint at the loop head: continue runs quanta onto it
                                               // through the bypassed (interpreted) page.
            assert_eq!(exch(&mut c, "Z0,100,2"), "OK");
            assert_eq!(exch(&mut c, "P0f=00000100"), "OK"); // pc = 0x100
            assert_eq!(exch(&mut c, "c"), "S05");
            assert_eq!(&exch(&mut c, "g")[15 * 8..15 * 8 + 8], "00010000");
            assert_eq!(exch(&mut c, "z0,100,2"), "OK");
            assert_eq!(exch(&mut c, "m100,2"), "0120"); // movs restored
                                                        // With the marker withdrawn the page translates again:
                                                        // stepping still reports per-insn stops.
            assert_eq!(exch(&mut c, "s"), "S05");
            assert_eq!(&exch(&mut c, "g")[15 * 8..15 * 8 + 8], "02010000");
            assert_eq!(exch(&mut c, "D"), "OK");
        });
        serve_jit(&mut machine, &listener, true).unwrap();
        thread.join().unwrap();
    }

    /// A semihosting `SYS_EXIT` under continue ends the session with a `W`
    /// packet carrying the exit code.
    #[test]
    fn semihost_exit_reports_w_packet() {
        use vioarm_core::mem::Bus as _;
        if !loopback_available() {
            eprintln!("skipping: loopback connect unavailable in this environment");
            return;
        }
        let mut machine = test_machine();
        machine.semihost_enable(true);
        write_half(&mut machine, 0x100, 0x2018); // movs r0, #0x18 (SYS_EXIT)
        write_half(&mut machine, 0x102, 0x4901); // ldr r1, [pc, #4] -> 0x108
        write_half(&mut machine, 0x104, 0xBEAB); // bkpt 0xAB
        machine.core.mem().write_word(0x108, 0x110).unwrap();
        machine.core.mem().write_word(0x110, 0x20026).unwrap(); // ADP_EXIT
        machine.core.mem().write_word(0x114, 0).unwrap();
        let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
        let thread = std::thread::spawn(move || {
            let mut c = std::net::TcpStream::connect(addr).unwrap();
            assert_eq!(exch(&mut c, "c"), "W00");
        });
        serve(&mut machine, &listener).unwrap();
        thread.join().unwrap();
    }

    /// A continue into an infinite loop is interruptible with Ctrl-C.
    #[test]
    fn continue_is_interruptible() {
        if !loopback_available() {
            eprintln!("skipping: loopback connect unavailable in this environment");
            return;
        }
        let mut machine = test_machine();
        write_half(&mut machine, 0x100, 0xE7FE); // b .
        let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let addr = format!("127.0.0.1:{}", listener.local_addr().unwrap().port());
        let thread = std::thread::spawn(move || {
            let mut c = std::net::TcpStream::connect(addr).unwrap();
            let sum = checksum(b"c");
            c.write_all(format!("$c#{sum:02x}").as_bytes()).unwrap();
            std::thread::sleep(Duration::from_millis(80));
            c.write_all(&[0x03]).unwrap();
            assert_eq!(read_reply(&mut c), "S02");
            assert_eq!(exch(&mut c, "D"), "OK");
        });
        serve(&mut machine, &listener).unwrap();
        thread.join().unwrap();
    }
}
