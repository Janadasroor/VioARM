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

use std::collections::HashMap;
use std::fmt::Write as _;
use std::io::{self, ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};

use vioarm_core::error::HaltCause;
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
    }
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
    /// The client closed the connection while the machine was running.
    Disconnect,
}

/// A transport-independent RSP session against one machine.
pub struct Session<'a, M: PeripheralBus> {
    machine: &'a mut Machine<M>,
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
            last_signal: 5,
            no_ack: false,
            bps: HashMap::new(),
        }
    }

    /// Read architectural register number `reg` (see [`REG_COUNT`]).
    fn read_reg(&self, reg: u8) -> Option<u32> {
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

    /// Write one byte of guest-visible memory.
    fn write_mem_byte(&mut self, addr: u32, value: u8) -> Result<(), HaltCause> {
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
        let Some((addr, len)) = mem_range(&req[3..]) else {
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
        Outcome::Reply("OK".to_owned())
    }

    fn remove_breakpoint(&mut self, addr: u32) -> Outcome {
        let Some(original) = self.bps.remove(&addr) else {
            return Outcome::Reply("OK".to_owned());
        };
        if self.machine.core.mem().write_half(addr, original).is_err() {
            return Outcome::Reply("E01".to_owned());
        }
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
            Err(cause) => {
                self.last_signal = gdb_signal(cause);
                Some(Stop::Signal(self.last_signal))
            }
        }
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
            match self.machine.step() {
                Ok(()) => {
                    if step {
                        self.last_signal = 5;
                        return Ok(Stop::Signal(self.last_signal));
                    }
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
    let (mut stream, _peer) = listener.accept()?;
    let mut session = Session::new(machine);
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

#[cfg(test)]
mod tests {
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::time::Duration;

    use vioarm_core::machine::Machine;
    use vioarm_core::mem::flat::FlatMemory;
    use vioarm_core::mem::Bus as _;

    use super::{checksum, gdb_signal, hex_u32, mem_range, serve, HaltCause};

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
        match std::net::TcpStream::connect(addr) {
            Ok(_) => true,
            Err(_) => false,
        }
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
