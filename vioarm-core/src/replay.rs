//! Golden-trace record/replay: re-execute a loaded machine deterministically
//! and diff the outcome.
//!
//! The machine is single-threaded and deterministic, so replay collapses to
//! re-running the same image under the same configuration: no checkpoints,
//! no event logs (those buy seeking on nondeterministic emulators; here
//! they would only slow the happy path). A [`Golden`] captures the image
//! digest, the final halt/counters, drained host channels, and digests of
//! RAM spans; [`replay`] re-runs and reports field-by-field diffs (empty =
//! match). A deliberately mutated model — or a `--patch` memory override —
//! shows up as a diff, which is the whole point.
//!
//! The on-disk format is plain `key value` lines (`vioarm-golden-v1`),
//! git-diffable by design and parsed without dependencies:
//!
//! ```text
//! vioarm-golden-v1
//! board stm32f103
//! base 0
//! budget 1000000
//! semihost false
//! image-sha256 <64 hex>
//! halt breakpoint
//! cycles 1234
//! cyccnt 1567
//! channel usart1 <hex bytes, may be empty>
//! semihost-out <hex bytes, may be empty>
//! snap 0x20000000:0x1000 <16 hex (sha256 truncated to 64 bits)>
//! ```

use crate::board::BoardMemory;
use crate::device::stm32::hash::sha256;
use crate::machine::Machine;
use crate::mem::Bus;

/// Which machine produced a golden trace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecordMeta {
    /// Board name as the CLI spells it (e.g. `"stm32f103"`).
    pub board: String,
    /// Image load base for flat binaries.
    pub base: u32,
    /// Run budget in rounds.
    pub budget: u64,
    /// Whether semihosting was armed.
    pub semihost: bool,
}

/// The outcome of one run: counters, halts, channels, RAM digests.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Golden {
    /// Format marker, always `"vioarm-golden-v1"`.
    pub version: String,
    /// See [`RecordMeta`].
    pub board: String,
    /// See [`RecordMeta`].
    pub base: u32,
    /// See [`RecordMeta`].
    pub budget: u64,
    /// See [`RecordMeta`].
    pub semihost: bool,
    /// SHA-256 of the loaded image bytes, hex.
    pub image_sha256: String,
    /// Halt cause display (`"breakpoint"`, `"cycle budget exhausted"`, ...).
    pub halt: String,
    /// Executed machine cycles.
    pub cycles: u64,
    /// `DWT CYCCNT`.
    pub cyccnt: u64,
    /// Non-empty drained host channels, `(name, hex bytes)`, sorted by name.
    pub channels: Vec<(String, String)>,
    /// Semihosting output, hex (may be empty).
    pub semihost_out: String,
    /// RAM span digests, `(base, len, hex64)`, in request order.
    pub snaps: Vec<(u32, u32, String)>,
}

fn hex(bytes: &[u8]) -> String {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(2 * bytes.len());
    for &b in bytes {
        out.push(DIGITS[(b >> 4) as usize] as char);
        out.push(DIGITS[(b & 0xF) as usize] as char);
    }
    out
}

fn unhex(s: &str) -> Result<Vec<u8>, String> {
    if s.len() % 2 != 0 {
        return Err(format!("odd hex length: {s}"));
    }
    let mut out = Vec::with_capacity(s.len() / 2);
    for i in (0..s.len()).step_by(2) {
        let byte = u8::from_str_radix(&s[i..i + 2], 16)
            .map_err(|_| format!("bad hex byte: {}", &s[i..i + 2]))?;
        out.push(byte);
    }
    Ok(out)
}

fn sha256_hex(bytes: &[u8]) -> String {
    let mut flat = Vec::with_capacity(32);
    for w in sha256(bytes) {
        flat.extend(w.to_be_bytes());
    }
    hex(&flat)
}

/// 64-bit digest of a RAM span (SHA-256 truncated, hex).
fn span_digest(mem: &mut BoardMemory, base: u32, len: u32) -> Result<String, String> {
    let mut bytes = Vec::new();
    for i in 0..len {
        let addr = base.wrapping_add(i);
        bytes.push(
            mem.read(addr)
                .map_err(|e| format!("snap {addr:#010x}: {e}"))?,
        );
    }
    Ok(sha256_hex(&bytes)[..16].to_owned())
}

impl Golden {
    /// Serialize to the `vioarm-golden-v1` line format.
    #[must_use]
    pub fn serialize(&self) -> String {
        let mut out = String::new();
        out.push_str("vioarm-golden-v1\n");
        out.push_str(&format!("board {}\n", self.board));
        out.push_str(&format!("base {}\n", self.base));
        out.push_str(&format!("budget {}\n", self.budget));
        out.push_str(&format!("semihost {}\n", self.semihost));
        out.push_str(&format!("image-sha256 {}\n", self.image_sha256));
        out.push_str(&format!("halt {}\n", self.halt));
        out.push_str(&format!("cycles {}\n", self.cycles));
        out.push_str(&format!("cyccnt {}\n", self.cyccnt));
        for (name, bytes) in &self.channels {
            out.push_str(&format!("channel {name} {bytes}\n"));
        }
        out.push_str(&format!("semihost-out {}\n", self.semihost_out));
        for (base, len, digest) in &self.snaps {
            out.push_str(&format!("snap {base:#010x}:{len:#x} {digest}\n"));
        }
        out
    }

    /// Parse the `vioarm-golden-v1` line format (unknown lines are an
    /// error; forward compatibility comes from versioned filenames, not
    /// silent skipping).
    ///
    /// # Errors
    ///
    /// Returns a message for malformed input.
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        match lines.next() {
            Some("vioarm-golden-v1") => {}
            other => return Err(format!("bad golden header: {other:?}")),
        }
        let mut get = |key: &str, lines: &mut std::str::Lines<'_>| -> Result<String, String> {
            lines
                .next()
                .ok_or_else(|| format!("missing {key}"))?
                .strip_prefix(&format!("{key} "))
                .map(str::to_owned)
                .ok_or_else(|| format!("bad {key} line"))
        };
        let board = get("board", &mut lines)?;
        let base: u32 = get("base", &mut lines)?
            .parse()
            .map_err(|_| "bad base".to_owned())?;
        let budget: u64 = get("budget", &mut lines)?
            .parse()
            .map_err(|_| "bad budget".to_owned())?;
        let semihost: bool = get("semihost", &mut lines)?
            .parse()
            .map_err(|_| "bad semihost".to_owned())?;
        let image_sha256 = get("image-sha256", &mut lines)?;
        if image_sha256.len() != 64 || unhex(&image_sha256).is_err() {
            return Err("bad image-sha256".to_owned());
        }
        let halt = get("halt", &mut lines)?;
        let cycles: u64 = get("cycles", &mut lines)?
            .parse()
            .map_err(|_| "bad cycles".to_owned())?;
        let cyccnt: u64 = get("cyccnt", &mut lines)?
            .parse()
            .map_err(|_| "bad cyccnt".to_owned())?;
        let mut channels = Vec::new();
        let mut semihost_out = None;
        let mut snaps = Vec::new();
        for line in lines {
            if let Some(rest) = line.strip_prefix("channel ") {
                let (name, bytes) = rest.split_once(' ').ok_or("bad channel line")?;
                unhex(bytes).map_err(|_| "bad channel hex".to_owned())?;
                channels.push((name.to_owned(), bytes.to_owned()));
            } else if let Some(bytes) = line.strip_prefix("semihost-out ") {
                if semihost_out.is_some() {
                    return Err("duplicate semihost-out".to_owned());
                }
                unhex(bytes).map_err(|_| "bad semihost-out hex".to_owned())?;
                semihost_out = Some(bytes.to_owned());
            } else if let Some(rest) = line.strip_prefix("snap ") {
                let (span, digest) = rest.split_once(' ').ok_or("bad snap line")?;
                let (base, len) = span.split_once(':').ok_or("bad snap span")?;
                let base = parse_u32(base)?;
                let len = parse_u32(len)?;
                unhex(digest).map_err(|_| "bad snap hex".to_owned())?;
                snaps.push((base, len, digest.to_owned()));
            } else {
                return Err(format!("unknown golden line: {line}"));
            }
        }
        Ok(Self {
            version: "vioarm-golden-v1".to_owned(),
            board,
            base,
            budget,
            semihost,
            image_sha256,
            halt,
            cycles,
            cyccnt,
            channels,
            semihost_out: semihost_out.unwrap_or_default(),
            snaps,
        })
    }
}

fn parse_u32(s: &str) -> Result<u32, String> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).map_err(|_| format!("bad number: {s}"))
    } else {
        s.parse().map_err(|_| format!("bad number: {s}"))
    }
}

/// Parse a `ADDR=VAL` memory patch (both hex or decimal, value is one byte).
///
/// # Errors
///
/// Returns a message for malformed input.
pub fn parse_patch(s: &str) -> Result<(u32, u8), String> {
    let (addr, val) = s.split_once('=').ok_or_else(|| format!("bad patch: {s}"))?;
    let addr = parse_u32(addr)?;
    let val = parse_u32(val)?;
    if val > 0xFF {
        return Err(format!("patch value out of byte range: {s}"));
    }
    #[allow(clippy::cast_possible_truncation)]
    Ok((addr, val as u8))
}

/// Parse a `BASE:LEN` RAM span (hex or decimal).
///
/// # Errors
///
/// Returns a message for malformed input.
pub fn parse_snap(s: &str) -> Result<(u32, u32), String> {
    let (base, len) = s.split_once(':').ok_or_else(|| format!("bad snap: {s}"))?;
    Ok((parse_u32(base)?, parse_u32(len)?))
}

/// Run a prepared machine to `meta.budget` and capture the golden trace.
/// `image` is the loaded bytes (digested, not reloaded); `snaps` are RAM
/// spans to digest. The machine must already be loaded and reset.
pub fn record(
    m: &mut Machine<BoardMemory>,
    meta: &RecordMeta,
    image: &[u8],
    snaps: &[(u32, u32)],
) -> Result<Golden, String> {
    let err = m.run(meta.budget);
    let halt = match err {
        Err(cause) => format!("{cause}"),
        Ok(()) => unreachable!("run always halts"),
    };
    let mut channels: Vec<(String, String)> = Vec::new();
    for device in m.core.mem().devices_mut() {
        let out = device.drain_output();
        if !out.is_empty() {
            channels.push((device.name().to_owned(), hex(&out)));
        }
    }
    channels.sort();
    let mut snap_digests = Vec::new();
    for (base, len) in snaps {
        let digest = span_digest(m.core.mem(), *base, *len)?;
        snap_digests.push((*base, *len, digest));
    }
    Ok(Golden {
        version: "vioarm-golden-v1".to_owned(),
        board: meta.board.clone(),
        base: meta.base,
        budget: meta.budget,
        semihost: meta.semihost,
        image_sha256: sha256_hex(image),
        halt,
        cycles: m.cycles(),
        cyccnt: m.core.cyccnt,
        channels,
        semihost_out: hex(&m.semihost_output()),
        snaps: snap_digests,
    })
}

/// Re-run a prepared machine and diff against `golden`: image digest,
/// halt, counters, channels, semihost output, and RAM spans. `patches`
/// apply before running (a deliberately mutated model). Empty means match.
pub fn replay(
    m: &mut Machine<BoardMemory>,
    golden: &Golden,
    image: &[u8],
    patches: &[(u32, u8)],
    snaps: &[(u32, u32)],
) -> Vec<String> {
    let mut diffs = Vec::new();
    if sha256_hex(image) != golden.image_sha256 {
        diffs.push("image digest differs from golden".to_owned());
        return diffs;
    }
    for (addr, val) in patches {
        // Reserved drops are the caller's bug; the diff below still speaks.
        let _ = m.core.mem().write(*addr, *val);
    }
    let meta = RecordMeta {
        board: golden.board.clone(),
        base: golden.base,
        budget: golden.budget,
        semihost: golden.semihost,
    };
    let rerun = match record(m, &meta, image, snaps) {
        Ok(g) => g,
        Err(e) => {
            diffs.push(format!("replay failed: {e}"));
            return diffs;
        }
    };
    if rerun.halt != golden.halt {
        diffs.push(format!(
            "halt: golden {} != replay {}",
            golden.halt, rerun.halt
        ));
    }
    if rerun.cycles != golden.cycles {
        diffs.push(format!(
            "cycles: golden {} != replay {}",
            golden.cycles, rerun.cycles
        ));
    }
    if rerun.cyccnt != golden.cyccnt {
        diffs.push(format!(
            "cyccnt: golden {} != replay {}",
            golden.cyccnt, rerun.cyccnt
        ));
    }
    if rerun.channels != golden.channels {
        diffs.push(format!(
            "channels: golden {:?} != replay {:?}",
            golden.channels, rerun.channels
        ));
    }
    if rerun.semihost_out != golden.semihost_out {
        diffs.push("semihost output differs".to_owned());
    }
    // Spans compare as full tuples (CLI replays golden's own spans, so a
    // base/len mismatch here means the caller moved them).
    if rerun.snaps != golden.snaps {
        for g in &golden.snaps {
            if !rerun.snaps.contains(g) {
                diffs.push(format!("snap {:#010x}:{:#x} differs", g.0, g.1));
            }
        }
        if rerun.snaps.len() != golden.snaps.len() {
            diffs.push(format!(
                "snap count: golden {} != replay {}",
                golden.snaps.len(),
                rerun.snaps.len()
            ));
        }
    }
    diffs
}

#[cfg(test)]
mod tests {
    use super::{parse_patch, parse_snap, record, replay, Golden, RecordMeta};
    use crate::board::{self, BoardMemory};
    use crate::loader::load_flat;
    use crate::machine::Machine;

    const RAM: u32 = 0x2000_0000;
    const SNAP: u32 = 0x2000_0100;

    /// NOP; MOVS r0, #0x5A; STR r0, [r1]; B . (spin). `r1` points at
    /// `SNAP`. Runs out the budget deterministically (a bare `BKPT`
    /// without a debugger HardFaults by design, QEMU parity).
    fn image() -> Vec<u8> {
        [0xBF00u16, 0x205A, 0x6008, 0xE7FE]
            .iter()
            .flat_map(|h| (*h).to_le_bytes())
            .collect()
    }

    fn machine() -> Machine<BoardMemory> {
        let mut m = Machine::armv7m(board::stm32f103());
        m.load_image(&load_flat(RAM, &image()));
        m.core.regs.pc = RAM;
        m.core.xpsr.set(0x0100_0000);
        m.core.regs.r[1] = SNAP;
        m
    }

    fn meta() -> RecordMeta {
        RecordMeta {
            board: "stm32f103".to_owned(),
            base: RAM,
            budget: 1_000,
            semihost: false,
        }
    }

    #[test]
    fn record_then_replay_matches() {
        let mut m = machine();
        let golden = record(&mut m, &meta(), &image(), &[(SNAP, 4)]).unwrap();
        assert_eq!(golden.halt, "cycle budget exhausted");
        assert_eq!(golden.cycles, 1_000);
        assert_eq!(golden.snaps.len(), 1);
        let mut m2 = machine();
        let diffs = replay(&mut m2, &golden, &image(), &[], &[(SNAP, 4)]);
        assert_eq!(diffs, Vec::<String>::new(), "clean replay matches");
        // Serialize/parse round-trips exactly.
        let text = golden.serialize();
        assert_eq!(Golden::parse(&text).unwrap(), golden);
    }

    #[test]
    fn replay_detects_a_mutated_model() {
        let mut m = machine();
        let golden = record(&mut m, &meta(), &image(), &[(SNAP, 4)]).unwrap();
        // A patch stands in for a mutated model: the MOVS immediate
        // (image byte 2) becomes 0, so the snapshot stores 0x00. Control
        // flow, halt, and counters are untouched — only the span differs.
        let mut m2 = machine();
        let diffs = replay(&mut m2, &golden, &image(), &[(RAM + 2, 0x00)], &[(SNAP, 4)]);
        assert!(!diffs.is_empty(), "mutation must show as a diff");
        assert!(
            diffs.iter().any(|d| d.contains("snap")),
            "diff names the span: {diffs:?}"
        );
    }

    #[test]
    fn replay_rejects_a_different_image() {
        let mut m = machine();
        let golden = record(&mut m, &meta(), &image(), &[(SNAP, 4)]).unwrap();
        let mut other = image();
        other[4] ^= 0xFF;
        let mut m2 = machine();
        let diffs = replay(&mut m2, &golden, &other, &[], &[(SNAP, 4)]);
        assert_eq!(diffs, ["image digest differs from golden"]);
    }

    #[test]
    fn parse_rejects_garbage() {
        assert!(Golden::parse("nope\n").is_err());
        assert!(Golden::parse("vioarm-golden-v1\nboard x\n").is_err());
        assert_eq!(parse_patch("0x20000000=0x41").unwrap(), (0x2000_0000, 0x41));
        assert!(parse_patch("0x20000000=0x141").is_err());
        assert!(parse_patch("nope").is_err());
        assert_eq!(
            parse_snap("0x20000000:0x1000").unwrap(),
            (0x2000_0000, 0x1000)
        );
        assert!(parse_snap("nope").is_err());
    }
}
