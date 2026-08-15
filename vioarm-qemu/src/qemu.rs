//! Running binaries under `qemu-system-arm` and parsing its per-instruction
//! CPU-state log.
//!
//! Invoked with `-accel tcg,one-insn-per-tb=on -d cpu,nochain`, QEMU prints one
//! state dump per executed instruction (the reset state first, then the state
//! before each subsequent instruction). Each dump is five lines: four register
//! lines (`R00=..R03`, `R04=..R07`, `R08=..R11`, `R12=..R15`) followed by an
//! `XPSR=...` line. Writing `-d cpu` output to a file (`-D`) is unbuffered, so
//! the harness can poll the log while QEMU spins on the test's landing loop
//! and stop the instant the target PC appears.

use crate::{CpuState, VfpState};
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

/// Candidate `qemu-system-arm` locations, in priority order.
fn qemu_candidates() -> Vec<PathBuf> {
    let mut out = Vec::new();
    if let Ok(p) = std::env::var("VIOARM_QEMU") {
        out.push(PathBuf::from(p));
    }
    if let Ok(paths) = std::env::var("PATH") {
        for dir in std::env::split_paths(&paths) {
            out.push(dir.join("qemu-system-arm.exe"));
            out.push(dir.join("qemu-system-arm"));
        }
    }
    out.push(PathBuf::from(r"C:\Program Files\qemu\qemu-system-arm.exe"));
    out
}

/// Locate a usable `qemu-system-arm`.
#[must_use]
pub fn find_qemu() -> Option<PathBuf> {
    qemu_candidates().into_iter().find(|p| p.is_file())
}

/// The QEMU invocation parameters for one conformance run. Defaults to an
/// ARMv7-M Cortex-M3 board (`mps2-an385`), the machine that matches
/// `Machine::armv7m` on the VioARM side. `fpu` adds the `fpu` log item so the
/// per-instruction dumps also carry the VFP register bank and FPSCR (only the
/// Cortex-M4 board has an FPU to report).
#[derive(Clone, Debug)]
pub struct QemuMachine {
    pub exe: PathBuf,
    pub machine: String,
    pub cpu: String,
    pub fpu: bool,
}

impl QemuMachine {
    #[must_use]
    pub fn detect() -> Option<Self> {
        Self::for_cpu("cortex-m3", "mps2-an385", false)
    }

    /// An ARMv7E-M reference (`Cortex-M4`), for diffing the DSP zone and the
    /// VFP zone that the M3 board faults on. The FPU is enabled in the program
    /// via `CPACR`, so the `-d fpu` dump reports the VFP state every step.
    #[must_use]
    pub fn detect_m4() -> Option<Self> {
        Self::for_cpu("cortex-m4", "mps2-an386", true)
    }

    #[must_use]
    fn for_cpu(cpu: &str, machine: &str, fpu: bool) -> Option<Self> {
        find_qemu().map(|exe| Self {
            exe,
            machine: machine.to_string(),
            cpu: cpu.to_string(),
            fpu,
        })
    }

    /// The `-d` log items for this machine: `cpu,nochain` plus `fpu` when the
    /// reference has an FPU.
    #[must_use]
    fn dump_flags(&self) -> String {
        if self.fpu {
            "cpu,fpu,nochain".to_string()
        } else {
            "cpu,nochain".to_string()
        }
    }
}

/// Parse the raw `-d cpu` log text into a sequence of CPU states. When the log
/// was produced with the `fpu` item, each state's VFP bank and FPSCR (the 16
/// `sXX=... dNN=...` lines and the `FPSCR:` line after `XPSR=`) are parsed
/// too; otherwise `vfp` is `None`.
#[must_use]
pub fn parse_log(text: &str) -> Vec<CpuState> {
    let lines: Vec<&str> = text.lines().collect();
    let mut states = Vec::new();
    let mut i = 0;
    while i < lines.len() {
        if !lines[i].starts_with("R00=") {
            i += 1;
            continue;
        }
        let mut good = i + 4 < lines.len();
        for (row, expect) in [(1, "R04="), (2, "R08="), (3, "R12=")] {
            if !lines[i + row].starts_with(expect) {
                good = false;
            }
        }
        if !good {
            i += 1;
            continue;
        }
        let mut r = [0u32; 16];
        let mut parse_ok = true;
        for row in 0..4 {
            for (token, idx) in lines[i + row].split_whitespace().zip(row * 4..row * 4 + 4) {
                match token.split_once('=') {
                    Some((_, hex)) => {
                        if let Ok(v) = u32::from_str_radix(hex, 16) {
                            r[idx] = v;
                        } else {
                            parse_ok = false;
                        }
                    }
                    None => parse_ok = false,
                }
            }
        }
        let xpsr = lines[i + 4]
            .strip_prefix("XPSR=")
            .and_then(|rest| rest.split_whitespace().next())
            .and_then(|hex| u32::from_str_radix(hex, 16).ok());
        match xpsr {
            Some(x) if parse_ok => {
                let vfp = parse_vfp_block(&lines, i + 5);
                let state = CpuState {
                    r,
                    xpsr: x,
                    vfp,
                };
                // QEMU dumps the reset state once per core (re)initialization,
                // which can repeat identical frames for a multi-phase board
                // reset. A real program can never produce two consecutive
                // identical states (the PC advances with every instruction), so
                // collapsing them is exact and keeps the trace aligned with the
                // per-instruction stream VioARM records.
                if states.last() != Some(&state) {
                    states.push(state);
                }
                // Advance past the FPU block (17 lines) when it was parsed.
                i += if vfp.is_some() { 22 } else { 5 };
            }
            _ => i += 1,
        }
    }
    states
}

/// Parse the FPU block that follows the `XPSR=` line: 16 lines of
/// `s{2n}=xxxxxxxx s{2n+1}=xxxxxxxx d{n}=xxxxxxxxxxxxxxxx` and a
/// `FPSCR: xxxxxxxx` line. Returns `None` when the block is absent.
fn parse_vfp_block(lines: &[&str], start: usize) -> Option<VfpState> {
    if start + 16 >= lines.len() || !lines[start].starts_with("s00=") {
        return None;
    }
    let mut s = [0u32; 32];
    for row in 0..16 {
        let line = lines[start + row];
        let mut ok = true;
        for (token, idx) in line.split_whitespace().zip([row * 2, row * 2 + 1]) {
            if let Some((_, hex)) = token.split_once('=') {
                if let Ok(v) = u32::from_str_radix(hex, 16) {
                    s[idx] = v;
                } else {
                    ok = false;
                }
            } else {
                ok = false;
            }
        }
        if !ok {
            return None;
        }
    }
    let fpscr = lines
        .get(start + 16)
        .and_then(|l| l.strip_prefix("FPSCR:"))
        .and_then(|rest| rest.split_whitespace().next())
        .and_then(|hex| u32::from_str_radix(hex, 16).ok())?;
    Some(VfpState { s, fpscr })
}

/// Run `bin` in QEMU, capturing the per-instruction state, and return the
/// trace up to and including the first state whose PC equals `stop_pc` (the
/// *state after* the final instruction — i.e. the address of the test's
/// landing loop, which QEMU then spins on forever).
///
/// Blocks until that state appears or `timeout` elapses; the QEMU process is
/// always terminated before returning.
#[must_use]
pub fn run_until_pc(mt: &QemuMachine, bin: &[u8], stop_pc: u32, timeout: Duration) -> Vec<CpuState> {
    let dir = std::env::temp_dir().join("vioarm-qemu");
    let _ = std::fs::create_dir_all(&dir);
    let nano = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock before epoch")
        .as_nanos();
    let bin_path = dir.join(format!("{}_{}.bin", std::process::id(), nano));
    let log_path = dir.join(format!("{}_{}.log", std::process::id(), nano));
    std::fs::write(&bin_path, bin).expect("write qemu input binary");
    let _ = std::fs::remove_file(&log_path);

    let device = format!("loader,file={},addr=0,force-raw=on", bin_path.to_string_lossy());
    let mut child = Command::new(&mt.exe)
        .arg("-machine")
        .arg(&mt.machine)
        .arg("-cpu")
        .arg(&mt.cpu)
        .arg("-nographic")
        .arg("-nodefaults")
        .arg("-accel")
        .arg("tcg,one-insn-per-tb=on")
        .arg("-d")
        .arg(mt.dump_flags())
        .arg("-D")
        .arg(&log_path)
        .arg("-device")
        .arg(&device)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn qemu-system-arm");

    let deadline = Instant::now() + timeout;
    let mut states = Vec::new();
    let mut stall = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(15));
        if let Ok(text) = std::fs::read_to_string(&log_path) {
            let parsed = parse_log(&text);
            if let Some(idx) = parsed.iter().position(|s| s.r[15] & !1 == stop_pc) {
                states = parsed[..=idx].to_vec();
                break;
            }
            if parsed.len() != states.len() {
                states = parsed;
                stall = Instant::now();
            }
        }
        // A locked-up core (double fault) stops emitting state dumps entirely;
        // once the trace goes quiet past the boot preamble, give up early
        // instead of holding the QEMU process until the deadline — under
        // parallel load a resident QEMU starves the other test spawns.
        if states.len() > 1 && Instant::now().duration_since(stall) > Duration::from_millis(300) {
            break;
        }
        if Instant::now() > deadline {
            break;
        }
    }
    let _ = child.kill();
    let _ = child.wait();
    if std::env::var_os("VIOARM_KEEP_LOGS").is_none() {
        let _ = std::fs::remove_file(&bin_path);
        let _ = std::fs::remove_file(&log_path);
    }
    states
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_state_dump() {
        let text = "R00=00000000 R01=00000000 R02=00000000 R03=00000000\n\
                    R04=00000000 R05=00000000 R06=00000000 R07=00000000\n\
                    R08=00000000 R09=00000000 R10=00000000 R11=00000000\n\
                    R12=00000000 R13=20001000 R14=ffffffff R15=00000080\n\
                    XPSR=41000000 -Z-- T priv-thread\n";
        let states = parse_log(text);
        assert_eq!(states.len(), 1);
        assert_eq!(states[0].r[15], 0x80);
        assert_eq!(states[0].xpsr, 0x4100_0000);
        assert!(states[0].vfp.is_none());
    }

    #[test]
    fn parses_the_fpu_block() {
        let mut text = String::from(
            "R00=00000000 R01=00000000 R02=00000000 R03=00000000\n\
             R04=00000000 R05=00000000 R06=00000000 R07=00000000\n\
             R08=00000000 R09=00000000 R10=00000000 R11=00000000\n\
             R12=00000000 R13=20002000 R14=ffffffff R15=00000100\n\
             XPSR=41000000 -Z-- T priv-thread\n",
        );
        for row in 0..16 {
            text.push_str(&format!(
                "s{:02}=00000000 s{:02}=00000000 d{:02}=0000000000000000\n",
                row * 2,
                row * 2 + 1,
                row
            ));
        }
        text.push_str("FPSCR: 00000000\n");
        let states = parse_log(&text);
        assert_eq!(states.len(), 1);
        let vfp = states[0].vfp.expect("fpu block parsed");
        assert_eq!(vfp.s, [0u32; 32]);
        assert_eq!(vfp.fpscr, 0);
    }
}