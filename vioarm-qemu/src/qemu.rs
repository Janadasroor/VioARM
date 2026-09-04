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
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

/// Where QEMU input binaries and `-d cpu` logs live. Prefers the disk-backed
/// Cargo `target` directory over tmpfs: an orphaned QEMU writing here consumes
/// disk, never the RAM a desktop session needs.
#[must_use]
fn log_base() -> PathBuf {
    if let Some(dir) = std::env::var_os("VIOARM_LOG_DIR") {
        return PathBuf::from(dir);
    }
    if let Ok(exe) = std::env::current_exe() {
        let target = exe
            .ancestors()
            .find(|p| p.file_name().is_some_and(|n| n == "target"));
        if let Some(target) = target {
            return target.join("vioarm-qemu-logs");
        }
    }
    std::env::temp_dir()
}

/// Remove run artifacts left behind by dead runs (an interrupted `cargo test`
/// orphans its QEMU children, whose `-d cpu` logs grow by gigabytes).
fn reap_stale_runs(dir: &Path) {
    const STALE: Duration = Duration::from_secs(3600);
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    let procfs = Path::new("/proc").is_dir();
    let stale = |entry: &std::fs::DirEntry| {
        entry
            .metadata()
            .ok()
            .and_then(|m| m.modified().ok())
            .and_then(|t| t.elapsed().ok())
            .is_some_and(|age| age > STALE)
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
                continue;
            };
            let Ok(pid) = name.parse::<u32>() else {
                continue;
            };
            let alive = procfs && Path::new("/proc").join(pid.to_string()).exists();
            if !alive || stale(&entry) {
                // The owning test process is gone, but its QEMU children may
                // still be running (a SIGKILLed cargo leaves them no chance to
                // clean up). Kill every recorded child whose /proc cmdline is
                // still a qemu-system binary — the cmdline check guards
                // against killing an unrelated process that recycled the pid.
                for record in std::fs::read_dir(&path).into_iter().flatten() {
                    let record = match record {
                        Ok(r) => r,
                        Err(_) => continue,
                    };
                    if record.path().extension().is_none_or(|e| e != "qemu") {
                        continue;
                    }
                    let Ok(child) = std::fs::read_to_string(record.path()) else {
                        continue;
                    };
                    let Ok(child_pid) = child.trim().parse::<u32>() else {
                        continue;
                    };
                    if procfs && is_qemu_process(child_pid) {
                        let _ = std::process::Command::new("/bin/kill")
                            .arg("-9")
                            .arg(child_pid.to_string())
                            .status();
                    }
                    let _ = std::fs::remove_file(record.path());
                }
                let _ = std::fs::remove_dir_all(&path);
            }
        } else if stale(&entry) {
            let _ = std::fs::remove_file(&path);
        }
    }
}

/// Whether `/proc/<pid>` exists and its executable is a `qemu-system-*`.
fn is_qemu_process(pid: u32) -> bool {
    use std::io::Read;
    let mut cmdline = String::new();
    if std::fs::File::open(format!("/proc/{pid}/cmdline"))
        .and_then(|mut f| f.read_to_string(&mut cmdline))
        .is_err()
    {
        return false;
    }
    cmdline.contains("qemu-system")
}

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

/// QEMU oracle version pinned in CI (`ci.yml`).
pub const ORACLE_VERSION: &str = "qemu-system-arm 10.2.1";

/// Every QEMU version the conformance traces are proven against: the
/// canonical pin above (local dev) plus CI realities (Ubuntu apt ships
/// 8.2.2, and the suite is green against it). Anything else (e.g. a brew
/// build with divergent `BKPT` handling) must skip the oracle tests.
pub const ORACLE_VERSIONS: &[&str] = &["10.2.1", "8.2.2"];

/// Locate a usable `qemu-system-arm`.
#[must_use]
pub fn find_qemu() -> Option<PathBuf> {
    qemu_candidates().into_iter().find(|p| p.is_file())
}

/// The `--version` first line of the QEMU at `exe`, if it runs.
#[must_use]
pub fn qemu_version(exe: &std::path::Path) -> Option<String> {
    let out = std::process::Command::new(exe)
        .arg("--version")
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&out.stdout);
    text.lines().next().map(str::to_string)
}

/// Whether the QEMU at `exe` is a proven oracle. Traces are only
/// comparable against [`ORACLE_VERSIONS`]: anything else (a distro or brew
/// build at another version) must skip the oracle tests, not fail them.
#[must_use]
pub fn qemu_is_oracle(exe: &std::path::Path) -> bool {
    qemu_version(exe).is_some_and(|v| ORACLE_VERSIONS.iter().any(|w| v.contains(w)))
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
    /// Where the loader places the raw image. Boards that boot from address 0
    /// (micro:bit, mps2-an385/386) use 0; the ARMv8-M reference boards
    /// (mps2-an505 & co.) boot from flash at `0x1000_0000`, which the reset
    /// vector field in the image already points back into.
    pub load_addr: u32,
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

    /// An ARMv6-M reference (`Cortex-M0` on the BBC micro:bit's nRF51), for
    /// diffing the ARMv6-M instruction subset and its exception model. The M0
    /// has no FPU, so the dump carries no VFP block. The board's SRAM is only
    /// 16 KiB (nRF51), well below the harness's `0x2000_2000` stack top.
    #[must_use]
    pub fn detect_m0() -> Option<Self> {
        Self::for_cpu("cortex-m0", "microbit", false)
    }

    /// An ARMv8-M mainline reference (`Cortex-M33` on the mps2-an505), the
    /// conformance oracle for the ARMv8-M baseline subset: every instruction
    /// the baseline (Cortex-M23) implements behaves identically on the M33.
    /// The board boots from flash at `0x1000_0000`, so the loader goes there
    /// too. `cortex-m33` carries an FPU, so the per-instruction dump always
    /// includes the VFP block; the baseline programs never touch it, leaving
    /// both sides' (zeroed) VFP state in agreement.
    #[must_use]
    pub fn detect_m33() -> Option<Self> {
        Self::for_cpu_at("cortex-m33", "mps2-an505", false, 0x1000_0000)
    }

    /// An ARMv7E-M reference (`Cortex-M7` with FPv5-D16), for diffing double-
    /// precision (`FPv5-D16`) instruction behavior that the M4 board faults on.
    /// Uses `mps2-an500` (Cortex-M7) with `fpu` log so the VFP block is dumped.
    #[must_use]
    pub fn detect_m7() -> Option<Self> {
        Self::for_cpu("cortex-m7", "mps2-an500", true)
    }

    #[must_use]
    fn for_cpu(cpu: &str, machine: &str, fpu: bool) -> Option<Self> {
        Self::for_cpu_at(cpu, machine, fpu, 0)
    }

    #[must_use]
    fn for_cpu_at(cpu: &str, machine: &str, fpu: bool, load_addr: u32) -> Option<Self> {
        find_qemu().map(|exe| Self {
            exe,
            machine: machine.to_string(),
            cpu: cpu.to_string(),
            fpu,
            load_addr,
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
    let mut feeder = LogFeeder::default();
    for line in text.lines() {
        feeder.feed(line);
    }
    // A log truncated right after its `XPSR=` row still yields that state
    // (minus VFP); anything shorter is discarded.
    if !feeder.pending.is_empty() {
        feeder.finish_block();
    }
    feeder.states
}

/// Incremental `-d cpu` state-block parser. [`LogFeeder::feed`] consumes one
/// complete line at a time and appends finished [`CpuState`]s to
/// [`LogFeeder::states`], so a poller can follow a growing log by feeding only
/// the newly appended lines instead of re-parsing the whole file every tick.
///
/// A block is a `R00=` header, its `R04=`/`R08=`/`R12=` rows and an `XPSR=`
/// line; when the log was produced with the `fpu` item the XPSR line is
/// followed by 16 `sXX=` rows and an `FPSCR:` line, all consumed as part of
/// the same block. Lines that do not fit a block under construction resync at
/// the next `R00=` (live QEMU logs are well-formed, so this only matters for
/// hand-written tests, which use [`parse_log`]).
#[derive(Default)]
pub struct LogFeeder {
    /// States parsed so far, consecutive-identical runs collapsed exactly like
    /// the whole-text parser.
    pub states: Vec<CpuState>,
    pending: Vec<String>,
}

impl LogFeeder {
    /// Feed one complete line (no trailing newline).
    pub fn feed(&mut self, line: &str) {
        if self.pending.is_empty() {
            if !line.starts_with("R00=") {
                return;
            }
        } else if self.pending.len() == 5 {
            // After a valid `XPSR=` row: an `s00=` row opens the FPU block.
            if !line.starts_with("s00=") {
                self.finish_block();
            }
        } else if self.pending.len() == 22 {
            self.finish_block();
        }
        self.pending.push(line.to_owned());
        if self.pending.len() == 22 {
            self.finish_block();
        }
    }

    /// Validate and emit the accumulated block, or drop it on mismatch.
    fn finish_block(&mut self) {
        let p = &self.pending;
        let ok = p.len() >= 5 && {
            let mut good = p[0].starts_with("R00=");
            for (row, expect) in [(1, "R04="), (2, "R08="), (3, "R12=")] {
                good &= p[row].starts_with(expect);
            }
            good && p[4].starts_with("XPSR=")
        };
        if !ok {
            // Resync: retry from the second line of the rejected block, which
            // may itself start a new one.
            if p.len() > 1 {
                let rest: Vec<String> = p[1..].to_vec();
                self.pending.clear();
                for line in rest {
                    self.feed(&line);
                }
            } else {
                self.pending.clear();
            }
            return;
        }
        let mut r = [0u32; 16];
        let mut parse_ok = true;
        for (row, line) in p.iter().enumerate().take(4) {
            for (token, idx) in line.split_whitespace().zip(row * 4..row * 4 + 4) {
                match token.split_once('=') {
                    Some((_, hex)) => match u32::from_str_radix(hex, 16) {
                        Ok(v) => r[idx] = v,
                        Err(_) => parse_ok = false,
                    },
                    None => parse_ok = false,
                }
            }
        }
        let xpsr = p[4]
            .strip_prefix("XPSR=")
            .and_then(|rest| rest.split_whitespace().next())
            .and_then(|hex| u32::from_str_radix(hex, 16).ok());
        if let (Some(x), true) = (xpsr, parse_ok) {
            let vfp = if p.len() >= 22 {
                parse_vfp_lines(&p[5..21], &p[21])
            } else {
                None
            };
            let state = CpuState { r, xpsr: x, vfp };
            // Consecutive identical frames collapse exactly like the
            // whole-text parser keeps traces aligned with QEMU's own dumps.
            if self.states.last() != Some(&state) {
                self.states.push(state);
            }
        }
        self.pending.clear();
    }
}

/// Parse the FPU block that follows the `XPSR=` line: the 16
/// `s{2n}=xxxxxxxx s{2n+1}=xxxxxxxx d{n}=xxxxxxxxxxxxxxxx` rows and the
/// `FPSCR: xxxxxxxx` line. Returns `None` when the block is absent or
/// malformed.
fn parse_vfp_lines(rows: &[String], fpscr_line: &str) -> Option<VfpState> {
    if rows.len() != 16 || !rows[0].starts_with("s00=") {
        return None;
    }
    let mut s = [0u32; 32];
    for (row, line) in rows.iter().enumerate() {
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
    let fpscr = fpscr_line
        .strip_prefix("FPSCR:")
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
pub fn run_until_pc(
    mt: &QemuMachine,
    bin: &[u8],
    stop_pc: u32,
    timeout: Duration,
) -> Vec<CpuState> {
    // Prefer the disk-backed Cargo target directory over tmpfs: an orphaned
    // QEMU writing here can consume disk, never the RAM the session lives in.
    let dir = log_base().join("vioarm-qemu");
    reap_stale_runs(&dir);
    let run_dir = dir.join(std::process::id().to_string());
    let _ = std::fs::create_dir_all(&run_dir);
    let nano = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock before epoch")
        .as_nanos();
    let bin_path = run_dir.join(format!("{}.bin", nano));
    let log_path = run_dir.join(format!("{}.log", nano));
    std::fs::write(&bin_path, bin).expect("write qemu input binary");
    let _ = std::fs::remove_file(&log_path);

    let device = format!(
        "loader,file={},addr={:#x},force-raw=on",
        bin_path.to_string_lossy(),
        mt.load_addr
    );
    let mut command = Command::new(&mt.exe);
    command
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
        .stderr(Stdio::null());
    let mut child = command.spawn().expect("spawn qemu-system-arm");
    // Record the child so a later reaper can kill it if this test process
    // dies without running cleanup (SIGKILL/OOM leaves no chance to run code).
    let _ = std::fs::write(
        run_dir.join(format!("{}.qemu", nano)),
        child.id().to_string(),
    );

    let deadline = Instant::now() + timeout;
    let spawned = Instant::now();
    let mut feeder = LogFeeder::default();
    let mut consumed: u64 = 0;
    let mut carry = String::new();
    let max_log_bytes: u64 = std::env::var("VIOARM_MAX_LOG_MB")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1024)
        * 1024
        * 1024;
    let mut stall = Instant::now();
    let mut seen_states = 0usize;
    'poll: loop {
        std::thread::sleep(Duration::from_millis(15));
        // Runaway-trace guard: a diverged core that never reaches the stop
        // would otherwise write `-d cpu` at disk speed until the deadline.
        // One capped trace beats a filled disk; the caller sees a short trace
        // and a visible assertion failure.
        if std::fs::metadata(&log_path).is_ok_and(|m| m.len() > max_log_bytes) {
            break;
        }
        if let Ok(mut f) = std::fs::File::open(&log_path) {
            use std::io::{Read, Seek, SeekFrom};
            if f.seek(SeekFrom::Start(consumed)).is_ok() {
                let mut chunk = String::new();
                if f.read_to_string(&mut chunk).is_ok() {
                    consumed += chunk.len() as u64;
                    carry.push_str(&chunk);
                    // Only complete lines feed the parser; the trailing
                    // partial line waits for the next poll.
                    if let Some((complete, _)) = carry.rsplit_once('\n') {
                        for line in complete.split('\n') {
                            feeder.feed(line);
                        }
                        carry.clear();
                        if feeder.states.len() > seen_states {
                            // Trace progressed; quiet-detection restarts.
                            seen_states = feeder.states.len();
                            stall = Instant::now();
                            if feeder
                                .states
                                .last()
                                .is_some_and(|s| s.r[15] & !1 == stop_pc)
                            {
                                break 'poll;
                            }
                        }
                    }
                }
            }
        }
        // A locked-up core (double fault) stops emitting state dumps entirely;
        // once the trace goes quiet past the boot preamble, give up early
        // instead of holding the QEMU process until the deadline — under
        // parallel load a resident QEMU starves the other test spawns. A core
        // that stalls inside the boot preamble gets the same treatment after
        // a grace window so a wedged board cannot hold the full timeout.
        if seen_states > 1 && Instant::now().duration_since(stall) > Duration::from_millis(300) {
            break;
        }
        if seen_states <= 1 && spawned.elapsed() > Duration::from_secs(5) {
            break;
        }
        if Instant::now() > deadline {
            break;
        }
    }
    let _ = child.kill();
    let _ = child.wait();
    // Normal completion: the recorded pid is stale the moment we reaped it.
    let _ = std::fs::remove_file(run_dir.join(format!("{}.qemu", nano)));
    let states = std::mem::take(&mut feeder.states);
    if std::env::var_os("VIOARM_KEEP_LOGS").is_none() {
        let _ = std::fs::remove_file(&bin_path);
        let _ = std::fs::remove_file(&log_path);
        let _ = std::fs::remove_dir(&run_dir);
    }
    states
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn oracle_gate_passes_the_pinned_qemu_and_rejects_garbage() {
        // A missing binary has no version and is never the oracle.
        let missing = std::path::Path::new("/nonexistent/qemu-system-arm");
        assert_eq!(qemu_version(missing), None);
        assert!(!qemu_is_oracle(missing));
        // Any installed QEMU agrees with the gate exactly when its version
        // line names a proven oracle.
        if let Some(exe) = find_qemu() {
            let v = qemu_version(&exe).expect("installed qemu reports a version");
            assert_eq!(
                qemu_is_oracle(&exe),
                ORACLE_VERSIONS.iter().any(|w| v.contains(w))
            );
        }
    }

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
