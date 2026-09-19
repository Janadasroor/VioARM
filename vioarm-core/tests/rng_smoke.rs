//! Phase-A RNG proof (boards-roadmap): statistical smoke with real C
//! firmware on `stm32h743` (CMSIS packs).
//!
//! Firmware enables the generator and snapshots 8 words; the host asserts
//! all are nonzero and pairwise distinct (the stream is deterministic, so
//! exact values would also match — distinctness is the statistical claim).

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::{AtomicU32, Ordering};
use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const FLASH: u32 = 0x0000_0000;
const CODE_BASE: u32 = 0x0000_0100;
const INIT_SP: u32 = 0x2000_4000;
const SNAP: u32 = 0x2000_0000;

static SEQ: AtomicU32 = AtomicU32::new(0);

fn which(name: &str) -> Option<String> {
    std::env::var_os("PATH").and_then(|p| {
        std::env::split_paths(&p)
            .map(|d| d.join(name))
            .find(|p| p.is_file())
            .map(|p| p.to_string_lossy().into_owned())
    })
}

fn run(cmd: &str, args: &[&str]) -> Result<String, String> {
    let out = Command::new(cmd)
        .args(args)
        .output()
        .map_err(|e| format!("spawn {cmd}: {e}"))?;
    if !out.status.success() {
        return Err(format!("{cmd} failed"));
    }
    String::from_utf8(out.stdout).map_err(|e| e.to_string())
}

fn build_firmware(source: &str) -> Option<(Vec<u8>, HashMap<String, u32>)> {
    let gcc = which("arm-none-eabi-gcc")?;
    let objcopy = which("arm-none-eabi-objcopy").or_else(|| which("llvm-objcopy"))?;
    let nm = which("arm-none-eabi-nm").or_else(|| which("nm"))?;
    let mf = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let h7 = mf.join("../packs/Keil.STM32H7xx_DFP/Drivers/CMSIS/Device/ST/STM32H7xx/Include");
    let core = mf.join("../packs/ARM.CMSIS/CMSIS/Core/Include");
    if !h7.is_dir() || !core.is_dir() {
        eprintln!("skipping rng smoke test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-rng-{}",
        SEQ.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).ok()?;
    let (src, obj, bin) = (dir.join("fw.c"), dir.join("fw.o"), dir.join("fw.bin"));
    std::fs::write(&src, source).ok()?;
    let (src, obj, bin) = (src.to_str()?, obj.to_str()?, bin.to_str()?);
    run(
        &gcc,
        &[
            "-mcpu=cortex-m7",
            "-mthumb",
            "-ffreestanding",
            "-nostdlib",
            "-O0",
            "-g0",
            "-c",
            src,
            "-o",
            obj,
            "-I",
            h7.to_str()?,
            "-I",
            core.to_str()?,
        ],
    )
    .ok()?;
    let listing = run(&nm, &["-n", obj]).ok()?;
    let mut symbols = HashMap::new();
    for line in listing.lines() {
        let mut it = line.split_whitespace();
        if let (Some(off), Some(_), Some(name)) = (it.next(), it.next(), it.next()) {
            if let Ok(off) = u32::from_str_radix(off, 16) {
                symbols.insert(name.to_string(), off);
            }
        }
    }
    run(
        &objcopy,
        &["--output-target=binary", "-j", ".text", obj, bin],
    )
    .ok()?;
    Some((std::fs::read(bin).ok()?, symbols))
}

const SMOKE_FW: &str = r#"
#include "stm32h743xx.h"

static unsigned wait_drdy(void) {
    unsigned t = 400000;
    while (!(RNG->SR & RNG_SR_DRDY)) {
        if (!--t) return 0xDEADu;
    }
    return RNG->DR;
}

void reset(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    RNG->CR |= RNG_CR_RNGEN;
    unsigned i;
    for (i = 0; i < 8; ++i) {
        ((volatile unsigned *)0x20000000u)[i] = wait_drdy();
    }
    __asm__ volatile("bkpt #0");
}
"#;

#[test]
fn rng_statistical_smoke_end_to_end() {
    let (code, syms) = match build_firmware(SMOKE_FW) {
        Some(v) => v,
        None => return,
    };
    let entry = *syms.get("reset").expect("no reset symbol");
    let mut m = Machine::armv7em(board::stm32h743());
    m.load_image(&load_flat(CODE_BASE, &code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, (CODE_BASE + entry) | 1).unwrap();
    }
    m.reset().expect("reset from vector table");
    let end = m.run(500_000);
    assert!(end.is_err(), "firmware ends at bkpt, got {end:?}");
    let mem = m.core.mem();
    let mut seen = HashSet::new();
    for i in 0..8 {
        let v = mem.read_word(SNAP + i * 4).unwrap();
        assert_ne!(v, 0, "sample {i} nonzero");
        assert_ne!(v, 0xDEAD, "sample {i} not a timeout");
        assert!(seen.insert(v), "sample {i} distinct");
    }
}
