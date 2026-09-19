//! Phase-A COMP proof (boards-roadmap): threshold crossings with
//! hysteresis on `stm32f042`, driven through the shared analog bus.
//!
//! Firmware only enables `COMP1` and spins; the host drives the bus
//! (`Machine::analog_drive_mv`) and reads `CSR.COMP1OUT`: low → 0, high →
//! 1, mid-band → holds 1 (hysteresis), low again → 0.

use std::collections::HashMap;
use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::{AtomicU32, Ordering};
use vioarm_core::board;
use vioarm_core::loader::load_flat;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

const FLASH: u32 = 0x0000_0000;
const CODE_BASE: u32 = 0x0000_0100;
const INIT_SP: u32 = 0x2000_1000;
const COMP_CSR: u32 = 0x4001_001C;
const COMP1OUT: u32 = 1 << 14;

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
    let f0 = mf.join("../packs/Keil.STM32F0xx_DFP/Device/Include");
    let core = mf.join("../packs/ARM.CMSIS/CMSIS/Core/Include");
    if !f0.is_dir() || !core.is_dir() {
        eprintln!("skipping comp hysteresis test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-comp-{}",
        SEQ.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).ok()?;
    let (src, obj, bin) = (dir.join("fw.c"), dir.join("fw.o"), dir.join("fw.bin"));
    std::fs::write(&src, source).ok()?;
    let (src, obj, bin) = (src.to_str()?, obj.to_str()?, bin.to_str()?);
    run(
        &gcc,
        &[
            "-mcpu=cortex-m0",
            "-mthumb",
            "-ffreestanding",
            "-nostdlib",
            "-O0",
            "-g0",
            "-DSTM32F042x6",
            "-c",
            src,
            "-o",
            obj,
            "-I",
            f0.to_str()?,
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

const COMP_FW: &str = r#"
#include "stm32f0xx.h"

void reset(void) {
    COMP->CSR |= COMP_CSR_COMP1EN;
    for (;;) { }
}
"#;

#[test]
fn comp_threshold_hysteresis_end_to_end() {
    let (code, syms) = match build_firmware(COMP_FW) {
        Some(v) => v,
        None => return,
    };
    let entry = *syms.get("reset").expect("no reset symbol");
    let mut m = Machine::new(board::stm32f042());
    m.load_image(&load_flat(CODE_BASE, &code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, (CODE_BASE + entry) | 1).unwrap();
    }
    m.reset().expect("reset from vector table");
    let out = |m: &mut Machine<board::BoardMemory>| {
        let _ = m.run(1_000);
        m.core.mem().read_word(COMP_CSR).unwrap() & COMP1OUT
    };
    m.analog_drive_mv(1000);
    assert_eq!(out(&mut m), 0, "below band: OUT low");
    m.analog_drive_mv(2500);
    assert_ne!(out(&mut m), 0, "above band: OUT high");
    m.analog_drive_mv(1650);
    assert_ne!(out(&mut m), 0, "mid band holds high (hysteresis)");
    m.analog_drive_mv(1000);
    assert_eq!(out(&mut m), 0, "below band: OUT low again");
}
