//! Phase-A CRC proof (boards-roadmap): standard check values with real C
//! firmware on `stm32g474xx` (programmable `CRC`, CMSIS packs).
//!
//! CRC-32/ISO-HDLC over "123456789" (byte reversal in, word reversal out,
//! firmware xors `0xFFFFFFFF`) must read `0xCBF43926`; CRC-16/CCITT-FALSE
//! (poly `0x1021`, 16-bit, init `0xFFFF`) over the same message must read
//! `0x29B1`.

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
const INIT_SP: u32 = 0x2000_4000;
const SNAP0: u32 = 0x2000_0000;
const SNAP1: u32 = 0x2000_0004;

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
    let g4 = mf.join("../packs/Keil.STM32G4xx_DFP/Drivers/CMSIS/Device/ST/STM32G4xx/Include");
    let core = mf.join("../packs/ARM.CMSIS/CMSIS/Core/Include");
    if !g4.is_dir() || !core.is_dir() {
        eprintln!("skipping crc vectors test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-crc-{}",
        SEQ.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).ok()?;
    let (src, obj, bin) = (dir.join("fw.c"), dir.join("fw.o"), dir.join("fw.bin"));
    std::fs::write(&src, source).ok()?;
    let (src, obj, bin) = (src.to_str()?, obj.to_str()?, bin.to_str()?);
    run(
        &gcc,
        &[
            "-mcpu=cortex-m4",
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
            g4.to_str()?,
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

const VECTORS_FW: &str = r#"
#include "stm32g474xx.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)

/* NOTE: no const arrays — objcopy extracts .text only, so rodata would
   read back zeros. Message bytes go out as immediates. */
#define FEED9() do { \
    *(volatile unsigned char *)&CRC->DR = 0x31u; \
    *(volatile unsigned char *)&CRC->DR = 0x32u; \
    *(volatile unsigned char *)&CRC->DR = 0x33u; \
    *(volatile unsigned char *)&CRC->DR = 0x34u; \
    *(volatile unsigned char *)&CRC->DR = 0x35u; \
    *(volatile unsigned char *)&CRC->DR = 0x36u; \
    *(volatile unsigned char *)&CRC->DR = 0x37u; \
    *(volatile unsigned char *)&CRC->DR = 0x38u; \
    *(volatile unsigned char *)&CRC->DR = 0x39u; \
} while (0)

void reset(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    /* CRC-32/ISO-HDLC: default poly, byte reflection in, word out. */
    CRC->CR = (1u << 5) | (1u << 7);
    CRC->CR |= 0x1u;   /* RESET */
    FEED9();
    SNAP0 = CRC->DR ^ 0xFFFFFFFFu;
    /* CRC-16/CCITT-FALSE: poly 0x1021, 16-bit, init FFFF. */
    CRC->POL = 0x1021u;
    CRC->CR = (1u << 3);
    CRC->CR |= 0x1u;   /* RESET */
    FEED9();
    SNAP1 = CRC->DR;
    __asm__ volatile("bkpt #0");
}
"#;

#[test]
fn crc_standard_vectors_end_to_end() {
    let (code, syms) = match build_firmware(VECTORS_FW) {
        Some(v) => v,
        None => return,
    };
    let entry = *syms.get("reset").expect("no reset symbol");
    let mut m = Machine::armv7em(board::stm32g474xx());
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
    assert_eq!(mem.read_word(SNAP0).unwrap(), 0xCBF4_3926, "ISO-HDLC");
    assert_eq!(mem.read_word(SNAP1).unwrap(), 0x29B1, "CCITT-FALSE");
}
