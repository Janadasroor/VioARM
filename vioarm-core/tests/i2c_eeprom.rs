//! Phase-A I2C proof (boards-roadmap): EEPROM random-read with real C
//! firmware on `stm32f103` (`I2C1`, CMSIS packs).
//!
//! Phase 1 (7-bit): `START`, `0xA0`, word `0x10`, repeated `START`,
//! `0xA1`, two reads (`ACK` then `NACK`+`STOP`) — the host EEPROM answers
//! `0x5A`. Phase 2 (10-bit): header `0xF0` (no `ADDR` yet), second byte
//! `0x55` (`ADDR` now), `STOP`. The host asserts the drained wire bytes
//! (`0xA0 0x10 0xA1 0xF0 0x55`), the read data, and the `BTF`/`ADDR`
//! snapshots.

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
const SNAP2: u32 = 0x2000_0008;
const SNAP4: u32 = 0x2000_0010;
const SNAP5: u32 = 0x2000_0014;

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
    let f1 = mf.join("../packs/Keil.STM32F1xx_DFP/Device/Include");
    let core = mf.join("../packs/ARM.CMSIS/CMSIS/Core/Include");
    if !f1.is_dir() || !core.is_dir() {
        eprintln!("skipping i2c eeprom test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-i2c-{}",
        SEQ.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).ok()?;
    let (src, obj, bin) = (dir.join("fw.c"), dir.join("fw.o"), dir.join("fw.bin"));
    std::fs::write(&src, source).ok()?;
    let (src, obj, bin) = (src.to_str()?, obj.to_str()?, bin.to_str()?);
    run(
        &gcc,
        &[
            "-mcpu=cortex-m3",
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
            f1.to_str()?,
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

const EEPROM_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)
#define SNAP2 (*(volatile unsigned *)0x20000008u)
#define SNAP4 (*(volatile unsigned *)0x20000010u)
#define SNAP5 (*(volatile unsigned *)0x20000014u)

static unsigned wait_flag(unsigned mask) {
    unsigned t = 400000;
    while (!(I2C1->SR1 & mask)) {
        if (!--t) return 0xDEADu;
    }
    return 0;
}

static void delay(unsigned n) {
    volatile unsigned i;
    for (i = 0; i < n; ++i) { }
}

void reset(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    GPIOB->CRL = (GPIOB->CRL & ~(0xFFu << 24)) | (0xFFu << 24);
    I2C1->CR1 |= I2C_CR1_PE;
    /* Phase 1: EEPROM random read. */
    I2C1->CR1 |= I2C_CR1_START;
    wait_flag(I2C_SR1_SB);
    I2C1->DR = 0xA0;
    wait_flag(I2C_SR1_ADDR);
    (void)I2C1->SR2;
    I2C1->DR = 0x10;
    wait_flag(I2C_SR1_BTF);
    SNAP2 = I2C1->SR1;
    I2C1->CR1 |= I2C_CR1_START;   /* repeated start */
    wait_flag(I2C_SR1_SB);
    I2C1->DR = 0xA1;
    wait_flag(I2C_SR1_ADDR);
    (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_ACK;
    wait_flag(I2C_SR1_RXNE);
    SNAP0 = I2C1->DR;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    wait_flag(I2C_SR1_RXNE);
    SNAP1 = I2C1->DR;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (I2C1->SR2 & I2C_SR2_BUSY) { }
    /* Phase 2: 10-bit header needs its second byte. */
    I2C1->CR1 |= I2C_CR1_START;
    wait_flag(I2C_SR1_SB);
    I2C1->DR = 0xF0;
    delay(100);
    SNAP4 = I2C1->SR1;            /* ADDR must NOT be set yet */
    I2C1->DR = 0x55;
    wait_flag(I2C_SR1_ADDR);
    SNAP5 = I2C1->SR1;            /* ADDR now */
    I2C1->CR1 |= I2C_CR1_STOP;
    while (I2C1->SR2 & I2C_SR2_BUSY) { }
    __asm__ volatile("bkpt #0");
}
"#;

#[test]
fn i2c_eeprom_random_read_end_to_end() {
    let (code, syms) = match build_firmware(EEPROM_FW) {
        Some(v) => v,
        None => return,
    };
    let entry = *syms.get("reset").expect("no reset symbol");
    let mut m = Machine::armv7m(board::stm32f103());
    m.load_image(&load_flat(CODE_BASE, &code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, (CODE_BASE + entry) | 1).unwrap();
    }
    m.reset().expect("reset from vector table");
    // The EEPROM answers every read with 0x5A.
    m.drive_device("i2c", &[0x5A]);
    let end = m.run(500_000);
    assert!(end.is_err(), "firmware ends at bkpt, got {end:?}");
    let mem = m.core.mem();
    assert_eq!(mem.read_word(SNAP0).unwrap(), 0x5A, "read byte 0");
    assert_eq!(mem.read_word(SNAP1).unwrap(), 0x5A, "read byte 1");
    let sr_tx = mem.read_word(SNAP2).unwrap();
    assert_eq!(sr_tx & (1 << 7), 1 << 7, "TXE after data byte");
    assert_eq!(sr_tx & (1 << 2), 1 << 2, "BTF after data byte");
    let sr_hdr = mem.read_word(SNAP4).unwrap();
    assert_eq!(sr_hdr & (1 << 1), 0, "no ADDR after header alone");
    let sr_full = mem.read_word(SNAP5).unwrap();
    assert_eq!(sr_full & (1 << 1), 1 << 1, "ADDR after second byte");
    // The wire saw both address phases, both directions, both headers.
    assert_eq!(m.drain_device("i2c"), [0xA0, 0x10, 0xA1, 0xF0, 0x55]);
}
