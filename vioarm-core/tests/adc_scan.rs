//! Phase-A ADC proof (boards-roadmap): multi-channel scan with real C
//! firmware on `stm32f103` (`ADC1`, CMSIS packs).
//!
//! The host drives distinct levels per channel (ch0=1000, ch1=2000,
//! ch2=3000); firmware scans `SQR{0,1,2}` and snapshots the three results,
//! then runs an analog-watchdog phase (`AWDSGL` ch1, window 500–1500:
//! ch1=2000 trips `AWD`) and an injected conversion (`JSWSTART` → `JDR1`).

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
const SNAP3: u32 = 0x2000_000C;
const SNAP4: u32 = 0x2000_0010;

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
        eprintln!("skipping adc scan test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-adc-{}",
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

const SCAN_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)
#define SNAP2 (*(volatile unsigned *)0x20000008u)
#define SNAP3 (*(volatile unsigned *)0x2000000Cu)
#define SNAP4 (*(volatile unsigned *)0x20000010u)

static unsigned wait_eoc(void) {
    unsigned t = 400000;
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if (!--t) return 0xDEADu;
    }
    return ADC1->DR;
}

void reset(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->SQR1 = (2u << 20);              /* L = 2: three conversions */
    ADC1->SQR3 = (0u << 0) | (1u << 5) | (2u << 10);
    ADC1->CR2 = ADC_CR2_ADON;
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_SWSTART;
    /* Inline polls (no call overhead): the scan chains every 14 cycles,
       so the read must chase EOC tightly or a later slot overwrites DR. */
    while (!(ADC1->SR & ADC_SR_EOC)) { }
    SNAP0 = ADC1->DR;
    while (!(ADC1->SR & ADC_SR_EOC)) { }
    SNAP1 = ADC1->DR;
    while (!(ADC1->SR & ADC_SR_EOC)) { }
    SNAP2 = ADC1->DR;
    /* Watchdog phase: ch1 watched, window 500-1500, ch1 = 2000 trips. */
    ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_AWDEN | ADC_CR1_AWDSGL | 1u;
    ADC1->HTR = 1500;
    ADC1->LTR = 500;
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_SWSTART;
    wait_eoc();
    wait_eoc();
    SNAP3 = ADC1->SR;
    /* Injected phase: JSWSTART converts JSQR ch0 into JDR1. */
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_JSWSTART;
    while (!(ADC1->SR & ADC_SR_JEOC)) { }
    SNAP4 = ADC1->JDR1;
    __asm__ volatile("bkpt #0");
}
"#;

#[test]
fn adc_scan_watchdog_injected_end_to_end() {
    let (code, syms) = match build_firmware(SCAN_FW) {
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
    // Distinct levels per channel through the stimulus protocol
    // (`level | (ch + 1) << 16`; direct calls carry the full word, since
    // the byte-wise `drive_device` path only fits single bytes).
    for d in m.core.mem().devices_mut() {
        if d.name() == "adc1" {
            for (ch, level) in [(0u32, 1000u32), (1, 2000), (2, 3000)] {
                d.drive_inputs(level | ((ch + 1) << 16));
            }
        }
    }
    let end = m.run(500_000);
    assert!(end.is_err(), "firmware ends at bkpt, got {end:?}");
    let mem = m.core.mem();
    assert_eq!(mem.read_word(SNAP0).unwrap(), 1000, "scan slot ch0");
    assert_eq!(mem.read_word(SNAP1).unwrap(), 2000, "scan slot ch1");
    assert_eq!(mem.read_word(SNAP2).unwrap(), 3000, "scan slot ch2");
    assert_eq!(mem.read_word(SNAP3).unwrap() & 1, 1, "AWD tripped on ch1");
    assert_eq!(mem.read_word(SNAP4).unwrap(), 1000, "injected JDR1 ch0");
}
