//! Phase-A DAC proof (boards-roadmap): triangle waveform plus a DAC→ADC
//! chain with real C firmware on `stm32f103` (`DAC` + `ADC1`, CMSIS packs).
//!
//! Triangle: `EN1|TEN1`, software trigger, `WAVE` = triangle, `MAMP` = 7
//! (mask `0xFF`); ten triggers → `DOR1 == 10`, 255 total → `255`, one
//! more → `254` (ramp turns). Chain: `DHR12R1 = 2048` latches straight
//! through and publishes to the analog bus; `ADC1` samples it back
//! (≈2048, within one LSB of the round trip).

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
        eprintln!("skipping dac wave test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-dac-{}",
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

const WAVE_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)
#define SNAP2 (*(volatile unsigned *)0x20000008u)
#define SNAP3 (*(volatile unsigned *)0x2000000Cu)

static unsigned wait_eoc(void) {
    unsigned t = 400000;
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if (!--t) return 0xDEADu;
    }
    return ADC1->DR;
}

void reset(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    /* Triangle: EN1 | TEN1 | WAVE = 2 | MAMP = 7, base 0. */
    DAC->CR = 0x1u | 0x4u | (2u << 6) | (7u << 8);
    DAC->DHR12R1 = 0;
    unsigned i;
    for (i = 0; i < 10; ++i) DAC->SWTRIGR = 1;
    SNAP0 = DAC->DOR1;
    for (; i < 255; ++i) DAC->SWTRIGR = 1;
    SNAP1 = DAC->DOR1;
    DAC->SWTRIGR = 1;
    SNAP2 = DAC->DOR1;
    /* Chain: DAC ch1 straight through, ADC1 samples the bus. */
    DAC->CR = 0x1u;
    DAC->DHR12R1 = 2048;
    ADC1->CR2 = ADC_CR2_ADON;
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_SWSTART;
    SNAP3 = wait_eoc();
    __asm__ volatile("bkpt #0");
}
"#;

#[test]
fn dac_triangle_and_chain_end_to_end() {
    let (code, syms) = match build_firmware(WAVE_FW) {
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
    let end = m.run(500_000);
    assert!(end.is_err(), "firmware ends at bkpt, got {end:?}");
    let mem = m.core.mem();
    assert_eq!(mem.read_word(SNAP0).unwrap(), 10, "ramp climbs");
    assert_eq!(mem.read_word(SNAP1).unwrap(), 255, "ramp peaks at mask");
    assert_eq!(mem.read_word(SNAP2).unwrap(), 254, "ramp turns down");
    let chain = mem.read_word(SNAP3).unwrap();
    assert!(
        (2046..=2048).contains(&chain),
        "DAC->ADC round trip within 1 LSB, got {chain}"
    );
}
