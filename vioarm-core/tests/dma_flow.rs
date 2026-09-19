//! Phase-A DMA proof (boards-roadmap): memory-to-memory plus circular
//! reload with real C firmware on `stm32f103` (`DMA1` Ch1, CMSIS packs).
//!
//! Firmware stages 8 words in RAM, runs a `MEM2MEM` copy (`MINC`/`PINC`,
//! 32-bit), polls `TCIF1`, and snapshots the destination ends plus `ISR`;
//! then it re-arms circular, completes a second period, and snapshots the
//! reloaded `CNDTR` (proving `CIRC` reload, not just completion).

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
        eprintln!("skipping dma flow test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-dma-{}",
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

const FLOW_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)
#define SNAP2 (*(volatile unsigned *)0x20000008u)
#define SNAP3 (*(volatile unsigned *)0x2000000Cu)
#define SNAP4 (*(volatile unsigned *)0x20000010u)
#define SRC ((volatile unsigned *)0x20000100u)
#define DST ((volatile unsigned *)0x20000200u)

static unsigned wait_tcif(void) {
    unsigned t = 400000;
    while (!(DMA1->ISR & DMA_ISR_TCIF1)) {
        if (!--t) return 0xDEADu;
    }
    return 0;
}

void reset(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    unsigned i;
    for (i = 0; i < 8; ++i) SRC[i] = 0xA0u + i;
    /* MEM2MEM copy, 32-bit, both sides increment. */
    DMA1_Channel1->CPAR = (unsigned)SRC;
    DMA1_Channel1->CMAR = (unsigned)DST;
    DMA1_Channel1->CNDTR = 8;
    DMA1_Channel1->CCR = DMA_CCR1_MEM2MEM | DMA_CCR1_MINC | DMA_CCR1_PINC
        | DMA_CCR1_MSIZE_32BIT | DMA_CCR1_PSIZE_32BIT | DMA_CCR1_EN;
    wait_tcif();
    SNAP0 = DST[0];
    SNAP1 = DST[7];
    SNAP2 = DMA1->ISR;
    /* Circular re-arm: a second period reloads CNDTR instead of stopping. */
    DMA1_Channel1->CCR = 0;
    DMA1_Channel1->CNDTR = 8;
    DMA1_Channel1->CCR = DMA_CCR1_MEM2MEM | DMA_CCR1_MINC | DMA_CCR1_PINC
        | DMA_CCR1_MSIZE_32BIT | DMA_CCR1_PSIZE_32BIT | DMA_CCR1_CIRC
        | DMA_CCR1_EN;
    wait_tcif();
    SNAP3 = DMA1_Channel1->CNDTR;
    SNAP4 = DST[0];
    __asm__ volatile("bkpt #0");
}
"#;

#[test]
fn dma_mem2mem_and_circular_end_to_end() {
    let (code, syms) = match build_firmware(FLOW_FW) {
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
    assert_eq!(mem.read_word(SNAP0).unwrap(), 0xA0, "first word copied");
    assert_eq!(mem.read_word(SNAP1).unwrap(), 0xA7, "last word copied");
    assert_eq!(mem.read_word(SNAP2).unwrap() & 0x2, 0x2, "TCIF1 latched");
    assert_eq!(mem.read_word(SNAP3).unwrap(), 8, "CIRC reloaded CNDTR");
    assert_eq!(mem.read_word(SNAP4).unwrap(), 0xA0, "second period intact");
}
