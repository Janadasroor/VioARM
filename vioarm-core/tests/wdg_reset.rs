//! Phase-A watchdog proof (boards-roadmap): reset-observed reboots with
//! real C firmware on `stm32f103` (CMSIS packs).
//!
//! `wwdg_ewi_then_underflow_reboots`: window wide open, `EWI` armed — the
//! `WWDG` handler snapshots once and refreshes once, then lets the counter
//! underflow; after the reboot a boot counter in RAM proves the second
//! boot. `iwdg_expiry_reboots`: short `RLR`, no refresh — expiry reboots
//! into the second boot the same way.

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
const BOOTC: u32 = 0x2000_0008;

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
        eprintln!("skipping watchdog test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-wdg-{}",
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

const WWDG_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)
#define BOOTC (*(volatile unsigned *)0x20000008u)
#define DONE (*(volatile unsigned *)0x2000000Cu)

void WWDG_IRQHandler(void) {
    WWDG->SR = 0;
    if (!DONE) {
        SNAP0 = 0xE1u;
        WWDG->CR = 0xFFu;   /* refresh once, then let it underflow */
        DONE = 1;
    }
}

void reset(void) {
    BOOTC += 1;
    if (BOOTC == 2) {
        SNAP1 = 0xE2E2u;    /* second boot: the reset happened */
        __asm__ volatile("bkpt #0");
    }
    RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
    WWDG->CFR = 0x7Fu | 0x200u;   /* window wide open, EWI armed */
    WWDG->CR = 0xFFu;             /* WDGA | 0x7F */
    /* NVIC_EnableIRQ is an out-of-line call the .text extract cannot
       relocate: enable IRQ 0 with a direct ISER store instead. */
    NVIC->ISER[0] = 1u << (WWDG_IRQn & 0x1Fu);
    for (;;) { }
}
"#;

const IWDG_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP1 (*(volatile unsigned *)0x20000004u)
#define BOOTC (*(volatile unsigned *)0x20000008u)

void reset(void) {
    BOOTC += 1;
    if (BOOTC == 2) {
        SNAP1 = 0x1D06u;    /* second boot: the reset happened */
        __asm__ volatile("bkpt #0");
    }
    IWDG->KR = 0x5555u;
    IWDG->PR = 0;
    IWDG->RLR = 20;
    IWDG->KR = 0xAAAAu;
    IWDG->KR = 0xCCCCu;
    for (;;) { }
}
"#;

fn boot(code: &[u8], entry: u32, handler: Option<(u32, u32)>) -> Machine<board::BoardMemory> {
    let mut m = Machine::armv7m(board::stm32f103());
    m.load_image(&load_flat(CODE_BASE, code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, (CODE_BASE + entry) | 1).unwrap();
        if let Some((vector, addr)) = handler {
            mem.write_word(FLASH + vector * 4, (CODE_BASE + addr) | 1)
                .unwrap();
        }
    }
    m.reset().expect("reset from vector table");
    m
}

#[test]
fn wwdg_ewi_then_underflow_reboots() {
    let (code, syms) = match build_firmware(WWDG_FW) {
        Some(v) => v,
        None => return,
    };
    let entry = *syms.get("reset").expect("no reset symbol");
    let handler = *syms.get("WWDG_IRQHandler").expect("no handler symbol");
    let mut m = boot(&code, entry, Some((16, handler)));
    let end = m.run(1_000_000);
    assert!(end.is_err(), "stage 2 ends at bkpt, got {end:?}");
    let mem = m.core.mem();
    assert_eq!(mem.read_word(SNAP0).unwrap(), 0xE1, "EWI fired pre-reset");
    assert_eq!(
        mem.read_word(SNAP1).unwrap(),
        0xE2E2,
        "reboot after underflow"
    );
}

#[test]
fn iwdg_expiry_reboots() {
    let (code, syms) = match build_firmware(IWDG_FW) {
        Some(v) => v,
        None => return,
    };
    let entry = *syms.get("reset").expect("no reset symbol");
    let mut m = boot(&code, entry, None);
    let end = m.run(1_000_000);
    assert!(end.is_err(), "stage 2 ends at bkpt, got {end:?}");
    assert_eq!(
        m.core.mem().read_word(SNAP1).unwrap(),
        0x1D06,
        "reboot after IWDG expiry"
    );
}
