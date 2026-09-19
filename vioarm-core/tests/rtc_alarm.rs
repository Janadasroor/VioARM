//! Phase-A RTC/BKP proof (boards-roadmap): a real alarm interrupt plus
//! backup-register persistence across reset, with C firmware on `stm32f103`.
//!
//! Stage 1: `PRL = 0`, `ALR = 200`, `CRH.ALRIE` + `NVIC_EnableIRQ` — the
//! `RTCAlarm` handler snapshots the counter and halts. Stage 2 (after a
//! host `reset()`): firmware finds the magic `BKP.DR1` still intact and
//! snapshots it — backup-domain persistence end to end.

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
    let f1 = mf.join("../packs/Keil.STM32F1xx_DFP/Device/Include");
    let core = mf.join("../packs/ARM.CMSIS/CMSIS/Core/Include");
    if !f1.is_dir() || !core.is_dir() {
        eprintln!("skipping rtc alarm test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-rtc-{}",
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

const ALARM_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)

void RTCAlarm_IRQHandler(void) {
    RTC->CRL &= (unsigned)~RTC_CRL_ALRF;
    SNAP0 = RTC->CNTL;
    __asm__ volatile("bkpt #0");
}

void reset(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    if (BKP->DR1 == 0xBEEFu) {
        SNAP1 = BKP->DR1;      /* stage 2: magic survived reset */
        __asm__ volatile("bkpt #0");
    }
    /* Stage 1: alarm 200 counts out, PRL = 0 (every tick). */
    RTC->CRL |= RTC_CRL_CNF;
    RTC->PRLL = 0;
    RTC->CNTH = 0;
    RTC->CNTL = 0;
    RTC->ALRH = 0;
    RTC->ALRL = 200;
    RTC->CRL &= (unsigned)~RTC_CRL_CNF;
    RTC->CRH |= RTC_CRH_ALRIE;
    NVIC_EnableIRQ(RTCAlarm_IRQn);
    BKP->DR1 = 0xBEEFu;
    for (;;) { }
}
"#;

#[test]
fn rtc_alarm_irq_and_bkp_persistence_end_to_end() {
    let (code, syms) = match build_firmware(ALARM_FW) {
        Some(v) => v,
        None => return,
    };
    let entry = *syms.get("reset").expect("no reset symbol");
    let handler = *syms.get("RTCAlarm_IRQHandler").expect("no handler symbol");
    let mut m = Machine::armv7m(board::stm32f103());
    m.load_image(&load_flat(CODE_BASE, &code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, (CODE_BASE + entry) | 1).unwrap();
        mem.write_word(FLASH + 57 * 4, (CODE_BASE + handler) | 1)
            .unwrap();
    }
    m.reset().expect("reset from vector table");
    // Stage 1: the alarm interrupt fires and snapshots the counter.
    let end = m.run(500_000);
    assert!(end.is_err(), "handler ends at bkpt, got {end:?}");
    let fired = m.core.mem().read_word(SNAP0).unwrap();
    assert!(
        (200..=220).contains(&fired),
        "alarm fired at counter ~200, got {fired}"
    );
    // Stage 2: across a system reset the backup magic persists.
    m.reset().expect("reset keeps backup domain");
    let end = m.run(500_000);
    assert!(end.is_err(), "stage 2 ends at bkpt, got {end:?}");
    assert_eq!(
        m.core.mem().read_word(SNAP1).unwrap(),
        0xBEEF,
        "BKP.DR1 survived reset"
    );
}
