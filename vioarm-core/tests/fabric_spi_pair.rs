//! SPI controller/peripheral pair e2e over the fabric: real CMSIS firmware
//! on two boards, one master (`f103`) and one slave (`f107`), exchanging
//! bytes through `link_spi`.
//!
//! Choreography (deterministic lockstep, firmware-paced on `RXNE`): the
//! controller's first frame primes the pipe (MISO reads `0x00` — nothing
//! staged yet, as on silicon), then after a delay the second frame carries
//! the peripheral's reply. Timeouts snapshot `0xDEAD` instead of hanging.
//! Skipped gracefully without a toolchain.

use std::collections::HashMap;
use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::{AtomicU32, Ordering};
use vioarm_core::board::{self, BoardMemory};
use vioarm_core::fabric::Fabric;
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
    std::env::split_paths(&std::env::var_os("PATH")?).find_map(|dir| {
        let p = dir.join(name);
        p.is_file().then(|| p.to_str().unwrap_or("").to_owned())
    })
}

fn run(cmd: &str, args: &[&str]) -> Result<String, String> {
    let out = Command::new(cmd)
        .args(args)
        .output()
        .map_err(|e| format!("spawn {cmd}: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "{cmd} failed: {}",
            String::from_utf8_lossy(&out.stderr)
        ));
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
        eprintln!("skipping spi pair test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-spi-{}",
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

const CONTROLLER_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)

static unsigned xfer(unsigned out) {
    unsigned t = 400000;
    SPI1->DR = out;
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if (!--t) return 0xDEADu;
    }
    return SPI1->DR;
}

static void delay(unsigned n) {
    volatile unsigned i;
    for (i = 0; i < n; ++i) { }
}

void reset(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN;
    GPIOA->CRL = (GPIOA->CRL & ~(0xFFFu << 20)) | (0xB4Bu << 20);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
    SNAP0 = xfer(0xA0);   /* primes the pipe: MISO still 0x00 */
    delay(20000);         /* let the reply cross the wire */
    SNAP1 = xfer(0xA1);   /* expect 0xB0 */
    __asm__ volatile("bkpt #0");
}
"#;

const PERIPHERAL_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)

static unsigned wait_rx(void) {
    unsigned t = 400000;
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if (!--t) return 0xDEADu;
    }
    return SPI1->DR;
}

void reset(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN;
    GPIOA->CRL = (GPIOA->CRL & ~(0xFFFu << 20)) | (0xB4Bu << 20);
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE; /* slave */
    SNAP0 = wait_rx();    /* expect 0xA0 */
    SPI1->DR = 0xB0;      /* reply for the controller's next frame */
    __asm__ volatile("bkpt #0");
}
"#;

fn boot(board: BoardMemory, code: &[u8], entry: u32) -> Machine<BoardMemory> {
    let mut m = Machine::armv7m(board);
    m.load_image(&load_flat(CODE_BASE, code));
    {
        let mem = m.core.mem();
        mem.write_word(FLASH, INIT_SP).unwrap();
        mem.write_word(FLASH + 4, (CODE_BASE + entry) | 1).unwrap();
    }
    m.reset().expect("reset from vector table");
    m
}

fn scenario(
    ctrl: &[u8],
    ctrl_entry: u32,
    peri: &[u8],
    peri_entry: u32,
) -> ([u32; 2], [u32; 2], [u64; 2]) {
    let mut f: Fabric<BoardMemory> = Fabric::new();
    let a = f.add(boot(board::stm32f103(), ctrl, ctrl_entry));
    let b = f.add(boot(board::stm32f107(), peri, peri_entry));
    assert_eq!((a, b), (0, 1));
    f.link_spi(0, "spi1", 1, "spi1");
    for _ in 0..40 {
        f.run(5_000);
        let c0 = f.machines_mut()[0].core.mem().read_word(SNAP1).unwrap_or(0);
        let p0 = f.machines_mut()[1].core.mem().read_word(SNAP0).unwrap_or(0);
        if c0 != 0 && p0 != 0 {
            break;
        }
    }
    let snap = |f: &mut Fabric<BoardMemory>, i: usize| {
        let mem = f.machines_mut()[i].core.mem();
        [
            mem.read_word(SNAP0).unwrap_or(0xFFFF_FFFF),
            mem.read_word(SNAP1).unwrap_or(0xFFFF_FFFF),
        ]
    };
    let cyc = [f.machines()[0].cycles(), f.machines()[1].cycles()];
    (snap(&mut f, 0), snap(&mut f, 1), cyc)
}

#[test]
fn spi_pair_exchanges_both_directions() {
    let Some((ctrl, ctrl_sym)) = build_firmware(CONTROLLER_FW) else {
        return;
    };
    let Some((peri, peri_sym)) = build_firmware(PERIPHERAL_FW) else {
        return;
    };
    let ctrl_entry = *ctrl_sym.get("reset").expect("no reset symbol");
    let peri_entry = *peri_sym.get("reset").expect("no reset symbol");
    let (c, p, _) = scenario(&ctrl, ctrl_entry, &peri, peri_entry);
    assert_eq!(c[0], 0x00, "first frame MISO is the priming zero");
    assert_eq!(c[1], 0xB0, "second frame carries the reply");
    assert_eq!(p[0], 0xA0, "peripheral received the command");
    assert_eq!(p[1], 0x00, "peripheral never clocks again");
}

#[test]
fn spi_pair_rerun_is_deterministic() {
    let Some((ctrl, ctrl_sym)) = build_firmware(CONTROLLER_FW) else {
        return;
    };
    let Some((peri, peri_sym)) = build_firmware(PERIPHERAL_FW) else {
        return;
    };
    let ctrl_entry = *ctrl_sym.get("reset").expect("no reset symbol");
    let peri_entry = *peri_sym.get("reset").expect("no reset symbol");
    let first = scenario(&ctrl, ctrl_entry, &peri, peri_entry);
    let second = scenario(&ctrl, ctrl_entry, &peri, peri_entry);
    assert_eq!(first, second, "byte-identical rerun");
}
