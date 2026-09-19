//! Phase-A SPI proof (boards-roadmap): full-duplex controller/peripheral
//! exchange with real C firmware over `Fabric::link_spi` (`f103` + `f107`,
//! CMSIS packs, same builder pattern as `fabric_spi_pair`).
//!
//! Controller: `xfer(0xA0)` → `SNAP0` (expect `0x00`, MISO idle),
//! `xfer(0xA1)` → `SNAP1`, `xfer(0xA2)`/`xfer(0xA3)` → `SNAP2`/`SNAP3`
//! (expect `0xB0`, the staged reply retained).
//! Peripheral: frame 1 → `SNAP0` (`0xA0`), `SR` → `SNAP3` (expect
//! `RXNE|UDR`: first frame unstaged); stage `0xB0`; frame 2 → `SNAP1`
//! (`0xA1`), `SR` → `SNAP2` (clean, `TXE` only); frames 3–4 unread → final
//! `SR` must read `RXNE|OVR|TXE` (overrun latched, first bytes kept).

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
const SNAP2: u32 = 0x2000_0008;
const SNAP3: u32 = 0x2000_000C;
const SPI1_SR: u32 = 0x4001_3008;

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
        eprintln!("skipping spi error test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-spierr-{}",
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

const CONTROLLER_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)
#define SNAP2 (*(volatile unsigned *)0x20000008u)
#define SNAP3 (*(volatile unsigned *)0x2000000Cu)

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
    SNAP0 = xfer(0xA0);
    delay(2000);
    SNAP1 = xfer(0xA1);
    delay(2000);
    SNAP2 = xfer(0xA2);
    delay(2000);
    SNAP3 = xfer(0xA3);
    __asm__ volatile("bkpt #0");
}
"#;

const PERIPHERAL_FW: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)
#define SNAP2 (*(volatile unsigned *)0x20000008u)
#define SNAP3 (*(volatile unsigned *)0x2000000Cu)

static unsigned wait_rx(void) {
    unsigned t = 400000;
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if (!--t) return 0xDEADu;
    }
    return 0;
}

void reset(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN;
    GPIOA->CRL = (GPIOA->CRL & ~(0xFFFu << 20)) | (0xB4Bu << 20);
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE; /* slave */
    wait_rx();                /* frame 1 (unstaged: UDR) */
    SNAP3 = SPI1->SR;         /* expect RXNE|UDR */
    SNAP0 = SPI1->DR;         /* expect 0xA0 (clears RXNE+UDR path) */
    SPI1->DR = 0xB0;          /* stage reply */
    wait_rx();                /* frame 2 (staged: clean) */
    SNAP1 = SPI1->DR;         /* expect 0xA1 */
    SNAP2 = SPI1->SR;         /* expect TXE only */
    __asm__ volatile("bkpt #0"); /* frames 3-4 arrive unread: OVR */
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

#[test]
fn spi_pair_errors_end_to_end() {
    let (ctrl, ctrl_syms) = match build_firmware(CONTROLLER_FW) {
        Some(v) => v,
        None => return,
    };
    let (peri, peri_syms) = match build_firmware(PERIPHERAL_FW) {
        Some(v) => v,
        None => return,
    };
    let ctrl_entry = *ctrl_syms.get("reset").expect("no reset symbol");
    let peri_entry = *peri_syms.get("reset").expect("no reset symbol");
    let mut f: Fabric<BoardMemory> = Fabric::new();
    let (a, b) = (
        f.add(boot(board::stm32f103(), &ctrl, ctrl_entry)),
        f.add(boot(board::stm32f107(), &peri, peri_entry)),
    );
    assert_eq!((a, b), (0, 1));
    f.link_spi(0, "spi1", 1, "spi1");
    for _ in 0..60 {
        f.run(5_000);
        let c3 = f.machines_mut()[0].core.mem().read_word(SNAP3).unwrap_or(0);
        let p1 = f.machines_mut()[1].core.mem().read_word(SNAP1).unwrap_or(0);
        if c3 != 0 && p1 != 0 {
            break;
        }
    }
    let snap = |f: &mut Fabric<BoardMemory>, i: usize, addr: u32| {
        f.machines_mut()[i]
            .core
            .mem()
            .read_word(addr)
            .unwrap_or(0xFFFF_FFFF)
    };
    // Controller: idle MISO, then the retained staged reply.
    assert_eq!(snap(&mut f, 0, SNAP0), 0x00, "frame 1 MISO idle");
    assert_eq!(snap(&mut f, 0, SNAP1), 0xB0, "frame 2 MISO reply");
    assert_eq!(snap(&mut f, 0, SNAP2), 0xB0, "frame 3 MISO retained");
    assert_eq!(snap(&mut f, 0, SNAP3), 0xB0, "frame 4 MISO retained");
    // Peripheral: both frames received; first unstaged (UDR), second clean.
    assert_eq!(snap(&mut f, 1, SNAP0), 0xA0, "frame 1 MOSI");
    assert_eq!(snap(&mut f, 1, SNAP1), 0xA1, "frame 2 MOSI");
    let sr1 = snap(&mut f, 1, SNAP3);
    assert_eq!(sr1 & 0x01, 0x01, "frame 1 RXNE was set");
    assert_eq!(sr1 & 0x08, 0x08, "frame 1 UDR (unstaged)");
    assert_eq!(sr1 & 0x40, 0x00, "frame 1 no OVR");
    let sr2 = snap(&mut f, 1, SNAP2);
    assert_eq!(sr2 & 0x01, 0x00, "frame 2 RXNE drained");
    assert_eq!(sr2 & 0x08, 0x00, "frame 2 no UDR (staged)");
    assert_eq!(sr2 & 0x40, 0x00, "frame 2 no OVR");
    // Frames 3-4 arrived unread: overrun latched, first bytes kept.
    let sr_end = f.machines_mut()[1]
        .core
        .mem()
        .read_word(SPI1_SR)
        .unwrap_or(0);
    assert_eq!(sr_end & 0x01, 0x01, "unread frame RXNE");
    assert_eq!(sr_end & 0x40, 0x40, "unread frames OVR");
}
