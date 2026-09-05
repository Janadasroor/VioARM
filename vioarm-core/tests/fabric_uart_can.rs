//! Multi-machine fabric e2e: two-board UART ping-pong with real C firmware
//! plus CAN identifier arbitration, all in deterministic lockstep.
//!
//! The UART legs compile CMSIS firmware against the vendored F1 headers
//! (skipped gracefully without a toolchain); the CAN leg drives
//! driver-style register sequences (same level as the USB/ETH e2e files).
//! Every scenario runs twice for the byte-identical determinism proof.

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
const CODE_BASE: u32 = 0x0000_1000;
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

/// Compile CMSIS C firmware for v6-M (runs on M3 too) with the F1 headers.
/// Returns the raw `.text` plus symbols, or skips (`None`) without a
/// toolchain or the pack includes.
fn build_firmware(source: &str) -> Option<(Vec<u8>, HashMap<String, u32>)> {
    let gcc = which("arm-none-eabi-gcc")?;
    let objcopy = which("arm-none-eabi-objcopy").or_else(|| which("llvm-objcopy"))?;
    let nm = which("arm-none-eabi-nm").or_else(|| which("nm"))?;
    let mf = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let f1 = mf.join("../packs/Keil.STM32F1xx_DFP/Device/Include");
    // NOTE: the `STM32F4xx_CMSIS` path older tests reference no longer
    // exists in-tree; the ARM CMSIS pack carries the same `core_cm3.h`.
    let core = mf.join("../packs/ARM.CMSIS/CMSIS/Core/Include");
    if !f1.is_dir() || !core.is_dir() {
        eprintln!("skipping fabric uart test: pack includes not found");
        return None;
    }
    let dir = std::env::temp_dir().join(format!(
        "vioarm-fabric-{}",
        SEQ.fetch_add(1, Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).ok()?;
    let src = dir.join("firmware.c");
    let obj = dir.join("firmware.o");
    let bin = dir.join("firmware.bin");
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
    .map_err(|e| {
        eprintln!("skipping fabric uart test: {e}");
        e
    })
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
    let bytes = std::fs::read(bin).ok()?;
    Some((bytes, symbols))
}

const PINGER: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)

static void tx_byte(unsigned c) {
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = c;
}

static unsigned rx_byte(void) {
    unsigned t = 400000;
    while (!(USART1->SR & USART_SR_RXNE)) {
        if (!--t) return 0xDEADu;
    }
    return USART1->DR;
}

void reset(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 4)) | (0xBu << 4);
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 8)) | (0x4u << 8);
    USART1->BRR = 0x341;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    tx_byte('P');
    SNAP0 = rx_byte();
    tx_byte('Q');
    SNAP1 = rx_byte();
    __asm__ volatile("bkpt #0");
}
"#;

const PONGER: &str = r#"
#define STM32F10X_HD
#include "stm32f10x.h"

#define SNAP0 (*(volatile unsigned *)0x20000000u)
#define SNAP1 (*(volatile unsigned *)0x20000004u)

static void tx_byte(unsigned c) {
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = c;
}

static unsigned rx_byte(void) {
    unsigned t = 400000;
    while (!(USART1->SR & USART_SR_RXNE)) {
        if (!--t) return 0xDEADu;
    }
    return USART1->DR;
}

void reset(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 4)) | (0xBu << 4);
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 8)) | (0x4u << 8);
    USART1->BRR = 0x341;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    SNAP0 = rx_byte();
    tx_byte('R');
    SNAP1 = rx_byte();
    tx_byte('S');
    __asm__ volatile("bkpt #0");
}
"#;

/// Boot `board` with `code`/`entry` through the vector table.
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

/// Run the ping-pong scenario once; returns both snapshot pairs plus both
/// machines' cycle counts for the determinism comparison.
fn ping_pong_once(
    ping: &[u8],
    ping_entry: u32,
    pong: &[u8],
    pong_entry: u32,
) -> ([u32; 2], [u32; 2], [u64; 2]) {
    let mut f: Fabric<BoardMemory> = Fabric::new();
    let a = f.add(boot(board::stm32f103(), ping, ping_entry));
    let b = f.add(boot(board::stm32f107(), pong, pong_entry));
    assert_eq!((a, b), (0, 1));
    f.link_uart(0, "usart1", 1, "usart1");
    // Step until both snapshots land on both machines (bounded: timeouts
    // in firmware snapshot 0xDEAD instead of hanging).
    for _ in 0..40 {
        f.run(5_000);
        let done = [0, 1].iter().all(|&i| {
            let mem = f.machines_mut()[i].core.mem();
            mem.read_word(SNAP0).unwrap_or(0) != 0 && mem.read_word(SNAP1).unwrap_or(0) != 0
        });
        if done {
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
fn uart_ping_pong_two_boards() {
    let Some((ping, ping_sym)) = build_firmware(PINGER) else {
        return;
    };
    let Some((pong, pong_sym)) = build_firmware(PONGER) else {
        return;
    };
    let ping_entry = *ping_sym.get("reset").expect("no reset symbol");
    let pong_entry = *pong_sym.get("reset").expect("no reset symbol");
    let (a, b, _) = ping_pong_once(&ping, ping_entry, &pong, pong_entry);
    assert_eq!(
        a,
        [u32::from(b'R'), u32::from(b'S')],
        "pinger heard the pong"
    );
    assert_eq!(
        b,
        [u32::from(b'P'), u32::from(b'Q')],
        "ponger heard the ping"
    );
}

#[test]
fn fabric_run_is_deterministic() {
    let Some((ping, ping_sym)) = build_firmware(PINGER) else {
        return;
    };
    let Some((pong, pong_sym)) = build_firmware(PONGER) else {
        return;
    };
    let ping_entry = *ping_sym.get("reset").expect("no reset symbol");
    let pong_entry = *pong_sym.get("reset").expect("no reset symbol");
    let first = ping_pong_once(&ping, ping_entry, &pong, pong_entry);
    let second = ping_pong_once(&ping, ping_entry, &pong, pong_entry);
    assert_eq!(first, second, "byte-identical rerun");
}

// ---- CAN arbitration (driver-style register sequences) ----

const CAN1: u32 = 0x4000_6400;
const MCR: u32 = CAN1;
const MSR: u32 = CAN1 + 0x04;
const TSR: u32 = CAN1 + 0x08;
const RF0R: u32 = CAN1 + 0x0C;
const MB0: u32 = CAN1 + 0x180;
const FIFO0: u32 = CAN1 + 0x1B0;
const FMR: u32 = CAN1 + 0x200;
const FM1R: u32 = CAN1 + 0x204;
const FS1R: u32 = CAN1 + 0x20C;
const FA1R: u32 = CAN1 + 0x21C;
const FR1: u32 = CAN1 + 0x240;
const FR2: u32 = CAN1 + 0x244;

fn wr(mem: &mut BoardMemory, addr: u32, val: u32) {
    for (i, b) in val.to_le_bytes().iter().enumerate() {
        mem.write(addr + i as u32, *b).unwrap();
    }
}

fn rd(mem: &mut BoardMemory, addr: u32) -> u32 {
    let mut bytes = [0u8; 4];
    for (i, b) in bytes.iter_mut().enumerate() {
        *b = mem.read(addr + i as u32).unwrap();
    }
    u32::from_le_bytes(bytes)
}

/// Bring CAN1 up with bank 0 as a 32-bit accept-all mask filter.
fn can_up(mem: &mut BoardMemory) {
    wr(mem, MCR, 1); // INRQ
    assert_ne!(rd(mem, MSR) & 1, 0, "INAK");
    wr(mem, FMR, 1); // FINIT
    wr(mem, FS1R, 1); // bank 0: 32-bit scale
    wr(mem, FM1R, 0); // mask mode
    wr(mem, FR1, 0);
    wr(mem, FR2, 0); // mask 0: accept everything
    wr(mem, FA1R, 1); // bank 0 active
    wr(mem, FMR, 0);
    wr(mem, MCR, 0);
    assert_eq!(rd(mem, MSR) & 1, 0, "left init mode");
}

/// Queue one standard-ID frame in mailbox 0 (silicon `TIR` layout).
fn can_tx(mem: &mut BoardMemory, std_id: u32, data: &[u8]) {
    assert!(std_id < 0x800);
    let mut dl = 0u32;
    let mut dh = 0u32;
    for (i, byte) in data.iter().enumerate() {
        let w = if i < 4 { &mut dl } else { &mut dh };
        *w |= u32::from(*byte) << (8 * (i % 4));
    }
    wr(mem, MB0 + 0x04, data.len() as u32); // TDTR DLC
    wr(mem, MB0 + 0x08, dl);
    wr(mem, MB0 + 0x0C, dh);
    wr(mem, MB0, (std_id << 21) | 1); // TIR + TXRQ
}

/// Read one FIFO0 frame: `(TI, DLC, bytes)`, then release the slot.
fn can_rx(mem: &mut BoardMemory) -> (u32, u32, Vec<u8>) {
    assert_ne!(rd(mem, RF0R) & 0b11, 0, "FMP0");
    let ti = rd(mem, FIFO0);
    let dt = rd(mem, FIFO0 + 0x04);
    let dl = rd(mem, FIFO0 + 0x08);
    let dh = rd(mem, FIFO0 + 0x0C);
    wr(mem, RF0R, 1 << 5); // RFOM0 release
    let dlc = dt & 0xF;
    let mut bytes = Vec::new();
    for w in [dl, dh] {
        bytes.extend(w.to_le_bytes());
    }
    bytes.truncate(dlc as usize);
    (ti, dlc, bytes)
}

/// Transmit `id0` from machine 0 and `id1` from machine 1 in the same
/// rounds while machine 2 only receives. Returns each machine's received
/// TI words in arrival order. (CAN has no self-delivery, so the senders
/// get one frame each; the pure receiver observes the arbitration order.)
fn arbitrate(id0: u32, data0: &[u8], id1: u32, data1: &[u8]) -> (Vec<u32>, Vec<u32>, Vec<u32>) {
    let mut f: Fabric<BoardMemory> = Fabric::new();
    let a = f.add(Machine::armv7m(board::stm32f105()));
    let b = f.add(Machine::armv7m(board::stm32f107()));
    let c = f.add(Machine::armv7m(board::stm32f107()));
    assert_eq!((a, b, c), (0, 1, 2));
    f.link_can(&[(0, "can1"), (1, "can1"), (2, "can1")]);
    for i in [0, 1, 2] {
        can_up(f.machines_mut()[i].core.mem());
    }
    {
        let mem = f.machines_mut()[0].core.mem();
        can_tx(mem, id0, data0);
    }
    {
        let mem = f.machines_mut()[1].core.mem();
        can_tx(mem, id1, data1);
    }
    f.run(2_000);
    // Both mailboxes completed on the wire.
    for i in [0, 1] {
        let mem = f.machines_mut()[i].core.mem();
        assert_ne!(rd(mem, TSR) & 1, 0, "RQCP0");
    }
    let mut got = [Vec::new(), Vec::new(), Vec::new()];
    for i in [0, 1, 2] {
        let mem = f.machines_mut()[i].core.mem();
        while rd(mem, RF0R) & 0b11 != 0 {
            let (ti, _, _) = can_rx(mem);
            got[i].push(ti);
        }
    }
    let [g0, g1, g2] = got;
    (g0, g1, g2)
}

#[test]
fn can_arbitration_delivers_lowest_id_first() {
    // Machine 0 sends 0x100, machine 1 sends 0x080: nobody receives its
    // own frame, and the pure receiver gets the higher-priority (lower-ID)
    // frame first.
    let (m0, m1, m2) = arbitrate(0x100, &[0xAA], 0x080, &[0xBB]);
    assert_eq!([m0.len(), m1.len()], [1, 1], "no self-delivery");
    assert_eq!(m0[0] >> 21, 0x080, "machine 0 heard 0x080");
    assert_eq!(m1[0] >> 21, 0x100, "machine 1 heard 0x100");
    assert_eq!(
        m2.iter().map(|ti| ti >> 21).collect::<Vec<_>>(),
        [0x080, 0x100],
        "arbitration order on the receiver"
    );
}

#[test]
fn can_arbitration_is_deterministic() {
    let first = arbitrate(0x100, &[0xAA], 0x080, &[0xBB]);
    let second = arbitrate(0x100, &[0xAA], 0x080, &[0xBB]);
    assert_eq!(first, second, "byte-identical rerun");
}
