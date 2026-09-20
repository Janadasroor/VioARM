//! Phase-C `stm32f102` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `GPIOD` answers `CRL` + `BSRR`-driven `ODR` (F1 layout).
//! * `DMA2` copies memory-to-memory with `TCIF1`.
//! * `RTC`/`BKP` answer (backup register round-trips the domain).
//! * `PWR`/`CRC` answer their windows (`CR` latch, computed `DR`).
//!
//! Register offsets come from the vendored `STM32F102` table
//! (`device::svd::stm32f102`); bit positions mirror the driver sources.

use vioarm_core::board;
use vioarm_core::machine::Machine;
use vioarm_core::mem::Bus;

fn ww(mem: &mut board::BoardMemory, addr: u32, val: u32) {
    mem.write_word(addr, val).unwrap();
}

fn rw(mem: &mut board::BoardMemory, addr: u32) -> u32 {
    mem.read_word(addr).unwrap()
}

fn pump(mem: &mut board::BoardMemory, n: u64) {
    for _ in 0..n {
        mem.tick_devices(1);
    }
}

// --- GPIO (F1 layout) ---

const GPIOD: u32 = 0x4001_1400;
const CRL: u32 = 0x00;
const ODR: u32 = 0x0C;
const BSRR: u32 = 0x10;

#[test]
fn gpiod_answers() {
    let mut m = Machine::armv7m(board::stm32f102());
    let mem = m.core.mem();
    ww(mem, GPIOD + CRL, 0x3333_3333);
    assert_eq!(rw(mem, GPIOD + CRL), 0x3333_3333, "CRL latches");
    ww(mem, GPIOD + BSRR, 0x0000_0005);
    assert_eq!(rw(mem, GPIOD + ODR), 0x05, "BSRR sets ODR");
    ww(mem, GPIOD + BSRR, 0x0005_0000);
    assert_eq!(rw(mem, GPIOD + ODR), 0x00, "BSRR resets ODR");
}

// --- DMA2 ---

const DMA2: u32 = 0x4002_0400;
const ISR_D: u32 = 0x00;
const CCR1: u32 = 0x08;
const CNDTR1: u32 = 0x0C;
const CPAR1: u32 = 0x10;
const CMAR1: u32 = 0x14;
const EN: u32 = 1 << 0;
const MINC: u32 = 1 << 7;
const PINC: u32 = 1 << 6;
const MEM2MEM: u32 = 1 << 14;
const TCIF1: u32 = 1 << 1;
const SRC: u32 = 0x2000_0100;
const DST: u32 = 0x2000_0200;

#[test]
fn dma2_mem2mem_copy() {
    let mut m = Machine::armv7m(board::stm32f102());
    let mem = m.core.mem();
    for (i, w) in [0x1111_1111u32, 0x2222_2222].iter().enumerate() {
        ww(mem, SRC + (i as u32) * 4, *w);
    }
    ww(mem, DMA2 + CPAR1, SRC);
    ww(mem, DMA2 + CMAR1, DST);
    ww(mem, DMA2 + CNDTR1, 2);
    ww(
        mem,
        DMA2 + CCR1,
        EN | MEM2MEM | MINC | PINC | (2 << 8) | (2 << 10),
    );
    pump(mem, 10);
    assert_eq!(rw(mem, DST), 0x1111_1111, "word 0 lands");
    assert_eq!(rw(mem, DST + 4), 0x2222_2222, "word 1 lands");
    assert_ne!(rw(mem, DMA2 + ISR_D) & TCIF1, 0, "TCIF1 latches");
}

// --- RTC/BKP ---

const BKP: u32 = 0x4000_6C04;
const BKP_DR1: u32 = 0x00;
const RTC: u32 = 0x4000_2800;
const RTC_CRL: u32 = 0x04;
const CNF: u32 = 1 << 4;

#[test]
fn bkp_domain_roundtrips() {
    let mut m = Machine::armv7m(board::stm32f102());
    let mem = m.core.mem();
    ww(mem, BKP + BKP_DR1, 0x1234);
    assert_eq!(rw(mem, BKP + BKP_DR1), 0x1234, "backup register holds");
    // F1 RTC configuration mode latches through CRL.
    ww(mem, RTC + RTC_CRL, CNF);
    assert_ne!(rw(mem, RTC + RTC_CRL) & CNF, 0, "CNF latches");
    ww(mem, RTC + RTC_CRL, 0);
    assert_eq!(rw(mem, RTC + RTC_CRL) & CNF, 0, "CNF exits");
}

// --- PWR + CRC windows ---

const PWR: u32 = 0x4000_7000;
const CR_P: u32 = 0x00;
const DBP: u32 = 1 << 8;
const CRC: u32 = 0x4002_3000;
const DR_C: u32 = 0x00;

#[test]
fn pwr_crc_answer() {
    let mut m = Machine::armv7m(board::stm32f102());
    let mem = m.core.mem();
    ww(mem, PWR + CR_P, DBP);
    assert_eq!(rw(mem, PWR + CR_P) & DBP, DBP, "DBP latches");
    let before = rw(mem, CRC + DR_C);
    ww(mem, CRC + DR_C, 0x01);
    let after = rw(mem, CRC + DR_C);
    assert_ne!(before, after, "CRC engine runs");
}
