//! Phase-C `stm32f407` close-out proof (boards-roadmap): every peripheral
//! the audit newly promotes off the latch list is exercised through
//! driver-style register pokes on the real board (no toolchain).
//!
//! * `GPIOF`–`GPIOK` answer `MODER` + `BSRR`-driven `ODR`.
//! * `UART7`/`UART8` move bytes CPU-side (F1 `SR`/`DR`).
//! * `TIM3`/`8`–`14` count to overflow.
//! * `RNG` latches fresh data with `DRDY`.
//! * `DCMI` captures staged bytes with `FRAMERI`.
//! * `FSMC` gates its NOR banks.
//! * `LTDC` reloads layers with `RRIF`.
//! * `HASH` digests `abc` to the SHA-1 vector.
//! * `SPI4`–`6` + `I2S2ext`/`3ext` shift (peerless `0x00`, staged byte).
//! * `SAI1` fills its TX FIFO (`FLVL`, overrun, flush).
//! * `CRYP` encrypts the FIPS-197 B block (AES-128-ECB).
//!
//! Register offsets come from the vendored `STM32F407` table
//! (`device::svd::stm32f407`); bit positions mirror the driver sources.

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

// --- GPIO ---

const GPIOF: u32 = 0x4002_1400;
const GPIOG: u32 = 0x4002_1800;
const GPIOH: u32 = 0x4002_1C00;
const GPIOI: u32 = 0x4002_2000;
const GPIOJ: u32 = 0x4002_2400;
const GPIOK: u32 = 0x4002_2800;
const MODER: u32 = 0x00;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;

#[test]
fn gpio_f_thru_k_answer() {
    let mut m = Machine::armv7m(board::stm32f407());
    let mem = m.core.mem();
    for base in [GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ, GPIOK] {
        ww(mem, base + MODER, 0x5555_5555);
        assert_eq!(rw(mem, base + MODER), 0x5555_5555, "MODER latches");
        ww(mem, base + BSRR, 0x0000_0005);
        assert_eq!(rw(mem, base + ODR), 0x05, "BSRR sets ODR");
        ww(mem, base + BSRR, 0x0005_0000);
        assert_eq!(rw(mem, base + ODR), 0x00, "BSRR resets ODR");
    }
}

// --- UART7/8 (F1 SR/DR) ---

const UART7: u32 = 0x4000_7800;
const UART8: u32 = 0x4000_7C00;
const SR: u32 = 0x00;
const DR: u32 = 0x04;
const CR1: u32 = 0x0C;
const UE: u32 = 1 << 13;
const TE: u32 = 1 << 3;
const RE: u32 = 1 << 2;
const RXNE: u32 = 1 << 5;

fn uart_loopback(m: &mut Machine<board::BoardMemory>, base: u32, name: &str) {
    {
        let mem = m.core.mem();
        ww(mem, base + CR1, UE | TE | RE);
        ww(mem, base + DR, u32::from(b'A'));
        pump(mem, 10_000);
    }
    assert_eq!(m.drain_device(name), b"A", "TX drains");
    m.drive_device(name, b"Q");
    let (sr, byte) = {
        let mem = m.core.mem();
        pump(mem, 10_000);
        let mut sr = 0;
        for _ in 0..100 {
            sr = rw(mem, base + SR);
            if sr & RXNE != 0 {
                break;
            }
            pump(mem, 100);
        }
        (sr, rw(mem, base + DR) & 0xFF)
    };
    assert_ne!(sr & RXNE, 0, "RXNE latches");
    assert_eq!(byte, u32::from(b'Q'), "RX byte");
}

#[test]
fn uart7_8_cpu_loopback() {
    let mut m = Machine::armv7m(board::stm32f407());
    uart_loopback(&mut m, UART7, "uart7");
    uart_loopback(&mut m, UART8, "uart8");
}

// --- Timers ---

const TIM3: u32 = 0x4000_0400;
const TIM8: u32 = 0x4001_0400;
const TIM9: u32 = 0x4001_4000;
const TIM10: u32 = 0x4001_4400;
const TIM11: u32 = 0x4001_4800;
const TIM12: u32 = 0x4000_1800;
const TIM13: u32 = 0x4000_1C00;
const TIM14: u32 = 0x4000_2000;
const CR1_T: u32 = 0x00;
const SR_T: u32 = 0x10;
const ARR: u32 = 0x2C;
const CEN: u32 = 1 << 0;
const UIF: u32 = 1 << 0;

#[test]
fn timers_count() {
    let mut m = Machine::armv7m(board::stm32f407());
    let mem = m.core.mem();
    for base in [TIM3, TIM8, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14] {
        ww(mem, base + ARR, 10);
        ww(mem, base + CR1_T, CEN);
    }
    pump(mem, 100_000);
    for base in [TIM3, TIM8, TIM9, TIM10, TIM11, TIM12, TIM13, TIM14] {
        assert_ne!(rw(mem, base + SR_T) & UIF, 0, "TIM@{base:#X} overflows");
    }
}

// --- RNG ---

const RNG: u32 = 0x5006_0800;
const CR_R: u32 = 0x00;
const SR_R: u32 = 0x04;
const DR_R: u32 = 0x08;
const RNGEN: u32 = 1 << 2;
const DRDY: u32 = 1 << 0;

#[test]
fn rng_produces_data() {
    let mut m = Machine::armv7m(board::stm32f407());
    let mem = m.core.mem();
    ww(mem, RNG + CR_R, RNGEN);
    pump(mem, 4);
    assert_ne!(rw(mem, RNG + SR_R) & DRDY, 0, "DRDY latches");
    let a = rw(mem, RNG + DR_R);
    pump(mem, 4);
    let b = rw(mem, RNG + DR_R);
    assert_ne!((a, b), (0, 0), "stream nonzero");
    assert_ne!(a, b, "stream advances");
}

// --- DCMI ---

const DCMI: u32 = 0x5005_0000;
const CR_D: u32 = 0x00;
const RIS: u32 = 0x08;
const DR_D: u32 = 0x28;
const CAPTURE: u32 = 1 << 0;
const ENABLE: u32 = 1 << 14;
const FRAMERI: u32 = 1 << 0;

#[test]
fn dcmi_captures_staged_frame() {
    let mut m = Machine::armv7m(board::stm32f407());
    m.drive_device("dcmi", &[0xDE, 0xAD, 0xBE, 0xEF]);
    let mem = m.core.mem();
    ww(mem, DCMI + CR_D, ENABLE | CAPTURE);
    pump(mem, 4);
    assert_ne!(rw(mem, DCMI + RIS) & FRAMERI, 0, "FRAMERI latches");
    assert_eq!(rw(mem, DCMI + DR_D), 0xEFBE_ADDE, "staged word pops LE");
}

// --- FSMC ---

const FSMC: u32 = 0xA000_0000;
const BCR1: u32 = 0x00;
const MBKEN: u32 = 1 << 0;
const NOR1: u32 = 0x6000_0000;

#[test]
fn fsmc_gates_nor_bank() {
    let mut m = Machine::armv7m(board::stm32f407());
    let mem = m.core.mem();
    ww(mem, FSMC + BCR1, 0x0000_30D0 | MBKEN);
    assert_eq!(rw(mem, NOR1), 0xFFFF_FFFF, "enabled bank floats");
}

// --- LTDC ---

const LTDC: u32 = 0x4001_6800;
const GCR: u32 = 0x18;
const SRCR: u32 = 0x24;
const ISR_L: u32 = 0x38;
const IER: u32 = 0x34;
const L1CR: u32 = 0x84;
const LTDCEN: u32 = 1;
const RRIE: u32 = 1 << 3;
const RRIF: u32 = 1 << 3;
const IMR: u32 = 1;
const LEN: u32 = 1;

#[test]
fn ltdc_reload_latches_rrif() {
    let mut m = Machine::armv7m(board::stm32f407());
    let mem = m.core.mem();
    ww(mem, LTDC + GCR, LTDCEN);
    ww(mem, LTDC + IER, RRIE);
    ww(mem, LTDC + L1CR, LEN);
    assert_eq!(rw(mem, LTDC + ISR_L) & RRIF, 0, "no reload yet");
    ww(mem, LTDC + SRCR, IMR);
    assert_ne!(rw(mem, LTDC + ISR_L) & RRIF, 0, "RRIF latches");
}

// --- HASH (SHA-1 "abc") ---

const HASH: u32 = 0x5006_0400;
const CR_H: u32 = 0x00;
const DIN: u32 = 0x04;
const STR: u32 = 0x08;
const HR0: u32 = 0x0C;
const INIT_DATATYPE: u32 = (2 << 4) | (1 << 2);

#[test]
fn hash_sha1_abc_vector() {
    let mut m = Machine::armv7m(board::stm32f407());
    let mem = m.core.mem();
    ww(mem, HASH + CR_H, INIT_DATATYPE);
    ww(mem, HASH + DIN, 0x0063_6261);
    ww(mem, HASH + STR, (1 << 8) | 24);
    pump(mem, 100);
    let d: Vec<u32> = (0..5).map(|i| rw(mem, HASH + HR0 + 4 * i)).collect();
    assert_eq!(
        d,
        [0xa9993e36, 0x4706816a, 0xba3e2571, 0x7850c26c, 0x9cd0d89d],
        "SHA-1(abc)"
    );
}

// --- SPI4-6 + I2S extensions ---

const SPI4: u32 = 0x4001_3400;
const SPI5: u32 = 0x4001_5000;
const SPI6: u32 = 0x4001_5400;
const I2S2EXT: u32 = 0x4000_3400;
const I2S3EXT: u32 = 0x4000_4000;
const CR1_S: u32 = 0x00;
const SR_S: u32 = 0x08;
const DR_S: u32 = 0x0C;
const SPE: u32 = 1 << 6;
const MSTR: u32 = 1 << 2;
const SSI: u32 = 1 << 8;
const SSM: u32 = 1 << 9;
const RXNE_S: u32 = 1 << 0;

fn spi_shifts(m: &mut Machine<board::BoardMemory>, base: u32, name: &str) {
    {
        let mem = m.core.mem();
        ww(mem, base + CR1_S, SPE | MSTR | SSI | SSM);
        ww(mem, base + DR_S, 0xA5);
        pump(mem, 100);
        assert_ne!(rw(mem, base + SR_S) & RXNE_S, 0, "RXNE after frame");
        assert_eq!(rw(mem, base + DR_S) & 0xFF, 0x00, "peerless reads zero");
    }
    m.drive_device(name, &[0x5A]);
    let byte = {
        let mem = m.core.mem();
        ww(mem, base + DR_S, 0xA5);
        pump(mem, 100);
        rw(mem, base + DR_S) & 0xFF
    };
    assert_eq!(byte, 0x5A, "slave byte");
}

#[test]
fn spi4_5_6_and_i2s_ext_shift() {
    let mut m = Machine::armv7m(board::stm32f407());
    spi_shifts(&mut m, SPI4, "spi4");
    spi_shifts(&mut m, SPI5, "spi5");
    spi_shifts(&mut m, SPI6, "spi6");
    spi_shifts(&mut m, I2S2EXT, "i2s2ext");
    spi_shifts(&mut m, I2S3EXT, "i2s3ext");
}

// --- SAI1 TX FIFO ---

const SAI1: u32 = 0x4001_5800;
const ACR1: u32 = 0x04;
const ADR: u32 = 0x20;
const ASR: u32 = 0x18;
const ACR2: u32 = 0x08;
const SAIAEN: u32 = 1 << 16;
const FLVL_MASK: u32 = 0x7 << 16;
const OVRUDR: u32 = 1 << 0;
const FFLUSH: u32 = 1 << 3;

#[test]
fn sai1_tx_fifo_fills_overruns_flushes() {
    let mut m = Machine::armv7m(board::stm32f407());
    let mem = m.core.mem();
    // Master transmitter, enabled, no clock divide: static FIFO.
    ww(mem, SAI1 + ACR1, SAIAEN);
    ww(mem, SAI1 + ADR, 0x1111_1111);
    ww(mem, SAI1 + ADR, 0x2222_2222);
    assert_eq!(rw(mem, SAI1 + ASR) & FLVL_MASK, 0x2 << 16, "FLVL == 2");
    for _ in 0..7 {
        ww(mem, SAI1 + ADR, 0x3333_3333);
    }
    assert_ne!(rw(mem, SAI1 + ASR) & OVRUDR, 0, "9th word overruns");
    ww(mem, SAI1 + ACR2, FFLUSH);
    assert_eq!(rw(mem, SAI1 + ASR) & FLVL_MASK, 0, "flush drains");
}

// --- CRYP (AES-128-ECB FIPS-197 B) ---

const CRYP: u32 = 0x5006_0000;
const CR_C: u32 = 0x00;
const SR_C: u32 = 0x04;
const DIN_C: u32 = 0x08;
const DOUT_C: u32 = 0x0C;
const K2LR: u32 = 0x30;
const ECB_EN: u32 = (0b0100 << 3) | (1 << 15);
const OFNE: u32 = 1 << 2;

#[test]
fn cryp_aes128_ecb_vector() {
    let mut m = Machine::armv7m(board::stm32f407());
    let mem = m.core.mem();
    // Key 000102..0f into K2LR..K3RR, MSB-first per word.
    for (i, w) in [0x00010203u32, 0x04050607, 0x08090a0b, 0x0c0d0e0f]
        .iter()
        .enumerate()
    {
        ww(mem, CRYP + K2LR + (i as u32) * 4, *w);
    }
    ww(mem, CRYP + CR_C, ECB_EN);
    for w in [0x00112233u32, 0x44556677, 0x8899aabb, 0xccddeeff] {
        ww(mem, CRYP + DIN_C, w);
    }
    pump(mem, 100);
    assert_ne!(rw(mem, CRYP + SR_C) & OFNE, 0, "output ready");
    let ct: Vec<u32> = (0..4).map(|_| rw(mem, CRYP + DOUT_C)).collect();
    assert_eq!(
        ct,
        [0x69c4e0d8, 0x6a7b0430, 0xd8cdb780, 0x70b4c55a],
        "FIPS-197 B"
    );
}
