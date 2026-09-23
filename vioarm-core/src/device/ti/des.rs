//! TI Tiva C DES engine (`DES`, `0x4403_8000`).
//!
//! Register map and `CTRL`/`IRQSTATUS` semantics follow the TI
//! `hw_des.h` + `des.c` oracle (BSD): `KEY1_L/H` 64-bit key (parity bits
//! carried as-written), `IV_L/H`, `CTRL` (`DIRECTION` 1 = encrypt,
//! `MODE` 0 = ECB / 1 = CBC, `TDES` selects triple-DES), `LENGTH` in
//! bytes, `DATA_L` then `DATA_H` (straight order, no reversal),
//! `OUTPUT_READY` synthesis, `IRQSTATUS.DATA_OUT` + `IRQENABLE` gating.
//!
//! Modeled: single-DES ECB + CBC encrypt/decrypt in pure software,
//! instant completion. Verified against the FIPS 46-3 reference vector
//! (`K=133457799BBCDFF1`, `P=0123456789ABCDEF` →
//! `C=85E813540F0AB405`). Deliberate scope: triple-DES, CFB, DMA,
//! `DIRTYBITS` side effects latch without acting (documented, untested
//! paths stay inert rather than wrong).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// `DES` base address.
pub const DES_BASE: u32 = 0x4403_8000;

const KEY1_L_OFF: u32 = 0x10;
const KEY1_H_OFF: u32 = 0x14;
const IV_L_OFF: u32 = 0x18;
const IV_H_OFF: u32 = 0x1C;
const CTRL_OFF: u32 = 0x20;
const DATA_L_OFF: u32 = 0x28;
const DATA_H_OFF: u32 = 0x2C;
const IRQSTATUS_OFF: u32 = 0x3C;
const IRQENABLE_OFF: u32 = 0x40;

/// `CTRL` bits.
const CTRL_OUTPUT_READY: u32 = 1 << 0;
const CTRL_INPUT_READY: u32 = 1 << 1;
const CTRL_DIRECTION: u32 = 1 << 2;
const CTRL_TDES: u32 = 1 << 3;
const CTRL_MODE_SHIFT: u32 = 4;
const CTRL_MODE_MASK: u32 = 0x03;
/// `IRQSTATUS` bits.
const IRQ_DATA_OUT: u32 = 1 << 2;

/// Initial permutation.
const IP: [u8; 64] = [
    58, 50, 42, 34, 26, 18, 10, 2, 60, 52, 44, 36, 28, 20, 12, 4, 62, 54, 46, 38, 30, 22, 14, 6,
    64, 56, 48, 40, 32, 24, 16, 8, 57, 49, 41, 33, 25, 17, 9, 1, 59, 51, 43, 35, 27, 19, 11, 3, 61,
    53, 45, 37, 29, 21, 13, 5, 63, 55, 47, 39, 31, 23, 15, 7,
];
/// Final permutation.
const FP: [u8; 64] = [
    40, 8, 48, 16, 56, 24, 64, 32, 39, 7, 47, 15, 55, 23, 63, 31, 38, 6, 46, 14, 54, 22, 62, 30,
    37, 5, 45, 13, 53, 21, 61, 29, 36, 4, 44, 12, 52, 20, 60, 28, 35, 3, 43, 11, 51, 19, 59, 27,
    34, 2, 42, 10, 50, 18, 58, 26, 33, 1, 41, 9, 49, 17, 57, 25,
];
/// Expansion.
const E: [u8; 48] = [
    32, 1, 2, 3, 4, 5, 4, 5, 6, 7, 8, 9, 8, 9, 10, 11, 12, 13, 12, 13, 14, 15, 16, 17, 16, 17, 18,
    19, 20, 21, 20, 21, 22, 23, 24, 25, 24, 25, 26, 27, 28, 29, 28, 29, 30, 31, 32, 1,
];
/// Straight permutation.
const P: [u8; 32] = [
    16, 7, 20, 21, 29, 12, 28, 17, 1, 15, 23, 26, 5, 18, 31, 10, 2, 8, 24, 14, 32, 27, 3, 9, 19,
    13, 30, 6, 22, 11, 4, 25,
];
/// S-boxes (row-major: 4 rows × 16 cols each).
const S: [[u8; 64]; 8] = [
    [
        14, 4, 13, 1, 2, 15, 11, 8, 3, 10, 6, 12, 5, 9, 0, 7, 0, 15, 7, 4, 14, 2, 13, 1, 10, 6, 12,
        11, 9, 5, 3, 8, 4, 1, 14, 8, 13, 6, 2, 11, 15, 12, 9, 7, 3, 10, 5, 0, 15, 12, 8, 2, 4, 9,
        1, 7, 5, 11, 3, 14, 10, 0, 6, 13,
    ],
    [
        15, 1, 8, 14, 6, 11, 3, 4, 9, 7, 2, 13, 12, 0, 5, 10, 3, 13, 4, 7, 15, 2, 8, 14, 12, 0, 1,
        10, 6, 9, 11, 5, 0, 14, 7, 11, 10, 4, 13, 1, 5, 8, 12, 6, 9, 3, 2, 15, 13, 8, 10, 1, 3, 15,
        4, 2, 11, 6, 7, 12, 0, 5, 14, 9,
    ],
    [
        10, 0, 9, 14, 6, 3, 15, 5, 1, 13, 12, 7, 11, 4, 2, 8, 13, 7, 0, 9, 3, 4, 6, 10, 2, 8, 5,
        14, 12, 11, 15, 1, 13, 6, 4, 9, 8, 15, 3, 0, 11, 1, 2, 12, 5, 10, 14, 7, 1, 10, 13, 0, 6,
        9, 8, 7, 4, 15, 14, 3, 11, 5, 2, 12,
    ],
    [
        7, 13, 14, 3, 0, 6, 9, 10, 1, 2, 8, 5, 11, 12, 4, 15, 13, 8, 11, 5, 6, 15, 0, 3, 4, 7, 2,
        12, 1, 10, 14, 9, 10, 6, 9, 0, 12, 11, 7, 13, 15, 1, 3, 14, 5, 2, 8, 4, 3, 15, 0, 6, 10, 1,
        13, 8, 9, 4, 5, 11, 12, 7, 2, 14,
    ],
    [
        2, 12, 4, 1, 7, 10, 11, 6, 8, 5, 3, 15, 13, 0, 14, 9, 14, 11, 2, 12, 4, 7, 13, 1, 5, 0, 15,
        10, 3, 9, 8, 6, 4, 2, 1, 11, 10, 13, 7, 8, 15, 9, 12, 5, 6, 3, 0, 14, 11, 8, 12, 7, 1, 14,
        2, 13, 6, 15, 0, 9, 10, 4, 5, 3,
    ],
    [
        12, 1, 10, 15, 9, 2, 6, 8, 0, 13, 3, 4, 14, 7, 5, 11, 10, 15, 4, 2, 7, 12, 9, 5, 6, 1, 13,
        14, 0, 11, 3, 8, 9, 14, 15, 5, 2, 8, 12, 3, 7, 0, 4, 10, 1, 13, 11, 6, 4, 3, 2, 12, 9, 5,
        15, 10, 11, 14, 1, 7, 6, 0, 8, 13,
    ],
    [
        4, 11, 2, 14, 15, 0, 8, 13, 3, 12, 9, 7, 5, 10, 6, 1, 13, 0, 11, 7, 4, 9, 1, 10, 14, 3, 5,
        12, 2, 15, 8, 6, 1, 4, 11, 13, 12, 3, 7, 14, 10, 15, 6, 8, 0, 5, 9, 2, 6, 11, 13, 8, 1, 4,
        10, 7, 9, 5, 0, 15, 14, 2, 3, 12,
    ],
    [
        13, 2, 8, 4, 6, 15, 11, 1, 10, 9, 3, 14, 5, 0, 12, 7, 1, 15, 13, 8, 10, 3, 7, 4, 12, 5, 6,
        11, 0, 14, 9, 2, 7, 11, 4, 1, 9, 12, 14, 2, 0, 6, 10, 13, 15, 3, 5, 8, 2, 1, 14, 7, 4, 10,
        8, 13, 15, 12, 9, 0, 3, 5, 6, 11,
    ],
];
/// Key schedule permutations + rotation schedule.
const PC1: [u8; 56] = [
    57, 49, 41, 33, 25, 17, 9, 1, 58, 50, 42, 34, 26, 18, 10, 2, 59, 51, 43, 35, 27, 19, 11, 3, 60,
    52, 44, 36, 63, 55, 47, 39, 31, 23, 15, 7, 62, 54, 46, 38, 30, 22, 14, 6, 61, 53, 45, 37, 29,
    21, 13, 5, 28, 20, 12, 4,
];
const PC2: [u8; 48] = [
    14, 17, 11, 24, 1, 5, 3, 28, 15, 6, 21, 10, 23, 19, 12, 4, 26, 8, 16, 7, 27, 20, 13, 2, 41, 52,
    31, 37, 47, 55, 30, 40, 51, 45, 33, 48, 44, 49, 39, 56, 34, 53, 46, 42, 50, 36, 29, 32,
];
const SHIFTS: [u32; 16] = [1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1];

/// Apply a 1-indexed-MSB bit permutation to a `width`-bit value held in
/// the low bits of `block`.
fn permute(block: u64, table: &[u8], width: u64) -> u64 {
    let mut out = 0u64;
    for &t in table {
        out = (out << 1) | ((block >> (width - u64::from(t))) & 1);
    }
    out
}

/// 16 DES subkeys from the 64-bit key (parity bits ignored).
fn subkeys(key: u64) -> [u64; 16] {
    let pc1 = permute(key, &PC1, 64);
    let mut c = ((pc1 >> 28) & 0x0FFF_FFFF) as u32;
    let mut d = (pc1 & 0x0FFF_FFFF) as u32;
    let mut ks = [0u64; 16];
    for (i, k) in ks.iter_mut().enumerate() {
        let s = SHIFTS[i];
        c = ((c << s) | (c >> (28 - s))) & 0x0FFF_FFFF;
        d = ((d << s) | (d >> (28 - s))) & 0x0FFF_FFFF;
        *k = permute((u64::from(c) << 28) | u64::from(d), &PC2, 56);
    }
    ks
}

/// Feistel function.
fn feistel(r: u32, k: u64) -> u32 {
    let r64 = u64::from(r);
    let mut e = 0u64;
    for &t in &E {
        e = (e << 1) | ((r64 >> (32 - u64::from(t))) & 1);
    }
    let x = e ^ k;
    let mut s = 0u32;
    for (i, sbox) in S.iter().enumerate() {
        let six = ((x >> (42 - 6 * i as u64)) & 0x3F) as usize;
        let row = ((six >> 4) & 0x02) | (six & 0x01);
        let col = (six >> 1) & 0x0F;
        s = (s << 4) | u32::from(sbox[row * 16 + col]);
    }
    let mut p = 0u32;
    for &t in &P {
        p = (p << 1) | ((u64::from(s) >> (32 - u64::from(t))) & 1) as u32;
    }
    p
}

/// One DES block (encrypt if `encrypt`, else decrypt).
fn des_block(block: u64, ks: &[u64; 16], encrypt: bool) -> u64 {
    let ip = permute(block, &IP, 64);
    let (mut l, mut r) = ((ip >> 32) as u32, (ip & 0xFFFF_FFFF) as u32);
    for i in 0..16 {
        let k = ks[if encrypt { i } else { 15 - i }];
        let next_l = r;
        r = l ^ feistel(r, k);
        l = next_l;
    }
    permute((u64::from(r) << 32) | u64::from(l), &FP, 64)
}

/// Behavior behind the DES window.
#[derive(Debug, Default)]
pub struct DesBehavior {
    out_lo: u32,
    out_hi: u32,
    output_ready: bool,
    data_out_irq: bool,
    /// CBC chaining value.
    chain: u64,
    chain_valid: bool,
}

impl Behavior for DesBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        *self = Self::default();
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(self.data_out_irq && view.get(IRQENABLE_OFF) & IRQ_DATA_OUT != 0)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            CTRL_OFF => {
                let mut ctrl = value & !(CTRL_OUTPUT_READY | CTRL_INPUT_READY);
                if self.output_ready {
                    ctrl |= CTRL_OUTPUT_READY;
                }
                ctrl |= CTRL_INPUT_READY;
                ctrl
            }
            DATA_L_OFF => self.out_lo,
            DATA_H_OFF => self.out_hi,
            0x3C => {
                let mut irq = 0;
                if self.data_out_irq {
                    irq |= IRQ_DATA_OUT;
                }
                irq
            }
            _ => value,
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == CTRL_OFF {
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            self.chain_valid = false;
            self.output_ready = false;
            self.data_out_irq = false;
            return word;
        }
        if offset == 0x3C {
            return view.get(offset);
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        // `DATA_H` completes the block (TivaWare writes `DATA_L` first);
        // ECB/CBC single-DES only, else inert.
        if offset == DATA_H_OFF && shift == 24 {
            let ctrl = view.get(CTRL_OFF);
            let tdes = ctrl & CTRL_TDES != 0;
            let mode = (ctrl >> CTRL_MODE_SHIFT) & CTRL_MODE_MASK;
            if !tdes && mode <= 1 {
                let key = (u64::from(view.get(KEY1_H_OFF)) << 32) | u64::from(view.get(KEY1_L_OFF));
                let ks = subkeys(key);
                let block =
                    (u64::from(view.get(DATA_H_OFF)) << 32) | u64::from(view.get(DATA_L_OFF));
                let encrypt = ctrl & CTRL_DIRECTION != 0;
                let out = if mode == 1 {
                    let iv = (u64::from(view.get(IV_H_OFF)) << 32) | u64::from(view.get(IV_L_OFF));
                    if !self.chain_valid {
                        self.chain = iv;
                        self.chain_valid = true;
                    }
                    if encrypt {
                        let out = des_block(block ^ self.chain, &ks, true);
                        self.chain = out;
                        out
                    } else {
                        let out = des_block(block, &ks, false) ^ self.chain;
                        self.chain = block;
                        out
                    }
                } else if encrypt {
                    des_block(block, &ks, true)
                } else {
                    des_block(block, &ks, false)
                };
                self.out_hi = (out >> 32) as u32;
                self.out_lo = (out & 0xFFFF_FFFF) as u32;
                self.output_ready = true;
                self.data_out_irq = true;
            }
        }
        word
    }
}

/// A Tiva C DES engine (single-DES ECB/CBC; triple-DES latches).
pub struct Des {
    inner: RegisterFile<DesBehavior>,
}

impl Des {
    /// A new DES engine on `D`'s tables (`"DES"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"DES"`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("DES").expect("generated tables contain DES");
        Self {
            inner: RegisterFile::new(def, DesBehavior::default()),
        }
    }
}

impl Peripheral for Des {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for Des {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c129xnczad::Tm4c129xnczad;

    const DES0_BASE: u32 = 0x4403_8000;
    const CTRL_ENC_ECB: u32 = CTRL_DIRECTION;
    const CTRL_DEC_ECB: u32 = 0x00;
    const CTRL_ENC_CBC: u32 = CTRL_DIRECTION | (1 << CTRL_MODE_SHIFT);

    fn new_des() -> Des {
        Des::for_device::<Tm4c129xnczad>()
    }

    fn read_word(des: &mut Des, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(des.read(DES0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(des: &mut Des, offset: u32, value: u32) {
        for lane in 0..4 {
            des.write(
                DES0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Classic FIPS vector: `K=133457799BBCDFF1`, `P=0123456789ABCDEF`
    /// → `C=85E813540F0AB405`.
    fn setup(des: &mut Des, key_hi: u32, key_lo: u32, ctrl: u32) {
        write_word(des, KEY1_H_OFF, key_hi);
        write_word(des, KEY1_L_OFF, key_lo);
        write_word(des, CTRL_OFF, ctrl);
    }

    fn block(des: &mut Des, hi: u32, lo: u32) -> (u32, u32) {
        write_word(des, DATA_L_OFF, lo);
        write_word(des, DATA_H_OFF, hi);
        (read_word(des, DATA_H_OFF), read_word(des, DATA_L_OFF))
    }

    #[test]
    fn fips46_encrypt() {
        let mut des = new_des();
        setup(&mut des, 0x1334_5779, 0x9BBC_DFF1, CTRL_ENC_ECB);
        assert_eq!(
            block(&mut des, 0x0123_4567, 0x89AB_CDEF),
            (0x85E8_1354, 0x0F0A_B405)
        );
        assert_eq!(
            read_word(&mut des, CTRL_OFF) & CTRL_OUTPUT_READY,
            CTRL_OUTPUT_READY
        );
        assert_eq!(read_word(&mut des, 0x3C) & IRQ_DATA_OUT, IRQ_DATA_OUT);
    }

    #[test]
    fn fips46_decrypt() {
        let mut des = new_des();
        setup(&mut des, 0x1334_5779, 0x9BBC_DFF1, CTRL_DEC_ECB);
        assert_eq!(
            block(&mut des, 0x85E8_1354, 0x0F0A_B405),
            (0x0123_4567, 0x89AB_CDEF)
        );
    }

    /// CBC chains through the IV (NIST SP 800-67 `TCBCINV` spot shape:
    /// first block equals ECB of `P^IV`).
    #[test]
    fn cbc_chains_iv() {
        let mut des = new_des();
        setup(&mut des, 0x1334_5779, 0x9BBC_DFF1, CTRL_ENC_CBC);
        write_word(&mut des, IV_H_OFF, 0x0123_4567);
        write_word(&mut des, IV_L_OFF, 0x89AB_CDEF);
        // Block 1: E(P1 ^ IV).
        let (hi, lo) = block(&mut des, 0x0123_4567, 0x89AB_CDEF);
        // P1 ^ IV = 0 → E(0) under this key.
        let mut check = new_des();
        setup(&mut check, 0x1334_5779, 0x9BBC_DFF1, CTRL_ENC_ECB);
        assert_eq!(block(&mut check, 0, 0), (hi, lo));
        // Block 2 chains on block-1 ciphertext.
        let (hi2, _) = block(&mut des, 0x0123_4567, 0x89AB_CDEF);
        assert_ne!((hi2, lo), (hi, lo));
    }

    /// `IRQENABLE` gates the line; triple-DES stays inert.
    #[test]
    fn irq_gating_and_deferred_tdes() {
        let mut des = new_des();
        write_word(&mut des, IRQENABLE_OFF, IRQ_DATA_OUT);
        setup(&mut des, 0x1334_5779, 0x9BBC_DFF1, CTRL_ENC_ECB);
        block(&mut des, 0x0123_4567, 0x89AB_CDEF);
        assert_eq!(des.pending(), 1);
        let mut des = new_des();
        setup(&mut des, 0x1334_5779, 0x9BBC_DFF1, CTRL_ENC_ECB | CTRL_TDES);
        block(&mut des, 0x0123_4567, 0x89AB_CDEF);
        assert_eq!(read_word(&mut des, 0x3C) & IRQ_DATA_OUT, 0);
    }
}
