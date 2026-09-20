//! F4 cryptographic processor (`CRYP@0x50060000`) — AES-ECB/CBC/CTR over
//! the shared FIPS-197 core.
//!
//! Register layout comes from the SVD. Key/IV conventions follow Linux's
//! `stm32_cryp` driver, cross-checked against the reference-manual key
//! description: every key/IV register holds four consecutive bytes
//! MSB-first, and the LAST key word lives in `K3RR` (`K2LR..K3RR` for
//! 128-bit, `K1RR..K3RR` for 192-bit, all eight for 256-bit). `DIN`
//! words assemble MSB-first with `DATATYPE` swapping applied both ways
//! (00 none, 01 halfword-swap, 10 byte-swap, 11 per-byte bit-reverse).
//!
//! Four `DIN` words make a block: with `CRYPEN` set in an AES mode the
//! engine pulses `BUSY`, consumes one block per 16 ticks, and lands four
//! `DOUT` words (`IFEM`/`IFNF`/`OFNE`/`OFNF` track the FIFOs; reading an
//! empty `DOUT` latches `OFFU`). CBC chains across blocks within a
//! `CRYPEN` session (IV sampled at the first block); CTR runs a
//! big-endian 128-bit counter from the IV, like the U5 engine. ECB/CBC
//! decrypt invert directly, so the key-preparation mode (`ALGOMODE 0111`)
//! just pulses `BUSY` with no output. `FFLUSH` drops the FIFOs
//! (self-clearing); key/IV writes are ignored while busy.
//!
//! Deliberately unmodeled: TDES/DES and GCM/CCM modes (`CRYPEN` there
//! latches nothing and raises nothing — no firmware in scope uses them),
//! DMA (`DMACR` latches), context save/restore (`CSGCM*` latch), RX-side
//! anything beyond the documented flags. `INIM`/`OUTIM` gate a single
//! interrupt line on input-room/output-ready.

use super::aes::{cipher_block, inv_cipher_block, key_expand};
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;

const CR: u32 = 0x00;
const SR: u32 = 0x04;
const DIN: u32 = 0x08;
const DOUT: u32 = 0x0C;
const IMSCR: u32 = 0x14;
const RISR: u32 = 0x18;
const MISR: u32 = 0x1C;
const K0LR: u32 = 0x20;
const IV0LR: u32 = 0x40;

const CR_ALGODIR: u32 = 1 << 2;
const CR_ALGOMODE_SHIFT: u32 = 3;
const CR_DATATYPE_SHIFT: u32 = 6;
const CR_KEYSIZE_SHIFT: u32 = 8;
const CR_FFLUSH: u32 = 1 << 14;
const CR_CRYPEN: u32 = 1 << 15;
const MODE_AES_ECB: u32 = 0b0100;
const MODE_AES_CBC: u32 = 0b0101;
const MODE_AES_CTR: u32 = 0b0110;
const MODE_KEY_PREP: u32 = 0b0111;

const SR_IFEM: u32 = 1 << 0;
const SR_IFNF: u32 = 1 << 1;
const SR_OFNE: u32 = 1 << 2;
const SR_OFNF: u32 = 1 << 3;
const SR_BUSY: u32 = 1 << 4;
const SR_OFFU: u32 = 1 << 3;

const IMSCR_INIM: u32 = 1 << 0;
const IMSCR_OUTIM: u32 = 1 << 1;
const RISR_INRIS: u32 = 1 << 0;
const RISR_OUTRIS: u32 = 1 << 1;

/// Ticks to consume one block once scheduled.
const BLOCK_TICKS: u64 = 16;
/// FIFO depth in words (two blocks each way).
const FIFO_WORDS: usize = 8;

#[derive(Clone, Debug, Default)]
struct CrypBehavior {
    din: VecDeque<u32>,
    dout: VecDeque<u32>,
    din_acc: u32,
    din_mask: u8,
    r_word: u32,
    r_lane: u8,
    remaining: u64,
    chain: [u8; 16],
    chain_valid: bool,
    /// Key-preparation completed in this session (stops re-pulsing).
    prepped: bool,
}

impl CrypBehavior {
    fn mode(view: &Registers<&mut [u32]>) -> u32 {
        let cr = view.get(CR);
        ((cr >> CR_ALGOMODE_SHIFT) & 0x7) | ((cr >> 19) & 0x8)
    }

    fn supported(mode: u32) -> bool {
        matches!(
            mode,
            MODE_AES_ECB | MODE_AES_CBC | MODE_AES_CTR | MODE_KEY_PREP
        )
    }

    fn enabled(view: &Registers<&mut [u32]>) -> bool {
        view.get(CR) & CR_CRYPEN != 0
    }

    fn key_bytes(view: &Registers<&mut [u32]>) -> Vec<u8> {
        let nwords = match (view.get(CR) >> CR_KEYSIZE_SHIFT) & 0x3 {
            1 => 6,
            2 => 8,
            _ => 4,
        };
        // Last key word in K3RR, each word MSB-first.
        let mut out = vec![0u8; nwords * 4];
        for (i, chunk) in out.chunks_exact_mut(4).enumerate() {
            let reg = K0LR + 4 * (8 - nwords + i) as u32;
            let w = view.get(reg);
            chunk.copy_from_slice(&w.to_be_bytes());
        }
        out
    }

    fn iv_bytes(view: &Registers<&mut [u32]>) -> [u8; 16] {
        let mut out = [0u8; 16];
        for (i, chunk) in out.chunks_exact_mut(4).enumerate() {
            let w = view.get(IV0LR + 4 * i as u32);
            chunk.copy_from_slice(&w.to_be_bytes());
        }
        out
    }

    /// Apply `DATATYPE` swapping to one input/output word.
    fn swap_word(view: &Registers<&mut [u32]>, w: u32) -> u32 {
        match (view.get(CR) >> CR_DATATYPE_SHIFT) & 0x3 {
            1 => w.rotate_left(16),
            2 => w.swap_bytes(),
            3 => {
                let mut r = 0u32;
                for i in 0..4 {
                    let byte = ((w >> (8 * i)) & 0xFF) as u8;
                    r |= u32::from(byte.reverse_bits()) << (8 * i);
                }
                r
            }
            _ => w,
        }
    }

    fn word_bytes(view: &Registers<&mut [u32]>, w: u32) -> [u8; 4] {
        Self::swap_word(view, w).to_be_bytes()
    }

    fn bytes_word(view: &Registers<&mut [u32]>, b: &[u8; 4]) -> u32 {
        let w = u32::from_be_bytes(*b);
        Self::swap_word(view, w)
    }

    fn sr_bits(&self, view: &Registers<&mut [u32]>) -> u32 {
        let mut sr = 0;
        if self.din.is_empty() {
            sr |= SR_IFEM;
        }
        if self.din.len() < FIFO_WORDS {
            sr |= SR_IFNF;
        }
        if !self.dout.is_empty() {
            sr |= SR_OFNE;
        }
        if self.dout.len() >= FIFO_WORDS {
            sr |= SR_OFNF;
        }
        if self.remaining > 0 {
            sr |= SR_BUSY;
        }
        sr |= view.get(SR) & SR_OFFU;
        sr
    }

    fn kick(&mut self, view: &Registers<&mut [u32]>) {
        if !Self::enabled(view) || !Self::supported(Self::mode(view)) || self.remaining != 0 {
            return;
        }
        // Key preparation runs once per session with no input; data
        // modes need a full block staged.
        if Self::mode(view) == MODE_KEY_PREP {
            if !self.prepped {
                self.remaining = BLOCK_TICKS;
            }
        } else if self.din.len() >= 4 {
            self.remaining = BLOCK_TICKS;
        }
    }

    fn process_block(&mut self, view: &mut Registers<&mut [u32]>) {
        let mode = Self::mode(view);
        if mode == MODE_KEY_PREP {
            self.prepped = true;
            return;
        }
        let cr = view.get(CR);
        let decrypt = cr & CR_ALGODIR != 0;
        let key = Self::key_bytes(view);
        let rk = key_expand(&key);
        let rounds = rk.len() / 4 - 1;
        if !self.chain_valid {
            let iv = Self::iv_bytes(view);
            self.chain = iv;
            self.chain_valid = true;
        }
        let mut block = [0u8; 16];
        for (i, chunk) in block.chunks_exact_mut(4).enumerate() {
            let w = self.din.pop_front().unwrap_or(0);
            chunk.copy_from_slice(&Self::word_bytes(view, w));
        }
        let out = match mode {
            MODE_AES_ECB => {
                if decrypt {
                    inv_cipher_block(&block, &rk, rounds)
                } else {
                    cipher_block(&block, &rk, rounds)
                }
            }
            MODE_AES_CBC => {
                if decrypt {
                    let mut pt = inv_cipher_block(&block, &rk, rounds);
                    for (i, b) in pt.iter_mut().enumerate() {
                        *b ^= self.chain[i];
                    }
                    self.chain = block;
                    pt
                } else {
                    let mut xored = [0u8; 16];
                    for (i, b) in xored.iter_mut().enumerate() {
                        *b = block[i] ^ self.chain[i];
                    }
                    let ct = cipher_block(&xored, &rk, rounds);
                    self.chain = ct;
                    ct
                }
            }
            _ => {
                // CTR: keystream from the big-endian counter, both ways.
                let ks = cipher_block(&self.chain, &rk, rounds);
                let mut out = [0u8; 16];
                for (i, b) in out.iter_mut().enumerate() {
                    *b = block[i] ^ ks[i];
                }
                let mut ctr = u128::from_be_bytes(self.chain);
                ctr = ctr.wrapping_add(1);
                self.chain = ctr.to_be_bytes();
                out
            }
        };
        for chunk in out.chunks_exact(4) {
            let mut b = [0u8; 4];
            b.copy_from_slice(chunk);
            self.dout.push_back(Self::bytes_word(view, &b));
        }
    }
}

impl Behavior for CrypBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            CR => {
                let old = view.get(CR);
                let new = view.merged(CR, val, shift);
                if new & CR_FFLUSH != 0 {
                    self.din.clear();
                    self.dout.clear();
                    self.din_acc = 0;
                    self.din_mask = 0;
                    self.chain_valid = false;
                    self.prepped = false;
                    self.remaining = 0;
                    return new & !CR_FFLUSH;
                }
                if new & CR_CRYPEN != 0 && old & CR_CRYPEN == 0 {
                    // Fresh session samples the IV at its first block.
                    self.chain_valid = false;
                    self.prepped = false;
                    view.set(CR, new);
                    self.kick(view);
                    return new;
                }
                if new & CR_CRYPEN == 0 {
                    self.remaining = 0;
                }
                new
            }
            DIN => {
                // Lanes assemble one FIFO word; a full FIFO drops extras.
                if shift == 0 {
                    self.din_acc = 0;
                    self.din_mask = 0;
                }
                self.din_acc |= u32::from(val) << shift;
                self.din_mask |= 1 << (shift / 8);
                if self.din_mask == 0xF {
                    let w = self.din_acc;
                    self.din_acc = 0;
                    self.din_mask = 0;
                    if self.din.len() < FIFO_WORDS {
                        self.din.push_back(w);
                        self.kick(view);
                    }
                }
                view.merged(offset, val, shift)
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            SR => self.sr_bits(view),
            RISR => {
                let mut r = 0;
                if Self::enabled(view) && self.din.len() < FIFO_WORDS {
                    r |= RISR_INRIS;
                }
                if !self.dout.is_empty() {
                    r |= RISR_OUTRIS;
                }
                r
            }
            MISR => {
                let imsc = view.get(IMSCR);
                let mut r = 0;
                if Self::enabled(view) && self.din.len() < FIFO_WORDS && imsc & 1 != 0 {
                    r |= RISR_INRIS;
                }
                if !self.dout.is_empty() && imsc & 2 != 0 {
                    r |= RISR_OUTRIS;
                }
                r
            }
            DOUT => {
                // Lanes share one pop per word, like the SAI DR path.
                if self.r_lane == 0 {
                    match self.dout.pop_front() {
                        Some(w) => self.r_word = w,
                        None => {
                            view.set(SR, view.get(SR) | SR_OFFU);
                            self.r_word = 0;
                        }
                    }
                }
                let w = self.r_word;
                self.r_lane += 1;
                if self.r_lane == 4 {
                    self.r_lane = 0;
                }
                w
            }
            _ => value,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if self.remaining == 0 {
            self.kick(view);
            return;
        }
        if self.remaining > delta {
            self.remaining -= delta;
            return;
        }
        self.remaining = 0;
        self.process_block(view);
        self.kick(view);
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let imsc = view.get(IMSCR);
        let cr = view.get(CR);
        let inr = cr & CR_CRYPEN != 0 && self.din.len() < FIFO_WORDS && imsc & 1 != 0;
        let outr = !self.dout.is_empty() && imsc & 2 != 0;
        u32::from(inr || outr)
    }

    fn reset(&mut self) {
        self.din.clear();
        self.dout.clear();
        self.din_acc = 0;
        self.din_mask = 0;
        self.r_word = 0;
        self.r_lane = 0;
        self.remaining = 0;
        self.chain_valid = false;
        self.prepped = false;
    }
}

/// F4 cryptographic processor (AES subset).
#[derive(Debug)]
pub struct Cryp {
    inner: RegisterFile<CrypBehavior>,
}

impl Cryp {
    /// A new `CRYP` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `CRYP`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("CRYP").expect("generated tables contain CRYP");
        Self {
            inner: RegisterFile::new(def, CrypBehavior::default()),
        }
    }
}

impl Peripheral for Cryp {
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

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }

    fn name(&self) -> &'static str {
        "cryp"
    }
}

impl InterruptSource for Cryp {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32f407::Stm32f407;
    use crate::device::{InterruptSource, Peripheral};

    const BASE: u32 = 0x5006_0000;
    const CRYP_CR: u32 = 0x00;
    const CRYP_DIN: u32 = 0x08;
    const CRYP_DOUT: u32 = 0x0C;
    const IV0LR: u32 = 0x40;

    fn hex(s: &[u8]) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(std::str::from_utf8(&s[i..i + 2]).unwrap(), 16).unwrap())
            .collect()
    }

    #[allow(clippy::cast_possible_truncation)]
    fn w(c: &mut Cryp, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            c.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(c: &mut Cryp, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = c.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    /// Program `key` (big-endian bytes) into the key registers per the
    /// Linux-driver convention, set `cr`, and return the device.
    fn setup(cr: u32, key_hex: &[u8]) -> Cryp {
        let mut c = Cryp::new_for::<Stm32f407>();
        c.reset();
        let key = hex(key_hex);
        let nwords = key.len() / 4;
        for (i, chunk) in key.chunks_exact(4).enumerate() {
            let off = 0x20 + 4 * (8 - nwords + i) as u32;
            w(
                &mut c,
                off,
                u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
            );
        }
        w(&mut c, CRYP_CR, cr);
        c
    }

    fn push_block(c: &mut Cryp, bytes: &[u8]) {
        for chunk in bytes.chunks_exact(4) {
            w(
                c,
                CRYP_DIN,
                u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
            );
        }
    }

    fn pull_block(c: &mut Cryp) -> Vec<u8> {
        let mut out = Vec::new();
        for _ in 0..4 {
            out.extend(r(c, CRYP_DOUT).to_be_bytes());
        }
        out
    }

    fn run(c: &mut Cryp) {
        for _ in 0..4 {
            c.tick(BLOCK_TICKS);
        }
    }

    #[test]
    fn aes128_ecb_fips197_b() {
        let mut c = setup(0x8020, b"000102030405060708090a0b0c0d0e0f");
        push_block(&mut c, &hex(b"00112233445566778899aabbccddeeff"));
        run(&mut c);
        assert_eq!(
            pull_block(&mut c),
            hex(b"69c4e0d86a7b0430d8cdb78070b4c55a"),
            "FIPS-197 B"
        );
    }

    #[test]
    fn aes128_ecb_decrypt_roundtrips() {
        let ct = hex(b"69c4e0d86a7b0430d8cdb78070b4c55a");
        let mut c = setup(0x8024, b"000102030405060708090a0b0c0d0e0f");
        push_block(&mut c, &ct);
        run(&mut c);
        assert_eq!(
            pull_block(&mut c),
            hex(b"00112233445566778899aabbccddeeff"),
            "ECB decrypt inverts"
        );
    }

    #[test]
    fn aes256_ecb_fips197_c3() {
        let mut c = setup(
            0x8220,
            b"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f",
        );
        push_block(&mut c, &hex(b"00112233445566778899aabbccddeeff"));
        run(&mut c);
        assert_eq!(
            pull_block(&mut c),
            hex(b"8ea2b7ca516745bfeafc49904b496089"),
            "FIPS-197 C.3"
        );
    }

    #[test]
    fn aes128_cbc_sp800_38a_f21() {
        let mut c = setup(0x8028, b"2b7e151628aed2a6abf7158809cf4f3c");
        let iv = hex(b"000102030405060708090a0b0c0d0e0f");
        for (i, chunk) in iv.chunks_exact(4).enumerate() {
            w(
                &mut c,
                IV0LR + 4 * i as u32,
                u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
            );
        }
        push_block(&mut c, &hex(b"6bc1bee22e409f96e93d7e117393172a"));
        run(&mut c);
        assert_eq!(
            pull_block(&mut c),
            hex(b"7649abac8119b246cee98e9b12e9197d"),
            "SP 800-38A F.2.1"
        );
        // Decrypt passes the IV again (fresh session).
        let mut c = setup(0x8028 | CR_ALGODIR, b"2b7e151628aed2a6abf7158809cf4f3c");
        let iv = hex(b"000102030405060708090a0b0c0d0e0f");
        for (i, chunk) in iv.chunks_exact(4).enumerate() {
            w(
                &mut c,
                IV0LR + 4 * i as u32,
                u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
            );
        }
        push_block(&mut c, &hex(b"7649abac8119b246cee98e9b12e9197d"));
        run(&mut c);
        assert_eq!(
            pull_block(&mut c),
            hex(b"6bc1bee22e409f96e93d7e117393172a"),
            "CBC decrypt round-trips"
        );
    }

    #[test]
    fn aes128_ctr_sp800_38a_f51() {
        let mut c = setup(0x8030, b"2b7e151628aed2a6abf7158809cf4f3c");
        let ctr = hex(b"f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff");
        for (i, chunk) in ctr.chunks_exact(4).enumerate() {
            w(
                &mut c,
                IV0LR + 4 * i as u32,
                u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
            );
        }
        push_block(&mut c, &hex(b"6bc1bee22e409f96e93d7e117393172a"));
        run(&mut c);
        assert_eq!(
            pull_block(&mut c),
            hex(b"874d6191b620e3261bef6864990db6ce"),
            "SP 800-38A F.5.1"
        );
    }

    #[test]
    fn datatype_swaps_roundtrip() {
        for dt in [1u32, 2, 3] {
            let mut c = setup(0x8020 | (dt << 6), b"000102030405060708090a0b0c0d0e0f");
            let pt = hex(b"00112233445566778899aabbccddeeff");
            push_block(&mut c, &pt);
            run(&mut c);
            let ct = pull_block(&mut c);
            assert_ne!(
                ct,
                hex(b"69c4e0d86a7b0430d8cdb78070b4c55a"),
                "swap changes output"
            );
            let mut d = setup(
                0x8020 | (dt << 6) | CR_ALGODIR,
                b"000102030405060708090a0b0c0d0e0f",
            );
            push_block(&mut d, &ct);
            run(&mut d);
            assert_eq!(pull_block(&mut d), pt, "swapped decrypt round-trips");
        }
    }

    #[test]
    fn key_prep_pulses_busy_without_output() {
        let mut c = setup(0x8038 | 0x8000, b"000102030405060708090a0b0c0d0e0f");
        assert_ne!(r(&mut c, 0x04) & SR_BUSY, 0, "BUSY during prep");
        run(&mut c);
        assert_eq!(r(&mut c, 0x04) & SR_BUSY, 0, "BUSY clears");
        assert_eq!(r(&mut c, 0x04) & SR_OFNE, 0, "no output block");
    }

    #[test]
    fn unmodeled_tdes_stays_idle() {
        let mut c = setup(0x8000, b"000102030405060708090a0b0c0d0e0f");
        push_block(&mut c, &hex(b"00112233445566778899aabbccddeeff"));
        run(&mut c);
        assert_eq!(r(&mut c, 0x04) & SR_BUSY, 0, "never busy");
        assert_eq!(r(&mut c, 0x04) & SR_OFNE, 0, "no output block");
    }

    #[test]
    fn irq_lines_mask() {
        let mut c = setup(0x8020, b"000102030405060708090a0b0c0d0e0f");
        w(&mut c, 0x14, 3); // INIM + OUTIM
        assert_eq!(c.pending(), 1, "input room pends");
        push_block(&mut c, &hex(b"00112233445566778899aabbccddeeff"));
        run(&mut c);
        assert_eq!(c.pending(), 1, "output ready pends");
    }
}
