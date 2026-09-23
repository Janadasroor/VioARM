//! TI Tiva C AES engine (`AES`, `0x4403_6000`).
//!
//! Register map and `CTRL`/`IRQSTATUS` semantics follow the TI
//! `hw_aes.h` + `aes.c` oracle (BSD): `KEY1_0-7` (LSW first; 4/6/8 words
//! for 128/192/256), `IV_IN_0-3`, `CTRL` as a plain config word
//! (`DIRECTION` 1 = encrypt, `MODE` 0 = ECB / 1 = CBC, `KEY_SIZE`
//! 1/2/3 = 128/192/256), `C_LENGTH` (0 = infinite for basic modes),
//! `DATA_IN_3-0` carrying words 0-3 (reversed: `src[0]` → `DATA_IN_3`),
//! result read back from the same regs, `OUTPUT_READY`/`INPUT_READY`
//! synthesis, `IRQSTATUS.DATA_OUT` + `IRQENABLE` gating.
//!
//! Modeled: AES-128/192/256 ECB + CBC encrypt/decrypt in pure software
//! (S-box computed at runtime — no transcribed tables), instant
//! completion. Verified against FIPS-197 Appendix B/C (`B` ECB-128,
//! `C.1` ECB-256) and SP 800-38A CBC-128. Deliberate scope: CTR/ICM/CFB/
//! XTS/F8/F9/CBC-MAC/GCM/CCM/DMA/`SAVE_CONTEXT` latch without acting
//! (documented, untested paths stay inert rather than wrong).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// `AES` base address.
pub const AES_BASE: u32 = 0x4403_6000;

const KEY1_OFF: u32 = 0x38;
/// `KEY1` word `j` lives at these offsets (interleaved pairs:
/// even words `0,2,4,6` count down `0x38,0x30,0x28,0x20`, odd words
/// `1,3,5,7` count down `0x3C,0x34,0x2C,0x24` — `hw_aes.h`).
const KEY1_WORD_OFF: [u32; 8] = [0x38, 0x3C, 0x30, 0x34, 0x28, 0x2C, 0x20, 0x24];
const IV_OFF: u32 = 0x40;
const CTRL_OFF: u32 = 0x50;
const DATA_OFF: u32 = 0x60;
const IRQSTATUS_OFF: u32 = 0x8C;
const IRQENABLE_OFF: u32 = 0x90;

/// `CTRL` bits.
const CTRL_OUTPUT_READY: u32 = 1 << 0;
const CTRL_INPUT_READY: u32 = 1 << 1;
const CTRL_DIRECTION: u32 = 1 << 2;
const CTRL_KEY_SIZE_SHIFT: u32 = 3;
const CTRL_KEY_SIZE_MASK: u32 = 0x03;
const CTRL_MODE: u32 = 1 << 5;
/// `IRQSTATUS` bits.
const IRQ_DATA_OUT: u32 = 1 << 2;

/// Galois multiply by 2 (`xtime`).
fn xtime(x: u8) -> u8 {
    if x & 0x80 != 0 {
        (x << 1) ^ 0x1B
    } else {
        x << 1
    }
}

/// Multiplicative inverse in GF(2^8)/`0x11B` (exponentiation: `x^254`).
fn gf_inv(x: u8) -> u8 {
    if x == 0 {
        return 0;
    }
    let mut r = 1u8;
    let mut b = x;
    let mut e = 254u32;
    while e > 0 {
        if e & 1 != 0 {
            r = gf_mul(r, b);
        }
        b = gf_mul(b, b);
        e >>= 1;
    }
    r
}

/// Galois multiply (Russian peasant).
fn gf_mul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0u8;
    while b != 0 {
        if b & 1 != 0 {
            p ^= a;
        }
        a = xtime(a);
        b >>= 1;
    }
    p
}

/// AES S-box (computed: inverse + affine transform).
fn sbox(x: u8) -> u8 {
    let i = gf_inv(x);
    i ^ i.rotate_left(1) ^ i.rotate_left(2) ^ i.rotate_left(3) ^ i.rotate_left(4) ^ 0x63
}

/// Inverse AES S-box (computed by search; 256-entry scan is cheap).
fn inv_sbox(y: u8) -> u8 {
    for x in 0..=255u8 {
        if sbox(x) == y {
            return x;
        }
    }
    0
}

/// Expand `key` (16/24/32 bytes) into `nr+1` round keys (16 bytes each).
fn expand_key(key: &[u8]) -> Vec<[u8; 16]> {
    let nk = key.len() / 4;
    let nr = nk + 6;
    let mut w = vec![0u32; 4 * (nr + 1)];
    for (i, chunk) in key.chunks(4).enumerate() {
        w[i] = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    let mut rcon = 1u8;
    for i in nk..4 * (nr + 1) {
        let mut t = w[i - 1];
        if i % nk == 0 {
            t = u32::from_be_bytes([
                sbox((t >> 16) as u8) ^ rcon,
                sbox((t >> 8) as u8),
                sbox(t as u8),
                sbox((t >> 24) as u8),
            ]);
            rcon = xtime(rcon);
        } else if nk > 6 && i % nk == 4 {
            t = u32::from_be_bytes([
                sbox((t >> 24) as u8),
                sbox((t >> 16) as u8),
                sbox((t >> 8) as u8),
                sbox(t as u8),
            ]);
        }
        w[i] = w[i - nk] ^ t;
    }
    w.chunks(4)
        .map(|rk| {
            let mut b = [0u8; 16];
            for (j, &word) in rk.iter().enumerate() {
                b[4 * j..4 * j + 4].copy_from_slice(&word.to_be_bytes());
            }
            b
        })
        .collect()
}

/// One AES block encrypt (`pt`, `rk` round keys).
fn encrypt_block(pt: &[u8; 16], rk: &[[u8; 16]]) -> [u8; 16] {
    let mut s = *pt;
    for (sb, &kb) in s.iter_mut().zip(rk[0].iter()) {
        *sb ^= kb;
    }
    for round in 1..rk.len() {
        for sb in s.iter_mut() {
            *sb = sbox(*sb);
        }
        // ShiftRows (column-major state).
        s = [
            s[0], s[5], s[10], s[15], s[4], s[9], s[14], s[3], s[8], s[13], s[2], s[7], s[12],
            s[1], s[6], s[11],
        ];
        if round < rk.len() - 1 {
            for c in 0..4 {
                let (a0, a1, a2, a3) = (s[4 * c], s[4 * c + 1], s[4 * c + 2], s[4 * c + 3]);
                s[4 * c] = gf_mul(a0, 2) ^ gf_mul(a1, 3) ^ a2 ^ a3;
                s[4 * c + 1] = a0 ^ gf_mul(a1, 2) ^ gf_mul(a2, 3) ^ a3;
                s[4 * c + 2] = a0 ^ a1 ^ gf_mul(a2, 2) ^ gf_mul(a3, 3);
                s[4 * c + 3] = gf_mul(a0, 3) ^ a1 ^ a2 ^ gf_mul(a3, 2);
            }
        }
        for (sb, &kb) in s.iter_mut().zip(rk[round].iter()) {
            *sb ^= kb;
        }
    }
    s
}

/// One AES block decrypt.
fn decrypt_block(ct: &[u8; 16], rk: &[[u8; 16]]) -> [u8; 16] {
    let mut s = *ct;
    let last = rk.len() - 1;
    for (sb, &kb) in s.iter_mut().zip(rk[last].iter()) {
        *sb ^= kb;
    }
    for round in (1..rk.len()).rev() {
        // InvShiftRows.
        s = [
            s[0], s[13], s[10], s[7], s[4], s[1], s[14], s[11], s[8], s[5], s[2], s[15], s[12],
            s[9], s[6], s[3],
        ];
        for sb in s.iter_mut() {
            *sb = inv_sbox(*sb);
        }
        for (sb, &kb) in s.iter_mut().zip(rk[round - 1].iter()) {
            *sb ^= kb;
        }
        if round > 1 {
            for c in 0..4 {
                let (a0, a1, a2, a3) = (s[4 * c], s[4 * c + 1], s[4 * c + 2], s[4 * c + 3]);
                s[4 * c] =
                    gf_mul(a0, 0x0E) ^ gf_mul(a1, 0x0B) ^ gf_mul(a2, 0x0D) ^ gf_mul(a3, 0x09);
                s[4 * c + 1] =
                    gf_mul(a0, 0x09) ^ gf_mul(a1, 0x0E) ^ gf_mul(a2, 0x0B) ^ gf_mul(a3, 0x0D);
                s[4 * c + 2] =
                    gf_mul(a0, 0x0D) ^ gf_mul(a1, 0x09) ^ gf_mul(a2, 0x0E) ^ gf_mul(a3, 0x0B);
                s[4 * c + 3] =
                    gf_mul(a0, 0x0B) ^ gf_mul(a1, 0x0D) ^ gf_mul(a2, 0x09) ^ gf_mul(a3, 0x0E);
            }
        }
    }
    s
}

/// Behavior behind the AES window.
#[derive(Debug, Default)]
pub struct AesBehavior {
    out: [u32; 4],
    output_ready: bool,
    data_out_irq: bool,
    /// CBC chaining value (encrypt: last ciphertext; decrypt: last input).
    chain: [u8; 16],
    chain_valid: bool,
}

impl Behavior for AesBehavior {
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
                ctrl |= CTRL_INPUT_READY; // Instant completion: always ready.
                ctrl
            }
            0x60..=0x6C => {
                let i = 3 - ((offset - DATA_OFF) / 4) as usize;
                self.out[i]
            }
            0x8C => {
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
            // Plain config latch (TivaWare writes the whole word); any
            // config change invalidates CBC chaining + pending output.
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            self.chain_valid = false;
            self.output_ready = false;
            self.data_out_irq = false;
            return word;
        }
        if offset == 0x8C {
            return view.get(offset); // `IRQSTATUS` clears via disable, not W1C.
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        // Data-In completes on the last lane of `DATA_IN_0` (TivaWare
        // writes 3,2,1,0); ECB/CBC only, else inert.
        if offset == DATA_OFF && shift == 24 {
            let ctrl = view.get(CTRL_OFF);
            let mode_cbc = ctrl & CTRL_MODE != 0;
            let advanced = ctrl & 0x000F_FFC0 & !CTRL_MODE != 0;
            if !advanced {
                let ks = (ctrl >> CTRL_KEY_SIZE_SHIFT) & CTRL_KEY_SIZE_MASK;
                let nk = match ks {
                    0x01 => 4,
                    0x02 => 6,
                    0x03 => 8,
                    _ => 0,
                };
                if nk > 0 {
                    let mut key = [0u8; 32];
                    for i in 0..nk {
                        let w = view.get(KEY1_WORD_OFF[i]);
                        key[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
                    }
                    let rk = expand_key(&key[..4 * nk]);
                    let mut block = [0u8; 16];
                    for i in 0..4 {
                        // Message word `i` lives in `DATA_IN_{3-i}`
                        // (TivaWare writes `src[0]` to `DATA_IN_3`).
                        let w = view.get(DATA_OFF + 12 - 4 * i as u32);
                        block[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
                    }
                    let encrypt = ctrl & CTRL_DIRECTION != 0;
                    let mut out;
                    if mode_cbc {
                        let mut iv = [0u8; 16];
                        for i in 0..4 {
                            let w = view.get(IV_OFF + 4 * i as u32);
                            iv[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
                        }
                        if !self.chain_valid {
                            self.chain = iv;
                            self.chain_valid = true;
                        }
                        if encrypt {
                            for (bb, &cb) in block.iter_mut().zip(self.chain.iter()) {
                                *bb ^= cb;
                            }
                            out = encrypt_block(&block, &rk);
                            self.chain = out;
                        } else {
                            let ct = block;
                            out = decrypt_block(&block, &rk);
                            for (ob, &cb) in out.iter_mut().zip(self.chain.iter()) {
                                *ob ^= cb;
                            }
                            self.chain = ct;
                        }
                    } else if encrypt {
                        out = encrypt_block(&block, &rk);
                    } else {
                        out = decrypt_block(&block, &rk);
                    }
                    for (i, chunk) in out.chunks(4).enumerate() {
                        self.out[i] = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                    }
                    self.output_ready = true;
                    self.data_out_irq = true;
                }
            }
        }
        word
    }
}

/// A Tiva C AES engine (ECB/CBC 128/192/256; other modes latch).
pub struct Aes {
    inner: RegisterFile<AesBehavior>,
}

impl Aes {
    /// A new AES engine on `D`'s tables (`"AES"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"AES"`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("AES").expect("generated tables contain AES");
        Self {
            inner: RegisterFile::new(def, AesBehavior::default()),
        }
    }
}

impl Peripheral for Aes {
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

impl InterruptSource for Aes {
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

    const AES0_BASE: u32 = 0x4403_6000;
    const CTRL_ENC_ECB_128: u32 = CTRL_DIRECTION | 0x08;
    const CTRL_DEC_ECB_128: u32 = 0x08;
    const CTRL_ENC_CBC_128: u32 = CTRL_DIRECTION | CTRL_MODE | 0x08;

    fn new_aes() -> Aes {
        Aes::for_device::<Tm4c129xnczad>()
    }

    fn read_word(aes: &mut Aes, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(aes.read(AES0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(aes: &mut Aes, offset: u32, value: u32) {
        for lane in 0..4 {
            aes.write(
                AES0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Write key (LSW-first words) + IV + config.
    fn setup(aes: &mut Aes, key_le: &[u32], iv_le: Option<[u32; 4]>, ctrl: u32) {
        for (i, &w) in key_le.iter().enumerate() {
            write_word(aes, KEY1_WORD_OFF[i], w);
        }
        if let Some(iv) = iv_le {
            for (i, &w) in iv.iter().enumerate() {
                write_word(aes, IV_OFF + 4 * i as u32, w);
            }
        }
        write_word(aes, CTRL_OFF, ctrl);
    }

    /// Write one block TivaWare-order (`src[0]` → `DATA_IN_3`) and read
    /// the result back the same way.
    fn block(aes: &mut Aes, src_le: [u32; 4]) -> [u32; 4] {
        for (i, &w) in src_le.iter().enumerate() {
            write_word(aes, DATA_OFF + 4 * (3 - i) as u32, w);
        }
        core::array::from_fn(|i| read_word(aes, DATA_OFF + 4 * (3 - i) as u32))
    }

    /// FIPS-197 Appendix B: AES-128 ECB encrypt.
    /// Key `2b7e1516 28aed2a6 abf71588 09cf4f3c`, PT `3243f6a8 885a308d
    /// 313198a2 e0370734` → CT `3925841d 02dc09fb dc118597 196a0b32`
    /// (words LE, LSW first per the TivaWare array convention).
    #[test]
    fn fips197_ecb128_encrypt() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &[0x1615_7e2b, 0xa6d2_ae28, 0x8815_f7ab, 0x3c4f_cf09],
            None,
            CTRL_ENC_ECB_128,
        );
        let ct = block(
            &mut aes,
            [0xa8f6_4332, 0x8d30_5a88, 0xa298_3131, 0x3407_37e0],
        );
        assert_eq!(ct, [0x1d84_2539, 0xfb09_dc02, 0x9785_11dc, 0x320b_6a19]);
        assert_eq!(
            read_word(&mut aes, CTRL_OFF) & CTRL_OUTPUT_READY,
            CTRL_OUTPUT_READY
        );
        assert_eq!(read_word(&mut aes, 0x8C) & IRQ_DATA_OUT, IRQ_DATA_OUT);
    }

    /// FIPS-197 Appendix B decrypt (reverse direction bit).
    #[test]
    fn fips197_ecb128_decrypt() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &[0x1615_7e2b, 0xa6d2_ae28, 0x8815_f7ab, 0x3c4f_cf09],
            None,
            CTRL_DEC_ECB_128,
        );
        let pt = block(
            &mut aes,
            [0x1d84_2539, 0xfb09_dc02, 0x9785_11dc, 0x320b_6a19],
        );
        assert_eq!(pt, [0xa8f6_4332, 0x8d30_5a88, 0xa298_3131, 0x3407_37e0]);
    }

    /// SP 800-38A F.2.1 CBC-128 encrypt (key/IV as above, PT block 1
    /// `6bc1bee2 2e409f96 e93d7e11 7393172a` → CT `7649abac 8119b246
    /// cee98e9b 12e9197d`).
    #[test]
    fn sp80038a_cbc128() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &[0x1615_7e2b, 0xa6d2_ae28, 0x8815_f7ab, 0x3c4f_cf09],
            Some([0x0302_0100, 0x0706_0504, 0x0b0a_0908, 0x0f0e_0d0c]),
            CTRL_ENC_CBC_128,
        );
        let ct = block(
            &mut aes,
            [0xe2be_c16b, 0x969f_402e, 0x117e_3de9, 0x2a17_9373],
        );
        assert_eq!(ct, [0xacab_4976, 0x46b2_1981, 0x9b8e_e9ce, 0x7d19_e912]);
    }

    /// FIPS-197 Appendix C.3: AES-256 ECB encrypt (key `00..1f`,
    /// PT `00112233…ccddeeff` → CT `8ea2b7ca…496089`).
    #[test]
    fn fips197_ecb256_encrypt() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &[
                0x0302_0100,
                0x0706_0504,
                0x0b0a_0908,
                0x0f0e_0d0c,
                0x1312_1110,
                0x1716_1514,
                0x1b1a_1918,
                0x1f1e_1d1c,
            ],
            None,
            CTRL_DIRECTION | 0x18,
        );
        let ct = block(
            &mut aes,
            [0x3322_1100, 0x7766_5544, 0xbbaa_9988, 0xffee_ddcc],
        );
        assert_eq!(ct, [0xcab7_a28e, 0xbf45_6751, 0x9049_fcea, 0x8960_494b]);
    }

    /// `IRQENABLE` gates the line; advanced modes stay inert.
    #[test]
    fn irq_gating_and_deferred_modes() {
        let mut aes = new_aes();
        write_word(&mut aes, IRQENABLE_OFF, IRQ_DATA_OUT);
        setup(
            &mut aes,
            &[0x1615_7e2b, 0xa6d2_ae28, 0x8815_f7ab, 0x3c4f_cf09],
            None,
            CTRL_ENC_ECB_128,
        );
        block(
            &mut aes,
            [0xa8f6_4332, 0x8d30_5a88, 0xa298_3131, 0x3407_37e0],
        );
        assert_eq!(aes.pending(), 1);
        // CTR mode latches without acting.
        let mut aes2 = new_aes();
        write_word(&mut aes2, CTRL_OFF, CTRL_DIRECTION | 0x08 | (1 << 6));
        block(
            &mut aes2,
            [0xa8f6_4332, 0x8d30_5a88, 0xa298_3131, 0x3407_37e0],
        );
        assert_eq!(read_word(&mut aes2, 0x8C) & IRQ_DATA_OUT, 0);
    }
}
