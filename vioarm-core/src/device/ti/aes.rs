//! TI Tiva C AES engine (`AES`, `0x4403_6000`).
//!
//! Register map and `CTRL`/`IRQSTATUS` semantics follow the TI
//! `hw_aes.h` + `aes.c` + `aes.h` oracle (BSD, fetched at runtime and
//! never vendored): `KEY1_0-7` (LSW first; 4/6/8 words for 128/192/256),
//! `KEY2_0-7` (same interleave, XTS second key), `IV_IN_0-3`, `CTRL` as
//! a plain config word (`DIRECTION` 1 = encrypt, `MODE` field under mask
//! `0x2007FE60` per the driverlib `AES_CFG_*` values: `ECB 0x0`, `CBC
//! 0x20`, `CTR 0x40` with `CTR_WIDTH` 32/64/96/128 in bits 7-8, `ICM
//! 0x200`, `CFB 0x400`, XTS `TWEAKJL 0x800` / `K2IJL 0x1000` / `K2ILJ0
//! 0x1800`, `F8 0x2000`, `F9 0x20004000`, `CBCMAC 0x20008000`, `GCM
//! 0x20030040`, `CCM 0x20040040` with `CCM_L_*` in bits 19-21 off
//! `IV_IN` byte 0 and `CCM_M_*` in bits 22-24),
//! `C_LENGTH_0/1` + `AUTH_length` partitioning auth/data streams,
//! `DATA_IN_3-0` carrying words 0-3 (reversed: `src[0]` → `DATA_IN_3`),
//! result read back from the same regs, `TAG_OUT_0-3` carrying the tag
//! as LE words (byte 0 first, same convention as data),
//! `OUTPUT_READY`/`INPUT_READY` synthesis, `IRQSTATUS.DATA_OUT` +
//! `IRQENABLE` gating.
//!
//! Modeled: AES-128/192/256 ECB + CBC + CTR encrypt/decrypt in pure
//! software (S-box computed at runtime — no transcribed tables),
//! ICM (full-width integer counter: the CTR path with the whole block
//! as span), full-block CFB with cross-completion chaining, XTS
//! (single continuous tweak chain over all three load variants;
//! 128/256-bit keys, tweak from `IV_IN`, unit index 0),
//! GCM/GMAC (12-byte nonces: `J0 = nonce || 1` from `IV_IN`; GHASH
//! via the NIST-proven STM32 primitives), raw CBC-MAC, and CCM (SP
//! 800-38C/RFC 3610: `B0`/AAD-format/CTR/CBC-MAC state machine over
//! `L` = 2-8, `M` = 4-16), instant completion. Verified against
//! FIPS-197 Appendix B/C (`B` ECB-128, `C.1` ECB-256), SP 800-38A
//! CBC-128/CTR-128, NIST SP 800-38D GCM cases, RFC 3610 CCM vectors,
//! and Python-`cryptography`-oracle vectors. Unlike GCM, the CCM MAC
//! covers PLAINTEXT in both directions (RFC 3610 §2.2/2.5).
//! Deliberate scope: F8/F9/DMA/`SAVE_CONTEXT` latch without acting
//! (F8/F9 are permanently latched: no TI-published construction exists —
//! the datasheet has no AES-engine chapter and TivaWare documents only
//! mode names + key slots — and no oracle is obtainable: no TivaWare
//! examples or vectors, the UMTS f8/f9 specs (3GPP TS 35.201, verified)
//! are KASUMI-based and do not apply, the LTE single-key shapes do not
//! match the engine's KEY1/KEY2/KEY3 slots, and no supported firmware
//! uses these cellular-protocol modes; wrong crypto is worse than
//! latched crypto); XTS-192 (IEEE 1619 has no 192-bit variant); XTS
//! ciphertext-stealing tails (unreachable: completions are always whole
//! 16-byte words off the `DATA_IN` lane path) and continuation-call
//! tweak reloads (each config starts a fresh chain); non-12-byte GCM
//! nonces (engine GHASH-derives `J0`;
//! unmodeled); tag-mismatch signaling (the engine exposes none —
//! firmware compares); CBCMAC `DATA_OUT` (tag-only mode, regs stay
//! stale). Malformed CCM configs stay inert.
//! Streaming: `DATA_IN` lane writes feed a byte FIFO drained in
//! order, so firmware chunking never misaligns the 16-byte crypto
//! blocks (a mid-stream AAD/data straddle settles per data block);
//! `DATA_OUT` publishes completed data blocks only.

use crate::device::stm32::aes::{ghash_acc, inc32};
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
const CLEN_LO_OFF: u32 = 0x54;
const CLEN_HI_OFF: u32 = 0x58;
const AUTH_LEN_OFF: u32 = 0x5C;
const DATA_OFF: u32 = 0x60;
const TAG_OFF: u32 = 0x70;
const IRQSTATUS_OFF: u32 = 0x8C;
const IRQENABLE_OFF: u32 = 0x90;

/// `CTRL` bits.
const CTRL_OUTPUT_READY: u32 = 1 << 0;
const CTRL_INPUT_READY: u32 = 1 << 1;
const CTRL_DIRECTION: u32 = 1 << 2;
const CTRL_KEY_SIZE_SHIFT: u32 = 3;
const CTRL_KEY_SIZE_MASK: u32 = 0x03;
/// `CTRL` mode field (TI driverlib `AES_CFG_MODE_M` + values).
const CTRL_MODE_M: u32 = 0x2007_FE60;
const CTRL_MODE_ECB: u32 = 0x00;
const CTRL_MODE_CBC: u32 = 0x20;
const CTRL_MODE_CTR: u32 = 0x40;
const CTRL_CTR_WIDTH_SHIFT: u32 = 7;
const CTRL_MODE_ICM: u32 = 0x200;
const CTRL_MODE_CFB: u32 = 0x400;
const CTRL_MODE_XTS_TWEAKJL: u32 = 0x800;
const CTRL_MODE_XTS_K2IJL: u32 = 0x1000;
const CTRL_MODE_XTS_K2ILJ0: u32 = 0x1800;
const CTRL_MODE_CBCMAC: u32 = 0x2000_8000;
/// `KEY2` word `j` offsets (XTS second key; same interleave as `KEY1`).
const KEY2_WORD_OFF: [u32; 8] = [0x18, 0x1C, 0x10, 0x14, 0x08, 0x0C, 0x00, 0x04];
const CTRL_MODE_GCM: u32 = 0x2003_0040;
const CTRL_MODE_CCM: u32 = 0x2004_0040;
const CTRL_CCM_M_SHIFT: u32 = 22;
const CTRL_CCM_M_MASK: u32 = 0x07;
/// `IRQSTATUS` bits.
const IRQ_DATA_OUT: u32 = 1 << 2;

/// Big-endian increment over the low `n` bytes (CTR width span).
fn incr_be(c: &mut [u8; 16], n: usize) {
    for b in c[16 - n..16].iter_mut().rev() {
        let (v, carry) = b.overflowing_add(1);
        *b = v;
        if !carry {
            break;
        }
    }
}

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
    /// CTR running counter + latched flag (shared with ICM: same
    /// engine primitive, full-width span).
    ctr: [u8; 16],
    ctr_valid: bool,
    /// CFB shift register + latched flag (ciphertext feeds back).
    cfb_sr: [u8; 16],
    cfb_valid: bool,
    /// XTS running tweak + latched flag (fresh chain per config).
    xts_tweak: [u8; 16],
    xts_valid: bool,
    /// GCM stream state (`C_LENGTH`/`AUTH_length` partition the input).
    g_latched: bool,
    g_h: [u8; 16],
    g_y: [u8; 16],
    g_ctr: [u8; 16],
    g_ek_j0: [u8; 16],
    g_aad_left: u64,
    g_data_left: u64,
    g_aad_len: u64,
    g_data_len: u64,
    g_tag: [u8; 16],
    g_tag_valid: bool,
    /// CBC-MAC stream state (zero-IV chain over `C_LENGTH` bytes).
    m_latched: bool,
    m_chain: [u8; 16],
    m_left: u64,
    m_tag: [u8; 16],
    m_tag_valid: bool,
    /// CCM stream state (SP 800-38C: `B0`/AAD-format/CTR/CBC-MAC).
    c_latched: bool,
    c_ok: bool,
    c_l: u32,
    c_m: usize,
    c_nonce: [u8; 13],
    c_nonce_len: usize,
    c_ctr: [u8; 16],
    c_mac: [u8; 16],
    c_tag: [u8; 16],
    c_tag_valid: bool,
    c_aad_left: u64,
    c_data_left: u64,
    /// Input byte FIFO: every `DATA_IN` lane write appends (TivaWare
    /// order is reconstructed by completion time); completions and
    /// `TAG_OUT` reads drain it in order, so firmware chunking never
    /// misaligns the 16-byte crypto blocks.
    in_fifo: Vec<u8>,
    /// Staging buffer for a partial crypto block (`st_aad` selects the
    /// AAD vs data stream); full blocks settle immediately, tails at
    /// stream end (`TAG_OUT` read).
    st_buf: [u8; 16],
    st_len: usize,
    st_aad: bool,
}

impl AesBehavior {
    /// Expanded round keys from `KEY1`, or `None` for a bad size field.
    fn round_keys(view: &Registers<&mut [u32]>) -> Option<Vec<[u8; 16]>> {
        let ks = (view.get(CTRL_OFF) >> CTRL_KEY_SIZE_SHIFT) & CTRL_KEY_SIZE_MASK;
        let nk = match ks {
            0x01 => 4,
            0x02 => 6,
            0x03 => 8,
            _ => return None,
        };
        let mut key = [0u8; 32];
        for i in 0..nk {
            let w = view.get(KEY1_WORD_OFF[i]);
            key[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
        }
        Some(expand_key(&key[..4 * nk]))
    }

    /// Expanded KEY2 round keys (XTS second key, same size field as
    /// KEY1; 128/256 only — IEEE 1619 has no 192-bit variant).
    fn key2_round_keys(view: &Registers<&mut [u32]>) -> Option<Vec<[u8; 16]>> {
        let ks = (view.get(CTRL_OFF) >> CTRL_KEY_SIZE_SHIFT) & CTRL_KEY_SIZE_MASK;
        let nk = match ks {
            0x01 => 4,
            0x03 => 8,
            _ => return None,
        };
        let mut key = [0u8; 32];
        for i in 0..nk {
            let w = view.get(KEY2_WORD_OFF[i]);
            key[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
        }
        Some(expand_key(&key[..4 * nk]))
    }

    /// GF(2^128) multiply-by-alpha (IEEE 1619 tweak update, polynomial
    /// x^128+x^7+x^2+x+1; tweak bytes little-endian, reduction into byte
    /// 0). Pinned by the Python-`cryptography` XTS oracle — the
    /// big-endian mirror fails block 2 while block 1 matches.
    fn gf128_mul_alpha(t: &mut [u8; 16]) {
        let mut carry = 0u8;
        for b in t.iter_mut() {
            let next = *b >> 7;
            *b = (*b << 1) | carry;
            carry = next;
        }
        if carry != 0 {
            t[0] ^= 0x87;
        }
    }

    /// One input block, TivaWare order (`DATA_IN_3` holds bytes 0-3).
    fn input_block(view: &Registers<&mut [u32]>) -> [u8; 16] {
        let mut block = [0u8; 16];
        for i in 0..4 {
            let w = view.get(DATA_OFF + 12 - 4 * i as u32);
            block[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
        }
        block
    }

    /// `IV_IN` as bytes (LE words, `IV_IN_0` first).
    fn iv_bytes(view: &Registers<&mut [u32]>) -> [u8; 16] {
        let mut iv = [0u8; 16];
        for i in 0..4 {
            let w = view.get(IV_OFF + 4 * i as u32);
            iv[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
        }
        iv
    }

    /// Publish a result block (LE words) + completion flags.
    fn publish(&mut self, out: [u8; 16]) {
        for (i, chunk) in out.chunks(4).enumerate() {
            self.out[i] = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        }
        self.output_ready = true;
        self.data_out_irq = true;
    }

    /// CTR block: keystream from the running counter (init from `IV`
    /// on first use), incrementing the width-selected span.
    /// Direction-independent.
    fn ctr_block(
        &mut self,
        rk: &[[u8; 16]],
        width: u32,
        iv: &[u8; 16],
        input: &[u8; 16],
    ) -> [u8; 16] {
        if !self.ctr_valid {
            self.ctr = *iv;
            self.ctr_valid = true;
        }
        let mut out = [0u8; 16];
        let ks = encrypt_block(&self.ctr, rk);
        for i in 0..16 {
            out[i] = input[i] ^ ks[i];
        }
        let span = match width {
            0 => 4,
            1 => 8,
            2 => 12,
            _ => 16,
        };
        incr_be(&mut self.ctr, span);
        out
    }

    /// Latch GCM stream state from the length regs + `IV` (`J0`).
    fn gcm_latch(&mut self, view: &Registers<&mut [u32]>, rk: &[[u8; 16]]) {
        let zero = [0u8; 16];
        self.g_h = encrypt_block(&zero, rk);
        self.g_y = [0u8; 16];
        // `J0` rides `IV_IN` whole (12-byte nonces arrive as
        // `nonce || 0x00000001`; anything else is out of scope).
        self.g_ctr = Self::iv_bytes(view);
        self.g_ek_j0 = encrypt_block(&self.g_ctr, rk);
        let lo = u64::from(view.get(CLEN_LO_OFF));
        let hi = u64::from(view.get(CLEN_HI_OFF));
        self.g_data_len = lo | (hi << 32);
        self.g_data_left = self.g_data_len;
        self.g_aad_len = u64::from(view.get(AUTH_LEN_OFF));
        self.g_aad_left = self.g_aad_len;
        self.g_tag_valid = false;
        self.g_latched = true;
    }

    /// Feed one 16-byte chunk through the GCM stream; returns the
    /// crypt block (partial tail zero-filled).
    /// Push stream bytes through GCM: AAD accumulates into 16-byte
    /// GHASH blocks (the tail pads when `g_aad_left` exhausts), data
    /// crypts per 16-byte block with the counter (tails stay staged
    /// for the next push or the `TAG_OUT` flush).
    fn gcm_push(&mut self, rk: &[[u8; 16]], bytes: &[u8], encrypt: bool) {
        for &b in bytes {
            if self.g_aad_left > 0 {
                debug_assert!(self.st_aad || self.st_len == 0);
                self.st_aad = true;
                self.st_buf[self.st_len] = b;
                self.st_len += 1;
                self.g_aad_left -= 1;
                if self.st_len == 16 || self.g_aad_left == 0 {
                    let mut blk = [0u8; 16];
                    blk[..self.st_len].copy_from_slice(&self.st_buf[..self.st_len]);
                    ghash_acc(&mut self.g_y, &self.g_h, &blk);
                    self.st_len = 0;
                    self.st_aad = false;
                }
            } else if self.g_data_left > 0 {
                self.st_buf[self.st_len] = b;
                self.st_len += 1;
                self.g_data_left -= 1;
                if self.st_len == 16 {
                    let blk = self.crypt_data_block(rk, encrypt);
                    self.st_len = 0;
                    self.publish(blk);
                }
            }
            // Overfed bytes (past `AUTH_length` + `C_LENGTH`) are ignored.
        }
    }

    /// Crypt one staged 16-byte data block (counter first for encrypt
    /// and decrypt alike; GHASHes ciphertext on encrypt, input on
    /// decrypt).
    fn crypt_data_block(&mut self, rk: &[[u8; 16]], encrypt: bool) -> [u8; 16] {
        let mut out = [0u8; 16];
        if encrypt {
            inc32(&mut self.g_ctr);
            let ks = encrypt_block(&self.g_ctr, rk);
            for i in 0..16 {
                out[i] = self.st_buf[i] ^ ks[i];
            }
            ghash_acc(&mut self.g_y, &self.g_h, &out);
        } else {
            ghash_acc(&mut self.g_y, &self.g_h, &self.st_buf);
            inc32(&mut self.g_ctr);
            let ks = encrypt_block(&self.g_ctr, rk);
            for i in 0..16 {
                out[i] = self.st_buf[i] ^ ks[i];
            }
        }
        out
    }

    /// Finalize the GCM tag (`len(AAD) || len(data)` block + `E(J0)`).
    fn gcm_tag(&mut self) -> [u8; 16] {
        if !self.g_tag_valid {
            let mut blk = [0u8; 16];
            blk[..8].copy_from_slice(&(self.g_aad_len.wrapping_mul(8)).to_be_bytes());
            blk[8..].copy_from_slice(&(self.g_data_len.wrapping_mul(8)).to_be_bytes());
            ghash_acc(&mut self.g_y, &self.g_h, &blk);
            for i in 0..16 {
                self.g_tag[i] = self.g_y[i] ^ self.g_ek_j0[i];
            }
            self.g_tag_valid = true;
        }
        self.g_tag
    }

    /// Latch CCM stream state: `L`/`M`/nonce from `IV_IN` + `CTRL`,
    /// lengths from the length regs, `B0` into the MAC chain. Malformed
    /// configs (`L` outside 2-8, `M` = 2, payload not fitting `L`
    /// bytes) latch invalid — the stream stays inert.
    fn ccm_latch(&mut self, view: &Registers<&mut [u32]>, rk: &[[u8; 16]], ctrl: u32) {
        let iv = Self::iv_bytes(view);
        let l = (u32::from(iv[0] & 7) + 1) as usize;
        let m = 2 * (((ctrl >> CTRL_CCM_M_SHIFT) & CTRL_CCM_M_MASK) as usize + 1);
        let lo = u64::from(view.get(CLEN_LO_OFF));
        let hi = u64::from(view.get(CLEN_HI_OFF));
        let data_len = lo | (hi << 32);
        let aad_len = u64::from(view.get(AUTH_LEN_OFF));
        self.c_latched = true;
        self.c_ok = false;
        if !(2..=8).contains(&l) || m < 4 || (l < 8 && data_len >= 1u64 << (8 * l)) {
            return;
        }
        let nonce_len = 15 - l;
        self.c_l = l as u32;
        self.c_m = m;
        self.c_nonce[..nonce_len].copy_from_slice(&iv[1..1 + nonce_len]);
        self.c_nonce_len = nonce_len;
        self.c_aad_left = aad_len;
        self.c_data_left = data_len;
        // Running counter block: flags (`L-1`) + nonce, BE count.
        self.c_ctr[0] = (l - 1) as u8;
        self.c_ctr[1..1 + nonce_len].copy_from_slice(&self.c_nonce[..nonce_len]);
        // `B0`: flags (Adata/`M'`/`L'`) + nonce + length, then `X1`.
        let mut b0 = [0u8; 16];
        b0[0] = (if aad_len > 0 { 64 } else { 0 }) | (((m - 2) / 2) as u8) << 3 | (l - 1) as u8;
        b0[1..1 + nonce_len].copy_from_slice(&self.c_nonce[..nonce_len]);
        let q = data_len.to_be_bytes();
        b0[16 - l..16].copy_from_slice(&q[8 - l..8]);
        self.c_mac = encrypt_block(&b0, rk);
        // AAD length prefix stages first (SP 800-38C §2.2.1).
        self.st_len = 0;
        self.st_aad = true;
        if aad_len > 0 {
            if aad_len < 65280 {
                let len = (aad_len as u16).to_be_bytes();
                self.st_buf[0..2].copy_from_slice(&len);
                self.st_len = 2;
            } else if aad_len < 1 << 32 {
                self.st_buf[0..2].copy_from_slice(&[0xFF, 0xFE]);
                self.st_buf[2..6].copy_from_slice(&(aad_len as u32).to_be_bytes());
                self.st_len = 6;
            } else {
                self.st_buf[0..2].copy_from_slice(&[0xFF, 0xFF]);
                self.st_buf[2..10].copy_from_slice(&aad_len.to_be_bytes());
                self.st_len = 10;
            }
        }
        self.c_tag_valid = false;
        self.c_ok = true;
    }

    /// Push stream bytes through CCM: AAD accumulates into 16-byte
    /// MAC blocks (tail pads when the AAD exhausts), payload crypts
    /// per 16-byte block (counter from 1) while MACing ciphertext on
    /// encrypt, input on decrypt.
    fn ccm_push(&mut self, rk: &[[u8; 16]], bytes: &[u8], encrypt: bool) {
        for &b in bytes {
            if !self.c_ok {
                return;
            }
            if self.c_aad_left > 0 {
                self.st_buf[self.st_len] = b;
                self.st_len += 1;
                self.c_aad_left -= 1;
                if self.st_len == 16 || self.c_aad_left == 0 {
                    let mut blk = [0u8; 16];
                    blk[..self.st_len].copy_from_slice(&self.st_buf[..self.st_len]);
                    for (dst, &m) in blk.iter_mut().zip(self.c_mac.iter()) {
                        *dst ^= m;
                    }
                    self.c_mac = encrypt_block(&blk, rk);
                    self.st_len = 0;
                    self.st_aad = false;
                }
            } else if self.c_data_left > 0 {
                self.st_buf[self.st_len] = b;
                self.st_len += 1;
                self.c_data_left -= 1;
                if self.st_len == 16 {
                    let blk = self.crypt_ccm_block(rk, encrypt);
                    self.st_len = 0;
                    self.publish(blk);
                }
            }
            // Overfed bytes are ignored.
        }
    }

    /// Crypt one staged 16-byte payload block: counter first (from 1),
    /// MAC over ciphertext on encrypt, over input on decrypt.
    /// Crypt one staged 16-byte payload block: counter first (from 1).
    /// Unlike GCM, the CCM MAC covers PLAINTEXT in both directions
    /// (RFC 3610 §2.2/2.5: the receiver decrypts, then verifies).
    fn crypt_ccm_block(&mut self, rk: &[[u8; 16]], encrypt: bool) -> [u8; 16] {
        for i in (16 - self.c_l as usize..16).rev() {
            let (v, carry) = self.c_ctr[i].overflowing_add(1);
            self.c_ctr[i] = v;
            if !carry {
                break;
            }
        }
        let ks = encrypt_block(&self.c_ctr, rk);
        let mut out = [0u8; 16];
        for i in 0..16 {
            out[i] = self.st_buf[i] ^ ks[i];
        }
        // MAC input is plaintext: staged input on encrypt, recovered
        // output on decrypt.
        let mut blk = [0u8; 16];
        if encrypt {
            for (dst, (&a, &b)) in blk
                .iter_mut()
                .zip(self.st_buf.iter().zip(self.c_mac.iter()))
            {
                *dst = a ^ b;
            }
        } else {
            for i in 0..16 {
                blk[i] = out[i] ^ self.c_mac[i];
            }
        }
        self.c_mac = encrypt_block(&blk, rk);
        out
    }

    /// Finalize the CCM tag: `MSB_M(MAC)` ^ `MSB_M(E(S0))`.
    fn ccm_tag(&mut self, rk: &[[u8; 16]]) -> [u8; 16] {
        if !self.c_tag_valid {
            let mut s0 = [0u8; 16];
            s0[0] = (self.c_l - 1) as u8;
            s0[1..1 + self.c_nonce_len].copy_from_slice(&self.c_nonce[..self.c_nonce_len]);
            let e = encrypt_block(&s0, rk);
            for (dst, (&m, &x)) in self.c_tag.iter_mut().zip(self.c_mac.iter().zip(e.iter())) {
                *dst = m ^ x;
            }
            for dst in self.c_tag.iter_mut().skip(self.c_m) {
                *dst = 0;
            }
            self.c_tag_valid = true;
        }
        self.c_tag
    }

    fn cbcmac_latch(&mut self, view: &Registers<&mut [u32]>) {
        let lo = u64::from(view.get(CLEN_LO_OFF));
        let hi = u64::from(view.get(CLEN_HI_OFF));
        self.m_left = lo | (hi << 32);
        self.m_chain = [0u8; 16];
        self.m_tag_valid = false;
        self.m_latched = true;
    }

    /// Feed one 16-byte chunk through the CBC-MAC chain (zero-padded
    /// tail per SP 800-38B §5.3).
    /// Push stream bytes through the CBC-MAC chain (zero-padded
    /// tail per SP 800-38B §5.3, flushed at `TAG_OUT`).
    fn cbcmac_push(&mut self, rk: &[[u8; 16]], bytes: &[u8]) {
        for &b in bytes {
            if self.m_left == 0 {
                break;
            }
            self.st_buf[self.st_len] = b;
            self.st_len += 1;
            self.m_left -= 1;
            if self.st_len == 16 {
                self.cbcmac_block(rk);
                self.st_len = 0;
            }
        }
    }

    /// Chain one staged 16-byte block.
    fn cbcmac_block(&mut self, rk: &[[u8; 16]]) {
        let mut blk = [0u8; 16];
        for (dst, (&a, &b)) in blk
            .iter_mut()
            .zip(self.st_buf.iter().zip(self.m_chain.iter()))
        {
            *dst = a ^ b;
        }
        self.m_chain = encrypt_block(&blk, rk);
        if self.m_left == 0 {
            self.m_tag = self.m_chain;
            self.m_tag_valid = true;
        }
    }
    /// Read the authentication tag, finalizing the stream: staged
    /// tails flush first (zero-padded); underfed streams (bytes still
    /// owed) finalize over the bytes fed so far.
    fn read_tag(&mut self, view: &Registers<&mut [u32]>) -> [u8; 16] {
        let ctrl = view.get(CTRL_OFF);
        match ctrl & CTRL_MODE_M {
            CTRL_MODE_GCM => {
                if !self.g_latched {
                    if let Some(rk) = Self::round_keys(view) {
                        self.gcm_latch(view, &rk);
                    } else {
                        return [0u8; 16];
                    }
                }
                let fifo: Vec<u8> = core::mem::take(&mut self.in_fifo);
                if !fifo.is_empty() {
                    if let Some(rk) = Self::round_keys(view) {
                        let encrypt = ctrl & CTRL_DIRECTION != 0;
                        self.gcm_push(&rk, &fifo, encrypt);
                    }
                }
                if self.st_len > 0 {
                    // Staged tail settles here (zero-padded): AAD tail
                    // GHASHes, data tail crypts (and publishes) too.
                    let n = self.st_len;
                    if self.st_aad {
                        let mut blk = [0u8; 16];
                        blk[..n].copy_from_slice(&self.st_buf[..n]);
                        ghash_acc(&mut self.g_y, &self.g_h, &blk);
                    } else if let Some(rk) = Self::round_keys(view) {
                        let encrypt = ctrl & CTRL_DIRECTION != 0;
                        inc32(&mut self.g_ctr);
                        let ks = encrypt_block(&self.g_ctr, &rk);
                        let mut out = [0u8; 16];
                        let mut blk = [0u8; 16];
                        for i in 0..n {
                            if encrypt {
                                out[i] = self.st_buf[i] ^ ks[i];
                                blk[i] = out[i];
                            } else {
                                blk[i] = self.st_buf[i];
                                out[i] = self.st_buf[i] ^ ks[i];
                            }
                        }
                        ghash_acc(&mut self.g_y, &self.g_h, &blk);
                        self.g_data_left = self.g_data_left.saturating_sub(n as u64);
                        self.publish(out);
                    }
                    self.st_len = 0;
                    self.st_aad = false;
                }
                self.gcm_tag()
            }
            CTRL_MODE_CBCMAC => {
                if !self.m_latched {
                    if Self::round_keys(view).is_some() {
                        // Latch needs no key: lengths only.
                        let lo = u64::from(view.get(CLEN_LO_OFF));
                        let hi = u64::from(view.get(CLEN_HI_OFF));
                        self.m_left = lo | (hi << 32);
                        self.m_chain = [0u8; 16];
                        self.m_tag_valid = false;
                        self.m_latched = true;
                    } else {
                        return [0u8; 16];
                    }
                }
                let fifo: Vec<u8> = core::mem::take(&mut self.in_fifo);
                if !fifo.is_empty() {
                    if let Some(rk) = Self::round_keys(view) {
                        self.cbcmac_push(&rk, &fifo);
                    }
                }
                if self.st_len > 0 {
                    if let Some(rk) = Self::round_keys(view) {
                        for i in self.st_len..16 {
                            self.st_buf[i] = 0;
                        }
                        self.cbcmac_block(&rk);
                        self.st_len = 0;
                    }
                }
                if self.m_tag_valid {
                    self.m_tag
                } else {
                    self.m_chain
                }
            }
            CTRL_MODE_CCM => {
                if !self.c_latched {
                    if let Some(rk) = Self::round_keys(view) {
                        self.ccm_latch(view, &rk, ctrl);
                    }
                }
                if !self.c_ok {
                    return [0u8; 16];
                }
                let fifo: Vec<u8> = core::mem::take(&mut self.in_fifo);
                if !fifo.is_empty() {
                    if let Some(rk) = Self::round_keys(view) {
                        let encrypt = ctrl & CTRL_DIRECTION != 0;
                        self.ccm_push(&rk, &fifo, encrypt);
                    }
                }
                if self.st_len > 0 {
                    // Staged data tail (AAD tails flush inline when the
                    // AAD exhausts): pad, MAC, crypt + publish.
                    if let Some(rk) = Self::round_keys(view) {
                        let encrypt = ctrl & CTRL_DIRECTION != 0;
                        let n = self.st_len;
                        for i in (16 - self.c_l as usize..16).rev() {
                            let (v, carry) = self.c_ctr[i].overflowing_add(1);
                            self.c_ctr[i] = v;
                            if !carry {
                                break;
                            }
                        }
                        let ks = encrypt_block(&self.c_ctr, &rk);
                        let mut out = [0u8; 16];
                        let mut blk = [0u8; 16];
                        // MAC input is plaintext (RFC 3610 §2.2).
                        for i in 0..n {
                            out[i] = self.st_buf[i] ^ ks[i];
                            if encrypt {
                                blk[i] = self.st_buf[i];
                            } else {
                                blk[i] = out[i];
                            }
                        }
                        for (dst, &m) in blk.iter_mut().zip(self.c_mac.iter()) {
                            *dst ^= m;
                        }
                        self.c_mac = encrypt_block(&blk, &rk);
                        self.c_data_left = self.c_data_left.saturating_sub(n as u64);
                        self.st_len = 0;
                        self.publish(out);
                    }
                }
                if let Some(rk) = Self::round_keys(view) {
                    self.ccm_tag(&rk)
                } else {
                    [0u8; 16]
                }
            }
            _ => [0u8; 16],
        }
    }
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
            0x70..=0x7C => {
                let tag = self.read_tag(view);
                let i = ((offset - TAG_OFF) / 4) as usize;
                u32::from_le_bytes([tag[4 * i], tag[4 * i + 1], tag[4 * i + 2], tag[4 * i + 3]])
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
            // config change invalidates chaining/stream state + output.
            let word = view.merged(offset, val, shift);
            view.set(offset, word);
            self.chain_valid = false;
            self.output_ready = false;
            self.data_out_irq = false;
            self.ctr_valid = false;
            self.cfb_valid = false;
            self.xts_valid = false;
            self.g_latched = false;
            self.g_tag_valid = false;
            self.m_latched = false;
            self.m_tag_valid = false;
            self.c_latched = false;
            self.c_tag_valid = false;
            self.in_fifo.clear();
            self.st_len = 0;
            self.st_aad = false;
            return word;
        }
        if offset == IRQSTATUS_OFF {
            return view.get(offset); // `IRQSTATUS` clears via disable, not W1C.
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        if (DATA_OFF..=DATA_OFF + 12).contains(&offset) {
            // Every lane write appends (TivaWare order is reconstructed
            // by completion time); completions drain in order.
            self.in_fifo.push(val);
        }
        // Data-In completes on the last lane of `DATA_IN_0` (TivaWare
        // writes 3,2,1,0); unknown modes stay inert.
        if offset == DATA_OFF && shift == 24 {
            let ctrl = view.get(CTRL_OFF);
            let rk = match Self::round_keys(view) {
                Some(rk) => rk,
                None => return word,
            };
            // ECB/CBC/CTR consume one block from the regs (their only
            // framing); GCM/CBC-MAC drain the FIFO stream instead.
            // Either way the FIFO resets per completion.
            let block = Self::input_block(view);
            let encrypt = ctrl & CTRL_DIRECTION != 0;
            let fifo: Vec<u8> = core::mem::take(&mut self.in_fifo);
            match ctrl & CTRL_MODE_M {
                CTRL_MODE_ECB => {
                    let out = if encrypt {
                        encrypt_block(&block, &rk)
                    } else {
                        decrypt_block(&block, &rk)
                    };
                    self.publish(out);
                }
                CTRL_MODE_CBC => {
                    let iv = Self::iv_bytes(view);
                    if !self.chain_valid {
                        self.chain = iv;
                        self.chain_valid = true;
                    }
                    let out = if encrypt {
                        let mut xored = block;
                        for (bb, &cb) in xored.iter_mut().zip(self.chain.iter()) {
                            *bb ^= cb;
                        }
                        let out = encrypt_block(&xored, &rk);
                        self.chain = out;
                        out
                    } else {
                        let out = decrypt_block(&block, &rk);
                        let mut pt = out;
                        for (ob, &cb) in pt.iter_mut().zip(self.chain.iter()) {
                            *ob ^= cb;
                        }
                        self.chain = block;
                        pt
                    };
                    self.publish(out);
                }
                CTRL_MODE_CTR => {
                    let width = (ctrl >> CTRL_CTR_WIDTH_SHIFT) & 0x03;
                    let iv = Self::iv_bytes(view);
                    let out = self.ctr_block(&rk, width, &iv, &block);
                    self.publish(out);
                }
                CTRL_MODE_ICM => {
                    // Integer counter: the CTR path with the whole
                    // block as span (width select ignored, per the
                    // driverlib config ASSERT).
                    let iv = Self::iv_bytes(view);
                    let out = self.ctr_block(&rk, 3, &iv, &block);
                    self.publish(out);
                }
                CTRL_MODE_CFB => {
                    let iv = Self::iv_bytes(view);
                    if !self.cfb_valid {
                        self.cfb_sr = iv;
                        self.cfb_valid = true;
                    }
                    let ks = encrypt_block(&self.cfb_sr, &rk);
                    let mut out = [0u8; 16];
                    for (dst, (&b, &k)) in out.iter_mut().zip(block.iter().zip(ks.iter())) {
                        *dst = b ^ k;
                    }
                    // Encrypt feeds back ciphertext, decrypt feeds back
                    // the input ciphertext.
                    self.cfb_sr = if encrypt { out } else { block };
                    self.publish(out);
                }
                CTRL_MODE_XTS_TWEAKJL | CTRL_MODE_XTS_K2IJL | CTRL_MODE_XTS_K2ILJ0 => {
                    // One continuous tweak chain over every load variant
                    // (continuation-call context reloads unmodeled: each
                    // config starts fresh). 192-bit keys have no XTS
                    // variant and stay inert.
                    if let Some(rk2) = Self::key2_round_keys(view) {
                        if !self.xts_valid {
                            self.xts_tweak = encrypt_block(&Self::iv_bytes(view), &rk2);
                            self.xts_valid = true;
                        }
                        let mut xored = [0u8; 16];
                        for (dst, (&b, &t)) in xored
                            .iter_mut()
                            .zip(block.iter().zip(self.xts_tweak.iter()))
                        {
                            *dst = b ^ t;
                        }
                        let mut mid = if encrypt {
                            encrypt_block(&xored, &rk)
                        } else {
                            decrypt_block(&xored, &rk)
                        };
                        for (dst, &t) in mid.iter_mut().zip(self.xts_tweak.iter()) {
                            *dst ^= t;
                        }
                        Self::gf128_mul_alpha(&mut self.xts_tweak);
                        self.publish(mid);
                    }
                }
                CTRL_MODE_GCM => {
                    if !self.g_latched {
                        self.gcm_latch(view, &rk);
                    }
                    self.gcm_push(&rk, &fifo, encrypt);
                }
                CTRL_MODE_CCM => {
                    if !self.c_latched {
                        self.ccm_latch(view, &rk, ctrl);
                    }
                    // Completion flags rise from `publish` on real
                    // blocks (push) or the `TAG_OUT` tail flush; an
                    // invalid config stays fully inert.
                    self.ccm_push(&rk, &fifo, encrypt);
                }
                CTRL_MODE_CBCMAC => {
                    if !self.m_latched {
                        self.cbcmac_latch(view);
                    }
                    self.cbcmac_push(&rk, &fifo);
                    // Tag-only mode: `DATA_OUT` stays stale, completion
                    // flags still rise per block.
                    self.output_ready = true;
                    self.data_out_irq = true;
                }
                _ => {}
            }
        }
        word
    }
}

/// A Tiva C AES engine (ECB/CBC/CTR/ICM/CFB/XTS/GCM/CBC-MAC/CCM
/// 128/192/256 — XTS without 192; other modes latch).
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
    const CTRL_ENC_CBC_128: u32 = CTRL_DIRECTION | CTRL_MODE_CBC | 0x08;

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

    /// Write crypto + auth lengths (bytes).
    fn lengths(aes: &mut Aes, data_len: u64, aad_len: u32) {
        write_word(aes, CLEN_LO_OFF, data_len as u32);
        write_word(aes, CLEN_HI_OFF, (data_len >> 32) as u32);
        write_word(aes, AUTH_LEN_OFF, aad_len);
    }

    /// Read the authentication tag (`TAG_OUT_0-3`, LE words).
    fn tag(aes: &mut Aes) -> [u32; 4] {
        core::array::from_fn(|i| read_word(aes, TAG_OFF + 4 * i as u32))
    }

    /// Hex string (spaces ignored) to bytes (test vectors).
    fn unhex(s: &str) -> Vec<u8> {
        let h: Vec<u8> = s.bytes().filter(|b| *b != b' ').collect();
        h.chunks(2)
            .map(|c| u8::from_str_radix(core::str::from_utf8(c).unwrap(), 16).unwrap())
            .collect()
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
        // F8 mode latches without acting (KEY3 construction, no oracle).
        let mut aes2 = new_aes();
        write_word(&mut aes2, CTRL_OFF, CTRL_DIRECTION | 0x08 | 0x2000);
        block(
            &mut aes2,
            [0xa8f6_4332, 0x8d30_5a88, 0xa298_3131, 0x3407_37e0],
        );
        assert_eq!(read_word(&mut aes2, 0x8C) & IRQ_DATA_OUT, 0);
        // F9 likewise inert.
        let mut aes3 = new_aes();
        write_word(&mut aes3, CTRL_OFF, CTRL_DIRECTION | 0x08 | 0x2000_4000);
        block(
            &mut aes3,
            [0xa8f6_4332, 0x8d30_5a88, 0xa298_3131, 0x3407_37e0],
        );
        assert_eq!(read_word(&mut aes3, 0x8C) & IRQ_DATA_OUT, 0);
    }

    const KEY128: [u32; 4] = [0x1615_7e2b, 0xa6d2_ae28, 0x8815_f7ab, 0x3c4f_cf09];
    const CTR_IV: [u32; 4] = [0xf3f2_f1f0, 0xf7f6_f5f4, 0xfbfa_f9f8, 0xfffe_fdfc];
    const CTR_PT: [u32; 4] = [0xe2be_c16b, 0x969f_402e, 0x117e_3de9, 0x2a17_9373];
    const CTR128_CT: [u32; 4] = [0x9161_4d87, 0x26e3_20b6, 0x6468_ef1b, 0xceb6_0d99];

    /// SP 800-38A F.5.1 shape: CTR-128 encrypt (Python-oracle vector).
    #[test]
    fn ctr128_encrypt() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &KEY128,
            Some(CTR_IV),
            CTRL_DIRECTION | CTRL_MODE_CTR | 0x08,
        );
        assert_eq!(block(&mut aes, CTR_PT), CTR128_CT);
        assert_eq!(
            read_word(&mut aes, CTRL_OFF) & CTRL_OUTPUT_READY,
            CTRL_OUTPUT_READY
        );
    }

    /// CTR decrypt = encrypt; round-trips the vector above.
    #[test]
    fn ctr128_decrypt_roundtrip() {
        let mut aes = new_aes();
        setup(&mut aes, &KEY128, Some(CTR_IV), CTRL_MODE_CTR | 0x08);
        assert_eq!(block(&mut aes, CTR128_CT), CTR_PT);
    }

    /// CTR-256 with a constructed key (oracle vector `f9c1736f…`).
    #[test]
    fn ctr256_encrypt() {
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
            Some(CTR_IV),
            CTRL_DIRECTION | CTRL_MODE_CTR | 0x18,
        );
        assert_eq!(
            block(&mut aes, CTR_PT),
            [0x6f73_c1f9, 0x5d1f_d60d, 0x4598_54b3, 0x3e74_a133]
        );
    }

    const GCM0_TAG: [u32; 4] = [0xd447_6eab, 0xbd13_ec2c, 0xb267_3af5, 0xdfbd_5712];
    const GCM0_CT: [u32; 4] = [0xceda_8803, 0x92a3_b660, 0xb9c2_28f3, 0x78fe_b271];

    /// NIST SP 800-38D case 2: zero key/PT, 12 zero nonce bytes.
    #[test]
    fn gcm_zero_case() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &[0, 0, 0, 0],
            Some([0, 0, 0, 0x0100_0000]),
            CTRL_DIRECTION | CTRL_MODE_GCM | 0x08,
        );
        lengths(&mut aes, 16, 0);
        assert_eq!(block(&mut aes, [0, 0, 0, 0]), GCM0_CT);
        assert_eq!(tag(&mut aes), GCM0_TAG);
    }

    /// GCM decrypt of the zero case recovers plaintext + tag.
    #[test]
    fn gcm_zero_decrypt() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &[0, 0, 0, 0],
            Some([0, 0, 0, 0x0100_0000]),
            CTRL_MODE_GCM | 0x08,
        );
        lengths(&mut aes, 16, 0);
        assert_eq!(block(&mut aes, GCM0_CT), [0, 0, 0, 0]);
        assert_eq!(tag(&mut aes), GCM0_TAG);
    }

    const GCM4_TAG: [u32; 4] = [0xfc59_750e, 0xe7f8_7bd5, 0xff1a_4642, 0x048e_103b];

    /// SP 800-38D case-4 shape (64 B data + 20 B AAD, oracle vectors):
    /// exercises the AAD/data straddle inside one completion.
    #[test]
    fn gcm_aad_data_straddle() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &[0x92e9_fffe, 0x1c73_6586, 0x948f_ff6d, 0x0b16_b964],
            Some([0xbeba_feca, 0xaddb_cefa, 0xbafe_cade, 0x0100_0000]),
            CTRL_DIRECTION | CTRL_MODE_GCM | 0x08,
        );
        lengths(&mut aes, 64, 20);
        // AAD block 1 (16 B).
        block(
            &mut aes,
            [0x0302_0100, 0x0706_0504, 0x0b0a_0908, 0x0f0e_0d0c],
        );
        // Straddle: AAD tail (4 B) + PT head (12 B); no data block
        // completes, so `DATA_OUT` stays zero.
        let ct0 = block(
            &mut aes,
            [0x1312_1110, 0x0302_0100, 0x0706_0504, 0x0b0a_0908],
        );
        assert_eq!(ct0, [0x0000_0000, 0x0000_0000, 0x0000_0000, 0x0000_0000]);
        // Each 16 B data write completes one 16-byte data block behind
        // the 12-byte staging offset.
        assert_eq!(
            block(
                &mut aes,
                [0x0f0e_0d0c, 0x1312_1110, 0x1716_1514, 0x1b1a_1918]
            ),
            [0x9595_325b, 0xd017_1180, 0x7671_0932, 0xd1ae_5520]
        );
        assert_eq!(
            block(
                &mut aes,
                [0x1f1e_1d1c, 0x2322_2120, 0x2726_2524, 0x2b2a_2928]
            ),
            [0xd7bb_3cf0, 0x339f_dfd8, 0x4bfa_2819, 0x02b7_e2f8]
        );
        assert_eq!(
            block(
                &mut aes,
                [0x2f2e_2d2c, 0x3332_3130, 0x3736_3534, 0x3b3a_3938]
            ),
            [0xff87_f48b, 0xabbc_c750, 0x65c0_88d5, 0xfce1_7f84]
        );
        // Trailing 4 PT bytes as one word (no completion), then TAG:
        // the staged 16 bytes complete the last block first.
        write_word(&mut aes, DATA_OFF + 12, 0x3f3e_3d3c);
        assert_eq!(tag(&mut aes), GCM4_TAG);
    }

    /// GMAC: empty plaintext, 16 B AAD (oracle tag).
    #[test]
    fn gmac_empty_plaintext() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &KEY128,
            Some([0xf3f2_f1f0, 0xf7f6_f5f4, 0xfbfaf9f8, 0x0100_0000]),
            CTRL_DIRECTION | CTRL_MODE_GCM | 0x08,
        );
        lengths(&mut aes, 0, 16);
        block(&mut aes, CTR_PT);
        assert_eq!(
            tag(&mut aes),
            [0xa75d_d2fb, 0x6a66_86d3, 0x681d_f6f6, 0xe667_12e9]
        );
    }

    /// Trailing-partial data via `TAG` slurp (20 B PT, oracle vectors).
    #[test]
    fn gcm_trailing_partial() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &[0x92e9_fffe, 0x1c73_6586, 0x948f_ff6d, 0x0b16_b964],
            Some([0xbeba_feca, 0xaddb_cefa, 0xbafe_cade, 0x0100_0000]),
            CTRL_DIRECTION | CTRL_MODE_GCM | 0x08,
        );
        lengths(&mut aes, 20, 0);
        let ct = block(
            &mut aes,
            [0x0302_0100, 0x0706_0504, 0x0b0a_0908, 0x0f0e_0d0c],
        );
        assert_eq!(ct, [0x9595_325b, 0xd017_1180, 0x7671_0932, 0xd1ae_5520]);
        write_word(&mut aes, DATA_OFF + 12, 0x1312_1110);
        assert_eq!(
            tag(&mut aes),
            [0xec57_b71f, 0x3a7a_7465, 0x3a3a_d950, 0x0121_9c3f]
        );
    }

    /// Raw CBC-MAC over two blocks (oracle tag `53414774…`).
    #[test]
    fn cbcmac_two_blocks() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &KEY128,
            None,
            CTRL_DIRECTION | CTRL_MODE_CBCMAC | 0x08,
        );
        lengths(&mut aes, 32, 0);
        block(&mut aes, CTR_PT);
        block(
            &mut aes,
            [0x3322_1100, 0x7766_5544, 0xbbaa_9988, 0xffee_ddcc],
        );
        assert_eq!(
            tag(&mut aes),
            [0x7447_4153, 0x4c77_c05a, 0xd5cf_1ca5, 0x4b3a_a71a]
        );
    }

    /// `IV_IN` for CCM: byte 0 = `L-1`, then the nonce bytes.
    fn iv_ccm(nonce: &[u8], l: u32) -> [u32; 4] {
        let mut iv = [0u8; 16];
        iv[0] = (l - 1) as u8;
        iv[1..1 + nonce.len()].copy_from_slice(nonce);
        core::array::from_fn(|i| {
            u32::from_le_bytes([iv[4 * i], iv[4 * i + 1], iv[4 * i + 2], iv[4 * i + 3]])
        })
    }

    /// CCM config word for tag length `m` (128-bit key).
    fn ctrl_ccm(encrypt: bool, m: u32) -> u32 {
        (if encrypt { CTRL_DIRECTION } else { 0 })
            | CTRL_MODE_CCM
            | ((m / 2 - 1) << CTRL_CCM_M_SHIFT)
            | 0x08
    }

    /// Feed a byte stream TivaWare-order (full 16 B blocks via
    /// `block`; trailing bytes lane by lane, so no phantom zero bytes
    /// enter the FIFO).
    fn feed_stream(aes: &mut Aes, data: &[u8]) {
        let mut chunks = data.chunks_exact(16);
        for blk in &mut chunks {
            block(
                aes,
                core::array::from_fn(|i| {
                    u32::from_le_bytes([blk[4 * i], blk[4 * i + 1], blk[4 * i + 2], blk[4 * i + 3]])
                }),
            );
        }
        // Remainder byte by byte into `DATA_IN_3` first (TivaWare
        // order); single-lane writes never complete spuriously.
        let mut off = DATA_OFF + 12;
        let mut lane = 0;
        for &b in chunks.remainder() {
            aes.write(AES0_BASE + off + lane, b).unwrap();
            lane += 1;
            if lane == 4 {
                lane = 0;
                off -= 4;
            }
        }
    }

    /// Read `n <= 16` output bytes from `DATA_OUT` (TivaWare order):
    /// the window holds the last completed block only, so streaming
    /// firmware reads per block.
    fn read_stream(aes: &mut Aes, n: usize) -> Vec<u8> {
        assert!(n <= 16);
        let words = n.div_ceil(4);
        let mut out = Vec::new();
        for k in 0..words {
            let w = read_word(aes, DATA_OFF + 12 - 4 * k as u32);
            out.extend_from_slice(&w.to_le_bytes());
        }
        out.truncate(n);
        out
    }

    /// CCM case A: 12 B nonce (`L` = 3), `M` = 8, 10 B AAD, 20 B data
    /// (oracle CT + tag). `DATA_OUT` is read per completed block, the
    /// streaming firmware pattern.
    #[test]
    fn ccm_encrypt_aad_tail() {
        let mut aes = new_aes();
        let key: Vec<u32> = (0..4)
            .map(|i| u32::from_le_bytes([4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3]))
            .collect();
        let nonce: Vec<u8> = (0..12).collect();
        let aad: Vec<u8> = (100..110).collect();
        let pt: Vec<u8> = (200..220).collect();
        setup(&mut aes, &key, Some(iv_ccm(&nonce, 3)), ctrl_ccm(true, 8));
        lengths(&mut aes, 20, 10);
        feed_stream(&mut aes, &aad);
        // First data block (16 B) completes; tail (4 B) stages.
        feed_stream(&mut aes, &pt[..16]);
        assert_eq!(
            read_stream(&mut aes, 16),
            unhex("fbdc39ac 104d0a7e a1c21b33 d63f5357")
        );
        feed_stream(&mut aes, &pt[16..]);
        assert_eq!(tag(&mut aes)[..2], [0x5e31_1c1d, 0x35e6_ba4d]);
        assert_eq!(read_stream(&mut aes, 4), unhex("b48fc3e9"));
    }

    /// CCM decrypt round-trips case A (tag recomputed identically).
    #[test]
    fn ccm_decrypt_aad_tail() {
        let mut aes = new_aes();
        let key: Vec<u32> = (0..4)
            .map(|i| u32::from_le_bytes([4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3]))
            .collect();
        let nonce: Vec<u8> = (0..12).collect();
        let aad: Vec<u8> = (100..110).collect();
        let ct = unhex("fbdc39ac 104d0a7e a1c21b33 d63f5357 b48fc3e9");
        setup(&mut aes, &key, Some(iv_ccm(&nonce, 3)), ctrl_ccm(false, 8));
        lengths(&mut aes, 20, 10);
        feed_stream(&mut aes, &aad);
        feed_stream(&mut aes, &ct[..16]);
        let pt: Vec<u8> = (200..220).collect();
        assert_eq!(read_stream(&mut aes, 16), pt[..16]);
        feed_stream(&mut aes, &ct[16..]);
        assert_eq!(tag(&mut aes)[..2], [0x5e31_1c1d, 0x35e6_ba4d]);
        assert_eq!(read_stream(&mut aes, 4), pt[16..]);
    }

    /// CCM case B: 7 B nonce (`L` = 8), `M` = 16, no AAD, 32 B data.
    #[test]
    fn ccm_encrypt_no_aad() {
        let mut aes = new_aes();
        let key: Vec<u32> = (0..4)
            .map(|i| u32::from_le_bytes([4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3]))
            .collect();
        let nonce: Vec<u8> = (0..7).collect();
        let pt: Vec<u8> = (0..32).collect();
        setup(&mut aes, &key, Some(iv_ccm(&nonce, 8)), ctrl_ccm(true, 16));
        lengths(&mut aes, 32, 0);
        feed_stream(&mut aes, &pt[..16]);
        assert_eq!(
            read_stream(&mut aes, 16),
            unhex("5715b1ef 39830708 a405a5ee 98eb09b0")
        );
        feed_stream(&mut aes, &pt[16..]);
        assert_eq!(
            read_stream(&mut aes, 16),
            unhex("cf21098c 7b865325 998ba6e7 0eb59e95")
        );
        assert_eq!(
            tag(&mut aes),
            [0xe076_b643, 0xcd9d_ac56, 0xb790_a6e0, 0xefc7_679e]
        );
    }

    /// CCM case C: 13 B nonce (`L` = 2), `M` = 4, short everything.
    #[test]
    fn ccm_encrypt_short() {
        let mut aes = new_aes();
        let key: Vec<u32> = (0..4)
            .map(|i| u32::from_le_bytes([4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3]))
            .collect();
        let nonce: Vec<u8> = (0..13).collect();
        let aad: Vec<u8> = (0..8).collect();
        let pt: Vec<u8> = (0..5).collect();
        setup(&mut aes, &key, Some(iv_ccm(&nonce, 2)), ctrl_ccm(true, 4));
        lengths(&mut aes, 5, 8);
        feed_stream(&mut aes, &aad);
        feed_stream(&mut aes, &pt);
        // Single partial block: nothing out until `TAG` flushes it.
        assert_eq!(tag(&mut aes)[0], 0xf19a_b5a2);
        assert_eq!(read_stream(&mut aes, 5), unhex("1635b68b 57"));
    }

    /// RFC 3610 Packet Vector #1 (`L` = 2, `M` = 8, 8 B header,
    /// 23 B message): the authoritative end-to-end check — key, nonce,
    /// AAD, PT, CT, and tag all transcribed from the RFC text (the
    /// Python oracle reproduces the RFC packet byte-for-byte).
    #[test]
    fn ccm_rfc3610_vector1() {
        let mut aes = new_aes();
        let key = unhex("C0C1C2C3 C4C5C6C7 C8C9CACB CCCDCECF");
        let key: Vec<u32> = key
            .chunks(4)
            .map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]]))
            .collect();
        let nonce = unhex("00000003 020100A0 A1A2A3A4 A5");
        let aad = unhex("00010203 04050607");
        let pt = unhex("08090A0B 0C0D0E0F 10111213 14151617 18191A1B 1C1D1E");
        setup(&mut aes, &key, Some(iv_ccm(&nonce, 2)), ctrl_ccm(true, 8));
        lengths(&mut aes, 23, 8);
        feed_stream(&mut aes, &aad);
        feed_stream(&mut aes, &pt[..16]);
        assert_eq!(
            read_stream(&mut aes, 16),
            unhex("588c979a 61c663d2 f066d0c2 c0f98980")
        );
        feed_stream(&mut aes, &pt[16..]);
        assert_eq!(tag(&mut aes)[..2], [0x2cd1_e817, 0xe026_f9fd]);
        assert_eq!(read_stream(&mut aes, 7), unhex("6d5f6b61 dac384"));
    }

    /// Malformed CCM config (`M` = 2) stays inert.
    #[test]
    fn ccm_malformed_stays_inert() {
        let mut aes = new_aes();
        let key: Vec<u32> = (0..4)
            .map(|i| u32::from_le_bytes([4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3]))
            .collect();
        let nonce: Vec<u8> = (0..12).collect();
        setup(
            &mut aes,
            &key,
            Some(iv_ccm(&nonce, 3)),
            CTRL_DIRECTION | CTRL_MODE_CCM | 0x08,
        );
        lengths(&mut aes, 16, 0);
        block(&mut aes, [0, 0, 0, 0]);
        assert_eq!(read_word(&mut aes, 0x8C) & IRQ_DATA_OUT, 0);
        assert_eq!(tag(&mut aes), [0, 0, 0, 0]);
    }

    /// Bytes <-> LE-word blocks for oracle vectors.
    fn w16(b: &[u8]) -> [u32; 4] {
        core::array::from_fn(|i| {
            u32::from_le_bytes([b[4 * i], b[4 * i + 1], b[4 * i + 2], b[4 * i + 3]])
        })
    }

    fn w32(b: &[u8]) -> Vec<u32> {
        b.chunks(4)
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect()
    }

    fn b16(w: &[u32; 4]) -> Vec<u8> {
        w.iter().flat_map(|x| x.to_le_bytes()).collect()
    }

    /// Write `KEY2` words (LSW first, XTS second key).
    fn setup2(aes: &mut Aes, key2_le: &[u32]) {
        for (i, &w) in key2_le.iter().enumerate() {
            write_word(aes, KEY2_WORD_OFF[i], w);
        }
    }

    /// Frozen oracle vectors (`/tmp/opencode/aes_resid_vecs.json`).
    const RK128: &str = "000102 030405 060708 090a0b 0c0d0e 0f";
    const RK256: &str =
        "000102 030405 060708 090a0b 0c0d0e 0f 101112 131415 161718 191a1b 1c1d1e 1f";
    const RK2_128: &str = "101112 131415 161718 191a1b 1c1d1e 1f";
    const RK2_256: &str =
        "202122 232425 262728 292a2b 2c2d2e 2f 303132 333435 363738 393a3b 3c3d3e 3f";
    const RPT2: &str =
        "001122 334455 667788 99aa bbcc ddee ff00 112233 445566 778899 aabb ccdd eeff";

    /// ICM encrypt, 2 blocks (`624cb605…602b` oracle).
    #[test]
    fn icm_encrypt_2block() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000000 000000 000000 000000 0000 fffe"))),
            CTRL_DIRECTION | CTRL_MODE_ICM | 0x08,
        );
        let pt = unhex(RPT2);
        let c0 = b16(&block(&mut aes, w16(&pt[..16])));
        let c1 = b16(&block(&mut aes, w16(&pt[16..])));
        assert_eq!(
            [c0, c1].concat(),
            unhex("624cb605 0c1c3093 fca0ae17 fb5e9914 6a2788ea 3cfa3846 eb55b253 5d20602b")
        );
    }

    /// ICM first keystream block is ECB(IV) by construction (`ks0` oracle).
    #[test]
    fn icm_first_keystream_is_ecb_iv() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000000 000000 000000 000000 0000 fffe"))),
            CTRL_DIRECTION | CTRL_MODE_ICM | 0x08,
        );
        let pt = unhex(RPT2);
        let c0 = b16(&block(&mut aes, w16(&pt[..16])));
        let ks0 = unhex("625d9436 484956e4 743904ac 378377eb");
        assert_eq!(
            c0,
            pt[..16]
                .iter()
                .zip(ks0.iter())
                .map(|(&a, &b)| a ^ b)
                .collect::<Vec<_>>()
        );
    }

    /// ICM full-width carry: `FF…FF` + 1 wraps the whole block (`3c553d…53686`).
    #[test]
    fn icm_carry_wrap() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w16(&unhex(RK128)),
            Some([0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF]),
            CTRL_DIRECTION | CTRL_MODE_ICM | 0x08,
        );
        let pt = unhex(RPT2);
        let c0 = b16(&block(&mut aes, w16(&pt[..16])));
        let c1 = b16(&block(&mut aes, w16(&pt[16..])));
        assert_eq!(
            [c0, c1].concat(),
            unhex("3c553d01 8a52e454 ec4e0822 c28d55ec c6b01904 c3da3df5 e7d62bd9 6d153686")
        );
    }

    /// ICM decrypt round-trips; 256-bit keys work the same path.
    #[test]
    fn icm_decrypt_roundtrip_256() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w32(&unhex(RK256)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_DIRECTION | CTRL_MODE_ICM | 0x18,
        );
        let pt = unhex(RPT2);
        let c0 = block(&mut aes, w16(&pt[..16]));
        let c1 = block(&mut aes, w16(&pt[16..]));
        let mut aes2 = new_aes();
        setup(
            &mut aes2,
            &w32(&unhex(RK256)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_MODE_ICM | 0x18,
        );
        assert_eq!(b16(&block(&mut aes2, c0)), pt[..16]);
        assert_eq!(b16(&block(&mut aes2, c1)), pt[16..]);
    }

    /// CFB-128 encrypt, 2 blocks (`0a852986…0f3c` oracle).
    #[test]
    fn cfb_encrypt_2block() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_DIRECTION | CTRL_MODE_CFB | 0x08,
        );
        let pt = unhex(RPT2);
        let c0 = b16(&block(&mut aes, w16(&pt[..16])));
        let c1 = b16(&block(&mut aes, w16(&pt[16..])));
        assert_eq!(
            [c0, c1].concat(),
            unhex("0a852986 053b9632 795a3ee3 0a8e04a5 ccefb18f ab11bca6 7c621a7b 947c0f3c")
        );
    }

    /// CFB-128 decrypt recovers the oracle ciphertext; 256-bit encrypt matches.
    #[test]
    fn cfb_decrypt_and_256() {
        let ct = unhex("0a852986 053b9632 795a3ee3 0a8e04a5 ccefb18f ab11bca6 7c621a7b 947c0f3c");
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_MODE_CFB | 0x08,
        );
        assert_eq!(b16(&block(&mut aes, w16(&ct[..16]))), unhex(RPT2)[..16]);
        assert_eq!(b16(&block(&mut aes, w16(&ct[16..]))), unhex(RPT2)[16..]);
        // Chaining holds across completions (second block used the first).
        let mut aes2 = new_aes();
        setup(
            &mut aes2,
            &w32(&unhex(RK256)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_DIRECTION | CTRL_MODE_CFB | 0x18,
        );
        let pt = unhex(RPT2);
        let c0 = b16(&block(&mut aes2, w16(&pt[..16])));
        let c1 = b16(&block(&mut aes2, w16(&pt[16..])));
        assert_eq!(
            [c0, c1].concat(),
            unhex("5a7f2664 4cae17e1 78b7ff86 ce1e486d 2e05f8bb e5166fbb d52a771a 3d4fc393")
        );
    }

    /// XTS-128 encrypt, 2 blocks (`34abbd…eeef` oracle, `TWEAKJL`).
    #[test]
    fn xts_encrypt_128() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_DIRECTION | CTRL_MODE_XTS_TWEAKJL | 0x08,
        );
        setup2(&mut aes, &w16(&unhex(RK2_128)));
        let pt = unhex(RPT2);
        let c0 = b16(&block(&mut aes, w16(&pt[..16])));
        let c1 = b16(&block(&mut aes, w16(&pt[16..])));
        assert_eq!(
            [c0, c1].concat(),
            unhex("34abbd23 f8f3a6a9 04e8288b 9af9f11c 492413fc 2e8d94d6 ddda059b 0614eeef")
        );
    }

    /// XTS-128 decrypt round-trips through the tweak chain.
    #[test]
    fn xts_decrypt_roundtrip() {
        let ct = unhex("34abbd23 f8f3a6a9 04e8288b 9af9f11c 492413fc 2e8d94d6 ddda059b 0614eeef");
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_MODE_XTS_TWEAKJL | 0x08,
        );
        setup2(&mut aes, &w16(&unhex(RK2_128)));
        assert_eq!(b16(&block(&mut aes, w16(&ct[..16]))), unhex(RPT2)[..16]);
        assert_eq!(b16(&block(&mut aes, w16(&ct[16..]))), unhex(RPT2)[16..]);
    }

    /// XTS-256 via `K2IJL` (`2cee1d…444f` oracle); `K2ILJ0` decodes the
    /// same single-shot math.
    #[test]
    fn xts_256_variants() {
        let pt = unhex(RPT2);
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w32(&unhex(RK256)),
            Some(w16(&unhex("101112 131415 161718 191a1b 1c1d1e 1f"))),
            CTRL_DIRECTION | CTRL_MODE_XTS_K2IJL | 0x18,
        );
        setup2(&mut aes, &w32(&unhex(RK2_256)));
        let c0 = b16(&block(&mut aes, w16(&pt[..16])));
        let c1 = b16(&block(&mut aes, w16(&pt[16..])));
        assert_eq!(
            [c0, c1].concat(),
            unhex("2cee1d0a 794cf6ce 0527876f 9c098750 4e840552 b30bfcf3 a91211f4 2bca444f")
        );
        // `K2ILJ0` agrees block-for-block with `TWEAKJL` on one shot.
        let mut a = new_aes();
        setup(
            &mut a,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_DIRECTION | CTRL_MODE_XTS_TWEAKJL | 0x08,
        );
        setup2(&mut a, &w16(&unhex(RK2_128)));
        let r0 = block(&mut a, w16(&pt[..16]));
        let mut b = new_aes();
        setup(
            &mut b,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_DIRECTION | CTRL_MODE_XTS_K2ILJ0 | 0x08,
        );
        setup2(&mut b, &w16(&unhex(RK2_128)));
        assert_eq!(block(&mut b, w16(&pt[..16])), r0);
    }

    /// XTS-192 has no IEEE 1619 variant: fully inert.
    #[test]
    fn xts_192_inert() {
        let mut aes = new_aes();
        setup(
            &mut aes,
            &w16(&unhex(RK128)),
            Some(w16(&unhex("000102 030405 060708 090a0b 0c0d0e 0f"))),
            CTRL_DIRECTION | CTRL_MODE_XTS_TWEAKJL | 0x10,
        );
        setup2(&mut aes, &w16(&unhex(RK2_128)));
        block(&mut aes, w16(&unhex(RPT2)[..16]));
        assert_eq!(read_word(&mut aes, CTRL_OFF) & CTRL_OUTPUT_READY, 0);
    }
}
