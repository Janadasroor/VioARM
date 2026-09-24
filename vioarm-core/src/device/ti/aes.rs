//! TI Tiva C AES engine (`AES`, `0x4403_6000`).
//!
//! Register map and `CTRL`/`IRQSTATUS` semantics follow the TI
//! `hw_aes.h` + `aes.c` oracle (BSD): `KEY1_0-7` (LSW first; 4/6/8 words
//! for 128/192/256), `IV_IN_0-3`, `CTRL` as a plain config word
//! (`DIRECTION` 1 = encrypt, `MODE` field under mask `0x2007FE60`
//! per the TI driverlib `AES_CFG_*` values: `ECB 0x0`, `CBC 0x20`,
//! `CTR 0x40` with `CTR_WIDTH` 32/64/96/128 in bits 7-8, `CBCMAC
//! 0x20008000`, `GCM 0x20030040`), `C_LENGTH_0/1` + `AUTH_length`
//! partitioning GCM/CBC-MAC streams, `DATA_IN_3-0` carrying words 0-3
//! (reversed: `src[0]` → `DATA_IN_3`), result read back from the same
//! regs, `TAG_OUT_0-3` carrying the tag as LE words (byte 0 first,
//! same convention as data), `OUTPUT_READY`/`INPUT_READY`
//! synthesis, `IRQSTATUS.DATA_OUT` + `IRQENABLE` gating.
//!
//! Modeled: AES-128/192/256 ECB + CBC + CTR encrypt/decrypt in pure
//! software (S-box computed at runtime — no transcribed tables),
//! GCM/GMAC (12-byte nonces: `J0 = nonce || 1` from `IV_IN`; GHASH
//! via the NIST-proven STM32 primitives) and raw CBC-MAC, instant
//! completion. Verified against FIPS-197 Appendix B/C (`B` ECB-128,
//! `C.1` ECB-256), SP 800-38A CBC-128/CTR-128, NIST SP 800-38D GCM
//! cases and Python-`cryptography`-oracle vectors. Deliberate scope:
//! ICM/CFB/XTS/F8/F9/CCM/DMA/`SAVE_CONTEXT` latch without acting;
//! non-12-byte GCM nonces (engine GHASH-derives `J0`; unmodeled);
//! `TAG` verify-fail signaling (the engine exposes none — firmware
//! compares); CBCMAC `DATA_OUT` (tag-only mode, regs stay stale).
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
const CTRL_MODE_CBCMAC: u32 = 0x2000_8000;
const CTRL_MODE_GCM: u32 = 0x2003_0040;
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
    /// CTR running counter + latched flag.
    ctr: [u8; 16],
    ctr_valid: bool,
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

    /// Latch CBC-MAC state (zero-IV chain over `C_LENGTH` bytes).
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
                    return [0u8; 16];
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
                    return [0u8; 16];
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
            self.g_latched = false;
            self.g_tag_valid = false;
            self.m_latched = false;
            self.m_tag_valid = false;
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
                CTRL_MODE_GCM => {
                    if !self.g_latched {
                        self.gcm_latch(view, &rk);
                    }
                    self.gcm_push(&rk, &fifo, encrypt);
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

/// A Tiva C AES engine (ECB/CBC/CTR/GCM/CBC-MAC 128/192/256; other
/// modes latch).
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
        // CFB mode latches without acting.
        let mut aes2 = new_aes();
        write_word(&mut aes2, CTRL_OFF, CTRL_DIRECTION | 0x08 | 0x400);
        block(
            &mut aes2,
            [0xa8f6_4332, 0x8d30_5a88, 0xa298_3131, 0x3407_37e0],
        );
        assert_eq!(read_word(&mut aes2, 0x8C) & IRQ_DATA_OUT, 0);
        // CCM likewise stays inert.
        let mut aes3 = new_aes();
        write_word(&mut aes3, CTRL_OFF, CTRL_DIRECTION | 0x08 | 0x2004_0040);
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
}
