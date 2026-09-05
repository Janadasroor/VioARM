//! STM32 AES hardware accelerator: the classic 128-bit IP (`L4`/`G0`,
//! `CR`/`SR`/`DINR`/`DOUTR`/`KEYR0-3`/`IVR0-3`) and the `U5` 128/256-bit IP
//! (`KEYR4-7`, `KEYSIZE`, `BUSY`/`KEYVALID`; `SAES` shares the programming
//! model at its own base).
//!
//! `CR.EN` + four `DINR` words push one block through ECB/CBC/CTR, encrypt
//! or decrypt per `MODE` (key derivation only latches `CCF`), with the key
//! from `KEYRx` and the CBC IV / CTR counter from `IVRx` chaining across
//! blocks until `EN` falls. `DOUTR` pops the result words, `SR.CCF`
//! latches done, and `CCFIE`/`ERRIE` gate the single interrupt line. The
//! cipher itself is a hand-rolled FIPS-197 core (no dependency to audit,
//! deterministic by construction), proven by NIST vectors driven through
//! the registers.
//!
//! Deliberate limits, all documented: GCM/GMAC/CMAC complete with zeroed
//! `DOUTR` (no GHASH engine); DMA request lines latch without effect;
//! suspend (`SUSPR`) latches without save/restore semantics; shared and
//! device-unique keys (`KMOD`, the `SAES` hardware key) run as normal
//! software-provided keys — key hiding and side-channel hardening are out
//! of scope. `DATATYPE` 0/1/2/3 swapping applies symmetrically to data in
//! and out.

use crate::device::svd::{
    Access, Behavior, Device, IrqDef, PeripheralDef, RegisterDef, RegisterFile, Registers,
};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

// Register offsets, shared by the classic and U5 layouts (`KEYR0-3` and
// `IVR0-3` sit at identical offsets; U5 appends `KEYR4-7` at `0x30`).
const CR: u32 = 0x00;
const SR: u32 = 0x04;
const DINR: u32 = 0x08;
const DOUTR: u32 = 0x0c;
const KEYR0: u32 = 0x10;
const IVR0: u32 = 0x20;
const KEYR4: u32 = 0x30;

// CR bits.
const EN: u32 = 1 << 0;
const DATATYPE_SHIFT: u32 = 1;
const MODE_SHIFT: u32 = 3;
const CHMOD_SHIFT: u32 = 5;
const CCFC: u32 = 1 << 7;
const ERRC: u32 = 1 << 8;
const CCFIE: u32 = 1 << 9;
const ERRIE: u32 = 1 << 10;
const CHMOD2: u32 = 1 << 16; // U5 GCM/GMAC/CMAC selector
const KEYSIZE: u32 = 1 << 18; // U5 256-bit key
const IPRST: u32 = 1 << 31; // U5 IP reset
const MODE_ENC: u32 = 0;
const MODE_KEYDERIV: u32 = 1;
const MODE_DEC: u32 = 2;
// SR bits.
const SR_CCF: u32 = 1 << 0;
const SR_RDERR: u32 = 1 << 1;
const SR_WRERR: u32 = 1 << 2;
const SR_KEYVALID: u32 = 1 << 7; // U5 only (see `allow_256`)

// --- FIPS-197 AES core over big-endian byte blocks ---

const SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

const INV_SBOX: [u8; 256] = [
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d,
];

const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

const fn xtime(a: u8) -> u8 {
    if a & 0x80 != 0 {
        (a << 1) ^ 0x1b
    } else {
        a << 1
    }
}

fn gmul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0u8;
    let mut i = 0;
    while i < 8 {
        if b & 1 != 0 {
            p ^= a;
        }
        let hi = a & 0x80 != 0;
        a <<= 1;
        if hi {
            a ^= 0x1b;
        }
        b >>= 1;
        i += 1;
    }
    p
}

/// Expand a 16- or 32-byte key into round-key words (big-endian per FIPS).
fn key_expand(key: &[u8]) -> Vec<u32> {
    let nk = key.len() / 4;
    let rounds = nk + 6;
    let mut w = vec![0u32; 4 * (rounds + 1)];
    for (i, chunk) in key.chunks_exact(4).enumerate() {
        w[i] = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    for i in nk..4 * (rounds + 1) {
        let mut temp = w[i - 1];
        if i % nk == 0 {
            temp = u32::from_be_bytes([
                SBOX[(temp >> 16) as u8 as usize],
                SBOX[(temp >> 8) as u8 as usize],
                SBOX[temp as u8 as usize],
                SBOX[(temp >> 24) as u8 as usize],
            ]) ^ (u32::from(RCON[i / nk - 1]) << 24);
        } else if nk > 6 && i % nk == 4 {
            temp = u32::from_be_bytes([
                SBOX[(temp >> 24) as u8 as usize],
                SBOX[(temp >> 16) as u8 as usize],
                SBOX[(temp >> 8) as u8 as usize],
                SBOX[temp as u8 as usize],
            ]);
        }
        w[i] = w[i - nk] ^ temp;
    }
    w
}

fn add_round_key(state: &mut [u8; 16], rk: &[u32], round: usize) {
    for col in 0..4 {
        let k = rk[4 * round + col].to_be_bytes();
        for row in 0..4 {
            state[row + 4 * col] ^= k[row];
        }
    }
}

fn cipher_block(block: &[u8; 16], rk: &[u32], rounds: usize) -> [u8; 16] {
    let mut s = *block;
    add_round_key(&mut s, rk, 0);
    for round in 1..rounds {
        for b in s.iter_mut() {
            *b = SBOX[*b as usize];
        }
        // ShiftRows: row r rotates left by r.
        let t = s;
        for row in 0..4 {
            for col in 0..4 {
                s[row + 4 * col] = t[row + 4 * ((col + row) % 4)];
            }
        }
        // MixColumns.
        for col in 0..4 {
            let a = [s[4 * col], s[1 + 4 * col], s[2 + 4 * col], s[3 + 4 * col]];
            s[4 * col] = gmul(a[0], 2) ^ gmul(a[1], 3) ^ a[2] ^ a[3];
            s[1 + 4 * col] = a[0] ^ gmul(a[1], 2) ^ gmul(a[2], 3) ^ a[3];
            s[2 + 4 * col] = a[0] ^ a[1] ^ gmul(a[2], 2) ^ gmul(a[3], 3);
            s[3 + 4 * col] = gmul(a[0], 3) ^ a[1] ^ a[2] ^ gmul(a[3], 2);
        }
        add_round_key(&mut s, rk, round);
    }
    for b in s.iter_mut() {
        *b = SBOX[*b as usize];
    }
    let t = s;
    for row in 0..4 {
        for col in 0..4 {
            s[row + 4 * col] = t[row + 4 * ((col + row) % 4)];
        }
    }
    add_round_key(&mut s, rk, rounds);
    s
}

fn inv_cipher_block(block: &[u8; 16], rk: &[u32], rounds: usize) -> [u8; 16] {
    let mut s = *block;
    add_round_key(&mut s, rk, rounds);
    for round in (1..rounds).rev() {
        // InvShiftRows: row r rotates right by r.
        let t = s;
        for row in 0..4 {
            for col in 0..4 {
                s[row + 4 * col] = t[row + 4 * ((col + 4 - row) % 4)];
            }
        }
        for b in s.iter_mut() {
            *b = INV_SBOX[*b as usize];
        }
        add_round_key(&mut s, rk, round);
        // InvMixColumns.
        for col in 0..4 {
            let a = [s[4 * col], s[1 + 4 * col], s[2 + 4 * col], s[3 + 4 * col]];
            s[4 * col] = gmul(a[0], 0x0e) ^ gmul(a[1], 0x0b) ^ gmul(a[2], 0x0d) ^ gmul(a[3], 0x09);
            s[1 + 4 * col] =
                gmul(a[0], 0x09) ^ gmul(a[1], 0x0e) ^ gmul(a[2], 0x0b) ^ gmul(a[3], 0x0d);
            s[2 + 4 * col] =
                gmul(a[0], 0x0d) ^ gmul(a[1], 0x09) ^ gmul(a[2], 0x0e) ^ gmul(a[3], 0x0b);
            s[3 + 4 * col] =
                gmul(a[0], 0x0b) ^ gmul(a[1], 0x0d) ^ gmul(a[2], 0x09) ^ gmul(a[3], 0x0e);
        }
    }
    let t = s;
    for row in 0..4 {
        for col in 0..4 {
            s[row + 4 * col] = t[row + 4 * ((col + 4 - row) % 4)];
        }
    }
    for b in s.iter_mut() {
        *b = INV_SBOX[*b as usize];
    }
    add_round_key(&mut s, rk, 0);
    s
}

/// `DATATYPE` word swapping, symmetric on the way in and out.
const fn datatype_swap(word: u32, datatype: u32) -> u32 {
    match datatype {
        1 => word.rotate_left(16),
        2 => word.swap_bytes(),
        3 => word.reverse_bits(),
        _ => word,
    }
}

#[derive(Clone, Debug)]
struct AesBehavior {
    /// 256-bit keys exist only where the tables carry `KEYR4-7` (U5).
    allow_256: bool,
    /// Staged input words and the byte-lane assembler behind them.
    din: [u32; 4],
    din_n: u8,
    din_acc: u32,
    din_mask: u8,
    /// Result words, how many the firmware already popped, and the byte
    /// lane within the current word (one pop per word read).
    dout: [u32; 4],
    dout_i: u8,
    dout_lane: u8,
    /// Running CBC IV / CTR counter, live once a session started one.
    chain: [u8; 16],
    chaining: bool,
    ccf: bool,
    rderr: bool,
    wrerr: bool,
    key_valid: bool,
}

impl Default for AesBehavior {
    fn default() -> Self {
        Self {
            allow_256: false,
            din: [0; 4],
            din_n: 0,
            din_acc: 0,
            din_mask: 0,
            dout: [0; 4],
            dout_i: 4,
            dout_lane: 0,
            chain: [0; 16],
            chaining: false,
            ccf: false,
            rderr: false,
            wrerr: false,
            key_valid: false,
        }
    }
}

impl AesBehavior {
    fn enabled(view: &Registers<&mut [u32]>) -> bool {
        view.get(CR) & EN != 0
    }

    /// Run one staged block through the programmed mode into `dout`.
    fn compute(&mut self, view: &mut Registers<&mut [u32]>) {
        let cr = view.get(CR);
        let datatype = (cr >> DATATYPE_SHIFT) & 0x3;
        let mode = (cr >> MODE_SHIFT) & 0x3;
        let chmod = (cr >> CHMOD_SHIFT) & 0x3;
        let gcm = chmod == 0x3 || cr & CHMOD2 != 0;
        // Key-derivation mode only latches done; decryption reuses the
        // encryption schedule directly, so no derived key is ever needed.
        if mode == MODE_KEYDERIV {
            self.dout = [0; 4];
            self.dout_i = 4;
            self.din_n = 0;
            self.ccf = true;
            return;
        }
        let use_256 = self.allow_256 && cr & KEYSIZE != 0;
        let mut key = [0u8; 32];
        for (i, chunk) in key[..if use_256 { 32 } else { 16 }]
            .chunks_exact_mut(4)
            .enumerate()
        {
            let w = if i < 4 {
                view.get(KEYR0 + 4 * i as u32)
            } else {
                view.get(KEYR4 + 4 * (i - 4) as u32)
            };
            chunk.copy_from_slice(&w.to_be_bytes());
        }
        let key_len = if use_256 { 32 } else { 16 };
        let rk = key_expand(&key[..key_len]);
        let rounds = rk.len() / 4 - 1;
        let mut block = [0u8; 16];
        for (i, chunk) in block.chunks_exact_mut(4).enumerate() {
            chunk.copy_from_slice(&datatype_swap(self.din[i], datatype).to_be_bytes());
        }
        let out = if gcm {
            // No GHASH engine: complete with zeros (documented).
            [0u8; 16]
        } else {
            match chmod {
                // ECB.
                0 => {
                    if mode == MODE_DEC {
                        inv_cipher_block(&block, &rk, rounds)
                    } else {
                        cipher_block(&block, &rk, rounds)
                    }
                }
                // CBC with IV chaining across the `EN` session.
                1 => {
                    if !self.chaining {
                        for (i, chunk) in self.chain.chunks_exact_mut(4).enumerate() {
                            chunk.copy_from_slice(&view.get(IVR0 + 4 * i as u32).to_be_bytes());
                        }
                        self.chaining = true;
                    }
                    if mode == MODE_DEC {
                        let pt = inv_cipher_block(&block, &rk, rounds);
                        let mut out = [0u8; 16];
                        for i in 0..16 {
                            out[i] = pt[i] ^ self.chain[i];
                        }
                        self.chain = block;
                        out
                    } else {
                        let mut xored = [0u8; 16];
                        for i in 0..16 {
                            xored[i] = block[i] ^ self.chain[i];
                        }
                        let out = cipher_block(&xored, &rk, rounds);
                        self.chain = out;
                        out
                    }
                }
                // CTR: big-endian counter from `IVR`, keystream XOR.
                _ => {
                    if !self.chaining {
                        for (i, chunk) in self.chain.chunks_exact_mut(4).enumerate() {
                            chunk.copy_from_slice(&view.get(IVR0 + 4 * i as u32).to_be_bytes());
                        }
                        self.chaining = true;
                    }
                    let ks = cipher_block(&self.chain, &rk, rounds);
                    // Standard 128-bit big-endian increment.
                    for b in self.chain.iter_mut().rev() {
                        let (v, carry) = b.overflowing_add(1);
                        *b = v;
                        if !carry {
                            break;
                        }
                    }
                    let mut out = [0u8; 16];
                    for i in 0..16 {
                        out[i] = block[i] ^ ks[i];
                    }
                    out
                }
            }
        };
        for (i, chunk) in out.chunks_exact(4).enumerate() {
            let w = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            self.dout[i] = datatype_swap(w, datatype);
        }
        self.dout_i = 0;
        self.dout_lane = 0;
        self.din_n = 0;
        self.ccf = true;
    }
}

impl Behavior for AesBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            CR => {
                let word = view.merged(CR, val, shift);
                if word & IPRST != 0 {
                    // IP reset: zero the programming (all AES resets are 0).
                    for off in [CR, SR, DINR, DOUTR] {
                        view.set(off, 0);
                    }
                    for i in 0..8u32 {
                        view.set(KEYR0 + 4 * i, 0);
                    }
                    for i in 0..4u32 {
                        view.set(IVR0 + 4 * i, 0);
                    }
                    *self = Self {
                        allow_256: self.allow_256,
                        ..Self::default()
                    };
                    return 0;
                }
                let was_en = view.get(CR) & EN != 0;
                let en = word & EN != 0;
                // Command bits read back zero.
                let stored = word & !(CCFC | ERRC);
                if en && !was_en {
                    // Fresh session: chaining restarts and `CCF` clears (error
                    // flags are sticky until `ERRC`); a block pre-staged
                    // while disabled runs now.
                    self.chaining = false;
                    self.ccf = false;
                    self.dout_i = 4;
                    view.set(CR, stored);
                    if self.din_n == 4 {
                        self.compute(view);
                    }
                    return view.get(CR);
                }
                if !en {
                    self.chaining = false;
                    self.ccf = false;
                }
                if word & CCFC != 0 {
                    self.ccf = false;
                }
                if word & ERRC != 0 {
                    self.rderr = false;
                    self.wrerr = false;
                }
                stored
            }
            SR => view.get(SR), // flags clear through CR, never SR writes
            DINR => {
                let lane = shift / 8;
                self.din_acc |= u32::from(val) << shift;
                self.din_mask |= 1 << lane;
                if self.din_mask == 0xF {
                    let word = self.din_acc;
                    self.din_acc = 0;
                    self.din_mask = 0;
                    // Overrun: staging past a full block, or pushing data
                    // while the last result still waits in `DOUTR`.
                    if self.din_n == 4 || (self.ccf && self.dout_i < 4) {
                        self.wrerr = true;
                    } else {
                        self.din[self.din_n as usize] = word;
                        self.din_n += 1;
                        if self.din_n == 4 && Self::enabled(view) {
                            self.compute(view);
                        }
                    }
                }
                view.merged(DINR, val, shift)
            }
            DOUTR => view.get(DOUTR), // read-only pop window
            KEYR0 | 0x14 | 0x18 | 0x1c => {
                if offset == 0x1c {
                    self.key_valid = true;
                }
                view.merged(offset, val, shift)
            }
            KEYR4 | 0x34 | 0x38 | 0x3c => {
                if offset == 0x3c {
                    self.key_valid = true;
                }
                view.merged(offset, val, shift)
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            SR => {
                let mut word = value & !(SR_CCF | SR_RDERR | SR_WRERR | SR_KEYVALID);
                if self.ccf {
                    word |= SR_CCF;
                }
                if self.rderr {
                    word |= SR_RDERR;
                }
                if self.wrerr {
                    word |= SR_WRERR;
                }
                if self.allow_256 && self.key_valid {
                    word |= SR_KEYVALID;
                }
                word
            }
            DOUTR => {
                // One pop per word read: the framework calls once per byte
                // lane, so lanes share a counter and only the fourth pops.
                // (HAL and boot code always read `DOUTR` as words.)
                if !self.ccf || self.dout_i >= 4 {
                    self.rderr = true;
                    return 0;
                }
                let word = self.dout[self.dout_i as usize];
                self.dout_lane += 1;
                if self.dout_lane == 4 {
                    self.dout_lane = 0;
                    self.dout_i += 1;
                }
                word
            }
            _ => value,
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let cr = view.get(CR);
        let done = self.ccf && cr & CCFIE != 0;
        let err = (self.rderr || self.wrerr) && cr & ERRIE != 0;
        u32::from(done || err)
    }

    fn reset(&mut self) {
        let allow_256 = self.allow_256;
        *self = Self {
            allow_256,
            ..Self::default()
        };
    }

    fn needs_tick() -> bool {
        false
    }
}

/// One manual register entry.
const fn reg(name: &'static str, offset: u32, access: Access) -> RegisterDef {
    RegisterDef {
        name,
        offset,
        width: 32,
        access,
        reset: 0,
        fields: &[],
    }
}

/// U5-layout AES/SAES registers (`KEYR0-3`, `IVR0-3`, `KEYR4-7`,
/// context-save latches; GCM registers omitted — GCM completes with zeros,
/// and a Reserved fault there would only hide that stance).
static U5_AES_REGS: &[RegisterDef] = &[
    reg("CR", 0x00, Access::ReadWrite),
    reg("SR", 0x04, Access::ReadWrite),
    reg("DINR", 0x08, Access::ReadWrite),
    reg("DOUTR", 0x0c, Access::ReadOnly),
    reg("KEYR0", 0x10, Access::ReadWrite),
    reg("KEYR1", 0x14, Access::ReadWrite),
    reg("KEYR2", 0x18, Access::ReadWrite),
    reg("KEYR3", 0x1c, Access::ReadWrite),
    reg("IVR0", 0x20, Access::ReadWrite),
    reg("IVR1", 0x24, Access::ReadWrite),
    reg("IVR2", 0x28, Access::ReadWrite),
    reg("IVR3", 0x2c, Access::ReadWrite),
    reg("KEYR4", 0x30, Access::ReadWrite),
    reg("KEYR5", 0x34, Access::ReadWrite),
    reg("KEYR6", 0x38, Access::ReadWrite),
    reg("KEYR7", 0x3c, Access::ReadWrite),
    reg("SUSPR0", 0x40, Access::ReadWrite),
    reg("SUSPR1", 0x44, Access::ReadWrite),
    reg("SUSPR2", 0x48, Access::ReadWrite),
    reg("SUSPR3", 0x4c, Access::ReadWrite),
    reg("SUSPR4", 0x50, Access::ReadWrite),
    reg("SUSPR5", 0x54, Access::ReadWrite),
    reg("SUSPR6", 0x58, Access::ReadWrite),
    reg("SUSPR7", 0x5c, Access::ReadWrite),
];

/// Classic 128-bit layout for chips whose tables lack AES (`G041`).
static G0_AES_REGS: &[RegisterDef] = &[
    reg("CR", 0x00, Access::ReadWrite),
    reg("SR", 0x04, Access::ReadWrite),
    reg("DINR", 0x08, Access::ReadWrite),
    reg("DOUTR", 0x0c, Access::ReadOnly),
    reg("KEYR0", 0x10, Access::ReadWrite),
    reg("KEYR1", 0x14, Access::ReadWrite),
    reg("KEYR2", 0x18, Access::ReadWrite),
    reg("KEYR3", 0x1c, Access::ReadWrite),
    reg("IVR0", 0x20, Access::ReadWrite),
    reg("IVR1", 0x24, Access::ReadWrite),
    reg("IVR2", 0x28, Access::ReadWrite),
    reg("IVR3", 0x2c, Access::ReadWrite),
];

static U5_AES_DEF: PeripheralDef = PeripheralDef {
    name: "AES",
    base: 0x420C_0000,
    window: 0x400,
    irqs: &[IrqDef {
        number: 93,
        name: "AES",
    }],
    registers: U5_AES_REGS,
};

static U5_SAES_DEF: PeripheralDef = PeripheralDef {
    name: "SAES",
    base: 0x420C_0C00,
    window: 0x400,
    irqs: &[IrqDef {
        number: 28,
        name: "SAES",
    }],
    registers: U5_AES_REGS,
};

static G0_AES_DEF: PeripheralDef = PeripheralDef {
    name: "AES",
    base: 0x4002_6000,
    window: 0x400,
    irqs: &[IrqDef {
        number: 31,
        name: "AES_RNG",
    }],
    registers: G0_AES_REGS,
};

/// A full AES accelerator behind one address space and one interrupt line.
#[derive(Debug)]
pub struct Aes {
    inner: RegisterFile<AesBehavior>,
}

impl Aes {
    fn behavior_for(def: &'static PeripheralDef) -> AesBehavior {
        AesBehavior {
            allow_256: def.registers.iter().any(|r| r.offset == KEYR4),
            ..AesBehavior::default()
        }
    }

    /// AES on a chip's own tables (`L4`/`G0` 128-bit; 256-bit only where
    /// the tables carry `KEYR4-7`).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain `AES`.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("AES").expect("generated tables contain AES");
        let behavior = Self::behavior_for(def);
        Self {
            inner: RegisterFile::new(def, behavior),
        }
    }

    /// Manual U5 AES at `0x420C_0000` (IRQ 93) when SVD lacks the entry.
    /// Same layout the U585 SVD describes, with `KEYR4-7` for 256-bit keys.
    #[must_use]
    pub fn new_u5_aes() -> Self {
        Self {
            inner: RegisterFile::new(
                &U5_AES_DEF,
                AesBehavior {
                    allow_256: true,
                    ..AesBehavior::default()
                },
            ),
        }
    }

    /// Manual U5 SAES at `0x420C_0C00` (IRQ 28): the same ECB/CBC/CTR
    /// core; hardware-shared keys run as normal programmed keys.
    #[must_use]
    pub fn new_u5_saes() -> Self {
        Self {
            inner: RegisterFile::new(
                &U5_SAES_DEF,
                AesBehavior {
                    allow_256: true,
                    ..AesBehavior::default()
                },
            ),
        }
    }

    /// Manual classic 128-bit AES at `0x4002_6000` (IRQ 31) for `G041`.
    #[must_use]
    pub fn new_g0() -> Self {
        Self {
            inner: RegisterFile::new(&G0_AES_DEF, AesBehavior::default()),
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
    fn name(&self) -> &'static str {
        "aes"
    }
}

impl InterruptSource for Aes {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines)
    }
}

#[cfg(test)]
mod tests {
    use super::Aes;
    use crate::device::svd::stm32l476::Stm32l476;
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const CR: u32 = 0x00;
    const SR: u32 = 0x04;
    const DINR: u32 = 0x08;
    const DOUTR: u32 = 0x0c;
    const KEYR0: u32 = 0x10;
    const IVR0: u32 = 0x20;

    fn aes() -> (Aes, u32) {
        let mut a = Aes::new_for::<Stm32l476>();
        let base = a.ranges()[0].base;
        assert_eq!(base, 0x5006_0000);
        a.reset();
        (a, base)
    }

    fn aes256() -> (Aes, u32) {
        let mut a = Aes::new_u5_aes();
        let base = a.ranges()[0].base;
        assert_eq!(base, 0x420C_0000);
        a.reset();
        (a, base)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn wr(a: &mut Aes, base: u32, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            a.write(base + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn rd(a: &mut Aes, base: u32, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = a.read(base + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn words(bytes: &[u8]) -> Vec<u32> {
        bytes
            .chunks_exact(4)
            .map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]]))
            .collect()
    }

    /// Program key + mode, enable, push one block, pop the result. Starts
    /// from a disabled peripheral (fresh `EN` session, like the HAL).
    /// Key words land MSB-first; words 4-7 use the U5 `KEYR4-7` window.
    fn block(
        a: &mut Aes,
        base: u32,
        key: &[u8],
        cr: u32,
        iv: Option<&[u8]>,
        pt: &[u8],
    ) -> Vec<u32> {
        wr(a, base, CR, 0); // disable: fresh session
        for (i, w) in words(key).iter().enumerate() {
            let off = if i < 4 {
                KEYR0 + 4 * i as u32
            } else {
                0x30 + 4 * (i - 4) as u32
            };
            wr(a, base, off, *w);
        }
        if let Some(iv) = iv {
            for (i, w) in words(iv).iter().enumerate() {
                wr(a, base, IVR0 + 4 * i as u32, *w);
            }
        }
        wr(a, base, CR, cr | 1); // EN
        for w in words(pt) {
            wr(a, base, DINR, w);
        }
        assert_ne!(rd(a, base, SR) & 1, 0, "CCF");
        (0..4).map(|_| rd(a, base, DOUTR)).collect()
    }

    fn hex(bytes: &[u8]) -> Vec<u8> {
        (0..bytes.len())
            .step_by(2)
            .map(|i| {
                u8::from_str_radix(std::str::from_utf8(&bytes[i..i + 2]).unwrap(), 16).unwrap()
            })
            .collect()
    }

    #[test]
    fn aes128_ecb_fips197() {
        let (mut a, base) = aes();
        let key = hex(b"000102030405060708090a0b0c0d0e0f");
        let pt = hex(b"00112233445566778899aabbccddeeff");
        let out = block(&mut a, base, &key, 0, None, &pt);
        let mut bytes = Vec::new();
        for w in out {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(
            bytes,
            hex(b"69c4e0d86a7b0430d8cdb78070b4c55a"),
            "FIPS-197 B"
        );
        // And back.
        let back = block(&mut a, base, &key, 2 << 3, None, &bytes);
        let mut bytes = Vec::new();
        for w in back {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(bytes, pt, "ECB decrypt round-trips");
    }

    #[test]
    fn aes256_ecb_fips197() {
        let (mut a, base) = aes256();
        let key = hex(b"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
        let pt = hex(b"00112233445566778899aabbccddeeff");
        let out = block(&mut a, base, &key, 1 << 18, None, &pt);
        let mut bytes = Vec::new();
        for w in out {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(
            bytes,
            hex(b"8ea2b7ca516745bfeafc49904b496089"),
            "FIPS-197 C.3"
        );
        let back = block(&mut a, base, &key, (1 << 18) | (2 << 3), None, &bytes);
        let mut bytes = Vec::new();
        for w in back {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(bytes, pt, "ECB-256 decrypt round-trips");
    }

    #[test]
    fn aes128_cbc_sp800_38a() {
        let (mut a, base) = aes();
        let key = hex(b"2b7e151628aed2a6abf7158809cf4f3c");
        let iv = hex(b"000102030405060708090a0b0c0d0e0f");
        let pt = hex(b"6bc1bee22e409f96e93d7e117393172a");
        // Two blocks chained through one EN session (F.2.1 XORs the second
        // plaintext with the first ciphertext on silicon; here the model
        // chains internally across consecutive blocks).
        let out1 = block(&mut a, base, &key, 1 << 5, Some(&iv), &pt);
        let mut bytes = Vec::new();
        for w in out1 {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(
            bytes,
            hex(b"7649abac8119b246cee98e9b12e9197d"),
            "SP 800-38A F.2.1"
        );
        // Decrypt passes the IV again after re-enabling.
        let back = block(&mut a, base, &key, (1 << 5) | (2 << 3), Some(&iv), &bytes);
        let mut bytes = Vec::new();
        for w in back {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(bytes, pt, "CBC decrypt round-trips");
    }

    #[test]
    fn aes128_ctr_sp800_38a() {
        let (mut a, base) = aes();
        let key = hex(b"2b7e151628aed2a6abf7158809cf4f3c");
        let ctr = hex(b"f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff");
        let pt = hex(b"6bc1bee22e409f96e93d7e117393172a");
        let out = block(&mut a, base, &key, 2 << 5, Some(&ctr), &pt);
        let mut bytes = Vec::new();
        for w in out {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(
            bytes,
            hex(b"874d6191b620e3261bef6864990db6ce"),
            "SP 800-38A F.5.1"
        );
    }

    #[test]
    fn cbc_chains_across_blocks_in_one_session() {
        let (mut a, base) = aes();
        let key = hex(b"2b7e151628aed2a6abf7158809cf4f3c");
        let iv = hex(b"000102030405060708090a0b0c0d0e0f");
        let pt1 = hex(b"6bc1bee22e409f96e93d7e117393172a");
        let pt2 = hex(b"00112233445566778899aabbccddeeff");
        for (i, w) in words(&key).iter().enumerate() {
            wr(&mut a, base, KEYR0 + 4 * i as u32, *w);
        }
        for (i, w) in words(&iv).iter().enumerate() {
            wr(&mut a, base, IVR0 + 4 * i as u32, *w);
        }
        wr(&mut a, base, CR, (1 << 5) | 1); // CBC EN
        for w in words(&pt1) {
            wr(&mut a, base, DINR, w);
        }
        let b1: Vec<u32> = (0..4).map(|_| rd(&mut a, base, DOUTR)).collect();
        let mut ct1 = Vec::new();
        for w in &b1 {
            ct1.extend(w.to_be_bytes());
        }
        assert_eq!(ct1, hex(b"7649abac8119b246cee98e9b12e9197d"), "F.2.1 blk1");
        // Second block chains on the first ciphertext without re-enabling.
        wr(&mut a, base, CR, (1 << 5) | 1 | (1 << 7)); // CCFC, stay enabled
        for w in words(&pt2) {
            wr(&mut a, base, DINR, w);
        }
        let b2: Vec<u32> = (0..4).map(|_| rd(&mut a, base, DOUTR)).collect();
        let mut ct2 = Vec::new();
        for w in &b2 {
            ct2.extend(w.to_be_bytes());
        }
        // Decrypting both blocks in one fresh session must return both
        // plaintexts — only correct chaining makes the second one land.
        wr(&mut a, base, CR, 0);
        for (i, w) in words(&key).iter().enumerate() {
            wr(&mut a, base, KEYR0 + 4 * i as u32, *w);
        }
        for (i, w) in words(&iv).iter().enumerate() {
            wr(&mut a, base, IVR0 + 4 * i as u32, *w);
        }
        wr(&mut a, base, CR, (1 << 5) | (2 << 3) | 1); // CBC decrypt EN
        for w in words(&ct1) {
            wr(&mut a, base, DINR, w);
        }
        let d1: Vec<u32> = (0..4).map(|_| rd(&mut a, base, DOUTR)).collect();
        wr(&mut a, base, CR, (1 << 5) | (2 << 3) | 1 | (1 << 7));
        for w in words(&ct2) {
            wr(&mut a, base, DINR, w);
        }
        let d2: Vec<u32> = (0..4).map(|_| rd(&mut a, base, DOUTR)).collect();
        let flat = |ws: &[u32]| {
            let mut b = Vec::new();
            for w in ws {
                b.extend(w.to_be_bytes());
            }
            b
        };
        assert_eq!(flat(&d1), pt1);
        assert_eq!(flat(&d2), pt2, "chained decrypt round-trips");
    }

    #[test]
    fn datatype_byte_swap_round_trips() {
        let (mut a, base) = aes();
        // `DATATYPE` swaps data words, never key words (silicon rule): the
        // key goes in MSB-first while memcpy-style data goes in LE, and the
        // swap restores NIST byte order through the core and back out.
        let key = hex(b"000102030405060708090a0b0c0d0e0f");
        let pt = hex(b"00112233445566778899aabbccddeeff");
        wr(&mut a, base, CR, 0);
        for (i, w) in words(&key).iter().enumerate() {
            wr(&mut a, base, KEYR0 + 4 * i as u32, *w);
        }
        wr(&mut a, base, CR, (2 << 1) | 1); // DATATYPE byte-swap + EN
        for c in pt.chunks_exact(4) {
            wr(
                &mut a,
                base,
                DINR,
                u32::from_le_bytes([c[0], c[1], c[2], c[3]]),
            );
        }
        assert_ne!(rd(&mut a, base, SR) & 1, 0, "CCF");
        let out: Vec<u32> = (0..4).map(|_| rd(&mut a, base, DOUTR)).collect();
        let expect: Vec<u32> = words(&hex(b"69c4e0d86a7b0430d8cdb78070b4c55a"))
            .into_iter()
            .map(u32::swap_bytes)
            .collect();
        assert_eq!(out, expect);
    }

    #[test]
    fn key_derivation_latches_done_without_data() {
        let (mut a, base) = aes();
        let key = hex(b"000102030405060708090a0b0c0d0e0f");
        for (i, w) in words(&key).iter().enumerate() {
            wr(&mut a, base, KEYR0 + 4 * i as u32, *w);
        }
        wr(&mut a, base, CR, (1 << 3) | 1); // MODE key-derivation + EN
                                            // No DINR writes: derivation alone still completes.
        wr(&mut a, base, DINR, 0);
        wr(&mut a, base, DINR, 0);
        wr(&mut a, base, DINR, 0);
        wr(&mut a, base, DINR, 0);
        assert_ne!(rd(&mut a, base, SR) & 1, 0, "CCF");
    }

    #[test]
    fn gcm_mode_completes_with_zeros() {
        let (mut a, base) = aes256();
        let key = hex(b"000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f");
        let out = block(&mut a, base, &key, (1 << 18) | (3 << 5), None, &[0u8; 16]);
        assert_eq!(out, vec![0, 0, 0, 0], "unmodeled GCM zeroes DOUTR");
    }

    #[test]
    fn error_flags_and_clear_bits() {
        let (mut a, base) = aes();
        // DOUTR read with nothing computed latches RDERR.
        assert_eq!(rd(&mut a, base, DOUTR), 0);
        assert_ne!(rd(&mut a, base, SR) & (1 << 1), 0, "RDERR");
        wr(&mut a, base, CR, 1 | (1 << 8)); // EN + ERRC
        assert_eq!(rd(&mut a, base, SR) & (1 << 1), 0, "ERRC clears");
        // Five DINR words without draining latches WRERR.
        wr(&mut a, base, CR, 1);
        for _ in 0..5 {
            wr(&mut a, base, DINR, 0xA5A5_A5A5);
        }
        assert_ne!(rd(&mut a, base, SR) & (1 << 2), 0, "WRERR");
        // CCF sets on the completed block; CCFC clears it.
        assert_ne!(rd(&mut a, base, SR) & 1, 0, "CCF");
        wr(&mut a, base, CR, 1 | (1 << 7));
        assert_eq!(rd(&mut a, base, SR) & 1, 0, "CCFC clears");
    }

    #[test]
    fn interrupts_gate_on_ccfie_and_errie() {
        let (mut a, base) = aes();
        assert_eq!(a.pending(), 0);
        assert_eq!(a.exception_number(), Some(16 + 79), "AES on L476");
        // Error with ERRIE raises the line.
        assert_eq!(rd(&mut a, base, DOUTR), 0); // RDERR
        assert_eq!(a.pending(), 0, "masked");
        wr(&mut a, base, CR, 1 | (1 << 10)); // EN + ERRIE
        assert_eq!(a.pending(), 1, "ERRIE raises");
        wr(&mut a, base, CR, 1 | (1 << 10) | (1 << 8)); // ERRC
        assert_eq!(a.pending(), 0);
        // Completion with CCFIE raises the line.
        wr(&mut a, base, CR, 1 | (1 << 9)); // EN + CCFIE
        for _ in 0..4 {
            wr(&mut a, base, DINR, 0);
        }
        assert_eq!(a.pending(), 1, "CCFIE raises");
    }

    #[test]
    fn u5_keyvalid_and_saes_layout() {
        let (mut a, base) = aes256();
        assert_eq!(rd(&mut a, base, SR) & (1 << 7), 0, "no key yet");
        let key = hex(b"000102030405060708090a0b0c0d0e0f");
        for (i, w) in words(&key).iter().enumerate() {
            wr(&mut a, base, KEYR0 + 4 * i as u32, *w);
        }
        assert_ne!(rd(&mut a, base, SR) & (1 << 7), 0, "KEYVALID");
        let mut s = Aes::new_u5_saes();
        let sbase = s.ranges()[0].base;
        assert_eq!(sbase, 0x420C_0C00);
        assert_eq!(s.exception_number(), Some(16 + 28), "SAES");
        // Same core answers through the SAES window.
        let out = block(
            &mut s,
            sbase,
            &key,
            0,
            None,
            &hex(b"00112233445566778899aabbccddeeff"),
        );
        let mut bytes = Vec::new();
        for w in out {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(bytes, hex(b"69c4e0d86a7b0430d8cdb78070b4c55a"));
    }

    #[test]
    fn g0_manual_aes_is_128_bit() {
        let mut a = Aes::new_g0();
        let base = a.ranges()[0].base;
        assert_eq!(base, 0x4002_6000);
        assert_eq!(a.exception_number(), Some(16 + 31), "AES_RNG");
        // No KEYR4 on the classic layout.
        assert!(matches!(
            a.write(base + 0x30, 0xFF),
            Err(BusError::Reserved)
        ));
        let key = hex(b"000102030405060708090a0b0c0d0e0f");
        let out = block(
            &mut a,
            base,
            &key,
            0,
            None,
            &hex(b"00112233445566778899aabbccddeeff"),
        );
        let mut bytes = Vec::new();
        for w in out {
            bytes.extend(w.to_be_bytes());
        }
        assert_eq!(bytes, hex(b"69c4e0d86a7b0430d8cdb78070b4c55a"));
    }
}
