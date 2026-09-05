//! STM32 HASH accelerator (U5/L5 layout: `CR`/`DIN`/`STR`, `HRA0-4` aliased
//! digest + `HR0-7` digest, `IMR`/`SR`).
//!
//! `CR.INIT` resets the engine, `DIN` words accumulate the message
//! (`DATATYPE`-swapped like AES), and `STR` with `DCAL` and the last-word
//! valid-bit count (`NBLW`) finalizes: the digest lands in `HR0-7`
//! (`HRA0-4` alias words 0-4 for SHA-1 software), `SR.DCIS` latches done.
//! SHA-1, SHA-224 (SHA-256 truncated) and SHA-256 are hand-rolled (`ALGO`
//! 00/10/11 per the HAL table: SHA1/MD5/SHA224/SHA256); `DINIS`/`DCIE`/
//! `DINIE` gate the single interrupt line. Digests are stored numerically
//! per word (firmware byte-reverses words for the byte stream, as the HAL
//! does with `__REV`).
//!
//! Deliberate limits, all documented: MD5 (`ALGO` 01) and HMAC (`MODE`)
//! complete with a zeroed digest (broken-by-design and staged-key flows
//! are out of scope); the context-swap bank (`CSR0-53`, multi-context
//! streaming stacks) stays a table latch — those flows keep working only
//! through the registers they share; `DMAE`/`MDMAT` latch without effect.

use crate::device::svd::{Behavior, Device, PeripheralDef, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

// Register offsets (U5 layout).
const CR: u32 = 0x00;
const DIN: u32 = 0x04;
const STR: u32 = 0x08;
const HRA0: u32 = 0x0c;
const HR0: u32 = 0x310;
const IMR: u32 = 0x20;
const SR: u32 = 0x24;

// CR bits.
const INIT: u32 = 1 << 2;
const DATATYPE_SHIFT: u32 = 4;
const MODE: u32 = 1 << 6;
const ALGO_SHIFT: u32 = 17;
// ALGO encodings (HAL table: SHA1 00, MD5 01, SHA224 10, SHA256 11).
const ALGO_SHA1: u32 = 0;
const ALGO_SHA224: u32 = 2;
const ALGO_SHA256: u32 = 3;
// STR bits.
const NBLW_MASK: u32 = 0x1F;
const DCAL: u32 = 1 << 8;
// IMR bits.
const DINIE: u32 = 1 << 0;
const DCIE: u32 = 1 << 1;
// SR bits.
const DINIS: u32 = 1 << 0;
const DCIS: u32 = 1 << 1;

// --- SHA-256 (FIPS 180-4; SHA-224 truncates its output) ---

const SHA256_K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

fn sha256_block(h: &mut [u32; 8], block: &[u8; 64]) {
    let mut w = [0u32; 64];
    for (i, chunk) in block.chunks_exact(4).enumerate().take(16) {
        w[i] = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }
    let (mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh) =
        (h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
    for i in 0..64 {
        let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let ch = (e & f) ^ ((!e) & g);
        let t1 = hh
            .wrapping_add(s1)
            .wrapping_add(ch)
            .wrapping_add(SHA256_K[i])
            .wrapping_add(w[i]);
        let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let t2 = s0.wrapping_add(maj);
        hh = g;
        g = f;
        f = e;
        e = d.wrapping_add(t1);
        d = c;
        c = b;
        b = a;
        a = t1.wrapping_add(t2);
    }
    h[0] = h[0].wrapping_add(a);
    h[1] = h[1].wrapping_add(b);
    h[2] = h[2].wrapping_add(c);
    h[3] = h[3].wrapping_add(d);
    h[4] = h[4].wrapping_add(e);
    h[5] = h[5].wrapping_add(f);
    h[6] = h[6].wrapping_add(g);
    h[7] = h[7].wrapping_add(hh);
}

fn sha256_with_iv(message: &[u8], iv: [u32; 8]) -> [u32; 8] {
    let mut h = iv;
    let bit_len = message.len() as u64 * 8;
    let mut padded = message.to_vec();
    padded.push(0x80);
    while padded.len() % 64 != 56 {
        padded.push(0);
    }
    padded.extend(bit_len.to_be_bytes());
    for block in padded.chunks_exact(64) {
        let mut arr = [0u8; 64];
        arr.copy_from_slice(block);
        sha256_block(&mut h, &arr);
    }
    h
}

fn sha256(message: &[u8]) -> [u32; 8] {
    sha256_with_iv(
        message,
        [
            0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
            0x5be0cd19,
        ],
    )
}

/// SHA-224: the SHA-256 compression with its own IV, truncated to 7 words.
fn sha224(message: &[u8]) -> [u32; 7] {
    let full = sha256_with_iv(
        message,
        [
            0xc1059ed8, 0x367cd507, 0x3070dd17, 0xf70e5939, 0xffc00b31, 0x68581511, 0x64f98fa7,
            0xbefa4fa4,
        ],
    );
    [
        full[0], full[1], full[2], full[3], full[4], full[5], full[6],
    ]
}

// --- SHA-1 (FIPS 180-4) ---

fn sha1_block(h: &mut [u32; 5], block: &[u8; 64]) {
    let mut w = [0u32; 80];
    for (i, chunk) in block.chunks_exact(4).enumerate().take(16) {
        w[i] = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    for i in 16..80 {
        w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
    }
    let (mut a, mut b, mut c, mut d, mut e) = (h[0], h[1], h[2], h[3], h[4]);
    for (i, &wi) in w.iter().enumerate() {
        let (f, k) = match i {
            0..20 => ((b & c) | ((!b) & d), 0x5a827999),
            20..40 => (b ^ c ^ d, 0x6ed9eba1),
            40..60 => ((b & c) | (b & d) | (c & d), 0x8f1bbcdc),
            _ => (b ^ c ^ d, 0xca62c1d6),
        };
        let t = a
            .rotate_left(5)
            .wrapping_add(f)
            .wrapping_add(e)
            .wrapping_add(k)
            .wrapping_add(wi);
        e = d;
        d = c;
        c = b.rotate_left(30);
        b = a;
        a = t;
    }
    h[0] = h[0].wrapping_add(a);
    h[1] = h[1].wrapping_add(b);
    h[2] = h[2].wrapping_add(c);
    h[3] = h[3].wrapping_add(d);
    h[4] = h[4].wrapping_add(e);
}

fn sha1(message: &[u8]) -> [u32; 5] {
    let mut h = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0];
    let bit_len = message.len() as u64 * 8;
    let mut padded = message.to_vec();
    padded.push(0x80);
    while padded.len() % 64 != 56 {
        padded.push(0);
    }
    padded.extend(bit_len.to_be_bytes());
    for block in padded.chunks_exact(64) {
        let mut arr = [0u8; 64];
        arr.copy_from_slice(block);
        sha1_block(&mut h, &arr);
    }
    h
}

/// `DATATYPE` word swapping, symmetric with the AES peripheral.
const fn datatype_swap(word: u32, datatype: u32) -> u32 {
    match datatype {
        1 => word.rotate_left(16),
        2 => word.swap_bytes(),
        3 => word.reverse_bits(),
        _ => word,
    }
}

#[derive(Clone, Debug, Default)]
struct HashBehavior {
    /// Split `ALGO0`/`ALGO1` encoding (F4/L5 tables) vs contiguous `ALGO`
    /// (U5 tables); the four values mean the same on both.
    algo_split: bool,
    /// Message bytes accumulated since `INIT` (post-`DATATYPE` order).
    msg: Vec<u8>,
    /// Byte-lane assembler for partial `DIN` words.
    din_acc: u32,
    din_mask: u8,
    /// Words pushed (for `CR.NBW`).
    words_pushed: u32,
    /// Completed digest + validity.
    digest: [u32; 8],
    has_digest: bool,
    dcis: bool,
    inited: bool,
}

impl Behavior for HashBehavior {
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
                if word & INIT != 0 {
                    // Fresh message: digest, flags, and count reset; `INIT`
                    // is a pulse and reads back zero.
                    self.msg.clear();
                    self.din_acc = 0;
                    self.din_mask = 0;
                    self.words_pushed = 0;
                    self.digest = [0; 8];
                    self.has_digest = false;
                    self.dcis = false;
                    self.inited = true;
                    return word & !INIT;
                }
                word
            }
            DIN => {
                let lane = shift / 8;
                self.din_acc |= u32::from(val) << shift;
                self.din_mask |= 1 << lane;
                if self.din_mask == 0xF {
                    let datatype = (view.get(CR) >> 4) & 0x3;
                    let acc = self.din_acc;
                    self.din_acc = 0;
                    self.din_mask = 0;
                    // Words carry bytes big-endian (silicon convention, as
                    // with AES): the swap restores firmware byte order.
                    let swapped = datatype_swap(acc, datatype);
                    self.msg.extend(swapped.to_be_bytes());
                    self.words_pushed = self.words_pushed.wrapping_add(1);
                }
                view.merged(DIN, val, shift)
            }
            STR => {
                let word = view.merged(STR, val, shift);
                if word & DCAL != 0 {
                    self.finalize(view);
                }
                // `STR` holds no state: reads return the last `NBLW`.
                word & NBLW_MASK
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            CR => {
                // `INIT` reads back zero; `NBW` reports words pushed.
                (value & !INIT & !(0xF << 8)) | ((self.words_pushed & 0xF) << 8)
            }
            SR => {
                let mut word = value & !(DINIS | DCIS);
                if self.inited && !self.dcis {
                    word |= DINIS;
                }
                if self.dcis {
                    word |= DCIS;
                }
                word
            }
            _ if offset >= HRA0 && offset < HRA0 + 20 && offset % 4 == 0 => {
                let i = ((offset - HRA0) / 4) as usize;
                if self.has_digest {
                    self.digest[i]
                } else {
                    0
                }
            }
            _ if offset >= HR0 && offset < HR0 + 32 && offset % 4 == 0 => {
                let i = ((offset - HR0) / 4) as usize;
                if self.has_digest {
                    self.digest[i]
                } else {
                    0
                }
            }
            _ => value,
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let imr = view.get(IMR);
        let input = self.inited && !self.dcis && imr & DINIE != 0;
        let done = self.dcis && imr & DCIE != 0;
        u32::from(input || done)
    }

    fn reset(&mut self) {
        let algo_split = self.algo_split;
        *self = Self {
            algo_split,
            ..Self::default()
        };
    }

    fn needs_tick() -> bool {
        false
    }
}

impl HashBehavior {
    /// `STR.DCAL`: pad per `NBLW` and run the selected algorithm. MD5 and
    /// HMAC complete with a zeroed digest (documented limits).
    fn finalize(&mut self, view: &Registers<&mut [u32]>) {
        // Flush a partially-written last word (lane order), then cut the
        // message to the valid bits (`NBLW` 0 means a full word).
        if self.din_mask != 0 {
            let datatype = (view.get(CR) >> 4) & 0x3;
            let swapped = datatype_swap(self.din_acc, datatype);
            // LE-packed firmware fills lanes 0..k: the valid message bytes
            // are the big-endian prefix (see the full-word path).
            let bytes = swapped.to_be_bytes();
            let k = self.din_mask.count_ones() as usize;
            self.msg.extend(&bytes[..k.min(4)]);
            self.din_acc = 0;
            self.din_mask = 0;
        }
        let str_word = view.get(STR);
        let nblw = str_word & NBLW_MASK;
        let cr = view.get(CR);
        let algo = if self.algo_split {
            (((cr >> 18) & 1) << 1) | ((cr >> 7) & 1)
        } else {
            (cr >> ALGO_SHIFT) & 0x3
        };
        let hmac = cr & MODE != 0;
        let mut msg = core::mem::take(&mut self.msg);
        if nblw != 0 {
            // `NBLW` counts valid bits in the last word; byte multiples only
            // (bit-string tails are out of scope, documented).
            let valid = msg.len().saturating_sub(4) + nblw as usize / 8;
            msg.truncate(valid.min(msg.len()));
        }
        // The digest is stored numerically per word (firmware byte-reverses
        // words for the byte stream, as the HAL does with `__REV`).
        if !hmac && algo == ALGO_SHA256 {
            self.digest = sha256(&msg);
            self.has_digest = true;
        } else if !hmac && algo == ALGO_SHA224 {
            let short = sha224(&msg);
            self.digest[..7].copy_from_slice(&short);
            self.digest[7] = 0;
            self.has_digest = true;
        } else if !hmac && algo == ALGO_SHA1 {
            let short = sha1(&msg);
            self.digest[..5].copy_from_slice(&short);
            self.digest[5..].fill(0);
            self.has_digest = true;
        } else {
            self.digest = [0; 8];
            self.has_digest = true;
        }
        self.dcis = true;
    }
}

/// A full HASH accelerator behind one address space and one interrupt line.
#[derive(Debug)]
pub struct Hash {
    inner: RegisterFile<HashBehavior>,
}

impl Hash {
    /// Whether `def` is the supported HASH generation: digest aliases at
    /// `HRA0`, the full digest at `HR0`, `IMR`/`SR` beside them, and either
    /// the contiguous 2-bit `ALGO` (U5) or the split `ALGO0`/`ALGO1` pair
    /// (F4/L5, same values). Anything else (notably the SHA-512-capable
    /// H5/WBA IP with its 4-bit selector) stays a latch.
    #[must_use]
    pub fn layout_matches(def: &'static PeripheralDef) -> bool {
        Self::algo_split(def).is_some()
            && [CR, DIN, STR, HRA0, IMR, SR, HR0]
                .iter()
                .all(|off| def.registers.iter().any(|r| r.offset == *off))
    }

    /// The `ALGO` encoding of a matching table: `false` contiguous (U5),
    /// `true` split (F4/L5), `None` unsupported.
    fn algo_split(def: &'static PeripheralDef) -> Option<bool> {
        let fields = |off: u32| {
            def.registers
                .iter()
                .find(|r| r.offset == off)
                .map(|r| r.fields)
                .unwrap_or(&[])
        };
        let has = |off: u32, name: &str, bit: u32, width: u32| {
            fields(off).iter().any(|f| {
                f.name == name && u32::from(f.offset) == bit && u32::from(f.width) == width
            })
        };
        if has(CR, "ALGO", ALGO_SHIFT, 2) {
            Some(false)
        } else if has(CR, "ALGO0", 7, 1) && has(CR, "ALGO1", 18, 1) {
            Some(true)
        } else {
            None
        }
    }

    /// HASH on a chip's own tables, or `None` when the tables describe an
    /// unsupported generation (the window stays a latch then).
    #[must_use]
    pub fn new_for_opt<D: Device>() -> Option<Self> {
        let def = D::peripheral("HASH")?;
        if !Self::layout_matches(def) {
            return None;
        }
        Some(Self {
            inner: RegisterFile::new(
                def,
                HashBehavior {
                    algo_split: Self::algo_split(def).unwrap_or(false),
                    ..HashBehavior::default()
                },
            ),
        })
    }

    /// HASH on a chip's own tables (U5/L5 layout).
    ///
    /// # Panics
    /// Panics if `D` lacks `HASH` or describes an unsupported generation.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        Self::new_for_opt::<D>().expect("supported HASH tables")
    }
}

impl Peripheral for Hash {
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
        "hash"
    }
}

impl InterruptSource for Hash {
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
    use super::Hash;
    use crate::device::svd::stm32l562::Stm32l562;
    use crate::device::svd::stm32u575::Stm32u575;
    use crate::device::{InterruptSource, Peripheral};

    const CR: u32 = 0x00;
    const DIN: u32 = 0x04;
    const STR: u32 = 0x08;
    const HRA0: u32 = 0x0c;
    const HR0: u32 = 0x310;
    const IMR: u32 = 0x20;
    const SR: u32 = 0x24;

    // ALGO per the HAL table.
    const SHA1: u32 = 0 << 17;
    const SHA224: u32 = 2 << 17;
    const SHA256: u32 = 3 << 17;

    fn hash() -> (Hash, u32) {
        let mut h = Hash::new_for::<Stm32u575>();
        let base = h.inner.def().base;
        assert_eq!(base, 0x420C_0400);
        h.reset();
        (h, base)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn wr(h: &mut Hash, base: u32, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            h.write(base + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn rd(h: &mut Hash, base: u32, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = h.read(base + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    /// INIT, push `msg` as LE words (memcpy style, DATATYPE 8-bit to keep
    /// byte order), then `STR` with `DCAL` and the valid-bit count.
    fn digest(h: &mut Hash, base: u32, algo: u32, msg: &[u8]) -> Vec<u32> {
        wr(h, base, CR, algo | (2 << 4) | (1 << 2)); // ALGO DATATYPE=8b INIT
        assert_ne!(rd(h, base, SR) & 1, 0, "DINIS ready");
        let mut chunks = msg.chunks_exact(4);
        for c in &mut chunks {
            wr(h, base, DIN, u32::from_le_bytes([c[0], c[1], c[2], c[3]]));
        }
        let rem = chunks.remainder();
        if !rem.is_empty() {
            let mut last = [0u8; 4];
            last[..rem.len()].copy_from_slice(rem);
            wr(h, base, DIN, u32::from_le_bytes(last));
        }
        let nblw = if rem.is_empty() {
            0
        } else {
            rem.len() as u32 * 8
        };
        wr(h, base, STR, (1 << 8) | nblw); // DCAL
        assert_ne!(rd(h, base, SR) & (1 << 1), 0, "DCIS");
        (0..8).map(|i| rd(h, base, HR0 + 4 * i)).collect()
    }

    #[test]
    fn sha256_nist_vectors() {
        let (mut h, base) = hash();
        // FIPS 180-4 A.1.
        assert_eq!(
            digest(&mut h, base, SHA256, b"abc"),
            [
                0xba7816bf, 0x8f01cfea, 0x414140de, 0x5dae2223, 0xb00361a3, 0x96177a9c, 0xb410ff61,
                0xf20015ad
            ],
            "SHA-256(abc)"
        );
        // Two-block message (A.2, 112 bytes).
        assert_eq!(
            digest(
                &mut h,
                base,
                SHA256,
                b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
            ),
            [
                0x248d6a61, 0xd20638b8, 0xe5c02693, 0x0c3e6039, 0xa33ce459, 0x64ff2167, 0xf6ecedd4,
                0x19db06c1
            ],
            "SHA-256(A.2)"
        );
        // Empty message (A.3).
        assert_eq!(
            digest(&mut h, base, SHA256, b"")[..2],
            [0xe3b0c442, 0x98fc1c14],
            "SHA-256(empty)"
        );
    }

    #[test]
    fn sha256_million_a() {
        let (mut h, base) = hash();
        let msg = vec![b'a'; 1_000_000];
        assert_eq!(
            digest(&mut h, base, SHA256, &msg)[..2],
            [0xcdc76e5c, 0x9914fb92],
            "SHA-256(1M a)"
        );
    }

    #[test]
    fn sha224_and_sha1_vectors() {
        let (mut h, base) = hash();
        let d224 = digest(&mut h, base, SHA224, b"abc");
        assert_eq!(
            d224[..7],
            [0x23097d22, 0x3405d822, 0x8642a477, 0xbda255b3, 0x2aadbce4, 0xbda0b3f7, 0xe36c9da7],
            "SHA-224(abc)"
        );
        let d1 = digest(&mut h, base, SHA1, b"abc");
        assert_eq!(
            d1[..5],
            [0xa9993e36, 0x4706816a, 0xba3e2571, 0x7850c26c, 0x9cd0d89d],
            "SHA-1(abc)"
        );
        // SHA-1 digest aliases through HRA0-4.
        let alias: Vec<u32> = (0..5).map(|i| rd(&mut h, base, HRA0 + 4 * i)).collect();
        assert_eq!(alias, d1[..5], "HRA aliases HR for SHA-1");
    }

    #[test]
    fn hmac_and_md5_complete_with_zeros() {
        let (mut h, base) = hash();
        // HMAC mode (documented: staged-key flows unmodeled).
        wr(&mut h, base, CR, SHA256 | (2 << 4) | (1 << 6) | (1 << 2));
        wr(&mut h, base, DIN, 0x61626380);
        wr(&mut h, base, STR, (1 << 8) | 24);
        assert_eq!(
            (0..8)
                .map(|i| rd(&mut h, base, HR0 + 4 * i))
                .collect::<Vec<_>>(),
            vec![0; 8],
            "HMAC zeroes the digest"
        );
        // MD5 ALGO (documented: broken-by-design, unmodeled).
        assert_eq!(
            digest(&mut h, base, 1 << 17, b"abc"),
            vec![0; 8],
            "MD5 zeroes the digest"
        );
    }

    #[test]
    fn init_resets_mid_message() {
        let (mut h, base) = hash();
        wr(&mut h, base, CR, SHA256 | (1 << 2));
        wr(&mut h, base, DIN, 0xdead_beef);
        wr(&mut h, base, CR, SHA256 | (1 << 2)); // INIT again
        assert_eq!(rd(&mut h, base, CR) & (1 << 2), 0, "INIT is a pulse");
        assert_eq!(
            digest(&mut h, base, SHA256, b"abc")[0],
            0xba7816bf,
            "stale words discarded"
        );
    }

    #[test]
    fn interrupts_gate_on_imr() {
        let (mut h, base) = hash();
        assert_eq!(h.pending(), 0);
        assert_eq!(h.exception_number(), Some(16 + 96), "HASH on U575");
        // DINIS with DINIE raises the line once initialized.
        wr(&mut h, base, IMR, 1); // DINIE
        wr(&mut h, base, CR, SHA256 | (1 << 2));
        assert_eq!(h.pending(), 1, "DINIS+DINIE");
        wr(&mut h, base, IMR, 2); // DCIE only
        assert_eq!(h.pending(), 0, "DINIE masked");
        wr(&mut h, base, DIN, 0x61626380);
        wr(&mut h, base, STR, (1 << 8) | 24);
        assert_eq!(h.pending(), 1, "DCIS+DCIE");
    }

    #[test]
    fn l562_tables_share_the_layout() {
        // The L5 HASH window probes compatible and runs the NIST vector on
        // its tables (split `ALGO0`/`ALGO1` path), proving the wiring.
        let mut h = Hash::new_for_opt::<Stm32l562>().expect("L562 probes");
        let base = h.inner.def().base;
        h.reset();
        // Split encoding: SHA256 = ALGO1|ALGO0 (F4 values).
        assert_eq!(
            digest(&mut h, base, (1 << 18) | (1 << 7), b"abc")[0],
            0xba7816bf,
            "SHA-256(abc) on L562 tables"
        );
    }

    #[test]
    fn probe_accepts_u5_and_rejects_h5() {
        use crate::device::svd::stm32h563::Stm32h563;
        use crate::device::svd::stm32u5g9::Stm32u5g9;
        use crate::device::svd::stm32wba55::Stm32wba55;
        use crate::device::svd::Device;
        assert!(Hash::new_for_opt::<Stm32u575>().is_some(), "U575");
        assert!(Hash::new_for_opt::<Stm32u5g9>().is_some(), "U5G9");
        assert!(Hash::new_for_opt::<Stm32l562>().is_some(), "L562");
        assert!(
            Stm32h563::peripheral("HASH").is_some_and(|p| !Hash::layout_matches(p)),
            "H5 stays a latch"
        );
        assert!(
            Stm32wba55::peripheral("HASH").is_some_and(|p| !Hash::layout_matches(p)),
            "WBA stays a latch"
        );
    }
}
