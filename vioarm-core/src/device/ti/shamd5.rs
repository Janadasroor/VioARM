//! TI Tiva C SHA/MD5 engine (`SHAMD5`, `0x4403_4000`).
//!
//! Register map and `MODE`/`IRQSTATUS` semantics follow the TI
//! `hw_shamd5.h` oracle (`ALGO`: 0 = MD5, 2 = SHA-1, 4 = SHA-224,
//! 6 = SHA-256; `ALGO_CONSTANT` reloads the algorithm IV;
//! `CLOSE_HASH` pads and finalizes at the end of the block; `LENGTH` in
//! bytes; `OUTPUT_READY`/`INPUT_READY`/`CONTEXT_READY` IRQs). The SVD
//! carries no `<resetValue>`, so all words reset 0.
//!
//! Modeled: single-shot and multi-block SHA-256 over `DATA_0-15_IN`
//! (big-endian message words; firmware packs message bytes BE, as
//! TivaWare does), `LENGTH`-bounded with engine-side `CLOSE_HASH`
//! padding, digest out `ODIGEST_A-H` (BE words), `OUTPUT_READY`
//! synthesis + `IRQENABLE` gating. Verified against NIST FIPS 180-4
//! (`"abc"` → `ba7816bf…f20015ad`). Deliberate scope: other `ALGO`s
//! (MD5/SHA-1/SHA-224), `HMAC_*`, and `IDIGEST` custom IVs latch without
//! hashing (documented, untested paths stay inert rather than wrong).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// `SHAMD5` base address.
pub const SHAMD5_BASE: u32 = 0x4403_4000;

const ODIGEST_OFF: u32 = 0x00;
const MODE_OFF: u32 = 0x44;
const LENGTH_OFF: u32 = 0x48;
const DATA_OFF: u32 = 0x80;
const IRQSTATUS_OFF: u32 = 0x118;
const IRQENABLE_OFF: u32 = 0x11C;

/// `MODE` bits.
const MODE_ALGO_MASK: u32 = 0x07;
const MODE_ALGO_SHA256: u32 = 0x06;
const MODE_ALGO_CONSTANT: u32 = 1 << 3;
const MODE_CLOSE_HASH: u32 = 1 << 4;
/// `IRQSTATUS` bits.
const IRQ_OUTPUT_READY: u32 = 1 << 0;
const IRQ_INPUT_READY: u32 = 1 << 1;
const IRQ_CONTEXT_READY: u32 = 1 << 3;

/// SHA-256 round constants.
const K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

/// SHA-256 initial digest.
const H_INIT: [u32; 8] = [
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
];

/// Compress one 64-byte block (BE words) into `h`.
fn compress(h: &mut [u32; 8], block: &[u32; 16]) {
    let mut w = [0u32; 64];
    w[..16].copy_from_slice(block);
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
        let ch = (e & f) ^ (!e & g);
        let t1 = hh
            .wrapping_add(s1)
            .wrapping_add(ch)
            .wrapping_add(K[i])
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

/// Behavior behind the hash window.
#[derive(Debug)]
pub struct Shamd5Behavior {
    /// Running digest (valid once `output_ready`).
    digest: [u32; 8],
    /// Staged input block (BE words) + valid count.
    block: [u32; 16],
    staged: usize,
    /// Bytes consumed toward `LENGTH` in the current message.
    consumed: u32,
    output_ready: bool,
}

impl Default for Shamd5Behavior {
    fn default() -> Self {
        Self {
            digest: H_INIT,
            block: [0; 16],
            staged: 0,
            consumed: 0,
            output_ready: false,
        }
    }
}

impl Shamd5Behavior {
    /// Pad `digest` per SHA-256 over `total_len` message bytes (the staged
    /// partial block holds the first `remaining` bytes, BE-packed).
    fn close(&mut self, total_len: u32) {
        let remaining = total_len.saturating_sub(self.consumed) as usize;
        let mut msg = [0u8; 128];
        for (i, &w) in self.block.iter().enumerate() {
            msg[4 * i..4 * i + 4].copy_from_slice(&w.to_be_bytes());
        }
        let mut len = remaining.min(64);
        msg[len] = 0x80;
        len += 1;
        let blocks = if len > 56 { 2 } else { 1 };
        for b in msg[len..64 * blocks].iter_mut() {
            *b = 0;
        }
        let bit_len = u64::from(total_len) * 8;
        msg[64 * blocks - 8..64 * blocks].copy_from_slice(&bit_len.to_be_bytes());
        let mut h = self.digest;
        for b in 0..blocks {
            let mut words = [0u32; 16];
            for (i, chunk) in msg[64 * b..64 * b + 64].chunks(4).enumerate() {
                words[i] = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            }
            compress(&mut h, &words);
        }
        self.digest = h;
        self.block = [0; 16];
        self.staged = 0;
        self.consumed = 0;
        self.output_ready = true;
    }
}

impl Behavior for Shamd5Behavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        *self = Self::default();
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let mut irq = 0;
        if self.output_ready {
            irq |= IRQ_OUTPUT_READY;
        }
        // Input + context are always ready (instant completion).
        irq |= IRQ_INPUT_READY | IRQ_CONTEXT_READY;
        u32::from(irq & view.get(IRQENABLE_OFF) != 0)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            0x00..=0x1C => {
                let i = (offset as usize) / 4;
                self.digest[i]
            }
            0x118 => {
                let mut irq = IRQ_INPUT_READY | IRQ_CONTEXT_READY;
                if self.output_ready {
                    irq |= IRQ_OUTPUT_READY;
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
        if (0x00..=0x3C).contains(&offset) {
            return view.get(offset); // Digest/context words are RO here.
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        match offset {
            MODE_OFF => {
                if word & MODE_ALGO_CONSTANT != 0 {
                    // New hash: algorithm IV in, count reset. Only
                    // SHA-256 digests; other `ALGO`s stay inert.
                    if word & MODE_ALGO_MASK == MODE_ALGO_SHA256 {
                        self.digest = H_INIT;
                    }
                    self.block = [0; 16];
                    self.staged = 0;
                    self.consumed = 0;
                    self.output_ready = false;
                }
            }
            0x80..=0xBC => {
                if view.get(MODE_OFF) & MODE_ALGO_MASK != MODE_ALGO_SHA256 {
                    return word; // Unmodeled `ALGO`: words latch, no hash.
                }
                // Bus word stores land lane by lane; only the last lane
                // (`shift==24`) carries the assembled word (mirroring UART
                // `DR` / CRC `CRCDIN`).
                if shift != 24 {
                    return word;
                }
                let i = ((offset - DATA_OFF) / 4) as usize;
                if i == self.staged {
                    self.block[i] = word;
                    self.staged += 1;
                }
                let total = view.get(LENGTH_OFF);
                let close = view.get(MODE_OFF) & MODE_CLOSE_HASH != 0;
                // `CLOSE_HASH` finalizes once `LENGTH` bytes are staged
                // (short messages complete without filling the block);
                // full blocks without close compress and chain.
                if close && total > 0 && self.consumed + (self.staged as u32) * 4 >= total {
                    self.close(total);
                } else if self.staged == 16 {
                    let mut h = self.digest;
                    compress(&mut h, &self.block);
                    self.digest = h;
                    self.consumed += 64;
                    self.block = [0; 16];
                    self.staged = 0;
                }
            }
            _ => {}
        }
        word
    }
}

/// A Tiva C SHA-256 engine (other algorithms latch).
pub struct Shamd5 {
    inner: RegisterFile<Shamd5Behavior>,
}

impl Shamd5 {
    /// A new hash engine on `D`'s tables (`"SHAMD5"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"SHAMD5"`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("SHAMD5").expect("generated tables contain SHAMD5");
        Self {
            inner: RegisterFile::new(def, Shamd5Behavior::default()),
        }
    }
}

impl Peripheral for Shamd5 {
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

impl InterruptSource for Shamd5 {
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

    const SHAMD5_BASE: u32 = 0x4403_4000;
    const MODE_SHA256_NEW_CLOSE: u32 = MODE_ALGO_SHA256 | MODE_ALGO_CONSTANT | MODE_CLOSE_HASH;

    fn sha() -> Shamd5 {
        Shamd5::for_device::<Tm4c129xnczad>()
    }

    fn read_word(sha: &mut Shamd5, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(sha.read(SHAMD5_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(sha: &mut Shamd5, offset: u32, value: u32) {
        for lane in 0..4 {
            sha.write(
                SHAMD5_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Feed `bytes` BE-packed (firmware packs message bytes big-endian,
    /// as TivaWare does) with `LENGTH` in bytes.
    fn feed(sha: &mut Shamd5, bytes: &[u8]) {
        write_word(sha, LENGTH_OFF, bytes.len() as u32);
        let mut padded = bytes.to_vec();
        while padded.len() % 4 != 0 {
            padded.push(0);
        }
        for (i, chunk) in padded.chunks(4).enumerate() {
            let w = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            write_word(sha, DATA_OFF + 4 * i as u32, w);
        }
    }

    fn digest(sha: &mut Shamd5) -> [u32; 8] {
        core::array::from_fn(|i| read_word(sha, 4 * i as u32))
    }

    /// NIST FIPS 180-4: `SHA-256("abc")` =
    /// `ba7816bf 8f01cfea 414140de 5dae2223 b00361a3 96177a9c b410ff61 f20015ad`.
    #[test]
    fn nist_sha256_abc() {
        let mut sha = sha();
        write_word(&mut sha, MODE_OFF, MODE_SHA256_NEW_CLOSE);
        feed(&mut sha, b"abc");
        assert_eq!(
            digest(&mut sha),
            [
                0xba7816bf, 0x8f01cfea, 0x414140de, 0x5dae2223, 0xb00361a3, 0x96177a9c, 0xb410ff61,
                0xf20015ad
            ]
        );
        assert_eq!(
            read_word(&mut sha, IRQSTATUS_OFF) & IRQ_OUTPUT_READY,
            IRQ_OUTPUT_READY
        );
    }

    /// NIST FIPS 180-4: `SHA-256("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")`.
    #[test]
    fn nist_sha256_448bit() {
        let mut sha = sha();
        write_word(&mut sha, MODE_OFF, MODE_SHA256_NEW_CLOSE);
        feed(
            &mut sha,
            b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
        );
        assert_eq!(
            digest(&mut sha),
            [
                0x248d6a61, 0xd20638b8, 0xe5c02693, 0x0c3e6039, 0xa33ce459, 0x64ff2167, 0xf6ecedd4,
                0x19db06c1
            ]
        );
    }

    /// Multi-block without `CLOSE_HASH` chains; `IRQENABLE` gates the line.
    #[test]
    fn multiblock_chains_and_gates() {
        let mut sha = sha();
        write_word(&mut sha, IRQENABLE_OFF, IRQ_OUTPUT_READY);
        // 64 bytes without close: compresses, no output yet.
        write_word(&mut sha, MODE_OFF, MODE_ALGO_SHA256 | MODE_ALGO_CONSTANT);
        write_word(&mut sha, LENGTH_OFF, 128);
        for i in 0..16 {
            write_word(&mut sha, DATA_OFF + 4 * i, 0x61626364);
        }
        assert_eq!(read_word(&mut sha, IRQSTATUS_OFF) & IRQ_OUTPUT_READY, 0);
        assert_eq!(sha.pending(), 0);
        // Second block with close finalizes the 128-byte message.
        write_word(&mut sha, MODE_OFF, MODE_ALGO_SHA256 | MODE_CLOSE_HASH);
        write_word(&mut sha, LENGTH_OFF, 128);
        for i in 0..16 {
            write_word(&mut sha, DATA_OFF + 4 * i, 0x61626364);
        }
        assert_eq!(
            read_word(&mut sha, IRQSTATUS_OFF) & IRQ_OUTPUT_READY,
            IRQ_OUTPUT_READY
        );
        assert_eq!(sha.pending(), 1);
    }
}
