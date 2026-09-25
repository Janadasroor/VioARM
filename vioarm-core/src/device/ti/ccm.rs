//! TI Tiva C CRC module (`CCM0`, `0x4403_0000`).
//!
//! Register map and bit meanings follow the TM4C1294NCPDT datasheet
//! (SPMS433B §12, CRCCTRL page: `SIZE` 0 = 32-bit word / 1 = 8-bit byte,
//! `INIT` 0 = seed / 2 = all-0 / 3 = all-1, `ENDIAN` swaps, `BR`/`OBR`
//! bit-reversals, `RESINV` result invert) plus the TI `hw_ccm.h` oracle
//! (`TYPE`: 0 = `P8055`, 1 = `P1021`, 2 = `P4C11DB7`, 3 = `P1EDC6F41`,
//! 8 = TCP checksum). The SVD carries no `<resetValue>`, so all words
//! reset 0.
//!
//! Modeled: CRC-32 IEEE (`TYPE=2`, `P4C11DB7`) over `CRCDIN` word/byte
//! writes with `CRCSEED`/`INIT` context start, `ENDIAN`/`BR` input
//! transforms, `RESINV`/`OBR` output post-processing into `CRCRSLTPP`
//! (RO). Verified against NIST `CRC-32/ISO-HDLC` (`"123456789"` →
//! `0xCBF43926` with seed all-1s + invert). Deliberate scope: other
//! `TYPE`s (16-bit polys, CRC-32C, TCP checksum) are accepted by the
//! latch but do not advance the context (documented, untested paths stay
//! inert rather than wrong); `uDMA` feeding is equivalent to CPU word
//! writes here since completion is instant.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// `CCM0` base address.
pub const CCM_BASE: u32 = 0x4403_0000;

const CRCCTRL_OFF: u32 = 0x400;
const CRCSEED_OFF: u32 = 0x410;
const CRCDIN_OFF: u32 = 0x414;
const CRCRSLTPP_OFF: u32 = 0x418;

/// `CRCCTRL` fields.
const CTRL_TYPE_MASK: u32 = 0x0F;
const CTRL_ENDIAN_SHIFT: u32 = 4;
const CTRL_ENDIAN_MASK: u32 = 0x03;
const CTRL_BR: u32 = 1 << 7;
const CTRL_OBR: u32 = 1 << 8;
const CTRL_RESINV: u32 = 1 << 9;
const CTRL_SIZE: u32 = 1 << 12;
const CTRL_INIT_SHIFT: u32 = 13;
const CTRL_INIT_MASK: u32 = 0x03;
/// Operation types.
const TYPE_P4C11DB7: u32 = 0x02;
/// `INIT` encodings.
const INIT_SEED: u32 = 0x00;
const INIT_ZEROS: u32 = 0x02;
const INIT_ONES: u32 = 0x03;
/// Reflected CRC-32 IEEE polynomial.
const CRC32_POLY: u32 = 0xEDB8_8320;

/// Reflected CRC-32 update over `word` (LE bytes, LSB-first).
fn crc32_update(mut crc: u32, word: u32) -> u32 {
    let mut v = word;
    for _ in 0..32 {
        let mix = (crc ^ v) & 1;
        crc >>= 1;
        if mix != 0 {
            crc ^= CRC32_POLY;
        }
        v >>= 1;
    }
    crc
}

/// Reverse the bits in each byte of `word` (`BR` transform).
fn bit_reverse_bytes(word: u32) -> u32 {
    let mut out = 0u32;
    for i in 0..4 {
        out |= (word >> (8 * i) & 0xFF).reverse_bits() << (8 * i);
    }
    out
}

/// Reverse all 32 bits (`OBR` transform).
fn bit_reverse_word(word: u32) -> u32 {
    word.reverse_bits()
}

/// `ENDIAN` byte-lane remap: `{B3,B2,B1,B0}` → selection.
fn endian_swap(word: u32, endian: u32) -> u32 {
    let b = word.to_be_bytes();
    match endian {
        0x01 => u32::from_be_bytes([b[2], b[3], b[0], b[1]]),
        0x02 => u32::from_be_bytes([b[1], b[0], b[3], b[2]]),
        0x03 => u32::from_be_bytes([b[0], b[1], b[2], b[3]]),
        _ => word,
    }
}

/// Behavior behind the CRC window.
#[derive(Debug, Default)]
pub struct CcmBehavior {
    /// Running CRC context (mirrors `CRCSEED`).
    crc: u32,
}

impl Behavior for CcmBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        self.crc = 0;
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            CRCSEED_OFF => self.crc,
            CRCRSLTPP_OFF => {
                let ctrl = view.get(CRCCTRL_OFF);
                let mut out = self.crc;
                if ctrl & CTRL_OBR != 0 {
                    out = bit_reverse_word(out);
                }
                if ctrl & CTRL_RESINV != 0 {
                    out = !out;
                }
                out
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
        if offset == CRCRSLTPP_OFF {
            return view.get(offset); // RO result rejects writes.
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        match offset {
            CRCCTRL_OFF => {
                // `INIT` selects the context start (`SEED` keeps the
                // current context; firmware loads it via `CRCSEED`).
                match (word >> CTRL_INIT_SHIFT) & CTRL_INIT_MASK {
                    INIT_ZEROS => self.crc = 0,
                    INIT_ONES => self.crc = 0xFFFF_FFFF,
                    _ => {}
                }
            }
            CRCSEED_OFF => self.crc = word,
            CRCDIN_OFF => {
                // Register 1 `INIT` is self-clearing: the first `CRCDIN`
                // write clears it to zero and it stays zero unless
                // rewritten (SPMS433B §12, Register 1). Clear on any lane
                // so partial-word writes still count as the first write.
                let ctrl_init = view.get(CRCCTRL_OFF);
                if (ctrl_init >> CTRL_INIT_SHIFT) & CTRL_INIT_MASK != 0 {
                    view.set(
                        CRCCTRL_OFF,
                        ctrl_init & !(CTRL_INIT_MASK << CTRL_INIT_SHIFT),
                    );
                }
                let ctrl = view.get(CRCCTRL_OFF);
                if ctrl & CTRL_TYPE_MASK == TYPE_P4C11DB7 {
                    if ctrl & CTRL_SIZE != 0 {
                        // Byte mode: each lane-0 byte feeds 8 clocks.
                        if shift == 0 {
                            let mut crc = self.crc;
                            let mut v = val as u32;
                            for _ in 0..8 {
                                let mix = (crc ^ v) & 1;
                                crc >>= 1;
                                if mix != 0 {
                                    crc ^= CRC32_POLY;
                                }
                                v >>= 1;
                            }
                            self.crc = crc;
                        }
                    } else if shift == 24 {
                        // Word mode: feed the assembled word once whole
                        // (bus word stores land lane by lane; `shift==24`
                        // is the last lane, mirroring UART `DR`).
                        let mut data = word;
                        data = endian_swap(data, (ctrl >> CTRL_ENDIAN_SHIFT) & CTRL_ENDIAN_MASK);
                        if ctrl & CTRL_BR != 0 {
                            data = bit_reverse_bytes(data);
                        }
                        self.crc = crc32_update(self.crc, data);
                    }
                }
            }
            _ => {}
        }
        word
    }
}

/// A Tiva C CRC module (CRC-32 IEEE path live; other types latch).
pub struct Ccm {
    inner: RegisterFile<CcmBehavior>,
}

impl Ccm {
    /// A new CRC module on `D`'s tables (`"CCM0"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"CCM0"`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("CCM0").expect("generated tables contain CCM0");
        Self {
            inner: RegisterFile::new(def, CcmBehavior::default()),
        }
    }
}

impl Peripheral for Ccm {
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

impl InterruptSource for Ccm {
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

    const CCM0_BASE: u32 = 0x4403_0000;

    fn ccm0() -> Ccm {
        Ccm::for_device::<Tm4c129xnczad>()
    }

    fn read_word(ccm: &mut Ccm, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(ccm.read(CCM0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(ccm: &mut Ccm, offset: u32, value: u32) {
        for lane in 0..4 {
            ccm.write(
                CCM0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Feed one byte (LSB-first clocks) — shared by word/byte tests.
    fn feed_bytes(ccm: &mut Ccm, bytes: &[u8]) {
        for chunk in bytes.chunks(4) {
            let mut w = 0u32;
            for (i, &b) in chunk.iter().enumerate() {
                w |= u32::from(b) << (8 * i);
            }
            write_word(ccm, CRCDIN_OFF, w);
        }
    }

    /// NIST CRC-32/ISO-HDLC over 8 bytes (`"12345678"` → `0x9AE0DAAF`;
    /// word-mode clocks whole words, so the 9-byte `"123456789"` vector
    /// runs in byte mode below).
    #[test]
    fn nist_crc32_check() {
        let mut ccm = ccm0();
        // `TYPE=P4C11DB7`, 32-bit words, `INIT` all-1s, result invert.
        write_word(
            &mut ccm,
            CRCCTRL_OFF,
            TYPE_P4C11DB7 | (INIT_ONES << CTRL_INIT_SHIFT) | CTRL_RESINV,
        );
        feed_bytes(&mut ccm, b"12345678");
        assert_eq!(read_word(&mut ccm, CRCRSLTPP_OFF), 0x9AE0_DAAF);
    }

    /// `CRCSEED` loads the context; `INIT=SEED` keeps it; byte mode feeds
    /// low bytes only (9-byte NIST vector runs in byte mode since word
    /// mode clocks whole words).
    #[test]
    fn seed_and_byte_mode() {
        let mut ccm = ccm0();
        write_word(&mut ccm, CRCCTRL_OFF, TYPE_P4C11DB7 | CTRL_RESINV);
        write_word(&mut ccm, CRCSEED_OFF, 0xFFFF_FFFF);
        feed_bytes(&mut ccm, b"12345678");
        assert_eq!(read_word(&mut ccm, CRCRSLTPP_OFF), 0x9AE0_DAAF);
        // Byte mode over the same string matches word mode.
        let mut ccm = ccm0();
        write_word(
            &mut ccm,
            CRCCTRL_OFF,
            TYPE_P4C11DB7 | (INIT_ONES << CTRL_INIT_SHIFT) | CTRL_RESINV | CTRL_SIZE,
        );
        for &b in b"123456789" {
            write_word(&mut ccm, CRCDIN_OFF, u32::from(b));
        }
        assert_eq!(read_word(&mut ccm, CRCRSLTPP_OFF), 0xCBF4_3926);
    }

    /// `CRCRSLTPP` is RO; other `TYPE`s leave the context alone.
    #[test]
    fn ro_and_deferred_types() {
        let mut ccm = ccm0();
        write_word(&mut ccm, CRCRSLTPP_OFF, 0xDEAD_BEEF);
        assert_eq!(read_word(&mut ccm, CRCRSLTPP_OFF), 0);
        write_word(&mut ccm, CRCCTRL_OFF, 0x01); // P1021
        write_word(&mut ccm, CRCDIN_OFF, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut ccm, CRCSEED_OFF), 0);
    }

    /// Register 1 `INIT` self-clears with the first `CRCDIN` write
    /// (SPMS433B §12) and stays clear for the rest of the operation.
    #[test]
    fn init_self_clears_on_first_din() {
        let mut ccm = ccm0();
        write_word(
            &mut ccm,
            CRCCTRL_OFF,
            TYPE_P4C11DB7 | (INIT_ONES << CTRL_INIT_SHIFT),
        );
        assert_eq!(
            (read_word(&mut ccm, CRCCTRL_OFF) >> CTRL_INIT_SHIFT) & CTRL_INIT_MASK,
            INIT_ONES,
            "INIT latched"
        );
        write_word(&mut ccm, CRCDIN_OFF, 0x1234_5678);
        assert_eq!(
            (read_word(&mut ccm, CRCCTRL_OFF) >> CTRL_INIT_SHIFT) & CTRL_INIT_MASK,
            0,
            "INIT self-cleared by first CRCDIN"
        );
        // A second block does not re-init: the context keeps running.
        let after_first = read_word(&mut ccm, CRCSEED_OFF);
        write_word(&mut ccm, CRCDIN_OFF, 0x1234_5678);
        assert_ne!(
            read_word(&mut ccm, CRCSEED_OFF),
            after_first,
            "context advances without re-init"
        );
        assert_eq!(
            (read_word(&mut ccm, CRCCTRL_OFF) >> CTRL_INIT_SHIFT) & CTRL_INIT_MASK,
            0,
            "INIT stays clear"
        );
    }
}
