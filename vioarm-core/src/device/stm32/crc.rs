//! The STM32 CRC calculation unit (`CRC`, `0x4002_3000`), driven by the
//! SVD-generated register tables.
//!
//! The unit holds a 32-bit `DR` (data register, offset `0x00`), an 8-bit `IDR`
//! (independent data, `0x04`), and a `CR` (`0x08`, bit 0 `RESET` self-clears
//! and reloads `0xFFFF_FFFF` into the CRC register). A write to `DR` feeds the
//! 32-bit word into the CRC with polynomial `0x04C1_1DB7` (init `0xFFFF_FFFF`,
//! no final xor, bit-by-bit as on silicon). Reads of `DR` return the current
//! CRC; `IDR` is plain 8-bit storage. The SVD tables drive the decode; the
//! `CrcBehavior` latch implements the calculation.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Poly `0x04C1_1DB7` CRC-32 without refin/refout, init `0xFFFF_FFFF`.
fn crc32_step(mut crc: u32, data: u32) -> u32 {
    crc ^= data;
    for _ in 0..32 {
        if crc & 0x8000_0000 != 0 {
            crc = (crc << 1) ^ 0x04C1_1DB7;
        } else {
            crc <<= 1;
        }
    }
    crc
}

#[derive(Debug)]
struct CrcBehavior {
    crc: u32,
}

impl CrcBehavior {
    const fn new() -> Self {
        Self { crc: 0xFFFF_FFFF }
    }
}

impl Behavior for CrcBehavior {
    // CRC folds synchronously on DR writes.
    fn needs_tick() -> bool {
        false
    }
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            0x00 => {
                let word = view.merged(offset, val, shift);
                // A `DR` word write feeds the CRC when the last byte arrives;
                // intermediate bytes just latch.
                if shift == 24 {
                    self.crc = crc32_step(self.crc, word);
                    return self.crc;
                }
                word
            }
            0x08 => {
                // `CR` bit 0 `RESET`: reload `0xFFFF_FFFF`, self-clears.
                let word = view.merged(offset, val, shift);
                if word & 1 != 0 {
                    self.crc = 0xFFFF_FFFF;
                    view.set(0x00, self.crc);
                    return 0;
                }
                word
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn reset(&mut self) {
        self.crc = 0xFFFF_FFFF;
    }
}

/// The `CRC` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct Crc {
    inner: RegisterFile<CrcBehavior>,
}

impl Crc {
    /// A new `CRC` at its SVD base, sharing no mailbox.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("CRC").expect("generated tables contain CRC");
        let mut rf = RegisterFile::new(def, CrcBehavior::new());
        // Reset value `0xFFFF_FFFF` in `DR`.
        rf.reset();
        Self { inner: rf }
    }
}

impl Peripheral for Crc {
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
        "crc"
    }
}

impl InterruptSource for Crc {
    fn exception_number(&self) -> Option<u8> {
        None
    }
    fn pending(&self) -> u32 {
        0
    }
    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::Crc;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::Peripheral;

    const BASE: u32 = 0x4002_3000;

    fn write_word(c: &mut Crc, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            c.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }
    fn read_word(c: &mut Crc, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = c.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn crc_resets_to_all_ones_and_computes() {
        let mut c = Crc::new_for::<Stm32f103>();
        assert_eq!(read_word(&mut c, BASE), 0xFFFF_FFFF);
        write_word(&mut c, BASE, 0x1234_5678);
        let v = read_word(&mut c, BASE);
        assert_ne!(v, 0xFFFF_FFFF);
        assert_ne!(v, 0x1234_5678);
        // RESET reloads.
        write_word(&mut c, BASE + 0x08, 1);
        assert_eq!(read_word(&mut c, BASE), 0xFFFF_FFFF);
    }

    #[test]
    fn idr_is_plain_storage() {
        let mut c = Crc::new_for::<Stm32f103>();
        write_word(&mut c, BASE + 0x04, 0xAB);
        assert_eq!(read_word(&mut c, BASE + 0x04) & 0xFF, 0xAB);
        // IDR does not affect CRC.
        let before = read_word(&mut c, BASE);
        write_word(&mut c, BASE + 0x04, 0xFF);
        assert_eq!(read_word(&mut c, BASE), before);
    }
}
