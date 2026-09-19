//! The STM32 CRC calculation unit, driven by the SVD-generated register tables.
//!
//! Fixed units (`F1`/`F4`: `DR`/`IDR`/`CR` with a `RESET` bit) compute
//! CRC-32/MPEG-2 (poly `0x04C1_1DB7`, init `0xFFFF_FFFF`, no reflection).
//! Programmable units (`H7`/`G4`/`L4`/`U5` and friends: `DR`/`IDR`/`CR` plus
//! `INIT` and `POL`) add: polynomial size (`CR.POLYSIZE`: 32/16/8/7),
//! input reversal (`CR.REV_IN`: none/per-byte/per-halfword/per-word),
//! output reversal (`CR.REV_OUT`), a programmable polynomial (`POL`, low
//! bits with the top bit implicit) and a programmable init (`INIT`).
//! A write to `DR` feeds one 32-bit word: the low `S` bits for an `S`-bit
//! poly (documented choice for sub-word polys), reflected per `REV_IN`,
//! folded MSB-first; the `S`-bit result lands low-aligned in `DR`
//! (`REV_OUT` reverses all 32 readback bits). `RESET` reloads `INIT`
//! (or `0xFFFF_FFFF` without it); changing the configuration wants a
//! `RESET` for deterministic results, as on silicon. Byte/halfword `DR`
//! writes latch only (32-bit path); `IDR` stays plain storage.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Reverse the bits of each byte in `v`.
fn rev_bytes(mut v: u32) -> u32 {
    v = ((v & 0x5555_5555) << 1) | ((v >> 1) & 0x5555_5555);
    v = ((v & 0x3333_3333) << 2) | ((v >> 2) & 0x3333_3333);
    v = ((v & 0x0F0F_0F0F) << 4) | ((v >> 4) & 0x0F0F_0F0F);
    v
}

/// Reverse the bits of each 16-bit half in `v`.
fn rev_halves(v: u32) -> u32 {
    let lo = rev_bytes(v & 0xFFFF);
    let hi = rev_bytes(v >> 16);
    (lo << 16) | hi
}

/// Reverse all 32 bits of `v`.
fn rev_word(v: u32) -> u32 {
    v.reverse_bits()
}

/// Mirror the low 8 bits.
fn rev_byte(b: u32) -> u32 {
    rev_bytes(b) & 0xFF
}

/// Mirror an `n`-bit value (`n <= 32`) as one unit.
fn mirror_n(v: u32, n: u8) -> u32 {
    let mut r = 0u32;
    for i in 0..n {
        r = (r << 1) | ((v >> i) & 1);
    }
    r
}

/// Mirror an `n`-bit group value (`n` a multiple of 8, first-fed bit at
/// top) per `REV_IN` mode: bytes, 16-bit halves (short groups fall back
/// to their own width), or the whole group.
fn mirror_group(v: u32, nbits: u8, mode: u8) -> u32 {
    match mode {
        1 => {
            let mut r = 0u32;
            let n = nbits / 8;
            for i in 0..n {
                r |= rev_byte((v >> (i * 8)) & 0xFF) << (i * 8);
            }
            r
        }
        2 => {
            if nbits <= 16 {
                mirror_n(v, nbits)
            } else {
                let mut r = 0u32;
                let n = nbits / 16;
                for i in 0..n {
                    r |= mirror_n((v >> (i * 16)) & 0xFFFF, 16) << (i * 16);
                }
                r |= (v >> (n * 16)) & ((1u32 << (nbits - n * 16)) - 1);
                r
            }
        }
        3 => mirror_n(v, nbits),
        _ => v,
    }
}

#[derive(Debug)]
struct CrcBehavior {
    crc: u32,
    /// Cached `INIT` value (reloaded by `RESET`); follows `INIT` writes,
    /// restored to `0xFFFF_FFFF` on system reset like the register.
    init: u32,
    dr_off: u32,
    cr_off: u32,
    pol_off: Option<u32>,
    init_off: Option<u32>,
    /// Lanes of the in-progress `DR` bus transaction (bit `i` = lane `i`
    /// written since the last fold). One transaction feeds one group.
    group_mask: u8,
}

impl CrcBehavior {
    /// Polynomial size from `CR.POLYSIZE` (00 = 32, 01 = 16, 10 = 8,
    /// 11 = 7); fixed 32 without the field.
    fn poly_size(view: &Registers<&mut [u32]>, cr_off: u32) -> u8 {
        match (view.get(cr_off) >> 3) & 0x3 {
            1 => 16,
            2 => 8,
            3 => 7,
            _ => 32,
        }
    }

    /// Whether this table declares a programmable unit (has `POL`).
    fn programmable(&self) -> bool {
        self.pol_off.is_some()
    }

    fn poly(&self, view: &Registers<&mut [u32]>, size: u8) -> u32 {
        let raw = match self.pol_off {
            Some(off) => view.get(off),
            None => 0x04C1_1DB7,
        };
        if size >= 32 {
            raw
        } else {
            raw & ((1 << size) - 1)
        }
    }

    fn rev_in(view: &Registers<&mut [u32]>, cr_off: u32) -> u8 {
        ((view.get(cr_off) >> 5) & 0x3) as u8
    }

    fn rev_out(view: &Registers<&mut [u32]>, cr_off: u32) -> bool {
        view.get(cr_off) & (1 << 7) != 0
    }

    /// Fold a complete lane group into the CRC, MSB-first: lanes descend
    /// (a word feeds its high byte first, like silicon), each byte
    /// MSB-first, mirrored per `REV_IN`, streamed through the `S`-bit
    /// engine in `S`-bit chunks (a short tail feeds left-aligned).
    fn flush_group(&mut self, view: &mut Registers<&mut [u32]>) {
        if self.group_mask == 0 {
            return;
        }
        let w = view.get(self.dr_off);
        let mut value = 0u32;
        let mut nbits = 0u8;
        for lane in [3u8, 2, 1, 0] {
            if self.group_mask & (1 << lane) != 0 {
                value = (value << 8) | ((w >> (u32::from(lane) * 8)) & 0xFF);
                nbits += 8;
            }
        }
        self.group_mask = 0;
        let size = if self.programmable() {
            Self::poly_size(view, self.cr_off)
        } else {
            32
        };
        let rev_in = if self.programmable() {
            Self::rev_in(view, self.cr_off)
        } else {
            0
        };
        let value = mirror_group(value, nbits, rev_in);
        let mask: u32 = if size >= 32 {
            u32::MAX
        } else {
            (1u32 << size) - 1
        };
        let poly = self.poly(view, size);
        let top: u32 = if size >= 32 {
            0x8000_0000
        } else {
            1u32 << (size - 1)
        };
        let mut rem = nbits;
        let mut v = value;
        while rem > 0 {
            let take = rem.min(size);
            let chunk = v >> (rem - take);
            v &= if rem == take {
                0
            } else {
                (1u32 << (rem - take)) - 1
            };
            let mut cur = (self.crc ^ (chunk << (size - take))) & mask;
            for _ in 0..take {
                cur = if cur & top != 0 {
                    ((cur << 1) ^ poly) & mask
                } else {
                    (cur << 1) & mask
                };
            }
            self.crc = cur;
            rem -= take;
        }
        // Note: the computed `crc` is *not* stored back here — `DR` keeps
        // the raw latched lanes for group assembly, and reads resolve
        // through `on_read` below.
    }
}

impl Behavior for CrcBehavior {
    fn needs_tick() -> bool {
        true
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        // Time passing ends the bus transaction: fold pending lanes.
        self.flush_group(view);
    }
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == self.dr_off {
            // A lane repeating starts a new transaction: fold the old one
            // first (firmware wrote the whole group already).
            if self.group_mask & (1 << (shift / 8)) != 0 {
                self.flush_group(view);
            }
            let word = view.merged(offset, val, shift);
            self.group_mask |= 1 << (shift / 8);
            return word;
        }
        // Anything else ends the transaction: fold with the old config
        // before touching configuration or status.
        self.flush_group(view);
        if Some(offset) == self.pol_off || Some(offset) == self.init_off {
            // `POL`/`INIT` are plain storage; `INIT` shadows into the
            // reload cache (a `RESET` loads it, a system reset restores
            // the default through `reset()` below).
            let word = view.merged(offset, val, shift);
            if Some(offset) == self.init_off {
                self.init = word;
            }
            return word;
        }
        if offset == self.cr_off {
            // Bit 0 `RESET`: reload `INIT` (or `0xFFFF_FFFF`), self-clears;
            // the configuration bits stick.
            let word = view.merged(offset, val, shift);
            if word & 1 != 0 {
                self.crc = match self.init_off {
                    Some(off) => view.get(off),
                    None => 0xFFFF_FFFF,
                };
                view.set(self.dr_off, self.crc);
                return word & !1;
            }
            return word;
        }
        view.merged(offset, val, shift)
    }
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, _value: u32) -> u32 {
        self.flush_group(view);
        if offset == self.dr_off {
            // Reads resolve the computed CRC (the stored lanes are just
            // the assembly buffer); `REV_OUT` mirrors the whole word.
            if self.programmable() && Self::rev_out(view, self.cr_off) {
                return self.crc.reverse_bits();
            }
            return self.crc;
        }
        view.get(offset)
    }
    fn reset(&mut self) {
        self.crc = 0xFFFF_FFFF;
        self.init = 0xFFFF_FFFF;
        self.group_mask = 0;
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
        // Plain names plus `CRC_`-prefixed variants (WBA-style tables).
        let off = |names: &[&str], fb: u32| {
            names
                .iter()
                .find_map(|n| def.registers.iter().find(|r| r.name == *n))
                .map(|r| r.offset)
                .unwrap_or(fb)
        };
        let maybe = |names: &[&str]| {
            names
                .iter()
                .find_map(|n| def.registers.iter().find(|r| r.name == *n))
                .map(|r| r.offset)
        };
        let behavior = CrcBehavior {
            crc: 0xFFFF_FFFF,
            init: 0xFFFF_FFFF,
            dr_off: off(&["DR", "CRC_DR"], 0x00),
            cr_off: off(&["CR", "CRC_CR"], 0x08),
            pol_off: maybe(&["POL", "CRC_POL"]),
            init_off: maybe(&["INIT", "CRC_INIT"]),
            group_mask: 0,
        };
        let mut rf = RegisterFile::new(def, behavior);
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

    /// Byte store (lane 0): feeds 8 bits.
    fn write_byte(c: &mut Crc, addr: u32, val: u8) {
        c.write(addr, val).unwrap();
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

    /// CRC-32/MPEG-2 check value: "123456789" -> 0x0376E6E7 (fixed unit,
    /// MSByte-first packing like silicon expects, byte tail for odd length).
    #[test]
    fn mpeg2_check_value_is_exact() {
        let mut c = Crc::new_for::<Stm32f103>();
        for w in [0x3132_3334u32, 0x3536_3738] {
            write_word(&mut c, BASE, w);
        }
        write_byte(&mut c, BASE, 0x39);
        assert_eq!(read_word(&mut c, BASE), 0x0376_E6E7);
    }

    /// CRC-32/ISO-HDLC check value: "123456789" -> 0xCBF43926 (reflected
    /// in and out, firmware xors the readback like silicon requires).
    #[test]
    fn isohdlc_check_value_is_exact() {
        use crate::device::svd::stm32h743::Stm32h743;
        let mut c = Crc::new_for::<Stm32h743>();
        let base = c.ranges()[0].base;
        // REV_IN = byte (0x20), REV_OUT (0x80), 32-bit poly.
        write_word(&mut c, base + 0x08, 0xA0);
        write_word(&mut c, base + 0x08, 0xA1); // + RESET
        for w in [0x3132_3334u32, 0x3536_3738] {
            write_word(&mut c, base, w);
        }
        write_byte(&mut c, base, 0x39);
        assert_eq!(read_word(&mut c, base) ^ 0xFFFF_FFFF, 0xCBF4_3926);
    }

    /// CRC-16/CCITT-FALSE check value: "123456789" -> 0x29B1 (poly
    /// 0x1021, 16-bit, init FFFF, fed byte-wise like silicon expects).
    #[test]
    fn ccitt_false_check_value_is_exact() {
        use crate::device::svd::stm32h743::Stm32h743;
        let mut c = Crc::new_for::<Stm32h743>();
        let base = c.ranges()[0].base;
        write_word(&mut c, base + 0x14, 0x1021); // POL
        write_word(&mut c, base + 0x08, 0x08 | 0x01); // POLYSIZE 16 + RESET
        for w in 0x31u8..=0x39 {
            write_byte(&mut c, base, w);
        }
        assert_eq!(read_word(&mut c, base) & 0xFFFF, 0x29B1);
    }

    /// CRC-8 (poly 0x07, init 0) check value: "123456789" -> 0xF4, and
    /// CRC-7/MMC (poly 0x09, init 0) -> 0x75.
    #[test]
    fn small_poly_check_values_are_exact() {
        use crate::device::svd::stm32h743::Stm32h743;
        let mut c = Crc::new_for::<Stm32h743>();
        let base = c.ranges()[0].base;
        write_word(&mut c, base + 0x10, 0); // INIT = 0
        write_word(&mut c, base + 0x14, 0x07); // POL
        write_word(&mut c, base + 0x08, 0x10 | 0x01); // POLYSIZE 8 + RESET
        for w in 0x31u8..=0x39 {
            write_byte(&mut c, base, w);
        }
        assert_eq!(read_word(&mut c, base) & 0xFF, 0xF4);
        write_word(&mut c, base + 0x14, 0x09); // POL
        write_word(&mut c, base + 0x08, 0x18 | 0x01); // POLYSIZE 7 + RESET
        for w in 0x31u8..=0x39 {
            write_byte(&mut c, base, w);
        }
        assert_eq!(read_word(&mut c, base) & 0x7F, 0x75);
    }

    /// `RESET` reloads `INIT` (not just all-ones) and keeps the config.
    #[test]
    fn reset_reloads_init_and_keeps_config() {
        use crate::device::svd::stm32h743::Stm32h743;
        let mut c = Crc::new_for::<Stm32h743>();
        let base = c.ranges()[0].base;
        write_word(&mut c, base + 0x10, 0x1234_5678); // INIT
        write_word(&mut c, base + 0x08, 0x01); // RESET only
        assert_eq!(read_word(&mut c, base), 0x1234_5678);
        write_word(&mut c, base + 0x08, 0xA0); // REV config sticks
        assert_eq!(read_word(&mut c, base + 0x08) & 0xA0, 0xA0, "config kept");
    }
}
