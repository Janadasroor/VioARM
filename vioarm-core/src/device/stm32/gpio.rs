//! The STM32F1 general-purpose I/O port (`GPIOx`, `0x4001_0800` +).
//!
//! Real register layout: `CRL`/`CRH` hold a 4-bit config nibble per pin
//! (`MODE[1:0] | CNF[1:0]`, `MODE == 0` means input), `ODR` is the output
//! latch, `IDR` reads the pin levels, and `BSRR`/`BRR` set/reset outputs with a
//! single write. Input pins reflect host-injected levels; output pins read
//! back their own latch, matching silicon.

use crate::device::Peripheral;
use crate::error::BusError;
use crate::mem::bus::Region;

// Word indices into the 4-byte-aligned register file.
const W_CRL: u32 = 0;
const W_CRH: u32 = 1;
const W_IDR: u32 = 2;
const W_ODR: u32 = 3;
const W_BSRR: u32 = 4;
const W_BRR: u32 = 5;
const W_LCKR: u32 = 6;

/// Register offsets, used by tests to address the file directly.
#[cfg(test)]
const BSRR: u32 = 0x10;
#[cfg(test)]
const IDR: u32 = 0x08;
#[cfg(test)]
const ODR: u32 = 0x0C;

const WINDOW_LEN: u32 = 0x1C;

/// A 16-pin STM32F1 GPIO port.
#[derive(Debug, Clone)]
pub struct Gpio {
    ranges: [Region; 1],
    name: &'static str,
    crl: u32,
    crh: u32,
    odr: u32,
    input: u32,
    lckr: u32,
}

impl Gpio {
    /// A new port mapped at `base`, reported as `name`.
    #[must_use]
    pub const fn at(base: u32, name: &'static str) -> Self {
        Self {
            ranges: [Region::new(base, WINDOW_LEN)],
            name,
            crl: 0,
            crh: 0,
            odr: 0,
            input: 0,
            lckr: 0,
        }
    }

    /// The current output latch, for host inspection.
    #[must_use]
    pub const fn output(&self) -> u32 {
        self.odr
    }

    /// Pin `p` (0..=15) config nibble: `MODE` in bits 0..1, `CNF` in 2..3.
    const fn config(&self, p: u32) -> u32 {
        let reg = if p < 8 { self.crl } else { self.crh };
        (reg >> (4 * (p & 7))) & 0xF
    }

    /// The level firmware observes on `IDR`: output pins read their own
    /// latch, input pins read the host-driven level.
    const fn idr(&self) -> u32 {
        let mut v = 0;
        let mut p = 0;
        while p < 16 {
            // `MODE == 0` (the low two config bits clear) means input.
            let src = if self.config(p).trailing_zeros() >= 2 {
                self.input
            } else {
                self.odr
            };
            if (src >> p) & 1 != 0 {
                v |= 1 << p;
            }
            p += 1;
        }
        v
    }
}

impl Peripheral for Gpio {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let word = match off >> 2 {
            W_CRL => self.crl,
            W_CRH => self.crh,
            W_IDR => self.idr(),
            W_ODR => self.odr,
            // Write-only on silicon; reads are ignored there too.
            W_BSRR | W_BRR => 0,
            W_LCKR => self.lckr,
            _ => return Err(BusError::Reserved),
        };
        #[allow(clippy::cast_possible_truncation)]
        Ok((word >> (8 * (off & 3))) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if !self.ranges[0].contains(addr) {
            return Err(BusError::Reserved);
        }
        let off = addr - self.ranges[0].base;
        let shift = 8 * (off & 3);
        let mask = 0xFFu32 << shift;
        let bits = u32::from(val) << shift;
        match off >> 2 {
            W_CRL => self.crl = (self.crl & !mask) | bits,
            W_CRH => self.crh = (self.crh & !mask) | bits,
            W_ODR => self.odr = (self.odr & !mask) | bits,
            W_BSRR => {
                // Byte `b` sets pins `8b..8b+7` and resets pins `8b+16..8b+23`.
                if shift < 16 {
                    self.odr |= bits;
                } else {
                    self.odr &= !(bits >> 16);
                }
            }
            W_BRR => self.odr &= !bits,
            W_LCKR => self.lckr = (self.lckr & !mask) | bits,
            // IDR is host-driven; firmware writes are consumed without fault.
            W_IDR => {}
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.crl = 0;
        self.crh = 0;
        self.odr = 0;
        self.input = 0;
        self.lckr = 0;
    }

    fn name(&self) -> &'static str {
        self.name
    }

    fn drive_inputs(&mut self, value: u32) {
        self.input = value;
    }
}

impl crate::device::InterruptSource for Gpio {}

#[cfg(test)]
mod tests {
    use super::{Gpio, BSRR, IDR, ODR};
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4001_0800;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(g: &mut Gpio, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            g.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(g: &mut Gpio, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = g.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    /// `CRH` nibble for pin `p` (8..=15).
    fn crh_set(g: &mut Gpio, p: u32, nibble: u32) {
        let shift = 4 * (p - 8);
        let v = read_word(g, BASE + 0x04);
        write_word(g, BASE + 0x04, (v & !(0xF << shift)) | (nibble << shift));
    }

    #[test]
    fn bsrr_sets_and_resets_in_one_write() {
        let mut g = Gpio::at(BASE, "gpioa");
        write_word(&mut g, BASE + BSRR, 0x0000_000F); // set pins 0..3
        assert_eq!(g.output() & 0xF, 0xF);
        write_word(&mut g, BASE + BSRR, 0x0000_0001 << 16); // reset pin 0
        assert_eq!(g.output() & 0xF, 0xE);
    }

    #[test]
    fn brr_resets_outputs() {
        let mut g = Gpio::at(BASE, "gpioa");
        write_word(&mut g, BASE + BSRR, 0x0000_00FF);
        write_word(&mut g, BASE + 0x14, 0x0000_0081); // BRR pins 0, 7
        assert_eq!(g.output() & 0xFF, 0x7E);
    }

    #[test]
    fn idr_reflects_outputs_and_host_inputs() {
        let mut g = Gpio::at(BASE, "gpioa");
        // Pin 0 output push-pull, pin 1 input float.
        crh_set(&mut g, 8, 0b0011); // PA8 output
        crh_set(&mut g, 9, 0b0100); // PA9 input float
        write_word(&mut g, BASE + BSRR, 1 << 8); // PA8 high
        g.drive_inputs(1 << 9); // PA9 held high by host
        assert_eq!(read_word(&mut g, BASE + IDR) & 0x0300, 0x0300);
    }

    #[test]
    fn firmware_writes_to_idr_are_ignored() {
        let mut g = Gpio::at(BASE, "gpioa");
        write_word(&mut g, BASE + IDR, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut g, BASE + IDR), 0);
    }

    #[test]
    fn reset_clears_all_registers() {
        let mut g = Gpio::at(BASE, "gpioa");
        write_word(&mut g, BASE + BSRR, 0xFFFF_FFFF);
        write_word(&mut g, BASE + ODR, 0xFFFF_FFFF);
        g.drive_inputs(0xFFFF_FFFF);
        g.reset();
        assert_eq!(read_word(&mut g, BASE + ODR), 0);
        assert_eq!(read_word(&mut g, BASE + IDR), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut g = Gpio::at(BASE, "gpioa");
        assert!(matches!(g.read(BASE + 0x1C), Err(BusError::Reserved)));
        assert!(matches!(g.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}
