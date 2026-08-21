//! The STM32F1 general-purpose I/O port (`GPIOx`, `0x4001_0800` +), driven by
//! the SVD-generated register tables.
//!
//! Register layout and resets come from the SVD: `CRL`/`CRH` hold a 4-bit
//! config nibble per pin (`MODE[1:0] | CNF[1:0]`, `MODE == 0` means input,
//! resetting to all-float-input `0x4444_4444`). The two things a table cannot
//! express live in [`GpioBehavior`]: `BSRR`/`BRR` writes set/reset `ODR` (a
//! cross-register side effect), and read-only `IDR` is synthesized from the
//! config nibbles, the output latch, and the host-injected input level.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use core::ops::Deref;

/// Register offsets within the port, per the SVD.
const CRL: u32 = 0x00;
const CRH: u32 = 0x04;
const IDR: u32 = 0x08;
const ODR: u32 = 0x0C;
const BSRR: u32 = 0x10;
const BRR: u32 = 0x14;

/// The cross-register side effects the SVD cannot express.
#[derive(Clone, Copy, Debug, Default)]
struct GpioBehavior {
    input: u32,
}

impl GpioBehavior {
    /// Pin `p` (0..=15) config nibble: `MODE` in bits 0..1, `CNF` in 2..3.
    fn config<W>(view: &Registers<W>, p: u32) -> u32
    where
        W: Deref<Target = [u32]>,
    {
        let reg = if p < 8 { CRL } else { CRH };
        (view.get(reg) >> (4 * (p & 7))) & 0xF
    }

    /// The level firmware observes on `IDR`: output pins read their own
    /// latch, input pins read the host-driven level.
    fn idr<W>(self, view: &Registers<W>) -> u32
    where
        W: Deref<Target = [u32]>,
    {
        let mut v = 0;
        let mut p = 0;
        while p < 16 {
            // `MODE == 0` (the low two config bits clear) means input.
            let src = if Self::config(view, p).trailing_zeros() >= 2 {
                self.input
            } else {
                view.get(ODR)
            };
            if (src >> p) & 1 != 0 {
                v |= 1 << p;
            }
            p += 1;
        }
        v
    }
}

impl Behavior for GpioBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            BSRR => {
                // A byte in the low half sets pins `shift..shift+7`; in the
                // high half it resets pins `shift-16..shift-9`.
                let odr = view.get(ODR);
                if shift < 16 {
                    view.set(ODR, odr | (u32::from(val) << shift));
                } else {
                    view.set(ODR, odr & !(u32::from(val) << (shift - 16)));
                }
            }
            BRR => view.set(ODR, view.get(ODR) & !(u32::from(val) << shift)),
            _ => {}
        }
        view.merged(offset, val, shift)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == IDR {
            self.idr(view)
        } else {
            value
        }
    }

    fn reset(&mut self) {
        self.input = 0;
    }

    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        self.input = value;
    }
}

/// A 16-pin STM32F1 GPIO port.
#[derive(Debug)]
pub struct Gpio {
    inner: RegisterFile<GpioBehavior>,
    name: &'static str,
}

impl Gpio {
    /// A new port at the generated peripheral whose base is `base`, reported
    /// as `name`.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain a peripheral at
    /// `base` (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn at(base: u32, name: &'static str) -> Self {
        Self::at_in::<stm32f103::Stm32f103>(base, name)
    }

    /// A new port on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain a peripheral at `base`.
    #[must_use]
    pub fn at_in<D: Device>(base: u32, name: &'static str) -> Self {
        let def = D::by_base(base).expect("generated tables contain a peripheral at base");
        Self {
            inner: RegisterFile::new(def, GpioBehavior::default()),
            name,
        }
    }

    /// The current output latch, for host inspection.
    #[must_use]
    pub fn output(&self) -> u32 {
        self.inner.word(ODR)
    }
}

impl Peripheral for Gpio {
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
        self.name
    }

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }
}

impl InterruptSource for Gpio {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

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
        write_word(&mut g, BASE + u32::from(BSRR), 0x0000_000F); // set pins 0..3
        assert_eq!(g.output() & 0xF, 0xF);
        write_word(&mut g, BASE + u32::from(BSRR), 0x0000_0001 << 16); // reset pin 0
        assert_eq!(g.output() & 0xF, 0xE);
    }

    #[test]
    fn brr_resets_outputs() {
        let mut g = Gpio::at(BASE, "gpioa");
        write_word(&mut g, BASE + u32::from(BSRR), 0x0000_00FF);
        write_word(&mut g, BASE + 0x14, 0x0000_0081); // BRR pins 0, 7
        assert_eq!(g.output() & 0xFF, 0x7E);
    }

    #[test]
    fn idr_reflects_outputs_and_host_inputs() {
        let mut g = Gpio::at(BASE, "gpioa");
        // Pin 0 output push-pull, pin 1 input float.
        crh_set(&mut g, 8, 0b0011); // PA8 output
        crh_set(&mut g, 9, 0b0100); // PA9 input float
        write_word(&mut g, BASE + u32::from(BSRR), 1 << 8); // PA8 high
        g.drive_inputs(1 << 9); // PA9 held high by host
        assert_eq!(read_word(&mut g, BASE + u32::from(IDR)) & 0x0300, 0x0300);
    }

    #[test]
    fn firmware_writes_to_idr_are_ignored() {
        let mut g = Gpio::at(BASE, "gpioa");
        write_word(&mut g, BASE + u32::from(IDR), 0xFFFF_FFFF);
        assert_eq!(read_word(&mut g, BASE + u32::from(IDR)), 0);
    }

    #[test]
    fn reset_clears_all_registers() {
        let mut g = Gpio::at(BASE, "gpioa");
        write_word(&mut g, BASE + u32::from(BSRR), 0xFFFF_FFFF);
        write_word(&mut g, BASE + u32::from(ODR), 0xFFFF_FFFF);
        g.drive_inputs(0xFFFF_FFFF);
        g.reset();
        assert_eq!(read_word(&mut g, BASE + u32::from(ODR)), 0);
        assert_eq!(read_word(&mut g, BASE + u32::from(IDR)), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut g = Gpio::at(BASE, "gpioa");
        assert!(matches!(g.read(BASE + 0x1C), Err(BusError::Reserved)));
        assert!(matches!(g.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}
