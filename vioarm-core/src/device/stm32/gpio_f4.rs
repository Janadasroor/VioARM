//! The STM32F4 general-purpose I/O port (`GPIOx`, `0x4002_0000` +), driven by
//! the SVD-generated register tables.
//!
//! F4 splits pin configuration across `MODER` (a 2-bit mode per pin: input,
//! general output, alternate, analog), `OTYPER`, `OSPEEDR`, and `PUPDR`, with
//! `AFRL`/`AFRH` selecting the alternate function — unlike the F1 port's
//! single 4-bit `CRL`/`CRH` nibble. Layouts and resets come from the SVD; the
//! two things a table cannot express live in [`Gpio4Behavior`]: `BSRR` writes
//! set (low half) and reset (high half) `ODR` — there is no `BRR` on F4 — and
//! read-only `IDR` is synthesized from the mode nibbles, the output latch, and
//! the host-injected input level.

use crate::device::svd::stm32f407;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use core::ops::Deref;

/// Register offsets within the port, per the SVD.
const MODER: u32 = 0x00;
const IDR: u32 = 0x10;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;

/// The cross-register side effects the SVD cannot express.
#[derive(Clone, Copy, Debug, Default)]
struct Gpio4Behavior {
    input: u32,
}

impl Gpio4Behavior {
    /// Pin `p` (0..=15) `MODER` mode: 0 input, 1 general output, 2 alternate,
    /// 3 analog.
    fn mode<W>(view: &Registers<W>, p: u32) -> u32
    where
        W: Deref<Target = [u32]>,
    {
        (view.get(MODER) >> (2 * p)) & 3
    }

    /// The level firmware observes on `IDR`: output pins read their own latch,
    /// input and alternate pins read the host-driven level, analog reads 0.
    fn idr<W>(self, view: &Registers<W>) -> u32
    where
        W: Deref<Target = [u32]>,
    {
        let mut v = 0;
        let mut p = 0;
        while p < 16 {
            let src = match Self::mode(view, p) {
                0 | 2 => self.input,
                1 => view.get(ODR),
                _ => 0,
            };
            if (src >> p) & 1 != 0 {
                v |= 1 << p;
            }
            p += 1;
        }
        v
    }
}

impl Behavior for Gpio4Behavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == BSRR {
            // A byte in the low half sets pins `shift..shift+7`; in the high
            // half it resets pins `shift-16..shift-9`. There is no `BRR`.
            let odr = view.get(ODR);
            if shift < 16 {
                view.set(ODR, odr | (u32::from(val) << shift));
            } else {
                view.set(ODR, odr & !(u32::from(val) << (shift - 16)));
            }
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

/// A 16-pin STM32F4 GPIO port.
#[derive(Debug)]
pub struct GpioF4 {
    inner: RegisterFile<Gpio4Behavior>,
    name: &'static str,
}

impl GpioF4 {
    /// A new port at the generated peripheral whose base is `base`, reported
    /// as `name`.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain a peripheral at
    /// `base` (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn at(base: u32, name: &'static str) -> Self {
        Self::at_in::<stm32f407::Stm32f407>(base, name)
    }

    /// A new port on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain a peripheral at `base`.
    #[must_use]
    pub fn at_in<D: Device>(base: u32, name: &'static str) -> Self {
        let def = D::by_base(base).expect("generated tables contain a peripheral at base");
        Self {
            inner: RegisterFile::new(def, Gpio4Behavior::default()),
            name,
        }
    }

    /// The current output latch, for host inspection.
    #[must_use]
    pub fn output(&self) -> u32 {
        self.inner.word(ODR)
    }
}

impl Peripheral for GpioF4 {
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

impl InterruptSource for GpioF4 {
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
    use super::{GpioF4, BSRR, IDR, MODER, ODR};
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4002_0000;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(g: &mut GpioF4, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            g.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(g: &mut GpioF4, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = g.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn new_port() -> GpioF4 {
        GpioF4::at(BASE, "gpioa")
    }

    /// `MODER` 2-bit mode for pin `p`.
    fn set_mode(g: &mut GpioF4, p: u32, mode: u32) {
        let shift = 2 * p;
        let v = read_word(g, BASE + MODER);
        write_word(g, BASE + MODER, (v & !(3 << shift)) | (mode << shift));
    }

    #[test]
    fn bsrr_sets_and_resets_in_one_write() {
        let mut g = new_port();
        write_word(&mut g, BASE + BSRR, 0x0000_000F); // set pins 0..3
        assert_eq!(g.output() & 0xF, 0xF);
        write_word(&mut g, BASE + BSRR, 1 << 16); // reset pin 0
        assert_eq!(g.output() & 0xF, 0xE);
    }

    #[test]
    fn no_brr_register() {
        let mut g = new_port();
        write_word(&mut g, BASE + BSRR, 0x0000_00FF);
        // On F1 the `BRR` slot resets pins; on F4 that slot is `ODR`, a latch.
        write_word(&mut g, BASE + ODR, 0x0000_0000);
        assert_eq!(g.output(), 0);
    }

    #[test]
    fn idr_reflects_modes_and_host_inputs() {
        let mut g = new_port();
        set_mode(&mut g, 0, 1); // PA0 general output
        set_mode(&mut g, 1, 0); // PA1 input
        write_word(&mut g, BASE + BSRR, 1); // PA0 high
        g.drive_inputs(1 << 1); // PA1 held high by host
        assert_eq!(read_word(&mut g, BASE + IDR) & 3, 3);
    }

    #[test]
    fn analog_pins_read_zero() {
        let mut g = new_port();
        set_mode(&mut g, 0, 3); // PA0 analog
        set_mode(&mut g, 1, 1); // PA1 output
        write_word(&mut g, BASE + BSRR, 0x0000_0003);
        g.drive_inputs(0xFFFF);
        assert_eq!(read_word(&mut g, BASE + IDR) & 3, 2);
    }

    #[test]
    fn firmware_writes_to_idr_are_ignored() {
        let mut g = new_port();
        write_word(&mut g, BASE + IDR, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut g, BASE + IDR), 0);
    }

    #[test]
    fn reset_clears_all_registers() {
        let mut g = new_port();
        write_word(&mut g, BASE + BSRR, 0xFFFF_FFFF);
        write_word(&mut g, BASE + ODR, 0xFFFF_FFFF);
        g.drive_inputs(0xFFFF_FFFF);
        g.reset();
        assert_eq!(read_word(&mut g, BASE + ODR), 0);
        assert_eq!(read_word(&mut g, BASE + IDR), 0);
    }

    #[test]
    fn addresses_between_registers_are_reserved() {
        let mut g = new_port();
        // Registers run MODER..AFRH (0x00..0x28); the next slot is reserved.
        assert!(matches!(g.read(BASE + 0x28), Err(BusError::Reserved)));
        assert!(matches!(g.read(BASE - 1), Err(BusError::Reserved)));
    }
}
