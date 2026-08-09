//! The STM32F1 external interrupt controller (`EXTI`, `0x4001_0400`), driven
//! by the SVD-generated register tables.
//!
//! The full 16-line register file comes from the SVD (`IMR`/`EMR`/`RTSR`/
//! `FTSR`/`SWIER`/`PR`); the table cannot express the edge-detection logic, so
//! [`ExtiBehavior`] runs it. Hosts raise lines through the stimulus channel to
//! simulate a button or GPIO pin; software can also self-trigger via `SWIER`.
//! Pending bits are cleared the silicon way: writing 1 to `PR`. Only line 0 is
//! wired to an exception (IRQ 6), which the SVD lists as `EXTI0`.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Register offsets within the controller, per the SVD.
const IMR: u16 = 0x00;
const RTSR: u16 = 0x08;
const FTSR: u16 = 0x0C;
const SWIER: u16 = 0x10;
const PR: u16 = 0x14;

/// The edge-detection logic the SVD cannot express.
#[derive(Clone, Copy, Debug, Default)]
struct ExtiBehavior {
    /// The last sampled host input level, to edge against.
    level: u32,
    /// Whether `level` is the first sample; there is no prior level yet.
    level_valid: bool,
}

impl Behavior for ExtiBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u16,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            // SWIER: writing 1 to a line sets that line pending.
            SWIER => view.set(PR, view.get(PR) | (u32::from(val) << shift)),
            // PR is write-1-to-clear: only the written byte's bits clear.
            PR => return view.get(PR) & !(u32::from(val) << shift),
            _ => {}
        }
        view.merged(offset, val, shift)
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        // Only EXTI line 0 is wired; it asserts while masked and pending.
        u32::from(view.get(PR) & view.get(IMR) & 1 != 0)
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        if self.level_valid {
            let old = self.level;
            let rising = value & !old;
            let falling = old & !value;
            let pr = view.get(PR) | (rising & view.get(RTSR)) | (falling & view.get(FTSR));
            view.set(PR, pr);
        }
        // The first sample after reset only establishes the reference level;
        // there is no prior level to edge against.
        self.level = value;
        self.level_valid = true;
    }
}

/// The `EXTI` controller. Line 0 drives the interrupt line.
#[derive(Debug)]
pub struct Exti {
    inner: RegisterFile<ExtiBehavior>,
}

impl Exti {
    /// A new `EXTI` at its STM32F1 base address.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `EXTI`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        let def = stm32f103::peripheral("EXTI").expect("generated tables contain EXTI");
        Self {
            inner: RegisterFile::new(def, ExtiBehavior::default()),
        }
    }
}

impl Default for Exti {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Exti {
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
        "exti"
    }

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }
}

impl InterruptSource for Exti {
    fn exception_number(&self) -> Option<u8> {
        // The SVD groups several lines here; the model only wires line 0.
        self.inner.irq_exception("EXTI0")
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
    use super::{Exti, FTSR, IMR, PR, RTSR, SWIER};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const BASE: u32 = 0x4001_0400;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(e: &mut Exti, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            e.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(e: &mut Exti, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = e.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn rising_and_falling_edges() {
        let mut e = Exti::new();
        write_word(&mut e, BASE + u32::from(RTSR), 0b101); // rising on 0, 2
        write_word(&mut e, BASE + u32::from(FTSR), 0b010); // falling on 1
                                                           // The first sample only establishes the reference level.
        e.drive_inputs(0);
        assert_eq!(read_word(&mut e, BASE + u32::from(PR)) & 0b111, 0);
        // Rising on lines 0 and 2 (both configured) sets pending.
        e.drive_inputs(0b101);
        assert_eq!(read_word(&mut e, BASE + u32::from(PR)) & 0b111, 0b101);
        // Falling on 0,2 and rising on 1 are not configured: no change.
        e.drive_inputs(0b010);
        assert_eq!(read_word(&mut e, BASE + u32::from(PR)) & 0b111, 0b101);
        // Falling on line 1 (configured) now sets its bit.
        e.drive_inputs(0);
        assert_eq!(read_word(&mut e, BASE + u32::from(PR)) & 0b111, 0b111);
    }

    #[test]
    fn pending_requires_imr_and_line_zero() {
        let mut e = Exti::new();
        write_word(&mut e, BASE + u32::from(RTSR), 1);
        write_word(&mut e, BASE + u32::from(SWIER), 1); // software self-trigger line 0
        assert_eq!(e.pending(), 0); // not masked
        write_word(&mut e, BASE + u32::from(IMR), 1);
        assert_eq!(e.pending(), 1);
        // Clearing PR (write 1) drops the request.
        write_word(&mut e, BASE + u32::from(PR), 1);
        assert_eq!(e.pending(), 0);
    }

    #[test]
    fn pr_is_write_one_to_clear() {
        let mut e = Exti::new();
        write_word(&mut e, BASE + u32::from(RTSR), 1);
        e.drive_inputs(0); // establish reference level
        e.drive_inputs(1);
        assert_eq!(read_word(&mut e, BASE + u32::from(PR)) & 1, 1);
        write_word(&mut e, BASE + u32::from(PR), 0xFFFF_FFFF);
        assert_eq!(read_word(&mut e, BASE + u32::from(PR)), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut e = Exti::new();
        assert!(matches!(e.read(BASE + 0x18), Err(BusError::Reserved)));
        assert!(matches!(e.write(BASE - 1, 0), Err(BusError::Reserved)));
    }
}
