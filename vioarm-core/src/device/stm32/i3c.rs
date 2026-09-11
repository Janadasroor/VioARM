use crate::device::svd::{Behavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// STM32H5/U5 `I3C` — stub behavior (enable latch + `EVR` status).
#[derive(Clone, Debug)]
struct I3cBehavior {
    enabled: bool,
}

impl Behavior for I3cBehavior {
    // Latch-only controller window; no modeled bus timing.
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
        let word = view.merged(offset, val, shift);
        if offset == 0x00 && word & 1 != 0 {
            self.enabled = true;
            // set EVR ready bit
            let evr = view.get(0x10);
            view.set(0x10, evr | 1);
        }
        word
    }
    fn tick(&mut self, _view: &mut Registers<&mut [u32]>, _delta: u64) {}
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(self.enabled && view.get(0x10) & 1 != 0)
    }
}

/// Improved Inter-Integrated Circuit.
#[derive(Debug)]
pub struct I3c {
    inner: RegisterFile<I3cBehavior>,
    name: &'static str,
}

impl I3c {
    /// A new `I3c` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: crate::device::svd::Device>() -> Self {
        Self::for_device::<D>("I3C1", "i3c1")
    }
    fn for_device<D: crate::device::svd::Device>(peripheral: &str, name: &'static str) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(def, I3cBehavior { enabled: false }),
            name,
        }
    }
}

impl Peripheral for I3c {
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
    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
    fn name(&self) -> &'static str {
        self.name
    }
}

impl InterruptSource for I3c {
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
