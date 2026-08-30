use crate::device::svd::{Behavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// STM32H5/U5 `OCTOSPI` — stub real behavior (EN + SR).
#[derive(Clone, Debug)]
struct OctospiBehavior {
    enabled: bool,
}

impl Behavior for OctospiBehavior {
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
            view.set(0x10, view.get(0x10) | 1);
        }
        word
    }
    fn tick(&mut self, _view: &mut Registers<&mut [u32]>, _delta: u64) {}
}

#[derive(Debug)]
pub struct Octospi {
    inner: RegisterFile<OctospiBehavior>,
    name: &'static str,
}

impl Octospi {
    #[must_use]
    pub fn new_for<D: crate::device::svd::Device>(name: &str) -> Self {
        let def = D::peripheral(name).unwrap_or_else(|| panic!("generated tables contain {name}"));
        Self {
            inner: RegisterFile::new(def, OctospiBehavior { enabled: false }),
            name: "octospi",
        }
    }
    #[must_use]
    pub fn octospi1_for<D: crate::device::svd::Device>() -> Self { Self::new_for::<D>("OCTOSPI1") }
    #[must_use]
    pub fn octospi2_for<D: crate::device::svd::Device>() -> Self { Self::new_for::<D>("OCTOSPI2") }
}

impl Peripheral for Octospi {
    fn ranges(&self) -> &[Region] { self.inner.ranges() }
    fn read(&mut self, addr: u32) -> Result<u8, BusError> { self.inner.read(addr) }
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> { self.inner.write(addr, val) }
    fn reset(&mut self) { self.inner.reset(); }
    fn tick(&mut self, delta: u64) { self.inner.tick(delta); }
    fn name(&self) -> &'static str { self.name }
}

impl InterruptSource for Octospi {
    fn exception_number(&self) -> Option<u8> { self.inner.exception_number() }
    fn pending(&self) -> u32 { self.inner.pending() }
    fn acknowledge(&mut self, lines: u32) { self.inner.acknowledge(lines); }
}
