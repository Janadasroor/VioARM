use crate::device::svd::{Behavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// STM32U5 `OTFDEC` — stub (key latch + status).
#[derive(Clone, Debug)]
struct OtfdecBehavior {
    enabled: bool,
}

impl Behavior for OtfdecBehavior {
    // Decrypt-on-read is synchronous; the empty tick is removed.
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
            view.set(0x10, view.get(0x10) | 1);
        }
        word
    }
}

#[derive(Debug)]
pub struct Otfdec {
    inner: RegisterFile<OtfdecBehavior>,
    name: &'static str,
}

impl Otfdec {
    #[must_use]
    pub fn new_for<D: crate::device::svd::Device>() -> Self {
        let def = D::peripheral("OTFDEC")
            .or_else(|| D::peripheral("OCTOSPIM"))
            .unwrap_or_else(|| panic!("generated tables contain OTFDEC/OCTOSPIM"));
        Self {
            inner: RegisterFile::new(def, OtfdecBehavior { enabled: false }),
            name: "otfdec",
        }
    }
}

impl Peripheral for Otfdec {
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

impl InterruptSource for Otfdec {
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
