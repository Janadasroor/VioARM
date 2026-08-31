use crate::device::svd::{Behavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// STM32WL `SUBGHZ` radio — stub real behavior (enable + status IRQ).
#[derive(Clone, Debug)]
struct SubghzBehavior {
    enabled: bool,
}

impl Behavior for SubghzBehavior {
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
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(self.enabled && view.get(0x10) & 1 != 0)
    }
}

#[derive(Debug)]
pub struct Subghz {
    inner: RegisterFile<SubghzBehavior>,
    name: &'static str,
}

impl Subghz {
    #[must_use]
    pub fn new_for<D: crate::device::svd::Device>() -> Self {
        let def = D::peripheral("SUBGHZSPI")
            .or_else(|| D::peripheral("SUBGHZ"))
            .unwrap_or_else(|| panic!("generated tables contain SUBGHZSPI/SUBGHZ"));
        Self {
            inner: RegisterFile::new(def, SubghzBehavior { enabled: false }),
            name: "subghz",
        }
    }

    #[must_use]
    pub fn new_for_opt<D: crate::device::svd::Device>() -> Option<Self> {
        let def = D::peripheral("SUBGHZSPI").or_else(|| D::peripheral("SUBGHZ"))?;
        Some(Self {
            inner: RegisterFile::new(def, SubghzBehavior { enabled: false }),
            name: "subghz",
        })
    }

    /// Manual WL SUBGHZ at `0x58001400` when SVD lacks the entry (WL alias boards).
    #[must_use]
    pub fn new_wl() -> Self {
        use crate::device::svd::{Access, FieldDef, IrqDef, PeripheralDef, RegisterDef};
        const WL_SUBGHZ: PeripheralDef = PeripheralDef {
            name: "SUBGHZSPI",
            base: 0x5800_1400,
            window: 0x400,
            irqs: &[IrqDef {
                number: 42,
                name: "SUBGHZSPI",
            }],
            registers: &[
                RegisterDef {
                    name: "CR",
                    offset: 0x00,
                    width: 32,
                    access: Access::ReadWrite,
                    reset: 0,
                    fields: &[FieldDef {
                        name: "EN",
                        offset: 0,
                        width: 1,
                        access: Access::ReadWrite,
                    }],
                },
                RegisterDef {
                    name: "SR",
                    offset: 0x10,
                    width: 32,
                    access: Access::ReadWrite,
                    reset: 0,
                    fields: &[],
                },
            ],
        };
        Self {
            inner: RegisterFile::new(&WL_SUBGHZ, SubghzBehavior { enabled: false }),
            name: "subghz",
        }
    }
}

impl Peripheral for Subghz {
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

impl InterruptSource for Subghz {
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
