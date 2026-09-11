use super::radio::PacketBehavior;
use crate::device::svd::{Access, FieldDef, IrqDef, PeripheralDef, RegisterDef, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// STM32WL `SUBGHZ` radio: `CR` enable + `READY` status IRQ (the original
/// stub surface, preserved bit-for-bit), plus the virtual packet endpoint
/// (`TXLEN`/`TXDATA`/`TXGO`/`RXSTAT`/`RXDATA`, see
/// [`PacketBehavior`]) — real WL packets move
/// through `SUBGHZSPI` commands, a whole protocol out of scope.
#[derive(Debug)]
pub struct Subghz {
    inner: RegisterFile<PacketBehavior>,
    name: &'static str,
}

impl Subghz {
    /// A new `Subghz` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: crate::device::svd::Device>() -> Self {
        let def = D::peripheral("SUBGHZSPI")
            .or_else(|| D::peripheral("SUBGHZ"))
            .unwrap_or_else(|| panic!("generated tables contain SUBGHZSPI/SUBGHZ"));
        Self {
            inner: RegisterFile::new(def, PacketBehavior::new(0x10)),
            name: "subghz",
        }
    }

    /// A new window, or `None` when the tables lack it.
    #[must_use]
    pub fn new_for_opt<D: crate::device::svd::Device>() -> Option<Self> {
        let def = D::peripheral("SUBGHZSPI").or_else(|| D::peripheral("SUBGHZ"))?;
        Some(Self {
            inner: RegisterFile::new(def, PacketBehavior::new(0x10)),
            name: "subghz",
        })
    }

    /// Manual WL SUBGHZ at `0x58001400` when SVD lacks the entry (WL alias boards).
    #[must_use]
    pub fn new_wl() -> Self {
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
                // Virtual packet endpoint (see `radio.rs`).
                RegisterDef {
                    name: "TXLEN",
                    offset: 0x20,
                    width: 32,
                    access: Access::ReadWrite,
                    reset: 0,
                    fields: &[],
                },
                RegisterDef {
                    name: "TXDATA",
                    offset: 0x24,
                    width: 32,
                    access: Access::ReadWrite,
                    reset: 0,
                    fields: &[],
                },
                RegisterDef {
                    name: "TXGO",
                    offset: 0x28,
                    width: 32,
                    access: Access::ReadWrite,
                    reset: 0,
                    fields: &[],
                },
                RegisterDef {
                    name: "RXSTAT",
                    offset: 0x2c,
                    width: 32,
                    access: Access::ReadOnly,
                    reset: 0,
                    fields: &[],
                },
                RegisterDef {
                    name: "RXDATA",
                    offset: 0x30,
                    width: 32,
                    access: Access::ReadOnly,
                    reset: 0,
                    fields: &[],
                },
            ],
        };
        Self {
            inner: RegisterFile::new(&WL_SUBGHZ, PacketBehavior::new(0x10)),
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
    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }
    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
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
