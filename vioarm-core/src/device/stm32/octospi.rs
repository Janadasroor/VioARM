use crate::device::stm32::fmc::QspiGate;
use crate::device::svd::{Behavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// STM32H5/U5 `OCTOSPI` — stub real behavior (EN + SR) with `QspiGate` for
/// the memory-mapped window (`0x90000000` for `OCTOSPI1`, `0x70000000` for
/// `OCTOSPI2` on `H723`).
#[derive(Debug)]
struct OctospiBehavior {
    enabled: bool,
    gate: Option<Arc<Mutex<QspiGate>>>,
}

impl OctospiBehavior {
    const fn new(gate: Option<Arc<Mutex<QspiGate>>>) -> Self {
        Self {
            enabled: false,
            gate,
        }
    }
}

impl Behavior for OctospiBehavior {
    // Gated-window access is synchronous; the empty tick is removed.
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
        if offset == 0x00 {
            let en = word & 1 != 0;
            self.enabled = en;
            if en {
                view.set(0x10, view.get(0x10) | 1);
            }
            if let Some(g) = &self.gate {
                g.lock().unwrap_or_else(|e| e.into_inner()).enabled = en;
            }
        }
        word
    }
    fn reset(&mut self) {
        self.enabled = false;
        if let Some(g) = &self.gate {
            g.lock().unwrap_or_else(|e| e.into_inner()).reset();
        }
    }
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
            inner: RegisterFile::new(def, OctospiBehavior::new(None)),
            name: "octospi",
        }
    }
    #[must_use]
    pub fn new_for_with_gate<D: crate::device::svd::Device>(
        name: &str,
        gate: Option<Arc<Mutex<QspiGate>>>,
    ) -> Self {
        if let Some(def) = D::peripheral(name) {
            return Self {
                inner: RegisterFile::new(def, OctospiBehavior::new(gate)),
                name: "octospi",
            };
        }
        // Fallback manual def for chips where SVD lacks OCTOSPI (e.g., H743 alias)
        use crate::device::svd::{Access, FieldDef, IrqDef, PeripheralDef, RegisterDef};
        const FALLBACK_OCTOSPI1: PeripheralDef = PeripheralDef {
            name: "OCTOSPI1",
            base: 0x5200_5000,
            window: 0x400,
            irqs: &[IrqDef {
                number: 92,
                name: "OCTOSPI1",
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
        const FALLBACK_OCTOSPI2: PeripheralDef = PeripheralDef {
            name: "OCTOSPI2",
            base: 0x5200_A000,
            window: 0x400,
            irqs: &[IrqDef {
                number: 93,
                name: "OCTOSPI2",
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
        let fallback = if name == "OCTOSPI2" {
            &FALLBACK_OCTOSPI2
        } else {
            &FALLBACK_OCTOSPI1
        };
        Self {
            inner: RegisterFile::new(fallback, OctospiBehavior::new(gate)),
            name: "octospi",
        }
    }
    #[must_use]
    pub fn octospi1_for<D: crate::device::svd::Device>() -> Self {
        Self::new_for::<D>("OCTOSPI1")
    }
    #[must_use]
    pub fn octospi1_for_with_gate<D: crate::device::svd::Device>(
        gate: Option<Arc<Mutex<QspiGate>>>,
    ) -> Self {
        Self::new_for_with_gate::<D>("OCTOSPI1", gate)
    }
    #[must_use]
    pub fn octospi2_for<D: crate::device::svd::Device>() -> Self {
        Self::new_for::<D>("OCTOSPI2")
    }
    #[must_use]
    pub fn octospi2_for_with_gate<D: crate::device::svd::Device>(
        gate: Option<Arc<Mutex<QspiGate>>>,
    ) -> Self {
        Self::new_for_with_gate::<D>("OCTOSPI2", gate)
    }

    #[must_use]
    pub fn new_wl() -> Self {
        use crate::device::svd::{Access, FieldDef, IrqDef, PeripheralDef, RegisterDef};
        const WL_OCTOSPI: PeripheralDef = PeripheralDef {
            name: "OCTOSPI1",
            base: 0x5800_4000,
            window: 0x400,
            irqs: &[IrqDef {
                number: 42,
                name: "OCTOSPI1",
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
            inner: RegisterFile::new(&WL_OCTOSPI, OctospiBehavior::new(None)),
            name: "octospi",
        }
    }
}

impl Peripheral for Octospi {
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

impl InterruptSource for Octospi {
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
