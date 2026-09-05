//! WBA BLE advertising window — a documented-virtual endpoint (no silicon
//! counterpart: real WBA BLE runs on CPU2 via IPCC, with no AP registers).
//!
//! Advertising bytes staged through `TXDATA` leave via `TXGO` for the host
//! channel (the [`Fabric`](crate::fabric::Fabric) radio bus carries them to
//! peer RX FIFOs); inbound bytes arrive the same way. `CR.LOOPBACK` copies
//! into the own RX FIFO for single-machine self-tests. No access address,
//! CRC, whitening, channels, or PHY exists here: payload bytes in, payload
//! bytes out, in order. There is deliberately no interrupt line either (no
//! silicon AP IRQ exists to mirror) — firmware polls `RXSTAT`/`TXDONE`.

use super::radio::PacketBehavior;
use crate::device::svd::{Access, PeripheralDef, RegisterDef, RegisterFile};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// A BLE advertising endpoint behind one address space. Shares the packet
/// engine (and register offsets) with the SUBGHZ packet endpoint.
#[derive(Debug)]
pub struct Ble {
    inner: RegisterFile<PacketBehavior>,
}

impl Ble {
    /// Manual BLE window at `0x5800_0000` (`Reserved` on WBA silicon today).
    /// No IRQ: firmware polls `RXSTAT`/`SR.TXDONE`.
    #[must_use]
    pub fn new_wba() -> Self {
        const WBA_BLE: PeripheralDef = PeripheralDef {
            name: "BLE",
            base: 0x5800_0000,
            window: 0x400,
            irqs: &[],
            registers: &[
                RegisterDef {
                    name: "CR",
                    offset: 0x00,
                    width: 32,
                    access: Access::ReadWrite,
                    reset: 0,
                    fields: &[],
                },
                RegisterDef {
                    name: "SR",
                    offset: 0x04,
                    width: 32,
                    access: Access::ReadWrite,
                    reset: 0,
                    fields: &[],
                },
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
            inner: RegisterFile::new(&WBA_BLE, PacketBehavior::new(0x04)),
        }
    }
}

impl Peripheral for Ble {
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
        self.inner.reset()
    }
    fn name(&self) -> &'static str {
        "ble"
    }
    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }
    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}

impl InterruptSource for Ble {
    fn exception_number(&self) -> Option<u8> {
        None
    }
    fn pending(&self) -> u32 {
        0
    }
    fn acknowledge(&mut self, _lines: u32) {}
}
