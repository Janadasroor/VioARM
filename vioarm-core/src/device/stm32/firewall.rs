//! L4 firewall (`FIREWALL@0x40011c00`) — code/data segment gate.
//!
//! The segment descriptors (`CSSA`/`CSL`, `NVDSSA`/`NVDSL`, `VDSSA`/`VDSL`)
//! and `CR.VDE` latch while the firewall is disarmed; setting `CR.FPA`
//! (pre-arm) locks them — further descriptor writes are ignored until
//! reset, exactly like silicon freezing the configuration at arm time.
//! `CR.VDS` (violation flag) reads 0: the bus carries no illegal access,
//! so no violation can occur (documented). Illegal-access trapping itself
//! stays out of scope: the firewall watches bus masters the simulator
//! does not arbitrate.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR: u32 = 0x20;
const FPA: u32 = 1 << 0;
const VDS: u32 = 1 << 1;

#[derive(Clone, Debug, Default)]
struct FirewallBehavior;

impl FirewallBehavior {
    fn armed(view: &Registers<&mut [u32]>) -> bool {
        view.get(CR) & FPA != 0
    }
}

impl Behavior for FirewallBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        // Armed configuration is frozen: every write is ignored until
        // reset (and `FPA`, once set, never clears).
        if Self::armed(view) {
            return view.get(offset);
        }
        let mut new = view.merged(offset, val, shift);
        if offset == CR {
            new |= view.get(CR) & FPA;
        }
        new
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == CR {
            let _ = view;
            return value & !VDS;
        }
        value
    }

    fn needs_tick() -> bool {
        false
    }
}

/// L4 firewall.
#[derive(Debug)]
pub struct Firewall {
    inner: RegisterFile<FirewallBehavior>,
}

impl Firewall {
    /// A new `FIREWALL` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `FIREWALL`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("FIREWALL").expect("generated tables contain FIREWALL");
        Self {
            inner: RegisterFile::new(def, FirewallBehavior),
        }
    }
}

impl Peripheral for Firewall {
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
        "firewall"
    }
}

impl InterruptSource for Firewall {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, _lines: u32) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32l476::Stm32l476;
    use crate::device::Peripheral;

    const BASE: u32 = 0x4001_1C00;
    const CSSA: u32 = 0x00;
    const CSL: u32 = 0x04;
    const CR_O: u32 = 0x20;

    #[allow(clippy::cast_possible_truncation)]
    fn w(f: &mut Firewall, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            f.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(f: &mut Firewall, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = f.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn firewall() -> Firewall {
        let mut f = Firewall::new_for::<Stm32l476>();
        f.reset();
        f
    }

    #[test]
    fn table_matches_silicon_layout() {
        let def = Stm32l476::peripheral("FIREWALL").unwrap();
        assert_eq!(def.base, BASE, "FIREWALL base");
        for (name, offset) in [
            ("CSSA", 0x00),
            ("CSL", 0x04),
            ("NVDSSA", 0x08),
            ("NVDSL", 0x0C),
            ("VDSSA", 0x10),
            ("VDSL", 0x14),
            ("CR", 0x20),
        ] {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("FIREWALL:{name} present"));
            assert_eq!(r.offset, offset, "FIREWALL:{name} offset");
        }
    }

    #[test]
    fn arm_freezes_configuration() {
        let mut f = firewall();
        w(&mut f, CSSA, 0x0800_0000);
        w(&mut f, CSL, 0x0000_0100);
        assert_eq!(r(&mut f, CSSA), 0x0800_0000, "segments latch pre-arm");
        w(&mut f, CR_O, FPA);
        assert_ne!(r(&mut f, CR_O) & FPA, 0, "FPA arms");
        w(&mut f, CSSA, 0xDEAD_BEEF);
        assert_eq!(r(&mut f, CSSA), 0x0800_0000, "frozen while armed");
        w(&mut f, CR_O, 0);
        assert_ne!(r(&mut f, CR_O) & FPA, 0, "FPA never clears");
        assert_eq!(r(&mut f, CR_O) & VDS, 0, "no violations observed");
        f.reset();
        w(&mut f, CSSA, 0xDEAD_BEEF);
        assert_eq!(r(&mut f, CSSA), 0xDEAD_BEEF, "reset disarms");
    }
}
