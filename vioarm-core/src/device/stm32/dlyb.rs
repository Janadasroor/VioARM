//! H7 delay-block tuning (`DELAY_Block_SDMMC1/2`, `DELAY_Block_QUADSPI`).
//!
//! Each block is two registers: `CR` (`DEN` delay enable, `SEN` sampler
//! enable) and `CFGR` (`SEL` input phase, `UNIT` unit count, `LNG` line
//! length, `LNGF` length-valid flag). The tuning itself latches; the one
//! derived state is `LNGF`, which reads set exactly while the delay path
//! is enabled (`DEN && SEN`) — the valid-path indication firmware checks
//! after programming a tuning. Phase granularity has no functional effect
//! in simulation (sampling is ideal), which the documentation states
//! rather than invents.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR: u32 = 0x00;
const CFGR: u32 = 0x04;
const DEN: u32 = 1 << 0;
const SEN: u32 = 1 << 1;
const LNGF: u32 = 1 << 31;

#[derive(Clone, Debug, Default)]
struct DlybBehavior;

impl Behavior for DlybBehavior {
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == CFGR {
            // Length-valid follows the enables, like silicon reporting a
            // programmed path.
            if view.get(CR) & (DEN | SEN) == DEN | SEN {
                return value | LNGF;
            }
            return value & !LNGF;
        }
        value
    }

    fn needs_tick() -> bool {
        false
    }
}

/// An H7 delay block.
#[derive(Debug)]
pub struct Dlyb {
    inner: RegisterFile<DlybBehavior>,
    name: &'static str,
}

impl Dlyb {
    fn for_name<D: Device>(peripheral: &str, name: &'static str) -> Self {
        let def = D::peripheral(peripheral).expect("generated tables contain delay block");
        Self {
            inner: RegisterFile::new(def, DlybBehavior),
            name,
        }
    }

    /// The `DELAY_Block_SDMMC1` window.
    #[must_use]
    pub fn sdmmc1_for<D: Device>() -> Self {
        Self::for_name::<D>("DELAY_Block_SDMMC1", "dlyb_sdmmc1")
    }

    /// The `DELAY_Block_SDMMC2` window.
    #[must_use]
    pub fn sdmmc2_for<D: Device>() -> Self {
        Self::for_name::<D>("DELAY_Block_SDMMC2", "dlyb_sdmmc2")
    }

    /// The `DELAY_Block_QUADSPI` window.
    #[must_use]
    pub fn quadspi_for<D: Device>() -> Self {
        Self::for_name::<D>("DELAY_Block_QUADSPI", "dlyb_quadspi")
    }
}

impl Peripheral for Dlyb {
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
}

impl InterruptSource for Dlyb {
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
    use crate::device::svd::stm32h743::Stm32h743;
    use crate::device::Peripheral;

    const SDMMC1: u32 = 0x5200_8000;
    const QUADSPI: u32 = 0x5200_6000;
    const SDMMC2: u32 = 0x4802_2800;

    #[allow(clippy::cast_possible_truncation)]
    fn w(d: &mut Dlyb, base: u32, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            d.write(base + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(d: &mut Dlyb, base: u32, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = d.read(base + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn tables_match_silicon_layout() {
        for (name, base) in [
            ("DELAY_Block_SDMMC1", SDMMC1),
            ("DELAY_Block_SDMMC2", SDMMC2),
            ("DELAY_Block_QUADSPI", QUADSPI),
        ] {
            let def = Stm32h743::peripheral(name).unwrap();
            assert_eq!(def.base, base, "{name} base");
            for (reg, offset) in [("CR", 0x00), ("CFGR", 0x04)] {
                let r = def
                    .registers
                    .iter()
                    .find(|r| r.name == reg)
                    .unwrap_or_else(|| panic!("{name}:{reg} present"));
                assert_eq!(r.offset, offset, "{name}:{reg} offset");
            }
        }
    }

    #[test]
    fn lngf_follows_enables() {
        let mut d = Dlyb::sdmmc1_for::<Stm32h743>();
        d.reset();
        w(&mut d, SDMMC1, CFGR, 0x0342_1105);
        assert_eq!(r(&mut d, SDMMC1, CFGR) & LNGF, 0, "disabled: no flag");
        w(&mut d, SDMMC1, CR, DEN | SEN);
        assert_ne!(r(&mut d, SDMMC1, CFGR) & LNGF, 0, "enabled: valid");
        assert_eq!(
            r(&mut d, SDMMC1, CFGR) & 0x0FFF_FFFF,
            0x0342_1105 & 0x0FFF_FFFF,
            "tuning latches under the flag"
        );
        w(&mut d, SDMMC1, CR, DEN);
        assert_eq!(r(&mut d, SDMMC1, CFGR) & LNGF, 0, "sampler off clears");
    }

    #[test]
    fn all_three_windows_answer() {
        let mut d1 = Dlyb::sdmmc1_for::<Stm32h743>();
        let mut d2 = Dlyb::sdmmc2_for::<Stm32h743>();
        let mut dq = Dlyb::quadspi_for::<Stm32h743>();
        for (d, base) in [(&mut d1, SDMMC1), (&mut d2, SDMMC2), (&mut dq, QUADSPI)] {
            d.reset();
            w(d, base, CR, DEN | SEN);
            assert_ne!(r(d, base, CFGR) & LNGF, 0, "{base:#x} live");
        }
    }
}
