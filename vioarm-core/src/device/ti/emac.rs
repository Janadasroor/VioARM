//! TI Tiva C Ethernet MAC (`EMAC0`, `0x400E_C000`) — package 1: register
//! shell + MII.
//!
//! Register map and MII semantics follow the TI `hw_emac.h` oracle
//! (Synopsys DWC core: `MIIADDR` `MIIB`/`MIIW`/`CR`/`MII`/`PLA`,
//! `MIIDATA`, `CFG` `RE`/`TE`, MAC address registers, `STATUS`,
//! `RIS`/`IM`). The SVD carries no `<resetValue>`, so all words reset 0.
//!
//! Modeled: `MIIADDR` writes complete instantly (`MIIB` self-clears;
//! the EPHY is unmodeled so `MIIDATA` reads all-ones — MDIO floats
//! high, physically honest), config/address/filter/MMC/PTP/DMA words
//! latch, `STATUS` reads engine-idle. Deliberate scope (package 2):
//! descriptor DMA (`TXDLADDR`/`RXDLADDR`, `TXPOLLD`/`RXPOLLD`, `DMARIS`
//! and the whole TX/RX data path — needs bus-mailbox DMA like the H7
//! model), checksum offload, MMC counting, PTP time, magic-packet wake.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// `EMAC0` base address.
pub const EMAC_BASE: u32 = 0x400E_C000;

const MIIADDR_OFF: u32 = 0x10;
const MIIDATA_OFF: u32 = 0x14;
const STATUS_OFF: u32 = 0x24;
const RIS_OFF: u32 = 0x38;
const IM_OFF: u32 = 0x3C;

/// `MIIADDR.MIIB` (busy) — self-clearing on instant completion.
const MII_MIIB: u32 = 1 << 0;

/// Behavior behind the MAC window: latch everything, complete MII
/// instantly, report engines idle.
#[derive(Debug, Default)]
pub struct EmacBehavior;

impl Behavior for EmacBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            MIIADDR_OFF => value & !MII_MIIB,
            MIIDATA_OFF => 0xFFFF, // No PHY: MDIO floats high.
            STATUS_OFF => 0,       // Engines idle (no DMA yet).
            RIS_OFF => 0,          // No events without traffic.
            _ => value,
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == RIS_OFF {
            return view.get(offset); // Nothing latches; writes sink.
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        word
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let _ = view;
        0
    }
}

/// A Tiva C Ethernet MAC shell (package 1; DMA deferred).
pub struct Emac {
    inner: RegisterFile<EmacBehavior>,
}

impl Emac {
    /// A new MAC on `D`'s tables (`"EMAC0"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"EMAC0"`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("EMAC0").expect("generated tables contain EMAC0");
        Self {
            inner: RegisterFile::new(def, EmacBehavior),
        }
    }
}

impl Peripheral for Emac {
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

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for Emac {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c129xnczad::Tm4c129xnczad;

    const EMAC0_BASE: u32 = 0x400E_C000;
    const CFG_OFF: u32 = 0x00;
    const ADDR0L_OFF: u32 = 0x44;

    fn emac() -> Emac {
        Emac::for_device::<Tm4c129xnczad>()
    }

    fn read_word(emac: &mut Emac, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(emac.read(EMAC0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(emac: &mut Emac, offset: u32, value: u32) {
        for lane in 0..4 {
            emac.write(
                EMAC0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// `CFG` + MAC address latches; engines report idle and quiet.
    #[test]
    fn config_latches_and_idle() {
        let mut emac = emac();
        write_word(&mut emac, CFG_OFF, (1 << 2) | (1 << 3)); // RE + TE.
        assert_eq!(read_word(&mut emac, CFG_OFF) & 0x0C, 0x0C);
        write_word(&mut emac, ADDR0L_OFF, 0x1234_5678);
        assert_eq!(read_word(&mut emac, ADDR0L_OFF), 0x1234_5678);
        assert_eq!(read_word(&mut emac, STATUS_OFF), 0);
        assert_eq!(read_word(&mut emac, RIS_OFF), 0);
        assert_eq!(emac.pending(), 0);
    }

    /// MII completes instantly; no PHY reads all-ones.
    #[test]
    fn mii_completes() {
        let mut emac = emac();
        write_word(&mut emac, MIIADDR_OFF, 0x01); // MIIB + PHY 0/reg 0.
        assert_eq!(read_word(&mut emac, MIIADDR_OFF) & MII_MIIB, 0);
        assert_eq!(read_word(&mut emac, MIIDATA_OFF), 0xFFFF);
    }
}
