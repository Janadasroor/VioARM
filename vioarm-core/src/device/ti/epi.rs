//! TI Tiva C External Peripheral Interface (`EPI0`, `0x400D_0000`).
//!
//! Register map and bit meanings follow the TI `hw_epi.h` + `epi.c`
//! oracle (BSD): `CFG.MODE` (0 = GP, 1 = SDRAM, 2 = HB8, 3 = HB16),
//! nonblocking reads via `RSIZEn`/`RADDRn` started by an `RPSTDn` count
//! write, `READFIFO0-7` pop + `RFIFOCNT`, `STAT` (`ACTIVE`/`NBRBUSY`/
//! `WBUSY`/`INITSEQ`/`XFEMPTY`), `RIS` (`RDRIS`/`WRRIS`/DMA/`ERRRIS`),
//! `MIS`, `EISC` W1C.
//!
//! Modeled (FSMC stance: configs + counters, no transfer engine —
//! there is no peer on the external bus): mode/config/timing/`ADDRMAP`
//! latches, `RPSTDn` posts `count` words of `0xFFFF_FFFF` (unconnected
//! bus floats high) into the 8-deep read FIFO with `RDRIS`, FIFO pops +
//! count synthesis, `STAT.ACTIVE` while a mode is selected +
//! `XFEMPTY`, `EISC` W1C, `IM` gating. Deliberate scope: mapped-window
//! reads/writes (the actual transfer engine), `WBUSY`/`NBRBUSY`
//! (instant completion), SDRAM init sequencing, DMA/error events,
//! `FIFOLVL` trigger selects (latched, unenforced).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;

/// `EPI0` base address.
pub const EPI_BASE: u32 = 0x400D_0000;

const CFG_OFF: u32 = 0x00;
const RSIZE0_OFF: u32 = 0x20;
const RADDR0_OFF: u32 = 0x24;
const RPSTD0_OFF: u32 = 0x28;
const RSIZE1_OFF: u32 = 0x30;
const RADDR1_OFF: u32 = 0x34;
const RPSTD1_OFF: u32 = 0x38;
const STAT_OFF: u32 = 0x60;
const RFIFOCNT_OFF: u32 = 0x6C;
const READFIFO_OFF: u32 = 0x70;
const READFIFO_END: u32 = 0x90;
const RIS_OFF: u32 = 0x214;
const MIS_OFF: u32 = 0x218;
const EISC_OFF: u32 = 0x21C;
const IM_REG_OFF: u32 = 0x210;

/// `CFG.MODE` values.
const MODE_MASK: u32 = 0x0F;
/// `STAT` bits (synthesized).
const STAT_ACTIVE: u32 = 1 << 0;
const STAT_XFEMPTY: u32 = 1 << 7;
/// Interrupt bits.
const RIS_RDRIS: u32 = 1 << 1;
/// Read FIFO depth (8 `READFIFO` aliases).
const RFIFO_DEPTH: usize = 8;

/// Behavior behind the EPI window.
#[derive(Debug, Default)]
pub struct EpiBehavior {
    /// Posted read words (unconnected bus: all-ones).
    rfifo: VecDeque<u32>,
    /// Latched interrupt flags.
    ris: u32,
}

impl Behavior for EpiBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        self.rfifo.clear();
        self.ris = 0;
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(self.ris & view.get(IM_REG_OFF) != 0)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            STAT_OFF => {
                let mut stat = value & !(STAT_ACTIVE | 0x30);
                if view.get(CFG_OFF) & MODE_MASK != 0 {
                    stat |= STAT_ACTIVE;
                }
                stat | STAT_XFEMPTY
            }
            RFIFOCNT_OFF => self.rfifo.len() as u32,
            READFIFO_OFF..READFIFO_END if offset % 4 == 0 => {
                self.rfifo.pop_front().unwrap_or(0xFFFF_FFFF)
            }
            RIS_OFF => self.ris,
            MIS_OFF => self.ris & view.get(IM_REG_OFF),
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
            return view.get(offset);
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        if offset == EISC_OFF {
            self.ris &= !word;
            view.set(offset, 0);
            return 0;
        }
        // `RPSTDn` count write (whole word) starts a nonblocking read.
        if (offset == RPSTD0_OFF || offset == RPSTD1_OFF) && shift == 24 {
            let count = (word & 0xFFF).min(0xFFF);
            for _ in 0..count {
                if self.rfifo.len() < RFIFO_DEPTH {
                    self.rfifo.push_back(0xFFFF_FFFF);
                }
            }
            if count > 0 {
                self.ris |= RIS_RDRIS;
            }
        }
        word
    }
}

/// A Tiva C EPI controller (configs + read FIFO; no transfer engine).
pub struct Epi {
    inner: RegisterFile<EpiBehavior>,
}

impl Epi {
    /// A new EPI controller on `D`'s tables (`"EPI0"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"EPI0"`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("EPI0").expect("generated tables contain EPI0");
        Self {
            inner: RegisterFile::new(def, EpiBehavior::default()),
        }
    }
}

impl Peripheral for Epi {
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

impl InterruptSource for Epi {
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

    const EPI0_BASE: u32 = 0x400D_0000;
    const IM_OFF: u32 = 0x210;

    fn epi() -> Epi {
        Epi::for_device::<Tm4c129xnczad>()
    }

    fn read_word(epi: &mut Epi, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(epi.read(EPI0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(epi: &mut Epi, offset: u32, value: u32) {
        for lane in 0..4 {
            epi.write(
                EPI0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// `CFG` mode latches; `STAT` synthesizes `ACTIVE` + `XFEMPTY`.
    #[test]
    fn mode_and_stat() {
        let mut epi = epi();
        assert_eq!(read_word(&mut epi, STAT_OFF) & STAT_ACTIVE, 0);
        write_word(&mut epi, CFG_OFF, 0x02); // HB8.
        assert_eq!(read_word(&mut epi, CFG_OFF) & MODE_MASK, 0x02);
        let stat = read_word(&mut epi, STAT_OFF);
        assert_eq!(stat & STAT_ACTIVE, STAT_ACTIVE);
        assert_eq!(stat & STAT_XFEMPTY, STAT_XFEMPTY);
        assert_eq!(stat & 0x30, 0); // Never busy.
    }

    /// `RPSTD0` posts words: `RFIFOCNT` + all-ones pops + `RDRIS`.
    #[test]
    fn nonblocking_read_posts() {
        let mut epi = epi();
        write_word(&mut epi, CFG_OFF, 0x03); // HB16.
        write_word(&mut epi, RSIZE0_OFF, 0x02);
        write_word(&mut epi, RADDR0_OFF, 0x1000);
        write_word(&mut epi, RPSTD0_OFF, 3);
        assert_eq!(read_word(&mut epi, RFIFOCNT_OFF), 3);
        assert_eq!(read_word(&mut epi, RIS_OFF) & RIS_RDRIS, RIS_RDRIS);
        for _ in 0..3 {
            assert_eq!(read_word(&mut epi, READFIFO_OFF), 0xFFFF_FFFF);
        }
        assert_eq!(read_word(&mut epi, RFIFOCNT_OFF), 0);
    }

    /// FIFO caps at 8; `EISC` W1C clears; `IM` gates the line.
    #[test]
    fn fifo_cap_and_gating() {
        let mut epi = epi();
        write_word(&mut epi, IM_OFF, RIS_RDRIS);
        write_word(&mut epi, RPSTD0_OFF, 20);
        assert_eq!(read_word(&mut epi, RFIFOCNT_OFF), 8);
        assert_eq!(epi.pending(), 1);
        write_word(&mut epi, EISC_OFF, RIS_RDRIS);
        assert_eq!(read_word(&mut epi, RIS_OFF) & RIS_RDRIS, 0);
        assert_eq!(epi.pending(), 0);
    }
}
