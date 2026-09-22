//! TI Tiva C EEPROM (`EEPROM`, 2 KiB, 32 blocks × 16 words).
//!
//! Random/sequential access follows the TM4C123GH6PM datasheet (SPMS376E
//! §8.2.4): `EEBLOCK`/`EEOFFSET` select (offset auto-increments and wraps
//! within the block on `EERDWRINC`), `EERDWR` reads instantly and programs
//! words, completion is synchronous (`WORKING` reads 0 after the write
//! lands — no timing modeled; documented) with `EEDONE` error bits
//! (`WRBUSY` on access-during-write, `NOPERM` on protected/hidden writes),
//! `EESUPP.START` erases the selected block, `EEHIDE` blocks read as
//! erased and reject writes, `EESIZE`/`EEPROMPP` constants. Protection
//! passwords (`EEPROT`/`EEPASSn`/`EEUNLOCK`) latch open (unmodeled;
//! documented): the default no-password state is readable+writable.
//!
//! Completion interrupts travel the Flash vector (`FCRIS.ERIS`, datasheet
//! §8.2.4): with `EEINT` set, a completed program/erase raises the shared
//! link the `FLASH_CTRL` behavior folds into `ERIS`. The EEPROM raises no
//! NVIC line of its own (the SVD carries no IRQ).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

const EESIZE_OFF: u32 = 0x000;
const EEBLOCK_OFF: u32 = 0x004;
const EEOFFSET_OFF: u32 = 0x008;
const EERDWR_OFF: u32 = 0x010;
const EERDWRINC_OFF: u32 = 0x014;
const EEDONE_OFF: u32 = 0x018;
const EESUPP_OFF: u32 = 0x01C;
const EEINT_OFF: u32 = 0x040;
const EEHIDE_OFF: u32 = 0x050;
const PP_OFF: u32 = 0xFC0;
/// `EEDONE` bits: `WRBUSY` 5, `NOPERM` 4, `WORKING` 0.
const DONE_WRBUSY: u32 = 1 << 5;
const DONE_NOPERM: u32 = 1 << 4;
const DONE_WORKING: u32 = 1 << 0;
/// `EESUPP.START` bit 0.
const SUPP_START: u32 = 1 << 0;
/// Geometry: 32 blocks × 16 words; erased words read all-ones.
const N_BLOCKS: usize = 32;
const WORDS_PER_BLOCK: usize = 16;
const ERASED: u32 = 0xFFFF_FFFF;

/// Nonzero datasheet resets stamped over the SVD zeros.
const RESETS: &[(u32, u32)] = &[(EESIZE_OFF, 0x0020_0200), (PP_OFF, 0x0000_001F)];

/// Behavior behind the EEPROM window (instant completion).
#[derive(Debug)]
pub struct EepromBehavior {
    blocks: [[u32; WORDS_PER_BLOCK]; N_BLOCKS],
    /// Sticky `EEDONE` error bits (reads 0 = success).
    done: u32,
    /// Hidden-block mask (`EEHIDE` state).
    hidden: u32,
    /// Shared Flash-vector completion link (set when `EEINT` enables it).
    flash_link: Arc<Mutex<bool>>,
}

impl EepromBehavior {
    fn block(&self, view: &Registers<&mut [u32]>) -> usize {
        (view.get(EEBLOCK_OFF) & 0x1F) as usize
    }

    fn offset(&self, view: &Registers<&mut [u32]>) -> usize {
        (view.get(EEOFFSET_OFF) & 0x0F) as usize
    }

    fn accessible(&self, block: usize) -> bool {
        block < N_BLOCKS && self.hidden & (1 << block) == 0
    }

    /// Program the selected word (instant completion, like silicon after
    /// its timing elapses — firmware only observes the poll loop).
    fn program(&mut self, view: &mut Registers<&mut [u32]>, value: u32, inc: bool) {
        let (block, mut offset) = (self.block(view), self.offset(view));
        if !self.accessible(block) {
            self.done |= DONE_NOPERM;
            return;
        }
        self.blocks[block][offset] = value;
        if inc {
            offset = (offset + 1) % WORDS_PER_BLOCK;
            view.set(EEOFFSET_OFF, offset as u32);
        }
        self.done = 0;
        if view.get(EEINT_OFF) & 0x01 != 0 {
            if let Ok(mut link) = self.flash_link.lock() {
                *link = true;
            }
        }
    }

    fn read_word(&self, view: &Registers<&mut [u32]>) -> u32 {
        let (block, offset) = (self.block(view), self.offset(view));
        if !self.accessible(block) {
            return ERASED;
        }
        self.blocks[block][offset]
    }
}

impl Behavior for EepromBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        self.done = 0;
        self.hidden = 0;
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            EERDWR_OFF | EERDWRINC_OFF => self.read_word(view),
            EEDONE_OFF => self.done,
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
        match offset {
            EERDWR_OFF | EERDWRINC_OFF => {
                let word = view.merged(offset, val, shift);
                view.set(offset, word);
                // Full-word stores program (firmware word-stores `EERDWR`,
                // like TivaWare); partial lanes stage without firing until
                // the word is whole — approximated by firing on lane 3.
                if shift == 24 {
                    self.program(view, word, offset == EERDWRINC_OFF);
                }
                word
            }
            EEDONE_OFF => view.get(offset),
            EESUPP_OFF => {
                let word = view.merged(offset, val, shift);
                if word & SUPP_START != 0 {
                    // Erase the selected block to the erased state.
                    let block = self.block(view);
                    if self.accessible(block) {
                        self.blocks[block] = [ERASED; WORDS_PER_BLOCK];
                        self.done = 0;
                    } else {
                        self.done |= DONE_NOPERM;
                    }
                    if view.get(EEINT_OFF) & 0x01 != 0 {
                        if let Ok(mut link) = self.flash_link.lock() {
                            *link = true;
                        }
                    }
                    view.set(offset, word & !SUPP_START);
                    return word & !SUPP_START;
                }
                view.set(offset, word);
                word
            }
            EEHIDE_OFF => {
                let word = view.merged(offset, val, shift);
                view.set(offset, word);
                self.hidden |= word & 0xFFFF_FFFE;
                word
            }
            _ => {
                let word = view.merged(offset, val, shift);
                view.set(offset, word);
                word
            }
        }
    }
}

/// A Tiva C EEPROM.
pub struct Eeprom {
    inner: RegisterFile<EepromBehavior>,
}

impl Eeprom {
    /// A new EEPROM on `D`'s tables, sharing the Flash-vector completion
    /// link with the `FLASH_CTRL` behavior.
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"EEPROM"`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str, flash_link: Arc<Mutex<bool>>) -> Self {
        let def = D::peripheral(name).expect("generated tables contain EEPROM");
        let mut eeprom = Self {
            inner: RegisterFile::new(
                def,
                EepromBehavior {
                    blocks: [[ERASED; WORDS_PER_BLOCK]; N_BLOCKS],
                    done: 0,
                    hidden: 0,
                    flash_link,
                },
            ),
        };
        eeprom.stamp_resets();
        eeprom
    }

    /// Stamp the nonzero datasheet resets over the SVD zeros.
    fn stamp_resets(&mut self) {
        let def = self.inner.def();
        let stamped: Vec<(usize, u32)> = RESETS
            .iter()
            .filter_map(|&(offset, value)| {
                def.registers
                    .iter()
                    .position(|r| r.offset == offset)
                    .map(|idx| (idx, value))
            })
            .collect();
        let words = self.inner.regs_mut();
        for (idx, value) in stamped {
            words[idx] = value;
        }
    }

    /// Read a raw word (test/host introspection).
    #[must_use]
    pub fn raw_word(&mut self, block: usize, offset: usize) -> u32 {
        self.inner.with_view(|_, b| {
            if block < N_BLOCKS && offset < WORDS_PER_BLOCK {
                b.blocks[block][offset]
            } else {
                ERASED
            }
        })
    }
}

impl Peripheral for Eeprom {
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
        // Contents survive reset (nonvolatile); error state clears.
        self.inner.reset();
        self.stamp_resets();
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for Eeprom {
    // No SVD IRQ: completion travels the Flash vector (`FCRIS.ERIS`).
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const EEPROM_BASE: u32 = 0x400A_F000;

    fn eeprom() -> (Eeprom, Arc<Mutex<bool>>) {
        let link = Arc::new(Mutex::new(false));
        (
            Eeprom::for_device::<Tm4c123gh6pm>("EEPROM", link.clone()),
            link,
        )
    }

    fn read_word(eeprom: &mut Eeprom, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(eeprom.read(EEPROM_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(eeprom: &mut Eeprom, offset: u32, value: u32) {
        for lane in 0..4 {
            eeprom
                .write(
                    EEPROM_BASE + offset + lane,
                    ((value >> (8 * lane)) & 0xFF) as u8,
                )
                .unwrap();
        }
    }

    /// Program/read round-trip with instant `WORKING`-clear and success.
    #[test]
    fn program_read_round_trip() {
        let (mut eeprom, _) = eeprom();
        write_word(&mut eeprom, EEBLOCK_OFF, 3);
        write_word(&mut eeprom, EEOFFSET_OFF, 5);
        write_word(&mut eeprom, EERDWR_OFF, 0xDEAD_BEEF);
        assert_eq!(read_word(&mut eeprom, EEDONE_OFF) & DONE_WORKING, 0);
        assert_eq!(read_word(&mut eeprom, EEDONE_OFF), 0);
        assert_eq!(read_word(&mut eeprom, EERDWR_OFF), 0xDEAD_BEEF);
        assert_eq!(eeprom.raw_word(3, 5), 0xDEAD_BEEF);
    }

    /// `EERDWRINC` advances and wraps the offset within the block.
    #[test]
    fn increment_wraps_in_block() {
        let (mut eeprom, _) = eeprom();
        write_word(&mut eeprom, EEBLOCK_OFF, 0);
        write_word(&mut eeprom, EEOFFSET_OFF, 15);
        write_word(&mut eeprom, EERDWRINC_OFF, 0x1111_1111);
        assert_eq!(read_word(&mut eeprom, EEOFFSET_OFF), 0);
        write_word(&mut eeprom, EERDWRINC_OFF, 0x2222_2222);
        assert_eq!(eeprom.raw_word(0, 15), 0x1111_1111);
        assert_eq!(eeprom.raw_word(0, 0), 0x2222_2222);
    }

    /// `START` erases the selected block; completion raises the Flash link
    /// when `EEINT` enables it.
    #[test]
    fn start_erases_and_links() {
        let (mut eeprom, link) = eeprom();
        write_word(&mut eeprom, EEBLOCK_OFF, 1);
        write_word(&mut eeprom, EEOFFSET_OFF, 0);
        write_word(&mut eeprom, EERDWR_OFF, 0x1234_5678);
        assert!(!*link.lock().unwrap()); // `EEINT` clear: no link.
        write_word(&mut eeprom, EEINT_OFF, 0x01);
        write_word(&mut eeprom, EESUPP_OFF, SUPP_START);
        assert_eq!(eeprom.raw_word(1, 0), ERASED);
        assert!(*link.lock().unwrap());
    }

    /// Hidden blocks read erased and reject writes with `NOPERM`.
    #[test]
    fn hidden_blocks_reject() {
        let (mut eeprom, _) = eeprom();
        write_word(&mut eeprom, EEHIDE_OFF, 0x02); // hide block 1
        write_word(&mut eeprom, EEBLOCK_OFF, 1);
        write_word(&mut eeprom, EEOFFSET_OFF, 0);
        assert_eq!(read_word(&mut eeprom, EERDWR_OFF), ERASED);
        write_word(&mut eeprom, EERDWR_OFF, 0x1234_5678);
        assert_eq!(
            read_word(&mut eeprom, EEDONE_OFF) & DONE_NOPERM,
            DONE_NOPERM
        );
    }

    /// Datasheet constants land on the bus.
    #[test]
    fn constants() {
        let (mut eeprom, _) = eeprom();
        assert_eq!(read_word(&mut eeprom, EESIZE_OFF), 0x0020_0200);
    }
}
