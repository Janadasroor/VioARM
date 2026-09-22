//! TI Tiva C 129-family flash memory controller (`FLASH_CTRL`, `0x400F_D000`).
//!
//! Register map and resets follow the TM4C1294NCPDT datasheet (SPMS433B §8,
//! Table 8-6 for resets). The 129 controller keeps the 123 command core
//! (`FMC` key + `WRITE`/`ERASE`/`MERASE`/`COMT`, `FCRIS`/`FCIM`/`FCMISC`)
//! but adds buffered programming (`FMC2` + `FWBN0-31` + `FWBVAL`), wider
//! protection (`FMPRE0-15`/`FMPPE0-15`), and ID registers (`PP`
//! `0xF014_01FF`: 1 MB flash, 16 KB sectors; `SSIZE` `0x03FF`: 256 KB
//! SRAM). Like [`crate::device::ti::flash`], this model is
//! protocol-exact with the array untouched: commands complete instantly,
//! raising `PRIS`/`ERIS`/`ARIS`.
//!
//! Modeled: keyed `FMC` commands (self-clearing), keyed `FMC2.WRBUF`
//! buffered program (valid words per `FWBVAL`, cleared on completion),
//! protection enforcement (open `0xFFFF_FFFF` stamped; committed-zero bits
//! deny, 2 KiB granules over the 1 MB array), `PP`/`SSIZE` RO ID,
//! `BOOTCFG` POR reset, `FCIM` gating + `FCMISC` W1C, the shared EEPROM
//! completion link (folds into `ERIS`). Deliberate approximations: program/
//! erase latencies are zero (instant completion, same convention as the
//! 123 model); the flash array itself is never modified.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// `FLASH_CTRL` base address.
pub const FLASH_BASE: u32 = 0x400F_D000;

const FMA_OFF: u32 = 0x000;
const FMD_OFF: u32 = 0x004;
const FMC_OFF: u32 = 0x008;
const FCRIS_OFF: u32 = 0x00C;
const FCIM_OFF: u32 = 0x010;
const FCMISC_OFF: u32 = 0x014;
const FMC2_OFF: u32 = 0x020;
const FWBVAL_OFF: u32 = 0x030;
const FWBN_OFF: u32 = 0x100;
const FWBN_WORDS: u32 = 32;
const PP_OFF: u32 = 0xFC0;
const SSIZE_OFF: u32 = 0xFC4;
const BOOTCFG_OFF: u32 = 0x11D0;
const USERREG0_OFF: u32 = 0x11E0;
const FMPRE_BASE_OFF: u32 = 0x1200;
const FMPPE_BASE_OFF: u32 = 0x1400;
const FLASH_LEN: u32 = 0x0010_0000;

/// `FMC` command bits.
const FMC_COMT: u32 = 1 << 3;
const FMC_MERASE: u32 = 1 << 2;
const FMC_ERASE: u32 = 1 << 1;
const FMC_WRITE: u32 = 1 << 0;
/// `FMC`/`FMC2` write key (default-`KEY` value).
const FMC_WRKEY: u32 = 0xA442;
const FMC_WRKEY_SHIFT: u32 = 16;
/// `FMC2.WRBUF` bit 0.
const FMC2_WRBUF: u32 = 1 << 0;
/// `FCRIS`/`FCIM`/`FCMISC` bits (same low block as the 123 family).
const RIS_ERIS: u32 = 1 << 2;
const RIS_PRIS: u32 = 1 << 1;
const RIS_ARIS: u32 = 1 << 0;

/// Nonzero datasheet resets stamped over the SVD zeros.
const RESETS: &[(u32, u32)] = &[
    (PP_OFF, 0xF014_01FF),
    (SSIZE_OFF, 0x0000_03FF),
    (BOOTCFG_OFF, 0xFFFF_FFFE),
    (USERREG0_OFF, 0xFFFF_FFFF),
    (USERREG0_OFF + 4, 0xFFFF_FFFF),
    (USERREG0_OFF + 8, 0xFFFF_FFFF),
    (USERREG0_OFF + 12, 0xFFFF_FFFF),
];

/// Behavior behind the 129 flash-controller window (protocol-exact).
#[derive(Debug)]
pub struct Flash129Behavior {
    ris: u32,
    /// Shared EEPROM completion link (folds into `ERIS`).
    eeprom_link: Arc<Mutex<bool>>,
    pending: bool,
    /// Write-buffer words (`FWBN0-31`): the pack SVD describes a single
    /// `FWBN`, so the 32-word window lives here, served by the custom
    /// `FlashCtrl129::read`/`write` below.
    fwb: [u32; 32],
}

impl Flash129Behavior {
    fn ris_word(&self) -> u32 {
        let mut ris = self.ris;
        if let Ok(link) = self.eeprom_link.lock() {
            if *link {
                ris |= RIS_ERIS;
            }
        }
        ris
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        self.pending = self.ris_word() & view.get(FCIM_OFF) != 0;
    }

    /// Protection check: `FMPREn`/`FMPPEn` deny the `FMA` word (open by
    /// default, `0xFFFF_FFFF`; committed-zero bits deny, 2 KiB granules
    /// over the 1 MB array).
    fn protected(&self, view: &Registers<&mut [u32]>, program: bool) -> bool {
        let fma = view.get(FMA_OFF);
        if fma >= FLASH_LEN {
            return false; // Resident-register space goes through `COMT`.
        }
        let bit = (fma >> 11) & 0x1FF;
        let base = if program {
            FMPPE_BASE_OFF
        } else {
            FMPRE_BASE_OFF
        };
        let word = view.get(base + 4 * (bit >> 5));
        word & (1 << (bit & 31)) == 0
    }

    /// Fire one keyed `FMC` command word; returns the completion word.
    fn fire_fmc(&mut self, view: &mut Registers<&mut [u32]>, staged: u32) -> u32 {
        let cmds = staged & 0x0F;
        if cmds & FMC_WRITE != 0 {
            if self.protected(view, true) {
                self.ris |= RIS_ARIS;
            } else {
                self.ris |= RIS_PRIS;
            }
        } else if cmds & (FMC_ERASE | FMC_MERASE) != 0 {
            self.ris |= RIS_ERIS;
        } else {
            // `COMT`: commits the resident register, instant done.
            self.ris |= RIS_PRIS;
        }
        view.set(FMC_OFF, 0);
        self.refresh(view);
        0
    }

    /// Fire a keyed `FMC2.WRBUF`: program every `FWBVAL`-valid buffer word
    /// at `FMA`, then clear the valid bits.
    fn fire_fmc2(&mut self, view: &mut Registers<&mut [u32]>) -> u32 {
        if self.protected(view, true) {
            self.ris |= RIS_ARIS;
        } else {
            // The array is untouched (protocol model); validity clears.
            view.set(FWBVAL_OFF, 0);
            self.ris |= RIS_PRIS;
        }
        view.set(FMC2_OFF, 0);
        self.refresh(view);
        0
    }
}

impl Behavior for Flash129Behavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        self.ris = 0;
        self.pending = false;
        self.fwb = [0xFFFF_FFFF; 32];
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(self.ris_word() & view.get(FCIM_OFF) != 0)
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            FCRIS_OFF => self.ris_word(),
            FCMISC_OFF => self.ris_word() & view.get(FCIM_OFF),
            FMC_OFF | FMC2_OFF => value & 0x0000_000F, // `WRKEY` reads 0.
            _ => value,
        };
        self.refresh(view);
        out
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == FCRIS_OFF {
            return view.get(offset);
        }
        if offset == PP_OFF || offset == SSIZE_OFF {
            return view.get(offset); // RO ID words reject writes.
        }
        if offset == FCMISC_OFF {
            let word = view.merged(offset, val, shift);
            self.ris &= !word;
            if word & RIS_ERIS != 0 {
                if let Ok(mut link) = self.eeprom_link.lock() {
                    *link = false;
                }
            }
            self.refresh(view);
            return self.ris_word() & view.get(FCIM_OFF);
        }
        if offset == FMC_OFF || offset == FMC2_OFF {
            // Commands fire only with the key in the staged word; command
            // bits self-clear on instant completion. Byte lanes stage (like
            // the GPIO lock key), so word stores fire once whole. The
            // staged word is returned so `RegisterFile` keeps it.
            let staged = view.merged(offset, val, shift);
            let key_ok = (staged >> FMC_WRKEY_SHIFT) == FMC_WRKEY;
            if key_ok {
                if offset == FMC_OFF {
                    let cmds = staged & 0x0F;
                    if cmds.count_ones() == 1 {
                        return self.fire_fmc(view, staged);
                    }
                } else if staged & FMC2_WRBUF != 0 {
                    return self.fire_fmc2(view);
                }
            }
            view.set(offset, staged);
            self.refresh(view);
            return staged;
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        self.refresh(view);
        word
    }
}

/// A Tiva C 129-family flash memory controller (protocol-exact, array untouched).
pub struct FlashCtrl129 {
    inner: RegisterFile<Flash129Behavior>,
    eeprom_link: Arc<Mutex<bool>>,
}

impl FlashCtrl129 {
    /// A new 129 flash controller on `D`'s tables, sharing the EEPROM
    /// completion link in `eeprom_link`.
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"FLASH_CTRL"`.
    #[must_use]
    pub fn for_device<D: Device>(eeprom_link: Arc<Mutex<bool>>) -> Self {
        let def = D::peripheral("FLASH_CTRL").expect("generated tables contain FLASH_CTRL");
        let mut flash = Self {
            inner: RegisterFile::new(
                def,
                Flash129Behavior {
                    ris: 0,
                    eeprom_link: eeprom_link.clone(),
                    pending: false,
                    fwb: [0xFFFF_FFFF; 32],
                },
            ),
            eeprom_link,
        };
        flash.stamp_resets();
        flash
    }

    /// Stamp the nonzero datasheet resets over the SVD zeros (open
    /// protection, ID words, `BOOTCFG`/`USERREGn` POR values).
    fn stamp_resets(&mut self) {
        let def = self.inner.def();
        let mut stamped: Vec<(usize, u32)> = RESETS
            .iter()
            .filter_map(|&(offset, value)| {
                def.registers
                    .iter()
                    .position(|r| r.offset == offset)
                    .map(|idx| (idx, value))
            })
            .collect();
        for n in 0..16 {
            for (base,) in [(FMPRE_BASE_OFF,), (FMPPE_BASE_OFF,)] {
                let offset = base + 4 * n;
                if let Some(idx) = def.registers.iter().position(|r| r.offset == offset) {
                    stamped.push((idx, 0xFFFF_FFFF));
                }
            }
        }
        let words = self.inner.regs_mut();
        for (idx, value) in stamped {
            words[idx] = value;
        }
    }
}

impl Peripheral for FlashCtrl129 {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let base = self.inner.ranges()[0].base;
        let off = addr
            .checked_sub(base)
            .filter(|o| *o < self.inner.ranges()[0].len)
            .ok_or(BusError::Reserved)?;
        // `FWBN0-31` live in the behavior (the SVD describes one `FWBN`).
        if (FWBN_OFF..FWBN_OFF + 4 * FWBN_WORDS).contains(&off) {
            let word = (off - FWBN_OFF) / 4;
            let lane = (off - FWBN_OFF) % 4;
            return Ok(self
                .inner
                .with_view(|_, b| ((b.fwb[word as usize] >> (8 * lane)) & 0xFF) as u8));
        }
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let base = self.inner.ranges()[0].base;
        let off = addr
            .checked_sub(base)
            .filter(|o| *o < self.inner.ranges()[0].len)
            .ok_or(BusError::Reserved)?;
        if (FWBN_OFF..FWBN_OFF + 4 * FWBN_WORDS).contains(&off) {
            let word = (off - FWBN_OFF) / 4;
            let lane = (off - FWBN_OFF) % 4;
            self.inner.with_view(|_, b| {
                let w = b.fwb[word as usize] & !(0xFF << (8 * lane));
                b.fwb[word as usize] = w | (u32::from(val) << (8 * lane));
            });
            return Ok(());
        }
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
        self.stamp_resets();
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for FlashCtrl129 {
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
    use crate::device::svd::tm4c1294ncpdt::Tm4c1294ncpdt;

    const FLASH129_BASE: u32 = 0x400F_D000;

    fn flash129() -> FlashCtrl129 {
        FlashCtrl129::for_device::<Tm4c1294ncpdt>(Arc::new(Mutex::new(false)))
    }

    fn read_word(flash: &mut FlashCtrl129, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(flash.read(FLASH129_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(flash: &mut FlashCtrl129, offset: u32, value: u32) {
        for lane in 0..4 {
            flash
                .write(
                    FLASH129_BASE + offset + lane,
                    ((value >> (8 * lane)) & 0xFF) as u8,
                )
                .unwrap();
        }
    }

    /// Datasheet ID + POR resets land on the bus.
    #[test]
    fn datasheet_resets() {
        let mut flash = flash129();
        assert_eq!(read_word(&mut flash, PP_OFF), 0xF014_01FF);
        assert_eq!(read_word(&mut flash, SSIZE_OFF), 0x0000_03FF);
        assert_eq!(read_word(&mut flash, BOOTCFG_OFF), 0xFFFF_FFFE);
        assert_eq!(read_word(&mut flash, 0x1200), 0xFFFF_FFFF); // FMPRE0 open
        assert_eq!(read_word(&mut flash, 0x1400), 0xFFFF_FFFF); // FMPPE0 open
        assert_eq!(flash.exception_number(), Some(45)); // FLASH = IRQ 29.
    }

    /// RO ID words reject writes.
    #[test]
    fn ro_enforcement() {
        let mut flash = flash129();
        write_word(&mut flash, PP_OFF, 0x0);
        assert_eq!(read_word(&mut flash, PP_OFF), 0xF014_01FF);
        write_word(&mut flash, SSIZE_OFF, 0x0);
        assert_eq!(read_word(&mut flash, SSIZE_OFF), 0x0000_03FF);
    }

    /// Keyed single-word write completes (`PRIS`, self-clear).
    #[test]
    fn keyed_write_completes() {
        let mut flash = flash129();
        write_word(&mut flash, FMA_OFF, 0x1000);
        write_word(&mut flash, FMD_OFF, 0x1234_5678);
        write_word(&mut flash, FCIM_OFF, RIS_PRIS);
        write_word(
            &mut flash,
            FMC_OFF,
            (FMC_WRKEY << FMC_WRKEY_SHIFT) | FMC_WRITE,
        );
        assert_eq!(read_word(&mut flash, FMC_OFF) & FMC_WRITE, 0);
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_PRIS, RIS_PRIS);
        assert_eq!(flash.pending(), 1);
        write_word(&mut flash, FCMISC_OFF, RIS_PRIS);
        assert_eq!(flash.pending(), 0);
    }

    /// Keyless commands sink (no-op).
    #[test]
    fn keyless_sinks() {
        let mut flash = flash129();
        write_word(&mut flash, FMC_OFF, FMC_WRITE);
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_PRIS, 0);
    }

    /// Buffered write programs the valid words and clears `FWBVAL`.
    #[test]
    fn buffered_write_flow() {
        let mut flash = flash129();
        write_word(&mut flash, FMA_OFF, 0x2000);
        write_word(&mut flash, FWBN_OFF, 0xDEAD_BEEF);
        write_word(&mut flash, FWBN_OFF + 4, 0xCAFE_0001);
        write_word(&mut flash, FWBVAL_OFF, 0x03);
        write_word(
            &mut flash,
            FMC2_OFF,
            (FMC_WRKEY << FMC_WRKEY_SHIFT) | FMC2_WRBUF,
        );
        assert_eq!(read_word(&mut flash, FWBVAL_OFF), 0);
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_PRIS, RIS_PRIS);
    }

    /// Protected words deny with `ARIS`; erase raises `ERIS`.
    #[test]
    fn protection_and_erase() {
        let mut flash = flash129();
        // Close the 2 KiB granule holding `FMA` (`bit 2` for `0x1000`).
        write_word(&mut flash, 0x1400, 0xFFFF_FFFB);
        write_word(&mut flash, FMA_OFF, 0x1000);
        write_word(&mut flash, FMD_OFF, 0x1);
        write_word(
            &mut flash,
            FMC_OFF,
            (FMC_WRKEY << FMC_WRKEY_SHIFT) | FMC_WRITE,
        );
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_ARIS, RIS_ARIS);
        // Open array erases cleanly.
        let mut flash = flash129();
        write_word(&mut flash, FMA_OFF, 0x10000);
        write_word(
            &mut flash,
            FMC_OFF,
            (FMC_WRKEY << FMC_WRKEY_SHIFT) | FMC_ERASE,
        );
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_ERIS, RIS_ERIS);
        write_word(
            &mut flash,
            FMC_OFF,
            (FMC_WRKEY << FMC_WRKEY_SHIFT) | FMC_MERASE,
        );
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_ERIS, RIS_ERIS);
    }

    /// The EEPROM link folds into `ERIS`.
    #[test]
    fn erase_and_eeprom_link() {
        let link = Arc::new(Mutex::new(false));
        let mut flash = FlashCtrl129::for_device::<Tm4c1294ncpdt>(link.clone());
        *link.lock().unwrap() = true;
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_ERIS, RIS_ERIS);
        write_word(&mut flash, FCMISC_OFF, RIS_ERIS);
        assert!(!*link.lock().unwrap());
    }
}
