//! TI Tiva C flash memory controller (`FLASH_CTRL`).
//!
//! Controller protocol follows the TM4C123GH6PM datasheet (SPMS376E
//! §8.2.3): `FMC` commands (`WRITE`/`ERASE`/`MERASE`/`COMT`) fire only
//! with the `WRKEY` (`0xA442`, the default-`KEY` value) in the same word
//! write — anything else is ignored — and complete synchronously (reads
//! show done; millisecond program/erase timing is unobservable to poll
//! loops, like the repo's other instant completions). Completion raises
//! `PRIS` (program/commit) or `ERIS` (erase/mass-erase) behind `FCIM`,
//! cleared by `FCMISC` W1C; protection violations (`FMPRE`/`FMPPE` deny
//! the `FMA` word) raise `ARIS`. `FSIZE`/`SSIZE` constants, `FMC2`/
//! `FWBVAL`/`FWBn`/protection/`USERREG`/`BOOTCFG` latches.
//!
//! Deliberate boundary: the controller is exact, the array is untouched —
//! programs/erases report `DONE` without mutating flash contents (the
//! behavior owns its window only; documented). Firmware polling `DONE`
//! and checking `ACCESS` errors runs fully; readback-after-program flows
//! do not. EEPROM completions arrive on the shared link as `ERIS`
//! (datasheet §8.2.4), cleared with the bit.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

const FMA_OFF: u32 = 0x000;
const FMD_OFF: u32 = 0x004;
const FMC_OFF: u32 = 0x008;
const FCRIS_OFF: u32 = 0x00C;
const FCIM_OFF: u32 = 0x010;
const FCMISC_OFF: u32 = 0x014;
const FSIZE_OFF: u32 = 0xFC0;
const SSIZE_OFF: u32 = 0xFC4;
/// `FMC` command bits: `COMT` 3, `MERASE` 2, `ERASE` 1, `WRITE` 0.
const FMC_COMT: u32 = 1 << 3;
const FMC_MERASE: u32 = 1 << 2;
const FMC_ERASE: u32 = 1 << 1;
const FMC_WRITE: u32 = 1 << 0;
/// `FMC.WRKEY` (default-`KEY` value; `BOOTCFG.KEY` selects the alternate).
const FMC_WRKEY: u32 = 0xA442;
const FMC_WRKEY_SHIFT: u32 = 16;
/// `FCRIS`/`FCIM`/`FCMISC` bits: program 13, error 11, invalid-data 10,
/// voltage 9, EEPROM 2, program-done... (`ARIS` 0 access, `PRIS` 1 prog).
const RIS_PROGRIS: u32 = 1 << 13;
const RIS_ERRIS: u32 = 1 << 11;
const RIS_INVDRIS: u32 = 1 << 10;
const RIS_VOLTRIS: u32 = 1 << 9;
const RIS_ERIS: u32 = 1 << 2;
const RIS_PRIS: u32 = 1 << 1;
const RIS_ARIS: u32 = 1 << 0;

/// Nonzero datasheet resets stamped over the SVD zeros (`FMPREn`/
/// `FMPPEn` reset open, `0xFFFF_FFFF`).
const RESETS: &[(u32, u32)] = &[
    (FSIZE_OFF, 0x0000_007F),
    (SSIZE_OFF, 0x0000_007F),
    (0x1200, 0xFFFF_FFFF), // FMPRE0
    (0x1204, 0xFFFF_FFFF), // FMPRE1
    (0x1208, 0xFFFF_FFFF), // FMPRE2
    (0x120C, 0xFFFF_FFFF), // FMPRE3
    (0x1400, 0xFFFF_FFFF), // FMPPE0
    (0x1404, 0xFFFF_FFFF), // FMPPE1
    (0x1408, 0xFFFF_FFFF), // FMPPE2
    (0x140C, 0xFFFF_FFFF), // FMPPE3
];

/// Behavior behind the flash-controller window (protocol-exact).
#[derive(Debug)]
pub struct FlashBehavior {
    ris: u32,
    /// Shared EEPROM completion link (folds into `ERIS`).
    eeprom_link: Arc<Mutex<bool>>,
    pending: bool,
}

impl FlashBehavior {
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
    /// default, `0xFFFF_FFFF`; committed-zero bits deny, 2 KiB granules).
    fn protected(&self, view: &Registers<&mut [u32]>, program: bool) -> bool {
        let fma = view.get(FMA_OFF);
        if fma >= 0x0004_0000 {
            return false; // Resident-register space goes through `COMT`.
        }
        let bit = (fma >> 11) & 0x7F;
        let base = if program { 0x1400 } else { 0x1200 };
        let word = view.get(base + 4 * (bit >> 5));
        word & (1 << (bit & 31)) == 0
    }
}

impl Behavior for FlashBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        self.ris = 0;
        self.pending = false;
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            FCRIS_OFF => self.ris_word(),
            FCMISC_OFF => self.ris_word() & view.get(FCIM_OFF),
            FMC_OFF => value & 0x0000_000F, // `WRKEY` reads 0.
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
        if offset == FMC_OFF {
            // Commands fire only with the key in the staged word; command
            // bits self-clear on instant completion. Byte lanes stage (like
            // the GPIO lock key), so word stores fire once whole. The
            // staged word is returned so `RegisterFile` keeps it.
            let staged = view.merged(offset, val, shift);
            let key_ok = (staged >> FMC_WRKEY_SHIFT) == FMC_WRKEY;
            let cmds = staged & 0x0F;
            if key_ok && cmds.count_ones() == 1 {
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
                // `FMC` reads back done (instant completion).
                view.set(offset, 0);
                self.refresh(view);
                return 0;
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

/// A Tiva C flash memory controller (protocol-exact, array untouched).
pub struct FlashCtrl {
    inner: RegisterFile<FlashBehavior>,
}

impl FlashCtrl {
    /// A new controller on `D`'s tables, sharing the EEPROM completion
    /// link with the `EEPROM` behavior.
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"FLASH_CTRL"`.
    #[must_use]
    pub fn for_device<D: Device>(eeprom_link: Arc<Mutex<bool>>) -> Self {
        let def = D::peripheral("FLASH_CTRL").expect("generated tables contain FLASH_CTRL");
        let mut flash = Self {
            inner: RegisterFile::new(
                def,
                FlashBehavior {
                    ris: 0,
                    eeprom_link,
                    pending: false,
                },
            ),
        };
        flash.stamp_resets();
        flash
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
}

impl Peripheral for FlashCtrl {
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
        self.stamp_resets();
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for FlashCtrl {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        u32::from(self.inner.behavior().pending)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const FLASH_BASE: u32 = 0x400F_D000;

    fn flash() -> (FlashCtrl, Arc<Mutex<bool>>) {
        let link = Arc::new(Mutex::new(false));
        (FlashCtrl::for_device::<Tm4c123gh6pm>(link.clone()), link)
    }

    fn read_word(flash: &mut FlashCtrl, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(flash.read(FLASH_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(flash: &mut FlashCtrl, offset: u32, value: u32) {
        for lane in 0..4 {
            flash
                .write(
                    FLASH_BASE + offset + lane,
                    ((value >> (8 * lane)) & 0xFF) as u8,
                )
                .unwrap();
        }
    }

    /// Keyed `WRITE` completes instantly with `PRIS`; `WRKEY` reads 0.
    #[test]
    fn keyed_write_completes() {
        let (mut flash, _) = flash();
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
        assert_eq!(flash.exception_number(), Some(45));
        write_word(&mut flash, FCMISC_OFF, RIS_PRIS);
        assert_eq!(flash.pending(), 0);
    }

    /// Missing/wrong keys are ignored: no-op, no interrupt.
    #[test]
    fn wrong_key_ignored() {
        let (mut flash, _) = flash();
        write_word(&mut flash, FMC_OFF, FMC_WRITE); // no key
        assert_eq!(read_word(&mut flash, FCRIS_OFF), 0);
        write_word(&mut flash, FMC_OFF, (0x1234 << FMC_WRKEY_SHIFT) | FMC_ERASE);
        assert_eq!(read_word(&mut flash, FCRIS_OFF), 0);
    }

    /// Keyed `ERASE` raises `ERIS`; the EEPROM link folds in too.
    #[test]
    fn erase_and_eeprom_link() {
        let (mut flash, link) = flash();
        write_word(&mut flash, FCIM_OFF, RIS_ERIS);
        write_word(&mut flash, FMA_OFF, 0x2000);
        write_word(
            &mut flash,
            FMC_OFF,
            (FMC_WRKEY << FMC_WRKEY_SHIFT) | FMC_ERASE,
        );
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_ERIS, RIS_ERIS);
        write_word(&mut flash, FCMISC_OFF, RIS_ERIS);
        *link.lock().unwrap() = true; // EEPROM completion arrives.
        assert_eq!(read_word(&mut flash, FCRIS_OFF) & RIS_ERIS, RIS_ERIS);
        assert_eq!(flash.pending(), 1);
        write_word(&mut flash, FCMISC_OFF, RIS_ERIS);
        assert!(!*link.lock().unwrap());
        assert_eq!(flash.pending(), 0);
    }

    /// Datasheet size constants land on the bus.
    #[test]
    fn size_constants() {
        let (mut flash, _) = flash();
        assert_eq!(read_word(&mut flash, FSIZE_OFF), 0x7F);
        assert_eq!(read_word(&mut flash, SSIZE_OFF), 0x7F);
    }
}
