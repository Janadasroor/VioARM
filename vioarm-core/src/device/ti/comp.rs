//! TI Tiva C analog comparators (`COMP`, comparators 0-1, or 0-2 where
//! the tables carry `ACCTL2`).
//!
//! Output and interrupt sense follow the TM4C123GH6PM datasheet
//! (SPMS376E §19): `OVAL` reports the driven input state through `CINV`,
//! `ISEN`/`ISLVAL` select level/falling/rising/either-edge sense into the
//! `INn` latches behind `ACINTEN`, with `ACMIS` W1C clearing both masked
//! and raw bits. `ACREFCTL` latches; trigger paths (`TOEN`/`TSEN`),
//! `ASRCP` source selection, and `ACMPPP` constants complete the window.
//!
//! Inputs arrive through [`Comp::drive`] (host/test stimulus; no analog
//! in T2): each comparator's output is its driven level. Per-comparator
//! NVIC lines (`COMP0`/`COMP1`/`COMP2`) carry the masked bits.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const ACMIS_OFF: u32 = 0x000;
const ACRIS_OFF: u32 = 0x004;
const ACINTEN_OFF: u32 = 0x008;
const ACSTAT0_OFF: u32 = 0x020;
const ACCTL0_OFF: u32 = 0x024;
const ACSTAT1_OFF: u32 = 0x040;
const ACCTL1_OFF: u32 = 0x044;
const PP_OFF: u32 = 0xFC0;
/// `ACSTATn.OVAL` bit 1.
const STAT_OVAL: u32 = 1 << 1;
/// `ACCTLn` fields: `ISEN` 3:2, `ISLVAL` 4, `CINV` 1.
const CTL_ISEN_SHIFT: u32 = 2;
const CTL_ISEN_MASK: u32 = 0x03;
const CTL_ISLVAL: u32 = 1 << 4;
const CTL_CINV: u32 = 1 << 1;
/// Sense modes.
const SENSE_LEVEL: u32 = 0x0;
const SENSE_FALLING: u32 = 0x1;
const SENSE_RISING: u32 = 0x2;
const SENSE_EITHER: u32 = 0x3;

/// Nonzero datasheet resets stamped over the SVD zeros.
const RESETS: &[(u32, u32)] = &[(PP_OFF, 0x0003_0003)];

/// Behavior behind the COMP window (driven outputs + sense).
#[derive(Debug)]
pub struct CompBehavior {
    /// Driven input levels per comparator.
    driven: [bool; 3],
    /// Last evaluated outputs (for edge detection).
    prev: [bool; 3],
    /// Edge-latch bits (`INn` in `ACRIS`).
    edge: [bool; 3],
    pending: [bool; 3],
    /// Comparator count from the tables (2, or 3 where `ACCTL2` exists).
    ncomp: usize,
}

impl Default for CompBehavior {
    fn default() -> Self {
        Self {
            driven: [false; 3],
            prev: [false; 3],
            edge: [false; 3],
            pending: [false; 3],
            ncomp: 2,
        }
    }
}

impl CompBehavior {
    /// `ACCTL` offset of comparator `n` (`0x24 + 0x20*n`).
    fn ctl_off(n: usize) -> u32 {
        ACCTL0_OFF + 0x20 * n as u32
    }

    /// `ACSTAT` offset of comparator `n` (`0x20 + 0x20*n`).
    fn stat_off(n: usize) -> u32 {
        ACSTAT0_OFF + 0x20 * n as u32
    }
}

impl CompBehavior {
    fn output(&self, view: &Registers<&mut [u32]>, n: usize) -> bool {
        let ctl = view.get(Self::ctl_off(n));
        self.driven[n] ^ (ctl & CTL_CINV != 0)
    }

    /// Re-evaluate comparator `n` after a drive change.
    fn poll(&mut self, view: &Registers<&mut [u32]>, n: usize) {
        let ctl = view.get(Self::ctl_off(n));
        let out = self.output(view, n);
        let before = self.prev[n];
        self.prev[n] = out;
        let sense = (ctl >> CTL_ISEN_SHIFT) & CTL_ISEN_MASK;
        match sense {
            SENSE_LEVEL => {}
            SENSE_FALLING => {
                if before && !out {
                    self.edge[n] = true;
                }
            }
            SENSE_RISING => {
                if !before && out {
                    self.edge[n] = true;
                }
            }
            _ => {
                if before != out {
                    self.edge[n] = true;
                }
            }
        }
        self.refresh(view);
    }

    /// Raw status bit: edge latch, or live level match.
    fn raw(&self, view: &Registers<&mut [u32]>, n: usize) -> bool {
        if self.edge[n] {
            return true;
        }
        let ctl = view.get(Self::ctl_off(n));
        if (ctl >> CTL_ISEN_SHIFT) & CTL_ISEN_MASK != SENSE_LEVEL {
            return false;
        }
        self.output(view, n) == (ctl & CTL_ISLVAL != 0)
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        let inten = view.get(ACINTEN_OFF);
        for n in 0..self.ncomp {
            self.pending[n] = self.raw(view, n) && inten & (1 << n) != 0;
        }
    }
}

impl Behavior for CompBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        let (driven, ncomp) = (self.driven, self.ncomp);
        *self = Self::default();
        self.driven = driven;
        self.ncomp = ncomp;
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            ACRIS_OFF => {
                (0..self.ncomp).fold(0, |acc, n| acc | (u32::from(self.raw(view, n)) << n))
            }
            ACMIS_OFF => {
                let inten = view.get(ACINTEN_OFF);
                (0..self.ncomp).fold(0, |acc, n| {
                    acc | ((u32::from(self.raw(view, n)) & ((inten >> n) & 0x01)) << n)
                })
            }
            _ if offset == ACSTAT0_OFF || offset == ACSTAT1_OFF || offset == 0x60 => {
                let n = ((offset - ACSTAT0_OFF) / 0x20) as usize;
                if n < self.ncomp {
                    u32::from(self.output(view, n)) << 1
                } else {
                    value
                }
            }
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
        if offset == ACSTAT0_OFF || offset == ACSTAT1_OFF || offset == 0x60 || offset == ACRIS_OFF {
            return view.get(offset);
        }
        if offset == ACMIS_OFF {
            // W1C: clears masked and raw bits (datasheet `ACMIS` page).
            let word = view.merged(offset, val, shift);
            for n in 0..self.ncomp {
                if word & (1 << n) != 0 {
                    self.edge[n] = false;
                }
            }
            self.refresh(view);
            return view.get(offset);
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        // `CINV`/`ISEN` changes re-evaluate levels immediately.
        if offset == ACCTL0_OFF || offset == ACCTL1_OFF || offset == 0x64 {
            let n = ((offset - ACCTL0_OFF) / 0x20) as usize;
            if n < self.ncomp {
                self.poll(view, n);
            }
        }
        self.refresh(view);
        word
    }
}

/// A Tiva C analog comparator block (comparators 0-1).
pub struct Comp {
    inner: RegisterFile<CompBehavior>,
}

impl Comp {
    /// A new comparator block on `D`'s tables (`"COMP"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"COMP"`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain COMP");
        let ncomp = if def.registers.iter().any(|r| r.offset == 0x64) {
            3
        } else {
            2
        };
        let mut comp = Self {
            inner: RegisterFile::new(
                def,
                CompBehavior {
                    ncomp,
                    ..CompBehavior::default()
                },
            ),
        };
        comp.stamp_resets();
        comp
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

    /// Drive comparator `n`'s input level (host/test stimulus).
    pub fn drive(&mut self, n: usize, high: bool) {
        self.inner.with_view(|view, b| {
            if n >= b.ncomp {
                return;
            }
            b.driven[n] = high;
            b.poll(view, n);
        });
    }
}

impl Peripheral for Comp {
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

impl InterruptSource for Comp {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn exception_for_line(&self, line: u32) -> Option<u8> {
        // `COMP0`/`COMP1` head the table, one line each.
        self.inner.exception_for_line(line)
    }

    fn lines(&self) -> u32 {
        self.inner.behavior().ncomp as u32
    }

    fn pending(&self) -> u32 {
        let b = self.inner.behavior();
        (0..b.ncomp).fold(0, |acc, n| acc | (u32::from(b.pending[n]) << n))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const COMP_BASE: u32 = 0x4003_C000;

    fn comp() -> Comp {
        Comp::for_device::<Tm4c123gh6pm>("COMP")
    }

    fn read_word(comp: &mut Comp, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(comp.read(COMP_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(comp: &mut Comp, offset: u32, value: u32) {
        for lane in 0..4 {
            comp.write(
                COMP_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Driven levels surface through `OVAL` with `CINV` applied.
    #[test]
    fn oval_follows_drive() {
        let mut comp = comp();
        comp.drive(0, true);
        assert_eq!(read_word(&mut comp, ACSTAT0_OFF) & STAT_OVAL, STAT_OVAL);
        write_word(&mut comp, ACCTL0_OFF, CTL_CINV);
        assert_eq!(read_word(&mut comp, ACSTAT0_OFF) & STAT_OVAL, 0);
        assert_eq!(comp.exception_number(), Some(41));
        assert_eq!(comp.exception_for_line(1), Some(42));
    }

    /// Third comparator (100-pin+ dice): `ACSTAT2`/`ACCTL2` live with
    /// their own line; two-comparator tables stay at two.
    #[test]
    fn third_comparator_where_present() {
        use crate::device::svd::tm4c123gh6pz::Tm4c123gh6pz;
        let mut comp3 = Comp::for_device::<Tm4c123gh6pz>("COMP");
        assert_eq!(comp3.lines(), 3);
        comp3.drive(2, true);
        assert_eq!(read_word(&mut comp3, 0x60) & STAT_OVAL, STAT_OVAL);
        write_word(&mut comp3, 0x64, CTL_CINV);
        assert_eq!(read_word(&mut comp3, 0x60) & STAT_OVAL, 0);
        // Out-of-range drives are ignored.
        comp3.drive(3, true);
        assert_eq!(comp().lines(), 2);
    }

    /// Rising-edge sense latches `IN0` behind `INTEN`; `ACMIS` W1C clears.
    #[test]
    fn rising_edge_flow() {
        let mut comp = comp();
        write_word(&mut comp, ACCTL0_OFF, SENSE_RISING << CTL_ISEN_SHIFT);
        write_word(&mut comp, ACINTEN_OFF, 0x01);
        assert_eq!(comp.pending() & 0x01, 0);
        comp.drive(0, true);
        assert_eq!(read_word(&mut comp, ACRIS_OFF) & 0x01, 0x01);
        assert_eq!(comp.pending() & 0x01, 0x01);
        write_word(&mut comp, ACMIS_OFF, 0x01);
        assert_eq!(read_word(&mut comp, ACRIS_OFF) & 0x01, 0);
        assert_eq!(comp.pending() & 0x01, 0);
    }

    /// Level sense tracks live with no latch.
    #[test]
    fn level_sense_tracks_live() {
        let mut comp = comp();
        // `ISEN=0` level, `ISLVAL=1`: interrupt while HIGH.
        write_word(&mut comp, ACCTL0_OFF, CTL_ISLVAL);
        write_word(&mut comp, ACINTEN_OFF, 0x01);
        comp.drive(0, true);
        assert_eq!(comp.pending() & 0x01, 0x01);
        comp.drive(0, false);
        assert_eq!(comp.pending() & 0x01, 0);
        assert_eq!(read_word(&mut comp, ACRIS_OFF) & 0x01, 0);
    }
}
