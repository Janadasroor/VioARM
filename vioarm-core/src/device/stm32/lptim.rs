//! Low-power timer (`LPTIM`): continuous/one-shot counter with compare and
//! autoreload matches, internal prescaled clock or host-injected external
//! edges, and two interrupt lines (compare + autoreload).
//!
//! Register layout is uniform across families (`ISR`@0x00, `ICR`@0x04,
//! `IER`/`DIER`@0x08, `CFGR`@0x0C, `CR`@0x10, `CMP`@0x14, `ARR`@0x18,
//! `CNT`@0x1C — U5/H5 split some into `_input`/`_output` views at identical
//! offsets), so fixed offsets drive every family; only `CR.ENABLE`,
//! `CFGR.CKSEL`/`PRESC`, and the match flags participate. Encoder mode and
//! capture channels latch only.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const ISR: u32 = 0x00;
const ICR: u32 = 0x04;
const IER: u32 = 0x08;
const CFGR: u32 = 0x0C;
const CR: u32 = 0x10;
const CMP: u32 = 0x14;
const ARR: u32 = 0x18;
const CNT: u32 = 0x1C;

const CR_ENABLE: u32 = 1 << 0;
const CR_SNGSTRT: u32 = 1 << 2;
const CR_CNTSTRT: u32 = 1 << 3;
const ISR_CMPM: u32 = 1 << 0;
const ISR_ARRM: u32 = 1 << 1;
const IER_CMPMIE: u32 = 1 << 0;
const IER_ARRMIE: u32 = 1 << 1;
const CFGR_CKSEL: u32 = 1 << 0;
const CFGR_PRESC_SHIFT: u32 = 9;

/// 16-bit mask for counter/compare/autoreload values.
const MASK16: u32 = 0xFFFF;

#[derive(Clone, Debug, Default)]
struct LptimBehavior {
    enabled: bool,
    oneshot: bool,
    counter: u32,
    phase: u64,
    last_input: bool,
}

impl LptimBehavior {
    fn prescale<W>(view: &Registers<W>) -> u32
    where
        W: std::ops::Deref<Target = [u32]>,
    {
        1 << ((view.get(CFGR) >> CFGR_PRESC_SHIFT) & 0x7).min(7)
    }

    fn external<W>(view: &Registers<W>) -> bool
    where
        W: std::ops::Deref<Target = [u32]>,
    {
        view.get(CFGR) & CFGR_CKSEL != 0
    }

    fn arr<W>(view: &Registers<W>) -> u32
    where
        W: std::ops::Deref<Target = [u32]>,
    {
        let arr = view.get(ARR) & MASK16;
        if arr == 0 {
            0xFFFF
        } else {
            arr
        }
    }

    /// Advance the counter by `ticks` internal/external steps, evaluating
    /// compare then autoreload matches in that order.
    fn advance(&mut self, view: &mut Registers<&mut [u32]>, mut ticks: u64) {
        let cmp = view.get(CMP) & MASK16;
        let arr = Self::arr(view);
        while ticks > 0 {
            ticks -= 1;
            self.counter = (self.counter + 1) & MASK16;
            if self.counter == cmp {
                view.set(ISR, view.get(ISR) | ISR_CMPM);
            }
            if self.counter >= arr {
                view.set(ISR, view.get(ISR) | ISR_ARRM);
                self.counter = 0;
                if self.oneshot {
                    self.enabled = false;
                    return;
                }
            }
        }
    }
}

impl Behavior for LptimBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        match offset {
            ICR => {
                // Write-1-to-clear the match flags; the register itself
                // always reads back zero.
                if word & ISR_CMPM != 0 {
                    view.set(ISR, view.get(ISR) & !ISR_CMPM);
                }
                if word & ISR_ARRM != 0 {
                    view.set(ISR, view.get(ISR) & !ISR_ARRM);
                }
                return 0;
            }
            CR => {
                let was = self.enabled;
                self.enabled = word & CR_ENABLE != 0;
                if !self.enabled {
                    self.counter = 0;
                    self.phase = 0;
                    self.oneshot = false;
                    view.set(ISR, view.get(ISR) & !(ISR_CMPM | ISR_ARRM));
                } else if !was {
                    self.oneshot = word & CR_SNGSTRT != 0;
                    self.counter = 0;
                    self.phase = 0;
                } else if word & (CR_SNGSTRT | CR_CNTSTRT) != 0 {
                    self.oneshot = word & CR_SNGSTRT != 0;
                }
                return word;
            }
            CNT => {
                // `CNT` reads the live counter; writes are consumed.
                return view.get(CNT);
            }
            _ => {}
        }
        word
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == CNT {
            return view.get(CNT) & !MASK16 | (self.counter & MASK16);
        }
        let _ = view;
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !self.enabled || Self::external(view) {
            return;
        }
        let presc = u64::from(Self::prescale(view));
        let acc = self.phase + delta;
        self.phase = acc % presc;
        self.advance(view, acc / presc);
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        // External counter mode: a rising edge on bit 0 is one count.
        let level = value & 1 != 0;
        let rising = level && !self.last_input;
        self.last_input = level;
        if rising && self.enabled && Self::external(view) {
            self.advance(view, 1);
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        // Single-IRQ device (compare and autoreload share the line), so like
        // the general timers this reports scalar 0/1, not a line bitmask.
        let isr = view.get(ISR);
        let ier = view.get(IER);
        let cmp = isr & ISR_CMPM != 0 && ier & IER_CMPMIE != 0;
        let arr = isr & ISR_ARRM != 0 && ier & IER_ARRMIE != 0;
        u32::from(cmp || arr)
    }

    fn acknowledge(&mut self, _lines: u32) {
        // No latch to clear: the ISR flag itself clears through ICR, like
        // silicon, and `pending()` re-derives from ISR/IER every call.
    }

    fn reset(&mut self) {
        self.enabled = false;
        self.oneshot = false;
        self.counter = 0;
        self.phase = 0;
        self.last_input = false;
    }
}

/// Low-power timer.
#[derive(Debug)]
pub struct Lptim {
    inner: RegisterFile<LptimBehavior>,
}

impl Lptim {
    /// A new `Lptim` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("LPTIM");
        Self {
            inner: RegisterFile::new(def, LptimBehavior::default()),
        }
    }
}

impl Peripheral for Lptim {
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

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn name(&self) -> &'static str {
        "lptim"
    }
}

impl InterruptSource for Lptim {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;

    fn w(c: &mut Lptim, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            #[allow(clippy::cast_possible_truncation)]
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }

    fn r(c: &mut Lptim, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }

    #[test]
    fn lptim_arr_match_latches() {
        let mut l = Lptim::new_for::<Stm32h743>("LPTIM1");
        w(&mut l, 0x08, 1 << 1); // IER ARRMIE
        w(&mut l, 0x18, 2); // ARR=2
        w(&mut l, 0x10, 1); // CR ENABLE
        assert_eq!(l.pending(), 0);
        l.tick(2);
        assert_ne!(r(&mut l, 0x00) & ISR_ARRM, 0);
        assert_eq!(l.pending(), 1);
        w(&mut l, 0x04, 1 << 1); // ICR ARRMCF
        assert_eq!(r(&mut l, 0x00) & ISR_ARRM, 0);
    }

    #[test]
    fn lptim_compare_match_raises_line() {
        let mut l = Lptim::new_for::<Stm32h743>("LPTIM1");
        w(&mut l, 0x08, 1); // IER CMPMIE
        w(&mut l, 0x14, 3); // CMP=3
        w(&mut l, 0x18, 0xFFFF); // ARR=max
        w(&mut l, 0x10, 1); // CR ENABLE
        l.tick(2);
        assert_eq!(l.pending(), 0);
        l.tick(1);
        assert_ne!(r(&mut l, 0x00) & ISR_CMPM, 0);
        assert_eq!(l.pending(), 1);
    }

    #[test]
    fn lptim_oneshot_stops_at_arr() {
        let mut l = Lptim::new_for::<Stm32h743>("LPTIM1");
        w(&mut l, 0x18, 2); // ARR=2
        w(&mut l, 0x10, 1 | (1 << 2)); // ENABLE + SNGSTRT
        l.tick(5);
        assert_ne!(r(&mut l, 0x00) & ISR_ARRM, 0);
        // One-shot stopped: further ticks do not re-latch (clear first).
        w(&mut l, 0x04, 1 << 1);
        l.tick(5);
        assert_eq!(r(&mut l, 0x00) & ISR_ARRM, 0);
    }

    #[test]
    fn lptim_external_counts_edges() {
        let mut l = Lptim::new_for::<Stm32h743>("LPTIM1");
        w(&mut l, 0x0C, 1); // CFGR CKSEL: external
        w(&mut l, 0x18, 0xFFFF);
        w(&mut l, 0x10, 1); // ENABLE
        l.tick(100);
        assert_eq!(r(&mut l, 0x1C), 0);
        l.drive_inputs(0);
        l.drive_inputs(1);
        l.drive_inputs(1);
        l.drive_inputs(0);
        l.drive_inputs(1);
        assert_eq!(r(&mut l, 0x1C), 2);
    }

    #[test]
    fn lptim_prescaler_divides() {
        let mut l = Lptim::new_for::<Stm32h743>("LPTIM1");
        w(&mut l, 0x0C, 2 << 9); // PRESC=/4
        w(&mut l, 0x18, 0xFFFF);
        w(&mut l, 0x10, 1);
        l.tick(3);
        assert_eq!(r(&mut l, 0x1C), 0);
        l.tick(1);
        assert_eq!(r(&mut l, 0x1C), 1);
    }
}
