//! RNG `0x48021800` (H7) / `0x50061800` (F4) — random number generator.
//! `CR.RNGEN@2` enable, `SR.DRDY@0` latches 1-tick with `DR` random, `SR.CECS@1` etc latch.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR_RNGEN: u32 = 1 << 2;
const SR_DRDY: u32 = 1 << 0;
const SR_CECS: u32 = 1 << 1;

#[derive(Clone, Debug, Default)]
struct RngBehavior {
    pending: bool,
    lfsr: u32,
}

impl RngBehavior {
    fn next(&mut self) -> u32 {
        // xorshift
        let mut x = self.lfsr;
        if x == 0 {
            x = 0x12345678;
        }
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.lfsr = x;
        x
    }
}

impl Behavior for RngBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            let en = word & CR_RNGEN != 0;
            let was_en = view.get(0x00) & CR_RNGEN != 0;
            if en && !was_en {
                self.pending = true;
                view.set(0x08, view.get(0x08) & !SR_DRDY); // SR at 0x08? Actually SR at 0x04?
                                                           // For H7 RNG, SR at 0x04, DR at 0x08
                                                           // We handle both
                view.set(0x04, view.get(0x04) & !SR_DRDY);
            }
            if !en {
                self.pending = false;
                view.set(0x04, view.get(0x04) & !SR_DRDY);
                view.set(0x08, view.get(0x08) & !SR_DRDY);
            }
        }
        if offset == 0x04 || offset == 0x08 {
            // SR is read-only, but W1C for CECS etc
            // For test, we allow write to clear DRDY
            if word & 1 != 0 {
                view.set(offset, view.get(offset) & !SR_DRDY);
                self.pending = false;
                return view.get(offset);
            }
        }
        word
    }
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == 0x08 || offset == 0x0C {
            // DR at 0x08 for H7, 0x0C for F4? Handle both
            // Reading DR clears DRDY
            view.set(offset, value);
            // Actually DR read should clear DRDY in SR
            let sr_off = if view.try_get(0x04).is_some() {
                0x04
            } else {
                0x08
            };
            view.set(sr_off, view.get(sr_off) & !SR_DRDY);
            // Schedule next
            if view.get(0x00) & CR_RNGEN != 0 {
                self.pending = true;
            }
        }
        value
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending {
            let sr_off = if view.try_get(0x04).is_some() && view.try_get(0x08).is_some() {
                // H7: SR at 0x04, DR at 0x08
                0x04
            } else {
                0x04
            };
            let dr_off = if view.try_get(0x08).is_some() {
                0x08
            } else {
                0x08
            };
            let rnd = self.next();
            view.set(sr_off, view.get(sr_off) | SR_DRDY);
            view.set(dr_off, rnd);
            self.pending = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(0x04);
        let cr = view.get(0x00);
        // IE at CR bit 3?
        let ie = (cr >> 3) & 1;
        u32::from(sr & SR_DRDY != 0 && ie != 0)
    }
    fn reset(&mut self) {
        self.pending = false;
        self.lfsr = 0x12345678;
    }
}

#[derive(Debug)]
pub struct Rng {
    inner: RegisterFile<RngBehavior>,
}
impl Rng {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("RNG").expect("RNG");
        Self {
            inner: RegisterFile::new(
                def,
                RngBehavior {
                    lfsr: 0x12345678,
                    pending: false,
                },
            ),
        }
    }
}
impl Peripheral for Rng {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }
    fn read(&mut self, a: u32) -> Result<u8, BusError> {
        self.inner.read(a)
    }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> {
        self.inner.write(a, v)
    }
    fn reset(&mut self) {
        self.inner.reset();
    }
    fn tick(&mut self, d: u64) {
        self.inner.tick(d);
    }
    fn name(&self) -> &'static str {
        "rng"
    }
}
impl InterruptSource for Rng {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, l: u32) {
        self.inner.acknowledge(l);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32h743::Stm32h743;
    fn w(c: &mut Rng, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> 8 * i) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Rng, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << 8 * i;
        }
        v
    }
    #[test]
    fn rng_enable_latches_drdy() {
        let mut rng = Rng::new_for::<Stm32h743>();
        w(&mut rng, 0x00, CR_RNGEN);
        assert_eq!(r(&mut rng, 0x04) & SR_DRDY, 0);
        rng.tick(1);
        assert_ne!(r(&mut rng, 0x04) & SR_DRDY, 0);
        let dr = r(&mut rng, 0x08);
        assert_ne!(dr, 0);
        // Reading DR should clear DRDY and schedule next, but our on_read clears
        // So after read, DRDY cleared
        assert_eq!(r(&mut rng, 0x04) & SR_DRDY, 0, "DR read clears DRDY");
    }
}
