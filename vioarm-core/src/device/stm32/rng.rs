//! RNG — random number generator (`CR`/`SR`/`DR`; `0x48021800` on H7,
//! `0x50060800` on F4 — offsets resolved from the SVD either way).
//!
//! Enabling (`CR.RNGEN`) schedules fresh data one tick later: the tick
//! latches a fresh xorshift word into `DR` with `SR.DRDY`.
//! Reading `DR` clears `DRDY` and schedules the next word, so polling
//! firmware streams values. The xorshift seed is fixed (`0x12345678`,
//! reseeded on construction/reset): output is deterministic across reruns
//! by design — this models the register protocol, not entropy.
//!
//! Health monitoring follows silicon: stimulus bit 8 latches a clock error
//! (`SR.CECS`), bit 9 a seed error (`SR.SECS`); either halts generation
//! (clock errors only when `CR.CED` detection is enabled, i.e. clear —
//! absent `CED`, as on F4, any clock error halts) until cleared by writing
//! 0 or disabling `RNGEN`. `SR.CEIS`/`SEIS` mirror their flags gated by
//! `CR.IE`, and any of `DRDY`/`CECS`/`SECS` with `IE` raises the single
//! interrupt line. `DR` reads as zero while a seed error stands.
//! Deliberately unmodeled: startup latency beyond one tick, NIST
//! conditioning/health-test internals, `CONDRST` and low-power modes.

use crate::device::svd::{
    Access, Behavior, Device, PeripheralDef, RegisterDef, RegisterFile, Registers,
};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Fixed xorshift seed: deterministic output, rerun-identical by design.
const SEED: u32 = 0x12345678;

#[derive(Clone, Debug)]
struct RngBehavior {
    pending: bool,
    lfsr: u32,
    cr_off: u32,
    sr_off: u32,
    dr_off: u32,
    rnen_bit: u32,
    ie_bit: u32,
    ced_bit: u32,
    drdy_bit: u32,
    cecs_bit: u32,
    secs_bit: u32,
    ceis_bit: u32,
    seis_bit: u32,
}

impl Default for RngBehavior {
    fn default() -> Self {
        Self {
            pending: false,
            lfsr: SEED,
            cr_off: 0x00,
            sr_off: 0x04,
            dr_off: 0x08,
            rnen_bit: 1 << 2,
            ie_bit: 1 << 3,
            ced_bit: 1 << 5,
            drdy_bit: 1 << 0,
            cecs_bit: 1 << 1,
            secs_bit: 1 << 2,
            ceis_bit: 1 << 5,
            seis_bit: 1 << 6,
        }
    }
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
        if offset == self.cr_off {
            let word = view.merged(offset, val, shift);
            let was_en = view.get(offset) & self.rnen_bit != 0;
            let en = word & self.rnen_bit != 0;
            if en && !was_en {
                self.pending = true;
            }
            if !en {
                self.pending = false;
                view.set(self.sr_off, view.get(self.sr_off) & !self.drdy_bit);
            }
            self.sync_irq_flags(view);
            return word;
        }
        if offset == self.sr_off {
            // `CECS`/`SECS` are write-0-to-clear; `DRDY` and the `CEIS`/
            // `SEIS` mirrors are read-only and ignore writes. Return the
            // post-side-effect word: the framework stores the return over
            // the register, so a pre-computed value would clobber `sync`.
            let cur = view.get(offset);
            let word = view.merged(offset, val, shift);
            let mut next = cur;
            if word & self.cecs_bit == 0 {
                next &= !self.cecs_bit;
            }
            if word & self.secs_bit == 0 {
                next &= !self.secs_bit;
            }
            view.set(offset, next);
            self.sync_irq_flags(view);
            return view.get(offset);
        }
        view.merged(offset, val, shift)
    }
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == self.dr_off {
            // Reading `DR` clears `DRDY` and — with the generator running
            // and healthy — schedules the next word. Zero while seed-broken.
            if view.get(self.sr_off) & self.drdy_bit != 0 {
                view.set(self.sr_off, view.get(self.sr_off) & !self.drdy_bit);
                if self.generating(view) {
                    self.pending = true;
                }
                if view.get(self.sr_off) & self.secs_bit != 0 {
                    return 0;
                }
            }
        }
        value
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        if self.pending && self.generating(view) {
            let rnd = self.next();
            view.set(self.dr_off, rnd);
            view.set(self.sr_off, view.get(self.sr_off) | self.drdy_bit);
            self.pending = false;
        }
    }
    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(self.sr_off);
        let ie = view.get(self.cr_off) & self.ie_bit != 0;
        u32::from(ie && (sr & (self.drdy_bit | self.cecs_bit | self.secs_bit) != 0))
    }
    fn reset(&mut self) {
        self.pending = false;
        self.lfsr = SEED;
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        // Bit 8 is a clock failure, bit 9 a seed failure: both latch and
        // halt generation until cleared (write-0) or disabled.
        let mut sr = view.get(self.sr_off);
        if value & (1 << 8) != 0 {
            sr |= self.cecs_bit;
        }
        if value & (1 << 9) != 0 {
            sr |= self.secs_bit;
        }
        view.set(self.sr_off, sr);
        self.sync_irq_flags(view);
    }
}

impl RngBehavior {
    /// Whether the generator may produce: enabled, no clock error (unless
    /// detection is off), no seed error.
    fn generating(&self, view: &Registers<&mut [u32]>) -> bool {
        if view.get(self.cr_off) & self.rnen_bit == 0 {
            return false;
        }
        let sr = view.get(self.sr_off);
        if sr & self.secs_bit != 0 {
            return false;
        }
        if sr & self.cecs_bit != 0 {
            // Without a `CED` bit (F4) any clock error halts; with one,
            // only an enabled detector halts.
            if self.ced_bit == 0 {
                return false;
            }
            return view.get(self.cr_off) & self.ced_bit != 0;
        }
        true
    }

    /// `CEIS`/`SEIS` mirror their error flags gated by `IE`.
    fn sync_irq_flags(&self, view: &mut Registers<&mut [u32]>) {
        let sr = view.get(self.sr_off);
        let ie = view.get(self.cr_off) & self.ie_bit != 0;
        let mut next = sr;
        if sr & self.cecs_bit != 0 && ie {
            next |= self.ceis_bit;
        } else {
            next &= !self.ceis_bit;
        }
        if sr & self.secs_bit != 0 && ie {
            next |= self.seis_bit;
        } else {
            next &= !self.seis_bit;
        }
        view.set(self.sr_off, next);
    }
}

/// Random number generator.
#[derive(Debug)]
pub struct Rng {
    inner: RegisterFile<RngBehavior>,
}
impl Rng {
    /// A new `Rng` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("RNG").expect("RNG");
        Self {
            inner: RegisterFile::new(def, Self::behavior_for(def)),
        }
    }

    /// Resolve offsets and bits from the SVD table (plain `CR`/`SR`/`DR`
    /// with `RNGEN`/`IE`/`CED` and `DRDY`/`CECS`/`SECS`/`CEIS`/`SEIS`),
    /// falling back to the F4/H7 layout.
    fn behavior_for(def: &'static PeripheralDef) -> RngBehavior {
        let reg = |name: &str, fb: u32| {
            def.registers
                .iter()
                .find(|r| r.name == name)
                .map(|r| r.offset)
                .unwrap_or(fb)
        };
        let bit = |reg: &str, field: &str, fb: u32| {
            def.registers
                .iter()
                .find(|r| r.name == reg)
                .and_then(|r| r.fields.iter().find(|f| f.name == field))
                .map(|f| 1u32 << f.offset)
                .unwrap_or(fb)
        };
        let cr = reg("CR", 0x00);
        let sr = reg("SR", 0x04);
        RngBehavior {
            pending: false,
            lfsr: SEED,
            cr_off: cr,
            sr_off: sr,
            dr_off: reg("DR", 0x08),
            rnen_bit: bit("CR", "RNGEN", 1 << 2),
            ie_bit: bit("CR", "IE", 1 << 3),
            ced_bit: bit("CR", "CED", 0),
            drdy_bit: bit("SR", "DRDY", 1),
            cecs_bit: bit("SR", "CECS", 1 << 1),
            secs_bit: bit("SR", "SECS", 1 << 2),
            ceis_bit: bit("SR", "CEIS", 1 << 5),
            seis_bit: bit("SR", "SEIS", 1 << 6),
        }
    }

    /// Manual G0 RNG at `0x4002_5000` (`CR`/`SR`/`DR`) when SVD lacks the
    /// entry (`G041`). No IRQ in the tables, matching the donor window.
    #[must_use]
    pub fn new_g0() -> Self {
        const fn reg(name: &'static str, offset: u32, access: Access) -> RegisterDef {
            RegisterDef {
                name,
                offset,
                width: 32,
                access,
                reset: 0,
                fields: &[],
            }
        }
        static REGS: &[RegisterDef] = &[
            reg("CR", 0x00, Access::ReadWrite),
            reg("SR", 0x04, Access::ReadWrite),
            reg("DR", 0x08, Access::ReadOnly),
        ];
        static DEF: PeripheralDef = PeripheralDef {
            name: "RNG",
            base: 0x4002_5000,
            window: 0x400,
            irqs: &[],
            registers: REGS,
        };
        Self {
            inner: RegisterFile::new(&DEF, Self::behavior_for(&DEF)),
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
    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
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
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Rng, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }
    // CR/SR/DR layout shared by F4/H7 tables.
    const CR: u32 = 0x00;
    const SR: u32 = 0x04;
    const DR: u32 = 0x08;
    const RNGEN: u32 = 1 << 2;
    const IE: u32 = 1 << 3;
    const CED: u32 = 1 << 5;
    const DRDY: u32 = 1 << 0;
    const CECS: u32 = 1 << 1;
    const SECS: u32 = 1 << 2;
    const CEIS: u32 = 1 << 5;
    const SEIS: u32 = 1 << 6;

    fn enabled() -> Rng {
        let mut rng = Rng::new_for::<Stm32h743>();
        w(&mut rng, CR, RNGEN);
        rng
    }

    fn sample(rng: &mut Rng) -> u32 {
        rng.tick(1);
        assert_ne!(r(rng, SR) & DRDY, 0);
        r(rng, DR)
    }
    #[test]
    fn rng_enable_latches_drdy() {
        let mut rng = Rng::new_for::<Stm32h743>();
        w(&mut rng, 0x00, 1 << 2);
        assert_eq!(r(&mut rng, 0x04) & 1, 0);
        rng.tick(1);
        assert_ne!(r(&mut rng, 0x04) & 1, 0);
        let dr = r(&mut rng, 0x08);
        assert_ne!(dr, 0);
        // Reading DR clears DRDY and schedules the next word.
        assert_eq!(r(&mut rng, 0x04) & 1, 0, "DR read clears DRDY");
        rng.tick(1);
        assert_ne!(r(&mut rng, 0x04) & 1, 0, "next word scheduled");
    }

    /// Statistical smoke: 64 samples are all nonzero and distinct, with
    /// a sane bit balance (xorshift is deterministic, not entropic —
    /// reruns produce the identical stream).
    #[test]
    fn stream_is_distinct_and_balanced() {
        let mut rng = enabled();
        let mut seen = std::collections::HashSet::new();
        let mut ones = 0u32;
        for _ in 0..64 {
            let v = sample(&mut rng);
            assert_ne!(v, 0);
            assert!(seen.insert(v), "no repeats in 64 samples");
            ones += v.count_ones();
        }
        let total = 64 * 32;
        assert!(
            ones > total / 2 - total / 8 && ones < total / 2 + total / 8,
            "bit balance {ones}/{total}"
        );
        // Deterministic rerun: a fresh unit repeats the stream exactly.
        let mut head = Vec::new();
        let mut first = enabled();
        for _ in 0..4 {
            head.push(sample(&mut first));
        }
        let mut second = enabled();
        for (i, v) in head.iter().enumerate() {
            assert_eq!(sample(&mut second), *v, "rerun identical at {i}");
        }
    }

    /// Clock failure halts generation with `CECS`/`CEIS` and the interrupt
    /// line; clearing resumes. With detection disabled the data flows.
    #[test]
    fn clock_error_halts_and_recovers() {
        use crate::device::InterruptSource;
        let mut rng = enabled();
        w(&mut rng, CR, RNGEN | IE);
        rng.drive_inputs(1 << 8);
        assert_ne!(r(&mut rng, SR) & CECS, 0, "CECS latched");
        assert_ne!(r(&mut rng, SR) & CEIS, 0, "CEIS mirrors with IE");
        assert_eq!(rng.pending(), 1, "error line asserts");
        rng.tick(1);
        assert_eq!(r(&mut rng, SR) & DRDY, 0, "halted: no data");
        // Detection off: generation resumes despite the latched flag.
        w(&mut rng, CR, RNGEN | IE | CED);
        rng.tick(1);
        assert_ne!(r(&mut rng, SR) & DRDY, 0, "runs with CED");
        // Write-0 clears the flag and drops the line.
        w(&mut rng, SR, 0);
        assert_eq!(r(&mut rng, SR) & (CECS | CEIS), 0);
        // The CED tick left real data: drain it, then the line drops.
        let _ = r(&mut rng, DR);
        assert_eq!(rng.pending(), 0);
    }

    /// Seed failure zeroes `DR` reads with `SECS`/`SEIS`; clearing resumes.
    #[test]
    fn seed_error_zeroes_data_until_cleared() {
        use crate::device::InterruptSource;
        let mut rng = enabled();
        w(&mut rng, CR, RNGEN | IE);
        rng.tick(1);
        assert_ne!(r(&mut rng, DR), 0, "healthy data");
        rng.drive_inputs(1 << 9);
        assert_ne!(r(&mut rng, SR) & SECS, 0, "SECS latched");
        assert_ne!(r(&mut rng, SR) & SEIS, 0, "SEIS mirrors with IE");
        assert_eq!(rng.pending(), 1);
        rng.tick(1);
        assert_eq!(r(&mut rng, SR) & DRDY, 0, "halted: no data");
        w(&mut rng, SR, 0);
        rng.tick(1);
        assert_ne!(r(&mut rng, SR) & DRDY, 0, "resumes after clear");
    }

    /// Disabling stops scheduling; `DRDY` clears.
    #[test]
    fn disable_stops_generation() {
        let mut rng = enabled();
        rng.tick(1);
        assert_ne!(r(&mut rng, SR) & DRDY, 0);
        w(&mut rng, CR, 0);
        assert_eq!(r(&mut rng, SR) & DRDY, 0);
        rng.tick(1);
        assert_eq!(r(&mut rng, SR) & DRDY, 0, "stays stopped");
    }
}
