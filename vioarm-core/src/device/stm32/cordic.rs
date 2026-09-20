//! CORDIC co-processor (G4 `CORDIC@0x40020C00`) — fixed-point math engine.
//!
//! Circular functions (angles in q1.31 radians/π): cosine (0), sine (1),
//! phase (2), modulus (3), arctangent (4). Hyperbolic (raw q1.31):
//! cosh (5), sinh (6), atanh (7). Natural log (8) and square root (9)
//! stay unmodeled (documented).
//!
//! The engine is an integer CORDIC (30 circular iterations; hyperbolic
//! with the 4/13 repeats), so no host float is involved: rotation starts
//! from the inverse-gain constant, vectoring divides the radius back out.
//! `SCALE` arithmetic-right-shifts every result with saturation
//! (documented choice matching the "multiply RES by 2^n" programming
//! model). `PRECISION` latches (the core always runs 24 iterations;
//! tolerance-tested so the difference is unobservable).
//! `ARGSIZE`/`RESSIZE` select 32- or low-halfword 16-bit words.
//! Arguments: `NARGS+1` `WDATA` writes arm one computation (full-word
//! completion like the SPI/I2S staging); `RDATA` pops `NRES+1` results
//! with `RRDY` while results remain. `IEN` gates the single line;
//! `DMAREN`/`DMAWEN` latch.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CSR: u32 = 0x00;
const WDATA: u32 = 0x04;
const RDATA: u32 = 0x08;

const CSR_FUNC_MASK: u32 = 0xF;
const CSR_SCALE_SHIFT: u32 = 8;
const CSR_IEN: u32 = 1 << 16;
const CSR_NRES: u32 = 1 << 19;
const CSR_NARGS: u32 = 1 << 20;
const CSR_RESSIZE: u32 = 1 << 21;
const CSR_ARGSIZE: u32 = 1 << 22;
const CSR_RRDY: u32 = 1 << 31;

const FUNC_COS: u32 = 0;
const FUNC_SIN: u32 = 1;
const FUNC_PHASE: u32 = 2;
const FUNC_MOD: u32 = 3;
const FUNC_ATAN: u32 = 4;
const FUNC_COSH: u32 = 5;
const FUNC_SINH: u32 = 6;
const FUNC_ATANH: u32 = 7;

/// Circular `atan(2^-i)/π` in q31 (30 iterations for q31-clean output).
const ATAN_Q31: [i64; 30] = [
    536870912, 316933406, 167458907, 85004756, 42667331, 21354465, 10679838, 5340245, 2670163,
    1335087, 667544, 333772, 166886, 83443, 41722, 20861, 10430, 5215, 2608, 1304, 652, 326, 163,
    81, 41, 20, 10, 5, 3, 1,
];
/// Hyperbolic `atanh(2^-j)` in raw q31 (`j = 1..30`).
const ATANH_Q31: [i64; 30] = [
    1179625963, 548494837, 269846813, 134392901, 67130722, 33557163, 16777557, 8388651, 4194309,
    2097153, 1048576, 524288, 262144, 131072, 65536, 32768, 16384, 8192, 4096, 2048, 1024, 512,
    256, 128, 64, 32, 16, 8, 4, 2,
];
/// Circular inverse gain (start vector length).
const K_Q31: i64 = 1304065748;
/// Hyperbolic inverse gain in q32 (headroom for the >1 start vector).
const KH_Q32: i64 = 5186160416;
const Q31_MAX: i64 = 0x7FFF_FFFF;
const Q31_MIN: i64 = -0x8000_0000;

/// Circular rotation: `(cos θ, sin θ)` for `θ` in angle/π q31.
fn circ_rot(theta: i64) -> (i64, i64) {
    // Range-reduce into [-π/2, π/2], tracking the quadrant flip.
    let mut t = theta;
    let mut neg = false;
    while t > 0x4000_0000 {
        t -= 0x8000_0000;
        neg = !neg;
    }
    while t < -0x4000_0000 {
        t += 0x8000_0000;
        neg = !neg;
    }
    let (mut x, mut y, mut z) = (K_Q31, 0i64, t);
    for (i, a) in ATAN_Q31.iter().enumerate() {
        let d = if z >= 0 { 1 } else { -1 };
        let xn = x - d * (y >> i);
        let yn = y + d * (x >> i);
        z -= d * a;
        x = xn;
        y = yn;
    }
    if neg {
        (-x, -y)
    } else {
        (x, y)
    }
}

/// Circular vectoring: `(K·r, angle/π)` for inputs `(x, y)`.
fn circ_vec(x0: i64, y0: i64) -> (i64, i64) {
    let (mut x, mut y, mut z) = (x0, y0, 0i64);
    for (i, a) in ATAN_Q31.iter().enumerate() {
        let d = if y >= 0 { -1 } else { 1 };
        let xn = x - d * (y >> i);
        let yn = y + d * (x >> i);
        z -= d * a;
        x = xn;
        y = yn;
    }
    (x, z)
}

/// Hyperbolic rotation in q32 headroom: returns `(cosh t, sinh t)` each
/// pre-shifted to q31 range (may exceed it; `res_word` saturates).
fn hyp_rot(t: i64) -> (i64, i64) {
    let (mut x, mut y, mut z) = (KH_Q32, 0i64, t);
    for j in 1..=30 {
        // Convergence repeats (`j = 4, 13`) reuse the same step.
        for _ in 0..(if j == 4 || j == 13 { 2 } else { 1 }) {
            let a = ATANH_Q31[j - 1];
            let d = if z >= 0 { 1 } else { -1 };
            let xn = x + d * (y >> j);
            let yn = y + d * (x >> j);
            z -= d * a;
            x = xn;
            y = yn;
        }
    }
    (x >> 1, y >> 1)
}

/// Hyperbolic vectoring of `(x, y)` in q32 headroom: returns
/// `(junk, atanh(y/x))` with the angle in q31.
fn hyp_vec(x0: i64, y0: i64) -> (i64, i64) {
    let (mut x, mut y, mut z) = (x0 << 1, y0 << 1, 0i64);
    for j in 1..=30 {
        for _ in 0..(if j == 4 || j == 13 { 2 } else { 1 }) {
            let a = ATANH_Q31[j - 1];
            let d = if y >= 0 { -1 } else { 1 };
            let xn = x + d * (y >> j);
            let yn = y + d * (x >> j);
            z -= d * a;
            x = xn;
            y = yn;
        }
    }
    (x >> 1, z)
}

#[derive(Clone, Debug, Default)]
struct CordicBehavior {
    args: [u32; 2],
    nargs_got: u8,
    results: [u32; 2],
    nres_got: u8,
    result_ready: bool,
    compute_pending: bool,
    w_acc: u32,
    w_mask: u8,
    r_word: u32,
    r_lane: u8,
}

impl CordicBehavior {
    fn nargs_needed(view: &Registers<&mut [u32]>) -> u8 {
        if view.get(CSR) & CSR_NARGS != 0 {
            2
        } else {
            1
        }
    }

    fn nres_out(view: &Registers<&mut [u32]>) -> u8 {
        if view.get(CSR) & CSR_NRES != 0 {
            2
        } else {
            1
        }
    }

    fn arg_word(view: &Registers<&mut [u32]>, w: u32) -> i64 {
        if view.get(CSR) & CSR_ARGSIZE != 0 {
            i64::from((w & 0xFFFF) as u16 as i16) << 16
        } else {
            i64::from(w as i32)
        }
    }

    fn res_word(view: &Registers<&mut [u32]>, v: i64) -> u32 {
        // Shift first, saturate after: hyperbolic results carry one
        // headroom bit above q31.
        let scale = (view.get(CSR) >> CSR_SCALE_SHIFT) & 0x7;
        let clamped = (v >> scale).clamp(Q31_MIN, Q31_MAX) as i32;
        if view.get(CSR) & CSR_RESSIZE != 0 {
            (clamped.clamp(-0x8000, 0x7FFF) & 0xFFFF) as u32
        } else {
            clamped as u32
        }
    }

    fn compute(view: &mut Registers<&mut [u32]>, args: [i64; 2]) -> [i64; 2] {
        let func = view.get(CSR) & CSR_FUNC_MASK;
        match func {
            FUNC_COS => {
                let (x, y) = circ_rot(args[0]);
                [x, y]
            }
            FUNC_SIN => {
                let (x, y) = circ_rot(args[0]);
                [y, x]
            }
            FUNC_PHASE => {
                let (r, z) = circ_vec(args[0], args[1]);
                [z, (r * K_Q31) >> 31]
            }
            FUNC_MOD => {
                let (r, z) = circ_vec(args[0], args[1]);
                [(r * K_Q31) >> 31, z]
            }
            FUNC_ATAN => {
                // atan(u): vector (0.5, u/2); the angle is scale-invariant.
                let (_, z) = circ_vec(1 << 30, args[0] >> 1);
                [z, 0]
            }
            FUNC_COSH => {
                let (x, y) = hyp_rot(args[0]);
                [x, y]
            }
            FUNC_SINH => {
                let (x, y) = hyp_rot(args[0]);
                [y, x]
            }
            FUNC_ATANH => {
                // atanh(u): hyperbolic vectoring of (0.5, u/2).
                let (_, z) = hyp_vec(1 << 30, args[0] >> 1);
                [z, 0]
            }
            _ => [0, 0],
        }
    }
}

impl Behavior for CordicBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            WDATA => {
                // Lanes assemble one argument word; a completed word arms.
                if shift == 0 {
                    self.w_acc = 0;
                    self.w_mask = 0;
                }
                self.w_acc |= u32::from(val) << shift;
                self.w_mask |= 1 << (shift / 8);
                if self.w_mask == 0xF {
                    let w = self.w_acc;
                    self.w_acc = 0;
                    self.w_mask = 0;
                    if self.nargs_got < 2 {
                        self.args[usize::from(self.nargs_got)] = w;
                        self.nargs_got += 1;
                    }
                    if self.nargs_got >= Self::nargs_needed(view) {
                        self.nargs_got = 0;
                        self.nres_got = 0;
                        self.result_ready = false;
                        self.compute_pending = true;
                    }
                }
                view.merged(offset, val, shift)
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            0x00 => {
                // RRDY reflects result availability.
                if self.result_ready {
                    value | CSR_RRDY
                } else {
                    value & !CSR_RRDY
                }
            }
            RDATA => {
                if self.r_lane == 0 {
                    self.r_word = if self.nres_got < Self::nres_out(view) {
                        self.nres_got += 1;
                        self.results[usize::from(self.nres_got - 1)]
                    } else {
                        0
                    };
                    if self.nres_got >= Self::nres_out(view) {
                        self.result_ready = false;
                    }
                }
                let w = self.r_word;
                self.r_lane += 1;
                if self.r_lane == 4 {
                    self.r_lane = 0;
                }
                w
            }
            _ => value,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        if self.compute_pending {
            self.compute_pending = false;
            let a0 = Self::arg_word(view, self.args[0]);
            let a1 = Self::arg_word(view, self.args[1]);
            let [r0, r1] = Self::compute(view, [a0, a1]);
            self.results = [Self::res_word(view, r0), Self::res_word(view, r1)];
            self.result_ready = true;
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(view.get(0x00) & CSR_IEN != 0 && self.result_ready)
    }

    fn reset(&mut self) {
        self.nargs_got = 0;
        self.nres_got = 0;
        self.result_ready = false;
        self.compute_pending = false;
        self.w_acc = 0;
        self.w_mask = 0;
        self.r_word = 0;
        self.r_lane = 0;
    }
}

/// CORDIC co-processor.
#[derive(Debug)]
pub struct Cordic {
    inner: RegisterFile<CordicBehavior>,
}

impl Cordic {
    /// A new `CORDIC` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `CORDIC`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("CORDIC").expect("generated tables contain CORDIC");
        Self {
            inner: RegisterFile::new(def, CordicBehavior::default()),
        }
    }
}

impl Peripheral for Cordic {
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

    fn name(&self) -> &'static str {
        "cordic"
    }
}

impl InterruptSource for Cordic {
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
    use crate::device::svd::stm32g474xx::Stm32g474;
    use crate::device::{InterruptSource, Peripheral};

    const BASE: u32 = 0x4002_0C00;

    #[allow(clippy::cast_possible_truncation)]
    fn w(c: &mut Cordic, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            c.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(c: &mut Cordic, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = c.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    /// Program `func` (single result unless `two`), run one computation,
    /// and return the result words.
    fn run(c: &mut Cordic, func: u32, two: bool, args: &[u32]) -> Vec<u32> {
        let nargs = if args.len() > 1 { CSR_NARGS } else { 0 };
        w(c, CSR, func | (u32::from(two) << 19) | nargs);
        for a in args {
            w(c, WDATA, *a);
        }
        c.tick(1);
        assert_ne!(r(c, CSR) & CSR_RRDY, 0, "RRDY latches");
        (0..if two { 2 } else { 1 }).map(|_| r(c, RDATA)).collect()
    }

    fn cordic() -> Cordic {
        let mut c = Cordic::new_for::<Stm32g474>();
        c.reset();
        c
    }

    /// Assert `got` is within `tol` LSBs of `want` (signed q31 distance).
    fn close(got: u32, want: i64, tol: i64) {
        let d = (i64::from(got as i32) - want).abs();
        assert!(d <= tol, "got {got:#x}, want {want:#x}, d={d}");
    }

    #[test]
    fn cos60_and_sin60() {
        let mut c = cordic();
        // θ = 60° = 2^31/3 in angle/π q31.
        let out = run(&mut c, FUNC_COS, true, &[715827883]);
        close(out[0], 1073741824, 32);
        close(out[1], 1859775393, 32);
        let out = run(&mut c, FUNC_SIN, true, &[715827883]);
        close(out[0], 1859775393, 32);
        close(out[1], 1073741824, 32);
    }

    #[test]
    fn phase_and_modulus() {
        let mut c = cordic();
        // v = (0.3, 0.4): phase = atan2/π, modulus = 0.5.
        let out = run(&mut c, FUNC_PHASE, true, &[644245094, 858993459]);
        close(out[0], 633866811, 32);
        close(out[1], 1073741824, 64);
        let out = run(&mut c, FUNC_MOD, false, &[644245094, 858993459]);
        close(out[0], 1073741824, 64);
    }

    #[test]
    fn atan_half() {
        let mut c = cordic();
        let out = run(&mut c, FUNC_ATAN, false, &[1073741824]);
        close(out[0], 316933406, 32);
    }

    #[test]
    fn cosh_sinh_scaled() {
        let mut c = cordic();
        // t = 0.5, SCALE = 1: results arrive halved.
        w(&mut c, CSR, 5 | (1 << 8));
        w(&mut c, WDATA, 1073741824);
        c.tick(1);
        close(r(&mut c, RDATA), 1210779161, 64);
        let mut c = cordic();
        w(&mut c, CSR, 6 | (1 << 8));
        w(&mut c, WDATA, 1073741824);
        c.tick(1);
        close(r(&mut c, RDATA), 559521824, 64);
    }

    #[test]
    fn atanh_quarter_scaled() {
        let mut m = cordic();
        w(&mut m, CSR, 7 | (1 << 8));
        w(&mut m, WDATA, 536870912);
        m.tick(1);
        close(r(&mut m, RDATA), 274247418, 64);
    }

    #[test]
    fn irq_and_sizes() {
        let mut c = cordic();
        w(&mut c, CSR, FUNC_COS | CSR_IEN);
        w(&mut c, WDATA, 0);
        c.tick(1);
        assert_eq!(c.pending(), 1, "IEN-gated RRDY line");
        // 16-bit words: low halfword in and out.
        let mut c = cordic();
        w(&mut c, CSR, FUNC_COS | (1 << 22) | (1 << 21));
        w(&mut c, WDATA, 0x0000_0000);
        c.tick(1);
        assert_eq!(r(&mut c, RDATA) & 0xFFFF, 0x7FFF, "cos(0) saturates q15");
    }
}
