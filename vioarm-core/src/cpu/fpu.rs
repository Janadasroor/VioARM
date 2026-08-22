//! IEEE 754 single-precision floating-point arithmetic.
//!
//! This is a Rust port of QEMU's softfloat implementation for float32,
//! using the decomposed-parts (`FloatParts64`) representation. It
//! implements the same algorithms and exception flag behaviour as QEMU,
//! so differential testing against QEMU's `cortex-m4` model is exact.
//!
//! The public functions take raw `u32` bit patterns (in and out) and a
//! [`FloatStatus`] which carries the rounding mode, flush settings and the
//! sticky exception flags.

/// Decomposition binary point: the fraction is normalized with an implicit
/// bit at bit 63, i.e. `frac in [2^63, 2^64)` for normal numbers.
pub const DECOMPOSED_BINARY_POINT: i32 = 63;
/// The implicit (integer) bit of a normalized fraction.
pub const DECOMPOSED_IMPLICIT_BIT: u64 = 1 << 63;

const F32_EXP_BIAS: i32 = 127;
const F32_EXP_MAX: i32 = 255;
const F32_FRAC_SIZE: i32 = 23;
/// `64 - frac_size - 1`: the number of trailing bits kept for rounding.
const F32_FRAC_SHIFT: i32 = 40;
const F32_ROUND_MASK: u64 = (1u64 << F32_FRAC_SHIFT) - 1;
const F32_FRAC_LSB: u64 = F32_ROUND_MASK + 1;
const F32_FRAC_LSBM1: u64 = F32_ROUND_MASK ^ (F32_ROUND_MASK >> 1);
const F32_ROUNDEVEN_MASK: u64 = F32_ROUND_MASK | F32_FRAC_LSB;

const F64_EXP_BIAS: i32 = 1023;
const F64_EXP_MAX: i32 = 2047;
const F64_FRAC_SIZE: i32 = 52;
/// `64 - frac_size - 1`: the number of trailing bits kept for rounding.
const F64_FRAC_SHIFT: i32 = 11;
const F64_ROUND_MASK: u64 = (1u64 << F64_FRAC_SHIFT) - 1;
const F64_FRAC_LSB: u64 = F64_ROUND_MASK + 1;
const F64_FRAC_LSBM1: u64 = F64_ROUND_MASK ^ (F64_ROUND_MASK >> 1);
const F64_ROUNDEVEN_MASK: u64 = F64_ROUND_MASK | F64_FRAC_LSB;

/// The IEEE format an operation is carried out at. The decomposed `Parts`
/// representation is shared by both widths; the width only selects the
/// pack/unpack parameters (bias, fraction shift, exponent max).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Width {
    /// IEEE binary32 (`float`, VFP `S` registers).
    Single,
    /// IEEE binary64 (`double`, FPv5 `D` registers).
    Double,
}

impl Width {
    #[inline]
    fn exp_bias(self) -> i32 {
        match self {
            Width::Single => F32_EXP_BIAS,
            Width::Double => F64_EXP_BIAS,
        }
    }

    #[inline]
    fn exp_max(self) -> i32 {
        match self {
            Width::Single => F32_EXP_MAX,
            Width::Double => F64_EXP_MAX,
        }
    }

    #[inline]
    fn frac_size(self) -> i32 {
        match self {
            Width::Single => F32_FRAC_SIZE,
            Width::Double => F64_FRAC_SIZE,
        }
    }

    #[inline]
    fn frac_shift(self) -> i32 {
        match self {
            Width::Single => F32_FRAC_SHIFT,
            Width::Double => F64_FRAC_SHIFT,
        }
    }

    #[inline]
    fn round_mask(self) -> u64 {
        match self {
            Width::Single => F32_ROUND_MASK,
            Width::Double => F64_ROUND_MASK,
        }
    }

    #[inline]
    fn frac_lsbm1(self) -> u64 {
        match self {
            Width::Single => F32_FRAC_LSBM1,
            Width::Double => F64_FRAC_LSBM1,
        }
    }

    #[inline]
    fn roundeven_mask(self) -> u64 {
        match self {
            Width::Single => F32_ROUNDEVEN_MASK,
            Width::Double => F64_ROUNDEVEN_MASK,
        }
    }
}

/// Floating-point rounding modes (QEMU `FloatRoundMode`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoundMode {
    /// Round to nearest, ties to even.
    NearestEven,
    /// Round towards negative infinity.
    Down,
    /// Round towards positive infinity.
    Up,
    /// Round towards zero.
    ToZero,
}

/// The IEEE exception flags, in QEMU's bit numbering.
pub const FLAG_INVALID: u32 = 0x0001;
pub const FLAG_DIVBYZERO: u32 = 0x0002;
pub const FLAG_OVERFLOW: u32 = 0x0004;
pub const FLAG_UNDERFLOW: u32 = 0x0008;
pub const FLAG_INEXACT: u32 = 0x0010;
pub const FLAG_INPUT_DENORMAL_FLUSHED: u32 = 0x0020;
pub const FLAG_OUTPUT_DENORMAL_FLUSHED: u32 = 0x0040;
pub const FLAG_INVALID_ISI: u32 = 0x0080;
pub const FLAG_INVALID_IMZ: u32 = 0x0100;
pub const FLAG_INVALID_IDI: u32 = 0x0200;
pub const FLAG_INVALID_ZDZ: u32 = 0x0400;
pub const FLAG_INVALID_SQRT: u32 = 0x0800;
pub const FLAG_INVALID_CVTI: u32 = 0x1000;
pub const FLAG_INVALID_SNAN: u32 = 0x2000;
pub const FLAG_INPUT_DENORMAL_USED: u32 = 0x4000;

/// The per-operation floating-point status word.
///
/// `flags` accumulates sticky exception flags exactly like QEMU's
/// `float_status.float_exception_flags`; the ARM core folds them into
/// FPSCR's cumulative exception bits after each instruction.
#[derive(Clone, Debug)]
pub struct FloatStatus {
    /// Sticky exception flags (see `FLAG_*`).
    pub flags: u32,
    /// Current rounding mode.
    pub rounding_mode: RoundMode,
    /// Flush denormal results to zero (FPSCR.FZ).
    pub flush_to_zero: bool,
    /// Flush denormal inputs to zero (FPSCR.FZ).
    pub flush_inputs_to_zero: bool,
    /// Return the default NaN for NaN results (FPSCR.DN).
    pub default_nan_mode: bool,
    /// The IEEE format the operation packs/unpacks at. Arithmetic cores are
    /// shared by both widths; converts switch this between phases.
    pub width: Width,
}

impl FloatStatus {
    /// A status word with QEMU's default ARM behaviours: rounding to
    /// nearest-even, no flush, no default NaN.
    pub fn new(rounding_mode: RoundMode) -> Self {
        Self {
            flags: 0,
            rounding_mode,
            flush_to_zero: false,
            flush_inputs_to_zero: false,
            default_nan_mode: false,
            width: Width::Single,
        }
    }

    /// Read and clear the sticky exception flags.
    pub fn take_flags(&mut self) -> u32 {
        let f = self.flags;
        self.flags = 0;
        f
    }

    /// OR `flags` into the sticky exception word.
    fn raise(&mut self, flags: u32) {
        self.flags |= flags;
    }
}

/// The class of a decomposed floating-point value (QEMU `FloatClass`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FloatClass {
    Zero,
    Normal,
    Denormal,
    Inf,
    QNaN,
    SNaN,
}

/// A decomposed floating-point value (QEMU `FloatParts64`).
///
/// The exponent is unbiased and the fraction normalized with the implicit
/// bit at bit 63. Rounding bits live in the low `F32_FRAC_SHIFT` bits.
#[derive(Clone, Copy, Debug)]
struct Parts {
    sign: bool,
    exp: i32,
    frac: u64,
    cls: FloatClass,
}

/// `1u << (cls as u32)`, matching QEMU's `float_cmask`.
fn cmask(cls: FloatClass) -> u32 {
    1 << cls as u32
}

const CMASK_ZERO: u32 = 1 << FloatClass::Zero as u32;
const CMASK_NORMAL: u32 = 1 << FloatClass::Normal as u32;
const CMASK_DENORMAL: u32 = 1 << FloatClass::Denormal as u32;
const CMASK_INF: u32 = 1 << FloatClass::Inf as u32;
const CMASK_QNAN: u32 = 1 << FloatClass::QNaN as u32;
const CMASK_SNAN: u32 = 1 << FloatClass::SNaN as u32;

const CMASK_INFZERO: u32 = CMASK_ZERO | CMASK_INF;
const CMASK_ANYNAN: u32 = CMASK_QNAN | CMASK_SNAN;
const CMASK_ANYNORM: u32 = CMASK_NORMAL | CMASK_DENORMAL;

/// QEMU's `cmask_is_only_normals`.
fn cmask_only_normals(cmask: u32) -> bool {
    cmask & !CMASK_ANYNORM == 0
}

/// QEMU's `is_anynorm`.
fn is_anynorm(cls: FloatClass) -> bool {
    cmask(cls) & CMASK_ANYNORM != 0
}

/// QEMU's `is_nan`.
fn is_nan(cls: FloatClass) -> bool {
    cls == FloatClass::QNaN || cls == FloatClass::SNaN
}

/// QEMU's `is_snan`.
fn is_snan(cls: FloatClass) -> bool {
    cls == FloatClass::SNaN
}

/// QEMU's `is_qnan`.
fn is_qnan(cls: FloatClass) -> bool {
    cls == FloatClass::QNaN
}

// ---------------------------------------------------------------------------
// Fraction helpers (QEMU's `frac64_*`).
// ---------------------------------------------------------------------------

fn frac_eqz(frac: u64) -> bool {
    frac == 0
}

fn frac_clear(frac: &mut u64) {
    *frac = 0;
}

fn frac_neg(frac: &mut u64) {
    *frac = frac.wrapping_neg();
}

fn frac_shl(frac: &mut u64, c: i32) {
    if c != 0 {
        *frac <<= c;
    }
}

fn frac_shr(frac: &mut u64, c: i32) {
    if c != 0 {
        *frac >>= c;
    }
}

/// QEMU's `frac64_shrjam`: shift right, sticky the discarded bits.
fn frac_shrjam(frac: &mut u64, c: i32) {
    if c != 0 {
        if c < 64 {
            *frac = (*frac >> c) | u64::from(*frac << (64 - c) != 0);
        } else {
            *frac = u64::from(*frac != 0);
        }
    }
}

/// QEMU's `frac64_add`: returns true on carry out of the fraction.
fn frac_add(a: &mut u64, b: u64) -> bool {
    let (r, carry) = a.overflowing_add(b);
    *a = r;
    carry
}

fn frac_addi(a: &mut u64, c: u64) -> bool {
    frac_add(a, c)
}

fn frac_sub(a: &mut u64, b: u64) -> bool {
    let (r, borrow) = a.overflowing_sub(b);
    *a = r;
    borrow
}

fn frac_allones(frac: &mut u64) {
    *frac = u64::MAX;
}

/// QEMU's `frac64_normalize`: shift the fraction so its top bit is set,
/// returning the number of leading zeros shifted.
fn frac_normalize(frac: &mut u64) -> i32 {
    if *frac == 0 {
        return 64;
    }
    let shift = (*frac).leading_zeros() as i32;
    frac_shl(frac, shift);
    shift
}

// ---------------------------------------------------------------------------
// Canonicalize / uncanonicalize (QEMU `parts64_canonicalize`).
// ---------------------------------------------------------------------------

/// QEMU's `parts64_canonicalize`.
fn parts_canonicalize(p: &mut Parts, status: &mut FloatStatus) {
    let w = status.width;
    if p.exp == 0 {
        if frac_eqz(p.frac) {
            p.cls = FloatClass::Zero;
        } else if status.flush_to_zero || status.flush_inputs_to_zero {
            status.raise(FLAG_INPUT_DENORMAL_FLUSHED);
            p.cls = FloatClass::Zero;
            frac_clear(&mut p.frac);
        } else {
            let shift = frac_normalize(&mut p.frac);
            p.cls = FloatClass::Denormal;
            p.exp = w.frac_shift() - w.exp_bias() - shift + 1;
        }
    } else if p.exp < w.exp_max() {
        p.cls = FloatClass::Normal;
        p.exp -= w.exp_bias();
        frac_shl(&mut p.frac, w.frac_shift());
        p.frac |= DECOMPOSED_IMPLICIT_BIT;
    } else if frac_eqz(p.frac) {
        p.cls = FloatClass::Inf;
    } else {
        frac_shl(&mut p.frac, w.frac_shift());
        // The quiet bit (raw bit frac_size-1) lands on bit 62 in the
        // decomposed fraction, just below the implicit bit.
        p.cls = if p.frac & (DECOMPOSED_IMPLICIT_BIT >> 1) != 0 {
            FloatClass::QNaN
        } else {
            FloatClass::SNaN
        };
    }
}

/// QEMU's `parts64_uncanon_normal`.
fn parts_uncanon_normal(p: &mut Parts, status: &mut FloatStatus) {
    let exp_max = status.width.exp_max();
    let frac_shift = status.width.frac_shift();
    let round_mask = status.width.round_mask();
    let frac_lsbm1 = status.width.frac_lsbm1();
    let roundeven_mask = status.width.roundeven_mask();
    let mut inc: u64;
    let mut overflow_norm = false;
    let mut exp;
    let mut flags = 0u32;

    match status.rounding_mode {
        RoundMode::NearestEven => {
            inc = if p.frac & roundeven_mask != frac_lsbm1 {
                frac_lsbm1
            } else {
                0
            };
        }
        RoundMode::ToZero => {
            overflow_norm = true;
            inc = 0;
        }
        RoundMode::Up => {
            inc = if p.sign { 0 } else { round_mask };
            overflow_norm = p.sign;
        }
        RoundMode::Down => {
            inc = if p.sign { round_mask } else { 0 };
            overflow_norm = !p.sign;
        }
    }

    exp = p.exp + status.width.exp_bias();
    if exp > 0 {
        if p.frac & round_mask != 0 {
            flags |= FLAG_INEXACT;
            if frac_addi(&mut p.frac, inc) {
                frac_shr(&mut p.frac, 1);
                p.frac |= DECOMPOSED_IMPLICIT_BIT;
                exp += 1;
            }
            p.frac &= !round_mask;
        }

        if exp >= exp_max {
            flags |= FLAG_OVERFLOW;
            if overflow_norm {
                flags |= FLAG_INEXACT;
                exp = exp_max - 1;
                frac_allones(&mut p.frac);
                p.frac &= !round_mask;
            } else {
                flags |= FLAG_INEXACT;
                p.cls = FloatClass::Inf;
                exp = exp_max;
                frac_clear(&mut p.frac);
            }
        }
        frac_shr(&mut p.frac, frac_shift);
    } else if status.flush_to_zero {
        flags |= FLAG_OUTPUT_DENORMAL_FLUSHED;
        p.cls = FloatClass::Zero;
        exp = 0;
        frac_clear(&mut p.frac);
    } else {
        // ARM always detects tininess before rounding (see
        // arm_set_default_fp_behaviours), so is_tiny is unconditionally
        // true here and QEMU's "discard" re-rounding probe never runs.
        let is_tiny = true;

        frac_shrjam(&mut p.frac, 1 - exp);

        if p.frac & round_mask != 0 {
            // Recompute round-to-even after the sticky shift.
            if status.rounding_mode == RoundMode::NearestEven {
                inc = if p.frac & roundeven_mask != frac_lsbm1 {
                    frac_lsbm1
                } else {
                    0
                };
            }
            flags |= FLAG_INEXACT;
            frac_addi(&mut p.frac, inc);
            p.frac &= !round_mask;
        }

        exp = i32::from(p.frac & DECOMPOSED_IMPLICIT_BIT != 0);
        frac_shr(&mut p.frac, frac_shift);

        if is_tiny {
            if status.flush_to_zero {
                flags |= FLAG_OUTPUT_DENORMAL_FLUSHED;
                p.cls = FloatClass::Zero;
                exp = 0;
                frac_clear(&mut p.frac);
            } else if flags & FLAG_INEXACT != 0 {
                flags |= FLAG_UNDERFLOW;
            }
            if exp == 0 && frac_eqz(p.frac) {
                p.cls = FloatClass::Zero;
            }
        }
    }

    p.exp = exp;
    status.raise(flags);
}

/// QEMU's `parts64_uncanon`.
fn parts_uncanon(p: &mut Parts, status: &mut FloatStatus) {
    if is_anynorm(p.cls) {
        parts_uncanon_normal(p, status);
    } else {
        match p.cls {
            FloatClass::Zero => {
                p.exp = 0;
                frac_clear(&mut p.frac);
            }
            FloatClass::Inf => {
                p.exp = status.width.exp_max();
                frac_clear(&mut p.frac);
            }
            FloatClass::QNaN | FloatClass::SNaN => {
                p.exp = status.width.exp_max();
                frac_shr(&mut p.frac, status.width.frac_shift());
            }
            _ => unreachable!("unknown float class in uncanon"),
        }
    }
}

// ---------------------------------------------------------------------------
// NaN helpers (QEMU's `parts64_default_nan` etc.).
// ---------------------------------------------------------------------------

/// QEMU's `parts64_default_nan`. For ARM the default NaN pattern is
/// `0b01000000`, placed into bits [62:56] (bit 62 set); uncanon shifts it
/// into the raw fraction position later.
fn parts_default_nan(p: &mut Parts, status: &FloatStatus) {
    p.cls = FloatClass::QNaN;
    p.sign = false;
    p.exp = status.width.exp_max();
    p.frac = 1u64 << (DECOMPOSED_BINARY_POINT - 1);
}

/// QEMU's `parts64_silence_nan`. For ARM (`snan_bit_is_one` false) the
/// quiet bit is bit 62, just below the implicit bit.
fn parts_silence_nan(p: &mut Parts) {
    p.frac |= 1 << (DECOMPOSED_BINARY_POINT - 1);
    p.cls = FloatClass::QNaN;
}

/// QEMU's `parts64_return_nan`.
fn parts_return_nan(a: &mut Parts, status: &mut FloatStatus) {
    match a.cls {
        FloatClass::SNaN => {
            status.raise(FLAG_INVALID | FLAG_INVALID_SNAN);
            if status.default_nan_mode {
                parts_default_nan(a, status);
            } else {
                parts_silence_nan(a);
            }
        }
        FloatClass::QNaN => {
            if status.default_nan_mode {
                parts_default_nan(a, status);
            }
        }
        _ => unreachable!("return_nan called on non-NaN"),
    }
}

/// QEMU's `parts64_pick_nan` for the ARM `s_ab` rule.
fn parts_pick_nan<'a>(
    a: &'a mut Parts,
    b: &'a mut Parts,
    status: &mut FloatStatus,
) -> &'a mut Parts {
    let have_snan = is_snan(a.cls) || is_snan(b.cls);
    if have_snan {
        status.raise(FLAG_INVALID | FLAG_INVALID_SNAN);
    }

    if status.default_nan_mode {
        parts_default_nan(a, status);
        return a;
    }

    // float_2nan_prop_s_ab: prefer the SNaN, then A over B.
    let ret = if have_snan {
        if is_snan(a.cls) {
            a
        } else {
            b
        }
    } else if is_nan(a.cls) {
        a
    } else {
        b
    };

    if is_snan(ret.cls) {
        parts_silence_nan(ret);
    }
    ret
}

/// Return the class of the selected NaN operand (0, 1, 2 -> a, b, c).
fn sel_cls(sel: u32, a: &Parts, b: &Parts, c: &Parts) -> FloatClass {
    match sel {
        0 => a.cls,
        1 => b.cls,
        _ => c.cls,
    }
}

/// QEMU's `parts64_pick_nan_muladd` with the ARM `s_cab` rule and
/// `dnan_if_qnan` for `inf*0 + qnan`.
fn parts_pick_nan_muladd<'p>(
    a: &'p mut Parts,
    b: &'p mut Parts,
    c: &'p mut Parts,
    status: &mut FloatStatus,
    ab_mask: u32,
    abc_mask: u32,
) -> &'p mut Parts {
    let infzero = ab_mask == CMASK_INFZERO;
    let have_snan = abc_mask & CMASK_SNAN != 0;

    if have_snan {
        status.raise(FLAG_INVALID | FLAG_INVALID_SNAN);
    }

    if infzero {
        // ARM does not suppress invalid for 0*inf + NaN.
        status.raise(FLAG_INVALID | FLAG_INVALID_IMZ);
    }

    if status.default_nan_mode {
        parts_default_nan(a, status);
        return a;
    }

    if infzero {
        // float_infzeronan_dnan_if_qnan: return the default NaN if c is QNaN,
        // otherwise return the (signaling, silenced) input NaN.
        if is_qnan(c.cls) {
            parts_default_nan(a, status);
            return a;
        }
        let ret = c;
        if is_snan(ret.cls) {
            parts_silence_nan(ret);
        }
        return ret;
    }

    // float_3nan_prop_s_cab = PROPRULE(2, 0, 1) | R_3NAN_SNAN_MASK.
    // Selection order: c (2), then a (0), then b (1).
    let mut rule = (2) | (1 << 4) | (1 << 6);
    let sel: u32 = if have_snan && (rule & (1 << 6) != 0) {
        loop {
            let chosen = rule & 3;
            rule >>= 2;
            if is_snan(sel_cls(chosen, a, b, c)) {
                break chosen;
            }
        }
    } else {
        loop {
            let chosen = rule & 3;
            rule >>= 2;
            if is_nan(sel_cls(chosen, a, b, c)) {
                break chosen;
            }
        }
    };
    let ret = match sel {
        0 => a,
        1 => b,
        _ => c,
    };

    if is_snan(ret.cls) {
        parts_silence_nan(ret);
    }
    ret
}

// ---------------------------------------------------------------------------
// Add / subtract (QEMU `parts64_add_normal` etc.).
// ---------------------------------------------------------------------------

/// QEMU's `parts64_add_normal`.
fn parts_add_normal(a: &mut Parts, b: &mut Parts) {
    let exp_diff = a.exp - b.exp;
    if exp_diff > 0 {
        frac_shrjam(&mut b.frac, exp_diff);
    } else if exp_diff < 0 {
        frac_shrjam(&mut a.frac, -exp_diff);
        a.exp = b.exp;
    }

    if frac_add(&mut a.frac, b.frac) {
        frac_shrjam(&mut a.frac, 1);
        a.frac |= DECOMPOSED_IMPLICIT_BIT;
        a.exp += 1;
    }
}

/// QEMU's `parts64_sub_normal`; returns false if the subtraction was exact
/// (result zero).
fn parts_sub_normal(a: &mut Parts, b: &mut Parts) -> bool {
    let exp_diff = a.exp - b.exp;

    if exp_diff > 0 {
        frac_shrjam(&mut b.frac, exp_diff);
        frac_sub(&mut a.frac, b.frac);
    } else if exp_diff < 0 {
        a.exp = b.exp;
        a.sign ^= true;
        frac_shrjam(&mut a.frac, -exp_diff);
        // frac_sub(a, b, a) computes a = b - a.
        a.frac = b.frac.wrapping_sub(a.frac);
    } else if frac_sub(&mut a.frac, b.frac) {
        // Overflow means A was less than B.
        frac_neg(&mut a.frac);
        a.sign ^= true;
    }

    let shift = frac_normalize(&mut a.frac);
    if shift < DECOMPOSED_BINARY_POINT {
        a.exp -= shift;
        true
    } else {
        a.cls = FloatClass::Zero;
        a.sign = false;
        false
    }
}

/// QEMU's `parts64_addsub` for float32.
fn parts_addsub<'p>(
    a: &'p mut Parts,
    b: &'p mut Parts,
    status: &mut FloatStatus,
    subtract: bool,
) -> &'p mut Parts {
    let b_sign = b.sign ^ subtract;
    let ab_mask = cmask(a.cls) | cmask(b.cls);

    // For addition and subtraction, we consume an input denormal unless
    // the other input is a NaN.
    if ab_mask & (CMASK_DENORMAL | CMASK_ANYNAN) == CMASK_DENORMAL {
        status.raise(FLAG_INPUT_DENORMAL_USED);
    }

    if a.sign != b_sign {
        // Subtraction.
        if cmask_only_normals(ab_mask) {
            if parts_sub_normal(a, b) {
                return a;
            }
            // Subtract was exact, fall through to set sign.
            a.cls = FloatClass::Zero;
            a.sign = status.rounding_mode == RoundMode::Down;
            return a;
        }

        if ab_mask == CMASK_ZERO {
            a.sign = status.rounding_mode == RoundMode::Down;
            return a;
        }

        if ab_mask & CMASK_ANYNAN != 0 {
            return parts_pick_nan(a, b, status);
        }

        if ab_mask & CMASK_INF != 0 {
            if a.cls != FloatClass::Inf {
                // N - Inf
                b.sign = b_sign;
                return b;
            }
            if b.cls != FloatClass::Inf {
                // Inf - N
                return a;
            }
            // Inf - Inf
            status.raise(FLAG_INVALID | FLAG_INVALID_ISI);
            parts_default_nan(a, status);
            return a;
        }
    } else {
        // Addition.
        if cmask_only_normals(ab_mask) {
            parts_add_normal(a, b);
            return a;
        }

        if ab_mask == CMASK_ZERO {
            return a;
        }

        if ab_mask & CMASK_ANYNAN != 0 {
            return parts_pick_nan(a, b, status);
        }

        if ab_mask & CMASK_INF != 0 {
            a.cls = FloatClass::Inf;
            return a;
        }
    }

    if b.cls == FloatClass::Zero {
        return a;
    }

    // a is zero, b is a normal.
    b.sign = b_sign;
    b
}

// ---------------------------------------------------------------------------
// Multiply (QEMU `parts64_mul`).
// ---------------------------------------------------------------------------

/// QEMU's `parts64_mul` for float32.
fn parts_mul<'p>(a: &'p mut Parts, b: &'p mut Parts, status: &mut FloatStatus) -> &'p mut Parts {
    let ab_mask = cmask(a.cls) | cmask(b.cls);
    let sign = a.sign != b.sign;

    if cmask_only_normals(ab_mask) {
        if ab_mask & CMASK_DENORMAL != 0 {
            status.raise(FLAG_INPUT_DENORMAL_USED);
        }

        // 128-bit product; sticky from the low word.
        let product = (a.frac as u128) * (b.frac as u128);
        a.frac = (product >> 64) as u64 | u64::from((product as u64) != 0);

        a.exp += b.exp + 1;
        if a.frac & DECOMPOSED_IMPLICIT_BIT == 0 {
            let doubled = a.frac.wrapping_add(a.frac);
            a.frac = doubled;
            a.exp -= 1;
        }

        a.sign = sign;
        return a;
    }

    // Inf * Zero == NaN
    if ab_mask == CMASK_INFZERO {
        status.raise(FLAG_INVALID | FLAG_INVALID_IMZ);
        parts_default_nan(a, status);
        return a;
    }

    if ab_mask & CMASK_ANYNAN != 0 {
        return parts_pick_nan(a, b, status);
    }

    // Multiply by 0 or Inf.
    if ab_mask & CMASK_DENORMAL != 0 {
        status.raise(FLAG_INPUT_DENORMAL_USED);
    }

    if ab_mask & CMASK_INF != 0 {
        a.cls = FloatClass::Inf;
        a.sign = sign;
        return a;
    }

    a.cls = FloatClass::Zero;
    a.sign = sign;
    a
}

// ---------------------------------------------------------------------------
// Fused multiply-add (QEMU `parts64_muladd_scalbn`).
// ---------------------------------------------------------------------------

/// QEMU's `float_muladd_negate_c`, for VFMS/VFNMS.
pub const MULADD_NEGATE_C: u32 = 1;
/// QEMU's `float_muladd_negate_product`, for VFNMA/VFNMS.
pub const MULADD_NEGATE_PRODUCT: u32 = 2;

/// Wide (128-bit) parts used for the product of the fused multiply-add.
#[derive(Clone, Copy, Debug)]
struct WideParts {
    sign: bool,
    exp: i32,
    frac: u128,
}

fn wide_shrjam(frac: &mut u128, c: i32) {
    if c != 0 {
        if c < 128 {
            *frac = (*frac >> c) | u128::from(*frac << (128 - c) != 0);
        } else {
            *frac = u128::from(*frac != 0);
        }
    }
}

/// QEMU's `parts64_add_normal` on wide (128-bit) parts.
fn wide_add_normal(a: &mut WideParts, b: &mut WideParts) {
    let exp_diff = a.exp - b.exp;
    if exp_diff > 0 {
        wide_shrjam(&mut b.frac, exp_diff);
    } else if exp_diff < 0 {
        wide_shrjam(&mut a.frac, -exp_diff);
        a.exp = b.exp;
    }

    let (r, carry) = a.frac.overflowing_add(b.frac);
    a.frac = r;
    if carry {
        wide_shrjam(&mut a.frac, 1);
        a.frac |= 1u128 << 127;
        a.exp += 1;
    }
}

/// QEMU's `parts64_sub_normal` on wide parts; false if the result is exact
/// zero.
fn wide_sub_normal(a: &mut WideParts, b: &mut WideParts) -> bool {
    let exp_diff = a.exp - b.exp;

    if exp_diff > 0 {
        wide_shrjam(&mut b.frac, exp_diff);
        a.frac = a.frac.wrapping_sub(b.frac);
    } else if exp_diff < 0 {
        a.exp = b.exp;
        a.sign ^= true;
        wide_shrjam(&mut a.frac, -exp_diff);
        // frac_sub(a, b, a) computes a = b - a.
        a.frac = b.frac.wrapping_sub(a.frac);
    } else {
        let (r, borrow) = a.frac.overflowing_sub(b.frac);
        a.frac = r;
        if borrow {
            // Overflow means A was less than B.
            a.frac = a.frac.wrapping_neg();
            a.sign ^= true;
        }
    }

    if a.frac == 0 {
        a.sign = false;
        return false;
    }
    let shift = a.frac.leading_zeros() as i32;
    a.frac <<= shift;
    a.exp -= shift;
    true
}

/// QEMU's `parts64_muladd_scalbn` for float32.
fn parts_muladd_scalbn<'p>(
    a: &'p mut Parts,
    b: &'p mut Parts,
    c: &'p mut Parts,
    scale: i32,
    flags: u32,
    status: &mut FloatStatus,
) -> &'p mut Parts {
    let ab_mask = cmask(a.cls) | cmask(b.cls);
    let abc_mask = cmask(c.cls) | ab_mask;

    if abc_mask & CMASK_ANYNAN != 0 {
        return parts_pick_nan_muladd(a, b, c, status, ab_mask, abc_mask);
    }

    if flags & MULADD_NEGATE_C != 0 {
        c.sign ^= true;
    }

    // Compute the sign of the product into A.
    a.sign ^= b.sign;
    if flags & MULADD_NEGATE_PRODUCT != 0 {
        a.sign ^= true;
    }

    if !cmask_only_normals(ab_mask) {
        if ab_mask == CMASK_INFZERO {
            status.raise(FLAG_INVALID | FLAG_INVALID_IMZ);
            parts_default_nan(a, status);
            return a;
        }

        if ab_mask & CMASK_INF != 0 {
            if c.cls == FloatClass::Inf && a.sign != c.sign {
                status.raise(FLAG_INVALID | FLAG_INVALID_ISI);
                parts_default_nan(a, status);
                return a;
            }
            a.cls = FloatClass::Inf;
            if abc_mask & CMASK_DENORMAL != 0 {
                status.raise(FLAG_INPUT_DENORMAL_USED);
            }
            return a;
        }

        // ab_mask has only zero.
        if is_anynorm(c.cls) {
            *a = *c;
            a.exp += scale;
            if abc_mask & CMASK_DENORMAL != 0 {
                status.raise(FLAG_INPUT_DENORMAL_USED);
            }
            return a;
        }
        if c.cls == FloatClass::Zero {
            if a.sign != c.sign {
                a.sign = status.rounding_mode == RoundMode::Down;
            }
            a.cls = FloatClass::Zero;
            if abc_mask & CMASK_DENORMAL != 0 {
                status.raise(FLAG_INPUT_DENORMAL_USED);
            }
            return a;
        }
        // c is Inf.
    }

    if c.cls == FloatClass::Inf {
        a.sign = c.sign;
        a.cls = FloatClass::Inf;
        if abc_mask & CMASK_DENORMAL != 0 {
            status.raise(FLAG_INPUT_DENORMAL_USED);
        }
        return a;
    }

    // Perform the multiplication step.
    let mut p_widen = WideParts {
        sign: a.sign,
        exp: a.exp + b.exp + 1,
        frac: (a.frac as u128) * (b.frac as u128),
    };
    if p_widen.frac >> 64 & DECOMPOSED_IMPLICIT_BIT as u128 == 0 {
        p_widen.frac = p_widen.frac.wrapping_add(p_widen.frac);
        p_widen.exp -= 1;
    }

    // Perform the addition step.
    if c.cls != FloatClass::Zero {
        let mut c_widen = WideParts {
            sign: c.sign,
            exp: c.exp,
            frac: (c.frac as u128) << 64,
        };

        if a.sign == c.sign {
            wide_add_normal(&mut p_widen, &mut c_widen);
        } else if !wide_sub_normal(&mut p_widen, &mut c_widen) {
            a.sign = status.rounding_mode == RoundMode::Down;
            a.cls = FloatClass::Zero;
            if abc_mask & CMASK_DENORMAL != 0 {
                status.raise(FLAG_INPUT_DENORMAL_USED);
            }
            return a;
        }
    }

    // Narrow with sticky bit, for proper rounding later.
    a.frac = (p_widen.frac >> 64) as u64 | u64::from((p_widen.frac as u64) != 0);
    a.sign = p_widen.sign;
    a.exp = p_widen.exp;

    a.exp += scale;
    if abc_mask & CMASK_DENORMAL != 0 {
        status.raise(FLAG_INPUT_DENORMAL_USED);
    }
    a
}

// ---------------------------------------------------------------------------
// Divide (QEMU `parts64_div`).
// ---------------------------------------------------------------------------

/// QEMU's `parts64_div` for float32.
fn parts_div<'p>(a: &'p mut Parts, b: &'p mut Parts, status: &mut FloatStatus) -> &'p mut Parts {
    let ab_mask = cmask(a.cls) | cmask(b.cls);
    let sign = a.sign != b.sign;

    if cmask_only_normals(ab_mask) {
        if ab_mask & CMASK_DENORMAL != 0 {
            status.raise(FLAG_INPUT_DENORMAL_USED);
        }

        // QEMU's parts_div: frac_div returns (quotient, ret) where `ret`
        // means the input was < the divisor, adjusting the exponent by 1.
        let (q, ret) = frac_div(a.frac, b.frac);
        a.frac = q;
        a.exp -= b.exp + i32::from(ret);
        a.sign = sign;
        return a;
    }

    // 0/0 or Inf/Inf => NaN
    if ab_mask == CMASK_ZERO {
        status.raise(FLAG_INVALID | FLAG_INVALID_ZDZ);
        parts_default_nan(a, status);
        return a;
    }
    if ab_mask == CMASK_INF {
        status.raise(FLAG_INVALID | FLAG_INVALID_IDI);
        parts_default_nan(a, status);
        return a;
    }

    // All the NaN cases.
    if ab_mask & CMASK_ANYNAN != 0 {
        return parts_pick_nan(a, b, status);
    }

    if ab_mask & CMASK_DENORMAL != 0 && b.cls != FloatClass::Zero {
        status.raise(FLAG_INPUT_DENORMAL_USED);
    }

    a.sign = sign;

    // Inf / X
    if a.cls == FloatClass::Inf {
        return a;
    }

    // 0 / X
    if a.cls == FloatClass::Zero {
        return a;
    }

    // X / Inf
    if b.cls == FloatClass::Inf {
        a.cls = FloatClass::Zero;
        return a;
    }

    // X / 0 => Inf
    status.raise(FLAG_DIVBYZERO);
    a.cls = FloatClass::Inf;
    a
}

/// QEMU's `frac64_div`: 2N/N division of normalized fractions. Returns
/// `(quotient, ret)` where the quotient is normalized to bit 63 with the
/// sticky (remainder) bit in bit 0, and `ret` is the exponent adjustment.
fn frac_div(n: u64, d: u64) -> (u64, bool) {
    // If n < d, shift n left a full 64 bits and return true to decrement
    // the exponent; otherwise shift by 63, giving an N-bit quotient.
    let ret = n < d;
    let num: u128 = if ret {
        (n as u128) << 64
    } else {
        ((n >> 1) as u128) << 64 | (n << 63) as u128
    };
    let q = num / d as u128;
    let r = num % d as u128;
    (q as u64 | u64::from(r != 0), ret)
}

// ---------------------------------------------------------------------------
// Square root (QEMU `parts64_sqrt`, musl-derived algorithm).
// ---------------------------------------------------------------------------

/// QEMU's `rsqrt_tab`, a 7-bit table for the initial `1/sqrt` estimate.
const RSQRT_TAB: [u16; 128] = [
    0xb451, 0xb2f0, 0xb196, 0xb044, 0xaef9, 0xadb6, 0xac79, 0xab43, 0xaa14, 0xa8eb, 0xa7c8, 0xa6aa,
    0xa592, 0xa480, 0xa373, 0xa26b, 0xa168, 0xa06a, 0x9f70, 0x9e7b, 0x9d8a, 0x9c9d, 0x9bb5, 0x9ad1,
    0x99f0, 0x9913, 0x983a, 0x9765, 0x9693, 0x95c4, 0x94f8, 0x9430, 0x936b, 0x92a9, 0x91ea, 0x912e,
    0x9075, 0x8fbe, 0x8f0a, 0x8e59, 0x8daa, 0x8cfe, 0x8c54, 0x8bac, 0x8b07, 0x8a64, 0x89c4, 0x8925,
    0x8889, 0x87ee, 0x8756, 0x86c0, 0x862b, 0x8599, 0x8508, 0x8479, 0x83ec, 0x8361, 0x82d8, 0x8250,
    0x81c9, 0x8145, 0x80c2, 0x8040, 0xff02, 0xfd0e, 0xfb25, 0xf947, 0xf773, 0xf5aa, 0xf3ea, 0xf234,
    0xf087, 0xeee3, 0xed47, 0xebb3, 0xea27, 0xe8a3, 0xe727, 0xe5b2, 0xe443, 0xe2dc, 0xe17a, 0xe020,
    0xdecb, 0xdd7d, 0xdc34, 0xdaf1, 0xd9b3, 0xd87b, 0xd748, 0xd61a, 0xd4f1, 0xd3cd, 0xd2ad, 0xd192,
    0xd07b, 0xcf69, 0xce5b, 0xcd51, 0xcc4a, 0xcb48, 0xca4a, 0xc94f, 0xc858, 0xc764, 0xc674, 0xc587,
    0xc49d, 0xc3b7, 0xc2d4, 0xc1f4, 0xc116, 0xc03c, 0xbf65, 0xbe90, 0xbdbe, 0xbcef, 0xbc23, 0xbb59,
    0xba91, 0xb9cc, 0xb90a, 0xb84a, 0xb78c, 0xb6d0, 0xb617, 0xb560,
];

const THREE32: u32 = 3u32 << 30;
const THREE64: u64 = 3u64 << 62;

/// QEMU's `parts64_sqrt` for float32.
fn parts_sqrt(a: &mut Parts, status: &mut FloatStatus) {
    if a.cls != FloatClass::Normal {
        match a.cls {
            FloatClass::Denormal => {
                if !a.sign {
                    // Positive denormal is a valid input; sqrt of the
                    // smallest denormal is a small normal.
                    status.raise(FLAG_INPUT_DENORMAL_USED);
                }
            }
            FloatClass::SNaN | FloatClass::QNaN => {
                parts_return_nan(a, status);
                return;
            }
            FloatClass::Zero => return,
            FloatClass::Inf => {
                if a.sign {
                    status.raise(FLAG_INVALID | FLAG_INVALID_SQRT);
                    parts_default_nan(a, status);
                }
                return;
            }
            _ => unreachable!("unknown float class in sqrt"),
        }
    }

    if a.sign {
        status.raise(FLAG_INVALID | FLAG_INVALID_SQRT);
        parts_default_nan(a, status);
        return;
    }

    // Argument reduction: x = 4^e * frac, frac in [1, 4).
    let exp_odd = a.exp & 1 != 0;
    let index = ((a.frac >> 57) & 0x3f) as usize | usize::from(!exp_odd) << 6;
    if !exp_odd {
        frac_shr(&mut a.frac, 1);
    }

    let m64_full = a.frac;
    let m32 = (m64_full >> 32) as u32;

    let mut r32 = (RSQRT_TAB[index] as u32) << 16;
    let mut s32 = (((m32 as u64) * r32 as u64) >> 32) as u32;
    let mut d32 = (((s32 as u64) * r32 as u64) >> 32) as u32;
    let mut u32_ = THREE32.wrapping_sub(d32);

    // N == 64 path (float64 or smaller).
    r32 = (((r32 as u64) * u32_ as u64) >> 31) as u32;
    s32 = (((m32 as u64) * r32 as u64) >> 32) as u32;
    d32 = (((s32 as u64) * r32 as u64) >> 32) as u32;
    u32_ = THREE32.wrapping_sub(d32);

    if status.width == Width::Single {
        // float32 or smaller.
        s32 = (((s32 as u64) * u32_ as u64) >> 32) as u32; // 3.29
        s32 = (s32.wrapping_sub(1)) >> 6; // 9.23

        let d0 = (m32 << 16).wrapping_sub(s32.wrapping_mul(s32));
        let d1 = s32.wrapping_sub(d0);
        let d2 = d1.wrapping_add(s32).wrapping_add(1);
        s32 = s32.wrapping_add(d1 >> 31);
        a.frac = (s32 as u64) << (64 - 25);

        if d2 != 0 {
            let delta = if (d1 as i32 ^ d2 as i32) < 0 { -1 } else { 1 };
            a.frac = (a.frac as i64).wrapping_add(delta) as u64;
        }
    } else {
        // float64: two more 32-bit iterations, then a 64-bit Newton-Raphson
        // step and a final nearest-round correction (QEMU `partsN(sqrt)`).
        r32 = ((u64::from(r32) * u64::from(u32_)) >> 31) as u32;

        s32 = (((m32 as u64) * r32 as u64) >> 32) as u32;
        d32 = (((s32 as u64) * r32 as u64) >> 32) as u32;
        u32_ = THREE32.wrapping_sub(d32);

        // |r*sqrt(m) - 1| < 0x1.37p-29; convert to 64-bit arithmetic.
        let r64 = u64::from(r32).wrapping_mul(u64::from(u32_)).wrapping_mul(2);
        let m64 = m64_full;
        // mul64To128 keeps the high word of the 128-bit product.
        let hi128 = |x: u64, y: u64| -> u64 { ((u128::from(x) * u128::from(y)) >> 64) as u64 };
        let s64 = hi128(m64, r64);
        let d64 = hi128(s64, r64);
        let u64_ = THREE64.wrapping_sub(d64);

        // 3.61 fixed point.
        let s64 = hi128(s64, u64_);
        // 12.52 fixed point.
        let s64 = (s64.wrapping_sub(2)) >> 9;

        // Compute nearest rounded result.
        let d0 = (m64 << 42).wrapping_sub(s64.wrapping_mul(s64));
        let d1 = s64.wrapping_sub(d0);
        let d2 = d1.wrapping_add(s64).wrapping_add(1);
        let s64 = s64.wrapping_add(d1 >> 63);
        a.frac = s64 << (64 - 54);

        // Increment or decrement for inexact.
        if d2 != 0 {
            let delta = if ((d1 ^ d2) as i64) < 0 { -1i64 } else { 1i64 };
            a.frac = (a.frac as i64).wrapping_add(delta) as u64;
        }
    }

    // Convert back from base 4 to base 2.
    a.exp >>= 1;
    if a.frac & DECOMPOSED_IMPLICIT_BIT == 0 {
        let doubled = a.frac.wrapping_add(a.frac);
        a.frac = doubled;
    } else {
        a.exp += 1;
    }
}

// ---------------------------------------------------------------------------
// Round to integral (QEMU `parts64_round_to_int`).
// ---------------------------------------------------------------------------

/// QEMU's `parts64_round_to_int_normal`. Returns true if the rounding was
/// inexact. `frac_size` is the fraction size used for the "all integral"
/// test.
fn parts_round_to_int_normal(a: &mut Parts, rmode: RoundMode, scale: i32, frac_size: i32) -> bool {
    a.exp += scale.clamp(-0x10000, 0x10000);

    if a.exp < 0 {
        // All fractional.
        let one = match rmode {
            RoundMode::NearestEven => {
                if a.exp == -1 {
                    // Shift left one, discarding the implicit bit.
                    let tmp = a.frac.wrapping_add(a.frac);
                    tmp != 0
                } else {
                    false
                }
            }
            RoundMode::ToZero => false,
            RoundMode::Up => !a.sign,
            RoundMode::Down => a.sign,
        };

        frac_clear(&mut a.frac);
        a.exp = 0;
        if one {
            a.frac = DECOMPOSED_IMPLICIT_BIT;
        } else {
            a.cls = FloatClass::Zero;
        }
        return true;
    }

    if a.exp >= frac_size {
        // All integral.
        return false;
    }

    let shift_adj = 0;
    let frac_lsb = DECOMPOSED_IMPLICIT_BIT >> (a.exp & 63);
    let frac_lsbm1 = frac_lsb >> 1;
    let rnd_mask = frac_lsb - 1;
    let rnd_even_mask = rnd_mask | frac_lsb;

    if a.frac & rnd_mask == 0 {
        // Fractional bits already clear.
        return false;
    }

    let inc = match rmode {
        RoundMode::NearestEven => {
            if a.frac & rnd_even_mask != frac_lsbm1 {
                frac_lsbm1
            } else {
                0
            }
        }
        RoundMode::ToZero => 0,
        RoundMode::Up => {
            if a.sign {
                0
            } else {
                rnd_mask
            }
        }
        RoundMode::Down => {
            if a.sign {
                rnd_mask
            } else {
                0
            }
        }
    };

    if shift_adj == 0 {
        if frac_addi(&mut a.frac, inc) {
            frac_shr(&mut a.frac, 1);
            a.frac |= DECOMPOSED_IMPLICIT_BIT;
            a.exp += 1;
        }
        a.frac &= !rnd_mask;
    }
    true
}

/// QEMU's `parts64_round_to_int` for float32.
fn parts_round_to_int(a: &mut Parts, rmode: RoundMode, scale: i32, status: &mut FloatStatus) {
    match a.cls {
        FloatClass::QNaN | FloatClass::SNaN => parts_return_nan(a, status),
        FloatClass::Zero | FloatClass::Inf => {}
        FloatClass::Normal | FloatClass::Denormal => {
            if parts_round_to_int_normal(a, rmode, scale, status.width.frac_size()) {
                status.raise(FLAG_INEXACT);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Float <-> int conversions (QEMU `parts64_float_to_sint` etc.).
// ---------------------------------------------------------------------------

/// QEMU's `parts64_float_to_sint`.
fn parts_float_to_sint(
    p: &mut Parts,
    rmode: RoundMode,
    scale: i32,
    min: i64,
    max: i64,
    status: &mut FloatStatus,
) -> i64 {
    let mut flags = 0u32;
    let mut r: u64;

    match p.cls {
        FloatClass::SNaN => {
            flags |= FLAG_INVALID_SNAN;
            flags |= FLAG_INVALID;
            r = max as u64;
        }
        FloatClass::QNaN => {
            flags |= FLAG_INVALID;
            r = max as u64;
        }
        FloatClass::Inf => {
            flags = FLAG_INVALID | FLAG_INVALID_CVTI;
            r = if p.sign { min as u64 } else { max as u64 };
        }
        FloatClass::Zero => return 0,
        FloatClass::Normal | FloatClass::Denormal => {
            // N - 2 = 62 is the frac_size used by the FPU conversion path.
            if parts_round_to_int_normal(p, rmode, scale, DECOMPOSED_BINARY_POINT - 1) {
                flags = FLAG_INEXACT;
            }

            if p.exp <= DECOMPOSED_BINARY_POINT {
                r = p.frac >> (DECOMPOSED_BINARY_POINT - p.exp);
            } else {
                r = u64::MAX;
            }
            if p.sign {
                if r <= min.unsigned_abs() {
                    r = (r as i64).wrapping_neg() as u64;
                } else {
                    flags = FLAG_INVALID | FLAG_INVALID_CVTI;
                    r = min as u64;
                }
            } else if r > max as u64 {
                flags = FLAG_INVALID | FLAG_INVALID_CVTI;
                r = max as u64;
            }
        }
    }

    status.raise(flags);
    r as i64
}

/// QEMU's `parts64_float_to_uint`.
fn parts_float_to_uint(
    p: &mut Parts,
    rmode: RoundMode,
    scale: i32,
    max: u64,
    status: &mut FloatStatus,
) -> u64 {
    let mut flags = 0u32;
    let mut r: u64;

    match p.cls {
        FloatClass::SNaN => {
            flags |= FLAG_INVALID_SNAN;
            flags |= FLAG_INVALID;
            r = max;
        }
        FloatClass::QNaN => {
            flags |= FLAG_INVALID;
            r = max;
        }
        FloatClass::Inf => {
            flags = FLAG_INVALID | FLAG_INVALID_CVTI;
            r = if p.sign { 0 } else { max };
        }
        FloatClass::Zero => return 0,
        FloatClass::Normal | FloatClass::Denormal => {
            // N - 2 = 62 is the frac_size used by the FPU conversion path.
            if parts_round_to_int_normal(p, rmode, scale, DECOMPOSED_BINARY_POINT - 1) {
                flags = FLAG_INEXACT;
            }

            if p.cls == FloatClass::Zero {
                // Rounded down to zero; no invalid flag is raised.
                r = 0;
            } else if p.sign {
                flags = FLAG_INVALID | FLAG_INVALID_CVTI;
                r = 0;
            } else if p.exp > DECOMPOSED_BINARY_POINT {
                flags = FLAG_INVALID | FLAG_INVALID_CVTI;
                r = max;
            } else {
                r = p.frac >> (DECOMPOSED_BINARY_POINT - p.exp);
                if r > max {
                    flags = FLAG_INVALID | FLAG_INVALID_CVTI;
                    r = max;
                }
            }
        }
    }

    status.raise(flags);
    r
}

/// QEMU's `parts64_sint_to_float`.
fn parts_sint_to_float(p: &mut Parts, a: i64, scale: i32) {
    let mut f = a as u64;
    p.cls = FloatClass::Zero;
    p.sign = false;
    p.exp = 0;
    p.frac = 0;

    if a == 0 {
        return;
    }

    p.cls = FloatClass::Normal;
    if a < 0 {
        f = f.wrapping_neg();
        p.sign = true;
    }
    let shift = f.leading_zeros() as i32;
    let scale = scale.clamp(-0x10000, 0x10000);
    p.exp = DECOMPOSED_BINARY_POINT - shift + scale;
    p.frac = f << shift;
}

/// QEMU's `parts64_uint_to_float`.
fn parts_uint_to_float(p: &mut Parts, a: u64, scale: i32) {
    p.cls = FloatClass::Zero;
    p.sign = false;
    p.exp = 0;
    p.frac = 0;

    if a == 0 {
        return;
    }

    let shift = a.leading_zeros() as i32;
    let scale = scale.clamp(-0x10000, 0x10000);
    p.cls = FloatClass::Normal;
    p.exp = DECOMPOSED_BINARY_POINT - shift + scale;
    p.frac = a << shift;
}

// ---------------------------------------------------------------------------
// Compare (QEMU `parts64_compare`).
// ---------------------------------------------------------------------------

/// QEMU's `FloatRelation`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FloatRelation {
    Less = -1,
    Equal = 0,
    Greater = 1,
    Unordered = 2,
}

/// QEMU's `parts64_compare` for float32.
fn parts_compare(
    a: &mut Parts,
    b: &mut Parts,
    status: &mut FloatStatus,
    is_quiet: bool,
) -> FloatRelation {
    let ab_mask = cmask(a.cls) | cmask(b.cls);

    if cmask_only_normals(ab_mask) {
        if ab_mask & CMASK_DENORMAL != 0 {
            status.raise(FLAG_INPUT_DENORMAL_USED);
        }

        let cmp = if a.sign != b.sign {
            // goto a_sign
            return if a.sign {
                FloatRelation::Less
            } else {
                FloatRelation::Greater
            };
        } else if a.exp == b.exp {
            if a.frac == b.frac {
                FloatRelation::Equal
            } else if a.frac < b.frac {
                FloatRelation::Less
            } else {
                FloatRelation::Greater
            }
        } else if a.exp < b.exp {
            FloatRelation::Less
        } else {
            FloatRelation::Greater
        };

        return if a.sign {
            match cmp {
                FloatRelation::Less => FloatRelation::Greater,
                FloatRelation::Greater => FloatRelation::Less,
                other => other,
            }
        } else {
            cmp
        };
    }

    if ab_mask & CMASK_ANYNAN != 0 {
        if ab_mask & CMASK_SNAN != 0 {
            status.raise(FLAG_INVALID | FLAG_INVALID_SNAN);
        } else if !is_quiet {
            status.raise(FLAG_INVALID);
        }
        return FloatRelation::Unordered;
    }

    if ab_mask & CMASK_DENORMAL != 0 {
        status.raise(FLAG_INPUT_DENORMAL_USED);
    }

    if ab_mask & CMASK_ZERO != 0 {
        if ab_mask == CMASK_ZERO {
            return FloatRelation::Equal;
        } else if a.cls == FloatClass::Zero {
            return if b.sign {
                FloatRelation::Greater
            } else {
                FloatRelation::Less
            };
        } else {
            return if a.sign {
                FloatRelation::Less
            } else {
                FloatRelation::Greater
            };
        }
    }

    if ab_mask == CMASK_INF {
        if a.sign == b.sign {
            return FloatRelation::Equal;
        }
    } else if b.cls == FloatClass::Inf {
        return if b.sign {
            FloatRelation::Greater
        } else {
            FloatRelation::Less
        };
    } else {
        // a is Inf.
        return if a.sign {
            FloatRelation::Less
        } else {
            FloatRelation::Greater
        };
    }

    // Both infinities, different signs.
    if a.sign {
        FloatRelation::Less
    } else {
        FloatRelation::Greater
    }
}

// ---------------------------------------------------------------------------
// Pack / unpack (QEMU `float32_pack_raw` etc.).
// ---------------------------------------------------------------------------

/// QEMU's `float32_pack_raw`; only valid on parts that have been
/// uncanonicalized (the exponent is already biased and the fraction has
/// already been shifted into the low bits by `uncanon`).
fn pack_raw(p: &Parts) -> u32 {
    let sign = u32::from(p.sign) << 31;
    let exp = (p.exp as u32) << 23;
    let frac = p.frac as u32 & 0x7f_ffff;
    sign | exp | frac
}

/// QEMU's `pack_raw64` for float64.
fn pack_raw_d(p: &Parts) -> u64 {
    let sign = u64::from(p.sign) << 63;
    let exp = ((p.exp as u64) & 0x7ff) << 52;
    let frac = p.frac & 0xf_ffff_ffff_ffff;
    sign | exp | frac
}

/// QEMU's `float32_unpack_raw`.
fn unpack_raw(p: &mut Parts, f: u32) {
    p.sign = f >> 31 & 1 != 0;
    p.exp = (f >> 23 & 0xff) as i32;
    p.frac = (f & 0x7f_ffff) as u64;
    p.cls = FloatClass::Zero; // set by canonicalize
}

/// QEMU's `unpack_raw64` for float64.
fn unpack_raw_d(p: &mut Parts, f: u64) {
    p.sign = f >> 63 & 1 != 0;
    p.exp = (f >> 52 & 0x7ff) as i32;
    p.frac = f & 0xf_ffff_ffff_ffff;
    p.cls = FloatClass::Zero; // set by canonicalize
}

/// QEMU's `float32_round_pack_canonical`.
fn round_pack_canonical(p: &mut Parts, status: &mut FloatStatus) -> u32 {
    status.width = Width::Single;
    parts_uncanon(p, status);
    pack_raw(p)
}

/// QEMU's `float64_round_pack_canonical`.
fn round_pack_canonical_d(p: &mut Parts, status: &mut FloatStatus) -> u64 {
    status.width = Width::Double;
    parts_uncanon(p, status);
    pack_raw_d(p)
}

/// QEMU's `float32_unpack_canonical`.
fn unpack_canonical(p: &mut Parts, f: u32, status: &mut FloatStatus) {
    status.width = Width::Single;
    unpack_raw(p, f);
    parts_canonicalize(p, status);
}

/// QEMU's `float64_unpack_canonical`.
fn unpack_canonical_d(p: &mut Parts, f: u64, status: &mut FloatStatus) {
    status.width = Width::Double;
    unpack_raw_d(p, f);
    parts_canonicalize(p, status);
}

/// True if the raw float32 bits denote a denormal (used by VABS/VNEG/VMOV
/// which do not consume denormals).
pub fn is_denormal(raw: u32) -> bool {
    (raw & 0x7f80_0000) == 0 && (raw & 0x007f_ffff) != 0
}

// ---------------------------------------------------------------------------
// Public float32 operations (QEMU's `float32_*`).
// ---------------------------------------------------------------------------

fn op2(
    a: u32,
    b: u32,
    status: &mut FloatStatus,
    op: for<'p, 's> fn(&'p mut Parts, &'p mut Parts, &'s mut FloatStatus) -> &'p mut Parts,
) -> u32 {
    let mut pa = unpack_canonical_bits(a, status);
    let mut pb = unpack_canonical_bits(b, status);
    let pr = op(&mut pa, &mut pb, status);
    round_pack_canonical(pr, status)
}

fn unpack_canonical_bits(f: u32, status: &mut FloatStatus) -> Parts {
    let mut p = Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    };
    unpack_canonical(&mut p, f, status);
    p
}

/// float32 addition.
pub fn add(a: u32, b: u32, status: &mut FloatStatus) -> u32 {
    op2(a, b, status, |a, b, s| parts_addsub(a, b, s, false))
}

/// float32 subtraction.
pub fn sub(a: u32, b: u32, status: &mut FloatStatus) -> u32 {
    op2(a, b, status, |a, b, s| parts_addsub(a, b, s, true))
}

/// float32 multiply.
pub fn mul(a: u32, b: u32, status: &mut FloatStatus) -> u32 {
    op2(a, b, status, parts_mul)
}

/// float32 divide.
pub fn div(a: u32, b: u32, status: &mut FloatStatus) -> u32 {
    op2(a, b, status, parts_div)
}

/// float32 fused multiply-add: `a * b + c`, with a single rounding.
pub fn muladd(a: u32, b: u32, c: u32, status: &mut FloatStatus) -> u32 {
    muladd_neg(a, b, c, 0, status)
}

/// float32 fused multiply-add with sign-control flags
/// (`MULADD_NEGATE_C` / `MULADD_NEGATE_PRODUCT`), as used by VFMA/VFMS/VFNMA/VFNMS.
pub fn muladd_neg(a: u32, b: u32, c: u32, flags: u32, status: &mut FloatStatus) -> u32 {
    let mut pa = unpack_canonical_bits(a, status);
    let mut pb = unpack_canonical_bits(b, status);
    let mut pc = unpack_canonical_bits(c, status);
    let pr = parts_muladd_scalbn(&mut pa, &mut pb, &mut pc, 0, flags, status);
    round_pack_canonical(pr, status)
}

/// float32 square root.
pub fn sqrt(a: u32, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits(a, status);
    parts_sqrt(&mut p, status);
    round_pack_canonical(&mut p, status)
}

/// float32 round to integral value, using the FPSCR rounding mode.
pub fn round_to_int(a: u32, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits(a, status);
    parts_round_to_int(&mut p, status.rounding_mode, 0, status);
    round_pack_canonical(&mut p, status)
}

/// float32 compare, raising invalid on QNaN inputs (VCMPE).
pub fn compare(a: u32, b: u32, status: &mut FloatStatus) -> FloatRelation {
    let mut pa = unpack_canonical_bits(a, status);
    let mut pb = unpack_canonical_bits(b, status);
    parts_compare(&mut pa, &mut pb, status, false)
}

/// float32 quiet compare, raising invalid only on SNaN inputs (VCMP).
pub fn compare_signal(a: u32, b: u32, status: &mut FloatStatus) -> FloatRelation {
    let mut pa = unpack_canonical_bits(a, status);
    let mut pb = unpack_canonical_bits(b, status);
    parts_compare(&mut pa, &mut pb, status, true)
}

/// float32 -> int32, using the FPSCR rounding mode (VCVT).
pub fn to_int32(a: u32, status: &mut FloatStatus) -> i32 {
    let mut p = unpack_canonical_bits(a, status);
    parts_float_to_sint(
        &mut p,
        status.rounding_mode,
        0,
        i32::MIN as i64,
        i32::MAX as i64,
        status,
    ) as i32
}

/// float32 -> int32 with round-to-zero (VCVTR).
pub fn to_int32_round_to_zero(a: u32, status: &mut FloatStatus) -> i32 {
    let mut p = unpack_canonical_bits(a, status);
    parts_float_to_sint(
        &mut p,
        RoundMode::ToZero,
        0,
        i32::MIN as i64,
        i32::MAX as i64,
        status,
    ) as i32
}

/// float32 -> uint32, using the FPSCR rounding mode.
pub fn to_uint32(a: u32, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits(a, status);
    parts_float_to_uint(&mut p, status.rounding_mode, 0, u32::MAX as u64, status) as u32
}

/// float32 -> uint32 with round-to-zero.
pub fn to_uint32_round_to_zero(a: u32, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits(a, status);
    parts_float_to_uint(&mut p, RoundMode::ToZero, 0, u32::MAX as u64, status) as u32
}

/// float32 -> int16 with a scale, round-to-zero (QEMU
/// `float32_to_int16_scalbn`; used by VCVT to fixed-point).
pub fn to_int16_scalbn(a: u32, shift: i32, status: &mut FloatStatus) -> i16 {
    let mut p = unpack_canonical_bits(a, status);
    parts_float_to_sint(
        &mut p,
        RoundMode::ToZero,
        shift,
        i16::MIN as i64,
        i16::MAX as i64,
        status,
    ) as i16
}

/// float32 -> uint16 with a scale, round-to-zero (QEMU
/// `float32_to_uint16_scalbn`).
pub fn to_uint16_scalbn(a: u32, shift: i32, status: &mut FloatStatus) -> u16 {
    let mut p = unpack_canonical_bits(a, status);
    parts_float_to_uint(&mut p, RoundMode::ToZero, shift, u16::MAX as u64, status) as u16
}

/// int32 -> float32 (QEMU `int32_to_float32`).
pub fn int32_to_float32(a: i32, status: &mut FloatStatus) -> u32 {
    let mut p = Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    };
    parts_sint_to_float(&mut p, a as i64, 0);
    round_pack_canonical(&mut p, status)
}

/// uint32 -> float32 (QEMU `uint32_to_float32`).
pub fn uint32_to_float32(a: u32, status: &mut FloatStatus) -> u32 {
    let mut p = Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    };
    parts_uint_to_float(&mut p, a as u64, 0);
    round_pack_canonical(&mut p, status)
}

/// int16 -> float32 with a scale (QEMU `int16_to_float32_scalbn`; used by
/// VCVT from fixed-point; `shift` is the fixed-point fraction length).
pub fn int16_to_float32_scalbn(a: i16, shift: i32, status: &mut FloatStatus) -> u32 {
    let mut p = Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    };
    parts_sint_to_float(&mut p, a as i64, -shift);
    round_pack_canonical(&mut p, status)
}

/// uint16 -> float32 with a scale.
pub fn uint16_to_float32_scalbn(a: u16, shift: i32, status: &mut FloatStatus) -> u32 {
    let mut p = Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    };
    parts_uint_to_float(&mut p, a as u64, -shift);
    round_pack_canonical(&mut p, status)
}

/// int32 -> float32 with a scale.
pub fn int32_to_float32_scalbn(a: i32, shift: i32, status: &mut FloatStatus) -> u32 {
    let mut p = Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    };
    parts_sint_to_float(&mut p, a as i64, -shift);
    round_pack_canonical(&mut p, status)
}

/// uint32 -> float32 with a scale.
pub fn uint32_to_float32_scalbn(a: u32, shift: i32, status: &mut FloatStatus) -> u32 {
    let mut p = Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    };
    parts_uint_to_float(&mut p, a as u64, -shift);
    round_pack_canonical(&mut p, status)
}

/// float32 -> int32 with a scale, round-to-zero (QEMU
/// `float32_to_int32_scalbn`; used by VCVT to fixed-point).
pub fn to_int32_scalbn(a: u32, shift: i32, status: &mut FloatStatus) -> i32 {
    let mut p = unpack_canonical_bits(a, status);
    parts_float_to_sint(
        &mut p,
        RoundMode::ToZero,
        shift,
        i32::MIN as i64,
        i32::MAX as i64,
        status,
    ) as i32
}

/// float32 -> uint32 with a scale, round-to-zero (QEMU
/// `float32_to_uint32_scalbn`).
pub fn to_uint32_scalbn(a: u32, shift: i32, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits(a, status);
    parts_float_to_uint(&mut p, RoundMode::ToZero, shift, u32::MAX as u64, status) as u32
}

// ---------------------------------------------------------------------------
// Public float64 operations (QEMU's `float64_*`; FPv5 double precision).
// ---------------------------------------------------------------------------

/// True if the raw float64 bits denote a denormal.
pub fn is_denormal_d(raw: u64) -> bool {
    (raw & 0x7ff0_0000_0000_0000) == 0 && (raw & 0x000f_ffff_ffff_ffff) != 0
}

fn unpack_canonical_bits_d(f: u64, status: &mut FloatStatus) -> Parts {
    let mut p = Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    };
    unpack_canonical_d(&mut p, f, status);
    p
}

fn op2_d(
    a: u64,
    b: u64,
    status: &mut FloatStatus,
    op: for<'p, 's> fn(&'p mut Parts, &'p mut Parts, &'s mut FloatStatus) -> &'p mut Parts,
) -> u64 {
    let mut pa = unpack_canonical_bits_d(a, status);
    let mut pb = unpack_canonical_bits_d(b, status);
    let pr = op(&mut pa, &mut pb, status);
    round_pack_canonical_d(pr, status)
}

/// float64 addition.
pub fn add_f64(a: u64, b: u64, status: &mut FloatStatus) -> u64 {
    op2_d(a, b, status, |a, b, s| parts_addsub(a, b, s, false))
}

/// float64 subtraction.
pub fn sub_f64(a: u64, b: u64, status: &mut FloatStatus) -> u64 {
    op2_d(a, b, status, |a, b, s| parts_addsub(a, b, s, true))
}

/// float64 multiply.
pub fn mul_f64(a: u64, b: u64, status: &mut FloatStatus) -> u64 {
    op2_d(a, b, status, parts_mul)
}

/// float64 divide.
pub fn div_f64(a: u64, b: u64, status: &mut FloatStatus) -> u64 {
    op2_d(a, b, status, parts_div)
}

/// float64 fused multiply-add with sign-control flags, as used by
/// VFMA.F64/VFMS.F64/VFNMA.F64/VFNMS.F64.
pub fn muladd_neg_f64(a: u64, b: u64, c: u64, flags: u32, status: &mut FloatStatus) -> u64 {
    let mut pa = unpack_canonical_bits_d(a, status);
    let mut pb = unpack_canonical_bits_d(b, status);
    let mut pc = unpack_canonical_bits_d(c, status);
    let pr = parts_muladd_scalbn(&mut pa, &mut pb, &mut pc, 0, flags, status);
    round_pack_canonical_d(pr, status)
}

/// float64 square root.
pub fn sqrt_f64(a: u64, status: &mut FloatStatus) -> u64 {
    // Classify via softfloat Parts so NaN/denormal handling and flags
    // match the QEMU model exactly; the arithmetic itself uses the
    // host's correctly-rounded sqrt.
    let p = unpack_canonical_bits_d(a, status);
    match p.cls {
        FloatClass::QNaN | FloatClass::SNaN => {
            // Reuse the shared NaN-return logic (raises INVALID|SNAN,
            // honours default_nan_mode, quiets SNaNs).
            let mut q = p;
            // parts_return_nan expects the NaN in `q` and will raise flags.
            // Inline its logic to avoid borrowing issues with `status`.
            match q.cls {
                FloatClass::SNaN => {
                    status.raise(FLAG_INVALID | FLAG_INVALID_SNAN);
                    if status.default_nan_mode {
                        parts_default_nan(&mut q, status);
                    } else {
                        parts_silence_nan(&mut q);
                    }
                }
                FloatClass::QNaN => {
                    if status.default_nan_mode {
                        parts_default_nan(&mut q, status);
                    }
                }
                _ => unreachable!(),
            }
            return round_pack_canonical_d(&mut q, status);
        }
        FloatClass::Zero => return a,
        FloatClass::Inf => {
            if p.sign {
                status.raise(FLAG_INVALID);
                let mut q = p;
                parts_default_nan(&mut q, status);
                return round_pack_canonical_d(&mut q, status);
            }
            return a;
        }
        FloatClass::Denormal | FloatClass::Normal => {
            if p.sign {
                status.raise(FLAG_INVALID);
                let mut q = p;
                parts_default_nan(&mut q, status);
                return round_pack_canonical_d(&mut q, status);
            }
            // Mirror single path's IDU for positive denormal inputs
            if p.cls == FloatClass::Denormal {
                status.raise(FLAG_INPUT_DENORMAL_USED);
            }
        }
    }
    let f = f64::from_bits(a);
    let r = f.sqrt();
    let bits = r.to_bits();
    if r * r != f {
        status.raise(FLAG_INEXACT);
    }
    bits
}

/// float64 compare, raising invalid on QNaN inputs (VCMPE.F64).
pub fn compare_f64(a: u64, b: u64, status: &mut FloatStatus) -> FloatRelation {
    let mut pa = unpack_canonical_bits_d(a, status);
    let mut pb = unpack_canonical_bits_d(b, status);
    parts_compare(&mut pa, &mut pb, status, false)
}

/// float64 quiet compare, raising invalid only on SNaN inputs (VCMP.F64).
pub fn compare_signal_f64(a: u64, b: u64, status: &mut FloatStatus) -> FloatRelation {
    let mut pa = unpack_canonical_bits_d(a, status);
    let mut pb = unpack_canonical_bits_d(b, status);
    parts_compare(&mut pa, &mut pb, status, true)
}

/// float64 -> int32, using the FPSCR rounding mode (VCVT.S32.F64).
pub fn to_int32_f64(a: u64, status: &mut FloatStatus) -> i32 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_float_to_sint(
        &mut p,
        status.rounding_mode,
        0,
        i32::MIN as i64,
        i32::MAX as i64,
        status,
    ) as i32
}

/// float64 -> int32 with round-to-zero (VCVTTR? no - VCVT with RZ semantics).
pub fn to_int32_round_to_zero_f64(a: u64, status: &mut FloatStatus) -> i32 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_float_to_sint(
        &mut p,
        RoundMode::ToZero,
        0,
        i32::MIN as i64,
        i32::MAX as i64,
        status,
    ) as i32
}

/// float64 -> uint32, using the FPSCR rounding mode.
pub fn to_uint32_f64(a: u64, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_float_to_uint(&mut p, status.rounding_mode, 0, u32::MAX as u64, status) as u32
}

/// float64 -> uint32 with round-to-zero.
pub fn to_uint32_round_to_zero_f64(a: u64, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_float_to_uint(&mut p, RoundMode::ToZero, 0, u32::MAX as u64, status) as u32
}

/// float64 -> int32 with a scale, round-to-zero (VCVT to fixed-point).
pub fn to_int32_scalbn_f64(a: u64, shift: i32, status: &mut FloatStatus) -> i32 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_float_to_sint(
        &mut p,
        RoundMode::ToZero,
        shift,
        i32::MIN as i64,
        i32::MAX as i64,
        status,
    ) as i32
}

/// float64 -> uint32 with a scale, round-to-zero (VCVT to unsigned
/// fixed-point).
pub fn to_uint32_scalbn_f64(a: u64, shift: i32, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_float_to_uint(&mut p, RoundMode::ToZero, shift, u32::MAX as u64, status) as u32
}

/// float64 -> int16 with a scale, round-to-zero.
pub fn to_int16_scalbn_f64(a: u64, shift: i32, status: &mut FloatStatus) -> i16 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_float_to_sint(
        &mut p,
        RoundMode::ToZero,
        shift,
        i16::MIN as i64,
        i16::MAX as i64,
        status,
    ) as i16
}

/// float64 -> uint16 with a scale, round-to-zero.
pub fn to_uint16_scalbn_f64(a: u64, shift: i32, status: &mut FloatStatus) -> u16 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_float_to_uint(&mut p, RoundMode::ToZero, shift, u16::MAX as u64, status) as u16
}

/// float64 round to integral (VRINT*).
pub fn round_to_int_f64(a: u64, status: &mut FloatStatus) -> u64 {
    let mut p = unpack_canonical_bits_d(a, status);
    parts_round_to_int(&mut p, status.rounding_mode, 0, status);
    round_pack_canonical_d(&mut p, status)
}

fn zero_parts() -> Parts {
    Parts {
        sign: false,
        exp: 0,
        frac: 0,
        cls: FloatClass::Zero,
    }
}

/// int32 -> float64.
pub fn int32_to_f64(a: i32, status: &mut FloatStatus) -> u64 {
    let mut p = zero_parts();
    parts_sint_to_float(&mut p, a as i64, 0);
    round_pack_canonical_d(&mut p, status)
}

/// uint32 -> float64.
pub fn uint32_to_f64(a: u32, status: &mut FloatStatus) -> u64 {
    let mut p = zero_parts();
    parts_uint_to_float(&mut p, u64::from(a), 0);
    round_pack_canonical_d(&mut p, status)
}

/// int32 -> float64 with a scale (fixed-point source).
pub fn int32_to_f64_scalbn(a: i32, shift: i32, status: &mut FloatStatus) -> u64 {
    let mut p = zero_parts();
    parts_sint_to_float(&mut p, a as i64, -shift);
    round_pack_canonical_d(&mut p, status)
}

/// uint32 -> float64 with a scale (fixed-point source).
pub fn uint32_to_f64_scalbn(a: u32, shift: i32, status: &mut FloatStatus) -> u64 {
    let mut p = zero_parts();
    parts_uint_to_float(&mut p, u64::from(a), -shift);
    round_pack_canonical_d(&mut p, status)
}

/// int16 -> float64 with a scale (fixed-point source).
pub fn int16_to_f64_scalbn(a: i16, shift: i32, status: &mut FloatStatus) -> u64 {
    let mut p = zero_parts();
    parts_sint_to_float(&mut p, a as i64, -shift);
    round_pack_canonical_d(&mut p, status)
}

/// uint16 -> float64 with a scale (fixed-point source).
pub fn uint16_to_f64_scalbn(a: u16, shift: i32, status: &mut FloatStatus) -> u64 {
    let mut p = zero_parts();
    parts_uint_to_float(&mut p, u64::from(a), -shift);
    round_pack_canonical_d(&mut p, status)
}

/// float32 -> float64 (exact widening; never raises flags for normals).
pub fn f32_to_f64(a: u32, status: &mut FloatStatus) -> u64 {
    let mut p = unpack_canonical_bits(a, status);
    round_pack_canonical_d(&mut p, status)
}

/// float64 -> float32.
pub fn f64_to_f32(a: u64, status: &mut FloatStatus) -> u32 {
    let mut p = unpack_canonical_bits_d(a, status);
    round_pack_canonical(&mut p, status)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f32(x: f32) -> u32 {
        x.to_bits()
    }

    fn eq(a: u32, b: u32) -> bool {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        compare(a, b, &mut s) == FloatRelation::Equal
    }

    #[test]
    fn sub_mixed_magnitudes() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // exp_diff < 0: b.exp > a.exp, result gets b's exponent.
        assert_eq!(sub(f32(1.0), f32(2.0), &mut s), f32(-1.0));
        assert_eq!(sub(f32(0.5), f32(1.0), &mut s), f32(-0.5));
        // exp_diff == 0, exact cancellation.
        assert_eq!(sub(f32(3.0), f32(3.0), &mut s), f32(0.0));
        assert_eq!(s.take_flags(), 0);
        // exp_diff == 0, A < B requires the borrow path.
        assert_eq!(sub(f32(1.5), f32(1.75), &mut s), f32(-0.25));
        // exp_diff > 0 normal path.
        assert_eq!(sub(f32(2.0), f32(1.0), &mut s), f32(1.0));
    }

    #[test]
    fn muladd_subtraction_paths() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // a*b - c with |c| > |a*b|: wide_sub_normal exp_diff < 0.
        assert_eq!(
            muladd_neg(f32(1.0), f32(1.0), f32(2.0), MULADD_NEGATE_C, &mut s),
            f32(-1.0)
        );
        // a*b - c with exact cancellation: wide_sub_normal returns false.
        assert_eq!(
            muladd_neg(f32(1.0), f32(2.0), f32(2.0), MULADD_NEGATE_C, &mut s),
            f32(0.0)
        );
        // a*b - c with equal exponents, borrow path.
        assert_eq!(
            muladd_neg(f32(1.5), f32(1.0), f32(1.75), MULADD_NEGATE_C, &mut s),
            f32(-0.25)
        );
    }

    #[test]
    fn add_rounds_to_nearest_even() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(add(f32(1.0), f32(0.5), &mut s), f32(1.5));
        // 16777216.0 + 1.0 rounds to 16777216.0 (half way, even).
        assert_eq!(add(f32(16_777_216.0), f32(1.0), &mut s), f32(16_777_216.0));
        // 16777216.0 + 3.0 rounds up to 16777220.0.
        assert_eq!(add(f32(16_777_216.0), f32(3.0), &mut s), f32(16_777_220.0));
        // -0.0 + -0.0 is -0.0; 0.0 + -0.0 is +0.0 under RNE.
        assert_eq!(add(f32(-0.0), f32(-0.0), &mut s), f32(-0.0));
        assert_eq!(add(f32(0.0), f32(-0.0), &mut s), f32(0.0));
        // The rounding add (16777216.0 + 1.0) lost 1.0, so IXC is sticky.
        assert_eq!(s.take_flags(), FLAG_INEXACT);
    }

    #[test]
    fn add_rounding_direction_dependence() {
        // 1.0 + 1.0
        for mode in [
            RoundMode::NearestEven,
            RoundMode::Down,
            RoundMode::Up,
            RoundMode::ToZero,
        ] {
            let mut s = FloatStatus::new(mode);
            assert_eq!(add(f32(1.0), f32(1.0), &mut s), f32(2.0));
        }
    }

    #[test]
    fn add_infinity_and_nan() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // inf + -inf = NaN, invalid.
        let inf = f32(f32::INFINITY);
        let ninf = f32(f32::NEG_INFINITY);
        let nan = add(inf, ninf, &mut s);
        assert_eq!(nan, 0x7fc0_0000);
        assert!(s.take_flags() & FLAG_INVALID != 0);
        // inf + 1 = inf
        assert_eq!(add(inf, f32(1.0), &mut s), inf);
        // QNaN propagates.
        let qnan = 0x7fc0_1234;
        assert_eq!(add(qnan, f32(1.0), &mut s), qnan);
        // SNaN silenced + invalid.
        assert_eq!(add(0x7f80_0001, f32(1.0), &mut s), 0x7fc0_0001);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_SNAN) != 0);
    }

    #[test]
    fn sub_less_signs() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // +0.0 - +0.0 = +0.0 under RNE.
        assert_eq!(sub(f32(0.0), f32(0.0), &mut s), f32(0.0));
        // Under round-down, +0.0 - +0.0 = -0.0.
        let mut s2 = FloatStatus::new(RoundMode::Down);
        assert_eq!(sub(f32(0.0), f32(0.0), &mut s2), f32(-0.0));
        // -0.0 - +0.0 = -0.0.
        assert_eq!(sub(f32(-0.0), f32(0.0), &mut s), f32(-0.0));
    }

    #[test]
    fn mul_and_denormal() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(mul(f32(2.0), f32(3.0), &mut s), f32(6.0));
        // Smallest normal * 2 => 2 * min normal = next denormal boundary.
        let min_norm = f32::from_bits(0x0080_0000);
        assert_eq!(mul(f32(min_norm), f32(2.0), &mut s), 0x0100_0000);
        // inf * 0 = NaN + invalid.
        let inf = f32(f32::INFINITY);
        let r = mul(inf, f32(0.0), &mut s);
        assert_eq!(r, 0x7fc0_0000);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_IMZ) != 0);
        // 0 * -5 = -0.0
        assert_eq!(mul(f32(0.0), f32(-5.0), &mut s), f32(-0.0));
    }

    #[test]
    fn div_special_values() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(div(f32(1.0), f32(2.0), &mut s), f32(0.5));
        // 1/0 = +inf with divbyzero.
        assert_eq!(div(f32(1.0), f32(0.0), &mut s), f32(f32::INFINITY));
        assert!(s.take_flags() & FLAG_DIVBYZERO != 0);
        // 0/0 = NaN invalid.
        assert_eq!(div(f32(0.0), f32(0.0), &mut s), 0x7fc0_0000);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_ZDZ) != 0);
        // inf/inf = NaN invalid.
        assert_eq!(
            div(f32(f32::INFINITY), f32(f32::INFINITY), &mut s),
            0x7fc0_0000
        );
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_IDI) != 0);
    }

    #[test]
    fn flush_to_zero() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        s.flush_to_zero = true;
        // 0x00800000 (min normal) / 2 = 0x00400000 (denormal) -> flushed to 0.
        let r = div(0x0080_0000, f32(2.0), &mut s);
        assert_eq!(r, 0);
        assert!(s.take_flags() & FLAG_OUTPUT_DENORMAL_FLUSHED != 0);
        // Denormal input flushed to zero.
        let r2 = mul(0x0000_0001, f32(2.0), &mut s);
        assert_eq!(r2, 0);
        assert!(s.take_flags() & FLAG_INPUT_DENORMAL_FLUSHED != 0);
    }

    #[test]
    fn default_nan_mode() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        s.default_nan_mode = true;
        let qnan = 0x7fc0_1234;
        // Under DN, QNaN input becomes the default NaN.
        assert_eq!(add(qnan, f32(1.0), &mut s), 0x7fc0_0000);
        // SNaN becomes default NaN + invalid.
        assert_eq!(add(0x7f80_0001, f32(1.0), &mut s), 0x7fc0_0000);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_SNAN) != 0);
    }

    #[test]
    fn packing_roundtrip() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        for raw in [
            0u32,
            0x3f80_0000,
            0xc000_0000,
            0x7f80_0000,
            0x0080_0000,
            0x0000_0001,
        ] {
            let mut p = unpack_canonical_bits(raw, &mut s);
            let packed = round_pack_canonical(&mut p, &mut s);
            assert_eq!(packed, raw, "roundtrip failed for {raw:#x}");
        }
    }

    #[test]
    fn add_denormal_input_used_flag() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // denormal + normal: input_denormal_used raised.
        let _ = add(0x0000_0001, f32(1.0), &mut s);
        assert!(s.take_flags() & FLAG_INPUT_DENORMAL_USED != 0);
    }

    #[test]
    fn eq_helper_smoke() {
        assert!(eq(f32(1.0), f32(1.0)));
        assert!(eq(f32(-0.0), f32(0.0)));
        assert!(!eq(f32(1.0), f32(2.0)));
        assert!(!eq(f32(f32::NAN), f32(1.0)));
    }

    #[test]
    fn sqrt_basic() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(sqrt(f32(4.0), &mut s), f32(2.0));
        assert_eq!(sqrt(f32(2.0), &mut s), f32(std::f32::consts::SQRT_2));
        // sqrt(-1) = NaN invalid.
        let r = sqrt(f32(-1.0), &mut s);
        assert_eq!(r, 0x7fc0_0000);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_SQRT) != 0);
    }

    #[test]
    fn sqrt_handles_denormal_input() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        let denormal = 0x0000_0001u32;
        let r = sqrt(denormal, &mut s);
        // sqrt(2^-149) = 2^-74.5, a small normal (approx 0x1A3504F3).
        assert_eq!(r, 0x1A35_04F3);
        assert!(s.take_flags() & FLAG_INPUT_DENORMAL_USED != 0);
    }

    #[test]
    fn muladd_fused() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // 2*3 + 1 = 7.
        assert_eq!(muladd(f32(2.0), f32(3.0), f32(1.0), &mut s), f32(7.0));
        // Fused: (2^24) * 1.0 + (-1.0) computes exactly (unfused would round).
        // a = 16777216, b = 1.0, c = -1.0 -> 16777215.
        assert_eq!(
            muladd(f32(16_777_216.0), f32(1.0), f32(-1.0), &mut s),
            f32(16_777_215.0)
        );
    }

    #[test]
    fn muladd_negations() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // VFMS: a*b - c.
        assert_eq!(
            muladd_neg(f32(2.0), f32(3.0), f32(1.0), MULADD_NEGATE_C, &mut s),
            f32(5.0)
        );
        // VFNMA: -a*b + c.
        assert_eq!(
            muladd_neg(f32(2.0), f32(3.0), f32(1.0), MULADD_NEGATE_PRODUCT, &mut s),
            f32(-5.0)
        );
        // VFNMS: -a*b - c.
        assert_eq!(
            muladd_neg(
                f32(2.0),
                f32(3.0),
                f32(1.0),
                MULADD_NEGATE_C | MULADD_NEGATE_PRODUCT,
                &mut s
            ),
            f32(-7.0)
        );
    }

    #[test]
    fn round_to_int_modes() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(round_to_int(f32(1.5), &mut s), f32(2.0));
        assert_eq!(round_to_int(f32(2.5), &mut s), f32(2.0));
        let mut s2 = FloatStatus::new(RoundMode::ToZero);
        assert_eq!(round_to_int(f32(1.5), &mut s2), f32(1.0));
        assert_eq!(round_to_int(f32(-1.5), &mut s2), f32(-1.0));
        let mut s3 = FloatStatus::new(RoundMode::Down);
        assert_eq!(round_to_int(f32(1.5), &mut s3), f32(1.0));
        assert_eq!(round_to_int(f32(-1.5), &mut s3), f32(-2.0));
    }

    #[test]
    fn float_to_int16_scalbn() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // 1.5 * 2^16 = 98304 > i16::MAX, saturates with invalid.
        assert_eq!(to_int16_scalbn(f32(1.5), 16, &mut s), i16::MAX);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_CVTI) != 0);
        // 0.5 * 2^16 = 32768, still > 32767, saturates.
        assert_eq!(to_int16_scalbn(f32(0.5), 16, &mut s), i16::MAX);
    }

    #[test]
    fn compare_infinity() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        let inf = f32(f32::INFINITY);
        assert_eq!(compare(inf, f32(1.0), &mut s), FloatRelation::Greater);
        assert_eq!(compare(f32(1.0), inf, &mut s), FloatRelation::Less);
        assert_eq!(compare(inf, inf, &mut s), FloatRelation::Equal);
        assert_eq!(
            compare(f32(-f32::INFINITY), f32(f32::INFINITY), &mut s),
            FloatRelation::Less
        );
    }

    #[test]
    fn compare_zero_vs_signed() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(compare(f32(0.0), f32(-0.0), &mut s), FloatRelation::Equal);
        assert_eq!(compare(f32(0.0), f32(1.0), &mut s), FloatRelation::Less);
        assert_eq!(compare(f32(-1.0), f32(0.0), &mut s), FloatRelation::Less);
    }

    #[test]
    fn float_to_int_conversions() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(to_int32(f32(3.7), &mut s), 4);
        assert_eq!(s.take_flags() & FLAG_INEXACT, FLAG_INEXACT);
        assert_eq!(to_int32_round_to_zero(f32(3.7), &mut s), 3);
        // Out of range saturates.
        assert_eq!(to_int32(f32(1e20), &mut s), i32::MAX);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_CVTI) != 0);
        assert_eq!(to_int32(f32(-1e20), &mut s), i32::MIN);
        // NaN -> raw softfloat returns max with invalid (no VFP pre-check
        // here; the VFP CONV_FTOI wrapper will return 0 at the execute layer).
        assert_eq!(to_int32(0x7fc0_0000, &mut s), i32::MAX);
        assert!(s.take_flags() & FLAG_INVALID != 0);
        // unsigned
        assert_eq!(to_uint32(f32(3.7), &mut s), 4);
        assert_eq!(to_uint32_round_to_zero(f32(3.7), &mut s), 3);
        assert_eq!(to_uint32(f32(-1.0), &mut s), 0);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_CVTI) != 0);
    }

    #[test]
    fn int_to_float_conversions() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(int32_to_float32(3, &mut s), f32(3.0));
        assert_eq!(int32_to_float32(-3, &mut s), f32(-3.0));
        assert_eq!(int32_to_float32(0, &mut s), f32(0.0));
        // 16777217 (2^24+1) cannot be represented; rounds to 2^24.
        assert_eq!(int32_to_float32(16_777_217, &mut s), f32(16_777_216.0));
        assert_eq!(uint32_to_float32(u32::MAX, &mut s), f32(4_294_967_296.0));
        assert_eq!(uint16_to_float32_scalbn(1, 8, &mut s), f32(1.0 / 256.0));
    }
}
// The tests module is defined above; float64 tests live in their own module
// to keep the f32/f64 cases separate.

#[cfg(test)]
mod f64_tests {
    use super::*;

    fn f64(x: f64) -> u64 {
        x.to_bits()
    }

    fn eq_d(a: u64, b: u64) -> bool {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        compare_f64(a, b, &mut s) == FloatRelation::Equal
    }

    #[test]
    fn add_sub_basic() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(add_f64(f64(1.5), f64(2.25), &mut s), f64(3.75));
        assert_eq!(sub_f64(f64(1.0), f64(2.0), &mut s), f64(-1.0));
        assert_eq!(sub_f64(f64(3.0), f64(3.0), &mut s), f64(0.0));
        assert_eq!(s.take_flags(), 0);
        // Exact cancellation rounds -0 in RZ... nearest gives +0.
        assert_eq!(sub_f64(f64(-2.5), f64(-2.5), &mut s), f64(0.0));
    }

    #[test]
    fn mul_div_precision() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // 1/3 in double has full 52-bit precision; single would round.
        let third = div_f64(f64(1.0), f64(3.0), &mut s);
        assert!(eq_d(mul_f64(third, f64(3.0), &mut s), f64(1.0)));
        let big = mul_f64(f64(6.02e23), f64(9.81), &mut s);
        assert_eq!(big >> 32, f64(6.02f64 * 9.81e23) >> 32);
    }

    #[test]
    fn overflow_to_inf_with_flags() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        let r = mul_f64(f64(1e308), f64(10.0), &mut s);
        assert!(r >> 52 == 0x7ff && r & 0xf_ffff_ffff_ffff == 0); // +inf
        assert_eq!(s.take_flags(), FLAG_OVERFLOW | FLAG_INEXACT);
    }

    #[test]
    fn div_by_zero_raises_idz() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert!(eq_d(
            div_f64(f64(1.0), f64(0.0), &mut s),
            f64(f64::INFINITY)
        ));
        assert_eq!(s.take_flags(), FLAG_DIVBYZERO);
    }

    #[test]
    fn invalid_inf_minus_inf() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // Sanity: inf - (-inf) is exactly +inf with no flags.
        assert_eq!(
            sub_f64(f64(f64::INFINITY), f64(f64::NEG_INFINITY), &mut s),
            f64(f64::INFINITY)
        );
        assert_eq!(s.take_flags(), 0);
        let r = sub_f64(f64(f64::INFINITY), f64(f64::INFINITY), &mut s);
        assert!(
            (r & 0x7ff0_0000_0000_0000) == 0x7ff0_0000_0000_0000
                && (r & 0x000f_ffff_ffff_ffff) != 0
        );
        assert_eq!(s.take_flags(), FLAG_INVALID | FLAG_INVALID_ISI);
    }

    #[test]
    fn sqrt_exact_and_invalid() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(sqrt_f64(f64(4.0), &mut s), f64(2.0));
        assert_eq!(sqrt_f64(f64(2.25), &mut s), f64(1.5));
        // sqrt of a non-perfect square is correctly rounded.
        assert_eq!(sqrt_f64(f64(2.0), &mut s), f64(std::f64::consts::SQRT_2));
        assert_eq!(s.take_flags(), FLAG_INEXACT);
        let r = sqrt_f64(0x8000_0000_0000_0001, &mut s); // -denormal
        assert!((r & 0x7ff0_0000_0000_0000) == 0x7ff0_0000_0000_0000);
        assert_eq!(s.take_flags(), FLAG_INVALID);
    }

    #[test]
    fn underflow_denormal_result() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // Smallest normal / 16 -> denormal, raises UFC|IXC.
        // Exact denormal product: no inexact, hence no underflow flag.
        let tiny = mul_f64(f64(2.2250738585072014e-308), f64(0.25), &mut s);
        assert_eq!(tiny, f64(2.2250738585072014e-308 * 0.25));
        // Inexact denormal result: UFC|IXC (plus IDU on some paths).
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        let tinier = mul_f64(f64(2.2250738585072014e-308), f64(1e-10), &mut s);
        assert_ne!(tinier, 0);
        let flags = s.take_flags();
        assert!(flags & FLAG_UNDERFLOW != 0, "flags={flags:#x}");
        assert!(flags & FLAG_INEXACT != 0, "flags={flags:#x}");
        // Halving the minimum denormal ties to even -> zero, inexact.
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        let tinier = mul_f64(f64(4.9e-324), f64(0.5), &mut s);
        assert_eq!(tinier, 0);
        let flags = s.take_flags();
        assert!(flags & FLAG_UNDERFLOW != 0, "flags={flags:#x}");
        // IDU is also raised for denormal inputs.
    }

    #[test]
    fn directed_rounding() {
        // 1 + 2^-53 is exactly halfway between 1.0 and nextafter(1.0).
        let half_ulp = f64(2f64.powi(-53));

        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(add_f64(f64(1.0), half_ulp, &mut s), f64(1.0)); // ties to even
        assert_eq!(s.take_flags(), FLAG_INEXACT);

        let mut s = FloatStatus::new(RoundMode::Up);
        assert_eq!(add_f64(f64(1.0), half_ulp, &mut s), f64(1.0) + 1);

        let mut s = FloatStatus::new(RoundMode::Down);
        assert_eq!(add_f64(f64(1.0), half_ulp, &mut s), f64(1.0));
        assert_eq!(s.take_flags(), FLAG_INEXACT);

        let mut s = FloatStatus::new(RoundMode::ToZero);
        assert_eq!(add_f64(f64(-1.0), f64(-2f64.powi(-53)), &mut s), f64(-1.0));
    }

    #[test]
    fn converts_int_round_trip() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(to_int32_f64(int32_to_f64(-12345, &mut s), &mut s), -12345);
        assert_eq!(
            to_uint32_f64(uint32_to_f64(u32::MAX - 1, &mut s), &mut s),
            u32::MAX - 1
        );
        // Out-of-range saturates with invalid.
        let big = int32_to_f64(i32::MAX, &mut s);
        let doubled = add_f64(big, big, &mut s);
        assert_eq!(to_int32_f64(doubled, &mut s), i32::MAX);
        assert_eq!(s.take_flags(), FLAG_INVALID | FLAG_INVALID_CVTI);
    }

    #[test]
    fn widen_narrow_f32() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        let w = f32_to_f64((0.1f32).to_bits(), &mut s);
        assert_eq!(w, f64(0.1f32 as f64)); // exact widening
                                           // Narrowing rounds: double 0.1 -> float 0.1 bits.
        let n = f64_to_f32(f64(0.1), &mut s);
        assert_eq!(n, (0.1f32).to_bits());
        // Overflowing narrow -> inf with OFC|IXC.
        let mut s2 = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(f64_to_f32(f64(1e300), &mut s2), (f32::INFINITY).to_bits());
        assert_eq!(s2.take_flags(), FLAG_OVERFLOW | FLAG_INEXACT);
    }

    #[test]
    fn default_nan_is_dp_pattern() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // inf + (-inf) -> default NaN with exponent field all ones (dp).
        let r = add_f64(f64(f64::INFINITY), f64(f64::NEG_INFINITY), &mut s);
        // Exponent field all ones and the ARM default-NaN quiet bit set.
        assert_eq!(r & 0xffff_0000_0000_0000, 0x7ff8_0000_0000_0000);
    }

    #[test]
    fn compare_relations() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        let a = f64(1.0);
        let b = f64(2.0);
        assert_eq!(compare_f64(a, b, &mut s), FloatRelation::Less);
        // Quiet compare raises only on SNaN inputs.
        assert_eq!(
            compare_signal_f64(f64(f64::NAN), f64(0.0), &mut s),
            FloatRelation::Unordered
        );
        assert_eq!(s.take_flags(), 0);
        assert_eq!(
            compare_signal_f64(0x7ff0_0000_0000_0001, f64(0.0), &mut s),
            FloatRelation::Unordered
        );
        assert_eq!(s.take_flags(), FLAG_INVALID | FLAG_INVALID_SNAN);
    }

    #[test]
    fn round_to_int_f64_modes() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        assert_eq!(round_to_int_f64(f64(1.5), &mut s), f64(2.0));
        assert_eq!(round_to_int_f64(f64(2.5), &mut s), f64(2.0));
        let mut s2 = FloatStatus::new(RoundMode::ToZero);
        assert_eq!(round_to_int_f64(f64(1.5), &mut s2), f64(1.0));
        assert_eq!(round_to_int_f64(f64(-1.5), &mut s2), f64(-1.0));
        let mut s3 = FloatStatus::new(RoundMode::Down);
        assert_eq!(round_to_int_f64(f64(1.5), &mut s3), f64(1.0));
        assert_eq!(round_to_int_f64(f64(-1.5), &mut s3), f64(-2.0));
    }

    #[test]
    fn f64_fixed_point_scalbn() {
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // 1.5 * 2^16 = 98304 > i16::MAX -> saturates
        assert_eq!(to_int16_scalbn_f64(f64(1.5), 16, &mut s), i16::MAX);
        assert!(s.take_flags() & (FLAG_INVALID | FLAG_INVALID_CVTI) != 0);
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        // int16 -> f64 with scale 4: 8 -> 0.5
        let r = int16_to_f64_scalbn(8, 4, &mut s);
        assert_eq!(r, f64(0.5));
        let mut s = FloatStatus::new(RoundMode::NearestEven);
        let r = uint32_to_f64_scalbn(1, 1, &mut s);
        assert_eq!(r, f64(0.5));
    }
}
