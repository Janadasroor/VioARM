//! Core register state: the ARMv6-M register bank and the `xPSR`.

/// Application-level program status register bits we model.
pub const APSR_N: u32 = 1 << 31;
pub const APSR_Z: u32 = 1 << 30;
pub const APSR_C: u32 = 1 << 29;
pub const APSR_V: u32 = 1 << 28;
pub const APSR_FLAGS: u32 = APSR_N | APSR_Z | APSR_C | APSR_V;

/// Thumb-mode indication bit (bit 24 of `xPSR`).
pub const XPSR_T: u32 = 1 << 24;

/// `xPSR` bits we allow host code to inspect and modify. Everything else is
/// reserved (and reading reserved bits yields 0 on the hardware we model).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Xpsr(u32);

impl Xpsr {
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn get(&self) -> u32 {
        self.0
    }

    pub const fn set(&mut self, value: u32) {
        self.0 = value;
    }

    #[must_use]
    pub const fn n(&self) -> bool {
        self.0 & APSR_N != 0
    }

    #[must_use]
    pub const fn z(&self) -> bool {
        self.0 & APSR_Z != 0
    }

    #[must_use]
    pub const fn c(&self) -> bool {
        self.0 & APSR_C != 0
    }

    #[must_use]
    pub const fn v(&self) -> bool {
        self.0 & APSR_V != 0
    }

    #[must_use]
    pub const fn is_thumb(&self) -> bool {
        self.0 & XPSR_T != 0
    }

    pub fn set_nzcv(&mut self, n: bool, z: bool, c: bool, v: bool) {
        self.0 &= !APSR_FLAGS;
        self.0 |= u32::from(n) << 31 | u32::from(z) << 30 | u32::from(c) << 29 | u32::from(v) << 28;
    }

    /// Set the Thumb bit. The simulator only executes Thumb; clearing it is
    /// not representable as a continuing state, so this is host-only.
    pub fn set_thumb(&mut self, t: bool) {
        if t {
            self.0 |= XPSR_T;
        } else {
            self.0 &= !XPSR_T;
        }
    }
}

/// The general-purpose register bank. `r0..r12` are plain; `sp`, `lr`, `pc`
/// are modelled alongside for one-hot semantics in instructions that accept a
/// high register.
///
/// ARMv6-M has two stack pointers. `sp` is the *currently active* one; `msp`
/// and `psp` hold the banked copies so switching `SPSEL` does not lose state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Registers {
    pub r: [u32; 13],
    pub sp: u32,
    /// Main stack pointer (the active `sp` when `SPSEL == 0`).
    pub msp: u32,
    /// Process stack pointer (the active `sp` when `SPSEL == 1`).
    pub psp: u32,
    pub lr: u32,
    pub pc: u32,
}

impl Default for Registers {
    fn default() -> Self {
        Self {
            r: [0; 13],
            sp: 0,
            msp: 0,
            psp: 0,
            lr: 0xFFFF_FFFF,
            pc: 0,
        }
    }
}

impl Registers {
    #[must_use]
    pub const fn new(pc: u32, sp: u32) -> Self {
        Self {
            r: [0; 13],
            sp,
            msp: sp,
            psp: 0,
            lr: 0xFFFF_FFFF,
            pc,
        }
    }

    /// Read an architectural register number 0..=15.
    #[inline]
    #[must_use]
    pub fn read(&self, n: u8) -> u32 {
        match n {
            0..=12 => self.r[n as usize],
            13 => self.sp,
            14 => self.lr,
            15 => self.pc,
            _ => unreachable!("register number {n} out of range"),
        }
    }

    /// Write an architectural register number 0..=15.
    #[inline]
    pub fn write(&mut self, n: u8, value: u32) {
        match n {
            0..=12 => self.r[n as usize] = value,
            13 => self.sp = value,
            14 => self.lr = value,
            15 => self.pc = value,
            _ => unreachable!("register number {n} out of range"),
        }
    }
}

/// A point-in-time view of the CPU suitable for `debug`/`trace`/tests. It is a
/// plain value: copying it is cheap and `snapshot()` does not allocate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    pub regs: Registers,
    pub xpsr: Xpsr,
    pub cycles: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xpsr_flags_roundtrip() {
        let mut x = Xpsr::default();
        x.set_nzcv(true, false, true, false);
        assert!(x.n());
        assert!(!x.z());
        assert!(x.c());
        assert!(!x.v());
        assert_eq!(x.get() & APSR_FLAGS, APSR_N | APSR_C);
    }

    #[test]
    fn xpsr_thumb_bit() {
        let mut x = Xpsr::default();
        assert!(!x.is_thumb());
        x.set_thumb(true);
        assert!(x.is_thumb());
        assert_eq!(x.get(), XPSR_T);
    }

    #[test]
    fn register_bank_indexing() {
        let mut r = Registers::new(0x1000, 0x2000_0000);
        r.write(7, 0xAA);
        assert_eq!(r.read(7), 0xAA);
        r.write(13, 0x2000_0100);
        assert_eq!(r.sp, 0x2000_0100);
        r.write(15, 0x2004);
        assert_eq!(r.pc, 0x2004);
    }
}
