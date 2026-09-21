//! The STM32F4 system configuration controller (`SYSCFG`, `0x4001_3800`),
//! driven by the SVD-generated register tables.
//!
//! Every register is a pure latch, so [`NoBehavior`] needs no hooks: `MEMRM`,
//! `PMC`, `EXTICR1..4`, and `CMPCR` all roundtrip so firmware that routes EXTI
//! lines through `EXTICR` or remaps the memory map does not fault. Routing is
//! accepted but not acted on — the EXTI model wires line 0 directly. `SYSCFG`
//! is the Cortex-M4 successor to the F1 `AFIO`.

use crate::device::svd::stm32f407;
use crate::device::svd::{Behavior, Device, NoBehavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// The `SYSCFG` register file.
#[derive(Debug)]
pub struct Syscfg {
    inner: RegisterFile<NoBehavior>,
    /// Tight window over the declared registers. Tables grant fat windows
    /// that swallow neighbors (F0 `SYSCFG@0x40010000/0x400` covers
    /// `COMP@0x4001001C` and reaches toward `EXTI`), and dispatch stops at
    /// the first claimant — so the latch must not advertise address space
    /// beyond its registers.
    window: [Region; 1],
}

impl Syscfg {
    /// A new `SYSCFG` at its STM32F4 base address.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `SYSCFG`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        Self::for_device::<stm32f407::Stm32f407>()
    }

    /// A new `SYSCFG` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `SYSCFG`
    /// peripheral.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        // Generic F0 donor tables merge SYSCFG (+COMP CSR) into one
        // `SYSCFG_COMP` window; same latch contract either way.
        let def = D::peripheral("SYSCFG")
            .or_else(|| D::peripheral("SYSCFG_COMP"))
            .expect("generated tables contain SYSCFG");
        let mut lo = u32::MAX;
        let mut hi = 0u32;
        for r in def.registers {
            lo = lo.min(r.offset);
            hi = hi.max(r.offset + u32::from(r.width) / 8);
        }
        let window = if lo <= hi {
            [Region::new(def.base + lo, hi - lo)]
        } else {
            [Region::new(def.base, def.window)]
        };
        Self {
            inner: RegisterFile::new(def, NoBehavior),
            window,
        }
    }
}

impl Default for Syscfg {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Syscfg {
    fn ranges(&self) -> &[Region] {
        &self.window
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        if self.window[0].contains(addr) {
            self.inner.read(addr)
        } else {
            Err(BusError::Reserved)
        }
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        if self.window[0].contains(addr) {
            self.inner.write(addr, val)
        } else {
            Err(BusError::Reserved)
        }
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn name(&self) -> &'static str {
        "syscfg"
    }
}

impl InterruptSource for Syscfg {
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

/// H7 I/O compensation cell startup, in ticks (silicon needs on the
/// order of milliseconds; the delay itself is idealized, its existence
/// is not).
const H7_READY_TICKS: u64 = 8;

const H7_CCCSR: u32 = 0x20;
const H7_CCCSR_EN: u32 = 1 << 0;
const H7_CCCSR_READY: u32 = 1 << 8;

#[derive(Clone, Debug, Default)]
struct SyscfgH7Behavior {
    ready_at: Option<u64>,
}

impl Behavior for SyscfgH7Behavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let new = view.merged(offset, val, shift);
        if offset == H7_CCCSR {
            if new & H7_CCCSR_EN == 0 {
                // Cell off: READY drops and any pending startup cancels.
                self.ready_at = None;
                return new & !H7_CCCSR_READY;
            }
            if new & H7_CCCSR_READY == 0 && self.ready_at.is_none() {
                self.ready_at = Some(H7_READY_TICKS);
            }
        }
        new
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if let Some(left) = self.ready_at {
            let left = left.saturating_sub(delta);
            self.ready_at = Some(left);
            if left == 0 {
                self.ready_at = None;
                view.set(H7_CCCSR, view.get(H7_CCCSR) | H7_CCCSR_READY);
            }
        }
    }

    fn reset(&mut self) {
        self.ready_at = None;
    }
}

/// H7 system configuration controller (`SYSCFG@0x58000400`).
///
/// `PMCR`, `EXTICR1..4`, `PWRCR`, `CCCR`, and the writable `URx` latch
/// from the tables (routing accepted but not acted on, like the F4
/// latch); `CCVR`/`PKGR`/read-only `URx` are table values. The one live
/// circuit is the I/O compensation cell: setting `CCCSR.EN` raises
/// `READY` after [`H7_READY_TICKS`] ticks, clearing `EN` drops it —
/// the handshake firmware polls before using high-speed I/O.
#[derive(Debug)]
pub struct SyscfgH7 {
    inner: RegisterFile<SyscfgH7Behavior>,
}

impl SyscfgH7 {
    /// A new H7 `SYSCFG` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `SYSCFG`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SYSCFG").expect("generated tables contain SYSCFG");
        Self {
            inner: RegisterFile::new(def, SyscfgH7Behavior::default()),
        }
    }
}

impl Peripheral for SyscfgH7 {
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
        "syscfg"
    }
}

impl InterruptSource for SyscfgH7 {
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
    use super::{NoBehavior, RegisterFile, Syscfg};
    use crate::device::svd::stm32f407;
    use crate::device::Peripheral;
    use crate::error::BusError;

    const BASE: u32 = 0x4001_3800;
    const EXTICR1: u32 = 0x08;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(s: &mut Syscfg, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            s.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[test]
    fn registers_roundtrip() {
        let mut s = Syscfg::new();
        write_word(&mut s, BASE + EXTICR1, 0xABCD_EF01);
        assert_eq!(s.read(BASE + EXTICR1).unwrap(), 0x01);
        assert_eq!(s.read(BASE + EXTICR1 + 3).unwrap(), 0xAB);
    }

    #[test]
    fn gap_between_exticr4_and_cmpcr_is_reserved() {
        let mut s = Syscfg::new();
        assert!(matches!(s.read(BASE + 0x18), Err(BusError::Reserved)));
        assert!(matches!(s.write(BASE + 0x18, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut s = Syscfg::new();
        assert!(matches!(s.read(BASE + 0x400), Err(BusError::Reserved)));
    }

    #[test]
    fn table_matches_silicon_layout() {
        let def = stm32f407::peripheral("SYSCFG").unwrap();
        let expect = [
            ("MEMRM", 0x00),
            ("PMC", 0x04),
            ("EXTICR1", 0x08),
            ("EXTICR2", 0x0C),
            ("EXTICR3", 0x10),
            ("EXTICR4", 0x14),
            ("CMPCR", 0x20),
        ];
        for (name, offset) in expect {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("SYSCFG:{name} present"));
            assert_eq!(r.offset, offset, "SYSCFG:{name} offset");
        }
        let rf = RegisterFile::<NoBehavior>::new(def, NoBehavior);
        assert_eq!(rf.irq_exception("SYSCFG_IRQn"), None);
    }

    #[test]
    fn h7_table_matches_silicon_layout() {
        use crate::device::svd::stm32h743;
        let def = stm32h743::peripheral("SYSCFG").unwrap();
        for (name, offset) in [
            ("PMCR", 0x04),
            ("EXTICR1", 0x08),
            ("EXTICR4", 0x14),
            ("CCCSR", 0x20),
            ("CCVR", 0x24),
            ("CCCR", 0x28),
            ("PWRCR", 0x2C),
            ("PKGR", 0x124),
        ] {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("SYSCFG:{name} present"));
            assert_eq!(r.offset, offset, "SYSCFG:{name} offset");
        }
    }

    #[test]
    fn h7_comp_cell_ready_handshake() {
        use super::SyscfgH7;
        use crate::device::svd::stm32h743;
        const BASE: u32 = 0x5800_0400;
        const CCCSR: u32 = 0x20;
        let mut s = SyscfgH7::new_for::<stm32h743::Stm32h743>();
        s.reset();
        #[allow(clippy::cast_possible_truncation)]
        fn w(s: &mut SyscfgH7, off: u32, val: u32) {
            for (i, b) in val.to_le_bytes().iter().enumerate() {
                s.write(BASE + off + i as u32, *b).unwrap();
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        fn r(s: &mut SyscfgH7, off: u32) -> u32 {
            let mut bytes = [0u8; 4];
            for (i, b) in bytes.iter_mut().enumerate() {
                *b = s.read(BASE + off + i as u32).unwrap();
            }
            u32::from_le_bytes(bytes)
        }
        w(&mut s, CCCSR, 1); // EN
        assert_eq!(r(&mut s, CCCSR) & (1 << 8), 0, "READY waits");
        s.tick(4);
        assert_eq!(r(&mut s, CCCSR) & (1 << 8), 0, "still starting");
        s.tick(4);
        assert_ne!(r(&mut s, CCCSR) & (1 << 8), 0, "READY latches");
        w(&mut s, CCCSR, 0);
        assert_eq!(r(&mut s, CCCSR) & (1 << 8), 0, "EN off drops READY");
    }

    #[test]
    fn h7_exticr_roundtrips() {
        use super::SyscfgH7;
        use crate::device::svd::stm32h743;
        const BASE: u32 = 0x5800_0400;
        let mut s = SyscfgH7::new_for::<stm32h743::Stm32h743>();
        s.reset();
        for (i, b) in 0x3210_7654u32.to_le_bytes().iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let off = i as u32;
            s.write(BASE + 0x08 + off, *b).unwrap();
        }
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let off = i as u32;
            *b = s.read(BASE + 0x08 + off).unwrap();
        }
        assert_eq!(u32::from_le_bytes(bytes), 0x3210_7654);
    }
}
