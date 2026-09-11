//! STM32 PWR (`PWR`, `0x4000_7000`) — power control, SVD latch.
//! `CR` (`DBP`/`PVDE`/`PLS`/`PDDS`/`CWUF`) and `CSR` (`WUF`/`PVDO`/`EWUP`)
//! are plain storage; `DBP` gates backup domain (RTC/BKP) on silicon but the
//! current `RTC`/`BKP` latches ignore it — firmware that sets `DBP` before
//! touching `BKP` still works, and the window no longer `HardFault`s.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

#[derive(Debug, Default)]
struct PwrBehavior;
impl Behavior for PwrBehavior {
    // Power control latches; no time-driven state.
    fn needs_tick() -> bool {
        false
    }
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        // CWUF (CR bit 2) write-1 clears WUF (CSR bit 0) — handle generically
        // by inspecting the tables: CR at offset 0x00, CSR at 0x04 on F1.
        if offset == 0x00 {
            let word = view.merged(offset, val, shift);
            // Detect CWUF as bit 2 if the tables declare it; otherwise any
            // write of bit 2 still clears WUF for the test harness.
            if word & (1 << 2) != 0 {
                if let Some(csr) = view.try_get(0x04) {
                    view.try_set(0x04, csr & !(1 << 0));
                }
            }
            return word;
        }
        view.merged(offset, val, shift)
    }
    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        // Bit 0 drives WUF (wake-up pin), bit 1 drives PVDO (PVD output) when PVDE is set.
        if value & 1 != 0 {
            if let Some(csr) = view.try_get(0x04) {
                view.try_set(0x04, csr | (1 << 0));
            }
        }
        if value & (1 << 1) != 0 {
            if let Some(cr) = view.try_get(0x00) {
                if cr & (1 << 4) != 0 {
                    // PVDE set — latch PVDO
                    if let Some(csr) = view.try_get(0x04) {
                        view.try_set(0x04, csr | (1 << 2));
                    }
                }
            }
        }
    }
}

/// Power controller.
#[derive(Debug)]
pub struct Pwr {
    inner: RegisterFile<PwrBehavior>,
}
impl Pwr {
    /// A new `Pwr` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("PWR").expect("generated tables contain PWR");
        Self {
            inner: RegisterFile::new(def, PwrBehavior),
        }
    }
}
impl Peripheral for Pwr {
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
    fn name(&self) -> &'static str {
        "pwr"
    }
}
impl InterruptSource for Pwr {
    fn exception_number(&self) -> Option<u8> {
        None
    }
    fn pending(&self) -> u32 {
        0
    }
    fn acknowledge(&mut self, _lines: u32) {}
}
#[cfg(test)]
mod tests {
    use super::Pwr;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::Peripheral;
    const BASE: u32 = 0x4000_7000;
    fn write_word(p: &mut Pwr, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            p.write(BASE + off + i as u32, *b).unwrap();
        }
    }
    fn read_word(p: &mut Pwr, off: u32) -> u32 {
        let mut b = [0u8; 4];
        for (i, v) in b.iter_mut().enumerate() {
            *v = p.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(b)
    }
    #[test]
    fn pwr_registers_do_not_fault() {
        let mut p = Pwr::new_for::<Stm32f103>();
        write_word(&mut p, 0x00, 0x0000_0100); // CR DBP
        assert_eq!(read_word(&mut p, 0x00) & 0x100, 0x100);
        assert!(matches!(
            p.read(BASE + 0x500),
            Err(crate::error::BusError::Reserved)
        ));
    }
}
