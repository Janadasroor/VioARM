//! The ARMv7-M System Control Block (subset).
//!
//! Models the `ICSR` `PendSV` set/clear bits, the `SHPR2`/`SHPR3` configurable
//! exception priorities (`SVCall`, `PendSV`, `SysTick`), the `CPACR`
//! coprocessor access-control register, and the reset/control registers
//! `AIRCR`, `SCR`, and `CCR`. The `CPACR` value is stored but does not gate
//! VFP execution (the core implements FPv4-SP unconditionally); the window
//! exists so firmware that enables the FPU (as every Cortex-M4 image does)
//! does not fault. `AIRCR.SYSRESETREQ` latches a reset request the machine
//! honors; `SCR.SLEEPONEXIT` and `CCR.STKALIGN`/`DIV_0_TRP` drive the core's
//! exception-return and exception-entry behavior.
//!
//! The `FPCCR` (`0xE000_EF34`) and `FPCAR` (`0xE000_EF38`) windows expose the
//! M-profile lazy-FP-stacking control to firmware. Their state lives in the
//! shared [`FpuCtx`], which the core also holds, so a firmware write is
//! immediately visible to the exception-entry/FP logic and vice versa.
//!
//! Priority encoding follows the architecture: lower value == higher
//! priority, with the `SVCall` byte in `SHPR2` and the `PendSV`/`SysTick`
//! bytes in `SHPR3` (CMSIS `SCB->SHP[7]`, `SHP[10]`, `SHP[11]`).

use crate::cpu::FpuCtx;
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

const ICSR: u32 = 0xE000_ED04;
const AIRCR: u32 = 0xE000_ED0C;
const SCR: u32 = 0xE000_ED10;
const CCR: u32 = 0xE000_ED14;
const SHPR2: u32 = 0xE000_ED1C;
const SHPR3: u32 = 0xE000_ED20;
const CPACR: u32 = 0xE000_ED88;
const FPCCR: u32 = 0xE000_EF34;
const FPCAR: u32 = 0xE000_EF38;
const FPDSCR: u32 = 0xE000_EF3C;

const PENDSVSET: u8 = 0x10; // ICSR byte 3, bit 28: write 1 to pend.
const PENDSVCLR: u8 = 0x08; // ICSR byte 3, bit 27: write 1 to clear.

// AIRCR: bits 31:16 carry the write `VECTKEY` (0x05FA) and read back as
// `VECTKEYSTAT` (0xFA05); bit 15 `ENDIANNESS` is read-only zero.
const AIRCR_VECTKEY: u32 = 0x05FA_0000;
const AIRCR_PRIGROUP: u32 = 0x0000_0700;
const AIRCR_SYSRESETREQ: u32 = 0x0000_0004;
const AIRCR_VECTCLRACTIVE: u32 = 0x0000_0002;
const AIRCR_VECTRESET: u32 = 0x0000_0001;

// SCR: SEVONPEND (4), SLEEPONEXIT (1). The deep-sleep bits (SLEEPDEEP 2,
// SLEEPDEEPS 3) are RAZ/WI: neither this model nor the QEMU oracle
// implements deep sleep, which the architecture permits.
const SCR_MASK: u32 = 0x0000_0012;

// CCR: STKALIGN (9), BFHFNMIGN (8), DIV_0_TRP (4), UNALIGN_TRP (3),
// USERSETMPEND (1), NONBASETHRDENA (0). STKALIGN is set at reset.
const CCR_MASK: u32 = 0x0000_033B;
const CCR_RESET: u32 = 0x0000_0200;

/// The System Control Block: interrupt control state, exception priorities,
/// the reset/control registers (`AIRCR`/`SCR`/`CCR`), the coprocessor
/// access-control register, and the FP context control.
#[derive(Debug, Clone)]
pub struct Scb {
    ranges: [Region; 10],
    pendsv: bool,
    shpr2: u32,
    shpr3: u32,
    cpacr: u32,
    /// The last `AIRCR` word written (key bytes included), so a 32-bit store's
    /// `VECTKEY` is recognized across its four byte-writes. Reads always
    /// report `VECTKEYSTAT`, never the stored key.
    aircr: u32,
    /// `SCR` (`SEVONPEND`/`SLEEPDEEP`/`SLEEPONEXIT`), written by firmware.
    scr: u32,
    /// `CCR` (`STKALIGN`, `DIV_0_TRP`, ...), written by firmware.
    ccr: u32,
    /// Latched by `AIRCR.SYSRESETREQ`; the machine performs the reset.
    reset_requested: bool,
    /// Shared `FPCCR`/`FPCAR`/`FPDSCR` state; the core holds the same `Rc`.
    fpu_ctx: Arc<Mutex<FpuCtx>>,
}

impl Scb {
    #[must_use]
    pub fn new() -> Self {
        Self {
            ranges: [
                Region::new(ICSR, 4),
                Region::new(AIRCR, 4),
                Region::new(SCR, 4),
                Region::new(CCR, 4),
                Region::new(SHPR2, 4),
                Region::new(SHPR3, 4),
                Region::new(CPACR, 4),
                Region::new(FPCCR, 4),
                Region::new(FPCAR, 4),
                Region::new(FPDSCR, 4),
            ],
            pendsv: false,
            shpr2: 0,
            shpr3: 0,
            cpacr: 0,
            aircr: 0,
            scr: 0,
            ccr: CCR_RESET,
            reset_requested: false,
            fpu_ctx: Arc::new(Mutex::new(FpuCtx::new())),
        }
    }

    /// Share the `FPCCR`/`FPCAR` state with the core. The core created the
    /// `Rc`; adopting it here makes firmware writes and core logic agree.
    pub fn set_fpu_ctx(&mut self, fpu_ctx: Arc<Mutex<FpuCtx>>) {
        self.fpu_ctx = fpu_ctx;
    }

    /// Whether the `PendSV` line is latched pending.
    #[must_use]
    pub const fn pendsv_pending(&self) -> bool {
        self.pendsv
    }

    /// The configurable priority byte for system exception `n` (`11`, `14`,
    /// `15`), lower meaning higher priority. Unrelated numbers read zero.
    #[must_use]
    pub const fn priority(&self, n: u8) -> u8 {
        match n {
            11 => ((self.shpr2 >> 24) & 0xFF) as u8,
            14 => ((self.shpr3 >> 16) & 0xFF) as u8,
            15 => ((self.shpr3 >> 24) & 0xFF) as u8,
            _ => 0,
        }
    }

    /// `AIRCR.PRIGROUP` (0..7): the number of subpriority bits the interrupt
    /// controller uses to split each priority byte into group and
    /// subpriority parts. Only visible after a write that carried the
    /// `VECTKEY`.
    #[must_use]
    pub const fn prig(&self) -> u8 {
        if self.aircr & 0xFFFF_0000 == AIRCR_VECTKEY {
            ((self.aircr & AIRCR_PRIGROUP) >> 8) as u8
        } else {
            0
        }
    }

    /// `SCR`: `SEVONPEND`, `SLEEPDEEP`, and `SLEEPONEXIT`.
    #[must_use]
    pub const fn scr(&self) -> u32 {
        self.scr
    }

    /// `CCR`: `STKALIGN`, `BFHFNMIGN`, `DIV_0_TRP`, `UNALIGN_TRP`,
    /// `USERSETMPEND`, and `NONBASETHRDENA`.
    #[must_use]
    pub const fn ccr(&self) -> u32 {
        self.ccr
    }

    /// Whether `AIRCR.SYSRESETREQ` has been written since the last reset.
    #[must_use]
    pub const fn reset_requested(&self) -> bool {
        self.reset_requested
    }

    fn read_reg(&self, base: u32, offset: u32) -> u8 {
        let reg = match base {
            ICSR => u32::from(self.pendsv) << 28,
            // The read reports VECTKEYSTAT; the stored key bytes are never
            // echoed back.
            AIRCR => 0xFA05_0000 | (u32::from(self.prig()) << 8),
            SCR => self.scr,
            CCR => self.ccr,
            SHPR2 => self.shpr2,
            SHPR3 => self.shpr3,
            CPACR => self.cpacr,
            FPCCR => self.fpu_ctx.lock().unwrap().fpccr(),
            FPCAR => self.fpu_ctx.lock().unwrap().fpcar,
            FPDSCR => self.fpu_ctx.lock().unwrap().fpdscr,
            _ => 0,
        };
        ((reg >> (8 * offset)) & 0xFF) as u8
    }

    fn write_reg(&mut self, base: u32, offset: u32, val: u8) {
        match base {
            ICSR => {
                if offset == 3 {
                    // Only the PendSV set/clear bits are modeled.
                    if val & PENDSVSET != 0 {
                        self.pendsv = true;
                    } else if val & PENDSVCLR != 0 {
                        self.pendsv = false;
                    }
                }
            }
            AIRCR => {
                // Only a store whose bits 31:16 carry `VECTKEY` (0x05FA) is
                // honored. The key bytes may arrive in any of the store's
                // byte-writes, so the last word written (key included) is
                // remembered; reads always report `VECTKEYSTAT`.
                let shift = 8 * offset;
                self.aircr = (self.aircr & !(0xFF << shift)) | (u32::from(val) << shift);
                if self.aircr & 0xFFFF_0000 == AIRCR_VECTKEY {
                    // `VECTRESET` (bit 0) is reserved on the M3/M4 and
                    // `VECTCLRACTIVE` (bit 1) is only meaningful inside a fault
                    // handler; neither is modeled.
                    if self.aircr & AIRCR_SYSRESETREQ != 0 {
                        self.reset_requested = true;
                    }
                }
            }
            SCR => {
                let shift = 8 * offset;
                self.scr = (self.scr & !(0xFF << shift)) | (u32::from(val) << shift);
                self.scr &= SCR_MASK;
            }
            CCR => {
                let shift = 8 * offset;
                self.ccr = (self.ccr & !(0xFF << shift)) | (u32::from(val) << shift);
                self.ccr &= CCR_MASK;
            }
            SHPR2 => {
                let shift = 8 * offset;
                self.shpr2 = (self.shpr2 & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            SHPR3 => {
                let shift = 8 * offset;
                self.shpr3 = (self.shpr3 & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            CPACR => {
                let shift = 8 * offset;
                self.cpacr = (self.cpacr & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            FPCCR => {
                // Only the ASPEN (bit 7 of byte 3) and LSPEN (bit 6 of byte 3)
                // are software-writable; the ready bits and LSPACT are
                // read-only.
                if offset == 3 {
                    let mut ctx = self.fpu_ctx.lock().unwrap();
                    ctx.write_fpccr(u32::from(val) << 24);
                }
            }
            FPCAR => {
                let shift = 8 * offset;
                let mut ctx = self.fpu_ctx.lock().unwrap();
                ctx.fpcar =
                    (ctx.fpcar & !(0xFF << shift)) | (u32::from(val) << shift) & !0x7;
            }
            FPDSCR => {
                let shift = 8 * offset;
                let mut ctx = self.fpu_ctx.lock().unwrap();
                ctx.fpdscr = (ctx.fpdscr & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            _ => {}
        }
    }
}

impl Default for Scb {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Scb {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        for r in &self.ranges {
            if r.contains(addr) {
                return Ok(self.read_reg(r.base, addr - r.base));
            }
        }
        Err(BusError::Reserved)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        for r in &self.ranges {
            if r.contains(addr) {
                self.write_reg(r.base, addr - r.base, val);
                return Ok(());
            }
        }
        Err(BusError::Reserved)
    }

    fn reset(&mut self) {
        self.pendsv = false;
        self.shpr2 = 0;
        self.shpr3 = 0;
        self.cpacr = 0;
        self.aircr = 0;
        self.scr = 0;
        self.ccr = CCR_RESET;
        self.reset_requested = false;
        *self.fpu_ctx.lock().unwrap() = FpuCtx::new();
    }

    fn name(&self) -> &'static str {
        "scb"
    }
}

impl InterruptSource for Scb {
    fn exception_number(&self) -> Option<u8> {
        Some(14)
    }

    fn pending(&self) -> u32 {
        u32::from(self.pendsv)
    }

    fn acknowledge(&mut self, _lines: u32) {
        self.pendsv = false;
    }
}

#[cfg(test)]
mod tests {
    use super::Scb;
    use crate::cpu::FpuCtx;
    use crate::device::Peripheral;
    use std::sync::{Arc, Mutex};

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(s: &mut Scb, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            s.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(s: &mut Scb, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = s.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn icsr_pendsv_set_clear_roundtrip() {
        let mut s = Scb::new();
        assert!(!s.pendsv_pending());
        // ICSR byte 3, bit 4 (PENDSVSET).
        write_word(&mut s, 0xE000_ED04, 0x1000_0000);
        assert!(s.pendsv_pending());
        assert_eq!(read_word(&mut s, 0xE000_ED04), 0x1000_0000);
        // ICSR byte 3, bit 3 (PENDSVCLR).
        write_word(&mut s, 0xE000_ED04, 0x0800_0000);
        assert!(!s.pendsv_pending());
    }

    #[test]
    fn shpr_priorities_stored_per_exception() {
        let mut s = Scb::new();
        // SHPR2 byte 3 -> SVCall; SHPR3 byte 3 -> SysTick; SHPR3 byte 2 -> PendSV.
        s.write(0xE000_ED1F, 0x40).unwrap();
        s.write(0xE000_ED23, 0x20).unwrap();
        s.write(0xE000_ED22, 0x80).unwrap();
        assert_eq!(s.priority(11), 0x40);
        assert_eq!(s.priority(15), 0x20);
        assert_eq!(s.priority(14), 0x80);
        assert_eq!(read_word(&mut s, 0xE000_ED1C), 0x4000_0000);
        assert_eq!(read_word(&mut s, 0xE000_ED20), 0x2080_0000);
    }

    #[test]
    fn reset_clears_pend_and_priorities() {
        let mut s = Scb::new();
        write_word(&mut s, 0xE000_ED04, 0x1000_0000);
        write_word(&mut s, 0xE000_ED1F, 0x10);
        s.reset();
        assert!(!s.pendsv_pending());
        assert_eq!(s.priority(11), 0);
    }

    #[test]
    fn cpacr_stores_and_resets() {
        let mut s = Scb::new();
        // Full access to CP10/CP11, as firmware enabling the FPU writes.
        write_word(&mut s, 0xE000_ED88, 0x00F0_0000);
        assert_eq!(read_word(&mut s, 0xE000_ED88), 0x00F0_0000);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED88), 0);
    }

    #[test]
    fn fpccr_resets_with_ready_bits_and_lazy_on() {
        let mut s = Scb::new();
        // HFRDY|BFRDY|MMRDY|MONRDY|UFRDY plus ASPEN|LSPEN = 0xC7800004.
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0xC780_0004);
        // FPCAR resets to zero.
        assert_eq!(read_word(&mut s, 0xE000_EF38), 0);
    }

    #[test]
    fn fpccr_aspen_lspen_are_software_writable() {
        let mut s = Scb::new();
        // Clear LSPEN (bit 30): ASPEN stays on.
        write_word(&mut s, 0xE000_EF34, 0x8780_0004);
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x8780_0004);
        // Clear ASPEN too: only the ready bits remain.
        write_word(&mut s, 0xE000_EF34, 0x0780_0004);
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x0780_0004);
        // LSPACT (bit 0) is read-only: firmware writes to the low bytes are
        // ignored and never appear in the read value.
        write_word(&mut s, 0xE000_EF34, 0x0000_0001);
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x0780_0004);
    }

    #[test]
    fn fpcar_aligns_and_stores() {
        let mut s = Scb::new();
        // FPCAR holds bits [31:3]; the low three bits read back as zero.
        write_word(&mut s, 0xE000_EF38, 0x2000_083F);
        assert_eq!(read_word(&mut s, 0xE000_EF38), 0x2000_0838);
    }

    #[test]
    fn fpdscr_stores_and_resets() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0);
        // Round-towards-zero: FPSCR RMODE bits 23:22 = 0b11.
        write_word(&mut s, 0xE000_EF3C, 0x00C0_0000);
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0x00C0_0000);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_EF3C), 0);
    }

    #[test]
    fn fpu_ctx_is_shared_with_the_core() {
        let fpu_ctx = Arc::new(Mutex::new(FpuCtx::new()));
        let mut s = Scb::new();
        s.set_fpu_ctx(fpu_ctx.clone());
        // Firmware clears LSPEN through the FPCCR window; the core sees it.
        write_word(&mut s, 0xE000_EF34, 0x8780_0004);
        assert!(!fpu_ctx.lock().unwrap().lspen);
        assert!(fpu_ctx.lock().unwrap().aspen);
        // Hardware sets LSPACT; the FPCCR read reflects it.
        fpu_ctx.lock().unwrap().lspact = true;
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0x8780_0005);
        // And the FPCCR write survives reset only until `reset()`.
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_EF34), 0xC780_0004);
        assert!(fpu_ctx.lock().unwrap().lspen);
    }

    #[test]
    fn aircr_requires_vectkey_and_stores_prigroup() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
        assert_eq!(s.prig(), 0);
        // A write without VECTKEY is ignored entirely.
        write_word(&mut s, 0xE000_ED0C, 0x0000_0700);
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
        assert_eq!(s.prig(), 0);
        // With VECTKEY the PRIGROUP field is stored; reads show VECTKEYSTAT.
        write_word(&mut s, 0xE000_ED0C, 0x05FA_0700);
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0700);
        assert_eq!(s.prig(), 7);
    }

    #[test]
    fn aircr_sysresetreq_latches_only_under_vectkey() {
        let mut s = Scb::new();
        // No key: the request never latches.
        write_word(&mut s, 0xE000_ED0C, 0x0000_0004);
        assert!(!s.reset_requested());
        // Properly keyed: SYSRESETREQ latches a reset request.
        write_word(&mut s, 0xE000_ED0C, 0x05FA_0004);
        assert!(s.reset_requested());
        // `reset()` clears it, along with the stored PRIGROUP.
        s.reset();
        assert!(!s.reset_requested());
        assert_eq!(read_word(&mut s, 0xE000_ED0C), 0xFA05_0000);
    }

    #[test]
    fn scr_stores_masked_and_resets() {
        let mut s = Scb::new();
        assert_eq!(read_word(&mut s, 0xE000_ED10), 0);
        // SEVONPEND (4) and SLEEPONEXIT (1) are the implemented bits.
        write_word(&mut s, 0xE000_ED10, 0x0000_0012);
        assert_eq!(read_word(&mut s, 0xE000_ED10), 0x0000_0012);
        assert_eq!(s.scr(), 0x0000_0012);
        // The deep-sleep bits (SLEEPDEEP 2, SLEEPDEEPS 3) are RAZ/WI.
        write_word(&mut s, 0xE000_ED10, 0x0000_001E);
        assert_eq!(read_word(&mut s, 0xE000_ED10), 0x0000_0012);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED10), 0);
    }

    #[test]
    fn ccr_resets_with_stkalign_and_masks_writes() {
        let mut s = Scb::new();
        // STKALIGN is set at reset.
        assert_eq!(read_word(&mut s, 0xE000_ED14), 0x0000_0200);
        assert_eq!(s.ccr(), 0x0000_0200);
        // Write DIV_0_TRP only: STKALIGN is cleared, DIV_0_TRP sticks.
        write_word(&mut s, 0xE000_ED14, 0x0000_0010);
        assert_eq!(read_word(&mut s, 0xE000_ED14), 0x0000_0010);
        assert_eq!(s.ccr(), 0x0000_0010);
        // Reserved bits (here bit 7) are dropped on write.
        write_word(&mut s, 0xE000_ED14, 0x0000_0090);
        assert_eq!(read_word(&mut s, 0xE000_ED14), 0x0000_0010);
        s.reset();
        assert_eq!(read_word(&mut s, 0xE000_ED14), 0x0000_0200);
    }
}
