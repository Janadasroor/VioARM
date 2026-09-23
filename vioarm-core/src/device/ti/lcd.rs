//! TI Tiva C LCD raster controller (`LCD0`, `0x4405_0000`).
//!
//! Register map and bit meanings come from the SVD (PL110-compatible
//! raster timing: `PPL`/`HSW`/`HFP`/`HBP` + MSB extends,
//! `LPP`/`VSW`/`VFP`/`VBP`, `LCDEN`, double frame buffers
//! `DMABAFB0/1` + `DMACAFB0/1`, `DONE`/`RRASTRDONE`/`EOF0`/`EOF1`
//! interrupts with `RISSET`/`MISCLR`/`IM`). LIDD command-interface
//! registers latch (no display attached).
//!
//! Modeled: raster enable, frame timing from the `RASTRTIMn` words
//! (pixels per line × lines per frame, 1 tick = 1 pixel — same
//! convention as the in-repo LTDC scanline counter), frame-done +
//! alternating `EOF0`/`EOF1` + one-shot `RRASTRDONE` on enable,
//! `MISCLR` W1C, `IM` gating. Deliberate scope: no pixel transport
//! (frame-buffer contents never move; `BYTESWAP`/`BIGDEND`/burst
//! fields latch), no FIFO-underflow/sync-loss/AC-bias/palette events
//! (no analog/display timing faults in simulation).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// `LCD0` base address.
pub const LCD_BASE: u32 = 0x4405_0000;

const RASTRCTL_OFF: u32 = 0x28;
const RASTRTIM0_OFF: u32 = 0x2C;
const RASTRTIM1_OFF: u32 = 0x30;
const RASTRTIM2_OFF: u32 = 0x34;
const RISSET_OFF: u32 = 0x58;
const MISCLR_OFF: u32 = 0x5C;
const IM_OFF: u32 = 0x60;

/// `RASTRCTL` bits.
const CTL_LCDEN: u32 = 1 << 0;
/// Interrupt bits (`RISSET`/`MISCLR`/`IM` share positions).
const IRQ_DONE: u32 = 1 << 0;
const IRQ_RRASTRDONE: u32 = 1 << 1;
const IRQ_EOF0: u32 = 1 << 8;
const IRQ_EOF1: u32 = 1 << 9;

/// Behavior behind the LCD window: a pixel counter over the programmed
/// frame geometry while `LCDEN` sets.
#[derive(Debug, Default)]
pub struct LcdBehavior {
    /// Ticks remaining in the current frame.
    frame_left: u64,
    /// Which end-of-frame flag the next completion raises.
    eof_toggle: bool,
    /// Latched interrupt flags (`RISSET` image).
    ris: u32,
    /// `RRASTRDONE` fires once per enable (raster reset complete).
    reset_done_armed: bool,
}

impl LcdBehavior {
    /// Frame length in ticks from the timing words (0 = degenerate).
    fn frame_ticks(view: &Registers<&mut [u32]>) -> u64 {
        let t0 = view.get(RASTRTIM0_OFF);
        let t1 = view.get(RASTRTIM1_OFF);
        let t2 = view.get(RASTRTIM2_OFF);
        let ppl = (((t0 >> 3) & 0x01) << 6) | ((t0 >> 4) & 0x3F);
        let hsw = (((t2 >> 27) & 0x0F) << 6) | ((t0 >> 10) & 0x3F);
        let hfp = ((t2 & 0x03) << 8) | ((t0 >> 16) & 0xFF);
        let hbp = (((t2 >> 4) & 0x03) << 8) | ((t0 >> 24) & 0xFF);
        let lpp = (((t2 >> 26) & 0x01) << 10) | (t1 & 0x3FF);
        let vsw = (t1 >> 10) & 0x3F;
        let vfp = (t1 >> 16) & 0xFF;
        let vbp = (t1 >> 24) & 0xFF;
        let htotal = u64::from((ppl + 1) * 16 + hsw + hfp + hbp);
        let vtotal = u64::from(lpp + 1 + vsw + vfp + vbp);
        htotal * vtotal
    }

    fn enabled(view: &Registers<&mut [u32]>) -> bool {
        view.get(RASTRCTL_OFF) & CTL_LCDEN != 0
    }
}

impl Behavior for LcdBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn reset(&mut self) {
        *self = Self::default();
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        u32::from(self.ris & view.get(IM_OFF) != 0)
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !Self::enabled(view) {
            self.frame_left = 0;
            return;
        }
        if self.reset_done_armed {
            self.reset_done_armed = false;
            self.ris |= IRQ_RRASTRDONE;
        }
        if self.frame_left == 0 {
            let ticks = Self::frame_ticks(view);
            if ticks == 0 {
                return; // Degenerate geometry: enabled but no frame.
            }
            self.frame_left = ticks;
        }
        let dt = self.frame_left.min(delta);
        self.frame_left -= dt;
        if self.frame_left == 0 {
            self.ris |= IRQ_DONE;
            self.ris |= if self.eof_toggle { IRQ_EOF1 } else { IRQ_EOF0 };
            self.eof_toggle = !self.eof_toggle;
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            0x58 => self.ris,
            0x5C => self.ris & view.get(IM_OFF),
            _ => value,
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == 0x58 {
            // `RISSET`: W1S test feature — firmware sets raw flags.
            let word = view.merged(offset, val, shift);
            self.ris |= word;
            return self.ris;
        }
        let word = view.merged(offset, val, shift);
        let was_enabled = view.get(RASTRCTL_OFF) & CTL_LCDEN != 0;
        view.set(offset, word);
        if offset == 0x5C {
            self.ris &= !word; // W1C clear.
        }
        if offset == RASTRCTL_OFF {
            if word & CTL_LCDEN != 0 {
                if !was_enabled {
                    self.reset_done_armed = true;
                }
                self.frame_left = 0; // Restart the frame on (re-)enable.
            } else {
                self.frame_left = 0;
            }
        }
        word
    }
}

/// A Tiva C LCD raster controller (frame timing + done IRQs; no pixels).
pub struct Lcd {
    inner: RegisterFile<LcdBehavior>,
}

impl Lcd {
    /// A new LCD controller on `D`'s tables (`"LCD0"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"LCD0"`.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("LCD0").expect("generated tables contain LCD0");
        Self {
            inner: RegisterFile::new(def, LcdBehavior::default()),
        }
    }
}

impl Peripheral for Lcd {
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
}

impl InterruptSource for Lcd {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c129xnczad::Tm4c129xnczad;

    const LCD0_BASE: u32 = 0x4405_0000;

    fn lcd() -> Lcd {
        Lcd::for_device::<Tm4c129xnczad>()
    }

    fn read_word(lcd: &mut Lcd, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(lcd.read(LCD0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(lcd: &mut Lcd, offset: u32, value: u32) {
        for lane in 0..4 {
            lcd.write(
                LCD0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Program a 16-pixel × 2-line frame: PPL=0 (16 px), all sync 0,
    /// LPP=1 (2 lines) → 32 ticks/frame.
    fn tiny_frame(lcd: &mut Lcd) {
        write_word(lcd, RASTRTIM0_OFF, 0x0000_0000); // PPL=0, HSW/HFP/HBP=0
        write_word(lcd, RASTRTIM1_OFF, 0x0000_0001); // LPP=1, syncs 0
        write_word(lcd, RASTRTIM2_OFF, 0x0000_0000);
        write_word(lcd, 0x44, 0xDEAD_BEEF); // DMABAFB0 latches
        write_word(lcd, RASTRCTL_OFF, CTL_LCDEN);
    }

    /// Enable runs one 32-tick frame: `DONE` + `EOF0`, then `EOF1`.
    #[test]
    fn frame_done_and_eof_alternate() {
        let mut lcd = lcd();
        tiny_frame(&mut lcd);
        lcd.tick(1);
        assert_eq!(
            read_word(&mut lcd, RISSET_OFF) & IRQ_RRASTRDONE,
            IRQ_RRASTRDONE
        );
        lcd.tick(30);
        assert_eq!(read_word(&mut lcd, RISSET_OFF) & IRQ_DONE, 0);
        lcd.tick(1);
        let ris = read_word(&mut lcd, RISSET_OFF);
        assert_eq!(ris & IRQ_DONE, IRQ_DONE);
        assert_eq!(ris & IRQ_EOF0, IRQ_EOF0);
        lcd.tick(32);
        assert_eq!(read_word(&mut lcd, RISSET_OFF) & IRQ_EOF1, IRQ_EOF1);
    }

    /// `MISCLR` W1C clears; `IM` gates the line.
    #[test]
    fn w1c_and_gating() {
        let mut lcd = lcd();
        write_word(&mut lcd, IM_OFF, IRQ_DONE);
        tiny_frame(&mut lcd);
        lcd.tick(32);
        assert_eq!(lcd.pending(), 1);
        write_word(&mut lcd, MISCLR_OFF, IRQ_DONE | IRQ_EOF0);
        assert_eq!(read_word(&mut lcd, RISSET_OFF) & (IRQ_DONE | IRQ_EOF0), 0);
        assert_eq!(lcd.pending(), 0);
    }

    /// Disabled raster holds; all-zero timing still yields the minimal
    /// 16×1 frame (PL110 convention: `PPL=0` → 16 clocks).
    #[test]
    fn disabled_holds() {
        let mut lcd = lcd();
        lcd.tick(1000);
        assert_eq!(read_word(&mut lcd, RISSET_OFF), 0);
        write_word(&mut lcd, RASTRCTL_OFF, CTL_LCDEN); // All-zero timing.
        lcd.tick(15);
        assert_eq!(read_word(&mut lcd, RISSET_OFF) & IRQ_DONE, 0);
        lcd.tick(1);
        assert_eq!(read_word(&mut lcd, RISSET_OFF) & IRQ_DONE, IRQ_DONE);
        assert_eq!(
            read_word(&mut lcd, RISSET_OFF) & IRQ_RRASTRDONE,
            IRQ_RRASTRDONE
        );
    }
}
