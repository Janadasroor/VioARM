//! L4 LCD glass controller (`LCD@0x40002400`).
//!
//! `CR` (duty/bias/voltage/`LCDEN`) and `FCR` (blink/contrast/dead-time,
//! `UDDIE`/`SOFIE`) latch from the tables; the eight `RAM_COMx` segment
//! words latch segment data. The live handshake is what firmware polls:
//! `SR.ENS`/`RDY` follow `LCDEN`; writing display RAM while enabled
//! raises `SR.UDD` (update-display-done, cleared by `CLR.UDDC`), with
//! `SR.UDR` if a second write lands while `UDD` is still pending; an
//! `FCR` write raises `SR.FCRSF` after a 2-tick register sync; `SR.SOF`
//! ticks once per [`SOF_TICKS`] while enabled (frame rate idealized).
//! The single line follows `UDDIE`/`SOFIE`. Blink patterns and glass
//! timing stay unmodeled (documented).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CR: u32 = 0x00;
const FCR: u32 = 0x04;
const SR: u32 = 0x08;
const CLR: u32 = 0x0C;

const LCDEN: u32 = 1 << 0;
const SR_ENS: u32 = 1 << 0;
const SR_SOF: u32 = 1 << 1;
const SR_UDR: u32 = 1 << 2;
const SR_UDD: u32 = 1 << 3;
const SR_RDY: u32 = 1 << 4;
const SR_FCRSF: u32 = 1 << 5;
const CLR_UDDC: u32 = 1 << 3;
const CLR_SOFC: u32 = 1 << 1;
const FCR_UDDIE: u32 = 1 << 3;
const FCR_SOFIE: u32 = 1 << 1;

/// Ticks per start-of-frame pulse while enabled.
const SOF_TICKS: u64 = 64;
/// Register-sync delay for `FCRSF` after an `FCR` write.
const SYNC_TICKS: u64 = 2;

#[derive(Clone, Debug, Default)]
struct LcdBehavior {
    sync_at: Option<u64>,
    sof_at: u64,
}

impl LcdBehavior {
    fn set_sr(view: &mut Registers<&mut [u32]>, bits: u32) {
        view.set(SR, view.get(SR) | bits);
    }

    fn enabled(view: &mut Registers<&mut [u32]>) -> bool {
        view.get(CR) & LCDEN != 0
    }

    fn is_ram(offset: u32) -> bool {
        (0x14..0x50).contains(&offset) && (offset - 0x14) % 8 == 0
    }
}

impl Behavior for LcdBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            CLR => {
                // Write-only flag clear.
                let new = view.merged(CLR, val, shift);
                if new & CLR_UDDC != 0 {
                    view.set(SR, view.get(SR) & !SR_UDD);
                }
                if new & CLR_SOFC != 0 {
                    view.set(SR, view.get(SR) & !SR_SOF);
                }
                new
            }
            FCR => {
                let new = view.merged(FCR, val, shift);
                view.set(SR, view.get(SR) & !SR_FCRSF);
                self.sync_at = Some(SYNC_TICKS);
                new
            }
            _ if Self::is_ram(offset) => {
                let new = view.merged(offset, val, shift);
                if Self::enabled(view) {
                    if view.get(SR) & SR_UDD != 0 {
                        Self::set_sr(view, SR_UDR);
                    }
                    Self::set_sr(view, SR_UDD);
                }
                new
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == SR {
            let en = Self::enabled(view);
            let mut v = value;
            if en {
                v |= SR_ENS | SR_RDY;
            } else {
                v &= !(SR_ENS | SR_RDY);
            }
            return v;
        }
        value
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if let Some(left) = self.sync_at {
            let left = left.saturating_sub(delta);
            self.sync_at = Some(left);
            if left == 0 {
                self.sync_at = None;
                Self::set_sr(view, SR_FCRSF);
            }
        }
        if Self::enabled(view) {
            self.sof_at += delta;
            if self.sof_at >= SOF_TICKS {
                self.sof_at = 0;
                Self::set_sr(view, SR_SOF);
            }
        } else {
            self.sof_at = 0;
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(SR);
        let fcr = view.get(FCR);
        u32::from(
            sr & SR_UDD != 0 && fcr & FCR_UDDIE != 0 || sr & SR_SOF != 0 && fcr & FCR_SOFIE != 0,
        )
    }

    fn reset(&mut self) {
        self.sync_at = None;
        self.sof_at = 0;
    }
}

/// L4 LCD glass controller.
#[derive(Debug)]
pub struct Lcd {
    inner: RegisterFile<LcdBehavior>,
}

impl Lcd {
    /// A new `LCD` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `LCD`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("LCD").expect("generated tables contain LCD");
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

    fn name(&self) -> &'static str {
        "lcd"
    }
}

impl InterruptSource for Lcd {
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
    use crate::device::svd::stm32l476::Stm32l476;
    use crate::device::{InterruptSource, Peripheral};

    const BASE: u32 = 0x4000_2400;
    const CR_O: u32 = 0x00;
    const FCR_O: u32 = 0x04;
    const SR_O: u32 = 0x08;
    const CLR_O: u32 = 0x0C;
    const RAM0_O: u32 = 0x14;

    #[allow(clippy::cast_possible_truncation)]
    fn w(l: &mut Lcd, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            l.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(l: &mut Lcd, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = l.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn lcd() -> Lcd {
        let mut l = Lcd::new_for::<Stm32l476>();
        l.reset();
        l
    }

    #[test]
    fn table_matches_silicon_layout() {
        let def = Stm32l476::peripheral("LCD").unwrap();
        assert_eq!(def.base, BASE, "LCD base");
        for (name, offset) in [
            ("LCD_CR", 0x00),
            ("LCD_FCR", 0x04),
            ("LCD_SR", 0x08),
            ("LCD_CLR", 0x0C),
        ] {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("LCD:{name} present"));
            assert_eq!(r.offset, offset, "LCD:{name} offset");
        }
    }

    #[test]
    fn enable_and_ram_handshake() {
        let mut l = lcd();
        assert_eq!(r(&mut l, SR_O) & SR_ENS, 0, "off at reset");
        w(&mut l, CR_O, LCDEN);
        assert_ne!(r(&mut l, SR_O) & (SR_ENS | SR_RDY), 0, "ENS+RDY follow");
        w(&mut l, RAM0_O, 0xDEAD_BEEF);
        assert_eq!(r(&mut l, RAM0_O), 0xDEAD_BEEF, "segments latch");
        assert_ne!(r(&mut l, SR_O) & SR_UDD, 0, "UDD on RAM write");
        w(&mut l, RAM0_O, 0x1234_5678);
        assert_ne!(r(&mut l, SR_O) & SR_UDR, 0, "UDR on overwrite");
        w(&mut l, CLR_O, CLR_UDDC);
        assert_eq!(r(&mut l, SR_O) & SR_UDD, 0, "CLR clears UDD");
    }

    #[test]
    fn ram_write_while_off_is_quiet() {
        let mut l = lcd();
        w(&mut l, RAM0_O, 0xFFFF_FFFF);
        assert_eq!(r(&mut l, SR_O) & SR_UDD, 0, "no UDD while off");
    }

    #[test]
    fn fcrsf_syncs_and_sof_ticks() {
        let mut l = lcd();
        w(&mut l, CR_O, LCDEN);
        w(&mut l, FCR_O, FCR_UDDIE | FCR_SOFIE);
        assert_eq!(r(&mut l, SR_O) & SR_FCRSF, 0, "sync pending");
        l.tick(2);
        assert_ne!(r(&mut l, SR_O) & SR_FCRSF, 0, "FCRSF after sync");
        l.tick(SOF_TICKS);
        assert_ne!(r(&mut l, SR_O) & SR_SOF, 0, "SOF pulses");
        assert_eq!(l.pending(), 1, "UDD/SOF line raises");
        w(&mut l, CLR_O, CLR_UDDC | CLR_SOFC);
        assert_eq!(l.pending(), 0, "line drops");
    }
}
