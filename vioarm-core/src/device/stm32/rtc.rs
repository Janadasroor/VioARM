//! STM32 RTC (`RTC`, `0x4000_2800`) — accurate prescaler/counter/alarm.
//! `PRLH`/`PRLL` (20-bit prescaler, reset `0x8000`), `DIVH`/`DIVL` (read-only
//! divider), `CNTH`/`CNTL` (32-bit counter), `ALRH`/`ALRL` (alarm), `CRL`
//! (`SECF`/`ALRF`/`OWF`/`RSF`/`CNF`/`RTOFF`) and `CRH` (`SECIE`/`ALRIE`/`OWIE`).
//! `CNF` gates `CNT`/`PRL`/`ALR` writes; `RTOFF` is `1` when not in config.
//! `DIV` decrements each `tick()` (1 RTCCLK per `delta`); on underflow it
//! reloads from `PRL` and increments `CNT`, setting `SECF`; on `CNT==ALR`
//! sets `ALRF`; on wrap sets `OWF`. Flags `SECF`/`ALRF`/`OWF` raise `RTC`
//! (IRQ 3) and `RTCAlarm` (IRQ 41) via `CRH` enables.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CRH: u32 = 0x00;
const CRL: u32 = 0x04;
const PRLH: u32 = 0x08;
const PRLL: u32 = 0x0C;
const DIVH: u32 = 0x10;
const DIVL: u32 = 0x14;
const CNTH: u32 = 0x18;
const CNTL: u32 = 0x1C;
const ALRH: u32 = 0x20;
const ALRL: u32 = 0x24;

const CRL_SECF: u32 = 1 << 0;
const CRL_ALRF: u32 = 1 << 1;
const CRL_OWF: u32 = 1 << 2;
const CRL_RSF: u32 = 1 << 3;
const CRL_CNF: u32 = 1 << 4;
const CRL_RTOFF: u32 = 1 << 5;

const CRH_SECIE: u32 = 1 << 0;
const CRH_ALRIE: u32 = 1 << 1;
const CRH_OWIE: u32 = 1 << 2;

#[derive(Debug)]
struct RtcBehavior {
    /// Divider countdown, mirrors `DIVH`/`DIVL`.
    div: u32,
}

impl RtcBehavior {
    const fn new() -> Self {
        Self { div: 0x8000 }
    }
    fn prl<W>(view: &Registers<W>) -> u32
    where
        W: std::ops::Deref<Target = [u32]>,
    {
        (view.get(PRLH) & 0xF_u32) << 16 | (view.get(PRLL) & 0xFFFF_u32)
    }
    fn cnt<W>(view: &Registers<W>) -> u32
    where
        W: std::ops::Deref<Target = [u32]>,
    {
        (view.get(CNTH) & 0xFFFF_u32) << 16 | (view.get(CNTL) & 0xFFFF_u32)
    }
    fn set_cnt(view: &mut Registers<&mut [u32]>, v: u32) {
        view.set(CNTH, (v >> 16) & 0xFFFF_u32);
        view.set(CNTL, v & 0xFFFF_u32);
    }
    fn alr<W>(view: &Registers<W>) -> u32
    where
        W: std::ops::Deref<Target = [u32]>,
    {
        (view.get(ALRH) & 0xFFFF_u32) << 16 | (view.get(ALRL) & 0xFFFF_u32)
    }
}

impl Behavior for RtcBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let cur = view.get(offset);
        let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
        match offset {
            CRL => {
                // `CNF` and flag clears are W0C? Actually `SECF`/`ALRF`/`OWF`
                // are cleared by writing 0, `RSF` by writing 0, `CNF` r/w.
                // `RTOFF` is read-only, always 1 outside config.
                let mut crl = view.get(CRL);
                // CNF is bit 4, r/w.
                if (word & CRL_CNF) != (crl & CRL_CNF) {
                    if word & CRL_CNF != 0 {
                        // Enter config: RTOFF -> 0 until exit.
                        crl = (crl & !CRL_RTOFF) | CRL_CNF;
                    } else {
                        // Exit config: set RSF, RTOFF=1, sync div.
                        crl = (crl & !CRL_CNF) | CRL_RSF | CRL_RTOFF;
                        let prl = Self::prl(view);
                        self.div = prl;
                        view.set(DIVH, (self.div >> 16) & 0xF_u32);
                        view.set(DIVL, self.div & 0xFFFF_u32);
                    }
                    // Preserve SECF/ALRF/OWF as they were (cleared via 0 below).
                    let keep = crl & (CRL_CNF | CRL_RTOFF | CRL_RSF);
                    let flags = view.get(CRL) & (CRL_SECF | CRL_ALRF | CRL_OWF);
                    // Apply flag clears: writing 0 clears, 1 keeps.
                    let mut new_flags = flags;
                    if word & CRL_SECF == 0 {
                        new_flags &= !CRL_SECF;
                    }
                    if word & CRL_ALRF == 0 {
                        new_flags &= !CRL_ALRF;
                    }
                    if word & CRL_OWF == 0 {
                        new_flags &= !CRL_OWF;
                    }
                    if word & CRL_RSF == 0 {
                        new_flags &= !CRL_RSF;
                        crl &= !CRL_RSF;
                    }
                    return (crl & !(CRL_SECF | CRL_ALRF | CRL_OWF)) | new_flags | keep & CRL_RTOFF;
                }
                // Not CNF toggle: handle flag clears.
                let mut new = cur;
                // Flags: writing 0 clears.
                if word & CRL_SECF == 0 {
                    new &= !CRL_SECF;
                }
                if word & CRL_ALRF == 0 {
                    new &= !CRL_ALRF;
                }
                if word & CRL_OWF == 0 {
                    new &= !CRL_OWF;
                }
                if word & CRL_RSF == 0 {
                    new &= !CRL_RSF;
                }
                // CNF and RTOFF stay as before (except CNF toggle handled above).
                (new & !(CRL_CNF | CRL_RTOFF)) | (cur & (CRL_CNF | CRL_RTOFF))
            }
            CRH => word & (CRH_SECIE | CRH_ALRIE | CRH_OWIE),
            PRLH | PRLL | CNTH | CNTL | ALRH | ALRL => {
                // Only writable when CNF=1.
                if view.get(CRL) & CRL_CNF == 0 {
                    return cur;
                }
                let masked = match offset {
                    PRLH => word & 0xF_u32,
                    PRLL | CNTH | CNTL | ALRH | ALRL => word & 0xFFFF_u32,
                    _ => word,
                };
                if offset == PRLH || offset == PRLL {
                    // Keep div in sync when PRL changes in config (reload on exit).
                    return masked;
                }
                masked
            }
            DIVH | DIVL => cur, // read-only
            _ => word,
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if delta == 0 {
            return;
        }
        // No tick while in config.
        if view.get(CRL) & CRL_CNF != 0 {
            return;
        }
        for _ in 0..delta {
            if self.div == 0 {
                let prl = Self::prl(view);
                self.div = prl;
                view.set(DIVH, (self.div >> 16) & 0xF_u32);
                view.set(DIVL, self.div & 0xFFFF_u32);
                let cnt = Self::cnt(view);
                let (next, overflow) = cnt.overflowing_add(1);
                Self::set_cnt(view, next);
                // SECF each increment.
                let crl = view.get(CRL) | CRL_SECF;
                view.set(CRL, crl);
                if overflow {
                    view.set(CRL, view.get(CRL) | CRL_OWF);
                }
                if next == Self::alr(view) {
                    view.set(CRL, view.get(CRL) | CRL_ALRF);
                }
            } else {
                self.div -= 1;
                view.set(DIVH, (self.div >> 16) & 0xF_u32);
                view.set(DIVL, self.div & 0xFFFF_u32);
                if self.div == 0 {
                    // Will be handled next tick as reload.
                }
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let crl = view.get(CRL);
        let crh = view.get(CRH);
        let mut lines = 0;
        if crl & CRL_SECF != 0 && crh & CRH_SECIE != 0 {
            lines |= 1 << 0; // RTC global
        }
        if crl & CRL_OWF != 0 && crh & CRH_OWIE != 0 {
            lines |= 1 << 0;
        }
        if crl & CRL_ALRF != 0 && crh & CRH_ALRIE != 0 {
            lines |= 1 << 1; // RTCAlarm
        }
        lines
    }

    fn lines(&self) -> u32 {
        2
    }

    fn reset(&mut self) {
        self.div = 0x8000;
    }
}

#[derive(Debug)]
pub struct Rtc {
    inner: RegisterFile<RtcBehavior>,
}
impl Rtc {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("RTC").expect("generated tables contain RTC");
        Self {
            inner: RegisterFile::new(def, RtcBehavior::new()),
        }
    }
}
impl Peripheral for Rtc {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }
    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        self.inner.read(addr)
    }
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        self.inner.write(addr, val)
    }
    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
    fn reset(&mut self) {
        self.inner.reset();
    }
    fn name(&self) -> &'static str {
        "rtc"
    }
}
impl InterruptSource for Rtc {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
    fn lines(&self) -> u32 {
        self.inner.lines()
    }
    fn exception_for_line(&self, line: u32) -> Option<u8> {
        self.inner.exception_for_line(line)
    }
}

#[cfg(test)]
mod tests {
    use super::Rtc;
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::{InterruptSource, Peripheral};
    const BASE: u32 = 0x4000_2800;
    fn write_word(r: &mut Rtc, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            r.write(BASE + off + i as u32, *b).unwrap();
        }
    }
    fn read_word(r: &mut Rtc, off: u32) -> u32 {
        let mut b = [0u8; 4];
        for (i, v) in b.iter_mut().enumerate() {
            *v = r.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(b)
    }
    #[test]
    fn rtc_prescaler_ticks_counter_and_alarm() {
        let mut rtc = Rtc::new_for::<Stm32f103>();
        // Enter config, set PRL=1 (div 1), CNT=0, ALR=2.
        write_word(&mut rtc, 0x04, 0x20 | 0x10); // CRL CNF=1, keep RTOFF? 0x20 is RTOFF reset, CNF 0x10
        write_word(&mut rtc, 0x08, 0x00); // PRLH
        write_word(&mut rtc, 0x0C, 0x0001); // PRLL=1
        write_word(&mut rtc, 0x18, 0x0000); // CNTH
        write_word(&mut rtc, 0x1C, 0x0000); // CNTL
        write_word(&mut rtc, 0x20, 0x0000); // ALRH
        write_word(&mut rtc, 0x24, 0x0002); // ALRL=2
        write_word(&mut rtc, 0x00, 0x02); // CRH ALRIE
        write_word(&mut rtc, 0x04, 0x20); // CRL CNF=0, exit config -> RSF + RTOFF
                                          // Tick: div 1 -> CNT increments every 2 ticks (1+ reload).
        for _ in 0..5 {
            rtc.tick(1);
        }
        let cnt = {
            let hi = read_word(&mut rtc, 0x18);
            let lo = read_word(&mut rtc, 0x1C);
            (hi << 16) | (lo & 0xFFFF)
        };
        assert!(cnt >= 2, "CNT advanced to {cnt}");
        let crl = read_word(&mut rtc, 0x04);
        assert_ne!(crl & 0x02, 0, "ALRF set");
        assert_ne!(rtc.pending() & (1 << 1), 0, "RTCAlarm pending");
        // Clear ALRF by writing 0.
        let crl2 = read_word(&mut rtc, 0x04);
        write_word(&mut rtc, 0x04, crl2 & !0x02);
        assert_eq!(read_word(&mut rtc, 0x04) & 0x02, 0);
        assert_eq!(rtc.pending() & (1 << 1), 0);
    }
    #[test]
    fn rtc_registers_do_not_fault() {
        let mut r = Rtc::new_for::<Stm32f103>();
        write_word(&mut r, 0x00, 0x07);
        assert_eq!(read_word(&mut r, 0x00) & 0x07, 0x07);
        assert!(matches!(
            r.read(BASE + 0x500),
            Err(crate::error::BusError::Reserved)
        ));
    }
}
