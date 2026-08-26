//! STM32 RTC (`RTC`, `0x4000_2800`).
//!
//! Two generations share this module, selected by the generated tables:
//!
//! **F1 (classic)** — `PRLH`/`PRLL` (20-bit prescaler), `DIVH`/`DIVL`,
//! `CNTH`/`CNTL`, `ALRH`/`ALRL`, `CRL` (`SECF`/`ALRF`/`OWF`/`RSF`/`CNF`/
//! `RTOFF`) and `CRH`. `CNF` gates `CNT`/`PRL`/`ALR` writes; the divider
//! decrements per tick, reloads from `PRL`, increments `CNT` (`SECF`),
//! matches `ALR` (`ALRF`) and wraps (`OWF`). Flags raise `RTC` (IRQ 3) and
//! `RTCAlarm` (IRQ 41) through `CRH`.
//!
//! **New-gen (F0/L4/G4/H7)** — table declares `ISR`: write protection via
//! `WPR` (0xCA 0x53 unlocks CR/ISR/PRER/ALRMAR), `INIT` enters init
//! (`INITF` while synchronizing), a free-running seconds calendar latched
//! into `TR`/`DR`, and alarm A: `ALRMAR` seconds field compared against the
//! running time unless its mask bit is set; match latches `ISR.ALRAF` and
//! — with `CR.ALRAIE` (ISR bit 12) — asserts the alarm line.

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

/// New-gen register offsets (tables that declare `ISR`).
const NG_TR: u32 = 0x00;
const NG_DR: u32 = 0x04;
const NG_CR: u32 = 0x08;
const NG_ISR: u32 = 0x0C;
const NG_PRER: u32 = 0x10;
const NG_ALRMAR: u32 = 0x1C;
const NG_WPR: u32 = 0x24;
/// `TR` reserved high bits kept clear.
const NG_TR_RESET_BASE: u32 = 0;
/// `TR` reset base (reserved bits read as written; keep zero).
#[allow(dead_code)]

/// `ISR` bit positions.
const NG_ISR_WUTWF: u32 = 1 << 2;
const NG_ISR_INITS: u32 = 1 << 4;
const NG_ISR_RSF: u32 = 1 << 5;
const NG_ISR_INITF: u32 = 1 << 6;
const NG_ISR_INIT: u32 = 1 << 7;
const NG_ISR_ALRAF: u32 = 1 << 8;
/// `CR.ALRAIE` lives in ISR bits [15:8] on new-gen.
const NG_CR_ALRAIE: u32 = 1 << 12;

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
    /// Divider countdown, mirrors `DIVH`/`DIVL` (F1) — or the free-running
    /// seconds calendar (new-gen).
    div: u32,
    /// New-generation tables (`ISR` present): WPR gate + calendar mode.
    new_gen: bool,
    /// `WPR` unlock state for new-gen (0xCA then 0x53 seen as words).
    wpr_step: u8,
    /// The `WPR` word being assembled from byte-lane writes.
    wpr_word: u32,
}

impl RtcBehavior {
    const fn new() -> Self {
        Self {
            div: 0x8000,
            new_gen: false,
            wpr_step: 0,
            wpr_word: 0,
        }
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

    /// New-gen `WPR` gate: the unlock sequence 0xCA then 0x53 (as full word
    /// writes) enables writes to `CR`/`ISR`/`PRER`/`ALRMAR`; any other value
    /// re-locks until reset.
    fn ng_unlocked(&self) -> bool {
        self.wpr_step == 2
    }

    #[allow(clippy::similar_names)]
    fn ng_on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == NG_WPR {
            // Assemble the key from byte lanes (a word store arrives as four
            // writes) and evaluate the two-key sequence per completed word.
            self.wpr_word = (self.wpr_word & !(0xFF << shift)) | (u32::from(val) << shift);
            if shift == 24 {
                let w = self.wpr_word;
                match (self.wpr_step, w) {
                    (0, 0xCA) => self.wpr_step = 1,
                    (1, 0x53) => self.wpr_step = 2,
                    _ => self.wpr_step = 0,
                }
                self.wpr_word = 0;
            }
            return 0;
        }
        // Protected registers ignore writes while locked. `ISR` is special:
        // INIT is r/w, ALRAF/RSF are write-0-to-clear.
        let protected =
            matches!(offset, NG_CR | NG_PRER | NG_ALRMAR | NG_TR | NG_DR) && !self.ng_unlocked();
        if protected {
            return view.get(offset);
        }
        if offset == NG_ISR {
            let cur = view.get(NG_ISR);
            let bits = view.merged(NG_ISR, val, shift);
            // INIT is plain r/w; the status flags are write-0-to-clear.
            let mut next = bits;
            for flag in [NG_ISR_RSF, NG_ISR_INITF, NG_ISR_ALRAF] {
                if bits & flag == 0 {
                    next &= !flag;
                }
            }
            // Entering init: INITF set immediately in this model; leaving
            // clears it and marks the calendar initialized.
            if cur & NG_ISR_INIT == 0 && next & NG_ISR_INIT != 0 {
                next |= NG_ISR_INITF;
                next &= !NG_ISR_INITS;
            } else if cur & NG_ISR_INIT != 0 && next & NG_ISR_INIT == 0 {
                next &= !NG_ISR_INITF;
                next |= NG_ISR_INITS | NG_ISR_RSF;
            }
            return next;
        }
        view.merged(offset, val, shift)
    }

    /// Advance the seconds calendar by `delta` ticks (1 tick = 1 s here) and
    /// evaluate alarm A unless in init or seconds-masked.
    fn ng_tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if view.get(NG_ISR) & NG_ISR_INIT != 0 || delta == 0 {
            return;
        }
        for _ in 0..delta {
            let s = Self::ng_seconds(view);
            let next = (s + 1) % 60;
            Self::ng_set_seconds(view, next);
            let alrmar = view.get(NG_ALRMAR);
            if alrmar & (1 << 7) != 0 {
                continue; // MSK1: seconds masked
            }
            let alarm_secs = ((alrmar >> 4) & 0x7) * 10 + (alrmar & 0xF);
            if next == alarm_secs {
                view.set(NG_ISR, view.get(NG_ISR) | NG_ISR_ALRAF);
            }
        }
    }

    /// The running seconds count latched into `TR.SU[5:0]`/`ST[2:0]`.
    fn ng_seconds(view: &Registers<&mut [u32]>) -> u32 {
        let tr = view.get(NG_TR);
        ((tr >> 4) & 0x7) * 10 + (tr & 0xF)
    }
    fn ng_set_seconds(view: &mut Registers<&mut [u32]>, s: u32) {
        let su = s % 10;
        let st = (s / 10) % 6;
        view.set(NG_TR, NG_TR_RESET_BASE | (st << 4) | su);
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
        if self.new_gen {
            return self.ng_on_write(view, offset, val, shift);
        }
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
        if self.new_gen {
            self.ng_tick(view, delta);
            return;
        }
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
        if self.new_gen {
            // Alarm A line: ISR.ALRAF gated by the CR-bank ALRAIE (ISR bit 12).
            let isr = view.get(NG_ISR);
            return u32::from(isr & NG_ISR_ALRAF != 0 && isr & NG_CR_ALRAIE != 0);
        }
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
        self.wpr_step = 0;
        self.wpr_word = 0;
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
        let mut b = RtcBehavior::new();
        // New-generation tables declare `ISR`; F1 declares `CRL`.
        b.new_gen = def.registers.iter().any(|r| r.name == "ISR");
        Self {
            inner: RegisterFile::new(def, b),
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
    /// New-generation RTC: WPR-protected writes, INIT/INITF, and alarm A
    /// matching the running seconds with `ALRAF`/`ALRAIE`.
    #[test]
    fn newgen_rtc_wpr_init_and_alarm_a() {
        use crate::device::svd::stm32l476::Stm32l476;
        const B: u32 = 0x4000_2800;
        let mut r = Rtc::new_for::<Stm32l476>();
        // Locked: CR/ISR/PRER/ALRMAR ignore writes.
        write_word(&mut r, 0x08, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut r, 0x08), 0, "CR locked");
        // Unlock WPR.
        write_word(&mut r, 0x24, 0xCA);
        write_word(&mut r, 0x24, 0x53);
        write_word(&mut r, 0x08, 0xFFFF_FFFF); // now latches
        assert_ne!(read_word(&mut r, 0x08), 0);
        // Enter init then leave it: INITS marks the calendar initialized.
        write_word(&mut r, 0x0C, 1 << 7); // ISR byte1 INIT
        let isr = read_word(&mut r, 0x0C);
        assert_ne!(isr & (1 << 6), 0, "INITF while in init");
        write_word(&mut r, 0x0C, 0); // leave init (byte1=0)
        let isr = read_word(&mut r, 0x0C);
        assert_eq!(isr & (1 << 7), 0);
        assert_ne!(isr & (1 << 4), 0, "INITS after init exit");
        // Program alarm A for seconds = 03, enable ALRAIE.
        write_word(&mut r, 0x1C, 0x03); // SU=3, ST=0, MSK1 clear
        write_word(&mut r, 0x0C, 1 << 12); // ALRAIE (byte lane 1)
                                           // Run the calendar to :03 (the match lands on the third second).
        for _ in 0..3 {
            r.tick(1);
        }
        assert_eq!(read_word(&mut r, 0x00) & 0xF, 3, "TR.SU == 3");
        assert_ne!(read_word(&mut r, 0x0C) & (1 << 8), 0, "ALRAF");
        assert_eq!(r.pending(), 1, "alarm line via ALRAIE");
        // Clear ALRAF (write zero into its lane).
        write_word(&mut r, 0x0D, 0); // byte1 of ISR clears nothing here
        write_word(&mut r, 0x0C, !(1 << 8));
        assert_eq!(read_word(&mut r, 0x0C) & (1 << 8), 0);
        assert_eq!(r.pending(), 0);
        // MSK1 masks the compare entirely.
        write_word(&mut r, 0x1C, 0x80 | 0x30);
        r.tick(60);
        assert_eq!(r.pending(), 0, "masked alarm never fires");
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
