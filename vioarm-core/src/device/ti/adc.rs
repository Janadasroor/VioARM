//! TI Tiva C analog-to-digital converters (`ADC0`-`ADC1`).
//!
//! Processor-triggered sample sequencers follow the TM4C123GH6PM
//! datasheet (SPMS376E §13): `ACTSS` enables, `PSSI` initiates enabled
//! sequencers, each sequence runs `SSMUXn` channels through `SSCTLn`
//! nibbles (`TS`/`IE`/`END`; `D` differential ignored, single-ended read)
//! into `SSFIFO` (depths 8/4/4/1), raising `INRn` when an `IE` sample
//! completes, gated by `IM` with `ISC` W1C clear. `SSFSTAT` tracks
//! `FULL`/`EMPTY` with head/tail pointers (p862), overflow sets `OSTAT`,
//! empty reads set `USTAT` (both RW1C).
//! `SAC` averaging folds `2^AVG` identical driven samples (exact for
//! constant inputs, the only kind simulation drives; documented).
//!
//! Channel codes come from the host/test via [`Adc::drive_channel`]
//! (12-bit, default 0); the temperature sensor (`TS=1`) reads a driven
//! temp code (default 0) — no silicon calibration constants are modeled.
//! Conversions complete synchronously on the `PSSI` write (no sample
//! timing; documented). Only processor triggers (`EMUX=0x0`, the reset)
//! initiate; simultaneous triggers run in `ADCSSPRI` priority order
//! (p845). Timer/PWM/GPIO triggers, comparator paths (`SSOP`/`SSDC`/`
//! DCCTL`), `GSYNC`/`SYNCWAIT`, and `ADCCTL`/`SPC` effects stay
//! latched (documented). Each sequencer raises its own SVD line
//! (`ADCnSS0`-`ADCnSS3`).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const ACTSS_OFF: u32 = 0x000;
const RIS_OFF: u32 = 0x004;
const IM_OFF: u32 = 0x008;
const ISC_OFF: u32 = 0x00C;
const OSTAT_OFF: u32 = 0x010;
const EMUX_OFF: u32 = 0x014;
const USTAT_OFF: u32 = 0x018;
const SSPRI_OFF: u32 = 0x020;
const PSSI_OFF: u32 = 0x028;
const SAC_OFF: u32 = 0x030;
/// Per-sequencer block stride (`SSMUXn` at `+0x00` through `SSDCn`).
const SEQ_STRIDE: u32 = 0x020;
const SSMUX_OFF: u32 = 0x040;
const SSCTL_OFF: u32 = 0x044;
const SSFIFO_OFF: u32 = 0x048;
const SSFSTAT_OFF: u32 = 0x04C;
/// `SSFSTAT` bits: `FULL` 12, `EMPTY` 8, `HPTR` 7:4, `TPTR` 3:0.
const FSTAT_FULL: u32 = 1 << 12;
const FSTAT_EMPTY: u32 = 1 << 8;
/// `SSCTL` nibble bits: `TS` 3, `IE` 2, `END` 1, `D` 0.
const SN_TS: u32 = 1 << 3;
const SN_IE: u32 = 1 << 2;
const SN_END: u32 = 1 << 1;
/// FIFO depths per sequencer (SS0 8, SS1/SS2 4, SS3 1).
const SEQ_DEPTH: &[usize] = &[8, 4, 4, 1];
/// Sample slots per sequencer (SS0 8, SS1-3 4/4/1… SS3 single).
const SEQ_SLOTS: &[usize] = &[8, 4, 4, 1];
/// Channels: AIN0-11.
const N_CHANNELS: usize = 12;

/// Nonzero datasheet resets stamped over the SVD zeros.
const RESETS: &[(u32, u32)] = &[
    (SSPRI_OFF, 0x0000_3210),
    (SSFSTAT_OFF, 0x0000_0100),
    (SSFSTAT_OFF + SEQ_STRIDE, 0x0000_0100),
    (SSFSTAT_OFF + 2 * SEQ_STRIDE, 0x0000_0100),
    (SSFSTAT_OFF + 3 * SEQ_STRIDE, 0x0000_0100),
];

/// Behavior behind one ADC window (processor-triggered sequencers).
#[derive(Debug)]
pub struct AdcBehavior {
    fifo: [Vec<u16>; 4],
    /// FIFO head (next-write) and tail (next-read) indices per sequencer
    /// (`SSFSTAT.HPTR/TPTR`, p862): advance modulo depth on push/pop, so
    /// they coincide when empty and diverge on wrap/pop.
    head: [u8; 4],
    tail: [u8; 4],
    popped: u16,
    /// Sticky raw interrupt flags per sequencer.
    inr: [bool; 4],
    /// Driven 12-bit channel codes (host/test stimulus).
    channels: [u16; N_CHANNELS],
    /// Driven temperature-sensor code (`TS=1` samples).
    temp_code: u16,
    /// Last popped FIFO entry (upper `SSFIFO` lanes read its high byte).
    /// Cached masked-interrupt bitmask (`InterruptSource::pending`).
    pending: u32,
}

impl Default for AdcBehavior {
    fn default() -> Self {
        Self {
            fifo: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            head: [0; 4],
            tail: [0; 4],
            inr: [false; 4],
            channels: [0; N_CHANNELS],
            temp_code: 0,
            popped: 0,
            pending: 0,
        }
    }
}

impl AdcBehavior {
    fn mux(&self, view: &Registers<&mut [u32]>, seq: usize) -> u32 {
        view.get(SSMUX_OFF + seq as u32 * SEQ_STRIDE)
    }

    fn ctl(&self, view: &Registers<&mut [u32]>, seq: usize) -> u32 {
        view.get(SSCTL_OFF + seq as u32 * SEQ_STRIDE)
    }

    /// Sample one channel code with `SAC` averaging folded in.
    fn sample(&self, view: &Registers<&mut [u32]>, channel: usize, temp: bool) -> u16 {
        let code = if temp {
            self.temp_code
        } else {
            self.channels.get(channel).copied().unwrap_or(0)
        } & 0x0FFF;
        let avg = view.get(SAC_OFF) & 0x07;
        if avg == 0 || avg > 6 {
            return code;
        }
        // `2^AVG` identical samples average to the sample itself.
        let _rounds = 1u32 << avg;
        code
    }

    /// Run sequencer `seq` to its `END` sample (instant completion).
    fn run_sequence(&mut self, view: &mut Registers<&mut [u32]>, seq: usize) {
        let mux = self.mux(view, seq);
        let ctl = self.ctl(view, seq);
        let depth = SEQ_DEPTH[seq];
        let mut fired = false;
        for slot in 0..SEQ_SLOTS[seq] {
            let nibble = (ctl >> (4 * slot)) & 0x0F;
            let channel = ((mux >> (4 * slot)) & 0x0F) as usize;
            let temp = nibble & SN_TS != 0;
            let code = self.sample(view, channel, temp);
            if self.fifo[seq].len() < depth {
                self.fifo[seq].push(code);
                self.head[seq] = (self.head[seq] + 1) % depth as u8;
            } else {
                let ostat = view.get(OSTAT_OFF);
                view.set(OSTAT_OFF, ostat | (1 << seq));
            }
            if nibble & SN_IE != 0 {
                fired = true;
            }
            if nibble & SN_END != 0 {
                break;
            }
        }
        if fired {
            self.inr[seq] = true;
        }
        self.refresh(view);
    }

    fn ris(&self) -> u32 {
        let mut ris = 0u32;
        for (i, f) in self.inr.iter().enumerate() {
            if *f {
                ris |= 1 << i;
            }
        }
        ris
    }

    fn fstat(&self, seq: usize) -> u32 {
        let len = self.fifo[seq].len() as u32;
        let depth = SEQ_DEPTH[seq] as u32;
        let mut w = u32::from(self.tail[seq] & 0x0F) | (u32::from(self.head[seq] & 0x0F) << 4);
        if len == 0 {
            w |= FSTAT_EMPTY;
        }
        if len >= depth {
            w |= FSTAT_FULL;
        }
        w
    }

    /// Sequencer execution order for simultaneous `PSSI` triggers:
    /// `ADCSSPRI` priority encodings ascending (`0x0` highest, p845),
    /// ties broken toward the lower sequencer (the encodings must be
    /// unique for the ADC to operate properly).
    fn sspri_order(view: &Registers<&mut [u32]>) -> [usize; 4] {
        let pri = view.get(SSPRI_OFF);
        let mut order = [0usize, 1, 2, 3];
        order.sort_by_key(|&seq| ((pri >> (4 * seq)) & 0x03, seq as u32));
        order
    }

    /// `SSFIFO`/`SSFSTAT` window test for any sequencer.
    fn fifo_seq(offset: u32, base: u32) -> Option<usize> {
        if offset >= base && offset < base + 4 * SEQ_STRIDE && (offset - base) % SEQ_STRIDE == 0 {
            Some(((offset - base) / SEQ_STRIDE) as usize)
        } else {
            None
        }
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        let im = view.get(IM_OFF);
        self.pending = self.ris() & im & 0x0F;
    }
}

impl Behavior for AdcBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        let (channels, temp_code) = (self.channels, self.temp_code);
        *self = Self::default();
        self.channels = channels;
        self.temp_code = temp_code;
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = if offset == RIS_OFF {
            self.ris()
        } else if offset == ISC_OFF {
            // Reads show the masked status.
            self.ris() & view.get(IM_OFF) & 0x0F
        } else if let Some(seq) = Self::fifo_seq(offset, SSFIFO_OFF) {
            let _ = seq;
            // Reached only by upper-lane reads routed around the
            // lane-0 pop below; report the latched entry's high byte.
            u32::from(self.popped >> 8)
        } else if let Some(seq) = Self::fifo_seq(offset, SSFSTAT_OFF) {
            self.fstat(seq)
        } else {
            value
        };
        self.refresh(view);
        out
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == RIS_OFF {
            return view.get(offset);
        }
        if offset == ISC_OFF {
            // W1C: clear raw flags; reads show masked status.
            let word = view.merged(offset, val, shift);
            for seq in 0..4 {
                if word & (1 << seq) != 0 {
                    self.inr[seq] = false;
                }
            }
            self.refresh(view);
            return self.ris() & view.get(IM_OFF) & 0x0F;
        }
        if offset == PSSI_OFF {
            // Processor trigger: run each requested *enabled* sequencer
            // with a processor-selected trigger (`EMUX=0x0`), in
            // `ADCSSPRI` priority order for simultaneous triggers (p845).
            let word = view.merged(offset, val, shift);
            for seq in Self::sspri_order(view) {
                let emux = (view.get(EMUX_OFF) >> (4 * seq)) & 0x0F;
                if word & (1 << seq) != 0 && view.get(ACTSS_OFF) & (1 << seq) != 0 && emux == 0 {
                    self.run_sequence(view, seq);
                }
            }
            self.refresh(view);
            return 0;
        }
        if Self::fifo_seq(offset, SSFIFO_OFF).is_some()
            || Self::fifo_seq(offset, SSFSTAT_OFF).is_some()
        {
            return view.get(offset);
        }
        if offset == OSTAT_OFF || offset == USTAT_OFF {
            // RW1C: writing 1 clears the bit; writing 0 leaves it.
            let word = view.get(offset) & !(u32::from(val) << shift);
            self.refresh(view);
            return word;
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        self.refresh(view);
        word
    }
}

/// A Tiva C ADC (processor-triggered sequencers).
pub struct Adc {
    inner: RegisterFile<AdcBehavior>,
}

impl Adc {
    /// A new ADC on `D`'s tables (`"ADC0"`/`"ADC1"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain ADC");
        let mut adc = Self {
            inner: RegisterFile::new(def, AdcBehavior::default()),
        };
        adc.stamp_resets();
        adc
    }

    /// Stamp the nonzero datasheet resets over the SVD zeros.
    fn stamp_resets(&mut self) {
        let def = self.inner.def();
        let stamped: Vec<(usize, u32)> = RESETS
            .iter()
            .filter_map(|&(offset, value)| {
                def.registers
                    .iter()
                    .position(|r| r.offset == offset)
                    .map(|idx| (idx, value))
            })
            .collect();
        let words = self.inner.regs_mut();
        for (idx, value) in stamped {
            words[idx] = value;
        }
    }

    /// Drive a 12-bit channel code (host/test stimulus, `AIN0`-`AIN11`).
    pub fn drive_channel(&mut self, channel: usize, code: u16) {
        self.inner.with_view(|_, b| {
            if channel < b.channels.len() {
                b.channels[channel] = code & 0x0FFF;
            }
        });
    }

    /// Drive the temperature-sensor code (`TS=1` samples).
    pub fn drive_temp(&mut self, code: u16) {
        self.inner.with_view(|_, b| b.temp_code = code & 0x0FFF);
    }
}

impl Peripheral for Adc {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        // `SSFIFO` pops one entry on the lane-0 read (firmware
        // word-reads ascending); upper lanes fetch the latched entry.
        let (base, len) = {
            let r = self.inner.ranges()[0];
            (r.base, r.len)
        };
        let off = addr
            .checked_sub(base)
            .filter(|o| *o < len)
            .ok_or(BusError::Reserved)?;
        for (seq, _) in SEQ_DEPTH.iter().enumerate() {
            let fifo = SSFIFO_OFF + seq as u32 * SEQ_STRIDE;
            if (fifo..fifo + 4).contains(&off) {
                let lane = off - fifo;
                if lane == 0 {
                    self.inner.with_view(|view, b| {
                        if b.fifo[seq].is_empty() {
                            let ustat = view.get(USTAT_OFF);
                            view.set(USTAT_OFF, ustat | (1 << seq));
                            b.popped = 0;
                        } else {
                            b.popped = b.fifo[seq].remove(0);
                            b.tail[seq] = (b.tail[seq] + 1) % SEQ_DEPTH[seq] as u8;
                        }
                        b.refresh(view);
                    });
                    return Ok((self.inner.behavior().popped & 0xFF) as u8);
                }
                if lane == 1 {
                    return Ok((self.inner.behavior().popped >> 8) as u8);
                }
                return Ok(0);
            }
        }
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
        self.stamp_resets();
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for Adc {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    // Lines map 1:1 onto `ADCnSS0`-`ADCnSS3` via the default
    // `exception_for_line` (base + line); `pending` carries one bit
    // per sequencer.
    fn lines(&self) -> u32 {
        4
    }

    fn pending(&self) -> u32 {
        self.inner.behavior().pending
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const ADC0_BASE: u32 = 0x4003_8000;
    /// `SSCTL` nibble: `END` set, single-ended, no interrupt, MUX channel.
    const SN_END: u32 = 0x02;
    const SN_IE_END: u32 = 0x06;

    fn adc0() -> Adc {
        Adc::for_device::<Tm4c123gh6pm>("ADC0")
    }

    fn read_word(adc: &mut Adc, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(adc.read(ADC0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(adc: &mut Adc, offset: u32, value: u32) {
        for lane in 0..4 {
            adc.write(
                ADC0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Enable sequencer 3 (single sample) for processor trigger.
    fn setup_ss3(adc: &mut Adc) {
        write_word(adc, ACTSS_OFF, 0x08);
        write_word(adc, SSMUX_OFF + 3 * SEQ_STRIDE, 0x02); // AIN2
        write_word(adc, SSCTL_OFF + 3 * SEQ_STRIDE, SN_IE_END);
        write_word(adc, IM_OFF, 0x08);
    }

    /// Datasheet resets: sequencer FIFOs read empty.
    #[test]
    fn datasheet_resets() {
        let mut adc = adc0();
        assert_eq!(read_word(&mut adc, SSPRI_OFF), 0x3210);
        assert_eq!(read_word(&mut adc, SSFSTAT_OFF), FSTAT_EMPTY);
        assert_eq!(read_word(&mut adc, RIS_OFF), 0);
    }

    /// Processor trigger converts the driven channel into the FIFO and
    /// raises `INR3` through the mask.
    #[test]
    fn processor_trigger_converts() {
        let mut adc = adc0();
        adc.drive_channel(2, 0xABC);
        setup_ss3(&mut adc);
        write_word(&mut adc, PSSI_OFF, 0x08);
        assert_eq!(
            read_word(&mut adc, SSFIFO_OFF + 3 * SEQ_STRIDE) & 0xFFF,
            0xABC
        );
        assert_eq!(read_word(&mut adc, RIS_OFF) & 0x08, 0x08);
        assert_eq!(read_word(&mut adc, ISC_OFF) & 0x08, 0x08);
        assert_eq!(adc.pending() & 0x08, 0x08);
        assert_eq!(adc.exception_number(), Some(30));
        assert_eq!(adc.exception_for_line(3), Some(33));
        // Clear and confirm the FIFO drained to empty.
        write_word(&mut adc, ISC_OFF, 0x08);
        assert_eq!(adc.pending() & 0x08, 0);
        assert_eq!(
            read_word(&mut adc, SSFSTAT_OFF + 3 * SEQ_STRIDE) & FSTAT_EMPTY,
            FSTAT_EMPTY
        );
    }

    /// A disabled sequencer ignores `PSSI`.
    #[test]
    fn disabled_sequencer_ignores_pssi() {
        let mut adc = adc0();
        adc.drive_channel(2, 0xABC);
        write_word(&mut adc, SSMUX_OFF + 3 * SEQ_STRIDE, 0x02);
        write_word(&mut adc, SSCTL_OFF + 3 * SEQ_STRIDE, SN_IE_END);
        write_word(&mut adc, PSSI_OFF, 0x08); // ACTSS clear
        assert_eq!(read_word(&mut adc, RIS_OFF), 0);
        assert_eq!(
            read_word(&mut adc, SSFSTAT_OFF + 3 * SEQ_STRIDE) & FSTAT_EMPTY,
            FSTAT_EMPTY
        );
    }

    /// Multi-sample SS0 sequence runs to `END` in order.
    #[test]
    fn ss0_runs_to_end() {
        let mut adc = adc0();
        adc.drive_channel(0, 0x100);
        adc.drive_channel(1, 0x200);
        adc.drive_channel(2, 0x300);
        write_word(&mut adc, ACTSS_OFF, 0x01);
        // MUX: slots 0,1,2 = AIN0,AIN1,AIN2.
        write_word(&mut adc, SSMUX_OFF, 0x0000_0210);
        // CTL: slots 0,1 plain, slot 2 END+IE.
        write_word(&mut adc, SSCTL_OFF, SN_IE_END << 8);
        write_word(&mut adc, IM_OFF, 0x01);
        write_word(&mut adc, PSSI_OFF, 0x01);
        assert_eq!(read_word(&mut adc, SSFIFO_OFF) & 0xFFF, 0x100);
        assert_eq!(read_word(&mut adc, SSFIFO_OFF) & 0xFFF, 0x200);
        assert_eq!(read_word(&mut adc, SSFIFO_OFF) & 0xFFF, 0x300);
        assert_eq!(adc.pending() & 0x01, 0x01);
    }

    /// Overflow sets `OSTAT`; empty reads set `USTAT`.
    #[test]
    fn overflow_and_underflow_flags() {
        let mut adc = adc0();
        setup_ss3(&mut adc);
        // SS3 is 1-deep: two triggers overflow.
        write_word(&mut adc, PSSI_OFF, 0x08);
        write_word(&mut adc, PSSI_OFF, 0x08);
        assert_eq!(read_word(&mut adc, OSTAT_OFF) & 0x08, 0x08);
        // Drain one, then read empty: underflow.
        read_word(&mut adc, SSFIFO_OFF + 3 * SEQ_STRIDE);
        read_word(&mut adc, SSFIFO_OFF + 3 * SEQ_STRIDE);
        assert_eq!(read_word(&mut adc, USTAT_OFF) & 0x08, 0x08);
    }

    /// `OSTAT`/`USTAT` are RW1C: writing 1 clears, writing 0 leaves the bit.
    #[test]
    fn ostat_ustat_rw1c() {
        let mut adc = adc0();
        setup_ss3(&mut adc);
        write_word(&mut adc, PSSI_OFF, 0x08);
        write_word(&mut adc, PSSI_OFF, 0x08); // SS3 is 1-deep: overflow
        assert_eq!(read_word(&mut adc, OSTAT_OFF) & 0x08, 0x08);
        write_word(&mut adc, OSTAT_OFF, 0x00); // writing 0 clears nothing
        assert_eq!(read_word(&mut adc, OSTAT_OFF) & 0x08, 0x08);
        write_word(&mut adc, OSTAT_OFF, 0x08); // writing 1 clears
        assert_eq!(read_word(&mut adc, OSTAT_OFF) & 0x08, 0);
        // Drain the one entry, then read empty: underflow, then clear.
        read_word(&mut adc, SSFIFO_OFF + 3 * SEQ_STRIDE);
        read_word(&mut adc, SSFIFO_OFF + 3 * SEQ_STRIDE);
        assert_eq!(read_word(&mut adc, USTAT_OFF) & 0x08, 0x08);
        write_word(&mut adc, USTAT_OFF, 0x08);
        assert_eq!(read_word(&mut adc, USTAT_OFF) & 0x08, 0);
    }

    /// `SSFSTAT.HPTR/TPTR` are real head/tail pointers (p862): reset 0/0
    /// with `EMPTY`, advancing on push/pop and diverging on wrap.
    #[test]
    fn ssfstat_head_tail_pointers() {
        let mut adc = adc0();
        let s1mux = SSMUX_OFF + SEQ_STRIDE;
        let s1ctl = SSCTL_OFF + SEQ_STRIDE;
        let s1fifo = SSFIFO_OFF + SEQ_STRIDE;
        let s1stat = SSFSTAT_OFF + SEQ_STRIDE;
        assert_eq!(read_word(&mut adc, s1stat), FSTAT_EMPTY);
        adc.drive_channel(0, 0x111);
        write_word(&mut adc, ACTSS_OFF, 0x02);
        write_word(&mut adc, s1mux, 0x00);
        write_word(&mut adc, s1ctl, SN_IE_END);
        write_word(&mut adc, PSSI_OFF, 0x02);
        write_word(&mut adc, PSSI_OFF, 0x02);
        write_word(&mut adc, PSSI_OFF, 0x02);
        let stat = read_word(&mut adc, s1stat);
        assert_eq!((stat >> 4) & 0x0F, 3); // HPTR: three pushes
        assert_eq!(stat & 0x0F, 0); // TPTR: no pops yet
        assert_eq!(stat & FSTAT_EMPTY, 0);
        read_word(&mut adc, s1fifo); // pop: only TPTR advances
        let stat = read_word(&mut adc, s1stat);
        assert_eq!((stat >> 4) & 0x0F, 3);
        assert_eq!(stat & 0x0F, 1); // diverged
        write_word(&mut adc, PSSI_OFF, 0x02); // head wraps 3 -> 0
        let stat = read_word(&mut adc, s1stat);
        assert_eq!((stat >> 4) & 0x0F, 0);
        assert_eq!(stat & 0x0F, 1);
    }

    /// Simultaneous `PSSI` triggers run in `ADCSSPRI` order (p845).
    #[test]
    fn pssi_follows_sspri() {
        let mut adc = adc0();
        let order = adc
            .inner
            .with_view(|view, _| AdcBehavior::sspri_order(view));
        assert_eq!(order, [0, 1, 2, 3]); // reset `0x3210`: SS0 highest
                                         // SS3 highest, SS0 lowest: SS3=0, SS2=1, SS1=2, SS0=3.
        write_word(&mut adc, SSPRI_OFF, 0x0000_0123);
        let order = adc
            .inner
            .with_view(|view, _| AdcBehavior::sspri_order(view));
        assert_eq!(order, [3, 2, 1, 0]);
        // Simultaneous triggers still convert every requested sequencer.
        adc.drive_channel(0, 0x100);
        adc.drive_channel(2, 0x300);
        write_word(&mut adc, ACTSS_OFF, 0x09);
        write_word(&mut adc, SSMUX_OFF, 0x00);
        write_word(&mut adc, SSCTL_OFF, SN_IE_END);
        write_word(&mut adc, SSMUX_OFF + 3 * SEQ_STRIDE, 0x02);
        write_word(&mut adc, SSCTL_OFF + 3 * SEQ_STRIDE, SN_IE_END);
        write_word(&mut adc, PSSI_OFF, 0x09);
        assert_eq!(read_word(&mut adc, SSFIFO_OFF) & 0xFFF, 0x100);
        assert_eq!(
            read_word(&mut adc, SSFIFO_OFF + 3 * SEQ_STRIDE) & 0xFFF,
            0x300
        );
    }

    /// Temperature-sensor samples read the driven temp code.
    #[test]
    fn temp_sensor_reads_driven_code() {
        let mut adc = adc0();
        adc.drive_temp(0x7FF);
        write_word(&mut adc, ACTSS_OFF, 0x08);
        write_word(&mut adc, SSMUX_OFF + 3 * SEQ_STRIDE, 0x00);
        write_word(&mut adc, SSCTL_OFF + 3 * SEQ_STRIDE, 0x0E); // TS+IE+END
        write_word(&mut adc, PSSI_OFF, 0x08);
        assert_eq!(
            read_word(&mut adc, SSFIFO_OFF + 3 * SEQ_STRIDE) & 0xFFF,
            0x7FF
        );
    }
}
