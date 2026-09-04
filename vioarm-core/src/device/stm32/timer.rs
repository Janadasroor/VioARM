//! The STM32 general-purpose and basic timers (`TIM1`..`TIM7`), driven by the
//! SVD-generated register tables.
//!
//! Register layout and resets come from the SVD; the table cannot express the
//! counting core, so [`TimerBehavior`] runs it: `CR1.CEN` gates the counter,
//! the prescaler `PSC` divides the clock by `PSC + 1`, and `CNT` wraps past
//! `ARR` into an update event that sets `SR.UIF` and — with `DIER.UIE` set —
//! raises a level-triggered IRQ. `EGR.UG` forces an update from software.
//! Channel logic (`CCMR`/`CCER`/`CCR`) implements output compare (frozen,
//! active/inactive, toggle, PWM1/2) with `EGR.CCxG` force, input capture
//! via host `inject_capture`, and one-pulse `CR1.OPM`.
//!
//! The counter width varies by instance: `TIM2`/`TIM5` are 32-bit, the rest
//! 16-bit. The advanced `TIM1` raises four lines (break, update, trigger/comm,
//! capture/compare); the update interrupt is the second line in the table, so
//! the model routes its single pending line to `TIM1_UP` (IRQ 25) rather than
//! the break line the table happens to list first.

use crate::device::stm32::dma::DmaRequests;
use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// Register offsets within the timer, per the SVD.
const CR1: u32 = 0x00;
const SMCR: u32 = 0x08;
const DIER: u32 = 0x0C;
const SR: u32 = 0x10;
const EGR: u32 = 0x14;
const CCMR1: u32 = 0x18;
const CCMR2: u32 = 0x1C;
const CCER: u32 = 0x20;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const CCR1: u32 = 0x34;
const CCR2: u32 = 0x38;
const CCR3: u32 = 0x3C;
const CCR4: u32 = 0x40;
/// `BDTR` (break and dead-time, advanced timers).
const BDTR: u32 = 0x44;

/// `SR.BIF` / `DIER.BIE`: the break interrupt.
const BIF: u32 = 1 << 7;
const BIE: u32 = 1 << 7;

/// `BDTR` bits: main output enable, break enable/polarity.
const MOE: u32 = 1 << 15;
const BKP: u32 = 1 << 13;
const BKE: u32 = 1 << 12;

/// `CEN` bit in `CR1`.
const CEN: u32 = 1 << 0;
const OPM: u32 = 1 << 3;
/// `UIF` bit in `SR` / `UG` bit in `EGR` / `UIE` bit in `DIER`.
const UIF: u32 = 1 << 0;
const UIE: u32 = 1 << 0;
const UG: u32 = 1 << 0;
const CC1G: u32 = 1 << 1;
const CC2G: u32 = 1 << 2;
const CC3G: u32 = 1 << 3;
const CC4G: u32 = 1 << 4;
const CC1IE: u32 = 1 << 1;
const CC2IE: u32 = 1 << 2;
const CC3IE: u32 = 1 << 3;
const CC4IE: u32 = 1 << 4;
const CC1IF: u32 = 1 << 1;
const CC2IF: u32 = 1 << 2;
const CC3IF: u32 = 1 << 3;
const CC4IF: u32 = 1 << 4;
const CC1E: u32 = 1 << 0;
const CC2E: u32 = 1 << 4;
const CC3E: u32 = 1 << 8;
const CC4E: u32 = 1 << 12;
const UDE: u32 = 1 << 8;
const CC1DE: u32 = 1 << 9;
const CC2DE: u32 = 1 << 10;
const CC3DE: u32 = 1 << 11;
const CC4DE: u32 = 1 << 12;

/// The counting core the SVD cannot express.
#[derive(Clone, Debug)]
struct TimerBehavior {
    /// Register width mask: `u32::MAX` for the 32-bit `TIM2`/`TIM5`,
    /// `0xFFFF` for the 16-bit counters.
    width_mask: u32,
    /// Prescaler phase within the current `PSC + 1` division.
    psc_phase: u64,
    /// Per-channel output levels (the pin, host-visible via `Timer::output_level`).
    output: [bool; 4],
    /// Complementary output levels (OCxN pins) with dead-time insertion:
    /// each `output_n` holds off for `dtg` ticks after its OCx toggles.
    output_n: [bool; 4],
    /// Remaining hold-off ticks per complementary channel.
    dtg_holdoff: [u64; 4],
    /// The decoded dead time captured when `BDTR.DTG` was last written.
    dtg: u64,
    /// Optional DMA update request line: `Some((requests, controller, channel))`.
    dma_update: Option<(Arc<Mutex<DmaRequests>>, u8, u8)>,
    /// The host-driven `BKIN` break input level (advanced timers only; the
    /// field is ignored on timers whose tables lack `BDTR`).
    bkin: bool,
    /// Encoder-interface input levels: bit 0 = TI1, bit 1 = TI2. Edges are
    /// detected against the previous drive and counted per `SMCR.SMS`.
    encoder_inputs: u8,
}

impl TimerBehavior {
    fn new(width_mask: u32) -> Self {
        Self {
            width_mask,
            psc_phase: 0,
            output: [false; 4],
            output_n: [false; 4],
            dtg_holdoff: [0; 4],
            dtg: 0,
            dma_update: None,
            bkin: false,
            encoder_inputs: 0,
        }
    }

    /// The `BDTR` break input is active: enabled (`BKE`) and the level
    /// disagrees with the selected polarity (`BKP`).
    const fn break_active(bdtr: Option<u32>, bkin: bool) -> bool {
        let Some(bdtr) = bdtr else {
            return false;
        };
        bdtr & BKE != 0 && bkin != (bdtr & BKP != 0)
    }

    /// Latch a break event: clear the main output, set `SR.BIF`, and drop the
    /// channel outputs (the pins go to their inactive/OSSI state).
    fn latch_break(&mut self, view: &mut Registers<&mut [u32]>) {
        if view.try_get(BDTR).is_some_and(|b| b & MOE != 0) {
            view.set(BDTR, view.get(BDTR) & !MOE);
            view.set(SR, view.get(SR) | BIF);
            self.output = [false; 4];
        }
    }

    /// Dead-time in `tDTS` units decoded from `BDTR.DTG` (RM0008 §17.4.18).
    fn dtg_ticks(&self, view: &Registers<&mut [u32]>) -> u32 {
        let Some(bdtr) = view.try_get(BDTR) else {
            return 0;
        };
        Self::decode_dtg((bdtr & 0xFF) as u8)
    }

    /// The RM0008 §17.4.18 dead-time generator decode, in `tDTS` units:
    /// `DTG[7:5]=0xx` → `DTG·t`, `10x` → `(64+DTG[5:0])·2t`,
    /// `110` → `(32+DTG[4:0])·8t`, `111` → `(32+DTG[4:0])·16t`.
    fn decode_dtg(dtg: u8) -> u32 {
        match dtg >> 5 {
            0b000..=0b011 => u32::from(dtg),
            0b100 | 0b101 => u32::from(64 + (dtg & 0x3F)) * 2,
            0b110 => u32::from(32 + (dtg & 0x1F)) * 8,
            _ => u32::from(32 + (dtg & 0x1F)) * 16,
        }
    }

    /// Dead-time insertion between complementary OCx/OCxN pairs (advanced
    /// timers with `BDTR`): while `MOE` is set each `OCxN` is the inverse of
    /// its `OCx` held off by the decoded dead time after every OCx edge.
    fn run_dead_time(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        let Some(bdtr) = view.try_get(BDTR) else {
            return;
        };
        if bdtr & MOE == 0 {
            // Main output disabled: both outputs inactive.
            self.output_n = [false; 4];
            return;
        }
        // Track DTG writes so a runtime change re-decodes.
        let dtg_raw = (bdtr & 0xFF) as u8;
        let decoded = Self::decode_dtg(dtg_raw) as u64;
        let ccer = view.get(CCER);
        for ch in 0..4usize {
            let (ccxp_bit, ccxnp_bit, ccxe_bit, ccxne_bit) = match ch {
                0 => (1, 1 << 2, CC1E, 1 << 2),
                1 => (1 << 5, 1 << 7, CC2E, 1 << 10),
                2 => (1 << 9, 1 << 11, CC3E, 1 << 12),
                _ => (1 << 13, 1 << 15, CC4E, 1 << 14),
            };
            let _ = ccxp_bit;
            // The complementary output exists when CCxNE is enabled.
            let has_n = ccer & ccxne_bit != 0;
            if !has_n {
                continue;
            }
            let want_n = !self.output[ch];
            if want_n != self.output_n[ch] && self.dtg_holdoff[ch] == 0 {
                // Edge on OCx: hold OCxN off for the dead time, then switch.
                if decoded == 0 {
                    self.output_n[ch] = want_n;
                } else {
                    self.dtg_holdoff[ch] = decoded;
                    self.output_n[ch] = false;
                }
            }
            if self.dtg_holdoff[ch] > 0 {
                let spent = delta.min(self.dtg_holdoff[ch]);
                self.dtg_holdoff[ch] -= spent;
                if self.dtg_holdoff[ch] == 0 {
                    self.output_n[ch] = want_n;
                }
            } else {
                self.output_n[ch] = want_n;
            }
            let _ = ccxnp_bit;
            let _ = ccxe_bit;
        }
    }
    /// One quadrature step: detect the TI1/TI2 edge that just arrived and
    /// move `CNT` per the RM0008 §17.3.12 encoding table, honoring the
    /// selected `SMCR.SMS` mode (`001` = count TI2 edges only, `010` = TI1
    /// only, `011` = both).
    fn count_encoder_edge(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        old_ti2: bool,
        old_ti1: bool,
        new_ti2: bool,
        new_ti1: bool,
    ) {
        let sms = view.get(SMCR) & 0x7;
        let ti1_rising = !old_ti1 && new_ti1;
        let ti1_falling = old_ti1 && !new_ti1;
        let ti2_rising = !old_ti2 && new_ti2;
        let ti2_falling = old_ti2 && !new_ti2;
        let mut delta: i64 = 0;
        match sms {
            0b001 => {
                // Encoder mode 1: edges of TI2.
                if ti2_rising && new_ti1 {
                    delta = 1;
                }
                if ti2_falling && !new_ti1 {
                    delta = 1;
                }
                if ti2_rising && !new_ti1 {
                    delta = -1;
                }
                if ti2_falling && new_ti1 {
                    delta = -1;
                }
            }
            0b010 => {
                // Encoder mode 2: edges of TI1.
                if ti1_rising && !new_ti2 {
                    delta = 1;
                }
                if ti1_falling && new_ti2 {
                    delta = 1;
                }
                if ti1_rising && new_ti2 {
                    delta = -1;
                }
                if ti1_falling && !new_ti2 {
                    delta = -1;
                }
            }
            _ => {
                // Encoder mode 3: both signal edges (full quadrature x4).
                if ti1_rising && !new_ti2
                    || ti1_falling && new_ti2
                    || ti2_rising && new_ti1
                    || ti2_falling && !new_ti1
                {
                    delta = 1;
                }
                if ti1_rising && new_ti2
                    || ti1_falling && !new_ti2
                    || ti2_rising && !new_ti1
                    || ti2_falling && new_ti1
                {
                    delta = -1;
                }
            }
        }
        if delta == 0 {
            return;
        }
        let arr = view.get(ARR) & self.width_mask;
        let cnt = view.get(CNT) & self.width_mask;
        let next = if delta > 0 {
            if cnt >= arr {
                0
            } else {
                cnt + 1
            }
        } else if cnt == 0 {
            arr
        } else {
            cnt - 1
        };
        view.set(CNT, next);
    }

    fn handle_compare(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        ch: usize,
        ocm: u32,
        next: u32,
        ccr: u32,
    ) {
        let ccer = view.try_get(CCER).unwrap_or(0);
        let en_bit = match ch {
            0 => CC1E,
            1 => CC2E,
            2 => CC3E,
            _ => CC4E,
        };
        if ccer & en_bit == 0 {
            return;
        }
        let ccr_masked = ccr & self.width_mask;
        // For PWM modes we update output every tick, otherwise on match.
        match ocm {
            6 => {
                // PWM1: active while CNT < CCR
                self.output[ch] = next < ccr_masked;
                if next == ccr_masked {
                    // Also set flag on match for interrupt.
                    let flag = match ch {
                        0 => CC1IF,
                        1 => CC2IF,
                        2 => CC3IF,
                        _ => CC4IF,
                    };
                    view.set(SR, view.get(SR) | flag);
                }
            }
            7 => {
                // PWM2: inactive while CNT < CCR (inverted)
                self.output[ch] = next >= ccr_masked;
                if next == ccr_masked {
                    let flag = match ch {
                        0 => CC1IF,
                        1 => CC2IF,
                        2 => CC3IF,
                        _ => CC4IF,
                    };
                    view.set(SR, view.get(SR) | flag);
                }
            }
            0 => {
                // Frozen: no output change, but still set flag on match.
                if next == ccr_masked {
                    let flag = match ch {
                        0 => CC1IF,
                        1 => CC2IF,
                        2 => CC3IF,
                        _ => CC4IF,
                    };
                    view.set(SR, view.get(SR) | flag);
                }
            }
            1 => {
                // Active on match
                if next == ccr_masked {
                    self.output[ch] = true;
                    let flag = match ch {
                        0 => CC1IF,
                        1 => CC2IF,
                        2 => CC3IF,
                        _ => CC4IF,
                    };
                    view.set(SR, view.get(SR) | flag);
                }
            }
            2 => {
                // Inactive on match
                if next == ccr_masked {
                    self.output[ch] = false;
                    let flag = match ch {
                        0 => CC1IF,
                        1 => CC2IF,
                        2 => CC3IF,
                        _ => CC4IF,
                    };
                    view.set(SR, view.get(SR) | flag);
                }
            }
            3 => {
                // Toggle on match
                if next == ccr_masked {
                    self.output[ch] = !self.output[ch];
                    let flag = match ch {
                        0 => CC1IF,
                        1 => CC2IF,
                        2 => CC3IF,
                        _ => CC4IF,
                    };
                    view.set(SR, view.get(SR) | flag);
                }
            }
            _ => {
                // Reserved 100,101: treat as frozen but set flag.
                if next == ccr_masked {
                    let flag = match ch {
                        0 => CC1IF,
                        1 => CC2IF,
                        2 => CC3IF,
                        _ => CC4IF,
                    };
                    view.set(SR, view.get(SR) | flag);
                }
            }
        }
    }

    fn inject_capture(&mut self, view: &mut Registers<&mut [u32]>, ch: usize) {
        // Latch current CNT into CCRx and set CCxIF. Honor width mask.
        let cnt = view.get(CNT) & self.width_mask;
        let (ccr_off, flag) = match ch {
            0 => (CCR1, CC1IF),
            1 => (CCR2, CC2IF),
            2 => (CCR3, CC3IF),
            _ => (CCR4, CC4IF),
        };
        if view.try_get(ccr_off).is_some() {
            view.set(ccr_off, cnt);
            view.set(SR, view.get(SR) | flag);
        }
    }
}

impl Behavior for TimerBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == EGR {
            let bits = u32::from(val) << shift;
            if bits & UG != 0 {
                // Any write with UG set forces a software update event: reload the
                // counter and prescaler, and set the update flag like a real
                // overflow would.
                view.set(CNT, 0);
                self.psc_phase = 0;
                view.set(SR, view.get(SR) | UIF);
                // OPM: if set, UG also clears CEN? On silicon update event
                // clears CEN when OPM=1. For UG we also handle.
                if view.get(CR1) & OPM != 0 {
                    view.set(CR1, view.get(CR1) & !CEN);
                }
                if view.get(DIER) & UDE != 0 {
                    if let Some((reqs, ctrl, ch)) = self.dma_update.as_ref() {
                        let reqs = Arc::clone(reqs);
                        reqs.lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .request(*ctrl, *ch, false);
                    }
                }
            }
            // CCxG force generates compare event.
            for (ch, bit) in [(0, CC1G), (1, CC2G), (2, CC3G), (3, CC4G)] {
                if bits & bit != 0 {
                    let flag = match ch {
                        0 => CC1IF,
                        1 => CC2IF,
                        2 => CC3IF,
                        _ => CC4IF,
                    };
                    view.set(SR, view.get(SR) | flag);
                    // For active/toggle modes, update output as if match.
                    // Use current CNT vs CCR comparison for PWM is tick-driven,
                    // but for force we just toggling per OCxM's active semantics.
                    if let (Some(ccr), Some(ccmr)) = (
                        view.try_get(match ch {
                            0 => CCR1,
                            1 => CCR2,
                            2 => CCR3,
                            _ => CCR4,
                        }),
                        view.try_get(if ch < 2 { CCMR1 } else { CCMR2 }),
                    ) {
                        let ocm = if ch % 2 == 0 {
                            (ccmr >> 4) & 0x7
                        } else {
                            (ccmr >> 12) & 0x7
                        };
                        let ccs = if ch % 2 == 0 {
                            ccmr & 0x3
                        } else {
                            (ccmr >> 8) & 0x3
                        };
                        if ccs == 0 {
                            // Only for output compare.
                            match ocm {
                                1 => self.output[ch] = true,
                                2 => self.output[ch] = false,
                                3 => self.output[ch] = !self.output[ch],
                                _ => {}
                            }
                        }
                    }
                }
            }
        }
        let word = view.merged(offset, val, shift);
        match offset {
            // The 16-bit counters and auto-reloads are 16-bit on silicon even
            // though the SVD describes 32-bit register words.
            CNT | ARR => word & self.width_mask,
            _ => word,
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        // The break input overrides counting: while active, the counter is
        // held and the main output stays off (RM0008 §17.4.18).
        if Self::break_active(view.try_get(BDTR), self.bkin) {
            self.latch_break(view);
            return;
        }
        if view.get(CR1) & CEN == 0 {
            self.run_dead_time(view, delta);
            return;
        }
        self.run_dead_time(view, delta);
        let div = u64::from(view.get(PSC).wrapping_add(1));
        let arr = u64::from(view.get(ARR));
        for _ in 0..delta {
            self.psc_phase += 1;
            if self.psc_phase >= div {
                self.psc_phase = 0;
                let cnt = u64::from(view.get(CNT)) + 1;
                let overflow = cnt > arr;
                let next = if overflow { 0 } else { cnt as u32 };
                if overflow {
                    view.set(SR, view.get(SR) | UIF);
                    if view.get(CR1) & OPM != 0 {
                        view.set(CR1, view.get(CR1) & !CEN);
                    }
                    if view.get(DIER) & UDE != 0 {
                        if let Some((reqs, ctrl, ch)) = self.dma_update.as_ref() {
                            let reqs = Arc::clone(reqs);
                            reqs.lock()
                                .unwrap_or_else(|e| e.into_inner())
                                .request(*ctrl, *ch, false);
                        }
                    }
                }
                view.set(CNT, next);
                // For PWM modes, update output every tick before compare check
                // so level reflects CNT < CCR across the period.
                // For other OC modes, handle on match.
                for ch in 0..4 {
                    let (ccr_off, ccmr_off) = match ch {
                        0 => (CCR1, CCMR1),
                        1 => (CCR2, CCMR1),
                        2 => (CCR3, CCMR2),
                        _ => (CCR4, CCMR2),
                    };
                    let Some(ccr) = view.try_get(ccr_off) else {
                        continue;
                    };
                    let Some(ccmr) = view.try_get(ccmr_off) else {
                        continue;
                    };
                    let ccs = if ch == 0 || ch == 2 {
                        ccmr & 0x3
                    } else {
                        (ccmr >> 8) & 0x3
                    };
                    if ccs != 0 {
                        continue; // Input capture mode, not output.
                    }
                    let ocm = if ch == 0 || ch == 2 {
                        (ccmr >> 4) & 0x7
                    } else {
                        (ccmr >> 12) & 0x7
                    };
                    // PWM needs per-tick level update.
                    if ocm == 6 || ocm == 7 {
                        self.handle_compare(view, ch, ocm, next, ccr);
                    } else {
                        // Non-PWM: only on match.
                        let ccr_masked = ccr & self.width_mask;
                        if next == ccr_masked {
                            self.handle_compare(view, ch, ocm, next, ccr);
                        }
                    }
                }
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sr = view.get(SR);
        let dier = view.get(DIER);
        let uie = sr & UIF != 0 && dier & UIE != 0;
        let cc1 = sr & CC1IF != 0 && dier & CC1IE != 0;
        let cc2 = sr & CC2IF != 0 && dier & CC2IE != 0;
        let cc3 = sr & CC3IF != 0 && dier & CC3IE != 0;
        let cc4 = sr & CC4IF != 0 && dier & CC4IE != 0;
        let brk = sr & BIF != 0 && dier & BIE != 0;
        u32::from(uie || brk || cc1 || cc2 || cc3 || cc4)
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        // bit 0 is the BKIN level on advanced timers; bits 1 (TI2) and 2
        // (TI1) feed the encoder interface. (BKIN and TI1 share the low bit
        // without conflict: BDTR exists only on TIM1, SMCR encoder modes only
        // on general-purpose timers.)
        let ti1 = value & (1 << 2) != 0;
        let ti2 = value & (1 << 1) != 0;
        self.bkin = value & 1 != 0;
        if Self::break_active(view.try_get(BDTR), self.bkin) {
            self.latch_break(view);
        }
        if view.try_get(SMCR).is_some_and(|smcr| smcr & 0x7 != 0) {
            let prev = self.encoder_inputs;
            if view.get(SMCR) & 0x7 == 0b100 {
                // Hall-sensor / reset mode: a rising TI1 edge captures the
                // counter into CCR1 and clears it (RM0008 §17.3.14).
                if (prev >> 2) & 1 == 0 && ti1 {
                    let cnt = view.get(CNT);
                    if view.try_get(CCR1).is_some() {
                        view.set(CCR1, cnt);
                        view.set(SR, view.get(SR) | CC1IF);
                    }
                    view.set(CNT, 0);
                }
            } else {
                self.count_encoder_edge(view, prev & 0b10 != 0, prev & 0b01 != 0, ti2, ti1);
            }
            self.encoder_inputs = u8::from(ti1) | (u8::from(ti2) << 1);
        }
    }
}

/// A timer: `TIM2`/`TIM5` (32-bit), the rest 16-bit. Base address, register
/// set, and IRQ come from `D`'s generated tables.
#[derive(Debug)]
pub struct Timer {
    inner: RegisterFile<TimerBehavior>,
    name: &'static str,
    /// The table index of the update interrupt line. The advanced `TIM1`
    /// lists break first, so its update event routes to line 1 (`TIM1_UP`);
    /// the general/basic timers have a single line at index 0.
    irq_line: u32,
}

impl Timer {
    /// A `TIM1` at `0x4001_2C00` (F1): 16-bit counter, update IRQ 25.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim1() -> Self {
        Self::tim1_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM1` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM1` peripheral.
    #[must_use]
    pub fn tim1_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM1", "tim1", 0xFFFF, 1)
    }

    /// A `TIM2` at `0x4000_0000`: 32-bit counter, IRQ 28.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM2`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim2() -> Self {
        Self::tim2_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM2` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM2` peripheral.
    #[must_use]
    pub fn tim2_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM2", "tim2", u32::MAX, 0)
    }

    /// A `TIM3` at `0x4000_0400`: 16-bit counter, IRQ 29.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM3`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim3() -> Self {
        Self::tim3_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM3` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM3` peripheral.
    #[must_use]
    pub fn tim3_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM3", "tim3", 0xFFFF, 0)
    }

    /// A `TIM4` at `0x4000_0800`: 16-bit counter, IRQ 30.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM4`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim4() -> Self {
        Self::tim4_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM4` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM4` peripheral.
    #[must_use]
    pub fn tim4_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM4", "tim4", 0xFFFF, 0)
    }

    /// A `TIM5` at `0x4000_0C00`: 32-bit counter, IRQ 50.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM5`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim5() -> Self {
        Self::tim5_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM5` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM5` peripheral.
    #[must_use]
    pub fn tim5_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM5", "tim5", u32::MAX, 0)
    }

    /// A `TIM6` at `0x4000_1000`: 16-bit basic counter, IRQ 54.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `TIM6`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn tim6() -> Self {
        Self::tim6_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM6` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM6` peripheral.
    #[must_use]
    pub fn tim6_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM6", "tim6", 0xFFFF, 0)
    }

    /// A `TIM7` at `0x4000_1400`: 16-bit basic counter, IRQ 55.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM7` peripheral.
    #[must_use]
    pub fn tim7() -> Self {
        Self::tim7_for::<stm32f103::Stm32f103>()
    }

    /// A `TIM7` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `TIM7` peripheral.
    #[must_use]
    pub fn tim7_for<D: Device>() -> Self {
        Self::for_device::<D>("TIM7", "tim7", 0xFFFF, 0)
    }

    fn for_device<D: Device>(
        peripheral: &str,
        name: &'static str,
        width_mask: u32,
        irq_line: u32,
    ) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        Self {
            inner: RegisterFile::new(def, TimerBehavior::new(width_mask)),
            name,
            irq_line,
        }
    }
    /// Generic `TIMx` by name (e.g. `TIM15`) — 16-bit except `TIM2`/`TIM5` 32-bit.
    #[must_use]
    pub fn new_for_name<D: Device>(name: &'static str) -> Self {
        let width = match name {
            "TIM2" | "TIM5" => u32::MAX,
            _ => 0xFFFF,
        };
        let irq_line = if name == "TIM1" { 1 } else { 0 };
        Self::for_device::<D>(name, name, width, irq_line)
    }

    /// Current output level for `channel` (1..4), the host-visible pin for
    /// output compare / PWM.
    #[must_use]
    pub fn output_level(&self, channel: u8) -> bool {
        if (1..=4).contains(&channel) {
            self.inner.behavior().output[usize::from(channel - 1)]
        } else {
            false
        }
    }

    /// Current complementary output level (OCxN) for `channel` (1..4),
    /// with dead-time insertion applied. Always `false` on timers without
    /// `BDTR` or while `MOE` is clear.
    #[must_use]
    pub fn output_level_n(&self, channel: u8) -> bool {
        if (1..=4).contains(&channel) {
            self.inner.behavior().output_n[usize::from(channel - 1)]
        } else {
            false
        }
    }

    /// Inject a capture event on `channel` (1..4): latches `CNT` into
    /// `CCRx` and sets `CCxIF` as if an edge arrived.
    pub fn inject_capture(&mut self, channel: u8) {
        if !(1..=4).contains(&channel) {
            return;
        }
        let ch = (channel - 1) as usize;
        self.inner
            .with_view(|view, beh| beh.inject_capture(view, ch));
    }

    /// Attach DMA update request line.
    #[must_use]
    pub fn with_dma_update(
        mut self,
        requests: Arc<Mutex<DmaRequests>>,
        controller: u8,
        channel: u8,
    ) -> Self {
        self.inner.behavior_mut().dma_update = Some((requests, controller, channel));
        self
    }

    /// Drive the `BKIN` break input level (advanced timers). An enabled break
    /// (`BDTR.BKE`) whose level disagrees with `BKP` clears `MOE`, latches
    /// `SR.BIF`, and drops the channel outputs.
    pub fn drive_bkin(&mut self, level: bool) {
        let value = u32::from(level);
        self.inner.drive_inputs(value);
    }

    /// The dead time in `tDTS` units decoded from `BDTR.DTG`
    /// (RM0008 §17.4.18). Zero when the timer has no `BDTR`.
    #[must_use]
    pub fn dtg_ticks(&self) -> u32 {
        let bdtr = self.inner.word(BDTR);
        TimerBehavior::decode_dtg((bdtr & 0xFF) as u8)
    }
}

impl Peripheral for Timer {
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

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn name(&self) -> &'static str {
        self.name
    }
}

impl InterruptSource for Timer {
    fn exception_number(&self) -> Option<u8> {
        // The advanced TIM1's first table line is the break interrupt; the
        // update event belongs to the line the constructor stored (1 = UP).
        self.inner.exception_for_line(self.irq_line)
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

#[cfg(test)]
mod tests {
    use super::{Timer, CEN, OPM, UIE, UIF};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const BASE: u32 = 0x4000_0000;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(t: &mut Timer, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            t.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(t: &mut Timer, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = t.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn armed() -> Timer {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x2C, 99); // ARR
        write_word(&mut t, BASE + 0x0C, UIE); // DIER
        write_word(&mut t, BASE + 0x14, 1); // EGR: UG
        write_word(&mut t, BASE + 0x10, 0); // SR: clear UIF
        write_word(&mut t, BASE, CEN); // CR1
        t
    }

    #[test]
    fn counts_up_and_wraps_at_arr() {
        let mut t = armed();
        for _ in 0..98 {
            t.tick(1);
            assert_eq!(t.pending(), 0);
        }
        assert_eq!(read_word(&mut t, BASE + 0x24), 98);
        t.tick(1);
        assert_eq!(read_word(&mut t, BASE + 0x24), 99);
        assert_eq!(t.pending(), 0);
        t.tick(1);
        assert_eq!(read_word(&mut t, BASE + 0x24), 0);
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn prescaler_divides_the_count() {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x28, 9); // PSC -> divide by 10
        write_word(&mut t, BASE + 0x2C, 0); // ARR 0 -> wrap every count
        write_word(&mut t, BASE + 0x0C, UIE);
        write_word(&mut t, BASE, CEN);
        t.tick(9);
        assert_eq!(t.pending(), 0);
        t.tick(1);
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn update_interrupt_needs_uie() {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x2C, 0);
        write_word(&mut t, BASE, CEN);
        t.tick(1);
        assert_eq!(read_word(&mut t, BASE + 0x10) & UIF, UIF);
        assert_eq!(t.pending(), 0);
    }

    #[test]
    fn disabled_timer_does_not_advance() {
        let mut t = armed();
        write_word(&mut t, BASE, 0); // CEN off
        t.tick(1000);
        assert_eq!(read_word(&mut t, BASE + 0x24), 0);
    }

    #[test]
    fn eg_ug_generates_an_update() {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x2C, 50);
        write_word(&mut t, BASE, CEN);
        t.tick(25);
        assert_eq!(read_word(&mut t, BASE + 0x24), 25);
        write_word(&mut t, BASE + 0x14, 1); // UG
        assert_eq!(read_word(&mut t, BASE + 0x24), 0);
        assert_eq!(read_word(&mut t, BASE + 0x10) & UIF, UIF);
    }

    #[test]
    fn tim3_counter_is_sixteen_bit() {
        let mut t = Timer::tim3();
        let tim3_base = crate::device::stm32::TIM3;
        write_word(&mut t, tim3_base + 0x24, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut t, tim3_base + 0x24), 0xFFFF);
    }

    #[test]
    fn tim4_and_tim7_counters_are_sixteen_bit() {
        for (timer, base) in [
            (Timer::tim4(), 0x4000_0800u32),
            (Timer::tim7(), 0x4000_1400),
        ] {
            let mut t = timer;
            write_word(&mut t, base + 0x24, 0xFFFF_FFFF);
            assert_eq!(read_word(&mut t, base + 0x24), 0xFFFF);
        }
    }

    #[test]
    fn tim5_counter_is_thirty_two_bit() {
        let mut t = Timer::tim5();
        write_word(&mut t, 0x4000_0C00 + 0x24, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut t, 0x4000_0C00 + 0x24), 0xFFFF_FFFF);
    }

    #[test]
    fn tim6_basic_counter_wraps_and_irqs() {
        let mut t = Timer::tim6();
        let base = 0x4000_1000u32;
        write_word(&mut t, base + 0x2C, 3); // ARR 3
        write_word(&mut t, base + 0x0C, UIE);
        write_word(&mut t, base, CEN);
        for _ in 0..4 {
            t.tick(1);
        }
        assert_eq!(read_word(&mut t, base + 0x24), 0);
        assert_eq!(t.pending(), 1);
        assert_eq!(t.exception_number(), Some(16 + 54));
    }

    #[test]
    fn full_auto_reload_wrap_sets_the_update_flag() {
        // A 16-bit counter with ARR = 0xFFFF wraps from 0xFFFF back to zero and
        // must raise UIF — the masked counter must not hide the overflow.
        let mut t = Timer::tim4();
        let base = 0x4000_0800u32;
        write_word(&mut t, base + 0x2C, 0xFFFF); // ARR
        write_word(&mut t, base + 0x0C, UIE);
        write_word(&mut t, base, CEN);
        for _ in 0..0xFFFF {
            t.tick(1);
        }
        assert_eq!(read_word(&mut t, base + 0x24), 0xFFFF);
        assert_eq!(t.pending(), 0);
        t.tick(1);
        assert_eq!(read_word(&mut t, base + 0x24), 0);
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn tim1_update_routes_to_the_up_line() {
        // The advanced TIM1's table lists break (24) first; the model must
        // surface the update interrupt (25), not the break line.
        let mut t = Timer::tim1();
        assert_eq!(t.ranges()[0].base, 0x4001_2C00);
        assert_eq!(t.exception_number(), Some(16 + 25));
        write_word(&mut t, 0x4001_2C00 + 0x2C, 1); // ARR 1
        write_word(&mut t, 0x4001_2C00 + 0x0C, UIE);
        write_word(&mut t, 0x4001_2C00, CEN);
        t.tick(2);
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn opm_clears_cen_on_update() {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x2C, 1); // ARR 1
        write_word(&mut t, BASE + 0x0C, UIE);
        write_word(&mut t, BASE, CEN | OPM);
        t.tick(2); // wrap
        assert_eq!(read_word(&mut t, BASE) & CEN, 0, "OPM clears CEN");
        assert_eq!(t.pending(), 1);
        let cnt_before = read_word(&mut t, BASE + 0x24);
        t.tick(10);
        assert_eq!(read_word(&mut t, BASE + 0x24), cnt_before, "stopped");
    }

    #[test]
    fn output_compare_active_sets_level() {
        let mut t = Timer::tim2();
        let base = BASE;
        write_word(&mut t, base + 0x2C, 10); // ARR
        write_word(&mut t, base + 0x34, 5); // CCR1
                                            // CCMR1: OC1M=001 (active), CC1S=00
        write_word(&mut t, base + 0x18, 1 << 4); // CCMR1 OC1M 001
        write_word(&mut t, base + 0x20, 1); // CCER CC1E
        write_word(&mut t, base + 0x0C, 1 << 1); // DIER CC1IE
        write_word(&mut t, base, CEN);
        assert!(!t.output_level(1));
        for _ in 0..5 {
            t.tick(1);
        }
        assert!(t.output_level(1), "active on match");
        assert_eq!(read_word(&mut t, base + 0x10) & (1 << 1), 1 << 1, "CC1IF");
    }

    #[test]
    fn output_compare_toggle() {
        let mut t = Timer::tim2();
        let base = BASE;
        write_word(&mut t, base + 0x2C, 10);
        write_word(&mut t, base + 0x34, 3);
        write_word(&mut t, base + 0x18, 3 << 4); // OC1M 011 toggle
        write_word(&mut t, base + 0x20, 1);
        write_word(&mut t, base, CEN);
        assert!(!t.output_level(1));
        for _ in 0..3 {
            t.tick(1);
        }
        assert!(t.output_level(1), "toggle 1");
        // Wrap and hit again
        for _ in 0..11 {
            t.tick(1);
        }
        assert!(!t.output_level(1), "toggle 2");
    }

    #[test]
    fn pwm1_output_level_sequence() {
        let mut t = Timer::tim2();
        let base = BASE;
        write_word(&mut t, base + 0x2C, 9); // ARR 9 => period 10
        write_word(&mut t, base + 0x34, 5); // CCR 5 => 50% duty
                                            // OC1M 110 PWM1
        write_word(&mut t, base + 0x18, 6 << 4);
        write_word(&mut t, base + 0x20, 1);
        write_word(&mut t, base, CEN);
        // PWM1 active while CNT < CCR; model updates on tick, so before first
        // tick the pin is still low, after the first tick active.
        t.tick(1);
        assert!(t.output_level(1), "PWM1 active after first tick");
        for _ in 0..4 {
            t.tick(1);
        }
        // CNT 5 => inactive (CNT >= CCR)
        assert!(!t.output_level(1), "PWM1 inactive at CCR");
        for _ in 0..4 {
            t.tick(1);
        }
        assert!(!t.output_level(1), "still inactive");
        t.tick(1); // wrap to 0
        assert!(t.output_level(1), "wrap active again");
    }

    #[test]
    fn input_capture_latches_cnt() {
        let mut t = Timer::tim2();
        let base = BASE;
        write_word(&mut t, base + 0x2C, 100);
        // CCMR1 CC1S=01 input TI1, IC1F etc default
        write_word(&mut t, base + 0x18, 1); // CC1S 01
        write_word(&mut t, base + 0x20, 1); // CC1E
        write_word(&mut t, base + 0x0C, 1 << 1); // CC1IE
        write_word(&mut t, base, CEN);
        for _ in 0..42 {
            t.tick(1);
        }
        assert_eq!(read_word(&mut t, base + 0x24), 42);
        t.inject_capture(1);
        assert_eq!(read_word(&mut t, base + 0x34), 42, "CCR1 latched");
        assert_eq!(read_word(&mut t, base + 0x10) & (1 << 1), 1 << 1, "CC1IF");
        assert_eq!(t.pending(), 1);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut t = Timer::tim2();
        assert!(matches!(t.read(BASE + 0x50), Err(BusError::Reserved)));
        assert!(matches!(t.write(BASE - 1, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn dma_update_request_on_overflow() {
        use crate::device::stm32::dma::DmaRequests;
        use std::sync::{Arc, Mutex};
        let reqs = Arc::new(Mutex::new(DmaRequests::new()));
        let mut t = Timer::tim2().with_dma_update(reqs.clone(), 0, 2);
        write_word(&mut t, BASE + 0x2C, 2); // ARR 2
        write_word(&mut t, BASE + 0x0C, 1 << 8); // DIER UDE
        write_word(&mut t, BASE, CEN);
        for _ in 0..2 {
            t.tick(1);
        }
        assert_eq!(reqs.lock().unwrap().to_periph(), 0);
        assert_eq!(reqs.lock().unwrap().from_periph(), 0);
        t.tick(1); // overflow at 2->0
                   // Update DMA is from_periph (timer is peripheral)
        assert_ne!(
            reqs.lock().unwrap().from_periph(),
            0,
            "update DMA requested"
        );
    }

    /// TIM1 break: an active `BKIN` clears `MOE`, latches `BIF`, drops the
    /// outputs, holds the counter; releasing resumes and the ISR's BIE path
    /// raises the line.
    #[test]
    fn tim1_break_clears_moe_and_latches_bif() {
        const T1: u32 = 0x4001_2C00;
        let mut t = Timer::tim1();
        // BDTR: MOE | BKE, BKP=0 (active low break).
        write_word(&mut t, T1 + super::BDTR, (1 << 15) | (1 << 12));
        write_word(&mut t, T1 + 0x2C, 10); // ARR
        write_word(&mut t, T1, CEN);
        assert_eq!(t.exception_number(), Some(16 + 25));
        // Break asserted (level high vs BKP=0): MOE clears, BIF latches.
        t.drive_bkin(true);
        assert_eq!(
            read_word(&mut t, T1 + super::BDTR) & (1 << 15),
            0,
            "MOE cleared by break"
        );
        assert_ne!(read_word(&mut t, T1 + 0x10) & super::BIF, 0, "BIF");
        // Counting is held while the break is active.
        let cnt = read_word(&mut t, T1 + 0x24);
        t.tick(5);
        assert_eq!(read_word(&mut t, T1 + 0x24), cnt, "counter held");
        // With DIER.BIE the pending line asserts until cleared.
        write_word(&mut t, T1 + 0x0C, super::BIE);
        assert_eq!(t.pending(), 1, "break interrupt line");
        // Release: counting resumes.
        t.drive_bkin(false);
        t.acknowledge(1); // NVIC took it; SR.BIF cleared via W1C below
        write_word(&mut t, T1 + 0x10, super::BIF);
        t.tick(1);
        assert_ne!(read_word(&mut t, T1 + 0x24), cnt, "resumed after break");
    }

    /// The `BDTR.DTG` dead-time decode follows RM0008 §17.4.18 exactly.
    #[test]
    fn tim1_dtg_decode_matches_rm_table() {
        let mut t = Timer::tim1();
        const T1: u32 = 0x4001_2C00;
        // 0xx: DT = DTG * tDTS.
        write_word(&mut t, T1 + super::BDTR, 0b0000_1010);
        assert_eq!(t.dtg_ticks(), 10);
        // 10x: DT = (64+DTG[5:0]) * 2.
        write_word(&mut t, T1 + super::BDTR, 0b1000_0001);
        assert_eq!(t.dtg_ticks(), (64 + 1) * 2);
        // 110: DT = (32+DTG[4:0]) * 8.
        write_word(&mut t, T1 + super::BDTR, 0b1100_0010);
        assert_eq!(t.dtg_ticks(), (32 + 2) * 8);
        // 111: DT = (32+DTG[4:0]) * 16.
        write_word(&mut t, T1 + super::BDTR, 0b1110_0011);
        assert_eq!(t.dtg_ticks(), (32 + 3) * 16);
    }

    /// Encoder mode 3 (both edges): a full quadrature cycle moves CNT
    /// forward four counts; reversing the rotation walks it back.
    #[test]
    fn encoder_mode3_quadrature_counts_forward_and_back() {
        const T2: u32 = BASE;
        let mut t = Timer::tim2();
        write_word(&mut t, T2 + super::SMCR, 0b011); // SMS=3: both edges
        write_word(&mut t, T2 + 0x2C, 9); // ARR 9
                                          // Drive a quadrature sequence A/B:
                                          // (T1,T2): 00 -> 10 -> 11 -> 01 -> 00
        let step = |t: &mut Timer, ti1: bool, ti2: bool| {
            let v = u32::from(ti1) << 2 | u32::from(ti2) << 1;
            t.drive_inputs(v);
        };
        step(&mut t, false, false); // establish level
        assert_eq!(read_word(&mut t, T2 + 0x24), 0);
        step(&mut t, true, false); // TI1 rise, TI2 low: up
        assert_eq!(read_word(&mut t, T2 + 0x24), 1);
        step(&mut t, true, true); // TI2 rise, TI1 high: up
        assert_eq!(read_word(&mut t, T2 + 0x24), 2);
        step(&mut t, false, true); // TI1 fall, TI2 high: up
        assert_eq!(read_word(&mut t, T2 + 0x24), 3);
        step(&mut t, false, false); // TI2 fall, TI1 low: up
        assert_eq!(read_word(&mut t, T2 + 0x24), 4);
        // Reverse direction: TI2 rise with TI1 low counts down.
        step(&mut t, false, true);
        assert_eq!(read_word(&mut t, T2 + 0x24), 3);
        step(&mut t, true, true);
        assert_eq!(read_word(&mut t, T2 + 0x24), 2);
    }

    /// Encoder mode wraps through zero into ARR on reverse counting.
    #[test]
    fn encoder_mode_wraps_through_zero() {
        const T3: u32 = crate::device::stm32::TIM3;
        let mut t = Timer::tim3_for::<crate::device::svd::stm32f103::Stm32f103>();
        write_word(&mut t, T3 + super::SMCR, 0b010); // mode 2: TI1 edges only
        write_word(&mut t, T3 + 0x2C, 99); // ARR 99
        let step = |t: &mut Timer, ti1: bool| {
            // Keep TI2 high so TI1 rising counts down in this phase.
            t.drive_inputs(u32::from(ti1) << 2 | 0b10);
        };
        step(&mut t, false);
        step(&mut t, true); // TI1 rise with TI2 high: down from 0 -> ARR
        assert_eq!(read_word(&mut t, T3 + 0x24), 99, "wrapped below zero");
    }

    /// Non-encoder timers ignore the encoder input bits entirely.
    #[test]
    fn plain_timer_ignores_encoder_levels() {
        let mut t = Timer::tim2();
        write_word(&mut t, BASE + 0x2C, 9);
        write_word(&mut t, BASE, CEN); // no SMCR programming
        t.drive_inputs(0b100); // "TI1 rising"
        t.tick(1);
        assert_eq!(
            read_word(&mut t, BASE + 0x24),
            1,
            "CNT advanced by tick, not by inputs"
        );
        t.drive_inputs(0b000);
        assert_eq!(read_word(&mut t, BASE + 0x24), 1, "no edge counting");
    }

    /// Hall-sensor mode (SMS=100): a rising TI1 edge latches CNT into CCR1
    /// (with CC1IF) and resets the counter — the dwell-time measurement
    /// firmware performs per commutation step.
    #[test]
    fn hall_sensor_mode_captures_and_resets_on_ti1_rise() {
        const T3: u32 = crate::device::stm32::TIM3;
        let mut t = Timer::tim3_for::<crate::device::svd::stm32f103::Stm32f103>();
        write_word(&mut t, T3 + super::SMCR, 0b100); // SMS=100 reset mode
        write_word(&mut t, T3 + 0x2C, 99); // ARR
        write_word(&mut t, T3, CEN);
        // Let the counter free-run a few ticks.
        for _ in 0..7 {
            t.tick(1);
        }
        assert_eq!(read_word(&mut t, T3 + 0x24), 7);
        // TI1 rising: capture into CCR1, clear CNT, latch CC1IF.
        t.drive_inputs(0b100); // TI1 high (bit 2)
        assert_eq!(read_word(&mut t, T3 + 0x34), 7, "CCR1 captured dwell");
        assert_eq!(read_word(&mut t, T3 + 0x24), 0, "counter reset");
        assert_ne!(read_word(&mut t, T3 + 0x10) & super::CC1IF, 0, "CC1IF");
        // Falling edge does nothing; counting resumes from zero.
        t.drive_inputs(0b000);
        t.tick(3);
        assert_eq!(read_word(&mut t, T3 + 0x24), 3);
        // Next rising captures again.
        t.drive_inputs(0b100);
        assert_eq!(read_word(&mut t, T3 + 0x34), 3, "second dwell");
    }

    /// Dead-time insertion: with `MOE` set and a PWM channel whose
    /// complementary output is enabled (`CCxNE`), `OCxN` inverts `OCx` and
    /// holds low through the decoded dead time after every OCx edge.
    #[test]
    fn dead_time_inserts_gap_between_oc_and_ocn() {
        const T1: u32 = 0x4001_2C00;
        let mut t = Timer::tim1();
        write_word(&mut t, T1 + 0x2C, 9); // ARR 9
        write_word(&mut t, T1 + 0x34, 4); // CCR1 = 4 => OC1 high while CNT<4
                                          // PWM1 on CH1; CC1E + CC1NE in CCER.
        write_word(&mut t, T1 + 0x18, 6 << 4); // OC1M PWM1
        write_word(&mut t, T1 + 0x20, super::CC1E | (1 << 2)); // CC1E|CC1NE
                                                               // BDTR: MOE | DTG=3 (dead time 3 ticks)
        write_word(&mut t, T1 + super::BDTR, (1 << 15) | 3);
        write_word(&mut t, T1, CEN);
        // Run past the compare so OC1 goes high then wraps low.
        for _ in 0..12 {
            t.tick(1);
            let _ = read_word(&mut t, T1 + 0x10);
        }
        // After the run both outputs settled: OC1N is the inverse of OC1.
        let oc1 = t.output_level(1);
        assert_eq!(t.output_level_n(1), !oc1, "OC1N inverted after gap");
        // MOE clear forces the complementary output inactive.
        write_word(&mut t, T1 + super::BDTR, 3);
        t.tick(1);
        assert!(!t.output_level_n(1), "MOE=0 -> OC1N inactive");
    }
}
