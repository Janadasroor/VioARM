//! TI Tiva C pulse-width modulators (`PWM0`-`PWM1`).
//!
//! Generator counting and output actions follow the TM4C123GH6PM
//! datasheet (SPMS376E §20): four generators per module at `0x040+0x40n`
//! (`CTL`/`INTEN`/`RIS`/`ISC`/`LOAD`/`COUNT`/`CMPA`/`CMPB`/`GENA`/`GENB`),
//! count-down (`MODE=0`, `LOAD`→0, wrap) or count-up/down (`MODE=1`,
//! 0→`LOAD`→0), compare matches with zero/load-over-compare and
//! compare-A-over-B priority, `GENA`/`GENB` actions (nothing/invert/low/
//! high, up-phase fields while counting up) applied per event, and
//! zero/load/compare interrupt latches with `INTEN` gating and `ISC` W1C
//! clear. `ENABLE` starts the counter from `LOAD` (down) or 0 (up/down).
//!
//! The SVD only carries module-level registers, so this is a hand-decoded
//! window (like the GPIO): generator blocks, dead-band, fault sources,
//! `MINFLTPER`, sync, triggers, and update-mode staging are latched words
//! (update modes always immediate; documented). Output levels are not
//! readable on silicon — [`Pwm::output`] exposes them as host
//! introspection (evaluated only while the generator and `PWMENABLE` bit
//! are set, else 0; `PWMINVERT` applied). Each generator raises its own
//! SVD line (`PWMn_0`-`PWMn_3`); the fault line never fires (documented).

use crate::device::svd::{Device, PeripheralDef};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

// Module offsets (in the SVD table).
const PWMENABLE_OFF: u32 = 0x008;
const PWMINVERT_OFF: u32 = 0x00C;
// Generator block stride and field offsets (datasheet Table 20-2 p1240).
const GEN_STRIDE: u32 = 0x040;
const GEN_BASE: u32 = 0x040;
const GCTL_OFF: u32 = 0x00;
const GINTEN_OFF: u32 = 0x04;
const GRIS_OFF: u32 = 0x08;
const GISC_OFF: u32 = 0x0C;
const GLOAD_OFF: u32 = 0x10;
const GCOUNT_OFF: u32 = 0x14;
const GCMPA_OFF: u32 = 0x18;
const GCMPB_OFF: u32 = 0x1C;
const GGENA_OFF: u32 = 0x20;
const GGENB_OFF: u32 = 0x24;
/// `CTL.ENABLE` bit 0, `CTL.MODE` bit 1.
const CTL_ENABLE: u32 = 1 << 0;
const CTL_MODE: u32 = 1 << 1;
/// Interrupt bits: zero 0, load 1, cmpA-up 2, cmpA-down 3, cmpB-up 4,
/// cmpB-down 5.
const INT_ZERO: u32 = 1 << 0;
const INT_LOAD: u32 = 1 << 1;
const INT_CMPAU: u32 = 1 << 2;
const INT_CMPAD: u32 = 1 << 3;
const INT_CMPBU: u32 = 1 << 4;
const INT_CMPBD: u32 = 1 << 5;
/// Generator actions.
const ACT_NOTHING: u32 = 0;
const ACT_INVERT: u32 = 1;
const ACT_LOW: u32 = 2;
const ACT_HIGH: u32 = 3;

/// One generator's live state.
#[derive(Clone, Copy, Debug, Default)]
struct GenState {
    count: u16,
    /// Count direction for up/down mode (`true` = up).
    going_up: bool,
    out_a: bool,
    out_b: bool,
    ris: u32,
}

/// Shared module state: latched words plus four generators.
#[derive(Debug)]
pub struct PwmCore {
    words: std::collections::HashMap<u32, u32>,
    gens: [GenState; 4],
    /// Cached per-generator `(RIS & INTEN) != 0` bitmask.
    pending: u32,
}

/// Behavior behind one PWM module window.
pub struct PwmBehavior {
    def: &'static PeripheralDef,
    shared: Arc<Mutex<PwmCore>>,
}

impl PwmBehavior {
    fn gen_block(gen: usize) -> u32 {
        GEN_BASE + gen as u32 * GEN_STRIDE
    }

    fn is_gen_reg(offset: u32) -> Option<(usize, u32)> {
        if offset >= GEN_BASE && offset < GEN_BASE + 4 * GEN_STRIDE {
            let gen = ((offset - GEN_BASE) / GEN_STRIDE) as usize;
            Some((gen, offset - (GEN_BASE + gen as u32 * GEN_STRIDE)))
        } else {
            None
        }
    }

    fn apply_action(out: bool, act: u32) -> bool {
        match act {
            ACT_INVERT => !out,
            ACT_LOW => false,
            ACT_HIGH => true,
            _ => out,
        }
    }

    /// Evaluate one counter step for generator `gen` (datasheet §20.3.4
    /// event/priority rules).
    fn step_gen(&self, core: &mut PwmCore, gen: usize) {
        let base = Self::gen_block(gen);
        let get = |off: u32| core.words.get(&off).copied().unwrap_or(0);
        let ctl = get(base + GCTL_OFF);
        if ctl & CTL_ENABLE == 0 {
            return;
        }
        let up_down = ctl & CTL_MODE != 0;
        let load = (get(base + GLOAD_OFF) & 0xFFFF) as u16;
        let cmpa = (get(base + GCMPA_OFF) & 0xFFFF) as u16;
        let cmpb = (get(base + GCMPB_OFF) & 0xFFFF) as u16;
        let gena = get(base + GGENA_OFF);
        let genb = get(base + GGENB_OFF);
        let g = &mut core.gens[gen];
        // (zero, load, cmpA, cmpB) events this step.
        let (mut ev_zero, mut ev_load, mut ev_cmpa, mut ev_cmpb) = (false, false, false, false);
        if up_down {
            if g.going_up {
                g.count = g.count.wrapping_add(1);
                if g.count >= load {
                    g.count = load;
                    ev_load = true;
                    g.going_up = false;
                }
                if g.count == cmpa {
                    ev_cmpa = true;
                }
                if g.count == cmpb {
                    ev_cmpb = true;
                }
            } else if g.count == 0 {
                ev_zero = true;
                g.going_up = true;
                g.count = 1;
            } else {
                g.count -= 1;
                if g.count == 0 {
                    ev_zero = true;
                    g.going_up = true;
                }
                if g.count == cmpa {
                    ev_cmpa = true;
                }
                if g.count == cmpb {
                    ev_cmpb = true;
                }
            }
        } else {
            if g.count == 0 {
                ev_zero = true;
                ev_load = true;
                g.count = load;
            } else {
                g.count -= 1;
                if g.count == 0 {
                    ev_zero = true;
                }
            }
            if g.count == cmpa {
                ev_cmpa = true;
            }
            if g.count == cmpb {
                ev_cmpb = true;
            }
        }
        // Actions with zero/load > compare-A > compare-B priority, using
        // the up-phase fields (ACTCMPAU/ACTCMPBU) while counting up.
        // Zero/load actions: ACTZERO bits 1:0, ACTLOAD bits 3:2.
        let za_zero = gena & 0x03;
        let za_load = (gena >> 2) & 0x03;
        let zb_zero = genb & 0x03;
        let zb_load = (genb >> 2) & 0x03;
        if ev_zero {
            g.out_a = Self::apply_action(g.out_a, za_zero);
            g.out_b = Self::apply_action(g.out_b, zb_zero);
            g.ris |= INT_ZERO;
        }
        if ev_load {
            g.out_a = Self::apply_action(g.out_a, za_load);
            g.out_b = Self::apply_action(g.out_b, zb_load);
            g.ris |= INT_LOAD;
        }
        // Compare actions (up-phase uses ACTCMPAU/ACTCMPBU).
        let up = up_down && g.going_up;
        let (caa, cba) = if up {
            ((gena >> 4) & 0x03, (gena >> 8) & 0x03)
        } else {
            ((gena >> 6) & 0x03, (gena >> 10) & 0x03)
        };
        let (cab, cbb) = if up {
            ((genb >> 4) & 0x03, (genb >> 8) & 0x03)
        } else {
            ((genb >> 6) & 0x03, (genb >> 10) & 0x03)
        };
        if ev_cmpa && !ev_zero && !ev_load {
            g.out_a = Self::apply_action(g.out_a, caa);
            g.out_b = Self::apply_action(g.out_b, cab);
            g.ris |= if up { INT_CMPAU } else { INT_CMPAD };
        }
        if ev_cmpb && !ev_zero && !ev_load && !ev_cmpa {
            g.out_a = Self::apply_action(g.out_a, cba);
            g.out_b = Self::apply_action(g.out_b, cbb);
            g.ris |= if up { INT_CMPBU } else { INT_CMPBD };
        }
        let inten = get(base + GINTEN_OFF);
        let mut pending = 0u32;
        for (i, gg) in core.gens.iter().enumerate() {
            let ie = core
                .words
                .get(&(Self::gen_block(i) + GINTEN_OFF))
                .copied()
                .unwrap_or(0);
            if gg.ris & ie & 0x3F != 0 {
                pending |= 1 << i;
            }
        }
        core.pending = pending;
        let _ = inten;
    }
}

/// A Tiva C PWM module (four generators).
pub struct Pwm {
    ranges: [Region; 1],
    exceptions: [Option<u8>; 5],
    inner: Mutex<PwmBehavior>,
}

impl Pwm {
    /// A new PWM module on `D`'s tables (`"PWM0"`/`"PWM1"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain PWM");
        let mut exceptions = [None; 5];
        for irq in def.irqs {
            // `PWMn_0`-`PWMn_3` index the generators, `PWMn_FAULT` last.
            let idx = if irq.name.ends_with("_FAULT") {
                4
            } else {
                irq.name
                    .rsplit('_')
                    .next()
                    .and_then(|s| s.parse::<usize>().ok())
                    .unwrap_or(4)
            };
            if idx < 5 {
                exceptions[idx] = Some(16 + irq.number);
            }
        }
        Self {
            ranges: [Region::new(def.base, def.window)],
            exceptions,
            inner: Mutex::new(PwmBehavior {
                def,
                shared: Arc::new(Mutex::new(PwmCore {
                    words: std::collections::HashMap::new(),
                    gens: [GenState::default(); 4],
                    pending: 0,
                })),
            }),
        }
    }

    /// Output levels `(pwmA, pwmB)` for generator `gen` (host
    /// introspection; not readable on silicon). Gated by the generator
    /// `ENABLE` and `PWMENABLE` bits with `PWMINVERT` applied.
    #[must_use]
    pub fn output(&self, gen: usize) -> (bool, bool) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        if gen >= 4 {
            return (false, false);
        }
        let base = PwmBehavior::gen_block(gen);
        let get = |off: u32| core.words.get(&off).copied().unwrap_or(0);
        if get(base + GCTL_OFF) & CTL_ENABLE == 0 {
            return (false, false);
        }
        let en = get(PWMENABLE_OFF);
        let inv = get(PWMINVERT_OFF);
        let g = core.gens[gen];
        let (mut a, mut bb) = (g.out_a, g.out_b);
        if en & (1 << (2 * gen)) == 0 {
            a = false;
        }
        if en & (1 << (2 * gen + 1)) == 0 {
            bb = false;
        }
        if inv & (1 << (2 * gen)) != 0 {
            a = !a;
        }
        if inv & (1 << (2 * gen + 1)) != 0 {
            bb = !bb;
        }
        (a, bb)
    }

    fn offset_in_window(&self, addr: u32) -> Result<u32, BusError> {
        let region = self.ranges[0];
        addr.checked_sub(region.base)
            .filter(|o| *o < region.len)
            .ok_or(BusError::Reserved)
    }

    /// Advance all enabled generators by `delta` device ticks.
    pub fn tick(&mut self, delta: u64) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        let mut steps = delta;
        while steps > 0 {
            steps -= 1;
            for gen in 0..4 {
                b.step_gen(&mut core, gen);
            }
        }
    }
}

impl Peripheral for Pwm {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let off = self.offset_in_window(addr)?;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        // Module registers come from the SVD table when present…
        let in_table = b.def.registers.iter().any(|r| {
            let start = r.offset;
            off >= start && off < start + 4
        });
        let word = if let Some((gen, field)) = PwmBehavior::is_gen_reg(off & !3) {
            let base = PwmBehavior::gen_block(gen);
            match field {
                GCOUNT_OFF => u32::from(core.gens[gen].count),
                GRIS_OFF => core.gens[gen].ris,
                _ => core.words.get(&(off & !3)).copied().unwrap_or(0),
            }
        } else if in_table || off == 0x28 {
            // …plus `ENUPD` (in the SVD gap); generator state otherwise.
            if off == 0x28 {
                core.words.get(&0x28).copied().unwrap_or(0)
            } else {
                let aligned = off & !3;
                match aligned {
                    0x18 => {
                        // Module `RIS`: fault lines only (generators have
                        // their own); never fires here.
                        0
                    }
                    0x20 => {
                        // `STATUS`: fault flags, all clear.
                        0
                    }
                    _ => core.words.get(&aligned).copied().unwrap_or(0),
                }
            }
        } else {
            return Err(BusError::Reserved);
        };
        Ok(((word >> (8 * (off % 4))) & 0xFF) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let off = self.offset_in_window(addr)?;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        let lane = off % 4;
        let shift = 8 * lane;
        if let Some((gen, field)) = PwmBehavior::is_gen_reg(off & !3) {
            let base = PwmBehavior::gen_block(gen);
            match field {
                GCOUNT_OFF | GRIS_OFF => {}
                GISC_OFF => {
                    let cur = core.gens[gen].ris;
                    let clear = (u32::from(val) << shift) & 0x3F;
                    core.gens[gen].ris = cur & !clear;
                }
                GCTL_OFF => {
                    let cur = core.words.get(&(base + GCTL_OFF)).copied().unwrap_or(0);
                    let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
                    let was = cur & CTL_ENABLE != 0;
                    core.words.insert(base + GCTL_OFF, word);
                    if word & CTL_ENABLE != 0 && !was {
                        // Enable: reload down-counters from LOAD,
                        // up/down counters from 0.
                        let load = (core.words.get(&(base + GLOAD_OFF)).copied().unwrap_or(0)
                            & 0xFFFF) as u16;
                        let g = &mut core.gens[gen];
                        if word & CTL_MODE != 0 {
                            g.count = 0;
                            g.going_up = true;
                        } else {
                            g.count = load;
                        }
                    }
                }
                _ => {
                    let cur = core.words.get(&(base + field)).copied().unwrap_or(0);
                    core.words.insert(
                        base + field,
                        (cur & !(0xFF << shift)) | (u32::from(val) << shift),
                    );
                }
            }
            // Refresh this generator's pending bit.
            let ie = core.words.get(&(base + GINTEN_OFF)).copied().unwrap_or(0);
            let mut pending = 0u32;
            for (i, gg) in core.gens.iter().enumerate() {
                let ie = core
                    .words
                    .get(&(PwmBehavior::gen_block(i) + GINTEN_OFF))
                    .copied()
                    .unwrap_or(0);
                if gg.ris & ie & 0x3F != 0 {
                    pending |= 1 << i;
                }
            }
            core.pending = pending;
            let _ = ie;
            return Ok(());
        }
        // Module window: SVD-table words latch, except RO `RIS`/`STATUS`.
        let in_table = b.def.registers.iter().any(|r| {
            let start = r.offset;
            off >= start && off < start + 4
        });
        if !in_table && off != 0x28 {
            return Err(BusError::Reserved);
        }
        let aligned = off & !3;
        if matches!(aligned, 0x18 | 0x20) {
            return Ok(());
        }
        if aligned == 0x1C {
            // Module `ISC`: fault clears only; generator faults never set.
            return Ok(());
        }
        let cur = core.words.get(&aligned).copied().unwrap_or(0);
        core.words.insert(
            aligned,
            (cur & !(0xFF << shift)) | (u32::from(val) << shift),
        );
        Ok(())
    }

    fn reset(&mut self) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        core.words.clear();
        core.gens = [GenState::default(); 4];
        core.pending = 0;
    }

    fn tick(&mut self, delta: u64) {
        Pwm::tick(self, delta);
    }
}

impl InterruptSource for Pwm {
    fn exception_number(&self) -> Option<u8> {
        self.exceptions[0]
    }

    fn exception_for_line(&self, line: u32) -> Option<u8> {
        self.exceptions.get(line as usize).copied().flatten()
    }

    fn lines(&self) -> u32 {
        4
    }

    fn pending(&self) -> u32 {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        core.pending
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PWM0_BASE: u32 = 0x4002_8000;
    /// Generator 0 field base.
    const G0: u32 = GEN_BASE;

    fn pwm0() -> Pwm {
        Pwm::for_device::<crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm>("PWM0")
    }

    fn read_word(pwm: &mut Pwm, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(pwm.read(PWM0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(pwm: &mut Pwm, offset: u32, value: u32) {
        for lane in 0..4 {
            pwm.write(
                PWM0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Classic down-count PWM: `GENA = HIGH on LOAD, LOW on CMPAD`.
    /// `LOAD=100`, `CMPA=25` → 75% duty on `pwm0A`.
    fn setup_pwm(pwm: &mut Pwm) {
        write_word(pwm, G0 + GLOAD_OFF, 100);
        write_word(pwm, G0 + GCMPA_OFF, 25);
        // ACTLOAD=HIGH (bits 3:2), ACTCMPAD=LOW (bits 7:6).
        write_word(pwm, G0 + GGENA_OFF, (ACT_HIGH << 2) | (ACT_LOW << 6));
        write_word(pwm, G0 + GCTL_OFF, CTL_ENABLE);
        write_word(pwm, PWMENABLE_OFF, 0x01);
    }

    /// Duty cycle: HIGH from LOAD down to CMPA, LOW below.
    #[test]
    fn down_count_duty_cycle() {
        let pwm = pwm0();
        let mut pwm = pwm;
        setup_pwm(&mut pwm);
        // Fresh enable: counter at LOAD, output LOW until first LOAD event.
        assert_eq!(pwm.output(0), (false, false));
        pwm.tick(1); // 100 → 99
        assert_eq!(read_word(&mut pwm, G0 + GCOUNT_OFF), 99);
        pwm.tick(74); // → 25: compare match drives LOW (already LOW).
        assert!(!pwm.output(0).0);
        pwm.tick(25); // → 0: zero event; wrap reloads 100 next step.
        pwm.tick(1); // wrap: LOAD event drives HIGH.
        assert!(pwm.output(0).0);
        pwm.tick(75); // 100 → 25: compare drives LOW again.
        assert!(!pwm.output(0).0);
    }

    /// Zero/load/compare interrupts latch, gate by `INTEN`, clear by `ISC`.
    #[test]
    fn interrupt_flow() {
        let mut pwm = pwm0();
        setup_pwm(&mut pwm);
        write_word(&mut pwm, G0 + GINTEN_OFF, INT_ZERO | INT_LOAD | INT_CMPAD);
        pwm.tick(100); // 100 → 0: compare at 25, zero at 0.
        let ris = read_word(&mut pwm, G0 + GRIS_OFF);
        assert_eq!(ris & (INT_ZERO | INT_CMPAD), INT_ZERO | INT_CMPAD);
        assert_eq!(pwm.pending() & 0x01, 0x01);
        assert_eq!(pwm.exception_number(), Some(26)); // PWM0_0 = IRQ 10.
        assert_eq!(pwm.exception_for_line(1), Some(27)); // PWM0_1 = IRQ 11.
                                                         // Clear zero only; compare stays latched.
        write_word(&mut pwm, G0 + GISC_OFF, INT_ZERO);
        let ris = read_word(&mut pwm, G0 + GRIS_OFF);
        assert_eq!(ris & INT_ZERO, 0);
        assert_eq!(ris & INT_CMPAD, INT_CMPAD);
        write_word(&mut pwm, G0 + GISC_OFF, INT_CMPAD);
        assert_eq!(pwm.pending() & 0x01, 0);
    }

    /// Disabled generator holds its counter; outputs gate off.
    #[test]
    fn disabled_holds_and_gates() {
        let mut pwm = pwm0();
        setup_pwm(&mut pwm);
        write_word(&mut pwm, G0 + GCTL_OFF, 0x00);
        pwm.tick(1000);
        assert_eq!(read_word(&mut pwm, G0 + GCOUNT_OFF), 100);
        write_word(&mut pwm, PWMENABLE_OFF, 0x00);
        write_word(&mut pwm, G0 + GCTL_OFF, CTL_ENABLE);
        pwm.tick(200);
        assert_eq!(pwm.output(0), (false, false)); // ENABLE bit clear.
    }

    /// Up/down mode wraps at LOAD and returns to zero with both events.
    #[test]
    fn up_down_wraps() {
        let mut pwm = pwm0();
        write_word(&mut pwm, G0 + GLOAD_OFF, 4);
        write_word(&mut pwm, G0 + GINTEN_OFF, INT_ZERO | INT_LOAD);
        write_word(&mut pwm, G0 + GCTL_OFF, CTL_ENABLE | CTL_MODE);
        pwm.tick(4); // 0 → 4: load event at top.
        assert_eq!(read_word(&mut pwm, G0 + GRIS_OFF) & INT_LOAD, INT_LOAD);
        pwm.tick(4); // 4 → 0: zero event at bottom.
        assert_eq!(read_word(&mut pwm, G0 + GRIS_OFF) & INT_ZERO, INT_ZERO);
    }

    /// Module `RIS`/`STATUS` stay clear (faults never fire here).
    #[test]
    fn module_fault_lines_quiet() {
        let mut pwm = pwm0();
        setup_pwm(&mut pwm);
        pwm.tick(500);
        assert_eq!(read_word(&mut pwm, 0x18), 0);
        assert_eq!(read_word(&mut pwm, 0x20), 0);
    }
}
