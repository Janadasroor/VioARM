//! OPAMP voltage followers/amplifiers with field-scanned channels.
//!
//! Each `OPAMPn_CSR`/`OPAMPn_CR` register resolves its own enable, mode,
//! and gain bits from the SVD fields (`OPAEN`/`OPAMPn_EN`, `VM_SEL`,
//! `PGA_GAIN`), so F3/G4/H7 layouts share one model: `VM_SEL` 1 follows
//! the shared bus, 0 amplifies it by the `PGA_GAIN` map
//! (`[2,4,8,16,32,64]`, documented contract), 2/3 follow ideally, and the
//! output publishes to the bus (clamped to `VDDA`) on enable, CSR
//! changes, and ticks — republishing only on change, reusing the consumed
//! input across config changes so partial-lane writes converge and a
//! follower never amplifies its own output. There is no ready flag on
//! these IPs (the old `RDY@8` was fiction — that bit is high-speed/trim on real tables).
//! Inputs always read the shared bus (`VP_SEL` ignored, documented);
//! calibration, locking, and timer control stay latches.

use crate::device::analog::AnalogBusState;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// PGA gain map by `PGA_GAIN` value (capped): ×2 … ×64.
const PGA_GAIN_MAP: [u32; 8] = [2, 4, 8, 16, 32, 64, 64, 64];

/// One resolved channel: its CSR offset, field positions, and the last
/// consumed input / published output / config (republish only on change —
/// otherwise a follower amplifies its own output, and partial-lane writes
/// converge once the full word lands).
#[derive(Clone, Copy, Debug)]
struct OpampCh {
    off: u32,
    en: u32,
    mode_shift: u32,
    gain_shift: u32,
    gain_width: u32,
    cfg_mask: u32,
    last_in: Option<u32>,
    last_out: Option<u32>,
    last_cfg: u32,
}

#[derive(Clone, Debug, Default)]
struct OpampBehavior {
    ch: Vec<OpampCh>,
    bus: Option<Arc<Mutex<AnalogBusState>>>,
}

impl OpampBehavior {
    /// Output for channel `c` from input millivolts (clamped to the rail).
    fn output_mv(&self, csr: u32, c: &OpampCh, mv: u32) -> u32 {
        let mode = (csr >> c.mode_shift) & 0x3;
        let gain = if mode == 1 {
            1 // follower (and standalone, ideally)
        } else {
            let v = ((csr >> c.gain_shift) & ((1 << c.gain_width) - 1)) as usize;
            PGA_GAIN_MAP[v.min(7)]
        };
        (mv * gain).min(3300)
    }

    fn publish(&mut self, idx: usize, csr: u32) {
        let c = self.ch[idx];
        let cfg = csr & c.cfg_mask;
        let bus_mv = self
            .bus
            .as_ref()
            .and_then(AnalogBusState::millivolts)
            .unwrap_or(0);
        // Stable (same bus output, same config): skip. A config change
        // under a stable bus reuses the consumed input, so partial-lane
        // writes converge once the full word lands.
        if Some(bus_mv) == c.last_out && cfg == c.last_cfg {
            return;
        }
        let inp = if Some(bus_mv) == c.last_out {
            c.last_in.unwrap_or(bus_mv)
        } else {
            bus_mv
        };
        let out = self.output_mv(csr, &c, inp);
        if let Some(bus) = &self.bus {
            AnalogBusState::set_millivolts(bus, out);
        }
        self.ch[idx].last_in = Some(inp);
        self.ch[idx].last_out = Some(out);
        self.ch[idx].last_cfg = cfg;
    }
}

impl Behavior for OpampBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        // Every write to an enabled channel republishes (converges on
        // full-word writes via the input/config memory below).
        if let Some(idx) = self.ch.iter().position(|c| c.off == offset) {
            if word & self.ch[idx].en != 0 {
                self.publish(idx, word);
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _d: u64) {
        for idx in 0..self.ch.len() {
            let c = self.ch[idx];
            if view.get(c.off) & c.en != 0 {
                self.publish(idx, view.get(c.off));
            }
        }
    }
    fn reset(&mut self) {
        // Channels and bus survive reset (wiring, not state).
    }
}

/// Operational amplifier.
#[derive(Debug)]
pub struct Opamp {
    inner: RegisterFile<OpampBehavior>,
}
impl Opamp {
    /// A new `Opamp` on another chip's device tables, resolving each
    /// `OPAMPn_CSR`/`OPAMPn_CR` channel's enable/mode/gain fields by name
    /// (`F3`/`G4`/`H7` layouts share this scan).
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("OPAMP")
            .or_else(|| D::peripheral("OPAMP1"))
            .expect("OPAMP");
        let mut ch = Vec::new();
        for n in 1..=8u32 {
            let is_csr = |name: &str| {
                name == format!("OPAMP{n}_CSR")
                    || name == format!("OPAMP{n}_CR")
                    || (n == 1 && (name == "CSR" || name == "CR"))
            };
            let Some(reg) = def.registers.iter().find(|r| is_csr(r.name)) else {
                continue;
            };
            let bit = |want: &[&str]| {
                reg.fields
                    .iter()
                    .find(|f| want.iter().any(|w| f.name == *w || f.name.ends_with(w)))
                    .map(|f| (u32::from(f.offset), u32::from(f.width)))
                    .unwrap_or((0, 1))
            };
            let (en, _) = bit(&[&format!("OPAMP{n}_EN"), "OPAEN", "EN"]);
            let (mode, _) = bit(&["VM_SEL"]);
            let (gain, width) = bit(&["PGA_GAIN"]);
            let width = width.max(1);
            let cfg_mask = (1 << en) | (0x3 << mode) | (((1 << width) - 1) << gain);
            ch.push(OpampCh {
                off: reg.offset,
                en: 1 << en,
                mode_shift: mode,
                gain_shift: gain,
                gain_width: width,
                cfg_mask,
                last_in: None,
                last_out: None,
                last_cfg: 0,
            });
        }
        Self {
            inner: RegisterFile::new(
                def,
                OpampBehavior {
                    ch,
                    ..OpampBehavior::default()
                },
            ),
        }
    }

    /// Attach the shared analog bus (boards): outputs publish to it.
    #[must_use]
    pub fn with_bus(mut self, bus: Arc<Mutex<AnalogBusState>>) -> Self {
        self.inner.behavior_mut().bus = Some(bus);
        self
    }
}
impl Peripheral for Opamp {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }
    fn read(&mut self, a: u32) -> Result<u8, BusError> {
        self.inner.read(a)
    }
    fn write(&mut self, a: u32, v: u8) -> Result<(), BusError> {
        self.inner.write(a, v)
    }
    fn reset(&mut self) {
        self.inner.reset();
    }
    fn tick(&mut self, d: u64) {
        self.inner.tick(d);
    }
    fn name(&self) -> &'static str {
        "opamp"
    }
}
impl InterruptSource for Opamp {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, l: u32) {
        self.inner.acknowledge(l);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::stm32g474xx::Stm32g474;
    use crate::device::svd::stm32h743::Stm32h743;
    fn w(c: &mut Opamp, off: u32, v: u32) {
        let b = c.ranges()[0].base;
        for i in 0..4 {
            c.write(b + off + i, ((v >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn r(c: &mut Opamp, off: u32) -> u32 {
        let b = c.ranges()[0].base;
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(c.read(b + off + i).unwrap()) << (8 * i);
        }
        v
    }
    fn opamp(bus_mv: u32) -> (Opamp, Arc<Mutex<AnalogBusState>>) {
        let bus = AnalogBusState::shared();
        AnalogBusState::set_millivolts(&bus, bus_mv);
        let o = Opamp::new_for::<Stm32h743>().with_bus(bus.clone());
        (o, bus)
    }

    #[test]
    fn opamp_enable_sets_no_fictional_ready() {
        // There is no ready flag on these IPs (old `RDY@8` hit real
        // high-speed/trim bits): enabling must not set bit 8.
        let (mut o, _) = opamp(1000);
        w(&mut o, 0x00, 1); // OPAEN
        o.tick(1);
        assert_eq!(r(&mut o, 0x00) & (1 << 8), 0, "no RDY fiction");
    }

    #[test]
    fn opamp_follower_holds_bus() {
        let (mut o, bus) = opamp(1000);
        w(&mut o, 0x00, 1 | (1 << 5)); // EN + follower
        o.tick(1);
        assert_eq!(AnalogBusState::millivolts(&bus), Some(1000));
    }

    #[test]
    fn opamp_pga_doubles_and_clamps() {
        let (mut o, bus) = opamp(800);
        w(&mut o, 0x00, 1); // EN, PGA ×2 (gain 0)
        o.tick(1);
        assert_eq!(AnalogBusState::millivolts(&bus), Some(1600));
        AnalogBusState::set_millivolts(&bus, 2000);
        o.tick(1);
        assert_eq!(AnalogBusState::millivolts(&bus), Some(3300), "rail clamp");
    }

    #[test]
    fn opamp_gain_selects_from_field() {
        let (mut o, bus) = opamp(500);
        w(&mut o, 0x00, 1 | (1 << 14)); // EN + gain 1 (×4)
        o.tick(1);
        assert_eq!(AnalogBusState::millivolts(&bus), Some(2000));
    }

    #[test]
    fn opamp_g474_second_channel_publishes() {
        let bus = AnalogBusState::shared();
        AnalogBusState::set_millivolts(&bus, 700);
        let mut o = Opamp::new_for::<Stm32g474>().with_bus(bus.clone());
        w(&mut o, 0x04, 1 | (1 << 5)); // OPAMP2 follower
        o.tick(1);
        assert_eq!(AnalogBusState::millivolts(&bus), Some(700));
    }
}
