//! FMAC filter math accelerator (G4 `FMAC@0x40021400`) — polling FIR/IIR.
//!
//! Function encodings and threshold/parameter ranges follow the HAL
//! (`LOAD_X1` 1, `LOAD_X2` 2, `LOAD_Y` 3, `CONVO_FIR` 8, `IIR_DF1` 9).
//! Buffers live in the model (sizes from `X1/X2/YBUFCFG`, watermarks
//! 1/2/4/8): loads append (`OVFL` when full), `START` arms a filter run
//! (clearing outputs and histories), and each `WDATA` input shifts one
//! sample through — producing one output per input once `P` samples are
//! present. FIR convolves `P` taps from `X2`; IIR direct-form-1 adds `Q`
//! feedback taps from `X2[P..]` over the output history. All math is
//! Q1.15 (`acc >> (15 + R)`); with `CLIPEN` out-of-range outputs
//! saturate (latching `SAT`), otherwise the low 16 bits land.
//! `RIEN`/`WIEN` gate output-ready/input-room lines, `OVFLIEN`/
//! `UNFLIEN`/`SATIEN` the error lines. Latched error flags (`OVFL`/
//! `UNFL`/`SAT`) clear only on `RESET` (self-clearing) or device reset —
//! like silicon, where the HAL recovers via DeInit/Init and a masked
//! line keeps its status. `DMAREN`/`DMAWEN` latch (no DMA engine).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;

const X1BUFCFG: u32 = 0x00;
const X2BUFCFG: u32 = 0x04;
const YBUFCFG: u32 = 0x08;
const PARAM: u32 = 0x0C;
const CR: u32 = 0x10;
const SR: u32 = 0x14;
const WDATA: u32 = 0x18;
const RDATA: u32 = 0x1C;

const PARAM_P_SHIFT: u32 = 0;
const PARAM_Q_SHIFT: u32 = 8;
const PARAM_R_SHIFT: u32 = 16;
const PARAM_FUNC_SHIFT: u32 = 24;
const PARAM_START: u32 = 1 << 31;

const FUNC_LOAD_X1: u32 = 1;
const FUNC_LOAD_X2: u32 = 2;
const FUNC_LOAD_Y: u32 = 3;
const FUNC_FIR: u32 = 8;
const FUNC_IIR: u32 = 9;

const CR_RESET: u32 = 1 << 16;
const CR_CLIPEN: u32 = 1 << 15;
const CR_SATIEN: u32 = 1 << 4;
const CR_UNFLIEN: u32 = 1 << 3;
const CR_OVFLIEN: u32 = 1 << 2;
const CR_WIEN: u32 = 1 << 1;
const CR_RIEN: u32 = 1 << 0;

const SR_YEMPTY: u32 = 1 << 0;
const SR_X1FULL: u32 = 1 << 1;
const SR_OVFL: u32 = 1 << 8;
const SR_UNFL: u32 = 1 << 9;
const SR_SAT: u32 = 1 << 10;

const WM_N: [usize; 4] = [1, 2, 4, 8];

#[derive(Clone, Debug, Default)]
struct FmacBehavior {
    x1: VecDeque<i16>,
    x2: Vec<i16>,
    y: VecDeque<i16>,
    x_hist: VecDeque<i16>,
    y_hist: VecDeque<i16>,
    started: bool,
    w_acc: u32,
    w_mask: u8,
    r_word: u32,
    r_lane: u8,
}

impl FmacBehavior {
    fn buf_size(view: &Registers<&mut [u32]>, off: u32) -> usize {
        let n = ((view.get(off) >> 8) & 0xFF) as usize;
        if n == 0 {
            256
        } else {
            n
        }
    }

    fn wm(view: &Registers<&mut [u32]>, off: u32) -> usize {
        WM_N[((view.get(off) >> 24) & 0x3) as usize]
    }

    fn func(view: &Registers<&mut [u32]>) -> u32 {
        (view.get(PARAM) >> PARAM_FUNC_SHIFT) & 0x7F
    }

    fn params(view: &Registers<&mut [u32]>) -> (usize, usize, u32) {
        let p = view.get(PARAM);
        (
            ((p >> PARAM_P_SHIFT) & 0xFF) as usize,
            ((p >> PARAM_Q_SHIFT) & 0xFF) as usize,
            (p >> PARAM_R_SHIFT) & 0x7,
        )
    }

    fn set_sr(view: &mut Registers<&mut [u32]>, bits: u32) {
        view.set(SR, view.get(SR) | bits);
    }

    fn update_flags(&self, view: &mut Registers<&mut [u32]>) {
        // Watermarked FULL/EMPTY; latched error bits are sticky elsewhere.
        let free = Self::buf_size(view, X1BUFCFG).saturating_sub(self.x1.len());
        let mut sr = view.get(SR) & !(SR_YEMPTY | SR_X1FULL);
        if free < Self::wm(view, X1BUFCFG) {
            sr |= SR_X1FULL;
        }
        if self.y.len() < Self::wm(view, YBUFCFG) {
            sr |= SR_YEMPTY;
        }
        view.set(SR, sr);
    }

    fn push_output(&mut self, view: &mut Registers<&mut [u32]>, acc: i64) {
        let (_, _, r) = Self::params(view);
        let v = acc >> (15 + r);
        let out = if view.get(CR) & CR_CLIPEN != 0 {
            let clipped = v.clamp(-0x8000, 0x7FFF);
            if clipped != v {
                Self::set_sr(view, SR_SAT);
            }
            clipped
        } else {
            v & 0xFFFF
        };
        self.y.push_back(out as i16);
        self.update_flags(view);
    }

    fn fir_step(&mut self, view: &mut Registers<&mut [u32]>, sample: i16) {
        let (p, _, _) = Self::params(view);
        self.x_hist.push_back(sample);
        if self.x_hist.len() < p.max(1) {
            return;
        }
        while self.x_hist.len() > p.max(1) {
            self.x_hist.pop_front();
        }
        let mut acc = 0i64;
        for (i, s) in self.x_hist.iter().rev().take(p).enumerate() {
            let h = self.x2.get(i).copied().unwrap_or(0) as i64;
            acc += i64::from(*s) * h;
        }
        self.push_output(view, acc);
    }

    fn iir_step(&mut self, view: &mut Registers<&mut [u32]>, sample: i16) {
        let (p, q, _) = Self::params(view);
        self.x_hist.push_back(sample);
        while self.x_hist.len() > p.max(1) {
            self.x_hist.pop_front();
        }
        if self.x_hist.len() < p.max(1) {
            return;
        }
        let mut acc = 0i64;
        for (i, s) in self.x_hist.iter().rev().take(p).enumerate() {
            let b = self.x2.get(i).copied().unwrap_or(0) as i64;
            acc += i64::from(*s) * b;
        }
        for (j, s) in self.y_hist.iter().rev().take(q).enumerate() {
            let a = self.x2.get(p + j).copied().unwrap_or(0) as i64;
            acc += i64::from(*s) * a;
        }
        self.push_output(view, acc);
        if let Some(last) = self.y.back().copied() {
            self.y_hist.push_back(last);
            while self.y_hist.len() > q.max(1) {
                self.y_hist.pop_front();
            }
        }
    }
}

impl Behavior for FmacBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            CR => {
                let new = view.merged(CR, val, shift);
                if new & CR_RESET != 0 {
                    self.x1.clear();
                    self.x2.clear();
                    self.y.clear();
                    self.x_hist.clear();
                    self.y_hist.clear();
                    self.started = false;
                    view.set(SR, 0);
                    self.update_flags(view);
                    return new & !CR_RESET;
                }
                new
            }
            PARAM => {
                let new = view.merged(PARAM, val, shift);
                if new & PARAM_START != 0 {
                    self.started = true;
                    self.y.clear();
                    self.x_hist.clear();
                    self.y_hist.clear();
                    self.update_flags(view);
                }
                new
            }
            WDATA => {
                // Lanes assemble the low-halfword sample; the low pair
                // completing (word or halfword stores) consumes it.
                self.w_acc |= u32::from(val) << shift;
                self.w_mask |= 1 << (shift / 8);
                if self.w_mask & 0x3 == 0x3 {
                    let sample = (self.w_acc & 0xFFFF) as u16 as i16;
                    self.w_acc = 0;
                    self.w_mask = 0;
                    match Self::func(view) {
                        FUNC_LOAD_X1 => {
                            if self.x1.len() < Self::buf_size(view, X1BUFCFG) {
                                self.x1.push_back(sample);
                            } else {
                                Self::set_sr(view, SR_OVFL);
                            }
                            self.update_flags(view);
                        }
                        FUNC_LOAD_X2 => {
                            if self.x2.len() < Self::buf_size(view, X2BUFCFG) {
                                self.x2.push(sample);
                            } else {
                                Self::set_sr(view, SR_OVFL);
                            }
                        }
                        FUNC_LOAD_Y => {
                            self.y_hist.push_back(sample);
                        }
                        FUNC_FIR if self.started => self.fir_step(view, sample),
                        FUNC_IIR if self.started => self.iir_step(view, sample),
                        _ => {}
                    }
                }
                view.merged(offset, val, shift)
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            SR => {
                self.update_flags(view);
                view.get(SR)
            }
            RDATA => {
                // One pop per word read; lanes share the staged word.
                if self.r_lane == 0 {
                    match self.y.pop_front() {
                        Some(w) => self.r_word = u32::from(w as u16),
                        None => {
                            Self::set_sr(view, SR_UNFL);
                            self.r_word = 0;
                        }
                    }
                    self.update_flags(view);
                }
                let w = self.r_word;
                self.r_lane += 1;
                if self.r_lane == 4 {
                    self.r_lane = 0;
                }
                w
            }
            _ => value,
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let cr = view.get(CR);
        let sr = view.get(SR);
        let ri = cr & CR_RIEN != 0 && sr & SR_YEMPTY == 0;
        let wi = cr & CR_WIEN != 0 && sr & SR_X1FULL == 0;
        let ov = cr & CR_OVFLIEN != 0 && sr & SR_OVFL != 0;
        let un = cr & CR_UNFLIEN != 0 && sr & SR_UNFL != 0;
        let st = cr & CR_SATIEN != 0 && sr & SR_SAT != 0;
        u32::from(ri || wi || ov || un || st)
    }

    fn reset(&mut self) {
        self.x1.clear();
        self.x2.clear();
        self.y.clear();
        self.x_hist.clear();
        self.y_hist.clear();
        self.started = false;
        self.w_acc = 0;
        self.w_mask = 0;
        self.r_word = 0;
        self.r_lane = 0;
    }
}

/// FMAC filter math accelerator.
#[derive(Debug)]
pub struct Fmac {
    inner: RegisterFile<FmacBehavior>,
}

impl Fmac {
    /// A new `FMAC` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `FMAC`
    /// peripheral.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("FMAC").expect("generated tables contain FMAC");
        Self {
            inner: RegisterFile::new(def, FmacBehavior::default()),
        }
    }
}

impl Peripheral for Fmac {
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
        "fmac"
    }
}

impl InterruptSource for Fmac {
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
    use crate::device::svd::stm32g474xx::Stm32g474;
    use crate::device::{InterruptSource, Peripheral};

    const BASE: u32 = 0x4002_1400;
    const X1BUFCFG_O: u32 = 0x00;
    const X2BUFCFG_O: u32 = 0x04;
    const PARAM_O: u32 = 0x0C;
    const CR_O: u32 = 0x10;
    const SR_O: u32 = 0x14;
    const WDATA_O: u32 = 0x18;
    const RDATA_O: u32 = 0x1C;

    #[allow(clippy::cast_possible_truncation)]
    fn w(f: &mut Fmac, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            f.write(BASE + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn r(f: &mut Fmac, off: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = f.read(BASE + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn fmac() -> Fmac {
        let mut f = Fmac::new_for::<Stm32g474>();
        f.reset();
        f
    }

    /// Program buffer sizes (8 in / 8 coeff / 8 out) and `func`+`P/Q/R`.
    fn setup(f: &mut Fmac, func: u32, p: u32, q: u32, r: u32) {
        w(f, X1BUFCFG_O, 8 << 8);
        w(f, X2BUFCFG_O, 8 << 8);
        w(f, PARAM_O, (func << 24) | (r << 16) | (q << 8) | p);
    }

    #[test]
    fn fir_moving_average() {
        let mut f = fmac();
        // y[n] = 0.5·x[n] + 0.5·x[n-1].
        setup(&mut f, FUNC_LOAD_X2, 0, 0, 0);
        w(&mut f, WDATA_O, 0x4000);
        w(&mut f, WDATA_O, 0x4000);
        w(&mut f, PARAM_O, (FUNC_FIR << 24) | 2 | (1 << 31));
        w(&mut f, WDATA_O, 0x4000);
        w(&mut f, WDATA_O, 0x4000);
        assert_eq!(r(&mut f, RDATA_O) & 0xFFFF, 0x4000, "(0.5+0.5)/2");
        w(&mut f, WDATA_O, 0x0000);
        assert_eq!(r(&mut f, RDATA_O) & 0xFFFF, 0x2000, "(0.5+0)/2");
    }

    #[test]
    fn iir_first_order() {
        let mut f = fmac();
        // y[n] = 0.5·x[n] + 0.5·y[n-1].
        setup(&mut f, FUNC_LOAD_X2, 0, 0, 0);
        w(&mut f, WDATA_O, 0x4000); // b0
        w(&mut f, WDATA_O, 0x4000); // a0
        setup(&mut f, FUNC_IIR, 1, 1, 0);
        w(&mut f, PARAM_O, (FUNC_IIR << 24) | 1 | (1 << 8) | (1 << 31));
        w(&mut f, WDATA_O, 0x4000);
        assert_eq!(r(&mut f, RDATA_O) & 0xFFFF, 0x2000, "y0 = 0.25");
        w(&mut f, WDATA_O, 0x0000);
        assert_eq!(r(&mut f, RDATA_O) & 0xFFFF, 0x1000, "y1 = 0.125");
    }

    #[test]
    fn clip_sat_and_truncate() {
        let mut f = fmac();
        setup(&mut f, FUNC_LOAD_X2, 0, 0, 0);
        w(&mut f, WDATA_O, 0x7FFF);
        w(&mut f, WDATA_O, 0x7FFF);
        setup(&mut f, FUNC_FIR, 2, 0, 0);
        w(&mut f, CR_O, CR_CLIPEN);
        w(&mut f, PARAM_O, (FUNC_FIR << 24) | 2 | (1 << 31));
        w(&mut f, WDATA_O, 0x7FFF);
        w(&mut f, WDATA_O, 0x7FFF);
        assert_eq!(r(&mut f, RDATA_O) & 0xFFFF, 0x7FFF, "clipped to max");
        assert_ne!(r(&mut f, SR_O) & SR_SAT, 0, "SAT latches");
        // Without clipping the low halfword lands instead.
        let mut f = fmac();
        setup(&mut f, FUNC_LOAD_X2, 0, 0, 0);
        w(&mut f, WDATA_O, 0x7FFF);
        w(&mut f, WDATA_O, 0x7FFF);
        setup(&mut f, FUNC_FIR, 2, 0, 0);
        w(&mut f, PARAM_O, (FUNC_FIR << 24) | 2 | (1 << 31));
        w(&mut f, WDATA_O, 0x7FFF);
        w(&mut f, WDATA_O, 0x7FFF);
        assert_eq!(r(&mut f, RDATA_O) & 0xFFFF, 0xFFFC, "truncated");
    }

    #[test]
    fn overflow_and_irq() {
        let mut f = fmac();
        // X1 holds 2 samples; the third overflows.
        w(&mut f, X1BUFCFG_O, (2 << 8) | (3 << 24));
        w(&mut f, PARAM_O, FUNC_LOAD_X1 << 24);
        w(&mut f, WDATA_O, 1);
        w(&mut f, WDATA_O, 2);
        // free = 0 < watermark 8 → FULL.
        assert_ne!(r(&mut f, SR_O) & SR_X1FULL, 0, "X1FULL watermark");
        w(&mut f, WDATA_O, 3);
        assert_ne!(r(&mut f, SR_O) & SR_OVFL, 0, "OVFL latches");
        w(&mut f, CR_O, CR_OVFLIEN | CR_RIEN);
        assert_eq!(f.pending(), 1, "error line raises");
        // No W1C on silicon: the flag persists until RESET.
        w(&mut f, CR_O, CR_RESET);
        assert_eq!(r(&mut f, SR_O) & SR_OVFL, 0, "RESET clears flags");
        assert_eq!(f.pending(), 0, "line drops after reset");
    }

    #[test]
    fn reset_clears() {
        let mut f = fmac();
        setup(&mut f, FUNC_LOAD_X1, 0, 0, 0);
        w(&mut f, WDATA_O, 0x1234);
        w(&mut f, CR_O, CR_RESET);
        assert_eq!(r(&mut f, SR_O) & (SR_OVFL | SR_UNFL | SR_SAT), 0);
        assert_eq!(r(&mut f, RDATA_O) & 0xFFFF, 0, "outputs drained");
        // UNFL latches on the empty read above; reset cleared the rest.
    }
}
