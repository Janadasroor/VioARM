//! The STM32F1 DMA controller (`DMA1`/`DMA2`), driven by the SVD-generated
//! register tables.
//!
//! The register layout is pure SVD; the transfer engine is not, so
//! [`DmaBehavior`] runs it. A channel enabled with `MEM2MEM` set copies
//! `CNDTR` units from `CPAR` to `CMAR` (the reference manual's
//! memory-to-memory direction, `DIR = 0`), honoring `PINC`/`MINC` stepping and
//! the `PSIZE`/`MSIZE` unit widths. A peripheral-request channel (`MEM2MEM`
//! clear) transfers one unit per request-line assertion: a `to_periph` request
//! reads memory (`CMAR`) and writes the peripheral (`CPAR`, e.g. USART TX into
//! `DR`), and a `from_periph` request does the reverse (`CPAR` to `CMAR`, e.g.
//! USART RX out of `DR`). The requests live in a [`DmaRequests`] mailbox shared
//! with the request source (the USART), keyed by `controller * 8 + channel`.
//!
//! The controller has no view of the memory bus, so each unit is queued into a
//! mailbox shared with [`SystemMemory`], which drains it after every device
//! tick. Completion is optimistic for memory-to-memory: the whole transfer
//! lands in the cycle the engine sees the channel enabled. In both modes
//! `CNDTR` counts down, `EN` self-clears at zero, and `TCIF` latches. Enabled
//! channels arbitrate by `CCR.PL` (higher first, then lower channel number,
//! `pl_priority_arbitrates_higher_first`). The controller raises one NVIC line
//! per channel (`DMA1` lines 11..=17, `DMA2` lines 56..=60), so `pending()`
//! reports a per-channel bitmask; the line is asserted when that channel's
//! `TCIE` and `TCIF` are both set.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// One queued transfer unit: read `len` bytes at `src`, write them at `dst`.
pub type DmaUnit = (u32, u32, u8);

/// The peripheral-request lines feeding the DMA controllers, shared with the
/// request sources (currently the USART). Bit `8 * controller + channel` set
/// means the peripheral wants one transfer on that channel, where `channel` is
/// the hardware channel number (1-based: 1..=7 on `DMA1`, 1..=5 on `DMA2`).
/// `to_periph` requests move memory→peripheral; `from_periph` requests move
/// peripheral→memory. A source asserts while it has data to move and the
/// controller clears the bit when it consumes the request.
#[derive(Debug, Clone)]
pub struct DmaRequests {
    /// Memory→peripheral request lines (e.g. USART TX writing `DR`).
    to_periph: u32,
    /// Peripheral→memory request lines (e.g. USART RX reading `DR`).
    from_periph: u32,
}

impl DmaRequests {
    /// A new mailbox with no requests asserted.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            to_periph: 0,
            from_periph: 0,
        }
    }

    /// The request bit for `controller`'s hardware `channel` (1-based).
    fn bit(controller: u8, channel: u8) -> u32 {
        1 << (8 * u32::from(controller) + u32::from(channel))
    }

    /// Assert the request for hardware `channel` on `controller` (both 1-based
    /// where channels are numbered 1..=7 on `DMA1`, 1..=5 on `DMA2`);
    /// `to_periph` selects the memory→peripheral direction.
    pub fn request(&mut self, controller: u8, channel: u8, to_periph: bool) {
        let bit = Self::bit(controller, channel);
        if to_periph {
            self.to_periph |= bit;
        } else {
            self.from_periph |= bit;
        }
    }

    /// Deassert the request for hardware `channel` on `controller`.
    pub fn release(&mut self, controller: u8, channel: u8, to_periph: bool) {
        let bit = Self::bit(controller, channel);
        if to_periph {
            self.to_periph &= !bit;
        } else {
            self.from_periph &= !bit;
        }
    }
}

impl Default for DmaRequests {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
impl DmaRequests {
    /// The asserted memory→peripheral request mask, for host inspection.
    #[must_use]
    pub(crate) const fn to_periph(&self) -> u32 {
        self.to_periph
    }

    /// The asserted peripheral→memory request mask, for host inspection.
    #[must_use]
    pub(crate) const fn from_periph(&self) -> u32 {
        self.from_periph
    }
}

const ISR: u32 = 0x00;
const IFCR: u32 = 0x04;
const CCR_CH: u32 = 0x08;
const CNDTR_CH: u32 = 0x0C;
const CPAR_CH: u32 = 0x10;
const CMAR_CH: u32 = 0x14;
const CH_STRIDE: u32 = 0x14;

const EN: u32 = 1 << 0;
const TCIE: u32 = 1 << 1;
const PINC: u32 = 1 << 6;
const MINC: u32 = 1 << 7;
const PSIZE_SHIFT: u32 = 8;
const MSIZE_SHIFT: u32 = 10;
const MEM2MEM: u32 = 1 << 14;
const NDT_MASK: u32 = 0xFFFF;

/// ISR `GIF` bit for channel `ch` (0-based).
const fn isr_gif(ch: u32) -> u32 {
    1 << (4 * ch)
}

/// ISR `TCIF` bit for channel `ch` (0-based).
const fn isr_tcif(ch: u32) -> u32 {
    1 << (4 * ch + 1)
}

/// The transfer engine the SVD cannot express.
#[derive(Debug)]
struct DmaBehavior {
    /// The shared queue of pending transfer units, drained by `SystemMemory`.
    mailbox: Arc<Mutex<Vec<DmaUnit>>>,
    /// The peripheral-request lines, shared with the request sources.
    requests: Arc<Mutex<DmaRequests>>,
    /// The controller index: `DMA1` is 0, `DMA2` is 1 (request-bit prefix).
    controller: u8,
    /// The number of DMA channels in the controller's register file.
    channels: u8,
}

impl DmaBehavior {
    const fn new(
        mailbox: Arc<Mutex<Vec<DmaUnit>>>,
        requests: Arc<Mutex<DmaRequests>>,
        controller: u8,
        channels: u8,
    ) -> Self {
        Self {
            mailbox,
            requests,
            controller,
            channels,
        }
    }
}

impl Behavior for DmaBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            // `IFCR` is a write-1-to-clear into `ISR`: `CGIF{n}` drops all
            // four flags of channel `n`, and `CTCIF{n}`/`CHTIF{n}`/`CTEIF{n}`
            // drop the transfer-complete / half-transfer / error flag.
            IFCR => {
                let word = view.merged(IFCR, val, shift);
                let mut isr = view.get(ISR);
                for ch in 0..self.channels {
                    let base = 4 * u32::from(ch);
                    if word & (1u32 << base) != 0 {
                        isr &= !(0xF << base);
                    }
                    if word & (1u32 << (base + 1)) != 0 {
                        isr &= !(1u32 << (base + 1));
                    }
                    if word & (1u32 << (base + 2)) != 0 {
                        isr &= !(1u32 << (base + 2));
                    }
                    if word & (1u32 << (base + 3)) != 0 {
                        isr &= !(1u32 << (base + 3));
                    }
                }
                view.set(ISR, isr);
                word
            }
            // `CNDTR` is writable only while the channel is disabled.
            offset if offset >= CNDTR_CH && (offset - CNDTR_CH) % CH_STRIDE == 0 => {
                let ch = (offset - CNDTR_CH) / CH_STRIDE;
                if view.get(CCR_CH + ch * CH_STRIDE) & EN != 0 {
                    view.get(offset)
                } else {
                    view.merged(offset, val, shift)
                }
            }
            _ => view.merged(offset, val, shift),
        }
    }

    // `cpar`/`cmar` mirror the CPAR/CMAR register names.
    #[allow(clippy::similar_names)]
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        // Arbitration: higher `PL` first, then lower channel number.
        let mut order: Vec<(u32, u8)> = (0..self.channels)
            .filter_map(|ch| {
                let base = CH_STRIDE * u32::from(ch);
                let ccr = view.get(CCR_CH + base);
                if ccr & EN == 0 {
                    return None;
                }
                if view.get(CNDTR_CH + base) & NDT_MASK == 0 {
                    return None;
                }
                let pl = (ccr >> 12) & 0x3;
                Some((pl, ch))
            })
            .collect();
        order.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
        for (_, ch) in order {
            let base = CH_STRIDE * u32::from(ch);
            let ccr = view.get(CCR_CH + base);
            if ccr & EN == 0 {
                continue;
            }
            let ndt = view.get(CNDTR_CH + base) & NDT_MASK;
            if ndt == 0 {
                continue;
            }
            let unit: u8 = match (ccr >> PSIZE_SHIFT) & 0x3 {
                0 => 1,
                1 => 2,
                _ => 4,
            };
            let step: u8 = match (ccr >> MSIZE_SHIFT) & 0x3 {
                0 => 1,
                1 => 2,
                _ => 4,
            };
            let pinc = ccr & PINC != 0;
            let minc = ccr & MINC != 0;
            let cpar = view.get(CPAR_CH + base);
            let cmar = view.get(CMAR_CH + base);
            let chn = u32::from(ch);
            if ccr & MEM2MEM != 0 {
                // Memory-to-memory: the whole transfer lands this cycle.
                let mut src = cpar;
                let mut dst = cmar;
                let mut units = Vec::with_capacity(ndt as usize);
                for _ in 0..ndt {
                    units.push((src, dst, unit));
                    if pinc {
                        src = src.wrapping_add(u32::from(unit));
                    }
                    if minc {
                        dst = dst.wrapping_add(u32::from(step));
                    }
                }
                self.mailbox.lock().unwrap().extend(units);
                // Optimistic completion: the whole transfer lands this cycle.
                view.set(CNDTR_CH + base, 0);
                view.set(CCR_CH + base, ccr & !EN);
                view.set(ISR, view.get(ISR) | isr_gif(chn) | isr_tcif(chn));
            } else {
                // Peripheral request: one unit per request-line assertion.
                // The request lines are keyed by the hardware channel number.
                let bit = DmaRequests::bit(self.controller, ch + 1);
                let (src, dst) = {
                    let mut req = self.requests.lock().unwrap();
                    if req.to_periph & bit != 0 {
                        // Memory → peripheral (e.g. USART TX into `DR`).
                        req.to_periph &= !bit;
                        (cmar, cpar)
                    } else if req.from_periph & bit != 0 {
                        // Peripheral → memory (e.g. USART RX out of `DR`).
                        req.from_periph &= !bit;
                        (cpar, cmar)
                    } else {
                        continue;
                    }
                };
                self.mailbox.lock().unwrap().push((src, dst, unit));
                // The incrementing side advances; the peripheral side is fixed.
                if pinc {
                    view.set(CPAR_CH + base, cpar.wrapping_add(u32::from(unit)));
                }
                if minc {
                    view.set(CMAR_CH + base, cmar.wrapping_add(u32::from(step)));
                }
                if ndt <= 1 {
                    view.set(CNDTR_CH + base, 0);
                    view.set(CCR_CH + base, ccr & !EN);
                    view.set(ISR, view.get(ISR) | isr_gif(chn) | isr_tcif(chn));
                } else {
                    view.set(CNDTR_CH + base, ndt - 1);
                }
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let mut lines = 0;
        for ch in 0..self.channels {
            let base = CH_STRIDE * u32::from(ch);
            if view.get(CCR_CH + base) & TCIE != 0 && view.get(ISR) & isr_tcif(u32::from(ch)) != 0 {
                lines |= 1 << ch;
            }
        }
        lines
    }

    fn lines(&self) -> u32 {
        (1 << self.channels) - 1
    }
}

/// An STM32F1 DMA controller: `DMA1` (7 channels) or `DMA2` (5 channels).
#[derive(Debug)]
pub struct DmaF1 {
    inner: RegisterFile<DmaBehavior>,
}

impl DmaF1 {
    /// A new `DMA1` controller sharing `mailbox`, fed by `requests`.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `DMA1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn dma1(mailbox: Arc<Mutex<Vec<DmaUnit>>>, requests: Arc<Mutex<DmaRequests>>) -> Self {
        Self::dma_for::<stm32f103::Stm32f103>("DMA1", mailbox, requests)
    }

    /// A DMA controller from another chip's device tables, found by SVD name
    /// (`"DMA1"` or `"DMA2"`), sharing `requests` with the request sources.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain `name`.
    #[must_use]
    pub fn dma_for<D: Device>(
        name: &str,
        mailbox: Arc<Mutex<Vec<DmaUnit>>>,
        requests: Arc<Mutex<DmaRequests>>,
    ) -> Self {
        let def = D::peripheral(name).expect("generated tables contain the DMA controller");
        let channels = u8::try_from(
            def.registers
                .iter()
                .filter(|r| r.name.starts_with("CCR"))
                .count(),
        )
        .expect("channel count fits u8");
        let controller = if name == "DMA2" { 1 } else { 0 };
        Self {
            inner: RegisterFile::new(
                def,
                DmaBehavior::new(mailbox, requests, controller, channels),
            ),
        }
    }
}

impl Peripheral for DmaF1 {
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
        self.inner.name()
    }
}

impl InterruptSource for DmaF1 {
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
}

#[cfg(test)]
mod tests {
    use super::{
        DmaF1, DmaRequests, DmaUnit, CCR_CH, CMAR_CH, CNDTR_CH, CPAR_CH, EN, ISR, MEM2MEM, MINC,
        PINC,
    };
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;
    use std::sync::{Arc, Mutex};

    const BASE: u32 = 0x4002_0000;
    const CCR1: u32 = CCR_CH;
    const CNDTR1: u32 = CNDTR_CH;
    const CPAR1: u32 = CPAR_CH;
    const CMAR1: u32 = CMAR_CH;
    const ISR1: u32 = ISR;

    fn mailbox() -> Arc<Mutex<Vec<DmaUnit>>> {
        Arc::new(Mutex::new(Vec::new()))
    }

    fn requests() -> Arc<Mutex<DmaRequests>> {
        Arc::new(Mutex::new(DmaRequests::new()))
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(d: &mut DmaF1, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            d.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(d: &mut DmaF1, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = d.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn memory_to_memory_copies_every_unit() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m.clone(), requests());
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 3);
        write_word(&mut d, BASE + CCR1, EN | MEM2MEM | MINC | PINC);
        d.tick(1);
        assert_eq!(
            *m.lock().unwrap(),
            vec![
                (0x1000, 0x2000, 1),
                (0x1001, 0x2001, 1),
                (0x1002, 0x2002, 1)
            ]
        );
        assert_eq!(read_word(&mut d, BASE + CNDTR1), 0, "CNDTR exhausted");
        assert_eq!(
            read_word(&mut d, BASE + CCR1) & EN,
            0,
            "EN self-clears on completion"
        );
        assert_eq!(read_word(&mut d, BASE + ISR1) & 0x3, 0x3, "GIF1|TCIF1 set");
    }

    #[test]
    fn word_units_step_by_msize() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m.clone(), requests());
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 2);
        write_word(
            &mut d,
            BASE + CCR1,
            EN | MEM2MEM | MINC | PINC | (2 << 8) | (2 << 10),
        );
        d.tick(1);
        assert_eq!(
            *m.lock().unwrap(),
            vec![(0x1000, 0x2000, 4), (0x1004, 0x2004, 4)]
        );
    }

    #[test]
    fn pending_requires_tcie() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m, requests());
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 1);
        write_word(&mut d, BASE + CCR1, EN | MEM2MEM);
        d.tick(1);
        assert_eq!(d.pending(), 0, "no TCIE, no line");
        write_word(&mut d, BASE + CCR1, 0x2); // TCIE on
        assert_eq!(d.pending(), 1, "TCIE raises line 0");
    }

    #[test]
    fn pending_is_per_channel() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m, requests());
        // Channel 1 (index 0) completes → line 0.
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 1);
        write_word(&mut d, BASE + CCR1, EN | MEM2MEM | 0x2);
        d.tick(1);
        assert_eq!(d.pending(), 0x1);
        // Channel 2 (index 1) completes → line 1.
        write_word(&mut d, BASE + 0x14 + CPAR1, 0x1000);
        write_word(&mut d, BASE + 0x14 + CMAR1, 0x2000);
        write_word(&mut d, BASE + 0x14 + CNDTR1, 1);
        write_word(&mut d, BASE + 0x14 + CCR1, EN | MEM2MEM | 0x2);
        d.tick(1);
        assert_eq!(d.pending(), 0x3, "both channel lines latch");
    }

    #[test]
    fn ifcr_clears_the_transfer_flag() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m, requests());
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 1);
        write_word(&mut d, BASE + CCR1, EN | MEM2MEM | 0x2);
        d.tick(1);
        assert_eq!(read_word(&mut d, BASE + ISR1) & 0x3, 0x3);
        write_word(&mut d, BASE + 0x04, 0x2); // IFCR CTCIF1
        assert_eq!(read_word(&mut d, BASE + ISR1) & 0x2, 0x0, "TCIF cleared");
        assert_eq!(
            read_word(&mut d, BASE + ISR1) & 0x1,
            0x1,
            "GIF waits for CGIF"
        );
    }

    #[test]
    fn cgif_clears_all_channel_flags() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m, requests());
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 1);
        write_word(&mut d, BASE + CCR1, EN | MEM2MEM);
        d.tick(1);
        assert_eq!(read_word(&mut d, BASE + ISR1) & 0x3, 0x3);
        write_word(&mut d, BASE + 0x04, 0x1); // IFCR CGIF1
        assert_eq!(read_word(&mut d, BASE + ISR1) & 0x3, 0x0);
    }

    #[test]
    fn cndtr_write_is_ignored_while_enabled() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m, requests());
        write_word(&mut d, BASE + CCR1, EN);
        write_word(&mut d, BASE + CNDTR1, 5);
        assert_eq!(read_word(&mut d, BASE + CNDTR1), 0);
        write_word(&mut d, BASE + CCR1, 0);
        write_word(&mut d, BASE + CNDTR1, 5);
        assert_eq!(read_word(&mut d, BASE + CNDTR1), 5);
    }

    #[test]
    fn peripheral_request_channels_transfer_on_request() {
        let m = mailbox();
        let req = requests();
        let mut d = DmaF1::dma1(m.clone(), req.clone());
        write_word(&mut d, BASE + CPAR1, 0x1000); // peripheral side
        write_word(&mut d, BASE + CMAR1, 0x2000); // memory side
        write_word(&mut d, BASE + CNDTR1, 5);
        write_word(&mut d, BASE + CCR1, EN | MINC);
        d.tick(1);
        assert!(m.lock().unwrap().is_empty(), "no request, no unit");
        assert_eq!(read_word(&mut d, BASE + CNDTR1), 5);
        assert_eq!(read_word(&mut d, BASE + ISR1), 0, "nothing completed");
        // A peripheral→memory request transfers one unit and is consumed.
        req.lock().unwrap().request(0, 1, false);
        d.tick(1);
        assert_eq!(*m.lock().unwrap(), vec![(0x1000, 0x2000, 1)]);
        assert_eq!(read_word(&mut d, BASE + CNDTR1), 4);
        assert_eq!(read_word(&mut d, BASE + CMAR1), 0x2001, "MINC advanced");
        assert_eq!(read_word(&mut d, BASE + CPAR1), 0x1000, "peripheral fixed");
        // A memory→peripheral request reads memory and writes the peripheral.
        req.lock().unwrap().request(0, 1, true);
        d.tick(1);
        assert_eq!(
            *m.lock().unwrap(),
            vec![(0x1000, 0x2000, 1), (0x2001, 0x1000, 1)]
        );
    }

    #[test]
    fn peripheral_request_completes_and_latches_tcif() {
        let m = mailbox();
        let req = requests();
        let mut d = DmaF1::dma1(m, req.clone());
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 1);
        write_word(&mut d, BASE + CCR1, EN | 0x2); // TCIE on
        req.lock().unwrap().request(0, 1, true);
        d.tick(1);
        assert_eq!(read_word(&mut d, BASE + CNDTR1), 0, "CNDTR exhausted");
        assert_eq!(read_word(&mut d, BASE + CCR1) & EN, 0, "EN self-clears");
        assert_eq!(read_word(&mut d, BASE + ISR1) & 0x3, 0x3, "GIF1|TCIF1 set");
        assert_eq!(d.pending(), 0x1, "channel line latched");
    }

    #[test]
    fn disabled_channels_do_not_transfer() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m.clone(), requests());
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 5);
        write_word(&mut d, BASE + CCR1, MEM2MEM);
        d.tick(1);
        assert!(m.lock().unwrap().is_empty());
        assert_eq!(read_word(&mut d, BASE + CNDTR1), 5);
    }

    #[test]
    fn dma2_mounts_five_channels_on_f107() {
        use crate::device::svd::stm32f107::Stm32f107;
        const D2: u32 = 0x4002_0400;
        let m = mailbox();
        let mut d = DmaF1::dma_for::<Stm32f107>("DMA2", m.clone(), requests());
        write_word(&mut d, D2 + 0x10 + 4 * 0x14, 0x3000); // CPAR5
        write_word(&mut d, D2 + 0x14 + 4 * 0x14, 0x4000); // CMAR5
        write_word(&mut d, D2 + 0x0C + 4 * 0x14, 1); // CNDTR5
        write_word(&mut d, D2 + 0x08 + 4 * 0x14, EN | MEM2MEM); // CCR5
        d.tick(1);
        assert_eq!(*m.lock().unwrap(), vec![(0x3000, 0x4000, 1)]);
    }

    #[test]
    fn dma2_requests_use_its_own_bit_prefix() {
        use crate::device::svd::stm32f107::Stm32f107;
        const D2: u32 = 0x4002_0400;
        let m = mailbox();
        let req = requests();
        let mut d = DmaF1::dma_for::<Stm32f107>("DMA2", m.clone(), req.clone());
        // Channel 5 on DMA2 = request bit 8 * 1 + 5 = 13.
        write_word(&mut d, D2 + 0x10 + 4 * 0x14, 0x3000); // CPAR5
        write_word(&mut d, D2 + 0x14 + 4 * 0x14, 0x4000); // CMAR5
        write_word(&mut d, D2 + 0x0C + 4 * 0x14, 1); // CNDTR5
        write_word(&mut d, D2 + 0x08 + 4 * 0x14, EN); // CCR5, peripheral mode
        req.lock().unwrap().request(1, 5, true);
        d.tick(1);
        assert_eq!(*m.lock().unwrap(), vec![(0x4000, 0x3000, 1)]);
        assert_eq!(
            read_word(&mut d, D2 + 0x0C + 4 * 0x14),
            0,
            "CNDTR exhausted"
        );
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m, requests());
        assert!(matches!(d.read(BASE + 0x90), Err(BusError::Reserved)));
        assert!(matches!(d.write(BASE - 1, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn pl_priority_arbitrates_higher_first() {
        let m = mailbox();
        let mut d = DmaF1::dma1(m.clone(), requests());
        // Channel 1 (PL 0) and channel 2 (PL 3) both M2M, 1 unit each.
        write_word(&mut d, BASE + CPAR1, 0x1000);
        write_word(&mut d, BASE + CMAR1, 0x2000);
        write_word(&mut d, BASE + CNDTR1, 1);
        write_word(&mut d, BASE + CCR1, EN | MEM2MEM); // PL 0
        let ch2 = 1u32;
        write_word(&mut d, BASE + 0x14 + CPAR1, 0x3000);
        write_word(&mut d, BASE + 0x14 + CMAR1, 0x4000);
        write_word(&mut d, BASE + 0x14 + CNDTR1, 1);
        write_word(&mut d, BASE + 0x14 + CCR1, EN | MEM2MEM | (3 << 12)); // PL 3
        d.tick(1);
        // Higher PL (ch2) queued first.
        assert_eq!(
            *m.lock().unwrap(),
            vec![(0x3000, 0x4000, 1), (0x1000, 0x2000, 1)]
        );
        assert_eq!(ch2, 1);
    }
}
