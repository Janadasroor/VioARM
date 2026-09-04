//! The STM32F4 DMA controllers (`DMA1`/`DMA2`), stream-based, driven by the
//! SVD-generated register tables.
//!
//! The F4 controllers replace the F1 channel set with eight streams per
//! controller, each carrying its own `SxCR`/`SxNDTR`/`SxPAR`/`SxM0AR`/
//! `SxM1AR`/`SxFCR` block and its own status flag group in `LISR`/`HISR`
//! (cleared through `LIFCR`/`HIFCR`). Any enabled stream copies `NDTR` units
//! honoring `DIR` (`00` peripheral-to-memory `SxPAR`→`SxM0AR`, `01`
//! memory-to-peripheral `SxM0AR`→`SxPAR`, `10` memory-to-memory `SxPAR`→
//! `SxM0AR`), `PINC`/`MINC` stepping and the `PSIZE`/`MSIZE` widths, queuing
//! the units into a mailbox [`SystemMemory`] drains after every device tick.
//! Completion is optimistic, matching the F1 memory-to-memory model: the whole
//! transfer lands in the cycle the engine sees the stream enabled. `NDTR`
//! counts down, `EN` self-clears at zero, and `TCIF` latches. Enabled streams
//! arbitrate by `SxCR.PL` (higher first, then lower stream number,
//! `pl_priority_arbitrates_higher_first`). The controller raises one NVIC line
//! per stream — `DMA1` puts streams 0..=6 on IRQ 11..=17 but stream 7 on IRQ
//! 47, and `DMA2` scatters streams 5..=7 at IRQ 68..=70 — so `pending()`
//! reports a per-stream bitmask and each line is routed by its table-mapped
//! IRQ. Peripheral-request modes are thus complete without a wired request
//! source, which firmware observes as an immediate transfer.

use crate::device::stm32::dma::{DmaRequests, DmaUnit};
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// `LISR` holds the flag groups of streams 0..=3.
const LISR: u32 = 0x00;
/// `HISR` holds the flag groups of streams 4..=7.
const HISR: u32 = 0x04;
/// `LIFCR` write-1-to-clears `LISR`.
const LIFCR: u32 = 0x08;
/// `HIFCR` write-1-to-clears `HISR`.
const HIFCR: u32 = 0x0C;
/// `S0CR`, the first stream configuration register.
const S0CR: u32 = 0x10;
/// The address stride between consecutive streams.
const STREAM_STRIDE: u32 = 0x18;
/// The number of streams in each controller.
const STREAMS: u8 = 8;

/// `SxCR.EN`: the stream is enabled.
const EN: u32 = 1 << 0;
/// `SxCR.TCIE`: transfer-complete interrupt enable.
const TCIE: u32 = 1 << 4;
/// `SxCR.DIR`: transfer direction (2 = memory-to-memory).
const DIR_SHIFT: u32 = 6;
const DIR_M2M: u32 = 2 << DIR_SHIFT;
/// `SxCR.PINC`: increment the peripheral address after each transfer.
const PINC: u32 = 1 << 9;
/// `SxCR.MINC`: increment the memory address after each transfer.
const MINC: u32 = 1 << 10;
/// `SxCR.PSIZE`: the peripheral-side data width.
const PSIZE_SHIFT: u32 = 11;
/// `SxCR.MSIZE`: the memory-side data width.
const MSIZE_SHIFT: u32 = 13;
/// `SxCR.MBURST`/`PBURST` burst transfer: 00 single, 01 INCR4, 10 INCR8, 11 INCR16.
const PBURST_SHIFT: u32 = 21;
const MBURST_SHIFT: u32 = 23;
/// `SxFCR.FTH` FIFO threshold: 00 quarter, 01 half, 10 3/4, 11 full.
const FTH_MASK: u32 = 0x3;
/// `SxNDTR` is a 16-bit count of data items.
const NDTR_MASK: u32 = 0xFFFF;

/// Sub-offsets of each stream block, relative to `SxCR`.
const SNDTR: u32 = 0x04;
const SPAR: u32 = 0x08;
const SM0AR: u32 = 0x0C;

/// The `TCIF` bit of `stream` (0-based) inside its `LISR`/`HISR` group.
///
/// The F4 packs each stream's five flags (`FEIF`, `DMEIF`, `TEIF`, `HTIF`,
/// `TCIF`) into a 6-bit slot, but leaves bits 12..=15 (and 28..=31) reserved:
/// within a status register the four stream bases are 0, 6, 16, 22 and `TCIF`
/// sits five bits above its base. `LISR` holds streams 0..=3 and `HISR` holds
/// streams 4..=7, so the in-register slot is `stream % 4`.
const fn tcif_bit(stream: u32) -> u32 {
    let base = match stream % 4 {
        0 => 0,
        1 => 6,
        2 => 16,
        _ => 22,
    };
    1 << (base + 5)
}

/// The stream's transfer engine, table-driven from the SVD.
#[derive(Debug)]
struct DmaF4Behavior {
    /// The shared queue of pending transfer units, drained by `SystemMemory`.
    mailbox: Arc<Mutex<Vec<DmaUnit>>>,
    /// Peripheral-request lines, shared with request sources (TIM/ADC/SPI/
    /// USART/DAC). Bit `8 * controller + stream` asserted means the source
    /// wants one transfer step on that stream. `None` (boards that wire no
    /// sources) keeps the legacy optimistic whole-transfer behavior.
    requests: Option<(Arc<Mutex<DmaRequests>>, u8)>,
}

impl DmaF4Behavior {
    const fn new(mailbox: Arc<Mutex<Vec<DmaUnit>>>) -> Self {
        Self {
            mailbox,
            requests: None,
        }
    }

    /// Whether stream `s` may transfer this tick: always, unless request
    /// lines are wired, in which case its bit must be asserted.
    fn requested(&self, s: u8) -> bool {
        let Some((reqs, controller)) = &self.requests else {
            return true;
        };
        reqs.lock()
            .unwrap_or_else(|e| e.into_inner())
            .asserted(*controller, s)
    }

    /// Consume one request assertion on stream `s`.
    fn consume(&self, s: u8) {
        if let Some((reqs, controller)) = &self.requests {
            let mut reqs = reqs.lock().unwrap_or_else(|e| e.into_inner());
            reqs.release(*controller, s, true);
            reqs.release(*controller, s, false);
        }
    }
}

impl Behavior for DmaF4Behavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            // `LIFCR`/`HIFCR` are write-1-to-clear into `LISR`/`HISR`; both
            // status registers read back zero on silicon.
            LIFCR => {
                let word = view.merged(LIFCR, val, shift);
                view.set(LISR, view.get(LISR) & !word);
                0
            }
            HIFCR => {
                let word = view.merged(HIFCR, val, shift);
                view.set(HISR, view.get(HISR) & !word);
                0
            }
            // `SxNDTR` is writable only while the stream is disabled.
            offset if offset >= S0CR + SNDTR && (offset - (S0CR + SNDTR)) % STREAM_STRIDE == 0 => {
                let cr = offset - SNDTR;
                if view.get(cr) & EN != 0 {
                    view.get(offset)
                } else {
                    view.merged(offset, val, shift)
                }
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        // Arbitration: higher `PL` first, then lower stream number.
        let mut order: Vec<(u32, u8)> = (0..STREAMS)
            .filter_map(|stream| {
                let cr = S0CR + STREAM_STRIDE * u32::from(stream);
                let cr_word = view.get(cr);
                if cr_word & EN == 0 {
                    return None;
                }
                if view.get(cr + SNDTR) & NDTR_MASK == 0 {
                    return None;
                }
                let pl = (cr_word >> 16) & 0x3;
                Some((pl, stream))
            })
            .collect();
        order.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
        for (_, stream) in order {
            let s = u32::from(stream);
            let cr = S0CR + STREAM_STRIDE * u32::from(stream);
            let cr_word = view.get(cr);
            if cr_word & EN == 0 {
                continue;
            }
            let ndt = view.get(cr + SNDTR) & NDTR_MASK;
            if ndt == 0 {
                continue;
            }
            let dir = (cr_word >> DIR_SHIFT) & 0x3;
            let unit: u8 = match (cr_word >> PSIZE_SHIFT) & 0x3 {
                0 => 1,
                1 => 2,
                _ => 4,
            };
            let step: u8 = match (cr_word >> MSIZE_SHIFT) & 0x3 {
                0 => 1,
                1 => 2,
                _ => 4,
            };
            let pinc = cr_word & PINC != 0;
            let minc = cr_word & MINC != 0;
            let mburst = (cr_word >> MBURST_SHIFT) & 0x3;
            let pburst = (cr_word >> PBURST_SHIFT) & 0x3;
            let burst_len = match mburst.max(pburst) {
                0 => 1,
                1 => 4,
                2 => 8,
                _ => 16,
            };
            // `DIR` selects the copy direction: `00` peripheral→memory,
            // `01` memory→peripheral, `10` memory-to-memory.
            let (mut src, mut dst) = match dir {
                0 => (view.get(cr + SPAR), view.get(cr + SM0AR)),
                1 => (view.get(cr + SM0AR), view.get(cr + SPAR)),
                _ => (view.get(cr + SPAR), view.get(cr + SM0AR)),
            };
            let src_inc = if dir == 1 { minc } else { pinc };
            let dst_inc = if dir == 1 { pinc } else { minc };
            let src_step = if dir == 1 { step } else { unit };
            let dst_step = if dir == 1 { unit } else { step };
            // Peripheral-request directions pace against the source when request
            // lines are wired: one burst per asserted request, consuming it.
            // Memory-to-memory (and unwired controllers) stay optimistic.
            let paced = dir != 2 && self.requests.is_some();
            if paced && !self.requested(stream) {
                continue;
            }
            if paced {
                self.consume(stream);
            }
            // Burst/FIFO: transfer burst_len per tick when burst>1, else whole (optimistic M2M).
            let chunk = if paced || burst_len > 1 {
                let bursts = _delta.max(1) as u32;
                (burst_len * bursts).min(ndt)
            } else {
                ndt
            };
            let mut units = Vec::with_capacity(chunk as usize);
            for _ in 0..chunk {
                units.push((src, dst, unit));
                if src_inc {
                    src = src.wrapping_add(u32::from(src_step));
                }
                if dst_inc {
                    dst = dst.wrapping_add(u32::from(dst_step));
                }
            }
            // FIFO threshold gating (SxFCR.FTH) when direct mode disabled.
            let fcr = view.get(cr + 0x14);
            let fifo_en = fcr & (1 << 2) != 0;
            let fth = (fcr & FTH_MASK) as u8;
            let fifo_ready = if fifo_en {
                // Simplified: need chunk >= FTH+1 quarter levels.
                let thresh = match fth {
                    0 => 1,
                    1 => 2,
                    2 => 3,
                    _ => 4,
                };
                chunk >= thresh
            } else {
                true
            };
            if !fifo_ready {
                continue;
            }
            self.mailbox
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .extend(units);
            let remain = ndt - chunk;
            // Update addresses after burst.
            if dir == 1 {
                if minc {
                    view.set(cr + SM0AR, src);
                }
                if pinc {
                    view.set(cr + SPAR, dst);
                }
            } else {
                if pinc {
                    view.set(cr + SPAR, src);
                }
                if minc {
                    view.set(cr + SM0AR, dst);
                }
            }
            view.set(cr + SNDTR, remain);
            if remain == 0 {
                view.set(cr, cr_word & !EN);
                let isr = if s < 4 { LISR } else { HISR };
                view.set(isr, view.get(isr) | tcif_bit(s));
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let mut lines = 0;
        for stream in 0..STREAMS {
            let s = u32::from(stream);
            let cr = S0CR + STREAM_STRIDE * u32::from(stream);
            let isr = if s < 4 { LISR } else { HISR };
            if view.get(cr) & TCIE != 0 && view.get(isr) & tcif_bit(s) != 0 {
                lines |= 1 << s;
            }
        }
        lines
    }

    fn lines(&self) -> u32 {
        (1 << STREAMS) - 1
    }
}

/// The `DMA1`/`DMA2` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct DmaF4 {
    inner: RegisterFile<DmaF4Behavior>,
}

impl DmaF4 {
    /// A new `DMA1` controller at its STM32F407 base address, sharing the
    /// transfer mailbox with the memory bus.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `DMA1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn dma1_for<D: Device>(mailbox: Arc<Mutex<Vec<DmaUnit>>>) -> Self {
        Self::for_device::<D>("DMA1", mailbox)
    }

    /// A new `DMA2` controller at its STM32F407 base address, sharing the
    /// transfer mailbox with the memory bus.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `DMA2`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn dma2_for<D: Device>(mailbox: Arc<Mutex<Vec<DmaUnit>>>) -> Self {
        Self::for_device::<D>("DMA2", mailbox)
    }

    /// A new controller named `name` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the named peripheral.
    #[must_use]
    pub fn for_device<D: Device>(name: &str, mailbox: Arc<Mutex<Vec<DmaUnit>>>) -> Self {
        let def = D::peripheral(name).expect("generated tables contain the controller");
        Self {
            inner: RegisterFile::new(def, DmaF4Behavior::new(mailbox)),
        }
    }

    /// Share peripheral-request lines: streams in peripheral-request
    /// directions then advance one burst per asserted request instead of
    /// completing optimistically. `controller` is 0 for DMA1, 1 for DMA2;
    /// sources address streams by number. Boards wire the conventional
    /// routes (see the board constructors); `CHSEL` remapping is not modeled.
    #[must_use]
    pub fn with_requests(mut self, requests: Arc<Mutex<DmaRequests>>, controller: u8) -> Self {
        self.inner.behavior_mut().requests = Some((requests, controller));
        self
    }
}

impl Peripheral for DmaF4 {
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
        "dma"
    }
}

impl InterruptSource for DmaF4 {
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
    use super::{DmaF4, EN, LISR, S0CR, STREAM_STRIDE, TCIE};
    use crate::device::stm32::dma::DmaUnit;
    use crate::device::svd::stm32f407::Stm32f407;
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;
    use std::sync::{Arc, Mutex};

    const D1: u32 = 0x4002_6000;

    fn s_cr(s: u32) -> u32 {
        0x10 + s * 0x18
    }

    fn s_ndtr(s: u32) -> u32 {
        s_cr(s) + 0x04
    }

    fn s_par(s: u32) -> u32 {
        s_cr(s) + 0x08
    }

    fn s_m0ar(s: u32) -> u32 {
        s_cr(s) + 0x0C
    }

    fn mailbox() -> Arc<Mutex<Vec<DmaUnit>>> {
        Arc::new(Mutex::new(Vec::new()))
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(d: &mut DmaF4, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            d.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(d: &mut DmaF4, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = d.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn stream_offsets_match_the_svd() {
        let m = mailbox();
        let d = DmaF4::dma1_for::<Stm32f407>(m);
        // S0CR at 0x10, stride 0x18: S7CR at 0xB8.
        for s in 0..8u32 {
            assert_eq!(
                d.ranges()[0].base + S0CR + s * STREAM_STRIDE,
                D1 + 0x10 + s * 0x18,
                "S{s}CR offset"
            );
        }
        // LISR/LIFCR pairs per the SVD.
        assert_eq!(d.ranges()[0].base, D1);
    }

    #[test]
    fn memory_to_memory_transfer_completes_and_latches_tcif() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone());
        // Stream 1: 16 words, byte-sized, both sides incrementing, M2M.
        write_word(&mut d, D1 + s_par(1), 0x1000);
        write_word(&mut d, D1 + s_m0ar(1), 0x2000);
        write_word(&mut d, D1 + s_ndtr(1), 16);
        write_word(&mut d, D1 + s_cr(1), EN | TCIE | (1 << 10) | (2 << 6));
        d.tick(1);
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner()).len(),
            16,
            "the whole M2M transfer is queued"
        );
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner())[0],
            (0x1000, 0x2000, 1),
            "M2M reads PAR, writes M0AR"
        );
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner())[15],
            (0x1000, 0x200F, 1),
            "MINC advanced the memory side"
        );
        // NDTR exhausted, EN self-cleared, TCIF1 latched in LISR (stream 1).
        assert_eq!(read_word(&mut d, D1 + s_ndtr(1)), 0);
        assert_eq!(read_word(&mut d, D1 + s_cr(1)) & EN, 0);
        assert_eq!(read_word(&mut d, D1 + LISR) & (1 << 11), 1 << 11);
        assert_eq!(d.pending(), 1 << 1, "stream-1 line latched");
    }

    #[test]
    fn disabled_streams_do_not_transfer() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone());
        write_word(&mut d, D1 + s_par(0), 0x1000);
        write_word(&mut d, D1 + s_m0ar(0), 0x2000);
        write_word(&mut d, D1 + s_ndtr(0), 16);
        write_word(&mut d, D1 + s_cr(0), 2 << 6); // M2M, no EN
        d.tick(1);
        assert!(m.lock().unwrap_or_else(|e| e.into_inner()).is_empty());
        assert_eq!(read_word(&mut d, D1 + s_ndtr(0)), 16);
    }

    #[test]
    fn peripheral_request_dirs_complete_optimistically() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone());
        // DIR 00 peripheral→memory: 2 units, PINC set, MINC set.
        write_word(&mut d, D1 + s_par(2), 0x4000_0400); // USART DR
        write_word(&mut d, D1 + s_m0ar(2), 0x2000_1000);
        write_word(&mut d, D1 + s_ndtr(2), 2);
        write_word(&mut d, D1 + s_cr(2), (EN | TCIE | (1 << 9) | (1 << 10)));
        d.tick(1);
        assert_eq!(m.lock().unwrap_or_else(|e| e.into_inner()).len(), 2);
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner())[0],
            (0x4000_0400, 0x2000_1000, 1)
        );
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner())[1],
            (0x4000_0401, 0x2000_1001, 1)
        );
        assert_eq!(read_word(&mut d, D1 + s_ndtr(2)), 0);
        assert_eq!(read_word(&mut d, D1 + s_cr(2)) & EN, 0);
        // DIR 01 memory→peripheral: 1 unit, MINC set, PINC clear.
        let m2 = mailbox();
        let mut d2 = DmaF4::dma1_for::<Stm32f407>(m2.clone());
        write_word(&mut d2, D1 + s_par(3), 0x4000_0400);
        write_word(&mut d2, D1 + s_m0ar(3), 0x2000_2000);
        write_word(&mut d2, D1 + s_ndtr(3), 1);
        write_word(&mut d2, D1 + s_cr(3), EN | (1 << 10) | (1 << 6));
        d2.tick(1);
        assert_eq!(
            *m2.lock().unwrap_or_else(|e| e.into_inner()),
            vec![(0x2000_2000, 0x4000_0400, 1)]
        );
        assert_eq!(read_word(&mut d2, D1 + s_ndtr(3)), 0);
    }

    #[test]
    fn ifcr_w1c_clears_the_status_flags() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m);
        // Force TCIF0 and TCIF3 into LISR via a stream transfer.
        for s in [0u32, 3] {
            write_word(&mut d, D1 + s_par(s), 0x1000 + s);
            write_word(&mut d, D1 + s_m0ar(s), 0x2000 + s);
            write_word(&mut d, D1 + s_ndtr(s), 1);
            write_word(&mut d, D1 + s_cr(s), EN | TCIE | (2 << 6));
        }
        d.tick(1);
        assert_eq!(
            read_word(&mut d, D1 + LISR),
            (1 << 5) | (1 << 27),
            "TCIF0 and TCIF3 set"
        );
        // Write-1-to-clear TCIF0 only.
        write_word(&mut d, D1 + 0x08, 1 << 5);
        assert_eq!(
            read_word(&mut d, D1 + LISR),
            1 << 27,
            "TCIF0 cleared, TCIF3 kept"
        );
        assert_eq!(d.pending(), 1 << 3);
    }

    #[test]
    fn ndtr_write_is_ignored_while_enabled() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m);
        write_word(&mut d, D1 + s_cr(0), EN);
        write_word(&mut d, D1 + s_ndtr(0), 5);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(0)), 0);
        write_word(&mut d, D1 + s_cr(0), 0);
        write_word(&mut d, D1 + s_ndtr(0), 5);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(0)), 5);
    }

    #[test]
    fn stream7_reaches_its_scattered_irq() {
        // DMA1 stream 7 lives at IRQ 47, not 18.
        let m = mailbox();
        let d = DmaF4::dma1_for::<Stm32f407>(m);
        assert_eq!(d.exception_number(), Some(27), "base IRQ 11");
        assert_eq!(d.exception_for_line(7), Some(63), "IRQ 47");
        assert_eq!(d.exception_for_line(6), Some(33), "IRQ 17");
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m);
        assert!(matches!(d.read(D1 + 0x400), Err(BusError::Reserved)));
        assert!(matches!(d.write(D1 - 1, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn peripheral_request_paces_one_burst_per_tick() {
        use crate::device::stm32::dma::DmaRequests;
        let m = mailbox();
        let reqs = Arc::new(Mutex::new(DmaRequests::new()));
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone()).with_requests(reqs.clone(), 0);
        // Stream 2, memory-to-peripheral, 8 units, single transfers.
        write_word(&mut d, D1 + s_par(2), 0x4001_1404);
        write_word(&mut d, D1 + s_m0ar(2), 0x2000_0000);
        write_word(&mut d, D1 + s_ndtr(2), 8);
        write_word(
            &mut d,
            D1 + s_cr(2),
            EN | TCIE | (1 << 6) | (1 << 9) | (1 << 10),
        );
        // No request asserted: the stream stalls, nothing queued.
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(2)), 8);
        assert!(m.lock().unwrap_or_else(|e| e.into_inner()).is_empty());
        // One assertion moves exactly one burst and is consumed.
        reqs.lock()
            .unwrap_or_else(|e| e.into_inner())
            .request(0, 2, true);
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(2)), 7);
        assert_eq!(m.lock().unwrap_or_else(|e| e.into_inner()).len(), 1);
        assert!(!reqs
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .asserted(0, 2));
        // Stalls again until the next assertion.
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(2)), 7);
        reqs.lock()
            .unwrap_or_else(|e| e.into_inner())
            .request(0, 2, false);
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(2)), 6);
        assert_eq!(m.lock().unwrap_or_else(|e| e.into_inner()).len(), 2);
        // Memory-to-memory on the same wired controller stays optimistic.
        write_word(&mut d, D1 + s_par(3), 0x2000_1000);
        write_word(&mut d, D1 + s_m0ar(3), 0x2000_2000);
        write_word(&mut d, D1 + s_ndtr(3), 4);
        write_word(&mut d, D1 + s_cr(3), EN | (2 << 6));
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(3)), 0);
    }

    #[test]
    fn pl_priority_arbitrates_higher_first() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone());
        // Stream 0 PL 0 and stream 1 PL 3, both M2M.
        write_word(&mut d, D1 + s_par(0), 0x1000);
        write_word(&mut d, D1 + s_m0ar(0), 0x2000);
        write_word(&mut d, D1 + s_ndtr(0), 1);
        write_word(&mut d, D1 + s_cr(0), EN | (2 << 6)); // PL 0
        write_word(&mut d, D1 + s_par(1), 0x3000);
        write_word(&mut d, D1 + s_m0ar(1), 0x4000);
        write_word(&mut d, D1 + s_ndtr(1), 1);
        write_word(&mut d, D1 + s_cr(1), EN | (3 << 16) | (2 << 6)); // PL 3
        d.tick(1);
        assert_eq!(
            *m.lock().unwrap_or_else(|e| e.into_inner()),
            vec![(0x3000, 0x4000, 1), (0x1000, 0x2000, 1)]
        );
    }
}
