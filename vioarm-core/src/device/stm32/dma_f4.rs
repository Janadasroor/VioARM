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
//! `NDTR` counts down and `TCIF` latches at zero; the half-transfer flag
//! `HTIF` latches when `NDTR` reaches half the programmed count, and a
//! host-injected bus fault (`DmaF4::inject_error`) latches `TEIF` and
//! disables the stream. `CIRC` (`SxCR:8`) reloads `NDTR`/`PAR`/`M0AR` from
//! the values latched when `EN` was set and keeps `EN`, so `TCIF` fires every
//! period; with `DBM` (`SxCR:18`) the `CT` bit (`SxCR:19`) toggles and periods
//! alternate between `M0AR` and `M1AR`. The FIFO (`SxFCR`, `DMDIS=1`)
//! accumulates produced bytes and releases them to the mailbox once the
//! `FTH` threshold is reached (flushing the residue when `NDTR` exhausts);
//! production past the 16-byte capacity latches `FEIF`. A burst programmed
//! while direct (`DMDIS=0`) latches `DMEIF` on `EN` set. Enabled streams
//! arbitrate by `SxCR.PL` (higher first, then lower stream number,
//! `pl_priority_arbitrates_higher_first`). The controller raises one NVIC line
//! per stream — `DMA1` puts streams 0..=6 on IRQ 11..=17 but stream 7 on IRQ
//! 47, and `DMA2` scatters streams 5..=7 at IRQ 68..=70 — so `pending()`
//! reports a per-stream bitmask and each line is routed by its table-mapped
//! IRQ. Each enable (`TCIE`&`TCIF`, `HTIE`&`HTIF`, `TEIE`&`TEIF`,
//! `DMEIE`&`DMEIF`, `FEIE`&`FEIF`) gates its own flag. Peripheral-request
//! modes are thus complete without a wired request
//! source, which firmware observes as an immediate transfer.
//
//! Request routing (board convention): sources assert stream-numbered bits
//! and streams listen on their stream number while `CHSEL` is zero; a
//! nonzero `CHSEL` (`SxCR` 27:25) remaps the stream to that channel's bit.
//! (On silicon `CHSEL=0` is channel 0; the zero-means-stream rule preserves
//! the board convention for firmware that leaves `CHSEL` at reset.)

use crate::device::stm32::dma::{DmaRequests, DmaUnit, Period};
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
/// `SxCR.DMEIE`: direct-mode-error interrupt enable.
const DMEIE: u32 = 1 << 1;
/// `SxCR.TEIE`: transfer-error interrupt enable.
const TEIE: u32 = 1 << 2;
/// `SxCR.HTIE`: half-transfer interrupt enable.
const HTIE: u32 = 1 << 3;
/// `SxCR.TCIE`: transfer-complete interrupt enable.
const TCIE: u32 = 1 << 4;
/// `SxCR.CIRC`: circular mode (reload the latched period, keep `EN`).
const CIRC: u32 = 1 << 8;
/// `SxCR.DBM`: double-buffer mode (periods alternate `M0AR`/`M1AR`).
const DBM: u32 = 1 << 18;
/// `SxCR.CT`: current target in double-buffer mode (0 = `M0AR`, 1 = `M1AR`).
const CT: u32 = 1 << 19;
/// `SxCR.CHSEL`: request-channel select (27:25); zero keeps the board
/// convention (stream-numbered request bits).
const CHSEL_SHIFT: u32 = 25;
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
/// `SxFCR.DMDIS`: FIFO mode (direct mode when clear).
const DMDIS: u32 = 1 << 2;
/// FIFO capacity in bytes (4 words).
const FIFO_CAP: u16 = 16;
/// `SxNDTR` is a 16-bit count of data items.
const NDTR_MASK: u32 = 0xFFFF;

/// Sub-offsets of each stream block, relative to `SxCR`.
const SNDTR: u32 = 0x04;
const SPAR: u32 = 0x08;
const SM0AR: u32 = 0x0C;
const SM1AR: u32 = 0x10;
const SFCR: u32 = 0x14;

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

/// Flag-slot base of `stream` (0-based) inside its `LISR`/`HISR` group.
const fn flag_base(stream: u32) -> u32 {
    match stream % 4 {
        0 => 0,
        1 => 6,
        2 => 16,
        _ => 22,
    }
}

/// The `FEIF` bit of `stream` (base+0).
const fn feif_bit(stream: u32) -> u32 {
    1 << flag_base(stream)
}

/// The `DMEIF` bit of `stream` (base+1).
const fn dmeif_bit(stream: u32) -> u32 {
    1 << (flag_base(stream) + 1)
}

/// The `TEIF` bit of `stream` (base+2).
const fn teif_bit(stream: u32) -> u32 {
    1 << (flag_base(stream) + 2)
}

/// The `HTIF` bit of `stream` (base+3).
const fn htif_bit(stream: u32) -> u32 {
    1 << (flag_base(stream) + 3)
}

/// The status register holding `stream`'s flag group.
const fn stream_isr(stream: u32) -> u32 {
    if stream < 4 {
        LISR
    } else {
        HISR
    }
}

/// FIFO threshold in bytes for an `FTH` field value.
const fn fth_bytes(fth: u32) -> u16 {
    match fth {
        0 => 4,
        1 => 8,
        2 => 12,
        _ => 16,
    }
}

/// The stream's transfer engine, table-driven from the SVD.
#[derive(Debug)]
struct DmaF4Behavior {
    /// The shared queue of pending transfer units, drained by `SystemMemory`.
    mailbox: Arc<Mutex<Vec<DmaUnit>>>,
    /// Peripheral-request lines, shared with request sources (TIM/ADC/SPI/
    /// USART/DAC). Bit `8 * controller + channel` asserted means the source
    /// wants one transfer step. `None` (boards that wire no sources) keeps
    /// the legacy optimistic whole-transfer behavior.
    requests: Option<(Arc<Mutex<DmaRequests>>, u8)>,
    /// Period state per stream, latched on `EN` set (plus `M1AR` for `DBM`).
    period: [Period; 8],
    /// Second-buffer bases per stream (`SM1AR` at `EN` set) for `DBM`.
    m1ar0: [u32; 8],
    /// Buffered FIFO bytes per stream (levels) and their units.
    fifo_lvl: [u16; 8],
    fifo_buf: [Vec<DmaUnit>; 8],
    /// Host-injected bus faults per stream bit (`inject_error`); applied on
    /// the next tick as `TEIF` with the stream disabled. Drain-time bus
    /// errors cannot reach the device (the mailbox is one-way), so this is
    /// the only `TEIF` source, and it is documented as such.
    error: u8,
    /// Host-injected FIFO errors per stream bit (`inject_fifo_error`);
    /// applied on the next tick as `FEIF` without disabling the stream.
    fifo_error: u8,
}

impl DmaF4Behavior {
    fn new(mailbox: Arc<Mutex<Vec<DmaUnit>>>) -> Self {
        Self {
            mailbox,
            requests: None,
            period: [Period::default(); 8],
            m1ar0: [0; 8],
            fifo_lvl: [0; 8],
            fifo_buf: [
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
            ],
            error: 0,
            fifo_error: 0,
        }
    }

    /// The request-channel bit stream `s` listens on: `CHSEL` when nonzero,
    /// else the stream number (board convention).
    fn request_sel(cr_word: u32, s: u8) -> u8 {
        let chsel = ((cr_word >> CHSEL_SHIFT) & 0x7) as u8;
        if chsel != 0 {
            chsel
        } else {
            s
        }
    }

    /// Whether stream `s` may transfer this tick: always, unless request
    /// lines are wired, in which case its selected bit must be asserted.
    fn requested(&self, cr_word: u32, s: u8) -> bool {
        let Some((reqs, controller)) = &self.requests else {
            return true;
        };
        reqs.lock()
            .unwrap_or_else(|e| e.into_inner())
            .asserted(*controller, Self::request_sel(cr_word, s))
    }

    /// Consume one request assertion on stream `s`'s selected bit.
    fn consume(&self, cr_word: u32, s: u8) {
        if let Some((reqs, controller)) = &self.requests {
            let sel = Self::request_sel(cr_word, s);
            let mut reqs = reqs.lock().unwrap_or_else(|e| e.into_inner());
            reqs.release(*controller, sel, true);
            reqs.release(*controller, sel, false);
        }
    }

    /// Latch a stream's period state from the current registers.
    fn latch_period(&mut self, view: &Registers<&mut [u32]>, s: u32) {
        let cr = S0CR + STREAM_STRIDE * s;
        if let Some(slot) = self.period.get_mut(s as usize) {
            *slot = Period {
                ndt0: (view.get(cr + SNDTR) & NDTR_MASK) as u16,
                cpar0: view.get(cr + SPAR),
                cmar0: view.get(cr + SM0AR),
            };
        }
        if let Some(slot) = self.m1ar0.get_mut(s as usize) {
            *slot = view.get(cr + SM1AR);
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
            // `SxCR` writes latch the period state on an `EN` 0→1 edge, so
            // `HTIF` and `CIRC`/`DBM` measure against the programmed transfer.
            // A burst programmed while direct (`DMDIS`=0) latches `DMEIF`;
            // the check runs on every `EN`-set write (not just the edge)
            // because firmware programs `SxCR` byte by byte and the burst
            // field arrives after the `EN` bit.
            offset if offset >= S0CR && (offset - S0CR) % STREAM_STRIDE == 0 => {
                let s = (offset - S0CR) / STREAM_STRIDE;
                let old = view.get(S0CR + s * STREAM_STRIDE);
                let word = view.merged(offset, val, shift);
                if s < u32::from(STREAMS) && old & EN == 0 && word & EN != 0 {
                    self.latch_period(view, s);
                }
                if s < u32::from(STREAMS) && word & EN != 0 {
                    let fcr = view.get(S0CR + s * STREAM_STRIDE + SFCR);
                    let burst = ((word >> MBURST_SHIFT) & 0x3).max((word >> PBURST_SHIFT) & 0x3);
                    if fcr & DMDIS == 0 && burst != 0 && (word >> DIR_SHIFT) & 0x3 != 2 {
                        let isr = stream_isr(s);
                        view.set(isr, view.get(isr) | dmeif_bit(s));
                    }
                }
                word
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        // Host-injected bus faults land first: `TEIF` latches and the stream
        // disables, matching silicon's error response. Injected FIFO errors
        // latch `FEIF` without stopping the stream.
        for stream in 0..STREAMS {
            if self.error & (1 << stream) != 0 {
                self.error &= !(1 << stream);
                let s = u32::from(stream);
                let isr = stream_isr(s);
                view.set(isr, view.get(isr) | teif_bit(s));
                let cr = S0CR + STREAM_STRIDE * s;
                view.set(cr, view.get(cr) & !EN);
            }
            if self.fifo_error & (1 << stream) != 0 {
                self.fifo_error &= !(1 << stream);
                let s = u32::from(stream);
                let isr = stream_isr(s);
                view.set(isr, view.get(isr) | feif_bit(s));
            }
        }
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
            if paced && !self.requested(cr_word, stream) {
                continue;
            }
            if paced {
                self.consume(cr_word, stream);
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
            // FIFO threshold gating (`SxFCR.FTH`) when FIFO mode is enabled
            // (`DMDIS`=1) on a peripheral direction: produced bytes
            // accumulate and release to the mailbox once the threshold is
            // reached; the residue flushes when `NDTR` exhausts. Direct mode
            // (and memory-to-memory, which is always direct here) passes
            // units straight through.
            let fcr = view.get(cr + SFCR);
            let fifo_on = fcr & DMDIS != 0 && dir != 2;
            let produced = chunk * u32::from(unit);
            let remain = ndt - chunk;
            // Update addresses after burst: the data left the source even
            // when the FIFO holds it.
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
            // Half-transfer: `NDTR` just reached half the programmed count
            // (`ndt0 >= 2`, so single-item transfers only complete).
            let ndt0 = u32::from(self.period.get(stream as usize).map_or(0, |p| p.ndt0));
            if ndt0 >= 2 && remain == ndt0 / 2 {
                let isr = stream_isr(s);
                view.set(isr, view.get(isr) | htif_bit(s));
            }
            if fifo_on {
                let idx = stream as usize;
                let thresh = u32::from(fth_bytes(fcr & FTH_MASK));
                let held = u32::from(self.fifo_lvl[idx]);
                if held + produced >= thresh || remain == 0 {
                    // Threshold reached (or end-of-transfer flush): release
                    // everything buffered, including this tick's production.
                    if let Some(buf) = self.fifo_buf.get_mut(idx) {
                        buf.extend(units);
                        let buffered = std::mem::take(buf);
                        self.mailbox
                            .lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .extend(buffered);
                    }
                    self.fifo_lvl[idx] = 0;
                } else {
                    // Below threshold: hold the bytes for a later tick.
                    if let Some(buf) = self.fifo_buf.get_mut(idx) {
                        buf.extend(units);
                    }
                    self.fifo_lvl[idx] = (held + produced) as u16;
                }
            } else {
                self.mailbox
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .extend(units);
            }
            if remain == 0 {
                let isr = stream_isr(s);
                // `CIRC` starts the next period instead of stopping; `DBM`
                // toggles `CT` so periods alternate `M0AR`/`M1AR`.
                if cr_word & CIRC != 0 {
                    let p = self
                        .period
                        .get(stream as usize)
                        .copied()
                        .unwrap_or_default();
                    view.set(cr + SNDTR, u32::from(p.ndt0));
                    view.set(cr + SPAR, p.cpar0);
                    if cr_word & DBM != 0 {
                        let to_m1 = view.get(cr) & CT == 0;
                        let mut active = view.get(cr);
                        if to_m1 {
                            active |= CT;
                        } else {
                            active &= !CT;
                        }
                        view.set(cr, active);
                        view.set(
                            cr + SM0AR,
                            if to_m1 {
                                self.m1ar0[stream as usize]
                            } else {
                                p.cmar0
                            },
                        );
                    } else {
                        view.set(cr + SM0AR, p.cmar0);
                    }
                    view.set(isr, view.get(isr) | tcif_bit(s));
                } else {
                    view.set(cr, cr_word & !EN);
                    view.set(isr, view.get(isr) | tcif_bit(s));
                }
            }
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let mut lines = 0;
        for stream in 0..STREAMS {
            let s = u32::from(stream);
            let cr = S0CR + STREAM_STRIDE * u32::from(stream);
            let isr = stream_isr(s);
            let cr_word = view.get(cr);
            let flags = view.get(isr);
            if cr_word & TCIE != 0 && flags & tcif_bit(s) != 0
                || cr_word & HTIE != 0 && flags & htif_bit(s) != 0
                || cr_word & TEIE != 0 && flags & teif_bit(s) != 0
                || cr_word & DMEIE != 0 && flags & dmeif_bit(s) != 0
            {
                lines |= 1 << s;
            }
        }
        // `FEIE` lives in `SxFCR` (bit 7), not `SxCR`.
        for stream in 0..STREAMS {
            let s = u32::from(stream);
            let cr = S0CR + STREAM_STRIDE * u32::from(stream);
            let isr = stream_isr(s);
            if view.get(cr + SFCR) & (1 << 7) != 0 && view.get(isr) & feif_bit(s) != 0 {
                lines |= 1 << s;
            }
        }
        lines
    }

    fn reset(&mut self) {
        self.period = [Period::default(); 8];
        self.m1ar0 = [0; 8];
        self.fifo_lvl = [0; 8];
        self.fifo_buf = [
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
        ];
        self.error = 0;
        self.fifo_error = 0;
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
    /// sources address streams by number while `CHSEL` is zero (board
    /// convention); a nonzero `CHSEL` remaps the stream to that channel.
    /// Boards wire the conventional routes (see the board constructors).
    #[must_use]
    pub fn with_requests(mut self, requests: Arc<Mutex<DmaRequests>>, controller: u8) -> Self {
        self.inner.behavior_mut().requests = Some((requests, controller));
        self
    }

    /// Inject a bus fault on stream `s` (0-based): the next tick latches
    /// `TEIF` and disables the stream. Drain-time bus errors cannot reach
    /// the device (the mailbox is one-way), so host injection is the only
    /// `TEIF` source.
    pub fn inject_error(&mut self, s: u8) {
        if s < STREAMS {
            self.inner.behavior_mut().error |= 1 << s;
        }
    }

    /// Inject a FIFO overrun/underrun on stream `s` (0-based): the next tick
    /// latches `FEIF` (gated by `SxFCR` `FEIE` for the line) without
    /// disabling the stream. The discrete tick model drains the FIFO
    /// atomically with production, so genuine contention overruns are
    /// unobservable and this host hook is the only `FEIF` source.
    pub fn inject_fifo_error(&mut self, s: u8) {
        if s < STREAMS {
            self.inner.behavior_mut().fifo_error |= 1 << s;
        }
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
        write_word(&mut d, D1 + s_cr(2), EN | TCIE | (1 << 9) | (1 << 10));
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

    fn s_fcr(s: u32) -> u32 {
        s_cr(s) + 0x14
    }

    fn s_m1ar(s: u32) -> u32 {
        s_cr(s) + 0x10
    }

    #[test]
    fn htif_latches_at_half_ndtr() {
        use crate::device::stm32::dma::DmaRequests;
        let m = mailbox();
        let reqs = Arc::new(Mutex::new(DmaRequests::new()));
        let mut d = DmaF4::dma1_for::<Stm32f407>(m).with_requests(reqs.clone(), 0);
        // Stream 0, 4 units, HTIE on: HTIF (LISR bit 3) at NDTR=2.
        write_word(&mut d, D1 + s_par(0), 0x4000_0400);
        write_word(&mut d, D1 + s_m0ar(0), 0x2000_1000);
        write_word(&mut d, D1 + s_ndtr(0), 4);
        write_word(&mut d, D1 + s_cr(0), EN | (1 << 10) | (1 << 3)); // MINC|HTIE
        for remaining in [3, 2] {
            reqs.lock()
                .unwrap_or_else(|e| e.into_inner())
                .request(0, 0, false);
            d.tick(1);
            assert_eq!(read_word(&mut d, D1 + s_ndtr(0)), remaining);
        }
        assert_eq!(read_word(&mut d, D1 + LISR) & (1 << 3), 1 << 3, "HTIF0");
        assert_eq!(d.pending(), 0x1, "HTIE raises stream-0 line");
    }

    #[test]
    fn circ_reloads_period_and_keeps_en() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone());
        // Stream 1, M2M, 2 units, CIRC + TCIE.
        write_word(&mut d, D1 + s_par(1), 0x1000);
        write_word(&mut d, D1 + s_m0ar(1), 0x2000);
        write_word(&mut d, D1 + s_ndtr(1), 2);
        write_word(
            &mut d,
            D1 + s_cr(1),
            EN | (1 << 4) | (2 << 6) | (1 << 8) | (1 << 10),
        ); // TCIE|M2M|CIRC|MINC
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(1)), 2, "NDTR reloaded");
        assert_ne!(read_word(&mut d, D1 + s_cr(1)) & EN, 0, "EN kept");
        assert_eq!(read_word(&mut d, D1 + LISR) & (1 << 11), 1 << 11, "TCIF1");
        d.tick(1);
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner()).len(),
            4,
            "two periods moved four units"
        );
        assert_eq!(read_word(&mut d, D1 + s_ndtr(1)), 2, "reloaded again");
    }

    #[test]
    fn dbm_alternates_m0ar_and_m1ar() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone());
        // Stream 1, M2M, 2 units, CIRC + DBM, two buffer bases.
        write_word(&mut d, D1 + s_par(1), 0x1000);
        write_word(&mut d, D1 + s_m0ar(1), 0x2000);
        write_word(&mut d, D1 + s_m1ar(1), 0x3000);
        write_word(&mut d, D1 + s_ndtr(1), 2);
        write_word(
            &mut d,
            D1 + s_cr(1),
            EN | (2 << 6) | (1 << 8) | (1 << 10) | (1 << 18),
        ); // M2M|CIRC|MINC|DBM
        d.tick(1);
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner())[0],
            (0x1000, 0x2000, 1),
            "first period targets M0AR"
        );
        assert_ne!(read_word(&mut d, D1 + s_cr(1)) & (1 << 19), 0, "CT set");
        d.tick(1);
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner())[2],
            (0x1000, 0x3000, 1),
            "second period targets M1AR"
        );
        assert_eq!(read_word(&mut d, D1 + s_cr(1)) & (1 << 19), 0, "CT back");
    }

    #[test]
    fn chsel_nonzero_remaps_request_routing() {
        use crate::device::stm32::dma::DmaRequests;
        let m = mailbox();
        let reqs = Arc::new(Mutex::new(DmaRequests::new()));
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone()).with_requests(reqs.clone(), 0);
        // Stream 2 with CHSEL=5 listens to channel 5, not stream 2.
        write_word(&mut d, D1 + s_par(2), 0x4000_0400);
        write_word(&mut d, D1 + s_m0ar(2), 0x2000_1000);
        write_word(&mut d, D1 + s_ndtr(2), 2);
        write_word(&mut d, D1 + s_cr(2), EN | (1 << 10) | (5 << 25)); // MINC|CHSEL=5
                                                                      // A request on the stream's own number stalls.
        reqs.lock()
            .unwrap_or_else(|e| e.into_inner())
            .request(0, 2, false);
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(2)), 2, "wrong channel stalls");
        assert!(m.lock().unwrap_or_else(|e| e.into_inner()).is_empty());
        // A request on channel 5 transfers.
        reqs.lock()
            .unwrap_or_else(|e| e.into_inner())
            .request(0, 5, false);
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + s_ndtr(2)), 1);
        assert_eq!(m.lock().unwrap_or_else(|e| e.into_inner()).len(), 1);
    }

    #[test]
    fn injected_errors_latch_flags_and_lines() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m);
        // TEIF on stream 0 (LISR bit 2), TEIE on.
        write_word(&mut d, D1 + s_par(0), 0x1000);
        write_word(&mut d, D1 + s_m0ar(0), 0x2000);
        write_word(&mut d, D1 + s_ndtr(0), 4);
        write_word(&mut d, D1 + s_cr(0), EN | (1 << 2)); // TEIE
        d.inject_error(0);
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + LISR) & 0x4, 0x4, "TEIF0");
        assert_eq!(read_word(&mut d, D1 + s_cr(0)) & EN, 0, "EN cleared");
        assert_eq!(d.pending(), 0x1, "TEIE raises stream-0 line");
        // FEIF on stream 1 (LISR bit 6), FEIE (FCR bit 7) on.
        write_word(&mut d, D1 + s_fcr(1), 1 << 7); // FEIE
        d.inject_fifo_error(1);
        d.tick(1);
        assert_eq!(read_word(&mut d, D1 + LISR) & (1 << 6), 1 << 6, "FEIF1");
        assert_eq!(d.pending(), 0x3, "both stream lines latched");
    }

    #[test]
    fn dmeif_latches_for_burst_in_direct_mode() {
        let m = mailbox();
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone());
        // Stream 0, memory→peripheral, INCR4 burst, direct mode (FCR reset
        // 0x21 has DMDIS=0), DMEIE on.
        write_word(&mut d, D1 + s_par(0), 0x4000_0400);
        write_word(&mut d, D1 + s_m0ar(0), 0x2000_1000);
        write_word(&mut d, D1 + s_ndtr(0), 4);
        write_word(
            &mut d,
            D1 + s_cr(0),
            EN | (1 << 1) | (1 << 6) | (1 << 10) | (1 << 23),
        ); // DMEIE|M2P|MINC|MBURST=INCR4
        assert_eq!(read_word(&mut d, D1 + LISR) & 0x2, 0x2, "DMEIF0");
        assert_eq!(d.pending(), 0x1, "DMEIE raises stream-0 line");
    }

    #[test]
    fn fifo_threshold_gates_and_flushes() {
        use crate::device::stm32::dma::DmaRequests;
        let m = mailbox();
        let reqs = Arc::new(Mutex::new(DmaRequests::new()));
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone()).with_requests(reqs.clone(), 0);
        // Stream 0, 4 byte units paced 1/tick, FIFO full threshold (16 B):
        // everything holds until NDTR exhausts, then flushes.
        write_word(&mut d, D1 + s_par(0), 0x4000_0400);
        write_word(&mut d, D1 + s_m0ar(0), 0x2000_1000);
        write_word(&mut d, D1 + s_ndtr(0), 4);
        write_word(&mut d, D1 + s_fcr(0), 0x21 | 0x04 | 0x03); // DMDIS|FTH=full
        write_word(&mut d, D1 + s_cr(0), EN | (1 << 10)); // MINC
        for _ in 0..3 {
            reqs.lock()
                .unwrap_or_else(|e| e.into_inner())
                .request(0, 0, false);
            d.tick(1);
        }
        assert_eq!(read_word(&mut d, D1 + s_ndtr(0)), 1, "source drained");
        assert!(
            m.lock().unwrap_or_else(|e| e.into_inner()).is_empty(),
            "below threshold the FIFO holds"
        );
        reqs.lock()
            .unwrap_or_else(|e| e.into_inner())
            .request(0, 0, false);
        d.tick(1);
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner()).len(),
            4,
            "end-of-transfer flush releases all four"
        );
        assert_eq!(read_word(&mut d, D1 + LISR) & (1 << 5), 1 << 5, "TCIF0");
    }

    #[test]
    fn fifo_quarter_threshold_releases_early() {
        use crate::device::stm32::dma::DmaRequests;
        let m = mailbox();
        let reqs = Arc::new(Mutex::new(DmaRequests::new()));
        let mut d = DmaF4::dma1_for::<Stm32f407>(m.clone()).with_requests(reqs.clone(), 0);
        // Stream 0, 8 byte units paced 1/tick, FIFO quarter threshold (4 B):
        // the first four release at tick 4, the rest at the end.
        write_word(&mut d, D1 + s_par(0), 0x4000_0400);
        write_word(&mut d, D1 + s_m0ar(0), 0x2000_1000);
        write_word(&mut d, D1 + s_ndtr(0), 8);
        write_word(&mut d, D1 + s_fcr(0), 0x20 | 0x04); // DMDIS|FTH=quarter
        write_word(&mut d, D1 + s_cr(0), EN | (1 << 10)); // MINC
        for _ in 0..3 {
            reqs.lock()
                .unwrap_or_else(|e| e.into_inner())
                .request(0, 0, false);
            d.tick(1);
        }
        assert!(
            m.lock().unwrap_or_else(|e| e.into_inner()).is_empty(),
            "3 B held below the 4 B threshold"
        );
        reqs.lock()
            .unwrap_or_else(|e| e.into_inner())
            .request(0, 0, false);
        d.tick(1);
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner()).len(),
            4,
            "threshold reached at 4 B"
        );
        // The second four buffer the same way and flush at exhaustion.
        for _ in 0..4 {
            reqs.lock()
                .unwrap_or_else(|e| e.into_inner())
                .request(0, 0, false);
            d.tick(1);
        }
        assert_eq!(
            m.lock().unwrap_or_else(|e| e.into_inner()).len(),
            8,
            "residue flushes at NDTR exhaustion"
        );
    }
}
