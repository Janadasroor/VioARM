//! STM32 FMC (Flexible Memory Controller) and QUADSPI/OCTOSPI stubs for H7.
//! The control registers at `0x52004000` (FMC) and `0x52005000` (QUADSPI) are
//! exposed as a register window that does not fault; the external memory
//! itself lives at `0xC0000000` (FMC SDRAM) and `0x90000000` (QSPI) as RAM
//! regions on the board.

use crate::device::svd::{Behavior, Device, NoBehavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// Flexible Memory Controller.
#[derive(Debug)]
pub struct Fmc {
    inner: RegisterFile<NoBehavior>,
}

impl Fmc {
    /// A new `Fmc` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("FMC").expect("generated tables contain FMC");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
}

impl Peripheral for Fmc {
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
        "fmc"
    }
}

impl InterruptSource for Fmc {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

/// Shared QUADSPI gate: the external `0x9000_0000` QSPI window is only
/// backed when `QUADSPI->CR.EN` (bit 0) is set, mirroring silicon where the
/// memory-mapped mode is `CR.EN == 1`. The gate is shared between the
/// `QUADSPI` register device (which drives `EN`) and the `SystemMemory` bus
/// (which drops `0x9000_0000` accesses while disabled). At reset `CR.EN`
/// is 0 so the window is gated; firmware must set `CR.EN=1` before touching
/// `0x90000000` (see `stm32h743_fmc_qspi` and `qspi_gate` e2es).
#[derive(Debug)]
pub struct QspiGate {
    /// The QSPI memory-mapped region `0x9000_0000 + len`.
    pub region: Region,
    /// Whether `CR.EN` is set (memory-mapped window is backed).
    pub enabled: bool,
}

impl QspiGate {
    /// A new `QspiGate`.
    #[must_use]
    pub const fn new(region: Region) -> Self {
        Self {
            region,
            enabled: false,
        }
    }
    /// Reset to gated (window reads zero until `CR.EN`).
    pub fn reset(&mut self) {
        self.enabled = false;
    }
}

#[derive(Debug)]
struct QspiBehavior {
    gate: Option<Arc<Mutex<QspiGate>>>,
}

impl QspiBehavior {
    const fn new(gate: Option<Arc<Mutex<QspiGate>>>) -> Self {
        Self { gate }
    }
}

impl Behavior for QspiBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset == 0x00 {
            if let Some(g) = &self.gate {
                g.lock().unwrap_or_else(|e| e.into_inner()).enabled = word & 1 != 0;
            }
        }
        word
    }
    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, _offset: u32, value: u32) -> u32 {
        value
    }
    fn reset(&mut self) {
        if let Some(g) = &self.gate {
            g.lock().unwrap_or_else(|e| e.into_inner()).reset();
        }
    }
}

/// Quad-SPI controller.
#[derive(Debug)]
pub struct Qspi {
    inner: RegisterFile<QspiBehavior>,
}

impl Qspi {
    /// A new `Qspi` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        Self::new_for_with_gate::<D>(None)
    }
    /// A new window with an optional gate.
    #[must_use]
    pub fn new_for_with_gate<D: Device>(gate: Option<Arc<Mutex<QspiGate>>>) -> Self {
        // H7 names it QUADSPI, F7 QSPI, H7 also OCTOSPI1/2 - try all
        let def = D::peripheral("QUADSPI")
            .or_else(|| D::peripheral("QSPI"))
            .or_else(|| D::peripheral("OCTOSPI1"))
            .expect("generated tables contain QUADSPI/QSPI/OCTOSPI1");
        Self {
            inner: RegisterFile::new(def, QspiBehavior::new(gate)),
        }
    }

    /// A new `OCTOSPI2` on another chip's device tables.
    #[must_use]
    pub fn octospi2_for<D: Device>() -> Self {
        Self::octospi2_for_with_gate::<D>(None)
    }
    /// A new `OCTOSPI2` with an optional gate.
    #[must_use]
    pub fn octospi2_for_with_gate<D: Device>(gate: Option<Arc<Mutex<QspiGate>>>) -> Self {
        let def = D::peripheral("OCTOSPI2").expect("generated tables contain OCTOSPI2");
        Self {
            inner: RegisterFile::new(def, QspiBehavior::new(gate)),
        }
    }
}

impl Peripheral for Qspi {
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
        "qspi"
    }
}

impl InterruptSource for Qspi {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

/// Request routing shared between a `DMAMUX1` and its DMA controllers.
///
/// DMAMUX1 channels 0..=7 feed DMA1 streams 0..=7 and channels 8..=15 feed
/// DMA2 streams 0..=7. Each channel's `CCR.DMAREQ_ID` selects the request
/// input; `0` (reset) keeps the board convention (stream-numbered request
/// bits), nonzero remaps the stream to that input's bit. Sync/trigger
/// fields (`SE`/`SPOL`/`NBREQ`/`SYNC_ID`), event generation (`EGE`), and
/// the request-generator block (`RGxCR`/`RGSR`/`RGCFR`) are latched no-ops:
/// the registers round-trip, nothing routes through them.
#[derive(Debug, Default)]
pub struct DmamuxRoutes {
    /// `DMAREQ_ID` per DMAMUX1 channel (7-bit values).
    pub ccr_id: [u8; 16],
    /// Host-injected sync-overrun flags, materialized into `CSR` on read.
    /// Genuine overruns need sync counting, which is unmodeled, so this
    /// host hook is the only `SOF` source.
    pub overrun: u16,
}

impl DmamuxRoutes {
    /// Empty routing: every channel at reset (`0` = board convention).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
}

/// `DMAMUX_CCRx` stride: 16 channels at `0x00..0x40`.
const DMAMUX_CCR_STRIDE: u32 = 0x04;
/// `DMAMUX_CSR` holds the `SOFx` sync-overrun flags (bit per channel).
const DMAMUX_CSR: u32 = 0x80;
/// `DMAMUX_CFR` write-1-to-clears `CSR`.
const DMAMUX_CFR: u32 = 0x84;
/// `DMAMUX_CCRx.SOIE`: sync-overrun interrupt enable (bit 8).
const DMAMUX_SOIE: u32 = 1 << 8;

#[derive(Debug, Default)]
struct DmamuxBehavior {
    /// Shared routing when this multiplexer drives DMA streams (`DMAMUX1`
    /// on routed boards); `None` stays a pure latch (`DMAMUX2`/BDMA).
    routes: Option<Arc<Mutex<DmamuxRoutes>>>,
}

impl Behavior for DmamuxBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        if offset < DMAMUX_CSR && offset % DMAMUX_CCR_STRIDE == 0 {
            // `CCRx`: latch `DMAREQ_ID` into the shared routing.
            let ch = (offset / DMAMUX_CCR_STRIDE) as usize;
            if let Some(routes) = self.routes.as_ref() {
                if let Ok(mut routes) = routes.lock() {
                    if let Some(slot) = routes.ccr_id.get_mut(ch) {
                        *slot = (word & 0x7F) as u8;
                    }
                }
            }
        } else if offset == DMAMUX_CFR {
            // `CFR` write-1-to-clears `CSR`.
            view.set(DMAMUX_CSR, view.get(DMAMUX_CSR) & !word);
        }
        word
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == DMAMUX_CSR {
            // Materialize host-injected overruns into `CSR` on read.
            if let Some(routes) = self.routes.as_ref() {
                if let Ok(mut routes) = routes.lock() {
                    let overrun = std::mem::take(&mut routes.overrun);
                    if overrun != 0 {
                        let merged = value | u32::from(overrun);
                        view.set(DMAMUX_CSR, merged);
                        return merged;
                    }
                }
            }
        }
        value
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        // One shared overrun line: any `CSR.SOFx` with its `CCRx.SOIE`.
        let mut csr = view.get(DMAMUX_CSR);
        if let Some(routes) = self.routes.as_ref() {
            if let Ok(routes) = routes.lock() {
                csr |= u32::from(routes.overrun);
            }
        }
        for ch in 0..16u32 {
            if csr & (1 << ch) != 0 && view.get(ch * DMAMUX_CCR_STRIDE) & DMAMUX_SOIE != 0 {
                return 1;
            }
        }
        0
    }
}

/// DMAMUX request router.
pub struct Dmamux {
    inner: RegisterFile<DmamuxBehavior>,
}

impl Dmamux {
    /// A new `Dmamux` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DMAMUX1")
            .or_else(|| D::peripheral("DMAMUX"))
            .expect("generated tables contain DMAMUX1");
        Self {
            inner: RegisterFile::new(def, DmamuxBehavior::default()),
        }
    }
    /// A new DMAMUX2 (BDMA side).
    #[must_use]
    pub fn dmamux2_for<D: Device>() -> Self {
        let def = D::peripheral("DMAMUX2").expect("generated tables contain DMAMUX2");
        Self {
            inner: RegisterFile::new(def, DmamuxBehavior::default()),
        }
    }

    /// Share request routing with the DMA controllers: streams transfer only
    /// when their DMAMUX channel selects the asserted input (`0` keeps the
    /// board convention). Boards pass the same table to their `DmaF4`
    /// controllers via `DmaF4::with_dmamux`.
    #[must_use]
    pub fn with_routes(mut self, routes: Arc<Mutex<DmamuxRoutes>>) -> Self {
        self.inner.behavior_mut().routes = Some(routes);
        self
    }

    /// Inject a sync-overrun on DMAMUX channel `ch` (0-based): the next
    /// `CSR` read latches `SOFx`, raising the shared overrun line when that
    /// channel's `SOIE` is set. Genuine overruns need sync counting, which
    /// is unmodeled, so this host hook is the only `SOF` source.
    pub fn inject_overrun(&mut self, ch: u8) {
        if ch < 16 {
            if let Some(routes) = self.inner.behavior_mut().routes.as_ref() {
                if let Ok(mut routes) = routes.lock() {
                    routes.overrun |= 1 << ch;
                }
            }
        }
    }
}

impl Peripheral for Dmamux {
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
        "dmamux"
    }
}

impl InterruptSource for Dmamux {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

// ---------------------------------------------------------------------------
// H7 high-integration peripherals: LTDC display layers/reload/line-IRQ and
// DMA2D pixel fills/copies run behaviorally (below); BDMA/MDMA/SAI/DFSDM/
// HRTIM keep the EN/START handshake (RM0433 §11 BDMA, §12 MDMA, §53 SAI,
// §22 DFSDM, §26 HRTIM).
// ---------------------------------------------------------------------------

#[derive(Debug, Default)]
struct LtdcBehavior {
    enabled: bool,
    pending: bool,
    /// Current scanline (0..frame lines).
    line: u32,
    /// Layer enable latched (`LEN`) vs active (applied at reload).
    len_armed: [bool; 2],
    len_active: [bool; 2],
    /// A `VBR` reload waits for the next frame wrap.
    vbr_armed: bool,
}

impl LtdcBehavior {
    fn raise_isr(view: &mut Registers<&mut [u32]>, bits: u32) {
        if let Some(mut isr) = view.try_get(0x38) {
            isr |= bits;
            view.try_set(0x38, isr);
        } else {
            view.try_set(0x38, bits);
        }
    }

    fn sync_pending(&mut self, view: &mut Registers<&mut [u32]>) {
        let isr = view.try_get(0x38).unwrap_or(0);
        let ier = view.try_get(0x34).unwrap_or(0);
        if isr & ier & 0xF != 0 {
            self.pending = true;
        }
    }
}

impl Behavior for LtdcBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        match offset {
            // GCR LTDCEN bit0.
            0x18 => {
                self.enabled = word & 1 != 0;
                if !self.enabled {
                    self.line = 0;
                    self.vbr_armed = false;
                }
            }
            // SRCR: IMR reloads now, VBR reloads at the next frame wrap;
            // both latch RRIF. Bits are self-clearing pulses.
            0x24 => {
                if word & 1 != 0 {
                    self.len_active = self.len_armed;
                    Self::raise_isr(view, 1 << 3);
                }
                if word & (1 << 1) != 0 {
                    self.vbr_armed = true;
                }
                self.sync_pending(view);
                return word & !0x3;
            }
            // Layer enables arm at `LEN`, applying at the next reload.
            0x84 | 0x104 => {
                let i = if offset == 0x84 { 0 } else { 1 };
                self.len_armed[i] = word & 1 != 0;
            }
            // ICR write-1-to-clear ISR bits at 0x38.
            0x3c => {
                if let Some(mut isr) = view.try_get(0x38) {
                    // bits 3:0
                    let clear = word & 0xF;
                    isr &= !clear;
                    view.try_set(0x38, isr);
                }
                // clearing ISR also clears pending if no flag remains
                if view.try_get(0x38).unwrap_or(0) & 0xF == 0 {
                    self.pending = false;
                }
                // ICR is WO, don't store
                return view.get(offset) & !(0xFFu32 << shift)
                    | ((word & 0xF) << shift) & (0xFFu32 << shift);
            }
            _ => {}
        }
        word
    }
    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        // CPSR: live scan position (Y from the line counter, X frozen —
        // no pixel clock is modeled).
        if offset == 0x44 {
            return self.line & 0x7FF;
        }
        value
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !self.enabled || delta == 0 {
            return;
        }
        // One tick is one scanline; a frame is `TOTALH + 1` lines
        // (registers hold total − 1, RM0433 § LTDC).
        let totalh = view.try_get(0x14).unwrap_or(0) & 0x7FF;
        let frame = totalh + 1;
        let lip = view.try_get(0x40).unwrap_or(0) & 0x7FF;
        // Line interrupt: the scan passes through `LIPOS` during the
        // advance (a deferred full frame away when already on it, so a
        // cleared flag never refires mid-line).
        if lip < frame {
            let mut dist = (lip + frame - (self.line % frame)) % frame;
            if dist == 0 {
                dist = frame;
            }
            if u64::from(dist) <= delta {
                Self::raise_isr(view, 1);
            }
        }
        let next = self.line + delta as u32;
        if next >= frame {
            // Frame wrap: a deferred `VBR` reload applies, with its flag.
            if self.vbr_armed {
                self.len_active = self.len_armed;
                Self::raise_isr(view, 1 << 3);
                self.vbr_armed = false;
            }
        }
        self.line = next % frame;
        self.sync_pending(view);
    }
    fn pending(&self, _view: &Registers<&[u32]>) -> u32 {
        if self.pending {
            1
        } else {
            0
        }
    }
    fn acknowledge(&mut self, _lines: u32) {
        self.pending = false;
    }
    fn reset(&mut self) {
        self.enabled = false;
        self.pending = false;
        self.line = 0;
        self.len_armed = [false; 2];
        self.len_active = [false; 2];
        self.vbr_armed = false;
    }
}

/// Display controller: enable, per-layer enables with immediate or
/// vertical-blanking reload (`RRIF`), line-position interrupt against a
/// scanline counter (`TOTALH + 1` lines per frame), and live `CPSR.Y`.
/// Layer framebuffers, pixel formats, and blending stay latches — no
/// pixel pipeline is modeled (documented).
#[derive(Debug)]
pub struct Ltdc {
    inner: RegisterFile<LtdcBehavior>,
}

impl Ltdc {
    /// A new `Ltdc` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("LTDC").expect("generated tables contain LTDC");
        Self {
            inner: RegisterFile::new(def, LtdcBehavior::default()),
        }
    }
}

impl Peripheral for Ltdc {
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
        "ltdc"
    }
}

impl InterruptSource for Ltdc {
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
        2
    }
}

/// One DMA2D source-row read for the bus to perform.
#[derive(Clone, Copy, Debug)]
pub struct Dma2dRead {
    /// First byte address.
    pub addr: u32,
    /// Byte count.
    pub len: u32,
    /// Destination row (0-based).
    pub row: u32,
}

/// A completed source-row read, correlated by row.
#[derive(Clone, Debug)]
pub struct Dma2dResult {
    /// Echo of [`Dma2dRead::row`].
    pub row: u32,
    /// The bytes read (short on reserved addresses).
    pub bytes: Vec<u8>,
}

/// The DMA2D controller/bus mailbox, shared like the DMA/Ethernet ones.
#[derive(Debug, Default)]
pub struct Dma2dMailbox {
    /// Pending row reads for the bus.
    pub reads: Vec<Dma2dRead>,
    /// Read results, for the controller to consume.
    pub results: VecDeque<Dma2dResult>,
    /// Pending `(address, bytes)` writes for the bus.
    pub writes: Vec<(u32, Vec<u8>)>,
}

/// Pixel-format code (`xPFCCR.CM`) to bytes per pixel. ARGB8888/RGB888/
/// RGB565/ARGB1555/ARGB4444/L8 only; CLUT and alpha-only formats latch
/// `TEIF` (documented).
fn dma2d_bpp(cm: u32) -> Option<usize> {
    match cm {
        0 => Some(4),
        1 => Some(3),
        2..=4 => Some(2),
        5 => Some(1),
        _ => None,
    }
}

/// One pixel to full ARGB. Memory order is little-endian words
/// (`0xAARRGGBB` reads `B G R A`); RGB888 reads `B G R`; 1555/4444 expand
/// their fields; L8 is white scaled by the byte (documented contract).
fn dma2d_to_argb(fmt: u32, px: &[u8]) -> (u8, u8, u8, u8) {
    match fmt {
        0 => (px[3], px[2], px[1], px[0]),
        1 => (255, px[2], px[1], px[0]),
        2 => {
            let w = u16::from_le_bytes([px[0], px[1]]);
            let r = ((w >> 11) & 0x1F) as u8;
            let g = ((w >> 5) & 0x3F) as u8;
            let b = (w & 0x1F) as u8;
            (
                255,
                (r << 3) | (r >> 2),
                (g << 2) | (g >> 4),
                (b << 3) | (b >> 2),
            )
        }
        3 => {
            let w = u16::from_le_bytes([px[0], px[1]]);
            let a = if w & 0x8000 != 0 { 255 } else { 0 };
            let r = ((w >> 10) & 0x1F) as u8;
            let g = ((w >> 5) & 0x1F) as u8;
            let b = (w & 0x1F) as u8;
            (
                a,
                (r << 3) | (r >> 2),
                (g << 3) | (g >> 2),
                (b << 3) | (b >> 2),
            )
        }
        4 => {
            let w = u16::from_le_bytes([px[0], px[1]]);
            let expand = |v: u16| (v & 0xF) as u8 * 0x11;
            (expand(w >> 12), expand(w >> 8), expand(w >> 4), expand(w))
        }
        _ => {
            // L8: white scaled by luminance.
            let y = px[0];
            (255, y, y, y)
        }
    }
}

/// Full ARGB to one output pixel (inverse of [`dma2d_to_argb`]).
fn dma2d_from_argb(fmt: u32, a: u8, r: u8, g: u8, b: u8) -> Vec<u8> {
    match fmt {
        0 => vec![b, g, r, a],
        1 => vec![b, g, r],
        2 => {
            let w = (u16::from(r >> 3) << 11) | (u16::from(g >> 2) << 5) | u16::from(b >> 3);
            w.to_le_bytes().to_vec()
        }
        3 => {
            let w = (u16::from(a > 127) << 15)
                | (u16::from(r >> 3) << 10)
                | (u16::from(g >> 3) << 5)
                | u16::from(b >> 3);
            w.to_le_bytes().to_vec()
        }
        4 => {
            let n = |v: u8| (v >> 4) as u16;
            (n(a) << 12 | n(r) << 8 | n(g) << 4 | n(b))
                .to_le_bytes()
                .to_vec()
        }
        _ => vec![(u16::from(r) + u16::from(g) + u16::from(b) + 2) as u8 / 3],
    }
}

/// A latched DMA2D transfer (register snapshot at `START`).
#[derive(Debug)]
struct Dma2dXfer {
    mode: u32,
    omar: u32,
    fgmar: u32,
    oor: u32,
    fgor: u32,
    nl: u32,
    pl: u32,
    out_cm: u32,
    fg_cm: u32,
    ocolr: u32,
    rows_done: u32,
}

#[derive(Debug, Default)]
struct Dma2dBehavior {
    active: bool,
    susp: bool,
    /// `START` requested but not yet executed: the transfer snapshot runs
    /// at the next tick, when all `CR` lanes (notably `MODE`) have landed.
    /// (Starting on lane 0 would run a half-programmed transfer.)
    start_armed: bool,
    elapsed: u64,
    ticks_total: u64,
    pending: bool,
    mailbox: Option<Arc<Mutex<Dma2dMailbox>>>,
    xfer: Option<Dma2dXfer>,
}

impl Dma2dBehavior {
    fn raise_isr(view: &mut Registers<&mut [u32]>, bits: u32) {
        let isr = view.try_get(0x04).unwrap_or(0) | bits;
        view.try_set(0x04, isr);
    }

    fn sync_pending(&mut self, view: &mut Registers<&mut [u32]>) {
        let isr = view.try_get(0x04).unwrap_or(0);
        let cr = view.try_get(0x00).unwrap_or(0);
        let ier = (cr >> 8) & 0x3F;
        if isr & ier & 0x3F != 0 {
            self.pending = true;
        }
    }

    /// Latch the transfer config at `START`; validate modes and pixel
    /// formats (`TEIF` otherwise), then run flag-only or fetch rows.
    fn start(&mut self, view: &mut Registers<&mut [u32]>) {
        let cr = view.try_get(0x00).unwrap_or(0);
        let mode = (cr >> 16) & 0x3;
        let nl = view.try_get(0x44).unwrap_or(0) & 0xFFFF;
        let pl = (view.try_get(0x44).unwrap_or(0) >> 16) & 0x3FFF;
        let out_cm = view.try_get(0x34).unwrap_or(0) & 0x7;
        let fg_cm = view.try_get(0x1c).unwrap_or(0) & 0x7;
        let out_bpp = dma2d_bpp(out_cm);
        let fg_bpp = dma2d_bpp(fg_cm);
        // Blend needs an alpha compositor, CLUT/alpha formats need tables:
        // visible error instead of a silent wrong blit (documented).
        if mode == 2 || out_bpp.is_none() || (mode != 3 && fg_bpp.is_none()) {
            Self::raise_isr(view, 1);
            self.sync_pending(view);
            Self::clear_start(view);
            return;
        }
        // Empty rectangles complete on the first tick, moving nothing
        // (ticks_total floors at 1 below).
        let pixels = u64::from(nl) * u64::from(pl);
        self.ticks_total = (pixels / 64).max(1);
        self.elapsed = 0;
        let out_bpp = out_bpp.unwrap_or(4);
        let fg_bpp = fg_bpp.unwrap_or(4);
        let xfer = Dma2dXfer {
            mode,
            omar: view.try_get(0x3c).unwrap_or(0),
            fgmar: view.try_get(0x0c).unwrap_or(0),
            oor: view.try_get(0x40).unwrap_or(0) & 0x3FFF,
            fgor: view.try_get(0x10).unwrap_or(0) & 0x3FFF,
            nl,
            pl,
            out_cm,
            fg_cm,
            ocolr: view.try_get(0x38).unwrap_or(0),
            rows_done: 0,
        };
        // Memory-to-memory fetches every source row up front; register
        // fills and flag-only runs finish on the tick count instead.
        if let (Some(mb), 0 | 1) = (&self.mailbox, mode) {
            let fg = u32::try_from(fg_bpp).unwrap_or(u32::MAX);
            let row_bytes = xfer.pl * fg;
            let stride = row_bytes + xfer.fgor * fg;
            let mut mb = mb.lock().unwrap_or_else(|e| e.into_inner());
            for y in 0..xfer.nl {
                mb.reads.push(Dma2dRead {
                    addr: xfer.fgmar.wrapping_add(y.wrapping_mul(stride)),
                    len: row_bytes,
                    row: y,
                });
            }
        }
        self.xfer = Some(xfer);
        self.active = true;
    }

    /// Finish: `TCIF`, `START` clears, transfer drops.
    fn complete(&mut self, view: &mut Registers<&mut [u32]>) {
        Self::raise_isr(view, 1 << 1);
        Self::clear_start(view);
        self.active = false;
        self.xfer = None;
        self.sync_pending(view);
    }

    /// Cancel: `CAEIF`, `START` clears, queued traffic drops.
    fn abort(&mut self, view: &mut Registers<&mut [u32]>) {
        Self::raise_isr(view, 1 << 3);
        Self::clear_start(view);
        self.active = false;
        self.susp = false;
        self.xfer = None;
        if let Some(mb) = &self.mailbox {
            let mut mb = mb.lock().unwrap_or_else(|e| e.into_inner());
            mb.reads.clear();
            mb.results.clear();
        }
        self.sync_pending(view);
    }

    fn clear_start(view: &mut Registers<&mut [u32]>) {
        if let Some(cr) = view.try_get(0x00) {
            view.try_set(0x00, cr & !1);
        }
    }

    /// Synthesize register-fill rows at completion (one converted color
    /// per pixel, pitched by `OOR`).
    fn fill_rect(&mut self, view: &mut Registers<&mut [u32]>) {
        let _ = view;
        let Some(x) = self.xfer.as_ref() else {
            return;
        };
        let px = x.ocolr.to_be_bytes();
        let (a, r, g, b) = (px[0], px[1], px[2], px[3]);
        let out_bpp = u32::try_from(dma2d_bpp(x.out_cm).unwrap_or(4)).unwrap_or(u32::MAX);
        let pl = usize::try_from(x.pl).unwrap_or(usize::MAX);
        let mut row = Vec::with_capacity(pl * out_bpp as usize);
        for _ in 0..pl {
            row.extend(dma2d_from_argb(x.out_cm, a, r, g, b));
        }
        let stride = x.pl * out_bpp + x.oor * out_bpp;
        if let Some(mb) = &self.mailbox {
            let mut mb = mb.lock().unwrap_or_else(|e| e.into_inner());
            for y in 0..x.nl {
                mb.writes
                    .push((x.omar.wrapping_add(y.wrapping_mul(stride)), row.clone()));
            }
        }
    }

    /// Consume arrived source rows: convert and emit destination writes.
    /// Rows must arrive in order (the bus preserves request order); a
    /// short row pads with zeros.
    fn drain_rows(&mut self, view: &mut Registers<&mut [u32]>) {
        let lwr = view.try_get(0x48).unwrap_or(0) & 0xFFFF;
        let mut rows = Vec::new();
        if let Some(mb) = &self.mailbox {
            let mut mb = mb.lock().unwrap_or_else(|e| e.into_inner());
            while let Some(r) = mb.results.pop_front() {
                rows.push(r);
            }
        }
        for r in rows {
            let done = match self.xfer.as_mut() {
                Some(x) if r.row == x.rows_done => {
                    let out_bpp =
                        u32::try_from(dma2d_bpp(x.out_cm).unwrap_or(4)).unwrap_or(u32::MAX);
                    let fg_bpp = u32::try_from(dma2d_bpp(x.fg_cm).unwrap_or(4)).unwrap_or(u32::MAX);
                    let pl = usize::try_from(x.pl).unwrap_or(usize::MAX);
                    let mut out = Vec::with_capacity(pl * out_bpp as usize);
                    let mut src = r.bytes;
                    src.resize(pl * fg_bpp as usize, 0);
                    for px in src.chunks_exact(fg_bpp as usize) {
                        let (a, rr, g, b) = dma2d_to_argb(x.fg_cm, px);
                        out.extend(dma2d_from_argb(x.out_cm, a, rr, g, b));
                    }
                    let stride = x.pl * out_bpp + x.oor * out_bpp;
                    let addr = x.omar.wrapping_add(r.row.wrapping_mul(stride));
                    if let Some(mb) = &self.mailbox {
                        mb.lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .writes
                            .push((addr, out));
                    }
                    x.rows_done += 1;
                    x.rows_done
                }
                _ => {
                    // No transfer, or out of order: park it back.
                    if let Some(mb) = &self.mailbox {
                        mb.lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .results
                            .push_front(r);
                    }
                    break;
                }
            };
            if lwr != 0 && done == lwr {
                Self::raise_isr(view, 1 << 2);
                self.sync_pending(view);
            }
        }
    }
}

impl Behavior for Dma2dBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        match offset {
            // CR: START arms for the next tick (full word converged),
            // SUSP pauses, ABORT cancels with CAEIF. ABORT is a pulse.
            0x00 => {
                if word & (1 << 2) != 0 {
                    self.start_armed = false;
                    self.abort(view);
                    return word & !(1 << 2);
                }
                self.susp = word & (1 << 1) != 0;
                if word & 1 != 0 && !self.active {
                    self.start_armed = true;
                }
            }
            0x08 => {
                // IFCR write-1-to-clear ISR bits at 0x04
                if let Some(mut isr) = view.try_get(0x04) {
                    let clear = word & 0x3F;
                    isr &= !clear;
                    view.try_set(0x04, isr);
                }
                if view.try_get(0x04).unwrap_or(0) & 0x3F == 0 {
                    self.pending = false;
                }
                return view.get(offset);
            }
            _ => {}
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if self.start_armed && !self.active && !self.susp {
            self.start_armed = false;
            self.start(view);
        }
        if !self.active || self.susp || delta == 0 {
            return;
        }
        self.elapsed += delta;
        // Register fills finish on the tick count; memory copies finish
        // as source rows arrive (no mailbox: everything is flag-only).
        let fill_done =
            matches!(&self.xfer, Some(x) if x.mode == 3) && self.elapsed >= self.ticks_total;
        self.drain_rows(view);
        let rows_done = matches!(&self.xfer, Some(x) if x.mode != 3 && x.rows_done >= x.nl);
        let flag_done = self.mailbox.is_none() && self.elapsed >= self.ticks_total;
        if self.active && (fill_done || rows_done || flag_done) {
            // Register-fill rows are synthesized at completion.
            if fill_done {
                self.fill_rect(view);
            }
            self.complete(view);
        }
    }
    fn pending(&self, _view: &Registers<&[u32]>) -> u32 {
        if self.pending {
            1
        } else {
            0
        }
    }
    fn acknowledge(&mut self, _lines: u32) {
        self.pending = false;
    }
    fn reset(&mut self) {
        self.active = false;
        self.susp = false;
        self.start_armed = false;
        self.elapsed = 0;
        self.ticks_total = 0;
        self.pending = false;
        self.xfer = None;
    }
}

/// Chrom-ART graphics accelerator: register-to-memory fills and
/// memory-to-memory copies with pixel-format conversion over a bus
/// mailbox (flag-only without one); blend/CLUT formats latch `TEIF`.
/// Line offsets count pixels (documented assumption); alpha blending is
/// out of scope (overwrite semantics).
#[derive(Debug)]
pub struct Dma2d {
    inner: RegisterFile<Dma2dBehavior>,
}

impl Dma2d {
    /// A new `Dma2d` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DMA2D").expect("generated tables contain DMA2D");
        Self {
            inner: RegisterFile::new(def, Dma2dBehavior::default()),
        }
    }

    /// Attach the memory-bus mailbox (boards): pixel traffic flows
    /// through it; without one every mode completes flag-only.
    #[must_use]
    pub fn with_mailbox(mut self, mailbox: Arc<Mutex<Dma2dMailbox>>) -> Self {
        self.inner.behavior_mut().mailbox = Some(mailbox);
        self
    }
}

impl Peripheral for Dma2d {
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
        "dma2d"
    }
}

impl InterruptSource for Dma2d {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

#[derive(Debug, Default)]
struct BdmaBehavior {
    active: [bool; 8],
    ticks: [u8; 8],
    pending: u32,
}

impl Behavior for BdmaBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        // ISR 0x00 RO, IFCR 0x04 W1C, CCR0 0x08 + 0x14*ch
        if offset == 0x04 {
            // IFCR write-1-to-clear ISR
            if let Some(mut isr) = view.try_get(0x00) {
                let clear = word;
                isr &= !clear;
                view.try_set(0x00, isr);
            }
            let isr = view.try_get(0x00).unwrap_or(0);
            // clear pending bits where GIF cleared
            for ch in 0..8 {
                if isr & (1 << (ch * 4)) == 0 {
                    self.pending &= !(1 << ch);
                }
            }
            return view.get(offset);
        }
        // check CCR
        if offset >= 0x08 && (offset - 0x08) % 0x14 == 0 {
            let ch = ((offset - 0x08) / 0x14) as usize;
            if ch < 8 {
                let en = word & 1 != 0;
                if en && !self.active[ch] {
                    // Optimistic completion: enable latches the channel for
                    // a 1-tick transfer even with `CNDTR == 0` (no unit
                    // moves, but `GIF`/`TCIF` still latch, as
                    // `bdma_channel0_en_completes` pins).
                    self.active[ch] = true;
                    self.ticks[ch] = 1;
                } else if !en {
                    self.active[ch] = false;
                    self.ticks[ch] = 0;
                    self.pending &= !(1 << ch);
                }
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        for ch in 0..8 {
            if !self.active[ch] || self.ticks[ch] == 0 {
                continue;
            }
            let dec = (delta as u8).min(self.ticks[ch]);
            self.ticks[ch] -= dec;
            if self.ticks[ch] == 0 {
                self.active[ch] = false;
                // set ISR GIF and TCIF for channel
                let mut isr = view.try_get(0x00).unwrap_or(0);
                let gif_bit = ch * 4;
                let tcif_bit = ch * 4 + 1;
                isr |= (1 << gif_bit) | (1 << tcif_bit);
                view.try_set(0x00, isr);
                // clear EN in CCR
                let ccr_off = 0x08 + (ch as u32) * 0x14;
                if let Some(mut ccr) = view.try_get(ccr_off) {
                    ccr &= !1;
                    view.try_set(ccr_off, ccr);
                }
                // check TCIE bit1 in CCR
                let ccr = view.try_get(ccr_off).unwrap_or(0);
                if ccr & (1 << 1) != 0 {
                    self.pending |= 1 << ch;
                }
            }
        }
    }
    fn pending(&self, _view: &Registers<&[u32]>) -> u32 {
        self.pending
    }
    fn acknowledge(&mut self, lines: u32) {
        self.pending &= !lines;
    }
    fn lines(&self) -> u32 {
        8
    }
    fn reset(&mut self) {
        self.active = [false; 8];
        self.ticks = [0; 8];
        self.pending = 0;
    }
}

/// Basic DMA (handshake model).
#[derive(Debug)]
pub struct Bdma {
    inner: RegisterFile<BdmaBehavior>,
}

impl Bdma {
    /// A new `Bdma` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("BDMA").expect("generated tables contain BDMA");
        Self {
            inner: RegisterFile::new(def, BdmaBehavior::default()),
        }
    }
}

impl Peripheral for Bdma {
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
        "bdma"
    }
}

impl InterruptSource for Bdma {
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

#[derive(Debug, Default)]
struct MdmaBehavior {
    active: [bool; 16],
    ticks: [u8; 16],
    pending: u32,
}

impl Behavior for MdmaBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        // GISR0 0x00, ISR per channel at 0x40+0x40*ch, IFCR at +0x04, CR at +0x0C (0x4C)
        // Check IFCR writes to clear ISR
        if (0x44..0x400).contains(&offset) && (offset - 0x44) % 0x40 == 0 {
            // C0IFCR 0x44, C1IFCR 0x84 etc
            let ch = ((offset - 0x44) / 0x40) as usize;
            let isr_off = 0x40 + (ch as u32) * 0x40;
            if let Some(mut isr) = view.try_get(isr_off) {
                let clear = word & 0x1F;
                isr &= !clear;
                view.try_set(isr_off, isr);
            }
            // clear GISR bit
            if let Some(mut gisr) = view.try_get(0x00) {
                let clear = word & 0x1F;
                if clear != 0 {
                    // GISR GIF per channel is bit ch
                    gisr &= !(1 << ch);
                    view.try_set(0x00, gisr);
                }
            }
            if view.try_get(0x40 + (ch as u32) * 0x40).unwrap_or(0) & 0x1F == 0 {
                self.pending &= !(1 << ch);
            }
            return view.get(offset);
        }
        // CCR check: C0CR 0x4C
        if (0x4C..0x400).contains(&offset) && (offset - 0x4C) % 0x40 == 0 {
            let ch = ((offset - 0x4C) / 0x40) as usize;
            if ch < 16 {
                let en = word & 1 != 0;
                if en && !self.active[ch] {
                    self.active[ch] = true;
                    self.ticks[ch] = 1;
                } else if !en {
                    self.active[ch] = false;
                    self.ticks[ch] = 0;
                    self.pending &= !(1 << ch);
                }
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        for ch in 0..16 {
            if !self.active[ch] || self.ticks[ch] == 0 {
                continue;
            }
            let dec = (delta as u8).min(self.ticks[ch]);
            self.ticks[ch] -= dec;
            if self.ticks[ch] == 0 {
                self.active[ch] = false;
                let isr_off = 0x40 + (ch as u32) * 0x40;
                let mut isr = view.try_get(isr_off).unwrap_or(0);
                isr |= 1 << 4; // TCIF
                view.try_set(isr_off, isr);
                // GISR GIF
                let mut gisr = view.try_get(0x00).unwrap_or(0);
                gisr |= 1 << ch;
                view.try_set(0x00, gisr);
                // clear EN
                let cr_off = 0x4C + (ch as u32) * 0x40;
                if let Some(mut cr) = view.try_get(cr_off) {
                    cr &= !1;
                    view.try_set(cr_off, cr);
                }
                // check TCIE? MDMA C0CR TEIE/CTCIE etc — use bit1 as TCIE proxy
                let cr = view.try_get(cr_off).unwrap_or(0);
                if cr & (1 << 1) != 0 || cr & 1 == 0 {
                    // pending if interrupt enabled (simplify: always pending when TC enabled)
                    self.pending |= 1 << ch;
                }
            }
        }
    }
    fn pending(&self, _view: &Registers<&[u32]>) -> u32 {
        // MDMA has single IRQ line, pending if any channel pending
        if self.pending != 0 {
            1
        } else {
            0
        }
    }
    fn acknowledge(&mut self, _lines: u32) {
        self.pending = 0;
    }
    fn reset(&mut self) {
        self.active = [false; 16];
        self.ticks = [0; 16];
        self.pending = 0;
    }
}

/// Master DMA (handshake model).
#[derive(Debug)]
pub struct Mdma {
    inner: RegisterFile<MdmaBehavior>,
}

impl Mdma {
    /// A new `Mdma` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("MDMA").expect("generated tables contain MDMA");
        Self {
            inner: RegisterFile::new(def, MdmaBehavior::default()),
        }
    }
}

impl Peripheral for Mdma {
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
        "mdma"
    }
}

impl InterruptSource for Mdma {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

#[derive(Debug, Default)]
struct SaiBlock {
    /// Transmit/receive FIFO (8 words).
    fifo: std::collections::VecDeque<u32>,
    /// Words staged for reception (tests, debugger inject).
    rx_stage: std::collections::VecDeque<u32>,
    /// Ticks since the last word moved.
    clock: u64,
    /// Consecutive mute-pattern words (RX `MUTEDET`).
    mute_run: u32,
    /// Byte-lane assembler for `DR` writes / word-pop counter for reads.
    w_acc: u32,
    w_mask: u8,
    r_word: u32,
    r_lane: u8,
}

#[derive(Debug, Default)]
struct Sai4Behavior {
    blocks: [SaiBlock; 2],
    pending: bool,
}

/// Block register bases: A at `0x04`, B at `0x24` (`CR1`).
fn sai_base(block: usize) -> u32 {
    0x04 + 0x20 * block as u32
}

impl Sai4Behavior {
    /// `true` for transmitter blocks (`MODE` 0/2: master/slave TX).
    fn is_tx(view: &Registers<&mut [u32]>, b: u32) -> bool {
        matches!(view.try_get(b).unwrap_or(0) & 0x3, 0 | 2)
    }

    fn sync_pending(&mut self, view: &mut Registers<&mut [u32]>) {
        for b in [0x04u32, 0x24] {
            let sr = view.try_get(b + 0x14).unwrap_or(0);
            let im = view.try_get(b + 0x10).unwrap_or(0);
            if sr & im & 0x7F != 0 {
                self.pending = true;
            }
        }
    }

    /// Move one TX word out (or one RX word in) per `(MCKDIV + 1)` ticks.
    fn block_tick(&mut self, view: &mut Registers<&mut [u32]>, b: usize, delta: u64) {
        let base = sai_base(b);
        if view.try_get(base).unwrap_or(0) & (1 << 16) == 0 {
            return; // `SAIXEN` clear: block off.
        }
        let tx = Self::is_tx(view, base);
        let mckdiv = u64::from((view.try_get(base).unwrap_or(0) >> 20) & 0xF);
        let blk = &mut self.blocks[b];
        blk.clock += delta;
        let period = mckdiv + 1;
        while blk.clock >= period {
            blk.clock -= period;
            if tx {
                blk.fifo.pop_front();
            } else if let Some(w) = blk.rx_stage.pop_front() {
                if blk.fifo.len() < 8 {
                    blk.fifo.push_back(w);
                    // Mute detect: `MUTECNT + 1` pattern words in a row.
                    let cr2 = view.try_get(base + 4).unwrap_or(0);
                    let expect_ones = cr2 & (1 << 6) != 0;
                    let pattern = if expect_ones {
                        w == 0xFFFF_FFFF
                    } else {
                        w == 0
                    };
                    if pattern {
                        blk.mute_run += 1;
                        let cnt = ((cr2 >> 7) & 0x3F) + 1;
                        if blk.mute_run >= cnt {
                            Self::set_sr(view, base, 1 << 1);
                        }
                    } else {
                        blk.mute_run = 0;
                    }
                }
            }
        }
        Self::update_sr(view, base, &blk.fifo, tx);
    }

    /// Set status bits (OR semantics).
    fn set_sr(view: &mut Registers<&mut [u32]>, base: u32, bits: u32) {
        let sr = view.try_get(base + 0x14).unwrap_or(0) | bits;
        view.try_set(base + 0x14, sr);
    }

    /// Recompute `FREQ`/`FLVL` from the FIFO level and direction.
    fn update_sr(
        view: &mut Registers<&mut [u32]>,
        base: u32,
        fifo: &std::collections::VecDeque<u32>,
        tx: bool,
    ) {
        let fth = (view.try_get(base + 4).unwrap_or(0) & 0x7) as usize;
        let mut sr = view.try_get(base + 0x14).unwrap_or(0) & !(0x7 << 16) & !(1 << 3);
        sr |= (fifo.len().min(7) as u32) << 16;
        let freq = if tx {
            fifo.len() <= fth
        } else {
            fifo.len() > fth
        };
        if freq {
            sr |= 1 << 3;
        }
        view.try_set(base + 0x14, sr);
    }
}

impl Behavior for Sai4Behavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        // Data registers stage/consume FIFO words; every other register
        // latches (protocol/clock/slot/mask bits have no further model).
        let block = if (0x04..=0x20).contains(&offset) {
            Some(0)
        } else if (0x24..=0x40).contains(&offset) {
            Some(1)
        } else {
            None
        };
        if let Some(b) = block {
            let base = sai_base(b);
            let tx = Self::is_tx(view, base);
            if offset == base + 0x1c {
                // `DR`: TX assembles lanes into FIFO words (full drops
                // with `OVRUDR`); RX writes are ignored (documented).
                if tx {
                    let (word, overrun) = {
                        let blk = &mut self.blocks[b];
                        blk.w_acc |= u32::from(val) << shift;
                        blk.w_mask |= 1 << (shift / 8);
                        if blk.w_mask != 0xF {
                            (None, false)
                        } else {
                            let w = blk.w_acc;
                            blk.w_acc = 0;
                            blk.w_mask = 0;
                            (Some(w), blk.fifo.len() >= 8)
                        }
                    };
                    if let Some(w) = word {
                        if overrun {
                            Self::set_sr(view, base, 1);
                        } else {
                            self.blocks[b].fifo.push_back(w);
                        }
                        Self::update_sr(view, base, &self.blocks[b].fifo, true);
                        self.sync_pending(view);
                    }
                }
            }
            if offset == base + 0x18 {
                // `CLRFR`: write-1-to-clear the sticky flags.
                if let Some(sr) = view.try_get(base + 0x14) {
                    view.try_set(base + 0x14, sr & !word);
                }
                if view.try_get(base + 0x14).unwrap_or(0) & 0x7F == 0 {
                    self.pending = false;
                }
                return view.get(offset);
            }
            if offset == base + 0x04 {
                // `CR2.FFLUSH`: drop the FIFO now (self-clearing pulse).
                if word & (1 << 3) != 0 {
                    self.blocks[b].fifo.clear();
                    self.blocks[b].mute_run = 0;
                    Self::update_sr(view, base, &self.blocks[b].fifo, tx);
                    return word & !(1 << 3);
                }
            }
        }
        // `CR1.SAIXEN` falling flushes the block FIFO and sticky flags.
        if (offset == 0x04 || offset == 0x24) && word & (1 << 16) == 0 {
            let b = if offset == 0x04 { 0 } else { 1 };
            self.blocks[b].fifo.clear();
            self.blocks[b].mute_run = 0;
            if let Some(sr) = view.try_get(sai_base(b) + 0x14) {
                view.try_set(sai_base(b) + 0x14, sr & !(1 | (1 << 1)));
            }
        }
        word
    }
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        // `DR` reads: TX returns the FIFO front without consuming
        // (documented); RX pops one word per word-read (lanes share a
        // counter — firmware always reads `DR` as words). Empty RX pops
        // flag `OVRUDR` and read 0.
        for b in 0..2 {
            let base = sai_base(b);
            if offset == base + 0x1c && !Self::is_tx(view, base) {
                if self.blocks[b].r_lane == 0 {
                    match self.blocks[b].fifo.pop_front() {
                        Some(w) => self.blocks[b].r_word = w,
                        None => {
                            Self::set_sr(view, base, 1);
                            self.sync_pending(view);
                            self.blocks[b].r_word = 0;
                        }
                    }
                    Self::update_sr(view, base, &self.blocks[b].fifo, false);
                    self.sync_pending(view);
                }
                let w = self.blocks[b].r_word;
                self.blocks[b].r_lane += 1;
                if self.blocks[b].r_lane == 4 {
                    self.blocks[b].r_lane = 0;
                }
                return w;
            }
        }
        value
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        for b in 0..2 {
            self.block_tick(view, b, delta);
        }
        self.sync_pending(view);
    }
    fn pending(&self, _view: &Registers<&[u32]>) -> u32 {
        if self.pending {
            1
        } else {
            0
        }
    }
    fn acknowledge(&mut self, _lines: u32) {
        self.pending = false;
    }
    fn reset(&mut self) {
        self.blocks = [SaiBlock::default(), SaiBlock::default()];
        self.pending = false;
    }
}

/// Serial audio interface: per-block TX/RX FIFOs with frame rhythm from
/// `MCKDIV`, `FREQ`/`FLVL` levels, overrun/underrun and RX mute-detect
/// flags, and interrupt gating. `DS`/slots/sync/mono/mute-TX/DMA stay
/// latches (documented); frames clock with no pins attached.
#[derive(Debug)]
pub struct Sai4 {
    inner: RegisterFile<Sai4Behavior>,
}

impl Sai4 {
    /// A new `Sai4` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SAI4").expect("generated tables contain SAI4");
        Self {
            inner: RegisterFile::new(def, Sai4Behavior::default()),
        }
    }
    /// A new peripheral window by SVD name.
    #[must_use]
    pub fn new_for_name<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain SAI");
        Self {
            inner: RegisterFile::new(def, Sai4Behavior::default()),
        }
    }

    /// Stage received words for block `b` (0 = A, 1 = B): they stream
    /// into the RX FIFO on frame ticks (tests, debugger inject).
    pub fn inject_rx(&mut self, b: usize, words: &[u32]) {
        if let Some(blk) = self.inner.behavior_mut().blocks.get_mut(b) {
            blk.rx_stage.extend(words.iter());
        }
    }
}

impl Peripheral for Sai4 {
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
        "sai4"
    }
}

impl InterruptSource for Sai4 {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

/// One DFSDM filter's conversion state.
#[derive(Clone, Copy, Debug, Default)]
struct DfsdmFilter {
    /// Ticks until the armed conversion completes (0 = idle).
    ticks: u64,
    /// Continuous regular conversions re-arm on completion.
    recont: bool,
    /// Injected conversion armed alongside.
    inj: bool,
}

#[derive(Debug, Default)]
struct DfsdmBehavior {
    filters: [DfsdmFilter; 4],
    /// Pending interrupt mask, one bit per filter.
    pending: u32,
    analog_bus: Option<Arc<Mutex<crate::device::analog::AnalogBusState>>>,
}

/// Filter register bases: `CR1` at `0xA0 + 4f` (stride 4 per group).
fn dfsdm_reg(group_base: u32, f: usize) -> u32 {
    group_base + 4 * f as u32
}

impl DfsdmBehavior {
    /// Conversion ticks from the oversampling ratio (one output sample
    /// per `FOSR + 1` ticks, documented tick-time model).
    fn conv_ticks(view: &Registers<&mut [u32]>, f: usize) -> u64 {
        u64::from((view.try_get(dfsdm_reg(0xF0, f)).unwrap_or(0) >> 16) & 0x3FF) + 1
    }

    /// The selected regular channel, if its clock is enabled.
    fn rch(view: &Registers<&mut [u32]>, f: usize) -> Option<u32> {
        let ch = (view.try_get(dfsdm_reg(0xA0, f)).unwrap_or(0) >> 24) & 0x7;
        let cfg = view.try_get(8 * ch).unwrap_or(0);
        if cfg & (1 << 7) != 0 && cfg & (1 << 31) != 0 {
            Some(ch)
        } else {
            None
        }
    }

    /// Sample the shared bus as signed 24-bit data (offset binary:
    /// 0 mV → −FS/2, 1650 mV → 0, 3300 mV → +FS/2; unset bus reads 0).
    fn sample(&self) -> i32 {
        let mv = self
            .analog_bus
            .as_ref()
            .and_then(crate::device::analog::AnalogBusState::millivolts)
            .unwrap_or(0);
        ((mv as i64 - 1650) * 8_388_608 / 1650).clamp(-8_388_608, 8_388_607) as i32
    }

    /// Latch one conversion result, flag overruns, run the watchdog.
    fn complete(&mut self, view: &mut Registers<&mut [u32]>, f: usize, injected: bool) {
        let data = self.sample();
        let (datar, eoc, ovr) = if injected {
            (dfsdm_reg(0x100, f), 1, 1 << 2)
        } else {
            (dfsdm_reg(0x110, f), 1 << 1, 1 << 3)
        };
        let ch = Self::rch(view, f).unwrap_or(0);
        let isr = dfsdm_reg(0xC0, f);
        let mut flags = view.try_get(isr).unwrap_or(0);
        if flags & eoc != 0 {
            flags |= ovr; // unread result overwritten (documented)
        }
        flags |= eoc;
        view.try_set(isr, flags);
        let word = ((data as u32) << 8) | ch;
        view.try_set(datar, word);
        // Analog watchdog on the signed result.
        let awht = view.try_get(dfsdm_reg(0x120, f)).unwrap_or(0) as i32 >> 8;
        let awlt = view.try_get(dfsdm_reg(0x130, f)).unwrap_or(0) as i32 >> 8;
        if data < awlt || data > awht {
            let isr2 = view.try_get(isr).unwrap_or(0) | (1 << 4);
            view.try_set(isr, isr2);
            let awsr = dfsdm_reg(0x140, f);
            view.try_set(awsr, view.try_get(awsr).unwrap_or(0) | (1 << ch));
        }
        self.sync_filter(view, f);
    }

    /// Recompute filter `f`'s pending bit from enabled flags.
    fn sync_filter(&mut self, view: &mut Registers<&mut [u32]>, f: usize) {
        let isr = view.try_get(dfsdm_reg(0xC0, f)).unwrap_or(0);
        let cr2 = view.try_get(dfsdm_reg(0xB0, f)).unwrap_or(0);
        let enabled = ((isr & 1) != 0 && cr2 & 1 != 0) // JEOCF & JEOCIE
            || ((isr & (1 << 1)) != 0 && cr2 & (1 << 1) != 0) // REOCF & REOCIE
            || ((isr & (1 << 4)) != 0 && cr2 & (1 << 4) != 0) // AWDF & AWDIE
            || ((isr & (1 << 2)) != 0 && cr2 & (1 << 2) != 0) // JOVRF & JOVRIE
            || ((isr & (1 << 3)) != 0 && cr2 & (1 << 3) != 0); // ROVRF & ROVRIE
        if enabled {
            self.pending |= 1 << f;
        } else {
            self.pending &= !(1 << f);
        }
    }
}

impl Behavior for DfsdmBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        // Filter control registers land every 4 bytes from their group.
        for f in 0..4usize {
            if offset == dfsdm_reg(0xA0, f) {
                // `CR1`: starts arm conversions, `RCONT` latches. Start
                // bits are pulses (cleared on store, like silicon).
                let mut stored = word;
                if word & (1 << 17) != 0 {
                    self.filters[f].ticks = Self::conv_ticks(view, f);
                    self.filters[f].recont = word & (1 << 18) != 0;
                    self.filters[f].inj = false;
                    stored &= !(1 << 17);
                }
                if word & (1 << 1) != 0 {
                    self.filters[f].ticks = Self::conv_ticks(view, f);
                    self.filters[f].inj = true;
                    stored &= !(1 << 1);
                }
                if word & 1 == 0 {
                    self.filters[f].ticks = 0;
                    self.filters[f].inj = false;
                }
                return stored;
            }
            if offset == dfsdm_reg(0xD0, f) {
                // `ICR`: clear overrun flags.
                if let Some(isr) = view.try_get(dfsdm_reg(0xC0, f)) {
                    let mut clear = 0;
                    if word & (1 << 2) != 0 {
                        clear |= 1 << 2;
                    }
                    if word & (1 << 3) != 0 {
                        clear |= 1 << 3;
                    }
                    view.try_set(dfsdm_reg(0xC0, f), isr & !clear);
                }
                self.sync_filter(view, f);
                return view.get(offset);
            }
            if offset == dfsdm_reg(0x150, f) {
                // `AWCFR`: any write clears the watchdog flag.
                if let Some(isr) = view.try_get(dfsdm_reg(0xC0, f)) {
                    view.try_set(dfsdm_reg(0xC0, f), isr & !(1 << 4));
                    view.try_set(dfsdm_reg(0x140, f), 0);
                }
                self.sync_filter(view, f);
            }
        }
        word
    }
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        // Reading a data register consumes its end-of-conversion flag
        // (documented contract; overruns then mean exactly "unread").
        for f in 0..4usize {
            if offset == dfsdm_reg(0x110, f) {
                if let Some(isr) = view.try_get(dfsdm_reg(0xC0, f)) {
                    view.try_set(dfsdm_reg(0xC0, f), isr & !(1 << 1));
                }
                self.sync_filter(view, f);
            }
            if offset == dfsdm_reg(0x100, f) {
                if let Some(isr) = view.try_get(dfsdm_reg(0xC0, f)) {
                    view.try_set(dfsdm_reg(0xC0, f), isr & !1);
                }
                self.sync_filter(view, f);
            }
        }
        value
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        for f in 0..4usize {
            if self.filters[f].ticks == 0 {
                continue;
            }
            let left = self.filters[f].ticks.saturating_sub(delta);
            self.filters[f].ticks = left;
            if left == 0 {
                let inj = self.filters[f].inj;
                self.filters[f].inj = false;
                self.complete(view, f, inj);
                if self.filters[f].recont {
                    self.filters[f].ticks = Self::conv_ticks(view, f);
                }
            }
        }
    }
    fn pending(&self, _view: &Registers<&[u32]>) -> u32 {
        self.pending
    }
    fn acknowledge(&mut self, lines: u32) {
        self.pending &= !lines;
    }
    fn reset(&mut self) {
        self.filters = [DfsdmFilter::default(); 4];
        self.pending = 0;
    }
}

/// Digital filter for sigma-delta modulators: channel enables, regular
/// and injected conversions of the shared analog bus (offset binary),
/// analog watchdog, and per-filter interrupt lines. Filter order and
/// integrator settings pace conversions only; DMA, serial breaks, and
/// extremes stay latches (documented).
#[derive(Debug)]
pub struct Dfsdm {
    inner: RegisterFile<DfsdmBehavior>,
}

impl Dfsdm {
    /// A new `Dfsdm` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DFSDM").expect("generated tables contain DFSDM");
        Self {
            inner: RegisterFile::new(def, DfsdmBehavior::default()),
        }
    }

    /// Attach the shared analog bus (boards): conversions sample it.
    #[must_use]
    pub fn with_bus(mut self, bus: Arc<Mutex<crate::device::analog::AnalogBusState>>) -> Self {
        self.inner.behavior_mut().analog_bus = Some(bus);
        self
    }
}

impl Peripheral for Dfsdm {
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
        "dfsdm"
    }
}

impl InterruptSource for Dfsdm {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }
    fn pending(&self) -> u32 {
        self.inner.pending()
    }
    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
    /// One line per filter (`DFSDM_FLT0-3`, consecutive vectors).
    fn lines(&self) -> u32 {
        0b1111
    }
}

#[derive(Debug, Default)]
struct HrtimBehavior {
    pending: bool,
    ticks: u8,
    active: bool,
}

impl Behavior for HrtimBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let word = view.merged(offset, val, shift);
        // Master: MCR 0x00 MCEN bit0; TIMx: TIMxCR 0x00 CEN bit0; Common: CCR at 0x00?
        // Simplify: offset 0x00 bit0 triggers
        if offset == 0x00 {
            if word & 1 != 0 && !self.active {
                self.active = true;
                self.ticks = 1;
            } else if word & 1 == 0 {
                self.active = false;
                self.ticks = 0;
                self.pending = false;
            }
        }
        // ICR-like at 0x08 clear ISR
        if offset == 0x08 || offset == 0x0C {
            // write-1-to-clear ISR at 0x08
            if view.try_get(0x08).is_some() {
                let mut v = view.try_get(0x08).unwrap_or(0);
                v &= !word;
                view.try_set(0x08, v);
                if v == 0 {
                    self.pending = false;
                }
            }
        }
        word
    }
    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        if !self.active || self.ticks == 0 {
            return;
        }
        let dec = (delta as u8).min(self.ticks);
        self.ticks -= dec;
        if self.ticks == 0 {
            self.active = false;
            // set ISR bit0 at 0x08
            if view.try_get(0x08).is_some() {
                let mut v = view.try_get(0x08).unwrap_or(0);
                v |= 1;
                view.try_set(0x08, v);
            }
            self.pending = true;
        }
    }
    fn pending(&self, _view: &Registers<&[u32]>) -> u32 {
        if self.pending {
            1
        } else {
            0
        }
    }
    fn acknowledge(&mut self, _lines: u32) {
        self.pending = false;
    }
    fn reset(&mut self) {
        self.pending = false;
        self.active = false;
        self.ticks = 0;
    }
}

/// High-resolution timer (handshake model).
#[derive(Debug)]
pub struct Hrtim {
    inner: RegisterFile<HrtimBehavior>,
}

impl Hrtim {
    /// The HRTIM master timer.
    #[must_use]
    pub fn master_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_Master").expect("generated tables contain HRTIM_Master");
        Self {
            inner: RegisterFile::new(def, HrtimBehavior::default()),
        }
    }
    /// HRTIM timer A.
    #[must_use]
    pub fn tima_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMA").expect("generated tables contain HRTIM_TIMA");
        Self {
            inner: RegisterFile::new(def, HrtimBehavior::default()),
        }
    }
    /// HRTIM timer B.
    #[must_use]
    pub fn timb_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMB").expect("generated tables contain HRTIM_TIMB");
        Self {
            inner: RegisterFile::new(def, HrtimBehavior::default()),
        }
    }
    /// HRTIM timer C.
    #[must_use]
    pub fn timc_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMC").expect("generated tables contain HRTIM_TIMC");
        Self {
            inner: RegisterFile::new(def, HrtimBehavior::default()),
        }
    }
    /// HRTIM timer D.
    #[must_use]
    pub fn timd_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMD").expect("generated tables contain HRTIM_TIMD");
        Self {
            inner: RegisterFile::new(def, HrtimBehavior::default()),
        }
    }
    /// HRTIM timer E.
    #[must_use]
    pub fn time_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIME").expect("generated tables contain HRTIM_TIME");
        Self {
            inner: RegisterFile::new(def, HrtimBehavior::default()),
        }
    }
    /// HRTIM timer F.
    #[must_use]
    pub fn timf_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMF").expect("generated tables contain HRTIM_TIMF");
        Self {
            inner: RegisterFile::new(def, HrtimBehavior::default()),
        }
    }
    /// HRTIM common registers.
    #[must_use]
    pub fn common_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_Common").expect("generated tables contain HRTIM_Common");
        Self {
            inner: RegisterFile::new(def, HrtimBehavior::default()),
        }
    }
}

impl Peripheral for Hrtim {
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
        "hrtim"
    }
}

impl InterruptSource for Hrtim {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
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
    use super::{
        Bdma, Dfsdm, Dma2d, Dma2dMailbox, Dma2dRead, Dma2dResult, Dmamux, DmamuxRoutes, Hrtim,
        Ltdc, Mdma, Sai4,
    };
    use crate::device::svd::stm32h743::Stm32h743;
    use crate::device::{InterruptSource, Peripheral};
    use std::sync::{Arc, Mutex};

    fn write_word<P: Peripheral>(p: &mut P, addr: u32, val: u32) {
        for i in 0..4 {
            p.write(addr + i, ((val >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn read_word<P: Peripheral>(p: &mut P, addr: u32) -> u32 {
        let mut w = 0;
        for i in 0..4 {
            w |= u32::from(p.read(addr + i).unwrap()) << (8 * i);
        }
        w
    }

    #[test]
    fn dmamux_routes_dma_req_id() {
        use super::DmamuxRoutes;
        use std::sync::{Arc, Mutex};
        let routes = Arc::new(Mutex::new(DmamuxRoutes::new()));
        let mut d = Dmamux::new_for::<Stm32h743>().with_routes(routes.clone());
        let base = d.ranges()[0].base;
        // CCR0 DMAREQ_ID = 0x12
        write_word(&mut d, base, 0x12);
        assert_eq!(read_word(&mut d, base) & 0x7F, 0x12);
        // CCR1
        write_word(&mut d, base + 4, 0x34);
        assert_eq!(read_word(&mut d, base + 4) & 0x7F, 0x34);
        // Check the shared routing table.
        let routes = routes.lock().unwrap();
        assert_eq!(routes.ccr_id[0], 0x12);
        assert_eq!(routes.ccr_id[1], 0x34);
    }

    #[test]
    fn dmamux_cfr_clears_csr_and_overrun_lines() {
        use super::DmamuxRoutes;
        use std::sync::{Arc, Mutex};
        let routes = Arc::new(Mutex::new(DmamuxRoutes::new()));
        let mut d = Dmamux::new_for::<Stm32h743>().with_routes(routes.clone());
        let base = d.ranges()[0].base;
        // CCR2 SOIE on (bit 8); inject a sync-overrun on channel 2.
        write_word(&mut d, base + 8, 0x100);
        assert_eq!(d.pending(), 0, "no flag, no line");
        d.inject_overrun(2);
        // The flag materializes on read; the line follows SOIE.
        assert_eq!(read_word(&mut d, base + 0x80) & (1 << 2), 1 << 2, "SOF2");
        assert_eq!(d.pending(), 1, "SOIE raises the shared line");
        // CFR write-1-to-clear drops the flag and the line.
        write_word(&mut d, base + 0x84, 1 << 2);
        assert_eq!(read_word(&mut d, base + 0x80) & (1 << 2), 0, "SOF2 cleared");
        assert_eq!(d.pending(), 0);
    }

    #[test]
    fn dmamux_without_routes_stays_a_latch() {
        // DMAMUX2 / unrouted instances keep pure latch behavior.
        let mut d = Dmamux::dmamux2_for::<Stm32h743>();
        let base = d.ranges()[0].base;
        write_word(&mut d, base, 0x12);
        assert_eq!(read_word(&mut d, base) & 0x7F, 0x12);
        assert_eq!(d.pending(), 0);
    }

    /// WP-L2 acceptance: `USART2` RX bytes reach RAM through a
    /// DMAMUX-routed `DMA1` stream. The USART asserts its board-convention
    /// input (RX = 2); with `CCR2.DMAREQ_ID` set to a wrong input the
    /// stream stalls (gating proved — legacy routing would have moved it),
    /// and with the matching ID both bytes land and `TCIF` latches.
    #[test]
    fn usart_rx_reaches_ram_through_dmamux() {
        use crate::device::stm32::dma::{DmaRequests, DmaUnit};
        use crate::device::stm32::dma_f4::DmaF4;
        use crate::device::stm32::usart::Usart;
        use std::sync::{Arc, Mutex};

        let reqs = Arc::new(Mutex::new(DmaRequests::new()));
        let routes = Arc::new(Mutex::new(DmamuxRoutes::new()));
        let mailbox: Arc<Mutex<Vec<DmaUnit>>> = Arc::new(Mutex::new(Vec::new()));
        let mut u = Usart::usart2_for::<Stm32h743>().with_dma_routed(reqs.clone(), 0, 3, 2);
        let mut mux = Dmamux::new_for::<Stm32h743>().with_routes(routes.clone());
        let mut d = DmaF4::dma1_for::<Stm32h743>(mailbox.clone())
            .with_requests(reqs.clone(), 0)
            .with_dmamux(routes.clone(), 0);
        let mbase = mux.ranges()[0].base;
        let dbase = d.ranges()[0].base;
        let ubase = u.ranges()[0].base;
        // DMA1 stream-2 block: CR 0x40, NDTR 0x44, PAR 0x48, M0AR 0x4C.
        let (cr2, ndtr2, par2, m0ar2) = (
            dbase + 0x10 + 2 * 0x18,
            dbase + 0x14 + 2 * 0x18,
            dbase + 0x18 + 2 * 0x18,
            dbase + 0x1C + 2 * 0x18,
        );
        // Stream 2: USART DR → RAM, 2 bytes, MINC.
        // (H7 USART RDR sits at +0x24; the address is recorded, not read.)
        write_word(&mut d, par2, ubase + 0x24); // PAR2
        write_word(&mut d, m0ar2, 0x2000_1000); // M0AR2
        write_word(&mut d, ndtr2, 2); // NDTR2
        write_word(&mut d, cr2, 0x1 | (1 << 10)); // EN|MINC
                                                  // Wrong input selected: the stream stalls despite RX traffic.
        write_word(&mut mux, mbase + 0x08, 7); // CCR2.DMAREQ_ID = 7
        u.drive_inputs(u32::from(b'A'));
        u.tick(1);
        d.tick(1);
        assert_eq!(read_word(&mut d, ndtr2), 2, "gated stream stalls");
        assert!(mailbox.lock().unwrap().is_empty());
        // Matching input: both bytes flow into RAM.
        write_word(&mut mux, mbase + 0x08, 2); // CCR2.DMAREQ_ID = 2
        for byte in b"AB" {
            u.drive_inputs(u32::from(*byte));
            u.tick(1);
            d.tick(1);
        }
        let units = mailbox.lock().unwrap().clone();
        assert_eq!(units.len(), 2, "two RX bytes moved");
        assert_eq!(units[0].1, 0x2000_1000);
        assert_eq!(units[1].1, 0x2000_1001);
        assert_eq!(read_word(&mut d, ndtr2), 0);
        // Stream 2 TCIF lives at LISR base 16 + 5.
        assert_eq!(read_word(&mut d, dbase) & (1 << 21), 1 << 21, "TCIF2");
    }

    #[test]
    fn ltdc_enables_and_raises_line_interrupt() {
        let mut l = Ltdc::new_for::<Stm32h743>();
        let base = l.ranges()[0].base;
        // GCR LTDCEN
        write_word(&mut l, base + 0x18, 1);
        // IER LIE
        write_word(&mut l, base + 0x34, 1);
        assert_eq!(l.pending(), 0);
        l.tick(100);
        assert_eq!(l.pending(), 1);
        assert_eq!(read_word(&mut l, base + 0x38) & 1, 1);
        // ICR clears
        write_word(&mut l, base + 0x3c, 1);
        assert_eq!(read_word(&mut l, base + 0x38) & 1, 0);
    }

    #[test]
    fn ltdc_layer_reload_paths() {
        let mut l = Ltdc::new_for::<Stm32h743>();
        let base = l.ranges()[0].base;
        write_word(&mut l, base + 0x18, 1); // GCR LTDCEN
        write_word(&mut l, base + 0x34, 1 << 3); // IER RRIE
                                                 // Layer 1 enable arms immediately with IMR, latching RRIF.
        write_word(&mut l, base + 0x84, 1); // L1CR LEN
        assert_eq!(
            read_word(&mut l, base + 0x38) & (1 << 3),
            0,
            "no reload yet"
        );
        write_word(&mut l, base + 0x24, 1); // SRCR IMR
        assert_ne!(read_word(&mut l, base + 0x38) & (1 << 3), 0, "RRIF");
        assert_ne!(l.pending(), 0, "reload line");
        write_word(&mut l, base + 0x3c, 1 << 3); // ICR
        assert_eq!(l.pending(), 0);
        // Layer 2 enable waits for the blanking reload.
        write_word(&mut l, base + 0x104, 1); // L2CR LEN
        write_word(&mut l, base + 0x24, 1 << 1); // SRCR VBR
        assert_eq!(read_word(&mut l, base + 0x38) & (1 << 3), 0, "deferred");
        l.tick(1); // one-line frame wraps, reload applies
        assert_ne!(read_word(&mut l, base + 0x38) & (1 << 3), 0, "RRIF at wrap");
    }

    #[test]
    fn ltdc_line_compare_tracks_scan() {
        let mut l = Ltdc::new_for::<Stm32h743>();
        let base = l.ranges()[0].base;
        write_word(&mut l, base + 0x18, 1); // GCR LTDCEN
        write_word(&mut l, base + 0x14, 99); // TWCR TOTALH 99 → 100 lines
        write_word(&mut l, base + 0x40, 50); // LIPCR line 50
        write_word(&mut l, base + 0x34, 1); // IER LIE
        l.tick(49);
        assert_eq!(read_word(&mut l, base + 0x38) & 1, 0, "not yet");
        assert_eq!(read_word(&mut l, base + 0x44) & 0x7FF, 49, "CPSR.Y live");
        l.tick(1);
        assert_ne!(read_word(&mut l, base + 0x38) & 1, 0, "LIF at line 50");
        assert_ne!(l.pending(), 0);
        // Past the line, no refire until the next frame.
        write_word(&mut l, base + 0x3c, 1);
        l.tick(10);
        assert_eq!(read_word(&mut l, base + 0x38) & 1, 0, "passed");
        l.tick(90); // wraps through line 50 again
        assert_ne!(read_word(&mut l, base + 0x38) & 1, 0, "LIF next frame");
    }

    #[test]
    fn dma2d_start_completes_and_pends() {
        let mut d = Dma2d::new_for::<Stm32h743>();
        let base = d.ranges()[0].base;
        // TCIE
        write_word(&mut d, base, 1 << 9);
        // START
        write_word(&mut d, base, (1 << 9) | 1);
        assert_eq!(d.pending(), 0);
        d.tick(1);
        assert_eq!(read_word(&mut d, base + 0x04) & (1 << 1), 1 << 1);
        assert_eq!(d.pending(), 1);
    }

    #[test]
    fn bdma_channel0_en_completes() {
        let mut b = Bdma::new_for::<Stm32h743>();
        let base = b.ranges()[0].base;
        // enable TCIE then EN
        write_word(&mut b, base + 0x08, (1 << 1) | 1);
        b.tick(1);
        let isr = read_word(&mut b, base);
        assert_eq!(isr & 0x3, 0x3); // GIF0 + TCIF0
        assert_eq!(b.pending(), 1);
    }

    #[test]
    fn mdma_channel0_en_completes() {
        let mut m = Mdma::new_for::<Stm32h743>();
        let base = m.ranges()[0].base;
        write_word(&mut m, base + 0x4c, 1);
        m.tick(1);
        let isr = read_word(&mut m, base + 0x40);
        assert_eq!(isr & (1 << 4), 1 << 4);
        assert_eq!(m.pending(), 1);
    }

    #[test]
    fn hrtim_master_en_pends() {
        let mut h = Hrtim::master_for::<Stm32h743>();
        let base = h.ranges()[0].base;
        write_word(&mut h, base, 1);
        h.tick(1);
        assert_eq!(h.pending(), 1);
    }

    #[test]
    fn sai4_tx_empty_requests_data() {
        let mut s = Sai4::new_for::<Stm32h743>();
        let base = s.ranges()[0].base;
        // Block A master TX, FIFO threshold 0, FREQ interrupt on.
        write_word(&mut s, base + 0x04, 1 << 16); // CR1 SAIXEN
        write_word(&mut s, base + 0x14, 1 << 3); // IM FREQIE
        assert_eq!(s.pending(), 0);
        s.tick(1);
        assert_ne!(read_word(&mut s, base + 0x18) & (1 << 3), 0, "FREQ");
        assert_eq!(read_word(&mut s, base + 0x18) >> 16 & 0x7, 0, "FLVL 0");
        assert_ne!(s.pending(), 0);
    }

    #[test]
    fn sai4_tx_streams_and_flags_overrun() {
        let mut s = Sai4::new_for::<Stm32h743>();
        let base = s.ranges()[0].base;
        write_word(&mut s, base + 0x04, 1 << 16); // TX enable
        write_word(&mut s, base + 0x14, 1); // IM OVRUDRIE
        for w in 0..8u32 {
            write_word(&mut s, base + 0x20, 0x1000 + w);
        }
        assert_eq!(read_word(&mut s, base + 0x18) >> 16 & 0x7, 7, "FLVL capped");
        write_word(&mut s, base + 0x20, 0xDEAD_BEEF); // ninth word
        assert_ne!(read_word(&mut s, base + 0x18) & 1, 0, "OVRUDR");
        assert_ne!(s.pending(), 0);
        write_word(&mut s, base + 0x1c, 1); // CLRFR COVRUDR
        assert_eq!(s.pending(), 0);
        s.tick(8); // one word per tick at MCKDIV 0
        assert_eq!(read_word(&mut s, base + 0x18) >> 16 & 0x7, 0, "drained");
    }

    #[test]
    fn sai4_rx_mute_detect_and_dr() {
        let mut s = Sai4::new_for::<Stm32h743>();
        let base = s.ranges()[0].base;
        // Block B master RX, mute after 2 zero words, interrupts on.
        write_word(&mut s, base + 0x24, (1 << 16) | 1); // CR1 SAIXEN RX
        write_word(&mut s, base + 0x28, (1 << 7) | 1); // CR2 MUTECNT1 FTH1
        write_word(&mut s, base + 0x34, (1 << 1) | (1 << 3)); // IM MUTEDETIE FREQIE
        s.inject_rx(1, &[0, 0, 0x1234_5678]);
        s.tick(3);
        assert_ne!(read_word(&mut s, base + 0x38) & (1 << 1), 0, "MUTEDET");
        assert_ne!(s.pending(), 0);
        assert_eq!(read_word(&mut s, base + 0x40), 0, "first zero");
        assert_eq!(read_word(&mut s, base + 0x40), 0, "second zero");
        assert_eq!(read_word(&mut s, base + 0x40), 0x1234_5678, "data");
        // Draining below threshold re-arms FREQ; flags clear by CLRFR.
        write_word(&mut s, base + 0x3c, (1 << 1) | 1);
        assert_eq!(read_word(&mut s, base + 0x38) & 0x3, 0);
    }

    /// DMA2D with a scripted bus: reads come from `mem`, writes land there.
    fn dma2d() -> (Dma2d, Arc<Mutex<Dma2dMailbox>>, u32) {
        let mb = Arc::new(Mutex::new(Dma2dMailbox::default()));
        let mut d = Dma2d::new_for::<Stm32h743>().with_mailbox(mb.clone());
        let base = d.ranges()[0].base;
        d.reset();
        (d, mb, base)
    }

    /// Serve one bus round against `mem` at `ram_base`.
    fn dma2d_drain(mb: &Arc<Mutex<Dma2dMailbox>>, mem: &mut [u8], ram_base: u32) {
        let mut mb = mb.lock().unwrap();
        let reads: Vec<Dma2dRead> = core::mem::take(&mut mb.reads);
        for r in reads {
            let off = r.addr.wrapping_sub(ram_base) as usize;
            mb.results.push_back(Dma2dResult {
                row: r.row,
                bytes: mem[off..off + r.len as usize].to_vec(),
            });
        }
        let writes: Vec<(u32, Vec<u8>)> = core::mem::take(&mut mb.writes);
        for (addr, bytes) in writes {
            let off = addr.wrapping_sub(ram_base) as usize;
            mem[off..off + bytes.len()].copy_from_slice(&bytes);
        }
    }

    /// Tick + drain rounds until `TCIF`, returning ticks used.
    fn dma2d_run(d: &mut Dma2d, mb: &Arc<Mutex<Dma2dMailbox>>, mem: &mut [u8], ram: u32) -> u64 {
        for t in 1..=40 {
            d.tick(1);
            dma2d_drain(mb, mem, ram);
            if read_word(d, d.ranges()[0].base + 0x04) & (1 << 1) != 0 {
                return t;
            }
        }
        panic!("TCIF never latched");
    }

    const RAM: u32 = 0x2000_0000;

    #[test]
    fn dma2d_r2m_fills_red_argb8888() {
        let (mut d, mb, base) = dma2d();
        let mut mem = vec![0u8; 0x1000];
        write_word(&mut d, base + 0x34, 0); // OPFCCR ARGB8888
        write_word(&mut d, base + 0x38, 0xFFFF_0000); // OCOLR opaque red
        write_word(&mut d, base + 0x3c, RAM); // OMAR
        write_word(&mut d, base + 0x44, 2 | (8 << 16)); // NLR 2 lines × 8 px
        write_word(&mut d, base, (1 << 9) | (3 << 16) | 1); // TCIE R2M START
        dma2d_run(&mut d, &mb, &mut mem, RAM);
        assert_ne!(d.pending(), 0);
        // B G R A per pixel, little-endian.
        assert_eq!(&mem[..8], &[0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF]);
        assert_eq!(&mem[32..40], &[0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF]);
        assert_eq!(read_word(&mut d, base) & 1, 0, "START clears");
    }

    #[test]
    fn dma2d_m2m_copies_bytes_exactly() {
        let (mut d, mb, base) = dma2d();
        let mut mem = vec![0u8; 0x1000];
        for (i, b) in mem[..64].iter_mut().enumerate() {
            *b = (i * 5 + 1) as u8;
        }
        let want = mem[..64].to_vec();
        write_word(&mut d, base + 0x1c, 0); // FGPFCCR ARGB8888
        write_word(&mut d, base + 0x34, 0); // OPFCCR ARGB8888
        write_word(&mut d, base + 0x0c, RAM); // FGMAR
        write_word(&mut d, base + 0x3c, RAM + 0x400); // OMAR
        write_word(&mut d, base + 0x44, 4 | (4 << 16)); // 4 lines × 4 px
        write_word(&mut d, base, (1 << 9) | 1); // TCIE M2M START
        dma2d_run(&mut d, &mb, &mut mem, RAM);
        assert_eq!(mem[0x400..0x440], want[..], "byte-exact copy");
    }

    #[test]
    fn dma2d_pfc_rgb565_red_to_argb() {
        let (mut d, mb, base) = dma2d();
        let mut mem = vec![0u8; 0x1000];
        mem[0] = 0x00;
        mem[1] = 0xF8; // RGB565 red
        write_word(&mut d, base + 0x1c, 2); // FGPFCCR RGB565
        write_word(&mut d, base + 0x34, 0); // OPFCCR ARGB8888
        write_word(&mut d, base + 0x0c, RAM);
        write_word(&mut d, base + 0x3c, RAM + 0x400);
        write_word(&mut d, base + 0x44, 1 | (1 << 16));
        write_word(&mut d, base, (1 << 9) | (1 << 16) | 1); // TCIE PFC START
        dma2d_run(&mut d, &mb, &mut mem, RAM);
        assert_eq!(&mem[0x400..0x404], &[0, 0, 0xFF, 0xFF], "expanded red");
    }

    #[test]
    fn dma2d_blend_and_bad_format_raise_teif() {
        let (mut d, mb, base) = dma2d();
        let mem = vec![0u8; 0x1000];
        // Config errors surface on the first tick after START, once the
        // full control word (notably `MODE`) has converged.
        write_word(&mut d, base, (1 << 8) | (2 << 16) | 1); // TEIE BLEND START
        d.tick(1);
        assert_ne!(read_word(&mut d, base + 0x04) & 1, 0, "TEIF on blend");
        assert_eq!(read_word(&mut d, base + 0x04) & (1 << 1), 0, "no TCIF");
        assert_eq!(read_word(&mut d, base) & 1, 0, "START clears");
        assert_ne!(d.pending(), 0, "error line");
        write_word(&mut d, base + 0x08, 1); // IFCR CTEIF
        assert_eq!(d.pending(), 0);
        // Unknown pixel format: same treatment.
        write_word(&mut d, base + 0x1c, 6); // FGPFCCR A4-ish
        write_word(&mut d, base, (1 << 8) | (1 << 16) | 1); // TEIE PFC START
        d.tick(1);
        assert_ne!(read_word(&mut d, base + 0x04) & 1, 0, "TEIF on format");
        let _ = (mb, mem);
    }

    #[test]
    fn dma2d_abort_raises_caeif() {
        let (mut d, mb, base) = dma2d();
        let mem = vec![0u8; 0x1000];
        write_word(&mut d, base + 0x1c, 0);
        write_word(&mut d, base + 0x34, 0);
        write_word(&mut d, base + 0x0c, RAM);
        write_word(&mut d, base + 0x3c, RAM + 0x400);
        write_word(&mut d, base + 0x44, 8 | (8 << 16));
        write_word(&mut d, base, (1 << 11) | 1); // CAEIE START
                                                 // No bus service: rows stall, then abort wins.
        d.tick(1);
        write_word(&mut d, base, (1 << 11) | (1 << 2)); // CAEIE ABORT
        assert_ne!(read_word(&mut d, base + 0x04) & (1 << 3), 0, "CAEIF");
        assert_eq!(read_word(&mut d, base) & 1, 0, "START clears");
        assert_ne!(d.pending(), 0);
        assert_eq!(read_word(&mut d, base + 0x04) & (1 << 1), 0, "no TCIF");
        let _ = (mb, mem);
    }

    #[test]
    fn dma2d_watermark_fires_at_lwr_line() {
        let (mut d, mb, base) = dma2d();
        let mut mem = vec![0u8; 0x1000];
        write_word(&mut d, base + 0x1c, 0);
        write_word(&mut d, base + 0x34, 0);
        write_word(&mut d, base + 0x0c, RAM);
        write_word(&mut d, base + 0x3c, RAM + 0x400);
        write_word(&mut d, base + 0x44, 4 | (2 << 16)); // 4 lines × 2 px
        write_word(&mut d, base + 0x48, 2); // LWR line 2
        write_word(&mut d, base, (1 << 10) | 1); // TWIE START
        for _ in 0..20 {
            d.tick(1);
            dma2d_drain(&mb, &mut mem, RAM);
            if read_word(&mut d, base + 0x04) & (1 << 2) != 0 {
                break;
            }
        }
        assert_ne!(read_word(&mut d, base + 0x04) & (1 << 2), 0, "TWIF");
        assert_ne!(d.pending(), 0);
        dma2d_run(&mut d, &mb, &mut mem, RAM);
    }

    /// DFSDM on a shared bus (millivolts set per test).
    fn dfsdm(bus_mv: u32) -> (Dfsdm, u32) {
        use crate::device::analog::AnalogBusState;
        let bus = AnalogBusState::shared();
        AnalogBusState::set_millivolts(&bus, bus_mv);
        let mut d = Dfsdm::new_for::<Stm32h743>().with_bus(bus);
        let base = d.ranges()[0].base;
        d.reset();
        // Channel 0 clock on; filter 0 regular on channel 0, IRQ on.
        write_word(&mut d, base, (1 << 7) | (1 << 31)); // CHCFG0R1 CHEN DFSDMEN
        write_word(&mut d, base + 0xb0, (1 << 1) | (1 << 4)); // CR2 REOCIE AWDIE
        (d, base)
    }

    #[test]
    fn dfsdm_regular_converts_bus_midscale() {
        let (mut d, base) = dfsdm(1650);
        write_word(&mut d, base + 0xa0, 1 | (1 << 17)); // CR1 DFEN RSWSTART
        d.tick(1); // FOSR reset 0 → one tick
        assert_ne!(read_word(&mut d, base + 0xc0) & (1 << 1), 0, "REOCF");
        assert_ne!(d.pending() & 1, 0, "filter 0 line");
        assert_eq!(read_word(&mut d, base + 0x110), 0, "midscale DATA");
    }

    #[test]
    fn dfsdm_data_tracks_rails() {
        let (mut d, base) = dfsdm(3300);
        write_word(&mut d, base + 0xa0, 1 | (1 << 17));
        d.tick(1);
        assert_eq!(read_word(&mut d, base + 0x110), 0x7FFF_FF00, "+FS");
        let (mut d, base) = dfsdm(0);
        write_word(&mut d, base + 0xa0, 1 | (1 << 17));
        d.tick(1);
        assert_eq!(read_word(&mut d, base + 0x110), 0x8000_0000, "-FS");
    }

    #[test]
    fn dfsdm_watchdog_trips_and_clears() {
        let (mut d, base) = dfsdm(1650); // DATA 0
        write_word(&mut d, base + 0x130, 0x0010_0000); // AWLTR +1M (DATA below)
        write_word(&mut d, base + 0xa0, 1 | (1 << 17));
        d.tick(1);
        assert_ne!(read_word(&mut d, base + 0xc0) & (1 << 4), 0, "AWDF");
        assert_ne!(read_word(&mut d, base + 0x140) & 1, 0, "channel 0");
        assert_ne!(d.pending() & 1, 0);
        assert_eq!(read_word(&mut d, base + 0x110), 0, "consume REOCF");
        write_word(&mut d, base + 0x150, 0xFFFF_FFFF); // AWCFR clears
        assert_eq!(read_word(&mut d, base + 0xc0) & (1 << 4), 0, "AWDF clears");
        assert_eq!(d.pending() & 1, 0);
    }

    #[test]
    fn dfsdm_injected_overruns_when_unread() {
        let (mut d, base) = dfsdm(1650);
        write_word(&mut d, base + 0xb0, 1 | (1 << 2)); // CR2 JEOCIE JOVRIE
        write_word(&mut d, base + 0xa0, 1 | (1 << 1)); // DFEN JSWSTART
        d.tick(1);
        assert_ne!(read_word(&mut d, base + 0xc0) & 1, 0, "JEOCF");
        write_word(&mut d, base + 0xa0, 1 | (1 << 1)); // convert again
        d.tick(1);
        assert_ne!(read_word(&mut d, base + 0xc0) & (1 << 2), 0, "JOVRF");
        assert_ne!(d.pending() & 1, 0);
        assert_eq!(read_word(&mut d, base + 0x100), 0, "data drains JEOCF");
    }

    #[test]
    fn dfsdm_continuous_rearms() {
        let (mut d, base) = dfsdm(1650);
        write_word(&mut d, base + 0xb0, 1 << 1); // REOCIE
        write_word(&mut d, base + 0xa0, 1 | (1 << 17) | (1 << 18)); // DFEN RSWSTART RCONT
        d.tick(1);
        assert_ne!(read_word(&mut d, base + 0xc0) & (1 << 1), 0, "first REOCF");
        assert_eq!(read_word(&mut d, base + 0x110), 0, "consume clears");
        d.tick(1);
        assert_ne!(read_word(&mut d, base + 0xc0) & (1 << 1), 0, "second REOCF");
    }
}
