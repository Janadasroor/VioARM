//! STM32 FMC (Flexible Memory Controller) and QUADSPI/OCTOSPI stubs for H7.
//! The control registers at `0x52004000` (FMC) and `0x52005000` (QUADSPI) are
//! exposed as a register window that does not fault; the external memory
//! itself lives at `0xC0000000` (FMC SDRAM) and `0x90000000` (QSPI) as RAM
//! regions on the board.

use crate::device::svd::{Behavior, Device, NoBehavior, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

#[derive(Debug)]
pub struct Fmc {
    inner: RegisterFile<NoBehavior>,
}

impl Fmc {
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
    #[must_use]
    pub const fn new(region: Region) -> Self {
        Self {
            region,
            enabled: false,
        }
    }
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

#[derive(Debug)]
pub struct Qspi {
    inner: RegisterFile<QspiBehavior>,
}

impl Qspi {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        Self::new_for_with_gate::<D>(None)
    }
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

    #[must_use]
    pub fn octospi2_for<D: Device>() -> Self {
        Self::octospi2_for_with_gate::<D>(None)
    }
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

#[derive(Debug, Default)]
struct DmamuxBehavior {
    req_id: [u8; 16],
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
        // CCR register at 0x00 + 4*ch: DMAREQ_ID is bits 0:6
        let ch = (offset / 4) as usize;
        if ch < 16 {
            self.req_id[ch] = (word & 0x7F) as u8;
        }
        word
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let _ = view;
        let _ = offset;
        value
    }
}

pub struct Dmamux {
    inner: RegisterFile<DmamuxBehavior>,
}

impl Dmamux {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DMAMUX1")
            .or_else(|| D::peripheral("DMAMUX"))
            .expect("generated tables contain DMAMUX1");
        Self {
            inner: RegisterFile::new(def, DmamuxBehavior::default()),
        }
    }
    #[must_use]
    pub fn dmamux2_for<D: Device>() -> Self {
        let def = D::peripheral("DMAMUX2").expect("generated tables contain DMAMUX2");
        Self {
            inner: RegisterFile::new(def, DmamuxBehavior::default()),
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

#[derive(Debug)]
pub struct Ltdc {
    inner: RegisterFile<NoBehavior>,
}

impl Ltdc {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("LTDC").expect("generated tables contain LTDC");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
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
}

#[derive(Debug)]
pub struct Dma2d {
    inner: RegisterFile<NoBehavior>,
}

impl Dma2d {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DMA2D").expect("generated tables contain DMA2D");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
}

#[derive(Debug)]
pub struct Bdma {
    inner: RegisterFile<NoBehavior>,
}

impl Bdma {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("BDMA").expect("generated tables contain BDMA");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
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
}

#[derive(Debug)]
pub struct Mdma {
    inner: RegisterFile<NoBehavior>,
}

impl Mdma {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("MDMA").expect("generated tables contain MDMA");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
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

#[derive(Debug)]
pub struct Sai4 {
    inner: RegisterFile<NoBehavior>,
}

impl Sai4 {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SAI4").expect("generated tables contain SAI4");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
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

#[derive(Debug)]
pub struct Dfsdm {
    inner: RegisterFile<NoBehavior>,
}

impl Dfsdm {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("DFSDM").expect("generated tables contain DFSDM");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
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
}

#[derive(Debug)]
pub struct Hrtim {
    inner: RegisterFile<NoBehavior>,
}

impl Hrtim {
    #[must_use]
    pub fn master_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_Master").expect("generated tables contain HRTIM_Master");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
    #[must_use]
    pub fn tima_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMA").expect("generated tables contain HRTIM_TIMA");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
    #[must_use]
    pub fn timb_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMB").expect("generated tables contain HRTIM_TIMB");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
    #[must_use]
    pub fn timc_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMC").expect("generated tables contain HRTIM_TIMC");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
    #[must_use]
    pub fn timd_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIMD").expect("generated tables contain HRTIM_TIMD");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
    #[must_use]
    pub fn time_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_TIME").expect("generated tables contain HRTIM_TIME");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
        }
    }
    #[must_use]
    pub fn common_for<D: Device>() -> Self {
        let def = D::peripheral("HRTIM_Common").expect("generated tables contain HRTIM_Common");
        Self {
            inner: RegisterFile::new(def, NoBehavior),
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

#[cfg(test)]
mod tests {
    use super::{Dmamux, DmamuxBehavior};
    use crate::device::svd::stm32h743::Stm32h743;
    use crate::device::{svd::Device, Peripheral};

    fn write_word(p: &mut Dmamux, addr: u32, val: u32) {
        for i in 0..4 {
            p.write(addr + i, ((val >> (8 * i)) & 0xFF) as u8).unwrap();
        }
    }
    fn read_word(p: &mut Dmamux, addr: u32) -> u32 {
        let mut w = 0;
        for i in 0..4 {
            w |= u32::from(p.read(addr + i).unwrap()) << (8 * i);
        }
        w
    }

    #[test]
    fn dmamux_routes_dma_req_id() {
        let mut d = Dmamux::new_for::<Stm32h743>();
        let base = d.ranges()[0].base;
        // CCR0 DMAREQ_ID = 0x12
        write_word(&mut d, base, 0x12);
        assert_eq!(read_word(&mut d, base) & 0x7F, 0x12);
        // CCR1
        write_word(&mut d, base + 4, 0x34);
        assert_eq!(read_word(&mut d, base + 4) & 0x7F, 0x34);
        // Check internal routing table via behavior
        assert_eq!(d.inner.behavior().req_id[0], 0x12);
        assert_eq!(d.inner.behavior().req_id[1], 0x34);
    }
}
