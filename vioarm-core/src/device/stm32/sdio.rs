//! STM32 SDIO (`SDIO`, `0x4001_8000` on F1 / `0x4001_2C00` on F4), SVD-driven.
//!
//! CMD-accurate plus a paced data path over a 1 MiB card image: `ARG`/`CMD`
//! with `CPSMEN` latches `RESPCMD`/`RESP1`, sets `STA.CMDSENT`/`CMDREND` and
//! completes `DATAEND`/`DBCKEND` as bytes move; `ICR` W1C, `MASK` gates
//! `SDIO` IRQ 49.
//!
//! Data timing: bytes stream card↔FIFO over ticks derived from `CLKCR`
//! (`CLKDIV+2` divisor, `WIDBUS` 1/4/8 lines) and the high-speed flag
//! (`CMD6` switch doubles the rate), so bus width and speed are observable
//! in tick counts. `CMD18`/`CMD25` stream multi-block runs (`DLEN` bytes
//! from consecutive blocks) with `DATAEND` deferred to `CMD12`.
//! `DTIMER` bounds the transfer in the same tick units: overrun latches
//! `DTIMEOUT` and aborts. A transfer-starting command (`CMD17`/`18`/`24`/
//! `25`/data-`CMD6`) while a transfer runs latches `CTIMEOUT` (single data
//! path); `CMD12`/`CMD13`/ACMDs never clash. `CCRCFAIL`/`DCRCFAIL` arrive by injection
//! (`drive_inputs` bits 16/17, clear 18); command CRCs never reach the
//! `CMD` register, so `CCRCFAIL` has no stimulus path. Reads append the
//! block CRC-16 after the payload; `CMD6` serves 64 zero status bytes.
//! FIFO depth is unbounded (firmware-paced); `CMD12` ends multi-block runs
//! unconditionally.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CMD: u32 = 0x0C;
const CLKCR: u32 = 0x04;
const RESPCMD: u32 = 0x10;
const RESP1: u32 = 0x14;
const DTIMER: u32 = 0x24;
const DLEN: u32 = 0x28;
const DCTRL: u32 = 0x2C;
const STA: u32 = 0x34;
const ICR: u32 = 0x38;
const MASK: u32 = 0x3C;
const FIFO: u32 = 0x80;

const STA_CCRCFAIL: u32 = 1 << 0;
const STA_DCRCFAIL: u32 = 1 << 1;
const STA_CTIMEOUT: u32 = 1 << 2;
const STA_DTIMEOUT: u32 = 1 << 3;
const STA_CMDSENT: u32 = 1 << 7;
const STA_CMDREND: u32 = 1 << 6;
const STA_DATAEND: u32 = 1 << 8;
const STA_DBCKEND: u32 = 1 << 10;
const STA_RXDAVL: u32 = 1 << 21;
const STA_TXDAVL: u32 = 1 << 20;

const DCTRL_DTEN: u32 = 1 << 0;
const DCTRL_DTDIR: u32 = 1 << 1;

/// The SD data-line CRC: CRC-16/CCITT-FALSE variant used by SD (`poly
/// 0x1021`, init 0, no reflection), MSB transmitted first.
pub(crate) fn sd_crc16(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in bytes {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[derive(Debug)]
struct SdioBehavior {
    fifo: std::collections::VecDeque<u8>,
    next_is_acmd: bool,
    rca: u32,
    ocr: u32,
    card: Vec<u8>,
    last_arg: u32,
    spi_mode: bool,
    inject_ccrcfail: bool,
    inject_dcrcfail: bool,
    /// High-speed timing (set by `CMD6` switch to function 1).
    hs: bool,
    /// A multi-block run (`CMD18`/`CMD25`) awaiting `CMD12`.
    multi: bool,
    /// Bytes of the current transfer still to move, with its deadline.
    xfer: Option<Xfer>,
}

/// One paced data transfer: reads stream staged bytes→FIFO on ticks,
/// writes complete when firmware fills the FIFO; both die past `DTIMER`.
#[derive(Debug)]
struct Xfer {
    /// Host-to-card (`DCTRL.DTDIR` set).
    write: bool,
    /// Payload bytes expected (writes: `DLEN` at dispatch).
    total: usize,
    /// Staged source bytes still to stream (reads: payload + CRCs).
    staged: Vec<u8>,
    /// Staged bytes already moved to the FIFO.
    staged_off: usize,
    /// Ticks elapsed (against `DTIMER`) and ticks budgeted overall.
    elapsed: u64,
    budget: u64,
    /// Bytes streamed per tick (width × speed scaled).
    step: usize,
}

impl Default for SdioBehavior {
    fn default() -> Self {
        Self {
            fifo: std::collections::VecDeque::new(),
            next_is_acmd: false,
            rca: 0x4567_0000,
            ocr: 0x80FF_8000,            // ready, 3.3V
            card: vec![0u8; 512 * 2048], // 1 MiB card image
            last_arg: 0,
            spi_mode: false,
            inject_ccrcfail: false,
            inject_dcrcfail: false,
            hs: false,
            multi: false,
            xfer: None,
        }
    }
}

impl SdioBehavior {
    /// Data-line width in bytes per tick unit from `CLKCR.WIDBUS`.
    fn width_bytes(view: &Registers<&mut [u32]>) -> usize {
        match (view.get(CLKCR) >> 11) & 0x3 {
            1 => 4,
            2 => 8,
            _ => 1,
        }
    }

    /// Ticks for `n` bytes: `(CLKDIV+2)` time quanta per line-round,
    /// halved in high-speed mode.
    fn transfer_ticks(&self, view: &Registers<&mut [u32]>, n: usize) -> u64 {
        let clkdiv = u64::from(view.get(CLKCR) & 0xFF) + 2;
        let per_tick = (Self::width_bytes(view) * if self.hs { 2 } else { 1 }) as u64;
        (n as u64 * clkdiv / per_tick.max(1)).max(1)
    }

    /// Card-image offset for `ARG`: block numbers (`CMD18`/`25` style,
    /// `ARG < image` and block-aligned) or byte addresses (SDSC).
    fn block_offset(&self, arg: u32) -> usize {
        let block = if arg >= 512 && arg % 512 == 0 && (arg as usize) < self.card.len() {
            (arg as usize) / 512
        } else {
            arg as usize
        };
        (block * 512).min(self.card.len())
    }

    /// Start a paced transfer for a data command: stage the read source
    /// (payload plus one CRC-16 per 512-byte block, as the data lines
    /// carry them) or arm a write commit; `CMD18`/`25` open multi-block
    /// runs with `DATAEND` deferred to `CMD12`.
    fn start_transfer(&mut self, view: &mut Registers<&mut [u32]>, idx: u32, sta: &mut u32) {
        self.last_arg = view.get(0x08);
        let dlen = view.get(DLEN) as usize;
        let write = view.get(DCTRL) & DCTRL_DTDIR != 0;
        self.multi = idx == 18 || idx == 25;
        self.fifo.clear();
        if dlen == 0 {
            *sta |= STA_DATAEND | STA_DBCKEND;
            return;
        }
        let budget = self.transfer_ticks(view, dlen);
        let step = (dlen as u64 / budget.max(1)).max(1) as usize;
        if !write {
            let staged = if idx == 6 {
                // `CMD6` function status: 64 bytes, group-1 function
                // selected when high-speed switched.
                let mut s = vec![0u8; 64];
                if self.hs {
                    s[13] = 0x01;
                }
                let crc = sd_crc16(&s);
                s.push((crc >> 8) as u8);
                s.push((crc & 0xFF) as u8);
                s
            } else {
                // Payload from consecutive blocks plus per-block CRCs.
                let mut s = Vec::with_capacity(dlen + 2 * dlen.div_ceil(512));
                let mut left = dlen;
                let mut off = self.block_offset(self.last_arg);
                while left > 0 {
                    let n = left.min(512);
                    let mut blk = vec![0xFFu8; n];
                    for (i, b) in blk.iter_mut().enumerate() {
                        *b = self.card.get(off + i).copied().unwrap_or(0xFF);
                    }
                    let crc = sd_crc16(&blk);
                    s.extend_from_slice(&blk);
                    s.push((crc >> 8) as u8);
                    s.push((crc & 0xFF) as u8);
                    left -= n;
                    off = off.saturating_add(n).min(self.card.len());
                }
                s
            };
            self.xfer = Some(Xfer {
                write: false,
                total: dlen,
                staged,
                staged_off: 0,
                elapsed: 0,
                budget,
                step,
            });
        } else {
            *sta |= STA_TXDAVL;
            self.xfer = Some(Xfer {
                write: true,
                total: dlen,
                staged: Vec::new(),
                staged_off: 0,
                elapsed: 0,
                budget,
                step,
            });
        }
    }

    /// Commit a filled write transfer to consecutive card blocks.
    fn commit_write(&mut self, view: &mut Registers<&mut [u32]>) {
        let arg = self.last_arg;
        let off = self.block_offset(arg);
        if let Some(x) = self.xfer.take() {
            for (i, b) in self.fifo.iter().enumerate().take(x.total) {
                if off + i < self.card.len() {
                    self.card[off + i] = *b;
                }
            }
        }
        let mut sta = view.get(STA) | STA_DBCKEND;
        if !self.multi {
            sta |= STA_DATAEND;
        }
        view.set(STA, sta);
    }

    /// Latch a data timeout and drop the transfer (partial FIFO stays).
    fn data_timeout(&mut self, view: &mut Registers<&mut [u32]>) {
        self.xfer = None;
        let sta = view.get(STA) | STA_DTIMEOUT;
        view.set(STA, sta);
    }
}

impl Behavior for SdioBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        let Some(x) = self.xfer.as_mut() else {
            return;
        };
        x.elapsed = x.elapsed.saturating_add(delta);
        // Deadline first: `DTIMER` bounds the run in the same tick units
        // (0 disables, for firmware that never programs it).
        let dtimer = u64::from(view.get(DTIMER));
        if dtimer != 0 && x.elapsed > dtimer {
            self.data_timeout(view);
            return;
        }
        if x.write {
            // Firmware-paced: the FIFO write arm commits when full.
            return;
        }
        let n = x.step.min(x.staged.len() - x.staged_off);
        for _ in 0..n {
            let b = x.staged[x.staged_off];
            x.staged_off += 1;
            self.fifo.push_back(b);
        }
        if x.staged_off >= x.staged.len() {
            self.xfer = None;
            let mut sta = view.get(STA) | STA_DBCKEND;
            if !self.multi {
                sta |= STA_DATAEND;
            }
            view.set(STA, sta);
        }
    }
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        let cur = view.get(offset);
        let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
        match offset {
            CMD => {
                // `CPSMEN` bit 10 starts the command.
                if word & (1 << 10) != 0 {
                    let idx = word & 0x3F;
                    let waitresp = (word >> 6) & 0x3;
                    let ce_atacmd = (word >> 14) & 1 != 0;
                    view.set(RESPCMD, idx);
                    // Card FSM stub: CMD0/2/3/7/8/55/41 with RCA/OCR.
                    let mut resp1 = 0x0000_0123u32;
                    let is_acmd = self.next_is_acmd || ce_atacmd;
                    self.next_is_acmd = false;
                    match idx {
                        0 => {
                            // GO_IDLE_STATE — in SPI mode, card enters idle (R1 0x01)
                            self.rca = 0;
                            self.spi_mode = true;
                            resp1 = if self.spi_mode { 0x01 } else { 0 };
                        }
                        2 => {
                            // ALL_SEND_CID — 136-bit, use RESP1..4
                            view.set(0x14, 0x1234_5678);
                            view.set(0x18, 0x9ABC_DEF0);
                            view.set(0x1C, 0x1357_9BDF);
                            view.set(0x20, 0x2468_ACF0);
                            resp1 = 0x1234_5678;
                        }
                        3 => {
                            // SEND_RELATIVE_ADDR — RCA
                            resp1 = self.rca;
                            view.set(RESP1, resp1);
                        }
                        7 => {
                            // SELECT_CARD — RCA in ARG
                            let arg = view.get(0x08);
                            if arg == self.rca {
                                resp1 = 0x0000_0700;
                            }
                        }
                        8 => {
                            // SEND_IF_COND — echo pattern
                            resp1 = 0x0000_01AA;
                        }
                        55 => {
                            // APP_CMD — next is ACMD
                            self.next_is_acmd = true;
                            resp1 = 0x0000_0120;
                        }
                        41 if is_acmd => {
                            // SD_APP_OP_COND — OCR ready
                            resp1 = self.ocr;
                        }
                        6 if is_acmd => {
                            // ACMD6 SET_BUS_WIDTH — accepted; the host-side
                            // `CLKCR.WIDBUS` governs pacing (documented).
                            resp1 = 0x0000_0900;
                        }
                        6 => {
                            // CMD6 SWITCH_FUNC — function 1 of group 1 with
                            // switch mode enters high-speed timing.
                            let arg = view.get(0x08);
                            if arg >> 31 & 1 == 1 && arg & 0xF == 1 {
                                self.hs = true;
                            }
                            resp1 = 0x0000_0900;
                        }
                        12 if self.multi => {
                            // STOP_TRANSMISSION ends the run: the data end
                            // lands here, unconditionally.
                            let sta = view.get(STA) | STA_DATAEND;
                            view.set(STA, sta);
                            self.multi = false;
                            self.xfer = None;
                            resp1 = 0x0000_0900;
                        }
                        13 => {
                            // SEND_STATUS — ready/idle outside transfers.
                            resp1 = if self.xfer.is_some() {
                                0x0000_0C00
                            } else {
                                0x0000_0900
                            };
                        }
                        _ => {}
                    }
                    if waitresp != 0 || self.spi_mode {
                        view.set(RESP1, resp1);
                    }
                    let mut sta = view.get(STA) | STA_CMDSENT;
                    if waitresp != 0 {
                        sta |= STA_CMDREND;
                    }
                    // CRC error injection: next command/data will latch CCRCFAIL/DCRCFAIL.
                    if self.inject_ccrcfail {
                        sta |= STA_CCRCFAIL;
                        self.inject_ccrcfail = false;
                    }
                    if self.inject_dcrcfail && view.get(DCTRL) & DCTRL_DTEN != 0 {
                        sta |= STA_DCRCFAIL;
                        self.inject_dcrcfail = false;
                    }
                    // Only transfer-starting commands engage the data path
                    // (`CMD12`/`CMD13`/ACMDs never do, even with `DTEN`
                    // sticky); a second starter mid-run hits the single
                    // data path and times out.
                    let starts = matches!(idx, 17 | 18 | 24 | 25) || (idx == 6 && !is_acmd);
                    if view.get(DCTRL) & DCTRL_DTEN != 0 && starts {
                        if self.xfer.is_some() {
                            sta |= STA_CTIMEOUT;
                            view.set(STA, sta);
                            return word & !(1 << 10);
                        }
                        self.start_transfer(view, idx, &mut sta);
                    }
                    view.set(STA, sta);
                    // CPSMEN self-clears after dispatch.
                    return word & !(1 << 10);
                }
                word
            }
            FIFO => {
                // Byte-lane write into FIFO (controller → card, DTDIR=1).
                self.fifo.push_back(val);
                let sta = view.get(STA) | STA_TXDAVL;
                view.set(STA, sta);
                // A filled write transfer commits to the card image.
                if let Some(x) = self.xfer.as_ref() {
                    if x.write && self.fifo.len() >= x.total {
                        self.commit_write(view);
                    }
                }
                // FIFO is not a stored word; keep the register word as 0.
                0
            }
            ICR => {
                // W1C into STA.
                let sta = view.get(STA) & !word;
                view.set(STA, sta);
                if sta & (STA_RXDAVL | STA_TXDAVL) == 0 {
                    // Clear FIFO when data complete is cleared.
                    if word & (STA_DATAEND | STA_DBCKEND) != 0 {
                        self.fifo.clear();
                    }
                }
                0
            }
            _ => word,
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == FIFO {
            if let Some(b) = self.fifo.pop_front() {
                let mut sta = view.get(STA);
                if self.fifo.is_empty() {
                    sta &= !STA_RXDAVL;
                    view.set(STA, sta);
                }
                // Return the byte in the appropriate lane (word read will assemble).
                return u32::from(b);
            }
            return 0;
        }
        if offset == STA {
            // Reflect FIFO status.
            let mut sta = value;
            if !self.fifo.is_empty() {
                sta |= STA_RXDAVL;
            } else {
                sta &= !STA_RXDAVL;
            }
            return sta;
        }
        value
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let sta = view.get(STA);
        let mask = view.get(MASK);
        if sta & mask != 0 {
            1
        } else {
            0
        }
    }

    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, value: u32) {
        // Bit 16 => inject CCRCFAIL on next command, bit 17 => DCRCFAIL on next data.
        if value & (1 << 16) != 0 {
            self.inject_ccrcfail = true;
        }
        if value & (1 << 17) != 0 {
            self.inject_dcrcfail = true;
        }
        // Bit 18 clears injected flags.
        if value & (1 << 18) != 0 {
            self.inject_ccrcfail = false;
            self.inject_dcrcfail = false;
        }
    }

    fn reset(&mut self) {
        self.fifo.clear();
        self.inject_ccrcfail = false;
        self.inject_dcrcfail = false;
        self.hs = false;
        self.multi = false;
        self.xfer = None;
    }
}

/// Secure digital I/O host.
#[derive(Debug)]
pub struct Sdio {
    inner: RegisterFile<SdioBehavior>,
}
impl Sdio {
    /// A new `Sdio` on another chip's device tables.
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SDIO")
            .or_else(|| D::peripheral("SDMMC1"))
            .or_else(|| D::peripheral("SDMMC"))
            .expect("generated tables contain SDIO/SDMMC1");
        Self {
            inner: RegisterFile::new(def, SdioBehavior::default()),
        }
    }
    /// A new `SDMMC2` on another chip's device tables.
    #[must_use]
    pub fn new_for_sdmmc2<D: Device>() -> Self {
        let def = D::peripheral("SDMMC2").expect("generated tables contain SDMMC2");
        Self {
            inner: RegisterFile::new(def, SdioBehavior::default()),
        }
    }
}
impl Sdio {
    /// Test access to the backing card image.
    #[cfg(test)]
    pub(crate) fn inner_behavior_card(&mut self) -> &mut Vec<u8> {
        &mut self.inner.behavior_mut().card
    }
}

impl Peripheral for Sdio {
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
        "sdio"
    }
}
impl InterruptSource for Sdio {
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
    use super::{sd_crc16, Sdio, CMD, FIFO};
    use crate::device::svd::stm32f103::Stm32f103;
    use crate::device::svd::Device;
    use crate::device::{InterruptSource, Peripheral};
    fn write_word(s: &mut Sdio, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            s.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }
    fn read_word(s: &mut Sdio, addr: u32) -> u32 {
        let mut b = [0u8; 4];
        for (i, v) in b.iter_mut().enumerate() {
            *v = s.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(b)
    }
    #[test]
    fn sdio_registers_do_not_fault() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        write_word(&mut s, base, 0x03); // POWER
        assert_eq!(read_word(&mut s, base) & 0x03, 0x03);
        assert!(matches!(
            s.read(base + 0x500),
            Err(crate::error::BusError::Reserved)
        ));
    }

    #[test]
    fn sdio_cmd_sets_sta_and_irq() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        // Enable CMDSENT interrupt.
        write_word(&mut s, base + 0x3C, 1 << 7); // MASK CMDSENTIE
                                                 // CMD0 GO_IDLE, no response, CPSMEN.
        write_word(&mut s, base + 0x08, 0x0000_0000); // ARG
        write_word(&mut s, base + 0x0C, 0x0000_0400); // CMDINDEX 0 + CPSMEN
        assert_ne!(read_word(&mut s, base + 0x34) & (1 << 7), 0, "CMDSENT set");
        assert_ne!(s.pending(), 0, "IRQ pending via MASK");
        // CMD8 SEND_IF_COND, WAITRESP=01, CPSMEN -> CMDREND also.
        write_word(&mut s, base + 0x3C, (1 << 6) | (1 << 7));
        write_word(&mut s, base + 0x0C, 0x0000_0448); // idx 8, WAITRESP 01, CPSMEN
        assert_ne!(read_word(&mut s, base + 0x34) & (1 << 6), 0, "CMDREND set");
        assert_eq!(read_word(&mut s, base + 0x10) & 0x3F, 8, "RESPCMD latched");
        // ICR clears.
        write_word(&mut s, base + 0x38, (1 << 6) | (1 << 7));
        assert_eq!(read_word(&mut s, base + 0x34) & ((1 << 6) | (1 << 7)), 0);
        assert_eq!(s.pending(), 0);
    }

    #[test]
    fn sdio_data_block_completes() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        // Enable DATAEND.
        write_word(&mut s, base + 0x3C, 1 << 8); // MASK DATAENDIE
        write_word(&mut s, base + 0x28, 512); // DLEN
        write_word(&mut s, base + 0x2C, 0x0000_0001); // DCTRL DTEN
        write_word(&mut s, base + 0x0C, (17 | (1 << 6)) | (1 << 10)); // CMD17 READ_SINGLE_BLOCK + CPSMEN
                                                                      // The paced path streams 512 + 2 CRC bytes, then completes.
        for _ in 0..1200 {
            s.tick(1);
            if read_word(&mut s, base + 0x34) & (1 << 8) != 0 {
                break;
            }
        }
        assert_ne!(read_word(&mut s, base + 0x34) & (1 << 8), 0, "DATAEND set");
        assert_ne!(read_word(&mut s, base + 0x34) & (1 << 10), 0, "DBCKEND set");
        assert_ne!(s.pending(), 0);
    }

    #[test]
    fn sdio_card_init_sequence() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        // CMD0
        write_word(&mut s, base + 0x0C, 0x0000_0400);
        assert_eq!(read_word(&mut s, base + 0x10) & 0x3F, 0);
        // CMD8
        write_word(&mut s, base + 0x0C, 0x0000_0448);
        assert_eq!(read_word(&mut s, base + 0x14), 0x0000_01AA);
        // CMD55 -> ACMD41
        write_word(&mut s, base + 0x0C, 0x0000_0477); // 55 + WAITRESP + CPSMEN
        assert_eq!(read_word(&mut s, base + 0x10) & 0x3F, 55);
        write_word(&mut s, base + 0x0C, 0x0000_4469); // 41 + CE_ATACMD (bit14) + WAITRESP + CPSMEN
        assert_ne!(read_word(&mut s, base + 0x14) & 0x8000_0000, 0, "OCR ready");
    }

    /// A block read serves the payload followed by its CRC-16 (XMODEM),
    /// matching the two trailing bytes the card puts on the data lines.
    #[test]
    fn block_read_appends_crc16() {
        let mut s = Sdio::new_for::<Stm32f103>();
        s.reset();
        // Known-answer: CRC-16/XMODEM of "123456789" is 0x31C3.
        assert_eq!(sd_crc16(b"123456789"), 0x31C3);
        // Prime the card image with that pattern.
        {
            let b = s.inner_behavior_card();
            for (i, byte) in b"123456789".iter().enumerate() {
                b[i] = *byte;
            }
        }
        const B: u32 = 0x4001_8000;
        // Read 9 bytes from address 0.
        write_word(&mut s, B + 0x08, 0); // ARG: block 0
        write_word(&mut s, B + super::DLEN, 9);
        write_word(&mut s, B + super::DTIMER, 0x2C);
        write_word(&mut s, B + super::DCTRL, super::DCTRL_DTEN);
        write_word(&mut s, B + CMD, (17 | (1 << 6)) | (1 << 10)); // CMD17
                                                                  // Stream the payload + CRC bytes, then drain.
        for _ in 0..40 {
            s.tick(1);
        }
        let mut got = Vec::new();
        for _ in 0..11 {
            got.push(s.read(B + FIFO).unwrap());
        }
        assert_eq!(&got[..9], b"123456789", "payload served");
        assert_eq!(
            u16::from_be_bytes([got[9], got[10]]),
            0x31C3,
            "trailing CRC-16"
        );
    }

    #[test]
    fn spi_mode_go_idle_sets_idle_response() {
        let mut s = Sdio::new_for::<Stm32f103>();
        s.reset();
        const B: u32 = 0x4001_8000;
        // CMD0 GO_IDLE with CPSMEN
        write_word(&mut s, B + 0x08, 0); // ARG 0
        write_word(&mut s, B + CMD, 0x0000_0400 | (1 << 10));
        // In SPI mode, response is R1 idle 0x01
        assert_eq!(read_word(&mut s, B + super::RESP1) & 0xFF, 0x01);
        // Subsequent read should still work via block read
        write_word(&mut s, B + super::DLEN, 512);
        write_word(&mut s, B + super::DCTRL, super::DCTRL_DTEN);
        write_word(&mut s, B + CMD, (17 | (1 << 6)) | (1 << 10)); // CMD17
        for _ in 0..1200 {
            s.tick(1);
            if read_word(&mut s, B + 0x34) & (1 << 21) != 0 {
                break;
            }
        }
        let _ = s.read(B + super::FIFO).unwrap();
        assert!(s.read(B + super::FIFO).is_ok());
    }

    /// Issue a data command word: index + R1 wait + CPSMEN.
    fn data_cmd(s: &mut Sdio, base: u32, idx: u32) {
        write_word(s, base + 0x0C, idx | (1 << 6) | (1 << 10));
    }

    /// Tick until `STA` shows `mask` (or `max` ticks pass); returns ticks used.
    fn tick_until(s: &mut Sdio, base: u32, mask: u32, max: u64) -> u64 {
        for t in 1..=max {
            s.tick(1);
            if read_word(s, base + 0x34) & mask != 0 {
                return t;
            }
        }
        panic!("STA {mask:#x} never latched");
    }

    /// Multi-block write (`CMD25`) then read (`CMD18`) of 4 blocks with a
    /// byte-compare, CRCs verified per block, `CMD12` ending each run.
    #[test]
    fn multiblock_write_then_read_compare() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        // ---- CMD25: write blocks 10..13 --------------------------------
        let pattern: Vec<u8> = (0..2048).map(|i| ((i * 7 + 3) & 0xFF) as u8).collect();
        write_word(&mut s, base + 0x08, 10 * 512); // ARG block 10
        write_word(&mut s, base + 0x28, 2048); // DLEN
        write_word(&mut s, base + 0x2C, 0x0000_0003); // DCTRL DTEN|DTDIR
        data_cmd(&mut s, base, 25);
        for b in &pattern {
            s.write(base + super::FIFO, *b).unwrap();
        }
        assert_ne!(read_word(&mut s, base + 0x34) & (1 << 10), 0, "DBCKEND");
        assert_eq!(
            read_word(&mut s, base + 0x34) & (1 << 8),
            0,
            "no DATAEND yet"
        );
        data_cmd(&mut s, base, 12); // STOP_TRANSMISSION
        assert_ne!(
            read_word(&mut s, base + 0x34) & (1 << 8),
            0,
            "DATAEND at CMD12"
        );
        // ---- CMD18: read them back --------------------------------------
        write_word(&mut s, base + 0x38, 0xFFFF_FFFF); // ICR clears
        write_word(&mut s, base + 0x08, 10 * 512);
        write_word(&mut s, base + 0x28, 2048);
        write_word(&mut s, base + 0x2C, 0x0000_0001); // DCTRL DTEN read
        data_cmd(&mut s, base, 18);
        let streamed = tick_until(&mut s, base, 1 << 10, 9000);
        assert_eq!(streamed, 2056, "2056 staged bytes at 1 per tick");
        let mut got = Vec::new();
        for _ in 0..2048 + 8 {
            got.push(s.read(base + super::FIFO).unwrap());
        }
        // Payload compares block by block; each block's CRC-16 verifies.
        for b in 0..4 {
            let off = b * 514;
            assert_eq!(
                &got[off..off + 512],
                &pattern[b * 512..(b + 1) * 512],
                "block {b}"
            );
            assert_eq!(
                u16::from_be_bytes([got[off + 512], got[off + 513]]),
                sd_crc16(&pattern[b * 512..(b + 1) * 512]),
                "block {b} CRC"
            );
        }
        assert_eq!(
            read_word(&mut s, base + 0x34) & (1 << 8),
            0,
            "no DATAEND yet"
        );
        data_cmd(&mut s, base, 12);
        assert_ne!(
            read_word(&mut s, base + 0x34) & (1 << 8),
            0,
            "DATAEND at CMD12"
        );
    }

    /// Bus width and high-speed switch observe exactly in transfer ticks
    /// for one 512-byte block (+2 CRC bytes staged): 1-bit 514, 4-bit 257,
    /// 4-bit + HS 129.
    #[test]
    fn bus_width_and_high_speed_scale_pacing() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let read512 = |s: &mut Sdio| {
            write_word(s, base + 0x38, 0xFFFF_FFFF); // ICR clears stale flags
            write_word(s, base + 0x08, 0);
            write_word(s, base + 0x28, 512);
            write_word(s, base + 0x2C, 0x0000_0001);
            data_cmd(s, base, 17);
            // `CMD13` polls tran state mid-transfer without clashing.
            data_cmd(s, base, 13);
            assert_eq!(read_word(s, base + 0x14), 0x0000_0C00);
            tick_until(s, base, 1 << 8, 2000)
        };
        let mut s = Sdio::new_for::<Stm32f103>();
        assert_eq!(read512(&mut s), 514, "1-bit ticks");
        // 4-bit bus.
        s.reset();
        write_word(&mut s, base + 0x04, 1 << 11); // CLKCR WIDBUS 4-bit
        assert_eq!(read512(&mut s), 257, "4-bit ticks");
        // High-speed switch via CMD6 then the same read (data path off:
        // the status block is not wanted here).
        write_word(&mut s, base + 0x2C, 0);
        write_word(&mut s, base + 0x08, 0x80FF_FFF1); // switch, group 1 = 1
        data_cmd(&mut s, base, 6);
        assert_eq!(read_word(&mut s, base + 0x14), 0x0000_0900, "CMD6 R1");
        assert_eq!(read512(&mut s), 129, "4-bit high-speed ticks");
    }

    /// A transfer outrunning `DTIMER` latches `DTIMEOUT` and aborts.
    #[test]
    fn dtimeout_aborts_overrun_transfer() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        write_word(&mut s, base + 0x08, 0);
        write_word(&mut s, base + 0x28, 512);
        write_word(&mut s, base + 0x24, 10); // DTIMER far short of 1024
        write_word(&mut s, base + 0x2C, 0x0000_0001);
        data_cmd(&mut s, base, 17);
        tick_until(&mut s, base, 1 << 3, 100);
        assert_eq!(read_word(&mut s, base + 0x34) & (1 << 8), 0, "no DATAEND");
        assert_eq!(read_word(&mut s, base + 0x34) & (1 << 10), 0, "no DBCKEND");
    }

    /// A second transfer-starting command mid-run latches `CTIMEOUT` and
    /// the first transfer still completes intact.
    #[test]
    fn ctimeout_on_clashing_data_command() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        write_word(&mut s, base + 0x08, 0);
        write_word(&mut s, base + 0x28, 9);
        write_word(&mut s, base + 0x2C, 0x0000_0001);
        data_cmd(&mut s, base, 17);
        for _ in 0..5 {
            s.tick(1);
        }
        // A write command into the running read: refused.
        write_word(&mut s, base + 0x2C, 0x0000_0003);
        data_cmd(&mut s, base, 24);
        assert_ne!(read_word(&mut s, base + 0x34) & (1 << 2), 0, "CTIMEOUT");
        // The read still streams to its end (9 + 2 bytes, all zeros here).
        tick_until(&mut s, base, 1 << 8, 100);
        let mut got = Vec::new();
        for _ in 0..11 {
            got.push(s.read(base + super::FIFO).unwrap());
        }
        assert_eq!(&got[..9], &[0; 9], "read intact");
        assert_eq!(
            u16::from_be_bytes([got[9], got[10]]),
            sd_crc16(&[0; 9]),
            "CRC intact"
        );
    }

    /// `ACMD6`/`CMD6`/`CMD13` answer the card-state words a stack polls.
    #[test]
    fn card_state_responses() {
        let base = Stm32f103::peripheral("SDIO").unwrap().base;
        let mut s = Sdio::new_for::<Stm32f103>();
        // CMD55 then ACMD6 SET_BUS_WIDTH.
        data_cmd(&mut s, base, 55);
        write_word(&mut s, base + 0x08, 2); // 4-bit
        write_word(&mut s, base + 0x0C, 6 | (1 << 6) | (1 << 10) | (1 << 14)); // ACMD6
        assert_eq!(read_word(&mut s, base + 0x14), 0x0000_0900, "ACMD6 ready");
        // Idle status outside transfers.
        data_cmd(&mut s, base, 13);
        assert_eq!(read_word(&mut s, base + 0x14), 0x0000_0900, "tran idle");
    }
}
