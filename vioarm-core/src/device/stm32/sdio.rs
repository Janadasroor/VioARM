//! STM32 SDIO (`SDIO`, `0x4001_8000` on F1 / `0x4001_2C00` on F4), SVD-driven.
//! CMD-accurate plus data stub: `ARG`/`CMD` with `CPSMEN` latches `RESPCMD`/
//! `RESP1`, sets `STA.CMDSENT`/`CMDREND` and `DATAEND`/`DBCKEND` when `DCTRL`/
//! `DLEN` indicate a block, `ICR` W1C, `MASK` gates `SDIO` IRQ 49. FIFO and
//! full card FSM remain stubbed.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CMD: u32 = 0x0C;
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
        }
    }
}

impl Behavior for SdioBehavior {
    // Card transfers complete synchronously in the model.
    fn needs_tick() -> bool {
        false
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
                    // If data path is enabled, complete the block and prime FIFO.
                    if view.get(DCTRL) & DCTRL_DTEN != 0 {
                        sta |= STA_DATAEND | STA_DBCKEND;
                        self.last_arg = view.get(0x08);
                        if view.get(DCTRL) & DCTRL_DTDIR == 0 {
                            let dlen = view.get(DLEN) as usize;
                            self.fifo.clear();
                            let arg = self.last_arg;
                            let block =
                                if arg >= 512 && arg % 512 == 0 && (arg as usize) < self.card.len()
                                {
                                    (arg as usize) / 512
                                } else {
                                    arg as usize
                                };
                            let off = (block * 512).min(self.card.len());
                            for i in 0..dlen.min(self.card.len().saturating_sub(off)) {
                                let b = self.card.get(off + i).copied().unwrap_or(0xFF);
                                self.fifo.push_back(b);
                            }
                            while self.fifo.len() < dlen {
                                self.fifo.push_back(0xFF);
                            }
                            // The card appends the block CRC on the data
                            // lines right after the payload.
                            let served: Vec<u8> = self.fifo.iter().take(dlen).copied().collect();
                            let crc = sd_crc16(&served);
                            self.fifo.push_back((crc >> 8) as u8);
                            self.fifo.push_back((crc & 0xFF) as u8);
                            if !self.fifo.is_empty() {
                                sta |= STA_RXDAVL;
                            }
                        } else {
                            self.fifo.clear();
                            sta |= STA_TXDAVL;
                        }
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
                // When a full block has been written, commit to card image.
                let dlen = view.get(DLEN);
                if dlen != 0 && (self.fifo.len() as u32) >= dlen {
                    let arg = self.last_arg;
                    let block = if arg >= 512 && arg % 512 == 0 && (arg as usize) < self.card.len()
                    {
                        (arg as usize) / 512
                    } else {
                        arg as usize
                    };
                    let off = (block * 512).min(self.card.len());
                    for (i, b) in self.fifo.iter().enumerate().take(dlen as usize) {
                        if off + i < self.card.len() {
                            self.card[off + i] = *b;
                        }
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
    }
}

#[derive(Debug)]
pub struct Sdio {
    inner: RegisterFile<SdioBehavior>,
}
impl Sdio {
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
        write_word(&mut s, base + 0x0C, 0x0000_0440 | (1 << 10)); // CMD17 READ_SINGLE_BLOCK + CPSMEN
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
        write_word(&mut s, B + CMD, 0x0000_0440 | (1 << 10)); // CMD17
                                                              // Drain payload + CRC bytes from the FIFO.
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
        write_word(&mut s, B + CMD, 0x0000_0440 | (1 << 10)); // CMD17
        let _ = s.read(B + super::FIFO).unwrap();
        assert!(s.read(B + super::FIFO).is_ok());
    }
}
