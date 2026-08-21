//! STM32 SDIO (`SDIO`, `0x4001_8000` on F1 / `0x4001_2C00` on F4), SVD-driven.
//! CMD-accurate plus data stub: `ARG`/`CMD` with `CPSMEN` latches `RESPCMD`/
//! `RESP1`, sets `STA.CMDSENT`/`CMDREND` and `DATAEND`/`DBCKEND` when `DCTRL`/
//! `DLEN` indicate a block, `ICR` W1C, `MASK` gates `SDIO` IRQ 49. FIFO and
//! full card FSM remain stubbed.

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

const CMD: u16 = 0x0C;
const RESPCMD: u16 = 0x10;
const RESP1: u16 = 0x14;
const DTIMER: u16 = 0x24;
const DLEN: u16 = 0x28;
const DCTRL: u16 = 0x2C;
const STA: u16 = 0x34;
const ICR: u16 = 0x38;
const MASK: u16 = 0x3C;
const FIFO: u16 = 0x80;

const STA_CMDSENT: u32 = 1 << 7;
const STA_CMDREND: u32 = 1 << 6;
const STA_DATAEND: u32 = 1 << 8;
const STA_DBCKEND: u32 = 1 << 10;
const STA_RXDAVL: u32 = 1 << 21;
const STA_TXDAVL: u32 = 1 << 20;

const DCTRL_DTEN: u32 = 1 << 0;
const DCTRL_DTDIR: u32 = 1 << 1;

#[derive(Debug, Default)]
struct SdioBehavior {
    fifo: std::collections::VecDeque<u8>,
}

impl Behavior for SdioBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u16,
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
                    view.set(RESPCMD, idx);
                    // Dummy response for any CMD with response.
                    if waitresp != 0 {
                        view.set(RESP1, 0x0000_0123);
                    }
                    let mut sta = view.get(STA) | STA_CMDSENT;
                    if waitresp != 0 {
                        sta |= STA_CMDREND;
                    }
                    // If data path is enabled, complete the block and prime FIFO.
                    if view.get(DCTRL) & DCTRL_DTEN != 0 {
                        sta |= STA_DATAEND | STA_DBCKEND;
                        // Prime FIFO with DLEN bytes of 0xFF for reads (card → controller).
                        if view.get(DCTRL) & DCTRL_DTDIR == 0 {
                            let dlen = view.get(DLEN);
                            self.fifo.clear();
                            for _ in 0..dlen.min(512) {
                                self.fifo.push_back(0xFF);
                            }
                            if !self.fifo.is_empty() {
                                sta |= STA_RXDAVL;
                            }
                        } else {
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
                // FIFO is not a stored word; keep the register word as 0.
                return 0;
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
                return 0;
            }
            _ => word,
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u16, value: u32) -> u32 {
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
}

#[derive(Debug)]
pub struct Sdio {
    inner: RegisterFile<SdioBehavior>,
}
impl Sdio {
    #[must_use]
    pub fn new_for<D: Device>() -> Self {
        let def = D::peripheral("SDIO").expect("generated tables contain SDIO");
        Self {
            inner: RegisterFile::new(def, SdioBehavior::default()),
        }
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
    use super::Sdio;
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
        write_word(&mut s, base + 0x00, 0x03); // POWER
        assert_eq!(read_word(&mut s, base + 0x00) & 0x03, 0x03);
        assert!(matches!(s.read(base + 0x500), Err(crate::error::BusError::Reserved)));
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
}
