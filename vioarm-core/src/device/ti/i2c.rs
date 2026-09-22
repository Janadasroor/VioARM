//! TI Tiva C I2C masters (`I2C0`-`I2C3`).
//!
//! Master transmit/receive follows the TM4C123GH6PM datasheet (SPMS376E
//! §16): `MSA` address + `R/S`, `MCS` Table 16-5 command decoding
//! (`RUN`/`START`/`STOP`/`ACK`), instant-completion status (`IDLE`,
//! `ADRACK`/`DATACK`/`ERROR`, `BUSBSY`), `MDR` data, `MTPR` period math
//! (`SCL = SysClk / (20 × (1+TPR))` standard mode), `MIMR`/`MRIS`/`MMIS`
//! with `MICR` W1C clear, `MCR.MFE` gating.
//!
//! Like the repo's other serials, transfers complete synchronously on the
//! `MCS` write — no SCL timing is modeled (`BUSY` reads 0 after the write
//! lands, so `while (MCS & BUSY)` poll loops fall through; documented).
//! Slave devices attach through [`I2c::attach_eeprom`] (address → byte
//! array with a sequential offset, the EEPROM pattern TivaWare examples
//! use); an unattached address NACKs (`ADRACK` + `ERROR`). The slave
//! windows (`SOAR`…`SACKCTL`), `LPBK`, high-speed mode, clock-timeout,
//! and arbitration stay latched/unmodeled (documented).

use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::HashMap;

const MSA_OFF: u32 = 0x000;
const MCS_OFF: u32 = 0x004;
const MDR_OFF: u32 = 0x008;
const MTPR_OFF: u32 = 0x00C;
const MIMR_OFF: u32 = 0x010;
const MRIS_OFF: u32 = 0x014;
const MMIS_OFF: u32 = 0x018;
const MICR_OFF: u32 = 0x01C;
const MCR_OFF: u32 = 0x020;
const MBMON_OFF: u32 = 0x02C;

/// `MCS` status bits (read view).
const MCS_BUSBSY: u32 = 1 << 6;
const MCS_IDLE: u32 = 1 << 5;
const MCS_ADRACK: u32 = 1 << 2;
const MCS_DATACK: u32 = 1 << 3;
const MCS_ERROR: u32 = 1 << 1;
const MCS_BUSY: u32 = 1 << 0;
/// `MCS` command bits (write view).
const MCS_ACK: u32 = 1 << 3;
const MCS_STOP: u32 = 1 << 2;
const MCS_START: u32 = 1 << 1;
const MCS_RUN: u32 = 1 << 0;
/// `MCR.MFE` bit 4 (master function enable).
const MCR_MFE: u32 = 1 << 4;
/// Master interrupt bit (MIMR/MRIS/MMIS/MICR bit 0).
const M_INT: u32 = 1 << 0;

/// Nonzero datasheet resets stamped over the SVD zeros (`MCS` reads
/// `IDLE`, `MTPR` powers at `0x01`, `MBMON` shows an idle bus).
const RESETS: &[(u32, u32)] = &[
    (MCS_OFF, MCS_IDLE),
    (MTPR_OFF, 0x0000_0001),
    (MBMON_OFF, 0x0000_0003),
];

/// Standard-mode SCL rate in Hz: `SysClk / (20 × (1 + TPR))`
/// (`SCL_PERIOD = 2×(1+TPR)×(6+4)×CLK_PRD`, datasheet `I2CMTPR` page).
#[must_use]
pub fn scl_hz_for(tpr: u32, sysclk_hz: u32) -> u32 {
    let tpr = tpr & 0x7F;
    if sysclk_hz == 0 {
        return 0;
    }
    u32::try_from(u64::from(sysclk_hz) / (20 * (u64::from(tpr) + 1))).unwrap_or(u32::MAX)
}

/// Master engine state (Table 16-5 rows).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum MasterState {
    #[default]
    Idle,
    Transmit,
    Receive,
}

/// Attached slave memory (EEPROM pattern): sequential offset, wraps.
#[derive(Clone, Debug)]
struct SlaveMem {
    bytes: Vec<u8>,
    offset: usize,
}

/// Behavior behind one I2C window (master only).
#[derive(Debug, Default)]
pub struct I2cBehavior {
    state: MasterState,
    /// Last status nibble (`ADRACK`/`DATACK`/`ERROR` held until the next
    /// command completes, like silicon holding status).
    status: u32,
    /// `BUSBSY` latch (set by `START`, cleared by `STOP`/error/`MCR` off).
    bus_busy: bool,
    /// Master interrupt raw flag.
    mris: bool,
    /// Attached slave memories by 7-bit address.
    slaves: HashMap<u8, SlaveMem>,
    /// Cached `(MRIS & MIMR) != 0` (`InterruptSource::pending` is `&self`).
    pending: bool,
}

impl I2cBehavior {
    fn mfe(&self, view: &Registers<&mut [u32]>) -> bool {
        view.get(MCR_OFF) & MCR_MFE != 0
    }

    /// Execute a Table 16-5 command word against the addressed slave.
    fn command(&mut self, view: &mut Registers<&mut [u32]>, cmd: u32) {
        let run = cmd & MCS_RUN != 0;
        let start = cmd & MCS_START != 0;
        let stop = cmd & MCS_STOP != 0;
        let ack = cmd & MCS_ACK != 0;
        if !self.mfe(view) || !run && !stop {
            return; // Disabled, or a NOP encoding.
        }
        let msa = view.get(MSA_OFF);
        let addr = ((msa >> 1) & 0x7F) as u8;
        let rs = msa & 0x01 != 0;
        if stop && !run && !start {
            // STOP alone: release the bus back to Idle.
            self.state = MasterState::Idle;
            self.bus_busy = false;
            self.mris = true;
            self.refresh(view);
            return;
        }
        if !run {
            return; // Remaining `RUN=0` encodings are NOPs.
        }
        if start {
            self.bus_busy = true;
            // (Repeated) START: (re-)address the slave.
            if !self.slaves.contains_key(&addr) {
                self.status = MCS_ADRACK | MCS_ERROR;
                self.mris = true;
                if stop {
                    self.state = MasterState::Idle;
                    self.bus_busy = false;
                }
                self.refresh(view);
                return;
            }
            self.status = 0;
            self.state = if rs {
                MasterState::Receive
            } else {
                MasterState::Transmit
            };
            let _ = ack;
            if stop && !rs {
                // START + TRANSMIT + STOP: single-byte write below.
                self.transmit_byte(view, addr);
                self.state = MasterState::Idle;
                self.bus_busy = false;
                self.mris = true;
                self.refresh(view);
                return;
            }
            if stop && rs {
                // START + RECEIVE + STOP: single-byte read below.
                self.receive_byte(view, addr, ack);
                self.state = MasterState::Idle;
                self.bus_busy = false;
                self.mris = true;
                self.refresh(view);
                return;
            }
            self.mris = true;
            self.refresh(view);
            return;
        }
        // No `START`: continue in the current state.
        match self.state {
            MasterState::Transmit => {
                self.transmit_byte(view, addr);
                if stop {
                    self.state = MasterState::Idle;
                    self.bus_busy = false;
                }
            }
            MasterState::Receive => {
                self.receive_byte(view, addr, ack);
                if stop {
                    self.state = MasterState::Idle;
                    self.bus_busy = false;
                }
            }
            MasterState::Idle => {}
        }
        self.mris = true;
        self.refresh(view);
    }

    /// Master-transmit one `MDR` byte to the slave at `addr`.
    fn transmit_byte(&mut self, view: &mut Registers<&mut [u32]>, addr: u8) {
        let byte = (view.get(MDR_OFF) & 0xFF) as u8;
        if let Some(slave) = self.slaves.get_mut(&addr) {
            if slave.bytes.is_empty() {
                self.status = MCS_DATACK | MCS_ERROR;
                return;
            }
            let at = slave.offset % slave.bytes.len();
            slave.bytes[at] = byte;
            slave.offset = slave.offset.wrapping_add(1);
            self.status = 0;
        } else {
            self.status = MCS_ADRACK | MCS_ERROR;
        }
    }

    /// Master-receive one byte from the slave at `addr` into `MDR`.
    fn receive_byte(&mut self, view: &mut Registers<&mut [u32]>, addr: u8, _ack: bool) {
        if let Some(slave) = self.slaves.get_mut(&addr) {
            let byte = if slave.bytes.is_empty() {
                0xFF
            } else {
                let b = slave.bytes[slave.offset % slave.bytes.len()];
                slave.offset = slave.offset.wrapping_add(1);
                b
            };
            view.set(MDR_OFF, u32::from(byte));
            self.status = 0;
        } else {
            self.status = MCS_ADRACK | MCS_ERROR;
        }
    }

    fn status_word(&self) -> u32 {
        // `IDLE` reads 1 in Idle; `BUSY` always 0 post-completion
        // (instant transfers, documented); `BUSBSY` tracks the bus.
        let mut w = self.status;
        if self.state == MasterState::Idle {
            w |= MCS_IDLE;
        }
        if self.bus_busy {
            w |= MCS_BUSBSY;
        }
        w
    }

    fn refresh(&mut self, view: &Registers<&mut [u32]>) {
        self.pending = self.mris && view.get(MIMR_OFF) & M_INT != 0;
    }
}

impl Behavior for I2cBehavior {
    fn needs_tick() -> bool {
        false
    }

    fn reset(&mut self) {
        let slaves = std::mem::take(&mut self.slaves);
        *self = Self::default();
        self.slaves = slaves;
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let out = match offset {
            MCS_OFF => self.status_word(),
            MRIS_OFF => u32::from(self.mris),
            MMIS_OFF => u32::from(self.mris && view.get(MIMR_OFF) & M_INT != 0),
            _ => value,
        };
        self.refresh(view);
        out
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            MCS_OFF => {
                // Write-only control view: decode the command, store
                // nothing (reads show status, never the command).
                // Firmware word-stores commands; lane 0 carries bits 3:0
                // and fires, upper zero lanes decode to NOP.
                let cmd = (u32::from(val) << shift) & 0x0F;
                self.command(view, cmd);
                view.get(offset)
            }
            MRIS_OFF | MMIS_OFF => view.get(offset),
            MICR_OFF => {
                if (view.merged(offset, val, shift)) & M_INT != 0 {
                    self.mris = false;
                }
                self.refresh(view);
                view.get(offset)
            }
            _ => {
                let word = view.merged(offset, val, shift);
                view.set(offset, word);
                self.refresh(view);
                word
            }
        }
    }
}

/// A Tiva C I2C controller (master mode; slave windows latched).
pub struct I2c {
    inner: RegisterFile<I2cBehavior>,
}

impl I2c {
    /// A new I2C controller on `D`'s tables (`"I2C0"`…`"I2C3"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain I2C");
        let mut i2c = Self {
            inner: RegisterFile::new(def, I2cBehavior::default()),
        };
        i2c.stamp_resets();
        i2c
    }

    /// Stamp the nonzero datasheet resets over the SVD zeros.
    fn stamp_resets(&mut self) {
        let def = self.inner.def();
        let stamped: Vec<(usize, u32)> = RESETS
            .iter()
            .filter_map(|&(offset, value)| {
                def.registers
                    .iter()
                    .position(|r| r.offset == offset)
                    .map(|idx| (idx, value))
            })
            .collect();
        let words = self.inner.regs_mut();
        for (idx, value) in stamped {
            words[idx] = value;
        }
    }

    /// Attach an EEPROM-pattern slave memory at a 7-bit address.
    pub fn attach_eeprom(&mut self, addr: u8, bytes: Vec<u8>) {
        self.inner.with_view(|_, b| {
            b.slaves.insert(addr & 0x7F, SlaveMem { bytes, offset: 0 });
        });
    }

    /// Snapshot an attached slave's memory (test/host introspection).
    #[must_use]
    pub fn slave_snapshot(&mut self, addr: u8) -> Option<Vec<u8>> {
        self.inner
            .with_view(|_, b| b.slaves.get(&(addr & 0x7F)).map(|s| s.bytes.clone()))
    }

    /// SCL rate in Hz over an explicit system-clock rate.
    #[must_use]
    pub fn scl_hz_over(&mut self, sysclk_hz: u32) -> u32 {
        self.inner
            .with_view(|view, _| scl_hz_for(view.get(MTPR_OFF), sysclk_hz))
    }
}

impl Peripheral for I2c {
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
        self.stamp_resets();
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for I2c {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        u32::from(self.inner.behavior().pending)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const I2C0_BASE: u32 = 0x4002_0000;
    /// `RUN | START | STOP`.
    const CMD_TX_SINGLE: u32 = MCS_RUN | MCS_START | MCS_STOP;
    /// `RUN | START` (transmit, stay on the bus).
    const CMD_TX: u32 = MCS_RUN | MCS_START;

    fn i2c0() -> I2c {
        I2c::for_device::<Tm4c123gh6pm>("I2C0")
    }

    fn read_word(i2c: &mut I2c, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(i2c.read(I2C0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(i2c: &mut I2c, offset: u32, value: u32) {
        for lane in 0..4 {
            i2c.write(
                I2C0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Master enable: `MFE` set.
    fn enable(i2c: &mut I2c) {
        write_word(i2c, MCR_OFF, MCR_MFE);
    }

    /// Datasheet resets: `MCS` reads `IDLE`, `MTPR` is `0x01`.
    #[test]
    fn datasheet_resets() {
        let mut i2c = i2c0();
        assert_eq!(read_word(&mut i2c, MCS_OFF), MCS_IDLE);
        assert_eq!(read_word(&mut i2c, MTPR_OFF), 0x01);
        assert_eq!(read_word(&mut i2c, MRIS_OFF), 0);
    }

    /// Standard-mode rate math: `TPR=7` at 16 MHz is 100 kHz.
    #[test]
    fn mtpr_math_is_100khz() {
        assert_eq!(scl_hz_for(7, 16_000_000), 100_000);
        let mut i2c = i2c0();
        write_word(&mut i2c, MTPR_OFF, 7);
        assert_eq!(i2c.scl_hz_over(16_000_000), 100_000);
    }

    /// Single-byte master write lands in the attached EEPROM; the bus
    /// returns to Idle with the master interrupt raised.
    #[test]
    fn master_write_single_byte() {
        let mut i2c = i2c0();
        enable(&mut i2c);
        i2c.attach_eeprom(0x50, vec![0x00; 16]);
        write_word(&mut i2c, MIMR_OFF, M_INT);
        write_word(&mut i2c, MSA_OFF, 0x50 << 1); // R/S = 0 (transmit)
        write_word(&mut i2c, MDR_OFF, 0xA5);
        write_word(&mut i2c, MCS_OFF, CMD_TX_SINGLE);
        assert_eq!(read_word(&mut i2c, MCS_OFF) & (MCS_ERROR | MCS_BUSBSY), 0);
        assert_eq!(read_word(&mut i2c, MCS_OFF) & MCS_IDLE, MCS_IDLE);
        assert_eq!(read_word(&mut i2c, MRIS_OFF), M_INT);
        assert_eq!(i2c.pending(), 1);
        assert_eq!(i2c.exception_number(), Some(24));
        write_word(&mut i2c, MICR_OFF, M_INT);
        assert_eq!(i2c.pending(), 0);
        // The byte landed at slave offset 0; a following single read
        // returns the next byte (offset advanced to 1).
        assert_eq!(i2c.slave_snapshot(0x50).unwrap()[0], 0xA5);
        write_word(&mut i2c, MSA_OFF, (0x50 << 1) | 0x01); // R/S = 1
        write_word(&mut i2c, MCS_OFF, CMD_TX_SINGLE);
        assert_eq!(read_word(&mut i2c, MDR_OFF) & 0xFF, 0x00);
    }

    /// Multi-byte write with repeated START stays on the bus until STOP.
    #[test]
    fn master_write_stays_on_bus() {
        let mut i2c = i2c0();
        enable(&mut i2c);
        i2c.attach_eeprom(0x50, vec![0x00; 16]);
        write_word(&mut i2c, MSA_OFF, 0x50 << 1);
        write_word(&mut i2c, MCS_OFF, CMD_TX); // START + address, no STOP
        assert_eq!(read_word(&mut i2c, MCS_OFF) & MCS_IDLE, 0);
        write_word(&mut i2c, MDR_OFF, 0x11);
        write_word(&mut i2c, MCS_OFF, MCS_RUN); // TRANSMIT
        write_word(&mut i2c, MDR_OFF, 0x22);
        write_word(&mut i2c, MCS_OFF, MCS_RUN | MCS_STOP); // TRANSMIT + STOP
        assert_eq!(read_word(&mut i2c, MCS_OFF) & MCS_IDLE, MCS_IDLE);
        // Both bytes landed in order.
        assert_eq!(i2c.slave_snapshot(0x50).unwrap()[..2], [0x11, 0x22]);
    }

    /// An unattached address NACKs: `ADRACK` + `ERROR`, bus released on STOP.
    #[test]
    fn missing_slave_nacks() {
        let mut i2c = i2c0();
        enable(&mut i2c);
        write_word(&mut i2c, MSA_OFF, 0x51 << 1);
        write_word(&mut i2c, MDR_OFF, 0xFF);
        write_word(&mut i2c, MCS_OFF, CMD_TX_SINGLE);
        let status = read_word(&mut i2c, MCS_OFF);
        assert_eq!(status & (MCS_ADRACK | MCS_ERROR), MCS_ADRACK | MCS_ERROR);
        assert_eq!(status & MCS_IDLE, MCS_IDLE);
    }

    /// With `MFE` clear, commands are NOPs: no interrupt, still Idle.
    #[test]
    fn mfe_gates_commands() {
        let mut i2c = i2c0();
        write_word(&mut i2c, MIMR_OFF, M_INT);
        write_word(&mut i2c, MSA_OFF, 0x50 << 1);
        write_word(&mut i2c, MCS_OFF, CMD_TX_SINGLE);
        assert_eq!(read_word(&mut i2c, MRIS_OFF), 0);
        assert_eq!(read_word(&mut i2c, MCS_OFF) & MCS_IDLE, MCS_IDLE);
    }
}
