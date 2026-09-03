//! ARM Instrumentation Trace Macrocell stimulus ports (`0xE000_0000`).
//!
//! Firmware `printf` retargets commonly write stimulus port 0; when the port
//! is enabled (`TER`) and the unit is on (`TCR.ITMENA`) the bytes queue into
//! a host-drainable buffer, exactly like the UART output channel. Disabled
//! ports consume writes silently, and stimulus reads report ready: the
//! channel never overruns, so firmware polling `STIM & 1` always proceeds.

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;

/// Stimulus ports 0..=31, four bytes each.
const STIM_BASE: u32 = 0xE000_0000;
const STIM_END: u32 = 0xE000_007C;
/// Trace-enable register: one bit per stimulus port.
const TER_BASE: u32 = 0xE000_0E00;
/// Privilege register: latched, currently advisory.
const TPR_BASE: u32 = 0xE000_0E40;
/// Control register: bit 0 is `ITMENA`.
const TCR_BASE: u32 = 0xE000_0E80;
/// Identification registers: read-as-zero probe window.
const ID_BASE: u32 = 0xE000_0FD0;
const ID_LEN: u32 = 0x30;
const ITMENA: u32 = 1;

/// ITM stimulus sink with host-drainable output.
#[derive(Debug, Default)]
pub struct Itm {
    ter: u32,
    tcr: u32,
    output: Vec<u8>,
}

impl Itm {
    /// A reset ITM: unit off, no ports enabled, no output.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ter: 0,
            tcr: 0,
            output: Vec::new(),
        }
    }

    fn port_enabled(&self, port: u32) -> bool {
        self.tcr & ITMENA != 0 && port < 32 && self.ter & (1u32 << port) != 0
    }

    /// Merge one byte lane into a latched word register.
    fn merge(reg: &mut u32, addr: u32, val: u8) {
        let shift = 8 * (addr & 3);
        *reg = (*reg & !(0xFF << shift)) | (u32::from(val) << shift);
    }
}

impl Peripheral for Itm {
    fn ranges(&self) -> &[Region] {
        const RANGES: [Region; 5] = [
            Region::new(STIM_BASE, 0x80),
            Region::new(TER_BASE, 4),
            Region::new(TPR_BASE, 4),
            Region::new(TCR_BASE, 4),
            Region::new(ID_BASE, ID_LEN),
        ];
        &RANGES
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let word = match addr & !3 {
            STIM_BASE..=STIM_END => 1,
            TER_BASE => self.ter,
            TCR_BASE => self.tcr,
            TPR_BASE | ID_BASE..=0xE000_0FFC => 0,
            _ => return Err(BusError::Reserved),
        };
        Ok((word >> (8 * (addr & 3))) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        match addr & !3 {
            STIM_BASE..=STIM_END => {
                if self.port_enabled((addr - STIM_BASE) >> 2) {
                    self.output.push(val);
                }
            }
            TER_BASE => Self::merge(&mut self.ter, addr, val),
            TCR_BASE => Self::merge(&mut self.tcr, addr, val),
            TPR_BASE | ID_BASE..=0xE000_0FFC => {}
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.ter = 0;
        self.tcr = 0;
        self.output.clear();
    }

    fn name(&self) -> &'static str {
        "itm"
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.output)
    }
}

impl InterruptSource for Itm {}
