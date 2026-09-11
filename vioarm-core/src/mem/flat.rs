//! A simple flat memory map used for the first milestone: a fixed set of RAM
//! banks and flash regions, backed by byte vectors. Everything outside the
//! declared regions is `Reserved`.

use crate::error::BusError;
use crate::hooks::Hooks;
use crate::mem::bus::{Bus, DataTranslate, MpuAccess, PeripheralBus, Region};

/// A single owned bank of bytes.
#[derive(Debug)]
pub struct Bank {
    /// Covered region.
    pub region: Region,
    bytes: Vec<u8>,
}

/// A `Bus` that services a list of fixed banks. Reads are always allowed;
/// writes go to RAM banks and are ignored (consumed, no fault) in flash banks
/// for compatibility with the "flash write ignored" behavior of many parts.
///
#[derive(Debug)]
pub struct FlatMemory {
    banks: Vec<Bank>,
    hooks: Hooks,
    /// Bus-write generation for JIT content validation (see `write_epoch`).
    /// Banks are fixed at construction (never reallocated), so host
    /// pointers handed out by `translate` stay valid for the bus lifetime.
    write_epoch: u64,
}

impl FlatMemory {
    /// Flat memory over explicit banks.
    #[must_use]
    pub const fn of_banks(banks: Vec<Bank>) -> Self {
        Self {
            banks,
            hooks: Hooks::empty(),
            write_epoch: 0,
        }
    }

    /// Build a memory from base/length pairs. Useful for tests.
    #[must_use]
    pub fn with_layout(starts: &[u32], lens: &[u32]) -> Self {
        let bytes = starts
            .iter()
            .zip(lens.iter())
            .map(|(base, len)| Bank {
                region: Region::new(*base, *len),
                bytes: vec![0; *len as usize],
            })
            .collect();
        Self {
            banks: bytes,
            hooks: Hooks::default(),
            write_epoch: 0,
        }
    }

    /// Locate a bank containing `addr`.
    fn bank_for(&self, addr: u32) -> Result<&Bank, BusError> {
        self.banks
            .iter()
            .find(|b| b.region.contains(addr))
            .ok_or(BusError::Reserved)
    }

    fn bank_for_mut(&mut self, addr: u32) -> Result<&mut Bank, BusError> {
        let idx = self
            .banks
            .iter()
            .position(|b| b.region.contains(addr))
            .ok_or(BusError::Reserved)?;
        Ok(&mut self.banks[idx])
    }

    /// The set of regions this map covers, for diagnostics.
    pub fn regions(&self) -> impl Iterator<Item = &Region> {
        self.banks.iter().map(|b| &b.region)
    }
}

impl Bus for FlatMemory {
    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let b = self.bank_for(addr)?;
        let off = (addr - b.region.base) as usize;
        let value = b.bytes[off];
        if self.hooks.has_read() {
            self.hooks.fire_read(addr, value);
        }
        Ok(value)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let b = self.bank_for_mut(addr)?;
        let off = (addr - b.region.base) as usize;
        b.bytes[off] = val;
        self.write_epoch = self.write_epoch.wrapping_add(1);
        if self.hooks.has_write() {
            self.hooks.fire_write(addr, val);
        }
        Ok(())
    }

    fn write_epoch_ptr(&mut self) -> Option<*mut u64> {
        Some(&mut self.write_epoch as *mut u64)
    }

    fn translate(&mut self, addr: u32, _access: MpuAccess) -> DataTranslate {
        let Some(idx) = self.banks.iter().position(|b| b.region.contains(addr)) else {
            return DataTranslate::Indirect;
        };
        // Host address of the guest page start. Banks are fixed-size (see
        // the struct docs), so this pointer is stable for the bus
        // lifetime. A bank starting mid-page cannot serve its first page
        // (split across banks): refuse rather than misalign.
        let base = self.banks[idx].region.base;
        let page_start = addr & !0xFFF;
        if page_start < base {
            return DataTranslate::Indirect;
        }
        let page_off = page_start - base;
        let len = (self.banks[idx].bytes.len() as u32).saturating_sub(page_off);
        if len == 0 {
            return DataTranslate::Indirect;
        }
        DataTranslate::Direct {
            ptr: self.banks[idx]
                .bytes
                .as_mut_ptr()
                .wrapping_add(page_off as usize),
            len,
        }
    }
}

impl PeripheralBus for FlatMemory {
    fn next_exception(
        &mut self,
        _primask: bool,
        _faultmask: bool,
        _basepri: u8,
        _current_ipsr: u8,
    ) -> Option<u8> {
        None
    }

    fn acknowledge_exception(&mut self, _n: u8) {}

    fn write_epoch(&self) -> Option<u64> {
        Some(self.write_epoch)
    }

    fn config_epoch(&self) -> Option<u64> {
        // Plain RAM/ROM banks: no MPU, no attribution, no fetch-affecting
        // configuration exists, so the generation is constant.
        Some(0)
    }

    fn hooks(&self) -> Option<&Hooks> {
        Some(&self.hooks)
    }

    fn hooks_mut(&mut self) -> Option<&mut Hooks> {
        Some(&mut self.hooks)
    }
}
