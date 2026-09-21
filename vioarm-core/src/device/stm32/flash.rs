//! The STM32 flash controllers (`FLASH`), driven by the SVD-generated register
//! tables: the F1 at `0x4002_2000` and the F4 at `0x4002_3C00`.
//!
//! The controller enforces the silicon write protocol on the main flash
//! region at `0x0800_0000`: the CPU can only program flash after unlocking the
//! controller with the KEYR key sequence and setting `CR.PG`, and page/mass
//! (F1) or sector/mass (F4) erases only run once `CR.STRT` is set. The actual
//! write gate lives on the memory side — a `SystemMemory` holds the shared
//! [`FlashGate`] and routes main-flash writes through it, while this device
//! owns the register window. Both halves see the same state through an
//! `Arc<Mutex<FlashGate>>`, the same sharing pattern the SCB and the core use
//! for the FP context.
//!
//! Status flags (`EOP`, `WRPRTERR`) are synthesized into `SR` from the gate on
//! read and cleared by the one-shot write-1-to-clear convention, so firmware
//! polling `SR` observes a program or erase that the memory gate completed.

use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// The first key of the KEYR unlock sequence.
pub const KEY1: u32 = 0x4567_0123;
/// The second key of the KEYR unlock sequence.
pub const KEY2: u32 = 0xCDEF_89AB;

/// `SR.EOP`: an operation (program or erase) finished.
const SR_EOP: u32 = 1 << 5;
/// `SR.WRPRTERR`: a write to a write-protected flash word was rejected.
const SR_WRPRTERR: u32 = 1 << 4;

/// `CR.PG`: programming mode is enabled (flash words accept program writes).
const CR_PG: u32 = 1 << 0;
/// `CR.PER`: page-erase mode.
const CR_PER: u32 = 1 << 1;
/// `CR.MER`: mass-erase mode.
const CR_MER: u32 = 1 << 2;
/// `CR.STRT`: start the selected erase.
const CR_STRT: u32 = 1 << 6;
/// `CR.LOCK`: the controller is locked against writes.
const CR_LOCK: u32 = 1 << 7;
/// `CR.ERRIE`: interrupt enable for error flags.
const CR_ERRIE: u32 = 1 << 10;
/// `CR.EOPIE`: interrupt enable for `EOP`.
const CR_EOPIE: u32 = 1 << 12;

/// `KEYR` lives at register offset `0x04`.
const KEYR: u32 = 0x04;
/// `SR` lives at register offset `0x0c`.
const SR: u32 = 0x0c;
/// `CR` lives at register offset `0x10`.
const CR: u32 = 0x10;
/// `AR` lives at register offset `0x14`.
const AR: u32 = 0x14;
/// `WRPR` (F1): the write-protection register.
const WRPR: u32 = 0x20;

/// A pending erase, executed by the memory gate on the next access to the
/// flash region so the controller's instant-completion model stays visible to
/// firmware polling `SR.EOP`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlashErase {
    /// Erase the `page_size`-byte page containing `addr`.
    Page(u32),
    /// Erase the whole main-flash region.
    Mass,
    /// Erase the `len`-byte range starting at `addr` (the F4 sector erase).
    Range(u32, u32),
}

/// Shared flash-controller state: the program/erase gate and the status bits
/// synthesized into `SR`.
///
/// The `FLASH` device sets it; the `SystemMemory` write gate reads it to
/// decide whether a main-flash write lands; and `SR` reads reflect it.
#[derive(Debug)]
#[allow(clippy::struct_excessive_bools)]
pub struct FlashGate {
    /// The programmable main-flash region (writes here are gated).
    pub region: Region,
    /// The page size in bytes for page erases.
    pub page_size: u32,
    /// Whether the KEYR unlock sequence has been completed.
    pub unlocked: bool,
    /// Whether `CR.PG` is set (programming mode enabled).
    pub program: bool,
    /// A pending erase not yet executed by the memory gate.
    pub erase: Option<FlashErase>,
    /// Latched `EOP`: a program or erase completed.
    pub eop: bool,
    /// Bank that latched `eop` (`0`/`1`; single-bank parts always `0`).
    pub eop_bank: u8,
    /// Latched `WRPRTERR`: a gated write was rejected.
    pub wrprterr: bool,
    /// Bank that latched `wrprterr` (`0`/`1`; single-bank parts always `0`).
    pub err_bank: u8,
    /// Dual-bank controller (H7): the upper half of `region` answers to
    /// `unlocked2`/`program2` instead of `unlocked`/`program`.
    pub dual_bank: bool,
    /// Bank-2 unlock / program enables (dual-bank only).
    pub unlocked2: bool,
    /// Bank-2 program enable (dual-bank only).
    pub program2: bool,
    /// The write-protection register (`WRPR`, F1): one bit per page, a
    /// cleared bit protects its page. All-ones (unprotected) at reset,
    /// matching default option bytes.
    pub wrpr: u32,
}

impl FlashGate {
    /// A locked gate guarding `region`, erasing `page_size`-byte pages.
    #[must_use]
    pub const fn new(region: Region, page_size: u32) -> Self {
        Self {
            region,
            page_size,
            unlocked: false,
            program: false,
            erase: None,
            eop: false,
            eop_bank: 0,
            wrprterr: false,
            err_bank: 0,
            dual_bank: false,
            unlocked2: false,
            program2: false,
            wrpr: u32::MAX,
        }
    }

    /// Dual-bank mode (H7): upper-half writes answer to bank-2 enables.
    #[must_use]
    pub const fn dual_bank(mut self) -> Self {
        self.dual_bank = true;
        self
    }

    /// Restore the post-reset state: locked, no mode bits, no latched status.
    pub fn reset(&mut self) {
        let dual = self.dual_bank;
        *self = Self::new(self.region, self.page_size);
        self.dual_bank = dual;
    }

    /// Which bank owns `addr` (`1` = upper half in dual-bank mode).
    #[must_use]
    pub fn bank_of(&self, addr: u32) -> u8 {
        u8::from(self.dual_bank && addr.wrapping_sub(self.region.base) >= self.region.len / 2)
    }
}

/// The flash controller's register side effects.
#[derive(Debug)]
struct FlashBehavior {
    gate: Arc<Mutex<FlashGate>>,
    /// The KEYR word being assembled from byte-lane writes.
    key_word: u32,
    /// Which key of the unlock sequence has been seen (`0`, `1`, or `2`).
    key_step: u8,
}

impl FlashBehavior {
    const fn new(gate: Arc<Mutex<FlashGate>>) -> Self {
        Self {
            gate,
            key_word: 0,
            key_step: 0,
        }
    }

    fn unlock_write(&mut self, word: u32) {
        match (self.key_step, word) {
            (0, KEY1) => self.key_step = 1,
            (1, KEY2) => {
                self.gate.lock().unwrap_or_else(|e| e.into_inner()).unlocked = true;
                self.key_step = 0;
            }
            _ => self.key_step = 0,
        }
    }
}

impl Behavior for FlashBehavior {
    // Unlock/program/erase complete synchronously; timing is latency math.
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
        match offset {
            KEYR => {
                // Reassemble the full 32-bit key word from the byte lanes; a
                // standard word store arrives as four lanes (0, 8, 16, 24).
                self.key_word = (self.key_word & !(0xFF << shift)) | (u32::from(val) << shift);
                if shift == 24 {
                    let word = self.key_word;
                    self.unlock_write(word);
                }
                view.merged(KEYR, val, shift)
            }
            SR => {
                // Write-1-to-clear on the synthesized status flags: the stored
                // word is the read value, and the one-shot bits drop from the
                // gate (a plain byte-lane merge would re-arm stale bits).
                let mut gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
                let mask = u32::from(val) << shift;
                if mask & SR_EOP != 0 {
                    gate.eop = false;
                }
                if mask & SR_WRPRTERR != 0 {
                    gate.wrprterr = false;
                }
                let stored = view.get(SR) & !mask;
                drop(gate);
                stored
            }
            CR => {
                let word = view.merged(CR, val, shift);
                let mut gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
                if !gate.unlocked {
                    // Locked controllers ignore `CR` writes entirely.
                    let stored = view.get(CR);
                    drop(gate);
                    return stored;
                }
                if word & CR_LOCK != 0 {
                    gate.unlocked = false;
                }
                gate.program = word & CR_PG != 0;
                if word & CR_STRT != 0 {
                    if word & CR_MER != 0 {
                        gate.eop = true;
                        gate.erase = Some(FlashErase::Mass);
                    } else if word & CR_PER != 0 {
                        gate.eop = true;
                        gate.erase = Some(FlashErase::Page(view.get(AR)));
                    }
                    // `STRT` self-clears on silicon.
                    let stored = word & !CR_STRT;
                    drop(gate);
                    stored
                } else {
                    drop(gate);
                    word
                }
            }
            WRPR => {
                // The option-loader model: WRPR is writable while the
                // controller is unlocked, ignored locked. A cleared bit
                // protects its page (enforced by the bus write gate).
                if self.gate.lock().unwrap_or_else(|e| e.into_inner()).unlocked {
                    view.merged(WRPR, val, shift)
                } else {
                    view.get(WRPR)
                }
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let synth = match offset {
            SR => {
                value
                    | if gate.eop { SR_EOP } else { 0 }
                    | if gate.wrprterr { SR_WRPRTERR } else { 0 }
            }
            CR => {
                // `LOCK` reflects the gate rather than the stored reset word,
                // and the self-clearing `STRT` never reads back set.
                (value & !(CR_STRT | CR_LOCK)) | if gate.unlocked { 0 } else { CR_LOCK }
            }
            _ => value,
        };
        drop(gate);
        synth
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let cr = view.get(CR);
        let gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let error = gate.wrprterr && cr & CR_ERRIE != 0;
        let done = gate.eop && cr & CR_EOPIE != 0;
        drop(gate);
        u32::from(error || done)
    }

    fn reset(&mut self) {
        self.gate.lock().unwrap_or_else(|e| e.into_inner()).reset();
        self.key_word = 0;
        self.key_step = 0;
    }
}

/// The `FLASH` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct FlashF1 {
    inner: RegisterFile<FlashBehavior>,
}

impl FlashF1 {
    /// A new `FLASH` controller at the STM32F1 base address sharing `gate`.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `FLASH`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new(gate: Arc<Mutex<FlashGate>>) -> Self {
        Self::for_device::<stm32f103::Stm32f103>(gate)
    }

    /// A new `FLASH` controller on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `FLASH` peripheral.
    #[must_use]
    pub fn for_device<D: Device>(gate: Arc<Mutex<FlashGate>>) -> Self {
        // F0 tables name it `Flash` (F1/F4 use `FLASH`); same controller.
        let def = D::peripheral("FLASH")
            .or_else(|| D::peripheral("Flash"))
            .expect("generated tables contain FLASH");
        Self {
            inner: RegisterFile::new(def, FlashBehavior::new(gate)),
        }
    }
}

impl Peripheral for FlashF1 {
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
        "flash"
    }
}

impl InterruptSource for FlashF1 {
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

/// `SR.EOP` on the F4 controller (bit 0, not bit 5 as on the F1).
const SR4_EOP: u32 = 1 << 0;
/// `SR.WRPERR` on the F4 controller.
const SR4_WRPERR: u32 = 1 << 4;

/// `CR.PG`: programming mode enabled.
const CR4_PG: u32 = 1 << 0;
/// `CR.SER`: sector-erase mode.
const CR4_SER: u32 = 1 << 1;
/// `CR.MER`: mass-erase mode.
const CR4_MER: u32 = 1 << 2;
/// `CR.SNB`: the selected sector, bits 3..=6.
const CR4_SNB_SHIFT: u32 = 3;
const CR4_SNB_MASK: u32 = 0xF;
/// `CR.STRT`: start the selected erase.
const CR4_STRT: u32 = 1 << 16;
/// `CR.EOPIE`: interrupt enable for `EOP`.
const CR4_EOPIE: u32 = 1 << 24;
/// `CR.ERRIE`: interrupt enable for error flags.
const CR4_ERRIE: u32 = 1 << 25;
/// `CR.LOCK`: the controller is locked against writes.
const CR4_LOCK: u32 = 1 << 31;

/// `KEYR` lives at register offset `0x04`.
const KEYR4: u32 = 0x04;
/// `SR` lives at register offset `0x0c`.
const SR4: u32 = 0x0c;
/// `CR` lives at register offset `0x10`.
const CR4: u32 = 0x10;

/// The STM32F407 sector-start offsets within the main-flash region, relative
/// to its base; sector `i` spans `[sectors[i], sectors[i + 1])` — four 16 KiB
/// sectors, one 64 KiB, then three 128 KiB for the 1 MiB bank.
const F407_SECTORS: [u32; 9] = [
    0x00000, 0x04000, 0x08000, 0x0C000, 0x10000, 0x20000, 0x40000, 0x60000, 0x80000,
];

/// The F4 flash controller's register side effects: `KEYR` unlock, the
/// `CR`-driven program/sector/mass-erase protocol against the shared gate, and
/// `SR` synthesis. The register layout differs from the F1 controller — `ACR`
/// replaces the alias window, `SER`+`SNB` select a sector for erase, and `EOP`
/// lives at `SR` bit 0 — but the memory gate is the same [`FlashGate`].
#[derive(Debug)]
struct FlashF4Behavior {
    gate: Arc<Mutex<FlashGate>>,
    /// The KEYR word being assembled from byte-lane writes.
    key_word: u32,
    /// Which key of the unlock sequence has been seen (`0`, `1`, or `2`).
    key_step: u8,
    /// H7 `OPTSR_PRG` latched value and `SWAP_BANK` state.
    optsr_prg: u32,
    optsr_cur_swap: bool,
    opt_change_err: bool,
}

impl FlashF4Behavior {
    fn new(gate: Arc<Mutex<FlashGate>>) -> Self {
        Self {
            gate,
            key_word: 0,
            key_step: 0,
            optsr_prg: 0,
            optsr_cur_swap: false,
            opt_change_err: false,
        }
    }

    fn unlock_write(&mut self, word: u32) -> bool {
        match (self.key_step, word) {
            (0, KEY1) => {
                self.key_step = 1;
                false
            }
            (1, KEY2) => {
                self.gate.lock().unwrap_or_else(|e| e.into_inner()).unlocked = true;
                self.key_step = 0;
                true
            }
            _ => {
                self.key_step = 0;
                false
            }
        }
    }
}

impl Behavior for FlashF4Behavior {
    // Unlock/program/erase complete synchronously; timing is latency math.
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
        match offset {
            KEYR4 => {
                // Reassemble the full 32-bit key word from the byte lanes.
                self.key_word = (self.key_word & !(0xFF << shift)) | (u32::from(val) << shift);
                if shift == 24 {
                    let word = self.key_word;
                    if self.unlock_write(word) {
                        // `CR.LOCK` is bit 31, a different byte lane from the
                        // mode bits, so a later byte write would merge the stale
                        // stored bit back and re-lock the gate; drop it now.
                        view.set(CR4, view.get(CR4) & !CR4_LOCK);
                    }
                }
                view.merged(KEYR4, val, shift)
            }
            SR4 => {
                // Write-1-to-clear on the synthesized status flags.
                let mut gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
                let mask = u32::from(val) << shift;
                if mask & SR4_EOP != 0 {
                    gate.eop = false;
                }
                if mask & SR4_WRPERR != 0 {
                    gate.wrprterr = false;
                }
                let stored = view.get(SR4) & !mask;
                drop(gate);
                stored
            }
            CR4 => {
                let word = view.merged(CR4, val, shift);
                let mut gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
                if !gate.unlocked {
                    // Locked controllers ignore `CR` writes entirely.
                    let stored = view.get(CR4);
                    drop(gate);
                    return stored;
                }
                if word & CR4_LOCK != 0 {
                    gate.unlocked = false;
                }
                gate.program = word & CR4_PG != 0;
                if word & CR4_STRT != 0 {
                    if word & CR4_MER != 0 {
                        gate.eop = true;
                        gate.erase = Some(FlashErase::Mass);
                    } else if word & CR4_SER != 0 {
                        let region = gate.region;
                        let snb = usize::from(((word >> CR4_SNB_SHIFT) & CR4_SNB_MASK) as u8);
                        let snb = snb.min(F407_SECTORS.len() - 2);
                        let base = region.base + F407_SECTORS[snb];
                        let len = F407_SECTORS[snb + 1] - F407_SECTORS[snb];
                        gate.eop = true;
                        gate.erase = Some(FlashErase::Range(base, len));
                    }
                    // `STRT` self-clears on silicon.
                    let stored = word & !CR4_STRT;
                    drop(gate);
                    stored
                } else {
                    drop(gate);
                    word
                }
            }
            WRPR => {
                // H7 `OPTSR_PRG@0x20` shares offset with F4 `WRPR`; detect H7 by
                // presence of `OPTSR_CUR@0x1C`.
                if view.try_get(0x1C).is_some() {
                    let word = view.merged(offset, val, shift);
                    self.optsr_prg = word;
                    self.optsr_cur_swap = word & (1 << 31) != 0;
                    self.opt_change_err = true;
                    word
                } else if self.gate.lock().unwrap_or_else(|e| e.into_inner()).unlocked {
                    view.merged(WRPR, val, shift)
                } else {
                    view.get(WRPR)
                }
            }
            0x24 => {
                // H7 `OPTCCR@0x24` — W1C `CLR_OPTCHANGEERR@30`
                if view.try_get(0x1C).is_some() {
                    let word = view.merged(offset, val, shift);
                    if word & (1 << 30) != 0 {
                        self.opt_change_err = false;
                    }
                    view.get(offset)
                } else {
                    view.merged(offset, val, shift)
                }
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let mut synth = match offset {
            SR4 => {
                value
                    | if gate.eop { SR4_EOP } else { 0 }
                    | if gate.wrprterr { SR4_WRPERR } else { 0 }
            }
            CR4 => {
                // `LOCK` reflects the gate and `STRT` never reads back set.
                (value & !(CR4_STRT | CR4_LOCK)) | if gate.unlocked { 0 } else { CR4_LOCK }
            }
            _ => value,
        };
        drop(gate);
        // H7 `OPTSR_CUR@0x1C` — SWAP_BANK@31 + OPTCHANGEERR@30
        if offset == 0x1C && view.try_get(0x1C).is_some() {
            if self.optsr_cur_swap {
                synth |= 1 << 31;
            }
            if self.opt_change_err {
                synth |= 1 << 30;
            }
        }
        if offset == 0x20 && view.try_get(0x1C).is_some() {
            synth = self.optsr_prg;
        }
        synth
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let cr = view.get(CR4);
        let gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let error = gate.wrprterr && cr & CR4_ERRIE != 0;
        let done = gate.eop && cr & CR4_EOPIE != 0;
        drop(gate);
        u32::from(error || done)
    }

    fn reset(&mut self) {
        self.gate.lock().unwrap_or_else(|e| e.into_inner()).reset();
        self.key_word = 0;
        self.key_step = 0;
        self.optsr_prg = 0;
        self.optsr_cur_swap = false;
        self.opt_change_err = false;
    }
}

/// The F4 `FLASH` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct FlashF4 {
    inner: RegisterFile<FlashF4Behavior>,
}

impl FlashF4 {
    /// A new `FLASH` controller on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `FLASH` peripheral.
    #[must_use]
    pub fn for_device<D: Device>(gate: Arc<Mutex<FlashGate>>) -> Self {
        let def = D::peripheral("FLASH").expect("generated tables contain FLASH");
        Self {
            inner: RegisterFile::new(def, FlashF4Behavior::new(gate)),
        }
    }
}

impl Peripheral for FlashF4 {
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
        "flash"
    }
}

impl InterruptSource for FlashF4 {
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

/// H7 `KEYR1` lives at register offset `0x04` (`KEYR2` at `0x104`).
const H7_KEYR1: u32 = 0x04;
/// H7 bank-1 control/status/clear at `0x0C`/`0x10`/`0x14` (bank 2 +`0x100`).
const H7_CR1: u32 = 0x0C;
const H7_SR1: u32 = 0x10;
const H7_CCR1: u32 = 0x14;
const H7_CR2: u32 = 0x10C;
const H7_SR2: u32 = 0x110;
const H7_CCR2: u32 = 0x114;
/// H7 `CR` bits: `LOCK` 0, `PG` 1, `SER` 2, `BER` 3, `START` 7, `SNB` 8:3,
/// `EOPIE` 16, `WRPERRIE` 17.
const H7_LOCK: u32 = 1 << 0;
const H7_PG: u32 = 1 << 1;
const H7_SER: u32 = 1 << 2;
const H7_BER: u32 = 1 << 3;
const H7_START: u32 = 1 << 7;
const H7_EOPIE: u32 = 1 << 16;
const H7_WRPERRIE: u32 = 1 << 17;
/// H7 `SR` bits: `EOP` 16, `WRPERR` 17 (`BSY` reads 0: instant completion).
const H7_EOP: u32 = 1 << 16;
const H7_WRPERR: u32 = 1 << 17;

/// The H7 flash controller's register side effects (dual bank).
#[derive(Debug)]
struct FlashH7Behavior {
    gate: Arc<Mutex<FlashGate>>,
    /// The `KEYR1` word being assembled from byte-lane writes.
    key_word: u32,
    /// Which key of the unlock sequence has been seen (`0`, `1`, or `2`).
    key_step: u8,
}

impl FlashH7Behavior {
    const fn new(gate: Arc<Mutex<FlashGate>>) -> Self {
        Self {
            gate,
            key_word: 0,
            key_step: 0,
        }
    }

    fn unlock_write(&mut self, word: u32) {
        match (self.key_step, word) {
            (0, KEY1) => self.key_step = 1,
            (1, KEY2) => {
                self.gate.lock().unwrap_or_else(|e| e.into_inner()).unlocked = true;
                self.key_step = 0;
            }
            _ => self.key_step = 0,
        }
    }

    /// Program/erase control for bank `bank` (`0`/`1`): returns the word to
    /// store. Locked banks ignore `CR` writes entirely (bank 2 is always
    /// locked: the table marks `KEYR2` read-only so its unlock sequence
    /// never reaches the model — documented).
    fn cr_write(&mut self, view: &mut Registers<&mut [u32]>, bank: u8, word: u32) -> u32 {
        let mut gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let unlocked = if bank == 0 {
            gate.unlocked
        } else {
            gate.unlocked2
        };
        if !unlocked {
            drop(gate);
            return if bank == 0 {
                view.get(H7_CR1)
            } else {
                view.get(H7_CR2)
            };
        }
        if word & H7_LOCK != 0 {
            if bank == 0 {
                gate.unlocked = false;
            } else {
                gate.unlocked2 = false;
            }
        }
        if bank == 0 {
            gate.program = word & H7_PG != 0;
        } else {
            gate.program2 = word & H7_PG != 0;
        }
        if word & H7_START != 0 {
            // Sectors are `region/16` (`H743` 2 MiB: 8×128 KiB per bank).
            let sector = gate.region.len / 16;
            let bank_base = gate.region.base + u32::from(bank) * gate.region.len / 2;
            if word & H7_BER != 0 {
                gate.eop = true;
                gate.eop_bank = bank;
                gate.erase = Some(FlashErase::Range(bank_base, gate.region.len / 2));
            } else if word & H7_SER != 0 {
                let snb = (word >> 8) & 0x7;
                gate.eop = true;
                gate.eop_bank = bank;
                gate.erase = Some(FlashErase::Range(bank_base + snb * sector, sector));
            }
            // `START` self-clears on silicon.
            let stored = word & !H7_START;
            drop(gate);
            stored
        } else {
            drop(gate);
            word
        }
    }

    /// Clear-on-write status for bank `bank`: `CCR` clears the gate's
    /// latched flags when they belong to this bank; the register reads 0.
    fn ccr_write(&mut self, bank: u8, mask: u32) {
        let mut gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        if mask & H7_EOP != 0 && gate.eop_bank == bank {
            gate.eop = false;
        }
        if mask & H7_WRPERR != 0 && gate.err_bank == bank {
            gate.wrprterr = false;
        }
    }
}

impl Behavior for FlashH7Behavior {
    // Unlock/program/erase complete synchronously; timing is latency math.
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
        match offset {
            H7_KEYR1 => {
                // Reassemble the full 32-bit key word from the byte lanes.
                self.key_word = (self.key_word & !(0xFF << shift)) | (u32::from(val) << shift);
                if shift == 24 {
                    let word = self.key_word;
                    self.unlock_write(word);
                }
                view.merged(H7_KEYR1, val, shift)
            }
            H7_CR1 => {
                // `SNB` (byte 1) arrives after `START` (byte 0) in a word
                // store, so side effects apply on word completion (lane 3)
                // when the full control word is assembled; HAL control
                // writes are word-wide.
                let word = view.merged(H7_CR1, val, shift);
                if shift == 24 {
                    self.cr_write(view, 0, word)
                } else {
                    word
                }
            }
            H7_CR2 => {
                let word = view.merged(H7_CR2, val, shift);
                if shift == 24 {
                    self.cr_write(view, 1, word)
                } else {
                    word
                }
            }
            H7_SR1 | H7_SR2 => {
                // Status clears through `CCR`; `SR` writes are swallowed.
                view.get(offset)
            }
            H7_CCR1 => {
                let mask = u32::from(val) << shift;
                self.ccr_write(0, mask);
                0
            }
            H7_CCR2 => {
                let mask = u32::from(val) << shift;
                self.ccr_write(1, mask);
                0
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let synth = match offset {
            H7_SR1 | H7_SR2 => {
                let bank = u8::from(offset != H7_SR1);
                value
                    | if gate.eop && gate.eop_bank == bank {
                        H7_EOP
                    } else {
                        0
                    }
                    | if gate.wrprterr && gate.err_bank == bank {
                        H7_WRPERR
                    } else {
                        0
                    }
            }
            H7_CR1 | H7_CR2 => {
                // `LOCK` reflects the gate and `START` never reads back set.
                let bank = u8::from(offset != H7_CR1);
                let unlocked = if bank == 0 {
                    gate.unlocked
                } else {
                    gate.unlocked2
                };
                (value & !(H7_START | H7_LOCK)) | if unlocked { 0 } else { H7_LOCK }
            }
            _ => value,
        };
        drop(gate);
        synth
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let bank_pending = |cr: u32, bank: u8| {
            let error = gate.wrprterr && gate.err_bank == bank && cr & H7_WRPERRIE != 0;
            let done = gate.eop && gate.eop_bank == bank && cr & H7_EOPIE != 0;
            error || done
        };
        let hit = bank_pending(view.get(H7_CR1), 0) | bank_pending(view.get(H7_CR2), 1);
        drop(gate);
        u32::from(hit)
    }

    fn reset(&mut self) {
        self.gate.lock().unwrap_or_else(|e| e.into_inner()).reset();
        self.key_word = 0;
        self.key_step = 0;
    }
}

/// The H7 `Flash` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct FlashH7 {
    inner: RegisterFile<FlashH7Behavior>,
}

impl FlashH7 {
    /// A new H7 `Flash` controller on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `Flash`
    /// peripheral.
    #[must_use]
    pub fn for_device<D: Device>(gate: Arc<Mutex<FlashGate>>) -> Self {
        let def = D::peripheral("Flash")
            .or_else(|| D::peripheral("FLASH"))
            .expect("generated tables contain Flash");
        Self {
            inner: RegisterFile::new(def, FlashH7Behavior::new(gate)),
        }
    }
}

impl Peripheral for FlashH7 {
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
        "flash"
    }
}

impl InterruptSource for FlashH7 {
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

/// L4 `KEYR` at `0x08`, `SR` at `0x10`, `CR` at `0x14`.
const L4_KEYR: u32 = 0x08;
const L4_SR: u32 = 0x10;
const L4_CR: u32 = 0x14;
/// L4 `CR` bits: `PG` 0, `PER` 1, `MER1` 2, `PNB` 3:8, `BKER` 11,
/// `MER2` 15, `START` 16, `EOPIE` 24, `ERRIE` 25, `LOCK` 31.
const L4_PG: u32 = 1 << 0;
const L4_PER: u32 = 1 << 1;
const L4_MER1: u32 = 1 << 2;
const L4_MER2: u32 = 1 << 15;
const L4_START: u32 = 1 << 16;
const L4_EOPIE: u32 = 1 << 24;
const L4_ERRIE: u32 = 1 << 25;
const L4_LOCK: u32 = 1 << 31;
/// L4 `SR` bits: `EOP` 0, `WRPERR` 4 (`BSY` reads 0: instant completion).
const L4_EOP: u32 = 1 << 0;
const L4_WRPERR: u32 = 1 << 4;

/// The L4 flash controller's register side effects (single `SR`/`CR` pair
/// over two 256-page banks; pages are `gate.page_size`).
#[derive(Debug)]
struct FlashL4Behavior {
    gate: Arc<Mutex<FlashGate>>,
    /// The `KEYR` word being assembled from byte-lane writes.
    key_word: u32,
    /// Which key of the unlock sequence has been seen (`0`, `1`, or `2`).
    key_step: u8,
}

impl FlashL4Behavior {
    const fn new(gate: Arc<Mutex<FlashGate>>) -> Self {
        Self {
            gate,
            key_word: 0,
            key_step: 0,
        }
    }

    fn unlock_write(&mut self, word: u32) {
        match (self.key_step, word) {
            (0, KEY1) => self.key_step = 1,
            (1, KEY2) => {
                self.gate.lock().unwrap_or_else(|e| e.into_inner()).unlocked = true;
                self.key_step = 0;
            }
            _ => self.key_step = 0,
        }
    }
}

impl Behavior for FlashL4Behavior {
    // Unlock/program/erase complete synchronously; timing is latency math.
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
        match offset {
            L4_KEYR => {
                // Reassemble the full 32-bit key word from the byte lanes.
                self.key_word = (self.key_word & !(0xFF << shift)) | (u32::from(val) << shift);
                if shift == 24 {
                    let word = self.key_word;
                    self.unlock_write(word);
                }
                view.merged(L4_KEYR, val, shift)
            }
            L4_SR => {
                // Write-1-to-clear on the synthesized status flags.
                let mut gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
                let mask = u32::from(val) << shift;
                if mask & L4_EOP != 0 {
                    gate.eop = false;
                }
                if mask & L4_WRPERR != 0 {
                    gate.wrprterr = false;
                }
                let stored = view.get(L4_SR) & !mask;
                drop(gate);
                stored
            }
            L4_CR => {
                // `START` (byte 2) always lands after `PNB` (bytes 0-1) in
                // a word store, so per-lane evaluation is exact here.
                // `LOCK` (byte 3) is gate-owned: the stored reset word
                // carries it, so only an explicit lane-3 `LOCK=1` write
                // relocks — otherwise stale storage would re-lock on
                // every control write (reads synthesize `LOCK` anyway).
                let word =
                    view.merged(L4_CR, val, shift) & !L4_LOCK | (u32::from(val) << shift) & L4_LOCK;
                let mut gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
                if !gate.unlocked {
                    // Locked controllers ignore `CR` writes entirely.
                    let stored = view.get(L4_CR);
                    drop(gate);
                    return stored;
                }
                if word & L4_LOCK != 0 {
                    gate.unlocked = false;
                }
                gate.program = word & L4_PG != 0;
                if word & L4_START != 0 {
                    // 256 pages per bank; `BKER` picks bank 2 (`L476` 1 MiB).
                    let bank = u8::from(word & (1 << 11) != 0);
                    let bank_base = gate.region.base + u32::from(bank) * gate.region.len / 2;
                    if word & (L4_MER1 | L4_MER2) != 0 {
                        gate.eop = true;
                        gate.eop_bank = bank;
                        gate.erase = Some(FlashErase::Range(bank_base, gate.region.len / 2));
                    } else if word & L4_PER != 0 {
                        let pnb = (word >> 3) & 0xFF;
                        gate.eop = true;
                        gate.eop_bank = bank;
                        gate.erase = Some(FlashErase::Page(bank_base + pnb * gate.page_size));
                    }
                    // `START` self-clears on silicon.
                    let stored = word & !L4_START;
                    drop(gate);
                    stored
                } else {
                    drop(gate);
                    word
                }
            }
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        let gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let synth = match offset {
            L4_SR => {
                value
                    | if gate.eop { L4_EOP } else { 0 }
                    | if gate.wrprterr { L4_WRPERR } else { 0 }
            }
            L4_CR => {
                // `LOCK` reflects the gate and `START` never reads back set.
                (value & !(L4_START | L4_LOCK)) | if gate.unlocked { 0 } else { L4_LOCK }
            }
            _ => value,
        };
        drop(gate);
        synth
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let cr = view.get(L4_CR);
        let gate = self.gate.lock().unwrap_or_else(|e| e.into_inner());
        let error = gate.wrprterr && cr & L4_ERRIE != 0;
        let done = gate.eop && cr & L4_EOPIE != 0;
        drop(gate);
        u32::from(error || done)
    }

    fn reset(&mut self) {
        self.gate.lock().unwrap_or_else(|e| e.into_inner()).reset();
        self.key_word = 0;
        self.key_step = 0;
    }
}

/// The L4 `FLASH` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct FlashL4 {
    inner: RegisterFile<FlashL4Behavior>,
}

impl FlashL4 {
    /// A new L4 `FLASH` controller on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `FLASH`
    /// peripheral.
    #[must_use]
    pub fn for_device<D: Device>(gate: Arc<Mutex<FlashGate>>) -> Self {
        let def = D::peripheral("FLASH").expect("generated tables contain FLASH");
        Self {
            inner: RegisterFile::new(def, FlashL4Behavior::new(gate)),
        }
    }
}

impl Peripheral for FlashL4 {
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
        "flash"
    }
}

impl InterruptSource for FlashL4 {
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
mod f4_tests {
    use super::{FlashErase, FlashF4, FlashGate, Region, KEY1, KEY2};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;
    use std::sync::{Arc, Mutex};

    const BASE: u32 = 0x4002_3C00;
    const KEYR: u32 = 0x04;
    const SR: u32 = 0x0c;
    const CR: u32 = 0x10;
    const EOP: u32 = 1 << 0;
    const PG: u32 = 1 << 0;
    const SER: u32 = 1 << 1;
    const MER: u32 = 1 << 2;
    const SNB: u32 = 5 << 3;
    const STRT: u32 = 1 << 16;
    const EOPIE: u32 = 1 << 24;
    const LOCK: u32 = 1 << 31;

    fn gate() -> Arc<Mutex<FlashGate>> {
        Arc::new(Mutex::new(FlashGate::new(
            Region::new(0x0800_0000, 0x10_0000),
            16 * 1024,
        )))
    }

    fn device(g: Arc<Mutex<FlashGate>>) -> FlashF4 {
        FlashF4::for_device::<crate::device::svd::stm32f407::Stm32f407>(g)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(f: &mut FlashF4, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            f.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(f: &mut FlashF4, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = f.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn unlock(f: &mut FlashF4) {
        write_word(f, BASE + KEYR, KEY1);
        write_word(f, BASE + KEYR, KEY2);
    }

    #[test]
    fn cr_starts_locked_and_rejects_writes() {
        let g = gate();
        let mut f = device(g.clone());
        assert_eq!(read_word(&mut f, BASE + CR), LOCK, "LOCK set at reset");
        write_word(&mut f, BASE + CR, PG);
        assert_eq!(read_word(&mut f, BASE + CR), LOCK, "locked writes ignored");
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).program);
    }

    #[test]
    fn unlock_sequence_clears_lock() {
        let g = gate();
        let mut f = device(g.clone());
        write_word(&mut f, BASE + KEYR, KEY1);
        assert_eq!(read_word(&mut f, BASE + CR), LOCK, "one key is not enough");
        write_word(&mut f, BASE + KEYR, KEY2);
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
        assert_eq!(read_word(&mut f, BASE + CR), 0, "CR.LOCK reads back clear");
    }

    #[test]
    fn sector_erase_targets_snb_range() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, SER | SNB | STRT);
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Range(0x0802_0000, 0x0002_0000)),
            "sector 5 spans 0x20000..=0x3FFFF"
        );
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).eop);
        assert_eq!(read_word(&mut f, BASE + CR) & STRT, 0, "STRT self-clears");
    }

    #[test]
    fn mass_erase_requests_full_region() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, MER | STRT);
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Mass)
        );
    }

    #[test]
    fn sr_synthesizes_and_w1c_clears_status() {
        let g = gate();
        let mut f = device(g.clone());
        assert_eq!(read_word(&mut f, BASE + SR) & EOP, 0, "SR reset clear");
        unlock(&mut f);
        write_word(&mut f, BASE + CR, MER | STRT);
        assert_eq!(read_word(&mut f, BASE + SR) & EOP, EOP, "EOP after erase");
        write_word(&mut f, BASE + SR, EOP);
        assert_eq!(read_word(&mut f, BASE + SR) & EOP, 0);
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).eop);
    }

    #[test]
    fn eop_interrupt_pends_when_enabled() {
        let g = gate();
        let mut f = device(g);
        assert_eq!(f.pending(), 0);
        unlock(&mut f);
        write_word(&mut f, BASE + CR, EOPIE | MER | STRT);
        assert_eq!(f.pending(), 1, "EOPIE raises the FLASH line");
        write_word(&mut f, BASE + SR, EOP);
        assert_eq!(f.pending(), 0);
    }

    #[test]
    fn acr_and_optcr_are_pure_latches() {
        let g = gate();
        let mut f = device(g);
        write_word(&mut f, BASE, 0x0000_0502); // ACR: LATENCY+PRFTEN
        assert_eq!(read_word(&mut f, BASE), 0x0000_0502);
        write_word(&mut f, BASE + 0x14, 0x0000_0014); // OPTCR reset
        assert_eq!(read_word(&mut f, BASE + 0x14), 0x0000_0014);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let g = gate();
        let mut f = device(g);
        assert!(matches!(f.read(BASE + 0x400), Err(BusError::Reserved)));
        assert!(matches!(f.write(BASE - 4, 0), Err(BusError::Reserved)));
    }
}

#[cfg(test)]
mod tests {
    use super::{FlashErase, FlashF1, FlashGate, Region, KEY1, KEY2};
    use crate::device::stm32::flash::{
        CR_EOPIE, CR_MER, CR_PER, CR_PG, CR_STRT, SR_EOP, SR_WRPRTERR,
    };
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;
    use std::sync::{Arc, Mutex};

    const BASE: u32 = 0x4002_2000;
    const KEYR: u32 = 0x04;
    const SR: u32 = 0x0c;
    const CR: u32 = 0x10;
    const AR: u32 = 0x14;
    /// `WRPR` (F1): the write-protection register.
    const WRPR: u32 = 0x20;

    fn gate() -> Arc<Mutex<FlashGate>> {
        Arc::new(Mutex::new(FlashGate::new(
            Region::new(0x0800_0000, 0x1_0000),
            1024,
        )))
    }

    fn device(g: Arc<Mutex<FlashGate>>) -> FlashF1 {
        FlashF1::new(g)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(f: &mut FlashF1, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            f.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(f: &mut FlashF1, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = f.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn unlock(f: &mut FlashF1) {
        write_word(f, BASE + KEYR, KEY1);
        write_word(f, BASE + KEYR, KEY2);
    }

    #[test]
    fn cr_starts_locked_and_rejects_writes() {
        let g = gate();
        let mut f = device(g.clone());
        assert_eq!(read_word(&mut f, BASE + CR), 0x80, "LOCK set at reset");
        write_word(&mut f, BASE + CR, CR_PG);
        assert_eq!(
            read_word(&mut f, BASE + CR),
            0x80,
            "locked CR writes are ignored"
        );
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).program);
    }

    #[test]
    fn unlock_sequence_clears_lock() {
        let g = gate();
        let mut f = device(g.clone());
        write_word(&mut f, BASE + KEYR, KEY1);
        assert_eq!(
            read_word(&mut f, BASE + CR),
            0x80,
            "a single key does not unlock"
        );
        write_word(&mut f, BASE + KEYR, KEY2);
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
        assert_eq!(
            read_word(&mut f, BASE + CR),
            0,
            "CR.LOCK reads back cleared once unlocked"
        );
    }

    #[test]
    fn wrong_keys_do_not_unlock() {
        let g = gate();
        let mut f = device(g.clone());
        write_word(&mut f, BASE + KEYR, KEY1);
        write_word(&mut f, BASE + KEYR, 0xDEAD_BEEF);
        write_word(&mut f, BASE + KEYR, KEY2);
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
    }

    #[test]
    fn cr_write_sets_program_mode_after_unlock() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, CR_PG);
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).program);
        assert_eq!(read_word(&mut f, BASE + CR), CR_PG);
    }

    #[test]
    fn writing_lock_relocks() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, 0x80);
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
        assert_eq!(read_word(&mut f, BASE + CR), 0x80);
        write_word(&mut f, BASE + CR, CR_PG);
        assert!(
            !g.lock().unwrap_or_else(|e| e.into_inner()).program,
            "relocked CR ignores writes"
        );
    }

    #[test]
    fn strt_requests_erase_and_self_clears() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + AR, 0x0800_0100);
        write_word(&mut f, BASE + CR, CR_PER | CR_STRT);
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Page(0x0800_0100)),
            "page erase targets AR"
        );
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).eop);
        assert_eq!(
            read_word(&mut f, BASE + CR) & CR_STRT,
            0,
            "STRT never reads back set"
        );
    }

    #[test]
    fn mass_erase_requests_full_region() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, CR_MER | CR_STRT);
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Mass)
        );
    }

    #[test]
    fn sr_synthesizes_and_w1c_clears_status() {
        let g = gate();
        let mut f = device(g.clone());
        assert_eq!(read_word(&mut f, BASE + SR), 0, "SR reset clear");
        unlock(&mut f);
        write_word(&mut f, BASE + CR, CR_MER | CR_STRT);
        assert_eq!(
            read_word(&mut f, BASE + SR) & SR_EOP,
            SR_EOP,
            "EOP synthesized after an operation"
        );
        // Write-1-to-clear: writing EOP drops it.
        write_word(&mut f, BASE + SR, SR_EOP);
        assert_eq!(read_word(&mut f, BASE + SR) & SR_EOP, 0);
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).eop);
    }

    #[test]
    fn rejected_writes_latch_wrprterr() {
        let g = gate();
        let mut f = device(g.clone());
        g.lock().unwrap_or_else(|e| e.into_inner()).wrprterr = true;
        assert_eq!(
            read_word(&mut f, BASE + SR) & SR_WRPRTERR,
            SR_WRPRTERR,
            "WRPRTERR reflected from the gate"
        );
        write_word(&mut f, BASE + SR, SR_WRPRTERR);
        assert_eq!(read_word(&mut f, BASE + SR) & SR_WRPRTERR, 0);
    }

    #[test]
    fn eop_interrupt_pends_when_enabled() {
        let g = gate();
        let mut f = device(g);
        assert_eq!(f.pending(), 0);
        unlock(&mut f);
        write_word(&mut f, BASE + CR, CR_EOPIE | CR_MER | CR_STRT);
        assert_eq!(f.pending(), 1, "EOPIE raises the FLASH line");
        write_word(&mut f, BASE + SR, SR_EOP);
        assert_eq!(f.pending(), 0, "clearing EOP drops the line");
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let g = gate();
        let mut f = device(g);
        assert!(matches!(f.read(BASE + 0x28), Err(BusError::Reserved)));
        assert!(matches!(f.write(BASE - 4, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn reset_relocks_and_clears_status() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, CR_MER | CR_STRT);
        f.reset();
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).eop);
        assert_eq!(g.lock().unwrap_or_else(|e| e.into_inner()).erase, None);
        assert_eq!(read_word(&mut f, BASE + CR), 0x80);
    }
}

#[cfg(test)]
mod h7_tests {
    use super::{FlashErase, FlashGate, FlashH7, Region, KEY1, KEY2};
    use crate::device::svd::stm32h743::Stm32h743;
    use crate::device::svd::Device;
    use crate::device::{InterruptSource, Peripheral};
    use std::sync::{Arc, Mutex};

    const BASE: u32 = 0x5200_2000;
    const KEYR1: u32 = 0x04;
    const CR1: u32 = 0x0C;
    const SR1: u32 = 0x10;
    const CCR1: u32 = 0x14;
    const CR2: u32 = 0x10C;
    const SR2: u32 = 0x110;
    const CCR2: u32 = 0x114;
    const FLASH_BASE: u32 = 0x0800_0000;
    const FLASH_LEN: u32 = 0x20_0000;

    fn gate() -> Arc<Mutex<FlashGate>> {
        Arc::new(Mutex::new(
            FlashGate::new(Region::new(FLASH_BASE, FLASH_LEN), 32 * 1024).dual_bank(),
        ))
    }

    fn device(g: Arc<Mutex<FlashGate>>) -> FlashH7 {
        FlashH7::for_device::<Stm32h743>(g)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(f: &mut FlashH7, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            f.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(f: &mut FlashH7, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = f.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn unlock(f: &mut FlashH7) {
        write_word(f, BASE + KEYR1, KEY1);
        write_word(f, BASE + KEYR1, KEY2);
    }

    #[test]
    fn table_matches_silicon_layout() {
        let def = Stm32h743::peripheral("Flash").unwrap();
        assert_eq!(def.base, BASE, "Flash base");
        for (name, offset) in [
            ("ACR", 0x00),
            ("KEYR1", 0x04),
            ("CR1", 0x0C),
            ("SR1", 0x10),
            ("CCR1", 0x14),
            ("CR2", 0x10C),
            ("SR2", 0x110),
            ("CCR2", 0x114),
        ] {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("Flash:{name} present"));
            assert_eq!(r.offset, offset, "Flash:{name} offset");
        }
    }

    #[test]
    fn locked_at_reset_ignores_cr() {
        let mut f = device(gate());
        f.reset();
        assert_ne!(read_word(&mut f, BASE + CR1) & 1, 0, "LOCK reads set");
        write_word(&mut f, BASE + CR1, 0x0002_0002); // PG + SER
        assert_ne!(read_word(&mut f, BASE + CR1) & 1, 0, "still locked");
    }

    #[test]
    fn unlock_and_program_mode() {
        let g = gate();
        let mut f = device(g.clone());
        f.reset();
        unlock(&mut f);
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
        assert_eq!(read_word(&mut f, BASE + CR1) & 1, 0, "LOCK clears");
        write_word(&mut f, BASE + CR1, 0x02); // PG
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).program);
        write_word(&mut f, BASE + CR1, 0x01); // LOCK
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
    }

    #[test]
    fn sector_erase_queues_range_and_eop() {
        let g = gate();
        let mut f = device(g.clone());
        f.reset();
        unlock(&mut f);
        // SER + SNB=2 + START (START self-clears).
        write_word(&mut f, BASE + CR1, (2 << 8) | (1 << 2) | (1 << 7));
        let sector = FLASH_LEN / 16;
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Range(FLASH_BASE + 2 * sector, sector))
        );
        assert_eq!(read_word(&mut f, BASE + CR1) & (1 << 7), 0, "START clears");
        assert_ne!(read_word(&mut f, BASE + SR1) & (1 << 16), 0, "EOP1");
        write_word(&mut f, BASE + CCR1, 1 << 16);
        assert_eq!(read_word(&mut f, BASE + SR1) & (1 << 16), 0, "CCR clears");
    }

    #[test]
    fn bank_erase_covers_half() {
        let g = gate();
        let mut f = device(g.clone());
        f.reset();
        unlock(&mut f);
        write_word(&mut f, BASE + CR1, (1 << 3) | (1 << 7)); // BER + START
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Range(FLASH_BASE, FLASH_LEN / 2))
        );
    }

    #[test]
    fn bank2_stays_locked() {
        let g = gate();
        let mut f = device(g.clone());
        f.reset();
        // KEYR2 is read-only in the tables: no unlock sequence can land.
        assert_ne!(read_word(&mut f, BASE + CR2) & 1, 0, "LOCK2 reads set");
        write_word(&mut f, BASE + CR2, 0x02); // PG ignored while locked
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).program2);
        // A locked START is ignored, not an error.
        write_word(&mut f, BASE + CR2, (1 << 2) | (1 << 7));
        assert_eq!(g.lock().unwrap_or_else(|e| e.into_inner()).erase, None);
    }

    #[test]
    fn irq_follows_eopie_per_bank() {
        let mut f = device(gate());
        f.reset();
        unlock(&mut f);
        write_word(
            &mut f,
            BASE + CR1,
            (1 << 16) | (2 << 8) | (1 << 2) | (1 << 7),
        );
        assert_eq!(f.pending(), 1, "EOP1 + EOPIE1 raises the line");
        write_word(&mut f, BASE + CCR1, 1 << 16);
        assert_eq!(f.pending(), 0, "cleared");
    }
}

#[cfg(test)]
mod l4_tests {
    use super::{FlashErase, FlashGate, FlashL4, Region, KEY1, KEY2};
    use crate::device::svd::stm32l476::Stm32l476;
    use crate::device::svd::Device;
    use crate::device::{InterruptSource, Peripheral};
    use std::sync::{Arc, Mutex};

    const BASE: u32 = 0x4002_2000;
    const KEYR: u32 = 0x08;
    const SR: u32 = 0x10;
    const CR: u32 = 0x14;
    const FLASH_BASE: u32 = 0x0800_0000;
    const FLASH_LEN: u32 = 0x10_0000;

    fn gate() -> Arc<Mutex<FlashGate>> {
        Arc::new(Mutex::new(FlashGate::new(
            Region::new(FLASH_BASE, FLASH_LEN),
            2048,
        )))
    }

    fn device(g: Arc<Mutex<FlashGate>>) -> FlashL4 {
        FlashL4::for_device::<Stm32l476>(g)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(f: &mut FlashL4, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            f.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(f: &mut FlashL4, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = f.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn unlock(f: &mut FlashL4) {
        write_word(f, BASE + KEYR, KEY1);
        write_word(f, BASE + KEYR, KEY2);
    }

    #[test]
    fn table_matches_silicon_layout() {
        let def = Stm32l476::peripheral("FLASH").unwrap();
        assert_eq!(def.base, BASE, "FLASH base");
        for (name, offset) in [
            ("ACR", 0x00),
            ("KEYR", 0x08),
            ("SR", 0x10),
            ("CR", 0x14),
            ("ECCR", 0x18),
            ("OPTR", 0x20),
        ] {
            let r = def
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("FLASH:{name} present"));
            assert_eq!(r.offset, offset, "FLASH:{name} offset");
        }
    }

    #[test]
    fn locked_at_reset_ignores_cr() {
        let mut f = device(gate());
        f.reset();
        assert_ne!(
            read_word(&mut f, BASE + CR) & (1 << 31),
            0,
            "LOCK reads set"
        );
        write_word(&mut f, BASE + CR, 0x01); // PG
        assert_ne!(read_word(&mut f, BASE + CR) & (1 << 31), 0, "still locked");
    }

    #[test]
    fn unlock_program_and_relock() {
        let g = gate();
        let mut f = device(g.clone());
        f.reset();
        unlock(&mut f);
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
        assert_eq!(read_word(&mut f, BASE + CR) & (1 << 31), 0, "LOCK clears");
        write_word(&mut f, BASE + CR, 0x01); // PG
        assert!(g.lock().unwrap_or_else(|e| e.into_inner()).program);
        write_word(&mut f, BASE + CR, 1 << 31); // LOCK
        assert!(!g.lock().unwrap_or_else(|e| e.into_inner()).unlocked);
    }

    #[test]
    fn page_erase_targets_pnb() {
        let g = gate();
        let mut f = device(g.clone());
        f.reset();
        unlock(&mut f);
        // PER + PNB=3 + START.
        write_word(&mut f, BASE + CR, (3 << 3) | (1 << 1) | (1 << 16));
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Page(FLASH_BASE + 3 * 2048))
        );
        assert_eq!(read_word(&mut f, BASE + CR) & (1 << 16), 0, "START clears");
        assert_ne!(read_word(&mut f, BASE + SR) & 1, 0, "EOP");
        write_word(&mut f, BASE + SR, 1);
        assert_eq!(read_word(&mut f, BASE + SR) & 1, 0, "W1C clears EOP");
    }

    #[test]
    fn bank2_page_selects_upper_half() {
        let g = gate();
        let mut f = device(g.clone());
        f.reset();
        unlock(&mut f);
        // PER + BKER + PNB=1 + START.
        write_word(
            &mut f,
            BASE + CR,
            (1 << 11) | (1 << 3) | (1 << 1) | (1 << 16),
        );
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Page(FLASH_BASE + FLASH_LEN / 2 + 2048))
        );
    }

    #[test]
    fn mass_erase_covers_bank1() {
        let g = gate();
        let mut f = device(g.clone());
        f.reset();
        unlock(&mut f);
        write_word(&mut f, BASE + CR, (1 << 2) | (1 << 16)); // MER1 + START
        assert_eq!(
            g.lock().unwrap_or_else(|e| e.into_inner()).erase,
            Some(FlashErase::Range(FLASH_BASE, FLASH_LEN / 2))
        );
    }

    #[test]
    fn irq_follows_eopie() {
        let mut f = device(gate());
        f.reset();
        unlock(&mut f);
        write_word(
            &mut f,
            BASE + CR,
            (1 << 24) | (3 << 3) | (1 << 1) | (1 << 16),
        );
        assert_eq!(f.pending(), 1, "EOP + EOPIE raises the line");
        write_word(&mut f, BASE + SR, 1);
        assert_eq!(f.pending(), 0, "cleared");
    }
}
