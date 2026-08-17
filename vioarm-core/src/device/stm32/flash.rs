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
const KEYR: u16 = 0x04;
/// `SR` lives at register offset `0x0c`.
const SR: u16 = 0x0c;
/// `CR` lives at register offset `0x10`.
const CR: u16 = 0x10;
/// `AR` lives at register offset `0x14`.
const AR: u16 = 0x14;

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
    /// Latched `WRPRTERR`: a gated write was rejected.
    pub wrprterr: bool,
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
            wrprterr: false,
        }
    }

    /// Restore the post-reset state: locked, no mode bits, no latched status.
    pub fn reset(&mut self) {
        *self = Self::new(self.region, self.page_size);
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
                self.gate.lock().unwrap().unlocked = true;
                self.key_step = 0;
            }
            _ => self.key_step = 0,
        }
    }
}

impl Behavior for FlashBehavior {
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u16,
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
                let mut gate = self.gate.lock().unwrap();
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
                let mut gate = self.gate.lock().unwrap();
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
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u16, value: u32) -> u32 {
        let gate = self.gate.lock().unwrap();
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
        let gate = self.gate.lock().unwrap();
        let error = gate.wrprterr && cr & CR_ERRIE != 0;
        let done = gate.eop && cr & CR_EOPIE != 0;
        drop(gate);
        u32::from(error || done)
    }

    fn reset(&mut self) {
        self.gate.lock().unwrap().reset();
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
        let def = D::peripheral("FLASH").expect("generated tables contain FLASH");
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
const KEYR4: u16 = 0x04;
/// `SR` lives at register offset `0x0c`.
const SR4: u16 = 0x0c;
/// `CR` lives at register offset `0x10`.
const CR4: u16 = 0x10;

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
}

impl FlashF4Behavior {
    const fn new(gate: Arc<Mutex<FlashGate>>) -> Self {
        Self {
            gate,
            key_word: 0,
            key_step: 0,
        }
    }

    fn unlock_write(&mut self, word: u32) -> bool {
        match (self.key_step, word) {
            (0, KEY1) => {
                self.key_step = 1;
                false
            }
            (1, KEY2) => {
                self.gate.lock().unwrap().unlocked = true;
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
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u16,
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
                let mut gate = self.gate.lock().unwrap();
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
                let mut gate = self.gate.lock().unwrap();
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
            _ => view.merged(offset, val, shift),
        }
    }

    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, offset: u16, value: u32) -> u32 {
        let gate = self.gate.lock().unwrap();
        let synth = match offset {
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
        synth
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let cr = view.get(CR4);
        let gate = self.gate.lock().unwrap();
        let error = gate.wrprterr && cr & CR4_ERRIE != 0;
        let done = gate.eop && cr & CR4_EOPIE != 0;
        drop(gate);
        u32::from(error || done)
    }

    fn reset(&mut self) {
        self.gate.lock().unwrap().reset();
        self.key_word = 0;
        self.key_step = 0;
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
        assert!(!g.lock().unwrap().unlocked);
        assert!(!g.lock().unwrap().program);
    }

    #[test]
    fn unlock_sequence_clears_lock() {
        let g = gate();
        let mut f = device(g.clone());
        write_word(&mut f, BASE + KEYR, KEY1);
        assert_eq!(read_word(&mut f, BASE + CR), LOCK, "one key is not enough");
        write_word(&mut f, BASE + KEYR, KEY2);
        assert!(g.lock().unwrap().unlocked);
        assert_eq!(read_word(&mut f, BASE + CR), 0, "CR.LOCK reads back clear");
    }

    #[test]
    fn sector_erase_targets_snb_range() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, SER | SNB | STRT);
        assert_eq!(
            g.lock().unwrap().erase,
            Some(FlashErase::Range(0x0802_0000, 0x0002_0000)),
            "sector 5 spans 0x20000..=0x3FFFF"
        );
        assert!(g.lock().unwrap().eop);
        assert_eq!(read_word(&mut f, BASE + CR) & STRT, 0, "STRT self-clears");
    }

    #[test]
    fn mass_erase_requests_full_region() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, MER | STRT);
        assert_eq!(g.lock().unwrap().erase, Some(FlashErase::Mass));
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
        assert!(!g.lock().unwrap().eop);
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
        assert!(!g.lock().unwrap().unlocked);
        assert!(!g.lock().unwrap().program);
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
        assert!(g.lock().unwrap().unlocked);
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
        assert!(!g.lock().unwrap().unlocked);
    }

    #[test]
    fn cr_write_sets_program_mode_after_unlock() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, CR_PG);
        assert!(g.lock().unwrap().program);
        assert_eq!(read_word(&mut f, BASE + CR), CR_PG);
    }

    #[test]
    fn writing_lock_relocks() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + CR, 0x80);
        assert!(!g.lock().unwrap().unlocked);
        assert_eq!(read_word(&mut f, BASE + CR), 0x80);
        write_word(&mut f, BASE + CR, CR_PG);
        assert!(!g.lock().unwrap().program, "relocked CR ignores writes");
    }

    #[test]
    fn strt_requests_erase_and_self_clears() {
        let g = gate();
        let mut f = device(g.clone());
        unlock(&mut f);
        write_word(&mut f, BASE + AR, 0x0800_0100);
        write_word(&mut f, BASE + CR, CR_PER | CR_STRT);
        assert_eq!(
            g.lock().unwrap().erase,
            Some(FlashErase::Page(0x0800_0100)),
            "page erase targets AR"
        );
        assert!(g.lock().unwrap().eop);
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
        assert_eq!(g.lock().unwrap().erase, Some(FlashErase::Mass));
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
        assert!(!g.lock().unwrap().eop);
    }

    #[test]
    fn rejected_writes_latch_wrprterr() {
        let g = gate();
        let mut f = device(g.clone());
        g.lock().unwrap().wrprterr = true;
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
        assert!(!g.lock().unwrap().unlocked);
        assert!(!g.lock().unwrap().eop);
        assert_eq!(g.lock().unwrap().erase, None);
        assert_eq!(read_word(&mut f, BASE + CR), 0x80);
    }
}
