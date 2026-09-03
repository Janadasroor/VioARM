//! SVD-driven register tables and the table-driven `RegisterFile` runtime.
//!
//! [`vioarm-svd`] generates [`stm32f103`], [`stm32f107`], and [`stm32f407`]
//! from CMSIS SVD files; this module holds the descriptor types those tables
//! plug into, plus a generic [`RegisterFile`] peripheral that derives its whole
//! address map from a [`PeripheralDef`]. Anything the SVD cannot express —
//! read-to-clear flags, write-one-to-clear, timer overflow, set/reset splits —
//! is a hand-written [`Behavior`] hook.
//!
//! [`vioarm-svd`]: ../../../../../vioarm-svd/src/main.rs

use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use core::ops::Deref;

pub mod per_dname;
pub mod stm32c011;
pub mod stm32c031;
pub mod stm32f030;
pub mod stm32f030c6;
pub mod stm32f030cc;
pub mod stm32f031;
pub mod stm32f038;
pub mod stm32f042;
pub mod stm32f042k6;
pub mod stm32f042t6;
pub mod stm32f048;
pub mod stm32f051;
pub mod stm32f070;
pub mod stm32f071;
pub mod stm32f072;
pub mod stm32f072c8;
pub mod stm32f091;
pub mod stm32f091rc;
pub mod stm32f098;
pub mod stm32f0x0;
pub mod stm32f0x1;
pub mod stm32f0x2;
pub mod stm32f0x8;
pub mod stm32f0xx;
pub mod stm32f100;
pub mod stm32f100c8;
pub mod stm32f100rb;
pub mod stm32f101;
pub mod stm32f101c8;
pub mod stm32f101re;
pub mod stm32f102;
pub mod stm32f102cb;
pub mod stm32f102re;
pub mod stm32f103;
pub mod stm32f103c6;
pub mod stm32f103c8;
pub mod stm32f103rb;
pub mod stm32f103re;
pub mod stm32f103vd;
pub mod stm32f103x49;
pub mod stm32f103ze;
pub mod stm32f105;
pub mod stm32f105rb;
pub mod stm32f105re;
pub mod stm32f107;
pub mod stm32f107re;
pub mod stm32f107vc;
pub mod stm32f205;
pub mod stm32f205rb;
pub mod stm32f205re;
pub mod stm32f207;
pub mod stm32f207rb;
pub mod stm32f207ze;
pub mod stm32f215;
pub mod stm32f215re;
pub mod stm32f215ze;
pub mod stm32f217;
pub mod stm32f217ig;
pub mod stm32f301;
pub mod stm32f301k8;
pub mod stm32f302;
pub mod stm32f302cb;
pub mod stm32f303;
pub mod stm32f303cc;
pub mod stm32f303xe;
pub mod stm32f318;
pub mod stm32f328;
pub mod stm32f333;
pub mod stm32f334;
pub mod stm32f358;
pub mod stm32f373;
pub mod stm32f373cc;
pub mod stm32f378;
pub mod stm32f378cc;
pub mod stm32f398;
pub mod stm32f401;
pub mod stm32f401cc;
pub mod stm32f401ce;
pub mod stm32f401re;
pub mod stm32f405;
pub mod stm32f407;
pub mod stm32f410;
pub mod stm32f410c8;
pub mod stm32f411;
pub mod stm32f411re;
pub mod stm32f412;
pub mod stm32f412ce;
pub mod stm32f413;
pub mod stm32f413ch;
pub mod stm32f415;
pub mod stm32f417;
pub mod stm32f423;
pub mod stm32f423ih;
pub mod stm32f427;
pub mod stm32f427ig;
pub mod stm32f429;
pub mod stm32f429ig;
pub mod stm32f429zi;
pub mod stm32f437;
pub mod stm32f437ii;
pub mod stm32f439;
pub mod stm32f446;
pub mod stm32f446ze;
pub mod stm32f469;
pub mod stm32f469ae;
pub mod stm32f479;
pub mod stm32f722;
pub mod stm32f722ze;
pub mod stm32f723;
pub mod stm32f730;
pub mod stm32f732;
pub mod stm32f732vg;
pub mod stm32f745;
pub mod stm32f745vg;
pub mod stm32f746;
pub mod stm32f746be;
pub mod stm32f746ig;
pub mod stm32f746zg;
pub mod stm32f756;
pub mod stm32f756zg;
pub mod stm32f765;
pub mod stm32f765bi;
pub mod stm32f777;
pub mod stm32g030;
pub mod stm32g031;
pub mod stm32g041;
pub mod stm32g050;
pub mod stm32g051;
pub mod stm32g061;
pub mod stm32g070;
pub mod stm32g071;
pub mod stm32g081;
pub mod stm32g0b1;
pub mod stm32g0c1;
pub mod stm32g431;
pub mod stm32g431rb;
pub mod stm32g441;
pub mod stm32g473;
pub mod stm32g474xx;
pub mod stm32g491;
pub mod stm32h533;
pub mod stm32h562;
pub mod stm32h723;
pub mod stm32h730;
pub mod stm32h735;
pub mod stm32h742;
pub mod stm32h743;
pub mod stm32h743zi;
pub mod stm32h745;
pub mod stm32h750;
pub mod stm32h753;
pub mod stm32h755;
pub mod stm32h7a3;
pub mod stm32h7b0;
pub mod stm32h7b3;
pub mod stm32h503;
pub mod stm32l010;
pub mod stm32l011;
pub mod stm32l021;
pub mod stm32l031;
pub mod stm32l041;
pub mod stm32l051;
pub mod stm32l052;
pub mod stm32l053;
pub mod stm32l062;
pub mod stm32l071;
pub mod stm32l073;
pub mod stm32l100;
pub mod stm32l151;
pub mod stm32l152;
pub mod stm32l162;
pub mod stm32h563;
pub mod stm32l412;
pub mod stm32l431;
pub mod stm32l432;
pub mod stm32l433;
pub mod stm32l443;
pub mod stm32l452;
pub mod stm32l471;
pub mod stm32l476;
pub mod stm32l476rg;
pub mod stm32l552;
pub mod stm32l562;
pub mod stm32u535;
pub mod stm32u575;
pub mod stm32u585;
pub mod stm32u595;
pub mod stm32u5a5;
pub mod stm32u5f9;
pub mod stm32u5g9;
pub mod stm32wba52;
pub mod stm32wba55;
pub mod stm32wb55;
pub mod stm32wle4;
pub mod stm32wle5;
pub mod stm32wl33;
pub mod stm32wl55;

pub mod stm32mp157;
pub mod stm32f446re;
pub mod stm32f469ai;
pub mod stm32f722ie;
pub mod stm32f723ie;
pub mod stm32f732ie;
pub mod stm32f745ie;
pub mod stm32f767zi;
pub mod stm32g030f6;
pub mod stm32g031f8;
pub mod stm32g070rb;
pub mod stm32g071rb;
pub mod stm32g474re;
pub mod stm32h743ii;
pub mod stm32h753ii;
pub mod stm32l476je;
pub mod stm32f030k6;
pub mod stm32f038k6;
pub mod stm32f042g6;
pub mod stm32f777ai;
pub mod stm32f051g8;
pub mod stm32f072rb;
pub mod stm32f091cb;
pub mod stm32f217ve;
pub mod stm32f301c8;
pub mod stm32f302cc;
pub mod stm32f303re;
pub mod stm32f373cb;
pub mod stm32f378be;
pub mod stm32f401rb;
pub mod stm32f401ve;
pub mod stm32f410rb;
pub mod stm32f412zg;
pub mod stm32f413rg;
pub mod stm32f423ze;
pub mod stm32f427ai;
pub mod stm32f429be;
pub mod stm32f437ai;
pub mod stm32f723ze;
pub mod stm32f756ig;
pub mod stm32f765ig;
pub mod stm32f777vg;
pub mod stm32g030k6;
pub mod stm32g031k8;
pub mod stm32g070cb;
pub mod stm32g071cb;
pub mod stm32g474ce;
pub mod stm32h743xi;
pub mod stm32h753xi;
pub mod stm32f042c8;
pub mod stm32f042r8;
pub mod stm32f051r8;
pub mod stm32f072r8;
pub mod stm32f091rb;
pub mod stm32f100re;
pub mod stm32f101rb;
pub mod stm32f102rb;
pub mod stm32f103r8;
pub mod stm32f105rg;
pub mod stm32f107rg;
pub mod stm32f205rg;
pub mod stm32f207rg;
pub mod stm32f215rg;
pub mod stm32f217rg;
pub mod stm32f767ig;
pub mod stm32l476jg;
pub mod stm32f030r8;
pub mod stm32f030v6;
pub mod stm32f038r8;
pub mod stm32f038v6;
pub mod stm32f301r8;
pub mod stm32f302r8;
pub mod stm32f303rg;
pub mod stm32f373rg;
pub mod stm32f378rg;
pub mod stm32f401rg;
pub mod stm32f410re;
pub mod stm32f412re;
pub mod stm32f413re;
pub mod stm32f423rg;
pub mod stm32f427rg;
pub mod stm32f429rg;
pub mod stm32f437rg;
pub mod stm32f446rg;
pub mod stm32f469rg;
pub mod stm32f722rg;
pub mod stm32f723rg;
pub mod stm32f732rg;
pub mod stm32f745rg;
pub mod stm32f746rg;
pub mod stm32f756rg;
pub mod stm32f765rg;
pub mod stm32f767rg;
pub mod stm32f777rg;
pub mod stm32g030c6;
pub mod stm32g031c6;
pub mod stm32g070re;
pub mod stm32g071re;
pub mod stm32g474rg;
pub mod stm32h743xg;
pub mod stm32h753xg;
pub mod stm32f030t6;
pub mod stm32f038t6;
pub mod stm32f042k4;
pub mod stm32f051k4;
pub mod stm32f072k8;
pub mod stm32f091vb;
pub mod stm32f100rc;
pub mod stm32f101rc;
pub mod stm32f102rc;
pub mod stm32f103rc;
pub mod stm32f105rc;
pub mod stm32f107rc;
pub mod stm32f205rc;
pub mod stm32f207rc;
pub mod stm32f215rc;
pub mod stm32f217rc;
pub mod stm32f301k6;
pub mod stm32f302k6;
pub mod stm32f303k8;
pub mod stm32f373k8;
pub mod stm32f378k8;
pub mod stm32f401k6;
pub mod stm32f410k8;
pub mod stm32f412k6;
pub mod stm32f413k6;
pub mod stm32f423k6;
pub mod stm32f427k6;
pub mod stm32f429k6;
pub mod stm32f437k6;
pub mod stm32f446k6;
pub mod stm32f469k6;
pub mod stm32f722k6;
pub mod stm32f723k6;
pub mod stm32f732k6;
pub mod stm32f745k6;
pub mod stm32f746k6;
pub mod stm32f756k6;
pub mod stm32f765k6;
pub mod stm32f767k6;
pub mod stm32f777k6;
pub mod stm32g030g4;
pub mod stm32g031g4;
pub mod stm32g070g4;
pub mod stm32g071g4;
pub mod stm32g474kb;
pub mod stm32h743ag;
pub mod stm32h753ag;
pub mod stm32l476ke;
pub mod stm32c011f6;
pub mod stm32c031f6;
pub mod stm32f030a0;
pub mod stm32f030a1;
pub mod stm32f042a0;
pub mod stm32f042a1;
pub mod stm32f072a0;
pub mod stm32f072a1;
pub mod stm32f091a0;
pub mod stm32f091a1;
pub mod stm32f100a0;
pub mod stm32f100a1;
pub mod stm32f101a0;
pub mod stm32f101a1;
pub mod stm32f102a0;
pub mod stm32f102a1;
pub mod stm32f103a0;
pub mod stm32f103a1;
pub mod stm32f105a0;
pub mod stm32f105a1;
pub mod stm32f107a0;
pub mod stm32f107a1;
pub mod stm32f205a0;
pub mod stm32f205a1;
pub mod stm32f207a0;
pub mod stm32f207a1;
pub mod stm32f215a0;
pub mod stm32f215a1;
pub mod stm32f217a0;
pub mod stm32f217a1;
pub mod stm32f301a0;
pub mod stm32f301a1;
pub mod stm32f302a0;
pub mod stm32f302a1;
pub mod stm32f303a0;
pub mod stm32f303a1;
pub mod stm32f373a0;
pub mod stm32f373a1;
pub mod stm32f378a0;
pub mod stm32f378a1;
pub mod stm32f401a0;
pub mod stm32f401a1;
pub mod stm32f410a0;
pub mod stm32f410a1;
pub mod stm32f412a0;
pub mod stm32f412a1;
pub mod stm32f413a0;
pub mod stm32f413a1;
pub mod stm32f423a0;
pub mod stm32f423a1;
pub mod stm32f427a0;
pub mod stm32f427a1;
pub mod stm32f429a0;
pub mod stm32f429a1;
pub mod stm32f437a0;
pub mod stm32f437a1;
pub mod stm32f446a0;
pub mod stm32f446a1;
pub mod stm32f469a0;
pub mod stm32f469a1;
pub mod stm32f722a0;
pub mod stm32f722a1;
pub mod stm32f723a0;
pub mod stm32f723a1;
pub mod stm32f732a0;
pub mod stm32f745a0;
pub mod stm32f745a1;
pub mod stm32f746a0;
pub mod stm32f746a1;
pub mod stm32f756a0;
pub mod stm32f756a1;
pub mod stm32f765a0;
pub mod stm32f765a1;
pub mod stm32f767a0;
pub mod stm32f732a1;
pub mod stm32f767a1;
pub mod stm32f777a0;
pub mod stm32f777a1;
pub mod stm32g030a0;
pub mod stm32g030a1;
pub mod stm32g031a0;
pub mod stm32g031a1;
pub mod stm32g070a0;
pub mod stm32g070a1;
pub mod stm32g071a0;
pub mod stm32g071a1;
pub mod stm32g474a0;
pub mod stm32g474a1;
pub mod stm32h743a0;
pub mod stm32h743a1;
pub mod stm32h753a0;
pub mod stm32h753a1;
pub mod stm32l476a0;
pub mod stm32l476a1;
pub mod stm32c011a0;
pub mod stm32c011a1;
pub mod stm32c031a0;
pub mod stm32c031a1;
pub mod stm32u575a0;
pub mod stm32u575a1;
pub mod stm32wba55a0;
pub mod stm32wba55a1;
pub mod stm32f030b0;
pub mod stm32f042b0;
pub mod stm32f072b0;
pub mod stm32f091b0;
pub mod stm32f100b0;
pub mod stm32f101b0;
pub mod stm32f102b0;
pub mod stm32f103b0;
pub mod stm32f105b0;
pub mod stm32f107b0;
pub mod stm32f205b0;
pub mod stm32f207b0;
pub mod stm32f215b0;
pub mod stm32f217b0;
pub mod stm32f301b0;
pub mod stm32f302b0;
pub mod stm32f303b0;
pub mod stm32f373b0;
pub mod stm32f378b0;
pub mod stm32f401b0;
pub mod stm32f410b0;
pub mod stm32f412b0;
pub mod stm32f413b0;
pub mod stm32f423b0;
pub mod stm32f427b0;
pub mod stm32f429b0;
pub mod stm32f437b0;
pub mod stm32f446b0;
pub mod stm32f469b0;
pub mod stm32f722b0;
pub mod stm32f723b0;
pub mod stm32f732b0;
pub mod stm32f745b0;
pub mod stm32f746b0;
pub mod stm32f756b0;
pub mod stm32f765b0;
pub mod stm32f767b0;
pub mod stm32f777b0;
pub mod stm32g030b0;
pub mod stm32g031b0;
pub mod stm32g070b0;
pub mod stm32g071b0;
pub mod stm32g474b0;
pub mod stm32h743b0;
pub mod stm32h753b0;
pub mod stm32l476b0;
pub mod stm32c011b0;
pub mod stm32c031b0;
pub mod stm32u575b0;
pub mod stm32wba55b0;
pub mod stm32f030b1;
pub mod stm32f042b1;
pub mod stm32f072b1;
pub mod stm32f091b1;
pub mod stm32f100b1;
pub mod stm32f101b1;
pub mod stm32f767;
pub mod stm32h747;
pub mod stm32f769;
/// The register access vocabulary used by the supported SVD files.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    ReadWrite,
    ReadOnly,
    WriteOnly,
}

/// A single bit field within a register.
#[derive(Clone, Copy, Debug)]
pub struct FieldDef {
    pub name: &'static str,
    pub offset: u8,
    pub width: u8,
    pub access: Access,
}

/// A single register within a peripheral.
#[derive(Clone, Copy, Debug)]
pub struct RegisterDef {
    pub name: &'static str,
    pub offset: u32,
    pub width: u8,
    pub access: Access,
    pub reset: u32,
    pub fields: &'static [FieldDef],
}

/// An interrupt line a peripheral can raise.
#[derive(Clone, Copy, Debug)]
pub struct IrqDef {
    pub number: u8,
    pub name: &'static str,
}

/// One memory-mapped peripheral, exactly as the SVD describes it.
#[derive(Clone, Copy, Debug)]
pub struct PeripheralDef {
    pub name: &'static str,
    pub base: u32,
    pub window: u32,
    pub irqs: &'static [IrqDef],
    pub registers: &'static [RegisterDef],
}

/// A generated device table: one chip's peripheral map.
///
/// Each generated table defines a chip type — [`stm32f103::Stm32f103`],
/// [`stm32f107::Stm32f107`], [`stm32f407::Stm32f407`] — that implements this
/// so a [`Behavior`] can be mounted against any chip's tables unchanged.
pub trait Device {
    /// Find the generated descriptor for a peripheral by SVD name.
    fn peripheral(name: &str) -> Option<&'static PeripheralDef>;

    /// Find the generated descriptor for a peripheral by base address.
    fn by_base(base: u32) -> Option<&'static PeripheralDef>;
}

/// A window into the register file's storage, keyed by register offset.
///
/// A [`Behavior`] gets one of these on every access so it can read or write
/// registers *other* than the one the bus touched — GPIO's `BSRR`/`BRR` set
/// and reset `ODR`, and `IDR` is synthesized from `CRL`/`CRH`/`ODR` plus the
/// host-driven input level. The storage type `W` is generic over mutability:
/// reads borrow the words immutably, writes and ticks mutably.
#[derive(Debug)]
pub struct Registers<W> {
    def: &'static PeripheralDef,
    words: W,
}

impl<W> Registers<W>
where
    W: Deref<Target = [u32]>,
{
    pub(crate) const fn new(def: &'static PeripheralDef, words: W) -> Self {
        Self { def, words }
    }

    fn index(&self, offset: u32) -> usize {
        self.def
            .registers
            .iter()
            .position(|r| r.offset == offset)
            .unwrap_or_else(|| panic!("no register at offset 0x{offset:x}"))
    }

    /// The stored word at register `offset`.
    #[must_use]
    pub fn get(&self, offset: u32) -> u32 {
        self.words[self.index(offset)]
    }

    /// The stored word at `offset` if the register exists, else `None`.
    #[must_use]
    pub fn try_get(&self, offset: u32) -> Option<u32> {
        self.def
            .registers
            .iter()
            .position(|r| r.offset == offset)
            .map(|i| self.words[i])
    }
}

impl Registers<&mut [u32]> {
    /// Replace the stored word at register `offset`.
    pub fn set(&mut self, offset: u32, value: u32) {
        let i = self.index(offset);
        self.words[i] = value;
    }

    /// Replace the stored word if the register exists; return whether it did.
    pub fn try_set(&mut self, offset: u32, value: u32) -> bool {
        if let Some(i) = self.def.registers.iter().position(|r| r.offset == offset) {
            self.words[i] = value;
            true
        } else {
            false
        }
    }

    /// The stored word at `offset` with byte `val` merged into lane `shift` —
    /// the byte-lane read-modify-write a plain register write stores.
    #[must_use]
    pub fn merged(&self, offset: u32, val: u8, shift: u32) -> u32 {
        let current = self.get(offset);
        let mask = 0xFFu32 << shift;
        (current & !mask) | (u32::from(val) << shift)
    }
}

/// Per-register side effects the SVD cannot express.
///
/// A behavior transforms a register value on read or write, can reach the rest
/// of the register file through a [`Registers`] view, and can advance in
/// simulated time. The trait is `Send` so any behavior can be boxed onto the
/// bus.
pub trait Behavior: Send {
    /// The word to store after firmware writes byte `val` at lane `shift`.
    ///
    /// The default does a byte-lane read-modify-write, so a plain register
    /// needs no hook. The byte and its lane are passed rather than a merged
    /// word so one-shot semantics — `BSRR`/`BRR` set/reset, write-1-to-clear —
    /// never re-fire stale bits from an earlier access.
    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        view.merged(offset, val, shift)
    }

    /// Transform a register value before firmware reads it.
    ///
    /// The view is mutable because reads have side effects on silicon: USART
    /// reading `DR` acknowledges `RXNE`.
    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, _offset: u32, value: u32) -> u32 {
        value
    }

    /// Advance simulated time by `delta` cycles. The register file is mutable
    /// so a behavior can run hardware the tables only describe (timer counters,
    /// prescaler phases, analog settle delays).
    fn tick(&mut self, _view: &mut Registers<&mut [u32]>, _delta: u64) {}

    /// Interrupt lines latched pending right now.
    fn pending(&self, _view: &Registers<&[u32]>) -> u32 {
        0
    }

    /// Clear the latched `lines` the NVIC is taking.
    fn acknowledge(&mut self, _lines: u32) {}

    /// Which device-local lines the device can raise, as a bitmask of line
    /// indices. The default of a single line keeps one-line devices unchanged;
    /// a multi-channel controller overrides it.
    fn lines(&self) -> u32 {
        1
    }

    /// Restore post-reset behavior state, after the register words reload.
    fn reset(&mut self) {}

    /// Host stimulus: set the externally-visible input level to `value`.
    fn drive_inputs(&mut self, _view: &mut Registers<&mut [u32]>, _value: u32) {}

    /// Bytes firmware wrote out, for the host to drain (a captured TX line).
    fn drain_output(&mut self) -> Vec<u8> {
        Vec::new()
    }
}

/// A behavior with no side effects: every register is a pure latch.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoBehavior;

impl Behavior for NoBehavior {}

/// A byte-addressable peripheral driven entirely by a [`PeripheralDef`].
///
/// Register bytes are read and written with byte-lane read-modify-write, so a
/// Cortex-M word store lands in the right lanes; `reset()` reloads the SVD
/// reset values; and reads of addresses inside the window but between
/// registers are `Reserved`, like gaps in the silicon address map.
#[derive(Debug)]
pub struct RegisterFile<B: Behavior> {
    ranges: [Region; 1],
    def: &'static PeripheralDef,
    regs: Box<[u32]>,
    behavior: B,
}

impl<B: Behavior> RegisterFile<B> {
    /// A new register file at `def.base`, loaded with `def`'s reset values,
    /// running `behavior`.
    #[must_use]
    pub fn new(def: &'static PeripheralDef, behavior: B) -> Self {
        let regs = def.registers.iter().map(|r| r.reset).collect();
        Self {
            ranges: [Region::new(def.base, def.window)],
            def,
            regs,
            behavior,
        }
    }

    /// Mutable access to the inner behavior, for peripherals that keep
    /// filter or gate state outside the register file.
    #[must_use]
    pub fn behavior_mut(&mut self) -> &mut B {
        &mut self.behavior
    }

    /// Immutable access to the inner behavior.
    #[must_use]
    pub fn behavior(&self) -> &B {
        &self.behavior
    }

    /// Accessor for the peripheral def.
    #[must_use]
    pub fn def(&self) -> &'static PeripheralDef {
        self.def
    }

    /// Mutable access to the register words.
    pub fn regs_mut(&mut self) -> &mut [u32] {
        &mut self.regs[..]
    }

    /// Run a closure with a mutable view and behavior.
    pub fn with_view<R>(&mut self, f: impl FnOnce(&mut Registers<&mut [u32]>, &mut B) -> R) -> R {
        let mut view = Registers::new(self.def, &mut self.regs[..]);
        f(&mut view, &mut self.behavior)
    }
}

impl<B: Behavior> Peripheral for RegisterFile<B> {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let off = self.offset_in_window(addr)?;
        let (reg, _idx) = self.reg_at(off)?;
        let word = if reg.access == Access::WriteOnly {
            0
        } else {
            let mut view = Registers::new(self.def, &mut self.regs[..]);
            let value = view.get(reg.offset);
            self.behavior.on_read(&mut view, reg.offset, value)
        };
        let shift = Self::byte_shift(reg, off);
        #[allow(clippy::cast_possible_truncation)]
        Ok(((word >> shift) & 0xFF) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let off = self.offset_in_window(addr)?;
        let (reg, _idx) = self.reg_at(off)?;
        if reg.access == Access::ReadOnly {
            return Ok(());
        }
        let shift = Self::byte_shift(reg, off);
        let mut view = Registers::new(self.def, &mut self.regs[..]);
        let word = self.behavior.on_write(&mut view, reg.offset, val, shift);
        view.set(reg.offset, word);
        Ok(())
    }

    fn reset(&mut self) {
        for (idx, reg) in self.def.registers.iter().enumerate() {
            self.regs[idx] = reg.reset;
        }
        self.behavior.reset();
    }

    fn tick(&mut self, delta: u64) {
        let mut view = Registers::new(self.def, &mut self.regs[..]);
        self.behavior.tick(&mut view, delta);
    }

    fn name(&self) -> &'static str {
        self.def.name
    }

    fn drive_inputs(&mut self, value: u32) {
        let mut view = Registers::new(self.def, &mut self.regs[..]);
        self.behavior.drive_inputs(&mut view, value);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.behavior.drain_output()
    }
}

impl<B: Behavior> InterruptSource for RegisterFile<B> {
    fn exception_number(&self) -> Option<u8> {
        self.def.irqs.first().map(|irq| 16 + irq.number)
    }

    fn pending(&self) -> u32 {
        let view = Registers::new(self.def, &self.regs[..]);
        self.behavior.pending(&view)
    }

    fn acknowledge(&mut self, lines: u32) {
        self.behavior.acknowledge(lines);
    }

    fn lines(&self) -> u32 {
        self.behavior.lines()
    }

    fn exception_for_line(&self, line: u32) -> Option<u8> {
        // The table lists each IRQ a peripheral can raise; a controller whose
        // lines are not consecutive (the F4 DMA) gets the exact mapping, and
        // everything else keeps the consecutive default.
        self.def.irqs.get(line as usize).map_or_else(
            || {
                self.exception_number()
                    .map(|base| base.wrapping_add(u8::try_from(line).unwrap_or(u8::MAX)))
            },
            |irq| Some(16 + irq.number),
        )
    }
}

impl<B: Behavior> RegisterFile<B> {
    /// The raw stored word at register `offset`, for host inspection. Bypasses
    /// the behavior so it observes the true latch (say, the GPIO output
    /// register) rather than a synthesized read.
    ///
    /// # Panics
    /// Panics if `offset` is not one of the table's registers.
    #[must_use]
    pub fn word(&self, offset: u32) -> u32 {
        let idx = self
            .def
            .registers
            .iter()
            .position(|r| r.offset == offset)
            .unwrap_or_else(|| panic!("no register at offset 0x{offset:x}"));
        self.regs[idx]
    }

    /// The exception number for the named interrupt line, if the table lists
    /// it. Some SVDs group several lines under one peripheral (EXTI lists
    /// `TAMPER`, `EXTI0`, ...); this picks the one the behavior actually wires.
    #[must_use]
    pub fn irq_exception(&self, name: &str) -> Option<u8> {
        self.def
            .irqs
            .iter()
            .find(|irq| irq.name == name)
            .map(|irq| 16 + irq.number)
    }

    const fn offset_in_window(&self, addr: u32) -> Result<u32, BusError> {
        if self.ranges[0].contains(addr) {
            Ok(addr - self.ranges[0].base)
        } else {
            Err(BusError::Reserved)
        }
    }

    fn reg_at(&self, off: u32) -> Result<(&'static RegisterDef, usize), BusError> {
        let idx = self
            .def
            .registers
            .iter()
            .position(|r| {
                let len = u32::from(r.width) / 8;
                off >= r.offset && off < r.offset + len
            })
            .ok_or(BusError::Reserved)?;
        Ok((&self.def.registers[idx], idx))
    }

    fn byte_shift(reg: &RegisterDef, off: u32) -> u32 {
        8 * (off - reg.offset)
    }
}

#[cfg(test)]
mod tests {
    use super::{Access, Behavior, FieldDef, IrqDef, PeripheralDef, RegisterDef, RegisterFile};
    use crate::device::InterruptSource;
    use crate::device::Peripheral;
    use crate::error::BusError;

    /// A tiny two-register device with one read-write and one read-only word.
    const TINY: PeripheralDef = PeripheralDef {
        name: "TINY",
        base: 0x4000_0000,
        window: 0x10,
        irqs: &[IrqDef {
            number: 2,
            name: "TINY_IRQ",
        }],
        registers: &[
            RegisterDef {
                name: "CTRL",
                offset: 0x00,
                width: 32,
                access: Access::ReadWrite,
                reset: 0x0000_0000,
                fields: &[FieldDef {
                    name: "EN",
                    offset: 0,
                    width: 1,
                    access: Access::ReadWrite,
                }],
            },
            RegisterDef {
                name: "STATUS",
                offset: 0x04,
                width: 32,
                access: Access::ReadOnly,
                reset: 0x0000_0042,
                fields: &[],
            },
        ],
    };

    #[derive(Default)]
    struct Noop;

    impl Behavior for Noop {}

    #[allow(clippy::cast_possible_truncation)]
    fn write_word<B: Behavior>(rf: &mut RegisterFile<B>, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            rf.write(rf.ranges()[0].base + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word<B: Behavior>(rf: &mut RegisterFile<B>, off: u32) -> u32 {
        let base = rf.ranges()[0].base;
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = rf.read(base + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[allow(clippy::cast_possible_truncation)]
    fn p_write_word<P: Peripheral>(p: &mut P, off: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            p.write(p.ranges()[0].base + off + i as u32, *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn p_read_word<P: Peripheral>(p: &mut P, off: u32) -> u32 {
        let base = p.ranges()[0].base;
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = p.read(base + off + i as u32).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn register_bytes_roundtrip() {
        let mut rf = RegisterFile::<Noop>::new(&TINY, Noop);
        write_word(&mut rf, 0x00, 0xDEAD_BEEF);
        assert_eq!(read_word(&mut rf, 0x00), 0xDEAD_BEEF);
    }

    #[test]
    fn byte_lane_writes_preserve_other_lanes() {
        let mut rf = RegisterFile::<Noop>::new(&TINY, Noop);
        let base = rf.ranges()[0].base;
        write_word(&mut rf, 0x00, 0xFFFF_FFFF);
        rf.write(base, 0xAB).unwrap();
        assert_eq!(read_word(&mut rf, 0x00), 0xFFFF_FFAB);
        rf.write(base + 0x02, 0xCD).unwrap();
        assert_eq!(read_word(&mut rf, 0x00), 0xFFCD_FFAB);
    }

    #[test]
    fn read_only_registers_reject_writes() {
        let mut rf = RegisterFile::<Noop>::new(&TINY, Noop);
        write_word(&mut rf, 0x04, 0xFFFF_FFFF);
        assert_eq!(read_word(&mut rf, 0x04), 0x0000_0042);
    }

    #[test]
    fn reset_reloads_svd_values() {
        let mut rf = RegisterFile::<Noop>::new(&TINY, Noop);
        write_word(&mut rf, 0x00, 0xFFFF_FFFF);
        rf.reset();
        assert_eq!(read_word(&mut rf, 0x00), 0x0000_0000);
        assert_eq!(read_word(&mut rf, 0x04), 0x0000_0042);
    }

    #[test]
    fn gaps_between_registers_are_reserved() {
        let mut rf = RegisterFile::<Noop>::new(&TINY, Noop);
        assert!(matches!(rf.read(0x4000_0008), Err(BusError::Reserved)));
        assert!(matches!(rf.read(0x4000_1000), Err(BusError::Reserved)));
    }

    #[test]
    fn first_irq_maps_to_an_exception_number() {
        let rf = RegisterFile::<Noop>::new(&TINY, Noop);
        assert_eq!(rf.exception_number(), Some(16 + 2));
        assert_eq!(rf.pending(), 0);
    }

    #[test]
    fn generated_stm32f103_matches_hand_typed_constants() {
        use crate::device::stm32;
        let cases = [
            ("RCC", stm32::RCC),
            ("AFIO", stm32::AFIO),
            ("EXTI", stm32::EXTI),
            ("TIM2", stm32::TIM2),
            ("TIM3", stm32::TIM3),
            ("USART1", stm32::USART1),
            ("GPIOA", stm32::GPIOA),
            ("GPIOB", stm32::GPIOB),
            ("GPIOC", stm32::GPIOC),
        ];
        for (name, base) in cases {
            let def = super::stm32f103::peripheral(name)
                .unwrap_or_else(|| panic!("generated tables contain {name}"));
            assert_eq!(def.base, base, "base address of {name}");
        }
    }

    #[test]
    fn generated_rcc_matches_hand_typed_offsets() {
        let rcc = super::stm32f103::peripheral("RCC").unwrap();
        let cr = rcc.registers.iter().find(|r| r.name == "CR").unwrap();
        assert_eq!(cr.offset, 0x00);
        assert_eq!(cr.reset, 0x0000_0083);
        assert!(cr.fields.iter().any(|f| f.name == "HSION" && f.offset == 0));
        let apb2 = rcc.registers.iter().find(|r| r.name == "APB2ENR").unwrap();
        assert_eq!(apb2.offset, 0x18);
        let apb1 = rcc.registers.iter().find(|r| r.name == "APB1ENR").unwrap();
        assert_eq!(apb1.offset, 0x1C);
    }

    #[test]
    fn generated_gpio_matches_silicon_layout() {
        let gpio = super::stm32f103::peripheral("GPIOA").unwrap();
        let expect = [
            ("CRL", 0x00, Access::ReadWrite, 0x4444_4444),
            ("CRH", 0x04, Access::ReadWrite, 0x4444_4444),
            ("IDR", 0x08, Access::ReadOnly, 0x0000_0000),
            ("ODR", 0x0C, Access::ReadWrite, 0x0000_0000),
            ("BSRR", 0x10, Access::WriteOnly, 0x0000_0000),
            ("BRR", 0x14, Access::WriteOnly, 0x0000_0000),
            ("LCKR", 0x18, Access::ReadWrite, 0x0000_0000),
        ];
        for (name, offset, access, reset) in expect {
            let r = gpio
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("GPIOA:{name} present"));
            assert_eq!(r.offset, offset, "GPIOA:{name} offset");
            assert_eq!(r.access, access, "GPIOA:{name} access");
            assert_eq!(r.reset, reset, "GPIOA:{name} reset");
        }
    }

    #[test]
    fn generated_tim2_matches_silicon_layout() {
        let tim2 = super::stm32f103::peripheral("TIM2").unwrap();
        let expect = [
            ("CR1", 0x00, Access::ReadWrite),
            ("DIER", 0x0C, Access::ReadWrite),
            ("SR", 0x10, Access::ReadWrite),
            ("EGR", 0x14, Access::WriteOnly),
            ("CNT", 0x24, Access::ReadWrite),
            ("PSC", 0x28, Access::ReadWrite),
            ("ARR", 0x2C, Access::ReadWrite),
        ];
        for (name, offset, access) in expect {
            let r = tim2
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("TIM2:{name} present"));
            assert_eq!(r.offset, offset, "TIM2:{name} offset");
            assert_eq!(r.access, access, "TIM2:{name} access");
        }
        let irq = tim2
            .irqs
            .first()
            .unwrap_or_else(|| panic!("TIM2 has an IRQ"));
        assert_eq!(irq.number, 28);
        assert_eq!(16 + irq.number, 44, "TIM2 update exception number");
    }

    #[test]
    fn generated_exti_matches_silicon_layout() {
        let exti = super::stm32f103::peripheral("EXTI").unwrap();
        let expect = [
            ("IMR", 0x00),
            ("EMR", 0x04),
            ("RTSR", 0x08),
            ("FTSR", 0x0C),
            ("SWIER", 0x10),
            ("PR", 0x14),
        ];
        for (name, offset) in expect {
            let r = exti
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("EXTI:{name} present"));
            assert_eq!(r.offset, offset, "EXTI:{name} offset");
            assert_eq!(r.access, Access::ReadWrite, "EXTI:{name} access");
        }
        // The SVD groups TAMPER + EXTI lines under one peripheral; the first
        // IRQ is TAMPER, but the model wires only EXTI line 0.
        assert_eq!(exti.irqs.first().map(|irq| irq.number), Some(2));
        let exti0 = exti.irqs.iter().find(|irq| irq.name == "EXTI0").unwrap();
        assert_eq!(exti0.number, 6);
        assert_eq!(16 + exti0.number, 22, "EXTI0 exception number");
    }

    #[test]
    fn generated_usart1_matches_silicon_layout() {
        let usart1 = super::stm32f103::peripheral("USART1").unwrap();
        let expect = [
            ("SR", 0x00, Access::ReadWrite, 0x0000_00c0),
            ("DR", 0x04, Access::ReadWrite, 0x0000_0000),
            ("BRR", 0x08, Access::ReadWrite, 0x0000_0000),
            ("CR1", 0x0C, Access::ReadWrite, 0x0000_0000),
            ("CR2", 0x10, Access::ReadWrite, 0x0000_0000),
            ("CR3", 0x14, Access::ReadWrite, 0x0000_0000),
            ("GTPR", 0x18, Access::ReadWrite, 0x0000_0000),
        ];
        for (name, offset, access, reset) in expect {
            let r = usart1
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("USART1:{name} present"));
            assert_eq!(r.offset, offset, "USART1:{name} offset");
            assert_eq!(r.access, access, "USART1:{name} access");
            assert_eq!(r.reset, reset, "USART1:{name} reset");
        }
        let irq = usart1
            .irqs
            .first()
            .unwrap_or_else(|| panic!("USART1 has an IRQ"));
        assert_eq!(irq.number, 37);
        assert_eq!(16 + irq.number, 53, "USART1 exception number");
    }

    #[test]
    fn generated_usart_family_matches_silicon_layout() {
        // USART2/USART3 on every line, UART4/UART5 on the connectivity and F4
        // lines, USART6 on the F4: bases and IRQ numbers from the manuals.
        let cases = [
            (
                super::stm32f103::peripheral("USART2").unwrap(),
                0x4000_4400u32,
                38,
            ),
            (
                super::stm32f103::peripheral("USART3").unwrap(),
                0x4000_4800,
                39,
            ),
            (
                super::stm32f107::peripheral("USART2").unwrap(),
                0x4000_4400,
                38,
            ),
            (
                super::stm32f107::peripheral("USART3").unwrap(),
                0x4000_4800,
                39,
            ),
            (
                super::stm32f107::peripheral("UART4").unwrap(),
                0x4000_4c00,
                52,
            ),
            (
                super::stm32f107::peripheral("UART5").unwrap(),
                0x4000_5000,
                53,
            ),
            (
                super::stm32f407::peripheral("USART2").unwrap(),
                0x4000_4400,
                38,
            ),
            (
                super::stm32f407::peripheral("USART3").unwrap(),
                0x4000_4800,
                39,
            ),
            (
                super::stm32f407::peripheral("UART4").unwrap(),
                0x4000_4c00,
                52,
            ),
            (
                super::stm32f407::peripheral("UART5").unwrap(),
                0x4000_5000,
                53,
            ),
            (
                super::stm32f407::peripheral("USART6").unwrap(),
                0x4001_1400,
                71,
            ),
        ];
        for (def, base, irq) in cases {
            assert_eq!(def.base, base, "base address of {}", def.name);
            let got = def
                .irqs
                .first()
                .unwrap_or_else(|| panic!("{} has an IRQ", def.name));
            assert_eq!(got.number, irq, "IRQ number of {}", def.name);
        }
    }

    #[test]
    fn generated_timer_family_matches_silicon_layout() {
        // TIM1/TIM4..TIM7 bases and update IRQs on every line. TIM1's table
        // lists break first, so its update event is the second line (IRQ 25);
        // the general/basic timers have a single line.
        let cases = [
            (
                super::stm32f103::peripheral("TIM1").unwrap(),
                0x4001_2c00u32,
                24,
                Some(25),
            ),
            (
                super::stm32f103::peripheral("TIM4").unwrap(),
                0x4000_0800,
                30,
                None,
            ),
            (
                super::stm32f103::peripheral("TIM5").unwrap(),
                0x4000_0c00,
                50,
                None,
            ),
            (
                super::stm32f103::peripheral("TIM6").unwrap(),
                0x4000_1000,
                54,
                None,
            ),
            (
                super::stm32f103::peripheral("TIM7").unwrap(),
                0x4000_1400,
                55,
                None,
            ),
            (
                super::stm32f107::peripheral("TIM1").unwrap(),
                0x4001_2c00,
                24,
                Some(25),
            ),
            (
                super::stm32f107::peripheral("TIM4").unwrap(),
                0x4000_0800,
                30,
                None,
            ),
            (
                super::stm32f107::peripheral("TIM5").unwrap(),
                0x4000_0c00,
                50,
                None,
            ),
            (
                super::stm32f107::peripheral("TIM6").unwrap(),
                0x4000_1000,
                54,
                None,
            ),
            (
                super::stm32f107::peripheral("TIM7").unwrap(),
                0x4000_1400,
                55,
                None,
            ),
            (
                super::stm32f407::peripheral("TIM1").unwrap(),
                0x4001_0000,
                24,
                Some(25),
            ),
            (
                super::stm32f407::peripheral("TIM4").unwrap(),
                0x4000_0800,
                30,
                None,
            ),
            (
                super::stm32f407::peripheral("TIM5").unwrap(),
                0x4000_0c00,
                50,
                None,
            ),
            (
                super::stm32f407::peripheral("TIM6").unwrap(),
                0x4000_1000,
                54,
                None,
            ),
            (
                super::stm32f407::peripheral("TIM7").unwrap(),
                0x4000_1400,
                55,
                None,
            ),
        ];
        for (def, base, irq0, update) in cases {
            assert_eq!(def.base, base, "base address of {}", def.name);
            assert_eq!(def.irqs[0].number, irq0, "first IRQ of {}", def.name);
            if let Some(update) = update {
                assert_eq!(def.irqs[1].number, update, "update IRQ of {}", def.name);
            }
        }
    }

    #[test]
    fn generated_can_family_matches_silicon_layout() {
        // CAN1 and CAN2 on the connectivity and F4 lines. Each controller has
        // four lines: transmit, receive FIFO 0, receive FIFO 1, status/error.
        let cases = [
            (
                super::stm32f107::peripheral("CAN1").unwrap(),
                0x4000_6400u32,
                [19u32, 20, 21, 22],
            ),
            (
                super::stm32f107::peripheral("CAN2").unwrap(),
                0x4000_6800,
                [63, 64, 65, 66],
            ),
            (
                super::stm32f407::peripheral("CAN1").unwrap(),
                0x4000_6400,
                [19, 20, 21, 22],
            ),
            (
                super::stm32f407::peripheral("CAN2").unwrap(),
                0x4000_6800,
                [63, 64, 65, 66],
            ),
        ];
        for (def, base, irqs) in cases {
            assert_eq!(def.base, base, "base address of {}", def.name);
            let got: Vec<u32> = def.irqs.iter().map(|i| u32::from(i.number)).collect();
            assert_eq!(got, irqs, "IRQ lines of {}", def.name);
        }
    }

    #[test]
    fn generated_usb_family_matches_silicon_layout() {
        // The OTG controller has one GLOBAL window per variant, named
        // `USB_OTG_` on the F1 line and `OTG_FS_`/`OTG_HS_` on the F4 line.
        // The model derives every window's base from the GLOBAL base, so the
        // four windows must sit at the silicon strides.
        let cases: &[(&'static super::PeripheralDef, u32, u32, &[u32])] = &[
            (
                super::stm32f107::peripheral("USB_OTG_GLOBAL").unwrap(),
                0x5000_0000u32,
                0x0400,
                &[67],
            ),
            (
                super::stm32f407::peripheral("OTG_FS_GLOBAL").unwrap(),
                0x5000_0000,
                0x0400,
                &[42, 67],
            ),
            (
                super::stm32f407::peripheral("OTG_HS_GLOBAL").unwrap(),
                0x4004_0000,
                0x0131,
                &[74, 75, 76, 77],
            ),
        ];
        for &(def, base, window, irqs) in cases {
            assert_eq!(def.base, base, "base address of {}", def.name);
            assert_eq!(def.window, window, "window of {}", def.name);
            let got: Vec<u32> = def.irqs.iter().map(|i| u32::from(i.number)).collect();
            assert_eq!(got, irqs, "IRQ lines of {}", def.name);
        }
        // The window bases are fixed strides from the GLOBAL base.
        let f107 = super::stm32f107::peripheral("USB_OTG_GLOBAL").unwrap();
        for (suffix, stride) in [("HOST", 0x400u32), ("DEVICE", 0x800), ("PWRCLK", 0xe00)] {
            let def = super::stm32f107::peripheral(&format!("USB_OTG_{suffix}")).unwrap();
            assert_eq!(def.base, f107.base + stride, "USB_OTG_{suffix}");
        }
    }

    #[test]
    fn generated_stm32f107_matches_silicon_layout() {
        let cases = [
            ("RCC", 0x4002_1000u32),
            ("AFIO", 0x4001_0000),
            ("EXTI", 0x4001_0400),
            ("TIM2", 0x4000_0000),
            ("USART1", 0x4001_3800),
            ("GPIOA", 0x4001_0800),
            ("GPIOE", 0x4001_1800),
            ("CAN1", 0x4000_6400),
            ("CAN2", 0x4000_6800),
            ("ETHERNET_MAC", 0x4002_8000),
            ("USB_OTG_GLOBAL", 0x5000_0000),
        ];
        for (name, base) in cases {
            let def = super::stm32f107::peripheral(name)
                .unwrap_or_else(|| panic!("generated tables contain {name}"));
            assert_eq!(def.base, base, "base address of {name}");
        }
        let usart1 = super::stm32f107::peripheral("USART1").unwrap();
        let irq = usart1.irqs.first().unwrap();
        assert_eq!(irq.number, 37, "USART1 IRQ number");
        assert_eq!(16 + irq.number, 53, "USART1 exception number");
        let eth = super::stm32f107::peripheral("ETHERNET_MAC").unwrap();
        assert!(eth.irqs.iter().any(|i| i.number == 61), "ETH has IRQ 61");
        let gpioa = super::stm32f107::peripheral("GPIOA").unwrap();
        let crl = gpioa.registers.iter().find(|r| r.name == "CRL").unwrap();
        assert_eq!(crl.offset, 0x00);
        assert_eq!(crl.reset, 0x4444_4444);
    }

    #[test]
    fn same_behavior_mounts_f107_tables() {
        use super::stm32f107;
        use crate::device::stm32::afio::Afio;
        use crate::device::stm32::exti::Exti;
        use crate::device::stm32::gpio::Gpio;
        use crate::device::stm32::rcc::Rcc;
        use crate::device::stm32::timer::Timer;
        use crate::device::stm32::usart::Usart;
        use crate::device::{InterruptSource, Peripheral};

        // USART1: the byte-oriented behavior on F107's tables.
        let mut u = Usart::for_device::<stm32f107::Stm32f107>();
        assert_eq!(u.ranges()[0].base, 0x4001_3800);
        u.drive_inputs(0x55);
        assert_eq!(u.pending(), 0);
        p_write_word(&mut u, 0x0C, 0x2020); // CR1 = UE | RXNEIE
        assert_eq!(u.pending(), 1);
        assert_eq!(u.read(0x4001_3804).unwrap(), 0x55);
        assert_eq!(u.pending(), 0);
        p_write_word(&mut u, 0x0C, 0x2008); // CR1 = UE | TE
        u.write(0x4001_3804, b'X').unwrap();
        assert_eq!(u.drain_output(), b"X");

        // RCC: HSIRDY synthesis on F107's tables.
        let mut rcc = Rcc::for_device::<stm32f107::Stm32f107>();
        assert_eq!(rcc.ranges()[0].base, 0x4002_1000);
        assert_eq!(p_read_word(&mut rcc, 0x00) & (1 << 1), 1 << 1);

        // GPIO: BSRR/IDR behavior on F107's GPIOE.
        let mut gpioe = Gpio::at_in::<stm32f107::Stm32f107>(0x4001_1800, "gpioe");
        p_write_word(&mut gpioe, 0x00, 0x0000_0002); // pin 0 output push-pull
        p_write_word(&mut gpioe, 0x10, 0x0000_0001); // BSRR sets pin 0
        assert_eq!(p_read_word(&mut gpioe, 0x0C) & 1, 1, "ODR follows BSRR");
        assert_eq!(
            p_read_word(&mut gpioe, 0x08) & 1,
            1,
            "IDR reads the output latch"
        );

        // Timer: tick-driven UIF on F107's TIM2.
        let mut tim2 = Timer::tim2_for::<stm32f107::Stm32f107>();
        p_write_word(&mut tim2, 0x2C, 1); // ARR = 1
        p_write_word(&mut tim2, 0x28, 0); // PSC = 0
        p_write_word(&mut tim2, 0x0C, 1); // DIER = UIE
        p_write_word(&mut tim2, 0x00, 1); // CR1 = CEN
        assert_eq!(tim2.pending(), 0);
        tim2.tick(2);
        assert_eq!(tim2.pending(), 1, "UIF latched on F107 TIM2");

        // EXTI and AFIO construct against F107's tables.
        assert_eq!(
            Exti::for_device::<stm32f107::Stm32f107>().ranges()[0].base,
            0x4001_0400
        );
        assert_eq!(
            Afio::for_device::<stm32f107::Stm32f107>().ranges()[0].base,
            0x4001_0000
        );
    }

    #[test]
    fn generated_stm32f407_matches_silicon_layout() {
        let cases = [
            ("GPIOA", 0x4002_0000u32),
            ("GPIOE", 0x4002_1000),
            ("GPIOH", 0x4002_1c00),
            ("RCC", 0x4002_3800),
            ("FLASH", 0x4002_3c00),
            ("USART1", 0x4001_1000),
            ("USART6", 0x4001_1400),
            ("TIM1", 0x4001_0000),
            ("TIM2", 0x4000_0000),
            ("TIM11", 0x4001_4800),
            ("SYSCFG", 0x4001_3800),
            ("EXTI", 0x4001_3c00),
            ("DMA1", 0x4002_6000),
            ("DMA2", 0x4002_6400),
            ("SPI1", 0x4001_3000),
            ("I2C1", 0x4000_5400),
            ("CAN1", 0x4000_6400),
            ("CAN2", 0x4000_6800),
            ("ADC1", 0x4001_2000),
            ("DAC", 0x4000_7400),
            ("SDIO", 0x4001_2c00),
            ("Ethernet_MAC", 0x4002_8000),
            ("OTG_FS_GLOBAL", 0x5000_0000),
            ("OTG_HS_GLOBAL", 0x4004_0000),
        ];
        for (name, base) in cases {
            let def = super::stm32f407::peripheral(name)
                .unwrap_or_else(|| panic!("generated tables contain {name}"));
            assert_eq!(def.base, base, "base address of {name}");
        }

        // GPIOA is the F4 split design: MODER mode nibbles replace the F1 CRL/CRH.
        let gpioa = super::stm32f407::peripheral("GPIOA").unwrap();
        let gpio_regs = [
            ("MODER", 0x00, Access::ReadWrite),
            ("OTYPER", 0x04, Access::ReadWrite),
            ("OSPEEDR", 0x08, Access::ReadWrite),
            ("PUPDR", 0x0c, Access::ReadWrite),
            ("IDR", 0x10, Access::ReadOnly),
            ("ODR", 0x14, Access::ReadWrite),
            ("BSRR", 0x18, Access::WriteOnly),
            ("LCKR", 0x1c, Access::ReadWrite),
            ("AFRL", 0x20, Access::ReadWrite),
            ("AFRH", 0x24, Access::ReadWrite),
        ];
        for (name, offset, access) in gpio_regs {
            let r = gpioa
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("GPIOA:{name} present"));
            assert_eq!(r.offset, offset, "GPIOA:{name} offset");
            assert_eq!(r.access, access, "GPIOA:{name} access");
        }
        assert!(gpioa
            .registers
            .iter()
            .all(|r| r.name != "CRL" && r.name != "BRR"));
        let moder = gpioa.registers.iter().find(|r| r.name == "MODER").unwrap();
        assert_eq!(moder.reset, 0xa800_0000, "pins 14/15 reset to analog");
        let pupdr = gpioa.registers.iter().find(|r| r.name == "PUPDR").unwrap();
        assert_eq!(
            pupdr.reset & 0x2400_0000,
            0x2400_0000,
            "SWD pins 13/14 reset pulled up"
        );

        // RCC keeps CR at 0x00 but gates clocks in AHB1ENR, not AHBENR.
        let rcc = super::stm32f407::peripheral("RCC").unwrap();
        let rcc_regs = [
            ("CR", 0x00, 0x0000_0083),
            ("PLLCFGR", 0x04, 0x2400_3010),
            ("AHB1ENR", 0x30, 0x0010_0000),
            ("APB1ENR", 0x40, 0x0000_0000),
            ("APB2ENR", 0x44, 0x0000_0000),
        ];
        for (name, offset, reset) in rcc_regs {
            let r = rcc
                .registers
                .iter()
                .find(|r| r.name == name)
                .unwrap_or_else(|| panic!("RCC:{name} present"));
            assert_eq!(r.offset, offset, "RCC:{name} offset");
            assert_eq!(r.reset, reset, "RCC:{name} reset");
        }
        assert!(rcc.registers.iter().all(|r| r.name != "AHBENR"));

        // AFIO is gone on the Cortex-M4 line; SYSCFG took its place.
        assert!(super::stm32f407::peripheral("AFIO").is_none());

        let usart1 = super::stm32f407::peripheral("USART1").unwrap();
        let irq = usart1.irqs.first().unwrap();
        assert_eq!(irq.number, 37, "USART1 IRQ number");
        assert_eq!(16 + irq.number, 53, "USART1 exception number");
    }

    #[test]
    fn same_behavior_mounts_f407_tables() {
        use super::stm32f407;
        use crate::device::stm32::timer::Timer;
        use crate::device::stm32::usart::Usart;
        use crate::device::{InterruptSource, Peripheral};

        // USART1: the byte-oriented behavior on F407's tables (same 7-register
        // layout as F1, different base).
        let mut u = Usart::for_device::<stm32f407::Stm32f407>();
        assert_eq!(u.ranges()[0].base, 0x4001_1000);
        u.drive_inputs(0x55);
        assert_eq!(u.pending(), 0);
        p_write_word(&mut u, 0x0C, 0x2020); // CR1 = UE | RXNEIE
        assert_eq!(u.pending(), 1);
        assert_eq!(u.read(0x4001_1004).unwrap(), 0x55);
        assert_eq!(u.pending(), 0);
        p_write_word(&mut u, 0x0C, 0x2008); // CR1 = UE | TE
        u.write(0x4001_1004, b'X').unwrap();
        assert_eq!(u.drain_output(), b"X");

        // Timer: tick-driven UIF on F407's TIM2.
        let mut tim2 = Timer::tim2_for::<stm32f407::Stm32f407>();
        p_write_word(&mut tim2, 0x2C, 1); // ARR = 1
        p_write_word(&mut tim2, 0x28, 0); // PSC = 0
        p_write_word(&mut tim2, 0x0C, 1); // DIER = UIE
        p_write_word(&mut tim2, 0x00, 1); // CR1 = CEN
        assert_eq!(tim2.pending(), 0);
        tim2.tick(2);
        assert_eq!(tim2.pending(), 1, "UIF latched on F407 TIM2");

        // RCC: the CR synthesis mounts unchanged on F407's RCC.
        let mut rcc = crate::device::stm32::rcc::Rcc::for_device::<stm32f407::Stm32f407>();
        assert_eq!(rcc.ranges()[0].base, 0x4002_3800);
        assert_eq!(p_read_word(&mut rcc, 0x00) & (1 << 1), 1 << 1);

        // EXTI: edge detection on F407's controller.
        let mut exti = crate::device::stm32::exti::Exti::for_device::<stm32f407::Stm32f407>();
        assert_eq!(exti.ranges()[0].base, 0x4001_3c00);
        p_write_word(&mut exti, 0x08, 1); // RTSR line 0
        p_write_word(&mut exti, 0x00, 1); // IMR line 0
        exti.drive_inputs(0); // establish reference level
        exti.drive_inputs(1); // rising edge
        assert_eq!(exti.pending(), 1);

        // SYSCFG: the F4 EXTI router latches EXTICR routing.
        let mut syscfg = crate::device::stm32::syscfg::Syscfg::for_device::<stm32f407::Stm32f407>();
        assert_eq!(syscfg.ranges()[0].base, 0x4001_3800);
        p_write_word(&mut syscfg, 0x08, 0x0000_0001); // EXTICR1: EXTI0 <- PA0
        assert_eq!(p_read_word(&mut syscfg, 0x08), 0x0000_0001);

        // F4 GPIO: MODER-configured outputs drive ODR/IDR through BSRR.
        let mut gpioa = crate::device::stm32::gpio_f4::GpioF4::at_in::<stm32f407::Stm32f407>(
            0x4002_0000,
            "gpioa",
        );
        p_write_word(&mut gpioa, 0x00, 0x0000_0001); // MODER: PA0 output
        p_write_word(&mut gpioa, 0x18, 0x0000_0001); // BSRR set pin 0
        assert_eq!(p_read_word(&mut gpioa, 0x14) & 1, 1, "ODR follows BSRR");
        assert_eq!(
            p_read_word(&mut gpioa, 0x10) & 1,
            1,
            "IDR reads the output latch"
        );
    }

    #[test]
    fn generated_eth_family_matches_silicon_layout() {
        // The four Ethernet windows exist on both the F1 connectivity line and
        // the F4. The F4 MAC window is trimmed to 0x61 (no MACRWUFFR), which is
        // exactly the divergence the controller model must respect.
        let f107 = [
            ("ETHERNET_MAC", 0x4002_8000u32, 0x0400u32),
            ("ETHERNET_MMC", 0x4002_8100, 0x0400),
            ("ETHERNET_PTP", 0x4002_8700, 0x0400),
            ("ETHERNET_DMA", 0x4002_9000, 0x0400),
        ];
        for (name, base, window) in f107 {
            let def = super::stm32f107::peripheral(name)
                .unwrap_or_else(|| panic!("generated tables contain {name}"));
            assert_eq!(def.base, base, "F107 {name} base");
            assert_eq!(def.window, window, "F107 {name} window");
        }
        let mac = super::stm32f107::peripheral("ETHERNET_MAC").unwrap();
        let irqs: Vec<(u8, &str)> = mac.irqs.iter().map(|i| (i.number, i.name)).collect();
        assert_eq!(irqs, [(61, "ETH"), (62, "ETH_WKUP")]);
        assert!(mac.registers.iter().any(|r| r.name == "MACRWUFFR"));

        let f407 = [
            ("Ethernet_MAC", 0x4002_8000u32, 0x61u32),
            ("Ethernet_MMC", 0x4002_8100, 0x0400),
            ("Ethernet_PTP", 0x4002_8700, 0x0400),
            ("Ethernet_DMA", 0x4002_9000, 0x0400),
        ];
        for (name, base, window) in f407 {
            let def = super::stm32f407::peripheral(name)
                .unwrap_or_else(|| panic!("generated tables contain {name}"));
            assert_eq!(def.base, base, "F407 {name} base");
            assert_eq!(def.window, window, "F407 {name} window");
        }
        let mac = super::stm32f407::peripheral("Ethernet_MAC").unwrap();
        let irqs: Vec<(u8, &str)> = mac.irqs.iter().map(|i| (i.number, i.name)).collect();
        assert_eq!(irqs, [(61, "ETH"), (62, "ETH_WKUP")]);
        assert!(mac.registers.iter().all(|r| r.name != "MACRWUFFR"));
    }

    #[test]
    fn generated_tables_are_internally_consistent() {
        let tables = [
            super::stm32c011::PERIPHERALS,
            super::stm32c011a0::PERIPHERALS,
            super::stm32c011a1::PERIPHERALS,
            super::stm32c011b0::PERIPHERALS,
            super::stm32c011f6::PERIPHERALS,
            super::stm32c031::PERIPHERALS,
            super::stm32c031a0::PERIPHERALS,
            super::stm32c031a1::PERIPHERALS,
            super::stm32c031b0::PERIPHERALS,
            super::stm32c031f6::PERIPHERALS,
            super::stm32f030::PERIPHERALS,
            super::stm32f030a0::PERIPHERALS,
            super::stm32f030a1::PERIPHERALS,
            super::stm32f030b0::PERIPHERALS,
            super::stm32f030b1::PERIPHERALS,
            super::stm32f030c6::PERIPHERALS,
            super::stm32f030cc::PERIPHERALS,
            super::stm32f030k6::PERIPHERALS,
            super::stm32f030r8::PERIPHERALS,
            super::stm32f030t6::PERIPHERALS,
            super::stm32f030v6::PERIPHERALS,
            super::stm32f031::PERIPHERALS,
            super::stm32f038::PERIPHERALS,
            super::stm32f038k6::PERIPHERALS,
            super::stm32f038r8::PERIPHERALS,
            super::stm32f038t6::PERIPHERALS,
            super::stm32f038v6::PERIPHERALS,
            super::stm32f042::PERIPHERALS,
            super::stm32f042a0::PERIPHERALS,
            super::stm32f042a1::PERIPHERALS,
            super::stm32f042b0::PERIPHERALS,
            super::stm32f042b1::PERIPHERALS,
            super::stm32f042c8::PERIPHERALS,
            super::stm32f042g6::PERIPHERALS,
            super::stm32f042k4::PERIPHERALS,
            super::stm32f042k6::PERIPHERALS,
            super::stm32f042r8::PERIPHERALS,
            super::stm32f042t6::PERIPHERALS,
            super::stm32f048::PERIPHERALS,
            super::stm32f051::PERIPHERALS,
            super::stm32f051g8::PERIPHERALS,
            super::stm32f051k4::PERIPHERALS,
            super::stm32f051r8::PERIPHERALS,
            super::stm32f070::PERIPHERALS,
            super::stm32f071::PERIPHERALS,
            super::stm32f072::PERIPHERALS,
            super::stm32f072a0::PERIPHERALS,
            super::stm32f072a1::PERIPHERALS,
            super::stm32f072b0::PERIPHERALS,
            super::stm32f072b1::PERIPHERALS,
            super::stm32f072c8::PERIPHERALS,
            super::stm32f072k8::PERIPHERALS,
            super::stm32f072r8::PERIPHERALS,
            super::stm32f072rb::PERIPHERALS,
            super::stm32f091::PERIPHERALS,
            super::stm32f091a0::PERIPHERALS,
            super::stm32f091a1::PERIPHERALS,
            super::stm32f091b0::PERIPHERALS,
            super::stm32f091b1::PERIPHERALS,
            super::stm32f091cb::PERIPHERALS,
            super::stm32f091rb::PERIPHERALS,
            super::stm32f091rc::PERIPHERALS,
            super::stm32f091vb::PERIPHERALS,
            super::stm32f098::PERIPHERALS,
            super::stm32f0x0::PERIPHERALS,
            super::stm32f0x1::PERIPHERALS,
            super::stm32f0x2::PERIPHERALS,
            super::stm32f0x8::PERIPHERALS,
            super::stm32f0xx::PERIPHERALS,
            super::stm32f100::PERIPHERALS,
            super::stm32f100a0::PERIPHERALS,
            super::stm32f100a1::PERIPHERALS,
            super::stm32f100b0::PERIPHERALS,
            super::stm32f100b1::PERIPHERALS,
            super::stm32f100c8::PERIPHERALS,
            super::stm32f100rb::PERIPHERALS,
            super::stm32f100rc::PERIPHERALS,
            super::stm32f100re::PERIPHERALS,
            super::stm32f101::PERIPHERALS,
            super::stm32f101a0::PERIPHERALS,
            super::stm32f101a1::PERIPHERALS,
            super::stm32f101b0::PERIPHERALS,
            super::stm32f101b1::PERIPHERALS,
            super::stm32f101c8::PERIPHERALS,
            super::stm32f101rb::PERIPHERALS,
            super::stm32f101rc::PERIPHERALS,
            super::stm32f101re::PERIPHERALS,
            super::stm32f102::PERIPHERALS,
            super::stm32f102a0::PERIPHERALS,
            super::stm32f102a1::PERIPHERALS,
            super::stm32f102b0::PERIPHERALS,
            super::stm32f102cb::PERIPHERALS,
            super::stm32f102rb::PERIPHERALS,
            super::stm32f102rc::PERIPHERALS,
            super::stm32f102re::PERIPHERALS,
            super::stm32f103::PERIPHERALS,
            super::stm32f103a0::PERIPHERALS,
            super::stm32f103a1::PERIPHERALS,
            super::stm32f103b0::PERIPHERALS,
            super::stm32f103c6::PERIPHERALS,
            super::stm32f103c8::PERIPHERALS,
            super::stm32f103r8::PERIPHERALS,
            super::stm32f103rb::PERIPHERALS,
            super::stm32f103rc::PERIPHERALS,
            super::stm32f103re::PERIPHERALS,
            super::stm32f103vd::PERIPHERALS,
            super::stm32f103x49::PERIPHERALS,
            super::stm32f103ze::PERIPHERALS,
            super::stm32f105::PERIPHERALS,
            super::stm32f105a0::PERIPHERALS,
            super::stm32f105a1::PERIPHERALS,
            super::stm32f105b0::PERIPHERALS,
            super::stm32f105rb::PERIPHERALS,
            super::stm32f105rc::PERIPHERALS,
            super::stm32f105re::PERIPHERALS,
            super::stm32f105rg::PERIPHERALS,
            super::stm32f107::PERIPHERALS,
            super::stm32f107a0::PERIPHERALS,
            super::stm32f107a1::PERIPHERALS,
            super::stm32f107b0::PERIPHERALS,
            super::stm32f107rc::PERIPHERALS,
            super::stm32f107re::PERIPHERALS,
            super::stm32f107rg::PERIPHERALS,
            super::stm32f107vc::PERIPHERALS,
            super::stm32f205::PERIPHERALS,
            super::stm32f205a0::PERIPHERALS,
            super::stm32f205a1::PERIPHERALS,
            super::stm32f205b0::PERIPHERALS,
            super::stm32f205rb::PERIPHERALS,
            super::stm32f205rc::PERIPHERALS,
            super::stm32f205re::PERIPHERALS,
            super::stm32f205rg::PERIPHERALS,
            super::stm32f207::PERIPHERALS,
            super::stm32f207a0::PERIPHERALS,
            super::stm32f207a1::PERIPHERALS,
            super::stm32f207b0::PERIPHERALS,
            super::stm32f207rb::PERIPHERALS,
            super::stm32f207rc::PERIPHERALS,
            super::stm32f207rg::PERIPHERALS,
            super::stm32f207ze::PERIPHERALS,
            super::stm32f215::PERIPHERALS,
            super::stm32f215a0::PERIPHERALS,
            super::stm32f215a1::PERIPHERALS,
            super::stm32f215b0::PERIPHERALS,
            super::stm32f215rc::PERIPHERALS,
            super::stm32f215re::PERIPHERALS,
            super::stm32f215rg::PERIPHERALS,
            super::stm32f215ze::PERIPHERALS,
            super::stm32f217::PERIPHERALS,
            super::stm32f217a0::PERIPHERALS,
            super::stm32f217a1::PERIPHERALS,
            super::stm32f217b0::PERIPHERALS,
            super::stm32f217ig::PERIPHERALS,
            super::stm32f217rc::PERIPHERALS,
            super::stm32f217rg::PERIPHERALS,
            super::stm32f217ve::PERIPHERALS,
            super::stm32f301::PERIPHERALS,
            super::stm32f301a0::PERIPHERALS,
            super::stm32f301a1::PERIPHERALS,
            super::stm32f301b0::PERIPHERALS,
            super::stm32f301c8::PERIPHERALS,
            super::stm32f301k6::PERIPHERALS,
            super::stm32f301k8::PERIPHERALS,
            super::stm32f301r8::PERIPHERALS,
            super::stm32f302::PERIPHERALS,
            super::stm32f302a0::PERIPHERALS,
            super::stm32f302a1::PERIPHERALS,
            super::stm32f302b0::PERIPHERALS,
            super::stm32f302cb::PERIPHERALS,
            super::stm32f302cc::PERIPHERALS,
            super::stm32f302k6::PERIPHERALS,
            super::stm32f302r8::PERIPHERALS,
            super::stm32f303::PERIPHERALS,
            super::stm32f303a0::PERIPHERALS,
            super::stm32f303a1::PERIPHERALS,
            super::stm32f303b0::PERIPHERALS,
            super::stm32f303cc::PERIPHERALS,
            super::stm32f303k8::PERIPHERALS,
            super::stm32f303re::PERIPHERALS,
            super::stm32f303rg::PERIPHERALS,
            super::stm32f303xe::PERIPHERALS,
            super::stm32f318::PERIPHERALS,
            super::stm32f328::PERIPHERALS,
            super::stm32f333::PERIPHERALS,
            super::stm32f334::PERIPHERALS,
            super::stm32f358::PERIPHERALS,
            super::stm32f373::PERIPHERALS,
            super::stm32f373a0::PERIPHERALS,
            super::stm32f373a1::PERIPHERALS,
            super::stm32f373b0::PERIPHERALS,
            super::stm32f373cb::PERIPHERALS,
            super::stm32f373cc::PERIPHERALS,
            super::stm32f373k8::PERIPHERALS,
            super::stm32f373rg::PERIPHERALS,
            super::stm32f378::PERIPHERALS,
            super::stm32f378a0::PERIPHERALS,
            super::stm32f378a1::PERIPHERALS,
            super::stm32f378b0::PERIPHERALS,
            super::stm32f378be::PERIPHERALS,
            super::stm32f378cc::PERIPHERALS,
            super::stm32f378k8::PERIPHERALS,
            super::stm32f378rg::PERIPHERALS,
            super::stm32f398::PERIPHERALS,
            super::stm32f401::PERIPHERALS,
            super::stm32f401a0::PERIPHERALS,
            super::stm32f401a1::PERIPHERALS,
            super::stm32f401b0::PERIPHERALS,
            super::stm32f401cc::PERIPHERALS,
            super::stm32f401ce::PERIPHERALS,
            super::stm32f401k6::PERIPHERALS,
            super::stm32f401rb::PERIPHERALS,
            super::stm32f401re::PERIPHERALS,
            super::stm32f401rg::PERIPHERALS,
            super::stm32f401ve::PERIPHERALS,
            super::stm32f405::PERIPHERALS,
            super::stm32f407::PERIPHERALS,
            super::stm32f410::PERIPHERALS,
            super::stm32f410a0::PERIPHERALS,
            super::stm32f410a1::PERIPHERALS,
            super::stm32f410b0::PERIPHERALS,
            super::stm32f410c8::PERIPHERALS,
            super::stm32f410k8::PERIPHERALS,
            super::stm32f410rb::PERIPHERALS,
            super::stm32f410re::PERIPHERALS,
            super::stm32f411::PERIPHERALS,
            super::stm32f411re::PERIPHERALS,
            super::stm32f412::PERIPHERALS,
            super::stm32f412a0::PERIPHERALS,
            super::stm32f412a1::PERIPHERALS,
            super::stm32f412b0::PERIPHERALS,
            super::stm32f412ce::PERIPHERALS,
            super::stm32f412k6::PERIPHERALS,
            super::stm32f412re::PERIPHERALS,
            super::stm32f412zg::PERIPHERALS,
            super::stm32f413::PERIPHERALS,
            super::stm32f413a0::PERIPHERALS,
            super::stm32f413a1::PERIPHERALS,
            super::stm32f413b0::PERIPHERALS,
            super::stm32f413ch::PERIPHERALS,
            super::stm32f413k6::PERIPHERALS,
            super::stm32f413re::PERIPHERALS,
            super::stm32f413rg::PERIPHERALS,
            super::stm32f415::PERIPHERALS,
            super::stm32f417::PERIPHERALS,
            super::stm32f423::PERIPHERALS,
            super::stm32f423a0::PERIPHERALS,
            super::stm32f423a1::PERIPHERALS,
            super::stm32f423b0::PERIPHERALS,
            super::stm32f423ih::PERIPHERALS,
            super::stm32f423k6::PERIPHERALS,
            super::stm32f423rg::PERIPHERALS,
            super::stm32f423ze::PERIPHERALS,
            super::stm32f427::PERIPHERALS,
            super::stm32f427a0::PERIPHERALS,
            super::stm32f427a1::PERIPHERALS,
            super::stm32f427ai::PERIPHERALS,
            super::stm32f427b0::PERIPHERALS,
            super::stm32f427ig::PERIPHERALS,
            super::stm32f427k6::PERIPHERALS,
            super::stm32f427rg::PERIPHERALS,
            super::stm32f429::PERIPHERALS,
            super::stm32f429a0::PERIPHERALS,
            super::stm32f429a1::PERIPHERALS,
            super::stm32f429b0::PERIPHERALS,
            super::stm32f429be::PERIPHERALS,
            super::stm32f429ig::PERIPHERALS,
            super::stm32f429k6::PERIPHERALS,
            super::stm32f429rg::PERIPHERALS,
            super::stm32f429zi::PERIPHERALS,
            super::stm32f437::PERIPHERALS,
            super::stm32f437a0::PERIPHERALS,
            super::stm32f437a1::PERIPHERALS,
            super::stm32f437ai::PERIPHERALS,
            super::stm32f437b0::PERIPHERALS,
            super::stm32f437ii::PERIPHERALS,
            super::stm32f437k6::PERIPHERALS,
            super::stm32f437rg::PERIPHERALS,
            super::stm32f439::PERIPHERALS,
            super::stm32f446::PERIPHERALS,
            super::stm32f446a0::PERIPHERALS,
            super::stm32f446a1::PERIPHERALS,
            super::stm32f446b0::PERIPHERALS,
            super::stm32f446k6::PERIPHERALS,
            super::stm32f446re::PERIPHERALS,
            super::stm32f446rg::PERIPHERALS,
            super::stm32f446ze::PERIPHERALS,
            super::stm32f469::PERIPHERALS,
            super::stm32f469a0::PERIPHERALS,
            super::stm32f469a1::PERIPHERALS,
            super::stm32f469ae::PERIPHERALS,
            super::stm32f469ai::PERIPHERALS,
            super::stm32f469b0::PERIPHERALS,
            super::stm32f469k6::PERIPHERALS,
            super::stm32f469rg::PERIPHERALS,
            super::stm32f479::PERIPHERALS,
            super::stm32f722::PERIPHERALS,
            super::stm32f722a0::PERIPHERALS,
            super::stm32f722a1::PERIPHERALS,
            super::stm32f722b0::PERIPHERALS,
            super::stm32f722ie::PERIPHERALS,
            super::stm32f722k6::PERIPHERALS,
            super::stm32f722rg::PERIPHERALS,
            super::stm32f722ze::PERIPHERALS,
            super::stm32f723::PERIPHERALS,
            super::stm32f723a0::PERIPHERALS,
            super::stm32f723a1::PERIPHERALS,
            super::stm32f723b0::PERIPHERALS,
            super::stm32f723ie::PERIPHERALS,
            super::stm32f723k6::PERIPHERALS,
            super::stm32f723rg::PERIPHERALS,
            super::stm32f723ze::PERIPHERALS,
            super::stm32f730::PERIPHERALS,
            super::stm32f732::PERIPHERALS,
            super::stm32f732a0::PERIPHERALS,
            super::stm32f732a1::PERIPHERALS,
            super::stm32f732b0::PERIPHERALS,
            super::stm32f732ie::PERIPHERALS,
            super::stm32f732k6::PERIPHERALS,
            super::stm32f732rg::PERIPHERALS,
            super::stm32f732vg::PERIPHERALS,
            super::stm32f745::PERIPHERALS,
            super::stm32f745a0::PERIPHERALS,
            super::stm32f745a1::PERIPHERALS,
            super::stm32f745b0::PERIPHERALS,
            super::stm32f745ie::PERIPHERALS,
            super::stm32f745k6::PERIPHERALS,
            super::stm32f745rg::PERIPHERALS,
            super::stm32f745vg::PERIPHERALS,
            super::stm32f746::PERIPHERALS,
            super::stm32f746a0::PERIPHERALS,
            super::stm32f746a1::PERIPHERALS,
            super::stm32f746b0::PERIPHERALS,
            super::stm32f746be::PERIPHERALS,
            super::stm32f746ig::PERIPHERALS,
            super::stm32f746k6::PERIPHERALS,
            super::stm32f746rg::PERIPHERALS,
            super::stm32f746zg::PERIPHERALS,
            super::stm32f756::PERIPHERALS,
            super::stm32f756a0::PERIPHERALS,
            super::stm32f756a1::PERIPHERALS,
            super::stm32f756b0::PERIPHERALS,
            super::stm32f756ig::PERIPHERALS,
            super::stm32f756k6::PERIPHERALS,
            super::stm32f756rg::PERIPHERALS,
            super::stm32f756zg::PERIPHERALS,
            super::stm32f765::PERIPHERALS,
            super::stm32f765a0::PERIPHERALS,
            super::stm32f765a1::PERIPHERALS,
            super::stm32f765b0::PERIPHERALS,
            super::stm32f765bi::PERIPHERALS,
            super::stm32f765ig::PERIPHERALS,
            super::stm32f765k6::PERIPHERALS,
            super::stm32f765rg::PERIPHERALS,
            super::stm32f767::PERIPHERALS,
            super::stm32f767a0::PERIPHERALS,
            super::stm32f767a1::PERIPHERALS,
            super::stm32f767b0::PERIPHERALS,
            super::stm32f767ig::PERIPHERALS,
            super::stm32f767k6::PERIPHERALS,
            super::stm32f767rg::PERIPHERALS,
            super::stm32f767zi::PERIPHERALS,
            super::stm32f769::PERIPHERALS,
            super::stm32f777::PERIPHERALS,
            super::stm32f777a0::PERIPHERALS,
            super::stm32f777a1::PERIPHERALS,
            super::stm32f777ai::PERIPHERALS,
            super::stm32f777b0::PERIPHERALS,
            super::stm32f777k6::PERIPHERALS,
            super::stm32f777rg::PERIPHERALS,
            super::stm32f777vg::PERIPHERALS,
            super::stm32g030::PERIPHERALS,
            super::stm32g030a0::PERIPHERALS,
            super::stm32g030a1::PERIPHERALS,
            super::stm32g030b0::PERIPHERALS,
            super::stm32g030c6::PERIPHERALS,
            super::stm32g030f6::PERIPHERALS,
            super::stm32g030g4::PERIPHERALS,
            super::stm32g030k6::PERIPHERALS,
            super::stm32g031::PERIPHERALS,
            super::stm32g031a0::PERIPHERALS,
            super::stm32g031a1::PERIPHERALS,
            super::stm32g031b0::PERIPHERALS,
            super::stm32g031c6::PERIPHERALS,
            super::stm32g031f8::PERIPHERALS,
            super::stm32g031g4::PERIPHERALS,
            super::stm32g031k8::PERIPHERALS,
            super::stm32g041::PERIPHERALS,
            super::stm32g050::PERIPHERALS,
            super::stm32g051::PERIPHERALS,
            super::stm32g061::PERIPHERALS,
            super::stm32g070::PERIPHERALS,
            super::stm32g070a0::PERIPHERALS,
            super::stm32g070a1::PERIPHERALS,
            super::stm32g070b0::PERIPHERALS,
            super::stm32g070cb::PERIPHERALS,
            super::stm32g070g4::PERIPHERALS,
            super::stm32g070rb::PERIPHERALS,
            super::stm32g070re::PERIPHERALS,
            super::stm32g071::PERIPHERALS,
            super::stm32g071a0::PERIPHERALS,
            super::stm32g071a1::PERIPHERALS,
            super::stm32g071b0::PERIPHERALS,
            super::stm32g071cb::PERIPHERALS,
            super::stm32g071g4::PERIPHERALS,
            super::stm32g071rb::PERIPHERALS,
            super::stm32g071re::PERIPHERALS,
            super::stm32g081::PERIPHERALS,
            super::stm32g0b1::PERIPHERALS,
            super::stm32g0c1::PERIPHERALS,
            super::stm32g431::PERIPHERALS,
            super::stm32g431rb::PERIPHERALS,
            super::stm32g441::PERIPHERALS,
            super::stm32g473::PERIPHERALS,
            super::stm32g474a0::PERIPHERALS,
            super::stm32g474a1::PERIPHERALS,
            super::stm32g474b0::PERIPHERALS,
            super::stm32g474ce::PERIPHERALS,
            super::stm32g474kb::PERIPHERALS,
            super::stm32g474re::PERIPHERALS,
            super::stm32g474rg::PERIPHERALS,
            super::stm32g474xx::PERIPHERALS,
            super::stm32g491::PERIPHERALS,
            super::stm32h503::PERIPHERALS,
            super::stm32h533::PERIPHERALS,
            super::stm32h562::PERIPHERALS,
            super::stm32h563::PERIPHERALS,
            super::stm32h723::PERIPHERALS,
            super::stm32h730::PERIPHERALS,
            super::stm32h735::PERIPHERALS,
            super::stm32h742::PERIPHERALS,
            super::stm32h743::PERIPHERALS,
            super::stm32h743a0::PERIPHERALS,
            super::stm32h743a1::PERIPHERALS,
            super::stm32h743ag::PERIPHERALS,
            super::stm32h743b0::PERIPHERALS,
            super::stm32h743ii::PERIPHERALS,
            super::stm32h743xg::PERIPHERALS,
            super::stm32h743xi::PERIPHERALS,
            super::stm32h743zi::PERIPHERALS,
            super::stm32h745::PERIPHERALS,
            super::stm32h747::PERIPHERALS,
            super::stm32h750::PERIPHERALS,
            super::stm32h753::PERIPHERALS,
            super::stm32h753a0::PERIPHERALS,
            super::stm32h753a1::PERIPHERALS,
            super::stm32h753ag::PERIPHERALS,
            super::stm32h753b0::PERIPHERALS,
            super::stm32h753ii::PERIPHERALS,
            super::stm32h753xg::PERIPHERALS,
            super::stm32h753xi::PERIPHERALS,
            super::stm32h755::PERIPHERALS,
            super::stm32h7a3::PERIPHERALS,
            super::stm32h7b0::PERIPHERALS,
            super::stm32h7b3::PERIPHERALS,
            super::stm32l010::PERIPHERALS,
            super::stm32l011::PERIPHERALS,
            super::stm32l021::PERIPHERALS,
            super::stm32l031::PERIPHERALS,
            super::stm32l041::PERIPHERALS,
            super::stm32l051::PERIPHERALS,
            super::stm32l052::PERIPHERALS,
            super::stm32l053::PERIPHERALS,
            super::stm32l062::PERIPHERALS,
            super::stm32l071::PERIPHERALS,
            super::stm32l073::PERIPHERALS,
            super::stm32l100::PERIPHERALS,
            super::stm32l151::PERIPHERALS,
            super::stm32l152::PERIPHERALS,
            super::stm32l162::PERIPHERALS,
            super::stm32l412::PERIPHERALS,
            super::stm32l431::PERIPHERALS,
            super::stm32l432::PERIPHERALS,
            super::stm32l433::PERIPHERALS,
            super::stm32l443::PERIPHERALS,
            super::stm32l452::PERIPHERALS,
            super::stm32l471::PERIPHERALS,
            super::stm32l476::PERIPHERALS,
            super::stm32l476a0::PERIPHERALS,
            super::stm32l476a1::PERIPHERALS,
            super::stm32l476b0::PERIPHERALS,
            super::stm32l476je::PERIPHERALS,
            super::stm32l476jg::PERIPHERALS,
            super::stm32l476ke::PERIPHERALS,
            super::stm32l476rg::PERIPHERALS,
            super::stm32l552::PERIPHERALS,
            super::stm32l562::PERIPHERALS,
            super::stm32mp157::PERIPHERALS,
            super::stm32u535::PERIPHERALS,
            super::stm32u575::PERIPHERALS,
            super::stm32u575a0::PERIPHERALS,
            super::stm32u575a1::PERIPHERALS,
            super::stm32u575b0::PERIPHERALS,
            super::stm32u585::PERIPHERALS,
            super::stm32u595::PERIPHERALS,
            super::stm32u5a5::PERIPHERALS,
            super::stm32u5f9::PERIPHERALS,
            super::stm32u5g9::PERIPHERALS,
            super::stm32wb55::PERIPHERALS,
            super::stm32wba52::PERIPHERALS,
            super::stm32wba55::PERIPHERALS,
            super::stm32wba55a0::PERIPHERALS,
            super::stm32wba55a1::PERIPHERALS,
            super::stm32wba55b0::PERIPHERALS,
            super::stm32wl33::PERIPHERALS,
            super::stm32wl55::PERIPHERALS,
            super::stm32wle4::PERIPHERALS,
            super::stm32wle5::PERIPHERALS,
        ];
        for p in tables.into_iter().flatten() {
            let mut spans: Vec<(u32, u32)> = Vec::new();
            for r in p.registers {
                let start = r.offset;
                let end = start + u32::from(r.width) / 8;
                assert!(
                    end <= p.window,
                    "{}:{} (offset 0x{:x}) exceeds window 0x{:x}",
                    p.name,
                    r.name,
                    r.offset,
                    p.window
                );
                assert!(end > start, "{}:{} is zero-width", p.name, r.name);
                for (prev_start, prev_end) in &spans {
                    // Input/output variants of a capture-compare register are
                    // the same physical word and may repeat the span exactly.
                    let exact_alias = start == *prev_start && end == *prev_end;
                    assert!(
                        exact_alias || end <= *prev_start || start >= *prev_end,
                        "{}:{} partially overlaps a previous register",
                        p.name,
                        r.name
                    );
                }
                spans.push((start, end));
                for f in r.fields {
                    assert!(
                        u32::from(f.offset) + u32::from(f.width) <= u32::from(r.width),
                        "{}:{}:{} exceeds register width",
                        p.name,
                        r.name,
                        f.name
                    );
                }
            }
        }
    }
}
