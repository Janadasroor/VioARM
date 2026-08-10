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

pub mod stm32f103;
pub mod stm32f107;
pub mod stm32f407;

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
    pub offset: u16,
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
    const fn new(def: &'static PeripheralDef, words: W) -> Self {
        Self { def, words }
    }

    fn index(&self, offset: u16) -> usize {
        self.def
            .registers
            .iter()
            .position(|r| r.offset == offset)
            .unwrap_or_else(|| panic!("no register at offset 0x{offset:x}"))
    }

    /// The stored word at register `offset`.
    #[must_use]
    pub fn get(&self, offset: u16) -> u32 {
        self.words[self.index(offset)]
    }
}

impl Registers<&mut [u32]> {
    /// Replace the stored word at register `offset`.
    pub fn set(&mut self, offset: u16, value: u32) {
        let i = self.index(offset);
        self.words[i] = value;
    }

    /// The stored word at `offset` with byte `val` merged into lane `shift` —
    /// the byte-lane read-modify-write a plain register write stores.
    #[must_use]
    pub fn merged(&self, offset: u16, val: u8, shift: u32) -> u32 {
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
        offset: u16,
        val: u8,
        shift: u32,
    ) -> u32 {
        view.merged(offset, val, shift)
    }

    /// Transform a register value before firmware reads it.
    ///
    /// The view is mutable because reads have side effects on silicon: USART
    /// reading `DR` acknowledges `RXNE`.
    fn on_read(&mut self, _view: &mut Registers<&mut [u32]>, _offset: u16, value: u32) -> u32 {
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
}

impl<B: Behavior> RegisterFile<B> {
    /// The raw stored word at register `offset`, for host inspection. Bypasses
    /// the behavior so it observes the true latch (say, the GPIO output
    /// register) rather than a synthesized read.
    ///
    /// # Panics
    /// Panics if `offset` is not one of the table's registers.
    #[must_use]
    pub fn word(&self, offset: u16) -> u32 {
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
                off >= u32::from(r.offset) && off < u32::from(r.offset) + len
            })
            .ok_or(BusError::Reserved)?;
        Ok((&self.def.registers[idx], idx))
    }

    fn byte_shift(reg: &RegisterDef, off: u32) -> u32 {
        8 * (off - u32::from(reg.offset))
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
        use crate::device::stm32::usart::Usart1;
        use crate::device::{InterruptSource, Peripheral};

        // USART1: the byte-oriented behavior on F107's tables.
        let mut u = Usart1::for_device::<stm32f107::Stm32f107>();
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
        use crate::device::stm32::usart::Usart1;
        use crate::device::{InterruptSource, Peripheral};

        // USART1: the byte-oriented behavior on F407's tables (same 7-register
        // layout as F1, different base).
        let mut u = Usart1::for_device::<stm32f407::Stm32f407>();
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
    }

    #[test]
    fn generated_tables_are_internally_consistent() {
        let tables = [
            super::stm32f103::PERIPHERALS,
            super::stm32f107::PERIPHERALS,
            super::stm32f407::PERIPHERALS,
        ];
        for p in tables.into_iter().flatten() {
            let mut spans: Vec<(u32, u32)> = Vec::new();
            for r in p.registers {
                let start = u32::from(r.offset);
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
