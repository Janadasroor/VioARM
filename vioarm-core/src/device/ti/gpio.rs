//! TI Tiva C GPIO ports (`GPIOA`-`GPIOF`, APB + AHB apertures).
//!
//! Register layout, resets, and commit rules follow the TM4C123GH6PM
//! datasheet (SPMS376E §10): `DATA` mask addressing from `ADDR[9:2]`
//! (writes touch unmasked output bits, reads mask to zero), `DIR`,
//! edge/level sense (`IS`/`IBE`/`IEV`) with `IM` masking and `ICR` W1C
//! clear, `AFSEL`/`PUR`/`PDR`/`DEN` commit gating through `LOCK`
//! (`0x4C4F_434B` unlocks; reads return locked/unlocked status) and `CR`
//! (Table 10-7 per-port resets: `PC[3:0]` JTAG/SWD committed AF,
//! `PD7`/`PF0` locked GPIO, everything else plain GPIO), plus `AMSEL`,
//! `PCTL`, drive-strength/open-drain/pull/slew latches.
//!
//! Both apertures (APB + AHB windows from the SVD) mount one shared port:
//! a single register set and pin core, like silicon after `GPIOHBCTL`
//! selects an aperture. The disabled-aperture fault is not modeled (both
//! apertures stay live; documented approximation). `DEN=0` or `AMSEL=1`
//! forces the digital read to 0; `AFSEL=1` leaves the pin state visible
//! (the owning peripheral drives it, same as a wired pin). Inputs without
//! a driver read 0; the host/test drives them with [`Gpio::drive_inputs`].
//! Edge latches evaluate on every pin/output change; level bits track live.

use crate::device::svd::{Device, PeripheralDef};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// `GPIOLOCK` unlock key.
const LOCK_KEY: u32 = 0x4C4F_434B;
/// `GPIOLOCK` read: locked.
const LOCK_LOCKED: u32 = 0x0000_0001;

/// Offsets shared by every GPIO port window.
const DATA_MASK_END: u32 = 0x400;
const DIR_OFF: u32 = 0x400;
const IS_OFF: u32 = 0x404;
const IBE_OFF: u32 = 0x408;
const IEV_OFF: u32 = 0x40C;
const IM_OFF: u32 = 0x410;
const RIS_OFF: u32 = 0x414;
const MIS_OFF: u32 = 0x418;
const ICR_OFF: u32 = 0x41C;
const AFSEL_OFF: u32 = 0x420;
const DR2R_OFF: u32 = 0x500;
const DR4R_OFF: u32 = 0x504;
const DR8R_OFF: u32 = 0x508;
const ODR_OFF: u32 = 0x50C;
const PUR_OFF: u32 = 0x510;
const PDR_OFF: u32 = 0x514;
const SLR_OFF: u32 = 0x518;
const DEN_OFF: u32 = 0x51C;
const LOCK_OFF: u32 = 0x520;
const CR_OFF: u32 = 0x524;
const AMSEL_OFF: u32 = 0x528;
const PCTL_OFF: u32 = 0x52C;
const ADCCTL_OFF: u32 = 0x530;
const DMACTL_OFF: u32 = 0x534;

/// Per-port reset words: `(port, AFSEL, PUR, DEN, PCTL, CR)`.
/// Table 10-7 (TM4C123GH6PM): `PC[3:0]` reset to JTAG/SWD (committed AF +
/// pull-up, `CR=0`), `PD7`/`PF0` reset to locked GPIO (`CR` bit clear),
/// all other pins to plain tri-stated GPIO.
const PORT_DEFAULTS_123: &[(&str, u32, u32, u32, u32, u32)] = &[
    ("GPIOA", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOB", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOC", 0x0F, 0x0F, 0x0F, 0x0000_1111, 0xF0),
    ("GPIOD", 0x00, 0x00, 0x00, 0x0000_0000, 0x7F),
    ("GPIOE", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOF", 0x00, 0x00, 0x00, 0x0000_0000, 0xFE),
];

/// Per-port reset words from Table 10-7 (TM4C1294NCPDT, SPMS433B):
/// `PC[3:0]` JTAG/SWD, `PD7` (NMI) and `PE7` locked GPIO, everything else
/// plain. Ports `A`–`J` are AHB-only, `K`–`Q` single-aperture (no
/// `GPIOHBCTL` on this family).
pub(crate) const PORT_DEFAULTS_129: &[(&str, u32, u32, u32, u32, u32)] = &[
    ("GPIOA", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOB", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOC", 0x0F, 0x0F, 0x0F, 0x0000_1111, 0xF0),
    ("GPIOD", 0x00, 0x00, 0x00, 0x0000_0000, 0x7F),
    ("GPIOE", 0x00, 0x00, 0x00, 0x0000_0000, 0x7F),
    ("GPIOF", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOG", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOH", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOJ", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOK", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOL", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOM", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPION", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOP", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
    ("GPIOQ", 0x00, 0x00, 0x00, 0x0000_0000, 0xFF),
];

/// Shared port state behind both apertures: one register set plus the pin
/// core. A single mutex guards the whole port, so either aperture sees the
/// same words and pins with no lock ordering to get wrong.
#[derive(Debug)]
pub struct GpioCore {
    regs: Vec<u32>,
    /// Output latch (the `DATA` word as written).
    out: u8,
    /// Externally driven input levels (host/test stimulus).
    pins: u8,
    /// Latched edge-detect interrupts (the `RIS` edge half).
    edge: u8,
    /// `GPIOCR` unlocked for commit writes.
    unlocked: bool,
}

/// Behavior behind one GPIO aperture window: address decode plus the port
/// register map over the shared [`GpioCore`].
pub struct GpioBehavior {
    def: &'static PeripheralDef,
    shared: Arc<Mutex<GpioCore>>,
    defaults: &'static [(&'static str, u32, u32, u32, u32, u32)],
}

impl GpioBehavior {
    fn reg_idx(&self, offset: u32) -> Option<usize> {
        self.def.registers.iter().position(|r| r.offset == offset)
    }

    fn get(&self, core: &GpioCore, offset: u32) -> u32 {
        self.reg_idx(offset).map_or(0, |i| core.regs[i])
    }

    fn set(&self, core: &mut GpioCore, offset: u32, value: u32) {
        if let Some(i) = self.reg_idx(offset) {
            core.regs[i] = value;
        }
    }

    /// Commit-gated write to `AFSEL`/`PUR`/`PDR`/`DEN`: only `CR`-set bits
    /// take the new value (other pins are hardwired-committable, i.e. their
    /// `CR` bit reads 1, so gating only bites on locked JTAG/NMI pins).
    fn commit_write(&self, core: &GpioCore, offset: u32, val: u8, shift: u32) -> u32 {
        let cur = self.get(core, offset);
        let mask = self.get(core, CR_OFF) & 0xFF;
        let lane = (u32::from(val) << shift) & 0xFF;
        let cur_lane = (cur >> shift) & 0xFF;
        let new_lane = (cur_lane & !mask) | (lane & mask);
        (cur & !(0xFF << shift)) | (new_lane << shift)
    }

    /// Sampled pin levels: driven outputs show the latch, inputs show the
    /// driven pins.
    fn levels(&self, core: &GpioCore) -> u8 {
        let dir = self.get(core, DIR_OFF) as u8;
        (core.out & dir) | (core.pins & !dir)
    }

    /// The `DATA` word firmware reads: levels gated by `DEN`/`AMSEL`.
    fn data_word(&self, core: &GpioCore) -> u8 {
        let den = self.get(core, DEN_OFF) as u8;
        let amsel = self.get(core, AMSEL_OFF) as u8;
        self.levels(core) & den & !amsel
    }

    /// Re-evaluate edge latches after a level change.
    fn poll_edges(&self, core: &mut GpioCore, before: u8, after: u8) {
        let is = self.get(core, IS_OFF) as u8;
        let ibe = self.get(core, IBE_OFF) as u8;
        let iev = self.get(core, IEV_OFF) as u8;
        let changed = before ^ after;
        for bit in 0..8 {
            let m = 1u8 << bit;
            if is & m != 0 {
                continue; // level-sensitive: tracked live, nothing latches.
            }
            if changed & m == 0 {
                continue;
            }
            let rising = after & m != 0;
            if ibe & m != 0 || rising == (iev & m != 0) {
                core.edge |= m;
            }
        }
    }

    /// Raw interrupt status: live level bits plus latched edge bits.
    fn ris(&self, core: &GpioCore) -> u8 {
        let levels = self.levels(core);
        let is = self.get(core, IS_OFF) as u8;
        let iev = self.get(core, IEV_OFF) as u8;
        let mut ris = core.edge & !is;
        for bit in 0..8 {
            let m = 1u8 << bit;
            if is & m != 0 && (levels & m != 0) == (iev & m != 0) {
                ris |= m;
            }
        }
        ris
    }

    fn stamp_defaults(&self, core: &mut GpioCore, port: &str) {
        let Some((_, afsel, pur, den, pctl, cr)) =
            self.defaults.iter().find(|(name, ..)| *name == port)
        else {
            return;
        };
        let (afsel, pur, den, pctl, cr) = (*afsel, *pur, *den, *pctl, *cr);
        for w in core.regs.iter_mut() {
            *w = 0;
        }
        core.out = 0;
        core.pins = 0;
        core.edge = 0;
        core.unlocked = false;
        self.set(core, AFSEL_OFF, afsel);
        self.set(core, PUR_OFF, pur);
        self.set(core, PDR_OFF, 0);
        self.set(core, DEN_OFF, den);
        self.set(core, PCTL_OFF, pctl);
        self.set(core, CR_OFF, cr);
        self.set(core, DR2R_OFF, 0xFF);
    }
}

/// A Tiva C GPIO port aperture (`GPIOA` APB or `GPIOA_AHB`, …).
pub struct Gpio {
    ranges: [Region; 1],
    exception: Option<u8>,
    port: &'static str,
    inner: Mutex<GpioBehavior>,
}

impl Gpio {
    /// A new aperture on `D`'s tables, sharing the port state in `shared`
    /// with its twin. `port` selects the Table 10-7 reset row.
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `aperture`, or the port has no reset row.
    #[must_use]
    pub fn for_aperture<D: Device>(
        aperture: &str,
        port: &'static str,
        shared: Arc<Mutex<GpioCore>>,
    ) -> Self {
        Self::for_aperture_with::<D>(aperture, port, shared, PORT_DEFAULTS_123)
    }

    /// A new aperture with an explicit Table 10-7 reset table (129-family
    /// ports use [`PORT_DEFAULTS_129`]).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `aperture`, or the port has no reset row.
    #[must_use]
    pub fn for_aperture_with<D: Device>(
        aperture: &str,
        port: &'static str,
        shared: Arc<Mutex<GpioCore>>,
        defaults: &'static [(&'static str, u32, u32, u32, u32, u32)],
    ) -> Self {
        assert!(
            defaults.iter().any(|(name, ..)| *name == port),
            "GPIO port {port} has no Table 10-7 reset row"
        );
        let def = D::peripheral(aperture).expect("generated tables contain GPIO aperture");
        let n = def.registers.len();
        {
            let mut core = shared.lock().unwrap_or_else(|e| e.into_inner());
            if core.regs.is_empty() {
                core.regs = vec![0; n];
            }
        }
        let mut gpio = Self {
            ranges: [Region::new(def.base, def.window)],
            exception: def.irqs.first().map(|irq| 16 + irq.number),
            port,
            inner: Mutex::new(GpioBehavior {
                def,
                shared: shared.clone(),
                defaults,
            }),
        };
        gpio.reset();
        gpio
    }

    /// A fresh shared port core for one port (both apertures take clones).
    /// Sized on first [`Gpio::for_aperture`] from the SVD register count.
    #[must_use]
    pub fn new_shared() -> Arc<Mutex<GpioCore>> {
        Arc::new(Mutex::new(GpioCore {
            regs: Vec::new(),
            out: 0,
            pins: 0,
            edge: 0,
            unlocked: false,
        }))
    }

    /// Drive the externally-visible input levels (host/test stimulus).
    /// Edge latches evaluate against the previous sample.
    pub fn drive_inputs(&self, levels: u8) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        let before = b.levels(&core);
        core.pins = levels;
        let after = b.levels(&core);
        b.poll_edges(&mut core, before, after);
    }

    fn offset_in_window(&self, addr: u32) -> Result<u32, BusError> {
        let region = self.ranges[0];
        addr.checked_sub(region.base)
            .filter(|o| *o < region.len)
            .ok_or(BusError::Reserved)
    }
}

impl Peripheral for Gpio {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let off = self.offset_in_window(addr)?;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        // `DATA` mask aliases: ADDR[9:2] selects readable bits.
        if off < DATA_MASK_END {
            if off % 4 != 0 {
                return Ok(0);
            }
            let mask = ((off >> 2) & 0xFF) as u8;
            return Ok(b.data_word(&core) & mask);
        }
        let lane = off % 4;
        let word_off = off - lane;
        let word = match word_off {
            RIS_OFF => u32::from(b.ris(&core)),
            MIS_OFF => u32::from(b.ris(&core)) & b.get(&core, IM_OFF),
            LOCK_OFF => {
                if core.unlocked {
                    0
                } else {
                    LOCK_LOCKED
                }
            }
            _ => {
                if b.reg_idx(word_off).is_none() {
                    return Err(BusError::Reserved);
                }
                b.get(&core, word_off)
            }
        };
        Ok(((word >> (8 * lane)) & 0xFF) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let off = self.offset_in_window(addr)?;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        if off < DATA_MASK_END {
            if off % 4 != 0 {
                return Ok(());
            }
            let mask = ((off >> 2) & 0xFF) as u8;
            let before = b.levels(&core);
            let dir = b.get(&core, DIR_OFF) as u8;
            core.out = (core.out & !(mask & dir)) | (val & mask & dir);
            let after = b.levels(&core);
            b.poll_edges(&mut core, before, after);
            return Ok(());
        }
        let lane = off % 4;
        let word_off = off - lane;
        let shift = 8 * lane;
        match word_off {
            ICR_OFF => {
                core.edge &= !val;
            }
            LOCK_OFF => {
                // Lane-wise key assembly: any non-key word relocks, the
                // exact key unlocks (firmware word-stores the key).
                let cur = b.get(&core, LOCK_OFF);
                let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
                b.set(&mut core, LOCK_OFF, word);
                core.unlocked = word == LOCK_KEY;
            }
            CR_OFF => {
                // `CR` commits only while unlocked; any `GPIOCR` write
                // relocks (datasheet GPIOLOCK page).
                if core.unlocked {
                    let cur = b.get(&core, CR_OFF);
                    b.set(
                        &mut core,
                        CR_OFF,
                        (cur & !(0xFF << shift)) | (u32::from(val) << shift),
                    );
                }
                core.unlocked = false;
            }
            AFSEL_OFF | PUR_OFF | PDR_OFF | DEN_OFF => {
                let word = b.commit_write(&core, word_off, val, shift);
                b.set(&mut core, word_off, word);
            }
            RIS_OFF | MIS_OFF => {}
            _ => {
                if b.reg_idx(word_off).is_none() {
                    return Err(BusError::Reserved);
                }
                let cur = b.get(&core, word_off);
                b.set(
                    &mut core,
                    word_off,
                    (cur & !(0xFF << shift)) | (u32::from(val) << shift),
                );
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        // SVD carries no resets: re-stamp the Table 10-7 port defaults.
        let port = self.port;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        b.stamp_defaults(&mut core, port);
    }

    fn tick(&mut self, _delta: u64) {}

    fn name(&self) -> &'static str {
        self.port
    }

    fn drive_inputs(&mut self, value: u32) {
        Gpio::drive_inputs(self, value as u8);
    }
}

impl InterruptSource for Gpio {
    fn exception_number(&self) -> Option<u8> {
        self.exception
    }

    fn pending(&self) -> u32 {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        u32::from(b.ris(&core) & (b.get(&core, IM_OFF) as u8) != 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm;

    const GPIOA_APB: u32 = 0x4000_4000;
    const GPIOA_AHB: u32 = 0x4005_8000;
    const GPIOC_APB: u32 = 0x4000_6000;

    fn port_a() -> (Gpio, Gpio) {
        let shared = Gpio::new_shared();
        (
            Gpio::for_aperture::<Tm4c123gh6pm>("GPIOA", "GPIOA", shared.clone()),
            Gpio::for_aperture::<Tm4c123gh6pm>("GPIOA_AHB", "GPIOA", shared),
        )
    }

    fn port_c() -> Gpio {
        Gpio::for_aperture::<Tm4c123gh6pm>("GPIOC", "GPIOC", Gpio::new_shared())
    }

    fn read_word(gpio: &mut Gpio, base: u32, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(gpio.read(base + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(gpio: &mut Gpio, base: u32, offset: u32, value: u32) {
        for lane in 0..4 {
            gpio.write(base + offset + lane, ((value >> (8 * lane)) & 0xFF) as u8)
                .unwrap();
        }
    }

    /// Table 10-7: plain ports reset to tri-stated GPIO, `PC[3:0]` to
    /// committed JTAG/SWD.
    #[test]
    fn table_10_7_resets() {
        let (mut a, _) = port_a();
        assert_eq!(read_word(&mut a, GPIOA_APB, AFSEL_OFF), 0);
        assert_eq!(read_word(&mut a, GPIOA_APB, DEN_OFF), 0);
        assert_eq!(read_word(&mut a, GPIOA_APB, CR_OFF), 0xFF);
        assert_eq!(read_word(&mut a, GPIOA_APB, LOCK_OFF), LOCK_LOCKED);
        let mut c = port_c();
        assert_eq!(read_word(&mut c, GPIOC_APB, AFSEL_OFF), 0x0F);
        assert_eq!(read_word(&mut c, GPIOC_APB, DEN_OFF), 0x0F);
        assert_eq!(read_word(&mut c, GPIOC_APB, PUR_OFF), 0x0F);
        assert_eq!(read_word(&mut c, GPIOC_APB, PCTL_OFF), 0x0000_1111);
        assert_eq!(read_word(&mut c, GPIOC_APB, CR_OFF), 0xF0);
    }

    /// Table 10-7 (TM4C1294NCPDT): same JTAG rows on port C, `PD7` (NMI)
    /// and `PE7` locked, late ports plain; AHB-only apertures serve.
    #[test]
    fn table_10_7_resets_129() {
        use crate::device::svd::tm4c1294ncpdt::Tm4c1294ncpdt;
        const GPIOC_AHB: u32 = 0x4005_A000;
        const GPIOD_AHB: u32 = 0x4005_B000;
        const GPIOE_AHB: u32 = 0x4005_C000;
        const GPIOQ: u32 = 0x4006_6000;
        fn port129(aperture: &str, port: &'static str) -> Gpio {
            Gpio::for_aperture_with::<Tm4c1294ncpdt>(
                aperture,
                port,
                Gpio::new_shared(),
                PORT_DEFAULTS_129,
            )
        }
        let mut c = port129("GPIOC_AHB", "GPIOC");
        assert_eq!(read_word(&mut c, GPIOC_AHB, AFSEL_OFF), 0x0F);
        assert_eq!(read_word(&mut c, GPIOC_AHB, DEN_OFF), 0x0F);
        assert_eq!(read_word(&mut c, GPIOC_AHB, CR_OFF), 0xF0);
        let mut d = port129("GPIOD_AHB", "GPIOD");
        assert_eq!(read_word(&mut d, GPIOD_AHB, CR_OFF), 0x7F);
        let mut e = port129("GPIOE_AHB", "GPIOE");
        assert_eq!(read_word(&mut e, GPIOE_AHB, CR_OFF), 0x7F);
        // Late single-aperture port: plain tri-state, unlocked.
        let mut q = port129("GPIOQ", "GPIOQ");
        assert_eq!(read_word(&mut q, GPIOQ, AFSEL_OFF), 0);
        assert_eq!(read_word(&mut q, GPIOQ, DEN_OFF), 0);
        assert_eq!(read_word(&mut q, GPIOQ, CR_OFF), 0xFF);
        // Drive + output round-trip on the late port.
        write_word(&mut q, GPIOQ, DIR_OFF, 0xFF);
        write_word(&mut q, GPIOQ, DEN_OFF, 0xFF);
        write_word(&mut q, GPIOQ, DATA_MASK_END - 4, 0xA5);
        assert_eq!(read_word(&mut q, GPIOQ, DATA_MASK_END - 4), 0xA5);
    }

    /// `DATA` mask addressing: writes touch unmasked output bits only,
    /// reads mask to zero.
    #[test]
    fn data_mask_addressing() {
        let (mut a, _) = port_a();
        // PA1 output + digital enable, PA0 stays input.
        write_word(&mut a, GPIOA_APB, DIR_OFF, 0x02);
        write_word(&mut a, GPIOA_APB, DEN_OFF, 0xFF);
        // Full-mask write sets PA1.
        a.write(GPIOA_APB + 0x3FC, 0xFF).unwrap();
        assert_eq!(a.read(GPIOA_APB + 0x3FC).unwrap(), 0x02);
        // Mask 0x02 alias reads only PA1; mask 0x01 reads 0 (input, undriven).
        assert_eq!(a.read(GPIOA_APB + (0x02 << 2)).unwrap(), 0x02);
        assert_eq!(a.read(GPIOA_APB + (0x01 << 2)).unwrap(), 0x00);
        // Masked write to PA0 (input) changes nothing.
        a.write(GPIOA_APB + (0x01 << 2), 0x01).unwrap();
        assert_eq!(a.read(GPIOA_APB + 0x3FC).unwrap(), 0x02);
    }

    /// Driven inputs read back through `DATA` once digitally enabled.
    #[test]
    fn driven_inputs_read_back() {
        let (mut a, _) = port_a();
        write_word(&mut a, GPIOA_APB, DEN_OFF, 0xFF);
        a.drive_inputs(0xA5);
        assert_eq!(a.read(GPIOA_APB + 0x3FC).unwrap(), 0xA5);
        // `DEN=0` forces the digital read to 0.
        write_word(&mut a, GPIOA_APB, DEN_OFF, 0x00);
        assert_eq!(a.read(GPIOA_APB + 0x3FC).unwrap(), 0x00);
    }

    /// `PC0` is commit-locked: `AFSEL` writes sink until unlock + `CR`.
    #[test]
    fn commit_lock_guards_jtag_pins() {
        let mut c = port_c();
        write_word(&mut c, GPIOC_APB, AFSEL_OFF, 0x00);
        assert_eq!(read_word(&mut c, GPIOC_APB, AFSEL_OFF), 0x0F);
        // Unlock, commit PC0, then the write lands.
        write_word(&mut c, GPIOC_APB, LOCK_OFF, LOCK_KEY);
        assert_eq!(read_word(&mut c, GPIOC_APB, LOCK_OFF), 0);
        write_word(&mut c, GPIOC_APB, CR_OFF, 0xF1);
        write_word(&mut c, GPIOC_APB, AFSEL_OFF, 0x00);
        assert_eq!(read_word(&mut c, GPIOC_APB, AFSEL_OFF) & 0x01, 0);
        // The `CR` write relocked.
        assert_eq!(read_word(&mut c, GPIOC_APB, LOCK_OFF), LOCK_LOCKED);
    }

    /// Both apertures share one port: APB writes show on AHB, including
    /// config words.
    #[test]
    fn apertures_share_state() {
        let (mut apb, mut ahb) = port_a();
        write_word(&mut apb, GPIOA_APB, DIR_OFF, 0xFF);
        write_word(&mut apb, GPIOA_APB, DEN_OFF, 0xFF);
        apb.write(GPIOA_APB + 0x3FC, 0x3C).unwrap();
        assert_eq!(ahb.read(GPIOA_AHB + 0x3FC).unwrap(), 0x3C);
        assert_eq!(read_word(&mut ahb, GPIOA_AHB, DIR_OFF), 0xFF);
    }

    /// Edge interrupt: rising on PA0 latches `RIS`, `IM` gates `MIS` and
    /// the NVIC line, `ICR` clears.
    #[test]
    fn edge_interrupt_flow() {
        let (mut a, _) = port_a();
        write_word(&mut a, GPIOA_APB, DEN_OFF, 0xFF);
        write_word(&mut a, GPIOA_APB, IEV_OFF, 0x01); // rising
        write_word(&mut a, GPIOA_APB, IM_OFF, 0x01);
        assert_eq!(a.pending(), 0);
        a.drive_inputs(0x01);
        assert_eq!(read_word(&mut a, GPIOA_APB, RIS_OFF) & 0x01, 0x01);
        assert_eq!(read_word(&mut a, GPIOA_APB, MIS_OFF) & 0x01, 0x01);
        assert_eq!(a.pending(), 1);
        assert_eq!(a.exception_number(), Some(16));
        a.write(GPIOA_APB + ICR_OFF, 0x01).unwrap();
        assert_eq!(a.pending(), 0);
    }

    /// Level interrupt tracks live while the level holds.
    #[test]
    fn level_interrupt_tracks_live() {
        let (mut a, _) = port_a();
        write_word(&mut a, GPIOA_APB, DEN_OFF, 0xFF);
        write_word(&mut a, GPIOA_APB, IS_OFF, 0x02); // level
        write_word(&mut a, GPIOA_APB, IEV_OFF, 0x02); // high
        write_word(&mut a, GPIOA_APB, IM_OFF, 0x02);
        a.drive_inputs(0x02);
        assert_eq!(a.pending(), 1);
        a.drive_inputs(0x00);
        assert_eq!(a.pending(), 0);
    }
}
