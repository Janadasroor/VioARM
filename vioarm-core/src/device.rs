//! Peripheral devices that sit on the bus.
//!
//! A `Peripheral` owns one or more address windows and answers byte-granular
//! bus requests, just like a `Bus`. It differs from a passive memory region in
//! two ways: it advances in simulated time (`tick`) and it can raise
//! interrupts. A device-driven memory dispatch window routes address windows
//! to devices; the CPU never talks to devices directly.
//!
//! All devices are "lazy": the machine only calls `tick` at synchronization
//! points with the true elapsed delta since the device last ran, so devices
//! that are idle cost the host nothing.

use crate::error::BusError;
use crate::mem::bus::Region;

pub mod gpio;
pub mod nvic;
pub mod scb;
pub mod stm32;
pub mod svd;
pub mod systick;
pub mod uart;

/// A time-advancing, addressable source of state on the memory bus.
///
/// Reads are `&mut self` by construction: on real silicon a read commonly has
/// side effects (read-to-clear status flags, FIFO pops), and modeling that as
/// the default keeps firmware behavior honest.
pub trait Peripheral: Send {
    /// The address windows this device services, in map order. Windows are
    /// kept small and disjoint so a linear scan by the dispatch bus is cheap.
    fn ranges(&self) -> &[Region];

    /// Whether clock to this device is currently gated off. A gated device is
    /// never `tick`ed and answers bus accesses as `Reserved`, like a part
    /// whose clock is off.
    fn gated(&self) -> bool {
        false
    }

    /// Handle a byte read inside one of `ranges()`. May have side effects.
    ///
    /// # Errors
    /// `BusError::Reserved` for an address the device does not service.
    fn read(&mut self, addr: u32) -> Result<u8, BusError>;

    /// Handle a byte write inside one of `ranges()`.
    ///
    /// # Errors
    /// `BusError::Reserved` for an address the device does not service.
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError>;

    /// Restore the post-reset state. The no-op default suits stateless
    /// devices, or those whose registers are all reset by a bus write.
    fn reset(&mut self) {}

    /// Advance simulated time by `delta` cycles. Called exactly once per
    /// synchronization point with the true delta since the last `tick`, and
    /// never while the device is `gated()`.
    fn tick(&mut self, _delta: u64) {}

    /// A short diagnostic name for this device.
    fn name(&self) -> &'static str {
        ""
    }

    /// Host-side output channel: bytes this device emitted since the last
    /// drain (e.g. UART transmissions). The machine never touches this; tools
    /// (CLI, tests, a future debugger) call it to observe what the firmware
    /// sent. The no-op default lets non-emitting devices opt out.
    fn drain_output(&mut self) -> Vec<u8> {
        Vec::new()
    }

    /// Host stimulus: set the device's externally-visible input level to
    /// `value` (e.g. GPIO input pins, future UART RX). Delivered through the
    /// same channel so a debugger can inject signals between steps. The no-op
    /// default lets devices without host inputs opt out.
    fn drive_inputs(&mut self, _value: u32) {}
}

/// A device that raises one or more interrupt lines observed by the NVIC.
///
/// The handshake is deterministic and strictly two-phase, mirroring the
/// recommended hardware protocol:
///
/// 1. **peek** — `pending()` reports which lines are latched and must have no
///    side effects; the NVIC may call it repeatedly while arbitrating.
/// 2. **take** — after the NVIC decides which line to service (lowest vector
///    number wins on ARMv6-M), `acknowledge()` clears that source's latch so
///    the identical request is not delivered twice.
pub trait InterruptSource: Peripheral {
    /// The ARMv6-M exception number this device raises when its line is
    /// pending: 15 for the system tick, 16..=47 for external IRQ 0..=31.
    /// `None` (the default) means the device never interrupts.
    fn exception_number(&self) -> Option<u8> {
        None
    }

    /// Bit `L` set == interrupt line `L` is latched pending right now. The
    /// default of "no lines" lets a purely memory-mapped device opt out.
    fn pending(&self) -> u32 {
        0
    }

    /// Acknowledge (clear) the latched `lines` the NVIC is taking. Lines the
    /// NVIC is not taking stay pending. The default clears nothing.
    fn acknowledge(&mut self, _lines: u32) {}

    /// Which device-local lines the device can raise, as a bitmask of line
    /// indices (bit `L` set == line `L` exists). A device that raises one NVIC
    /// line keeps the default of a single line; a controller that raises one
    /// NVIC line per channel (the F1 DMA) reports its full set so the NVIC
    /// acknowledges the right line when it takes an exception.
    fn lines(&self) -> u32 {
        1
    }

    /// The exception number raised when device-local `line` is taken.
    ///
    /// Most devices raise one line, so the default maps `line` onto
    /// `exception_number() + line`. A controller whose lines are *not*
    /// consecutive — the F4 DMA1 puts streams 0..=6 on IRQ 11..=17 but stream
    /// 7 on IRQ 47 — overrides this per line.
    fn exception_for_line(&self, line: u32) -> Option<u8> {
        self.exception_number()
            .map(|base| base.wrapping_add(u8::try_from(line).unwrap_or(u8::MAX)))
    }
}

/// The lowest set bit index of `lines` — the NVIC's fixed-priority
/// arbitration picks the vector with the lowest number.
#[must_use]
pub const fn lowest_pending(lines: u32) -> Option<u32> {
    if lines == 0 {
        None
    } else {
        Some(lines.trailing_zeros())
    }
}

/// A device that reads as zero and discards writes: the "no peripheral
/// implemented" standby, mirroring zero-writes-ignored windows on many parts.
#[derive(Clone, Copy, Debug, Default)]
pub struct NullDevice {
    ranges: [Region; 1],
}

impl NullDevice {
    #[must_use]
    pub const fn new(region: Region) -> Self {
        Self { ranges: [region] }
    }
}

impl Peripheral for NullDevice {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, _addr: u32) -> Result<u8, BusError> {
        Ok(0)
    }

    fn write(&mut self, _addr: u32, _val: u8) -> Result<(), BusError> {
        Ok(())
    }

    fn name(&self) -> &'static str {
        "null"
    }
}

impl InterruptSource for NullDevice {}

#[cfg(test)]
mod tests {
    use super::lowest_pending;

    #[test]
    fn lowest_pending_selects_min_line() {
        assert_eq!(lowest_pending(0), None);
        assert_eq!(lowest_pending(1 << 3), Some(3));
        assert_eq!(lowest_pending(0x8000_0000), Some(31));
        assert_eq!(lowest_pending(0b1010), Some(1));
    }
}
