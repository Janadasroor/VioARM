//! The guest-visible memory interface.
//!
//! A `Bus` is what a CPU executes against. It is deliberately byte-granular:
//! multi-byte accesses must be implemented on top of it so alignment policy is
//! owned by one place per memory type. The CPU never knows whether it is
//! talking to RAM, flash, or a device; it only sees this trait.

use crate::error::BusError;

/// A region of the address space that a `Bus` can service.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Region {
    /// First address (inclusive).
    pub base: u32,
    /// Size in bytes.
    pub len: u32,
}

impl Region {
    #[must_use]
    pub const fn new(base: u32, len: u32) -> Self {
        Self { base, len }
    }

    /// Whether `addr` falls inside this region.
    #[must_use]
    pub const fn contains(&self, addr: u32) -> bool {
        let end = self.base.saturating_add(self.len);
        addr >= self.base && addr < end
    }
}

/// A `Bus` is the unit that services load/store requests from the CPU.
pub trait Bus {
    /// Read one byte.
    ///
    /// # Errors
    ///
    /// Returns `BusError::Reserved` if `addr` is not serviced by this bus.
    fn read(&mut self, addr: u32) -> Result<u8, BusError>;

    /// Write one byte.
    ///
    /// # Errors
    ///
    /// Returns `BusError::Reserved` if `addr` is not serviced by this bus.
    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError>;

    /// Read a little-endian halfword. Implemented as two byte reads so
    /// alignment is a property of the underlying memory, not this default.
    ///
    /// # Errors
    ///
    /// Returns `BusError::Reserved` if `addr` is not serviced by this bus.
    fn read_half(&mut self, addr: u32) -> Result<u16, BusError> {
        let lo = self.read(addr)?;
        let hi = self.read(addr.wrapping_add(1))?;
        Ok(u16::from_le_bytes([lo, hi]))
    }

    /// Write a little-endian halfword.
    ///
    /// # Errors
    ///
    /// Returns `BusError::Reserved` if `addr` is not serviced by this bus.
    fn write_half(&mut self, addr: u32, val: u16) -> Result<(), BusError> {
        let [lo, hi] = val.to_le_bytes();
        self.write(addr, lo)?;
        self.write(addr.wrapping_add(1), hi)
    }

    /// Read a little-endian word.
    ///
    /// # Errors
    ///
    /// Returns `BusError::Reserved` if `addr` is not serviced by this bus.
    fn read_word(&mut self, addr: u32) -> Result<u32, BusError> {
        let lo = self.read(addr)?;
        let hi = self.read(addr.wrapping_add(1))?;
        let b2 = self.read(addr.wrapping_add(2))?;
        let b3 = self.read(addr.wrapping_add(3))?;
        Ok(u32::from_le_bytes([lo, hi, b2, b3]))
    }

    /// Write a little-endian word.
    ///
    /// # Errors
    ///
    /// Returns `BusError::Reserved` if `addr` is not serviced by this bus.
    fn write_word(&mut self, addr: u32, val: u32) -> Result<(), BusError> {
        let [b0, b1, b2, b3] = val.to_le_bytes();
        self.write(addr, b0)?;
        self.write(addr.wrapping_add(1), b1)?;
        self.write(addr.wrapping_add(2), b2)?;
        self.write(addr.wrapping_add(3), b3)
    }
}

/// What the CPU is doing with an address, as far as MPU permission checks are
/// concerned: an instruction fetch, a data read, or a data write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MpuAccess {
    /// An instruction fetch.
    Fetch,
    /// A data read.
    Read,
    /// A data write.
    Write,
}

/// The `PMSAv8` permission bits, mirroring QEMU's `PAGE_*` protection flags.
pub const PROT_READ: u8 = 0x1;
/// The `PMSAv8` permission bits, mirroring QEMU's `PAGE_*` protection flags.
pub const PROT_WRITE: u8 = 0x2;
/// The `PMSAv8` permission bits, mirroring QEMU's `PAGE_*` protection flags.
pub const PROT_EXEC: u8 = 0x4;

/// The `PMSAv8` MPU permission result for one address, mirroring QEMU's
/// `pmsav8_mpu_lookup`: the raw permission bits plus the matched region.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MpuLookup {
    /// The `PROT_READ`/`PROT_WRITE`/`PROT_EXEC` permission bits.
    pub prot: u8,
    /// The matched MPU region number, or `None` when the default map decided
    /// (MPU disabled, `PRIVDEFENA` background, or a multiple-region hit).
    pub region: Option<u8>,
}

impl MpuLookup {
    /// The M-profile default system address map permission bits, from QEMU's
    /// `get_phys_addr_pmsav7_default` (`ROM`/`SRAM`/`RAM`: read/write/execute;
    /// peripheral/device/system: read/write).
    #[must_use]
    pub const fn background_prot(addr: u32) -> u8 {
        match addr {
            // ROM / SRAM / two RAM ranges.
            0x0000_0000..=0x1FFF_FFFF
            | 0x2000_0000..=0x3FFF_FFFF
            | 0x6000_0000..=0x7FFF_FFFF
            | 0x8000_0000..=0x9FFF_FFFF => PROT_READ | PROT_WRITE | PROT_EXEC,
            // Peripheral / Device / System.
            0x4000_0000..=0x5FFF_FFFF
            | 0xA000_0000..=0xBFFF_FFFF
            | 0xC000_0000..=0xDFFF_FFFF
            | 0xE000_0000..=0xFFFF_FFFF => PROT_READ | PROT_WRITE,
        }
    }
}

/// An IDAU's security attribution for one address — the implementation-defined
/// half of the ARMv8-M security attribution.
///
/// The core consults this through `PeripheralBus` for the `TT` instruction
/// family. A `None` (no IDAU) machine falls back to the `irvalid = false`
/// case.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IdauAttribution {
    /// The IDAU region number (the address `Addr[31:28]` on the ARMSSE).
    pub region: u8,
    /// Whether the region is attributed Non-secure.
    pub ns: bool,
    /// Whether the region is a Non-secure-callable region.
    pub nsc: bool,
    /// Whether the IDAU exempts the address from the SAU/IDAU attribution
    /// (the ARMSSE exempts its system-address aliases).
    pub exempt: bool,
}

/// The SAU's region match for one address — the architecturally-defined half
/// of the ARMv8-M security attribution, consulted when `SAU_CTRL.ENABLE == 1`
/// (the `TT` family's security lookup, mirroring QEMU's `v8m_security_lookup`
/// region loop).
///
/// The core folds `SAU_CTRL.ALLNS` and the IDAU upgrade around the result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SauHit {
    /// No enabled SAU region covers the address (the result is then the
    /// default, which the QEMU oracle leaves Secure).
    None,
    /// Exactly one enabled region covers the address; `index` is its region
    /// number and `nsc` whether `SAU_RLAR.NSC` marks it Non-secure-callable.
    Region { index: u8, nsc: bool },
    /// Two or more enabled regions overlap the address: the result collapses
    /// to Secure with no valid region number.
    Multiple,
}

/// A `Bus` whose machine can also advance time and deliver interrupts.
///
/// Plain memory implements the default no-ops; a `SystemMemory` with an NVIC
/// and timer peripherals implements the real behavior. The interrupt
/// handshake is two-phase: `next_exception` *peeks* (side-effect free, may be
/// called repeatedly) and `acknowledge_exception` *consumes* after the core
/// commits to taking the exception.
pub trait PeripheralBus: Bus {
    /// Peek: the lowest-numbered exception that should be taken right now,
    /// honoring the core's mask (`PRIMASK`/`FAULTMASK`/`BASEPRI`) and current
    /// exception priority, or `None`. Must not consume.
    fn next_exception(
        &mut self,
        primask: bool,
        faultmask: bool,
        basepri: u8,
        current_ipsr: u8,
    ) -> Option<u8>;

    /// Consume: the core is taking exception `n`; drop the corresponding
    /// pending latch(es).
    fn acknowledge_exception(&mut self, n: u8);

    /// Advance all peripherals by one machine cycle.
    fn tick_cycle(&mut self) {}

    /// Reset all peripheral state.
    fn reset_peripherals(&mut self) {}

    /// Whether the system control block has latched `AIRCR.SYSRESETREQ`; the
    /// machine performs a full reset when set. Default: no such request.
    fn reset_requested(&self) -> bool {
        false
    }

    /// Record escalated-fault status into the SCB's `CFSR`/`HFSR` (the bits to
    /// OR in). The core calls this when it folds a fault into HardFault, so
    /// firmware can read the cause back. Default: no recording (plain memory).
    fn record_fault(&mut self, _cfsr_bits: u32, _hfsr_bits: u32) {}

    /// Whether the memory-protection unit (if any) permits the core's `access`
    /// to `addr` in the given privilege and exception context. `privileged` is
    /// whether the core is executing privileged; `exception` is the current
    /// exception number (0 in Thread mode) so the NMI/HardFault bypass can be
    /// honored. The core calls this before every instruction fetch and data
    /// access. Default: always permitted (plain memory has no MPU).
    fn mpu_check(
        &mut self,
        _addr: u32,
        _access: MpuAccess,
        _privileged: bool,
        _exception: u8,
    ) -> bool {
        true
    }

    /// Record an MPU access violation at `addr` (an instruction fetch when
    /// `fetch` is true), so firmware reads the cause back through `MMFAR` and
    /// the `MMFSR` bits in `CFSR`. Default: no recording (plain memory).
    fn mpu_fault(&mut self, _addr: u32, _fetch: bool) {}

    /// The IDAU's security attribution for `addr`, or `None` when the machine
    /// has no IDAU. The core consults this for the `TT` instruction family's
    /// security lookup. Default: no IDAU.
    fn idau_attribution(&mut self, _addr: u32) -> Option<IdauAttribution> {
        None
    }

    /// The SAU's region match for `addr` (see [`SauHit`]), consulted for the
    /// `TT` security lookup when `SAU_CTRL.ENABLE == 1`. Default: no SAU.
    fn sau_hit(&mut self, _addr: u32) -> SauHit {
        SauHit::None
    }

    /// The `PMSAv8` MPU permission lookup for `addr` in the `secure` register
    /// bank and `privileged` mode (QEMU's `pmsav8_mpu_lookup`, probed with a
    /// read), for the `TT` instruction family's permission bits. Default (no
    /// `PMSAv8` MPU): the M-profile default map with no matched region.
    fn mpu_lookup_v8(&mut self, addr: u32, _secure: bool, _privileged: bool) -> MpuLookup {
        MpuLookup {
            prot: MpuLookup::background_prot(addr),
            region: None,
        }
    }

    /// Mirror the core's current security state into a device that banks its
    /// registers on it (the `v8-M` MPU). Default: nothing.
    fn set_secure(&mut self, _secure: bool) {}
}
