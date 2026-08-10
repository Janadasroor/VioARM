//! Core-wide error types shared by the whole simulator.

use core::fmt;

/// Why the machine stopped. Every halt is a typed, analyzable cause; there is
/// no silent stop.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum HaltCause {
    /// An instruction that is not implemented by this simulator configuration
    /// was fetched (or a genuinely unknown opcode).
    UndefinedInstruction,
    /// `bl`/`b`/`bx`/`pop` computed a target with bit 0 == 0: not executing
    /// in Thumb mode.
    NonThumbTarget,
    /// The fetched halfword is an unaligned or invalid program address.
    FetchAbort,
    /// The CPU executed a `bkpt` instruction.
    Breakpoint,
    /// The CPU attempted to take an exception the simulator does not model,
    /// or executed a system instruction it cannot service.
    UnhandledException,
    /// The cycle budget passed to `Core::run` ran out before the machine
    /// halted of its own accord. This is a host-side limit, not a machine
    /// state.
    CycleBudgetExhausted,
    /// A data access fault: reserved address, unaligned access to a device
    /// that requires alignment, etc.
    BusFault { address: u32 },
    /// The stack-pointer ended up misaligned at a point where the machine
    /// requires a word-aligned SP (exceptions, `pop {pc}`). Mirrors a
    /// HardFault-with-UNALIGNED condition.
    StackMisaligned { address: u32 },
    /// A load returned an invalid PC-like value; e.g. `bx` to an even
    /// address in an ARMv6-M system that is required to run Thumb.
    NotImplemented { what: &'static str },
}

impl fmt::Display for HaltCause {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UndefinedInstruction => f.write_str("undefined instruction"),
            Self::NonThumbTarget => f.write_str("non-thumb branch target"),
            Self::FetchAbort => f.write_str("instruction fetch abort"),
            Self::Breakpoint => f.write_str("breakpoint"),
            Self::CycleBudgetExhausted => f.write_str("cycle budget exhausted"),
            Self::UnhandledException => f.write_str("unsupported system exception"),
            Self::BusFault { address } => write!(f, "bus fault at {address:#010x}"),
            Self::StackMisaligned { address } => {
                write!(f, "misaligned stack at {address:#010x}")
            }
            Self::NotImplemented { what } => write!(f, "not implemented: {what}"),
        }
    }
}

/// An ARMv6-M exception the core can take and return from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Exception {
    /// Exception number 3: the only fault ARMv6-M software can reach without
    /// a peripheral-provided interrupt.
    HardFault,
    /// Exception number 11: the supervisor call, taken by executing `svc`.
    /// Its vector is read from the same table as the other exceptions.
    SvCall,
    /// Exception number 14: the pendable service call, the RTOS context-switch
    /// hook. Pended through `ICSR.PENDSVSET`.
    PendSV,
    /// Exception number 15: the system tick, driven by the `SysTick`
    /// peripheral.
    SysTick,
    /// An external interrupt: exception number `16 + n` for IRQ `n` (0..=31).
    Irq(u8),
}

impl Exception {
    /// The exception number used for IPSR and as the vector-table index.
    #[must_use]
    pub const fn number(self) -> u8 {
        match self {
            Self::HardFault => 3,
            Self::SvCall => 11,
            Self::PendSV => 14,
            Self::SysTick => 15,
            Self::Irq(n) => 16 + n,
        }
    }
}

/// An access to the memory system that cannot be satisfied.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BusError {
    Reserved,
    Unaligned { width: u32 },
}

impl fmt::Display for BusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Reserved => f.write_str("access to reserved address"),
            Self::Unaligned { width } => {
                write!(f, "unaligned access of width {width}")
            }
        }
    }
}

/// Everything that can fail while turning bytes into a loadable `Image`.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum ImageError {
    Io(String),
    Truncated,
    BadMagic,
    UnsupportedElfClass,
    UnsupportedMachine,
    NoLoadableSegments,
    SegmentOutOfMemory,
    UnsupportedBss,
    /// A record that is not valid Intel HEX syntax (bad characters, missing
    /// `:`, wrong length).
    HexSyntax,
    /// A record whose checksum does not sum to zero mod 256.
    HexChecksum,
    /// A data record whose address overflows 32 bits once its bytes are
    /// placed.
    HexAddressOverflow,
}

impl fmt::Display for ImageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(why) => write!(f, "I/O error: {why}"),
            Self::Truncated => f.write_str("truncated image"),
            Self::BadMagic => f.write_str("bad magic number"),
            Self::UnsupportedElfClass => f.write_str("unsupported ELF class"),
            Self::UnsupportedMachine => f.write_str("unsupported ELF machine"),
            Self::NoLoadableSegments => f.write_str("no loadable segments"),
            Self::SegmentOutOfMemory => f.write_str("segment does not fit in memory"),
            Self::UnsupportedBss => f.write_str(".bss zero-padding not supported"),
            Self::HexSyntax => f.write_str("malformed Intel HEX record"),
            Self::HexChecksum => f.write_str("Intel HEX checksum mismatch"),
            Self::HexAddressOverflow => f.write_str("Intel HEX address overflow"),
        }
    }
}
