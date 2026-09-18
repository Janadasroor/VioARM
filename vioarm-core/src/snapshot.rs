//! Opaque CPU+RAM snapshots for embedding hosts (WP-R7).
//!
//! [`Machine::snapshot_save`](crate::machine::Machine::snapshot_save) freezes
//! the integer CPU state plus one RAM span into a versioned little-endian
//! buffer; `snapshot_restore` puts it back. The RAM span is exactly the
//! `record --snap` default (`0x2000_0000:0x1000`), so snapshots and goldens
//! agree on what "RAM" means.
//!
//! Explicitly NOT covered (documented limitation): FPU/VFP banks, ARMv8-M
//! security banks, the exclusive monitor, NVIC pending state, peripheral and
//! device state, semihosting/RTT buffers, the backtrace ring, and the
//! pipeline load-use tracker (timing-only: at most one `cyccnt` stall on the
//! first step after restore). Snapshots rewind short deterministic CPU+RAM
//! sections; they are not full-machine checkpoints.

use crate::board::BoardMemory;
use crate::machine::Machine;
use crate::mem::Bus;

/// First RAM 4 KiB — the `record --snap` default span.
pub const SNAPSHOT_RAM_BASE: u32 = 0x2000_0000;
/// Length of the snapshotted RAM span in bytes.
pub const SNAPSHOT_RAM_LEN: usize = 0x1000;
/// Snapshot format magic (`b"VASN"`, little-endian).
pub const SNAPSHOT_MAGIC: u32 = 0x4E_53_41_56;
/// Snapshot format version (bumped on any layout change).
pub const SNAPSHOT_VERSION: u32 = 2;
/// API major the snapshot was frozen under (stamped at save, gated at
/// restore): the workspace versions every crate in lockstep, so this is
/// both the core and the embedding-ABI major. Layout-compatible lines
/// share it; a different major fails closed even when the layout version
/// matches, because out-of-line semantics may have moved.
pub const SNAPSHOT_API_MAJOR: u32 = crate_api_major();
/// API minor at save time (diagnostic only — never gated: minor versions
/// promise no layout change).
pub const SNAPSHOT_API_MINOR: u32 = crate_api_minor();
/// API patch at save time (diagnostic only, same promise as minor).
pub const SNAPSHOT_API_PATCH: u32 = crate_api_patch();

/// Crate version triple backing the API stamp above.
const fn crate_api_major() -> u32 {
    parse_version(env!("CARGO_PKG_VERSION_MAJOR"))
}

/// Crate version triple backing the API stamp above.
const fn crate_api_minor() -> u32 {
    parse_version(env!("CARGO_PKG_VERSION_MINOR"))
}

/// Crate version triple backing the API stamp above.
const fn crate_api_patch() -> u32 {
    parse_version(env!("CARGO_PKG_VERSION_PATCH"))
}

/// Parse a decimal `CARGO_PKG_VERSION_*` component at compile time (workspace
/// versions are small integers; anything unparseable becomes 0 rather than
/// failing the build).
const fn parse_version(component: &str) -> u32 {
    let bytes = component.as_bytes();
    let mut value = 0u32;
    let mut i = 0;
    while i < bytes.len() {
        let digit = bytes[i].wrapping_sub(b'0');
        if digit > 9 {
            return 0;
        }
        value = value * 10 + digit as u32;
        i += 1;
    }
    value
}
/// Header bytes before the RAM image (magic + version + regs + xpsr +
/// control block + counters + span descriptor + API stamp).
pub const SNAPSHOT_HEADER_LEN: usize = 152;
/// Total snapshot bytes (header + RAM span).
pub const SNAPSHOT_LEN: usize = SNAPSHOT_HEADER_LEN + SNAPSHOT_RAM_LEN;

/// A snapshot save/restore failure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SnapshotError {
    /// Caller buffer smaller than [`SNAPSHOT_LEN`].
    BufferTooSmall {
        /// Bytes the caller offered.
        got: usize,
    },
    /// Buffer is not a snapshot (short, bad magic, or wrong total length).
    BadImage(String),
    /// Snapshot version newer than this build understands.
    BadVersion(u32),
    /// Snapshot frozen under a different API major (layout may match, but
    /// out-of-line semantics may have moved — fails closed).
    BadApiVersion {
        /// Stamped major.
        got: u32,
        /// This build's major.
        want: u32,
    },
    /// A RAM span byte is not serviced by this board's bus.
    MemoryOutOfRange {
        /// Faulting address.
        addr: u32,
    },
}

impl std::fmt::Display for SnapshotError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BufferTooSmall { got } => {
                write!(f, "snapshot needs {SNAPSHOT_LEN} bytes, got {got}")
            }
            Self::BadImage(why) => write!(f, "bad snapshot image: {why}"),
            Self::BadVersion(v) => write!(f, "snapshot version {v} unsupported"),
            Self::BadApiVersion { got, want } => {
                write!(f, "snapshot api major {got} unsupported (want {want})")
            }
            Self::MemoryOutOfRange { addr } => {
                write!(f, "snapshot RAM {addr:#010x} not serviced by this board")
            }
        }
    }
}

fn put_u32(out: &mut [u8], off: usize, v: u32) {
    out[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

fn put_u64(out: &mut [u8], off: usize, v: u64) {
    out[off..off + 8].copy_from_slice(&v.to_le_bytes());
}

fn get_u32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

fn get_u64(buf: &[u8], off: usize) -> u64 {
    u64::from_le_bytes([
        buf[off],
        buf[off + 1],
        buf[off + 2],
        buf[off + 3],
        buf[off + 4],
        buf[off + 5],
        buf[off + 6],
        buf[off + 7],
    ])
}

impl Machine<BoardMemory> {
    /// Snapshot byte size (constant: [`SNAPSHOT_LEN`]).
    #[must_use]
    pub const fn snapshot_len(&self) -> usize {
        SNAPSHOT_LEN
    }

    /// Freeze integer CPU state + the [`SNAPSHOT_RAM_BASE`] span into `out`.
    /// Returns bytes written ([`SNAPSHOT_LEN`]).
    ///
    /// # Errors
    ///
    /// [`SnapshotError::BufferTooSmall`] when `out` is short, or
    /// [`SnapshotError::MemoryOutOfRange`] when the board does not service
    /// the span (nothing is written then — validation reads first).
    pub fn snapshot_save(&mut self, out: &mut [u8]) -> Result<usize, SnapshotError> {
        if out.len() < SNAPSHOT_LEN {
            return Err(SnapshotError::BufferTooSmall { got: out.len() });
        }
        // Validate the span before writing anything.
        let mut ram = [0u8; SNAPSHOT_RAM_LEN];
        for (i, byte) in ram.iter_mut().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let addr = SNAPSHOT_RAM_BASE.wrapping_add(i as u32);
            *byte = self
                .core
                .mem()
                .read(addr)
                .map_err(|_| SnapshotError::MemoryOutOfRange { addr })?;
        }
        put_u32(out, 0, SNAPSHOT_MAGIC);
        put_u32(out, 4, SNAPSHOT_VERSION);
        for (i, r) in self.core.regs.r.iter().enumerate() {
            put_u32(out, 8 + 4 * i, *r);
        }
        put_u32(out, 60, self.core.regs.sp);
        put_u32(out, 64, self.core.regs.msp);
        put_u32(out, 68, self.core.regs.psp);
        put_u32(out, 72, self.core.regs.lr);
        put_u32(out, 76, self.core.regs.pc);
        put_u32(out, 80, self.core.xpsr.get());
        put_u32(out, 84, self.core.control);
        put_u32(out, 88, u32::from(self.core.primask));
        put_u32(out, 92, u32::from(self.core.basepri));
        put_u32(out, 96, u32::from(self.core.faultmask));
        put_u32(out, 100, u32::from(self.core.ipsr));
        put_u32(out, 104, u32::from(self.core.itstate));
        put_u32(out, 108, u32::from(self.core.sleeping));
        put_u32(out, 112, u32::from(self.core.event));
        put_u64(out, 116, self.core.cycles);
        put_u64(out, 124, self.core.cyccnt);
        put_u32(out, 132, SNAPSHOT_RAM_BASE);
        put_u32(out, 136, SNAPSHOT_RAM_LEN as u32);
        put_u32(out, 140, SNAPSHOT_API_MAJOR);
        put_u32(out, 144, SNAPSHOT_API_MINOR);
        put_u32(out, 148, SNAPSHOT_API_PATCH);
        out[SNAPSHOT_HEADER_LEN..SNAPSHOT_LEN].copy_from_slice(&ram);
        Ok(SNAPSHOT_LEN)
    }

    /// Restore state frozen by [`Self::snapshot_save`]. Validation
    /// (magic, version, API major, exact length) runs before anything is
    /// touched, so a rejected buffer leaves the machine exactly as it was.
    ///
    /// # Errors
    ///
    /// [`SnapshotError::BadImage`] / [`SnapshotError::BadVersion`] /
    /// [`SnapshotError::BadApiVersion`] /
    /// [`SnapshotError::MemoryOutOfRange`] (the last only after the CPU
    /// state is already restored — on serviced boards it cannot happen).
    pub fn snapshot_restore(&mut self, buf: &[u8]) -> Result<(), SnapshotError> {
        if buf.len() != SNAPSHOT_LEN {
            return Err(SnapshotError::BadImage(format!(
                "want exactly {SNAPSHOT_LEN} bytes, got {}",
                buf.len()
            )));
        }
        if get_u32(buf, 0) != SNAPSHOT_MAGIC {
            return Err(SnapshotError::BadImage("bad magic".to_owned()));
        }
        let version = get_u32(buf, 4);
        if version != SNAPSHOT_VERSION {
            return Err(SnapshotError::BadVersion(version));
        }
        let api_major = get_u32(buf, 140);
        if api_major != SNAPSHOT_API_MAJOR {
            return Err(SnapshotError::BadApiVersion {
                got: api_major,
                want: SNAPSHOT_API_MAJOR,
            });
        }
        if get_u32(buf, 132) != SNAPSHOT_RAM_BASE || get_u32(buf, 136) != SNAPSHOT_RAM_LEN as u32 {
            return Err(SnapshotError::BadImage("span mismatch".to_owned()));
        }
        for (i, r) in self.core.regs.r.iter_mut().enumerate() {
            *r = get_u32(buf, 8 + 4 * i);
        }
        self.core.regs.sp = get_u32(buf, 60);
        self.core.regs.msp = get_u32(buf, 64);
        self.core.regs.psp = get_u32(buf, 68);
        self.core.regs.lr = get_u32(buf, 72);
        self.core.regs.pc = get_u32(buf, 76);
        self.core.xpsr.set(get_u32(buf, 80));
        self.core.control = get_u32(buf, 84);
        self.core.primask = get_u32(buf, 88) != 0;
        #[allow(clippy::cast_possible_truncation)]
        {
            self.core.basepri = get_u32(buf, 92) as u8;
        }
        self.core.faultmask = get_u32(buf, 96) != 0;
        #[allow(clippy::cast_possible_truncation)]
        {
            self.core.ipsr = get_u32(buf, 100) as u8;
            self.core.itstate = get_u32(buf, 104) as u8;
        }
        self.core.sleeping = get_u32(buf, 108) != 0;
        self.core.event = get_u32(buf, 112) != 0;
        self.core.cycles = get_u64(buf, 116);
        self.core.cyccnt = get_u64(buf, 124);
        for (i, byte) in buf[SNAPSHOT_HEADER_LEN..SNAPSHOT_LEN].iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let addr = SNAPSHOT_RAM_BASE.wrapping_add(i as u32);
            self.core
                .mem()
                .write(addr, *byte)
                .map_err(|_| SnapshotError::MemoryOutOfRange { addr })?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{
        SNAPSHOT_API_MAJOR, SNAPSHOT_API_MINOR, SNAPSHOT_API_PATCH, SNAPSHOT_LEN, SNAPSHOT_MAGIC,
        SNAPSHOT_RAM_BASE, SNAPSHOT_VERSION,
    };
    use crate::board::BoardMemory;
    use crate::loader::load_flat;
    use crate::machine::Machine;
    use crate::mem::Bus;

    fn boot() -> Machine<BoardMemory> {
        // `ldr r0, =RAM; movs r1, #0x5A; str r1, [r0]; b .` at 0x8
        // (`ldr r0, [pc, #4]`: fetch pc reads 0x0C, pool sits at 0x10).
        let mut fw = vec![
            0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x01, 0x48, 0x5A, 0x21, 0x01, 0x60,
            0xFE, 0xE7,
        ];
        fw.extend_from_slice(&SNAPSHOT_RAM_BASE.to_le_bytes());
        let mut m =
            Machine::for_name_bare("virtual", &load_flat(0x0, &fw)).expect("virtual exists");
        m.set_vector_base(0x0);
        m.reset().expect("reset");
        m
    }

    #[test]
    fn save_restore_round_trips_pc_and_ram() {
        let mut m = boot();
        let _ = m.run(16);
        assert_eq!(m.core.mem().read(SNAPSHOT_RAM_BASE).unwrap(), 0x5A);
        let pc = m.core.regs.pc;
        let cycles = m.core.cycles;
        let mut buf = vec![0u8; SNAPSHOT_LEN];
        assert_eq!(m.snapshot_save(&mut buf).unwrap(), SNAPSHOT_LEN);
        assert_eq!(
            u32::from_le_bytes(buf[0..4].try_into().unwrap()),
            SNAPSHOT_MAGIC
        );
        assert_eq!(
            u32::from_le_bytes(buf[4..8].try_into().unwrap()),
            SNAPSHOT_VERSION
        );
        // The API stamp rides at the header tail (diagnostic minor/patch,
        // gated major).
        assert_eq!(
            u32::from_le_bytes(buf[140..144].try_into().unwrap()),
            SNAPSHOT_API_MAJOR
        );
        assert_eq!(
            u32::from_le_bytes(buf[144..148].try_into().unwrap()),
            SNAPSHOT_API_MINOR
        );
        assert_eq!(
            u32::from_le_bytes(buf[148..152].try_into().unwrap()),
            SNAPSHOT_API_PATCH
        );
        // Mutate past the snapshot, then rewind (the `b .` tail pins pc,
        // so the cycle counter proves forward motion, not pc).
        m.core.mem().write(SNAPSHOT_RAM_BASE, 0x00).unwrap();
        let _ = m.run(32);
        assert!(m.core.cycles > cycles);
        m.snapshot_restore(&buf).unwrap();
        assert_eq!(m.core.regs.pc, pc);
        assert_eq!(m.core.cycles, cycles);
        assert_eq!(m.core.mem().read(SNAPSHOT_RAM_BASE).unwrap(), 0x5A);
    }

    #[test]
    fn save_rejects_a_short_buffer() {
        let mut m = boot();
        let mut buf = vec![0u8; SNAPSHOT_LEN - 1];
        assert!(m.snapshot_save(&mut buf).is_err());
    }

    #[test]
    fn restore_rejects_garbage() {
        let mut m = boot();
        let mut buf = vec![0u8; SNAPSHOT_LEN];
        m.snapshot_save(&mut buf).unwrap();
        let pc = m.core.regs.pc;
        let mut bad = buf.clone();
        bad[0] ^= 0xFF;
        assert!(m.snapshot_restore(&bad).is_err());
        assert_eq!(m.core.regs.pc, pc, "rejected buffer touches nothing");
        assert!(m.snapshot_restore(&buf[..SNAPSHOT_LEN - 1]).is_err());
        let mut bad_ver = buf.clone();
        bad_ver[4] = 0x7F;
        assert!(m.snapshot_restore(&bad_ver).is_err());
        // A foreign API major fails closed too (layout may match, semantics
        // may have moved) and touches nothing.
        let mut bad_api = buf.clone();
        let foreign = SNAPSHOT_API_MAJOR.wrapping_add(1);
        bad_api[140..144].copy_from_slice(&foreign.to_le_bytes());
        assert_eq!(
            m.snapshot_restore(&bad_api),
            Err(super::SnapshotError::BadApiVersion {
                got: foreign,
                want: SNAPSHOT_API_MAJOR,
            })
        );
        assert_eq!(m.core.regs.pc, pc, "rejected buffer touches nothing");
    }
}
