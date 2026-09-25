//! TI Tiva C μDMA controller (`UDMA`, `0x400F_F000`).
//!
//! Register map and transfer semantics follow the TI `hw_udma.h` +
//! `udma.c`/`udma.h` oracle (BSD, fetched at runtime, never vendored):
//! `STAT` (`MASTEN`, `DMACHANS`, state machine), `CFG` (`MASTEN`),
//! `CTLBASE` (table base, low 10 bits reserved), `ALTBASE` (`RO` reset
//! `0x200`), `WAITSTAT` (`RO` reset `0x03C3CF00`), `SWREQ` (software
//! trigger), `USEBURSTSET/CLR`, `REQMASKSET/CLR`, `ENASET/CLR`
//! (auto-cleared by hardware on completion), `ALTSET/CLR`, `PRIOSET/CLR`,
//! `ERRCLR`, `CHASGN`, `CHMAP0-3`. Table 9-13 lists no register at
//! `0x504` (`0x500` `CHASGN` → `0x510` `CHMAP0` is a reserved gap), so
//! `0x504` faults `Reserved`: completion is signaled `dma_done` to the
//! peripheral plus the Table 9-6 vectors (44 software, 45 error) behind
//! the two NVIC lines, with `DMAENASET` auto-clear and `XFERMODE=STOP`
//! write-back for status — there is no per-channel done register.
//!
//! Modeled: 32 channels over the SRAM control table — `AUTO` runs to
//! completion, `BASIC` on a software request transfers only `ARBSIZE`
//! items per trigger (§9.2.6.2, partials update `XFERSIZE` and keep the
//! channel enabled; full completions write back `STOP`), `PINGPONG`
//! alternates primary/alternate with the `ALTSET` bit flipping, `STOP`
//! disables the channel if enabled (§9.2.6.1), scatter-gather inert.
//! `SRCENDP`/`DSTENDP` hold end pointers (`start = end - N*width + 1`,
//! fixed address on `*_INC_NONE`), `N = XFERSIZE+1` (1..1024) in 8/16/32
//! bit units, completion writes back `CHCTL` updating only
//! `XFERSIZE+XFERMODE` (preserving `NXTUSEBURST`), `dma_done` + `ERRCLR`
//! behind the two NVIC lines (done, error),
//! priority service order (high-priority group first, lower channel number
//! first), instant transfers through the shared DMA mailbox — a trigger on
//! an enabled channel runs as if the request were asserted (the F4 request
//! convention). Verified against a Python reference of the TivaWare
//! ENDP/N/INC arithmetic plus TivaWare-documented sequences.
//! Deliberate scope: scatter-gather task lists (no firmware demand);
//! `USEBURST` (timing-transparent — transfers complete whole); `CHMAP`/
//! `CHASGN` routing (no peripheral request sources modeled; `SWREQ`
//! triggers any channel); `REQMASK` (software requests bypass it);
//! mid-transfer residual counts beyond the `XFERSIZE` write-back
//! (nothing else is observable mid-transfer); SRC/DST size mismatch
//! (source width wins); reserved size encodings raise the error flag.

use crate::device::stm32::dma::DmaUnit;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// `UDMA` base address.
pub const UDMA_BASE: u32 = 0x400F_F000;

/// Register offsets.
const STAT_OFF: u32 = 0x00;
const CFG_OFF: u32 = 0x04;
const CTLBASE_OFF: u32 = 0x08;
const ALTBASE_OFF: u32 = 0x0C;
const WAITSTAT_OFF: u32 = 0x10;
const SWREQ_OFF: u32 = 0x14;
const USEBURSTSET_OFF: u32 = 0x18;
const USEBURSTCLR_OFF: u32 = 0x1C;
const REQMASKSET_OFF: u32 = 0x20;
const REQMASKCLR_OFF: u32 = 0x24;
const ENASET_OFF: u32 = 0x28;
const ENACLR_OFF: u32 = 0x2C;
const ALTSET_OFF: u32 = 0x30;
const ALTCLR_OFF: u32 = 0x34;
const PRIOSET_OFF: u32 = 0x38;
const PRIOCLR_OFF: u32 = 0x3C;
const ERRCLR_OFF: u32 = 0x4C;
const CHASGN_OFF: u32 = 0x500;
const CHMAP_OFF: u32 = 0x510;
/// Table 9-13 reserved gap `0x504` (`CHASGN` → `CHMAP0`): no register;
/// accesses fault `Reserved` via the inner `RegisterFile`.
const CHIS_GAP_OFF: u32 = 0x504;
/// `DMAALTBASE` is `RO` reset `0x200` (Table 9-13/Register 7); it derives
/// from `CTLBASE` (`CTLBASE & !0x3FF + 0x200`, which is `0x200` at reset).
const ALTBASE_RESET: u32 = 0x200;
/// `DMAWAITSTAT` is `RO` reset `0x03C3CF00` (Table 9-13/Register 8).
const WAITSTAT_RESET: u32 = 0x03C3_CF00;

/// `STAT` fields (state machine reads idle — transfers are instant).
const STAT_MASTEN: u32 = 1 << 0;
const STAT_DMACHANS_SHIFT: u32 = 16;
/// Channel count on both families (TivaWare `DMACHANS` = count − 1).
const CHANNELS: u32 = 32;

/// `CHCTL` fields (TivaWare `UDMA_CHCTL_*`).
const CHCTL_DSTINC_SHIFT: u32 = 30;
const CHCTL_DSTSIZE_SHIFT: u32 = 28;
const CHCTL_SRCINC_SHIFT: u32 = 26;
const CHCTL_SRCSIZE_SHIFT: u32 = 24;
const CHCTL_ARBSIZE_SHIFT: u32 = 14;
const CHCTL_ARBSIZE_MASK: u32 = 0x0F;
const CHCTL_N_SHIFT: u32 = 4;
const CHCTL_N_MASK: u32 = 0x3FF;
const CHCTL_NXTUSEBURST: u32 = 1 << 3;
const CHCTL_MODE_MASK: u32 = 0x07;
/// Write-back updates only `XFERSIZE[13:4]` + `XFERMODE[2:0]` (§9.2.5):
/// `NXTUSEBURST[3]` and all upper fields are preserved.
const CHCTL_WB_MASK: u32 = 0x3FF7;
const MODE_STOP: u32 = 0;
const MODE_BASIC: u32 = 1;
const MODE_AUTO: u32 = 2;
const MODE_PINGPONG: u32 = 3;

/// Arbitration size in items for an `ARBSIZE` encoding (§9.2.3/Register 3:
/// powers of two, `0xA-0xF` all mean 1024).
fn arb_size(arb: u32) -> u32 {
    match arb & CHCTL_ARBSIZE_MASK {
        0 => 1,
        1 => 2,
        2 => 4,
        3 => 8,
        4 => 16,
        5 => 32,
        6 => 64,
        7 => 128,
        8 => 256,
        9 => 512,
        _ => 1024,
    }
}

/// One queued table access: reads land in `results`, writes go direct.
#[derive(Debug)]
pub enum UdmaOp {
    /// Read `len` bytes at `addr` (short on `Reserved` — the error path).
    Read {
        /// First byte address.
        addr: u32,
        /// Byte count requested.
        len: u16,
    },
    /// Write `bytes` at `addr` (control-word write-backs).
    Write {
        /// First byte address.
        addr: u32,
        /// Payload bytes.
        bytes: Vec<u8>,
    },
}

/// Mailbox between the μDMA controller and the bus (mirrors the
/// Ethernet-mailbox shape: requests out, read results back).
#[derive(Debug, Default)]
pub struct UdmaMailbox {
    /// Queued table accesses, drained in order after each device tick.
    pub ops: VecDeque<UdmaOp>,
    /// Landed read results, in request order.
    pub results: VecDeque<Vec<u8>>,
}

/// Transfer phase: one outstanding table read at a time.
#[derive(Debug, Default, PartialEq, Eq)]
enum Phase {
    #[default]
    Idle,
    AwaitTable {
        ch: u32,
        alt: bool,
        entry: u32,
    },
}

/// Behavior behind the μDMA window.
#[derive(Debug)]
pub struct UdmaBehavior {
    dma_mailbox: Arc<Mutex<Vec<DmaUnit>>>,
    udma_mailbox: Arc<Mutex<UdmaMailbox>>,
    master_en: bool,
    useburst: u32,
    reqmask: u32,
    ena: u32,
    alt: u32,
    prio: u32,
    swreq: u32,
    /// Completed-channel `dma_done` latch behind NVIC line 0 (Table 9-6
    /// vector 44; no register — Table 9-13 `0x504` is a reserved gap).
    /// Cleared by the NVIC `acknowledge` when the line is taken.
    done: u32,
    err: bool,
    phase: Phase,
}

impl UdmaBehavior {
    fn push_unit(&self, src: u32, dst: u32, len: u8) {
        if let Ok(mut mb) = self.dma_mailbox.lock() {
            mb.push((src, dst, len));
        }
    }

    fn push_read(&self, addr: u32, len: u16) {
        if let Ok(mut mb) = self.udma_mailbox.lock() {
            mb.ops.push_back(UdmaOp::Read { addr, len });
        }
    }

    fn push_write(&self, addr: u32, bytes: Vec<u8>) {
        if let Ok(mut mb) = self.udma_mailbox.lock() {
            mb.ops.push_back(UdmaOp::Write { addr, bytes });
        }
    }

    fn pop_result(&self) -> Option<Vec<u8>> {
        self.udma_mailbox
            .lock()
            .ok()
            .and_then(|mut mb| mb.results.pop_front())
    }

    /// Alternate table base (Register 7 `RO`): `CTLBASE & !0x3FF + 0x200`
    /// (reset `0x200` when `CTLBASE` is zero). Writes to `ALTBASE` are
    /// rejected; the stored word is ignored.
    fn alt_base<W>(view: &Registers<W>) -> u32
    where
        W: core::ops::Deref<Target = [u32]>,
    {
        (view.get(CTLBASE_OFF) & !0x3FF).wrapping_add(ALTBASE_RESET)
    }

    /// Control-table entry address for `ch` (primary or alternate).
    fn entry_addr<W>(view: &Registers<W>, ch: u32, alt: bool) -> u32
    where
        W: core::ops::Deref<Target = [u32]>,
    {
        let base = if alt {
            Self::alt_base(view)
        } else {
            view.get(CTLBASE_OFF) & !0x3FF
        };
        base.wrapping_add(ch * 16)
    }

    /// Next ready channel in service order (high-priority group first,
    /// lower channel number first), or `None`.
    fn ready_channel(&self) -> Option<u32> {
        for high in [true, false] {
            for ch in 0..CHANNELS {
                let hp = self.prio & (1 << ch) != 0;
                if hp == high && self.ena & (1 << ch) != 0 && self.swreq & (1 << ch) != 0 {
                    return Some(ch);
                }
            }
        }
        None
    }

    /// Flag a bus error: `ERRCLR` latches, the channel disables (no `done`).
    fn raise_error(&mut self, ch: u32) {
        self.err = true;
        self.ena &= !(1 << ch);
    }

    /// Run `count` items from a parsed table entry: queue the data units.
    /// `count <= N` (partial `BASIC` chunks pass fewer than `N`).
    fn run_transfer(&mut self, srcend: u32, dstend: u32, chctl: u32, count: u32) {
        let n = ((chctl >> CHCTL_N_SHIFT) & CHCTL_N_MASK) + 1;
        debug_assert!(count <= n);
        let srcw = 1u64 << ((chctl >> CHCTL_SRCSIZE_SHIFT) & 3);
        let sstep = [1u64, 2, 4, 0][((chctl >> CHCTL_SRCINC_SHIFT) & 3) as usize];
        let dstep = [1u64, 2, 4, 0][((chctl >> CHCTL_DSTINC_SHIFT) & 3) as usize];
        let total = u64::from(n) * srcw;
        let srcstart = (srcend as u64).wrapping_sub(total - 1);
        let dststart = (dstend as u64).wrapping_sub(total - 1);
        for k in 0..count {
            let s = if sstep == 0 {
                srcend
            } else {
                (srcstart + u64::from(k) * sstep) as u32
            };
            let d = if dstep == 0 {
                dstend
            } else {
                (dststart + u64::from(k) * dstep) as u32
            };
            #[allow(clippy::cast_possible_truncation)]
            self.push_unit(s, d, srcw as u8);
        }
    }

    /// Write back `CHCTL` updating only `XFERSIZE+XFERMODE` (§9.2.5):
    /// `remaining` items left (0 for a full completion), `mode` next mode.
    fn writeback(&self, entry: u32, chctl: u32, remaining: u32, mode: u32) {
        let xf = if remaining == 0 {
            0
        } else {
            (remaining - 1) & CHCTL_N_MASK
        };
        let wb = (chctl & !CHCTL_WB_MASK) | (xf << CHCTL_N_SHIFT) | (mode & CHCTL_MODE_MASK);
        self.push_write(entry + 8, wb.to_le_bytes().to_vec());
    }

    /// Complete a landed table read: run, write back, flag — or error.
    fn complete_table(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        ch: u32,
        alt: bool,
        entry: u32,
        bytes: &[u8],
    ) {
        let _ = alt;
        if bytes.len() < 12 {
            self.raise_error(ch);
            return;
        }
        let w = |i: usize| {
            u32::from_le_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ])
        };
        let (srcend, dstend, chctl) = (w(0), w(1), w(2));
        // Reserved size encodings are rejected (TivaWare asserts valid
        // sizes; silicon is undefined — surfacing beats silence).
        if (chctl >> CHCTL_SRCSIZE_SHIFT) & 3 == 3 || (chctl >> CHCTL_DSTSIZE_SHIFT) & 3 == 3 {
            self.raise_error(ch);
            return;
        }
        let n = ((chctl >> CHCTL_N_SHIFT) & CHCTL_N_MASK) + 1;
        match chctl & CHCTL_MODE_MASK {
            // §9.2.6.1: `STOP` performs no transfer and disables the
            // channel if it is enabled.
            MODE_STOP => {
                self.ena &= !(1 << ch);
            }
            // §9.2.6.2: `BASIC` on a software request transfers only
            // `ARBSIZE` items. Partials update `XFERSIZE`, keep the mode
            // and the enable, and do not signal completion; the final
            // chunk behaves like `AUTO` (write back `STOP`, auto-clear
            // `DMAENASET` per §9.2.5/§9.3.2.3, signal `dma_done`).
            MODE_BASIC => {
                let chunk = arb_size((chctl >> CHCTL_ARBSIZE_SHIFT) & CHCTL_ARBSIZE_MASK).min(n);
                self.run_transfer(srcend, dstend, chctl, chunk);
                if chunk < n {
                    self.writeback(entry, chctl, n - chunk, MODE_BASIC);
                } else {
                    self.writeback(entry, chctl, 0, MODE_STOP);
                    self.ena &= !(1 << ch);
                    self.done |= 1 << ch;
                }
            }
            MODE_AUTO => {
                self.run_transfer(srcend, dstend, chctl, n);
                self.writeback(entry, chctl, 0, MODE_STOP);
                self.ena &= !(1 << ch);
                self.done |= 1 << ch;
            }
            MODE_PINGPONG => {
                self.run_transfer(srcend, dstend, chctl, n);
                self.writeback(entry, chctl, 0, MODE_STOP);
                self.alt ^= 1 << ch;
                self.done |= 1 << ch;
            }
            _ => {} // Scatter-gather task lists stay inert (documented).
        }
    }
}

impl Behavior for UdmaBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn reset(&mut self) {
        self.master_en = false;
        self.useburst = 0;
        self.reqmask = 0;
        self.ena = 0;
        self.alt = 0;
        self.prio = 0;
        self.swreq = 0;
        self.done = 0;
        self.err = false;
        self.phase = Phase::Idle;
        if let Ok(mut mb) = self.dma_mailbox.lock() {
            mb.clear();
        }
        if let Ok(mut mb) = self.udma_mailbox.lock() {
            mb.ops.clear();
            mb.results.clear();
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let _ = view;
        u32::from(self.done != 0) | (u32::from(self.err) << 1)
    }

    fn acknowledge(&mut self, lines: u32) {
        // The NVIC taking the done line consumes the `dma_done` latch
        // (there is no done register — Table 9-13 `0x504` is reserved).
        if lines & 1 != 0 {
            self.done = 0;
        }
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        if self.phase != Phase::Idle {
            if let Some(result) = self.pop_result() {
                let Phase::AwaitTable { ch, alt, entry } =
                    core::mem::replace(&mut self.phase, Phase::Idle)
                else {
                    return;
                };
                self.complete_table(view, ch, alt, entry, &result);
            }
            return;
        }
        if !self.master_en {
            return;
        }
        if let Some(ch) = self.ready_channel() {
            self.swreq &= !(1 << ch);
            let alt = self.alt & (1 << ch) != 0;
            let entry = Self::entry_addr(view, ch, alt);
            self.push_read(entry, 12);
            self.phase = Phase::AwaitTable { ch, alt, entry };
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            STAT_OFF => {
                let _ = value;
                (u32::from(self.master_en)) | ((CHANNELS - 1) << STAT_DMACHANS_SHIFT)
            }
            ALTBASE_OFF => Self::alt_base(view),
            WAITSTAT_OFF => {
                let _ = value;
                WAITSTAT_RESET
            }
            USEBURSTSET_OFF => self.useburst,
            REQMASKSET_OFF => self.reqmask,
            ENASET_OFF => self.ena,
            ALTSET_OFF => self.alt,
            PRIOSET_OFF => self.prio,
            ERRCLR_OFF => u32::from(self.err),
            _ => value,
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        // Whole-word stores act (the TivaWare driver convention);
        // partial lanes merge into the stored word only.
        let word = view.merged(offset, val, shift);
        match offset {
            CFG_OFF => {
                if shift == 24 {
                    self.master_en = word & 1 != 0;
                }
                view.set(offset, word);
                word
            }
            SWREQ_OFF => {
                if shift == 24 {
                    // Software requests bypass `REQMASK` (the mask gates
                    // peripheral requests only).
                    self.swreq |= word;
                }
                view.set(offset, word);
                word
            }
            USEBURSTSET_OFF | REQMASKSET_OFF | ENASET_OFF | ALTSET_OFF | PRIOSET_OFF => {
                // Per-lane OR: any write pattern (bytes or words) sets
                // exactly its bits.
                let bits = match offset {
                    USEBURSTSET_OFF => &mut self.useburst,
                    REQMASKSET_OFF => &mut self.reqmask,
                    ENASET_OFF => &mut self.ena,
                    ALTSET_OFF => &mut self.alt,
                    _ => &mut self.prio,
                };
                *bits |= u32::from(val) << shift;
                let out = *bits;
                view.set(offset, out);
                out
            }
            USEBURSTCLR_OFF | REQMASKCLR_OFF | ENACLR_OFF | ALTCLR_OFF | PRIOCLR_OFF => {
                // Per-lane clear, mirroring the SET arm.
                let mask = u32::from(val) << shift;
                match offset {
                    USEBURSTCLR_OFF => self.useburst &= !mask,
                    REQMASKCLR_OFF => self.reqmask &= !mask,
                    ENACLR_OFF => self.ena &= !mask,
                    ALTCLR_OFF => self.alt &= !mask,
                    _ => self.prio &= !mask,
                }
                0
            }
            ERRCLR_OFF => {
                if shift == 24 && word & 1 != 0 {
                    self.err = false;
                }
                let v = u32::from(self.err);
                view.set(offset, v);
                v
            }
            ALTBASE_OFF | WAITSTAT_OFF => {
                // `RO` (Table 9-13): writes never land.
                view.get(offset)
            }
            _ => {
                view.set(offset, word);
                word
            }
        }
    }
}

/// A Tiva C μDMA controller (shared DMA + table mailboxes).
pub struct Udma {
    inner: RegisterFile<UdmaBehavior>,
}

impl Udma {
    /// A new controller on `D`'s tables (`"UDMA"`), sharing the data-copy
    /// mailbox with the memory bus and a table mailbox for control-table
    /// reads and write-backs.
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"UDMA"`.
    #[must_use]
    pub fn for_device<D: Device>(
        dma_mailbox: Arc<Mutex<Vec<DmaUnit>>>,
        udma_mailbox: Arc<Mutex<UdmaMailbox>>,
    ) -> Self {
        let def = D::peripheral("UDMA").expect("generated tables contain UDMA");
        Self {
            inner: RegisterFile::new(
                def,
                UdmaBehavior {
                    dma_mailbox,
                    udma_mailbox,
                    master_en: false,
                    useburst: 0,
                    reqmask: 0,
                    ena: 0,
                    alt: 0,
                    prio: 0,
                    swreq: 0,
                    done: 0,
                    err: false,
                    phase: Phase::Idle,
                },
            ),
        }
    }
}

impl Peripheral for Udma {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        // Table 9-13 `0x504` is a reserved gap (`CHASGN` → `CHMAP0`):
        // no interception — the inner `RegisterFile` faults `Reserved`.
        self.inner.read(addr)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
}

impl InterruptSource for Udma {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }

    fn lines(&self) -> u32 {
        0b11
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c129xnczad::Tm4c129xnczad;

    const UDMA0_BASE: u32 = 0x400F_F000;

    fn new_udma() -> (Udma, Arc<Mutex<Vec<DmaUnit>>>, Arc<Mutex<UdmaMailbox>>) {
        let dma = Arc::new(Mutex::new(Vec::new()));
        let mb = Arc::new(Mutex::new(UdmaMailbox::default()));
        (
            Udma::for_device::<Tm4c129xnczad>(dma.clone(), mb.clone()),
            dma,
            mb,
        )
    }

    fn read_word(udma: &mut Udma, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(udma.read(UDMA0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(udma: &mut Udma, offset: u32, value: u32) {
        for lane in 0..4 {
            udma.write(
                UDMA0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Seed one landed table read (simulates the bus drain).
    fn seed_table(mb: &Arc<Mutex<UdmaMailbox>>, words: &[u32]) {
        let bytes = words.iter().flat_map(|w| w.to_le_bytes()).collect();
        mb.lock().unwrap().results.push_back(bytes);
    }

    /// Take queued data-copy units.
    fn take_units(mb: &Arc<Mutex<Vec<DmaUnit>>>) -> Vec<DmaUnit> {
        core::mem::take(&mut *mb.lock().unwrap())
    }

    /// Take queued table writes as `(addr, word)`.
    fn take_writes(mb: &Arc<Mutex<UdmaMailbox>>) -> Vec<(u32, u32)> {
        let ops = core::mem::take(&mut mb.lock().unwrap().ops);
        ops.into_iter()
            .filter_map(|op| match op {
                UdmaOp::Write { addr, bytes } if bytes.len() == 4 => Some((
                    addr,
                    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
                )),
                _ => None,
            })
            .collect()
    }

    /// Take queued table-read addresses.
    fn take_reads(mb: &Arc<Mutex<UdmaMailbox>>) -> Vec<u32> {
        let ops = core::mem::take(&mut mb.lock().unwrap().ops);
        ops.into_iter()
            .filter_map(|op| match op {
                UdmaOp::Read { addr, .. } => Some(addr),
                _ => None,
            })
            .collect()
    }

    /// Master on, table at `0x2000_0400`, channel armed + requested.
    fn arm(udma: &mut Udma, ch: u32) {
        write_word(udma, CFG_OFF, 1);
        write_word(udma, CTLBASE_OFF, 0x2000_0400);
        write_word(udma, ENASET_OFF, 1 << ch);
        write_word(udma, SWREQ_OFF, 1 << ch);
    }

    /// Frozen oracle vectors (`/tmp/opencode/udma_vecs.json`).
    /// `BASIC_CHCTL` is `BASIC` with `ARBSIZE=8` (`0x3`) and `N=16`
    /// (`XFERSIZE=15`); `BASIC_WB` is its full-completion write-back
    /// (`STOP`, `N=0`, upper fields preserved).
    const BASIC_CHCTL: u32 = 0xAA00_C0F1;
    const BASIC_WB: u32 = 0xAA00_C000;

    #[test]
    fn regs_latch_and_stat() {
        let (mut udma, _, _) = new_udma();
        assert_eq!(read_word(&mut udma, STAT_OFF) & STAT_MASTEN, 0);
        write_word(&mut udma, CFG_OFF, 1);
        assert_eq!(
            read_word(&mut udma, STAT_OFF),
            STAT_MASTEN | (31 << STAT_DMACHANS_SHIFT)
        );
        write_word(&mut udma, CTLBASE_OFF, 0x2000_0400);
        assert_eq!(read_word(&mut udma, CTLBASE_OFF), 0x2000_0400);
        write_word(&mut udma, CHMAP_OFF, 0x0006_0000);
        assert_eq!(read_word(&mut udma, CHMAP_OFF), 0x0006_0000);
        write_word(&mut udma, CHASGN_OFF, 0x0000_00FF);
        assert_eq!(read_word(&mut udma, CHASGN_OFF), 0x0000_00FF);
        // Set/clear pairs move the behavior bits.
        for (set, clr) in [
            (USEBURSTSET_OFF, USEBURSTCLR_OFF),
            (REQMASKSET_OFF, REQMASKCLR_OFF),
            (ENASET_OFF, ENACLR_OFF),
            (ALTSET_OFF, ALTCLR_OFF),
            (PRIOSET_OFF, PRIOCLR_OFF),
        ] {
            write_word(&mut udma, set, 0xA5A5_0000);
            assert_eq!(read_word(&mut udma, set), 0xA5A5_0000);
            write_word(&mut udma, clr, 0xA500_0000);
            assert_eq!(read_word(&mut udma, set), 0x00A5_0000);
            write_word(&mut udma, clr, 0xFFFF_FFFF);
        }
        assert_eq!(udma.pending(), 0);
    }

    /// `BASIC` word copy over two software triggers (§9.2.6.2): each
    /// trigger moves `ARBSIZE=8` of the `N=16` items; the first write-back
    /// keeps `BASIC` with `XFERSIZE=7`, the second writes back `STOP`
    /// (`BASIC_WB`), auto-clears `DMAENASET`, and signals `dma_done`.
    #[test]
    fn basic_mem_to_mem_words() {
        let (mut udma, dma, mb) = new_udma();
        arm(&mut udma, 0);
        udma.tick(1);
        assert_eq!(take_reads(&mb), [0x2000_0400]);
        seed_table(&mb, &[0x2000_003F, 0x2000_013F, BASIC_CHCTL]);
        udma.tick(1);
        // First `ARBSIZE` chunk only.
        let units = take_units(&dma);
        assert_eq!(units.len(), 8);
        assert_eq!(units[0], (0x2000_0000, 0x2000_0100, 4));
        assert_eq!(units[7], (0x2000_001C, 0x2000_011C, 4));
        assert_eq!(
            take_writes(&mb),
            [(
                0x2000_0408,
                BASIC_CHCTL & !CHCTL_WB_MASK | (7 << CHCTL_N_SHIFT) | MODE_BASIC
            )]
        );
        assert_eq!(read_word(&mut udma, ENASET_OFF) & 1, 1, "still enabled");
        assert_eq!(udma.pending() & 1, 0, "no done until full");
        // Second trigger finishes the frame.
        write_word(&mut udma, SWREQ_OFF, 1);
        udma.tick(1);
        assert_eq!(take_reads(&mb), [0x2000_0400]);
        seed_table(
            &mb,
            &[
                0x2000_003F,
                0x2000_013F,
                BASIC_CHCTL & !CHCTL_WB_MASK | (7 << CHCTL_N_SHIFT) | MODE_BASIC,
            ],
        );
        udma.tick(1);
        let units = take_units(&dma);
        assert_eq!(units.len(), 8);
        assert_eq!(units[0], (0x2000_0020, 0x2000_0120, 4));
        assert_eq!(units[7], (0x2000_003C, 0x2000_013C, 4));
        assert_eq!(take_writes(&mb), [(0x2000_0408, BASIC_WB)]);
        assert_eq!(read_word(&mut udma, ENASET_OFF) & 1, 0, "auto-cleared");
        assert_eq!(udma.pending() & 1, 1, "done line");
        udma.acknowledge(1);
        assert_eq!(udma.pending(), 0);
    }

    /// `BASIC` on a software request transfers only `ARBSIZE` items even
    /// when more remain (§9.2.6.2): `N=16`, `ARBSIZE=4` moves 4 per trigger.
    #[test]
    fn basic_swreq_transfers_arb_only() {
        // `ARBSIZE=2` (4 transfers), `XFERSIZE=15` (`N=16`), `BASIC`.
        let chctl = (BASIC_CHCTL & !(CHCTL_ARBSIZE_MASK << CHCTL_ARBSIZE_SHIFT))
            | (2 << CHCTL_ARBSIZE_SHIFT);
        let (mut udma, dma, mb) = new_udma();
        arm(&mut udma, 0);
        udma.tick(1);
        seed_table(&mb, &[0x2000_003F, 0x2000_013F, chctl]);
        udma.tick(1);
        assert_eq!(take_units(&dma).len(), 4, "only ARBSIZE items");
        assert_eq!(read_word(&mut udma, ENASET_OFF) & 1, 1, "kept enabled");
        assert_eq!(udma.pending() & 1, 0, "no done on partial");
    }

    /// `AUTO` completes the same shape in one trigger (ignores `ARBSIZE`).
    #[test]
    fn auto_completes_whole() {
        let (mut udma, dma, mb) = new_udma();
        arm(&mut udma, 3);
        udma.tick(1);
        assert_eq!(take_reads(&mb), [0x2000_0400 + 3 * 16]);
        seed_table(
            &mb,
            &[0x2000_003F, 0x2000_013F, BASIC_CHCTL & !7 | MODE_AUTO],
        );
        udma.tick(1);
        assert_eq!(take_units(&dma).len(), 16);
        assert_eq!(udma.pending() & 1, 1, "done line");
        assert_eq!(
            read_word(&mut udma, ENASET_OFF) & (1 << 3),
            0,
            "auto-cleared"
        );
        udma.acknowledge(1);
        assert_eq!(udma.pending(), 0);
    }

    /// Fixed source, byte units: one unit per transfer at the same address
    /// (`AUTO` runs the `N=8` to completion in one trigger).
    #[test]
    fn fixed_src_bytes() {
        let (mut udma, dma, mb) = new_udma();
        arm(&mut udma, 1);
        udma.tick(1);
        seed_table(&mb, &[0x4000_C000, 0x2000_0207, 0x0C00_0072]);
        udma.tick(1);
        let units = take_units(&dma);
        assert_eq!(units.len(), 8);
        assert_eq!(units[0], (0x4000_C000, 0x2000_0200, 1));
        assert_eq!(units[7], (0x4000_C000, 0x2000_0207, 1));
    }

    /// Fixed destination, halfword units (`AUTO` to completion).
    #[test]
    fn fixed_dst_halfwords() {
        let (mut udma, dma, mb) = new_udma();
        arm(&mut udma, 2);
        udma.tick(1);
        seed_table(&mb, &[0x2000_030F, 0x4000_C004, 0xC500_0072]);
        udma.tick(1);
        let units = take_units(&dma);
        assert_eq!(units.len(), 8);
        assert_eq!(units[0], (0x2000_0300, 0x4000_C004, 2));
        assert_eq!(units[7], (0x2000_030E, 0x4000_C004, 2));
    }

    /// PINGPONG alternates primary/alternate with `ALTSET` flipping.
    /// The alternate base derives from `CTLBASE` (Register 7 `RO`):
    /// `CTLBASE=0x2000_0400` → alternate `0x2000_0600`.
    #[test]
    fn pingpong_alternates() {
        let (mut udma, dma, mb) = new_udma();
        write_word(&mut udma, CFG_OFF, 1);
        write_word(&mut udma, CTLBASE_OFF, 0x2000_0400);
        write_word(&mut udma, ENASET_OFF, 1 << 4);
        // Primary first (`ALTSET` clear).
        write_word(&mut udma, SWREQ_OFF, 1 << 4);
        udma.tick(1);
        assert_eq!(take_reads(&mb), [0x2000_0440]);
        seed_table(
            &mb,
            &[0x2000_001F, 0x2000_021F, BASIC_CHCTL & !7 | MODE_PINGPONG],
        );
        udma.tick(1);
        assert_eq!(take_units(&dma).len(), 16);
        assert_eq!(read_word(&mut udma, ALTSET_OFF) & (1 << 4), 1 << 4);
        // Alternate next (same channel, other struct at `CTLBASE+0x200`).
        write_word(&mut udma, SWREQ_OFF, 1 << 4);
        udma.tick(1);
        assert_eq!(take_reads(&mb), [0x2000_0640]);
        seed_table(
            &mb,
            &[0x2000_005F, 0x2000_025F, BASIC_CHCTL & !7 | MODE_PINGPONG],
        );
        udma.tick(1);
        let units = take_units(&dma);
        assert_eq!(units[0], (0x2000_0020, 0x2000_0220, 4));
        assert_eq!(read_word(&mut udma, ALTSET_OFF) & (1 << 4), 0, "flips back");
        // Completion writes back mode STOP with N = 0 (both structs).
        assert_eq!(take_writes(&mb), [(0x2000_0648, BASIC_WB)]);
    }

    /// `STOP` disables the channel if enabled (§9.2.6.1); scatter-gather
    /// stays inert without moving data or signaling completion.
    #[test]
    fn stop_and_sg_inert() {
        // `STOP` with the channel enabled: no data, enable cleared.
        let (mut udma, dma, mb) = new_udma();
        arm(&mut udma, 0);
        udma.tick(1);
        seed_table(
            &mb,
            &[0x2000_003F, 0x2000_013F, BASIC_CHCTL & !7 | MODE_STOP],
        );
        udma.tick(1);
        assert!(take_units(&dma).is_empty());
        assert_eq!(read_word(&mut udma, ENASET_OFF) & 1, 0, "STOP disables");
        assert_eq!(udma.pending() & 1, 0, "STOP flags nothing");
        // Scatter-gather modes stay inert.
        for mode in [4, 6] {
            let (mut udma, dma, mb) = new_udma();
            arm(&mut udma, 0);
            udma.tick(1);
            seed_table(&mb, &[0x2000_003F, 0x2000_013F, BASIC_CHCTL & !7 | mode]);
            udma.tick(1);
            assert!(take_units(&dma).is_empty(), "mode {mode} moves nothing");
            assert_eq!(udma.pending() & 1, 0, "mode {mode} flags nothing");
        }
    }

    /// Disabled channel, masked master, and cleared enable stay quiet.
    #[test]
    fn triggers_need_arm_and_master() {
        // Never enabled: `SWREQ` alone does nothing.
        let (mut udma, _, mb) = new_udma();
        write_word(&mut udma, CFG_OFF, 1);
        write_word(&mut udma, CTLBASE_OFF, 0x2000_0400);
        write_word(&mut udma, SWREQ_OFF, 1);
        udma.tick(1);
        assert!(take_reads(&mb).is_empty());
        // Master off: armed + requested does nothing.
        let (mut udma2, _, mb2) = new_udma();
        write_word(&mut udma2, CTLBASE_OFF, 0x2000_0400);
        write_word(&mut udma2, ENASET_OFF, 1);
        write_word(&mut udma2, SWREQ_OFF, 1);
        udma2.tick(1);
        assert!(take_reads(&mb2).is_empty());
        // Enable cleared before the tick: quiet.
        let (mut udma3, _, mb3) = new_udma();
        arm(&mut udma3, 0);
        write_word(&mut udma3, ENACLR_OFF, 1);
        udma3.tick(1);
        assert!(take_reads(&mb3).is_empty());
    }

    /// Lower channel number wins; high priority beats number.
    #[test]
    fn priority_order() {
        let (mut udma, _, mb) = new_udma();
        write_word(&mut udma, CFG_OFF, 1);
        write_word(&mut udma, CTLBASE_OFF, 0x2000_0400);
        write_word(&mut udma, ENASET_OFF, (1 << 2) | (1 << 5));
        write_word(&mut udma, SWREQ_OFF, (1 << 2) | (1 << 5));
        udma.tick(1);
        assert_eq!(take_reads(&mb), [0x2000_0420], "ch2 before ch5");
        // High priority on ch5 flips the order (request re-armed).
        let (mut udma2, _, mb2) = new_udma();
        write_word(&mut udma2, CFG_OFF, 1);
        write_word(&mut udma2, CTLBASE_OFF, 0x2000_0400);
        write_word(&mut udma2, PRIOSET_OFF, 1 << 5);
        write_word(&mut udma2, ENASET_OFF, (1 << 2) | (1 << 5));
        write_word(&mut udma2, SWREQ_OFF, (1 << 2) | (1 << 5));
        udma2.tick(1);
        assert_eq!(take_reads(&mb2), [0x2000_0450], "prio ch5 first");
    }

    /// Short table read raises the error flag and disables the channel.
    #[test]
    fn error_on_short_table() {
        let (mut udma, _, mb) = new_udma();
        arm(&mut udma, 0);
        udma.tick(1);
        mb.lock()
            .unwrap()
            .results
            .push_back(vec![0xAA, 0xBB, 0xCC, 0xDD]);
        udma.tick(1);
        assert_eq!(read_word(&mut udma, ERRCLR_OFF) & 1, 1, "error latched");
        assert_eq!(read_word(&mut udma, ENASET_OFF) & 1, 0, "channel disabled");
        assert_eq!(udma.pending() & 0b10, 0b10, "error line");
        assert_eq!(udma.pending() & 1, 0, "no done flag");
        write_word(&mut udma, ERRCLR_OFF, 1);
        assert_eq!(read_word(&mut udma, ERRCLR_OFF) & 1, 0, "W1C clears");
        assert_eq!(udma.pending(), 0);
    }

    /// Reserved size encodings are rejected, not silently honored.
    #[test]
    fn reserved_size_errors() {
        let (mut udma, _, mb) = new_udma();
        arm(&mut udma, 0);
        udma.tick(1);
        seed_table(&mb, &[0x2000_003F, 0x2000_013F, BASIC_CHCTL | (3 << 24)]);
        udma.tick(1);
        assert_eq!(read_word(&mut udma, ERRCLR_OFF) & 1, 1);
    }

    /// Table 9-13: `0x504` is a reserved gap (`CHASGN` → `CHMAP0`) —
    /// reads and writes fault `Reserved`; completion travels `dma_done` /
    /// the Table 9-6 vectors, cleared by `acknowledge`.
    #[test]
    fn gap_504_is_reserved() {
        let (mut udma, _, _) = new_udma();
        assert!(
            udma.read(UDMA0_BASE + CHIS_GAP_OFF).is_err(),
            "0x504 reads Reserved"
        );
        assert!(
            udma.read(UDMA0_BASE + CHIS_GAP_OFF + 3).is_err(),
            "0x504 lane 3 reads Reserved"
        );
        assert!(
            udma.write(UDMA0_BASE + CHIS_GAP_OFF, 0xFF).is_err(),
            "0x504 writes Reserved"
        );
    }

    /// Done latches behind NVIC line 0 and clears on `acknowledge`
    /// (no done register exists on this die).
    #[test]
    fn done_clears_on_acknowledge() {
        let (mut udma, _, mb) = new_udma();
        arm(&mut udma, 0);
        udma.tick(1);
        seed_table(
            &mb,
            &[0x2000_0003, 0x2000_0013, BASIC_CHCTL & !0x3F0 | (1 << 4)],
        );
        udma.tick(1);
        // `N=2`, `ARBSIZE=8`: full `AUTO`? No — this vector is `BASIC`
        // with `N=2 <= ARBSIZE`, so it completes in one trigger.
        assert_eq!(udma.pending() & 1, 1);
        udma.acknowledge(1);
        assert_eq!(udma.pending(), 0);
    }

    /// Register 7 `DMAALTBASE` is `RO` reset `0x200`, deriving from
    /// `CTLBASE` (writes never land).
    #[test]
    fn altbase_ro_reset_200() {
        let (mut udma, _, _) = new_udma();
        assert_eq!(read_word(&mut udma, ALTBASE_OFF), ALTBASE_RESET);
        write_word(&mut udma, CTLBASE_OFF, 0x2000_0400);
        assert_eq!(read_word(&mut udma, ALTBASE_OFF), 0x2000_0600);
        write_word(&mut udma, ALTBASE_OFF, 0x2000_0300);
        assert_eq!(
            read_word(&mut udma, ALTBASE_OFF),
            0x2000_0600,
            "ALTBASE writes rejected"
        );
    }

    /// Register 8 `DMAWAITSTAT` is `RO` reset `0x03C3CF00` (writes ignored).
    #[test]
    fn waitstat_ro_reset() {
        let (mut udma, _, _) = new_udma();
        assert_eq!(read_word(&mut udma, WAITSTAT_OFF), WAITSTAT_RESET);
        write_word(&mut udma, WAITSTAT_OFF, 0);
        assert_eq!(read_word(&mut udma, WAITSTAT_OFF), WAITSTAT_RESET);
    }

    /// §9.2.5 write-back updates only `XFERSIZE+XFERMODE`: `NXTUSEBURST`
    /// survives completion.
    #[test]
    fn writeback_preserves_nxtuseburst() {
        let (mut udma, _, mb) = new_udma();
        arm(&mut udma, 0);
        udma.tick(1);
        let chctl = BASIC_CHCTL | CHCTL_NXTUSEBURST;
        seed_table(&mb, &[0x2000_003F, 0x2000_013F, chctl & !7 | MODE_AUTO]);
        udma.tick(1);
        let wb = take_writes(&mb);
        assert_eq!(wb.len(), 1);
        assert_eq!(wb[0].0, 0x2000_0408);
        assert_eq!(wb[0].1 & CHCTL_NXTUSEBURST, CHCTL_NXTUSEBURST);
        assert_eq!(wb[0].1 & CHCTL_WB_MASK, 0, "XFERSIZE+XFERMODE cleared");
    }
}
