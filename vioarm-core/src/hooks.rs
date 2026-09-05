//! In-process hook registry: pre/post-instruction callbacks (PC-range
//! filtered), memory read/write watchpoints, and peripheral-access events.
//!
//! Hooks observe — callbacks receive a [`HookEvent`] and no machine handle,
//! so a hook cannot reenter the machine (safe by construction). Instruction
//! hooks fire from [`Machine::step`](crate::machine::Machine::step) around
//! the retired instruction (never for exception-entry steps, which retire
//! nothing, and never twice for a sleeping core); memory and peripheral
//! hooks fire from the bus after a successful access, byte-granular (one
//! event per byte of a wide access).
//!
//! With no hooks armed the cost is one predicted branch per step/access.
//! Rust API only: there is deliberately no dynamic-plugin ABI (an `.so`
//! boundary is a versioning trap — the registry plus `--trace jsonl` cover
//! the observed plugin use cases: tracing, coverage, watchpoints).

use std::collections::BTreeSet;

/// What a hook observed. All addresses are guest addresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HookEvent {
    /// An instruction retired at `pc` (`cycles` = `DWT CYCCNT` then).
    /// Pre-hooks see the pc about to retire; post-hooks the pc retired.
    Instruction { pc: u32, cycles: u64 },
    /// A byte was read from `addr`.
    MemRead { addr: u32, value: u8 },
    /// A byte was written to `addr`.
    MemWrite { addr: u32, value: u8 },
    /// A peripheral byte was accessed: device `name`, offset `addr`, and
    /// direction. Fires alongside (not instead of) matching watchpoints.
    Peripheral {
        name: &'static str,
        addr: u32,
        value: u8,
        write: bool,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Before,
    After,
    Read,
    Write,
    PeripheralRead,
    PeripheralWrite,
}

struct Entry {
    kind: Kind,
    /// PC range (`Before`/`After`) or address range (the rest), `start`
    /// inclusive, `end` exclusive.
    start: u32,
    end: u32,
    /// Peripheral-name filter (`Peripheral*` only; `None` = every device).
    peripheral: Option<String>,
    callback: Box<dyn FnMut(&HookEvent)>,
}

/// The registry. Empty by default; register through
/// [`Machine`](crate::machine::Machine)'s `on_*` methods.
#[derive(Default)]
pub struct Hooks {
    entries: Vec<Entry>,
    has_before: bool,
    has_after: bool,
    has_read: bool,
    has_write: bool,
    has_prd: bool,
    has_pwr: bool,
}

impl std::fmt::Debug for Hooks {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Hooks")
            .field("entries", &self.entries.len())
            .finish()
    }
}

impl Hooks {
    /// An empty registry, usable in `const` constructors.
    #[must_use]
    pub(crate) const fn empty() -> Self {
        Self {
            entries: Vec::new(),
            has_before: false,
            has_after: false,
            has_read: false,
            has_write: false,
            has_prd: false,
            has_pwr: false,
        }
    }
}

impl Hooks {
    /// A pre- or post-instruction hook over `start..end` (`before` selects
    /// which). `0..u32::MAX` covers every realistic PC.
    pub fn on_instruction(
        &mut self,
        before: bool,
        start: u32,
        end: u32,
        callback: impl FnMut(&HookEvent) + 'static,
    ) {
        if before {
            self.has_before = true;
        } else {
            self.has_after = true;
        }
        self.entries.push(Entry {
            kind: if before { Kind::Before } else { Kind::After },
            start,
            end,
            peripheral: None,
            callback: Box::new(callback),
        });
    }

    /// A read watchpoint over `start..end`.
    pub fn on_read(&mut self, start: u32, end: u32, callback: impl FnMut(&HookEvent) + 'static) {
        self.has_read = true;
        self.entries.push(Entry {
            kind: Kind::Read,
            start,
            end,
            peripheral: None,
            callback: Box::new(callback),
        });
    }

    /// A write watchpoint over `start..end`.
    pub fn on_write(&mut self, start: u32, end: u32, callback: impl FnMut(&HookEvent) + 'static) {
        self.has_write = true;
        self.entries.push(Entry {
            kind: Kind::Write,
            start,
            end,
            peripheral: None,
            callback: Box::new(callback),
        });
    }

    /// A peripheral-access hook: `write` selects direction, `name` (if
    /// given) restricts to one device (`Peripheral::name`, e.g. `"usart1"`).
    pub fn on_peripheral(
        &mut self,
        write: bool,
        name: Option<&str>,
        callback: impl FnMut(&HookEvent) + 'static,
    ) {
        if write {
            self.has_pwr = true;
        } else {
            self.has_prd = true;
        }
        self.entries.push(Entry {
            kind: if write {
                Kind::PeripheralWrite
            } else {
                Kind::PeripheralRead
            },
            start: 0,
            end: u32::MAX,
            peripheral: name.map(str::to_owned),
            callback: Box::new(callback),
        });
    }

    /// No callbacks registered at all (the fast path).
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        !self.has_before
            && !self.has_after
            && !self.has_read
            && !self.has_write
            && !self.has_prd
            && !self.has_pwr
    }

    pub(crate) const fn has_before(&self) -> bool {
        self.has_before
    }

    pub(crate) const fn has_after(&self) -> bool {
        self.has_after
    }

    pub(crate) const fn has_read(&self) -> bool {
        self.has_read
    }

    pub(crate) const fn has_write(&self) -> bool {
        self.has_write
    }

    pub(crate) const fn has_prd(&self) -> bool {
        self.has_prd
    }

    pub(crate) const fn has_pwr(&self) -> bool {
        self.has_pwr
    }

    pub(crate) fn fire_before(&mut self, pc: u32, cycles: u64) {
        let event = HookEvent::Instruction { pc, cycles };
        for e in &mut self.entries {
            if e.kind == Kind::Before && pc >= e.start && pc < e.end {
                (e.callback)(&event);
            }
        }
    }

    pub(crate) fn fire_after(&mut self, pc: u32, cycles: u64) {
        let event = HookEvent::Instruction { pc, cycles };
        for e in &mut self.entries {
            if e.kind == Kind::After && pc >= e.start && pc < e.end {
                (e.callback)(&event);
            }
        }
    }

    pub(crate) fn fire_read(&mut self, addr: u32, value: u8) {
        let event = HookEvent::MemRead { addr, value };
        for e in &mut self.entries {
            if e.kind == Kind::Read && addr >= e.start && addr < e.end {
                (e.callback)(&event);
            }
        }
    }

    pub(crate) fn fire_write(&mut self, addr: u32, value: u8) {
        let event = HookEvent::MemWrite { addr, value };
        for e in &mut self.entries {
            if e.kind == Kind::Write && addr >= e.start && addr < e.end {
                (e.callback)(&event);
            }
        }
    }

    pub(crate) fn fire_peripheral(
        &mut self,
        name: &'static str,
        addr: u32,
        value: u8,
        write: bool,
    ) {
        let event = HookEvent::Peripheral {
            name,
            addr,
            value,
            write,
        };
        for e in &mut self.entries {
            let kind = if write {
                Kind::PeripheralWrite
            } else {
                Kind::PeripheralRead
            };
            if e.kind == kind && e.peripheral.as_deref().is_none_or(|n| n == name) {
                (e.callback)(&event);
            }
        }
    }
}

/// Block coverage over executed PCs: leaders are the entry, every observed
/// branch/call target, and every fall-through after one; a block is covered
/// when its leader executed. Only 16-bit Thumb plus `BL` are decoded —
/// anything else ends a block conservatively (documented; the e2e program
/// is 16-bit-only).
#[derive(Debug, Default)]
pub struct Coverage {
    pcs: BTreeSet<u32>,
}

impl Coverage {
    /// An empty collection; feed it from a pre-instruction hook.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Record one retired PC.
    pub fn record(&mut self, pc: u32) {
        self.pcs.insert(pc);
    }

    /// The recorded PCs, in order.
    #[must_use]
    pub fn pcs(&self) -> Vec<u32> {
        self.pcs.iter().copied().collect()
    }

    /// Reconstruct blocks: `fetch` returns the halfword at `addr` (`None`
    /// ends the block). Returns `(leaders, covered)` with leaders sorted;
    /// `covered[i]` tells whether block `i` ran.
    #[must_use]
    pub fn analyze(&self, fetch: &mut dyn FnMut(u32) -> Option<u16>) -> (Vec<u32>, Vec<bool>) {
        if self.pcs.is_empty() {
            return (Vec::new(), Vec::new());
        }
        // Leaders: entry, branch targets, post-branch fall-throughs.
        let mut leaders = BTreeSet::new();
        leaders.insert(*self.pcs.first().unwrap_or(&0));
        for &pc in &self.pcs {
            let Some(h) = fetch(pc) else { continue };
            if let Some((target, fall)) = branch_edges(pc, h, fetch) {
                if let Some(t) = target {
                    leaders.insert(t);
                }
                if let Some(f) = fall {
                    leaders.insert(f);
                }
            }
        }
        let leaders: Vec<u32> = leaders.into_iter().collect();
        // A block runs from its leader to the next leader (or a
        // terminating instruction); covered iff the leader executed.
        let mut covered = Vec::with_capacity(leaders.len());
        for &l in &leaders {
            covered.push(self.pcs.contains(&l));
        }
        (leaders, covered)
    }
}

/// The edges out of the instruction at `pc`: `(target, fallthrough)`.
/// `target` is `None` where unknowable (returns); `fallthrough` is `None`
/// for terminal instructions (`BKPT`/`SVC`, which halt, never fall through).
fn branch_edges(
    pc: u32,
    h: u16,
    fetch: &mut dyn FnMut(u32) -> Option<u16>,
) -> Option<(Option<u32>, Option<u32>)> {
    // 16-bit branches address `PC` (= `pc + 4`) directly; 32-bit ones the
    // word-aligned PC.
    let pc16 = pc.wrapping_add(4);
    let pc32 = pc16 & !3;
    // Conditional branch `B<cond>`: target + fall-through.
    if h & 0xF000 == 0xD000 && h & 0x0F00 != 0x0F00 {
        let imm = u32::from(h & 0xFF);
        let off = if imm & 0x80 != 0 {
            imm | 0xFFFF_FF00
        } else {
            imm
        };
        let target = pc16.wrapping_add(off.wrapping_mul(2));
        return Some((Some(target), Some(pc.wrapping_add(2))));
    }
    // Unconditional `B`: target + (textbook) fall-through leader.
    if h & 0xF800 == 0xE000 {
        let imm = u32::from(h & 0x7FF);
        let off = if imm & 0x400 != 0 {
            imm | 0xFFFF_F800
        } else {
            imm
        };
        let target = pc16.wrapping_add(off.wrapping_mul(2));
        return Some((Some(target), Some(pc.wrapping_add(2))));
    }
    // `CBZ`/`CBNZ`: `1011 op 0 i 1 imm5 Rn`, forward-only `i:imm5:0`
    // (verified against `arm-none-eabi-as`: `cbz r0,+6` = `0xB108`,
    // `cbnz r0,+6` from `0x00` = `0xB908`).
    if h & 0xF500 == 0xB100 {
        let i = u32::from((h >> 9) & 1);
        let imm5 = u32::from((h >> 3) & 0x1F);
        let off = (i << 6) | (imm5 << 1);
        let target = pc16.wrapping_add(off);
        return Some((Some(target), Some(pc.wrapping_add(2))));
    }
    // `BL`: decoded call target + return address.
    if h & 0xF800 == 0xF000 {
        let h1 = fetch(pc.wrapping_add(2))?;
        if h1 & 0xF800 == 0xF800 {
            let s = u32::from((h >> 10) & 1);
            let j1 = u32::from((h1 >> 13) & 1);
            let j2 = u32::from((h1 >> 11) & 1);
            let i1 = (!(j1 ^ s)) & 1;
            let i2 = (!(j2 ^ s)) & 1;
            let imm = (s << 24)
                | (i1 << 23)
                | (i2 << 22)
                | (u32::from(h & 0x3FF) << 12)
                | (u32::from(h1 & 0x7FF) << 1);
            let off = if s != 0 { imm | 0xFF00_0000 } else { imm };
            let target = pc32.wrapping_add(off);
            return Some((Some(target), Some(pc.wrapping_add(4))));
        }
        return Some((None, Some(pc.wrapping_add(4))));
    }
    // `BX`/`BLX` register, `POP {pc}`: target unknown, fall-through leader.
    if h & 0xFF80 == 0x4700 || h & 0xFF00 == 0xBD00 {
        return Some((None, Some(pc.wrapping_add(2))));
    }
    // `BKPT`/`SVC`: terminal, no fall-through.
    if h & 0xFF00 == 0xBE00 || h & 0xFF00 == 0xDF00 {
        return Some((None, None));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::{Coverage, HookEvent, Hooks};

    #[test]
    fn registry_fires_by_kind_and_range() {
        use std::cell::RefCell;
        use std::rc::Rc;
        let mut h = Hooks::default();
        assert!(h.is_empty());
        let hits = Rc::new(RefCell::new(Vec::new()));
        let sink = hits.clone();
        h.on_instruction(true, 0x100, 0x200, move |e| {
            if let HookEvent::Instruction { pc, .. } = *e {
                sink.borrow_mut().push(pc);
            }
        });
        assert!(!h.is_empty());
        h.fire_before(0xFF, 0);
        h.fire_before(0x100, 1);
        h.fire_before(0x1FF, 2);
        h.fire_before(0x200, 3);
        h.fire_after(0x100, 4);
        assert_eq!(
            *hits.borrow(),
            [0x100, 0x1FF],
            "range is start-inclusive, end-exclusive"
        );
    }

    #[test]
    fn watchpoints_fire_per_byte() {
        use std::cell::RefCell;
        use std::rc::Rc;
        let mut h = Hooks::default();
        let writes = Rc::new(RefCell::new(Vec::new()));
        let sink = writes.clone();
        h.on_write(0x2000_0000, 0x2000_0004, move |e| {
            if let HookEvent::MemWrite { addr, value } = *e {
                sink.borrow_mut().push((addr, value));
            }
        });
        h.fire_write(0x1FFF_FFFF, 9);
        h.fire_write(0x2000_0000, 1);
        h.fire_write(0x2000_0003, 2);
        h.fire_write(0x2000_0004, 3);
        assert_eq!(*writes.borrow(), [(0x2000_0000, 1), (0x2000_0003, 2)]);
    }

    #[test]
    fn peripheral_name_filter() {
        use std::cell::RefCell;
        use std::rc::Rc;
        let mut h = Hooks::default();
        let hits = Rc::new(RefCell::new(Vec::new()));
        let sink = hits.clone();
        h.on_peripheral(false, Some("usart1"), move |e| {
            if let HookEvent::Peripheral { name, addr, .. } = *e {
                sink.borrow_mut().push((name, addr));
            }
        });
        h.on_peripheral(true, None, |_| {});
        h.fire_peripheral("usart1", 0x2C, 0x41, false);
        h.fire_peripheral("usart2", 0x2C, 0x41, false);
        assert_eq!(*hits.borrow(), [("usart1", 0x2C)]);
        assert!(h.has_pwr() && h.has_prd());
    }

    #[test]
    fn coverage_finds_untaken_else() {
        // 0x00: CBNZ r0, 0x06 ; 0x02: B 0x08 ; 0x04: else (never runs) ;
        // 0x06: ; 0x08: BKPT. Run with r0 == 0: PCs 00, 02, 08
        // (`0xB908` verified with `arm-none-eabi-as`).
        let prog: [u16; 5] = [0xB908, 0xE001, 0x2001, 0x2002, 0xBE00];
        let mut fetch = |addr: u32| prog.get((addr / 2) as usize).copied();
        let mut c = Coverage::new();
        for pc in [0x00, 0x02, 0x08] {
            c.record(pc);
        }
        let (leaders, covered) = c.analyze(&mut fetch);
        assert_eq!(leaders, [0x00, 0x02, 0x04, 0x06, 0x08]);
        assert_eq!(covered, [true, true, false, false, true]);
    }
}
