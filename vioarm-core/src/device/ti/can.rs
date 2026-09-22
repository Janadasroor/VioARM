//! TI Tiva C CAN controllers (`CAN0`-`CAN1`, Bosch C_CAN core).
//!
//! Message-object operation follows the TM4C123GH6PM datasheet (SPMS376E
//! §17): `INIT`-gated transfers with `CCE`-gated bit timing, 32 message
//! objects behind the `IF1`/`IF2` command/mask/arb/control/data windows
//! (`MNUM` 0→32, `0x21`-`0x3F`→`0x01`-`0x1F`), TX completing into the host
//! mailbox with `TXOK` + `INTPND` (when `TXIE`), RX filtering by ID/mask
//! (`UMASK`, `MDIR`/`MXTD`) into `NEWDAT` objects with `MSGLST` on
//! overwrite, remote-frame auto-response (`RMTEN`), `TXRQ`/`NWDA`/
//! `MSGINT`/`MSGVAL` synthesis, and `CANINT` object/status priority with
//! status-clear on `STS` read. `TXOK`/`RXOK`/`LEC` are CPU-cleared.
//!
//! Like the repo's other serials, transfers complete synchronously on the
//! `CRQ` write — no bit timing is modeled (`BUSY` reads 0, error counters
//! stay 0, `BOFF`/`EWARN`/`EPASS` never set; documented). Pending
//! `TXRQST` objects flush when `INIT` clears. Multi-object FIFO chains
//! (`EOB=0`) deliver to every match (documented approximation). Exact-ID
//! paths are bit-exact; masked-`MXTD=0` cross-width compares use the low
//! 11 bits (documented).

use crate::device::svd::{Device, PeripheralDef};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

// Control/status offsets.
const CTL_OFF: u32 = 0x000;
const STS_OFF: u32 = 0x004;
const ERR_OFF: u32 = 0x008;
const BIT_OFF: u32 = 0x00C;
const INT_OFF: u32 = 0x010;
const TST_OFF: u32 = 0x014;
const BRPE_OFF: u32 = 0x018;
// IF register sets.
const IF1_BASE: u32 = 0x020;
const IF2_BASE: u32 = 0x080;
const IF_CRQ: u32 = 0x00;
const IF_CMSK: u32 = 0x04;
const IF_MSK1: u32 = 0x08;
const IF_MSK2: u32 = 0x0C;
const IF_ARB1: u32 = 0x10;
const IF_ARB2: u32 = 0x14;
const IF_MCTL: u32 = 0x18;
const IF_DA1: u32 = 0x1C;
const IF_DA2: u32 = 0x20;
const IF_DB1: u32 = 0x24;
const IF_DB2: u32 = 0x28;
// Status blocks.
const TXRQ1_OFF: u32 = 0x100;
const TXRQ2_OFF: u32 = 0x104;
const NWDA1_OFF: u32 = 0x120;
const NWDA2_OFF: u32 = 0x124;
const MSG1INT_OFF: u32 = 0x140;
const MSG2INT_OFF: u32 = 0x144;
const MSG1VAL_OFF: u32 = 0x160;
const MSG2VAL_OFF: u32 = 0x164;

/// `CTL` bits: `TEST` 7, `CCE` 6, `DAR` 5, `EIE` 3, `SIE` 2, `IE` 1, `INIT` 0.
const CTL_TEST: u32 = 1 << 7;
const CTL_CCE: u32 = 1 << 6;
const CTL_DAR: u32 = 1 << 5;
const CTL_EIE: u32 = 1 << 3;
const CTL_SIE: u32 = 1 << 2;
const CTL_IE: u32 = 1 << 1;
const CTL_INIT: u32 = 1 << 0;
/// `STS` bits: `BOFF` 7, `EWARN` 6, `EPASS` 5, `RXOK` 4, `TXOK` 3, `LEC` 2:0.
const STS_RXOK: u32 = 1 << 4;
const STS_TXOK: u32 = 1 << 3;
/// `CMSK` bits.
const CMSK_WRNRD: u32 = 1 << 7;
const CMSK_MASK: u32 = 1 << 6;
const CMSK_ARB: u32 = 1 << 5;
const CMSK_CONTROL: u32 = 1 << 4;
const CMSK_CLRINTPND: u32 = 1 << 3;
const CMSK_TXRQST_NEWDAT: u32 = 1 << 2;
const CMSK_DATAA: u32 = 1 << 1;
const CMSK_DATAB: u32 = 1 << 0;
/// `ARB2` bits: `MSGVAL` 15, `XTD` 14, `DIR` 13.
const ARB2_MSGVAL: u32 = 1 << 15;
const ARB2_XTD: u32 = 1 << 14;
const ARB2_DIR: u32 = 1 << 13;
/// `MSK2` bits: `MXTD` 15, `MDIR` 14.
const MSK2_MXTD: u32 = 1 << 15;
const MSK2_MDIR: u32 = 1 << 14;
/// `MCTL` bits.
const MCTL_NEWDAT: u32 = 1 << 15;
const MCTL_MSGLST: u32 = 1 << 14;
const MCTL_INTPND: u32 = 1 << 13;
const MCTL_UMASK: u32 = 1 << 12;
const MCTL_TXIE: u32 = 1 << 11;
const MCTL_RXIE: u32 = 1 << 10;
const MCTL_RMTEN: u32 = 1 << 9;
const MCTL_TXRQST: u32 = 1 << 8;
const MCTL_EOB: u32 = 1 << 7;

/// One message object (the message RAM word set).
#[derive(Clone, Copy, Debug, Default)]
struct MsgObject {
    msk1: u16,
    msk2: u16,
    arb1: u16,
    arb2: u16,
    mctl: u16,
    data: [u8; 8],
}

/// One interface register set (`IF1`/`IF2`).
#[derive(Clone, Copy, Debug, Default)]
struct IfRegs {
    crq: u16,
    cmsk: u16,
    msk1: u16,
    msk2: u16,
    arb1: u16,
    arb2: u16,
    mctl: u16,
    da1: u16,
    da2: u16,
    db1: u16,
    db2: u16,
}

/// A transmitted frame, for the host to drain.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanFrame {
    /// 11-bit standard or 29-bit extended identifier.
    pub id: u32,
    /// Extended (29-bit) identifier format.
    pub extended: bool,
    /// Remote-transmission-request frame.
    pub rtr: bool,
    /// Payload bytes (`DLC` length).
    pub data: Vec<u8>,
}

/// Shared controller state behind the window.
#[derive(Debug)]
pub struct CanCore {
    ctl: u32,
    sts: u32,
    err: u32,
    bit: u32,
    tst: u32,
    brpe: u32,
    objects: [MsgObject; 32],
    ifs: [IfRegs; 2],
    tx_out: Vec<CanFrame>,
    status_int: bool,
    pending: bool,
}

/// Behavior behind one CAN window.
pub struct CanBehavior {
    def: &'static PeripheralDef,
    shared: Arc<Mutex<CanCore>>,
}

impl Default for CanCore {
    fn default() -> Self {
        Self {
            ctl: CTL_INIT,
            sts: 0,
            err: 0,
            bit: 0x2301,
            tst: 0,
            brpe: 0,
            objects: [MsgObject::default(); 32],
            ifs: [IfRegs::default(); 2],
            tx_out: Vec::new(),
            status_int: false,
            pending: false,
        }
    }
}

impl CanBehavior {
    /// Object index (0-based) from an `MNUM` value: 0→32, `0x21`-`0x3F`
    /// fold to `0x01`-`0x1F` (datasheet `CRQ` page).
    fn obj_index(mnum: u16) -> usize {
        let m = mnum & 0x3F;
        if m == 0 {
            31
        } else if m > 0x20 {
            (m - 0x21) as usize
        } else {
            (m - 1) as usize
        }
    }

    /// 29-bit ID + XTD flag from an `ARB1`/`ARB2` pair.
    fn arb_id(arb1: u16, arb2: u16) -> (u32, bool) {
        if arb2 & ARB2_XTD as u16 != 0 {
            (u32::from(arb2 & 0x1FFF) << 16 | u32::from(arb1), true)
        } else {
            (u32::from(arb2 >> 2) & 0x07FF, false)
        }
    }

    /// Whether `frame` matches `obj` for reception.
    fn matches(obj: &MsgObject, id: u32, extended: bool, rtr: bool) -> bool {
        if obj.arb2 & ARB2_MSGVAL as u16 == 0 {
            return false;
        }
        if obj.arb2 & ARB2_DIR as u16 != 0 {
            return false; // Transmit direction.
        }
        let (oid, oxtd) = Self::arb_id(obj.arb1, obj.arb2);
        if obj.mctl & MCTL_UMASK as u16 == 0 {
            // Mask ignored: exact ID + XTD match.
            return oid == id && oxtd == extended && !rtr;
        }
        if obj.msk2 & MSK2_MXTD as u16 != 0 && oxtd != extended {
            return false;
        }
        if obj.msk2 & MSK2_MDIR as u16 != 0 && rtr {
            return false; // Direction-gated: data frames only here.
        }
        if oxtd || extended {
            if obj.msk2 & MSK2_MXTD as u16 == 0 {
                // Cross-width: low 11 bits.
                return oid & 0x07FF == id & 0x07FF;
            }
            let mask = u32::from(obj.msk2 & 0x1FFF) << 16 | u32::from(obj.msk1);
            (oid ^ id) & mask == 0
        } else {
            let mask = u32::from(obj.msk2 >> 2) & 0x07FF;
            (oid ^ id) & mask == 0
        }
    }

    /// Transmit object `idx` now (instant completion).
    fn transmit(core: &mut CanCore, idx: usize) {
        let obj = &mut core.objects[idx];
        let (id, extended) = Self::arb_id(obj.arb1, obj.arb2);
        let dlc = (obj.mctl & 0x0F).min(8) as usize;
        let rtr = obj.mctl & MCTL_RMTEN as u16 != 0;
        let data = obj.data[..dlc].to_vec();
        core.tx_out.push(CanFrame {
            id,
            extended,
            rtr,
            data,
        });
        obj.mctl &= !MCTL_TXRQST as u16;
        core.sts |= STS_TXOK;
        core.status_int = true;
        if obj.mctl & MCTL_TXIE as u16 != 0 {
            obj.mctl |= MCTL_INTPND as u16;
        }
        Self::refresh(core, core.ctl);
    }

    /// Deliver a received data frame to matching objects.
    fn deliver(core: &mut CanCore, id: u32, extended: bool, data: &[u8]) {
        for idx in 0..32 {
            let obj = &mut core.objects[idx];
            if !Self::matches(obj, id, extended, false) {
                continue;
            }
            if obj.mctl & MCTL_NEWDAT as u16 != 0 {
                obj.mctl |= MCTL_MSGLST as u16;
            }
            let dlc = data.len().min(8);
            obj.data[..dlc].copy_from_slice(&data[..dlc]);
            obj.mctl = obj.mctl & !0x0F | dlc as u16;
            obj.mctl |= MCTL_NEWDAT as u16;
            if obj.mctl & MCTL_RXIE as u16 != 0 {
                obj.mctl |= MCTL_INTPND as u16;
            }
            core.sts |= STS_RXOK;
            core.status_int = true;
            if obj.mctl & MCTL_EOB as u16 != 0 {
                break;
            }
        }
        Self::refresh(core, core.ctl);
    }

    /// Deliver a received remote frame: `MSGVAL` + receive-direction
    /// + `RMTEN` objects with a matching ID answer automatically.
    ///
    /// (`matches` is data-frame-only, so the ID check runs inline here.)
    fn deliver_remote(core: &mut CanCore, id: u32, extended: bool) {
        for idx in 0..32 {
            let obj = &core.objects[idx];
            if obj.arb2 & ARB2_MSGVAL as u16 == 0
                || obj.arb2 & ARB2_DIR as u16 != 0
                || obj.mctl & MCTL_RMTEN as u16 == 0
            {
                continue;
            }
            let (oid, oxtd) = Self::arb_id(obj.arb1, obj.arb2);
            let id_ok = if obj.mctl & MCTL_UMASK as u16 == 0 {
                oid == id && oxtd == extended
            } else if obj.msk2 & MSK2_MXTD as u16 != 0 && oxtd != extended {
                false
            } else {
                let mask = u32::from(obj.msk2 & 0x1FFF) << 16 | u32::from(obj.msk1);
                (oid ^ id) & mask == 0
            };
            if id_ok {
                core.objects[idx].mctl |= MCTL_TXRQST as u16;
                if core.ctl & CTL_INIT == 0 {
                    Self::transmit(core, idx);
                }
            }
        }
    }

    /// Current `CANINT` value: lowest `INTPND` object outranks the
    /// status interrupt (`0x8000`), or 0 when nothing pends.
    fn int_value(core: &CanCore) -> u32 {
        for (i, obj) in core.objects.iter().enumerate() {
            if obj.mctl & MCTL_INTPND as u16 != 0 {
                return i as u32 + 1;
            }
        }
        if core.status_int {
            return 0x8000;
        }
        0
    }

    fn refresh(core: &mut CanCore, ctl: u32) {
        core.pending = ctl & CTL_IE != 0 && Self::int_value(core) != 0;
    }

    /// [`refresh`](Self::refresh) without split borrows (reads `ctl` first).
    fn refresh_locked(core: &mut CanCore) {
        let ctl = core.ctl;
        Self::refresh(core, ctl);
    }

    /// Execute an `IF` transfer for interface `ifs` (`0`/`1`).
    fn transfer(core: &mut CanCore, which: usize) {
        let mnum = core.ifs[which].crq;
        let idx = Self::obj_index(mnum);
        let cmsk = u32::from(core.ifs[which].cmsk);
        let wr = cmsk & CMSK_WRNRD != 0;
        if wr {
            // Interface → object.
            let dst = &mut core.objects[idx];
            if cmsk & CMSK_MASK != 0 {
                dst.msk1 = core.ifs[which].msk1;
                dst.msk2 = core.ifs[which].msk2;
            }
            if cmsk & CMSK_ARB != 0 {
                dst.arb1 = core.ifs[which].arb1;
                dst.arb2 = core.ifs[which].arb2;
            }
            if cmsk & CMSK_CONTROL != 0 {
                dst.mctl = core.ifs[which].mctl;
            }
            if cmsk & CMSK_CLRINTPND != 0 {
                dst.mctl &= !MCTL_INTPND as u16;
            }
            if cmsk & CMSK_TXRQST_NEWDAT != 0 {
                dst.mctl |= MCTL_TXRQST as u16;
            }
            if cmsk & CMSK_DATAA != 0 {
                dst.data[0] = (core.ifs[which].da1 & 0xFF) as u8;
                dst.data[1] = (core.ifs[which].da1 >> 8) as u8;
                dst.data[2] = (core.ifs[which].da2 & 0xFF) as u8;
                dst.data[3] = (core.ifs[which].da2 >> 8) as u8;
            }
            if cmsk & CMSK_DATAB != 0 {
                dst.data[4] = (core.ifs[which].db1 & 0xFF) as u8;
                dst.data[5] = (core.ifs[which].db1 >> 8) as u8;
                dst.data[6] = (core.ifs[which].db2 & 0xFF) as u8;
                dst.data[7] = (core.ifs[which].db2 >> 8) as u8;
            }
            if dst.mctl & MCTL_TXRQST as u16 != 0 && core.ctl & CTL_INIT == 0 {
                Self::transmit(core, idx);
            }
        } else {
            // Object → interface.
            let src = core.objects[idx];
            if cmsk & CMSK_MASK != 0 {
                core.ifs[which].msk1 = src.msk1;
                core.ifs[which].msk2 = src.msk2;
            }
            if cmsk & CMSK_ARB != 0 {
                core.ifs[which].arb1 = src.arb1;
                core.ifs[which].arb2 = src.arb2;
            }
            if cmsk & CMSK_CONTROL != 0 {
                core.ifs[which].mctl = src.mctl;
            }
            if cmsk & CMSK_CLRINTPND != 0 {
                core.objects[idx].mctl &= !MCTL_INTPND as u16;
            }
            if cmsk & CMSK_TXRQST_NEWDAT != 0 {
                core.objects[idx].mctl &= !MCTL_NEWDAT as u16;
            }
            if cmsk & CMSK_DATAA != 0 {
                let o = core.objects[idx];
                core.ifs[which].da1 = u16::from(o.data[0]) | u16::from(o.data[1]) << 8;
                core.ifs[which].da2 = u16::from(o.data[2]) | u16::from(o.data[3]) << 8;
            }
            if cmsk & CMSK_DATAB != 0 {
                let o = core.objects[idx];
                core.ifs[which].db1 = u16::from(o.data[4]) | u16::from(o.data[5]) << 8;
                core.ifs[which].db2 = u16::from(o.data[6]) | u16::from(o.data[7]) << 8;
            }
        }
        Self::refresh(core, core.ctl);
    }
}

/// A Tiva C CAN controller (message objects + IF windows).
pub struct Can {
    ranges: [Region; 1],
    exception: Option<u8>,
    inner: Mutex<CanBehavior>,
}

impl Can {
    /// A new CAN controller on `D`'s tables (`"CAN0"`/`"CAN1"`).
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `name`.
    #[must_use]
    pub fn for_device<D: Device>(name: &str) -> Self {
        let def = D::peripheral(name).expect("generated tables contain CAN");
        Self {
            ranges: [Region::new(def.base, def.window)],
            exception: def.irqs.first().map(|irq| 16 + irq.number),
            inner: Mutex::new(CanBehavior {
                def,
                shared: Arc::new(Mutex::new(CanCore::default())),
            }),
        }
    }

    fn offset_in_window(&self, addr: u32) -> Result<u32, BusError> {
        let region = self.ranges[0];
        addr.checked_sub(region.base)
            .filter(|o| *o < region.len)
            .ok_or(BusError::Reserved)
    }

    /// Frames firmware transmitted, for the host to drain.
    pub fn drain_output(&mut self) -> Vec<CanFrame> {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        std::mem::take(&mut core.tx_out)
    }

    /// Feed a received frame (host/test stimulus).
    pub fn inject_frame(&mut self, id: u32, extended: bool, rtr: bool, data: &[u8]) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        if core.ctl & CTL_INIT != 0 {
            return;
        }
        if rtr {
            CanBehavior::deliver_remote(&mut core, id, extended);
        } else {
            CanBehavior::deliver(&mut core, id, extended, data);
        }
    }

    /// Read an interface register word (test hook).
    pub fn if_word(&self, which: usize, field: u32) -> u16 {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        let r = core.ifs[which];
        match field {
            IF_MSK1 => r.msk1,
            IF_MSK2 => r.msk2,
            IF_ARB1 => r.arb1,
            IF_ARB2 => r.arb2,
            IF_MCTL => r.mctl,
            IF_DA1 => r.da1,
            IF_DA2 => r.da2,
            IF_DB1 => r.db1,
            IF_DB2 => r.db2,
            _ => 0,
        }
    }
}

impl Peripheral for Can {
    fn ranges(&self) -> &[Region] {
        &self.ranges
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let off = self.offset_in_window(addr)?;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        // IF windows live in the shared core (the SVD names them twice).
        for (i, base) in [IF1_BASE, IF2_BASE].iter().copied().enumerate() {
            if (base + IF_CRQ..base + IF_DB2 + 4).contains(&off) {
                let field = (off - base) & !3;
                let word = match field {
                    IF_CRQ => u32::from(core.ifs[i].crq),
                    IF_CMSK => u32::from(core.ifs[i].cmsk),
                    IF_MSK1 => u32::from(core.ifs[i].msk1),
                    IF_MSK2 => u32::from(core.ifs[i].msk2),
                    IF_ARB1 => u32::from(core.ifs[i].arb1),
                    IF_ARB2 => u32::from(core.ifs[i].arb2),
                    IF_MCTL => u32::from(core.ifs[i].mctl),
                    IF_DA1 => u32::from(core.ifs[i].da1),
                    IF_DA2 => u32::from(core.ifs[i].da2),
                    IF_DB1 => u32::from(core.ifs[i].db1),
                    IF_DB2 => u32::from(core.ifs[i].db2),
                    _ => return Err(BusError::Reserved),
                };
                return Ok(((word >> (8 * (off % 4))) & 0xFF) as u8);
            }
        }
        let word = match off & !3 {
            CTL_OFF => core.ctl,
            STS_OFF => {
                let w = core.sts;
                // Reading `STS` clears a pending status interrupt.
                core.status_int = false;
                CanBehavior::refresh_locked(&mut core);
                w
            }
            ERR_OFF => core.err,
            BIT_OFF => core.bit,
            INT_OFF => CanBehavior::int_value(&core),
            TST_OFF => core.tst,
            BRPE_OFF => core.brpe,
            TXRQ1_OFF => core.objects[..16]
                .iter()
                .enumerate()
                .fold(0u32, |a, (i, o)| {
                    a | (u32::from(o.mctl & MCTL_TXRQST as u16 != 0) << i)
                }),
            TXRQ2_OFF => core.objects[16..]
                .iter()
                .enumerate()
                .fold(0u32, |a, (i, o)| {
                    a | (u32::from(o.mctl & MCTL_TXRQST as u16 != 0) << i)
                }),
            NWDA1_OFF => core.objects[..16]
                .iter()
                .enumerate()
                .fold(0u32, |a, (i, o)| {
                    a | (u32::from(o.mctl & MCTL_NEWDAT as u16 != 0) << i)
                }),
            NWDA2_OFF => core.objects[16..]
                .iter()
                .enumerate()
                .fold(0u32, |a, (i, o)| {
                    a | (u32::from(o.mctl & MCTL_NEWDAT as u16 != 0) << i)
                }),
            MSG1INT_OFF => core.objects[..16]
                .iter()
                .enumerate()
                .fold(0u32, |a, (i, o)| {
                    a | (u32::from(o.mctl & MCTL_INTPND as u16 != 0) << i)
                }),
            MSG2INT_OFF => core.objects[16..]
                .iter()
                .enumerate()
                .fold(0u32, |a, (i, o)| {
                    a | (u32::from(o.mctl & MCTL_INTPND as u16 != 0) << i)
                }),
            MSG1VAL_OFF => core.objects[..16]
                .iter()
                .enumerate()
                .fold(0u32, |a, (i, o)| {
                    a | (u32::from(o.arb2 & ARB2_MSGVAL as u16 != 0) << i)
                }),
            MSG2VAL_OFF => core.objects[16..]
                .iter()
                .enumerate()
                .fold(0u32, |a, (i, o)| {
                    a | (u32::from(o.arb2 & ARB2_MSGVAL as u16 != 0) << i)
                }),
            _ => return Err(BusError::Reserved),
        };
        Ok(((word >> (8 * (off % 4))) & 0xFF) as u8)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        let off = self.offset_in_window(addr)?;
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        for (i, base) in [IF1_BASE, IF2_BASE].iter().copied().enumerate() {
            if (base + IF_CRQ..base + IF_DB2 + 4).contains(&off) {
                let field = (off - base) & !3;
                let lane = off % 4;
                let shift = 8 * lane;
                // `CRQ` fires the transfer once the low lane lands (firmware
                // word-stores the message number).
                if field == IF_CRQ {
                    let cur = u32::from(core.ifs[i].crq);
                    let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
                    core.ifs[i].crq = (word & 0x3F) as u16;
                    if lane == 0 {
                        CanBehavior::transfer(&mut core, i);
                    }
                    return Ok(());
                }
                let cur = match field {
                    IF_CMSK => u32::from(core.ifs[i].cmsk),
                    IF_MSK1 => u32::from(core.ifs[i].msk1),
                    IF_MSK2 => u32::from(core.ifs[i].msk2),
                    IF_ARB1 => u32::from(core.ifs[i].arb1),
                    IF_ARB2 => u32::from(core.ifs[i].arb2),
                    IF_MCTL => u32::from(core.ifs[i].mctl),
                    IF_DA1 => u32::from(core.ifs[i].da1),
                    IF_DA2 => u32::from(core.ifs[i].da2),
                    IF_DB1 => u32::from(core.ifs[i].db1),
                    IF_DB2 => u32::from(core.ifs[i].db2),
                    _ => return Err(BusError::Reserved),
                };
                let word = ((cur & !(0xFF << shift)) | (u32::from(val) << shift)) & 0xFFFF;
                match field {
                    IF_CMSK => core.ifs[i].cmsk = word as u16,
                    IF_MSK1 => core.ifs[i].msk1 = word as u16,
                    IF_MSK2 => core.ifs[i].msk2 = word as u16,
                    IF_ARB1 => core.ifs[i].arb1 = word as u16,
                    IF_ARB2 => core.ifs[i].arb2 = word as u16,
                    IF_MCTL => core.ifs[i].mctl = word as u16,
                    IF_DA1 => core.ifs[i].da1 = word as u16,
                    IF_DA2 => core.ifs[i].da2 = word as u16,
                    IF_DB1 => core.ifs[i].db1 = word as u16,
                    IF_DB2 => core.ifs[i].db2 = word as u16,
                    _ => {}
                }
                return Ok(());
            }
        }
        let lane = off % 4;
        let shift = 8 * lane;
        match off & !3 {
            CTL_OFF => {
                let was_init = core.ctl & CTL_INIT != 0;
                let word = (core.ctl & !(0xFF << shift)) | (u32::from(val) << shift);
                // Reserved upper half and bit 4 read 0.
                core.ctl = word & 0x0000_00EF;
                if was_init && core.ctl & CTL_INIT == 0 {
                    // Leaving init flushes held `TXRQST` objects.
                    for idx in 0..32 {
                        if core.objects[idx].mctl & MCTL_TXRQST as u16 != 0 {
                            CanBehavior::transmit(&mut core, idx);
                        }
                    }
                }
                CanBehavior::refresh_locked(&mut core);
            }
            STS_OFF => {
                // `TXOK`/`RXOK`/`LEC` take written values; the rest is RO.
                let w = (u32::from(val) << shift) & (STS_TXOK | STS_RXOK | 0x07);
                core.sts = (core.sts & !(0xFF << shift)) | w;
                CanBehavior::refresh_locked(&mut core);
            }
            BIT_OFF => {
                // Gated by `CCE && INIT` (datasheet `CANCTL` page).
                if core.ctl & (CTL_CCE | CTL_INIT) == CTL_CCE | CTL_INIT {
                    core.bit = (core.bit & !(0xFF << shift)) | (u32::from(val) << shift);
                }
            }
            TST_OFF => {
                core.tst = (core.tst & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            BRPE_OFF => {
                core.brpe = (core.brpe & !(0xFF << shift)) | (u32::from(val) << shift);
            }
            _ => return Err(BusError::Reserved),
        }
        Ok(())
    }

    fn reset(&mut self) {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        *core = CanCore::default();
    }

    fn tick(&mut self, _delta: u64) {}
}

impl InterruptSource for Can {
    fn exception_number(&self) -> Option<u8> {
        self.exception
    }

    fn pending(&self) -> u32 {
        let b = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let core = b.shared.lock().unwrap_or_else(|e| e.into_inner());
        u32::from(core.pending)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAN0_BASE: u32 = 0x4004_0000;
    /// `CMSK`: write + `ARB` + `CONTROL` + `TXRQST` + `DATAA` + `DATAB`.
    const CMSK_SETUP_TX: u32 =
        CMSK_WRNRD | CMSK_ARB | CMSK_CONTROL | CMSK_TXRQST_NEWDAT | CMSK_DATAA | CMSK_DATAB;
    /// `CMSK`: read + `ARB` + `CONTROL` + `DATAA` + `DATAB` + clear flags.
    const CMSK_READ_ALL: u32 =
        CMSK_ARB | CMSK_CONTROL | CMSK_DATAA | CMSK_DATAB | CMSK_CLRINTPND | CMSK_TXRQST_NEWDAT;

    fn can0() -> Can {
        Can::for_device::<crate::device::svd::tm4c123gh6pm::Tm4c123gh6pm>("CAN0")
    }

    fn read_word(can: &mut Can, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(can.read(CAN0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(can: &mut Can, offset: u32, value: u32) {
        for lane in 0..4 {
            can.write(
                CAN0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Leave init with interrupts on.
    fn start(can: &mut Can) {
        write_word(can, CTL_OFF, CTL_IE);
    }

    /// Standard-ID TX object (11-bit `id`, `dlc`, `DIR=1`, `MSGVAL`).
    fn setup_tx(can: &mut Can, obj: u32, id: u32, dlc: u32, data: &[u8]) {
        let arb2 = ARB2_MSGVAL | ARB2_DIR | ((id & 0x07FF) << 2);
        write_word(can, IF1_BASE + IF_ARB2, arb2);
        write_word(can, IF1_BASE + IF_ARB1, 0);
        write_word(can, IF1_BASE + IF_MCTL, MCTL_TXIE | MCTL_EOB | (dlc & 0x0F));
        for (i, chunk) in data.chunks(2).enumerate() {
            let w = u16::from(chunk[0]) | u16::from(*chunk.get(1).unwrap_or(&0)) << 8;
            write_word(can, IF1_BASE + IF_DA1 + 4 * i as u32, u32::from(w));
        }
        write_word(can, IF1_BASE + IF_CMSK, CMSK_SETUP_TX);
        write_word(can, IF1_BASE + IF_CRQ, obj);
    }

    /// Standard-ID RX object (exact match, `DIR=0`, `MSGVAL`).
    fn setup_rx(can: &mut Can, obj: u32, id: u32) {
        let arb2 = ARB2_MSGVAL | ((id & 0x07FF) << 2);
        write_word(can, IF1_BASE + IF_ARB2, arb2);
        write_word(can, IF1_BASE + IF_ARB1, 0);
        write_word(can, IF1_BASE + IF_MCTL, MCTL_RXIE | MCTL_EOB | 8);
        write_word(
            can,
            IF1_BASE + IF_CMSK,
            CMSK_WRNRD | CMSK_MASK | CMSK_ARB | CMSK_CONTROL | CMSK_DATAA | CMSK_DATAB,
        );
        write_word(can, IF1_BASE + IF_CRQ, obj);
    }

    /// Reset state: `INIT` set, interrupts quiet.
    #[test]
    fn reset_state() {
        let mut can = can0();
        assert_eq!(read_word(&mut can, CTL_OFF) & CTL_INIT, CTL_INIT);
        assert_eq!(read_word(&mut can, INT_OFF), 0);
        assert_eq!(can.pending(), 0);
    }

    /// TX object transmits on `TXRQST`: mailbox frame, `TXOK`, `INTPND`,
    /// `CANINT` names the object, `TXRQ` clears.
    #[test]
    fn tx_flow() {
        let mut can = can0();
        start(&mut can);
        setup_tx(&mut can, 1, 0x123, 2, &[0xDE, 0xAD]);
        let frames = can.drain_output();
        assert_eq!(
            frames,
            [CanFrame {
                id: 0x123,
                extended: false,
                rtr: false,
                data: vec![0xDE, 0xAD]
            }]
        );
        assert_eq!(read_word(&mut can, STS_OFF) & STS_TXOK, STS_TXOK);
        assert_eq!(read_word(&mut can, INT_OFF), 1);
        assert_eq!(can.pending(), 1);
        assert_eq!(can.exception_number(), Some(55));
        assert_eq!(read_word(&mut can, TXRQ1_OFF), 0);
        // `TXOK` is CPU-cleared; the status read drops the status line.
        write_word(&mut can, STS_OFF, 0x00);
        assert_eq!(read_word(&mut can, STS_OFF) & STS_TXOK, 0);
    }

    /// While `INIT` holds, `TXRQST` waits; leaving init flushes it.
    #[test]
    fn init_holds_tx() {
        let mut can = can0();
        assert_eq!(read_word(&mut can, CTL_OFF) & CTL_INIT, CTL_INIT);
        setup_tx(&mut can, 2, 0x200, 1, &[0x01]);
        assert_eq!(can.drain_output(), []);
        assert_eq!(read_word(&mut can, TXRQ1_OFF) & (1 << 1), 1 << 1);
        start(&mut can);
        assert_eq!(can.drain_output().len(), 1);
        assert_eq!(read_word(&mut can, TXRQ1_OFF), 0);
    }

    /// Injected frames filter by ID into `NEWDAT` objects with `RXOK`;
    /// reading via `IF` returns data and clears flags.
    #[test]
    fn rx_filter_flow() {
        let mut can = can0();
        start(&mut can);
        setup_rx(&mut can, 3, 0x123);
        can.inject_frame(0x456, false, false, &[0xAA]); // no match
        assert_eq!(read_word(&mut can, NWDA1_OFF), 0);
        can.inject_frame(0x123, false, false, &[0xAA, 0xBB]);
        assert_eq!(read_word(&mut can, NWDA1_OFF) & (1 << 2), 1 << 2);
        assert_eq!(read_word(&mut can, STS_OFF) & STS_RXOK, STS_RXOK);
        assert_eq!(read_word(&mut can, INT_OFF), 3);
        // Read object 3 back through IF2.
        write_word(&mut can, IF2_BASE + IF_CMSK, CMSK_READ_ALL);
        write_word(&mut can, IF2_BASE + IF_CRQ, 3);
        assert_eq!(can.if_word(1, IF_DA1), 0xBBAA);
        assert_eq!(read_word(&mut can, NWDA1_OFF), 0);
        assert_eq!(read_word(&mut can, INT_OFF) & 0x7FFF, 0);
    }

    /// Overwriting an unread object sets `MSGLST`.
    #[test]
    fn overwrite_sets_msglst() {
        let mut can = can0();
        start(&mut can);
        setup_rx(&mut can, 4, 0x100);
        can.inject_frame(0x100, false, false, &[0x01]);
        can.inject_frame(0x100, false, false, &[0x02]);
        // Read the object control word: `MSGLST` latched.
        write_word(&mut can, IF2_BASE + IF_CMSK, CMSK_ARB | CMSK_CONTROL);
        write_word(&mut can, IF2_BASE + IF_CRQ, 4);
        assert_eq!(
            can.if_word(1, IF_MCTL) & MCTL_MSGLST as u16,
            MCTL_MSGLST as u16
        );
    }

    /// `MNUM` 0 addresses object 32.
    #[test]
    fn mnum_zero_is_object_32() {
        let mut can = can0();
        start(&mut can);
        setup_tx(&mut can, 0, 0x300, 0, &[]);
        assert_eq!(can.drain_output().len(), 1);
        assert_eq!(read_word(&mut can, MSG2VAL_OFF) & (1 << 15), 1 << 15);
    }
}
