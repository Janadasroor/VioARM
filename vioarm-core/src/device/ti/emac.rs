//! TI Tiva C Ethernet MAC (`EMAC0`, `0x400E_C000`).
//!
//! Register map and MII/descriptor semantics follow the TI `hw_emac.h` +
//! `emac.c`/`lwiplib` oracle (Synopsys DWC core, BSD): `MIIADDR` `MIIB`/
//! `MIIW`/`CR`/`MII`/`PLA`, `MIIDATA`, `CFG` `RE`/`TE`, enhanced
//! descriptors (`TDES0`: `OWN` 31/`IC` 30/`LS` 29/`FS` 28/`TCH` 20/
//! `TER` 21; `TDES1`: `TBS1` 12:0/`TBS2` 28:16; `TDES2/3` buffer/next;
//! `RDES0`: `OWN`/`ES` 15/`FL` 29:16/`FS` 9/`LS` 8; `RDES1`: `RBS1`
//! 12:0/`RCH` 14/`RER` 15/`DIC` 31), `DMAOPMODE` `ST`/`SR`,
//! `DMARIS` `TI`/`RI`/`NIS`. The SVD carries no `<resetValue>`, so all
//! words reset 0.
//!
//! Modeled: instant `MIIB` completion (no PHY — `MIIDATA` reads
//! all-ones), config/address latches, descriptor-ring DMA through
//! the shared [`EthMailbox`] (same bus coupling as the H7 model):
//! TX reads descriptors + buffers on `TXPOLLD`/store-forward, clears
//! `OWN`, latches `TI`; RX stores injected frames into owned
//! descriptors with `FS`/`LS`/`FL` (length as delivered — CRC bytes are
//! not appended), latches `RI`; `DMARIS` W1C, `NIS` synthesis.
//! Checksum offload (SPMS433B §20.3.8, DWC Type 2): TX `TDES0.CIC`
//! insertion (IPv4 header + TCP/UDP/ICMP/ICMPv6, full or segment-only)
//! with `IHE`/`ES` write-back; RX validation under `EMACCFG.IPC`
//! driving Table 20-9 `RDES0` bits. MMC counters (§20.3.9, all eight
//! 32-bit) with `MMCCTRL` modes, half/full interrupts + masks.
//! Deliberate scope: PTP time, magic-packet wake, second-buffer
//! (`TBS2`/`RBS2`) scatter, DMA burst arbitration (instant, unpaced),
//! `CRCR` CRC replacement (needs an FCS engine), `VLIC` tag edits
//! (control bits preserved on write-back), IPv6 extension headers,
//! `UCDBC`. Notes: store-forward is structural here (the frame only
//! exists whole at `LS`), so the `TSF`/size-limit note needs no
//! enforcement; `DC`/`DP` are trivially satisfied (no CRC appended,
//! no padding added); collision counters stay zero (no half-duplex
//! collisions occur); CRC/alignment counters stay zero (no wire
//! errors occur — injected frames land whole).
//!
//! Two judgment calls, both documented: (a) `CIC=2` computes over the
//! segment with the checksum field zeroed, then folds the input field
//! value in (`C = ~(S + F)`; `F = 0` reduces to segment-only). The
//! databook sentence ("pseudo-header assumed present in the Checksum
//! field") is meaningless if the engine overwrote the field, so the
//! field must participate — and with software providing the raw
//! pseudo-header partial sum the result verifies exactly. (b) RX
//! header-error-only reads `(FT=1, Err=1, Pld=0)`: the table lists it
//! reserved, but the `ES` equation ORs independent error bits, so the
//! bits are driven independently. The `ES` line mentioning
//! `RDES[4:3]` matches no checksum field in `RDES4` (timestamp/PTP
//! only) — taken as a doc carryover; `ES` ORs the modeled error bits.

use crate::device::stm32::eth::{EthMailbox, EthMemOp};
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use core::ops::Deref;
use std::sync::{Arc, Mutex};

/// `EMAC0` base address.
pub const EMAC_BASE: u32 = 0x400E_C000;

const MIIADDR_OFF: u32 = 0x10;
const MIIDATA_OFF: u32 = 0x14;
const EMACRIS_OFF: u32 = 0x38;
const EMACIM_OFF: u32 = 0x3C;
const CFG_OFF: u32 = 0x00;
const DMAOPMODE_OFF: u32 = 0xC18;
const TXPOLLD_OFF: u32 = 0xC04;
const RXPOLLD_OFF: u32 = 0xC08;
const RXDLADDR_OFF: u32 = 0xC0C;
const TXDLADDR_OFF: u32 = 0xC10;
const DMARIS_OFF: u32 = 0xC14;
/// MMC block.
const MMCCTRL_OFF: u32 = 0x100;
const MMCRXRIS_OFF: u32 = 0x104;
const MMCTXRIS_OFF: u32 = 0x108;
const MMCRXIM_OFF: u32 = 0x10C;
const MMCTXIM_OFF: u32 = 0x110;
/// MMC counter offsets, `mmc[]` order: TXGB, TXSCOL, TXMCOL, TXOCT,
/// RXGB, RXCRC, RXALGN, RXGUNI (all 32-bit per the register pages).
const MMC_OFFS: [u32; 8] = [0x118, 0x14C, 0x150, 0x164, 0x180, 0x194, 0x198, 0x1C4];
/// `EMACCFG.IPC` (receive checksum offload enable).
const CFG_IPC: u32 = 1 << 10;
/// `EMACRIS` MMC summary bits.
const RIS_MMC: u32 = 1 << 4;
const RIS_MMCRX: u32 = 1 << 5;
const RIS_MMCTX: u32 = 1 << 6;
/// `MMCCTRL` mode bits.
const MMC_CNTRST: u32 = 1 << 0;
const MMC_CNTSTPRO: u32 = 1 << 1;
const MMC_RSTONRD: u32 = 1 << 2;
const MMC_CNTFREEZ: u32 = 1 << 3;
const MMC_CNTPRST: u32 = 1 << 4;
const MMC_CNTPRSTLVL: u32 = 1 << 5;
/// RX/TX RIS bits per counter (`mmc[]` order); `0` marks the two
/// TX-only collision slots on the RX side and vice versa.
const MMC_RXRIS_BIT: [u32; 8] = [0, 0, 0, 0, 1 << 0, 1 << 5, 1 << 6, 1 << 17];
const MMC_TXRIS_BIT: [u32; 8] = [1 << 1, 1 << 14, 1 << 15, 1 << 20, 0, 0, 0, 0];
/// Half-maximum interrupt threshold (all counters 32-bit).
const MMC_HALF: u32 = 0x8000_0000;

/// `MIIADDR.MIIB` (busy) — self-clearing on instant completion.
const MII_MIIB: u32 = 1 << 0;
/// `DMAOPMODE` engine enables.
const OP_ST: u32 = 1 << 13;
const OP_SR: u32 = 1 << 1;
/// Descriptor bits.
const OWN: u32 = 1 << 31;
const TDES0_FS: u32 = 1 << 28;
const TDES0_LS: u32 = 1 << 29;
const TDES0_TCH: u32 = 1 << 20;
const TDES0_TER: u32 = 1 << 21;
const TBS1_MASK: u32 = 0x1FFF;
const RDES1_RCH: u32 = 1 << 14;
const RDES1_RER: u32 = 1 << 15;
const RBS1_MASK: u32 = 0x1FFF;
const RDES0_FS: u32 = 1 << 9;
const RDES0_LS: u32 = 1 << 8;
/// `DMARIS` bits.
const DMA_TI: u32 = 1 << 0;
const DMA_RI: u32 = 1 << 6;
const DMA_NIS: u32 = 1 << 16;
/// `TDES0` checksum/status bits (Table 20-2).
const TDES0_CIC_SHIFT: u32 = 22;
const TDES0_ES: u32 = 1 << 15;
const TDES0_IHE: u32 = 1 << 16;
/// `RDES0` checksum/status bits (Tables 20-8/20-9).
const RDES0_ES: u32 = 1 << 15;
const RDES0_FT: u32 = 1 << 5;
const RDES0_IPCERR: u32 = 1 << 7;
const RDES0_PCSERR: u32 = 1 << 0;

/// DMA phase: one mailbox round-trip per tick.
#[derive(Debug, Default, PartialEq, Eq)]
enum Phase {
    #[default]
    Idle,
    AwaitTxDesc,
    AwaitTxData {
        desc: u32,
        t0: u32,
        next: u32,
    },
    AwaitRxDesc,
}

/// One's-complement sum accumulator over big-endian words (RFC 1071).
fn csum_add(mut acc: u32, bytes: &[u8]) -> u32 {
    let mut it = bytes.chunks_exact(2);
    for w in &mut it {
        acc += u32::from(u16::from_be_bytes([w[0], w[1]]));
    }
    if let Some(&last) = it.remainder().first() {
        acc += u32::from(last) << 8;
    }
    acc
}

/// Fold carries and complement: the checksum to store.
fn csum_fold(sum: u32) -> u16 {
    let mut acc = sum;
    while acc >> 16 != 0 {
        acc = (acc & 0xFFFF) + (acc >> 16);
    }
    !(acc as u16)
}

/// Whether `bytes` (checksum field included) sum to a valid total.
fn csum_valid(bytes: &[u8]) -> bool {
    csum_fold(csum_add(0, bytes)) == 0
}

/// EtherType + IP offset of a frame, skipping one VLAN tag (§20.3.8.3
/// applies identification to single-tagged frames too).
fn eth_ip_split(frame: &[u8]) -> Option<(u16, usize)> {
    if frame.len() < 14 {
        return None;
    }
    let mut ether = u16::from_be_bytes([frame[12], frame[13]]);
    let mut off = 14;
    if ether == 0x8100 {
        if frame.len() < 18 {
            return None;
        }
        ether = u16::from_be_bytes([frame[16], frame[17]]);
        off = 18;
    }
    Some((ether, off))
}

/// Copy with the 2-byte checksum at `ck_off` zeroed.
fn seg_zeroed(seg: &[u8], ck_off: usize) -> Vec<u8> {
    let mut s = seg.to_vec();
    if s.len() >= ck_off + 2 {
        s[ck_off] = 0;
        s[ck_off + 1] = 0;
    }
    s
}

/// Transport checksum location + pseudo-header need for `(version,
/// proto)`: TCP/UDP/ICMPv4/ICMPv6 only; `None` is bypassed.
fn l4_slot(version: u8, proto: u8) -> Option<(usize, bool)> {
    match (version, proto) {
        (4, 6) | (6, 6) => Some((16, true)),  // TCP
        (4, 17) | (6, 17) => Some((6, true)), // UDP
        (4, 1) => Some((2, false)),           // ICMPv4: no pseudo-header
        (6, 58) => Some((2, true)),           // ICMPv6: pseudo-header
        _ => None,
    }
}

/// IPv6 pseudo-header sum (RFC 2460 §8.1): src + dst + length + zeros
/// + next header. No extension headers (documented).
fn pseudo6_sum(seg: &[u8], ip: &[u8], proto: u8) -> Option<u32> {
    if ip.len() < 40 {
        return None;
    }
    let mut sum = csum_add(0, &ip[8..24]);
    sum += u32::from(u16::from_be_bytes([ip[24], ip[25]]));
    sum += u32::from(ip[26]);
    sum += u32::from(proto);
    sum += seg.len() as u32;
    Some(sum)
}

/// Insert checksums per `TDES0.CIC` (§20.3.8.1/2). Returns `TDES0`
/// status bits (`IHE`/`ES`). IPv4 header checksum is inserted whenever
/// the EtherType says IPv4, even with a header error; L4 follows `CIC`
/// (`2` folds the input field in, `3` uses the pseudo-header).
fn tx_insert_checksum(frame: &mut [u8], cic: u32) -> u32 {
    if cic == 0 {
        return 0;
    }
    // Compute every checksum before touching the frame: the header and
    // segment slices borrow it immutably.
    enum Work {
        Nothing(u32),
        Insert {
            ip_sum: Option<(usize, u16)>,
            l4_sum: Option<(usize, u16)>,
            ihe: bool,
        },
    }
    let work = (|| {
        let (ether, ip_off) = eth_ip_split(frame)?;
        let ip = frame.get(ip_off..)?;
        let version = ip.first().copied().unwrap_or(0) >> 4;
        let (is_v4, is_v6) = (ether == 0x0800, ether == 0x86DD);
        if !is_v4 && !is_v6 {
            return Some(Work::Nothing(0)); // Engine bypasses non-IP.
        }
        let mut ihe = false;
        let (seg_off, proto) = if is_v4 {
            if version != 4 {
                ihe = true;
                (usize::MAX, 0)
            } else {
                let ihl = usize::from(ip.first().copied().unwrap_or(0) & 0x0F) * 4;
                if ihl < 20 || ip.len() < ihl {
                    ihe = true;
                }
                (ip_off + ihl, ip.get(9).copied().unwrap_or(0))
            }
        } else if version != 6 || ip.len() < 40 {
            ihe = true;
            (usize::MAX, 0)
        } else {
            (ip_off + 40, ip[6])
        };
        // IPv4 header checksum inserts even with the error flagged.
        let mut ip_sum = None;
        if is_v4 {
            let hdr_len =
                (usize::from(ip.first().copied().unwrap_or(0) & 0x0F) * 4).clamp(12, ip.len());
            let mut hdr = ip[..hdr_len].to_vec();
            if hdr_len >= 12 {
                hdr[10] = 0;
                hdr[11] = 0;
            }
            ip_sum = Some((ip_off + 10, csum_fold(csum_add(0, &hdr))));
        }
        if cic < 2 || seg_off == usize::MAX {
            return Some(Work::Insert {
                ip_sum,
                l4_sum: None,
                ihe,
            });
        }
        let seg = match frame.get(seg_off..) {
            Some(s) => s,
            // Truncated past the header: the `ihl` check already set
            // `ihe`; the header checksum (if any) still lands.
            None => {
                return Some(Work::Insert {
                    ip_sum,
                    l4_sum: None,
                    ihe,
                });
            }
        };
        let (ck_off, with_pseudo) = l4_slot(if is_v4 { 4 } else { 6 }, proto)?;
        if seg.len() < ck_off + 2 {
            ihe = true;
            return Some(Work::Insert {
                ip_sum,
                l4_sum: None,
                ihe,
            });
        }
        let stored = u16::from_be_bytes([seg[ck_off], seg[ck_off + 1]]);
        let mut sum = csum_add(0, &seg_zeroed(seg, ck_off));
        if cic == 3 {
            if with_pseudo {
                if is_v4 {
                    let hdr = frame.get(ip_off..seg_off)?;
                    if hdr.len() >= 20 {
                        sum = csum_add(sum, &hdr[12..20]);
                        sum += u32::from(proto);
                        sum += seg.len() as u32;
                    }
                } else if let Some(p) = pseudo6_sum(seg, ip, proto) {
                    sum += p;
                }
            }
        } else {
            // CIC=2: the pseudo-header partial sum rides in the field.
            sum += u32::from(stored);
        }
        Some(Work::Insert {
            ip_sum,
            l4_sum: Some((seg_off + ck_off, csum_fold(sum))),
            ihe,
        })
    })();
    match work {
        None | Some(Work::Nothing(_)) => 0,
        Some(Work::Insert {
            ip_sum,
            l4_sum,
            ihe,
        }) => {
            if let Some((off, sum)) = ip_sum {
                if frame.len() >= off + 2 {
                    frame[off..off + 2].copy_from_slice(&sum.to_be_bytes());
                }
            }
            if let Some((off, sum)) = l4_sum {
                if frame.len() >= off + 2 {
                    frame[off..off + 2].copy_from_slice(&sum.to_be_bytes());
                }
            }
            if ihe {
                TDES0_IHE | TDES0_ES
            } else {
                0
            }
        }
    }
}

/// RX checksum verdict (§20.3.8.3 + Table 20-9): `(FT, header-error,
/// payload-error)` for `RDES0` bits 5/7/0. `None` when the frame is a
/// runt or carries no verdict at all.
fn rx_checksum_status(frame: &[u8]) -> Option<(bool, bool, bool)> {
    let (ether, ip_off) = eth_ip_split(frame)?;
    if ether < 1536 {
        return Some((false, false, false)); // IEEE 802.3 length frame.
    }
    let (is_v4, is_v6) = (ether == 0x0800, ether == 0x86DD);
    if !is_v4 && !is_v6 {
        return Some((true, false, false)); // Ethertype, engine bypasses.
    }
    let ip = frame.get(ip_off..)?;
    let version = ip.first().copied().unwrap_or(0) >> 4;
    // Version mismatch or truncated version byte: the engine treats the
    // frame as neither IPv4 nor IPv6 (Table 20-9 row 7).
    if (is_v4 && version != 4) || (is_v6 && version != 6) || ip.is_empty() {
        return Some((false, true, true));
    }
    let (seg_off, proto, header_ok) = if is_v4 {
        let ihl = usize::from(ip[0] & 0x0F) * 4;
        if ihl < 20 || ip.len() < ihl {
            return Some((false, true, true));
        }
        let total = usize::from(u16::from_be_bytes([ip[2], ip[3]]));
        let header_ok = csum_valid(&ip[..ihl]) && total >= ihl;
        (ip_off + ihl, ip[9], header_ok)
    } else {
        if ip.len() < 40 {
            return Some((false, true, true));
        }
        (ip_off + 40, ip[6], true)
    };
    let seg_avail = frame.len().saturating_sub(seg_off);
    let (ck_off, with_pseudo) = match l4_slot(if is_v4 { 4 } else { 6 }, proto) {
        Some(v) => v,
        // Unsupported payload with a good header: bypassed (row 6);
        // with a bad header the engine bypasses outright (row 7).
        None => {
            return Some(if header_ok {
                (false, false, true)
            } else {
                (false, true, true)
            })
        }
    };
    if seg_avail < ck_off + 2 {
        return Some((true, !header_ok, true));
    }
    // Payload length from the IP header must cover the bytes present.
    let expect = if is_v4 {
        usize::from(u16::from_be_bytes([ip[2], ip[3]])).saturating_sub(seg_off - ip_off)
    } else {
        usize::from(u16::from_be_bytes([ip[4], ip[5]]))
    };
    if seg_avail < expect {
        return Some((true, !header_ok, true));
    }
    let seg = &frame[seg_off..seg_off + expect.max(ck_off + 2).min(seg_avail)];
    let stored = u16::from_be_bytes([seg[ck_off], seg[ck_off + 1]]);
    // UDP may legally send zero ("not computed"): accept either way.
    if proto == 17 && stored == 0 {
        return Some((true, !header_ok, false));
    }
    let mut sum = csum_add(0, &seg_zeroed(seg, ck_off));
    if with_pseudo {
        if is_v4 {
            sum = csum_add(sum, &frame[ip_off + 12..ip_off + 20]);
            sum += u32::from(proto);
            sum += seg.len() as u32;
        } else if let Some(p) = pseudo6_sum(seg, ip, proto) {
            sum += p;
        }
    }
    let payload_ok = csum_fold(sum) == stored;
    Some((true, !header_ok, !payload_ok))
}

/// Behavior behind the MAC window: shell registers plus the
/// descriptor-ring DMA engine.
#[derive(Debug)]
pub struct EmacBehavior {
    mailbox: Arc<Mutex<EthMailbox>>,
    tx_desc: u32,
    rx_desc: u32,
    tx_init: bool,
    rx_init: bool,
    tx_poll: bool,
    tx_accum: Vec<u8>,
    /// `CIC` latched from the frame's first descriptor.
    tx_cic: u32,
    /// Frames firmware transmitted (test/closeout introspection).
    tx_frames: Vec<Vec<u8>>,
    /// Host-injected receive frame.
    rx_pending: Option<Vec<u8>>,
    dmaris: u32,
    phase: Phase,
    /// MMC counters (`MMC_OFFS` order), behavior-side (SVD reset 0).
    mmc: [u32; 8],
    rx_ris: u32,
    tx_ris: u32,
}

impl EmacBehavior {
    /// Queue a descriptor/buffer read; bytes land in `results`.
    fn push_read(&self, addr: u32, len: u16) {
        if let Ok(mut mb) = self.mailbox.lock() {
            mb.ops.push(EthMemOp::Read { addr, len });
        }
    }

    /// Queue a memory write.
    fn push_write(&self, addr: u32, bytes: Vec<u8>) {
        if let Ok(mut mb) = self.mailbox.lock() {
            mb.ops.push(EthMemOp::Write { addr, bytes });
        }
    }

    /// Consume one landed mailbox result, if any.
    fn pop_result(&self) -> Option<Vec<u8>> {
        self.mailbox
            .lock()
            .ok()
            .and_then(|mut mb| mb.results.pop_front())
    }

    fn tx_enabled(view: &Registers<&mut [u32]>) -> bool {
        view.get(DMAOPMODE_OFF) & OP_ST != 0
    }

    fn rx_enabled(view: &Registers<&mut [u32]>) -> bool {
        view.get(DMAOPMODE_OFF) & OP_SR != 0
    }

    fn mmc_ctrl<W>(view: &Registers<W>) -> u32
    where
        W: Deref<Target = [u32]>,
    {
        view.get(MMCCTRL_OFF)
    }

    /// Bump counter `idx` by `n` (octet counter takes the length):
    /// frozen under `CNTFREEZ`, saturating under `CNTSTPRO`, and the
    /// half/full interrupt bit latches once reached.
    fn mmc_bump(&mut self, ctrl: u32, idx: usize, n: u32) {
        if ctrl & MMC_CNTFREEZ != 0 {
            return;
        }
        let (v, overflow) = self.mmc[idx].overflowing_add(n);
        self.mmc[idx] = if overflow {
            if ctrl & MMC_CNTSTPRO != 0 {
                u32::MAX
            } else {
                v // Wrapped: the crossing still interrupts.
            }
        } else {
            v
        };
        if self.mmc[idx] >= MMC_HALF || overflow {
            if idx < 4 {
                self.tx_ris |= MMC_TXRIS_BIT[idx];
            } else {
                self.rx_ris |= MMC_RXRIS_BIT[idx];
            }
        }
    }

    /// Unmasked MMC summaries for `EMACRIS`: `(rx, tx)`.
    fn mmc_summary<W>(&self, view: &Registers<W>) -> (bool, bool)
    where
        W: Deref<Target = [u32]>,
    {
        let rx = self.rx_ris & !view.get(MMCRXIM_OFF) != 0;
        let tx = self.tx_ris & !view.get(MMCTXIM_OFF) != 0;
        (rx, tx)
    }

    /// `mmc[]` index + owning RIS side for a counter offset.
    fn mmc_slot(offset: u32) -> Option<(usize, bool)> {
        MMC_OFFS
            .iter()
            .position(|&o| o == offset)
            .map(|idx| (idx, idx >= 4))
    }

    /// A counter's LSB lane was read: `RSTONRD` zeroes it, and the
    /// mapped interrupt bit clears (it fires only from its counter).
    fn counter_lane0_read(&mut self, view: &Registers<&mut [u32]>, offset: u32) {
        if let Some((idx, is_rx)) = Self::mmc_slot(offset) {
            if view.get(MMCCTRL_OFF) & MMC_RSTONRD != 0 {
                self.mmc[idx] = 0;
            }
            let bit = if is_rx {
                MMC_RXRIS_BIT[idx]
            } else {
                MMC_TXRIS_BIT[idx]
            };
            if is_rx {
                self.rx_ris &= !bit;
            } else {
                self.tx_ris &= !bit;
            }
        }
    }

    /// Consume a fetched TX descriptor: read its buffer next.
    fn consume_tx_desc(&mut self, view: &mut Registers<&mut [u32]>, bytes: &[u8]) {
        let w = |i: usize| {
            u32::from_le_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ])
        };
        let (t0, t1, buf, t3) = (w(0), w(1), w(2), w(3));
        if t0 & OWN == 0 {
            // Nothing owned by the DMA; the poll was spurious.
            self.phase = Phase::Idle;
            return;
        }
        let tbs1 = (t1 & TBS1_MASK) as u16;
        let desc = self.tx_desc;
        if t0 & TDES0_FS != 0 {
            // `CIC` is valid on the first segment; it rides the frame.
            self.tx_cic = (t0 >> TDES0_CIC_SHIFT) & 3;
        }
        let mut next = if t0 & TDES0_TCH != 0 {
            t3
        } else {
            desc.wrapping_add(16)
        };
        if t0 & TDES0_TER != 0 {
            next = view.get(TXDLADDR_OFF);
        }
        self.push_read(buf, tbs1);
        self.phase = Phase::AwaitTxData { desc, t0, next };
    }

    /// Complete a TX buffer read: frame bytes out, ownership home.
    fn complete_tx(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        data: &[u8],
        desc: u32,
        t0: u32,
        next: u32,
    ) {
        let ctrl = Self::mmc_ctrl(view);
        let mut status = 0;
        if t0 & TDES0_LS != 0 {
            self.tx_accum.extend_from_slice(data);
            let mut frame = core::mem::take(&mut self.tx_accum);
            if self.tx_cic != 0 {
                status = tx_insert_checksum(&mut frame, self.tx_cic);
            }
            self.mmc_bump(ctrl, 0, 1);
            self.mmc_bump(ctrl, 3, frame.len() as u32);
            self.tx_frames.push(frame);
            self.dmaris |= DMA_TI;
        } else {
            self.tx_accum.extend_from_slice(data);
        }
        self.push_write(desc, ((t0 & !OWN) | status).to_le_bytes().to_vec());
        self.tx_desc = next;
        self.phase = Phase::Idle;
    }

    /// Advance the RX ring: store the pending frame, write status back.
    fn rx_advance(&mut self, view: &mut Registers<&mut [u32]>, bytes: &[u8]) {
        let w = |i: usize| {
            u32::from_le_bytes([
                bytes[4 * i],
                bytes[4 * i + 1],
                bytes[4 * i + 2],
                bytes[4 * i + 3],
            ])
        };
        let (r0, r1, buf, r3) = (w(0), w(1), w(2), w(3));
        if r0 & OWN == 0 {
            // Not owned: drop rather than overwrite firmware's data
            // (the ISR resumes the ring with a poll).
            self.rx_pending = None;
            self.phase = Phase::Idle;
            return;
        }
        if let Some(frame) = self.rx_pending.take() {
            let rbs1 = (r1 & RBS1_MASK) as usize;
            let n = frame.len().min(rbs1);
            if n > 0 {
                self.push_write(buf, frame[..n].to_vec());
            }
            // Status: ownership home, first+last, length as delivered
            // (CRC bytes are not appended by this model), plus Table
            // 20-9 checksum bits when `IPC` is set.
            let fl = ((frame.len() as u32) & 0x3FFF) << 16;
            let mut status = fl | RDES0_FS | RDES0_LS;
            if view.get(CFG_OFF) & CFG_IPC != 0 {
                if let Some((ft, herr, perr)) = rx_checksum_status(&frame) {
                    if ft {
                        status |= RDES0_FT;
                    }
                    if herr {
                        status |= RDES0_IPCERR | RDES0_ES;
                    }
                    if perr {
                        status |= RDES0_PCSERR | RDES0_ES;
                    }
                }
            }
            self.push_write(self.rx_desc, status.to_le_bytes().to_vec());
            let ctrl = Self::mmc_ctrl(view);
            self.mmc_bump(ctrl, 4, 1);
            if frame.len() >= 6 && frame[0] & 1 == 0 {
                self.mmc_bump(ctrl, 7, 1); // Unicast DA.
            }
            let mut next = if r1 & RDES1_RCH != 0 {
                r3
            } else {
                self.rx_desc.wrapping_add(16)
            };
            if r1 & RDES1_RER != 0 {
                next = view.get(RXDLADDR_OFF);
            }
            self.rx_desc = next;
            self.dmaris |= DMA_RI;
        }
        self.phase = Phase::Idle;
    }
}

impl Behavior for EmacBehavior {
    fn needs_tick() -> bool {
        true
    }

    fn reset(&mut self) {
        self.tx_desc = 0;
        self.rx_desc = 0;
        self.tx_init = false;
        self.rx_init = false;
        self.tx_poll = false;
        self.tx_accum.clear();
        self.tx_cic = 0;
        self.rx_pending = None;
        self.dmaris = 0;
        self.phase = Phase::Idle;
        self.mmc = [0; 8];
        self.rx_ris = 0;
        self.tx_ris = 0;
        if let Ok(mut mb) = self.mailbox.lock() {
            mb.ops.clear();
            mb.results.clear();
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        let (rx, tx) = self.mmc_summary(view);
        u32::from(self.dmaris & (DMA_TI | DMA_RI) != 0 || rx || tx)
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        // Consume one landed result per tick (bus drains after ticks).
        if self.phase != Phase::Idle {
            if let Some(result) = self.pop_result() {
                match core::mem::replace(&mut self.phase, Phase::Idle) {
                    Phase::AwaitRxDesc => self.rx_advance(view, &result),
                    Phase::AwaitTxDesc => self.consume_tx_desc(view, &result),
                    Phase::AwaitTxData { desc, t0, next } => {
                        self.complete_tx(view, &result, desc, t0, next)
                    }
                    Phase::Idle => {}
                }
            }
            return;
        }
        // Nothing in flight: start new work.
        if Self::rx_enabled(view) && self.rx_pending.is_some() {
            if !self.rx_init {
                self.rx_desc = view.get(RXDLADDR_OFF);
                self.rx_init = true;
            }
            self.push_read(self.rx_desc, 16);
            self.phase = Phase::AwaitRxDesc;
            return;
        }
        if Self::tx_enabled(view) && self.tx_poll {
            self.tx_poll = false;
            if !self.tx_init {
                self.tx_desc = view.get(TXDLADDR_OFF);
                self.tx_init = true;
            }
            self.push_read(self.tx_desc, 16);
            self.phase = Phase::AwaitTxDesc;
        }
    }

    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        match offset {
            MIIADDR_OFF => value & !MII_MIIB,
            MIIDATA_OFF => 0xFFFF, // No PHY: MDIO floats high.
            0x24 => 0,             // Engines idle between bursts.
            EMACRIS_OFF => {
                // MAC `RIS`: MMC summaries only (PMT/TS latch by scope).
                let (rx, tx) = self.mmc_summary(view);
                let mut r = 0;
                if rx || tx {
                    r |= RIS_MMC;
                }
                if rx {
                    r |= RIS_MMCRX;
                }
                if tx {
                    r |= RIS_MMCTX;
                }
                r
            }
            MMCRXRIS_OFF => self.rx_ris,
            MMCTXRIS_OFF => self.tx_ris,
            DMARIS_OFF => {
                let mut d = self.dmaris;
                if d & (DMA_TI | DMA_RI) != 0 {
                    d |= DMA_NIS;
                }
                d
            }
            _ => {
                if let Some((idx, _)) = Self::mmc_slot(offset) {
                    return self.mmc[idx];
                }
                value
            }
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        if offset == EMACRIS_OFF {
            return view.get(offset);
        }
        if offset == MMCRXRIS_OFF || offset == MMCTXRIS_OFF {
            // Read-only status (SVD marks RW): writes never land.
            return view.get(offset);
        }
        if offset == DMARIS_OFF {
            // W1C status (only whole-word stores clear, like `FCMISC`).
            if shift == 24 {
                let word = view.merged(offset, val, shift);
                self.dmaris &= !word;
            }
            return self.dmaris;
        }
        let word = view.merged(offset, val, shift);
        view.set(offset, word);
        if offset == MMCCTRL_OFF && shift == 24 {
            // Whole-word stores act; self-clearing bits clear in the
            // stored word (`CNTRST` beats `CNTPRST`).
            let mut ctrl = view.get(MMCCTRL_OFF);
            if ctrl & MMC_CNTRST != 0 {
                self.mmc = [0; 8];
                self.rx_ris = 0;
                self.tx_ris = 0;
                ctrl &= !MMC_CNTRST;
            } else if ctrl & MMC_CNTPRST != 0 {
                // Preset literals from the `CNTPRSTLVL` description
                // (octet counter 2 KiB below the frame preset).
                let half = ctrl & MMC_CNTPRSTLVL == 0;
                let (frame, octet) = if half {
                    (0x7FFF_FFF0, 0x7FFF_F800)
                } else {
                    (0xFFFF_FFF0, 0xFFFF_F800)
                };
                for (idx, slot) in self.mmc.iter_mut().enumerate() {
                    *slot = if idx == 3 { octet } else { frame };
                }
                ctrl &= !MMC_CNTPRST;
            }
            view.set(MMCCTRL_OFF, ctrl);
            return ctrl;
        }
        if offset == TXPOLLD_OFF && shift == 24 {
            self.tx_poll = true; // Resume-poll: any word store wakes TX.
        }
        word
    }
}

/// A Tiva C Ethernet MAC (shell + descriptor-ring DMA).
pub struct Emac {
    inner: RegisterFile<EmacBehavior>,
}

impl Emac {
    /// A new MAC on `D`'s tables (`"EMAC0"`), sharing the Ethernet
    /// mailbox with the memory bus for descriptor/frame DMA.
    ///
    /// # Panics
    /// Panics if `D`'s tables lack `"EMAC0"`.
    #[must_use]
    pub fn for_device<D: Device>(mailbox: Arc<Mutex<EthMailbox>>) -> Self {
        let def = D::peripheral("EMAC0").expect("generated tables contain EMAC0");
        Self {
            inner: RegisterFile::new(
                def,
                EmacBehavior {
                    mailbox,
                    tx_desc: 0,
                    rx_desc: 0,
                    tx_init: false,
                    rx_init: false,
                    tx_poll: false,
                    tx_accum: Vec::new(),
                    tx_cic: 0,
                    tx_frames: Vec::new(),
                    rx_pending: None,
                    dmaris: 0,
                    phase: Phase::Idle,
                    mmc: [0; 8],
                    rx_ris: 0,
                    tx_ris: 0,
                },
            ),
        }
    }

    /// Queue a received frame (host stimulus).
    pub fn inject_frame(&mut self, bytes: &[u8]) {
        self.inner.with_view(|_, b| {
            b.rx_pending = Some(bytes.to_vec());
        });
    }

    /// Frames firmware transmitted since the last drain.
    #[must_use]
    pub fn drain_frames(&mut self) -> Vec<Vec<u8>> {
        self.inner
            .with_view(|_, b| core::mem::take(&mut b.tx_frames))
    }
}

impl Peripheral for Emac {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
        let byte = self.inner.read(addr)?;
        // `RSTONRD` + interrupt-clear fire on the counter's LSB lane,
        // which `on_read` cannot see (per-byte calls, no lane).
        if addr & 3 == 0 {
            let off = addr.wrapping_sub(EMAC_BASE);
            self.inner
                .with_view(|view, b| b.counter_lane0_read(view, off));
        }
        Ok(byte)
    }

    fn write(&mut self, addr: u32, val: u8) -> Result<(), BusError> {
        self.inner.write(addr, val)
    }

    fn reset(&mut self) {
        self.inner.reset();
        // Clear the shared mailbox in place (the board holds the twin).
        self.inner.with_view(|_, b| {
            if let Ok(mut mb) = b.mailbox.lock() {
                mb.ops.clear();
                mb.results.clear();
            }
        });
    }

    fn tick(&mut self, delta: u64) {
        self.inner.tick(delta);
    }
}

impl InterruptSource for Emac {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::svd::tm4c129xnczad::Tm4c129xnczad;

    const EMAC0_BASE: u32 = 0x400E_C000;
    const CFG_OFF: u32 = 0x00;
    const ADDR0L_OFF: u32 = 0x44;

    fn emac() -> (Emac, Arc<Mutex<EthMailbox>>) {
        let mb = Arc::new(Mutex::new(EthMailbox::default()));
        (Emac::for_device::<Tm4c129xnczad>(mb.clone()), mb)
    }

    fn read_word(emac: &mut Emac, offset: u32) -> u32 {
        let mut v = 0u32;
        for lane in 0..4 {
            v |= u32::from(emac.read(EMAC0_BASE + offset + lane).unwrap()) << (8 * lane);
        }
        v
    }

    fn write_word(emac: &mut Emac, offset: u32, value: u32) {
        for lane in 0..4 {
            emac.write(
                EMAC0_BASE + offset + lane,
                ((value >> (8 * lane)) & 0xFF) as u8,
            )
            .unwrap();
        }
    }

    /// Seed one landed mailbox result (simulates the bus drain).
    fn seed_result(mb: &Arc<Mutex<EthMailbox>>, bytes: Vec<u8>) {
        mb.lock().unwrap().results.push_back(bytes);
    }

    fn word_le(w: u32) -> Vec<u8> {
        w.to_le_bytes().to_vec()
    }

    /// Hex string (spaces ignored) to bytes.
    fn unhex(s: &str) -> Vec<u8> {
        let h: Vec<u8> = s.bytes().filter(|b| *b != b' ').collect();
        h.chunks(2)
            .map(|c| u8::from_str_radix(core::str::from_utf8(c).unwrap(), 16).unwrap())
            .collect()
    }

    /// Ethernet frame around an IP payload (unicast DAs).
    fn eth_frame(payload: &[u8]) -> Vec<u8> {
        let mut f = vec![
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xA1, 0x08, 0x00,
        ];
        f.extend_from_slice(payload);
        f
    }

    /// Run one single-descriptor TX frame with `t0` control bits;
    /// returns the transmitted frame.
    fn tx_frame(emac: &mut Emac, mb: &Arc<Mutex<EthMailbox>>, t0: u32, frame: &[u8]) -> Vec<u8> {
        write_word(emac, DMAOPMODE_OFF, OP_ST);
        write_word(emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(emac, TXPOLLD_OFF, 0x01);
        emac.tick(1);
        let desc = [t0, frame.len() as u32, 0x2000_0100, 0x2000_0010]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect::<Vec<_>>();
        seed_result(mb, desc);
        emac.tick(1);
        seed_result(mb, frame.to_vec());
        emac.tick(1);
        let frames = emac.drain_frames();
        assert_eq!(frames.len(), 1);
        frames.into_iter().next().unwrap()
    }

    /// Status word the DMA wrote back to the descriptor.
    fn tx_status_wb(mb: &Arc<Mutex<EthMailbox>>) -> u32 {
        let ops = &mb.lock().unwrap().ops;
        for op in ops.iter().rev() {
            if let EthMemOp::Write {
                addr: 0x2000_0000,
                bytes,
            } = op
            {
                return u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
            }
        }
        panic!("no descriptor write-back");
    }

    /// Run one RX frame with `CFG` word `cfg`; returns the status word.
    fn rx_status(emac: &mut Emac, mb: &Arc<Mutex<EthMailbox>>, cfg: u32, frame: &[u8]) -> u32 {
        write_word(emac, DMAOPMODE_OFF, OP_SR);
        write_word(emac, CFG_OFF, cfg);
        write_word(emac, RXDLADDR_OFF, 0x2000_1000);
        emac.inject_frame(frame);
        emac.tick(1);
        let desc = [OWN, 1536, 0x2000_0200, 0x2000_1010]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect::<Vec<_>>();
        seed_result(mb, desc);
        emac.tick(1);
        let ops = &mb.lock().unwrap().ops;
        for op in ops.iter().rev() {
            if let EthMemOp::Write {
                addr: 0x2000_1000,
                bytes,
            } = op
            {
                return u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
            }
        }
        panic!("no RX status write-back");
    }

    /// `CFG` + MAC address latches; engines report idle and quiet.
    #[test]
    fn config_latches_and_idle() {
        let (mut emac, _) = emac();
        write_word(&mut emac, CFG_OFF, (1 << 2) | (1 << 3)); // RE + TE.
        assert_eq!(read_word(&mut emac, CFG_OFF) & 0x0C, 0x0C);
        write_word(&mut emac, ADDR0L_OFF, 0x1234_5678);
        assert_eq!(read_word(&mut emac, ADDR0L_OFF), 0x1234_5678);
        assert_eq!(read_word(&mut emac, 0x24), 0);
        assert_eq!(read_word(&mut emac, 0x38), 0);
        assert_eq!(emac.pending(), 0);
    }

    /// MII completes instantly; no PHY reads all-ones.
    #[test]
    fn mii_completes() {
        let (mut emac, _) = emac();
        write_word(&mut emac, MIIADDR_OFF, 0x01); // MIIB + PHY 0/reg 0.
        assert_eq!(read_word(&mut emac, MIIADDR_OFF) & MII_MIIB, 0);
        assert_eq!(read_word(&mut emac, MIIDATA_OFF), 0xFFFF);
    }

    /// TX: descriptor → buffer → ownership home + `TI`, frame observed.
    #[test]
    fn tx_single_desc_flow() {
        let (mut emac, mb) = emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac, TXPOLLD_OFF, 0x01);
        emac.tick(1); // Queues the descriptor read.
                      // Descriptor: OWN|LS|FS, TBS1=4, buffer @0x20000100.
        let desc = [OWN | TDES0_LS | TDES0_FS, 4, 0x2000_0100, 0x2000_0010]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect::<Vec<_>>();
        seed_result(&mb, desc);
        emac.tick(1); // Consumes desc, queues the buffer read.
        seed_result(&mb, vec![0xDE, 0xAD, 0xBE, 0xEF]);
        emac.tick(1); // Completes: frame out, OWN home, TI.
        assert_eq!(emac.drain_frames(), [vec![0xDE, 0xAD, 0xBE, 0xEF]]);
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_TI, DMA_TI);
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_NIS, DMA_NIS);
        assert_eq!(emac.pending(), 1);
        // Ownership write-back queued for the bus.
        let ops = &mb.lock().unwrap().ops;
        assert!(ops.iter().any(|op| matches!(op,
            EthMemOp::Write { addr: 0x2000_0000, bytes } if bytes == &word_le(TDES0_LS | TDES0_FS))));
        // W1C clears.
        write_word(&mut emac, DMARIS_OFF, DMA_TI | DMA_NIS);
        assert_eq!(emac.pending(), 0);
    }

    /// TX with `OWN` clear: spurious poll, nothing moves.
    #[test]
    fn tx_spurious_poll() {
        let (mut emac, mb) = emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac, TXPOLLD_OFF, 0x01);
        emac.tick(1);
        seed_result(&mb, vec![0u8; 16]);
        emac.tick(1);
        assert_eq!(emac.drain_frames(), Vec::<Vec<u8>>::new());
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_TI, 0);
    }

    /// RX: injected frame lands in the owned buffer with `FS`/`LS`/`FL`.
    #[test]
    fn rx_flow() {
        let (mut emac, mb) = emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_SR);
        write_word(&mut emac, RXDLADDR_OFF, 0x2000_1000);
        emac.inject_frame(&[0x01, 0x02, 0x03]);
        emac.tick(1); // Queues the descriptor read.
                      // Descriptor: OWN, RBS1=1536, buffer @0x20000200.
        let desc = [OWN, 1536, 0x2000_0200, 0x2000_1010]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect::<Vec<_>>();
        seed_result(&mb, desc);
        emac.tick(1); // Stores the frame, writes status, RI.
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_RI, DMA_RI);
        let ops = &mb.lock().unwrap().ops;
        assert!(ops.iter().any(|op| matches!(op,
            EthMemOp::Write { addr: 0x2000_0200, bytes } if bytes == &vec![0x01, 0x02, 0x03])));
        assert!(ops.iter().any(|op| matches!(op,
            EthMemOp::Write { addr: 0x2000_1000, bytes } if bytes == &word_le((3 << 16) | RDES0_FS | RDES0_LS))));
    }

    /// RX with no owned descriptor drops the frame.
    #[test]
    fn rx_drop_unowned() {
        let (mut emac, mb) = emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_SR);
        write_word(&mut emac, RXDLADDR_OFF, 0x2000_1000);
        emac.inject_frame(&[0x01]);
        emac.tick(1);
        seed_result(&mb, vec![0u8; 16]); // OWN clear.
        emac.tick(1);
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & DMA_RI, 0);
    }

    /// TX `CIC=1` inserts the IPv4 header checksum (RFC 1071 `b1e6`
    /// vector), status clean.
    #[test]
    fn tx_cic1_inserts_ipv4_header() {
        let (mut emac, mb) = emac();
        let iph = unhex("4500 003c 1c46 4000 4006 0000 ac10 0a63 ac10 0a0c");
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | (1 << TDES0_CIC_SHIFT),
            &eth_frame(&iph),
        );
        assert_eq!(&out[14 + 10..14 + 12], &[0xb1, 0xe6]);
        assert_eq!(tx_status_wb(&mb) & (TDES0_IHE | TDES0_ES), 0);
    }

    /// TX `CIC=3` inserts IPv4 + full TCP checksum with pseudo-header
    /// (`9d36`/`eead` vectors), status clean.
    #[test]
    fn tx_cic3_tcp_full() {
        let (mut emac, mb) = emac();
        let iph = unhex("4500 0028 1c46 4000 4006 0000 c0a8 0001 c0a8 0002");
        let tcp = unhex("0050 1f90 0000 0001 0000 0000 5002 2000 0000 0000");
        let mut payload = iph;
        payload.extend_from_slice(&tcp);
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | (3 << TDES0_CIC_SHIFT),
            &eth_frame(&payload),
        );
        assert_eq!(&out[14 + 10..14 + 12], &[0x9d, 0x36]);
        assert_eq!(&out[14 + 20 + 16..14 + 20 + 18], &[0xee, 0xad]);
        assert_eq!(tx_status_wb(&mb) & (TDES0_IHE | TDES0_ES), 0);
        // The produced frame validates end to end.
        assert_eq!(rx_checksum_status(&out), Some((true, false, false)));
    }

    /// TX `CIC=2` folds the input field in: raw pseudo-header partial
    /// `6cbb` yields a verifying `0361`.
    #[test]
    fn tx_cic2_folds_input_field() {
        let (mut emac, mb) = emac();
        let iph = unhex("4500 0028 1c46 4000 4006 0000 ac10 0a63 ac10 0a0c");
        let tcp = unhex("0050 1f90 0000 0001 0000 0000 5002 2000 6caa 0000");
        let mut payload = iph;
        payload.extend_from_slice(&tcp);
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | (2 << TDES0_CIC_SHIFT),
            &eth_frame(&payload),
        );
        assert_eq!(&out[14 + 20 + 16..14 + 20 + 18], &[0x03, 0x72]);
        assert_eq!(rx_checksum_status(&out), Some((true, false, false)));
    }

    /// TX `CIC=2` with a zero field is segment-only (`701c` vector).
    #[test]
    fn tx_cic2_zero_field_is_segment_only() {
        let tcp = unhex("0050 1f90 0000 0001 0000 0000 5002 2000 0000 0000");
        let mut frame = eth_frame(&unhex("4500 0028 1c46 4000 4006 0000 ac10 0a63 ac10 0a0c"));
        frame.extend_from_slice(&tcp);
        assert_eq!(tx_insert_checksum(&mut frame, 2), 0);
        assert_eq!(&frame[14 + 20 + 16..14 + 20 + 18], &[0x70, 0x1c]);
    }

    /// Good UDP frame under `IPC`: `(FT=1, no errors)`, `ES` clear.
    #[test]
    fn rx_ipc_good_frame() {
        let (mut emac, mb) = emac();
        let mut payload = unhex("4500 0021 1c46 4000 4011 b1f6 ac10 0a63 ac10 0a0c");
        payload.extend_from_slice(&unhex("04d2 162e 000d 3472 6865 6c6c 6f"));
        let st = rx_status(&mut emac, &mb, CFG_IPC, &eth_frame(&payload));
        assert_eq!(st & 0x3FFF_0000, (47 << 16), "length as delivered");
        assert_eq!(
            st & (RDES0_FS | RDES0_LS | RDES0_FT),
            RDES0_FS | RDES0_LS | RDES0_FT
        );
        assert_eq!(st & (RDES0_ES | RDES0_IPCERR | RDES0_PCSERR), 0);
    }

    /// Corrupted payload byte: `(FT=1, payload error)` + `ES`.
    #[test]
    fn rx_ipc_bad_payload() {
        let (mut emac, mb) = emac();
        let mut payload = unhex("4500 0021 1c46 4000 4011 b1f6 ac10 0a63 ac10 0a0c");
        payload.extend_from_slice(&unhex("04d2 162e 000d 3472 6865 6c6c 6f"));
        let mut frame = eth_frame(&payload);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        let st = rx_status(&mut emac, &mb, CFG_IPC, &frame);
        assert_eq!(
            st & (RDES0_FT | RDES0_PCSERR | RDES0_ES),
            RDES0_FT | RDES0_PCSERR | RDES0_ES
        );
        assert_eq!(st & RDES0_IPCERR, 0, "header still good");
    }

    /// Corrupted header checksum: `(FT=1, header error)` + `ES`
    /// (the header-error-only encoding, table-reserved — see docs).
    #[test]
    fn rx_ipc_bad_header() {
        let (mut emac, mb) = emac();
        let mut payload = unhex("4500 0021 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c");
        payload.extend_from_slice(&unhex("04d2 162e 000d 3472 6865 6c6c 6f"));
        let st = rx_status(&mut emac, &mb, CFG_IPC, &eth_frame(&payload));
        assert_eq!(
            st & (RDES0_FT | RDES0_IPCERR | RDES0_ES),
            RDES0_FT | RDES0_IPCERR | RDES0_ES
        );
    }

    /// `IPC` clear: no checksum bits at all.
    #[test]
    fn rx_ipc_disabled_is_quiet() {
        let (mut emac, mb) = emac();
        let mut payload = unhex("4500 0021 1c46 4000 4011 b1f6 ac10 0a63 ac10 0a0c");
        payload.extend_from_slice(&unhex("04d2 162e 000d 3472 6865 6c6c 6f"));
        let st = rx_status(&mut emac, &mb, 0, &eth_frame(&payload));
        assert_eq!(st & (RDES0_FT | RDES0_IPCERR | RDES0_PCSERR | RDES0_ES), 0);
    }

    /// Non-IP EtherType: `FT` set, engine bypasses.
    #[test]
    fn rx_non_ip_frame() {
        let (mut emac, mb) = emac();
        let mut frame = vec![
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xA1, 0x88, 0xB5,
        ];
        frame.resize(60, 0);
        let st = rx_status(&mut emac, &mb, CFG_IPC, &frame);
        assert_eq!(st & RDES0_FT, RDES0_FT);
        assert_eq!(st & (RDES0_IPCERR | RDES0_PCSERR | RDES0_ES), 0);
    }

    /// MMC frame/octet counters move with traffic.
    #[test]
    fn mmc_counts_traffic() {
        let (mut emac, mb) = emac();
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS,
            &[0xDE, 0xAD, 0xBE, 0xEF],
        );
        assert_eq!(read_word(&mut emac, 0x118), 1, "TXCNTGB");
        assert_eq!(read_word(&mut emac, 0x164), 4, "TXOCTCNTG");
        assert_eq!(read_word(&mut emac, 0x180), 0, "no RX yet");
        let st = rx_status(&mut emac, &mb, 0, &[0x02, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(st & (RDES0_FS | RDES0_LS), RDES0_FS | RDES0_LS);
        assert_eq!(read_word(&mut emac, 0x180), 1, "RXCNTGB");
        assert_eq!(read_word(&mut emac, 0x1C4), 1, "RXCNTGUNI unicast");
        assert_eq!(read_word(&mut emac, 0x194), 0, "no CRC errors");
    }

    /// `CNTRST` / `CNTPRST` / `CNTFREEZ` / `RSTONRD` modes.
    #[test]
    fn mmc_control_modes() {
        let (mut emac, mb) = emac();
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS,
            &[0xDE, 0xAD, 0xBE, 0xEF],
        );
        assert_eq!(read_word(&mut emac, 0x118), 1);
        // Reset-all is self-clearing.
        write_word(&mut emac, MMCCTRL_OFF, MMC_CNTRST);
        assert_eq!(read_word(&mut emac, 0x118), 0);
        assert_eq!(read_word(&mut emac, MMCCTRL_OFF) & MMC_CNTRST, 0);
        // Preset-half literals from the datasheet.
        write_word(&mut emac, MMCCTRL_OFF, MMC_CNTPRST);
        assert_eq!(read_word(&mut emac, 0x118), 0x7FFF_FFF0);
        assert_eq!(read_word(&mut emac, 0x164), 0x7FFF_F800);
        assert_eq!(read_word(&mut emac, MMCCTRL_OFF) & MMC_CNTPRST, 0);
        // Preset-full.
        write_word(&mut emac, MMCCTRL_OFF, MMC_CNTPRST | MMC_CNTPRSTLVL);
        assert_eq!(read_word(&mut emac, 0x118), 0xFFFF_FFF0);
        // Freeze stops counting; unfreeze resumes.
        write_word(&mut emac, MMCCTRL_OFF, MMC_CNTRST | MMC_CNTFREEZ);
        tx_frame(&mut emac, &mb, OWN | TDES0_LS | TDES0_FS, &[0x01, 0x02]);
        assert_eq!(read_word(&mut emac, 0x118), 0, "frozen");
        write_word(&mut emac, MMCCTRL_OFF, 0);
        tx_frame(&mut emac, &mb, OWN | TDES0_LS | TDES0_FS, &[0x01, 0x02]);
        assert_eq!(read_word(&mut emac, 0x118), 1, "resumed");
        // Reset-on-read zeroes the counter just read.
        write_word(&mut emac, MMCCTRL_OFF, MMC_RSTONRD);
        assert_eq!(read_word(&mut emac, 0x118), 1, "value first");
        assert_eq!(read_word(&mut emac, 0x118), 0, "then zero");
    }

    /// Half/full interrupts, counter-read clear, masks, `EMACRIS`.
    #[test]
    fn mmc_interrupts_and_masks() {
        let (mut emac, mb) = emac();
        // Preset TXCNTGB almost-full, then one frame crosses max.
        write_word(&mut emac, MMCCTRL_OFF, MMC_CNTPRST | MMC_CNTPRSTLVL);
        assert_eq!(read_word(&mut emac, MMCTXRIS_OFF), 0);
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS,
            &[
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
                0x0F, 0x10,
            ],
        );
        assert_eq!(
            read_word(&mut emac, MMCTXRIS_OFF) & (1 << 1),
            1 << 1,
            "TX GBF crossed max"
        );
        assert_eq!(
            read_word(&mut emac, EMACRIS_OFF) & (RIS_MMC | RIS_MMCTX),
            RIS_MMC | RIS_MMCTX
        );
        assert_ne!(emac.pending(), 0, "MMC raises the line");
        // Reading the counter clears its interrupt (octet counter too).
        read_word(&mut emac, 0x118);
        read_word(&mut emac, 0x164);
        assert_eq!(read_word(&mut emac, MMCTXRIS_OFF), 0);
        assert_eq!(read_word(&mut emac, EMACRIS_OFF) & (RIS_MMC | RIS_MMCTX), 0);
        // Mask suppresses the summary but not the raw bit.
        write_word(&mut emac, MMCCTRL_OFF, MMC_CNTPRST | MMC_CNTPRSTLVL);
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS,
            &[
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
                0x0F, 0x10,
            ],
        );
        write_word(&mut emac, MMCTXIM_OFF, (1 << 1) | (1 << 20));
        assert_eq!(
            read_word(&mut emac, MMCTXRIS_OFF) & (1 << 1),
            1 << 1,
            "raw bit still set"
        );
        assert_eq!(read_word(&mut emac, EMACRIS_OFF) & (RIS_MMC | RIS_MMCTX), 0);
    }

    /// TX version mismatch flags `IHE` + `ES` in the write-back.
    #[test]
    fn tx_ihe_on_version_mismatch() {
        let (mut emac, mb) = emac();
        let mut bad = vec![0x60, 0x00, 0x00, 0x00];
        bad.resize(20, 0);
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | (1 << TDES0_CIC_SHIFT),
            &eth_frame(&bad),
        );
        assert_eq!(
            tx_status_wb(&mb) & (TDES0_IHE | TDES0_ES),
            TDES0_IHE | TDES0_ES
        );
    }
}
