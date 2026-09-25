//! TI Tiva C Ethernet MAC (`EMAC0`, `0x400E_C000`).
//!
//! Register map and MII/descriptor semantics follow the TI `hw_emac.h` +
//! `emac.c`/`lwiplib` oracle (Synopsys DWC core, BSD): `MIIADDR` `MIIB`/
//! `MIIW`/`CR`/`MII`/`PLA`, `MIIDATA`, `CFG` `RE`/`TE`, enhanced
//! descriptors (`TDES0`: `OWN` 31/`IC` 30/`LS` 29/`FS` 28/`TCH` 20/
//! `TER` 21; `TDES1`: `TBS1` 12:0/`TBS2` 28:16/`SAIC` 31:29; `TDES2/3` buffer/next;
//! `RDES0`: `OWN`/`ES` 15/`FL` 29:16/`FS` 9/`LS` 8/`DE` 14/`OE` 11; `RDES1`: `RBS1`
//! 12:0/`RCH` 14/`RER` 15/`DIC` 31), `DMAOPMODE` `ST`/`SR`,
//! `DMARIS` `TI`/`RI`/`NIS`, `DMABUSMOD` `ATDS` 7. The SVD carries no `<resetValue>`, so all
//! words reset 0.
//!
//! Modeled: instant `MIIB` completion (no PHY — `MIIDATA` reads
//! all-ones), config/address latches, descriptor-ring DMA through
//! the shared [`EthMailbox`] (same bus coupling as the H7 model):
//! TX reads descriptors + buffers on `TXPOLLD`/store-forward, clears
//! `OWN`, latches `TI` only if the closing descriptor's `IC` is set
//! (Table 20-2); RX stores injected frames into owned
//! descriptors with `FS`/`LS`/`FL` (length as delivered — CRC bytes are
//! not appended; `FL` is the truncated length with `DE`+`ES` when the
//! frame exceeds `RBS1`, valid only when `DE`/`OE` are clear per
//! Table 20-8), latches `RI` unless `DIC` suppresses it
//! (Table 20-10); `DMARIS` W1C, `NIS` synthesis.
//! Checksum offload (SPMS433B §20.3.8, DWC Type 2): TX `TDES0.CIC`
//! insertion (IPv4 header + TCP/UDP/ICMP/ICMPv6, full or segment-only)
//! with `IHE`/`IPE`/`ES` write-back (`IPE` on IP payload-length mismatch,
//! ORed into `ES` per Table 20-2); RX validation under `EMACCFG.IPC`
//! driving Table 20-9 `RDES0` bits (header-error forces payload-error so
//! the reserved `(FT=1,HERR=1,PERR=0)` encoding never emits — it maps to
//! the both-errors row). MMC counters (§20.3.9, all eight
//! 32-bit) with `MMCCTRL` modes, half/full interrupts + masks.
//! Also modeled (package 4, SPMS433B §20.3.5/§20.3.9.3/§20.3.10): the
//! PTP system-time generator (coarse `SSINC` or fine `TSAR` advance,
//! `TSINIT`/`TSUPDT`/`ADDSUB`, digital/binary rollover, target-time
//! `TT` interrupt + `TIMSTAT`), TX snapshots (`TTSE` → `TDES6/7` +
//! `TTSS`, gated on `DMABUSMOD.ATDS` per §20.3.2.5) and RX snapshots
//! (`ALLF`/PTP-select → `RDES6/7` + `RDES4`
//! info + `RDES0[7]`, likewise `ATDS`-gated), PMT magic-packet + global-unicast wake with
//! power-down gating (`WUPRX`/`MGKPRX` read-clear, `DMARIS.PMT` +
//! `EMACRIS.PMT`), TX `DC`/`CRCR` FCS append/replace + `DP` padding
//! (IEEE 802.3 CRC-32), `SAIC` SA insert/replace from `ADDR0`/`ADDR1`
//! with FCS corrected via the same CRC-32 path, and `VLIC` strip/insert/replace via
//! `EMACVLNINCREP` (`VF` status, RX VLAN-presence bit).
//! Deliberate scope: remote wake-up pattern CRC match (filter-bank
//! mechanics only), second-buffer (`TBS2`/`RBS2`) scatter, DMA burst
//! arbitration (instant, unpaced) + `DMAIM` masking (one shared line,
//! unmasked), IPv6 extension headers, `UCDBC`, PPS output (latched),
//! RX FCS validation (injected frames carry none). Notes: store-forward is structural here (the frame only
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
//! pseudo-header partial sum the result verifies exactly. (b) `SAIC`
//! with bit 31 set uses `ADDR1H/L` when programmed, else `ADDR0H/L`
//! (the model keeps both latches; reset zeros read as zero MAC).
//! The `ES` line mentioning `RDES[4:3]` matches no checksum field in
//! `RDES4` (timestamp/PTP only) — taken as a doc carryover; `ES` ORs
//! the modeled error bits.

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
/// `TDES0.IC` (Table 20-2 bit 30): gates `DMARIS.TI` on completion.
const TDES0_IC: u32 = 1 << 30;
/// `TDES0.IPE` (Table 20-2 bit 12): payload-length mismatch, ORed into `ES`.
const TDES0_IPE: u32 = 1 << 12;
/// `RDES0` checksum/status bits (Tables 20-8/20-9).
const RDES0_ES: u32 = 1 << 15;
const RDES0_FT: u32 = 1 << 5;
const RDES0_IPCERR: u32 = 1 << 7;
const RDES0_PCSERR: u32 = 1 << 0;
/// `RDES0` VLAN-tag status (Table 20-8).
const RDES0_VLAN: u32 = 1 << 10;
/// `RDES0.DE` (Table 20-8 bit 14): truncation, ORed into `ES`.
const RDES0_DE: u32 = 1 << 14;
/// `RDES0.OE` (Table 20-8 bit 11): overflow; `FL` valid only if `DE`/`OE` clear.
const RDES0_OE: u32 = 1 << 11;
/// `RDES1.DIC` (Table 20-10 bit 31): suppresses `DMARIS.RI`.
const RDES1_DIC: u32 = 1 << 31;
/// `TDES0` TX-path control/status bits (Table 20-2).
const TDES0_DC: u32 = 1 << 27;
const TDES0_DP: u32 = 1 << 26;
const TDES0_TTSE: u32 = 1 << 25;
const TDES0_CRCR: u32 = 1 << 24;
const TDES0_VLIC_SHIFT: u32 = 18;
const TDES0_TTSS: u32 = 1 << 17;
const TDES0_VF: u32 = 1 << 7;
/// `DMARIS` timestamp/PMT bits.
const DMA_TT: u32 = 1 << 29;
const DMA_PMT: u32 = 1 << 28;
/// Power-management registers (`EMACRWUFF` bank mechanics only; the
/// wake-pattern CRC engine stays latched).
const RWUFF_OFF: u32 = 0x028;
const PMTCTLSTAT_OFF: u32 = 0x02C;
const PMT_PWRDWN: u32 = 1 << 0;
const PMT_MGKPKTEN: u32 = 1 << 1;
const PMT_WUPFREN: u32 = 1 << 2;
const PMT_MGKPRX: u32 = 1 << 5;
const PMT_WUPRX: u32 = 1 << 6;
const PMT_GLBLUCAST: u32 = 1 << 9;
const PMT_RWKPTR_SHIFT: u32 = 24;
const PMT_RWKPTR_MASK: u32 = 0x7 << PMT_RWKPTR_SHIFT;
const PMT_WUPFRRST: u32 = 1 << 31;
const PMT_WRITABLE: u32 = PMT_PWRDWN | PMT_MGKPKTEN | PMT_WUPFREN | PMT_GLBLUCAST;
/// `EMACRIS` timestamp/PMT reflections.
const RIS_TS: u32 = 1 << 9;
const RIS_PMT: u32 = 1 << 3;
/// MAC address + TX VLAN registers.
const ADDR0H_OFF: u32 = 0x040;
const ADDR0L_OFF: u32 = 0x044;
const ADDR1H_OFF: u32 = 0x048;
const ADDR1L_OFF: u32 = 0x04C;
/// DMA bus mode + `ATDS` (Table 20-23, §20.3.2.5 bit 7): extended
/// descriptors `DES4-7` valid only when set.
const BUSMOD_OFF: u32 = 0xC00;
const BUSMOD_ATDS: u32 = 1 << 7;
/// `TDES1.SAIC` (Table 20-3 bits 31:29): bit 31 selects `ADDR1`/`ADDR0`,
/// bits 30:29 select 0 none / 1 insert / 2 replace (3 reserved).
const TDES1_SAIC_SHIFT: u32 = 29;
const VLNINCREP_OFF: u32 = 0x584;
const VLN_VLC_SHIFT: u32 = 16;
const VLN_VLP: u32 = 1 << 18;
const VLN_CSVL: u32 = 1 << 19;
/// PTP timestamp registers.
const TIMSTCTRL_OFF: u32 = 0x700;
const SUBSECINC_OFF: u32 = 0x704;
const TIMSEC_OFF: u32 = 0x708;
const TIMNANO_OFF: u32 = 0x70C;
const TIMSECU_OFF: u32 = 0x710;
const TIMNANOU_OFF: u32 = 0x714;
const TIMADD_OFF: u32 = 0x718;
const TARGSEC_OFF: u32 = 0x71C;
const TARGNANO_OFF: u32 = 0x720;
const HWORDSEC_OFF: u32 = 0x724;
const TIMSTAT_OFF: u32 = 0x728;
/// `TIMSTCTRL` control bits.
const TS_TSEN: u32 = 1 << 0;
const TS_TSFCUPDT: u32 = 1 << 1;
const TS_TSINIT: u32 = 1 << 2;
const TS_TSUPDT: u32 = 1 << 3;
const TS_INTTRIG: u32 = 1 << 4;
const TS_ADDREGUP: u32 = 1 << 5;
const TS_ALLF: u32 = 1 << 8;
const TS_DGTLBIN: u32 = 1 << 9;
const TS_PTPVER2: u32 = 1 << 10;
const TS_PTPETH: u32 = 1 << 11;
const TS_PTPIPV6: u32 = 1 << 12;
const TS_PTPIPV4: u32 = 1 << 13;
const TS_TSEVNT: u32 = 1 << 14;
const TS_TSMAST: u32 = 1 << 15;
const TS_SELP_SHIFT: u32 = 16;
/// `TIMSTAT` status bits.
const TIM_TSTARGT: u32 = 1 << 1;
const TIM_TSSOVF: u32 = 1 << 0;
/// `TARGNANO` busy bit: PPS modes unmodeled, always reads 0.
const TARG_TRGTBUSY: u32 = 1 << 31;
/// `RDES4` PTP info bits (Table 20-13).
const RDES4_VER: u32 = 1 << 13;
const RDES4_FTYPE: u32 = 1 << 12;
const RDES4_MSG_SHIFT: u32 = 8;
const RDES4_IPV6: u32 = 1 << 7;
const RDES4_IPV4: u32 = 1 << 6;
const RDES4_BYP: u32 = 1 << 5;
const RDES4_PERR: u32 = 1 << 4;
const RDES4_HERR: u32 = 1 << 3;
/// Digital-rollover nanoseconds per second.
const NS_PER_SEC: u64 = 1_000_000_000;
/// Binary-rollover sub-second modulus (2^31, ~0.465 ns/bit).
const BIN_MOD: u64 = 0x8000_0000;

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
    // UDP may legally send zero ("not computed"): accept when the header
    // is good; a header error still poisons the verdict (no phantom row).
    if proto == 17 && stored == 0 {
        return Some((true, !header_ok, !header_ok));
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
    // Table 20-9 has no `(FT=1,HERR=1,PERR=0)` row (reserved row 8 is
    // `(0,1,0)`): a header error poisons the payload verdict, so map to
    // the nearest real row — both errors.
    Some((true, !header_ok, !header_ok || !payload_ok))
}

/// IEEE 802.3 CRC-32 (reflected, init all-ones, xor-out): the FCS
/// the MAC appends (`DC=0`) or swaps in (`DC=1` + `CRCR`).
fn crc32_ieee(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in bytes {
        crc ^= u32::from(b);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// Whether the frame carries a VLAN tag (`0x8100`/`0x88A8`).
fn frame_tagged(frame: &[u8]) -> bool {
    if frame.len() < 14 {
        return false;
    }
    let tpid = u16::from_be_bytes([frame[12], frame[13]]);
    tpid == 0x8100 || tpid == 0x88A8
}

/// TX payload-length mismatch (Table 20-2 `IPE` bit 12): the transmitter
/// checks the IP-declared payload length against the actual TCP/UDP/ICMP
/// bytes. `true` on mismatch. Non-IP, unsupported L4, or short headers
/// never mismatch (no verdict).
fn tx_payload_mismatch(frame: &[u8]) -> bool {
    let (ether, ip_off) = match eth_ip_split(frame) {
        Some(v) => v,
        None => return false,
    };
    let (is_v4, is_v6) = (ether == 0x0800, ether == 0x86DD);
    if !is_v4 && !is_v6 {
        return false;
    }
    let ip = match frame.get(ip_off..) {
        Some(v) => v,
        None => return false,
    };
    let version = ip.first().copied().unwrap_or(0) >> 4;
    if is_v4 {
        if version != 4 || ip.len() < 20 {
            return false;
        }
        let ihl = usize::from(ip[0] & 0x0F) * 4;
        if ihl < 20 || ip.len() < ihl {
            return false;
        }
        let proto = ip[9];
        if l4_slot(4, proto).is_none() {
            return false;
        }
        let total = usize::from(u16::from_be_bytes([ip[2], ip[3]]));
        if total < ihl {
            return false;
        }
        let expect = total - ihl;
        let seg_off = ip_off + ihl;
        let avail = frame.len().saturating_sub(seg_off);
        avail != expect
    } else {
        if version != 6 || ip.len() < 40 {
            return false;
        }
        let proto = ip[6];
        if l4_slot(6, proto).is_none() {
            return false;
        }
        let expect = usize::from(u16::from_be_bytes([ip[4], ip[5]]));
        let seg_off = ip_off + 40;
        let avail = frame.len().saturating_sub(seg_off);
        avail != expect
    }
}

/// Apply a `SAIC` source-address edit (Table 20-3 + §20.3.7.1): `1`
/// insert `sa` after the destination address (frames arrive without `SA`),
/// `2` replace the six bytes following `DA` in place. Short frames pass
/// through untouched; `0`/`3` (reserved) are no-ops.
fn sa_edit(frame: Vec<u8>, action: u32, sa: [u8; 6]) -> Vec<u8> {
    match action {
        1 => {
            if frame.len() < 6 {
                return frame;
            }
            let mut f = frame[..6].to_vec();
            f.extend_from_slice(&sa);
            f.extend_from_slice(&frame[6..]);
            f
        }
        2 => {
            if frame.len() < 12 {
                return frame;
            }
            let mut f = frame;
            f[6..12].copy_from_slice(&sa);
            f
        }
        _ => frame,
    }
}

/// Apply a `VLIC`/register-`VLC` tag edit (Table 20-2 + `EMACVLNINCREP`):
/// `1` strip, `2` insert `tpid`/`vlt` after the source address, `3`
/// replace the tag in place. Short or untagged frames pass through
/// untouched for strip/replace.
fn vlan_edit(frame: Vec<u8>, action: u32, tpid: u16, vlt: u16) -> Vec<u8> {
    if frame.len() < 14 {
        return frame;
    }
    match action {
        1 => {
            if frame_tagged(&frame) {
                let mut f = frame[..12].to_vec();
                f.extend_from_slice(&frame[16..]);
                f
            } else {
                frame
            }
        }
        2 => {
            let mut f = frame[..12].to_vec();
            f.extend_from_slice(&tpid.to_be_bytes());
            f.extend_from_slice(&vlt.to_be_bytes());
            f.extend_from_slice(&frame[12..]);
            f
        }
        3 => {
            if frame_tagged(&frame) {
                let mut f = frame;
                f[12..14].copy_from_slice(&tpid.to_be_bytes());
                f[14..16].copy_from_slice(&vlt.to_be_bytes());
                f
            } else {
                frame
            }
        }
        _ => frame,
    }
}

/// PTP payload location for a frame: `(eth_direct, ptp_offset)`.
/// IPv4/UDP and IPv6/UDP (no extension headers) plus EtherType
/// `0x88F7` directly, each gated by its `TIMSTCTRL` transport enable.
/// UDP port filtering is not modeled (no port gate appears in the
/// databook's PTP detection description).
fn ptp_info(frame: &[u8], ctrl: u32) -> Option<(bool, bool, u8)> {
    let (ether, off) = eth_ip_split(frame)?;
    let (eth_direct, ptp_off) = if ether == 0x88F7 {
        if ctrl & TS_PTPETH == 0 {
            return None;
        }
        (true, off)
    } else if ether == 0x0800 {
        if ctrl & TS_PTPIPV4 == 0 {
            return None;
        }
        if frame.len() < off + 20 {
            return None;
        }
        let ihl = ((frame[off] & 0x0F) as usize) * 4;
        if ihl < 20 || frame[off + 9] != 17 {
            return None;
        }
        let u = off + ihl;
        if frame.len() < u + 8 {
            return None;
        }
        (false, u + 8)
    } else if ether == 0x86DD {
        if ctrl & TS_PTPIPV6 == 0 {
            return None;
        }
        if frame.len() < off + 40 || frame[off + 6] != 17 {
            return None;
        }
        (false, off + 48)
    } else {
        return None;
    };
    if frame.len() < ptp_off + 2 {
        return None;
    }
    let msg = frame[ptp_off] & 0x0F;
    let ver = frame[ptp_off + 1] & 0x0F;
    if ctrl & TS_PTPVER2 != 0 {
        if ver != 2 {
            return None;
        }
    } else if ver != 1 {
        return None;
    }
    // Message-class filter (judgment: the databook names the selects
    // but no exact class table, so master nodes take request classes,
    // slave nodes the rest, and Pdelay classes need peer-to-peer
    // `SELPTP`; event-only drops Announce/Management/Signaling).
    if matches!(msg, 0x5..=0x7) && (ctrl >> TS_SELP_SHIFT) & 3 != 3 {
        return None;
    }
    let master_set = matches!(msg, 0x3 | 0x5);
    if ctrl & TS_TSMAST != 0 {
        if !master_set {
            return None;
        }
    } else if master_set {
        return None;
    }
    if ctrl & TS_TSEVNT != 0 {
        if !matches!(msg, 0x1 | 0x3 | 0x5 | 0x6) {
            return None;
        }
    } else if matches!(msg, 0x8..=0xA) {
        return None;
    }
    Some((eth_direct, ver == 2, msg))
}

/// Station MAC bytes from `EMACADDR0H/L` (first DA byte on the wire is
/// `ADDR0L[7:0]`, Table 20 register 14/15).
fn station_mac<W>(view: &Registers<W>) -> [u8; 6]
where
    W: Deref<Target = [u32]>,
{
    mac_from(view, ADDR0L_OFF, ADDR0H_OFF)
}

/// MAC bytes from an `ADDRnH/L` pair (shared wire order for `ADDR0/1`).
fn mac_from<W>(view: &Registers<W>, lo_off: u32, hi_off: u32) -> [u8; 6]
where
    W: Deref<Target = [u32]>,
{
    let lo = view.get(lo_off);
    let hi = view.get(hi_off);
    [
        (lo & 0xFF) as u8,
        ((lo >> 8) & 0xFF) as u8,
        ((lo >> 16) & 0xFF) as u8,
        ((lo >> 24) & 0xFF) as u8,
        (hi & 0xFF) as u8,
        ((hi >> 8) & 0xFF) as u8,
    ]
}

/// Magic-packet match (§20.3.10.3): DA is ours, broadcast, or multicast,
/// then a 6-byte all-ones sync stream (scanned from past DA+SA, so a
/// break simply resumes the scan) followed by 16 contiguous repetitions
/// of the station address.
fn is_magic_packet(frame: &[u8], mac: &[u8; 6]) -> bool {
    if frame.len() < 12 {
        return false;
    }
    let da = &frame[..6];
    if da != mac && *da != [0xFF; 6] && da[0] & 1 == 0 {
        return false;
    }
    let mut rep = [0u8; 96];
    for chunk in rep.chunks_mut(6) {
        chunk.copy_from_slice(mac);
    }
    let mut i = 12;
    while i + 102 <= frame.len() {
        if frame[i..i + 6] == [0xFF; 6] && frame[i + 6..i + 102] == rep {
            return true;
        }
        i += 1;
    }
    false
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
    /// `DC`/`DP`/`TTSE`/`CRCR`/`VLIC`/`SAIC` latched from the frame's first
    /// descriptor (validity is FS-gated per Table 20-2/20-3).
    tx_dc: bool,
    tx_dp: bool,
    tx_ttse: bool,
    tx_crcr: bool,
    tx_vlic: u32,
    tx_saic: u32,
    /// PTP system time: seconds, sub-seconds, higher-word seconds,
    /// fine-correction accumulator, seconds-overflow latch.
    ts_sec: u32,
    ts_nsss: u32,
    ts_hword: u16,
    ts_acc: u32,
    ts_tssovf: bool,
    /// PMT wake flags (`WUPRX`/`MGKPRX`, cleared on register read).
    pmt_wuprx: bool,
    pmt_mgkprx: bool,
    /// Remote wake-up filter bank + pointer + word-read cache (pattern
    /// CRC matching stays latched; only the bank mechanics move).
    rwuff_bank: [u32; 8],
    rwuff_ptr: u8,
    /// Word staged by the lane-0 peek (lanes 1-3 of a word read repeat
    /// it; word reads must start at lane 0, the `read_word` convention).
    rwuff_cache: u32,
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

    /// Add whole seconds with wrap: overflow bumps the higher word
    /// and latches `TSSOVF` (the 130-year register still counts here).
    fn ptp_add_sec(&mut self, carry: u64) {
        let total = u64::from(self.ts_sec) + carry;
        self.ts_sec = (total & 0xFFFF_FFFF) as u32;
        if total >> 32 != 0 {
            self.ts_hword = self.ts_hword.wrapping_add((total >> 32) as u16);
            self.ts_tssovf = true;
        }
    }

    /// Fold raw sub-seconds into seconds per the rollover mode.
    fn ptp_normalize(&mut self, nsss: u64, digital: bool) {
        if digital {
            self.ptp_add_sec(nsss / NS_PER_SEC);
            self.ts_nsss = (nsss % NS_PER_SEC) as u32;
        } else {
            self.ptp_add_sec(nsss / BIN_MOD);
            self.ts_nsss = (nsss % BIN_MOD) as u32;
        }
    }

    /// Advance the PTP system time: one device tick is one PTP reference
    /// clock (documented unit — the databook's MOSC cycle). Coarse mode
    /// adds `SSINC` per tick; fine mode accumulates the `TSAR` addend
    /// and adds `SSINC` per accumulator overflow.
    fn ptp_advance<W>(&mut self, view: &Registers<W>, delta: u64)
    where
        W: Deref<Target = [u32]>,
    {
        let ctrl = view.get(TIMSTCTRL_OFF);
        if ctrl & TS_TSEN == 0 || delta == 0 {
            return;
        }
        let ssinc = u64::from(view.get(SUBSECINC_OFF) & 0xFF);
        let mut nsss = u64::from(self.ts_nsss);
        if ctrl & TS_TSFCUPDT != 0 {
            let acc = u64::from(self.ts_acc) + u64::from(view.get(TIMADD_OFF)) * delta;
            self.ts_acc = (acc & 0xFFFF_FFFF) as u32;
            nsss += ssinc * (acc >> 32);
        } else {
            nsss += ssinc * delta;
        }
        self.ptp_normalize(nsss, ctrl & TS_DGTLBIN != 0);
    }

    /// `TSINIT`: overwrite the time from the update registers.
    fn ptp_init(&mut self, sec: u32, ns: u32, digital: bool) {
        if digital {
            self.ts_sec = sec.wrapping_add((u64::from(ns) / NS_PER_SEC) as u32);
            self.ts_nsss = (u64::from(ns) % NS_PER_SEC) as u32;
        } else {
            self.ts_sec = sec;
            self.ts_nsss = (u64::from(ns) % BIN_MOD) as u32;
        }
    }

    /// `TSUPDT`: add or subtract (`ADDSUB`) the update registers. A
    /// subtract past zero saturates at zero (the databook defines no
    /// borrow out of the epoch).
    fn ptp_update(&mut self, sec: u32, ns: u32, sub: bool, digital: bool) {
        if digital {
            let cur = u64::from(self.ts_sec) * NS_PER_SEC + u64::from(self.ts_nsss);
            let adj = u64::from(sec) * NS_PER_SEC + u64::from(ns);
            let out = if sub {
                cur.saturating_sub(adj)
            } else {
                cur + adj
            };
            self.ptp_set_digital(out);
        } else {
            let cur = (u64::from(self.ts_sec) << 31) | u64::from(self.ts_nsss);
            let adj = (u64::from(sec) << 31) | u64::from(ns);
            if sub && adj > cur {
                self.ts_sec = 0;
                self.ts_nsss = 0;
            } else {
                let out = if sub { cur - adj } else { cur + adj };
                let secs = out >> 31;
                if secs >> 32 != 0 {
                    self.ts_hword = self.ts_hword.wrapping_add((secs >> 32) as u16);
                    self.ts_tssovf = true;
                }
                self.ts_sec = (secs & 0xFFFF_FFFF) as u32;
                self.ts_nsss = (out & (BIN_MOD - 1)) as u32;
            }
        }
    }

    /// Split an absolute nanosecond count into seconds + sub-seconds.
    fn ptp_set_digital(&mut self, out: u64) {
        let secs = out / NS_PER_SEC;
        if secs >> 32 != 0 {
            self.ts_hword = self.ts_hword.wrapping_add((secs >> 32) as u16);
            self.ts_tssovf = true;
        }
        self.ts_sec = (secs & 0xFFFF_FFFF) as u32;
        self.ts_nsss = (out % NS_PER_SEC) as u32;
    }

    /// `INTTRIG`: fire once when the time reaches the target (the bit
    /// self-clears; the `TT` status bit clears via `DMARIS` W1C).
    fn ptp_target_check(&mut self, view: &mut Registers<&mut [u32]>) {
        if view.get(TIMSTCTRL_OFF) & TS_INTTRIG == 0 {
            return;
        }
        let tgt = (view.get(TARGSEC_OFF), view.get(TARGNANO_OFF) & 0x7FFF_FFFF);
        if (self.ts_sec, self.ts_nsss & 0x7FFF_FFFF) >= tgt {
            let mut ctrl = view.get(TIMSTCTRL_OFF);
            ctrl &= !TS_INTTRIG;
            view.set(TIMSTCTRL_OFF, ctrl);
            self.dmaris |= DMA_TT;
        }
    }

    /// Power-down RX triage (§20.3.10.5): the frame is consumed by the
    /// PMT, never delivered. A wake event self-clears `PWRDWN` and
    /// latches `DMARIS.PMT`; anything else is dropped silently.
    fn pmt_rx_wake(&mut self, view: &mut Registers<&mut [u32]>, frame: &[u8]) {
        let ctl = view.get(PMTCTLSTAT_OFF);
        let mac = station_mac(view);
        let unicast_us = frame.len() >= 6 && frame[..6] == mac;
        let mut woke = false;
        if ctl & PMT_MGKPKTEN != 0 && is_magic_packet(frame, &mac) {
            self.pmt_mgkprx = true;
            woke = true;
        }
        if ctl & PMT_GLBLUCAST != 0 && unicast_us {
            self.pmt_wuprx = true;
            woke = true;
        }
        // Remote wake-up pattern filters: bank mechanics only, so
        // `WUPFREN` alone never matches (documented).
        if woke {
            let mut pmt = view.get(PMTCTLSTAT_OFF);
            pmt &= !PMT_PWRDWN;
            view.set(PMTCTLSTAT_OFF, pmt);
            self.dmaris |= DMA_PMT;
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
            // `CIC`/`DC`/`DP`/`TTSE`/`CRCR`/`VLIC`/`SAIC` are valid on the
            // first segment; they ride the frame.
            self.tx_cic = (t0 >> TDES0_CIC_SHIFT) & 3;
            self.tx_dc = t0 & TDES0_DC != 0;
            self.tx_dp = t0 & TDES0_DP != 0;
            self.tx_ttse = t0 & TDES0_TTSE != 0;
            self.tx_crcr = t0 & TDES0_CRCR != 0;
            self.tx_vlic = (t0 >> TDES0_VLIC_SHIFT) & 3;
            self.tx_saic = (t1 >> TDES1_SAIC_SHIFT) & 7;
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
            // 1. Source-address insert/replace (Table 20-3 + §20.3.7.1)
            // runs before VLAN: it shifts every later offset. Bit 31
            // selects `ADDR1`/`ADDR0`; bits 30:29 select insert/replace.
            let sa_mode = self.tx_saic & 3;
            if sa_mode == 1 || sa_mode == 2 {
                let sa = if self.tx_saic & 4 != 0 {
                    mac_from(view, ADDR1L_OFF, ADDR1H_OFF)
                } else {
                    mac_from(view, ADDR0L_OFF, ADDR0H_OFF)
                };
                frame = sa_edit(frame, sa_mode, sa);
            }
            // 2. VLAN tag edits shift every later offset, so they run
            // next. The register `VLC` wins when `VLP` says so.
            let vlr = view.get(VLNINCREP_OFF);
            let vlan_action = if vlr & VLN_VLP != 0 {
                (vlr >> VLN_VLC_SHIFT) & 3
            } else {
                self.tx_vlic
            };
            if vlan_action != 0 {
                let tpid = if vlr & VLN_CSVL != 0 { 0x88A8 } else { 0x8100 };
                frame = vlan_edit(frame, vlan_action, tpid, (vlr & 0xFFFF) as u16);
            }
            if frame_tagged(&frame) {
                status |= TDES0_VF;
            }
            // 3. Checksum insertion over the edited frame, plus the
            // payload-length check (`IPE`, ORed into `ES`). `IPE` covers
            // the TCP/UDP/ICMP payload, so it applies only when the
            // payload checksum is in play (`CIC>=2`).
            if self.tx_cic != 0 {
                status |= tx_insert_checksum(&mut frame, self.tx_cic);
                if self.tx_cic >= 2 && tx_payload_mismatch(&frame) {
                    status |= TDES0_IPE | TDES0_ES;
                }
            }
            // 4. Runt padding, then FCS (Table 20-19): `DC=0` appends
            // the computed CRC; `DC=1` + `CRCR` replaces the last four
            // bytes the CPU left in the frame. `SAIC`/`VLIC` edits ride
            // the same CRC-32 path, so the appended/replaced FCS already
            // covers them.
            if !self.tx_dp && frame.len() < 60 {
                frame.resize(60, 0);
            }
            if !self.tx_dc {
                let crc = crc32_ieee(&frame);
                frame.extend_from_slice(&crc.to_be_bytes());
            } else if self.tx_crcr && frame.len() >= 4 {
                let n = frame.len();
                let crc = crc32_ieee(&frame[..n - 4]);
                frame[n - 4..].copy_from_slice(&crc.to_be_bytes());
            }
            // 5. Timestamp snapshot into the extended descriptor words,
            // valid only with `ATDS=1` (§20.3.2.5).
            if view.get(TIMSTCTRL_OFF) & TS_TSEN != 0
                && self.tx_ttse
                && view.get(BUSMOD_OFF) & BUSMOD_ATDS != 0
            {
                status |= TDES0_TTSS;
                self.push_write(desc + 24, self.ts_nsss.to_le_bytes().to_vec());
                self.push_write(desc + 28, self.ts_sec.to_le_bytes().to_vec());
            }
            self.mmc_bump(ctrl, 0, 1);
            self.mmc_bump(ctrl, 3, frame.len() as u32);
            self.tx_frames.push(frame);
            // Table 20-2 `TI` latches only when the closing descriptor's
            // `IC` is set.
            if t0 & TDES0_IC != 0 {
                self.dmaris |= DMA_TI;
            }
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
            if view.get(PMTCTLSTAT_OFF) & PMT_PWRDWN != 0 {
                // Power-down: the PMT consumes the frame (wake or drop),
                // the DMA never sees it.
                self.pmt_rx_wake(view, &frame);
                self.phase = Phase::Idle;
                return;
            }
            let rbs1 = (r1 & RBS1_MASK) as usize;
            let truncated = frame.len() > rbs1;
            let n = frame.len().min(rbs1);
            if n > 0 {
                self.push_write(buf, frame[..n].to_vec());
            }
            // Status: ownership home, first+last, length as delivered
            // (CRC bytes are not appended by this model), plus Table
            // 20-9 checksum bits when `IPC` is set. Table 20-8: `DE`
            // on truncation, `FL` valid only when `DE`/`OE` are clear
            // (`OE` never sets in store-forward; partials drop).
            let fl = ((n as u32) & 0x3FFF) << 16;
            let mut status = fl | RDES0_FS | RDES0_LS;
            if truncated {
                status |= RDES0_DE | RDES0_ES;
            }
            if frame_tagged(&frame) {
                status |= RDES0_VLAN;
            }
            let ipc = view.get(CFG_OFF) & CFG_IPC != 0;
            if ipc {
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
            // Timestamp snapshot into the extended descriptor words,
            // valid only with `ATDS=1` (§20.3.2.5).
            // `RDES0[7]` doubles as timestamp-available here (the same
            // bit reads `IPCERR` for checksum errors above).
            let tctrl = view.get(TIMSTCTRL_OFF);
            if tctrl & TS_TSEN != 0
                && view.get(BUSMOD_OFF) & BUSMOD_ATDS != 0
                && (tctrl & TS_ALLF != 0 || ptp_info(&frame, tctrl).is_some())
            {
                status |= RDES0_IPCERR;
                let mut rdes4 = 0u32;
                if let Some((eth, v2, msg)) = ptp_info(&frame, tctrl) {
                    if v2 {
                        rdes4 |= RDES4_VER;
                    }
                    if eth {
                        rdes4 |= RDES4_FTYPE;
                    }
                    rdes4 |= u32::from(msg) << RDES4_MSG_SHIFT;
                }
                if ipc {
                    if let Some((_, herr, perr)) = rx_checksum_status(&frame) {
                        if let Some((ether, _)) = eth_ip_split(&frame) {
                            if ether == 0x0800 {
                                rdes4 |= RDES4_IPV4;
                            } else if ether == 0x86DD {
                                rdes4 |= RDES4_IPV6;
                            }
                        }
                        if herr {
                            rdes4 |= RDES4_HERR;
                        }
                        if perr {
                            rdes4 |= RDES4_PERR;
                        }
                    } else {
                        rdes4 |= RDES4_BYP;
                    }
                } else {
                    rdes4 |= RDES4_BYP;
                }
                self.push_write(self.rx_desc + 16, rdes4.to_le_bytes().to_vec());
                self.push_write(self.rx_desc + 24, self.ts_nsss.to_le_bytes().to_vec());
                self.push_write(self.rx_desc + 28, self.ts_sec.to_le_bytes().to_vec());
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
            // Table 20-10 `DIC` suppresses `RI` (and the interrupt).
            if r1 & RDES1_DIC == 0 {
                self.dmaris |= DMA_RI;
            }
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
        self.tx_dc = false;
        self.tx_dp = false;
        self.tx_ttse = false;
        self.tx_crcr = false;
        self.tx_vlic = 0;
        self.tx_saic = 0;
        self.ts_sec = 0;
        self.ts_nsss = 0;
        self.ts_hword = 0;
        self.ts_acc = 0;
        self.ts_tssovf = false;
        self.pmt_wuprx = false;
        self.pmt_mgkprx = false;
        self.rwuff_bank = [0; 8];
        self.rwuff_ptr = 0;
        self.rwuff_cache = 0;
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
        u32::from(self.dmaris & (DMA_TI | DMA_RI | DMA_TT | DMA_PMT) != 0 || rx || tx)
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, delta: u64) {
        // The time generator runs on every tick, even mid-DMA.
        self.ptp_advance(view, delta);
        self.ptp_target_check(view);
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
                // MAC `RIS`: MMC summaries plus the timestamp (`TT`
                // latch or seconds overflow) and PMT (wake flags)
                // reflections.
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
                if self.dmaris & DMA_TT != 0 || self.ts_tssovf {
                    r |= RIS_TS;
                }
                if self.pmt_wuprx || self.pmt_mgkprx {
                    r |= RIS_PMT;
                }
                r
            }
            TIMSEC_OFF => self.ts_sec,
            TIMNANO_OFF => self.ts_nsss & 0x7FFF_FFFF,
            HWORDSEC_OFF => u32::from(self.ts_hword),
            TIMSTAT_OFF => {
                let mut t = 0;
                if self.ts_tssovf {
                    t |= TIM_TSSOVF;
                }
                if (self.ts_sec, self.ts_nsss)
                    >= (view.get(TARGSEC_OFF), view.get(TARGNANO_OFF) & 0x7FFF_FFFF)
                {
                    t |= TIM_TSTARGT;
                }
                t
            }
            TARGNANO_OFF => value & !TARG_TRGTBUSY,
            RWUFF_OFF => self.rwuff_cache,
            PMTCTLSTAT_OFF => {
                (value & !(PMT_WUPRX | PMT_MGKPRX | PMT_RWKPTR_MASK))
                    | (u32::from(self.rwuff_ptr) << PMT_RWKPTR_SHIFT)
                    | if self.pmt_wuprx { PMT_WUPRX } else { 0 }
                    | if self.pmt_mgkprx { PMT_MGKPRX } else { 0 }
            }
            MMCRXRIS_OFF => self.rx_ris,
            MMCTXRIS_OFF => self.tx_ris,
            DMARIS_OFF => {
                let mut d = self.dmaris;
                if d & (DMA_TI | DMA_RI | DMA_TT | DMA_PMT) != 0 {
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
        if offset == TIMSTCTRL_OFF && shift == 24 {
            let mut ctrl = view.merged(offset, val, shift);
            if ctrl & TS_TSINIT != 0 {
                let digital = ctrl & TS_DGTLBIN != 0;
                self.ptp_init(
                    view.get(TIMSECU_OFF),
                    view.get(TIMNANOU_OFF) & 0x7FFF_FFFF,
                    digital,
                );
                ctrl &= !TS_TSINIT;
            }
            if ctrl & TS_TSUPDT != 0 {
                let digital = ctrl & TS_DGTLBIN != 0;
                let ns = view.get(TIMNANOU_OFF);
                self.ptp_update(
                    view.get(TIMSECU_OFF),
                    ns & 0x7FFF_FFFF,
                    ns & 0x8000_0000 != 0,
                    digital,
                );
                ctrl &= !TS_TSUPDT;
            }
            // The addend register is live (no shadow), so `ADDREGUP`
            // completes the moment it is written.
            ctrl &= !TS_ADDREGUP;
            view.set(offset, ctrl);
            return ctrl;
        }
        if offset == PMTCTLSTAT_OFF {
            // Status bits are read-only; the filter pointer is
            // behavior-side. Whole-word stores additionally service
            // the pointer reset.
            let merged = view.merged(offset, val, shift);
            if shift == 24 && merged & PMT_WUPFRRST != 0 {
                self.rwuff_ptr = 0;
            }
            let word = (merged & PMT_WRITABLE) | (view.get(offset) & !PMT_WRITABLE);
            view.set(offset, word);
            return word;
        }
        if offset == RWUFF_OFF && shift == 24 {
            // Sequential bank load: whole-word stores rotate the pointer.
            let word = view.merged(offset, val, shift);
            self.rwuff_bank[self.rwuff_ptr as usize] = word;
            self.rwuff_ptr = (self.rwuff_ptr + 1) & 7;
            view.set(offset, word);
            return word;
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
                    tx_dc: false,
                    tx_dp: false,
                    tx_ttse: false,
                    tx_crcr: false,
                    tx_vlic: 0,
                    tx_saic: 0,
                    ts_sec: 0,
                    ts_nsss: 0,
                    ts_hword: 0,
                    ts_acc: 0,
                    ts_tssovf: false,
                    pmt_wuprx: false,
                    pmt_mgkprx: false,
                    rwuff_bank: [0; 8],
                    rwuff_ptr: 0,
                    rwuff_cache: 0,
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
        // The `RWUFF` bank peeks on lane 0 (the staged word feeds
        // lanes 1-3, so word reads must start at lane 0).
        if addr & 3 == 0 && addr.wrapping_sub(EMAC_BASE) == RWUFF_OFF {
            self.inner.with_view(|_, b| {
                b.rwuff_cache = b.rwuff_bank[b.rwuff_ptr as usize];
                b.rwuff_ptr = (b.rwuff_ptr + 1) & 7;
            });
        }
        let byte = self.inner.read(addr)?;
        // Every other lane-0 side effect returns the word first, then
        // moves the state (`on_read` sees per-byte calls with no lane):
        // `RSTONRD` + interrupt-clear on the counters, the PMT wake
        // flags, and the timestamp overflow latch.
        if addr & 3 == 0 {
            let off = addr.wrapping_sub(EMAC_BASE);
            self.inner.with_view(|view, b| {
                b.counter_lane0_read(view, off);
                if off == PMTCTLSTAT_OFF {
                    b.pmt_wuprx = false;
                    b.pmt_mgkprx = false;
                }
                if off == TIMSTAT_OFF {
                    b.ts_tssovf = false;
                }
            });
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

    fn new_emac() -> (Emac, Arc<Mutex<EthMailbox>>) {
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
        rx_status_with_r1(emac, mb, cfg, 1536, frame)
    }

    /// Run one RX frame with an explicit `RDES1` word (for `DIC`/`RBS1`
    /// cases); returns `(status, RI latched)`.
    fn rx_status_with_r1(
        emac: &mut Emac,
        mb: &Arc<Mutex<EthMailbox>>,
        cfg: u32,
        r1: u32,
        frame: &[u8],
    ) -> u32 {
        write_word(emac, DMAOPMODE_OFF, OP_SR);
        write_word(emac, CFG_OFF, cfg);
        write_word(emac, RXDLADDR_OFF, 0x2000_1000);
        emac.inject_frame(frame);
        emac.tick(1);
        let desc = [OWN, r1, 0x2000_0200, 0x2000_1010]
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
        let (mut emac, _) = new_emac();
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
        let (mut emac, _) = new_emac();
        write_word(&mut emac, MIIADDR_OFF, 0x01); // MIIB + PHY 0/reg 0.
        assert_eq!(read_word(&mut emac, MIIADDR_OFF) & MII_MIIB, 0);
        assert_eq!(read_word(&mut emac, MIIDATA_OFF), 0xFFFF);
    }

    /// TX: descriptor → buffer → ownership home + `TI`, frame observed.
    /// Table 20-2: `TI` latches only when the closing descriptor's `IC` is set.
    #[test]
    fn tx_single_desc_flow() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac, TXPOLLD_OFF, 0x01);
        emac.tick(1); // Queues the descriptor read.
                      // Descriptor: OWN|IC|LS|FS, TBS1=4, buffer @0x20000100.
        let desc = [
            OWN | TDES0_IC | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
            4,
            0x2000_0100,
            0x2000_0010,
        ]
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
            EthMemOp::Write { addr: 0x2000_0000, bytes } if bytes == &word_le(TDES0_IC | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP))));
        // W1C clears.
        write_word(&mut emac, DMARIS_OFF, DMA_TI | DMA_NIS);
        assert_eq!(emac.pending(), 0);
    }

    /// Table 20-2: `TI` stays clear when the closing descriptor's `IC` is clear.
    #[test]
    fn tx_ti_gated_by_ic() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac, TXPOLLD_OFF, 0x01);
        emac.tick(1);
        // Same shape as above but without `IC`.
        let desc = [
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
            4,
            0x2000_0100,
            0x2000_0010,
        ]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect::<Vec<_>>();
        seed_result(&mb, desc);
        emac.tick(1);
        seed_result(&mb, vec![0xDE, 0xAD, 0xBE, 0xEF]);
        emac.tick(1);
        assert_eq!(emac.drain_frames().len(), 1);
        assert_eq!(
            read_word(&mut emac, DMARIS_OFF) & DMA_TI,
            0,
            "TI gated by IC"
        );
        assert_eq!(emac.pending(), 0);
    }

    /// TX with `OWN` clear: spurious poll, nothing moves.
    #[test]
    fn tx_spurious_poll() {
        let (mut emac, mb) = new_emac();
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
        let (mut emac, mb) = new_emac();
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
        let (mut emac, mb) = new_emac();
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
        let (mut emac, mb) = new_emac();
        let iph = unhex("4500 003c 1c46 4000 4006 0000 ac10 0a63 ac10 0a0c");
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (1 << TDES0_CIC_SHIFT),
            &eth_frame(&iph),
        );
        assert_eq!(&out[14 + 10..14 + 12], &[0xb1, 0xe6]);
        assert_eq!(tx_status_wb(&mb) & (TDES0_IHE | TDES0_ES), 0);
    }

    /// TX `CIC=3` inserts IPv4 + full TCP checksum with pseudo-header
    /// (`9d36`/`eead` vectors), status clean.
    #[test]
    fn tx_cic3_tcp_full() {
        let (mut emac, mb) = new_emac();
        let iph = unhex("4500 0028 1c46 4000 4006 0000 c0a8 0001 c0a8 0002");
        let tcp = unhex("0050 1f90 0000 0001 0000 0000 5002 2000 0000 0000");
        let mut payload = iph;
        payload.extend_from_slice(&tcp);
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (3 << TDES0_CIC_SHIFT),
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
        let (mut emac, mb) = new_emac();
        let iph = unhex("4500 0028 1c46 4000 4006 0000 ac10 0a63 ac10 0a0c");
        let tcp = unhex("0050 1f90 0000 0001 0000 0000 5002 2000 6caa 0000");
        let mut payload = iph;
        payload.extend_from_slice(&tcp);
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (2 << TDES0_CIC_SHIFT),
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
        let (mut emac, mb) = new_emac();
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
        let (mut emac, mb) = new_emac();
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

    /// Corrupted header checksum: `(FT=1, both errors)` + `ES`.
    /// Table 20-9 has no header-only row: the header error poisons the
    /// payload verdict, so the nearest real row (both errors) emits.
    #[test]
    fn rx_ipc_bad_header() {
        let (mut emac, mb) = new_emac();
        let mut payload = unhex("4500 0021 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c");
        payload.extend_from_slice(&unhex("04d2 162e 000d 3472 6865 6c6c 6f"));
        let st = rx_status(&mut emac, &mb, CFG_IPC, &eth_frame(&payload));
        assert_eq!(
            st & (RDES0_FT | RDES0_IPCERR | RDES0_PCSERR | RDES0_ES),
            RDES0_FT | RDES0_IPCERR | RDES0_PCSERR | RDES0_ES
        );
    }

    /// Table 20-9: header-error-only never emits the reserved
    /// `(FT=1,HERR=1,PERR=0)` — it maps to both-errors.
    #[test]
    fn rx_no_phantom_row() {
        let mut payload = unhex("4500 0021 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c");
        payload.extend_from_slice(&unhex("04d2 162e 000d 3472 6865 6c6c 6f"));
        assert_eq!(
            rx_checksum_status(&eth_frame(&payload)),
            Some((true, true, true)),
            "header error forces payload error"
        );
    }

    /// `IPC` clear: no checksum bits at all.
    #[test]
    fn rx_ipc_disabled_is_quiet() {
        let (mut emac, mb) = new_emac();
        let mut payload = unhex("4500 0021 1c46 4000 4011 b1f6 ac10 0a63 ac10 0a0c");
        payload.extend_from_slice(&unhex("04d2 162e 000d 3472 6865 6c6c 6f"));
        let st = rx_status(&mut emac, &mb, 0, &eth_frame(&payload));
        assert_eq!(st & (RDES0_FT | RDES0_IPCERR | RDES0_PCSERR | RDES0_ES), 0);
    }

    /// Non-IP EtherType: `FT` set, engine bypasses.
    #[test]
    fn rx_non_ip_frame() {
        let (mut emac, mb) = new_emac();
        let mut frame = vec![
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xA1, 0x88, 0xB5,
        ];
        frame.resize(60, 0);
        let st = rx_status(&mut emac, &mb, CFG_IPC, &frame);
        assert_eq!(st & RDES0_FT, RDES0_FT);
        assert_eq!(st & (RDES0_IPCERR | RDES0_PCSERR | RDES0_ES), 0);
    }

    /// Table 20-10: `DIC` suppresses `RI` (frame still lands, status still
    /// written, but no interrupt).
    #[test]
    fn rx_dic_suppresses_ri() {
        let (mut emac, mb) = new_emac();
        let st = rx_status_with_r1(&mut emac, &mb, 0, RDES1_DIC | 1536, &[0x01, 0x02, 0x03]);
        assert_eq!(
            st & (RDES0_FS | RDES0_LS),
            RDES0_FS | RDES0_LS,
            "status still written"
        );
        assert_eq!(
            read_word(&mut emac, DMARIS_OFF) & DMA_RI,
            0,
            "DIC suppresses RI"
        );
        assert_eq!(emac.pending(), 0);
        // Without `DIC` the same frame interrupts.
        let (mut emac2, mb2) = new_emac();
        let st = rx_status_with_r1(&mut emac2, &mb2, 0, 1536, &[0x01, 0x02, 0x03]);
        assert_eq!(st & (RDES0_FS | RDES0_LS), RDES0_FS | RDES0_LS);
        assert_ne!(read_word(&mut emac2, DMARIS_OFF) & DMA_RI, 0);
    }

    /// Table 20-8: truncation sets `DE`+`ES` and `FL` carries the truncated
    /// length (valid only when `DE`/`OE` are clear — `OE` stays clear in
    /// store-forward).
    #[test]
    fn rx_truncation_sets_de() {
        let (mut emac, mb) = new_emac();
        // 10-byte frame into a 4-byte buffer.
        let frame: Vec<u8> = (0..10).collect();
        let st = rx_status_with_r1(&mut emac, &mb, 0, 4, &frame);
        assert_eq!(st & RDES0_DE, RDES0_DE, "truncation flags DE");
        assert_eq!(st & RDES0_ES, RDES0_ES, "DE ORs into ES");
        assert_eq!(st & RDES0_OE, 0, "no overflow in store-forward");
        assert_eq!((st >> 16) & 0x3FFF, 4, "FL is the truncated length");
        // Only the truncated bytes land in the buffer.
        let ops = &mb.lock().unwrap().ops;
        assert!(ops.iter().any(|op| matches!(op,
            EthMemOp::Write { addr: 0x2000_0200, bytes } if bytes == &vec![0, 1, 2, 3])));
        // A fitting frame stays clean with the full length.
        let (mut emac2, mb2) = new_emac();
        let st = rx_status_with_r1(&mut emac2, &mb2, 0, 1536, &[0x01, 0x02, 0x03]);
        assert_eq!(st & (RDES0_DE | RDES0_ES), 0);
        assert_eq!((st >> 16) & 0x3FFF, 3);
    }

    /// Table 20-2 `IPE`: `CIC>=2` with an IP-declared payload length that
    /// disagrees with the actual bytes sets `IPE`+`ES`.
    #[test]
    fn tx_ipe_on_length_mismatch() {
        let (mut emac, mb) = new_emac();
        // IPv4/TCP claiming 40 payload bytes but carrying 20: `IPE`.
        // (`DC|DP` keep the frame unpadded so the length check sees the
        // raw 54-byte frame.)
        let mut payload = unhex("4500 003c 1c46 4000 4006 0000 ac10 0a63 ac10 0a0c");
        payload.extend_from_slice(&unhex("0050 1f90 0000 0001 0000 0000 5002 2000 0000 0000"));
        // Trim to 20 bytes of TCP instead of 40.
        payload.truncate(20 + 20);
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (3 << TDES0_CIC_SHIFT),
            &eth_frame(&payload),
        );
        assert_eq!(out.len(), 54, "DC|DP leave the short frame as-is");
        assert_eq!(
            tx_status_wb(&mb) & (TDES0_IPE | TDES0_ES),
            TDES0_IPE | TDES0_ES,
            "payload mismatch flags IPE+ES"
        );
        // A consistent frame stays clean.
        let (mut emac2, mb2) = new_emac();
        let iph = unhex("4500 0028 1c46 4000 4006 0000 c0a8 0001 c0a8 0002");
        let tcp = unhex("0050 1f90 0000 0001 0000 0000 5002 2000 0000 0000");
        let mut good = iph;
        good.extend_from_slice(&tcp);
        tx_frame(
            &mut emac2,
            &mb2,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (3 << TDES0_CIC_SHIFT),
            &eth_frame(&good),
        );
        assert_eq!(tx_status_wb(&mb2) & (TDES0_IPE | TDES0_ES), 0);
    }

    /// Table 20-3 `SAIC`: insert adds `ADDR0` after `DA`, replace swaps
    /// `SA` in place; the FCS path covers the edit.
    #[test]
    fn tx_saic_insert_replace() {
        // Station address `02:00:5E:AA:BB:CC` in wire order.
        let mac = [0x02, 0x00, 0x5E, 0xAA, 0xBB, 0xCC];
        // Insert: frame arrives as DA + payload (no SA).
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, ADDR0L_OFF, 0xAA5E_0002);
        write_word(&mut emac, ADDR0H_OFF, 0xCCBB);
        let mut no_sa = vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        no_sa.extend_from_slice(&[0x08, 0x00, 0x45, 0x00]);
        emac.tick(0);
        let _ = &mac;
        write_word(&mut emac, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac, TXPOLLD_OFF, 0x01);
        emac.tick(1);
        // TDES1 carries `SAIC=1` (insert) in bits 31:29.
        let desc = [
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
            (1 << TDES1_SAIC_SHIFT) | (no_sa.len() as u32),
            0x2000_0100,
            0x2000_0010,
        ]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect::<Vec<_>>();
        seed_result(&mb, desc);
        emac.tick(1);
        seed_result(&mb, no_sa.clone());
        emac.tick(1);
        let frames = emac.drain_frames();
        assert_eq!(frames.len(), 1);
        assert_eq!(&frames[0][..6], &[0xFF; 6], "DA preserved");
        assert_eq!(&frames[0][6..12], &mac, "SA inserted");
        // Replace: frame arrives with a placeholder SA.
        let (mut emac2, mb2) = new_emac();
        write_word(&mut emac2, ADDR0L_OFF, 0xAA5E_0002);
        write_word(&mut emac2, ADDR0H_OFF, 0xCCBB);
        let mut with_sa = vec![0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        with_sa.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        with_sa.extend_from_slice(&[0x08, 0x00, 0x45, 0x00]);
        write_word(&mut emac2, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac2, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac2, TXPOLLD_OFF, 0x01);
        emac2.tick(1);
        let desc = [
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
            (2 << TDES1_SAIC_SHIFT) | (with_sa.len() as u32),
            0x2000_0100,
            0x2000_0010,
        ]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect::<Vec<_>>();
        seed_result(&mb2, desc);
        emac2.tick(1);
        seed_result(&mb2, with_sa);
        emac2.tick(1);
        let frames = emac2.drain_frames();
        assert_eq!(frames.len(), 1);
        assert_eq!(&frames[0][6..12], &mac, "SA replaced");
    }

    /// Table 20-3 `SAIC` with bit 31 set uses `ADDR1` instead of `ADDR0`.
    #[test]
    fn tx_saic_selects_addr1() {
        let mac1 = [0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F];
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, ADDR0L_OFF, 0xAA5E_0002);
        write_word(&mut emac, ADDR0H_OFF, 0xCCBB);
        write_word(&mut emac, ADDR1L_OFF, 0x0D0C_0B0A);
        write_word(&mut emac, ADDR1H_OFF, 0x0F0E);
        let mut with_sa = vec![0x00, 0x11, 0x22, 0x33, 0x44, 0x55];
        with_sa.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        with_sa.extend_from_slice(&[0x08, 0x00]);
        write_word(&mut emac, DMAOPMODE_OFF, OP_ST);
        write_word(&mut emac, TXDLADDR_OFF, 0x2000_0000);
        write_word(&mut emac, TXPOLLD_OFF, 0x01);
        emac.tick(1);
        // `SAIC = 0b110`: bit 31 set (ADDR1) + mode 2 (replace).
        let desc = [
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
            (6 << TDES1_SAIC_SHIFT) | (with_sa.len() as u32),
            0x2000_0100,
            0x2000_0010,
        ]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect::<Vec<_>>();
        seed_result(&mb, desc);
        emac.tick(1);
        seed_result(&mb, with_sa);
        emac.tick(1);
        let frames = emac.drain_frames();
        assert_eq!(&frames[0][6..12], &mac1, "ADDR1 selected");
    }

    /// MMC frame/octet counters move with traffic.
    #[test]
    fn mmc_counts_traffic() {
        let (mut emac, mb) = new_emac();
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
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
        let (mut emac, mb) = new_emac();
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
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
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
            &[0x01, 0x02],
        );
        assert_eq!(read_word(&mut emac, 0x118), 0, "frozen");
        write_word(&mut emac, MMCCTRL_OFF, 0);
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
            &[0x01, 0x02],
        );
        assert_eq!(read_word(&mut emac, 0x118), 1, "resumed");
        // Reset-on-read zeroes the counter just read.
        write_word(&mut emac, MMCCTRL_OFF, MMC_RSTONRD);
        assert_eq!(read_word(&mut emac, 0x118), 1, "value first");
        assert_eq!(read_word(&mut emac, 0x118), 0, "then zero");
    }

    /// Half/full interrupts, counter-read clear, masks, `EMACRIS`.
    #[test]
    fn mmc_interrupts_and_masks() {
        let (mut emac, mb) = new_emac();
        // Preset TXCNTGB almost-full, then one frame crosses max.
        write_word(&mut emac, MMCCTRL_OFF, MMC_CNTPRST | MMC_CNTPRSTLVL);
        assert_eq!(read_word(&mut emac, MMCTXRIS_OFF), 0);
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
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
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
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
        let (mut emac, mb) = new_emac();
        let mut bad = vec![0x60, 0x00, 0x00, 0x00];
        bad.resize(20, 0);
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (1 << TDES0_CIC_SHIFT),
            &eth_frame(&bad),
        );
        assert_eq!(
            tx_status_wb(&mb) & (TDES0_IHE | TDES0_ES),
            TDES0_IHE | TDES0_ES
        );
    }

    /// Scan the mailbox for the last 4-byte write to `addr`.
    fn desc_word(mb: &Arc<Mutex<EthMailbox>>, addr: u32) -> Option<u32> {
        let ops = &mb.lock().unwrap().ops;
        for op in ops.iter().rev() {
            if let EthMemOp::Write { addr: a, bytes } = op {
                if *a == addr && bytes.len() == 4 {
                    return Some(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]));
                }
            }
        }
        None
    }

    /// Frozen oracle vectors (`/tmp/opencode/emac_vecs.json`): IEEE
    /// CRC-32 check `cbf43926`, 47 B UDP frame append `ad14391b`,
    /// 43 B prefix replace `0671fdb2`.
    const UDP47: &str = "0011 2233 4455 6677 8899 00A1 0800 4500 0021 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c 04d2 162e 000d 0000 6865 6c6c 6f";
    /// 64 B sibling (`/tmp/opencode/emac_vecs.json`): append `fb445ca4`,
    /// replace-over-prefix `6ec6a5e0`.
    const UDP64: &str = "0011 2233 4455 6677 8899 00A1 0800 4500 0032 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c 04d2 162e 001e 0000 6865 6c6c 6f00 1122 3344 5566 7788 99aa bbcc ddee ff00";
    /// UDP frame with valid checksums (for snapshot-absence negatives).
    const UDPGOOD: &str = "0011 2233 4455 6677 8899 00A1 0800 4500 0021 1c46 4000 4011 b1f6 ac10 0a63 ac10 0a0c 04d2 162e 000d 3472 6865 6c6c 6f";

    #[test]
    fn crc32_unit_vectors() {
        assert_eq!(crc32_ieee(b"123456789"), 0xcbf4_3926);
        assert_eq!(crc32_ieee(&unhex(UDP47)), 0xad14_391b);
        let f = unhex(UDP47);
        assert_eq!(crc32_ieee(&f[..43]), 0x0671_fdb2);
    }

    /// TX `DC=0` appends the computed FCS (`ad14391b`, big-endian).
    #[test]
    fn tx_crcr_append() {
        let (mut emac, mb) = new_emac();
        let frame = unhex(UDP64);
        let out = tx_frame(&mut emac, &mb, OWN | TDES0_LS | TDES0_FS, &frame);
        assert_eq!(out.len(), 68);
        assert_eq!(&out[64..68], &[0xfb, 0x44, 0x5c, 0xa4]);
        assert_eq!(tx_status_wb(&mb) & (TDES0_IHE | TDES0_ES), 0);
    }

    /// TX `DC=1` + `CRCR` replaces the trailing four bytes (`0671fdb2`).
    #[test]
    fn tx_crcr_replace() {
        let (mut emac, mb) = new_emac();
        let frame = unhex(UDP64);
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | TDES0_CRCR,
            &frame,
        );
        assert_eq!(out.len(), 64);
        assert_eq!(&out[60..64], &[0x6e, 0xc6, 0xa5, 0xe0]);
    }

    /// TX `DP=0` pads runts to 60 bytes before the FCS decision.
    #[test]
    fn tx_dp_pads_runt() {
        let (mut emac, mb) = new_emac();
        // Pad only (`DC=1`, no replace): 4 bytes -> 60.
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC,
            &[0xDE, 0xAD, 0xBE, 0xEF],
        );
        assert_eq!(out.len(), 60);
        // Pad + append (`DC=0`): 60 + 4 FCS, trailer is the CRC of the
        // padded prefix.
        let (mut emac2, mb2) = new_emac();
        let out = tx_frame(&mut emac2, &mb2, OWN | TDES0_LS | TDES0_FS, &[0x01, 0x02]);
        assert_eq!(out.len(), 64);
        let mut prefix = vec![0x01, 0x02];
        prefix.resize(60, 0);
        assert_eq!(&out[60..64], &crc32_ieee(&prefix).to_be_bytes());
    }

    /// TX `VLIC=2` inserts `EMACVLNINCREP.VLT` after the source address.
    #[test]
    fn tx_vlan_insert() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, VLNINCREP_OFF, 0x0064);
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (2 << TDES0_VLIC_SHIFT),
            &unhex(UDP47),
        );
        let mut expect = unhex("0011 2233 4455 6677 8899 00A1 8100 0064");
        expect.extend_from_slice(&unhex(
            "0800 4500 0021 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c 04d2 162e 000d 0000 6865 6c6c 6f",
        ));
        assert_eq!(out, expect);
        assert_ne!(tx_status_wb(&mb) & TDES0_VF, 0, "VLAN frame status");
    }

    /// TX `VLIC=1` strips a present tag; `VLIC=3` replaces it.
    #[test]
    fn tx_vlan_strip_replace() {
        let tagged = unhex(
            "0011 2233 4455 6677 8899 00A1 8100 000A 0800 4500 0021 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c 04d2 162e 000d 0000 6865 6c6c 6f",
        );
        let (mut emac, mb) = new_emac();
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (1 << TDES0_VLIC_SHIFT),
            &tagged,
        );
        assert_eq!(out, unhex(UDP47));
        assert_eq!(tx_status_wb(&mb) & TDES0_VF, 0, "no tag left");
        let (mut emac2, mb2) = new_emac();
        write_word(&mut emac2, VLNINCREP_OFF, 0x0064);
        let out = tx_frame(
            &mut emac2,
            &mb2,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (3 << TDES0_VLIC_SHIFT),
            &tagged,
        );
        let mut expect = unhex("0011 2233 4455 6677 8899 00A1 8100 0064");
        expect.extend_from_slice(&unhex(
            "0800 4500 0021 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c 04d2 162e 000d 0000 6865 6c6c 6f",
        ));
        assert_eq!(out, expect);
        assert_ne!(tx_status_wb(&mb2) & TDES0_VF, 0);
    }

    /// Register `VLC` wins when `VLP` is set, even against the descriptor.
    #[test]
    fn tx_vlan_register_override() {
        let tagged = unhex(
            "0011 2233 4455 6677 8899 00A1 8100 000A 0800 4500 0021 1c46 4000 4011 0000 ac10 0a63 ac10 0a0c",
        );
        // `VLC=1` (strip) overrides descriptor `VLIC=2` (insert).
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, VLNINCREP_OFF, VLN_VLP | (1 << VLN_VLC_SHIFT));
        let out = tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (2 << TDES0_VLIC_SHIFT),
            &tagged,
        );
        assert_eq!(&out[12..14], &[0x08, 0x00], "tag stripped");
        // `VLC=0` with `VLP` set means no-op despite descriptor `VLIC=2`.
        let (mut emac2, mb2) = new_emac();
        write_word(&mut emac2, VLNINCREP_OFF, VLN_VLP);
        let out = tx_frame(
            &mut emac2,
            &mb2,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | (2 << TDES0_VLIC_SHIFT),
            &unhex(UDP47),
        );
        assert_eq!(out, unhex(UDP47));
    }

    /// RX tags the VLAN status bit on tag presence.
    #[test]
    fn rx_vlan_bit() {
        let (mut emac, mb) = new_emac();
        let tagged = unhex("0011 2233 4455 6677 8899 00A1 8100 000A 0800 4500 0021");
        let st = rx_status(&mut emac, &mb, 0, &tagged);
        assert_ne!(st & RDES0_VLAN, 0);
        let (mut emac2, mb2) = new_emac();
        let st = rx_status(&mut emac2, &mb2, 0, &unhex(UDP47));
        assert_eq!(st & RDES0_VLAN, 0);
    }

    /// PTP `TSINIT`/`TSUPDT` program the time; command bits self-clear.
    #[test]
    fn ptp_init_update() {
        let (mut emac, _) = new_emac();
        write_word(&mut emac, TIMSECU_OFF, 100);
        write_word(&mut emac, TIMNANOU_OFF, 500);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_TSINIT);
        assert_eq!(read_word(&mut emac, TIMSEC_OFF), 100);
        assert_eq!(read_word(&mut emac, TIMNANO_OFF), 500);
        assert_eq!(read_word(&mut emac, TIMSTCTRL_OFF) & TS_TSINIT, 0);
        // Add 2 s + 600 ns.
        write_word(&mut emac, TIMSECU_OFF, 2);
        write_word(&mut emac, TIMNANOU_OFF, 600);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_TSUPDT);
        assert_eq!(read_word(&mut emac, TIMSEC_OFF), 102);
        assert_eq!(read_word(&mut emac, TIMNANO_OFF), 1100);
        // Subtract 3 s (`ADDSUB`): borrows through seconds.
        write_word(&mut emac, TIMSECU_OFF, 3);
        write_word(&mut emac, TIMNANOU_OFF, 0x8000_0000 | 100);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_TSUPDT);
        assert_eq!(read_word(&mut emac, TIMSEC_OFF), 99);
        assert_eq!(read_word(&mut emac, TIMNANO_OFF), 1000);
    }

    /// PTP coarse advance + digital rollover at 1e9 ns.
    #[test]
    fn ptp_coarse_rollover() {
        let (mut emac, _) = new_emac();
        write_word(&mut emac, SUBSECINC_OFF, 40);
        write_word(&mut emac, TIMSECU_OFF, 0);
        write_word(&mut emac, TIMNANOU_OFF, 999_999_990);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_TSINIT);
        emac.tick(1);
        assert_eq!(read_word(&mut emac, TIMSEC_OFF), 1);
        assert_eq!(read_word(&mut emac, TIMNANO_OFF), 30);
    }

    /// PTP binary rollover wraps at 2^31.
    #[test]
    fn ptp_binary_rollover() {
        let (mut emac, _) = new_emac();
        write_word(&mut emac, SUBSECINC_OFF, 0x20);
        write_word(&mut emac, TIMSECU_OFF, 5);
        write_word(&mut emac, TIMNANOU_OFF, 0x7FFF_FFF0);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_TSINIT);
        emac.tick(1);
        assert_eq!(read_word(&mut emac, TIMSEC_OFF), 6);
        assert_eq!(read_word(&mut emac, TIMNANO_OFF), 0x10);
    }

    /// PTP fine correction: `TSAR=2^31` overflows every second tick.
    #[test]
    fn ptp_fine_advance() {
        let (mut emac, _) = new_emac();
        write_word(&mut emac, SUBSECINC_OFF, 40);
        write_word(&mut emac, TIMADD_OFF, 0x8000_0000);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_TSFCUPDT);
        emac.tick(1);
        assert_eq!(read_word(&mut emac, TIMNANO_OFF), 0);
        emac.tick(1);
        assert_eq!(read_word(&mut emac, TIMNANO_OFF), 40);
    }

    /// PTP target: `INTTRIG` fires `TT` + `EMACRIS.TS` once at the mark.
    #[test]
    fn ptp_target_interrupt() {
        let (mut emac, _) = new_emac();
        write_word(&mut emac, SUBSECINC_OFF, 40);
        write_word(&mut emac, TIMSECU_OFF, 9);
        write_word(&mut emac, TIMNANOU_OFF, 999_999_999);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_TSINIT);
        write_word(&mut emac, TARGSEC_OFF, 10);
        write_word(&mut emac, TARGNANO_OFF, 0);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_INTTRIG);
        emac.tick(1);
        assert_eq!(read_word(&mut emac, TIMSTCTRL_OFF) & TS_INTTRIG, 0);
        assert_ne!(read_word(&mut emac, DMARIS_OFF) & DMA_TT, 0);
        assert_ne!(read_word(&mut emac, EMACRIS_OFF) & RIS_TS, 0);
        assert_ne!(read_word(&mut emac, TIMSTAT_OFF) & TIM_TSTARGT, 0);
        assert_ne!(emac.pending(), 0);
        write_word(&mut emac, DMARIS_OFF, DMA_TT);
        assert_eq!(read_word(&mut emac, EMACRIS_OFF) & RIS_TS, 0);
    }

    /// TX `TTSE` captures the time into `TDES6/7` with `TTSS`
    /// (§20.3.2.5: extended words valid only with `ATDS=1`).
    #[test]
    fn tx_ttse_snapshot() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, BUSMOD_OFF, BUSMOD_ATDS);
        write_word(&mut emac, TIMSECU_OFF, 7);
        write_word(&mut emac, TIMNANOU_OFF, 123);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_TSINIT);
        let frame = unhex(UDP47);
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | TDES0_TTSE,
            &frame,
        );
        assert_eq!(desc_word(&mb, 0x2000_0018), Some(123));
        assert_eq!(desc_word(&mb, 0x2000_001C), Some(7));
        assert_ne!(tx_status_wb(&mb) & TDES0_TTSS, 0);
        // Without `TTSE` no timestamp words land.
        let (mut emac2, mb2) = new_emac();
        write_word(&mut emac2, BUSMOD_OFF, BUSMOD_ATDS);
        write_word(&mut emac2, TIMSTCTRL_OFF, TS_TSEN);
        tx_frame(
            &mut emac2,
            &mb2,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP,
            &frame,
        );
        assert_eq!(desc_word(&mb2, 0x2000_0018), None);
        assert_eq!(tx_status_wb(&mb2) & TDES0_TTSS, 0);
    }

    /// §20.3.2.5: with `ATDS=0` the extended words stay untouched even
    /// when `TTSE` asks for a snapshot (no `TTSS`, no `TDES6/7` writes).
    #[test]
    fn tx_ttse_needs_atds() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, TIMSECU_OFF, 7);
        write_word(&mut emac, TIMNANOU_OFF, 123);
        write_word(&mut emac, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_TSINIT);
        // `BUSMOD` reset is `ATDS=0`: no snapshot.
        let frame = unhex(UDP47);
        tx_frame(
            &mut emac,
            &mb,
            OWN | TDES0_LS | TDES0_FS | TDES0_DC | TDES0_DP | TDES0_TTSE,
            &frame,
        );
        assert_eq!(desc_word(&mb, 0x2000_0018), None);
        assert_eq!(desc_word(&mb, 0x2000_001C), None);
        assert_eq!(tx_status_wb(&mb) & TDES0_TTSS, 0);
    }

    /// RX `ALLF` snapshots every frame with the bypassed-checksum mark
    /// (`ATDS=1` required for the extended words).
    #[test]
    fn rx_allf_snapshot() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, BUSMOD_OFF, BUSMOD_ATDS);
        write_word(&mut emac, TIMSECU_OFF, 3);
        write_word(&mut emac, TIMNANOU_OFF, 9);
        write_word(
            &mut emac,
            TIMSTCTRL_OFF,
            TS_TSEN | TS_DGTLBIN | TS_TSINIT | TS_ALLF,
        );
        let st = rx_status(&mut emac, &mb, 0, &[0x01, 0x02, 0x03]);
        assert_ne!(st & RDES0_IPCERR, 0, "timestamp-available bit");
        assert_eq!(desc_word(&mb, 0x2000_1018), Some(9));
        assert_eq!(desc_word(&mb, 0x2000_101C), Some(3));
        assert_eq!(desc_word(&mb, 0x2000_1010), Some(RDES4_BYP));
    }

    /// §20.3.2.5: with `ATDS=0` no RX extended words land and the
    /// timestamp-available bit stays clear.
    #[test]
    fn rx_snapshot_needs_atds() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, TIMSECU_OFF, 3);
        write_word(&mut emac, TIMNANOU_OFF, 9);
        write_word(
            &mut emac,
            TIMSTCTRL_OFF,
            TS_TSEN | TS_DGTLBIN | TS_TSINIT | TS_ALLF,
        );
        let st = rx_status(&mut emac, &mb, 0, &[0x01, 0x02, 0x03]);
        assert_eq!(st & RDES0_IPCERR, 0, "no timestamp without ATDS");
        assert_eq!(desc_word(&mb, 0x2000_1018), None);
        assert_eq!(desc_word(&mb, 0x2000_101C), None);
        assert_eq!(desc_word(&mb, 0x2000_1010), None);
    }

    /// PTP Sync v2 over IPv4/UDP (frozen `ptp_sync_hex` shape): selective
    /// snapshot fills `RDES4` (version + IPv4 + `Delay_Req`-class message
    /// fields); the zeroed IP checksum mirrors into `HERR`.
    fn ptp_sync_frame() -> Vec<u8> {
        let mut f = vec![
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xA1, 0x08, 0x00,
        ];
        f.extend_from_slice(&unhex("4500 0048 0000 4000 4011 0000 c0a8 0001 e000 0000"));
        f.extend_from_slice(&unhex("013F 013F 0034 0000"));
        f.extend_from_slice(&[0x01, 0x02]);
        f.extend_from_slice(&[0u8; 42]);
        f
    }

    #[test]
    fn rx_ptp_selective_snapshot() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, BUSMOD_OFF, BUSMOD_ATDS);
        write_word(
            &mut emac,
            TIMSTCTRL_OFF,
            TS_TSEN | TS_DGTLBIN | TS_PTPVER2 | TS_PTPIPV4,
        );
        let st = rx_status(&mut emac, &mb, CFG_IPC, &ptp_sync_frame());
        assert_ne!(st & RDES0_IPCERR, 0, "timestamp-available");
        // `VER` + IPv4 + message `0x1` + header-error mirror. The header
        // error poisons the payload verdict (Table 20-9 has no
        // `(FT=1,HERR=1,PERR=0)` row), so `PERR` rides along.
        assert_eq!(
            desc_word(&mb, 0x2000_1010),
            Some(RDES4_VER | RDES4_IPV4 | (0x1 << RDES4_MSG_SHIFT) | RDES4_HERR | RDES4_PERR)
        );
        // A non-PTP frame takes no snapshot.
        let (mut emac2, mb2) = new_emac();
        write_word(&mut emac2, BUSMOD_OFF, BUSMOD_ATDS);
        write_word(
            &mut emac2,
            TIMSTCTRL_OFF,
            TS_TSEN | TS_DGTLBIN | TS_PTPVER2 | TS_PTPIPV4,
        );
        let st = rx_status(&mut emac2, &mb2, CFG_IPC, &unhex(UDPGOOD));
        assert_eq!(st & (RDES0_IPCERR | RDES0_ES), 0);
        assert_eq!(desc_word(&mb2, 0x2000_1018), None);
        // Transport disabled: the same Sync frame is ignored.
        let (mut emac3, mb3) = new_emac();
        write_word(&mut emac3, TIMSTCTRL_OFF, TS_TSEN | TS_DGTLBIN | TS_PTPVER2);
        // Transport disabled: the same Sync frame takes no snapshot
        // (its zeroed IP checksum still flags `HERR`, which is not one).
        rx_status(&mut emac3, &mb3, CFG_IPC, &ptp_sync_frame());
        assert_eq!(desc_word(&mb3, 0x2000_1018), None);
    }

    /// Magic-packet wake: pattern match sets `MGKPRX`, self-clears
    /// `PWRDWN`, raises `DMARIS.PMT`; the register read clears the flags.
    fn pmt_rx_flow(emac: &mut Emac, mb: &Arc<Mutex<EthMailbox>>, frame: &[u8]) {
        write_word(emac, DMAOPMODE_OFF, OP_SR);
        write_word(emac, RXDLADDR_OFF, 0x2000_1000);
        emac.inject_frame(frame);
        emac.tick(1);
        let desc = [OWN, 1536, 0x2000_0200, 0x2000_1010]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect::<Vec<_>>();
        seed_result(mb, desc);
        emac.tick(1);
    }

    fn magic_frame() -> Vec<u8> {
        let mac = [0x02, 0x00, 0x5E, 0xAA, 0xBB, 0xCC];
        let mut f = vec![0xFF; 6];
        f.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        f.extend_from_slice(&[0xFF; 6]);
        for _ in 0..16 {
            f.extend_from_slice(&mac);
        }
        f
    }

    #[test]
    fn magic_packet_wakes() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, ADDR0L_OFF, 0xAA5E_0002);
        write_word(&mut emac, ADDR0H_OFF, 0xCCBB);
        write_word(&mut emac, PMTCTLSTAT_OFF, PMT_PWRDWN | PMT_MGKPKTEN);
        pmt_rx_flow(&mut emac, &mb, &magic_frame());
        assert_ne!(read_word(&mut emac, DMARIS_OFF) & DMA_PMT, 0);
        assert_ne!(read_word(&mut emac, EMACRIS_OFF) & RIS_PMT, 0);
        assert_ne!(emac.pending(), 0);
        assert_eq!(
            read_word(&mut emac, DMARIS_OFF) & DMA_RI,
            0,
            "wake frame not delivered"
        );
        // The first read shows the flag with `PWRDWN` self-cleared;
        // the second read shows the read-clear.
        let pmt = read_word(&mut emac, PMTCTLSTAT_OFF);
        assert_eq!(pmt & PMT_PWRDWN, 0, "wake self-cleared power-down");
        assert_ne!(pmt & PMT_MGKPRX, 0, "magic flag visible once");
        assert_eq!(
            read_word(&mut emac, PMTCTLSTAT_OFF) & (PMT_MGKPRX | PMT_WUPRX),
            0,
            "read-clear"
        );
        assert_eq!(read_word(&mut emac, EMACRIS_OFF) & RIS_PMT, 0);
        write_word(&mut emac, DMARIS_OFF, DMA_PMT);
        assert_eq!(emac.pending(), 0);
    }

    /// A broken repetition sequence is not magic: still powered down.
    #[test]
    fn magic_packet_broken_stays_down() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, ADDR0L_OFF, 0xAA5E_0002);
        write_word(&mut emac, ADDR0H_OFF, 0xCCBB);
        write_word(&mut emac, PMTCTLSTAT_OFF, PMT_PWRDWN | PMT_MGKPKTEN);
        let mut bad = magic_frame();
        bad[30] ^= 0xFF;
        pmt_rx_flow(&mut emac, &mb, &bad);
        assert_ne!(
            read_word(&mut emac, PMTCTLSTAT_OFF) & PMT_PWRDWN,
            0,
            "still powered down"
        );
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & (DMA_PMT | DMA_RI), 0);
        assert_eq!(emac.pending(), 0);
    }

    /// Global-unicast wake fires `WUPRX` on our address.
    #[test]
    fn glblucast_wakes() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, ADDR0L_OFF, 0xAA5E_0002);
        write_word(&mut emac, ADDR0H_OFF, 0xCCBB);
        write_word(&mut emac, PMTCTLSTAT_OFF, PMT_PWRDWN | PMT_GLBLUCAST);
        let mut f = vec![
            0x02, 0x00, 0x5E, 0xAA, 0xBB, 0xCC, 0x66, 0x77, 0x88, 0x99, 0x00, 0xA1,
        ];
        f.extend_from_slice(&[0x08, 0x00, 0x45, 0x00]);
        pmt_rx_flow(&mut emac, &mb, &f);
        assert_ne!(read_word(&mut emac, EMACRIS_OFF) & RIS_PMT, 0);
        let pmt = read_word(&mut emac, PMTCTLSTAT_OFF);
        assert_eq!(pmt & PMT_PWRDWN, 0, "woke");
        assert_ne!(pmt & PMT_WUPRX, 0, "unicast flag visible once");
        assert_eq!(
            read_word(&mut emac, PMTCTLSTAT_OFF) & PMT_WUPRX,
            0,
            "read-clear"
        );
    }

    /// Power-down with no wake enables drops frames silently.
    #[test]
    fn powerdown_drops_quiet() {
        let (mut emac, mb) = new_emac();
        write_word(&mut emac, PMTCTLSTAT_OFF, PMT_PWRDWN);
        pmt_rx_flow(&mut emac, &mb, &[0x01, 0x02, 0x03]);
        assert_eq!(read_word(&mut emac, DMARIS_OFF) & (DMA_PMT | DMA_RI), 0);
        assert_ne!(
            read_word(&mut emac, PMTCTLSTAT_OFF) & PMT_PWRDWN,
            0,
            "still down"
        );
    }

    /// `RWUFF` bank rotates on whole-word access; the pointer reset works.
    #[test]
    fn rwuff_bank_mechanics() {
        let (mut emac, _) = new_emac();
        write_word(&mut emac, RWUFF_OFF, 0x1111_1111);
        write_word(&mut emac, RWUFF_OFF, 0x2222_2222);
        write_word(&mut emac, RWUFF_OFF, 0x3333_3333);
        assert_eq!(
            read_word(&mut emac, PMTCTLSTAT_OFF) >> PMT_RWKPTR_SHIFT & 7,
            3
        );
        write_word(&mut emac, PMTCTLSTAT_OFF, PMT_WUPFRRST);
        assert_eq!(
            read_word(&mut emac, PMTCTLSTAT_OFF) >> PMT_RWKPTR_SHIFT & 7,
            0
        );
        assert_eq!(read_word(&mut emac, RWUFF_OFF), 0x1111_1111);
        assert_eq!(read_word(&mut emac, RWUFF_OFF), 0x2222_2222);
        assert_eq!(read_word(&mut emac, RWUFF_OFF), 0x3333_3333);
    }
}
