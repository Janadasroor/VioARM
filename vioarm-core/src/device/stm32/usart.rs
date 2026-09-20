//! The STM32 USART/UART family (`USART1`/`USART2`/`USART3`, plus the
//! `UART4`/`UART5`/`USART6` instances present on the connectivity line and the
//! F4), driven by the SVD-generated register tables.
//!
//! A byte-oriented model of the real register file: `SR`/`DR`/`BRR`/`CR1..3`
//! with the `TXE`/`TC`/`RXNE` status bits. Transmitted bytes are captured into
//! a host-readable buffer (like the generic `UartTx`); received bytes are
//! injected by the host through the stimulus channel. `RXNE` is set on
//! receive, cleared by reading `DR`, and — together with `CR1.RXNEIE` and
//! `CR1.UE` — drives a level-triggered IRQ that keeps asserting until the
//! handler drains the data register, exactly as on silicon. The two things a
//! table cannot express live in `UsartBehavior`: the `TXE`/`TC` status
//! synthesis and the read-to-clear `RXNE` side effect.
//!
//! With a [`DmaRequests`] mailbox wired in, the USART also feeds the DMA
//! controllers: the F1 routes each instance's transmit line to a dedicated
//! `DMA1` channel and its receive line to another, asserted while `UE`/`TE`
//! are set and while a received byte is pending in `DR`. Reading `DR` (or
//! writing zero to it) drops the RX request, mirroring the level request line
//! that falls when the data register is drained.
//!
//! Receive errors follow silicon: a byte arriving while `RXNE` is still set
//! latches `ORE` and is dropped (the first byte is kept); stimulus bit 9
//! injects a framing error (`FE`), bit 10 a parity error (`PE`). On F1
//! tables the flags clear with the `SR`-then-`DR` read sequence (a bare
//! `DR` read only clears `RXNE`); on new-gen tables they clear by writing
//! 1 to `ICR` (`ORECF`/`FECF`/`PECF`). `ORE`/`FE` assert the IRQ line via
//! `CR3.EIE`, `PE` via `CR1.PEIE`. Deliberately unmodeled: noise errors
//! (`NE`, needs an oversampling model), `OVRDIS` variants (overrun always
//! latches), and `RE` gating — like `UE`-off arrival, injected bytes latch
//! regardless, keeping host injection order-free by design.

use crate::device::stm32::dma::DmaRequests;
use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// Register offsets within the peripheral, per the SVD. F1 uses `SR`/`DR`/`CR1` at
/// `0x00`/`0x04`/`0x0C` with `UE` `13`; F0/F3/L4 use `ISR`/`RDR`/`TDR`/`CR1` at
/// `0x1C`/`0x24`/`0x28`/`0x00` with `UE` `0`. Offsets and bit positions are
/// resolved per-instance from the SVD table.
const SR_F1: u32 = 0x00;
const DR_F1: u32 = 0x04;
const CR1_F1: u32 = 0x0C;

/// `SR` bit positions.
const RXNE: u32 = 1 << 5;
const TC: u32 = 1 << 6;
const TXE: u32 = 1 << 7;
/// Receive error flags, same positions on F1 `SR` and new-gen `ISR`.
const ORE_FALLBACK: u32 = 1 << 3;
const FE_FALLBACK: u32 = 1 << 1;
const PE_FALLBACK: u32 = 1 << 0;
/// `SR.LBD` (F1 bit 8): a LIN break was detected on `RX`.
const LBD_F1: u32 = 1 << 8;
/// `CR2` offset and bits (LIN support).
const CR2_F1: u32 = 0x10;
/// `CR2.LBDIE` (bit 6): the LIN break detection interrupt enable.
const LBDIE_F1: u32 = 1 << 6;

/// `CR1` bit positions.
const UE: u32 = 1 << 13;
const TE: u32 = 1 << 3;
const RXNEIE: u32 = 1 << 5;
/// `CR1.PEIE` (both generations, bit 8): parity-error interrupt enable.
const PEIE_FALLBACK: u32 = 1 << 8;
/// `CR3.EIE` (both generations, bit 0): framing/overrun interrupt enable.
const EIE_FALLBACK: u32 = 1 << 0;

/// The F1 USART→DMA1 request routing, per the reference manual. USART1 TX
/// lives on channel 4, USART2 RX on channel 6, and so on; the connectivity
/// line's `UART4`/`UART5` share channels with `USART1`/`USART2`.
const DMA1_CONTROLLER: u8 = 0;
const USART1_TX_CHANNEL: u8 = 4;
const USART1_RX_CHANNEL: u8 = 5;
const USART2_TX_CHANNEL: u8 = 7;
const USART2_RX_CHANNEL: u8 = 6;
const USART3_TX_CHANNEL: u8 = 2;
const USART3_RX_CHANNEL: u8 = 3;
const UART4_TX_CHANNEL: u8 = 5;
const UART4_RX_CHANNEL: u8 = 6;
const UART5_TX_CHANNEL: u8 = 7;
const UART5_RX_CHANNEL: u8 = 4;

/// The cross-register side effects the SVD cannot express: status synthesis
/// on `SR` reads, `RXNE` acknowledge on `DR` reads and zero writes, and the
/// DMA request lines asserted while data is ready to move.
#[derive(Debug)]
struct UsartBehavior {
    /// Bytes firmware wrote to the data register, for host draining.
    output: Vec<u8>,
    /// The DMA request mailbox, when a DMA controller is wired to this USART.
    requests: Option<Arc<Mutex<DmaRequests>>>,
    /// The request routing `(controller, tx_channel, rx_channel)`.
    dma: Option<(u8, u8, u8)>,
    cr1_off: u32,
    sr_off: u32,
    rdr_off: u32,
    tdr_off: u32,
    cr2_off: Option<u32>,
    isr_off: Option<u32>,
    ue_bit: u32,
    te_bit: u32,
    re_bit: u32,
    rxneie_bit: u32,
    rxne_bit: u32,
    txe_bit: u32,
    tc_bit: u32,
    /// Receive error flag positions (`ORE`/`FE`/`PE`), resolved from the
    /// status register fields.
    ore_bit: u32,
    fe_bit: u32,
    pe_bit: u32,
    /// `CR1.PEIE`: parity-error interrupt enable.
    peie_bit: u32,
    /// `CR3` offset (`None` when the table has no `CR3`: no `EIE` path).
    cr3_off: Option<u32>,
    /// `CR3.EIE`: framing/overrun-error interrupt enable.
    eie_bit: u32,
    /// New-gen `ICR` offset (`None` on F1 tables, which clear errors with
    /// the `SR`-then-`DR` read sequence instead).
    icr_off: Option<u32>,
    /// New-gen `ICR` write-1-to-clear bits.
    icr_orecf: u32,
    icr_fecf: u32,
    icr_pecf: u32,
    /// F1 clear sequence arm: set by an `SR` read, consumed by the next
    /// data-register access. Error flags only clear inside the sequence.
    sr_armed: bool,
    clken_bit: Option<u32>,
    abren_bit: Option<u32>,
    abrf_bit: Option<u32>,
}

impl Default for UsartBehavior {
    fn default() -> Self {
        Self {
            output: Vec::new(),
            requests: None,
            dma: None,
            cr1_off: CR1_F1,
            sr_off: SR_F1,
            rdr_off: DR_F1,
            tdr_off: DR_F1,
            cr2_off: Some(CR2_F1),
            isr_off: None,
            ue_bit: 1 << 13,
            te_bit: 1 << 3,
            re_bit: 1 << 2,
            rxneie_bit: 1 << 5,
            rxne_bit: 1 << 5,
            txe_bit: 1 << 7,
            tc_bit: 1 << 6,
            clken_bit: None,
            abren_bit: None,
            abrf_bit: None,
            ore_bit: ORE_FALLBACK,
            fe_bit: FE_FALLBACK,
            pe_bit: PE_FALLBACK,
            peie_bit: PEIE_FALLBACK,
            cr3_off: None,
            eie_bit: 0,
            icr_off: None,
            icr_orecf: ORE_FALLBACK,
            icr_fecf: FE_FALLBACK,
            icr_pecf: PE_FALLBACK,
            sr_armed: false,
        }
    }
}

impl Behavior for UsartBehavior {
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u32, value: u32) -> u32 {
        if offset == self.sr_off {
            // `TXE`/`TC` track the enable bits, the rest is latched status.
            let mut v = value & !(self.txe_bit | self.tc_bit);
            if let Some(cr1) = view.try_get(self.cr1_off) {
                if cr1 & self.ue_bit != 0 {
                    v |= self.tc_bit;
                    if cr1 & self.te_bit != 0 {
                        v |= self.txe_bit;
                    }
                }
            }
            // Reading the status register arms the F1 error-clear sequence:
            // the following data-register access clears `ORE`/`FE`/`PE`.
            self.sr_armed = true;
            return v;
        }
        if offset == self.rdr_off {
            // Reading the data register drains the received byte and
            // acknowledges `RXNE`, dropping the DMA RX request line. Error
            // flags clear only inside the `SR`-then-`DR` sequence — and only
            // on F1 tables; new-gen tables need `ICR`.
            if let Some(sr) = view.try_get(self.sr_off) {
                let mut sr = sr & !self.rxne_bit;
                if self.sr_armed && self.icr_off.is_none() {
                    sr &= !(self.ore_bit | self.fe_bit | self.pe_bit);
                }
                view.try_set(self.sr_off, sr);
            }
            self.sr_armed = false;
            self.drop_rx_request();
            return value;
        }
        value
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u32,
        val: u8,
        shift: u32,
    ) -> u32 {
        // Auto-baud flag ABRF (ISR) is write-0-to-clear on new-gen tables.
        if Some(offset) == self.isr_off {
            if let Some(abrf) = self.abrf_bit {
                let cur = view.get(offset);
                let word = (cur & !(0xFF << shift)) | (u32::from(val) << shift);
                if word & abrf == 0 && cur & abrf != 0 {
                    let cleared = cur & !abrf;
                    view.set(offset, cleared);
                    return cleared;
                }
            }
        }
        if offset == self.tdr_off {
            // Byte 0 of the data register is the transmitted character;
            // the higher lanes are the extension of the same store on
            // silicon, so only it queues a byte for output.
            if shift == 0 {
                if view.try_get(self.cr1_off).unwrap_or(0) & self.te_bit != 0 {
                    self.output.push(val);
                }
                // A zero write to the data register acknowledges `RXNE`,
                // matching the silicon rule that zero is not data.
                if val == 0 {
                    if let Some(sr) = view.try_get(self.sr_off) {
                        view.try_set(self.sr_off, sr & !self.rxne_bit);
                    }
                    self.drop_rx_request();
                }
                // A data-register write completes the F1 error-clear
                // sequence the same way a read does.
                if self.sr_armed {
                    if let Some(sr) = view.try_get(self.sr_off) {
                        view.try_set(
                            self.sr_off,
                            sr & !(self.ore_bit | self.fe_bit | self.pe_bit),
                        );
                    }
                    self.sr_armed = false;
                }
            }
            // A transmit does not alter the received data.
            return view.try_get(self.tdr_off).unwrap_or(0);
        }
        // New-gen interrupt-clear register: writing 1 to `ORECF`/`FECF`/
        // `PECF` clears the matching `ISR` error flag.
        if Some(offset) == self.icr_off {
            if let Some(sr) = view.try_get(self.sr_off) {
                let written = u32::from(val);
                let mut sr = sr;
                if written & self.icr_orecf != 0 {
                    sr &= !self.ore_bit;
                }
                if written & self.icr_fecf != 0 {
                    sr &= !self.fe_bit;
                }
                if written & self.icr_pecf != 0 {
                    sr &= !self.pe_bit;
                }
                view.try_set(self.sr_off, sr);
            }
            return view.merged(offset, val, shift);
        }
        // For old single-DR (rdr==tdr), also handle DR at same offset for TX
        if self.rdr_off == self.tdr_off && offset == self.rdr_off {
            if shift == 0 {
                if view.try_get(self.cr1_off).unwrap_or(0) & self.te_bit != 0 {
                    self.output.push(val);
                }
                if val == 0 {
                    if let Some(sr) = view.try_get(self.sr_off) {
                        view.try_set(self.sr_off, sr & !self.rxne_bit);
                    }
                    self.drop_rx_request();
                }
                // Like the `TDR` branch above, a write completes the F1
                // sequence (never on new-gen tables with an `ICR`).
                if self.sr_armed && self.icr_off.is_none() {
                    if let Some(sr) = view.try_get(self.sr_off) {
                        view.try_set(
                            self.sr_off,
                            sr & !(self.ore_bit | self.fe_bit | self.pe_bit),
                        );
                    }
                    self.sr_armed = false;
                }
            }
            return view.try_get(self.rdr_off).unwrap_or(0);
        }
        view.merged(offset, val, shift)
    }

    fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        let (requests, dma) = (&self.requests, self.dma);
        let (Some(requests), Some((controller, tx_channel, rx_channel))) = (requests, dma) else {
            return;
        };
        let mut req = requests.lock().unwrap_or_else(|e| e.into_inner());
        let cr1 = view.try_get(self.cr1_off).unwrap_or(0);
        // The TX line is high whenever the transmitter can take a byte.
        if cr1 & (self.ue_bit | self.te_bit) == (self.ue_bit | self.te_bit) {
            req.request(controller, tx_channel, true);
        }
        // The RX line is high while a received byte sits unread in `DR`.
        if view.try_get(self.sr_off).unwrap_or(0) & self.rxne_bit != 0 {
            req.request(controller, rx_channel, false);
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        // Level-triggered: asserts while a received byte is both unread and
        // unmasked. The handler clears it by reading `DR`. A LIN break
        // (`SR.LBD` with `CR2.LBDIE`) asserts the same line until cleared.
        // Receive errors assert it too when unmasked: `ORE`/`FE` via
        // `CR3.EIE`, `PE` via `CR1.PEIE`.
        let sr = view.try_get(self.sr_off).unwrap_or(0);
        let cr1 = view.try_get(self.cr1_off).unwrap_or(0);
        let ue = cr1 & self.ue_bit != 0;
        let rx = sr & self.rxne_bit != 0
            && cr1 & (self.rxneie_bit | self.ue_bit) == (self.rxneie_bit | self.ue_bit);
        let eie = self
            .cr3_off
            .is_some_and(|off| view.try_get(off).unwrap_or(0) & self.eie_bit != 0);
        let err = ue
            && ((sr & (self.ore_bit | self.fe_bit) != 0 && eie)
                || (sr & self.pe_bit != 0 && cr1 & self.peie_bit != 0));
        let lbd = view.try_get(CR2_F1).is_some_and(|cr2| cr2 & LBDIE_F1 != 0) && sr & LBD_F1 != 0;
        u32::from(rx || lbd || err)
    }

    fn reset(&mut self) {
        self.output.clear();
        self.sr_armed = false;
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        // The low byte is one received character. A byte arriving while the
        // previous one is still unread overruns: silicon latches `ORE` and
        // drops the newcomer, keeping the first byte.
        if let Some(sr) = view.try_get(self.sr_off) {
            if sr & self.rxne_bit != 0 {
                view.try_set(self.sr_off, sr | self.ore_bit);
            } else {
                view.try_set(self.rdr_off, value & 0xFF);
                let mut sr = sr | self.rxne_bit;
                // Stimulus error injection: bit 9 frames badly, bit 10
                // carries a parity error (bit 8 stays the LIN break).
                if value & (1 << 9) != 0 {
                    sr |= self.fe_bit;
                }
                if value & (1 << 10) != 0 {
                    sr |= self.pe_bit;
                }
                view.try_set(self.sr_off, sr);
            }
        }
        // Auto-baud detection: if ABREN is set in CR2, the next received
        // byte completes the measurement — clear ABREN and latch ABRF in ISR.
        if let (Some(cr2_off), Some(abren), Some(abrf)) =
            (self.cr2_off, self.abren_bit, self.abrf_bit)
        {
            if let Some(cr2) = view.try_get(cr2_off) {
                if cr2 & abren != 0 {
                    view.try_set(cr2_off, cr2 & !abren);
                    if let Some(sr) = view.try_get(self.sr_off) {
                        view.try_set(self.sr_off, sr | abrf);
                    }
                }
            }
        }
        // Bit 8 set signals a LIN break (RX held low for a full frame):
        // latch `SR.LBD` on F1-style tables that declare it.
        if value & (1 << 8) != 0 {
            if let Some(cr2) = view.try_get(CR2_F1) {
                if cr2 & LBDIE_F1 != 0 {
                    if let Some(sr) = view.try_get(self.sr_off) {
                        view.try_set(self.sr_off, sr | LBD_F1);
                    }
                }
            }
        }
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.output)
    }
}

impl UsartBehavior {
    /// Deassert the RX DMA request line (the data register was drained).
    fn drop_rx_request(&mut self) {
        if let (Some(requests), Some((controller, _, rx_channel))) = (&self.requests, self.dma) {
            requests
                .lock()
                .unwrap()
                .release(controller, rx_channel, false);
        }
    }
}

/// A USART/UART register file, table-driven from the SVD.
///
/// The instance (`USART1`, `USART2`, ...), its base address, and its IRQ
/// number all come from `D`'s generated tables; only the host-visible name and
/// the F1 DMA routing differ between instances.
#[derive(Debug)]
pub struct Usart {
    inner: RegisterFile<UsartBehavior>,
    name: &'static str,
}

impl Usart {
    /// A new F103 `USART1` at `0x4001_3800`, IRQ 37.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `USART1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        Self::usart1()
    }

    /// A new F103 `USART1` at `0x4001_3800`, IRQ 37.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `USART1`
    /// peripheral.
    #[must_use]
    pub fn usart1() -> Self {
        Self::for_device_with::<stm32f103::Stm32f103>("USART1", "usart1", None, None)
    }

    /// A new `USART1` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART1`
    /// peripheral.
    #[must_use]
    pub fn usart1_for<D: Device>() -> Self {
        Self::for_device_with::<D>("USART1", "usart1", None, None)
    }

    /// A new `USART1` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART1`
    /// peripheral.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        Self::usart1_for::<D>()
    }

    /// A new F103 `USART2` at `0x4000_4400`, IRQ 38.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `USART2`
    /// peripheral.
    #[must_use]
    pub fn usart2() -> Self {
        Self::for_device_with::<stm32f103::Stm32f103>("USART2", "usart2", None, None)
    }

    /// A new `USART2` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART2`
    /// peripheral.
    #[must_use]
    pub fn usart2_for<D: Device>() -> Self {
        Self::for_device_with::<D>("USART2", "usart2", None, None)
    }

    /// A new F103 `USART3` at `0x4000_4800`, IRQ 39.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `USART3`
    /// peripheral.
    #[must_use]
    pub fn usart3() -> Self {
        Self::for_device_with::<stm32f103::Stm32f103>("USART3", "usart3", None, None)
    }

    /// A new `USART3` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART3`
    /// peripheral.
    #[must_use]
    pub fn usart3_for<D: Device>() -> Self {
        Self::for_device_with::<D>("USART3", "usart3", None, None)
    }

    /// A new `UART4` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `UART4`
    /// peripheral.
    #[must_use]
    pub fn uart4_for<D: Device>() -> Self {
        Self::for_device_with::<D>("UART4", "uart4", None, None)
    }

    /// A new `USART4` on another chip's device tables (F0/F3 name the
    /// fourth port `USART4` rather than `UART4`).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART4`
    /// peripheral.
    #[must_use]
    pub fn usart4_for<D: Device>() -> Self {
        Self::for_device_with::<D>("USART4", "usart4", None, None)
    }

    /// A new `UART5` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `UART5`
    /// peripheral.
    #[must_use]
    pub fn uart5_for<D: Device>() -> Self {
        Self::for_device_with::<D>("UART5", "uart5", None, None)
    }

    /// A new `USART6` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART6`
    /// peripheral.
    #[must_use]
    pub fn usart6_for<D: Device>() -> Self {
        Self::for_device_with::<D>("USART6", "usart6", None, None)
    }

    /// A new `LPUART1` on another chip's device tables.
    #[must_use]
    pub fn lpuart1_for<D: Device>() -> Self {
        Self::for_device_with::<D>("LPUART1", "lpuart1", None, None)
    }

    /// A new F103 `USART1` sharing `requests` with its DMA controller. The F1
    /// routes USART1 TX to DMA1 channel 4 and RX to DMA1 channel 5.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `USART1`
    /// peripheral.
    #[must_use]
    pub fn with_dma(requests: Arc<Mutex<DmaRequests>>) -> Self {
        Self::dma_for::<stm32f103::Stm32f103>(
            requests,
            "USART1",
            "usart1",
            USART1_TX_CHANNEL,
            USART1_RX_CHANNEL,
        )
    }

    /// A new `USART1` on another chip's device tables, wired to its DMA
    /// controller through the F1 request routing (TX on DMA1 channel 4, RX on
    /// DMA1 channel 5).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART1`
    /// peripheral.
    #[must_use]
    pub fn for_device_with_dma<D: Device>(requests: Arc<Mutex<DmaRequests>>) -> Self {
        Self::dma_for::<D>(
            requests,
            "USART1",
            "usart1",
            USART1_TX_CHANNEL,
            USART1_RX_CHANNEL,
        )
    }

    /// A new `USART2` on another chip's device tables, wired to its DMA
    /// controller through the F1 request routing (TX on DMA1 channel 7, RX on
    /// DMA1 channel 6).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART2`
    /// peripheral.
    #[must_use]
    pub fn usart2_for_device_with_dma<D: Device>(requests: Arc<Mutex<DmaRequests>>) -> Self {
        Self::dma_for::<D>(
            requests,
            "USART2",
            "usart2",
            USART2_TX_CHANNEL,
            USART2_RX_CHANNEL,
        )
    }

    /// Share peripheral-request lines with explicit routing `(controller, tx,
    /// rx)` (F4 boards address DMA1/2 streams by number; F1 boards use the
    /// baked `*_for_device_with_dma` constructors instead).
    #[must_use]
    pub fn with_dma_routed(
        mut self,
        requests: Arc<Mutex<DmaRequests>>,
        controller: u8,
        tx: u8,
        rx: u8,
    ) -> Self {
        self.inner.behavior_mut().requests = Some(requests);
        self.inner.behavior_mut().dma = Some((controller, tx, rx));
        self
    }

    /// A new `USART3` on another chip's device tables, wired to its DMA
    /// controller through the F1 request routing (TX on DMA1 channel 2, RX on
    /// DMA1 channel 3).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART3`
    /// peripheral.
    #[must_use]
    pub fn usart3_for_device_with_dma<D: Device>(requests: Arc<Mutex<DmaRequests>>) -> Self {
        Self::dma_for::<D>(
            requests,
            "USART3",
            "usart3",
            USART3_TX_CHANNEL,
            USART3_RX_CHANNEL,
        )
    }

    /// A new `UART4` on another chip's device tables, wired to its DMA
    /// controller through the F1 request routing (TX on DMA1 channel 5, RX on
    /// DMA1 channel 6).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `UART4`
    /// peripheral.
    #[must_use]
    pub fn uart4_for_device_with_dma<D: Device>(requests: Arc<Mutex<DmaRequests>>) -> Self {
        Self::dma_for::<D>(
            requests,
            "UART4",
            "uart4",
            UART4_TX_CHANNEL,
            UART4_RX_CHANNEL,
        )
    }

    /// A new `UART5` on another chip's device tables, wired to its DMA
    /// controller through the F1 request routing (TX on DMA1 channel 7, RX on
    /// DMA1 channel 4).
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `UART5`
    /// peripheral.
    #[must_use]
    pub fn uart5_for_device_with_dma<D: Device>(requests: Arc<Mutex<DmaRequests>>) -> Self {
        Self::dma_for::<D>(
            requests,
            "UART5",
            "uart5",
            UART5_TX_CHANNEL,
            UART5_RX_CHANNEL,
        )
    }

    fn dma_for<D: Device>(
        requests: Arc<Mutex<DmaRequests>>,
        peripheral: &str,
        name: &'static str,
        tx_channel: u8,
        rx_channel: u8,
    ) -> Self {
        Self::for_device_with::<D>(
            peripheral,
            name,
            Some(requests),
            Some((DMA1_CONTROLLER, tx_channel, rx_channel)),
        )
    }

    /// Find a register by logical name across SVD naming variants: plain
    /// (`ISR`), U5 enable-split (`ISR_enabled`), and H5/WBA prefixed
    /// (`USART_ISR`, `USART_ISR_enabled`).
    fn reg_find<'d>(
        def: &'d crate::device::svd::PeripheralDef,
        name: &str,
    ) -> Option<&'d crate::device::svd::RegisterDef> {
        def.registers.iter().find(|r| {
            r.name == name
                || r.name == format!("{name}_enabled")
                || r.name == format!("USART_{name}")
                || r.name == format!("USART_{name}_enabled")
        })
    }
    fn reg_offset(def: &crate::device::svd::PeripheralDef, name: &str) -> Option<u32> {
        Self::reg_find(def, name).map(|r| r.offset)
    }
    fn field_bit(def: &crate::device::svd::PeripheralDef, reg: &str, field: &str) -> Option<u8> {
        Self::reg_find(def, reg)
            .and_then(|r| r.fields.iter().find(|f| f.name == field).map(|f| f.offset))
    }

    fn for_device_with<D: Device>(
        peripheral: &str,
        name: &'static str,
        requests: Option<Arc<Mutex<DmaRequests>>>,
        dma: Option<(u8, u8, u8)>,
    ) -> Self {
        let def = D::peripheral(peripheral)
            .unwrap_or_else(|| panic!("generated tables contain {peripheral}"));
        // Resolve per-family register offsets and bit positions from the SVD.
        // Old (F1) uses SR/DR/CR1@0x0C with UE13; new (F0/F3/L4) uses ISR/RDR/TDR/CR1@0x00 with UE0.
        let is_new = Self::reg_offset(def, "ISR").is_some();
        let (cr1_off, sr_off, rdr_off, tdr_off) = if is_new {
            (
                Self::reg_offset(def, "CR1").unwrap_or(0x00),
                Self::reg_offset(def, "ISR").unwrap_or(0x1C),
                Self::reg_offset(def, "RDR").unwrap_or(0x24),
                Self::reg_offset(def, "TDR").unwrap_or(0x28),
            )
        } else {
            (
                Self::reg_offset(def, "CR1").unwrap_or(CR1_F1),
                Self::reg_offset(def, "SR").unwrap_or(SR_F1),
                Self::reg_offset(def, "DR").unwrap_or(DR_F1),
                Self::reg_offset(def, "DR").unwrap_or(DR_F1),
            )
        };
        let ue_bit = Self::field_bit(def, "CR1", "UE")
            .map(|b| 1u32 << b)
            .unwrap_or(if is_new { 1 << 0 } else { 1 << 13 });
        let te_bit = Self::field_bit(def, "CR1", "TE")
            .map(|b| 1u32 << b)
            .unwrap_or(1 << 3);
        let re_bit = Self::field_bit(def, "CR1", "RE")
            .map(|b| 1u32 << b)
            .unwrap_or(1 << 2);
        let rxneie_bit = Self::field_bit(def, "CR1", "RXNEIE")
            .map(|b| 1u32 << b)
            .unwrap_or(1 << 5);
        let rxne_bit = Self::field_bit(def, if is_new { "ISR" } else { "SR" }, "RXNE")
            .map(|b| 1u32 << b)
            .unwrap_or(1 << 5);
        let txe_bit = Self::field_bit(def, if is_new { "ISR" } else { "SR" }, "TXE")
            .map(|b| 1u32 << b)
            .unwrap_or(1 << 7);
        let tc_bit = Self::field_bit(def, if is_new { "ISR" } else { "SR" }, "TC")
            .map(|b| 1u32 << b)
            .unwrap_or(1 << 6);
        let cr2_off = Self::reg_offset(def, "CR2");
        let isr_off = if is_new {
            Some(Self::reg_offset(def, "ISR").unwrap_or(0x1C))
        } else {
            None
        };
        let clken_bit = Self::field_bit(def, "CR2", "CLKEN").map(|b| 1u32 << b);
        let abren_bit = Self::field_bit(def, "CR2", "ABREN").map(|b| 1u32 << b);
        let abrf_bit = Self::field_bit(def, "ISR", "ABRF")
            .or_else(|| Self::field_bit(def, "ISR", "ABRE"))
            .map(|b| 1u32 << b);
        // Receive error model, resolved from the status register fields so
        // F1 `SR` and new-gen `ISR` layouts both work.
        let status_reg = if is_new { "ISR" } else { "SR" };
        let ore_bit = Self::field_bit(def, status_reg, "ORE")
            .map(|b| 1u32 << b)
            .unwrap_or(ORE_FALLBACK);
        let fe_bit = Self::field_bit(def, status_reg, "FE")
            .map(|b| 1u32 << b)
            .unwrap_or(FE_FALLBACK);
        let pe_bit = Self::field_bit(def, status_reg, "PE")
            .map(|b| 1u32 << b)
            .unwrap_or(PE_FALLBACK);
        let peie_bit = Self::field_bit(def, "CR1", "PEIE")
            .map(|b| 1u32 << b)
            .unwrap_or(PEIE_FALLBACK);
        let (cr3_off, eie_bit) = match Self::reg_offset(def, "CR3") {
            Some(off) => (
                Some(off),
                Self::field_bit(def, "CR3", "EIE")
                    .map(|b| 1u32 << b)
                    .unwrap_or(EIE_FALLBACK),
            ),
            None => (None, 0),
        };
        let icr_off = if is_new {
            Self::reg_offset(def, "ICR")
        } else {
            None
        };
        let icr_orecf = Self::field_bit(def, "ICR", "ORECF")
            .map(|b| 1u32 << b)
            .unwrap_or(ORE_FALLBACK);
        let icr_fecf = Self::field_bit(def, "ICR", "FECF")
            .map(|b| 1u32 << b)
            .unwrap_or(FE_FALLBACK);
        let icr_pecf = Self::field_bit(def, "ICR", "PECF")
            .map(|b| 1u32 << b)
            .unwrap_or(PE_FALLBACK);
        let behavior = UsartBehavior {
            output: Vec::new(),
            requests,
            dma,
            cr1_off,
            sr_off,
            rdr_off,
            tdr_off,
            cr2_off,
            isr_off,
            ue_bit,
            te_bit,
            re_bit,
            rxneie_bit,
            rxne_bit,
            txe_bit,
            tc_bit,
            clken_bit,
            abren_bit,
            abrf_bit,
            ore_bit,
            fe_bit,
            pe_bit,
            peie_bit,
            cr3_off,
            eie_bit,
            icr_off,
            icr_orecf,
            icr_fecf,
            icr_pecf,
            sr_armed: false,
        };
        Self {
            inner: RegisterFile::new(def, behavior),
            name,
        }
    }
}

impl Default for Usart {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Usart {
    fn ranges(&self) -> &[Region] {
        self.inner.ranges()
    }

    fn read(&mut self, addr: u32) -> Result<u8, BusError> {
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

    fn name(&self) -> &'static str {
        self.name
    }

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}

impl InterruptSource for Usart {
    fn exception_number(&self) -> Option<u8> {
        self.inner.exception_number()
    }

    fn pending(&self) -> u32 {
        self.inner.pending()
    }

    fn acknowledge(&mut self, lines: u32) {
        self.inner.acknowledge(lines);
    }
}

#[cfg(test)]
mod tests {
    use super::{
        Usart, CR1_F1 as CR1, CR2_F1 as CR2, DR_F1 as DR, RXNE, RXNEIE, SR_F1 as SR, TC, TE, TXE,
        UE,
    };
    use crate::device::svd::stm32f407::Stm32f407;
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(u: &mut Usart, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            u.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(u: &mut Usart, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = u.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn enabled() -> Usart {
        let mut u = Usart::new();
        write_word(&mut u, 0x4001_3800 + CR1, UE | TE | (1 << 2));
        u
    }

    #[test]
    fn tx_queues_bytes_and_drains() {
        let mut u = enabled();
        u.write(0x4001_3800 + DR, b'H').unwrap();
        assert_eq!(u.drain_output(), b"H");
        assert_eq!(u.drain_output(), b"");
    }

    #[test]
    fn status_tracks_enable_and_data_ready() {
        let mut u = enabled();
        assert_eq!(read_word(&mut u, 0x4001_3800 + SR) & (TXE | TC), TXE | TC);
        u.drive_inputs(0x5A);
        assert_eq!(read_word(&mut u, 0x4001_3800 + SR) & RXNE, RXNE);
    }

    #[test]
    fn rxne_is_cleared_by_reading_dr() {
        let mut u = enabled();
        u.drive_inputs(0x77);
        assert_eq!(read_word(&mut u, 0x4001_3800 + DR) & 0xFF, 0x77);
        assert_eq!(read_word(&mut u, 0x4001_3800 + SR) & RXNE, 0);
    }

    #[test]
    fn rx_interrupt_is_level_triggered_and_masked() {
        let mut u = Usart::new();
        // Byte arrives before the peripheral is enabled: no interrupt yet.
        u.drive_inputs(0x41);
        assert_eq!(u.pending(), 0);
        // Enabling UE + RXNEIE (firmware init) surfaces the waiting byte.
        write_word(&mut u, 0x4001_3800 + CR1, UE | RXNEIE);
        assert_eq!(u.pending(), 1);
        // The handler drains DR; the level drops.
        let _ = read_word(&mut u, 0x4001_3800 + DR);
        assert_eq!(u.pending(), 0);
    }

    #[test]
    fn rxneie_off_means_no_interrupt() {
        let mut u = enabled();
        u.drive_inputs(0x42);
        assert_eq!(u.pending(), 0);
    }

    #[test]
    fn addresses_outside_the_window_are_reserved() {
        let mut u = Usart::new();
        assert!(matches!(
            u.read(0x4001_3800 + 0x1C),
            Err(BusError::Reserved)
        ));
        assert!(matches!(
            u.write(0x4001_3800 - 1, 0),
            Err(BusError::Reserved)
        ));
    }

    #[test]
    fn reset_clears_status_and_output() {
        let mut u = enabled();
        u.drive_inputs(0x11);
        write_word(&mut u, 0x4001_3800 + DR, 0x22);
        u.reset();
        assert_eq!(read_word(&mut u, 0x4001_3800 + SR) & (TXE | TC | RXNE), 0);
        assert_eq!(u.drain_output(), b"");
    }

    #[test]
    fn dma_requests_follow_tx_enable_and_rx_pending() {
        use super::DmaRequests;
        use std::sync::{Arc, Mutex};
        let requests = Arc::new(Mutex::new(DmaRequests::new()));
        let mut u = Usart::with_dma(requests.clone());
        // Disabled: no TX request yet.
        u.tick(1);
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .to_periph(),
            0
        );
        // UE + TE: the TX line asserts on DMA1 channel 4.
        write_word(&mut u, 0x4001_3800 + CR1, UE | TE);
        u.tick(1);
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .to_periph(),
            1 << 4
        );
        // A received byte asserts the RX line on DMA1 channel 5.
        u.drive_inputs(0x5A);
        u.tick(1);
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .from_periph(),
            1 << 5
        );
        // Reading DR drains the byte and drops the RX request.
        let _ = read_word(&mut u, 0x4001_3800 + DR);
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .from_periph(),
            0
        );
        // The TX line stays asserted while the transmitter is enabled.
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .to_periph(),
            1 << 4
        );
    }

    #[test]
    fn second_and_third_instances_keep_the_same_behavior() {
        // USART2 on F103, USART3 on F407: the byte-oriented behavior is
        // instance-independent, only the table (base + IRQ) changes.
        let mut u2 = Usart::usart2();
        assert_eq!(u2.ranges()[0].base, 0x4000_4400);
        write_word(&mut u2, 0x4000_4400 + CR1, UE | RXNEIE);
        u2.drive_inputs(0x2A);
        assert_eq!(u2.pending(), 1);
        assert_eq!(u2.exception_number(), Some(16 + 38));

        let mut u3 = Usart::usart3_for::<Stm32f407>();
        assert_eq!(u3.ranges()[0].base, 0x4000_4800);
        write_word(&mut u3, 0x4000_4800 + CR1, UE | TE);
        u3.write(0x4000_4800 + DR, b'Z').unwrap();
        assert_eq!(u3.drain_output(), b"Z");
        assert_eq!(u3.exception_number(), Some(16 + 39));
    }

    #[test]
    fn connectivity_and_f4_instances_surface_the_waiting_byte() {
        use crate::device::svd::stm32f107::Stm32f107;
        // UART4 on F107 (base 0x4000_4C00, IRQ 52) and USART6 on F407 (base
        // 0x4001_1400, IRQ 71): a byte injected before enable is surfaced by
        // UE + RXNEIE, and drained by reading DR.
        let mut u4 = Usart::uart4_for::<Stm32f107>();
        assert_eq!(u4.ranges()[0].base, 0x4000_4C00);
        u4.drive_inputs(0x44);
        write_word(&mut u4, 0x4000_4C00 + CR1, UE | RXNEIE);
        assert_eq!(u4.pending(), 1);
        assert_eq!(read_word(&mut u4, 0x4000_4C00 + DR) & 0xFF, 0x44);
        assert_eq!(u4.pending(), 0);

        let mut u6 = Usart::usart6_for::<Stm32f407>();
        assert_eq!(u6.ranges()[0].base, 0x4001_1400);
        u6.drive_inputs(0x66);
        write_word(&mut u6, 0x4001_1400 + CR1, UE | RXNEIE);
        assert_eq!(u6.pending(), 1);
        assert_eq!(read_word(&mut u6, 0x4001_1400 + DR) & 0xFF, 0x66);
        assert_eq!(u6.exception_number(), Some(16 + 71));
    }

    #[test]
    fn each_instance_uses_its_own_dma_channels() {
        use super::DmaRequests;
        use crate::device::svd::stm32f107::Stm32f107;
        use std::sync::{Arc, Mutex};
        // USART2: TX on channel 7, RX on channel 6; USART3: TX on channel 2,
        // RX on channel 3; UART5: TX on channel 7, RX on channel 4.
        let requests = Arc::new(Mutex::new(DmaRequests::new()));
        let mut u2 = Usart::usart2_for_device_with_dma::<Stm32f107>(requests.clone());
        write_word(&mut u2, 0x4000_4400 + CR1, UE | TE);
        u2.drive_inputs(0x5A);
        u2.tick(1);
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .to_periph(),
            1 << 7
        );
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .from_periph(),
            1 << 6
        );

        let mut u3 = Usart::usart3_for_device_with_dma::<Stm32f107>(requests.clone());
        write_word(&mut u3, 0x4000_4800 + CR1, UE | TE);
        u3.drive_inputs(0x5A);
        u3.tick(1);
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .to_periph(),
            1 << 7 | 1 << 2
        );
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .from_periph(),
            1 << 6 | 1 << 3
        );

        let mut u5 = Usart::uart5_for_device_with_dma::<Stm32f107>(requests.clone());
        write_word(&mut u5, 0x4000_5000 + CR1, UE | TE);
        u5.drive_inputs(0x5A);
        u5.tick(1);
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .to_periph(),
            1 << 7 | 1 << 2
        );
        assert_eq!(
            requests
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .from_periph(),
            1 << 6 | 1 << 3 | 1 << 4
        );
    }

    /// LIN break detection: a break input (drive bit 8) latches `SR.LBD`
    /// only when `CR2.LBDIE` is set, asserts the line, and software clears
    /// the flag by writing zeros.
    #[test]
    fn lin_break_latches_lbd_when_enabled() {
        const B: u32 = 0x4001_3800;
        let mut u = Usart::new();
        u.reset();
        // Enable UE + LBDIE.
        write_word(&mut u, B + CR1, UE);
        write_word(&mut u, B + CR2, 1 << 6); // CR2.LBDIE
                                             // Break arrives.
        u.drive_inputs(1 << 8);
        assert_ne!(read_word(&mut u, B + SR) & (1 << 8), 0, "LBD latched");
        assert_eq!(u.pending(), 1, "line via LBDIE");
        // Software clear: write zeros over the flag.
        write_word(&mut u, B + SR, !(1u32 << 8));
        assert_eq!(read_word(&mut u, B + SR) & (1 << 8), 0);
        assert_eq!(u.pending(), 0);
        // Without LBDIE the break is invisible.
        write_word(&mut u, B + CR2, 0);
        u.drive_inputs(1 << 8);
        assert_eq!(read_word(&mut u, B + SR) & (1 << 8), 0, "masked");
        assert_eq!(u.pending(), 0);
    }

    /// F1 `SR` error bits for the tests below.
    const ORE: u32 = 1 << 3;
    const FE: u32 = 1 << 1;
    const PE: u32 = 1 << 0;
    /// F1 `CR1.PEIE` (bit 8), `CR3` offset and `CR3.EIE` (bit 0).
    const PEIE: u32 = 1 << 8;
    const CR3_F1: u32 = 0x14;
    const EIE: u32 = 1 << 0;

    /// Overrun: a byte arriving while `RXNE` is set latches `ORE` and is
    /// dropped; the first byte survives.
    #[test]
    fn overrun_sets_ore_and_keeps_first_byte() {
        const B: u32 = 0x4001_3800;
        let mut u = enabled();
        u.drive_inputs(0x41);
        u.drive_inputs(0x42);
        assert_eq!(read_word(&mut u, B + SR) & ORE, ORE, "ORE latched");
        assert_eq!(read_word(&mut u, B + DR) & 0xFF, 0x41, "first byte kept");
    }

    /// `ORE` survives a bare `DR` read and clears only inside the
    /// `SR`-then-`DR` sequence.
    #[test]
    fn ore_clears_only_via_sr_then_dr_sequence() {
        const B: u32 = 0x4001_3800;
        let mut u = enabled();
        u.drive_inputs(0x41);
        u.drive_inputs(0x42);
        // Bare DR read drains the byte but keeps the flag.
        let _ = read_word(&mut u, B + DR);
        assert_eq!(read_word(&mut u, B + SR) & ORE, ORE, "flag survives");
        // The sequence clears it.
        let _ = read_word(&mut u, B + SR);
        let _ = read_word(&mut u, B + DR);
        assert_eq!(read_word(&mut u, B + SR) & ORE, 0, "flag cleared");
    }

    /// Stimulus bits 9/10 inject framing/parity errors; they clear with
    /// the same sequence.
    #[test]
    fn framing_and_parity_inject_and_clear_via_sequence() {
        const B: u32 = 0x4001_3800;
        let mut u = enabled();
        u.drive_inputs((1 << 10) | (1 << 9) | 0x55);
        // Bare DR read drains the byte but keeps the flags (no SR read
        // armed the sequence yet).
        let _ = read_word(&mut u, B + DR);
        assert_eq!(read_word(&mut u, B + SR) & (FE | PE), FE | PE, "kept");
        let _ = read_word(&mut u, B + DR);
        assert_eq!(read_word(&mut u, B + SR) & (FE | PE), 0, "cleared");
    }

    /// Error IRQs: `ORE` needs `CR3.EIE`, `PE` needs `CR1.PEIE`; without
    /// them the flags latch silently.
    #[test]
    fn error_interrupt_asserts_via_eie_and_peie() {
        const B: u32 = 0x4001_3800;
        let mut u = enabled();
        // Overrun with no enables: flag, no line.
        u.drive_inputs(0x41);
        u.drive_inputs(0x42);
        assert_eq!(u.pending(), 0);
        // `EIE` routes the overrun to the line; the sequence drops it.
        write_word(&mut u, B + CR3_F1, EIE);
        assert_eq!(u.pending(), 1, "ORE line via EIE");
        let _ = read_word(&mut u, B + SR);
        let _ = read_word(&mut u, B + DR);
        assert_eq!(u.pending(), 0);
        // `PE` needs `PEIE`: `EIE` alone does not route it.
        u.drive_inputs((1 << 10) | 0x33);
        assert_eq!(u.pending(), 0, "PE masked without PEIE");
        write_word(&mut u, B + CR1, UE | TE | (1 << 2) | PEIE);
        assert_eq!(u.pending(), 1, "PE line via PEIE");
    }

    /// New-gen tables clear errors by writing 1 to `ICR`, not with the
    /// `SR` sequence: F0 `USART1` overrun clears via `ICR.ORECF`.
    #[test]
    fn icr_write_clears_overrun_on_new_gen() {
        use crate::device::svd::stm32f030::Stm32f030;
        const B: u32 = 0x4001_3800;
        const CR1: u32 = 0x00;
        const ISR: u32 = 0x1C;
        const RDR: u32 = 0x24;
        const ICR: u32 = 0x20;
        const UE0: u32 = 1 << 0;
        let mut u = Usart::usart1_for::<Stm32f030>();
        write_word(&mut u, B + CR1, UE0 | (1 << 3) | (1 << 2));
        u.drive_inputs(0x41);
        u.drive_inputs(0x42);
        assert_eq!(read_word(&mut u, B + ISR) & ORE, ORE, "ORE latched");
        // A bare RDR read keeps the flag (no F1 sequence on new-gen).
        let _ = read_word(&mut u, B + RDR);
        assert_eq!(read_word(&mut u, B + ISR) & ORE, ORE, "flag survives");
        // `ICR.ORECF` clears it; the first byte's data is still there.
        u.write(B + ICR, 1 << 3).unwrap();
        assert_eq!(read_word(&mut u, B + ISR) & ORE, 0, "flag cleared");
        assert_eq!(read_word(&mut u, B + RDR) & 0xFF, 0x41, "data kept");
    }
}
