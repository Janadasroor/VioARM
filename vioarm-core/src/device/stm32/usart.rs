//! The STM32F1 `USART1` (`0x4001_3800`, IRQ 37), driven by the SVD-generated
//! register tables.
//!
//! A byte-oriented model of the real register file: `SR`/`DR`/`BRR`/`CR1..3`
//! with the `TXE`/`TC`/`RXNE` status bits. Transmitted bytes are captured into
//! a host-readable buffer (like the generic `UartTx`); received bytes are
//! injected by the host through the stimulus channel. `RXNE` is set on
//! receive, cleared by reading `DR`, and — together with `CR1.RXNEIE` and
//! `CR1.UE` — drives a level-triggered IRQ that keeps asserting until the
//! handler drains the data register, exactly as on silicon. The two things a
//! table cannot express live in [`UsartBehavior`]: the `TXE`/`TC` status
//! synthesis and the read-to-clear `RXNE` side effect.
//!
//! With a [`DmaRequests`] mailbox wired in, the USART also feeds the DMA
//! controllers: `USART1` asserts a memory→peripheral request on DMA1 channel 4
//! (the TX line) while `UE` and `TE` are set, and a peripheral→memory request
//! on DMA1 channel 5 (the RX line) while a received byte is pending in `DR`.
//! Reading `DR` (or writing zero to it) drops the RX request, mirroring the
//! level request line that falls when the data register is drained.

use crate::device::stm32::dma::DmaRequests;
use crate::device::svd::stm32f103;
use crate::device::svd::{Behavior, Device, RegisterFile, Registers};
use crate::device::{InterruptSource, Peripheral};
use crate::error::BusError;
use crate::mem::bus::Region;
use std::sync::{Arc, Mutex};

/// Register offsets within the peripheral, per the SVD.
const SR: u16 = 0x00;
const DR: u16 = 0x04;
const CR1: u16 = 0x0C;

/// `SR` bit positions.
const RXNE: u32 = 1 << 5;
const TC: u32 = 1 << 6;
const TXE: u32 = 1 << 7;

/// `CR1` bit positions.
const UE: u32 = 1 << 13;
const TE: u32 = 1 << 3;
const RXNEIE: u32 = 1 << 5;

/// The F1 USART1→DMA1 request routing: TX on channel 4, RX on channel 5.
const DMA1_CONTROLLER: u8 = 0;
const TX_DMA_CHANNEL: u8 = 4;
const RX_DMA_CHANNEL: u8 = 5;

/// The cross-register side effects the SVD cannot express: status synthesis
/// on `SR` reads, `RXNE` acknowledge on `DR` reads and zero writes, and the
/// DMA request lines asserted while data is ready to move.
#[derive(Debug, Default)]
struct UsartBehavior {
    /// Bytes firmware wrote to the data register, for host draining.
    output: Vec<u8>,
    /// The DMA request mailbox, when a DMA controller is wired to this USART.
    requests: Option<Arc<Mutex<DmaRequests>>>,
    /// The request routing `(controller, tx_channel, rx_channel)`.
    dma: Option<(u8, u8, u8)>,
}

impl Behavior for UsartBehavior {
    fn on_read(&mut self, view: &mut Registers<&mut [u32]>, offset: u16, value: u32) -> u32 {
        match offset {
            SR => {
                // `TXE`/`TC` track the enable bits, the rest is latched status.
                let mut v = value & !(TXE | TC);
                let cr1 = view.get(CR1);
                if cr1 & UE != 0 {
                    v |= TC;
                    if cr1 & TE != 0 {
                        v |= TXE;
                    }
                }
                v
            }
            DR => {
                // Reading the data register drains the received byte and
                // acknowledges `RXNE`, dropping the DMA RX request line.
                view.set(SR, view.get(SR) & !RXNE);
                self.drop_rx_request();
                value
            }
            _ => value,
        }
    }

    fn on_write(
        &mut self,
        view: &mut Registers<&mut [u32]>,
        offset: u16,
        val: u8,
        shift: u32,
    ) -> u32 {
        match offset {
            DR => {
                // Byte 0 of the data register is the transmitted character;
                // the higher lanes are the extension of the same store on
                // silicon, so only it queues a byte for output.
                if shift == 0 {
                    if view.get(CR1) & TE != 0 {
                        self.output.push(val);
                    }
                    // A zero write to the data register acknowledges `RXNE`,
                    // matching the silicon rule that zero is not data.
                    if val == 0 {
                        view.set(SR, view.get(SR) & !RXNE);
                        self.drop_rx_request();
                    }
                }
                // A transmit does not alter the received data.
                view.get(DR)
            }
            _ => view.merged(offset, val, shift),
        }
    }

fn tick(&mut self, view: &mut Registers<&mut [u32]>, _delta: u64) {
        let (requests, dma) = (&self.requests, self.dma);
        let (Some(requests), Some((controller, tx_channel, rx_channel))) = (requests, dma) else {
            return;
        };
        let mut req = requests.lock().unwrap();
        let cr1 = view.get(CR1);
        // The TX line is high whenever the transmitter can take a byte.
        if cr1 & (UE | TE) == (UE | TE) {
            req.request(controller, tx_channel, true);
        }
        // The RX line is high while a received byte sits unread in `DR`.
        if view.get(SR) & RXNE != 0 {
            req.request(controller, rx_channel, false);
        }
    }

    fn pending(&self, view: &Registers<&[u32]>) -> u32 {
        // Level-triggered: asserts while a received byte is both unread and
        // unmasked. The handler clears it by reading `DR`.
        u32::from(view.get(SR) & RXNE != 0 && view.get(CR1) & (RXNEIE | UE) == (RXNEIE | UE))
    }

    fn reset(&mut self) {
        self.output.clear();
    }

    fn drive_inputs(&mut self, view: &mut Registers<&mut [u32]>, value: u32) {
        // The low byte is one received character.
        view.set(DR, value & 0xFF);
        view.set(SR, view.get(SR) | RXNE);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.output)
    }
}

impl UsartBehavior {
    /// Deassert the RX DMA request line (the data register was drained).
    fn drop_rx_request(&mut self) {
        if let (Some(requests), Some((controller, _, rx_channel))) =
            (&self.requests, self.dma)
        {
            requests
                .lock()
                .unwrap()
                .release(controller, rx_channel, false);
        }
    }
}

/// The `USART1` register file, table-driven from the SVD.
#[derive(Debug)]
pub struct Usart1 {
    inner: RegisterFile<UsartBehavior>,
}

impl Usart1 {
    /// A new `USART1` at the STM32F1 base address.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `USART1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn new() -> Self {
        Self::for_device::<stm32f103::Stm32f103>()
    }

    /// A new `USART1` on another chip's device tables.
    ///
    /// # Panics
    /// Panics if `D`'s generated tables do not contain the `USART1`
    /// peripheral.
    #[must_use]
    pub fn for_device<D: Device>() -> Self {
        let def = D::peripheral("USART1").expect("generated tables contain USART1");
        Self {
            inner: RegisterFile::new(def, UsartBehavior::default()),
        }
    }

    /// A new `USART1` sharing `requests` with its DMA controller. The F1
    /// routes USART1 TX to DMA1 channel 4 and RX to DMA1 channel 5.
    ///
    /// # Panics
    /// Panics if the generated device tables do not contain the `USART1`
    /// peripheral (a build-time invariant enforced by conformance tests).
    #[must_use]
    pub fn with_dma(requests: Arc<Mutex<DmaRequests>>) -> Self {
        Self::for_device_with_dma::<stm32f103::Stm32f103>(requests)
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
        let def = D::peripheral("USART1").expect("generated tables contain USART1");
        let behavior = UsartBehavior {
            output: Vec::new(),
            requests: Some(requests),
            dma: Some((DMA1_CONTROLLER, TX_DMA_CHANNEL, RX_DMA_CHANNEL)),
        };
        Self {
            inner: RegisterFile::new(def, behavior),
        }
    }
}

impl Default for Usart1 {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral for Usart1 {
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
        "usart1"
    }

    fn drive_inputs(&mut self, value: u32) {
        self.inner.drive_inputs(value);
    }

    fn drain_output(&mut self) -> Vec<u8> {
        self.inner.drain_output()
    }
}

impl InterruptSource for Usart1 {
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
    use super::{Usart1, CR1, DR, RXNE, RXNEIE, SR, TC, TE, TXE, UE};
    use crate::device::{InterruptSource, Peripheral};
    use crate::error::BusError;

    const BASE: u32 = 0x4001_3800;

    #[allow(clippy::cast_possible_truncation)]
    fn write_word(u: &mut Usart1, addr: u32, val: u32) {
        for (i, b) in val.to_le_bytes().iter().enumerate() {
            u.write(addr.wrapping_add(i as u32), *b).unwrap();
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn read_word(u: &mut Usart1, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = u.read(addr.wrapping_add(i as u32)).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn enabled() -> Usart1 {
        let mut u = Usart1::new();
        write_word(&mut u, BASE + u32::from(CR1), UE | TE | (1 << 2));
        u
    }

    #[test]
    fn tx_queues_bytes_and_drains() {
        let mut u = enabled();
        u.write(BASE + u32::from(DR), b'H').unwrap();
        assert_eq!(u.drain_output(), b"H");
        assert_eq!(u.drain_output(), b"");
    }

    #[test]
    fn status_tracks_enable_and_data_ready() {
        let mut u = enabled();
        assert_eq!(
            read_word(&mut u, BASE + u32::from(SR)) & (TXE | TC),
            TXE | TC
        );
        u.drive_inputs(0x5A);
        assert_eq!(read_word(&mut u, BASE + u32::from(SR)) & RXNE, RXNE);
    }

    #[test]
    fn rxne_is_cleared_by_reading_dr() {
        let mut u = enabled();
        u.drive_inputs(0x77);
        assert_eq!(read_word(&mut u, BASE + u32::from(DR)) & 0xFF, 0x77);
        assert_eq!(read_word(&mut u, BASE + u32::from(SR)) & RXNE, 0);
    }

    #[test]
    fn rx_interrupt_is_level_triggered_and_masked() {
        let mut u = Usart1::new();
        // Byte arrives before the peripheral is enabled: no interrupt yet.
        u.drive_inputs(0x41);
        assert_eq!(u.pending(), 0);
        // Enabling UE + RXNEIE (firmware init) surfaces the waiting byte.
        write_word(&mut u, BASE + u32::from(CR1), UE | RXNEIE);
        assert_eq!(u.pending(), 1);
        // The handler drains DR; the level drops.
        let _ = read_word(&mut u, BASE + u32::from(DR));
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
        let mut u = Usart1::new();
        assert!(matches!(u.read(BASE + 0x1C), Err(BusError::Reserved)));
        assert!(matches!(u.write(BASE - 1, 0), Err(BusError::Reserved)));
    }

    #[test]
    fn reset_clears_status_and_output() {
        let mut u = enabled();
        u.drive_inputs(0x11);
        write_word(&mut u, BASE + u32::from(DR), 0x22);
        u.reset();
        assert_eq!(
            read_word(&mut u, BASE + u32::from(SR)) & (TXE | TC | RXNE),
            0
        );
        assert_eq!(u.drain_output(), b"");
    }

    #[test]
    fn dma_requests_follow_tx_enable_and_rx_pending() {
        use super::DmaRequests;
        use std::sync::{Arc, Mutex};
        let requests = Arc::new(Mutex::new(DmaRequests::new()));
        let mut u = Usart1::with_dma(requests.clone());
        // Disabled: no TX request yet.
        u.tick(1);
        assert_eq!(requests.lock().unwrap().to_periph(), 0);
        // UE + TE: the TX line asserts on DMA1 channel 4.
        write_word(&mut u, BASE + u32::from(CR1), UE | TE);
        u.tick(1);
        assert_eq!(requests.lock().unwrap().to_periph(), 1 << 4);
        // A received byte asserts the RX line on DMA1 channel 5.
        u.drive_inputs(0x5A);
        u.tick(1);
        assert_eq!(requests.lock().unwrap().from_periph(), 1 << 5);
        // Reading DR drains the byte and drops the RX request.
        let _ = read_word(&mut u, BASE + u32::from(DR));
        assert_eq!(requests.lock().unwrap().from_periph(), 0);
        // The TX line stays asserted while the transmitter is enabled.
        assert_eq!(requests.lock().unwrap().to_periph(), 1 << 4);
    }
}

