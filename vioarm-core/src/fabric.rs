//! Deterministic multi-machine fabric: virtual wires between `Machine`
//! instances stepped in lockstep by one scheduler. No threads, ever —
//! machines step in index order, then wires pump, so rerunning the same
//! scenario is byte-identical.
//!
//! Wires today: bidirectional UART links (drained TX bytes drive the peer's
//! RX) and a CAN bus with identifier arbitration (pending TX frames sort
//! lowest-identifier-first, standard before extended, machine index breaks
//! ties; each frame reaches every other member, whose own filters still
//! accept or drop it). SPI controller/peripheral pairs are deferred: the
//! SPI model is master-oriented (a slave never drains), so pair wiring
//! needs slave TX capture first.
//!
//! Lockstep is instruction-step granularity, not cycle-exact across
//! machines: each machine keeps its own `cyccnt`, and wires move bytes
//! between steps. Byte delivery is immediate within the pumping round.

use crate::error::HaltCause;
use crate::machine::Machine;
use crate::mem::PeripheralBus;

/// One endpoint of a wire: machine index plus device name.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Endpoint {
    /// Index into [`Fabric::machines`](Fabric::machines) (see [`Fabric::add`]).
    pub machine: usize,
    /// Device name (`Peripheral::name`, e.g. `"usart1"`, `"can1"`).
    pub device: String,
}

/// A bidirectional UART link: each round, drained TX bytes from each side
/// drive the other side's RX.
#[derive(Clone, Debug)]
pub struct UartLink {
    /// First endpoint.
    pub a: Endpoint,
    /// Second endpoint.
    pub b: Endpoint,
}

/// A shared CAN bus: every member's drained TX frames reach every other
/// member, ordered by identifier (arbitration).
#[derive(Clone, Debug)]
pub struct CanBus {
    /// Bus members.
    pub members: Vec<Endpoint>,
}

/// A shared radio medium (virtual air): every member's drained bytes reach
/// every other member in machine-index order. No arbitration, no
/// collisions — concurrent transmissions both land, lowest machine index
/// first (documented simplification: no RF physics by design).
#[derive(Clone, Debug)]
pub struct RadioBus {
    /// Air members.
    pub members: Vec<Endpoint>,
}

/// An SPI controller/peripheral pair: the controller's shifted-out bytes
/// become the peripheral's received frames, and the peripheral's staged
/// bytes become the controller's next MISO bytes. One lockstep round of
/// pipeline delay (documented): bytes staged this round are consumed next
/// round, so firmware paces on `RXNE`/`TXE` exactly like on silicon.
#[derive(Clone, Debug)]
pub struct SpiPair {
    /// Controller endpoint (master mode).
    pub controller: Endpoint,
    /// Peripheral endpoint (slave mode: `SPE` without `MSTR`).
    pub peripheral: Endpoint,
}

/// `Fabric::step` outcomes, one per machine in index order: `Ok` for a
/// retired step, `Err` with the halt cause (a halted machine keeps
/// reporting its cause; stepping it again is harmless).
pub type StepOutcomes = Vec<Result<(), HaltCause>>;

/// Two or more machines with virtual wires, stepped in lockstep.
pub struct Fabric<M: PeripheralBus> {
    machines: Vec<Machine<M>>,
    uart_links: Vec<UartLink>,
    can_buses: Vec<CanBus>,
    radio_buses: Vec<RadioBus>,
    spi_pairs: Vec<SpiPair>,
}

impl<M: PeripheralBus> Fabric<M> {
    /// An empty fabric; add machines with [`Fabric::add`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            machines: Vec::new(),
            uart_links: Vec::new(),
            can_buses: Vec::new(),
            radio_buses: Vec::new(),
            spi_pairs: Vec::new(),
        }
    }
    pub fn add(&mut self, machine: Machine<M>) -> usize {
        self.machines.push(machine);
        self.machines.len() - 1
    }

    /// The machines, in index order.
    #[must_use]
    pub fn machines(&self) -> &[Machine<M>] {
        &self.machines
    }

    /// The machines, mutably (inspect snapshots, drain channels, ...).
    pub fn machines_mut(&mut self) -> &mut [Machine<M>] {
        &mut self.machines
    }

    /// Link two UART endpoints bidirectionally.
    pub fn link_uart(&mut self, a_machine: usize, a_dev: &str, b_machine: usize, b_dev: &str) {
        self.uart_links.push(UartLink {
            a: Endpoint {
                machine: a_machine,
                device: a_dev.to_owned(),
            },
            b: Endpoint {
                machine: b_machine,
                device: b_dev.to_owned(),
            },
        });
    }

    /// Share one CAN bus between members.
    pub fn link_can(&mut self, members: &[(usize, &str)]) {
        self.can_buses.push(CanBus {
            members: members
                .iter()
                .map(|(machine, device)| Endpoint {
                    machine: *machine,
                    device: (*device).to_owned(),
                })
                .collect(),
        });
    }

    /// Share one radio medium between members (virtual air for the
    /// SUBGHZ/BLE packet endpoints; pair and star both work).
    pub fn link_radio(&mut self, members: &[(usize, &str)]) {
        self.radio_buses.push(RadioBus {
            members: members
                .iter()
                .map(|(machine, device)| Endpoint {
                    machine: *machine,
                    device: (*device).to_owned(),
                })
                .collect(),
        });
    }

    /// Pair an SPI controller with a peripheral. The controller side runs
    /// master mode, the peripheral side slave mode (`SPE` without `MSTR`,
    /// `SSM` recommended to dodge mode faults).
    pub fn link_spi(
        &mut self,
        controller: usize,
        controller_dev: &str,
        peripheral: usize,
        peripheral_dev: &str,
    ) {
        self.spi_pairs.push(SpiPair {
            controller: Endpoint {
                machine: controller,
                device: controller_dev.to_owned(),
            },
            peripheral: Endpoint {
                machine: peripheral,
                device: peripheral_dev.to_owned(),
            },
        });
    }

    /// Move SPI bytes across every pair: controller MOSI becomes peripheral
    /// frames, peripheral staged bytes become controller MISO staging.
    fn pump_spi(&mut self) {
        for i in 0..self.spi_pairs.len() {
            let pair = self.spi_pairs[i].clone();
            // Controller frames land on the peripheral as received frames.
            Self::move_bytes(&mut self.machines, &pair.controller, &pair.peripheral);
            // Peripheral staged bytes become the controller's next MISO.
            Self::move_bytes(&mut self.machines, &pair.peripheral, &pair.controller);
        }
    }

    /// One lockstep round: step every machine in index order, then pump
    /// every wire in attach order.
    pub fn step(&mut self) -> StepOutcomes {
        let mut outcomes = Vec::with_capacity(self.machines.len());
        for m in &mut self.machines {
            outcomes.push(m.step());
        }
        self.pump_uart();
        self.pump_can();
        self.pump_radio();
        self.pump_spi();
        outcomes
    }

    /// `rounds` lockstep rounds.
    pub fn run(&mut self, rounds: u64) {
        for _ in 0..rounds {
            self.step();
        }
    }

    /// Move UART bytes across every link, both directions. Declined
    /// endpoints (unknown device, or memory without devices) move nothing;
    /// wiring mistakes surface as missing traffic, never as faults.
    fn pump_uart(&mut self) {
        for i in 0..self.uart_links.len() {
            let link = self.uart_links[i].clone();
            Self::move_bytes(&mut self.machines, &link.a, &link.b);
            Self::move_bytes(&mut self.machines, &link.b, &link.a);
        }
    }

    /// Drain `from`'s device into `to`'s device. Returns whether both ends
    /// exist.
    fn move_bytes(machines: &mut [Machine<M>], from: &Endpoint, to: &Endpoint) -> bool {
        if from.machine >= machines.len()
            || to.machine >= machines.len()
            || from.machine == to.machine
        {
            return false;
        }
        if from.machine < to.machine {
            let (head, tail) = machines.split_at_mut(to.machine);
            let bytes = head[from.machine].drain_device(&from.device);
            tail[0].drive_device(&to.device, &bytes)
        } else {
            let (head, tail) = machines.split_at_mut(from.machine);
            let bytes = tail[0].drain_device(&from.device);
            head[to.machine].drive_device(&to.device, &bytes)
        }
    }

    /// Move CAN frames across every bus with identifier arbitration.
    fn pump_can(&mut self) {
        for i in 0..self.can_buses.len() {
            let bus = self.can_buses[i].clone();
            // Gather one round's transmissions: (sort key, source, frame).
            let mut pending: Vec<((u8, u32), usize, [u8; 16])> = Vec::new();
            for ep in &bus.members {
                if ep.machine >= self.machines.len() {
                    continue;
                }
                let out = self.machines[ep.machine].drain_device(&ep.device);
                for frame in out.chunks_exact(16) {
                    let mut raw = [0u8; 16];
                    raw.copy_from_slice(frame);
                    let ti = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
                    pending.push((can_sort_key(ti), ep.machine, raw));
                }
            }
            // Arbitration: lowest identifier first, standard before
            // extended, machine index breaks ties (deterministic).
            pending.sort();
            for (_, src, frame) in &pending {
                for ep in &bus.members {
                    if ep.machine == *src || ep.machine >= self.machines.len() {
                        continue;
                    }
                    let _ = self.machines[ep.machine].drive_device(&ep.device, frame);
                }
            }
        }
    }

    /// Move radio bytes across every medium: each member's drained bytes
    /// reach every other member, machines in index order.
    fn pump_radio(&mut self) {
        for i in 0..self.radio_buses.len() {
            let bus = self.radio_buses[i].clone();
            // Drain first (one pass, index order), then deliver: a member
            // never hears its own transmission.
            let mut aired: Vec<(usize, Vec<u8>)> = Vec::new();
            for ep in &bus.members {
                if ep.machine >= self.machines.len() {
                    continue;
                }
                let bytes = self.machines[ep.machine].drain_device(&ep.device);
                if !bytes.is_empty() {
                    aired.push((ep.machine, bytes));
                }
            }
            for (src, bytes) in &aired {
                for ep in &bus.members {
                    if ep.machine == *src || ep.machine >= self.machines.len() {
                        continue;
                    }
                    let _ = self.machines[ep.machine].drive_device(&ep.device, bytes);
                }
            }
        }
    }
}

impl<M: PeripheralBus> Default for Fabric<M> {
    fn default() -> Self {
        Self::new()
    }
}

/// CAN 2.0 arbitration order for a `TIR` word: standard frames (IDE clear)
/// beat extended ones; lower identifiers win. Firmware programs silicon
/// `TIR` layout (`STID` at 31:21, `IDE` at bit 2, `EXID` at 20:3).
const fn can_sort_key(ti: u32) -> (u8, u32) {
    if ti & (1 << 2) == 0 {
        (0, (ti >> 21) & 0x7FF)
    } else {
        (1, (ti >> 3) & 0x3_FFFF)
    }
}

#[cfg(test)]
mod tests {
    use super::{can_sort_key, Fabric};
    use crate::machine::Machine;

    #[test]
    fn sort_key_orders_standard_before_extended_then_id() {
        assert!(can_sort_key(0x080 << 21) < can_sort_key(0x100 << 21));
        assert!(can_sort_key(0x7FF << 21) < can_sort_key(1 << 2));
        assert!(can_sort_key((1 << 2) | (5 << 3)) < can_sort_key((1 << 2) | (9 << 3)));
    }

    #[test]
    fn fabric_steps_two_machines_in_order() {
        use crate::loader::load_flat;
        use crate::mem::flat::FlatMemory;
        const RAM: u32 = 0x2000_0000;
        let mut f: Fabric<FlatMemory> = Fabric::new();
        let mk = || {
            let mut m = Machine::armv7m(FlatMemory::with_layout(&[RAM], &[0x1000]));
            m.load_image(&load_flat(RAM, &[0x00, 0xBE]));
            m.core.regs.pc = RAM;
            m.core.xpsr.set(0x0100_0000);
            m
        };
        let (a, b) = (f.add(mk()), f.add(mk()));
        assert_eq!((a, b), (0, 1));
        f.link_uart(0, "usart1", 1, "usart1");
        // Park both on a BKPT: steps report the halt, the pump moves no
        // traffic (no such devices: endpoints decline), and stepping a
        // halted machine stays harmless.
        let out = f.step();
        assert_eq!(out.len(), 2);
        assert!(out.iter().all(|r| r.is_err()));
        let out = f.step();
        assert!(out.iter().all(|r| r.is_err()));
        assert!(f.machines().len() == 2);
    }
}
