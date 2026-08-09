//! The generic virtual board: a `Cortex-M` core with flash, RAM, `GPIO`,
//! `UART`, and `SysTick` — the first concrete board from the roadmap.
//!
//! Memory map:
//! - `0x0000_0000` flash (64 KiB)
//! - `0x2000_0000` RAM (64 KiB)
//! - `0x4000_0000` GPIO
//! - `0x4000_1000` UART (transmit)
//! - `0xE000_E000` NVIC / `0xE000_E010` `SysTick`

use crate::device::gpio::Gpio;
use crate::device::systick::SysTick;
use crate::device::uart::UartTx;
use crate::mem::flat::FlatMemory;
use crate::mem::system::SystemMemory;

/// The concrete memory type the virtual board is built from.
pub type BoardMemory = SystemMemory<FlatMemory>;

/// Flash base and size.
pub const FLASH: u32 = 0x0000_0000;
/// Flash size in bytes.
pub const FLASH_LEN: u32 = 0x0001_0000;
/// RAM base.
pub const RAM: u32 = 0x2000_0000;
/// RAM size in bytes.
pub const RAM_LEN: u32 = 0x0001_0000;
/// GPIO port base.
pub const GPIO_BASE: u32 = 0x4000_0000;
/// UART base.
pub const UART_BASE: u32 = 0x4000_1000;

/// A fully-attached virtual-board memory: flash + RAM backing, `GPIO`, `UART`,
/// and `SysTick`, with the `NVIC` wired for whichever device lines assert.
#[must_use]
pub fn virtual_board() -> BoardMemory {
    let backing = FlatMemory::with_layout(&[FLASH, RAM], &[FLASH_LEN, RAM_LEN]);
    let mut mem = SystemMemory::new(backing);
    mem.attach(Box::new(Gpio::at(GPIO_BASE)));
    mem.attach(Box::new(UartTx::at(UART_BASE)));
    mem.attach(Box::new(SysTick::new()));
    mem
}
