//! The generic virtual board: a `Cortex-M` core with flash, RAM, `GPIO`,
//! `UART`, and `SysTick` — the first concrete board from the roadmap.
//!
//! Memory map:
//! - `0x0000_0000` flash (64 KiB)
//! - `0x2000_0000` RAM (64 KiB)
//! - `0x4000_0000` GPIO
//! - `0x4000_1000` UART (transmit)
//! - `0xE000_E000` NVIC / `0xE000_E010` `SysTick`
//!
//! A second board, [`stm32f103`], models a Blue-Pill-class STM32F103 with its
//! real peripheral map; a third, [`stm32f107`], models a connectivity-line
//! STM32F107 on the shared `STM32F107xx.svd` tables.

use crate::device::gpio::Gpio;
use crate::device::stm32::afio::Afio;
use crate::device::stm32::exti::Exti;
use crate::device::stm32::gpio::Gpio as StmGpio;
use crate::device::stm32::gpio_f4::GpioF4;
use crate::device::stm32::rcc::Rcc;
use crate::device::stm32::syscfg::Syscfg;
use crate::device::stm32::timer::Timer;
use crate::device::stm32::usart::Usart1;
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

/// STM32F103 (Blue Pill class) flash base — the `0x0800_0000` alias is not yet
/// modeled; firmware is loaded at the `0x0000_0000` boot alias.
pub const STM32_F103_FLASH: u32 = FLASH;
/// STM32F103 flash size in bytes (medium density).
pub const STM32_F103_FLASH_LEN: u32 = 0x0001_0000;
/// STM32F103 RAM size in bytes.
pub const STM32_F103_RAM_LEN: u32 = 0x0000_5000;

/// A Blue-Pill-class STM32F103 board: the M0 core with the STM32F1 peripheral
/// set mapped at its real addresses (from `STM32F103xx.svd`).
///
/// Firmware boots from the flash alias at `0x0000_0000`; peripherals are
/// ungated (see the `stm32` module docs).
#[must_use]
pub fn stm32f103() -> BoardMemory {
    use crate::device::stm32;
    let backing =
        FlatMemory::with_layout(&[FLASH, RAM], &[STM32_F103_FLASH_LEN, STM32_F103_RAM_LEN]);
    let mut mem = SystemMemory::new(backing);
    mem.attach(Box::new(Rcc::new()));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOA, "gpioa")));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOB, "gpiob")));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOC, "gpioc")));
    mem.attach(Box::new(Usart1::new()));
    mem.attach(Box::new(Timer::tim2()));
    mem.attach(Box::new(Timer::tim3()));
    mem.attach(Box::new(Exti::new()));
    mem.attach(Box::new(Afio::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32F107 (connectivity line) flash size in bytes.
pub const STM32_F107_FLASH_LEN: u32 = 0x0004_0000;
/// STM32F107 RAM size in bytes.
pub const STM32_F107_RAM_LEN: u32 = 0x0001_0000;

/// A connectivity-line STM32F107 board: the M0 core with the STM32F1
/// peripheral set mapped at its real addresses (from `STM32F107xx.svd`).
///
/// The shared cores mount the same behaviors as the F103 board; the F107 adds
/// Ethernet, USB OTG, and a second CAN, which the model does not yet expose.
#[must_use]
pub fn stm32f107() -> BoardMemory {
    use crate::device::svd::stm32f107::Stm32f107;
    let backing =
        FlatMemory::with_layout(&[FLASH, RAM], &[STM32_F107_FLASH_LEN, STM32_F107_RAM_LEN]);
    let mut mem = SystemMemory::new(backing);
    mem.attach(Box::new(Rcc::for_device::<Stm32f107>()));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_0800, "gpioa")));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_0C00, "gpiob")));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_1000, "gpioc")));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_1400, "gpiod")));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_1800, "gpioe")));
    mem.attach(Box::new(Usart1::for_device::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim2_for::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim3_for::<Stm32f107>()));
    mem.attach(Box::new(Exti::for_device::<Stm32f107>()));
    mem.attach(Box::new(Afio::for_device::<Stm32f107>()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32F407 (Discovery-class) flash base — booted at the `0x0000_0000` alias.
pub const STM32_F407_FLASH: u32 = FLASH;
/// STM32F407 flash size in bytes (1 MiB).
pub const STM32_F407_FLASH_LEN: u32 = 0x0010_0000;
/// STM32F407 main SRAM size in bytes (128 KiB at `0x2000_0000`; the 64 KiB CCM
/// at `0x1000_0000` is not modeled).
pub const STM32_F407_RAM_LEN: u32 = 0x0002_0000;

/// An STM32F407 board: the M0 core with the F4 peripheral set mapped at its
/// real addresses (from `STM32F407.svd`).
///
/// GPIO uses the F4 register semantics (`MODER`/`BSRR`/`AFR`, no `CRL`/`CRH`);
/// RCC/EXTI/USART/TIM mount the same chip-agnostic cores as the F1 boards, and
/// `SYSCFG` replaces `AFIO` as the `EXTI` line-router.
#[must_use]
pub fn stm32f407() -> BoardMemory {
    use crate::device::svd::stm32f407::Stm32f407;
    let backing =
        FlatMemory::with_layout(&[FLASH, RAM], &[STM32_F407_FLASH_LEN, STM32_F407_RAM_LEN]);
    let mut mem = SystemMemory::new(backing);
    mem.attach(Box::new(Rcc::for_device::<Stm32f407>()));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_0000, "gpioa")));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_0400, "gpiob")));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_0800, "gpioc")));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_0C00, "gpiod")));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_1000, "gpioe")));
    mem.attach(Box::new(Usart1::for_device::<Stm32f407>()));
    mem.attach(Box::new(Timer::tim2_for::<Stm32f407>()));
    mem.attach(Box::new(Exti::for_device::<Stm32f407>()));
    mem.attach(Box::new(Syscfg::for_device::<Stm32f407>()));
    mem.attach(Box::new(SysTick::new()));
    mem
}
