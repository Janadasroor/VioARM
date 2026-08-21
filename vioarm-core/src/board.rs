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
use crate::device::stm32::adc::Adc;
use crate::device::stm32::afio::Afio;
use crate::device::stm32::can::Can;
use crate::device::stm32::bkp::Bkp;
use crate::device::stm32::crc::Crc;
use crate::device::stm32::dac::Dac;
use crate::device::stm32::dma::{DmaF1, DmaRequests};
use crate::device::stm32::rtc::Rtc;
use crate::device::stm32::sdio::Sdio;
use crate::device::stm32::dma_f4::DmaF4;
use crate::device::stm32::eth::{Eth, EthMailbox};
use crate::device::stm32::exti::Exti;
use crate::device::stm32::flash::{FlashF1, FlashF4, FlashGate};
use crate::device::stm32::gpio::Gpio as StmGpio;
use crate::device::stm32::gpio_f4::GpioF4;
use crate::device::stm32::i2c::I2c;
use crate::device::stm32::rcc::Rcc;
use crate::device::stm32::spi::Spi;
use crate::device::stm32::syscfg::Syscfg;
use crate::device::stm32::timer::Timer;
use crate::device::stm32::usart::Usart;
use crate::device::stm32::usb::Usb;
use crate::device::stm32::watchdog::{Iwdg, Wwdg};
use crate::device::systick::SysTick;
use crate::device::uart::UartTx;
use crate::mem::bus::Region;
use crate::mem::flat::FlatMemory;
use crate::mem::system::SystemMemory;
use std::sync::{Arc, Mutex};

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

/// A Cortex-M0/M0+ board: the generic virtual-board memory, to be paired with
/// the ARMv6-M core (`Machine::new`, not `Machine::armv7m`).
///
/// This is the explicit ARMv6-M home for the generic peripheral set. The
/// STM32F1/F4 boards reuse the same core and are described elsewhere; here the
/// distinction is spelled out so firmware that must never see Thumb-2 or the
/// v7-M-only `BASEPRI`/`FAULTMASK`/distinct-`UsageFault` facilities has a
/// named home to run against.
#[must_use]
pub fn cortex_m0() -> BoardMemory {
    virtual_board()
}

/// STM32F103 (Blue Pill class) flash base — the `0x0000_0000` boot alias; the
/// main flash region at `0x0800_0000` is modeled separately and gated by the
/// `FLASH` controller.
pub const STM32_F103_FLASH: u32 = FLASH;
/// STM32F103 flash size in bytes (medium density).
pub const STM32_F103_FLASH_LEN: u32 = 0x0001_0000;
/// STM32F103 RAM size in bytes.
pub const STM32_F103_RAM_LEN: u32 = 0x0000_5000;
/// STM32F1 main-flash base (where `FLASH` controller programming lands).
pub const STM32_MAIN_FLASH: u32 = 0x0800_0000;

/// A Blue-Pill-class STM32F103 board: the M0 core with the STM32F1 peripheral
/// set mapped at its real addresses (from `STM32F103xx.svd`).
///
/// Firmware boots from the flash alias at `0x0000_0000`; peripherals are
/// ungated (see the `stm32` module docs). The main flash at `0x0800_0000` is a
/// separate store guarded by the `FLASH` controller's unlock/program gate.
#[must_use]
pub fn stm32f103() -> BoardMemory {
    use crate::device::stm32;
    use crate::device::svd::stm32f103::Stm32f103;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F103_FLASH_LEN,
            STM32_F103_FLASH_LEN,
            STM32_F103_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F103_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    mem.attach(Box::new(FlashF1::new(gate)));
    mem.attach(Box::new(Rcc::new()));
    mem.attach(Box::new(Crc::new_for::<crate::device::svd::stm32f103::Stm32f103>()));
    mem.attach(Box::new(Dac::new_for::<crate::device::svd::stm32f103::Stm32f103>()));
    mem.attach(Box::new(Rtc::new_for::<crate::device::svd::stm32f103::Stm32f103>()));
    mem.attach(Box::new(Bkp::new_for::<crate::device::svd::stm32f103::Stm32f103>()));
    mem.attach(Box::new(Sdio::new_for::<crate::device::svd::stm32f103::Stm32f103>()));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOA, "gpioa")));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOB, "gpiob")));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOC, "gpioc")));
    let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
    mem.attach(Box::new(Usart::with_dma(dma_requests.clone())));
    mem.attach(Box::new(Usart::usart2_for_device_with_dma::<Stm32f103>(
        dma_requests.clone(),
    )));
    mem.attach(Box::new(Usart::usart3_for_device_with_dma::<Stm32f103>(
        dma_requests.clone(),
    )));
    mem.attach(Box::new(Timer::tim2()));
    mem.attach(Box::new(Timer::tim3()));
    mem.attach(Box::new(Timer::tim1()));
    mem.attach(Box::new(Timer::tim4()));
    mem.attach(Box::new(Timer::tim6()));
    mem.attach(Box::new(Timer::tim7()));
    mem.attach(Box::new(Exti::new()));
    mem.attach(Box::new(Afio::new()));
    mem.attach(Box::new(Adc::adc1()));
    mem.attach(Box::new(Adc::adc2()));
    mem.attach(Box::new(Adc::adc3()));
    mem.attach(Box::new(Spi::spi1()));
    mem.attach(Box::new(Spi::spi2()));
    mem.attach(Box::new(Spi::spi3()));
    mem.attach(Box::new(I2c::new()));
    mem.attach(Box::new(I2c::i2c2_for::<Stm32f103>()));
    let dma = Arc::new(Mutex::new(Vec::new()));
    mem.set_dma_mailbox(dma.clone());
    mem.attach(Box::new(DmaF1::dma1(dma, dma_requests)));
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    mem.attach(Box::new(Iwdg::new(watchdog.clone())));
    mem.attach(Box::new(Wwdg::new(watchdog)));
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
/// Ethernet, USB OTG, and a second CAN, of which the USB OTG controller and
/// the second CAN are modeled here, and its `FLASH` controller guards a
/// 256 KiB main-flash region.
#[must_use]
pub fn stm32f107() -> BoardMemory {
    use crate::device::svd::stm32f107::Stm32f107;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F107_FLASH_LEN,
            STM32_F107_FLASH_LEN,
            STM32_F107_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F107_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    mem.attach(Box::new(FlashF1::for_device::<Stm32f107>(gate)));
    mem.attach(Box::new(Rcc::for_device::<Stm32f107>()));
    mem.attach(Box::new(Crc::new_for::<Stm32f107>()));
    mem.attach(Box::new(Dac::new_for::<Stm32f107>()));
    mem.attach(Box::new(Rtc::new_for::<Stm32f107>()));
    mem.attach(Box::new(Bkp::new_for::<Stm32f107>()));
    mem.attach(Box::new(Sdio::new_for::<Stm32f107>()));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_0800, "gpioa")));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_0C00, "gpiob")));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_1000, "gpioc")));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_1400, "gpiod")));
    mem.attach(Box::new(StmGpio::at_in::<Stm32f107>(0x4001_1800, "gpioe")));
    mem.attach(Box::new(Usart::for_device::<Stm32f107>()));
    mem.attach(Box::new(Usart::usart2_for::<Stm32f107>()));
    mem.attach(Box::new(Usart::usart3_for::<Stm32f107>()));
    mem.attach(Box::new(Usart::uart4_for::<Stm32f107>()));
    mem.attach(Box::new(Usart::uart5_for::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim2_for::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim3_for::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim1_for::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim4_for::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim5_for::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim6_for::<Stm32f107>()));
    mem.attach(Box::new(Timer::tim7_for::<Stm32f107>()));
    mem.attach(Box::new(Exti::for_device::<Stm32f107>()));
    mem.attach(Box::new(Afio::for_device::<Stm32f107>()));
    mem.attach(Box::new(Adc::adc1_for::<Stm32f107>()));
    mem.attach(Box::new(Adc::adc2_for::<Stm32f107>()));
    mem.attach(Box::new(Spi::spi1_for::<Stm32f107>()));
    mem.attach(Box::new(Spi::spi2_for::<Stm32f107>()));
    mem.attach(Box::new(Spi::spi3_for::<Stm32f107>()));
    mem.attach(Box::new(I2c::for_device::<Stm32f107>()));
    mem.attach(Box::new(I2c::i2c2_for::<Stm32f107>()));
    mem.attach(Box::new(Can::can1_for::<Stm32f107>()));
    mem.attach(Box::new(Can::can2_for::<Stm32f107>()));
    mem.attach(Box::new(Usb::otg_fs_for::<Stm32f107>()));
    let eth_mailbox = Arc::new(Mutex::new(EthMailbox::default()));
    mem.set_eth_mailbox(eth_mailbox.clone());
    mem.attach(Box::new(Eth::for_device::<Stm32f107>(eth_mailbox)));
    let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
    mem.attach(Box::new(Usart::for_device_with_dma::<Stm32f107>(
        dma_requests.clone(),
    )));
    mem.attach(Box::new(Usart::usart2_for_device_with_dma::<Stm32f107>(
        dma_requests.clone(),
    )));
    mem.attach(Box::new(Usart::usart3_for_device_with_dma::<Stm32f107>(
        dma_requests.clone(),
    )));
    mem.attach(Box::new(Usart::uart4_for_device_with_dma::<Stm32f107>(
        dma_requests.clone(),
    )));
    mem.attach(Box::new(Usart::uart5_for_device_with_dma::<Stm32f107>(
        dma_requests.clone(),
    )));
    let dma = Arc::new(Mutex::new(Vec::new()));
    mem.set_dma_mailbox(dma.clone());
    mem.attach(Box::new(DmaF1::dma_for::<Stm32f107>(
        "DMA1",
        dma.clone(),
        dma_requests.clone(),
    )));
    mem.attach(Box::new(DmaF1::dma_for::<Stm32f107>(
        "DMA2",
        dma,
        dma_requests,
    )));
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    mem.attach(Box::new(Iwdg::for_device::<Stm32f107>(watchdog.clone())));
    mem.attach(Box::new(Wwdg::for_device::<Stm32f107>(watchdog)));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32F407 (Discovery-class) flash base — booted at the `0x0000_0000` alias.
pub const STM32_F407_FLASH: u32 = FLASH;
/// STM32F407 flash size in bytes (1 MiB).
pub const STM32_F407_FLASH_LEN: u32 = 0x0010_0000;
/// STM32F407 main SRAM size in bytes (128 KiB at `0x2000_0000`).
pub const STM32_F407_RAM_LEN: u32 = 0x0002_0000;
/// STM32F407 core-coupled-memory base (64 KiB at `0x1000_0000`).
pub const STM32_F407_CCM: u32 = 0x1000_0000;
/// STM32F407 core-coupled-memory size in bytes.
pub const STM32_F407_CCM_LEN: u32 = 0x0001_0000;

/// An STM32F407 board: the M4 core with the F4 peripheral set mapped at its
/// real addresses (from `STM32F407.svd`).
///
/// GPIO uses the F4 register semantics (`MODER`/`BSRR`/`AFR`, no `CRL`/`CRH`);
/// RCC/EXTI/USART/TIM mount the same chip-agnostic cores as the F1 boards, and
/// `SYSCFG` replaces `AFIO` as the `EXTI` line-router. The main flash at
/// `0x0800_0000` is a gated store guarded by the F4 `FLASH` controller, the
/// 64 KiB CCM sits at `0x1000_0000`, and `DMA1`/`DMA2` use the F4 stream
/// register set.
#[must_use]
pub fn stm32f407() -> BoardMemory {
    use crate::device::svd::stm32f407::Stm32f407;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, STM32_F407_CCM, RAM],
        &[
            STM32_F407_FLASH_LEN,
            STM32_F407_FLASH_LEN,
            STM32_F407_CCM_LEN,
            STM32_F407_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F407_FLASH_LEN),
        16 * 1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    mem.attach(Box::new(FlashF4::for_device::<Stm32f407>(gate)));
    mem.attach(Box::new(Rcc::for_device::<Stm32f407>()));
    mem.attach(Box::new(Crc::new_for::<Stm32f407>()));
    mem.attach(Box::new(Dac::new_for::<Stm32f407>()));
    mem.attach(Box::new(Rtc::new_for::<Stm32f407>()));
    mem.attach(Box::new(Sdio::new_for::<Stm32f407>()));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_0000, "gpioa")));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_0400, "gpiob")));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_0800, "gpioc")));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_0C00, "gpiod")));
    mem.attach(Box::new(GpioF4::at_in::<Stm32f407>(0x4002_1000, "gpioe")));
    mem.attach(Box::new(Usart::for_device::<Stm32f407>()));
    mem.attach(Box::new(Usart::usart2_for::<Stm32f407>()));
    mem.attach(Box::new(Usart::usart3_for::<Stm32f407>()));
    mem.attach(Box::new(Usart::uart4_for::<Stm32f407>()));
    mem.attach(Box::new(Usart::uart5_for::<Stm32f407>()));
    mem.attach(Box::new(Usart::usart6_for::<Stm32f407>()));
    mem.attach(Box::new(Timer::tim2_for::<Stm32f407>()));
    mem.attach(Box::new(Timer::tim1_for::<Stm32f407>()));
    mem.attach(Box::new(Timer::tim4_for::<Stm32f407>()));
    mem.attach(Box::new(Timer::tim5_for::<Stm32f407>()));
    mem.attach(Box::new(Timer::tim6_for::<Stm32f407>()));
    mem.attach(Box::new(Timer::tim7_for::<Stm32f407>()));
    mem.attach(Box::new(Exti::for_device::<Stm32f407>()));
    mem.attach(Box::new(Syscfg::for_device::<Stm32f407>()));
    mem.attach(Box::new(Adc::adc1_for::<Stm32f407>()));
    mem.attach(Box::new(Adc::adc2_for::<Stm32f407>()));
    mem.attach(Box::new(Adc::adc3_for::<Stm32f407>()));
    mem.attach(Box::new(Spi::spi1_for::<Stm32f407>()));
    mem.attach(Box::new(Spi::spi2_for::<Stm32f407>()));
    mem.attach(Box::new(Spi::spi3_for::<Stm32f407>()));
    mem.attach(Box::new(I2c::for_device::<Stm32f407>()));
    mem.attach(Box::new(I2c::i2c2_for::<Stm32f407>()));
    mem.attach(Box::new(I2c::i2c3_for::<Stm32f407>()));
    mem.attach(Box::new(Can::can1_for::<Stm32f407>()));
    mem.attach(Box::new(Can::can2_for::<Stm32f407>()));
    mem.attach(Box::new(Usb::otg_fs_for::<Stm32f407>()));
    mem.attach(Box::new(Usb::otg_hs_for::<Stm32f407>()));
    let eth_mailbox = Arc::new(Mutex::new(EthMailbox::default()));
    mem.set_eth_mailbox(eth_mailbox.clone());
    mem.attach(Box::new(Eth::for_device::<Stm32f407>(eth_mailbox)));
    let dma = Arc::new(Mutex::new(Vec::new()));
    mem.set_dma_mailbox(dma.clone());
    mem.attach(Box::new(DmaF4::dma1_for::<Stm32f407>(dma.clone())));
    mem.attach(Box::new(DmaF4::dma2_for::<Stm32f407>(dma)));
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    mem.attach(Box::new(Iwdg::for_device::<Stm32f407>(watchdog.clone())));
    mem.attach(Box::new(Wwdg::for_device::<Stm32f407>(watchdog)));
    mem.attach(Box::new(SysTick::new()));
    mem
}
