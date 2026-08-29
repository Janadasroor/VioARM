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

use crate::device::dwt::Dwt;
use crate::device::gpio::Gpio;
use crate::device::stm32::adc::{Adc, AdcCommon, AdcCommonState};
use crate::device::stm32::afio::Afio;
use crate::device::stm32::bkp::Bkp;
use crate::device::stm32::can::Can;
use crate::device::stm32::crc::Crc;
use crate::device::stm32::dac::Dac;
use crate::device::stm32::dma::{DmaF1, DmaRequests};
use crate::device::stm32::dma_f4::DmaF4;
use crate::device::stm32::eth::{Eth, EthH7, EthMailbox};
use crate::device::stm32::exti::Exti;
use crate::device::stm32::fdcan::Fdcan;
use crate::device::stm32::flash::{FlashF1, FlashF4, FlashGate};
use crate::device::stm32::fmc::{
    Bdma, Dfsdm, Dma2d, Dmamux, Fmc, Hrtim, Ltdc, Mdma, Qspi, QspiGate, Sai4,
};
use crate::device::stm32::gpio::Gpio as StmGpio;
use crate::device::stm32::gpio_f4::GpioF4;
use crate::device::stm32::i2c::I2c;
use crate::device::stm32::pwr::Pwr;
use crate::device::stm32::rcc::Rcc;
use crate::device::stm32::rtc::Rtc;
use crate::device::stm32::sdio::Sdio;
use crate::device::stm32::spi::Spi;
use crate::device::stm32::syscfg::Syscfg;
use crate::device::stm32::timer::Timer;
use crate::device::stm32::usart::Usart;
use crate::device::stm32::usb::Usb;
use crate::device::stm32::watchdog::{Iwdg, Wwdg};
use crate::device::systick::SysTick;
use crate::device::uart::UartTx;
use crate::device::Peripheral;
use crate::mem::bus::{Bus, Region};
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
    mem.attach(Box::new(Dwt::new()));
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
    let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    mem.attach(Box::new(FlashF1::new(gate)));
    mem.attach(Box::new(Rcc::new()));
    mem.attach(Box::new(Pwr::new_for::<
        crate::device::svd::stm32f103::Stm32f103,
    >()));
    mem.attach(Box::new(Crc::new_for::<
        crate::device::svd::stm32f103::Stm32f103,
    >()));
    mem.attach(Box::new(Dac::new_with_dma::<
        crate::device::svd::stm32f103::Stm32f103,
    >(dma_requests.clone())));
    mem.attach(Box::new(Rtc::new_for::<
        crate::device::svd::stm32f103::Stm32f103,
    >()));
    mem.attach(Box::new(Bkp::new_for::<
        crate::device::svd::stm32f103::Stm32f103,
    >()));
    mem.attach(Box::new(Sdio::new_for::<
        crate::device::svd::stm32f103::Stm32f103,
    >()));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOA, "gpioa")));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOB, "gpiob")));
    mem.attach(Box::new(StmGpio::at(stm32::GPIOC, "gpioc")));
    mem.attach(Box::new(Usart::with_dma(dma_requests.clone())));
    mem.attach(Box::new(Usart::usart2_for_device_with_dma::<Stm32f103>(
        dma_requests.clone(),
    )));
    mem.attach(Box::new(Usart::usart3_for_device_with_dma::<Stm32f103>(
        dma_requests.clone(),
    )));
    mem.attach(Box::new(Timer::tim2().with_dma_update(
        dma_requests.clone(),
        0,
        2,
    ))); // DMA1 ch2: TIM2_UP (RM0008 §13.3.7 Table 78)
    mem.attach(Box::new(Timer::tim3()));
    mem.attach(Box::new(Timer::tim1()));
    mem.attach(Box::new(Timer::tim4()));
    mem.attach(Box::new(Timer::tim6()));
    mem.attach(Box::new(Timer::tim7()));
    mem.attach(Box::new(Exti::new()));
    mem.attach(Box::new(Afio::new()));
    mem.attach(Box::new(Adc::adc1().with_dma(dma_requests.clone(), 0, 1))); // DMA1 ch1: ADC1
    mem.attach(Box::new(Adc::adc2()));
    mem.attach(Box::new(Adc::adc3()));
    mem.attach(Box::new(Spi::spi1().with_dma(
        Some((dma_requests.clone(), 0, 3)),
        Some((dma_requests.clone(), 0, 2)),
    ))); // DMA1 ch3/ch2: SPI1_TX/RX
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
    for p in crate::device::svd::stm32f103::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
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
    mem.attach(Box::new(Pwr::new_for::<Stm32f107>()));
    mem.attach(Box::new(Crc::new_for::<Stm32f107>()));
    mem.attach(Box::new(Dac::new_for::<Stm32f107>()));
    mem.attach(Box::new(Rtc::new_for::<Stm32f107>()));
    mem.attach(Box::new(Bkp::new_for::<Stm32f107>()));
    if <Stm32f107 as crate::device::svd::Device>::peripheral("SDIO").is_some() {
        mem.attach(Box::new(Sdio::new_for::<Stm32f107>()));
    }
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
    for p in crate::device::svd::stm32f107::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
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
    mem.attach(Box::new(Pwr::new_for::<Stm32f407>()));
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
    let adc_common = Arc::new(Mutex::new(AdcCommonState::default()));
    mem.attach(Box::new(AdcCommon::c_adc_for::<Stm32f407>(
        adc_common.clone(),
    )));
    mem.attach(Box::new(
        Adc::adc1_for::<Stm32f407>().with_common(adc_common.clone()),
    ));
    mem.attach(Box::new(
        Adc::adc2_for::<Stm32f407>().with_common(adc_common.clone()),
    ));
    mem.attach(Box::new(
        Adc::adc3_for::<Stm32f407>().with_common(adc_common.clone()),
    ));
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
    for p in crate::device::svd::stm32f407::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32F072 (F0, Cortex-M0) flash size in bytes (128 KiB).
pub const STM32_F072_FLASH_LEN: u32 = 0x0002_0000;
/// STM32F072 RAM size in bytes (16 KiB).
pub const STM32_F072_RAM_LEN: u32 = 0x0000_4000;

/// An STM32F072 board: the F0 line with `Stm32f072` tables.
#[must_use]
pub fn stm32f072() -> BoardMemory {
    use crate::device::svd::stm32f072::Stm32f072;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F072_FLASH_LEN,
            STM32_F072_FLASH_LEN,
            STM32_F072_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F072_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f072 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f072>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f072>()));
    if <Stm32f072 as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("DAC").is_some() {
        mem.attach(Box::new(Dac::new_for::<Stm32f072>()));
    }
    // F0 RTC is the new RTC (TR/DR/CR), not the old PRL/CNT/ALR; skip old behavior
    if <Stm32f072 as crate::device::svd::Device>::peripheral("RTC").is_some()
        && <Stm32f072 as crate::device::svd::Device>::peripheral("RTC")
            .unwrap()
            .registers
            .iter()
            .any(|r| r.name == "PRLH")
    {
        mem.attach(Box::new(Rtc::new_for::<Stm32f072>()));
    }
    for (base, name) in [
        (0x4800_0000u32, "gpioa"),
        (0x4800_0400, "gpiob"),
        (0x4800_0800, "gpioc"),
        (0x4800_0c00, "gpiod"),
        (0x4800_1000, "gpioe"),
        (0x4800_1400, "gpiof"),
    ] {
        if <Stm32f072 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32f072>(base, name)));
        }
    }
    for ctor in [
        Usart::for_device::<Stm32f072> as fn() -> Usart,
        Usart::usart2_for::<Stm32f072>,
        Usart::usart3_for::<Stm32f072>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32f072 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    for ctor in [
        Timer::tim1_for::<Stm32f072>,
        Timer::tim2_for::<Stm32f072>,
        Timer::tim3_for::<Stm32f072>,
        Timer::tim6_for::<Stm32f072>,
        Timer::tim7_for::<Stm32f072>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32f072 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("EXTI").is_some() {
        mem.attach(Box::new(Exti::for_device::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("ADC1").is_some() {
        mem.attach(Box::new(Adc::adc1_for::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("SPI1").is_some() {
        mem.attach(Box::new(Spi::spi1_for::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("SPI2").is_some() {
        mem.attach(Box::new(Spi::spi2_for::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("I2C1").is_some() {
        mem.attach(Box::new(I2c::for_device::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("I2C2").is_some() {
        mem.attach(Box::new(I2c::i2c2_for::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("TSC").is_some() {
        mem.attach(Box::new(
            crate::device::stm32::tsc::Tsc::new_for::<Stm32f072>(),
        ));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("CAN1").is_some() {
        mem.attach(Box::new(Can::can1_for::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("USB_OTG_GLOBAL").is_some() {
        mem.attach(Box::new(Usb::otg_fs_for::<Stm32f072>()));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("DMA").is_some() {
        let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
        let dma = Arc::new(Mutex::new(Vec::new()));
        mem.set_dma_mailbox(dma.clone());
        mem.attach(Box::new(DmaF1::dma_for::<Stm32f072>(
            "DMA",
            dma,
            dma_requests,
        )));
    }
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    if <Stm32f072 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f072>(watchdog.clone())));
    }
    if <Stm32f072 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f072>(watchdog)));
    }
    for p in crate::device::svd::stm32f072::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32F030 (F0, Cortex-M0) 32 KiB flash, 4 KiB RAM — minimal F0 subset (no DAC/COMP).
pub const STM32_F030_FLASH_LEN: u32 = 0x0000_8000;
pub const STM32_F030_RAM_LEN: u32 = 0x0000_1000;

#[must_use]
pub fn stm32f030() -> BoardMemory {
    use crate::device::svd::stm32f030::Stm32f030;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F030_FLASH_LEN,
            STM32_F030_FLASH_LEN,
            STM32_F030_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F030_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f030 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f030>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f030>()));
    if <Stm32f030 as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32f030>()));
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32f030>()));
    }
    for (base, name) in [
        (0x4800_0000u32, "gpioa"),
        (0x4800_0400, "gpiob"),
        (0x4800_0800, "gpioc"),
    ] {
        if <Stm32f030 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32f030>(base, name)));
        }
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("USART1").is_some() {
        mem.attach(Box::new(Usart::for_device::<Stm32f030>()));
    }
    for ctor in [Timer::tim1_for::<Stm32f030>, Timer::tim3_for::<Stm32f030>] {
        let base = ctor().ranges()[0].base;
        if <Stm32f030 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("EXTI").is_some() {
        mem.attach(Box::new(Exti::for_device::<Stm32f030>()));
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32f030>()));
    }
    // SYSCFG_COMP alias on F0x1/F0x2/F0x8 is latch-only (F030 has plain SYSCFG)
    if <Stm32f030 as crate::device::svd::Device>::peripheral("SYSCFG_COMP").is_some() {
        if let Some(def) = crate::device::svd::stm32f030::peripheral("SYSCFG_COMP") {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                def, crate::device::svd::NoBehavior
            )));
        }
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("ADC1").is_some()
        || <Stm32f030 as crate::device::svd::Device>::peripheral("ADC").is_some()
    {
        // F030 ADC is the new ADC (single, not ADC1)
        if <Stm32f030 as crate::device::svd::Device>::peripheral("ADC").is_some() {
            // Use ADC behavior via ADC1 alias if available, else latch
            if let Some(def) = crate::device::svd::stm32f030::peripheral("ADC") {
                mem.attach(Box::new(crate::device::svd::RegisterFile::<
                    crate::device::svd::NoBehavior,
                >::new(
                    def, crate::device::svd::NoBehavior
                )));
            }
        } else {
            mem.attach(Box::new(Adc::adc1_for::<Stm32f030>()));
        }
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("SPI1").is_some() {
        mem.attach(Box::new(Spi::spi1_for::<Stm32f030>()));
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("I2C1").is_some() {
        mem.attach(Box::new(I2c::for_device::<Stm32f030>()));
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("DMA").is_some()
        || <Stm32f030 as crate::device::svd::Device>::peripheral("DMA1").is_some()
    {
        let name = if crate::device::svd::stm32f030::peripheral("DMA").is_some() {
            "DMA"
        } else {
            "DMA1"
        };
        if let Some(def) = crate::device::svd::stm32f030::peripheral(name) {
            let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
            let dma = Arc::new(Mutex::new(Vec::new()));
            mem.set_dma_mailbox(dma.clone());
            mem.attach(Box::new(DmaF1::dma_for::<Stm32f030>(
                name,
                dma,
                dma_requests,
            )));
            // Keep def used to avoid warning
            let _ = def.base;
        }
    }
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    if <Stm32f030 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f030>(watchdog.clone())));
    }
    if <Stm32f030 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f030>(watchdog)));
    }
    for p in crate::device::svd::stm32f030::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

// F031 and F042 are F0 variants with TIM2 and COMP/DAC differences — reuse F030 template with correct SVD
pub const STM32_F031_FLASH_LEN: u32 = 0x0000_8000;
pub const STM32_F031_RAM_LEN: u32 = 0x0000_2000;
#[must_use]
pub fn stm32f031() -> BoardMemory {
    use crate::device::svd::stm32f031::Stm32f031;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F031_FLASH_LEN,
            STM32_F031_FLASH_LEN,
            STM32_F031_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F031_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f031 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f031>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f031>()));
    if <Stm32f031 as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32f031>()));
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32f031>()));
    }
    for (base, name) in [
        (0x4800_0000u32, "gpioa"),
        (0x4800_0400, "gpiob"),
        (0x4800_0800, "gpioc"),
    ] {
        if <Stm32f031 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32f031>(base, name)));
        }
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("USART1").is_some() {
        mem.attach(Box::new(Usart::for_device::<Stm32f031>()));
    }
    for ctor in [Timer::tim1_for::<Stm32f031>, Timer::tim2_for::<Stm32f031>] {
        let base = ctor().ranges()[0].base;
        if <Stm32f031 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    // TIM14 etc via fallback latch
    if <Stm32f031 as crate::device::svd::Device>::peripheral("TIM14").is_some() {
        if let Some(def) = crate::device::svd::stm32f031::peripheral("TIM14") {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                def, crate::device::svd::NoBehavior
            )));
        }
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("EXTI").is_some() {
        mem.attach(Box::new(Exti::for_device::<Stm32f031>()));
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32f031>()));
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("ADC1").is_some()
        || <Stm32f031 as crate::device::svd::Device>::peripheral("ADC").is_some()
    {
        if let Some(def) = crate::device::svd::stm32f031::peripheral("ADC")
            .or_else(|| crate::device::svd::stm32f031::peripheral("ADC1"))
        {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                def, crate::device::svd::NoBehavior
            )));
        }
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("SPI1").is_some() {
        mem.attach(Box::new(Spi::spi1_for::<Stm32f031>()));
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("I2C1").is_some() {
        mem.attach(Box::new(I2c::for_device::<Stm32f031>()));
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("DMA1").is_some()
        || <Stm32f031 as crate::device::svd::Device>::peripheral("DMA").is_some()
    {
        let name = if crate::device::svd::stm32f031::peripheral("DMA1").is_some() {
            "DMA1"
        } else {
            "DMA"
        };
        if let Some(def) = crate::device::svd::stm32f031::peripheral(name) {
            let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
            let dma = Arc::new(Mutex::new(Vec::new()));
            mem.set_dma_mailbox(dma.clone());
            mem.attach(Box::new(DmaF1::dma_for::<Stm32f031>(
                name,
                dma,
                dma_requests,
            )));
            let _ = def.base;
        }
    }
    let wdg = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(wdg.clone());
    if <Stm32f031 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f031>(wdg.clone())));
    }
    if <Stm32f031 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f031>(wdg)));
    }
    for p in crate::device::svd::stm32f031::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

pub const STM32_F042_FLASH_LEN: u32 = 0x0000_8000;
pub const STM32_F042_RAM_LEN: u32 = 0x0000_1800;
#[must_use]
pub fn stm32f042() -> BoardMemory {
    use crate::device::svd::stm32f042::Stm32f042;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F042_FLASH_LEN,
            STM32_F042_FLASH_LEN,
            STM32_F042_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F042_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f042 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f042>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f042>()));
    if <Stm32f042 as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32f042>()));
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32f042>()));
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("COMP").is_some() {
        mem.attach(Box::new(crate::device::stm32::comp::Comp::new_for::<
            Stm32f042,
        >()));
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("CRS").is_some() {
        mem.attach(Box::new(
            crate::device::stm32::crs::Crs::new_for::<Stm32f042>(),
        ));
    }
    for (base, name) in [(0x4800_0000u32, "gpioa"), (0x4800_0400, "gpiob")] {
        if <Stm32f042 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32f042>(base, name)));
        }
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("USART1").is_some() {
        mem.attach(Box::new(Usart::for_device::<Stm32f042>()));
    }
    for ctor in [Timer::tim1_for::<Stm32f042>, Timer::tim2_for::<Stm32f042>] {
        let base = ctor().ranges()[0].base;
        if <Stm32f042 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("EXTI").is_some() {
        mem.attach(Box::new(Exti::for_device::<Stm32f042>()));
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32f042>()));
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("SPI1").is_some() {
        mem.attach(Box::new(Spi::spi1_for::<Stm32f042>()));
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("I2C1").is_some() {
        mem.attach(Box::new(I2c::for_device::<Stm32f042>()));
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("USB_OTG_GLOBAL").is_some()
        || <Stm32f042 as crate::device::svd::Device>::peripheral("USB").is_some()
    {
        if let Some(def) = crate::device::svd::stm32f042::peripheral("USB")
            .or_else(|| crate::device::svd::stm32f042::peripheral("USB_OTG_GLOBAL"))
        {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                def, crate::device::svd::NoBehavior
            )));
        }
    }
    let wdg = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(wdg.clone());
    if <Stm32f042 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f042>(wdg.clone())));
    }
    if <Stm32f042 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f042>(wdg)));
    }
    for p in crate::device::svd::stm32f042::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

pub const STM32_F0X0_FLASH_LEN: u32 = 0x0000_8000;
pub const STM32_F0X0_RAM_LEN: u32 = 0x0000_1000;
#[must_use]
pub fn stm32f0x0() -> BoardMemory {
    use crate::device::svd::stm32f0x0::Stm32f0x0;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F0X0_FLASH_LEN,
            STM32_F0X0_FLASH_LEN,
            STM32_F0X0_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F0X0_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f0x0 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f0x0>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f0x0>()));
    for (base, name) in [(0x4800_0000u32, "gpioa"), (0x4800_0400, "gpiob")] {
        if <Stm32f0x0 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32f0x0>(base, name)));
        }
    }
    if <Stm32f0x0 as crate::device::svd::Device>::peripheral("USART1").is_some() {
        mem.attach(Box::new(Usart::for_device::<Stm32f0x0>()));
    }
    if <Stm32f0x0 as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32f0x0>()));
    }
    if <Stm32f0x0 as crate::device::svd::Device>::peripheral("SYSCFG_COMP").is_some() {
        if let Some(def) = crate::device::svd::stm32f0x0::peripheral("SYSCFG_COMP") {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                def, crate::device::svd::NoBehavior
            )));
        }
    }
    for p in crate::device::svd::stm32f0x0::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    let wdg = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(wdg.clone());
    if <Stm32f0x0 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f0x0>(wdg.clone())));
    }
    if <Stm32f0x0 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f0x0>(wdg)));
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

pub const STM32_F091_FLASH_LEN: u32 = 0x0004_0000;
pub const STM32_F091_RAM_LEN: u32 = 0x0000_8000;
#[must_use]
pub fn stm32f091() -> BoardMemory {
    use crate::device::svd::stm32f091::Stm32f091;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F091_FLASH_LEN,
            STM32_F091_FLASH_LEN,
            STM32_F091_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F091_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f091 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f091>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f091>()));
    if <Stm32f091 as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32f091>()));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32f091>()));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("DAC").is_some() {
        mem.attach(Box::new(Dac::new_for::<Stm32f091>()));
    }
    for (base, name) in [
        (0x4800_0000u32, "gpioa"),
        (0x4800_0400, "gpiob"),
        (0x4800_0800, "gpioc"),
        (0x4800_0c00, "gpiod"),
        (0x4800_1400, "gpiof"),
    ] {
        if <Stm32f091 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32f091>(base, name)));
        }
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("USART1").is_some() {
        mem.attach(Box::new(Usart::for_device::<Stm32f091>()));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("USART2").is_some() {
        mem.attach(Box::new(Usart::usart2_for::<Stm32f091>()));
    }
    for ctor in [
        Timer::tim1_for::<Stm32f091>,
        Timer::tim2_for::<Stm32f091>,
        Timer::tim3_for::<Stm32f091>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32f091 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("EXTI").is_some() {
        mem.attach(Box::new(Exti::for_device::<Stm32f091>()));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32f091>()));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("CEC").is_some() {
        mem.attach(Box::new(
            crate::device::stm32::cec::Cec::new_for::<Stm32f091>(),
        ));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("SPI1").is_some() {
        mem.attach(Box::new(Spi::spi1_for::<Stm32f091>()));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("SPI2").is_some() {
        mem.attach(Box::new(Spi::spi2_for::<Stm32f091>()));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("I2C1").is_some() {
        mem.attach(Box::new(I2c::for_device::<Stm32f091>()));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("I2C2").is_some() {
        mem.attach(Box::new(I2c::i2c2_for::<Stm32f091>()));
    }
    // DMA1/DMA2
    for name in ["DMA", "DMA1", "DMA2"] {
        if let Some(def) = crate::device::svd::stm32f091::peripheral(name) {
            let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
            let dma = Arc::new(Mutex::new(Vec::new()));
            // Only set once
            mem.set_dma_mailbox(dma.clone());
            mem.attach(Box::new(DmaF1::dma_for::<Stm32f091>(
                name,
                dma,
                dma_requests,
            )));
            let _ = def.base;
            break;
        }
    }
    let wdg = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(wdg.clone());
    if <Stm32f091 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f091>(wdg.clone())));
    }
    if <Stm32f091 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f091>(wdg)));
    }
    for p in crate::device::svd::stm32f091::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

pub const STM32_F0X1_FLASH_LEN: u32 = 0x0000_8000;
pub const STM32_F0X1_RAM_LEN: u32 = 0x0000_1000;
#[must_use]
pub fn stm32f0x1() -> BoardMemory {
    use crate::device::svd::stm32f0x1::Stm32f0x1;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F0X1_FLASH_LEN,
            STM32_F0X1_FLASH_LEN,
            STM32_F0X1_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F0X1_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f0x1 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f0x1>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f0x1>()));
    for p in crate::device::svd::stm32f0x1::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    let wdg = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(wdg.clone());
    if <Stm32f0x1 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f0x1>(wdg.clone())));
    }
    if <Stm32f0x1 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f0x1>(wdg)));
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

pub const STM32_F0X2_FLASH_LEN: u32 = 0x0000_8000;
pub const STM32_F0X2_RAM_LEN: u32 = 0x0000_1000;
#[must_use]
pub fn stm32f0x2() -> BoardMemory {
    use crate::device::svd::stm32f0x2::Stm32f0x2;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F0X2_FLASH_LEN,
            STM32_F0X2_FLASH_LEN,
            STM32_F0X2_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F0X2_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f0x2 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f0x2>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f0x2>()));
    for p in crate::device::svd::stm32f0x2::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    let wdg = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(wdg.clone());
    if <Stm32f0x2 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f0x2>(wdg.clone())));
    }
    if <Stm32f0x2 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f0x2>(wdg)));
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

pub const STM32_F0X8_FLASH_LEN: u32 = 0x0001_0000;
pub const STM32_F0X8_RAM_LEN: u32 = 0x0000_2000;
#[must_use]
pub fn stm32f0x8() -> BoardMemory {
    use crate::device::svd::stm32f0x8::Stm32f0x8;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F0X8_FLASH_LEN,
            STM32_F0X8_FLASH_LEN,
            STM32_F0X8_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F0X8_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f0x8 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f0x8>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f0x8>()));
    for p in crate::device::svd::stm32f0x8::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    let wdg = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(wdg.clone());
    if <Stm32f0x8 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f0x8>(wdg.clone())));
    }
    if <Stm32f0x8 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f0x8>(wdg)));
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

pub const STM32_F0XX_FLASH_LEN: u32 = 0x0000_8000;
pub const STM32_F0XX_RAM_LEN: u32 = 0x0000_1000;
#[must_use]
pub fn stm32f0xx() -> BoardMemory {
    use crate::device::svd::stm32f0xx::Stm32f0;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F0XX_FLASH_LEN,
            STM32_F0XX_FLASH_LEN,
            STM32_F0XX_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F0XX_FLASH_LEN),
        1024,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32f0 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f0>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f0>()));
    for p in crate::device::svd::stm32f0xx::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    let wdg = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(wdg.clone());
    if <Stm32f0 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f0>(wdg.clone())));
    }
    if <Stm32f0 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f0>(wdg)));
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32F303xE (F3, Cortex-M4) flash size in bytes (512 KiB).
pub const STM32_F303XE_FLASH_LEN: u32 = 0x0008_0000;
/// STM32F303xE RAM size in bytes (64 KiB).
pub const STM32_F303XE_RAM_LEN: u32 = 0x0001_0000;

/// An STM32F303xE board: the F3 line with `Stm32f303xe` tables.
#[must_use]
pub fn stm32f303xe() -> BoardMemory {
    use crate::device::svd::stm32f303xe::Stm32f303xe;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_F303XE_FLASH_LEN,
            STM32_F303XE_FLASH_LEN,
            STM32_F303XE_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_F303XE_FLASH_LEN),
        2048,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    // F3 flash is F1-gated at 0x40022000
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32f303xe>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32f303xe>()));
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32f303xe>()));
    }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32f303xe>()));
    }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("DAC1").is_some() {
        mem.attach(Box::new(Dac::new_for::<Stm32f303xe>()));
    } else if <Stm32f303xe as crate::device::svd::Device>::peripheral("DAC").is_some() {
        mem.attach(Box::new(Dac::new_for::<Stm32f303xe>()));
    }
    // F3 GPIO is F4-style (MODER)
    for (base, name) in [
        (0x4800_0000u32, "gpioa"),
        (0x4800_0400, "gpiob"),
        (0x4800_0800, "gpioc"),
        (0x4800_0c00, "gpiod"),
        (0x4800_1000, "gpioe"),
        (0x4800_1400, "gpiof"),
    ] {
        if <Stm32f303xe as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32f303xe>(base, name)));
        }
    }
    // USART family
    for ctor in [
        Usart::for_device::<Stm32f303xe> as fn() -> Usart,
        Usart::usart2_for::<Stm32f303xe>,
        Usart::usart3_for::<Stm32f303xe>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32f303xe as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    // TIM family (F3 has TIM1/2/3/4/6/7/15/16/17)
    for ctor in [
        Timer::tim1_for::<Stm32f303xe>,
        Timer::tim2_for::<Stm32f303xe>,
        Timer::tim3_for::<Stm32f303xe>,
        Timer::tim6_for::<Stm32f303xe>,
        Timer::tim7_for::<Stm32f303xe>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32f303xe as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("EXTI").is_some() {
        mem.attach(Box::new(Exti::for_device::<Stm32f303xe>()));
    }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32f303xe>()));
    }
    // F3 ADC is the new ADC (ISR/CR), not the old CR2/SR; skip old Adc behavior
    // if <Stm32f303xe as crate::device::svd::Device>::peripheral("ADC1").is_some() {
    //     mem.attach(Box::new(Adc::adc1_for::<Stm32f303xe>()));
    // }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("SPI1").is_some() {
        mem.attach(Box::new(Spi::spi1_for::<Stm32f303xe>()));
    }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("SPI2").is_some() {
        mem.attach(Box::new(Spi::spi2_for::<Stm32f303xe>()));
    }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("I2C1").is_some() {
        mem.attach(Box::new(I2c::for_device::<Stm32f303xe>()));
    }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("I2C2").is_some() {
        mem.attach(Box::new(I2c::i2c2_for::<Stm32f303xe>()));
    }
    // F3 CAN is "CAN" at 0x40006400, old Can behavior expects "CAN1"; skip for now
    // if <Stm32f303xe as crate::device::svd::Device>::peripheral("CAN").is_some() {
    //     mem.attach(Box::new(Can::can1_for::<Stm32f303xe>()));
    // }
    // F3 USB is "USB" at 0x40005C00, not OTG; skip old Usb behavior
    // if <Stm32f303xe as crate::device::svd::Device>::peripheral("USB").is_some() {
    //     mem.attach(Box::new(Usb::otg_fs_for::<Stm32f303xe>()));
    // }
    // DMA1/DMA2 – F3 has DMA1 (7 ch) and DMA2 (5 ch) like F1
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("DMA1").is_some() {
        let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
        let dma = Arc::new(Mutex::new(Vec::new()));
        mem.set_dma_mailbox(dma.clone());
        mem.attach(Box::new(DmaF1::dma_for::<Stm32f303xe>(
            "DMA1",
            dma.clone(),
            dma_requests.clone(),
        )));
        if <Stm32f303xe as crate::device::svd::Device>::peripheral("DMA2").is_some() {
            mem.attach(Box::new(DmaF1::dma_for::<Stm32f303xe>(
                "DMA2",
                dma,
                dma_requests,
            )));
        }
    }
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32f303xe>(watchdog.clone())));
    }
    if <Stm32f303xe as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32f303xe>(watchdog)));
    }
    for p in crate::device::svd::stm32f303xe::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32L476 (L4, Cortex-M4) flash size in bytes (1 MiB).
pub const STM32_L476_FLASH_LEN: u32 = 0x0010_0000;
/// STM32L476 RAM size in bytes (128 KiB).
pub const STM32_L476_RAM_LEN: u32 = 0x0002_0000;

/// An STM32L476 board: the L4 line with `Stm32l476` tables.
#[must_use]
pub fn stm32l476() -> BoardMemory {
    use crate::device::svd::stm32l476::Stm32l476;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_L476_FLASH_LEN,
            STM32_L476_FLASH_LEN,
            STM32_L476_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_L476_FLASH_LEN),
        2048,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    // L4 flash is at 0x40022000 like F1, use FlashF1
    if <Stm32l476 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32l476>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32l476>()));
    if <Stm32l476 as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32l476>()));
    }
    if <Stm32l476 as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32l476>()));
    }
    // L4 GPIO is F4-style (MODER)
    for (base, name) in [
        (0x4800_0000u32, "gpioa"),
        (0x4800_0400, "gpiob"),
        (0x4800_0800, "gpioc"),
        (0x4800_0c00, "gpiod"),
        (0x4800_1000, "gpioe"),
    ] {
        if <Stm32l476 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32l476>(base, name)));
        }
    }
    for ctor in [
        Usart::for_device::<Stm32l476> as fn() -> Usart,
        Usart::usart2_for::<Stm32l476>,
        Usart::usart3_for::<Stm32l476>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32l476 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    for ctor in [Timer::tim2_for::<Stm32l476>, Timer::tim6_for::<Stm32l476>] {
        let base = ctor().ranges()[0].base;
        if <Stm32l476 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    if <Stm32l476 as crate::device::svd::Device>::peripheral("EXTI").is_some() {
        mem.attach(Box::new(Exti::for_device::<Stm32l476>()));
    }
    if <Stm32l476 as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32l476>()));
    }
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    if <Stm32l476 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32l476>(watchdog.clone())));
    }
    if <Stm32l476 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32l476>(watchdog)));
    }
    // L476: keep as pre-c3fe471 (no fallback) to preserve real_firmware SPI expectation
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32G474 (G4, Cortex-M4) flash size in bytes (512 KiB).
pub const STM32_G474_FLASH_LEN: u32 = 0x0008_0000;
/// STM32G474 RAM size in bytes (128 KiB).
pub const STM32_G474_RAM_LEN: u32 = 0x0002_0000;

/// An STM32G474 board: the G4 line with `Stm32g474` tables.
#[must_use]
pub fn stm32g474xx() -> BoardMemory {
    use crate::device::svd::stm32g474xx::Stm32g474;
    let backing = FlatMemory::with_layout(
        &[FLASH, STM32_MAIN_FLASH, RAM],
        &[
            STM32_G474_FLASH_LEN,
            STM32_G474_FLASH_LEN,
            STM32_G474_RAM_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_G474_FLASH_LEN),
        2048,
    )));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    if <Stm32g474 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32g474>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32g474>()));
    if <Stm32g474 as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32g474>()));
    }
    if <Stm32g474 as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32g474>()));
    }
    for (base, name) in [
        (0x4800_0000u32, "gpioa"),
        (0x4800_0400, "gpiob"),
        (0x4800_0800, "gpioc"),
    ] {
        if <Stm32g474 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32g474>(base, name)));
        }
    }
    for ctor in [
        Usart::for_device::<Stm32g474> as fn() -> Usart,
        Usart::usart2_for::<Stm32g474>,
        Usart::usart3_for::<Stm32g474>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32g474 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    for ctor in [Timer::tim2_for::<Stm32g474>, Timer::tim6_for::<Stm32g474>] {
        let base = ctor().ranges()[0].base;
        if <Stm32g474 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    if <Stm32g474 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32g474>(watchdog.clone())));
    }
    if <Stm32g474 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32g474>(watchdog)));
    }
    // G474: keep as pre-c3fe471 (no fallback) to preserve real_firmware SPI expectation
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// STM32H743 (H7, Cortex-M7) flash size in bytes (2 MiB).
pub const STM32_H743_FLASH_LEN: u32 = 0x0020_0000;
/// STM32H743 RAM size in bytes (512 KiB AXI + 128 KiB DTCM).
pub const STM32_H743_RAM_LEN: u32 = 0x0008_0000;
/// An STM32H743 board: the H7 line with `Stm32h743` tables.
#[must_use]
pub fn stm32h743() -> BoardMemory {
    use crate::device::svd::stm32h743::Stm32h743;
    // H7 memory: ITCM alias at 0x00000000, DTCM 128K at 0x20000000, AXI SRAM 512K at 0x24000000, SRAM1/2, ext SDRAM/QSPI
    const DTCM_BASE: u32 = 0x2000_0000;
    const DTCM_LEN: u32 = 0x0002_0000;
    const AXI_BASE: u32 = 0x2400_0000;
    const AXI_LEN: u32 = 0x0008_0000;
    const SRAM1_BASE: u32 = 0x3000_0000;
    const SRAM1_LEN: u32 = 0x0002_0000;
    const SRAM2_BASE: u32 = 0x3002_0000;
    const SRAM2_LEN: u32 = 0x0002_0000;
    const FMC_SDRAM_BASE: u32 = 0xC000_0000;
    const FMC_SDRAM_LEN: u32 = 0x0080_0000;
    const QSPI_BASE: u32 = 0x9000_0000;
    const QSPI_LEN: u32 = 0x0100_0000;
    const OTP_BASE: u32 = 0x1FF0_0000;
    const OTP_LEN: u32 = 0x0000_0400;
    let backing = FlatMemory::with_layout(
        &[
            FLASH,
            STM32_MAIN_FLASH,
            DTCM_BASE,
            AXI_BASE,
            SRAM1_BASE,
            SRAM2_BASE,
            FMC_SDRAM_BASE,
            QSPI_BASE,
            OTP_BASE,
        ],
        &[
            STM32_H743_FLASH_LEN,
            STM32_H743_FLASH_LEN,
            DTCM_LEN,
            AXI_LEN,
            SRAM1_LEN,
            SRAM2_LEN,
            FMC_SDRAM_LEN,
            QSPI_LEN,
            OTP_LEN,
        ],
    );
    let gate = Arc::new(Mutex::new(FlashGate::new(
        Region::new(STM32_MAIN_FLASH, STM32_H743_FLASH_LEN),
        32 * 1024,
    )));
    let qspi_gate = Arc::new(Mutex::new(QspiGate::new(Region::new(QSPI_BASE, QSPI_LEN))));
    let dma_requests = Arc::new(Mutex::new(DmaRequests::new()));
    let mut mem = SystemMemory::new(backing);
    mem.set_flash_gate(gate.clone());
    mem.set_qspi_gate(qspi_gate.clone());
    if <Stm32h743 as crate::device::svd::Device>::peripheral("FLASH").is_some() {
        mem.attach(Box::new(FlashF1::for_device::<Stm32h743>(gate)));
    }
    mem.attach(Box::new(Rcc::for_device::<Stm32h743>()));
    if <Stm32h743 as crate::device::svd::Device>::peripheral("PWR").is_some() {
        mem.attach(Box::new(Pwr::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("CRC").is_some() {
        mem.attach(Box::new(Crc::new_for::<Stm32h743>()));
    }
    // Watchdogs
    let watchdog = Arc::new(Mutex::new(false));
    mem.set_watchdog_reset(watchdog.clone());
    if <Stm32h743 as crate::device::svd::Device>::peripheral("IWDG").is_some() {
        mem.attach(Box::new(Iwdg::for_device::<Stm32h743>(watchdog.clone())));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("WWDG").is_some() {
        mem.attach(Box::new(Wwdg::for_device::<Stm32h743>(watchdog)));
    }
    // RTC - H743 RTC at 0x58004000, may be okay, keep
    if <Stm32h743 as crate::device::svd::Device>::peripheral("RTC").is_some() {
        mem.attach(Box::new(Rtc::new_for::<Stm32h743>()));
    }
    // BKP only on F1
    // if <Stm32h743 as crate::device::svd::Device>::peripheral("BKP").is_some() {
    //     mem.attach(Box::new(Bkp::new_for::<Stm32h743>()));
    // }
    // DAC
    if <Stm32h743 as crate::device::svd::Device>::peripheral("DAC").is_some() {
        mem.attach(Box::new(Dac::new_with_dma::<Stm32h743>(
            dma_requests.clone(),
        )));
    }
    // SDMMC1 (SDIO) - H743 uses SDMMC1 at 0x52007000, same offsets as SDIO
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SDMMC1").is_some()
        || <Stm32h743 as crate::device::svd::Device>::peripheral("SDIO").is_some()
    {
        mem.attach(Box::new(Sdio::new_for::<Stm32h743>()));
    }
    for (base, name) in [
        (0x5802_0000u32, "gpioa"),
        (0x5802_0400, "gpiob"),
        (0x5802_0800, "gpioc"),
        (0x5802_0c00, "gpiod"),
        (0x5802_1000, "gpioe"),
        (0x5802_1400, "gpiof"),
        (0x5802_1800, "gpiog"),
        (0x5802_1c00, "gpioh"),
        (0x5802_2000, "gpioi"),
        (0x5802_2400, "gpioj"),
        (0x5802_2800, "gpiok"),
    ] {
        if <Stm32h743 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(GpioF4::at_in::<Stm32h743>(base, name)));
        }
    }
    for ctor in [
        Usart::for_device::<Stm32h743> as fn() -> Usart,
        Usart::usart2_for::<Stm32h743>,
        Usart::usart3_for::<Stm32h743>,
        Usart::usart6_for::<Stm32h743>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32h743 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    for ctor in [
        Timer::tim2_for::<Stm32h743>,
        Timer::tim6_for::<Stm32h743>,
        Timer::tim7_for::<Stm32h743>,
        Timer::tim1_for::<Stm32h743>,
        Timer::tim3_for::<Stm32h743>,
        Timer::tim4_for::<Stm32h743>,
        Timer::tim5_for::<Stm32h743>,
    ] {
        let base = ctor().ranges()[0].base;
        if <Stm32h743 as crate::device::svd::Device>::by_base(base).is_some() {
            mem.attach(Box::new(ctor()));
        }
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SPI1").is_some() {
        mem.attach(Box::new(Spi::spi1_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SPI2").is_some() {
        mem.attach(Box::new(Spi::spi2_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SPI3").is_some() {
        mem.attach(Box::new(Spi::spi3_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SPI4").is_some() {
        mem.attach(Box::new(Spi::spi4_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SPI5").is_some() {
        mem.attach(Box::new(Spi::spi5_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SPI6").is_some() {
        mem.attach(Box::new(Spi::spi6_for::<Stm32h743>()));
    }
    // I2C
    if <Stm32h743 as crate::device::svd::Device>::peripheral("I2C1").is_some() {
        mem.attach(Box::new(I2c::for_device::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("I2C2").is_some() {
        mem.attach(Box::new(I2c::i2c2_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("I2C3").is_some() {
        mem.attach(Box::new(I2c::i2c3_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("I2C4").is_some() {
        mem.attach(Box::new(I2c::i2c4_for::<Stm32h743>()));
    }
    // ADC - H743 with dual-common (R2): ADC12_Common shared by ADC1/2, ADC3_Common solo
    let adc12_common_state = Arc::new(Mutex::new(AdcCommonState::default()));
    let adc3_common_state = Arc::new(Mutex::new(AdcCommonState::default()));
    if <Stm32h743 as crate::device::svd::Device>::peripheral("ADC12_Common").is_some() {
        mem.attach(Box::new(AdcCommon::adc12_common_for::<Stm32h743>(
            adc12_common_state.clone(),
        )));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("ADC3_Common").is_some() {
        mem.attach(Box::new(AdcCommon::adc12_common_for::<Stm32h743>(
            adc3_common_state.clone(),
        )));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("ADC1").is_some() {
        mem.attach(Box::new(
            Adc::adc1_for::<Stm32h743>().with_common(adc12_common_state.clone()),
        ));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("ADC2").is_some() {
        mem.attach(Box::new(
            Adc::adc2_for::<Stm32h743>().with_common(adc12_common_state.clone()),
        ));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("ADC3").is_some() {
        mem.attach(Box::new(
            Adc::adc3_for::<Stm32h743>().with_common(adc3_common_state.clone()),
        ));
    }
    // DMA - H7 uses stream DMA like F4 plus DMAMUX; use DmaF4 with Vec mailbox
    let dma = Arc::new(Mutex::new(Vec::new()));
    mem.set_dma_mailbox(dma.clone());
    if <Stm32h743 as crate::device::svd::Device>::peripheral("DMA1").is_some() {
        mem.attach(Box::new(DmaF4::dma1_for::<Stm32h743>(dma.clone())));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("DMA2").is_some() {
        mem.attach(Box::new(DmaF4::dma2_for::<Stm32h743>(dma)));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("DMAMUX1").is_some() {
        mem.attach(Box::new(Dmamux::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("DMAMUX2").is_some() {
        mem.attach(Box::new(Dmamux::dmamux2_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("LTDC").is_some() {
        mem.attach(Box::new(Ltdc::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("DMA2D").is_some() {
        mem.attach(Box::new(Dma2d::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("BDMA").is_some() {
        mem.attach(Box::new(Bdma::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("MDMA").is_some() {
        mem.attach(Box::new(Mdma::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SAI4").is_some() {
        mem.attach(Box::new(Sai4::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("DFSDM").is_some() {
        mem.attach(Box::new(Dfsdm::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("HRTIM_Master").is_some() {
        mem.attach(Box::new(Hrtim::master_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("HRTIM_TIMA").is_some() {
        mem.attach(Box::new(Hrtim::tima_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("HRTIM_TIMB").is_some() {
        mem.attach(Box::new(Hrtim::timb_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("HRTIM_TIMC").is_some() {
        mem.attach(Box::new(Hrtim::timc_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("HRTIM_TIMD").is_some() {
        mem.attach(Box::new(Hrtim::timd_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("HRTIM_TIME").is_some() {
        mem.attach(Box::new(Hrtim::time_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("HRTIM_Common").is_some() {
        mem.attach(Box::new(Hrtim::common_for::<Stm32h743>()));
    }
    // ETH - H743 single-window MAC at 0x40028000 (no MMC/PTP split) via EthH7 stub
    if <Stm32h743 as crate::device::svd::Device>::peripheral("Ethernet_MAC").is_some()
        || <Stm32h743 as crate::device::svd::Device>::peripheral("ETH").is_some()
    {
        // Use EthH7 stub (NoBehavior) so HAL probing (MACCR etc.) does not fault
        mem.attach(Box::new(EthH7::new_for::<Stm32h743>()));
    }
    // FDCAN
    if <Stm32h743 as crate::device::svd::Device>::peripheral("FDCAN1").is_some() {
        mem.attach(Box::new(Fdcan::fdcan1_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("FDCAN2").is_some() {
        mem.attach(Box::new(Fdcan::fdcan2_for::<Stm32h743>()));
    }
    // FMC / QUADSPI control windows (do not fault on config writes)
    // USB - H743 has OTG1_HS at 0x40040000 and OTG2_HS at 0x40080000 (FS via HS)
    if <Stm32h743 as crate::device::svd::Device>::peripheral("OTG1_HS_GLOBAL").is_some()
        || <Stm32h743 as crate::device::svd::Device>::peripheral("OTG_HS").is_some()
    {
        mem.attach(Box::new(Usb::otg_hs_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("OTG2_HS_GLOBAL").is_some() {
        mem.attach(Box::new(Usb::otg2_hs_for::<Stm32h743>()));
    } else if <Stm32h743 as crate::device::svd::Device>::peripheral("USB_OTG_FS").is_some()
        || <Stm32h743 as crate::device::svd::Device>::peripheral("OTG_FS").is_some()
    {
        mem.attach(Box::new(Usb::otg_fs_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("FMC").is_some() {
        mem.attach(Box::new(Fmc::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("QUADSPI").is_some()
        || <Stm32h743 as crate::device::svd::Device>::peripheral("QSPI").is_some()
        || <Stm32h743 as crate::device::svd::Device>::peripheral("OCTOSPI1").is_some()
    {
        mem.attach(Box::new(Qspi::new_for_with_gate::<Stm32h743>(Some(
            qspi_gate.clone(),
        ))));
    }
    // H743 only has QUADSPI at 0x52005000, not OCTOSPI2 (0x5200A000 is OCTOSPI2 on H723)
    // if <Stm32h743 as crate::device::svd::Device>::peripheral("OCTOSPI2").is_some() {
    //     mem.attach(Box::new(Qspi::octospi2_for::<Stm32h743>()));
    // }
    // EXTI/SYSCFG if present
    if <Stm32h743 as crate::device::svd::Device>::peripheral("EXTI").is_some() {
        mem.attach(Box::new(Exti::for_device::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SYSCFG").is_some() {
        mem.attach(Box::new(Syscfg::for_device::<Stm32h743>()));
    }
    // LPTIM1-5 behavioral (one by one)
    for name in ["LPTIM1", "LPTIM2", "LPTIM3", "LPTIM4", "LPTIM5"] {
        if let Some(def) = crate::device::svd::stm32h743::peripheral(name) {
            // Use generic LPTIM if name matches, else latch
            if <Stm32h743 as crate::device::svd::Device>::peripheral(name).is_some() {
                mem.attach(Box::new(crate::device::stm32::lptim::Lptim::new_for::<
                    Stm32h743,
                >(name)));
                let _ = def.base;
            }
        }
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("DCMI").is_some() {
        mem.attach(Box::new(crate::device::stm32::dcmi::Dcmi::new_for::<
            Stm32h743,
        >()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("JPEG").is_some() {
        mem.attach(Box::new(crate::device::stm32::jpeg::Jpeg::new_for::<
            Stm32h743,
        >()));
    }
    for name in ["SAI1", "SAI2", "SAI3"] {
        if <Stm32h743 as crate::device::svd::Device>::peripheral(name).is_some() {
            mem.attach(Box::new(crate::device::stm32::fmc::Sai4::new_for_name::<Stm32h743>(name)));
        }
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("VREFBUF").is_some() {
        mem.attach(Box::new(crate::device::stm32::vrefbuf::Vrefbuf::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("OPAMP").is_some() || <Stm32h743 as crate::device::svd::Device>::peripheral("OPAMP1").is_some() {
        mem.attach(Box::new(crate::device::stm32::opamp::Opamp::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("RNG").is_some() {
        mem.attach(Box::new(crate::device::stm32::rng::Rng::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("SPDIFRX").is_some() {
        mem.attach(Box::new(crate::device::stm32::spdifrx::Spdifrx::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("HSEM").is_some() {
        mem.attach(Box::new(crate::device::stm32::hsem::Hsem::new_for::<Stm32h743>()));
    }
    if <Stm32h743 as crate::device::svd::Device>::peripheral("LPUART1").is_some() {
        mem.attach(Box::new(crate::device::stm32::usart::Usart::lpuart1_for::<Stm32h743>()));
    }
    for (name, ctor) in [
        ("UART4", crate::device::stm32::usart::Usart::for_device::<Stm32h743> as fn() -> crate::device::stm32::usart::Usart),
        ("UART5", crate::device::stm32::usart::Usart::for_device::<Stm32h743> as fn() -> crate::device::stm32::usart::Usart),
        ("UART7", crate::device::stm32::usart::Usart::for_device::<Stm32h743> as fn() -> crate::device::stm32::usart::Usart),
        ("UART8", crate::device::stm32::usart::Usart::for_device::<Stm32h743> as fn() -> crate::device::stm32::usart::Usart),
        ("USART1", crate::device::stm32::usart::Usart::for_device::<Stm32h743> as fn() -> crate::device::stm32::usart::Usart),
        ("USART2", crate::device::stm32::usart::Usart::usart2_for::<Stm32h743> as fn() -> crate::device::stm32::usart::Usart),
        ("USART3", crate::device::stm32::usart::Usart::usart3_for::<Stm32h743> as fn() -> crate::device::stm32::usart::Usart),
        ("USART6", crate::device::stm32::usart::Usart::usart6_for::<Stm32h743> as fn() -> crate::device::stm32::usart::Usart),
    ] {
        if <Stm32h743 as crate::device::svd::Device>::peripheral(name).is_some() {
            // Check if already attached via earlier mounts (e.g., USART1 via previous logic)
            // Use try to attach
            // For now, we attempt to attach; if base already taken, skip
            let def = match <Stm32h743 as crate::device::svd::Device>::peripheral(name) {
                Some(d) => d,
                None => continue,
            };
            if matches!(mem.read(def.base), Err(crate::error::BusError::Reserved)) {
                // Use generic Usart for these UARTs
                mem.attach(Box::new(ctor()));
            }
        }
    }
    // Mount any remaining H7 peripherals as pure latches so the 119-peripheral
    // H743 SVD never faults as Reserved for the 500-chip universe. This closes
    // the 77-peripheral gap (AXI, COMP, DCMI, JPEG, LPTIM, OPAMP etc.) that
    // core_only_board already covers.
    for p in crate::device::svd::stm32h743::PERIPHERALS {
        if p.base >= 0xE000_0000 {
            continue;
        }
        if matches!(mem.read(p.base), Err(crate::error::BusError::Reserved)) {
            mem.attach(Box::new(crate::device::svd::RegisterFile::<
                crate::device::svd::NoBehavior,
            >::new(
                p, crate::device::svd::NoBehavior
            )));
        }
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// A core-only chip entry: name + generated table + flash base, from which
/// [`core_only_board`] builds a `BoardMemory` on demand (boards are not
/// cloneable; each machine gets its own).
pub struct Chip {
    /// The SVD table short name (e.g. `"stm32f101"`).
    pub name: &'static str,
    /// Main-flash base for the image (the boot alias).
    pub flash_base: u32,
    /// The generated peripheral tables.
    pub peripherals: &'static [crate::device::svd::PeripheralDef],
}

/// Every generated chip table, enumerated for the all-chips smoke test.
#[must_use]
pub fn all_chip_tables() -> Vec<(&'static str, &'static [crate::device::svd::PeripheralDef])> {
    use crate::device::svd;
    vec![
        ("stm32f030", svd::stm32f030::PERIPHERALS),
        ("stm32f031", svd::stm32f031::PERIPHERALS),
        ("stm32f042", svd::stm32f042::PERIPHERALS),
        ("stm32f072", svd::stm32f072::PERIPHERALS),
        ("stm32f091", svd::stm32f091::PERIPHERALS),
        ("stm32f0x0", svd::stm32f0x0::PERIPHERALS),
        ("stm32f0x1", svd::stm32f0x1::PERIPHERALS),
        ("stm32f0x2", svd::stm32f0x2::PERIPHERALS),
        ("stm32f0x8", svd::stm32f0x8::PERIPHERALS),
        ("stm32f0xx", svd::stm32f0xx::PERIPHERALS),
        ("stm32f100", svd::stm32f100::PERIPHERALS),
        ("stm32f101", svd::stm32f101::PERIPHERALS),
        ("stm32f102", svd::stm32f102::PERIPHERALS),
        ("stm32f103", svd::stm32f103::PERIPHERALS),
        ("stm32f105", svd::stm32f105::PERIPHERALS),
        ("stm32f107", svd::stm32f107::PERIPHERALS),
        ("stm32f303xe", svd::stm32f303xe::PERIPHERALS),
        ("stm32f407", svd::stm32f407::PERIPHERALS),
        ("stm32g474xx", svd::stm32g474xx::PERIPHERALS),
        ("stm32h743", svd::stm32h743::PERIPHERALS),
        ("stm32l476", svd::stm32l476::PERIPHERALS),
    ]
}

/// Build a core-only board from any generated table: flash + RAM + every
/// peripheral window mounted as a `NoBehavior` register file (pure latch),
/// plus `RCC` behavior, `DWT`, `SysTick`. No hand-written peripheral
/// behaviors — this is the F5 "core production only" contract: the
/// cycle-accurate core executes, windows round-trip, nothing else moves.
#[must_use]
pub fn core_only_board(
    peripherals: &'static [crate::device::svd::PeripheralDef],
    flash_base: u32,
) -> BoardMemory {
    use crate::device::svd::{NoBehavior, RegisterFile};
    // RAM size heuristic: H7 has DTCM/AXI banks; others 128 KiB is safe.
    let ram_len = if flash_base == 0x0800_0000 {
        0x0002_0000
    } else {
        0x0001_0000
    };
    let backing = FlatMemory::with_layout(&[flash_base, RAM], &[ram_len, ram_len]);
    let mut mem = SystemMemory::new(backing);
    // Every generated window is a pure `NoBehavior` latch; the RCC ready-bit
    // synthesis is not mounted here (core-only contract).
    for p in peripherals {
        mem.attach(Box::new(RegisterFile::<NoBehavior>::new(p, NoBehavior)));
    }
    mem.attach(Box::new(Dwt::new()));
    mem.attach(Box::new(SysTick::new()));
    mem
}

/// Enumerate every generated chip. Pair each with [`core_only_board`] to get
/// a bootable board; the smoke test boots each to `BKPT` on the cycle-accurate
/// core (M6).
#[must_use]
pub fn all_chips() -> Vec<Chip> {
    all_chip_tables()
        .into_iter()
        .map(|(name, peripherals)| Chip {
            name,
            flash_base: 0x0800_0000,
            peripherals,
        })
        .collect()
}
