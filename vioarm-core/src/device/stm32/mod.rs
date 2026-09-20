//! STM32-family peripherals, driven by the SVD-generated register tables.
//!
//! These model the real register layouts (base addresses and offsets) of a
//! Blue-Pill-class STM32F103 so that unmodified, `clang`-compiled firmware can
//! drive them. The shared cores — `RCC`, `EXTI`, `USART`, `Timer` — are
//! chip-agnostic and mount against any `Device` table (F103, F107, F407)
//! through `for_device`; `gpio_f4` and `syscfg` model the Cortex-M4 port and
//! its EXTI router. Clock gating is *not* enforced: the `RCC` latches the
//! enable bits faithfully, but every peripheral runs as if its clock were on.
//! Conforming firmware (which always enables the clocks it uses before
//! touching a peripheral) behaves identically to silicon.

pub mod adc;
pub mod aes;
pub mod afio;
pub mod bkp;
pub mod ble;
pub mod can;
pub mod cec;
pub mod comp;
pub mod cordic;
pub mod crc;
pub mod crs;
pub mod cryp;
pub mod dac;
pub mod dbgmcu;
pub mod dcmi;
pub mod dma;
pub mod dma_f4;
pub mod eth;
pub mod exti;
pub mod fdcan;
pub mod flash;
pub mod fmac;
pub mod fmc;
pub mod fsmc;
pub mod gpio;
pub mod gpio_f4;
pub mod hash;
pub mod hsem;
pub mod i2c;
/// STM32 I3C peripheral.
pub mod i3c;
pub mod jpeg;
pub mod lptim;
pub mod mdios;
/// STM32 OCTOSPI peripheral.
pub mod octospi;
pub mod opamp;
/// STM32 OTFDEC peripheral.
pub mod otfdec;
pub mod pwr;
pub mod radio;
pub mod rcc;
pub mod rng;
pub mod rtc;
pub mod sdio;
pub mod spdifrx;
pub mod spi;
/// STM32 SUBGHZ radio peripheral.
pub mod subghz;
pub mod swpmi;
pub mod syscfg;
pub mod tamp;
pub mod timer;
pub mod tsc;
pub mod usart;
pub mod usb;
pub mod usb_fs;
pub mod vrefbuf;
pub mod watchdog;

/// STM32F103 (medium-density) peripheral base addresses, from the SVD.
pub const RCC: u32 = 0x4002_1000;
/// Base address of the AFIO peripheral.
pub const AFIO: u32 = 0x4001_0000;
/// Base address of the EXTI peripheral.
pub const EXTI: u32 = 0x4001_0400;
/// Base address of TIM2.
pub const TIM2: u32 = 0x4000_0000;
/// Base address of TIM3.
pub const TIM3: u32 = 0x4000_0400;
/// Base address of USART1.
pub const USART1: u32 = 0x4001_3800;
/// Base address of GPIOA.
pub const GPIOA: u32 = 0x4001_0800;
/// Base address of GPIOB.
pub const GPIOB: u32 = 0x4001_0C00;
/// Base address of GPIOC.
pub const GPIOC: u32 = 0x4001_1000;
/// Base address of GPIOD.
pub const GPIOD: u32 = 0x4001_1400;
/// Base address of GPIOE.
pub const GPIOE: u32 = 0x4001_1800;
/// Base address of GPIOF.
pub const GPIOF: u32 = 0x4001_1C00;
/// Base address of GPIOG.
pub const GPIOG: u32 = 0x4001_2000;
