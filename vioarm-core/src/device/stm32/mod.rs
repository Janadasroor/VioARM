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

pub mod afio;
pub mod exti;
pub mod gpio;
pub mod gpio_f4;
pub mod rcc;
pub mod syscfg;
pub mod timer;
pub mod usart;

/// STM32F103 (medium-density) peripheral base addresses, from the SVD.
pub const RCC: u32 = 0x4002_1000;
pub const AFIO: u32 = 0x4001_0000;
pub const EXTI: u32 = 0x4001_0400;
pub const TIM2: u32 = 0x4000_0000;
pub const TIM3: u32 = 0x4000_0400;
pub const USART1: u32 = 0x4001_3800;
pub const GPIOA: u32 = 0x4001_0800;
pub const GPIOB: u32 = 0x4001_0C00;
pub const GPIOC: u32 = 0x4001_1000;
