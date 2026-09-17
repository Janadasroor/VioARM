//! Static per-board pin-label catalog (WP-R6): human pin names for HIL wiring.
//!
//! `vioarm boards` lists peripheral counts per die; nothing maps `PA9` to
//! `USART1_TX` on a given die, so every lab setup rebuilds that table by
//! hand from headers. These tables record the default alternate-function
//! mapping firmware actually uses (`PA9: USART1_TX`, `PC13: GPIO_O`, …).
//! The full AF matrix is explicitly out of scope and unstated: each pin
//! carries exactly one function here, the power-on/default one.
//!
//! Scope: the 13 headline full boards first (`PIN_BOARDS`). BIOS-level
//! claims only: `PA9/PA10 = USART1_TX/RX`, `PA13/PA14 = SWDIO/SWCLK`,
//! and the LED/user pins proven by `examples/` (`PC13` on `stm32f103`)
//! or the Nucleo/Discovery convention (`PA5` LD2, `PC13` B1, `PD12-15`
//! Discovery LEDs, Nucleo-144 `PB0/PE1/PB14`). Everything else is plain
//! `GPIO_I`/`GPIO_O` or the default peripheral on that pin. No new
//! dependency, no SVD schema change: `const` tables beside the board
//! constructors.

/// Boards carrying a pin-label table (13 headline full boards).
pub const PIN_BOARDS: &[&str] = &[
    "stm32f103",
    "stm32f107",
    "stm32f407",
    "stm32f401",
    "stm32f072",
    "stm32f030",
    "stm32f303xe",
    "stm32l476",
    "stm32g474xx",
    "stm32h743",
    "stm32u575",
    "stm32wba55",
    "stm32l552",
];

/// Blue-Pill-class `stm32f103`: `PC13` is the board LED (`examples/blink.rs`
/// toggles it via `BSRR`); `PA9/PA10` the USART1 pair (`real_firmware.rs`
/// drives `PA9` alternate push-pull for TX).
const STM32F103_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "SPI1_SCK"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PA15", "JTDI"),
    ("PB3", "JTDO"),
    ("PB4", "JNTRST"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_O"),
];

/// Connectivity-line `stm32f107`: F1 core plus `USB_OTG_FS` on
/// `PA11/PA12`; high-pin-count ETH RMII is out of scope (see module docs).
const STM32F107_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "SPI1_SCK"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA11", "USB_OTG_FS_DM"),
    ("PA12", "USB_OTG_FS_DP"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_O"),
];

/// Discovery-class `stm32f407`: user button `PA0`, four LEDs
/// `PD12-15`, `USB_OTG_FS` on `PA11/PA12`.
const STM32F407_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "SPI1_SCK"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA11", "USB_OTG_FS_DM"),
    ("PA12", "USB_OTG_FS_DP"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PD12", "GPIO_O"),
    ("PD13", "GPIO_O"),
    ("PD14", "GPIO_O"),
    ("PD15", "GPIO_O"),
];

/// Nucleo-64 `stm32f401`: `PA5` LD2, `PC13` B1, ST-Link VCP on
/// `PA2/PA3`.
const STM32F401_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "GPIO_O"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_I"),
];

/// Nucleo-64 `stm32f072`: `PA5` LD2, `PC13` B1, full-speed USB on
/// `PA11/PA12`.
const STM32F072_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "GPIO_O"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA11", "USB_DM"),
    ("PA12", "USB_DP"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_I"),
];

/// Value-line `stm32f030`: discovery LEDs `PC8/PC9`, USART1 on
/// `PA9/PA10`.
const STM32F030_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA5", "GPIO_O"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC8", "GPIO_O"),
    ("PC9", "GPIO_O"),
];

/// Nucleo-64 `stm32f303xe`: `PA5` LD2, `PC13` B1, VCP on `PA2/PA3`.
const STM32F303XE_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "GPIO_O"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_I"),
];

/// Nucleo-64 `stm32l476`: `PA5` LD2, `PC13` B1, VCP on `PA2/PA3`;
/// discovery pair `PB2/PE8` included as plain outputs.
const STM32L476_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "GPIO_O"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB2", "GPIO_O"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_I"),
    ("PE8", "GPIO_O"),
];

/// Nucleo `stm32g474xx`: `PA5` LD2, `PC13` B1, `PA2/PA3` USART2,
/// `PA9/PA10` USART1 (all AF7-capable; default mapping only).
const STM32G474XX_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "GPIO_O"),
    ("PA6", "SPI1_MISO"),
    ("PA7", "SPI1_MOSI"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_I"),
];

/// Nucleo-144 `stm32h743`: ST-Link VCP on `PD8/PD9` (USART3),
/// `PC13` B1, LEDs `PB0/PE1/PB14`.
const STM32H743_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB0", "GPIO_O"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PB14", "GPIO_O"),
    ("PC13", "GPIO_I"),
    ("PD8", "USART3_TX"),
    ("PD9", "USART3_RX"),
    ("PE1", "GPIO_O"),
];

/// Nucleo-144 `stm32u575`: `PC13` B1, `PA5` Arduino D13; serial and
/// SWD pairs are the family defaults. No LD-number claims (see module
/// docs).
const STM32U575_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "GPIO_O"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB0", "GPIO_O"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_I"),
];

/// Nucleo-64 `stm32wba55`: `PA9/PA10` USART1, `PA13/PA14` SWD,
/// `PB6/PB7` I2C1; radio and full AF matrix out of scope.
const STM32WBA55_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA5", "GPIO_O"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB4", "GPIO_O"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_I"),
];

/// Nucleo-144 `stm32l552`: `PC13` B1, `PA5` D13; `PA2/PA3` USART2,
/// `PA9/PA10` USART1 defaults.
const STM32L552_PINS: &[(&str, &str)] = &[
    ("PA0", "GPIO_I"),
    ("PA2", "USART2_TX"),
    ("PA3", "USART2_RX"),
    ("PA5", "GPIO_O"),
    ("PA9", "USART1_TX"),
    ("PA10", "USART1_RX"),
    ("PA13", "SWDIO"),
    ("PA14", "SWCLK"),
    ("PB6", "I2C1_SCL"),
    ("PB7", "I2C1_SDA"),
    ("PC13", "GPIO_I"),
];

/// Pin-label table for `board` (case-insensitive short name, e.g.
/// `"stm32f103"`): `None` for unknown boards.
#[must_use]
pub fn pin_labels(board: &str) -> Option<&'static [(&'static str, &'static str)]> {
    let mut lower = [0u8; 16];
    let bytes = board.as_bytes();
    if bytes.len() > lower.len() {
        return None;
    }
    for (i, b) in bytes.iter().enumerate() {
        lower[i] = b.to_ascii_lowercase();
    }
    match &lower[..bytes.len()] {
        b"stm32f103" => Some(STM32F103_PINS),
        b"stm32f107" => Some(STM32F107_PINS),
        b"stm32f407" => Some(STM32F407_PINS),
        b"stm32f401" => Some(STM32F401_PINS),
        b"stm32f072" => Some(STM32F072_PINS),
        b"stm32f030" => Some(STM32F030_PINS),
        b"stm32f303xe" => Some(STM32F303XE_PINS),
        b"stm32l476" => Some(STM32L476_PINS),
        b"stm32g474xx" => Some(STM32G474XX_PINS),
        b"stm32h743" => Some(STM32H743_PINS),
        b"stm32u575" => Some(STM32U575_PINS),
        b"stm32wba55" => Some(STM32WBA55_PINS),
        b"stm32l552" => Some(STM32L552_PINS),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{pin_labels, PIN_BOARDS};

    fn function(board: &str, pin: &str) -> Option<&'static str> {
        pin_labels(board)?
            .iter()
            .find(|(name, _)| *name == pin)
            .map(|(_, function)| *function)
    }

    #[test]
    fn all_thirteen_boards_carry_tables() {
        assert_eq!(PIN_BOARDS.len(), 13);
        for board in PIN_BOARDS {
            let table = pin_labels(board);
            assert!(table.is_some(), "{board} carries a table");
            assert!(!table.expect("checked").is_empty(), "{board} non-empty");
        }
    }

    #[test]
    fn serial_and_swd_pairs_agree_everywhere() {
        for board in PIN_BOARDS {
            assert_eq!(function(board, "PA9"), Some("USART1_TX"), "{board}");
            assert_eq!(function(board, "PA10"), Some("USART1_RX"), "{board}");
            assert_eq!(function(board, "PA13"), Some("SWDIO"), "{board}");
            assert_eq!(function(board, "PA14"), Some("SWCLK"), "{board}");
        }
    }

    #[test]
    fn led_and_button_pins_match_examples() {
        // `examples/blink.rs` toggles Blue-Pill `PC13`; `real_firmware.rs`
        // drives `PA9` alternate push-pull for USART1 TX.
        assert_eq!(function("stm32f103", "PC13"), Some("GPIO_O"));
        assert_eq!(function("stm32f103", "PA9"), Some("USART1_TX"));
        assert_eq!(function("stm32f407", "PD12"), Some("GPIO_O"));
        assert_eq!(function("stm32h743", "PB0"), Some("GPIO_O"));
    }

    #[test]
    fn lookup_is_case_insensitive_and_fail_closed() {
        assert!(pin_labels("STM32F103").is_some());
        assert!(pin_labels("Stm32H743").is_some());
        assert!(pin_labels("stm32f999").is_none());
        assert!(pin_labels("").is_none());
        assert!(pin_labels("stm32f103xx-extra-long-name").is_none());
    }
}
