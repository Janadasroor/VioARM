# Boards

| Board | Core | Flash | RAM | Peripherals |
|---|---|---|---|---|
| stm32f103 | M3 | 64 KiB | 20 KiB | GPIO+USART+TIM+ADC+SPI+I2C+CAN |
| stm32f107 | M3 | 256 KiB | 64 KiB | +ETH+USB+CAN2 |
| stm32f407 | M4 | 1 MiB | 128 KiB+CCM | +FPU+DMA F4 paced (TIM/ADC/SPI/USART request lines) |
| stm32h743 | M7 | 2 MiB | 512 KiB | +FPU-DP+SDRAM+QSPI+DTCM/AXI 0x20000+0x60000+FDCAN1+LPTIM1-5 |
| **505 Dnames** | M0/M3/M4/M7 | 16K–2M | 4K–512K | **506 boards `100%` `bootable` `32` `SVD` donors + removal deltas `505` `per_dname.rs` `true-generation` `F2`/`F7`/`G0` `IROM1/IRAM1` `H7` `DTCM` `all_boards()` `all_per_dnames()` `RCC` `HPRE/PPRE` → `APB` timer ratios `FLASH` `WS` fetch stalls |
| stm32f207/f217 | M3 | 256K–1M | 64–128K | `F4`-gen set, no `FPU`, `ETH`/`DCMI` per die, paced DMA + `LPTIM1` |
| stm32f722/f746/f767/f730 | M7 | 256K–2M | 64–512K | `F4`-gen set, `QUADSPI`/`FMC` latch, `F730` own donor, paced DMA + `LPTIM1` |
| stm32g031/g081/g050/g061/g0c1 | M0+ | 16–512K | 4–144K | `G031` behaviors, `FLASH`/`I2C`/`EXTI`/`SYSCFG` latch, `G0` `RCC`, `FDCAN`-cut + `LPTIM` |
| stm32u535/u575/u5g9 | M33 | 128K–4M | 128K–3M | `GPIO`/`TIM`/`USART`/`RTC`/`RNG`/`CRC`/`WDG`/`FDCAN`-cut/`LPTIM`, `SPI`/`ADC`/`FLASH` latch |
| stm32h503/h533/h563 | M33 | 128K–2M | 32–640K | same set, `H533` own donor (`_S` aliases) |
| stm32wba55 | M33 | 512K–1M | 128–512K | same set, radio permanently latch-only |
| stm32l552/l562 | M33 | 256–512K | 128–256K | same set + `ADC` (plain names) |

## Pin labels (`vioarm boards --pins`)

HIL wiring needs human pin names, not SVD windows:

```sh
vioarm boards --pins stm32f103   # one board
vioarm boards --pins all         # all 13 tables
```

Tables live in `vioarm-core/src/pins.rs` (`pin_labels`, `PIN_BOARDS`):
`stm32f103`, `stm32f107`, `stm32f407`, `stm32f401`, `stm32f072`,
`stm32f030`, `stm32f303xe`, `stm32l476`, `stm32g474xx`, `stm32h743`,
`stm32u575`, `stm32wba55`, `stm32l552`. Each row is `PIN: FUNCTION` in
the default alternate-function mapping firmware actually uses
(`PA9: USART1_TX`, `PC13: GPIO_O`, …); the full AF matrix is explicitly
out of scope. Unknown boards error with the valid list. `vioarm docs
boards` points here.
