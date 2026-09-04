# Boards

| Board | Core | Flash | RAM | Peripherals |
|---|---|---|---|---|
| stm32f103 | M3 | 64 KiB | 20 KiB | GPIO+USART+TIM+ADC+SPI+I2C+CAN |
| stm32f107 | M3 | 256 KiB | 64 KiB | +ETH+USB+CAN2 |
| stm32f407 | M4 | 1 MiB | 128 KiB+CCM | +FPU+DMA F4 |
| stm32h743 | M7 | 2 MiB | 512 KiB | +FPU-DP+SDRAM+QSPI+DTCM/AXI 0x20000+0x60000 |
| **505 Dnames** | M0/M3/M4/M7 | 16K–2M | 4K–512K | **506 boards `100%` `bootable` `32` `SVD` donors + removal deltas `505` `per_dname.rs` `true-generation` `F2`/`F7`/`G0` `IROM1/IRAM1` `H7` `DTCM` `all_boards()` `all_per_dnames()` `RCC` `HPRE/PPRE` → `APB` timer ratios `FLASH` `WS` fetch stalls |
| stm32f207/f217 | M3 | 256K–1M | 64–128K | `F4`-gen set, no `FPU`, `ETH`/`DCMI` per die |
| stm32f722/f746/f767/f730 | M7 | 256K–2M | 64–512K | `F4`-gen set, `QUADSPI`/`FMC` latch, `F730` own donor |
| stm32g031/g081/g050/g061/g0c1 | M0+ | 16–512K | 4–144K | `G031` behaviors, `FLASH`/`I2C`/`EXTI`/`SYSCFG` latch, `G0` `RCC` |
| stm32u535/u575/u5g9 | M33 | 128K–4M | 128K–3M | `GPIO`/`TIM`/`USART`/`RTC`/`RNG`/`CRC`/`WDG`, `SPI`/`ADC`/`FLASH` latch |
| stm32h503/h533/h563 | M33 | 128K–2M | 32–640K | same set, `H533` own donor (`_S` aliases) |
| stm32wba55 | M33 | 512K–1M | 128–512K | same set, radio permanently latch-only |
| stm32l552/l562 | M33 | 256–512K | 128–256K | same set + `ADC` (plain names) |
