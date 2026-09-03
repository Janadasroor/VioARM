# Boards

| Board | Core | Flash | RAM | Peripherals |
|---|---|---|---|---|
| stm32f103 | M3 | 64 KiB | 20 KiB | GPIO+USART+TIM+ADC+SPI+I2C+CAN |
| stm32f107 | M3 | 256 KiB | 64 KiB | +ETH+USB+CAN2 |
| stm32f407 | M4 | 1 MiB | 128 KiB+CCM | +FPU+DMA F4 |
| stm32h743 | M7 | 2 MiB | 512 KiB | +FPU-DP+SDRAM+QSPI+DTCM/AXI 0x20000+0x60000 |
| **505 Dnames** | M0/M3/M4/M7 | 16K–2M | 4K–512K | **506 boards `100%` `bootable` `21` `SVD` donors + removal deltas `505` `per_dname.rs` `100%` `alias→real` `IROM1/IRAM1` `H7` `DTCM` `all_boards()` `all_per_dnames()` `RCC` `HPRE/PPRE` → `APB` timer ratios `FLASH` `WS` fetch stalls |
