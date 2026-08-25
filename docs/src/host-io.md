# Host I/O

- `Peripheral::drive_inputs` / `drain_output` for CAN/ETH/USB/SDIO/DAC etc.
- `DmaRequests` mailbox for USART/SPI/ADC/TIM, `EthMailbox`, `Sdio` card image.
- `QspiGate` for `0x90000000` via `QUADSPI CR.EN`.
