# Getting Started

```bash
cargo run -p vioarm-cli -- run --board stm32f103 --elf firmware.elf
cargo run -p vioarm-cli -- gdb --board stm32f103 --elf firmware.elf
```

Boards: `stm32f103` (F1), `stm32f107` (F107), `stm32f407` (F4), `stm32h743` (H7), `stm32f072` etc.
