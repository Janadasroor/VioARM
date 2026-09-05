# Host I/O

- `Peripheral::drive_inputs` / `drain_output` for CAN/ETH/USB/SDIO/DAC etc.
- `DmaRequests` mailbox for USART/SPI/ADC/TIM, `EthMailbox`, `Sdio` card image.
- `QspiGate` for `0x90000000` via `QUADSPI CR.EN`.
- Semihosting: `vioarm run fw.elf --semihost` arms `BKPT 0xAB` (`WRITEC`/`WRITE0`/`WRITE` fd 1/2 → `semihost:` output, `READC` from queued input, `EXIT` → `semihost exit(code)` halt). Disabled by default, so `BKPT` keeps faulting for QEMU parity.
- ITM: writes to stimulus ports (`0xE0000000`, `TER` `0xE0000E00`, `TCR` `0xE0000E80`) queue into the `itm:` channel on every board; drain with `Machine::drain_channel("itm")`.
- RTT: `Machine::rtt_poll(ram_base, ram_len)` drains target→host bytes on channel 0 (scans for `SEGGER RTT` once, ring offsets wrap at size), `rtt_inject` feeds channel 0. Virtual timebase is `cyccnt` at 1 MHz (`TICKFREQ`), so `CLOCK`/`ELAPSED` are deterministic.
- Hooks (`Machine::on_instruction`/`on_read`/`on_write`/`on_peripheral`): pre/post-instruction callbacks over a PC range, byte-granular memory watchpoints, and peripheral-access events by device name. Callbacks observe only (no machine handle — reentry is impossible by construction). With nothing armed the cost is one predicted branch per step/access (measured 2.4% worst-case, `BENCHMARKS.md`). `Coverage` rebuilds basic blocks from retired PCs (`cargo run -p vioarm-core --example coverage`). CLI: `vioarm trace fw.elf --format jsonl [--output trace.jsonl]` streams `{"cycle","pc"}` per retired instruction. Rust API only, deliberately no `.so` plugin ABI.
