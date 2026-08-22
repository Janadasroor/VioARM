# VioARM

An ARM Cortex-M instruction-set simulator written in Rust. VioARM executes
ARMv6-M and ARMv7-M (Cortex-M3/M4) firmware with a soundness-first design:
every illegal or impossible state is a typed error, never silent undefined
behavior, and execution is deterministic.

The integer core, the ARMv7E-M DSP extension, and the FPv4-SP floating-point
unit are validated instruction-by-instruction against `qemu-system-arm`
(Cortex-M3 and Cortex-M4 board models) by a differential conformance suite.

## Features

- ARMv7-M (Cortex-M3/M4) execution with broad Thumb-2 coverage and the
  ARMv7E-M DSP zone.
- FPv4-SP VFP (Cortex-M4F), including M-profile FP context switching: FPCCR
  ASPEN/LSPEN/LSPACT, FPCAR, FPDSCR, CONTROL.FPCA, extended exception frames,
  and lazy stacking.
- ARMv6-M 16-bit instruction set.
- Exceptions: vector-table entry, EXC_RETURN (basic and FP-carrying), SVC,
  fault escalation to HardFault, lockup on double fault, PRIMASK, FAULTMASK,
  BASEPRI, and IT blocks.
- Board models for a generic virtual board and the STM32F103, STM32F107, and
  STM32F407, with an SCB subset, NVIC, SysTick, UART-Tx, GPIO, RCC, USART,
  timers, EXTI, the STM32F1 flash controller (KEYR unlock, page/mass erase,
  and gated programming at `0x0800_0000`), and the STM32F1 DMA controllers
  (memory-to-memory transfers through a bus-shared mailbox).
- Raw, ELF32, and Intel HEX image loaders with strict validation.
- A GDB Remote Serial Protocol server for `arm-none-eabi-gdb`.

## Workspace layout

| Crate       | Role                                                        |
| ----------- | ----------------------------------------------------------- |
| `vioarm-core` | Simulator core: CPU, memory, devices, loaders. No runtime deps. |
| `vioarm-cli`  | Thin runner: run images, per-step trace, GDB server.         |
| `vioarm-gdb`  | GDB Remote Serial Protocol server.                           |
| `vioarm-qemu` | Differential conformance harness against `qemu-system-arm`. |
| `vioarm-svd`  | SVD-driven register-table code generation.                   |

## Building

The workspace builds on Rust 1.75 or newer.

```sh
cargo build --workspace
```

## Running firmware

Run an ELF, Intel HEX, or raw binary image on any board:

```sh
# An ELF on the generic virtual board
cargo run -p vioarm-cli -- --elf firmware.elf

# A raw image on an STM32F103 board at the flash base
cargo run -p vioarm-cli -- --bin image.bin --board stm32f103 --base 0x08000000

# Print a per-step register trace
cargo run -p vioarm-cli -- --elf firmware.elf --trace

# Print the image layout and exit
cargo run -p vioarm-cli -- --elf firmware.elf --info
```

The machine halts with a typed cause (`CycleBudgetExhausted`, faults, or a
reserved/unimplemented instruction) and prints the final register state.
UART-Tx output is drained and printed on exit.

### Debugging with GDB

Serve the machine to GDB over the Remote Serial Protocol:

```sh
cargo run -p vioarm-cli -- --elf firmware.elf --gdb 3333
```

Then attach from GDB:

```text
target remote :3333
```

## Testing

```sh
cargo test --workspace
```

The suite includes unit tests, end-to-end images, firmware compiled with clang
in-test, and the differential conformance tests. The conformance suite runs
when `qemu-system-arm` is available on `PATH` and skips gracefully otherwise;
the clang-compiled firmware tests skip when clang is not installed.

```sh
# Run the conformance suite only
cargo test -p vioarm-qemu --test conformance
```

## Known limitations

- Cycle timing: `DWT_CYCCNT` counts TRM `cyccnt` (per-opcode CPI + `FLASH_ACR` `LATENCY` `WS`), `WFI`/`WFE`/`SEV` sleep/event modeled; `cycles` stays `1c/insn` for compat.
- `HFSR.FORCED` at bit 30 and `SHCSR` `*ACT` zero to match QEMU oracle.
- STM32H743 (Cortex-M7 `FPv5-D16`): `RCC` `HSEON→HSERDY`/`PLL1-3`, `FLASH` `ACR` `LATENCY`, `PWR` `CR1/CR3` synthesize; `FMC`/`QUADSPI` (`0x52004000`/`0x52005000`) + `FDCAN1/2` + `LTDC`/`DMA2D` + `DMAMUX1/2` + `SDMMC1` (via `SDIO`) are register-window stubs `NoBehavior` with ext `SDRAM 0xC0000000`/`QSPI 0x90000000` always-mapped (QSPI not yet `CR.EN` gated, `EthH7` `0x40028000` single-window MAC stub, `OCTOSPI2` is `H723`-only).
- STM32F107 Ethernet/USB/CAN and STM32F407 CCM RAM are modeled (see `Implemented`); F1 main flash `0x0800_0000` is a gated store (`0x0000_0000` alias ungated), F407 flash at `0x40023C00` shares the gate. DMA `PL` arbitration and `PSIZE`/`MSIZE` `PINC`/`MINC` are modeled; burst/FIFO and timer/ADC/SPI request lines (beyond USART) are not.
- Double-precision `UNDEFINED` on `FPv4-SP` (M4F) except `D↔S` alias moves/loads; full `FPv5-D16` arithmetic on `M7/H743` only.

See `roadmap.md` for the full inventory and the development plan.

## License

MIT. See `LICENSE`.