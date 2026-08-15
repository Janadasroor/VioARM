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
  timers, EXTI, and the STM32F1 flash controller (KEYR unlock, page/mass
  erase, and gated programming at `0x0800_0000`).
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

- No cycle-accurate timing: one cycle per retired instruction, and WFI/WFE
  are no-ops. Time-sensitive firmware does not reproduce silicon behavior.
- CPACR is stored but does not gate FP access; the FPU is always enabled.
- No DMA or MPU.
- `AIRCR`/`SCR`/`CCR` are modeled (PRIGROUP, SYSRESETREQ, SLEEPONEXIT,
  STKALIGN, DIV_0_TRP), as are `VTOR`, `SHCSR`, and `CFSR`/`HFSR`. To match
  the QEMU oracle, `HFSR.FORCED` is reported at bit 30 and the `SHCSR`
  active-status bits read zero.
- STM32F107 Ethernet/USB/CAN and STM32F407 CCM RAM are not modeled. The
  STM32F1 main flash at `0x0800_0000` is a separate gated store (the boot-time
  alias at `0x0000_0000` is ungated), and the F407 flash controller is not
  modeled.
- Double-precision arithmetic is not implemented (D registers alias S pairs
  for moves and loads only).

See `roadmap.md` for the full inventory and the development plan.

## License

MIT. See `LICENSE`.