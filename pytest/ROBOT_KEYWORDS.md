# Robot Framework keyword catalog

The pytest harness workflows below, spelled as Robot keywords. Each keyword
maps to one `vioarm_harness` call (or one CLI invocation via the `Process`
library); import this file's examples into a `.robot` resource backed by
`Process.Run Process` on `$VIOARM_BIN`.

Conventions: `${BOARD}` defaults to `stm32f103`, `${BASE}` to `0`,
`${BUDGET}` to `1000000`. Failures surface the CLI's stderr (golden diffs
print as `diff:` lines; a clean replay prints `MATCH` and exits 0).

## Build Firmware

Builds CMSIS C firmware to raw `.text` plus a symbol table.

| Arg | Meaning |
|---|---|
| `${SRC}` | path to the `.c` file |
| `${MCPU}` | default `cortex-m0` (runs on M3 boards too) |

```robot
${text}    ${symbols}    Build Firmware    ${SRC}
Should Not Be Empty    ${text}
```

Backed by `build_c` (needs `arm-none-eabi-gcc` + vendored packs; skip the
test when absent, mirroring the Rust suite's skip convention).

## Flash Image With Vectors

Prepends an SP + reset-vector table (Thumb bit set) to raw `.text`, using
the `reset` symbol offset.

```robot
${image}    Flash Image With Vectors    ${text}    ${symbols}[reset]
```

Backed by `image_with_vectors` (`struct.pack("<II", 0x20001000, 8 + off | 1)`).

## Run Board

Runs an image file on a board; returns `(rc, out, err)` with ANSI stripped.

```robot
${rc}    ${out}    ${err}    Run Board    ${image}    stm32f103
```

Backed by `run_board` (`vioarm run --board … --budget …`).

## Channel Should Contain

Asserts a drain channel carried expected bytes.

```robot
Run Board    ${image}    stm32f103
Channel Should Contain    ${out}    uart:    ok!
```

Backed by `channel_text` (matches `uart:`, `semihost:`, `itm:` lines the
CLI prints for non-empty drains).

## Record Golden Trace

```robot
Record Golden Trace    ${image}    stm32f103    ${CURDIR}/golden.txt
```

Backed by `record` (`vioarm record --output …`, optional `--snap BASE:LEN`
repeats; default snapshots 4 KiB at `0x20000000`).

## Replay Should Match

```robot
Replay Should Match    ${image}    ${CURDIR}/golden.txt    stm32f103
```

Backed by `replay` (rc 0 plus `MATCH` in output).

## Replay With Patch Should Differ

A `--patch ADDR=VAL` stands in for a mutated model; the replay must exit 1
with a `snap … differs` line.

```robot
Replay With Patch Should Differ    ${image}    ${CURDIR}/golden.txt    0x20000010=0x41
```

## Full example

```robot
*** Test Cases ***
USART Transmit Reaches The Host
    ${text}    ${symbols}    Build Firmware    usart_tx.c
    ${image}    Flash Image With Vectors    ${text}    ${symbols}[reset]
    ${rc}    ${out}    ${err}    Run Board    ${image}    stm32f103
    Channel Should Contain    ${out}    uart:    ok!

Golden Trace Survives A Rerun
    Record Golden Trace    replay.bin    stm32f103    golden.txt
    Replay Should Match    replay.bin    golden.txt    stm32f103
    Replay With Patch Should Differ    replay.bin    golden.txt    0x20000010=0x41
```
