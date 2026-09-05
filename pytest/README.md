# VioARM pytest integration

Framework integration tests driving the built CLI end to end. The Rust
suite proves the models; this suite proves the *workflow*: build firmware,
run a board, assert what came out, and golden-trace replay.

## Layout

- `vioarm_harness.py` — the template. Copy it into your project and adapt:
  `build_c` (CMSIS firmware via `arm-none-eabi-gcc` + vendored packs),
  `image_with_vectors`, `run_board`, `record`, `replay`, `channel_text`.
  Only the standard library is used besides `pytest`.
- `test_usart_tx.py` — build-image → run → assert-drain example (needs the
  toolchain + packs, skips otherwise).
- `test_replay.py` — record/replay/mutate golden traces with a
  hand-assembled image (no toolchain, always runs).

## Running

```sh
pip install -r pytest/requirements.txt
cargo build -p vioarm-cli
VIOARM_BIN=$PWD/target/debug/vioarm-cli pytest pytest/ -v
```

`VIOARM_BIN` defaults to `target/debug/vioarm-cli` under the repo root.
`test_usart_tx.py` additionally needs `arm-none-eabi-gcc` (plus `objcopy`
and `nm`) and the vendored packs; it skips with a note when absent.

## CI

The `pytest` CI job (ubuntu) installs `gcc-arm-none-eabi`, builds the CLI
in debug, and runs this directory — `test_usart_tx.py` goes green there,
not just skipped.

## Robot Framework

`ROBOT_KEYWORDS.md` catalogs the same workflows as Robot keywords
(`Build Firmware`, `Run Board`, `Channel Should Contain`, `Record Golden
Trace`, `Replay Should Match`, `Replay With Patch Should Differ`) with
copy-paste examples written against the `Process` library and `$VIOARM_BIN`.
