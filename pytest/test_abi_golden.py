"""E4 ABI golden: CLI and C ABI agree bit-for-bit on real firmware.

Pin edges: `trace --format vcd` timestamps (integer ns) converted to
cyccnt must equal the `run_quantum` event stamps for the same image.
UART bytes: the CLI `uart:` drain must equal the ABI `drain("uart")`.
Any CLI/embedding divergence fails here — that is the whole point.
"""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bindings" / "python"))

# PC13 blink on stm32f103 (assembled from blink.s during accuracy work:
# RCC IOPCEN, CRH push-pull, BSRR set, 50x delay, BSRR reset, 50x delay).
# Half-periods: 202/208 cyccnt alternating (DWT-verified).
BLINK_BIN = bytes([
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00, 0x0a, 0x48, 0x01, 0x68,
    0x41, 0xf0, 0x10, 0x01, 0x01, 0x60, 0x09, 0x48, 0x20, 0x21, 0x09, 0x04,
    0x01, 0x60, 0x08, 0x48, 0x01, 0x21, 0x49, 0x03, 0x01, 0x60, 0x32, 0x22,
    0x01, 0x3a, 0xfd, 0xd1, 0x09, 0x04, 0x01, 0x60, 0x32, 0x22, 0x01, 0x3a,
    0xfd, 0xd1, 0xf2, 0xe7, 0x18, 0x10, 0x02, 0x40, 0x04, 0x10, 0x01, 0x40,
    0x10, 0x10, 0x01, 0x40,
])

# Virtual-UART TX 'H' then spin (mirrors tests/host.rs).
UART_BIN = bytes([
    0x00, 0x10, 0x00, 0x20, 0x09, 0x00, 0x00, 0x00,
    0x02, 0x48, 0x48, 0x21, 0x01, 0x70, 0xFE, 0xE7,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x40,
])

VIOARM_BIN = os.environ.get("VIOARM_BIN", str(REPO / "target" / "debug" / "vioarm-cli"))


def _lib():
    for profile in ("release", "debug"):
        lib = REPO / "target" / profile / "libvioarm_cosim.so"
        if lib.exists():
            return lib
    return None


def _vcd_edges(vcd_path):
    """(ns, level) rows for c13."""
    lines = Path(vcd_path).read_text().splitlines()
    sig = next(re.match(r"\$var wire 1 (\S+) c13", ln).group(1)
               for ln in lines if re.match(r"\$var wire 1 (\S+) c13", ln))
    cur, edges = 0, []
    for ln in lines:
        if ln.startswith("#"):
            cur = int(ln[1:])
        elif ln[:1] in "01" and ln[1:] == sig:
            edges.append((cur, ln[0]))
    return edges


def test_pin_edges_match_vcd_stamps(tmp_path):
    fw = tmp_path / "blink.bin"
    fw.write_bytes(BLINK_BIN)
    vcd = tmp_path / "blink.vcd"
    run = subprocess.run(
        [VIOARM_BIN, "trace", str(fw), "--board", "stm32f103",
         "--format", "vcd", "--pin-log", "c13",
         "--output", str(vcd), "--budget", "20000"],
        capture_output=True, text=True, timeout=300)
    assert run.returncode == 0, run.stderr
    cli_edges = [(ns // 125, level) for ns, level in _vcd_edges(vcd)][1:7]

    lib = _lib()
    if lib is None:
        pytest.skip("no built libvioarm_cosim.so")
    os.environ["VIOARM_LIB"] = str(lib)
    import vioarm
    with vioarm.Machine("stm32f103", BLINK_BIN) as m:
        abi_edges = []
        for _ in range(40):
            code, events, _ = m.run_quantum(0.0001)
            for e in events:
                if e.kind == vioarm.EVENT_PIN and (e.port, e.pin) == (2, 13):
                    abi_edges.append((e.cyccnt, str(e.level)))
            if len(abi_edges) >= 6:
                break
    abi_edges = abi_edges[:8]
    assert len(abi_edges) >= 7, abi_edges
    # The ABI stream also reports driving/mode edges (e.g. the firmware's
    # CRH write at cyccnt 14, level still 0); the VCD rows level changes
    # only. Compare level transitions against the reset level instead.
    trans = [e for i, e in enumerate(abi_edges)
             if e[1] != (abi_edges[i - 1][1] if i > 0 else '0')][:6]
    assert [c for c, _ in trans] == [c for c, _ in cli_edges], (
        trans, cli_edges)
    spacings = [b - a for a, b in zip([c for c, _ in trans],
                                      [c for c, _ in trans][1:])]
    assert spacings == [202, 208, 202, 208, 202], spacings


def test_uart_drain_matches_cli(tmp_path):
    fw = tmp_path / "uart.bin"
    fw.write_bytes(UART_BIN)
    run = subprocess.run(
        [VIOARM_BIN, "run", str(fw), "--board", "virtual", "--budget", "100000"],
        capture_output=True, text=True, timeout=300)
    assert run.returncode == 0, run.stderr
    cli_line = next(line for line in run.stdout.splitlines()
                    if line.startswith("uart:"))
    assert "H" in cli_line, run.stdout

    lib = _lib()
    if lib is None:
        pytest.skip("no built libvioarm_cosim.so")
    os.environ["VIOARM_LIB"] = str(lib)
    import vioarm
    with vioarm.Machine("virtual", UART_BIN) as m:
        assert m.step(0.001) == vioarm.STEP_OK
        data = m.drain("uart")
    assert data == b"H", data
