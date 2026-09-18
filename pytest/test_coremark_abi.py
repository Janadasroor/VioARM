"""E4 CoreMark-via-ABI: drive a CoreMark `.bin` through the C ABI and
diff against the CLI result on the same binary. Any CLI/embedding
divergence fails — "if they ever disagree, the embedding layer lied".

Opt-in (slow): set `COREMARK_BIN` to a flash-alias raw binary built like
`/tmp` notes (arm-none-eabi-gcc, `-T` flash at 0x0, fixed seeds). Skips
otherwise. Prefers the release CLI when present; either way the two
paths must print identical CRC lines and identical total ticks. When the
binary reports the standard 2K-performance profile, the CRCs must also
equal the published known-good (e714/1fd7/8e3a).
"""

import os
import re
import subprocess
import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
COREMARK_BIN = os.environ.get("COREMARK_BIN")
VIOARM_BIN = os.environ.get("VIOARM_BIN", str(REPO / "target" / "debug" / "vioarm-cli"))

KNOWN_2K_PERF = {"list": "e714", "matrix": "1fd7", "state": "8e3a"}


def _release_cli():
    candidate = REPO / "target" / "release" / "vioarm-cli"
    if candidate.exists():
        return str(candidate)
    return VIOARM_BIN


def _parse_coremark(text):
    """(crc dict, total_ticks or None, validated bool)."""
    crcs = dict(re.findall(r"\[0\]crc(list|matrix|state)\s*:\s*0x([0-9a-f]+)", text))
    ticks = re.search(r"Total ticks\s*:\s*(\d+)", text)
    return crcs, int(ticks.group(1)) if ticks else None, \
        "Correct operation validated" in text


def test_coremark_via_abi_matches_cli():
    if not COREMARK_BIN or not Path(COREMARK_BIN).exists():
        pytest.skip("set COREMARK_BIN to a flash-alias CoreMark .bin to run")
    fw = Path(COREMARK_BIN).read_bytes()

    # CLI side: bounded budget, parsed after exit. (The firmware spins
    # after main returns and the CLI drains UART only at the end, so
    # streaming cannot detect completion early — size the budget past
    # the benchmark instead. Too small a budget fails below with a
    # clear message rather than hanging.)
    cli = subprocess.run(
        [_release_cli(), "run", str(COREMARK_BIN), "--board", "stm32f103",
         "--budget", "150000000"],
        capture_output=True, text=True, timeout=1500)
    assert cli.returncode == 0, cli.stderr
    cli_text = cli.stdout + cli.stderr
    cli_crcs, cli_ticks, _ = _parse_coremark(cli_text)
    assert cli_crcs, (
        "CLI run produced no CRC lines — binary needs >150M steps; "
        "raise the budget"
    )

    sys.path.insert(0, str(REPO / "bindings" / "python"))
    os.environ["VIOARM_LIB"] = str(
        next(p for p in (REPO / "target" / "release" / "libvioarm_cosim.so",
                         REPO / "target" / "debug" / "libvioarm_cosim.so")
             if p.exists()))
    import vioarm
    out_text = ""
    deadline = time.monotonic() + 1200
    with vioarm.Machine("stm32f103", fw) as m:
        for _ in range(2000):
            assert m.step(0.05) == vioarm.STEP_OK
            data = m.drain("usart1")
            out_text += data.decode("utf-8", "replace")
            if "Errors detected" in out_text or "Correct operation validated" in out_text:
                break
            assert time.monotonic() < deadline, "ABI run overran its wall budget"
    assert "Iterations/Sec" in out_text, "benchmark did not finish via ABI"
    abi_crcs, abi_ticks, _ = _parse_coremark(out_text)
    assert abi_crcs == cli_crcs, (abi_crcs, cli_crcs)
    assert abi_ticks is not None and abi_ticks == cli_ticks, (abi_ticks, cli_ticks)
    if "2K performance run parameters" in out_text:
        for key, want in KNOWN_2K_PERF.items():
            assert abi_crcs.get(key) == want, (key, abi_crcs)
