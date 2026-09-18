"""E4 ABI fuzz: seeded random call sequences in a subprocess (crash
isolation) with a wall-time backstop and an RSS tripwire."""

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
FUZZER = Path(__file__).resolve().parent / "fuzz_abi.py"


def _lib():
    for profile in ("release", "debug"):
        lib = REPO / "target" / profile / "libvioarm_cosim.so"
        if lib.exists():
            return lib
    return None


def _run_fuzz(seed, iters, timeout_s=240):
    lib = _lib()
    if lib is None:
        pytest.skip("no built libvioarm_cosim.so (cargo build -p vioarm-cosim first)")
    env = dict(os.environ, VIOARM_LIB=str(lib),
               PYTHONPATH=str(REPO / "bindings" / "python"))
    run = subprocess.run(
        ["python3", str(FUZZER), "--seed", str(seed), "--iters", str(iters)],
        capture_output=True, text=True, env=env, timeout=timeout_s)
    assert run.returncode == 0, run.stdout + run.stderr
    assert "FUZZ-OK" in run.stdout, run.stdout
    match = re.search(r"peak_rss_kb=(\d+)", run.stdout)
    assert match, run.stdout
    return int(match.group(1))


def test_abi_fuzz_seed_7():
    peak = _run_fuzz(7, 3000)
    assert peak < 768 * 1024, f"runaway RSS, possible leak: {peak} KiB"


def test_abi_fuzz_seed_99():
    peak = _run_fuzz(99, 3000)
    assert peak < 768 * 1024, f"runaway RSS, possible leak: {peak} KiB"
