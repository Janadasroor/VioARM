"""E4 C conformance: compile `abi_conformance.c` against the built ABI
(header + cdylib) and run it. Covers every function, every status code,
null probes, short buffers, version gates, and snapshot round-trips —
from C, so header/representation drift fails here too."""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
INCLUDE = REPO / "vioarm-cosim" / "include"
SOURCE = Path(__file__).resolve().parent / "abi_conformance.c"


def _cdylib():
    for profile in ("release", "debug"):
        lib = REPO / "target" / profile / "libvioarm_cosim.so"
        if lib.exists():
            return lib
    return None


def test_abi_conformance(tmp_path):
    if shutil.which("cc") is None and shutil.which("gcc") is None:
        pytest.skip("no C compiler")
    lib = _cdylib()
    if lib is None:
        pytest.skip("no built libvioarm_cosim.so (cargo build -p vioarm-cosim first)")
    exe = tmp_path / "abi_conformance"
    cc = shutil.which("cc") or shutil.which("gcc")
    compile = subprocess.run(
        [cc, "-std=c11", "-Wall", "-Wextra", str(SOURCE),
         "-I", str(INCLUDE), str(lib), "-o", str(exe)],
        capture_output=True, text=True,
    )
    assert compile.returncode == 0, compile.stderr
    # Runtime resolves SONAME `libvioarm_cosim.so.1` via a staged symlink,
    # exactly like a packager layout.
    staged = tmp_path / "stagelib"
    staged.mkdir()
    shutil.copy(lib, staged / "libvioarm_cosim.so")
    (staged / "libvioarm_cosim.so.1").symlink_to("libvioarm_cosim.so")
    env = dict(os.environ, LD_LIBRARY_PATH=str(staged))
    run = subprocess.run([str(exe)], capture_output=True, text=True, env=env,
                         timeout=300)
    assert run.returncode == 0, run.stdout + run.stderr
    assert "ABI-CONFORM-OK" in run.stdout, run.stdout
    count = int(run.stdout.strip().split()[-1])
    assert count > 60, f"too few checks ran: {count}"
