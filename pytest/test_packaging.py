"""E2 packaging suite: the C/Python host examples build and run against
the built ABI (installed layout or in-tree paths), and the CMake find
module configures end to end. Toolchains may skip gracefully."""

import os
import shutil
import subprocess
from pathlib import Path

import pytest
import vioarm_harness as harness

REPO = Path(__file__).resolve().parent.parent
EXAMPLES = REPO / "packaging" / "examples"
INCLUDE = REPO / "vioarm-cosim" / "include"


def _cdylib():
    for profile in ("release", "debug"):
        lib = REPO / "target" / profile / "libvioarm_cosim.so"
        if lib.exists():
            return lib
    return None


def _need_cdylib():
    lib = _cdylib()
    if lib is None:
        pytest.skip("no built libvioarm_cosim.so (cargo build -p vioarm-cosim first)")
    return lib


def test_c_example_builds_and_runs(tmp_path):
    if shutil.which("cc") is None and shutil.which("gcc") is None:
        pytest.skip("no C compiler")
    lib = _need_cdylib()
    exe = tmp_path / "host_minimal"
    cc = shutil.which("cc") or shutil.which("gcc")
    compile = subprocess.run(
        [cc, str(EXAMPLES / "host_minimal.c"), "-I", str(INCLUDE),
         str(lib), "-o", str(exe)],
        capture_output=True, text=True,
    )
    assert compile.returncode == 0, compile.stderr
    # The link records SONAME `libvioarm_cosim.so.1`: stage it the way a
    # packager would (real file + version symlink) and prove the chain.
    staged = tmp_path / "stagelib"
    staged.mkdir()
    shutil.copy(lib, staged / "libvioarm_cosim.so")
    (staged / "libvioarm_cosim.so.1").symlink_to("libvioarm_cosim.so")
    env = dict(os.environ, LD_LIBRARY_PATH=str(staged))
    run = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
    assert run.returncode == 0, run.stderr
    assert "pin0=0" in run.stdout, run.stdout
    assert run.stdout.startswith("pc="), run.stdout


def test_python_example_runs():
    lib = _need_cdylib()
    env = dict(os.environ, VIOARM_LIB=str(lib),
               PYTHONPATH=str(REPO / "bindings" / "python"))
    run = subprocess.run(
        ["python3", str(EXAMPLES / "host_minimal.py")],
        capture_output=True, text=True, env=env,
    )
    if run.returncode != 0 and "libvioarm_cosim" in run.stderr:
        pytest.skip(f"loader refused the library: {run.stderr.strip()}")
    assert run.returncode == 0, run.stderr
    assert "pin0=0" in run.stdout, run.stdout


def test_cmake_find_module_configures(tmp_path):
    if shutil.which("cmake") is None:
        pytest.skip("no cmake")
    if shutil.which("cc") is None and shutil.which("gcc") is None:
        pytest.skip("no C compiler")
    lib = _need_cdylib()
    # Staged install layout: <stage>/{include,lib}.
    stage = tmp_path / "stage"
    (stage / "include").mkdir(parents=True)
    (stage / "lib").mkdir()
    shutil.copy(INCLUDE / "vioarm_cosim.h", stage / "include")
    shutil.copy(lib, stage / "lib" / "libvioarm_cosim.so")
    (stage / "lib" / "libvioarm_cosim.so.1").symlink_to("libvioarm_cosim.so")
    build = tmp_path / "build"
    configure = subprocess.run(
        ["cmake", "-S", str(EXAMPLES), "-B", str(build),
         f"-DVioARM_ROOT={stage}"],
        capture_output=True, text=True,
    )
    assert configure.returncode == 0, configure.stderr
    assert "Found VioARM" in configure.stdout + configure.stderr
    build_run = subprocess.run(
        ["cmake", "--build", str(build)], capture_output=True, text=True)
    assert build_run.returncode == 0, build_run.stderr
    exe = next(build.rglob("host_minimal"), None)
    assert exe is not None, "example binary built"
    env = dict(os.environ, LD_LIBRARY_PATH=str(stage / "lib"))
    run = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
    assert run.returncode == 0, run.stderr
    assert "pin0=0" in run.stdout, run.stdout


def _rate_env():
    lib = _need_cdylib()
    return dict(os.environ, VIOARM_LIB=str(lib),
                PYTHONPATH=str(REPO / "bindings" / "python"))


def _rate_summary(args):
    run = subprocess.run(
        ["python3", str(EXAMPLES / "rate_limited.py")] + args,
        capture_output=True, text=True, env=_rate_env())
    assert run.returncode == 0, run.stderr
    summary = next(line for line in run.stdout.splitlines()
                   if line.startswith("SUMMARY"))
    fields = dict(part.split("=") for part in summary.split()[1:])
    return {key: float(value) for key, value in fields.items()}


def test_rate_limiter_holds_a_holdable_cadence():
    summary = _rate_summary(["--scale", "0.01", "--quanta", "20",
                             "--quantum-us", "100"])
    assert summary["max_abs_drift_ms"] < 250.0, summary
    assert abs(summary["lag_ms"]) < 250.0, summary


def test_rate_limiter_reports_lag_without_spiraling():
    summary = _rate_summary(["--scale", "1.0", "--quanta", "10",
                             "--quantum-us", "100"])
    assert summary["lag_ms"] > 0, summary
    assert summary["quanta"] == 10
