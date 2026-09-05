"""Template harness for framework integration tests: build firmware images,
run them on VioARM boards through the CLI, and assert drain channels.

Copy this file into your project and adapt; the example suites next to it
show the intended shape (``test_usart_tx.py``: build C, run, assert drain;
``test_replay.py``: record/replay/mutate golden traces without a toolchain).

The CLI under test is ``$VIOARM_BIN`` (default: ``target/debug/vioarm-cli``
relative to the repo root). All CLI output is ANSI-stripped before matching.
"""

import os
import re
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
VIOARM_BIN = os.environ.get("VIOARM_BIN", str(REPO / "target" / "debug" / "vioarm-cli"))

ANSI = re.compile(rb"\x1b\[[0-9;]*m")


def vioarm(*args):
    """Run the CLI; returns ``(returncode, stdout, stderr)`` ANSI-stripped."""
    proc = subprocess.run(
        [VIOARM_BIN, *args], capture_output=True, timeout=600, check=False
    )
    out = ANSI.sub(b"", proc.stdout).decode("utf-8", "replace")
    err = ANSI.sub(b"", proc.stderr).decode("utf-8", "replace")
    return proc.returncode, out, err


def gcc_available():
    """arm-none-eabi-gcc for compiling CMSIS firmware."""
    return shutil.which("arm-none-eabi-gcc") is not None


def pack_include_dirs():
    """Vendored F1 + core CMSIS include dirs, or ``None`` when absent.

    Note: the ``STM32F4xx_CMSIS`` path older Rust tests reference no longer
    exists in-tree; the ARM CMSIS pack carries the same ``core_cm3.h``.
    """
    f1 = REPO / "packs" / "Keil.STM32F1xx_DFP" / "Device" / "Include"
    core = REPO / "packs" / "ARM.CMSIS" / "CMSIS" / "Core" / "Include"
    if f1.is_dir() and core.is_dir():
        return [str(f1), str(core)]
    return None


def build_c(source, mcpu="cortex-m0"):
    """Compile CMSIS C to ``(text_bytes, symbols)``.

    Raises ``RuntimeError`` when the toolchain is missing (callers usually
    turn that into ``pytest.skip``).
    """
    gcc = shutil.which("arm-none-eabi-gcc")
    objcopy = shutil.which("arm-none-eabi-objcopy") or shutil.which("llvm-objcopy")
    nm = shutil.which("arm-none-eabi-nm") or shutil.which("nm")
    if not (gcc and objcopy and nm):
        raise RuntimeError("arm-none-eabi toolchain not on PATH")
    includes = pack_include_dirs()
    if includes is None:
        raise RuntimeError("vendored pack includes not found")
    tmp = Path(tempfile.mkdtemp(prefix="vioarm-pytest-"))
    src = tmp / "firmware.c"
    obj = tmp / "firmware.o"
    out = tmp / "firmware.bin"
    src.write_text(source)
    subprocess.run(
        [gcc, f"-mcpu={mcpu}", "-mthumb", "-ffreestanding", "-nostdlib",
         "-O0", "-g0", "-c", str(src), "-o", str(obj)]
        + [flag for inc in includes for flag in ("-I", inc)],
        check=True,
        capture_output=True,
        timeout=300,
    )
    listing = subprocess.run([nm, "-n", str(obj)], check=True,
                             capture_output=True, timeout=60,
                             text=True).stdout
    symbols = {}
    for line in listing.splitlines():
        parts = line.split()
        if len(parts) == 3:
            try:
                symbols[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    subprocess.run([objcopy, "--output-target=binary", "-j", ".text",
                    str(obj), str(out)],
                   check=True, capture_output=True, timeout=60)
    return out.read_bytes(), symbols


def image_with_vectors(text, reset_offset, base=0, sp=0x20001000):
    """Prepend an SP + reset-vector table (Thumb bit set) to raw ``.text``."""
    vectors = struct.pack("<II", sp, base + 8 + reset_offset | 1)
    return vectors + text


def write_image(image_bytes, suffix=".bin"):
    """Spill ``image_bytes`` to a temp file; returns its path string."""
    tmp = Path(tempfile.mkdtemp(prefix="vioarm-pytest-"))
    path = tmp / f"image{suffix}"
    path.write_bytes(image_bytes)
    return str(path)


def run_board(image_path, board="stm32f103", base=0, budget=1_000_000,
              semihost=False, extra=()):
    """``vioarm run`` an image file; returns ``(rc, out, err)``."""
    args = ["run", image_path, "--board", board, "--base", str(base),
            "--budget", str(budget)]
    if semihost:
        args.append("--semihost")
    args.extend(extra)
    return vioarm(*args)


def channel_text(out, name="uart:"):
    """Drain-channel payload lines for ``name`` (e.g. ``uart:``, ``semihost:``)."""
    return [line for line in out.splitlines() if name in line]


def record(image_path, board="stm32f103", base=0, budget=1_000_000,
           semihost=False, snaps=(), output=None):
    """``vioarm record`` into ``output`` (an explicit path); returns
    ``(rc, out, err)``."""
    args = ["record", image_path, "--board", board, "--base", str(base),
            "--budget", str(budget), "--output", output]
    if semihost:
        args.append("--semihost")
    for span in snaps:
        args += ["--snap", span]
    return vioarm(*args)


def replay(image_path, golden_path, board="stm32f103", base=0, patches=()):
    """``vioarm replay``; returns ``(rc, out, err)`` (rc 0 == MATCH)."""
    args = ["replay", image_path, golden_path, "--board", board,
            "--base", str(base)]
    for patch in patches:
        args += ["--patch", patch]
    return vioarm(*args)
