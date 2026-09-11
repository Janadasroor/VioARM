"""Cross-check against Renode (independent STM32 implementation).

Builds one bare-metal USART2 firmware image, runs it on VioARM
(``stm32f407``) and on Renode (``stm32f4_discovery``), and asserts the
TX byte streams are identical. Skips when the ARM toolchain or the
Renode binary is absent (CI has neither); set ``RENODE_BIN`` to point
at a ``renode`` executable (portable builds work, no install needed).
"""

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest
import vioarm_harness as harness

RENODE_BIN = os.environ.get("RENODE_BIN") or shutil.which("renode")

# F407 USART2 TX, header-free (RM0090 offsets in comments), M0 encoding
# so the image needs no literal pools beyond PC-relative data.
FIRMWARE = r"""
#define REG32(a) (*(volatile unsigned *)(a))
#define RCC_AHB1ENR REG32(0x40023830u)
#define RCC_APB1ENR REG32(0x40023840u)
#define GPIOA_MODER REG32(0x40020000u)
#define GPIOA_AFRL REG32(0x40020020u)
#define USART2_SR REG32(0x40004400u)
#define USART2_DR REG32(0x40004404u)
#define USART2_BRR REG32(0x40004408u)
#define USART2_CR1 REG32(0x4000440Cu)

static void tx(unsigned c)
{
    while ((USART2_SR & (1u << 7)) == 0) {
    }
    USART2_DR = c;
}

void reset(void)
{
    RCC_AHB1ENR |= 1u << 0;
    RCC_APB1ENR |= 1u << 17;
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 4)) | (2u << 4);
    GPIOA_AFRL = (GPIOA_AFRL & ~(0xFu << 8)) | (7u << 8);
    USART2_BRR = 0x8AEu;
    USART2_CR1 = (1u << 13) | (1u << 3);
    tx('o'); tx('k'); tx('!'); tx('\n');
    for (;;) {
    }
}
"""

LD_SCRIPT = """\
ENTRY(reset)
MEMORY {{
  FLASH(rx) : ORIGIN = {flash} LENGTH = 64K
  RAM(rwx) : ORIGIN = 0x20000000 LENGTH = 128K
}}
SECTIONS {{
  .isr_vector : {{ LONG({sp}); LONG(reset + 1); }} >FLASH
  .text : {{ *(.text*) *(.rodata*) }} >FLASH
}}
"""

RESC = """\
mach create
machine LoadPlatformDescription @{repl}
sysbus LoadELF @{elf}
sysbus.usart2 CreateFileBackend @{uart} true
start
"""


def build_objects(tmp):
    gcc = shutil.which("arm-none-eabi-gcc")
    ld = shutil.which("arm-none-eabi-ld")
    if not (gcc and ld):
        pytest.skip("arm-none-eabi toolchain not on PATH")
    src = tmp / "xcheck.c"
    obj = tmp / "xcheck.o"
    src.write_text(FIRMWARE)
    subprocess.run(
        [gcc, "-mcpu=cortex-m0", "-mthumb", "-ffreestanding", "-nostdlib",
         "-O0", "-g0", "-c", str(src), "-o", str(obj)],
        check=True, capture_output=True, timeout=300,
    )
    return obj


def link_elf(obj, tmp, name, flash, sp):
    ld = shutil.which("arm-none-eabi-ld")
    (tmp / f"{name}.ld").write_text(LD_SCRIPT.format(flash=flash, sp=sp))
    elf = tmp / f"{name}.elf"
    subprocess.run(
        [ld, "-T", str(tmp / f"{name}.ld"), str(obj), "-o", str(elf)],
        check=True, capture_output=True, timeout=120,
    )
    return elf


def test_renode_uart_matches_vioarm(tmp_path):
    if RENODE_BIN is None:
        pytest.skip("renode binary not found (set RENODE_BIN)")
    repl = (Path(RENODE_BIN).parent / "platforms" / "boards"
            / "stm32f4_discovery.repl")
    if not repl.is_file():
        pytest.skip(f"renode platforms missing next to {RENODE_BIN}")
    tmp = Path(tempfile.mkdtemp(prefix="vioarm-xcheck-"))
    obj = build_objects(tmp)

    # VioARM artifact (vectors-first image at base 0, like the harness).
    elf0 = link_elf(obj, tmp, "vio", "0x00000000", "0x20001000")
    vio_bin = tmp / "vio.bin"
    subprocess.run(
        [shutil.which("arm-none-eabi-objcopy"), "--output-target=binary",
         str(elf0), str(vio_bin)],
        check=True, capture_output=True, timeout=60,
    )
    rc, out, err = harness.run_board(str(vio_bin), board="stm32f407",
                                     base=0, budget=2_000_000)
    vio_lines = harness.channel_text(out, "uart:")
    assert vio_lines, f"no uart drain on VioARM:\n{out}\n{err}"
    # Drain lines look like `uart: <payload>`; strip the prefix per line.
    vio_bytes = "".join(
        line.split("uart:", 1)[1][1:] + "\n" for line in vio_lines
    ).encode()

    # Renode artifact (same source, linked at real flash).
    elf8 = link_elf(obj, tmp, "ren", "0x08000000", "0x20020000")
    uart_log = tmp / "renode_uart.log"
    resc = tmp / "xcheck.resc"
    resc.write_text(RESC.format(repl=repl, elf=elf8, uart=uart_log))
    subprocess.run(
        [RENODE_BIN, "--console", "-e", f"include @{resc}"],
        capture_output=True, timeout=300, check=False,
    )
    assert uart_log.is_file(), "renode produced no UART log"
    renode_bytes = uart_log.read_bytes()
    assert renode_bytes == vio_bytes, (
        f"UART mismatch: renode={renode_bytes!r} vioarm={vio_bytes!r}"
    )
