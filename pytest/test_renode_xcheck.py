"""Cross-check against Renode (independent STM32 implementation).

Builds bare-metal firmware images, runs each on VioARM (``stm32f407``)
and on Renode (``stm32f4_discovery``), and asserts the TX byte streams
are identical. Skips when the ARM toolchain or the Renode binary is
absent (CI has neither); set ``RENODE_BIN`` to point at a ``renode``
executable (portable builds work, no install needed).

Shapes (all M0-encoded, header-free, vectors-first images):
- basic USART2 TX (``test_renode_uart_matches_vioarm``);
- GPIO modeling: `GPIOD` pin 12 toggled with `ODR` read back over USART2
  (``test_renode_gpio_matches_vioarm``);
- timer modeling: `TIM2` update events polled and counted over USART2
  (``test_renode_timer_matches_vioarm``; `ARR=99` — `ARR=0` never raises
  `UIF` in Renode's timer model).
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

# GPIO shape: PD12 to output, toggle with ODR read-back over USART2.
GPIO_FIRMWARE = r"""
#define REG32(a) (*(volatile unsigned *)(a))
#define RCC_AHB1ENR REG32(0x40023830u)
#define RCC_APB1ENR REG32(0x40023840u)
#define GPIOA_MODER REG32(0x40020000u)
#define GPIOA_AFRL REG32(0x40020020u)
#define GPIOD_MODER REG32(0x40020C00u)
#define GPIOD_ODR REG32(0x40020C14u)
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

static void txhex(unsigned v)
{
    tx("0123456789ABCDEF"[v & 15u]);
}

void reset(void)
{
    RCC_AHB1ENR |= (1u << 0) | (1u << 3);
    RCC_APB1ENR |= 1u << 17;
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 4)) | (2u << 4);
    GPIOA_AFRL = (GPIOA_AFRL & ~(0xFu << 8)) | (7u << 8);
    USART2_BRR = 0x8AEu;
    USART2_CR1 = (1u << 13) | (1u << 3);
    GPIOD_MODER = (GPIOD_MODER & ~(3u << 24)) | (1u << 24);
    for (unsigned i = 0; i < 4u; i++) {
        GPIOD_ODR |= 1u << 12;
        unsigned v = GPIOD_ODR;
        txhex((v >> 12) & 15u); txhex((v >> 8) & 15u);
        txhex((v >> 4) & 15u); txhex(v & 15u); tx(';');
        GPIOD_ODR &= ~(1u << 12);
    }
    tx('\n');
    for (;;) {
    }
}
"""

# Timer shape: TIM2 free-runs (PSC=0, ARR=99); four update events counted
# over USART2. The count is exact on both simulators (timing-independent);
# only the event count is compared, never CNT (racy by construction).
TIMER_FIRMWARE = r"""
#define REG32(a) (*(volatile unsigned *)(a))
#define RCC_AHB1ENR REG32(0x40023830u)
#define RCC_APB1ENR REG32(0x40023840u)
#define GPIOA_MODER REG32(0x40020000u)
#define GPIOA_AFRL REG32(0x40020020u)
#define USART2_SR REG32(0x40004400u)
#define USART2_DR REG32(0x40004404u)
#define USART2_BRR REG32(0x40004408u)
#define USART2_CR1 REG32(0x4000440Cu)
#define TIM2_CR1 REG32(0x40000000u)
#define TIM2_SR REG32(0x40000010u)
#define TIM2_EGR REG32(0x40000014u)
#define TIM2_PSC REG32(0x40000028u)
#define TIM2_ARR REG32(0x4000002Cu)

static void tx(unsigned c)
{
    while ((USART2_SR & (1u << 7)) == 0) {
    }
    USART2_DR = c;
}

void reset(void)
{
    RCC_AHB1ENR |= 1u << 0;
    RCC_APB1ENR |= (1u << 17) | (1u << 0);
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 4)) | (2u << 4);
    GPIOA_AFRL = (GPIOA_AFRL & ~(0xFu << 8)) | (7u << 8);
    USART2_BRR = 0x8AEu;
    USART2_CR1 = (1u << 13) | (1u << 3);
    TIM2_PSC = 0u; TIM2_ARR = 99u; TIM2_EGR = 1u; TIM2_SR = 0u; TIM2_CR1 = 1u;
    for (unsigned i = 0; i < 4u; i++) {
        while ((TIM2_SR & 1u) == 0) {
        }
        TIM2_SR = 0u;
        tx('0' + i);
    }
    tx('\n');
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


def build_objects(tmp, source=FIRMWARE):
    gcc = shutil.which("arm-none-eabi-gcc")
    ld = shutil.which("arm-none-eabi-ld")
    if not (gcc and ld):
        pytest.skip("arm-none-eabi toolchain not on PATH")
    src = tmp / "xcheck.c"
    obj = tmp / "xcheck.o"
    src.write_text(source)
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


def check_shape(tmp_path, repl, source, tag):
    """Run one firmware shape on both simulators, compare TX bytes."""
    tmp = Path(tempfile.mkdtemp(prefix=f"vioarm-xcheck-{tag}-"))
    obj = build_objects(tmp, source)

    # VioARM artifact (vectors-first image at base 0, like the harness).
    elf0 = link_elf(obj, tmp, f"vio-{tag}", "0x00000000", "0x20001000")
    vio_bin = tmp / f"vio-{tag}.bin"
    subprocess.run(
        [shutil.which("arm-none-eabi-objcopy"), "--output-target=binary",
         str(elf0), str(vio_bin)],
        check=True, capture_output=True, timeout=60,
    )
    rc, out, err = harness.run_board(str(vio_bin), board="stm32f407",
                                     base=0, budget=2_000_000)
    vio_lines = harness.channel_text(out, "uart:")
    assert vio_lines, f"no uart drain on VioARM:\n{out}\n{err}"
    vio_bytes = "".join(
        line.split("uart:", 1)[1][1:] + "\n" for line in vio_lines
    ).encode()

    # Renode artifact (same source, linked at real flash).
    elf8 = link_elf(obj, tmp, f"ren-{tag}", "0x08000000", "0x20020000")
    uart_log = tmp / f"renode-{tag}-uart.log"
    resc = tmp / f"xcheck-{tag}.resc"
    resc.write_text(RESC.format(repl=repl, elf=elf8, uart=uart_log))
    subprocess.run(
        [RENODE_BIN, "--console", "-e", f"include @{resc}"],
        capture_output=True, timeout=300, check=False,
    )
    assert uart_log.is_file(), "renode produced no UART log"
    renode_bytes = uart_log.read_bytes()
    assert renode_bytes == vio_bytes, (
        f"UART mismatch [{tag}]: renode={renode_bytes!r} vioarm={vio_bytes!r}"
    )


def _repl_or_skip():
    if RENODE_BIN is None:
        pytest.skip("renode binary not found (set RENODE_BIN)")
    repl = (Path(RENODE_BIN).parent / "platforms" / "boards"
            / "stm32f4_discovery.repl")
    if not repl.is_file():
        pytest.skip(f"renode platforms missing next to {RENODE_BIN}")
    return repl


def test_renode_gpio_matches_vioarm(tmp_path):
    repl = _repl_or_skip()
    check_shape(tmp_path, repl, GPIO_FIRMWARE, "gpio")


def test_renode_timer_matches_vioarm(tmp_path):
    repl = _repl_or_skip()
    check_shape(tmp_path, repl, TIMER_FIRMWARE, "timer")
