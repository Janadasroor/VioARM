"""LED fade on stm32f103 PC13: on bright, gradual fade-out, dark, repeat.

Builds the firmware in C (skips without the toolchain), runs it with
``--pin-log``, and asserts the brightness shape: starts HIGH, fades
through mid duties, goes dark, and re-lights (second period).
"""

import pytest
import vioarm_harness as harness

FIRMWARE = r"""
#define RCC_APB2ENR (*(volatile unsigned *)0x40021018)
#define GPIOC_CRH   (*(volatile unsigned *)0x40011004)
#define GPIOC_BSRR  (*(volatile unsigned *)0x40011010)

static void delay(unsigned n) {
    while (n--) __asm__ volatile("" ::: "memory");
}

void reset(void) {
    RCC_APB2ENR |= 1u << 4;
    GPIOC_CRH = (GPIOC_CRH & ~(0xFu << 20)) | (0x3u << 20);
    /* Scaled-down timing for CI speed (same shape as the 2s demo):
     * ~1s fade, ~1.5s dark in sim time. */
    for (;;) {
        for (unsigned b = 47; b > 0; b--) {
            unsigned hi = b << 7, lo = 6144u - hi;
            GPIOC_BSRR = 1u << 13;
            delay(hi);
            GPIOC_BSRR = 1u << (13 + 16);
            delay(lo);
        }
        delay(64u * 6144u);
    }
}
"""


def pin_levels(out):
    """``t=`` line levels for c13 in sample order."""
    levels = []
    for line in out.splitlines():
        if line.startswith("t="):
            for tok in line.split():
                if tok.startswith("c13="):
                    levels.append(int(tok[4:]))
    return levels


def window_duties(levels, width=20):
    return [sum(levels[i:i + width]) * 100 // width
            for i in range(0, len(levels) - width + 1, width)]


def test_led_fade_shape(tmp_path):
    try:
        text, symbols = harness.build_c(FIRMWARE)
    except RuntimeError as e:
        pytest.skip(str(e))
    entry = symbols.get("reset")
    assert entry is not None, "no reset symbol in firmware"
    image = harness.image_with_vectors(text, entry)
    path = tmp_path / "led_fade.bin"
    path.write_bytes(image)
    # fade ~2.5M + dark ~2.5M instr (O0 loop is fat); 7M covers fade,
    # dark, relight (~3 min debug).
    rc, out, err = harness.run_board(
        str(path), board="stm32f103", budget=7_000_000,
        extra=("--pin-log", "c13", "--pin-sample", "25000"))
    assert rc == 0, f"run failed:\n{out}\n{err}"
    levels = pin_levels(out)
    assert len(levels) >= 70, f"too few samples: {len(levels)}"
    duties = window_duties(levels)
    assert duties[0] >= 70, f"must start bright: {duties[:3]}"
    assert any(d < 50 for d in duties[1:]), "must fade through mid duties"
    dark = next(i for i, d in enumerate(duties) if d == 0)
    relit = any(d >= 70 for d in duties[dark + 1:])
    assert relit, f"must go dark then re-light: {duties}"
