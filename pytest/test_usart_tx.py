"""Example suite: build a USART transmitter in C, run it on stm32f103,
assert the drained channel. Template for build-image → run → assert-drain
firmware tests."""

import pytest
import vioarm_harness as harness

FIRMWARE = r"""
#define STM32F10X_HD
#include "stm32f10x.h"

static void tx_byte(unsigned c) {
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = c;
}

void reset(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 4)) | (0xBu << 4);
    GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 8)) | (0x4u << 8);
    USART1->BRR = 0x341;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    tx_byte('o'); tx_byte('k'); tx_byte('!'); tx_byte('\n');
    __asm__ volatile("bkpt #0");
}
"""


def test_usart_tx_drain(tmp_path):
    try:
        text, symbols = harness.build_c(FIRMWARE)
    except RuntimeError as e:
        pytest.skip(str(e))
    entry = symbols.get("reset")
    assert entry is not None, "no reset symbol in firmware"
    image = harness.image_with_vectors(text, entry)
    path = tmp_path / "usart_tx.bin"
    path.write_bytes(image)
    rc, out, err = harness.run_board(str(path), board="stm32f103", budget=50_000)
    lines = harness.channel_text(out, "uart:")
    assert lines, f"no uart drain channel in output:\n{out}\n{err}"
    assert any("ok!" in line for line in lines), f"TX bytes missing: {lines}"
