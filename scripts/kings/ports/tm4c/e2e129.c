/* Tiva-129 kings e2e: UART hello + GPIOF loopback + TIMER0 timeout, all
 * polled, deterministic output. 129 differences from `e2e.c`: GPIOF is
 * AHB-only (`0x4005_D000`). */
#include "board.h"

#define SYSCTL_RCGCGPIO (*(volatile uint32_t *)0x400FE608u)

#define GPIOF_BASE 0x4005D000u
#define GPIOF_DATA (*(volatile uint32_t *)(GPIOF_BASE + 0x3FCu))
#define GPIOF_DIR  (*(volatile uint32_t *)(GPIOF_BASE + 0x400u))
#define GPIOF_DEN  (*(volatile uint32_t *)(GPIOF_BASE + 0x51Cu))

#define TIMER0_BASE 0x40030000u
#define TIMER0_TAMR  (*(volatile uint32_t *)(TIMER0_BASE + 0x004u))
#define TIMER0_CTL   (*(volatile uint32_t *)(TIMER0_BASE + 0x00Cu))
#define TIMER0_RIS   (*(volatile uint32_t *)(TIMER0_BASE + 0x01Cu))
#define TIMER0_ICR   (*(volatile uint32_t *)(TIMER0_BASE + 0x024u))
#define TIMER0_TAILR (*(volatile uint32_t *)(TIMER0_BASE + 0x028u))

int main(void) {
    board_init();
    uart_puts("E2E_UART_OK\n");

    SYSCTL_RCGCGPIO |= 1u << 5;
    GPIOF_DIR |= 1u << 1;
    GPIOF_DEN |= 1u << 1;
    GPIOF_DATA |= 1u << 1;
    if (GPIOF_DATA & (1u << 1)) uart_puts("E2E_GPIO_OK\n");

    TIMER0_TAMR = 0x2; /* periodic */
    TIMER0_TAILR = 1000;
    TIMER0_CTL = 1u; /* TAEN */
    while (!(TIMER0_RIS & 1u)) { }
    TIMER0_ICR = 1u;
    uart_puts("E2E_TIMER_OK\n");

    uart_puts("E2E_DONE\n");
    for (;;) { }
}
