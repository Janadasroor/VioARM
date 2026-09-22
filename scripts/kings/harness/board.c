/* Board support for kings runs: stm32f103/f107 @ 8 MHz HSI (no PLL),
 * USART1 TX (PA9) polled output, DWT CYCCNT timing, newlib-nano syscalls.
 *
 * Two hard-won rules live here:
 * - `board_init()` must run before ANY output or timing call: with UE=0
 *   the TXE poll never sets (infinite hang), and CYCCNT never counts.
 * - String/memory ops come from newlib-nano (tuned); hand-rolled byte
 *   loops cost ~20% on string-heavy benchmarks, and a freestanding
 *   `memset` loop gets miscompiled into a self-call without
 *   -ffreestanding -fno-builtin. Only syscalls are stubbed here. */
#include "board.h"
#include <stddef.h>

#define RCC_BASE   0x40021000u
#define RCC_APB2ENR (*(volatile uint32_t *)(RCC_BASE + 0x18u))
#define GPIOA_BASE 0x40010800u
#define GPIOA_CRH  (*(volatile uint32_t *)(GPIOA_BASE + 0x04u))
#define USART1_BASE 0x40013800u
#define USART1_SR  (*(volatile uint32_t *)(USART1_BASE + 0x00u))
#define USART1_DR  (*(volatile uint32_t *)(USART1_BASE + 0x04u))
#define USART1_BRR (*(volatile uint32_t *)(USART1_BASE + 0x08u))
#define USART1_CR1 (*(volatile uint32_t *)(USART1_BASE + 0x0Cu))

#define DEMCR      (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)

void SystemInit(void) { /* HSI already on at reset; nothing to do */ }

void board_init(void) {
    /* IOPA + AFIO + USART1 clocks */
    RCC_APB2ENR |= (1u << 2) | (1u << 0) | (1u << 14);
    /* PA9 (CRH bits 4..7): alternate-function push-pull, 50 MHz (0xB) */
    GPIOA_CRH = (GPIOA_CRH & ~(0xFu << 4)) | (0xBu << 4);
    /* 8 MHz / 115200 = 69.44 -> mantissa 69, frac 7 */
    USART1_BRR = (69u << 4) | 7u;
    USART1_CR1 = (1u << 13) | (1u << 3); /* UE + TE */
    /* Enable DWT cycle counter */
    DEMCR |= (1u << 24);
    DWT_CTRL |= 1u;
    DWT_CYCCNT = 0;
}

uint32_t cycnt(void) { return DWT_CYCCNT; }

void uart_putc(char c) {
    if (c == '\n') uart_putc('\r');
    while (!(USART1_SR & (1u << 7))) { /* TXE */ }
    USART1_DR = (uint32_t)(unsigned char)c;
}

void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

void uart_puthex32(uint32_t v) {
    static const char h[] = "0123456789abcdef";
    uart_puts("0x");
    for (int i = 7; i >= 0; i--) uart_putc(h[(v >> (i * 4)) & 0xFu]);
}

void uart_putdec32(uint32_t v) {
    char buf[11];
    int n = 0;
    if (v == 0) { uart_putc('0'); return; }
    while (v) { buf[n++] = (char)('0' + v % 10); v /= 10; }
    while (n--) uart_putc(buf[n]);
}

/* newlib-nano syscalls: polled UART, heap between _heap_start/_heap_end. */
extern char _heap_start, _heap_end;
void *_sbrk(intptr_t incr) {
    static char *brk = 0;
    if (!brk) brk = &_heap_start;
    if (brk + incr > &_heap_end) return (void *)-1;
    char *prev = brk;
    brk += incr;
    return prev;
}
int _write(int fd, const void *buf, unsigned long n) {
    (void)fd;
    const char *p = (const char *)buf;
    for (unsigned long i = 0; i < n; i++) uart_putc(p[i]);
    return (int)n;
}
int _read(int fd, void *buf, unsigned long n) {
    (void)fd; (void)buf; (void)n; return 0;
}
int _close(int fd) { (void)fd; return 0; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return 0; }
int _getpid(void) { return 1; }
int _fstat(int fd, void *buf) { (void)fd; (void)buf; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd, int off, int wh) { (void)fd; (void)off; (void)wh; return 0; }
void _exit(int code) { (void)code; for (;;) { __asm__ volatile("bkpt #0"); } }
