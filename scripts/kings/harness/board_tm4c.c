/* Board support for kings runs: tm4c123gh6pm @ 16 MHz PIOSC (no PLL),
 * UART0 TX (PA1) polled output, DWT CYCCNT timing, newlib-nano syscalls.
 *
 * Same two hard-won rules as the STM32 harness (see harness/board.c):
 * - `board_init()` must run before ANY output or timing call: with
 *   UARTEN=0 the TXFE poll never sets (infinite hang), and CYCCNT
 *   never counts without DEMCR.TRCENA.
 * - String/memory ops come from newlib-nano (tuned); only syscalls
 *   are stubbed here.
 *
 * Codegen note: keep -mcpu=cortex-m3 for Tiva kings binaries. M3 code
 * runs unchanged on the M4F (integer benchmarks never touch the FPU),
 * and it keeps the whole run inside the proven ARMv7-M ISA subset. */
#include "board.h"
#include <stddef.h>

/* TM4C123GH6PM @ 16 MHz PIOSC: 16e6 / (16 * 115200) = 8.6806. */
#define TM4C_CPU_HZ 16000000u

#define SYSCTL_RCGCUART (*(volatile uint32_t *)0x400FE618u)
#define SYSCTL_RCGCGPIO (*(volatile uint32_t *)0x400FE608u)

#define GPIOA_BASE  0x40004000u
#define GPIOA_DIR   (*(volatile uint32_t *)(GPIOA_BASE + 0x400u))
#define GPIOA_AFSEL (*(volatile uint32_t *)(GPIOA_BASE + 0x420u))
#define GPIOA_DEN   (*(volatile uint32_t *)(GPIOA_BASE + 0x51Cu))
#define GPIOA_PCTL  (*(volatile uint32_t *)(GPIOA_BASE + 0x52Cu))

#define UART0_BASE 0x4000C000u
#define UART0_DR   (*(volatile uint32_t *)(UART0_BASE + 0x000u))
#define UART0_FR   (*(volatile uint32_t *)(UART0_BASE + 0x018u))
#define UART0_IBRD (*(volatile uint32_t *)(UART0_BASE + 0x024u))
#define UART0_FBRD (*(volatile uint32_t *)(UART0_BASE + 0x028u))
#define UART0_LCRH (*(volatile uint32_t *)(UART0_BASE + 0x02Cu))
#define UART0_CTL  (*(volatile uint32_t *)(UART0_BASE + 0x030u))

#define DEMCR      (*(volatile uint32_t *)0xE000EDFCu)
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)

void SystemInit(void) { /* PIOSC already on at reset; no PLL */ }

void board_init(void) {
    /* UART0 + GPIOA clocks */
    SYSCTL_RCGCUART |= 1u << 0;
    SYSCTL_RCGCGPIO |= 1u << 0;
    /* PA0/PA1: alternate function, U0 (PCTL nibbles = 1), digital */
    GPIOA_AFSEL |= 0x03u;
    GPIOA_PCTL = (GPIOA_PCTL & ~0xFFu) | 0x11u;
    GPIOA_DEN |= 0x03u;
    GPIOA_DIR |= 1u << 1; /* PA1 (U0TX) out */
    /* 115200 8N1 + FIFOs; enable last, per the datasheet sequence */
    UART0_CTL = 0;
    UART0_IBRD = 8;
    UART0_FBRD = 44;
    UART0_LCRH = 0x70u;
    UART0_CTL = (1u << 0) | (1u << 8) | (1u << 9); /* UARTEN + TXE + RXE */
    /* Enable DWT cycle counter */
    DEMCR |= (1u << 24);
    DWT_CTRL |= 1u;
    DWT_CYCCNT = 0;
}

uint32_t cycnt(void) { return DWT_CYCCNT; }

void uart_putc(char c) {
    if (c == '\n') uart_putc('\r');
    while (UART0_FR & (1u << 5)) { /* TXFF */ }
    UART0_DR = (uint32_t)(unsigned char)c;
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
