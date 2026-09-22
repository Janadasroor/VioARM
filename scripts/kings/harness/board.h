/* Board support for kings runs on stm32f103 @ 8 MHz HSI (no PLL).
 * USART1 TX (PA9) polled output, DWT CYCCNT timing, freestanding libc
 * shims so no semihosting/syscalls are needed. */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CPU_HZ 8000000u

void board_init(void);        /* clocks + USART1 115200 8N1 + DWT CYCCNT */
uint32_t cycnt(void);         /* DWT CYCCNT (0 if unit absent, e.g. QEMU) */
void uart_putc(char c);
void uart_puts(const char *s);
void uart_puthex32(uint32_t v);
void uart_putdec32(uint32_t v);

#ifdef __cplusplus
}
#endif

#endif
