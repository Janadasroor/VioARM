/* Host shim for the TFLM x86 reference build: UART -> stdout. */
#include <stdint.h>
#include <stdio.h>

#include "board.h"

void board_init(void) {}
uint32_t cycnt(void) { return 0; }
void uart_putc(char c) { putchar(c); }
void uart_puts(const char* s) { fputs(s, stdout); }
void uart_puthex32(uint32_t v) { printf("0x%08x", v); }
void uart_putdec32(uint32_t v) { printf("%u", v); }
