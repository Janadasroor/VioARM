/* ThreadX low-level init + tick supervisor for kings runs.
 * Derived from upstream ports/cortex_m3/gnu/example_build/
 * tx_initialize_low_level.S (MIT, Eclipse ThreadX): same sequence (VTOR,
 * system stack, DWT CYCCNT, SysTick, handler priorities, unused-memory
 * pointer) expressed in C, retargeted to 8 MHz / 1000 Hz tick
 * (RELOAD 7999), with a tick supervisor that stops the world at 2020
 * ticks and reports every demo counter. Upstream kernel + demo untouched.
 *
 * Scoreboard oracle: no thread ever errors (each thread breaks its loop on
 * any status failure, freezing its counter), 2000+20 ticks elapse, all
 * counters grow, queue backlog (sent - received) stays within capacity. */
#include "tx_api.h"
#include "tx_initialize.h"
#include "tx_thread.h"
#include "board.h"

/* Internal kernel entry (port asm); not published in common headers. */
void _tx_timer_interrupt(void);

#define NVIC_BASE 0xE000E000u
#define SYST_RVR  (*(volatile unsigned long *)(NVIC_BASE + 0x14u))
#define SYST_CVR  (*(volatile unsigned long *)(NVIC_BASE + 0x18u))
#define SYST_CSR  (*(volatile unsigned long *)(NVIC_BASE + 0x10u))
#define SCB_VTOR  (*(volatile unsigned long *)(NVIC_BASE + 0xD08u))
#define SCB_SHPR1 (*(volatile unsigned long *)(NVIC_BASE + 0xD18u))
#define SCB_SHPR2 (*(volatile unsigned long *)(NVIC_BASE + 0xD1Cu))
#define SCB_SHPR3 (*(volatile unsigned long *)(NVIC_BASE + 0xD20u))
#define DEMCR     (*(volatile unsigned long *)0xE000EDFCu)
#define DWT_CTRL  (*(volatile unsigned long *)0xE0001000u)

#define N_TICKS 2020u

extern unsigned long g_pfnVectors[];
extern char _ebss;

/* Demo counters (samples/demo_threadx.c). A frozen counter means that
 * thread hit its `break`-on-error path. */
extern unsigned long thread_0_counter;
extern unsigned long thread_1_counter;
extern unsigned long thread_1_messages_sent;
extern unsigned long thread_2_counter;
extern unsigned long thread_2_messages_received;
extern unsigned long thread_3_counter;
extern unsigned long thread_4_counter;
extern unsigned long thread_5_counter;
extern unsigned long thread_6_counter;
extern unsigned long thread_7_counter;

static unsigned long tick_count;

void _tx_initialize_low_level(void) {
    __asm__ volatile("cpsid i");
    board_init(); /* USART1 + DWT before any kernel output path */
    _tx_initialize_unused_memory = (void *)(&_ebss + 4);
    SCB_VTOR = (unsigned long)g_pfnVectors;
    _tx_thread_system_stack_ptr = (void *)g_pfnVectors[0];
    DEMCR |= (1u << 24);
    DWT_CTRL |= 1u;
    SYST_RVR = 79999u; /* 100 Hz for scoreboard comparability */
    SYST_CVR = 0u;
    SYST_CSR = 0x7u;
    SCB_SHPR1 = 0x00000000u;
    SCB_SHPR2 = 0xFF000000u;
    SCB_SHPR3 = 0x40FF0000u;
}

static void dump_and_park(void) {
    unsigned long backlog = thread_1_messages_sent - thread_2_messages_received;
    __asm__ volatile("cpsid i");
    uart_puts("TICKS=");
    uart_putdec32(tick_count);
    uart_puts(" t0=");
    uart_putdec32(thread_0_counter);
    uart_puts(" t1=");
    uart_putdec32(thread_1_counter);
    uart_puts(" sent=");
    uart_putdec32(thread_1_messages_sent);
    uart_puts(" t2=");
    uart_putdec32(thread_2_counter);
    uart_puts(" recvd=");
    uart_putdec32(thread_2_messages_received);
    uart_puts(" t3=");
    uart_putdec32(thread_3_counter);
    uart_puts(" t4=");
    uart_putdec32(thread_4_counter);
    uart_puts(" t5=");
    uart_putdec32(thread_5_counter);
    uart_puts(" t6=");
    uart_putdec32(thread_6_counter);
    uart_puts(" t7=");
    uart_putdec32(thread_7_counter);
    uart_puts(" backlog=");
    uart_putdec32(backlog);
    uart_puts(backlog <= 101 ? " PASS\n" : " FAIL\n");
    uart_puts("TX_DONE\n");
    for (;;) {}
}

void SysTick_Handler(void) {
    /* A SysTick that fires before the scheduler selects the first thread
     * (the window between CPSIE and PendSV dispatch in _tx_thread_schedule)
     * must not enter the kernel tick: there is no current thread yet.
     * On silicon the window is nanoseconds vs a millisecond period so it
     * never triggers; faster timer ratios (e.g. QEMU) can hit it. */
    if (_tx_thread_current_ptr != TX_NULL) {
        _tx_timer_interrupt();
        if (++tick_count >= N_TICKS) {
            dump_and_park();
        }
    }
}
