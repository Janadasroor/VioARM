/* FreeRTOS tick-accuracy probe (matches the scoreboard claim: two
 * demo-style tasks + tick counter, exact 250.000 ms LED periods).
 * LED task wakes every 250 ticks via vTaskDelayUntil and stamps CYCCNT;
 * with a 1 kHz tick from 8 MHz, periods must be exactly 2000000 cycles.
 * Spin task proves preemption/time-slicing doesn't perturb the periods. */
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"

#define LED_TICKS 250u
#define N_STAMPS  10u
#define EXPECTED  (LED_TICKS * 8000u)

static uint32_t stamps[N_STAMPS];
static unsigned n_stamps = 0;
static volatile uint32_t spin_count = 0;

static void led_task(void *arg) {
    TickType_t last;
    (void)arg;
    last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, LED_TICKS);
        if (n_stamps < N_STAMPS) {
            stamps[n_stamps++] = cycnt();
            if (n_stamps == N_STAMPS) {
                unsigned i, bad = 0;
                uart_puts("PERIODS:");
                for (i = 1; i < N_STAMPS; i++) {
                    uint32_t p = stamps[i] - stamps[i - 1];
                    uart_puts(" ");
                    uart_putdec32(p);
                    if (p != EXPECTED) bad++;
                }
                uart_puts(bad ? " FAIL\n" : " PASS\n");
                uart_puts("SPIN=");
                uart_putdec32(spin_count);
                uart_puts("\nFR_DONE\n");
                for (;;) {}
            }
        }
    }
}

static void spin_task(void *arg) {
    (void)arg;
    for (;;) {
        spin_count++;
    }
}

/* Cortex-M3 system-handler priorities: SVCall/PendSV/SysTick to lowest. */
#define SCB_SHPR2 (*(volatile uint32_t *)0xE000ED1Cu)
#define SCB_SHPR3 (*(volatile uint32_t *)0xE000ED20u)
#define NVIC_IPR0 (*(volatile uint32_t *)0xE000E400u)

void SysTick_Handler(void);
void xPortPendSVHandler(void);
void vPortSVCHandler(void);
void xPortSysTickHandler(void);

void SysTick_Handler(void) { xPortSysTickHandler(); }

int main(void) {
    uint32_t save;
    board_init();
    /* NVIC priority-bit diagnostic: stock STM32F1 implements 4 bits. */
    save = NVIC_IPR0;
    NVIC_IPR0 = 0xFFFFFFFFu;
    uart_puts("IPBITS=");
    uart_puthex32(NVIC_IPR0);
    uart_puts("\n");
    NVIC_IPR0 = save;
    SCB_SHPR2 |= 0xFF000000u; /* SVCall lowest */
    SCB_SHPR3 |= 0xFFFF0000u; /* PendSV + SysTick lowest */
    xTaskCreate(led_task, "led", 128, 0, 2, 0);
    xTaskCreate(spin_task, "spin", 128, 0, 1, 0);
    vTaskStartScheduler();
    uart_puts("SCHEDULER_EXIT\n");
    for (;;) {}
    return 0;
}

void vApplicationMallocFailedHook(void) {
    uart_puts("MALLOC_FAIL\n");
    for (;;) {}
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *n) {
    (void)t; (void)n;
    uart_puts("STACK_OVERFLOW\n");
    for (;;) {}
}
