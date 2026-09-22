/* FreeRTOSConfig.h for kings runs: stm32f103 @ 8 MHz HSI, 1 kHz tick.
 * Minimal: 2 tasks + vTaskDelayUntil, no queues/semaphores/timers. */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
extern void uart_puts(const char *s);
extern void uart_putdec32(uint32_t v);

#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configCPU_CLOCK_HZ                      8000000u
#define configTICK_RATE_HZ                      1000u
#define configMAX_PRIORITIES                    4
#define configMINIMAL_STACK_SIZE                128
#define configTOTAL_HEAP_SIZE                   4096
#define configMAX_TASK_NAME_LEN                 12
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       0
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_TIMERS                        0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configKERNEL_INTERRUPT_PRIORITY         255
/* Low nibble zero: only implemented priority bits may be set (STM32F1
 * implements 4; the stock demo value 191/0xBF sets unimplemented bits and
 * trips the port's own config check on silicon too). */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    160
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_xTaskGetTickCount               1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskSuspend                    0
#define INCLUDE_xTaskGetSchedulerState          0

#define configASSERT(x) \
    do { if (!(x)) { uart_puts("ASSERT@"); uart_putdec32(__LINE__); uart_puts("\n"); for (;;) {} } } while (0)

#endif /* FREERTOS_CONFIG_H */
