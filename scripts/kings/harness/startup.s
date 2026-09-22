/* Minimal Cortex-M3 startup for kings runs.
 * Vector table + Reset_Handler that copies .data (LMA->VMA), zeroes .bss,
 * calls SystemInit (clock stub) then main(), then BKPTs to halt the sim.
 * C++ static constructors are NOT run here; C++ ports call
 * __libc_init_array() first thing in main(). */
    .syntax unified
    .cpu cortex-m3
    .thumb

    .section .isr_vector, "a", %progbits
    .globl g_pfnVectors
    .type g_pfnVectors, %object
g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word Default_Handler   /* NMI */
    .word Default_Handler   /* HardFault */
    .word Default_Handler   /* MemManage */
    .word Default_Handler   /* BusFault */
    .word Default_Handler   /* UsageFault */
    .word 0
    .word 0
    .word 0
    .word 0
    .word Default_Handler   /* SVCall */
    .word Default_Handler   /* DebugMon */
    .word 0
    .word Default_Handler   /* PendSV */
    .word SysTick_Handler   /* SysTick */
    .size g_pfnVectors, .-g_pfnVectors

    .section .text.Reset_Handler, "ax", %progbits
    .thumb_func
    .globl Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* r0 = dst (VMA), r1 = src (LMA), r2 = end */
    ldr r0, =_sdata
    ldr r1, =_sidata
    ldr r2, =_edata
    cmp r0, r2
    bhs 1f
0:  ldr r3, [r1], #4
    str r3, [r0], #4
    cmp r0, r2
    blo 0b
1:  ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
    cmp r0, r1
    bhs 3f
2:  str r2, [r0], #4
    cmp r0, r1
    blo 2b
3:  bl SystemInit
    bl main
    bkpt #0
4:  b 4b
    .size Reset_Handler, .-Reset_Handler

    .section .text.Default_Handler, "ax", %progbits
    .thumb_func
    .globl Default_Handler
    .type Default_Handler, %function
Default_Handler:
    bkpt #0
    b Default_Handler
    .size Default_Handler, .-Default_Handler

    .thumb_func
    .weak SysTick_Handler
    .set SysTick_Handler, Default_Handler
