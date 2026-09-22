/* Linker script for kings bare-metal runs on stm32f103.
 *
 * VioARM boots STM32 boards from the 0x0 flash alias (the 0x0800_0000
 * window is a separately gated store), so VioARM images link at 0x0.
 * Flat layout: vectors + code in flash, .data (LMA right after code,
 * copied by Reset_Handler), .bss, then newlib heap up to top-512B.
 */
ENTRY(Reset_Handler)

MEMORY {
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 128K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 20K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS {
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > FLASH

    .text : {
        *(.text*)
        KEEP(*(.init))
        KEEP(*(.fini))
        *(.rodata*)
        _etext = .;
    } > FLASH

    .ARM.exidx : {
        *(.ARM.exidx*)
    } > FLASH

    .ARM.extab : {
        *(.ARM.extab*)
    } > FLASH

    _sidata = .;

    .data : AT(_sidata) {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM

    .bss (NOLOAD) : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM

    _heap_start = .;
    . = ORIGIN(RAM) + LENGTH(RAM) - 512;
    _heap_end = .;

    /DISCARD/ : { *(.ARM.attributes*) }
}
