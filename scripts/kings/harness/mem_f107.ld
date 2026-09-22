/* Linker script for kings bare-metal runs on stm32f107
 * (256 KiB flash, 64 KiB RAM). Same shape as mem_f103.ld; VioARM boots
 * from the 0x0 flash alias. */
ENTRY(Reset_Handler)

MEMORY {
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 64K
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
