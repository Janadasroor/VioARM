/* Minimal layout for the SHA-256 soak blob: code and data linked at the
 * RAM base the benches load it to. Everything the firmware touches is
 * inside the single output binary (no CRT, no relocations). */
ENTRY(_start)
SECTIONS {
    . = 0x20000000;
    .text : { *(.text*) }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    /DISCARD/ : { *(.ARM.attributes*) *(.comment*) }
}
