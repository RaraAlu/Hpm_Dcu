ENTRY(fixture_start)
MEMORY {
    FLASH (rx) : ORIGIN = 0x80000000, LENGTH = 512K
    RAM (rwx) : ORIGIN = 0x01200000, LENGTH = 128K
}
SECTIONS {
    .text : { *(.text*) KEEP(*(.isr_vector*)) *(.rodata*) } > FLASH
    .data : { *(.data*) *(.sdata*) } > RAM
    .bss (NOLOAD) : { *(.bss*) *(.sbss*) *(COMMON) } > RAM
    __freertos_irq_stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
