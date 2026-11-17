ENTRY(dcu_time_now_ms)
MEMORY {
    FLASH (rx) : ORIGIN = 0x80000000, LENGTH = 64K
    RAM (rw) : ORIGIN = 0x01200000, LENGTH = 64K
}
SECTIONS {
    .text : { *(.text*) *(.rodata*) } > FLASH
    .data : { *(.data*) *(.sdata*) } > RAM
    .bss (NOLOAD) : { *(.bss*) *(.sbss*) *(COMMON) } > RAM
}
