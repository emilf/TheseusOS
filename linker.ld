ENTRY(kernel_main)

/* Virtual memory layout for x86_64 kernel */
VIRTUAL_BASE = 0xffffffff80000000;  /* Kernel virtual base address */
PHYSICAL_BASE = 0x100000;           /* Physical load address */

SECTIONS
{
    /* Set virtual address for linking */
    . = VIRTUAL_BASE;
    
    /* Kernel text section (read-only, executable) */
    .text : AT(ADDR(.text) - VIRTUAL_BASE) {
        *(.text .text.*)
    }
    
    /* Read-only data */
    .rodata : AT(ADDR(.rodata) - VIRTUAL_BASE) {
        *(.rodata .rodata.*)
    }
    
    /* Read-write data */
    .data : AT(ADDR(.data) - VIRTUAL_BASE) {
        *(.data .data.*)
    }
    
    /* Uninitialized data */
    .bss : AT(ADDR(.bss) - VIRTUAL_BASE) {
        *(.bss .bss.*)
        *(COMMON)
    }
    
    /* Discard sections */
    /DISCARD/ : {
        *(.eh_frame)
        *(.note .note.*)
    }
    
    /* Physical load address (where kernel is actually loaded) */
    _kernel_start = PHYSICAL_BASE;
    _kernel_end = _kernel_start + SIZEOF(.text) + SIZEOF(.rodata) + SIZEOF(.data) + SIZEOF(.bss);
}
