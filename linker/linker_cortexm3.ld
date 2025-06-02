/*
 * Minimal Linker Script for a generic Cortex-M3 target (e.g., QEMU's lm3s6965evb)
 */

/* Define Memory Regions */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K /* Emulated Flash for lm3s6965evb */
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 64K  /* Emulated RAM for lm3s6965evb */
}

/* Define a symbol for the top of the stack.
 * This must match the symbol used in the startup file (e.g., _estack).
 * It's placed at the very end of the RAM region.
 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Entry Point of the application */
ENTRY(Reset_Handler)

/* Define Sections */
SECTIONS
{
    /* The .isr_vector section from the startup file goes at the beginning of FLASH */
    .isr_vector :
    {
        KEEP(*(.isr_vector)) /* Keep this section, and keep it first */
    } > FLASH

    /* The .text section (program code and read-only data) */
    .text :
    {
        *(.text*)          /* All .text sections from all input files */
        *(.rodata*)        /* All .rodata sections (read-only data) */
        . = ALIGN(4);      /* Align to a 4-byte boundary */
    } > FLASH

    /* The .data section (initialized data)
     * Values are stored in FLASH (LMA - Load Memory Address)
     * and copied to RAM (VMA - Virtual Memory Address) at startup.
     * Our minimal startup file currently SKIPS this copy, so avoid initialized global/static data
     * in main.c for the very first test, or understand it won't be initialized yet.
     */
    .data : AT (ADDR(.text) + SIZEOF(.text)) /* Place LMA for .data after .text in FLASH */
    {
        _sdata = .;        /* Start of .data in RAM (VMA) */
        *(.data*)          /* All .data sections */
        . = ALIGN(4);
        _edata = .;        /* End of .data in RAM (VMA) */
    } > RAM

    /* The .bss section (uninitialized data)
     * This section is zeroed out at startup.
     * Our minimal startup file currently SKIPS this zeroing.
     */
    .bss :
    {
        _sbss = .;         /* Start of .bss in RAM */
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;         /* End of .bss in RAM */
    } > RAM

    /* Remove information from the standard library if it accidentally gets linked.
     * The -nostdlib linker flag should primarily handle this.
     */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    /* This section is mandatory for ARM EABI compatibility. */
    .ARM.attributes 0 : { *(.ARM.attributes) }
}