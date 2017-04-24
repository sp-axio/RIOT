

/* Linker script to configure memory regions. */
MEMORY
{
    rom (rx) : ORIGIN = 0x00000000, LENGTH = 512K
    ram (rwx)  : ORIGIN = 0x2000C000, LENGTH = 64K
}

INCLUDE cortexm_base.ld
