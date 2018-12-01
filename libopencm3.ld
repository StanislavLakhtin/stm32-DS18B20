MEMORY
{
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    ram (xrw)  : ORIGIN = 0x20000000, LENGTH = 20K
}

INCLUDE cortex-m-generic.ld

