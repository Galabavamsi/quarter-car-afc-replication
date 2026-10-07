/* Minimal layout for QEMU mps2-an386 (Cortex-M4): 4 MB code at 0, 4 MB RAM at 0x20000000 */
MEMORY { FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 4M
         RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 4M }
ENTRY(Reset_Handler)
SECTIONS {
  .vectors : { KEEP(*(.vectors)) } > FLASH
  .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) } > FLASH
  .ARM.exidx : { *(.ARM.exidx*) } > FLASH
  __etext = .;
  .data : AT(__etext) { __data_start__ = .; *(.data*) __data_end__ = .; } > RAM
  .bss : { __bss_start__ = .; *(.bss*) *(COMMON) __bss_end__ = .; } > RAM
  end = .; _end = .;
  __stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
