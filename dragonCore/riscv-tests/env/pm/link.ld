OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS
{

  /* tohost in uncacheable IO region */
  . = 0x40000000;
  .tohost : { *(.tohost) }

  . = 0x80000000;
  .text.init : { *(.text.init) }
  //. = ALIGN(0x1000);
  //.tohost : { *(.tohost) }
  . = ALIGN(0x1000);
  .text : { *(.text) }
  . = ALIGN(0x1000);
  .data : { *(.data) }
  .bss : { *(.bss) }
  _end = .;
}

