
ENTRY(_start)

SECTIONS
{
	. = 0x00000000;

	.text : ALIGN(16) {
		__TEXT_BEGIN__ = .;
		*(.initial_jump)
		*(.entry.text)
		*(.init.literal)
		*(.init)
		*(.text)
		*(.literal .text .literal.* .text.* .stub)
		*(.out_jump.literal.*)
		*(.out_jump.*)
		__TEXT_END__ = .;
	}

    .rodata ALIGN(16):
    {
        *(.rodata)
        *(.rodata.*)
        *(.rodata1)
    }

    .data ALIGN(16):
    {
        *(.data)
        *(.data1)
        *(.sdata)
        *(.sdata2)
        *(.got.plt)
        *(.got)
        *(.dynamic)
    }
}
