/* SPDX-License-Identifier: GPL-2.0-only */

.code32

.section .init, "ax", @progbits

.section .init._gdt_, "ax", @progbits

	.globl gdt_init
gdt_init:
	lgdt	%cs:gdtptr
	ret

.previous
	.align	4
.globl gdtptr
gdtptr:
	.word	gdt_end - gdt -1 /* compute the table limit */
	.long	gdt		 /* we know the offset */

#ifdef __x86_64__
.code64
.section .init._gdt64_, "ax", @progbits
	.globl gdt_init64
gdt_init64:
	movabs	$gdtptr64, %rax
	lgdt	(%rax)
	ret

.previous
	.align	4
.globl gdtptr64
gdtptr64:
	.word	gdt_end - gdt -1 /* compute the table limit */
	.quad	gdt		 /* we know the offset */
#endif

	.align	4
gdt:
	/* selgdt 0, unused */
	.word	0x0000, 0x0000		/* dummy */
	.byte	0x00, 0x00, 0x00, 0x00

	/* selgdt 0x08, flat code segment */
	.word	0xffff, 0x0000
	.byte	0x00, 0x9b, 0xcf, 0x00 /* G=1 and 0x0f, So we get 4Gbytes
					  for limit */

	/* selgdt 0x10,flat data segment */
	.word	0xffff, 0x0000
	.byte	0x00, 0x93, 0xcf, 0x00

	/* selgdt 0x18, flat code segment (64-bit) */
	.word   0xffff, 0x0000
	.byte   0x00, 0x9b, 0xaf, 0x00

gdt_end:
