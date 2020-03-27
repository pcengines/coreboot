/*
 * This file is part of the coreboot project.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; version 2 of
 * the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <arch/pirq_routing.h>

static const struct irq_routing_table intel_irq_routing_table = {
	PIRQ_SIGNATURE,		/* u32 signature */
	PIRQ_VERSION,		/* u16 version */
	32 + 16 * 15,		/* Max. number of devices on the bus */
	0x00,			/* Interrupt router bus */
	(0x1f << 3) | 0x0,	/* Interrupt router dev */
	0,			/* IRQs devoted exclusively to PCI usage */
	0x8086,			/* Vendor */
	0x122e,			/* Device */
	0,			/* Miniport */
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, /* u8 rfu[11] */
	0xf5,			/* Checksum (has to be set to some value that
				 * would give 0 after the sum of all bytes
				 * for this structure (including checksum).
				 */
	{
		/* bus,        dev | fn,   {link, bitmap}, {link, bitmap}, {link, bitmap}, {link, bitmap}, slot, rfu */
		{0x00, (0x02 << 3) | 0x0, {{0x00, 0xdef8}, {0x61, 0x1cf8}, {0x00, 0xdef8}, {0x00, 0xdef8}}, 0x0, 0x0}, /* VGA       0:02.0 */
		{0x00, (0x1b << 3) | 0x0, {{0x00, 0xdef8}, {0x61, 0x1cf8}, {0x00, 0xdef8}, {0x00, 0xdef8}}, 0x0, 0x0}, /* HD Audio  0:1b.0 */
		{0x00, (0x1c << 3) | 0x0, {{0x68, 0x1cf8}, {0x69, 0x1cf8}, {0x6a, 0x1cf8}, {0x6b, 0x1cf8}}, 0x0, 0x0}, /* PCIe      0:1c.0 */
		{0x00, (0x1c << 3) | 0x1, {{0x68, 0x1cf8}, {0x69, 0x1cf8}, {0x6a, 0x1cf8}, {0x6b, 0x1cf8}}, 0x0, 0x0}, /* PCIe      0:1c.1 */
		{0x00, (0x1c << 3) | 0x2, {{0x68, 0x1cf8}, {0x69, 0x1cf8}, {0x6a, 0x1cf8}, {0x6b, 0x1cf8}}, 0x0, 0x0}, /* PCIe      0:1c.2 */
		{0x00, (0x1c << 3) | 0x3, {{0x68, 0x1cf8}, {0x69, 0x1cf8}, {0x6a, 0x1cf8}, {0x6b, 0x1cf8}}, 0x0, 0x0}, /* PCIe      0:1c.3 */
		{0x00, (0x1d << 3) | 0x0, {{0x60, 0x1cf8}, {0x61, 0x1cf8}, {0x62, 0x1cf8}, {0x63, 0x1cf8}}, 0x0, 0x0}, /* USB       0:1d.0 */
		{0x00, (0x1d << 3) | 0x1, {{0x60, 0x1cf8}, {0x61, 0x1cf8}, {0x62, 0x1cf8}, {0x63, 0x1cf8}}, 0x0, 0x0}, /* USB       0:1d.1 */
		{0x00, (0x1d << 3) | 0x2, {{0x60, 0x1cf8}, {0x61, 0x1cf8}, {0x62, 0x1cf8}, {0x63, 0x1cf8}}, 0x0, 0x0}, /* USB       0:1d.2 */
		{0x00, (0x1d << 3) | 0x3, {{0x60, 0x1cf8}, {0x61, 0x1cf8}, {0x62, 0x1cf8}, {0x63, 0x1cf8}}, 0x0, 0x0}, /* USB       0:1d.3 */
		{0x00, (0x1e << 3) | 0x0, {{0x60, 0x1cf8}, {0x61, 0x1cf8}, {0x62, 0x1cf8}, {0x63, 0x1cf8}}, 0x0, 0x0}, /* PCI       0:1e.0 */
		{0x00, (0x1f << 3) | 0x0, {{0x6b, 0x1cf8}, {0x60, 0x1cf8}, {0x60, 0x1cf8}, {0x00, 0xdef8}}, 0x0, 0x0}, /* LPC       0:1f.0 */
		{0x00, (0x1f << 3) | 0x1, {{0x6b, 0x1cf8}, {0x60, 0x1cf8}, {0x60, 0x1cf8}, {0x00, 0xdef8}}, 0x0, 0x0}, /* IDE       0:1f.1 */
		{0x00, (0x1f << 3) | 0x2, {{0x6b, 0x1cf8}, {0x60, 0x1cf8}, {0x60, 0x1cf8}, {0x00, 0xdef8}}, 0x0, 0x0}, /* SATA      0:1f.2 */
	}
};

unsigned long write_pirq_routing_table(unsigned long addr)
{
	return copy_pirq_routing_table(addr, &intel_irq_routing_table);
}
