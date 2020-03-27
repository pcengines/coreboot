/*
 * This file is part of the coreboot project.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <stdint.h>
#include <device/pci_ops.h>
#include <device/pci.h>
#include <device/pci_def.h>
#include <device/pci_ids.h>
#include "i82371eb.h"

void enable_pm(void)
{
	pci_devfn_t dev;
	u8 reg8;
	u16 reg16;

	/* Get the SMBus/PM device of the 82371AB/EB/MB. */
	dev = pci_locate_device(PCI_ID(PCI_VENDOR_ID_INTEL,
				PCI_DEVICE_ID_INTEL_82371AB_SMB_ACPI), 0);

	/* Set the PM I/O base. */
	pci_write_config32(dev, PMBA, DEFAULT_PMBASE | 1);

	/* Enable access to the PM I/O space. */
	reg16 = pci_read_config16(dev, PCI_COMMAND);
	reg16 |= PCI_COMMAND_IO;
	pci_write_config16(dev, PCI_COMMAND, reg16);

	/* PM I/O Space Enable (PMIOSE). */
	reg8 = pci_read_config8(dev, PMREGMISC);
	reg8 |= PMIOSE;
	pci_write_config8(dev, PMREGMISC, reg8);
}
