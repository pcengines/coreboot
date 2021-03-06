/* SPDX-License-Identifier: GPL-2.0-only */

#include <acpi/acpi.h>

DefinitionBlock(
	"dsdt.aml",
	"DSDT",
	ACPI_DSDT_REV_2,
	OEM_ID,
	ACPI_TABLE_CREATOR,
	0x20090419
)
{
	#include <acpi/dsdt_top.asl>

	#include <southbridge/intel/common/acpi/platform.asl>
	#include <southbridge/intel/i82801gx/acpi/globalnvs.asl>

	#include <cpu/intel/speedstep/acpi/cpu.asl>

	Device (\_SB.PCI0)
	{
		#include <northbridge/intel/pineview/acpi/pineview.asl>
		#include <southbridge/intel/i82801gx/acpi/ich7.asl>
	}

	#include <southbridge/intel/common/acpi/sleepstates.asl>
}
