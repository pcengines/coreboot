/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2012 Advanced Micro Devices, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <stdint.h>
#include <string.h>
#include <arch/io.h>
#include <arch/cpu.h>
#include <cbfs.h>
#include <cpu/amd/car.h>
#include <cpu/x86/bist.h>
#include <cpu/x86/lapic.h>
#include <console/console.h>
#include <commonlib/loglevel.h>
#include <commonlib/region.h>
#include <device/pci_def.h>
#include <device/pnp.h>
#include <fmap.h>
#include <northbridge/amd/agesa/state_machine.h>
#include <northbridge/amd/pi/agesawrapper.h>
#include <northbridge/amd/pi/agesawrapper_call.h>
#include <security/vboot/vboot_crtm.h>
#include <security/vboot/misc.h>
#include <southbridge/amd/pi/hudson/hudson.h>
#include <superio/nuvoton/common/nuvoton.h>
#include <superio/nuvoton/nct5104d/nct5104d.h>
#include <timestamp.h>
#include <Fch/Fch.h>

#include "gpio_ftns.h"

#define SIO_PORT 0x2e
#define SERIAL1_DEV PNP_DEV(SIO_PORT, NCT5104D_SP1)
#define SERIAL2_DEV PNP_DEV(SIO_PORT, NCT5104D_SP2)

static void early_lpc_init(void);

static void measure_amd_blobs(struct sysinfo *cb)
{
	struct region_device rdev;
	uint32_t cbfs_type = CBFS_TYPE_RAW;
	struct cbfsf fh;

	if (tpm_setup(cb->s3resume) != TPM_SUCCESS)
		return;

	printk(BIOS_DEBUG, "Measuring AMD blobs\n");

	if(fmap_locate_area_as_rdev("PSPDIR", &rdev)) {
		printk(BIOS_ERR, "Error: Couldn't find PSPDIR region.");
		return;
	}
	tpm_measure_region(&rdev, TPM_RUNTIME_DATA_PCR,"PSPDIR");

	if (cbfs_locate_file_in_region(&fh, "COREBOOT",
				       (const char *)CONFIG_AGESA_CBFS_NAME,
				       &cbfs_type) < 0) {
		printk(BIOS_ERR, "Error: Couldn't find AGESA.");
		return;
	}
	cbfsf_file_type(&fh, &cbfs_type);
	cbfs_file_data(&rdev, &fh);
	tpm_measure_region(&rdev, TPM_CRTM_PCR, "AGESA");

	/* Measure SPD */
	cbfs_type = CBFS_TYPE_SPD;
	cbfs_boot_locate(&fh, "spd.bin", &cbfs_type);
	cbfsf_file_type(&fh, &cbfs_type);
	cbfs_file_data(&rdev, &fh);
	tpm_measure_region(&rdev, TPM_CRTM_PCR, "spd.bin");

}

void cache_as_ram_main(unsigned long bist, unsigned long cpu_init_detectedx)
{
	u32 val;

	/*
	 *  In Hudson RRG, PMIOxD2[5:4] is "Drive strength control for
	 *  LpcClk[1:0]".  This following register setting has been
	 *  replicated in every reference design since Parmer, so it is
	 *  believed to be required even though it is not documented in
	 *  the SoC BKDGs.  Without this setting, there is no serial
	 *  output.
	 */
	outb(0xD2, 0xcd6);
	outb(0x00, 0xcd7);

	hudson_lpc_port80();

	if (!cpu_init_detectedx && boot_cpu()) {
		pci_devfn_t dev;
		u32 data;

		timestamp_init(timestamp_get());
		timestamp_add_now(TS_START_ROMSTAGE);

		post_code(0x30);
		early_lpc_init();

		hudson_clk_output_48Mhz();
		post_code(0x31);

		dev = PCI_DEV(0, 0x14, 3);
		data = pci_read_config32(dev, LPC_IO_OR_MEM_DECODE_ENABLE);
		/* enable 0x2e/0x4e IO decoding before configuring SuperIO */
		pci_write_config32(dev, LPC_IO_OR_MEM_DECODE_ENABLE, data | 3);

		/* COM2 on apu5 is reserved so only COM1 should be supported */
		if ((CONFIG_UART_FOR_CONSOLE == 1) &&
			!IS_ENABLED(CONFIG_BOARD_PCENGINES_APU5))
			nuvoton_enable_serial(SERIAL2_DEV, CONFIG_TTYS0_BASE);
		else if (CONFIG_UART_FOR_CONSOLE == 0)
			nuvoton_enable_serial(SERIAL1_DEV, CONFIG_TTYS0_BASE);

		console_init();
	}

	/* Halt if there was a built in self test failure */
	post_code(0x34);
	report_bist_failure(bist);

	/* Load MPB */
	val = cpuid_eax(1);
	printk(BIOS_DEBUG, "BSP Family_Model: %08x\n", val);
	printk(BIOS_DEBUG, "cpu_init_detectedx = %08lx\n", cpu_init_detectedx);

	post_code(0x37);
	AGESAWRAPPER(amdinitreset);

	post_code(0x38);
	printk(BIOS_DEBUG, "Got past avalon_early_setup\n");

	post_code(0x39);
	AGESAWRAPPER(amdinitearly);

	/* Disable SVI2 controller to wait for command completion */
	val = pci_read_config32(PCI_DEV(0, 0x18, 5), 0x12C);
	if (val & (1 << 30)) {
		printk(BIOS_DEBUG, "SVI2 Wait completion disabled\n");
	} else {
		printk(BIOS_DEBUG, "Disabling SVI2 Wait completion\n");
		val |= (1 << 30);
		pci_write_config32(PCI_DEV(0, 0x18, 5), 0x12C, val);
	}

	timestamp_add_now(TS_BEFORE_INITRAM);

	post_code(0x40);
	AGESAWRAPPER(amdinitpost);

	/* FIXME: Detect if TSC frequency changed during raminit? */
	timestamp_rescale_table(1, 4);

	timestamp_add_now(TS_AFTER_INITRAM);
}

void agesa_postcar(struct sysinfo *cb)
{
	//PspMboxBiosCmdDramInfo();
	post_code(0x41);
	AGESAWRAPPER(amdinitenv);

	if (IS_ENABLED(CONFIG_VBOOT_MEASURED_BOOT)) {
		/* Measure AGESA and PSPDIR */
		measure_amd_blobs(cb);
	}

	outb(0xEA, 0xCD6);
	outb(0x1, 0xcd7);
}

static void early_lpc_init(void)
{
	u32 setting = 0x0;

	//
	// Configure output disabled, value low, pull up/down disabled
	//
	if (IS_ENABLED(CONFIG_BOARD_PCENGINES_APU5)) {
		configure_gpio(IOMUX_GPIO_22, Function0, GPIO_22, setting);
	}

	if (IS_ENABLED(CONFIG_BOARD_PCENGINES_APU2) ||
		IS_ENABLED(CONFIG_BOARD_PCENGINES_APU3) ||
		IS_ENABLED(CONFIG_BOARD_PCENGINES_APU4)) {
		configure_gpio(IOMUX_GPIO_32, Function0, GPIO_32, setting);
	}

	configure_gpio(IOMUX_GPIO_49, Function2, GPIO_49, setting);
	configure_gpio(IOMUX_GPIO_50, Function2, GPIO_50, setting);
	configure_gpio(IOMUX_GPIO_71, Function0, GPIO_71, setting);

	//
	// Configure output enabled, value low, pull up/down disabled
	//
	setting = GPIO_OUTPUT_ENABLE;
	if (IS_ENABLED(CONFIG_BOARD_PCENGINES_APU3) ||
		IS_ENABLED(CONFIG_BOARD_PCENGINES_APU4)) {
		configure_gpio(IOMUX_GPIO_33, Function0, GPIO_33, setting);
	}

	configure_gpio(IOMUX_GPIO_57, Function1, GPIO_57, setting);
	configure_gpio(IOMUX_GPIO_58, Function1, GPIO_58, setting);
	configure_gpio(IOMUX_GPIO_59, Function3, GPIO_59, setting);

	//
	// Configure output enabled, value high, pull up/down disabled
	//
	setting = GPIO_OUTPUT_ENABLE | GPIO_OUTPUT_VALUE;

	if (IS_ENABLED(CONFIG_BOARD_PCENGINES_APU5)) {
		configure_gpio(IOMUX_GPIO_32, Function0, GPIO_32, setting);
		configure_gpio(IOMUX_GPIO_33, Function0, GPIO_33, setting);
	}

	configure_gpio(IOMUX_GPIO_51, Function2, GPIO_51, setting);
	configure_gpio(IOMUX_GPIO_55, Function3, GPIO_55, setting);
	configure_gpio(IOMUX_GPIO_64, Function2, GPIO_64, setting);
	configure_gpio(IOMUX_GPIO_68, Function0, GPIO_68, setting);
}
