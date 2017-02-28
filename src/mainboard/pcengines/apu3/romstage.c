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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdint.h>
#include <string.h>
#include <device/pci_def.h>
#include <device/pci_ids.h>
#include <arch/acpi.h>
#include <arch/io.h>
#include <arch/stages.h>
#include <device/pnp_def.h>
#include <arch/cpu.h>
#include <cpu/x86/lapic.h>
#include <console/console.h>
#include <console/loglevel.h>
#include <cpu/amd/car.h>
#include <agesawrapper.h>
#include <northbridge/amd/pi/agesawrapper_call.h>
#include <cpu/x86/bist.h>
#include <cpu/x86/lapic.h>
#include <hudson.h>
#include <cpu/amd/pi/s3_resume.h>
#include <fchgpio.h>
#include "apu2.h"
#include <northbridge/amd/pi/00730F01/eltannorthbridge.h>
#include <eltanhudson.h>
#include <build.h>

//
// GPIO Init Table
//
//		GPIO_DEFINITION(gpio, 	function, outputenable, output, pullup, pulldown)
static const GPIO_CONTROL gGpioInitTable[] = {
		GPIO_DEFINITION (APU2_SPD_STRAP0_GPIO,	APU2_SPD_STRAP0_FUNC,	0, 0, 0, 0),
		GPIO_DEFINITION (APU2_SPD_STRAP1_GPIO,	APU2_SPD_STRAP1_FUNC,	0, 0, 0, 0),
		GPIO_DEFINITION (APU2_PE3_RST_L_GPIO,	APU2_PE3_RST_L_FUNC,	1, 1, 0, 0),
		GPIO_DEFINITION (APU2_PE4_RST_L_GPIO,	APU2_PE4_RST_L_FUNC,	1, 1, 0, 0),
		GPIO_DEFINITION (APU2_LED1_L_GPIO,		APU2_LED1_L_FUNC,		1, 0, 0, 0),	// Turn on the LEDs by default
		GPIO_DEFINITION (APU2_LED2_L_GPIO,		APU2_LED2_L_FUNC,		1, 0, 0, 0),
		GPIO_DEFINITION (APU2_LED3_L_GPIO,		APU2_LED3_L_FUNC,		1, 0, 0, 0),
		GPIO_DEFINITION (APU2_PE3_WDIS_L_GPIO,	APU2_PE3_WDIS_L_FUNC,	1, 1, 0, 0),
		GPIO_DEFINITION (APU2_PE4_WDIS_L_GPIO,	APU2_PE4_WDIS_L_FUNC,	1, 1, 0, 0),
// SPKR doesn't require init, left at default
		GPIO_DEFINITION (APU2_PROCHOT_GPIO,		APU2_PROCHOT_FUNC,		0, 0, 0, 0),
		GPIO_DEFINITION (APU2_BIOS_CONSOLE_GPIO, APU2_BIOS_CONSOLE_FUNC,	0, 0, 0, 0),
		{0xFF, 0xFF, 0xFF}									// Terminator
	};


void cache_as_ram_main(unsigned long bist, unsigned long cpu_init_detectedx)
{
	u32 val;
#if CONFIG_SVI2_SLOW_SPEED
	device_t d18f3_dev = PCI_DEV(0, 0x18, 3);
#endif //CONFIG_SVI2_SLOW_SPEED
#if CONFIG_SVI_WAIT_COMP_DIS
	device_t d18f5_dev = PCI_DEV(0, 0x18, 5);
#endif //CONFIG_SVI_WAIT_COMP_DIS

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

	AGESAWRAPPER_PRE_CONSOLE(amdinitmmio);

	if (!cpu_init_detectedx && boot_cpu()) {

		u32 data, *memptr;

		hudson_lpc_port80();
		//
		// Configure the GPIO's
		//
		HandleFchGpioTbl ( (GPIO_CONTROL *) &gGpioInitTable[0] );

		hudson_clk_output_48Mhz();

		post_code(0x31);
		console_init();

		printk(BIOS_INFO, "14-25-48Mhz Clock settings\n");

		memptr = (u32 *)(ACPI_MMIO_BASE + MISC_BASE + FCH_MISC_REG28 );
		data = *memptr;
		printk(BIOS_INFO, "FCH_MISC_REG28 is 0x%08x \n", data);

		memptr = (u32 *)(ACPI_MMIO_BASE + MISC_BASE + FCH_MISC_REG40 );
		data = *memptr;
		printk(BIOS_INFO, "FCH_MISC_REG40 is 0x%08x \n", data);

		//
		// Configure clock request
		//
		data = *((u32 *)(ACPI_MMIO_BASE + MISC_BASE+FCH_MISC_REG00));

		data &= 0xFFFF0000;
		data |= (0 + 1) << (0 * 4);	// CLKREQ 0 to CLK0
		data |= (1 + 1) << (1 * 4);	// CLKREQ 1 to CLK1
		data |= (2 + 1) << (2 * 4);	// CLKREQ 2 to CLK2
		// make CLK3 to ignore CLKREQ# input
		// force it to be always on
		data |= ( 0xf ) << (3 * 4);	// CLKREQ 3 to CLK3

		*((u32 *)(ACPI_MMIO_BASE + MISC_BASE+FCH_MISC_REG00)) = data;

		data = *((u32 *)(ACPI_MMIO_BASE + MISC_BASE+FCH_MISC_REG04));

		data &= 0xFFFFFF0F;
		data |= 0xA << (1 * 4);	// CLKREQ GFX to GFXCLK

		*((u32 *)(ACPI_MMIO_BASE + MISC_BASE+FCH_MISC_REG04)) = data;

//			//
//			// Configure clock strength
//			//
//			data = *((u32 *)(ACPI_MMIO_BASE + MISC_BASE+FCH_MISC_REG24));
//
//			data &= ~( (3 << 18) | (3 << 6) | (3 << 4) | (3 << 2) | (3 << 0) );
//			data |= 3 << 18;		// GFX CLOCK
//			data |= 3 << (0 * 2);	// CLK0
//			data |= 3 << (1 * 2);	// CLK1
//			data |= 3 << (2 * 2);	// CLK2
//			data |= 3 << (3 * 2);	// CLK3
//
//			*((u32 *)(ACPI_MMIO_BASE + MISC_BASE+FCH_MISC_REG24)) = data;
	}

	/* Halt if there was a built in self test failure */
	post_code(0x34);
	report_bist_failure(bist);

	/* Load MPB */
	val = cpuid_eax(1);
	printk(BIOS_DEBUG, "BSP Family_Model: %08x \n", val);
	printk(BIOS_DEBUG, "cpu_init_detectedx = %08lx \n", cpu_init_detectedx);

	/*
	 * This refers to LpcClkDrvSth settling time.  Without this setting, processor
	 * initialization is slow or incorrect, so this wait has been replicated from
	 * earlier development boards.
	 */
	{ int i; for(i = 0; i < 200000; i++) inb(0xCD6); }

	post_code(0x37);
	AGESAWRAPPER(amdinitreset);

	/* TODO There is no debug output between the return from amdinitreset and amdinitearly */

	post_code(0x38);
	printk(BIOS_DEBUG, "Got past avalon_early_setup\n");

	post_code(0x39);
	AGESAWRAPPER(amdinitearly);

	/*
	// Moved here to prevent double signon message
	// amdinitreset AGESA code might issue a reset when the hardware is in a wrong state.
	*/

	printk(BIOS_ERR, CONFIG_MAINBOARD_PART_NUMBER "\n");
	printk(BIOS_ERR, "coreboot build " COREBOOT_YYYYMMDD_DATE "\n");

#if CONFIG_SVI2_SLOW_SPEED
	/* Force SVI2 to slow speed for APU3 */
	val = pci_read_config32( d18f3_dev, 0xA0);
	if ( val & (1 << 14 ) ) {

		printk(BIOS_DEBUG, "SVI2 FREQUENCY 20 Mhz changing to 3.4\n");
		val &= ~(1 << 14 );
		pci_write_config32(d18f3_dev, 0xA0, val );

	} else {

		printk(BIOS_DEBUG, "SVI2 FREQUENCY 3.4 Mhz\n");
	}
#endif //CONFIG_SVI2_SLOW_SPEED

#if CONFIG_SVI_WAIT_COMP_DIS
	/* Disable SVI2 controller to wait for command completion */
	val = pci_read_config32( d18f5_dev, 0x12C);
	if ( val & (1 << 30 ) ) {

		printk(BIOS_DEBUG, "SVI2 Wait completion disabled\n");

	} else {

		printk(BIOS_DEBUG, "Disabling SVI2 Wait completion\n");
		val |= (1 << 30 );
		pci_write_config32(d18f5_dev, 0x12C, val );
	}

#endif //CONFIG_SVI_WAIT_COMP_DIS

	int s3resume = acpi_is_wakeup_s3();
	if (!s3resume) {
		post_code(0x40);
		AGESAWRAPPER(amdinitpost);

		//PspMboxBiosCmdDramInfo();
		post_code(0x41);
		AGESAWRAPPER(amdinitenv);
		/*
		  If code hangs here, please check cahaltasm.S
		*/
		disable_cache_as_ram();

	} else { /* S3 detect */

		printk(BIOS_INFO, "S3 detected\n");

		post_code(0x60);
		AGESAWRAPPER(amdinitresume);

		AGESAWRAPPER(amds3laterestore);

		post_code(0x61);
		prepare_for_resume();
	}

	outb(0xEA, 0xCD6);
	outb(0x1, 0xcd7);

	post_code(0x50);
	copy_and_run();

	post_code(0x54);  /* Should never see this post code. */
}
