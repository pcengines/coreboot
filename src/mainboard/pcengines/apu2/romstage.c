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
#include <delay.h>
#include <device/pci_def.h>
#include <arch/io.h>
#include <arch/mmio.h>
#include <amdblocks/acpimmio.h>
#include <device/pci_def.h>
#include <device/pci_ops.h>
#include <console/console.h>
#include <console/uart.h>
#include <northbridge/amd/agesa/state_machine.h>
#include <southbridge/amd/pi/hudson/hudson.h>
#include <Fch/Fch.h>
#include <smp/node.h>

#include "bios_knobs.h"
#include "gpio_ftns.h"

static void early_lpc_init(void);
static void print_sign_of_life(void);
static void lpc_mcu_msg(void);
extern char coreboot_dmi_date[];
extern char coreboot_version[];

void board_BeforeAgesa(struct sysinfo *cb)
{
	u32 val;

	pm_write8(FCH_PMIOA_REGC5, 0);

	early_lpc_init();
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
		u32 data;
		pci_devfn_t dev;
		volatile u8 *CF9_shadow;
		CF9_shadow = (u8 *)(ACPI_MMIO_BASE + PMIO_BASE + FCH_PMIOA_REGC5);
		*CF9_shadow = 0x0;

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

		if ((check_com2() || (CONFIG_UART_FOR_CONSOLE == 1)) &&
		    !CONFIG(BOARD_PCENGINES_APU5))
			nuvoton_enable_serial(SERIAL2_DEV, 0x2f8);

		console_init();

		/* Check if cold boot was requested */
		val = pci_read_config32(PCI_DEV(0, 0x18, 0), 0x6C);
		if (val & (1 << 4)) {
			volatile u32 *ptr;
			printk(BIOS_ALERT, "Forcing cold boot path\n");
			val &= ~(0x630);	// ColdRstDet[4], BiosRstDet[10:9, 5]
			pci_write_config32(PCI_DEV(0, 0x18, 0), 0x6C, val);

			ptr = (u32*)FCH_PMIOxC0_S5ResetStatus;
			*ptr = 0x3fff003f;	// Write-1-to-clear

			*CF9_shadow = 0xe;	// FullRst, SysRst, RstCmd
			printk(BIOS_ALERT, "Did not reset (yet)\n");
		}

		data = *(u32*)FCH_PMIOxC0_S5ResetStatus;
		// do not print SOL if reset will take place in FchInit
		if (check_console() &&
		    !(data & FCH_PMIOxC0_S5ResetStatus_All_Status)) {
			print_sign_of_life();
		}

		volatile u32 *ptr = (u32 *)(ACPI_MMIO_BASE + WATCHDOG_BASE);
		u16 watchdog_timeout = get_watchdog_timeout();

		if (watchdog_timeout == 0) {
			//disable watchdog
			printk(BIOS_WARNING, "Watchdog is disabled\n");
			*ptr |= (1 << 3);
		} else {
			// enable
			*ptr &= ~(1 << 3);
			*ptr |= (1 << 0);
			// configure timeout
			*(ptr + 1) = (u16) watchdog_timeout;
			// trigger
			*ptr |= (1 << 7);

			printk(BIOS_WARNING, "Watchdog is enabled, state = 0x%x, time = %d\n", *ptr, *(ptr + 1));
		}
	}

	/* Halt if there was a built in self test failure */
	post_code(0x34);
	report_bist_failure(bist);

	/* Load MPB */
	val = cpuid_eax(1);
	printk(BIOS_DEBUG, "BSP Family_Model: %08x\n", val);
	printk(BIOS_DEBUG, "cpu_init_detectedx = %08lx\n", cpu_init_detectedx);

	/* Disable SVI2 controller to wait for command completion */
	val = pci_read_config32(PCI_DEV(0, 0x18, 5), 0x12C);
	if (!(val & (1 << 30))) {
		val |= (1 << 30);
		pci_write_config32(PCI_DEV(0, 0x18, 5), 0x12C, val);
	}

	/* Release GPIO32/33 for other uses. */
	pm_write8(0xea, 1);
}

static void early_lpc_init(void)
{
	u32 setting = 0x0;

	//
	// Configure output disabled, value low, pull up/down disabled
	//
	if (CONFIG(BOARD_PCENGINES_APU5)) {
		configure_gpio(IOMUX_GPIO_22, Function0, GPIO_22, setting);
	}

	if (CONFIG(BOARD_PCENGINES_APU2) ||
		CONFIG(BOARD_PCENGINES_APU3) ||
		CONFIG(BOARD_PCENGINES_APU4)) {
		configure_gpio(IOMUX_GPIO_32, Function0, GPIO_32, setting);
	}

	configure_gpio(IOMUX_GPIO_49, Function2, GPIO_49, setting);
	configure_gpio(IOMUX_GPIO_50, Function2, GPIO_50, setting);
	configure_gpio(IOMUX_GPIO_71, Function0, GPIO_71, setting);

	//
	// Configure output enabled, value low, pull up/down disabled
	//
	setting = GPIO_OUTPUT_ENABLE;
	if (CONFIG(BOARD_PCENGINES_APU3) ||
		CONFIG(BOARD_PCENGINES_APU4)) {
		configure_gpio(IOMUX_GPIO_33, Function0, GPIO_33, setting);
	}

	configure_gpio(IOMUX_GPIO_57, Function1, GPIO_57, setting);
	configure_gpio(IOMUX_GPIO_58, Function1, GPIO_58, setting);
	configure_gpio(IOMUX_GPIO_59, Function3, GPIO_59, setting);

	//
	// Configure output enabled, value high, pull up/down disabled
	//
	setting = GPIO_OUTPUT_ENABLE | GPIO_OUTPUT_VALUE;

	if (CONFIG(BOARD_PCENGINES_APU5)) {
		configure_gpio(IOMUX_GPIO_32, Function0, GPIO_32, setting);
		configure_gpio(IOMUX_GPIO_33, Function0, GPIO_33, setting);
	}

	configure_gpio(IOMUX_GPIO_51, Function2, GPIO_51, setting);
	configure_gpio(IOMUX_GPIO_55, Function3, GPIO_55, setting);

	/* W_DIS# pins are connected directly to the SoC GPIOs without
	 * any external pull-ups. This causes issues with certain mPCIe
	 * modems. Configure pull-ups and output high in order to
	 * prevent disabling WLAN on the modules. APU5 has hardware
	 * pull-ups on the PCB.
	 */
	if (!CONFIG(BOARD_PCENGINES_APU5))
		setting = GPIO_PULL_UP_ENABLE | GPIO_OUTPUT_VALUE;

	configure_gpio(IOMUX_GPIO_64, Function2, GPIO_64, setting);
	configure_gpio(IOMUX_GPIO_68, Function0, GPIO_68, setting);
}

static const char *mainboard_bios_version(void)
{
	if (strlen(CONFIG_LOCALVERSION))
		return CONFIG_LOCALVERSION;
	else
		return coreboot_version;
}

static void print_sign_of_life()
{
	char tmp[9];
	strncpy(tmp,   coreboot_dmi_date+6, 4);
	strncpy(tmp+4, coreboot_dmi_date+3, 2);
	strncpy(tmp+6, coreboot_dmi_date,   2);
	tmp[8] = '\0';
	printk(BIOS_ALERT, CONFIG_MAINBOARD_VENDOR " "
	                   CONFIG_MAINBOARD_PART_NUMBER "\n");
	printk(BIOS_ALERT, "coreboot build %s\n", tmp);
	printk(BIOS_ALERT, "BIOS version %s\n", mainboard_bios_version());
}

void board_BeforeInitReset(struct sysinfo *cb, AMD_RESET_PARAMS *Reset)
{
	u32 val, data;

	if (boot_cpu()) {
		pm_write8(FCH_PMIOA_REGC5, 0);

		/* Check if cold boot was requested */
		val = pci_read_config32(PCI_DEV(0, 0x18, 0), 0x6C);
		if (val & (1 << 4)) {
			printk(BIOS_ALERT, "Forcing cold boot path\n");
			val &= ~(0x630); // ColdRstDet[4], BiosRstDet[10:9, 5]
			pci_write_config32(PCI_DEV(0, 0x18, 0), 0x6C, val);

			pm_write32(0xc0, 0x3fff003f); // Write-1-to-clear resets

			/* FullRst, SysRst, RstCmd */
			pm_write8(FCH_PMIOA_REGC5, 0xe);
			printk(BIOS_ALERT, "Did not reset (yet)\n");
		}

		// do not print SOL if reset will take place in FchInit
		if (!(pm_read32(0xc0) & FCH_PMIOxC0_S5ResetStatus_All_Status)) {
			if (check_console())
				print_sign_of_life();
			
			lpc_mcu_msg();
		}

		if ((check_mpcie2_clk() || CONFIG(FORCE_MPCIE2_CLK)) &&
		     CONFIG(BOARD_PCENGINES_APU2)) {
			// make GFXCLK to ignore CLKREQ# input
			// force it to be always on
			data = misc_read32(FCH_MISC_REG04);
			data &= 0xFFFFFF0F;
			data |= 0xF << (1 * 4); // CLKREQ GFX to GFXCLK
			misc_write32(FCH_MISC_REG04, data);
			printk(BIOS_DEBUG, "force mPCIe clock enabled\n");
		}
	}
}

static void lpc_mcu_msg(void)
{
	unsigned int i, timeout;
	const char *post_msg = "BIOSBOOT";
	unsigned char sync_byte = 0;

	if (!CONFIG(BOARD_PCENGINES_APU5))
		return;

	uart_init(1);

	for (i = 0; i < 4; i++) {
		uart_tx_byte(1, 0xe1);
		uart_tx_flush(1);
		timeout = 10;
		while (sync_byte != 0xe1) {
			sync_byte = uart_rx_byte(1);
			if (timeout == 0) {
				uart_init(CONFIG_UART_FOR_CONSOLE);
				udelay(10000);
				printk(BIOS_ERR, "Failed to sync with LPC"
				       " MCU, number of retries %d\n", 3 - i);
				udelay(10000);
				uart_init(1);
				udelay(10000);
				break;
			}
			udelay(100);
			timeout--;
		}
		if (sync_byte == 0xe1)
			break;
	}

	if (sync_byte != 0xe1)
		return;

	uart_init(1);
	timeout = 10;

	for (i = 0; i < strlen(post_msg); i++)
		uart_tx_byte(1, *(post_msg + i));

	uart_tx_byte(1, 0xe1);
	uart_tx_flush(1);

	while (uart_rx_byte(1) != 0xe1) {
		if (timeout == 0) {
			uart_init(CONFIG_UART_FOR_CONSOLE);
			printk(BIOS_ERR, "Did not receive response to BIOSBOOT\n");
			return;
		}
		udelay(100);
		timeout--;
	}

	uart_init(CONFIG_UART_FOR_CONSOLE);
}
