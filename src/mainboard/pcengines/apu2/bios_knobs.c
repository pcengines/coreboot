/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2018 PC Engines GmbH
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

#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include <console/console.h>
#include <program_loading.h>
#include <cbfs.h>
#include <commonlib/cbfs.h>
#include <commonlib/region.h>
#include <drivers/vpd/vpd.h>
#include "bios_knobs.h"

static int check_knob_value(const char *s, enum vpd_region region)
{
	char buffer[10];

	if (!vpd_gets(s, buffer, sizeof(buffer), region))
		return -1;
	
	printk(BIOS_INFO, "Knob %s %s: %s\n", s, buffer,
			  region == VPD_RO ? "RO_VPD" : "RW_VPD");

	if (!strcmp(buffer, "enabled"))
		return 1;
	else if (!strcmp(buffer, "disabled"))
		return 0;
	else 
		return -1;
}

static int is_knob_enabled(const char *s)
{
	int knob = check_knob_value(s, VPD_RW);

	if (knob == -1)
		printk(BIOS_WARNING, "%s knob missing in RW VPD\n", s);
	else
		return knob;

	knob = check_knob_value(s, VPD_RO);

	if (knob == -1) {
		printk(BIOS_WARNING, "%s knob missing in RO VPD\n", s);
		// return -1 as error
		return knob;
	} else
		return knob;

}

u8 check_iommu(void)
{
	u8 iommu;
	iommu = check_knob_value("iommu");

	switch (iommu) {
	case 0:
		return 0;
		break;
	case 1:
		return 1;
		break;
	default:
		printk(BIOS_INFO,
			"Missing or invalid iommu knob, disable IOMMU.\n");
		break;
	}

	return 0;
}

u8 check_console(void)
{
	int scon = is_knob_enabled("scon");

	switch (scon) {
	case 0:
		return 0;
		break;
	case 1:
		return 1;
		break;
	default:
		printk(BIOS_INFO, "Enable serial console\n");
		return true;
		break;
	}
<<<<<<< HEAD

	return 1;
=======
>>>>>>> src/mainboard/pcengines/apu2/bios_knobs.c: rewrite knobs to use VPD
}

bool check_com2(void)
{
	int com2en = is_knob_enabled("com2en");

	switch (com2en) {
	case 0:
		return false;
		break;
	case 1:
		return true;
		break;
	default:
		printk(BIOS_INFO, "Disable COM2 output\n");
		return false;
		break;
	}
}

bool check_boost(void)
{
	int boost = is_knob_enabled("boosten");

	switch (boost) {
	case 0:
<<<<<<< HEAD
		return 0;
		break;
	case 1:
		return 1;
		break;
	default:
		printk(BIOS_INFO,
			"Missing or invalid boost knob, disable CPU boost.\n");
		break;
	}

	return 0;
}

static u8 check_uart(char uart_letter)
{
	u8 uarten;

	switch (uart_letter) {
	case 'c':
		uarten = check_knob_value("uartc");
		break;
	case 'd':
		uarten = check_knob_value("uartd");
		break;
	default:
		uarten = -1;
		break;
	}

	switch (uarten) {
	case 0:
		return 0;
=======
		return false;
>>>>>>> src/mainboard/pcengines/apu2/bios_knobs.c: rewrite knobs to use VPD
		break;
	case 1:
		return 1;
		break;
	default:
		printk(BIOS_INFO, "Enable CPU boost\n");
		return true;
		break;
	}
<<<<<<< HEAD

	return 0;
}

inline u8 check_uartc(void)
{
	return check_uart('c');
}

inline u8 check_uartd(void)
{
	return check_uart('d');
=======
>>>>>>> src/mainboard/pcengines/apu2/bios_knobs.c: rewrite knobs to use VPD
}

u8 check_ehci0(void)
{
	u8 ehci0 = is_knob_enabled("ehcien");

	switch (ehci0) {
	case 0:
		return 0;
		break;
	case 1:
		return 1;
		break;
	default:
<<<<<<< HEAD
		printk(BIOS_INFO,
			"Missing or invalid ehci0 knob, enable ehci0.\n");
		break;
	}

	return 1;
}

u8 check_mpcie2_clk(void)
{
	u8 mpcie2_clk;

	//
	// Find the mPCIe2 clock item
	//
	mpcie2_clk = check_knob_value("mpcie2_clk");

	switch (mpcie2_clk) {
	case 0:
		return 0;
		break;
	case 1:
		return 1;
=======
		printk(BIOS_INFO,"Enable EHCI0.\n");
		return true;
>>>>>>> src/mainboard/pcengines/apu2/bios_knobs.c: rewrite knobs to use VPD
		break;
	}
<<<<<<< HEAD

	return 0;
=======
>>>>>>> src/mainboard/pcengines/apu2/bios_knobs.c: rewrite knobs to use VPD
}

u8 check_sd3_mode(void)
{
	u8 sd3mode = is_knob_enabled("sd3mode");

	switch (sd3mode) {
	case 0:
		return 0;
		break;
	case 1:
		return 1;
		break;
	default:
		printk(BIOS_INFO, "Disable SD3.0 mode\n");
		return false;
		break;
	}
<<<<<<< HEAD

	return 0;
=======
>>>>>>> src/mainboard/pcengines/apu2/bios_knobs.c: rewrite knobs to use VPD
}

static int _valid(char ch, int base)
{
	char end = (base > 9) ? '9' : '0' + (base - 1);

	/* all bases will be some subset of the 0-9 range */

	if (ch >= '0' && ch <= end)
		return 1;

	/* Bases > 11 will also have to match in the a-z range */

	if (base > 11) {
		if (tolower(ch) >= 'a' &&
		    tolower(ch) <= 'a' + (base - 11))
			return 1;
	}

	return 0;
}

/* Return the "value" of the character in the given base */

static int _offset(char ch, int base)
{
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	else
		return 10 + tolower(ch) - 'a';
}


static unsigned long long int strtoull(const char *ptr, char **endptr, int base)
{
	unsigned long long int ret = 0;

	if (endptr != NULL)
		*endptr = (char *) ptr;

	/* Purge whitespace */

	for( ; *ptr && isspace(*ptr); ptr++);

	if (!*ptr)
		return 0;

	/* Determine the base */

	if (base == 0) {
		if (ptr[0] == '0' && (ptr[1] == 'x' || ptr[1] == 'X'))
			base = 16;
		else if (ptr[0] == '0') {
			base = 8;
			ptr++;
		}
		else
			base = 10;
	}

	/* Base 16 allows the 0x on front - so skip over it */

	if (base == 16) {
		if (ptr[0] == '0' && (ptr[1] == 'x' || ptr[1] == 'X') &&
		    _valid(ptr[2], base))
			ptr += 2;
	}

	for( ; *ptr && _valid(*ptr, base); ptr++)
		ret = (ret * base) + _offset(*ptr, base);

	if (endptr != NULL)
		*endptr = (char *) ptr;

	return ret;
}

#define ULONG_MAX	((unsigned long int)~0UL)

static unsigned long int strtoul(const char *ptr, char **endptr, int base)
{
	unsigned long long val = strtoull(ptr, endptr, base);
	if (val > ULONG_MAX) return ULONG_MAX;
	return val;
}

u16 get_watchdog_timeout(void)
{
	u16 timeout_ro, timeout_rw;

	char buffer_ro[10];
	char buffer_rw[10];

	vpd_gets("watchdog", buffer_ro, sizeof(buffer_ro), VPD_RO);
	vpd_gets("watchdog", buffer_rw, sizeof(buffer_rw), VPD_RW);

	timeout_ro = (u16) strtoul(buffer_ro, NULL, 10);
	timeout_rw = (u16) strtoul(buffer_rw, NULL, 10);

	if (timeout_ro == 0 && timeout_rw == 0)
		return 0;
	else if (timeout_rw > timeout_ro)
		return timeout_rw;
	else
		return timeout_ro;
}

bool check_uartc(void)
{
	u8 uartc = is_knob_enabled("uartc");

	switch (uartc) {
	case 0:
		return false;
		break;
	case 1:
		return true;
		break;
	default:
		printk(BIOS_INFO, "Disable UARTC and enable GPIO0\n");
		return false;
		break;
	}
}

bool check_uartd(void)
{
	u8 uartd = is_knob_enabled("uartd");

	switch (uartd) {
	case 0:
		return false;
		break;
	case 1:
		return true;
		break;
	default:
		printk(BIOS_INFO, "Disable UARTD and enable GPIO1\n");
		return false;
		break;
	}
}

<<<<<<< HEAD
	return timeout;
}
=======
>>>>>>> src/mainboard/pcengines/apu2/bios_knobs.c: rewrite knobs to use VPD
