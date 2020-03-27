/*
 * This file is part of the coreboot project.
 *
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

#include <bootblock_common.h>
#include <intelblocks/lpc_lib.h>
#include "gpio.h"

void bootblock_mainboard_init(void)
{
	const struct pad_config *pads;
	size_t num;

	lpc_configure_pads();

	/* Configure GPIOs needed prior to ramstage. */
	pads = early_gpio_table(&num);
	gpio_configure_pads(pads, num);
}
