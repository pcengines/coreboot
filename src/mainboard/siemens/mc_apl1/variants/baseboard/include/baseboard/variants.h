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

#ifndef _BASEBOARD_VARIANTS_H_
#define _BASEBOARD_VARIANTS_H_

#include <soc/gpio.h>

/*
 * The next set of functions return the gpio table and fill in the number of
 * entries for each table.
 */
const struct pad_config *variant_gpio_table(size_t *num);
const struct pad_config *variant_early_gpio_table(size_t *num);

/* This function provides the swizzle data for the DRAM initialization. */
const struct lpddr4_swizzle_cfg *variant_lpddr4_swizzle_config(void);

/* The following function performs board specific things. */
void variant_mainboard_final(void);

#endif /* _BASEBOARD_VARIANTS_H_ */
