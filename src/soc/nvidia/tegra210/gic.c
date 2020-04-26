/* SPDX-License-Identifier: GPL-2.0-only */
/* This file is part of the coreboot project. */

#include <gic.h>
#include <soc/addressmap.h>

void *gicd_base(void)
{
	return (void *)(uintptr_t)TEGRA_GICD_BASE;
}

void *gicc_base(void)
{
	return (void *)(uintptr_t)TEGRA_GICC_BASE;
}
