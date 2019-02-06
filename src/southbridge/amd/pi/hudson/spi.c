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
#include <stdlib.h>
#include <string.h>
#include <arch/io.h>
#include <arch/early_variables.h>
#include <commonlib/helpers.h>
#include <console/console.h>
#include <delay.h>
#include <device/device.h>
#include <device/pci.h>
#include <device/pci_ops.h>
#include <lib.h>
#include "pci_devs.h"
#include <spi_flash.h>
#include <spi-generic.h>
#include <timer.h>

#include <Proc/Fch/FchPlatform.h>

/* SPI Controller (base address in D14F3xA0) */
#define SPI_CNTRL0			0x00
#define   SPI_BUSY			BIT(31)
#define   SPI_READ_MODE_MASK		(BIT(30) | BIT(29) | BIT(18))
/* Nominal is 16.7MHz on older devices, 33MHz on newer */
#define   SPI_READ_MODE_NOM		0x00000000
#define   SPI_READ_MODE_DUAL112		(          BIT(29)          )
#define   SPI_READ_MODE_QUAD114		(          BIT(29) | BIT(18))
#define   SPI_READ_MODE_DUAL122		(BIT(30)                    )
#define   SPI_READ_MODE_QUAD144		(BIT(30) |           BIT(18))
#define   SPI_READ_MODE_NORMAL66	(BIT(30) | BIT(29)          )
#define   SPI_READ_MODE_FAST		(BIT(30) | BIT(29) | BIT(18))
#define   SPI_FIFO_PTR_CLR		BIT(20)
#define   SPI_ARB_ENABLE		BIT(19)
#define   EXEC_OPCODE			BIT(16)
#define SPI_CNTRL1			0x0c
#define SPI_CMD_CODE			0x45
#define SPI_CMD_TRIGGER			0x47
#define   SPI_CMD_TRIGGER_EXECUTE	BIT(7)
#define SPI_TX_BYTE_COUNT		0x48
#define SPI_RX_BYTE_COUNT		0x4b
#define SPI_STATUS			0x4c
#define   SPI_DONE_BYTE_COUNT_SHIFT	0
#define   SPI_DONE_BYTE_COUNT_MASK	0xff
#define   SPI_FIFO_WR_PTR_SHIFT		8
#define   SPI_FIFO_WR_PTR_MASK		0x7f
#define   SPI_FIFO_RD_PTR_SHIFT		16
#define   SPI_FIFO_RD_PTR_MASK		0x7f
#define SPI_FIFO			0x80
#define   SPI_FIFO_DEPTH		(0xc7 - SPI_FIFO)
#define SPI_DEBUG_DRIVER IS_ENABLED(CONFIG_DEBUG_SPI_FLASH)

static uintptr_t spibar CAR_GLOBAL;

static uintptr_t get_spibase(void)
{
	return car_get_var(spibar);
}

static void set_spibar(uintptr_t base)
{
	car_set_var(spibar, base);
}

static inline uint8_t spi_read8(uint8_t reg)
{
	return read8((void *)(get_spibase() + reg));
}

static inline uint32_t spi_read32(uint8_t reg)
{
	return read32((void *)(get_spibase() + reg));
}

static inline void spi_write8(uint8_t reg, uint8_t val)
{
	write8((void *)(get_spibase() + reg), val);
}

static inline void spi_write32(uint8_t reg, uint32_t val)
{
	write32((void *)(get_spibase() + reg), val);
}

static void dump_state(const char *str)
{
	if (!SPI_DEBUG_DRIVER)
		return;

	printk(BIOS_DEBUG, "SPI: %s\n", str);
	printk(BIOS_DEBUG, "Cntrl0: %08x\n", spi_read32(SPI_CNTRL0));
	printk(BIOS_DEBUG, "Status: %08x\n", spi_read32(SPI_STATUS));
	printk(BIOS_DEBUG, "TxByteCount: %d\n", spi_read8(SPI_TX_BYTE_COUNT));
	printk(BIOS_DEBUG, "RxByteCount: %d\n", spi_read8(SPI_RX_BYTE_COUNT));
	printk(BIOS_DEBUG, "CmdCode: %02x\n", spi_read8(SPI_CMD_CODE));
	hexdump((void *)(get_spibase() + SPI_FIFO), SPI_FIFO_DEPTH);
}

static int wait_for_ready(void)
{
	const uint32_t timeout_ms = 500;
	struct stopwatch sw;

	stopwatch_init_msecs_expire(&sw, timeout_ms);

	do {
		if (!(spi_read32(SPI_STATUS) & SPI_BUSY))
			return 0;
	} while (!stopwatch_expired(&sw));

	return -1;
}

static int execute_command(void)
{
	dump_state("Before Execute");

	spi_write8(SPI_CMD_TRIGGER, SPI_CMD_TRIGGER_EXECUTE);

	if (wait_for_ready()) {
		printk(BIOS_DEBUG,
			"FCH SPI Error: Timeout executing command\n");
		return -1;
	}

	dump_state("Transaction finished");

	return 0;
}

void spi_init(void)
{
	uintptr_t bar;

	bar = pci_read_config32(LPC_PCIDEV, 0xA0) & ~0x1F;
	bar = ALIGN_DOWN(bar, 64);
	set_spibar(bar);
}

static int spi_ctrlr_xfer(const struct spi_slave *slave, const void *dout,
		size_t bytesout, void *din, size_t bytesin)
{
	/* First byte is cmd which can not being sent through FIFO. */
	u8 cmd = *(u8 *)dout++;
	size_t count;

	if (SPI_DEBUG_DRIVER)
		printk(BIOS_DEBUG, "%s(%zx, %zx)\n", __func__, bytesout,
			bytesin);

	bytesout--;

	/*
	 * Check if this is a write command attempting to transfer more bytes
	 * than the controller can handle. Iterations for writes are not
	 * supported here because each SPI write command needs to be preceded
	 * and followed by other SPI commands, and this sequence is controlled
	 * by the SPI chip driver.
	 */
	if (bytesout > SPI_FIFO_DEPTH) {
		printk(BIOS_WARNING, "FCH SPI: Too much to write."
			" Does your SPI chip driver use spi_crop_chunk()?\n");
		return -1;
	}

	spi_write8(SPI_CMD_CODE, cmd);
	spi_write8(SPI_TX_BYTE_COUNT, bytesout);
	spi_write8(SPI_RX_BYTE_COUNT, bytesin);


	for (count = 0; count < bytesout; count++, dout++)
		spi_write8(SPI_FIFO + count, *(uint8_t *)dout);

	if (execute_command())
		return -1;

	for (count = 0; count < bytesin; count++, din++)
		*(uint8_t *)din = spi_read8(SPI_FIFO + count + bytesout);

#if !ENV_RAMSTAGE
	/*
	 * FIXME: too frequent SPI transactions lead to reset in early boot
	 * stages
	 */
	udelay(100);
#endif

	return 0;
}

int chipset_volatile_group_begin(const struct spi_flash *flash)
{
	if (!IS_ENABLED(CONFIG_HUDSON_IMC_FWM))
		return 0;

	ImcSleep(NULL);
	return 0;
}

int chipset_volatile_group_end(const struct spi_flash *flash)
{
	if (!IS_ENABLED(CONFIG_HUDSON_IMC_FWM))
		return 0;

	ImcWakeup(NULL);
	return 0;
}

static int xfer_vectors(const struct spi_slave *slave,
			struct spi_op vectors[], size_t count)
{
	return spi_flash_vector_helper(slave, vectors, count, spi_ctrlr_xfer);
}

static const struct spi_ctrlr spi_ctrlr = {
	.xfer_vector = xfer_vectors,
	.max_xfer_size = SPI_FIFO_DEPTH,
	.flags = SPI_CNTRLR_DEDUCT_CMD_LEN | SPI_CNTRLR_DEDUCT_OPCODE_LEN,
};

const struct spi_ctrlr_buses spi_ctrlr_bus_map[] = {
	{
		.ctrlr = &spi_ctrlr,
		.bus_start = 0,
		.bus_end = 0,
	},
};

const size_t spi_ctrlr_bus_map_count = ARRAY_SIZE(spi_ctrlr_bus_map);
