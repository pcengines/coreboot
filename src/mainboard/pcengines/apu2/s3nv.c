/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2014 Google Inc.
 * Copyright (C) 2019 PC Engines GmbH
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

#include <AGESA.h>
#include <boot_device.h>
#include <cbfs.h>
#include <commonlib/region.h>
#include <commonlib/cbfs.h>
#include <console/console.h>
#include <ip_checksum.h>
#include <string.h>
#include <stdlib.h>
#include <spi_flash.h>
#include <spi-generic.h>
#include "s3nv.h"

static size_t get_s3nv_cbfs_offset(const char *name, uint32_t type)
{
	const struct region_device *boot_dev;
	struct cbfsf fh;

	boot_dev = boot_device_ro();

	if (cbfs_boot_locate(&fh, name, &type)) {
		printk(BIOS_WARNING, "Can't locate file in CBFS\n");
		return 0;
	}

	return (size_t) rdev_relative_offset(boot_dev, &fh.data);
}

static int flash_s3nv(size_t offset, size_t fsize, const void *buffer)
{
	const struct spi_flash *flash;

	boot_device_init();
	flash = boot_device_spi_flash();

	if (flash == NULL) {
		printk(BIOS_ERR, "Can't get boot flash device\n");
		return -1;
	}

	if (spi_flash_write(flash, offset, fsize, buffer)) {
		printk(BIOS_ERR, "SPI write failed\n");
		return -1;
	}

	printk(BIOS_DEBUG, "S3NV write successed\n");
	return 0;
}

static int memctx_needs_update(const void *buffer)
{
	size_t fsize = 0;
	void* data = NULL;
	struct mrc_metadata *md;

	data = cbfs_boot_map_with_leak("s3nv", CBFS_TYPE_RAW, &fsize);

	if (!data || fsize == 0) {
		printk(BIOS_ERR, "CBFS: S3NV not found\n");
		return -1;
	}

	md = (struct mrc_metadata *) data;

	if (md->data_size == 0 || md->data_size == 0xffffffff)
		return 1;

	if (memcmp(buffer, data, CONFIG_S3_DATA_SIZE)) {
		printk(BIOS_DEBUG, "S3NV: MemCtx needs update\n");
		return 1;
	}

	return 0;
}

void save_memctx(AMD_S3_PARAMS *MemContext)
{
	uint8_t buffer[CONFIG_S3_DATA_SIZE];
	struct mrc_metadata *md;
	size_t memctx_offset;

	md = (struct mrc_metadata *) buffer;

	memset(md, 0, CONFIG_S3_DATA_SIZE);
	md->signature = MemContext->Signature;
	md->data_size = MemContext->NvStorageSize;
	md->version = (uint32_t) MemContext->Version;
	md->data_checksum = compute_ip_checksum(MemContext->NvStorage,
						MemContext->NvStorageSize);
	md->header_checksum = compute_ip_checksum(md, sizeof(*md));
	memcpy(&md[1], MemContext->NvStorage,
				     MemContext->NvStorageSize);

	if(!memctx_needs_update(buffer))
		return;

	memctx_offset = get_s3nv_cbfs_offset("s3nv", CBFS_TYPE_RAW);
	if (flash_s3nv(memctx_offset, CONFIG_S3_DATA_SIZE, buffer))
		printk(BIOS_ERR, "Failed to save S3 memctx\n");
}

VOID GetMemS3NV(AMD_POST_PARAMS *PostParams)
{
	size_t fsize = 0;
	void* data = NULL;
	struct mrc_metadata *md;

	data = cbfs_boot_map_with_leak("s3nv", CBFS_TYPE_RAW, &fsize);

	if (!data || fsize == 0) {
		PostParams->MemConfig.MemContext.NvStorageSize = 0;
		return;
	}

	md = (struct mrc_metadata *) data;

	if (md->data_size == 0 || md->data_size == 0xffffffff) {
		PostParams->MemConfig.MemContext.NvStorageSize = 0;
		return;
	}

	PostParams->MemConfig.MemContext.NvStorageSize = md->data_size;
	PostParams->MemConfig.MemContext.NvStorage = &md[1];
}