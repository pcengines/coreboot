/*
 * This file is part of the coreboot project.
 *
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

/* "ASTR" for AMD Suspend-To-RAM */
#define MEMCTX_DATA_SIGNATURE       (('A'<<0)|('S'<<8)|('T'<<16)|('R'<<24))

struct mrc_metadata {
	uint32_t signature;
	uint32_t data_size;
	uint16_t data_checksum;
	uint16_t header_checksum;
	uint32_t version;
} __packed;

void save_memctx(AMD_S3_PARAMS *MemContext);
VOID GetMemS3NV(AMD_POST_PARAMS *PostParams);
