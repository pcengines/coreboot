#
# This file is part of the coreboot project.
#
# Copyright (C) 2012 Advanced Micro Devices, Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; version 2 of the License.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#

romstage-y += bios_knobs.c
romstage-y += BiosCallOuts.c
romstage-y += OemCustomize.c
romstage-y += gpio_ftns.c
romstage-y += s1_button.c
romstage-y += s3nv.c

ramstage-y += bios_knobs.c
ramstage-y += BiosCallOuts.c
ramstage-y += OemCustomize.c
ramstage-y += gpio_ftns.c
ramstage-y += s1_button.c

# Order of names in SPD_SOURCES is important!
SPD_SOURCES  = HYNIX-2G-1333
SPD_SOURCES += HYNIX-4G-1333-ECC

subdirs-y += variants/$(VARIANT_DIR)

$(obj)/coreboot_s3nv.rom: $(obj)/config.h
	echo "    S3 NVRAM   (S3 storage area)"
	# force C locale, so cygwin awk doesn't try to interpret the 0xff below as UTF-8 (or worse)
	printf %d $(CONFIG_S3_DATA_SIZE) | LC_ALL=C awk '{for (i=0; i<$$1; i++) {printf "%c", 255}}' > $@.tmp
	mv $@.tmp $@

cbfs-files-y += s3nv
s3nv-file := $(obj)/coreboot_s3nv.rom
s3nv-align := 0x1000
s3nv-type := raw
