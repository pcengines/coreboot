##
## This file is part of the coreboot project.
##
## Copyright (C) 2016 Google Inc.
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; version 2 of the License.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##

project_name=Linux
project_dir=$(CURDIR)/linux
project_git_repo=https://github.com/3mdeb/linux-stable.git
project_git_branch=linux-sl-5.1-sha2-amd

unexport $(COREBOOT_EXPORTS)

all: build

$(project_dir):
	echo "    Cloning $(project_name) from Git"
	git clone $(project_git_repo) $(project_dir) --depth=1 -b $(project_git_branch)

build: $(project_dir)
	echo "    MAKE       $(project_name)"
	cp $(CURDIR)/linux_defconfig-5.1 $(project_dir)/.config
	$(MAKE) -C $(project_dir) olddefconfig V=1
	$(MAKE) -C $(project_dir) bzImage V=1
	cp $(project_dir)/arch/x86/boot/bzImage $(project_dir)/bzImage

clean:
	test -d $(project_dir) && rm $(project_dir)/bzImage || exit 0
	test -d $(project_dir) && $(MAKE) -C $(project_dir) clean || exit 0

distclean:
	rm -rf $(project_dir)

print-repo-info:
	echo "$(project_git_repo) $(project_dir)"

.PHONY: all build clean distclean print-repo-info
