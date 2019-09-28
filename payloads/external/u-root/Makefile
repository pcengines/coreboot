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

project_name=uroot
project_dir=$(CURDIR)/uroot
project_git_repo=github.com/u-root/u-root

export GOPATH:=$(CURDIR)/uroot
export PATH:=/usr/local/go/bin:$(GOPATH)/bin:$(PATH)

all: build

$(project_dir):
	echo "    Cloning $(project_name) with Go"
	mkdir $(project_dir)
	go get $(project_git_repo)
	go get github.com/3mdeb/uinit

build: $(project_dir)
	cd $(project_dir)/src/$(project_git_repo) ; \
	u-root -format=cpio -o $(project_dir)/initramfs.cpio -build=bb \
		$(project_git_repo)/cmds/core/init \
		$(project_git_repo)/cmds/core/kexec \
		$(project_git_repo)/cmds/core/mount \
		github.com/3mdeb/uinit
	cd $(project_dir)
	lzma -f -k $(project_dir)/initramfs.cpio

clean:
	test -d $(project_dir) && rm $(project_dir)/initramfs.cpio* || exit 0

distclean:
	rm -rf $(project_dir)

print-repo-info:
	echo "$(project_git_repo) $(project_dir)"

.PHONY: all build clean distclean print-repo-info
