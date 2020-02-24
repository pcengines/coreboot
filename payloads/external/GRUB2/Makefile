TAG-$(CONFIG_GRUB2_MASTER)=origin/HEAD
TAG-$(CONFIG_GRUB2_REVISION)=$(CONFIG_GRUB2_REVISION_ID)
TAG-$(CONFIG_GRUB2_STABLE)=grub-2.04
NAME-$(CONFIG_GRUB2_MASTER)=HEAD
NAME-$(CONFIG_GRUB2_REVISION)=$(CONFIG_GRUB2_REVISION_ID)
NAME-$(CONFIG_GRUB2_STABLE)=2.04

project_git_repo=https://github.com/3mdeb/grub2.git
project_dir=grub2

unexport HOSTCC CC LD OBJCOPY STRIP
MAKEOVERRIDES :=

all: grub2

clone:
	echo "    Cloning GRUB2 from Git"
	git clone $(project_git_repo) $(project_dir)

fetch: clone
ifeq ($(TAG-y),)
	echo "Error: The specified tag is invalid"
ifeq ($(CONFIG_GRUB2_REVISION),y)
	echo "Error: There is no revision specified for GRUB2"
	false
endif
	false
endif
	cd grub2; git show $(TAG-y) >/dev/null 2>&1 ;	if [ $$? -ne 0 ] || \
		[ "$(TAG-y)" = "origin/master" ]; then \
		echo "    Fetching new commits from the GRUB2 git repo"; \
		git fetch; fi

checkout: fetch
	echo "    Checking out GRUB2 revision $(TAG-y)"
	cd grub2; git checkout master; git branch -D coreboot 2>/dev/null; git checkout -b coreboot $(TAG-y)

grub2/build/config.h: $(CONFIG_DEP) | checkout
	echo "    CONFIG     GRUB2 $(NAME-y)"
	rm -rf grub2/build
	mkdir grub2/build
	cd grub2 && ./bootstrap && ./autogen.sh
	cd grub2/build && ../configure CC="$(HOSTCC)" LD="$(LD)" \
	FREETYPE="pkg-config freetype2" BUILD_FREETYPE="pkg-config freetype2" \
	TARGET_CC="$(CC)" TARGET_OBJCOPY="$(OBJCOPY)" TARGET_STRIP="$(STRIP)" \
	CFLAGS=-O2 TARGET_CFLAGS=-Os \
	--with-platform=coreboot --enable-boot-time --disable-werror

config: grub2/build/config.h checkout

grub2: config
	echo "    MAKE       GRUB2 $(NAME-y)"
	$(MAKE) -C grub2/build
	$(MAKE) -C grub2/build default_payload.elf \
		EXTRA_PAYLOAD_MODULES="$(CONFIG_GRUB2_EXTRA_MODULES)"

clean:
	test -f grub2/build/Makefile && $(MAKE) -C grub2/build clean || exit 0

distclean:
	rm -rf grub2

print-repo-info:
	echo "$(project_git_repo) $(project_dir)"

.PHONY: checkout config grub2 clean distclean print-repo-info
