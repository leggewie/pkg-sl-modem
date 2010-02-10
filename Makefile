###########################################################################
#
#
#       Makefile  --  modem Makefile.
#
#       Copyright(c) 2003, Smart Link Ltd. (www.smlink.com)
#	All rights reserved.
#
#       Author: Sasha K (sashak@smlink.com)
#
#
###########################################################################
#
###########################################################################

KERNEL_DIR:=/lib/modules/$(shell uname -r)/build

# tools
INSTALL:=install

all: modem drivers

modem:
	$(MAKE) -C $@ all

install: all install-drivers
	$(INSTALL) -D -m 755 modem/slmodemd ${DESTDIR}/usr/sbin/slmodemd
	$(RM) -rf ${DESTDIR}/var/lib/slmodem
	$(INSTALL) -d -D -m 755 ${DESTDIR}/var/lib/slmodem

uninstall: uninstall-drivers
	$(RM) ${DESTDIR}/usr/sbin/slmodemd
	$(RM) -rf ${DESTDIR}/var/lib/slmodem

drivers:
	$(MAKE) -C drivers KERNEL_DIR=$(KERNEL_DIR)

install-drivers:
	$(MAKE) install -C drivers KERNEL_DIR=$(KERNEL_DIR)
uninstall-drivers:
	$(MAKE) uninstall -C drivers KERNEL_DIR=$(KERNEL_DIR)

# misc rules
sub-dirs:= modem drivers
.PHONY: $(sub-dirs) all old clean dep install
clean dep: %: %-sub-dirs
%-sub-dirs:
	$(foreach dir,$(sub-dirs),$(MAKE) -C $(dir) $(patsubst %-sub-dirs,%,$@) && ) echo "done."

