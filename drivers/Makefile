###########################################################################
#
#
#       Makefile  --  modem kernel drivers Makefile.
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

#KBUILD_VERBOSE=1
#export KBUILD_VERBOSE

KERNEL_DIR:=/lib/modules/$(shell uname -r)/build

EXTRA_CFLAGS = -I$(obj) -I$(obj)/../modem

obj-m := slamr.o slusb.o

slamr-objs:= amrmo_init.o sysdep_amr.o amrlibs.o
slusb-objs:= st7554.o

ifndef KERNELRELEASE
ifndef KERNEL_VER


KVERS?=$(shell ./kernel-ver)



all install uninstall: kernel-ver
	$(MAKE) $@ KERNEL_VER=$(KVERS)
#	$(MAKE) $@ KERNEL_VER=$(shell ./kernel-ver)

install: install-devices
uninstall: remove-devices

kernel-ver::
	$(CC) -I$(KERNEL_DIR)/include -o $@ $@.c

dep:
clean:
	$(RM) kernel-ver $(obj-m) $(obj-m:.o=.ko) *st7554.o amrmo_init.o sysdep_amr.o *.mod.* .*.cmd *~
	$(RM) -r .tmp_versions

install-devices:
	mkdir -p ${DESTDIR}/dev
	$(foreach minor,0 1 2 3, \
	    mknod -m 600 ${DESTDIR}/dev/slamr$(minor) c 242 $(minor) ; ) echo -n
	$(foreach minor,0 1 2 3, \
	    mknod -m 600 ${DESTDIR}/dev/slusb$(minor) c 243 $(minor) ; ) echo -n
remove-devices:
	$(foreach minor,0 1 2 3, \
	    $(RM) ${DESTDIR}/dev/slamr$(minor) ; ) echo -n
	$(foreach minor,0 1 2 3, \
	    $(RM) ${DESTDIR}/dev/slusb$(minor) ; ) echo -n

else

ifeq ($(shell grep 'UTS.*"2\.6' $(KERNEL_DIR)/include/linux/version.h),)

slusb-objs:= old_st7554.o
obj:=.
module-dir ?= ${DESTDIR}/lib/modules/$(KERNEL_VER)/misc


CFLAGS:= -Wall -pipe -O3 -fomit-frame-pointer -D__KERNEL__ -DMODULE -DEXPORT_SYMTAB `test -f $(KERNEL_DIR)/include/linux/modversions.h && echo -DMODVERSIONS --include $(KERNEL_DIR)/include/linux/modversions.h -I$(KERNEL_DIR)/include`

all: $(obj-m)

slamr.o: $(slamr-objs)
slusb.o: $(slusb-objs)
slamr.o slusb.o:
	$(LD) -r -o $@ $^

install: uninstall-old
	install -D -m 644 slamr.o $(module-dir)/slamr.o
	install -D -m 644 slusb.o $(module-dir)/slusb.o
	cp /etc/modules.conf /etc/modules.conf.slamr && \
	    echo 'alias char-major-242 slamr' >> /etc/modules.conf && \
	    echo 'alias char-major-243 slusb' >> /etc/modules.conf 
	/sbin/depmod -a
uninstall:
	/sbin/modprobe -r slamr slusb
	cp /etc/modules.conf /etc/modules.conf.slamr && \
	   egrep -ve 'alias char-major-242 slamr|alias char-major-243 slusb' /etc/modules.conf.slamr > /etc/modules.conf
	$(RM) $(module-dir)/slamr.o
	$(RM) $(module-dir)/slusb.o
	/sbin/depmod -a
uninstall-old:
	$(RM) $(module-dir)/slmdm.o \
              $(module-dir)/slfax.o \
              $(module-dir)/slusb.o \
              $(module-dir)/slamrmo.o \
              $(module-dir)/slmodem.o  # remove old 2.7,2.8 version

%.o: %.c
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $(CFLAGS_$@) -o $@ -c $<

else

module-dir:=${DESTDIR}/lib/modules/$(KERNEL_VER)/extra

all:
	$(MAKE) modules -C $(KERNEL_DIR) SUBDIRS=$(shell pwd)
install:
	install -D -m 644 slamr.ko $(module-dir)/slamr.ko
	install -D -m 644 slusb.ko $(module-dir)/slusb.ko
	/sbin/depmod -a
uninstall:
	modprobe -r slamr ; echo -n
	modprobe -r slusb ; echo -n
	$(RM) $(module-dir)/slamr.ko
	$(RM) $(module-dir)/slusb.ko
	/sbin/depmod -a
endif

endif
endif

$(obj)/amrlibs.o:
	echo "$@ done"

