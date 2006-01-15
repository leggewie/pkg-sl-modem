
KERNEL_VER:=$(shell uname -r)
KERNEL_DIR:=/lib/modules/$(KERNEL_VER)/build
INSTALL_DIR:=/lib/modules/$(KERNEL_VER)/extra

obj-m := ungrab-winmodem.o


all:
	$(MAKE) modules -C $(KERNEL_DIR) SUBDIRS=$(shell pwd)

clean:
	$(RM) *.o *.ko *.mod.* .*.cmd *~
	$(RM) -r .tmp_versions

install: all
	install -D -m 644 ungrab-winmodem.ko $(INSTALL_DIR)/ungrab-winmodem.ko
	/sbin/depmod -a
uninstall:
	modprobe -r ungrab-winmodem ; echo -n
	$(RM) $(INSTALL_DIR)/ungrab-winmodem.ko
	/sbin/depmod -a

