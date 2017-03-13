# In order to build the kernel module, this file is processed twice,
# first as a regular build, and once after invoking the kernel
# build system.

# Note: to use the kernel build system, you must have write access
# to /usr/src/linux-`uname -r`/.tmp_versions/<module>_mod as well as
# /usr/src/linux-`uname -r`/.__modpost.cmd .  The easiest ways to do
# this is to make them world writeable (which has been done on frodo).

ifneq ($(KERNELRELEASE),)
# Second pass: inside kernel build system
# All we need to do here is delcare the module objects.
# The kernel build system will do everything else.
obj-m := decom_pci.o

else
# First pass: normal build
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD		:= $(shell pwd)
CC    := gcc -O2 -g

all: decom_pci.ko

decom_pci.ko: decom_pci.c decom_pci.h
# invoke kernel build system
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean: 
	rm -f .decom* *.ko *.o decom_pci.mod.c

install: decom_pci.ko
#	[ -e /dev/decom_pci ] || ( mknod /dev/decom_pci c 251 0 && chmod 777 /dev/decom_pci )
	install -m 644 decom_pci.h /usr/local/include/decom_pci.h
	install decom_pci.ko -m 644 /usr/local/lib/decom_pci.ko

# Commenting it out since we don't use svn anymore. Not sure if this is needed.
# svn aliases
#include ../common/Makefile.update
#
#../common/Makefile.update:
#	@echo "ERROR: **************************************************************"
#	@echo "Common build files not found; unable to continue.  Try:"
#	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
#	@false
#

endif
