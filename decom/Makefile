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
KVER  	:= $(shell uname -r)
KDIR	:= /lib/modules/$(KVER)/build
PWD	:= $(shell pwd)
CC 	:= gcc -O2 -g

all: decom_pci.ko

decom_pci.ko: decom_pci.c decom_pci.h
# invoke kernel build system
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean: 
	rm -f .decom* *.ko *.o decom_pci.mod.c

install: decom_pci.ko
	install -m 644 decom_pci.h /usr/local/include/decom_pci.h
	install -m 644 -D decom_pci.ko /lib/modules/$(KVER)/misc/decom_pci.ko
	depmod $(KVER)

#	install decom_pci.ko -m 644 /usr/local/lib/decom_pci.ko # This was the previous install


endif
