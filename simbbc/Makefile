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
obj-m := simbbc.o

else
# First pass: normal build
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
CC    	:= gcc -O2 -g

all: simbbc.ko

simbbc.ko: simbbc.c simbbc.h
# invoke kernel build system
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules


clean: 
	rm -f .simbbc* 
	rm -f *.ko 
	rm -f *.o 
	rm -f simbbc.mod.c

purge:
	rm -f *~

MODULEDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/misc

install: simbbc.ko
	install simbbc.ko -m 644 -D $(MODULEDIR)/simbbc.ko
	depmod simbbc.ko
	install simbbc -m 744 /etc/init.d/simbbc
endif
