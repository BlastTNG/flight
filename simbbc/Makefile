# In order to build the kernel module, this file is processed twice,
# first as a regular build, and once after invoking the kernel
# build system.

# Note: to use the kernel build system, you must have write access
# to /usr/src/linux-`uname -r`/.tmp_versions/<module>_mod as well as
# /usr/src/linux-`uname -r`/.__modpost.cmd .  The easiest ways to do
# this is to make them world writeable (which has been done on frodo).

# the name of the shared code project in the repo
SHARED = common

ifneq ($(KERNELRELEASE),)
# Second pass: inside kernel build system
# All we need to do here is delcare the module objects.
# The kernel build system will do everything else.
obj-m := simbbc_pci.o
simbbc_pci-objs := simbbc.o simdata.o

else
# First pass: normal build
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
CC    	:= gcc -O2 -g 
CPPFLAGS = -I../$(SHARED)

all: simbbc_pci.ko

simbbc_pci.ko: simbbc.c simbbc.h simdata.c simdata.h
# invoke kernel build system
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) KCPPFLAGS=-I$(PWD)/../$(SHARED) modules

up update:
	@for dir in $(SHARED); do \
		if [ -d ../$$dir ]; then \
			echo "Updating $$dir..."; \
			( cd ../$$dir && svn up ); \
		else \
			echo; \
			echo "ERROR: **************************************"; \
			echo "Project ../$$dir missing! try running:"; \
			echo "  ( cd .. && svn co $$dir )"; \
			false; \
		fi; \
	done

clean: 
	rm -f .simbbc* 
	rm -f *.ko 
	rm -f *.o 
	rm -f simbbc.mod.c

purge:
	rm -f *~

MODULEDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/misc

install: simbbc_pci.ko
	install simbbc_pci.ko -m 644 -D $(MODULEDIR)/simbbc_pci.ko
	depmod $(MODULEDIR)/simbbc_pci.ko
	install simbbc -m 755 /etc/init.d/simbbc
endif
