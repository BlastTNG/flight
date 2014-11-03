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
obj-m := bbc_pci.o

else
# First pass: normal build
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD		:= $(shell pwd)
CC    := gcc -O2 -g
CPPFLAGS = -I../$(SHARED)

all: bbc_pci.ko test_bbcpci_int test_bbcpci_ext test_biphase test_irq test_data test_spiderbiphase

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

bbc_pci.ko: bbc_pci.c
# invoke kernel build system
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) KCPPFLAGS=-I$(PWD)/../$(SHARED) modules

test_data: test_data.c
	$(CC) $(CPPFLAGS) -o test_data test_data.c

test_bbcpci_int: test_bbcpci.c
	$(CC) $(CPPFLAGS) -o test_bbcpci_int test_bbcpci.c

test_bbcpci_ext: test_bbcpci.c
	$(CC) $(CPPFLAGS) -DUSE_EXT_SERIAL -o test_bbcpci_ext test_bbcpci.c

test_biphase: test_biphase.c
	$(CC) $(CPPFLAGS) -o test_biphase test_biphase.c -lm

test_spiderbiphase: test_spiderbiphase.c
	$(CC) $(CPPFLAGS) -o test_spiderbiphase test_spiderbiphase.c -lm

test_irq: test_irq.c
	$(CC) $(CPPFLAGS) -o test_irq test_irq.c -lm

clean: 
	rm -f .bbc* *.ko *.o test_bbcpci_int test_bbcpci_ext test_biphase test_irq bbc_pci.mod.c

MODULEDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/misc

install: bbc_pci.ko
	install bbc_pci.ko -m 644 -D $(MODULEDIR)/bbc_pci.ko

endif
