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
obj-m := bbc_pci.o

else
# First pass: normal build
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD		:= $(shell pwd)
CC    := gcc -O2 -g

all: bbc_pci.ko test_bbcpci

bbc_pci.ko: bbc_pci.c bbc_pci.h
# invoke kernel build system
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

test_bbcpci: test_bbcpci.c
	$(CC) -o test_bbcpci test_bbcpci.c -lm

echo_test: echo_test.c
	$(CC) -o echo_test echo_test.c -lm

clean: 
	rm -f .bbc* *.ko *.o test_bbcpci bbc_pci.mod.c

install: bbc_pci.ko
	install -m 644 bbc_pci.h /usr/local/include/bbc_pci.h
	install bbc_pci.ko -m 644 /usr/local/lib/bbc_pci.ko

endif
