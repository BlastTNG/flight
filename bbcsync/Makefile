CFLAGS += -Wall -DDEBUG
ifneq ($(KERNELRELEASE),)  # call from kernel build system
  obj-m   := bbc_sync.o

else
  KERNELDIR ?= /lib/modules/$(shell uname -r)/build
  PWD       := $(shell pwd)
  CONFIG_MODVERSIONS=y

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)  modules

endif


MODULEDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/misc

install: bbc_sync.ko
	install -m 644 -D bbc_sync.h /usr/local/include/bbc_sync.h
	install -m 644 -D 15-bbcsync.rules /etc/udev/rules.d/15-bbcsync.rules
#	install -m 644 -D bbc_sync.ko $(MODULEDIR)/bbc_sync.ko

clean:
	-rm -f *.o *.ko *.~*~ TAGS


.PHONY: tags
tags: TAGS

TAGLIST := bbc_sync.c bbc_sync.h
TAGS: $(TAGLIST)
	etags $(TAGLIST)

