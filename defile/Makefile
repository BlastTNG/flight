TOPDIR=.
CC = gcc
CFLAGS = -O3 -Wall -D__DEFILE__
LDFLAGS = -lpthread -lz

all: Makefile.config defile
	cd $(TOPDIR)/etc && $(MAKE) all

debug:
	${MAKE} CFLAGS="$(CFLAGS) -g" LDFLAGS="$(LDFLAGS) -g" all

defile: defile.o reader.o channels.o writer.o

defile.o: defile.c defile.h tx_struct.h

reader.o: reader.c tx_struct.h defile.h

channels.o: channels.c tx_struct.h

writer.o: writer.c tx_struct.h defile.h

clean:
	rm -f *.o defile
	cd $(TOPDIR)/etc && $(MAKE) clean
	rm Makefile.config

install: defile
	install -m 755 defile /usr/local/bin
	install -m 644 defile.1 /usr/local/man/man1
	cd $(TOPDIR)/etc && $(MAKE) install

Makefile.config:
	./configure
