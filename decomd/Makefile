CC = gcc
CFLAGS = -O3 -Wall -D__DECOMD__ $(EXTRAFLAGS)
LDFLAGS = -lpthread $(EXTRAFLAGS)

all: decomd

OBJS = decomd.o crc.o framefile.o channels.o tx_struct.o blast.o
HEADERS = bbc_pci.h crc.h decomd.h decom_pci.h channels.h blast.h

decomd: $(OBJS)

$(OBJS): $(HEADERS)

clean:
	rm -f decomd
	rm -f *.o

install:
	install -m 700 -o root -g root decomd /usr/local/sbin
	install -m 755 -o root -g root decomctl /usr/local/sbin
	install -m 644 decomd.8 decomctl.8 /usr/local/man/man8
