CC = gcc
CFLAGS = -O3 -Wall -D__DECOMD__ $(EXTRAFLAGS)
LDFLAGS = -lpthread $(EXTRAFLAGS)

all: decomd

OBJS = decomd.o crc.o framefile.o channels.o tx_struct.o
HEADERS = bbc_pci.h crc.h decomd.h decom_pci.h tx_struct.h

decomd: $(OBJS)

$(OBJS): $(HEADERS)

clean:
	rm -f decomd
	rm -f *.o

install:
	install -m 755 decomd /usr/local/sbin
	install -m 644 defile.8 /usr/local/man/man8
