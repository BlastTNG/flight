CC = gcc
CFLAGS = -O3 -Wall -D__DECOMD__
LDFLAGS = -lpthread

all: decomd

OBJS = decomd.o crc.o framefile.o channels.o tx_struct.o
HEADERS = bbc_pci.h crc.h decomd.h decom_pci.h tx_struct.h

decomd: $(OBJS)

$(OBJS): $(HEADERS)

clean:
	rm -f decomd
	rm -f *.o
