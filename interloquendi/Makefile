CC = gcc
CFLAGS = -O3 -Wall -D__BLASTD__

OBJS = interloquendi.o channels.o quendi.o frameread.o blast.o crc.o
HEADERS = quendi.h frameread.h channels.h bbc_pci.h blast.h derived.h quenya.h \
					crc.h

all: interloquendi

interloquendi: $(OBJS)
	$(CC) $(OBJS) -lnsl -lwrap -o interloquendi

$(OBJS): $(HEADERS)

clean:
	rm -f *.o interloquendi
