CC = gcc
CFLAGS = -O3 -Wall -D__BLASTD__

OBJS = interloquendi.o channels.o quendi.o frameread.o blast.o crc.o \
			 rendezvous.o quendiclient.o
HEADERS = quendi.h frameread.h channels.h bbc_pci.h blast.h derived.h quenya.h \
					crc.h quendiclient.h

all: interloquendi

interloquendi: $(OBJS)
	$(CC) $(OBJS) -lnsl -lwrap -o interloquendi

$(OBJS): $(HEADERS)

install:
	install -m 755 -o root -g root interloquendi /usr/local/sbin
	install -m 644 -o root -g root interloquendi.conf /etc

clean:
	rm -f *.o interloquendi
