CC = gcc
CFLAGS = -g -O3 -Wall -D__BLASTD__

OBJS = interloquendi.o quendi.o frameread.o blast.o crc.o \
       rendezvous.o quendiclient.o
BLAST_OBJS = $(OBJS) channels-blast.o
SPIDER_OBJS = $(OBJS) channels.o
HEADERS = quendi.h frameread.h channels.h bbc_pci.h blast.h derived.h quenya.h \
	  crc.h quendiclient.h

all: interloquendi-blast interloquendi-spider

$(OBJS): $(HEADERS)

channels-blast.o: channels.c
	$(CC) $(CFLAGS) -DDAS_CARDS=12 -c -o channels-blast.o channels.c

interloquendi-spider: $(SPIDER_OBJS)
	$(CC) $(SPIDER_OBJS) -lnsl -lwrap -o interloquendi-spider

interloquendi-blast: $(BLAST_OBJS)
	$(CC) $(BLAST_OBJS) -lnsl -lwrap -o interloquendi-blast

install:
	install -m 755 -o root -g root interloquendi /usr/local/sbin
	install -m 644 -o root -g root interloquendi.conf /etc

clean:
	rm -f *.o interloquendi-blast interloquendi-spider
