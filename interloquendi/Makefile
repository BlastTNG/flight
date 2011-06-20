CC = gcc
CFLAGS = -g -O3 -Wall -D__BLASTD__

OBJS = interloquendi.o quendi.o frameread.o share/blast.o share/crc.o \
       rendezvous.o quendiclient.o
BLAST_OBJS = $(OBJS) share/channels-blast.o
SPIDER_OBJS = $(OBJS) share/channels.o
HEADERS = quendi.h frameread.h share/channels.h share/bbc_pci.h share/blast.h \
	  share/derived.h quenya.h share/crc.h quendiclient.h

all: interloquendi-blast interloquendi-spider

$(OBJS): $(HEADERS)

share/channels-blast.o: share/channels.c
	$(CC) $(CFLAGS) -DDAS_CARDS=12 \
	  -c -o share/channels-blast.o share/channels.c

interloquendi-spider: $(SPIDER_OBJS)
	$(CC) $(SPIDER_OBJS) -lnsl -lwrap -o interloquendi-spider

interloquendi-blast: $(BLAST_OBJS)
	$(CC) $(BLAST_OBJS) -lnsl -lwrap -o interloquendi-blast

install:
	install -m 755 -o root -g root interloquendi /usr/local/sbin
	install -m 644 -o root -g root interloquendi.conf /etc

clean:
	rm -f *.o interloquendi-blast interloquendi-spider
	rm -f share/*.o
