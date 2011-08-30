CC = gcc
CFLAGS = -O3 -Wall -g -D__DECOMD__ $(EXTRAFLAGS)
BLAST_CFLAGS = $(CFLAGS) -DDAS_CARDS=12 -D__BLAST__
SPIDER_CFLAGS = $(CFLAGS) -D__SPIDER__
LDFLAGS = -lpthread $(EXTRAFLAGS)

all: decomd-blast decomd-spider

OBJS = decomd.o share/blast.o share/crc.o 
BLAST_OBJS = $(OBJS) share/channels-blast.o share/framefile-blast.o \
	     derived_blast.o tx_struct_blast.o
SPIDER_OBJS = $(OBJS) share/channels.o share/framefile.o \
	     derived_spider.o tx_struct_spider.o
HEADERS = share/bbc_pci.h share/blast.h share/channels.h share/crc.h \
	  decom_pci.h share/derived.h

share/framefile-blast.o: share/framefile.c
	$(CC) $(BLAST_CFLAGS) -c -o share/framefile-blast.o share/framefile.c

share/channels-blast.o: share/channels.c
	$(CC) $(BLAST_CFLAGS) -c -o share/channels-blast.o share/channels.c

tx_struct_blast.o: tx_struct_blast.c
	$(CC) $(BLAST_CFLAGS) -c -o tx_struct_blast.o tx_struct_blast.c

tx_struct_spider.o: tx_struct_spider.c
	$(CC) $(SPIDER_CFLAGS) -c -o tx_struct_spider.o tx_struct_spider.c

decomd-blast: $(BLAST_OBJS)
	$(CC) $(LDFLAGS) $(BLAST_OBJS) -o decomd-blast

decomd-spider: $(SPIDER_OBJS)
	$(CC) $(LDFLAGS) $(SPIDER_OBJS) -o decomd-spider

$(OBJS): $(HEADERS)

clean:
	rm -f decomd
	rm -f *.o

install: all
	install -m 700 -o root -g root decomd-blast /usr/local/sbin
	install -m 700 -o root -g root decomd-spider /usr/local/sbin
	install -m 755 -o root -g root decomctl /usr/local/sbin
	install -m 644 decomd.8 decomctl.8 /usr/local/man/man8

install-blast: install
	rm -f /usr/local/sbin/decomd
	ln -s /usr/local/sbin/decomd-blast /usr/local/sbin/decomd

install-spider: install
	rm -f /usr/local/sbin/decomd
	ln -s /usr/local/sbin/decomd-spider /usr/local/sbin/decomd

