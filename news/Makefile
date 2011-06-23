ALL = fox-blast msnbc-blast rush-blast fox-spider msnbc-spider rush-spider

CC=gcc
CFLAGS = -O2 -W -Wall
BLAST_CFLAGS = $(CFLAGS) -D__BLAST__
SPIDER_CFLAGS = $(CFLAGS) -D__SPIDER__

OBJS = fox.o share/compressfuncs.o
BLAST_OBJS = $(OBJS) tx_struct_blast.o compressstruct_blast.o derived_blast.o 
SPIDER_OBJS = $(OBJS) tx_struct_spider.o \
	      compressstruct_spider.o derived_spider.o 

HEADERS = calibrate.h share/compressstruct.h share/channels.h share/derived.h

.PHONEY: all install clean

all: $(ALL)

$(OBJS): $(HEADERS)

tx_struct_blast.o: tx_struct_blast.c
	$(CC) $(BLAST_CFLAGS) -c -o tx_struct_blast.o tx_struct_blast.c

tx_struct_spider.o: tx_struct_spider.c
	$(CC) $(SPIDER_CFLAGS) -c -o tx_struct_spider.o tx_struct_spider.c

fox-blast: $(BLAST_OBJS)
	$(CC) $(BLAST_CFLAGS) -lm -o fox-blast $(BLAST_OBJS)

msnbc-blast: $(BLAST_OBJS)
	$(CC) $(BLAST_CFLAGS) -lm -o msnbc-blast $(BLAST_OBJS)

rush-blast: $(BLAST_OBJS)
	$(CC) $(BLAST_CFLAGS) -lm -o rush-blast $(BLAST_OBJS)

fox-spider: $(SPIDER_OBJS)
	$(CC) $(SPIDER_CFLAGS) -lm -o fox-spider $(SPIDER_OBJS)

msnbc-spider: $(SPIDER_OBJS)
	$(CC) $(SPIDER_CFLAGS) -lm -o msnbc-spider $(SPIDER_OBJS)

rush-spider: $(SPIDER_OBJS)
	$(CC) $(SPIDER_CFLAGS) -lm -o rush-spider $(SPIDER_OBJS)

clean:
	rm -f fox-blast msnbc-blast rush-blast || true;
	rm -f fox-spider msnbc-spider rush-spider || true;
	rm -f *.o || true;
	rm -f share/*.o || true;

install: all
	install -m 755 fox /usr/local/bin
	install -m 755 msnbc /usr/local/bin
	install -m 755 rush /usr/local/bin

