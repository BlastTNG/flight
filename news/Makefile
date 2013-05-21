CC=gcc
CFLAGS = -g 

########################################################
# External repository dependencies
#

SPIDER_OR_BLAST=1   # we need at least one of Spider or BLAST config

########################################################
# Inputs
#

SHARED_OBJS = compressfuncs.o
LOCAL_OBJS = news_utils.o
EXPT_OBJS = tx_struct.o compressstruct.o derived.o slowdl_struct.o

SHARED_HEADERS = compressstruct.h channels.h derived.h

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Set up targets
#

BLAST_ALL = fox-blast msnbc-blast rush-blast rumor-blast
SPIDER_ALL = fox-spider msnbc-spider rush-spider rumor-spider

ifeq ($(HAVE_BLASTCONF),1)
	ALL += $(BLAST_ALL)
endif

ifeq ($(HAVE_SPIDERCONF),1)
	ALL += $(SPIDER_ALL)
endif

########################################################
# Build Stuff
#

.PHONY: install clean

all:: $(ALL)

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

fox-blast.o: fox.c
	$(CC) -c -I../common -I../blast_config fox.c -o fox-blast.o

fox-spider.o: fox.c
	$(CC) -c -I../common -I../spider_config fox.c -o fox-spider.o

fox-blast: fox-blast.o $(BLAST_OBJS)
	$(CC) -o fox-blast fox-blast.o $(BLAST_OBJS) -lm 

msnbc-blast: fox-blast.o $(BLAST_OBJS)
	$(CC) -o msnbc-blast fox-blast.o $(BLAST_OBJS) -lm 

rush-blast: fox-blast.o $(BLAST_OBJS)
	$(CC) -o rush-blast fox-blast.o $(BLAST_OBJS) -lm 

rumor-blast: rumor.o $(BLAST_OBJS)
	$(CC) -o rumor-blast rumor.o $(BLAST_OBJS) -lm -lgetdata 

fox-spider: fox-spider.o $(SPIDER_OBJS)
	$(CC) -o fox-spider fox-spider.o $(SPIDER_OBJS) -lm 

msnbc-spider: fox-spider.o $(SPIDER_OBJS)
	$(CC) -o msnbc-spider fox-spider.o $(SPIDER_OBJS) -lm 

rush-spider: fox-spider.o $(SPIDER_OBJS)
	$(CC) -o rush-spider fox-spider.o $(SPIDER_OBJS) -lm 

rumor-spider: rumor.o $(SPIDER_OBJS)
	$(CC) -o rumor-spider rumor.o $(SPIDER_OBJS) -lm -lgetdata

clean:
	rm -f $(BLAST_ALL) $(SPIDER_ALL)
	rm -f *.o

########################################################
# Install stuff
#

install: all
	rm -f /usr/local/bin/fox
	rm -f /usr/local/bin/msnbc
	rm -f /usr/local/bin/rush
ifeq ($(HAVE_BLASTCONF),1)
	install -m 755 fox-blast   /usr/local/bin
	install -m 755 msnbc-blast /usr/local/bin
	install -m 755 rush-blast  /usr/local/bin
endif
ifeq ($(HAVE_SPIDERCONF),1)
	install -m 755 fox-spider   /usr/local/bin
	install -m 755 msnbc-spider /usr/local/bin
	install -m 755 rush-spider  /usr/local/bin
endif

