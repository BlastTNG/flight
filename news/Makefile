CC=gcc
CFLAGS = -O2 -W -Wall

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

fox-blast: fox.o $(BLAST_OBJS)
	$(CC) -lm -o fox-blast fox.o $(BLAST_OBJS)

msnbc-blast: fox.o $(BLAST_OBJS)
	$(CC) -lm -o msnbc-blast fox.o $(BLAST_OBJS)

rush-blast: fox.o $(BLAST_OBJS)
	$(CC) -lm -o rush-blast fox.o $(BLAST_OBJS)

rumor-blast: rumor.o $(BLAST_OBJS)
	$(CC) -lm -lgetdata -o rumor-blast rumor.o $(BLAST_OBJS)

fox-spider: fox.o $(SPIDER_OBJS)
	$(CC) -lm -o fox-spider fox.o $(SPIDER_OBJS)

msnbc-spider: fox.o $(SPIDER_OBJS)
	$(CC) -lm -o msnbc-spider fox.o $(SPIDER_OBJS)

rush-spider: fox.o $(SPIDER_OBJS)
	$(CC) -lm -o rush-spider fox.o $(SPIDER_OBJS)

rumor-spider: rumor.o $(SPIDER_OBJS)
	$(CC) -lm -lgetdata -o rumor-spider rumor.o $(SPIDER_OBJS) 

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

