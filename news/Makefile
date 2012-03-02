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
LOCAL_OBJS = fox.o
EXPT_OBJS = tx_struct.o compressstruct.o derived.o

SHARED_HEADERS = compressstruct.h channels.h derived.h

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Set up targets
#

BLAST_ALL = fox-blast msnbc-blast rush-blast
SPIDER_ALL = fox-spider msnbc-spider rush-spider

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
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common"
	@false

fox-blast: $(BLAST_OBJS)
	$(CC) -lm -o fox-blast $(BLAST_OBJS)

msnbc-blast: $(BLAST_OBJS)
	$(CC) -lm -o msnbc-blast $(BLAST_OBJS)

rush-blast: $(BLAST_OBJS)
	$(CC) -lm -o rush-blast $(BLAST_OBJS)

fox-spider: $(SPIDER_OBJS)
	$(CC) -lm -o fox-spider $(SPIDER_OBJS)

msnbc-spider: $(SPIDER_OBJS)
	$(CC) -lm -o msnbc-spider $(SPIDER_OBJS)

rush-spider: $(SPIDER_OBJS)
	$(CC) -lm -o rush-spider $(SPIDER_OBJS)

clean:
	rm -f $(BLAST_ALL) $(SPIDER_ALL)
	rm -f *.o

install: 
	install -m 755 fox /usr/local/bin
	install -m 755 msnbc /usr/local/bin
	install -m 755 rush /usr/local/bin

