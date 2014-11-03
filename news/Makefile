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

framelist.inc: ../spider_config/tx_struct.c tx_struct_mce.c
	../spider_config/make_framelist_inc > $@

compressstruct_spider.o :: framelist.inc

tx_struct_spider.o :: tx_struct_mce.c

tx_struct_mce.c: ../spider_config/make_tx_struct_mce \
	../spider_config/tes.h
	../spider_config/make_tx_struct_mce > $@

fox-blast: fox_blast.o $(BLAST_OBJS)
	$(CC) -o fox-blast $^ -lm 

fox-spider: fox_spider.o $(SPIDER_OBJS)
	$(CC) -o fox-spider $^ -lm 

rumor-blast: rumor.o $(BLAST_OBJS)
	$(CC) -o rumor-blast rumor.o $(BLAST_OBJS) -lm -lgetdata 

rumor-spider: rumor.o $(SPIDER_OBJS)
	$(CC) -o rumor-spider rumor.o $(SPIDER_OBJS) -lm -lgetdata

# Hard-links
msnbc-blast rush-blast: fox-blast
	ln -f $< $@

msnbc-spider rush-spider: fox-spider
	ln -f $< $@

clean:
	rm -f $(BLAST_ALL) $(SPIDER_ALL)
	rm -f *.o framelist.inc
	rm -f tx_struct_mce.c

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

