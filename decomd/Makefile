CC = gcc
CFLAGS = -O3 -Wall -g $(EXTRAFLAGS)
CPPFLAGS = -D__DECOMD__
BLAST_CPPFLAGS = -DDAS_CARDS=12
LDFLAGS = -lpthread $(EXTRAFLAGS)

########################################################
# External repository dependencies
#

SPIDER_OR_BLAST=1   # we need at least one of Spider or BLAST config

########################################################
# Inputs
#

LOCAL_OBJS = decomd.o
SHARED_OBJS = channels.o framefile.o blast.o crc.o 
EXPT_OBJS = derived.o tx_struct.o
SHARED_HEADERS = bbc_pci.h blast.h channels.h crc.h derived.h
LOCAL_HEADERS = decom_pci.h

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Set up targets
#

BLAST_ALL = decomd-blast
SPIDER_ALL = decomd-spider

ifeq ($(HAVE_BLASTCONF),1)
	ALL += $(BLAST_ALL)
endif

ifeq ($(HAVE_SPIDERCONF),1)
	ALL += $(SPIDER_ALL)
endif

########################################################
# Build Stuff
#

all:: $(ALL)

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

tx_struct_spider.o :: tx_struct_mce.c

tx_struct_mce.c: ../spider_config/make_tx_struct_mce \
	../spider_config/tes.h
	../spider_config/make_tx_struct_mce > $@

decomd-blast: $(BLAST_OBJS) readdecom_blast.o
	  $(CC) $(LDFLAGS) $(BLAST_OBJS) readdecom_blast.o -o decomd-blast

decomd-spider: $(SPIDER_OBJS) readdecom_spider.o
	  $(CC) $(LDFLAGS) $(SPIDER_OBJS) readdecom_spider.o -o decomd-spider

clean:
	rm -f $(BLAST_ALL) $(SPIDER_ALL)
	rm -f *.o
	rm -f tx_struct_mce.c

########################################################
# Install stuff
#

install: all
ifeq ($(HAVE_BLASTCONF),1)
	install -m 700 -o root -g root decomd-blast /usr/local/sbin
endif
ifeq ($(HAVE_SPIDERCONF),1)
	install -m 700 -o root -g root decomd-spider /usr/local/sbin
endif
	install -m 755 -o root -g root decomctl /usr/local/sbin
	install -m 755 -o root -g root -d /data/etc/decomd
	install -m 644 decomd.8 decomctl.8 /usr/local/man/man8

install-blast: install
ifeq ($(HAVE_BLASTCONF),1)
	rm -f /usr/local/sbin/decomd
	ln -s /usr/local/sbin/decomd-blast /usr/local/sbin/decomd
else
	$(error Required repository missing.  Unable to continue)
endif

install-spider: install
ifeq ($(HAVE_SPIDERCONF),1)
	rm -f /usr/local/sbin/decomd
	ln -s /usr/local/sbin/decomd-spider /usr/local/sbin/decomd
else
	$(error Required repository missing.  Unable to continue)
endif

