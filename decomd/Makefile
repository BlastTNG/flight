CC = gcc
CFLAGS = -O3 -Wall -g $(EXTRAFLAGS) -I ../common/include/
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
SHARED_OBJS = channels_tng.o framefile.o blast.o crc.o 
EXPT_OBJS = derived.o tx_struct.o
SHARED_HEADERS = bbc_pci.h blast.h channels_tng.h crc.h derived.h
LOCAL_HEADERS = decom_pci.h

########################################################
# Run the sanity checks and prepare the build system
#


########################################################
# Set up targets
#

BLAST_ALL = decomd-blast
ALL += $(BLAST_ALL)

########################################################
# Build Stuff
#

all:: $(ALL)

decomd-blast: $(BLAST_OBJS) readdecom_blast.o
	  $(CC) $(LDFLAGS) $(BLAST_OBJS) readdecom_blast.o -o decomd-blast

clean:
	rm -f $(BLAST_ALL) 
	rm -f *.o

########################################################
# Install stuff
#

install: all
ifeq ($(HAVE_BLASTCONF),1)
	install -m 700 -o root -g root decomd-blast /usr/local/sbin
endif
	install -m 755 -o root -g root decomctl /usr/local/sbin
	install -m 755 -o root -g root -d /data/etc/decomd
	install -m 644 decomd.8 decomctl.8 /usr/local/man/man8

install-blast: install
	rm -f /usr/local/sbin/decomd
	ln -s /usr/local/sbin/decomd-blast /usr/local/sbin/decomd


