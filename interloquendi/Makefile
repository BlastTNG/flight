CC = gcc
CFLAGS = -g -O3 -Wall
CPPFLAGS = -D__BLASTD__
BLAST_CPPFLAGS = -DDAS_CARDS=12

########################################################
# Inputs
#

LOCAL_OBJS = interloquendi.o quendi.o rendezvous.o
SHARED_OBJS = blast.o channels.o crc.o quendiclient.o frameread.o
SHARED_HEADERS = frameread.h channels.h bbc_pci.h blast.h derived.h crc.h \
								 quendiclient.h quenya.h
LOCAL_HEADERS = quendi.h

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Build Stuff
#

all:: interloquendi-blast interloquendi-spider

interloquendi-spider: $(SPIDER_OBJS)
	$(CC) $(SPIDER_OBJS) -lnsl -lwrap -o interloquendi-spider

interloquendi-blast: $(BLAST_OBJS)
	$(CC) $(BLAST_OBJS) -lnsl -lwrap -o interloquendi-blast

########################################################
# Install Stuff
#

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

install:
	install -m 755 -o root -g root interloquendi-blast /usr/local/sbin
	install -m 755 -o root -g root interloquendi-spider /usr/local/sbin
	install -m 644 -o root -g root interloquendi.conf /etc
	@echo "\n*** NB *** On decom systems, you should run: make install-decom"

install-decom:
	install -m 755 -o root -g root interloquendi-blast /usr/local/sbin
	install -m 755 -o root -g root interloquendi-spider /usr/local/sbin
	install -m 644 -o root -g root interloquendi.conf.decom /etc/interloquendi.conf

clean:
	rm -f *.o interloquendi-blast interloquendi-spider
	rm -f share/*.o
