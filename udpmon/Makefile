CC = gcc
CPPFLAGS =
CFLAGS = -g -O3 -Wall
LDLIBS = -lm

########################################################
# Inputs
#

SHARED_HEADERS = blast.h udp.h
LOCAL_HEADERS = 

# These are needed by both bolotest and flight versions
SHARED_OBJS = blast.o udp.o
LOCAL_OBJS = udpmon.o tesmon.o

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Build Stuff
#

.PHONY: all

all:: udpmon tesmon

udpmon: udpmon.o blast.o udp.o
tesmon: tesmon.o blast.o udp.o

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

clean:
	rm -f udpmon || true;
	rm -f *.o || true;

