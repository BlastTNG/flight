CC = gcc
CPPFLAGS =
CFLAGS = -g -O3 -Wall
LDLIBS = -lm

########################################################
# External repository dependencies
#

SPIDER_ONLY=1

########################################################
# Inputs
#

SHARED_HEADERS = crc.h
EXPT_HEADERS = mce_blob.h
LOCAL_HEADERS = 

# These are needed by both bolotest and flight versions
SHARED_OBJS = crc.o
LOCAL_OBJS = unblob.o

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Build Stuff
#

.PHONY: all

all:: unblob

LDFLAGS = -lgetdata

unblob: $(SPIDER_OBJS)

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

clean:
	rm -f unblob || true;
	rm -f *.o || true;

