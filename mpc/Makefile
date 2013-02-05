CC = gcc
CPPFLAGS = -D__MPC__
CFLAGS = -g -O3 -Wall
LDLIBS = -lm

########################################################
# External repository dependencies
#

SPIDER_ONLY=1

########################################################
# Inputs
#

EXPT_HEADERS = 
SHARED_HEADERS = 
LOCAL_HEADERS = 

# These are needed by both bolotest and flight versions
SHARED_OBJS = 
LOCAL_OBJS = mpc.o
EXPT_OBJS =

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Build Stuff
#

.PHONY: all

all:: mpc

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

clean:
	rm -f mpc
	rm -f *.o

pcm: $(OBJS)
	$(CC) $(LDFLGS) $(CFLAGS) $(OBJS) $(LDLIBS) -o mpc

########################################################
# Install Stuff
#

install: mpc
	install -m 755 mpc /usr/mas/bin
