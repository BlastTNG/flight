CC = gcc
CPPFLAGS = -D__MPC__
CFLAGS = -g -O3 -Wall
LDLIBS = -lm -lpthread

########################################################
# External repository dependencies
#

SPIDER_ONLY=1

########################################################
# Inputs
#

EXPT_HEADERS = command_list.h command_struct.h fset.h mpc_proto.h tes.h
SHARED_HEADERS = blast.h command_common.h udp.h 
LOCAL_HEADERS =

# These are needed by both bolotest and flight versions
EXPT_OBJS = command_list.o mpc_proto.o
SHARED_OBJS = blast.o command_common.o udp.o
LOCAL_OBJS = mpc.o

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

mpc: $(OBJS)
	$(CC) $(LDFLGS) $(CFLAGS) $(OBJS) $(LDLIBS) -o mpc

########################################################
# Install Stuff
#

install: mpc
	install -m 755 mpc /usr/mas/bin
