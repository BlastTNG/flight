CC = gcc
CXX = g++
CFLAGS = -g -O3 -Wall
CPPFLAGS = -D__MCP__ 
LDFLAGS = -lpthread -lm -lstdc++ 
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

########################################################
# External repository dependencies
#

MINICP_ONLY=1

########################################################
# Inputs
#

EXPT_HEADERS = command_list.h
SHARED_HEADERS = bbc_pci.h blast.h channels.h derived.h netcmd.h
LOCAL_HEADERS = mcp.h tx.h command_struct.h

EXPT_OBJS = command_list.o
LOCAL_OBJS = az-el.o derived.o hk.o mcp.o tx.o commands.o tx_struct.o
SHARED_OBJS = blast.o channels.o framefile.o

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

.PHONY: fifo clean install all

########################################################
# Build Stuff
#

all:: minicp

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

clean:
	rm -f minicp || true;
	rm -f *.o || true;
	rm -f share/*.o || true;

minicp: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o minicp

########################################################
# Install Stuff
#

install: minicp fifo
	install -m 755 minicp /usr/local/bin
	install -d -m 777 /data/etc/minicp

fifo:
	[ -e /data/etc/SIPSS.FIFO ] || mkfifo -m 0777 /data/etc/SIPSS.FIFO
