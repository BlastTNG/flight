include Makefile.config

CC = gcc
CPP = g++
LD = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ 
LDFLAGS = -lpthread -lm -lstdc++
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

USE_FIFO_CMD = 0
ifeq ($(USE_FIFO_CMD), 1)
	CPPFLAGS := $(CPPFLAGS) -DUSE_FIFO_CMD
endif

HEADERS = bbc_pci.h blast.h channels.h derived.h mcp.h tx.h command_list.h \
	  drivecommunicator.h motorcommand.h pointing_struct.h \
	  command_struct.h camcommunicator.h camstruct.h slow_dl.h

OBJS = mcp.o blast.o channels.o derived.o framefile.o tx.o tx_struct.o \
       commands.o drivecommunicator.o motorcommand.o motors.o pointing.o \
       command_list.o camcommunicator.o starcamera.o slow_dl.o

.PHONEY: fifo install all debug

all: mcp
ifeq ($(DEBUG), 1)
	@echo "******************** WARNING ********************"
	@echo "Debugging symbols are being output to the binary"
	@echo "******************** WARNING ********************"
endif

debug::
	${MAKE} DEBUG=1 all

debug:: all

clean:
	rm -f mcp || true;
	rm -f *.o || true;

Makefile.config:
	./configure

mcp: $(OBJS)

$(OBJS): $(HEADERS)

install: mcp
	install -d -m 777 /data/rawdir
	install -m 755 mcp /usr/local/bin
#	install -m 666 etc/WMM.COF etc/*.aml etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 /data/etc


fifo:
	[ -e /tmp/SIPSS.FIFO ] || mkfifo -m 0777 /tmp/SIPSS.FIFO
