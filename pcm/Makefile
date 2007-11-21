include Makefile.config

CC = gcc
CPP = g++
LD = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ $(HAVE_ACS) $(HAVE_CRYO) $(USE_FIFO_CMD)
#CPPFLAGS = $(CPPFLAGS) $(NO_MOTORS)
LDFLAGS = -lpthread -lm -lstdc++
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

#common files
COM_HEADERS = bbc_pci.h blast.h channels.h derived.h mcp.h tx.h command_list.h \
	      command_struct.h slow_dl.h pointing_struct.h 

COM_OBJS = mcp.o blast.o channels.o derived.o framefile.o tx.o tx_struct.o \
	   commands.o command_list.o slow_dl.o

#ACS specific files
ifdef HAVE_ACS
ACS_HEADERS = drivecommunicator.h motorcommand.h camcommunicator.h camstruct.h \
                 reactcommand.h pivotcommand.h              

ACS_OBJS = drivecommunicator.o motorcommand.o motors.o pointing.o \
	   camcommunicator.o starcamera.o reactcommand.o pivotcommand.o
endif

#CRYO specific files
ifdef HAVE_CRYO
CRYO_HEADERS = 

CRYO_OBJS = 
endif

HEADERS = $(COM_HEADERS) $(ACS_HEADERS) $(CRYO_HEADERS)
OBJS = $(COM_OBJS) $(ACS_OBJS) $(CRYO_OBJS)

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
