include Makefile.config

CC = gcc
CPP = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ $(BOLOTEST)
LDFLAGS = -lpthread -lm -lstdc++
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

ifeq ($(USE_FIFO_CMD), 1)
	CPPFLAGS := $(CPPFLAGS) -DUSE_FIFO_CMD
endif

HEADERS = alice.h bbc_pci.h blast.h channels.h command_list.h command_struct.h \
					crc.h dataholder.h derived.h ephem_read.h ephem_types.h fir.h \
					isc_protocol.h lut.h mcp.h pointing_struct.h slow_dl.h \
					sss_struct.h starpos.h tdrss.h tx.h copleycommand.h motordefs.h \
                                        amccommand.h

# These are needed by both bolotest and flight versions
OBJS = mcp.o actuators.o blast.o channels.o command_list.o commands.o das.o \
			 derived.o framefile.o lut.o tx.o tx_struct.o xystage.o 

# These are only needed by the flight version
ifndef BOLOTEST
OBJS := $(OBJS) auxiliary.o crc.o dataholder.o dgps.o ephem_read.o \
		    fftsg_h.o fir.o geomag.o isc.o motors.o pointing.o radbox.o sched.o \
			  slow_dl.o starpos.o sunsensor.o tdrss.o copleycommand.o \
                                amccommand.o

endif

.PHONEY: fifo install all debug

all::
ifeq ($(DEBUG), 1)
	@echo "******************** WARNING ******************************"
	@echo "Debugging symbols are being output to the binary"
	@echo "Be sure to run \`make clean' before compiling flight version"
	@echo "******************** WARNING ******************************"
endif

all:: mcp
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
	install -m 666 etc/WMM.COF etc/*.aml etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 /data/etc
ifeq ($(BOLOTEST),-DBOLOTEST)
	$(MAKE) fifo
endif

fifo:
	[ -e /tmp/SIPSS.FIFO ] || mkfifo -m 0777 /tmp/SIPSS.FIFO
