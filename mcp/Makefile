include Makefile.config

CC = gcc
CPP = g++
CPPFLAGS = -O3 -Wall -D__MCP__ $(BOLOTEST)
LDFLAGS = -lpthread -lm -lstdc++
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g
	LDFLAGS := $(LDFLAGS) -g
endif

ifeq ($(USE_FIFO_CMD), 1)
	CPPFLAGS := $(CPPFLAGS) -DUSE_FIFO_CMD
endif

HEADERS = alice.h bbc_pci.h command_list.h crc.h dataholder.h ephem_read.h \
		  ephem_types.h isc_protocol.h lut.h mcp.h pointing_struct.h slow_dl.h \
		  small.h sslutNA.h starpos.h tx.h channels.h small_c.h command_struct.h \
                  ss_struct.h 

OBJS = mcp.o auxiliary.o channels.o command_list.o commands.o crc.o das.o \
	   dataholder.o dgps.o fftsg_h.o framefile.o ephem_read.o geomag.o isc.o \
	   lut.o motors.o pointing.o sched.o slow_dl.o small.o sslutNA.o starpos.o \
	   sunsensor.o tx.o tx_struct.o

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
	cd etc && $(MAKE) clean

Makefile.config:
	./configure

mcp: $(OBJS)

$(OBJS): $(HEADERS)

install: mcp
	install -d -m 777 /data/rawdir
	install -m 755 mcp /usr/local/bin
ifeq ($(BOLOTEST),-DBOLOTEST)
	$(MAKE) fifo
endif
	cd etc && $(MAKE) install

fifo:
	[ -e /tmp/SIPSS.FIFO ] || mkfifo -m 0777 /tmp/SIPSS.FIFO
