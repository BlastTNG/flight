include Makefile.config

CC = gcc
CFLAGS = -O3 -Wall -D__MCP__ $(BOLOTEST)
LDFLAGS = -lpthread -lm
ifeq ($(DEBUG), 1)
	CFLAGS := $(CFLAGS) -g
	LDFLAGS := $(LDFLAGS) -g
endif

ifeq ($(USE_FIFO_CMD), 1)
	CFLAGS := $(CFLAGS) -DUSE_FIFO_CMD
endif

HEADERS = bbc_pci.h commands.h ephem_read.h ephem_types.h isc_protocol.h lut.h \
					mcp.h pointing_struct.h sort.h sslutNA.h starpos.h tx.h tx_struct.h

OBJS = auxiliary.o channels.o commands.o das.o dgps.o framefile.o \
			 ephem_read.o geomag.o isc.o lut.o mcp.o \
			 motors.o pointing.o sched.o sslutNA.o starpos.o tx.o \
			 tx_struct.o

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

tx_test: tx_struct.c tx_test.c
	gcc -O3 -Wall -o tx_test tx_struct.c tx_test.c

tx_test.o:
	

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
	cd etc && $(MAKE) install
