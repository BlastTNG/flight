include Makefile.config
#FL_LIBS = -lfl
FL_LIBS = 
CC = gcc -O3 -Wall $(BOLOTEST)
ifeq ($(DEBUG), 1)
  CC = gcc -O2 -DEBUG -Wall $(BOLOTEST)
endif

OBJS = auxiliary.o commands.o das.o dgps.o framefile.o \
      ephemeris/creader/Source/ephem_read.o geomag.o isc.o lut.o mcp.o \
      motors.o pointing.o sched.o sslutNA.o starpos.o tx.o \
      tx_struct.o

all: mcp

clean:
	rm -f mcp || true;
	rm -f *.o || true;
	rm -f Makefile.config || true;
	cd etc && $(MAKE) clean

Makefile.config:
	./configure

#mcp: $(OBJS)
#	$(CC) -L/usr/X11R6/lib -L. -o mcp $(OBJS) -lpthread -liticonfig_g \
#	$(FL_LIBS) -lm

mcp: $(OBJS)
	$(CC) -L/usr/X11R6/lib -L. -o mcp $(OBJS) -lpthread \
	$(FL_LIBS) -lm

auxiliary.o: auxiliary.c tx_struct.h tx.h command_struct.h isc_protocol.h \
	pointing_struct.h

commands.o: commands.c command_struct.h commands.h pointing_struct.h \
	tx_struct.h isc_protocol.h mcp.h

das.o: das.c tx_struct.h command_struct.h isc_protocol.h tx.h

dgps.o: dgps.c pointing_struct.h isc_protocol.h mcp.h

framefile.o: framefile.c tx_struct.h mcp.h

geomag.o: geomag.c mcp.h

isc.o: isc.c pointing_struct.h isc_protocol.h command_struct.h tx_struct.h mcp.h

lut.o: lut.c lut.h mcp.h

mcp.o: mcp.c bbc.h tx_struct.h tx.h pointing_struct.h command_struct.h \
	starpos.h isc_protocol.h mcp.h

motors.o: motors.c pointing_struct.h isc_protocol.h command_struct.h tx.h \
	tx_struct.h mcp.h

pointing.o: pointing.c pointing_struct.h isc_protocol.h command_struct.h \
		tx_struct.h lut.h sslutNA.h

sched.o: sched.c command_struct.h pointing_struct.h isc_protocol.h tx_struct.h \
	mcp.h

starpos.o: starpos.c ephem_read.h ephem_types.h command_struct.h tx_struct.h \
	isc_protocol.h mcp.h

tx.o: tx.c tx.h tx_struct.h command_struct.h pointing_struct.h isc_protocol.h \
	mcp.h

tx_struct.o: tx_struct.c tx_struct.h mcp.h

sslutNA.o: sslutNA.h sslutNA.c mcp.h

install: mcp
	install -d -m 777 /data/rawdir
	install -m 755 mcp /usr/local/bin
	install -m 755 doSensors /usr/local/bin
	install -m 755 doMcp /usr/local/bin
	cd etc && $(MAKE) install
