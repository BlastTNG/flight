include Makefile.config
FL_LIBS = -lfl
CC = gcc -O3 -Wall $(BOLOTEST)
ifeq ($(DEBUG), 1)
  CC = gcc -O2 -DEBUG -Wall $(BOLOTEST)
endif

OBJS = mcp.o tx.o commands.o ephemeris/creader/Source/ephem_read.o starpos.o \
			 starfind.o pointing.o tx_struct.o dirfile.o geomag.o sched.o dgps.o \
			 isc.o

all: mcp

clean:
	rm -f mcp || true;
	rm -f *.o || true;
	rm -f Makefile.config || true;

Makefile.config:
	./configure

mcp: $(OBJS)
	$(CC) -L/usr/X11R6/lib -L. -o mcp $(OBJS) -lpthread -liticonfig_g \
	$(FL_LIBS) -lm

mcp.o: tx.h tx_struct.h command_struct.h pointing_struct.h ephem_read.h ephem_types.h mcp.c bbc.h starpos.h
	$(CC) -c mcp.c

tx.o: tx.h tx.c tx_struct.h command_struct.h bbc.h pointing_struct.h
	$(CC) -c tx.c

tx_struct.o: tx.h tx_struct.c tx_struct.h
	$(CC) -c tx_struct.c

dirfile.o: tx.h tx_struct.h tx_struct.c dirfile.c
	$(CC) -c dirfile.c

commands.o: command_struct.h commands.c commands.h pointing_struct.h tx_struct.h
	$(CC) -c commands.c

geomag.o: geomag.c
	$(CC) -c geomag.c

sched.o: sched.c command_struct.h pointing_struct.h 
	$(CC) -c sched.c

starpos.o: ephem_read.h ephem_types.h starpos.c command_struct.h 
	$(CC) -c starpos.c

starfind.o: starfind.c pointing_struct.h starpos.h ephem_read.h
	$(CC) -c starfind.c
#	  $(CC) \
#	  -c -L/usr/X11R6/lib -L. -o $@ $< \
#	  -liticonfig_g $(FL_LIBS) -lm -lc  starpos.o ephem_read.o

pointing.o: pointing_struct.h pointing.c
	  $(CC) -c pointing.c

dgps.o: dgps.c pointing_struct.h
	$(CC) -c dgps.c

isc.o: isc.c pointing_struct.h isc_protocol.h
	$(CC) -c isc.c

install: mcp
	install -m 755 mcp /usr/local/bin
	install -m 755 doSensors /usr/local/bin
