include Makefile.config
FL_LIBS = -lfl
CC = gcc -O3 -Wall
ifeq ($(DEBUG), 1)
  CC = gcc -O2 -DEBUG -Wall
endif

all: mcp-bin

mcp: mcp-bin

clean:
	rm -f mcp-bin || true;
	rm -f *.o || true;
	rm -f Makefile.config || true;

Makefile.config:
	./configure

mcp-bin: mcp.o tx.o tx.h tx_struct.h commands.o command_struct.h ephemeris/creader/Source/ephem_read.o starpos.o starfind.o pointing_struct.h pointing.o commands.h tx_struct.o dirfile.o geomag.o
	$(CC) -L/usr/X11R6/lib -L. -o mcp-bin mcp.o tx.o commands.o ephemeris/creader/Source/ephem_read.o starpos.o starfind.o \
	 dirfile.o geomag.o pointing.o tx_struct.o \
        -lpthread -liticonfig_g $(FL_LIBS) -lm

mcp.o: tx.h tx_struct.h command_struct.h pointing_struct.h ephem_read.h ephem_types.h mcp.c
	$(CC) $(BOLOTEST) -c mcp.c

tx.o: tx.h tx.c tx_struct.h
	$(CC) -c tx.c

tx_struct.o: tx.h tx_struct.c tx_struct.h
	$(CC) -c tx_struct.c

dirfile.o: tx.h tx_struct.h dirfile.c
	$(CC) -c dirfile.c

commands.o: command_struct.h commands.c commands.h
	$(CC) -c commands.c

geomag.o: geomag.c
	$(CC) -c geomag.c

starpos.o: ephem_read.h ephem_types.h starpos.c
	$(CC) -c starpos.c

starfind.o: starfind.c pointing_struct.h starpos.h
	$(CC) -c starfind.c
#	  $(CC) \
#	  -c -L/usr/X11R6/lib -L. -o $@ $< \
#	  -liticonfig_g $(FL_LIBS) -lm -lc  starpos.o ephem_read.o

pointing.o: pointing_struct.h pointing.c
	  $(CC) -c pointing.c

