CC = gcc
CFLAGS = -O3 -Wall
LDFLAGS = -lpthread

all: defile

debug:
	${MAKE} CFLAGS="$(CFLAGS) -g" LDFLAGS="$(LDFLAGS) -g" all

defile: defile.o reader.o tx_struct.o writer.o

defile.o: defile.c defile.h tx_struct.h

reader.o: reader.c tx_struct.h defile.h

tx_struct.o: tx_struct.c tx_struct.h

writer.o: writer.c tx_struct.h defile.h

clean:
	rm -f *.o defile

install: defile
	install -m 755 defile /usr/local/bin
	install -m 644 defile.1 /usr/local/man/man1
