CC = gcc
CFLAGS = -O3 -Wall -D__DECOMD__
LDFLAGS = -lpthread

all: decomd

decomd: decomd.o framefile.o channels.o tx_struct.o

clean:
	rm -f decom
	rm -f *.o
