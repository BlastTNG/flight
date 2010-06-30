ALL = fox

CC=gcc -O2 -W -Wall
all: $(ALL)

fox:fox.o tx_struct.o compressstruct.o derived.o
	$(CC) -o fox fox.o tx_struct.o compressstruct.o derived.o -lm

install:fox
	install -m 700 -o root -g root fox /usr/local/sbin

