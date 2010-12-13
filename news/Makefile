ALL = fox msnbc rush

CC=gcc -O2 -W -Wall
all: $(ALL)

fox:fox.o tx_struct.o compressstruct.o derived.o
	$(CC) -o fox fox.o tx_struct.o compressstruct.o derived.o -lm

msnbc:fox.o tx_struct.o compressstruct.o derived.o
	$(CC) -o msnbc fox.o tx_struct.o compressstruct.o derived.o -lm

rush:fox.o tx_struct.o compressstruct.o derived.o
	$(CC) -o rush fox.o tx_struct.o compressstruct.o derived.o -lm

install:fox msnbc rush
	install -m 755 fox /usr/local/bin
	install -m 755 msnbc /usr/local/bin
	install -m 755 rush /usr/local/bin

