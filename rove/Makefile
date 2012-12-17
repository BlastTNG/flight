CC=gcc -O2 -W -Wall

rove: rove.o
	$(CC) -o rove rove.o -lm

install: rove
	install -m 755 rove /usr/local/sbin
