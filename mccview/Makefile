CC=gcc
LDFLAGS=-lgetdata -lncurses
CFLAGS=-Wall

mccview: mccview.c
	${CC} ${CFLAGS} -o $@ $< ${LDFLAGS}

install: mccview
	install -m 755 mccview /usr/local/bin
