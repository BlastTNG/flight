ALL = fox

all: $(ALL)

fox:fox.o
	gcc -Wall -o fox fox.o -lm

install:fox
	install -m 700 -o root -g root fox /usr/local/sbin

