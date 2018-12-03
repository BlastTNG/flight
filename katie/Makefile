all: katie

katie: katie.c
	gcc -Wall -g -o katie katie.c

install: katie
	install -m 755 katie /usr/local/bin

clean:
	rm -f katie

