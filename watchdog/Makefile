all: watchdog

wdog: watchdog.c
	gcc -O3 -o watchdog watchdog.c

install: watchdog
	cp watchdog /usr/local/bin/watchdog
