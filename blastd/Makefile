ALL = rnc tdrsslogger dnc tea lnc
#
all: $(ALL)

rnc:rnc.c
	gcc -Wall rnc.c -o rnc

dnc:rnc.c
	gcc -Wall rnc.c -o dnc

tea:rnc.c
	gcc -Wall rnc.c -o tea

lnc:rnc.c
	gcc -Wall rnc.c -o lnc
#
tdrsslogger: tdrsslogger.c
	gcc -Wall tdrsslogger.c -o tdrsslogger
#

install:tdrsslogger tea dnc rnc lnc
	install -m 700 -o root -g root  tdrsslogger /usr/local/sbin
	install -m 700 -o root -g root  rnc /usr/local/sbin
	install -m 700 -o root -g root  dnc /usr/local/sbin
	install -m 700 -o root -g root  tea /usr/local/sbin
	install -m 700 -o root -g root  lnc /usr/local/sbin
	install -m 700 -o root -g root  doRestart /usr/local/sbin
	
clean:	
	rm  -f $(ALL)

purge:
	rm -f *~
