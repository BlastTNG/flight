include Makefile.config
CC = gcc 

LDFLAGS = -lm
INC  = -I.
CFLAGS = -Wall -O3 $(INC)

OBJS = blastcmd.o command_list.o

#----------------------------------------------------------------------------

all: blastcmd

blastcmd: $(OBJS)

command_list.o: command_list.c command_list.h isc_protocol.h

blastcmd.o: blastcmd.c command_list.h isc_protocol.h

Makefile.config:
	./configure

install_fifocmd:
	install -D -m 755 fifocmd $(EXEC_DIR)/fifocmd

install:
	install -D -m 755 blastcmd $(EXEC_DIR)/blastcmd
	[ -e $(DATA_DIR)/log.txt ] || install -D -m 666 log.txt $(DATA_DIR)/log.txt

clean:
	rm -f *~ *.o blastcmd Makefile.config || true
