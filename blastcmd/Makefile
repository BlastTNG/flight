include Makefile.config
CC = gcc -g

LIBS = -lm
INC  = -I.
FLAGS = -Wall -O $(INC)

OBJS = blastcmd.o

#----------------------------------------------------------------------------

all: blastcmd

blastcmd: $(OBJS)
	$(CC) $(LIBS) $(OBJS) -o blastcmd

blastcmd.o: blastcmd.c commands.h
	$(CC) $(FLAGS) -DLIB_DIR=\"$(DATA_DIR)\" -c blastcmd.c -o blastcmd.o

install:
	install -D -m 755 blastcmd $(EXEC_DIR)/blastcmd
	[ -e $(DATA_DIR)/log.txt ] || install -D -m 666 log.txt $(DATA_DIR)/log.txt

clean:
	rm -f *~ *.o blastcmd Makefile.config || true
