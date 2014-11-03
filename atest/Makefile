# Compiler
CC = gcc

# Include directories for objects
IDIR0 = ../mcp
IDIR1 = ../common
IDIR2 = ../blast_config
IDIR3 = ../atest
IDIR4 = ../pcm
IDIR5 = /usr/include/glib-2.0 # for glib.h
IDIR6 = /usr/lib/x86_64-linux-gnu/glib-2.0/include # for glibconfig.h
CFLAGS = -I$(IDIR0) -I$(IDIR1) -I$(IDIR2) -I$(IDIR3) \
		-I$(IDIR4) -I$(IDIR5) -I$(IDIR6) -std=c99

# Libraries
LIBS = -L/usr/include/glib-2.0 -lglib-2.0
	
# This turns on the #defines needed for wrappers in compressionwriter.c
DEPS = ../atest/test.h

# These need full path extension
OBJS = test.o \
	../blast_config/compressstruct.o \
	../blast_config/tx_struct.o \
	../blast_config/tx_struct_tng.o \
	../blast_config/derived.o \
	../common/compressionwriter.o \
	../common/blast.o \
	../common/channels.o \
	../common/channels_tng.o \
	../common/compressfuncs.o \
	../common/mputs.o \
	../common/PMurHash.o \
	../mcp/commands.o
ifdef DEBUG
	CFLAGS += -DDEBUG -O0 -ggdb
else
	CFLAGS += -O2
endif


# Note: % = wildcard, $@ = target, $< = prerequisite

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $< $(LIBS) -include $(DEPS)

default: test

test: $(OBJS)
	$(CC) $(CFLAGS) -o test $(OBJS) $(LIBS)

clean:
	rm -f *.o test 

