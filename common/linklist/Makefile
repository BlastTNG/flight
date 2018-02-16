EXECUTABLE = pilot_gnd
INCLUDEDIRS = -I ../common/include/ -I ../blast_config/include/ -I ../mcp/include/
SOURCES = linklist.c CRC.c linklist_compress.c ../common/channels_tng.c \
    ../common/blast.c ../common/PMurHash.c ../blast_config/tx_struct_tng.c \
    bitserver.c FIFO.c pilot_gnd_test.c
CFLAGS = `pkg-config --cflags glib-2.0` $(INCLUDEDIRS) 
LIBS = -lssl -lcrypto -lpthread `pkg-config --libs glib-2.0`

OBJECTS = $(SOURCES:.c=.o)

all: $(EXECUTABLE) $(OBJECTS)

$(EXECUTABLE): $(OBJECTS)
	gcc $(SOURCES) $(CFLAGS) -o $@ $(LIBS)

.cpp.o:
	gcc $(CFLAGS) $< -o $@

.PHONY: linklist_test

linklist_test: 
	gcc -D_TESTING $(SOURCES) $(INCLUDEDIRS) $(CFLAGS) -o $@ $(LIBS)

