EXECUTABLE = linklistcheck 
INCLUDEDIRS = -I ../common/linklist/ -I ../common/include/ -I../mcp/include/ \
			-I ../blast_config/include/
SOURCES = ../common/linklist/linklist.c ../common/CRC_func.c \
			../common/linklist/linklist_compress.c ../common/channels_tng.c \
			../common/blast.c ../common/PMurHash.c ../blast_config/tx_struct_tng.c \
			linklistcheck.c
CFLAGS = `pkg-config --cflags glib-2.0` $(INCLUDEDIRS) 
LIBS = -lssl -lcrypto -lpthread `pkg-config --libs glib-2.0`

OBJECTS = $(SOURCES:.c=.o)

all: $(EXECUTABLE) $(OBJECTS)

$(EXECUTABLE): $(OBJECTS)
	gcc $(SOURCES) $(CFLAGS) -o $@ $(LIBS)

.cpp.o:
	gcc $(CFLAGS) $< -o $@

