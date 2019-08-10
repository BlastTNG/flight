EXECUTABLE = linklistcheck 
LINKLIST_DIR = ../external_libs/linklist
INCLUDEDIRS = -I ../liblinklist/ -I ../common/include/ -I../mcp/include/ \
			-I ../blast_config/include/ -I $(LINKLIST_DIR)/
SOURCES = ../common/CRC_func.c ../common/channels_tng.c \
			../common/blast.c ../common/PMurHash.c ../blast_config/tx_struct_tng.c \
			$(LINKLIST_DIR)/linklist.c $(LINKLIST_DIR)/linklist_compress.c \
			$(LINKLIST_DIR)/linklist_writer.c $(LINKLIST_DIR)/linklist_connect.c \
			linklistcheck.c

CFLAGS = `pkg-config --cflags glib-2.0` $(INCLUDEDIRS) 
LIBS = -lssl -lcrypto -lpthread `pkg-config --libs glib-2.0`

OBJECTS = $(SOURCES:.c=.o)

all: $(EXECUTABLE) $(OBJECTS)

$(EXECUTABLE): $(OBJECTS)
	gcc $(SOURCES) $(CFLAGS) -o $@ $(LIBS)

.cpp.o:
	gcc $(CFLAGS) $< -o $@

.PHONY: clean

clean: 
	rm -f *.o $(EXECUTABLE)
