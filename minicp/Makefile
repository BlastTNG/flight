CC = gcc
CPP = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ 
LDFLAGS = -lpthread -lm -lstdc++ 
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

HEADERS = bbc_pci.h blast.h channels.h derived.h mcp.h tx.h \
	  command_list.h command_struct.h

OBJS = blast.o channels.o derived.o framefile.o mcp.o tx.o \
       commands.o command_list.o tx_struct.o az-el.o

.PHONEY: fifo install all

all: minicp

clean:
	rm -f minicp || true;
	rm -f *.o || true;

minicp: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o minicp

$(OBJS): $(HEADERS)

install: minicp
	@echo "No installation is implemented right now."

fifo:
	[ -e /data/etc/SIPSS.FIFO ] || mkfifo -m 0777 /data/etc/SIPSS.FIFO
