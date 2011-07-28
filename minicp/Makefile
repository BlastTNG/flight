CC = gcc
CPP = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ 
LDFLAGS = -lpthread -lm -lstdc++ 
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

HEADERS = share/bbc_pci.h share/blast.h share/channels.h share/derived.h \
	  mcp.h tx.h command_list.h command_struct.h share/netcmd.h

OBJS = az-el.o share/blast.o share/channels.o derived.o share/framefile.o hk.o \
       mcp.o tx.o commands.o command_list.o tx_struct.o
.PHONEY: fifo install all

all: minicp

clean:
	rm -f minicp || true;
	rm -f *.o || true;
	rm -f share/*.o || true;

minicp: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o minicp

$(OBJS): $(HEADERS)

install: minicp
	@echo "No installation is implemented right now."

fifo:
	[ -e /data/etc/SIPSS.FIFO ] || mkfifo -m 0777 /data/etc/SIPSS.FIFO
