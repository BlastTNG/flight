ALL = fox msnbc rush

CC=gcc
CFLAGS = -O2 -W -Wall

OBJS = fox.o tx_struct.o compressstruct.o derived.o share/compressfuncs.o

HEADERS = calibrate.h share/compressstruct.h share/channels.h share/derived.h

.PHONEY: all install clean

all: $(ALL)

$(OBJS): $(HEADERS)

fox: $(OBJS)
	$(CC) $(CFLAGS) -lm -o fox $(OBJS)

msnbc: $(OBJS)
	$(CC) $(CFLAGS) -lm -o msnbc $(OBJS)

rush: $(OBJS)
	$(CC) $(CFLAGS) -lm -o rush $(OBJS)

clean:
	rm -f fox msnbc rush || true;
	rm -f *.o || true;

install: all
	install -m 755 fox /usr/local/bin
	install -m 755 msnbc /usr/local/bin
	install -m 755 rush /usr/local/bin

