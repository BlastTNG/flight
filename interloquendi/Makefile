CC = gcc
CFLAGS = -O3 -Wall

OBJS = interloquendi.o quendi.o
HEADERS = quendi.h

all: interloquendi

interloquendi: $(OBJS)
	$(CC) interloquendi.o quendi.o -lwrap -o interloquendi

$(OBJS): $(HEADERS)

clean:
	rm -f *.o interloquendi
