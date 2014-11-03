CC = gcc
CFLAGS = -Wall -O3

OBJS = sss.o mpc_funcs.o net.o lut.o
HEADS = sss.h sss_struct.h net.h mpc_funcs.h mpc.h lut.h

all: sss client

sss: $(OBJS)
	gcc $(OBJS) -lm -lgsl -lgslcblas -o sss

sss.o: sss.c $(HEADS)
mpc_funcs.o: mpc_funcs.c $(HEADS)
net.o: net.c $(HEADS)

client: client.o
	gcc -o client client.o
client.o: client.c client.h sss_struct.h

clean:
	rm -f *.o sss *~ client

