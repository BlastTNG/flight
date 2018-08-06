# First pass: normal build
PWD		:= $(shell pwd)
CC    := gcc -O2 -g

all: test_decom test_decom_packets simple_test

test_decom_packets: test_decom_packets.c
	$(CC) -o test_decom_packets test_decom_packets.c -lm

simple_test: simple_test.c
	$(CC) -o simple_test simple_test.c -lm

test_decom: test_decom.c
	$(CC) -o test_decom test_decom.c -lm

clean: 
	rm -f .decom* *.o test_decom test_decom_packets simple_test
