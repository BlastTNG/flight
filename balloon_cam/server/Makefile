all: server
.PHONY: clean

server: main.o server.o
	g++ -o server main.o server.o

clean:
	rm server server.o main.o
