include ../Makefile.config
CC = g++
MOC = $(QT_DIR)/bin/moc
DEFINES = -DBLAST_CMD=\"$(BLAST_CMD)\" -DDATA_DIR=\"$(DATA_DIR)\" \
					-DCUR_DIR=\"$(CUR_DIR)\"
CPPFLAGS = -I. -I$(QT_DIR)/include -I../include -g $(DEFINES)
LIBS = -L../lib/ -L$(QT_DIR)/lib -lqt-mt -lRD ../lib/kstfile.o
LDFLAGS = $(LIBS)

OBJS = narsil.o command_list.o doubleentry.o moc_narsil.o moc_doubleentry.o ../lib/getdata.o

all: narsil

narsil: $(OBJS)
	$(CC) $(LDFLAGS) -o narsil $(OBJS) -lRD

narsil.o: narsil.cpp narsil.h command_list.h isc_protocol.h

command_list.o: command_list.c command_list.h isc_protocol.h

doubleentry.o: doubleentry.cpp doubleentry.h command_list.h isc_protocol.h

moc_doubleentry.o: moc_doubleentry.cpp doubleentry.h isc_protocol.h

moc_narsil.o: moc_narsil.cpp narsil.h command_list.h isc_protocol.h

moc_doubleentry.cpp: doubleentry.cpp doubleentry.h isc_protocol.h
	$(MOC) -o moc_doubleentry.cpp doubleentry.h

moc_narsil.cpp: narsil.cpp narsil.h command_list.h isc_protocol.h
	$(MOC) -o moc_narsil.cpp narsil.h

install:
	install -D -m 755 narsil $(EXEC_DIR)/narsil
	install -D -m 755 blastcmdatgaladriel $(EXEC_DIR)/blastcmdatgaladriel
	install -D -m 755 dummycmd $(EXEC_DIR)/dummycmd

clean:
	rm -f narsil moc_*.cpp *.o *~ || true
