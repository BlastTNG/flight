include ../Makefile.config
MOC = $(QT_DIR)/bin/moc
FLAGS = -I. -I$(QT_DIR)/include -I../include -g
DEFINES = -DBLAST_CMD=\"$(BLAST_CMD)\" -DDATA_DIR=\"$(DATA_DIR)\" \
					-DCUR_DIR=\"$(CUR_DIR)\"
CC = gcc $(DEFINES)
CPP = g++ $(DEFINES)
LIBS = -L../lib/ -L$(QT_DIR)/lib -lqt-mt -lkdecore -lRD

all: narsil

narsil: command_list.o narsil.o moc_narsil.o doubleentry.o moc_doubleentry.o \
	command_list.h
	$(CPP) ../lib/kstfile.o ../lib/getdata.o narsil.o command_list.o \
	doubleentry.o moc_narsil.o moc_doubleentry.o $(LIBS)\
	-o narsil

moc_narsil.o: moc_narsil.cpp narsil.h command_list.h isc_protocol.h
	$(CPP) $(FLAGS) -c moc_narsil.cpp

command_list.o: command_list.c command_list.h isc_protocol.h
	$(CC) $(FLAGS) -c command_list.c

narsil.o: narsil.cpp narsil.h command_list.h isc_protocol.h
	$(CPP) $(FLAGS) -c narsil.cpp

doubleentry.o: doubleentry.cpp doubleentry.h command_list.h isc_protocol.h
	$(CPP) $(FLAGS) -c doubleentry.cpp

moc_doubleentry.cpp: doubleentry.cpp doubleentry.h isc_protocol.h
	$(MOC) -o moc_doubleentry.cpp doubleentry.h

moc_doubleentry.o: moc_doubleentry.cpp doubleentry.h isc_protocol.h
	$(CPP) $(FLAGS) -c moc_doubleentry.cpp

moc_narsil.cpp: narsil.cpp narsil.h command_list.h isc_protocol.h
	$(MOC) -o moc_narsil.cpp narsil.h

install:
	install -D -m 755 narsil $(EXEC_DIR)/narsil
	install -D -m 755 blastcmdatgaladriel $(EXEC_DIR)/blastcmdatgaladriel
	install -D -m 755 dummycmd $(EXEC_DIR)/dummycmd

clean:
	rm narsil moc_narsil.cpp *.o *~ moc_narsil.cpp || true
