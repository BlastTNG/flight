include ../Makefile.config
MOC = $(QT_DIR)/bin/moc
CMDS = ./commands.h
FLAGS = -I. -I$(QT_DIR)/include -I../include -g
DEFINES = -DBLAST_CMD=\"$(BLAST_CMD)\" -DDATA_DIR=\"$(DATA_DIR)\" \
					-DCUR_DIR=\"$(CUR_DIR)\"
CPP = g++ $(DEFINES)
LIBS = -L../lib/ -L$(QT_DIR)/lib -lqt-mt -lRD

all: narsil

narsil: narsil.o moc_narsil.o doubleentry.o moc_doubleentry.o $(CMDS) 
	$(CPP) ../lib/kstfile.o ../lib/getdata.o narsil.o \
	doubleentry.o moc_narsil.o moc_doubleentry.o $(LIBS)\
	-o narsil

moc_narsil.o: moc_narsil.cpp narsil.h commands.h isc_protocol.h
	$(CPP) $(FLAGS) -c moc_narsil.cpp

narsil.o: narsil.cpp narsil.h commands.h isc_protocol.h
	$(CPP) $(FLAGS) -c narsil.cpp

doubleentry.o: doubleentry.cpp doubleentry.h commands.h isc_protocol.h
	$(CPP) $(FLAGS) -c doubleentry.cpp

moc_doubleentry.cpp: doubleentry.cpp doubleentry.h isc_protocol.h
	$(MOC) -o moc_doubleentry.cpp doubleentry.h

moc_doubleentry.o: moc_doubleentry.cpp doubleentry.h isc_protocol.h
	$(CPP) $(FLAGS) -c moc_doubleentry.cpp

moc_narsil.cpp: narsil.cpp narsil.h commands.h isc_protocol.h
	$(MOC) -o moc_narsil.cpp narsil.h

install:
	install -D -m 755 narsil $(EXEC_DIR)/narsil
	install -D -m 755 blastcmdatgaladriel $(EXEC_DIR)/blastcmdatgaladriel
	install -D -m 755 dummycmd $(EXEC_DIR)/dummycmd

clean:
	rm narsil moc_narsil.cpp *.o *~ moc_narsil.cpp || true
