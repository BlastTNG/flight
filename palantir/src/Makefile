include ../Makefile.config
MOC = $(QT_DIR)/bin/moc
CPP = g++ -I$(QT_DIR)/include
CC = gcc
LIB_DIR = -DLIB_DIR=\"$(DATA_DIR)/\"
EXTRA = moc_palantir.cpp -L$(QT_DIR)/lib -I../include -L../lib -lqt-mt -lRD -lncurses $(LIB_DIR)

all: palantir

palantir: moc_palantir.cpp palantir.cpp adamdom.o
	$(CPP) $(PC_SPEAKER) ../lib/kstfile.o palantir.cpp ../lib/getdata.o adamdom.o $(EXTRA) -o palantir

moc_palantir.cpp: palantir.cpp palantir.h
	$(MOC) -o moc_palantir.cpp palantir.h

adamdom.o: adamdom.cpp adamdom.h
	$(CPP) -c adamdom.cpp

install:
	install -D -m 755 palantir $(EXEC_DIR)/palantir

clean:
	rm palantir moc_palantir.cpp *.o || true
