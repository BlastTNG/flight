include ../Makefile.config
MOC = $(QT_DIR)/bin/moc
LIB_DIR = -DLIB_DIR=\"$(DATA_DIR)/\"
INCLUDE = -I../include -I$(QT_DIR)/include
CPP = g++
CC = gcc
CPPFLAGS = -g $(INCLUDE) $(LIB_DIR) -DQT_THREAD_SUPPORT
LDFLAGS = -g $(INCLUDE) -L$(QT_DIR)/lib -L../lib -lqt-mt -lRD

all: palantir

palantir: moc_palantir.o adamdom.o palantir.o
	$(CPP) $(LDFLAGS) ../lib/kstfile.o moc_palantir.o palantir.o ../lib/getdata.o adamdom.o -lRD -o palantir

palantir.o: palantir.cpp palantir.h

moc_palantir.o: moc_palantir.cpp

moc_palantir.cpp: palantir.cpp palantir.h
	$(MOC) -o moc_palantir.cpp palantir.h

adamdom.o: adamdom.cpp adamdom.h

install:
	install -D -m 755 palantir $(EXEC_DIR)/palantir

clean:
	rm -f palantir moc_palantir.cpp *.o || true
