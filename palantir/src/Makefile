include ../Makefile.config
MOC = $(QT_DIR)/bin/moc
LIB_DIR = -DLIB_DIR=\"$(DATA_DIR)/\"
INCLUDE = -I../include -I$(QT_DIR)/include
CPP = g++ $(INCLUDE)
CC = gcc
EXTRA = -L$(QT_DIR)/lib -L../lib -lqt-mt -lRD

all: palantir

palantir: moc_palantir.o adamdom.o palantir.o
	$(CPP) ../lib/kstfile.o moc_palantir.o palantir.o ../lib/getdata.o adamdom.o $(EXTRA) -o palantir

palantir.o: palantir.cpp palantir.h
	$(CPP) $(PC_SPEAKER) $(LIB_DIR) -c palantir.cpp

moc_palantir.o: moc_palantir.cpp
	$(CPP) -c moc_palantir.cpp

moc_palantir.cpp: palantir.cpp palantir.h
	$(MOC) -o moc_palantir.cpp palantir.h

adamdom.o: adamdom.cpp adamdom.h
	$(CPP) -c adamdom.cpp

install:
	install -D -m 755 palantir $(EXEC_DIR)/palantir

clean:
	rm palantir moc_palantir.cpp *.o || true
