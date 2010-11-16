CPP = g++ -O3 
LIBS = -lX11 -lXpm -lblas -lblast -lmatpack -lcfitsio -lgetdata
SLIBS = -lX11 -lXpm -lblas -lblast  -lcfitsio
INC_DIR = /usr/local/include/matpack
LIB_DIR = /usr/X11R6/lib

all:naivepol
naivepol:naivepol.cc
	$(CPP) -Wall -I$(INC_DIR) $@.cc  -o $@ -L$(LIB_DIR) $(LIBS)

naivemapkir:naivemapkir.cc
	$(CPP) -Wall -I$(INC_DIR) $@.cc  -o $@ -L$(LIB_DIR) $(LIBS)

mapoffs:mapoffs.cc map.o
	$(CPP) -Wall -I$(INC_DIR) $@.cc map.o -o $@ -L$(LIB_DIR) $(LIBS) -lblast


install: 
	install -D -m 755 naivepol /usr/local/bin/naivepol

clean:
	rm -f naivepol

purge:
	rm -f *~
