CPP = g++
LIBS = -lgetdata++ #-lX11 -lXpm -lblas -lblast -lmatpack -lcfitsio 
CPPFLAGS = -O3 -Wall -g


all:  lockinmap

lockinmap: lockinmap.o
	$(CPP) $(CPPFLAGS) lockinmap.o $(LIBS) -o lockinmap

lockinmap.o : iirfilter.h

clean:
	rm -f lockinmap *.o
	
purge: clean
	rm -f *~
	
