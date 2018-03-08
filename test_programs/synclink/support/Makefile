 #DEBUG=1
 
 CFLAGS := -Wall -Wstrict-prototypes -fomit-frame-pointer
 ifdef DEBUG
 CFLAGS += -g -O
 else
 CFLAGS += -O2
 endif
 
 all : test_biphase
 
test_biphase: test_biphase_noblast.c ./synclink.h
		 gcc $(CFLAGS) -I ../ -o test_biphase test_biphase_noblast.c
 
 CLEAN_TARGETS = *~ core test_biphase *.o
 clean:
		 @for f in $(CLEAN_TARGETS) ; do \
			     find . -name "$$f" | xargs rm -f ; \
		 done;
 

