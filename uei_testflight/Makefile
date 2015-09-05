UEIPAC_INCLUDES= -I "$(UEIPACROOT)/include"
UEIPAC_LIBS= -L "$(UEIPACROOT)/lib"

CC=powerpc-604-linux-gnu-gcc
CCFLAGS= -g -Wall -std=gnu99 $(UEIPAC_INCLUDES)
LDFLAGS= -lpowerdna -lpthread $(UEIPAC_LIBS)

target= uei_testflight
objects= uei_testflight.o

all: $(target)

$(target): $(objects) 
	$(CC) $(LDFLAGS) $(objects) -o $@ 

%.o: src/%.c 
	$(CC) $(CCFLAGS) -c $< -o $@

clean:
	rm -f $(objects)
	rm -f $(target)
