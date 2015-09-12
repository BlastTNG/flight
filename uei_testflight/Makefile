UEIPAC_INCLUDES= -I "$(UEIPACROOT)/include"
UEIPAC_LIBS= -L "$(UEIPACROOT)/lib"

ifeq ($(DEBUG),1)
	CFLAGS += ${DEBUGFLAGS}
	LIBS+= -lUeiPalD -L "$(UEIPACROOT)/sdk/UeiPalLib"
else
	LIBS+= -lUeiPal -L "$(UEIPACROOT)/sdk/UeiPalLib"
endif

CC=powerpc-604-linux-gnu-gcc
CCFLAGS= -g -Wall -std=gnu99 $(UEIPAC_INCLUDES) -I include
LDFLAGS= -lpowerdna -lpthread $(LIBS) $(UEIPAC_LIBS)

target= uei_testflight
objects= uei_testflight.o minmea.o

all: $(target)

$(target): $(objects) 
	$(CC) $(LDFLAGS) $(objects) -o $@ 

%.o: src/%.c 
	$(CC) $(CCFLAGS) -c $< -o $@

clean:
	rm -f $(objects)
	rm -f $(target)
