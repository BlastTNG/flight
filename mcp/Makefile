include Makefile.config

CC = gcc
CPP = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ $(BOLOTEST)
LDFLAGS = -lpthread -lm -lstdc++ -lgsl -lgslcblas
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

ifeq ($(USE_FIFO_CMD), 1)
	CPPFLAGS := $(CPPFLAGS) -DUSE_FIFO_CMD
endif

HEADERS = bbc_pci.h blast.h channels.h command_list.h command_struct.h hwpr.h \
					crc.h derived.h ephem_read.h ephem_types.h fir.h \
					isc_protocol.h lut.h mcp.h pointing_struct.h slow_dl.h \
					sss_struct.h starpos.h tx.h copleycommand.h motordefs.h \
                                        amccommand.h ezstep.h chrgctrl.h compressstruct.h \
					sbsc_protocol.h sbsccommunicator.h

# These are needed by both bolotest and flight versions
OBJS = mcp.o actuators.o blast.o channels.o command_list.o commands.o das.o hwpr.o \
			 derived.o framefile.o lut.o tx.o tx_struct.o xystage.o ezstep.o

# These are only needed by the flight version
ifndef BOLOTEST
OBJS := $(OBJS) auxiliary.o crc.o dgps.o ephem_read.o \
		    fftsg_h.o fir.o geomag.o isc.o motors.o pointing.o radbox.o sched.o \
		    slow_dl.o starpos.o sunsensor.o  compressionwriter.o copleycommand.o \
                    chrgctrl.o amccommand.o compressstruct.o sbsc.o sbsccommunicator.o

endif

.PHONEY: fifo install all debug

all::
ifeq ($(DEBUG), 1)
	@echo "******************** WARNING ******************************"
	@echo "Debugging symbols are being output to the binary"
	@echo "Be sure to run \`make clean' before compiling flight version"
	@echo "******************** WARNING ******************************"
endif

all:: mcp
ifeq ($(DEBUG), 1)
	@echo "******************** WARNING ********************"
	@echo "Debugging symbols are being output to the binary"
	@echo "******************** WARNING ********************"
endif

debug::
	${MAKE} DEBUG=1 all

debug:: all

clean:
	rm -f mcp || true;
	rm -f *.o || true;

Makefile.config:
	./configure

mcp: $(OBJS)

$(OBJS): $(HEADERS)

install: mcp
	install -d -m 777 /data/rawdir
	install -m 755 mcp /usr/local/bin
	install -m 755 mcp_inf_loop /usr/local/bin
	install -m 666 etc/WMM.COF etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 etc/sched.library /data/etc
#	install -m 666 etc/*.aml /data/etc
ifeq ($(BOLOTEST),-DBOLOTEST)
	$(MAKE) fifo
endif

fifo:
	[ -e /data/etc/SIPSS.FIFO ] || mkfifo -m 0777 /data/etc/SIPSS.FIFO

restart: install
ifeq ($(shell hostname),south)
	@echo "** I am south **"
	killall mcp
	scp /usr/local/bin/mcp north.blast:/tmp/mcp
	ssh north.blast 'install -d -m 666 /tmp/etc'
	scp etc/WMM.COF etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 etc/sched.library north.blast:/tmp/etc/
	ssh north.blast 'install -m 755 /tmp/mcp /usr/local/bin'
	ssh north.blast 'install -m 666 /tmp/etc/* /data/etc'
	ssh north.blast 'killall mcp'
else
ifeq ($(shell hostname),north)
	@echo "** I am north **"
	killall mcp
	scp /usr/local/bin/mcp south.blast:/tmp/mcp
	ssh south.blast 'install -d -m 666 /tmp/etc'
	scp etc/WMM.COF etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 etc/sched.library south.blast:/tmp/etc/
	ssh south.blast 'install -m 755 /tmp/mcp /usr/local/bin'
	ssh south.blast 'install -m 666 /tmp/etc/* /data/etc'
	ssh south.blast 'killall mcp'
else
	@echo "******************** WARNING ******************************"
	@echo "** I am neither north nor south"
	@echo "** 'make restart' should not be run on other systems"
	@echo "** Just stop and start mcp as you would any other program"
	@echo "******************** WARNING ******************************"
endif
endif

