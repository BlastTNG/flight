CC = gcc
CPP = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ $(BOLOTEST) -DDAS_CARDS=12
LDFLAGS = -lpthread -lm -lstdc++ -lgsl -lgslcblas
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

HEADERS = amccommand.h share/bbc_pci.h share/blast.h share/channels.h \
	  calibrate.h chrgctrl.h command_list.h command_struct.h \
	  share/compressstruct.h copleycommand.h hwpr.h share/crc.h \
	  share/derived.h share/ephem_read.h share/ephem_types.h ezstep.h \
	  share/fir.h isc_protocol.h share/lut.h mcp.h motordefs.h \
	  share/netcmd.h pointing_struct.h sbsc_protocol.h sbsccommunicator.h \
	  share/sip.h slow_dl.h sss_struct.h share/starpos.h tx.h
		

# These are needed by both bolotest and flight versions
OBJS = actuators.o share/blast.o share/channels.o command_list.o commands.o \
       das.o derived.o ezstep.o share/framefile.o hwpr.o share/lut.o mcp.o \
       share/sip.o tx.o tx_struct.o xystage.o

# These are only needed by the flight version
ifndef BOLOTEST
OBJS := $(OBJS) amccommand.o auxiliary.o chrgctrl.o share/compressfuncs.o \
  		share/compressionwriter.o compressstruct.o copleycommand.o \
		share/crc.o dgps.o share/ephem_read.o share/fir.o \
		share/geomag.o isc.o motors.o pointing.o share/radbox.o sbsc.o \
		sbsccommunicator.o sched.o slow_dl.o share/starpos.o sunsensor.o
endif

.PHONEY: fifo install all restart

all: mcp

clean:
	rm -f mcp || true;
	rm -f *.o || true;
	rm -f share/*.o || true;

mcp: $(OBJS)

$(OBJS): $(HEADERS)

install: mcp
	install -d -m 777 /data/rawdir
	install -m 755 mcp /usr/local/bin
	install -m 755 mcp_inf_loop /usr/local/bin
	install -d -m 777 /data/etc/blast
	install -m 666 etc/WMM.COF etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 etc/sched.library /data/etc/blast
#	install -m 666 etc/*.aml /data/etc/blast
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
	ssh north.blast 'install -m 666 /tmp/etc/* /data/etc/blast'
	ssh north.blast 'killall mcp'
else
ifeq ($(shell hostname),north)
	@echo "** I am north **"
	killall mcp
	scp /usr/local/bin/mcp south.blast:/tmp/mcp
	ssh south.blast 'install -d -m 666 /tmp/etc'
	scp etc/WMM.COF etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 etc/sched.library south.blast:/tmp/etc/
	ssh south.blast 'install -m 755 /tmp/mcp /usr/local/bin'
	ssh south.blast 'install -m 666 /tmp/etc/* /data/etc/blast'
	ssh south.blast 'killall mcp'
else
	@echo "******************** WARNING ******************************"
	@echo "** I am neither north nor south"
	@echo "** 'make restart' should not be run on other systems"
	@echo "** Just stop and start mcp as you would any other program"
	@echo "******************** WARNING ******************************"
endif
endif

