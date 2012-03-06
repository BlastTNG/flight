CC = gcc
CXX = g++
CFLAGS = -g -O3 -Wall
CPPFLAGS = -D__MCP__ $(BOLOTEST) -DDAS_CARDS=12
LDFLAGS = -lpthread -lm -lstdc++ -lgsl -lgslcblas
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

########################################################
# External repository dependencies
#

BLAST_ONLY=1

########################################################
# Inputs
#

EXPT_HEADERS = calibrate.h command_list.h isc_protocol.h
SHARED_HEADERS = bbc_pci.h blast.h channels.h compressstruct.h crc.h derived.h \
								 ephem_read.h ephem_types.h fir.h lut.h netcmd.h sip.h starpos.h
LOCAL_HEADERS = amccommand.h chrgctrl.h command_struct.h copleycommand.h \
								hwpr.h ezstep.h mcp.h motordefs.h pointing_struct.h \
								sbsc_protocol.h sbsccommunicator.h slow_dl.h sss_struct.h tx.h

# These are needed by both bolotest and flight versions
EXPT_OBJS = command_list.o derived.o tx_struct.o
SHARED_OBJS = blast.o channels.o framefile.o lut.o sip.o
LOCAL_OBJS = actuators.o commands.o das.o ezstep.o hwpr.o mcp.o tx.o xystage.o

# These are only needed by the flight version
ifndef BOLOTEST
EXPT_OBJS += compressstruct.o
SHARED_OBJS += compressfuncs.o compressionwriter.o crc.o ephem_read.o fir.o \
							 geomag.o radbox.o starpos.o
LOCAL_OBJS += amccommand.o auxiliary.o chrgctrl.o copleycommand.o dgps.o isc.o \
							motors.o pointing.o sbsc.o sbsccommunicator.o sched.o slow_dl.o \
							sunsensor.o
endif

ETC_DATA = WMM.COF *.sch *.lut *.txt ephem.2000 sched.library

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Build Stuff
#

.PHONY: fifo install all restart

all:: mcp

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

clean:
	rm -f mcp
	rm -f *.o

mcp: $(OBJS)


########################################################
# Install Stuff
#

install: mcp
	install -d -m 777 /data/rawdir
	install -m 755 mcp /usr/local/bin
	install -m 755 mcp_inf_loop /usr/local/bin
	install -d -m 777 /data/etc/blast
	install -m 666  $(ETC_DATA) /data/etc/blast
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

