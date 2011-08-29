CC = gcc
CPP = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ $(BOLOTEST)
LDFLAGS = -lpthread -lm -lstdc++ -lgsl -lgslcblas

#enable to setup a test run on non-flight systems (eg with simbbc)
#use test-run-enable target to enable
#to use 32-bit compilation on 64-bit systmems requires extra packages:
#  gcc-multilib g++-multilib libc6-dev-i386
include Makefile.testing
ifeq ($(TEST_RUN), 1)
	CPPFLAGS := $(CPPFLAGS) -m32 -DTEST_RUN
	LDFLAGS := $(LDFLAGS) -m32
endif

HEADERS = amccommand.h share/bbc_pci.h share/blast.h share/channels.h \
	  calibrate.h chrgctrl.h command_list.h command_struct.h \
	  share/compressstruct.h hwpr.h share/crc.h \
	  share/derived.h share/ephem_read.h share/ephem_types.h ezstep.h \
	  share/fir.h share/lut.h mcp.h motordefs.h \
	  share/netcmd.h pointing_struct.h sbsc_protocol.h sbsccommunicator.h \
	  slow_dl.h share/starpos.h tx.h
		

# These are needed by both bolotest and flight versions
OBJS = actuators.o share/blast.o share/channels.o command_list.o commands.o \
       derived.o ezstep.o share/framefile.o hk.o hwpr.o share/lut.o mcp.o \
       tx.o tx_struct.o xystage.o

# These are only needed by the flight version
ifndef BOLOTEST
OBJS := $(OBJS) amccommand.o auxiliary.o chrgctrl.o share/compressfuncs.o \
  		share/compressionwriter.o compressstruct.o \
		share/crc.o dgps.o share/ephem_read.o share/fir.o \
		share/geomag.o motors.o pointing.o share/radbox.o sbsc.o \
		sbsccommunicator.o sched.o slow_dl.o share/starpos.o 
endif

ETC_DATA = etc/WMM.COF etc/ephem.2000 etc/sched.library \
	   etc/*.sch etc/*.lut

.PHONEY: fifo install all restart test-run-enable test-run-disable

all: pcm

clean:
	rm -f pcm || true;
	rm -f *.o || true;
	rm -f share/*.o || true;

Makefile.testing:
	touch Makefile.testing

test-run-enable: clean
	echo "TEST_RUN = 1" > Makefile.testing

test-run-disable: clean
	echo "TEST_RUN = 0" > Makefile.testing

pcm: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o pcm

$(OBJS): $(HEADERS)

install: pcm
	install -d -m 777 /data/rawdir
	install -m 755 pcm /usr/local/bin
	install -m 755 pcm_inf_loop /usr/local/bin
	install -d -m 777 /data/etc/spider
	install -m 666  $(ETC_DATA) /data/etc/spider
ifeq ($(BOLOTEST),-DBOLOTEST)
	$(MAKE) fifo
endif

fifo:
	[ -e /data/etc/SIPSS.FIFO ] || mkfifo -m 0777 /data/etc/SIPSS.FIFO

restart: install
ifeq ($(shell hostname),bitsy)
	@echo "** I am bitsy **"
	killall pcm
	scp /usr/local/bin/pcm itsy.spider:/tmp/pcm
	ssh itsy.spider 'install -d -m 666 /tmp/etc'
	scp $(ETC_DATA) itsy.spider:/tmp/etc/
	ssh itsy.spider 'install -m 755 /tmp/pcm /usr/local/bin'
	ssh itsy.spider 'install -m 666 /tmp/etc/* /data/etc/spider'
	ssh itsy.spider 'killall pcm'
else
ifeq ($(shell hostname),itsy)
	@echo "** I am itsy **"
	killall pcm
	scp /usr/local/bin/pcm bitsy.spider:/tmp/pcm
	ssh bitsy.spider 'install -d -m 666 /tmp/etc'
	scp $(ETC_DATA) bitsy.spider:/tmp/etc/
	ssh bitsy.spider 'install -m 755 /tmp/pcm /usr/local/bin'
	ssh bitsy.spider 'install -m 666 /tmp/etc/* /data/etc/spider'
	ssh bitsy.spider 'killall pcm'
else
	@echo "******************** WARNING ******************************"
	@echo "** I am neither itsy nor bitsy"
	@echo "** 'make restart' should not be run on other systems"
	@echo "** Just stop and start pcm as you would any other program"
	@echo "******************** WARNING ******************************"
endif
endif

