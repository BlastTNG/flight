CC = gcc
CPP = g++
CPPFLAGS = -g -O3 -Wall -D__MCP__ $(BOLOTEST)
LDFLAGS = -lpthread -lm -lstdc++ -lgsl -lgslcblas
ifeq ($(DEBUG), 1)
	CPPFLAGS := $(CPPFLAGS) -g -DDEBUG
	LDFLAGS := $(LDFLAGS) -g -DDEBUG
endif

HEADERS = amccommand.h bbc_pci.h blast.h channels.h calibrate.h chrgctrl.h \
		command_list.h command_struct.h compressstruct.h \
		copleycommand.h hwpr.h crc.h derived.h ephem_read.h \
		ephem_types.h ezstep.h fir.h isc_protocol.h lut.h mcp.h \
		motordefs.h netcmd.h pointing_struct.h sbsc_protocol.h \
		sbsccommunicator.h slow_dl.h sss_struct.h starpos.h tx.h
		

# These are needed by both bolotest and flight versions
OBJS = actuators.o blast.o channels.o command_list.o commands.o das.o \
		derived.o ezstep.o framefile.o hwpr.o lut.o mcp.o tx.o \
		tx_struct.o xystage.o

# These are only needed by the flight version
ifndef BOLOTEST
OBJS := $(OBJS) amccommand.o auxiliary.o chrgctrl.o compressstruct.o \
		compressionwriter.o copleycommand.o crc.o dgps.o ephem_read.o \
		fir.o geomag.o isc.o motors.o pointing.o radbox.o \
		sbsc.o sbsccommunicator.o sched.o slow_dl.o starpos.o \
		sunsensor.o
endif

.PHONEY: fifo install all restart

all: pcm

clean:
	rm -f pcm || true;
	rm -f *.o || true;

pcm: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o pcm

$(OBJS): $(HEADERS)

install: pcm
	install -d -m 777 /data/rawdir
	install -m 755 pcm /usr/local/bin
	install -m 755 pcm_inf_loop /usr/local/bin
	install -m 666 etc/WMM.COF etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 etc/sched.library /data/etc
#	install -m 666 etc/*.aml /data/etc
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
	scp etc/WMM.COF etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 etc/sched.library itsy.spider:/tmp/etc/
	ssh itsy.spider 'install -m 755 /tmp/pcm /usr/local/bin'
	ssh itsy.spider 'install -m 666 /tmp/etc/* /data/etc'
	ssh itsy.spider 'killall pcm'
else
ifeq ($(shell hostname),itsy)
	@echo "** I am itsy **"
	killall pcm
	scp /usr/local/bin/pcm bitsy.spider:/tmp/pcm
	ssh bitsy.spider 'install -d -m 666 /tmp/etc'
	scp etc/WMM.COF etc/*.sch etc/*.lut etc/*.txt etc/ephem.2000 etc/sched.library bitsy.spider:/tmp/etc/
	ssh bitsy.spider 'install -m 755 /tmp/pcm /usr/local/bin'
	ssh bitsy.spider 'install -m 666 /tmp/etc/* /data/etc'
	ssh bitsy.spider 'killall pcm'
else
	@echo "******************** WARNING ******************************"
	@echo "** I am neither itsy nor bitsy"
	@echo "** 'make restart' should not be run on other systems"
	@echo "** Just stop and start pcm as you would any other program"
	@echo "******************** WARNING ******************************"
endif
endif

