CC = gcc
CXX = g++
CPPFLAGS = -D__MCP__ $(BOLOTEST)
CFLAGS = -g -O3 -Wall
LDLIBS = -pthread -lm -lstdc++ -lgsl -lgslcblas

########################################################
# External repository dependencies
#

SPIDER_ONLY=1

########################################################
# Inputs
#

SHARED_HEADERS = bbc_pci.h blast.h camcommunicator.h camstruct.h channels.h \
								 command_common.h compressstruct.h crc.h derived.h \
								 ephem_read.h ephem_types.h fir.h flcdataswap.h lut.h \
								 mcp_sched.h mputs.h netcmd.h sip.h slowdl.h starpos.h sys.h \
								 udp.h
EXPT_HEADERS = bset.h calibrate.h command_list.h mce_blob.h mce_counts.h \
							 mpc_proto.h pointing_struct.h tes.h
LOCAL_HEADERS = amccommand.h bset_dir.h command_struct.h chrgctrl.h \
								drivecommunicator.h hwpr.h mceserv.h mcp.h motorcommand.h \
								motordefs.h phytron.h pyramid.h svd.h sync_comms.h tx.h

# These are needed by both bolotest and flight versions
SHARED_OBJS = blast.o channels.o command_common.o framefile.o lut.o mputs.o \
							sip.o sys.o udp.o
EXPT_OBJS = bset.o command_list.o derived.o tx_struct.o
LOCAL_OBJS = commands.o hk.o hwpr.o mcp.o phytron.o tx.o

# These are only needed by the flight version
ifndef BOLOTEST
SHARED_OBJS += camcommunicator.o compressfuncs.o compressionwriter.o crc.o \
								 ephem_read.o fir.o flcdataswap.o geomag.o radbox.o starpos.o  slowdl.o
LOCAL_OBJS += amccommand.o auxiliary.o chrgctrl.o dgps.o mag.o drivecommunicator.o \
								mceserv.o motors.o motorcommand.o pointing.o pyramid.o \
								sc.o sched.o svd.o sync_comms.o table.o
EXPT_OBJS += compressstruct.o mpc_proto.o slowdl_struct.o
endif

ETC_DATA = WMM.COF ephem.2000 sched.library *.sch *.lut *.bin *.bset

########################################################
# Test-run
#

#enable to setup a test run on non-flight systems (eg with simbbc)
#use test-run-enable target to enable
#to use 32-bit compilation on 64-bit systmems requires extra packages:
#  gcc-multilib g++-multilib libc6-dev-i386 ia32-libs
include Makefile.testing
ifeq ($(TEST_RUN), 1)
	CPPFLAGS := $(CPPFLAGS) -m32 -DTEST_RUN
	LDFLAGS := $(LDFLAGS) -m32
endif

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Build Stuff
#

.PHONY: fifo install all restart test-run-enable test-run-disable

all:: pcm

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

clean:
	rm -f pcm || true;
	rm -f *.o || true;
	rm -f tx_struct_mce.c || true
	rm -f framelist.inc || true

Makefile.testing:
	touch Makefile.testing

test-run-enable: clean
	echo "TEST_RUN = 1" > Makefile.testing

test-run-disable: clean
	echo "TEST_RUN = 0" > Makefile.testing

framelist.inc: ../spider_config/tx_struct.c tx_struct_mce.c
	../spider_config/make_framelist_inc > $@

compressstruct_spider.o :: framelist.inc

tx_struct_spider.o :: tx_struct_mce.c

tx_struct_mce.c: ../spider_config/make_tx_struct_mce \
	../spider_config/tes.h
	../spider_config/make_tx_struct_mce > $@

pcm: $(OBJS)
	$(CC) $(LDFLGS) $(CFLAGS) $(OBJS) $(LDLIBS) -o pcm

########################################################
# Install Stuff
#

install: pcm
	install -d -m 777 /data/rawdir
	install -m 755 pcm /usr/local/bin
	install -m 755 pcm_inf_loop /usr/local/bin
	install -d -m 777 /data/etc/spider
	install -m 666  $(ETC_DATA) /data/etc/spider
	install -m 755 set_sync.sh /usr/local/bin/set_sync
ifeq ($(TEST_RUN), 1)
	$(MAKE) fifo
endif

fifo:
	[ -e /data/etc/SIPSS.FIFO ] || mkfifo -m 0777 /data/etc/SIPSS.FIFO

restart: install
ifeq ($(shell hostname),bitsy)
	@echo "** I am bitsy **"
	killall pcm || true
	rsync -avP /usr/local/bin/pcm itsy.spider:/tmp/pcm
	ssh itsy.spider 'install -d -m 666 /tmp/etc'
	rsync -avP $(ETC_DATA) itsy.spider:/tmp/etc/
	ssh itsy.spider 'install -m 755 /tmp/pcm /usr/local/bin'
	ssh itsy.spider 'install -m 666 /tmp/etc/* /data/etc/spider'
	ssh itsy.spider 'killall pcm'
else
ifeq ($(shell hostname),itsy)
	@echo "** I am itsy **"
	killall pcm || true
	rsync -avP /usr/local/bin/pcm bitsy.spider:/tmp/pcm
	ssh bitsy.spider 'install -d -m 666 /tmp/etc'
	rsync -avP $(ETC_DATA) bitsy.spider:/tmp/etc/
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

