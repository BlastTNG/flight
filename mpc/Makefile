# MPC: MCE-PCM communicator
#
# Copyright (C) 2013, D. V. Wiebe
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

########################################################
# MAS integration
#

# Can we execute ${MAS_VAR}?
ifneq (${MAS_VAR},)
MAS_VAR_WORKS=$(shell if [ -x ${MAS_VAR} ]; then echo yes; else echo no; fi)
ifeq (${MAS_VAR_WORKS},no)
$(warning *********************************************)
$(warning Ignoring bad MAS_VAR value: ${MAS_VAR})
MAS_VAR=
endif
endif

# Is mas_var new enough?
MIN_MAS_VERSION=820
ifneq (${MAS_VAR},)
MAS_HAS_INT_VERSION=$(shell if ${MAS_VAR} --int-verson >/dev/null 2>/dev/null; \
										then echo no; else echo yes; fi)
ifeq (${MAS_HAS_INT_VERSION},no)
	MAS_NEW_ENOUGH=no
else
	MAS_NEW_ENOUGH=$(shell \
								 if [ `${MAS_VAR} --int-version` -ge ${MIN_MAS_VERSION} ]; \
								 then echo yes else echo no; \
								 fi)
endif
ifeq (${MAS_NEW_ENOUGH},no)
$(warning *********************************************)
$(warning MAS too old.  Revision ${MIN_MAS_VERSION} or newer required.)
MAS_VAR=
endif

endif

ifeq (${MAS_VAR},)
$(warning *********************************************)
$(warning MAS_VAR not set; unable to link against MAS)
$(warning *********************************************)
$(error Cannot continue)
else
MAS_SCRIPT=$(shell ${MAS_VAR} --script-dir)
MAS_VERSION=$(shell ${MAS_VAR} --version)

MAS_LIBS=$(shell ${MAS_VAR} --libs) -Wl,-rpath \
				 -Wl,$(shell ${MAS_VAR} --prefix)/lib -lconfig
MAS_INC=$(shell ${MAS_VAR} --cflags)
endif
MAS_DEFS=-DMAS_DATA_ROOT=\"${MAS_DATA_ROOT}\" \
				 -DMAS_SCRIPT=\"${MAS_SCRIPT}\" \
				 -DMAS_VERSION=\"${MAS_VERSION}\"

########################################################
# Build system set up
#
CC = gcc
CPPFLAGS = -D__MPC__ ${MAS_DEFS} ${MAS_INC}
CFLAGS = -g -O3 -Wall
LDLIBS = -lm -lpthread ${MAS_LIBS}

########################################################
# External repository dependencies
#

SPIDER_ONLY=1

########################################################
# Inputs
#

EXPT_HEADERS = bset.h command_list.h mce_blob.h mce_counts.h mpc_proto.h tes.h
SHARED_HEADERS = blast.h command_common.h mcp_sched.h mputs.h sys.h udp.h
LOCAL_HEADERS = bset_dir.h expcfg_list.h mpc.h mce_frame.h mce_struct.h

EXPT_OBJS = bset.o command_list.o mpc_proto.o
SHARED_OBJS = blast.o command_common.o mputs.o sys.o udp.o
LOCAL_OBJS = blob.o cfg.o data_mode.o expcfg_list.o bolo_stats.o frame.o \
						 mas.o meta.o mce_struct.o mpc.o task.o

# We don't acutally need this, but it causes Makefile.shared to require
# spider_etc, which is necessary to build this file.
ETC_DATA=*.bset

########################################################
# Run the sanity checks and prepare the build system
#

include ../common/Makefile.shared

########################################################
# Build Stuff
#

.PHONY: all

all:: mpc format.bolo_stats

FORCE:

mce_status.g: FORCE
	/data/mas/bin/mce_status -G > mce_status.g.new
	@if test -e mce_status.g && diff -q mce_status.g mce_status.g.new; then \
		echo "mce_status.g is unchanged."; \
		rm -f mce_status.g.new; \
	else \
		echo "regenerated mce_status.g"; \
		mv -f mce_status.g.new mce_status.g; \
	fi

mce_counts.stamp: mce_counts.h.new
	@if test -e mce_counts.h.new; then \
		if test -e ../spider_config/mce_counts.h && \
		diff -q ../spider_config/mce_counts.h mce_counts.h.new; then \
			echo "mce_counts.h is unchanged."; \
			rm -f mce_counts.h.new; \
		else \
			echo "regenerated mce_counts.h"; \
			mv -f mce_counts.h.new ../spider_config/mce_counts.h; \
		fi; \
	fi
	@touch $@

MCE_STATUS_STUFF=mce_struct.c mce_counts.h.new

../spider_config/mce_counts.h: mce_counts.stamp
	@if test ! -f $@; then \
		rm -f $<; \
		$(MAKE) $<; \
	fi

${MCE_STATUS_STUFF}: mce_struct.stamp

mce_struct.stamp: mce_status.g make_mce_struct expcfg_list.c
	./make_mce_struct || ( rm -f ${MCE_STATUS_STUFF}  && exit 1 )
	@touch $@

../common/Makefile.shared:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

clean:
	rm -f mpc
	rm -f make_stats_etc
	rm -f *.o
	rm -f mce_status.g mce_status.g.stamp
	rm -f ${MCE_STATUS_STUFF}

mpc: $(OBJS)
	$(CC) $(LDFLGS) $(CFLAGS) $(OBJS) $(LDLIBS) -o mpc

make_stats_etc: make_stats_etc.o
	$(CC) $(LDFLGS) $(CFLAGS) make_stats_etc.o $(LDLIBS) -o $@

format.bolo_stats: make_stats_etc
	./make_stats_etc

########################################################
# Install Stuff
#

MAS_BIN_STUFF=mpc mpc_inf_loop replicate choose_tes_bias eval_squid_tune
install: mpc mpc_inf_loop
	install -d /data/mas/bin
	install -m 755 ${MAS_BIN_STUFF} /data/mas/bin
	install -m 755 set_directory /data/mas/mce_script/script
	install -m 666 $(ETC_DATA) /data/mas/etc
	/data/mas/bin/replicate

install-all: install
	for x in 1 2 3 4 5 6; do \
		if [ `hostname` != "x$$x" ]; then \
		echo "================== X$$x ====================="; \
		rsync -avP ${MAS_BIN_STUFF} x$$x.spider:/data/mas/bin; \
		rsync -avP set_directory x$$x.spider:/data/mas/mce_script/script; \
		rsync -avP $(ETC_DATA) x$$x.spider:/data/mas/etc; \
		ssh x$$x.spider /data/mas/bin/replicate; \
		fi; done

install-mce-script:
	(cd ../../mce_script && make install)
	/data/mas/bin/replicate
	for x in 1 2 3 4 5 6; do \
		if [ `hostname` != "x$$x" ]; then \
		echo "================== X$$x ====================="; \
		rsync -avP /data/mas/mce_script x$$x.spider:/data/mas/; \
		ssh x$$x.spider /data/mas/bin/replicate; \
		fi; done

install-config:
	(cd ../../spider_tools/config/mce && make install )
	/data/mas/bin/replicate
	for x in 1 2 3 4 5 6; do \
		if [ `hostname` != "x$$x" ]; then \
		echo "================== X$$x ====================="; \
		rsync -avP /data/mas/config x$$x.spider:/data/mas/; \
		ssh x$$x.spider /data/mas/bin/replicate; \
		fi; done

install-mas:
	(cd ../../mas && make install)
	/data/mas/bin/replicate
	for x in 1 2 3 4 5 6; do \
		if [ `hostname` != "x$$x" ]; then \
		echo "================== X$$x ====================="; \
		rsync -avP /data/mas/bin x$$x.spider:/data/mas/; \
		rsync -avP /data/mas/etc x$$x.spider:/data/mas/; \
		rsync -avP /data/mas/include x$$x.spider:/data/mas/; \
		rsync -avP /data/mas/lib x$$x.spider:/data/mas/; \
		rsync -avP /data/mas/python x$$x.spider:/data/mas/; \
		ssh x$$x.spider /data/mas/bin/replicate; \
		fi; done

mpcrestart: install-all
	for x in 1 2 3 4 5 6; do ssh x$$x.spider killall mpc; done
