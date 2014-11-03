shared=$(top_srcdir)/../common

AM_CFLAGS = -g -Wall -I$(shared)

AUTOMAKE_OPTIONS = foreign

if INSTALL_BLAST
blast_config=$(top_srcdir)/../blast_config
blastcmd_SOURCES = blastcmd.c $(blast_config)/command_list.c daemon.c paramdefs.c  $(shared)/netcmd.c \
		   $(blast_config)/command_list.h daemon.h elog.h isc_protocol.h \
			 $(shared)/netcmd.h
blastcmd_CPPFLAGS = -D__BLAST__ -I$(blast_config)
BLASTCMD=blastcmd
endif

if INSTALL_MINICP
mini_config=$(top_srcdir)/../minicp_config
minicmd_SOURCES = blastcmd.c $(mini_config)/command_list.c daemon.c paramdefs.c  $(shared)/netcmd.c \
		  $(mini_config)/command_list.h daemon.h elog.h isc_protocol.h \
			$(shared)/netcmd.h
minicmd_CPPFLAGS = -D__MINI__ -I$(mini_config)
MINICMD=minicmd
endif

if INSTALL_SPIDER
spider_config=$(top_srcdir)/../spider_config
spidercmd_SOURCES = blastcmd.c $(spider_config)/command_list.c daemon.c paramdefs.c  \
										$(shared)/netcmd.c $(spider_config)/command_list.h daemon.h elog.h \
										isc_protocol.h $(shared)/netcmd.h
spidercmd_CPPFLAGS = -D__SPIDER__ -I$(spider_config)
SPIDERCMD=spidercmd
endif

bin_PROGRAMS = $(BLASTCMD) $(MINICMD) $(SPIDERCMD)
dist_bin_SCRIPTS = blastcmd-authorize

dataetcdir = @DATA_ETC_DIR@
EXTRA_DIST = blastcmd.log bootstrap.sh 

dataetc_DATA = blastcmd.auth.orig
if INSTALL_BLASTCMD_LOG
dataetc_DATA += blastcmd.log
endif

install-data-hook:
if INSTALL_BLASTCMD_LOG
	chmod 666 $(DESTDIR)$(dataetcdir)/blastcmd.log
endif
	chmod 666 $(DESTDIR)$(dataetcdir)/blastcmd.auth.orig
	if [ -e "$(DESTDIR)$(dataetcdir)/blastcmd.auth" ]; then \
	  echo "Cowardly refusing to overwrite existing file $(DESTDIR)$(dataetcdir)/blastcmd.auth"; \
	  rm -f $(DESTDIR)$(dataetcdir)/blastcmd.auth.orig; \
	else \
	  mv -f $(DESTDIR)$(dataetcdir)/blastcmd.auth.orig $(DESTDIR)$(dataetcdir)/blastcmd.auth; \
	  fi
	if [ ! -e $(DESTDIR)$(dataetcdir)/cmddefs.txt ]; then \
		touch $(DESTDIR)$(dataetcdir)/cmddefs.txt; \
		chmod 666 $(DESTDIR)$(dataetcdir)/cmddefs.txt; \
	fi

@MAKEFILE_UPDATE@
