shared=$(top_srcdir)/../common

AM_CFLAGS = -g -Wall -I$(shared)/include

AUTOMAKE_OPTIONS = foreign

if INSTALL_BLAST
blast_config=$(top_srcdir)/../blast_config
blastcmd_SOURCES = blastcmd.c $(blast_config)/command_list.c daemon.c elog.c paramdefs.c  $(shared)/netcmd.c \
		   $(blast_config)/command_list.h daemon.h elog.h isc_protocol.h \
			 $(shared)/netcmd.h
blastcmd_CPPFLAGS = -D__BLAST__ -I$(blast_config)/include
BLASTCMD=blastcmd
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

