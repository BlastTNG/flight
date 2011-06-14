CFLAGS = -g -Wall
blastcmd_CFLAGS = -D__BLAST__
minicmd_CFLAGS = -D__MINI__

AUTOMAKE_OPTIONS = foreign
blastcmd_SOURCES = blastcmd.c command_list_blast.c daemon.c netcmd.c \
		   command_list_blast.h daemon.h isc_protocol.h netcmd.h
minicmd_SOURCES = blastcmd.c command_list_mini.c daemon.c netcmd.c \
		  command_list_mini.h daemon.h isc_protocol.h netcmd.h
bin_PROGRAMS = blastcmd minicmd
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

