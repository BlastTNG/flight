shared=$(top_srcdir)/../common

AM_CFLAGS = -g -Wall -I$(shared)

AUTOMAKE_OPTIONS = foreign

if INSTALL_BIT
bit_config=$(top_srcdir)/../bit_config
bitcmd_SOURCES = bitcmd.c bitdaemon.c paramdefs.c  $(shared)/netcmd.c \
		   bitdaemon.h \
			 $(shared)/netcmd.h
bitcmd_CPPFLAGS = -D__BIT__ -I$(bit_config)
BITCMD=bitcmd
endif

bin_PROGRAMS = $(BITCMD)
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
