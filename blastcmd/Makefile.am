buildserial = @CMDVERSION@
CONFIG_STATUS_DEPENDENCIES = $(top_srcdir)/command_list.c

INCLUDES=-g -Wall

AUTOMAKE_OPTIONS = foreign
blastcmd_SOURCES = blastcmd.c command_list.c daemon.c netcmd.c command_list.h \
		   daemon.h isc_protocol.h netcmd.h
bin_PROGRAMS = blastcmd
dist_bin_SCRIPTS = blastcmd-authorize

dataetcdir = @DATA_ETC_DIR@
EXTRA_DIST = blastcmd.log bootstrap.sh 

dataetc_DATA = blastcmd.auth.orig
if INSTALL_BLASTCMD_LOG
dataetc_DATA += blastcmd.log
endif

DISTCHECK_CONFIGURE_FLAGS = --with-data-etc="$$dc_install_base/data"

distinstall: distcheck
	cp ${distdir}.tar.gz ${distdir}-${buildserial}.tar.gz
	chown dwiebe:blast ${distdir}-${buildserial}.tar.gz
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}-${buildserial}.tar.gz ${prefix}/src
	scp ${distdir}-${buildserial}.tar.gz blast.physics.utoronto.ca:/data/src

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

