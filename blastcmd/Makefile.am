buildserial = @CMDVERSION@
CONFIG_STATUS_DEPENDENCIES = $(top_srcdir)/command_list.c

INCLUDES=-Wall

AUTOMAKE_OPTIONS = foreign
blastcmd_SOURCES = blastcmd.c command_list.c daemon.c netcmd.c command_list.h \
									 daemon.h isc_protocol.h netcmd.h
bin_PROGRAMS = blastcmd

dataetcdir = @DATA_ETC_DIR@
EXTRA_DIST = blastcmd.log bootstrap.sh

if INSTALL_BLASTCMD_LOG
dataetc_DATA = blastcmd.log
endif

DISTCHECK_CONFIGURE_FLAGS = --with-data-etc="$$dc_install_base/data"

distinstall: distcheck
	cp ${distdir}.tar.gz ${distdir}-${buildserial}.tar.gz
	chown dwiebe:blast ${distdir}-${buildserial}.tar.gz
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}-${buildserial}.tar.gz ${prefix}/src
	scp ${distdir}-${buildserial}.tar.gz blast.physics.utoronto.ca:/data/src

if INSTALL_BLASTCMD_LOG
install-data-hook:
	chmod 666 $(DESTDIR)$(dataetcdir)/blastcmd.log
else
install-data-hook:
	/bin/true
endif
