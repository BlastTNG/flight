buildserial = @CMDVERSION@
CONFIG_STATUS_DEPENDENCIES = $(top_srcdir)/command_list.c

AUTOMAKE_OPTIONS = foreign
blastcmd_SOURCES = blastcmd.c command_list.c command_list.h isc_protocol.h
if INSTALL_BLASTCMD
bin_PROGRAMS = blastcmd
endif

dataetcdir = @DATA_ETC_DIR@
EXTRA_DIST = blastcmd.log fifocmd dummycmd

if INSTALL_BLASTCMD_LOG
dataetc_DATA = blastcmd.log
endif

if INSTALL_FIFOCMD
FIFOCMD_BIN = fifocmd
endif

if INSTALL_DUMMYCMD
DUMMYCMD_BIN = dummycmd
endif

if INSTALL_BLASTCMD_REMOTE
BLASTCMD_REMOTE_BIN = blastcmd-remote
endif

bin_SCRIPTS = $(FIFOCMD_BIN) $(BLASTCMD_REMOTE_BIN) $(DUMMYCMD_BIN)

DISTCHECK_CONFIGURE_FLAGS = --with-data-etc="$$dc_install_base/data" --enable-fifocmd --enable-blastcmd --enable-dummycmd

distinstall: distcheck
	cp ${distdir}.tar.gz ${distdir}-${buildserial}.tar.gz
	chown dwiebe:blast ${distdir}-${buildserial}.tar.gz
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}-${buildserial}.tar.gz ${prefix}/src
	scp ${distdir}-${buildserial}.tar.gz blast.physics.utoronto.ca:/data/src
