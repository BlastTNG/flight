AUTOMAKE_OPTIONS = foreign
blastcmd_SOURCES = blastcmd.c command_list.c command_list.h isc_protocol.h
if INSTALL_BLASTCMD
bin_PROGRAMS = blastcmd
endif

dataetcdir = @DATA_ETC_DIR@
EXTRA_DIST = blastcmd.log fifocmd

if INSTALL_BLASTCMD_LOG
dataetc_DATA = blastcmd.log
endif

if INSTALL_FIFOCMD
FIFOCMD_BIN = fifocmd
endif

if INSTALL_BLASTCMD_REMOTE
BLASTCMD_REMOTE_BIN = blastcmd-remote
endif

bin_SCRIPTS = $(FIFOCMD_BIN) $(BLASTCMD_REMOTE_BIN)

DISTCHECK_CONFIGURE_FLAGS = --with-data-etc="$$dc_install_base/data" --enable-fifocmd --enable-blastcmd

distinstall: distcheck
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}.tar.gz ${prefix}/src
	scp ${distdir}.tar.gz blast.physics.utoronto.ca:/data/src
