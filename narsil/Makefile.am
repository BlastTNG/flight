AUTOMAKE_OPTIONS = foreign

buildserial = @CMDVERSION@

SUBDIRS = data src
EXTRA_DIST = lib/libRD.a lib/kstfile.o lib/getdata.o include/kstfile.h include/getdata.h bootstrap.sh
DISTCHECK_CONFIGURE_FLAGS = --with-data-etc="$$dc_install_base/data"
CONFIG_STATUS_DEPENDENCIES = $(top_srcdir)/src/command_list.c

distinstall: distcheck
	cp ${distdir}.tar.gz ${distdir}-${buildserial}.tar.gz
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}-${buildserial}.tar.gz ${prefix}/src
	scp ${distdir}-${buildserial}.tar.gz blast.physics.utoronto.ca:/data/src
