AUTOMAKE_OPTIONS = foreign

SUBDIRS = data src
EXTRA_DIST = bootstrap.sh
DISTCHECK_CONFIGURE_FLAGS = --with-data-etc="$$dc_install_base/data"

distinstall: distcheck
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}.tar.gz ${prefix}/src
	scp ${distdir}.tar.gz blast.physics.utoronto.ca:/data/src

@MAKEFILE_UPDATE@
