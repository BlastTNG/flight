AUTOMAKE_OPTIONS = foreign
SUBDIRS = etc
bin_PROGRAMS = defile
man_MANS = defile.1
EXTRA_DIST = fixup.sh
if INSTALL_DEFILE_CONF
sysconf_DATA = defile.conf
endif
defile_SOURCES = defile.c frameread.c reader.c channels.c writer.c blast.c blast.h defile.h channels.h frameread.h derived.h
INCLUDES = -D__DEFILE__ -DLOCALSTATEDIR=\"${localstatedir}\"
DISTCHECK_CONFIGURE_FLAGS = --with-data-etc="$$dc_install_base/data"

defile.1: Makefile $(top_builddir)/defile.1.in $(top_srcdir)/fixup.sh
	rm -f defile.1 defile.1.tmp
	sh $(top_srcdir)/fixup.sh "$(prefix)" "$(top_builddir)" defile.1

distclean-local:
	rm -f defile.1

distinstall: distcheck
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}.tar.gz ${prefix}/src
	scp ${distdir}.tar.gz blast.physics.utoronto.ca:/data/src
