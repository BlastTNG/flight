AUTOMAKE_OPTIONS = foreign
SUBDIRS = etc
bin_PROGRAMS = defile
man_MANS = defile.1
defile_SOURCES = defile.c frameread.c reader.c channels.c writer.c blast.c blast.h defile.h channels.h frameread.h derived.h
INCLUDES = -D__DEFILE__

distinstall: distcheck
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}.tar.gz ${prefix}/src
