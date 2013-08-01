AUTOMAKE_OPTIONS = foreign

man_MANS = defile-blast.1 defile-spider.1
EXTRA_DIST = fixup.sh bootstrap.sh

blastetcdir = @DATA_ETC_DIR@/blast
spideretcdir = @DATA_ETC_DIR@/spider

# calculate the svn revision
SVNREV=$(shell svn info | awk '/^Revision/ { print $$2 }')

if INSTALL_DEFILE_CONF
etc_DATA = defile.conf
endif
if INSTALL_BLAST_ETC
blastetc_DATA = ../blast_etc/*
endif
if INSTALL_SPIDER_ETC
spideretc_DATA = ../spider_etc/*
endif

bin_PROGRAMS = defile-blast defile-spider
bin_SCRIPTS = bolo_lookup

defile_blast_SOURCES = defile.c ../common/frameread.c reader.c \
											 ../common/channels.c writer.c ../common/blast.c \
											 client.c ../common/crc.c \
											 ../common/quendiclient.c ../common/blast.h defile.h \
											 ../common/channels.h frameread.h ../common/derived.h \
											 quenya.h ../common/crc.h quendiclient.h
defile_spider_SOURCES = $(defile_blast_SOURCES)

CFLAGS = -Wall -O3
AM_CPPFLAGS = -I../common -D__DEFILE__ -DLOCALSTATEDIR=\"${localstatedir}\" \
				 -DSVNREV=\"${SVNREV}\" -g
defile_blast_CPPFLAGS = -DDAS_CARDS=12 $(AM_CPPFLAGS) -D__BLAST__
defile_spider_CPPFLAGS = $(AM_CPPFLAGS) -D__SPIDER__

include ../common/Makefile.update

DISTCHECK_CONFIGURE_FLAGS = --with-data-etc="$$dc_install_base/data" --with-etc="$$dc_install_base/etc"

defile.conf: Makefile $(top_builddir)/defile.conf.in $(top_srcdir)/fixup.sh
	rm -f defile.conf defile.conf.tmp
	sh $(top_srcdir)/fixup.sh "$(prefix)" "$(top_builddir)" defile.conf

defile-blast.1 defile-spider.1: defile.1
	cp $< $@

defile.1: Makefile $(top_builddir)/defile.1.in $(top_srcdir)/fixup.sh
	rm -f defile.1 defile.1.tmp
	sh $(top_srcdir)/fixup.sh "$(prefix)" "$(top_builddir)" defile.1

distclean-local:
	rm -f defile.1
	rm -f defile.conf

distinstall: distcheck
	$(mkinstalldirs) ${prefix}/src
	${INSTALL_DATA} ${distdir}.tar.gz ${prefix}/src
	scp ${distdir}.tar.gz blast.physics.utoronto.ca:/data/src
