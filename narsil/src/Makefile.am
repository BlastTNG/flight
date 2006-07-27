AUTOMAKE_OPTIONS = foreign
BUILT_SOURCES = moc_narsil.cpp moc_doubleentry.cpp
narsil_s = narsil.cpp command_list.c doubleentry.cpp netcmd.c \
								 doubleentry.h command_list.h isc_protocol.h narsil.h netcmd.h \
								 $(BUILT_SOURCES)
narsil_l = @QT_LDADD@ $(top_srcdir)/lib/getdata.o $(top_srcdir)/lib/kstfile.o -L$(top_srcdir)/lib -lRD
if NARSILPP
bin_PROGRAMS = narsil.bin
narsil_bin_SOURCES = ${narsil_s}
narsil_bin_LDADD = ${narsil_l}
bin_SCRIPTS = narsil

.PHONY: narsil
narsil:
	echo '#!/bin/sh' > narsil
	echo 'export NARSIL_HOST=$$1; if [ "x$$NARSIL_HOST" == "x" ]; then unset NARSIL_HOST; fi; narsil.bin' >> narsil
	chmod 755 narsil

else
bin_PROGRAMS = narsil
narsil_SOURCES = ${narsil_s}
narsil_LDADD = ${narsil_l}
endif
INCLUDES = -Wall -I$(top_srcdir)/include @QT_CXXFLAGS@
MOC = @MOC@

moc_doubleentry.cpp: doubleentry.h
	$(MOC) -o moc_doubleentry.cpp doubleentry.h

moc_narsil.cpp: narsil.h
	$(MOC) -o moc_narsil.cpp narsil.h
