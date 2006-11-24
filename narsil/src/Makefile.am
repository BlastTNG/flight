AUTOMAKE_OPTIONS = foreign
BUILT_SOURCES = moc_narsil.cpp moc_doubleentry.cpp
narsil_s = getdata.c kstfile.cpp narsil.cpp command_list.c doubleentry.cpp netcmd.c \
								 doubleentry.h command_list.h isc_protocol.h narsil.h netcmd.h \
								 $(BUILT_SOURCES)
narsil_l = @QT_LDADD@ -L$(top_srcdir)/lib -lRD
bin_PROGRAMS = narsil
narsil_SOURCES = ${narsil_s}
narsil_LDADD = ${narsil_l}
INCLUDES = -g -Wall -I$(top_srcdir)/include @QT_CXXFLAGS@
MOC = @MOC@

moc_doubleentry.cpp: doubleentry.h
	$(MOC) -o moc_doubleentry.cpp doubleentry.h

moc_narsil.cpp: narsil.h
	$(MOC) -o moc_narsil.cpp narsil.h
