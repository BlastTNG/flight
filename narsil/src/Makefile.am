AUTOMAKE_OPTIONS = foreign
BUILT_SOURCES = moc_narsil.cpp moc_doubleentry.cpp
bin_PROGRAMS = narsil
narsil_SOURCES = doubleentry.cpp doubleentry.h command_list.c narsil.cpp \
								 command_list.h isc_protocol.h narsil.h $(BUILT_SOURCES) \
								 $(top_srcdir)/include/kstfile.h $(top_srcdir)/include/getdata.h
narsil_LDADD = @QT_LDADD@ $(top_srcdir)/lib/getdata.o $(top_srcdir)/lib/kstfile.o -lRD
INCLUDES = -I$(top_srcdir)/include @QT_CXXFLAGS@
MOC = @MOC@

moc_doubleentry.cpp: doubleentry.cpp doubleentry.h
	  $(MOC) -o moc_doubleentry.cpp doubleentry.h

moc_narsil.cpp: narsil.cpp narsil.h
	  $(MOC) -o moc_narsil.cpp narsil.h
