AUTOMAKE_OPTIONS = foreign
BUILT_SOURCES = moc_palantir.cpp
bin_PROGRAMS = palantir
palantir_SOURCES = adamdom.cpp adamdom.h palantir.cpp palantir.h $(BUILT_SOURCES) $(top_srcdir)/include/kstfile.h $(top_srcdir)/include/getdata.h
palantir_LDADD = @QT_LDADD@ $(top_srcdir)/lib/getdata.o $(top_srcdir)/lib/kstfile.o -L$(top_srcdir)/lib -lRD
INCLUDES = -I$(top_srcdir)/include @QT_CXXFLAGS@
MOC = @MOC@

moc_palantir.cpp: palantir.cpp palantir.h
	  $(MOC) -o moc_palantir.cpp palantir.h
