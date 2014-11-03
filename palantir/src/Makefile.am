AUTOMAKE_OPTIONS = foreign
BUILT_SOURCES = moc_palantir.cpp
bin_PROGRAMS = palantir
palantir_SOURCES = adamdom.cpp adamdom.h decompoll.cpp decompoll.h palantir.cpp palantir.h \
		   $(BUILT_SOURCES)
palantir_LDADD = @QT_LDADD@ -L$(top_srcdir)/lib -lgetdata++
INCLUDES = -I$(top_srcdir)/include @QT_CXXFLAGS@
MOC = @MOC@

moc_palantir.cpp: palantir.cpp palantir.h
	  $(MOC) -o moc_palantir.cpp palantir.h
