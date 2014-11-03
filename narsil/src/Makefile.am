AUTOMAKE_OPTIONS = foreign
BUILT_SOURCES = moc_narsil.cpp moc_doubleentry.cpp
narsil_s = narsil.cpp doubleentry.cpp ../../common/netcmd.c \
	   doubleentry.h narsil.h ../common/narsil.h $(BUILT_SOURCES)
narsil_l = @QT_LDADD@ 
bin_PROGRAMS = narsil
narsil_SOURCES = ${narsil_s}
narsil_LDADD = ${narsil_l} -lgetdata++
INCLUDES = -g -Wall $(QT_CXXFLAGS) $(SHARED_CPPFLAGS)

moc_doubleentry.cpp: doubleentry.h
	$(MOC) -o moc_doubleentry.cpp doubleentry.h

moc_narsil.cpp: narsil.h
	$(MOC) -o moc_narsil.cpp narsil.h

up update:
	cd .. && $(MAKE) $@
