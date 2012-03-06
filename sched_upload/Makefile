all: sched_upload sanity

# the name of the BLAST-etc project in the repository
BLASTETC = blast_etc

# a list of all the remote projects needed to build
REMOTEPROJ = $(BLASTETC)

ETC_DATA = sched.library

include ../common/Makefile.update

../common/Makefile.update:
	@echo "ERROR: **************************************************************"
	@echo "Common build files not found; unable to continue.  Try:"
	@echo "  ( cd .. && svn co `svn info --non-interactive | awk ' /URL/ { print $$2 } ' | sed -e 's/trunk\/.*/trunk\/common/'` common )"
	@false

sched_upload: sched_upload.c
	gcc -O3 -o sched_upload sched_upload.c -lm

sanity: sanity.c
	gcc -o sanity sanity.c

install: sched_upload sanity
	install -m 755 sched_upload /usr/local/bin
	install -m 755 sanity /usr/local/bin

clean:
	rm -f sched_upload sanity
	rm -f *.o
