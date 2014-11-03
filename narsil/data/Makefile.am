dataetcnarsildir = @DATA_ETC_DIR@/narsil

DATA_FILES = prev_status sword0.jpg sword1.jpg sword2.jpg sword3.jpg narsil.png
EXTRA_DIST = ${DATA_FILES} log.txt

if INSTALL_LOG_TXT
NARSIL_LOG = log.txt
INSTALLED_LOG = $(DESTDIR)$(dataetcnarsildir)/log.txt
endif

dataetcnarsil_DATA = ${DATA_FILES} ${NARSIL_LOG}

install-data-hook:
	chmod 666 $(DESTDIR)$(dataetcnarsildir)/prev_status $(INSTALLED_LOG)
	[ -d $(DESTDIR)$(dataetcnarsildir)/log ] || mkdir $(DESTDIR)$(dataetcnarsildir)/log
	chmod 777 $(DESTDIR)$(dataetcnarsildir)/log
