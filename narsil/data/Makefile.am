dataetcnarsildir = @DATA_ETC_DIR@/narsil
EXTRA_DIST = prev_status sword0.jpg sword1.jpg sword2.jpg sword3.jpg log.txt

if INSTALL_LOG_TXT
NARSIL_LOG = log.txt
endif

dataetcnarsil_DATA = prev_status narsil.png sword0.jpg sword1.jpg sword2.jpg \
										 sword3.jpg ${NARSIL_LOG}

install-data-hook:
	chmod 666 $(DESTDIR)$(dataetcnarsildir)/prev_status $(DESTDIR)$(dataetcnarsildir)/log.txt
	[ -d $(DESTDIR)$(dataetcnarsildir)/log ] || mkdir $(DESTDIR)$(dataetcnarsildir)/log
	chmod 777 $(DESTDIR)$(dataetcnarsildir)/log
