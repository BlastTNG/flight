dataetcnarsildir = @DATA_ETC_DIR@/narsil
EXTRA_DIST = prev_status sword0.jpg sword1.jpg sword2.jpg sword3.jpg log.txt

if INSTALL_LOG_TXT
NARSIL_LOG = log.txt
endif

dataetcnarsil_DATA = prev_status sword0.jpg sword1.jpg sword2.jpg sword3.jpg \
										 ${NARSIL_LOG}

install-data-hook:
	chmod 666 $(dataetcnarsildir)/prev_status $(dataetcnarsildir)/log.txt
	[ -d $(dataetcnarsildir)/log ] || mkdir $(dataetcnarsildir)/log
	chmod 777 $(dataetcnarsildir)/log
