include ../Makefile.config

all: 

install:
	install -d -m 777 $(DATA_DIR)/log
	install -D -m 666 prev_status $(DATA_DIR)/prev_status
	[ -e $(DATA_DIR)/log.txt ] || install -m 666 log.txt $(DATA_DIR)
	install -m 644 *.jpg $(DATA_DIR)

clean:
	rm *~ || true
