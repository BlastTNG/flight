all: sched_upload sanity

sched_upload: sched_upload.c
	gcc -O3 -o sched_upload sched_upload.c -lm

sanity: sanity.c
	gcc -o sanity sanity.c

install: sched_upload sanity
	install -m 755 sched_upload /usr/local/bin
	install -m 755 sanity /usr/local/bin

