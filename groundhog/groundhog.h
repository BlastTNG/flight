#ifndef INCLUDE_GROUNDHOG_H
#define INCLUDE_GROUNDHOG_H

#define FILE_SAVE_DIR "/data/groundhog"

void pilot_receive(void *arg);
void pilot_publish(void *arg);
void biphase_receive(void *arg);
void biphase_publish(void *arg);
void highrate_receive(void *arg);
void highrate_publish(void *arg);

extern char datestring[80];

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a):(b))
#endif

#endif
