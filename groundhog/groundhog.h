#ifndef INCLUDE_GROUNDHOG_H
#define INCLUDE_GROUNDHOG_H

void pilot_receive(void *arg);
void pilot_publish(void *arg);
void biphase_receive(void *arg);
void biphase_publish(void *arg);
void tdrss_receive(void *arg);
void tdrss_publish(void *arg);

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a):(b))
#endif

#endif
