#ifndef INCLUDE_GROUNDHOG_H
#define INCLUDE_GROUNDHOG_H

#define GROUNDHOG_MAX_FRAMES_RESET 900

#include "groundhog_framing.h"

struct UDPSetup {
  char name[80];
  char addr[80];
  unsigned int port;
  unsigned int maxsize;
  unsigned int packetsize;
  int downlink_index;
};

void udp_receive(void *arg);
void pilot_publish(void *arg);

void biphase_receive(void *arg);
void biphase_publish(void *arg);

void highrate_receive(void *arg);
void highrate_publish(void *arg);

void make_linklist_rawfile_name(linklist_t *, char *);

extern char datestring[80];

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a):(b))
#endif

#endif
