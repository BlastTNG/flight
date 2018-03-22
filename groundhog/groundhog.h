#ifndef INCLUDE_GROUNDHOG_H
#define INCLUDE_GROUNDHOG_H

#define FILE_SAVE_DIR "/data/groundhog"

#include "groundhog_framing.h"

struct UDPSetup {
  char name[80];
  char addr[80];
  unsigned int port;
  unsigned int maxsize;
  unsigned int packetsize;
  superframes_list_t * sf;
};

void udp_receive(void *arg);
void pilot_publish(void *arg);

void biphase_receive(void *arg);
void biphase_publish(void *arg);

void highrate_receive(void *arg);
void highrate_publish(void *arg);

extern char datestring[80];
extern superframes_list_t pilot_superframes;
extern superframes_list_t highrate_superframes;
extern superframes_list_t biphase_superframes; 

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a):(b))
#endif

#endif