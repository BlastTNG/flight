#ifndef INCLUDE_GROUNDHOG_H
#define INCLUDE_GROUNDHOG_H

#define GROUNDHOG_MAX_FRAMES_RESET 900

#define ROACH_CHANNEL_REF_NAME "kidA_roachN"
#define ROACH_CHANNEL_REF_INDEX_NAME "kidA_roachN_index"

#include <stdarg.h>

// BLAST function
#include "blast.h"
#include "blast_time.h"

// linklist
#include "linklist.h"
#include "linklist_writer.h"
#include "linklist_compress.h"

// BLAST general telemetry
#include "channels_tng.h"
#include "derived.h"
#include "FIFO.h"

// BLAST pilot
#include "bitserver.h"
#include "pilot.h"

// BLAST biphase
#include "bbc_pci.h"
#include "decom_pci.h"
#include "bi0.h"

// BLAST highrate
#include "highrate.h"
#include "comms_serial.h"

enum DownLinkTypes {PILOT, BI0, HIGHRATE, 
                      NUM_DOWNLINKS};

struct UDPSetup {
  char name[80];
  char addr[80];
  unsigned int port;
  unsigned int maxsize;
  unsigned int packetsize;
  int downlink_index;
};

struct TlmReport {
  linklist_t * ll;
  uint64_t framenum;
  int allframe;
};

void udp_receive(void *arg);
void pilot_publish(void *arg);

void biphase_receive(void *arg);
void biphase_publish(void *arg);

void highrate_receive(void *arg);
void highrate_publish(void *arg);

void groundhog_write_calspecs(char *, derived_tng_t *);
linklist_rawfile_t * groundhog_open_new_rawfile(linklist_rawfile_t *, linklist_t *, char *);

extern char datestring[80];
extern int verbose;
extern struct TlmReport pilot_report;
extern struct TlmReport bi0_report;
extern struct TlmReport highrate_report;
extern struct TlmReport sbd_report;

#define groundhog_info(fmt, ...) blast_info(fmt, ##__VA_ARGS__)
#define groundhog_warn(fmt, ...) blast_warn(fmt, ##__VA_ARGS__)
#define groundhog_fatal(fmt, ...) blast_fatal(fmt, ##__VA_ARGS__)

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a):(b))
#endif

#endif
