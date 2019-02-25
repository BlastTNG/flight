#include "groundhog_funcs.h"

/* ------ BEGIN CUSTOM GROUNDHOG DEFINITIONS ------ */

// groundhog customization
#define GROUNDHOG_MAX_FRAMES_RESET 900

// BLAST general
#include "blast.h"
#include "blast_time.h"
#define ROACH_CHANNEL_REF_NAME "kidA_roachN"
#define ROACH_CHANNEL_REF_INDEX_NAME "kidA_roachN_index"

// BLAST telemetry
#include "channels_tng.h"
#include "derived.h"
#include "FIFO.h"
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a):(b))
#endif
enum DownLinkTypes {PILOT, BI0, HIGHRATE, 
                      NUM_DOWNLINKS};

// BLAST pilot
#include "bitserver.h"
#include "pilot.h"
void udp_receive(void *arg);

// BLAST biphase
#include "bbc_pci.h"
#include "decom_pci.h"
#include "bi0.h"
void biphase_receive(void *arg);

// BLAST highrate
#include "highrate.h"
#include "comms_serial.h"
void highrate_receive(void *arg);

