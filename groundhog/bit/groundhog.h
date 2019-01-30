#include "groundhog_funcs.h"

/* ------ BEGIN CUSTOM GROUNDHOG DEFINITIONS ------ */

// groundhog customization
#define GROUNDHOG_MAX_FRAMES_RESET 900

// BIT telemetry
#include "netcmd.h"
#include "cmdparser.h"
#include "telemparser.h"
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a):(b))
#endif

// BIT pilot
#include "bitserver.h"
void udp_receive(void *arg);

// BIT biphase
void biphase_receive(void *arg);

// BIT telemetry reports
extern int verbose;
extern struct TlmReport pilot_report;
extern struct TlmReport bi0_report;
