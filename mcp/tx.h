#define EXPOSURE2I (65536. / 5000000.)  /* ISC exposure time to int */

#define NIOS_QUEUE  0
#define NIOS_FLUSH -1

/* An ISC/OSC handshaking debugging macro should be set to 0 for flight */
#define WHICH (which == 1)

#include "channels.h"

/* length of the balance autoveto in 100Hz frames */
#define BAL_VETO_MAX 1000

extern int mcp_initial_controls;

void InitTxFrame(unsigned short*);
void UpdateBBCFrame(unsigned short*);

void RawNiosWrite(unsigned int, unsigned int, int);
void WriteData(struct NiosStruct*, unsigned int, int);
