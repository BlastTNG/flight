#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern "C" {
#include "pointing_struct.h"
#include "tx_struct.h"
#include "mcp.h"
}
#include "alice.h"

extern unsigned short* slow_data[FAST_PER_SLOW];

extern "C" void Alice(void) {
  unsigned short* local_data;
  
  mputs(MCP_STARTUP, "Alice startup\n");

  if ((local_data = (unsigned short*)malloc(BiPhaseFrameSize)) == NULL)
    merror(MCP_FATAL, "Unable to malloc local_data");

  while (1) {
    memcpy(local_data, AliceData[GETREADINDEX(alice_index)], BiPhaseFrameSize);
    usleep(10000000);
  }
}
