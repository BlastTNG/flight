/* mcp: the master control program
 *
 * This software is copyright (C) 2002-2011 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef MCP_H
#define MCP_H

#include <pthread.h>
#include "share/channels.h"
#include "calibrate.h"
#include "share/blast.h"

//configuration for BLASTBus
//use external serial numbers and synchronzation; remove for internal
//#define USE_EXT_SERIAL
//number of snyc [sic] box serial pulses per BLASTbus frame (only for external)
#define SERIAL_PER_FRAME 2

extern unsigned short* slow_data[FAST_PER_SLOW];
extern unsigned int BBFrameIndex;
extern unsigned short* RxFrame;

extern time_t mcp_systime(time_t *t);
extern struct frameBuffer hiGain_buffer;

extern int BLASTBusUseful;

#define GETREADINDEX(i) ((i+2) % 3)  /* i - 1 modulo 3 */
#define INC_INDEX(i) ((i + 1) %3)    /* i + 1 modulo 3 */

struct chat_buf {
  char msg[4][2 * FAST_PER_SLOW]; /* 4 buffers of FAST_PER_SLOW BLASTbus words of characters */
  int reading; /* the buffer we're currently reading from */
  int writing; /* the buffer we're currently writing to */
};

// Max Slew Veto
#define VETO_MAX 60000

#define TEMPORAL_OFFSET 0

#define MAX_LINE_LENGTH 1024

#ifdef TEST_RUN
#define USE_FIFO_CMD
#endif

//#define USE_XY_THREAD

#endif
