/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2004 University of Toronto
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
#include "tx_struct.h"
extern unsigned short* slow_data[FAST_PER_SLOW];

/* thread identity */
extern pthread_key_t identity;

/* logging definitions */
#define MCP_INFO    0
#define MCP_STARTUP 1
#define MCP_SCHED   2
#define MCP_WARNING 3
#define MCP_ERROR   4
#define MCP_TFATAL  5
#define MCP_FATAL   6

void mprintf(int flag, char* fmt, ...);
void merror(int flag, char* fmt, ...);
void mputs(int flag, const char* message);

/* debugging definitions */
extern unsigned int debug;

#define DBG_NONE   0x00000000
#define DBG_TRACE1 0x00000001 /* trace level 1 - major functions */
#define DBG_TRACE2 0x00000002 /* trace level 2 - */
#define DBG_TRACE3 0x00000004 /* trace level 3 - everything */
#define DBG_ASSRT  0x00000008
#define DBG_WATCH  0x00000010
#define DBG_INFO1  0x00000020
#define DBG_INFO2  0x00000040
#define DBG_INFO3  0x00000080
#define DBG_INFO4  0x00000100
#define DBG_ALL    0xFFFFFFFF

#endif
