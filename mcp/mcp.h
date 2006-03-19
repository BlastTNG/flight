/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2004-2005 University of Toronto
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
#include "channels.h"
#include "blast.h"
extern unsigned short* slow_data[FAST_PER_SLOW];
extern unsigned int RxFrameFastSamp;

extern time_t mcp_systime(time_t *t);
extern unsigned short* tdrss_data[3];
extern unsigned int tdrss_index;

#define TEMPORAL_OFFSET 25401600

#define MAX_LINE_LENGTH 1024

#ifdef BOLOTEST
#  define USE_FIFO_CMD
#endif

#ifdef DEBUG
#warning "Debugging set."
#endif

#endif
