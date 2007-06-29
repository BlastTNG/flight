/* mcp: the Spider master control program
 *
 * mcp.h
 * 
 * This software is copyright (C) 2002-2007 University of Toronto
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
extern unsigned short* tdrss_data[2];
extern unsigned int tdrss_index;
extern pthread_mutex_t tdrss_lock;

#define GETREADINDEX(i) ((i+1) % 2)  /* index other than write index */

#define TEMPORAL_OFFSET 0

#define MAX_LINE_LENGTH 1024

#ifdef DEBUG
#warning "Debugging set."
#endif

#endif
