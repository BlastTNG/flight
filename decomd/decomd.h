/* decomd: reads the decom stream, performs integrity checking, and writes it
 * to disk
 *
 * This software is copyright (C) 2004 D.V. Wiebe
 * 
 * This file is part of decomd.
 * 
 * decomd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * decomd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with decomd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef DECOMD_H
#define DECOMD_H

#include <syslog.h>

extern inline void syserror(int priority, char* preamble);

/* logging definitions */
#define MCP_INFO    LOG_INFO
#define MCP_STARTUP LOG_INFO
#define MCP_WARNING LOG_WARNING
#define MCP_ERROR   LOG_ERR
#define MCP_TFATAL  LOG_CRIT
#define MCP_FATAL   LOG_CRIT

#define mprintf syslog
#define merror syserror
#define mputs(x,y) syslog(x, "%s", y)

#endif
