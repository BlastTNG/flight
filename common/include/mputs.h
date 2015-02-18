/* mputs.c: BUOS backend for mcp/pcm/mpc
 *
 * This software is copyright (C) 2002-2013 University of Toronto
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef MPUTS_H
#define MPUTS_H

#include "blast.h"
#include "stdio.h"

void nameThread(const char* name);
void mputs(buos_t flag, const char* message);
off_t openMCElog(const char *name);

#define TEMPORAL_OFFSET 0

#endif
