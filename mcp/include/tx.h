/* mcp: the master control program
 *
 * This software is copyright (C) 2003-2011 University of Toronto
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


/* An ISC/OSC handshaking debugging macro should be set to 0 for flight */
#define WHICH (0)

#include "channels_tng.h"
#include "calibrate.h"
#include "channels_tng.h"

extern int mcp_initial_controls;

void UpdateBBCFrame(unsigned short*);
double ReadCalData(channel_t *m_ch);
void StoreData(void);
void WriteAux(void);
