/* mcp: the Spider master control program
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

#define EXPOSURE2I (65536. / 5000000.)  /* ISC exposure time to int */

#define NIOS_QUEUE  0
#define NIOS_FLUSH -1

/* An ISC/OSC handshaking debugging macro should be set to 0 for flight */
#define WHICH (0)

#include "channels.h"
#include "calibrate.h"
#include "tes.h" /* for NUM_MCE_FIELDS */

extern int mcp_initial_controls;
extern short int InCharge;

void InitTxFrame();
void UpdateBBCFrame();

void RawNiosWrite(unsigned int, unsigned int, int);
void WriteData(struct NiosStruct*, unsigned int, int);
void WriteCalData(struct NiosStruct*, double, int);
unsigned int ReadData(struct BiPhaseStruct*);
double ReadCalData(struct BiPhaseStruct*);
