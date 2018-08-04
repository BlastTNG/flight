/* 
 * actuators.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 * History:
 * Created on: Apr 16, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_ACTUATORS_H_
#define INCLUDE_ACTUATORS_H_
// set microstep resolution for lockpin and shutter
#define LOCK_PREAMBLE "j256"
#define SHUTTER_PREAMBLE "j256"
// set encoder/microstep ratio (aE25600), coarse correction band (aC50),
// fine correction tolerance (ac%d), stall retries (au5),
// enable encoder feedback mode (n8)
// NB: this is a printf template now, requires a move tolerance (ac) to be set, default from BLAST-Pol is 2
#define ACT_PREAMBLE  "aE25600aC50ac%dau5n8"

void StoreActBus(void);
void SecondaryMirror(void);
void *ActuatorBus(void *param);
int GetActAddr(int ind);

#endif /* INCLUDE_ACTUATORS_H_ */
