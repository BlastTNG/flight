/*
 * xystage.h
 *
 * @date Mar 18, 2016
 * @author unk
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_XYSTAGE_H_
#define INCLUDE_XYSTAGE_H_

#define XYSTAGE_PREAMBLE "j256n2" // set microstep res, use limit switches

void StoreStageBus(int index);
void GoWait(struct ezbus *bus, int dest, int vel, int is_y);
void Raster(struct ezbus *bus, int start, int end, int is_y, int y,
    int ymin, int ymax, int xvel, int yvel, int ss);
void ControlXYStage(struct ezbus* bus);
void StageBus(void);


#endif /* INCLUDE_XYSTAGE_H_ */
