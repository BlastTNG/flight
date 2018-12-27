/**
 * @file hwp.h
 *
 * @date Aug 5, 2015
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

#ifndef INCLUDE_HWPR_H
#define INCLUDE_HWPR_H

#include "ezstep.h"

// #define HWPR_STEPS_PER_MOTENC (64)
// #define HWPR_REV_PER_MOTREV (24./100.) // is this from BLAST-Pol? PAW 20181203

#define HWPR_NAME "HWPR Motor"
#define HWPRNUM 5
#define HWPR_PREAMBLE "j256"

#define HWPR_CHECK_NONE 0
#define HWPR_CHECK_BEFORE 1
#define HWPR_CHECK_AFTER 2
#define HWPR_CHECK_BOTH 3

#define HWPR_POT_MIN 0.1
#define HWPR_POT_MAX 0.9

#define HWPR_DEFAULT_STEP 22.5 // used if pot is dead

void DoHWPR(struct ezbus* bus);
void StoreHWPRBus(void);
void ReadHWPREnc(void);
int GetHWPRIndex(double);
#endif
