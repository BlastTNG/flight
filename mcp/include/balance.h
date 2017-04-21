 /* @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2016 University of Pennsylvania
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

#ifndef INCLUDE_BALANCE_H_
#define INCLUDE_BALANCE_H_

#include "ezstep.h"

/* Index of the balance motor in the Actbus structures */
#define BALANCENUM 3

// TODO(laura): Change this once we test different acceleration/velocity parameters.
#define BALANCE_PREAMBLE "j256n2" // set positive direction, enable limits, set microstep res.

void DoBalance(struct ezbus* bus);
void ControlBalance(void);

#endif /* BALANCE_H_ */
