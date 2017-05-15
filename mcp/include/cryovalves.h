/*
 * cryovalves.h:
 *
 * This software is copyright (C) 2017 Peter Ashton
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
 * Created on: Feb 23, 2017 Peter Ashton
 */

#ifndef INCLUDE_CRYOVALVES_H_
#define INCLUDE_CRYOVALVES_H_

#include "ezstep.h"

/* Index of the balance motor in the Actbus structures */
#define POTVALVE_NUM 7

#define POTVALVE_PREAMBLE "j256"

void DoCryovalves(struct ezbus* bus);
void GetPotValvePos(struct ezbus bus);
int SetValveState(void);
void WriteValves(void);

// void ControlBalance(void);

#endif /* CRYOVALVES_H_ */
