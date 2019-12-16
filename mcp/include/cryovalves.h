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

/* Index of the cyrostat valve motors in the Actbus structures */
/* 1 less than the ez-stepper addresses */
#define POTVALVE_NUM 7
#define PUMP1_VALVE_NUM 8
#define PUMP2_VALVE_NUM 9
#define N_PUMP_VALVES 2 // just the pump valves
#define NVALVES (N_PUMP_VALVES + 1) // all of the valves (pump valves + potvalve)

#define POTVALVE_ENC_MAX 15000
#define POTVALVE_ENC_MIN 2000

#define POTVALVE_PREAMBLE "j256"
#define PUMP_VALVES_PREAMBLE "n2j256"

void DoCryovalves(struct ezbus* bus, unsigned int actuators_init);
void DoPotValve(struct ezbus* bus);
void ControlPotValve(struct ezbus* bus);
void DoValves(struct ezbus* bus, int index, char addr);
void GetPotValvePos(struct ezbus* bus);
int SetValveState(int tight_flag);
void WriteValves(unsigned int actuators_init, int* valve_addr);

#endif /* CRYOVALVES_H_ */
