/*
 * cryovalves.c:
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

#include <stdlib.h>
#include <stdio.h>

#include "mcp.h"
#include "ezstep.h"
#include "command_struct.h"
#include "tx.h"
#include "actuators.h"
#include "cryovalves.h"

#define POTVALVE_OPEN 8000
#define POTVALVE_CLOSED 2000
#define POTVALVE_LOOSE_CLOSED 4000


typedef enum {
	no_move = 0, opening, closing, tighten
}move_type_t;

static struct potvalve_struct {
	int addr;
	int pos;
	int adc[4];
	int moving;
	int open_i;
	int close_i;
	valve_state_t current, goal;
	move_type_t potvalve_move;
} potvalve_data;

void DoCryovalves(struct ezbus* bus)
{
	static int firsttime = 1;
	static int pot_init = 0;

	if (CommandData.Cryo.potvalve_goal == opened) {
		potvalve_data.goal =  CommandData.Cryo.potvalve_goal;
	} else if (CommandData.Cryo.potvalve_goal == closed) {
		if ((potvalve_data.current == opened) || (potvalve_data.current == intermed)) {
			potvalve_data.goal = loose_closed;
		} else {
			potvalve_data.goal = closed;
		}
	} else {
		bputs(info, "Invalid Pot Valve Goal");
	}

	if (firsttime) {
		potvalve_data.addr = GetActAddr(POTVALVE_NUM);
		EZBus_Take(bus, potvalve_data.addr);
		blast_info("Making sure the potvalve is not running on startup.");
		EZBus_Stop(bus, potvalve_data.addr);
		EZBus_Release(bus, potvalve_data.addr);

		potvalve_data.pos = 0;
		potvalve_data.moving = 0;
		potvalve_data.current = 0;
		potvalve_data.goal = 0;
		potvalve_data.potvalve_move = 0;

		if(EZBus_Comm(bus, potvalve_data.addr, "z0R") != EZ_ERR_OK)
			bputs(info, "Error initializing valve position");
		firsttime = 0;
	}

	// TODO(PCA): Add velocity/current settings

	GetPotValvePos(*bus);
	SetValveState();

	if (potvalve_data.current == potvalve_data.goal) {
		potvalve_data.potvalve_move = no_move;
		potvalve_data.moving = 0;

		if (potvalve_data.current == loose_closed) {
			potvalve_data.goal = closed;
			potvalve_data.potvalve_move = tighten;
		}
	} else {
		if (potvalve_data.moving != 1) {
			if (potvalve_data.current != opened && potvalve_data.goal == opened) {
				potvalve_data.potvalve_move = opening;
			} else if (potvalve_data.current != closed && potvalve_data.goal == closed) {
				potvalve_data.potvalve_move = closing;
			}
		}
	}

	switch(potvalve_data.potvalve_move) {
		case(no_move):
			EZBus_Stop(bus, potvalve_data.addr);
			potvalve_data.moving = 0;
			break;
		case(opening):
			if(EZBus_Comm(bus, potvalve_data.addr, "m50P0R") != EZ_ERR_OK)
                        	bputs(info, "Error opening pot valve");
				potvalve_data.moving = 1;
			break;
		case(closing):
			if(EZBus_Comm(bus, potvalve_data.addr, "m25D0R") != EZ_ERR_OK)
				bputs(info, "Error closing pot valve");
			potvalve_data.moving = 1;
			break;
		case(tighten):
			if(potvalve_data.moving == 0) { // if we're just starting to tighten
				EZBus_Stop(bus, potvalve_data.addr); // make sure we're actually stopped
				if(EZBus_Comm(bus, potvalve_data.addr, "m25D1000000R") != EZ_ERR_OK) // close by ~.5 turn
					bputs(info, "Error tightening pot valve");
				potvalve_data.moving = 1; // so command only gets sent once
			}
			break;
	}
	blast_info("pos: %d, current: %d, goal: %d, moving: %d",
		potvalve_data.adc[0], potvalve_data.current, potvalve_data.goal, potvalve_data.moving); // DEBUG PCA

	WriteValves();
}
void GetPotValvePos(struct ezbus bus)
{
	// EZBus_ReadInt(&bus, potvalve_data.addr, "?0", &potvalve_data.pos);
	EZBus_Comm(&bus, potvalve_data.addr, "?aa");
	sscanf(bus.buffer, "%hi,%hi,%hi,%hi", &potvalve_data.adc[0], &potvalve_data.adc[1],
		&potvalve_data.adc[2], &potvalve_data.adc[3]);
}


void SetValveState(void)
{
	if (potvalve_data.adc[0] >= POTVALVE_OPEN) {
		potvalve_data.current = opened;
	} else if (potvalve_data.adc[0] <= POTVALVE_LOOSE_CLOSED && potvalve_data.adc[0] > POTVALVE_CLOSED) {
		potvalve_data.current = loose_closed;
	} else if (potvalve_data.adc[0] <= POTVALVE_CLOSED
		&& potvalve_data.potvalve_move == tighten) { // only declare it closed if we went through tightening
		potvalve_data.current = closed;
	} else {
		potvalve_data.current = intermed;
	}
}

void WriteValves(void)
{
	static int firsttime = 1;

	static channel_t* posPotValveAddr;
	static channel_t* statePotValveAddr;

	if (firsttime) {
		posPotValveAddr = channels_find_by_name("potvalve_pos");
		statePotValveAddr = channels_find_by_name("potvalve_state");
	}
	SET_UINT16(posPotValveAddr, potvalve_data.adc[0]);
	SET_UINT16(statePotValveAddr, potvalve_data.current);
}
