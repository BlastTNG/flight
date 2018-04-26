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
#include <unistd.h>

#include "mcp.h"
#include "ezstep.h"
#include "command_struct.h"
#include "tx.h"
#include "actuators.h"
#include "cryovalves.h"


#define POTVALVE_OPEN 6000
#define POTVALVE_CLOSED 3500
#define POTVALVE_LOOSE_CLOSED 4500
#define NVALVES 2

typedef enum {
	no_move = 0, opening, closing, tighten
} move_type_t;

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

static struct valve_struct {
	int addr;
	int limit;
	int ready;
} valve_data[NVALVES];

void DoCryovalves(struct ezbus* bus, unsigned int actuators_init)
{
	int i;
	int valve_addr[NVALVES] = {PUMPVALVE_NUM, FILLVALVE_NUM};

	if (actuators_init & (0x1 << POTVALVE_NUM)) {
		DoPotValve(bus);
	}

	for (i = 0; i < NVALVES; i++) {
		if (actuators_init & (0x1 << valve_addr[i])) {
			DoValves(bus, i, valve_addr[i]);
		}
	}
	WriteValves(actuators_init, valve_addr);
}

void DoValves(struct ezbus* bus, int index, int addr)
{
	static int firsttime_pump_valve = 1;
	static int firsttime_fill_valve = 1;

	if (firsttime_pump_valve && (index == 0)) {
		valve_data[index].addr = GetActAddr(addr);
		// Debug PAW 04/24/2018
		// blast_info("Valve %d address is %c (firsttime loop)", index, valve_data[index].addr);

		EZBus_Take(bus, valve_data[index].addr);
		blast_info("Making sure Valve %d is not running on startup", index);
		EZBus_Stop(bus, valve_data[index].addr);
		EZBus_MoveComm(bus, valve_data[index].addr, VALVE_PREAMBLE);
		EZBus_Release(bus, valve_data[index].addr);
		CommandData.Cryo.valve_goals[index] = 0;
		firsttime_pump_valve = 0;
	}

	if (firsttime_fill_valve && (index == 1)) {
		valve_data[index].addr = GetActAddr(addr);
		// Debug PAW 04/24/2018
		// blast_info("Valve %d address is %c (firsttime loop)", index, valve_data[index].addr);

		EZBus_Take(bus, valve_data[index].addr);
		blast_info("Making sure Valve %d is not running on startup", index);
		EZBus_Stop(bus, valve_data[index].addr);
		EZBus_MoveComm(bus, valve_data[index].addr, VALVE_PREAMBLE);
		EZBus_Release(bus, valve_data[index].addr);
		CommandData.Cryo.valve_goals[index] = 0;
		firsttime_fill_valve = 0;
	}
	// blast_info("Valve %d address is %c", index, valve_data[index].addr);

	EZBus_SetVel(bus, valve_data[index].addr, CommandData.Cryo.valve_vel);
	EZBus_SetIMove(bus, valve_data[index].addr, CommandData.Cryo.valve_current);

	// Debug PAW 04/24/2018
	// blast_info("commanded valve velocity is %d", CommandData.Cryo.valve_vel);
	// blast_info("actual valve 9 velocity is %d", bus->stepper[8].vel);
	// blast_info("actual valve 10 velocity is %d", bus->stepper[9].vel);
	// blast_info("commanded valve current is %d", CommandData.Cryo.valve_current);
	// blast_info("actual valve 9 current is %d", bus->stepper[8].imove);
	// blast_info("actual valve 10 current is %d", bus->stepper[9].imove);


	// ?4 returns status of all 4 inputs, Bit 2 = opto 1, Bit 3 = opto 2
	EZBus_ReadInt(bus, valve_data[index].addr, "?4", &(valve_data[index].limit));
	valve_data[index].ready = !(EZBus_IsBusy(bus, valve_data[index].addr));

	if ((CommandData.Cryo.valve_goals[index] == opened) && (valve_data[index].limit != 11)) {
		if (valve_data[index].ready) {
			EZBus_Take(bus, valve_data[index].addr);
			EZBus_RelMove(bus, valve_data[index].addr, INT_MAX);
			EZBus_Release(bus, valve_data[index].addr);
			// blast_info("Starting to open Valve %d", index); // debug PAW
		} else {
			blast_info("Valve %d opening...", index);
		}
	} else if ((CommandData.Cryo.valve_goals[index] == closed) && (valve_data[index].limit != 7)) {
		if (valve_data[index].ready) {
			EZBus_Take(bus, valve_data[index].addr);
			EZBus_RelMove(bus, valve_data[index].addr, INT_MIN);
			EZBus_Release(bus, valve_data[index].addr);
                        // blast_info("Starting to close Valve %d", index); // debug PAW
		} else {
			blast_info("Valve %d closing...", index);
		}
	} else if (valve_data[index].limit == 7 || valve_data[index].limit == 11) {
		CommandData.Cryo.valve_goals[index] = 0;
	}
}


void DoPotValve(struct ezbus* bus)
{
	static int firsttime = 1;
	static int pot_init = 0;
	static int tight_flag;
	int firstmove;
	int newstate;
	int do_move;
	char buffer[EZ_BUS_BUF_LEN];

	if (CommandData.Cryo.potvalve_goal == opened) {
		potvalve_data.goal =  CommandData.Cryo.potvalve_goal;
	} else if (CommandData.Cryo.potvalve_goal == closed) {
		if ((potvalve_data.current == opened) || (potvalve_data.current == intermed)) {
			potvalve_data.goal = loose_closed;
		} else {
			potvalve_data.goal = closed;
		}
	}
	if (firsttime) {
		blast_info("IN FIRSTTIME BLOCK"); // DEBUG PCA
		potvalve_data.addr = GetActAddr(POTVALVE_NUM);

		EZBus_Take(bus, potvalve_data.addr);
		blast_info("Making sure the potvalve is not running on startup.");
		EZBus_Stop(bus, potvalve_data.addr);
		EZBus_MoveComm(bus, potvalve_data.addr, POTVALVE_PREAMBLE);
		EZBus_Release(bus, potvalve_data.addr);


		potvalve_data.pos = 0;
		potvalve_data.moving = 0;
		potvalve_data.current = 0;
		potvalve_data.goal = 0;
		potvalve_data.potvalve_move = 0;
		newstate = 1;
		firstmove = 1;
		tight_flag = 1;

		if(EZBus_Comm(bus, potvalve_data.addr, "z0R") != EZ_ERR_OK)
			bputs(info, "Error initializing valve position");
		firsttime = 0;
	}

	EZBus_SetVel(bus, potvalve_data.addr, CommandData.Cryo.potvalve_vel);

	GetPotValvePos(*bus);
	newstate = SetValveState(tight_flag);

	if (newstate) blast_info("POT VALVE NEW STATE"); // DEBUG PCA

	if (potvalve_data.current == potvalve_data.goal) {
		potvalve_data.potvalve_move = no_move;
		potvalve_data.moving = 0;

		if (potvalve_data.current == loose_closed) {
			blast_info("RESET GOAL CLOSED"); // DEBUG PCA
			potvalve_data.goal = closed;
			potvalve_data.potvalve_move = tighten;
		}
	} else {
		if (potvalve_data.moving != 1) {
			if (potvalve_data.current != opened && potvalve_data.goal == opened) {
				potvalve_data.potvalve_move = opening;
			} else if (potvalve_data.current != closed && (potvalve_data.goal == closed || potvalve_data.goal == loose_closed)) {
				potvalve_data.potvalve_move = closing;
			}
		}
	}

	do_move = (newstate || firstmove);
	firstmove = 0;

	if(do_move) {
	switch(potvalve_data.potvalve_move) {
		case(no_move):
			EZBus_Stop(bus, potvalve_data.addr);
			potvalve_data.moving = 0;
			break;
		case(opening):
			EZBus_SetIMove(bus, potvalve_data.addr, CommandData.Cryo.potvalve_opencurrent);
			if(EZBus_RelMove(bus, potvalve_data.addr, INT_MAX) != EZ_ERR_OK)
                        	bputs(info, "Error opening pot valve");
			potvalve_data.moving = 1;
			tight_flag = 0;
			break;
		case(closing):
			EZBus_SetIMove(bus, potvalve_data.addr, CommandData.Cryo.potvalve_closecurrent);
			if(EZBus_RelMove(bus, potvalve_data.addr, INT_MIN) != EZ_ERR_OK)
				bputs(info, "Error closing pot valve");
			potvalve_data.moving = 1;
			break;
		case(tighten):
			if(potvalve_data.moving == 0) { // if we're just starting to tighten
				blast_info("BEGIN TIGHTEN"); // DEBUG PCA
				blast_info("tighten current: %d; goal: %d", potvalve_data.current, potvalve_data.goal);
				EZBus_Stop(bus, potvalve_data.addr); // make sure we're actually stopped
				EZBus_SetIMove(bus, potvalve_data.addr, CommandData.Cryo.potvalve_closecurrent);
				if(EZBus_StrComm(bus, potvalve_data.addr, sizeof(buffer), buffer, "D1000000R") != EZ_ERR_OK) // close by ~.5 turn
					bputs(info, "Error tightening pot valve");
				tight_flag = 1;
				potvalve_data.moving = 1; // so command only gets sent once
			} else {
				blast_info("tighten, move != 0");
				if (!(EZBus_IsBusy(bus, potvalve_data.addr))) potvalve_data.moving = 0;
			}
			break;
	} // end switch
	} // end if(do_move)



	// blast_info("pos: %d, current: %d, goal: %d, moving: %d, tight: %d",
	// 	potvalve_data.adc[0], potvalve_data.current, potvalve_data.goal, potvalve_data.moving, tight_flag); // DEBUG PCA

	usleep(10000);
}
void GetPotValvePos(struct ezbus bus)
{
	// EZBus_ReadInt(&bus, potvalve_data.addr, "?0", &potvalve_data.pos);
	EZBus_Comm(&bus, potvalve_data.addr, "?aa");
	sscanf(bus.buffer, "%i,%i,%i,%i", &potvalve_data.adc[0], &potvalve_data.adc[1],
		&potvalve_data.adc[2], &potvalve_data.adc[3]);
}


int SetValveState(int tight_flag)
{
	valve_state_t prev = potvalve_data.current;

	if ((potvalve_data.adc[0] <= POTVALVE_CLOSED) && (tight_flag == 1)) {
		potvalve_data.current = closed;
	} else if (potvalve_data.adc[0] <= POTVALVE_LOOSE_CLOSED) {
		potvalve_data.current = loose_closed;
	} else if (potvalve_data.adc[0] >= POTVALVE_OPEN) {
		potvalve_data.current = opened;
	/* } else if (potvalve_data.adc[0] <= POTVALVE_LOOSE_CLOSED) {
		potvalve_data.current = loose_closed;
	} else if (potvalve_data.adc[0] <= POTVALVE_CLOSED) {
		// && tight_flag == 1) { // only declare it closed if we went through tightening
		potvalve_data.current = closed;*/
	} else {
		potvalve_data.current = intermed;
	}

	return (potvalve_data.current != prev);
}

void WriteValves(unsigned int actuators_init, int* valve_addr)
{
	int i;
	int valve_flag = 0;

	static channel_t* posPotValveAddr;
	static channel_t* statePotValveAddr;
	static channel_t* velPotValveAddr;
	static channel_t* openCurPotValveAddr;
	static channel_t* closeCurPotValveAddr;

	static channel_t* limsPumpValveAddr;
	static channel_t* velValveAddr;
	static channel_t* currentValveAddr;
	static channel_t* limsFillValveAddr;

	static int firsttime = 1;

	if (firsttime) {
		posPotValveAddr = channels_find_by_name("potvalve_pos");
		statePotValveAddr = channels_find_by_name("potvalve_state");
		velPotValveAddr = channels_find_by_name("potvalve_vel");
		openCurPotValveAddr = channels_find_by_name("potvalve_I_open");
		closeCurPotValveAddr = channels_find_by_name("potvalve_I_close");

		limsPumpValveAddr = channels_find_by_name("pumpvalve_lims");
		velValveAddr = channels_find_by_name("valve_vel");
		currentValveAddr = channels_find_by_name("valve_current");
		limsFillValveAddr = channels_find_by_name("fillvalve_lims");
	}

	if (actuators_init & (0x1 << POTVALVE_NUM)) {
		SET_UINT16(posPotValveAddr, potvalve_data.adc[0]);
		SET_UINT16(statePotValveAddr, potvalve_data.current);
		SET_UINT16(velPotValveAddr, CommandData.Cryo.potvalve_vel);
		SET_UINT8(openCurPotValveAddr, CommandData.Cryo.potvalve_opencurrent);
		SET_UINT8(closeCurPotValveAddr, CommandData.Cryo.potvalve_closecurrent);
	}

	for (i = 0; i < NVALVES; i++) {
		if (actuators_init & (0x1 << valve_addr[i])) {
			if (i == 0) {
				SET_UINT8(limsPumpValveAddr, valve_data[0].limit);
			} else if (i == 1) {
				SET_UINT8(limsFillValveAddr, valve_data[1].limit);
			}
			valve_flag = 1;
		}
	}

	if (valve_flag == 1) {
		SET_UINT16(velValveAddr, CommandData.Cryo.valve_vel);
		SET_UINT8(currentValveAddr, CommandData.Cryo.valve_current);
	}
}
