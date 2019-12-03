/***************************************************************************
 mcp: the BLAST master control program
 
 This software is copyright (C) 2002-2006 University of Toronto
 
 This file is part of mcp.
 
 mcp is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 at your option) any later version.
 
 mcp is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with mcp; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 
 created by Ian Lowe 11-29-19
 **************************************************************************/


/*************************************************************************
 
 microscroll.c code to control the power relays and thermistors for the micro
 scroll pump
 
 *************************************************************************/

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "blast.h"
#include "microscroll.h"




typedef struct {
    float pump_1_on, pump_1_off, pump_2_on, pump_2_off;
    float aalborg_supply_on, aalborg_supply_off;
} microscroll_control_t;

typedef struct {
	uint16_t valve_state[N_AALBORG_VALVES];
	uint16_t valve_goal[N_AALBORG_VALVES];
} aalborg_control_t;

aalborg_control_t aalborg_data;

void ControlAalborg(int index)
{
	static int firsttime = 1;
	float labjack_ain[N_AALBORG_VALVES];
	static channel_t* labjackAinAddr[N_AALBORG_VALVES];
	char channel_name[128] = {0};
	int i;

	if (firsttime) {
		firsttime = 0;
		for (i = 0; i < N_AALBORG_VALVES; i++) {
			snprintf(channel_name, sizeof(channel_name), "state_aalborg_valve_%d", i);
			labjackAinAddr[i] = channels_find_by_name(channel_name);
		}
	}

	GET_SCALED_VALUE(labjackAinAddr[index], labjack_ain[index]);
	aalborg_data.valve_goal[index] = CommandData.Cryo.aalborg_valve_goals[index];

	if (labjack_ain[index] > AALBORG_CLOSE_LEVEL) {
		aalborg_data.valve_state[index] = AALBORG_CLOSED;
	} else if (labjack_ain[index] < AALBORG_OPEN_LEVEL) {
		aalborg_data.valve_state[index] = AALBORG_OPENED;
	}

	// if the state of the current valve does not match the goal, we need to do something
	if (aalborg_data.valve_state[index] != aalborg_data.valve_goal[index]) {
		// if the goal is open and we got here, we know we aren't open
		// if we aren't already opening the valve, do it
		if ((aalborg_data.valve_goal[index] == AALBORG_OPENED) && (aalborg_data.valve_state[index] != AALBORG_OPENING)) {
			// set labjack output to open this valve
			aalborg_data.valve_state[index] = AALBORG_OPENING;
		}
		// if the goal is closed and we got here, we know we aren't closed
		// if we aren't already closing the valve, do it
		if ((aalborg_data.valve_goal[index] == AALBORG_CLOSED) && (aalborg_data.valve_state[index] != AALBORG_CLOSING)) {
			// set labjack output to close this valve
			aalborg_data.valve_state[index] = AALBORG_CLOSING;
		}
	}
}
