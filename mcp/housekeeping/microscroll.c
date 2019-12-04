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
	uint16_t valve_dir_mb_addr[N_AALBORG_VALVES];
	float valve_speed;
} aalborg_control_t;

aalborg_control_t aalborg_data;

void ControlAalborg(int index)
{
	static int firsttime = 1;
	float labjack_ain[N_AALBORG_VALVES];
	static channel_t* labjackAinAddr[N_AALBORG_VALVES];
	char channel_name[128] = {0};
	float prev_speed;
	int i;
	int valve_timer = -1;

	// probably need some initialization things in whatever function inializes this labjack
	// and then we should check that it is initialized before calling this function
	if (firsttime) {
		firsttime = 0;
		// find the addresses for the channels we need to read the first time
		for (i = 0; i < N_AALBORG_VALVES; i++) {
			snprintf(channel_name, sizeof(channel_name), "ain_aalborg_valve_%d", i);
			labjackAinAddr[i] = channels_find_by_name(channel_name);
		}
	}

	// get labjack AIN values from the frame, store locally
	GET_SCALED_VALUE(labjackAinAddr[index], labjack_ain[index]);
	// get the current goal from the command struct
	aalborg_data.valve_goal[index] = CommandData.Cryo.aalborg_valve_goals[index];
	// set prev_speed so we can compare previous with current value in command struct
	prev_speed = aalborg_data.valve_speed;
	aalborg_data.valve_speed = CommandData.Cryo.aalborg_speed;

	// if the aalborg speed has been changed in commanding, we need to change it
	if (aalborg_data.valve_speed != prev_speed) {
		// labjack function to set the new speed goes here
	}

	// if the value on the AIN is high, the valve is closed
	if (labjack_ain[index] > AALBORG_HIGH_LEVEL) {
		aalborg_data.valve_state[index] = AALBORG_CLOSED;
	// if it is low, we are NOT closed (either open or moving in either direction)
	} else if (labjack_ain[index] < AALBORG_LOW_LEVEL) {
		// if the timer is still going, we are still closing
		if (valve_timer > 0) {
			aalborg_data.valve_state[index] = AALBORG_NOT_CLOSED;
		// but if it has reached zero, we should be open
		} else if (valve_timer == 0) {
			aalborg_data.valve_state[index] = AALBORG_OPENED;
		}
	// if the value on the AIN is in the middle, we are intermediate
	} else {
		aalborg_data.valve_state[index] = AALBORG_INTERMED;
	}

	// if the state of the current valve does not match the goal, we need to do something
	if (aalborg_data.valve_state[index] != aalborg_data.valve_goal[index]) {
		// if the goal is open and we got here, we know we aren't open
		// if we aren't already opening the valve, do it
		if ((aalborg_data.valve_goal[index] == AALBORG_OPENED) && (aalborg_data.valve_state[index] != AALBORG_OPENING)) {
			// set labjack output to open this valve

			// set state to opening
			aalborg_data.valve_state[index] = AALBORG_OPENING;
			// start a timer because the signal immediately tells us the valve is NOT closed
			// and we know it should take about 6 seconds to open, so set a 7 second timer
			valve_timer = AALBORG_WAIT_OPENING;
		}
		// if the goal is closed and we got here, we know we aren't closed
		// if we aren't already closing the valve, do it
		if ((aalborg_data.valve_goal[index] == AALBORG_CLOSED) && (aalborg_data.valve_state[index] != AALBORG_CLOSING)) {
			// set labjack output to close this valve

			// set state to closing
			aalborg_data.valve_state[index] = AALBORG_CLOSING;
			// set valve timer less than 0 so it doesn't become 0
			valve_timer = -1;
		}
	}
}
