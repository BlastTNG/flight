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

extern labjack_state_t state[NUM_LABJACKS];

typedef struct {
    float supply_24va, supply_24vb;
    float relay_12v_on, relay_12v_off;
    int have_pulsed_relay;
} microscroll_t;

microscroll_t microscroll;

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


static void clear_fio() {
    static int first_time = 1;
    if (first_time && state[9].connected) {
        first_time = 0;
        labjack_queue_command(LABJACK_MICROSCROLL, 2000, 0);
        labjack_queue_command(LABJACK_MICROSCROLL, 2001, 0);
        labjack_queue_command(LABJACK_MICROSCROLL, 2006, 0);
        labjack_queue_command(LABJACK_MICROSCROLL, 2007, 0);
    }
}

static void publish_values() {
    static int first_time = 1;
    static channel_t* pumpa_Addr;
    static channel_t* pumpb_Addr;
    if (first_time) {
        pumpa_Addr = channels_find_by_name("microscroll_a");
        pumpb_Addr = channels_find_by_name("microscroll_b");
        first_time = 0;
    }
    if (state[9].connected) {
        SET_SCALED_VALUE(pumpa_Addr, microscroll.supply_24va);
        SET_SCALED_VALUE(pumpb_Addr, microscroll.supply_24vb);
    }
}

static void update_microscroll() {
    microscroll.supply_24va = CommandData.Microscroll.supply_24va;
    microscroll.supply_24vb = CommandData.Microscroll.supply_24vb;
    microscroll.relay_12v_on = CommandData.Microscroll.relay_12v_on;
    microscroll.relay_12v_off = CommandData.Microscroll.relay_12v_off;
}
// add protection below here to not send these things continuously
static void control_24va_supply() {
    if (state[9].connected) {
        labjack_queue_command(LABJACK_MICROSCROLL, supply_24Va, microscroll.supply_24va);
    }
}

static void control_12v_relay() {
    static int have_pulsed = 0;
    if (state[9].connected) {
        if (have_pulsed) {
            labjack_queue_command(LABJACK_MICROSCROLL, relay_12V_on, 0);
            labjack_queue_command(LABJACK_MICROSCROLL, relay_12V_off, 0);
        }
        if (microscroll.relay_12v_on) {
            CommandData.Microscroll.relay_12v_on = 0;
            have_pulsed = 1;
            labjack_queue_command(LABJACK_MICROSCROLL, relay_12V_on, 1);
        }
        if (microscroll.relay_12v_on) {
            CommandData.Microscroll.relay_12v_off = 0;
            have_pulsed = 1;
            labjack_queue_command(LABJACK_MICROSCROLL, relay_12V_off, 1);
        }
    }
}

static void control_24vb_supply() {
    if (state[9].connected) {
        labjack_queue_command(LABJACK_MICROSCROLL, supply_24Vb, microscroll.supply_24vb);
    }
}

static void update_microscroll_thermistors() {
    static int first_time = 1;
    static channel_t* therm1_Addr;
    static channel_t* therm2_Addr;
    static channel_t* therm3_Addr;
    static channel_t* therm4_Addr;
    static channel_t* therm5_Addr;
    static channel_t* therm6_Addr;
    static channel_t* therm7_Addr;
    static channel_t* therm8_Addr;
    if (first_time) {
        therm1_Addr = channels_find_by_name("micro_thermistor1");
        therm2_Addr = channels_find_by_name("micro_thermistor2");
        therm3_Addr = channels_find_by_name("micro_thermistor3");
        therm4_Addr = channels_find_by_name("micro_thermistor4");
        therm5_Addr = channels_find_by_name("micro_thermistor5");
        therm6_Addr = channels_find_by_name("micro_thermistor6");
        therm7_Addr = channels_find_by_name("micro_thermistor7");
        therm8_Addr = channels_find_by_name("micro_thermistor8");
        first_time = 0;
    }
    if (state[9].connected) {
        SET_SCALED_VALUE(therm1_Addr, labjack_get_value(LABJACK_MICROSCROLL,thermistor_1));
        SET_SCALED_VALUE(therm2_Addr, labjack_get_value(LABJACK_MICROSCROLL,thermistor_2));
        SET_SCALED_VALUE(therm3_Addr, labjack_get_value(LABJACK_MICROSCROLL,thermistor_3));
        SET_SCALED_VALUE(therm4_Addr, labjack_get_value(LABJACK_MICROSCROLL,thermistor_4));
        SET_SCALED_VALUE(therm5_Addr, labjack_get_value(LABJACK_MICROSCROLL,thermistor_5));
        SET_SCALED_VALUE(therm6_Addr, labjack_get_value(LABJACK_MICROSCROLL,thermistor_6));
        SET_SCALED_VALUE(therm7_Addr, labjack_get_value(LABJACK_MICROSCROLL,thermistor_7));
        SET_SCALED_VALUE(therm8_Addr, labjack_get_value(LABJACK_MICROSCROLL,thermistor_8));
    }
}
