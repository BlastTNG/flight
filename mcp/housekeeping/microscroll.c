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
 scroll pump, and control aalborg valves from the labjack
 
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
    float supply_12v;
    int have_pulsed_relay;
} microscroll_t;

microscroll_t microscroll;

typedef struct {
	float dir;
	float prev_dir;
	// valve_dir_addr is the modbus address used to set the direction for each valve
	uint16_t valve_dir_addr;
	// the speed the valve should move at, 0-2.5
	float valve_speed;
	float prev_speed;
	// opening timer for the one aalborg we are actually using
	// int16_t timer;
	// modbus register on labjack for speed control
	uint16_t speed_addr;
    // timeout for setting speed back to zero
} aalborg_control_t;

aalborg_control_t aalborg_data[N_AALBORG_VALVES] = {{0}};

void ControlAalborg(int index)
{
	static int firsttime = 1;
	// static channel_t* labjackAinAddr[N_AALBORG_VALVES];
	// char channel_name[128] = {0};
	int i;

	// if we aren't connected, return before doing anything
	if (!state[LABJACK_MICROSCROLL].connected) {
		// want to re-run firsttime stuff if we loose connection
		firsttime = 1;
		return;
	}

	// do this stuff the first time we are connected
	if (firsttime) {
		firsttime = 0;
		// store the valve direction modbus addresses in the struct
		aalborg_data[0].valve_dir_addr = VALVE1_DIR;
		aalborg_data[1].valve_dir_addr = VALVE2_DIR;
		aalborg_data[2].valve_dir_addr = VALVE3_DIR;
		// store the valve speed modbus addresses in the struct
		aalborg_data[0].speed_addr = SPEED_1_REG;
		aalborg_data[1].speed_addr = SPEED_2_REG;
		aalborg_data[2].speed_addr = SPEED_3_REG;

		// find the addresses for the channels we need to read the first time
		for (i = 0; i < N_AALBORG_VALVES; i++) {
			// snprintf(channel_name, sizeof(channel_name), "ain_%d_aalborg", i+1);
			// labjackAinAddr[i] = channels_find_by_name(channel_name);
			// clear the command struct goals
			// CommandData.Aalborg.goal[i] = 0;
			// set the timers so they don't change
			// aalborg_data[i].timer = -1;
            CommandData.Aalborg.timeout[i] = 0;
			// set speed to zero the first time
        	labjack_queue_command(LABJACK_MICROSCROLL, aalborg_data[i].speed_addr, 0.0);
		}
	}

	// get labjack AIN values from the frame, store in struct
	// aalborg_data[index].labjack_ain = GET_FLOAT(labjackAinAddr[index]);
	// set prev_speed so we can compare previous with current value in command struct
	aalborg_data[index].prev_speed = aalborg_data[index].valve_speed;

    // count timeout (in seconds) before setting the speed to zero
    // if timeout < 0, then there the speed never times out
	// aalborg_data[index].speed_timeout = CommandData.Aalborg.timeout[index];

    if (CommandData.Aalborg.timeout[index] == 0) {
	   aalborg_data[index].valve_speed = 0;
    } else {
	    aalborg_data[index].valve_speed = CommandData.Aalborg.speed[index];
    }
    // decrement counter
    if (CommandData.Aalborg.timeout[index] > 0) {
        CommandData.Aalborg.timeout[index]--;
    }

	// if the aalborg speed has been changed in commanding, we need to change it
	if (aalborg_data[index].valve_speed != aalborg_data[index].prev_speed) {
		// set the new speed on the labjack DAC
        labjack_queue_command(LABJACK_MICROSCROLL, aalborg_data[index].speed_addr, aalborg_data[index].valve_speed);
	}

	// set prev_dir so we can compare previous with current
	aalborg_data[index].prev_dir = aalborg_data[index].dir;
	// get the current dir from the command struct
	aalborg_data[index].dir = CommandData.Aalborg.dir[index];

	// if the aalborg direction has changed in commanding, we need to set the labjack output
	if (aalborg_data[index].dir != aalborg_data[index].prev_dir) {
		// set the direction on the labjack TDAC
        labjack_queue_command(LABJACK_MICROSCROLL, aalborg_data[index].valve_dir_addr, aalborg_data[index].dir);
	}

	/* // if the valve timer is active, decrement it
	if (aalborg_data[index].timer > 0) {
		aalborg_data[index].timer--;
	}

	// if the value on the AIN is high, the valve is closed
	if (aalborg_data[index].labjack_ain > AALBORG_HIGH_LEVEL) {
		// set CLOSED bit, and clear all others (shouldn't be moving now)
		aalborg_data[index].valve_state |= AALBORG_CLOSED;
		// clear opened bit
		aalborg_data[index].valve_state &= ~AALBORG_OPENED;
	// if it is low, we are NOT closed (either open or moving in either direction)
	} else if (aalborg_data[index].labjack_ain < AALBORG_LOW_LEVEL) {
		// low level means we are NOT closed
		aalborg_data[index].valve_state |= AALBORG_NOT_CLOSED;
		// but if it has reached zero, we should be open
		if (aalborg_data[index].timer == 0 && (aalborg_data[index].valve_state & AALBORG_OPENING)) {
			// clear opening bit, we should be done moving now
			aalborg_data[index].valve_state &= ~AALBORG_OPENING;
			// set OPENED bit, leaving others (like NOT closed)
			aalborg_data[index].valve_state |= AALBORG_OPENED;
		}
	} else {
		// the output from the valve is digital, should never be neither high nor low
		// this shouldn't happen...
		aalborg_data[index].valve_state |= AALBORG_UNK;
	}

	// if the state of the current valve does not match the goal, we need to do something
	if (~(aalborg_data[index].valve_state & aalborg_data[index].valve_goal)) {
		// if the goal is open and we got here, we know we aren't open
		// if we aren't already opening the valve, do it
		if ((aalborg_data[index].valve_goal & AALBORG_OPENED) & ~(aalborg_data[index].valve_state & AALBORG_OPENING)) {
			// set labjack output to open this valve
        	labjack_queue_command(LABJACK_MICROSCROLL, aalborg_data[index].valve_dir_addr, AALBORG_OPEN_CMD);
			// set state to opening
			aalborg_data[index].valve_state |= AALBORG_OPENING;
			// clear the closed bit
			aalborg_data[index].valve_state &= ~AALBORG_CLOSED;
			// start a timer because the signal immediately tells us the valve is NOT closed
			// and we know it should take about 11 seconds to open, so set a 12 second timer
			aalborg_data[index].timer = AALBORG_WAIT_OPENING;
		}
		// if the goal is closed and we got here, we know we aren't closed
		// if we aren't already closing the valve, do it
		if ((aalborg_data[index].valve_goal & AALBORG_CLOSED) & ~(aalborg_data[index].valve_state & AALBORG_CLOSING)) {
			// set labjack output to close this valve
        	labjack_queue_command(LABJACK_MICROSCROLL, aalborg_data[index].valve_dir_addr, AALBORG_CLOSE_CMD);
			// set state to closing
			aalborg_data[index].valve_state |= AALBORG_CLOSING;
			// clear the opened bit
			aalborg_data[index].valve_state &= ~AALBORG_OPENED;
			// set valve timer less than 0 so it doesn't become 0
			aalborg_data[index].timer = -1;
		}
	} */
}

void TestLjWrites()
{
	if (state[LABJACK_MICROSCROLL].connected) {
		if (CommandData.Aalborg.new_cmd) {
			// blast_info("new_cmd on labjack10, queueing command now");
			// blast_info("register=%d, value=%f", CommandData.Aalborg.reg, CommandData.Aalborg.value);
			CommandData.Aalborg.new_cmd = 0;
			labjack_queue_command(LABJACK_MICROSCROLL, CommandData.Aalborg.reg, CommandData.Aalborg.value);
		}
	}
}

void WriteAalborgs()
{
	static int first_time = 1;

	static channel_t* aalborg1SpeedAddr;
	static channel_t* aalborg2SpeedAddr;
	static channel_t* aalborg3SpeedAddr;
	static channel_t* aalborg1GoalAddr;
	static channel_t* aalborg2GoalAddr;
	static channel_t* aalborg3GoalAddr;
	static channel_t* ainAalborg1Addr;
	static channel_t* ainAalborg2Addr;
	static channel_t* ainAalborg3Addr;
	static channel_t* timerAalborg1Addr;
	static channel_t* timerAalborg2Addr;
	static channel_t* timerAalborg3Addr;

	if (first_time) {
		first_time = 0;
		aalborg1SpeedAddr = channels_find_by_name("speed_1_aalborg");
		aalborg2SpeedAddr = channels_find_by_name("speed_2_aalborg");
		aalborg3SpeedAddr = channels_find_by_name("speed_3_aalborg");
		aalborg1GoalAddr = channels_find_by_name("goal_1_aalborg");
		aalborg2GoalAddr = channels_find_by_name("goal_2_aalborg");
		aalborg3GoalAddr = channels_find_by_name("goal_3_aalborg");
		timerAalborg1Addr = channels_find_by_name("timer_1_aalborg");
		timerAalborg2Addr = channels_find_by_name("timer_2_aalborg");
		timerAalborg3Addr = channels_find_by_name("timer_3_aalborg");
		ainAalborg1Addr = channels_find_by_name("ain_1_aalborg");
		ainAalborg2Addr = channels_find_by_name("ain_2_aalborg");
		ainAalborg3Addr = channels_find_by_name("ain_3_aalborg");
	}

	// only write any of these channels if we are connected to the labjack
	if (state[LABJACK_MICROSCROLL].connected) {
		SET_FLOAT(aalborg1GoalAddr, aalborg_data[0].dir);
		SET_FLOAT(aalborg2GoalAddr, aalborg_data[1].dir);
		SET_FLOAT(aalborg3GoalAddr, aalborg_data[2].dir);
		SET_INT16(timerAalborg1Addr, CommandData.Aalborg.timeout[0]);
		SET_INT16(timerAalborg2Addr, CommandData.Aalborg.timeout[1]);
		SET_INT16(timerAalborg3Addr, CommandData.Aalborg.timeout[2]);
		SET_FLOAT(aalborg1SpeedAddr, aalborg_data[0].valve_speed);
		SET_FLOAT(aalborg2SpeedAddr, aalborg_data[1].valve_speed);
		SET_FLOAT(aalborg3SpeedAddr, aalborg_data[2].valve_speed);
    	SET_FLOAT(ainAalborg1Addr, labjack_get_value(LABJACK_MICROSCROLL, VALVE_1_STATUS));
    	SET_FLOAT(ainAalborg2Addr, labjack_get_value(LABJACK_MICROSCROLL, VALVE_2_STATUS));
    	SET_FLOAT(ainAalborg3Addr, labjack_get_value(LABJACK_MICROSCROLL, VALVE_3_STATUS));
	}
}

static void clear_fio() {
    static int first_time = 1;
    if (first_time && state[9].connected) {
        first_time = 0;
        labjack_queue_command(LABJACK_MICROSCROLL, 2000, 1);
        labjack_queue_command(LABJACK_MICROSCROLL, 2006, 0);
        labjack_queue_command(LABJACK_MICROSCROLL, 2007, 1);
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
    // microscroll.relay_12v_on = CommandData.Microscroll.relay_12v_on;
    // microscroll.relay_12v_off = CommandData.Microscroll.relay_12v_off;
    microscroll.supply_12v = CommandData.Microscroll.supply_12v;
}
// add protection below here to not send these things continuously

static void control_24va_supply() {
    static float prev_status = 0;
    if (state[9].connected && prev_status != microscroll.supply_24va) {
        labjack_queue_command(LABJACK_MICROSCROLL, supply_24Va, microscroll.supply_24va);
        prev_status = microscroll.supply_24va;
    }
}

// Deprecated with moving to science stack.

// We need to move to something like controlling the none
// of the relays, but instead putting an enable/disable value written directly
// to the pins on the vicor, the SS will instead control the
// relays.
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

// Now controls the aalborg
static void control_24vb_supply() {
    static float prev_status = 1;
    if (state[9].connected && prev_status != microscroll.supply_24vb) {
        prev_status = microscroll.supply_24vb;
        labjack_queue_command(LABJACK_MICROSCROLL, supply_24Vb, microscroll.supply_24vb);
    }
}

static void control_12v_supply() {
    static float prev_status = 1;
    if (state[9].connected && prev_status != microscroll.supply_12v) {
        prev_status = microscroll.supply_12v;
        labjack_queue_command(LABJACK_MICROSCROLL, relay_12V_on, microscroll.supply_12v);
    }
}


static void start_up_defaults() {
    static int first_time = 1;
    if (state[9].connected && first_time) {
        labjack_queue_command(LABJACK_MICROSCROLL, supply_24Va, 0);
        labjack_queue_command(LABJACK_MICROSCROLL, supply_24Vb, 1);
        labjack_queue_command(LABJACK_MICROSCROLL, relay_12V_on, 1);
        first_time = 0;
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
        therm1_Addr = channels_find_by_name("micro_thermistor_1");
        therm2_Addr = channels_find_by_name("micro_thermistor_2");
        therm3_Addr = channels_find_by_name("micro_thermistor_3");
        therm4_Addr = channels_find_by_name("micro_thermistor_4");
        therm5_Addr = channels_find_by_name("micro_thermistor_5");
        therm6_Addr = channels_find_by_name("micro_thermistor_6");
        therm7_Addr = channels_find_by_name("micro_thermistor_7");
        therm8_Addr = channels_find_by_name("micro_thermistor_8");
        first_time = 0;
    }
    if (state[9].connected) {
        SET_SCALED_VALUE(therm1_Addr, labjack_get_value(LABJACK_MICROSCROLL, thermistor_1));
        SET_SCALED_VALUE(therm2_Addr, labjack_get_value(LABJACK_MICROSCROLL, thermistor_2));
        SET_SCALED_VALUE(therm3_Addr, labjack_get_value(LABJACK_MICROSCROLL, thermistor_3));
        SET_SCALED_VALUE(therm4_Addr, labjack_get_value(LABJACK_MICROSCROLL, thermistor_4));
        SET_SCALED_VALUE(therm5_Addr, labjack_get_value(LABJACK_MICROSCROLL, thermistor_5));
        SET_SCALED_VALUE(therm6_Addr, labjack_get_value(LABJACK_MICROSCROLL, thermistor_6));
        SET_SCALED_VALUE(therm7_Addr, labjack_get_value(LABJACK_MICROSCROLL, thermistor_7));
        SET_SCALED_VALUE(therm8_Addr, labjack_get_value(LABJACK_MICROSCROLL, thermistor_8));
    }
}


void execute_microscroll_functions() {
    update_microscroll();
    clear_fio();
    start_up_defaults();
    update_microscroll_thermistors();
    control_12v_supply();
    control_24vb_supply();
    control_24va_supply();
}
