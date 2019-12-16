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

#define N_PUMP_VALVES 2 // valves on microscroll pumps, don't count pot valve here

/* this ratio is wrong! Plotting encoder counts vs micro-steps gives 230-250, but
 * it is working so we will leave it for now. If this were changed, the thresholds
 * and maybe the 750000 (microsteps to overshoot closed) would need to be changed
 * and we don't have time to do that before flight
 * -PAW and PCA 2018/12/16
 */
#define POTVALVE_STEP_ADC_RATIO 250 // estimated empirically -PCA
#define MOVE_OVERSHOOT_FACTOR 1.1

typedef enum {
	no_move = 0, opening, closing, tighten, valve_stop
} valve_move_type_t;

static struct potvalve_struct {
	int init;
	char addr;
	int pos;
	int adc[4];
	int moving;
	int open_i;
	int close_i;
	int do_move;
	int on;
	valve_state_t state, goal;
	valve_state_t prev_state, prev_goal;
	valve_move_type_t potvalve_move;
} potvalve_data;

static struct valve_struct {
	char addr;
	int pos;
	int limit;
	int ready;
	int stop;
	valve_state_t goal;
	int move_size;
} valve_data[N_PUMP_VALVES];

void DoCryovalves(struct ezbus* bus, unsigned int actuators_init)
{
	// blast_info("starting DoCryovalves"); // DEBUG PAW
	int i;
	int valve_addr[N_PUMP_VALVES] = {PUMP1_VALVE_NUM, PUMP2_VALVE_NUM};

	if (actuators_init & (0x1 << POTVALVE_NUM)) {
		// blast_info("calling DoPotValve"); // DEBUG PAW
		DoPotValve(bus);
	}

	for (i = 0; i < N_PUMP_VALVES; i++) {
		if (actuators_init & (0x1 << valve_addr[i])) {
		// blast_info("calling DoValves"); // DEBUG PAW
			DoValves(bus, i, valve_addr[i]);
		}
	}
	// blast_info("calling WriteValves"); // DEBUG PAW
	WriteValves(actuators_init, valve_addr);
	// blast_info("called WriteValves"); // DEBUG PAW
}

void DoValves(struct ezbus* bus, int index, char addr)
{
	static int firsttime_pump_1_valve = 1;
	static int firsttime_pump_2_valve = 1;

	if (firsttime_pump_1_valve && (index == 0)) {
		valve_data[index].addr = (char) GetActAddr(addr);
		// Debug PAW 04/24/2018
		// blast_info("Valve %d address is %c (firsttime loop)", index, valve_data[index].addr);

		EZBus_Take(bus, valve_data[index].addr);
		blast_info("Making sure Valve %d is not running on startup", index);
		EZBus_Stop(bus, valve_data[index].addr);
		EZBus_MoveComm(bus, valve_data[index].addr, PUMP_VALVES_PREAMBLE);
		EZBus_Release(bus, valve_data[index].addr);
		// CommandData.Cryo.valve_goals[index] = 0;
		// set the default move size for valve A
		valve_data[index].move_size = 800000;
		firsttime_pump_1_valve = 0;
	}

	if (firsttime_pump_2_valve && (index == 1)) {
		valve_data[index].addr = (char) GetActAddr(addr);
		// Debug PAW 04/24/2018
		// blast_info("Valve %d address is %c (firsttime loop)", index, valve_data[index].addr);

		EZBus_Take(bus, valve_data[index].addr);
		blast_info("Making sure Valve %d is not running on startup", index);
		EZBus_Stop(bus, valve_data[index].addr);
		EZBus_MoveComm(bus, valve_data[index].addr, PUMP_VALVES_PREAMBLE);
		EZBus_Release(bus, valve_data[index].addr);
		// CommandData.Cryo.valve_goals[index] = 0;
		// set the default move size for valve B
		valve_data[index].move_size = 900000;
		firsttime_pump_2_valve = 0;
	}
	// blast_info("Valve %d address is %c", index, valve_data[index].addr);

	valve_data[index].goal = CommandData.Cryo.valve_goals[index];
	valve_data[index].stop = CommandData.Cryo.valve_stop[index];
	if (valve_data[index].stop) valve_data[index].goal = 0;
	EZBus_SetVel(bus, valve_data[index].addr, CommandData.Cryo.valve_vel);
	EZBus_SetIMove(bus, valve_data[index].addr, CommandData.Cryo.valve_move_i);
	EZBus_SetIHold(bus, valve_data[index].addr, CommandData.Cryo.valve_hold_i);
	EZBus_SetAccel(bus, valve_data[index].addr, CommandData.Cryo.valve_acc);

	// Debug PAW 04/24/2018
	// blast_info("commanded valve velocity is %d", CommandData.Cryo.valve_vel);
	// blast_info("actual valve 9 velocity is %d", bus->stepper[8].vel);
	// blast_info("actual valve 10 velocity is %d", bus->stepper[9].vel);
	// blast_info("commanded valve current is %d", CommandData.Cryo.valve_move_i);
	// blast_info("actual valve 9 current is %d", bus->stepper[8].imove);
	// blast_info("actual valve 10 current is %d", bus->stepper[9].imove);


	// ?4 returns status of all 4 inputs, Bit 2 = opto 1, Bit 3 = opto 2
	EZBus_ReadInt(bus, valve_data[index].addr, "?4", &(valve_data[index].limit));
	EZBus_ReadInt(bus, valve_data[index].addr, "?0", &(valve_data[index].pos));
	// blast_info("limit switch for %c = %d", valve_data[index].addr, valve_data[index].limit);
	valve_data[index].ready = !(EZBus_IsBusy(bus, valve_data[index].addr));

	if ((valve_data[index].goal == opened) && (valve_data[index].limit != 11)) {
		if (valve_data[index].ready) {
			EZBus_Take(bus, valve_data[index].addr);
			EZBus_Stop(bus, valve_data[index].addr);
			// EZBus_RelMove(bus, valve_data[index].addr, INT_MAX);
			EZBus_RelMove(bus, valve_data[index].addr, valve_data[index].move_size);
			EZBus_Release(bus, valve_data[index].addr);
			blast_info("Starting to open Valve %d", index); // debug PAW
		} else {
			blast_info("Valve %d opening...", index);
		}
	} else if ((valve_data[index].goal == closed) && (valve_data[index].limit != 7)) {
		if (valve_data[index].ready) {
			EZBus_Take(bus, valve_data[index].addr);
			EZBus_Stop(bus, valve_data[index].addr);
			// EZBus_RelMove(bus, valve_data[index].addr, INT_MIN);
			EZBus_RelMove(bus, valve_data[index].addr, (-1)*valve_data[index].move_size);
			EZBus_Release(bus, valve_data[index].addr);
                        blast_info("Starting to close Valve %d", index); // debug PAW
		} else {
			blast_info("Valve %d closing...", index);
		}
	} else if (valve_data[index].stop) {
	       EZBus_Stop(bus, valve_data[index].addr);
	       // valve_data[index].stop = 0;
	       // CommandData.Cryo.valve_stop[index] = 0;
	} else if (valve_data[index].limit == 7 || valve_data[index].limit == 11) {
		valve_data[index].goal = 0;
		CommandData.Cryo.valve_goals[index] = 0;
	}
}

void DoPotValve(struct ezbus* bus)
{
	static int firsttime = 1;
	// static int pot_init = 0;
	static int tight_flag;
	valve_state_t prev_goal;
	int delta = 0;
	int new_goal;
	// int firstmove;
	int newstate;
	// int do_move;
	// char buffer[EZ_BUS_BUF_LEN];

	// blast_info("Starting DoPotValve"); // DEBUG PAW

	if (firsttime) {
		potvalve_data.init = 0;
		blast_info("IN FIRSTTIME BLOCK"); // DEBUG PCA
		potvalve_data.addr = (char) GetActAddr(POTVALVE_NUM);

		EZBus_Take(bus, potvalve_data.addr);
		blast_info("Making sure the potvalve is not running on startup.");
		EZBus_Stop(bus, potvalve_data.addr);
	    EZBus_SetAccel(bus, potvalve_data.addr, 1000);
		// I don't think there is any point to this PAW 2018/06/20
		// EZBus_MoveComm(bus, potvalve_data.addr, POTVALVE_PREAMBLE);
		EZBus_Release(bus, potvalve_data.addr);


		potvalve_data.pos = 0;
		potvalve_data.moving = 0;
		potvalve_data.state = 0;
		potvalve_data.goal = 0;
		potvalve_data.potvalve_move = 0;
		newstate = 1;
		// firstmove = 1;
		tight_flag = 1;

		// this command always returns an error I think because it is supposed to be
		// sent only after an A command (or other move?) also does nothing
		// if(EZBus_Comm(bus, potvalve_data.addr, "z0R") != EZ_ERR_OK)
		// 	bputs(info, "Error initializing valve position");
		firsttime = 0;
		potvalve_data.init = 1;
	}

	// blast_info("past firsttime loop"); // DEBUG PAW
	// blast_info("firstmove = %d", firstmove); // DEBUG PAW

	prev_goal = potvalve_data.goal;
	potvalve_data.on = CommandData.Cryo.potvalve_on;
	EZBus_SetVel(bus, potvalve_data.addr, CommandData.Cryo.potvalve_vel);
	EZBus_SetIHold(bus, potvalve_data.addr, CommandData.Cryo.potvalve_hold_i);

	if (CommandData.Cryo.potvalve_goal == opened) {
		potvalve_data.goal = CommandData.Cryo.potvalve_goal;
		// blast_info("set goal open"); // DEBUG PAW
	} else if (CommandData.Cryo.potvalve_goal == closed) {
		if ((potvalve_data.state == opened) || (potvalve_data.state == intermed)) {
			potvalve_data.goal = loose_closed;
			// blast_info("set goal loose_closed"); // DEBUG PAW
		} else {
			potvalve_data.goal = closed;
			// blast_info("set goal closed"); // DEBUG PAW
		}
	}


	// blast_info("about to call GetPotValvePos"); // DEBUG PAW
	GetPotValvePos(bus);
	// blast_info("about to call SetValveState, newstate = %d", newstate); // DEBUG PAW
	newstate = SetValveState(tight_flag);
	// blast_info("called SetValveState, newstate = %d", newstate); // DEBUG PAW

	// if (newstate) blast_info("POT VALVE NEW STATE"); // DEBUG PCA

	if (potvalve_data.state == potvalve_data.goal) {
		potvalve_data.potvalve_move = no_move;
		potvalve_data.moving = 0;
		// blast_info("current = goal, so no move"); // DEBUG PAW

		if (potvalve_data.state == loose_closed) {
			blast_info("RESET GOAL CLOSED"); // DEBUG PCA
			potvalve_data.goal = closed;
			potvalve_data.potvalve_move = tighten;
			// blast_info("goal was loose_closed, now is closed"); // DEBUG PAW
		}
	} else {
		if (potvalve_data.moving != 1) {
			// blast_info("not currently moving (according to pot valve struct)"); // DEBUG PAW
			if (potvalve_data.state != opened && potvalve_data.goal == opened) {
				potvalve_data.potvalve_move = opening;
				// blast_info("move type set to opening"); // DEBUG PAW
			} else if (potvalve_data.state != closed && (potvalve_data.goal == closed || potvalve_data.goal == loose_closed)) {
				potvalve_data.potvalve_move = closing;
				// blast_info("move type set to closing"); // DEBUG PAW
			}
		}
	}

	new_goal = !(prev_goal == potvalve_data.goal);
	// blast_info("prev_goal = %d, current goal = %d", prev_goal, potvalve_data.goal); // DEBUG PAW
	// blast_info("compared previous and current goal, new_goal=%d", new_goal); // DEBUG PAW

	potvalve_data.do_move = (newstate || new_goal);
	// blast_info("Pot Valve do_move = %d", potvalve_data.do_move); // DEBUG PAW
	// firstmove = 0;
	// blast_info("firstmove = %d", firstmove); // DEBUG PAW

	if (potvalve_data.on == 0) {
	    // this if statement overrides everything else if the potvalve is turned off
	    potvalve_data.do_move = 1;
	    potvalve_data.potvalve_move = valve_stop;
	    potvalve_data.goal = 0;
	}

	if (potvalve_data.do_move) {
		// enable this for debugging
		// bus->chatter = EZ_CHAT_BUS;
	switch (potvalve_data.potvalve_move) {
		case(valve_stop):
	    	if (EZBus_Stop(bus, potvalve_data.addr) == EZ_ERR_OK) potvalve_data.moving = 0;
			break;
		case(no_move):
			// blast_info("in case no_move"); // DEBUG PAW
                        // don't stop if closed, want to hit torque limit
			if (potvalve_data.state != closed) EZBus_Stop(bus, potvalve_data.addr);
			// blast_info("called EZBus_Stop"); // DEBUG PAW
			potvalve_data.moving = 0;
			break;
		case(opening):
			// blast_info("in case opening"); // DEBUG PAW
			EZBus_SetIMove(bus, potvalve_data.addr, CommandData.Cryo.potvalve_opencurrent);
			// blast_info("called EZBus_SetIMove"); // DEBUG PAW
			delta = (int) (POTVALVE_STEP_ADC_RATIO *
				MIN(((CommandData.Cryo.potvalve_open_threshold - potvalve_data.adc[0]) * MOVE_OVERSHOOT_FACTOR),
				(POTVALVE_ENC_MAX - potvalve_data.adc[0])));
			if(EZBus_RelMove(bus, potvalve_data.addr, delta) != EZ_ERR_OK)
                        	blast_info("Error opening pot valve");
            // blast_info("potvalve_open: called EZBus_RelMove"); // DEBUG PAW
			potvalve_data.moving = 1;
			tight_flag = 0;
			break;
		case(closing):
			// blast_info("in case closing"); // DEBUG PAW
			EZBus_SetIMove(bus, potvalve_data.addr, CommandData.Cryo.potvalve_closecurrent);
			// blast_info("called EZBus_SetIMove"); // DEBUG PAW
			delta = (int) (-1 * POTVALVE_STEP_ADC_RATIO *
				MIN(((potvalve_data.adc[0] - CommandData.Cryo.potvalve_lclosed_threshold) * MOVE_OVERSHOOT_FACTOR),
				(potvalve_data.adc[0] - POTVALVE_ENC_MIN)));
			if(EZBus_RelMove(bus, potvalve_data.addr, delta) != EZ_ERR_OK)
				blast_info("Error closing pot valve");
            // blast_info("potvalve_close: called EZBus_RelMove"); // DEBUG PAW
			potvalve_data.moving = 1;
			break;
		case(tighten):
			// blast_info("in case tighten"); // DEBUG PAW
			if(potvalve_data.moving == 0) { // if we're just starting to tighten
				GetPotValvePos(bus); // check location before tightening
				delta = (int)(-1 * POTVALVE_STEP_ADC_RATIO *
					MAX((potvalve_data.adc[0] - CommandData.Cryo.potvalve_closed_threshold), 0) -
				 	(CommandData.Cryo.potvalve_min_tighten_move * POTVALVE_STEP_ADC_RATIO));

				// blast_info("after checking that we aren't moving yet"); // DEBUG PCA
				// blast_info("tighten current: %d; goal: %d", potvalve_data.state, potvalve_data.goal);
				EZBus_Stop(bus, potvalve_data.addr); // make sure we're actually stopped
				// blast_info("called EZBus_Stop"); // DEBUG PAW
				EZBus_SetIMove(bus, potvalve_data.addr, CommandData.Cryo.potvalve_closecurrent);
				// blast_info("called EZBus_SetIMove"); // DEBUG PAW
				// usleep(500000);
				if(EZBus_RelMove(bus, potvalve_data.addr, delta) != EZ_ERR_OK) // close by ~.5 turn
					blast_info("Error tightening pot valve");
				blast_info("command tighten move of size %d, ADC = %d", delta, delta/POTVALVE_STEP_ADC_RATIO); // DEBUG PCA
				tight_flag = 1;
				potvalve_data.moving = 1; // so command only gets sent once
			} else {
				blast_info("tighten, move != 0");
				if (!(EZBus_IsBusy(bus, potvalve_data.addr))) potvalve_data.moving = 0;
			}
			break;
	} // end switch
		// bus->chatter = ACTBUS_CHATTER;
	} // end if(do_move)


	// blast_info("pos: %d, current: %d, goal: %d, moving: %d, tight: %d",
	// potvalve_data.adc[0], potvalve_data.state, potvalve_data.goal, potvalve_data.moving, tight_flag); // DEBUG PAW

	usleep(10000);
}
void GetPotValvePos(struct ezbus* bus)
{
	EZBus_ReadInt(bus, potvalve_data.addr, "?0", &potvalve_data.pos);
	EZBus_Comm(bus, potvalve_data.addr, "?aa");
	sscanf(bus->buffer, "%i,%i,%i,%i", &potvalve_data.adc[0], &potvalve_data.adc[1],
		&potvalve_data.adc[2], &potvalve_data.adc[3]);
	// blast_info("Pot Valve encoder position is %i", potvalve_data.adc[0]); // DEBUG PAW
}


int SetValveState(int tight_flag)
{
	int retval;

	if ((potvalve_data.adc[0] <= CommandData.Cryo.potvalve_closed_threshold) && (tight_flag == 1)) {
		potvalve_data.state = closed;
	} else if (potvalve_data.adc[0] <= CommandData.Cryo.potvalve_lclosed_threshold) {
		potvalve_data.state = loose_closed;
	} else if (potvalve_data.adc[0] >= CommandData.Cryo.potvalve_open_threshold) {
		potvalve_data.state = opened;
	} else {
		potvalve_data.state = intermed;
	}

	retval = (potvalve_data.state != potvalve_data.prev_state);

	// if current state is intermed we don't want to set newstate because we don't need to send a new command
	// unless the goal is also new
	// Also don't send a command if we just un-tightened (started to open) otherwise we send open twice
	// if (potvalve_data.state == intermed || (prev == closed && potvalve_data.state == loose_closed))  {
	// 	retval = 0;
	// }
	return retval;
}

void WriteValves(unsigned int actuators_init, int* valve_addr)
{
	int i;
	int valve_flag = 0;

	static channel_t* encPotValveAddr;
	static channel_t* posPotValveAddr;
	static channel_t* statePotValveAddr;
	static channel_t* velPotValveAddr;
	static channel_t* openCurPotValveAddr;
	static channel_t* closeCurPotValveAddr;
	static channel_t* holdCurPotValveAddr;
	static channel_t* closedThresholdPotValveAddr;
	static channel_t* lclosedThresholdPotValveAddr;
	static channel_t* openThresholdPotValveAddr;
	static channel_t* potvalveMinTightenMoveAddr;
	static channel_t* enablePotValveAddr;

	static channel_t* limsPump1ValveAddr;
	static channel_t* limsPump2ValveAddr;
	static channel_t* posPump1ValveAddr;
	static channel_t* posPump2ValveAddr;
	static channel_t* velValveAddr;
	static channel_t* imoveValveAddr;
	static channel_t* iholdValveAddr;
	static channel_t* accValveAddr;

	static int firsttime = 1;

	if (firsttime) {
		encPotValveAddr = channels_find_by_name("enc_potvalve");
		posPotValveAddr = channels_find_by_name("pos_potvalve");
		statePotValveAddr = channels_find_by_name("state_potvalve");
		velPotValveAddr = channels_find_by_name("vel_potvalve");
		openCurPotValveAddr = channels_find_by_name("i_open_potvalve");
		closeCurPotValveAddr = channels_find_by_name("i_close_potvalve");
		holdCurPotValveAddr = channels_find_by_name("i_hold_potvalve");
		closedThresholdPotValveAddr = channels_find_by_name("thresh_clos_potvalve");
	    lclosedThresholdPotValveAddr = channels_find_by_name("threshlclos_potvalve");
	    openThresholdPotValveAddr = channels_find_by_name("thresh_open_potvalve");
		potvalveMinTightenMoveAddr = channels_find_by_name("tight_move_potvalve");

		limsPump1ValveAddr = channels_find_by_name("lims_pump_1_valve");
		limsPump2ValveAddr = channels_find_by_name("lims_pump_2_valve");
		posPump1ValveAddr = channels_find_by_name("pos_pump_1_valve");
		posPump2ValveAddr = channels_find_by_name("pos_pump_2_valve");
		velValveAddr = channels_find_by_name("vel_valves");
		imoveValveAddr = channels_find_by_name("i_move_valves");
		iholdValveAddr = channels_find_by_name("i_hold_valves");
		accValveAddr = channels_find_by_name("acc_valves");
		enablePotValveAddr = channels_find_by_name("enable_potvalve");
		firsttime = 0;
	}

	if (actuators_init & (0x1 << POTVALVE_NUM)) {
		SET_INT16(encPotValveAddr, potvalve_data.adc[0]);
		SET_INT32(posPotValveAddr, potvalve_data.pos);
		SET_UINT8(statePotValveAddr, potvalve_data.state);
		SET_UINT32(velPotValveAddr, CommandData.Cryo.potvalve_vel);
		SET_UINT8(openCurPotValveAddr, CommandData.Cryo.potvalve_opencurrent);
		SET_UINT8(closeCurPotValveAddr, CommandData.Cryo.potvalve_closecurrent);
		SET_UINT8(holdCurPotValveAddr, CommandData.Cryo.potvalve_hold_i);
		SET_UINT16(closedThresholdPotValveAddr, CommandData.Cryo.potvalve_closed_threshold);
		SET_UINT16(lclosedThresholdPotValveAddr, CommandData.Cryo.potvalve_lclosed_threshold);
		SET_UINT16(openThresholdPotValveAddr, CommandData.Cryo.potvalve_open_threshold);
		SET_UINT16(potvalveMinTightenMoveAddr, CommandData.Cryo.potvalve_min_tighten_move);
		SET_UINT8(enablePotValveAddr, CommandData.Cryo.potvalve_on);
	}

	for (i = 0; i < N_PUMP_VALVES; i++) {
		if (actuators_init & (0x1 << valve_addr[i])) {
			if (i == 0) {
				SET_UINT8(limsPump1ValveAddr, valve_data[0].limit);
				SET_INT32(posPump1ValveAddr, valve_data[0].pos);
			} else if (i == 1) {
				SET_UINT8(limsPump2ValveAddr, valve_data[1].limit);
				SET_INT32(posPump2ValveAddr, valve_data[1].pos);
			}
			valve_flag = 1;
		}
	}

	if (valve_flag == 1) {
		SET_UINT16(velValveAddr, CommandData.Cryo.valve_vel);
		SET_UINT8(imoveValveAddr, CommandData.Cryo.valve_move_i);
		SET_UINT8(iholdValveAddr, CommandData.Cryo.valve_hold_i);
		SET_UINT16(accValveAddr, CommandData.Cryo.valve_acc);
	}
}
