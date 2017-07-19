/* 
 * balance.c: 
 *
 * This software is copyright (C) 2016 Laura Fissel
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
 * Created on: Apr 27, 2016 by Laura Fissel
 */
#include <stdlib.h>
#include <ec_motors.h>
#include <unistd.h>

#include "actuators.h"
#include "mcp.h"
#include "lut.h"
#include "ezstep.h"
#include "command_struct.h"
#include "pointing_struct.h" /* To access ACSData */
#include "tx.h" /* InCharge */
#include "balance.h"
#include "motors.h"

#define BAL_EL_FILTER_LEN 150 // 30 seconds

typedef enum
{
    negative = 0, no_move, positive
} move_type_t;

typedef struct {
	uint16_t init;
	int addr;
	int ind;
    	int do_move;
	int moving;
	move_type_t dir;
	double i_el_avg;
	int32_t pos;
	uint8_t lims;
} balance_state_t;

static balance_state_t balance_state;

// Decides when the balance system should be turned on and off.
void ControlBalance(void)
{
	double i_el = 0.0;
    static int firsttime = 1;
    int i_motors = GETREADINDEX(motor_index);

//  Initialize balance state
    if (firsttime) {
        balance_state.i_el_avg = 0.0;
        firsttime = 0;
    }

	i_el = ElevMotorData[i_motors].current;

//   calculate speed and direction
    balance_state.i_el_avg = i_el / BAL_EL_FILTER_LEN +
                           balance_state.i_el_avg * (BAL_EL_FILTER_LEN - 1) / BAL_EL_FILTER_LEN;
// The balance system should not be on if we are slewing,
// or if the balance system is commanded off, or if we are doing an El scan.

    if ((CommandData.balance.mode == bal_rest) || (CommandData.pointing_mode.nw > 0) ||
       (CommandData.pointing_mode.mode == P_EL_SCAN)) {
           balance_state.do_move = 0;
	   balance_state.dir = no_move;
       } else if (CommandData.balance.mode == bal_manual) {
           balance_state.dir = CommandData.balance.bal_move_type;
       	   if (balance_state.dir != no_move) {
		balance_state.do_move = 1;
	   } else {
		balance_state.do_move = 0;
	   }
	} else if (CommandData.balance.mode == bal_auto) {
	   if (balance_state.i_el_avg > 0) {
               if (balance_state.i_el_avg > CommandData.balance.i_el_on_bal) {
                   blast_info("Setting the balance system to move in the negative direction.");
                   balance_state.do_move = 1;
                   balance_state.dir = negative;
               } else if (balance_state.moving && (balance_state.i_el_avg > CommandData.balance.i_el_off_bal)) {
                   // blast_info("Balance system still moving in negative direction.");
		   balance_state.do_move = 1;
                   balance_state.dir = negative;
               } else if (balance_state.moving) {
		   blast_info("Balanced.");
		   balance_state.do_move = 0;
		   balance_state.dir = no_move;
	       }
           } else if (balance_state.i_el_avg < 0) {
               if (balance_state.i_el_avg < (-1.0)*CommandData.balance.i_el_on_bal) {
                   blast_info("Setting the balance system to move in the positive direction.");
                   balance_state.do_move = 1;
                   balance_state.dir = positive;
               } else if (balance_state.moving && (balance_state.i_el_avg < (-1.0)*CommandData.balance.i_el_off_bal)) {
                   // blast_info("Balance system still moving in positive direction.");
		   balance_state.do_move = 1;
                   balance_state.dir = positive;
               } else if (balance_state.moving) {
		   blast_info("Balanced.");
		   balance_state.do_move = 0;
		   balance_state.dir = no_move;
	       }
           }
       } else {
           balance_state.do_move = 0;
      	   balance_state.dir = no_move;
       }
}

void WriteBalance_5Hz(void)
{
    static channel_t* velBalAddr;
    static channel_t* accBalAddr;
    static channel_t* iMoveBalAddr;
    static channel_t* iHoldBalAddr;
    static channel_t* iLevelOnBalAddr;
    static channel_t* iLevelOffBalAddr;
    static channel_t* iElReqAvgBalAddr;
    static channel_t* statusBalAddr;
    static channel_t* posBalAddr;
    static channel_t* limBalAddr;
    static int firsttime = 1;
    uint8_t status = 0;
    if (firsttime) {
      limBalAddr = channels_find_by_name("lim_bal");
      posBalAddr = channels_find_by_name("pos_bal");
      velBalAddr = channels_find_by_name("vel_bal");
      accBalAddr = channels_find_by_name("acc_bal");
      iMoveBalAddr = channels_find_by_name("i_move_bal");
      iHoldBalAddr = channels_find_by_name("i_hold_bal");
      iLevelOnBalAddr = channels_find_by_name("i_level_on_bal");
      iLevelOffBalAddr = channels_find_by_name("i_level_off_bal");
      iElReqAvgBalAddr = channels_find_by_name("i_el_req_avg_bal");
      statusBalAddr = channels_find_by_name("status_bal");
    }
    SET_UINT32(velBalAddr, CommandData.balance.vel);
    SET_UINT16(accBalAddr, CommandData.balance.acc);
    SET_UINT16(iMoveBalAddr, CommandData.balance.move_i);
    SET_UINT16(iHoldBalAddr, CommandData.balance.hold_i);
    SET_SCALED_VALUE(iLevelOnBalAddr, CommandData.balance.i_el_on_bal);
    SET_SCALED_VALUE(iLevelOffBalAddr, CommandData.balance.i_el_off_bal);
    SET_SCALED_VALUE(iElReqAvgBalAddr, balance_state.i_el_avg);
    status |= ((0x03) & balance_state.dir);
    status |= (((0x01) & balance_state.init) << 2);
    status |= (((0x01) & balance_state.do_move) << 3);
    status |= (((0x01) & balance_state.moving) << 4);
    status |= (((0x03) & CommandData.balance.mode) << 5);
    SET_UINT8(statusBalAddr, status);
    SET_INT32(posBalAddr, balance_state.pos);
    SET_UINT8(limBalAddr, balance_state.lims);
}
// Handles stepper communication for the balance motor.
void DoBalance(struct ezbus* bus)
{
    static int firsttime = 1;

    if (firsttime) {
        balance_state.init = 0;
        balance_state.ind = BALANCENUM;
        balance_state.addr = GetActAddr(balance_state.ind);
        /* Attempt to stop the balance motor */
        EZBus_Take(bus, balance_state.addr);
        blast_info("Making sure the balance system is not running on startup.");
		EZBus_Stop(bus, balance_state.addr);
        EZBus_MoveComm(bus, balance_state.addr, BALANCE_PREAMBLE);
	EZBus_Release(bus, balance_state.addr);
        balance_state.moving = 0;
        balance_state.dir = no_move;
        balance_state.do_move = 0;
	balance_state.lims = 0;
        firsttime = 0;
     }

        /* update the Balance move parameters */
    EZBus_SetVel(bus, balance_state.addr, CommandData.balance.vel);
    EZBus_SetAccel(bus, balance_state.addr, CommandData.balance.acc);
    EZBus_SetIMove(bus, balance_state.addr, CommandData.balance.move_i);
    EZBus_SetIHold(bus, balance_state.addr, CommandData.balance.hold_i);

	// get (relative) position on the rail and read limit switches
    EZBus_ReadInt(bus, balance_state.addr, "?0", &balance_state.pos);
    EZBus_ReadInt(bus, balance_state.addr, "?4", &balance_state.lims);

// TODO(laura): Add checking to make sure that the motor commands actually went through
// updating the status variables.
    if ((balance_state.do_move) && (!balance_state.moving)) {
        EZBus_Take(bus, balance_state.addr);
        if (balance_state.dir == positive) { // Positive direction
		if (EZBus_RelMove(bus, balance_state.addr, INT_MAX) != EZ_ERR_OK) {
			blast_warn("Error starting positive balance move");
		}
	} else if (balance_state.dir == negative) {
		if (EZBus_RelMove(bus, balance_state.addr, INT_MIN) != EZ_ERR_OK) {
			blast_warn("Error starting negative balance move");
		}
	}
        EZBus_Release(bus, balance_state.addr);
        balance_state.moving = 1;
    } else if (!balance_state.do_move && balance_state.moving) {
        EZBus_Take(bus, balance_state.addr);
        blast_info("Stopping balance motor.");
		EZBus_Stop(bus, balance_state.addr);
        EZBus_Release(bus, balance_state.addr);
        balance_state.moving = 0;
    }
// Write balance data
    WriteBalance_5Hz();

    usleep(100000);
}
