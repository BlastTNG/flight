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

#include "actuators.h"
#include "mcp.h"
#include "lut.h"
#include "ezstep.h"
#include "command_struct.h"
#include "pointing_struct.h" /* To access ACSData */
#include "tx.h" /* InCharge */
#include "balance.h"
#include "motors.h"

#define BAL_EL_FILTER_LEN 500 // 100 seconds

typedef enum
{
    negative = 0, positive
} move_type_t;

typedef struct {
	uint16_t init;
	int addr;
	int ind;
    int do_move;
	int moving;
	move_type_t dir;
} balance_state_t;

static balance_state_t balance_state;

// Decides when the balance system should be turned on and off.
void ControlBalance(void)
{
	double i_el = 0.0;
	static double i_el_avg = 0.0;
    static int firsttime = 1;
    int i_motors = GETREADINDEX(motor_index);

	i_el = ElevMotorData[i_motors].current;

//   calculate speed and direction
    i_el_avg = i_el / BAL_EL_FILTER_LEN + i_el_avg * (BAL_EL_FILTER_LEN - 1) / BAL_EL_FILTER_LEN;
    blast_info("i_el = %f, i_el_avg = %f", i_el, i_el_avg);
// The balance system should not be on if we are slewing,
// or if the balance system is commanded off, or if we are doing an El scan.
    if ((CommandData.balance.mode == bal_rest) || (CommandData.pointing_mode.nw > 0) ||
       (CommandData.pointing_mode.mode == P_EL_SCAN)) {
           balance_state.do_move = 0;
       } else if (CommandData.balance.mode == bal_manual) {
           balance_state.do_move = 0;
           balance_state.dir = CommandData.balance.bal_move_type;
       } else if (CommandData.balance.mode == bal_auto) {
           if (i_el_avg > 0) {
               if (i_el_avg > CommandData.balance.i_el_on_bal) {
                   blast_info("Setting the balance system to move in the positive direction.");
                   balance_state.do_move = 1;
                   balance_state.do_move = positive;
               } else if (balance_state.moving && (i_el_avg > CommandData.balance.i_el_off_bal)) {
                   blast_info("Still moving and above i_el_off_bal. Keep the balance system going.");
                   balance_state.do_move = 1;
                   balance_state.do_move = positive;
               }
           } else if (i_el_avg < 0) {
               if (i_el_avg < (-1.0)*CommandData.balance.i_el_on_bal) {
                   blast_info("Setting the balance system to move in the positive direction.");
                   balance_state.do_move = 1;
                   balance_state.do_move = negative;
               } else if (balance_state.moving && (i_el_avg < (-1.0)*CommandData.balance.i_el_off_bal)) {
                   blast_info("Still moving and below -i_el_off_bal. Keep the balance system going.");
                   balance_state.do_move = 1;
                   balance_state.do_move = negative;
               }
           }
       } else {
           balance_state.do_move = 0;
       }
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
        EZBus_Release(bus, balance_state.addr);
        balance_state.moving = 0;
        balance_state.dir = 0;
        balance_state.do_move = 0;
        firsttime = 0;
    }

        /* update the Balance move parameters */
    EZBus_SetVel(bus, balance_state.addr, CommandData.balance.vel);
    EZBus_SetAccel(bus, balance_state.addr, CommandData.balance.acc);
    EZBus_SetIMove(bus, balance_state.addr, CommandData.balance.move_i);
    EZBus_SetIHold(bus, balance_state.addr, CommandData.balance.hold_i);

// TODO(laura): Add checking to make sure that the motor commands actually went through
// updating the status variables.
    if ((balance_state.do_move) && (!balance_state.moving)) {
        EZBus_Take(bus, balance_state.addr);
        blast_info("Starting the balance system. Direction = %d", balance_state.dir);
        if (balance_state.dir) { // Positive direction
//            EZBus_Send(bus, balance_state.addr, "P0R");
        } else {
//            EZBus_Send(bus, balance_state.addr, "D0R");
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
}
