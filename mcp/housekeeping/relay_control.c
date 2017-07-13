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
 
 created by Ian Lowe 1-18-17
 **************************************************************************/


/*************************************************************************
 
 relay_control.c -- mcp code to control power relays
 
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
#include "multiplexed_labjack.h"
#include "relay_control.h"

/*
sets of bit values for the different channels
REC:
rec_on = 1
amp_supply_on = 2
therm_supply_on = 4
heater_supply_on = 8
OF:
relay_1 = 1
relay_2 = 2
relay_3 = 4
relay_4 = 8
relay_5 = 16
relay_6 = 32
relay_7 = 64
relay_8 = 128
relay_9 = 256
relay_10 = 512
relay_11 = 1024
relay_12 = 2048
relay_13 = 4096
relay_14 = 8192
relay_15 = 16384
relay_16 = 32768
IF:
relay_1 = 1
relay_2 = 2
relay_3 = 4
relay_4 = 8
relay_5 = 16
relay_6 = 32
relay_7 = 64
relay_8 = 128
relay_9 = 256
relay_10 = 512
*/
// structure to control the heater and amp power box
typedef struct {
    float rec_on;
    float rec_off;
    float amp_supply_on;
    float amp_supply_off;
    float therm_supply_on;
    float therm_supply_off;
    float heater_supply_on;
    float heater_supply_off;
    float update_rec;
} rec_control_t;
// structure that contains information about the OF relays state
typedef struct {
    float of_1_on;
    float of_2_on;
    float of_3_on;
    float of_4_on;
    float of_5_on;
    float of_6_on;
    float of_7_on;
    float of_8_on;
    float of_9_on;
    float of_10_on;
    float of_11_on;
    float of_12_on;
    float of_13_on;
    float of_14_on;
    float of_15_on;
    float of_16_on;
    float of_1_off;
    float of_2_off;
    float of_3_off;
    float of_4_off;
    float of_5_off;
    float of_6_off;
    float of_7_off;
    float of_8_off;
    float of_9_off;
    float of_10_off;
    float of_11_off;
    float of_12_off;
    float of_13_off;
    float of_14_off;
    float of_15_off;
    float of_16_off;
    float update_of;
} of_control_t;
// structure that contains information about the IF relay state
typedef struct {
    float if_1_on;
    float if_2_on;
    float if_3_on;
    float if_4_on;
    float if_5_on;
    float if_6_on;
    float if_7_on;
    float if_8_on;
    float if_9_on;
    float if_10_on;
    float if_1_off;
    float if_2_off;
    float if_3_off;
    float if_4_off;
    float if_5_off;
    float if_6_off;
    float if_7_off;
    float if_8_off;
    float if_9_off;
    float if_10_off;
    float update_if;
} if_control_t;

rec_control_t rec_state;

of_control_t of_state;

if_control_t if_state;
// initializes the REC state values to 0 before starting
static void rec_init(void) {
    rec_state.rec_on = 0;
    rec_state.rec_off = 0;
    rec_state.amp_supply_on = 0;
    rec_state.amp_supply_off = 0;
    rec_state.therm_supply_on = 0;
    rec_state.therm_supply_off = 0;
    rec_state.heater_supply_on = 0;
    rec_state.heater_supply_off = 0;
    rec_state.update_rec = 0;
}
// pulls data from the command data structure
static void rec_update_values(void) {
    rec_state.rec_on = CommandData.Relays.rec_on;
    rec_state.rec_off = CommandData.Relays.rec_off;
    rec_state.amp_supply_on = CommandData.Relays.amp_supply_on;
    rec_state.amp_supply_off = CommandData.Relays.amp_supply_off;
    rec_state.therm_supply_on = CommandData.Relays.therm_supply_on;
    rec_state.therm_supply_off = CommandData.Relays.therm_supply_off;
    rec_state.heater_supply_on = CommandData.Relays.heater_supply_on;
    rec_state.heater_supply_off = CommandData.Relays.heater_supply_off;
    CommandData.Relays.rec_on = 0;
    CommandData.Relays.rec_off = 0;
    CommandData.Relays.amp_supply_on = 0;
    CommandData.Relays.amp_supply_off = 0;
    CommandData.Relays.therm_supply_on = 0;
    CommandData.Relays.therm_supply_off = 0;
    CommandData.Relays.heater_supply_on = 0;
    CommandData.Relays.heater_supply_off = 0;
}
// function called to send the values to the labjack registers
static void rec_send_values(void) {
    labjack_queue_command(LABJACK_CRYO_2, POWER_BOX_ON, rec_state.rec_on);
    labjack_queue_command(LABJACK_CRYO_2, POWER_BOX_OFF, rec_state.rec_off);
    labjack_queue_command(LABJACK_CRYO_2, AMP_SUPPLY_ON, rec_state.amp_supply_on);
    labjack_queue_command(LABJACK_CRYO_2, AMP_SUPPLY_OFF, rec_state.amp_supply_off);
    labjack_queue_command(LABJACK_CRYO_2, THERM_READOUT_ON, rec_state.therm_supply_on);
    labjack_queue_command(LABJACK_CRYO_2, THERM_READOUT_OFF, rec_state.therm_supply_off);
    labjack_queue_command(LABJACK_CRYO_2, HEATER_SUPPLY_ON, rec_state.heater_supply_on);
    labjack_queue_command(LABJACK_CRYO_2, HEATER_SUPPLY_OFF, rec_state.heater_supply_off);
}
// function called in the main loop of MCP
void rec_control(void) {
    if (CommandData.Relays.labjack[1] == 1) {
        static int rec_startup = 1;
        static int rec_trigger = 0;
        if (rec_trigger == 1) { // turns off the power pulse after 1 second
            rec_init();
            rec_trigger = 0;
            rec_send_values();
        } // turns on a power pulse and sets reminder to turn it off
        if ((rec_state.update_rec = CommandData.Relays.update_rec) == 1) {
            rec_update_values();
            CommandData.Relays.update_rec = 0;
            rec_send_values();
            rec_trigger = 1;
        }
        if (rec_startup == 1) { // initializes the power box to feed power to relays (ONLY REC)
            rec_startup = 0;
            labjack_queue_command(LABJACK_CRYO_2, POWER_BOX_ON, 1);
            labjack_queue_command(LABJACK_CRYO_2, POWER_BOX_OFF, 0);
            rec_init();
            rec_trigger = 1;
        }
    }
}
// initializes the OF relay structure
static void of_init(void) {
    of_state.of_1_on = 0;
    of_state.of_1_off = 0;
    of_state.of_2_on = 0;
    of_state.of_2_off = 0;
    of_state.of_3_on = 0;
    of_state.of_3_off = 0;
    of_state.of_4_on = 0;
    of_state.of_4_off = 0;
    of_state.of_5_on = 0;
    of_state.of_5_off = 0;
    of_state.of_6_on = 0;
    of_state.of_6_off = 0;
    of_state.of_7_on = 0;
    of_state.of_7_off = 0;
    of_state.of_8_on = 0;
    of_state.of_8_off = 0;
    of_state.of_9_on = 0;
    of_state.of_9_off = 0;
    of_state.of_10_on = 0;
    of_state.of_10_off = 0;
    of_state.of_11_on = 0;
    of_state.of_11_off = 0;
    of_state.of_12_on = 0;
    of_state.of_12_off = 0;
    of_state.of_13_on = 0;
    of_state.of_13_off = 0;
    of_state.of_14_on = 0;
    of_state.of_14_off = 0;
    of_state.of_15_on = 0;
    of_state.of_15_off = 0;
    of_state.of_16_on = 0;
    of_state.of_16_off = 0;
    CommandData.Relays.of_1_on = of_state.of_1_on;
    CommandData.Relays.of_1_off = of_state.of_1_off;
    CommandData.Relays.of_2_on = of_state.of_2_on;
    CommandData.Relays.of_2_off = of_state.of_2_off;
    CommandData.Relays.of_3_on = of_state.of_3_on;
    CommandData.Relays.of_3_off = of_state.of_3_off;
    CommandData.Relays.of_4_on = of_state.of_4_on;
    CommandData.Relays.of_4_off = of_state.of_4_off;
    CommandData.Relays.of_5_on = of_state.of_5_on;
    CommandData.Relays.of_5_off = of_state.of_5_off;
    CommandData.Relays.of_6_on = of_state.of_6_on;
    CommandData.Relays.of_6_off = of_state.of_6_off;
    CommandData.Relays.of_7_on = of_state.of_7_on;
    CommandData.Relays.of_7_off = of_state.of_7_off;
    CommandData.Relays.of_8_on = of_state.of_8_on;
    CommandData.Relays.of_8_off = of_state.of_8_off;
    CommandData.Relays.of_9_on = of_state.of_9_on;
    CommandData.Relays.of_9_off = of_state.of_9_off;
    CommandData.Relays.of_10_on = of_state.of_10_on;
    CommandData.Relays.of_10_off = of_state.of_10_off;
    CommandData.Relays.of_11_on = of_state.of_11_on;
    CommandData.Relays.of_11_off = of_state.of_11_off;
    CommandData.Relays.of_12_on = of_state.of_12_on;
    CommandData.Relays.of_12_off = of_state.of_12_off;
    CommandData.Relays.of_13_on = of_state.of_13_on;
    CommandData.Relays.of_13_off = of_state.of_13_off;
    CommandData.Relays.of_14_on = of_state.of_14_on;
    CommandData.Relays.of_14_off = of_state.of_14_off;
    CommandData.Relays.of_15_on = of_state.of_15_on;
    CommandData.Relays.of_15_off = of_state.of_15_off;
    CommandData.Relays.of_16_on = of_state.of_16_on;
    CommandData.Relays.of_16_off = of_state.of_16_off;
}
// pulls data from the command data struct
static void of_update_values(void) {
    of_state.of_1_on = CommandData.Relays.of_1_on;
    of_state.of_1_off = CommandData.Relays.of_1_off;
    of_state.of_2_on = CommandData.Relays.of_2_on;
    of_state.of_2_off = CommandData.Relays.of_2_off;
    of_state.of_3_on = CommandData.Relays.of_3_on;
    of_state.of_3_off = CommandData.Relays.of_3_off;
    of_state.of_4_on = CommandData.Relays.of_4_on;
    of_state.of_4_off = CommandData.Relays.of_4_off;
    of_state.of_5_on = CommandData.Relays.of_5_on;
    of_state.of_5_off = CommandData.Relays.of_5_off;
    of_state.of_6_on = CommandData.Relays.of_6_on;
    of_state.of_6_off = CommandData.Relays.of_6_off;
    of_state.of_7_on = CommandData.Relays.of_7_on;
    of_state.of_7_off = CommandData.Relays.of_7_off;
    of_state.of_8_on = CommandData.Relays.of_8_on;
    of_state.of_8_off = CommandData.Relays.of_8_off;
    of_state.of_9_on = CommandData.Relays.of_9_on;
    of_state.of_9_off = CommandData.Relays.of_9_off;
    of_state.of_10_on = CommandData.Relays.of_10_on;
    of_state.of_10_off = CommandData.Relays.of_10_off;
    of_state.of_11_on = CommandData.Relays.of_11_on;
    of_state.of_11_off = CommandData.Relays.of_11_off;
    of_state.of_12_on = CommandData.Relays.of_12_on;
    of_state.of_12_off = CommandData.Relays.of_12_off;
    of_state.of_13_on = CommandData.Relays.of_13_on;
    of_state.of_13_off = CommandData.Relays.of_13_off;
    of_state.of_14_on = CommandData.Relays.of_14_on;
    of_state.of_14_off = CommandData.Relays.of_14_off;
    of_state.of_15_on = CommandData.Relays.of_15_on;
    of_state.of_15_off = CommandData.Relays.of_15_off;
    of_state.of_16_on = CommandData.Relays.of_16_on;
    of_state.of_16_off = CommandData.Relays.of_16_off;
}
// sends all of the new values to the labjacks for the OF
static void of_send_values(void) {
    labjack_queue_command(LABJACK_OF_1, RELAY_1_ON, of_state.of_1_on);
    labjack_queue_command(LABJACK_OF_1, RELAY_1_OFF, of_state.of_1_off);
    labjack_queue_command(LABJACK_OF_1, RELAY_2_ON, of_state.of_2_on);
    labjack_queue_command(LABJACK_OF_1, RELAY_2_OFF, of_state.of_2_off);
    labjack_queue_command(LABJACK_OF_1, RELAY_3_ON, of_state.of_3_on);
    labjack_queue_command(LABJACK_OF_1, RELAY_3_OFF, of_state.of_3_off);
    labjack_queue_command(LABJACK_OF_1, RELAY_4_ON, of_state.of_4_on);
    labjack_queue_command(LABJACK_OF_1, RELAY_4_OFF, of_state.of_4_off);
    labjack_queue_command(LABJACK_OF_1, RELAY_5_ON, of_state.of_5_on);
    labjack_queue_command(LABJACK_OF_1, RELAY_5_OFF, of_state.of_5_off);
    labjack_queue_command(LABJACK_OF_1, RELAY_6_ON, of_state.of_6_on);
    labjack_queue_command(LABJACK_OF_1, RELAY_6_OFF, of_state.of_6_off);
    labjack_queue_command(LABJACK_OF_1, RELAY_7_ON, of_state.of_7_on);
    labjack_queue_command(LABJACK_OF_1, RELAY_7_OFF, of_state.of_7_off);
    labjack_queue_command(LABJACK_OF_2, RELAY_8_ON, of_state.of_8_on);
    labjack_queue_command(LABJACK_OF_2, RELAY_8_OFF, of_state.of_8_off);
    labjack_queue_command(LABJACK_OF_2, RELAY_9_ON, of_state.of_9_on);
    labjack_queue_command(LABJACK_OF_2, RELAY_9_OFF, of_state.of_9_off);
    labjack_queue_command(LABJACK_OF_2, RELAY_10_ON, of_state.of_10_on);
    labjack_queue_command(LABJACK_OF_2, RELAY_10_OFF, of_state.of_10_off);
    labjack_queue_command(LABJACK_OF_2, RELAY_11_ON, of_state.of_11_on);
    labjack_queue_command(LABJACK_OF_2, RELAY_11_OFF, of_state.of_11_off);
    labjack_queue_command(LABJACK_OF_2, RELAY_12_ON, of_state.of_12_on);
    labjack_queue_command(LABJACK_OF_2, RELAY_12_OFF, of_state.of_12_off);
    labjack_queue_command(LABJACK_OF_2, RELAY_13_ON, of_state.of_13_on);
    labjack_queue_command(LABJACK_OF_2, RELAY_13_OFF, of_state.of_13_off);
    labjack_queue_command(LABJACK_OF_2, RELAY_14_ON, of_state.of_14_on);
    labjack_queue_command(LABJACK_OF_2, RELAY_14_OFF, of_state.of_14_off);
    labjack_queue_command(LABJACK_OF_2, RELAY_15_ON, of_state.of_15_on);
    labjack_queue_command(LABJACK_OF_2, RELAY_15_OFF, of_state.of_15_off);
    labjack_queue_command(LABJACK_OF_1, RELAY_16_ON, of_state.of_16_on);
    labjack_queue_command(LABJACK_OF_1, RELAY_16_OFF, of_state.of_16_off);
}
//
void of_control(void) {
    if (CommandData.Relays.labjack[2] == 1 && CommandData.Relays.labjack[3] == 1) {
        static int of_trigger = 0;
        static int counter = 0;
        // 1 second later sends all 0s
        if (of_trigger == 1) { // turns off the previous set of pulses
            of_trigger = 0;
            of_init();
            of_send_values();
        }
        // sends the values from command data
        if (of_trigger == 2) {
            counter--;
            if (counter == 0) {
                of_trigger = 1;
            }
        }
        if ((of_state.update_of = CommandData.Relays.update_of) == 1) {
            of_update_values();
            of_trigger = 1;
            counter = 1;
            blast_info("preparing to send values");
            of_send_values();
            blast_info("values queued");
            CommandData.Relays.update_of = 0;
        }
    }
}
// initializes the IF relay structure to 0
static void if_init(void) {
    if_state.if_1_on = 0;
    if_state.if_1_off = 0;
    if_state.if_2_on = 0;
    if_state.if_2_off = 0;
    if_state.if_3_on = 0;
    if_state.if_3_off = 0;
    if_state.if_4_on = 0;
    if_state.if_4_off = 0;
    if_state.if_5_on = 0;
    if_state.if_5_off = 0;
    if_state.if_6_on = 0;
    if_state.if_6_off = 0;
    if_state.if_7_on = 0;
    if_state.if_7_off = 0;
    if_state.if_8_on = 0;
    if_state.if_8_off = 0;
    if_state.if_9_on = 0;
    if_state.if_9_off = 0;
    if_state.if_10_on = 0;
    if_state.if_10_off = 0;
    CommandData.Relays.if_1_on = if_state.if_1_on;
    CommandData.Relays.if_1_off = if_state.if_1_off;
    CommandData.Relays.if_2_on = if_state.if_2_on;
    CommandData.Relays.if_2_off = if_state.if_2_off;
    CommandData.Relays.if_3_on = if_state.if_3_on;
    CommandData.Relays.if_3_off = if_state.if_3_off;
    CommandData.Relays.if_4_on = if_state.if_4_on;
    CommandData.Relays.if_4_off = if_state.if_4_off;
    CommandData.Relays.if_5_on = if_state.if_5_on;
    CommandData.Relays.if_5_off = if_state.if_5_off;
    CommandData.Relays.if_6_on = if_state.if_6_on;
    CommandData.Relays.if_6_off = if_state.if_6_off;
    CommandData.Relays.if_7_on = if_state.if_7_on;
    CommandData.Relays.if_7_off = if_state.if_7_off;
    CommandData.Relays.if_8_on = if_state.if_8_on;
    CommandData.Relays.if_8_off = if_state.if_8_off;
    CommandData.Relays.if_9_on = if_state.if_9_on;
    CommandData.Relays.if_9_off = if_state.if_9_off;
    CommandData.Relays.if_10_on = if_state.if_10_on;
    CommandData.Relays.if_10_off = if_state.if_10_off;
}
// pulls the inner frame relay values from the command data struct
static void if_update_values(void) {
    if_state.if_1_on = CommandData.Relays.if_1_on;
    if_state.if_1_off = CommandData.Relays.if_1_off;
    if_state.if_2_on = CommandData.Relays.if_2_on;
    if_state.if_2_off = CommandData.Relays.if_2_off;
    if_state.if_3_on = CommandData.Relays.if_3_on;
    if_state.if_3_off = CommandData.Relays.if_3_off;
    if_state.if_4_on = CommandData.Relays.if_4_on;
    if_state.if_4_off = CommandData.Relays.if_4_off;
    if_state.if_5_on = CommandData.Relays.if_5_on;
    if_state.if_5_off = CommandData.Relays.if_5_off;
    if_state.if_6_on = CommandData.Relays.if_6_on;
    if_state.if_6_off = CommandData.Relays.if_6_off;
    if_state.if_7_on = CommandData.Relays.if_7_on;
    if_state.if_7_off = CommandData.Relays.if_7_off;
    if_state.if_8_on = CommandData.Relays.if_8_on;
    if_state.if_8_off = CommandData.Relays.if_8_off;
    if_state.if_9_on = CommandData.Relays.if_9_on;
    if_state.if_9_off = CommandData.Relays.if_9_off;
    if_state.if_10_on = CommandData.Relays.if_10_on;
    if_state.if_10_off = CommandData.Relays.if_10_off;
}
// sends the inner frame relay values to the labjack
static void if_send_values(void) {
    labjack_queue_command(LABJACK_OF_3, RELAY_1_ON, if_state.if_1_on);
    labjack_queue_command(LABJACK_OF_3, RELAY_1_OFF, if_state.if_1_off);
    labjack_queue_command(LABJACK_OF_3, RELAY_2_ON, if_state.if_2_on);
    labjack_queue_command(LABJACK_OF_3, RELAY_2_OFF, if_state.if_2_off);
    labjack_queue_command(LABJACK_OF_3, RELAY_3_ON, if_state.if_3_on);
    labjack_queue_command(LABJACK_OF_3, RELAY_3_OFF, if_state.if_3_off);
    labjack_queue_command(LABJACK_OF_3, RELAY_4_ON, if_state.if_4_on);
    labjack_queue_command(LABJACK_OF_3, RELAY_4_OFF, if_state.if_4_off);
    labjack_queue_command(LABJACK_OF_3, RELAY_5_ON, if_state.if_5_on);
    labjack_queue_command(LABJACK_OF_3, RELAY_5_OFF, if_state.if_5_off);
    labjack_queue_command(LABJACK_OF_3, RELAY_6_ON, if_state.if_6_on);
    labjack_queue_command(LABJACK_OF_3, RELAY_6_OFF, if_state.if_6_off);
    labjack_queue_command(LABJACK_OF_3, RELAY_7_ON, if_state.if_7_on);
    labjack_queue_command(LABJACK_OF_3, RELAY_7_OFF, if_state.if_7_off);
    labjack_queue_command(LABJACK_OF_3, RELAY_8_ON, if_state.if_8_on);
    labjack_queue_command(LABJACK_OF_3, RELAY_8_OFF, if_state.if_8_off);
    labjack_queue_command(LABJACK_OF_3, IF_RELAY_9_ON, if_state.if_9_on);
    labjack_queue_command(LABJACK_OF_3, IF_RELAY_9_OFF, if_state.if_9_off);
    labjack_queue_command(LABJACK_OF_3, IF_RELAY_10_ON, if_state.if_10_on);
    labjack_queue_command(LABJACK_OF_3, IF_RELAY_10_OFF, if_state.if_10_off);
}
// function that calls all of the sub functions ffor controlling the IF relays
void if_control(void) {
    if (CommandData.Relays.labjack[3] == 1) {
        static int if_trigger = 0;
        static int if_counter = 0;
        if (if_trigger == 1) { // turns off the previous set of pulses
            if_trigger = 0;
            if_init();
            if_send_values();
        }
        if (if_trigger == 2) {
            if_counter--;
            if (if_counter == 0) {
                if_trigger = 1;
            }
        }
        if ((if_state.update_if = CommandData.Relays.update_if) == 1) {
            if_update_values();
            if_trigger = 2;
            if_counter = 3;
            if_send_values();
            CommandData.Relays.update_if = 0;
        }
    }
}








