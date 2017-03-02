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
#include "blast.h"
#include "multiplexed_labjack.h"
#include "relay_control.h"


typedef struct {
    uint16_t rec_on;
    uint16_t rec_off;
    uint16_t amp_supply_on;
    uint16_t amp_supply_off;
    uint16_t therm_supply_on;
    uint16_t therm_supply_off;
    uint16_t heater_supply_on;
    uint16_t heater_supply_off;
    uint16_t update_rec;
} rec_control_t;

typedef struct {
    uint16_t of_1_on;
    uint16_t of_2_on;
    uint16_t of_3_on;
    uint16_t of_4_on;
    uint16_t of_5_on;
    uint16_t of_6_on;
    uint16_t of_7_on;
    uint16_t of_8_on;
    uint16_t of_9_on;
    uint16_t of_10_on;
    uint16_t of_11_on;
    uint16_t of_12_on;
    uint16_t of_13_on;
    uint16_t of_14_on;
    uint16_t of_15_on;
    uint16_t of_16_on;
    uint16_t of_1_off;
    uint16_t of_2_off;
    uint16_t of_3_off;
    uint16_t of_4_off;
    uint16_t of_5_off;
    uint16_t of_6_off;
    uint16_t of_7_off;
    uint16_t of_8_off;
    uint16_t of_9_off;
    uint16_t of_10_off;
    uint16_t of_11_off;
    uint16_t of_12_off;
    uint16_t of_13_off;
    uint16_t of_14_off;
    uint16_t of_15_off;
    uint16_t of_16_off;
    uint16_t update_of;
} of_control_t;

typedef struct {
    uint16_t if_1_on;
    uint16_t if_2_on;
    uint16_t if_3_on;
    uint16_t if_4_on;
    uint16_t if_5_on;
    uint16_t if_6_on;
    uint16_t if_7_on;
    uint16_t if_8_on;
    uint16_t if_9_on;
    uint16_t if_10_on;
    uint16_t if_1_off;
    uint16_t if_2_off;
    uint16_t if_3_off;
    uint16_t if_4_off;
    uint16_t if_5_off;
    uint16_t if_6_off;
    uint16_t if_7_off;
    uint16_t if_8_off;
    uint16_t if_9_off;
    uint16_t if_10_off;
    uint16_t update_if;
} if_control_t;

rec_control_t rec_state;

of_control_t of_state;

if_control_t if_state;

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

static void rec_update_values(void) {
    rec_state.rec_on = CommandData.Relays.rec_on;
    rec_state.rec_off = CommandData.Relays.rec_off;
    rec_state.amp_supply_on = CommandData.Relays.amp_supply_on;
    rec_state.amp_supply_off = CommandData.Relays.amp_supply_off;
    rec_state.therm_supply_on = CommandData.Relays.therm_supply_on;
    rec_state.therm_supply_off = CommandData.Relays.therm_supply_off;
    rec_state.heater_supply_on = CommandData.Relays.heater_supply_on;
    rec_state.heater_supply_off = CommandData.Relays.heater_supply_off;
}

static void rec_send_values(void) {
    heater_write(LABJACK_CRYO_2, POWER_BOX_ON, CommandData.Relays.rec_on);
    heater_write(LABJACK_CRYO_2, POWER_BOX_OFF, CommandData.Relays.rec_off);
    heater_write(LABJACK_CRYO_2, AMP_SUPPLY_ON, CommandData.Relays.amp_supply_on);
    heater_write(LABJACK_CRYO_2, AMP_SUPPLY_OFF, CommandData.Relays.amp_supply_off);
    heater_write(LABJACK_CRYO_2, THERM_READOUT_ON, CommandData.Relays.therm_supply_on);
    heater_write(LABJACK_CRYO_2, THERM_READOUT_OFF, CommandData.Relays.therm_supply_off);
    heater_write(LABJACK_CRYO_2, HEATER_SUPPLY_ON, CommandData.Relays.heater_supply_on);
    heater_write(LABJACK_CRYO_2, HEATER_SUPPLY_OFF, CommandData.Relays.heater_supply_off);
}

void rec_control(void) {
    static int rec_startup = 1;
    static int rec_trigger = 0;
    if (rec_trigger == 1) { // turns off the power pulse after 1 second
        rec_init();
        rec_send_values();
        rec_trigger = 0;
    } // turns on a power pulse and sets reminder to turn it off
    if ((rec_state.update_rec = CommandData.Relays.update_rec) == 1) {
        rec_update_values();
        rec_trigger = 1;
        rec_send_values();
        CommandData.Relays.update_rec = 0;
    }
    if (rec_startup == 1) { // initializes the power box to feed power to relays (ONLY REC)
        heater_write(LABJACK_CRYO_2, POWER_BOX_ON, 1);
        heater_write(LABJACK_CRYO_2, POWER_BOX_OFF, 0);
        rec_init();
        rec_startup = 0;
        rec_trigger = 1;
    }
}

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
}

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

static void of_send_values(void) {
    heater_write(LABJACK_OF_1, RELAY_1_ON, of_state.of_1_on);
    heater_write(LABJACK_OF_1, RELAY_1_OFF, of_state.of_1_off);
    heater_write(LABJACK_OF_1, RELAY_2_ON, of_state.of_2_on);
    heater_write(LABJACK_OF_1, RELAY_2_OFF, of_state.of_2_off);
    heater_write(LABJACK_OF_1, RELAY_3_ON, of_state.of_3_on);
    heater_write(LABJACK_OF_1, RELAY_3_OFF, of_state.of_3_off);
    heater_write(LABJACK_OF_1, RELAY_4_ON, of_state.of_4_on);
    heater_write(LABJACK_OF_1, RELAY_4_OFF, of_state.of_4_off);
    heater_write(LABJACK_OF_1, RELAY_5_ON, of_state.of_5_on);
    heater_write(LABJACK_OF_1, RELAY_5_OFF, of_state.of_5_off);
    heater_write(LABJACK_OF_1, RELAY_6_ON, of_state.of_6_on);
    heater_write(LABJACK_OF_1, RELAY_6_OFF, of_state.of_6_off);
    heater_write(LABJACK_OF_1, RELAY_7_ON, of_state.of_7_on);
    heater_write(LABJACK_OF_1, RELAY_7_OFF, of_state.of_7_off);
    heater_write(LABJACK_OF_2, RELAY_8_ON, of_state.of_8_on);
    heater_write(LABJACK_OF_2, RELAY_8_OFF, of_state.of_8_off);
    heater_write(LABJACK_OF_2, RELAY_9_ON, of_state.of_9_on);
    heater_write(LABJACK_OF_2, RELAY_9_OFF, of_state.of_9_off);
    heater_write(LABJACK_OF_2, RELAY_10_ON, of_state.of_10_on);
    heater_write(LABJACK_OF_2, RELAY_10_OFF, of_state.of_10_off);
    heater_write(LABJACK_OF_2, RELAY_11_ON, of_state.of_11_on);
    heater_write(LABJACK_OF_2, RELAY_11_OFF, of_state.of_11_off);
    heater_write(LABJACK_OF_2, RELAY_12_ON, of_state.of_12_on);
    heater_write(LABJACK_OF_2, RELAY_12_OFF, of_state.of_12_off);
    heater_write(LABJACK_OF_2, RELAY_13_ON, of_state.of_13_on);
    heater_write(LABJACK_OF_2, RELAY_13_OFF, of_state.of_13_off);
    heater_write(LABJACK_OF_2, RELAY_14_ON, of_state.of_14_on);
    heater_write(LABJACK_OF_2, RELAY_14_OFF, of_state.of_14_off);
    heater_write(LABJACK_OF_2, RELAY_15_ON, of_state.of_15_on);
    heater_write(LABJACK_OF_2, RELAY_15_OFF, of_state.of_15_off);
    heater_write(LABJACK_OF_1, RELAY_16_ON, of_state.of_16_on);
    heater_write(LABJACK_OF_1, RELAY_16_OFF, of_state.of_16_off);
}

void of_control(void) {
    static int of_trigger = 0;
    if (of_trigger == 1) { // turns off the previous set of pulses
        of_trigger = 0;
        of_init();
        of_send_values();
    }
    if ((of_state.update_of = CommandData.Relays.update_of) == 1) {
        of_update_values();
        of_trigger = 1;
        of_send_values();
        CommandData.Relays.update_of = 0;
    }
}

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
}

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

static void if_send_values(void) {
    heater_write(LABJACK_OF_3, RELAY_1_ON, if_state.if_1_on);
    heater_write(LABJACK_OF_3, RELAY_1_OFF, if_state.if_1_off);
    heater_write(LABJACK_OF_3, RELAY_2_ON, if_state.if_2_on);
    heater_write(LABJACK_OF_3, RELAY_2_OFF, if_state.if_2_off);
    heater_write(LABJACK_OF_3, RELAY_3_ON, if_state.if_3_on);
    heater_write(LABJACK_OF_3, RELAY_3_OFF, if_state.if_3_off);
    heater_write(LABJACK_OF_3, RELAY_4_ON, if_state.if_4_on);
    heater_write(LABJACK_OF_3, RELAY_4_OFF, if_state.if_4_off);
    heater_write(LABJACK_OF_3, RELAY_5_ON, if_state.if_5_on);
    heater_write(LABJACK_OF_3, RELAY_5_OFF, if_state.if_5_off);
    heater_write(LABJACK_OF_3, RELAY_6_ON, if_state.if_6_on);
    heater_write(LABJACK_OF_3, RELAY_6_OFF, if_state.if_6_off);
    heater_write(LABJACK_OF_3, RELAY_7_ON, if_state.if_7_on);
    heater_write(LABJACK_OF_3, RELAY_7_OFF, if_state.if_7_off);
    heater_write(LABJACK_OF_3, RELAY_8_ON, if_state.if_8_on);
    heater_write(LABJACK_OF_3, RELAY_8_OFF, if_state.if_8_off);
    heater_write(LABJACK_OF_3, IF_RELAY_9_ON, if_state.if_9_on);
    heater_write(LABJACK_OF_3, IF_RELAY_9_OFF, if_state.if_9_off);
    heater_write(LABJACK_OF_3, IF_RELAY_10_ON, if_state.if_10_on);
    heater_write(LABJACK_OF_3, IF_RELAY_10_OFF, if_state.if_10_off);
}

void if_control(void) {
    static int if_trigger = 0;
    if (if_trigger == 1) { // turns off the previous set of pulses
        if_trigger = 0;
        if_init();
        if_send_values();
    }
    if ((if_state.update_if = CommandData.Relays.update_if) == 1) {
        if_update_values();
        if_trigger = 1;
        if_send_values();
        CommandData.Relays.update_if = 0;
    }
}








