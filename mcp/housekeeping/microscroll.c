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
extern labjack_state_t state[NUM_LABJACKS];





typedef struct {
    float supply_24va, supply_24vb;
    float relay_12v_on, relay_12v_off;
    int have_pulsed_relay;
} microscroll_control_t;

microscroll_control_t microscroll;


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
    static channel_t* pumpa_Addr, pumpb_Addr;
    if (first_time) {
        pumpa_Addr = channels_find_by_name("microscroll_a");
        pumpb_Addr = channels_find_by_name("microscroll_b");
    }
    if (state[9].connected) {
        SET_SCALED_VALUE(pumpa_Addr, microscroll.supply_24va_on)
    }
}

static void update_microscroll() {
    micrscroll.supply_24va_on = CommandData.Microscroll.supply_24va_on;
    micrscroll.supply_24va_off = CommandData.Microscroll.supply_24va_off;
    micrscroll.supply_24vb_on = CommandData.Microscroll.supply_24vb_on;
    micrscroll.supply_24vb_off = CommandData.Microscroll.supply_24vb_off;
    microscroll.relay_12v_on = CommandData.Microscroll.relay_12v_on;
    microscroll.relay_12v_off = CommandData.Microscroll.relay_12v_off;
}

static void control_24va_supply() {
    if (state[9].connected) {
        labjack_queue_command(LABJACK_MICROSCROLL, supply_24Va, microscroll.supply_24va);
    }
}

static void control_12v_relay() {
    if (state[9].connected) {
        int a = 1;
    }
}

static void control_24vb_supply() {
    if (state[9].connected) {
        labjack_queue_command(LABJACK_MICROSCROLL, supply_24Vb, microscroll.supply_24vb);
    }
}

static void update_microscroll_thermistors() {
    static int first_time = 1;
    static channel_t* therm1_Addr, therm2_Addr, therm3_Addr, therm4_Addr;
    static channel_t* therm5_Addr, therm6_Addr, therm7_Addr, therm8_Addr;
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
        SET_SCALED_VALUE(therm1_Addr,labjack_get_value(LABJACK_MICROSCROLL,thermistor_1));
        SET_SCALED_VALUE(therm2_Addr,labjack_get_value(LABJACK_MICROSCROLL,thermistor_2));
        SET_SCALED_VALUE(therm3_Addr,labjack_get_value(LABJACK_MICROSCROLL,thermistor_3));
        SET_SCALED_VALUE(therm4_Addr,labjack_get_value(LABJACK_MICROSCROLL,thermistor_4));
        SET_SCALED_VALUE(therm5_Addr,labjack_get_value(LABJACK_MICROSCROLL,thermistor_5));
        SET_SCALED_VALUE(therm6_Addr,labjack_get_value(LABJACK_MICROSCROLL,thermistor_6));
        SET_SCALED_VALUE(therm7_Addr,labjack_get_value(LABJACK_MICROSCROLL,thermistor_7));
        SET_SCALED_VALUE(therm8_Addr,labjack_get_value(LABJACK_MICROSCROLL,thermistor_8));
    }
}

