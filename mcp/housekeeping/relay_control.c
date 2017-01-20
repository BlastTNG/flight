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

static int powerbox_on, powerbox_off, ampsupply_on, ampsupply_off;
static int thermreadout_on, thermreadout_off, heatersupply_on, heatersupply_off;
static int rec_signal;

static int ofrelay1_on, ofrelay1_off, ofrelay2_on, ofrelay2_off, ofrelay3_on, ofrelay3_off;
static int ofrelay4_on, ofrelay4_off, ofrelay5_on, ofrelay5_off, ofrelay6_on, ofrelay6_off;
static int ofrelay7_on, ofrelay7_off, ofrelay8_on, ofrelay8_off, of_1_4_signal, of_5_8_signal;
static int ofrelay9_on, ofrelay9_off, ofrelay10_on, ofrelay10_off, ofrelay11_on, ofrelay11_off;
static int ofrelay12_on, ofrelay12_off, ofrelay13_on, ofrelay13_off, ofrelay14_on, ofrelay14_off;
static int ofrelay15_on, ofrelay15_off, ofrelay16_on, ofrelay16_off, of_9_12_signal, of_13_16_signal;

void rec_switch(int which) {
    switch (which) {
        case 1:
            rec_signal = 1;
            powerbox_on = 1;
            break;
        case 2:
            rec_signal = 1;
            powerbox_off = 1;
            break;
        case 3:
            rec_signal = 1;
            ampsupply_on = 1;
            break;
        case 4:
            rec_signal = 1;
            ampsupply_off = 1;
            break;
        case 5:
            rec_signal = 1;
            thermreadout_on = 1;
            break;
        case 6:
            rec_signal = 1;
            thermreadout_off = 1;
            break;
        case 7:
            rec_signal = 1;
            heatersupply_on = 1;
            break;
        case 8:
            rec_signal = 1;
            heatersupply_off = 1;
            break;
    }
}

static void init_rec(void) {
    powerbox_on = 0;
    powerbox_off = 0;
    ampsupply_on = 0;
    ampsupply_off = 0;
    thermreadout_on = 0;
    thermreadout_off = 0;
    heatersupply_on = 0;
    heatersupply_off = 0;
}

void rec_control(void) {
    static int firsttime_rec = 1;
    static int was_signaled = 0;
    if (firsttime_rec == 1) {
        firsttime_rec = 0;
        heater_write(LABJACK_CRYO_2, POWER_BOX_OFF, 0);
        heater_write(LABJACK_CRYO_2, POWER_BOX_ON, 1);
        rec_signal = 0;
        init_rec();
    }
    if (was_signaled == 1) {
        heater_write(LABJACK_CRYO_2, POWER_BOX_OFF, powerbox_off);
        heater_write(LABJACK_CRYO_2, POWER_BOX_ON, powerbox_on);
        heater_write(LABJACK_CRYO_2, AMP_SUPPLY_OFF, ampsupply_off);
        heater_write(LABJACK_CRYO_2, AMP_SUPPLY_ON, ampsupply_on);
        heater_write(LABJACK_CRYO_2, THERM_READOUT_OFF, thermreadout_off);
        heater_write(LABJACK_CRYO_2, THERM_READOUT_ON, thermreadout_on);
        heater_write(LABJACK_CRYO_2, HEATER_SUPPLY_OFF, heatersupply_off);
        heater_write(LABJACK_CRYO_2, HEATER_SUPPLY_ON, heatersupply_on);
        was_signaled = 0;
    }
    if (rec_signal == 1) {
        heater_write(LABJACK_CRYO_2, POWER_BOX_OFF, powerbox_off);
        heater_write(LABJACK_CRYO_2, POWER_BOX_ON, powerbox_on);
        heater_write(LABJACK_CRYO_2, AMP_SUPPLY_OFF, ampsupply_off);
        heater_write(LABJACK_CRYO_2, AMP_SUPPLY_ON, ampsupply_on);
        heater_write(LABJACK_CRYO_2, THERM_READOUT_OFF, thermreadout_off);
        heater_write(LABJACK_CRYO_2, THERM_READOUT_ON, thermreadout_on);
        heater_write(LABJACK_CRYO_2, HEATER_SUPPLY_OFF, heatersupply_off);
        heater_write(LABJACK_CRYO_2, HEATER_SUPPLY_ON, heatersupply_on);
        rec_signal = 0;
        was_signaled = 1;
        init_rec();
    }
}








