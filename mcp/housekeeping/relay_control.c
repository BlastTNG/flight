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

static int power_box_on, power_box_off, amp_supply_on, amp_supply_off;
static int therm_readout_on, therm_readout_off, heater_supply_on, heater_supply_off;
static int signal, was_signaled;

void rec_switch(int which) {
    switch (which) {
        case 0:
            signal = 1;
            break;
        case 1:
            signal = 1;
            break;
        case 2:
            signal = 1;
            break;
        case 3:
            signal = 1;
            break;
        case 4:
            signal = 1;
            break;
        case 5:
            signal = 1;
            break;
        case 6:
            signal = 1;
            break;
        case 7:
            signal = 1;
            break;
    }
}

void rec_control(void) {
    static int firsttime_rec = 1;
    if (firsttime_rec) {
        firsttime_rec = 0;
        heater_write(LABJACK_CRYO_2, POWER_BOX_OFF, 0);
        heater_write(LABJACK_CRYO_2, POWER_BOX_ON, 1);
        heater_write(LABJACK_CRYO_2, AMP_SUPPLY_OFF, 0);
        heater_write(LABJACK_CRYO_2, AMP_SUPPLY_ON, 1);
        heater_write(LABJACK_CRYO_2, THERM_READOUT_OFF, 0);
        heater_write(LABJACK_CRYO_2, THERM_READOUT_ON, 1);
        heater_write(LABJACK_CRYO_2, HEATER_SUPPLY_OFF, 0);
        heater_write(LABJACK_CRYO_2, HEATER_SUPPLY_ON, 1);
    }
}








