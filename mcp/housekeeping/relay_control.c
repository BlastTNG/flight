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
static int ifrelay1_on, ifrelay1_off, ifrelay2_on, ifrelay2_off, ifrelay3_on, ifrelay3_off;
static int ifrelay4_on, ifrelay4_off, ifrelay5_on, ifrelay5_off, ifrelay6_on, ifrelay6_off;
static int ifrelay7_on, ifrelay7_off, ifrelay8_on, ifrelay8_off, if_1_5_signal, if_6_10_signal;
static int ifrelay9_on, ifrelay9_off, ifrelay10_on, ifrelay10_off;

// controls for the REC box

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

// controls for relays 1-4 of the OF power box


void of_1_4_switch(int which) {
    switch (which) {
        case 1:
            of_1_4_signal = 1;
            ofrelay1_on = 1;
            break;
        case 2:
            of_1_4_signal = 1;
            ofrelay1_off = 1;
            break;
        case 3:
            of_1_4_signal = 1;
            ofrelay2_on = 1;
            break;
        case 4:
            of_1_4_signal = 1;
            ofrelay2_off = 1;
            break;
        case 5:
            of_1_4_signal = 1;
            ofrelay3_on = 1;
            break;
        case 6:
            of_1_4_signal = 1;
            ofrelay3_off = 1;
            break;
        case 7:
            of_1_4_signal = 1;
            ofrelay4_on = 1;
            break;
        case 8:
            of_1_4_signal = 1;
            ofrelay4_off = 1;
            break;
    }
}

static void init_of_1_4(void) {
    ofrelay1_on = 0;
    ofrelay1_off = 0;
    ofrelay2_on = 0;
    ofrelay2_off = 0;
    ofrelay3_on = 0;
    ofrelay3_off = 0;
    ofrelay4_on = 0;
    ofrelay4_off = 0;
}

void of_1_4_control(void) {
    static int firsttime_of_1_4 = 1;
    static int was_signaled_of_1_4 = 0;
    if (firsttime_of_1_4 == 1) {
        firsttime_of_1_4 = 0;
        // heater_write(LABJACK_OF_1, RELAY_1_OFF, 0);
        // heater_write(LABJACK_OF_1, RELAY_1_ON, 1);
        of_1_4_signal = 0;
        init_of_1_4();
    }
    if (was_signaled_of_1_4 == 1) {
        heater_write(LABJACK_OF_1, RELAY_1_OFF, ofrelay1_off);
        heater_write(LABJACK_OF_1, RELAY_1_ON, ofrelay1_on);
        heater_write(LABJACK_OF_1, RELAY_2_OFF, ofrelay2_off);
        heater_write(LABJACK_OF_1, RELAY_2_ON, ofrelay2_on);
        heater_write(LABJACK_OF_1, RELAY_3_OFF, ofrelay3_off);
        heater_write(LABJACK_OF_1, RELAY_3_ON, ofrelay3_on);
        heater_write(LABJACK_OF_1, RELAY_4_OFF, ofrelay4_off);
        heater_write(LABJACK_OF_1, RELAY_4_ON, ofrelay4_on);
        was_signaled_of_1_4 = 0;
    }
    if (of_1_4_signal == 1) {
        heater_write(LABJACK_OF_1, RELAY_1_OFF, ofrelay1_off);
        heater_write(LABJACK_OF_1, RELAY_1_ON, ofrelay1_on);
        heater_write(LABJACK_OF_1, RELAY_2_OFF, ofrelay2_off);
        heater_write(LABJACK_OF_1, RELAY_2_ON, ofrelay2_on);
        heater_write(LABJACK_OF_1, RELAY_3_OFF, ofrelay3_off);
        heater_write(LABJACK_OF_1, RELAY_3_ON, ofrelay3_on);
        heater_write(LABJACK_OF_1, RELAY_4_OFF, ofrelay4_off);
        heater_write(LABJACK_OF_1, RELAY_4_ON, ofrelay4_on);
        of_1_4_signal = 0;
        was_signaled_of_1_4 = 1;
        init_of_1_4();
    }
}
// controls for relays 5-8 of the OF power box


void of_5_8_switch(int which) {
    switch (which) {
        case 1:
            of_5_8_signal = 1;
            ofrelay5_on = 1;
            break;
        case 2:
            of_5_8_signal = 1;
            ofrelay5_off = 1;
            break;
        case 3:
            of_5_8_signal = 1;
            ofrelay6_on = 1;
            break;
        case 4:
            of_5_8_signal = 1;
            ofrelay6_off = 1;
            break;
        case 5:
            of_5_8_signal = 1;
            ofrelay7_on = 1;
            break;
        case 6:
            of_5_8_signal = 1;
            ofrelay7_off = 1;
            break;
        case 7:
            of_5_8_signal = 1;
            ofrelay8_on = 1;
            break;
        case 8:
            of_5_8_signal = 1;
            ofrelay8_off = 1;
            break;
    }
}

static void init_of_5_8(void) {
    ofrelay5_on = 0;
    ofrelay5_off = 0;
    ofrelay6_on = 0;
    ofrelay6_off = 0;
    ofrelay7_on = 0;
    ofrelay7_off = 0;
    ofrelay8_on = 0;
    ofrelay8_off = 0;
}

void of_5_8_control(void) {
    static int firsttime_of_5_8 = 1;
    static int was_signaled_of_5_8 = 0;
    if (firsttime_of_5_8 == 1) {
        firsttime_of_5_8 = 0;
        // heater_write(LABJACK_OF_1, RELAY_1_OFF, 0);
        // heater_write(LABJACK_OF_1, RELAY_1_ON, 1);
        of_5_8_signal = 0;
        init_of_5_8();
    }
    if (was_signaled_of_5_8 == 1) {
        heater_write(LABJACK_OF_1, RELAY_5_OFF, ofrelay5_off);
        heater_write(LABJACK_OF_1, RELAY_5_ON, ofrelay5_on);
        heater_write(LABJACK_OF_1, RELAY_6_OFF, ofrelay6_off);
        heater_write(LABJACK_OF_1, RELAY_6_ON, ofrelay6_on);
        heater_write(LABJACK_OF_1, RELAY_7_OFF, ofrelay7_off);
        heater_write(LABJACK_OF_1, RELAY_7_ON, ofrelay7_on);
        heater_write(LABJACK_OF_2, RELAY_8_OFF, ofrelay8_off);
        heater_write(LABJACK_OF_2, RELAY_8_ON, ofrelay8_on);
        was_signaled_of_5_8 = 0;
    }
    if (of_5_8_signal == 1) {
        heater_write(LABJACK_OF_1, RELAY_5_OFF, ofrelay5_off);
        heater_write(LABJACK_OF_1, RELAY_5_ON, ofrelay5_on);
        heater_write(LABJACK_OF_1, RELAY_6_OFF, ofrelay6_off);
        heater_write(LABJACK_OF_1, RELAY_6_ON, ofrelay6_on);
        heater_write(LABJACK_OF_1, RELAY_7_OFF, ofrelay7_off);
        heater_write(LABJACK_OF_1, RELAY_7_ON, ofrelay7_on);
        heater_write(LABJACK_OF_2, RELAY_8_OFF, ofrelay8_off);
        heater_write(LABJACK_OF_2, RELAY_8_ON, ofrelay8_on);
        of_5_8_signal = 0;
        was_signaled_of_5_8 = 1;
        init_of_5_8();
    }
}

// controls for relays 9-12 of the OF power box


void of_9_12_switch(int which) {
    switch (which) {
        case 1:
            of_9_12_signal = 1;
            ofrelay9_on = 1;
            break;
        case 2:
            of_9_12_signal = 1;
            ofrelay9_off = 1;
            break;
        case 3:
            of_9_12_signal = 1;
            ofrelay10_on = 1;
            break;
        case 4:
            of_9_12_signal = 1;
            ofrelay10_off = 1;
            break;
        case 5:
            of_9_12_signal = 1;
            ofrelay11_on = 1;
            break;
        case 6:
            of_9_12_signal = 1;
            ofrelay11_off = 1;
            break;
        case 7:
            of_9_12_signal = 1;
            ofrelay12_on = 1;
            break;
        case 8:
            of_9_12_signal = 1;
            ofrelay12_off = 1;
            break;
    }
}

static void init_of_9_12(void) {
    ofrelay9_on = 0;
    ofrelay9_off = 0;
    ofrelay10_on = 0;
    ofrelay10_off = 0;
    ofrelay11_on = 0;
    ofrelay11_off = 0;
    ofrelay12_on = 0;
    ofrelay12_off = 0;
}

void of_9_12_control(void) {
    static int firsttime_of_9_12 = 1;
    static int was_signaled_of_9_12 = 0;
    if (firsttime_of_9_12 == 1) {
        firsttime_of_9_12 = 0;
        // heater_write(LABJACK_OF_1, RELAY_1_OFF, 0);
        // heater_write(LABJACK_OF_1, RELAY_1_ON, 1);
        of_9_12_signal = 0;
        init_of_9_12();
    }
    if (was_signaled_of_9_12 == 1) {
        heater_write(LABJACK_OF_2, RELAY_9_OFF, ofrelay9_off);
        heater_write(LABJACK_OF_2, RELAY_9_ON, ofrelay9_on);
        heater_write(LABJACK_OF_2, RELAY_10_OFF, ofrelay10_off);
        heater_write(LABJACK_OF_2, RELAY_10_ON, ofrelay10_on);
        heater_write(LABJACK_OF_2, RELAY_11_OFF, ofrelay11_off);
        heater_write(LABJACK_OF_2, RELAY_11_ON, ofrelay11_on);
        heater_write(LABJACK_OF_2, RELAY_12_OFF, ofrelay12_off);
        heater_write(LABJACK_OF_2, RELAY_12_ON, ofrelay12_on);
        was_signaled_of_9_12 = 0;
    }
    if (of_9_12_signal == 1) {
        heater_write(LABJACK_OF_2, RELAY_9_OFF, ofrelay9_off);
        heater_write(LABJACK_OF_2, RELAY_9_ON, ofrelay9_on);
        heater_write(LABJACK_OF_2, RELAY_10_OFF, ofrelay10_off);
        heater_write(LABJACK_OF_2, RELAY_10_ON, ofrelay10_on);
        heater_write(LABJACK_OF_2, RELAY_11_OFF, ofrelay11_off);
        heater_write(LABJACK_OF_2, RELAY_11_ON, ofrelay11_on);
        heater_write(LABJACK_OF_2, RELAY_12_OFF, ofrelay12_off);
        heater_write(LABJACK_OF_2, RELAY_12_ON, ofrelay12_on);
        of_9_12_signal = 0;
        was_signaled_of_9_12 = 1;
        init_of_9_12();
    }
}

// controls for relays 13-16 of the OF power box


void of_13_16_switch(int which) {
    switch (which) {
        case 1:
            of_13_16_signal = 1;
            ofrelay13_on = 1;
            break;
        case 2:
            of_13_16_signal = 1;
            ofrelay13_off = 1;
            break;
        case 3:
            of_13_16_signal = 1;
            ofrelay14_on = 1;
            break;
        case 4:
            of_13_16_signal = 1;
            ofrelay14_off = 1;
            break;
        case 5:
            of_13_16_signal = 1;
            ofrelay15_on = 1;
            break;
        case 6:
            of_13_16_signal = 1;
            ofrelay15_off = 1;
            break;
        case 7:
            of_13_16_signal = 1;
            ofrelay16_on = 1;
            break;
        case 8:
            of_13_16_signal = 1;
            ofrelay16_off = 1;
            break;
    }
}

static void init_of_13_16(void) {
    ofrelay13_on = 0;
    ofrelay13_off = 0;
    ofrelay14_on = 0;
    ofrelay14_off = 0;
    ofrelay15_on = 0;
    ofrelay15_off = 0;
    ofrelay16_on = 0;
    ofrelay16_off = 0;
}

void of_13_16_control(void) {
    static int firsttime_of_13_16 = 1;
    static int was_signaled_of_13_16 = 0;
    if (firsttime_of_13_16 == 1) {
        firsttime_of_13_16 = 0;
        // heater_write(LABJACK_OF_1, RELAY_1_OFF, 0);
        // heater_write(LABJACK_OF_1, RELAY_1_ON, 1);
        of_13_16_signal = 0;
        init_of_13_16();
    }
    if (was_signaled_of_13_16 == 1) {
        heater_write(LABJACK_OF_2, RELAY_13_OFF, ofrelay13_off);
        heater_write(LABJACK_OF_2, RELAY_13_ON, ofrelay13_on);
        heater_write(LABJACK_OF_2, RELAY_14_OFF, ofrelay14_off);
        heater_write(LABJACK_OF_2, RELAY_14_ON, ofrelay14_on);
        heater_write(LABJACK_OF_2, RELAY_15_OFF, ofrelay15_off);
        heater_write(LABJACK_OF_2, RELAY_15_ON, ofrelay15_on);
        heater_write(LABJACK_OF_1, RELAY_16_OFF, ofrelay16_off);
        heater_write(LABJACK_OF_1, RELAY_16_ON, ofrelay16_on);
        was_signaled_of_13_16 = 0;
    }
    if (of_13_16_signal == 1) {
        heater_write(LABJACK_OF_2, RELAY_13_OFF, ofrelay13_off);
        heater_write(LABJACK_OF_2, RELAY_13_ON, ofrelay13_on);
        heater_write(LABJACK_OF_2, RELAY_14_OFF, ofrelay14_off);
        heater_write(LABJACK_OF_2, RELAY_14_ON, ofrelay14_on);
        heater_write(LABJACK_OF_2, RELAY_15_OFF, ofrelay15_off);
        heater_write(LABJACK_OF_2, RELAY_15_ON, ofrelay15_on);
        heater_write(LABJACK_OF_1, RELAY_16_OFF, ofrelay16_off);
        heater_write(LABJACK_OF_1, RELAY_16_ON, ofrelay16_on);
        of_13_16_signal = 0;
        was_signaled_of_13_16 = 1;
        init_of_13_16();
    }
}

// controls for relays 1-5 of the IF power box

void if_1_5_switch(int which) {
    switch (which) {
        case 1:
            if_1_5_signal = 1;
            ifrelay1_on = 1;
            break;
        case 2:
            if_1_5_signal = 1;
            ifrelay1_off = 1;
            break;
        case 3:
            if_1_5_signal = 1;
            ifrelay2_on = 1;
            break;
        case 4:
            if_1_5_signal = 1;
            ifrelay2_off = 1;
            break;
        case 5:
            if_1_5_signal = 1;
            ifrelay3_on = 1;
            break;
        case 6:
            if_1_5_signal = 1;
            ifrelay3_off = 1;
            break;
        case 7:
            if_1_5_signal = 1;
            ifrelay4_on = 1;
            break;
        case 8:
            if_1_5_signal = 1;
            ifrelay4_off = 1;
            break;
        case 9:
            if_1_5_signal = 1;
            ifrelay5_on = 1;
            break;
        case 10:
            if_1_5_signal = 1;
            ifrelay5_off = 1;
            break;
    }
}

static void init_if_1_5(void) {
    ifrelay1_on = 0;
    ifrelay1_off = 0;
    ifrelay2_on = 0;
    ifrelay2_off = 0;
    ifrelay3_on = 0;
    ifrelay3_off = 0;
    ifrelay4_on = 0;
    ifrelay4_off = 0;
    ifrelay5_on = 0;
    ifrelay5_off = 0;
}

void if_1_5_control(void) {
    static int firsttime_if_1_5 = 1;
    static int was_signaled_if_1_5 = 0;
    if (firsttime_if_1_5 == 1) {
        firsttime_if_1_5 = 0;
        // heater_write(LABJACK_OF_1, RELAY_1_OFF, 0);
        // heater_write(LABJACK_OF_1, RELAY_1_ON, 1);
        if_1_5_signal = 0;
        init_if_1_5();
    }
    if (was_signaled_if_1_5 == 1) {
        heater_write(LABJACK_OF_3, RELAY_1_OFF, ifrelay1_on);
        heater_write(LABJACK_OF_3, RELAY_1_ON, ifrelay1_off);
        heater_write(LABJACK_OF_3, RELAY_2_OFF, ifrelay2_on);
        heater_write(LABJACK_OF_3, RELAY_2_ON, ifrelay2_off);
        heater_write(LABJACK_OF_3, RELAY_3_OFF, ifrelay3_on);
        heater_write(LABJACK_OF_3, RELAY_3_ON, ifrelay3_off);
        heater_write(LABJACK_OF_3, RELAY_4_OFF, ifrelay4_on);
        heater_write(LABJACK_OF_3, RELAY_4_ON, ifrelay4_off);
        heater_write(LABJACK_OF_3, RELAY_5_OFF, ifrelay5_on);
        heater_write(LABJACK_OF_3, RELAY_5_ON, ifrelay5_off);
        was_signaled_if_1_5 = 0;
    }
    if (if_1_5_signal == 1) {
        heater_write(LABJACK_OF_3, RELAY_1_OFF, ifrelay1_on);
        heater_write(LABJACK_OF_3, RELAY_1_ON, ifrelay1_off);
        heater_write(LABJACK_OF_3, RELAY_2_OFF, ifrelay2_on);
        heater_write(LABJACK_OF_3, RELAY_2_ON, ifrelay2_off);
        heater_write(LABJACK_OF_3, RELAY_3_OFF, ifrelay3_on);
        heater_write(LABJACK_OF_3, RELAY_3_ON, ifrelay3_off);
        heater_write(LABJACK_OF_3, RELAY_4_OFF, ifrelay4_on);
        heater_write(LABJACK_OF_3, RELAY_4_ON, ifrelay4_off);
        heater_write(LABJACK_OF_3, RELAY_5_OFF, ifrelay5_on);
        heater_write(LABJACK_OF_3, RELAY_5_ON, ifrelay5_off);
        if_1_5_signal = 0;
        was_signaled_if_1_5 = 1;
        init_if_1_5();
    }
}

// controls for relays 6-10 of the IF power box

void if_6_10_switch(int which) {
    switch (which) {
        case 1:
            if_6_10_signal = 1;
            ifrelay6_on = 1;
            break;
        case 2:
            if_6_10_signal = 1;
            ifrelay6_off = 1;
            break;
        case 3:
            if_6_10_signal = 1;
            ifrelay7_on = 1;
            break;
        case 4:
            if_6_10_signal = 1;
            ifrelay7_off = 1;
            break;
        case 5:
            if_6_10_signal = 1;
            ifrelay8_on = 1;
            break;
        case 6:
            if_6_10_signal = 1;
            ifrelay8_off = 1;
            break;
        case 7:
            if_6_10_signal = 1;
            ifrelay9_on = 1;
            break;
        case 8:
            if_6_10_signal = 1;
            ifrelay9_off = 1;
            break;
        case 9:
            if_6_10_signal = 1;
            ifrelay10_on = 1;
            break;
        case 10:
            if_6_10_signal = 1;
            ifrelay10_off = 1;
            break;
    }
}

static void init_if_6_10(void) {
    ifrelay6_on = 0;
    ifrelay6_off = 0;
    ifrelay7_on = 0;
    ifrelay7_off = 0;
    ifrelay8_on = 0;
    ifrelay8_off = 0;
    ifrelay9_on = 0;
    ifrelay9_off = 0;
    ifrelay10_on = 0;
    ifrelay10_off = 0;
}

void if_6_10_control(void) {
    static int firsttime_if_6_10 = 1;
    static int was_signaled_if_6_10 = 0;
    if (firsttime_if_6_10 == 1) {
        firsttime_if_6_10 = 0;
        // heater_write(LABJACK_OF_1, RELAY_1_OFF, 0);
        // heater_write(LABJACK_OF_1, RELAY_1_ON, 1);
        if_6_10_signal = 0;
        init_if_6_10();
    }
    if (was_signaled_if_6_10 == 1) {
        heater_write(LABJACK_OF_3, RELAY_6_OFF, ifrelay6_on);
        heater_write(LABJACK_OF_3, RELAY_6_ON, ifrelay6_off);
        heater_write(LABJACK_OF_3, RELAY_7_OFF, ifrelay7_on);
        heater_write(LABJACK_OF_3, RELAY_7_ON, ifrelay7_off);
        heater_write(LABJACK_OF_3, RELAY_8_OFF, ifrelay8_on);
        heater_write(LABJACK_OF_3, RELAY_8_ON, ifrelay8_off);
        heater_write(LABJACK_OF_3, RELAY_9_OFF, ifrelay9_on);
        heater_write(LABJACK_OF_3, RELAY_9_ON, ifrelay9_off);
        heater_write(LABJACK_OF_3, RELAY_10_OFF, ifrelay10_on);
        heater_write(LABJACK_OF_3, RELAY_10_ON, ifrelay10_off);
        was_signaled_if_6_10 = 0;
    }
    if (if_6_10_signal == 1) {
        heater_write(LABJACK_OF_3, RELAY_6_OFF, ifrelay6_on);
        heater_write(LABJACK_OF_3, RELAY_6_ON, ifrelay6_off);
        heater_write(LABJACK_OF_3, RELAY_7_OFF, ifrelay7_on);
        heater_write(LABJACK_OF_3, RELAY_7_ON, ifrelay7_off);
        heater_write(LABJACK_OF_3, RELAY_8_OFF, ifrelay8_on);
        heater_write(LABJACK_OF_3, RELAY_8_ON, ifrelay8_off);
        heater_write(LABJACK_OF_3, RELAY_9_OFF, ifrelay9_on);
        heater_write(LABJACK_OF_3, RELAY_9_ON, ifrelay9_off);
        heater_write(LABJACK_OF_3, RELAY_10_OFF, ifrelay10_on);
        heater_write(LABJACK_OF_3, RELAY_10_ON, ifrelay10_off);
        if_6_10_signal = 0;
        was_signaled_if_6_10 = 1;
        init_if_6_10();
    }
}
