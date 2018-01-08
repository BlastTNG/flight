/***************************************************************************
 mcp: the BLAST master control program
 
 This software is copyright (C) 2002-2006 University of Pennsylvania
 
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
 
 created by Ian Lowe 5-13-16
 **************************************************************************/


/*************************************************************************
 
 highbay.c -- code to control and log the meters in the highbay
 
 *************************************************************************/

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "highbay.h"

#define N2_FLOW_CHAN 0
#define HE_BLOW_CHAN 1
#define HE_POT_FLOW_CHAN 2
#define HE_PURGE_FLOW_CHAN 3
#define ALARM_GAUGE 4

static void aalborg_n2(void) {
    static channel_t* flow_n2_Addr;
    static int first_time_n2 = 1;
    if (first_time_n2) {
        flow_n2_Addr = channels_find_by_name("n2_flow_v");
        first_time_n2 = 0;
    }
    SET_SCALED_VALUE(flow_n2_Addr, labjack_get_value(LABJACK_HIGHBAY, N2_FLOW_CHAN));
}

static void aalborg_he_blow(void) {
    static channel_t* he_blow_Addr;
    static int first_time_he_blow = 1;
    if (first_time_he_blow) {
        he_blow_Addr = channels_find_by_name("he_blow_v");
        first_time_he_blow = 0;
    }
    SET_SCALED_VALUE(he_blow_Addr, labjack_get_value(LABJACK_HIGHBAY, HE_BLOW_CHAN));
}

static void aalborg_he_pot(void) {
    static channel_t* flow_he_pot_Addr;
    static int first_time_he_pot = 1;
    if (first_time_he_pot) {
        flow_he_pot_Addr = channels_find_by_name("he_pot_flow_v");
        first_time_he_pot = 0;
    }
    // SET_SCALED_VALUE(flow_he_pot_Addr, labjack_get_value(LABJACK_HIGHBAY, HE_POT_FLOW_CHAN));
}

static void aalborg_he_purge(void) {
    static channel_t* flow_he_purge_Addr;
    static int first_time_he_purge = 1;
    if (first_time_he_purge) {
        flow_he_purge_Addr = channels_find_by_name("he_purge_flow_v");
        first_time_he_purge = 0;
    }
    // SET_SCALED_VALUE(flow_he_purge_Addr, labjack_get_value(LABJACK_HIGHBAY, HE_PURGE_FLOW_CHAN));
}

static void read_alarm_gauge(void) {
    static int first_gauge = 1;
    static channel_t* gauge_Addr;
    if (first_gauge) {
        gauge_Addr = channels_find_by_name("alarm_gauge");
    }
    SET_SCALED_VALUE(gauge_Addr, labjack_get_value(LABJACK_HIGHBAY, 3));
}

void monitor_flow(int on) {
    if (on) {
        static channel_t* flow_n2_Addr;
        static int first_time_n2 = 1;
        static float n2_volts_critical = 0; // set actual value
        static int burn_counter = -1;
        float n2_volts;
        if (first_time_n2) {
            flow_n2_Addr = channels_find_by_name("n2_flow_v");
            first_time_n2 = 0;
        }
        n2_volts = labjack_get_value(LABJACK_HIGHBAY, N2_FLOW_CHAN);
        if (burn_counter > 0) {
            burn_counter--;
        }
        if (burn_counter == 0) {
            labjack_queue_command(LABJACK_HIGHBAY, 2008, 0); // set a real DIO here instead of 2008
            burn_counter = -1;
        }
        if (n2_volts < n2_volts_critical && burn_counter == -1) {
            labjack_queue_command(LABJACK_HIGHBAY, 2008, 1); // set a real DIO here instead of 2008
            burn_counter = 10;
        }
    }
}


void highbay(int n2, int he_pot, int he_blow, int he_purge, int alarm) {
    if (n2) {
        aalborg_n2();
    }
    if (he_pot) {
        aalborg_he_pot();
    }
    if (he_blow) {
        aalborg_he_blow();
    }
    if (he_purge) {
        aalborg_he_purge();
    }
    if (alarm) {
        read_alarm_gauge();
    }
}
