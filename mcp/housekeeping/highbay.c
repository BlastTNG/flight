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
#define HE_POT_HI_FLOW_CHAN 3 // hi flow
#define HE_PURGE_FLOW_CHAN 5 // purge flow
#define ALARM_GAUGE 4
#define HE_POT_LO_FLOW_CHAN 2 // lo flow

extern labjack_state_t state[NUM_LABJACKS];

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

static void aalborg_he_pot_hi(void) {
    static channel_t* flow_he_pot_hi_Addr;
    static int first_time_he_pot_hi = 1;
    if (first_time_he_pot_hi) {
        flow_he_pot_hi_Addr = channels_find_by_name("he_pot_hi_flow_v");
        first_time_he_pot_hi = 0;
    }
    SET_SCALED_VALUE(flow_he_pot_hi_Addr, labjack_get_value(LABJACK_HIGHBAY, HE_POT_HI_FLOW_CHAN));
}

static void aalborg_he_pot_lo(void) {
    static channel_t* flow_he_pot_lo_Addr;
    static int first_time_he_pot_lo = 1;
    if (first_time_he_pot_lo) {
        flow_he_pot_lo_Addr = channels_find_by_name("he_pot_lo_flow_v");
        first_time_he_pot_lo = 0;
    }
    SET_SCALED_VALUE(flow_he_pot_lo_Addr, labjack_get_value(LABJACK_HIGHBAY, HE_POT_LO_FLOW_CHAN));
}

static void aalborg_he_purge(void) {
    static channel_t* flow_he_purge_Addr;
    static int first_time_he_purge = 1;
    if (first_time_he_purge) {
        flow_he_purge_Addr = channels_find_by_name("he_purge_flow_v");
        first_time_he_purge = 0;
    }
    SET_SCALED_VALUE(flow_he_purge_Addr, labjack_get_value(LABJACK_HIGHBAY, HE_PURGE_FLOW_CHAN));
}

static void read_alarm_gauge(void) {
    static int first_gauge = 1;
    static channel_t* gauge_Addr;
    if (first_gauge) {
        gauge_Addr = channels_find_by_name("alarm_gauge");
    }
    SET_SCALED_VALUE(gauge_Addr, labjack_get_value(LABJACK_HIGHBAY, ALARM_GAUGE));
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


void highbay(int on) {
    if (on && state[7].connected) {
        aalborg_n2();
        aalborg_he_pot_hi();
        aalborg_he_blow();
        aalborg_he_purge();
        read_alarm_gauge();
        aalborg_he_pot_lo();
    }
}

static void write_to_mux(int value) {
    switch (value) {
        case 1:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 0);
            break;
        case 2:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 0);
            break;
        case 3:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 1);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 0);
            break;
        case 4:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 1);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 0);
            break;
        case 5:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 1);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 0);
            break;
        case 6:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 1);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 0);
            break;
        case 7:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 1);
            labjack_queue_command(8, 2002, 1);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 0);
            break;
        case 8:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 1);
            labjack_queue_command(8, 2002, 1);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 0);
            break;
        case 9:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 1);
            labjack_queue_command(8, 2004, 0);
            break;
        case 10:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 1);
            labjack_queue_command(8, 2004, 0);
            break;
        case 11:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 1);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 1);
            labjack_queue_command(8, 2004, 0);
            break;
        case 12:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 1);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 1);
            labjack_queue_command(8, 2004, 0);
            break;
        case 13:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 1);
            labjack_queue_command(8, 2003, 1);
            labjack_queue_command(8, 2004, 0);
            break;
        case 14:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 1);
            labjack_queue_command(8, 2003, 1);
            labjack_queue_command(8, 2004, 0);
            break;
        case 15:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 1);
            labjack_queue_command(8, 2002, 1);
            labjack_queue_command(8, 2003, 1);
            labjack_queue_command(8, 2004, 0);
            break;
        case 16:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 1);
            labjack_queue_command(8, 2002, 1);
            labjack_queue_command(8, 2003, 1);
            labjack_queue_command(8, 2004, 0);
            break;
        case 17:
            labjack_queue_command(8, 2000, 0);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 1);
            break;
        case 18:
            labjack_queue_command(8, 2000, 1);
            labjack_queue_command(8, 2001, 0);
            labjack_queue_command(8, 2002, 0);
            labjack_queue_command(8, 2003, 0);
            labjack_queue_command(8, 2004, 1);
            break;
        default:
            break;
    }
}

static void setup_write(void) { // sets the mux pins to s1 0, s2 1, w1 0, w2 0
    labjack_queue_command(8, 2005, 0);
    labjack_queue_command(8, 2006, 0);
    labjack_queue_command(8, 2007, 0);
    labjack_queue_command(8, 2020, 1);
}

static void write_1(void) {
    labjack_queue_command(8, 2005, 1);
    labjack_queue_command(8, 2006, 1);
    labjack_queue_command(8, 2020, 0);
}

static void write_2(void) {
    labjack_queue_command(8, 2007, 1);
    labjack_queue_command(8, 2020, 1);
}

static void choose_volts(int polarity, float voltage) {
    if (polarity == 1) { // polarity 1 grounds 2 and writes to 1
        labjack_queue_command(8, 1000, voltage);
        labjack_queue_command(8, 1001, 0);
    }
    if (polarity == -1) { // polarity -1 grounds 1 and writes to 2
        labjack_queue_command(8, 1001, voltage);
        labjack_queue_command(8, 1000, 0);
    }
}


void mapper_command(int mux1, int mux2, int polarity, float voltage) {
    if (mux1 == mux2) {
        blast_info("Error can't send current in and out on same line");
    } else {
    if (state[8].connected == 1) {
        choose_volts(polarity, voltage);
        setup_write();
        write_to_mux(mux1);
        write_1();
        write_to_mux(mux1);
        write_2();
        blast_info("wrote %d to mux1, %d to mux2, with polarity %d, and voltage %f", mux1, mux2, polarity, voltage);
    }
    }
}

void all_map_5v(void) {
    static int polarity = 1;
    static int mux1 = 1;
    static int mux2 = 1;
    static float voltage = 5.0;
    if (polarity == 1) {
        if (mux1 < 19) {
            if (mux2 < 19 && mux2 != mux1) {
                // mapper_command(mux1, mux2, polarity, voltage);
                mux2++;
            }
            if (mux2 == 19) {
                mux2 = 1;
                mux1++;
            }
        }
        if (mux1 == 19) {
            mux1 = 1;
            mux2 = 1;
            polarity = -1;
        }
    }
    if (polarity == -1) {
        blast_info("failed");
    }
}














