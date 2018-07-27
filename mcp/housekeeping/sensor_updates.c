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
 
 created by Ian Lowe 1-12-17
 **************************************************************************/
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
#include "sensor_updates.h"
#include "pointing_struct.h"

extern labjack_state_t state[NUM_LABJACKS];

static labjack_10hz_filter_t vPSSfilt[NUM_LABJACKS][NUM_PSS_V];
static labjack_10hz_filter_t TPSSfilt[NUM_LABJACKS];

void update_sun_sensors(void) {
    static int firsttime = 1;
    static channel_t* v1_1_Addr;
    static channel_t* v2_1_Addr;
    static channel_t* v3_1_Addr;
    static channel_t* v4_1_Addr;
    static channel_t* v5_1_Addr;
    static channel_t* v1_2_Addr;
    static channel_t* v2_2_Addr;
    static channel_t* v3_2_Addr;
    static channel_t* v4_2_Addr;
    static channel_t* v5_2_Addr;
    static channel_t* v1_3_Addr;
    static channel_t* v2_3_Addr;
    static channel_t* v3_3_Addr;
    static channel_t* v4_3_Addr;
    static channel_t* v5_3_Addr;
    static channel_t* v1_4_Addr;
    static channel_t* v2_4_Addr;
    static channel_t* v3_4_Addr;
    static channel_t* v4_4_Addr;
    static channel_t* v5_4_Addr;
    static channel_t* v1_5_Addr;
    static channel_t* v2_5_Addr;
    static channel_t* v3_5_Addr;
    static channel_t* v4_5_Addr;
    static channel_t* v5_5_Addr;
    static channel_t* v1_6_Addr;
    static channel_t* v2_6_Addr;
    static channel_t* v3_6_Addr;
    static channel_t* v4_6_Addr;
    static channel_t* v5_6_Addr;
    static channel_t* v1_7_Addr;
    static channel_t* v2_7_Addr;
    static channel_t* v3_7_Addr;
    static channel_t* v4_7_Addr;
    static channel_t* v5_7_Addr;
    static channel_t* v1_8_Addr;
    static channel_t* v2_8_Addr;
    static channel_t* v3_8_Addr;
    static channel_t* v4_8_Addr;
    static channel_t* v5_8_Addr;
    if (firsttime == 1) {
        firsttime = 0;
        v1_1_Addr = channels_find_by_name("v1_1_pss");
        v2_1_Addr = channels_find_by_name("v2_1_pss");
        v3_1_Addr = channels_find_by_name("v3_1_pss");
        v4_1_Addr = channels_find_by_name("v4_1_pss");
        v5_1_Addr = channels_find_by_name("v5_1_pss");
        v1_2_Addr = channels_find_by_name("v1_2_pss");
        v2_2_Addr = channels_find_by_name("v2_2_pss");
        v3_2_Addr = channels_find_by_name("v3_2_pss");
        v4_2_Addr = channels_find_by_name("v4_2_pss");
        v5_2_Addr = channels_find_by_name("v5_2_pss");
        v1_3_Addr = channels_find_by_name("v1_3_pss");
        v2_3_Addr = channels_find_by_name("v2_3_pss");
        v3_3_Addr = channels_find_by_name("v3_3_pss");
        v4_3_Addr = channels_find_by_name("v4_3_pss");
        v5_3_Addr = channels_find_by_name("v5_3_pss");
        v1_4_Addr = channels_find_by_name("v1_4_pss");
        v2_4_Addr = channels_find_by_name("v2_4_pss");
        v3_4_Addr = channels_find_by_name("v3_4_pss");
        v4_4_Addr = channels_find_by_name("v4_4_pss");
        v5_4_Addr = channels_find_by_name("v5_4_pss");
        v1_5_Addr = channels_find_by_name("v1_5_pss");
        v2_5_Addr = channels_find_by_name("v2_5_pss");
        v3_5_Addr = channels_find_by_name("v3_5_pss");
        v4_5_Addr = channels_find_by_name("v4_5_pss");
        v5_5_Addr = channels_find_by_name("v5_5_pss");
        v1_6_Addr = channels_find_by_name("v1_6_pss");
        v2_6_Addr = channels_find_by_name("v2_6_pss");
        v3_6_Addr = channels_find_by_name("v3_6_pss");
        v4_6_Addr = channels_find_by_name("v4_6_pss");
        v5_6_Addr = channels_find_by_name("v5_6_pss");
        v1_7_Addr = channels_find_by_name("v1_7_pss");
        v2_7_Addr = channels_find_by_name("v2_7_pss");
        v3_7_Addr = channels_find_by_name("v3_7_pss");
        v4_7_Addr = channels_find_by_name("v4_7_pss");
        v5_7_Addr = channels_find_by_name("v5_7_pss");
        v1_8_Addr = channels_find_by_name("v1_8_pss");
        v2_8_Addr = channels_find_by_name("v2_8_pss");
        v3_8_Addr = channels_find_by_name("v3_8_pss");
        v4_8_Addr = channels_find_by_name("v4_8_pss");
        v5_8_Addr = channels_find_by_name("v5_8_pss");
        for (int i = 0; i < NUM_LABJACKS; i++) {
            for (int j = 0; j < NUM_PSS_V; j++) {
                init_labjack_10hz_filter(&vPSSfilt[i][j]);
            }
            init_labjack_10hz_filter(&TPSSfilt[i]);
        }
    }
    if (state[5].connected) {
	blast_info("Sending data from labjack6 to channels");
        SET_FLOAT(v1_1_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 26), &vPSSfilt[0][0]));
	blast_info("%f", labjack_get_value(5, 26));
        SET_FLOAT(v2_1_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 24), &vPSSfilt[1][0]));
        SET_FLOAT(v3_1_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 22), &vPSSfilt[2][0]));
        SET_FLOAT(v4_1_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 20), &vPSSfilt[3][0]));
        SET_FLOAT(v5_1_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 18), &TPSSfilt[0]));
        SET_FLOAT(v1_2_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 17), &vPSSfilt[0][1]));
        SET_FLOAT(v2_2_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 15), &vPSSfilt[1][1]));
        SET_FLOAT(v3_2_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 13), &vPSSfilt[2][1]));
        SET_FLOAT(v4_2_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 11), &vPSSfilt[3][1]));
        SET_FLOAT(v5_2_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 9), &TPSSfilt[1]));
        SET_FLOAT(v1_3_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 7), &vPSSfilt[0][2]));
        SET_FLOAT(v2_3_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 5), &vPSSfilt[1][2]));
        SET_FLOAT(v3_3_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 27), &vPSSfilt[2][2]));
        SET_FLOAT(v4_3_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 25), &vPSSfilt[3][2]));
        SET_FLOAT(v5_3_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 23), &TPSSfilt[2]));
        SET_FLOAT(v1_4_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 21), &vPSSfilt[0][3]));
        SET_FLOAT(v2_4_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 19), &vPSSfilt[1][3]));
        SET_FLOAT(v3_4_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 16), &vPSSfilt[2][3]));
        SET_FLOAT(v4_4_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 14), &vPSSfilt[3][3]));
        SET_FLOAT(v5_4_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 12), &TPSSfilt[3]));
        SET_FLOAT(v1_5_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 50), &vPSSfilt[0][4]));
        SET_FLOAT(v2_5_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 48), &vPSSfilt[1][4]));
        SET_FLOAT(v3_5_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 46), &vPSSfilt[2][4]));
        SET_FLOAT(v4_5_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 44), &vPSSfilt[3][4]));
        SET_FLOAT(v5_5_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 42), &TPSSfilt[4]));
        SET_FLOAT(v1_6_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 41), &vPSSfilt[0][5]));
        SET_FLOAT(v2_6_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 39), &vPSSfilt[1][5]));
        SET_FLOAT(v3_6_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 37), &vPSSfilt[2][5]));
        SET_FLOAT(v4_6_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 35), &vPSSfilt[3][5]));
        SET_FLOAT(v5_6_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 33), &TPSSfilt[5]));
        SET_FLOAT(v1_7_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 31), &vPSSfilt[0][6]));
        SET_FLOAT(v2_7_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 29), &vPSSfilt[1][6]));
        SET_FLOAT(v3_7_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 51), &vPSSfilt[2][6]));
        SET_FLOAT(v4_7_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 49), &vPSSfilt[3][6]));
        SET_FLOAT(v5_7_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 47), &TPSSfilt[6]));
        SET_FLOAT(v1_8_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 45), &vPSSfilt[0][7]));
        SET_FLOAT(v2_8_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 43), &vPSSfilt[1][7]));
        SET_FLOAT(v3_8_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 40), &vPSSfilt[2][7]));
        SET_FLOAT(v4_8_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 38), &vPSSfilt[3][7]));
        SET_FLOAT(v5_8_Addr, filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_PSS, 36), &TPSSfilt[7]));
    }
}
