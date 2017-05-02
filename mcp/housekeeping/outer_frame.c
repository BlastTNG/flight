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
 
 created by Ian Lowe 1-23-17
 **************************************************************************/


/*************************************************************************
 
 outer_frame.c -- mcp code to control outer frame readout (labjack)
 
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


// Add clinometer channels and also add the derived channels when we get to testing.

void update_thermistors(void) {
    static int first_time_therm = 1;
    static channel_t* thermistor_1_Addr;
    static channel_t* thermistor_2_Addr;
    static channel_t* thermistor_3_Addr;
    static channel_t* thermistor_4_Addr;
    static channel_t* thermistor_5_Addr;
    static channel_t* thermistor_6_Addr;
    static channel_t* thermistor_7_Addr;
    static channel_t* thermistor_8_Addr;
    static channel_t* thermistor_9_Addr;
    static channel_t* thermistor_10_Addr;
    static channel_t* thermistor_11_Addr;
    static channel_t* thermistor_12_Addr;
    static channel_t* thermistor_13_Addr;
    static channel_t* thermistor_14_Addr;
    static channel_t* thermistor_15_Addr;
    static channel_t* thermistor_16_Addr;
    static channel_t* thermistor_17_Addr;
    static channel_t* thermistor_18_Addr;
    static channel_t* thermistor_19_Addr;
    static channel_t* thermistor_20_Addr;
    static channel_t* thermistor_21_Addr;
    static channel_t* thermistor_22_Addr;
    static channel_t* thermistor_23_Addr;
    static channel_t* thermistor_24_Addr;
    static channel_t* thermistor_25_Addr;
    static channel_t* thermistor_26_Addr;
    static channel_t* thermistor_27_Addr;
    static channel_t* thermistor_28_Addr;
    if (first_time_therm == 1) {
        thermistor_1_Addr = channels_find_by_name("thermistor_1");
        thermistor_2_Addr = channels_find_by_name("thermistor_2");
        thermistor_3_Addr = channels_find_by_name("thermistor_3");
        thermistor_4_Addr = channels_find_by_name("thermistor_4");
        thermistor_5_Addr = channels_find_by_name("thermistor_5");
        thermistor_6_Addr = channels_find_by_name("thermistor_6");
        thermistor_7_Addr = channels_find_by_name("thermistor_7");
        thermistor_8_Addr = channels_find_by_name("thermistor_8");
        thermistor_9_Addr = channels_find_by_name("thermistor_9");
        thermistor_10_Addr = channels_find_by_name("thermistor_10");
        thermistor_11_Addr = channels_find_by_name("thermistor_11");
        thermistor_12_Addr = channels_find_by_name("thermistor_12");
        thermistor_13_Addr = channels_find_by_name("thermistor_13");
        thermistor_14_Addr = channels_find_by_name("thermistor_14");
        thermistor_15_Addr = channels_find_by_name("thermistor_15");
        thermistor_16_Addr = channels_find_by_name("thermistor_16");
        thermistor_17_Addr = channels_find_by_name("thermistor_17");
        thermistor_18_Addr = channels_find_by_name("thermistor_18");
        thermistor_19_Addr = channels_find_by_name("thermistor_19");
        thermistor_20_Addr = channels_find_by_name("thermistor_20");
        thermistor_21_Addr = channels_find_by_name("thermistor_21");
        thermistor_22_Addr = channels_find_by_name("thermistor_22");
        thermistor_23_Addr = channels_find_by_name("thermistor_23");
        thermistor_24_Addr = channels_find_by_name("thermistor_24");
        thermistor_25_Addr = channels_find_by_name("thermistor_25");
        thermistor_26_Addr = channels_find_by_name("thermistor_26");
        thermistor_27_Addr = channels_find_by_name("thermistor_27");
        thermistor_28_Addr = channels_find_by_name("thermistor_28");
        first_time_therm = 0;
    }
    SET_SCALED_VALUE(thermistor_1_Addr, labjack_get_value(LABJACK_OF_2, 0));
    SET_SCALED_VALUE(thermistor_2_Addr, labjack_get_value(LABJACK_OF_2, 1));
    SET_SCALED_VALUE(thermistor_3_Addr, labjack_get_value(LABJACK_OF_2, 2));
    SET_SCALED_VALUE(thermistor_4_Addr, labjack_get_value(LABJACK_OF_2, 3));
    SET_SCALED_VALUE(thermistor_5_Addr, labjack_get_value(LABJACK_OF_2, 4));
    SET_SCALED_VALUE(thermistor_6_Addr, labjack_get_value(LABJACK_OF_2, 5));
    SET_SCALED_VALUE(thermistor_7_Addr, labjack_get_value(LABJACK_OF_2, 6));
    SET_SCALED_VALUE(thermistor_8_Addr, labjack_get_value(LABJACK_OF_2, 7));
    SET_SCALED_VALUE(thermistor_9_Addr, labjack_get_value(LABJACK_OF_2, 8));
    SET_SCALED_VALUE(thermistor_10_Addr, labjack_get_value(LABJACK_OF_2, 9));
    SET_SCALED_VALUE(thermistor_11_Addr, labjack_get_value(LABJACK_OF_2, 10));
    SET_SCALED_VALUE(thermistor_12_Addr, labjack_get_value(LABJACK_OF_2, 11));
    SET_SCALED_VALUE(thermistor_13_Addr, labjack_get_value(LABJACK_OF_2, 12));
    SET_SCALED_VALUE(thermistor_14_Addr, labjack_get_value(LABJACK_OF_2, 13));
    SET_SCALED_VALUE(thermistor_15_Addr, labjack_get_value(LABJACK_OF_3, 0));
    SET_SCALED_VALUE(thermistor_16_Addr, labjack_get_value(LABJACK_OF_3, 1));
    SET_SCALED_VALUE(thermistor_17_Addr, labjack_get_value(LABJACK_OF_3, 2));
    SET_SCALED_VALUE(thermistor_18_Addr, labjack_get_value(LABJACK_OF_3, 3));
    SET_SCALED_VALUE(thermistor_19_Addr, labjack_get_value(LABJACK_OF_3, 4));
    SET_SCALED_VALUE(thermistor_20_Addr, labjack_get_value(LABJACK_OF_3, 5));
    SET_SCALED_VALUE(thermistor_21_Addr, labjack_get_value(LABJACK_OF_3, 6));
    SET_SCALED_VALUE(thermistor_22_Addr, labjack_get_value(LABJACK_OF_3, 7));
    SET_SCALED_VALUE(thermistor_23_Addr, labjack_get_value(LABJACK_OF_3, 8));
    SET_SCALED_VALUE(thermistor_24_Addr, labjack_get_value(LABJACK_OF_3, 9));
    SET_SCALED_VALUE(thermistor_25_Addr, labjack_get_value(LABJACK_OF_3, 10));
    SET_SCALED_VALUE(thermistor_26_Addr, labjack_get_value(LABJACK_OF_3, 11));
    SET_SCALED_VALUE(thermistor_27_Addr, labjack_get_value(LABJACK_OF_3, 12));
    SET_SCALED_VALUE(thermistor_28_Addr, labjack_get_value(LABJACK_OF_3, 13));
}

void update_clinometer(void) {
    static int first_time_clin = 1;
    static channel_t* clin_1_x_Addr;
    static channel_t* clin_1_y_Addr;
    static channel_t* clin_2_x_Addr;
    static channel_t* clin_2_y_Addr;
    if (first_time_clin == 1) {
        first_time_clin = 0;
        clin_1_x_Addr = channels_find_by_name("clin_1_x");
        clin_1_y_Addr = channels_find_by_name("clin_1_y");
        clin_2_x_Addr = channels_find_by_name("clin_2_x");
        clin_2_y_Addr = channels_find_by_name("clin_2_y");
    }
    SET_SCALED_VALUE(clin_1_x_Addr, labjack_get_value(LABJACK_OF_1, 10));
    SET_SCALED_VALUE(clin_1_y_Addr, labjack_get_value(LABJACK_OF_1, 11));
    SET_SCALED_VALUE(clin_2_x_Addr, labjack_get_value(LABJACK_OF_1, 12));
    SET_SCALED_VALUE(clin_2_y_Addr, labjack_get_value(LABJACK_OF_1, 13));
}

void update_current_sensors(void) {
    static int first_time_current = 1;
    static channel_t* current_loop_1_Addr;
    static channel_t* current_loop_2_Addr;
    static channel_t* current_loop_3_Addr;
    static channel_t* current_loop_4_Addr;
    static channel_t* current_loop_5_Addr;
    static channel_t* current_loop_6_Addr;
    static channel_t* current_loop_7_Addr;
    static channel_t* current_loop_8_Addr;
    static channel_t* current_loop_9_Addr;
    static channel_t* current_loop_10_Addr;
    if (first_time_current == 1) {
        first_time_current = 0;
        current_loop_1_Addr = channels_find_by_name("current_loop_1");
        current_loop_2_Addr = channels_find_by_name("current_loop_2");
        current_loop_3_Addr = channels_find_by_name("current_loop_3");
        current_loop_4_Addr = channels_find_by_name("current_loop_4");
        current_loop_5_Addr = channels_find_by_name("current_loop_5");
        current_loop_6_Addr = channels_find_by_name("current_loop_6");
        current_loop_7_Addr = channels_find_by_name("current_loop_7");
        current_loop_8_Addr = channels_find_by_name("current_loop_8");
        current_loop_9_Addr = channels_find_by_name("current_loop_9");
        current_loop_10_Addr = channels_find_by_name("current_loop_10");
    }
    SET_SCALED_VALUE(current_loop_1_Addr, labjack_get_value(LABJACK_OF_1, 0));
    SET_SCALED_VALUE(current_loop_2_Addr, labjack_get_value(LABJACK_OF_1, 1));
    SET_SCALED_VALUE(current_loop_3_Addr, labjack_get_value(LABJACK_OF_1, 2));
    SET_SCALED_VALUE(current_loop_4_Addr, labjack_get_value(LABJACK_OF_1, 3));
    SET_SCALED_VALUE(current_loop_5_Addr, labjack_get_value(LABJACK_OF_1, 4));
    SET_SCALED_VALUE(current_loop_6_Addr, labjack_get_value(LABJACK_OF_1, 5));
    SET_SCALED_VALUE(current_loop_7_Addr, labjack_get_value(LABJACK_OF_1, 6));
    SET_SCALED_VALUE(current_loop_8_Addr, labjack_get_value(LABJACK_OF_1, 7));
    SET_SCALED_VALUE(current_loop_9_Addr, labjack_get_value(LABJACK_OF_1, 8));
    SET_SCALED_VALUE(current_loop_10_Addr, labjack_get_value(LABJACK_OF_1, 9));
}



