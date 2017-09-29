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
#include "labjack_functions.h"
#include "blast.h"
#include "multiplexed_labjack.h"


// Add clinometer channels and also add the derived channels when we get to testing.
// this function is called to update the thermometry on the outside of the IF and OF
// same build as the read thermometers code
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
    SET_SCALED_VALUE(thermistor_15_Addr, labjack_get_value(LABJACK_OF_1, 0));
    SET_SCALED_VALUE(thermistor_16_Addr, labjack_get_value(LABJACK_OF_1, 1));
    SET_SCALED_VALUE(thermistor_17_Addr, labjack_get_value(LABJACK_OF_1, 2));
    SET_SCALED_VALUE(thermistor_18_Addr, labjack_get_value(LABJACK_OF_1, 3));
    SET_SCALED_VALUE(thermistor_19_Addr, labjack_get_value(LABJACK_OF_1, 4));
    SET_SCALED_VALUE(thermistor_20_Addr, labjack_get_value(LABJACK_OF_1, 5));
    SET_SCALED_VALUE(thermistor_21_Addr, labjack_get_value(LABJACK_OF_1, 6));
    SET_SCALED_VALUE(thermistor_22_Addr, labjack_get_value(LABJACK_OF_1, 7));
    SET_SCALED_VALUE(thermistor_23_Addr, labjack_get_value(LABJACK_OF_1, 8));
    SET_SCALED_VALUE(thermistor_24_Addr, labjack_get_value(LABJACK_OF_1, 9));
    SET_SCALED_VALUE(thermistor_25_Addr, labjack_get_value(LABJACK_OF_1, 10));
    SET_SCALED_VALUE(thermistor_26_Addr, labjack_get_value(LABJACK_OF_1, 11));
    SET_SCALED_VALUE(thermistor_27_Addr, labjack_get_value(LABJACK_OF_1, 12));
    SET_SCALED_VALUE(thermistor_28_Addr, labjack_get_value(LABJACK_OF_1, 13));
}
// updates clinometers instead of thermometers
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
    SET_SCALED_VALUE(clin_1_x_Addr, labjack_get_value(LABJACK_OF_3, 10));
    SET_SCALED_VALUE(clin_1_y_Addr, labjack_get_value(LABJACK_OF_3, 11));
    SET_SCALED_VALUE(clin_2_x_Addr, labjack_get_value(LABJACK_OF_3, 12));
    SET_SCALED_VALUE(clin_2_y_Addr, labjack_get_value(LABJACK_OF_3, 13));
}
// same deal for current sensors
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
    SET_SCALED_VALUE(current_loop_1_Addr, labjack_get_value(LABJACK_OF_3, 0));
    SET_SCALED_VALUE(current_loop_2_Addr, labjack_get_value(LABJACK_OF_3, 1));
    SET_SCALED_VALUE(current_loop_3_Addr, labjack_get_value(LABJACK_OF_3, 2));
    SET_SCALED_VALUE(current_loop_4_Addr, labjack_get_value(LABJACK_OF_3, 3));
    SET_SCALED_VALUE(current_loop_5_Addr, labjack_get_value(LABJACK_OF_3, 4));
    SET_SCALED_VALUE(current_loop_6_Addr, labjack_get_value(LABJACK_OF_3, 5));
    SET_SCALED_VALUE(current_loop_7_Addr, labjack_get_value(LABJACK_OF_3, 6));
    SET_SCALED_VALUE(current_loop_8_Addr, labjack_get_value(LABJACK_OF_3, 7));
    SET_SCALED_VALUE(current_loop_9_Addr, labjack_get_value(LABJACK_OF_3, 8));
    SET_SCALED_VALUE(current_loop_10_Addr, labjack_get_value(LABJACK_OF_3, 9));
}

void outer_frame_multiplexed(void) {
    static channel_t* thermistor_29_Addr;
    static channel_t* thermistor_30_Addr;
    static channel_t* thermistor_31_Addr;
    static channel_t* thermistor_32_Addr;
    static channel_t* thermistor_33_Addr;
    static channel_t* thermistor_34_Addr;
    static channel_t* thermistor_35_Addr;
    static channel_t* thermistor_36_Addr;
    static channel_t* thermistor_37_Addr;
    static channel_t* thermistor_38_Addr;
    static channel_t* thermistor_39_Addr;
    static channel_t* thermistor_40_Addr;
    static channel_t* thermistor_41_Addr;
    static channel_t* thermistor_42_Addr;
    static channel_t* thermistor_43_Addr;
    static channel_t* thermistor_44_Addr;
    static channel_t* thermistor_45_Addr;
    static channel_t* thermistor_46_Addr;
    static channel_t* thermistor_47_Addr;
    static channel_t* thermistor_48_Addr;
    static channel_t* thermistor_49_Addr;
    static channel_t* thermistor_50_Addr;
    static channel_t* thermistor_51_Addr;
    static channel_t* thermistor_52_Addr;
    static channel_t* thermistor_53_Addr;
    static channel_t* thermistor_54_Addr;
    static channel_t* thermistor_55_Addr;
    static channel_t* thermistor_56_Addr;
    static channel_t* thermistor_57_Addr;
    static channel_t* thermistor_58_Addr;
    static channel_t* thermistor_59_Addr;
    static channel_t* thermistor_60_Addr;
    static channel_t* thermistor_61_Addr;
    static channel_t* thermistor_62_Addr;
    static channel_t* thermistor_63_Addr;
    static channel_t* thermistor_64_Addr;
    static channel_t* thermistor_65_Addr;
    static channel_t* thermistor_66_Addr;
    static int first_mult = 1;
    if (first_mult == 1) {
        first_mult = 0;
        thermistor_29_Addr = channels_find_by_name("thermistor_29");
        thermistor_30_Addr = channels_find_by_name("thermistor_30");
        thermistor_31_Addr = channels_find_by_name("thermistor_31");
        thermistor_32_Addr = channels_find_by_name("thermistor_32");
        thermistor_33_Addr = channels_find_by_name("thermistor_33");
        thermistor_34_Addr = channels_find_by_name("thermistor_34");
        thermistor_35_Addr = channels_find_by_name("thermistor_35");
        thermistor_36_Addr = channels_find_by_name("thermistor_36");
        thermistor_37_Addr = channels_find_by_name("thermistor_37");
        thermistor_38_Addr = channels_find_by_name("thermistor_38");
        thermistor_39_Addr = channels_find_by_name("thermistor_39");
        thermistor_40_Addr = channels_find_by_name("thermistor_40");
        thermistor_41_Addr = channels_find_by_name("thermistor_41");
        thermistor_42_Addr = channels_find_by_name("thermistor_42");
        thermistor_43_Addr = channels_find_by_name("thermistor_43");
        thermistor_44_Addr = channels_find_by_name("thermistor_44");
        thermistor_45_Addr = channels_find_by_name("thermistor_45");
        thermistor_46_Addr = channels_find_by_name("thermistor_46");
        thermistor_47_Addr = channels_find_by_name("thermistor_47");
        thermistor_48_Addr = channels_find_by_name("thermistor_48");
        thermistor_49_Addr = channels_find_by_name("thermistor_49");
        thermistor_50_Addr = channels_find_by_name("thermistor_50");
        thermistor_51_Addr = channels_find_by_name("thermistor_51");
        thermistor_52_Addr = channels_find_by_name("thermistor_52");
        thermistor_53_Addr = channels_find_by_name("thermistor_53");
        thermistor_54_Addr = channels_find_by_name("thermistor_54");
        thermistor_55_Addr = channels_find_by_name("thermistor_55");
        thermistor_56_Addr = channels_find_by_name("thermistor_56");
        thermistor_57_Addr = channels_find_by_name("thermistor_57");
        thermistor_58_Addr = channels_find_by_name("thermistor_58");
        thermistor_59_Addr = channels_find_by_name("thermistor_58");
        thermistor_60_Addr = channels_find_by_name("thermistor_60");
        thermistor_61_Addr = channels_find_by_name("thermistor_61");
        thermistor_62_Addr = channels_find_by_name("thermistor_62");
        thermistor_63_Addr = channels_find_by_name("thermistor_63");
        thermistor_64_Addr = channels_find_by_name("thermistor_64");
        thermistor_65_Addr = channels_find_by_name("thermistor_65");
        thermistor_66_Addr = channels_find_by_name("thermistor_66");
    }
    blast_info("the value of 29 is: %f", labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_29_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_30_Addr, labjack_get_value(6, 5));
    SET_SCALED_VALUE(thermistor_31_Addr, labjack_get_value(6, 6));
    SET_SCALED_VALUE(thermistor_32_Addr, labjack_get_value(6, 7));
    SET_SCALED_VALUE(thermistor_33_Addr, labjack_get_value(6, 8));
    SET_SCALED_VALUE(thermistor_34_Addr, labjack_get_value(6, 9));
    SET_SCALED_VALUE(thermistor_35_Addr, labjack_get_value(6, 10));
    SET_SCALED_VALUE(thermistor_36_Addr, labjack_get_value(6, 11));
    SET_SCALED_VALUE(thermistor_37_Addr, labjack_get_value(6, 12));
    SET_SCALED_VALUE(thermistor_38_Addr, labjack_get_value(6, 13));
    SET_SCALED_VALUE(thermistor_39_Addr, labjack_get_value(6, 14));
    SET_SCALED_VALUE(thermistor_40_Addr, labjack_get_value(6, 15));
    SET_SCALED_VALUE(thermistor_41_Addr, labjack_get_value(6, 16));
    SET_SCALED_VALUE(thermistor_42_Addr, labjack_get_value(6, 17));
    SET_SCALED_VALUE(thermistor_43_Addr, labjack_get_value(6, 18));
    SET_SCALED_VALUE(thermistor_44_Addr, labjack_get_value(6, 19));
    SET_SCALED_VALUE(thermistor_45_Addr, labjack_get_value(6, 20));
    SET_SCALED_VALUE(thermistor_46_Addr, labjack_get_value(6, 21));
    SET_SCALED_VALUE(thermistor_47_Addr, labjack_get_value(6, 22));
    SET_SCALED_VALUE(thermistor_48_Addr, labjack_get_value(6, 23));
    SET_SCALED_VALUE(thermistor_49_Addr, labjack_get_value(6, 24));
    SET_SCALED_VALUE(thermistor_50_Addr, labjack_get_value(6, 25));
    SET_SCALED_VALUE(thermistor_51_Addr, labjack_get_value(6, 26));
    SET_SCALED_VALUE(thermistor_52_Addr, labjack_get_value(6, 27));
    SET_SCALED_VALUE(thermistor_53_Addr, labjack_get_value(6, 28));
    SET_SCALED_VALUE(thermistor_54_Addr, labjack_get_value(6, 29));
    SET_SCALED_VALUE(thermistor_55_Addr, labjack_get_value(6, 30));
    SET_SCALED_VALUE(thermistor_56_Addr, labjack_get_value(6, 31));
    SET_SCALED_VALUE(thermistor_57_Addr, labjack_get_value(6, 32));
    SET_SCALED_VALUE(thermistor_58_Addr, labjack_get_value(6, 33));
    SET_SCALED_VALUE(thermistor_59_Addr, labjack_get_value(6, 34));
    SET_SCALED_VALUE(thermistor_60_Addr, labjack_get_value(6, 35));
    SET_SCALED_VALUE(thermistor_61_Addr, labjack_get_value(6, 36));
    SET_SCALED_VALUE(thermistor_62_Addr, labjack_get_value(6, 37));
    SET_SCALED_VALUE(thermistor_63_Addr, labjack_get_value(6, 38));
    SET_SCALED_VALUE(thermistor_64_Addr, labjack_get_value(6, 39));
    SET_SCALED_VALUE(thermistor_65_Addr, labjack_get_value(6, 40));
    SET_SCALED_VALUE(thermistor_66_Addr, labjack_get_value(6, 41));
}

void outer_frame(int setting) {
    if (setting == 1) {
        update_current_sensors();
        update_thermistors();
        // update_clinometers();
    }
}




