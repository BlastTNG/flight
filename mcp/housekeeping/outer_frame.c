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
#include "calibrate.h"
#include "therm_roach.h"

extern int16_t InCharge;
extern labjack_state_t state[NUM_LABJACKS];

static labjack_10hz_filter_t OFCurFilt[17];

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
    static channel_t* thermistor_67_Addr;
    static channel_t* thermistor_68_Addr;
    static channel_t* thermistor_69_Addr;
    static channel_t* thermistor_70_Addr;
    static channel_t* thermistor_71_Addr;
    static channel_t* thermistor_72_Addr;
    static channel_t* thermistor_73_Addr;
    static channel_t* thermistor_74_Addr;
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
        thermistor_59_Addr = channels_find_by_name("thermistor_59");
        thermistor_60_Addr = channels_find_by_name("thermistor_60");
        thermistor_61_Addr = channels_find_by_name("thermistor_61");
        thermistor_62_Addr = channels_find_by_name("thermistor_62");
        thermistor_63_Addr = channels_find_by_name("thermistor_63");
        thermistor_64_Addr = channels_find_by_name("thermistor_64");
        thermistor_65_Addr = channels_find_by_name("thermistor_65");
        thermistor_66_Addr = channels_find_by_name("thermistor_66");
        thermistor_67_Addr = channels_find_by_name("thermistor_67");
        thermistor_68_Addr = channels_find_by_name("thermistor_68");
        thermistor_69_Addr = channels_find_by_name("thermistor_69");
        thermistor_70_Addr = channels_find_by_name("thermistor_70");
        thermistor_71_Addr = channels_find_by_name("thermistor_71");
        thermistor_72_Addr = channels_find_by_name("thermistor_72");
        thermistor_73_Addr = channels_find_by_name("thermistor_73");
        thermistor_74_Addr = channels_find_by_name("thermistor_74");
    }
    SET_SCALED_VALUE(thermistor_1_Addr, labjack_get_value(LABJACK_OF_1, 0));
    SET_SCALED_VALUE(thermistor_2_Addr, labjack_get_value(LABJACK_OF_1, 1));
    SET_SCALED_VALUE(thermistor_3_Addr, labjack_get_value(LABJACK_OF_1, 2));
    SET_SCALED_VALUE(thermistor_4_Addr, labjack_get_value(LABJACK_OF_1, 3));
    SET_SCALED_VALUE(thermistor_5_Addr, labjack_get_value(LABJACK_OF_1, 4));
    SET_SCALED_VALUE(thermistor_6_Addr, labjack_get_value(LABJACK_OF_1, 5));
    SET_SCALED_VALUE(thermistor_7_Addr, labjack_get_value(LABJACK_OF_1, 6));
    SET_SCALED_VALUE(thermistor_8_Addr, labjack_get_value(LABJACK_OF_1, 7));
    SET_SCALED_VALUE(thermistor_9_Addr, labjack_get_value(LABJACK_OF_1, 8));
    SET_SCALED_VALUE(thermistor_10_Addr, labjack_get_value(LABJACK_OF_1, 9));
    SET_SCALED_VALUE(thermistor_11_Addr, labjack_get_value(LABJACK_OF_1, 10));
    SET_SCALED_VALUE(thermistor_12_Addr, labjack_get_value(LABJACK_OF_1, 11));
    SET_SCALED_VALUE(thermistor_13_Addr, labjack_get_value(LABJACK_OF_1, 12));
    SET_SCALED_VALUE(thermistor_14_Addr, labjack_get_value(LABJACK_OF_1, 13));
    SET_SCALED_VALUE(thermistor_15_Addr, labjack_get_value(LABJACK_MULT_OF, 4));
    SET_SCALED_VALUE(thermistor_16_Addr, labjack_get_value(LABJACK_MULT_OF, 5));
    SET_SCALED_VALUE(thermistor_17_Addr, labjack_get_value(LABJACK_MULT_OF, 6));
    SET_SCALED_VALUE(thermistor_18_Addr, labjack_get_value(LABJACK_MULT_OF, 7));
    SET_SCALED_VALUE(thermistor_19_Addr, labjack_get_value(LABJACK_MULT_OF, 8));
    SET_SCALED_VALUE(thermistor_20_Addr, labjack_get_value(LABJACK_MULT_OF, 9));
    SET_SCALED_VALUE(thermistor_21_Addr, labjack_get_value(LABJACK_MULT_OF, 10));
    SET_SCALED_VALUE(thermistor_22_Addr, labjack_get_value(LABJACK_MULT_OF, 11));
    SET_SCALED_VALUE(thermistor_23_Addr, labjack_get_value(LABJACK_MULT_OF, 12));
    SET_SCALED_VALUE(thermistor_24_Addr, labjack_get_value(LABJACK_MULT_OF, 13));
    SET_SCALED_VALUE(thermistor_25_Addr, labjack_get_value(LABJACK_MULT_OF, 14));
    SET_SCALED_VALUE(thermistor_26_Addr, labjack_get_value(LABJACK_MULT_OF, 15));
    SET_SCALED_VALUE(thermistor_27_Addr, labjack_get_value(LABJACK_MULT_OF, 16));
    SET_SCALED_VALUE(thermistor_28_Addr, labjack_get_value(LABJACK_MULT_OF, 17));
    SET_SCALED_VALUE(thermistor_29_Addr, labjack_get_value(LABJACK_MULT_OF, 18));
    SET_SCALED_VALUE(thermistor_30_Addr, labjack_get_value(LABJACK_MULT_OF, 19));
    SET_SCALED_VALUE(thermistor_31_Addr, labjack_get_value(LABJACK_MULT_OF, 20));
    SET_SCALED_VALUE(thermistor_32_Addr, labjack_get_value(LABJACK_OF_2, 0));
    SET_SCALED_VALUE(thermistor_33_Addr, labjack_get_value(LABJACK_OF_2, 1));
    SET_SCALED_VALUE(thermistor_34_Addr, labjack_get_value(LABJACK_OF_2, 2));
    SET_SCALED_VALUE(thermistor_35_Addr, labjack_get_value(LABJACK_OF_2, 3));
    SET_SCALED_VALUE(thermistor_36_Addr, labjack_get_value(LABJACK_OF_2, 4));
    SET_SCALED_VALUE(thermistor_37_Addr, labjack_get_value(LABJACK_OF_2, 5));
    SET_SCALED_VALUE(thermistor_38_Addr, labjack_get_value(LABJACK_OF_2, 6));
    SET_SCALED_VALUE(thermistor_39_Addr, labjack_get_value(LABJACK_OF_2, 7));
    SET_SCALED_VALUE(thermistor_40_Addr, labjack_get_value(LABJACK_OF_2, 8));
    SET_SCALED_VALUE(thermistor_41_Addr, labjack_get_value(LABJACK_OF_2, 9));
    SET_SCALED_VALUE(thermistor_42_Addr, labjack_get_value(LABJACK_OF_2, 10));
    SET_SCALED_VALUE(thermistor_43_Addr, labjack_get_value(LABJACK_OF_2, 11));
    SET_SCALED_VALUE(thermistor_44_Addr, labjack_get_value(LABJACK_OF_2, 12));
    SET_SCALED_VALUE(thermistor_45_Addr, labjack_get_value(LABJACK_OF_2, 13));
    SET_SCALED_VALUE(thermistor_46_Addr, labjack_get_value(LABJACK_MULT_OF, 21));
    SET_SCALED_VALUE(thermistor_47_Addr, labjack_get_value(LABJACK_MULT_OF, 22));
    SET_SCALED_VALUE(thermistor_48_Addr, labjack_get_value(LABJACK_MULT_OF, 23));
    SET_SCALED_VALUE(thermistor_49_Addr, labjack_get_value(LABJACK_MULT_OF, 24));
    SET_SCALED_VALUE(thermistor_50_Addr, labjack_get_value(LABJACK_MULT_OF, 25));
    SET_SCALED_VALUE(thermistor_51_Addr, labjack_get_value(LABJACK_MULT_OF, 26));
    SET_SCALED_VALUE(thermistor_52_Addr, labjack_get_value(LABJACK_MULT_OF, 27));
    SET_SCALED_VALUE(thermistor_53_Addr, labjack_get_value(LABJACK_MULT_OF, 28));
    SET_SCALED_VALUE(thermistor_54_Addr, labjack_get_value(LABJACK_MULT_OF, 29));
    SET_SCALED_VALUE(thermistor_55_Addr, labjack_get_value(LABJACK_MULT_OF, 30));
    SET_SCALED_VALUE(thermistor_56_Addr, labjack_get_value(LABJACK_MULT_OF, 31));
    SET_SCALED_VALUE(thermistor_57_Addr, labjack_get_value(LABJACK_MULT_OF, 32));
    SET_SCALED_VALUE(thermistor_58_Addr, labjack_get_value(LABJACK_MULT_OF, 33));
    SET_SCALED_VALUE(thermistor_59_Addr, labjack_get_value(LABJACK_MULT_OF, 34));
    SET_SCALED_VALUE(thermistor_60_Addr, labjack_get_value(LABJACK_MULT_OF, 35));
    SET_SCALED_VALUE(thermistor_61_Addr, labjack_get_value(LABJACK_MULT_OF, 36));
    SET_SCALED_VALUE(thermistor_62_Addr, labjack_get_value(LABJACK_MULT_OF, 37));
    SET_FLOAT(thermistor_63_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 38)));
    SET_FLOAT(thermistor_64_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 39)));
    SET_FLOAT(thermistor_65_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 40)));
    SET_FLOAT(thermistor_66_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 41)));
    SET_FLOAT(thermistor_67_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 42)));
    SET_FLOAT(thermistor_68_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 43)));
    SET_FLOAT(thermistor_69_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 44)));
    SET_FLOAT(thermistor_70_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 45)));
    SET_FLOAT(thermistor_71_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 46)));
    SET_FLOAT(thermistor_72_Addr, temp(labjack_get_value(LABJACK_MULT_OF, 47)));
    /*
    blast_info("ROACH THERMISTOR 63 LJ voltage = %f", labjack_get_value(LABJACK_MULT_OF, 38));
    blast_info("ROACH THERMISTOR 63 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 38)));
    blast_info("ROACH THERMISTOR 64 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 39)));
    blast_info("ROACH THERMISTOR 65 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 40)));
    blast_info("ROACH THERMISTOR 66 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 41)));
    blast_info("ROACH THERMISTOR 67 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 42)));
    blast_info("ROACH THERMISTOR 68 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 43)));
    blast_info("ROACH THERMISTOR 69 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 44)));
    blast_info("ROACH THERMISTOR 70 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 45)));
    blast_info("ROACH THERMISTOR 71 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 46)));
    blast_info("ROACH THERMISTOR 72 = %f", temp(labjack_get_value(LABJACK_MULT_OF, 47)));
    */
    /*
    SET_SCALED_VALUE(thermistor_63_Addr, labjack_get_value(LABJACK_MULT_OF, 38));
    SET_SCALED_VALUE(thermistor_64_Addr, labjack_get_value(LABJACK_MULT_OF, 39));
    SET_SCALED_VALUE(thermistor_65_Addr, labjack_get_value(LABJACK_MULT_OF, 40));
    SET_SCALED_VALUE(thermistor_66_Addr, labjack_get_value(LABJACK_MULT_OF, 41));
    SET_SCALED_VALUE(thermistor_67_Addr, labjack_get_value(LABJACK_MULT_OF, 42));
    SET_SCALED_VALUE(thermistor_68_Addr, labjack_get_value(LABJACK_MULT_OF, 43));
    SET_SCALED_VALUE(thermistor_69_Addr, labjack_get_value(LABJACK_MULT_OF, 44));
    SET_SCALED_VALUE(thermistor_70_Addr, labjack_get_value(LABJACK_MULT_OF, 45));
    SET_SCALED_VALUE(thermistor_71_Addr, labjack_get_value(LABJACK_MULT_OF, 46));
    SET_SCALED_VALUE(thermistor_72_Addr, labjack_get_value(LABJACK_MULT_OF, 47));
    */
    SET_SCALED_VALUE(thermistor_73_Addr, labjack_get_value(LABJACK_MULT_OF, 49));
    SET_SCALED_VALUE(thermistor_74_Addr, labjack_get_value(LABJACK_MULT_OF, 50));
}

// updates clinometers instead of thermometers
static void update_clinometers(void) {
    static int first_time_clin = 1;
    static channel_t* clin_1_x_Addr;
    static channel_t* clin_1_y_Addr;
    static channel_t* clin_2_x_Addr;
    static channel_t* clin_2_y_Addr;
    static channel_t* clin_1_t_Addr;
    static channel_t* clin_2_t_Addr;
    if (first_time_clin == 1) {
        first_time_clin = 0;
        clin_1_x_Addr = channels_find_by_name("clin_of_x");
        clin_1_y_Addr = channels_find_by_name("clin_of_y");
        clin_2_x_Addr = channels_find_by_name("clin_if_x");
        clin_2_y_Addr = channels_find_by_name("clin_if_y");
        clin_1_t_Addr = channels_find_by_name("clin_of_t");
        clin_2_t_Addr = channels_find_by_name("clin_if_t");
    }
    SET_SCALED_VALUE(clin_1_x_Addr, labjack_get_value(LABJACK_OF_3, 10));
    SET_SCALED_VALUE(clin_1_y_Addr, labjack_get_value(LABJACK_OF_3, 11));
    SET_SCALED_VALUE(clin_2_x_Addr, labjack_get_value(LABJACK_OF_3, 12));
    SET_SCALED_VALUE(clin_2_y_Addr, labjack_get_value(LABJACK_OF_3, 13));
    SET_SCALED_VALUE(clin_1_t_Addr, labjack_get_value(LABJACK_MULT_OF, 0));
    SET_SCALED_VALUE(clin_2_t_Addr, labjack_get_value(LABJACK_MULT_OF, 1));
}
// same deal for current sensors
void process_current_sensors(void) {
    static int first_time_current = 1;
    if (first_time_current == 1) {
        first_time_current = 0;
        for (int i = 0; i < 17; i++) {
            init_labjack_10hz_filter(&OFCurFilt[i]);
        }
    }
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 0)*CURLOOP_CONV, &OFCurFilt[0]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 1)*CURLOOP_CONV, &OFCurFilt[1]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 2)*CURLOOP_CONV, &OFCurFilt[2]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 3)*CURLOOP_CONV, &OFCurFilt[3]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 4)*CURLOOP_CONV, &OFCurFilt[4]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 5)*CURLOOP_CONV, &OFCurFilt[5]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 6)*CURLOOP_CONV, &OFCurFilt[6]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 7)*CURLOOP_CONV, &OFCurFilt[7]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 8)*CURLOOP_CONV, &OFCurFilt[8]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_OF_3, 9)*CURLOOP_CONV, &OFCurFilt[9]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_OF, 48)*CURLOOP_CONV, &OFCurFilt[10]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_OF, 52)*CURLOOP_CONV, &OFCurFilt[11]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_OF, 53)*CURLOOP_CONV, &OFCurFilt[12]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_OF, 54)*CURLOOP_CONV, &OFCurFilt[13]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_OF, 55)*CURLOOP_CONV, &OFCurFilt[14]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_OF, 72)*CURLOOP_CONV, &OFCurFilt[15]);
    filter_labjack_channel_10hz(labjack_get_value(LABJACK_MULT_OF, 74)*CURLOOP_CONV, &OFCurFilt[16]);
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
    static channel_t* current_loop_11_Addr;
//     static channel_t* current_loop_12_Addr;
//     static channel_t* current_loop_13_Addr;
//     static channel_t* current_loop_14_Addr;
//     static channel_t* current_loop_15_Addr;
//     static channel_t* current_loop_16_Addr;
//     static channel_t* current_loop_17_Addr;
    if (first_time_current == 1) {
        first_time_current = 0;
        current_loop_1_Addr = channels_find_by_name("current_eth_switch");
        current_loop_2_Addr = channels_find_by_name("current_fc1");
        current_loop_3_Addr = channels_find_by_name("current_xsc1");
        current_loop_4_Addr = channels_find_by_name("current_fc2");
        current_loop_5_Addr = channels_find_by_name("current_xsc0");
        current_loop_6_Addr = channels_find_by_name("current_ele_mot");
        current_loop_7_Addr = channels_find_by_name("current_pivot");
        current_loop_8_Addr = channels_find_by_name("current_rw_mot");
        current_loop_9_Addr = channels_find_by_name("current_hd_pv");
        current_loop_10_Addr = channels_find_by_name("current_gyros");
        current_loop_11_Addr = channels_find_by_name("current_data_transmit");
        // current_loop_12_Addr = channels_find_by_name("current_if1");
        // current_loop_13_Addr = channels_find_by_name("current_if2");
        // current_loop_14_Addr = channels_find_by_name("current_if3");
        // current_loop_15_Addr = channels_find_by_name("current_if4");
        // current_loop_16_Addr = channels_find_by_name("current_if5");
        // current_loop_17_Addr = channels_find_by_name("current_if6");
        for (int i = 0; i < 17; i++) {
            init_labjack_10hz_filter(&OFCurFilt[i]);
        }
    }
//    blast_info("Current Loops: Relay #4 = %f, Relay #8 = %f",
//    			labjack_get_value(LABJACK_OF_3, 3)*CURLOOP_CONV, labjack_get_value(LABJACK_OF_3, 7)*CURLOOP_CONV);
    SET_SCALED_VALUE(current_loop_1_Addr, OFCurFilt[0].filt_val);
    SET_SCALED_VALUE(current_loop_2_Addr, OFCurFilt[1].filt_val);
    SET_SCALED_VALUE(current_loop_3_Addr, OFCurFilt[2].filt_val);
    SET_SCALED_VALUE(current_loop_4_Addr, OFCurFilt[3].filt_val);
    SET_SCALED_VALUE(current_loop_5_Addr, OFCurFilt[4].filt_val);
    SET_SCALED_VALUE(current_loop_6_Addr, OFCurFilt[5].filt_val);
    SET_SCALED_VALUE(current_loop_7_Addr, OFCurFilt[6].filt_val);
    SET_SCALED_VALUE(current_loop_8_Addr, OFCurFilt[7].filt_val);
    SET_SCALED_VALUE(current_loop_9_Addr, OFCurFilt[8].filt_val);
    SET_SCALED_VALUE(current_loop_10_Addr, OFCurFilt[9].filt_val);
    SET_SCALED_VALUE(current_loop_11_Addr, OFCurFilt[10].filt_val);
// These are current sensing loops that don't exist
//    SET_SCALED_VALUE(current_loop_12_Addr, OFCurFilt[11].filt_val);
//     SET_SCALED_VALUE(current_loop_13_Addr, OFCurFilt[12].filt_val);
//     SET_SCALED_VALUE(current_loop_14_Addr, OFCurFilt[13].filt_val);
//     SET_SCALED_VALUE(current_loop_15_Addr, OFCurFilt[14].filt_val);
//     SET_SCALED_VALUE(current_loop_16_Addr, OFCurFilt[15].filt_val);
//     SET_SCALED_VALUE(current_loop_17_Addr, OFCurFilt[16].filt_val);
}
/*
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
        thermistor_67_Addr = channels_find_by_name("thermistor_67");
        thermistor_68_Addr = channels_find_by_name("thermistor_68");
        thermistor_69_Addr = channels_find_by_name("thermistor_69");
        thermistor_70_Addr = channels_find_by_name("thermistor_70");
        thermistor_71_Addr = channels_find_by_name("thermistor_71");
        thermistor_72_Addr = channels_find_by_name("thermistor_72");
        thermistor_73_Addr = channels_find_by_name("thermistor_73");
        thermistor_74_Addr = channels_find_by_name("thermistor_74");

    }
    SET_SCALED_VALUE(thermistor_29_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_30_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_31_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_32_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_33_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_34_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_35_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_36_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_37_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_38_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_39_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_40_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_41_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_42_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_43_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_44_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_45_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_46_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_47_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_48_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_49_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_50_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_51_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_52_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_53_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_54_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_55_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_56_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_57_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_58_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_59_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_60_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_61_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_62_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_63_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_64_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_65_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_66_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_67_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_68_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_69_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_70_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_71_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_72_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_73_Addr, labjack_get_value(6, 4));
    SET_SCALED_VALUE(thermistor_74_Addr, labjack_get_value(6, 4));
}
*/

void update_status() {
    static int first_time = 1;
    static channel_t * labjack_conn_status_Addr = NULL;

    if (first_time) {
        labjack_conn_status_Addr = channels_find_by_name("labjack_conn_status");
        first_time = 0;
    }

    uint16_t labjack_conn_status = 0;

    for (int i = 0; i < NUM_LABJACKS; i++) {
        labjack_conn_status |= state[i].connected << i;
    }
    SET_UINT16(labjack_conn_status_Addr, labjack_conn_status);
}

void outer_frame_200hz(int setting) {
    if (setting == 1 && state[2].connected && state[3].connected && state[4].connected) {
        process_current_sensors();
    }
}

void outer_frame_1hz(int setting) {
    if (setting == 1 && state[2].connected && state[3].connected && state[4].connected) {
        update_current_sensors();
        update_thermistors();
        update_clinometers();
    }
    update_status();
}

// below is only used for testing the multiplexed labjack in the thermal vac chamber

void update_mult_vac(void) {
    static int counter = 1;
    if (counter == 1 && state[6].connected) {
        blast_info("multiplexed");
        for (int i = 0; i < 84; i++) {
            blast_info(" %d is %f", i, labjack_get_value(6, i));
        }
        blast_info("of1");
        for (int j = 0; j < 14; j++) {
            blast_info(" %d is %f", j, labjack_get_value(2, j));
        }
        blast_info("of2");
        for (int k = 0; k < 14; k++) {
            blast_info(" %d is %f", k, labjack_get_value(3, k));
        }
    }
    if (counter < 50) {
        counter++;
    }
    if (counter == 50) {
        counter = 1;
    }
}




