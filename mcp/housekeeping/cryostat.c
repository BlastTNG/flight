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
 
 created by Ian Lowe 5-13-16
 **************************************************************************/


/*************************************************************************
 
 crystat.c -- mcp code to handle cryostat control. Including heaters and
 fridge cycles.
 
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

/*************************************************************************/
/* CryoControl: Control valves, heaters, and calibrator (a fast control) */
/*************************************************************************/
// TODO(IAN): add use of InCharge global variable so only incharge comp runs
// TODO(IAN): write cal_length to the frame, add periodic option
// write level length, do_cal_pulse, everything!

typedef struct {
    uint16_t cal_length;
    uint16_t level_length;
} cryo_control_t;

typedef struct {
    uint16_t heater_300mk, charcoal_hs, charcoal, lna_250, lna_350, lna_500, heater_1k;
    uint16_t heater_status;
} heater_control_t;

heater_control_t heater_state;
cryo_control_t cryo_state;

static void update_heater_values(void) {
    heater_state.heater_300mk = CommandData.Cryo.heater_300mk;
    heater_state.charcoal_hs = CommandData.Cryo.charcoal_hs;
    heater_state.charcoal = CommandData.Cryo.charcoal;
    heater_state.lna_250 = CommandData.Cryo.lna_250;
    heater_state.lna_350 = CommandData.Cryo.lna_350;
    heater_state.lna_500 = CommandData.Cryo.lna_500;
    heater_state.heater_1k = CommandData.Cryo.heater_1k;
    heater_state.heater_status = CommandData.Cryo.heater_status;
}

void heater_all_off(void) {
    CommandData.Cryo.heater_300mk = 0;
    CommandData.Cryo.charcoal_hs = 0;
    CommandData.Cryo.charcoal = 0;
    CommandData.Cryo.lna_250 = 0;
    CommandData.Cryo.lna_350 = 0;
    CommandData.Cryo.lna_500 = 0;
    CommandData.Cryo.heater_1k = 0;
    CommandData.Cryo.heater_status = 0;
    update_heater_values();
    heater_write(LABJACK_CRYO_1, HEATER_300MK_COMMAND, heater_state.heater_300mk);
    heater_write(LABJACK_CRYO_1, HEATER_1K_COMMAND, heater_state.heater_1k);
    heater_write(LABJACK_CRYO_1, LNA_250_COMMAND, heater_state.lna_250);
    heater_write(LABJACK_CRYO_1, LNA_350_COMMAND, heater_state.lna_350);
    heater_write(LABJACK_CRYO_1, LNA_500_COMMAND, heater_state.lna_500);
    heater_write(LABJACK_CRYO_1, CHARCOAL_COMMAND, heater_state.charcoal);
    heater_write(LABJACK_CRYO_1, CHARCOAL_HS_COMMAND, heater_state.charcoal_hs);
}

void heater_control(void) {
    static int first_heater = 1;
    static channel_t* heater_status_Addr;
    if (first_heater == 1) {
        heater_status_Addr = channels_find_by_name("heater_status_write");
        // read in the heater channels and update accordingly here
    }
    if (CommandData.Cryo.heater_update == 1) {
        CommandData.Cryo.heater_update = 0;
        update_heater_values();
        heater_write(LABJACK_CRYO_1, HEATER_300MK_COMMAND, heater_state.heater_300mk);
        heater_write(LABJACK_CRYO_1, HEATER_1K_COMMAND, heater_state.heater_1k);
        heater_write(LABJACK_CRYO_1, LNA_250_COMMAND, heater_state.lna_250);
        heater_write(LABJACK_CRYO_1, LNA_350_COMMAND, heater_state.lna_350);
        heater_write(LABJACK_CRYO_1, LNA_500_COMMAND, heater_state.lna_500);
        heater_write(LABJACK_CRYO_1, CHARCOAL_COMMAND, heater_state.charcoal);
        heater_write(LABJACK_CRYO_1, CHARCOAL_HS_COMMAND, heater_state.charcoal_hs);
        SET_SCALED_VALUE(heater_status_Addr, CommandData.Cryo.heater_status);
    }
}

void heater_read(void) {
    static int first_time_read = 1;
    static channel_t* heater_read_Addr;
    static channel_t* h300mk_Addr;
    static channel_t* heaterstatus_Addr;
    double h300mk = 0;
    uint16_t read_field;
    if (first_time_read == 1) {
        heater_read_Addr = channels_find_by_name("heater_status_read");
        h300mk_Addr = channels_find_by_name("HEATER_300MK_READ");
        heaterstatus_Addr = channels_find_by_name("heater_status_write");
    }
    // below resets the read field and gets all information from the inputs
    read_field = 0;
    GET_VALUE(h300mk_Addr, h300mk);
    if (h300mk < -0.1) { // may need to be changed in the future
        read_field += 1;
    }
    read_field += 2*labjack_get_value(LABJACK_CRYO_2, READ_1K_HEATER);
    read_field += 4*labjack_get_value(LABJACK_CRYO_2, READ_250LNA);
    read_field += 8*labjack_get_value(LABJACK_CRYO_2, READ_350LNA);
    read_field += 16*labjack_get_value(LABJACK_CRYO_2, READ_500LNA);
    read_field += 32*labjack_get_value(LABJACK_CRYO_2, READ_CHARCOAL);
    read_field += 64*labjack_get_value(LABJACK_CRYO_2, READ_CHARCOAL_HS);
    SET_SCALED_VALUE(heater_read_Addr, read_field);
    if (CommandData.Cryo.sync == 1) {
        CommandData.Cryo.sync = 0;
        SET_SCALED_VALUE(heaterstatus_Addr, read_field);
    }
}
/*
Heater status bits
1 = heater 300mk
2 = heater 1k
4 = lna 250
8 = lna 350
16 = lna 500
32 = charcoal
64 = charcoal_hs
*/
void cal_control(void) {
    if (CommandData.Cryo.do_cal_pulse) {
        cryo_state.cal_length = CommandData.Cryo.cal_length;
        CommandData.Cryo.do_cal_pulse = 0;
    }
    static int pulsed = 0;
    if (cryo_state.cal_length > 0) {
        if (!pulsed) {
            pulsed = 1;
            heater_write(LABJACK_CRYO_1, CALLAMP_COMMAND, 1);
        }
        cryo_state.cal_length--;
    } else {
        if (pulsed) {
            heater_write(LABJACK_CRYO_1, CALLAMP_COMMAND, 0);
            pulsed = 0;
        }
    }
}

void level_control(void) {
    if (CommandData.Cryo.do_level_pulse) {
        cryo_state.level_length = CommandData.Cryo.level_length;
        CommandData.Cryo.do_level_pulse = 0;
    }
    static int l_pulsed = 0;
    if (cryo_state.level_length > 0) {
        if (!l_pulsed) {
            l_pulsed = 1;
            heater_write(LABJACK_CRYO_1, LEVEL_SENSOR_COMMAND, 1);
        }
        cryo_state.level_length--;
    } else {
        if (l_pulsed) {
            heater_write(LABJACK_CRYO_1, LEVEL_SENSOR_COMMAND, 0);
            l_pulsed = 0;
        }
    }
}

void test_frequencies(void) {
    static channel_t* test_Addr;
    static int first_test = 1;
    if (first_test) {
        test_Addr = channels_find_by_name("test_values");
    }
    SET_SCALED_VALUE(test_Addr, labjack_get_value(1, 12));
}

void tie_up(void) {
    static int good = 1;
    while (good) {
        good = 1;
    }
}


void test_labjacks(int m_which) {
    float test0, test1, test2, test3, test4;
    float test5, test6, test7, test8, test9;
    float test10, test11, test12, test13;
    test0 = labjack_get_value(m_which, 0);
    test1 = labjack_get_value(m_which, 1);
    test2 = labjack_get_value(m_which, 2);
    test3 = labjack_get_value(m_which, 3);
    test4 = labjack_get_value(m_which, 4);
    test5 = labjack_get_value(m_which, 5);
    test6 = labjack_get_value(m_which, 6);
    test7 = labjack_get_value(m_which, 7);
    test8 = labjack_get_value(m_which, 8);
    test9 = labjack_get_value(m_which, 9);
    test10 = labjack_get_value(m_which, 10);
    test11 = labjack_get_value(m_which, 11);
    test12 = labjack_get_value(m_which, 12);
    test13 = labjack_get_value(m_which, 13);
    blast_warn(" AIN 0 is %f", test0);
    blast_warn(" AIN 1 is %f", test1);
    blast_warn(" AIN 2 is %f", test2);
    blast_warn(" AIN 3 is %f", test3);
    blast_warn(" AIN 4 is %f", test4);
    blast_warn(" AIN 5 is %f", test5);
    blast_warn(" AIN 6 is %f", test6);
    blast_warn(" AIN 7 is %f", test7);
    blast_warn(" AIN 8 is %f", test8);
    blast_warn(" AIN 9 is %f", test9);
    blast_warn(" AIN 10 is %f", test10);
    blast_warn(" AIN 11 is %f", test11);
    blast_warn(" AIN 12 is %f", test12);
    blast_warn(" AIN 13 is %f", test13);
}

void test_read(void) { // labjack dio reads 1 when open, 0 when shorted to gnd.
    static channel_t* reader;
    static int firsttime = 1;
    if (firsttime) {
        firsttime = 0;
        reader = channels_find_by_name("read_dio");
    }
    // blast_warn("Fio0 on LABJACK 1 value is %u", labjack_read_dio(LABJACK_CRYO_1, 2000));
    SET_SCALED_VALUE(reader, labjack_read_dio(LABJACK_CRYO_1, 2000));
}

void read_thermometers(void) {
    static int firsttime_therm = 1;
    static channel_t* rox_fpa_1k_Addr;
    static channel_t* rox_250_fpa_Addr;
    static channel_t* rox_1k_plate_Addr;
    static channel_t* rox_300mk_strap_Addr;
    static channel_t* rox_350_fpa_Addr;
    static channel_t* rox_he4_pot_Addr;
    static channel_t* rox_he3_fridge_Addr;
    static channel_t* rox_500_fpa_Addr;
    static channel_t* rox_bias_Addr;

    static channel_t* diode_charcoal_hs_Addr;
    static channel_t* diode_vcs2_filt_Addr;
    static channel_t* diode_250fpa_Addr;
    static channel_t* diode_hwp_Addr;
    static channel_t* diode_vcs1_hx_Addr;
    static channel_t* diode_1k_fridge_Addr;
    static channel_t* diode_4k_plate_Addr;
    static channel_t* diode_vcs1_filt_Addr;
    static channel_t* diode_m3_Addr;
    static channel_t* diode_charcoal_Addr;
    static channel_t* diode_ob_filter_Addr;
    static channel_t* diode_vcs2_plate_Addr;
    static channel_t* diode_m4_Addr;
    static channel_t* diode_4k_filt_Addr;
    static channel_t* diode_vcs2_hx_Addr;
    static channel_t* diode_vcs1_plate_Addr;

    static channel_t* level_sensor_read_Addr;
    static channel_t* heater_300mk_Addr;
    static channel_t* cal_lamp_Addr;

    if (firsttime_therm == 1) {
        rox_fpa_1k_Addr = channels_find_by_name("tr_fpa_1k");
        rox_250_fpa_Addr = channels_find_by_name("tr_250_fpa");
        rox_1k_plate_Addr = channels_find_by_name("tr_1k_plate");
        rox_300mk_strap_Addr = channels_find_by_name("tr_300mk_strap");
        rox_350_fpa_Addr = channels_find_by_name("tr_350_fpa");
        rox_he4_pot_Addr = channels_find_by_name("tr_he4_pot");
        rox_he3_fridge_Addr = channels_find_by_name("tr_he3_fridge");
        rox_500_fpa_Addr = channels_find_by_name("tr_500_fpa");
        rox_bias_Addr = channels_find_by_name("rox_bias");
        // rox channel pointers defined above
        // diode channel pointers defined below
        diode_charcoal_hs_Addr = channels_find_by_name("td_charcoal_hs");
        diode_vcs2_filt_Addr = channels_find_by_name("td_vcs2_filt");
        diode_250fpa_Addr = channels_find_by_name("td_250fpa");
        diode_hwp_Addr = channels_find_by_name("td_hwp");
        diode_vcs1_hx_Addr = channels_find_by_name("td_vcs1_hx");
        diode_1k_fridge_Addr = channels_find_by_name("td_1k_fridge"); // MAPS TO HE4 POT
        diode_4k_plate_Addr = channels_find_by_name("td_4k_plate"); // SAME AS COLD PLATE
        diode_vcs1_filt_Addr = channels_find_by_name("td_vcs1_filt");
        diode_m3_Addr = channels_find_by_name("td_m3");
        diode_charcoal_Addr = channels_find_by_name("td_charcoal");
        diode_ob_filter_Addr = channels_find_by_name("td_ob_filter");
        diode_vcs2_plate_Addr = channels_find_by_name("td_vcs2_plate");
        diode_m4_Addr = channels_find_by_name("td_m4");
        diode_4k_filt_Addr = channels_find_by_name("td_4k_filt");
        diode_vcs2_hx_Addr = channels_find_by_name("td_vcs2_hx");
        diode_vcs1_plate_Addr = channels_find_by_name("td_vcs1_plate");
        // other channels defined below
        level_sensor_read_Addr = channels_find_by_name("level_sensor_read");
        cal_lamp_Addr = channels_find_by_name("cal_lamp_read");
        heater_300mk_Addr = channels_find_by_name("heater_300mk_read");
        firsttime_therm = 0;
    }

    SET_SCALED_VALUE(diode_charcoal_hs_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_CHARCOAL_HS));
    SET_SCALED_VALUE(diode_vcs2_filt_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_VCS2_FILT));
    SET_SCALED_VALUE(diode_250fpa_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_250FPA));
    SET_SCALED_VALUE(diode_hwp_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_HWP));
    SET_SCALED_VALUE(diode_vcs1_hx_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_VCS1_HX));
    SET_SCALED_VALUE(diode_1k_fridge_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_1K_FRIDGE));
    SET_SCALED_VALUE(diode_4k_plate_Addr, labjack_get_value(LABJACK_CRYO_2, DIODE_4K_PLATE));
    SET_SCALED_VALUE(diode_vcs1_filt_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_VCS1_FILT));
    SET_SCALED_VALUE(diode_m3_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_M3));
    SET_SCALED_VALUE(diode_charcoal_Addr, labjack_get_value(LABJACK_CRYO_2, DIODE_CHARCOAL));
    SET_SCALED_VALUE(diode_ob_filter_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_OB_FILTER));
    SET_SCALED_VALUE(diode_vcs2_plate_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_VCS2_PLATE));
    SET_SCALED_VALUE(diode_m4_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_M4));
    SET_SCALED_VALUE(diode_4k_filt_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_4K_FILT));
    SET_SCALED_VALUE(diode_vcs2_hx_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_VCS2_HX));
    SET_SCALED_VALUE(diode_vcs1_plate_Addr, labjack_get_value(LABJACK_CRYO_1, DIODE_VCS1_PLATE));
    // above are the diodes, below, the ROXes
    SET_SCALED_VALUE(rox_fpa_1k_Addr, labjack_get_value(LABJACK_CRYO_2, ROX_FPA_1K));
    SET_SCALED_VALUE(rox_250_fpa_Addr, labjack_get_value(LABJACK_CRYO_2, ROX_250_FPA));
    SET_SCALED_VALUE(rox_1k_plate_Addr, labjack_get_value(LABJACK_CRYO_2, ROX_1K_STRAP));
    SET_SCALED_VALUE(rox_300mk_strap_Addr, labjack_get_value(LABJACK_CRYO_2, ROX_300MK_STRAP));
    SET_SCALED_VALUE(rox_350_fpa_Addr, labjack_get_value(LABJACK_CRYO_2, ROX_350_FPA));
    SET_SCALED_VALUE(rox_he4_pot_Addr, labjack_get_value(LABJACK_CRYO_2, ROX_HE4_POT));
    SET_SCALED_VALUE(rox_he3_fridge_Addr, labjack_get_value(LABJACK_CRYO_2, ROX_HE3_FRIDGE));
    SET_SCALED_VALUE(rox_500_fpa_Addr, labjack_get_value(LABJACK_CRYO_2, ROX_500_FPA));
    SET_SCALED_VALUE(rox_bias_Addr, labjack_get_value(LABJACK_CRYO_2, BIAS));
    // below are the random cryo labjack channels
    SET_SCALED_VALUE(level_sensor_read_Addr, labjack_get_value(LABJACK_CRYO_2, LEVEL_SENSOR_READ));
    SET_SCALED_VALUE(cal_lamp_Addr, labjack_get_value(LABJACK_CRYO_2, CAL_LAMP_READ));
    SET_SCALED_VALUE(heater_300mk_Addr, labjack_get_value(LABJACK_CRYO_2, HEATER_300MK_READ));
}

void test_cycle(void) {
    static channel_t* test_channel;
    static int first_test = 1;
    float t_test;
    if (first_test == 1) {
        first_test = 0;
        test_channel = channels_find_by_name("tr_350_fpa");
    }
    GET_VALUE(test_channel, t_test);
    blast_warn("channel is %f", t_test);
}

auto_cycle_allowed, force_cycle, auto_cycling;

void autocycle_ian(void)
{
    static channel_t* tfpa250_Addr; // set channel address pointers
    static channel_t* tfpa350_Addr;
    static channel_t* tfpa500_Addr;
    static channel_t* tcharcoal_Addr;
    static channel_t* tcharcoalhs_Addr;

    static int firsttime = 1;
    static int cooling_counter = 0;
    static int start_up_counter = 0;
    double t250, t350, t500, tcharcoal, tcharcoalhs;
    double t250_old, t350_old, t500_old, tcharcoal_old, tcharcoalhs_old;
    static double tcrit_fpa = 0.31; // this will likely get changed in the future
    static double tcrit_charcoal = 37.0; // temp of charcoal
    static int cooling_length = 3600;
    if (firsttime == 1) {
        firsttime = 0;
        tfpa250_Addr = channels_find_by_name("tr_250_fpa");
        tfpa350_Addr = channels_find_by_name("tr_350_fpa");
        tfpa500_Addr = channels_find_by_name("tr_500_fpa");
        tcharcoal_Addr = channels_find_by_name("td_charcoal");
        tcharcoalhs_Addr = channels_find_by_name("td_charcoal_hs");
    }
    if (start_up_counter < 60) { // won't try to autocycle in the first minute
        start_up_counter++;
    } else {
        firsttime++;
    }
}











