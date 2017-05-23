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
#include "labjack_functions.h"
#include "blast.h"
#include "multiplexed_labjack.h"
#include "bias_tone.h"

/*************************************************************************/
/* CryoControl: Control valves, heaters, and calibrator (a fast control) */
/*************************************************************************/
// TODO(IAN): add use of InCharge global variable so only incharge comp runs
// TODO(IAN): write cal_length to the frame, add periodic option
// write level length, do_cal_pulse, everything!

typedef struct { // structure to read commands for level and cal pulses
    uint16_t cal_length;
    uint16_t level_length;
} cryo_control_t;

typedef struct { // structure that contains data about heater commands
    uint16_t heater_300mk, charcoal_hs, charcoal, lna_250, lna_350, lna_500, heater_1k;
    uint16_t heater_status;
} heater_control_t;

typedef struct { // structure that contains all of the fridge cycling information
    int standby, cooling, burning_off, heating, heat_delay;
    double t250, t350, t500, tcharcoal, tcharcoalhs;
    double t250_old, t350_old, t500_old, tcharcoal_old, tcharcoalhs_old;
    channel_t* tfpa250_Addr; // set channel address pointers
    channel_t* tfpa350_Addr;
    channel_t* tfpa500_Addr;
    channel_t* tcharcoal_Addr;
    channel_t* tcharcoalhs_Addr;
    channel_t* cycle_state_Addr;
    int start_up_counter;
    int burning_length;
    int reset_cycle;
    int burning_counter;
    int reheating;
    // change these to the 16bit values. (uint16_t)
    uint16_t tcrit_fpa; // this will likely get changed in the future
    // likely have 3 different t crit
    uint16_t tcrit_charcoal; // temp of charcoal
    uint16_t tmin_charcoal; // minimum during burnoff
} cycle_control_t;

cycle_control_t cycle_state;

heater_control_t heater_state;

cryo_control_t cryo_state;
// pulls data for heater state writing
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
// turns all of the heaters off
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
// function that runs in the MCP loop to control heaters
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
// function that runs in MCP loop to read in the heater status bits
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

// function that creates cal lamp pulses via the mcp loop
// utilizes the cryo control structure
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

// function that creates level sensor pulses via the mcp loop
// utilizes the cryo control structure
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
// function that breaks mcp, for testing WD
void tie_up(void) {
    static int good = 1;
    while (good) {
        good = 1;
    }
}

// prints out all AIN values for an arbitrary labjack
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

// test function for reading a digital input
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


// this function sets all the addresses for the thermometry on the cryostat
// then runs in the MCP main loop updating the temperatures at 1hz
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

void read_chopper(void)
{
	/*
		Code to read in the chopper signal for the xy stage.
		Included here b/c it's an analog in read through a LabJack.
	*/	

	static channel_t* stage_chopper_Addr;
	static int firsttime = 1;

	if (firsttime == 1) {
		firsttime = 0;
		stage_chopper_Addr = channels_find_by_name("stage_chopper");
	}
    SET_SCALED_VALUE(stage_chopper_Addr, labjack_get_value(1, 12));
}

// test read for a channel written from the thermometry function
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
// addresses the channels used in the auto cycle structure
static void init_cycle_channels(void) {
    cycle_state.tfpa250_Addr = channels_find_by_name("tr_250_fpa");
    cycle_state.tfpa350_Addr = channels_find_by_name("tr_350_fpa");
    cycle_state.tfpa500_Addr = channels_find_by_name("tr_500_fpa");
    cycle_state.tcharcoal_Addr = channels_find_by_name("td_charcoal");
    cycle_state.tcharcoalhs_Addr = channels_find_by_name("td_charcoal_hs");
    blast_info("on to startup phase");
}

static void init_cycle_values(void) {
    cycle_state.standby = 0;
    cycle_state.cooling = 0;
    cycle_state.burning_off = 0;
    cycle_state.heating = 0;
    cycle_state.heat_delay = 0;
    cycle_state.tcharcoal = 0;
    cycle_state.tcharcoal_old = 0;
    cycle_state.t250 = 0;
    cycle_state.t350 = 0;
    cycle_state.t500 = 0;
    cycle_state.tcharcoalhs = 0;
    cycle_state.tcharcoalhs_old = 0;
    cycle_state.t250_old = 0;
    cycle_state.t350_old = 0;
    cycle_state.t500_old = 0;
    cycle_state.start_up_counter = 0;
    cycle_state.burning_length = 3600;
    cycle_state.burning_counter = 0;
    cycle_state.reheating = 0;
    blast_info("values written");
    cycle_state.tcrit_charcoal = 49441; // temp of charcoal
    cycle_state.tmin_charcoal = 49834;
}
// performs the startup operations of the cycle,
// averaging the temperatures for 60 seconds before any other actions
static void start_cycle(void) {
    if (cycle_state.start_up_counter < 60) { // won't try to autocycle in the first minute
        cycle_state.start_up_counter++;
        GET_VALUE(cycle_state.tfpa250_Addr, cycle_state.t250_old);
        GET_VALUE(cycle_state.tfpa350_Addr, cycle_state.t350_old);
        GET_VALUE(cycle_state.tfpa500_Addr, cycle_state.t500_old);
        blast_info("got values");
        cycle_state.t250 += cycle_state.t250_old;
        cycle_state.t350 += cycle_state.t350_old;
        cycle_state.t500 += cycle_state.t500_old;
        blast_info("added values...");
        if (cycle_state.start_up_counter == 60) {
            cycle_state.t250 = cycle_state.t250/60;
            cycle_state.t350 = cycle_state.t350/60;
            cycle_state.t500 = cycle_state.t500/60;
            cycle_state.standby = 1;
            blast_info("moving to standby");
        }
    }
}
// most the cycle is spent in this phase, every second the previous values are
// multiplied by 0.98333 and the newest value is added at 0.016667x contribution
// then the system check to see if any of the averages are out of the acceptable range of operating temperatures
// if so, it moves to the next phase
static void standby_cycle(void) {
    if (cycle_state.standby == 1) {
        cycle_state.t250_old = cycle_state.t250;
        cycle_state.t350_old = cycle_state.t350;
        cycle_state.t500_old = cycle_state.t500;
        GET_VALUE(cycle_state.tfpa250_Addr, cycle_state.t250);
        GET_VALUE(cycle_state.tfpa350_Addr, cycle_state.t350);
        GET_VALUE(cycle_state.tfpa500_Addr, cycle_state.t500);
        cycle_state.t250 = cycle_state.t250_old*(59/60) + cycle_state.t250*(1/60);
        cycle_state.t350 = cycle_state.t350_old*(59/60) + cycle_state.t350*(1/60);
        cycle_state.t500 = cycle_state.t500_old*(59/60) + cycle_state.t500*(1/60);
        if (cycle_state.t250 > cycle_state.tcrit_fpa) {
            cycle_state.standby = 0;
            cycle_state.heating = 1;
        } else {
            if (cycle_state.t350 > cycle_state.tcrit_fpa) {
                cycle_state.standby = 0;
                cycle_state.heating = 1;
            } else {
                if (cycle_state.t500 > cycle_state.tcrit_fpa) {
                    cycle_state.standby = 0;
                    cycle_state.heating = 1;
                    blast_info("moving on to the heating phase");
                }
            }
        }
    }
}
// during this portion of the cycle, we turn off the charcoal hs and allow it to cool
// for a few minutes. Afterwards the charcoal is turned on until it reaches a critical
// temperature (~37K). At which point it is turned off and we move on.
static void heating_cycle(void) {
    if (cycle_state.heating == 1) {
        if (cycle_state.heat_delay == 0) {
            CommandData.Cryo.heater_update = 1;
            CommandData.Cryo.charcoal_hs = 0;
            cycle_state.heat_delay++;
        }
        if (cycle_state.heat_delay < 180) { // give the charcoal HS time to cool off
            // we could add the open the pumped pot valve here.
            cycle_state.heat_delay++;
        } else {
            CommandData.Cryo.heater_update = 1;
            CommandData.Cryo.charcoal = 1;
        }
        GET_VALUE(cycle_state.tcharcoal_Addr, cycle_state.tcharcoal);
        if (cycle_state.tcharcoal < cycle_state.tcrit_charcoal) {
            CommandData.Cryo.heater_update = 1;
            CommandData.Cryo.charcoal = 0;
            cycle_state.heating = 0;
            cycle_state.burning_off = 1;
            cycle_state.burning_counter = 0;
            blast_info("heating done, burning off");
        }
    }
}
// in the burnoff phase, we monitor the temperature of the charcoal heater while the counter ticks along
// if the temperature drops below a minimum threshold, we reheat the charcoal and then continue along
// when the timer runs out, the charcoal heat switch is turned on
static void burnoff_cycle(void) {
    if (cycle_state.burning_off == 1) {
        GET_VALUE(cycle_state.tcharcoal_Addr, cycle_state.tcharcoal);
        if (cycle_state.burning_counter < cycle_state.burning_length && cycle_state.reheating == 0) {
            cycle_state.burning_counter++;
        }
        if (cycle_state.tcharcoal > cycle_state.tmin_charcoal && cycle_state.reheating == 0) {
            cycle_state.reheating = 1;
            CommandData.Cryo.heater_update = 1;
            CommandData.Cryo.charcoal = 1;
            blast_info("reheating, dropped below boiling temp");
        }
        if (cycle_state.reheating == 1 && cycle_state.tcharcoal < cycle_state.tcrit_charcoal) {
            cycle_state.reheating = 0;
            CommandData.Cryo.heater_update = 1;
            CommandData.Cryo.charcoal = 0;
            blast_info("reheating over, back to burning off");
        }
        if (cycle_state.burning_counter == cycle_state.burning_length) {
            cycle_state.burning_off = 0;
            cycle_state.cooling = 1;
            CommandData.Cryo.heater_update = 1;
            CommandData.Cryo.charcoal_hs = 0;
            GET_VALUE(cycle_state.tfpa250_Addr, cycle_state.t250);
            GET_VALUE(cycle_state.tfpa350_Addr, cycle_state.t350);
            GET_VALUE(cycle_state.tfpa500_Addr, cycle_state.t500);
            blast_info("helium burned off, moving to cooling");
        }
    }
}
// during this cooling portion of the cycle, we simply update the temperatures and wait for all of the
// FPAs to drop below their maximum allowed operating temperature,
// at which point we transition back into standby mode
static void cooling_cycle(void) {
    if ( cycle_state.cooling == 1 ) {
        // we can close the pumped pot here
        cycle_state.t250_old = cycle_state.t250;
        cycle_state.t350_old = cycle_state.t350;
        cycle_state.t500_old = cycle_state.t500;
        GET_VALUE(cycle_state.tfpa250_Addr, cycle_state.t250);
        GET_VALUE(cycle_state.tfpa350_Addr, cycle_state.t350);
        GET_VALUE(cycle_state.tfpa500_Addr, cycle_state.t500);
        cycle_state.t250 = cycle_state.t250_old*(59/60) + cycle_state.t250*(1/60);
        cycle_state.t350 = cycle_state.t350_old*(59/60) + cycle_state.t350*(1/60);
        cycle_state.t500 = cycle_state.t500_old*(59/60) + cycle_state.t500*(1/60);
        if (cycle_state.t250 < cycle_state.tcrit_fpa &&
            cycle_state.t350 < cycle_state.tcrit_fpa &&
            cycle_state.t500 < cycle_state.tcrit_fpa) {
            cycle_state.standby = 1;
            cycle_state.cooling = 0;
            blast_info("Arrays are cool, standby operating mode");
        }
    }
}
// this function forces the fridge to start cycling NOW
static void forced(void) {
    cycle_state.standby = 0;
    cycle_state.cooling = 0;
    cycle_state.burning_off = 0;
    cycle_state.heating = 1;
    cycle_state.heat_delay = 0;
}
/*
standby = 1
cooling = 4
burning_off = 3
heating = 2
*/
// this function updates a channel with what our current mode is
// the key is above
static void output_cycle(void) {
    static int first_output = 1;
    uint8_t state_value;
    if (first_output == 1) {
        cycle_state.cycle_state_Addr = channels_find_by_name("cycle_state");
    }
    if (cycle_state.standby == 1) {
        state_value = 1;
    } else {
        if (cycle_state.heating == 1) {
            state_value = 2;
        } else {
            if (cycle_state.burning_off == 1) {
                state_value = 3;
            } else {
                if (cycle_state.cooling == 1) {
                    state_value = 4;
                }
            }
        }
    }
    SET_SCALED_VALUE(cycle_state.cycle_state_Addr, state_value);
}
// structure based cycle code
void auto_cycle_mk2(void) {
    static int first_time = 1;
    if (first_time == 1) {
        blast_info("initalizing");
        init_cycle_channels();
        init_cycle_values();
        first_time = 0;
        blast_info("first time done");
    }
    if (CommandData.Cryo.forced == 1) {// checks to see if we forced a cycle
        forced();
        blast_info("STARTING FRIDGE CYCLE NOW");
    }
    start_cycle();
    standby_cycle();
    heating_cycle();
    burnoff_cycle();
    cooling_cycle();
    output_cycle();
}








