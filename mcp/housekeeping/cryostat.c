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

/* Heater control bits (BIAS_D G4) */
#define HEAT_HELIUM_LEVEL    0x0001
#define HEAT_CHARCOAL        0x0002
#define HEAT_POT_HS          0x0004
#define HEAT_CHARCOAL_HS     0x0008
#define HEAT_UNDEF           0x0010
#define HEAT_BDA             0x0020
#define HEAT_CALIBRATOR      0x0040
#define HEAT_HWPR_POS        0x0080

static uint16_t heatctrl;

/*************************************************************************/
/* CryoControl: Control valves, heaters, and calibrator (a fast control) */
/*************************************************************************/
void cryo_control(void)
{
    heatctrl = 0;
    if (CommandData.Cryo.charcoalHeater)
        heatctrl |= HEAT_CHARCOAL;
}

void store_100hz_cryo(void)
{
    static int firsttime = 1;

    static channel_t* heaterAddr;

    if (firsttime) {
        heaterAddr = channels_find_by_name("dio_heaters");
        firsttime = 0;
    }
    SET_UINT16(heaterAddr, heatctrl);
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
    static channel_t* diode_charcoal_hs_Addr; // provisional names until we figure out where all thermometers are
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
    if (firsttime_therm == 1) {
        rox_fpa_1k_Addr = channels_find_by_name("tr_fpa_1k");
        rox_250_fpa_Addr = channels_find_by_name("tr_250_fpa");
        rox_1k_plate_Addr = channels_find_by_name("tr_1k_plate");
        rox_300mk_strap_Addr = channels_find_by_name("tr_300mk_strap");
        rox_350_fpa_Addr = channels_find_by_name("tr_350_fpa");
        rox_he4_pot_Addr = channels_find_by_name("tr_he4_pot");
        rox_he3_fridge_Addr = channels_find_by_name("tr_he3_fridge");
        rox_500_fpa_Addr = channels_find_by_name("tr_500_fpa");
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
        firsttime_therm = 0;
    }

    // these labjack channels need to all be changed once set up
    SET_SCALED_VALUE(diode_charcoal_hs_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_vcs2_filt_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_250fpa_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_hwp_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_vcs1_hx_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_1k_fridge_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_4k_plate_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_vcs1_filt_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_m3_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_charcoal_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_ob_filter_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_vcs2_plate_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_m4_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_4k_filt_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_vcs2_hx_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(diode_vcs1_plate_Addr, labjack_get_value(LABJACK_CRYO, 0));
    // above are the diodes, below, the ROXes
    SET_SCALED_VALUE(rox_fpa_1k_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(rox_250_fpa_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(rox_1k_plate_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(rox_300mk_strap_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(rox_350_fpa_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(rox_he4_pot_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(rox_he3_fridge_Addr, labjack_get_value(LABJACK_CRYO, 0));
    SET_SCALED_VALUE(rox_500_fpa_Addr, labjack_get_value(LABJACK_CRYO, 0));
}
