/* 
 * tx_struct_tng.c: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University,  Sacramento
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston,          MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 5, 2014 by seth
 */

#include <limits.h>

#include <channels_tng.h>
#include "calibrate.h"

/* Analog channel calibrations */
/* 16-bit channels with analog preamps. To Volts */
#define CAL16(m,b) ((m)*M_16PRE),               ((b) + B_16PRE*(m)*M_16PRE)
/* bare thermomtstor. To Volts. Use LUT for temperature conversion */
#define CAL16T(m,b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T)
/* AD590 conversion. To Celsius */
#define CAL_AD590(m,b) ((m)*M_16_AD590),        ((b)+B_16_AD590*(m)*M_16_AD590-273.15)

#define U_NONE  "",           ""
#define U_T_C   "Temperature","^oC"
#define U_T_K   "Temperature", "K"
#define U_P_PSI   "Pressure", "PSI"
#define U_V_DPS "Rate",       "^o/s"
#define U_V_MPS "Speed",      "m/s"
#define U_V_KPH "Speed",      "km/hr"
#define U_ALT_M "Altitude",   "m"
#define U_P_DEG "Position",   "^o"
#define U_LA_DEG "Latitude",  "^o"
#define U_LO_DEG "Longitude","^o"
#define U_D_DEG "Direction",  "^o"
#define U_V_V "Voltage",      "V"
#define U_I_A   "Current",    "A"
#define U_T_MS  "Time",       "ms"
#define U_T_MIN "Time",       "min"
#define U_R_O   "Resistance","Ohms"
#define U_RATE "Rate",        "bps"
#define U_GB  "",             "GB"
#define U_TRIM_DEG "Trim",    "^o"
#define U_TRIM_MM "Trim",     "mm"

channel_t channel_list[] =
  {

    { "tr_he3_fridge",        CRYO_HE3_FRIDGE_M,CRYO_HE3_FRIDGE_B, TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0 },
    { "tr_m5",                CRYO_M5_M,        CRYO_M5_B, TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0 },
    { "tr_m4",                CRYO_M4_M,        CRYO_M4_B, TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0 },
    { "tr_hwpr",              CRYO_HWPR_M,      CRYO_HWPR_B, TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0 },
    { "tr_horn_500",          CRYO_HORN_500_M,  CRYO_HORN_500_B, TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0 },
    { "tr_horn_350",          CRYO_HORN_350_M,  CRYO_HORN_350_B, TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0 },
    { "tr_horn_250",          CRYO_HORN_250_M,  CRYO_HORN_250_B, TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0 },
    { "tr_300mk_strap",       CRYO_300MK_STRAP_M, CRYO_300MK_STRAP_B, TYPE_UINT32, RATE_100HZ,    SRC_IF_UEI, U_R_O, 0 },
    { "tr_he4_pot",           CRYO_HE4_POT_M,   CRYO_HE4_POT_B, TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0 },
    { "tr_optbox_filt",       CRYO_OPTBOX_FILT_M, CRYO_OPTBOX_FILT_B, TYPE_UINT32, RATE_100HZ,    SRC_IF_UEI, U_R_O, 0 },

    { "uei_if_ai201_ts",      0,                0,        TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_NONE, 0},
    { "td_vcs2_filt",         CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_3he_fridge",        CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d01_pumped_pot",    CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d07_vcs1_filt",     CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d06_250ppa",        CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d15_vcs1_hx",       CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d15_vcs2_hx",       CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d05_m3",            CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d11_vcs2",          CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d12_vcs1",          CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d04_m4",            CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d02_ob_filter",     CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d09_lhe_filter",    CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },
    { "td_d03_hwp",           CRYO_D_M,         CRYO_D_B, TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },


    { "uei_if_ts",            0,                0,          TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_NONE, 0},
    { "uei_if_2_5V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_2_5Vnic",       UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_3_3V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_3_3Vnic",       UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_24V",           UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_24Vnic",        UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_Vin",           UEI_INTVOLT_M,    UEI_INTVOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_1_5V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_1_2V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_8Vfan",         UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "uei_if_i_in",          UEI_CURRENT_M,    UEI_CURRENT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_I_A, 0 },
    { "uei_if_temp1",         UEI_TEMP_M,       UEI_TEMP_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "uei_if_temp2",         UEI_TEMP_M,       UEI_TEMP_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },

    { "uei_of_ts",            0,                0,          TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_NONE, 0},
    { "uei_of_2_5V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_2_5Vnic",       UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_3_3V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_3_3Vnic",       UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_24V",           UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_24Vnic",        UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_Vin",           UEI_INTVOLT_M,    UEI_INTVOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_1_5V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_1_2V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_8Vfan",         UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_i_in",          UEI_CURRENT_M,    UEI_CURRENT_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_temp1",         UEI_TEMP_M,       UEI_TEMP_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },
    { "uei_of_temp2",         UEI_TEMP_M,       UEI_TEMP_B, TYPE_UINT32, RATE_5HZ, SRC_OF_UEI, U_V_V, 0 },


    { "time",                 1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "time_usec",            1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "time_sip",             1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "lst",                  1.0 / 3600.0,     0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "ra_isc",               LI2H,             0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "dec_isc",              LI2DEG / 2.,      -90., TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "parts_sched",          1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "time_n_flc",           1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "framenum_isc",         1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "time_s_flc",           1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "lat",                  LI2DEG,           0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "lon",                  LI2DEG,           0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "state_isc",            1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "mcpnum_isc",           1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "ra",                   LI2H,             0.0,        TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "dec",                  LI2DEG,           0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "ra_osc",               LI2H,             0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "dec_osc",              LI2DEG / 2.,      -90., TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "framenum_osc",         1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "state_osc",            1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "mcpnum_osc",           1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "start_cycle",          1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },

    { "lst_sched",            1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "frame_sbsc",           1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "sec_sbsc",             1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "usec_sbsc",            1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },

    { "start_set_cycle",      1.0,              0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },

    { "ampl_500_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ampl_350_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ampl_250_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ampl_rox_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ampl_x_bias",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dig21_das",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },

    { "dig65_das",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "ramp_ena_bias",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ramp_ampl_bias",       1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },

    { "t_padcdc_rec",         CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_pauram_rec",         CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_hkdcdc_rec",         CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "vt_stbd_das",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_stbd_rec",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_port_das",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_port_rec",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },

    { "vt_mot_pump_val",      CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_1_prime",           CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_if_top_back",       CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_hwpr_mot",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_if_top_frnt",       CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_hwpr_feed",         CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_if_bot_frnt",       CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_strut_bot",         CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_strut_side",        CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_2_prime",           CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_if_bot_back",       CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "vt_dac_box",           CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "t_mot_act",            CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_push_plate",         CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_1_second",           CAL_AD590(1.0, 0),                TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_2_second",           CAL_AD590(1.0, 0),                TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },

    { "he4_lev",              CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0 },
    { "i_charcoal",           CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "i_cal_lamp",           CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "i_hs_char",            CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "i_hs_pot",             CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "i_300mk",              CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "i_jfet",               CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },

    { "pin_in_lock",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "fpulse_isc",           10.,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "period_cal",           .20,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "status_eth",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "timeout_n",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "az_sun",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_D_DEG, 0 },
    { "lvdt_low_act",         1.0,              -5000.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "status_mcc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, //south_i_am, at_float, schedule, slot_sched
    { "cryostate",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "upslot_sched",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "t_chip_flc",           0.01,             0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "veto_sensor",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "level_on_bal",         1. / 1990.13,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_I_A, 0 },
    { "level_off_bal",        1. / 1990.13,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_I_A, 0 },
    { "level_target_bal",     1. / 1990.13,     -5.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_I_A, 0 },

    { "alt_sip",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "mapmean_isc",          1.,               0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "lat_sip",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_LA_DEG, 0 },
    { "lon_sip",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_LO_DEG, 0 },

    { "df_n_flc",             1. / 250,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_GB, 0 },
    { "mode_p",               1, 0.0,           TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "x_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "y_p",                  I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "vel_az_p",             I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "del_p",                I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "blob_idx_isc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_x_isc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_y_isc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_f_isc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_s_isc",         1000. / 65536.,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_x_isc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_y_isc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_f_isc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_s_isc",         1000. / 65536.,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_x_isc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_y_isc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_f_isc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_s_isc",         1000. / 65536.,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "w_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // pointing scan width
    { "rtol_isc",             I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "apert_isc",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "maglimit_isc",         1. / 1000.,       0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "nrad_isc",             I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mtol_isc",             100. / 65536.,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "qtol_isc",             100. / 65536.,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "lrad_isc",             I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "thresh_isc",           1. / 10.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "grid_isc",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "real_trig_osc",        1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mdist_isc",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "nblobs_isc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "delay_isc",            1.0 / 1000.0,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "delay_osc",            1.0 / 1000.0,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "foc_off_osc",          1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "tol_isc",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "ok_pss",               1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "offset_ifel_gy",       1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0 },
    { "offset_ifroll_gy",     1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0 },
    { "offset_ifyaw_gy",      1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0 },

    { "lvdt_high_act",        1.0,              -5000.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "az_isc",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "el_isc",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "sigma_isc",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pulse_cal",            10.0,             0.,                TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "sigma_pss",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "sigma_clin",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "spulse_isc",           10.0,             0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "hx_flag_isc",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "brra_isc",             I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "brdec_isc",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "x_off_isc",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "gain_osc",             100. / 65536.,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_hold_isc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "save_prd_isc",         0.01,             0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "y_off_isc",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "offset_isc",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "bbc_fifo_size",        1. / 624,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "t_cpu_n_flc",          0.01,             0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_mb_flc",             0.01,             0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "mks_hi_sip",           0.003256,         -0.226858, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mks_med_sip",          0.032614,         -0.072580, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob_idx_osc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_x_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_y_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_f_osc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_s_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_x_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_y_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_f_osc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_s_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_x_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_y_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_f_osc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_s_osc",         1. / 40.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mapmean_osc",          1.,               0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "fpulse_osc",           10.,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, //apparently not used

    { "az_osc",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "el_osc",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "sigma_osc",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "tol_osc",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "apert_osc",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "maglimit_osc",         1. / 1000.,       0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "nrad_osc",             I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mtol_osc",             100. / 65536.,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "qtol_osc",             100. / 65536.,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "offset_osc",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "lrad_osc",             I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "thresh_osc",           1. / 10.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "grid_osc",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "real_trig_isc",        1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "foc_off_isc",          1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mdist_osc",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "nblobs_osc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "rtol_osc",             I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "rd_sigma_osc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "spulse_osc",           10.0,             0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "hx_flag_osc",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "brra_osc",             I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "brdec_osc",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "x_off_osc",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_hold_osc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "save_prd_osc",         0.01,             0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "y_off_osc",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "pref_tp_sf",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "maxblobs_isc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "maxblobs_osc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "bi0_fifo_size",        1. / 624,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "plover",               1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "t_flange_isc",         1. / 100.,        -273.15, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_lens_isc",           1. / 100.,        -273.15, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_heat_isc",           1. / 100.,        -273.15, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_comp_isc",           1. / 100.,        -273.15, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "pressure1_isc",        1. / 2000.,       0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_PSI, 0 },
    { "t_flange_osc",         1. / 100.,        -273.15, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_lens_osc",           1. / 100.,        -273.15, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_heat_osc",           1. / 100.,        -273.15, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_comp_osc",           1. / 100.,        -273.15, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "pressure1_osc",        1. / 2000.,       0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_PSI, 0 },
    { "gain_isc",             100. / 65536.,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "jfet_set_on",          1 / 100.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "jfet_set_off",         1 / 100.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "state_cycle",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "trig_type_isc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "exposure_isc",         100.,             0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "trig_type_osc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "exposure_osc",         100.,             0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "fieldrot_isc",         I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "fieldrot_osc",         I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_p_heat_gy",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_i_heat_gy",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_d_heat_gy",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "t_set_gy",             (100.0 / 32768.0),0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "h_age_gy",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "h_hist_gy",            (100.0 / 32768.0),0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "ra_1_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // pointing mode coordinates
    { "dec_1_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ra_2_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dec_2_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ra_3_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dec_3_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ra_4_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dec_4_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "trim_clin",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "trim_enc",             I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "trim_null",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "trim_mag",             I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "gain_bal",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "el_clin",              I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "h_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // scan height

    { "error_isc",            1.,               0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "el_lut_clin",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "rd_sigma_isc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mks_lo_sip",           0.327045,         -5.944902, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "error_osc",            1.,               0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "alt",                  1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mode_az_mc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mode_el_mc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dest_az_mc",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dest_el_mc",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "vel_az_mc",            1. / 6000,        0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "vel_el_mc",            1. / 6000,        0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dir_az_mc",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dir_el_mc",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "slew_veto",            4.0 / SR,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "sveto_len",            4.0 / SR,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "pot_lock",             -100.0 / 16068.0, 1636800.0 / 16068.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dith_el",              0.5 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_D_DEG, 0 },
    { "state_lock",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "goal_lock",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "seized_act",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "n_dith_p",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "x_vel_stage",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // not used in flight...
    { "x_stp_stage",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "x_str_stage",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "y_lim_stage",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "i_dith_el",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "y_stp_stage",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "y_str_stage",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "x_lim_stage",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "y_vel_stage",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "he4_lev_old",          CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "focus_isc",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "focus_osc",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "diskfree_isc",         5.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "diskfree_osc",         5.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "off_ifel_gy_isc",      1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "off_ifel_gy_osc",      1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "off_ifroll_gy_isc",    1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "off_ifroll_gy_osc",    1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "off_ifyaw_gy_isc",     1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "off_ifyaw_gy_osc",     1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "pos_lock",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos_0_act",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos_1_act",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos_2_act",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "enc_0_act",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "enc_1_act",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "enc_2_act",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "goal_sf",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "focus_sf",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "maxslew_isc",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "maxslew_osc",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "offset_ifrollmag_gy",  1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0 },
    { "offset_ifyawmag_gy",   1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0 },
    { "offset_ifrollpss_gy",  1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0 },
    { "offset_ifyawpss_gy",   1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0 },
    { "next_i_hwpr_p",        1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "next_i_dith_p",        1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "cal_off_pss1",         40.0 / 65536.0,   0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_DEG, 0 },
    { "cal_off_pss2",         40.0 / 65536.0,   0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_DEG, 0 },
    { "cal_off_pss3",         40.0 / 65536.0,   0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_DEG, 0 },
    { "cal_off_pss4",         40.0 / 65536.0,   0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_DEG, 0 },
    { "cal_d_pss1",           4.0 / 65536.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_MM, 0 },
    { "cal_d_pss2",           4.0 / 65536.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_MM, 0 },
    { "cal_d_pss3",           4.0 / 65536.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_MM, 0 },
    { "cal_d_pss4",           4.0 / 65536.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_MM, 0 },
    { "cal_imin_pss",         40.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0 },

    { "pref_ts_sf",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "spread_sf",            1 / 500.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "acc_lock",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_move_act",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "i_hold_act",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "vel_act",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "acc_act",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_move_lock",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "i_hold_lock",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "vel_lock",             100.,             0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_prime_sf",           0.01,             0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_second_sf",          0.01,             0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_sf",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "wait_sf",              1 / 30.,          0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mode_sf",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "correction_sf",        1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "age_sf",               1 / 30.,          0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "offset_sf",            1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "t_prime_sf",           CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_second_sf",          CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "flags_act",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "lvdt_spread_act",      1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "el_sun",               I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "hwpr_cal",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mode_cal",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "minblobs_isc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "minblobs_osc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "max_age_isc",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MS, 0 },
    { "max_age_osc",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MS, 0 },
    { "age_isc",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MS, 0 },
    { "age_osc",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MS, 0 },
    { "overshoot_hwpr",       1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "drive_info_rw",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "drive_err_cts_rw",     1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "drive_info_el",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "drive_err_cts_el",     1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "drive_info_piv",       1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "drive_err_cts_piv",    1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "vel_hwpr",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "acc_hwpr",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_move_hwpr",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "i_hold_hwpr",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "pos_hwpr",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "enc_hwpr",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mode_bal",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_start_bias",      0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_end_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_n_bias",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_time_bias",       1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_pul_len_bias",    1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_array_bias",      1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_start_phase",     0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_end_phase",       0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_nsteps_phase",    1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_time_phase",      1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_ena_bias",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "step_ena_phase",       1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },



    { "v_batt_cc1",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0 },/* charge controller related channels */
    { "v_arr_cc1",            1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0 },
    { "i_batt_cc1",           1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0 },
    { "i_arr_cc1",            1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0 },
    { "t_hs_cc1",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "fault_cc1",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },  // fault bitfield
    { "alarm_hi_cc1",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // alarm high bitfield
    { "alarm_lo_cc1",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // alarm low bitfield
    { "v_targ_cc1",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0 },
    { "state_cc1",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },


    { "lvdt_0_act",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },/* filtered LVDTs, rotated to motor positions */
    { "lvdt_1_act",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "lvdt_2_act",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "az_gy",                1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_V_DPS, 0 },
    { "offset_0_act",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "offset_1_act",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "offset_2_act",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "goal_0_act",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "goal_1_act",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "goal_2_act",           1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "az_raw_pss",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "el_raw_pss",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "snr_pss1",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "snr_pss2",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "snr_pss3",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "snr_pss4",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "az_pss",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "trim_pss",             I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "accel_az",             2.0 / 65536,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos_focus_isc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos_focus_osc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "az_raw_pss1",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "az_raw_pss2",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "az_raw_pss3",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "led_cc1",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // charge controller LED state
    { "force_sbsc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "exp_int_sbsc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MS, 0 },
    { "exp_time_sbsc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MS, 0 },
    { "foc_res_sbsc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "move_tol_sbsc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "maxblob_sbsc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "grid_sbsc",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "thresh_sbsc",          1.0 / 1000.0,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mdist_sbsc",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mapmean_sbsc",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mapsigma_sbsc",        1.0 / 10.0,       0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "ccd_t_sbsc",           1.0 / 100.0,      0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "nblobs_sbsc",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_x_sbsc",        CAM_WIDTH / SHRT_MAX,                0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_y_sbsc",        CAM_WIDTH / SHRT_MAX,                0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_f_sbsc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob00_s_sbsc",        1.0 / 100.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_x_sbsc",        CAM_WIDTH / SHRT_MAX,                0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_y_sbsc",        CAM_WIDTH / SHRT_MAX,                0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_f_sbsc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob01_s_sbsc",        1.0 / 100.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_x_sbsc",        CAM_WIDTH / SHRT_MAX,                0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_y_sbsc",        CAM_WIDTH / SHRT_MAX,                0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_f_sbsc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob02_s_sbsc",        1.0 / 100.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_pt_az",              1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_pt_el",              1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos0_hwpr",            1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos1_hwpr",            1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos2_hwpr",            1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos3_hwpr",            1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_pos_rq_hwpr",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "rate_tdrss",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_RATE, 0 },
    { "rate_iridium",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_RATE, 0 },
    { "read_wait_hwpr",       1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_pos_hwpr",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "stop_cnt_hwpr",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "rel_move_hwpr",        2.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "stat_control_hwpr",    1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pot_targ_hwpr",        1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "mode_act",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "enc_targ_hwpr",        1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "enc_err_hwpr",         1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dr_0_act",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dr_1_act",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dr_2_act",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "tol_act",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "status_actbus",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pot_err_hwpr",         1.0 / 32767.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "blob_idx_sbsc",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "az_raw_pss4",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "el_raw_pss1",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "el_raw_pss2",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "el_raw_pss3",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },
    { "el_raw_pss4",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0 },

    { "el_raw_enc",           I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_P_DEG, 0 },
    { "el_enc",               I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_P_DEG, 0 },
    { "sigma_enc",            I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "el_motor_enc",         I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_P_DEG, 0 },
    { "sigma_motor_enc",      I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "chatter",              1.0,              0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0 },

    { "focpos_sbsc",          1.0 / 10.0,       0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "delay_sbsc",           1.0 / 1000.0,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "daz_p",                I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "steps_shutter",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "steps_slow_shutter",   1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "pos_shutter",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "ra_sbsc",              1.0 / 1000.0,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dec_sbsc",             1.0 / 1000.0,     0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "i_tot",                1.0e-3,           0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0 }, // sum of currents read through ACS1 A1
    { "t_set_sbsc",           (100.0 / 32768.0),0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },

    { "pot_hwpr",             1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "last_n_cmd",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "count_n_cmd",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "last_s_cmd",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "count_s_cmd",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "df_s_flc",             1.0 / 250.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_GB, 0 },
    { "timeout_s",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "t_cpu_s_flc",          0.01,             0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "t_start_cycle",        4.0 / 65536.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_K, 0 },
    { "t_pot_max_cycle",      10.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_K, 0 },
    { "t_char_max_cycle",     70.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_K, 0 },
    { "t_char_set_cycle",     70.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_K, 0 },
    { "time_char_cycle",      120.0 / 65536.0,  0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MIN, 0 },
    { "time_set_cycle",       120.0 / 65536.0,  0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MIN, 0 },
    { "thresh_atrim",         10.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "time_atrim",           1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "rate_atrim",           30.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "v_batt_cc2",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0 },
    { "v_arr_cc2",            1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0 },
    { "i_batt_cc2",           1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0 },
    { "i_arr_cc2",            1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0 },
    { "t_hs_cc2",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "fault_cc2",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },  // fault bitfield
    { "alarm_hi_cc2",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // alarm high bitfield
    { "alarm_lo_cc2",         1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // alarm low bitfield
    { "v_targ_cc2",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0 },
    { "state_cc2",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "led_cc2",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 }, // charge controller LED state

    #ifndef BOLOTEST

    { "latch0",               1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "latch1",               1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "switch_gy",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "switch_misc",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "bus_reset_act",        1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "v1_3_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v2_3_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v3_3_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v4_3_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v1_4_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v2_4_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v3_4_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v4_4_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },

    { "v_pump_bal",           3.91 / 13107.0,   -9.775, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "dac2_ampl",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "mask_gy",              1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "bits_vtx",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "bits_bal",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "trig_s_sbsc",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "trig_l_sbsc",          1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },

    { "pch_pyr_clin",         0.001343,         -45.426, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "roll_pyr_clin",        0.001413,         -45.398, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "t_pyr_clin",           100.0 * 10.0 / 32768.0, -100.0 * 10.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },
    { "xel_if_clin",          0.00546739,       -25. * 6.144, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "el_raw_if_clin",       1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "t_if_clin",            100.0 * 10.0 / 32768.0, -100.0 * 10.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0 },

    { "x_stage",              2.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "y_stage",              2.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "ifpm_hall",            1.0,              0.0,            TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "lvdt_65_act",          LVDT65_ADC_TO_ENC,LVDT65_ZERO,    TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "lvdt_63_act",          LVDT63_ADC_TO_ENC,LVDT63_ZERO,    TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "lvdt_64_act",          LVDT64_ADC_TO_ENC,LVDT64_ZERO,    TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "sbsc_trig",            1.0,              0.0,            TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0 },
    { "v1_1_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v2_1_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v3_1_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v4_1_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v1_2_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v2_2_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v3_2_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },
    { "v4_2_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0 },

    #endif

    { "ifyaw_1_gy",           1.0,      0.0,            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0 },
    { "ifroll_1_gy",          1.0,      0.0,            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0 },
    { "ifyaw_2_gy",           1.0,      0.0,            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0 },
    { "ifel_1_gy",            1.0,      0.0,            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0 },
    { "ifel_2_gy",            1.0,      0.0,            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0 },
    { "ifroll_2_gy",          1.0,      0.0,            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0 },
    { "az",                   LI2DEG,   0.0,            TYPE_UINT32, RATE_200HZ, SRC_FC, U_P_DEG, 0 },
    { "el",                   LI2DEG,   0.0,            TYPE_UINT32, RATE_200HZ, SRC_FC, U_P_DEG, 0 },

    { "heat_gy",              1.0,      0.0,            TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "uei_if_framenum",      1.0,      0.0,            TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_NONE, 0 },
    { "uei_of_framenum",      1.0,      0.0,            TYPE_UINT16, RATE_100HZ, SRC_OF_UEI, U_NONE, 0 },

    { "ifel_gy",              1.0,      0.0,            TYPE_FLOAT, RATE_100HZ, SRC_FC, U_V_DPS, 0 },
    { "ifroll_gy",            1.0,      0.0,            TYPE_FLOAT, RATE_100HZ, SRC_FC, U_V_DPS, 0 },
    { "ifyaw_gy",             1.0,      0.0,            TYPE_FLOAT, RATE_100HZ, SRC_FC, U_V_DPS, 0 },

    { "vel_req_el",           1.0,      0.0,            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0 },
    { "vel_req_az",           1.0,      0.0,            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0 },

    { "trigger_isc",          1.0,      0.0,            TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "trigger_osc",          1.0,      0.0,            TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "pulse_sc",             1.0,      0.0,            TYPE_UINT8, RATE_200HZ, SRC_FC, U_NONE, 0 },

    { "dig43_das",            1.0,      0.0,            TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "chopper",              CAL16(1.0, 0.0),          TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0 },

    { "x_mag",                1.0,              0,                 TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_NONE, 0 },
    { "y_mag",                1.0,              0,                 TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_NONE, 0 },
    { "z_mag",                1.0,              0,                 TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_NONE, 0 },

    { "az_mag",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_D_DEG, 0 },
    { "az_raw_mag",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_D_DEG, 0 },
    { "pitch_mag",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "sigma_mag",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "declination_mag",      I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_D_DEG, 0 }, // magnetic declination

    { "cal_xmax_mag",         1, 0.0,           TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "cal_xmin_mag",         1, 0.0,           TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "cal_ymax_mag",         1, 0.0,           TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "cal_ymin_mag",         1, 0.0,           TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },


    /** Motor Channels */
    { "mc_rw_vel",               RW_ENCODER_SCALING * 0.1,  0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_V_DPS, 0 },
    { "mc_rw_pos",               RW_ENCODER_SCALING,        0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_D_DEG, 0 },
    { "mc_el_vel",               EL_MOTOR_ENCODER_SCALING * 0.1,  0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_V_DPS, 0 },
    { "mc_el_pos",               EL_LOAD_ENCODER_SCALING,   0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_D_DEG, 0 },
    { "mc_el_motor_pos",         EL_MOTOR_ENCODER_SCALING,  0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_D_DEG, 0 },
    { "mc_piv_vel",              PIV_RESOLVER_SCALING * 0.1,0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_V_DPS, 0 },
    { "mc_piv_pos",              PIV_RESOLVER_SCALING,      0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_D_DEG, 0 },

    { "mc_el_biss_status",       1.0,  0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0 },

    {"control_word_read_el",     1.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0},
    {"control_word_read_rw",     1.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0},
    {"control_word_read_piv",     1.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0},
    {"latched_fault_el",         1.0,    0.0, TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0},
    {"latched_fault_rw",         1.0,    0.0, TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0},
    {"latched_fault_piv",         1.0,    0.0, TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0},
    {"network_status_el",        1.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0},
    {"network_status_rw",        1.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0},
    {"network_status_piv",        1.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0},

    ///TODO: Ensure that scale factors for all currents/commands are in Amps
    { "mc_piv_i_cmd",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0 },
    { "mc_rw_i_cmd",            1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0 },
    { "mc_el_i_cmd",            1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0 },

    { "mc_piv_i_read",          1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0 },
    { "mc_rw_i_read",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0 },
    { "mc_el_i_read",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0 },

    /** Velocity control loop commanded P/I terms */
    { "g_p_el",               1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_i_el",               1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_d_el",               1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_db_el",              1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_p_az",               1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_i_az",               1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_d_az",               1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_db_az",              1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_pe_piv",             1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_pv_piv",             1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "g_iv_piv",             1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "set_rw",               1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_V_DPS, 0 },

    { "fault_gy",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "vel_el_p",             I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "frict_off_piv",        1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "frict_term_piv",       1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "frict_term_uf_piv",    1.0,              0.0, TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 }, // For debugging remove later

    /** Motor Controller State and Status Registers */
    { "status_rw",            1.0,              0.0, TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "state_rw",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "status_el",            1.0,              0.0, TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "state_el",             1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "status_piv",           1.0,              0.0, TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "state_piv",            1.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0 },

    /** Motor Controller Temperatures */
    { "t_mc_rw",              1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_T_C, 0 },
    { "t_mc_el",              1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_T_C, 0 },
    { "t_mc_piv",             1.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_T_C, 0 },

    /** Calculated P/I and Error (diff btw commanded/actual velocity) terms from control loop */
    { "p_term_el",            1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_term_el",            1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "d_term_el",            1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "error_el",             1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "el_integral_step",     1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "p_term_az",            1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_term_az",            1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "d_term_az",            1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "error_az",             1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "az_integral_step",     1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "p_rw_term_piv",        1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "i_rw_term_piv",        1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },
    { "p_err_term_piv",       1.0,              0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0 },

    { "mcp_1hz_framecount",     1.0,    0.0,        TYPE_UINT32,    RATE_1HZ,   SRC_FC, U_NONE, 0 },
    { "mcp_5hz_framecount",     1.0,    0.0,        TYPE_UINT32,    RATE_5HZ,   SRC_FC, U_NONE, 0 },
    { "mcp_100hz_framecount",   1.0,    0.0,        TYPE_UINT32,    RATE_100HZ, SRC_FC, U_NONE, 0 },
    { "mcp_200hz_framecount",   1.0,    0.0,        TYPE_UINT32,    RATE_200HZ, SRC_FC, U_NONE, 0 },
    { {0} }
  };

