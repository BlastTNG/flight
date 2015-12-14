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

#include "channels_tng.h"
#include "calibrate.h"
#include "conversions.h"

/* Analog channel calibrations */
/* 16-bit channels with analog preamps. To Volts */
#define CAL16(m, b) ((m)*M_16PRE),               ((b) + B_16PRE*(m)*M_16PRE)
/* bare thermistor. To Volts. Use LUT for temperature conversion */
#define CAL16T(m, b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T)
/* AD590 conversion. To Celsius */
#define CAL_AD590(m, b) ((m)*M_16_AD590),        ((b)+B_16_AD590*(m)*M_16_AD590-273.15)

#define U_NONE  "",           ""
#define U_T_C   "Temperature", "^oC"
#define U_T_K   "Temperature", "K"
#define U_P_PSI   "Pressure", "PSI"
#define U_V_DPS "Rate",       "^o/s"
#define U_V_MPS "Speed",      "m/s"
#define U_V_KPH "Speed",      "km/hr"
#define U_ALT_M "Altitude",   "m"
#define U_P_DEG "Position",   "^o"
#define U_LA_DEG "Latitude",  "^o"
#define U_LO_DEG "Longitude", "^o"
#define U_D_DEG "Direction",  "^o"
#define U_V_V "Voltage",      "V"
#define U_I_A   "Current",    "A"
#define U_T_MS  "Time",       "ms"
#define U_T_S  "Time",       "s"
#define U_T_MIN "Time",       "min"
#define U_R_O   "Resistance", "Ohms"
#define U_RATE "Rate",        "bps"
#define U_GB  "",             "GB"
#define U_TRIM_DEG "Trim",    "^o"
#define U_TRIM_MM "Trim",     "mm"

#define SCALE(_type)  _type ## _M, _type ## _B
// TODO(seth): Unify the _M, _B scale factor offset terms in a single location
channel_t channel_list[] =
  {
    { "tr_he3_fridge",   SCALE(CRYO_HE3_FRIDGE), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_m5",           SCALE(CRYO_M5), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_m4",           SCALE(CRYO_M4), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_hwpr",         SCALE(CRYO_HWPR), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_horn_500",     SCALE(CRYO_HORN_500), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_horn_350",     SCALE(CRYO_HORN_350), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_horn_250",     SCALE(CRYO_HORN_250), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_300mk_strap",  SCALE(CRYO_300MK_STRAP), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_he4_pot",      SCALE(CRYO_HE4_POT), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },
    { "tr_optbox_filt",  SCALE(CRYO_OPTBOX_FILT), TYPE_UINT32, RATE_100HZ, SRC_IF_UEI, U_R_O, 0, 0, 0 },

    { "td_vcs2_filt",     SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 10, 0 },
    { "td_3he_fridge",    SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 8, 0 },
    { "td_pumped_pot",    SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 1, 0 },
    { "td_vcs1_filt",     SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 7, 0 },
    { "td_250ppa",        SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 6, 0 },
    { "td_vcs1_hx",       SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 13, 0 },
    { "td_vcs2_hx",       SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 16, 0 },
    { "td_m3",            SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 5, 0 },
    { "td_vcs2",          SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 11, 0 },
    { "td_vcs1",          SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 12, 0 },
    { "td_m4",            SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 4, 0 },
    { "td_ob_filter",     SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 2, 0 },
    { "td_lhe_filter",    SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 9, 0 },
    { "td_hwp",           SCALE(CRYO_D), TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 0, 3, 0 },

    { "225_ch0",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 0, 0 },
    { "225_ch1",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 1, 0 },
    { "225_ch2",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 2, 0 },
    { "225_ch3",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 3, 0 },
    { "225_ch4",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 4, 0 },
    { "225_ch5",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 5, 0 },
    { "225_ch6",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 6, 0 },
    { "225_ch7",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 7, 0 },
    { "225_ch8",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 8, 0 },
    { "225_ch9",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 9, 0 },
    { "225_ch24",          1.25 / UINT32_MAX,  -1.25, TYPE_UINT32, RATE_100HZ, SRC_OF_UEI, U_V_V, 2, 24, 0 },

    { "uei_if_ts",            0,                0,          TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 255, 0 },
    { "uei_if_2_5V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 14, 0, 0 },
    { "uei_if_3_3V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 14, 0, 0 },
    { "uei_if_24V",           UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 14, 0, 0 },
    { "uei_if_Vin",           UEI_INTVOLT_M,    UEI_INTVOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 14, 0, 0 },
    { "uei_if_1_5V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 14, 0, 0 },
    { "uei_if_1_2V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_V_V, 14, 0, 0 },
    { "uei_if_i_in",          UEI_CURRENT_M,    UEI_CURRENT_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_I_A, 14, 0, 0 },
    { "uei_if_temp1",         UEI_TEMP_M,       UEI_TEMP_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_T_C, 14, 0, 0 },
    { "uei_if_temp2",         UEI_TEMP_M,       UEI_TEMP_B, TYPE_UINT32, RATE_5HZ, SRC_IF_UEI, U_T_C, 14, 0, 0 },

    { "uei_of_ts",            1.0,              0,          TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_NONE, 0, 255, 0 },
    { "uei_of_2_5V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },
    { "uei_of_3_3V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },
    { "uei_of_24V",           UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },
    { "uei_of_Vin",           UEI_INTVOLT_M,    UEI_INTVOLT_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },
    { "uei_of_1_5V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },
    { "uei_of_1_2V",          UEI_VOLT_M,       UEI_VOLT_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },
    { "uei_of_i_in",          UEI_CURRENT_M,    UEI_CURRENT_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },
    { "uei_of_temp1",         UEI_TEMP_M,       UEI_TEMP_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },
    { "uei_of_temp2",         UEI_TEMP_M,       UEI_TEMP_B, TYPE_UINT32, RATE_1HZ, SRC_OF_UEI, U_V_V, 14, 0, 0 },

    {"x0_last_trig_motion_caz_px", SCALE(CONVERT_VEL), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_last_trig_motion_el_px", SCALE(CONVERT_VEL), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_last_trig_motion_px", SCALE(CONVERT_VEL), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_point_az_raw", SCALE(CONVERT_ANGLE_DEG), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_point_az", SCALE(CONVERT_ANGLE_DEG), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_point_el_raw", SCALE(CONVERT_ANGLE_DEG), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_point_el", SCALE(CONVERT_ANGLE_DEG), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_point_sigma", SCALE(CONVERT_ANGLE_DEG), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_point_az_trim", SCALE(CONVERT_ANGLE), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_point_el_trim", SCALE(CONVERT_ANGLE), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_cd_robust_mode", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_cd_motion_psf", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_last_trig_motion_caz_px", CONVERT_VEL_M, CONVERT_VEL_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_last_trig_motion_el_px", CONVERT_VEL_M, CONVERT_VEL_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_last_trig_motion_px", CONVERT_VEL_M, CONVERT_VEL_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_point_az_raw", CONVERT_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_point_az", CONVERT_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_point_el_raw", CONVERT_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_point_el", CONVERT_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_point_sigma", CONVERT_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_point_az_trim", CONVERT_ANGLE_M, CONVERT_ANGLE_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_point_el_trim", CONVERT_ANGLE_M, CONVERT_ANGLE_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_cd_robust_mode", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_cd_motion_psf", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_heater", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_heater", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x0_ctr_mcp", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_last_trig_age_cs", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_last_trig_ctr_mcp", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_predicted_motion_px", CONVERT_VEL_M, CONVERT_VEL_B, TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_last_trig_ctr_stars", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_blobn_x", CONVERT_BLOB_POS_M, CONVERT_BLOB_POS_B, TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_blobn_y", CONVERT_BLOB_POS_M, CONVERT_BLOB_POS_B, TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_blobn_flux", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_blobn_peak_to_flux", SCALE(CONVERT_0_TO_10), TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_last_trig_ctr_stars", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_blobn_x", CONVERT_BLOB_POS_M, CONVERT_BLOB_POS_B, TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_blobn_y", CONVERT_BLOB_POS_M, CONVERT_BLOB_POS_B, TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_blobn_flux", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_blobn_peak_to_flux", SCALE(CONVERT_0_TO_10), TYPE_UINT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},


    {"x0_last_trig_lat", DEG2LI, 0.0, TYPE_UINT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_last_trig_lst", SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x0_image_num_blobs",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x0_ctr_stars",               SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_ctr_stars",         SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_ctr_mcp",           SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x0_hk_temp_lens",    CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0},
    {"x0_hk_temp_comp",    CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0},
    {"x0_hk_temp_plate",   CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0},
    {"x0_hk_temp_flange",  CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0},
    {"x0_hk_pressure",     CONVERT_PRES_M,  CONVERT_PRES_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_P_PSI, 0, 0, 0},
    {"x0_hk_disk",         CONVERT_GB_M,    CONVERT_GB_B,    TYPE_UINT16, RATE_1HZ, SRC_FC, U_GB, 0, 0, 0},

    {"x0_image_eq_valid",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_cam_gain_valid",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_valid",         SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_afocus_metric_valid", SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x0_stars_run_time",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_T_S, 0, 0, 0},
    {"x0_cam_gain_db",             SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_lens_focus",              SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_lens_aperture",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x0_image_num_exposures",     SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_stats_mean",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_stats_noise",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_stats_gaindb",      128.0/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_stats_num_px_sat",  SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_stats_frac_px_sat", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_afocus_metric",     SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x0_image_eq_iplate",         10.0/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_iplate",        10.0/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x0_image_eq_ra",             SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_eq_dec",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_eq_roll",           SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_eq_sigma_ra",       SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_eq_sigma_dec",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_eq_sigma_roll",     SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_eq_sigma_pointing", SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_az",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_el",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_roll",          SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_sigma_az",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_sigma_el",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_sigma_roll",    SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x0_image_hor_sigma_pointing", SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_image_num_blobs",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_ctr_stars",               SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_ctr_stars",         SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_ctr_mcp",           SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_hk_temp_lens",    CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0},
    {"x1_hk_temp_comp",    CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0},
    {"x1_hk_temp_plate",   CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0},
    {"x1_hk_temp_flange",  CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0},
    {"x1_hk_pressure",     CONVERT_PRES_M,     CONVERT_PRES_B,     TYPE_UINT16, RATE_1HZ, SRC_FC, U_P_PSI, 0, 0, 0},
    {"x1_hk_disk",         CONVERT_GB_M, CONVERT_GB_B, TYPE_UINT16, RATE_1HZ, SRC_FC, U_GB, 0, 0, 0},

    {"x1_image_eq_valid",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_cam_gain_valid",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_valid",         SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_afocus_metric_valid", SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_stars_run_time",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_T_S, 0, 0, 0},
    {"x1_cam_gain_db",             SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_lens_focus",              SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_lens_aperture",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_image_num_exposures",     SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_stats_mean",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_stats_noise",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_stats_gaindb",      128.0/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_stats_num_px_sat",  SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_stats_frac_px_sat", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_afocus_metric",     SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_image_eq_iplate",         10.0/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_iplate",        10.0/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},

    {"x1_image_eq_ra",             SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_eq_dec",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_eq_roll",           SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_eq_sigma_ra",       SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_eq_sigma_dec",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_eq_sigma_roll",     SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_eq_sigma_pointing", SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_az",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_el",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_roll",          SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_sigma_az",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_sigma_el",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_sigma_roll",    SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},
    {"x1_image_hor_sigma_pointing", SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, SRC_FC, U_NONE, 0, 0, 0},


    { "time",                 SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "time_usec",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "time_sip",             SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "lst",                  1.0 / 3600.0,     0.0, TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "parts_sched",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "lat",                  LI2DEG,           0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "lon",                  LI2DEG,           0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "ra",                   LI2H,             0.0,        TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dec",                  LI2DEG,           0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "start_cycle",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "lst_sched",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "start_set_cycle",      SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "ampl_500_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ampl_350_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ampl_250_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ampl_rox_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ampl_x_bias",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dig21_das",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },

    { "dig65_das",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "ramp_ena_bias",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ramp_ampl_bias",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },

    { "t_padcdc_rec",         CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "t_pauram_rec",         CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "t_hkdcdc_rec",         CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "vt_stbd_das",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_stbd_rec",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_port_das",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_port_rec",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },

    { "vt_mot_pump_val",      CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_1_prime",           CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_if_top_back",       CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_hwpr_mot",          CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_if_top_frnt",       CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_hwpr_feed",         CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_if_bot_frnt",       CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_strut_bot",         CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_strut_side",        CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_2_prime",           CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_if_bot_back",       CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "vt_dac_box",           CAL16T(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "t_mot_act",            CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "t_push_plate",         CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "t_1_second",           CAL_AD590(1.0, 0),                TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "t_2_second",           CAL_AD590(1.0, 0),                TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },

    { "he4_lev",              CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "i_charcoal",           CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "i_cal_lamp",           CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "i_hs_char",            CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "i_hs_pot",             CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "i_300mk",              CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "i_jfet",               CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },

    { "pin_in_lock",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "period_cal",           .20,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "status_eth",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "az_sun",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_D_DEG, 0, 0, 0 },
    { "lvdt_low_act",         1.0,              -5000.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "status_mcc",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "cryostate",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "upslot_sched",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "t_chip_flc",           0.01,             0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "veto_sensor",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "level_on_bal",         1. / 1990.13,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_I_A, 0, 0, 0 },
    { "level_off_bal",        1. / 1990.13,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_I_A, 0, 0, 0 },
    { "level_target_bal",     1. / 1990.13,     -5.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_I_A, 0, 0, 0 },

    { "alt_sip",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "lat_sip",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_LA_DEG, 0, 0, 0 },
    { "lon_sip",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_LO_DEG, 0, 0, 0 },

    { "mode_p",               1, 0.0,           TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "x_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "y_p",                  I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "vel_az_p",             I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "del_p",                I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "w_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "ok_pss",               SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "offset_ifel_gy",       1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "offset_ifroll_gy",     1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "offset_ifyaw_gy",      1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },

    { "lvdt_high_act",        1.0,              -5000.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pulse_cal",            10.0,             0.,                TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "sigma_pss",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "sigma_clin",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "mks_hi_sip",           0.003256,         -0.226858, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mks_med_sip",          0.032614,         -0.072580, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "pref_tp_sf",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "bi0_fifo_size",        1. / 624,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "plover",               SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "jfet_set_on",          1 / 100.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "jfet_set_off",         1 / 100.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "state_cycle",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_p_heat_gy",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_i_heat_gy",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_d_heat_gy",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "t_set_gy",             (100.0 / 32768.0), 0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "h_age_gy",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "h_hist_gy",            (100.0 / 32768.0), 0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "ra_1_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dec_1_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ra_2_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dec_2_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ra_3_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dec_3_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ra_4_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dec_4_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "trim_clin",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "trim_enc",             I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "trim_null",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "trim_mag",             I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "gain_bal",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "el_clin",              I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "h_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "el_lut_clin",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mks_lo_sip",           0.327045,         -5.944902, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "alt",                  SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mode_az_mc",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mode_el_mc",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dest_az_mc",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dest_el_mc",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "vel_az_mc",            1. / 6000,        0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "vel_el_mc",            1. / 6000,        0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dir_az_mc",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dir_el_mc",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "slew_veto",            4.0 / SR,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "sveto_len",            4.0 / SR,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "pot_lock",             -100.0 / 16068.0, 1636800.0 / 16068.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dith_el",              0.5 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_D_DEG, 0, 0, 0 },
    { "state_lock",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "goal_lock",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "seized_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "n_dith_p",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "x_vel_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "x_stp_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "x_str_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "y_lim_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "i_dith_el",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "y_stp_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "y_str_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "x_lim_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "y_vel_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "he4_lev_old",          CAL16(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "off_ifel_gy_xsc0",      1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "off_ifel_gy_xsc1",      1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "off_ifroll_gy_xsc0",    1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "off_ifroll_gy_xsc1",    1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "off_ifyaw_gy_xsc0",     1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "off_ifyaw_gy_xsc1",     1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "pos_lock",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pos_0_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pos_1_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pos_2_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "enc_0_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "enc_1_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "enc_2_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "goal_sf",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "focus_sf",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "offset_ifrollmag_gy",  1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "offset_ifyawmag_gy",   1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "offset_ifrollpss_gy",  1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "offset_ifyawpss_gy",   1.0 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "next_i_hwpr_p",        SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "next_i_dith_p",        SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "cal_off_pss1",         40.0 / 65536.0,   0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_DEG, 0, 0, 0 },
    { "cal_off_pss2",         40.0 / 65536.0,   0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_DEG, 0, 0, 0 },
    { "cal_off_pss3",         40.0 / 65536.0,   0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_DEG, 0, 0, 0 },
    { "cal_off_pss4",         40.0 / 65536.0,   0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_DEG, 0, 0, 0 },
    { "cal_d_pss1",           4.0 / 65536.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_MM, 0, 0, 0 },
    { "cal_d_pss2",           4.0 / 65536.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_MM, 0, 0, 0 },
    { "cal_d_pss3",           4.0 / 65536.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_MM, 0, 0, 0 },
    { "cal_d_pss4",           4.0 / 65536.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_TRIM_MM, 0, 0, 0 },
    { "cal_imin_pss",         40.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0, 0, 0 },

    { "pref_ts_sf",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "spread_sf",            1 / 500.,         0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "acc_lock",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "i_move_act",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "i_hold_act",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "vel_act",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "acc_act",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "i_move_lock",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "i_hold_lock",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "vel_lock",             100.,             0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_prime_sf",           0.01,             0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_second_sf",          0.01,             0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_sf",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "wait_sf",              1 / 30.,          0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mode_sf",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "correction_sf",        SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "age_sf",               1 / 30.,          0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "offset_sf",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "t_prime_sf",           CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "t_second_sf",          CAL_AD590(1.0, 0.0), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "flags_act",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "lvdt_spread_act",      SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "el_sun",               I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "hwpr_cal",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mode_cal",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "overshoot_hwpr",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "drive_info_rw",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "drive_err_cts_rw",     SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "drive_info_el",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "drive_err_cts_el",     SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "drive_info_piv",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "drive_err_cts_piv",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "vel_hwpr",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "acc_hwpr",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "i_move_hwpr",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "i_hold_hwpr",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "pos_hwpr",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "enc_hwpr",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mode_bal",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_start_bias",      0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_end_bias",        0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_n_bias",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_time_bias",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_pul_len_bias",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_array_bias",      SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_start_phase",     0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_end_phase",       0.5,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_nsteps_phase",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_time_phase",      SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_ena_bias",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "step_ena_phase",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },



    { "v_batt_cc1",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "v_arr_cc1",            1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "i_batt_cc1",           1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0, 0, 0 },
    { "i_arr_cc1",            1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0, 0, 0 },
    { "t_hs_cc1",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "fault_cc1",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "alarm_hi_cc1",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "alarm_lo_cc1",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "v_targ_cc1",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "state_cc1",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },


    { "lvdt_0_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "lvdt_1_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "lvdt_2_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "gy_az_vel",            SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "gy_el_vel",            SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "gy_total_vel",         SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "gy_total_accel",       SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },

    { "offset_0_act",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "offset_1_act",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "offset_2_act",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "goal_0_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "goal_1_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "goal_2_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "az_raw_pss",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "el_raw_pss",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "snr_pss1",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "snr_pss2",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "snr_pss3",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "snr_pss4",             1 / 1000.,        0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "az_pss",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "trim_pss",             I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "accel_az",             2.0 / 65536,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "az_raw_pss1",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "az_raw_pss2",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "az_raw_pss3",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "led_cc1",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "force_sbsc",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "exp_int_sbsc",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MS, 0, 0, 0 },
    { "exp_time_sbsc",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MS, 0, 0, 0 },
    { "foc_res_sbsc",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "move_tol_sbsc",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "maxblob_sbsc",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "grid_sbsc",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "thresh_sbsc",          1.0 / 1000.0,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mdist_sbsc",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mapmean_sbsc",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mapsigma_sbsc",        1.0 / 10.0,       0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "ccd_t_sbsc",           1.0 / 100.0,      0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "nblobs_sbsc",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob00_x_sbsc",        CAM_WIDTH / SHRT_MAX, 0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob00_y_sbsc",        CAM_WIDTH / SHRT_MAX, 0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob00_f_sbsc",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob00_s_sbsc",        1.0 / 100.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob01_x_sbsc",        CAM_WIDTH / SHRT_MAX, 0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob01_y_sbsc",        CAM_WIDTH / SHRT_MAX, 0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob01_f_sbsc",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob01_s_sbsc",        1.0 / 100.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob02_x_sbsc",        CAM_WIDTH / SHRT_MAX, 0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob02_y_sbsc",        CAM_WIDTH / SHRT_MAX, 0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob02_f_sbsc",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob02_s_sbsc",        1.0 / 100.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_pt_az",              SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_pt_el",              SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pos0_hwpr",            1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pos1_hwpr",            1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pos2_hwpr",            1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pos3_hwpr",            1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "i_pos_rq_hwpr",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "rate_tdrss",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_RATE, 0, 0, 0 },
    { "rate_iridium",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_RATE, 0, 0, 0 },
    { "read_wait_hwpr",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "i_pos_hwpr",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "stop_cnt_hwpr",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "rel_move_hwpr",        2.0,              0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "stat_control_hwpr",    SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pot_targ_hwpr",        1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mode_act",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "enc_targ_hwpr",        SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "enc_err_hwpr",         SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dr_0_act",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dr_1_act",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dr_2_act",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "tol_act",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "status_actbus",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pot_err_hwpr",         1.0 / 32767.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "blob_idx_sbsc",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "az_raw_pss4",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "el_raw_pss1",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "el_raw_pss2",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "el_raw_pss3",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "el_raw_pss4",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_P_DEG, 0, 0, 0 },

    { "el_raw_enc",           I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "el_enc",               I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "sigma_enc",            I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "el_motor_enc",         I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "sigma_motor_enc",      I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "chatter",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "focpos_sbsc",          1.0 / 10.0,       0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "delay_sbsc",           1.0 / 1000.0,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "daz_p",                I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "steps_shutter",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "steps_slow_shutter",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "pos_shutter",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "ra_sbsc",              1.0 / 1000.0,     0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dec_sbsc",             1.0 / 1000.0,     0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "i_tot",                1.0e-3,           0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0, 0, 0 },
    { "t_set_sbsc",           (100.0 / 32768.0), 0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },

    { "pot_hwpr",             1.0 / 65535.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },

    /// Shared Variables
    { "t_cpu0_flc_s",          SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0 },
    { "t_cpu0_flc_n",         SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0 },

    { "t_cpu1_flc_s",          SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0 },
    { "t_cpu1_flc_n",         SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, SRC_FC, U_T_C, 0, 0, 0 },

    { "v_12v_flc_s",          0.01,             0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "v_12v_flc_n",          0.01,             0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_V_V, 0, 0, 0 },

    { "v_5v_flc_s",           0.01,             0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "v_5v_flc_n",           0.01,             0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_V_V, 0, 0, 0 },

    { "v_batt_flc_s",         0.01,             0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "v_batt_flc_n",         0.01,             0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_V_V, 0, 0, 0 },

    { "i_flc_s",              0.01,             0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_I_A, 0, 0, 0 },
    { "i_flc_n",              0.01,             0.0, TYPE_UINT16, RATE_1HZ, SRC_FC, U_I_A, 0, 0, 0 },

    { "last_cmd_s",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "last_cmd_n",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "count_cmd_s",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "count_cmd_n",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "df_flc_s",             1.0 / 250.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_GB, 0, 0, 0 },
    { "df_flc_n",             1.0 / 250.0,      0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_GB, 0, 0, 0 },

    { "time_flc_s",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "time_flc_n",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "timeout_s",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "timeout_n",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "t_start_cycle",        4.0 / 65536.0,    0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_K, 0, 0, 0 },
    { "t_pot_max_cycle",      10.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_K, 0, 0, 0 },
    { "t_char_max_cycle",     70.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_K, 0, 0, 0 },
    { "t_char_set_cycle",     70.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_T_K, 0, 0, 0 },
    { "time_char_cycle",      120.0 / 65536.0,  0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MIN, 0, 0, 0 },
    { "time_set_cycle",       120.0 / 65536.0,  0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_T_MIN, 0, 0, 0 },
    { "thresh_atrim",         10.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "time_atrim",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "rate_atrim",           30.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "v_batt_cc2",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "v_arr_cc2",            1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "i_batt_cc2",           1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0, 0, 0 },
    { "i_arr_cc2",            1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_I_A, 0, 0, 0 },
    { "t_hs_cc2",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_IF_UEI, U_T_C, 0, 0, 0 },
    { "fault_cc2",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "alarm_hi_cc2",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "alarm_lo_cc2",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "v_targ_cc2",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_V_V, 0, 0, 0 },
    { "state_cc2",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "led_cc2",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    #ifndef BOLOTEST

    { "latch0",               SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "latch1",               SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "switch_gy",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "switch_misc",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "bus_reset_act",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "v1_3_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v2_3_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v3_3_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v4_3_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v1_4_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v2_4_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v3_4_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v4_4_pss",             CAL16(-1., 0.), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },

    { "v_pump_bal",           3.91 / 13107.0,   -9.775, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "dac2_ampl",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "mask_gy",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "bits_vtx",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "bits_bal",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "pch_pyr_clin",         0.001343,         -45.426, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "roll_pyr_clin",        0.001413,         -45.398, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "xel_if_clin",          0.00546739,       -25. * 6.144, TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "el_raw_if_clin",       SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },

    { "x_stage",              2.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "y_stage",              2.0,              0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "ifpm_hall",            SCALE(CONVERT_UNITY),            TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "lvdt_65_act",          LVDT65_ADC_TO_ENC, LVDT65_ZERO,    TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "lvdt_63_act",          LVDT63_ADC_TO_ENC, LVDT63_ZERO,    TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "lvdt_64_act",          LVDT64_ADC_TO_ENC, LVDT64_ZERO,    TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "v1_1_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v2_1_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v3_1_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v4_1_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v1_2_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v2_2_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v3_2_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },
    { "v4_2_pss",             CAL16(-1., 0.),                   TYPE_UINT16, RATE_5HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },

    #endif

    { "ifyaw_1_gy",           SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifroll_1_gy",          SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifyaw_2_gy",           SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifel_1_gy",            SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifel_2_gy",            SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifroll_2_gy",          SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0, 0, 0 },

    { "good_pktcnt_yaw_1_gy",  SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "good_pktcnt_roll_1_gy", SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "good_pktcnt_el_1_gy",   SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "good_pktcnt_yaw_2_gy",  SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "good_pktcnt_roll_2_gy", SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "good_pktcnt_el_2_gy",   SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "az",                   LI2DEG,   0.0,            TYPE_UINT32, RATE_200HZ, SRC_FC, U_P_DEG, 0, 0, 0 },
    { "el",                   LI2DEG,   0.0,            TYPE_UINT32, RATE_200HZ, SRC_FC, U_P_DEG, 0, 0, 0 },

    { "uei_if_framenum",      SCALE(CONVERT_UNITY),            TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_NONE, 0, 0, 0 },
    { "uei_of_framenum",      SCALE(CONVERT_UNITY),            TYPE_UINT16, RATE_100HZ, SRC_OF_UEI, U_NONE, 0, 0, 0 },

    { "ifel_gy",              SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_100HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifroll_gy",            SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_100HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifyaw_gy",             SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_100HZ, SRC_FC, U_V_DPS, 0, 0, 0 },

    { "vel_req_el",           SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "vel_req_az",           SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, SRC_FC, U_V_DPS, 0, 0, 0 },

    { "trigger_xsc",          SCALE(CONVERT_UNITY),            TYPE_UINT8, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "dig43_das",            SCALE(CONVERT_UNITY),            TYPE_UINT16, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "chopper",              CAL16(1.0, 0.0),          TYPE_UINT16, RATE_100HZ, SRC_IF_UEI, U_V_V, 0, 0, 0 },

    { "x_mag",                1.0/15000.0,              0,  TYPE_INT16, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "y_mag",                1.0/15000.0,              0,  TYPE_INT16, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "z_mag",                1.0/15000.0,              0,  TYPE_INT16, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "ifel_earth_gy",        0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifroll_earth_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "ifyaw_earth_gy",       0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },

    { "az_mag",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_D_DEG, 0, 0, 0 },
    { "az_raw_mag",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_D_DEG, 0, 0, 0 },
    { "pitch_mag",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_D_DEG, 0, 0, 0 },

    { "sigma_mag",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "declination_mag",      I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_D_DEG, 0, 0, 0 },
    { "dip_mag",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, SRC_FC, U_D_DEG, 0, 0, 0 },

    { "cal_xmax_mag",         1, 0.0,           TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "cal_xmin_mag",         1, 0.0,           TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "cal_ymax_mag",         1, 0.0,           TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "cal_ymin_mag",         1, 0.0,           TYPE_INT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },


    /** Motor Channels */
    { "mc_rw_vel",           RW_ENCODER_SCALING * 0.1,  0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "mc_rw_pos",           RW_ENCODER_SCALING,        0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_D_DEG, 0, 0, 0 },
    { "mc_el_vel",           EL_MOTOR_ENCODER_SCALING * 0.1,  0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "mc_el_pos",           EL_LOAD_ENCODER_SCALING,   0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_D_DEG, 0, 0, 0 },
    { "mc_el_motor_pos",     EL_MOTOR_ENCODER_SCALING,  0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_D_DEG, 0, 0, 0 },
    { "mc_piv_vel",          PIV_RESOLVER_SCALING * 0.1, 0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_V_DPS, 0, 0, 0 },
    { "mc_piv_pos",          PIV_RESOLVER_SCALING,      0.0, TYPE_INT32, RATE_100HZ, SRC_FC, U_D_DEG, 0, 0, 0 },

    { "mc_el_biss_status",   SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },

    {"control_word_read_el", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    {"control_word_read_rw", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    {"control_word_read_piv", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    {"latched_fault_el",     SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    {"latched_fault_rw",     SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    {"latched_fault_piv",    SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    {"network_status_el",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    {"network_status_rw",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    {"network_status_piv",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    // TODO(seth): Ensure that scale factors for all currents/commands are in Amps
    { "mc_piv_i_cmd",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mc_rw_i_cmd",            1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mc_el_i_cmd",            1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "mc_piv_i_read",          1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mc_rw_i_read",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mc_el_i_read",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },

    /** Velocity control loop commanded P/I terms */
    { "g_p_el",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_i_el",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_d_el",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_db_el",              SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_p_az",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_i_az",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_d_az",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_pe_piv",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_pv_piv",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "g_iv_piv",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "set_rw",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_V_DPS, 0, 0, 0 },

    { "fault_gy",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "vel_el_p",             I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "frict_off_piv",        SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "frict_term_piv",       SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "frict_term_uf_piv",    SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "frict_off_el",        SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "frict_term_el",       SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "frict_term_uf_el",    SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    /** Motor Controller State and Status Registers */
    { "status_rw",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "state_rw",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "status_el",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "state_el",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "status_piv",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "state_piv",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    /** Motor Controller Temperatures */
    { "t_mc_rw",              SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_T_C, 0, 0, 0 },
    { "t_mc_el",              SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_T_C, 0, 0, 0 },
    { "t_mc_piv",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, SRC_FC, U_T_C, 0, 0, 0 },

    /** Calculated P/I and Error (diff btw commanded/actual velocity) terms from control loop */
    { "p_term_el",            0.01,             0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "i_term_el",            0.01,             0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "d_term_el",            0.01,             0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "error_el",             SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "el_integral_step",     SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "p_term_az",            0.01,             0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "i_term_az",            0.01,             0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "d_term_az",            0.01,             0.0,    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "error_az",             SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "az_integral_step",     SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "p_rw_term_piv",        SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "i_rw_term_piv",        SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "p_err_term_piv",       SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_5HZ, SRC_FC, U_NONE, 0, 0, 0 },

    { "uei_of_1hz_framecount",   SCALE(CONVERT_UNITY), TYPE_UINT32,    RATE_1HZ,   SRC_OF_UEI, U_NONE, 0, 0, 0 },
    { "uei_of_100hz_framecount", SCALE(CONVERT_UNITY), TYPE_UINT32,    RATE_100HZ, SRC_OF_UEI, U_NONE, 0, 0, 0 },

    { "mcp_1hz_framecount",     SCALE(CONVERT_UNITY),  TYPE_UINT32,    RATE_1HZ,   SRC_FC, U_NONE, 0, 0, 0 },
    { "mcp_5hz_framecount",     SCALE(CONVERT_UNITY),  TYPE_UINT32,    RATE_5HZ,   SRC_FC, U_NONE, 0, 0, 0 },
    { "mcp_100hz_framecount",   SCALE(CONVERT_UNITY),  TYPE_UINT32,    RATE_100HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { "mcp_200hz_framecount",   SCALE(CONVERT_UNITY),  TYPE_UINT32,    RATE_200HZ, SRC_FC, U_NONE, 0, 0, 0 },
    { {0} }
  };

