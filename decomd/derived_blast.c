/* derived.c: a list of derived channels
 *
 * This software is copyright (C) 2002-2005 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 *
 * IF YOU ADD, MODIFY, OR DELETE *ANY* CHANNELS IN THIS FILE YOU *MUST*
 * RECOMPILE AND RESTART THE DECOM DAEMON (DECOMD) ON ARWEN!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include "derived.h"

/* Don's Handy Guide to adding derived channels:
 *
 * There are seven types of derived channels which can be added to the format
 * file.  Channel names are, of course, case sensitive.  On start-up, both
 * mcp and decomd run sanity checks on the channel lists, including the derived
 * channel list.  The checks will fail for the derived channel list if either
 * a source channel doesn't exist or a derived channel has a name already in
 * use.  Listed below are the derived channel types and their arguments.  If
 * more derived channel types are desired, please ask me to add them.  --dvw
 *
 * o LINCOM: Single field calibration.  Arguments:
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Calibration Slope (double)
 *   4.  Calibration Intercept (double)
 * o LINCOM2: Two field linear combination
 *   1.  Derived Channel Name (string)
 *   2.  Source 1 Channel Name (string)
 *   3.  Source 1 Calibration Slope (double)
 *   4.  Source 1 Calibration Intercept (double)
 *   5.  Source 2 Channel Name (string)
 *   6.  Source 2 Calibration Slope (double)
 *   7.  Source 2 Calibration Intercept (double)
 * o LINTERP: Linearly interpolated look up table
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Full Path to Look Up Table File (string)
 * o BITFIELD: Single bit channels derived from a common source channel
 *   1.  Source Channel Name (string)
 *   2.  Bit 0 Derived Channel Name (string)
 *   3.  Bit 1 Derived Channel Name (string)
 *   ...
 *   17. Bit 15 Derived Channel Name (string)
 *
 *      NB: Bits for which no channel is defined should be given an empty string
 *      "" as a channel name.  Additionally, unused trailing high-bit channels
 *      can be omitted.
 * o BITWORD: A multi-bit channel extracted from a larger source channel
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  First bit of the bitword, numbered from zero (integer)
 *   4.  Length of the bitword (integer)
 * o COMMENT: A litteral comment to be inserted into the format file
 *   1.  Comment Text (string) -- there is no need to include the comment
 *       delimiter (#).
 * o UNITS: meta data defining units and quantity for existing fields
 *   1.  Source Channel Name (string)
 *   2.  String for the quantity (eg "Temperature")
 *   3.  String for the Units (eg, "^oC")
 *
 *
 * In addition to the derived channels derived below, defile will add the
 * "Nice CPU Values" (to wit: CPU_SEC, CPU_HOUR, etc.), properly offset to the
 * start of the file, to the end of the format file.
 */

union DerivedUnion DerivedChannels[] = {
  /* Pointing */
  COMMENT("Microsecond Resolution Time"),
  LINCOM2("Time", "time_usec", 1.0E-6, 0, "time",  1, 0),
  UNITS("Time", "Time", "s"),

  COMMENT("General BLAST Status"),
  BITFIELD("status_mcc",
      "SOUTH_I_AM",
      "AT_FLOAT",
      "",
      "",
      "BLAST_SUCKS"),
  BITWORD("SCHEDULE", "status_mcc", 4, 3),
  BITWORD("ALICE_FILE", "status_mcc", 8, 8),

#ifndef BOLOTEST
  COMMENT("Pointing Stuff"),
  LINCOM("P_X_H", "p_x_deg", 0.0003662109375, 0),
  LINTERP("Clin_Elev", "clin_elev", "/data/etc/clin_elev.lut"),
  UNITS("Clin_Elev", "Elevation", "^o"),
  LINTERP("SS_AzRelSun", "SS_AZ", "/data/etc/ss.lut"),
  BITFIELD("sensor_veto",
      "SUN_VETO",
      "ISC_VETO",
      "ELENC_VETO",
      "MAG_VETO",
      "GPS_VETO",
      "ELCLIN_VETO",
      "OSC_VETO",
      "IS_SCHED",
      "AZ_AUTO_GYRO",
      "EL_AUTO_GYRO",
      "DISABLE_EL",
      "DISABLE_AZ",
      "FORCE_EL"
      ),

#if 0
  BITFIELD("sensor_reset",
      "",
      "",
      "",
      "DGPS_RESET",
      "ISC_RESET",
      "GYRO_RESET",
      "SS_RESET",
      "OSC_RESET"
      ),
#endif

  /* ISC and OSC */
  COMMENT("Star Camera State"),
  BITFIELD("isc_state",
      "ISC_SAVE_IMAGES",
      "ISC_PAUSE",
      "ISC_ABORT",
      "ISC_AUTOFOCUS",
      "ISC_SHUTDOWN",
      "ISC_REBOOT",
      "ISC_EYE_ON",
      "ISC_HEATER_ON",
      "ISC_USE_LOST"
      ),
  BITWORD("ISC_SENT_TRIG", "isc_trigger", 0, 14),

  BITFIELD("osc_state",
      "OSC_SAVE_IMAGES",
      "OSC_PAUSE",
      "OSC_ABORT",
      "OSC_AUTOFOCUS",
      "OSC_SHUTDOWN",
      "OSC_REBOOT",
      "OSC_EYE_ON",
      "OSC_HEATER_ON",
      "OSC_USE_LOST"
      ),
  BITWORD("OSC_SENT_TRIG", "osc_trigger", 0, 14),

  COMMENT("ACS Digital Signals"),

  BITFIELD("bits_bal",
      "DIR_BAL",
      "VALVE_BAL",
      "HEAT_BAL",
      ),

  /* charge controller (CC) faults and alarms */

  BITFIELD("fault_chrgctrl",
      "CC_F_OVERCURRENT", 
      "CC_F_FET_SHORT", 
      "CC_F_SOFTWARE_BUG", 
      "CC_F_BATT_HVD", 
      "CC_F_ARR_HVD", 
      "CC_F_DIP_CHANGED", 
      "CC_F_SETTINGS_CHANGE", 
      "CC_F_RTS_SHORT", 
      "CC_F_RTS_DISCONN",
      "CC_F_EEPROM_LIM",
      "CC_F_SLAVE_TO" 
      ),

  BITFIELD("alarm_lo_chrgctrl",
      "CC_A_RTS_OPEN", 
      "CC_A_RTS_SHORT", 
      "CC_A_RTS_DISCONN", 
      "CC_A_TSENSE_OPEN", 
      "CC_A_TSENSE_SHORT", 
      "CC_A_HITEMP_LIM", 
      "CC_A_CURRENT_LIM", 
      "CC_A_CURRENT_OFFSET", 
      "CC_A_BATTSENSE_RANGE",
      "CC_A_BATTSENSE_DISC",
      "CC_A_UNCALIB",
      "CC_A_RTS_MISWIRE",
      "CC_A_HVD",
      "",
      "CC_A_SYS_MISWIRE",
      "CC_A_FET_OPEN",
      ),

 BITFIELD("alarm_hi_chrgctrl",
      "CC_A_VP12_OFF",  
      "CC_A_HI_INPUT_LIM", 
      "CC_A_ADC_MAX_IN", 
      "CC_A_RESET", 
      ),

#endif

  COMMENT("Lock Motor/Actuators"),
  BITFIELD("lock_state",
      "LS_OPEN",
      "LS_CLOSED",
      "LS_DRIVE_OFF",
      "LS_POT_RAIL",
      "LS_DRIVE_EXT",
      "LS_DRIVE_RET",
      "LS_DRIVE_STP",
      "LS_DRIVE_JIG",
      "LS_DRIVE_UNK",
      "LS_EL_OK",
      "LS_IGNORE_EL",
      "LS_DRIVE_FORCE"
      ),

  /* Secondary Focus */
  COMMENT("Secondary Focus"),
  LINCOM2("REL_FOCUS", "SF_CORRECTION", 1, 0, "SF_OFFSET", 1, 0),
  LINCOM2("TC_VETO", "TC_WAIT", 1, 0, "SF_AGE", -1, 0),

  BITFIELD("act_flags",
      "ACT_LOST",
      "ACT_DR_POS_BAD",
      "ACT_DR_ENC_BAD",
      "ACT_ENC_POS_BAD",
      "ACT_DR_LVDT_BAD",
      "ACT_ENC_LVDT_BAD",
      "ACT_POS_LVDT_BAD",
      "ACT_BAD_MOVE",
      "ACT0_FAULT",
      "ACT1_FAULT",
      "ACT2_FAULT"
      ),

  BITFIELD("stat_1_el",
	   "ST_SHORT_CIRC_EL",
	   "ST_AMP_OVER_T_EL",
	   "ST_OVER_V_EL",
	   "ST_UNDER_V_EL",
	   "",
	   "ST_FEEDBACK_ERR_EL",
	   "ST_MOT_PHAS_ERR_EL",
	   "ST_I_LIMITED_EL",
	   "ST_VOLT_LIM_EL",
	   "",
           "",
	   "ST_DISAB_HWARE_EL",
	   "ST_DISAB_SWARE_EL",	
	   "ST_ATTEMPT_STOP_EL",
	   "ST_MOT_BREAK_ACT_EL",
	   "ST_PWM_OUT_DIS_EL"
	   ),
  BITFIELD("stat_2_el",
	   "ST_POS_SOFT_LIM_EL",
	   "ST_NEG_SOFT_LIM_EL",
	   "ST_FOLLOW_ERR_EL",
	   "ST_FOLLOW_WARN_EL",
	   "ST_AMP_HAS_RESET_EL",
	   "ST_ENCODER_WRAP_EL",
	   "ST_AMP_FAULT_EL",
	   "ST_VEL_LIMITED_EL",
	   "ST_ACCEL_LIMITED_EL",
	   "",
	   "",
	   "ST_IN_MOTION_EL",
	   "ST_V_OUT_TRACK_W_EL",
	   "ST_PHASE_NOT_INIT_EL",
	   "",
	   ""
   ),
  BITFIELD("stat_1_rw",
	   "ST_SHORT_CIRC_RW",
	   "ST_AMP_OVER_T_RW",
	   "ST_OVER_V_RW",
	   "ST_UNDER_V_RW",
	   "",
	   "ST_FEEDBACK_ERR_RW",
	   "ST_MOT_PHASE_ERR_RW",
	   "ST_I_LIMITED_RW",
	   "ST_VOLT_LIM_RW",
	   "",
           "",
	   "ST_DISAB_HWARE_RW",
	   "ST_DISAB_SWARE_RW",	
	   "ST_ATTEMPT_STOP_RW",
	   "ST_MOT_BREAK_ACT_RW",
	   "ST_PWM_OUT_DISAB_RW"
   ),
  BITFIELD("stat_2_rw",
	   "ST_POS_SOFT_LIM_RW",
	   "ST_NEG_SOFT_LIM_RW",
	   "ST_FOLLOW_ERR_RW",
	   "ST_FOLLOW_WARN_RW",
	   "ST_AMP_HAS_RESET_RW",
	   "ST_ENCODER_WRAP_RW",
	   "ST_AMP_FAULT_RW",
	   "ST_VEL_LIMITED_RW",
	   "ST_ACCEL_LIMITED_RW",
	   "",
	   "",
	   "ST_IN_MOTION_RW",
	   "ST_V_OUT_TRACK_W_RW",
	   "ST_PHASE_NOT_INIT_RW",
	   "",
	   ""
   ),
  BITFIELD("drive_info_rw",
	   "DR_INFO_OPEN_RW",
	   "DR_INFO_RESET_RW",
	   "DR_INFO_INIT_1_RW",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "DR_INFO_CLOSING_RW"
   ),
  BITWORD("DR_INFO_ERR_RW","drive_info_rw",10,5),
  BITWORD("DR_INFO_DISAB_RW","drive_info_rw",4,2),
  BITWORD("DR_INFO_BDRATE_RW","drive_info_rw",6,2),
  BITWORD("DR_INFO_WRSET_RW","drive_info_rw",8,2),
  BITWORD("DR_INFO_INIT_RW", "drive_info_rw",2,2),
  LINCOM2("DR_INFO_IO_RW","DR_INFO_OPEN_RW",1,0,"DR_INFO_INIT_1_RW",2,0),
  LINCOM2("DR_INFO_RIO_RW","DR_INFO_RESET_RW",4,0,"DR_INFO_IO_RW",1,0),

  BITFIELD("drive_info_el",
	   "DR_INFO_OPEN_EL",
	   "DR_INFO_RESET_EL",
	   "DR_INFO_INIT_1_EL",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "DR_INFO_CLOSING_EL"
   ),
  BITWORD("DR_INFO_ERR_EL","drive_info_el",10,5),
  BITWORD("DR_INFO_DISAB_EL","drive_info_el",4,2),
  BITWORD("DR_INFO_BDRATE_EL","drive_info_el",6,2),
  BITWORD("DR_INFO_WRSET_EL","drive_info_el",8,2),
  BITWORD("DR_INFO_INIT_EL", "drive_info_el",2,2),
  LINCOM2("DR_INFO_IO_EL","DR_INFO_OPEN_EL",1,0,"DR_INFO_INIT_1_EL",2,0),
  LINCOM2("DR_INFO_RIO_EL","DR_INFO_RESET_EL",4,0,"DR_INFO_IO_EL",1,0),
  BITFIELD("drive_info_piv",
	   "DR_INFO_OPEN_PIV",
	   "DR_INFO_RESET_PIV",
	   "DR_INFO_INIT_1_PIV",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "",
	   "DR_INFO_CLOSING_PIV"
   ),
  BITWORD("DR_INFO_ERR_PIV","drive_info_piv",10,5),
  BITWORD("DR_INFO_DISAB_PIV","drive_info_piv",4,2),
  BITWORD("DR_INFO_BDRATE_PIV","drive_info_piv",6,2),
  BITWORD("DR_INFO_WRSET_PIV","drive_info_piv",8,2),
  BITWORD("DR_INFO_INIT_PIV", "drive_info_piv",2,2),
  LINCOM2("DR_INFO_IO_PIV","DR_INFO_OPEN_PIV",1,0,"DR_INFO_INIT_1_PIV",2,0),
  LINCOM2("DR_INFO_RIO_PIV","DR_INFO_RESET_PIV",4,0,"DR_INFO_IO_PIV",1,0),
  BITFIELD("fault_el",
	   "ST_F_CRC_EL",
	   "ST_F_AD_OFF_RANGE_EL",
	   "ST_F_SHORT_CIRC_EL",
	   "ST_F_AMP_OVER_T_EL",
	   "",
	   "ST_F_OVER_VOLT_EL",
	   "ST_F_UNDER_VOLT_EL",
	   "ST_F_FEEDBACK_ERR_EL",
	   "ST_F_MOT_PHAS_ERR_EL",
	   "ST_F_FOLL_ERR_EL",
	   "ST_F_OVER_CURR_EL"
	   ),
  BITFIELD("fault_rw",
	   "ST_F_CRC_RW",
	   "ST_F_AD_OFF_RANGE_RW",
	   "ST_F_SHORT_CIRC_RW",
	   "ST_F_AMP_OVER_T_RW",
	   "",
	   "ST_F_OVER_VOLT_RW",
	   "ST_F_UNDER_VOLT_RW",
	   "ST_F_FEEDBACK_ERR_RW",
	   "ST_F_MOT_PHAS_ERR_RW",
	   "ST_F_FOLL_ERR_RW",
	   "ST_F_OVER_CURR_RW"
	   ),
  BITFIELD("stat_dr_piv",
	   "ST_BRIDGE_ENA_PIV",
	   "ST_DYN_BRAKE_ENA_PIV",
	   "ST_SHUNT_EN_PIV",
	   "ST_POS_STOP_ENA_PIV",
	   "ST_NEG_STOP_ENA_PIV",
	   "ST_POS_TORQ_INH_PIV",
	   "ST_NEG_TORQ_INH_PIV",
	   "ST_EXT_BRAKE_PIV",
	   "ST_DR_RESET_PIV",
	   "ST_DR_INTER_ERR_PIV",
	   "ST_DR_SHORT_CIRC_PIV",
	   "ST_DR_I_OVERSHOT_PIV",
	   "ST_DR_UNDER_V_PIV",
	   "ST_DR_OVER_V_PIV",
	   "ST_DR_OVER_TEMP_PIV",
	   ""
	   ),
  /* CRYO */

#if 0
  COMMENT("Internal (to mcp) reference of the cryo stat"),
  BITFIELD("cryostate",
      "HE_LEV_SENS",
      "CHARC_HEATER",
      "COLDP_HEATER",
      "FRIDGE_CYCLING",
      "POT_VALVE",
      "POT_DIREC",
      "LHE_VALVE",
      "CRYO_DIREC",
      "LN_VALVE",
      "AUTO_JFET_HEAT"
      ),

  COMMENT("Control bits to the Cryo control card"),
  BITFIELD("cryoctrl",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "CAL_LAMP_ON_CMD",
      "AUTO_BDA_HEAT_ON"
      ),

  COMMENT("Cryo Valve Limit Switches"),

  BITFIELD("cryoin",
      "POT_IS_CLOSED",
      "POT_IS_OPEN",
      "LHE_IS_CLOSED",
      "LHE_IS_OPEN",
      "LN_IS_CLOSED",
      "LN_IS_OPEN"
      ),

  LINCOM2("POT_STATE", "POT_IS_CLOSED", 2, 0, "POT_IS_OPEN",  1, 0),
  LINCOM2("LHE_STATE", "LHE_IS_CLOSED", 2, 0, "LHE_IS_OPEN",  1, 0),
  LINCOM2("LN_STATE",  "LN_IS_CLOSED", 2, 0, "LN_IS_OPEN", 1, 0),

  COMMENT("Limit Switch Niceties"),

  LINCOM2("LS_EL_STATUS", "LS_EL_OK", 2, 0, "LS_IGNORE_EL", 1, 0),
  LINCOM2("LS_OPENCLOSE", "LS_OPEN", 2, 0, "LS_CLOSED", 1, 0),
  LINCOM2("LS_MOTION", "LS_DRIVE_RET", 2, 0, "LS_DRIVE_EXT", 1, 0),
  LINCOM2("LS_NICE_STAT", "LS_MOTION", 3, 0, "LS_OPENCLOSE", 1, 0),
#endif

  COMMENT("DAS Digital Controls"),
  BITFIELD("das_dig43",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "HEAT_HELIUM_LEVEL",
      "HEAT_CHARCOAL",
      "HEAT_CHARCOAL_HS",
      "HEAT_POT_HS",
      "HEAT_JFET",
      "HEAT_BDA",
      "HEAT_CALIBRATOR",
      "HEAT_HWPR_POS"),

  COMMENT("Cryo Table Lookups"),
  COMMENT("Diodes"),

  LINTERP("Td_charcoal",   "TD_CHARCOAL"   , "/data/etc/dt600.txt"),
    UNITS("Td_charcoal", "Temperature", "K"),
  LINTERP("Td_lhe",        "TD_LHE"        , "/data/etc/dt600.txt"),
    UNITS("Td_lhe", "Temperature", "K"),
  LINTERP("Td_ln",         "TD_LN"         , "/data/etc/dt600.txt"),
    UNITS("Td_ln", "Temperature", "K"),
  LINTERP("Td_hs_pot",     "TD_HS_POT"     , "/data/etc/dt-simonchase.txt"),
    UNITS("Td_hs_pot", "Temperature", "K"),
  LINTERP("Td_jfet",       "TD_JFET"       , "/data/etc/dt600.txt"),
    UNITS("Td_jfet", "Temperature", "K"),
  LINTERP("Td_vcs_filt",   "TD_VCS_FILT"   , "/data/etc/dt600.txt"),
    UNITS("Td_vcs_filt", "Temperature", "K"),
  LINTERP("Td_ln_filt",    "TD_LN_FILT"    , "/data/etc/dt600.txt"),
    UNITS("Td_ln_filt", "Temperature", "K"),
  LINTERP("Td_lhe_filt",   "TD_LHE_FILT"   , "/data/etc/dt600.txt"),
    UNITS("Td_lhe_filt", "Temperature", "K"),
  LINTERP("Td_vcs_jfet",   "TD_VCS_JFET"   , "/data/etc/dt600.txt"),
    UNITS("Td_vcs_jfet", "Temperature", "K"),
  LINTERP("Td_hs_charcoal","TD_HS_CHARCOAL", "/data/etc/dt-simonchase.txt"),
    UNITS("Td_hs_charcoal", "Temperature", "K"),

  COMMENT("ROXes"), // raw calibration only -- warning!
  LINTERP("Tr_300mk_strap","TR_300MK_STRAP", "/data/etc/rox-raw.txt"),
    UNITS("Tr_300mk_strap", "Temperature", "K"),
  LINTERP("Tr_he3_fridge", "TR_HE3_FRIDGE" , "/data/etc/rox-raw.txt"),
    UNITS("Tr_he3_fridge", "Temperature", "K"),
  LINTERP("Tr_m5",	   "TR_M5"         , "/data/etc/rox-raw.txt"),
    UNITS("Tr_m5", "Temperature", "K"),
  LINTERP("Tr_m4",         "TR_M4"         , "/data/etc/rox-raw.txt"),
    UNITS("Tr_m4", "Temperature", "K"),
  LINTERP("Tr_hwpr",       "TR_HWPR"       , "/data/etc/rox-raw.txt"),
    UNITS("Tr_hwpr", "Temperature", "K"),
  LINTERP("Tr_horn_500",   "TR_HORN_500"   , "/data/etc/rox-raw.txt"),
    UNITS("Tr_horn_500", "Temperature", "K"),
  LINTERP("Tr_horn_350",   "TR_HORN_350"   , "/data/etc/rox-raw.txt"),
    UNITS("Tr_horn_350", "Temperature", "K"),
  LINTERP("Tr_horn_250",   "TR_HORN_250"   , "/data/etc/rox-raw.txt"),
    UNITS("Tr_horn_250", "Temperature", "K"),
  LINTERP("Tr_he4_pot",    "TR_HE4_POT"    , "/data/etc/rox-raw.txt"),
    UNITS("Tr_he4_pot", "Temperature", "K"),
  LINTERP("Tr_optbox_filt","TR_OPTBOX_FILT", "/data/etc/rox-raw.txt"),
    UNITS("Tr_optbox_filt", "Temperature", "K"),

  COMMENT("ROX Voltage Calibrations for noise tests"),
  LINCOM("Vr_HE3_FRIDGE" , "tr_he3_fridge"  , 2.7351E-9, -5.8625),
   UNITS("Vr_HE3_FRIDGE", "RMS Voltage", "V"),
  LINCOM("Vr_M5"         , "tr_m5"          , 2.7351E-9, -5.8625),
   UNITS("Vr_M5", "RMS Voltage", "V"),
  LINCOM("Vr_M4"         , "tr_m4"          , 2.7351E-9, -5.8625),
   UNITS("Vr_M4", "RMS Voltage", "V"),
  LINCOM("Vr_HWPR"       , "tr_hwpr"        , 2.7351E-9, -5.8625),
   UNITS("Vr_HWPR", "RMS Voltage", "V"),
  LINCOM("Vr_HORN_500"   , "tr_horn_500"    , 2.7351E-9, -5.8625),
   UNITS("Vr_HORN_500", "RMS Voltage", "V"),
  LINCOM("Vr_HORN_350"   , "tr_horn_350"    , 2.7351E-9, -5.8625),
   UNITS("Vr_HORN_350", "RMS Voltage", "V"),
  LINCOM("Vr_HORN_250"   , "tr_horn_250"    , 2.7351E-9, -5.8625),
   UNITS("Vr_HORN_250", "RMS Voltage", "V"),
  LINCOM("Vr_300MK_STRAP", "tr_300mk_strap" , 2.7351E-9, -5.8625),
   UNITS("Vr_300MK_STRAP", "RMS Voltage", "V"),
  LINCOM("Vr_HE4_POT"    , "tr_he4_pot"     , 2.7351E-9, -5.8625),
   UNITS("Vr_HE4_POT", "RMS Voltage", "V"),
  LINCOM("Vr_OPTBOX_FILT", "tr_optbox_filt" , 2.7351E-9, -5.8625),
   UNITS("Vr_OPTBOX_FILT", "RMS Voltage", "V"),


//  COMMENT("Level Sensor"),
//  LINTERP("HE4_LITRE", "HE4_LEV", "/data/etc/he4_litre.txt"),
//  LINTERP("HE4_PERCENT", "HE4_LEV", "/data/etc/he4_percent.txt"),

  END_OF_DERIVED_CHANNELS
};
