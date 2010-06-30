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
  LINCOM("X_H_P", "x_p", 0.0003662109375, 0),
  LINTERP("EL_IF_CLIN", "el_raw_if_clin", "/data/etc/clin_elev.lut"),
  UNITS("EL_IF_CLIN", "Elevation", "^o"),
#if 0  //TODO we probably aren't using a LUT for AZ_SS anymore
  LINTERP("AZ_LUT_SS", "AZ_SS", "/data/etc/ss.lut"),
#endif
  BITFIELD("veto_sensor",
      "VETO_SS",
      "VETO_ISC",
      "VETO_EL_ENC",
      "VETO_MAG",
      "VETO_GPS",
      "VETO_EL_CLIN",
      "VETO_OSC",
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
  BITFIELD("state_isc",
      "SAVE_IMAGES_ISC",
      "PAUSE_ISC",
      "ABORT_ISC",
      "AUTOFOCUS_REQ_ISC",
      "SHUTDOWN_ISC",
      "REBOOT_ISC",
      "EYE_ON_ISC",
      "HEATER_ON_ISC",
      "USE_LOST_ISC",
      "AUTOFOCUS_ISC"
      ),
  BITWORD("SENT_TRIG_ISC", "trigger_isc", 0, 14),

  BITFIELD("state_osc",
      "SAVE_IMAGES_OSC",
      "PAUSE_OSC",
      "ABORT_OSC",
      "AUTOFOCUS_REQ_OSC",
      "SHUTDOWN_OSC",
      "REBOOT_OSC",
      "EYE_ON_OSC",
      "HEATER_ON_OSC",
      "USE_LOST_OSC",
      "AUTOFOCUS_OSC"
      ),
  BITWORD("SENT_TRIG_OSC", "trigger_osc", 0, 14),

  COMMENT("ACS Digital Signals"),

  BITFIELD("bits_bal",
      "DIR_BAL",
      "VALVE_BAL",
      "HEAT_BAL",
      ),

  /* charge controller (CC) faults and alarms */

  BITFIELD("fault_cc",
      "F_OVERCURRENT_CC", 
      "F_FET_SHORT_CC", 
      "F_SOFTWARE_BUG_CC", 
      "F_BATT_HVD_CC", 
      "F_ARR_HVD_CC", 
      "F_DIP_CHANGED_CC", 
      "F_SETTINGS_CHANGE_CC", 
      "F_RTS_SHORT_CC", 
      "F_RTS_DISCONN_CC",
      "F_EEPROM_LIM_CC",
      "F_SLAVE_TO_CC" 
      ),

  BITFIELD("alarm_lo_cc",
      "A_RTS_OPEN_CC", 
      "A_RTS_SHORT_CC", 
      "A_RTS_DISCONN_CC", 
      "A_TSENSE_OPEN_CC", 
      "A_TSENSE_SHORT_CC", 
      "A_HITEMP_LIM_CC", 
      "A_CURRENT_LIM_CC", 
      "A_CURRENT_OFFSET_CC", 
      "A_BATTSENSE_RANGE_CC",
      "A_BATTSENSE_DISC_CC",
      "A_UNCALIB_CC",
      "A_RTS_MISWIRE_CC",
      "A_HVD_CC",
      "",
      "A_SYS_MISWIRE_CC",
      "A_FET_OPEN_CC",
      ),

 BITFIELD("alarm_hi_cc",
      "A_VP12_OFF_CC",  
      "C_A_HI_INPUT_LIM_CC", 
      "C_A_ADC_MAX_IN_CC", 
      "C_A_RESET_CC", 
      ),

#endif

  COMMENT("Lock Motor/Actuators"),
  BITFIELD("state_lock",
      "LS_OPEN_LOCK",
      "LS_CLOSED_LOCK",
      "LS_DRIVE_OFF_LOCK",
      "LS_POT_RAIL_LOCK",
      "LS_DRIVE_EXT_LOCK",
      "LS_DRIVE_RET_LOCK",
      "LS_DRIVE_STP_LOCK",
      "LS_DRIVE_JIG_LOCK",
      "LS_DRIVE_UNK_LOCK",
      "LS_EL_OK_LOCK",
      "LS_IGNORE_EL_LOCK",
      "LS_DRIVE_FORCE_LOCK"
      ),

  /* Secondary Focus */
  COMMENT("Secondary Focus"),
  LINCOM2("REL_FOCUS_SF", "CORRECTION_SF", 1, 0, "OFFSET_SF", 1, 0),
  LINCOM2("VETO_SF", "WAIT_SF", 1, 0, "AGE_SF", -1, 0),

  LINCOM2("Pos_0_act", "POS_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
  LINCOM2("Pos_1_act", "POS_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
  LINCOM2("Pos_2_act", "POS_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),
  LINCOM2("Enc_0_act", "ENC_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
  LINCOM2("Enc_1_act", "ENC_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
  LINCOM2("Enc_2_act", "ENC_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),
  LINCOM2("Lvdt_0_act", "LVDT_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
  LINCOM2("Lvdt_1_act", "LVDT_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
  LINCOM2("Lvdt_2_act", "LVDT_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),
  LINCOM2("Goal_0_act", "GOAL_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
  LINCOM2("Goal_1_act", "GOAL_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
  LINCOM2("Goal_2_act", "GOAL_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),

  //TODO flags_act will probably change
  BITFIELD("flags_act",
      "LOST_ACT",
      "DR_POS_BAD_ACT",
      "DR_ENC_BAD_ACT",
      "ENC_POS_BAD_ACT",
      "DR_LVDT_BAD_ACT",
      "ENC_LVDT_BAD_ACT",
      "POS_LVDT_BAD_ACT",
      "BAD_MOVE_ACT",
      "FAULT_0_ACT",
      "FAULT_1_ACT",
      "FAULT_2_ACT"
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
  BITFIELD("dig43_das",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "",
      "HELIUM_LEVEL_HEAT",
      "CHARCOAL_HEAT",
      "CHARCOAL_HS_HEAT",
      "POT_HS_HEAT",
      "JFET_HEAT",
      "BDA_HEAT",
      "CALIBRATOR_HEAT",
      "HWPR_POS_HEAT"),

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

  COMMENT("\"Named\" Bolometers"),
  
  PHASE("B2D06H", "N19C00", 0),
  PHASE("B2B06H", "N19C01", 0),
  PHASE("B2C05V", "N19C02", 0),
  PHASE("B2A05V", "N19C03", 0),
  PHASE("B2E05V", "N19C04", 0),
  PHASE("B2B05V", "N19C05", 0),
  PHASE("B2D05V", "N19C06", 0),
  PHASE("B2C04H", "N19C07", 0),
  PHASE("B2A04H", "N19C08", 0),
  PHASE("B2D04H", "N19C09", 0),
  PHASE("X2B04H", "N19C10", 0),
  PHASE("B2C03V", "N19C11", 0),
  PHASE("B2B03V", "N19C12", 0),
  PHASE("B2A03V", "N19C13", 0),
  PHASE("B2A02H", "N19C14", 0),
  PHASE("B2D03V", "N19C15", 0),
  PHASE("B2C02H", "N19C16", 0),
  PHASE("B2B02H", "N19C17", 0),
  PHASE("B2D02H", "N19C18", 0),
  PHASE("B2A01V", "N19C19", 0),
  PHASE("B2C01V", "N19C20", 0),
  PHASE("B2B01V", "N19C21", 0),
  PHASE("B2D1",   "N19C22", 0),
  PHASE("B2D01V", "N19C23", 0),
  PHASE("B2R1",   "N22C00", 0), 
  PHASE("B2D16H", "N22C01", 0),
  PHASE("B2T1",   "N22C02", 0), 
  PHASE("B2B16H", "N22C03", 0),
  PHASE("B2C15V", "N22C04", 0),
  PHASE("B2A15V", "N22C05", 0),
  PHASE("B2D15V", "N22C06", 0),
  PHASE("B2B15V", "N22C07", 0),
  PHASE("B2C14H", "N22C08", 0),
  PHASE("B2D14H", "N22C09", 0),
  PHASE("B2A14H", "N22C10", 0),
  PHASE("B2A13V", "N22C11", 0),
  PHASE("B2B14H", "N22C12", 0),
  PHASE("B2C13V", "N22C13", 0),
  PHASE("B2B13V", "N22C14", 0),
  PHASE("B2D13V", "N22C15", 0),
  PHASE("B2A12H", "N22C16", 0),
  PHASE("B2C12H", "N22C17", 0),
  PHASE("B2D12H", "N22C18", 0),
  PHASE("B2B12H", "N22C19", 0),
  PHASE("B2E11V", "N22C20", 0),
  PHASE("B2A11V", "N22C21", 0),
  PHASE("B2C11V", "N22C22", 0),
  PHASE("B2B11V", "N22C23", 0),
  PHASE("B2D11V", "N23C00", 0),
  PHASE("B2A10H", "N23C01", 0),
  PHASE("B2E10H", "N23C02", 0),
  PHASE("B2C10H", "N23C03", 0),
  PHASE("B2B10H", "N23C04", 0),
  PHASE("B2D10H", "N23C05", 0),
  PHASE("B2A09V", "N23C06", 0),
  PHASE("B2E09V", "N23C07", 0),
  PHASE("B2C09V", "N23C08", 0),
  PHASE("B2B09V", "N23C09", 0),
  PHASE("B2D09V", "N23C10", 0),
  PHASE("B2A08H", "N23C11", 0),
  PHASE("B2C08H", "N23C12", 0),
  PHASE("B2E08H", "N23C13", 0),
  PHASE("B2D08H", "N23C14", 0),
  PHASE("B2B08H", "N23C15", 0),
  PHASE("B2C07V", "N23C16", 0),
  PHASE("B2E07V", "N23C17", 0),
  PHASE("B2A07V", "N23C18", 0),
  PHASE("B2D07V", "N23C19", 0),
  PHASE("B2B07V", "N23C20", 0),
  PHASE("B2C06H", "N23C21", 0),
  PHASE("B2E06H", "N23C22", 0),
  PHASE("B2A06H", "N23C23", 0),
  PHASE("B2F12H", "N17C00", 0),
  PHASE("B2J11V", "N17C01", 0),
  PHASE("B2E12H", "N17C02", 0),
  PHASE("B2H12H", "N17C03", 0),
  PHASE("B2G12H", "N17C04", 0),
  PHASE("B2F13V", "N17C05", 0),
  PHASE("B2E13V", "N17C06", 0),
  PHASE("B2J12H", "N17C07", 0),
  PHASE("B2H13V", "N17C08", 0),
  PHASE("B2G13V", "N17C09", 0),
  PHASE("B2F14H", "N17C10", 0),
  PHASE("B2E14H", "N17C11", 0),
  PHASE("B2J13V", "N17C12", 0),
  PHASE("B2H14H", "N17C13", 0),
  PHASE("B2G14H", "N17C14", 0),
  PHASE("B2J14H", "N17C15", 0),
  PHASE("B2F15V", "N17C16", 0),
  PHASE("B2H15V", "N17C17", 0),
  PHASE("B2J15V", "N17C18", 0),
  PHASE("B2G15V", "N17C19", 0),
  PHASE("B2H16H", "N17C20", 0),
  PHASE("B2D2",   "N17C21", 0),
  PHASE("B2F16H", "N17C22", 0),
  PHASE("B2E15V", "N17C23", 0),
  PHASE("B2E01V", "N18C00", 0),
  PHASE("B2F01V", "N18C01", 0),
  PHASE("B2T2",   "N18C02", 0), 
  PHASE("B2H01V", "N18C03", 0),
  PHASE("B2G01V", "N18C04", 0),
  PHASE("B2J01V", "N18C05", 0),
  PHASE("B2H02H", "N18C06", 0),
  PHASE("B2F02H", "N18C07", 0),
  PHASE("B2J02H", "N18C08", 0),
  PHASE("B2G02H", "N18C09", 0),
  PHASE("B2H03V", "N18C10", 0),
  PHASE("B2J03V", "N18C11", 0),
  PHASE("B2E02H", "N18C12", 0),
  PHASE("B2F03V", "N18C13", 0),
  PHASE("B2G03V", "N18C14", 0),
  PHASE("B2H04H", "N18C15", 0),
  PHASE("B2J04H", "N18C16", 0),
  PHASE("B2E03V", "N18C17", 0),
  PHASE("B2F04H", "N18C18", 0),
  PHASE("B2G04H", "N18C19", 0),
  PHASE("B2H05V", "N18C20", 0),
  PHASE("B2E04H", "N18C21", 0),
  PHASE("B2J05V", "N18C22", 0),
  PHASE("B2F05V", "N18C23", 0),
  PHASE("B2G05V", "N21C00", 0),
  PHASE("B2H06H", "N21C01", 0),
  PHASE("B2J06H", "N21C02", 0),
  PHASE("B2F06H", "N21C03", 0),
  PHASE("B2G06H", "N21C04", 0), 
  PHASE("B2H07V", "N21C05", 0),
  PHASE("B2F07V", "N21C06", 0),
  PHASE("B2J07V", "N21C07", 0),
  PHASE("B2G07V", "N21C08", 0),
  PHASE("B2H08H", "N21C09", 0),
  PHASE("B2F08H", "N21C10", 0),
  PHASE("B2G08H", "N21C11", 0),
  PHASE("B2J08H", "N21C12", 0),
  PHASE("B2F09V", "N21C13", 0),
  PHASE("B2H09V", "N21C14", 0),
  PHASE("B2G09V", "N21C15", 0),
  PHASE("B2J09V", "N21C16", 0),
  PHASE("B2F10H", "N21C17", 0),
  PHASE("B2H10H", "N21C18", 0),
  PHASE("B2G10H", "N21C19", 0),
  PHASE("B2F11V", "N21C20", 0),
  PHASE("B2J10H", "N21C21", 0),
  PHASE("B2H11V", "N21C22", 0),
  PHASE("B2G11V", "N21C23", 0),
  PHASE("B3A07V", "N27C00", 0),
  PHASE("B3A06H", "N27C01", 0),
  PHASE("B3B06V", "N27C02", 0),
  PHASE("B3C07V", "N27C03", 0),
  PHASE("B3A05V", "N27C04", 0),
  PHASE("B3B05H", "N27C05", 0),
  PHASE("B3C06H", "N27C06", 0),
  PHASE("B3D06V", "N27C07", 0),
  PHASE("B3B04V", "N27C08", 0),
  PHASE("B3C05V", "N27C09", 0),
  PHASE("B3D04V", "N27C10", 0),
  PHASE("B3A04H", "N27C11", 0),
  PHASE("B3C04H", "N27C12", 0),
  PHASE("B3B03H", "N27C13", 0),
  PHASE("B3C03V", "N27C14", 0),
  PHASE("B3B02V", "N27C15", 0),
  PHASE("B3D02V", "N27C16", 0),
  PHASE("B3A03V", "N27C17", 0),
  PHASE("B3A02H", "N27C18", 0),
  PHASE("B3C02H", "N27C19", 0),
  PHASE("B3B01H", "N27C20", 0),
  PHASE("B3A01V", "N27C21", 0),
  PHASE("B3D1",   "N27C22", 0),
  PHASE("X3C01V", "N27C23", 0),
  PHASE("B3E07V", "N25C00", 0),
  PHASE("B3D07H", "N25C01", 0),
  PHASE("B3F07H", "N25C02", 0),
  PHASE("B3E08H", "N25C03", 0),
  PHASE("B3G08H", "N25C04", 0),
  PHASE("B3F08V", "N25C05", 0),
  PHASE("B3E09V", "N25C06", 0),
  PHASE("B3G09V", "N25C07", 0),
  PHASE("B3D09H", "N25C08", 0),
  PHASE("B3F09H", "N25C09", 0),
  PHASE("B3E10H", "N25C10", 0),
  PHASE("B3G10H", "N25C11", 0),
  PHASE("X3O1",   "N25C12", 0),
  PHASE("B3F10V", "N25C13", 0),
  PHASE("B3E11V", "N25C14", 0),
  PHASE("B3G11V", "N25C15", 0),
  PHASE("B3F11H", "N25C16", 0),
  PHASE("B3E12H", "N25C17", 0),
  PHASE("B3G12H", "N25C18", 0),
  PHASE("B3F12V", "N25C19", 0),
  PHASE("B3G13V", "N25C20", 0),
  PHASE("B3D2",   "N25C21", 0),
  PHASE("X3O2",   "N25C22", 0),
  PHASE("B3R2",   "N25C23", 0),
  PHASE("B3A13V", "N30C00", 0),
  PHASE("B3T2",   "N30C01", 0), 
  PHASE("B3B12V", "N30C02", 0),
  PHASE("B3C13V", "N30C03", 0),
  PHASE("B3A12H", "N30C04", 0),
  PHASE("B3D12V", "N30C05", 0),
  PHASE("B3C12H", "N30C06", 0),
  PHASE("B3B11H", "N30C07", 0),
  PHASE("B3A11V", "N30C08", 0),
  PHASE("B3E13V", "N30C09", 0),
  PHASE("B3D11H", "N30C10", 0),
  PHASE("B3C11V", "N30C11", 0),
  PHASE("B3B10V", "N30C12", 0),
  PHASE("B3A10H", "N30C13", 0),
  PHASE("X3D10V", "N30C14", 0),
  PHASE("B3B09H", "N30C15", 0),
  PHASE("B3C10H", "N30C16", 0),
  PHASE("B3C09V", "N30C17", 0),
  PHASE("B3A09V", "N30C18", 0),
  PHASE("B3B08V", "N30C19", 0),
  PHASE("B3A08H", "N30C20", 0),
  PHASE("B3D08V", "N30C21", 0),
  PHASE("B3C08H", "N30C22", 0),
  PHASE("X3B07H", "N30C23", 0),
  PHASE("B3R1",   "N26C00", 0), 
  PHASE("X3G01V", "N26C01", 0),
  PHASE("B3T1",   "N26C02", 0),
  PHASE("B3E01V", "N26C03", 0),
  PHASE("B3D01H", "N26C04", 0),
  PHASE("B3F01H", "N26C05", 0),
  PHASE("B3E02H", "N26C06", 0),
  PHASE("X3G02H", "N26C07", 0), 
  PHASE("B3F02V", "N26C08", 0),
  PHASE("B3G03V", "N26C09", 0),
  PHASE("B3E03V", "N26C10", 0),
  PHASE("X3D03H", "N26C11", 0),
  PHASE("B3F03H", "N26C12", 0),
  PHASE("B3G04H", "N26C13", 0),
  PHASE("B3E04H", "N26C14", 0),
  PHASE("B3F04V", "N26C15", 0),
  PHASE("B3E05V", "N26C16", 0),
  PHASE("B3D05H", "N26C17", 0),
  PHASE("B3F05H", "N26C18", 0),
  PHASE("B3G05V", "N26C19", 0),
  PHASE("B3E06H", "N26C20", 0),
  PHASE("B3G06H", "N26C21", 0),
  PHASE("B3F06V", "N26C22", 0),
  PHASE("B3G07V", "N26C23", 0),
  PHASE("B5R1",   "N31C00", 0),
  PHASE("B5A08H", "N31C01", 0),
  PHASE("B5A07V", "N31C02", 0),
  PHASE("B5A06H", "N31C03", 0),
  PHASE("B5A09V", "N31C04", 0),
  PHASE("B5C09V", "N31C05", 0),
  PHASE("B5B08H", "N31C06", 0),
  PHASE("B5B07V", "N31C07", 0),
  PHASE("B5C07V", "N31C08", 0),
  PHASE("B5B05V", "N31C09", 0),
  PHASE("B5B06H", "N31C10", 0),
  PHASE("B5A05V", "N31C11", 0),
  PHASE("B5T1",   "N31C12", 0),
  PHASE("B5B04H", "N31C13", 0),
  PHASE("B5C04H", "N31C14", 0),
  PHASE("B5B03V", "N31C15", 0),
  PHASE("B5C02H", "N31C16", 0),
  PHASE("B5B02H", "N31C17", 0),
  PHASE("B5B01V", "N31C18", 0),
  PHASE("B5A03V", "N31C19", 0),
  PHASE("B5A04H", "N31C20", 0),
  PHASE("B5A01V", "N31C21", 0),
  PHASE("B5D1",   "N31C22", 0),
  PHASE("B5A02H", "N31C23", 0),
  PHASE("B5E01V", "N29C00", 0),
  PHASE("B5E02H", "N29C01", 0),
  PHASE("B5E03V", "N29C02", 0),
  PHASE("B5E04H", "N29C03", 0),
  PHASE("B5D01V", "N29C04", 0),
  PHASE("X5D02H", "N29C05", 0),
  PHASE("B5D03V", "N29C06", 0),
  PHASE("B5D04H", "N29C07", 0),
  PHASE("B5C01V", "N29C08", 0),
  PHASE("B5C03V", "N29C09", 0),
  PHASE("B5C05V", "N29C10", 0),
  PHASE("B5T2",   "N29C11", 0),
  PHASE("B5E05V", "N29C12", 0),
  PHASE("B5C06H", "N29C13", 0),
  PHASE("B5C08H", "N29C14", 0),
  PHASE("B5D05V", "N29C15", 0),
  PHASE("B5D06H", "N29C16", 0),
  PHASE("B5D07V", "N29C17", 0),
  PHASE("B5D08H", "N29C18", 0),
  PHASE("B5E07V", "N29C19", 0),
  PHASE("B5E06H", "N29C20", 0),
  PHASE("B5E08H", "N29C21", 0),
  PHASE("X5D2",   "N29C22", 0),
  PHASE("B5E09V", "N29C23", 0),

  END_OF_DERIVED_CHANNELS
};
