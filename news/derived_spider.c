/* derived.c: a list of derived channels
 *
 * This software is copyright (C) 2002-20010 University of Toronto
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
      "UPLINK_SCHED",
      "BLAST_SUCKS"),
  BITWORD("SCHEDULE", "status_mcc", 4, 3),
  BITWORD("SLOT_SCHED", "status_mcc", 8, 8),
  BITWORD("STATUS_SUN_ETH", "status_eth", 0, 2),
  BITWORD("STATUS_ISC_ETH", "status_eth", 2, 2),
  BITWORD("STATUS_OSC_ETH", "status_eth", 4, 2),
  BITWORD("STATUS_SBSC_ETH", "status_eth", 6, 2),

#ifndef BOLOTEST
  COMMENT("Pointing Stuff"),
  LINCOM("X_H_P", "x_p", 0.0003662109375, 0),
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
      "FORCE_EL",
      "VETO_PSS1",
      "VETO_PSS2"
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

  BITFIELD("pulse_sc",
      "PULSE_ISC",
      "PULSE_OSC",
      "PULSE_SBSC"
      ),

  COMMENT("ACS Digital Signals"),

  BITFIELD("bits_bal",
      "DIR_BAL",
      "VALVE_BAL",
      "HEAT_BAL",
      ),

  /* charge controller (CC) faults and alarms */

  COMMENT("Charge Controller Bitfields"),

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
  LINCOM2("Dr_0_act", "DR_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
  LINCOM2("Dr_1_act", "DR_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
  LINCOM2("Dr_2_act", "DR_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),

  BITFIELD("flags_act",
      "TRIM_WAIT_ACT",
      "TRIMMED_ACT",
      "BUSY_0_ACT",
      "BUSY_1_ACT",
      "BUSY_2_ACT",
      "BAD_MOVE_ACT"
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

  /* HWPR Control Info */
  BITFIELD("stat_control_hwpr"
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
	   "DO_OVERSHOOT_HWPR",
	   "DONE_MOVE_HWPR",
	   "DONE_ALL_HWPR",
	   "",
	   "",
	   "",
	   ),
  BITWORD("MOVE_TYPE_HWPR","stat_control_hwpr",0,3),
  BITWORD("MOVE_STAT_HWPR","stat_control_hwpr",3,3),
  BITWORD("READ_BEFORE_HWPR","stat_control_hwpr",6,2),
  BITWORD("READ_AFTER_HWPR","stat_control_hwpr",8,2),
  /* CRYO */

  COMMENT("Internal (to mcp) reference of the cryo stat"),
  BITFIELD("cryostate",
      "",
      "",
      "",
      "",
      "POT_VALVE",
      "POT_DIREC",
      "LHE_VALVE",
      "CRYO_DIREC",
      "LN_VALVE",
      "AUTO_JFET_HEAT"
      ),

#if 0
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
#endif

  COMMENT("Cryo Valve Limit Switches"),

  BITFIELD("dig65_das",
      "LHE_IS_OPEN",
      "LHE_IS_CLOSED",
      "LN_IS_OPEN",
      "LN_IS_CLOSED",
      "POT_IS_OPEN",
      "POT_IS_CLOSED"),

  LINCOM2("POT_STATE", "POT_IS_CLOSED", 2, 0, "POT_IS_OPEN",  1, 0),
  LINCOM2("LHE_STATE", "LHE_IS_CLOSED", 2, 0, "LHE_IS_OPEN",  1, 0),
  LINCOM2("LN_STATE",  "LN_IS_CLOSED", 2, 0, "LN_IS_OPEN", 1, 0),

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
      "POT_HS_HEAT",
      "CHARCOAL_HS_HEAT",
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
  
  END_OF_DERIVED_CHANNELS
};
