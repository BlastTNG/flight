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
 * o BITS: A bit channel extracted from a larger source channel
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  First bit of the bitword, numbered from zero (integer)
 *   4.  Length of the bitword (integer)
 * o COMMENT: A literal comment to be inserted into the format file
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

#define LUT_DIR "/data/etc/blast/"

derived_tng_t derived_list[] = {
  /* Pointing */
  COMMENT("Microsecond Resolution Time"),
  LINCOM2("Time", "time_usec", 1.0E-6, 0, "time",  1, 0),
  UNITS("Time", "Time", "s"),

  COMMENT("General BLAST Status"),
  BITWORD("SOUTH_I_AM","status_mcc",0,1),
  BITWORD("AT_FLOAT","status_mcc",1,1),
  BITWORD("UPLINK_SCHED","status_mcc",2,1),
  BITWORD("BLAST_SUCKS","status_mcc",3,1),
  BITWORD("SCHEDULE", "status_mcc", 4, 3),
  BITWORD("SLOT_SCHED", "status_mcc", 8, 8),

  //first two bits used to be sun sensor
  BITWORD("STATUS_ISC_ETH", "status_eth", 2, 2),
  BITWORD("STATUS_OSC_ETH", "status_eth", 4, 2),
  BITWORD("STATUS_SBSC_ETH", "status_eth", 6, 2),

#ifndef BOLOTEST
  COMMENT("Pointing Stuff"),
  LINCOM("X_H_P", "x_p", 0.0003662109375, 0),

  BITWORD("VETO_ISC","veto_sensor",1,1),
  BITWORD("VETO_EL_ENC","veto_sensor",2,1),
  BITWORD("VETO_MAG","veto_sensor",3,1),
  BITWORD("VETO_GPS","veto_sensor",4,1),
  BITWORD("VETO_EL_CLIN","veto_sensor",5,1),
  BITWORD("VETO_OSC","veto_sensor",6,1),
  BITWORD("IS_SCHED","veto_sensor",7,1),
  BITWORD("AZ_AUTO_GYRO","veto_sensor",8,1),
  BITWORD("EL_AUTO_GYRO","veto_sensor",9,1),
  BITWORD("DISABLE_EL","veto_sensor",10,1),
  BITWORD("DISABLE_AZ","veto_sensor",11,1),
  BITWORD("FORCE_EL","veto_sensor",12,1),
  BITWORD("VETO_PSS","veto_sensor",13,1),

  /* ISC and OSC */
  COMMENT("Star Camera State"),

    BITWORD("SAVE_IMAGES_ISC","state_isc",0,1),
    BITWORD( "PAUSE_ISC","state_isc",1,1),
    BITWORD("ABORT_ISC","state_isc",2,1),
    BITWORD( "AUTOFOCUS_REQ_ISC","state_isc",3,1),
    BITWORD("SHUTDOWN_ISC","state_isc",4,1),
    BITWORD("REBOOT_ISC","state_isc",5,1),
    BITWORD("EYE_ON_ISC","state_isc",6,1),
    BITWORD("HEATER_ON_ISC","state_isc",7,1),
    BITWORD("USE_LOST_ISC","state_isc",8,1),
    BITWORD("AUTOFOCUS_ISC","state_isc",9,1),

  BITWORD("SENT_TRIG_ISC", "trigger_isc", 0, 14),

  BITWORD("SAVE_IMAGES_OSC","state_osc",0,1),
  BITWORD("PAUSE_OSC","state_osc",0,1),
  BITWORD("ABORT_OSC","state_osc",0,1),
  BITWORD("AUTOFOCUS_REQ_OSC","state_osc",0,1),
  BITWORD("SHUTDOWN_OSC","state_osc",0,1),
  BITWORD("REBOOT_OSC","state_osc",0,1),
  BITWORD("EYE_ON_OSC","state_osc",0,1),
  BITWORD("HEATER_ON_OSC","state_osc",0,1),
  BITWORD("USE_LOST_OSC","state_osc",0,1),
  BITWORD("AUTOFOCUS_OSC","state_osc",0,1),

  BITWORD("SENT_TRIG_OSC", "trigger_osc", 0, 14),

  BITWORD("PULSE_ISC","pulse_sc",0,1),
  BITWORD("PULSE_OSC","pulse_sc",1,1),
  BITWORD("PULSE_SBSC","pulse_sc",2,1),

  COMMENT("ACS Digital Signals"),

  /* charge controller (CC) faults and alarms */
//TODO: Convert BITFIELD to BITWORD on Charge Controllers
//  COMMENT("Charge Controller Bitfields"),

//  BITFIELD("fault_cc1",
//      "F_OVERCURRENT_CC1",
//      "F_FET_SHORT_CC1",
//      "F_SOFTWARE_BUG_CC1",
//      "F_BATT_HVD_CC1",
//      "F_ARR_HVD_CC1",
//      "F_DIP_CHANGED_CC1",
//      "F_SETTING_CHANGE_CC1",
//      "F_RTS_SHORT_CC1",
//      "F_RTS_DISCONN_CC1",
//      "F_EEPROM_LIM_CC1",
//      "F_SLAVE_TO_CC1"
//      ),
//
//  BITFIELD("fault_cc2",
//      "F_OVERCURRENT_CC2",
//      "F_FET_SHORT_CC2",
//      "F_SOFTWARE_BUG_CC2",
//      "F_BATT_HVD_CC2",
//      "F_ARR_HVD_CC2",
//      "F_DIP_CHANGED_CC2",
//      "F_SETTING_CHANGE_CC2",
//      "F_RTS_SHORT_CC2",
//      "F_RTS_DISCONN_CC2",
//      "F_EEPROM_LIM_CC2",
//      "F_SLAVE_TO_CC2"
//      ),
//
//  BITFIELD("alarm_lo_cc1",
//      "A_RTS_OPEN_CC1",
//      "A_RTS_SHORT_CC1",
//      "A_RTS_DISCONN_CC1",
//      "A_TSENSE_OPEN_CC1",
//      "A_TSENSE_SHORT_CC1",
//      "A_HITEMP_LIM_CC1",
//      "A_CURRENT_LIM_CC1",
//      "A_CURRENT_OFFSET_CC1",
//      "A_BATT_RANGE_CC1",
//      "A_BATTSENSE_DISC_CC1",
//      "A_UNCALIB_CC1",
//      "A_RTS_MISWIRE_CC1",
//      "A_HVD_CC1",
//      "",
//      "A_SYS_MISWIRE_CC1",
//      "A_FET_OPEN_CC1",
//      ),
//
//   BITFIELD("alarm_lo_cc2",
//      "A_RTS_OPEN_CC2",
//      "A_RTS_SHORT_CC2",
//      "A_RTS_DISCONN_CC2",
//      "A_TSENSE_OPEN_CC2",
//      "A_TSENSE_SHORT_CC2",
//      "A_HITEMP_LIM_CC2",
//      "A_CURRENT_LIM_CC2",
//      "A_CURRENT_OFFSET_CC2",
//      "A_BATT_RANGE_CC2",
//      "A_BATTSENSE_DISC_CC2",
//      "A_UNCALIB_CC2",
//      "A_RTS_MISWIRE_CC2",
//      "A_HVD_CC2",
//      "",
//      "A_SYS_MISWIRE_CC2",
//      "A_FET_OPEN_CC2",
//      ),
//
// BITFIELD("alarm_hi_cc1",
//      "A_VP12_OFF_CC1",
//      "C_A_HI_INPUT_LIM_CC1",
//      "C_A_ADC_MAX_IN_CC1",
//      "C_A_RESET_CC1",
//      ),
//
// BITFIELD("alarm_hi_cc2",
//      "A_VP12_OFF_CC2",
//      "C_A_HI_INPUT_LIM_CC2",
//      "C_A_ADC_MAX_IN_CC2",
//      "C_A_RESET_CC2",
//      ),

      #endif

  /*GONDOLA THERMISTOR CALIBRATION */
#define THERMISTOR(tch, vch) \
    LINTERP(tch, vch, LUT_DIR "thermistor.lut"), \
    UNITS(tch, "Temperature", "^oC")

  COMMENT("Thermistor calibrations"),
  /*Misc Thermometers Outer Frame cable*/
  THERMISTOR("T_PORT_HEXC", "VT_PORT_HEXC"),
  THERMISTOR("T_RW", "VT_RW"),
  THERMISTOR("T_PORT_PYR", "VT_PORT_PYR"),
  THERMISTOR("T_PORT_BACK", "VT_PORT_BACK"),
  THERMISTOR("T_EARTH", "VT_EARTH"),
  THERMISTOR("T_EL", "VT_EL"),
  THERMISTOR("T_1_BAT", "VT_1_BAT"),
  THERMISTOR("T_2_BAT", "VT_2_BAT"),
  THERMISTOR("T_CHIN", "VT_CHIN"),
  THERMISTOR("T_SUN", "VT_SUN"),
  THERMISTOR("T_ARRAY", "VT_ARRAY"),
  THERMISTOR("T_STAR_FRONT", "VT_STAR_FRONT"),
  /*T DAS Cable*/
  THERMISTOR("T_STBD_DAS", "VT_STBD_DAS"),
  THERMISTOR("T_STBD_REC", "VT_STBD_REC"),
  THERMISTOR("T_PORT_DAS", "VT_PORT_DAS"),
  THERMISTOR("T_PORT_REC", "VT_PORT_REC"),
  /*T BETH Cable*/
  THERMISTOR("T_MOT_PUMP_VALVE", "VT_MOT_PUMP_VAL"),
  THERMISTOR("T_1_PRIME", "VT_1_PRIME"),
  THERMISTOR("T_IF_TOP_BACK", "VT_IF_TOP_BACK"),
  THERMISTOR("T_HWPR_MOT", "VT_HWPR_MOT"),
  THERMISTOR("T_IF_TOP_FRNT", "VT_IF_TOP_FRNT"),
  THERMISTOR("T_HWPR_FEED", "VT_HWPR_FEED"),
  THERMISTOR("T_IF_BOT_FRNT", "VT_IF_BOT_FRNT"),
  /*T ISREAL Cable*/
  THERMISTOR("T_STRUT_BOT", "VT_STRUT_BOT"),
  THERMISTOR("T_STRUT_SIDE", "VT_STRUT_SIDE"),
  THERMISTOR("T_2_PRIME", "VT_2_PRIME"),
  THERMISTOR("T_IF_BOT_BACK", "VT_IF_BOT_BACK"),
  THERMISTOR("T_DAC_BOX", "VT_DAC_BOX"),

/*Misc T*/
  THERMISTOR("T_PUMP_BAL", "VT_PUMP_BAL"),
  THERMISTOR("T_MC_LOCK", "VT_MC_LOCK"),
  THERMISTOR("T_SBSC", "VT_SBSC"), 

  //TODO: Convert BITFIELD to BITWORD on Lock Motor
//  COMMENT("Lock Motor/Actuators"),
//  BITFIELD("state_lock",
//      "LS_OPEN_LOCK",
//      "LS_CLOSED_LOCK",
//      "LS_DRIVE_OFF_LOCK",
//      "LS_POT_RAIL_LOCK",
//      "LS_DRIVE_EXT_LOCK",
//      "LS_DRIVE_RET_LOCK",
//      "LS_DRIVE_STP_LOCK",
//      "LS_DRIVE_JIG_LOCK",
//      "LS_DRIVE_UNK_LOCK",
//      "LS_EL_OK_LOCK",
//      "LS_IGNORE_EL_LOCK",
//      "LS_DRIVE_FORCE_LOCK"
//      ),

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

    BITWORD("TRIM_WAIT_ACT","flags_act",0,1),
    BITWORD("TRIMMED_ACT","flags_act",1,1),
    BITWORD("BUSY_0_ACT","flags_act",2,1),
    BITWORD("BUSY_1_ACT","flags_act",3,1),
    BITWORD("BUSY_2_ACT","flags_act",4,1),
    BITWORD("BAD_MOVE_ACT","flags_act",5,1),

    BITWORD("BISS_CRC_ERR_EL", "mc_el_biss_status",0,1),
    BITWORD("BISS_NO_DATA_EL", "mc_el_biss_status",1,1),
    BITWORD("BISS_ERR_BIT_EL", "mc_el_biss_status",2,1),
    BITWORD("BISS_WARN_BIT_EL", "mc_el_biss_status",3,1),
    BITWORD("BISS_ENC_DELAY_EL", "mc_el_biss_status",4,1),

    BITWORD("LATCHED_DATA_CRC_EL", "latched_fault_el", 0, 1),
    BITWORD("LATCHED_AMP_INT_EL", "latched_fault_el", 1, 1),
    BITWORD("LATCHED_SHORT_CIRC_EL", "latched_fault_el", 2, 1),
    BITWORD("LATCHED_AMP_OVER_T_EL", "latched_fault_el", 3, 1),
    BITWORD("LATCHED_MOTOR_OVER_T_EL", "latched_fault_el", 4, 1),
    BITWORD("LATCHED_OVERVOLT_EL", "latched_fault_el", 5, 1),
    BITWORD("LATCHED_UNDERVOLT_EL", "latched_fault_el", 6, 1),
    BITWORD("LATCHED_FEEDBACK_EL", "latched_fault_el", 7, 1),
    BITWORD("LATCHED_PHASING_ERR_EL", "latched_fault_el", 8, 1),
    BITWORD("LATCHED_TRACKING_ERR_EL", "latched_fault_el", 9, 1),
    BITWORD("LATCHED_OVERCURRENT_EL", "latched_fault_el", 10, 1),
    BITWORD("LATCHED_FPGA_FAIL_EL", "latched_fault_el", 11, 1),
    BITWORD("LATCHED_CMD_INPUT_LOST_EL", "latched_fault_el", 12, 1),
    BITWORD("LATCHED_FPGA_FAIL2_EL", "latched_fault_el", 13, 1),
    BITWORD("LATCHED_SAFETY_CIRC_EL", "latched_fault_el", 14, 1),
    BITWORD("LATCHED_CONT_CURRENT_EL", "latched_fault_el", 15, 1),

    BITWORD("LATCHED_DATA_CRC_PIV", "latched_fault_piv", 0, 1),
    BITWORD("LATCHED_AMP_INT_PIV", "latched_fault_piv", 1, 1),
    BITWORD("LATCHED_SHORT_CIRC_PIV", "latched_fault_piv", 2, 1),
    BITWORD("LATCHED_AMP_OVER_T_PIV", "latched_fault_piv", 3, 1),
    BITWORD("LATCHED_MOTOR_OVER_T_PIV", "latched_fault_piv", 4, 1),
    BITWORD("LATCHED_OVERVOLT_PIV", "latched_fault_piv", 5, 1),
    BITWORD("LATCHED_UNDERVOLT_PIV", "latched_fault_piv", 6, 1),
    BITWORD("LATCHED_FEEDBACK_PIV", "latched_fault_piv", 7, 1),
    BITWORD("LATCHED_PHASING_ERR_PIV", "latched_fault_piv", 8, 1),
    BITWORD("LATCHED_TRACKING_ERR_PIV", "latched_fault_piv", 9, 1),
    BITWORD("LATCHED_OVERCURRENT_PIV", "latched_fault_piv", 10, 1),
    BITWORD("LATCHED_FPGA_FAIL_PIV", "latched_fault_piv", 11, 1),
    BITWORD("LATCHED_CMD_INPUT_LOST_PIV", "latched_fault_piv", 12, 1),
    BITWORD("LATCHED_FPGA_FAIL2_PIV", "latched_fault_piv", 13, 1),
    BITWORD("LATCHED_SAFETY_CIRC_PIV", "latched_fault_piv", 14, 1),
    BITWORD("LATCHED_CONT_CURRENT_PIV", "latched_fault_piv", 15, 1),

    BITWORD("LATCHED_DATA_CRC_RW", "latched_fault_rw", 0, 1),
    BITWORD("LATCHED_AMP_INT_RW", "latched_fault_rw", 1, 1),
    BITWORD("LATCHED_SHORT_CIRC_RW", "latched_fault_rw", 2, 1),
    BITWORD("LATCHED_AMP_OVER_T_RW", "latched_fault_rw", 3, 1),
    BITWORD("LATCHED_MOTOR_OVER_T_RW", "latched_fault_rw", 4, 1),
    BITWORD("LATCHED_OVERVOLT_RW", "latched_fault_rw", 5, 1),
    BITWORD("LATCHED_UNDERVOLT_RW", "latched_fault_rw", 6, 1),
    BITWORD("LATCHED_FEEDBACK_RW", "latched_fault_rw", 7, 1),
    BITWORD("LATCHED_PHASING_ERR_RW", "latched_fault_rw", 8, 1),
    BITWORD("LATCHED_TRACKING_ERR_RW", "latched_fault_rw", 9, 1),
    BITWORD("LATCHED_OVERCURRENT_RW", "latched_fault_rw", 10, 1),
    BITWORD("LATCHED_FPGA_FAIL_RW", "latched_fault_rw", 11, 1),
    BITWORD("LATCHED_CMD_INPUT_LOST_RW", "latched_fault_rw", 12, 1),
    BITWORD("LATCHED_FPGA_FAIL2_RW", "latched_fault_rw", 13, 1),
    BITWORD("LATCHED_SAFETY_CIRC_RW", "latched_fault_rw", 14, 1),
    BITWORD("LATCHED_CONT_CURRENT_RW", "latched_fault_rw", 15, 1),

    BITWORD("ST_SHORT_CIRC_EL","status_el",0,1),
    BITWORD("ST_AMP_OVER_T_EL","status_el",1,1),
    BITWORD("ST_OVER_V_EL","status_el",2,1),
    BITWORD("ST_UNDER_V_EL","status_el",3,1),

    BITWORD("ST_FEEDBACK_ERR_EL","status_el",5,1),
    BITWORD("ST_MOT_PHAS_ERR_EL","status_el",6,1),
    BITWORD("ST_I_LIMITED_EL","status_el",7,1),
    BITWORD("ST_VOLT_LIM_EL","status_el",8,1),
    BITWORD("ST_LIM_SW_POS_EL","status_el",9,1),
    BITWORD("ST_LIM_SW_NEG_EL","status_el",10,1),

    BITWORD("ST_DISAB_HWARE_EL","status_el",11,1),
    BITWORD("ST_DISAB_SWARE_EL","status_el",12,1),
    BITWORD("ST_ATTEMPT_STOP_EL","status_el",13,1),
    BITWORD("ST_MOT_BREAK_ACT_EL","status_el",14,1),
    BITWORD("ST_PWM_OUT_DIS_EL","status_el",15,1),
    BITWORD("ST_POS_SOFT_LIM_EL","status_el",16,1),
    BITWORD("ST_NEG_SOFT_LIM_EL","status_el",17,1),
    BITWORD("ST_FOLLOW_ERR_EL","status_el",18,1),
    BITWORD("ST_FOLLOW_WARN_EL","status_el",19,1),
    BITWORD("ST_AMP_HAS_RESET_EL","status_el",20,1),
    BITWORD("ST_ENCODER_WRAP_EL","status_el",21,1),
    BITWORD("ST_AMP_FAULT_EL","status_el",22,1),
    BITWORD("ST_VEL_LIMITED_EL","status_el",23,1),
    BITWORD("ST_ACCEL_LIMITED_EL","status_el",24,1),

    BITWORD("ST_IN_MOTION_EL","status_el",27,1),
    BITWORD("ST_V_OUT_TRACK_W_EL","status_el",28,1),
    BITWORD("ST_PHASE_NOT_INIT_EL","status_el",29,1),

    BITWORD("ST_SHORT_CIRC_PIV","status_piv",0,1),
    BITWORD("ST_AMP_OVER_T_PIV","status_piv",1,1),
    BITWORD("ST_OVER_V_PIV","status_piv",2,1),
    BITWORD("ST_UNDER_V_PIV","status_piv",3,1),

    BITWORD("ST_FEEDBACK_ERR_PIV","status_piv",5,1),
    BITWORD("ST_MOT_PHAS_ERR_PIV","status_piv",6,1),
    BITWORD("ST_I_LIMITED_PIV","status_piv",7,1),
    BITWORD("ST_VOLT_LIM_PIV","status_piv",8,1),
    BITWORD("ST_LIM_SW_POS_PIV","status_piv",9,1),
    BITWORD("ST_LIM_SW_NEG_PIV","status_piv",10,1),

    BITWORD("ST_DISAB_HWARE_PIV","status_piv",11,1),
    BITWORD("ST_DISAB_SWARE_PIV","status_piv",12,1),
    BITWORD("ST_ATTEMPT_STOP_PIV","status_piv",13,1),
    BITWORD("ST_MOT_BREAK_ACT_PIV","status_piv",14,1),
    BITWORD("ST_PWM_OUT_DIS_PIV","status_piv",15,1),
    BITWORD("ST_POS_SOFT_LIM_PIV","status_piv",16,1),
    BITWORD("ST_NEG_SOFT_LIM_PIV","status_piv",17,1),
    BITWORD("ST_FOLLOW_ERR_PIV","status_piv",18,1),
    BITWORD("ST_FOLLOW_WARN_PIV","status_piv",19,1),
    BITWORD("ST_AMP_HAS_RESET_PIV","status_piv",20,1),
    BITWORD("ST_ENCODER_WRAP_PIV","status_piv",21,1),
    BITWORD("ST_AMP_FAULT_PIV","status_piv",22,1),
    BITWORD("ST_VEL_LIMITED_PIV","status_piv",23,1),
    BITWORD("ST_ACCEL_LIMITED_PIV","status_piv",24,1),

    BITWORD("ST_IN_MOTION_PIV","status_piv",27,1),
    BITWORD("ST_V_OUT_TRACK_W_PIV","status_piv",28,1),
    BITWORD("ST_PHASE_NOT_INIT_PIV","status_piv",29,1),

    BITWORD("ST_SHORT_CIRC_RW","status_rw",0,1),
    BITWORD("ST_AMP_OVER_T_RW","status_rw",1,1),
    BITWORD("ST_OVER_V_RW","status_rw",2,1),
    BITWORD("ST_UNDER_V_RW","status_rw",3,1),

    BITWORD("ST_FEEDBACK_ERR_RW","status_rw",5,1),
    BITWORD("ST_MOT_PHAS_ERR_RW","status_rw",6,1),
    BITWORD("ST_I_LIMITED_RW","status_rw",7,1),
    BITWORD("ST_VOLT_LIM_RW","status_rw",8,1),
    BITWORD("ST_LIM_SW_POS_RW","status_rw",9,1),
    BITWORD("ST_LIM_SW_NEG_RW","status_rw",10,1),

    BITWORD("ST_DISAB_HWARE_RW","status_rw",11,1),
    BITWORD("ST_DISAB_SWARE_RW","status_rw",12,1),
    BITWORD("ST_ATTEMPT_STOP_RW","status_rw",13,1),
    BITWORD("ST_MOT_BREAK_ACT_RW","status_rw",14,1),
    BITWORD("ST_PWM_OUT_DIS_RW","status_rw",15,1),
    BITWORD("ST_POS_SOFT_LIM_RW","status_rw",16,1),
    BITWORD("ST_NEG_SOFT_LIM_RW","status_rw",17,1),
    BITWORD("ST_FOLLOW_ERR_RW","status_rw",18,1),
    BITWORD("ST_FOLLOW_WARN_RW","status_rw",19,1),
    BITWORD("ST_AMP_HAS_RESET_RW","status_rw",20,1),
    BITWORD("ST_ENCODER_WRAP_RW","status_rw",21,1),
    BITWORD("ST_AMP_FAULT_RW","status_rw",22,1),
    BITWORD("ST_VEL_LIMITED_RW","status_rw",23,1),
    BITWORD("ST_ACCEL_LIMITED_RW","status_rw",24,1),

    BITWORD("ST_IN_MOTION_RW","status_rw",27,1),
    BITWORD("ST_V_OUT_TRACK_W_RW","status_rw",28,1),
    BITWORD("ST_PHASE_NOT_INIT_RW","status_rw",29,1),

    BITWORD("STATE_READY_EL", "state_el", 0, 1),
    BITWORD("STATE_SWITCHED_ON_EL", "state_el", 1, 1),
    BITWORD("STATE_OP_ENABLED_EL", "state_el", 2, 1),
    BITWORD("STATE_FAULT_EL", "state_el", 3, 1),
    BITWORD("STATE_VOLTAGE_ENABLED_EL", "state_el", 4, 1),
    BITWORD("STATE_NOT_QUICK_STOP_EL", "state_el", 5, 1),
    BITWORD("STATE_ON_DISABLE_EL", "state_el", 6, 1),
    BITWORD("STATE_WARNING_EL", "state_el", 7, 1),
    BITWORD("STATE_ABORTED_EL", "state_el", 8, 1),
    BITWORD("STATE_REMOTE_EL", "state_el", 9, 1),
    BITWORD("STATE_ON_TARGET_EL", "state_el", 10, 1),
    BITWORD("STATE_AMP_LIMIT_EL", "state_el", 11, 1),
    BITWORD("STATE_MOVING_EL", "state_el", 14, 1),

    BITWORD("STATE_READY_RW", "state_rw", 0, 1),
    BITWORD("STATE_SWITCHED_ON_RW", "state_rw", 1, 1),
    BITWORD("STATE_OP_ENABLED_RW", "state_rw", 2, 1),
    BITWORD("STATE_FAULT_RW", "state_rw", 3, 1),
    BITWORD("STATE_VOLTAGE_ENABLED_RW", "state_rw", 4, 1),
    BITWORD("STATE_NOT_QUICK_STOP_RW", "state_rw", 5, 1),
    BITWORD("STATE_ON_DISABLE_RW", "state_rw", 6, 1),
    BITWORD("STATE_WARNING_RW", "state_rw", 7, 1),
    BITWORD("STATE_ABORTED_RW", "state_rw", 8, 1),
    BITWORD("STATE_REMOTE_RW", "state_rw", 9, 1),
    BITWORD("STATE_ON_TARGET_RW", "state_rw", 10, 1),
    BITWORD("STATE_AMP_LIMIT_RW", "state_rw", 11, 1),
    BITWORD("STATE_MOVING_RW", "state_rw", 14, 1),

    BITWORD("STATE_READY_PIV", "state_piv", 0, 1),
    BITWORD("STATE_SWITCHED_ON_PIV", "state_piv", 1, 1),
    BITWORD("STATE_OP_ENABLED_PIV", "state_piv", 2, 1),
    BITWORD("STATE_FAULT_PIV", "state_piv", 3, 1),
    BITWORD("STATE_VOLTAGE_ENABLED_PIV", "state_piv", 4, 1),
    BITWORD("STATE_NOT_QUICK_STOP_PIV", "state_piv", 5, 1),
    BITWORD("STATE_ON_DISABLE_PIV", "state_piv", 6, 1),
    BITWORD("STATE_WARNING_PIV", "state_piv", 7, 1),
    BITWORD("STATE_ABORTED_PIV", "state_piv", 8, 1),
    BITWORD("STATE_REMOTE_PIV", "state_piv", 9, 1),
    BITWORD("STATE_ON_TARGET_PIV", "state_piv", 10, 1),
    BITWORD("STATE_AMP_LIMIT_PIV", "state_piv", 11, 1),
    BITWORD("STATE_MOVING_PIV", "state_piv", 14, 1),

//  TODO: Convert HWPR bitfield to bitword
//  /* HWPR Control Info */
//  BITFIELD("stat_control_hwpr"
//	   "",
//	   "",
//	   "",
//	   "",
//	   "",
//	   "",
//	   "",
//	   "",
//	   "",
//	   "",
//	   "DO_OVERSHOOT_HWPR",
//	   "DONE_MOVE_HWPR",
//	   "DONE_ALL_HWPR",
//	   "DEAD_POT_HWPR",
//	   "DO_CALPULSE_HWPR",
//	   "",
//	   ),
  BITWORD("MOVE_TYPE_HWPR","stat_control_hwpr",0,3),
  BITWORD("MOVE_STAT_HWPR","stat_control_hwpr",3,3),
  BITWORD("READ_BEFORE_HWPR","stat_control_hwpr",6,2),
  BITWORD("READ_AFTER_HWPR","stat_control_hwpr",8,2),
  /* CRYO */
// TODO: Convert cyrostate bitfield
//  COMMENT("Internal (to mcp) reference of the cryo stat"),
//  BITFIELD("cryostate",
//      "",
//      "",
//      "",
//      "",
//      "POT_VALVE",
//      "POT_DIREC",
//      "LHE_VALVE",
//      "CRYO_DIREC",
//      "LN_VALVE",
//      "AUTO_JFET_HEAT"
//      ),

  LINCOM2("SINCE_START_CYCLE", "TIME", 1, 0, "START_CYCLE",  -1, 0),


//  COMMENT("Cryo Valve Limit Switches"),
//
//  BITFIELD("dig65_das",
//      "LHE_IS_OPEN",
//      "LHE_IS_CLOSED",
//      "LN_IS_OPEN",
//      "LN_IS_CLOSED",
//      "POT_IS_OPEN",
//      "POT_IS_CLOSED"),
//
//  LINCOM2("POT_STATE", "POT_IS_CLOSED", 2, 0, "POT_IS_OPEN",  1, 0),
//  LINCOM2("LHE_STATE", "LHE_IS_CLOSED", 2, 0, "LHE_IS_OPEN",  1, 0),
//  LINCOM2("LN_STATE",  "LN_IS_CLOSED", 2, 0, "LN_IS_OPEN", 1, 0),
//
//  COMMENT("DAS Digital Controls"),
//  BITFIELD("dig43_das",
//      "",
//      "",
//      "",
//      "",
//      "",
//      "",
//      "",
//      "",
//      "HELIUM_LEVEL_HEAT",
//      "CHARCOAL_HEAT",
//      "POT_HS_HEAT",
//      "CHARCOAL_HS_HEAT",
//      "JFET_HEAT",
//      "BDA_HEAT",
//      "CALIBRATOR_HEAT",
//      "HWPR_POS_HEAT"),

  COMMENT("Cryo Table Lookups"),
  COMMENT("Diodes"),

  LINTERP("Td_charcoal",   "TD_CHARCOAL"   , LUT_DIR "dt600.txt"),
    UNITS("Td_charcoal", "Temperature", "K"),
  LINTERP("Td_lhe",        "TD_LHE"        , LUT_DIR "dt600.txt"),
    UNITS("Td_lhe", "Temperature", "K"),
  LINTERP("Td_ln",         "TD_LN"         , LUT_DIR "dt600.txt"),
    UNITS("Td_ln", "Temperature", "K"),
  LINTERP("Td_hs_pot",     "TD_HS_POT"     , LUT_DIR "dt-simonchase.txt"),
    UNITS("Td_hs_pot", "Temperature", "K"),
  LINTERP("Td_jfet",       "TD_JFET"       , LUT_DIR "dt600.txt"),
    UNITS("Td_jfet", "Temperature", "K"),
  LINTERP("Td_vcs_filt",   "TD_VCS_FILT"   , LUT_DIR "dt600.txt"),
    UNITS("Td_vcs_filt", "Temperature", "K"),
  LINTERP("Td_ln_filt",    "TD_LN_FILT"    , LUT_DIR "dt600.txt"),
    UNITS("Td_ln_filt", "Temperature", "K"),
  LINTERP("Td_lhe_filt",   "TD_LHE_FILT"   , LUT_DIR "dt600.txt"),
    UNITS("Td_lhe_filt", "Temperature", "K"),
  LINTERP("Td_vcs_jfet",   "TD_VCS_JFET"   , LUT_DIR "dt600.txt"),
    UNITS("Td_vcs_jfet", "Temperature", "K"),
  LINTERP("Td_hs_charcoal","TD_HS_CHARCOAL", LUT_DIR "dt-simonchase.txt"),
    UNITS("Td_hs_charcoal", "Temperature", "K"),

  COMMENT("ROXes"), // raw calibration only -- warning!
  LINTERP("Tr_300mk_strap","TR_300MK_STRAP", LUT_DIR "rox-raw.txt"),
    UNITS("Tr_300mk_strap", "Temperature", "K"),
  LINTERP("Tr_he3_fridge", "TR_HE3_FRIDGE" , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_he3_fridge", "Temperature", "K"),
  LINTERP("Tr_m5",	   "TR_M5"         , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_m5", "Temperature", "K"),
  LINTERP("Tr_m4",         "TR_M4"         , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_m4", "Temperature", "K"),
  LINTERP("Tr_hwpr",       "TR_HWPR"       , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_hwpr", "Temperature", "K"),
  LINTERP("Tr_horn_500",   "TR_HORN_500"   , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_horn_500", "Temperature", "K"),
  LINTERP("Tr_horn_350",   "TR_HORN_350"   , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_horn_350", "Temperature", "K"),
  LINTERP("Tr_horn_250",   "TR_HORN_250"   , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_horn_250", "Temperature", "K"),
  LINTERP("Tr_he4_pot",    "TR_HE4_POT"    , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_he4_pot", "Temperature", "K"),
  LINTERP("Tr_optbox_filt","TR_OPTBOX_FILT", LUT_DIR "rox-raw.txt"),
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

  END_OF_DERIVED_CHANNELS
};
