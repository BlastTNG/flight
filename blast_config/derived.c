/* derived.c: a list of derived channels
 *
 * This software is copyright (C) 2002-2010 University of Toronto
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
  BITWORD("SOUTH_I_AM", "status_mcc", 0, 1),
  BITWORD("AT_FLOAT", "status_mcc", 1, 1),
  BITWORD("UPLINK_SCHED", "status_mcc", 2, 1),
  BITWORD("BLAST_SUCKS", "status_mcc", 3, 1),
  BITWORD("SCHEDULE", "status_mcc", 4, 3),
  BITWORD("SLOT_SCHED", "status_mcc", 8, 8),

  BITWORD("STATUS_ISC_ETH", "status_eth", 2, 2),
  BITWORD("STATUS_OSC_ETH", "status_eth", 4, 2),
  BITWORD("STATUS_SBSC_ETH", "status_eth", 6, 2),

#ifndef BOLOTEST
    COMMENT("Pointing Stuff"),
    LINCOM("X_H_P", "x_p", 0.0003662109375, 0),

    BITWORD("VETO_EL_MOTOR_ENC", "veto_sensor", 0, 1),
    BITWORD("VETO_ISC", "veto_sensor", 1, 1),
    BITWORD("VETO_EL_ENC", "veto_sensor", 2, 1),
    BITWORD("VETO_MAG", "veto_sensor", 3, 1),
    BITWORD("VETO_GPS", "veto_sensor", 4, 1),
    BITWORD("VETO_EL_CLIN", "veto_sensor", 5, 1),
    BITWORD("VETO_OSC", "veto_sensor", 6, 1),
    BITWORD("IS_SCHED", "veto_sensor", 7, 1),
    BITWORD("AZ_AUTO_GYRO", "veto_sensor", 8, 1),
    BITWORD("EL_AUTO_GYRO", "veto_sensor", 9, 1),
    BITWORD("DISABLE_EL", "veto_sensor", 10, 1),
    BITWORD("DISABLE_AZ", "veto_sensor", 11, 1),
    BITWORD("FORCE_EL", "veto_sensor", 12, 1),
    BITWORD("VETO_PSS", "veto_sensor", 13, 1),

    BITWORD("PULSE_XSC0", "pulse_sc", 0, 1),
    BITWORD("PULSE_XSC1", "pulse_sc", 1, 1),

    COMMENT("ACS Digital Signals"),

    /* charge controller (CC) faults and alarms */
    COMMENT("Charge Controller Bitfields"),

    BITWORD("F_OVERCURRENT_CC1", "fault_cc1", 0, 1),
    BITWORD("F_FET_SHORT_CC1", "fault_cc1", 1, 1),
    BITWORD("F_SOFTWARE_BUG_CC1", "fault_cc1", 2, 1),
    BITWORD("F_BATT_HVD_CC1", "fault_cc1", 3, 1),
    BITWORD("F_ARR_HVD_CC1", "fault_cc1", 4, 1),
    BITWORD("F_DIP_CHANGED_CC1", "fault_cc1", 5, 1),
    BITWORD("F_SETTING_CHANGE_CC1", "fault_cc1", 6, 1),
    BITWORD("F_RTS_SHORT_CC1", "fault_cc1", 7, 1),
    BITWORD("F_RTS_DISCONN_CC1", "fault_cc1", 8, 1),
    BITWORD("F_EEPROM_LIM_CC1", "fault_cc1", 9, 1),
    BITWORD("F_SLAVE_TO_CC1", "fault_cc1", 10, 1),

    BITWORD("F_OVERCURRENT_CC2", "fault_cc2", 0, 1),
    BITWORD("F_FET_SHORT_CC2", "fault_cc2", 1, 1),
    BITWORD("F_SOFTWARE_BUG_CC2", "fault_cc2", 2, 1),
    BITWORD("F_BATT_HVD_CC2", "fault_cc2", 3, 1),
    BITWORD("F_ARR_HVD_CC2", "fault_cc2", 4, 1),
    BITWORD("F_DIP_CHANGED_CC2", "fault_cc2", 5, 1),
    BITWORD("F_SETTING_CHANGE_CC2", "fault_cc2", 6, 1),
    BITWORD("F_RTS_SHORT_CC2", "fault_cc2", 7, 1),
    BITWORD("F_RTS_DISCONN_CC2", "fault_cc2", 8, 1),
    BITWORD("F_EEPROM_LIM_CC2", "fault_cc2", 9, 1),
    BITWORD("F_SLAVE_TO_CC2", "fault_cc2", 10, 1),

    BITWORD("A_RTS_OPEN_CC1", "alarm_cc1", 0, 1),
    BITWORD("A_RTS_SHORT_CC1", "alarm_cc1", 1, 1),
    BITWORD("A_RTS_DISCONN_CC1", "alarm_cc1", 2, 1),
    BITWORD("A_TSENSE_OPEN_CC1", "alarm_cc1", 3, 1),
    BITWORD("A_TSENSE_SHORT_CC1", "alarm_cc1", 4, 1),
    BITWORD("A_HITEMP_LIM_CC1", "alarm_cc1", 5, 1),
    BITWORD("A_CURRENT_LIM_CC1", "alarm_cc1", 6, 1),
    BITWORD("A_CURRENT_OFFSET_CC1", "alarm_cc1", 7, 1),
    BITWORD("A_BATT_RANGE_CC1", "alarm_cc1", 8, 1),
    BITWORD("A_BATTSENSE_DISC_CC1", "alarm_cc1", 9, 1),
    BITWORD("A_UNCALIB_CC1", "alarm_cc1", 10, 1),
    BITWORD("A_RTS_MISWIRE_CC1", "alarm_cc1", 11, 1),
    BITWORD("A_HVD_CC1", "alarm_cc1", 12, 1),
    BITWORD("A_SYS_MISWIRE_CC1", "alarm_cc1", 14, 1),
    BITWORD("A_FET_OPEN_CC1", "alarm_cc1", 15, 1),

    BITWORD("A_RTS_OPEN_CC2", "alarm_cc2", 0, 1),
    BITWORD("A_RTS_SHORT_CC2", "alarm_cc2", 1, 1),
    BITWORD("A_RTS_DISCONN_CC2", "alarm_cc2", 2, 1),
    BITWORD("A_TSENSE_OPEN_CC2", "alarm_cc2", 3, 1),
    BITWORD("A_TSENSE_SHORT_CC2", "alarm_cc2", 4, 1),
    BITWORD("A_HITEMP_LIM_CC2", "alarm_cc2", 5, 1),
    BITWORD("A_CURRENT_LIM_CC2", "alarm_cc2", 6, 1),
    BITWORD("A_CURRENT_OFFSET_CC2", "alarm_cc2", 7, 1),
    BITWORD("A_BATT_RANGE_CC2", "alarm_cc2", 8, 1),
    BITWORD("A_BATTSENSE_DISC_CC2", "alarm_cc2", 9, 1),
    BITWORD("A_UNCALIB_CC2", "alarm_cc2", 10, 1),
    BITWORD("A_RTS_MISWIRE_CC2", "alarm_cc2", 11, 1),
    BITWORD("A_HVD_CC2", "alarm_cc2", 12, 1),
    BITWORD("A_SYS_MISWIRE_CC2", "alarm_cc2", 14, 1),
    BITWORD("A_FET_OPEN_CC2", "alarm_cc2", 15, 1),

    BITWORD("A_VP12_OFF_CC1", "alarm_cc1", 16, 1),
    BITWORD("C_A_HI_INPUT_LIM_CC1", "alarm_cc1", 17, 1),
    BITWORD("C_A_ADC_MAX_IN_CC1", "alarm_cc1", 18, 1),
    BITWORD("C_A_RESET_CC1", "alarm_cc1", 19, 1),

    BITWORD("A_VP12_OFF_CC2", "alarm_cc2", 16, 1),
    BITWORD("C_A_HI_INPUT_LIM_CC2", "alarm_cc2", 17, 1),
    BITWORD("C_A_ADC_MAX_IN_CC2", "alarm_cc2", 18, 1),
    BITWORD("C_A_RESET_CC2", "alarm_cc2", 19, 1),

    BITWORD("ACT0_INIT_ACTBUS", "status_actbus", 0, 1),
    BITWORD("ACT1_INIT_ACTBUS", "status_actbus", 1, 1),
    BITWORD("ACT2_INIT_ACTBUS", "status_actbus", 2, 1),
    BITWORD("BALANCE_INIT_ACTBUS", "status_actbus", 3, 1),
    BITWORD("LOCK_INIT_ACTBUS", "status_actbus", 4, 1),
    BITWORD("HWPR_INIT_ACTBUS", "status_actbus", 5, 1),
    BITWORD("SHUTTER_INIT_ACTBUS", "status_actbus", 6, 1),

      #endif

    COMMENT("Lock Motor/Actuators"),
    BITWORD("LS_OPEN_LOCK", "state_lock", 0, 1),
    BITWORD("LS_CLOSED_LOCK", "state_lock", 1, 1),
    BITWORD("LS_DRIVE_OFF_LOCK", "state_lock", 2, 1),
    BITWORD("LS_POT_RAIL_LOCK", "state_lock", 3, 1),
    BITWORD("LS_DRIVE_EXT_LOCK", "state_lock", 4, 1),
    BITWORD("LS_DRIVE_RET_LOCK", "state_lock", 5, 1),
    BITWORD("LS_DRIVE_STP_LOCK", "state_lock", 6, 1),
    BITWORD("LS_DRIVE_JIG_LOCK", "state_lock", 7, 1),
    BITWORD("LS_DRIVE_UNK_LOCK", "state_lock", 8, 1),
    BITWORD("LS_EL_OK_LOCK", "state_lock", 9, 1),
    BITWORD("LS_IGNORE_EL_LOCK", "state_lock", 10, 1),
    BITWORD("LS_DRIVE_FORCE_LOCK", "state_lock", 11, 1),

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

    BITWORD("TRIM_WAIT_ACT", "flags_act", 0, 1),
    BITWORD("TRIMMED_ACT", "flags_act", 1, 1),
    BITWORD("BUSY_0_ACT", "flags_act", 2, 1),
    BITWORD("BUSY_1_ACT", "flags_act", 3, 1),
    BITWORD("BUSY_2_ACT", "flags_act", 4, 1),
    BITWORD("BAD_MOVE_ACT", "flags_act", 5, 1),

    BITWORD("LATCHED_DATA_CRC_EL", "latched_fault_el", 0, 1),
    BITWORD("LATCHED_AMP_INT_EL", "latched_fault_el", 1, 1),
    BITWORD("LATCHED_SHORT_CIRC_EL", "latched_fault_el", 2, 1),
    BITWORD("LATCHED_AMP_OVER_R_EL", "latched_fault_el", 3, 1),
    BITWORD("LATCHED_MOTOR_OVER_R_EL", "latched_fault_el", 4, 1),
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

    BITWORD("ST_SHORT_CIRC_EL", "status_el", 0, 1),
    BITWORD("ST_AMP_OVER_T_EL", "status_el", 1, 1),
    BITWORD("ST_OVER_V_EL", "status_el", 2, 1),
    BITWORD("ST_UNDER_V_EL", "status_el", 3, 1),

    BITWORD("ST_FEEDBACK_ERR_EL", "status_el", 5, 1),
    BITWORD("ST_MOT_PHAS_ERR_EL", "status_el", 6, 1),
    BITWORD("ST_I_LIMITED_EL", "status_el", 7, 1),
    BITWORD("ST_VOLT_LIM_EL", "status_el", 8, 1),
    BITWORD("ST_LIM_SW_POS_EL", "status_el", 9, 1),
    BITWORD("ST_LIM_SW_NEG_EL", "status_el", 10, 1),

    BITWORD("ST_DISAB_HWARE_EL", "status_el", 11, 1),
    BITWORD("ST_DISAB_SWARE_EL", "status_el", 12, 1),
    BITWORD("ST_ATTEMPT_STOP_EL", "status_el", 13, 1),
    BITWORD("ST_MOT_BREAK_ACT_EL", "status_el", 14, 1),
    BITWORD("ST_PWM_OUT_DIS_EL", "status_el", 15, 1),
    BITWORD("ST_POS_SOFT_LIM_EL", "status_el", 16, 1),
    BITWORD("ST_NEG_SOFT_LIM_EL", "status_el", 17, 1),
    BITWORD("ST_FOLLOW_ERR_EL", "status_el", 18, 1),
    BITWORD("ST_FOLLOW_WARN_EL", "status_el", 19, 1),
    BITWORD("ST_AMP_HAS_RESET_EL", "status_el", 20, 1),
    BITWORD("ST_ENCODER_WRAP_EL", "status_el", 21, 1),
    BITWORD("ST_AMP_FAULT_EL", "status_el", 22, 1),
    BITWORD("ST_VEL_LIMITED_EL", "status_el", 23, 1),
    BITWORD("ST_ACCEL_LIMITED_EL", "status_el", 24, 1),

    BITWORD("ST_IN_MOTION_EL", "status_el", 27, 1),
    BITWORD("ST_V_OUT_TRACK_W_EL", "status_el", 28, 1),
    BITWORD("ST_PHASE_NOT_INIT_EL", "status_el", 29, 1),

    BITWORD("ST_SHORT_CIRC_PIV", "status_piv", 0, 1),
    BITWORD("ST_AMP_OVER_T_PIV", "status_piv", 1, 1),
    BITWORD("ST_OVER_V_PIV", "status_piv", 2, 1),
    BITWORD("ST_UNDER_V_PIV", "status_piv", 3, 1),

    BITWORD("ST_FEEDBACK_ERR_PIV", "status_piv", 5, 1),
    BITWORD("ST_MOT_PHAS_ERR_PIV", "status_piv", 6, 1),
    BITWORD("ST_I_LIMITED_PIV", "status_piv", 7, 1),
    BITWORD("ST_VOLT_LIM_PIV", "status_piv", 8, 1),
    BITWORD("ST_LIM_SW_POS_PIV", "status_piv", 9, 1),
    BITWORD("ST_LIM_SW_NEG_PIV", "status_piv", 10, 1),

    BITWORD("ST_DISAB_HWARE_PIV", "status_piv", 11, 1),
    BITWORD("ST_DISAB_SWARE_PIV", "status_piv", 12, 1),
    BITWORD("ST_ATTEMPT_STOP_PIV", "status_piv", 13, 1),
    BITWORD("ST_MOT_BREAK_ACT_PIV", "status_piv", 14, 1),
    BITWORD("ST_PWM_OUT_DIS_PIV", "status_piv", 15, 1),
    BITWORD("ST_POS_SOFT_LIM_PIV", "status_piv", 16, 1),
    BITWORD("ST_NEG_SOFT_LIM_PIV", "status_piv", 17, 1),
    BITWORD("ST_FOLLOW_ERR_PIV", "status_piv", 18, 1),
    BITWORD("ST_FOLLOW_WARN_PIV", "status_piv", 19, 1),
    BITWORD("ST_AMP_HAS_RESET_PIV", "status_piv", 20, 1),
    BITWORD("ST_ENCODER_WRAP_PIV", "status_piv", 21, 1),
    BITWORD("ST_AMP_FAULT_PIV", "status_piv", 22, 1),
    BITWORD("ST_VEL_LIMITED_PIV", "status_piv", 23, 1),
    BITWORD("ST_ACCEL_LIMITED_PIV", "status_piv", 24, 1),

    BITWORD("ST_IN_MOTION_PIV", "status_piv", 27, 1),
    BITWORD("ST_V_OUT_TRACK_W_PIV", "status_piv", 28, 1),
    BITWORD("ST_PHASE_NOT_INIT_PIV", "status_piv", 29, 1),

    BITWORD("ST_SHORT_CIRC_RW", "status_rw", 0, 1),
    BITWORD("ST_AMP_OVER_T_RW", "status_rw", 1, 1),
    BITWORD("ST_OVER_V_RW", "status_rw", 2, 1),
    BITWORD("ST_UNDER_V_RW", "status_rw", 3, 1),

    BITWORD("ST_FEEDBACK_ERR_RW", "status_rw", 5, 1),
    BITWORD("ST_MOT_PHAS_ERR_RW", "status_rw", 6, 1),
    BITWORD("ST_I_LIMITED_RW", "status_rw", 7, 1),
    BITWORD("ST_VOLT_LIM_RW", "status_rw", 8, 1),
    BITWORD("ST_LIM_SW_POS_RW", "status_rw", 9, 1),
    BITWORD("ST_LIM_SW_NEG_RW", "status_rw", 10, 1),

    BITWORD("ST_DISAB_HWARE_RW", "status_rw", 11, 1),
    BITWORD("ST_DISAB_SWARE_RW", "status_rw", 12, 1),
    BITWORD("ST_ATTEMPT_STOP_RW", "status_rw", 13, 1),
    BITWORD("ST_MOT_BREAK_ACT_RW", "status_rw", 14, 1),
    BITWORD("ST_PWM_OUT_DIS_RW", "status_rw", 15, 1),
    BITWORD("ST_POS_SOFT_LIM_RW", "status_rw", 16, 1),
    BITWORD("ST_NEG_SOFT_LIM_RW", "status_rw", 17, 1),
    BITWORD("ST_FOLLOW_ERR_RW", "status_rw", 18, 1),
    BITWORD("ST_FOLLOW_WARN_RW", "status_rw", 19, 1),
    BITWORD("ST_AMP_HAS_RESET_RW", "status_rw", 20, 1),
    BITWORD("ST_ENCODER_WRAP_RW", "status_rw", 21, 1),
    BITWORD("ST_AMP_FAULT_RW", "status_rw", 22, 1),
    BITWORD("ST_VEL_LIMITED_RW", "status_rw", 23, 1),
    BITWORD("ST_ACCEL_LIMITED_RW", "status_rw", 24, 1),

    BITWORD("ST_IN_MOTION_RW", "status_rw", 27, 1),
    BITWORD("ST_V_OUT_TRACK_W_RW", "status_rw", 28, 1),
    BITWORD("ST_PHASE_NOT_INIT_RW", "status_rw", 29, 1),

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

//  /* HWPR Control Info */
    BITWORD("DO_OVERSHOOT_HWPR", "stat_control_hwpr", 10, 1),
    BITWORD("DONE_MOVE_HWPR", "stat_control_hwpr", 11, 1),
    BITWORD("DONE_ALL_HWPR", "stat_control_hwpr", 12, 1),
    BITWORD("DEAD_POT_HWPR", "stat_control_hwpr", 13, 1),
    BITWORD("DO_CALPULSE_HWPR", "stat_control_hwpr", 14, 1),

    BITWORD("MOVE_TYPE_HWPR", "stat_control_hwpr", 0, 3),
    BITWORD("MOVE_STAT_HWPR", "stat_control_hwpr", 3, 3),
    BITWORD("READ_BEFORE_HWPR", "stat_control_hwpr", 6, 2),
    BITWORD("READ_AFTER_HWPR", "stat_control_hwpr", 8, 2),

    /* CRYO */
    /*BITWORD("","dio_heaters",0,1) */
    BITWORD("POT_VALVE", "cryostate", 4, 1),
    BITWORD("POT_DIREC", "cryostate", 5, 1),
    BITWORD("LHE_VALVE", "cryostate", 6, 1),
    BITWORD("CRYO_DIREC", "cryostate", 7, 1),
    BITWORD("LN_VALVE", "cryostate", 8, 1),
    BITWORD("AUTO_JFET_HEAT", "cryostate", 9, 1),


    LINCOM2("SINCE_START_CYCLE", "TIME", 1, 0, "START_CYCLE",  -1, 0),


  COMMENT("Cryo Table Lookups"),
  COMMENT("Diodes"),
    LINCOM("TD_CHARCOAL_HS_INT", "TD_CHARCOAL_HS", 0.199204, 0.000054),
    LINCOM("TD_VCS2_FILT_INT", "TD_VCS2_FILT", 0.198863, -0.00015),
    LINCOM("TD_250FPA_INT", "TD_250FPA", 0.199271, 0.000037),
    LINCOM("TD_HWP_INT", "TD_HWP", 0.199518, -0.000016),
    LINCOM("TD_VCS1_HX_INT", "TD_VCS1_HX", 0.199721, 0.000025),
    LINCOM("TD_1K_FRIDGE_INT", "TD_1K_FRIDGE", 0.199697, 0.000019),
    LINCOM("TD_4K_PLATE_INT", "TD_4K_PLATE", 0.198982, 0.000030),
    LINCOM("TD_VCS1_FILT_INT", "TD_VCS1_FILT", 0.199619, -0.000064),
    LINCOM("TD_M3_INT", "TD_M3", 0.199267, -0.000070),
    LINCOM("TD_CHARCOAL_INT", "TD_CHARCOAL", 0.199082, -0.000092),
    LINCOM("TD_OB_FILTER_INT", "TD_OB_FILTER", 0.200040, 0.000018),
    LINCOM("TD_VCS2_PLATE_INT", "TD_VCS2_PLATE", 0.199134, -0.000079),
    LINCOM("TD_M4_INT", "TD_M4", 0.199140, 0.000057),
    LINCOM("TD_4K_FILT_INT", "TD_4K_FILT", 0.199656, 0.000034),
    LINCOM("TD_VCS2_HX_INT", "TD_VCS2_HX", 0.199033, -0.000013),
    LINCOM("TD_VCS1_PLATE_INT", "TD_VCS1_PLATE", 0.199517, -0.000026),
    LINTERP("Td_charcoal_hs", "TD_CHARCOAL_HS_INT", LUT_DIR "dt-simonchase.txt"),
    LINTERP("Td_vcs2_filt", "TD_VCS2_FILT_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_250fpa", "TD_250FPA_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_hwp", "TD_HWP_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_vcs1_hx", "TD_VCS1_HX_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_1k_fridge", "TD_1K_FRIDGE_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_4k_plate", "TD_4K_PLATE_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_vcs1_filt", "TD_VCS1_FILT_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_m3", "TD_M3_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_charcoal", "TD_CHARCOAL_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_ob_filter", "TD_OB_FILTER_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_vcs2_plate", "TD_VCS2_PLATE_INT", LUT_DIR "dt-simonchase.txt"),
    LINTERP("Td_m4", "TD_M4_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_4k_filt", "TD_4K_FILT_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_vcs2_hx", "TD_VCS2_HX_INT", LUT_DIR "dt670_orig.text"),
    LINTERP("Td_vcs1_plate", "TD_VCS1_PLATE_INT", LUT_DIR "dt670_orig.text"),


  COMMENT("ROXes"), // raw calibration only -- warning!
    LINCOM("TR_FPA_1K_INT", "TR_FPA_1K", -1, 0),
    LINCOM("TR_250_FPA_INT", "TR_250_FPA", -1, 0),
    LINCOM("TR_1K_PLATE_INT", "TR_1K_PLATE", -1, 0),
    LINCOM("TR_300MK_STRAP_INT", "TR_300MK_STRAP", -1, 0),
    LINCOM("TR_350_FPA_INT", "TR_350_FPA", -1, 0),
    LINCOM("TR_HE4_POT_INT", "TR_HE4_POT", -1, 0),
    LINCOM("TR_HE3_FRIDGE_INT", "TR_HE3_FRIDGE", -1, 0),
    LINCOM("TR_500_FPA_INT", "TR_500_FPA", -1, 0),
    LINCOM("ROX_BIAS_INT", "ROX_BIAS", -1, 0),
    LINTERP("NOMINAL_BIAS", "ROX_BIAS_INT", LUT_DIR "Vbias_LUT.txt"),
    LINCOM("ROX_BIAS_COMPARE", "ROX_BIAS_INT", 3.42763512, 0),
    DIVIDE("TR_FPA_1K_CORRECTED", "TR_FPA_1K_INT", "ROX_BIAS_COMPARE"),
    DIVIDE("TR_250_FPA_CORRECTED", "TR_250_FPA_INT", "ROX_BIAS_COMPARE"),
    DIVIDE("TR_1K_PLATE_CORRECTED", "TR_1K_PLATE_INT", "ROX_BIAS_COMPARE"),
    DIVIDE("TR_300MK_STRAP_CORRECTED", "TR_300MK_STRAP_INT", "ROX_BIAS_COMPARE"),
    DIVIDE("TR_350_FPA_CORRECTED", "TR_350_FPA_INT", "ROX_BIAS_COMPARE"),
    DIVIDE("TR_HE4_POT_CORRECTED", "TR_HE4_POT_INT", "ROX_BIAS_COMPARE"),
    DIVIDE("TR_HE3_FRIDGE_CORRECTED", "TR_HE3_FRIDGE_INT", "ROX_BIAS_COMPARE"),
    DIVIDE("TR_500_FPA_CORRECTED", "TR_500_FPA_INT", "ROX_BIAS_COMPARE"),
    LINTERP("RR_FPA_1K", "TR_FPA_1K_CORRECTED", LUT_DIR "VR_LUT_fpa1k.txt"),
    LINTERP("RR_250_FPA", "TR_250_FPA_CORRECTED", LUT_DIR "VR_LUT_fpa250.txt"),
    LINTERP("RR_1K_PLATE", "TR_1K_PLATE_CORRECTED", LUT_DIR "VR_LUT_plate1k.txt"),
    LINTERP("RR_300MK_STRAP", "TR_300MK_STRAP_CORRECTED", LUT_DIR "VR_LUT_strap300mk.txt"),
    LINTERP("RR_350_FPA", "TR_350_FPA_CORRECTED", LUT_DIR "VR_LUT_fpa350.txt"),
    LINTERP("RR_HE4_POT", "TR_HE4_POT_CORRECTED", LUT_DIR "VR_LUT_he4pot.txt"),
    LINTERP("RR_HE3_FRIDGE", "TR_HE3_FRIDGE_CORRECTED", LUT_DIR "VR_LUT_he3fridge.txt"),
    LINTERP("RR_500_FPA", "TR_500_FPA_CORRECTED", LUT_DIR "VR_LUT_fpa500.txt"),
    LINTERP("Tr_fpa_1k", "RR_FPA_1K", LUT_DIR "rox-raw.txt"),
    UNITS("Tr_fpa_1k", "Temperature", "K"),
    LINTERP("Tr_250_fpa", "RR_250_FPA" , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_250_fpa", "Temperature", "K"),
    LINTERP("Tr_1k_plate", "RR_1K_PLATE", LUT_DIR "rox-raw.txt"),
    UNITS("Tr_1k_plate", "Temperature", "K"),
    LINTERP("Tr_300mk_strap", "RR_300MK_STRAP", LUT_DIR "rox-raw.txt"),
    UNITS("Tr_300mk_strap", "Temperature", "K"),
    LINTERP("Tr_350_fpa",   "RR_350_FPA"   , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_350_fpa", "Temperature", "K"),
    LINTERP("Tr_he4_pot",   "RR_HE4_POT"   , LUT_DIR "rox-raw.txt"),
    UNITS("Tr_he4_pot", "Temperature", "K"),
    LINTERP("Tr_he3_fridge",    "RR_HE3_FRIDGE"    , LUT_DIR "ROX_Cal_U04486.LUT"),
    UNITS("Tr_he3_fridge", "Temperature", "K"),
    LINTERP("Tr_500_fpa", "RR_500_FPA", LUT_DIR "rox-raw.txt"),
    UNITS("Tr_500_fpa", "Temperature", "K"),
    // cryo labjack stuff below
    LINCOM("Helium_level_inches", "LEVEL_SENSOR_READ", 4.598, 28.307),
    LINTERP("Helium_volume_liters", "Helium_level_inches", LUT_DIR "LevelSensor_Volume.LUT"),
    LINCOM("Alarm_gauge_pressure", "ALARM_GAUGE", .0809, 2.0218),

    /*GONDOLA THERMISTOR CALIBRATION */
    COMMENT("Thermistors for OF/IF"),
    LINTERP("THERMISTOR_1_R", "THERMISTOR_1", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_2_R", "THERMISTOR_2", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_3_R", "THERMISTOR_3", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_4_R", "THERMISTOR_4", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_5_R", "THERMISTOR_5", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_6_R", "THERMISTOR_6", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_7_R", "THERMISTOR_7", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_8_R", "THERMISTOR_8", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_9_R", "THERMISTOR_9", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_10_R", "THERMISTOR_10", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_11_R", "THERMISTOR_11", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_12_R", "THERMISTOR_12", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_13_R", "THERMISTOR_13", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_14_R", "THERMISTOR_14", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_15_R", "THERMISTOR_15", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_16_R", "THERMISTOR_16", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_17_R", "THERMISTOR_17", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_18_R", "THERMISTOR_18", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_19_R", "THERMISTOR_19", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_20_R", "THERMISTOR_20", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_21_R", "THERMISTOR_21", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_22_R", "THERMISTOR_22", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_23_R", "THERMISTOR_23", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_24_R", "THERMISTOR_24", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_25_R", "THERMISTOR_25", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_26_R", "THERMISTOR_26", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_27_R", "THERMISTOR_27", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_28_R", "THERMISTOR_28", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_29_R", "THERMISTOR_29", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_30_R", "THERMISTOR_30", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_31_R", "THERMISTOR_31", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_32_R", "THERMISTOR_32", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_33_R", "THERMISTOR_33", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_34_R", "THERMISTOR_34", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_35_R", "THERMISTOR_35", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_36_R", "THERMISTOR_36", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_37_R", "THERMISTOR_37", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_38_R", "THERMISTOR_38", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_39_R", "THERMISTOR_39", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_40_R", "THERMISTOR_40", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_41_R", "THERMISTOR_41", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_42_R", "THERMISTOR_42", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_43_R", "THERMISTOR_43", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_44_R", "THERMISTOR_44", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_45_R", "THERMISTOR_45", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_46_R", "THERMISTOR_46", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_47_R", "THERMISTOR_47", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_48_R", "THERMISTOR_48", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_49_R", "THERMISTOR_49", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_50_R", "THERMISTOR_50", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_51_R", "THERMISTOR_51", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_52_R", "THERMISTOR_52", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_53_R", "THERMISTOR_53", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_54_R", "THERMISTOR_54", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_55_R", "THERMISTOR_55", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_56_R", "THERMISTOR_56", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_57_R", "THERMISTOR_57", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_58_R", "THERMISTOR_58", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_59_R", "THERMISTOR_59", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_60_R", "THERMISTOR_60", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_61_R", "THERMISTOR_61", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_62_R", "THERMISTOR_62", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_63_R", "THERMISTOR_63", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_64_R", "THERMISTOR_64", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_65_R", "THERMISTOR_65", LUT_DIR "Thermistor_VtoR.LUT"),
    LINTERP("THERMISTOR_66_R", "THERMISTOR_66", LUT_DIR "Thermistor_VtoR.LUT"),
    
    LINTERP("THERMISTOR_1_T", "THERMISTOR_1_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_2_T", "THERMISTOR_2_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_3_T", "THERMISTOR_3_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_4_T", "THERMISTOR_4_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_5_T", "THERMISTOR_5_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_6_T", "THERMISTOR_6_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_7_T", "THERMISTOR_7_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_8_T", "THERMISTOR_8_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_9_T", "THERMISTOR_9_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_10_T", "THERMISTOR_10_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_11_T", "THERMISTOR_11_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_12_T", "THERMISTOR_12_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_13_T", "THERMISTOR_13_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_14_T", "THERMISTOR_14_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_15_T", "THERMISTOR_15_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_16_T", "THERMISTOR_16_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_17_T", "THERMISTOR_17_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_18_T", "THERMISTOR_18_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_19_T", "THERMISTOR_19_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_20_T", "THERMISTOR_20_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_21_T", "THERMISTOR_21_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_22_T", "THERMISTOR_22_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_23_T", "THERMISTOR_23_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_24_T", "THERMISTOR_24_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_25_T", "THERMISTOR_25_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_26_T", "THERMISTOR_26_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_27_T", "THERMISTOR_27_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_28_T", "THERMISTOR_28_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_29_T", "THERMISTOR_29_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_30_T", "THERMISTOR_30_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_31_T", "THERMISTOR_31_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_32_T", "THERMISTOR_32_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_33_T", "THERMISTOR_33_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_34_T", "THERMISTOR_34_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_35_T", "THERMISTOR_35_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_36_T", "THERMISTOR_36_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_37_T", "THERMISTOR_37_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_38_T", "THERMISTOR_38_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_39_T", "THERMISTOR_39_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_40_T", "THERMISTOR_40_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_41_T", "THERMISTOR_41_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_42_T", "THERMISTOR_42_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_43_T", "THERMISTOR_43_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_44_T", "THERMISTOR_44_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_45_T", "THERMISTOR_45_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_46_T", "THERMISTOR_46_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_47_T", "THERMISTOR_47_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_48_T", "THERMISTOR_48_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_49_T", "THERMISTOR_49_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_50_T", "THERMISTOR_50_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_51_T", "THERMISTOR_51_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_52_T", "THERMISTOR_52_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_53_T", "THERMISTOR_53_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_54_T", "THERMISTOR_54_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_55_T", "THERMISTOR_55_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_56_T", "THERMISTOR_56_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_57_T", "THERMISTOR_57_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_58_T", "THERMISTOR_58_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_59_T", "THERMISTOR_59_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_60_T", "THERMISTOR_60_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_61_T", "THERMISTOR_61_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_62_T", "THERMISTOR_62_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_63_T", "THERMISTOR_63_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_64_T", "THERMISTOR_64_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_65_T", "THERMISTOR_65_R", LUT_DIR "Thermistor_RtoT.LUT"),
    LINTERP("THERMISTOR_66_T", "THERMISTOR_66_R", LUT_DIR "Thermistor_RtoT.LUT"),
    
    BITWORD("TRIGGER_XSC0", "trigger_xsc", 0, 1),
    BITWORD("TRIGGER_XSC1", "trigger_xsc", 1, 1),
    BITWORD("TRIGGER_STATE_XSC", "trigger_xsc", 2, 6),

  END_OF_DERIVED_CHANNELS
};
