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
 * o PHASE: Phase shift a field. Use shift 0 for channel aliases. Arguments:
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Phase shift (integer).
 * o RECIP: Calculate reciprocal of a field. Arguments:
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Dividned (double). Result is (Dividend / Source Channel)
 * o MULTIPLY/DIVIDE: Multiply/divide a pair of fields. Arguments:
 *   1.  Derived Channel Name (string)
 *   2.  Source 1 Channel Name (string)
 *   3.  Source 2 Channel Name (string) Result is (Source 1 *,/ Source 2)
 * o MPLEX: a multiplexed channel.  Arguments:
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Multiplex index Channel Name (string)
 *   4.  Value of the Multiplex Index for the derived channel (int)
 *   5.  Maximum value of the Multiplex Index, or zero if unknown (int)
 *
 * In addition to the derived channels derived below, defile will add the
 * "Nice CPU Values" (to wit: CPU_SEC, CPU_HOUR, etc.), properly offset to the
 * start of the file, to the end of the format file.
 */

#define LUT_DIR "/data/etc/spider/"

union DerivedUnion DerivedChannels[] = {
  /* Pointing */
  COMMENT("Microsecond Resolution Time"),
  LINCOM2("Time", "time_usec", 1.0E-6, 0, "time",  1, 0),
  UNITS("Time", "Time", "s"),

  COMMENT("General BLAST Status"),
  BITFIELD("status_flc",
      "BITSY_I_AM",
      "ITSY_I_AM",
      "AT_FLOAT",
      "IS_EXT_BBC",
      "UPLINK_SCHED",
      "BLAST_SUCKS"),
  BITWORD("SCHEDULE", "status_flc", 4, 3),
  BITWORD("SLOT_SCHED", "status_flc", 8, 8),
  BITWORD("STATUS_GOOD_ETH", "status_eth", 0, 2),
  BITWORD("STATUS_BAD_ETH", "status_eth", 2, 2),
  BITWORD("STATUS_UGLY_ETH", "status_eth", 4, 2),

#ifndef BOLOTEST
  COMMENT("Pointing Stuff"),
  LINCOM("X_H_P", "x_p", 0.0003662109375, 0),
  BITFIELD("veto_sensor",
      "VETO_EL_1_ENC",
      "",
      "VETO_MAG",
      "VETO_GPS",
      "VETO_EL_2_ENC",
      "",
      "",
      "IS_SCHED",
      "AZ_AUTO_GYRO",
      "EL_AUTO_GYRO",
      "DISABLE_EL",
      "DISABLE_AZ",
      "FORCE_EL",
      "VETO_PSS"
      ),

  LINCOM("AMPL_P", "W_P", 0.5, 0.0), // convert scan width to scan amplitude

  COMMENT("ACS Digital Signals"),

  BITFIELD("CONTROL_LOCK",
      "CLOSING_PORT_LOCK",
      "OPENING_PORT_LOCK",
      "CLOSING_STAR_LOCK",
      "OPENING_STAR_LOCK",
      ),
  BITFIELD("LIMIT_LOCK",
      "LIM_NCLOSE_PORT_LOCK",
      "LIM_NOPEN_PORT_LOCK",
      "LIM_NCLOSE_STAR_LOCK",
      "LIM_NOPEN_STAR_LOCK"
      ),
  BITWORD("PIN_IN_LOCK", "STATE_LOCK", 0, 1),
  BITWORD("GOAL_LOCK", "STATE_LOCK", 1, 4),
  BITWORD("STATE_P_LOCK", "STATE_LOCK", 5, 4),
  BITWORD("STATE_S_LOCK", "STATE_LOCK", 9, 4),

  /* charge controller (CC) faults and alarms */

  COMMENT("Charge Controller Bitfields"),

  BITFIELD("fault_of_cc",
      "F_OVRCURRENT_OF_CC",
      "F_FET_SHRT_OF_CC",
      "F_SFTWRE_BUG_OF_CC",
      "F_BATT_HVD_OF_CC",
      "F_ARR_HVD_OF_CC",
      "F_DIP_CHANGED_OF_CC",
      "F_SETTINGCHNG_OF_CC",
      "F_RTS_SHORT_OF_CC",
      "F_RTS_DISCONN_OF_CC",
      "F_EEPROM_LIM_OF_CC",
      "F_SLAVE_TO_OF_CC"
      ),

  BITFIELD("alarm_lo_of_cc",
      "A_RTS_OPEN_OF_CC",
      "A_RTS_SHORT_OF_CC",
      "A_RTS_DISCONN_OF_CC",
      "A_TSENSE_OPEN_OF_CC",
      "A_TSENSE_SHRT_OF_CC",
      "A_HITEMP_LIM_OF_CC",
      "A_CURRNT_LIM_OF_CC",
      "A_CURRNT_OFFS_OF_CC",
      "A_BATTSEN_RNG_OF_CC",
      "A_BATTSEN_DSC_OF_CC",
      "A_UNCALIB_OF_CC",
      "A_RTS_MISWIRE_OF_CC",
      "A_HVD_OF_CC",
      "",
      "A_SYS_MISWIRE_OF_CC",
      "A_FET_OPEN_OF_CC",
      ),

 BITFIELD("alarm_hi_of_cc",
      "A_VP12_OFF_OF_CC",
      "A_HI_INPUTLIM_OF_CC",
      "A_ADC_MAX_IN_OF_CC",
      "A_RESET_OF_CC",
      ),

  BITFIELD("fault_if_cc",
      "F_OVRCURRENT_IF_CC",
      "F_FET_SHRT_IF_CC",
      "F_SFTWRE_BUG_IF_CC",
      "F_BATT_HVD_IF_CC",
      "F_ARR_HVD_IF_CC",
      "F_DIP_CHANGED_IF_CC",
      "F_SETTINGCHNG_IF_CC",
      "F_RTS_SHORT_IF_CC",
      "F_RTS_DISCONN_IF_CC",
      "F_EEPROM_LIM_IF_CC",
      "F_SLAVE_TO_IF_CC"
      ),

  BITFIELD("alarm_lo_if_cc",
      "A_RTS_OPEN_IF_CC",
      "A_RTS_SHORT_IF_CC",
      "A_RTS_DISCONN_IF_CC",
      "A_TSENSE_OPEN_IF_CC",
      "A_TSENSE_SHRT_IF_CC",
      "A_HITEMP_LIM_IF_CC",
      "A_CURRNT_LIM_IF_CC",
      "A_CURRNT_OFFS_IF_CC",
      "A_BATTSEN_RNG_IF_CC",
      "A_BATTSEN_DSC_IF_CC",
      "A_UNCALIB_IF_CC",
      "A_RTS_MISWIRE_IF_CC",
      "A_HVD_IF_CC",
      "",
      "A_SYS_MISWIRE_IF_CC",
      "A_FET_OPEN_IF_CC",
      ),

 BITFIELD("alarm_hi_if_cc",
      "A_VP12_OFF_IF_CC",
      "A_HI_INPUTLIM_IF_CC",
      "A_ADC_MAX_IN_IF_CC",
      "A_RESET_IF_CC",
      ),

#endif

  BITFIELD("stat_dr_rw",
      "ST_BRIDGE_ENA_RW",
      "ST_DYN_BRAKE_ENA_RW",
      "ST_SHUNT_EN_RW",
      "ST_POS_STOP_ENA_RW",
      "ST_NEG_STOP_ENA_RW",
      "ST_POS_TORQ_INH_RW",
      "ST_NEG_TORQ_INH_RW",
      "ST_EXT_BRAKE_RW",
      "ST_DR_RESET_RW",
      "ST_DR_INTER_ERR_RW",
      "ST_DR_SHORT_CIRC_RW",
      "ST_DR_I_OVERSHOT_RW",
      "ST_DR_UNDER_V_RW",
      "ST_DR_OVER_V_RW",
      "ST_DR_OVER_TEMP_RW",
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
  

  BITFIELD("mode_el",
           "MODE_AUTO_EL",
           "MODE_TWISTSTOP_EL",
           "MODE_ON_EL"
  ),
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

  /* internal frame rate (from period) */
  RECIP("RATE_INT_BBC", "FRAME_INT_BBC", 4.e6/384.),
  UNITS("RATE_INT_BBC", "Frequency", "Hz"),

/*
#define T_ACS(tch, vch) \
  LINTERP(tch, vch, LUT_DIR "thermistor.lut"), \
  UNITS(tch, "Temperature", "^oC")
*/

  /* Housekeeping */
  COMMENT("Housekeeping digital channels"),

  BITFIELD("CTL_SFTV",
      "", "", "", "", "", "", "", "",
      "OPENING_ATM_SFTV", "",
      "CLOSING_ATM_SFTV", "",
      "OPENING_PUMP_SFTV", "",
      "CLOSING_PUMP_SFTV", ""
      ),
  BITFIELD("LIM_SFTV",
      "LIM_NOPEN_ATM_SFTV", "IS_POWERED_SFTV",
      "LIM_NCLOSE_ATM_SFTV", "",
      "LIM_NOPEN_PUMP_SFTV", "",
      "LIM_NCLOSE_PUMP_SFTV", ""
      ),
  BITWORD("GOAL_ATM_SFTV", "STATE_SFTV", 0, 3),
  BITWORD("STATE_ATM_SFTV", "STATE_SFTV", 4, 7),
  BITWORD("GOAL_PUMP_SFTV", "STATE_SFTV", 8, 11),
  BITWORD("STATE_PUMP_SFTV", "STATE_SFTV", 12, 15),

  BITFIELD("heat_13_hk",
      "HEAT_PUMP_X3_HK",
      "HEAT_HSW_X3_HK",
      "HEAT_HTR2_X3_HK",
      "HEAT_HTR1_X3_HK",
      "HEAT_SSA_X3_HK",
      "HEAT_FPHI_X3_HK",
      "HEAT_HTR3_X3_HK",
      "SERVO_PUMP_X3_HK",
      "HEAT_PUMP_X1_HK",
      "HEAT_HSW_X1_HK",
      "HEAT_HTR2_X1_HK",
      "HEAT_HTR1_X1_HK",
      "HEAT_SSA_X1_HK",
      "HEAT_FPHI_X1_HK",
      "HEAT_HTR3_X1_HK",
      "SERVO_PUMP_X1_HK",
      ),

  BITFIELD("heat_45_hk",
      "HEAT_PUMP_X5_HK",
      "HEAT_HSW_X5_HK",
      "HEAT_HTR2_X5_HK",
      "HEAT_HTR1_X5_HK",
      "HEAT_SSA_X5_HK",
      "HEAT_FPHI_X5_HK",
      "HEAT_HTR3_X5_HK",
      "SERVO_PUMP_X5_HK",
      "HEAT_PUMP_X4_HK",
      "HEAT_HSW_X4_HK",
      "HEAT_HTR2_X4_HK",
      "HEAT_HTR1_X4_HK",
      "HEAT_SSA_X4_HK",
      "HEAT_FPHI_X4_HK",
      "HEAT_HTR3_X4_HK",
      "SERVO_PUMP_X4_HK",
      ),

  BITFIELD("heat_26_hk",
      "HEAT_PUMP_X6_HK",
      "HEAT_HSW_X6_HK",
      "HEAT_HTR2_X6_HK",
      "HEAT_HTR1_X6_HK",
      "HEAT_SSA_X6_HK",
      "HEAT_FPHI_X6_HK",
      "HEAT_HTR3_X6_HK",
      "SERVO_PUMP_X6_HK",
      "HEAT_PUMP_X2_HK",
      "HEAT_HSW_X2_HK",
      "HEAT_HTR2_X2_HK",
      "HEAT_HTR1_X2_HK",
      "HEAT_SSA_X2_HK",
      "HEAT_FPHI_X2_HK",
      "HEAT_HTR3_X2_HK",
      "SERVO_PUMP_X2_HK",
      ),

  BITFIELD("heat_t_hk",
      "HEAT_MT_BOTTOM_T_HK",
      "HEAT_SFT_LINES_T_HK",
      "HEAT_CAPILLARY_T_HK",
      "HEAT_VCS2_HX_T_HK",
      "HEAT_VCS1_HX_T_HK",
      "HEAT_MT_LINES_T_HK",
      "HEAT_SFT_BOTTOM_T_HK",
      "",
      ),

  /*GONDOLA THERMISTOR CALIBRATION */
#define THERMISTOR(tch, vch) \
    LINTERP(tch, vch, LUT_DIR "thermistor.lut"), \
    UNITS(tch, "Temperature", "^oC")

  COMMENT("Thermistor calibrations"),
  THERMISTOR("T_GY", "VT_GY"),
  THERMISTOR("T_RSC", "VT_RSC"),
  THERMISTOR("T_MCC_POWER", "VT_MCC_POWER"),
  THERMISTOR("T_BSC", "VT_BSC"),
  THERMISTOR("T_SC_MOT", "VT_SC_MOT"),
  THERMISTOR("T_RW_MC", "VT_RW1"),
  THERMISTOR("T_RW_MOT", "VT_RW2"),
  THERMISTOR("T_LOCKPORT_MC", "VT_LOCKPORT_MC"),
  THERMISTOR("T_LOCKPORT_MOT", "VT_LOCKPORT_MOT"),
  THERMISTOR("T_LOCKSTAR_MC", "VT_LOCKSTAR_MC"),
  THERMISTOR("T_LOCKSTAR_MOT", "VT_LOCKSTAR_MOT"),
  THERMISTOR("T_DGPS", "VT_DGPS"),
  THERMISTOR("T_ELPORT_MOT", "VT_ELPORT_MOT"),
  THERMISTOR("T_ELSTAR_MOT", "VT_ELSTAR_MOT"),
  THERMISTOR("T_ELPORT_MC", "VT_ELPORT_MC"),
  THERMISTOR("T_MT_TAVCO", "VT_MT_TAVCO"),
  THERMISTOR("T_SFTV", "VT_SFTV"),
  THERMISTOR("T_AMBIENT1", "VT_AMBIENT1"),
  THERMISTOR("T_IF1", "VT_IF1"),
  THERMISTOR("T_IF2", "VT_IF2"),
  THERMISTOR("T_PSS", "VT_PSS"),
  THERMISTOR("T_FLOOR", "VT_FLOOR"),
  THERMISTOR("T_MYLAR_SUN", "VT_MYLAR_SUN"),
  THERMISTOR("T_MYLAR_IN", "VT_MYLAR_IN"),
  THERMISTOR("T_PIV", "VT_PIV"),
  THERMISTOR("T_SERIAL", "VT_SERIAL"),
  THERMISTOR("T_3_IF", "VT_3_IF"),
  THERMISTOR("T_4_IF", "VT_4_IF"),
  THERMISTOR("T_5_IF", "VT_5_IF"),
  THERMISTOR("T_6_IF", "VT_6_IF"),
  THERMISTOR("T_7_IF", "VT_7_IF"),


  //T_HK: Alias for LINTERP that sets UNITS.
#define T_HK(tch, vch, lut) \
  LINTERP(tch, vch, lut), \
  UNITS(tch, "Temperature", "K")
  //TD_HK: Alias for T_HK that sets UNITS. Use for insert/Theo diodes
  //  ch:   all-caps channel name
  //  i:    insert number
  //  lut:  filename of lut file converting V [volt] to T [K]
#define TD_HK(ch, i, lut) \
  T_HK("TD_"#ch"_"#i"_HK", "VD_"#ch"_"#i"_HK", lut)

  //P_HK: Alias for LINTERP that sets units. Use directly for pressure sensors
#define P_HK(pch, vch, lut) \
  LINTERP(pch, vch, lut), \
  UNITS(pch, "Pressure", "psi")

#define NTD_LUT LUT_DIR "r_ntd.lut"
#define CNX_LUT LUT_DIR "r_cernox.lut"
  //NTD_HK, CNX_HK: create fields required to calibrate NTD and CNX thermistors
  //  ch:   all-caps channel name
  //  i:    insert number
  //  lut:  filename of lut file converting R [ohm] to T [K]
#define NTD_HK(ch, i, lut) \
  DIVIDE("XR_"#ch"_"#i"_HK", "VR_"#ch"_"#i"_HK", "V_NTD_"#i"_HK"), \
  LINTERP("R_"#ch"_"#i"_HK", "XR_"#ch"_"#i"_HK", NTD_LUT), \
  UNITS("R_"#ch"_"#i"_HK", "Resistance", "\\\\Omega"), \
  T_HK("TR_"#ch"_"#i"_HK", "R_"#ch"_"#i"_HK", lut)
#define NTD_HK_NOLUT(ch, i) \
  DIVIDE("XR_"#ch"_"#i"_HK", "VR_"#ch"_"#i"_HK", "V_NTD_"#i"_HK"), \
  LINTERP("R_"#ch"_"#i"_HK", "XR_"#ch"_"#i"_HK", NTD_LUT), \
  UNITS("R_"#ch"_"#i"_HK", "Resistance", "\\\\Omega")
#define CNX_HK(ch, i, lut) \
  DIVIDE("XR_"#ch"_"#i"_HK", "VR_"#ch"_"#i"_HK", "V_CNX_"#i"_HK"), \
  LINTERP("R_"#ch"_"#i"_HK", "XR_"#ch"_"#i"_HK", CNX_LUT), \
  UNITS("R_"#ch"_"#i"_HK", "Resistance", "\\\\Omega"), \
  T_HK("TR_"#ch"_"#i"_HK", "R_"#ch"_"#i"_HK", lut)

  COMMENT("Housekeeping Cernox Temperature Calibration"),
  // NB: make sure to also set LUT filename in hk.c for FridgeCycle
  CNX_HK(STILL, X1, LUT_DIR "thelma3_still.lut"), // Thelma 3
  CNX_HK(FP,    X1, LUT_DIR "X80210.lut"),
  CNX_HK(RING,  X1, LUT_DIR "X58331.lut"),

  CNX_HK(STILL, X2, LUT_DIR "thelma4_still.lut"), // Thelma 4
  CNX_HK(FP,    X2, LUT_DIR "X41767.lut"),
  CNX_HK(RING,  X2, LUT_DIR "X40799.lut"),

  CNX_HK(STILL, X3, LUT_DIR "X42401.lut"),  // Thelma 8
  CNX_HK(FP,    X3, LUT_DIR "X58085.lut"),
  CNX_HK(RING,  X3, LUT_DIR "X82505.lut"),

  CNX_HK(STILL, X4, LUT_DIR "thelma7_still.lut"), // Thelma 7
  CNX_HK(FP,    X4, LUT_DIR "X41468.lut"),
  CNX_HK(RING,  X4, LUT_DIR "X56799.lut"),

  /*
  CNX_HK(STILL, X5, LUT_DIR "thelma2_still.lut"), // Thelma 2
  CNX_HK(FP,    X5, LUT_DIR ""),
  CNX_HK(RING,  X5, LUT_DIR ""),
  */

  CNX_HK(STILL, X6, LUT_DIR "thelma6_still.lut"), // Thelma 6??
  CNX_HK(FP,    X6, LUT_DIR "X41474.lut"),
  CNX_HK(RING,  X6, LUT_DIR "X58472.lut"),

  COMMENT("Housekeeping NTD Temperature Calibration"),
  NTD_HK_NOLUT(NTD1, X1),
  NTD_HK_NOLUT(NTD2, X1),
  NTD_HK_NOLUT(NTD3, X1),
  NTD_HK_NOLUT(NTD4, X1),

  NTD_HK(NTD1, X2, LUT_DIR "x2_ntd1.lut"),
  NTD_HK(NTD2, X2, LUT_DIR "x2_ntd2.lut"),
  NTD_HK(NTD3, X2, LUT_DIR "x2_ntd3.lut"),
  NTD_HK(NTD4, X2, LUT_DIR "x2_ntd4.lut"),

  NTD_HK_NOLUT(NTD1, X3),
  NTD_HK_NOLUT(NTD2, X3),
  NTD_HK_NOLUT(NTD3, X3),
  NTD_HK_NOLUT(NTD4, X3),

  NTD_HK_NOLUT(NTD1, X4),
  // NTD_HK_NOLUT(NTD2, X4), // bad NTD, not connected
  NTD_HK_NOLUT(NTD3, X4),
  NTD_HK_NOLUT(NTD4, X4),

  /*
  NTD_HK_NOLUT(NTD1, X5),
  NTD_HK_NOLUT(NTD2, X5),
  NTD_HK_NOLUT(NTD3, X5),
  NTD_HK_NOLUT(NTD4, X5),
  */

  NTD_HK_NOLUT(NTD1, X6),
  NTD_HK_NOLUT(NTD2, X6),
  NTD_HK_NOLUT(NTD3, X6),
  NTD_HK_NOLUT(NTD4, X6),

  COMMENT("Housekeeping Diode Temperature Calibration"),
  // NB: make sure to also set LUT filename in hk.c for FridgeCycle
  TD_HK(TRUSS,     X1, LUT_DIR "dt670.lut"),
  TD_HK(CP,        X1, LUT_DIR "thelma3_cp.lut"),
  TD_HK(PUMP,      X1, LUT_DIR "d_simonchase.lut"),
  TD_HK(HSW,       X1, LUT_DIR "d_simonchase.lut"),
  TD_HK(PLATE,     X1, LUT_DIR "D6032724.lut"),
  TD_HK(SNOUT,     X1, LUT_DIR "d_curve10.lut"),
  TD_HK(EYEPIECE,  X1, LUT_DIR "d_curve10.lut"),
  TD_HK(OBJECTIVE, X1, LUT_DIR "d_curve10.lut"),
  TD_HK(SPITTOON,  X1, LUT_DIR "D6032135.lut"),
  TD_HK(AUX_POST,  X1, LUT_DIR "dt670.lut"),
  TD_HK(STOP,      X1, LUT_DIR "D77249.lut"),
  TD_HK(SSA,       X1, LUT_DIR "D84461.lut"),

  TD_HK(TRUSS,     X2, LUT_DIR "dt670.lut"),
  TD_HK(CP,        X2, LUT_DIR "thelma4_cp.lut"),
  TD_HK(PUMP,      X2, LUT_DIR "d_simonchase.lut"),
  TD_HK(HSW,       X2, LUT_DIR "d_simonchase.lut"),
  TD_HK(PLATE,     X2, LUT_DIR "D6032131.lut"),
  TD_HK(SNOUT,     X2, LUT_DIR "d_curve10.lut"),
  TD_HK(EYEPIECE,  X2, LUT_DIR "D77241.lut"),
  TD_HK(OBJECTIVE, X2, LUT_DIR "d_curve10.lut"),
  TD_HK(SPITTOON,  X2, LUT_DIR "D78323.lut"),
  TD_HK(AUX_POST,  X2, LUT_DIR "D87587.lut"),
  TD_HK(STOP,      X2, LUT_DIR "D6032700.lut"),
  TD_HK(SSA,       X2, LUT_DIR "d_curve10.lut"),

  TD_HK(TRUSS,     X3, LUT_DIR "dt670.lut"),
  TD_HK(CP,        X3, LUT_DIR "d_simonchase.lut"),
  TD_HK(PUMP,      X3, LUT_DIR "d_simonchase.lut"),
  TD_HK(HSW,       X3, LUT_DIR "d_simonchase.lut"),
  TD_HK(PLATE,     X3, LUT_DIR "D6032722.lut"),
  TD_HK(SNOUT,     X3, LUT_DIR "d_curve10.lut"),
  TD_HK(EYEPIECE,  X3, LUT_DIR "d_curve10.lut"),
  TD_HK(OBJECTIVE, X3, LUT_DIR "d_curve10.lut"),
  TD_HK(SPITTOON,  X3, LUT_DIR "D6032142.lut"),
  TD_HK(AUX_POST,  X3, LUT_DIR "D6032429.lut"),
  TD_HK(STOP,      X3, LUT_DIR "D77240.lut"),
  TD_HK(SSA,       X3, LUT_DIR "d_curve10.lut"),

  // TD_HK(TRUSS,     X4, LUT_DIR "dt670.lut"), // diode reversed
  TD_HK(CP,        X4, LUT_DIR "d_simonchase.lut"),
  TD_HK(PUMP,      X4, LUT_DIR "d_simonchase.lut"),
  TD_HK(HSW,       X4, LUT_DIR "d_simonchase.lut"),
  TD_HK(PLATE,     X4, LUT_DIR "D6031471.lut"),
  TD_HK(SNOUT,     X4, LUT_DIR "dt670.lut"),
  TD_HK(EYEPIECE,  X4, LUT_DIR "d_curve10.lut"),
  TD_HK(OBJECTIVE, X4, LUT_DIR "d_curve10.lut"),
  TD_HK(SPITTOON,  X4, LUT_DIR "D6032718.lut"),
  TD_HK(AUX_POST,  X4, LUT_DIR "dt670.lut"),
  TD_HK(STOP,      X4, LUT_DIR "D77248.lut"),
  TD_HK(SSA,       X4, LUT_DIR "dt670.lut"),

  /*
  TD_HK(TRUSS,     X5, LUT_DIR "d_curve10.lut"),
  TD_HK(CP,        X5, LUT_DIR "d_simonchase.lut"),
  TD_HK(PUMP,      X5, LUT_DIR "d_simonchase.lut"),
  TD_HK(HSW,       X5, LUT_DIR "d_simonchase.lut"),
  TD_HK(PLATE,     X5, LUT_DIR "d_curve10.lut"),
  TD_HK(SNOUT,     X5, LUT_DIR "d_curve10.lut"),
  TD_HK(EYEPIECE,  X5, LUT_DIR "d_curve10.lut"),
  TD_HK(OBJECTIVE, X5, LUT_DIR "d_curve10.lut"),
  TD_HK(SPITTOON,  X5, LUT_DIR "d_curve10.lut"),
  TD_HK(AUX_POST,  X5, LUT_DIR "d_curve10.lut"),
  TD_HK(STOP,      X5, LUT_DIR "d_curve10.lut"),
  TD_HK(SSA,       X5, LUT_DIR "d_curve10.lut"),
  */

  TD_HK(TRUSS,     X6, LUT_DIR "dt670.lut"),
  TD_HK(CP,        X6, LUT_DIR "d_simonchase.lut"),
  TD_HK(PUMP,      X6, LUT_DIR "d_simonchase.lut"),
  TD_HK(HSW,       X6, LUT_DIR "d_simonchase.lut"),
  TD_HK(PLATE,     X6, LUT_DIR "D6032136.lut"),
  TD_HK(SNOUT,     X6, LUT_DIR "dt670.lut"),
  TD_HK(EYEPIECE,  X6, LUT_DIR "dt670.lut"),
  TD_HK(OBJECTIVE, X6, LUT_DIR "dt670.lut"),
  TD_HK(SPITTOON,  X6, LUT_DIR "D6032721.lut"),
  TD_HK(AUX_POST,  X6, LUT_DIR "dt670.lut"),
  TD_HK(STOP,      X6, LUT_DIR "D77243.lut"),
  TD_HK(SSA,       X6, LUT_DIR "dt670.lut"),
  
  //Theo diode calibrations.
  TD_HK(MT_TOPHI,    T, LUT_DIR "D6032723.lut"),
  TD_HK(VCS1_BOTTOM, T, LUT_DIR "d_curve10.lut"),
  TD_HK(MT_BOTLO2,   T, LUT_DIR "d_curve10.lut"),
  TD_HK(VCS2_FILTER, T, LUT_DIR "d_curve10.lut"),
  TD_HK(SFT_NOSE,    T, LUT_DIR "D78318.lut"),
  TD_HK(VCS2_BOTTOM, T, LUT_DIR "d_curve10.lut"),
  TD_HK(VCS1_FILTER, T, LUT_DIR "d_curve10.lut"),
  TD_HK(CAPILLARY,   T, LUT_DIR "D77232.lut"),
  TD_HK(VCS2_TOP,    T, LUT_DIR "d_curve10.lut"),
  TD_HK(MT_BOTLO,    T, LUT_DIR "D75551.lut"),
  TD_HK(MT_BOTHI,    T, LUT_DIR "D78016.lut"),
  TD_HK(SFT_BOTTOM,  T, LUT_DIR "D77239.lut"),
  TD_HK(VCS1_TOP,    T, LUT_DIR "d_curve10.lut"),
  TD_HK(MT_TOPLO,    T, LUT_DIR "D78317.lut"),
  TD_HK(CAPILLARY2,  T, LUT_DIR "D75559.lut"),
  TD_HK(VCS2_FLEX,   T, LUT_DIR "d_curve10.lut"),
  TD_HK(SFT_RING,    T, LUT_DIR "D75322.lut"),
  TD_HK(VCS1_APERT,  T, LUT_DIR "d_curve10.lut"),

  //Pressure sensor calibration
  P_HK("P_MT_VENT_T_HK",  "VP_01_HK",   LUT_DIR "pressure_tube.lut"),
  P_HK("P_SFT_VENT_T_HK", "VP_02_HK",   LUT_DIR "pressure_omega1.lut"),

  /* Field sets */
  BITWORD("BSET_NUM", "bset", 0, 8),
  BITWORD("BSET_SER", "bset", 8, 8),

  /* MCE Power Banks */
  BITFIELD("mce_power",
      "MCE2_MCE3_OFF",
      "MCE4_MCE6_OFF",
      "MCE1_MCE5_OFF",
      ""
      ),

  /* att ok bits */
  BITFIELD("att_ok",
           "ATT_OK_DGPS",
           "ATT_OK_MAG",
           "ATT_OK_PSS",
           ""),
  
  LINCOM2("I_IF_TOT", "I_MCE", 1.0, 0, "I_HK_MISC",  1, 0),
  
  LINCOM2("TWIST_EL", "EL_RAW_1_ENC", 1.0, 0.0, "EL_RAW_2_ENC",  -1.0, 0.0),

  BITWORD( "GOAL_MPC1", "dtg_mpc1", 8, 8),
  BITWORD("DTASK_MPC1", "dtg_mpc1", 0, 8),
  BITWORD( "GOAL_MPC2", "dtg_mpc2", 8, 8),
  BITWORD("DTASK_MPC2", "dtg_mpc2", 0, 8),
  BITWORD( "GOAL_MPC3", "dtg_mpc3", 8, 8),
  BITWORD("DTASK_MPC3", "dtg_mpc3", 0, 8),
  BITWORD( "GOAL_MPC4", "dtg_mpc4", 8, 8),
  BITWORD("DTASK_MPC4", "dtg_mpc4", 0, 8),
  BITWORD( "GOAL_MPC5", "dtg_mpc5", 8, 8),
  BITWORD("DTASK_MPC5", "dtg_mpc5", 0, 8),
  BITWORD( "GOAL_MPC6", "dtg_mpc6", 8, 8),
  BITWORD("DTASK_MPC6", "dtg_mpc6", 0, 8),

  /* data_mode_bits is:
   * .UUUUUuuuuuLLLLL
   *
   * U = upper start bit
   * u = upper nbits
   * L = lower start bit
   * also: lower nbits = 16 - upper nbits
   */
  BITWORD("UPPER_START_DMB", "data_mode_bits", 10, 5),
  BITWORD("UPPER_NBITS_DMB", "data_mode_bits",  5, 5),
  BITWORD("LOWER_START_DMB", "data_mode_bits",  0, 5),
  LINCOM("LOWER_NBITS_DMB", "UPPER_NBITS_DMB", -1, 16),

  END_OF_DERIVED_CHANNELS
};
