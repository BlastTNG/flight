/* command_list.c: Spider command specification file
 *
 * This software is copyright (C) 2002-20010 University of Toronto
 *
 * This file is part of the Spider flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 *
 * IF YOU ADD, MODIFY, OR DELETE *ANY* COMMANDS IN THIS FILE YOU *MUST*
 * RECOMPILE AND REINSTALL BLASTCMD ON ARWEN/WIDOW/!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include <limits.h>
#include <stdio.h>

#include "tes.h"
#include "command_list.h"
#ifdef __MCP__
#include "camstruct.h"
#endif

#include "mce_counts.h"

const char *const command_list_serial = "$Rev$";

/* parse the above; returns -1 if command_list_serial can't be parsed */
const int command_list_serial_as_int(void)
{
  int cmd_rev = -1;
  sscanf(command_list_serial, "$R" /* prevent SVN from munging this string */
      "ev: %i $", &cmd_rev);

  return cmd_rev;
}

#define CHOOSE_INSERT_PARAM "Insert", 0, 6, 'i', "NONE", {mce_names}, 1
#define CHOOSE_INSERT_NO_ALL "Insert", 1, 6, 'i',"NONE", {mce_names + 1}, 2
#define MCE_ACTION_PARAM(n,w) "Action", 0, n, 'i', "NONE", {w}

#define MCECMD1(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 1, { \
      {CHOOSE_INSERT_PARAM}, \
    }

#define MCECMD1P(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 1, { \
      {CHOOSE_INSERT_NO_ALL}, \
    }

#define MCECMD1AD(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 2, { \
      {CHOOSE_INSERT_PARAM}, \
      {MCE_ACTION_PARAM(4,daction_names)}, \
    }

#define MCECMD2(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 2, { \
      {CHOOSE_INSERT_PARAM}, \
      {pname, min, max, typ, "NONE"}, \
    }

#define MCECMD2P(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 2, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {pname, min, max, typ, "NONE"}, \
    }

#define MCECMD2A(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 3, { \
      {CHOOSE_INSERT_PARAM}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMD2AP(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 3, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMDC(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 3, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
    }

#define MCECMDCA(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMDCAD(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(4,daction_names)}, \
    }

#define MCECMDRAD(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Row", 0, 32, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(4,daction_names)}, \
    }

#define MCECMDCR(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {"Row", 0, 32, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
    }

#define MCECMDCRA(cmd,desc,grp,pname,min,max,typ) \
    COMMAND(cmd), desc, grp | MCECMD, 5, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {"Row", 0, 32, 'i', "NONE"}, \
      {pname, min, max, typ, "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMDCR1A(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_NO_ALL}, \
      {"Column", 0, 15, 'i', "NONE"}, \
      {"Row", 0, 32, 'i', "NONE"}, \
      {MCE_ACTION_PARAM(3,action_names)}, \
    }

#define MCECMDSCS(cmd,desc,grp) \
    COMMAND(cmd), desc, grp | MCECMD, 4, { \
      {CHOOSE_INSERT_PARAM}, \
      {"Start", -32768, 32767, 'i', "NONE"}, \
      {"Count", 0, 65535, 'i', "NONE"}, \
      {"Step", -32768, 32767, 'i', "NONE"}, \
    }

const char *const GroupNames[N_GROUPS] = {
  "Pointing Modes",        "Aux. Electronics", "Waveplate Rotator",
  "Pointing Sensor Trims", "SFT Valve Motors", "HK Bias",
  "Pointing Sensor Vetos", "Lock Motor",       "Gyro Power",
  "Pointing Motor Gains",  "SC Table",         "HK Insert Heat",
  "ACS Power",             "The Good SC",      "HK Theo Heat",
  "Telemetry",             "The Bad SC",       "Cryo/HK Power",
  "MCE Power",             "The Ugly SC",      "MPC Control",
  "Tuning Params",         "Detectors",        "Load Curves",
  "Miscellaneous",         "CMB Grenades",     "Sync Box"
  };

//echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

/* parameter value lists */
const char *autonoyes_names[] = {"auto", "no", "yes", NULL};
const char *noyes_names[] = {"no", "yes", NULL};
const char *kick_names[] = {"none", "1 Volt", "2 Volts", NULL};
const char *mce_names[] = {"all", "X1", "X2", "X3", "X4", "X5", "X6", NULL};
const char *just_mce_names[] = {"X1", "X2", "X3", "X4", "X5", "X6", NULL};
const char *tuning_stages[] = {"SA ramp", "SQ2 servo", "SQ1 servo", "SQ1 ramp",
  "SQ1 ramp TES", NULL};
const char *wb_cards[] = {"CC", "RC1", "RC2", "BC1", "BC2", "AC", NULL};
const char *action_names[] = {"Apply & Record", "Apply only",
  "Record & Reconfig", "Record only", NULL};
const char *daction_names[] = {"Apply & Record", "Apply only",
  "Record & Reconfig", "Record only", "Record default only", NULL};
const char *tunedata_names[] = {"expt.cfg", "sqtune"};

const struct scom scommands[N_SCOMMANDS] = {
  {COMMAND(stop), "servo off of gyros to zero speed now", GR_POINT},
  {COMMAND(antisun), "turn antisolar now", GR_POINT},

  {COMMAND(gps_off), "turn off the dGPS", GR_POWER | CONFIRM},
  {COMMAND(gps_on), "turn on the dGPS", GR_POWER},
  {COMMAND(gps_cycle), "power cycle the dGPS", GR_POWER | CONFIRM},
  {COMMAND(gybox_off), "turn off the digital gyros' box", GR_POWER | GR_GYPWR},
  {COMMAND(gybox_on), "turn on the digital gyros' box", GR_POWER | GR_GYPWR},
  {COMMAND(gybox_cycle), "power cycle the digital gyros' box", 
   GR_POWER | GR_GYPWR},
  {COMMAND(ofroll_1_gy_off), "turn off ofroll_1_gy", GR_GYPWR},
  {COMMAND(ofroll_1_gy_on), "turn on ofroll_1_gy", GR_GYPWR},
  {COMMAND(ofroll_1_gy_cycle), "power cycle ofroll_1_gy", GR_GYPWR},
  {COMMAND(ofroll_2_gy_off), "turn off ofroll_2_gy", GR_GYPWR},
  {COMMAND(ofroll_2_gy_on), "turn on ofroll_2_gy", GR_GYPWR},
  {COMMAND(ofroll_2_gy_cycle), "power cycle ofroll_2_gy", GR_GYPWR},
  {COMMAND(ofyaw_1_gy_off), "turn off ofyaw_1_gy", GR_GYPWR},
  {COMMAND(ofyaw_1_gy_on), "turn on ofyaw_1_gy", GR_GYPWR},
  {COMMAND(ofyaw_1_gy_cycle), "power cycle ofyaw_1_gy", GR_GYPWR},
  {COMMAND(ofyaw_2_gy_off), "turn off ofyaw_2_gy", GR_GYPWR},
  {COMMAND(ofyaw_2_gy_on), "turn on ofyaw_2_gy", GR_GYPWR},
  {COMMAND(ofyaw_2_gy_cycle), "power cycle ofyaw_2_gy", GR_GYPWR},
  {COMMAND(ofpch_1_gy_off), "turn off ofpch_1_gy", GR_GYPWR},
  {COMMAND(ofpch_1_gy_on), "turn on ofpch_1_gy", GR_GYPWR},
  {COMMAND(ofpch_1_gy_cycle), "power cycle ofpch_1_gy", GR_GYPWR},
  {COMMAND(ofpch_2_gy_off), "turn off ofpch_2_gy", GR_GYPWR},
  {COMMAND(ofpch_2_gy_on), "turn on ofpch_2_gy", GR_GYPWR},
  {COMMAND(ofpch_2_gy_cycle), "power cycle ofpch_2_gy", GR_GYPWR},
  {COMMAND(rw_off), "turn off the reaction wheel motor", GR_POWER},
  {COMMAND(rw_on), "turn on the reaction wheel motor", GR_POWER},
  {COMMAND(rw_cycle), "power cycle the reaction wheel motor", GR_POWER},
  {COMMAND(piv_off), "turn off the pivot motor", GR_POWER},
  {COMMAND(piv_on), "turn on the pivot motor", GR_POWER},
  {COMMAND(piv_cycle), "power cycle the pivot motor", GR_POWER},
  {COMMAND(elmot_off), "turn off the elevation motor", GR_POWER},
  {COMMAND(elmot_on), "turn on the elevation motor", GR_POWER},
  {COMMAND(elmot_cycle), "power cycle the elevation motor", GR_POWER},
  {COMMAND(elmot_auto), "el motors automatically power on for a move, "
    "and then power off again", GR_POWER},
  {COMMAND(vtx_off), "turn off the video transmitters", GR_TELEM | GR_POWER},
  {COMMAND(vtx_on), "turn on the video transmitters", GR_TELEM | GR_POWER},
  {COMMAND(bi0_off), "turn off the biphase transmitter", GR_TELEM | GR_POWER},
  {COMMAND(bi0_on), "turn on the biphase transmitter", GR_TELEM | GR_POWER},
  {COMMAND(table_off), "turn off the SC rotary table", GR_POWER | CONFIRM},
  {COMMAND(table_on), "turn on the SC rotary table", GR_POWER},
  {COMMAND(table_cycle), "power cycle the SC rotary table", GR_POWER | CONFIRM},
  {COMMAND(rsc_off), "turn off the rotating star cameras", GR_POWER | CONFIRM},
  {COMMAND(rsc_on), "turn on the rotating star cameras", GR_POWER},
  {COMMAND(rsc_cycle), "power cycle the rotating star cameras", GR_POWER |
    CONFIRM},
  {COMMAND(bsc_off), "turn off the boresight star camera", GR_POWER | CONFIRM},
  {COMMAND(bsc_on), "turn on the boresight star camera", GR_POWER},
  {COMMAND(bsc_cycle), "power cycle the boresight star camera", GR_POWER |
    CONFIRM},
  {COMMAND(hub232_off), "turn off the RS-232 (serial) hub", GR_POWER},
  {COMMAND(hub232_on), "turn on the RS-232 (serial) hub", GR_POWER},
  {COMMAND(hub232_cycle), "power cycle the RS-232 (serial) hub", GR_POWER},
  {COMMAND(das_off), "turn off the DAS", GR_IFPOWER},
  {COMMAND(das_on), "turn on the DAS", GR_IFPOWER},
  {COMMAND(das_cycle), "power cycle the DAS", GR_IFPOWER},
  {COMMAND(of_charge_off), "turn off the outer frame charge controller",
    GR_POWER | CONFIRM},
  {COMMAND(of_charge_on), "turn on the outer frame charge controller",
    GR_POWER},
  {COMMAND(if_charge_off), "turn off the inner frame charge controller",
    GR_IFPOWER | CONFIRM},
  {COMMAND(if_charge_on), "turn on the inner frame charge controller",
    GR_IFPOWER},
  {COMMAND(of_charge_cycle), "power cycle the outer frame charge controller", 
    GR_POWER | CONFIRM},
  {COMMAND(if_charge_cycle), "power cycle the inner frame charge controller", 
    GR_IFPOWER | CONFIRM},

  {COMMAND(mce23_on), "turn on MCE power supply for X2 and X3", GR_MCEPWR},
  {COMMAND(mce23_off), "turn off MCE power supply for X2 and X3", GR_MCEPWR},
  {COMMAND(mce23_cycle), "power cycle MCE power supply for X2 and X3",
    GR_MCEPWR},

  {COMMAND(mce46_on), "turn on MCE power supply for X4 and X6", GR_MCEPWR},
  {COMMAND(mce46_off), "turn off MCE power supply for X4 and X6", GR_MCEPWR},
  {COMMAND(mce46_cycle), "power cycle MCE power supply for X4 and X6",
    GR_MCEPWR},

  {COMMAND(mce15_on), "turn on MCE power supply for X1 and X5", GR_MCEPWR},
  {COMMAND(mce15_off), "turn off MCE power supply for X1 and X5", GR_MCEPWR},
  {COMMAND(mce15_cycle), "power cycle MCE power supply for X1 and X5",
    GR_MCEPWR},

  {COMMAND(mcc1_on), "turn on MCC 1", GR_MCEPWR},
  {COMMAND(mcc1_off), "turn off MCC 1", GR_MCEPWR},
  {COMMAND(mcc1_cycle), "power cycle MCC 1", GR_MCEPWR},

  {COMMAND(mcc2_on), "turn on MCC 2", GR_MCEPWR},
  {COMMAND(mcc2_off), "turn off MCC 2", GR_MCEPWR},
  {COMMAND(mcc2_cycle), "power cycle MCC 2", GR_MCEPWR},

  {COMMAND(mcc3_on), "turn on MCC 3", GR_MCEPWR},
  {COMMAND(mcc3_off), "turn off MCC 3", GR_MCEPWR},
  {COMMAND(mcc3_cycle), "power cycle MCC 3", GR_MCEPWR},
  
  {COMMAND(mcc4_on), "turn on MCC 4", GR_MCEPWR},
  {COMMAND(mcc4_off), "turn off MCC 4", GR_MCEPWR},
  {COMMAND(mcc4_cycle), "power cycle MCC 4", GR_MCEPWR},
  
  {COMMAND(mcc5_on), "turn on MCC 5", GR_MCEPWR},
  {COMMAND(mcc5_off), "turn off MCC 5", GR_MCEPWR},
  {COMMAND(mcc5_cycle), "power cycle MCC 5", GR_MCEPWR},
  
  {COMMAND(mcc6_on), "turn on MCC 6", GR_MCEPWR},
  {COMMAND(mcc6_off), "turn off MCC 6", GR_MCEPWR},
  {COMMAND(mcc6_cycle), "power cycle MCC 6", GR_MCEPWR},

  {COMMAND(sync_on), "turn on the sync box", GR_MCEPWR},
  {COMMAND(sync_off), "turn off the sync box", GR_MCEPWR},
  {COMMAND(sync_cycle), "power cycle the sync box", GR_MCEPWR},
  
  {COMMAND(hwp_on), "turn on the HWP rotators", GR_IFPOWER},
  {COMMAND(hwp_off), "turn off the HWP rotators", GR_IFPOWER},
  {COMMAND(hwp_cycle), "power cycle HWP rotators", GR_IFPOWER},
  
  {COMMAND(hk_preamp_on), "turn on the HK preamp crate", GR_IFPOWER},
  {COMMAND(hk_preamp_off), "turn off the HK preamp crate", GR_IFPOWER},
  {COMMAND(hk_preamp_cycle), "power cycle the HK preamp crate", GR_IFPOWER},
  
  {COMMAND(reset_rw), "reset the serial connection to the RW controller",
    GR_GAIN},
  {COMMAND(reset_piv), "reset the serial connection to the pivot controller",
    GR_GAIN},
  {COMMAND(reset_elev), "reset the serial connection to the elev controller",
    GR_GAIN},
  {COMMAND(restore_piv), "restore the serial settings for the pivot controller",
    GR_GAIN},
  {COMMAND(az_disable), "disable az motors' gains", GR_GAIN},
  {COMMAND(az_enable), "enable az motors' gains", GR_GAIN},
  {COMMAND(el_disable), "disable el motor gains", GR_GAIN},
  {COMMAND(el_enable), "enable el motor gains", GR_GAIN},
  {COMMAND(force_el_enable), "force enable el motors despite the pin being in",
    CONFIRM | GR_GAIN},

  {COMMAND(elenc1_veto), "veto elevation encoder 1", GR_VETO},
  {COMMAND(elenc2_veto), "veto elevation encoder 2", GR_VETO},
  {COMMAND(elenc1_allow), "un-veto elevation encoder 1", GR_VETO},
  {COMMAND(elenc2_allow), "un-veto elevation encoder 2", GR_VETO},
  {COMMAND(gps_veto), "veto differntial gps", GR_VETO},
  {COMMAND(gps_allow), "un-veto differential gps", GR_VETO},
  {COMMAND(mag_veto), "veto magnotometer", GR_VETO},
  {COMMAND(mag_allow), "un-veto magnetometer", GR_VETO},
  {COMMAND(pss_veto), "veto pss sensor", GR_VETO},
  {COMMAND(pss_allow), "un-veto pss sensor", GR_VETO},
  {COMMAND(ofroll_1_gy_allow), "enable ofroll_1_gy", GR_VETO},
  {COMMAND(ofroll_1_gy_veto), "disable ofroll_1_gy", GR_VETO},
  {COMMAND(ofroll_2_gy_allow), "enable ofroll_2_gy", GR_VETO},
  {COMMAND(ofroll_2_gy_veto), "disable ofroll_2_gy", GR_VETO},
  {COMMAND(ofyaw_1_gy_allow), "enable ofyaw_1_gy", GR_VETO},
  {COMMAND(ofyaw_1_gy_veto), "disable ofyaw_1_gy", GR_VETO},
  {COMMAND(ofyaw_2_gy_allow), "enable ofyaw_2_gy", GR_VETO},
  {COMMAND(ofyaw_2_gy_veto), "disable ofyaw_2_gy", GR_VETO},
  {COMMAND(ofpch_1_gy_allow), "enable ofpch_1_gy", GR_VETO},
  {COMMAND(ofpch_1_gy_veto), "disable ofpch_1_gy", GR_VETO},
  {COMMAND(ofpch_2_gy_allow), "enable ofpch_2_gy", GR_VETO},
  {COMMAND(ofpch_2_gy_veto), "disable ofpch_2_gy", GR_VETO},

  {COMMAND(az_auto_gyro), "automatically calculate az gyro offsets", GR_TRIM},
  {COMMAND(el_auto_gyro), "automatically calculate el gyro offset", GR_TRIM},
  {COMMAND(reset_trims), "reset coarse pointing trims to zero", GR_TRIM},
  {COMMAND(trim_to_isc), "trim coarse sensors to ISC", GR_TRIM},
  {COMMAND(trim_to_osc), "trim coarse sensors to OSC", GR_TRIM},

  {COMMAND(blast_rocks), "the receiver rocks, use the happy schedule file",
    GR_TELEM},
  {COMMAND(blast_sucks), "the receiver sucks, use the sad schedule file",
    GR_TELEM},
  {COMMAND(at_float),
    "tell the scheduler that we're at float (don't run initial float controls)",
    GR_TELEM},
  {COMMAND(not_at_float), "tell the scheduler that we're not at float",
    GR_TELEM},
  {COMMAND(vtx1_isc), "put ISC video on transmitter #1", GR_TELEM},
  {COMMAND(vtx1_osc), "put OSC video on transmitter #1", GR_TELEM},
  {COMMAND(vtx1_bsc), "put BSC video on transmitter #1", GR_TELEM},
  {COMMAND(vtx2_isc), "put ISC video on transmitter #2", GR_TELEM},
  {COMMAND(vtx2_osc), "put OSC video on transmitter #2", GR_TELEM},
  {COMMAND(vtx2_bsc), "put BSC video on transmitter #2", GR_TELEM},

  {COMMAND(halt_itsy), "ask MCP to halt *ITSY* MCC", GR_MISC | CONFIRM},
  {COMMAND(halt_bitsy), "ask MCP to halt *BITSY* MCC", GR_MISC | CONFIRM},
  {COMMAND(reap_itsy), "ask MCP to reap the *ITSY* watchdog tickle", 
    GR_MISC | CONFIRM},
  {COMMAND(reap_bitsy), "ask MCP to reap the *BITSY* watchdog tickle", 
    GR_MISC | CONFIRM},
  {COMMAND(bbc_sync_ext), "Set BBC to external (sync box) mode", 
    GR_MISC | CONFIRM},
  {COMMAND(bbc_sync_int), "Set BBC to internal sync mode", GR_MISC | CONFIRM},
  {COMMAND(bbc_sync_auto),
    "Auto-set BBC to external (sync box) mode if possible", GR_MISC | CONFIRM},
  {COMMAND(pin_in), "close lock pin without checking encoder (dangerous)",
    GR_LOCK | CONFIRM},
  {COMMAND(lock), "lock inner frame", GR_LOCK | GR_POINT},
  {COMMAND(unlock), "unlock the inner frame", GR_LOCK},
  {COMMAND(lock_on), "turn on the lock motor", GR_LOCK},
  {COMMAND(lock_off), "turn off the lock motor", GR_LOCK},
  {COMMAND(sftv_on), "turn on the SFT valve motors", GR_SFTV},
  {COMMAND(sftv_off), "turn off the SFT valve motors", GR_SFTV},
  {COMMAND(sftv_atm_open), "open the SFT to atmosphere", GR_SFTV | CONFIRM},
  {COMMAND(sftv_atm_close), "close the SFT to atmosphere", GR_SFTV | CONFIRM},
  {COMMAND(sftv_pump_open), "open the SFT to its pump", GR_SFTV | CONFIRM},
  {COMMAND(sftv_pump_close), "close the SFT to its pump", GR_SFTV | CONFIRM},
  {COMMAND(hwp_repoll), "repoll HWP bus for stepper controllers", GR_HWPR},
  {COMMAND(hwp_panic), "stop all HWP rotators immediately", GR_HWPR},
  {COMMAND(hwp_step), "step the HWPs to their next position", GR_HWPR},

  //The Good commands
  {COMMAND(thegood_expose), "Start The Good exposure (in triggered mode)",
    GR_SCGOOD},
  {COMMAND(thegood_autofocus), "The Good autofocus mode", GR_SCGOOD},
  {COMMAND(thegood_settrig_ext), "Set external The Good trigger mode",
    GR_SCGOOD},
  {COMMAND(thegood_pause), "Stop automatic image capture", GR_SCGOOD},
  {COMMAND(thegood_run), "Start automatic image capture", GR_SCGOOD},
  //The Bad commands
  {COMMAND(thebad_expose), "Start The Bad exposure (in triggered mode)",
    GR_SCBAD},
  {COMMAND(thebad_autofocus), "The Bad autofocus mode", GR_SCBAD},
  {COMMAND(thebad_settrig_ext), "Set external The Bad trigger mode", GR_SCBAD},
  {COMMAND(thebad_pause), "Stop automatic image capture", GR_SCBAD},
  {COMMAND(thebad_run), "Start automatic image capture", GR_SCBAD},
  //The Ugly commands
  {COMMAND(theugly_expose), "Start The Ugly exposure (in triggered mode)",
    GR_SCUGLY},
  {COMMAND(theugly_autofocus), "The Ugly autofocus mode", GR_SCUGLY},
  {COMMAND(theugly_settrig_ext), "Set external The Ugly trigger mode",
    GR_SCUGLY},
  {COMMAND(theugly_pause), "Stop automatic image capture", GR_SCUGLY},
  {COMMAND(theugly_run), "Start automatic image capture", GR_SCUGLY},
  //Star Camera table
  {COMMAND(table_track), "Put the table in track mode", GR_SCTAB},
  //Theo heater housekeeping commands
  {COMMAND(hk_mt_bottom_heat_on), 
   "Turn on Theo's MT Bottom heaters. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_mt_bottom_heat_off), 
   "Turn off Theo's MT Bottom heaters. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_sft_lines_heat_on), 
   "Turn on Theo's SFT fill and vent line heaters. Disable pulse",
   GR_THEO_HEAT},
  {COMMAND(hk_sft_lines_heat_off), 
   "Turn off Theo's SFT fill and vent line heaters. Disable pulse",
   GR_THEO_HEAT},
  {COMMAND(hk_capillary_heat_on), 
   "Turn on Theo's capillary heater. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_capillary_heat_off), 
   "Turn off Theo's capillary heater. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_vcs2_hx_heat_on), 
   "Turn on Theo's VCS2 HX heaters. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_vcs2_hx_heat_off), 
   "Turn off Theo's VCS2 HX heaters. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_vcs1_hx_heat_on),
   "Turn on Theo's VCS1 HX heaters. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_vcs1_hx_heat_off),
   "Turn off Theo's VCS1 HX heaters. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_mt_lines_heat_on),
   "Turn on Theo's MT fill and vent line heaters. Disable pulse", 
   GR_THEO_HEAT},
  {COMMAND(hk_mt_lines_heat_off),
   "Turn off Theo's MT fill and vent line heaters. Disable pulse",
   GR_THEO_HEAT},
  {COMMAND(hk_sft_bottom_heat_on),
   "Turn on Theo's SFT Bottom Heater. Disable pulse", GR_THEO_HEAT},
  {COMMAND(hk_sft_bottom_heat_off),
   "Turn off Theo's SFT Bottom Heater. Disable pulse", GR_THEO_HEAT},

  //make better use of unused groups
  {COMMAND(pull_cmb_pin), "????", GR_CMB | CONFIRM},
  {COMMAND(global_thermonuclear_war), "Shall we play a game?",
    GR_CMB | CONFIRM},

  {COMMAND(mcc_wdog_enable), "Enable pcm watchdog of MCCs", GR_MCEPWR},
  {COMMAND(mcc_wdog_disable), "Disable pcm watchdog of MCCs", GR_MCEPWR},
  {COMMAND(get_superslow), "Re-fetch the super-slow data from MPC", GR_MPC},

  {COMMAND(xyzzy), "nothing happens here", GR_MISC}
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * l :  parameter is 30 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * d :  parameter is 30 bit renormalised floating point
 * s :  parameter is 7-bit character string
 */
const struct mcom mcommands[N_MCOMMANDS] = {
  {COMMAND(dac2_level), "*UNUSED* DAC2 output level. Does nothing", GR_MISC, 1,
    {
      {"Level", 0, 32767, 'i', "dac2_ampl"}
    }
  },

  {COMMAND(slot_sched), "set uplinked slot to use for schedule file",
    GR_TELEM, 1,
    {
      {"Slot #", 0, 250, 'i', "SLOT_SCHED"}
    }
  },

  /* pointing modes */
  {COMMAND(az_el_goto), "goto point in azimuth and elevation", GR_POINT, 2,
    {
      {"Azimuth (deg)",     -360, 360, 'f', "AZ"},
      {"Elevation (deg)", 15.0,  45.0, 'f', "EL"}
    }
  },
  {COMMAND(az_el_trim), "trim sensors to azimuth and elevation", GR_TRIM, 2,
    {
      {"Azimuth (deg)", 0, 360, 'f', "AZ"},
      {"Elevation (deg)", 0, 90, 'f', "EL"}
    }
  },
  
  {COMMAND(mag_cal), "set magnetometer calibration", GR_TRIM, 4,
    {
      {"Max X", -65535, 65535, 'f', "cal_xmax_mag"},
      {"Min X", -65535, 65535, 'f', "cal_xmin_mag"},
      {"Max Y", -65535, 65535, 'f', "cal_ymax_mag"},
      {"Min Y", -65535, 65535, 'f', "cal_ymin_mag"}
    }
  }, // 10 10 10.5 10.34
  
  {COMMAND(pss_off_cal), "set pss offsets", GR_TRIM, 6,
    {
      {"Offset 1", -20.0, 20.0, 'f', "CAL_OFF_PSS1"},
      {"Offset 2", -20.0, 20.0, 'f', "CAL_OFF_PSS2"},
      {"Offset 3", -20.0, 20.0, 'f', "CAL_OFF_PSS3"},
      {"Offset 4", -20.0, 20.0, 'f', "CAL_OFF_PSS4"},
      {"Offset 5", -20.0, 20.0, 'f', "CAL_OFF_PSS5"},
      {"Offset 6", -20.0, 20.0, 'f', "CAL_OFF_PSS6"},
    }
  },

  {COMMAND(pss_d_cal), "set pss distances and Imin", GR_TRIM, 7,
    {
      {"Distance 1", -2.0, 2.0, 'f', "CAL_D_PSS1"},
      {"Distance 2", -2.0, 2.0, 'f', "CAL_D_PSS2"},
      {"Distance 3", -2.0, 2.0, 'f', "CAL_D_PSS3"},
      {"Distance 4", -2.0, 2.0, 'f', "CAL_D_PSS4"},
      {"Distance 5", -2.0, 2.0, 'f', "CAL_D_PSS5"},
      {"Distance 6", -2.0, 2.0, 'f', "CAL_D_PSS6"},
      {"I Min", 0.0, 20.0, 'f', "CAL_IMIN_PSS"}
    }
  },

  {COMMAND(az_gain), "az reaction wheel gains", GR_GAIN, 3,
    {
      {"Proportional Gain", 0, USHRT_MAX, 'i', "g_p_az"},
      {"Integral Gain",     0, USHRT_MAX, 'i', "g_i_az"},
      {"Pointing Gain", 0, USHRT_MAX, 'i', "g_pt_az"}
    }
  },
  {COMMAND(az_scan), "scan in azimuth", GR_POINT, 4,
    {
      {"Az centre (deg)",       -180, 360, 'f', "AZ"},
      {"El centre (deg)",         20,  45, 'f', "EL"},
      {"Width (deg on sky)",       0, 360, 'f', "W_P"},
      {"Az Scan Speed (deg az/s)", 0,  10, 'f', "VEL_AZ_P"}
    }
  },
  {COMMAND(drift), "move at constant speed in az", GR_POINT, 1,
    {
      {"Az Speed (deg/s)", -10.0, 10.0, 'f', "0.0"}
    }
  },
  {COMMAND(ra_dec_goto), "track a location RA/Dec", GR_POINT, 2,
    {
      {"RA of Centre (h)",      0, 24, 'f', "RA"},
      {"Dec of Centre (deg)", -90, 90, 'f', "DEC"}
    }
  },
  {COMMAND(set_scan_params), "set common scan parameters for flight", 
   GR_POINT, 7,
    {
      {"Az Scan Accel (deg/s^2)",   0,  2, 'f', "ACCEL_AZ"},
      {"Az MAX Acceleration (deg/s^2)",  0.0, 100.0, 'f', "ACCEL_MAX_AZ"},
      {"Number of half-scans per el microstep", 1, 100, 'i', "N_SCAN_PER_STEP"},
      {"El microstep size (deg)", 0.0, 1.0, 'f', "SIZE_EL_STEP"},
      {"Total number of el microsteps", 1, 500, 'i', "N_EL_STEPS"},
      {"Az Overshoot Band (deg)", 0.0, 30.0, 'f', "BAND_AZ"},
      {"Az Time Delay (frames)",     0.0,  10.0, 'f', "DELAY_AZ"}
    }
  },
  {COMMAND(spider_scan), "scan in azimuth within a quad region in RA/Dec", 
   GR_POINT, 10,
    {
      {"RA of Corner 1 (h)",        0, 24, 'f', "RA_1_P"},
      {"Dec of Corner 1 (deg)",   -90, 90, 'f', "DEC_1_P"},
      {"RA of Corner 2 (h)",        0, 24, 'f', "RA_2_P"},
      {"Dec of Corner 2 (deg)",   -90, 90, 'f', "DEC_2_P"},
      {"RA of Corner 3 (h)",        0, 24, 'f', "RA_3_P"},
      {"Dec of Corner 3 (deg)",   -90, 90, 'f', "DEC_3_P"},
      {"RA of Corner 4 (h)",        0, 24, 'f', "RA_4_P"},
      {"Dec of Corner 4 (deg)",   -90, 90, 'f', "DEC_4_P"},
      {"Scan Starting RA (h)",      0, 24, 'f', "RA"},
      {"Scan Starting Dec (deg)", -90, 90, 'f', "DEC"}
    }
  },
  {COMMAND(sine_scan), "scan sinusoidally in azimuth with a specific amplitude",
   GR_POINT, 3,
    {
      {"scan amplitude (deg)",        0, 90, 'f', "AMPL_P"},
      {"scan az centre (deg)",        0, 360,'f', "AZ"},
      {"scan elevation (deg)",        15, 50, 'f', "EL"},
    }
  },   
  {COMMAND(ra_dec_set), "define RA/Dec of current position", GR_TRIM, 2,
    {
      {"Current RA (h)",      0, 24, 'f', "RA"},
      {"Current Dec (deg)", -90, 90, 'f', "DEC"}
    }
  },
  {COMMAND(pivot_gain), 
   "pivot gains ([v] = velocity mode only, [t] = torque mode only)", GR_GAIN, 7,
    {
      {"RW Set Point (dps)",   -500, 500, 'f', "SET_RW"},
      {"V_RW P Gain [t]", 0, USHRT_MAX, 'i', "G_PV_PIV"},
      {"RW vel. P gain [v]", 0, USHRT_MAX, 'i', "G_V_RW_PIV"},
      {"V_err P Gain [t]", 0, USHRT_MAX, 'i', "G_PE_PIV"},
      {"RW torque P gain [v]", 0, USHRT_MAX, 'i', "G_T_RW_PIV"},
      {"Static Friction offset [t]", 0, 2, 'f', "FRICT_OFF_PIV"},
      {"Az vel. request P Gain [v]", 0, USHRT_MAX, 'i', "G_V_REQ_AZ_PIV"}
    }
  },
  {COMMAND(set_piv_mode), "set pivot drive mode", GR_GAIN | CONFIRM, 1,
    {
      {"Drive Mode (0 = VEL, 1 = TORQUE)", 0, 1, 'i', "MODE_PIV"}
    }
  },
  {COMMAND(el_gain), "elevation motor gain", GR_GAIN, 1,
    {
      {"Common-Mode Gain (sqrt(accel))", 0.0,  3.0, 'f', "G_COM_EL"}
    }
  },
  {COMMAND(el_pulse), "manually set el motor pulse rates", GR_GAIN, 2,
    {
      {"port motor pulse rate (Hz)", -10000.0, 10000.0, 'f', "STEP_1_EL"},
      {"starboard motor pulse rate (Hz)", -10000.0, 10000.0, 'f', "STEP_2_EL"}
    }
  },
  {COMMAND(el_rel_move), "El relative move, separate for each side", 
   GR_POINT, 4,
    {
      {"port distance (deg)", -1.0, 1.0, 'f', "DEL_RELMOVE_PORT"},
      {"starboard distance (deg)", -1.0, 1.0, 'f', "DEL_RELMOVE_STBD"},
      {"port speed (deg/s)", 0.0, 0.2, 'f', "V_RELMOVE_PORT"},
      {"starboard speed (deg/s)", 0.0, 0.2, 'f', "V_RELMOVE_STBD"}
    }
  },

  {COMMAND(az_gyro_offset), "manually set az gyro offsets", GR_TRIM, 2,
    {
      {"IF Roll Gyro offset (deg/s)", -0.5, 0.5, 'f', "OFFSET_IFROLL_GY"},
      {"IF Yaw Gyro offset (deg/s)", -0.5, 0.5, 'f', "OFFSET_IFYAW_GY"}
    }
  },
  {COMMAND(el_gyro_offset), "manually set el gyro offset", GR_TRIM, 1,
    {
      {"IF Elev Gyro offset (deg/s)", -0.5, 0.5, 'f', "OFFSET_IFEL_GY"},
    }
  },
  {COMMAND(slew_veto), "set the length of the gyro offset slew veto", GR_TRIM,
    1,
    {
      {"Slew Veto (s)", 0., 1200., 'f', "SVETO_LEN"},
    }
  },
  {COMMAND(cov_gps), "set the threshhold for allowable DGPS covariance",
    GR_TRIM, 1,
    {
      {"Covariance (deg^2)", 0, 5.0, 'f', "COV_LIM_DGPS"},
    }
  },
  {COMMAND(ants_gps),
    "set the threshhold for allowable DGPS antenna separation error", GR_TRIM,
    1,
    {
      {"Antenna Separation Error (m)", 0, 10.0, 'f', "ANT_E_DGPS"},
    }
  },

  {COMMAND(hwp_general), "send a general phytron command string", GR_HWPR, 2,
    {
      {"Stepper (1-6)", 1, 6, 'i', ""},
      {"Command", 0, 32, 's', ""},
    }
  },
  {COMMAND(hwp_vel), "set the HWP rotator velocity", GR_HWPR, 1,
    {
      {"Velocity (dps)", 0, 10, 'f', "VEL_HWP"},
    }
  },
  {COMMAND(hwp_i), "set the HWP rotator current", GR_HWPR, 1,
    {
      {"Move current (A)", 0, 1.2, 'f', "I_MOVE_HWP"},
    }
  },
  {COMMAND(hwp_phase), "set the HWP analog lock-in phase", GR_HWPR, 1,
    {
      {"Phase (degrees)", 0, 360., 'f', "PHASE_00_HWP"},
    }
  },
  {COMMAND(hwp_halt), "halt HWP rotator motion", GR_HWPR, 1,
    {
      {CHOOSE_INSERT_PARAM}
    }
  },
  {COMMAND(hwp_move), "move the HWP rotator to relative position", GR_HWPR, 2,
    {
      {CHOOSE_INSERT_PARAM},
      {"delta (degrees)", -720, 720, 'd', "0"}
    }
  },
  {COMMAND(hwp_bias_on), "turn on bias for HWP encoders", GR_HWPR, 1,
    {
      {CHOOSE_INSERT_PARAM}
    }
  },
  {COMMAND(hwp_bias_off), "turn off bias for HWP encoders", GR_HWPR, 1,
    {
      {CHOOSE_INSERT_PARAM}
    }
  },

  /*******************************************************/
  /*************** Telemetry/Scheduling  *****************/
  {COMMAND(timeout), "time until schedule mode", GR_TELEM, 1,
    {
      {"Timeout (s)", 2, 65535, 'f', "TIMEOUT"}
    }
  },

  {COMMAND(tdrss_bw), "tdrss omni bandwith", GR_TELEM, 1,
    {
      {"Bandwidth (bps)", 100, 75000, 'f', "rate_tdrss"}
    }
  },

  {COMMAND(iridium_bw), "iridium dialup bandwith", GR_TELEM, 1,
    {
      {"Bandwidth (bps)", 100, 75000, 'f', "rate_iridium"}
    }
  },

  {COMMAND(pilot_bw), "pilot bandwith", GR_TELEM, 1,
    {
      {"Bandwidth (bps)", 100, 92000, 'f', "rate_pilot"}
    }
  },


  {COMMAND(oth_set), "OTH Link channel set", GR_TELEM, 1,
    {
      {"Channel Set", 0, 2, 'i', "channelset_oth"}
    }
  },
  
  {COMMAND(arraystat_oth), "OTH Link array stats allocation", GR_TELEM, 1,
    {
      {"N Array Stats", 0, 1000, 'i', "n_arraystats"}
    }
  },

  {COMMAND(get_mce_param), "Get MCE Parameter", GR_TELEM, 2,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Parameter Index", 0, N_MCE_STAT, 'i', "NONE"}
    }
  },

  /****************************************/
  /*************** Misc.  *****************/
  {COMMAND(reset_adc), "Reset an ADC motherboard", GR_POWER | GR_IFPOWER, 1,
    {
      {"Node number",  0, 64, 'i', ""}
    }
  },
  {COMMAND(t_gyro_gain), "gyro box heater gains", GR_ELECT, 3,
    {
      {"Proportional Gain", 0, USHRT_MAX, 'i', "g_p_heat_gy"},
      {"Integral Gain",     0, USHRT_MAX, 'i', "g_i_heat_gy"},
      {"Derrivative Gain",  0, USHRT_MAX, 'i', "g_d_heat_gy"}
    }
  },
  {COMMAND(t_gyro_set), "gyro box temperature set point", GR_ELECT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "T_SET_GY"}
    }
  },
  {COMMAND(t_rsc_set), "RSC temperature set point", GR_ELECT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "T_SET_RSC"}
    }
  },
  {COMMAND(t_bsc_set), "BSC temperature set point", GR_ELECT, 1,
    {
      {"Set Point (deg C)", 0, 60, 'f', "T_SET_BSC"}
    }
  },

  /***************************************/
  /*************** Bias  *****************/
  {COMMAND(hk_ampl_cernox), "Set cernox bias amplitude", GR_BIAS, 2,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Amplitude (V)", 0.0, 5.0, 'f', ""}
    }
  },
  {COMMAND(hk_ampl_ntd), "Set NTD bias amplitude", GR_BIAS, 2,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Amplitude (V)", 0.0, 5.0, 'f', ""}
    }
  },
  {COMMAND(hk_phase_cernox), "Set cernox bias phase", GR_BIAS, 2,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Phase (degrees)", 0.0, 360.0, 'f', ""}
    }
  },
  {COMMAND(hk_phase_ntd), "Set NTD bias phase", GR_BIAS, 2,
    {
      {"Insert", 1, HK_MAX, 'i', "INSERT_LAST_HK", {just_mce_names}},
      {"Phase (degrees)", 0.0, 360.0, 'f', ""}
    }
  },
  {COMMAND(hk_phase_step_cernox), "Sweep Cernox bias phase", GR_BIAS, 5,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Phase start (degrees)", 0.0, 360.0, 'f', ""},
      {"Phase end (degrees)",   0.0, 360.0, 'f', ""},
      {"Number of steps", 0, USHRT_MAX, 'i', ""},
      {"Time per step (s)", 0, USHRT_MAX, 'i', ""},
    }
  },
  {COMMAND(hk_phase_step_ntd), "Sweep NTD bias phase", GR_BIAS, 5,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Phase start (degrees)", 0.0, 360.0, 'f', ""},
      {"Phase end (degrees)",   0.0, 360.0, 'f', ""},
      {"Number of steps", 0, USHRT_MAX, 'i', ""},
      {"Time per step (s)", 0, USHRT_MAX, 'i', ""},
    }
  },
  {COMMAND(hk_bias_freq), "Set NTD & cernox bias frequency", GR_BIAS, 1,
    {
      {"Frequency (Hz)", 10, 400, 'i', "F_BIAS_CMD_HK"}
    }
  },

  /***************************************/
  /*************** Heat  *****************/
  {COMMAND(hk_pump_heat_on),
    "Turn on the pump (charcoal) heater. Disable autocycle or servo",
    GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_pump_heat_off),
    "Turn off the pump (charcoal) heater. Disable autocycle or servo", 
    GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_heat_switch_on),
    "Turn on the heat switch heater. Disable autocycle", GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_heat_switch_off),
    "Turn off the heat switch heater. Disable autocycle", GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_fphi_heat_on), "Turn on the high-current focal plane heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_fphi_heat_off), "Turn off the high-current focal plane heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_fphi_heat_pulse), "Pulse the high-current focal plane heater",
      GR_CRYO_HEAT, 2,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Length (# of ~200ms slow frames)", 0, USHRT_MAX, 'i', "1"},
    }
  },
  {COMMAND(hk_ssa_heat_on), "Turn on the SSA heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_ssa_heat_off), "Turn off the SSA heater",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_htr1_heat_on), "Turn on heater 1",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_htr1_heat_off), "Turn off heater 1",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_htr2_heat_on), "Turn on heater 2",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_htr2_heat_off), "Turn off heater 2",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_htr3_heat_on), "Turn on heater 3",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_htr3_heat_off), "Turn off heater 3",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_ring_heat_set), "Set 300mK ring heater voltage", GR_CRYO_HEAT, 2,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Level (V)", -5.0, 5.0, 'f', "V_HEAT_LAST_HK"},
    }
  },
  {COMMAND(hk_fplo_heat_set), "Set low-current focal plane heater voltage",
      GR_CRYO_HEAT, 2,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Level (V)", -5.0, 5.0, 'f', "V_HEAT_LAST_HK"},
    }
  },
  {COMMAND(hk_auto_cycle_on), "Enable autocycling of He3 fridge",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_auto_cycle_off), "Disable autocycling of He3 fridge",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_fridge_cycle), "Force an He3 fridge cycle. Enable autocycle",
      GR_CRYO_HEAT, 1,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
    }
  },
  {COMMAND(hk_pump_servo_on), 
      "Enable pump servo mode: maintain pump temperature between setpoints.",
      GR_CRYO_HEAT, 3,
    {
      {"Insert", 0, HK_MAX, 'i', "INSERT_LAST_HK", {mce_names}},
      {"Set Low (K):",           0.0, 50.0,     'f', ""},
      {"Set High (K):",          0.0, 50.0,     'f', ""},
    }
  },
  {COMMAND(hk_mt_bottom_pulse), "Pulse Theo's MT bottom heaters",
    GR_THEO_HEAT, 2,
    {
      {"Power (W,max=9.0)", 0.0, 9.0, 'f', ""},
      {"Duration (minutes) (0-1440,-1=infinity)", -1.0, 1440.0, 'f', ""},
    }
  },
  {COMMAND(hk_sft_lines_pulse), "Pulse Theo's SFT fill and vent line heaters",
    GR_THEO_HEAT, 2,
    {
      {"Power (W,max=12.0)", 0.0, 12.0, 'f', ""},
      {"Duration (minutes) (0=1440,-1=infinity)", -1.0, 1440.0, 'f', ""},
    }
  },
  {COMMAND(hk_capillary_pulse), "Pulse Theo's capillary heater",
    GR_THEO_HEAT, 2,
    {
      {"Power (W,max=4.5)", 0.0, 4.5, 'f', ""},
      {"Duration (minutes) (0-1440,-1=infinity)", -1.0, 1440.0, 'f', ""},
    }
  },
  {COMMAND(hk_vcs2_hx_pulse), "Pulse Theo's VCS2 HX heaters",
    GR_THEO_HEAT, 2,
    {
      {"Power (W,max=3.6)", 0.0, 3.6, 'f', ""},
      {"Duration (minutes) (0-1440,-1=infinity)", -1.0, 1440.0, 'f', ""},
    }
  },
  {COMMAND(hk_vcs1_hx_pulse), "Pulse Theo's VCS1 HX heaters",
    GR_THEO_HEAT, 2,
    {
      {"Power (W,max=6.3)", 0.0, 6.3, 'f', ""},
      {"Duration (minutes) (0-1440,-1=infinity)", -1.0, 1440.0, 'f', ""},
    }
  },
  {COMMAND(hk_mt_lines_pulse), "Pulse Theo's MT fill and vent line heaters",
    GR_THEO_HEAT, 2,
    {
      {"Power (W,max=9.0)", 0.0, 9.0, 'f', ""},
      {"Duration (minutes) (0-1440,-1=infinity)", -1.0, 1440.0, 'f', ""},
    }
  },
  {COMMAND(hk_sft_bottom_pulse), "Pulse Theo's SFT Bottom heater",
    GR_THEO_HEAT, 2,
    {
      {"Power (W,max=4.5)", 0.0, 4.5, 'f', ""},
      {"Duration (minutes) (0-1440,-1=infinity)", -1.0, 1440.0, 'f', ""},
    }
  },
  {COMMAND(mce_veto), "Set all squid and TES biases to zero, "
      "disable muxing, and stop data acquisition", GR_MPC | GR_CRYO_HEAT, 1,
      {
        {CHOOSE_INSERT_PARAM}
      }
  },
  {COMMAND(mce_unveto), "Resume normal MCE operations", GR_MPC | GR_CRYO_HEAT,
    1,
      {
        {CHOOSE_INSERT_PARAM}
      }
  },

  /***************************************/
  /*************** The Good  *****************/
  {COMMAND(thegood_any), "Execute arbitrary The Good command", GR_SCGOOD, 1,
    {
      {"Command String", 0, 32, 's', ""}
    }
  },
  {COMMAND(thegood_settrig_timed), "Use timed exposure mode on The Good",
    GR_SCGOOD, 1,
    {
      {"Exposure Interval (ms)", 0, USHRT_MAX, 'i', "exp_int_thegood"}
    }
  },
  {COMMAND(thegood_exp_params), "set The Good exposure commands", GR_SCGOOD, 1,
    {
      {"Exposure duration (ms)", 40, USHRT_MAX, 'i', "exp_time_thegood"}
    }
  },
  {COMMAND(thegood_focus_params), "set The Good autofocus params", GR_SCGOOD, 2,
    {
      {"Resolution (number total positions)", 0, USHRT_MAX, 'i',
        "foc_res_thegood"},
      {"Range (inverse fraction of total range)", 0, USHRT_MAX, 'i', "NONE"} 
    }
  },
  {COMMAND(thegood_bad_pix), "Indicate pixel to ignore on The Good", GR_SCGOOD,
    3,
    {
      {"Camera ID (0 or 1)", 0, 1, 'i', ""},
      //1530 = CAM_WIDTH, 1020 = CAM_HEIGHT (camstruct.h)
      {"x (0=left)", 0, 1530, 'i', ""},
      {"y (0=top)", 0, 1020, 'i', ""}
    }
  },
  {COMMAND(thegood_platescale), "set The Good plate scale", GR_SCGOOD, 1,
    {
      {"Plate scale (arcsec/pixel)", 0, 20, 'f', ""}
    }
  },
  {COMMAND(thegood_blob_params), "set blob finder params on The Good",
    GR_SCGOOD, 4,
    {
      {"Max number of blobs", 1, USHRT_MAX, 'i', "maxblob_thegood"},
      {"Search grid size (pix)", 1, 1530 , 'i', "grid_thegood"},
      {"Threshold (# sigma)", 0, 100, 'f', "thresh_thegood"},
      {"Min blob separation ^2 (pix^2)", 1, 1530 , 'i', "mdist_thegood"}
    }
  },
  {COMMAND(thegood_lens_any), "execute The Good lens command directly",
    GR_SCGOOD, 1,
    {
      {"Lens command string", 0, 32, 's', ""}
    }
  },
  {COMMAND(thegood_lens_move), "move The Good lens", GR_SCGOOD, 1,
    {
      //total range on Sigma EX 120-300mm is about 3270
      {"New position (ticks)", -10000, 10000, 'i', ""}
    }
  },
  {COMMAND(thegood_lens_params), "set The Good lens params", GR_SCGOOD, 1,
    {
      {"Allowed move error (ticks)", 0, USHRT_MAX, 'i', "move_tol_thegood"}
    }
  },
  {COMMAND(thegood_viewer), "switch video (0=TheGood, 1=TheBad, 2=TheUgly)",
    GR_SCGOOD, 1,
    {
      {"Camera", 0, 2, 'i', ""}
    }
  },
  /***************************************/
  /*************** The Bad  *****************/
  {COMMAND(thebad_any), "Execute arbitrary The Bad command", GR_SCBAD, 1,
    {
      {"Command String", 0, 32, 's', ""}
    }
  },
  {COMMAND(thebad_settrig_timed), "Use timed exposure mode on The Bad",
    GR_SCBAD, 1,
    {
      {"Exposure Interval (ms)", 0, USHRT_MAX, 'i', "exp_int_thebad"}
    }
  },
  {COMMAND(thebad_exp_params), "set The Bad exposure commands", GR_SCBAD, 1,
    {
      {"Exposure duration (ms)", 40, USHRT_MAX, 'i', "exp_time_thebad"}
    }
  },
  {COMMAND(thebad_focus_params), "set The Bad autofocus params", GR_SCBAD, 2,
    {
      {"Resolution (number total positions)", 0, USHRT_MAX, 'i',
        "foc_res_thebad"},
      {"Range (inverse fraction of total range)", 0, USHRT_MAX, 'i', "NONE"} 
    }
  },
  {COMMAND(thebad_bad_pix), "Indicate pixel to ignore on The Bad", GR_SCBAD, 3,
    {
      {"Camera ID (0 or 1)", 0, 1, 'i', ""},
      //1530 = CAM_WIDTH, 1020 = CAM_HEIGHT (camstruct.h)
      {"x (0=left)", 0, 1530, 'i', ""},
      {"y (0=top)", 0, 1020, 'i', ""}
    }
  },
  {COMMAND(thebad_platescale), "set The Bad plate scale", GR_SCBAD, 1,
    {
      {"Plate scale (arcsec/pixel)", 0, 20, 'f', ""}
    }
  },
  {COMMAND(thebad_blob_params), "set blob finder params on The Bad", GR_SCBAD,
    4,
    {
      {"Max number of blobs", 1, USHRT_MAX, 'i', "maxblob_thebad"},
      {"Search grid size (pix)", 1, 1530 , 'i', "grid_thebad"},
      {"Threshold (# sigma)", 0, 100, 'f', "thresh_thebad"},
      {"Min blob separation ^2 (pix^2)", 1, 1530 , 'i', "mdist_thebad"}
    }
  },
  {COMMAND(thebad_lens_any), "execute The Bad lens command directly", GR_SCBAD,
    1,
    {
      {"Lens command string", 0, 32, 's', ""}
    }
  },
  {COMMAND(thebad_lens_move), "move The Bad lens", GR_SCBAD, 1,
    {
      //total range on Sigma EX 120-300mm is about 3270
      {"New position (ticks)", -10000, 10000, 'i', ""}
    }
  },
  {COMMAND(thebad_lens_params), "set The Bad lens params", GR_SCBAD, 1,
    {
      {"Allowed move error (ticks)", 0, USHRT_MAX, 'i', "move_tol_thebad"}
    }
  },
  /***************************************/
  /*************** The Ugly  *****************/
  {COMMAND(theugly_any), "Execute arbitrary The Ugly command", GR_SCUGLY, 1,
    {
      {"Command String", 0, 32, 's', ""}
    }
  },
  {COMMAND(theugly_settrig_timed), "Use timed exposure mode on The Ugly",
    GR_SCUGLY, 1,
    {
      {"Exposure Interval (ms)", 0, USHRT_MAX, 'i', "exp_int_theugly"}
    }
  },
  {COMMAND(theugly_exp_params), "set The Ugly exposure commands", GR_SCUGLY, 1,
    {
      {"Exposure duration (ms)", 40, USHRT_MAX, 'i', "exp_time_theugly"}
    }
  },
  {COMMAND(theugly_focus_params), "set The Ugly autofocus params", GR_SCUGLY, 2,
    {
      {"Resolution (number total positions)", 0, USHRT_MAX, 'i',
        "foc_res_theugly"},
      {"Range (inverse fraction of total range)", 0, USHRT_MAX, 'i', "NONE"} 
    }
  },
  {COMMAND(theugly_bad_pix), "Indicate pixel to ignore on The Ugly", GR_SCUGLY,
    3,
    {
      {"Camera ID (0 or 1)", 0, 1, 'i', ""},
      //1530 = CAM_WIDTH, 1020 = CAM_HEIGHT (camstruct.h)
      {"x (0=left)", 0, 1530, 'i', ""},
      {"y (0=top)", 0, 1020, 'i', ""}
    }
  },
  {COMMAND(theugly_platescale), "set The Ugly plate scale", GR_SCUGLY, 1,
    {
      {"Plate scale (arcsec/pixel)", 0, 20, 'f', ""}
    }
  },
  {COMMAND(theugly_blob_params), "set blob finder params on The Ugly",
    GR_SCUGLY, 4,
    {
      {"Max number of blobs", 1, USHRT_MAX, 'i', "maxblob_theugly"},
      {"Search grid size (pix)", 1, 1530 , 'i', "grid_theugly"},
      {"Threshold (# sigma)", 0, 100, 'f', "thresh_theugly"},
      {"Min blob separation ^2 (pix^2)", 1, 1530 , 'i', "mdist_theugly"}
    }
  },
  {COMMAND(theugly_lens_any), "execute The Ugly lens command directly",
    GR_SCUGLY, 1,
    {
      {"Lens command string", 0, 32, 's', ""}
    }
  },
  {COMMAND(theugly_lens_move), "move The Ugly lens", GR_SCUGLY, 1,
    {
      //total range on Sigma EX 120-300mm is about 3270
      {"New position (ticks)", -10000, 10000, 'i', ""}
    }
  },
  {COMMAND(theugly_lens_params), "set The Ugly lens params", GR_SCUGLY, 1,
    {
      {"Allowed move error (ticks)", 0, USHRT_MAX, 'i', "move_tol_theugly"}
    }
  },
  {COMMAND(rsc_trig_wait), "rotating star camera trigger interval", GR_TRIM, 1,
    {
      {"interval (s)", 0, 100, 'i', ""}
    }
  },
  {COMMAND(bsc_trig_delay), "delay to trigger boresight camera", GR_TRIM, 1,
    {
      {"Delay (frames)", 0, 100, 'f', "TRIG_DELAY"}
    }
  },
  //STAR CAMERA TABLE
  {COMMAND(table_gain), "RSC rotary table gains", GR_SCTAB, 3,
    {
      {"Proportional Gain", 0, USHRT_MAX, 'i', "g_p_table"},
      {"Integral Gain",     0, USHRT_MAX, 'i', "g_i_table"},
      {"Derivative Gain",   0, USHRT_MAX, 'i', "g_d_table"}
    }
  },
  {COMMAND(table_goto), "move RSC table to specific encoder position", GR_SCTAB,
    1,
    {
      {"Goto position (deg)", 0, 360, 'd', "table_goto"}
    }
  },
  {COMMAND(table_drift), "move RSC table at constant speed", GR_SCTAB, 1,
    {
      {"Speed (deg/s)", -30, 30, 'd', "table_speed"}
    }
  },

  {COMMAND(motors_verbose), "Set verbosity of motor serial threads "
    "(0=norm, 1=verbose, 2= superverbose )", GR_MISC, 2,
   {
     {"Reaction Wheel", 0, 5, 'i', "VERBOSE_RW"},
     {"Pivot", 0, 5, 'i', "VERBOSE_PIV"}
   }
  },

  {COMMAND(bbc_rate_ext), "Set BBC external (sync box) sample rate",
    GR_MISC | CONFIRM, 1,
    {
      {"Rate (Sync frames per BBC)", 1, USHRT_MAX, 'i', "FRAME_EXT_BBC"}
    }
  },
  {COMMAND(bbc_rate_int), "Set BBC internal sample rate",
    GR_MISC | CONFIRM, 1,
    {
      {"Rate (# ADC samples (~10kHz))", 1, USHRT_MAX, 'i', "FRAME_INT_BBC"}
    }
  },

  {COMMAND(data_mode), "Set the MCE data mode", GR_MPC, 1,
    {
      {"Data Mode", 0, 12, 'i', "DATA_MODE"}
    }
  },

  {COMMAND(data_mode_bits), "Change how 32->16 bit translation for TES data "
    "happens", GR_MPC, 5,
    {
      {"Data mode", 0, 12, 'i', "DATA_MODE"},
      {"Upper subfield first bit", 0, 32, 'i', ""},
      {"Upper subfield num bits", 0, 32, 'i', ""},
      {"Lower subfield first bit", 0, 32, 'i', ""},
      {"Lower subfield num bits", 0, 32, 'i', ""}
    }
  },

  {COMMAND(bset), "Change the bolometer field set in use", GR_TELEM, 1,
    {
      {"number", 0, 255, 'i', "BSET_NUM"}
    }
  },

  /* Sync Box Commands */
  {COMMAND(mce_row_len), "Set the MCE per-row dwell time", GR_SYNC, 1,
    {
      {"Row Len (25MHz cycles)", 5, 4094, 'i', "ROW_LEN_SYNC"}
    }
  },
  {COMMAND(mce_num_rows), "Set the number of MCE rows servoed", GR_SYNC, 1,
    {
      {"Num Rows", 1, 41, 'i', "NUM_ROWS_SYNC"}
    }
  },
  {COMMAND(mce_data_rate), "Set the MCE data rate (ARZ per data frame)",
    GR_SYNC, 1,
    {
      {"Data Rate", 1, 4095, 'i', "DATA_RATE_SYNC"}
    }
  },

  /***********************************************/
  /*************** MCE COMMANDS  *****************/

  {MCECMD1(acquire, "Acquire regular MCE data", GR_MPC)},
  {MCECMD1(cycle_acq, "Force a restart of MCE data acquisition", GR_MPC)},
  {MCECMD1(force_config, "Start data acquisition without reconfiguring",
      GR_MPC)},
  {MCECMD1(reconfig, "Reconfig the MCE and re-start data acquisition", GR_MPC)},
  {MCECMD1(drive_check, "Force MPC to recheck its data drives", GR_MPC)},
  {MCECMD1(pause_acq, "Pause data acquisition", GR_MPC)},
  {MCECMD1P(send_exptcfg, "Send down experiment.cfg", GR_MPC)},
  {MCECMD1P(reload_mce_config, "Reset experiment.cfg to template",
      GR_MPC | CONFIRM)},
  {MCECMD1P(reload_dead_masks, "Reset dead and frail masks to template",
      GR_MPC | CONFIRM)},
  {MCECMD1(mce_clock_ext, "Unveto sync box use by the MCE", GR_MPC)},
  {MCECMD1(mce_clock_int, "Veto sync box use by the MCE", GR_MPC)},

  {MCECMD2A(integral_clamp, "Set the integral clamping factor", GR_DET,
      "Factor", 0, 1, 'f')},
  {COMMAND(tune_array), "Tune a focal plane with current tuning parameters",
    GR_MPC | MCECMD, 4,
    {
      {CHOOSE_INSERT_PARAM},
      {"First stage", 0, 4, 'i', "NONE", {tuning_stages}},
      {"Last stage", 0, 4, 'i', "NONE", {tuning_stages}},
      {"Apply?", 0, 3, 'i', "NONE", {autonoyes_names}}
    }
  },
  {COMMAND(tune_biases), "Tune, with bias ramping forced on", GR_MPC | MCECMD,
    4,
    {
      {CHOOSE_INSERT_PARAM},
      {"First stage", 0, 4, 'i', "NONE", {tuning_stages}},
      {"Last stage", 0, 4, 'i', "NONE", {tuning_stages}},
      {"Apply?", 0, 3, 'i', "NONE", {autonoyes_names}}
    }
  },

  {COMMAND(column_on), "Turn on a MCE column", GR_DET | MCECMD, 5,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Column", 0, 15, 'i', "NONE"},
      {"SA Bias", 0, 65535, 'i', "NONE"},
      {"SQ2 Bias", 0, 65535, 'i', "NONE"},
      {MCE_ACTION_PARAM(4,daction_names)}
    }
  },

  {MCECMD2AP(column_off, "Turn off a MCE column", GR_DET, "Column", 0, 15,
      'i')},
  {MCECMD2(sa_offset_bias_ratio, "Set the SA offset bias ratio", GR_TUNE,
      "Ratio", 0, 2, 'f')},
  {MCECMD1(sa_ramp_bias_on, "Turn on SA ramping while tuning", GR_TUNE)},
  {MCECMD1(sa_ramp_bias_off, "Turn off SA ramping while tuning", GR_TUNE)},
  {MCECMDSCS(sa_ramp_flux, "Set the SA flux ramp parameters", GR_TUNE)},
  {MCECMDSCS(sa_ramp_bias, "Set the SA bias ramp parameters", GR_TUNE)},
  {MCECMD2(sq2_tuning_row, "Set the preferred row for auto tuning", GR_TUNE,
      "Row", 0, 32, 'i')},
  {MCECMDC(sq2_servo_gain, "Set the SQ2 servo gain", GR_TUNE,
      "Gain", -2, 2, 'f')},
  {MCECMDC(sq1_servo_gain, "Set the SQ1 servo gain", GR_TUNE,
      "Gain", -2, 2, 'f')},
  {MCECMD1(sq1_servo_bias_on, "Turn on SQ1 servoing while tuning", GR_TUNE)},
  {MCECMD1(sq1_servo_bias_off, "Turn off SQ1 servoing while tuning", GR_TUNE)},
  {MCECMDSCS(sq1_servo_flux, "Set the SQ1 servo flux ramp parameters",
      GR_TUNE)},
  {MCECMDSCS(sq1_servo_bias, "Set the SQ1 servo bias ramp parameters",
      GR_TUNE)},
  {MCECMD1(sq2_servo_bias_on, "Turn on SQ2 servoing while tuning", GR_TUNE)},
  {MCECMD1(sq2_servo_bias_off, "Turn off SQ2 servoing while tuning", GR_TUNE)},
  {MCECMDSCS(sq2_servo_flux, "Set the SQ2 servo flux ramp parameters",
      GR_TUNE)},
  {MCECMDSCS(sq2_servo_bias, "Set the SQ2 servo bias ramp parameters",
      GR_TUNE)},
  {MCECMDSCS(sq1_ramp_flux, "Set the SQ1 flux ramp parameters", GR_TUNE)},
  {MCECMD1(sq1_ramp_tes_bias_on,
      "Turn on the final TES bias ramp at the end of tuning", GR_TUNE)},
  {MCECMD1(sq1_ramp_tes_bias_off,
      "Turn off the final TES bias ramp at the end of tuning", GR_TUNE)},
  {MCECMDSCS(sq1_ramp_tes_bias, "Set the SQ1 ramp TES bias parameters",
      GR_TUNE)},
  {MCECMDC(tes_bias_idle, "Set the idle detector bias level", GR_TUNE,
      "Level", 0, 65535, 'i')},
  {MCECMDC(tes_bias_normal, "Set the normal detector bias level", GR_TUNE,
      "Level", 0, 65535, 'i')},
  {MCECMD2(tes_bias_normal_time, "Set the normal detector bias time", GR_TUNE,
      "Time", 0, 10, 'f')},
  {MCECMD1(tuning_check_bias_on, "Turn on thermalisation time for tuning",
      GR_TUNE)},
  {MCECMD1(tuning_check_bias_off, "Turn off thermalisation time for tuning",
      GR_TUNE)},
  {MCECMD1(sq1_ramp_check_on, "Turn on the SQ1 ramp check", GR_TUNE)},
  {MCECMD1(sq1_ramp_check_off, "Turn off the SQ1 ramp check", GR_TUNE)},
  {MCECMD2(tuning_therm_time, "Thermalisation time for squids", GR_TUNE,
      "Time", 0, 65535, 'i')},
  {MCECMD1(tuning_do_plots_on, "Turn on plot generation for tuning", GR_TUNE)},
  {MCECMD1(tuning_do_plots_off, "Turn off plot generation for tuning",
      GR_TUNE)},
  {MCECMD2(sq2servo_safb_init, "Override start point for sq2servo call",
      GR_TUNE, "Value", 0, 65535, 'i')},
  {MCECMD2(sq1servo_sq2fb_init, "Override start point for sq1servo call",
      GR_TUNE, "Value", 0, 65535, 'i')},
  {MCECMDSCS(ramp_tes, "set the TES bias ramp parameters", GR_IV)},
  {MCECMD2(ramp_tes_final_bias, "Final bias for the TES bias ramp", GR_IV,
      "Level", 0, 65535, 'i')},
  {MCECMD2(ramp_tes_initial_pause, "Initial pause for the TES bias ramp", GR_IV,
      "Time (us)", 0, 65535, 'i')},
  {MCECMD2(ramp_tes_period, "Period for the TES bias ramp", GR_IV, "Time (us)",
      0, 1000000000, 'l')},
  {MCECMD2A(sample_dly, "Sample delay", GR_DET, "Count", 0, 200, 'i')},
  {MCECMD2A(sample_num, "Sample number", GR_DET, "Count", 0, 100, 'i')},
  {MCECMD2A(fb_dly, "Feedback delay", GR_DET, "Count",  0, 100, 'i')},
  {MCECMD2A(row_dly, "Row delay", GR_DET, "Count",  0, 100, 'i')},
  {MCECMD1AD(flux_jumping_on, "Turn on flux jumping", GR_DET)},
  {MCECMD1AD(flux_jumping_off, "Turn off flux jumping", GR_DET)},
  {COMMAND(servo_pid_col), "Set the servo gains for a column", GR_DET | MCECMD,
    6,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Column", 0, 15, 'i', "NONE"},
      {"P Gain", 0, 65535, 'i', "NONE"}, 
      {"I Gain", 0, 65535, 'i', "NONE"},
      {"D Gain", 0, 65535, 'i', "NONE"},
      {MCE_ACTION_PARAM(4,daction_names)},
    }
  },
  {COMMAND(servo_pid_pixel), "Set the servo gains for a pixel", GR_DET | MCECMD,
    6,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Column", 0, 15, 'i', "NONE"},
      {"Row", 0, 32, 'i', "NONE"},
      {"P Gain", 0, 65535, 'i', "NONE"}, 
      {"I Gain", 0, 65535, 'i', "NONE"},
      {"D Gain", 0, 65535, 'i', "NONE"},
    }
  },
  {COMMAND(servo_pid_frail), "Set the frail servo gains", GR_DET | MCECMD, 4,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"P Gain", 0, 65535, 'i', "1"}, 
      {"I Gain", 0, 65535, 'i', "2"},
      {"D Gain", 0, 65535, 'i', "3"},
    }
  },
  {MCECMDCR1A(dead_detector, "Set a detector as dead", GR_DET)},
  {MCECMDCR1A(frail_detector, "Set a detector as frail", GR_DET)},
  {MCECMDCR1A(healthy_detector, "Set a detector as healthy", GR_DET)},
  {MCECMDCA(bias_tes_col, "Set the TES bias level for a column", GR_DET,
      "Bias", 0, 65535, 'i')},
  {MCECMDC(sa_flux_quantum, "SA flux quantum", GR_DET,
      "Quantum", 0, 65535, 'i')},
  {MCECMDC(sq2_flux_quantum, "SQ2 flux quantum", GR_DET,
      "Quantum", 0, 65535, 'i')},
  {MCECMDCR(sq1_flux_quantum, "SQ1 flux quantum", GR_DET,
      "Quantum", 0, 65535, 'i')},
  {MCECMDRAD(sq1_bias, "SQ1 bias", GR_DET, "Bias", 0, 65535, 'i')},
  {MCECMDRAD(sq1_off_bias, "SQ1 off bias", GR_DET, "Bias", 0, 65535, 'i')},
  {MCECMDCAD(sq2_bias, "SQ2 bias", GR_DET, "Bias", 0, 65535, 'i')},
  {MCECMDCRA(sq2_fb, "SQ2 feeback", GR_DET, "Feeback", 0, 65535, 'i')},
  {MCECMDCAD(sa_bias, "SA bias", GR_DET, "Bias", 0, 65535, 'i')},
  {MCECMDCA(sa_fb, "SA feeback", GR_DET, "Bias", 0, 65535, 'i')},
  {MCECMDCA(sa_offset, "SA offset", GR_DET, "Bias", 0, 65535, 'i')},
  {MCECMDCRA(adc_offset, "ADC offset ", GR_DET, "Offset", 0, 65535, 'i')},
  {MCECMD2P(tile_heater_on, "Turn on the tile heater", GR_IV,
      "Level (V)", 0, 5, 'f')},
  {MCECMD1(tile_heater_off, "Turn off the tile heater", GR_IV)},
  {COMMAND(tile_heater_kick), "Pulse the tile heater", GR_IV | MCECMD, 3,
    {
      {CHOOSE_INSERT_PARAM},
      {"Level (V)", 0, 5, 'f', "NONE"},
      {"Duration (s)", 0, 100, 'f', "NONE"},
    }
  },
  {MCECMDC(servo_reset, "Reset a detector's servo", GR_DET, "Row", 0, 32, 'i')},
  {MCECMD1(flux_loop_init, "Reset the MCE flux-loop servo", GR_DET)},
  {MCECMD1(lcloop, "Run load curves forever", GR_IV)},
  {COMMAND(bias_tes_all), "Set all TES biases", GR_DET | MCECMD, 4,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Kick", 0, 2, 'i', "NONE", {kick_names}},
      {"Bias", 0, 65535, 'i', "NONE"},
      {MCE_ACTION_PARAM(3,action_names)}
    }
  },
  {COMMAND(bias_tes_rc1), "Set TES biases for columns 0 to 7", GR_DET | MCECMD,
    11,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Kick", 0, 2, 'i', "NONE", {kick_names}},
      {"Column 0", 0, 65535, 'i', "NONE"},
      {"Column 1", 0, 65535, 'i', "NONE"},
      {"Column 2", 0, 65535, 'i', "NONE"},
      {"Column 3", 0, 65535, 'i', "NONE"},
      {"Column 4", 0, 65535, 'i', "NONE"},
      {"Column 5", 0, 65535, 'i', "NONE"},
      {"Column 6", 0, 65535, 'i', "NONE"},
      {"Column 7", 0, 65535, 'i', "NONE"},
      {MCE_ACTION_PARAM(3,action_names)}
    }
  },
  {COMMAND(bias_tes_rc2), "Set TES biases for columns 8 to 15", GR_DET | MCECMD,
    11,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Kick", 0, 2, 'i', "NONE", {kick_names}},
      {"Column  8", 0, 65535, 'i', "NONE"},
      {"Column  9", 0, 65535, 'i', "NONE"},
      {"Column 10", 0, 65535, 'i', "NONE"},
      {"Column 11", 0, 65535, 'i', "NONE"},
      {"Column 12", 0, 65535, 'i', "NONE"},
      {"Column 13", 0, 65535, 'i', "NONE"},
      {"Column 14", 0, 65535, 'i', "NONE"},
      {"Column 15", 0, 65535, 'i', "NONE"},
      {MCE_ACTION_PARAM(3,action_names)}
    }
  },
  {COMMAND(mce_wb), "General purpose MCE write block (wb)",
    GR_MPC | MCECMD | CONFIRM, 5,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Card", 0, 6, 'i', "NONE", {wb_cards}},
      {"Block (Parameter) Number", 0, 0xFF, 'i', "NONE"},
      {"Element Number", 0, 41, 'i', "NONE"},
      {"Value", 0, 4294967295UL, 'l', "NONE"},
    }
  },

  {COMMAND(send_iv_curve), "Send down IV curves", GR_IV | MCECMD, 3,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"First curve", 0, NUM_MCE_FIELDS - 1, 'i', "NONE"},
      {"Count", 0, 100, 'i', "NONE"}
    }
  },

  {COMMAND(acq_iv_curve), "Acquire IV curves and pick biases", GR_IV | MCECMD,
    8,
    {
      {CHOOSE_INSERT_PARAM},
      {"Kick", 0, 2, 'i', "NONE", {kick_names}},
      {"Post-kick wait (s)", 0, 1000, 'f', "NONE"},
      {"Start bias", 0, 65535, 'i', "NONE"},
      {"Last bias", 0, 65535, 'i', "NONE"},
      {"Step size", -200, 200, 'i', "NONE"}, /* sign is ignored */
      {"Step wait (s)", 0, 10, 'f', "NONE"},
      {"Set biases", 0, 1, 'i', "NONE", {noyes_names}}
    }
  },

  {COMMAND(send_tuning), "Send the experiment.cfg file resulting from a tuning",
    GR_MPC | MCECMD, 3,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Tuning number", 0, 65535, 'i', "NONE"},
      {"Item", 0, 1, 'i', "NONE", {tunedata_names}}
    }
  },

  {COMMAND(use_tuning), "Apply a previous tuning", GR_MPC | MCECMD, 2,
    {
      {CHOOSE_INSERT_NO_ALL},
      {"Tuning number (0 = last)", 0, 65535, 'i', "NONE"},
    }
  },

  {COMMAND(bias_step), "Step the TES bias above and then below the current "
    "level several times", GR_MPC | MCECMD, 4,
    {
      {CHOOSE_INSERT_PARAM},
      {"Step size (count)", 1, 3000, 'i', "NONE"},
      {"Period (s)", 0.1, 10, 'f', "NONE"},
      {"Num periods", 0, 100, 'i', "NONE"},
    }
  },

  {COMMAND(bias_ramp), "Step through a bias range", GR_MPC | MCECMD, 7,
    {
      {CHOOSE_INSERT_PARAM},
      {"Kick", 0, 2, 'i', "NONE", {kick_names}},
      {"Post-kick wait (s)", 0, 1000, 'f', "NONE"},
      {"Start bias", 0, 65535, 'i', "NONE"},
      {"Last bias", 0, 65535, 'i', "NONE"},
      {"Step size", -1000, 1000, 'i', "NONE"}, /* sign is ignored */
      {"Step wait (s)", 0, 900, 'f', "NONE"},
    }
  },

  /* DON'T PUT ANYTHING BELOW THIS */
  {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
    {
      {"Plover", 0, USHRT_MAX, 'i', "PLOVER"}
    }
  }
};

/* validate parameters of an mcom -- called by spidercmd before tranmitting a
 * command and by pcm after decoding one.  Inputs:
 *
 * cmd:         command number
 * [irs]values: mcp-style parsed parameters
 * buflen       size of the err_buffer
 * err_buffer   a place to write the error string
 *
 * Return value:
 *
 *  0:  if parameters are okay; err_buffer ignored.
 *  !0: if parameters are not okay.  In this case a descriptive error message
 *      should be written to err_buffer.
 */
int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer)
{
  switch (cmd) {
    case data_mode_bits:
      /* we have five integer parameters:
       * 0 = MCE number (ignored)
       * 1 = upper 1st bit
       * 2 = upper nbits
       * 3 = lower 1st bit
       * 4 = lower nbits
       */

      /* everything must fit in 32-bits */
      if ((ivalues[1] + ivalues[2] > 32) ||
          (ivalues[1] + ivalues[2] > 32))
      {
        snprintf(err_buffer, buflen, "Subfields exceed 32-bits");
        return 1;
      }
      /* the two subfields must total 16 bits */
      if (ivalues[2] + ivalues[4] != 16) {
        snprintf(err_buffer, buflen, "Subfield lengths don't make 16-bits (%i)",
            ivalues[2] + ivalues[4]);
        return 1;
      }

      /* if only one subfield is used, it must be the upper one */
      if (ivalues[2] == 0) {
        snprintf(err_buffer, buflen, "Upper subfield may not be empty");
        return 1;
      }

      if (ivalues[4] > 0) { /* two field checks */
        /* subfields must be ordered */
        if (ivalues[1] < ivalues[2]) {
          snprintf(err_buffer, buflen, "Upper subfield below lower subfield");
          return 1;
        }

        /* subfields may not overlap */
        if (ivalues[3] + ivalues[4] >= ivalues[1]) {
          snprintf(err_buffer, buflen, "Subfields overlap");
          return 1;
        }
      }
      break;
    default:
      break; /* default: assume everything's fine */
  }

  /* if we got here parameter checks passed, I guess */
  return 0;
}
