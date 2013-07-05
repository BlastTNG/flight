/* mcp: the BLAST master control program
 *
 * commands.c: functions for listening to and processing commands
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of mcp.
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
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include "command_list.h"
#include "command_struct.h"
#include "mcp.h"
#include "tx.h"
#include "pointing_struct.h"
#include "channels.h"
#include "sip.h"
#include "bset.h"
#include "mceserv.h"

void RecalcOffset(double, double);  /* actuators.c */

void SetRaDec(double, double); /* defined in pointing.c */
void SetTrimToSC(int);
void ClearTrim();
void AzElTrim(double, double);
void NormalizeAngle(double*);

int LoadUplinkFile(int slot); /*sched.c */

extern int doing_schedule; /* sched.c */

extern pthread_t watchdog_id;  /* mcp.c */
extern short int BitsyIAm;
pthread_mutex_t mutex;
void setup_bbc();

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

const char* SName(enum singleCommand command); // share/sip.c

/* in sc.cpp: */
int sendTheGoodCommand(const char *cmd);
int sendTheBadCommand(const char *cmd);
int sendTheUglyCommand(const char *cmd);

/* (re-)load the [bf]set number 'i' into the local buffer -- no change on error;
 * 'init'=1 occurs at start up, when there's no fallback initialised.
 */
static void change_bset(int i, int init)
{
  static uint8_t bset_serial = 0xF9;
  struct bset new_bset;

  /* range checking */
  if (i < 0 || i > 255) {
    bprintf(warning, "Set: ignoring out-of-range BSET index %i\n", i);
    return;
  }

  /* avoid the forbidden serial number */
  if (bset_serial == 0xFF)
    bset_serial++;

  /* special empty sets -- always succeeds */
  if (i == 0) {
    memset(&new_bset, 0, sizeof(new_bset));
    set_bset(&new_bset, bset_serial++ << 8);
    return;
  }

  /* try to load the bset */
  if (read_bset(i, &new_bset) == -1) {
    /* no bset loaded -- load the empty default */
    if (init) {
      change_bset(0, 0);
      return;
    }

    bprintf(warning, "Set: unable to read BSET%03i; still using BSET%03i", i,
        (CommandData.bset_num & 0xFF));
    return;
  }

  /* update the current bset */
  set_bset(&new_bset, i | (bset_serial++ << 8));
}

/* forward an unrecognised command to the MCE computers.  Returns zero if this
 * isn't in fact, a mce command */
static int MCEcmd(int command, const double *rvalues, const int *ivalues,
    char svalues[][CMD_STRING_LEN])
{
  int cmd, index;

  /* find the command, and bail on non-MCE commands */
  if (rvalues) {
    cmd = MIndex(command);
    if (!(mcommands[cmd].group & MCECMD))
      return 0;
  } else {
    cmd = SIndex(command);
    if (!(scommands[cmd].group & MCECMD))
      return 0;
  }

  /* queue the command */
  index = CommandData.mcecmd_index;

  /* invalidate */
  CommandData.mcecmd[index].t = -1;

  CommandData.mcecmd[index].is_multi = rvalues ? 1 : 0;
  CommandData.mcecmd[index].command = command;
  if (rvalues) {
    memcpy(CommandData.mcecmd[index].rvalues, rvalues,
        sizeof(double) * MAX_N_PARAMS);
    memcpy(CommandData.mcecmd[index].ivalues, ivalues,
        sizeof(int) * MAX_N_PARAMS);
    memcpy(CommandData.mcecmd[index].svalues, svalues,
        MAX_N_PARAMS * CMD_STRING_LEN);
  }

  /* Activate */
  CommandData.mcecmd[index].t = cmd;
  CommandData.mcecmd[index].done = 0;

  CommandData.mcecmd_index = INC_INDEX(index);

  bprintf(info, "Sent %s command #%i to MPC.\n",
      rvalues ? "multi" : "single", command);

  return 1;
}

void SingleCommand (enum singleCommand command, int scheduled)
{
  int i_point = GETREADINDEX(point_index);
  double sun_az;
  int is_new;

  if (!scheduled)
    bprintf(info, "Single command: %s (%d)\n", SName(command), command);

  /* Update CommandData structure with new info */

  switch (command) {
    case stop: /* Pointing abort */
      CommandData.ele_gain.manual_pulses = 0;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      break;
    case antisun: /* turn antisolar (az-only) */

      /* point solar panels to sun */
      //sun_az = PointingData[i_point].sun_az + 250;
      sun_az = PointingData[i_point].sun_az + 180.0; // set to this for now

      NormalizeAngle(&sun_az);

      CommandData.ele_gain.manual_pulses = 0;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_AZEL_GOTO;
      CommandData.pointing_mode.X = sun_az;  /* az */
      //CommandData.pointing_mode.Y = PointingData[i_point].el;  /* el */
      CommandData.pointing_mode.Y = ACSData.enc_mean_el;
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      break;

    case reset_trims:
      ClearTrim();
      break;
    case az_auto_gyro:
      CommandData.az_autogyro = 1;
      break;
    case el_auto_gyro:
      CommandData.el_autogyro = 1;
      break;

    case az_disable:/* disable az motors */
      CommandData.disable_az = 1;
      break;
    case az_enable:/* enable az motors */
      CommandData.disable_az = 0;
      break;
    case el_disable: /* disable el motors */
      CommandData.disable_el = 1;
      CommandData.force_el = 0;
      break;
    case el_enable: /* enable el motors */
      CommandData.disable_el = 0;
      CommandData.force_el = 0;
      break;
    case force_el_enable: /* force enabling of el motors */
      CommandData.disable_el = 0;
      CommandData.force_el = 1;
      break;
    case reset_rw:
      CommandData.reset_rw=1;
      break;
    case reset_piv:
      CommandData.reset_piv=1;
      break;
    case reset_elev:
      CommandData.reset_elev=1;
      break;
    case restore_piv:
      CommandData.restore_piv=1;
      break;
    case pss_veto:      /* Veto sensors */
      CommandData.use_pss = 0;
      break;
    case mag_veto:
      CommandData.use_mag = 0;
      break;
    case gps_veto:
      CommandData.use_gps = 0;
      break;
    case elenc1_veto:
      CommandData.use_elenc1 = 0;
      break;
    case elenc2_veto:
      CommandData.use_elenc2 = 0;
      break;
    case pss_allow:      /* Un-veto sensors */
      CommandData.use_pss = 1;
      break;
    case mag_allow:
      CommandData.use_mag = 1;
      break;
    case gps_allow:
      CommandData.use_gps = 1;
      break;
    case elenc1_allow:
      CommandData.use_elenc1 = 1;
      break;
    case elenc2_allow:
      CommandData.use_elenc2 = 1;
      break;

    case gps_off:          /* power switching */
      CommandData.power.gps.set_count = 0;
      CommandData.power.gps.rst_count = LATCH_PULSE_LEN;
      break;
    case gps_on:
      CommandData.power.gps.rst_count = 0;
      CommandData.power.gps.set_count = LATCH_PULSE_LEN;
      break;
    case gps_cycle:
      CommandData.power.gps.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.gps.rst_count = LATCH_PULSE_LEN;
      break;
    case rw_off:
      CommandData.power.rw.set_count = 0;
      CommandData.power.rw.rst_count = LATCH_PULSE_LEN;
      break;
    case rw_on:
      CommandData.power.rw.rst_count = 0;
      CommandData.power.rw.set_count = LATCH_PULSE_LEN;
      break;
    case rw_cycle:
      CommandData.power.rw.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.rw.rst_count = LATCH_PULSE_LEN;
      break;
    case piv_off:
      CommandData.power.piv.set_count = 0;
      CommandData.power.piv.rst_count = LATCH_PULSE_LEN;
      break;
    case piv_on:
      CommandData.power.piv.rst_count = 0;
      CommandData.power.piv.set_count = LATCH_PULSE_LEN;
      break;
    case piv_cycle:
      // Pivot takes a long time to properly power cycle.
      CommandData.power.piv.set_count = 2*PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.piv.rst_count = LATCH_PULSE_LEN;
      break;
    case elmot_off:
      CommandData.power.elmot_auto = 0;
      CommandData.power.elmot.set_count = 0;
      CommandData.power.elmot.rst_count = LATCH_PULSE_LEN;
      break;
    case elmot_on:
      CommandData.power.elmot_auto = 0;
      CommandData.power.elmot.rst_count = 0;
      CommandData.power.elmot.set_count = LATCH_PULSE_LEN;
      break;
    case elmot_cycle:
      CommandData.power.elmot_auto = 0;
      CommandData.power.elmot.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.elmot.rst_count = LATCH_PULSE_LEN;
      break;
    case elmot_auto:
      CommandData.power.elmot_auto = 1;
      break;
    case vtx_off:
      CommandData.power.sc_tx.set_count = 0;
      CommandData.power.sc_tx.rst_count = LATCH_PULSE_LEN;
      break;
    case vtx_on:
      CommandData.power.sc_tx.rst_count = 0;
      CommandData.power.sc_tx.set_count = LATCH_PULSE_LEN;
      break;
    case bi0_off:
      CommandData.power.bi0.set_count = 0;
      CommandData.power.bi0.rst_count = LATCH_PULSE_LEN;
      break;
    case bi0_on:
      CommandData.power.bi0.rst_count = 0;
      CommandData.power.bi0.set_count = LATCH_PULSE_LEN;
      break;
    case das_off:
      CommandData.power.das.set_count = 0;
      CommandData.power.das.rst_count = LATCH_PULSE_LEN;
      break;
    case das_on:
      CommandData.power.das.rst_count = 0;
      CommandData.power.das.set_count = LATCH_PULSE_LEN;
      break;
    case das_cycle:
      CommandData.power.das.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.das.rst_count = LATCH_PULSE_LEN;
      break;
    case table_off:
      CommandData.power.table.set_count = 0;
      CommandData.power.table.rst_count = LATCH_PULSE_LEN;
      break;
    case table_on:
      CommandData.power.table.rst_count = 0;
      CommandData.power.table.set_count = LATCH_PULSE_LEN;
      break;
    case table_cycle:
      CommandData.power.table.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.table.rst_count = LATCH_PULSE_LEN;
      break;
    case rsc_off:
      CommandData.power.rsc.set_count = 0;
      CommandData.power.rsc.rst_count = LATCH_PULSE_LEN;
      break;
    case rsc_on:
      CommandData.power.rsc.rst_count = 0;
      CommandData.power.rsc.set_count = LATCH_PULSE_LEN;
      break;
    case rsc_cycle:
      CommandData.power.rsc.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.rsc.rst_count = LATCH_PULSE_LEN;
      break;
    case bsc_off:
      CommandData.power.bsc.set_count = 0;
      CommandData.power.bsc.rst_count = LATCH_PULSE_LEN;
      break;
    case bsc_on:
      CommandData.power.bsc.rst_count = 0;
      CommandData.power.bsc.set_count = LATCH_PULSE_LEN;
      break;
    case bsc_cycle:
      CommandData.power.bsc.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.bsc.rst_count = LATCH_PULSE_LEN;
      break;
    case of_charge_off:
      CommandData.power.charge.rst_count = 0;
      CommandData.power.charge.set_count = LATCH_PULSE_LEN;
      break;
    case of_charge_on:
      CommandData.power.charge.set_count = 0;
      CommandData.power.charge.rst_count = LATCH_PULSE_LEN;
      break;
    case of_charge_cycle:
      CommandData.power.charge.rst_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.charge.set_count = LATCH_PULSE_LEN;
      break;
    case if_charge_on:
      CommandData.power.ifcharge.set_count = 0;
      CommandData.power.ifcharge.rst_count = LATCH_PULSE_LEN;
      break;
    case if_charge_off:
      CommandData.power.ifcharge.rst_count = 0;
      CommandData.power.ifcharge.set_count = LATCH_PULSE_LEN;
      break;
    case if_charge_cycle:
      CommandData.power.ifcharge.rst_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.ifcharge.set_count = LATCH_PULSE_LEN;
      break;
    case ofroll_1_gy_allow:
      CommandData.gymask |= 0x01;
      break;
    case ofroll_1_gy_veto:
      CommandData.gymask &= ~0x01;
      break;
    case ofroll_2_gy_allow:
      CommandData.gymask |= 0x02;
      break;
    case ofroll_2_gy_veto:
      CommandData.gymask &= ~0x02;
      break;
    case ofyaw_1_gy_allow:
      CommandData.gymask |= 0x04;
      break;
    case ofyaw_1_gy_veto:
      CommandData.gymask &= ~0x04;
      break;
    case ofyaw_2_gy_allow:
      CommandData.gymask |= 0x08;
      break;
    case ofyaw_2_gy_veto:
      CommandData.gymask &= ~0x08;
      break;
    case ofpch_1_gy_allow:
      CommandData.gymask |= 0x10;
      break;
    case ofpch_1_gy_veto:
      CommandData.gymask &= ~0x10;
      break;
    case ofpch_2_gy_allow:
      CommandData.gymask |= 0x20;
      break;
    case ofpch_2_gy_veto:
      CommandData.gymask &= ~0x20;
      break;
    case ofroll_1_gy_off:
      CommandData.power.gyro_off[1] = -1;
      break;
    case ofroll_1_gy_on:
      CommandData.power.gyro_off[1] = 0;
      break;
    case ofroll_1_gy_cycle:
      CommandData.power.gyro_off[1] = PCYCLE_HOLD_LEN;
      break;
    case ofroll_2_gy_off:
      CommandData.power.gyro_off[5] = -1;
      break;
    case ofroll_2_gy_on:
      CommandData.power.gyro_off[5] = 0;
      break;
    case ofroll_2_gy_cycle:
      CommandData.power.gyro_off[5] = PCYCLE_HOLD_LEN;
      break;
    case ofyaw_1_gy_off:
      CommandData.power.gyro_off[0] = -1;
      break;
    case ofyaw_1_gy_on:
      CommandData.power.gyro_off[0] = 0;
      break;
    case ofyaw_1_gy_cycle:
      CommandData.power.gyro_off[0] = PCYCLE_HOLD_LEN;
      break;
    case ofyaw_2_gy_off:
      CommandData.power.gyro_off[2] = -1;
      break;
    case ofyaw_2_gy_on:
      CommandData.power.gyro_off[2] = 0;
      break;
    case ofyaw_2_gy_cycle:
      CommandData.power.gyro_off[2] = PCYCLE_HOLD_LEN;
      break;
    case ofpch_1_gy_off:
      CommandData.power.gyro_off[3] = -1;
      break;
    case ofpch_1_gy_on:
      CommandData.power.gyro_off[3] = 0;
      break;
    case ofpch_1_gy_cycle:
      CommandData.power.gyro_off[3] = PCYCLE_HOLD_LEN;
      break;
    case ofpch_2_gy_off:
      CommandData.power.gyro_off[4] = -1;
      break;
    case ofpch_2_gy_on:
      CommandData.power.gyro_off[4] = 0;
      break;
    case ofpch_2_gy_cycle:
      CommandData.power.gyro_off[4] = PCYCLE_HOLD_LEN;
      break;
    case gybox_off:
      CommandData.power.gybox_off = -1;
      break;
    case gybox_on:
      CommandData.power.gybox_off = 0;
      break;
    case gybox_cycle:
      CommandData.power.gybox_off = PCYCLE_HOLD_LEN;
      break;
    case hub232_off:
      CommandData.power.hub232_off = -1;
      break;
    case hub232_on:
      CommandData.power.hub232_off = 0;
      break;
    case hub232_cycle:
      CommandData.power.hub232_off = PCYCLE_HOLD_LEN;
      break;
    case actbus_off:
      CommandData.actbus.off = -1;
      break;
    case actbus_on:
      CommandData.actbus.off = 0;
      CommandData.actbus.force_repoll = 1;
      break;
    case actbus_cycle:
      CommandData.actbus.off = PCYCLE_HOLD_LEN;
      CommandData.actbus.force_repoll = 1;
      break;
    case mcc1_on:
      CommandData.power.mcc1.set_count = LATCH_PULSE_LEN;
      CommandData.power.mcc1.rst_count = 0;
      break;
    case mcc1_off:
      CommandData.power.mcc1.rst_count = LATCH_PULSE_LEN;
      CommandData.power.mcc1.set_count = 0;
      break;
    case mcc1_cycle:
      CommandData.power.mcc1.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.mcc1.rst_count = LATCH_PULSE_LEN;
      break;
    case mcc2_on:
      CommandData.power.mcc2.set_count = LATCH_PULSE_LEN;
      CommandData.power.mcc2.rst_count = 0;
      break;
    case mcc2_off:
      CommandData.power.mcc2.rst_count = LATCH_PULSE_LEN;
      CommandData.power.mcc2.set_count = 0;
      break;
    case mcc2_cycle:
      CommandData.power.mcc2.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.mcc2.rst_count = LATCH_PULSE_LEN;
      break;
    case mcc3_on:
      CommandData.power.mcc3.set_count = LATCH_PULSE_LEN;
      CommandData.power.mcc3.rst_count = 0;
      break;
    case mcc3_off:
      CommandData.power.mcc3.rst_count = LATCH_PULSE_LEN;
      CommandData.power.mcc3.set_count = 0;
      break;
    case mcc3_cycle:
      CommandData.power.mcc3.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.mcc3.rst_count = LATCH_PULSE_LEN;
      break;
    case mcc4_on:
      CommandData.power.mcc4.set_count = LATCH_PULSE_LEN;
      CommandData.power.mcc4.rst_count = 0;
      break;
    case mcc4_off:
      CommandData.power.mcc4.rst_count = LATCH_PULSE_LEN;
      CommandData.power.mcc4.set_count = 0;
      break;
    case mcc4_cycle:
      CommandData.power.mcc4.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.mcc4.rst_count = LATCH_PULSE_LEN;
      break;
    case mcc5_on:
      CommandData.power.mcc5.set_count = LATCH_PULSE_LEN;
      CommandData.power.mcc5.rst_count = 0;
      break;
    case mcc5_off:
      CommandData.power.mcc5.rst_count = LATCH_PULSE_LEN;
      CommandData.power.mcc5.set_count = 0;
      break;
    case mcc5_cycle:
      CommandData.power.mcc5.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.mcc5.rst_count = LATCH_PULSE_LEN;
      break;
    case mcc6_on:
      CommandData.power.mcc6.set_count = LATCH_PULSE_LEN;
      CommandData.power.mcc6.rst_count = 0;
      break;
    case mcc6_off:
      CommandData.power.mcc6.rst_count = LATCH_PULSE_LEN;
      CommandData.power.mcc6.set_count = 0;
      break;
    case mcc6_cycle:
      CommandData.power.mcc6.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.mcc6.rst_count = LATCH_PULSE_LEN;
      break;

    /* Inner Frame Power */

    case mce23_off:
      CommandData.ifpower.mce_op[0] = off;
      break;
    case mce23_on:
      CommandData.ifpower.mce_op[0] = on;
      break;
    case mce23_cycle:
      CommandData.ifpower.mce_op[0] = cyc;
      break;
    case mce46_off:
      CommandData.ifpower.mce_op[1] = off;
      break;
    case mce46_on:
      CommandData.ifpower.mce_op[1] = on;
      break;
    case mce46_cycle:
      CommandData.ifpower.mce_op[1] = cyc;
      break;
    case mce15_off:
      CommandData.ifpower.mce_op[2] = off;
      break;
    case mce15_on:
      CommandData.ifpower.mce_op[2] = on;
      break;
    case mce15_cycle:
      CommandData.ifpower.mce_op[2] = cyc;
      break;
    case sync_off:
      CommandData.power.sync.set_count = 0;
      CommandData.power.sync.rst_count = LATCH_PULSE_LEN;
      break;
    case sync_on:
      CommandData.power.sync.rst_count = 0;
      CommandData.power.sync.set_count = LATCH_PULSE_LEN;
      break;
    case sync_cycle:
      CommandData.power.sync.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.sync.rst_count = LATCH_PULSE_LEN;
      break;
    case hwp_off:
      CommandData.ifpower.hwp.set_count = 0;
      CommandData.ifpower.hwp.rst_count = LATCH_PULSE_LEN;
      break;
    case hwp_on:
      CommandData.ifpower.hwp.rst_count = 0;
      CommandData.ifpower.hwp.set_count = LATCH_PULSE_LEN;
      break;
    case hwp_cycle:
      CommandData.ifpower.hwp.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.ifpower.hwp.rst_count = LATCH_PULSE_LEN;
      break;
    case hk_preamp_off:
      CommandData.ifpower.hk_preamp_off = -1;
      break;
    case hk_preamp_on:
      CommandData.ifpower.hk_preamp_off = 0;
      break;
    case hk_preamp_cycle:
      CommandData.ifpower.hk_preamp_off = PCYCLE_HOLD_LEN;
      break;

    /* Lock */
    case lock_on:
      CommandData.power.lock_off = 0;
      break;
    case lock_off:
      CommandData.power.lock_off = -1;
      break;
    case pin_in:
      CommandData.lock.goal = lock_insert;
      break;
    case lock:  /* Lock Inner Frame */
      CommandData.ele_gain.manual_pulses = 0;
      if (CommandData.pointing_mode.nw >= 0)
        CommandData.pointing_mode.nw = VETO_MAX;
      CommandData.lock.goal = lock_el_wait_insert;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_LOCK;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = LOCK_POSITION;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      break;
    case unlock:
      CommandData.lock.goal = lock_retract;
      if (CommandData.pointing_mode.mode == P_LOCK) {
        CommandData.pointing_mode.nw = CommandData.slew_veto;
        CommandData.pointing_mode.mode = P_DRIFT;
        CommandData.pointing_mode.X = 0;
        CommandData.pointing_mode.Y = 0;
        CommandData.pointing_mode.vaz = 0.0;
        CommandData.pointing_mode.del = 0.0;
        CommandData.pointing_mode.w = 0;
        CommandData.pointing_mode.h = 0;
      }
      break;

    /* Actuators */
    case actuator_stop:
      CommandData.actbus.focus_mode = ACTBUS_FM_PANIC;
      CommandData.actbus.tc_mode = TC_MODE_VETOED;
      break;

    case hwp_panic:
      CommandData.hwp.mode = hwp_m_panic;
      break;
    case hwp_repoll:
      //TODO integrate repoll into hwp on/cycle commands, when implemented
      CommandData.hwp.force_repoll = 1;
      break;
    case hwp_step:
      CommandData.hwp.mode = hwp_m_step;
      break;

    case repoll:
      CommandData.actbus.force_repoll = 1;
      CommandData.xystage.force_repoll = 1;
      break;

      /***************************************/
      /********* The Good Commanding  *************/
    case thegood_expose:
      sendTheGoodCommand("CtrigExp");
      break;
    case thegood_autofocus:
      sendTheGoodCommand("CtrigFocus");
      break;
    case thegood_settrig_ext:
      sendTheGoodCommand("CsetExpInt=0");
      CommandData.StarCam[0].expInt = 0;
      break;
    case thegood_pause:
      CommandData.StarCam[0].paused = 1;
      break;
    case thegood_run:
      CommandData.StarCam[0].paused = 0;
      break;
      /***************************************/
      /********* The Bad Commanding  *************/
    case thebad_expose:
      sendTheBadCommand("CtrigExp");
      break;
    case thebad_autofocus:
      sendTheBadCommand("CtrigFocus");
      break;
    case thebad_settrig_ext:
      sendTheBadCommand("CsetExpInt=0");
      CommandData.StarCam[1].expInt = 0;
      break;
    case thebad_pause:
      CommandData.StarCam[1].paused = 1;
      break;
    case thebad_run:
      CommandData.StarCam[1].paused = 0;
      break;
      /***************************************/
      /********* The Ugly Commanding  *************/
    case theugly_expose:
      sendTheUglyCommand("CtrigExp");
      break;
    case theugly_autofocus:
      sendTheUglyCommand("CtrigFocus");
      break;
    case theugly_settrig_ext:
      sendTheUglyCommand("CsetExpInt=0");
      CommandData.StarCam[2].expInt = 0;
      break;
    case theugly_pause:
      CommandData.StarCam[2].paused = 1;
      break;
    case theugly_run:
      CommandData.StarCam[2].paused = 0;
      break;
      /**************************************/
      /********** Star Camera Table *********/
    case table_track:
      CommandData.table.mode = 0;

    case blast_rocks:
      CommandData.sucks = 0;
      CommandData.uplink_sched = 0;
      break;
    case blast_sucks:
      CommandData.sucks = 1;
      CommandData.uplink_sched = 0;
      break;

    case at_float:
      CommandData.at_float = 1;
      break;
    case not_at_float:
      CommandData.at_float = 0;
      break;
    case vtx1_isc:
      CommandData.vtx_sel[0] = vtx_isc;
      break;
    case vtx1_osc:
      CommandData.vtx_sel[0] = vtx_osc;
      break;
    case vtx1_bsc:
      CommandData.vtx_sel[0] = vtx_bsc;
      break;
    case vtx2_isc:
      CommandData.vtx_sel[1] = vtx_isc;
      break;
    case vtx2_osc:
      CommandData.vtx_sel[1] = vtx_osc;
      break;
    case vtx2_bsc:
      CommandData.vtx_sel[1] = vtx_bsc;
      break;

    case reap_itsy:  /* Miscellaneous commands */
    case reap_bitsy:
      if ((command == reap_itsy && !BitsyIAm) ||
          (command == reap_bitsy && BitsyIAm)) {
        bprintf(err, "Reaping the watchdog tickle on command.");
        pthread_cancel(watchdog_id);
      }
      break;
    case halt_itsy:
    case halt_bitsy:
      if ((command == halt_itsy && !BitsyIAm) ||
          (command == halt_bitsy && BitsyIAm)) {
        bputs(warning, "Halting the MCC\n");
        if (system("/sbin/reboot") < 0)
          berror(fatal, "failed to reboot, dying\n");
      }
      break;
    case bbc_sync_ext:
      is_new = !CommandData.bbcIsExt;
      CommandData.bbcAutoExt = 0;
      CommandData.bbcIsExt = 1;
      if (is_new) setup_bbc();
      break;
    case bbc_sync_int:
      is_new = CommandData.bbcIsExt;
      CommandData.bbcAutoExt = 0;
      CommandData.bbcIsExt = 0;
      if (is_new) setup_bbc();
      break;
    case bbc_sync_auto:
      CommandData.bbcAutoExt = 1;
      break;

    //Theo heater housekeeping commands.
    case hk_mt_bottom_heat_on:
      CommandData.hk_theo_heat[0].state = 1;
      CommandData.hk_theo_heat[0].duration = 0;
      CommandData.hk_theo_heat[0].start_time = 0;
      break;
    case hk_mt_bottom_heat_off:
      CommandData.hk_theo_heat[0].state = 0;
      CommandData.hk_theo_heat[0].duration = 0;
      CommandData.hk_theo_heat[0].start_time = 0;
      break;
    case hk_sft_lines_heat_on:
      CommandData.hk_theo_heat[1].state = 1;
      CommandData.hk_theo_heat[1].duration = 0;
      CommandData.hk_theo_heat[1].start_time = 0;
      break;
    case hk_sft_lines_heat_off:
      CommandData.hk_theo_heat[1].state = 0;
      CommandData.hk_theo_heat[1].duration = 0;
      CommandData.hk_theo_heat[1].start_time = 0;
      break;
    case hk_capillary_heat_on:
      CommandData.hk_theo_heat[2].state = 1;
      CommandData.hk_theo_heat[2].duration = 0;
      CommandData.hk_theo_heat[2].start_time = 0;
      break;
    case hk_capillary_heat_off:
      CommandData.hk_theo_heat[2].state = 0;
      CommandData.hk_theo_heat[2].duration = 0;
      CommandData.hk_theo_heat[2].start_time = 0;
      break;
    case hk_vcs2_hx_heat_on:
      CommandData.hk_theo_heat[3].state = 1;
      CommandData.hk_theo_heat[3].duration = 0;
      CommandData.hk_theo_heat[3].start_time = 0;
      break;
    case hk_vcs2_hx_heat_off:
      CommandData.hk_theo_heat[3].state = 0;
      CommandData.hk_theo_heat[3].duration = 0;
      CommandData.hk_theo_heat[3].start_time = 0;
      break;
    case hk_vcs1_hx_heat_on:
      CommandData.hk_theo_heat[4].state = 1;
      CommandData.hk_theo_heat[4].duration = 0;
      CommandData.hk_theo_heat[4].start_time = 0;
      break;
    case hk_vcs1_hx_heat_off:
      CommandData.hk_theo_heat[4].state = 0;
      CommandData.hk_theo_heat[4].duration = 0;
      CommandData.hk_theo_heat[4].start_time = 0;
      break;
    case hk_mt_lines_heat_on:
      CommandData.hk_theo_heat[5].state = 1;
      CommandData.hk_theo_heat[5].duration = 0;
      CommandData.hk_theo_heat[5].start_time = 0;
      break;
    case hk_mt_lines_heat_off:
      CommandData.hk_theo_heat[5].state = 0;
      CommandData.hk_theo_heat[5].duration = 0;
      CommandData.hk_theo_heat[5].start_time = 0;
      break;
    case hk_sft_bottom_heat_on:
      CommandData.hk_theo_heat[6].state = 1;
      CommandData.hk_theo_heat[6].duration = 0;
      CommandData.hk_theo_heat[6].start_time = 0;
      break;
    case hk_sft_bottom_heat_off:
      CommandData.hk_theo_heat[6].state = 0;
      CommandData.hk_theo_heat[6].duration = 0;
      CommandData.hk_theo_heat[6].start_time = 0;
      break;
   case mcc_wdog_enable:
      CommandData.mcc_wdog = 1;
      break;
   case mcc_wdog_disable:
      CommandData.mcc_wdog = 0;
      break;
   case get_superslow:
      request_ssdata = 1;
      break;

    case pull_cmb_pin:
      bputs(info, "... What now?");
      CommandData.questionable_behaviour++;
      break;
    case global_thermonuclear_war:
      bputs(info, "The only winning move is not to play");
      CommandData.questionable_behaviour++;
      break;
#if 0
    case get_some:
      bputs(info, "Aaaarrrgggh!");
      CommandData.questionable_behaviour++;
      break;
    case stab:
      bputs(info, "Swish, slash");
      CommandData.questionable_behaviour++;
      break;
    case lock_and_load:
      bputs(info, "Cachink.");
      CommandData.questionable_behaviour++;
      break;
#endif

    case xyzzy:
      break;
    default:
      /* Render, therefore, unto Caesar the things which are Caesar's */
      if (!MCEcmd(command, NULL, NULL, NULL)) {
        bputs(warning, "Invalid Single Word Command\n");
        return; /* invalid command - no write or update */
      }
  }
  
  CommandData.command_count++;
  CommandData.last_command = (unsigned short)command;
  

  if (!scheduled) {
    if (doing_schedule)
      bprintf(info, "*** Out of schedule file mode ***");
    CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
  } else
    CommandData.pointing_mode.t = PointingData[i_point].t;

  WritePrevStatus();
}

static inline void copysvalue(char* dest, const char* src)
{
  strncpy(dest, src, CMD_STRING_LEN - 1);
  dest[CMD_STRING_LEN - 1] = '\0';
}

void MultiCommand(enum multiCommand command, double *rvalues,
    int *ivalues, char svalues[][CMD_STRING_LEN], int scheduled)
{
  int i;
  char buf[256]; //for SC Commands
  int is_new;
  double step;
  double duty_cycle;

  /* Update CommandData struct with new info
   * If the parameter is type 'i'/'l' set CommandData using ivalues[i]
   * If the parameter is type 'f'/'d' set CommandData using rvalues[i]
   */

  /* Pointing Modes */
  switch(command) {
    case az_el_goto:
      if ((CommandData.pointing_mode.mode != P_AZEL_GOTO) ||
          (CommandData.pointing_mode.X != rvalues[0]) ||
          (CommandData.pointing_mode.Y != rvalues[1])) {
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }
      CommandData.ele_gain.manual_pulses = 0;
      CommandData.pointing_mode.mode = P_AZEL_GOTO;
      CommandData.pointing_mode.el_mode = P_EL_GOTO;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      break;
    case az_scan:
      CommandData.ele_gain.manual_pulses = 0;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_AZ_SCAN;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      CommandData.pointing_mode.w = rvalues[2];  /* width */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      break;
    case drift:
      CommandData.ele_gain.manual_pulses = 0;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = rvalues[0]; /* az speed */
      CommandData.pointing_mode.del = 0; 
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      break;
    case ra_dec_goto:
      CommandData.ele_gain.manual_pulses = 0;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_RADEC_GOTO;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      break;
    case spider_scan:
      is_new = 0;
      if ( (CommandData.pointing_mode.mode != P_SPIDER) ||
          (CommandData.pointing_mode.X != rvalues[8]) || /* starting ra */
          (CommandData.pointing_mode.Y != rvalues[9]) ) { /* starting dec */
        is_new = 1;
      }
      for (i = 0; i < 4; i++) {
        if ( (CommandData.pointing_mode.ra[i] != rvalues[i*2]) ||
            (CommandData.pointing_mode.dec[i] != rvalues[i*2 + 1]) ) {
          is_new = 1;
        }
      }
      if (is_new) {
        CommandData.pointing_mode.new_spider = 1;
      }
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = rvalues[i*2];
        CommandData.pointing_mode.dec[i] = rvalues[i*2 + 1];
      }
      CommandData.pointing_mode.X = rvalues[8];
      CommandData.pointing_mode.Y = rvalues[9];
      CommandData.ele_gain.manual_pulses = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      CommandData.pointing_mode.mode = P_SPIDER;
      CommandData.pointing_mode.el_mode = P_EL_GOTO;
      break;
    case set_scan_params:
      CommandData.az_accel = rvalues[0];
      CommandData.az_accel_max = rvalues[1];
      CommandData.pointing_mode.Nscans = ivalues[2];
      CommandData.pointing_mode.del = CommandData.pointing_mode.el_step
        = rvalues[3];
      CommandData.pointing_mode.Nsteps = ivalues[4];
      CommandData.pointing_mode.overshoot_band = rvalues[5];
      CommandData.pointing_mode.az_delay = rvalues[6];
      break;
    case sine_scan:
      CommandData.pointing_mode.w = 2.0*rvalues[0];
      CommandData.pointing_mode.X = rvalues[1];
      CommandData.pointing_mode.Y = rvalues[2];
      CommandData.ele_gain.manual_pulses = 0;
      CommandData.pointing_mode.is_turn_around = 0;
      CommandData.pointing_mode.mode = P_SINE;
      break;
      /***************************************/
      /********** Pointing Motor Trims *******/
    case az_el_trim:
      AzElTrim(rvalues[0], rvalues[1]);
      break;
    case ra_dec_set:
      SetRaDec(rvalues[0], rvalues[1]);
      break;
    case az_gyro_offset:
      CommandData.offset_ofroll_gy = rvalues[0];
      CommandData.offset_ofyaw_gy = rvalues[1];
      CommandData.az_autogyro = 0;
      break;
    case el_gyro_offset:
      CommandData.offset_ofpch_gy = rvalues[0];
      CommandData.el_autogyro = 0;
      break;
    case slew_veto:
      CommandData.slew_veto = rvalues[0] * ACSData.bbc_rate;
            bprintf(info,"CommandData.slew_veto = %i,"
                " CommandData.pointing_mode.nw = %i",
                CommandData.slew_veto, CommandData.pointing_mode.nw);
      if (CommandData.pointing_mode.nw > CommandData.slew_veto)
        CommandData.pointing_mode.nw = CommandData.slew_veto;

      break;

      /***************************************/
      /********** Pointing Motor Gains *******/
    case el_gain:  /* ele gains */
      CommandData.ele_gain.com = rvalues[0];
      break;
   case el_pulse: /* manual el motor pulses */
      CommandData.ele_gain.pulse_port = rvalues[0];
      CommandData.ele_gain.pulse_starboard = rvalues[1];
      CommandData.ele_gain.manual_pulses = 1;
      break;
   case el_rel_move: /* el relative move (separate for each side) */
      CommandData.pointing_mode.d_el_p = rvalues[0];
      CommandData.pointing_mode.d_el_s = rvalues[1];
      CommandData.pointing_mode.v_el_p = rvalues[2];
      CommandData.pointing_mode.v_el_s = rvalues[3];
      CommandData.pointing_mode.el_rel_move = 1;
      CommandData.pointing_mode.el_mode = P_EL_RELMOVE;
      break;
    case az_gain:  /* az gains */
      CommandData.azi_gain.P = ivalues[0];
      CommandData.azi_gain.I = ivalues[1];
      CommandData.azi_gain.PT = ivalues[2];
      break;
    case pivot_gain:  /* pivot gains */
      CommandData.pivot_gain.SP =   rvalues[0];
      CommandData.pivot_gain.PV = ivalues[1];
      CommandData.pivot_gain.V_RW = ivalues[2];
      CommandData.pivot_gain.PE = ivalues[3];
      CommandData.pivot_gain.T_RW = ivalues[4];
      CommandData.pivot_gain.F = rvalues[5];
      CommandData.pivot_gain.V_REQ = ivalues[6];
      break;
    case set_piv_mode: /* switch btwn vel/torque mode*/
      CommandData.pointing_mode.piv_mode = ivalues[0];
      break;

      //***********DGPS tests****************/
    case cov_gps:
      CommandData.dgps_cov_limit = rvalues[0];
      break;
    case ants_gps:
      CommandData.dgps_ants_limit = rvalues[0];
      break;
      /*************************************/

      /***************************************/
      /*****           test of motor DACs ****/
    case dac2_level:
      CommandData.Temporary.dac_out[1] = ivalues[0] << 1;
      CommandData.Temporary.setLevel[1] = 1;
      break;
    case motors_verbose:
      CommandData.verbose_rw = ivalues[0];
      CommandData.verbose_piv = ivalues[1];
      break;

      /***************************************/
      /**********        Actuators  **********/
    case general: /* General actuator bus command */
      CommandData.actbus.caddr[CommandData.actbus.cindex] = ivalues[0] + 0x30;
      copysvalue(CommandData.actbus.command[CommandData.actbus.cindex],
          svalues[1]);
      CommandData.actbus.cindex = INC_INDEX(CommandData.actbus.cindex);
      break;
    case actuator_servo:
      CommandData.actbus.goal[0] = ivalues[0] + CommandData.actbus.offset[0];
      CommandData.actbus.goal[1] = ivalues[1] + CommandData.actbus.offset[1];
      CommandData.actbus.goal[2] = ivalues[2] + CommandData.actbus.offset[2];
      CommandData.actbus.focus_mode = ACTBUS_FM_SERVO;
      break;
    case actuator_delta:
      CommandData.actbus.delta[0] = ivalues[0];
      CommandData.actbus.delta[1] = ivalues[1];
      CommandData.actbus.delta[2] = ivalues[2];
      CommandData.actbus.focus_mode = ACTBUS_FM_DELTA;
      break;
    case actuator_vel:
      CommandData.actbus.act_vel = ivalues[0];
      CommandData.actbus.act_acc = ivalues[1];
      break;
    case actuator_i:
      CommandData.actbus.act_move_i = ivalues[0];
      CommandData.actbus.act_hold_i = ivalues[1];
      break;
    case actuator_tol:
      CommandData.actbus.act_tol = ivalues[0];
      break;
    case act_offset:
      CommandData.actbus.offset[0] = (int)(rvalues[0]+0.5);
      CommandData.actbus.offset[1] = (int)(rvalues[1]+0.5);
      CommandData.actbus.offset[2] = (int)(rvalues[2]+0.5);
      break;
    case act_enc_trim:
      CommandData.actbus.trim[0] = rvalues[0];
      CommandData.actbus.trim[1] = rvalues[1];
      CommandData.actbus.trim[2] = rvalues[2];
      CommandData.actbus.focus_mode = ACTBUS_FM_TRIM;
      break;
    case lvdt_limit:
      CommandData.actbus.lvdt_delta = rvalues[0];
      if (rvalues[1] > rvalues[2]) {
        CommandData.actbus.lvdt_low = rvalues[2];
        CommandData.actbus.lvdt_high = rvalues[1];
      } else {
        CommandData.actbus.lvdt_low = rvalues[1];
        CommandData.actbus.lvdt_high = rvalues[2];
      }
      break;

    case hwp_vel:
      CommandData.hwp.vel = rvalues[0];
      break;
    case hwp_i:
      CommandData.hwp.move_i = rvalues[0];
      break;
    case hwp_phase:
      CommandData.hwp.phase = rvalues[0];
      break;
    case hwp_move:
      CommandData.hwp.who = ivalues[0] - 1;
      CommandData.hwp.delta = rvalues[1];
      CommandData.hwp.mode = hwp_m_rel_move;
      break;
    case hwp_halt:
      CommandData.hwp.who = ivalues[0] - 1;
      CommandData.hwp.mode = hwp_m_halt;
      break;
    case hwp_bias_off:
      // extra << 6 is to use bits that turn bias off, not always on
      if (ivalues[0]) {
        CommandData.hwp.bias_mask |= ( 0x01 << (ivalues[0]-1) ) << 6;
      } else {
        CommandData.hwp.bias_mask |= 0x3f << 6;
      }
      break;
    case hwp_bias_on:
      // extra << 6 is to use bits that turn bias off, not always on
      if (ivalues[0]) {
        CommandData.hwp.bias_mask &= ~( 0x01 << (ivalues[0]-1) << 6);
      } else {
        CommandData.hwp.bias_mask &= ~(0x3f << 6);
      }
      break;
    case hwp_general:
      CommandData.hwp.caddr[CommandData.hwp.cindex] = ivalues[0] - 1;
      copysvalue(CommandData.hwp.command[CommandData.hwp.cindex], svalues[1]);
      CommandData.hwp.cindex = INC_INDEX(CommandData.hwp.cindex);
      break;

      /***************************************/
      /******** Electronics Heaters  *********/
    case t_gyro_set:  /* gyro heater setpoint */
      CommandData.gyheat.setpoint = rvalues[0];
      break;
    case t_gyro_gain:  /* gyro heater gains */
      CommandData.gyheat.gain.P = ivalues[0];
      CommandData.gyheat.gain.I = ivalues[1];
      CommandData.gyheat.gain.D = ivalues[2];
      break;
    case t_rsc_set:  /* RSC heater setpoint */
      CommandData.t_set_rsc = rvalues[0];
      break;
    case t_bsc_set:  /* BSC heater setpoint */
      CommandData.t_set_bsc = rvalues[0];
      break;

      /***************************************/
      /*************** Misc  *****************/
//    case gyro_off:
//      CommandData.power.gyro_off[ivalues[0]-1] |= 0x01;
//      break;
//    case gyro_on:
//      CommandData.power.gyro_off[ivalues[0]-1] &= ~0x01;
//      break;
    case reset_adc:
      if (ivalues[0] < 64)
        CommandData.power.adc_reset[ivalues[0]/4] = RESET_ADC_LEN;
      break;
    case timeout:       /* Set timeout */
      CommandData.timeout = rvalues[0];
      break;
    case pilot_bw:
      CommandData.pilot_bw = rvalues[0];
      break;
    case tdrss_bw:
      CommandData.tdrss_bw = rvalues[0];
      break;
    case iridium_bw:
      CommandData.iridium_bw = rvalues[0];
      break;
    case oth_set:
      CommandData.channelset_oth = ivalues[0];
      break;
    case get_mce_param:
      CommandData.mce_param_index = ivalues[0];
      break;
    case slot_sched: /* change uplinked schedule file */
      if (LoadUplinkFile(ivalues[0])) {
        CommandData.uplink_sched = 1;
        CommandData.slot_sched = ivalues[0];
      }
      break;
    case bbc_rate_ext:
      CommandData.bbcExtFrameRate = ivalues[0];
      setup_bbc();
      break;
    case bbc_rate_int:
      CommandData.bbcIntFrameRate = ivalues[0];
      setup_bbc();
      break;
    case bset:
      change_bset(ivalues[0], 0);
      break;

    case plugh:/* A hollow voice says "Plugh". */
      CommandData.plover = ivalues[0];
      break;

      /***************************************/
      /*************** Bias  *****************/
    case hk_ampl_ntd:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ntd.ampl = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ntd.ampl = rvalues[1];
      break;
    case hk_ampl_cernox:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].cernox.ampl = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].cernox.ampl = rvalues[1];
      break;
    case hk_phase_ntd:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].ntd.phase = rvalues[1];
        CommandData.hk[ivalues[0]-1].ntd.do_phase_step = 0;
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].ntd.phase = rvalues[1];
        CommandData.hk[i].ntd.do_phase_step = 0;
      }
      break;
    case hk_phase_cernox:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].cernox.phase= rvalues[1];
        CommandData.hk[ivalues[0]-1].cernox.do_phase_step = 0;
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].cernox.phase = rvalues[1];
        CommandData.hk[i].cernox.do_phase_step = 0;
      }
      break;
    case hk_phase_step_ntd:
      CommandData.hk_last = ivalues[0];
      step = (rvalues[2]-rvalues[1])/rvalues[3];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].ntd.do_phase_step = 1;
        CommandData.hk[ivalues[0]-1].ntd.phase = rvalues[1];
        CommandData.hk[ivalues[0]-1].ntd.phase_start = rvalues[1];
        CommandData.hk[ivalues[0]-1].ntd.phase_end = rvalues[2];
        CommandData.hk[ivalues[0]-1].ntd.phase_step = step;
        CommandData.hk[ivalues[0]-1].ntd.phase_dt = ivalues[4];
        CommandData.hk[ivalues[0]-1].ntd.phase_time = mcp_systime(NULL);
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].ntd.do_phase_step = 1;
        CommandData.hk[i].ntd.phase = rvalues[1];
        CommandData.hk[i].ntd.phase_start = rvalues[1];
        CommandData.hk[i].ntd.phase_end = rvalues[2];
        CommandData.hk[i].ntd.phase_step = step;
        CommandData.hk[i].ntd.phase_dt = ivalues[4];
        CommandData.hk[i].ntd.phase_time = mcp_systime(NULL);
      }
      break;
    case hk_phase_step_cernox:
      CommandData.hk_last = ivalues[0];
      step = (rvalues[2]-rvalues[1])/rvalues[3];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].cernox.do_phase_step = 1;
        CommandData.hk[ivalues[0]-1].cernox.phase = rvalues[1];
        CommandData.hk[ivalues[0]-1].cernox.phase_start = rvalues[1];
        CommandData.hk[ivalues[0]-1].cernox.phase_end = rvalues[2];
        CommandData.hk[ivalues[0]-1].cernox.phase_step = step;
        CommandData.hk[ivalues[0]-1].cernox.phase_dt = ivalues[4];
        CommandData.hk[ivalues[0]-1].cernox.phase_time = mcp_systime(NULL);
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].cernox.do_phase_step = 1;
        CommandData.hk[i].cernox.phase = rvalues[1];
        CommandData.hk[i].cernox.phase_start = rvalues[1];
        CommandData.hk[i].cernox.phase_end = rvalues[2];
        CommandData.hk[i].cernox.phase_step = step;
        CommandData.hk[i].cernox.phase_dt = ivalues[4];
        CommandData.hk[i].cernox.phase_time = mcp_systime(NULL);
      }
      break;
    case hk_bias_freq:
      //TODO consider scaling phases as fixed time delay, when freq changes
      CommandData.hk_bias_freq = ivalues[0];
      break;

      /***************************************/
      /*************** Heat  *****************/
    case hk_pump_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].pump_heat = 1;
        CommandData.hk[ivalues[0]-1].auto_cycle_on = 0;
        CommandData.hk[ivalues[0]-1].pump_servo_on = 0;
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].pump_heat = 1;
        CommandData.hk[i].auto_cycle_on = 0;
        CommandData.hk[i].pump_servo_on = 0;
      }
      break;
    case hk_pump_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].pump_heat = 0;
        CommandData.hk[ivalues[0]-1].auto_cycle_on = 0;
        CommandData.hk[ivalues[0]-1].pump_servo_on = 0;
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].pump_heat = 0;
        CommandData.hk[i].auto_cycle_on = 0;
        CommandData.hk[i].pump_servo_on = 0;
      }
      break;
    case hk_heat_switch_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].heat_switch = 1;
        CommandData.hk[ivalues[0]-1].auto_cycle_on = 0;
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].heat_switch = 1;
        CommandData.hk[i].auto_cycle_on = 0;
      }
      break;
    case hk_heat_switch_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].heat_switch = 0;
        CommandData.hk[ivalues[0]-1].auto_cycle_on = 0;
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].heat_switch = 0;
        CommandData.hk[i].auto_cycle_on = 0;
      }
      break;
    case hk_fphi_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].fphi_heat = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].fphi_heat = 1;
      break;
    case hk_fphi_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].fphi_heat = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].fphi_heat = 0;
      break;
    case hk_ssa_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ssa_heat = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ssa_heat = 1;
      break;
    case hk_ssa_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ssa_heat = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ssa_heat = 0;
      break;
    case hk_htr1_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].htr1_heat = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].htr1_heat = 1;
      break;
    case hk_htr1_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].htr1_heat = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].htr1_heat = 0;
      break;
    case hk_htr2_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].htr2_heat = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].htr2_heat = 1;
      break;
    case hk_htr2_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].htr2_heat = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].htr2_heat = 0;
      break;
    case hk_htr3_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].htr3_heat = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].htr3_heat = 1;
      break;
    case hk_htr3_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].htr3_heat = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].htr3_heat = 0;
      break;
    case hk_fplo_heat_set:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_vheat_last = rvalues[1];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].fplo_heat = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].fplo_heat = rvalues[1];
      break;
    case hk_ring_heat_set:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_vheat_last = rvalues[1];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ring_heat = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ring_heat = rvalues[1];
      break;

    case hk_auto_cycle_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].auto_cycle_on = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].auto_cycle_on = 1;
      break;
    case hk_auto_cycle_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].auto_cycle_on = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].auto_cycle_on = 0;
      break;
    case hk_fridge_cycle:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].auto_cycle_on = 1;
        CommandData.hk[ivalues[0]-1].force_cycle = 1;
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].auto_cycle_on = 1;
        CommandData.hk[i].force_cycle = 1;
      }
      break;
    case hk_pump_servo_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) {
        CommandData.hk[ivalues[0]-1].pump_servo_on = 1;
        CommandData.hk[ivalues[0]-1].pump_servo_low = rvalues[1];
        CommandData.hk[ivalues[0]-1].pump_servo_high = rvalues[2];
      } else for (i=0; i<HK_MAX; i++) {
        CommandData.hk[i].pump_servo_on = 1;
        CommandData.hk[i].pump_servo_low = rvalues[1];
        CommandData.hk[i].pump_servo_high = rvalues[2];
      }
      break;

    case hk_mt_bottom_pulse:
      duty_cycle = rvalues[0]/HK_MT_BOTTOM_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      CommandData.hk_theo_heat[0].duty_target =
        ((int)(duty_cycle*256) << 8); // 8-bit resolution, pad with zeros
      CommandData.hk_theo_heat[0].duration = // seconds
        ( (rvalues[1]<0) ? -1 : (int)(rvalues[1]*60) );
      // initialize
      CommandData.hk_theo_heat[0].duty_avg =
        CommandData.hk_theo_heat[0].duty_target;
      CommandData.hk_theo_heat[0].start_time = mcp_systime(NULL);
      break;
    case hk_sft_lines_pulse:
      duty_cycle = rvalues[0]/HK_SFT_LINES_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      CommandData.hk_theo_heat[1].duty_target =
        ((int)(duty_cycle*256) << 8); // 8-bit resolution, pad with zeros
      CommandData.hk_theo_heat[1].duration = // seconds
        ( (rvalues[1]<0) ? -1 : (int)(rvalues[1]*60) );
      // initialize
      CommandData.hk_theo_heat[1].duty_avg =
        CommandData.hk_theo_heat[1].duty_target;
      CommandData.hk_theo_heat[1].start_time = mcp_systime(NULL);
      break;
    case hk_capillary_pulse:
      duty_cycle = rvalues[0]/HK_CAPILLARY_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      CommandData.hk_theo_heat[2].duty_target =
        ((int)(duty_cycle*256) << 8); // 8-bit resolution, pad with zeros
      CommandData.hk_theo_heat[2].duration = // seconds
        ( (rvalues[1]<0) ? -1 : (int)(rvalues[1]*60) );
      // initialize
      CommandData.hk_theo_heat[2].duty_avg =
        CommandData.hk_theo_heat[2].duty_target;
      CommandData.hk_theo_heat[2].start_time = mcp_systime(NULL);
      break;
    case hk_vcs2_hx_pulse:
      duty_cycle = rvalues[0]/HK_VCS2_HX_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      CommandData.hk_theo_heat[3].duty_target =
        ((int)(duty_cycle*256) << 8); // 8-bit resolution, pad with zeros
      CommandData.hk_theo_heat[3].duration = // seconds
        ( (rvalues[1]<0) ? -1 : (int)(rvalues[1]*60) );
      // initialize
      CommandData.hk_theo_heat[3].duty_avg =
        CommandData.hk_theo_heat[3].duty_target;
      CommandData.hk_theo_heat[3].start_time = mcp_systime(NULL);
      break;
    case hk_vcs1_hx_pulse:
      duty_cycle = rvalues[0]/HK_VCS1_HX_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      CommandData.hk_theo_heat[4].duty_target =
        ((int)(duty_cycle*256) << 8); // 8-bit resolution, pad with zeros
      CommandData.hk_theo_heat[4].duration = // seconds
        ( (rvalues[1]<0) ? -1 : (int)(rvalues[1]*60) );
      // initialize
      CommandData.hk_theo_heat[4].duty_avg =
        CommandData.hk_theo_heat[4].duty_target;
      CommandData.hk_theo_heat[4].start_time = mcp_systime(NULL);
      break;
    case hk_mt_lines_pulse:
      duty_cycle = rvalues[0]/HK_MT_LINES_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      CommandData.hk_theo_heat[5].duty_target =
        ((int)(duty_cycle*256) << 8); // 8-bit resolution, pad with zeros
      CommandData.hk_theo_heat[5].duration = // seconds
        ( (rvalues[1]<0) ? -1 : (int)(rvalues[1]*60) );
      // initialize
      CommandData.hk_theo_heat[5].duty_avg =
        CommandData.hk_theo_heat[5].duty_target;
      CommandData.hk_theo_heat[5].start_time = mcp_systime(NULL);
      break;
    case hk_sft_bottom_pulse:
      duty_cycle = rvalues[0]/HK_SFT_BOTTOM_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      CommandData.hk_theo_heat[6].duty_target =
        ((int)(duty_cycle*256) << 8); // 8-bit resolution, pad with zeros
      CommandData.hk_theo_heat[6].duration = // seconds
        ( (rvalues[1]<0) ? -1 : (int)(rvalues[1]*60) );
      // initialize
      CommandData.hk_theo_heat[6].duty_avg =
        CommandData.hk_theo_heat[6].duty_target;
      CommandData.hk_theo_heat[6].start_time = mcp_systime(NULL);
      break;

      /***************************************/
      /********* The Good Commanding  *************/
    case thegood_any:
      sendTheGoodCommand(svalues[0]);
      break;
    case thegood_settrig_timed:
      sprintf(buf, "CsetExpInt=%d", ivalues[0]);
      sendTheGoodCommand(buf);
      CommandData.StarCam[0].expInt = ivalues[0];
      break;
    case thegood_exp_params:
      sprintf(buf, "CsetExpTime=%d", ivalues[0]);
      CommandData.StarCam[0].expTime = ivalues[0];
      sendTheGoodCommand(buf);
      break;
    case thegood_focus_params:
      sprintf(buf, "CsetFocRsln=%d", ivalues[0]);
      sendTheGoodCommand(buf);
      sprintf(buf, "CsetFocRnge=%d", ivalues[1]);
      sendTheGoodCommand(buf);
      CommandData.StarCam[0].focusRes = ivalues[0];
      CommandData.StarCam[0].focusRng = ivalues[1];
      break;
    case thegood_bad_pix:
      sprintf(buf, "IsetBadpix=%d %d %d", ivalues[0], ivalues[1], ivalues[2]);
      sendTheGoodCommand(buf);
      break;
    case thegood_blob_params:
      sprintf(buf, "IsetMaxBlobs=%d", ivalues[0]);
      sendTheGoodCommand(buf);
      sprintf(buf, "IsetGrid=%d", ivalues[1]);
      sendTheGoodCommand(buf);
      sprintf(buf, "IsetThreshold=%f", rvalues[2]);
      sendTheGoodCommand(buf);
      sprintf(buf, "IsetDisttol=%d", ivalues[3]);
      sendTheGoodCommand(buf);
      CommandData.StarCam[0].maxBlobs = ivalues[0];
      CommandData.StarCam[0].grid = ivalues[1];
      CommandData.StarCam[0].threshold = rvalues[2];
      CommandData.StarCam[0].minBlobDist = ivalues[3];
      break;
    case thegood_lens_any:
      sprintf(buf, "L=%s", svalues[0]);
      sendTheGoodCommand(buf);
      break;
    case thegood_lens_move:
      sprintf(buf, "Lmove=%d", ivalues[0]);
      sendTheGoodCommand(buf);
      break;
    case thegood_lens_params:
      sprintf(buf, "LsetTol=%d", ivalues[0]);
      sendTheGoodCommand(buf);
      CommandData.StarCam[0].moveTol = ivalues[0];
      break;
      /***************************************/
      /********* The Bad Commanding  *************/
    case thebad_any:
      sendTheBadCommand(svalues[0]);
      break;
    case thebad_settrig_timed:
      sprintf(buf, "CsetExpInt=%d", ivalues[0]);
      sendTheBadCommand(buf);
      CommandData.StarCam[1].expInt = ivalues[0];
      break;
    case thebad_exp_params:
      sprintf(buf, "CsetExpTime=%d", ivalues[0]);
      CommandData.StarCam[1].expTime = ivalues[0];
      sendTheBadCommand(buf);
      break;
    case thebad_focus_params:
      sprintf(buf, "CsetFocRsln=%d", ivalues[0]);
      sendTheBadCommand(buf);
      sprintf(buf, "CsetFocRnge=%d", ivalues[1]);
      sendTheBadCommand(buf);
      CommandData.StarCam[1].focusRes = ivalues[0];
      CommandData.StarCam[1].focusRng = ivalues[1];
      break;
    case thebad_bad_pix:
      sprintf(buf, "IsetBadpix=%d %d %d", ivalues[0], ivalues[1], ivalues[2]);
      sendTheBadCommand(buf);
      break;
    case thebad_blob_params:
      sprintf(buf, "IsetMaxBlobs=%d", ivalues[0]);
      sendTheBadCommand(buf);
      sprintf(buf, "IsetGrid=%d", ivalues[1]);
      sendTheBadCommand(buf);
      sprintf(buf, "IsetThreshold=%f", rvalues[2]);
      sendTheBadCommand(buf);
      sprintf(buf, "IsetDisttol=%d", ivalues[3]);
      sendTheBadCommand(buf);
      CommandData.StarCam[1].maxBlobs = ivalues[0];
      CommandData.StarCam[1].grid = ivalues[1];
      CommandData.StarCam[1].threshold = rvalues[2];
      CommandData.StarCam[1].minBlobDist = ivalues[3];
      break;
    case thebad_lens_any:
      sprintf(buf, "L=%s", svalues[0]);
      sendTheBadCommand(buf);
      break;
    case thebad_lens_move:
      sprintf(buf, "Lmove=%d", ivalues[0]);
      sendTheBadCommand(buf);
      break;
    case thebad_lens_params:
      sprintf(buf, "LsetTol=%d", ivalues[0]);
      sendTheBadCommand(buf);
      CommandData.StarCam[1].moveTol = ivalues[0];
      break;
      /***************************************/
      /********* The Ugly Commanding  *************/
    case theugly_any:
      sendTheUglyCommand(svalues[0]);
      break;
    case theugly_settrig_timed:
      sprintf(buf, "CsetExpInt=%d", ivalues[0]);
      sendTheUglyCommand(buf);
      CommandData.StarCam[2].expInt = ivalues[0];
      break;
    case theugly_exp_params:
      sprintf(buf, "CsetExpTime=%d", ivalues[0]);
      CommandData.StarCam[2].expTime = ivalues[0];
      sendTheUglyCommand(buf);
      break;
    case theugly_focus_params:
      sprintf(buf, "CsetFocRsln=%d", ivalues[0]);
      sendTheUglyCommand(buf);
      sprintf(buf, "CsetFocRnge=%d", ivalues[1]);
      sendTheUglyCommand(buf);
      CommandData.StarCam[2].focusRes = ivalues[0];
      CommandData.StarCam[2].focusRng = ivalues[1];
      break;
    case theugly_bad_pix:
      sprintf(buf, "IsetBadpix=%d %d %d", ivalues[0], ivalues[1], ivalues[2]);
      sendTheUglyCommand(buf);
      break;
    case theugly_blob_params:
      sprintf(buf, "IsetMaxBlobs=%d", ivalues[0]);
      sendTheUglyCommand(buf);
      sprintf(buf, "IsetGrid=%d", ivalues[1]);
      sendTheUglyCommand(buf);
      sprintf(buf, "IsetThreshold=%f", rvalues[2]);
      sendTheUglyCommand(buf);
      sprintf(buf, "IsetDisttol=%d", ivalues[3]);
      sendTheUglyCommand(buf);
      CommandData.StarCam[2].maxBlobs = ivalues[0];
      CommandData.StarCam[2].grid = ivalues[1];
      CommandData.StarCam[2].threshold = rvalues[2];
      CommandData.StarCam[2].minBlobDist = ivalues[3];
      break;
    case theugly_lens_any:
      sprintf(buf, "L=%s", svalues[0]);
      sendTheUglyCommand(buf);
      break;
    case theugly_lens_move:
      sprintf(buf, "Lmove=%d", ivalues[0]);
      sendTheUglyCommand(buf);
      break;
    case theugly_lens_params:
      sprintf(buf, "LsetTol=%d", ivalues[0]);
      sendTheUglyCommand(buf);
      CommandData.StarCam[2].moveTol = ivalues[0];
      break;
    case table_gain:  /* rotary table gains */
      // TODO PID loop performed in controller, figure out how to set gains
      // and write them to frame.
      CommandData.table.tableGain.P = ivalues[0];
      CommandData.table.tableGain.I = ivalues[1];
      CommandData.table.tableGain.D = ivalues[2];
      break;
    case table_goto:
      CommandData.table.mode = 1;
      CommandData.table.pos = rvalues[0];
      break;
    case table_drift:
      CommandData.table.mode = 2;
      CommandData.table.vel = rvalues[0];
      break;

    /*******************************************/
    /*************** Sync Box  *****************/
    case write_row_len:
      CommandData.sync_box.write_param = rl;
      CommandData.sync_box.param_value = ivalues[0];
      CommandData.sync_box.rl_value = CommandData.sync_box.param_value;
      CommandData.sync_box.cmd = 1;
      break;
    case write_num_rows:
      CommandData.sync_box.write_param = nr;
      CommandData.sync_box.param_value = ivalues[0];
      CommandData.sync_box.nr_value = CommandData.sync_box.param_value;
      CommandData.sync_box.cmd = 1;
      break;
    case write_free_run:
      CommandData.sync_box.write_param = fr;
      CommandData.sync_box.param_value = ivalues[0];
      CommandData.sync_box.fr_value = CommandData.sync_box.param_value;
      CommandData.sync_box.cmd = 1;
      break;

      /* MPC related things handled by PCM */
    case data_mode_bits:
      if (ivalues[0] >= 0 && ivalues[0] <= 12) { /* try to avoid crashing */
        CommandData.data_mode_bits[ivalues[0]][0][0] = (char)ivalues[1];
        CommandData.data_mode_bits[ivalues[0]][0][1] = (char)ivalues[2];
        CommandData.data_mode_bits[ivalues[0]][1][0] = (char)ivalues[3];
        CommandData.data_mode_bits[ivalues[0]][1][1] = (char)ivalues[4];
        CommandData.data_mode_bits_serial++;
      }
      break;

    default:
      if (!MCEcmd(command, rvalues, ivalues, svalues)) {
        bputs(warning, "Invalid Multi Word Command\n");
        return; /* invalid command - don't update */
      }
  }
  
  CommandData.command_count++;
  //set high bit to differentiate multi-commands from single
  CommandData.last_command = (unsigned short)command | 0x8000;
  
  int i_point = GETREADINDEX(point_index);

  if (!scheduled)
    CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
  else
    CommandData.pointing_mode.t = PointingData[i_point].t;

  WritePrevStatus();
}


void CheckCommandList(void)
{
  int i, c;

  /* the scommand enum isn't the same length as the scommand array */
  if ((int)xyzzy != N_SCOMMANDS - 1)
    bprintf(fatal, "N_SCOMMANDS should be %d\n", (int)xyzzy + 1);

  /* the scommand enum and scommand array are the same length, but there's a
   * problem in the scommand arrray initialiser */
  if (scommands[xyzzy].command != xyzzy) {
    c = -1;
    for (i = 0; i < N_SCOMMANDS; ++i)
      if (scommands[i].command == xyzzy) {
        c = i;
        break;
      }

    if (c == -1) {
      /* the initialiser is too long -- look for duplicate definitions */
      bprintf(fatal, "scommand #%i should be xyzzy, but it's \"%s\" "
          "(%i); xyzzy is not in the list at all.\n", (int)xyzzy,
          scommands[xyzzy].name, scommands[xyzzy].command);
    } else {
      /* the initialiser is too short -- look for missing definitions or extra
       * things in the enum */
      bprintf(fatal, "scommand #%i should be xyzzy, but it's \"%s\" "
          "(%i); xyzzy is scommand #%i\n", (int)xyzzy, scommands[xyzzy].name,
          scommands[xyzzy].command, c);
    }
  }

  /* the mcommand enum isn't the same length as the mcommand array */
  if ((int)plugh != N_MCOMMANDS - 1)
    bprintf(fatal, "N_MCOMMANDS should be %d\n", (int)plugh + 1);

  /* the mcommand enum and mcommand array are the same length, but there's a
   * problem in the mcommand arrray initialiser */
  if (mcommands[plugh].command != plugh) {
    c = -1;
    for (i = 0; i < N_MCOMMANDS; ++i)
      if (mcommands[i].command == plugh) {
        c = i;
        break;
      }

    if (c == -1) {
      /* the initialiser is too long -- look for duplicate definitions */
      bprintf(fatal, "mcommand #%i should be plugh, but it's \"%s\" "
          "(%i); plugh is not in the list at all.\n", (int)plugh,
          mcommands[plugh].name, mcommands[plugh].command);
    } else {
      /* the initialiser is too short -- look for missing definitions or extra
       * things in the enum */
      bprintf(fatal, "mcommand #%i should be plugh, but it's \"%s\" "
          "(%i); plugh is mcommand #%i\n", (int)plugh, mcommands[plugh].name,
          mcommands[plugh].command, c);
    }
  }

  //bprintf(info, "Commands: All Checks Passed.\n");
}

/* do necessary stuff after reading prev_status */
static void PostProcessInitCommand(void)
{
  change_bset(CommandData.bset_num & 0xFF, 1);
}

/************************************************************/
/*                                                          */
/*  Initialize CommandData: read last valid state: if there */
/*   is no previous state file, set to default              */
/*                                                          */
/************************************************************/
void InitCommandData()
{
  int fp, n_read = 0, junk, extra = 0, i;

  //run a basic check of the command lists before initializing
  CheckCommandList();

  if ((fp = open("/data/etc/spider/pcm.prev_status", O_RDONLY)) < 0) {
    berror(err, "Unable to open prev_status file for reading");
  } else {
    if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0)
      berror(err, "prev_status read()");
    if ((extra = read(fp, &junk, sizeof(junk))) < 0)
      berror(err, "extra prev_status read()");
    if (close(fp) < 0)
      berror(err, "prev_status close()");
  }

  /** this overrides prev_status **/
  CommandData.df = 0;
  CommandData.force_el = 0;

  CommandData.actbus.off = 0;
  CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
  CommandData.lock.goal = lock_do_nothing;
  CommandData.actbus.force_repoll = 0;

  CommandData.xystage.is_new = 0;
  CommandData.xystage.force_repoll = 0;

  CommandData.hwp.force_repoll = 0;
  CommandData.hwp.cindex = 0;
  CommandData.hwp.caddr[0] = -1;
  CommandData.hwp.caddr[1] = -1;
  CommandData.hwp.caddr[2] = -1;
  CommandData.hwp.mode = hwp_m_sleep;
  CommandData.hwp.bias_mask = 0x3f << 6;

  CommandData.table.vel = 0.0;
  CommandData.table.pos = 0.0;
  CommandData.table.mode = 0;
  CommandData.StarCam[0].paused = 1;
  CommandData.StarCam[1].paused = 1;
  CommandData.StarCam[2].paused = 1;

  CommandData.Temporary.setLevel[0] = 1;
  CommandData.Temporary.setLevel[1] = 1;
  CommandData.Temporary.setLevel[2] = 1;
  CommandData.Temporary.setLevel[3] = 1;
  CommandData.Temporary.setLevel[4] = 1;

  CommandData.power.sc_tx.rst_count = 0;
  CommandData.power.sc_tx.set_count = 0;
  CommandData.power.das.rst_count = 0;
  CommandData.power.das.set_count = 0;
  CommandData.power.rsc.rst_count = 0;
  CommandData.power.rsc.set_count = 0;
  CommandData.power.bsc.rst_count = 0;
  CommandData.power.bsc.set_count = 0;
  CommandData.power.gps.rst_count = 0;
  CommandData.power.gps.set_count = 0;
  CommandData.power.rw.rst_count = 0;
  CommandData.power.rw.set_count = 0;
  CommandData.power.piv.rst_count = 0;
  CommandData.power.piv.set_count = 0;
  CommandData.power.elmot.rst_count = 0;
  CommandData.power.elmot.set_count = 0;
  CommandData.power.bi0.rst_count = 0;
  CommandData.power.bi0.set_count = 0;
  CommandData.power.mcc1.set_count = 0;
  CommandData.power.mcc1.rst_count = 0;
  CommandData.power.mcc2.set_count = 0;
  CommandData.power.mcc2.rst_count = 0;
  CommandData.power.mcc3.set_count = 0;
  CommandData.power.mcc3.rst_count = 0;
  CommandData.power.mcc4.set_count = 0;
  CommandData.power.mcc4.rst_count = 0;
  CommandData.power.mcc5.set_count = 0;
  CommandData.power.mcc5.rst_count = 0;
  CommandData.power.mcc6.set_count = 0;
  CommandData.power.mcc6.rst_count = 0;
  CommandData.power.sync.rst_count = 0;
  CommandData.power.sync.set_count = 0;
  CommandData.power.gybox_off = 0;
  CommandData.power.gyro_off[0] = 0;
  CommandData.power.gyro_off[1] = 0;
  CommandData.power.gyro_off[2] = 0;
  CommandData.power.gyro_off[3] = 0;
  CommandData.power.gyro_off[4] = 0;
  CommandData.power.gyro_off[5] = 0;
  CommandData.power.hub232_off = 0;
  CommandData.power.lock_off = 0;
  CommandData.power.elmot_is_on = 0;
  for (i=0; i<16; i++)
    CommandData.power.adc_reset[i] = 0;

  CommandData.ifpower.mce_op[0] = nop;
  CommandData.ifpower.mce_op[1] = nop;
  CommandData.ifpower.mce_op[2] = nop;
  CommandData.ifpower.mce[0].rst_count = 0;
  CommandData.ifpower.mce[0].set_count = 0;
  CommandData.ifpower.mce[1].rst_count = 0;
  CommandData.ifpower.mce[1].set_count = 0;
  CommandData.ifpower.mce[2].rst_count = 0;
  CommandData.ifpower.mce[2].set_count = 0;
  CommandData.ifpower.hwp.rst_count = 0;
  CommandData.ifpower.hwp.set_count = 0;

  /* don't use the fast gy offset calculator */
  CommandData.fast_offset_gy = 0;

  CommandData.reset_rw = 0;
  CommandData.reset_piv = 0;
  CommandData.reset_elev = 0;
  CommandData.restore_piv = 0;

  CommandData.slot_sched = 0x100;
  CommandData.parts_sched=0x0;

  CommandData.mcecmd_index = 0;
  CommandData.mcecmd[0].t = -1;
  CommandData.mcecmd[1].t = -1;
  CommandData.mcecmd[2].t = -1;
  CommandData.mcecmd[0].done = 1;
  CommandData.mcecmd[1].done = 1;
  CommandData.mcecmd[2].done = 1;

  CommandData.sync_box.write_param = none;
  CommandData.sync_box.cmd = 0;
  CommandData.sync_box.param_value = 0;

  /** return if we succsesfully read the previous status **/
  if (n_read != sizeof(struct CommandDataStruct))
    bprintf(warning, "prev_status: Wanted %i bytes but got %i.\n",
        (int) sizeof(struct CommandDataStruct), n_read);
  else if (extra > 0)
    bputs(warning, "prev_status: Extra bytes found.\n");
  else {
    PostProcessInitCommand();
    return;
  }

  bputs(warning, "Regenerating Command Data and prev_status\n");

  /** prev_status overrides this stuff **/
  CommandData.command_count = 0;
  CommandData.last_command = 0xffff;
  
  CommandData.at_float = 0;
  CommandData.timeout = 3600;
  CommandData.slot_sched = 0;
  CommandData.pilot_bw = 50000;
  CommandData.tdrss_bw = 6000;
  CommandData.iridium_bw = 2000;
  CommandData.channelset_oth = 0;
  CommandData.vtx_sel[0] = vtx_isc;
  CommandData.vtx_sel[1] = vtx_osc;

  CommandData.slew_veto = VETO_MAX; /* 5 minutes */

  CommandData.pointing_mode.nw = 0;
  CommandData.pointing_mode.mode = P_DRIFT;
  CommandData.pointing_mode.el_mode = P_EL_NONE;
  CommandData.pointing_mode.X = 0;
  CommandData.pointing_mode.Y = 0;
  CommandData.pointing_mode.vaz = 0.0;
  CommandData.pointing_mode.del = 0.0;
  CommandData.pointing_mode.d_el_p = 0.0;
  CommandData.pointing_mode.d_el_s = 0.0;
  CommandData.pointing_mode.v_el_p = 0.0;
  CommandData.pointing_mode.v_el_s = 0.0;
  CommandData.pointing_mode.el_rel_move = 0;
  CommandData.pointing_mode.w = 0;
  CommandData.pointing_mode.h = 0;
  CommandData.pointing_mode.t = mcp_systime(NULL) + CommandData.timeout;
  CommandData.pointing_mode.Nscans = 1;
  CommandData.pointing_mode.Nsteps = 10;
  CommandData.pointing_mode.new_spider = 1;
  CommandData.pointing_mode.overshoot_band = 0.15;
  CommandData.pointing_mode.el_step = 0.0;
  CommandData.pointing_mode.is_turn_around = 0;
  CommandData.pointing_mode.az_delay = 2.75;
  CommandData.pointing_mode.piv_mode = 0;

  CommandData.az_accel = 0.1;
  CommandData.az_accel_max = 1.0;

  CommandData.ele_gain.com = 0;
  CommandData.ele_gain.manual_pulses = 0;
  CommandData.ele_gain.pulse_port = 0.0;
  CommandData.ele_gain.pulse_starboard = 0.0;

  CommandData.power.elmot_auto = 0;

  CommandData.azi_gain.P = 4000;
  CommandData.azi_gain.I = 100;
  CommandData.azi_gain.PT = 3000;

  CommandData.pivot_gain.SP = 0;   // dps
  CommandData.pivot_gain.V_RW = 0;
  CommandData.pivot_gain.P_RW = 0;
  CommandData.pivot_gain.V_AZ = 0;
  CommandData.pivot_gain.T_RW = 0;
  CommandData.pivot_gain.V_REQ = 0;

  CommandData.disable_az = 1;
  CommandData.disable_el = 0;

  CommandData.verbose_rw = 0;
  CommandData.verbose_piv = 0;

  CommandData.gyheat.setpoint = 15.0;
  CommandData.gyheat.gain.P = 30;
  CommandData.gyheat.gain.I = 10;
  CommandData.gyheat.gain.D = 3;

  CommandData.table.tableGain.P = 6652;  //thousandths
  CommandData.table.tableGain.I = 302;   //ten-thousandths
  CommandData.table.tableGain.D = 13520; //hundredths

  CommandData.use_elenc1 = 1;
  CommandData.use_elenc2 = 1;
  CommandData.use_pss = 1;
  CommandData.use_mag = 1;
  CommandData.use_gps = 0;
  CommandData.lat_range = 1;
  CommandData.sucks = 1;

  CommandData.null_az_trim = 0.0;
  CommandData.mag_az_trim = 0.0;
  CommandData.dgps_az_trim = 0.0;
  CommandData.pss_az_trim = 0.0;

  CommandData.dgps_cov_limit = 0.3;
  CommandData.dgps_ants_limit = 0.5;

  CommandData.cal_off_pss1 = 0.0;
  CommandData.cal_off_pss2 = 0.0;
  CommandData.cal_off_pss3 = 0.0;
  CommandData.cal_off_pss4 = 0.0;
  CommandData.cal_off_pss5 = 0.0;
  CommandData.cal_off_pss6 = 0.0;

  CommandData.cal_d_pss1 = 0.0;
  CommandData.cal_d_pss2 = 0.0;
  CommandData.cal_d_pss3 = 0.0;
  CommandData.cal_d_pss4 = 0.0;
  CommandData.cal_d_pss5 = 0.0;
  CommandData.cal_d_pss6 = 0.0;

  CommandData.cal_imin_pss = 4.5;

  SIPData.MKScal.m_hi = 0.01;
  SIPData.MKScal.m_med = 0.1;
  SIPData.MKScal.m_lo = 1;
  SIPData.MKScal.b_hi = 0;
  SIPData.MKScal.b_med = 0;
  SIPData.MKScal.b_lo = 0;

  CommandData.az_autogyro = 1;
  CommandData.el_autogyro = 1;
  CommandData.offset_ofpch_gy = 0;
  CommandData.offset_ofroll_gy = 0;
  CommandData.offset_ofyaw_gy = 0;
  CommandData.gymask = 0x3f;

  CommandData.Temporary.dac_out[0] = 0x8000;
  CommandData.Temporary.dac_out[1] = 0x8000;
  CommandData.Temporary.dac_out[2] = 0x8000;
  CommandData.Temporary.dac_out[3] = 0x8000;
  CommandData.Temporary.dac_out[4] = 0x8000;

  for (i=0; i<6; i++) {
    CommandData.hk[i].pump_heat = 0;
    CommandData.hk[i].heat_switch = 1;   //heat switch defaults to on
    CommandData.hk[i].fphi_heat = 0;
    CommandData.hk[i].ssa_heat = 0;
    CommandData.hk[i].htr1_heat = 0;
    CommandData.hk[i].htr2_heat = 0;
    CommandData.hk[i].htr3_heat = 0;
    CommandData.hk[i].fplo_heat = 0.0;
    CommandData.hk[i].ring_heat = 0.0;

    CommandData.hk[i].auto_cycle_on = 0;
    CommandData.hk[i].force_cycle = 0;
    CommandData.hk[i].pump_servo_on = 0;
    CommandData.hk[i].pump_servo_low = 20.0;
    CommandData.hk[i].pump_servo_high = 25.0;

    CommandData.hk[i].cernox.phase = 5.0;
    CommandData.hk[i].cernox.ampl = 0.3;
    CommandData.hk[i].ntd.phase = 5.0;
    CommandData.hk[i].ntd.ampl = 0.3;
    CommandData.hk[i].ntd.phase_start = 0;
    CommandData.hk[i].ntd.phase_end = 0;
    CommandData.hk[i].ntd.phase_step = 0;
    CommandData.hk[i].ntd.phase_dt = 0;
    CommandData.hk[i].ntd.phase_time = 0;
    CommandData.hk[i].cernox.phase_start = 0;
    CommandData.hk[i].cernox.phase_end = 0;
    CommandData.hk[i].cernox.phase_step = 0;
    CommandData.hk[i].cernox.phase_dt = 0;
    CommandData.hk[i].cernox.phase_time = 0;
  }
  for (i=0; i<8; i++) {
    CommandData.hk_theo_heat[i].state = 0;
    CommandData.hk_theo_heat[i].duration = 0;
    CommandData.hk_theo_heat[i].start_time = 0;
    CommandData.hk_theo_heat[i].duty_target = 0;
    CommandData.hk_theo_heat[i].duty_avg = 0;
  }
  CommandData.hk_last = 0;
  CommandData.hk_vheat_last = 0.0;
  CommandData.hk_bias_freq = 50;
  CommandData.ifpower.hk_preamp_off = 0;

  CommandData.actbus.tc_mode = TC_MODE_VETOED;
  CommandData.actbus.tc_step = 100; /* microns */
  CommandData.actbus.tc_wait = 3000; /* = 10 minutes in 5-Hz frames */
  CommandData.actbus.tc_spread = 5; /* centigrade degrees */
  CommandData.actbus.tc_prefp = 1;
  CommandData.actbus.tc_prefs = 1;

  CommandData.actbus.lvdt_delta = 1000;
  CommandData.actbus.lvdt_low = 25000;
  CommandData.actbus.lvdt_high = 35000;

  CommandData.actbus.offset[0] = 33333;
  CommandData.actbus.offset[1] = 33333;
  CommandData.actbus.offset[2] = 33333;

  /* The first is due to change in radius of curvature, the second due to
   * displacement of the secondary due to the rigid struts */

  /* Don sez:   50.23 + 9.9 and 13.85 - 2.2 */
  /* Marco sez: 56          and 10          */

  CommandData.actbus.g_primary = 56; /* um/deg */
  CommandData.actbus.g_secondary = 10; /* um/deg */
  CommandData.actbus.focus = 0;
  CommandData.actbus.sf_time = 0;
  CommandData.actbus.sf_offset = 0;

  CommandData.actbus.act_vel = 10;
  CommandData.actbus.act_acc = 1;
  CommandData.actbus.act_move_i = 85;
  CommandData.actbus.act_hold_i = 40;
  CommandData.actbus.act_tol = 2;

  CommandData.hwp.vel = 1.0;
  CommandData.hwp.move_i = 0.8;
  CommandData.hwp.phase = 0.0;

  CommandData.lock.pin_is_in = 1;
  CommandData.lock.state_p = lock_unknown;
  CommandData.lock.state_s = lock_unknown;

  CommandData.temp1 = 0;
  CommandData.temp2 = 0;
  CommandData.temp3 = 0;

  CommandData.lat = -77.86;  //McMurdo Building 096
  CommandData.lon = -167.04; //Willy Field Dec 2010

  //TODO make PCI card respect internal frame rate
  //TODO add commands to set frame rate
  //TODO add ability to auto-set internal rate based on sync rate
  CommandData.bbcIntFrameRate = 104;    //in ADC samples
  CommandData.bbcExtFrameRate = 1;    //in sync box frames
  CommandData.bbcExtFrameMeas = 0;
  CommandData.bbcIsExt = 0 /*1*/;
  CommandData.bbcAutoExt = 0 /*1*/;

  /* sync box parameter defaults*/
  CommandData.sync_box.rl_value = 57;
  CommandData.sync_box.nr_value = 33;
  CommandData.sync_box.fr_value = 120;

  /* some data modes are unsupported and so not initialised here */
  CommandData.data_mode_bits[ 0][0][0] = 16;
  CommandData.data_mode_bits[ 0][0][1] = 16;
  CommandData.data_mode_bits[ 1][0][0] = 16;
  CommandData.data_mode_bits[ 1][0][1] = 16;
  CommandData.data_mode_bits[ 2][0][0] = 16;
  CommandData.data_mode_bits[ 2][0][1] = 16;
  CommandData.data_mode_bits[ 4][0][0] = 14;
  CommandData.data_mode_bits[ 4][0][1] =  9;
  CommandData.data_mode_bits[ 4][1][0] =  0;
  CommandData.data_mode_bits[ 4][1][1] =  7;
  CommandData.data_mode_bits[ 5][0][0] =  8;
  CommandData.data_mode_bits[ 5][0][1] = 12;
  CommandData.data_mode_bits[ 5][1][0] =  0;
  CommandData.data_mode_bits[ 5][1][1] =  4;
  CommandData.data_mode_bits[10][0][0] =  7;
  CommandData.data_mode_bits[10][0][1] = 12;
  CommandData.data_mode_bits[10][1][0] =  0;
  CommandData.data_mode_bits[10][1][1] =  4;
  CommandData.data_mode_bits[12][0][0] = 16;
  CommandData.data_mode_bits[12][0][1] = 16;
  CommandData.data_mode_bits_serial = 0;

  CommandData.mcc_wdog = 0;

  CommandData.questionable_behaviour = 0;

  WritePrevStatus();
  PostProcessInitCommand();
}
