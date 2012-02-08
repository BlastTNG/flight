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

/* Define this symbol to have mcp log all SIP traffic */
#undef SIP_CHATTER
#undef VERBOSE_SIP_CHATTER

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
#include "share/channels.h"
#include "share/sip.h"

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
#define LOCK_OFFSET (-1.4) /* Updated by LMF on December 9th, 2010 */

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

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

const char* SName(enum singleCommand command); // share/sip.c

/* in sc.cpp: */
int sendTheGoodCommand(const char *cmd);
int sendTheBadCommand(const char *cmd);
int sendTheUglyCommand(const char *cmd);

/* calculate the nearest lockable elevation */
double LockPosition (double elevation)
{
  double position;

  position = floor(elevation / 10.0) * 10.0 + 5.0;
  if (position > 79.0) {
    position = 90.0;
  } else if (position < 10.0) {
    position = 5.0;
  }

  return position + LOCK_OFFSET;
}

void SingleCommand (enum singleCommand command, int scheduled)
{
#ifndef BOLOTEST
  int i_point = GETREADINDEX(point_index);
  double sun_az;
#endif

  if (!scheduled)
    bprintf(info, "Commands: Single command: %d (%s)\n", command,
        SName(command));

  /* Update CommandData structure with new info */

  switch (command) {
#ifndef BOLOTEST
    case stop: /* Pointing abort */
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case antisun: /* turn antisolar (az-only) */
      sun_az = PointingData[i_point].sun_az + 250; /* point solar panels to sun */
      NormalizeAngle(&sun_az);

      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_AZEL_GOTO;
      CommandData.pointing_mode.X = sun_az;  /* az */
      CommandData.pointing_mode.Y = PointingData[i_point].el;  /* el */
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
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

    case az_off:/* disable az motors */
      CommandData.disable_az = 1;
      break;
    case az_on:/* enable az motors */
      CommandData.disable_az = 0;
      break;
    case el_off: /* disable el motors */
      CommandData.disable_el = 1;
      CommandData.force_el = 0;
      break;
    case el_on: /* enable el motors */
      CommandData.disable_el = 0;
      CommandData.force_el = 0;
      break;
    case force_el_on: /* force enabling of el motors */
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
    case elclin_veto:
      CommandData.use_elclin = 0;
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
    case elclin_allow:
      CommandData.use_elclin = 1;
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
      CommandData.power.elmot.set_count = 0;
      CommandData.power.elmot.rst_count = LATCH_PULSE_LEN;
      break;
    case elmot_on:
      CommandData.power.elmot.rst_count = 0;
      CommandData.power.elmot.set_count = LATCH_PULSE_LEN;
      break;
    case elmot_cycle:
      CommandData.power.elmot.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.elmot.rst_count = LATCH_PULSE_LEN;
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
    case rx_off:
      CommandData.power.rx_main.set_count = 0;
      CommandData.power.rx_main.rst_count = LATCH_PULSE_LEN;
      break;
    case rx_on:
      CommandData.power.rx_main.rst_count = 0;
      CommandData.power.rx_main.set_count = LATCH_PULSE_LEN;
      break;
    case rx_hk_off:
      CommandData.power.rx_hk.set_count = 0;
      CommandData.power.rx_hk.rst_count = LATCH_PULSE_LEN;
      break;
    case rx_hk_on:
      CommandData.power.rx_hk.rst_count = 0;
      CommandData.power.rx_hk.set_count = LATCH_PULSE_LEN;
      break;
    case rx_amps_off:
      CommandData.power.rx_amps.set_count = 0;
      CommandData.power.rx_amps.rst_count = LATCH_PULSE_LEN;
      break;
    case rx_amps_on:
      CommandData.power.rx_amps.rst_count = 0;
      CommandData.power.rx_amps.set_count = LATCH_PULSE_LEN;
      break;
    case charge_on:
      CommandData.power.charge.set_count = 0;
      CommandData.power.charge.rst_count = LATCH_PULSE_LEN;
      break;
    case charge_off:
      CommandData.power.charge.rst_count = 0;
      CommandData.power.charge.set_count = LATCH_PULSE_LEN;
      break;
    case charge_cycle:
      CommandData.power.charge.rst_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.charge.set_count = LATCH_PULSE_LEN;
      break;
    case ifroll_1_gy_allow:
      CommandData.gymask |= 0x01;
      break;
    case ifroll_1_gy_veto:
      CommandData.gymask &= ~0x01;
      break;
    case ifroll_2_gy_allow:
      CommandData.gymask |= 0x02;
      break;
    case ifroll_2_gy_veto:
      CommandData.gymask &= ~0x02;
      break;
    case ifyaw_1_gy_allow:
      CommandData.gymask |= 0x04;
      break;
    case ifyaw_1_gy_veto:
      CommandData.gymask &= ~0x04;
      break;
    case ifyaw_2_gy_allow:
      CommandData.gymask |= 0x08;
      break;
    case ifyaw_2_gy_veto:
      CommandData.gymask &= ~0x08;
      break;
    case ifel_1_gy_allow:
      CommandData.gymask |= 0x10;
      break;
    case ifel_1_gy_veto:
      CommandData.gymask &= ~0x10;
      break;
    case ifel_2_gy_allow:
      CommandData.gymask |= 0x20;
      break;
    case ifel_2_gy_veto:
      CommandData.gymask &= ~0x20;
      break;
    case ifroll_1_gy_off:
      CommandData.power.gyro_off[1] = -1;
      break;
    case ifroll_1_gy_on:
      CommandData.power.gyro_off[1] = 0;
      break;
    case ifroll_1_gy_cycle:
      CommandData.power.gyro_off[1] = PCYCLE_HOLD_LEN;
      break;
    case ifroll_2_gy_off:
      CommandData.power.gyro_off[5] = -1;
      break;
    case ifroll_2_gy_on:
      CommandData.power.gyro_off[5] = 0;
      break;
    case ifroll_2_gy_cycle:
      CommandData.power.gyro_off[5] = PCYCLE_HOLD_LEN;
      break;
    case ifyaw_1_gy_off:
      CommandData.power.gyro_off[0] = -1;
      break;
    case ifyaw_1_gy_on:
      CommandData.power.gyro_off[0] = 0;
      break;
    case ifyaw_1_gy_cycle:
      CommandData.power.gyro_off[0] = PCYCLE_HOLD_LEN;
      break;
    case ifyaw_2_gy_off:
      CommandData.power.gyro_off[2] = -1;
      break;
    case ifyaw_2_gy_on:
      CommandData.power.gyro_off[2] = 0;
      break;
    case ifyaw_2_gy_cycle:
      CommandData.power.gyro_off[2] = PCYCLE_HOLD_LEN;
      break;
    case ifel_1_gy_off:
      CommandData.power.gyro_off[3] = -1;
      break;
    case ifel_1_gy_on:
      CommandData.power.gyro_off[3] = 0;
      break;
    case ifel_1_gy_cycle:
      CommandData.power.gyro_off[3] = PCYCLE_HOLD_LEN;
      break;
    case ifel_2_gy_off:
      CommandData.power.gyro_off[4] = -1;
      break;
    case ifel_2_gy_on:
      CommandData.power.gyro_off[4] = 0;
      break;
    case ifel_2_gy_cycle:
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
    case thegood_off:
      CommandData.power.thegood_cam_off = -1;
      CommandData.power.thegood_cpu_off = -1;
      break;
    case thegood_on:
      CommandData.power.thegood_cam_off = 0;
      CommandData.power.thegood_cpu_off = 0;
      break;
    case thegood_cam_cycle:
      CommandData.power.thegood_cam_off = PCYCLE_HOLD_LEN;
      break;
    case thegood_cpu_cycle:
      CommandData.power.thegood_cpu_off = PCYCLE_HOLD_LEN;
      break;
    case thebad_off:
      CommandData.power.thebad_cam_off = -1;
      CommandData.power.thebad_cpu_off = -1;
      break;
    case thebad_on:
      CommandData.power.thebad_cam_off = 0;
      CommandData.power.thebad_cpu_off = 0;
      break;
    case thebad_cam_cycle:
      CommandData.power.thebad_cam_off = PCYCLE_HOLD_LEN;
      break;
    case thebad_cpu_cycle:
      CommandData.power.thebad_cpu_off = PCYCLE_HOLD_LEN;
      break;
    case theugly_off:
      CommandData.power.theugly_cam_off = -1;
      CommandData.power.theugly_cpu_off = -1;
      break;
    case theugly_on:
      CommandData.power.theugly_cam_off = 0;
      CommandData.power.theugly_cpu_off = 0;
      break;
    case theugly_cam_cycle:
      CommandData.power.theugly_cam_off = PCYCLE_HOLD_LEN;
      break;
    case theugly_cpu_cycle:
      CommandData.power.theugly_cpu_off = PCYCLE_HOLD_LEN;
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
      CommandData.hwpr.force_repoll = 1;
      break;
    case actbus_cycle:
      CommandData.actbus.off = PCYCLE_HOLD_LEN;
      CommandData.actbus.force_repoll = 1;
      CommandData.hwpr.force_repoll = 1;
      break;

#endif

    /* Lock */
    case pin_in:
      CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF | LS_IGNORE_EL;
      break;
    case lock_off:
      CommandData.actbus.lock_goal = LS_DRIVE_OFF | LS_DRIVE_FORCE;
      break;
    case unlock:
      CommandData.actbus.lock_goal = LS_OPEN | LS_DRIVE_OFF;
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
    case repoll:
      CommandData.actbus.force_repoll = 1;
      CommandData.hwpr.force_repoll = 1;
      CommandData.xystage.force_repoll = 1;
      break;

    /* Actuators */
    case actuator_stop:
      CommandData.actbus.focus_mode = ACTBUS_FM_PANIC;
      CommandData.actbus.tc_mode = TC_MODE_VETOED;
      break;

    case hwpr_panic:
      CommandData.hwpr.mode = HWPR_PANIC;
      CommandData.hwpr.is_new = 1;
      break;

#ifndef BOLOTEST
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
      CommandData.thegood.expInt = 0;
      break;
    case thegood_pause:
      CommandData.thegood.paused = 1;
      break;
    case thegood_run:
      CommandData.thegood.paused = 0;
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
      CommandData.thebad.expInt = 0;
      break;
    case thebad_pause:
      CommandData.thebad.paused = 1;
      break;
    case thebad_run:
      CommandData.thebad.paused = 0;
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
      CommandData.theugly.expInt = 0;
      break;
    case theugly_pause:
      CommandData.theugly.paused = 1;
      break;
    case theugly_run:
      CommandData.theugly.paused = 0;
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
#endif
    case hwpr_step:
      CommandData.hwpr.mode = HWPR_STEP;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_step_off:
      CommandData.hwpr.no_step = 1;
      break;
    case hwpr_step_on:
      CommandData.hwpr.no_step = 0;
      break;
    case hwpr_pot_is_dead:
      CommandData.hwpr.use_pot = 0;
      break;
    case hwpr_pot_is_alive:
      CommandData.hwpr.use_pot = 1;
      break;

    case reap_itsy:  /* Miscellaneous commands */
    case reap_bitsy:
      if ((command == reap_itsy && !BitsyIAm) || 
	  (command == reap_bitsy && BitsyIAm)) {
	bprintf(err, "Commands: Reaping the watchdog tickle on command.");
	pthread_cancel(watchdog_id);
      }
      break;
    case halt_itsy:
    case halt_bitsy:
      if ((command == halt_itsy && !BitsyIAm) || 
	  (command == halt_bitsy && BitsyIAm)) {
        bputs(warning, "Commands: Halting the MCC\n");
        if (system("/sbin/reboot") < 0)
	  berror(fatal, "Commands: failed to reboot, dying\n");
      }
      break;
    case xy_panic:
      CommandData.xystage.mode = XYSTAGE_PANIC;
      CommandData.xystage.is_new = 1;

    //Theo heater housekeeping commands. TODO-theo: temporarily uses insert 6
    case hk_t0_heat_on:
      CommandData.hk[5].pump_heat = 1;
      break;
    case hk_t0_heat_off:
      CommandData.hk[5].pump_heat = 0;
      break;
    //NB: because heat_switch on insert 6 is NC, logic is inverted
    case hk_t1_heat_on:
      CommandData.hk[5].heat_switch = 0;
      break;
    case hk_t1_heat_off:
      CommandData.hk[5].heat_switch = 1;
      break;
    case hk_t2_heat_on:
      CommandData.hk[5].tile_heat[2] = -1;
      break;
    case hk_t2_heat_off:
      CommandData.hk[5].tile_heat[2] = 0;
      break;
    case hk_t3_heat_on:
      CommandData.hk[5].tile_heat[1] = -1;
      break;
    case hk_t3_heat_off:
      CommandData.hk[5].tile_heat[1] = 0;
      break;
    case hk_t4_heat_on:
      CommandData.hk[5].tile_heat[0] = -1;
      break;
    case hk_t4_heat_off:
      CommandData.hk[5].tile_heat[0] = 0;
      break;
    case hk_t5_heat_on:
      CommandData.hk[5].fphi_heat = 1;
      break;
    case hk_t5_heat_off:
      CommandData.hk[5].fphi_heat = 0;
      break;
    case hk_t6_heat_on:
      CommandData.hk[5].tile_heat[3] = -1;
      break;
    case hk_t6_heat_off:
      CommandData.hk[5].tile_heat[3] = 0;
      break;


    case xyzzy:
      break;
    default:
      bputs(warning, "Commands: ***Invalid Single Word Command***\n");
      return; /* invalid command - no write or update */
  }

#ifndef BOLOTEST
  if (!scheduled) {
    if (doing_schedule)
      bprintf(info, "Scheduler: *** Out of schedule file mode ***");
    CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
  } else
    CommandData.pointing_mode.t = PointingData[i_point].t;
#endif

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
  int i, j;
  char buf[256]; //for SC Commands
  int is_new;

  /* Update CommandData struct with new info
   * If the parameter is type 'i'/'l' set CommandData using ivalues[i]
   * If the parameter is type 'f'/'d' set CommandData using rvalues[i]
   */

  /* Pointing Modes */
  switch(command) {
#ifndef BOLOTEST
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

      CommandData.pointing_mode.mode = P_AZEL_GOTO;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case az_scan:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_AZ_SCAN;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      CommandData.pointing_mode.w = rvalues[2];  /* width */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.h = 0;
      break;
    case drift:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_DRIFT;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = 0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = rvalues[0]; /* az speed */
      CommandData.pointing_mode.del = rvalues[1]; /* el speed */
      CommandData.pointing_mode.h = 0;
      break;
    case ra_dec_goto:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_RADEC_GOTO;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case vcap:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_VCAP;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* radius */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[4]; /* el drift speed */
      CommandData.pointing_mode.h = 0;
      break;
    case cap:
      
      if ((CommandData.pointing_mode.mode != P_CAP) ||
          (CommandData.pointing_mode.X != rvalues[0]) ||  /* ra */
          (CommandData.pointing_mode.Y != rvalues[1]) ||  /* dec */
          (CommandData.pointing_mode.w != rvalues[2]) ||  /* radius */
          (CommandData.pointing_mode.vaz != rvalues[3]) ||  /* az scan speed */
          (CommandData.pointing_mode.del != rvalues[4]) ||  /* el step size */
          (CommandData.pointing_mode.h != 0) || 
          (CommandData.pointing_mode.dith != rvalues[5])) { /* el step size */
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }


      CommandData.pointing_mode.mode = P_CAP;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* radius */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[4]; /* el step size */
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.dith = rvalues[5]; /* el step size */
      break;
    case box:

      if ((CommandData.pointing_mode.mode != P_BOX) ||
          (CommandData.pointing_mode.X != rvalues[0]) || /* ra */
          (CommandData.pointing_mode.Y != rvalues[1]) || /* dec */
          (CommandData.pointing_mode.w != rvalues[2]) || /* width */
          (CommandData.pointing_mode.h != rvalues[3]) || /* height */
          (CommandData.pointing_mode.vaz != rvalues[4]) || /* az scan speed */
          (CommandData.pointing_mode.del != rvalues[5]) || /* el step size */
          (CommandData.pointing_mode.dith != rvalues[6])) { /* el step size */
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
     
      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }
      
      CommandData.pointing_mode.mode = P_BOX;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* width */
      CommandData.pointing_mode.h = rvalues[3]; /* height */
      CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[5]; /* el step size */
      CommandData.pointing_mode.dith = rvalues[6]; /* el step size */
      break;
    case vbox:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_VBOX;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* width */
      CommandData.pointing_mode.h = rvalues[3]; /* height */
      CommandData.pointing_mode.vaz = rvalues[4]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[5]; /* el drift speed */
      break;
    case quad:
      is_new = 0;
      if ((CommandData.pointing_mode.mode != P_QUAD) ||
          (CommandData.pointing_mode.vaz != rvalues[8]) || /* az scan speed */
          (CommandData.pointing_mode.del != rvalues[9]) || /* el step size */
          (CommandData.pointing_mode.dith != rvalues[10])) { /* el dith size */
        is_new=1;
      }
      for (i = 0; i < 4; i++) {
        if ((CommandData.pointing_mode.ra[i] != rvalues[i * 2]) ||
            (CommandData.pointing_mode.dec[i] != rvalues[i * 2 + 1])) {
          is_new = 1;
        }
      }
      
      if (is_new) {
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
      CommandData.pointing_mode.X = 0; /* ra */
      CommandData.pointing_mode.Y = 0; /* dec */
      CommandData.pointing_mode.w = 0; /* width */
      CommandData.pointing_mode.h = 0; /* height */
      
      CommandData.pointing_mode.mode = P_QUAD;
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = rvalues[i * 2];
        CommandData.pointing_mode.dec[i] = rvalues[i * 2 + 1];
      }
      CommandData.pointing_mode.vaz = rvalues[8]; /* az scan speed */
      CommandData.pointing_mode.del = rvalues[9]; /* el step size */
      CommandData.pointing_mode.dith = rvalues[10]; /* el dith size */
      break;
    case spider_scan:
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = rvalues[i*2];
        CommandData.pointing_mode.dec[i] = rvalues[i*2 + 1];        
      }
      CommandData.az_accel = rvalues[8];
      CommandData.pointing_mode.Y = rvalues[9];
      CommandData.pointing_mode.mode = P_SPIDER;
      break;
    case sine_scan:
      CommandData.az_accel = rvalues[0];
      CommandData.pointing_mode.w = 2.0*rvalues[1];
      CommandData.pointing_mode.X = rvalues[2];
      CommandData.pointing_mode.Y = rvalues[3];
      CommandData.pointing_mode.mode = P_SINE;
      break;
    case set_az_accel:
      CommandData.az_accel = rvalues[0];
      CommandData.az_accel_max = rvalues[1];
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
      CommandData.offset_ifroll_gy = rvalues[0];
      CommandData.offset_ifyaw_gy = rvalues[1];
      CommandData.az_autogyro = 0;
      break;
    case el_gyro_offset:
      CommandData.offset_ifel_gy = rvalues[0];
      CommandData.el_autogyro = 0;
      break;
    case slew_veto:
      CommandData.slew_veto = rvalues[0] * SR;
            bprintf(info,"CommandData.slew_veto = %i, CommandData.pointing_mode.nw = %i", CommandData.slew_veto, CommandData.pointing_mode.nw);
      if (CommandData.pointing_mode.nw > CommandData.slew_veto) CommandData.pointing_mode.nw = CommandData.slew_veto; 
     
      break;

      /***************************************/
      /********** Pointing Motor Gains *******/
    case el_gain:  /* ele gains */
      //CommandData.ele_gain.P = ivalues[0];
      CommandData.ele_gain.com = rvalues[0];
      //CommandData.ele_gain.I = ivalues[1];
      CommandData.ele_gain.diff = rvalues[1];
      //CommandData.ele_gain.PT = ivalues[2];
      CommandData.ele_gain.accel_max = rvalues[2];
      break;
    case az_gain:  /* az gains */
      CommandData.azi_gain.P = ivalues[0];
      CommandData.azi_gain.I = ivalues[1];
      CommandData.azi_gain.PT = ivalues[2];
      break;
    case pivot_gain:  /* pivot gains */
      CommandData.pivot_gain.SP = rvalues[0];
      CommandData.pivot_gain.PE = ivalues[1];
      CommandData.pivot_gain.PV = ivalues[2];
      CommandData.pivot_gain.F = rvalues[3];
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
      CommandData.verbose_el = ivalues[1];
      CommandData.verbose_piv = ivalues[2];
      break;
#endif

      /***************************************/
      /********** Lock / Actuators  **********/
    case lock:  /* Lock Inner Frame */
      if (CommandData.pointing_mode.nw >= 0)
        CommandData.pointing_mode.nw = VETO_MAX;
      CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_LOCK;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = LockPosition(rvalues[0]);
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      bprintf(info, "Commands: Lock Mode: %g\n", CommandData.pointing_mode.Y);
      break;
    case lock_vel:
      CommandData.actbus.lock_vel = ivalues[0];
      CommandData.actbus.lock_acc = ivalues[1];
      break;
    case lock_i:
      CommandData.actbus.lock_move_i = ivalues[0];
      CommandData.actbus.lock_hold_i = ivalues[1];
      break;
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

    case hwpr_vel:
      CommandData.hwpr.vel = ivalues[0];
      CommandData.hwpr.acc = ivalues[1];
      break;
    case hwpr_i:
      CommandData.hwpr.move_i = ivalues[0];
      CommandData.hwpr.hold_i = ivalues[1];
      break;
    case hwpr_goto:
      CommandData.hwpr.target = ivalues[0];
      CommandData.hwpr.mode = HWPR_GOTO;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_jump:
      CommandData.hwpr.target = ivalues[0];
      CommandData.hwpr.mode = HWPR_JUMP;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_repeat:
      CommandData.hwpr.n_pos = ivalues[0];
      CommandData.hwpr.repeats = ivalues[1];
      CommandData.hwpr.step_wait = ivalues[2]*5;
      CommandData.hwpr.step_size = ivalues[3];
      CommandData.hwpr.mode = HWPR_REPEAT;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_define_pos:
      CommandData.hwpr.pos[0] = rvalues[0];
      CommandData.hwpr.pos[1] = rvalues[1];
      CommandData.hwpr.pos[2] = rvalues[2];
      CommandData.hwpr.pos[3] = rvalues[3];
      break;
    case hwpr_goto_pot:
      CommandData.hwpr.pot_targ = rvalues[0];
      CommandData.hwpr.mode = HWPR_GOTO_POT;
      CommandData.hwpr.is_new = 1;
      break;
    case hwpr_set_overshoot:
      CommandData.hwpr.overshoot = ivalues[0];
      break;
    case hwpr_goto_i:
      CommandData.hwpr.i_pos = ivalues[0];
      CommandData.hwpr.mode = HWPR_GOTO_I;
      CommandData.hwpr.is_new = 1;
      break;
      /* XY Stage */
    case xy_goto:
      CommandData.xystage.x1 = ivalues[0];
      CommandData.xystage.y1 = ivalues[1];
      CommandData.xystage.xvel = ivalues[2];
      CommandData.xystage.yvel = ivalues[3];
      CommandData.xystage.mode = XYSTAGE_GOTO;
      CommandData.xystage.is_new = 1;
      break;
    case xy_jump:
      CommandData.xystage.x1 = ivalues[0];
      CommandData.xystage.y1 = ivalues[1];
      CommandData.xystage.xvel = ivalues[2];
      CommandData.xystage.yvel = ivalues[3];
      CommandData.xystage.mode = XYSTAGE_JUMP;
      CommandData.xystage.is_new = 1;
      break;
    case xy_xscan:
      CommandData.xystage.x1 = ivalues[0];
      CommandData.xystage.x2 = ivalues[1];
      CommandData.xystage.xvel = ivalues[2];
      CommandData.xystage.yvel = 0;
      CommandData.xystage.mode = XYSTAGE_SCAN;
      CommandData.xystage.is_new = 1;
      break;
    case xy_yscan:
      CommandData.xystage.y1 = ivalues[0];
      CommandData.xystage.y2 = ivalues[1];
      CommandData.xystage.yvel = ivalues[2];
      CommandData.xystage.xvel = 0;
      CommandData.xystage.mode = XYSTAGE_SCAN;
      CommandData.xystage.is_new = 1;
      break;
    case xy_raster:
      CommandData.xystage.x1 = ivalues[0];
      CommandData.xystage.x2 = ivalues[1];
      CommandData.xystage.y1 = ivalues[2];
      CommandData.xystage.y2 = ivalues[3];
      CommandData.xystage.xvel = ivalues[4];
      CommandData.xystage.yvel = ivalues[5];
      CommandData.xystage.step = ivalues[6];
      CommandData.xystage.mode = XYSTAGE_RASTER;
      CommandData.xystage.is_new = 1;
      break;

#ifndef BOLOTEST
      /***************************************/
      /******** Electronics Heaters  *********/
    case t_gyro_set:  /* gyro heater setpoint */
      CommandData.gyheat.setpoint = rvalues[0];
      CommandData.gyheat.age = 0;
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
    case tdrss_bw:
      CommandData.tdrss_bw = rvalues[0];
      break;      
    case iridium_bw:
      CommandData.iridium_bw = rvalues[0];
      break;
    case slot_sched: /* change uplinked schedule file */
      if (LoadUplinkFile(ivalues[0])) {
        CommandData.uplink_sched = 1;
        CommandData.slot_sched = ivalues[0];
      }
      break;
    case plugh:/* A hollow voice says "Plugh". */
      CommandData.plover = ivalues[0];
      break;
#endif

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
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ntd.phase = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ntd.phase = rvalues[1];
      break;
    case hk_phase_cernox:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].cernox.phase= rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].cernox.phase = rvalues[1];
      break;
    case hk_bias_freq:
      //TODO consider scaling phases as fixed time delay, when freq changes
      CommandData.hk_bias_freq = ivalues[0];
      break;

      /***************************************/
      /*************** Heat  *****************/
    case hk_pump_heat_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].pump_heat = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].pump_heat = 1;
      break;
    case hk_pump_heat_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].pump_heat = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].pump_heat = 0;
      break;
    case hk_heat_switch_on:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].heat_switch = 1;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].heat_switch = 1;
      break;
    case hk_heat_switch_off:
      CommandData.hk_last = ivalues[0];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].heat_switch = 0;
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].heat_switch = 0;
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
    case hk_tile_heat_on:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_tile_last = ivalues[1];
      for (i = ((ivalues[0] > 0) ? ivalues[0] - 1 : 0);
	   i < ((ivalues[0] > 0) ? ivalues[0]     : HK_MAX); i++) {
	for (j = ((ivalues[1] > 0) ? ivalues[1] - 1 : 0);
	     j < ((ivalues[1] > 0) ? ivalues[1]     : 4); j++) {
	  CommandData.hk[i].tile_heat[j] = -1;
	}
      }
      break;
    case hk_tile_heat_off:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_tile_last = ivalues[1];
      for (i = ((ivalues[0] > 0) ? ivalues[0] - 1 : 0);
	   i < ((ivalues[0] > 0) ? ivalues[0]     : HK_MAX); i++) {
	for (j = ((ivalues[1] > 0) ? ivalues[1] - 1 : 0);
	     j < ((ivalues[1] > 0) ? ivalues[1]     : 4); j++) {
	  CommandData.hk[i].tile_heat[j] = 0;
	}
      }
      break;
    case hk_tile_heat_pulse:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_tile_last = ivalues[1];
      CommandData.hk_pulse_last = ivalues[2];
      for (i = ((ivalues[0] > 0) ? ivalues[0] - 1 : 0);
	   i < ((ivalues[0] > 0) ? ivalues[0]     : HK_MAX); i++) {
	for (j = ((ivalues[1] > 0) ? ivalues[1] - 1 : 0);
	     j < ((ivalues[1] > 0) ? ivalues[1]     : 4); j++) {
	  CommandData.hk[i].tile_heat[j] = ivalues[2];
	}
      }
      break;
    case hk_fplo_heat_set:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_vheat_last = rvalues[1];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].fplo_heat = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].fplo_heat = rvalues[1];
      break;
    case hk_ssa_heat_set:
      CommandData.hk_last = ivalues[0];
      CommandData.hk_vheat_last = rvalues[1];
      if (ivalues[0] > 0) CommandData.hk[ivalues[0]-1].ssa_heat = rvalues[1];
      else for (i=0; i<HK_MAX; i++) CommandData.hk[i].ssa_heat = rvalues[1];
      break;


#ifndef BOLOTEST
      /***************************************/
      /********* The Good Commanding  *************/ 
    case thegood_any:
      sendTheGoodCommand(svalues[0]);
      break;
    case thegood_settrig_timed:
      sprintf(buf, "CsetExpInt=%d", ivalues[0]);
      sendTheGoodCommand(buf);
      CommandData.thegood.expInt = ivalues[0];
      break;
    case thegood_exp_params:
      sprintf(buf, "CsetExpTime=%d", ivalues[0]);
      CommandData.thegood.expTime = ivalues[0];
      sendTheGoodCommand(buf);
      break;
    case thegood_focus_params:
      sprintf(buf, "CsetFocRsln=%d", ivalues[0]);
      sendTheGoodCommand(buf);
      sprintf(buf, "CsetFocRnge=%d", ivalues[1]);
      sendTheGoodCommand(buf); 
      CommandData.thegood.focusRes = ivalues[0];
      CommandData.thegood.focusRng = ivalues[1];
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
      CommandData.thegood.maxBlobs = ivalues[0];
      CommandData.thegood.grid = ivalues[1];
      CommandData.thegood.threshold = rvalues[2];
      CommandData.thegood.minBlobDist = ivalues[3];
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
      CommandData.thegood.moveTol = ivalues[0];
      break;
      /***************************************/
      /********* The Bad Commanding  *************/ 
    case thebad_any:
      sendTheBadCommand(svalues[0]);
      break;
    case thebad_settrig_timed:
      sprintf(buf, "CsetExpInt=%d", ivalues[0]);
      sendTheBadCommand(buf);
      CommandData.thebad.expInt = ivalues[0];
      break;
    case thebad_exp_params:
      sprintf(buf, "CsetExpTime=%d", ivalues[0]);
      CommandData.thebad.expTime = ivalues[0];
      sendTheBadCommand(buf);
      break;
    case thebad_focus_params:
      sprintf(buf, "CsetFocRsln=%d", ivalues[0]);
      sendTheBadCommand(buf);
      sprintf(buf, "CsetFocRnge=%d", ivalues[1]);
      sendTheBadCommand(buf); 
      CommandData.thebad.focusRes = ivalues[0];
      CommandData.thebad.focusRng = ivalues[1];
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
      CommandData.thebad.maxBlobs = ivalues[0];
      CommandData.thebad.grid = ivalues[1];
      CommandData.thebad.threshold = rvalues[2];
      CommandData.thebad.minBlobDist = ivalues[3];
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
      CommandData.thebad.moveTol = ivalues[0];
      break;
      /***************************************/
      /********* The Ugly Commanding  *************/ 
    case theugly_any:
      sendTheUglyCommand(svalues[0]);
      break;
    case theugly_settrig_timed:
      sprintf(buf, "CsetExpInt=%d", ivalues[0]);
      sendTheUglyCommand(buf);
      CommandData.theugly.expInt = ivalues[0];
      break;
    case theugly_exp_params:
      sprintf(buf, "CsetExpTime=%d", ivalues[0]);
      CommandData.theugly.expTime = ivalues[0];
      sendTheUglyCommand(buf);
      break;
    case theugly_focus_params:
      sprintf(buf, "CsetFocRsln=%d", ivalues[0]);
      sendTheUglyCommand(buf);
      sprintf(buf, "CsetFocRnge=%d", ivalues[1]);
      sendTheUglyCommand(buf); 
      CommandData.theugly.focusRes = ivalues[0];
      CommandData.theugly.focusRng = ivalues[1];
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
      CommandData.theugly.maxBlobs = ivalues[0];
      CommandData.theugly.grid = ivalues[1];
      CommandData.theugly.threshold = rvalues[2];
      CommandData.theugly.minBlobDist = ivalues[3];
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
      CommandData.theugly.moveTol = ivalues[0];
      break;
    case table_gain:  /* rotary table gains */
      //TODO PID loop performed in controller, figure out how to set gains
      CommandData.table.tableGain.P = ivalues[0];
      CommandData.table.tableGain.I = ivalues[1];
      CommandData.table.tableGain.D = ivalues[2];
      break;
    case table_goto:
      CommandData.table.mode = 1;
      CommandData.table.pos = rvalues[0];
      break;
    case table_relmove:
      CommandData.table.mode = 2;
      CommandData.table.move = rvalues[0];
      break;
    case table_speed:
      CommandData.table.vel = rvalues[0];
      break;

#endif
    default:
      bputs(warning, "Commands: ***Invalid Multi Word Command***\n");
      return; /* invalid command - don't update */
  }

#ifndef BOLOTEST
  int i_point = GETREADINDEX(point_index);

  if (!scheduled)
    CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
  else
    CommandData.pointing_mode.t = PointingData[i_point].t;
#endif

  WritePrevStatus();
}


/************************************************************/
/*                                                          */
/*  Initialize CommandData: read last valid state: if there */
/*   is no previous state file, set to default              */
/*                                                          */
/************************************************************/
void InitCommandData()
{
  int fp, n_read = 0, junk, extra = 0, i, j;

  if ((fp = open("/data/etc/spider/pcm.prev_status", O_RDONLY)) < 0) {
    berror(err, "Commands: Unable to open prev_status file for reading");
  } else {
    if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0)
      berror(err, "Commands: prev_status read()");
    if ((extra = read(fp, &junk, sizeof(junk))) < 0)
      berror(err, "Commands: extra prev_status read()");
    if (close(fp) < 0)
      berror(err, "Commands: prev_status close()");
  }

  /** this overrides prev_status **/
  CommandData.df = 0;
  CommandData.force_el = 0;

  CommandData.actbus.off = 0;
  CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
  CommandData.actbus.lock_goal = LS_DRIVE_OFF;
  CommandData.actbus.force_repoll = 0;
  CommandData.actbus.cindex = 0;
  CommandData.actbus.caddr[0] = 0;
  CommandData.actbus.caddr[1] = 0;
  CommandData.actbus.caddr[2] = 0;

  CommandData.xystage.is_new = 0;
  CommandData.xystage.force_repoll = 0;
  CommandData.hwpr.is_new = 0;
  CommandData.hwpr.force_repoll = 0;
  CommandData.hwpr.repeats = 0;

  CommandData.table.vel = 0.0;
  CommandData.table.pos = 90.0;
  CommandData.table.move = 0.0;
  CommandData.table.mode = 0;

  CommandData.Temporary.setLevel[0] = 1;
  CommandData.Temporary.setLevel[1] = 1;
  CommandData.Temporary.setLevel[2] = 1;
  CommandData.Temporary.setLevel[3] = 1;
  CommandData.Temporary.setLevel[4] = 1;

  CommandData.power.sc_tx.rst_count = 0;
  CommandData.power.sc_tx.set_count = 0;
  CommandData.power.das.rst_count = 0;
  CommandData.power.das.set_count = 0;
  CommandData.power.isc.rst_count = 0;
  CommandData.power.isc.set_count = 0;
  CommandData.power.osc.rst_count = 0;
  CommandData.power.osc.set_count = 0;
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
  CommandData.power.rx_main.rst_count = 0;
  CommandData.power.rx_main.set_count = 0;
  CommandData.power.rx_hk.rst_count = 0;
  CommandData.power.rx_hk.set_count = 0;
  CommandData.power.rx_amps.rst_count = 0;
  CommandData.power.rx_amps.set_count = 0;
  CommandData.power.gybox_off = 0;
  CommandData.power.gyro_off[0] = 0;
  CommandData.power.gyro_off[1] = 0;
  CommandData.power.gyro_off[2] = 0;
  CommandData.power.gyro_off[3] = 0;
  CommandData.power.gyro_off[4] = 0;
  CommandData.power.gyro_off[5] = 0;
  CommandData.power.hub232_off = 0;
  for (i=0; i<16; i++)
    CommandData.power.adc_reset[i] = 0;

  CommandData.gyheat.age = 0;

  /* don't use the fast gy offset calculator */
  CommandData.fast_offset_gy = 0;

  CommandData.reset_rw = 0;
  CommandData.reset_piv = 0;
  CommandData.reset_elev = 0;
  CommandData.restore_piv = 0;
  
  CommandData.slot_sched = 0x100;
  CommandData.parts_sched=0x0;

  /** return if we succsesfully read the previous status **/
  if (n_read != sizeof(struct CommandDataStruct))
    bprintf(warning, "Commands: prev_status: Wanted %i bytes but got %i.\n",
        (int) sizeof(struct CommandDataStruct), n_read);
  else if (extra > 0)
    bputs(warning, "Commands: prev_status: Extra bytes found.\n");
  else
    return;

  bputs(warning, "Commands: Regenerating Command Data and prev_status\n");

  /** prev_status overrides this stuff **/
  CommandData.at_float = 0;
  CommandData.timeout = 3600;
  CommandData.slot_sched = 0;
  CommandData.tdrss_bw = 6000;
  CommandData.iridium_bw = 2000;
  CommandData.vtx_sel[0] = vtx_isc;
  CommandData.vtx_sel[1] = vtx_osc;

  CommandData.slew_veto = VETO_MAX; /* 5 minutes */

  CommandData.pointing_mode.nw = 0;
  CommandData.pointing_mode.mode = P_DRIFT;
  CommandData.pointing_mode.X = 0;
  CommandData.pointing_mode.Y = 0;
  CommandData.pointing_mode.vaz = 0.0;
  CommandData.pointing_mode.del = 0.0;
  CommandData.pointing_mode.w = 0;
  CommandData.pointing_mode.h = 0;
  CommandData.pointing_mode.t = mcp_systime(NULL) + CommandData.timeout;
  CommandData.pointing_mode.dith = 0.0;

  CommandData.az_accel = 0.4; 
  CommandData.az_accel_max = 100.0;

  //CommandData.ele_gain.I = 5000; /* was 8000 */
  //CommandData.ele_gain.P = 5000; /* was 1200 */
  //CommandData.ele_gain.PT = 3000;

  CommandData.ele_gain.com = 10;
  CommandData.ele_gain.diff = 10;
  CommandData.ele_gain.accel_max = 1.0;

  CommandData.azi_gain.P = 4000;
  CommandData.azi_gain.I = 100;
  CommandData.azi_gain.PT = 3000;

  CommandData.pivot_gain.SP = 50; // dps
  CommandData.pivot_gain.PV = 400;
  CommandData.pivot_gain.PE = 0;
  CommandData.pivot_gain.F = 0.3;

  CommandData.disable_az = 1; 
  CommandData.disable_el = 0;

  CommandData.verbose_rw = 0;
  CommandData.verbose_el = 0;
  CommandData.verbose_piv = 0;

  CommandData.gyheat.setpoint = 15.0;
  CommandData.gyheat.age = 0;
  CommandData.gyheat.gain.P = 30;
  CommandData.gyheat.gain.I = 10;
  CommandData.gyheat.gain.D = 3;

  CommandData.table.tableGain.P = 6652;  //thousandths
  CommandData.table.tableGain.I = 302;   //ten-thousandths
  CommandData.table.tableGain.D = 13520; //hundredths

  CommandData.use_elenc1 = 1;
  CommandData.use_elenc2 = 1;
  CommandData.use_elclin = 1;
  CommandData.use_pss = 1;
  CommandData.use_mag = 1;
  CommandData.use_gps = 0;
  CommandData.lat_range = 1;
  CommandData.sucks = 1;

  CommandData.clin_el_trim = 0;
  CommandData.enc_el_trim = 0;
  CommandData.null_az_trim = 0;
  CommandData.mag_az_trim = 0;
  CommandData.dgps_az_trim = 0;
  CommandData.pss_az_trim = 0;

  CommandData.dgps_cov_limit = 0.3;
  CommandData.dgps_ants_limit = 0.5;

  SIPData.MKScal.m_hi = 0.01;
  SIPData.MKScal.m_med = 0.1;
  SIPData.MKScal.m_lo = 1;
  SIPData.MKScal.b_hi = 0;
  SIPData.MKScal.b_med = 0;
  SIPData.MKScal.b_lo = 0;

  CommandData.az_autogyro = 1;
  CommandData.el_autogyro = 1;
  CommandData.offset_ifel_gy = 0;
  CommandData.offset_ifroll_gy = 0;
  CommandData.offset_ifyaw_gy = 0;
  CommandData.gymask = 0x3f;
  
  CommandData.Temporary.dac_out[0] = 0x8000;
  CommandData.Temporary.dac_out[1] = 0x8000;
  CommandData.Temporary.dac_out[2] = 0x8000;
  CommandData.Temporary.dac_out[3] = 0x8000;
  CommandData.Temporary.dac_out[4] = 0x8000;

  for (i=0; i<6; i++) {
    CommandData.hk[i].pump_heat = 0;
    CommandData.hk[i].heat_switch = 1;	  //heat switch defaults to on
    CommandData.hk[i].fphi_heat = 0;
    for (j=0; j<4; j++) CommandData.hk[i].tile_heat[j] = 0;
    CommandData.hk[i].fplo_heat = 0.0;
    CommandData.hk[i].ssa_heat = 0.0;

    CommandData.hk[i].cernox.phase = 5.0;
    CommandData.hk[i].cernox.ampl = 0.3;
    CommandData.hk[i].ntd.phase = 5.0;
    CommandData.hk[i].ntd.ampl = 0.3;
  }
  CommandData.hk_last = 0;
  CommandData.hk_tile_last = 0;
  CommandData.hk_pulse_last = 0;
  CommandData.hk_vheat_last = 0.0;
  CommandData.hk_bias_freq = 50;

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

  CommandData.actbus.lock_vel = 110000;
  CommandData.actbus.lock_acc = 100;
  CommandData.actbus.lock_move_i = 50;
  CommandData.actbus.lock_hold_i = 0;

  CommandData.hwpr.vel = 1600;
  CommandData.hwpr.acc = 4;
  CommandData.hwpr.move_i = 20;
  CommandData.hwpr.hold_i = 10;

  /* hwpr positions separated by 22.5 degs.
     Calculated by Tristan on July 19th 2010*/
  CommandData.hwpr.pos[3] = 0.263096; 
  CommandData.hwpr.pos[2] = 0.326452;
  CommandData.hwpr.pos[1] = 0.38909;
  CommandData.hwpr.pos[0] = 0.450767;

  CommandData.hwpr.overshoot = 300;
  CommandData.hwpr.i_pos = 0;
  CommandData.hwpr.no_step = 0;
  CommandData.hwpr.use_pot = 1;
  CommandData.hwpr.pot_targ = 0.5;

  CommandData.pin_is_in = 1;

  CommandData.temp1 = 0;
  CommandData.temp2 = 0;
  CommandData.temp3 = 0;

  CommandData.lat = -77.86;  //McMurdo Building 096
  CommandData.lon = -167.04; //Willy Field Dec 2010

  WritePrevStatus();
}
