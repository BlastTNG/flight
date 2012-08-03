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

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
#define LOCK_OFFSET (-0.55) /* Updated by LMF on July 12th, 2012 */

/* based on isc_protocol.h */
#define ISC_SHUTDOWN_NONE     0
#define ISC_SHUTDOWN_HALT     1
#define ISC_SHUTDOWN_REBOOT   2
#define ISC_SHUTDOWN_CAMCYCLE 3

#define ISC_TRIGGER_INT  0
#define ISC_TRIGGER_EDGE 1
#define ISC_TRIGGER_POS  2
#define ISC_TRIGGER_NEG  3

void RecalcOffset(double, double);  /* actuators.c */

void SetRaDec(double, double); /* defined in pointing.c */
void SetTrimToSC(int);
void TrimOSCToISC();
void TrimISCToOSC();
void ClearTrim();
void AzElTrim(double, double);
void NormalizeAngle(double*);

int LoadUplinkFile(int slot); /*sched.c */

extern int doing_schedule; /* sched.c */

extern pthread_t watchdog_id;  /* mcp.c */
extern short int SouthIAm;
pthread_mutex_t mutex;

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

int sendSBSCCommand(const char *cmd); //sbsc.cpp

const char* SName(enum singleCommand command); // share/sip.c


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

    case trim_to_isc:
      CommandData.autotrim_enable = 0;
      CommandData.autotrim_rate = 0.0;
      SetTrimToSC(0);
      break;
    case trim_to_osc:
      CommandData.autotrim_enable = 0;
      CommandData.autotrim_rate = 0.0;
      SetTrimToSC(1);
      break;
    case trim_osc_to_isc:
      TrimOSCToISC();
      break;
    case trim_isc_to_osc:
      TrimISCToOSC();
      break;
    case reset_trims:
      CommandData.autotrim_enable = 0;
      CommandData.autotrim_rate = 0.0;
      ClearTrim();
      break;
    case autotrim_off:
      CommandData.autotrim_enable = 0;
      CommandData.autotrim_rate = 0.0;
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
    case pss_veto:
      CommandData.use_pss = 0;
      break;
    case isc_veto:
      CommandData.use_isc = 0;
      break;
    case osc_veto:
      CommandData.use_osc = 0;
      break;
    case mag_veto:
      CommandData.use_mag = 0;
      break;
    case gps_veto:
      CommandData.use_gps = 0;
      break;
    case elenc_veto:
      CommandData.use_elenc = 0;
      break;
    case elclin_veto:
      CommandData.use_elclin = 0;
      break;

    case pss_allow:
      CommandData.use_pss = 1;
      break;
    case isc_allow:
      CommandData.use_isc = 1;
      break;
    case osc_allow:
      CommandData.use_osc = 1;
      break;
    case mag_allow:
      CommandData.use_mag = 1;
      break;
    case gps_allow:
      CommandData.use_gps = 1;
      break;
    case elenc_allow:
      CommandData.use_elenc = 1;
      break;
    case elclin_allow:
      CommandData.use_elclin = 1;
      break;

    case sbsc_off:          /* power switching */
      CommandData.power.sbsc.set_count = 0;
      CommandData.power.sbsc.rst_count = LATCH_PULSE_LEN;
      break;
    case sbsc_on:
      CommandData.power.sbsc.rst_count = 0;
      CommandData.power.sbsc.set_count = LATCH_PULSE_LEN;
      break;
    case sbsc_cycle:
      CommandData.power.sbsc.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.sbsc.rst_count = LATCH_PULSE_LEN;
      break;
    case isc_off:
      CommandData.power.isc.set_count = 0;
      CommandData.power.isc.rst_count = LATCH_PULSE_LEN;
      break;
    case isc_on:
      CommandData.power.isc.rst_count = 0;
      CommandData.power.isc.set_count = LATCH_PULSE_LEN;
      break;
    case isc_cycle:
      CommandData.power.isc.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.isc.rst_count = LATCH_PULSE_LEN;
      break;
    case osc_off:
      CommandData.power.osc.set_count = 0;
      CommandData.power.osc.rst_count = LATCH_PULSE_LEN;
      break;
    case osc_on:
      CommandData.power.osc.rst_count = 0;
      CommandData.power.osc.set_count = LATCH_PULSE_LEN;
      break;
    case osc_cycle:
      CommandData.power.osc.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
      CommandData.power.osc.rst_count = LATCH_PULSE_LEN;
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

    case ramp:
      CommandData.Bias.biasRamp = 1;
      break;
    case fixed:
      CommandData.Bias.biasRamp = 0;
      break;

    case level_on:   /* Cryo commanding */
      CommandData.Cryo.heliumLevel = -1;
      break;
    case level_off:
      CommandData.Cryo.heliumLevel = 0;
      break;
    case level_pulse:
      CommandData.Cryo.heliumLevel = 350;  /* unit is 100Hz frames */
      break;
    case hwpr_enc_on:
      CommandData.Cryo.hwprPos = -1;
      break;
    case hwpr_enc_off:
      CommandData.Cryo.hwprPos = 0;
      break;
    case hwpr_enc_pulse:
      CommandData.Cryo.hwprPos = 50;
      break;
    case charcoal_on:
      CommandData.Cryo.charcoalHeater = 1;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case charcoal_off:
      CommandData.Cryo.charcoalHeater = 0;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case hs_charcoal_on:
      CommandData.Cryo.hsCharcoal= 1;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case hs_charcoal_off:
      CommandData.Cryo.hsCharcoal= 0;
      CommandData.Cryo.fridgeCycle = 0;
      break;
    case auto_cycle:
      CommandData.Cryo.fridgeCycle = 1;
      CommandData.Cryo.force_cycle = 0;
      break;
    case fridge_cycle:
      CommandData.Cryo.fridgeCycle = 1;
      CommandData.Cryo.force_cycle = 1;
      break;
    case cal_on:
      CommandData.Cryo.calibrator = on;
      break;
    case cal_off:
      CommandData.Cryo.calibrator = off;
      break;
    case jfet_on:
      CommandData.Cryo.JFETHeat = 1;
      CommandData.Cryo.autoJFETheat = 0;
      break;
    case jfet_off:
      CommandData.Cryo.JFETHeat = 0;
      CommandData.Cryo.autoJFETheat = 0;
      break;
    case hs_pot_on:
      CommandData.Cryo.hsPot = 1;
      break;
    case hs_pot_off:
      CommandData.Cryo.hsPot = 0;
      break;
    case bda_on:
      CommandData.Cryo.BDAHeat = 1;
      break;
    case bda_off:
      CommandData.Cryo.BDAHeat = 0;
      break;
    case pot_valve_open:
      CommandData.Cryo.potvalve_open = 100;
      CommandData.Cryo.potvalve_close = 0;
      break;
    case pot_valve_close:
      CommandData.Cryo.potvalve_close = 100;
      CommandData.Cryo.potvalve_open = 0;
      break;
    case pot_valve_on:
      CommandData.Cryo.potvalve_on = 1;
      break;
    case pot_valve_off:
      CommandData.Cryo.potvalve_on = 0;
      break;
    case l_valve_open:
      CommandData.Cryo.lvalve_open = 100;
      CommandData.Cryo.lvalve_close = 0;
      break;
    case l_valve_close:
      CommandData.Cryo.lvalve_close = 100;
      CommandData.Cryo.lvalve_open = 0;
      break;
    case he_valve_on:
      CommandData.Cryo.lhevalve_on = 1;
      break;
    case he_valve_off:
      CommandData.Cryo.lhevalve_on = 0;
      break;
    case ln_valve_on:
      CommandData.Cryo.lnvalve_on = 1;
      break;
    case ln_valve_off:
      CommandData.Cryo.lnvalve_on = 0;
      break;
    case auto_jfetheat:
      CommandData.Cryo.autoJFETheat = 1;
      break;

#ifndef BOLOTEST
    case balance_off:/* Balance pump commanding */
      CommandData.pumps.mode = bal_rest;
      CommandData.pumps.level = 0;
      break;
    case balance_auto:
      CommandData.pumps.mode= bal_auto;
      break;
    case balance_heat_on:/* Balance pump heating card commanding */
      CommandData.pumps.heat_on = 1;
      break;
    case balance_heat_off:
      CommandData.pumps.heat_on = 0;
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
  case lock45:  /* Lock Inner Frame at 45 (to be sent by CSBF pre-termination) */
      if (CommandData.pointing_mode.nw >= 0)
        CommandData.pointing_mode.nw = VETO_MAX;
      CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF;
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_LOCK;
      CommandData.pointing_mode.X = 0;
      CommandData.pointing_mode.Y = LockPosition(45.0);
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      bprintf(info, "Commands: Lock at : %g\n", CommandData.pointing_mode.Y);
      break;
    case repoll:
      CommandData.actbus.force_repoll = 1;
      CommandData.hwpr.force_repoll = 1;
      CommandData.xystage.force_repoll = 1;
      break;

    /* Shutter */
    case shutter_init:
      CommandData.actbus.shutter_goal = SHUTTER_INIT;
      break;
    case shutter_close:
      CommandData.actbus.shutter_goal = SHUTTER_CLOSED;
      break;
    case shutter_reset:
      CommandData.actbus.shutter_goal = SHUTTER_RESET;
      break;
    case shutter_open:
      CommandData.actbus.shutter_goal = SHUTTER_OPEN;
      break;
    case shutter_off:
      CommandData.actbus.shutter_goal = SHUTTER_OFF;
      break;
    case shutter_open_close:
      CommandData.actbus.shutter_goal = SHUTTER_CLOSED2;
      break;
    case shutter_close_slow:
      CommandData.actbus.shutter_goal = SHUTTER_CLOSED_SLOW;
      break;

    /* Actuators */
    case actuator_stop:
      CommandData.actbus.focus_mode = ACTBUS_FM_PANIC;
      /* fallthrough */
    case autofocus_veto:
      CommandData.actbus.tc_mode = TC_MODE_VETOED;
      break;
    case autofocus_allow:
      CommandData.actbus.tc_mode = TC_MODE_ENABLED;
      break;

    case hwpr_panic:
      CommandData.hwpr.mode = HWPR_PANIC;
      CommandData.hwpr.is_new = 1;
      break;

#ifndef BOLOTEST
      /***************************************/
      /********* ISC Commanding  *************/
    case isc_run:
      CommandData.ISCState[0].pause = 0;
      break;
    case isc_shutdown:
      CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_HALT;
      break;
    case isc_reboot:
      CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_REBOOT;
      break;
    case isc_cam_cycle:
      CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_CAMCYCLE;
      break;
    case isc_pause:
      CommandData.ISCState[0].pause = 1;
      break;
    case isc_abort:
      CommandData.ISCState[0].abort = 1;
      break;
    case isc_reconnect:
      CommandData.ISCControl[0].reconnect = 1;
      break;
    case isc_save_images:
      CommandData.ISCState[0].save = 1;
      break;
    case isc_discard_images:
      CommandData.ISCState[0].save = 0;
      break;
    case isc_full_screen:
      CommandData.ISCState[0].display_mode = full;
      break;
    case isc_trig_int:
      CommandData.ISCState[0].triggertype = ISC_TRIGGER_INT;
      break;
    case isc_trig_ext:
      CommandData.ISCState[0].triggertype = ISC_TRIGGER_NEG;
      break;
    case isc_auto_focus:
      CommandData.ISCControl[0].autofocus = 10;
      break;
    case isc_eye_on:
      CommandData.ISCState[0].eyeOn = 1;
      break;
    case isc_eye_off:
      CommandData.ISCState[0].eyeOn = 0;
      break;
    case isc_no_pyramid:
      CommandData.ISCState[0].useLost = 0;
      break;
    case isc_use_pyramid:
      CommandData.ISCState[0].useLost = 1;
      break;

      /***************************************/
      /********* OSC Commanding  *************/
      break;
    case osc_run:
      CommandData.ISCState[1].pause = 0;
      break;
    case osc_shutdown:
      CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_HALT;
      break;
    case osc_reboot:
      CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_REBOOT;
      break;
    case osc_cam_cycle:
      CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_CAMCYCLE;
      break;
    case osc_pause:
      CommandData.ISCState[1].pause = 1;
      break;
    case osc_abort:
      CommandData.ISCState[1].abort = 1;
      break;
    case osc_reconnect:
      CommandData.ISCControl[1].reconnect = 1;
      break;
    case osc_save_images:
      CommandData.ISCState[1].save = 1;
      break;
    case osc_discard_images:
      CommandData.ISCState[1].save = 0;
      break;
    case osc_full_screen:
      CommandData.ISCState[1].display_mode = full;
      break;
    case osc_trig_int:
      CommandData.ISCState[1].triggertype = ISC_TRIGGER_INT;
      break;
    case osc_trig_ext:
      CommandData.ISCState[1].triggertype = ISC_TRIGGER_NEG;
      break;
    case osc_auto_focus:
      CommandData.ISCControl[1].autofocus = 10;
      break;
    case osc_eye_on:
      CommandData.ISCState[1].eyeOn = 1;
      break;
    case osc_eye_off:
      CommandData.ISCState[1].eyeOn = 0;
      break;
    case osc_no_pyramid:
      CommandData.ISCState[1].useLost = 0;
      break;
    case osc_use_pyramid:
      CommandData.ISCState[1].useLost = 1;
      break;

      /***************************************/
      /********* SBSC Commanding  *************/
    case cam_cycle:
      sendSBSCCommand("Cpower");
    case cam_expose:
      sendSBSCCommand("CtrigExp");
      break;
    case cam_autofocus:
      if (CommandData.cam.forced)
	sendSBSCCommand("CtrigFocusF");
      else sendSBSCCommand("CtrigFocus");
      break;
    case cam_settrig_ext:
      sendSBSCCommand("CsetExpInt=0");
      CommandData.cam.expInt = 0;
      break;
    case cam_force_lens:
      CommandData.cam.forced = 1;
      break;
    case cam_unforce_lens:
      CommandData.cam.forced = 0;
      break;

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
    case vtx1_sbsc:
      CommandData.vtx_sel[0] = vtx_sbsc;
      break;
    case vtx2_isc:
      CommandData.vtx_sel[1] = vtx_isc;
      break;
    case vtx2_osc:
      CommandData.vtx_sel[1] = vtx_osc;
      break;
    case vtx2_sbsc:
      CommandData.vtx_sel[1] = vtx_sbsc;
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

    case reap_north:  /* Miscellaneous commands */
    case reap_south:
      if ((command == reap_north && !SouthIAm) || 
	  (command == reap_south && SouthIAm)) {
	bprintf(err, "Commands: Reaping the watchdog tickle on command.");
	pthread_cancel(watchdog_id);
      }
      break;
    case north_halt:
    case south_halt:
      if ((command == north_halt && !SouthIAm) || 
	  (command == south_halt && SouthIAm)) {
        bputs(warning, "Commands: Halting the MCC\n");
        if (system("/sbin/reboot") < 0)
	  berror(fatal, "Commands: failed to reboot, dying\n");
      }
      break;
    case xy_panic:
      CommandData.xystage.mode = XYSTAGE_PANIC;
      CommandData.xystage.is_new = 1;
    case xyzzy:
      break;
    default:
      bputs(warning, "Commands: ***Invalid Single Word Command***\n");
      return; /* invalid command - no write or update */
  }

  CommandData.command_count++;
  CommandData.last_command = (unsigned short)command;

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
  int i;
  char buf[256]; //for SBSC Commands
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
      bprintf(info,"Scan center: %f, %f", CommandData.pointing_mode.X,CommandData.pointing_mode.Y);
      CommandData.pointing_mode.w = rvalues[2];  /* width */
      CommandData.pointing_mode.vaz = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.h = 0;
      break;
    case el_scan:
      //      bprintf(info,"Commands: El scan not enabled yet!");     
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_EL_SCAN;
      CommandData.pointing_mode.X = rvalues[0];  /* az */
      CommandData.pointing_mode.Y = rvalues[1];  /* el */
      //      bprintf(info,"Scan center: %f, %f", CommandData.pointing_mode.X,CommandData.pointing_mode.Y);
      CommandData.pointing_mode.h = rvalues[2];  /* height */
      CommandData.pointing_mode.vel = rvalues[3]; /* az scan speed */
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
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
          (CommandData.pointing_mode.h != 0))  { /* N dither steps */
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
      CommandData.pointing_mode.n_dith = ivalues[5]; /* No of dither steps */
      break;
    case box:

      if ((CommandData.pointing_mode.mode != P_BOX) ||
          (CommandData.pointing_mode.X != rvalues[0]) || /* ra */
          (CommandData.pointing_mode.Y != rvalues[1]) || /* dec */
          (CommandData.pointing_mode.w != rvalues[2]) || /* width */
          (CommandData.pointing_mode.h != rvalues[3]) || /* height */
          (CommandData.pointing_mode.vaz != rvalues[4]) || /* az scan speed */
          (CommandData.pointing_mode.del != rvalues[5])) { /* el step size */ 
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
      CommandData.pointing_mode.n_dith = ivalues[6]; /* number of el dither steps */
      break;
    case el_box:

      if ((CommandData.pointing_mode.mode != P_EL_BOX) ||
          (CommandData.pointing_mode.X != rvalues[0]) || /* ra */
          (CommandData.pointing_mode.Y != rvalues[1]) || /* dec */
          (CommandData.pointing_mode.w != rvalues[2]) || /* width */
          (CommandData.pointing_mode.h != rvalues[3]) || /* height */
          (CommandData.pointing_mode.vel != rvalues[4]) || /* az scan speed */
          (CommandData.pointing_mode.daz != rvalues[5])) { /* el step size */

        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
     
      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }
      
      CommandData.pointing_mode.mode = P_EL_BOX;
      CommandData.pointing_mode.X = rvalues[0]; /* ra */
      CommandData.pointing_mode.Y = rvalues[1]; /* dec */
      CommandData.pointing_mode.w = rvalues[2]; /* width */
      CommandData.pointing_mode.h = rvalues[3]; /* height */
      CommandData.pointing_mode.vel = rvalues[4]; /* az scan speed */
      CommandData.pointing_mode.daz = rvalues[5]; /* el step size */
      CommandData.pointing_mode.n_dith = ivalues[6]; /* number of el dither steps */

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
          (CommandData.pointing_mode.del != rvalues[9])) {/* el step size */
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
      CommandData.pointing_mode.n_dith = ivalues[10]; /* N dither steps */
      break;
    case az_scan_accel:
      CommandData.az_accel = rvalues[0];
      break;
    case set_scan_params:
      CommandData.pointing_mode.next_i_hwpr = ivalues[0];
      CommandData.pointing_mode.next_i_dith = ivalues[1];
      break;

      /***************************************/
      /********** Pointing Sensor Trims *******/
    case az_el_trim:
      AzElTrim(rvalues[0], rvalues[1]);
      break;
    case ra_dec_set:
      SetRaDec(rvalues[0], rvalues[1]);
      break;
    case autotrim_to_sc:
      CommandData.autotrim_thresh = rvalues[0];
      CommandData.autotrim_time = ivalues[1];
      CommandData.autotrim_rate = rvalues[2];
      CommandData.autotrim_isc_last_bad = mcp_systime(NULL);
      CommandData.autotrim_osc_last_bad = CommandData.autotrim_isc_last_bad;
      CommandData.autotrim_enable = 1;
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
    case mag_cal:
      CommandData.cal_xmax_mag = ivalues[0];
      CommandData.cal_xmin_mag = ivalues[1];
      CommandData.cal_ymax_mag = ivalues[2];
      CommandData.cal_ymin_mag = ivalues[3];      
      break;
      
      /***************************************/
      /********** Pointing Motor Gains *******/
    case el_gain:  /* ele gains */
      CommandData.ele_gain.P = ivalues[0];
      CommandData.ele_gain.I = ivalues[1];
      CommandData.ele_gain.PT = ivalues[2];
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
    case shutter_step:
      CommandData.actbus.shutter_step = ivalues[0];
      break;
    case shutter_step_slow:
      CommandData.actbus.shutter_step_slow = ivalues[0];
      break;
    case general: /* General actuator bus command */
      CommandData.actbus.caddr[CommandData.actbus.cindex] = ivalues[0] + 0x30;
      copysvalue(CommandData.actbus.command[CommandData.actbus.cindex],
          svalues[1]);
      CommandData.actbus.cindex = INC_INDEX(CommandData.actbus.cindex);
      break;
    case delta_secondary:
      CommandData.actbus.focus = ivalues[0];
      CommandData.actbus.focus_mode = ACTBUS_FM_DELFOC;
      break;
    case set_secondary:
      CommandData.actbus.focus = ivalues[0] + POSITION_FOCUS 
	+ CommandData.actbus.sf_offset;
      CommandData.actbus.focus_mode = ACTBUS_FM_FOCUS;
      break;
    case thermo_gain:
      CommandData.actbus.tc_step = ivalues[2];
      CommandData.actbus.tc_wait = ivalues[3] * 300; /* convert min->5Hz */
      CommandData.actbus.sf_time = CommandData.actbus.tc_wait - 5;
      RecalcOffset(rvalues[0], rvalues[1]);
      CommandData.actbus.g_primary = rvalues[0];
      CommandData.actbus.g_secondary = rvalues[1];
      break;
    case focus_offset:
      CommandData.actbus.sf_offset = ivalues[0];
      CommandData.actbus.sf_time = CommandData.actbus.tc_wait - 5;
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
    case thermo_param:
      CommandData.actbus.tc_spread = rvalues[0];
      CommandData.actbus.tc_prefp = ivalues[1];
      CommandData.actbus.tc_prefs = ivalues[2];
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
      /********** Balance System  ************/
    case balance_gain:
      CommandData.pumps.level_on_bal = rvalues[0] * 1990.13; /* 1990.13 DAC/Amp*/
      CommandData.pumps.level_off_bal = rvalues[1] * 1990.13;
      CommandData.pumps.level_target_bal = rvalues[2] * 1990.13;
      CommandData.pumps.gain_bal = rvalues[3];
      break;
    case balance_manual:
      CommandData.pumps.level = rvalues[0];
      CommandData.pumps.mode = bal_manual;
      break;
    case balance_tset:
      CommandData.pumps.heat_tset = rvalues[0];
      break;

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
    case params_test:/* Do nothing, with lots of parameters */
      bprintf(info, "Integer params 'i': %d 'l' %d", ivalues[0], ivalues[1]);
      bprintf(info, "Float params 'f': %g 'd' %g", rvalues[2], rvalues[3]);
      bprintf(info, "String param 's': %s", svalues[4]);
      CommandData.plover = ivalues[0];
      break;
    case plugh:/* A hollow voice says "Plugh". */
      CommandData.plover = ivalues[0];
      break;
#endif

      /***************************************/
      /*************** Bias  *****************/
      /* used to be multiplied by 2 here, but screw up prev_satus */
      /* need to multiply later instead */
    case bias_level_500:    /* Set bias 1 (500) */
      CommandData.Bias.bias[0] = ivalues[0];
      CommandData.Bias.setLevel[0] = 1;
      break;
    case bias_level_350:   /* Set bias 2 (350) */
      CommandData.Bias.bias[1] = ivalues[0];
      CommandData.Bias.setLevel[1] = 1;
      break;
    case bias_level_250:   /* Set bias 3 (250) */
      CommandData.Bias.bias[2] = ivalues[0];
      CommandData.Bias.setLevel[2] = 1;
      break;
    case bias_level_rox:   /* Set bias 4 (ROX) */
      CommandData.Bias.bias[3] = ivalues[0];
      CommandData.Bias.setLevel[3] = 1;
      break;
    case bias_level_x:   /* Set bias 5 (spare) */
      CommandData.Bias.bias[4] = ivalues[0];
      CommandData.Bias.setLevel[4] = 1;
      break;
    case phase:
      CommandData.phaseStep.do_step=0;
      if (ivalues[0] >= DAS_START && ivalues[0] <= DAS_START + DAS_CARDS*4/3
	  && ivalues[0]%4 != 0)
        CommandData.Phase[(ivalues[0] - DAS_START)*3/4] = ivalues[1];
      else if (ivalues[0] == 0)
        for (i = 0; i < DAS_CARDS; ++i)
          CommandData.Phase[i] = ivalues[1];
      else if (ivalues[0] == 13)
        CommandData.Phase[DAS_CARDS] = ivalues[1];
      break;
    case phase_step:
      CommandData.phaseStep.do_step=1;
      CommandData.phaseStep.start=ivalues[0];
      CommandData.phaseStep.end=ivalues[1];
      CommandData.phaseStep.nsteps=ivalues[2];
      CommandData.phaseStep.dt=ivalues[3];
      break;
    case bias_step:
      CommandData.Bias.biasStep.do_step = 1;
      CommandData.Bias.biasStep.start = ivalues[0];
      CommandData.Bias.biasStep.end = ivalues[1];
      CommandData.Bias.biasStep.nsteps = ivalues[2];
      CommandData.Bias.biasStep.dt = ivalues[3];
      CommandData.Bias.biasStep.pulse_len = ivalues[4];
      CommandData.Bias.biasStep.arr_ind = ivalues[5];
      break;
      /***************************************/
      /*********** Cal Lamp  *****************/
    case cal_pulse:
      CommandData.Cryo.calibrator = pulse;
      CommandData.Cryo.calib_pulse = ivalues[0] / 10;
      break;
    case cal_repeat: 
      CommandData.Cryo.calibrator = repeat;
      CommandData.Cryo.calib_pulse = ivalues[0] / 10;
      CommandData.Cryo.calib_period = ivalues[1];
      
      break;

      /***************************************/
      /********* Cryo heat   *****************/
    case jfet_set:
      CommandData.Cryo.JFETSetOn = rvalues[0];
      CommandData.Cryo.JFETSetOff = rvalues[1];
      break;

    case fridge_cycle_params:
      CommandData.Cryo.cycle_start_temp = rvalues[0];
      CommandData.Cryo.cycle_pot_max = rvalues[1];
      CommandData.Cryo.cycle_charcoal_max = rvalues[2];
      CommandData.Cryo.cycle_charcoal_timeout = rvalues[3];
      CommandData.Cryo.cycle_charcoal_settle = rvalues[4];
      CommandData.Cryo.cycle_settle_timeout = rvalues[5];
      break;

#ifndef BOLOTEST
      /***************************************/
      /********* ISC Commanding  *************/
    case isc_set_focus:
      CommandData.ISCState[0].focus_pos = ivalues[0];
      break;
    case isc_foc_off:
      CommandData.ISCState[0].focusOffset = ivalues[0];
      break;
    case isc_set_aperture:
      CommandData.ISCState[0].ap_pos = ivalues[0];
      break;
    case isc_pixel_centre:
      CommandData.ISCState[0].roi_x = ivalues[0];
      CommandData.ISCState[0].roi_y = ivalues[1];
      CommandData.ISCState[0].display_mode = roi;
      break;
    case isc_blob_centre:
      CommandData.ISCState[0].blob_num = ivalues[0];
      CommandData.ISCState[0].display_mode = blob;
      break;
    case isc_offset:
      CommandData.ISCState[0].azBDA = rvalues[0] * DEG2RAD;
      CommandData.ISCState[0].elBDA = rvalues[1] * DEG2RAD;
      break;
    case isc_integrate:
      i = (int)(rvalues[0]/10. + 0.5) ;
      if (i % 2) i++;
      CommandData.ISCControl[0].fast_pulse_width = i;
      i = (int)(rvalues[1]/10. + 0.5) ;
      if (i % 2) i++;
      CommandData.ISCControl[0].pulse_width = i;
      break;
    case isc_det_set:
      CommandData.ISCState[0].grid = ivalues[0];
      CommandData.ISCState[0].sn_threshold = rvalues[1];
      CommandData.ISCState[0].mult_dist = ivalues[2];
      break;
    case isc_blobs:
      CommandData.ISCState[0].minBlobMatch = ivalues[0];
      CommandData.ISCState[0].maxBlobMatch = ivalues[1];
      break;
    case isc_catalogue:
      CommandData.ISCState[0].mag_limit = rvalues[0];
      CommandData.ISCState[0].norm_radius = rvalues[1] * DEG2RAD;
      CommandData.ISCState[0].lost_radius = rvalues[2] * DEG2RAD;
      break;
    case isc_tolerances:
      CommandData.ISCState[0].tolerance = 
	(rvalues[0] / 3600. * DEG2RAD);
      CommandData.ISCState[0].match_tol = (rvalues[1] / 100);
      CommandData.ISCState[0].quit_tol = (rvalues[2] / 100);
      CommandData.ISCState[0].rot_tol = (rvalues[3] * DEG2RAD);
      break;
    case isc_hold_current:
      CommandData.ISCState[0].hold_current = ivalues[0];
      break;
    case isc_save_period:
      CommandData.ISCControl[0].save_period = ivalues[0] * 100;
      break;
    case isc_gain:
      CommandData.ISCState[0].gain = rvalues[0];
      CommandData.ISCState[0].offset = ivalues[1];
      break;
    case isc_max_age:
      CommandData.ISCControl[0].max_age = ivalues[0]/10; //convert from ms to frames
      break;

      /***************************************/
      /********* OSC Commanding  *************/
    case osc_set_focus:
      CommandData.ISCState[1].focus_pos = ivalues[0];
      break;
    case osc_foc_off:
      CommandData.ISCState[1].focusOffset = ivalues[0];
      break;
    case osc_set_aperture:
      CommandData.ISCState[1].ap_pos = ivalues[0];
      break;
    case osc_pixel_centre:
      CommandData.ISCState[1].roi_x = ivalues[0];
      CommandData.ISCState[1].roi_y = ivalues[1];
      CommandData.ISCState[1].display_mode = roi;
      break;
    case osc_blob_centre:
      CommandData.ISCState[1].blob_num = ivalues[0];
      CommandData.ISCState[1].display_mode = blob;
      break;
    case osc_offset:
      CommandData.ISCState[1].azBDA = rvalues[0] * DEG2RAD;
      CommandData.ISCState[1].elBDA = rvalues[1] * DEG2RAD;
      break;
    case osc_integrate:
      i = (int)(rvalues[0]/10. + 0.5) ;
      if (i % 2) i++;
      CommandData.ISCControl[1].fast_pulse_width = i;
      i = (int)(rvalues[1]/10. + 0.5) ;
      if (i % 2) i++;
      CommandData.ISCControl[1].pulse_width = i;
      break;
    case osc_det_set:
      CommandData.ISCState[1].grid = ivalues[0];
      CommandData.ISCState[1].sn_threshold = rvalues[1];
      CommandData.ISCState[1].mult_dist = ivalues[2];
      break;
    case osc_blobs:
      CommandData.ISCState[1].minBlobMatch = ivalues[0];
      CommandData.ISCState[1].maxBlobMatch = ivalues[1];
      break;
    case osc_catalogue:
      CommandData.ISCState[1].mag_limit = rvalues[0];
      CommandData.ISCState[1].norm_radius = rvalues[1] * DEG2RAD;
      CommandData.ISCState[1].lost_radius = rvalues[2] * DEG2RAD;
      break;
    case osc_tolerances:
      CommandData.ISCState[1].tolerance = 
	(rvalues[0] / 3600. * DEG2RAD);
      CommandData.ISCState[1].match_tol = (rvalues[1] / 100);
      CommandData.ISCState[1].quit_tol = (rvalues[2] / 100);
      CommandData.ISCState[1].rot_tol = (rvalues[3] * DEG2RAD);
      break;
    case osc_hold_current:
      CommandData.ISCState[1].hold_current = ivalues[0];
      break;
    case osc_save_period:
      CommandData.ISCControl[1].save_period = ivalues[0] * 100;
      break;
    case osc_gain:
      CommandData.ISCState[1].gain = rvalues[0];
      CommandData.ISCState[1].offset = ivalues[1];
      break;
    case osc_max_age:
      CommandData.ISCControl[1].max_age = ivalues[0]/10; //convert from ms to frames
      break;
      /***************************************/
      /********* SBSC Commanding  *************/ 
    case cam_any:
      sendSBSCCommand(svalues[0]);
      break;
    case cam_trig_delay:
      CommandData.cam.delay = rvalues[0];
      break;
    case cam_settrig_timed:
      sprintf(buf, "CsetExpInt=%d", ivalues[0]);
      sendSBSCCommand(buf);
      CommandData.cam.expInt = ivalues[0];
      break;
    case cam_exp_params:
      sprintf(buf, "CsetExpTime=%d", ivalues[0]);
      CommandData.cam.expTime = ivalues[0];
      sendSBSCCommand(buf);
      break;
    case cam_focus_params:
      sprintf(buf, "CsetFocRsln=%d", ivalues[0]);
      sendSBSCCommand(buf);
      sprintf(buf, "CsetFocRnge=%d", ivalues[1]);
      sendSBSCCommand(buf); 
      CommandData.cam.focusRes = ivalues[0];
      CommandData.cam.focusRng = ivalues[1];
      break;
    case cam_bad_pix:
      sprintf(buf, "IsetBadpix=%d %d %d", ivalues[0], ivalues[1], ivalues[2]);
      sendSBSCCommand(buf);
      break;
    case cam_blob_params:
      sprintf(buf, "IsetMaxBlobs=%d", ivalues[0]);
      sendSBSCCommand(buf);
      sprintf(buf, "IsetGrid=%d", ivalues[1]);
      sendSBSCCommand(buf);
      sprintf(buf, "IsetThreshold=%f", rvalues[2]);
      sendSBSCCommand(buf);
      sprintf(buf, "IsetDisttol=%d", ivalues[3]);
      sendSBSCCommand(buf);
      CommandData.cam.maxBlobs = ivalues[0];
      CommandData.cam.grid = ivalues[1];
      CommandData.cam.threshold = rvalues[2];
      CommandData.cam.minBlobDist = ivalues[3];
      break;
    case cam_lens_any:
      sprintf(buf, "L=%s", svalues[0]);
      sendSBSCCommand(buf);
      break;
    case cam_lens_move:
      if (CommandData.cam.forced)
	sprintf(buf, "Lforce=%d", ivalues[0]);
      else sprintf(buf, "Lmove=%d", ivalues[0]);
      sendSBSCCommand(buf);
      break;
    case cam_lens_params:
      sprintf(buf, "LsetTol=%d", ivalues[0]);
      sendSBSCCommand(buf);
      CommandData.cam.moveTol = ivalues[0];
      break;
#endif
    default:
      bputs(warning, "Commands: ***Invalid Multi Word Command***\n");
      return; /* invalid command - don't update */
  }

  CommandData.command_count++;
  //set high bit to differentiate multi-commands from single
  CommandData.last_command = (unsigned short)command | 0x8000;

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
  int fp, n_read = 0, junk, extra = 0, i;

  if ((fp = open(PREV_STATUS_FILE, O_RDONLY)) < 0) {
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

  CommandData.Bias.biasRamp = 0;
  CommandData.Bias.biasStep.do_step = 0;
  CommandData.Bias.biasStep.start = 1;
  CommandData.Bias.biasStep.end = 32767;
  CommandData.Bias.biasStep.nsteps = 1000;
  CommandData.Bias.biasStep.pulse_len = 10;
  CommandData.Bias.biasStep.dt = 1000;
  CommandData.Bias.biasStep.arr_ind = 0;

  CommandData.phaseStep.do_step = 0;
  CommandData.phaseStep.start = 1;
  CommandData.phaseStep.end = 32767;
  CommandData.phaseStep.nsteps = 1000;
  CommandData.phaseStep.dt = 1000;

  //forces reload of saved bias values
  CommandData.Bias.setLevel[0] = 1;
  CommandData.Bias.setLevel[1] = 1;
  CommandData.Bias.setLevel[2] = 1;
  CommandData.Bias.setLevel[3] = 1;
  CommandData.Bias.setLevel[4] = 1;

  CommandData.Temporary.setLevel[0] = 1;
  CommandData.Temporary.setLevel[1] = 1;
  CommandData.Temporary.setLevel[2] = 1;
  CommandData.Temporary.setLevel[3] = 1;
  CommandData.Temporary.setLevel[4] = 1;

  CommandData.ISCState[0].shutdown = ISC_SHUTDOWN_NONE;
  CommandData.ISCState[1].shutdown = ISC_SHUTDOWN_NONE;

  CommandData.power.sc_tx.rst_count = 0;
  CommandData.power.sc_tx.set_count = 0;
  CommandData.power.das.rst_count = 0;
  CommandData.power.das.set_count = 0;
  CommandData.power.isc.rst_count = 0;
  CommandData.power.isc.set_count = 0;
  CommandData.power.osc.rst_count = 0;
  CommandData.power.osc.set_count = 0;
  CommandData.power.sbsc.rst_count = 0;
  CommandData.power.sbsc.set_count = 0;
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

  CommandData.Cryo.BDAHeat = 0;

  CommandData.Cryo.potvalve_on = 0;
  CommandData.Cryo.potvalve_open = 0;
  CommandData.Cryo.potvalve_close = 0;
  CommandData.Cryo.lhevalve_on = 0;
  CommandData.Cryo.lvalve_open = 0;
  CommandData.Cryo.lvalve_close = 0;
  CommandData.Cryo.lnvalve_on = 0;

  /* don't use the fast gy offset calculator */
  CommandData.fast_offset_gy = 0;

  /* force autotrim to reset its wait time on restart */
  CommandData.autotrim_isc_last_bad = mcp_systime(NULL);
  CommandData.autotrim_osc_last_bad = CommandData.autotrim_isc_last_bad;

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
  CommandData.command_count = 0;
  CommandData.last_command = 0xffff;

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
  CommandData.pointing_mode.n_dith = 0;
  CommandData.pointing_mode.next_i_dith = 0;
  CommandData.pointing_mode.next_i_hwpr = 0;
  CommandData.pointing_mode.n_dith = 0;
  CommandData.pointing_mode.vel = 0.0;
  CommandData.pointing_mode.daz = 0.0;

  CommandData.az_accel = 0.4; 

  CommandData.ele_gain.I = 200; /* was 8000 */
  CommandData.ele_gain.P = 200; /* was 1200 */
  CommandData.ele_gain.PT = 200;

  CommandData.azi_gain.P = 200;
  CommandData.azi_gain.I = 200;
  CommandData.azi_gain.PT = 200;

  CommandData.pivot_gain.SP = 30; // dps
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

  CommandData.use_elenc = 1;
  CommandData.use_elclin = 1;
  CommandData.use_pss = 1;
  CommandData.use_isc = 1;
  CommandData.use_osc = 1;
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

  CommandData.autotrim_enable = 0;
  CommandData.autotrim_thresh = 0.05;
  CommandData.autotrim_rate = 1.0;
  CommandData.autotrim_time = 60;

  CommandData.cal_xmax_mag = 34783;
  CommandData.cal_ymax_mag = 34691;
  CommandData.cal_xmin_mag = 32250;
  CommandData.cal_ymin_mag = 32180;
  
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
  
  CommandData.pumps.level_on_bal = 2.0 * 1990.13;  
  CommandData.pumps.level_off_bal = 0.5 * 1900.13;
  CommandData.pumps.level_target_bal = 0.0 * 1990.13;
  CommandData.pumps.gain_bal = 0.2;
  CommandData.pumps.mode = bal_auto;
  CommandData.pumps.heat_on = 1;
  CommandData.pumps.heat_tset = 20;

  CommandData.Temporary.dac_out[0] = 0x8000;
  CommandData.Temporary.dac_out[1] = 0x8000;
  CommandData.Temporary.dac_out[2] = 0x8000;
  CommandData.Temporary.dac_out[3] = 0x8000;
  CommandData.Temporary.dac_out[4] = 0x8000;

  CommandData.Bias.bias[0] = 12470;   //500um
  CommandData.Bias.bias[1] = 11690;   //350um
  CommandData.Bias.bias[2] = 13940;   //250um
  CommandData.Bias.bias[3] = 1050;   //ROX
  CommandData.Bias.bias[4] = 16384;  //X

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

  CommandData.Cryo.charcoalHeater = 0;
  CommandData.Cryo.hsCharcoal = 1;
  CommandData.Cryo.fridgeCycle = 0;
  CommandData.Cryo.force_cycle = 0;
  CommandData.Cryo.hsPot = 0;
  CommandData.Cryo.heliumLevel = 0;
  CommandData.Cryo.he4_lev_old = 0;
  CommandData.Cryo.hwprPos = 0;
  CommandData.Cryo.hwpr_pos_old = 0;
  CommandData.Cryo.JFETHeat = 0;
  CommandData.Cryo.autoJFETheat = 1;
  CommandData.Cryo.JFETSetOn = 120;
  CommandData.Cryo.JFETSetOff = 135;
  CommandData.Cryo.calibrator = repeat;
  CommandData.Cryo.calib_pulse = 30; /* = 300 ms @ 100Hz */
  CommandData.Cryo.calib_period = 3000; /* = 600 s @ 5Hz */
  CommandData.Cryo.calib_repeats = -1;  //indefinitely

  CommandData.Cryo.cycle_start_temp = 0.375;
  CommandData.Cryo.cycle_pot_max = 2.5;
  CommandData.Cryo.cycle_charcoal_max = 40.0;
  CommandData.Cryo.cycle_charcoal_timeout = 30.0;
  CommandData.Cryo.cycle_charcoal_settle = 25.0;
  CommandData.Cryo.cycle_settle_timeout = 40.0;

  CommandData.ISCState[0].useLost = 1;
  CommandData.ISCState[0].abort = 0;
  CommandData.ISCState[0].pause = 0;
  CommandData.ISCState[0].save = 0;
  CommandData.ISCState[0].eyeOn = 1;
  CommandData.ISCState[0].hold_current = 50;
  CommandData.ISCState[0].autofocus = 0;
  CommandData.ISCState[0].focus_pos = 0;
  CommandData.ISCState[0].MCPFrameNum = 0;
  CommandData.ISCState[0].focusOffset = 0;
  CommandData.ISCState[0].ap_pos = 495;
  CommandData.ISCState[0].display_mode = full;
  /* ISC-BDA offsets per Lorenzo Moncelsi on 2010-12-16 */
  CommandData.ISCState[0].azBDA = 0.16527 * DEG2RAD;
  CommandData.ISCState[0].elBDA = 0.81238 * DEG2RAD;
  CommandData.ISCControl[0].max_age = 200; /* 2000 ms*/

  CommandData.ISCState[0].brightStarMode = 0;
  CommandData.ISCState[0].grid = 38;
  CommandData.ISCState[0].minBlobMatch =  3;
  CommandData.ISCState[0].maxBlobMatch =  7;
  CommandData.ISCState[0].sn_threshold = 4.5;
  CommandData.ISCState[0].mult_dist = 30;
  CommandData.ISCState[0].mag_limit = 9.5;
  CommandData.ISCState[0].norm_radius = 3. * DEG2RAD;
  CommandData.ISCState[0].lost_radius = 6. * DEG2RAD;
  CommandData.ISCState[0].tolerance = 10. / 3600. * DEG2RAD; /* 10 arcsec */
  CommandData.ISCState[0].match_tol = 0.5;
  CommandData.ISCState[0].quit_tol = 1;
  CommandData.ISCState[0].rot_tol = 10 * DEG2RAD;
  CommandData.ISCState[0].triggertype = ISC_TRIGGER_NEG;
  CommandData.ISCState[0].gain = 1;
  CommandData.ISCState[0].offset = 0;
  CommandData.ISCControl[0].autofocus = 0;
  CommandData.ISCControl[0].save_period = 12000; /* 120 sec */
  CommandData.ISCControl[0].pulse_width = 18; /* 180.00 msec */
  CommandData.ISCControl[0].fast_pulse_width = 8; /* 80.00 msec */

  CommandData.ISCState[1].useLost = 1;
  CommandData.ISCState[1].abort = 0;
  CommandData.ISCState[1].pause = 0;
  CommandData.ISCState[1].save = 0;
  CommandData.ISCState[1].eyeOn = 1;
  CommandData.ISCState[1].hold_current = 50;
  CommandData.ISCState[1].autofocus = 0;
  CommandData.ISCState[1].focus_pos = 0;
  CommandData.ISCState[1].MCPFrameNum = 0;
  CommandData.ISCState[1].focusOffset = 450;
  CommandData.ISCState[1].ap_pos = 495;
  CommandData.ISCState[1].display_mode = full;
  /* ISC-BDA offsets per Lorenzo Moncelsi on 2010-12-16 */
  CommandData.ISCState[1].azBDA = -0.2862 * DEG2RAD;
  CommandData.ISCState[1].elBDA = -0.5918 * DEG2RAD;
  CommandData.ISCControl[1].max_age = 200;  /* 2000 ms*/

  CommandData.ISCState[1].brightStarMode = 0;
  CommandData.ISCState[1].grid = 38;
  CommandData.ISCState[1].minBlobMatch =  3;
  CommandData.ISCState[1].maxBlobMatch =  7;
  CommandData.ISCState[1].sn_threshold = 4.5;
  CommandData.ISCState[1].mult_dist = 30;
  CommandData.ISCState[1].mag_limit = 9.5;
  CommandData.ISCState[1].norm_radius = 3. * DEG2RAD;
  CommandData.ISCState[1].lost_radius = 6. * DEG2RAD;
  CommandData.ISCState[1].tolerance = 10. / 3600. * DEG2RAD; /* 10 arcsec */
  CommandData.ISCState[1].match_tol = 0.5;
  CommandData.ISCState[1].quit_tol = 1;
  CommandData.ISCState[1].rot_tol = 10 * DEG2RAD;
  CommandData.ISCState[1].triggertype = ISC_TRIGGER_NEG;
  CommandData.ISCState[1].gain = 1;
  CommandData.ISCState[1].offset = 0;
  CommandData.ISCControl[1].autofocus = 0;
  CommandData.ISCControl[1].save_period = 12000; /* 120 sec */
  CommandData.ISCControl[1].pulse_width = 18; /* 180.00 msec */
  CommandData.ISCControl[1].fast_pulse_width = 8; /* 80.00 msec */

  CommandData.temp1 = 0;
  CommandData.temp2 = 0;
  CommandData.temp3 = 0;
  CommandData.df = 0;

  CommandData.lat = -77.86;  //McMurdo Building 096
  CommandData.lon = -167.04; //Willy Field Dec 2010

  CommandData.Phase[0] = 27000;
  CommandData.Phase[1] = 27000;
  CommandData.Phase[2] = 27000;
  CommandData.Phase[3] = 26700;
  CommandData.Phase[4] = 26700;
  CommandData.Phase[5] = 26700;
  CommandData.Phase[6] =  9800;
  CommandData.Phase[7] =  9800;
  CommandData.Phase[8] =  9800;
  CommandData.Phase[9] =  9800;
  CommandData.Phase[10] = 10000;
  CommandData.Phase[11] =  9800;
  CommandData.Phase[12] = 11600;    //ROX

  WritePrevStatus();
}
