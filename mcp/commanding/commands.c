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

#include <conversions.h>
#include <crc.h>
#include <pointing.h>
#include <blast_sip_interface.h>
#include <ec_motors.h>

#include "command_list.h"
#include "command_struct.h"
#include "framing.h"
#include "mcp.h"
#include "tx.h"
#include "pointing_struct.h"
#include "channels_tng.h"

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
// #define LOCK_OFFSET (-0.77) /* Updated by LMF on July 12th, 2012 */
#define LOCK_OFFSET (0.0)
#define NUM_LOCK_POS 10
static const double lock_positions[NUM_LOCK_POS] = {0.03, 5.01, 14.95, 24.92, 34.88, 44.86, 54.83, 64.81, 74.80, 89.78};

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

/* defined in pointing.c */
void SetTrimToSC(int);
void ClearTrim();
void AzElTrim(double, double);
void NormalizeAngle(double*);

int LoadUplinkFile(int slot); /*sched.c */

extern int doing_schedule; /* sched.c */

extern int16_t SouthIAm;
pthread_mutex_t mutex;

struct SIPDataStruct SIPData;
struct CommandDataStruct CommandData;

const char* SName(enum singleCommand command); // share/sip.c

/** Write the Previous Status: called whenever anything changes */
static void WritePrevStatus()
{
  int fp, n;

  CommandData.checksum = 0;
  CommandData.checksum = crc32_le(0, (uint8_t*) &CommandData, sizeof(CommandData));
  /** write the default file */
  fp = open(PREV_STATUS_FILE, O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    berror(err, "mcp.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    berror(err, "mcp.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    berror(err, "mcp.prev_status close()");
    return;
  }

  framing_publish_command_data(&CommandData);
}

/* calculate the nearest lockable elevation */
double LockPosition(double elevation)
{
  int i_pos;
  double min_err = 360.0;
  double err;
  int i_min_err = 0;

  for (i_pos = 0; i_pos <NUM_LOCK_POS; i_pos++) {
    err = fabs(elevation - lock_positions[i_pos]);
    if (err < min_err) {
      i_min_err = i_pos;
      min_err = err;
    }
  }
  return (lock_positions[i_min_err] + LOCK_OFFSET);
}

static bool xsc_command_applies_to(int which_to_check, int which)
{
    if (which_to_check == 0 || which_to_check == 1) {
        if (which == 2 || which == which_to_check) {
            return true;
        }
    }
    return false;
}

void xsc_activate_command(int which, int command_index)
{
    if (command_index < xC_num_command_admins) {
        CommandData.XSC[which].net.command_admins[command_index].is_new_countdown =
                CommandData.XSC[which].is_new_window_period_cs;
        CommandData.XSC[which].net.command_admins[command_index].counter++;
    } else {
        blast_warn("Warning: xsc_activate_command called with invalid index");
    }
}

void SingleCommand(enum singleCommand command, int scheduled)
{
#ifndef BOLOTEST
    int i_point = GETREADINDEX(point_index);
    double sun_az;
#endif

    if (!scheduled)
    blast_info("Commands: Single command: %d (%s)\n", command, SName(command));

//   Update CommandData structure with new info

    switch (command) {
#ifndef BOLOTEST
        case stop:  // Pointing abort
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_DRIFT;
            CommandData.pointing_mode.X = 0;
            CommandData.pointing_mode.Y = 0;
            CommandData.pointing_mode.vaz = 0.0;
            CommandData.pointing_mode.del = 0.0;
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.h = 0;
            break;
        case antisun:  // turn antisolar (az-only)
            sun_az = PointingData[i_point].sun_az + 250;  // point solar panels to sun
            NormalizeAngle(&sun_az);

            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_AZEL_GOTO;
            CommandData.pointing_mode.X = sun_az;   // az
            CommandData.pointing_mode.Y = PointingData[i_point].el;   // el
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
            trim_xsc(0);
            break;
        case trim_isc_to_osc:
            trim_xsc(1);
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

        case az_off: // disable az motors
            CommandData.disable_az = 1;
            break;
        case az_on: // enable az motors
            CommandData.disable_az = 0;
            break;
        case el_off:  // disable el motors
            CommandData.disable_el = 1;
            CommandData.force_el = 0;
            break;
        case el_on: // enable el motors
            CommandData.disable_el = 0;
            CommandData.force_el = 0;
            break;
        case force_el_on:  // force enabling of el motors
            CommandData.disable_el = 0;
            CommandData.force_el = 1;
            break;
        case reset_rw:
            rw_reset_fault();
            break;
        case reset_piv:
            piv_reset_fault();
            break;
        case reset_elev:
            el_reset_fault();
            break;
        case restore_piv:
            CommandData.restore_piv = 1;
            break;
        case pss_veto:
            CommandData.use_pss = 0;
            break;
        case xsc0_veto:
            CommandData.use_xsc0 = 0;
            break;
        case xsc1_veto:
            CommandData.use_xsc1 = 0;
            break;
        case mag_veto:
            CommandData.use_mag = 0;
            break;
        case elenc_veto:
            CommandData.use_elenc = 0;
            break;
        case elclin_veto:
            CommandData.use_elclin = 0;
            break;
        case elmotenc_veto:
            CommandData.use_elmotenc = 0;
            break;

        case pss_allow:
            CommandData.use_pss = 1;
            break;
        case xsc0_allow:
            CommandData.use_xsc0 = 1;
            break;
        case xsc1_allow:
            CommandData.use_xsc1 = 1;
            break;
        case mag_allow:
            CommandData.use_mag = 1;
            break;
        case elenc_allow:
            CommandData.use_elenc = 1;
            break;
        case elmotenc_allow:
            CommandData.use_elmotenc = 1;
            break;
        case elclin_allow:
            CommandData.use_elclin = 1;
            break;

        case xsc0_off:
            CommandData.power.xsc0.set_count = 0;
            CommandData.power.xsc0.rst_count = LATCH_PULSE_LEN;
            break;
        case xsc0_on:
            CommandData.power.xsc0.rst_count = 0;
            CommandData.power.xsc0.set_count = LATCH_PULSE_LEN;
            break;
        case xsc0_cycle:
            CommandData.power.xsc0.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
            CommandData.power.xsc0.rst_count = LATCH_PULSE_LEN;
            break;
        case xsc1_off:
            CommandData.power.xsc1.set_count = 0;
            CommandData.power.xsc1.rst_count = LATCH_PULSE_LEN;
            break;
        case xsc1_on:
            CommandData.power.xsc1.rst_count = 0;
            CommandData.power.xsc1.set_count = LATCH_PULSE_LEN;
            break;
        case xsc1_cycle:
            CommandData.power.xsc1.set_count = PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
            CommandData.power.xsc1.rst_count = LATCH_PULSE_LEN;
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
            CommandData.power.piv.set_count = 2 * PCYCLE_HOLD_LEN + LATCH_PULSE_LEN;
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

        case level_on:   // Cryo commanding
            CommandData.Cryo.heliumLevel = -1;
            break;
        case level_off:
            CommandData.Cryo.heliumLevel = 0;
            break;
        case level_pulse:
            CommandData.Cryo.heliumLevel = 350;   // unit is 100Hz frames
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
            CommandData.Cryo.hsCharcoal = 1;
            CommandData.Cryo.fridgeCycle = 0;
            break;
        case hs_charcoal_off:
            CommandData.Cryo.hsCharcoal = 0;
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

            // Lock
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
        case lock45:   // Lock Inner Frame at 45 (to be sent by CSBF pre-termination)
            if (CommandData.pointing_mode.nw >= 0) CommandData.pointing_mode.nw = VETO_MAX;
            CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF;
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_LOCK;
            CommandData.pointing_mode.X = 0;
            CommandData.pointing_mode.Y = LockPosition(45.0);
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.h = 0;
            CommandData.pointing_mode.vaz = 0;
            CommandData.pointing_mode.del = 0;
            blast_info("Commands: Lock at : %g\n", CommandData.pointing_mode.Y);
            break;
        case repoll:
            CommandData.actbus.force_repoll = 1;
            CommandData.hwpr.force_repoll = 1;
#ifdef USE_XY_THREAD
            CommandData.xystage.force_repoll = 1;
#endif
            break;

            // Shutter
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

            // Actuators
        case actuator_stop:
            CommandData.actbus.focus_mode = ACTBUS_FM_PANIC;
            CommandData.actbus.tc_mode = TC_MODE_VETOED;
            break;
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
        case vtx2_isc:
            CommandData.vtx_sel[1] = vtx_isc;
            break;
        case vtx2_osc:
            CommandData.vtx_sel[1] = vtx_osc;
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

        case reap_north:  // Miscellaneous commands
        case reap_south:
            if ((command == reap_north && !SouthIAm) || (command == reap_south && SouthIAm)) {
                blast_err("Commands: Reaping the watchdog tickle on command in 1 second.");
                sleep(1);
                // TODO(seth): Enable Watchdog reap
            }
            break;
        case north_halt:
        case south_halt:
            if ((command == north_halt && !SouthIAm) || (command == south_halt && SouthIAm)) {
                bputs(warning, "Commands: Halting the MCC\n");
                if (system("/sbin/reboot") < 0) berror(fatal, "Commands: failed to reboot, dying\n");
            }
            break;
        case xyzzy:
            break;
        default:
            bputs(warning, "Commands: ***Invalid Single Word Command***\n");
            return;  // invalid command - no write or update
    }

    CommandData.command_count++;
    CommandData.last_command = (uint16_t) command;

#ifndef BOLOTEST
    if (!scheduled) {
        // TODO(seth): RE-enable doing_schedule
//    if (doing_schedule)
//      blast_info("Scheduler: *** Out of schedule file mode ***");
        CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
    } else {
        CommandData.pointing_mode.t = PointingData[i_point].t;
    }
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
  int is_new;

//   Update CommandData struct with new info
//   * If the parameter is type 'i'/'l' set CommandData using ivalues[i]
//   * If the parameter is type 'f'/'d' set CommandData using rvalues[i]


//   Pointing Modes
  switch (command) {
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
      CommandData.pointing_mode.X = rvalues[0];   // az
      CommandData.pointing_mode.Y = rvalues[1];   // el
      CommandData.pointing_mode.vaz = 0.0;
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case az_scan:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_AZ_SCAN;
      CommandData.pointing_mode.X = rvalues[0];   // az
      CommandData.pointing_mode.Y = rvalues[1];   // el
      blast_info("Scan center: %f, %f", CommandData.pointing_mode.X, CommandData.pointing_mode.Y);
      CommandData.pointing_mode.w = rvalues[2];   // width
      CommandData.pointing_mode.vaz = rvalues[3];  // az scan speed
      CommandData.pointing_mode.del = 0.0;
      CommandData.pointing_mode.h = 0;
      break;
    case el_scan:
      //      blast_info("Commands: El scan not enabled yet!");
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_EL_SCAN;
      CommandData.pointing_mode.X = rvalues[0];   // az
      CommandData.pointing_mode.Y = rvalues[1];   // el
      //      blast_info("Scan center: %f, %f", CommandData.pointing_mode.X, CommandData.pointing_mode.Y);
      CommandData.pointing_mode.h = rvalues[2];   // height
      CommandData.pointing_mode.vel = rvalues[3];  // az scan speed
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
      CommandData.pointing_mode.vaz = rvalues[0];  // az speed
      CommandData.pointing_mode.del = rvalues[1];  // el speed
      CommandData.pointing_mode.h = 0;
      break;
    case ra_dec_goto:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_RADEC_GOTO;
      CommandData.pointing_mode.X = rvalues[0];  // ra
      CommandData.pointing_mode.Y = rvalues[1];  // dec
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      CommandData.pointing_mode.h = 0;
      break;
    case vcap:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_VCAP;
      CommandData.pointing_mode.X = rvalues[0];  // ra
      CommandData.pointing_mode.Y = rvalues[1];  // dec
      CommandData.pointing_mode.w = rvalues[2];  // radius
      CommandData.pointing_mode.vaz = rvalues[3];  // az scan speed
      CommandData.pointing_mode.del = rvalues[4];  // el drift speed
      CommandData.pointing_mode.h = 0;
      break;
    case cur_mode:
      CommandData.pointing_mode.mode = P_CURRENT;
      CommandData.pointing_mode.X = rvalues[0];  // pivot current
      CommandData.pointing_mode.Y = rvalues[1];  // rw current
      CommandData.pointing_mode.w = rvalues[2];  // el current
      break;
    case cap:

      if ((CommandData.pointing_mode.mode != P_CAP) ||
          (CommandData.pointing_mode.X != rvalues[0]) ||   // ra
          (CommandData.pointing_mode.Y != rvalues[1]) ||   // dec
          (CommandData.pointing_mode.w != rvalues[2]) ||   // radius
          (CommandData.pointing_mode.vaz != rvalues[3]) ||   // az scan speed
          (CommandData.pointing_mode.del != rvalues[4]) ||   // el step size
          (CommandData.pointing_mode.h != 0))  {  // N dither steps
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }
      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }


      CommandData.pointing_mode.mode = P_CAP;
      CommandData.pointing_mode.X = rvalues[0];  // ra
      CommandData.pointing_mode.Y = rvalues[1];  // dec
      CommandData.pointing_mode.w = rvalues[2];  // radius
      CommandData.pointing_mode.vaz = rvalues[3];  // az scan speed
      CommandData.pointing_mode.del = rvalues[4];  // el step size
      CommandData.pointing_mode.h = 0;
      CommandData.pointing_mode.n_dith = ivalues[5];  // No of dither steps
      break;
    case box:

      if ((CommandData.pointing_mode.mode != P_BOX) ||
          (CommandData.pointing_mode.X != rvalues[0]) ||  // ra
          (CommandData.pointing_mode.Y != rvalues[1]) ||  // dec
          (CommandData.pointing_mode.w != rvalues[2]) ||  // width
          (CommandData.pointing_mode.h != rvalues[3]) ||  // height
          (CommandData.pointing_mode.vaz != rvalues[4]) ||  // az scan speed
          (CommandData.pointing_mode.del != rvalues[5])) {  // el step size
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }

      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }

      CommandData.pointing_mode.mode = P_BOX;
      CommandData.pointing_mode.X = rvalues[0];  // ra
      CommandData.pointing_mode.Y = rvalues[1];  // dec
      CommandData.pointing_mode.w = rvalues[2];  // width
      CommandData.pointing_mode.h = rvalues[3];  // height
      CommandData.pointing_mode.vaz = rvalues[4];  // az scan speed
      CommandData.pointing_mode.del = rvalues[5];  // el step size
      CommandData.pointing_mode.n_dith = ivalues[6];  // number of el dither steps
      break;
    case el_box:

      if ((CommandData.pointing_mode.mode != P_EL_BOX) ||
          (CommandData.pointing_mode.X != rvalues[0]) ||  // ra
          (CommandData.pointing_mode.Y != rvalues[1]) ||  // dec
          (CommandData.pointing_mode.w != rvalues[2]) ||  // width
          (CommandData.pointing_mode.h != rvalues[3]) ||  // height
          (CommandData.pointing_mode.vel != rvalues[4]) ||  // az scan speed
          (CommandData.pointing_mode.daz != rvalues[5])) {  // el step size
        CommandData.pointing_mode.nw = CommandData.slew_veto;
      }

      // zero unused parameters
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = 0;
        CommandData.pointing_mode.dec[i] = 0;
      }

      CommandData.pointing_mode.mode = P_EL_BOX;
      CommandData.pointing_mode.X = rvalues[0];  // ra
      CommandData.pointing_mode.Y = rvalues[1];  // dec
      CommandData.pointing_mode.w = rvalues[2];  // width
      CommandData.pointing_mode.h = rvalues[3];  // height
      CommandData.pointing_mode.vel = rvalues[4];  // az scan speed
      CommandData.pointing_mode.daz = rvalues[5];  // el step size
      CommandData.pointing_mode.n_dith = ivalues[6];  // number of el dither steps

      break;
    case vbox:
      CommandData.pointing_mode.nw = CommandData.slew_veto;
      CommandData.pointing_mode.mode = P_VBOX;
      CommandData.pointing_mode.X = rvalues[0];  // ra
      CommandData.pointing_mode.Y = rvalues[1];  // dec
      CommandData.pointing_mode.w = rvalues[2];  // width
      CommandData.pointing_mode.h = rvalues[3];  // height
      CommandData.pointing_mode.vaz = rvalues[4];  // az scan speed
      CommandData.pointing_mode.del = rvalues[5];  // el drift speed
      break;
    case quad:
      is_new = 0;
      if ((CommandData.pointing_mode.mode != P_QUAD) ||
          (CommandData.pointing_mode.vaz != rvalues[8]) ||  // az scan speed
          (CommandData.pointing_mode.del != rvalues[9])) { // el step size
                is_new = 1;
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
      CommandData.pointing_mode.X = 0;  // ra
      CommandData.pointing_mode.Y = 0;  // dec
      CommandData.pointing_mode.w = 0;  // width
      CommandData.pointing_mode.h = 0;  // height

      CommandData.pointing_mode.mode = P_QUAD;
      for (i = 0; i < 4; i++) {
        CommandData.pointing_mode.ra[i] = rvalues[i * 2];
        CommandData.pointing_mode.dec[i] = rvalues[i * 2 + 1];
      }
      CommandData.pointing_mode.vaz = rvalues[8];  // az scan speed
      CommandData.pointing_mode.del = rvalues[9];  // el step size
      CommandData.pointing_mode.n_dith = ivalues[10];  // N dither steps
      break;
    case az_scan_accel:
      CommandData.az_accel = rvalues[0];
      if (CommandData.az_accel < 0.005) {
	blast_warn("Attempt to set az_accel to %f, that is too low! Setting %f instead", rvalues[0], CommandData.az_accel);
      }
      break;
    case set_scan_params:
      CommandData.pointing_mode.next_i_hwpr = ivalues[0];
      CommandData.pointing_mode.next_i_dith = ivalues[1];
      break;

      /*************************************
      ********* Pointing Sensor Trims ******/
    case az_el_trim:
      AzElTrim(rvalues[0], rvalues[1]);
      break;
    case ra_dec_set:
      SetRaDec(rvalues[0], rvalues[1]);
      break;
    case pos_set:
      set_position(rvalues[0], rvalues[1]);
      break;
    case autotrim_to_sc:
      CommandData.autotrim_thresh = rvalues[0];
      CommandData.autotrim_time = ivalues[1];
      CommandData.autotrim_rate = rvalues[2];
      CommandData.autotrim_xsc0_last_bad = mcp_systime(NULL);
      CommandData.autotrim_osc_last_bad = CommandData.autotrim_xsc0_last_bad;
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
            blast_info("CommandData.slew_veto = %i, CommandData.pointing_mode.nw = %i",
                       CommandData.slew_veto, CommandData.pointing_mode.nw);
      if (CommandData.pointing_mode.nw > CommandData.slew_veto) CommandData.pointing_mode.nw = CommandData.slew_veto;
      break;
    case mag_cal:
      CommandData.cal_xmax_mag = ivalues[0];
      CommandData.cal_xmin_mag = ivalues[1];
      CommandData.cal_ymax_mag = ivalues[2];
      CommandData.cal_ymin_mag = ivalues[3];
      break;

    case pss_cal:
      CommandData.cal_off_pss1 = rvalues[0];
      CommandData.cal_d_pss1 = rvalues[1];
      CommandData.cal_off_pss2 = rvalues[2];
      CommandData.cal_d_pss2 = rvalues[3];
      CommandData.cal_off_pss3 = rvalues[4];
      CommandData.cal_d_pss3 = rvalues[5];
      CommandData.cal_off_pss4 = rvalues[6];
      CommandData.cal_d_pss4 = rvalues[7];
      CommandData.cal_imin_pss = rvalues[8];
      break;


    /*************************************
     ********* Pointing Motor Gains ******/
    case el_gain:   // ele gains
      CommandData.ele_gain.P = rvalues[0];
      if (rvalues[1] <= 0.0005) {
          blast_err("You tried to set the Elevation Motor time constant to less than 0.5ms!"
                  "  This is invalid, so we will assume you wanted a really long time.");
          CommandData.ele_gain.I = 1000.0;
      } else {
          CommandData.ele_gain.I = rvalues[1];
      }
      CommandData.ele_gain.D = rvalues[2];
      CommandData.ele_gain.PT = rvalues[3];
      CommandData.ele_gain.DB = rvalues[4];
      CommandData.ele_gain.F = rvalues[5];
      break;
    case az_gain:   // az gains
      CommandData.azi_gain.P = rvalues[0];
      if (rvalues[1] <= 0.0005) {
            blast_err("You tried to set the Azimuth Motor time constant to less than 0.5ms!"
                    "  This is invalid, so we will assume you wanted a really long time.");
            CommandData.azi_gain.I = 1000.0;
        } else {
            CommandData.azi_gain.I = rvalues[1];
        }
      CommandData.azi_gain.D = rvalues[2];
      CommandData.azi_gain.PT = rvalues[3];
      break;
    case pivot_gain:   // pivot gains
      CommandData.pivot_gain.SP = rvalues[0];
      CommandData.pivot_gain.PE = rvalues[1];
      CommandData.pivot_gain.PV = rvalues[2];
      CommandData.pivot_gain.IV = rvalues[3];
      CommandData.pivot_gain.F = rvalues[4];
      break;

     /*************************************
      ****           test of motor DACs ***/

    case motors_verbose:
      CommandData.verbose_rw = ivalues[0];
      CommandData.verbose_el = ivalues[1];
      CommandData.verbose_piv = ivalues[2];
      break;
#endif

     /*************************************
      ********* Lock / Actuators  *********/
    case lock:   // Lock Inner Frame
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
      blast_info("Commands: Lock Mode: %g\n", CommandData.pointing_mode.Y);
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
    case general:  // General actuator bus command
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
      CommandData.actbus.tc_wait = ivalues[3] * 300;  // convert min->5Hz
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

#ifndef BOLOTEST
      /*************************************
      ********* Balance System  ***********/
    case balance_gain:
      CommandData.balance.i_el_on_bal = rvalues[0];
      CommandData.balance.i_el_off_bal = rvalues[1];
//      CommandData.balance.i_el_target_bal = rvalues[2];
//      CommandData.balance.gain_bal = rvalues[3];
      break;
    case balance_manual:
      CommandData.balance.bal_move_type = rvalues[0];
      CommandData.balance.mode = bal_manual;
      break;
    case balance_vel:
      CommandData.balance.vel = ivalues[0];
      CommandData.balance.acc = ivalues[1];
      break;
    case balance_i:
      CommandData.balance.move_i = ivalues[0];
      CommandData.balance.hold_i = ivalues[1];
      break;



      /*************************************
      ************** Misc  ****************/
//    case gyro_off:
//      CommandData.power.gyro_off[ivalues[0]-1] |= 0x01;
//      break;
//    case gyro_on:
//      CommandData.power.gyro_off[ivalues[0]-1] &= ~0x01;
//      break;
    case timeout:        // Set timeout
      CommandData.timeout = rvalues[0];
      break;
    case tdrss_bw:
      CommandData.tdrss_bw = rvalues[0];
      break;
    case iridium_bw:
      CommandData.iridium_bw = rvalues[0];
      break;
    case slot_sched:  // change uplinked schedule file
        // TODO(seth): Re-enable Uplink file loading
//      if (LoadUplinkFile(ivalues[0])) {
//        CommandData.uplink_sched = 1;
//        CommandData.slot_sched = ivalues[0];
//      }
      break;
    case params_test: // Do nothing, with lots of parameters
      blast_info("Integer params 'i': %d 'l' %d", ivalues[0], ivalues[1]);
      blast_info("Float params 'f': %g 'd' %g", rvalues[2], rvalues[3]);
      blast_info("String param 's': %s", svalues[4]);
      CommandData.plover = ivalues[0];
      break;
    case plugh: // A hollow voice says "Plugh".
      CommandData.plover = ivalues[0];
      break;
#endif

      /*************************************
      ************** Bias  ****************/
//       used to be multiplied by 2 here, but screw up prev_satus
//       need to multiply later instead
    case bias_level_500:     // Set bias 1 (500)
      CommandData.Bias.bias[0] = ivalues[0];
      CommandData.Bias.setLevel[0] = 1;
      break;
    case bias_level_350:     // Set bias 2 (350)
      CommandData.Bias.bias[1] = ivalues[0];
      CommandData.Bias.setLevel[1] = 1;
      break;
    case bias_level_250:     // Set bias 3 (250)
      CommandData.Bias.bias[2] = ivalues[0];
      CommandData.Bias.setLevel[2] = 1;
      break;
    case bias_level_rox:     // Set bias 4 (ROX)
      CommandData.Bias.bias[3] = ivalues[0];
      CommandData.Bias.setLevel[3] = 1;
      break;
    case bias_level_x:     // Set bias 5 (spare)
      CommandData.Bias.bias[4] = ivalues[0];
      CommandData.Bias.setLevel[4] = 1;
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
      /*************************************
      ********** Cal Lamp  ****************/
    case cal_pulse:
      CommandData.Cryo.calibrator = pulse;
      CommandData.Cryo.calib_pulse = ivalues[0] / 10;
      break;
    case cal_repeat:
      CommandData.Cryo.calibrator = repeat;
      CommandData.Cryo.calib_pulse = ivalues[0] / 10;
      CommandData.Cryo.calib_period = ivalues[1]*5;
      CommandData.Cryo.calib_hwpr = ivalues[2];
      break;

      /*************************************
      ******** Cryo heat   ****************/
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
      /********* XSC Commanding  *************/
        case xsc_is_new_window_period:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].is_new_window_period_cs = ivalues[1];
                }
            }
            break;
        }
        case xsc_offset:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].cross_el_trim = from_degrees(rvalues[1]);
                    CommandData.XSC[which].el_trim = from_degrees(rvalues[2]);
                }
            }
            break;
        }
        case xsc_heaters_off:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].heaters.mode = xsc_heater_off;
                }
            }
            break;
        }
        case xsc_heaters_on:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].heaters.mode = xsc_heater_on;
                }
            }
            break;
        }
        case xsc_heaters_auto:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].heaters.mode = xsc_heater_auto;
                    CommandData.XSC[which].heaters.setpoint = rvalues[1];
                }
            }
            break;
        }
        case xsc_exposure_timing:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].trigger.exposure_time_cs = ivalues[1];
                }
                CommandData.XSC[which].trigger.grace_period_cs = rvalues[2] * 100.0;
                CommandData.XSC[which].trigger.post_trigger_counter_mcp_share_delay_cs = ivalues[3];
            }
            break;
        }
        case xsc_multi_trigger:
        {
            for (unsigned int which = 0; which < 2; which++) {
                CommandData.XSC[which].trigger.num_triggers = ivalues[1];
                CommandData.XSC[which].trigger.multi_trigger_time_between_triggers_cs = ivalues[2];
                xsc_activate_command(which, xC_multi_triggering);
            }
            break;
        }
        case xsc_trigger_threshold:
        {
            int which = 0;
            for (which = 0; which < 2; which++) {
                CommandData.XSC[which].trigger.threshold.enabled = (ivalues[1] != 0);
                CommandData.XSC[which].trigger.threshold.blob_streaking_px = rvalues[2];
            }
            break;
        }
        case xsc_scan_force_trigger:
        {
            int which = 0;
            for (which = 0; which < 2; which++) {
                CommandData.XSC[which].trigger.scan_force_trigger_enabled = (ivalues[1] != 0);
            }
            break;
        }
        case xsc_quit:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_quit);
                }
            }
            break;
        }
        case xsc_shutdown:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.shutdown.shutdown_now = true;
                    CommandData.XSC[which].net.shutdown.include_restart = (ivalues[1] != 0);
                    xsc_activate_command(which, xC_shutdown);
                }
            }
            break;
        }
        case xsc_network_reset:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.network_reset.reset_now = (ivalues[1] != 0);
                    CommandData.XSC[which].net.network_reset.reset_on_lull_enabled = (ivalues[2] != 0);
                    CommandData.XSC[which].net.network_reset.reset_on_lull_delay = rvalues[3];
                    xsc_activate_command(which, xC_network_reset);
                }
            }
            break;
        }
        case xsc_main_settings:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.main_settings.display_frequency = rvalues[1];
                    CommandData.XSC[which].net.main_settings.display_fullscreen = (ivalues[2] != 0);
                    CommandData.XSC[which].net.main_settings.display_image_only = (ivalues[3] != 0);
                    CommandData.XSC[which].net.main_settings.display_solving_filters = (ivalues[4] != 0);
                    CommandData.XSC[which].net.main_settings.display_image_brightness = rvalues[5];
                    xsc_activate_command(which, xC_main_settings);
                }
            }
            break;
        }
        case xsc_display_zoom:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.main_settings.display_zoom_x = ivalues[1];
                    CommandData.XSC[which].net.main_settings.display_zoom_y = ivalues[2];
                    CommandData.XSC[which].net.main_settings.display_zoom_magnitude = rvalues[3];
                    xsc_activate_command(which, xC_display_zoom);
                }
            }
            break;
        }
        case xsc_image_client:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.image_client_enabled = (ivalues[1] != 0);
                    xsc_activate_command(which, xC_image_client);
                }
            }
            break;
        }
        case xsc_init_focus:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_init_focus);
                }
            }
            break;
        }
        case xsc_set_focus:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.set_focus_value = ivalues[1];
                    xsc_activate_command(which, xC_set_focus);
                }
            }
            break;
        }
        case xsc_set_focus_incremental:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.set_focus_incremental_value = ivalues[1];
                    xsc_activate_command(which, xC_set_focus_incremental);
                }
            }
            break;
        }
        case xsc_run_autofocus:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_run_autofocus);
                }
            }
            break;
        }
        case xsc_set_autofocus_range:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.autofocus_search_min = ivalues[1];
                    CommandData.XSC[which].net.autofocus_search_max = ivalues[2];
                    CommandData.XSC[which].net.autofocus_search_step = ivalues[3];
                    xsc_activate_command(which, xC_set_autofocus_range);
                }
            }
            break;
        }
        case xsc_abort_autofocus:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.abort_autofocus_still_use_solution = (ivalues[1] != 0);
                    xsc_activate_command(which, xC_abort_autofocus);
                }
            }
            break;
        }
        case xsc_autofocus_display_mode:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    if (ivalues[1] >= xC_autofocus_display_mode_auto && ivalues[1] <= xC_autofocus_display_mode_off) {
                        CommandData.XSC[which].net.autofocus_display_mode = ivalues[1];
                        xsc_activate_command(which, xC_autofocus_display_mode);
                    } else {
                        blast_warn("warning: command xsc_autofocus_display_mode: display mode out of range");
                    }
                }
            }
            break;
        }
        case xsc_init_aperture:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_init_aperture);
                }
            }
            break;
        }
        case xsc_set_aperture:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.set_aperture_value = ivalues[1];
                    xsc_activate_command(which, xC_set_aperture);
                }
            }
            break;
        }
        case xsc_get_gain:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_get_gain);
                }
            }
            break;
        }
        case xsc_set_gain:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.set_gain_value = rvalues[1];
                    xsc_activate_command(which, xC_set_gain);
                }
            }
            break;
        }
        case xsc_fake_sky_brightness:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.brightness.counter++;
                    CommandData.XSC[which].net.brightness.enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].net.brightness.level_kepsa = rvalues[2];
                    CommandData.XSC[which].net.brightness.gain_db = rvalues[3];
                    CommandData.XSC[which].net.brightness.actual_exposure = rvalues[4];
                    CommandData.XSC[which].net.brightness.simulated_exposure = rvalues[5];
                    xsc_activate_command(which, xC_brightness);
                }
            }
            break;
        }
        case xsc_solver_general:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].net.solver.timeout = rvalues[2];
                    xsc_activate_command(which, xC_solver_general);
                }
            }
            break;
        }
        case xsc_solver_abort:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_solver_abort);
                }
            }
            break;
        }
        case xsc_selective_mask:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.mask.enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].net.solver.mask.field0 = (unsigned int) ivalues[2];
                    CommandData.XSC[which].net.solver.mask.field1 = (unsigned int) ivalues[3];
                    CommandData.XSC[which].net.solver.mask.field2 = (unsigned int) ivalues[4];
                    xsc_activate_command(which, xC_solver_mask);
                }
            }
            break;
        }
        case xsc_blob_finding:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.snr_threshold = rvalues[1];
                    CommandData.XSC[which].net.solver.max_num_blobs = ivalues[2];
                    CommandData.XSC[which].net.solver.robust_mode_enabled = (ivalues[3] != 0);
                    if (ivalues[4] >= xC_solver_fitting_method_none && ivalues[4]
                            <= xC_solver_fitting_method_double_gaussian) {
                        CommandData.XSC[which].net.solver.fitting_method = ivalues[4];
                    } else {
                        blast_warn(
                                "warning: command xsc_blob_finder: fitting_method out of range.  Defaulting to 'none'");
                        CommandData.XSC[which].net.solver.fitting_method = xC_solver_fitting_method_none;
                    }
                    xsc_activate_command(which, xC_solver_blob_finder);
                }
            }
            break;
        }
        case xsc_blob_cells:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.cell_size = pow(2, ivalues[1]);
                    ;
                    CommandData.XSC[which].net.solver.max_num_blobs_per_cell = ivalues[2];
                    ;
                    xsc_activate_command(which, xC_solver_blob_cells);
                }
            }
            break;
        }
        case xsc_pattern_matching:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.pattern_matcher_enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].net.solver.display_star_names = (ivalues[2] != 0);
                    CommandData.XSC[which].net.solver.match_tolerance_px = rvalues[3];
                    CommandData.XSC[which].net.solver.iplatescale_min = from_arcsec(rvalues[4]);
                    CommandData.XSC[which].net.solver.iplatescale_max = from_arcsec(rvalues[5]);
                    CommandData.XSC[which].net.solver.platescale_always_fixed = (ivalues[6] != 0);
                    CommandData.XSC[which].net.solver.iplatescale_fixed = from_arcsec(rvalues[7]);
                    xsc_activate_command(which, xC_solver_pattern_matcher);
                }
            }
            break;
        }

        case xsc_filter_hor_location:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.filters.hor_location_limit_enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].net.solver.filters.hor_location_limit_radius = from_degrees(rvalues[2]);
                    xsc_activate_command(which, xC_solver_filter_hor_location);
                }
            }
            break;
        }

        case xsc_filter_hor_roll:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.filters.hor_roll_limit_enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].net.solver.filters.hor_roll_limit_min = from_degrees(rvalues[2]);
                    CommandData.XSC[which].net.solver.filters.hor_roll_limit_max = from_degrees(rvalues[3]);
                    xsc_activate_command(which, xC_solver_filter_hor_roll);
                }
            }
            break;
        }

        case xsc_filter_el:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.filters.hor_el_limit_enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].net.solver.filters.hor_el_limit_min = from_degrees(rvalues[2]);
                    CommandData.XSC[which].net.solver.filters.hor_el_limit_max = from_degrees(rvalues[3]);
                    xsc_activate_command(which, xC_solver_filter_hor_el);
                }
            }
            break;
        }

        case xsc_filter_eq_location:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.filters.eq_location_limit_enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].net.solver.filters.eq_location_limit_radius = from_degrees(rvalues[2]);
                    xsc_activate_command(which, xC_solver_filter_eq_location);
                }
            }
            break;
        }

        case xsc_filter_matching:
        {
            for (int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.solver.filters.matching_pointing_error_threshold = from_arcsec(
                            rvalues[1]);
                    CommandData.XSC[which].net.solver.filters.matching_fit_error_threshold_px = rvalues[2];
                    CommandData.XSC[which].net.solver.filters.matching_num_matched = (unsigned int) ivalues[3];
                    xsc_activate_command(which, xC_solver_filter_matching);
                }
            }
            break;
        }

#endif
    default:
      bputs(warning, "Commands: ***Invalid Multi Word Command***\n");
      return;  // invalid command - don't update
  }

  CommandData.command_count++;
  // set high bit to differentiate multi-commands from single
  CommandData.last_command = (uint16_t)command | 0x8000;

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
    int fp, n_read = 0, junk, extra = 0;
    int is_valid = 0;
    uint32_t prev_crc;

    if ((fp = open(PREV_STATUS_FILE, O_RDONLY)) < 0) {
        berror(err, "Commands: Unable to open prev_status file for reading");
    } else {
        if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) berror(
                err, "Commands: prev_status read()");
        if ((extra = read(fp, &junk, sizeof(junk))) < 0) berror(err, "Commands: extra prev_status read()");
        if (close(fp) < 0) berror(err, "Commands: prev_status close()");
    }
    prev_crc = CommandData.checksum;
    CommandData.checksum = 0;
    is_valid = (prev_crc == crc32_le(0, (uint8_t*)&CommandData, sizeof(CommandData)));

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

    // forces reload of saved bias values
    CommandData.Bias.setLevel[0] = 1;
    CommandData.Bias.setLevel[1] = 1;
    CommandData.Bias.setLevel[2] = 1;
    CommandData.Bias.setLevel[3] = 1;
    CommandData.Bias.setLevel[4] = 1;

    CommandData.power.sc_tx.rst_count = 0;
    CommandData.power.sc_tx.set_count = 0;
    CommandData.power.das.rst_count = 0;
    CommandData.power.das.set_count = 0;
    CommandData.power.xsc0.rst_count = 0;
    CommandData.power.xsc0.set_count = 0;
    CommandData.power.xsc1.rst_count = 0;
    CommandData.power.xsc1.set_count = 0;
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

    CommandData.Cryo.BDAHeat = 0;

    CommandData.Cryo.potvalve_on = 0;
    CommandData.Cryo.potvalve_open = 0;
    CommandData.Cryo.potvalve_close = 0;
    CommandData.Cryo.lhevalve_on = 0;
    CommandData.Cryo.lvalve_open = 0;
    CommandData.Cryo.lvalve_close = 0;
    CommandData.Cryo.lnvalve_on = 0;

    CommandData.uei_command.uei_of_dio_432_out = 0;
    /* don't use the fast gy offset calculator */
    CommandData.fast_offset_gy = 0;

    /* force autotrim to reset its wait time on restart */
    CommandData.autotrim_xsc0_last_bad = mcp_systime(NULL);
    CommandData.autotrim_osc_last_bad = CommandData.autotrim_xsc0_last_bad;

    CommandData.reset_rw = 0;
    CommandData.reset_piv = 0;
    CommandData.reset_elev = 0;
    CommandData.restore_piv = 0;

    CommandData.slot_sched = 0x100;
    CommandData.parts_sched = 0x0;

    /* return if we successfully read the previous status */
    if (n_read != sizeof(struct CommandDataStruct))
        blast_warn("Commands: prev_status: Wanted %i bytes but got %i.\n",
                   (int) sizeof(struct CommandDataStruct), n_read);
    else if (extra > 0)
        bputs(warning, "Commands: prev_status: Extra bytes found.\n");
    else if (!is_valid)
        blast_warn("Invalid Checksum on saved data.  Reverting to defaults!");
    else
        return;

    bputs(warning, "Commands: Regenerating Command Data and prev_status\n");

    /* prev_status overrides this stuff */
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

    CommandData.ele_gain.I = 200;
    CommandData.ele_gain.P = 23.9;
    CommandData.ele_gain.D = 0;
    CommandData.ele_gain.PT = 200;
    CommandData.ele_gain.DB = 0;
    CommandData.ele_gain.F = 0;

    CommandData.azi_gain.P = 200;
    CommandData.azi_gain.I = 200;
    CommandData.azi_gain.PT = 200;

    CommandData.pivot_gain.SP = 30; // dps
    CommandData.pivot_gain.PV = 400;
    CommandData.pivot_gain.IV = 10000;
    CommandData.pivot_gain.PE = 0;
    CommandData.pivot_gain.F = 0.3;

    // /TODO: Re-enable El prior to flight
    CommandData.disable_az = 1;
    CommandData.disable_el = 1;

    CommandData.verbose_rw = 0;
    CommandData.verbose_el = 0;
    CommandData.verbose_piv = 0;

    CommandData.use_elenc = 1;
    CommandData.use_elclin = 1;
    CommandData.use_pss = 1;
    CommandData.use_xsc0 = 1;
    CommandData.use_xsc1 = 1;
    CommandData.use_mag = 1;
    CommandData.lat_range = 1;
    CommandData.sucks = 1;
    CommandData.uplink_sched = 0;

    CommandData.clin_el_trim = 0;
    CommandData.enc_el_trim = 0;
    CommandData.enc_motor_el_trim = 0;
    CommandData.null_az_trim = 0;
    CommandData.mag_az_trim = 0;
    CommandData.pss_az_trim = 0;

    CommandData.autotrim_enable = 0;
    CommandData.autotrim_thresh = 0.05;
    CommandData.autotrim_rate = 1.0;
    CommandData.autotrim_time = 60;

    CommandData.cal_xmax_mag = 41587;
    CommandData.cal_ymax_mag = 41300;
    CommandData.cal_xmin_mag = 40659;
    CommandData.cal_ymin_mag = 40650;

    CommandData.cal_off_pss1 = 0.0;
    CommandData.cal_off_pss2 = 2.7997;
    CommandData.cal_off_pss3 = 4.9994;
    CommandData.cal_off_pss4 = 10.3497;

    CommandData.cal_d_pss1 = 0.0;
    CommandData.cal_d_pss2 = 0.0;
    CommandData.cal_d_pss3 = 0.0;
    CommandData.cal_d_pss4 = 0.0;

    CommandData.cal_imin_pss = 4.5;

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

    CommandData.balance.i_el_on_bal = 2.5;
    CommandData.balance.i_el_off_bal = 1.0;
    CommandData.balance.mode = bal_rest;

    CommandData.Bias.bias[0] = 12470;   // 500um
    CommandData.Bias.bias[1] = 11690;   // 350um
    CommandData.Bias.bias[2] = 13940;   // 250um
    CommandData.Bias.bias[3] = 1050;   // ROX
    CommandData.Bias.bias[4] = 16384;  // X

    CommandData.actbus.tc_mode = TC_MODE_VETOED;
    CommandData.actbus.tc_step = 100; /* microns */
    CommandData.actbus.tc_wait = 3000; /* = 10 minutes in 5-Hz frames */
    CommandData.actbus.tc_spread = 5; /* centigrade degrees */
    CommandData.actbus.tc_prefp = 1;
    CommandData.actbus.tc_prefs = 1;

    CommandData.actbus.lvdt_delta = 1000;
    CommandData.actbus.lvdt_low = 30000;
    CommandData.actbus.lvdt_high = 50000;

    CommandData.actbus.offset[0] = 40000;
    CommandData.actbus.offset[1] = 40000;
    CommandData.actbus.offset[2] = 40000;

    /* The first is due to change in radius of curvature, the second due to
     * displacement of the secondary due to the rigid struts */

    /* Don sez:   50.23 + 9.9 and 13.85 - 2.2 */
    /* Marco sez: 56          and 10          */

    CommandData.actbus.g_primary = 56; /* um/deg */
    CommandData.actbus.g_secondary = 10; /* um/deg */
    CommandData.actbus.focus = 0;
    CommandData.actbus.sf_time = 0;
    CommandData.actbus.sf_offset = 6667;

    CommandData.actbus.act_vel = 20;
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
    CommandData.hwpr.hold_i = 0;

    CommandData.balance.vel = 1600;
    CommandData.balance.acc = 4;
    CommandData.balance.move_i = 20;
    CommandData.balance.hold_i = 0;

    /* hwpr positions separated by 22.5 degs.
     entered by Barth on December 25, 2012 */
    CommandData.hwpr.pos[3] = 0.3418;
    CommandData.hwpr.pos[2] = 0.2168;
    CommandData.hwpr.pos[1] = 0.2779;
    CommandData.hwpr.pos[0] = 0.4047;

    CommandData.hwpr.overshoot = 300;
    CommandData.hwpr.i_pos = 0;
    CommandData.hwpr.no_step = 0;
    CommandData.hwpr.use_pot = 1;
    CommandData.hwpr.pot_targ = 0.5;

    CommandData.pin_is_in = 1;

    CommandData.Cryo.charcoalHeater = 0;
    CommandData.Cryo.hsCharcoal = 1;
    CommandData.Cryo.fridgeCycle = 1;
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
    CommandData.Cryo.calib_repeats = -1;  // indefinitely
    CommandData.Cryo.calib_hwpr = 1;  // pulse after every hwpr step

    CommandData.Cryo.cycle_start_temp = 0.375;
    CommandData.Cryo.cycle_pot_max = 2.5;
    CommandData.Cryo.cycle_charcoal_max = 40.0;
    CommandData.Cryo.cycle_charcoal_timeout = 30.0;
    CommandData.Cryo.cycle_charcoal_settle = 25.0;
    CommandData.Cryo.cycle_settle_timeout = 40.0;

    CommandData.ISCControl[0].max_age = 200; /* 2000 ms*/

    CommandData.ISCControl[0].autofocus = 0;
    CommandData.ISCControl[0].save_period = 12000; /* 120 sec */
    CommandData.ISCControl[0].pulse_width = 18; /* 180.00 msec */
    CommandData.ISCControl[0].fast_pulse_width = 8; /* 80.00 msec */

    CommandData.ISCControl[1].max_age = 200; /* 2000 ms*/

    CommandData.ISCControl[1].autofocus = 0;
    CommandData.ISCControl[1].save_period = 12000; /* 120 sec */
    CommandData.ISCControl[1].pulse_width = 18; /* 180.00 msec */
    CommandData.ISCControl[1].fast_pulse_width = 8; /* 80.00 msec */

    for (int which = 0; which < 2; which++) {
        CommandData.XSC[which].heaters.mode = xsc_heater_auto;
        CommandData.XSC[which].heaters.setpoint = 10.0;

        CommandData.XSC[which].trigger.exposure_time_cs = 12;
        CommandData.XSC[which].trigger.grace_period_cs = 4500;
        CommandData.XSC[which].trigger.post_trigger_counter_mcp_share_delay_cs = 200;

        CommandData.XSC[which].trigger.num_triggers = 1;
        CommandData.XSC[which].trigger.multi_trigger_time_between_triggers_cs = 18;

        CommandData.XSC[which].trigger.threshold.enabled = true;
        CommandData.XSC[which].trigger.threshold.blob_streaking_px = 2.0;

        CommandData.XSC[which].trigger.scan_force_trigger_enabled = true;
        CommandData.XSC[which].el_trim = 0.0;
        CommandData.XSC[which].cross_el_trim = 0.0;

        xsc_clear_client_data(&CommandData.XSC[which].net);
    }

    CommandData.temp1 = 0;
    CommandData.temp2 = 0;
    CommandData.temp3 = 0;
    CommandData.df = 0;

    CommandData.lat = -77.86;  // McMurdo Building 096
    CommandData.lon = -167.04; // Willy Field Dec 2010

    WritePrevStatus();
}
