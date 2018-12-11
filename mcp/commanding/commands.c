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
#include <ec_motors.h>

#include <linklist.h>
#include <linklist_compress.h>

#include "command_list.h"
#include "command_struct.h"
#include "framing.h"
#include "mcp.h"
#include "tx.h"
#include "pointing_struct.h"
#include "channels_tng.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "cryostat.h"
#include "relay_control.h"
#include "bias_tone.h"
#include "sip.h"

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
// #define LOCK_OFFSET (-0.77) /* Updated by LMF on July 12th, 2012 */
#define LOCK_OFFSET (0.0)
#define NUM_LOCK_POS 10
static const double lock_positions[NUM_LOCK_POS] = {0.03, 5.01, 14.95, 24.92, 34.88, 44.86, 54.83, 64.81, 74.80, 89.78};

/* based on xsc0.h */
#define ISC_SHUTDOWN_NONE     0
#define ISC_SHUTDOWN_HALT     1
#define ISC_SHUTDOWN_REBOOT   2
#define ISC_SHUTDOWN_CAMCYCLE 3

#define ISC_TRIGGER_INT  0
#define ISC_TRIGGER_EDGE 1
#define ISC_TRIGGER_POS  2
#define ISC_TRIGGER_NEG  3

#define PSN_EAST_BAY_LAT 31.779300
#define PSN_EAST_BAY_LON 264.283000

void RecalcOffset(double, double);  /* actuators.c */

/* defined in pointing.c */
void SetTrimToSC(int);
void ClearTrim();
void AzElTrim(double, double);
void NormalizeAngle(double*);

int LoadUplinkFile(int slot); /*sched.c */

extern int doing_schedule; /* sched.c */

extern linklist_t * linklist_array[MAX_NUM_LINKLIST_FILES];
extern linklist_t * telemetries_linklist[NUM_TELEMETRIES];
extern char * ROACH_TYPES[NUM_RTYPES];

extern int16_t SouthIAm;
pthread_mutex_t mutex;

struct SIPDataStruct SIPData = {.GPSpos = {.lat = PSN_EAST_BAY_LAT, .lon = PSN_EAST_BAY_LON}};
struct CommandDataStruct CommandData;

const char* SName(enum singleCommand command); // share/sip.c
char * linklist_nt[64] = {NULL};

/** Write the Previous Status: called whenever anything changes */
void WritePrevStatus()
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

  // framing_publish_command_data(&CommandData);
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
        case vtx_xsc1:
            CommandData.vtx_sel[0] = VTX_XSC1;
            CommandData.Relays.video_trans = 1;
            CommandData.Relays.update_video = 1;
            break;
        case vtx_xsc0:
            CommandData.vtx_sel[0] = VTX_XSC0;
            CommandData.Relays.video_trans = 0;
            CommandData.Relays.update_video = 1;
            break;
        case load_curve:
            CommandData.Cryo.load_curve = 1;
            break;
        case allow_cycle:
            CommandData.Cryo.cycle_allowed = 1;
            break;
        case disallow_cycle:
            CommandData.Cryo.cycle_allowed = 0;
            break;
        case allow_watchdog:
            CommandData.Cryo.watchdog_allowed = 1;
            break;
        case disallow_watchdog:
            CommandData.Cryo.watchdog_allowed = 0;
            break;
        case force_cycle:
            CommandData.Cryo.forced = 1;
            break;
        case heaters_off:
            heater_all_off();
            break;
        case reboot_ljcryo1:
            labjack_reboot(LABJACK_CRYO_1);
            break;
        case power_box_on:
            CommandData.Relays.rec_on = 1;
            CommandData.Relays.update_rec = 1;
            break;
        case power_box_off:
            CommandData.Relays.rec_off = 1;
            CommandData.Relays.update_rec = 1;
            break;
        case amp_supply_on:
            CommandData.Relays.amp_supply_on = 1;
            CommandData.Relays.update_rec = 1;
            break;
        case amp_supply_off:
            CommandData.Relays.amp_supply_off = 1;
            CommandData.Relays.update_rec = 1;
            break;
        case therm_readout_on:
            CommandData.Relays.therm_supply_on = 1;
            CommandData.Relays.update_rec = 1;
            break;
        case therm_readout_off:
            CommandData.Relays.therm_supply_off = 1;
            CommandData.Relays.update_rec = 1;
            break;
        case heater_supply_on:
            CommandData.Relays.heater_supply_on = 1;
            CommandData.Relays.update_rec = 1;
            break;
        case heater_supply_off:
            CommandData.Relays.heater_supply_off = 1;
            CommandData.Relays.update_rec = 1;
            break;
        case heater_300mk_on:
            CommandData.Cryo.heater_300mk = 1;
            CommandData.Cryo.heater_update = 1;
            break;
        case heater_300mk_off:
            CommandData.Cryo.heater_300mk = 0;
            CommandData.Cryo.heater_update = 1;
            break;
        case charcoal_hs_on:
            CommandData.Cryo.charcoal_hs = 1;
            CommandData.Cryo.heater_update = 1;
            break;
        case charcoal_hs_off:
            CommandData.Cryo.charcoal_hs = 0;
            CommandData.Cryo.heater_update = 1;
            break;
        case single_cal_pulse:
            CommandData.Cryo.do_cal_pulse = 1;
            break;
        case lna350_on:
            CommandData.Cryo.lna_350 = 1;
            CommandData.Cryo.heater_update = 1;
            break;
        case lna350_off:
            CommandData.Cryo.lna_350 = 0;
            CommandData.Cryo.heater_update = 1;
            break;
        case lna500_on:
            CommandData.Cryo.lna_500 = 1;
            CommandData.Cryo.heater_update = 1;
            break;
        case lna500_off:
            CommandData.Cryo.lna_500 = 0;
            CommandData.Cryo.heater_update = 1;
            break;
        case level_sensor_pulse:
            CommandData.Cryo.do_level_pulse = 1;
            break;
        case heater_sync:
            CommandData.Cryo.sync = 1;
            break;
        case charcoal_on:
            CommandData.Cryo.charcoal = 1;
            CommandData.Cryo.heater_update = 1;
            break;
        case charcoal_off:
            CommandData.Cryo.charcoal = 0;
            CommandData.Cryo.heater_update = 1;
            break;
        case lna250_on:
            CommandData.Cryo.lna_250 = 1;
            CommandData.Cryo.heater_update = 1;
            break;
        case lna250_off:
            CommandData.Cryo.lna_250 = 0;
            CommandData.Cryo.heater_update = 1;
            break;
        case heater_1k_on:
            CommandData.Cryo.heater_1k = 1;
            CommandData.Cryo.heater_update = 1;
            break;
        case heater_1k_off:
            CommandData.Cryo.heater_1k = 0;
            CommandData.Cryo.heater_update = 1;
            break;
        case hd_pv_cycle:
            CommandData.Relays.cycle_of_1 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[0] = 1;
            break;
        case eth_switch_cycle:
            CommandData.Relays.cycle_of_2 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[1] = 1;
            break;
        case fc1_cycle:
            CommandData.Relays.cycle_of_3 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[2] = 1;
            break;
        case xsc1_cycle:
            CommandData.Relays.cycle_of_4 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[3] = 1;
            break;
        case fc2_cycle:
            CommandData.Relays.cycle_of_5 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[4] = 1;
            break;
        case xsc0_cycle:
            CommandData.Relays.cycle_of_6 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[5] = 1;
            break;
        case gyros_cycle:
            CommandData.Relays.cycle_of_7 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[6] = 1;
            break;
        case data_transmit_cycle:
            CommandData.Relays.cycle_of_8 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[7] = 1;
            break;
        case elmot_cycle:
            CommandData.Relays.cycle_of_9 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[8] = 1;
            break;
        case pivot_cycle:
            CommandData.Relays.cycle_of_10 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[9] = 1;
            break;
        case mag_cycle:
            CommandData.Relays.cycle_of_11 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[10] = 1;
            break;
        case rw_cycle:
            CommandData.Relays.cycle_of_12 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[11] = 1;
            break;
        case steppers_cycle:
            CommandData.Relays.cycle_of_13 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[12] = 1;
            break;
        case clino_cycle:
            CommandData.Relays.cycle_of_14 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[13] = 1;
            break;
        case of_lj_cycle:
            CommandData.Relays.cycle_of_15 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[14] = 1;
            break;
        case gps_timing_cycle:
            CommandData.Relays.cycle_of_16 = 1;
            CommandData.Relays.cycled_of = 1;
            CommandData.Relays.of_relays[15] = 1;
            break;
        case hd_pv_on:
            CommandData.Relays.of_1_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[0] = 1;
            break;
        case hd_pv_off:
            CommandData.Relays.of_1_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[0] = 0;
            break;
        case eth_switch_on:
            CommandData.Relays.of_2_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[1] = 1;
            break;
        case eth_switch_off:
            CommandData.Relays.of_2_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[1] = 0;
            break;
        case fc1_on:
            CommandData.Relays.of_3_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[2] = 1;
            break;
        case fc1_off:
            CommandData.Relays.of_3_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[2] = 0;
            break;
        case xsc1_on:
            CommandData.Relays.of_4_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[3] = 1;
            break;
        case xsc1_off:
            CommandData.Relays.of_4_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[3] = 0;
            break;
        case fc2_on:
            CommandData.Relays.of_5_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[4] = 1;
            break;
        case fc2_off:
            CommandData.Relays.of_5_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[4] = 0;
            break;
        case xsc0_on:
            CommandData.Relays.of_6_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[5] = 1;
            break;
        case xsc0_off:
            CommandData.Relays.of_6_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[5] = 0;
            break;
        case gyros_on:
            CommandData.Relays.of_7_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[6] = 1;
            break;
        case gyros_off:
            CommandData.Relays.of_7_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[6] = 0;
            break;
        case data_transmit_on:
            CommandData.Relays.of_8_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[7] = 1;
            break;
        case data_transmit_off:
            CommandData.Relays.of_8_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[7] = 0;
            break;
        case elmot_on:
            CommandData.Relays.of_9_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[8] = 1;
            break;
        case elmot_off:
            CommandData.Relays.of_9_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[8] = 0;
            break;
        case pivot_on:
            CommandData.Relays.of_10_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[9] = 1;
            break;
        case pivot_off:
            CommandData.Relays.of_10_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[9] = 0;
            break;
        case mag_on:
            CommandData.Relays.of_11_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[10] = 1;
            break;
        case mag_off:
            CommandData.Relays.of_11_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[10] = 0;
            break;
        case rw_on:
            CommandData.Relays.of_12_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[11] = 1;
            break;
        case rw_off:
            CommandData.Relays.of_12_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[11] = 0;
            break;
        case steppers_on:
            CommandData.Relays.of_13_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[12] = 1;
            break;
        case steppers_off:
            CommandData.Relays.of_13_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[12] = 0;
            break;
        case clino_on:
            CommandData.Relays.of_14_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[13] = 1;
            break;
        case clino_off:
            CommandData.Relays.of_14_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[13] = 0;
            break;
        case of_lj_on:
            CommandData.Relays.of_15_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[14] = 1;
            break;
        case of_lj_off:
            CommandData.Relays.of_15_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[14] = 0;
            break;
        case gps_timing_on:
            CommandData.Relays.of_16_on = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[15] = 1;
            break;
        case gps_timing_off:
            CommandData.Relays.of_16_off = 1;
            CommandData.Relays.update_of = 1;
            CommandData.Relays.of_relays[15] = 0;
            break;
        case gps_sw_reset:
            system("/usr/local/bin/gps_sw_reset");
            // berror(fatal, "Commands: failed to reboot gps software\n");
            break;
        case if_1_cycle:
            CommandData.Relays.cycle_if_1 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[0] = 1;
            break;
        case if_lj_cycle:
            CommandData.Relays.cycle_if_2 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[1] = 1;
            break;
        case timing_dist_cycle:
            CommandData.Relays.cycle_if_3 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[2] = 1;
            break;
        case vtx_cycle:
            CommandData.Relays.cycle_if_4 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[3] = 1;
            break;
        case bi0_cycle:
            CommandData.Relays.cycle_if_5 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[4] = 1;
            break;
        case if_6_cycle:
            CommandData.Relays.cycle_if_6 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[5] = 1;
            break;
        case if_eth_switch_cycle:
            CommandData.Relays.cycle_if_7 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[6] = 1;
            break;
        case if_8_cycle:
            CommandData.Relays.cycle_if_8 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[7] = 1;
            break;
        case roach_cycle:
            CommandData.Relays.cycle_if_9 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[8] = 1;
            break;
        case cryo_hk_cycle:
            CommandData.Relays.cycle_if_10 = 1;
            CommandData.Relays.cycled_if = 1;
            CommandData.Relays.if_relays[9] = 1;
            break;
        case if_relay_1_on:
            CommandData.Relays.if_1_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[0] = 1;
            break;
        case if_relay_1_off:
            CommandData.Relays.if_1_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[0] = 0;
            break;
        case if_lj_on:
            CommandData.Relays.if_2_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[1] = 1;
            break;
        case if_lj_off:
            CommandData.Relays.if_2_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[1] = 0;
            break;
        case timing_dist_on:
            CommandData.Relays.if_3_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[2] = 1;
            break;
        case timing_dist_off:
            CommandData.Relays.if_3_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[2] = 0;
            break;
        case vtx_on:
            CommandData.power.bi0.rst_count = 0;
            CommandData.power.bi0.set_count = LATCH_PULSE_LEN;
            CommandData.Relays.if_4_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[3] = 1;
            break;
        case vtx_off:
            CommandData.power.bi0.set_count = 0;
            CommandData.power.bi0.rst_count = LATCH_PULSE_LEN;
            CommandData.Relays.if_4_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[3] = 0;
            break;
        case bi0_on:
            CommandData.power.sc_tx.rst_count = 0;
            CommandData.power.sc_tx.set_count = LATCH_PULSE_LEN;
            CommandData.Relays.if_5_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[4] = 1;
            break;
        case bi0_off:
            CommandData.power.sc_tx.set_count = 0;
            CommandData.power.sc_tx.rst_count = LATCH_PULSE_LEN;
            CommandData.Relays.if_5_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[4] = 0;
            break;
        case if_relay_6_on:
            CommandData.Relays.if_6_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[5] = 1;
            break;
        case if_relay_6_off:
            CommandData.Relays.if_6_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[5] = 0;
            break;
        case if_eth_switch_on:
            CommandData.Relays.if_7_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[6] = 1;
            break;
        case if_eth_switch_off:
            CommandData.Relays.if_7_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[6] = 0;
            break;
        case if_relay_8_on:
            CommandData.Relays.if_8_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[7] = 1;
            break;
        case if_relay_8_off:
            CommandData.Relays.if_8_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[7] = 0;
            break;
        case roach_on:
            CommandData.Relays.if_9_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[8] = 1;
            break;
        case roach_off:
            CommandData.Relays.if_9_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[8] = 0;
            break;
        case cryo_hk_on:
            CommandData.Relays.if_10_on = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[9] = 1;
            break;
        case cryo_hk_off:
            CommandData.Relays.if_10_off = 1;
            CommandData.Relays.update_if = 1;
            CommandData.Relays.if_relays[9] = 0;
            break;
        case mag_reset:
            CommandData.mag_reset = 1;
            break;
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

        case trim_to_xsc0:
            CommandData.autotrim_enable = 0;
            CommandData.autotrim_rate = 0.0;
            SetTrimToSC(0);
            break;
        case trim_to_xsc1:
            CommandData.autotrim_enable = 0;
            CommandData.autotrim_rate = 0.0;
            SetTrimToSC(1);
            break;
        case trim_xsc1_to_xsc0:
            trim_xsc(0);
            break;
        case trim_xsc0_to_xsc1:
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
        case reset_ethercat:
            CommandData.ec_devices.reset = 1;
            break;
        case pss_veto:
            CommandData.use_pss = 0;
            break;
        case dgps_veto:
            CommandData.use_dgps = 0;
            break;
        case xsc0_veto:
            CommandData.use_xsc0 = 0;
            break;
        case xsc1_veto:
            CommandData.use_xsc1 = 0;
            break;
        case mag_veto_fc1:
            CommandData.use_mag1 = 0;
            break;
        case mag_veto_fc2:
            CommandData.use_mag2 = 0;
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
        case dgps_allow:
            CommandData.use_dgps = 1;
            break;
        case xsc0_allow:
            CommandData.use_xsc0 = 1;
            break;
        case xsc1_allow:
            CommandData.use_xsc1 = 1;
            break;
        case mag_allow_fc1:
            CommandData.use_mag1 = 1;
            break;
        case mag_allow_fc2:
            CommandData.use_mag2 = 1;
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
        case hwpr_enc_on:
            CommandData.Cryo.hwprPos = -1;
            break;
        case hwpr_enc_off:
            CommandData.Cryo.hwprPos = 0;
            break;
        case hwpr_enc_pulse:
            CommandData.Cryo.hwprPos = 50;
            break;
        // case auto_cycle:
            // CommandData.Cryo.fridgeCycle = 1;
            // CommandData.Cryo.force_cycle = 0;
            // break;
        // case fridge_cycle:
            // CommandData.Cryo.fridgeCycle = 1;
            // CommandData.Cryo.force_cycle = 1;
            // break;
        // case cal_on:
            // CommandData.Cryo.calibrator = on;
            // break;
        // case cal_off:
            // CommandData.Cryo.calibrator = off;
            // break;
        // case hs_pot_on:
            // CommandData.Cryo.hsPot = 1;
            // break;
        // case hs_pot_off:
            // CommandData.Cryo.hsPot = 0;
            // break;
        // case bda_on:
            // CommandData.Cryo.BDAHeat = 1;
            // break;
        // case bda_off:
            // CommandData.Cryo.BDAHeat = 0;
            // break;
        // cryo valves
		case potvalve_open:
        	CommandData.Cryo.potvalve_goal = opened;
        	break;
    	case potvalve_close:
        	CommandData.Cryo.potvalve_goal = closed;
        	break;
    	case potvalve_on:
        	CommandData.Cryo.potvalve_on = 1;
	    	CommandData.Cryo.potvalve_goal = 0;
        	break;
    	case potvalve_off:
        	CommandData.Cryo.potvalve_on = 0;
	    	CommandData.Cryo.potvalve_goal = 0;
        	break;
    	case pump_valve_open:
	    	CommandData.Cryo.valve_goals[0] = opened;
	    	break;
		case pump_valve_close:
	    	CommandData.Cryo.valve_goals[0] = closed;
	    	break;
		case pump_valve_off:
	    	CommandData.Cryo.valve_goals[0] = 0;
	    	CommandData.Cryo.valve_stop[0] = 1;
	    	break;
		case pump_valve_on:
	    	CommandData.Cryo.valve_goals[0] = 0;
	    	CommandData.Cryo.valve_stop[0] = 0;
		case fill_valve_open:
	    	CommandData.Cryo.valve_goals[1] = opened;
	    	break;
		case fill_valve_close:
	    	CommandData.Cryo.valve_goals[1] = closed;
	    	break;
		case fill_valve_off:
	    	CommandData.Cryo.valve_goals[1] = 0;
	    	CommandData.Cryo.valve_stop[1] = 1;
	    	break;
		case fill_valve_on:
	    	CommandData.Cryo.valve_goals[1] = 0;
	    	CommandData.Cryo.valve_stop[1] = 0;
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
	case balance_auto:
	    CommandData.balance.mode = bal_auto;
	    break;
	case balance_off:
	    CommandData.balance.mode = bal_rest;
	    break;
	case balance_terminate:
	  // after lock45, before termination, drive balance system to lower limit
	  CommandData.balance.vel = 200000;
	  CommandData.balance.mode = bal_manual;
	  CommandData.balance.bal_move_type = 2;
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
        case vna_sweep_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].do_sweeps = 1;
            }
            break;
        case targ_sweep_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].do_sweeps = 2;
            }
            break;
        case df_targ_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
              CommandData.roach[i].do_df_targ = 1;
            }
          break;
        case find_kids_default_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].find_kids = 1;
            }
            break;
        case center_lo_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].set_lo = 1;
            }
          break;
        case load_freqs_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].load_new_freqs = 1;
            }
            break;
        case calc_dfs:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].do_df_calc = 2;
            }
            break;
        case change_amps:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].change_tone_amps = 1;
            }
            break;
        case reload_vna_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].load_vna_amps = 2;
            }
            break;
        case end_sweeps_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].do_sweeps = 0;
            }
            break;
        case new_ref_params_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].calc_ref_params = 1;
            }
            break;
        case set_attens_default:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].set_attens = 2;
            }
            break;
        case set_attens_min_output:
          for (int i = 0; i < NUM_ROACHES; i++) {
              CommandData.roach_params[i].out_atten = 30.0;
              CommandData.roach[i].set_attens = 1;
          }
          break;
        case set_attens_last_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].set_attens = 3;
            }
            break;
        case auto_find_kids_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].auto_find = 1;
            }
            break;
        case zero_df_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].recenter_df = 1;
            }
            break;
        case reset_roach_all:
            for (int i = 0; i < NUM_ROACHES; i++) {
              CommandData.roach[i].roach_new_state = ROACH_STATE_BOOT;
              CommandData.roach[i].roach_desired_state = ROACH_STATE_STREAMING;
              CommandData.roach[i].change_roach_state = 1;
            }
          break;
        case flight_mode:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].go_flight_mode = 1;
            }
            break;
        case debug_mode:
            for (int i = 0; i < NUM_ROACHES; i++) {
                CommandData.roach[i].go_flight_mode = 0;
                // CommandData.roach[i].auto_find = 0;
                CommandData.roach[i].do_sweeps = 0;
            }
            break;
        case change_freqs_all:
          for (int i = 0; i < NUM_ROACHES; i++) {
              CommandData.roach[i].change_targ_freq = 2;
          }
        case xyzzy:
            break;
	#ifdef USE_XY_THREAD
	case xy_panic:
	    CommandData.xystage.mode = XYSTAGE_PANIC;
	    CommandData.xystage.is_new = 1;
	    break;
	#endif


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
  char * filename;

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
      CommandData.autotrim_xsc1_last_bad = CommandData.autotrim_xsc0_last_bad;
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
    case mag_cal_fc1:
      CommandData.cal_xmax_mag[0] = rvalues[0];
      CommandData.cal_xmin_mag[0] = rvalues[1];
      CommandData.cal_ymax_mag[0] = rvalues[2];
      CommandData.cal_ymin_mag[0] = rvalues[3];
      CommandData.cal_mag_align[0] = rvalues[4];
      blast_info("Updating mag1 cal coeffs: xmax = %f, xmin = %f, ymin = %f, ymax = %f, align = %f",
                 CommandData.cal_xmax_mag[0], CommandData.cal_xmin_mag[0],
                 CommandData.cal_ymax_mag[0], CommandData.cal_ymin_mag[0], CommandData.cal_mag_align[0]);
      break;
    case mag_cal_fc2:
      CommandData.cal_xmax_mag[1] = rvalues[0];
      CommandData.cal_xmin_mag[1] = rvalues[1];
      CommandData.cal_ymax_mag[1] = rvalues[2];
      CommandData.cal_ymin_mag[1] = rvalues[3];
      CommandData.cal_mag_align[1] = rvalues[4];
      blast_info("Updating mag1 cal coeffs: xmax = %f, xmin = %f, ymin = %f, ymax = %f, align = %f",
                 CommandData.cal_xmax_mag[1], CommandData.cal_xmin_mag[1],
                 CommandData.cal_ymax_mag[1], CommandData.cal_ymin_mag[1], CommandData.cal_mag_align[1]);
      break;

    case pss_set_imin:
      CommandData.cal_imin_pss = rvalues[0];
      break;

	case pss_cal_n:
	  i = ivalues[0]-1;
	  CommandData.cal_d_pss[i] = rvalues[1];
	  CommandData.cal_az_pss[i] = rvalues[2];
	  CommandData.cal_el_pss[i] = rvalues[3];
	  CommandData.cal_roll_pss[i] = rvalues[4];
	  break;

	case pss_cal_d:
	  for (i = 0; i < NUM_PSS; i++) {
		  CommandData.cal_d_pss[i] = rvalues[i];
	  }
	  break;

	case pss_cal_az:
	  for (i = 0; i < NUM_PSS; i++) {
		  CommandData.cal_az_pss[i] = rvalues[i];
	  }
	  break;
	case pss_cal_array_az:
	  CommandData.cal_az_pss_array = rvalues[0];
	  break;

	case pss_cal_el:
	  for (i = 0; i < NUM_PSS; i++) {
		  CommandData.cal_el_pss[i] = rvalues[i];
	  }
	  break;

	case pss_cal_roll:
	  for (i = 0; i < NUM_PSS; i++) {
		  CommandData.cal_roll_pss[i] = rvalues[i];
	  }
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
    case fix_ethercat:
      CommandData.ec_devices.fix_rw = ivalues[0];
      CommandData.ec_devices.fix_el = ivalues[1];
      CommandData.ec_devices.fix_piv = ivalues[2];
      CommandData.ec_devices.fix_hwpr = ivalues[3];
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
    case hwpr_goto_rel:
      CommandData.hwpr.target = ivalues[0];
      CommandData.hwpr.mode = HWPR_GOTO_REL;
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
	case hwpr_set_margin:
	  CommandData.hwpr.margin = ivalues[0];
	  break;
    case potvalve_set_vel:
      CommandData.Cryo.potvalve_vel = ivalues[0];
      break;
    case potvalve_set_current:
      CommandData.Cryo.potvalve_opencurrent = ivalues[0];
      CommandData.Cryo.potvalve_closecurrent = ivalues[1];
      break;
    case potvalve_set_hold_current:
      CommandData.Cryo.potvalve_hold_i = ivalues[0];
      break;
    case potvalve_set_thresholds:
      CommandData.Cryo.potvalve_closed_threshold = ivalues[0];
      CommandData.Cryo.potvalve_lclosed_threshold = ivalues[1];
      CommandData.Cryo.potvalve_open_threshold = ivalues[2];
      break;
    case valves_set_vel:
      CommandData.Cryo.valve_vel = ivalues[0];
      break;
    case valves_set_move_i:
      CommandData.Cryo.valve_move_i = ivalues[0];
      break;
    case valves_set_hold_i:
      CommandData.Cryo.valve_hold_i = ivalues[0];
      break;
    case valves_set_acc:
      CommandData.Cryo.valve_acc = ivalues[0];
      break;

// .
    // XY STAGE
// .
#ifdef USE_XY_THREAD
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
#endif
// .
    /*************************************
    ********* Cryostat  ***********/
    case set_queue_execute:
      set_execute(ivalues[0]);
      break;
    case reconnect_lj:
      set_reconnect(ivalues[0]-1);
      break;
    case cal_length: // specify length in ms (multiples of 5)
      CommandData.Cryo.cal_length = (ivalues[0]/5);
      break;
    case level_length: // specify length in seconds
      CommandData.Cryo.level_length = (ivalues[0]*5);
      break;
    case set_tcrit_fpa: // specify temp in ADC counts
      CommandData.Cryo.tcrit_fpa = ivalues[0];
      break;
    case periodic_cal:
      CommandData.Cryo.periodic_pulse = 1;
      CommandData.Cryo.num_pulse = ivalues[0];
      CommandData.Cryo.separation = ivalues[1];
      CommandData.Cryo.length = ivalues[2];
      break;
    case send_dac:
      CommandData.Cryo.dac_value = (rvalues[0]);
      CommandData.Cryo.labjack = ivalues[0];
      CommandData.Cryo.send_dac = 1;
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
      CommandData.balance.bal_move_type = ((int)(0 < ivalues[0]) - (int)(ivalues[0] < 0)) + 1;
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
    case set_linklists:
      if (ivalues[0] == 0) {
        copysvalue(CommandData.pilot_linklist_name, linklist_nt[ivalues[1]]);
        telemetries_linklist[PILOT_TELEMETRY_INDEX] =
            linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
      } else if (ivalues[0] == 1) {
        copysvalue(CommandData.bi0_linklist_name, linklist_nt[ivalues[1]]);
        telemetries_linklist[BI0_TELEMETRY_INDEX] =
            linklist_find_by_name(CommandData.bi0_linklist_name, linklist_array);
      } else if (ivalues[0] == 2) {
        copysvalue(CommandData.highrate_linklist_name, linklist_nt[ivalues[1]]);
        telemetries_linklist[HIGHRATE_TELEMETRY_INDEX] =
            linklist_find_by_name(CommandData.highrate_linklist_name, linklist_array);
      } else if (ivalues[0] == 3) {
        copysvalue(CommandData.sbd_linklist_name, linklist_nt[ivalues[1]]);
        telemetries_linklist[SBD_TELEMETRY_INDEX] =
            linklist_find_by_name(CommandData.sbd_linklist_name, linklist_array);
      } else {
        blast_err("Unknown downlink index %d", ivalues[0]);
      }
      break;
    case request_file:
      i = 0;
      while (linklist_nt[i]) i++;
      if (ivalues[0] < i) {
        send_file_to_linklist(linklist_find_by_name(
            (char *) linklist_nt[ivalues[0]], linklist_array), "file_block", svalues[1]);
      } else {
        blast_err("Index %d is outside linklist name range", ivalues[0]);
      }
      break;
    case request_stream_file:
      filename = svalues[0];
			if (svalues[1][0] == '$') filename = getenv(svalues[1]+1); // hook for environment variable

      if (filename) {
        if (ivalues[0] == 0) { // pilot
					snprintf(CommandData.pilot_linklist_name, sizeof(CommandData.pilot_linklist_name), FILE_LINKLIST);
					telemetries_linklist[PILOT_TELEMETRY_INDEX] =
							linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
        } else if (ivalues[0] == 1) { // BI0
					snprintf(CommandData.bi0_linklist_name, sizeof(CommandData.bi0_linklist_name), FILE_LINKLIST);
					telemetries_linklist[BI0_TELEMETRY_INDEX] =
							linklist_find_by_name(CommandData.bi0_linklist_name, linklist_array);
        } else if (ivalues[0] == 2) { // highrate
					snprintf(CommandData.highrate_linklist_name, sizeof(CommandData.highrate_linklist_name), FILE_LINKLIST);
					telemetries_linklist[HIGHRATE_TELEMETRY_INDEX] =
							linklist_find_by_name(CommandData.highrate_linklist_name, linklist_array);
        } else {
          blast_err("Cannot send files over link index %d", ivalues[0]);
          break;
        }
				send_file_to_linklist(linklist_find_by_name(FILE_LINKLIST, linklist_array),
															 "file_block", filename);
      } else {
        blast_err("Could not resolve filename \"%s\"", svalues[1]);
      }
      break;
		case set_pilot_oth:
				CommandData.pilot_oth = ivalues[0];
				blast_info("Switched to Pilot to stream to \"%s\"\n", pilot_target_names[CommandData.pilot_oth]);
				break;
    case biphase_clk_speed:
      // Value entered by user in kbps but stored in bps
      if (ivalues[0] == 100) {
        CommandData.biphase_clk_speed = 100000;
      } else if (ivalues[0] == 500) {
        CommandData.biphase_clk_speed = 500000;
      } else if (ivalues[0] == 1000) {
        CommandData.biphase_clk_speed = 1000000;
      } else {
        char *str;
        char *str2;
        char str3[1000];
        asprintf(&str, "Biphase clk_speed : %d kbps is not allowed (try 100, 500 or 1000).\n", ivalues[0]);
        asprintf(&str2, "Biphase clk_speed has not been changed, it\'s %d bps", CommandData.biphase_clk_speed);
        snprintf(str3, sizeof(str3), "%s %s", str, str2);
        blast_warn("%s", str3);
      }
      break;
    case highrate_through_tdrss:
      // route through tdrss or otherwise
      if (ivalues[0]) {
        CommandData.highrate_through_tdrss = true;
      } else {
        CommandData.highrate_through_tdrss = false;
      }
      break;
    case timeout:        // Set timeout
      CommandData.timeout = rvalues[0];
      break;
    case highrate_bw:
      // Value entered by user in kbps but stored in Bps
      CommandData.highrate_bw = rvalues[0]*1000.0/8.0;
      CommandData.highrate_allframe_fraction = rvalues[1];
      blast_info("Changed highrate bw to %f kbps (%f percent allframe)", rvalues[0], rvalues[1]*100.0);
      break;
    case pilot_bw:
      // Value entered by user in kbps but stored in Bps
      CommandData.pilot_bw = rvalues[0]*1000.0/8.0;
      CommandData.pilot_allframe_fraction = rvalues[1];
      blast_info("Changed pilot bw to %f kbps (%f percent allframe)", rvalues[0], rvalues[1]*100.0);
      break;
    case biphase_bw:
      // Value entered by user in kbps but stored in Bps
      CommandData.biphase_bw = rvalues[0]*1000.0/8.0;
      CommandData.biphase_allframe_fraction = rvalues[1];
      blast_info("Changed biphase bw to %f kbps (%f percent allframe)", rvalues[0], rvalues[1]*100.0);
      break;
    case set_roach_mode:
        if (ivalues[0] == 0) CommandData.roach_tlm_mode = ROACH_TLM_IQDF;
        else if (ivalues[0] == 1) CommandData.roach_tlm_mode = ROACH_TLM_DELTA;
        break;
    case set_roach_all_chan:
        if (ivalues[0] > 0 && ivalues[0] <= NUM_ROACHES) {
          CommandData.num_channels_all_roaches[ivalues[0]-1] = ivalues[1];
          blast_info("Roach %d to send %d kids\n", ivalues[0], ivalues[1]);
        }
        break;
    case set_roach_chan:
      for (i = 0; i < NUM_ROACH_TLM; i++) {
        CommandData.roach_tlm[i].kid = ivalues[(i/3)*2+0];
        CommandData.roach_tlm[i].roach = ivalues[(i/3)*2+1];
        CommandData.roach_tlm[i].rtype = i%3;

        CommandData.roach_tlm[i].index = get_roach_index(CommandData.roach_tlm[i].roach,
                                                         CommandData.roach_tlm[i].kid,
                                                         CommandData.roach_tlm[i].rtype);
        make_name_from_roach_index(CommandData.roach_tlm[i].index,
                                   CommandData.roach_tlm[i].name);
        CommandData.roach_tlm_mode = ROACH_TLM_IQDF;
      }
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

// *****************************************
// ROACH Commands
// *****************************************
    case load_new_vna_amps:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((ivalues[1] >= 0) && ivalues[1] <= 2)) {
          CommandData.roach[ivalues[0]-1].load_vna_amps = ivalues[1];
      }
      break;
    case load_new_targ_amps:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].load_targ_amps = ivalues[1];
          blast_info("LOAD TARG AMPS: %u", CommandData.roach[ivalues[0]-1].load_targ_amps);
      }
      break;
    case load_freqs:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].load_new_freqs = 1;
      }
      break;
    case cal_adc:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].calibrate_adc = 1;
      }
      break;
    case end_sweep:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].do_sweeps = 0;
          CommandData.roach[ivalues[0]-1].do_cal_sweeps = 0;
      }
      break;
    case vna_sweep:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].do_sweeps = 1;
      }
      break;
    case cal_sweeps:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
            // &&
            // ((rvalues[1] >= 0.5) && (rvalues[1] <= 6.0)) &&
            // ((rvalues[2] >= 5) && ((rvalues[2] <= 101)) &&
            // ((rvalues[3] >= 2) && (rvalues[3] <= 10)))) {
          CommandData.roach_params[ivalues[0]-1].atten_step = rvalues[1];
          CommandData.roach_params[ivalues[0]-1].npoints = rvalues[2];
          CommandData.roach_params[ivalues[0]-1].ncycles = rvalues[3];
          CommandData.roach[ivalues[0]-1].do_cal_sweeps = 1;
      }
      break;
    case targ_sweep:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].do_sweeps = 2;
      }
      break;
    case reset_roach:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].roach_new_state = ROACH_STATE_BOOT;
          CommandData.roach[ivalues[0]-1].roach_desired_state = ROACH_STATE_STREAMING;
          CommandData.roach[ivalues[0]-1].change_roach_state = 1;
          CommandData.roach[ivalues[0]-1].do_sweeps = 1;
      }
      break;
    case calc_df:
      if ((ivalues[0] > 0)) {
          CommandData.roach[ivalues[0]-1].do_df_calc = 1;
          CommandData.roach[ivalues[0]-1].chan = ivalues[1];
      }
      break;
    case check_retune:
      if ((ivalues[0] > 0)) {
          CommandData.roach[ivalues[0]-1].do_check_retune = 1;
      }
    case retune:
      if ((ivalues[0] > 0)) {
          CommandData.roach[ivalues[0]-1].do_retune = 1;
      }
      break;
    case auto_retune:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((ivalues[1] >= 0) && ivalues[1] <= 1)) {
          CommandData.roach[ivalues[0]-1].auto_retune = ivalues[1];
      }
      break;
    case opt_tones:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((ivalues[1] >= 0) && ivalues[1] <= 1)) {
          CommandData.roach[ivalues[0]-1].opt_tones = ivalues[1];
      }
      break;
    case set_attens:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((rvalues[1] >= 0.0) && rvalues[1] <= 30.0)
                 && ((rvalues[2] >= 0.0) && rvalues[2] <= 30.0)) {
          CommandData.roach_params[ivalues[0]-1].out_atten = rvalues[1];
          CommandData.roach_params[ivalues[0]-1].in_atten = rvalues[2];
          CommandData.roach[ivalues[0]-1].set_attens = 1;
      }
      break;
    case set_attens_conserve:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((rvalues[1] >= 0.0) && rvalues[1] <= 30.0)) {
          CommandData.roach_params[ivalues[0]-1].out_atten = rvalues[1];
          CommandData.roach[ivalues[0]-1].set_attens = 4;
      }
      break;
    case set_attens_calc:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach_params[ivalues[0]-1].dBm_per_tone = rvalues[1];
          CommandData.roach[ivalues[0]-1].set_attens = 5;
      }
      break;
    case reboot_pi:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].reboot_pi_now = 1;
      }
      break;
    case read_attens:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].read_attens = 1;
      }
      break;
    case new_output_atten:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((rvalues[1] > 0) && rvalues[1] <= 30)) {
          CommandData.roach_params[ivalues[0]-1].new_out_atten = rvalues[1];
          CommandData.roach[ivalues[0]-1].new_atten = 1;
      }
      break;
    case find_kids:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach_params[ivalues[0]-1].smoothing_scale = rvalues[1];
          CommandData.roach_params[ivalues[0]-1].peak_threshold = rvalues[2];
          CommandData.roach_params[ivalues[0]-1].spacing_threshold = rvalues[3];
          CommandData.roach[ivalues[0]-1].find_kids = 2;
      }
      break;
    case find_kids_default:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].find_kids = 1;
      }
      break;
    case show_adc_rms:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].adc_rms = 1;
      }
      break;
    case test_tone:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((rvalues[1] >= 1.0e6) && rvalues[1] <= 250.0e6)) {
          CommandData.roach_params[ivalues[0]-1].test_freq = rvalues[1];
          CommandData.roach[ivalues[0]-1].test_tone = 1;
      }
      break;
    case change_state:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((ivalues[1] >= 0) && ivalues[1] <= 11)
                           && ((ivalues[2] >= 0) && ivalues[2] <= 11)) {
          CommandData.roach[ivalues[0]-1].roach_new_state = ivalues[1];
          CommandData.roach[ivalues[0]-1].roach_desired_state = ivalues[2];
          CommandData.roach[ivalues[0]-1].change_roach_state = 1;
      }
      break;
    case get_state:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].get_roach_state = 1;
      }
      break;
    case calc_phase_centers:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((ivalues[1] >= 0) && ivalues[1] <= 1)) {
          CommandData.roach[ivalues[0]-1].get_phase_centers = ivalues[1];
      }
      break;
    case timestream:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES) && ((ivalues[1] >= 0) && ivalues[1] <= 1000)
                          && ((rvalues[2] >= 0.0) && rvalues[2] <= 300.0)) {
          CommandData.roach[ivalues[0]-1].chan = ivalues[1];
          CommandData.roach_params[ivalues[0]-1].num_sec = rvalues[2];
          CommandData.roach[ivalues[0]-1].get_timestream = 1;
      }
      break;
    case all_roach_ts:
      if ((rvalues[0] >= 0.0) && (rvalues[0] <= 300.0)) {
          for (int i = 0; i < NUM_ROACHES; i++) {
              CommandData.roach_params[i].num_sec = rvalues[0];
              CommandData.roach[i].get_timestream = 2;
          }
      }
      break;
    case all_roach_df:
      if ((rvalues[0] >= 0.0) && (rvalues[0] <= 300.0)) {
          for (int i = 0; i < NUM_ROACHES; i++) {
              CommandData.roach_params[i].num_sec = rvalues[0];
              CommandData.roach[i].get_timestream = 3;
          }
      }
      break;
    case set_attens_all:
      if  ((rvalues[0] >= 0.0) && (rvalues[0] <= 30.0) &&
              ((rvalues[1] >= 0.0) && (rvalues[1] <= 30.0))) {
          for (int i = 0; i < NUM_ROACHES; i++) {
              CommandData.roach_params[i].out_atten = rvalues[0];
              CommandData.roach_params[i].in_atten = rvalues[1];
              CommandData.roach[i].set_attens = 1;
          }
      }
      break;
    case cal_amps:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach_params[ivalues[0]-1].num_sec = rvalues[1];
          CommandData.roach_params[ivalues[0]-1].ncycles = rvalues[2];
          CommandData.roach_params[ivalues[0]-1].delta_amp = rvalues[3];
          CommandData.roach_params[ivalues[0]-1].resp_thresh = rvalues[4];
          CommandData.roach[ivalues[0]-1].tune_amps = 1;
      }
      break;
    case refit_freqs:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].refit_res_freqs = 1;
          CommandData.roach[ivalues[0]-1].on_res = ivalues[1];
      }
      break;
    case df_targ:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].do_df_targ = 1;
      }
      break;
    case targ_refit_all:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          for (int i = 0; i < NUM_ROACHES; i++) {
              CommandData.roach[ivalues[i]].do_sweeps = 2;
              CommandData.roach[ivalues[i]].refit_res_freqs = 1;
              CommandData.roach[ivalues[i]].on_res = 1;
              CommandData.roach[ivalues[i]].check_response = ivalues[1];
          }
      }
      break;
    case targ_refit:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].do_sweeps = 2;
          CommandData.roach[ivalues[0]-1].refit_res_freqs = 1;
          CommandData.roach[ivalues[0]-1].on_res = 1;
          CommandData.roach[ivalues[0]-1].check_response = ivalues[1];
      }
      break;
    case refit_freqs_all:
        for (int i = 0; i < NUM_ROACHES; i++) {
            CommandData.roach[i].refit_res_freqs = 1;
            CommandData.roach[i].on_res = ivalues[0];
        }
        break;
    case chop_template:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].do_master_chop = 1;
      }
      break;
    case new_ref_params:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].calc_ref_params = 1;
      }
      break;
    case offset_lo:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].set_lo = 2;
          CommandData.roach_params[ivalues[0]-1].lo_offset = rvalues[1];
      }
      break;
    case offset_lo_all:
      for (int i = 0; i < NUM_ROACHES; i++) {
          CommandData.roach[i].set_lo = 2;
          CommandData.roach_params[i].lo_offset = rvalues[0];
      }
      break;
    case center_lo:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].set_lo = 1;
      }
      break;
    case read_lo:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].read_lo = 1;
      }
      break;
    case set_lo_MHz:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach_params[ivalues[0]-1].lo_freq_MHz = rvalues[1];
          CommandData.roach[ivalues[0]-1].set_lo = 3;
      }
      break;
    case change_amp:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].change_tone_amps = 1;
          CommandData.roach[ivalues[0]-1].chan = ivalues[1];
          CommandData.roach_params[ivalues[0]-1].delta_amp = rvalues[2];
      }
      break;
    case change_phase:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].change_tone_phase = 1;
          CommandData.roach[ivalues[0]-1].chan = ivalues[1];
          CommandData.roach_params[ivalues[0]-1].delta_phase = rvalues[2];
      }
      break;
    case change_freq:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].change_targ_freq = 1;
          CommandData.roach[ivalues[0]-1].chan = ivalues[1];
      }
      break;
    case offset_freq:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].change_tone_freq = 1;
          CommandData.roach[ivalues[0]-1].chan = ivalues[1];
          CommandData.roach_params[ivalues[0]-1].freq_offset = rvalues[2];
      }
      break;
    case auto_find_kids:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
        CommandData.roach[ivalues[0]-1].auto_find = 1;
      }
      break;
    case lamp_check_all:
        for (int i = 0; i < NUM_ROACHES; i++) {
            CommandData.roach[i].check_response = 1;
            CommandData.roach_params[i].num_sec = rvalues[0];
        }
        break;
    case full_loop:
      if ((ivalues[0] > 0) && (ivalues[0] <= NUM_ROACHES)) {
          CommandData.roach[ivalues[0]-1].do_full_loop = 1;
          CommandData.roach[ivalues[0]-1].find_kids = ivalues[1];
      }
      break;
    case full_loop_all:
      for (int i = 0; i < NUM_ROACHES; i++) {
          CommandData.roach[i].do_full_loop = 1;
          CommandData.roach[i].find_kids = ivalues[0];
      }
      break;
      /*************************************
      ************** Bias  ****************/
//       used to be multiplied by 2 here, but screw up prev_satus
//       need to multiply later instead
    case set_rox_bias_amp: // Set the amplitude of the rox bias signal
      CommandData.rox_bias.amp = ivalues[0];
      set_rox_bias();
      break;
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
                CommandData.XSC[which].trigger.grace_period_cs = rvalues[2] * 100.0; // Commanded value is in seconds!
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
        case xsc_get_focus:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_get_focus);
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
        case xsc_stop_focus:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_stop_focus);
                }
            }
            break;
        }
        case xsc_define_focus:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.define_focus_value = ivalues[1];
                    xsc_activate_command(which, xC_define_focus);
                }
            }
            break;
        }
        case xsc_set_focus_incremental:
        {
            if (ivalues[0]) {
								for (unsigned int which = 0; which < 2; which++) {
										if (xsc_command_applies_to(which, ivalues[0])) {
												CommandData.XSC[which].net.set_focus_incremental_value = ivalues[1];
												xsc_activate_command(which, xC_set_focus_incremental);
										}
								}
            } else {
                blast_err("Commands: must provide non-zero incremental value\n");
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
        case xsc_get_aperture:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_get_aperture);
                }
            }
            break;
        }
        case xsc_stop_aperture:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    xsc_activate_command(which, xC_stop_aperture);
                }
            }
            break;
        }
        case xsc_define_aperture:
        {
            for (unsigned int which = 0; which < 2; which++) {
                if (xsc_command_applies_to(which, ivalues[0])) {
                    CommandData.XSC[which].net.define_aperture_value = ivalues[1];
                    xsc_activate_command(which, xC_define_aperture);
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
static int one(const struct dirent *unused) {
  return 1;
}
void InitCommandData()
{
		/* --- Start of Convenience hack for linklist --- */
		struct dirent **dir;
		int n = scandir("/data/etc/linklists/", &dir, one, alphasort);
		int num_ll = 0;
    int i = 0;

		// get the list of linklists in the directory
		for (i = 0; i < n; i++) {
			if (num_ll >= 63) {
				printf("Reached maximum linklists for dropdown\n");
				break;
			}
			int len = strlen(dir[i]->d_name);
			if ((len >=3) && strcmp(&dir[i]->d_name[len-3], ".ll") == 0) {
				linklist_nt[num_ll] = calloc(1, 80);
				strncpy(linklist_nt[num_ll], dir[i]->d_name, 64);
				num_ll++;
			}
		}
		// assign the list of linklists to the parameters that have linklist dropdowns
		if (num_ll > 0) {
			for (i = 0; i < N_MCOMMANDS; i++) {
				int p;
				for (p = 0; p < mcommands[i].numparams; p++) {
					if (strcmp(mcommands[i].params[p].name, "Linklist") == 0) {
						mcommands[i].params[p].nt = (const char **) linklist_nt;
					}
				}
			}
			linklist_nt[num_ll] = calloc(1, 80);
			strncpy(linklist_nt[num_ll],  ALL_TELEMETRY_NAME, 79);
      num_ll++;
			linklist_nt[num_ll] = calloc(1, 80);
			strncpy(linklist_nt[num_ll], "no_linklist", 79);
      num_ll++;
		}

		/* --- End of Convenience hack for linklists --- */

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
    CommandData.mag_reset = 0;

    for (i = 0; i < NUM_ROACHES; i++) {
        CommandData.roach[i].calibrate_adc = 0;
        CommandData.roach[i].set_attens = 0;
        CommandData.roach[i].read_attens = 0;
        CommandData.roach[i].do_df_calc = 0;
        CommandData.roach[i].auto_retune = 0;
        CommandData.roach[i].do_sweeps = 0;
        CommandData.roach[i].do_cal_sweeps = 0;
        CommandData.roach[i].change_roach_state = 0;
        CommandData.roach[i].get_roach_state = 0;
        CommandData.roach[i].find_kids = 0;
        CommandData.roach[i].opt_tones = 0;
        CommandData.roach[i].adc_rms = 0;
        CommandData.roach[i].test_tone = 0;
        CommandData.roach[i].new_atten = 0;
        CommandData.roach[i].load_vna_amps = 0;
        CommandData.roach[i].load_targ_amps = 0;
        CommandData.roach[i].get_phase_centers = 0;
        CommandData.roach[i].get_timestream = 0;
        CommandData.roach[i].tune_amps = 0;
        CommandData.roach[i].refit_res_freqs = 0;
        CommandData.roach[i].change_tone_amps = 0;
        CommandData.roach[i].do_master_chop = 0;
        CommandData.roach[i].load_new_freqs = 0;
        CommandData.roach[i].calc_ref_params = 0;
        CommandData.roach[i].do_retune = 0;
        CommandData.roach[i].set_lo = 0;
        CommandData.roach[i].read_lo = 0;
        CommandData.roach[i].chan = 0;
        CommandData.roach[i].change_targ_freq = 0;
        CommandData.roach[i].change_tone_phase = 0;
        CommandData.roach[i].change_tone_freq = 0;
        CommandData.roach[i].on_res = 1;
        CommandData.roach[i].auto_find = 0;
        CommandData.roach_params[i].in_atten = 19;
        CommandData.roach[i].recenter_df = 0;
        CommandData.roach[i].go_flight_mode = 0;
        CommandData.roach[i].check_response = 0;
        CommandData.roach[i].reboot_pi_now = 0;
        CommandData.roach[i].do_df_targ = 0;
        CommandData.roach[i].do_full_loop = 0;
        CommandData.roach_params[i].read_in_atten = 0;
        CommandData.roach_params[i].read_out_atten = 0;
        CommandData.roach_params[i].lo_freq_MHz = 750.0;
    }
    CommandData.roach_params[0].out_atten = 4;
    CommandData.roach_params[1].out_atten = 4;
    CommandData.roach_params[2].out_atten = 4;
    CommandData.roach_params[3].out_atten = 4;
    CommandData.roach_params[4].out_atten = 4;

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
    CommandData.power.bi0.rst_count = 0;
    CommandData.power.bi0.set_count = 0;
    // CommandData.power.rx_main.rst_count = 0;
    // CommandData.power.rx_main.set_count = 0;
    // CommandData.power.rx_hk.rst_count = 0;
    // CommandData.power.rx_hk.set_count = 0;
    // CommandData.power.rx_amps.rst_count = 0;
    // CommandData.power.rx_amps.set_count = 0;
    // CommandData.power.gybox_off = 0;
    // CommandData.power.gyro_off[0] = 0;
    // CommandData.power.gyro_off[1] = 0;
    // CommandData.power.gyro_off[2] = 0;
    // CommandData.power.gyro_off[3] = 0;
    // CommandData.power.gyro_off[4] = 0;
    // CommandData.power.gyro_off[5] = 0;
    // CommandData.power.hub232_off = 0;

    // CommandData.Cryo.BDAHeat = 0;

    CommandData.Cryo.potvalve_on = 1;
	CommandData.Cryo.valve_stop[0] = 0;
	CommandData.Cryo.valve_stop[1] = 0;
    CommandData.Cryo.valve_goals[0] = intermed;
    CommandData.Cryo.valve_goals[1] = intermed;
    CommandData.Cryo.potvalve_goal = intermed;

    // BLAST-Pol stuff
    // CommandData.Cryo.lhevalve_on = 0;
    // CommandData.Cryo.lvalve_open = 0;
    // CommandData.Cryo.lvalve_close =0;
    // CommandData.Cryo.lnvalve_on = 0;

    CommandData.uei_command.uei_of_dio_432_out = 0;
    /* don't use the fast gy offset calculator */
    CommandData.fast_offset_gy = 0;

    /* force autotrim to reset its wait time on restart */
    CommandData.autotrim_xsc0_last_bad = mcp_systime(NULL);
    CommandData.autotrim_xsc1_last_bad = CommandData.autotrim_xsc0_last_bad;

    CommandData.reset_rw = 0;
    CommandData.reset_piv = 0;
    CommandData.reset_elev = 0;
    CommandData.restore_piv = 0;

    CommandData.slot_sched = 0x100;
    CommandData.parts_sched = 0x0;
    CommandData.Cryo.do_cal_pulse = 0;
    CommandData.Cryo.do_level_pulse = 0;
    CommandData.Cryo.sync = 0;
    CommandData.Cryo.num_pulse = 1;
    CommandData.Cryo.separation = 1;
    CommandData.Cryo.periodic_pulse = 0;
    CommandData.Cryo.length = 1;

    /* Added for triggering cal lamp */
    CommandData.Cryo.num_pulse = 1;
    CommandData.Cryo.separation = 1;
    CommandData.Cryo.periodic_pulse = 0;
    CommandData.Cryo.length = 1;
    /* relays should always be set to zero when starting MCP */
    /* relays */
    CommandData.Relays.cycle_of_1 = 0;
    CommandData.Relays.cycle_of_2 = 0;
    CommandData.Relays.cycle_of_3 = 0;
    CommandData.Relays.cycle_of_4 = 0;
    CommandData.Relays.cycle_of_5 = 0;
    CommandData.Relays.cycle_of_6 = 0;
    CommandData.Relays.cycle_of_7 = 0;
    CommandData.Relays.cycle_of_8 = 0;
    CommandData.Relays.cycle_of_9 = 0;
    CommandData.Relays.cycle_of_10 = 0;
    CommandData.Relays.cycle_of_11 = 0;
    CommandData.Relays.cycle_of_12 = 0;
    CommandData.Relays.cycle_of_13 = 0;
    CommandData.Relays.cycle_of_14 = 0;
    CommandData.Relays.cycle_of_15 = 0;
    CommandData.Relays.cycle_of_16 = 0;
    CommandData.Relays.of_1_on = 0;
    CommandData.Relays.of_2_on = 0;
    CommandData.Relays.of_3_on = 0;
    CommandData.Relays.of_4_on = 0;
    CommandData.Relays.of_5_on = 0;
    CommandData.Relays.of_6_on = 0;
    CommandData.Relays.of_7_on = 0;
    CommandData.Relays.of_8_on = 0;
    CommandData.Relays.of_9_on = 0;
    CommandData.Relays.of_10_on = 0;
    CommandData.Relays.of_11_on = 0;
    CommandData.Relays.of_12_on = 0;
    CommandData.Relays.of_13_on = 0;
    CommandData.Relays.of_14_on = 0;
    CommandData.Relays.of_15_on = 0;
    CommandData.Relays.of_16_on = 0;
    CommandData.Relays.of_1_off = 0;
    CommandData.Relays.of_2_off = 0;
    CommandData.Relays.of_3_off = 0;
    CommandData.Relays.of_4_off = 0;
    CommandData.Relays.of_5_off = 0;
    CommandData.Relays.of_6_off = 0;
    CommandData.Relays.of_7_off = 0;
    CommandData.Relays.of_8_off = 0;
    CommandData.Relays.of_9_off = 0;
    CommandData.Relays.of_10_off = 0;
    CommandData.Relays.of_11_off = 0;
    CommandData.Relays.of_12_off = 0;
    CommandData.Relays.of_13_off = 0;
    CommandData.Relays.of_14_off = 0;
    CommandData.Relays.of_15_off = 0;
    CommandData.Relays.of_16_off = 0;
    CommandData.Relays.cycle_if_1 = 0;
    CommandData.Relays.cycle_if_2 = 0;
    CommandData.Relays.cycle_if_3 = 0;
    CommandData.Relays.cycle_if_4 = 0;
    CommandData.Relays.cycle_if_5 = 0;
    CommandData.Relays.cycle_if_6 = 0;
    CommandData.Relays.cycle_if_7 = 0;
    CommandData.Relays.cycle_if_8 = 0;
    CommandData.Relays.cycle_if_9 = 0;
    CommandData.Relays.cycle_if_10 = 0;
    CommandData.Relays.if_1_on = 0;
    CommandData.Relays.if_2_on = 0;
    CommandData.Relays.if_3_on = 0;
    CommandData.Relays.if_4_on = 0;
    CommandData.Relays.if_5_on = 0;
    CommandData.Relays.if_6_on = 0;
    CommandData.Relays.if_7_on = 0;
    CommandData.Relays.if_8_on = 0;
    CommandData.Relays.if_9_on = 0;
    CommandData.Relays.if_10_on = 0;
    CommandData.Relays.if_1_off = 0;
    CommandData.Relays.if_2_off = 0;
    CommandData.Relays.if_3_off = 0;
    CommandData.Relays.if_4_off = 0;
    CommandData.Relays.if_5_off = 0;
    CommandData.Relays.if_6_off = 0;
    CommandData.Relays.if_7_off = 0;
    CommandData.Relays.if_8_off = 0;
    CommandData.Relays.if_9_off = 0;
    CommandData.Relays.if_10_off = 0;
    CommandData.Relays.rec_on = 0;
    CommandData.Relays.rec_off = 0;
    CommandData.Relays.amp_supply_on = 0;
    CommandData.Relays.amp_supply_off = 0;
    CommandData.Relays.therm_supply_on = 0;
    CommandData.Relays.therm_supply_off = 0;
    CommandData.Relays.heater_supply_on = 0;
    CommandData.Relays.heater_supply_off = 0;
    CommandData.Relays.update_rec = 0;
    CommandData.Relays.update_of = 0;
    CommandData.Relays.update_if = 0;
    CommandData.Relays.cycled_of = 0;
    CommandData.Relays.cycled_if = 0;
    CommandData.Relays.labjack[0] = 0;
    CommandData.Relays.labjack[1] = 0;
    CommandData.Relays.labjack[2] = 0;
    CommandData.Relays.labjack[3] = 0;
    CommandData.Relays.labjack[4] = 0;
    CommandData.Labjack_Queue.lj_q_on = 0;
    CommandData.Labjack_Queue.set_q = 1;
    CommandData.Labjack_Queue.which_q[0] = 0;
    CommandData.Labjack_Queue.which_q[1] = 0;
    CommandData.Labjack_Queue.which_q[2] = 0;
    CommandData.Labjack_Queue.which_q[3] = 0;
    CommandData.Labjack_Queue.which_q[4] = 0;
    CommandData.Cryo.load_curve = 0;
    CommandData.Cryo.dac_value = 0;
    CommandData.Cryo.labjack = 0;
    CommandData.Cryo.send_dac = 0;
    CommandData.Cryo.cycle_allowed = 0;
    CommandData.Cryo.watchdog_allowed = 0;
    CommandData.Cryo.forced = 0;
    CommandData.Cryo.heater_update = 0;
    CommandData.Relays.update_video = 0;

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
    CommandData.Cryo.tcrit_fpa = 9900;
    CommandData.Relays.video_trans = 0;
    CommandData.command_count = 0;
    CommandData.last_command = 0xffff;

    CommandData.at_float = 0;
    CommandData.timeout = 3600;
    CommandData.slot_sched = 0;

    CommandData.highrate_bw = 6000/8.0; /* Bps */
    CommandData.pilot_bw = 8000000/8.0; /* Bps */
    CommandData.biphase_bw = 1000000/8.0; /* Bps */

    CommandData.highrate_allframe_fraction = 0.1;
    CommandData.pilot_allframe_fraction = 0.1;
    CommandData.biphase_allframe_fraction = 0.1;

    CommandData.biphase_clk_speed = 1000000; /* bps */
    CommandData.biphase_rnrz = false;
    CommandData.highrate_through_tdrss = true;
    copysvalue(CommandData.pilot_linklist_name, ALL_TELEMETRY_NAME);
    copysvalue(CommandData.bi0_linklist_name, "roach_noise_psd.ll");
    copysvalue(CommandData.highrate_linklist_name, "test3.ll");
    copysvalue(CommandData.sbd_linklist_name, "sbd.ll");
    CommandData.vtx_sel[0] = vtx_xsc0;
    CommandData.vtx_sel[1] = vtx_xsc1;
    CommandData.roach_tlm_mode = ROACH_TLM_IQDF;
    for (i = 0; i < NUM_ROACH_TLM; i++) {
      CommandData.roach_tlm[i].roach = 1;
      CommandData.roach_tlm[i].kid = 0;
      CommandData.roach_tlm[i].rtype= 0;
    }
    memset(CommandData.num_channels_all_roaches, 0, sizeof(CommandData.num_channels_all_roaches));
    CommandData.pilot_oth = 0;

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

    CommandData.ele_gain.I = 1.2;
    CommandData.ele_gain.P = 1000;
    CommandData.ele_gain.D = 0;
    CommandData.ele_gain.PT = 40;
    CommandData.ele_gain.DB = 0;
    CommandData.ele_gain.F = 0;

    CommandData.azi_gain.P = 2500;
    CommandData.azi_gain.I = 4;
    CommandData.azi_gain.PT = 125;

    CommandData.pivot_gain.SP = 30; // dps
    CommandData.pivot_gain.PV = 12;
    CommandData.pivot_gain.IV = 100;
    CommandData.pivot_gain.PE = 0;
    CommandData.pivot_gain.F = 0.0;

    CommandData.ec_devices.reset = 0;
    // By default don't try to fix the Ethercat devices to an operational state.
    CommandData.ec_devices.fix_rw = 0;
    CommandData.ec_devices.fix_el = 0;
    CommandData.ec_devices.fix_piv = 0;
    CommandData.ec_devices.fix_hwpr = 0;
    // /TODO: Re-enable El prior to flight
    CommandData.disable_az = 1;
    CommandData.disable_el = 1;

    CommandData.verbose_rw = 0;
    CommandData.verbose_el = 0;
    CommandData.verbose_piv = 0;

    CommandData.use_elenc = 0;
    CommandData.use_elmotenc = 1;
    CommandData.use_elclin = 1;
    CommandData.use_pss = 1;
    CommandData.use_dgps = 0;
    CommandData.use_xsc0 = 1;
    CommandData.use_xsc1 = 1;
    CommandData.use_mag1 = 1;
    CommandData.use_mag2 = 1;
    CommandData.lat_range = 1;
    CommandData.sucks = 1;
    CommandData.uplink_sched = 0;

    CommandData.clin_el_trim = 0;
    CommandData.enc_el_trim = 0;
    CommandData.enc_motor_el_trim = 0;
    CommandData.null_az_trim = 0;
    CommandData.mag_az_trim[0] = 0;
    CommandData.mag_az_trim[1] = 0;
    CommandData.pss_az_trim = 0;
    CommandData.dgps_az_trim = 0;

    CommandData.autotrim_enable = 0;
    CommandData.autotrim_thresh = 0.05;
    CommandData.autotrim_rate = 1.0;
    CommandData.autotrim_time = 60;

    CommandData.cal_xmax_mag[0] = 0.0115;
    CommandData.cal_ymax_mag[0] = -0.3527;
    CommandData.cal_xmin_mag[0] = 0.3835;
    CommandData.cal_ymin_mag[0] = 0.2059;
    CommandData.cal_mag_align[0] = 0.0;

    CommandData.cal_xmax_mag[1] = 0.0050;
    CommandData.cal_ymax_mag[1] = -0.337;
    CommandData.cal_xmin_mag[1] = 0.362;
    CommandData.cal_ymin_mag[1] = 0.1898;
    CommandData.cal_mag_align[1] = 0.0;

    CommandData.cal_az_pss[0] = 0.0;
    CommandData.cal_az_pss[1] = 0.0;
    CommandData.cal_az_pss[2] = 0.0;
    CommandData.cal_az_pss[3] = 0.0;
    CommandData.cal_az_pss[4] = 0.0;
    CommandData.cal_az_pss[5] = 0.0;

    CommandData.cal_d_pss[0] = 0.0;
    CommandData.cal_d_pss[1] = 0.0;
    CommandData.cal_d_pss[2] = 0.0;
    CommandData.cal_d_pss[3] = 0.0;
    CommandData.cal_d_pss[4] = 0.0;
    CommandData.cal_d_pss[5] = 0.0;

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

    for (i = 0; i < NUM_ROACHES; i++) {
        CommandData.udp_roach[i].store_udp = 1;
        CommandData.udp_roach[i].publish_udp = 1;
        // find_kids
        CommandData.roach_params[i].smoothing_scale = 1.0e4; // kHz
        CommandData.roach_params[i].peak_threshold = 1; // dB
        CommandData.roach_params[i].spacing_threshold = 100; // kHz
        // set_attens
        // these settings were determined on August 2, 2018 (Palestine)
        CommandData.roach_params[i].test_freq = 10.0125e6;
        CommandData.roach_params[i].atten_step = 1.0;
        CommandData.roach_params[i].npoints = 11;
        CommandData.roach_params[i].ncycles = 3;
        // For saving short timestream
        CommandData.roach_params[i].num_sec = 3.0;
        CommandData.roach_params[i].lo_offset = 1000.;
        CommandData.roach_params[i].delta_amp = 0.0;
        CommandData.roach_params[i].delta_phase = 0.0;
        CommandData.roach_params[i].freq_offset = 0.0;
        CommandData.roach_params[i].resp_thresh = 2000;
        CommandData.roach_params[i].dBm_per_tone = -60;
    }

    CommandData.rox_bias.amp = 56;
    CommandData.rox_bias.status = 0;
    CommandData.rox_bias.reset = 0;
    // TODO(laura): These are for the BLASTPol detector biasing and should be removed.
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

    CommandData.actbus.act_vel = 200;
    CommandData.actbus.act_acc = 1000;
    CommandData.actbus.act_move_i = 75;
    CommandData.actbus.act_hold_i = 10;
    CommandData.actbus.act_tol = 5;

    CommandData.actbus.lock_vel = 110000;
    CommandData.actbus.lock_acc = 100;
    CommandData.actbus.lock_move_i = 50;
    CommandData.actbus.lock_hold_i = 0;

    CommandData.hwpr.vel = 20000;
    CommandData.hwpr.acc = 1000;
    CommandData.hwpr.move_i = 20;
    CommandData.hwpr.hold_i = 0;

    CommandData.balance.vel = 1600;
    CommandData.balance.acc = 1000;
    CommandData.balance.move_i = 20;
    CommandData.balance.hold_i = 0;

    CommandData.balance.i_el_on_bal = 2.5;
    CommandData.balance.i_el_off_bal = 1.0;
    CommandData.balance.mode = bal_rest;

    CommandData.actbus.shutter_step = 4224;
    CommandData.actbus.shutter_step_slow = 300;
    CommandData.actbus.shutter_move_i = 40;
    CommandData.actbus.shutter_hold_i = 40;
    CommandData.actbus.shutter_vel = 5000;
    CommandData.actbus.shutter_acc = 1000;

    CommandData.Cryo.potvalve_opencurrent = 75;
    CommandData.Cryo.potvalve_closecurrent = 50;
    CommandData.Cryo.potvalve_hold_i = 0;
    CommandData.Cryo.potvalve_vel = 50000;
    CommandData.Cryo.potvalve_closed_threshold = 6000;
    CommandData.Cryo.potvalve_lclosed_threshold = 8000;
    CommandData.Cryo.potvalve_open_threshold = 12000;
    CommandData.Cryo.valve_vel = 50000;
    CommandData.Cryo.valve_move_i = 75;
    CommandData.Cryo.valve_hold_i = 0;
    CommandData.Cryo.valve_acc = 16;


    /* hwpr positions separated by 22.5 degs.
     entered by Barth on December 25, 2012 */
    CommandData.hwpr.pos[3] = 0.3418;
    CommandData.hwpr.pos[2] = 0.2168;
    CommandData.hwpr.pos[1] = 0.2779;
    CommandData.hwpr.pos[0] = 0.4047;

    CommandData.hwpr.overshoot = 300;
	CommandData.hwpr.backoff = 0.9;
    CommandData.hwpr.i_pos = 0;
    CommandData.hwpr.no_step = 0;
    CommandData.hwpr.use_pot = 1;
    CommandData.hwpr.pot_targ = 0.5;

    CommandData.pin_is_in = 1;

    // XY STAGE
    CommandData.xystage.x1 = 0;
    CommandData.xystage.y1 = 0;
    CommandData.xystage.x2 = 0;
    CommandData.xystage.y2 = 0;
    CommandData.xystage.step = 0;
    CommandData.xystage.xvel = 0;
    CommandData.xystage.yvel = 0;
    CommandData.xystage.is_new = 1;
    CommandData.xystage.mode = XYSTAGE_GOTO;
    CommandData.xystage.force_repoll = 0;

    CommandData.Cryo.hwprPos = 0;
    CommandData.Cryo.hwpr_pos_old = 0;
    CommandData.Cryo.cal_length = 30; /* = 150 ms @ 200Hz */
    CommandData.Cryo.calib_period = 3000; /* = 600 s @ 5Hz */ // write into periodic version
    CommandData.Cryo.calib_repeats = -1;  // indefinitely
    CommandData.Cryo.calib_hwpr = 1;  // pulse after every hwpr step
    CommandData.Cryo.level_length = 30;

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
        CommandData.XSC[which].is_new_window_period_cs = 1500;

        // CommandData.XSC[which].heaters.mode = xsc_heater_auto;
        CommandData.XSC[which].heaters.mode = xsc_heater_off;
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


    CommandData.Relays.of_relays[0] = 0;
    CommandData.Relays.of_relays[1] = 0;
    CommandData.Relays.of_relays[3] = 0;
    CommandData.Relays.of_relays[4] = 0;
    CommandData.Relays.of_relays[5] = 0;
    CommandData.Relays.of_relays[6] = 0;
    CommandData.Relays.of_relays[7] = 0;
    CommandData.Relays.of_relays[8] = 0;
    CommandData.Relays.of_relays[9] = 0;
    CommandData.Relays.of_relays[10] = 0;
    CommandData.Relays.of_relays[11] = 0;
    CommandData.Relays.of_relays[12] = 0;
    CommandData.Relays.of_relays[13] = 0;
    CommandData.Relays.of_relays[14] = 0;
    CommandData.Relays.of_relays[15] = 0;

    CommandData.Relays.if_relays[0] = 0;
    CommandData.Relays.if_relays[1] = 0;
    CommandData.Relays.if_relays[3] = 0;
    CommandData.Relays.if_relays[4] = 0;
    CommandData.Relays.if_relays[5] = 0;
    CommandData.Relays.if_relays[6] = 0;
    CommandData.Relays.if_relays[7] = 0;
    CommandData.Relays.if_relays[8] = 0;
    CommandData.Relays.if_relays[9] = 0;

    WritePrevStatus();
}
