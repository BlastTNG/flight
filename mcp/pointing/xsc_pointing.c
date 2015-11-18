/**
 * @file xsc_pointing.c
 *
 * @date Nov 5, 2015
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLAST project
 *
 * This software is copyright (C) 2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include <time.h>

#include <blast.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "channels_tng.h"
#include "tx.h"
#include "commands.h"
#include "command_struct.h"
#include "conversions.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "angles.h"
#include "xsc_network.h"

bool scan_entered_snap_mode;
bool scan_leaving_snap_mode;
bool scan_bypass_last_trigger_on_next_trigger;

typedef enum xsc_trigger_state_t
{
    xsc_trigger_first_time,
    xsc_trigger_waiting_to_send_trigger,
    xsc_trigger_sending_triggers,
    xsc_trigger_in_grace_period,
    xsc_trigger_in_hard_grace_period,
    xsc_trigger_readout_period
} xsc_trigger_state_t;

const char *xsc_trigger_file[2] = {  "/sys/class/gpio/gpio504/value",
                                "/sys/class/gpio/gpio505/value"};

static void xsc_trigger(int m_which, int m_value)
{
    const char val[2] = {'0', '1'};
    static int fd[2] = {-1, -1};

    if (fd[m_which] < 0) {
        if ((fd[m_which] = open (xsc_trigger_file[m_which], O_WRONLY)) < 0) {
            blast_strerror("Could not open %s for writing", xsc_trigger_file[m_which]);
            return;
        }
    }

    write(fd[m_which], &val[m_value], 1);
}

static void calculate_predicted_motion_px(double exposure_time)
{
    int i_point = GETREADINDEX(point_index);
    double standard_iplatescale = 6.680;
    double predicted_motion_deg = 0.0;
    predicted_motion_deg += PointingData[i_point].gy_total_vel * exposure_time;
    predicted_motion_deg += 0.5 * PointingData[i_point].gy_total_accel * exposure_time * exposure_time;
    for (unsigned int which=0; which<2; which++) {
        xsc_pointing_state[which].predicted_motion_px = to_arcsec(from_degrees(predicted_motion_deg)) / standard_iplatescale;
    }
}

static bool xsc_trigger_thresholds_satisfied()
{
    if (!CommandData.XSC[0].trigger.threshold.enabled) {
        return true;
    }
    if (xsc_pointing_state[0].predicted_motion_px < CommandData.XSC[0].trigger.threshold.blob_streaking_px) {
        return true;
    }
    return false;
}

static bool xsc_scan_force_grace_period()
{
    if (!CommandData.XSC[0].trigger.scan_force_trigger_enabled) {
        return false;
    }
    //blast_info("in xsc_scan_force_grace_period(): scan_entered_snap_mode is %d\n", scan_entered_snap_mode);
    return scan_entered_snap_mode;
}

static bool xsc_scan_force_trigger_threshold()
{
    if (!CommandData.XSC[0].trigger.scan_force_trigger_enabled) {
        return false;
    }
    //blast_info("in xsc_scan_force_trigger_threshold(): scan_leaving_snap_mode is %d\n", scan_leaving_snap_mode);
    return scan_leaving_snap_mode;
}

static void xsc_motion_psf_record_timestep(int index, xsc_trigger_state_t trigger_state, int state_counter, int* exposure_time_cs, int multi_trigger_counter)
{
    int i_point = GETREADINDEX(point_index);

    if (index < XSC_MOTION_PSF_MAX_NUM_TIMESTEPS) {
        for (int which=0; which<2; which++) {
            if (trigger_state == xsc_trigger_sending_triggers && state_counter < exposure_time_cs[which]) {
                CommandData.XSC[which].net.solver.motion_psf.timesteps[index].exposure_num = multi_trigger_counter;
            }
            CommandData.XSC[which].net.solver.motion_psf.timesteps[index].gy_az = from_degrees(PointingData[i_point].gy_az);
            CommandData.XSC[which].net.solver.motion_psf.timesteps[index].gy_el = from_degrees(PointingData[i_point].gy_el);
        }
        if (false) {
            for (int which=0; which<1; which++) {
                blast_info("CHAPPY DEBUG: x%i: recording timestep %i with exposure_num %i, gys %f %f (deg/s)", which, index,
                    CommandData.XSC[which].net.solver.motion_psf.timesteps[index].exposure_num,
                    to_degrees(CommandData.XSC[which].net.solver.motion_psf.timesteps[index].gy_az),
                    to_degrees(CommandData.XSC[which].net.solver.motion_psf.timesteps[index].gy_el));
            }
        }
    }
}

static void xsc_motion_psf_initiate(unsigned int* motion_psf_index)
{
    int i_point = GETREADINDEX(point_index);

    *motion_psf_index = 0;

    for (int which=0; which<2; which++) {
        CommandData.XSC[which].net.solver.motion_psf.counter_fcp = xsc_pointing_state[which].counter_fcp;
        CommandData.XSC[which].net.solver.motion_psf.counter_stars = XSC_SERVER_DATA(which).channels.ctr_stars;
        CommandData.XSC[which].net.solver.motion_psf.el = from_degrees(PointingData[i_point].el);
        for (unsigned int i=0; i<XSC_MOTION_PSF_MAX_NUM_TIMESTEPS; i++) {
            CommandData.XSC[which].net.solver.motion_psf.timesteps[i].exposure_num = -1;
        }
    }
}

static void xsc_motion_psf_finalize()
{
    for (int which=0; which<2; which++) {
        if (CommandData.XSC[which].net.solver.motion_psf.enabled) {
            xsc_activate_command(which, xC_motion_psf);
        }
        double first_exposure_motion_caz_px = 0.0;
        double first_exposure_motion_el_px = 0.0;
        double cos_el = cos(CommandData.XSC[which].net.solver.motion_psf.el);
        double platescale = 1.0 / CommandData.XSC[which].net.solver.motion_psf.iplatescale;
        for (unsigned int i=0; i<XSC_MOTION_PSF_MAX_NUM_TIMESTEPS; i++) {
            if (CommandData.XSC[which].net.solver.motion_psf.timesteps[i].exposure_num == 0) {
                first_exposure_motion_caz_px += CommandData.XSC[which].net.solver.motion_psf.timesteps[i].gy_az * platescale * 0.00998 * cos_el;
                first_exposure_motion_el_px  += CommandData.XSC[which].net.solver.motion_psf.timesteps[i].gy_el * platescale * 0.00998;
            }
        }
        if (false) {
            blast_info("CHAPPY DEBUG: x%i: motion_psf: first_exposure motion_caz %f and motion_y %f (px)", which,
                first_exposure_motion_caz_px, first_exposure_motion_el_px);
        }
        xsc_pointing_state[which].last_trigger.motion_caz_px = first_exposure_motion_caz_px;
        xsc_pointing_state[which].last_trigger.motion_el_px = first_exposure_motion_el_px;
    }
}

void xsc_control_triggers()
{
    static int state_counter = 0;

    static xsc_trigger_state_t trigger_state = xsc_trigger_first_time;
    static int max_exposure_time_used_cs = -1;
    int exposure_time_cs[2];
    int grace_period_cs = 300;

    int num_triggers = 1;
    int multi_trigger_time_between_triggers_cs = 18;
    static int multi_trigger_counter = 0;
    static bool recording_motion_psf = false;
    static unsigned int motion_psf_index = 0;


    int i_point = GETREADINDEX(point_index);

    grace_period_cs                        = CommandData.XSC[0].trigger.grace_period_cs;
    num_triggers                           = CommandData.XSC[0].trigger.num_triggers;
    multi_trigger_time_between_triggers_cs = CommandData.XSC[0].trigger.multi_trigger_time_between_triggers_cs;
    exposure_time_cs[0]                    = CommandData.XSC[0].trigger.exposure_time_cs;
    exposure_time_cs[1]                    = CommandData.XSC[1].trigger.exposure_time_cs;

    // this is where we don't trust CommandData
    limit_value_to_ints(&grace_period_cs, 300, 360000);
    limit_value_to_ints(&exposure_time_cs[0], 1, 500);
    limit_value_to_ints(&exposure_time_cs[1], 1, 500);
    limit_value_to_ints(&num_triggers, 1, INT32_MAX);

    if (recording_motion_psf) {
        xsc_motion_psf_record_timestep(motion_psf_index, trigger_state, state_counter, exposure_time_cs, multi_trigger_counter);
        motion_psf_index++;
    }
    double max_exposure_time_to_use = ((double) max(exposure_time_cs[0], exposure_time_cs[1]))/100.0;
    calculate_predicted_motion_px(max_exposure_time_to_use);

    xsc_pointing_state[0].last_trigger.age_cs++;
    xsc_pointing_state[1].last_trigger.age_cs++;
    xsc_pointing_state[0].last_trigger.age_of_end_of_trigger_cs++;
    xsc_pointing_state[1].last_trigger.age_of_end_of_trigger_cs++;
    switch (trigger_state)
    {
        case xsc_trigger_first_time:

            state_counter = 0;
            trigger_state = xsc_trigger_in_grace_period;
            /// Fall through
        case xsc_trigger_in_hard_grace_period:
            state_counter++;
//            blast_dbg("In hard grace period with counter %i", state_counter);
            if (state_counter > 60) {
                state_counter = 0;
                trigger_state = xsc_trigger_in_grace_period;
            }
            break;

        case xsc_trigger_in_grace_period:
            state_counter++;
//            blast_dbg("In grace period with counter %i", state_counter);
            if (state_counter > grace_period_cs || xsc_scan_force_grace_period()) {
                xsc_pointing_state[0].last_trigger.forced_grace_period = xsc_scan_force_grace_period();
                state_counter = 0;
                multi_trigger_counter = 0;
                trigger_state = xsc_trigger_waiting_to_send_trigger;
            }
            scan_entered_snap_mode = false;
            scan_leaving_snap_mode = false;
            break;

        case xsc_trigger_waiting_to_send_trigger:
            state_counter++;
            blast_dbg("In waiting_to_send with counter %i", state_counter);
            if (xsc_trigger_thresholds_satisfied() || (multi_trigger_counter > 0) || xsc_scan_force_trigger_threshold()) {
                xsc_pointing_state[0].last_trigger.forced_trigger_threshold = xsc_scan_force_trigger_threshold();
                if (!scan_bypass_last_trigger_on_next_trigger) {
                    xsc_pointing_state[0].last_trigger.age_cs = 0;
                    xsc_pointing_state[1].last_trigger.age_cs = 0;
                }
                max_exposure_time_used_cs = max(exposure_time_cs[0], exposure_time_cs[1]);
                for (int which=0; which<2; which++) {
                    xsc_trigger(which, 1);
                    if (!scan_bypass_last_trigger_on_next_trigger) {
                        xsc_pointing_state[which].last_trigger.counter_fcp = xsc_pointing_state[which].counter_fcp;
                        xsc_pointing_state[which].last_trigger.counter_stars = XSC_SERVER_DATA(which).channels.ctr_stars;
                        xsc_pointing_state[which].last_trigger.lat = PointingData[i_point].lat;
                        xsc_pointing_state[which].last_trigger.lst = PointingData[i_point].lst;
                    }
                }
                if (multi_trigger_counter == 0) {
                    recording_motion_psf = true;
                    xsc_motion_psf_initiate(&motion_psf_index);
                }

                state_counter = 0;
                trigger_state = xsc_trigger_sending_triggers;
            }
            scan_entered_snap_mode = false;
            scan_leaving_snap_mode = false;
            break;

        case xsc_trigger_sending_triggers:
            state_counter++;
            blast_dbg("In sending_triggers with counter %i, counter_fcp: %d", state_counter, xsc_pointing_state[0].counter_fcp);
            for (int which = 0; which < 2; which++) {
                if (state_counter >= exposure_time_cs[which]) xsc_trigger(which, 0);
            }
            if (state_counter >= max_exposure_time_used_cs) {
                state_counter = 0;
                multi_trigger_counter = (multi_trigger_counter+1) % num_triggers;
                if (multi_trigger_counter > 0) {
                    trigger_state = xsc_trigger_readout_period;
                } else {
                    scan_bypass_last_trigger_on_next_trigger = false;
                    trigger_state = xsc_trigger_in_hard_grace_period;
                    xsc_pointing_state[0].last_trigger.age_of_end_of_trigger_cs = 0;
                    xsc_pointing_state[1].last_trigger.age_of_end_of_trigger_cs = 0;
                    xsc_pointing_state[0].last_counter_fcp = xsc_pointing_state[0].counter_fcp;
                    xsc_pointing_state[1].last_counter_fcp = xsc_pointing_state[1].counter_fcp;
                    xsc_pointing_state[0].counter_fcp++;
                    xsc_pointing_state[1].counter_fcp = xsc_pointing_state[0].counter_fcp;
                    recording_motion_psf = false;
                    xsc_motion_psf_finalize();
                }
            }
            break;

        case xsc_trigger_readout_period:
            state_counter++;
            blast_dbg("In readout with counter %i", state_counter);
            if (state_counter >= (multi_trigger_time_between_triggers_cs-1)) {
                state_counter = 0;
                trigger_state = xsc_trigger_waiting_to_send_trigger;
            }
            break;

        default:
            blast_dbg("In default with counter %i", state_counter);
            state_counter = 0;
            trigger_state = xsc_trigger_first_time;
            break;
    }
}

static double xsc_get_temperature(int which)
{
    return XSC_SERVER_DATA(which).channels.hk_temp_lens;
}

void xsc_control_heaters(void)
{
    static channel_t* address[2];
    static bool first_time = true;
    static int setpoint_counter[2] = {0, 0};
    int setpoint_counter_threshold = 30*5; // 30 seconds
    bool heater_on = false;
    double temperature = 100.0;
    static double last_temperature = 100.0;
    static int counter_since_last_temperature_change[2] = {0, 0};
    static unsigned int periodic_10_second_counter = 0;

    if (first_time) {
        first_time = false;
        address[0] = channels_find_by_name("x0_heater");
        address[1] = channels_find_by_name("x1_heater");
    }
    periodic_10_second_counter = (periodic_10_second_counter + 1) % (10*5);

    for (int which = 0; which<2; which++) {
        heater_on = false;

        if (CommandData.XSC[which].heaters.mode == xsc_heater_on) {
            if (periodic_10_second_counter == 0) {
                blast_info("notice: xsc%i heaters on (mode xsc_heater_on)", which);
            }
            heater_on = true;
        }

        if (CommandData.XSC[which].heaters.mode == xsc_heater_auto) {

            // keep track of temperature and whether it is still being updated
            counter_since_last_temperature_change[which]++;
            temperature = xsc_get_temperature(which);
            if (temperature != last_temperature) {
                counter_since_last_temperature_change[which] = 0;
            }
            last_temperature = temperature;

            // if temperature is still being updated, set heaters on or off
            if (counter_since_last_temperature_change[which] < 5*60*5) { // 5 minutes
                if (temperature < CommandData.XSC[which].heaters.setpoint) {
                    if (setpoint_counter[which] > setpoint_counter_threshold) {
                        if (periodic_10_second_counter == 0) {
                            blast_info("notice: xsc%i heaters on (lens temp %f less than setpoint %f)",
                                which, temperature, CommandData.XSC[which].heaters.setpoint);
                        }
                        heater_on = true;
                    }
                    setpoint_counter[which]++;
                } else {
                    setpoint_counter[which] = 0;
                }
            }

        }

        if (heater_on) {
            SET_VALUE(address[which], 0x2);
        } else {
            SET_VALUE(address[which], 0x0);
        }
    }
}

