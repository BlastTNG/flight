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


#include "xsc_network.h"
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "blast.h"
#include "xsc_fifo.h"
#include "framing.h"
#include "channels_tng.h"
#include "tx.h"
#include "commands.h"
#include "command_struct.h"
#include "conversions.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "angles.h"

bool scan_entered_snap_mode = false;
bool scan_leaving_snap_mode = false;

static int32_t loop_counter = 0;
static xsc_fifo_t *trigger_fifo[2] = {NULL};

typedef enum xsc_trigger_state_t
{
    xsc_trigger_waiting_to_send_trigger,
    xsc_trigger_sending_triggers,
    xsc_trigger_in_grace_period,
    xsc_trigger_in_hard_grace_period,
    xsc_trigger_readout_period
} xsc_trigger_state_t;

const char *xsc_trigger_file[2] = {  "/sys/class/gpio/gpio504/value",
                                "/sys/class/gpio/gpio505/value"};

/**
 * Resets the GPIO state to our expected direction and pin enable.  This should
 * be taken care of by udev but there is a chance that something changes on the fly.
 * @return -1 if we cannot open the expected gpiochip, 0 otherwise
 */
static inline int xsc_initialize_gpio(void)
{
    int fd;
    const char *gpio_name[2] = { "504", "505"};
    const char *gpio_chip = "/sys/class/gpio/export";
    const char *gpio_direction[2] = { "/sys/class/gpio/gpio504/direction",
                                      "/sys/class/gpio/gpio505/direction"};

    if ((fd = open(gpio_chip, O_WRONLY)) < 0) {
        blast_strerror("Could not open %s for writing", gpio_chip);
        return -1;
    }
    write(fd, gpio_name[0], 4);
    fsync(fd);
    write(fd, gpio_name[1], 4);
    close(fd);

    for (int i = 0; i < 2; i++) {
        if ((fd = open(gpio_direction[i], O_WRONLY)) < 0) {
            blast_strerror("Could not open %s for writing", gpio_direction[i]);
            continue;
        }
        write(fd, "out", 4);
        close(fd);
    }
    return 0;
}

/**
 * Sets or resets the trigger value for each camera.  We have hard-wired the cameras to GPIO
 * pins 0 and 1 for XSC0 and XSC1.  Setting the value high results in ~3.1V differential on the
 * GPIO pin.
 * @param m_which Which GPIO pin
 * @param m_value Value (0 or 1) for the pin
 */
static void xsc_trigger(int m_which, int m_value)
{
    const char val[2] = {'0', '1'};
    static int fd[2] = {-1, -1};

    if (fd[m_which] < 0) {
        if ((fd[m_which] = open(xsc_trigger_file[m_which], O_WRONLY)) < 0) {
            blast_strerror("Could not open %s for writing", xsc_trigger_file[m_which]);
            return;
        }
    }

    if (write(fd[m_which], &val[m_value], 1) < 0) {
        (void)close(fd[m_which]);
        fd[m_which] = -1;
        blast_strerror("Could not write trigger %d to XSC%d", m_value, m_which);
        xsc_initialize_gpio();
    }
}

xsc_last_trigger_state_t *xsc_get_trigger_data(int m_which)
{
    xsc_last_trigger_state_t *last = xsc_fifo_pop(trigger_fifo[m_which]);
    return last;
}

static inline void xsc_store_trigger_data(int m_which, const xsc_last_trigger_state_t *m_state)
{
    if (!(trigger_fifo[m_which])) trigger_fifo[m_which] = xsc_fifo_new();
    xsc_last_trigger_state_t *stored = malloc(sizeof(xsc_last_trigger_state_t));
    memcpy(stored, m_state, sizeof(*m_state));
    if (!xsc_fifo_push(trigger_fifo[m_which], stored)) {
        free(stored);
    }
}

int32_t xsc_get_loop_counter(void)
{
    return loop_counter;
}

static void calculate_predicted_motion_px(double exposure_time)
{
    int i_point = GETREADINDEX(point_index);
    const double standard_iplatescale[2] = {6.66, 6.62}; // arcseconds per pixel
    double predicted_streaking_deg = 0.0;
    predicted_streaking_deg += PointingData[i_point].gy_total_vel * exposure_time;
    predicted_streaking_deg += 0.5 * PointingData[i_point].gy_total_accel * exposure_time * exposure_time;
    predicted_streaking_deg = fabs(predicted_streaking_deg);
    for (unsigned int which = 0; which < 2; which++) {
        xsc_pointing_state[which].predicted_streaking_px =
                to_arcsec(from_degrees(predicted_streaking_deg)) / standard_iplatescale[which];
        if (xsc_pointing_state[which].predicted_streaking_px > 6500.0) {
            xsc_pointing_state[which].predicted_streaking_px = 6500.0;
        }
    }
}

static bool xsc_trigger_thresholds_satisfied()
{
    if (!CommandData.XSC[0].trigger.threshold.enabled) {
        return true;
    }
    if (xsc_pointing_state[0].predicted_streaking_px < CommandData.XSC[0].trigger.threshold.blob_streaking_px) {
        return true;
    }
    return false;
}

static bool xsc_scan_force_grace_period()
{
    if (!CommandData.XSC[0].trigger.scan_force_trigger_enabled) {
        return false;
    }

    return scan_entered_snap_mode;
}

static bool xsc_scan_force_trigger_threshold()
{
    if (!CommandData.XSC[0].trigger.scan_force_trigger_enabled) {
        return false;
    }
    return scan_leaving_snap_mode;
}

void xsc_control_triggers()
{
    static int state_counter = 0;

    static xsc_trigger_state_t trigger_state = xsc_trigger_in_hard_grace_period;
    static int max_exposure_time_used_cs = -1;
    int exposure_time_cs[2];
    int grace_period_cs = 300;

    int num_triggers = 1;
    int multi_trigger_time_between_triggers_cs = 18;
    static int multi_trigger_counter = 0;
    static channel_t *xsc_trigger_channel = NULL;
    static uint8_t trigger = 0;

    int i_point = GETREADINDEX(point_index);

    if (!xsc_trigger_channel) {
    	xsc_trigger_channel = channels_find_by_name("trigger_xsc");
    }

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

    loop_counter++;

    double max_exposure_time_to_use = ((double) max(exposure_time_cs[0], exposure_time_cs[1]))/100.0;
    calculate_predicted_motion_px(max_exposure_time_to_use);

    state_counter++;
    switch (trigger_state) {
        case xsc_trigger_in_hard_grace_period:
            if (state_counter > 60) {
                state_counter = 0;
                trigger_state = xsc_trigger_in_grace_period;
            }
            break;

        case xsc_trigger_in_grace_period:
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
            // TODO(seth): Remove multiple trigger mode from STARS
            if (state_counter == 1) blast_dbg("Waiting to send trigger");
            if (xsc_trigger_thresholds_satisfied()
                    || (multi_trigger_counter > 0)
                    || xsc_scan_force_trigger_threshold()) {
                xsc_pointing_state[0].last_trigger.forced_trigger_threshold = xsc_scan_force_trigger_threshold();

                max_exposure_time_used_cs = max(exposure_time_cs[0], exposure_time_cs[1]);
                blast_dbg("Sending trigger with MCP Counter: %d", xsc_pointing_state[0].counter_mcp);
                for (int which = 0; which < 2; which++) {
                	trigger |= (1 << which);
                    xsc_trigger(which, 1);

                    xsc_pointing_state[which].last_trigger.counter_mcp = xsc_pointing_state[which].counter_mcp;
                    xsc_pointing_state[which].last_trigger.counter_stars =
                            XSC_SERVER_DATA(which).channels.ctr_stars;
                    xsc_pointing_state[which].last_trigger.lat = PointingData[i_point].lat;
                    xsc_pointing_state[which].last_trigger.lst = PointingData[i_point].lst;
                    xsc_pointing_state[which].last_trigger.trigger_time = get_100hz_framenum();
                    xsc_pointing_state[which].last_trigger_time = get_100hz_framenum();
                    xsc_store_trigger_data(which, &(xsc_pointing_state[which].last_trigger));
                }

                state_counter = 0;
                trigger_state = xsc_trigger_sending_triggers;
            }
            scan_entered_snap_mode = false;
            scan_leaving_snap_mode = false;
            break;

        case xsc_trigger_sending_triggers:
            for (int which = 0; which < 2; which++) {
                if (state_counter >= exposure_time_cs[which]) {
                    trigger &= (~(1 << which));
                	xsc_trigger(which, 0);
                }
            }
            if (state_counter >= max_exposure_time_used_cs) {
                state_counter = 0;
                multi_trigger_counter = (multi_trigger_counter+1) % num_triggers;
                if (multi_trigger_counter > 0) {
                    trigger_state = xsc_trigger_readout_period;
                } else {
                    trigger_state = xsc_trigger_in_hard_grace_period;
                    xsc_pointing_state[0].last_counter_mcp = xsc_pointing_state[0].counter_mcp;
                    xsc_pointing_state[1].last_counter_mcp = xsc_pointing_state[1].counter_mcp;
                    xsc_pointing_state[0].counter_mcp++;
                    xsc_pointing_state[1].counter_mcp = xsc_pointing_state[0].counter_mcp;
                }
            }
            break;

        case xsc_trigger_readout_period:
            if (state_counter >= (multi_trigger_time_between_triggers_cs-1)) {
                state_counter = 0;
                trigger_state = xsc_trigger_waiting_to_send_trigger;
            }
            break;

        default:
            state_counter = 0;
            trigger_state = xsc_trigger_in_hard_grace_period;
            break;
    }

    trigger = (trigger_state << 2) | (trigger & 0b11);
    SET_UINT8(xsc_trigger_channel, trigger);
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

    for (int which = 0; which < 2; which++) {
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

