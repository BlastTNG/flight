/*
 * watchdog.c:
 *
 * This software is copyright (C) 2013-2015 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 * History:
 *  Created on: Oct 30, 2015
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/watchdog.h>

#include "watchdog.h"
#include "biphase_hardware.h"
#include "blast.h"
#include "channels_tng.h"
#include "mputs.h"
#include "command_struct.h"

#define BI0_INCHARGE_CALL_PERIOD 250000 // Number of microseconds between in charge calls.
#define WATCHDOG_CTRL_INIT_TIMEOUT 10   // Wait n calls before we actually decide whether we are in charge.
#define WATCHDOG_CTRL_MIN_COUNT 4      // Need n consistent calls before we change in charge state

extern int16_t SouthIAm;
extern int16_t InCharge;
extern bool shutdown_mcp;
void close_mcp(int);
static const char watchdog_magic = 'V';

static int watchdog_fd = -1;
static int comms_card_tickle = 1;

static int stop_watchdog_flag = 0;


/**
 * Sets a flag to stop triggering the watchdog tickle
 */
void watchdog_stop() {
    stop_watchdog_flag = 1;
}

/**
 * Resets the watchdog timer.  Should be called liberally relative to the timeout period.
 */
void watchdog_ping()
{
    int dummy;
    // Check to make sure we have not been ordered to reap the fc.
    if (!stop_watchdog_flag) {
        if (watchdog_fd != -1) {
            ioctl(watchdog_fd, WDIOC_KEEPALIVE, &dummy);
        }
        comms_card_tickle ^= 1;
    } else {
      usleep(1000);
    }
}

/**
 * Returns the current bit value of the tickle pin for the communication card control
 */
int watchdog_get_tickle(void)
{
    return comms_card_tickle;
}

/**
 * Close the watchdog in such a manner as to prevent it from resetting the computer after closed.
 * The 'V' character is the Magic Byte that tells the watchdog that we really meant to close it
 * nicely.
 */
void watchdog_close(void)
{
    if (watchdog_fd != -1) {
        write(watchdog_fd, &watchdog_magic, 1);
        close(watchdog_fd);
    }
    watchdog_fd = -1;
}

/**
 * Setup the watchdog timer to begin monitoring the system.
 * @param m_timeout Number of seconds after which the system will power cycle if the watchdog
 *          hasn't received a ping
 * @return 0 on success, -1 on failure
 */
int initialize_watchdog(int m_timeout)
{
    if (watchdog_fd > 0) close(watchdog_fd);

    if ((watchdog_fd = open("/dev/watchdog1", O_WRONLY)) < 0) {
        blast_strerror("Could not open Watchdog");
        return -1;
    }

    ioctl(watchdog_fd, WDIOC_SETTIMEOUT, &m_timeout);
    watchdog_ping();
    return 0;
}

/******* In Charge Functions *********/

void set_incharge(int in_charge_from_wd) {
    static int first_call = 1;
    static int incharge_old=-1;
    static int init_timeout = WATCHDOG_CTRL_INIT_TIMEOUT+1;
    static int incharge_count = 0;
    int in_charge = incharge_old;

    if (first_call == 1) {
        blast_info("Called set_incharge for the first time");
        first_call = 0;
    }

    /*
    static struct timeval call_time;
    struct timeval now;
    gettimeofday(&now, NULL);
    blast_info("set_incharge called after %f s", (now.tv_sec+(now.tv_usec/1000000.0) -
                                    (call_time.tv_sec+(call_time.tv_usec/1000000.0))));
    gettimeofday(&call_time, NULL);
    */

    // Handle initialization timeout.
    // Return early if we have not satisfied the timeout on initialization.
    if (init_timeout > 0) {
        init_timeout--;
        return;
    }

    // Handle the in charge state from the watchdog with a hysteresis filter.
    // If this fc is set as in charge for at least WATCHDOG_CTRL_MIN_COUNT, then it is in charge.
    if (in_charge_from_wd == SouthIAm) { // watchdog says in charge
        if (incharge_count >= WATCHDOG_CTRL_MIN_COUNT) {
            // reached minimum counts, so set in_charge
            in_charge = in_charge_from_wd;
        } else {
            // have not reach min counts, so increment
            incharge_count++;
        }
    } else { // watchdog says not in charge
        if (incharge_count <= 0) {
            // reached zero counts, so set to not in charge
            in_charge = in_charge_from_wd;
        } else {
            // have not reached zero counts, so decrement
            incharge_count--;
        }
    }

    // blast_warn("in_charge = %d, incharge_old = %d, SouthIAm = %d", in_charge, incharge_old, SouthIAm);
    // After filtering, handle whether or not we are actually in charge.
    if (in_charge == SouthIAm) {
        // We're in charge!
        // set incharge here to 1 if the && comes true
        InCharge = 1;
        if (incharge_old != in_charge) {
            if (SouthIAm == 1) {
                blast_info("I, South, have now gained control");
            } else {
                blast_info("I, North, have now gained control");
            }
            CommandData.actbus.force_repoll = 1;
        }
    } else {
        /*
        // if you had been in charge, then you lost control and should reset
        if (InCharge) {
            blast_info("Had control and lost it. Shutting down.\n");
            close_mcp(0);
        }
        */

        InCharge = 0;
        if (incharge_old != in_charge) {
            if (SouthIAm == 1) {
                blast_info("I, South, have lost control");
            } else {
                blast_info("I, North, have lost control");
            }
        }
    }
    incharge_old = in_charge;
}
