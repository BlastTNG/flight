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
#include "mpsse.h"
#include "channels_tng.h"


#define BI0_INCHARGE_CALL_PERIOD 250000 // Number of microseconds between in charge calls.
#define WATCHDOG_CTRL_INIT_TIMEOUT 10   // Wait 10 calls before we actually decide whether we are in charge.

extern int16_t SouthIAm;
extern int16_t InCharge;

static const char watchdog_magic = 'V';

static int watchdog_fd = -1;
static int comms_card_tickle = 1;

/**
 * Resets the watchdog timer.  Should be called liberally relative to the timeout period.
 */
void watchdog_ping()
{
    int dummy;
    if (watchdog_fd != -1) {
        ioctl(watchdog_fd, WDIOC_KEEPALIVE, &dummy);
    }
    comms_card_tickle ^= 1;
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

static void set_incharge(struct mpsse_ctx *ctx_passed_read) {
    static int first_call = 1;
    int in_charge=-1;
    static int incharge_old=-1;
    static channel_t* incharge_Addr;
    static int init_timeout = WATCHDOG_CTRL_INIT_TIMEOUT;
    if (first_call == 1) {
        blast_info("Called set_incharge for the first time");
        incharge_Addr = channels_find_by_name("incharge");
        first_call = 0;
    } else if (init_timeout > 0) {
        init_timeout--;
    } else {
        in_charge = mpsse_watchdog_get_incharge(ctx_passed_read);
//        blast_warn("in_charge = %d, incharge_old = %d, SouthIAm = %d", in_charge, incharge_old, SouthIAm);
        SET_SCALED_VALUE(incharge_Addr, in_charge);
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
            }
        } else {
            InCharge = 0;
            if (incharge_old != in_charge) {
                if (SouthIAm == 1) {
                    blast_info("I, South, have lost control");
                } else {
                    blast_info("I, North, have lost control");
                }
            }
        }
    }
    incharge_old = in_charge;
}

void watchdog_ping_and_set_in_charge(void) {
    struct mpsse_ctx *ctx = NULL;
    const char *serial = NULL;
    uint8_t direction = 0x00; // All pins set to read (we only use pin 6)

    nameThread("Watchdog");

    if (!SouthIAm) {
        serial = "FC1NS9HU"; // "FC1"
    } else {
        serial = "?";
    }
    setup_mpsse(&ctx, serial, direction);
    while (true) {
        mpsse_watchdog_ping(ctx);
        set_incharge(ctx);
        usleep(100000); // 10 Hz
    }
}
