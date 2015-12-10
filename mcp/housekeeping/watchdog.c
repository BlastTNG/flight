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

#include "blast.h"

static const char watchdog_magic = 'V';

static int watchdog_fd = -1;

/**
 * Resets the watchdog timer.  Should be called liberally relative to the timeout period.
 */
void watchdog_ping()
{
    int dummy;
    ioctl(watchdog_fd, WDIOC_KEEPALIVE, &dummy);
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

    if ((watchdog_fd = open("/dev/watchdog", O_WRONLY)) < 0) {
        blast_strerror("Could not open Watchdog");
        return -1;
    }

    ioctl(watchdog_fd, WDIOC_SETTIMEOUT, &m_timeout);
    watchdog_ping();
    return 0;
}
