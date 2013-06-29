/* sys.h: Various Linux system routines
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "blast.h"
/* Synchronously execute a process, with error handling.  The process's STDIN
 * and STDOUT are set to /dev/null.  The process's STDERR is piped into the
 * BUOS log.  Waits for termination and/or timeout.
 *
 * Inputs:
 *    elev: the BUOS level to use for the process's STDERR
 *    path: path to the executable
 *    argv: the process's command line.  The first element in this list should
 *          be the name of the program (usually equal to path).  The string
 *          list should be terminated by a NULL pointer.  As a convenience,
 *          executables which need no arguments may pass NULL here, and
 *          a suitable command line will be automatically created.  (see
 *          execve(2)).
 *    timeout: the number of seconds to wait before forcibly terminating the
 *             process, or zero to wait forever.  Note: this timeout is
 *             approximate; this function may wait longer than the time
 *             specified.
 *    announce: if non-zero, report the name of the process being executed
 *              and its return value at BUOS level "info".
 *
 * Return value:
 *    the exit status of the process.  See wait(2).
 */
int exec_and_wait(buos_t elev, const char *path, char *argv[], unsigned timeout,
    int announce);

/* mount something listed in the fstab called "name", which can either be a 
 * mount point or a device name. There's also a timeout in seconds (0 = forever)
 * returns non-zero on error.
 */
int do_mount(const char *name, int timeout);

/* lazy unmount something; returns non-zero on error */
int do_umount(const char *target);
