/*
 * watchdog.h:
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
#ifndef INCLUDE_WATCHDOG_H_
#define INCLUDE_WATCHDOG_H_


void watchdog_ping();
void watchdog_close(void);
int initialize_watchdog(int m_timeout);

#endif /* INCLUDE_WATCHDOG_H_ */
