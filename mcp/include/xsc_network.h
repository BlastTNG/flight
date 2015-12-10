/**
 * @file xsc_network.h
 *
 * @date Sep 23, 2015
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
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

#ifndef INCLUDE_XSC_NETWORK_H
#define INCLUDE_XSC_NETWORK_H

#include <xsc_protocol.h>
#include <stdbool.h>

#define XSC_SERVER_DATA(_which) xsc_mserver_data[_which][GETREADINDEX(xsc_server_index[_which])]

extern XSCServerData xsc_mserver_data[2][3];
extern int xsc_server_index[2];

void xsc_networking_init(int which);
void xsc_write_data(int which);

#endif
