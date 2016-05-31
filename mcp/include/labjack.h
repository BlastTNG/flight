/**
 * @file data_sharing.h
 *
 * @date Dec 25, 2012
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2016 University of Pennsylvania
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

#ifndef INCLUDE_LABJACK_H_
#define INCLUDE_LABJACK_H_

#include <stdint.h>

#define LABJACK_CRYO 0

void labjack_networking_init(int m_which, size_t m_numchannels);
uint16_t labjack_get_value(int m_labjack, int m_channel);

#endif /* LABJACK_H_ */
