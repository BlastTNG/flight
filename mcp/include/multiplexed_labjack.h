/**
 * @file multiplxed_labjack.h
 *
 * @date   Jan 2, 2017
 * @author Ian
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2017 University of Pennsylvania
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

#ifndef INCLUDE_MULTIPLEXED_LABJACK_H_
#define INCLUDE_MULTIPLEXED_LABJACK_H_

#include <stdint.h>

#define LABJACK_OF_NCHAN 84 // Number of Channels to stream (14 = all analog input channels)
#define LABJACK_OF_SPP 1 // Number of scans to readout per streaming packet

void mult_labjack_networking_init(int m_which, size_t m_numchannels, size_t m_scans_per_packet);
void mult_initialize_labjack_commands(int m_which);
void query_mult(int m_labjack, int m_channel);
#endif /* MULTIPLEXED_LABJACK_H_ */
