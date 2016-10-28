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

#define LABJACK_CRYO_1 0
#define LABJACK_CRYO_2 1
#define LABJACK_CRYO_NCHAN 14 // Number of Channels to stream (14 = all analog input channels)
#define LABJACK_CRYO_SPP 1 // Number of scans to readout per streaming packet

// Define all of the cryo channels here
// labjack 1
#define DIODE_VCS2_FILT 3
#define DIODE_250FPA 0
#define DIODE_HWP 9
#define DIODE_VCS1_HX 6
#define DIODE_1K_FRIDGE 12
#define DIODE_VCS1_FILT 2
#define DIODE_M3 8
#define DIODE_OB_FILTER 5
#define DIODE_VCS2_PLATE 11
#define DIODE_M4 1
#define DIODE_4K_FILT 4
#define DIODE_VCS2_HX 7
#define DIODE_VCS1_PLATE 10
#define DIODE_CHARCOAL_HS 13
// labjack 2
#define DIODE_CHARCOAL 0
#define DIODE_4K_PLATE 1

// These defines specify with AIN voltage on the cyro labjack reads out which diode or ROX channel
// TODO(ian): Update these for the thermometers and channels we have.
#define LJ_R_FPA_1K_IND 2

void labjack_networking_init(int m_which, size_t m_numchannels, size_t m_scans_per_packet);
float labjack_get_value(int m_labjack, int m_channel);
void initialize_labjack_commands(int m_which);
void store_labjack_data(void);
int labjack_dio(int m_labjack, int address, int command);

#endif /* LABJACK_H_ */
