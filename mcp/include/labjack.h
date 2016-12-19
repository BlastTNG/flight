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
#define LEVEL_SENSOR 11
#define CAL_LAMP 12
#define LNA_250 13
#define ROX_FPA_1K 2

// DIO addresses LJ CRYO 1
#define LEVEL_SENSOR_COMMAND 2008
#define CHARCOAL_COMMAND 2009
#define LNA_250_COMMAND 2010
#define HEATER_1K_COMMAND 2011
#define HEATER_300MK_COMMAND 2012
#define CHARCOAL_HS_COMMAND 2013
#define CALLAMP_COMMAND 2014
#define LNA_350_COMMAND 2015
#define LNA_500_COMMAND 2016

// DIO addresses LJ CRYO 2
#define SWITCH_5V_ON 2000
#define SWITCH_5V_OFF 2001
#define SWITCH_12V_ON 2002
#define SWITCH_12V_OFF 2003
#define SWITCH_15V_ON 2004
#define SWITCH_15V_OFF 2005
#define SWITCH_40V_ON 2006
#define SWITCH_40V_OFF 2007

// These defines specify with AIN voltage on the cyro labjack reads out which diode or ROX channel
// TODO(ian): Update these for the thermometers and channels we have.
#define LJ_R_FPA_1K_IND 2

void labjack_networking_init(int m_which, size_t m_numchannels, size_t m_scans_per_packet);
float labjack_get_value(int m_labjack, int m_channel);
void initialize_labjack_commands(int m_which);
void store_labjack_data(void);
int labjack_dio(int m_labjack, int address, int command);
void heater_write(int m_labjack, int address, int command);
uint16_t labjack_read_dio(int m_labjack, int address);
#endif /* LABJACK_H_ */
