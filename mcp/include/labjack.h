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

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

#define LABJACK_CRYO_1 0
#define LABJACK_CRYO_2 1
#define LABJACK_OF_1 2
#define LABJACK_OF_2 3
#define LABJACK_OF_3 4
#define LABJACK_HIGHBAY 7
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
#define BIAS 10
#define LEVEL_SENSOR_READ 11
#define CAL_LAMP_READ 12
#define HEATER_300MK_READ 13
#define ROX_FPA_1K 2
#define ROX_250_FPA 3
#define ROX_350_FPA 4
#define ROX_500_FPA 5
#define ROX_300MK_STRAP 6
#define ROX_1K_STRAP 7
#define ROX_HE3_FRIDGE 8
#define ROX_HE4_POT 9


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
#define POWER_BOX_ON 2001
#define POWER_BOX_OFF 2000
#define AMP_SUPPLY_ON 2003
#define AMP_SUPPLY_OFF 2002
#define THERM_READOUT_ON 2005
#define THERM_READOUT_OFF 2004
#define HEATER_SUPPLY_ON 2007
#define HEATER_SUPPLY_OFF 2006

// Digital reads on LJ CRYO 2
#define READ_CHARCOAL 2009
#define READ_250LNA 2010
#define READ_1K_HEATER 2011
#define READ_CHARCOAL_HS 2013
#define READ_350LNA 2015
#define READ_500LNA 2016

// DACS
#define DAC0 1000
#define DAC1 1002

// These defines specify with AIN voltage on the cyro labjack reads out which diode or ROX channel

void labjack_networking_init(int m_which, size_t m_numchannels, size_t m_scans_per_packet);
float labjack_get_value(int m_labjack, int m_channel);
ph_thread_t* initialize_labjack_commands(int m_which);
void store_labjack_data(void);
void labjack_test_dac(float v_value, int m_labjack);
int labjack_dio(int m_labjack, int address, int command);
void heater_write(int m_labjack, int address, float command);
uint16_t labjack_read_dio(int m_labjack, int address);
void labjack_reboot(int m_labjack);
void labjack_queue_command(int, int, float);
void query_time(int m_labjack);
void initialize_labjack_queue(void);
void labjack_choose_execute(void);
void init_labjack_digital(void);
void init_labjacks(int set_1, int set_2, int set_3, int set_4, int set_5, int q_set);
#endif /* LABJACK_H_ */
