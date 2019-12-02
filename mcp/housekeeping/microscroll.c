/***************************************************************************
 mcp: the BLAST master control program
 
 This software is copyright (C) 2002-2006 University of Toronto
 
 This file is part of mcp.
 
 mcp is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 at your option) any later version.
 
 mcp is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with mcp; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 
 created by Ian Lowe 11-29-19
 **************************************************************************/


/*************************************************************************
 
 microscroll.c code to control the power relays and thermistors for the micro
 scroll pump
 
 *************************************************************************/

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "blast.h"




typedef struct {
    float pump_1_on, pump_1_off, pump_2_on, pump_2_off;
    float aalborg_supply_on, aalborg_supply_off;
} microscroll_control_t;

#define thermistor_1 = 6;
#define thermistor_2 = 7;
#define thermistor_3 = 8;
#define thermistor_4 = 9;
#define thermistor_5 = 10;
#define thermistor_6 = 11;
#define thermistor_7 = 12;
#define thermistor_8 = 13;

