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
 
 created by Ian Lowe 1-12-17
 **************************************************************************/
#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "labjack.h"
#include "blast.h"
#include "multiplexed_labjack.h"
#include "sensor_updates.h"

void update_sun_sensors(void) {
    static int firsttime = 1;
    static channel_t* ss_1_1_Addr;
    static channel_t* ss_1_2_Addr;
    static channel_t* ss_1_3_Addr;
    static channel_t* ss_1_4_Addr;
    static channel_t* ss_1_5_Addr;
    static channel_t* ss_1_6_Addr;
    static channel_t* ss_1_7_Addr;
    static channel_t* ss_1_8_Addr;
    static channel_t* ss_2_1_Addr;
    static channel_t* ss_2_2_Addr;
    static channel_t* ss_2_3_Addr;
    static channel_t* ss_2_4_Addr;
    static channel_t* ss_2_5_Addr;
    static channel_t* ss_2_6_Addr;
    static channel_t* ss_2_7_Addr;
    static channel_t* ss_2_8_Addr;
    static channel_t* ss_3_1_Addr;
    static channel_t* ss_3_2_Addr;
    static channel_t* ss_3_3_Addr;
    static channel_t* ss_3_4_Addr;
    static channel_t* ss_3_5_Addr;
    static channel_t* ss_3_6_Addr;
    static channel_t* ss_3_7_Addr;
    static channel_t* ss_3_8_Addr;
    static channel_t* ss_4_1_Addr;
    static channel_t* ss_4_2_Addr;
    static channel_t* ss_4_3_Addr;
    static channel_t* ss_4_4_Addr;
    static channel_t* ss_4_5_Addr;
    static channel_t* ss_4_6_Addr;
    static channel_t* ss_4_7_Addr;
    static channel_t* ss_4_8_Addr;
    static channel_t* ss_5_1_Addr;
    static channel_t* ss_5_2_Addr;
    static channel_t* ss_5_3_Addr;
    static channel_t* ss_5_4_Addr;
    static channel_t* ss_5_5_Addr;
    static channel_t* ss_5_6_Addr;
    static channel_t* ss_5_7_Addr;
    static channel_t* ss_5_8_Addr;
    if (firsttime == 1) {
        firsttime = 0;
        ss_1_1_Addr = channels_find_by_name("ss_1_1");
        ss_1_2_Addr = channels_find_by_name("ss_1_2");
        ss_1_3_Addr = channels_find_by_name("ss_1_3");
        ss_1_4_Addr = channels_find_by_name("ss_1_4");
        ss_1_5_Addr = channels_find_by_name("ss_1_5");
        ss_1_6_Addr = channels_find_by_name("ss_1_6");
        ss_1_7_Addr = channels_find_by_name("ss_1_7");
        ss_1_8_Addr = channels_find_by_name("ss_1_8");
        ss_2_1_Addr = channels_find_by_name("ss_2_1");
        ss_2_2_Addr = channels_find_by_name("ss_2_2");
        ss_2_3_Addr = channels_find_by_name("ss_2_3");
        ss_2_4_Addr = channels_find_by_name("ss_2_4");
        ss_2_5_Addr = channels_find_by_name("ss_2_5");
        ss_2_6_Addr = channels_find_by_name("ss_2_6");
        ss_2_7_Addr = channels_find_by_name("ss_2_7");
        ss_2_8_Addr = channels_find_by_name("ss_2_8");
        ss_3_1_Addr = channels_find_by_name("ss_3_1");
        ss_3_2_Addr = channels_find_by_name("ss_3_2");
        ss_3_3_Addr = channels_find_by_name("ss_3_3");
        ss_3_4_Addr = channels_find_by_name("ss_3_4");
        ss_3_5_Addr = channels_find_by_name("ss_3_5");
        ss_3_6_Addr = channels_find_by_name("ss_3_6");
        ss_3_7_Addr = channels_find_by_name("ss_3_7");
        ss_3_8_Addr = channels_find_by_name("ss_3_8");
        ss_4_1_Addr = channels_find_by_name("ss_4_1");
        ss_4_2_Addr = channels_find_by_name("ss_4_2");
        ss_4_3_Addr = channels_find_by_name("ss_4_3");
        ss_4_4_Addr = channels_find_by_name("ss_4_4");
        ss_4_5_Addr = channels_find_by_name("ss_4_5");
        ss_4_6_Addr = channels_find_by_name("ss_4_6");
        ss_4_7_Addr = channels_find_by_name("ss_4_7");
        ss_4_8_Addr = channels_find_by_name("ss_4_8");
        ss_5_1_Addr = channels_find_by_name("ss_5_1");
        ss_5_2_Addr = channels_find_by_name("ss_5_2");
        ss_5_3_Addr = channels_find_by_name("ss_5_3");
        ss_5_4_Addr = channels_find_by_name("ss_5_4");
        ss_5_5_Addr = channels_find_by_name("ss_5_5");
        ss_5_6_Addr = channels_find_by_name("ss_5_6");
        ss_5_7_Addr = channels_find_by_name("ss_5_7");
        ss_5_8_Addr = channels_find_by_name("ss_5_8");
    }
}
