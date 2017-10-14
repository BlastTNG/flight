/***************************************************************************
 mcp: the BLAST master control program
 
 This software is copyright (C) 2002-2006 University of Pennsylvania
 
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
 
 created by Ian Lowe 5-13-16
 **************************************************************************/


/*************************************************************************
 
 highbay.c -- code to control and log the meters in the highbay
 
 *************************************************************************/

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "highbay.h"

static void aalborg_n2(void) {
    static channel_t* flow_n2_Addr;
    static int first_time_n2 = 1;
    if (first_time_n2) {
        flow_n2_Addr = channels_find_by_name("n2_flow_v");
        first_time_n2 = 0;
    }
    // SET_SCALED_VALUE(flow_n2_Addr, labjack_get_value(LABJACK_HIGHBAY, N2_FLOW_CHAN));
}

static void aalborg_he_blow(void) {
    static channel_t* he_blow_Addr;
    static int first_time_he_blow = 1;
    if (first_time_he_blow) {
        he_blow_Addr = channels_find_by_name("he_blow_v");
        first_time_he_blow = 0;
    }
    // SET_SCALED_VALUE(he_blow_Addr, labjack_get_value(LABJACK_HIGHBAY, HE_BLOW_CHAN));
}

static void aalborg_he_pot(void) {
    static channel_t* flow_he_pot_Addr;
    static int first_time_he_pot = 1;
    if (first_time_he_pot) {
        flow_he_pot_Addr = channels_find_by_name("he_pot_flow_v");
        first_time_he_pot = 0;
    }
    // SET_SCALED_VALUE(flow_he_pot_Addr, labjack_get_value(LABJACK_HIGHBAY, HE_POT_FLOW_CHAN));
}


void highbay(int n2, int he_pot, int he_blow) {
    if (n2) {
        aalborg_n2();
    }
    if (he_pot) {
        aalborg_he_pot();
    }
    if (he_blow) {
        aalborg_he_blow();
    }
}
