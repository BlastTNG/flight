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
 
 created by Ian Lowe 5-13-16
 **************************************************************************/


/*************************************************************************
 
 crystat.c -- mcp code to handle cryostat control. Including heaters and
 fridge cycles.
 
 *************************************************************************/

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"

/* Heater control bits (BIAS_D G4) */
#define HEAT_HELIUM_LEVEL    0x0001
#define HEAT_CHARCOAL        0x0002
#define HEAT_POT_HS          0x0004
#define HEAT_CHARCOAL_HS     0x0008
#define HEAT_UNDEF           0x0010
#define HEAT_BDA             0x0020
#define HEAT_CALIBRATOR      0x0040
#define HEAT_HWPR_POS        0x0080

static uint16_t heatctrl;

/*************************************************************************/
/* CryoControl: Control valves, heaters, and calibrator (a fast control) */
/*************************************************************************/
void cryo_control(void)
{
    heatctrl = 0;
    if (CommandData.Cryo.charcoalHeater)
        heatctrl |= HEAT_CHARCOAL;
}

void store_100hz_cryo(void)
{
    static int firsttime = 1;

    static channel_t* heaterAddr;

    if (firsttime) {
        heaterAddr = channels_find_by_name("dio_heaters");
        firsttime = 0;
    }
    SET_UINT16(heaterAddr, heatctrl);
}

/*void autocycle(void)
{
    static channel_t* tfpa250_Addr; set channel address pointers
    static channel_t* tfpa350_Addr;
    static channel_t* tfpa500_Addr;
    static channel_t* tcharcoal_Addr;
    
    static int firsttime = 1;
    static int iterator = 0;
    double t250, t350, t500, tcharcoal;
    static double tcrit = 0.31;
    static int trigger = 0;
    
    if (firsttime) {
        tfpa250_Addr = channels_find_by_name("PLACEHOLDER_250um");  these three are ROX
        tfpa350_Addr = channels_find_by_name("PLACEHOLDER_350um");
        tfpa500_Addr = channels_find_by_name("PLACEHOLDER_500um");
        tcharcoal_Addr = channels_find_by_name("PLACEHOLDER_CHARCOAL") diode
        firsttime = 0;
    }
    
    t250 = GET_SCALED_VAL(tfpa250_Addr);
    t350 = GET_SCALED_VAL(tfpa350_Addr);
    t500 = GET_SCALED_VAL(tfpa500_Addr);
    tcharcoal = GET_SCALED_VAL(tcharcoal_Addr);
    if (t250 > tcrit) {
        if (!trigger) {
            HEAT_CHARCOAL_HS = 0;
            trigger = 1;
            goto fridge_auto_cycle;
        }
    }
    if (t350 > tcrit) {
        if (!trigger) {
            HEAT_CHARCOAL_HS = 0;
            trigger = 1;
            goto fridge_auto_cycle;
        }
    }
    if (t500 > tcrit) {
        if (!trigger) {
            HEAT_CHARCOAL_HS = 0;
            trigger = 1;
            goto fridge_auto_cycle;
        }
    }
fridge_auto_cycle:
    if (trigger) {
        if (!(iterator++ % 199)) { borrowed from das.c, if this command is run at 100hz, this slows it down to 0.5 hz
            t250 = GET_SCALED_VAL(tfpa250_Addr);
            t350 = GET_SCALED_VAL(tfpa350_Addr); commented out because current implementation looks only at charcoal temperature
            t500 = GET_SCALED_VAL(tfpa500_Addr);
            tcharcoal = GET_SCALED_VAL(tcharcoal_Addr);
        }
    }
}
*/
