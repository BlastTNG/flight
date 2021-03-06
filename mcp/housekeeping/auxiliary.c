/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/statvfs.h>

#include "mcp.h"
#include "mputs.h"

#include "therm_heater.h"
#include "channels_tng.h"
#include "tx.h"
#include "command_struct.h"
#include "pointing_struct.h"
#include "chrgctrl.h"
#include "lut.h"

extern int16_t InCharge; /* tx.c */

/* ACS2 digital signals */
#define BAL_DIR      0x01  /* ACS2 Group 2 Bit 1 */
#define BAL_VALV     0x02  /* ACS2 Group 2 Bit 2 */
#define BAL_HEAT     0x04  /* ACS2 Group 2 Bit 3 - DAC */

#define PUMP_MAX 26214      /*  3.97*2.0V   */
#define PUMP_MIN  3277      /*  3.97*0.25V   */

#define PUMP_ZERO 32773


/************************************************************************/
/*    ControlPumpHeat:  Controls balance system pump temp               */
/************************************************************************/

// static int ControlPumpHeat(int bits_bal)
// {
//   static channel_t *tBoxBalAddr, *vtPumpBalAddr;
//   static struct LutType tPumpBalLut =
//      {"/data/etc/blast/thermistor.lut", 0, NULL, NULL, 0};
//   static int firsttime = 1;
//
//   double temp1, temp2;
//
//   if (firsttime) {
//     firsttime = 0;
//     tBoxBalAddr = channels_find_by_name("t_box_bal");
//     vtPumpBalAddr = channels_find_by_name("vt_pump_bal");
//     LutInit(&tPumpBalLut);
//   }
//
//
//   temp1 = (double)GET_UINT16(tBoxBalAddr);
//   temp2 = (double)GET_UINT16(vtPumpBalAddr);
//
//   temp1 = calibrate_ad590(temp1) - 273.15;
//   temp2 = calibrate_thermister(temp2) - 273.15;
//
//   if (CommandData.pumps.heat_on) {
//     if (temp1 < CommandData.pumps.heat_tset) {
//       bits_bal |= BAL_HEAT;  /* set heat bit */
//     } else {
//       bits_bal &= (0xFF - BAL_HEAT); /* clear heat bit */
//     }
//   } else {
//       bits_bal &= (0xFF - BAL_HEAT); /* clear heat bit */
//   }
//
//   return bits_bal;
// }
//

/*****************************************************************/
/*                                                               */
/*   Control the pumps                                           */
/*                                                               */
/*****************************************************************/
// void ControlAuxMotors()
// {
//   static channel_t* levelOnBalAddr, *levelOffBalAddr;
//   static channel_t* levelTargetBalAddr;
//   static channel_t* gainBalAddr;
//   static channel_t* bitsBalAddr;
//
//   int bits_bal = 0;
//
//   static int firsttime = 1;
//   if (firsttime) {
//     firsttime = 0;
//     bitsBalAddr = channels_find_by_name("bits_bal");
//     levelOnBalAddr = channels_find_by_name("level_on_bal");
//     levelOffBalAddr = channels_find_by_name("level_off_bal");
//     levelTargetBalAddr = channels_find_by_name("level_target_bal");
//     gainBalAddr = channels_find_by_name("gain_bal");
//   }
//
//   /* Run Heating card, maybe */
//   bits_bal = ControlPumpHeat(bits_bal);
//
//   SET_VALUE(levelOnBalAddr, CommandData.pumps.level_on_bal);
//   SET_VALUE(levelOffBalAddr, CommandData.pumps.level_off_bal);
//   SET_VALUE(levelTargetBalAddr, (CommandData.pumps.level_target_bal + 1990.13*5.));
//   SET_VALUE(gainBalAddr, (int)(CommandData.pumps.gain_bal * 1000.));
//   SET_VALUE(bitsBalAddr, bits_bal);
// }

void VideoTx(void)
{
    static channel_t* bitsVtxAddr;
    static int firsttime = 1;
    int vtx_bits = 0;

    if (firsttime) {
        firsttime = 0;
        bitsVtxAddr = channels_find_by_name("bits_vtx");
    }

    if (CommandData.vtx_sel[0] == vtx_xsc1) vtx_bits |= 0x1;
    if (CommandData.vtx_sel[1] == vtx_xsc0) vtx_bits |= 0x4;

    SET_VALUE(bitsVtxAddr, vtx_bits);
}

