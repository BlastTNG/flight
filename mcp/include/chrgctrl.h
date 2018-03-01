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

**************************************************************************/

/*************************************************************************
 
 chrgctrl.h -- function declarations and #define statements for 
               charge controller code. 		

*************************************************************************/

#ifndef INCLUDE_CHRGCTRL_H
#define INCLUDE_CHRGCTRL_H

/* MODBUS restrictions on # of bytes in request/response packets */

#define MAX_QUERY_LENGTH    256
#define MAX_RESPONSE_LENGTH 256
#define MAX_DATA_LENGTH     MAX_RESPONSE_LENGTH - 6

/* MAX_DATA LENGTH = MAX_RESPONSE_LENGTH - server address (1 B)
                                         - CRC            (2 B)
                                         - function code  (1 B)
                                         - byte count     (1 B)
                                         - 1 to make even (1 B) 
                                         ----------------------
                                         -                (6 B) */

#define MAX_READ_REGS MAX_DATA_LENGTH/2  // each register is 2 bytes

void startChrgCtrl(int m_controller);
void store_charge_controller_data(void);

#endif  // CHRGCTRL_H
