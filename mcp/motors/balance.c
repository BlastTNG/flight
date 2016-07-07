/* 
 * balance.c: 
 *
 * This software is copyright (C) 2016 Laura Fissel
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 * History:
 * Created on: Apr 27, 2016 by Laura Fissel
 */
#include <stdlib.h>

#include "mcp.h"
#include "lut.h"
#include "ezstep.h"
#include "command_struct.h"
#include "pointing_struct.h" /* To access ACSData */
#include "tx.h" /* InCharge */
#include "balance.h"

static void DoBalance(void)
{
    static int firsttime = 1;
}
