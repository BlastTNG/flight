/* 
 * acs.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
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
 * Created on: Mar 24, 2015 by Seth Hillbrand
 */

#include <stdio.h>
#include <string.h>

#include <blast.h>

#include <calibrate.h>
#include <channel_macros.h>
#include <conversions.h>
#include <channels_tng.h>
#include <command_struct.h>
#include <ec_motors.h>
#include <motors.h>
#include <mcp.h>
#include <pointing_struct.h>
#include <dsp1760.h>
#include "sip.h"
#include "gps.h"
#include "xsc_network.h"

static const float gy_inv[64][3][6] =
        {
        /* mask = 000000 (0) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000001 (1) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000010 (2) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000011 (3) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000100 (4) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000101 (5) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000110 (6) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000111 (7) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001000 (8) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001001 (9) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001010 (10) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001011 (11) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001100 (12) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001101 (13) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001110 (14) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001111 (15) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 010000 (16) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010001 (17) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010010 (18) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010011 (19) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010100 (20) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010101 (21) */
        { { 0.99782252, 0.000000e+00, -0.080823624, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.080823624, 0.000000e+00, 0.99782252, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.99782252, 0.000000e+00 } },

        /* mask = 010110 (22) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010111 (23) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011000 (24) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011001 (25) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011010 (26) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011011 (27) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011100 (28) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011101 (29) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011110 (30) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011111 (31) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 100000 (32) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100001 (33) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100010 (34) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100011 (35) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100100 (36) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100101 (37) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100110 (38) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100111 (39) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101000 (40) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101001 (41) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101010 (42) */
        { { 0.000000e+00, 1.0040662, 0.000000e+00, -0.11044728, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.11044728, 0.000000e+00, 1.0040662, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, -1.0040662 } },

        /* mask = 101011 (43) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101100 (44) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101101 (45) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101110 (46) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101111 (47) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 110000 (48) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110001 (49) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110010 (50) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110011 (51) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110100 (52) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110101 (53) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110110 (54) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110111 (55) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111000 (56) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111001 (57) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111010 (58) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111011 (59) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111100 (60) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111101 (61) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111110 (62) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111111 (63) */
        { { 0.49891126, 0.50203309, -0.040411812, -0.055223640, 0.000000e+00, 0.000000e+00 },
          { 0.040411812, 0.055223640, 0.49891126, 0.50203309, 0.000000e+00, 0.000000e+00 },
          { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.49891126, -0.50203309 } },
        };
// TODO(seth): Extern sched_lst after enabling sched.c
unsigned int sched_lst; /* sched_lst */

struct ACSDataStruct ACSData;

/**
 * Reads the 5Hz data from the most recent frame received from UEIs and stores
 * it into the ACSData structure for use in pointing
 */
void read_5hz_acs(void)
{
  static channel_t* vPssAddr[NUM_PSS][NUM_PSS_V];
  static channel_t* elRawIfClinAddr;
  static channel_t* mag_x_n_addr;
  static channel_t* mag_y_n_addr;
  static channel_t* mag_z_n_addr;
  static channel_t* mag_x_s_addr;
  static channel_t* mag_y_s_addr;
  static channel_t* mag_z_s_addr;

  char channel_name[128] = {0};

  int i, j;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    for (i = 0; i < NUM_PSS; i++) {
      for (j = 0; j < NUM_PSS_V; j++) {
	    snprintf(channel_name, sizeof(channel_name), "v%d_%d_pss", j+1, i+1);
	    vPssAddr[i][j] = channels_find_by_name(channel_name);
	    // blast_info("PSS read, i=%d, j=%d, channel name =%s", i, j, channel_name);
      }
    }
    elRawIfClinAddr = channels_find_by_name("el_raw_if_clin");
    /* mag_x_n_addr = channels_find_by_name("x_mag1_n");
    mag_y_n_addr = channels_find_by_name("y_mag1_n");
    mag_z_n_addr = channels_find_by_name("z_mag1_n");
    mag_x_s_addr = channels_find_by_name("x_mag2_s");
    mag_y_s_addr = channels_find_by_name("y_mag2_s");
    mag_z_s_addr = channels_find_by_name("z_mag2_s"); */
  }
  for (i = 0; i < NUM_PSS; i++) {
    for (j = 0; j < NUM_PSS_V; j++) {
		GET_SCALED_VALUE(vPssAddr[i][j], ACSData.pss_i[i][j]);
    }
  }

  GET_VALUE(elRawIfClinAddr, ACSData.clin_elev);
  /* ACSData.mag_x[0] = ((double)GET_INT16(mag_x_n_addr))*M_16MAG;
  ACSData.mag_y[0] = ((double)GET_INT16(mag_y_n_addr))*M_16MAG;
  ACSData.mag_z[0] = ((double)GET_INT16(mag_z_n_addr))*M_16MAG;
  ACSData.mag_x[1] = ((double)GET_INT16(mag_x_s_addr))*M_16MAG;
  ACSData.mag_y[1] = ((double)GET_INT16(mag_y_s_addr))*M_16MAG;
  ACSData.mag_z[1] = ((double)GET_INT16(mag_z_s_addr))*M_16MAG; */
}
/**
 * Reads the 100Hz data from the most recent frame received from UEIs and stores
 * it into the ACSData structure for use in pointing
 */
void read_100hz_acs(void)
{
    ACSData.enc_elev = el_get_position_degrees();
	ACSData.enc_motor_elev = el_get_motor_position_degrees();
}

/**
 * Stores the 200Hz ACS data read by the flight computer into the frame.
 */
void store_200hz_acs(void)
{
    float ifel_gy1;
    float ifroll_gy1;
    float ifyaw_gy1;
    static channel_t* ifElgy1Addr;
    static channel_t* ifRollgy1Addr;
    static channel_t* ifYawgy1Addr;

    float ifel_gy2;
    float ifroll_gy2;
    float ifyaw_gy2;
    static channel_t* ifElgy2Addr;
    static channel_t* ifRollgy2Addr;
    static channel_t* ifYawgy2Addr;

    float gy_ifel;
    float gy_ifroll;
    float gy_ifyaw;
    static channel_t* ifel_gy_addr;
    static channel_t* ifroll_gy_addr;
    static channel_t* ifyaw_gy_addr;

    uint16_t gymask = 0x3f;
    uint16_t gyfault = 0;
    static channel_t* mask_gy_addr;
    static channel_t* fault_gy_addr;
    static channel_t* gyro_valid_addr[2][3];
    static uint32_t gyro_valid_count[2][3] = {{0}};
    static uint32_t gyro_valid_set[2][3] = {{0}};

    static int firsttime = 1;
    if (firsttime) {
      firsttime = 0;
      ifel_gy_addr = channels_find_by_name("ifel_gy");
      ifroll_gy_addr = channels_find_by_name("ifroll_gy");
      ifyaw_gy_addr = channels_find_by_name("ifyaw_gy");
      ifElgy1Addr = channels_find_by_name("ifel_1_gy");
      ifRollgy1Addr = channels_find_by_name("ifroll_1_gy");
      ifYawgy1Addr = channels_find_by_name("ifyaw_1_gy");
      ifElgy2Addr = channels_find_by_name("ifel_2_gy");
      ifRollgy2Addr = channels_find_by_name("ifroll_2_gy");
      ifYawgy2Addr = channels_find_by_name("ifyaw_2_gy");

      gyro_valid_addr[0][0] = channels_find_by_name("good_pktcnt_yaw_1_gy");
      gyro_valid_addr[0][1] = channels_find_by_name("good_pktcnt_roll_1_gy");
      gyro_valid_addr[0][2] = channels_find_by_name("good_pktcnt_el_1_gy");
      gyro_valid_addr[1][0] = channels_find_by_name("good_pktcnt_yaw_2_gy");
      gyro_valid_addr[1][1] = channels_find_by_name("good_pktcnt_roll_2_gy");
      gyro_valid_addr[1][2] = channels_find_by_name("good_pktcnt_el_2_gy");

      mask_gy_addr = channels_find_by_name("mask_gy");
      fault_gy_addr = channels_find_by_name("fault_gy");
    }

    gymask = GET_UINT16(mask_gy_addr);

    ifroll_gy1 = dsp1760_getval(0, 0);
    ifyaw_gy1 = dsp1760_getval(0, 1);
    ifel_gy1 = dsp1760_getval(0, 2);
    SET_FLOAT(ifElgy1Addr, ifel_gy1);
    SET_FLOAT(ifRollgy1Addr, ifroll_gy1);
    SET_FLOAT(ifYawgy1Addr, ifyaw_gy1);

    ifyaw_gy2 = dsp1760_getval(1, 0);
    ifroll_gy2 = dsp1760_getval(1, 1);
    ifel_gy2 = dsp1760_getval(1, 2);
    SET_FLOAT(ifElgy2Addr, ifel_gy2);
    SET_FLOAT(ifRollgy2Addr, ifroll_gy2);
    SET_FLOAT(ifYawgy2Addr, ifyaw_gy2);

    gy_ifroll= gy_inv[gymask][0][0]*ifroll_gy1 + gy_inv[gymask][0][1]*ifroll_gy2 + gy_inv[gymask][0][2]*ifyaw_gy1
             + gy_inv[gymask][0][3]*ifyaw_gy2 + gy_inv[gymask][0][4]*ifel_gy1 + gy_inv[gymask][0][5]*ifel_gy2;
    gy_ifyaw = gy_inv[gymask][1][0]*ifroll_gy1 + gy_inv[gymask][1][1]*ifroll_gy2 + gy_inv[gymask][1][2]*ifyaw_gy1
             + gy_inv[gymask][1][3]*ifyaw_gy2 + gy_inv[gymask][1][4]*ifel_gy1 + gy_inv[gymask][1][5]*ifel_gy2;
    gy_ifel  = gy_inv[gymask][2][0]*ifroll_gy1 + gy_inv[gymask][2][1]*ifroll_gy2 + gy_inv[gymask][2][2]*ifyaw_gy1
             + gy_inv[gymask][2][3]*ifyaw_gy2 + gy_inv[gymask][2][4]*ifel_gy1 + gy_inv[gymask][2][5]*ifel_gy2;
    SET_FLOAT(ifel_gy_addr, gy_ifel);
    SET_FLOAT(ifroll_gy_addr, gy_ifroll);
    SET_FLOAT(ifyaw_gy_addr, gy_ifyaw);
    ACSData.ifel_gy = gy_ifel;
    ACSData.ifroll_gy = gy_ifroll;
    ACSData.ifyaw_gy = gy_ifyaw;

    /**
     * We determine whether a gyro is fault by examining the count of valid packets received.
     * This should increment by 5 each time.  Not incrementing for 2 sequences indicates a
     * persistent fault.
     */
    for (int box = 0; box < 2; box++) {
        for (int gyro = 0; gyro < 3; gyro++) {
            uint32_t gyro_valid = dsp1760_get_valid_packet_count(box, gyro);
            if (gyro_valid == gyro_valid_count[box][gyro])
                gyro_valid_set[box][gyro]++;
            else
                gyro_valid_set[box][gyro] = 0;

            gyro_valid_count[box][gyro] = gyro_valid;
            SET_UINT32(gyro_valid_addr[box][gyro], gyro_valid_count[box][gyro]);

            if (gyro_valid_set[box][gyro] > 1)
                gyfault |= (1 << (gyro * 2 + box));
            else
                gyfault &= ~(1 << (gyro * 2 + box));
        }
    }

    SET_UINT16(fault_gy_addr, gyfault);
}

/**
 * Stores the 100Hz ACS data read by the flight computer into the frame.
 */
void store_100hz_acs(void)
{
    static channel_t *azAddr;
    static channel_t *elAddr;
    static channel_t *elEncAddr;
    static channel_t *sigmaEncAddr;
    static channel_t *elMotEncAddr;
    static channel_t *sigmaMotEncAddr;

    static channel_t *vel_rw_addr;
    static channel_t *pos_rw_addr;
    static channel_t *vel_el_addr;
    static channel_t *pos_el_addr;
    static channel_t *pos_motor_el_addr;
    static channel_t *vel_piv_addr;
    static channel_t *pos_piv_addr;

    static channel_t *newOffsetIFElMotEncAddr;
    static channel_t *intIFElMotEncAddr;
    static channel_t *newOffsetIFYawMag1Addr;
    static channel_t *newOffsetIFRollMag1Addr;
    static channel_t *intIFYawMag1Addr;
    static channel_t *intIFRollMag1Addr;
    static channel_t *dAzMag1Addr;
    static channel_t *newOffsetIFYawMag2Addr;
    static channel_t *newOffsetIFRollMag2Addr;
    static channel_t *intIFYawMag2Addr;
    static channel_t *intIFRollMag2Addr;
    static channel_t *dAzMag2Addr;
    static channel_t *newOffsetIFElXSC0Addr;
    static channel_t *newOffsetIFYawXSC0Addr;
    static channel_t *newOffsetIFRollXSC0Addr;
    static channel_t *dAzRollXSC0Addr;
    static channel_t *intIFYawXSC0Addr;
    static channel_t *intIFRollXSC0Addr;
    static channel_t *intIFElXSC0Addr;
    static channel_t *prevSolAzXSC0Addr;
    static channel_t *prevSolElXSC0Addr;
    static channel_t *newOffsetIFElXSC1Addr;
    static channel_t *newOffsetIFYawXSC1Addr;
    static channel_t *newOffsetIFRollXSC1Addr;
    static channel_t *dAzRollXSC1Addr;
    static channel_t *intIFYawXSC1Addr;
    static channel_t *intIFRollXSC1Addr;
    static channel_t *intIFElXSC1Addr;
    static channel_t *prevSolAzXSC1Addr;
    static channel_t *prevSolElXSC1Addr;

    static int firsttime = 1;
    int i_point;
    int i_motors;

    /******** Obtain correct indexes the first time here ***********/
    if (firsttime) {
        firsttime = 0;
        azAddr = channels_find_by_name("az");
        elAddr = channels_find_by_name("el");

        elEncAddr = channels_find_by_name("el_enc");
        sigmaEncAddr = channels_find_by_name("sigma_enc");
        elMotEncAddr = channels_find_by_name("el_motor_enc");
        sigmaMotEncAddr = channels_find_by_name("sigma_motor_enc");

        vel_rw_addr = channels_find_by_name("mc_rw_vel");
        pos_rw_addr = channels_find_by_name("mc_rw_pos");

        vel_el_addr = channels_find_by_name("mc_el_vel");
        pos_el_addr = channels_find_by_name("mc_el_pos");
        pos_motor_el_addr = channels_find_by_name("mc_el_motor_pos");

        vel_piv_addr = channels_find_by_name("mc_piv_vel");
        pos_piv_addr = channels_find_by_name("mc_piv_pos");

        newOffsetIFElMotEncAddr = channels_find_by_name("new_offset_ifelmotorenc_gy");
        intIFElMotEncAddr = channels_find_by_name("int_ifelmotorenc");
        newOffsetIFYawMag1Addr = channels_find_by_name("new_offset_ifyawmag1_gy");
        newOffsetIFRollMag1Addr = channels_find_by_name("new_offset_ifrollmag1_gy");
        intIFYawMag1Addr = channels_find_by_name("int_ifyawmag1");
        intIFRollMag1Addr = channels_find_by_name("int_ifrollmag1");
        dAzMag1Addr =  channels_find_by_name("d_az_mag1");
        newOffsetIFYawMag2Addr = channels_find_by_name("new_offset_ifyawmag2_gy");
        newOffsetIFRollMag2Addr = channels_find_by_name("new_offset_ifrollmag2_gy");
        intIFYawMag2Addr = channels_find_by_name("int_ifyawmag2");
        intIFRollMag2Addr = channels_find_by_name("int_ifrollmag2");
        dAzMag2Addr =  channels_find_by_name("d_az_mag2");
        newOffsetIFElXSC0Addr = channels_find_by_name("new_offset_ifelxsc0_gy");
        newOffsetIFYawXSC0Addr = channels_find_by_name("new_offset_ifyawxsc0_gy");
        newOffsetIFRollXSC0Addr = channels_find_by_name("new_offset_ifrollxsc0_gy");
        dAzRollXSC0Addr =  channels_find_by_name("d_az_xsc0");
        intIFYawXSC0Addr = channels_find_by_name("int_ifyawxsc0");
        intIFRollXSC0Addr = channels_find_by_name("int_ifrollxsc0");
        intIFElXSC0Addr = channels_find_by_name("int_ifelxsc0");
        prevSolAzXSC0Addr = channels_find_by_name("prev_soln_az_xsc0");
        prevSolElXSC0Addr = channels_find_by_name("prev_soln_el_xsc0");
        newOffsetIFElXSC1Addr = channels_find_by_name("new_offset_ifelxsc1_gy");
        newOffsetIFYawXSC1Addr = channels_find_by_name("new_offset_ifyawxsc1_gy");
        newOffsetIFRollXSC1Addr = channels_find_by_name("new_offset_ifrollxsc1_gy");
        dAzRollXSC1Addr =  channels_find_by_name("d_az_xsc1");
        intIFYawXSC1Addr = channels_find_by_name("int_ifyawxsc1");
        intIFRollXSC1Addr = channels_find_by_name("int_ifrollxsc1");
        intIFElXSC1Addr = channels_find_by_name("int_ifelxsc1");
        prevSolAzXSC1Addr = channels_find_by_name("prev_soln_az_xsc1");
        prevSolElXSC1Addr = channels_find_by_name("prev_soln_el_xsc1");
    }
    i_point = GETREADINDEX(point_index);
    i_motors = GETREADINDEX(motor_index);

    SET_SCALED_VALUE(azAddr, PointingData[i_point].az);
    SET_SCALED_VALUE(elAddr, PointingData[i_point].el);

    SET_SCALED_VALUE(elEncAddr, (PointingData[i_point].enc_el + CommandData.enc_el_trim));
    SET_SCALED_VALUE(sigmaEncAddr, PointingData[i_point].enc_sigma);
    SET_SCALED_VALUE(elMotEncAddr, (PointingData[i_point].enc_motor_el + CommandData.enc_motor_el_trim));
    SET_SCALED_VALUE(sigmaMotEncAddr, PointingData[i_point].enc_motor_sigma);

    SET_INT32(vel_rw_addr, RWMotorData[i_motors].velocity);
    SET_INT32(pos_rw_addr, RWMotorData[i_motors].position);
    SET_INT32(vel_el_addr, ElevMotorData[i_motors].velocity);
    SET_INT32(pos_el_addr, ElevMotorData[i_motors].position);
    SET_INT32(pos_motor_el_addr, ElevMotorData[i_motors].motor_position);
    SET_INT32(vel_piv_addr, PivotMotorData[i_motors].velocity);
    SET_INT32(pos_piv_addr, PivotMotorData[i_motors].position);

    SET_SCALED_VALUE(newOffsetIFElMotEncAddr, PointingData[i_point].new_offset_ifel_elmotenc_gy);
    SET_SCALED_VALUE(intIFElMotEncAddr, PointingData[i_point].int_ifel_elmotenc);
    SET_SCALED_VALUE(newOffsetIFYawMag1Addr, PointingData[i_point].new_offset_ifyaw_mag1_gy);
    SET_SCALED_VALUE(newOffsetIFRollMag1Addr, PointingData[i_point].new_offset_ifroll_mag1_gy);
    SET_SCALED_VALUE(intIFYawMag1Addr, PointingData[i_point].int_ifyaw_mag1);
    SET_SCALED_VALUE(intIFRollMag1Addr, PointingData[i_point].int_ifroll_mag1);
    SET_SCALED_VALUE(dAzMag1Addr, PointingData[i_point].d_az_mag1);
    SET_SCALED_VALUE(newOffsetIFYawMag2Addr, PointingData[i_point].new_offset_ifyaw_mag2_gy);
    SET_SCALED_VALUE(newOffsetIFRollMag2Addr, PointingData[i_point].new_offset_ifroll_mag2_gy);
    SET_SCALED_VALUE(intIFYawMag2Addr, PointingData[i_point].int_ifyaw_mag2);
    SET_SCALED_VALUE(intIFRollMag2Addr, PointingData[i_point].int_ifroll_mag2);
    SET_SCALED_VALUE(dAzMag2Addr, PointingData[i_point].d_az_mag2);

    SET_SCALED_VALUE(newOffsetIFElXSC0Addr, PointingData[i_point].new_offset_ifel_xsc0_gy);
    SET_SCALED_VALUE(newOffsetIFYawXSC0Addr, PointingData[i_point].new_offset_ifyaw_xsc0_gy);
    SET_SCALED_VALUE(newOffsetIFRollXSC0Addr, PointingData[i_point].new_offset_ifroll_xsc0_gy);
    SET_SCALED_VALUE(dAzRollXSC0Addr, PointingData[i_point].d_az_xsc0);
    SET_SCALED_VALUE(intIFYawXSC0Addr, PointingData[i_point].int_ifyaw_xsc0);
    SET_SCALED_VALUE(intIFRollXSC0Addr, PointingData[i_point].int_ifroll_xsc0);
    SET_SCALED_VALUE(intIFElXSC0Addr, PointingData[i_point].int_ifel_xsc0);
    SET_SCALED_VALUE(prevSolAzXSC0Addr, PointingData[i_point].prev_sol_az_xsc0);
    SET_SCALED_VALUE(prevSolElXSC0Addr, PointingData[i_point].prev_sol_el_xsc0);
    SET_SCALED_VALUE(newOffsetIFElXSC1Addr, PointingData[i_point].new_offset_ifel_xsc1_gy);
    SET_SCALED_VALUE(newOffsetIFYawXSC1Addr, PointingData[i_point].new_offset_ifyaw_xsc1_gy);
    SET_SCALED_VALUE(newOffsetIFRollXSC1Addr, PointingData[i_point].new_offset_ifroll_xsc1_gy);
    SET_SCALED_VALUE(dAzRollXSC1Addr, PointingData[i_point].d_az_xsc1);
    SET_SCALED_VALUE(intIFYawXSC1Addr, PointingData[i_point].int_ifyaw_xsc1);
    SET_SCALED_VALUE(intIFRollXSC1Addr, PointingData[i_point].int_ifroll_xsc1);
    SET_SCALED_VALUE(intIFElXSC1Addr, PointingData[i_point].int_ifel_xsc1);
    SET_SCALED_VALUE(prevSolAzXSC1Addr, PointingData[i_point].prev_sol_az_xsc1);
    SET_SCALED_VALUE(prevSolElXSC1Addr, PointingData[i_point].prev_sol_el_xsc1);
}

static inline channel_t* get_xsc_channel(const char *m_field, int m_which)
{
  char buffer[FIELD_LEN];
  const char prefix[2][3] = {"x0", "x1"};
  snprintf(buffer, sizeof(buffer), "%s_%s", prefix[m_which], m_field);
  return channels_find_by_name(buffer);
}

void store_5hz_xsc(int m_which)
{
    static bool firsttime[2] = {true, true};
    static channel_t* address_xN_point_az[2];
    static channel_t* address_xN_point_el[2];
    static channel_t* address_xN_point_var[2];
    static channel_t* address_xN_point_sigma[2];

    int i_point = GETREADINDEX(point_index);

    if (firsttime[m_which]) {
        firsttime[m_which] = false;
        address_xN_point_az[m_which]      = get_xsc_channel("point_az"        , m_which);
        address_xN_point_el[m_which]      = get_xsc_channel("point_el"        , m_which);
        address_xN_point_var[m_which]   = get_xsc_channel("point_var"     , m_which);
        address_xN_point_sigma[m_which]   = get_xsc_channel("point_sigma"     , m_which);
    }
    SET_SCALED_VALUE(address_xN_point_az[m_which]     , PointingData[i_point].xsc_az[m_which]);
    SET_SCALED_VALUE(address_xN_point_el[m_which]     , PointingData[i_point].xsc_el[m_which]);
    SET_SCALED_VALUE(address_xN_point_var[m_which]  , PointingData[i_point].xsc_var[m_which]);
    SET_SCALED_VALUE(address_xN_point_sigma[m_which]  , PointingData[i_point].xsc_sigma[m_which]);
}

void store_1hz_xsc(int m_which)
{
    static bool firsttime[2] = {true, true};

    static channel_t* address_xN_point_az_raw[2];
    static channel_t* address_xN_point_el_raw[2];
    static channel_t* address_xN_point_az_trim[2];
    static channel_t* address_xN_point_el_trim[2];
    static channel_t* address_xN_cd_robust_mode[2];
//    static channel_t* address_xN_num_images_saved[2];

    static channel_t* address_xN_last_trig_lat;
    static channel_t* address_xN_last_trig_lst;

    static channel_t *address_xN_hk_temp_lens[2];
    static channel_t *address_xN_hk_temp_comp[2];
    static channel_t *address_xN_hk_temp_plate[2];
    static channel_t *address_xN_hk_temp_flange[2];
    static channel_t *address_xN_hk_pressure[2];
    static channel_t *address_xN_hk_disk[2];

    static channel_t *address_xN_image_eq_valid[2];
    static channel_t *address_xN_cam_gain_valid[2];
    static channel_t *address_xN_image_hor_valid[2];
    static channel_t *address_xN_image_afocus_metric_valid[2];

    static channel_t *address_xN_stars_run_time[2];
    static channel_t *address_xN_cam_gain_db[2];
    static channel_t *address_xN_lens_focus[2];
    static channel_t *address_xN_lens_aperture[2];

    static channel_t *address_xN_image_num_exposures[2];
    static channel_t *address_xN_image_stats_mean[2];
    static channel_t *address_xN_image_stats_noise[2];
    static channel_t *address_xN_image_stats_gaindb[2];
    static channel_t *address_xN_image_stats_num_px_sat[2];
    static channel_t *address_xN_image_stats_frac_px_sat[2];
    static channel_t *address_xN_image_afocus_metric[2];

    static channel_t *address_xN_image_eq_iplate[2];
    static channel_t *address_xN_image_hor_iplate[2];

    static channel_t *address_xN_image_eq_ra[2];
    static channel_t *address_xN_image_eq_dec[2];
    static channel_t *address_xN_image_eq_roll[2];
    static channel_t *address_xN_image_eq_sigma_ra[2];
    static channel_t *address_xN_image_eq_sigma_dec[2];
    static channel_t *address_xN_image_eq_sigma_roll[2];
    static channel_t *address_xN_image_eq_sigma_pointing[2];
    static channel_t *address_xN_image_hor_az[2];
    static channel_t *address_xN_image_hor_el[2];
    static channel_t *address_xN_image_hor_roll[2];
    static channel_t *address_xN_image_hor_sigma_az[2];
    static channel_t *address_xN_image_hor_sigma_el[2];
    static channel_t *address_xN_image_hor_sigma_roll[2];
    static channel_t *address_xN_image_hor_sigma_pointing[2];

    static channel_t *address_xN_image_num_blobs_found[2];
    static channel_t *address_xN_image_num_blobs_matched[2];

    int i_point = GETREADINDEX(point_index);

    if (firsttime[m_which]) {
        firsttime[m_which] = false;

        address_xN_image_num_blobs_found[m_which] = get_xsc_channel("image_num_blobs_found", m_which);
        address_xN_image_num_blobs_matched[m_which] = get_xsc_channel("image_num_blobs_matched", m_which);

        address_xN_hk_temp_lens[m_which] = get_xsc_channel("hk_temp_lens", m_which);
        address_xN_hk_temp_comp[m_which] = get_xsc_channel("hk_temp_comp", m_which);
        address_xN_hk_temp_plate[m_which] = get_xsc_channel("hk_temp_plate", m_which);
        address_xN_hk_temp_flange[m_which] = get_xsc_channel("hk_temp_flange", m_which);
        address_xN_hk_pressure[m_which] = get_xsc_channel("hk_pressure", m_which);
        address_xN_hk_disk[m_which] = get_xsc_channel("hk_disk", m_which);

        address_xN_image_eq_valid[m_which] = get_xsc_channel("image_eq_valid", m_which);
        address_xN_cam_gain_valid[m_which] = get_xsc_channel("cam_gain_valid", m_which);
        address_xN_image_hor_valid[m_which] = get_xsc_channel("image_hor_valid", m_which);
        address_xN_image_afocus_metric_valid[m_which] = get_xsc_channel("image_afocus_metric_valid", m_which);

        address_xN_stars_run_time[m_which] = get_xsc_channel("stars_run_time", m_which);
        address_xN_cam_gain_db[m_which] = get_xsc_channel("cam_gain_db", m_which);
        address_xN_lens_focus[m_which] = get_xsc_channel("lens_focus", m_which);
        address_xN_lens_aperture[m_which] = get_xsc_channel("lens_aperture", m_which);

        address_xN_image_num_exposures[m_which] = get_xsc_channel("image_num_exposures", m_which);
        address_xN_image_stats_mean[m_which] = get_xsc_channel("image_stats_mean", m_which);
        address_xN_image_stats_noise[m_which] = get_xsc_channel("image_stats_noise", m_which);
        address_xN_image_stats_gaindb[m_which] = get_xsc_channel("image_stats_gaindb", m_which);
        address_xN_image_stats_num_px_sat[m_which] = get_xsc_channel("image_stats_num_px_sat", m_which);
        address_xN_image_stats_frac_px_sat[m_which] = get_xsc_channel("image_stats_frac_px_sat", m_which);
        address_xN_image_afocus_metric[m_which] = get_xsc_channel("image_afocus_metric", m_which);

        address_xN_image_eq_iplate[m_which] = get_xsc_channel("image_eq_iplate", m_which);
        address_xN_image_hor_iplate[m_which] = get_xsc_channel("image_hor_iplate", m_which);

        address_xN_image_eq_ra[m_which] = get_xsc_channel("image_eq_ra", m_which);
        address_xN_image_eq_dec[m_which] = get_xsc_channel("image_eq_dec", m_which);
        address_xN_image_eq_roll[m_which] = get_xsc_channel("image_eq_roll", m_which);
        address_xN_image_eq_sigma_ra[m_which] = get_xsc_channel("image_eq_sigma_ra", m_which);
        address_xN_image_eq_sigma_dec[m_which] = get_xsc_channel("image_eq_sigma_dec", m_which);
        address_xN_image_eq_sigma_roll[m_which] = get_xsc_channel("image_eq_sigma_roll", m_which);
        address_xN_image_eq_sigma_pointing[m_which] = get_xsc_channel("image_eq_sigma_pointing", m_which);
        address_xN_image_hor_az[m_which] = get_xsc_channel("image_hor_az", m_which);
        address_xN_image_hor_el[m_which] = get_xsc_channel("image_hor_el", m_which);
        address_xN_image_hor_roll[m_which] = get_xsc_channel("image_hor_roll", m_which);
        address_xN_image_hor_sigma_az[m_which] = get_xsc_channel("image_hor_sigma_az", m_which);
        address_xN_image_hor_sigma_el[m_which] = get_xsc_channel("image_hor_sigma_el", m_which);
        address_xN_image_hor_sigma_roll[m_which] = get_xsc_channel("image_hor_sigma_roll", m_which);
        address_xN_image_hor_sigma_pointing[m_which] = get_xsc_channel("image_hor_sigma_pointing", m_which);

        address_xN_point_az_raw[m_which]  = get_xsc_channel("point_az_raw"    , m_which);
        address_xN_point_el_raw[m_which]  = get_xsc_channel("point_el_raw"    , m_which);
        address_xN_point_az_trim[m_which] = get_xsc_channel("point_az_trim"   , m_which);
        address_xN_point_el_trim[m_which] = get_xsc_channel("point_el_trim"   , m_which);
        address_xN_cd_robust_mode[m_which] = get_xsc_channel("cd_robust_mode"   , m_which);
//        address_xN_num_images_saved[m_which] = get_xsc_channel("num_images_saved"   , m_which);
        if (m_which == 0) {
            address_xN_last_trig_lat                = get_xsc_channel("last_trig_lat"        , 0);
            address_xN_last_trig_lst                = get_xsc_channel("last_trig_lst"        , 0);
        }
    }

    SET_SCALED_VALUE(address_xN_image_num_blobs_found[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_num_blobs_found);
    SET_SCALED_VALUE(address_xN_image_num_blobs_matched[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_num_blobs_matched);

    SET_SCALED_VALUE(address_xN_hk_temp_lens[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_lens);
    SET_SCALED_VALUE(address_xN_hk_temp_comp[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_comp);
    SET_SCALED_VALUE(address_xN_hk_temp_plate[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_plate);
    SET_SCALED_VALUE(address_xN_hk_temp_flange[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_flange);
    SET_SCALED_VALUE(address_xN_hk_pressure[m_which], XSC_SERVER_DATA(m_which).channels.hk_pressure);
    SET_SCALED_VALUE(address_xN_hk_disk[m_which], XSC_SERVER_DATA(m_which).channels.hk_disk);

    SET_SCALED_VALUE(address_xN_image_eq_valid[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_valid);
    SET_SCALED_VALUE(address_xN_cam_gain_valid[m_which], XSC_SERVER_DATA(m_which).channels.cam_gain_valid);
    SET_SCALED_VALUE(address_xN_image_hor_valid[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_valid);
    SET_SCALED_VALUE(address_xN_image_afocus_metric_valid[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_afocus_metric_valid);

    SET_SCALED_VALUE(address_xN_stars_run_time[m_which], XSC_SERVER_DATA(m_which).channels.stars_run_time);
    SET_SCALED_VALUE(address_xN_cam_gain_db[m_which], XSC_SERVER_DATA(m_which).channels.cam_gain_db);
    SET_SCALED_VALUE(address_xN_lens_focus[m_which], XSC_SERVER_DATA(m_which).channels.lens_focus);
    SET_SCALED_VALUE(address_xN_lens_aperture[m_which], XSC_SERVER_DATA(m_which).channels.lens_aperture);

    SET_SCALED_VALUE(address_xN_image_num_exposures[m_which], XSC_SERVER_DATA(m_which).channels.image_num_exposures);
    SET_SCALED_VALUE(address_xN_image_stats_mean[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_mean);
    SET_SCALED_VALUE(address_xN_image_stats_noise[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_noise);
    SET_SCALED_VALUE(address_xN_image_stats_gaindb[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_gaindb);
    SET_SCALED_VALUE(address_xN_image_stats_num_px_sat[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_stats_num_px_sat);
    SET_SCALED_VALUE(address_xN_image_stats_frac_px_sat[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_stats_frac_px_sat);
    SET_SCALED_VALUE(address_xN_image_afocus_metric[m_which], XSC_SERVER_DATA(m_which).channels.image_afocus_metric);

    SET_SCALED_VALUE(address_xN_image_eq_iplate[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_iplate);
    SET_SCALED_VALUE(address_xN_image_hor_iplate[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_iplate);

    SET_SCALED_VALUE(address_xN_image_eq_ra[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_ra);
    SET_SCALED_VALUE(address_xN_image_eq_dec[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_dec);
    SET_SCALED_VALUE(address_xN_image_eq_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_roll);
    SET_SCALED_VALUE(address_xN_image_eq_sigma_ra[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_ra);
    SET_SCALED_VALUE(address_xN_image_eq_sigma_dec[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_dec);
    SET_SCALED_VALUE(address_xN_image_eq_sigma_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_roll);
    SET_SCALED_VALUE(address_xN_image_eq_sigma_pointing[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_eq_sigma_pointing);
    SET_SCALED_VALUE(address_xN_image_hor_az[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_az);
    SET_SCALED_VALUE(address_xN_image_hor_el[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_el);
    SET_SCALED_VALUE(address_xN_image_hor_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_roll);
    SET_SCALED_VALUE(address_xN_image_hor_sigma_az[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_az);
    SET_SCALED_VALUE(address_xN_image_hor_sigma_el[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_el);
    SET_SCALED_VALUE(address_xN_image_hor_sigma_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_roll);
    SET_SCALED_VALUE(address_xN_image_hor_sigma_pointing[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_hor_sigma_pointing);

    SET_SCALED_VALUE(address_xN_point_az_raw[m_which] , xsc_pointing_state[m_which].az);
    SET_SCALED_VALUE(address_xN_point_el_raw[m_which] , xsc_pointing_state[m_which].el);
    SET_SCALED_VALUE(address_xN_point_az_trim[m_which], CommandData.XSC[m_which].cross_el_trim);
    SET_SCALED_VALUE(address_xN_point_el_trim[m_which], CommandData.XSC[m_which].el_trim);
    SET_SCALED_VALUE(address_xN_cd_robust_mode[m_which], CommandData.XSC[m_which].net.solver.robust_mode_enabled);

    /// TODO(seth): Re-add local image saving
//    SET_SCALED_VALUE(address_xN_num_images_saved[m_which], images_num_saved[m_which]);
    if (m_which == 0) {
        SET_SCALED_VALUE(address_xN_last_trig_lat       , xsc_pointing_state[m_which].last_trigger.lat);
        SET_VALUE(address_xN_last_trig_lst              , xsc_pointing_state[m_which].last_trigger.lst*SEC2LI);
    }
}

void store_100hz_xsc(int which)
{
    static bool firsttime[2] = {true, true};
    static int last_blob_counter_stars[2] = {-1, -1};
    static int last_blob_i[2] = {1000, 1000};
    static int intermediate_frame_counter[2] = {0, 0};

    static channel_t* address_xN_ctr_stars[2];
    static channel_t* address_xN_image_ctr_mcp[2];
    static channel_t* address_xN_image_ctr_stars[2];

    static channel_t* address_xN_ctr_mcp;
    static channel_t* address_xN_last_trig_age_cs;
    static channel_t* address_xN_last_trig_ctr_mcp;
    static channel_t* address_xN_last_trig_ctr_stars[2];
    static channel_t* address_xN_predicted_streaking_px[2];
    static channel_t* address_xN_image_blobn_x[2];
    static channel_t* address_xN_image_blobn_y[2];
    static channel_t* address_xN_image_blobn_flux[2];
    static channel_t* address_xN_image_blobn_peak_to_flux[2];

    if (firsttime[which]) {
        firsttime[which] = false;

        if (which == 0) {
            address_xN_ctr_mcp                     = get_xsc_channel("ctr_mcp", 0);
            address_xN_last_trig_age_cs            = get_xsc_channel("last_trig_age_cs", 0);
            address_xN_last_trig_ctr_mcp           = get_xsc_channel("last_trig_ctr_mcp", 0);
        }
        address_xN_predicted_streaking_px[which]   = get_xsc_channel("predicted_streaking_px", which);
        address_xN_ctr_stars[which]                = get_xsc_channel("ctr_stars", which);
        address_xN_image_ctr_stars[which]          = get_xsc_channel("image_ctr_stars", which);
        address_xN_image_ctr_mcp[which]            = get_xsc_channel("image_ctr_mcp", which);
        address_xN_last_trig_ctr_stars[which]      = get_xsc_channel("last_trig_ctr_stars", which);
        address_xN_image_blobn_x[which]            = get_xsc_channel("image_blobn_x", which);
        address_xN_image_blobn_y[which]            = get_xsc_channel("image_blobn_y", which);
        address_xN_image_blobn_flux[which]         = get_xsc_channel("image_blobn_flux", which);
        address_xN_image_blobn_peak_to_flux[which] = get_xsc_channel("image_blobn_peak_to_flux", which);
    }

    if (which == 0) {
        SET_SCALED_VALUE(address_xN_ctr_mcp, xsc_pointing_state[which].counter_mcp);
        SET_SCALED_VALUE(address_xN_last_trig_age_cs, xsc_pointing_state[which].last_trigger.trigger_time);
        SET_SCALED_VALUE(address_xN_last_trig_ctr_mcp, xsc_pointing_state[which].last_trigger.counter_mcp);
    }


    SET_SCALED_VALUE(address_xN_predicted_streaking_px[which], xsc_pointing_state[which].predicted_streaking_px);
    SET_INT32(address_xN_ctr_stars[which], XSC_SERVER_DATA(which).channels.ctr_stars);
    SET_INT32(address_xN_image_ctr_stars[which], XSC_SERVER_DATA(which).channels.image_ctr_stars);
    SET_INT32(address_xN_image_ctr_mcp[which], XSC_SERVER_DATA(which).channels.image_ctr_mcp);
    SET_SCALED_VALUE(address_xN_last_trig_ctr_stars[which],
                     xsc_pointing_state[which].last_trigger.counter_stars);

    if (XSC_SERVER_DATA(which).blobs.counter_stars != last_blob_counter_stars[which] &&
        XSC_SERVER_DATA(which).blobs.counter_stars > 0) {
        last_blob_counter_stars[which] = XSC_SERVER_DATA(which).blobs.counter_stars;
        last_blob_i[which] = 0;
    }
    if (intermediate_frame_counter[which] == 0) {
        if (last_blob_i[which] < XSC_SERVER_DATA(which).blobs.num_blobs
                && last_blob_i[which] < XSC_BLOBS_ARRAY_SIZE) {
            SET_SCALED_VALUE(address_xN_image_blobn_x[which],
                             XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].x);
            SET_SCALED_VALUE(address_xN_image_blobn_y[which],
                             XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].y);
            SET_SCALED_VALUE(address_xN_image_blobn_flux[which],
                             XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].flux);
            SET_SCALED_VALUE(address_xN_image_blobn_peak_to_flux[which],
                             XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].peak_to_flux);
            last_blob_i[which]++;
        } else if (last_blob_i[which] == XSC_SERVER_DATA(which).blobs.num_blobs) {
            SET_SCALED_VALUE(address_xN_image_blobn_x[which], 0);
            SET_SCALED_VALUE(address_xN_image_blobn_y[which], 0);
            SET_SCALED_VALUE(address_xN_image_blobn_flux[which], 0);
            SET_SCALED_VALUE(address_xN_image_blobn_peak_to_flux[which], 0);
        }
    }

    intermediate_frame_counter[which] = (intermediate_frame_counter[which]+1) % 3;
}

void store_5hz_acs(void)
{
    static int firsttime = 1;

    static channel_t* latSipAddr;
    static channel_t* lonSipAddr;
    static channel_t* altSipAddr;
    static channel_t* timeSipAddr;
    static channel_t* mksLoSipAddr;
    static channel_t* mksMedSipAddr;
    static channel_t* mksHiSipAddr;

    /** pointing mode indexes **/
    static channel_t* svetoLenAddr;
    static channel_t* slewVetoAddr;
    static channel_t* modePAddr;
    static channel_t* xPAddr, *yPAddr;
    static channel_t* velAzPAddr, *delPAddr;
    static channel_t* velElPAddr, *dazPAddr;
    static channel_t* wPAddr, *hPAddr;
    static channel_t* ra1PAddr, *dec1PAddr;
    static channel_t* ra2PAddr, *dec2PAddr;
    static channel_t* ra3PAddr, *dec3PAddr;
    static channel_t* ra4PAddr, *dec4PAddr;
    static channel_t* nextIHwprPAddr;
    static channel_t* nextIDithPAddr;
    static channel_t* nDithPAddr;

    static channel_t* vetoSensorAddr;

    /** derived pointing data */
    static channel_t *gy_azvel_addr;
    static channel_t *gy_elvel_addr;
    static channel_t *gy_totalvel_addr;
    static channel_t *gy_totalaccel_addr;

    static channel_t* OffsetIFelGYAddr;
    static channel_t* OffsetIFelGYiscAddr;
    static channel_t* OffsetIFrollGYiscAddr;
    static channel_t* OffsetIFyawGYiscAddr;
    static channel_t* OffsetIFelGYoscAddr;
    static channel_t* OffsetIFrollGYoscAddr;
    static channel_t* OffsetIFyawGYoscAddr;
    static channel_t* OffsetIFrollGYAddr;
    static channel_t* OffsetIFyawGYAddr;
    static channel_t* OffsetIFrollMagNGYAddr;
    static channel_t* OffsetIFyawMagNGYAddr;
    static channel_t* OffsetIFrollMagSGYAddr;
    static channel_t* OffsetIFyawMagSGYAddr;
    static channel_t* OffsetIFrollPSSGYAddr;
    static channel_t* OffsetIFyawPSSGYAddr;
    static channel_t* IFyawEarthGyAddr;
    static channel_t* IFrollEarthGyAddr;
    static channel_t* IFelEarthGyAddr;

    static channel_t* raAddr;
    static channel_t* decAddr;
    static channel_t* altAddr;
    static channel_t* latAddr;
    static channel_t* lonAddr;
    static channel_t* lstAddr;
    static channel_t* azNullAddr;
    static channel_t* azMagNAddr;
    static channel_t* azRawMagNAddr;
    static channel_t* declinationMagNAddr;
    static channel_t* elMagNAddr;
    static channel_t* dipMagNAddr;
    static channel_t* calXMaxMagNAddr;
    static channel_t* calXMinMagNAddr;
    static channel_t* calYMaxMagNAddr;
    static channel_t* calYMinMagNAddr;
    static channel_t* azMagSAddr;
    static channel_t* azRawMagSAddr;
    static channel_t* declinationMagSAddr;
    static channel_t* elMagSAddr;
    static channel_t* dipMagSAddr;
    static channel_t* calXMaxMagSAddr;
    static channel_t* calXMinMagSAddr;
    static channel_t* calYMaxMagSAddr;
    static channel_t* calYMinMagSAddr;
	static channel_t* calAzPssArrayAddr;
    static channel_t* calAzPss1Addr;
    static channel_t* calAzPss2Addr;
    static channel_t* calAzPss3Addr;
    static channel_t* calAzPss4Addr;
    static channel_t* calAzPss5Addr;
    static channel_t* calAzPss6Addr;
    static channel_t* calDPss1Addr;
    static channel_t* calDPss2Addr;
    static channel_t* calDPss3Addr;
    static channel_t* calDPss4Addr;
    static channel_t* calDPss5Addr;
    static channel_t* calDPss6Addr;
    static channel_t* calIMinPssAddr;
    static channel_t* sigmaMagNAddr;
    static channel_t* sigmaMagSAddr;
    static channel_t* sigmaPssAddr;
    static channel_t* azrawPss1Addr;
    static channel_t* azrawPss2Addr;
    static channel_t* azrawPss3Addr;
    static channel_t* azrawPss4Addr;
    static channel_t* azrawPss5Addr;
    static channel_t* azrawPss6Addr;
    static channel_t* elrawPss1Addr;
    static channel_t* elrawPss2Addr;
    static channel_t* elrawPss3Addr;
    static channel_t* elrawPss4Addr;
    static channel_t* elrawPss5Addr;
    static channel_t* elrawPss6Addr;
    static channel_t* snrPss1Addr;
    static channel_t* snrPss2Addr;
    static channel_t* snrPss3Addr;
    static channel_t* snrPss4Addr;
    static channel_t* snrPss5Addr;
    static channel_t* snrPss6Addr;
    static channel_t* azPssAddr;
    static channel_t* PssOkAddr;
    static channel_t* azSunAddr;
    static channel_t* elSunAddr;
    static channel_t* elClinAddr;
    static channel_t* elLutClinAddr;
    static channel_t* sigmaClinAddr;
    static channel_t* DGPSAzAddr;
    static channel_t* DGPSSigmaAzAddr;
    static channel_t *DGPSRawAzAddr;
    static channel_t *MagOKAddr[2];
    static channel_t *EncMotorOK;
    static channel_t *DGPSOK;
    static channel_t *ElClinOK;

    /* trim fields */
    static channel_t *trimClinAddr;
    static channel_t *trimEncAddr;
    static channel_t *trimEncMotorAddr;
    static channel_t *trimNullAddr;
    static channel_t *trimMagNAddr;
    static channel_t *trimMagSAddr;
    static channel_t *trimPssAddr;
    static channel_t *trimDGPSAddr;

    static channel_t *threshAtrimAddr;
    static channel_t *timeAtrimAddr;
    static channel_t *rateAtrimAddr;
    static channel_t *rateAtrimPtAddr;

    static channel_t *modeCalAddr;
    static channel_t *hwprCalAddr;
    static channel_t *periodCalAddr;
    static channel_t *lstSchedAddr;
    static channel_t *freshTrimAddr;
    static channel_t *newAzAddr;
    static channel_t *newElAddr;
    static channel_t *weightAzAddr;
    static channel_t *weightElAddr;

    int i_point;
    int sensor_veto;

    /******** Obtain correct indexes the first time here ***********/
    if (firsttime) {
        firsttime = 0;

        latSipAddr = channels_find_by_name("lat_sip");
        lonSipAddr = channels_find_by_name("lon_sip");
        altSipAddr = channels_find_by_name("alt_sip");
        timeSipAddr = channels_find_by_name("time_sip");

        mksLoSipAddr = channels_find_by_name("mks_lo_sip");
        mksMedSipAddr = channels_find_by_name("mks_med_sip");
        mksHiSipAddr = channels_find_by_name("mks_hi_sip");

        OffsetIFelGYAddr = channels_find_by_name("offset_ifel_gy");
        OffsetIFelGYiscAddr = channels_find_by_name("offset_ifelxsc0_gy");
        OffsetIFrollGYiscAddr = channels_find_by_name("offset_ifrollxsc0_gy");
        OffsetIFyawGYiscAddr = channels_find_by_name("offset_ifyawxsc0_gy");
        OffsetIFelGYoscAddr = channels_find_by_name("offset_ifelxsc1_gy");
        OffsetIFrollGYoscAddr = channels_find_by_name("offset_ifrollxsc1_gy");
        OffsetIFyawGYoscAddr = channels_find_by_name("offset_ifyawxsc1_gy");
        OffsetIFrollGYAddr = channels_find_by_name("offset_ifroll_gy");
        OffsetIFyawGYAddr = channels_find_by_name("offset_ifyaw_gy");

        OffsetIFrollMagNGYAddr = channels_find_by_name("offset_ifrollmag1_gy");
        OffsetIFyawMagNGYAddr = channels_find_by_name("offset_ifyawmag1_gy");
        OffsetIFrollMagSGYAddr = channels_find_by_name("offset_ifrollmag2_gy");
        OffsetIFyawMagSGYAddr = channels_find_by_name("offset_ifyawmag2_gy");

        OffsetIFrollPSSGYAddr = channels_find_by_name("offset_ifrollpss_gy");
        OffsetIFyawPSSGYAddr = channels_find_by_name("offset_ifyawpss_gy");

        IFyawEarthGyAddr = channels_find_by_name("ifyaw_earth_gy");
        IFrollEarthGyAddr = channels_find_by_name("ifroll_earth_gy");
        IFelEarthGyAddr = channels_find_by_name("ifel_earth_gy");

        raAddr = channels_find_by_name("ra");
        decAddr = channels_find_by_name("dec");
        latAddr = channels_find_by_name("lat");
        altAddr = channels_find_by_name("alt");
        lonAddr = channels_find_by_name("lon");
        lstAddr = channels_find_by_name("lst");
        gy_azvel_addr = channels_find_by_name("gy_az_vel");
        gy_elvel_addr = channels_find_by_name("gy_el_vel");
        gy_totalvel_addr = channels_find_by_name("gy_total_vel");
        gy_totalaccel_addr = channels_find_by_name("gy_total_accel");

        azMagNAddr = channels_find_by_name("az_mag1");
        azRawMagNAddr = channels_find_by_name("az_raw_mag1");
        declinationMagNAddr = channels_find_by_name("declination_mag1");

        elMagNAddr = channels_find_by_name("pitch_mag1");
        dipMagNAddr = channels_find_by_name("dip_mag1");

        calXMaxMagNAddr = channels_find_by_name("cal_xmax_mag1");
        calXMinMagNAddr = channels_find_by_name("cal_xmin_mag1");
        calYMaxMagNAddr = channels_find_by_name("cal_ymax_mag1");
        calYMinMagNAddr = channels_find_by_name("cal_ymin_mag1");

        azMagSAddr = channels_find_by_name("az_mag2");
        azRawMagSAddr = channels_find_by_name("az_raw_mag2");
        declinationMagSAddr = channels_find_by_name("declination_mag2");

        elMagSAddr = channels_find_by_name("pitch_mag2");
        dipMagSAddr = channels_find_by_name("dip_mag2");

        calXMaxMagSAddr = channels_find_by_name("cal_xmax_mag2");
        calXMinMagSAddr = channels_find_by_name("cal_xmin_mag2");
        calYMaxMagSAddr = channels_find_by_name("cal_ymax_mag2");
        calYMinMagSAddr = channels_find_by_name("cal_ymin_mag2");

		calAzPssArrayAddr = channels_find_by_name("cal_az_pss_array");
        calAzPss1Addr = channels_find_by_name("cal_az_pss1");
        calAzPss2Addr = channels_find_by_name("cal_az_pss2");
        calAzPss3Addr = channels_find_by_name("cal_az_pss3");
        calAzPss4Addr = channels_find_by_name("cal_az_pss4");
        calAzPss5Addr = channels_find_by_name("cal_az_pss5");
        calAzPss6Addr = channels_find_by_name("cal_az_pss6");
        calDPss1Addr = channels_find_by_name("cal_d_pss1");
        calDPss2Addr = channels_find_by_name("cal_d_pss2");
        calDPss3Addr = channels_find_by_name("cal_d_pss3");
        calDPss4Addr = channels_find_by_name("cal_d_pss4");
        calDPss5Addr = channels_find_by_name("cal_d_pss5");
        calDPss6Addr = channels_find_by_name("cal_d_pss6");
        calIMinPssAddr = channels_find_by_name("cal_imin_pss");
        sigmaMagNAddr = channels_find_by_name("sigma_mag1");
        sigmaMagSAddr = channels_find_by_name("sigma_mag2");
        azNullAddr = channels_find_by_name("az_null");
        azSunAddr = channels_find_by_name("az_sun");
        elSunAddr = channels_find_by_name("el_sun");
        sigmaPssAddr = channels_find_by_name("sigma_pss");
        azrawPss1Addr = channels_find_by_name("az_raw_pss1");
        azrawPss2Addr = channels_find_by_name("az_raw_pss2");
        azrawPss3Addr = channels_find_by_name("az_raw_pss3");
        azrawPss4Addr = channels_find_by_name("az_raw_pss4");
        azrawPss5Addr = channels_find_by_name("az_raw_pss5");
        azrawPss6Addr = channels_find_by_name("az_raw_pss6");
        elrawPss1Addr = channels_find_by_name("el_raw_pss1");
        elrawPss2Addr = channels_find_by_name("el_raw_pss2");
        elrawPss3Addr = channels_find_by_name("el_raw_pss3");
        elrawPss4Addr = channels_find_by_name("el_raw_pss4");
        elrawPss5Addr = channels_find_by_name("el_raw_pss5");
        elrawPss6Addr = channels_find_by_name("el_raw_pss6");
        snrPss1Addr = channels_find_by_name("snr_pss1");
        snrPss2Addr = channels_find_by_name("snr_pss2");
        snrPss3Addr = channels_find_by_name("snr_pss3");
        snrPss4Addr = channels_find_by_name("snr_pss4");
        snrPss5Addr = channels_find_by_name("snr_pss5");
        snrPss6Addr = channels_find_by_name("snr_pss6");
        azPssAddr = channels_find_by_name("az_pss");  // evolved az
        PssOkAddr = channels_find_by_name("ok_pss");
        hwprCalAddr = channels_find_by_name("hwpr_cal");
        modeCalAddr = channels_find_by_name("mode_cal");
        periodCalAddr = channels_find_by_name("period_cal");
        elClinAddr = channels_find_by_name("el_clin");
        elLutClinAddr = channels_find_by_name("el_lut_clin");
        sigmaClinAddr = channels_find_by_name("sigma_clin");

        svetoLenAddr = channels_find_by_name("sveto_len");
        slewVetoAddr = channels_find_by_name("slew_veto");
        modePAddr = channels_find_by_name("mode_p");
        xPAddr = channels_find_by_name("x_p");
        yPAddr = channels_find_by_name("y_p");
        velAzPAddr = channels_find_by_name("vel_az_p");
        velElPAddr = channels_find_by_name("vel_el_p");
        delPAddr = channels_find_by_name("del_p");
        dazPAddr = channels_find_by_name("daz_p");
        wPAddr = channels_find_by_name("w_p");
        hPAddr = channels_find_by_name("h_p");
        ra1PAddr = channels_find_by_name("ra_1_p");
        dec1PAddr = channels_find_by_name("dec_1_p");
        ra2PAddr = channels_find_by_name("ra_2_p");
        dec2PAddr = channels_find_by_name("dec_2_p");
        ra3PAddr = channels_find_by_name("ra_3_p");
        dec3PAddr = channels_find_by_name("dec_3_p");
        ra4PAddr = channels_find_by_name("ra_4_p");
        dec4PAddr = channels_find_by_name("dec_4_p");
        nDithPAddr = channels_find_by_name("n_dith_p");
        nextIDithPAddr = channels_find_by_name("next_i_dith_p");
        nextIHwprPAddr = channels_find_by_name("next_i_hwpr_p");

        vetoSensorAddr = channels_find_by_name("veto_sensor");
        MagOKAddr[0] = channels_find_by_name("ok_mag1");
        MagOKAddr[1] = channels_find_by_name("ok_mag2");
        EncMotorOK = channels_find_by_name("ok_motor_enc");
        ElClinOK = channels_find_by_name("ok_elclin");
        DGPSOK = channels_find_by_name("ok_dgps");

        lstSchedAddr = channels_find_by_name("lst_sched");

        trimClinAddr = channels_find_by_name("trim_clin");
        trimEncAddr = channels_find_by_name("trim_enc");
        trimEncMotorAddr = channels_find_by_name("trim_motor_enc");  // This should be added as a channel
        trimNullAddr = channels_find_by_name("trim_null");
        trimMagNAddr = channels_find_by_name("trim_mag1");
        trimMagSAddr = channels_find_by_name("trim_mag2");
        trimPssAddr = channels_find_by_name("trim_pss");
        trimDGPSAddr = channels_find_by_name("trim_dgps");

        threshAtrimAddr = channels_find_by_name("thresh_cmd_atrim");
        timeAtrimAddr = channels_find_by_name("time_cmd_atrim");
        rateAtrimAddr = channels_find_by_name("rate_cmd_atrim");
        rateAtrimPtAddr = channels_find_by_name("rate_atrim");
        freshTrimAddr = channels_find_by_name("fresh_trim");
        newAzAddr = channels_find_by_name("new_az");
        newElAddr = channels_find_by_name("new_el");
        DGPSRawAzAddr = channels_find_by_name("az_raw_dgps");
        DGPSAzAddr = channels_find_by_name("az_dgps");
        DGPSSigmaAzAddr = channels_find_by_name("sigma_dgps");
        weightAzAddr = channels_find_by_name("weight_az");
        weightElAddr = channels_find_by_name("weight_el");
    }

    i_point = GETREADINDEX(point_index);

    /********* PSS data *************/
    SET_SCALED_VALUE(azrawPss1Addr, PointingData[i_point].pss_azraw[0]);
    SET_SCALED_VALUE(azrawPss2Addr, PointingData[i_point].pss_azraw[1]);
    SET_SCALED_VALUE(azrawPss3Addr, PointingData[i_point].pss_azraw[2]);
    SET_SCALED_VALUE(azrawPss4Addr, PointingData[i_point].pss_azraw[3]);
    SET_SCALED_VALUE(azrawPss5Addr, PointingData[i_point].pss_azraw[4]);
    SET_SCALED_VALUE(azrawPss6Addr, PointingData[i_point].pss_azraw[5]);
    SET_SCALED_VALUE(elrawPss1Addr, PointingData[i_point].pss_elraw[0]);
    SET_SCALED_VALUE(elrawPss2Addr, PointingData[i_point].pss_elraw[1]);
    SET_SCALED_VALUE(elrawPss3Addr, PointingData[i_point].pss_elraw[2]);
    SET_SCALED_VALUE(elrawPss4Addr, PointingData[i_point].pss_elraw[3]);
    SET_SCALED_VALUE(elrawPss5Addr, PointingData[i_point].pss_elraw[4]);
    SET_SCALED_VALUE(elrawPss6Addr, PointingData[i_point].pss_elraw[5]);
    SET_SCALED_VALUE(snrPss1Addr, PointingData[i_point].pss_snr[0]);
    SET_SCALED_VALUE(snrPss2Addr, PointingData[i_point].pss_snr[1]);
    SET_SCALED_VALUE(snrPss3Addr, PointingData[i_point].pss_snr[2]);
    SET_SCALED_VALUE(snrPss4Addr, PointingData[i_point].pss_snr[3]);
    SET_SCALED_VALUE(snrPss5Addr, PointingData[i_point].pss_snr[4]);
    SET_SCALED_VALUE(snrPss6Addr, PointingData[i_point].pss_snr[5]);
    // TODO(seth): Why are we manually adding the trim here?
    SET_SCALED_VALUE(azPssAddr, (PointingData[i_point].pss_az + CommandData.pss_az_trim));
    SET_SCALED_VALUE(PssOkAddr, PointingData[i_point].pss_ok);
    /********** SIP GPS Data **********/
    SET_SCALED_VALUE(latSipAddr, SIPData.GPSpos.lat);
    SET_SCALED_VALUE(lonSipAddr, SIPData.GPSpos.lon);
    SET_SCALED_VALUE(altSipAddr, SIPData.GPSpos.alt);
    SET_SCALED_VALUE(timeSipAddr, SIPData.GPStime.UTC);

    /********** SIP MKS Altitude ************/
    SET_SCALED_VALUE(mksLoSipAddr, SIPData.MKSalt.lo);
    SET_SCALED_VALUE(mksMedSipAddr, SIPData.MKSalt.med);
    SET_SCALED_VALUE(mksHiSipAddr, SIPData.MKSalt.hi);

    /************* processed pointing data *************/
    SET_SCALED_VALUE(raAddr, PointingData[i_point].ra);
    SET_SCALED_VALUE(decAddr, PointingData[i_point].dec);

    SET_SCALED_VALUE(OffsetIFelGYAddr, PointingData[i_point].offset_ifel_gy);
    SET_SCALED_VALUE(OffsetIFelGYiscAddr, PointingData[i_point].offset_ifel_gy_xsc[0]);
    SET_SCALED_VALUE(OffsetIFrollGYiscAddr, PointingData[i_point].offset_ifroll_gy_xsc[0]);
    SET_SCALED_VALUE(OffsetIFyawGYiscAddr, PointingData[i_point].offset_ifyaw_gy_xsc[0]);
    SET_SCALED_VALUE(OffsetIFelGYoscAddr, PointingData[i_point].offset_ifel_gy_xsc[1]);
    SET_SCALED_VALUE(OffsetIFrollGYoscAddr, PointingData[i_point].offset_ifroll_gy_xsc[1]);
    SET_SCALED_VALUE(OffsetIFyawGYoscAddr, PointingData[i_point].offset_ifyaw_gy_xsc[1]);
    SET_SCALED_VALUE(OffsetIFrollGYAddr, PointingData[i_point].offset_ifroll_gy);
    SET_SCALED_VALUE(OffsetIFyawGYAddr, PointingData[i_point].offset_ifyaw_gy);

    SET_SCALED_VALUE(OffsetIFrollMagNGYAddr, PointingData[i_point].offset_ifrollmag_gy[0]);
    SET_SCALED_VALUE(OffsetIFyawMagNGYAddr, PointingData[i_point].offset_ifyawmag_gy[0]);
    SET_SCALED_VALUE(OffsetIFrollMagSGYAddr, PointingData[i_point].offset_ifrollmag_gy[1]);
    SET_SCALED_VALUE(OffsetIFyawMagSGYAddr, PointingData[i_point].offset_ifyawmag_gy[1]);
    SET_SCALED_VALUE(OffsetIFrollPSSGYAddr, PointingData[i_point].offset_ifrollpss_gy);
    SET_SCALED_VALUE(OffsetIFyawPSSGYAddr, PointingData[i_point].offset_ifyawpss_gy);

    SET_SCALED_VALUE(IFyawEarthGyAddr, PointingData[i_point].ifyaw_earth_gy);
    SET_SCALED_VALUE(IFrollEarthGyAddr, PointingData[i_point].ifroll_earth_gy);
    SET_SCALED_VALUE(IFelEarthGyAddr, PointingData[i_point].ifel_earth_gy);

    SET_SCALED_VALUE(latAddr, PointingData[i_point].lat);
    SET_SCALED_VALUE(lonAddr, PointingData[i_point].lon);
    SET_SCALED_VALUE(altAddr, PointingData[i_point].alt);

//    SET_SCALED_VALUE(mcpFrameAddr, PointingData[i_point].mcp_frame);
    SET_SCALED_VALUE(lstAddr, PointingData[i_point].lst);
    // TODO(seth): Update LST Schedule channel
    SET_SCALED_VALUE(lstSchedAddr, 0);

    SET_SCALED_VALUE(azMagNAddr, (PointingData[i_point].mag_az[0] + CommandData.mag_az_trim[0]));
    SET_SCALED_VALUE(azRawMagNAddr, (PointingData[i_point].mag_az_raw[0]));
    SET_SCALED_VALUE(declinationMagNAddr, PointingData[i_point].mag_model_dec[0]);

    SET_SCALED_VALUE(elMagNAddr, PointingData[i_point].mag_el[0]);
    SET_SCALED_VALUE(dipMagNAddr, PointingData[i_point].mag_model_dip[0]);

    SET_SCALED_VALUE(calXMaxMagNAddr, CommandData.cal_xmax_mag[0]);
    SET_SCALED_VALUE(calXMinMagNAddr, CommandData.cal_xmin_mag[0]);
    SET_SCALED_VALUE(calYMaxMagNAddr, CommandData.cal_ymax_mag[0]);
    SET_SCALED_VALUE(calYMinMagNAddr, CommandData.cal_ymin_mag[0]);

    SET_SCALED_VALUE(azMagSAddr, (PointingData[i_point].mag_az[1] + CommandData.mag_az_trim[1]));
    SET_SCALED_VALUE(azRawMagSAddr, (PointingData[i_point].mag_az_raw[1]));
    SET_SCALED_VALUE(declinationMagSAddr, PointingData[i_point].mag_model_dec[1]);

    SET_SCALED_VALUE(elMagSAddr, PointingData[i_point].mag_el[1]);
    SET_SCALED_VALUE(dipMagSAddr, PointingData[i_point].mag_model_dip[1]);

    SET_SCALED_VALUE(calXMaxMagSAddr, CommandData.cal_xmax_mag[1]);
    SET_SCALED_VALUE(calXMinMagSAddr, CommandData.cal_xmin_mag[1]);
    SET_SCALED_VALUE(calYMaxMagSAddr, CommandData.cal_ymax_mag[1]);
    SET_SCALED_VALUE(calYMinMagSAddr, CommandData.cal_ymin_mag[1]);

	SET_SCALED_VALUE(calAzPssArrayAddr, CommandData.cal_az_pss_array);
    SET_SCALED_VALUE(calAzPss1Addr, CommandData.cal_az_pss[0]);
    SET_SCALED_VALUE(calAzPss2Addr, CommandData.cal_az_pss[1]);
    SET_SCALED_VALUE(calAzPss3Addr, CommandData.cal_az_pss[2]);
    SET_SCALED_VALUE(calAzPss4Addr, CommandData.cal_az_pss[3]);
    SET_SCALED_VALUE(calAzPss5Addr, CommandData.cal_az_pss[4]);
    SET_SCALED_VALUE(calAzPss6Addr, CommandData.cal_az_pss[5]);
    SET_SCALED_VALUE(calDPss1Addr, CommandData.cal_d_pss[0]);
    SET_SCALED_VALUE(calDPss2Addr, CommandData.cal_d_pss[1]);
    SET_SCALED_VALUE(calDPss3Addr, CommandData.cal_d_pss[2]);
    SET_SCALED_VALUE(calDPss4Addr, CommandData.cal_d_pss[3]);
    SET_SCALED_VALUE(calDPss5Addr, CommandData.cal_d_pss[4]);
    SET_SCALED_VALUE(calDPss6Addr, CommandData.cal_d_pss[5]);
    SET_SCALED_VALUE(calIMinPssAddr, CommandData.cal_imin_pss);

    SET_SCALED_VALUE(sigmaMagNAddr, PointingData[i_point].mag_sigma[0]);
    SET_SCALED_VALUE(trimMagNAddr, CommandData.mag_az_trim[0]);
    SET_SCALED_VALUE(sigmaMagSAddr, PointingData[i_point].mag_sigma[1]);
    SET_SCALED_VALUE(trimMagSAddr, CommandData.mag_az_trim[1]);

    SET_SCALED_VALUE(sigmaPssAddr, PointingData[i_point].pss_sigma);
    SET_SCALED_VALUE(trimPssAddr, CommandData.pss_az_trim);
    SET_SCALED_VALUE(trimDGPSAddr, CommandData.dgps_az_trim);

    SET_SCALED_VALUE(azSunAddr, PointingData[i_point].sun_az);
    SET_SCALED_VALUE(elSunAddr, PointingData[i_point].sun_el);

    SET_SCALED_VALUE(azNullAddr, PointingData[i_point].null_az);

    SET_SCALED_VALUE(hwprCalAddr, CommandData.Cryo.calib_hwpr);

    SET_SCALED_VALUE(trimEncAddr, CommandData.enc_el_trim);
    SET_SCALED_VALUE(trimEncMotorAddr, CommandData.enc_motor_el_trim);

    SET_SCALED_VALUE(elClinAddr, (PointingData[i_point].clin_el_lut + CommandData.clin_el_trim));
    SET_SCALED_VALUE(elLutClinAddr, PointingData[i_point].clin_el);
    SET_SCALED_VALUE(sigmaClinAddr, PointingData[i_point].clin_sigma);
    SET_SCALED_VALUE(trimClinAddr, CommandData.clin_el_trim);

    SET_SCALED_VALUE(trimNullAddr, CommandData.null_az_trim);

    SET_SCALED_VALUE(threshAtrimAddr, CommandData.autotrim_thresh);
    SET_SCALED_VALUE(timeAtrimAddr, CommandData.autotrim_time);
    SET_SCALED_VALUE(rateAtrimAddr, CommandData.autotrim_rate);
    SET_SCALED_VALUE(rateAtrimPtAddr, PointingData[i_point].autotrim_rate_xsc);
    SET_SCALED_VALUE(freshTrimAddr, PointingData[i_point].fresh);
    SET_SCALED_VALUE(newAzAddr, PointingData[i_point].new_az);
    SET_SCALED_VALUE(newElAddr, PointingData[i_point].new_el);

    SET_FLOAT(gy_azvel_addr, (float) (PointingData[i_point].gy_az));
    SET_FLOAT(gy_elvel_addr, (float) (PointingData[i_point].gy_el));
    SET_FLOAT(gy_totalvel_addr, (float) (PointingData[i_point].gy_total_vel));
    SET_FLOAT(gy_totalaccel_addr, (float) (PointingData[i_point].gy_total_accel));

    /************* Pointing mode fields *************/
    SET_SCALED_VALUE(slewVetoAddr, (CommandData.pointing_mode.nw));
    SET_SCALED_VALUE(svetoLenAddr, (CommandData.slew_veto));
    SET_SCALED_VALUE(nextIHwprPAddr, (CommandData.pointing_mode.next_i_hwpr));
    SET_SCALED_VALUE(nextIDithPAddr, (CommandData.pointing_mode.next_i_dith));
    SET_SCALED_VALUE(nDithPAddr, (CommandData.pointing_mode.n_dith));
    SET_SCALED_VALUE(modePAddr, (CommandData.pointing_mode.mode));
    if ((CommandData.pointing_mode.mode == P_AZEL_GOTO) || (CommandData.pointing_mode.mode == P_AZ_SCAN)
            || (CommandData.pointing_mode.mode == P_EL_SCAN))
    	SET_VALUE(xPAddr, (int) (CommandData.pointing_mode.X * DEG2I));
    else
    	SET_VALUE(xPAddr, (int) (CommandData.pointing_mode.X * H2I));

    SET_SCALED_VALUE(DGPSRawAzAddr, PointingData[i_point].dgps_az_raw);
    SET_SCALED_VALUE(DGPSAzAddr, PointingData[i_point].dgps_az + CommandData.dgps_az_trim);
    SET_SCALED_VALUE(DGPSSigmaAzAddr, PointingData[i_point].dgps_sigma);

    SET_SCALED_VALUE(yPAddr, CommandData.pointing_mode.Y);
    SET_SCALED_VALUE(velAzPAddr, CommandData.pointing_mode.vaz);
    SET_SCALED_VALUE(velElPAddr, CommandData.pointing_mode.vel);
    SET_SCALED_VALUE(delPAddr, CommandData.pointing_mode.del);
    SET_SCALED_VALUE(dazPAddr, CommandData.pointing_mode.daz);
    SET_SCALED_VALUE(wPAddr, CommandData.pointing_mode.w);
    SET_SCALED_VALUE(hPAddr, CommandData.pointing_mode.h);
    SET_SCALED_VALUE(ra1PAddr, CommandData.pointing_mode.ra[0]);
    SET_SCALED_VALUE(dec1PAddr, CommandData.pointing_mode.dec[0]);
    SET_SCALED_VALUE(ra2PAddr, CommandData.pointing_mode.ra[1]);
    SET_SCALED_VALUE(dec2PAddr, CommandData.pointing_mode.dec[1]);
    SET_SCALED_VALUE(ra3PAddr, CommandData.pointing_mode.ra[2]);
    SET_SCALED_VALUE(dec3PAddr, CommandData.pointing_mode.dec[2]);
    SET_SCALED_VALUE(ra4PAddr, CommandData.pointing_mode.ra[3]);
    SET_SCALED_VALUE(dec4PAddr, CommandData.pointing_mode.dec[3]);
    sensor_veto = ((!CommandData.use_elmotenc))
    		| ((!CommandData.use_xsc0) << 1) | ((!CommandData.use_elenc) << 2)
			| ((!CommandData.use_mag1) << 3)  | ((!CommandData.use_mag2) << 4)
			| ((!CommandData.use_elclin) << 5)
			| ((!CommandData.use_xsc1) << 6) | ((CommandData.uplink_sched) << 7)
			| ((CommandData.az_autogyro)  << 8) | ((CommandData.az_autogyro)  << 9)
			| ((CommandData.disable_el) << 10)
            | ((CommandData.disable_az) << 11) | ((CommandData.force_el) << 12)
			| ((!CommandData.use_pss) << 13) | ((!CommandData.use_dgps) << 14);

    if (PointingData[i_point].t >= CommandData.pointing_mode.t)
        sensor_veto |= (1 << 7);

    sensor_veto |= (CommandData.az_autogyro << 8);
    sensor_veto |= (CommandData.el_autogyro << 9);

    SET_UINT16(vetoSensorAddr, sensor_veto);
    SET_UINT8(MagOKAddr[0], PointingData[i_point].mag_ok[0]);
    SET_UINT8(MagOKAddr[1], PointingData[i_point].mag_ok[0]);
    SET_UINT8(EncMotorOK, PointingData[i_point].enc_motor_ok);
    SET_UINT8(ElClinOK, PointingData[i_point].clin_ok);
    SET_UINT8(DGPSOK, PointingData[i_point].dgps_ok);
    SET_UINT16(weightAzAddr, PointingData[i_point].weight_az);
    SET_UINT16(weightElAddr, PointingData[i_point].weight_el);
}
void store_1hz_acs(void)
{
    int i_point;
    static int firsttime = 1;
    static channel_t *OffsetIFrollDGPSGYAddr;
    static channel_t *OffsetIFyawDGPSGYAddr;
    static channel_t *latDGPSAddr;
    static channel_t *lonDGPSAddr;
    static channel_t *altDGPSAddr;
    static channel_t *qualityDGPSAddr;
    static channel_t *numSatDGPSAddr;
    if (firsttime) {
        OffsetIFrollDGPSGYAddr = channels_find_by_name("offset_ifrolldgps_gy");
        OffsetIFyawDGPSGYAddr = channels_find_by_name("offset_ifyawdgps_gy");
        latDGPSAddr = channels_find_by_name("lat_dgps");
        lonDGPSAddr = channels_find_by_name("lon_dgps");
        altDGPSAddr = channels_find_by_name("alt_dgps");
        qualityDGPSAddr = channels_find_by_name("quality_dgps");
        numSatDGPSAddr = channels_find_by_name("num_sat_dgps");
        firsttime = 0;
    }
    i_point = GETREADINDEX(point_index);
    SET_SCALED_VALUE(OffsetIFrollDGPSGYAddr, PointingData[i_point].offset_ifyawdgps_gy);
    SET_SCALED_VALUE(OffsetIFrollDGPSGYAddr, PointingData[i_point].offset_ifrolldgps_gy);
    SET_SCALED_VALUE(latDGPSAddr, CSBFGPSData.latitude);
    SET_SCALED_VALUE(lonDGPSAddr, CSBFGPSData.longitude);
    SET_SCALED_VALUE(altDGPSAddr, CSBFGPSData.altitude);
    SET_INT8(qualityDGPSAddr, CSBFGPSData.quality);
    SET_INT8(numSatDGPSAddr, CSBFGPSData.num_sat);
}
