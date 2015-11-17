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

#include <blast_sip_interface.h>
#include <conversions.h>
#include <channels_tng.h>
#include <command_struct.h>
#include <ec_motors.h>
#include <motors.h>
#include <mcp.h>
#include <pointing_struct.h>
#include <dsp1760.h>

#include "xsc_network.h"

static const float gy_inv[64][3][6] =
        {

        /* mask = 000000 (0) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000001 (1) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000010 (2) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000011 (3) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000100 (4) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000101 (5) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000110 (6) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 000111 (7) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001000 (8) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001001 (9) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001010 (10) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001011 (11) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001100 (12) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001101 (13) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001110 (14) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 001111 (15) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 } },

        /* mask = 010000 (16) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010001 (17) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010010 (18) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010011 (19) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010100 (20) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010101 (21) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010110 (22) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 010111 (23) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011000 (24) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011001 (25) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011010 (26) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011011 (27) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011100 (28) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011101 (29) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011110 (30) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 011111 (31) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00 } },

        /* mask = 100000 (32) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100001 (33) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100010 (34) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100011 (35) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100100 (36) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100101 (37) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100110 (38) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 100111 (39) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101000 (40) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101001 (41) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101010 (42) */
        { { 0.000000e+00, 0.72055111, 0.000000e+00, 0.000000e+00, 0.000000e+00, -0.69340183 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, -1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, -0.72055111, 0.000000e+00, 0.000000e+00, 0.000000e+00, -0.69340183 } },

        /* mask = 101011 (43) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101100 (44) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101101 (45) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101110 (46) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 101111 (47) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

        /* mask = 110000 (48) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110001 (49) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110010 (50) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110011 (51) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110100 (52) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110101 (53) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110110 (54) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 110111 (55) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111000 (56) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111001 (57) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111010 (58) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111011 (59) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111100 (60) */
        { { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111101 (61) */
        { { 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111110 (62) */
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } },

        /* mask = 111111 (63) */
        { { 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 5.000000e-01, 5.000000e-01 } }

        };
//TODO: Extern sched_lst after enabling sched.c
unsigned int sched_lst; /* sched_lst */

/**
 * Reads the 5Hz data from the most recent frame received from UEIs and stores
 * it into the ACSData structure for use in pointing
 */
void read_5hz_acs(void)
{

  static channel_t* elRawIfClinAddr;
  static channel_t* v11PssAddr;
  static channel_t* v21PssAddr;
  static channel_t* v31PssAddr;
  static channel_t* v41PssAddr;
  static channel_t* v12PssAddr;
  static channel_t* v22PssAddr;
  static channel_t* v32PssAddr;
  static channel_t* v42PssAddr;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    elRawIfClinAddr = channels_find_by_name("el_raw_if_clin");
    v11PssAddr = channels_find_by_name("v1_1_pss");
    v21PssAddr = channels_find_by_name("v2_1_pss");
    v31PssAddr = channels_find_by_name("v3_1_pss");
    v41PssAddr = channels_find_by_name("v4_1_pss");
    v12PssAddr = channels_find_by_name("v1_2_pss");
    v22PssAddr = channels_find_by_name("v2_2_pss");
    v32PssAddr = channels_find_by_name("v3_2_pss");
    v42PssAddr = channels_find_by_name("v4_2_pss");
  }

  ACSData.pss1_i1 = GET_UINT16(v11PssAddr);
  ACSData.pss1_i2 = GET_UINT16(v21PssAddr);
  ACSData.pss1_i3 = GET_UINT16(v31PssAddr);
  ACSData.pss1_i4 = GET_UINT16(v41PssAddr);

  ACSData.pss2_i1 = GET_UINT16(v12PssAddr);
  ACSData.pss2_i2 = GET_UINT16(v22PssAddr);
  ACSData.pss2_i3 = GET_UINT16(v32PssAddr);
  ACSData.pss2_i4 = GET_UINT16(v42PssAddr);

  ///TODO: Add PSS3-8 read functions

  ACSData.clin_elev = GET_UINT16(elRawIfClinAddr);
}
/**
 * Reads the 100Hz data from the most recent frame received from UEIs and stores
 * it into the ACSData structure for use in pointing
 */
void read_100hz_acs(void)
{
    static channel_t* xMagAddr;
    static channel_t* yMagAddr;
    static channel_t* zMagAddr;

    static int firsttime = 1;
    if (firsttime) {
      firsttime = 0;
        xMagAddr = channels_find_by_name("x_mag");
        yMagAddr = channels_find_by_name("y_mag");
        zMagAddr = channels_find_by_name("z_mag");
    }

    ACSData.mag_x = GET_UINT16(xMagAddr);
    ACSData.mag_y = GET_UINT16(yMagAddr);
    ACSData.mag_z = GET_UINT16(zMagAddr);

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

      mask_gy_addr = channels_find_by_name("mask_gy");
      fault_gy_addr = channels_find_by_name("fault_gy");
    }

    gymask = GET_UINT16(mask_gy_addr);

    ifel_gy1 = dsp1760_getval(0,0);
    ifroll_gy1 = dsp1760_getval(0,1);
    ifyaw_gy1 = dsp1760_getval(0,2);
    SET_FLOAT(ifElgy1Addr, ifel_gy1);
    SET_FLOAT(ifRollgy1Addr, ifroll_gy1);
    SET_FLOAT(ifYawgy1Addr, ifyaw_gy1);

    ifel_gy2 = dsp1760_getval(1,0);
    ifroll_gy2 = dsp1760_getval(1,1);
    ifyaw_gy2 = dsp1760_getval(1,2);
    SET_FLOAT(ifElgy2Addr, ifel_gy2);
    SET_FLOAT(ifRollgy2Addr, ifroll_gy2);
    SET_FLOAT(ifYawgy2Addr, ifyaw_gy2);

    gy_ifroll= gy_inv[gymask][0][0]*ifroll_gy1 + gy_inv[gymask][0][1]*ifroll_gy2 + gy_inv[gymask][0][2]*ifyaw_gy1 + gy_inv[gymask][0][3]*ifyaw_gy2 + gy_inv[gymask][0][4]*ifel_gy1 + gy_inv[gymask][0][5]*ifel_gy2;
    gy_ifyaw = gy_inv[gymask][1][0]*ifroll_gy1 + gy_inv[gymask][1][1]*ifroll_gy2 + gy_inv[gymask][1][2]*ifyaw_gy1 + gy_inv[gymask][1][3]*ifyaw_gy2 + gy_inv[gymask][1][4]*ifel_gy1 + gy_inv[gymask][1][5]*ifel_gy2;
    gy_ifel  = gy_inv[gymask][2][0]*ifroll_gy1 + gy_inv[gymask][2][1]*ifroll_gy2 + gy_inv[gymask][2][2]*ifyaw_gy1 + gy_inv[gymask][2][3]*ifyaw_gy2 + gy_inv[gymask][2][4]*ifel_gy1 + gy_inv[gymask][2][5]*ifel_gy2;
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
            if (gyro_valid == gyro_valid_count[box][gyro]) gyro_valid_set[box][gyro]++;
            else gyro_valid_set[box][gyro] = 0;

            if (gyro_valid_set[box][gyro] > 1) gyfault |= (1 << (gyro * 2 + box));
            else gyfault &= ~(1 << (gyro * 2 + box));
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
    static channel_t *encstatus_el_addr;
    static channel_t *pos_el_addr;
    static channel_t *pos_motor_el_addr;
    static channel_t *vel_piv_addr;
    static channel_t *pos_piv_addr;

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
        encstatus_el_addr = channels_find_by_name("mc_el_biss_status");
        pos_el_addr = channels_find_by_name("mc_el_pos");
        pos_motor_el_addr = channels_find_by_name("mc_el_motor_pos");

        vel_piv_addr = channels_find_by_name("mc_piv_vel");
        pos_piv_addr = channels_find_by_name("mc_piv_pos");

    }
    i_point = GETREADINDEX(point_index);
    i_motors = GETREADINDEX(motor_index);

    SET_INT32(azAddr, (unsigned int) (PointingData[i_point].az * DEG2LI));
    SET_INT32(elAddr, (unsigned int) (PointingData[i_point].el * DEG2LI));

    SET_INT16(elEncAddr, (unsigned int) ((PointingData[i_point].enc_el + CommandData.enc_el_trim) * DEG2I));
    SET_INT16(sigmaEncAddr, (unsigned int) (PointingData[i_point].enc_sigma * DEG2I));
    SET_INT16(elMotEncAddr, (unsigned int) ((PointingData[i_point].enc_motor_el + CommandData.enc_el_trim) * DEG2I));
    SET_INT16(sigmaMotEncAddr, (unsigned int) (PointingData[i_point].enc_motor_sigma * DEG2I));

    SET_INT32(vel_rw_addr, RWMotorData[i_motors].velocity);
    SET_INT32(pos_rw_addr, RWMotorData[i_motors].position);
    SET_INT32(vel_el_addr, ElevMotorData[i_motors].velocity);
    SET_INT32(encstatus_el_addr, ElevMotorData[i_motors].load_state);
    SET_INT32(pos_el_addr, ElevMotorData[i_motors].position);
    SET_INT32(pos_motor_el_addr, ElevMotorData[i_motors].motor_position);
    SET_INT32(vel_piv_addr, PivotMotorData[i_motors].velocity);
    SET_INT32(pos_piv_addr, PivotMotorData[i_motors].position);
}

static inline channel_t* get_xsc_channel(const char *m_field, int m_which)
{
  char buffer[FIELD_LEN];
  const char prefix[2][3] = {"x0", "x1"};
  sprintf(buffer, "%s_%s", prefix[m_which], m_field);
  return channels_find_by_name(buffer);
}

void store_1hz_xsc(int m_which)
{
    static bool firsttime[2] = {true, true};

    static channel_t* address_xN_last_trig_motion_caz_px[2];
    static channel_t* address_xN_last_trig_motion_el_px[2];
    static channel_t* address_xN_last_trig_motion_px[2];
    static channel_t* address_xN_trigreq_meas_azvel[2];
    static channel_t* address_xN_trigreq_req_azvel[2];
    static channel_t* address_xN_trigreq_meas_totvel[2];
    static channel_t* address_xN_trigreq_meas_azacc[2];
    static channel_t* address_xN_point_az_raw[2];
    static channel_t* address_xN_point_az[2];
    static channel_t* address_xN_point_el_raw[2];
    static channel_t* address_xN_point_el[2];
    static channel_t* address_xN_point_sigma[2];
    static channel_t* address_xN_point_az_trim[2];
    static channel_t* address_xN_point_el_trim[2];
    static channel_t* address_xN_cd_robust_mode[2];
    static channel_t* address_xN_cd_motion_psf[2];
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

    static channel_t *address_xN_image_num_blobs[2];

    int i_point = GETREADINDEX(point_index);

    if (firsttime[m_which]) {
        firsttime[m_which] = false;

        address_xN_image_num_blobs[m_which] = get_xsc_channel("image_num_blobs", m_which);

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

        address_xN_last_trig_motion_caz_px[m_which]  = get_xsc_channel("last_trig_motion_caz_px"     , m_which);
        address_xN_last_trig_motion_el_px[m_which]  = get_xsc_channel("last_trig_motion_el_px"     , m_which);
        address_xN_last_trig_motion_px[m_which]  = get_xsc_channel("last_trig_motion_px"     , m_which);
        address_xN_trigreq_meas_azvel[m_which]   = get_xsc_channel("trigreq_meas_azvel"      , m_which);
        address_xN_trigreq_req_azvel[m_which]    = get_xsc_channel("trigreq_req_azvel"       , m_which);
        address_xN_trigreq_meas_totvel[m_which]  = get_xsc_channel("trigreq_meas_totvel"     , m_which);
        address_xN_trigreq_meas_azacc[m_which]   = get_xsc_channel("trigreq_meas_azacc"      , m_which);
        address_xN_point_az_raw[m_which]  = get_xsc_channel("point_az_raw"    , m_which);
        address_xN_point_az[m_which]      = get_xsc_channel("point_az"        , m_which);
        address_xN_point_el_raw[m_which]  = get_xsc_channel("point_el_raw"    , m_which);
        address_xN_point_el[m_which]      = get_xsc_channel("point_el"        , m_which);
        address_xN_point_sigma[m_which]   = get_xsc_channel("point_sigma"     , m_which);
        address_xN_point_az_trim[m_which] = get_xsc_channel("point_az_trim"   , m_which);
        address_xN_point_el_trim[m_which] = get_xsc_channel("point_el_trim"   , m_which);
        address_xN_cd_robust_mode[m_which] = get_xsc_channel("cd_robust_mode"   , m_which);
        address_xN_cd_motion_psf[m_which] = get_xsc_channel("cd_motion_psf"   , m_which);
//        address_xN_num_images_saved[m_which] = get_xsc_channel("num_images_saved"   , m_which);
        if (m_which == 0) {
            address_xN_last_trig_lat                = get_xsc_channel("last_trig_lat"        , 0);
            address_xN_last_trig_lst                = get_xsc_channel("last_trig_lst"        , 0);
        }
    }

    SET_VALUE(address_xN_image_num_blobs[m_which], (XSC_SERVER_DATA(m_which).channels.image_num_blobs_found << 6) |
            (XSC_SERVER_DATA(m_which).channels.image_num_blobs_matched & 0b111111));

    SET_VALUE(address_xN_hk_temp_lens[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_lens);
    SET_VALUE(address_xN_hk_temp_comp[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_comp);
    SET_VALUE(address_xN_hk_temp_plate[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_plate);
    SET_VALUE(address_xN_hk_temp_flange[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_flange);
    SET_VALUE(address_xN_hk_pressure[m_which], XSC_SERVER_DATA(m_which).channels.hk_pressure);
    SET_VALUE(address_xN_hk_disk[m_which], XSC_SERVER_DATA(m_which).channels.hk_disk);

    SET_VALUE(address_xN_image_eq_valid[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_valid);
    SET_VALUE(address_xN_cam_gain_valid[m_which], XSC_SERVER_DATA(m_which).channels.cam_gain_valid);
    SET_VALUE(address_xN_image_hor_valid[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_valid);
    SET_VALUE(address_xN_image_afocus_metric_valid[m_which], XSC_SERVER_DATA(m_which).channels.image_afocus_metric_valid);

    SET_VALUE(address_xN_stars_run_time[m_which], XSC_SERVER_DATA(m_which).channels.stars_run_time);
    SET_VALUE(address_xN_cam_gain_db[m_which], XSC_SERVER_DATA(m_which).channels.cam_gain_db);
    SET_VALUE(address_xN_lens_focus[m_which], XSC_SERVER_DATA(m_which).channels.lens_focus);
    SET_VALUE(address_xN_lens_aperture[m_which], XSC_SERVER_DATA(m_which).channels.lens_aperture);

    SET_VALUE(address_xN_image_num_exposures[m_which], XSC_SERVER_DATA(m_which).channels.image_num_exposures);
    SET_VALUE(address_xN_image_stats_mean[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_mean);
    SET_VALUE(address_xN_image_stats_noise[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_noise);
    SET_VALUE(address_xN_image_stats_gaindb[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_gaindb);
    SET_VALUE(address_xN_image_stats_num_px_sat[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_num_px_sat);
    SET_VALUE(address_xN_image_stats_frac_px_sat[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_frac_px_sat);
    SET_VALUE(address_xN_image_afocus_metric[m_which], XSC_SERVER_DATA(m_which).channels.image_afocus_metric);

    SET_VALUE(address_xN_image_eq_iplate[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_iplate);
    SET_VALUE(address_xN_image_hor_iplate[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_iplate);

    SET_VALUE(address_xN_image_eq_ra[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_ra);
    SET_VALUE(address_xN_image_eq_dec[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_dec);
    SET_VALUE(address_xN_image_eq_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_roll);
    SET_VALUE(address_xN_image_eq_sigma_ra[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_ra);
    SET_VALUE(address_xN_image_eq_sigma_dec[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_dec);
    SET_VALUE(address_xN_image_eq_sigma_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_roll);
    SET_VALUE(address_xN_image_eq_sigma_pointing[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_pointing);
    SET_VALUE(address_xN_image_hor_az[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_az);
    SET_VALUE(address_xN_image_hor_el[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_el);
    SET_VALUE(address_xN_image_hor_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_roll);
    SET_VALUE(address_xN_image_hor_sigma_az[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_az);
    SET_VALUE(address_xN_image_hor_sigma_el[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_el);
    SET_VALUE(address_xN_image_hor_sigma_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_roll);
    SET_VALUE(address_xN_image_hor_sigma_pointing[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_pointing);

    SET_VALUE(address_xN_last_trig_motion_caz_px[m_which], TX_CONVERT(VELOCITY, xsc_pointing_state[m_which].last_trigger.motion_caz_px));
    SET_VALUE(address_xN_last_trig_motion_el_px[m_which], TX_CONVERT(VELOCITY, xsc_pointing_state[m_which].last_trigger.motion_el_px));
    double motion_px = sqrt(pow(xsc_pointing_state[m_which].last_trigger.motion_caz_px, 2.0) + pow(xsc_pointing_state[m_which].last_trigger.motion_el_px, 2.0));
    SET_VALUE(address_xN_last_trig_motion_px[m_which], TX_CONVERT(VELOCITY, motion_px));
    SET_VALUE(address_xN_trigreq_meas_azvel[m_which]   , 0);
    SET_VALUE(address_xN_trigreq_req_azvel[m_which]    , 0);
    SET_VALUE(address_xN_trigreq_meas_totvel[m_which]  , 0);
    SET_VALUE(address_xN_trigreq_meas_azacc[m_which]   , 0);
    SET_VALUE(address_xN_point_az[m_which]     , TX_CONVERT(ANGLE_DEGREES, PointingData[i_point].xsc_az[m_which]));
    SET_VALUE(address_xN_point_el[m_which]     , TX_CONVERT(ANGLE_DEGREES, PointingData[i_point].xsc_el[m_which]));
    SET_VALUE(address_xN_point_sigma[m_which]  , TX_CONVERT(ANGLE_DEGREES, PointingData[i_point].xsc_sigma[m_which]));
    SET_VALUE(address_xN_point_az_raw[m_which] , TX_CONVERT(ANGLE_DEGREES, xsc_pointing_state[m_which].az));
    SET_VALUE(address_xN_point_el_raw[m_which] , TX_CONVERT(ANGLE_DEGREES, xsc_pointing_state[m_which].el));
    SET_VALUE(address_xN_point_az_trim[m_which], TX_CONVERT(ANGLE, CommandData.XSC[m_which].cross_el_trim));
    SET_VALUE(address_xN_point_el_trim[m_which], TX_CONVERT(ANGLE, CommandData.XSC[m_which].el_trim));
    SET_VALUE(address_xN_cd_robust_mode[m_which], CommandData.XSC[m_which].net.solver.robust_mode_enabled);
    SET_VALUE(address_xN_cd_motion_psf[m_which], CommandData.XSC[m_which].net.solver.motion_psf.enabled);
    ///TODO: Re-add local image saving
//    SET_VALUE(address_xN_num_images_saved[m_which], images_num_saved[m_which]);
    if (m_which == 0) {
        SET_VALUE(address_xN_last_trig_lat       , xsc_pointing_state[m_which].last_trigger.lat*DEG2LI);
        SET_VALUE(address_xN_last_trig_lst       , xsc_pointing_state[m_which].last_trigger.lst);
    }
}

void store_100hz_xsc(int which)
{
    static bool firsttime[2] = {true, true};
    static int last_blob_counter_stars[2] = {-1, -1};
    static int last_blob_i[2] = {1000, 1000};
    static int intermediate_frame_counter[2] = {0, 0};

    static channel_t* address_xN_ctr_stars[2];
    static channel_t* address_xN_image_ctr_fcp[2];
    static channel_t* address_xN_image_ctr_stars[2];

    static channel_t* address_xN_ctr_fcp;
    static channel_t* address_xN_last_trig_age_cs;
    static channel_t* address_xN_last_trig_ctr_fcp;
    static channel_t* address_xN_last_trig_ctr_stars[2];
    static channel_t* address_xN_predicted_motion_px;
    static channel_t* address_xN_image_blobn_x[2];
    static channel_t* address_xN_image_blobn_y[2];
    static channel_t* address_xN_image_blobn_flux[2];
    static channel_t* address_xN_image_blobn_peak_to_flux[2];

    if (firsttime[which]) {
        firsttime[which] = false;

        if (which == 0) {
            address_xN_ctr_fcp                      = get_xsc_channel("ctr_fcp"              , 0);
            address_xN_last_trig_age_cs             = get_xsc_channel("last_trig_age_cs"     , 0);
            address_xN_last_trig_ctr_fcp            = get_xsc_channel("last_trig_ctr_fcp"    , 0);
            address_xN_predicted_motion_px          = get_xsc_channel("predicted_motion_px"     , which);
        }
        address_xN_ctr_stars[which]                 = get_xsc_channel("ctr_stars", which);
        address_xN_image_ctr_stars[which]           = get_xsc_channel("image_ctr_stars", which);
        address_xN_image_ctr_fcp[which]             = get_xsc_channel("image_ctr_fcp", which);
        address_xN_last_trig_ctr_stars[which]       = get_xsc_channel("last_trig_ctr_stars"     , which);
        address_xN_image_blobn_x[which]             = get_xsc_channel("image_blobn_x"   , which);
        address_xN_image_blobn_y[which]             = get_xsc_channel("image_blobn_y"   , which);
        address_xN_image_blobn_flux[which]          = get_xsc_channel("image_blobn_flux"   , which);
        address_xN_image_blobn_peak_to_flux[which]  = get_xsc_channel("image_blobn_peak_to_flux"   , which);
    }

    if (which == 0) {
        SET_VALUE(address_xN_ctr_fcp             , xsc_pointing_state[which].counter_fcp);
        SET_VALUE(address_xN_last_trig_age_cs    , xsc_pointing_state[which].last_trigger.age_cs);
        SET_VALUE(address_xN_last_trig_ctr_fcp   , xsc_pointing_state[which].last_trigger.counter_fcp);
        SET_VALUE(address_xN_predicted_motion_px, TX_CONVERT(VELOCITY, xsc_pointing_state[0].predicted_motion_px));
    }


    SET_INT32(address_xN_ctr_stars[which], XSC_SERVER_DATA(which).channels.ctr_stars);
    SET_INT32(address_xN_image_ctr_stars[which], XSC_SERVER_DATA(which).channels.image_ctr_stars);
    SET_INT32(address_xN_image_ctr_fcp[which], XSC_SERVER_DATA(which).channels.image_ctr_fcp);
    SET_VALUE(address_xN_last_trig_ctr_stars[which], xsc_pointing_state[which].last_trigger.counter_stars);

    if (XSC_SERVER_DATA(which).blobs.counter_stars != last_blob_counter_stars[which] &&
        XSC_SERVER_DATA(which).blobs.counter_stars > 0)
    {
        last_blob_counter_stars[which] = XSC_SERVER_DATA(which).blobs.counter_stars;
        last_blob_i[which] = 0;
    }
    if (intermediate_frame_counter[which] == 0) {
        if (last_blob_i[which] < XSC_SERVER_DATA(which).blobs.num_blobs) {
            SET_VALUE(address_xN_image_blobn_x[which], TX_CONVERT(BLOB_POS, XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].x));
            SET_VALUE(address_xN_image_blobn_y[which], TX_CONVERT(BLOB_POS, XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].y));
            SET_VALUE(address_xN_image_blobn_flux[which], XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].flux);
            SET_VALUE(address_xN_image_blobn_peak_to_flux[which], TX_CONVERT(0_TO_10, XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].peak_to_flux));
            last_blob_i[which]++;
        } else if (last_blob_i[which] == XSC_SERVER_DATA(which).blobs.num_blobs) {
            SET_VALUE(address_xN_image_blobn_x[which], 0);
            SET_VALUE(address_xN_image_blobn_y[which], 0);
            SET_VALUE(address_xN_image_blobn_flux[which], 0);
            SET_VALUE(address_xN_image_blobn_peak_to_flux[which], 0);
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
    static channel_t *azGyAddr;
    static channel_t* OffsetIFelGYAddr;
    static channel_t* OffsetIFelGYiscAddr;
    static channel_t* OffsetIFrollGYiscAddr;
    static channel_t* OffsetIFyawGYiscAddr;
    static channel_t* OffsetIFelGYoscAddr;
    static channel_t* OffsetIFrollGYoscAddr;
    static channel_t* OffsetIFyawGYoscAddr;
    static channel_t* OffsetIFrollGYAddr;
    static channel_t* OffsetIFyawGYAddr;
    static channel_t* OffsetIFrollMagGYAddr;
    static channel_t* OffsetIFyawMagGYAddr;
    static channel_t* OffsetIFrollPSSGYAddr;
    static channel_t* OffsetIFyawPSSGYAddr;
    static channel_t* raAddr;
    static channel_t* decAddr;
    static channel_t* altAddr;
    static channel_t* latAddr;
    static channel_t* lonAddr;
    static channel_t* lstAddr;
    static channel_t* azMagAddr;
    static channel_t* azRawMagAddr;
    static channel_t* declinationMagAddr;
    static channel_t* elMagAddr;
    static channel_t* dipMagAddr;
    static channel_t* calXMaxMagAddr;
    static channel_t* calXMinMagAddr;
    static channel_t* calYMaxMagAddr;
    static channel_t* calYMinMagAddr;
    static channel_t* calOffPss1Addr;
    static channel_t* calOffPss2Addr;
    static channel_t* calOffPss3Addr;
    static channel_t* calOffPss4Addr;
    static channel_t* calDPss1Addr;
    static channel_t* calDPss2Addr;
    static channel_t* calDPss3Addr;
    static channel_t* calDPss4Addr;
    static channel_t* calIMinPssAddr;
    static channel_t* sigmaMagAddr;
    static channel_t* sigmaPssAddr;
    static channel_t* azrawPss1Addr;
    static channel_t* azrawPss2Addr;
    static channel_t* elrawPss1Addr;
    static channel_t* elrawPss2Addr;
    static channel_t* snrPss1Addr;
    static channel_t* snrPss2Addr;
    static channel_t* azPssAddr;
    static channel_t* PssOkAddr;
    static channel_t* azSunAddr;
    static channel_t* elSunAddr;
    static channel_t* azIscAddr;
    static channel_t* elIscAddr;
    static channel_t* sigmaIscAddr;
    static channel_t* azOscAddr;
    static channel_t* elOscAddr;
    static channel_t* sigmaOscAddr;
    static channel_t* elClinAddr;
    static channel_t* elLutClinAddr;
    static channel_t* sigmaClinAddr;

    /* trim fields */
    static channel_t *trimClinAddr;
    static channel_t *trimEncAddr;
    static channel_t *trimNullAddr;
    static channel_t *trimMagAddr;
    static channel_t *trimPssAddr;

    static channel_t *threshAtrimAddr;
    static channel_t *timeAtrimAddr;
    static channel_t *rateAtrimAddr;

    static channel_t *modeCalAddr;
    static channel_t *hwprCalAddr;
    static channel_t *periodCalAddr;
    static channel_t *lstSchedAddr;


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
        OffsetIFelGYiscAddr = channels_find_by_name("off_ifel_gy_isc");
        OffsetIFrollGYiscAddr = channels_find_by_name("off_ifroll_gy_isc");
        OffsetIFyawGYiscAddr = channels_find_by_name("off_ifyaw_gy_isc");
        OffsetIFelGYoscAddr = channels_find_by_name("off_ifel_gy_osc");
        OffsetIFrollGYoscAddr = channels_find_by_name("off_ifroll_gy_osc");
        OffsetIFyawGYoscAddr = channels_find_by_name("off_ifyaw_gy_osc");
        OffsetIFrollGYAddr = channels_find_by_name("offset_ifroll_gy");
        OffsetIFyawGYAddr = channels_find_by_name("offset_ifyaw_gy");

        OffsetIFrollMagGYAddr = channels_find_by_name("offset_ifrollmag_gy");
        OffsetIFyawMagGYAddr = channels_find_by_name("offset_ifyawmag_gy");

        OffsetIFrollPSSGYAddr = channels_find_by_name("offset_ifrollpss_gy");
        OffsetIFyawPSSGYAddr = channels_find_by_name("offset_ifyawpss_gy");

        raAddr = channels_find_by_name("ra");
        decAddr = channels_find_by_name("dec");
        latAddr = channels_find_by_name("lat");
        altAddr = channels_find_by_name("alt");
        lonAddr = channels_find_by_name("lon");
        lstAddr = channels_find_by_name("lst");
        azGyAddr = channels_find_by_name("az_gy");
        azMagAddr = channels_find_by_name("az_mag");
        azRawMagAddr = channels_find_by_name("az_raw_mag");
        declinationMagAddr = channels_find_by_name("declination_mag");

        elMagAddr = channels_find_by_name("el_mag");
        dipMagAddr = channels_find_by_name("dip_mag");

        calXMaxMagAddr = channels_find_by_name("cal_xmax_mag");
        calXMinMagAddr = channels_find_by_name("cal_xmin_mag");
        calYMaxMagAddr = channels_find_by_name("cal_ymax_mag");
        calYMinMagAddr = channels_find_by_name("cal_ymin_mag");
        calOffPss1Addr = channels_find_by_name("cal_off_pss1");
        calOffPss2Addr = channels_find_by_name("cal_off_pss2");
        calOffPss3Addr = channels_find_by_name("cal_off_pss3");
        calOffPss4Addr = channels_find_by_name("cal_off_pss4");
        calDPss1Addr = channels_find_by_name("cal_d_pss1");
        calDPss2Addr = channels_find_by_name("cal_d_pss2");
        calDPss3Addr = channels_find_by_name("cal_d_pss3");
        calDPss4Addr = channels_find_by_name("cal_d_pss4");
        calIMinPssAddr = channels_find_by_name("cal_imin_pss");
        sigmaMagAddr = channels_find_by_name("sigma_mag");
        azSunAddr = channels_find_by_name("az_sun");
        elSunAddr = channels_find_by_name("el_sun");
        sigmaPssAddr = channels_find_by_name("sigma_pss");
        azrawPss1Addr = channels_find_by_name("az_raw_pss1");
        azrawPss2Addr = channels_find_by_name("az_raw_pss2");
        elrawPss1Addr = channels_find_by_name("el_raw_pss1");
        elrawPss2Addr = channels_find_by_name("el_raw_pss2");
        snrPss1Addr = channels_find_by_name("snr_pss1");
        snrPss2Addr = channels_find_by_name("snr_pss2");
        azPssAddr = channels_find_by_name("az_pss");  // evolved az
        PssOkAddr = channels_find_by_name("ok_pss");
        hwprCalAddr = channels_find_by_name("hwpr_cal");
        modeCalAddr = channels_find_by_name("mode_cal");
        periodCalAddr = channels_find_by_name("period_cal");
        azIscAddr = channels_find_by_name("az_isc");
        elIscAddr = channels_find_by_name("el_isc");
        sigmaIscAddr = channels_find_by_name("sigma_isc");
        azOscAddr = channels_find_by_name("az_osc");
        elOscAddr = channels_find_by_name("el_osc");
        sigmaOscAddr = channels_find_by_name("sigma_osc");
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

        lstSchedAddr = channels_find_by_name("lst_sched");

        trimClinAddr = channels_find_by_name("trim_clin");
        trimEncAddr = channels_find_by_name("trim_enc");
        trimNullAddr = channels_find_by_name("trim_null");
        trimMagAddr = channels_find_by_name("trim_mag");
        trimPssAddr = channels_find_by_name("trim_pss");

        threshAtrimAddr = channels_find_by_name("thresh_atrim");
        timeAtrimAddr = channels_find_by_name("time_atrim");
        rateAtrimAddr = channels_find_by_name("rate_atrim");

    }

    i_point = GETREADINDEX(point_index);

    /********* PSS data *************/
    SET_VALUE(azrawPss1Addr, PointingData[i_point].pss1_azraw * DEG2I);
    SET_VALUE(azrawPss2Addr, PointingData[i_point].pss2_azraw * DEG2I);
    SET_VALUE(elrawPss1Addr, PointingData[i_point].pss1_elraw * DEG2I);
    SET_VALUE(elrawPss2Addr, PointingData[i_point].pss2_elraw * DEG2I);
    SET_VALUE(snrPss1Addr, PointingData[i_point].pss1_snr * 1000.);
    SET_VALUE(snrPss2Addr, PointingData[i_point].pss2_snr * 1000.);
    SET_VALUE(azPssAddr, (PointingData[i_point].pss_az + CommandData.pss_az_trim) * DEG2I);
    SET_VALUE(PssOkAddr, PointingData[i_point].pss_ok);
    /********** SIP GPS Data **********/
    SET_VALUE(latSipAddr, (int) (SIPData.GPSpos.lat * DEG2I));
    SET_VALUE(lonSipAddr, (int) (SIPData.GPSpos.lon * DEG2I));
    SET_VALUE(altSipAddr, (int) (SIPData.GPSpos.alt));
    SET_VALUE(timeSipAddr, SIPData.GPStime.UTC);

    /********** SIP MKS Altitude ************/
    SET_VALUE(mksLoSipAddr, (int) (SIPData.MKSalt.lo));
    SET_VALUE(mksMedSipAddr, (int) (SIPData.MKSalt.med));
    SET_VALUE(mksHiSipAddr, (int) (SIPData.MKSalt.hi));

    /************* processed pointing data *************/
    SET_VALUE(raAddr, (unsigned int) (PointingData[i_point].ra * H2LI));
    SET_VALUE(decAddr, (unsigned int) (PointingData[i_point].dec * DEG2LI));

    SET_VALUE(OffsetIFelGYAddr, (signed int) (PointingData[i_point].offset_ifel_gy * 32768.));
    SET_VALUE(OffsetIFelGYiscAddr, (signed int) (PointingData[i_point].offset_ifel_gy_xsc[0] * 32768.));
    SET_VALUE(OffsetIFrollGYiscAddr, (signed int) (PointingData[i_point].offset_ifroll_gy_xsc[0] * 32768.));
    SET_VALUE(OffsetIFyawGYiscAddr, (signed int) (PointingData[i_point].offset_ifyaw_gy_xsc[0] * 32768.));
    SET_VALUE(OffsetIFelGYoscAddr, (signed int) (PointingData[i_point].offset_ifel_gy_xsc[1] * 32768.));
    SET_VALUE(OffsetIFrollGYoscAddr, (signed int) (PointingData[i_point].offset_ifroll_gy_xsc[1] * 32768.));
    SET_VALUE(OffsetIFyawGYoscAddr, (signed int) (PointingData[i_point].offset_ifyaw_gy_xsc[1] * 32768.));
    SET_VALUE(OffsetIFrollGYAddr, (signed int) (PointingData[i_point].offset_ifroll_gy * 32768.));
    SET_VALUE(OffsetIFyawGYAddr, (signed int) (PointingData[i_point].offset_ifyaw_gy * 32768.));

    SET_VALUE(OffsetIFrollMagGYAddr, (signed int) (PointingData[i_point].offset_ifrollmag_gy * 32768.));
    SET_VALUE(OffsetIFyawMagGYAddr, (signed int) (PointingData[i_point].offset_ifyawmag_gy * 32768.));
    SET_VALUE(OffsetIFrollPSSGYAddr, (signed int) (PointingData[i_point].offset_ifrollpss_gy * 32768.));
    SET_VALUE(OffsetIFyawPSSGYAddr, (signed int) (PointingData[i_point].offset_ifyawpss_gy * 32768.));

    SET_VALUE(latAddr, (unsigned int) (PointingData[i_point].lat * DEG2LI));
    SET_VALUE(lonAddr, (unsigned int) (PointingData[i_point].lon * DEG2LI));
    SET_VALUE(altAddr, (unsigned int) (PointingData[i_point].alt));

//    SET_VALUE(mcpFrameAddr, PointingData[i_point].mcp_frame);
    SET_VALUE(lstAddr, PointingData[i_point].lst);

    SET_VALUE(azMagAddr, (unsigned int) ((PointingData[i_point].mag_az + CommandData.mag_az_trim) * DEG2I));
    SET_VALUE(azRawMagAddr, (unsigned int) ((PointingData[i_point].mag_az_raw) * DEG2I));
    SET_VALUE(declinationMagAddr, (unsigned int) (PointingData[i_point].mag_model_dec * DEG2I));

    SET_VALUE(elMagAddr, (unsigned int) (PointingData[i_point].mag_el * DEG2I));
    SET_VALUE(dipMagAddr, (unsigned int) (PointingData[i_point].mag_model_dip * DEG2I));

    SET_VALUE(calXMaxMagAddr, (unsigned int) (CommandData.cal_xmax_mag));
    SET_VALUE(calXMinMagAddr, (unsigned int) (CommandData.cal_xmin_mag));
    SET_VALUE(calYMaxMagAddr, (unsigned int) (CommandData.cal_ymax_mag));
    SET_VALUE(calYMinMagAddr, (unsigned int) (CommandData.cal_ymin_mag));

    SET_VALUE(calOffPss1Addr, (unsigned int) (CommandData.cal_off_pss1 * 65536.0 / 40.0));
    SET_VALUE(calOffPss2Addr, (unsigned int) (CommandData.cal_off_pss2 * 65536.0 / 40.0));
    SET_VALUE(calOffPss3Addr, (unsigned int) (CommandData.cal_off_pss3 * 65536.0 / 40.0));
    SET_VALUE(calOffPss4Addr, (unsigned int) (CommandData.cal_off_pss4 * 65536.0 / 40.0));
    SET_VALUE(calDPss1Addr, (unsigned int) (CommandData.cal_d_pss1 * 65536.0 / 4.0));
    SET_VALUE(calDPss2Addr, (unsigned int) (CommandData.cal_d_pss2 * 65536.0 / 4.0));
    SET_VALUE(calDPss3Addr, (unsigned int) (CommandData.cal_d_pss3 * 65536.0 / 4.0));
    SET_VALUE(calDPss4Addr, (unsigned int) (CommandData.cal_d_pss4 * 65536.0 / 4.0));
    SET_VALUE(calIMinPssAddr, (unsigned int) (CommandData.cal_imin_pss * 65536.0 / 40.0));

    SET_VALUE(sigmaMagAddr, (unsigned int) (PointingData[i_point].mag_sigma * DEG2I));
    SET_VALUE(trimMagAddr, CommandData.mag_az_trim * DEG2I);

    SET_VALUE(sigmaPssAddr, (unsigned int) (PointingData[i_point].pss_sigma * DEG2I));
    SET_VALUE(trimPssAddr, CommandData.pss_az_trim * DEG2I);

    SET_VALUE(azSunAddr, (unsigned int) (PointingData[i_point].sun_az * DEG2I));
    SET_VALUE(elSunAddr, (int) (PointingData[i_point].sun_el * DEG2I));

    SET_VALUE(modeCalAddr, CommandData.Cryo.calibrator);
    SET_VALUE(hwprCalAddr, CommandData.Cryo.calib_hwpr);
    SET_VALUE(periodCalAddr, CommandData.Cryo.calib_period);

    SET_VALUE(azIscAddr, (unsigned int) (PointingData[i_point].xsc_az[0] * DEG2I));
    SET_VALUE(elIscAddr, (unsigned int) (PointingData[i_point].xsc_el[0] * DEG2I));
    SET_VALUE(sigmaIscAddr, (unsigned int) (PointingData[i_point].xsc_sigma[0] * DEG2I));

    SET_VALUE(azOscAddr, (unsigned int) (PointingData[i_point].xsc_az[1] * DEG2I));
    SET_VALUE(elOscAddr, (unsigned int) (PointingData[i_point].xsc_el[1] * DEG2I));
    SET_VALUE(sigmaOscAddr, (unsigned int) (PointingData[i_point].xsc_sigma[1] * DEG2I));

    SET_VALUE(trimEncAddr, CommandData.enc_el_trim * DEG2I);

    SET_VALUE(elClinAddr, (unsigned int) ((PointingData[i_point].clin_el_lut + CommandData.clin_el_trim) * DEG2I));
    SET_VALUE(elLutClinAddr, (unsigned int) (PointingData[i_point].clin_el * DEG2I));
    SET_VALUE(sigmaClinAddr, (unsigned int) (PointingData[i_point].clin_sigma * DEG2I));
    SET_VALUE(trimClinAddr, CommandData.clin_el_trim * DEG2I);

    SET_VALUE(trimNullAddr, CommandData.null_az_trim * DEG2I);

    SET_VALUE(threshAtrimAddr, CommandData.autotrim_thresh * 65536.0 / 10.0);
    SET_VALUE(timeAtrimAddr, CommandData.autotrim_time);
    SET_VALUE(rateAtrimAddr, CommandData.autotrim_rate * 65536.0 / 30.0);

    SET_FLOAT(azGyAddr, (float) (PointingData[i_point].v_az));

    /************* Pointing mode fields *************/
    SET_VALUE(slewVetoAddr, (int) (CommandData.pointing_mode.nw) / 4.);
    SET_VALUE(svetoLenAddr, (int) (CommandData.slew_veto) / 4.);
    SET_VALUE(nextIHwprPAddr, (int) (CommandData.pointing_mode.next_i_hwpr));
    SET_VALUE(nextIDithPAddr, (int) (CommandData.pointing_mode.next_i_dith));
    SET_VALUE(nDithPAddr, (int) (CommandData.pointing_mode.n_dith));
    SET_VALUE(modePAddr, (int) (CommandData.pointing_mode.mode));
    if ((CommandData.pointing_mode.mode == P_AZEL_GOTO) || (CommandData.pointing_mode.mode == P_AZ_SCAN)
            || (CommandData.pointing_mode.mode == P_EL_SCAN))
        SET_VALUE(xPAddr, (int) (CommandData.pointing_mode.X * DEG2I));
    else
        SET_VALUE(xPAddr, (int) (CommandData.pointing_mode.X * H2I));

    SET_VALUE(yPAddr, (int) (CommandData.pointing_mode.Y * DEG2I));
    SET_VALUE(velAzPAddr, (int) (CommandData.pointing_mode.vaz * VEL2I));
    SET_VALUE(velElPAddr, (int) (CommandData.pointing_mode.vel * VEL2I));
    SET_VALUE(delPAddr, (int) (CommandData.pointing_mode.del * VEL2I));
    SET_VALUE(dazPAddr, (int) (CommandData.pointing_mode.daz * VEL2I));
    SET_VALUE(wPAddr, (int) (CommandData.pointing_mode.w * DEG2I));
    SET_VALUE(hPAddr, (int) (CommandData.pointing_mode.h * DEG2I));
    SET_VALUE(ra1PAddr, (int) (CommandData.pointing_mode.ra[0] * H2I));
    SET_VALUE(dec1PAddr, (int) (CommandData.pointing_mode.dec[0] * DEG2I));
    SET_VALUE(ra2PAddr, (int) (CommandData.pointing_mode.ra[1] * H2I));
    SET_VALUE(dec2PAddr, (int) (CommandData.pointing_mode.dec[1] * DEG2I));
    SET_VALUE(ra3PAddr, (int) (CommandData.pointing_mode.ra[2] * H2I));
    SET_VALUE(dec3PAddr, (int) (CommandData.pointing_mode.dec[2] * DEG2I));
    SET_VALUE(ra4PAddr, (int) (CommandData.pointing_mode.ra[3] * H2I));
    SET_VALUE(dec4PAddr, (int) (CommandData.pointing_mode.dec[3] * DEG2I));

    sensor_veto = ((!CommandData.use_elmotenc)) | ((!CommandData.use_xsc0) << 1) | ((!CommandData.use_elenc) << 2) | ((!CommandData.use_mag) << 3)
            | ((!CommandData.use_elclin) << 5) | ((!CommandData.use_xsc1) << 6) | ((CommandData.disable_el) << 10)
            | ((CommandData.disable_az) << 11) | ((CommandData.force_el) << 12) | ((!CommandData.use_pss) << 13);

    if (PointingData[i_point].t >= CommandData.pointing_mode.t)
        sensor_veto |= (1 << 7);

    sensor_veto |= (CommandData.az_autogyro << 8);
    sensor_veto |= (CommandData.el_autogyro << 9);

    SET_VALUE(vetoSensorAddr, sensor_veto);


}
