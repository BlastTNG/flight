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
#include <motors.h>
#include <mcp.h>
#include <pointing_struct.h>
#include <dsp1760.h>

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
        { { 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00, 0.000000e+00 }, { 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00 } },

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
    static channel_t* mask_gy_addr;

    static int firsttime = 1;
    if (firsttime) {
      firsttime = 0;
      ifElgy1Addr = channels_find_by_name("ifel_gy");
      ifRollgy1Addr = channels_find_by_name("ifroll_gy");
      ifYawgy1Addr = channels_find_by_name("ifyaw_gy");
      ifElgy1Addr = channels_find_by_name("ifel_1_gy");
      ifRollgy1Addr = channels_find_by_name("ifroll_1_gy");
      ifYawgy1Addr = channels_find_by_name("ifyaw_1_gy");
      ifElgy2Addr = channels_find_by_name("ifel_2_gy");
      ifRollgy2Addr = channels_find_by_name("ifroll_2_gy");
      ifYawgy2Addr = channels_find_by_name("ifyaw_2_gy");

      mask_gy_addr = channels_find_by_name("mask_gy");
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
    SET_FLOAT(ifElgy2Addr, dsp1760_getval(1,0));
    SET_FLOAT(ifRollgy2Addr, dsp1760_getval(1,1));
    SET_FLOAT(ifYawgy2Addr, dsp1760_getval(1,2));

    gy_ifroll= gy_inv[gymask][0][0]*ifroll_gy1 + gy_inv[gymask][0][1]*ifroll_gy2 + gy_inv[gymask][0][2]*ifyaw_gy1 + gy_inv[gymask][0][3]*ifyaw_gy2 + gy_inv[gymask][0][4]*ifel_gy1 + gy_inv[gymask][0][5]*ifel_gy2;
    gy_ifyaw = gy_inv[gymask][1][0]*ifroll_gy1 + gy_inv[gymask][1][1]*ifroll_gy2 + gy_inv[gymask][1][2]*ifyaw_gy1 + gy_inv[gymask][1][3]*ifyaw_gy2 + gy_inv[gymask][1][4]*ifel_gy1 + gy_inv[gymask][1][5]*ifel_gy2;
    gy_ifel  = gy_inv[gymask][2][0]*ifroll_gy1 + gy_inv[gymask][2][1]*ifroll_gy2 + gy_inv[gymask][2][2]*ifyaw_gy1 + gy_inv[gymask][2][3]*ifyaw_gy2 + gy_inv[gymask][2][4]*ifel_gy1 + gy_inv[gymask][2][5]*ifel_gy2;
    SET_FLOAT(ifel_gy_addr, gy_ifel);
    SET_FLOAT(ifroll_gy_addr, gy_ifroll);
    SET_FLOAT(ifyaw_gy_addr, gy_ifyaw);
    ACSData.ifel_gy = gy_ifel;
    ACSData.ifroll_gy = gy_ifroll;
    ACSData.ifyaw_gy = gy_ifyaw;

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

    static channel_t *vel_rw_addr;
    static channel_t *encvel_rw_addr;
    static channel_t *pos_rw_addr;
    static channel_t *vel_el_addr;
    static channel_t *encvel_el_addr;
    static channel_t *pos_el_addr;
    static channel_t *vel_piv_addr;
    static channel_t *encvel_piv_addr;
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

        vel_rw_addr = channels_find_by_name("mc_rw_vel");
        encvel_rw_addr = channels_find_by_name("mc_rw_encvel");
        pos_rw_addr = channels_find_by_name("mc_rw_pos");

        vel_el_addr = channels_find_by_name("mc_el_vel");
        encvel_el_addr = channels_find_by_name("mc_el_encvel");
        pos_el_addr = channels_find_by_name("mc_el_pos");

        vel_piv_addr = channels_find_by_name("mc_piv_vel");
        encvel_piv_addr = channels_find_by_name("mc_piv_encvel");
        pos_piv_addr = channels_find_by_name("mc_piv_pos");

    }
    i_point = GETREADINDEX(point_index);
    i_motors = GETREADINDEX(motor_index);

    SET_INT32(azAddr, (unsigned int) (PointingData[i_point].az * DEG2LI));
    SET_INT32(elAddr, (unsigned int) (PointingData[i_point].el * DEG2LI));

    SET_INT16(elEncAddr, (unsigned int) ((PointingData[i_point].enc_el + CommandData.enc_el_trim) * DEG2I));
    SET_INT16(sigmaEncAddr, (unsigned int) (PointingData[i_point].enc_sigma * DEG2I));

    SET_INT32(vel_rw_addr, RWMotorData[i_motors].velocity);
    SET_INT32(encvel_rw_addr, RWMotorData[i_motors].enc_velocity);
    SET_INT32(pos_rw_addr, RWMotorData[i_motors].position);
    SET_INT32(vel_el_addr, ElevMotorData[i_motors].velocity);
    SET_INT32(encvel_el_addr, ElevMotorData[i_motors].enc_velocity);
    SET_INT32(pos_el_addr, ElevMotorData[i_motors].position);
    SET_INT32(vel_piv_addr, PivotMotorData[i_motors].velocity);
    SET_INT32(encvel_piv_addr, PivotMotorData[i_motors].enc_velocity);
    SET_INT32(pos_piv_addr, PivotMotorData[i_motors].position);
}

//static channel_t* GetSCNiosAddr(char* field, int which)
//{
//  char buffer[FIELD_LEN];
//  sprintf(buffer, "%s_%s", field, which ? "osc" : "isc");
//
//  return channels_find_by_name(buffer);
//}

//TODO: Update StoreStarCameraData for XSC
static void StoreStarCameraData(int index, int which)
{
//    static int firsttime[2] = { 1, 1 };
//    static int blob_index[2] = { 0, 0 };
//    static int blob_data[2][15][4];
//
//    int i, i_isc = 0;
//
//    /** isc fields **/
//    static channel_t* Blob0XAddr[2];
//    static channel_t* Blob1XAddr[2];
//    static channel_t* Blob2XAddr[2];
//    static channel_t* Blob0YAddr[2];
//    static channel_t* Blob1YAddr[2];
//    static channel_t* Blob2YAddr[2];
//    static channel_t* Blob0FAddr[2];
//    static channel_t* Blob1FAddr[2];
//    static channel_t* Blob2FAddr[2];
//    static channel_t* Blob0SAddr[2];
//    static channel_t* Blob1SAddr[2];
//    static channel_t* Blob2SAddr[2];
//    static channel_t* ErrorAddr[2];
//    static channel_t* MapmeanAddr[2];
//    static channel_t* FramenumAddr[2];
//    static channel_t* RdSigmaAddr[2];
//    static channel_t* RaAddr[2];
//    static channel_t* DecAddr[2];
//    static channel_t* HxFlagAddr[2];
//    static channel_t* McpnumAddr[2];
//    static channel_t* ApertAddr[2];
//    static channel_t* MdistAddr[2];
//    static channel_t* NblobsAddr[2];
//    static channel_t* FocusAddr[2];
//    static channel_t* FocOffAddr[2];
//    static channel_t* ThreshAddr[2];
//    static channel_t* GridAddr[2];
//    static channel_t* StateAddr[2];
//    static channel_t* MinblobsAddr[2];
//    static channel_t* MaxblobsAddr[2];
//    static channel_t* MaglimitAddr[2];
//    static channel_t* NradAddr[2];
//    static channel_t* LradAddr[2];
//    static channel_t* TolAddr[2];
//    static channel_t* MtolAddr[2];
//    static channel_t* QtolAddr[2];
//    static channel_t* RtolAddr[2];
//    static channel_t* FpulseAddr[2];
//    static channel_t* SpulseAddr[2];
//    static channel_t* XOffAddr[2];
//    static channel_t* YOffAddr[2];
//    static channel_t* IHoldAddr[2];
//    static channel_t* SavePrdAddr[2];
//    static channel_t* Temp1Addr[2];
//    static channel_t* Temp2Addr[2];
//    static channel_t* Temp3Addr[2];
//    static channel_t* Temp4Addr[2];
//    static channel_t* PressureAddr[2];
//    static channel_t* GainAddr[2];
//    static channel_t* OffsetAddr[2];
//    static channel_t* ExposureAddr[2];
//    static channel_t* TrigTypeAddr[2];
//    static channel_t* RealTrigAddr[2];
//    static channel_t* BlobIdxAddr[2];
//    static channel_t* FieldrotAddr[2];
//    static channel_t* DiskfreeAddr[2];
//    static channel_t* MaxslewAddr[2];
//    static channel_t* MaxAgeAddr[2];
//    static channel_t* AgeAddr[2];
//    static channel_t* PosFocusAddr[2];
//
//    if (firsttime[which]) {
//        firsttime[which] = 0;
//        Blob0XAddr[which] = GetSCNiosAddr("blob00_x", which);
//        Blob1XAddr[which] = GetSCNiosAddr("blob01_x", which);
//        Blob2XAddr[which] = GetSCNiosAddr("blob02_x", which);
//        Blob0YAddr[which] = GetSCNiosAddr("blob00_y", which);
//        Blob1YAddr[which] = GetSCNiosAddr("blob01_y", which);
//        Blob2YAddr[which] = GetSCNiosAddr("blob02_y", which);
//        Blob0FAddr[which] = GetSCNiosAddr("blob00_f", which);
//        Blob1FAddr[which] = GetSCNiosAddr("blob01_f", which);
//        Blob2FAddr[which] = GetSCNiosAddr("blob02_f", which);
//        Blob0SAddr[which] = GetSCNiosAddr("blob00_s", which);
//        Blob1SAddr[which] = GetSCNiosAddr("blob01_s", which);
//        Blob2SAddr[which] = GetSCNiosAddr("blob02_s", which);
//        ErrorAddr[which] = GetSCNiosAddr("error", which);
//        MapmeanAddr[which] = GetSCNiosAddr("mapmean", which);
//        RdSigmaAddr[which] = GetSCNiosAddr("rd_sigma", which);
//        FramenumAddr[which] = GetSCNiosAddr("framenum", which);
//        RaAddr[which] = GetSCNiosAddr("ra", which);
//        DecAddr[which] = GetSCNiosAddr("dec", which);
//        NblobsAddr[which] = GetSCNiosAddr("nblobs", which);
//        HxFlagAddr[which] = GetSCNiosAddr("hx_flag", which);
//        McpnumAddr[which] = GetSCNiosAddr("mcpnum", which);
//
//        StateAddr[which] = GetSCNiosAddr("state", which);
//        FocusAddr[which] = GetSCNiosAddr("focus", which);
//        FocOffAddr[which] = GetSCNiosAddr("foc_off", which);
//        ApertAddr[which] = GetSCNiosAddr("apert", which);
//        ThreshAddr[which] = GetSCNiosAddr("thresh", which);
//        GridAddr[which] = GetSCNiosAddr("grid", which);
//        MdistAddr[which] = GetSCNiosAddr("mdist", which);
//        MinblobsAddr[which] = GetSCNiosAddr("minblobs", which);
//        MaxblobsAddr[which] = GetSCNiosAddr("maxblobs", which);
//        MaglimitAddr[which] = GetSCNiosAddr("maglimit", which);
//        NradAddr[which] = GetSCNiosAddr("nrad", which);
//        LradAddr[which] = GetSCNiosAddr("lrad", which);
//        TolAddr[which] = GetSCNiosAddr("tol", which);
//        MtolAddr[which] = GetSCNiosAddr("mtol", which);
//        QtolAddr[which] = GetSCNiosAddr("qtol", which);
//        RtolAddr[which] = GetSCNiosAddr("rtol", which);
//        FpulseAddr[which] = GetSCNiosAddr("fpulse", which);
//        SpulseAddr[which] = GetSCNiosAddr("spulse", which);
//        XOffAddr[which] = GetSCNiosAddr("x_off", which);
//        YOffAddr[which] = GetSCNiosAddr("y_off", which);
//        IHoldAddr[which] = GetSCNiosAddr("i_hold", which);
//        SavePrdAddr[which] = GetSCNiosAddr("save_prd", which);
//        PressureAddr[which] = GetSCNiosAddr("pressure1", which);
//        GainAddr[which] = GetSCNiosAddr("gain", which);
//        OffsetAddr[which] = GetSCNiosAddr("offset", which);
//        ExposureAddr[which] = GetSCNiosAddr("exposure", which);
//        TrigTypeAddr[which] = GetSCNiosAddr("trig_type", which);
//        FieldrotAddr[which] = GetSCNiosAddr("fieldrot", which);
//        RealTrigAddr[which] = GetSCNiosAddr("real_trig", which);
//        BlobIdxAddr[which] = GetSCNiosAddr("blob_idx", which);
//        DiskfreeAddr[which] = GetSCNiosAddr("diskfree", which);
//        MaxslewAddr[which] = GetSCNiosAddr("maxslew", which);
//        MaxAgeAddr[which] = GetSCNiosAddr("max_age", which);
//        AgeAddr[which] = GetSCNiosAddr("age", which);
//        PosFocusAddr[which] = GetSCNiosAddr("pos_focus", which);
//
//        Temp1Addr[0] = channels_find_by_name("t_flange_isc");
//        Temp2Addr[0] = channels_find_by_name("t_heat_isc");
//        Temp3Addr[0] = channels_find_by_name("t_lens_isc");
//        Temp4Addr[0] = channels_find_by_name("t_comp_isc");
//        Temp1Addr[1] = channels_find_by_name("t_flange_osc");
//        Temp2Addr[1] = channels_find_by_name("t_heat_osc");
//        Temp3Addr[1] = channels_find_by_name("t_lens_osc");
//        Temp4Addr[1] = channels_find_by_name("t_comp_osc");
//    }

//    /** Increment isc index -- this only happens once per slow frame */
//    if (index == 0)
//        if (((iscread_index[which] + 1) % 5) != iscwrite_index[which]) {
//            iscread_index[which] = (iscread_index[which] + 1) % 5;
//            /* reset blob multiplexing if this is a pointing packet */
//            if (ISCSolution[which][iscread_index[which]].flag == 1)
//                blob_index[which] = 0;
//        }
//
//    i_isc = iscread_index[which];
//
//    /*** State Info ***/
//    SET_VALUE(StateAddr[which], (unsigned int)
//            (ISCSentState[which].save * 0x0001
//            + ISCSentState[which].pause * 0x0002
//            + ISCSentState[which].abort * 0x0004
//            + ISCSentState[which].autofocus * 0x0008
//            + ISCSentState[which].shutdown * 0x0010 /* 2 bits */
//            + ISCSentState[which].eyeOn * 0x0040
//            + ISCSolution[which][i_isc].heaterOn * 0x0080
//            + ISCSentState[which].useLost * 0x0100
//            + ISCSolution[which][i_isc].autofocusOn * 0x0200));
//    SET_VALUE(FocusAddr[which], (unsigned int) ISCSentState[which].focus_pos);
//    SET_VALUE(FocOffAddr[which], (unsigned int) ISCSentState[which].focusOffset);
//    SET_VALUE(ApertAddr[which], (unsigned int) ISCSentState[which].ap_pos);
//    SET_VALUE(ThreshAddr[which], (unsigned int) (ISCSentState[which].sn_threshold * 10.));
//    SET_VALUE(GridAddr[which], (unsigned int) ISCSentState[which].grid);
//    SET_VALUE(MdistAddr[which], (unsigned int) ISCSentState[which].mult_dist);
//    SET_VALUE(MinblobsAddr[which], (unsigned int) ISCSentState[which].minBlobMatch);
//    SET_VALUE(MaxblobsAddr[which], (unsigned int) ISCSentState[which].maxBlobMatch);
//    SET_VALUE(MaglimitAddr[which], (unsigned int) (ISCSentState[which].mag_limit * 1000.));
//    SET_VALUE(NradAddr[which], (unsigned int) (ISCSentState[which].norm_radius * RAD2I));
//    SET_VALUE(LradAddr[which], (unsigned int) (ISCSentState[which].lost_radius * RAD2I));
//    SET_VALUE(TolAddr[which], (unsigned int) (ISCSentState[which].tolerance * RAD2ARCSEC));
//    SET_VALUE(MtolAddr[which], (unsigned int) (ISCSentState[which].match_tol * 65535.));
//    SET_VALUE(QtolAddr[which], (unsigned int) (ISCSentState[which].quit_tol * 65535.));
//    SET_VALUE(RtolAddr[which], (unsigned int) (ISCSentState[which].rot_tol * RAD2I));
//    SET_VALUE(XOffAddr[which], (unsigned int) (ISCSentState[which].azBDA * RAD2I));
//    SET_VALUE(YOffAddr[which], (unsigned int) (ISCSentState[which].elBDA * RAD2I));
//    SET_VALUE(IHoldAddr[which], (unsigned int) (ISCSentState[which].hold_current));
//    SET_VALUE(Temp1Addr[which], (unsigned int) (ISCSolution[which][i_isc].temp1 * 100.));
//    SET_VALUE(Temp2Addr[which], (unsigned int) (ISCSolution[which][i_isc].temp2 * 100.));
//    SET_VALUE(Temp3Addr[which], (unsigned int) (ISCSolution[which][i_isc].temp3 * 100.));
//    SET_VALUE(Temp4Addr[which], (unsigned int) (ISCSolution[which][i_isc].temp4 * 100.));
//    SET_VALUE(PressureAddr[which], (unsigned int) (ISCSolution[which][i_isc].pressure1 * 2000.));
//    SET_VALUE(GainAddr[which], (unsigned int) (ISCSentState[which].gain * 655.36));
//    SET_VALUE(OffsetAddr[which], ISCSentState[which].offset);
//    SET_VALUE(ExposureAddr[which], ISCSentState[which].exposure / 100);
//    SET_VALUE(TrigTypeAddr[which], ISCSentState[which].triggertype);
//    SET_VALUE(MaxslewAddr[which], (unsigned int) ISCSentState[which].maxSlew / RAD2I);
//
//    SET_VALUE(FpulseAddr[which], (unsigned int) (CommandData.ISCControl[which].fast_pulse_width));
//    SET_VALUE(SpulseAddr[which], (unsigned int) (CommandData.ISCControl[which].pulse_width));
//    SET_VALUE(MaxAgeAddr[which], (unsigned int) (CommandData.ISCControl[which].max_age * 10));
//    SET_VALUE(AgeAddr[which], (unsigned int) (CommandData.ISCControl[which].age * 10));
//    SET_VALUE(SavePrdAddr[which], (unsigned int) (CommandData.ISCControl[which].save_period));
//
//    /* The handshake flag -- for handshakes, we only write this. */
//    SET_VALUE(HxFlagAddr[which], (unsigned int) ISCSolution[which][i_isc].flag);
//
//    /*** Blobs ***/
//    /* Save current blob data if the current frame is a pointing solution;
//     * we only do this once per slow frame */
//    if (index == 0 && ISCSolution[which][i_isc].flag)
//        for (i = 0; i < 15; ++i) {
//            blob_data[which][i][0] = (int) (ISCSolution[which][i_isc].blob_x[i] * 40.);
//            blob_data[which][i][1] = (int) (ISCSolution[which][i_isc].blob_y[i] * 40.);
//            blob_data[which][i][2] = ISCSolution[which][i_isc].blob_flux[i];
//            blob_data[which][i][3] = (int) (ISCSolution[which][i_isc].blob_sn[i] * 65.536);
//        }
//
//    if (index == 0) {
//        /* When we're writing a handshake packet, these blobs are still from the
//         * previous pointing packet */
//        SET_VALUE(Blob0XAddr[which], blob_data[which][blob_index[which] * 3 + 0][0]);
//        SET_VALUE(Blob1XAddr[which], blob_data[which][blob_index[which] * 3 + 1][0]);
//        SET_VALUE(Blob2XAddr[which], blob_data[which][blob_index[which] * 3 + 2][0]);
//
//        SET_VALUE(Blob0YAddr[which], blob_data[which][blob_index[which] * 3 + 0][1]);
//        SET_VALUE(Blob1YAddr[which], blob_data[which][blob_index[which] * 3 + 1][1]);
//        SET_VALUE(Blob2YAddr[which], blob_data[which][blob_index[which] * 3 + 2][1]);
//
//        SET_VALUE(Blob0FAddr[which], blob_data[which][blob_index[which] * 3 + 0][2]);
//        SET_VALUE(Blob1FAddr[which], blob_data[which][blob_index[which] * 3 + 1][2]);
//        SET_VALUE(Blob2FAddr[which], blob_data[which][blob_index[which] * 3 + 2][2]);
//
//        SET_VALUE(Blob0SAddr[which], blob_data[which][blob_index[which] * 3 + 0][3]);
//        SET_VALUE(Blob1SAddr[which], blob_data[which][blob_index[which] * 3 + 1][3]);
//        SET_VALUE(Blob2SAddr[which], blob_data[which][blob_index[which] * 3 + 2][3]);
//
//        SET_VALUE(BlobIdxAddr[which], blob_index[which]);
//
//        /* increment blob index once per slow frame */
//        blob_index[which] = (blob_index[which] + 1) % 5;
//    }
//
//    if (!ISCSolution[which][i_isc].flag)
//        return;
//
//    /* Everything after this happens only for pointing packets */
//
//    /*** Solution Info ***/
//    SET_VALUE(FramenumAddr[which], (unsigned int) ISCSolution[which][i_isc].framenum);
//    SET_VALUE(RaAddr[which], (unsigned int) (ISCSolution[which][i_isc].ra * RAD2LI));
//    SET_VALUE(DecAddr[which], (unsigned int) ((ISCSolution[which][i_isc].dec + M_PI / 2) * 2. * RAD2LI));
//    SET_VALUE(NblobsAddr[which], (unsigned int) ISCSolution[which][i_isc].n_blobs);
//
//    if (ISCSolution[which][i_isc].sigma * RAD2ARCSEC > 65535)
//        SET_VALUE(RdSigmaAddr[which], 65535);
//    else
//        SET_VALUE(RdSigmaAddr[which], (unsigned int) (ISCSolution[which][i_isc].sigma * RAD2ARCSEC));
//
//    SET_VALUE(FieldrotAddr[which], (unsigned int) (ISCSolution[which][i_isc].rot * RAD2I));
//
//    SET_VALUE(McpnumAddr[which], (unsigned int) ISCSolution[which][i_isc].MCPFrameNum);
//    SET_VALUE(RealTrigAddr[which], (unsigned int) ISCSolution[which][i_isc].triggertype);
//    SET_VALUE(ErrorAddr[which], (unsigned int) ISCSolution[which][i_isc].cameraerr);
//    SET_VALUE(MapmeanAddr[which], (unsigned int) ISCSolution[which][i_isc].mapMean);
//    SET_VALUE(DiskfreeAddr[which], (unsigned int) ISCSolution[which][i_isc].diskspace / 5);
//    SET_VALUE(PosFocusAddr[which], ISCSolution[which][i_isc].current_focus_pos);
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
    static channel_t* OffsetIFrollDGPSGYAddr;
    static channel_t* OffsetIFyawDGPSGYAddr;
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
    static channel_t* dgpsAzAddr;
    static channel_t* dgpsSigmaAddr;
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

    /** dgps fields **/
    static channel_t* dgpsTimeAddr;
    static channel_t* dgpsLatAddr;
    static channel_t* dgpsLonAddr;
    static channel_t* dgpsAltAddr;
    static channel_t* dgpsSpeedAddr;
    static channel_t* dgpsDirAddr;
    static channel_t* dgpsClimbAddr;
    static channel_t* dgpsAttOkAddr;
    static channel_t* dgpsAzRawAddr;
    static channel_t* dgpsPitchRawAddr;
    static channel_t* dgpsRollRawAddr;
    static channel_t* dgpsNSatAddr;

    /* trim fields */
    static channel_t *trimClinAddr;
    static channel_t *trimEncAddr;
    static channel_t *trimNullAddr;
    static channel_t *trimMagAddr;
    static channel_t *trimPssAddr;
    static channel_t *dgpsTrimAddr;

    static channel_t *threshAtrimAddr;
    static channel_t *timeAtrimAddr;
    static channel_t *rateAtrimAddr;

    static channel_t *modeCalAddr;
    static channel_t *hwprCalAddr;
    static channel_t *periodCalAddr;
    static channel_t *lstSchedAddr;


    int i_point;
    int i_dgps;
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
        OffsetIFrollDGPSGYAddr = channels_find_by_name("offset_ifrollgps_gy");
        OffsetIFyawDGPSGYAddr = channels_find_by_name("offset_ifyawdgps_gy");
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
        dgpsAzAddr = channels_find_by_name("az_dgps");
        dgpsSigmaAddr = channels_find_by_name("sigma_dgps");
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

        dgpsTimeAddr = channels_find_by_name("time_dgps");
        dgpsLatAddr = channels_find_by_name("lat_dgps");
        dgpsLonAddr = channels_find_by_name("lon_dgps");
        dgpsAltAddr = channels_find_by_name("alt_dgps");
        dgpsSpeedAddr = channels_find_by_name("speed_dgps");
        dgpsDirAddr = channels_find_by_name("dir_dgps");
        dgpsClimbAddr = channels_find_by_name("climb_dgps");
        dgpsNSatAddr = channels_find_by_name("n_sat_dgps");
        dgpsAttOkAddr = channels_find_by_name("att_ok_dgps");
        dgpsAzRawAddr = channels_find_by_name("az_raw_dgps");
        dgpsPitchRawAddr = channels_find_by_name("pitch_raw_dgps");
        dgpsRollRawAddr = channels_find_by_name("roll_raw_dgps");

        lstSchedAddr = channels_find_by_name("lst_sched");

        trimClinAddr = channels_find_by_name("trim_clin");
        trimEncAddr = channels_find_by_name("trim_enc");
        trimNullAddr = channels_find_by_name("trim_null");
        trimMagAddr = channels_find_by_name("trim_mag");
        trimPssAddr = channels_find_by_name("trim_pss");
        dgpsTrimAddr = channels_find_by_name("trim_dgps");

        threshAtrimAddr = channels_find_by_name("thresh_atrim");
        timeAtrimAddr = channels_find_by_name("time_atrim");
        rateAtrimAddr = channels_find_by_name("rate_atrim");

    }

    StoreStarCameraData(0, 0); /* write ISC data */
    StoreStarCameraData(0, 1); /* write OSC data */

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
    SET_VALUE(OffsetIFrollDGPSGYAddr, (signed int) (PointingData[i_point].offset_ifrolldgps_gy * 32768.));
    SET_VALUE(OffsetIFyawDGPSGYAddr, (signed int) (PointingData[i_point].offset_ifyawdgps_gy * 32768.));
    SET_VALUE(OffsetIFrollPSSGYAddr, (signed int) (PointingData[i_point].offset_ifrollpss_gy * 32768.));
    SET_VALUE(OffsetIFyawPSSGYAddr, (signed int) (PointingData[i_point].offset_ifyawpss_gy * 32768.));

    SET_VALUE(latAddr, (unsigned int) (PointingData[i_point].lat * DEG2LI));
    SET_VALUE(lonAddr, (unsigned int) (PointingData[i_point].lon * DEG2LI));
    SET_VALUE(altAddr, (unsigned int) (PointingData[i_point].alt));

//    SET_VALUE(mcpFrameAddr, PointingData[i_point].mcp_frame);
    SET_VALUE(lstAddr, PointingData[i_point].lst);

    SET_VALUE(azMagAddr, (unsigned int) ((PointingData[i_point].mag_az + CommandData.mag_az_trim) * DEG2I));
    SET_VALUE(azRawMagAddr, (unsigned int) ((PointingData[i_point].mag_az_raw) * DEG2I));
//    SET_VALUE(pitchMagAddr, (unsigned int) (PointingData[i_point].mag_el * DEG2I));
    SET_VALUE(declinationMagAddr, (unsigned int) (PointingData[i_point].mag_model_dec * DEG2I));

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

    SET_VALUE(dgpsAzAddr, (unsigned int) ((PointingData[i_point].dgps_az + CommandData.dgps_az_trim) * DEG2I));
    SET_VALUE(dgpsSigmaAddr,
            (((unsigned int) (PointingData[i_point].dgps_sigma * DEG2I)) > 65535) ? 65535 :
                    ((unsigned int) (PointingData[i_point].dgps_sigma * DEG2I)));
    SET_VALUE(dgpsTrimAddr, CommandData.dgps_az_trim * DEG2I);
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

    sensor_veto = /* 1st bit used to be sun */((!CommandData.use_isc) << 1) | ((!CommandData.use_elenc) << 2) | ((!CommandData.use_mag) << 3)
            | ((!CommandData.use_gps) << 4) | ((!CommandData.use_elclin) << 5) | ((!CommandData.use_osc) << 6) | ((CommandData.disable_el) << 10)
            | ((CommandData.disable_az) << 11) | ((CommandData.force_el) << 12) | ((!CommandData.use_pss) << 13);

    if (PointingData[i_point].t >= CommandData.pointing_mode.t)
        sensor_veto |= (1 << 7);

    sensor_veto |= (CommandData.az_autogyro << 8);
    sensor_veto |= (CommandData.el_autogyro << 9);

    SET_VALUE(vetoSensorAddr, sensor_veto);

    /************* dgps fields *************/
    SET_VALUE(dgpsTimeAddr, DGPSTime);

    /** Pos fields **/
    i_dgps = GETREADINDEX(dgpspos_index);
    SET_VALUE(dgpsLatAddr, (int) (DGPSPos[i_dgps].lat * DEG2I));
    SET_VALUE(dgpsLonAddr, (int) (DGPSPos[i_dgps].lon * DEG2I));
    SET_VALUE(dgpsAltAddr, DGPSPos[i_dgps].alt);
    SET_VALUE(dgpsSpeedAddr, DGPSPos[i_dgps].speed * 100);
    SET_VALUE(dgpsDirAddr, (unsigned int) DGPSPos[i_dgps].direction * DEG2I);
    SET_VALUE(dgpsClimbAddr, DGPSPos[i_dgps].climb * 100);
    SET_VALUE(dgpsNSatAddr, DGPSPos[i_dgps].n_sat);

    SET_VALUE(lstSchedAddr, sched_lst);

    /** Att fields **/
    i_dgps = GETREADINDEX(dgpsatt_index);
    SET_VALUE(dgpsAzRawAddr, DGPSAtt[i_dgps].az * DEG2I);
    SET_VALUE(dgpsPitchRawAddr, DGPSAtt[i_dgps].pitch * DEG2I);
    SET_VALUE(dgpsRollRawAddr, DGPSAtt[i_dgps].roll * DEG2I);
    SET_VALUE(dgpsAttOkAddr, DGPSAtt[i_dgps].att_ok);


}
