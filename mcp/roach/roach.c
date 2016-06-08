/*
 * roach.c
 *
 * This software is copyright (C) 2013-2016 University of Pennsylvania
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
 *  Created on: Apr 5, 2015
 *      Author: seth
 */

#include "roach.h"
#include <katcp.h>

static int roach_fft_shift = 255;
static double dac_samp_freq = 512.0e6;
static double fpga_samp_freq = 256.0e6;
static int dds_shift = 304; // This varies b/t fpg/bof files
static int f_base = 300;
static int fft_len = 1024;

static ph_thread_t *katcp_thread = NULL;

int init_roach(void) {

    ph_thread_t *xy_thread = ph_thread_spawn(StageBus, NULL);
}
