/**************************************************************
 * sss source code
 *
 * Copyright 2005 (C) Matthew Truch
 *
 * Released under the GPL
 *
 ***************************************************************/

#ifndef SSS_H
#define SSS_H

#include "sss_struct.h"

//#define NO_NET
#define NUM_CHANS    32
#define N_FAST_CHAN  12
#define N_OUTER_LOOP 4
#define N_INNER_LOOP 24
#define N_SLOW_CHAN  8
#define N_OUTER_LOOP_SLOW 2
#define N_INNER_LOOP_SLOW 8

#define MAX_ITERATIONS 10000

#define C2V_M (1.0 / 3245.93)
// Raw counts (gain divided out) to volts slope
#define C2V_B (-4.668 / 3245.93)
// Raw counts to volts intercept
#define V2K 100  // Volts to Kelvin slope

#define SLOW_G (N_OUTER_LOOP_SLOW * N_INNER_LOOP_SLOW)
#define FAST_G (N_OUTER_LOOP * N_INNER_LOOP)

#endif
