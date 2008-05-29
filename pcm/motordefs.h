/*
 * motordefs.h: Contains any functions or structures used by both
 *              pivotcommand.c and reactcommand.c
 *
 */

#ifndef MOTORDEFS_H
#define MOTORDEFS_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */

enum CheckType {resp, comm, both};


// Limits and scaling ranges for control loop gains.
#define SPR1_LIM 10000.0
#define SPR2_LIM 10000.0
#define SPP1_LIM 200.0
#define SPP2_LIM 2.0
#define SCR1_LIM 10000.0
#define SCR2_LIM 10000.0
#define SCP1_LIM 200.0
#define SCP2_LIM 2.0
// Place holder for more defines.
#endif
