/*
 * motordefs.h: Contains any functions or structures used by both
 *              copleycommand.c and pivotcommand.c
 *
 */

#ifndef MOTORDEFS_H
#define MOTORDEFS_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */

enum CheckType {resp, comm, both};

enum MotorType {pivot, rw, elev};

#endif
