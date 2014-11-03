/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2004 University of Toronto
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

#ifndef TYPES_DEFINED
#include "ephem_types.h"
#endif
#include <time.h>

double HourToRad(int h, int m, float s);
double DegToRad(int d, int m, float s);
double GetJulian(struct tm *now);
void PrintAngle(double angle);
void StarPos(double t, double ra0, double dec0, double mra, double mdec,
		            double pi, double rvel, double *ra, double *dec);
void PlanetPos(double tt, int target, double *ra, double *dec);
void SunPos(double tt, double *ra, double *dec);
void ReductionInit(const char* ephemFile);
