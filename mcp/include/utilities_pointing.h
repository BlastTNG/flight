/**
 * @file utilities_pointing.h
 *
 * @date Nov 23, 2015
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_UTILITIES_POINTING_H
#define INCLUDE_UTILITIES_POINTING_H

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <conversions.h>

/* Public functions declarations */
void calc_sun_position(time_t m_utc, double *m_ra, double *m_dec);

/*
 * Returns an angle that is between 0 and limit.
 *
 * angle -- The angle to be reduced.
 * limit -- The maximum value for the angle.
 *
 * Return: An angle that is between 0 and limit.
 */
static inline double range_fast(double *m_val, double m_limit)
{
	double val = *m_val;
	while (val < 0.0)
		val += m_limit;
	while (val >= m_limit)
		val -= m_limit;

	*m_val = val;
	return val;
}

#endif
