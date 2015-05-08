#ifndef _utilities_pointing_H
#define _utilities_pointing_H

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <conversions.h>

/* Public functions declarations */
void calc_sun_position (time_t m_utc, double *m_ra, double *m_dec);

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
