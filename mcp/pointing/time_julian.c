/**
 * @file ebex_julian.c
 *
 * @date Aug 5, 2011
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <time_julian.h>

/**
 * Converts a calendar date (standard gregorian) to the equivalent Julian Date
 * @param m_year Astronomical year (must be later than 4800BC)
 * @param m_month Month (1-12)
 * @param m_day Day of month (1-31)
 * @param m_jd Pointer to the returned Julian Date
 * @return 0 on success, -1 on failure
 */
int calendar_to_julian_date(int m_year, int m_month, int m_day, struct julian_date *m_jd)
{
	int temp_m;

	if (m_day < 1 || m_day > 31 || m_month < 1 || m_month > 12 || m_year < -4799)
		return -1;

	temp_m = (m_month - 14) / 12;

	m_jd->epoch = MJD_EPOCH;
	m_jd->mjd = (double) ((long)m_day + (1461L * (long)(m_year + 4800L + temp_m)) / 4L
					+ (367L * (long)(m_month - 2 - temp_m * 12)) / 12L
					- (3L * (long)((m_year + 4900L + temp_m) / 100L)) / 4L
					- 2432076L);

	return 0;
}

/**
 * Converts a Julian Day to the equivalent calendar time
 * @param m_jd Pointer to the Julian Day struct
 * @param m_time Pointer to the Gregorian calendar time struct
 * @return 0 on success, -1 on failure
 */
int julian_to_calendar_date(struct julian_date *m_jd, struct tm *m_time)
{
	double j;
	int a, arem;
	int c, crem;
	int g, grem;
	int y, m, d;
	div_t b, mrem;

	j = m_jd->epoch + m_jd->mjd;

	if (j < -32044.5)
		return -1;

	j += 32044.5;
	g = j / 146097.0;
	grem = fmod(j, 146097.0);

	c = ((grem / 36524 + 1) * 3) / 4;
	crem = grem - c * 36524;
	b = div(crem, 1461);

	a = ((b.rem / 365 + 1) * 3) / 4;
	arem = b.rem - a * 365;

	y = g * 400 + c * 100 + b.quot * 4 + a;
	m = (arem * 5 + 308) / 153 - 2;
	d = arem - ((m + 4) * 153) / 5 + 122;

	mrem = div(m + 2, 12);

	m_time->tm_year = y - 4800 + mrem.quot;
	m_time->tm_mon = mrem.rem;
	m_time->tm_mday = d + 1;

	j = fmod(m_jd->epoch + m_jd->mjd + 0.5, 1.0);
	m_time->tm_hour = 24.0 * j;
	m_time->tm_min = 60.0 * (24.0 * j - (double)m_time->tm_hour);
	m_time->tm_sec = 60.0 * (60.0 * (24.0 * j - (double)m_time->tm_hour) - (double)m_time->tm_min);

	return 0;
}
