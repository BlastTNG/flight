/**
 * @file time_julian.h
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

#ifndef EBEX_JULIAN_H_
#define EBEX_JULIAN_H_
#include <time.h>

/**
 * This structure is used wherever a Julian Day Number (JDN) is expected or
 * returned. Two doubles are used to preserve precision.
 */
struct julian_date {
	double epoch;
	double mjd;
};

#define TAI_TT_OFFSET	32.184
#define J2000_EPOCH 	2451545.0
#define MJD_EPOCH 		2400000.5

#define JULIAN_CENTURY_LENGTH		36525.0
#define JULIAN_MILLENNIUM_LENGTH	365250.0

#define JULIAN_CENTURIES(d1, d2)	\
	(((d1 - J2000_EPOCH) + d2) / JULIAN_CENTURY_LENGTH)

#define JULIAN_MILLENNIA(d1, d2)	\
	(((d1 - J2000_EPOCH) + d2) / JULIAN_MILLENNIUM_LENGTH)

int calendar_to_julian_date(int year, int month, int day,struct julian_date *jd);
int julian_to_calendar_date(struct julian_date *m_jd, struct tm *m_time);

#endif /* EBEX_JULIAN_H_ */
