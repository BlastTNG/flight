/**
 * @file ebex_nutation.h
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

#ifndef EBEX_NUTATION_H_
#define EBEX_NUTATION_H_

#include "time_julian.h"

typedef enum
{
	ARG_LONGITUDE_MERCURY = 0,	/// Heliocentric planetary ecliptic longitudes
	ARG_LONGITUDE_VENUS,
	ARG_LONGITUDE_EARTH,
	ARG_LONGITUDE_MARS,
	ARG_LONGITUDE_JUPITER,
	ARG_LONGITUDE_SATURN,
	ARG_LONGITUDE_URANUS,
	ARG_LONGITUDE_NEPTUNE,
	ARG_PRECESSION,			///!< General precession
	ARG_ANOMALY_MOON,		///!< Mean anomaly of the moon (\ell)
	ARG_ANOMALY_SUN,		///!< Mean anomaly of the sun (\ell^\prime)
	ARG_LATITUDE_MOON,		///!< Mean latitude of the moon (F)
	ARG_ELONGATION_MOON,	///!< Mean elongation of the moon relative to the sun (D)
	ARG_LONGITUDE_NODE,		///!< Mean longitude of the moon ascension (\Omega)
	ARG_LONGITUDE_MOON		///!< Mean longitude of the moon (\omega)
} ebex_fund_arguments_t;

double iau2000a_fundamental_arguments(ebex_fund_arguments_t m_arg, double m_centuries);
double iau2000a_mean_obliquity(struct julian_date *m_tdb);
void iau2000a_nutation(struct julian_date *tdb, double *d_psi, double *d_epsilon);

#endif /* EBEX_NUTATION_H_ */
