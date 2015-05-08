/**
 * @file time_lst.h
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

#ifndef BLAST_LST_H_
#define BLAST_LST_H_

#include "time_julian.h"

int unix_to_julian_date(time_t m_unixtime, struct julian_date *m_jd);
void time_UT1_UTC(int m_year, int m_month, int m_day, double *m_delta_utc);
void time_TT_UTC (int m_year, int m_month, double *m_delta_utc);
int time_est_deltat(int m_year, int m_month, double *m_deltat);
double time_lst_unix(time_t m_unixtime, double m_longitude);
double mean_sidereal_time(struct julian_date *m_ut1, struct julian_date *m_tdb, double m_longitude);

#endif /* BLAST_LST_H_ */
