/**
 * @file angles.h
 *
 * @date Aug 5, 2015
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

#ifndef INCLUDE_ANGLES_H
#define INCLUDE_ANGLES_H

#include <time.h>
#include <stdbool.h>

double angular_distance(double ra0, double dec0, double ra1, double dec1);
double approximate_az_from_cross_el(double cross_el, double el);
double wrap_to(double angle, double max);
int wrap_to_ints(int angle, int max);
bool limit_value_to(double* value, double min, double max);
bool limit_value_to_ints(int* value, int min, int max);
double unwind_around(double reference, double angle);
void equatorial_to_horizontal(double ra_hours, double dec_deg, time_t lst_s, double lat_deg,
                              double* az_deg, double* el_deg);
void horizontal_to_equatorial(double az_deg, double el_deg, time_t lst_s, double lat_deg,
                              double* ra_hours, double* dec_deg);

double normalize_angle_360(double m_angle);
double normalize_angle_180(double m_angle);

#endif

