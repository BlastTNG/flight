/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
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
 * These functions were written by Ed Chapin.
 *
 */

#include <stdio.h>
#include <math.h>
#include "blast.h"
//-----------------------------------------------------------------------------
// Given two points in az+el calculate the azimuth that corresponds to
// a given elevation which falls on the great circle connecting the two
// points (on the smallest segment since there are two solutions on either
// side of the sphere).
//
// All quantities in degrees
//
// Inputs:
//
// az    = 2 element array with az of endpoints in degrees
// el    = "    "      "        el
// el_in = elevation along the great circle
//
// Return:
//
// Azimuth in degrees
//
//-----------------------------------------------------------------------------

static double az_gcirc(double *az, double *el, double el_in)
{
    double azdist;
    double u[2], v[2], v_in;
    double c2_1, c2_2, c1_1, c1_2;
    double u_1a, u_1b, u_2a, u_2b;
    double az_1a, az_1b, az_2a, az_2b;
    int cross_eq;

    // the sign of azdist is used to figure out which solution we use
    azdist = az[1] - az[0];

    if (fabs(azdist) > 180) {
        if (az[0] < az[1])
            azdist = -(360 - fabs(azdist));
        else
            azdist = (360 - fabs(azdist));
    }

    // change of variable
    u[0] = az[0] * M_PI / 180.;
    u[1] = az[1] * M_PI / 180.;
    v[0] = M_PI / 2. - el[0] * M_PI / 180.;
    v[1] = M_PI / 2. - el[1] * M_PI / 180.;

    v_in = M_PI / 2. - el_in * M_PI / 180.;

    // calculate c1 and c2 which define the great circle and then find all
    // of the solutions for the given input elevation

    c2_1 = atan2(tan(v[0]) * sin(u[0]) - tan(v[1]) * sin(u[1]), tan(v[0]) * cos(u[0]) - tan(v[1]) * cos(u[1]));

    c2_2 = atan2(tan(v[0]) * sin(u[0]) - tan(v[1]) * sin(u[1]), tan(v[0]) * cos(u[0]) - tan(v[1]) * cos(u[1])) + M_PI;

    if (fmod((c2_1 - u[0]), M_PI) != 0) {
        c1_1 = sin(v[0]) / sqrt(pow((cos(v[0]) / tan(c2_1 - u[0])), 2.) + 1.);

        u_1a = -asin(1 / (tan(v_in) * sqrt(1 / (c1_1 * c1_1) - 1.))) + c2_1;
        u_1b = M_PI + asin(1 / (tan(v_in) * sqrt(1 / (c1_1 * c1_1) - 1.))) + c2_1;

    } else {
        c1_1 = 0.;
        u_1a = c2_1;
        u_1b = c2_1;
    }

    if (fmod((c2_1 - u[0]), M_PI) != 0) {
        c1_2 = sin(v[0]) / sqrt(pow((cos(v[0]) / tan(c2_2 - u[0])), 2.) + 1);

        u_2a = -asin(1 / (tan(v_in) * sqrt(1 / (c1_2 * c1_2) - 1.))) + c2_2;
        u_2b = M_PI + asin(1 / (tan(v_in) * sqrt(1 / (c1_2 * c1_2) - 1.))) + c2_2;

    } else {
        c1_2 = 0.;
        u_2a = c2_2;
        u_2b = c2_2;
    }

    // get az into the interval [0,360]

    az_1a = fmod(((u_1a * 180. / M_PI) + 360.), 360.);
    az_1b = fmod(((u_1b * 180. / M_PI) + 360.), 360.);
    az_2a = fmod(((u_2a * 180. / M_PI) + 360.), 360.);
    az_2b = fmod(((u_2b * 180. / M_PI) + 360.), 360.);

    // return one of the above solutions based on the relative orientations
    // of the end points

    cross_eq = 0;  // special case when arc crosses the equator
    if (((el[0] > 0) && (el[1] < 0)) || ((el[0] < 0) && (el[1] > 0))) cross_eq = 1;

    if ((azdist <= 0) && (el[0] <= el[1])) {
        if (cross_eq)
            return az_2a;
        else
            return az_1a;
    }

    if ((azdist <= 0) && (el[0] > el[1])) {
        if (cross_eq)
            return az_2b;
        else
            return az_1b;
    }

    if ((azdist > 0) && (el[0] > el[1])) {
        if (cross_eq)
            return az_1a;
        else
            return az_2a;
    }

    if ((azdist > 0) && (el[0] <= el[1])) {
        if (cross_eq)
            return az_1b;
        else
            return az_2b;
    }

    return -1;
}

//-----------------------------------------------------------------------------
// Given az/el for vertices (listed in order) of a 4-sided polygon,
// calculate the az endpoints for a given elevation as well as the elevation
// of the bottom and top of the polygon.
// The left az endpoint is returned first.
//
// All quantities in degrees
//
// Inputs:
//
// az      = 4 element array for azimuth of corner points (in order!)
// el      = "    "              elevation
// el_in   = input elevation at which to evaluate the az endpoints
//
// Outputs:
//
// az_left = left az endpoint
// az_right= right "  "
// min_el  = minimum elevation of the polygon
// maz_el  = maximum elevation of the polygon
//
//-----------------------------------------------------------------------------

void radbox_endpoints(double az[4], double el[4], double el_in, double *az_left, double *az_right, double *min_el,
                      double *max_el, double *az_of_bot)
{
    int i;
    int min_index, max_index;
    double az_end1 = 0, az_end2 = 0;
    int side1_index[2], side2_index[2];
    double az_side1[2], az_side2[2], el_side1[2], el_side2[2];
    double temp;
    int horizontal_side;

    *az_left = -1;
    *az_right = -1;

    // determine which points are the bottom/top
    min_index = 0;
    *min_el = el[0];
    max_index = 0;
    *max_el = el[0];

    for (i = 1; i < 4; i++) {
        if (el[i] <= *min_el) {
            min_index = i;
            *min_el = el[i];
        }

        if (el[i] >= *max_el) {
            max_index = i;
            *max_el = el[i];
        }
    }

    *az_of_bot = az[min_index];

    // if the requested elevation within the polygon do the calculation
    if ((el_in >= *min_el) && (el_in <= *max_el)) {
        // look to the adjacent vertices to see which sides of the polygon
        // contain the endpoints we are interested in

        if (el_in <= el[((min_index - 1) + 4) % 4]) {
            side1_index[0] = min_index;
            side1_index[1] = ((min_index - 1) + 4) % 4;
        } else if ((el_in > el[((min_index - 1) + 4) % 4]) && (el_in <= el[((min_index - 2) + 4) % 4])) {
            side1_index[0] = ((min_index - 1) + 4) % 4;
            side1_index[1] = ((min_index - 2) + 4) % 4;
        } else {
            side1_index[0] = ((min_index - 2) + 4) % 4;
            side1_index[1] = ((min_index - 3) + 4) % 4;
        }

        if (el_in <= el[((min_index + 1) + 4) % 4]) {
            side2_index[0] = min_index;
            side2_index[1] = ((min_index + 1) + 4) % 4;
        } else if ((el_in > el[((min_index + 1) + 4) % 4]) && (el_in <= el[((min_index + 2) + 4) % 4])) {
            side2_index[0] = ((min_index + 1) + 4) % 4;
            side2_index[1] = ((min_index + 2) + 4) % 4;
        } else {
            side2_index[0] = ((min_index + 2) + 4) % 4;
            side2_index[1] = ((min_index + 3) + 4) % 4;
        }

        // evaluate the azimuth along the two sides

        az_side1[0] = az[side1_index[0]];
        az_side1[1] = az[side1_index[1]];
        el_side1[0] = el[side1_index[0]];
        el_side1[1] = el[side1_index[1]];

        az_side2[0] = az[side2_index[0]];
        az_side2[1] = az[side2_index[1]];
        el_side2[0] = el[side2_index[0]];
        el_side2[1] = el[side2_index[1]];

        // check for cases where endpoints of side same elevation

        horizontal_side = 0;

        if (el_side1[0] != el_side1[1])
            az_end1 = az_gcirc(az_side1, el_side1, el_in);
        else
            horizontal_side = 1;

        if (el_side2[0] != el_side2[1])
            az_end2 = az_gcirc(az_side2, el_side2, el_in);
        else
            horizontal_side = 2;

        if (horizontal_side != 0) {
            if (horizontal_side == 1) {
                if (az[side1_index[0]] < az[side1_index[1]]) {
                    *az_left = az[side1_index[0]];
                    *az_right = az[side1_index[1]];
                } else {
                    *az_left = az[side1_index[1]];
                    *az_right = az[side1_index[0]];
                }
            } else {
                if (az[side2_index[0]] < az[side2_index[1]]) {
                    *az_left = az[side2_index[0]];
                    *az_right = az[side2_index[1]];
                } else {
                    *az_left = az[side2_index[1]];
                    *az_right = az[side2_index[0]];
                }
            }
        } else {
            // return the order of the endpoints left, right
            if (az_end1 < az_end2) {
                *az_left = az_end1;
                *az_right = az_end2;
            } else {
                *az_left = az_end2;
                *az_right = az_end1;
            }

            // swap the order if the distance > 180
            if (fabs(*az_right - *az_left) > 180) {
                temp = *az_left;
                *az_left = *az_right;
                *az_right = temp;
            }
        }
    } else if (el_in <= *min_el) {
        *az_left = *az_right = az[min_index];
    } else  {
        *az_left = *az_right = az[max_index];
    }
}
