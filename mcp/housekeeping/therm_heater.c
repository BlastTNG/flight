/* 
 * therm_heater.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 * History:
 * Created on: Mar 24, 2015 by Seth Hillbrand
 */

#include <stddef.h>

#include <lut.h>
#include <calibrate.h>

/**
 * Returns the linearized temperature in Kelvin of an AD590, given
 * a voltage level.
 * @param m_count Input voltage level, 16-bit unsigned value
 * @return Temperature in Kelvin or negative value if out of range
 */
double calibrate_ad590(int m_count)
{
  double t = M_16_AD590 * (m_count + B_16_AD590);

  /* if t < -73C or t > 67C, assume AD590 is broken */
  if (t < 170)
    t = -1;
  else if (t > 360)
    t = -2;

  return t;
}

/**
 * Returns the temperature in Kelvin of an thermistor, given
 * a voltage level.  Temperature is calculated by lookup table
 * named "thermistor.lut"
 *
 * @param m_count Input voltage level, 16-bit unsigned value
 * @return Temperature in Kelvin or negative value if out of range
 */
double calibrate_thermister(int m_count)
{
  static struct LutType temperature_lut =
     {"/data/etc/blast/thermistor.lut", 0, NULL, NULL, 0};
  static int firsttime = 1;

  double vt;
  double t;

  if (firsttime) {
    firsttime =0;
    LutInit(&temperature_lut);
  }

  vt = M_16T * (m_count + B_16T);
  t = LutCal(&temperature_lut, vt) + 273.15;

  /* if t < -73C or t > 67C, assume thermistor is broken */
  if (t < 170)
    t = -1;
  else if (t > 360)
    t = -2;

  return t;
}
