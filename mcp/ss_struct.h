/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2004 Brown University
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

/****************************************************
  ss_packet_data contains the data sent to mcp
 ****************************************************/
#ifndef SS_STRUCT_H
#define SS_STRUCT_H


typedef struct
{
  float az_center;  //az pixel centroid [0,DATA0_WIDTH)
  float el_center;  //el pixel centroid [0,DATA1_WIDTH)
  int prin;         //current prin value [0,72]
  float az_snr;     //signal to noise ratio.
  float el_snr;     //~2 when there is no sun; ~30 when there is sun
  float cpu_temp;   //cpu temp from I2C bus (celsius)
  float pc_temp;    //motherboard temp from I2C bus (celsius)
  float chipset_temp; //third thermometer from I2C bus (celcius)
} ss_packet_data;

#endif


