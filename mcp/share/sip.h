/* mcp: the BLAST master control program
 *
 * sip.h list of functions for listening to and processing sip data (in sip.c)
 *
 * This software is copyright (C) 2002-2012 University of Toronto
 *
 * This file is part of mcp and pcm
 *
 * mcp and pcm are free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp and pcm are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

void WritePrevStatus();

//int sip_setserial(const char *input_tty);

/**********************************************/
/*  SIPDataStruct                             */
/*  Purpose: Store data from the SIP          */
/*   Source: Commands thread (commands.c)     */
/*     Used: Main thread                      */
struct GPSposStruct {
  double lat;   // probably degrees
  double lon;   // probably degrees
  double alt;
};

struct MKSaltStruct {
  float hi;
  float med;
  float lo;
};

struct TimeStruct {
  int UTC;
  int CPU;
};

struct MKScalStruct {
  float m_hi, m_med, m_lo;
  float b_hi, b_med, b_lo;
};

struct SIPDataStruct {
  struct GPSposStruct GPSpos;
  struct TimeStruct GPStime;
  struct MKSaltStruct MKSalt;
  char GPSstatus1;
  char GPSstatus2;
  struct MKScalStruct MKScal;
};

extern struct SIPDataStruct SIPData;
