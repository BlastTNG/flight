/* mcp: the Spider master control program
 *
 * pointing_struct.h: definition of globals useful to pointing
 *
 * This software is copyright (C) 2003-2007 University of Toronto
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

/********************************************************************/
/*
                    Global data structures:
    Each of the structures below (except PointingDataStruct)
    is used by a thread to collect raw pointing data from some
    sensor or set of sensors.
    -The data are to be written to the downlinkframe (as needed)
     in tx.c::StoreData() -- or other appropriate main-loop function
    -The data are to be read and collected into PointingData in pointing.c
     unless the raw data is actually what is wanted...
    -Where the data are written and read in different threads, a 3 way
     buffer is used.
    -r_index = GETREADINDEX(wr_index) can be used to get the readindex.
     it should be valid for atomic reads for at least 100ms.
    -index = INC_INDEX(index) can be used to increment the write index.
*/

#include <math.h>
#include <time.h>

#define RAD2SEC (180. * 3600. / M_PI / 15.)  /* radians to seconds (of time) */
#define SEC2RAD (1. / RAD2SEC)
#define DEG2RAD (M_PI / 180.)  /* degrees to radians */
#define RAD2DEG (180. / M_PI)  /* radians to degrees */
#define RAD2ARCSEC (180. * 3600. / M_PI)  /* radians to arcseconds */

/**********************************************/
/*  ReactWheelStruct                          */
/*  Purpose: Store the RW vel and current     */
/*           read in from the BLASTBus        */
/*  Source:                                   */
/*  Used:                                     */
struct ReactWheelStruct {
  double vel; // deg/s
  double i;   // Amps
};

extern struct ReactWheelStruct RWData;

/**********************************************/
/*  ACSDataStruct                             */
/*  Purpose: Store raw pointing info          */
/*   Source: main thread; GetACS()            */
/*     Used: Main thread;                     */
/*  Does not need to be a curcular buffer...  */
struct ACSDataStruct {
  double enc_table; // degrees
  double gyro1;    // deg/s
  double gyro2;    // deg/s
  double gyro3;    // deg/s
  double gyro4;    // deg/s
  double gyro5;    // deg/s
  double gyro6;    // deg/s
  int mcp_frame;
  time_t t;
};

extern struct ACSDataStruct ACSData;


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

/**********************************************/
/*  PointingDataStruct                        */
/*  Purpose: Store derived pointing info      */
/*   Source: main thread; pointing.c          */
/*     Used: Main thread; VSC thread          */
struct PointingDataStruct {
  double az; // lmf: For now we are just integrating the 
             // gyros.
  time_t t;
};

extern struct PointingDataStruct PointingData[3];
extern int point_index;
