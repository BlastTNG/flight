/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2003-2004 University of Toronto
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

/********************************************************************/
/*
                    Global data structures:
    Each of the structures below (except PointingDataStruct)
    is used by a thread to collect raw pointing data from some
    sensor or set of sensors.
    -The data are to be written to the downlinkframe (as needed)
     in tx.c::StoreData()
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
#include "isc_protocol.h"   /* required for ISCData; get updates from
                               Ed Chapin and/or the ISC computer */
#include "ss_struct.h"

// GETREADINDEX: converts circular buffer pointer to
#define GETREADINDEX(i) ((i+2) % 3)  /* i - 1 modulo 3 */
#define INC_INDEX(i) ((i + 1) %3)    /* i + 1 modulo 3 */

#define RAD2SEC (180. * 3600. / M_PI / 15.)  /* radians to seconds (of time) */
#define SEC2RAD (1. / RAD2SEC)
#define DEG2RAD (M_PI / 180.)  /* degrees to radians */
#define RAD2DEG (180. / M_PI)  /* radians to degrees */
#define RAD2ARCSEC (180. * 3600. / M_PI)  /* radians to arcseconds */

/**********************************************/
/*  VSCDataStruct                             */
/*  Purpose: Store VSC data                   */
/*   Source: VSC thread in starfind.c         */
/*     Used: Main thread                      */
struct VSCDataStruct {
  int sf_frame; // frame of found blob
  double col;
  double row;
  double mag;
  double az; // az based on ID'd star
  double el; // el based on ID'd star
  int id_frame; // frame of ID'd star
  time_t mcp_time; // time of ID'd star
};

extern struct VSCDataStruct VSCData[3];
extern int vsc_index;


/**********************************************/
/*  ACSDataStruct                             */
/*  Purpose: Store raw pointing info          */
/*   Source: main thread; GetACS()            */
/*     Used: Main thread;                     */
/*  Does not need to be a curcular buffer...  */
struct ACSDataStruct {
  double mag_x; // counts; bias removed
  double mag_y; // counts; bias removed
  double enc_elev; // degrees
  double clin_elev;// counts
  double gyro1;    // deg/s
  double gyro2;    // deg/s
  double gyro3;    // deg/s
  int mcp_frame;
  time_t t;
};

extern struct ACSDataStruct ACSData;

extern ss_packet_data SunSensorData[3];
extern int ss_index;

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
  double az;        // degrees
  double el;        // degrees
  double ra;        // hours, aparent
  double dec;       // degrees, aparent
  double gy1_offset;
  double gy2_offset;
  double gy3_offset;
  double gy1_earth;
  double gy2_earth;
  double gy3_earth;
  double gy_roll_amp;
  double lat;       // degrees
  double lon;       // degrees
  int mcp_frame;
  time_t t;
  time_t lst;
  double mag_az;   // degrees
  double mag_model; // degrees
  double mag_sigma; // degrees
  double dgps_az; // degrees
  double dgps_pitch; // degrees
  double dgps_roll; // degrees
  double dgps_sigma; // degrees
  double ss_az; // degrees
  double ss_sigma; // degrees
  double sun_az; // degrees current calculated az of sun
  double sun_el; // degrees current calculated el of sun
  double isc_az; // degrees
  double isc_el; // degrees
  double isc_sigma; // degrees
  double enc_el;
  double enc_sigma;
  double clin_el;
  double clin_sigma;
};

extern struct PointingDataStruct PointingData[3];
extern int point_index;

/**********************************************/
/*  DGPS Attittude struct                     */
/*  Purpose: Store dgps attitude info         */
/*   Source: dgps thread: dgps.c              */
/*     Used: Main thread;                     */
struct DGPSAttStruct {
  double az;
  double pitch;
  double roll;
  int att_ok; //
};

/**********************************************/
/*  DGPS Position  struct                     */
/*  Purpose: Store dgps position info         */
/*   Source: dgps thread: dgps.c              */
/*     Used: Main thread;                     */
struct DGPSPosStruct{
  double lat; //
  double lon; //
  double alt; //
  double speed; //
  double direction; //
  double climb; //
  int n_sat;  //
  int at_float; //
};

struct ISCPulseType {
  int age; // time since start of last trigger
  int ctr; // where we are in the period
  int pulse_index;  // pulse counter index
  int is_fast; // if a fast pulse is requested: set in motors.c
  int last_save;  // time since last autosaved image
  int pulse_req; // the pulse request waiting to be writen
  int ack_wait; // whether we are waiting for ACK from SC
  int ack_timeout; // length of time to wait for ACK before giving up
};

extern struct ISCPulseType isc_pulses[2];

struct AxesModeStruct {
  int az_mode;
  int el_mode;
  double az_dest;
  double el_dest;
  double az_vel;
  double el_vel;
};

extern struct AxesModeStruct axes_mode;

extern struct DGPSPosStruct DGPSPos[3];
extern int dgpspos_index;

extern struct DGPSAttStruct DGPSAtt[3];
extern int dgpsatt_index;

extern time_t DGPSTime;

/**********************************************/
/* ISC Data struct                            */
/* ISCSolutionStruct is a struct defined in   */
/* isc_protocol.h which is copied verbatim    */
/* from the ISC computer.                     */
/*                                            */
/*  Purpose: Store isc pointing and blob data */
/*   Source: isc thread: isc.c                */
/*     Used: Main thread;                     */
extern struct ISCSolutionStruct ISCSolution[2][3]; /* isc.c */
extern int iscdata_index[2];       /* isc.c */
