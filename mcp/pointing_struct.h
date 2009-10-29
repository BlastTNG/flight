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
#include "sss_struct.h"

#define RAD2SEC (180. * 3600. / M_PI / 15.)  /* radians to seconds (of time) */
#define SEC2RAD (1. / RAD2SEC)
#define DEG2RAD (M_PI / 180.)  /* degrees to radians */
#define RAD2DEG (180. / M_PI)  /* radians to degrees */
#define RAD2ARCSEC (180. * 3600. / M_PI)  /* radians to arcseconds */

/**********************************************/
/*  ACSDataStruct                             */
/*  Purpose: Store raw pointing info          */
/*   Source: main thread; GetACS()            */
/*     Used: Main thread;                     */
/*  Does not need to be a curcular buffer...  */
struct ACSDataStruct {
  double mag_x;    // counts
  double mag_y;    // counts
  double mag_z;    // counts
  double mag_pitch; // degrees
  double enc_el_raw; // degrees
  double clin_elev;// counts
  double rw_vel_raw; // deg/s
  double gy_ifel;    // deg/s
  double gy_ifroll;    // deg/s
  double gy_ifyaw;    // deg/s
  int mcp_frame;
  time_t t;
};
extern struct ACSDataStruct ACSData;

/**********************************************/
/*  RW Motor Data Struct                      */
/*  - Stores encoder/velocity information     */
/*  from the RW                               */
/*  - Written to struct in the serial thread  */
/*  reactComm in motors.c                     */
/*  - Written to the frame in the main thread */
/*  USE A CIRCULAR BUFFER !!!                 */
struct RWMotorDataStruct{
  double rw_vel_raw; // in degrees per second
  int temp; // drive temperature in deg Celcius
  double current; // drive current read from controller
  unsigned int status;  // drive status
  unsigned int fault_reg; // drive fault register
};
extern struct RWMotorDataStruct RWMotorData[3];
extern int rw_motor_index; // defined in motors.c

/**********************************************/
/*  Elev Motor Data Struct                    */
/*  - Stores encoder/velocity information     */
/*  from the Elevation Drive                  */
/*  - Written to struct in the serial thread  */
/*  reactComm in motors.c                     */
/*  - Written to the frame in the main thread */
/*  USE A CIRCULAR BUFFER !!!                 */
struct ElevMotorDataStruct{
  double enc_el_raw; // in degrees
  int temp; // drive temperature in deg Celcius
  double current; // drive current read from controller
  unsigned int status;  // drive status
  unsigned int fault_reg; // drive fault register
};
extern struct ElevMotorDataStruct ElevMotorData[3];
extern int elev_motor_index; // defined in motors.c

extern sss_packet_data SunSensorData[3];
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
  double gy_ifel_offset;
  double isc_gy_ifel_offset; 
  double isc_gy_ifroll_offset; 
  double isc_gy_ifyaw_offset; 
  double osc_gy_ifel_offset; 
  double osc_gy_ifroll_offset; 
  double osc_gy_ifyaw_offset; 
  double gy_ifroll_offset;
  double gy_ifyaw_offset;
  double gy_ifel_earth;
  double gy_ifroll_earth;
  double gy_ifyaw_earth;
  double lat;       // degrees
  double lon;       // degrees
  double alt;       // m
  int at_float;
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
  double ss_snr; // max/ave
  double ss_phase; // az rel cell
  double ss_az_rel_sun;
  double sun_az; // degrees current calculated az of sun
  double sun_el; // degrees current calculated el of sun
  double isc_az; // degrees
  double isc_el; // degrees
  double isc_sigma; // degrees
  double osc_az; // degrees
  double osc_el; // degrees
  double osc_sigma; // degrees
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
};

struct ISCPulseType {
  int age; // time since start of last trigger
  int pulse_index;  // pulse counter index
  int is_fast; // if a fast pulse is requested: set in motors.c
  int last_save;  // time since last autosaved image
  int pulse_req; // the pulse request waiting to be writen
  int ack_wait; // whether we are waiting for ACK from SC
  int ack_timeout; // length of time to wait for ACK before giving up
  int start_wait; // whether we are waiting for data to come back from SC
  int start_timeout; // length of time to wait for data before giving up
  int force_sync; // A semaphore to force MCP into lock-step with ISC
};

extern struct ISCPulseType isc_pulses[2];

struct AxesModeStruct {
  int az_mode;
  int el_mode;
  int el_dir;
  int az_dir;
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
extern struct ISCSolutionStruct ISCSolution[2][5]; /* isc.c */

/* Read and write indicies must be separate for the ISC, since the
 * data essentially comes in bursts -- the average data rate is about
 * 0.4 isc packets per slow frame, but the burst rate can be ~2 packets
 * per slow frame.  There's also a separate index for the pointing solution
 * which points to the last solution packet sent back from ISC (which can be
 * different than the last packet sent back */
extern int iscread_index[2];        /* isc.c */
extern int iscwrite_index[2];       /* isc.c */
extern int iscpoint_index[2];       /* isc.c */
