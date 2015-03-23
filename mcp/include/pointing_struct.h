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
  double pss1_i1;   //counts
  double pss1_i2;   //counts
  double pss1_i3;   //counts
  double pss1_i4;   //counts
  double pss2_i1;   //counts
  double pss2_i2;   //counts
  double pss2_i3;   //counts
  double pss2_i4;   //counts
  double pss3_i1;   //counts
  double pss3_i2;   //counts
  double pss3_i3;   //counts
  double pss3_i4;   //counts
  double pss4_i1;   //counts
  double pss4_i2;   //counts
  double pss4_i3;   //counts
  double pss4_i4;   //counts
  double enc_raw_el; // degrees
  double clin_elev;// counts
  double vel_rw; // deg/s
  double res_piv; // deg/s
  double ifel_gy;    // deg/s
  double ifroll_gy;    // deg/s
  double ifyaw_gy;    // deg/s
  int hwpr_pot; // used in hwpr.c
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
  double vel_rw; // in degrees per second
  int temp; // drive temperature in deg Celcius
  double current; // drive current read from controller
  unsigned int status;  // drive status
  unsigned int fault_reg; // drive fault register
  unsigned short int drive_info; // motorinfo struct
  unsigned int err_count; // count of serious serial errors
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
  double enc_raw_el; // in degrees
  int temp; // drive temperature in deg Celcius
  double current; // drive current read from controller
  unsigned int status;  // drive status
  unsigned int fault_reg; // drive fault register
  unsigned short int drive_info; // motorinfo struct
  unsigned int err_count; // count of serious serial errors
};
extern struct ElevMotorDataStruct ElevMotorData[3];
extern int elev_motor_index; // defined in motors.c
/**********************************************/
/*  Pivot Motor Data Struct                   */
/*  - Stores information read from serial     */
/*  from the Pivot                  */
/*  - Written to struct in the serial thread  */
/*  reactComm in motors.c                     */
/*  - Written to the frame in the main thread */
/*  USE A CIRCULAR BUFFER !!!                 */
struct PivotMotorDataStruct{
  double res_piv; // in degrees
  double current; // drive current read from controller
  int dps_piv; // filtered pivot velocity
  unsigned int db_stat;  // drive bridge status
  unsigned int dp_stat;  // drive protection status
  unsigned int ds1_stat;  // drive system 1 status
  unsigned short int drive_info; // motorinfo struct
  unsigned int err_count; // count of serious serial errors
};
extern struct PivotMotorDataStruct PivotMotorData[3];
extern int pivot_motor_index; // defined in motors.c


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
  double offset_ifel_gy;
  double offset_ifel_gy_isc; 
  double offset_ifroll_gy_isc; 
  double offset_ifyaw_gy_isc; 
  double offset_ifel_gy_osc; 
  double offset_ifroll_gy_osc; 
  double offset_ifyaw_gy_osc; 
  double offset_ifroll_gy;
  double offset_ifyaw_gy;

  double offset_ifrollmag_gy;
  double offset_ifyawmag_gy;
  double offset_ifrolldgps_gy;
  double offset_ifyawdgps_gy;
  double offset_ifrollpss_gy;
  double offset_ifyawpss_gy;
  
  double ifel_earth_gy;
  double ifroll_earth_gy;
  double ifyaw_earth_gy;
  double lat;       // degrees
  double lon;       // degrees
  double alt;       // m
  int at_float;
  int mcp_frame;
  time_t t;
  time_t lst;
  double mag_az;   // degrees
  double mag_az_raw;   // degrees
  double mag_model; // degrees
  double mag_sigma; // degrees
  double dgps_az; // degrees
  double dgps_pitch; // degrees
  double dgps_roll; // degrees
  double dgps_sigma; // degrees
  double sun_az; // degrees current calculated az of sun
  double sun_el; // degrees current calculated el of sun
  double pss_azraw; //degrees
  double pss1_azraw; //degrees
  double pss2_azraw; //degrees
  double pss3_azraw; //degrees
  double pss4_azraw; //degrees
  double pss_elraw; //degrees
  double pss1_elraw;  //degrees
  double pss2_elraw;  //degrees
  double pss3_elraw;  //degrees
  double pss4_elraw;  //degrees
  double pss1_snr;
  double pss2_snr;
  double pss3_snr;
  double pss4_snr;
  double pss_az;   //degrees
  double pss_sigma;   //degrees
  int pss_ok;
  double isc_az; // degrees
  double isc_el; // degrees
  double isc_sigma; // degrees
  double osc_az; // degrees
  double osc_el; // degrees
  double osc_sigma; // degrees
  double enc_el;
  double enc_sigma;
  double clin_el;
  double clin_el_lut;
  double clin_sigma;
  double v_az; // dps
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
  int att_ok; 
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

struct AxesModeStruct {
  int az_mode;
  int el_mode;
  int el_dir;
  int az_dir;
  double az_dest;
  double el_dest;
  double az_vel;
  double el_vel;
  double el_dith;
  unsigned int i_dith;
};

extern struct AxesModeStruct axes_mode;

extern struct DGPSPosStruct DGPSPos[3];
extern int dgpspos_index;

extern struct DGPSAttStruct DGPSAtt[3];
extern int dgpsatt_index;

extern time_t DGPSTime;

