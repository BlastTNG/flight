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

/**********************************************/
/*  SunSensorDataStruct                       */
/*  Purpose: Store raw sun sensor data        */
/*   Source: Sun Sensor thread                */
/*     Used: Main thread                      */
struct SunSensorDataStruct {
  short int raw_az; // uncalibrated
  short int raw_el; // uncalibrated
  short int prin;
};

extern struct SunSensorDataStruct SunSensorData[3];
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
  int pulse_width; // how wide this pulse should be
  int is_fast; // if a fast pulse is requested: set in motors.c
};

extern struct ISCPulseType isc_pulses;

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
extern struct ISCSolutionStruct ISCSolution[3]; /* isc.c */
extern int iscdata_index;       /* isc.c */

extern short int write_ISC_pointing;     /* isc.c */
