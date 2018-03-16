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

#ifndef INCLUDE_POINTING_STRUCT_H
#define INCLUDE_POINTING_STRUCT_H

#include <math.h>
#include <time.h>
#include <stdbool.h>

#define NUM_PSS 8
#define NUM_PSS_V 4

/**********************************************/
/*  ACSDataStruct                             */
/*  Purpose: Store raw pointing info          */
/*   Source: main thread; GetACS()            */
/*     Used: Main thread;                     */
/*  Does not need to be a circular buffer...  */
struct ACSDataStruct {
  double mag_x;     // counts;
  double mag_y;     // counts;
  double mag_z;     // counts;
  double pss_i[NUM_PSS][NUM_PSS_V]; // pss voltage
  double enc_elev;  // degrees
  double enc_motor_elev;  // degrees
  double clin_elev; // counts
  double ifel_gy;   // deg/s
  double ifyaw_gy;  // deg/s
  double ifroll_gy; // deg/s
  time_t t;
};

extern struct ACSDataStruct ACSData;




/**********************************************/
/*  PointingDataStruct                        */
/*  Purpose: Store derived pointing info      */
/*   Source: main thread; pointing.c          */
/*     Used: Main thread; VSC thread          */
struct PointingDataStruct {
  double az;        // degrees
  double el;        // degrees
  double az_sigma;  // degrees
  double el_sigma;  // degrees
  double el_noenc;  // degrees
  double ra;        // hours, apparent
  double dec;       // degrees, apparent
  double offset_ifel_gy;
  double offset_ifyaw_gy;
  double offset_ifroll_gy;
  double ifel_earth_gy;
  double ifyaw_earth_gy;
  double ifroll_earth_gy;
  double gy_roll_amp;
  double gy_az;
  double gy_el;
  double gy_total_vel;
  double gy_total_accel;

  double lat;       // degrees
  double lon;       // degrees

  int longitude_octave_since_launch;
  double alt;       // m
  int at_float;
  time_t t;
  time_t lst;
  time_t unix_lsd;  // local sidereal date in seconds

  double mag_az;   // degrees
  double mag_az_raw;   // degrees
  double mag_el;   // degrees
  double mag_el_raw;   // degrees
  double mag_model_dec; // degrees
  double mag_model_dip; // degrees
  double mag_sigma; // degrees
  double mag_strength; // nanoTesla
  double offset_ifrollmag_gy;
  double offset_ifyawmag_gy;

  double sun_az; // degrees current calculated az of sun
  double sun_el; // degrees current calculated el of sun

  int pss_ok;
  double pss_az;
  double pss_el;

  double pss1_azraw; // degrees
  double pss1_elraw; // degrees
  double pss1_snr;
  double pss2_azraw; // degrees
  double pss2_elraw; // degrees
  double pss2_snr;
  double pss_sigma;
  double offset_ifrollpss_gy;
  double offset_ifyawpss_gy;

  double xsc_az[2];
  double xsc_el[2];
  double xsc_sigma[2];
  double offset_ifel_gy_xsc[2];
  double offset_ifyaw_gy_xsc[2];
  double offset_ifroll_gy_xsc[2];
  double estimated_xsc_az_deg[2]; // these come from the full pointing solution, not the individual star camera solution
  double estimated_xsc_el_deg[2];
  double estimated_xsc_ra_hours[2];
  double estimated_xsc_dec_deg[2];

  double enc_el;
  double enc_sigma;
  double enc_motor_el;
  double enc_motor_sigma;

  double clin_el;
  double clin_el_lut;
  double clin_sigma;

  bool requested_el_out_of_bounds;
  bool az_destination_capped;
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
  double mrms;
  double brms;
  unsigned int ant[4];
  int att_ok;
};

/**********************************************/
/*  DGPS Position  struct                     */
/*  Purpose: Store dgps position info         */
/*   Source: dgps thread: dgps.c              */
/*     Used: Main thread;                     */
struct DGPSPosStruct {
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

// extern time_t csbf_gps_time;

typedef struct XSCLastTriggerState
{
    int counter_mcp;                        // mcp counter at the time of last trigger
    int counter_stars;                      // stars counter at the time of last trigger
    double lat;
    time_t lst;
    int trigger_time;                       // Time of the last trigger, measured in loops through xsc_control_triggers
    bool forced_grace_period;
    bool forced_trigger_threshold;
} xsc_last_trigger_state_t;

typedef struct XSCPointingState {
    struct XSCLastTriggerState last_trigger;
    int counter_mcp;                        // the current counter_mcp, passed to the star camera after some delay
    int last_counter_mcp;                   // the previous counter_mcp passed to the star camera
    int last_solution_stars_counter;        // stars counter of last solution used in pointing solution
    double az;                              // XSC Az
    double el;                              // XSC El
    int last_trigger_time;
    int exposure_time_cs;
    double predicted_streaking_px;
} xsc_pointing_state_t;

extern struct XSCPointingState xsc_pointing_state[2];

typedef enum
{
    EL_DRIVE,
    EL_INHIBIT
} elevation_pointing_state_enabled_t;

extern bool scan_entered_snap_mode;
extern bool scan_leaving_snap_mode;

#endif
