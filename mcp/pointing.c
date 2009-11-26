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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <math.h>
#include <termios.h>
#include <ctype.h>
#include <pthread.h>

#include "blast.h"
#include "mcp.h"
#include "isc_protocol.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "lut.h"
#include "tx.h"
#include "fir.h"

#define FLOAT_ALT 30480
#define FRAMES_TO_OK_ATFLOAT 100

#define GY_IFEL_OFFSET   (0)
#define GY_IFROLL_OFFSET (0)
#define GY_IFYAW_OFFSET  (0)

#define FIR_LENGTH (60*30 * SR)

void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
    double *el);
void azel2radec(double *ra_out, double *dec_out,
    double az, double el, time_t lst, double lat);

double getlst(time_t t, double lon); // defined in starpos.c
double GetJulian(time_t t);

/* Functions in the file 'geomag.c' */
void MagModelInit(int maxdeg);
void GetMagModel(float alt, float glat, float glon, float time,
    float *dec, float *dip, float *ti, float *gv);

int point_index = 0;
struct PointingDataStruct PointingData[3];

struct ElAttStruct {
  double el;
  double gy_offset;
  double weight;
};

struct AzAttStruct {
  double az;
  double gy_ifroll_offset;
  double gy_ifyaw_offset;
  double weight;
};

struct ElSolutionStruct {
  double angle;    // solution's current angle
  double varience; // solution's current sample varience
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic varience - can't do better than this
  double trim; // externally set trim to solution
  double last_input; // last good data point
  double gy_int; // integral of the gyro since the last solution
  double gy_offset; // averaged offset measurement
  double FC; // filter constant
  int n_solutions; // number of angle inputs
  int since_last;
  struct FirStruct *fs;
};

struct AzSolutionStruct {
  double angle;    // solution's current angle
  double varience; // solution's current sample varience
  double samp_weight; // sample weight per sample
  double sys_var;  // sytematic varience - can't do better than this
  double trim; // externally set trim to solution
  double last_input; // last good data point
  double gy_ifroll_int; // integral of the gyro since the last solution
  double gy_ifyaw_int; // integral of the gyro since the last solution
  double gy_ifroll_offset; // offset associated with solution
  double gy_ifyaw_offset;
  double FC; // filter constant
  int n_solutions; // number of angle inputs
  int since_last;
  struct FirStruct *fs2;
  struct FirStruct *fs3;
};

static struct HistoryStruct {
  double *elev_history;
  double *gy_ifel_history;
  double *gy_ifroll_history;
  double *gy_ifyaw_history;
  int i_history;  // points to last valid point.  Not thread safe.
} hs = {NULL, NULL, NULL, NULL, 0};

static struct {
  double az;
  double el;
  int fresh;
} NewAzEl = {0.0, 0.0, 0};

// gyros, with earth's rotation removed
static struct {
  double gy_ifel;
  double gy_ifroll;
  double gy_ifyaw;
} RG;

#define MAX_SUN_EL 5.0

static double sun_az, sun_el; // set in SSConvert and used in UnwindDiff

void SunPos(double tt, double *ra, double *dec); // in starpos.c

#define M2DV(x) ((x / 60.0) * (x / 60.0))

#define MAG_ALIGNMENT 0
//(4.2681)

// limit to 0 to 360.0
void NormalizeAngle(double *A)
{
  *A = fmod(*A, 360.0);
  if (*A < 0)
    *A += 360.0;
}

void UnwindDiff(double ref, double *A)
{
  *A = ref + remainder(*A - ref, 360.0);
}

// adjust *A to be within +-180 of ref
void SetSafeDAz(double ref, double *A)
{
  *A = ref + remainder(*A - ref, 360.0);
  if (sun_el < MAX_SUN_EL)
    return;

  sun_az = ref + remainder(sun_az - ref, 360.0);

  if ((ref < sun_az) && (sun_az < *A)) {
    *A -= 360.0;
  } else if ((ref > sun_az) && (sun_az > *A)) {
    *A += 360.0;
  }
}

/************************************************************************/
/*                                                                      */
/*   MagRead:  use the world magnetic model, atan2 and a lookup table   */
/*             to convert mag_x and mag_y to mag_az                     */
/*                                                                      */
/************************************************************************/
static int MagConvert(double *mag_az)
{
  float year;
  static double mvx, mvy, mvz;
  static double raw_mag_az, raw_mag_pitch;
  static float fdec, dip, ti, gv;
  static double dec;
  static time_t t, oldt;
  struct tm now;
  int i_point_read;
  static int firsttime = 1;
  //static struct LutType magLut = {"/data/etc/mag.lut",0,NULL,NULL,0};

  i_point_read = GETREADINDEX(point_index);

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    /* Initialise magnetic model reader: I'm not sure what the '12' is, but */
    /* I think it has something to do with the accuracy of the modelling -- */
    /* probably shouldn't change this value.  (Adam H.) */
    MagModelInit(12);
    //LutInit(&magLut);

    oldt = 1;
    firsttime = 0;
  }

  /* Every 300 s = 5 min, get new data from the magnetic model.
   *
   * dec = magnetic declination (field direction in az)
   * dip = magnetic inclination (field direction in ele)
   * ti  = intensity of the field in nT
   * gv  = modified form of dec used in polar reasons -- haven't researched
   *       this one
   *
   * The year must be between 2000.0 and 2005.0 with current model data
   *
   * The functions called are in 'geomag.c' (Adam. H) */
  if ((t = PointingData[i_point_read].t) > oldt + 10) {
    oldt = t;
    gmtime_r(&t, &now);
    year = 1900 + now.tm_year + now.tm_yday / 365.25;

    GetMagModel(PointingData[i_point_read].alt / 1000.0,
        PointingData[i_point_read].lat, -PointingData[i_point_read].lon,
        year, &fdec, &dip, &ti, &gv);

    dec = fdec;

  }

  /* The dec is the correction to the azimuth of the magnetic field. */
  /* If negative is west and positive is east, then: */
  /* */
  /*   true bearing = magnetic bearing + dec */
  /* */
  /* Thus, depending on the sign convention, you have to either add or */
  /* subtract dec from az to get the true bearing. (Adam H.) */


  // Enzo commented out these two lines
  //raw_mag_az = (180.0 / M_PI) * atan2(ACSData.mag_y, ACSData.mag_x);
  //*mag_az = LutCal(&magLut, raw_mag_az);

  // cbn added this line
  mvx = MAGX_M*ACSData.mag_x + MAGX_B;
  //mvx = (mvx-0.009)/0.38;
  mvy = MAGY_M*ACSData.mag_y + MAGY_B;
  //mvy = (mvy + 0.018)/0.39;
  mvz = MAGZ_M*ACSData.mag_z + MAGZ_B;
  //mvz = mvz/0.385; 

  raw_mag_az = (180.0 / M_PI) * atan2(mvy, mvx);
  raw_mag_pitch = (180.0/M_PI) * atan(mvz/sqrt(mvx*mvx + mvy*mvy));
  *mag_az = raw_mag_az;
  ACSData.mag_pitch = raw_mag_pitch+(double)dip;

  // Enzo inserted these two lines
  //mag_az_tmp = MagLutCal(&magLut, ACSData.mag_x, ACSData.mag_y, mag_az_tmp);
  //*mag_az = mag_az_tmp;

#if 0
#warning THE MAGNETIC MODEL HAS BEEN DISABLED
  dec = 0; // disable mag model.
#endif

  *mag_az += dec + MAG_ALIGNMENT;

  NormalizeAngle(mag_az);

  NormalizeAngle(&dec);

  PointingData[point_index].mag_model = dec;

  return (1);
}

static int DGPSConvert(double *dgps_az, double *dgps_pitch, double *dgps_roll)
{
  static int last_i_dgpsatt = 0;
  int i_dgpsatt;

  i_dgpsatt = GETREADINDEX(dgpsatt_index);
  *dgps_az = DGPSAtt[i_dgpsatt].az;
  NormalizeAngle(dgps_az);

  *dgps_pitch = DGPSAtt[i_dgpsatt].pitch;
  NormalizeAngle(dgps_pitch);

  *dgps_roll = DGPSAtt[i_dgpsatt].roll;
  NormalizeAngle(dgps_roll);

  if (i_dgpsatt != last_i_dgpsatt) {
    if (DGPSAtt[i_dgpsatt].att_ok == 1) {
      return (1);
    }
  }
  /* *dgps_az = 0; */
  return(0);
}

// return 1 if new sun, and 0 otherwise
#define MIN_SS_AMP 1000
#define MIN_SS_SNR 1.21

#define SS_N_MAX          12
#define BUNK_FUDGE_FACTOR 400
#define MIN_AMP_VAL       2000
static int SSConvert(double *ss_az)
{
  int i_point, i_ss;
  double az;
  double sun_ra, sun_dec, jd;
  static int last_i_ss = -1;

  double max, dtmp;
  double ave;
  double min;
  int i;
  int i_max;
  double sensors[SS_N_MAX];
  unsigned int sensor_uint[SS_N_MAX];
  double nominator, denominator;
  
  double module_calibration[] =
  { /*Palestine 2006*/ /*Brown 2006    */
    11788.0 / 11788.0, //5842.0 / 5842.0,
    11788.0 / 10425.0, //5842.0 / 8816.0,
    11788.0 / 10242.0, //5842.0 / 7147.0,
    11788.0 /  8824.0, //5842.0 / 8951.0,
    11788.0 /  8708.0, //5842.0 / 5914.0,
    11788.0 / 10097.0, //5842.0 / 5769.0,
    11788.0 /  8721.0, //5842.0 / 6790.0,
    11788.0 / 11814.0, //5842.0 / 5761.0,
    11788.0 /  9981.0, //5842.0 / 8989.0,
    11788.0 / 11880.0, //5842.0 / 5928.0,
    11788.0 /  8776.0, //5842.0 / 7140.0,
    11788.0 /  9974.0  //5842.0 / 6360.0
  };

  double module_offsets[] =
  {
    360.,
    330.,
    300.,
    270.,
    240.,
    210.,
    180.,
    150.,
    120.,
    90.,
    60.,
    30.,
  };
  
  
  i_ss = GETREADINDEX(ss_index);
  i_point = GETREADINDEX(point_index);

  /* get current sun az, el */
  jd = GetJulian(PointingData[i_point].t);
  SunPos(jd, &sun_ra, &sun_dec);
  sun_ra *= (12.0 / M_PI);
  sun_dec *= (180.0 / M_PI);

  if (sun_ra < 0)
    sun_ra += 24;

  radec2azel(sun_ra, sun_dec, PointingData[i_point].lst,
      PointingData[i_point].lat, &sun_az, &sun_el);
  
  NormalizeAngle(&sun_az);
  PointingData[point_index].sun_az = sun_az;
  PointingData[point_index].sun_el = sun_el;

  if (i_ss == last_i_ss)
    return (0);

  /* BEGIN NEW SSS RAW CALCULATIONS */

  sensor_uint[0] = SunSensorData[i_ss].m01;
  sensor_uint[1] = SunSensorData[i_ss].m02;
  sensor_uint[2] = SunSensorData[i_ss].m03;
  sensor_uint[3] = SunSensorData[i_ss].m04;
  sensor_uint[4] = SunSensorData[i_ss].m05;
  sensor_uint[5] = SunSensorData[i_ss].m06;
  sensor_uint[6] = SunSensorData[i_ss].m07;
  sensor_uint[7] = SunSensorData[i_ss].m08;
  sensor_uint[8] = SunSensorData[i_ss].m09;
  sensor_uint[9] = SunSensorData[i_ss].m10;
  sensor_uint[10] = SunSensorData[i_ss].m11;
  sensor_uint[11] = SunSensorData[i_ss].m12;

  /* calibrate modules and determine module with max intensity */
  max = 0;  //max sensor value.
  ave = 0;
  min = 1e300;
  i_max = -1;
  for (i = 0; i < SS_N_MAX; i++) {
    sensors[i] = module_calibration[i] * (double)sensor_uint[i];
    ave += sensors[i];
    if (sensors[i] > max) {
      max = sensors[i];
      i_max = i;
    }
    if (sensors[i] < min) {
      min= sensors[i];
    }
  }
  ave/=SS_N_MAX;

  //Determine the minimum value of the immediate neighbors of the maximum
  if (sensors[(i_max + 12 + 1) % 12] < sensors[(i_max + 12 - 1) % 12])
    dtmp = sensors[(i_max + 12 + 1) % 12];
  else
    dtmp = sensors[(i_max + 12 - 1) % 12];

  if (dtmp < min + MIN_AMP_VAL) { //this is a bunk condition
    PointingData[point_index].ss_snr = 0.05;
    return 0;
  }

  // Software tape
  if(i_max < 4 || i_max > 8) {
    PointingData[point_index].ss_snr = 0.0;
    return 0;
  }
 
  // SNR calculation
  PointingData[point_index].ss_snr = max/ave;
  if (PointingData[point_index].ss_snr<MIN_SS_SNR)
    return 0;
  
  nominator = sensors[(i_max+12+1)%12] - sensors[(i_max+12-1)%12];
  denominator = 2.0*sensors[i_max];
  denominator -= sensors[(i_max+12+1)%12] + sensors[(i_max+12-1)%12];
  denominator *= 1.732050808; //sqrt(3)
  //denominator *= cos(sun_el * (M_PI/180.0));

  if (denominator == 0.0) {
    // unphysical solution;
    PointingData[point_index].ss_snr = 0.2;
    return 0;
  }

  az = 0.5*atan2(nominator, denominator);
  if(az < -(1.2*M_PI/12.0) || az > 1.2*M_PI/12.0) {
    // unphysical solution;
    PointingData[point_index].ss_snr = 0.3;
    return 0;
  }
  
  az *= 180.0/M_PI;
  PointingData[point_index].ss_phase = az;
  PointingData[point_index].ss_az_rel_sun =   az - module_offsets[i_max];
  
  /* END SSS RAW CALCULATIONS */

  *ss_az =  PointingData[point_index].ss_az_rel_sun + sun_az  ;

  NormalizeAngle(ss_az);

  return 1;
}

//make history 30s long so that nobody ever exceeds it (with sc_max_age)
#define GY_HISTORY 3000
static void RecordHistory(int index)
{
  /*****************************************/
  /*   Allocate Memory                     */
  if (hs.gy_ifel_history == NULL) {
    hs.gy_ifel_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.gy_ifel_history, 0, GY_HISTORY * sizeof(double));
    hs.gy_ifroll_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.gy_ifroll_history, 0, GY_HISTORY * sizeof(double));
    hs.gy_ifyaw_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.gy_ifyaw_history, 0, GY_HISTORY * sizeof(double));
    hs.elev_history  = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.elev_history, 0, GY_HISTORY * sizeof(double));
  }

  /*****************************************/
  /* record history                        */
  hs.i_history++;
  if (hs.i_history >= GY_HISTORY)
    hs.i_history = 0;

  hs.gy_ifel_history[hs.i_history] = RG.gy_ifel;
  hs.gy_ifroll_history[hs.i_history] = RG.gy_ifroll;
  hs.gy_ifyaw_history[hs.i_history] = RG.gy_ifyaw;
  hs.elev_history[hs.i_history] = PointingData[index].el * M_PI / 180.0;
}

int possible_solution(double az, double el, int i_point) {
  double mag_az, enc_el, d_az;
  
  // test for insanity
  if (!finite(az)) return(0);
  if (!finite(el)) return(0);
  if (el > 70.0) return (0);
  if (el < 0.0) return(0);

  mag_az = PointingData[i_point].mag_az; 

  if (CommandData.use_elenc) {
    enc_el = ACSData.enc_el_raw;
    if (el - enc_el > 5.0) return (0);
    if (enc_el - el > 5.0) return (0);
  }
  
  if (CommandData.use_mag) {
    d_az = az - mag_az;

    if (d_az > 180.0) d_az -= 360;
    if (d_az < -180.0) d_az += 360;
  
    if (d_az > 30.0) return (0);
    if (d_az < -30.0) return (0);
  }


  return(1);
}

/* #define GYRO_VAR 3.7808641975309e-08
   (0.02dps/sqrt(100Hz))^2 : gyro offset error dominated */
#define GYRO_VAR (2.0E-6)
static void EvolveSCSolution(struct ElSolutionStruct *e,
    struct AzSolutionStruct *a, double gy_ifel, double gy_ifel_off, 
    double gy_ifroll, double gy_ifroll_off, double gy_ifyaw, 
    double gy_ifyaw_off, double old_el, int which)
{

  double gy_az;
  static int last_isc_framenum[2] = {0xfffffff, 0xfffffff};
  int i_isc, i_point;
  double new_az, new_el, ra, dec;

  // when we get a new frame, use these to correct for history
  double gy_el_delta = 0;
  double gy_az_delta = 0;
  double gy_raw_el_delta = 0;
  double gy_ifroll_raw_delta = 0;
  double gy_ifyaw_raw_delta = 0;
  double daz;
  int i,j;

  double w1, w2;
  double new_el_offset = 0;
  double new_gy_ifroll_offset = 0;
  double new_gy_ifyaw_offset = 0;

  // evolve el
  e->angle += (gy_ifel + gy_ifel_off) / SR;
  e->varience += GYRO_VAR;
  e->gy_int += gy_ifel / SR; // in degrees

  // evolve az
  old_el *= M_PI / 180.0;
  gy_az = -(gy_ifroll + gy_ifroll_off) * cos(old_el) + -(gy_ifyaw + gy_ifyaw_off) * sin(old_el);
  a->angle += gy_az / SR;
  a->varience += GYRO_VAR;
  a->gy_ifroll_int += gy_ifroll / SR; // in degrees
  a->gy_ifyaw_int += gy_ifyaw / SR; // in degrees

  i_isc = iscpoint_index[which];
  /* in theory, iscpoint_index points to the last ISCSolution with flag set.
   * In cases where we've been having handshaking issues this last solution may
   * have been overwritten, so we check flag, just as a sanity check */
  if (ISCSolution[which][i_isc].flag && ISCSolution[which][i_isc].framenum
      != last_isc_framenum[which]) {
    // get az and el for new solution
    i_point = GETREADINDEX(point_index);
    ra = ISCSolution[which][i_isc].ra * (12.0 / M_PI);
    dec = ISCSolution[which][i_isc].dec * (180.0 / M_PI);
    radec2azel(ra, dec, PointingData[i_point].lst, PointingData[i_point].lat,
        &new_az, &new_el);

    if (possible_solution(new_az, new_el, i_point)) {  // no nans!
      // new solution
      if (isc_pulses[which].age < CommandData.ISCControl[which].max_age) {
        /* Add BDA offset -- there's a pole here at EL = 90 degrees! */
        new_el += CommandData.ISCState[which].elBDA * RAD2DEG;
        if (old_el < 80. * M_PI / 180)
          new_az += CommandData.ISCState[which].azBDA * RAD2DEG / cos(old_el);

        // this solution is isc_pulses.age old: how much have we moved?
        gy_el_delta = 0;
        gy_az_delta = 0;
        gy_raw_el_delta = 0;
        gy_ifroll_raw_delta = 0;
        gy_ifyaw_raw_delta = 0;

        for (i = 0; i < isc_pulses[which].age; i++) {
          j = hs.i_history - i;
          if (j < 0)
            j += GY_HISTORY;

          gy_el_delta += (hs.gy_ifel_history[j] + gy_ifel_off) * (1.0 / SR);
          gy_raw_el_delta += (hs.gy_ifel_history[j]) * (1.0 / SR);

          gy_az_delta += (-(hs.gy_ifroll_history[j] + gy_ifroll_off) *
              cos(hs.elev_history[j]) + -(hs.gy_ifyaw_history[j] + gy_ifyaw_off) *
              sin(hs.elev_history[j])) * (1.0 / SR);
          gy_ifroll_raw_delta += (hs.gy_ifroll_history[j]) * (1.0 / SR);
          gy_ifyaw_raw_delta += (hs.gy_ifyaw_history[j]) * (1.0 / SR);
        }

        // evolve el solution
        e->angle -= gy_el_delta; // rewind to when the frame was grabbed
        w1 = 1.0 / (e->varience);
        if (ISCSolution[which][i_isc].sigma > M_PI) {
          w2 = 0;
        } else {
          w2 = 10.0 * ISCSolution[which][i_isc].sigma
            * (180.0 / M_PI); //e->samp_weight;
          if (w2 > 0)
            w2 = 1 / (w2 * w2);
          else
            w2 = 0; // shouldn't happen
        }

        if (w2 > 0.0 ) {
          // Calculate el offset
          e->since_last -= isc_pulses[which].age;
          new_el_offset = ((new_el - e->last_input) - e->gy_int+gy_raw_el_delta)/
            ((1.0/SR) * (double)e->since_last);
          e->last_input = new_el;
          e->since_last = isc_pulses[which].age;
          e->gy_offset = new_el_offset;
          e->gy_int = gy_raw_el_delta;
        }

        UnwindDiff(e->angle, &new_el);
        e->angle = (w1 * e->angle + new_el * w2) / (w1 + w2);
        e->varience = 1.0 / (w1 + w2);
        e->angle += gy_el_delta; // add back to now

        NormalizeAngle(&(e->angle));

        // evolve az solution
        a->angle -= gy_az_delta; // rewind to when the frame was grabbed
        w1 = 1.0 / (a->varience);
        // w2 already set

        UnwindDiff(a->angle, &new_az);
        a->angle = (w1 * a->angle + new_az * w2) / (w1 + w2);
        a->varience = 1.0 / (w1 + w2);
        a->angle += gy_az_delta; // add back to now

        NormalizeAngle(&(a->angle));

        if (w2 > 0.0 ) {
          // Calculate az offset
          daz = remainder(new_az - a->last_input, 360.0);

          a->since_last -= isc_pulses[which].age;
          new_gy_ifroll_offset = -(daz * cos(new_el*M_PI/180) + a->gy_ifroll_int-gy_ifroll_raw_delta) /
            ((1.0/SR) * (double)a->since_last);

          /* Do Gyro_IFyaw */
          new_gy_ifyaw_offset = -(daz * sin(new_el*M_PI/180.0) + a->gy_ifyaw_int-gy_ifyaw_raw_delta) /
            ((1.0/SR) * (double)a->since_last);

          a->last_input = new_az;
          a->since_last = isc_pulses[which].age;
          a->gy_ifroll_offset = new_gy_ifroll_offset;
          a->gy_ifyaw_offset = new_gy_ifyaw_offset;
          a->gy_ifroll_int = gy_ifroll_raw_delta;
          a->gy_ifyaw_int = gy_ifyaw_raw_delta;
        }
      }

      last_isc_framenum[which] = ISCSolution[which][i_isc].framenum;
      isc_pulses[which].age = -1; // reset counter.
    } else if (!finite(new_el) || !finite(new_az)) {
      bprintf(err, "Pointing: Aphysical star camera solution discarded.");
    }
  }
  e->since_last++;
  a->since_last++;
}

/* Gyro noise: 7' / rt(hour) */
/** the new solution is a weighted mean of:
  the old solution evolved by gyro motion and
  the new solution. **/
static void EvolveElSolution(struct ElSolutionStruct *s,
    double gyro, double gy_off,
    double new_angle, int new_reading)
{
  static int i=0;
  double w1, w2;
  double new_offset = 0;
 
  //  if (i%500==1) bprintf(info,"EvolveElSolution: #1 Initial angle %f, new angle %f",s->angle, new_angle);
  s->angle += (gyro + gy_off) / SR;
  s->varience += GYRO_VAR;

  s->gy_int += gyro / SR; // in degrees

  //  if (i%500==1) bprintf(info,"EvolveElSolution: #2 Angle %f, gyro %f, gy_off %f, varience %f, gy_int %f",s->angle, gyro, gy_off, s->varience, s->gy_int);
  if (new_reading) {
    w1 = 1.0 / (s->varience);
    w2 = s->samp_weight;

    s->angle = (w1 * s->angle + new_angle * w2) / (w1 + w2);
    s->varience = 1.0 / (w1 + w2);
    //    if (i%500==1) bprintf(info,"EvolveElSolution: #2 Angle %f, w1 %f, w2 %f, varience %f",s->angle, w1, w2,s->varience);

    if (CommandData.pointing_mode.nw > 0)
      CommandData.pointing_mode.nw--; /* slew veto */
    else {
      /** calculate offset **/
      if (s->n_solutions > 10) { // only calculate if we have had at least 10
        new_offset = ((new_angle - s->last_input) - s->gy_int) /
          ((1.0/SR) * (double)s->since_last);

        if (fabs(new_offset) > 500.0)
          new_offset = 0; // 5 deg step is bunk!


        s->gy_offset = filter(new_offset, s->fs);
      }
      s->since_last = 0;
      if (s->n_solutions<10000) {
        s->n_solutions++;
      }
    }

    s->gy_int = 0.0;
    s->last_input = new_angle;
  }
  s->since_last++;
  i++;
}

// Weighted mean of ElAtt and ElSol
static void AddElSolution(struct ElAttStruct *ElAtt,
    struct ElSolutionStruct *ElSol, int add_offset)
{
  double weight, var;

  var = ElSol->varience + ElSol->sys_var;

  if (var > 0)
    weight = 1.0 / var;
  else
    weight = 1.0E30; // should be impossible

  ElAtt->el = (weight * (ElSol->angle + ElSol->trim) +
      ElAtt->weight * ElAtt->el) /
    (weight + ElAtt->weight);

  if (add_offset) {
    ElAtt->gy_offset = (weight * ElSol->gy_offset +
        ElAtt->weight * ElAtt->gy_offset) /
      (weight + ElAtt->weight);
  }

  ElAtt->weight += weight;
}

// Weighted mean of AzAtt and AzSol
static void AddAzSolution(struct AzAttStruct *AzAtt,
    struct AzSolutionStruct *AzSol, int add_offset)
{
  double weight, var, az;

  var = AzSol->varience + AzSol->sys_var;
  az = AzSol->angle + AzSol->trim;

  if (var > 0)
    weight = 1.0 / var;
  else
    weight = 1.0E30; // should be impossible

  UnwindDiff(AzAtt->az, &az);
  AzAtt->az = (weight * (az) + AzAtt->weight * AzAtt->az) /
    (weight + AzAtt->weight);
  NormalizeAngle(&(AzAtt->az));

  if (add_offset) {
    AzAtt->gy_ifroll_offset = (weight * AzSol->gy_ifroll_offset +
        AzAtt->weight * AzAtt->gy_ifroll_offset) / (weight + AzAtt->weight);
    AzAtt->gy_ifyaw_offset = (weight * AzSol->gy_ifyaw_offset +
        AzAtt->weight * AzAtt->gy_ifyaw_offset) / (weight + AzAtt->weight);
  }

  AzAtt->weight += weight;
}

//FIXME: need to add rotation of earth correction
static void EvolveAzSolution(struct AzSolutionStruct *s, double gy_ifroll,
    double gy_ifroll_offset, double gy_ifyaw, double gy_ifyaw_offset, double el, double new_angle,
    int new_reading)
{
  double w1, w2;
  double gy_az;
  double new_offset, daz;

  el *= M_PI / 180.0; // want el in radians
  gy_az = -(gy_ifroll + gy_ifroll_offset) * cos(el) + -(gy_ifyaw + gy_ifyaw_offset) * sin(el);

  s->angle += gy_az / SR;
  s->varience += GYRO_VAR;

  s->gy_ifroll_int += gy_ifroll / SR; // in degrees
  s->gy_ifyaw_int += gy_ifyaw / SR; // in degrees

  if (new_reading) {
    w1 = 1.0 / (s->varience);
    w2 = s->samp_weight;

    UnwindDiff(s->angle, &new_angle);
    s->angle = (w1 * s->angle + new_angle * w2) / (w1 + w2);
    s->varience = 1.0 / (w1 + w2);
    NormalizeAngle(&(s->angle));

    if (CommandData.pointing_mode.nw > 0)
      CommandData.pointing_mode.nw--; /* slew veto */
    else {
      if (s->n_solutions > 10) { // only calculate if we have had at least 10

        daz = remainder(new_angle - s->last_input, 360.0);

        /* Do Gyro_IFroll */
        new_offset = -(daz * cos(el) + s->gy_ifroll_int) /
          ((1.0/SR) * (double)s->since_last);
        s->gy_ifroll_offset = filter(new_offset, s->fs2);;

        /* Do Gyro_IFyaw */
        new_offset = -(daz * sin(el) + s->gy_ifyaw_int) /
          ((1.0/SR) * (double)s->since_last);
        s->gy_ifyaw_offset = filter(new_offset, s->fs3);;

      }
      s->since_last = 0;
      if (s->n_solutions<10000) {
        s->n_solutions++;
      }
    }
    s->gy_ifroll_int = 0.0;
    s->gy_ifyaw_int = 0.0;
    s->last_input = new_angle;
  }
  s->since_last++;
}

/*****************************************************************
  do sensor selection;
  update the pointing;
  */
/* Elevation encoder uncertainty: */
void Pointing(void)
{
  double R, cos_e, cos_l, cos_a;
  double sin_e, sin_l, sin_a;
  double ra, dec, az, el;
  static int j=0;

  int ss_ok, mag_ok, dgps_ok;
  static unsigned dgps_since_ok = 500;
  static unsigned ss_since_ok = 500;
  double ss_az, mag_az;
  double dgps_az, dgps_pitch, dgps_roll;
  double clin_elev;
  static int no_dgps_pos = 0, last_i_dgpspos = 0, using_sip_gps = -1;
  static double last_good_lat=0, last_good_lon=0;
  static int since_last_good_dgps_pos=5;
  static int i_at_float = 0;

  static int firsttime = 1;

  int i_dgpspos, dgpspos_ok;
  int i_point_read;

  static struct LutType elClinLut = {"/data/etc/clin_elev.lut",0,NULL,NULL,0};

  struct ElAttStruct ElAtt = {0.0, 0.0, 0.0};
  struct AzAttStruct AzAtt = {0.0, 0.0, 0.0, 0.0};

  static struct ElSolutionStruct EncEl = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(60), //sample weight
    M2DV(20), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    GY_IFEL_OFFSET, // gy offset
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL // firstruct					
  };
  static struct ElSolutionStruct ClinEl = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(60), //sample weight
    M2DV(60), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    GY_IFEL_OFFSET, // gy offset
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL // firstruct					
  };
  static struct ElSolutionStruct ISCEl = {0.0, // starting angle
    719.9 * 719.9, // starting varience
    1.0 / M2DV(0.2), //sample weight
    M2DV(0.2), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    GY_IFEL_OFFSET, // gy offset
    0.0001, // filter constant
    0, 0 // n_solutions, since_last
  };
  static struct ElSolutionStruct OSCEl = {0.0, // starting angle
    719.9 * 719.9, // starting varience
    1.0 / M2DV(0.2), //sample weight
    M2DV(0.2), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, // gy integral
    GY_IFEL_OFFSET, // gy offset
    0.0001, // filter constant
    0, 0 // n_solutions, since_last
  };
  static struct AzSolutionStruct NullAz = {91.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(6), //sample weight
    M2DV(6000), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    GY_IFROLL_OFFSET, GY_IFYAW_OFFSET, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };
  static struct AzSolutionStruct MagAz = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(60), //sample weight
    M2DV(90), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    GY_IFROLL_OFFSET, GY_IFYAW_OFFSET, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };
  static struct AzSolutionStruct DGPSAz = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(20), //sample weight
    M2DV(15), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    GY_IFROLL_OFFSET, GY_IFYAW_OFFSET, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };
  static struct AzSolutionStruct SSAz =  {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(30), //sample weight
    M2DV(60), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    GY_IFROLL_OFFSET, GY_IFYAW_OFFSET, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };
  static struct AzSolutionStruct ISCAz = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(0.3), //sample weight
    M2DV(0.2), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    GY_IFROLL_OFFSET, GY_IFYAW_OFFSET, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };
  static struct AzSolutionStruct OSCAz = {0.0, // starting angle
    360.0 * 360.0, // starting varience
    1.0 / M2DV(0.3), //sample weight
    M2DV(0.2), // systemamatic varience
    0.0, // trim
    0.0, // last input
    0.0, 0.0, // gy integrals
    GY_IFROLL_OFFSET, GY_IFYAW_OFFSET, // gy offsets
    0.0001, // filter constant
    0, 0, // n_solutions, since_last
    NULL, NULL
  };

  if (firsttime) {
    firsttime = 0;
    ClinEl.trim = CommandData.clin_el_trim;
    EncEl.trim = CommandData.enc_el_trim;
    NullAz.trim = CommandData.null_az_trim;
    MagAz.trim = CommandData.mag_az_trim;
    DGPSAz.trim = CommandData.dgps_az_trim;
    SSAz.trim = CommandData.ss_az_trim;

    ClinEl.fs = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    initFir(ClinEl.fs, FIR_LENGTH);
    EncEl.fs = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    initFir(EncEl.fs, FIR_LENGTH);

    NullAz.fs2 = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    NullAz.fs3 = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    initFir(NullAz.fs2, (int)(10)); // not used
    initFir(NullAz.fs3, (int)(10)); // not used

    MagAz.fs2 = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    MagAz.fs3 = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    initFir(MagAz.fs2, FIR_LENGTH);
    initFir(MagAz.fs3, FIR_LENGTH);

    DGPSAz.fs2 = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    DGPSAz.fs3 = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    initFir(DGPSAz.fs2, FIR_LENGTH);
    initFir(DGPSAz.fs3, FIR_LENGTH);

    SSAz.fs2 = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    SSAz.fs3 = (struct FirStruct *)balloc(fatal, sizeof(struct FirStruct));
    initFir(SSAz.fs2, FIR_LENGTH);
    initFir(SSAz.fs3, FIR_LENGTH);

    // the first t about to be read needs to be set
    PointingData[GETREADINDEX(point_index)].t = mcp_systime(NULL); // CPU time

    /* Load lat/lon from disk */
    last_good_lon = PointingData[0].lon = PointingData[1].lon = PointingData[2].lon
      = CommandData.lon;
    last_good_lat = PointingData[0].lat = PointingData[1].lat = PointingData[2].lat
      = CommandData.lat;
  }

  if (elClinLut.n == 0)
    LutInit(&elClinLut);

  i_dgpspos = GETREADINDEX(dgpspos_index);
  i_point_read = GETREADINDEX(point_index);

  // Make aristotle correct
  R = 15.0 / 3600.0;
  cos_e = cos(PointingData[i_point_read].el * (M_PI / 180.0));
  sin_e = sin(PointingData[i_point_read].el * (M_PI / 180.0));
  cos_l = cos(PointingData[i_point_read].lat * (M_PI / 180.0));
  sin_l = sin(PointingData[i_point_read].lat * (M_PI / 180.0));
  cos_a = cos(PointingData[i_point_read].az * (M_PI / 180.0));
  sin_a = sin(PointingData[i_point_read].az * (M_PI / 180.0));

  PointingData[point_index].gy_ifel_earth = R * (-cos_l * sin_a);
  PointingData[point_index].gy_ifroll_earth = R *
    (cos_e * sin_l - cos_l * sin_e * cos_a);
  PointingData[point_index].gy_ifyaw_earth = R *
    (sin_e * sin_l + cos_l * cos_e * cos_a);
  RG.gy_ifel = ACSData.gy_ifel - PointingData[point_index].gy_ifel_earth;
  //  if (j%500==0) bprintf(info,"Pointing: ACSData.enc_el_raw = %f",ACSData.enc_el_raw);
  //  if (j%500==0) bprintf(info,"Pointing: RG.gy1 = %f, gy1_earth= %f, cos_l =%f sin_a = %f",ACSData.gy_ifel,PointingData[point_index].gy1_earth, cos_a, sin_a);
  RG.gy_ifroll = ACSData.gy_ifroll - PointingData[point_index].gy_ifroll_earth;
  RG.gy_ifyaw = ACSData.gy_ifyaw - PointingData[point_index].gy_ifyaw_earth;
  PointingData[point_index].v_az = (-1.0)*RG.gy_ifroll*cos_e-RG.gy_ifyaw*sin_e;
  /*************************************/
  /** Record history for gyro offsets **/
  RecordHistory(i_point_read);

  PointingData[point_index].t = mcp_systime(NULL); // CPU time

  /************************************************/
  /** Set the official Lat and Long: prefer dgps **/
  if (i_dgpspos != last_i_dgpspos) {
    if (using_sip_gps != 0)
      bprintf(info, "Pointing: Using dGPS for positional data");
    last_i_dgpspos = i_dgpspos;
    dgpspos_ok = ((fabs(last_good_lat - DGPSPos[i_dgpspos].lat) < 0.5) &&
                 (fabs(last_good_lon - DGPSPos[i_dgpspos].lon) < 0.5)) ||
	         (since_last_good_dgps_pos >=5);
    
    if (dgpspos_ok) {
      last_good_lat = PointingData[point_index].lat = DGPSPos[i_dgpspos].lat;
      last_good_lon = PointingData[point_index].lon = DGPSPos[i_dgpspos].lon;
      PointingData[point_index].alt = DGPSPos[i_dgpspos].alt;
      using_sip_gps = 0;
      no_dgps_pos = 0;
      since_last_good_dgps_pos = 0;
    } else {
      since_last_good_dgps_pos++;
    }
  } else {
    no_dgps_pos++;
    if (no_dgps_pos > 3000) { // no dgps for 30 seconds - revert to sip
      if (using_sip_gps != 1)
        bprintf(info, "Pointing: Using SIP for positional data");
      PointingData[point_index].lat = SIPData.GPSpos.lat;
      PointingData[point_index].lon = SIPData.GPSpos.lon;
      PointingData[point_index].alt = SIPData.GPSpos.alt;
      using_sip_gps = 1;
    }
  }

  /* At float check */
  if (PointingData[point_index].alt < FLOAT_ALT) {
    PointingData[point_index].at_float = 0;
    i_at_float = 0;
  } else {
    i_at_float++;
    if (i_at_float > FRAMES_TO_OK_ATFLOAT) {
      PointingData[point_index].at_float = 1;
    } else {
      PointingData[point_index].at_float = 0;
    }
  }

  /* Save lat/lon */
  CommandData.lat = PointingData[point_index].lat;
  CommandData.lon = PointingData[point_index].lon;

  /*****************************/
  /** set time related things **/
  PointingData[point_index].mcp_frame = ACSData.mcp_frame;
  PointingData[point_index].t = mcp_systime(NULL); // for now use CPU time
  PointingData[point_index].lst = getlst(PointingData[point_index].t,
      PointingData[point_index].lon);

  /*************************************/
  /**      do ISC Solution            **/
  EvolveSCSolution(&ISCEl, &ISCAz,
      RG.gy_ifel,   PointingData[i_point_read].gy_ifel_offset,
      RG.gy_ifroll, PointingData[i_point_read].gy_ifroll_offset,
      RG.gy_ifyaw,  PointingData[i_point_read].gy_ifyaw_offset,
      PointingData[point_index].el, 0);

  /*************************************/
  /**      do OSC Solution            **/
  EvolveSCSolution(&OSCEl, &OSCAz,
      RG.gy_ifel,   PointingData[i_point_read].gy_ifel_offset,
      RG.gy_ifroll, PointingData[i_point_read].gy_ifroll_offset,
      RG.gy_ifyaw,  PointingData[i_point_read].gy_ifyaw_offset,
      PointingData[point_index].el, 1);

  /*************************************/
  /**      do elevation solution      **/
  clin_elev = LutCal(&elClinLut, ACSData.clin_elev);
  /* x = ACSData.clin_elev; */
  /*   clin_elev = ((((1.13288E-19*x - 1.83627E-14)*x + */
  /* 		 1.17066e-9)*x - 3.66444E-5)*x + 0.567815)*x - 3513.56; */

  EvolveElSolution(&ClinEl, RG.gy_ifel, PointingData[i_point_read].gy_ifel_offset,
      clin_elev, 1);
  EvolveElSolution(&EncEl, RG.gy_ifel, PointingData[i_point_read].gy_ifel_offset,
      ACSData.enc_el_raw, 1);

  if (CommandData.use_elenc) {
    AddElSolution(&ElAtt, &EncEl, 1);
  }

  if (CommandData.use_elclin) {
    AddElSolution(&ElAtt, &ClinEl, 1);
  }
  if (CommandData.use_isc) {
    AddElSolution(&ElAtt, &ISCEl, 0);
  }

  if (CommandData.use_osc) {
    AddElSolution(&ElAtt, &OSCEl, 0);
  }

  PointingData[point_index].gy_ifel_offset = (CommandData.el_autogyro)
    ? ElAtt.gy_offset : CommandData.gy_ifel_offset;
  PointingData[point_index].el = ElAtt.el;

  /*******************************/
  /**      do az solution      **/
  /** Convert Sensors **/
  mag_ok = MagConvert(&mag_az);

  ss_ok = SSConvert(&ss_az);
  if (ss_ok) {
    ss_since_ok = 0;
  } else {
    ss_since_ok++;
  }

  dgps_ok = DGPSConvert(&dgps_az, &dgps_pitch, &dgps_roll);
  if (dgps_ok) {
    dgps_since_ok = 0;
  } else {
    dgps_since_ok++;
  }

  /** evolve solutions **/
  EvolveAzSolution(&NullAz,
      RG.gy_ifroll, PointingData[i_point_read].gy_ifroll_offset,
      RG.gy_ifyaw,  PointingData[i_point_read].gy_ifyaw_offset,
      PointingData[point_index].el,
      0.0, 0);
  /** MAG Az **/
  EvolveAzSolution(&MagAz,
      RG.gy_ifroll, PointingData[i_point_read].gy_ifroll_offset,
      RG.gy_ifyaw,  PointingData[i_point_read].gy_ifyaw_offset,
      PointingData[point_index].el,
      mag_az, mag_ok);

  /** DGPS Az **/
  EvolveAzSolution(&DGPSAz,
      RG.gy_ifroll, PointingData[i_point_read].gy_ifroll_offset,
      RG.gy_ifyaw,  PointingData[i_point_read].gy_ifyaw_offset,
      PointingData[point_index].el,
      dgps_az, dgps_ok);

  /** Sun Sensor **/
  EvolveAzSolution(&SSAz,
      RG.gy_ifroll, PointingData[i_point_read].gy_ifroll_offset,
      RG.gy_ifyaw,  PointingData[i_point_read].gy_ifyaw_offset,
      PointingData[point_index].el,
      ss_az, ss_ok);

  if (CommandData.fast_gy_offset>0) {
    CommandData.fast_gy_offset--;
  }

  //bprintf(info, "off: %g %g %g %g\n", EncEl.angle, ClinEl.angle, EncEl.gy_offset, ClinEl.gy_offset);

  AddAzSolution(&AzAtt, &NullAz, 1);
  /** add az solutions **/
  if (CommandData.use_mag) {
    AddAzSolution(&AzAtt, &MagAz, 1);
  }
  if (CommandData.use_sun) {
    AddAzSolution(&AzAtt, &SSAz, 1);
  }
  if (CommandData.use_gps) {
    AddAzSolution(&AzAtt, &DGPSAz, 1);
  }
  if (CommandData.use_isc) {
    AddAzSolution(&AzAtt, &ISCAz, 0);
  }
  if (CommandData.use_osc) {
    AddAzSolution(&AzAtt, &OSCAz, 0);
  }

  if(j==500) bprintf(info, "Pointing use_mag = %i, use_sun = %i, use_gps = %i, use_isc = %i, use_osc = %i",CommandData.use_mag,CommandData.use_sun, CommandData.use_gps, CommandData.use_isc, CommandData.use_osc);
  PointingData[point_index].az = AzAtt.az;
  if (CommandData.az_autogyro) {
    PointingData[point_index].gy_ifroll_offset = AzAtt.gy_ifroll_offset;
    PointingData[point_index].gy_ifyaw_offset  = AzAtt.gy_ifyaw_offset;
  } else {
    PointingData[point_index].gy_ifroll_offset = CommandData.gy_ifroll_offset;
    PointingData[point_index].gy_ifyaw_offset  = CommandData.gy_ifyaw_offset;
  }

  /** calculate ra/dec for convenience on the ground **/
  azel2radec(&ra, &dec,
      PointingData[point_index].az,  PointingData[point_index].el,
      PointingData[point_index].lst, PointingData[point_index].lat);
  radec2azel(ra, dec, PointingData[point_index].lst,
      PointingData[point_index].lat,
      &az, &el);

  PointingData[point_index].ra = ra;
  PointingData[point_index].dec = dec;
  /** record solutions in pointing data **/
  //  if (j%500==0) bprintf(info,"Pointing: PointingData.enc_el = %f",PointingData[point_index].enc_el);
  PointingData[point_index].enc_el = EncEl.angle;
  PointingData[point_index].enc_sigma = sqrt(EncEl.varience + EncEl.sys_var);
  PointingData[point_index].clin_el = ClinEl.angle;
  PointingData[point_index].clin_sigma = sqrt(ClinEl.varience + ClinEl.sys_var);

  PointingData[point_index].mag_az = MagAz.angle;
  PointingData[point_index].mag_sigma = sqrt(MagAz.varience + MagAz.sys_var);
  PointingData[point_index].dgps_az = DGPSAz.angle;
  PointingData[point_index].dgps_pitch = dgps_pitch;
  PointingData[point_index].dgps_roll = dgps_roll;
  PointingData[point_index].dgps_sigma = sqrt(DGPSAz.varience + DGPSAz.sys_var);
  PointingData[point_index].ss_az = SSAz.angle;
  PointingData[point_index].ss_sigma = sqrt(SSAz.varience + SSAz.sys_var);

  PointingData[point_index].isc_az = ISCAz.angle;
  PointingData[point_index].isc_el = ISCEl.angle;
  PointingData[point_index].isc_sigma = sqrt(ISCEl.varience + ISCEl.sys_var);
  PointingData[point_index].isc_gy_ifel_offset   = ISCEl.gy_offset;
  PointingData[point_index].isc_gy_ifroll_offset = ISCAz.gy_ifroll_offset;
  PointingData[point_index].isc_gy_ifyaw_offset  = ISCAz.gy_ifyaw_offset;

  PointingData[point_index].osc_az = OSCAz.angle;
  PointingData[point_index].osc_el = OSCEl.angle;
  PointingData[point_index].osc_sigma = sqrt(OSCEl.varience + OSCEl.sys_var);
  PointingData[point_index].osc_gy_ifel_offset   = OSCEl.gy_offset;
  PointingData[point_index].osc_gy_ifroll_offset = OSCAz.gy_ifroll_offset;
  PointingData[point_index].osc_gy_ifyaw_offset  = OSCAz.gy_ifyaw_offset;

  /********************/
  /* Set Manual Trims */
  if (NewAzEl.fresh == -1) {
    ClinEl.trim = 0.0;
    EncEl.trim = 0.0;
    NullAz.trim = 0.0;
    MagAz.trim = 0.0;
    DGPSAz.trim = 0.0;
    SSAz.trim = 0.0;
    NewAzEl.fresh = 0;
  }

  if (NewAzEl.fresh==1) {
    ClinEl.trim = NewAzEl.el - ClinEl.angle;	
    EncEl.trim = NewAzEl.el - EncEl.angle;	
    NullAz.trim = NewAzEl.az - NullAz.angle;
    MagAz.trim = NewAzEl.az - MagAz.angle;
    if (dgps_since_ok<500) {
      DGPSAz.trim = NewAzEl.az - DGPSAz.angle;
    }
    if (ss_since_ok<500) {
      SSAz.trim = NewAzEl.az - SSAz.angle;
    }
    NewAzEl.fresh = 0;
  }

  point_index = INC_INDEX(point_index);

  CommandData.clin_el_trim = ClinEl.trim;
  CommandData.enc_el_trim = EncEl.trim;
  CommandData.null_az_trim = NullAz.trim;
  CommandData.mag_az_trim = MagAz.trim;
  CommandData.dgps_az_trim = DGPSAz.trim;
  CommandData.ss_az_trim = SSAz.trim;
  j++;

}

// called from the command thread in command.h
void SetRaDec(double ra, double dec)
{
  int i_point;
  i_point = GETREADINDEX(point_index);

  radec2azel(ra, dec, PointingData[i_point].lst,
      PointingData[i_point].lat,
      &(NewAzEl.az), &(NewAzEl.el));

  NewAzEl.fresh = 1;
}

void SetTrimToSC(int which)
{
  int i_point;

  i_point = GETREADINDEX(point_index);
  if (which == 0) {
    NewAzEl.az = PointingData[i_point].isc_az;
    NewAzEl.el = PointingData[i_point].isc_el;
  } else {
    NewAzEl.az = PointingData[i_point].osc_az;
    NewAzEl.el = PointingData[i_point].osc_el;
  }

  NewAzEl.fresh = 1;
}

void AzElTrim(double az, double el)
{
  NewAzEl.az = az;
  NewAzEl.el = el;

  NewAzEl.fresh = 1;
}

void ClearTrim()
{
  NewAzEl.fresh = -1;
}
