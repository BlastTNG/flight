/* mcp: the Spider master control program
 *
 * poiting.c: handles pointing solution logic
 * 	from mcp call the Pointing() function
 *
 * This software is copyright (C) 2002-2007 University of Toronto
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
#include <sys/time.h>

#include "blast.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "tx.h"

#define FLOAT_ALT 30480
#define FRAMES_TO_OK_ATFLOAT 100

#define GY1_OFFSET (0)
#define GY2_OFFSET (0)
#define GY3_OFFSET (0)

#define MAX_ISC_AGE 200

#if 0
void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
    double *el);
void azel2radec(double *ra_out, double *dec_out,
    double az, double el, time_t lst, double lat);

double getlst(time_t t, double lon); // defined in starpos.c
double GetJulian(time_t t);
#endif

int point_index = 0;
struct PointingDataStruct PointingData[2];
pthread_mutex_t point_lock;

#if 0
// gyros, with earth's rotation removed
static struct {
  double gy1;
  double gy2;
  double gy3;
} RG;

static struct HistoryStruct {
  double *elev_history;
  double *gyro1_history;
  double *gyro2_history;
  double *gyro3_history;
  int i_history;  // points to last valid point.  Not thread safe.
} hs = {NULL, NULL, NULL, NULL, 0};

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

#define GY_HISTORY 300
static void RecordHistory(int index)
{
  /*****************************************/
  /*   Allocate Memory                     */
  if (hs.gyro1_history == NULL) {
    hs.gyro1_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.gyro1_history, 0, GY_HISTORY * sizeof(double));
    hs.gyro2_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.gyro2_history, 0, GY_HISTORY * sizeof(double));
    hs.gyro3_history = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.gyro3_history, 0, GY_HISTORY * sizeof(double));
    hs.elev_history  = (double *)balloc(fatal, GY_HISTORY * sizeof(double));
    memset(hs.elev_history, 0, GY_HISTORY * sizeof(double));
  }

  /*****************************************/
  /* record history                        */
  hs.i_history++;
  if (hs.i_history >= GY_HISTORY)
    hs.i_history = 0;

  hs.gyro1_history[hs.i_history] = RG.gy1;
  hs.gyro2_history[hs.i_history] = RG.gy2;
  hs.gyro3_history[hs.i_history] = RG.gy3;
  hs.elev_history[hs.i_history] = PointingData[index].el * M_PI / 180.0;
}
#endif

//TODO eventually need to reimplement code for finding pointing solution

/*****************************************************************
  do sensor selection;
  update the pointing;
  */
/* Elevation encoder uncertainty: */
void Pointing(void)
{
//  double gy_roll, gy2, gy3, el_rad;
//  static double last_good_lat=0, last_good_lon=0;

  static int firsttime = 1;
  static double prevVel,prevTime;
  double curVel,curTime,avVel;
  double dt;
  static struct NiosStruct* gondAz = NULL;
  struct timeval timer;

  int i_point_read;

  if (firsttime) {
    firsttime = 0;
    // the first t about to be read needs to be set
    PointingData[GETREADINDEX(point_index)].t = mcp_systime(NULL); // CPU time
    // initialize PointingData.az
    // lmf: for now assume that the initial position is zero.
    // TODO: Fix this later!
    PointingData[GETREADINDEX(point_index)].az = 0.0; 

    prevVel=ACSData.gyro2;
    gettimeofday(&timer, NULL);
    prevTime=(double)timer.tv_sec + timer.tv_usec/1000000.0;
    gondAz = GetNiosAddr("gond_az");
  }
  curVel=ACSData.gyro2;
  gettimeofday(&timer, NULL);
  curTime=(double)timer.tv_sec + timer.tv_usec/1000000.0;
  avVel=(curVel+prevVel)/2.0;
  dt = (curTime - prevTime);
  PointingData[point_index].az = PointingData[GETREADINDEX(point_index)].az + (avVel * dt); 
  
  int data = (int) ((PointingData[GETREADINDEX(point_index)].az/70.0)*65535.0); // if we overflow we go back to zero
  WriteData(gondAz, data, NIOS_QUEUE);
  i_point_read = GETREADINDEX(point_index);
}

