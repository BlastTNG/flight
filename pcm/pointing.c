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
#define GY2_OFFSET (-0.0182551)
#define GY3_OFFSET (0)

#define MAX_ISC_AGE 200

#define NSGF 11 // Number of points in the Savitsky-Golay polynomial fit.
#if 0
void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
    double *el);
void azel2radec(double *ra_out, double *dec_out,
    double az, double el, time_t lst, double lat);

double getlst(time_t t, double lon); // defined in starpos.c
double GetJulian(time_t t);
#endif

int point_index = 0;
struct PointingDataStruct PointingData[3];
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

/*****************************************************************
  do sensor selection;
  update the pointing;
  */
void Pointing(void)
{
//  double gy_roll, gy2, gy3, el_rad;
//  static double last_good_lat=0, last_good_lon=0;
  static int tst;

  static int firsttime = 1;
  static int initcount = 0;
  static double prevVel,prevTime;
  double curVel,curTime,avVel;
  double dt,dv;
  static struct NiosStruct* gondAz = NULL;
  static struct NiosStruct* dpspsGond = NULL;
  static struct NiosStruct* dpspsGondRough = NULL;
  struct timeval timer;

  // 11pt Cubic First Derivative Filter from Table 4 in 
  // Savitzky and Golay, 1964 
  double sgfilt[] ={300.0,-294.0,-532.0,-503.0,-296.0,0.0,296.0,503.0,532.0,294.0,-300.0};
  double sgnorm=5148.0;
  double sgdata[NSGF];
  double a1=0.0;
  int i_point_read=GETREADINDEX(point_index);
  int i;

  if (firsttime) {
    firsttime = 0;
    // the first t about to be read needs to be set
    PointingData[i_point_read].t = mcp_systime(NULL); // CPU time
    // initialize PointingData.az
    // lmf: for now assume that the initial position is zero.
    // TODO: Fix this later!
    PointingData[i_point_read].az = 0.0; 

    prevVel=ACSData.gyro2-GY2_OFFSET;
    gettimeofday(&timer, NULL);
    prevTime=(double)timer.tv_sec + timer.tv_usec/1000000.0;
    gondAz = GetNiosAddr("gond_az");
    dpspsGond=GetNiosAddr("dpsps_gond");      
    dpspsGondRough=GetNiosAddr("dpsps_gond_rough");      
  }
  curVel=ACSData.gyro2-GY2_OFFSET;
  gettimeofday(&timer, NULL);
  curTime=(double)timer.tv_sec + timer.tv_usec/1000000.0;
  avVel=(curVel+prevVel)/2.0;
  dt = (curTime - prevTime);
  dv=avVel * dt;
  PointingData[point_index].az = PointingData[i_point_read].az+dv; 
  
  int data = (int) ((PointingData[i_point_read].az/70.0)*65535.0); 
     // if we overflow we go back to zero
  WriteData(gondAz, data, NIOS_QUEUE);
  WriteData(dpspsGondRough, ((int)(((curVel-prevVel)/dt)/100.0*32767.0)),NIOS_QUEUE);

  //===================================================
  // Calculate the azimuth velocity using Savitsky-Golay
  // Convolution LS Fitting. 
  //===================================================
  if(initcount<NSGF)
    {
      sgdata[initcount]=ACSData.gyro2-GY2_OFFSET;
      initcount++;
      PointingData[point_index].dvdt=0.0;
    }
  else
    {
      // Increment the SF data vector
      for(i=0;i<(NSGF-1);i++)
	{
	  sgdata[i]=sgdata[i+1];
	}
      sgdata[NSGF-1]=ACSData.gyro2-GY2_OFFSET;
      a1=0.0;
      // Calculate the a1 factor.
      for(i=0;i<NSGF;i++)
	{
	  a1+=sgdata[i]*sgfilt[i]/sgnorm;
	} 
      //Derivative is the a1 factor divided by the step size
      // h=0.01 seconds.
      PointingData[point_index].dvdt=a1/0.01; 
   }
  WriteData(dpspsGond, ((int)((PointingData[GETREADINDEX(point_index)].dvdt)/100.0*32767.0)), NIOS_QUEUE);
  point_index=INC_INDEX(point_index);

  //  if (tst<100) {
  //     bprintf(info,"Pointing: curVel=%f, prevVel=%f, data=%i",curVel,prevVel,data); 
  //     bprintf(info,"Pointing: dt=%f, dv=%f, az=%f", dt,dv,PointingData[i_point_read].az);
  //    }
  prevVel=curVel;
  prevTime=curTime;
  //  tst=tst+1;
}

