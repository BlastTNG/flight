/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2003-2006 University of Toronto
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "pointing_struct.h"
#include "mcp.h"

#define NTPD_BASE 0x4e545030 //shared memory segment key

#define GPSCOM "/dev/ttySI5"

extern short int InCharge;

#define DGPS_ALIGNMENT    90.0 // rotates in positive azimuth

void nameThread(const char*);	/* mcp.c */

struct shmTime {
  int	  mode;
  int	  count;
  time_t  clockTimeStampSec; /* external clock */
  int	  clockTimeStampUsec;
  time_t  receiveTimeStampSec; /* internal clock, when external value was received */
  int	  receiveTimeStampUsec;
  int	  leap;
  int	  precision;
  int	  nsamples;
  int	  valid;
  int	  dummy[10];
};

struct DGPSAttStruct DGPSAtt[3];
int dgpsatt_index = 0;

struct DGPSPosStruct DGPSPos[3];
int dgpspos_index = 0;

time_t DGPSTime;

#define LEAP_SECONDS 0

static int SetGPSPort(speed_t speed)
{
  int fd;

  struct termios term;

  if ((fd = open(GPSCOM, O_RDWR)) < 0) {
    return 0;
    // berror(tfatal, "dGPS: Unable to open dgps serial port");
  }

  if (tcgetattr(fd, &term)) {
    return 0;
    //berror(tfatal, "dGPS: Unable to get dgps serial port attributes");
  }

  term.c_iflag = 0;
  term.c_oflag = 0;
  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_cflag |= CS8;
  term.c_lflag = 0;

  if (cfsetispeed(&term, speed)) {
    return 0;
    //berror(tfatal, "dGPS: error setting serial input speed");
  }

  if (tcsetattr(fd, TCSANOW, &term)) {
    return 0;
    berror(tfatal, "dGPS: Unable to set serial attributes");
  }

  close(fd);
  return 1;
}

/** grab the next field from a command delimeted list **/
/** outstr will hold the field and should be allocated by the **/
/** calling function **/
/** instr should be incremented by the return value to prepare for **/
/** the next call **/
static int GetField(char *instr, char *outstr)
{
  int i = 0;

  while ((instr[i] != ',') && (instr[i] != '\0') && (instr[i] != '*')) {
    outstr[i] = instr[i];
    i++;
  }
  outstr[i] = '\0';

  if (instr[i] == '\0')
    return i;
  else
    return i + 1;
}

struct shmTime *getShmTime() {
  int shmid;
  if ((shmid = shmget (NTPD_BASE, sizeof (struct shmTime), IPC_CREAT|0660))== -1) {
    return 0;
  } else {
    struct shmTime *p=(struct shmTime *)shmat (shmid, 0, 0);
    if ((int)(long)p==-1) {
      return 0;
    }
    return p;
  }
}

int ntpshm_put(double fixtime) {
  static struct shmTime *shmTime = NULL;
  struct timeval tv;
  double seconds, microseconds;
  (void)gettimeofday(&tv,NULL);
  microseconds = 1000000.0 * modf(fixtime,&seconds);

  if (shmTime == NULL) {
    shmTime = getShmTime();
  }

  shmTime->valid =0;
  shmTime->count++;
  shmTime->clockTimeStampSec = (time_t)seconds;
  shmTime->clockTimeStampUsec = (int)microseconds;
  shmTime->receiveTimeStampSec = (time_t)tv.tv_sec;
  shmTime->receiveTimeStampUsec = (int)tv.tv_usec;
  shmTime->count++;
  shmTime->valid = 1;

  return 1;

}


void WatchDGPS()
{
  FILE *fp = NULL;
  char instr[500], outstr[500];
  char *inptr;
  int d;
  float s,m;
  struct tm ts;
  int pos_ok;
  int serial_error = 0;
  int serial_set = 0;

  nameThread("dGPS");
  bputs(startup, "dGPS: WatchDGPS startup\n");

  //sleep (12); 
  
  while (!InCharge) {  // wait to be the boss to open the port!
    usleep(10000);
  }
  

  DGPSAtt[0].az = 0;
  DGPSAtt[0].pitch = 0;
  DGPSAtt[0].roll = 0;
  DGPSAtt[0].att_ok = 0;
  dgpsatt_index = 1;

  DGPSPos[0].lat = 0;
  DGPSPos[0].lon = 0;
  DGPSPos[0].alt = 0;
  DGPSPos[0].speed = 0;
  DGPSPos[0].direction = 0;
  DGPSPos[0].climb = 0;
  DGPSPos[0].n_sat = 0;
  dgpspos_index = 1;

  DGPSTime = 0;

#if 0
  /************************ Set serial ports: *******************/
  /* Set our serial port to 9600 (the dGPS default, then
   * tell the dgps to set its port B to 38400, then set our port
   * to 38400 */
  SetGPSPort(B9600);

  fp = fopen(GPSCOM, "r+");
  if (fp == NULL)
    berror(tfatal, "dGPS: error opening gps port for i/o");

  fprintf(fp,"$PASHS,SPD,B,7\r\n"); // 38400 Pg 66

  fclose(fp);

  SetGPSPort(B115200);

  fp = fopen(GPSCOM, "r+");
  if (fp == NULL)
    berror(tfatal, "dGPS: error opening gps port for i/o");

  fprintf(fp,"$PASHS,SPD,B,7\r\n"); // 38400 Pg 66

  fclose(fp);
#endif 

  do {
    serial_set = SetGPSPort(B38400);

    if (serial_set){
      fp = fopen(GPSCOM, "r+");
    }
    
    if (fp == NULL) {
      if (!serial_error) {
        blast_info("error opening gps serial port.  Will silently try again every 1s.");
        serial_error = 1;
      }
      sleep(1);
    }
  } while (fp == NULL);
  
#if 0
  /****************** Set up Port A for ntp output *********/
  /* see file:/usr/share/doc/ntp-4.2.0-r2/html/drivers/driver20.html */
  fprintf(fp,"$PASHS,SPD,A,4\r\n"); // 4800 baud Pg66
  fprintf(fp,"$PASHS,NME,ALL,A,OFF\r\n"); // turn off all messages
  fprintf(fp,"$PASHS,NME,GLL,A,ON\r\n"); // turn on GLL Pg89
  //fprintf(fp,"$PASHS,NME,ZDA,A,ON\r\n");  // turn on time message P109
  //fprintf(fp,"$PASHS,NME,PER,1\r\n"); // NEMA period = 1s

  //fprintf(fp,"$PASHS,RST\r\n");  // reset to defaults
  //sleep(10);

  /********* Set up array dimention and phase shifts *********/

  /****** These numbers are for Kiruna ********/
  //fprintf(fp,"$PASHS,3DF,V12,+000.000,+001.566,+000.089\r\n");
  //fprintf(fp,"$PASHS,3DF,V13,+002.983,-000.691,+000.011\r\n");
  //fprintf(fp,"$PASHS,3DF,V14,+003.696,+000.740,+000.069\r\n");
  //fprintf(fp,"$PASHS,3DF,OFS,-168.70,+00.00,+00.00\r\n"); // array offset p71

  /***** Numbers for palestine, 2006-AUG-01 *********/
  //fprintf(fp,"$PASHS,3DF,V12,+000.000,+001.153,+000.050\r\n");
  //fprintf(fp,"$PASHS,3DF,V13,-001.783,+001.825,+000.014\r\n");
  //fprintf(fp,"$PASHS,3DF,V14,-002.546,+000.958,-000.033\r\n");
  //fprintf(fp,"$PASHS,3DF,OFS,-168.70,+00.00,+00.00\r\n"); // array offset p71
  //fprintf(fp,"$PASHS,SAV,Y\r\n");
  /***** Numbers for McMurdo, 2006-Nov-27 *********/
  fprintf(fp,"$PASHS,3DF,V12,+000.000,+003.130,+000.000\r\n");
  fprintf(fp,"$PASHS,3DF,V13,-001.237,+000.624,+000.000\r\n");
  fprintf(fp,"$PASHS,3DF,V14,+001.284,+002.474,+000.009\r\n");
  //fprintf(fp,"$PASHS,3DF,OFS,-64.39,+00.00,+00.00\r\n"); // array offset p71
  fprintf(fp,"$PASHS,3DF,OFS,-64.002,+00.00,+00.00\r\n"); // adjusted based on SC
  fprintf(fp,"$PASHS,SAV,Y\r\n");
  /**********************************************/
  fprintf(fp,"$PASHS,ELM,15\r\n"); // minimum elevation for sattelite p 53
  fprintf(fp,"$PASHS,3DF,FLT,N\r\n"); // no averaging filter
  fprintf(fp,"$PASHS,3DF,ANG,3\r\n"); // max array tilt p 73
  fprintf(fp,"$PASHS,NME,ALL,B,OFF\r\n"); // turn off all messages on B
  fprintf(fp,"$PASHS,NME,PER,0\r\n");  	  // set to 2Hz messages
  fprintf(fp,"$PASHS,POS,0\r\n");
  fprintf(fp,"$PASHS,NME,ZDA,B,ON\r\n");  // turn on time message P109
  fprintf(fp,"$PASHS,NME,PAT,B,ON\r\n");   // turn on attitude/position P98
  fprintf(fp,"$PASHS,NME,POS,B,ON\r\n");   // turn on pos/vel message P99
#endif

  berror(info, "dGPS: GPS initialised");

  while (1) {
    if (fgets(instr, 499, fp) == NULL)
      berror(tfatal, "dGPS: error getting input string");
    if (strncmp(instr, "$GPZDA", 6) == 0) { /* p 109 */
      sscanf(instr,"$GPZDA,%2d%2d%f,%d,%d,%d",&(ts.tm_hour),&(ts.tm_min),
          &s,&(ts.tm_mday),&(ts.tm_mon),&(ts.tm_year));
      ts.tm_sec = s;
      ts.tm_year -= 1900;

      ts.tm_isdst = 0;
      ts.tm_mon--; /* Jan is 0 in struct tm.tm_mon, not 1 */

      //DGPSTime = mktime(&ts) - timezone + LEAP_SECONDS;
      DGPSTime = timegm(&ts) + LEAP_SECONDS;
      ntpshm_put((double)DGPSTime); //segfault unless run mcp as sudo
    } else if (strncmp(instr, "$GPPAT",6) == 0) { /* position & attitude:
                                                   * Page 98 */
      inptr = instr + 7;

      // Time field: skip it
      inptr += GetField(inptr,outstr);
      // Skip Latitude
      inptr += GetField(inptr,outstr);
      inptr += GetField(inptr,outstr);
      // Skip Longitude
      inptr += GetField(inptr,outstr);
      inptr += GetField(inptr,outstr);
      // Skip Altitude
      inptr += GetField(inptr,outstr);
      // Az (heading)
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSAtt[dgpsatt_index].az));
      DGPSAtt[dgpsatt_index].az-= DGPS_ALIGNMENT;
      //2012: CSBF gps array mounted 90 deg off, so their pitch is our -roll, and their roll is our pitch
      // Pitch
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSAtt[dgpsatt_index].roll));
      DGPSAtt[dgpsatt_index].roll *= -1.0;
      // Roll
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSAtt[dgpsatt_index].pitch));

      // skip phase error and baseline error
      inptr += GetField(inptr,outstr);
      inptr += GetField(inptr,outstr);

      // Attitude Valid flag
      inptr += GetField(inptr,outstr);
      if (outstr[0] == '0') {
        DGPSAtt[dgpsatt_index].att_ok = 1;
      } else {
        DGPSAtt[dgpsatt_index].att_ok = 0;
      }
      dgpsatt_index = INC_INDEX(dgpsatt_index);
    } else if (strncmp(instr, "$PASHR,POS",10) == 0) { // speed and position p99
      pos_ok = 1;
      inptr = instr + 7;

      // skip type
      inptr += GetField(inptr,outstr);
      inptr += GetField(inptr,outstr);

      // # Sattelites
      inptr += GetField(inptr,outstr);
      if (sscanf(outstr,"%d", &(DGPSPos[dgpspos_index].n_sat)) != 1) {
        pos_ok = 0;
      } else if (DGPSPos[dgpspos_index].n_sat < 4) {
        pos_ok = 0;
      }

      // skip time
      inptr += GetField(inptr,outstr);

      // Latitude
      inptr += GetField(inptr,outstr);
      if (sscanf(outstr,"%2d%f", &d,&m) != 2) {
        pos_ok = 0;
      } else {
        DGPSPos[dgpspos_index].lat = (double)d + (double) m * (1.0 / 60.0);
      }

      // North/South
      inptr += GetField(inptr,outstr);
      if (outstr[0] == 'S')
        DGPSPos[dgpspos_index].lat *=-1;

      // Longitude
      inptr += GetField(inptr,outstr);
      if (sscanf(outstr,"%3d%f", &d,&m) !=  2) {
        pos_ok = 0;
      } else {
        DGPSPos[dgpspos_index].lon = (double)d + (double) m * (1.0 / 60.0);
      }
      // East/West
      inptr += GetField(inptr,outstr);
      if (outstr[0] == 'E')
        DGPSPos[dgpspos_index].lon *=-1;

      // Altitude
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSPos[dgpspos_index].alt));

      // skip
      inptr += GetField(inptr,outstr);

      // Direction
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSPos[dgpspos_index].direction));

      // speed in knots
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSPos[dgpspos_index].speed));

      // rate of climb in m/s
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSPos[dgpspos_index].climb));

      if (DGPSPos[dgpspos_index].n_sat <= 3)
        pos_ok = 0;

      if (pos_ok)
        dgpspos_index = INC_INDEX(dgpspos_index);
    }
  }
}
