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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "pointing_struct.h"
#include "mcp.h"

#define GPSCOM "/dev/ttyS1"

struct DGPSAttStruct DGPSAtt[3];
int dgpsatt_index = 0;

struct DGPSPosStruct DGPSPos[3];
int dgpspos_index = 0;

time_t DGPSTime;

#define FLOAT_ALT 30480
#define FRAMES_TO_OK_ATFLOAT 100
#define LEAP_SECONDS 0

void SetGPSPort(speed_t speed) {
  int fd;

  struct termios term; 

  if ((fd = open(GPSCOM, O_RDWR)) < 0)
    merror(MCP_TFATAL, "Unable to open dgps serial port");

  if (tcgetattr(fd, &term))
    merror(MCP_TFATAL, "Unable to get dgps serial port attributes");

  term.c_iflag = 0;
  term.c_oflag = 0;
  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_cflag |= CS8;
  term.c_lflag = 0;

  if (cfsetispeed(&term, speed))
    merror(MCP_TFATAL, "error setting serial input speed");

  if (tcsetattr(fd, TCSANOW, &term))
    merror(MCP_TFATAL, "Unable to set serial attributes");

  close(fd);
}

/** grab the next field from a command delimeted list **/
/** outstr will hold the field and should be allocated by the **/
/** calling function **/
/** instr should be incremented by the return value to prepare for **/
/** the next call **/
int GetField(char *instr, char *outstr) {
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

void WatchDGPS() {
  FILE *fp;
  char instr[500], outstr[500];
  char *inptr;
  int d;
  float s,m;
  struct tm ts;
  int pos_ok;
  static int i_at_float = 0;

  pthread_setspecific(identity, "dgps");
  mputs(MCP_STARTUP, "WatchDGPS startup\n");

  DGPSAtt[0].az = 0;
  DGPSAtt[0].pitch = 0;
  DGPSAtt[0].roll = 0;
  DGPSAtt[0].att_ok = 0;
  dgpsatt_index = 1;

  DGPSPos[0].lat = 0;
  DGPSPos[0].lon = 0;
  DGPSPos[0].alt = 0;
  DGPSPos[0].at_float = 0;
  DGPSPos[0].speed = 0;
  DGPSPos[0].direction = 0;
  DGPSPos[0].climb = 0;
  DGPSPos[0].n_sat = 0;
  dgpspos_index = 1;

  DGPSTime = 0; 

  SetGPSPort(B9600);

  fp = fopen(GPSCOM, "r+");
  if (fp == NULL)
    merror(MCP_TFATAL, "error opening gps port for i/o");

  fprintf(fp,"$PASHS,SPD,B,7\r\n");

  fclose(fp);

  SetGPSPort(B38400);

  fp = fopen(GPSCOM, "r+");
  if (fp == NULL)
    merror(MCP_TFATAL, "error opening gps port for i/o");

  /* fprintf(fp,"$PASHS,RST\r\n");  // reset to defaults */
  /* sleep(10); */
  /***** THESE were set by MD/ 8/28/03 ******/
  fprintf(fp,"$PASHS,3DF,V12,+000.000,+003.239,-000.000\r\n");
  fprintf(fp,"$PASHS,3DF,V13,-001.254,+002.446,-000.024\r\n");
  fprintf(fp,"$PASHS,3DF,V14,+001.346,+000.560,-000.015\r\n");
  fprintf(fp,"$PASHS,3DF,OFS,-117.14,+00.00,+00.00\r\n"); // array offset p71
  fprintf(fp,"$PASHS,SAV,Y\r\n");
  /**********************************************/
  fprintf(fp,"$PASHS,ELM,15\r\n"); // minimum elevation for sattelite p 53
  fprintf(fp,"$PASHS,3DF,FLT,N\r\n"); // no averaging filter
  fprintf(fp,"$PASHS,3DF,ANG,3\r\n"); // max array tilt p 73
  fprintf(fp,"$PASHS,NME,ALL,B,OFF\r\n"); // turn off all messages
  fprintf(fp,"$PASHS,NME,PER,0\r\n");  	  // set to 2Hz messages
  fprintf(fp,"$PASHS,NME,ZDA,B,ON\r\n");  // turn on time message P109
  fprintf(fp,"$PASHS,NME,PAT,B,ON\r\n");   // turn on attitude/position P98
  fprintf(fp,"$PASHS,NME,POS,B,ON\r\n");   // turn on pos/vel message P99

  while (1) {
    fgets(instr, 499, fp);

    if (strncmp(instr, "$GPZDA", 6) == 0) { /* p 109 */
      sscanf(instr,"$GPZDA,%2d%2d%f,%d,%d,%d",&(ts.tm_hour),&(ts.tm_min),
          &s,&(ts.tm_mday),&(ts.tm_mon),&(ts.tm_year));
      ts.tm_sec = s;
      ts.tm_year -= 1900;

      ts.tm_isdst = 0;
      ts.tm_mon--; /* Jan is 0 in struct tm.tm_mon, not 1 */

      DGPSTime = mktime(&ts) - timezone + LEAP_SECONDS;
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
      // Pitch
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSAtt[dgpsatt_index].pitch));
      // Roll
      inptr += GetField(inptr,outstr);
      sscanf(outstr,"%lf", &(DGPSAtt[dgpsatt_index].roll));

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

      if (DGPSPos[dgpspos_index].n_sat <= 3) {
        pos_ok = 0;
      }

      if (pos_ok) {
        dgpspos_index = INC_INDEX(dgpspos_index);
        if (DGPSPos[dgpspos_index].alt < FLOAT_ALT) {
          DGPSPos[dgpspos_index].at_float = 0;
          i_at_float = 0;
        } else {
          i_at_float++;
          if (i_at_float > FRAMES_TO_OK_ATFLOAT) {
            DGPSPos[dgpspos_index].at_float = 1;
          } else {
            DGPSPos[dgpspos_index].at_float = 0;
          }
        }
      }
    }
  }
}
