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

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include "command_struct.h"
#include "pointing_struct.h"
#include "mcp.h"

#define MAX_LINE_LENGTH 1024
#define MAX_NSCHED 8000
struct ScheduleType _S[2][3];
void StarPos(double t, double ra0, double dec0, double mra, double mdec,
	     double pi, double rvel, double *ra, double *dec);
double GetJulian(time_t t);
void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
		double *el);
int pinIsIn();

#define NOMINAL_LATITUDE 67.82 /* degrees North */
#define LATITUDE_BAND     8.00 /* in degrees of latitude */
#define LATITUDE_OVERLAP  1.00 /* hysteresis between the bands,
                                  in degress of latitude */

/* in commands.c */
enum multiCommand MCommand(char*);
int MIndex(enum multiCommand);
enum singleCommand SCommand(char*);
void ScheduledCommand(struct ScheduleEvent*);

const char filename[2][3][32] = {
  {"/data/etc/happy.north.sch",
    "/data/etc/happy.mid.sch",
    "/data/etc/happy.south.sch"},
  {"/data/etc/sad.north.sch",
    "/data/etc/sad.mid.sch",
    "/data/etc/sad.south.sch"}
};

/***************************************************************************/
/*    GetLine: read non-comment line from file                             */
/*        The line is placed   in *line.                                   */
/*        Returns 1 if succesful, 0 if unsuccesful                         */
/***************************************************************************/
int GetLine(FILE *fp, char *line)
{
  char buffer[MAX_LINE_LENGTH];
  char *ret_val;
  int first_char;

  do {
    ret_val = fgets(buffer, MAX_LINE_LENGTH, fp);
    first_char = 0;
    while ((buffer[first_char] == ' ') || (buffer[first_char] == '\t'))
      first_char++;
    strncpy(line, &buffer[first_char], MAX_LINE_LENGTH);
  } while (((line[0] == '#') || (strlen(line) < 2)) && (ret_val != NULL));

  if (ret_val != NULL)
    return 1; /* a line was read */
  else
    return 0; /* there were no valid lines */
}

#define CHECK_LAT 67.49
#define CHECK_LON 50.00
void LoadSchedFile(const char* file, struct ScheduleType* S)
{
  FILE *fp;
  char line_in[MAX_LINE_LENGTH];
  char *token[MAX_N_PARAMS + 4];
  char *ptr;
  double hours;
  struct tm ts;
  double dt;
  double d_lon;
  int day, command, type;

  double az1, az2, el1, el2, height;

  int i, j, k, entry_ok;
  int n_fields, mindex;
  int el_range_warning;

  /*******************************************/
  /*** Count number of schedule file lines ***/
  if ((fp = fopen(file, "r")) == NULL) {
    berror(err, "Scheduler: Unable to open schedule file %s", file);
    S->n_sched = 0;
    return;
  }

  while (GetLine(fp, line_in))
    S->n_sched++;

  S->n_sched--; /* don't count date line */

  if (S->n_sched > MAX_NSCHED)
    S->n_sched = MAX_NSCHED;

  S->event = (struct ScheduleEvent*)balloc(fatal, S->n_sched *
      sizeof(struct ScheduleEvent));

  bprintf(info, "Scheduler: Found %i lines in schedule file %s\n", S->n_sched,
      file);

  if (fclose(fp) == EOF)
    berror(err, "Scheduler: Error on close");

  /**************************/
  /*** Read Starting Time ***/
  if ((fp = fopen(file, "r")) == NULL) {
    berror(err, "Scheduler: Unable to open schedule file");
    S->n_sched = 0;
    return;
  }

  GetLine(fp, line_in);
  sscanf(line_in, "%d-%d-%d %d:%d:%d", &(ts.tm_year), &(ts.tm_mon),
      &(ts.tm_mday), &(ts.tm_hour), &(ts.tm_min), &(ts.tm_sec));

  if (ts.tm_year < 50)
    ts.tm_year += 100;
  else
    ts.tm_year -= 1900;

  ts.tm_isdst = 0;
  ts.tm_mon--; /* Jan is 0 in struct tm.tm_mon, not 1 */

  S->t0 = mktime(&ts) - timezone; 

  /*************************************************************/
  /** find local comoving siderial date (in siderial seconds) **/
  dt = (mcp_systime(NULL) - S->t0) * 1.002737909; /* Ref Siderial Time */
  d_lon = CHECK_LON;
  while (d_lon < 0)
    d_lon += 360.0;
  while (d_lon >= 360.0)
    d_lon -= 360.0;
  dt -= (d_lon * 3600.00 * 24.00 / 360.0); /* add longitude correction */

  dt /= 3600.0;

  bprintf(sched,
      "Scheduler: ***********************************************************\n"
      "Scheduler: ***       Schedule File: %s\n"
      "Scheduler: *** Current local sid. date (hours relative to epoch) %g\n"
      "Scheduler: *** Assuming LAT = %g , LON = %g for checks\n", file, dt,
      CHECK_LAT, CHECK_LON);

  /***********************/
  /*** Read the events ***/
  for (i = j = 0; i < S->n_sched; i++) {
    entry_ok = 1;
    GetLine(fp, line_in);

    /* Tokenise the string */
    k = 0;
    n_fields = 1;
    ptr = line_in;
    token[0] = ptr;
    for (;;) {
      if (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r') {
        if (k) 
          *(ptr - 1) = '\0';
        else
          k = 1;
      } else {
        if (k) {
          *(ptr - 1) = '\0';
          token[n_fields++] = ptr;
        }
        k = 0;
      }
      if (*ptr == '\0' || *ptr == '#' || *ptr == '\n')
        break;
      ++ptr;
    }

    /* decrypt tag to find command */
    command = MCommand(token[0]);
    if (command == -1) {
      S->event[j].is_multi = 0;
      command = SCommand(token[0]);
      if (command == -1) {
        bprintf(sched, "Scheduler: ERROR: command not recognised: %s\n",
            token[0]);
        entry_ok = 0;
      }
    } else
      S->event[j].is_multi = 1;

    S->event[j].command = command;

    /* lst */
    if (n_fields < 3) {
      bprintf(sched, "Scheduler: ERROR: cannot find lst!\n");
      entry_ok = 0;
    } else {
      day = atoi(token[1]);
      hours = atof(token[2]);
      S->event[j].t = day * 24l * 3600l + hours * 3600l;
    }

    /* Parameters */
    if (entry_ok && S->event[j].is_multi) {
      mindex = MIndex(command);
      if (n_fields < 3 + mcommands[mindex].numparams) {
        bprintf(sched, "Scheduler: ERROR: insufficient parameters for command "
            "(wanted %i; got %i)\n", mcommands[mindex].numparams, n_fields - 3);
        entry_ok = 0;
      } else
        for (k = 0; k < mcommands[mindex].numparams; ++k) {
          type = mcommands[mindex].params[k].type;
          if (type == 'i')  /* 15 bit unsigned integer */
            S->event[j].ivalues[k] = atoi(token[k + 3]);
          else if (type == 'f')  /* 15 bit floating point */
            S->event[j].rvalues[k] = atof(token[k + 3]);
          else if (type == 'l') /* 30 bit floating point */
            S->event[j].rvalues[k] = atof(token[k + 3]);
        }
    }

    if (!entry_ok)
      bprintf(sched,
          "Scheduler: ****** Warning Line %i is Malformed: Skipping *****\n",
          i);
    else 
      j++;
  }

  for (i = 0; i < S->n_sched; i++) {
    if (S->event[i].command == box || S->event[i].command == vbox ||
        S->event[i].command == cap || S->event[i].command == vcap) {
      radec2azel(S->event[i].rvalues[0], S->event[i].rvalues[1], S->event[i].t,
          CHECK_LAT, &az1, &el1);
      if (i == S->n_sched - 1)
        radec2azel(S->event[i].rvalues[0], S->event[i].rvalues[1],
            S->event[i].t, CHECK_LAT, &az2, &el2);
      else
        radec2azel(S->event[i].rvalues[0], S->event[i].rvalues[1],
            S->event[i + 1].t, CHECK_LAT, &az2, &el2);

      height = S->event[i].rvalues[3];
      if (S->event[i].command == box || S->event[i].command == vbox)
        height /= 2;

      el_range_warning = 0;
      if (el1 > el2) {
        el1 += height;
        el2 -= height;
        if (el1 > 60.0)
          el_range_warning = 1;
        if (el2 < 25.0)
          el_range_warning = 1;
      } else {
        el1 -= height;
        el2 += height;
        if (el2 > 60.0)
          el_range_warning = 1;
        if (el1 < 25.0)
          el_range_warning = 1;
      }
      if (el_range_warning) {
        bputs(sched, "Scheduler: ******************************************\n"
            "Scheduler: *** Warning: El Range\n");
        bprintf(sched, "Scheduler: *** LST: %7.4f Ra: %8.3f  Dec: %8.3f\n",
            S->event[i].t / 3600.0, S->event[i].rvalues[0],
            S->event[i].rvalues[1]);
        bprintf(sched, "Scheduler: *** %3d LST: %7.4f Az: %8.3f - %8.3f El: "
            "%8.3f - %8.3f\n", i, S->event[i].t / 3600.0, az1, az2, el1, el2);
      }
    }
  }
  bputs(sched, "Scheduler: "
      "***********************************************************\n");

  if (fclose(fp) == EOF)
    berror(err, "Scheduler: Error on close");
}

/*********************************************************************/
/*            Init Sched Structure                                   */
/*********************************************************************/
void InitSched(void)
{
  int l, s;

  bprintf(info, "Scheduler: schedule file initialisation begins.");

  for (s = 0; s < 2; ++s)
    for (l = 0; l < 3; ++l)
      LoadSchedFile(filename[s][l], &_S[s][l]);
}

void DoSched(void) {
  time_t t;
  double dt;
  double d_lon;
  double d_lat;
  static int last_is = -1;
  static int last_s = -1;
  static int last_l = -1;
  int i_sched, i_point;
  int i_dgps;
  struct ScheduleType *S = &_S[CommandData.sucks][CommandData.lat_range];
  struct ScheduleEvent event;

  i_point = GETREADINDEX(point_index);
  d_lat = PointingData[i_point].lat - NOMINAL_LATITUDE;

  /* check our latitude band */
  if (CommandData.lat_range == 2) { /* southern band */
    if (d_lat > (LATITUDE_BAND / 2) - LATITUDE_OVERLAP) {
      bprintf(info, "Scheduler: Entering middle latitude band.\n");
      CommandData.lat_range = 1;
    }
  } else if (CommandData.lat_range == 1) { /* middle band */
    if (d_lat < -(LATITUDE_BAND / 2)) {
      bprintf(info, "Scheduler: Entering southern latitude band.\n");
      CommandData.lat_range = 2;
    } else if (d_lat > (LATITUDE_BAND / 2)) {
      bprintf(info, "Scheduler: Entering northern latitude band.\n");
      CommandData.lat_range = 0;
    }
  } else if (CommandData.lat_range == 0) { /* norhtern band */
    if (d_lat < -(LATITUDE_BAND / 2) + LATITUDE_OVERLAP) {
      bprintf(info, "Scheduler: Entering middle latitude band.\n");
      CommandData.lat_range = 1;
    }
  } else {
    bprintf(warning, "Scheduler: Unexpected latitude band: %i\n",
        CommandData.lat_range);
    CommandData.lat_range = 1;
  }

  S = &_S[CommandData.sucks][CommandData.lat_range];

  /* check to see if we've changed schedule files */
  if (last_l != CommandData.lat_range) {
    last_is = -1;
    last_l = CommandData.lat_range;
  }

  if (last_s != CommandData.sucks) {
    last_is = -1;
    last_s = CommandData.sucks;
  }

  /* no schedule file case */
  if (S->n_sched < 1)
    return;

  t = PointingData[i_point].t;
  if (t < CommandData.pointing_mode.t) {
    last_is = -1;
    return;
  }

  i_dgps = GETREADINDEX(dgpspos_index);
  if (DGPSPos[i_dgps].at_float)
    if (pinIsIn()) {
      bputs(info, "Scheduler: *** Executing initial float commands. ***\n");
      /* el on */
      event.command = el_on;
      ScheduledCommand(&event);
      /* unlock */
      event.command = unlock;
      event.is_multi = 0;
      ScheduledCommand(&event);
      /* az on */
      event.command = az_on;
      ScheduledCommand(&event);
      /* point antisolar */
      event.command = antisun;
      ScheduledCommand(&event);
      // out of sched mode for a while
      event.command = timeout;
      event.is_multi = 1;
      event.ivalues[0] = 600;
      ScheduledCommand(&event);
      return;
    }

  /*************************************************************/
  /** find local comoving siderial date (in siderial seconds) **/
  dt = (PointingData[i_point].t - S->t0) * 1.002737909; /*Ref Siderial Time */
  d_lon = PointingData[i_point].lon;
  while (d_lon < 0)
    d_lon += 360.0;
  while (d_lon >= 360.0)
    d_lon -= 360.0;
  dt -= ((d_lon) * 3600.00 * 24.00 / 360.0); /* add longitude correction */

  /******************/
  /** find i_sched **/
  i_sched = last_is;
  if (i_sched < 0)
    i_sched = 0;
  while ((i_sched < S->n_sched - 1) && (dt > S->event[i_sched].t))
    i_sched++;
  while ((i_sched > 0) && (dt < S->event[i_sched].t))
    i_sched--;

  /*******************************/
  /** Execute scheduled command **/
  dt /= 3600;
  if (i_sched != last_is) {
    bprintf(info, "Scheduler: Submitting event %i from %s to command "
        "subsystem (LST = %i/%f)", i_sched,
        filename[CommandData.sucks][CommandData.lat_range], (int)(dt / 24), 
        fmod(dt, 24));
    ScheduledCommand(&S->event[i_sched]);
  }
  last_is = i_sched;
}
