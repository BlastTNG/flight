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

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include "command_struct.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "angles.h"

#define MAX_NSCHED 8000
struct ScheduleType _S[2][3];

struct ScheduleType *Uplink_S = 0;

int doing_schedule = 0;
unsigned int sched_lst = 0;

void StarPos(double t, double ra0, double dec0, double mra, double mdec,
	     double pi, double rvel, double *ra, double *dec);
double GetJulian(time_t t);
void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
		double *el);

char lst0str[82];

// Toronto Highbay
// #define CHECK_LON -79.3994
// #define NOMINAL_LATITUDE 44.6601
// McMurdo (LDB)
#define CHECK_LON 167.0592
#define NOMINAL_LATITUDE -77.8616
// Palestine (CSBF)
// #define CHECK_LON -95.7168
// #define NOMINAL_LATITUDE 31.7799

#define LATITUDE_BAND     6.00 /* in degrees of latitude */
#define LATITUDE_OVERLAP  1.00 /* hysteresis between the bands, in deg lat */

/* in commands.c */
enum multiCommand MCommand(char*);
int MIndex(enum multiCommand);
enum singleCommand SCommand(char*);
void ScheduledCommand(struct ScheduleEvent*);
const char* CommandName(int, int);

static const char filename[2][3][32] = {
  {"/data/etc/blast/happy.north.sch",
    "/data/etc/blast/happy.mid.sch",
    "/data/etc/blast/happy.south.sch"},
  {"/data/etc/blast/sad.north.sch",
    "/data/etc/blast/sad.mid.sch",
    "/data/etc/blast/sad.south.sch"}
};

int GetLine(FILE *fp, char *line); // defined in lut.c

static void LoadSchedFile(const char* file, struct ScheduleType* S, int lband)
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
  double check_lat = NOMINAL_LATITUDE + lband * (LATITUDE_BAND
      - LATITUDE_OVERLAP);

  double az1, az2, el1, el2, height;

  int i, j, k, entry_ok;
  int n_fields, mindex;
  int el_range_warning;
  int discarded_lines;

  blast_sched("********************************************\n"
          "*** Schedule: %s\n", file);
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

  if (S->n_sched > MAX_NSCHED) {
    berror(err, "Scheduler: schedule '%s' has too many commands", file);
    S->n_sched = MAX_NSCHED;
  }

  S->event = (struct ScheduleEvent*)balloc(fatal, S->n_sched *
      sizeof(struct ScheduleEvent));

  blast_sched("*** Lines: %i\n", S->n_sched);

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

  strncpy(lst0str, line_in, 80);

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
  /** find local comoving siderial date (in siderial seconds) **/ // 166 east lon
  dt = (mcp_systime(NULL) - S->t0) * 1.002737909; /* Ref Siderial Time */
  d_lon = CHECK_LON;
  while (d_lon < 0)
    d_lon += 360.0;
  while (d_lon >= 360.0)
    d_lon -= 360.0;
  dt -= (d_lon * 3600.00 * 24.00 / 360.0); /* add longitude correction */

  dt /= 3600.0;

  if (lband > -10) {
    blast_sched("*** Current LST (hours) %g\n"  // relative to schedule epoch
          "*** For checks: LAT=%g, LON=%g\n", dt, check_lat, CHECK_LON);
  }

  /***********************/
  /*** Read the events ***/
  discarded_lines = 0;
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
        blast_sched("Scheduler: *** ERROR: command not recognised: %s\n",
            token[0]);
        entry_ok = 0;
      }
    } else {
      S->event[j].is_multi = 1;
    }
    S->event[j].command = command;

    /* lst */
    if (n_fields < 3) {
      blast_sched("Scheduler: *** ERROR: cannot find lst!\n");
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
        blast_sched("Scheduler: *** ERROR: insufficient parameters for "
            "command (wanted %i; got %i)\n", mcommands[mindex].numparams,
            n_fields - 3);
        entry_ok = 0;
      } else {
        for (k = 0; k < mcommands[mindex].numparams; ++k) {
          type = mcommands[mindex].params[k].type;
          if (type == 'i')  /* 15 bit unsigned integer */
            S->event[j].ivalues[k] = atoi(token[k + 3]);
          else if (type == 'l')  /* 30 bit unsigned integer */
            S->event[j].ivalues[k] = atoi(token[k + 3]);
          else if (type == 'f')  /* 15 bit floating point */
            S->event[j].rvalues[k] = atof(token[k + 3]);
          else if (type == 'd') /* 30 bit floating point */
            S->event[j].rvalues[k] = atof(token[k + 3]);
        }
      }
    }

    if (!entry_ok) {
      blast_sched("Scheduler: ****** Warning Line %i is Malformed: Skipping *****\n", i);
      discarded_lines++;
    } else {
      j++;
    }
  }
  S->n_sched -= discarded_lines;

  if (lband > -10) {
    for (i = 0; i < S->n_sched; i++) {
      if (S->event[i].command == box || S->event[i].command == vbox ||
        S->event[i].command == cap || S->event[i].command == vcap) {
        equatorial_to_horizontal(S->event[i].rvalues[0], S->event[i].rvalues[1], S->event[i].t,
                   check_lat, &az1, &el1);
//        radec2azel(S->event[i].rvalues[0], S->event[i].rvalues[1], S->event[i].t,
//                   check_lat, &az1, &el1);
        if (i == S->n_sched - 1)
          equatorial_to_horizontal(S->event[i].rvalues[0], S->event[i].rvalues[1],
                     S->event[i].t, check_lat, &az2, &el2);
//          radec2azel(S->event[i].rvalues[0], S->event[i].rvalues[1],
//                     S->event[i].t, check_lat, &az2, &el2);
          else
            equatorial_to_horizontal(S->event[i].rvalues[0], S->event[i].rvalues[1],
                       S->event[i + 1].t, check_lat, &az2, &el2);
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
          if (el_range_warning && 0) { // FIXME: reenable
            blast_sched("Scheduler: ******************************************\n"
                           "Scheduler: *** Warning: El Range of Event %i (%s)\n", i,
                    CommandName(S->event[i].is_multi, S->event[i].command));
            blast_sched("Scheduler: *** LST: %i/%7.4f Ra: %8.3f  Dec: %8.3f\n",
                    (int)(S->event[i].t / 86400), fmod(S->event[i].t / 3600.0, 24),
                    S->event[i].rvalues[0], S->event[i].rvalues[1]);
            blast_sched("Scheduler: *** LST: %7.4f Az: %8.3f - %8.3f El: "
                           "%8.3f - %8.3f\n", S->event[i].t / 3600.0, az1, az2, el1, el2);
          }
        }
    }
    bputs(sched, "********************************************\n");
  }
  if (discarded_lines)
    blast_warn("Discarded %i malformed lines from schedule file.",
        discarded_lines);

  if (fclose(fp) == EOF)
    berror(err, "Scheduler: Error on close");
}

// load uplink file
//   load into a 2 element cirular buffer
//   set Uplink_S to new one once loaded (not before).
//   return 1 on success
int LoadUplinkFile(int slot) {
  char filename[27];
  FILE *fp;
  static struct ScheduleType S[2];
  static int i_s = -1; // which S are we about to fill

  if (i_s < 0) {
    S[0].n_sched = S[1].n_sched = 0;
    S[0].t0 = S[1].t0 = 0;
    S[0].event = S[1].event = NULL;
    i_s = 0;
  }

  // free the schedule
  if (S[i_s].n_sched > 0) {
    free(S[i_s].event);
    S[i_s].n_sched = 0;
    S[i_s].event = NULL;
  }

  // check to make sure the file exists
  snprintf(filename, sizeof(slot), "/data/etc/blast/%d.sch", slot);
  fp = fopen(filename, "r");
  if (fp == NULL) {
    return(0);
  } else {
    fclose(fp);
  }

  LoadSchedFile(filename, &(S[i_s]), -10);

  blast_info("Scheduler: Read %i lines in schedule file %s\n", S[i_s].n_sched, filename);
  if (S[i_s].n_sched > 2) { // we read something
    Uplink_S = &(S[i_s]);
    if (i_s == 0) {
      i_s = 1;
    } else {
      i_s = 0;
    }
  } else {
    return(0);
  }

  return (1);
}

/*********************************************************************/
/*            Init Sched Structure                                   */
/*********************************************************************/
void InitSched(void)
{
  int l, s;

  blast_info("Scheduler: schedule file initialisation begins.");

  for (s = 0; s < 2; ++s) {
    for (l = 0; l < 3; ++l) {
      LoadSchedFile(filename[s][l], &_S[s][l], 1 - l);
    }
  }

  if (!LoadUplinkFile(CommandData.slot_sched)) {
    CommandData.uplink_sched = 0;
  }
}

void DoSched(void)
{
  time_t t;
  double dt;
  double d_lon;
  double d_lat;
  static int last_is = -1;
  static int last_s = -1;
  static int last_l = -1;
  static int last_slot = -1;
  static int last_up = -1;
  int i_sched, i_point;
  int i, index;
  struct ScheduleType *S = &_S[CommandData.sucks][CommandData.lat_range];
  struct ScheduleEvent event;
  static int first_time = 1;

  if (first_time) {
    blast_info("Calling DoSched for the First Time");
    first_time = 0;
  }
  i_point = GETREADINDEX(point_index);
  d_lat = PointingData[i_point].lat - NOMINAL_LATITUDE;

  if (last_up != CommandData.uplink_sched) {
    last_up = CommandData.uplink_sched;
    last_is = -1;
  }

  // Check for invalid uplinked schedule file
  if (CommandData.uplink_sched) {
    if (Uplink_S == 0) {
      CommandData.uplink_sched = 0;
    } else if (Uplink_S->n_sched < 2) {
      CommandData.uplink_sched = 0;
    }
  }

  if (CommandData.uplink_sched) {
    if (last_slot != CommandData.slot_sched) {
      last_slot = CommandData.slot_sched;
      last_is = -1;
    }
    S = Uplink_S;
  } else {
    /* check our latitude band */
    if (CommandData.lat_range == 2) { /* southern band */
      if (d_lat > -(LATITUDE_BAND / 2) + LATITUDE_OVERLAP) {
        blast_info("Scheduler: Entering middle latitude band. (%g)\n", d_lat);
        CommandData.lat_range = 1;
      }
    } else if (CommandData.lat_range == 1) { /* middle band */
      if (d_lat < -(LATITUDE_BAND / 2)) {
        blast_info("Scheduler: Entering southern latitude band. (%g)\n", d_lat);
        CommandData.lat_range = 2;
      } else if (d_lat > (LATITUDE_BAND / 2)) {
        blast_info("Scheduler: Entering northern latitude band. (%g)\n", d_lat);
        CommandData.lat_range = 0;
      }
    } else if (CommandData.lat_range == 0) { /* northern band */
      if (d_lat < (LATITUDE_BAND / 2) - LATITUDE_OVERLAP) {
        blast_info("Scheduler: Entering middle latitude band. (%g)\n", d_lat);
        CommandData.lat_range = 1;
      }
    } else {
      blast_warn("Scheduler: Unexpected latitude band: %i\n",
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
  }

  /* no schedule file case */
  if (S->n_sched < 1) {
    doing_schedule = 0;
    return;
  }

/*************************************************************/
  /** find local comoving siderial date (in siderial seconds) **/
  dt = (PointingData[i_point].t - S->t0) * 1.002737909; /*Ref Siderial Time */
  d_lon = PointingData[i_point].lon;

  while (d_lon < -170)
    d_lon += 360.0;
  while (d_lon >= 190.0)
    d_lon -= 360.0;

  dt -= ((d_lon) * 3600.00 * 24.00 / 360.0); /* add longitude correction */

  sched_lst = dt;

  t = PointingData[i_point].t;
  if (t < CommandData.pointing_mode.t) {
    doing_schedule = 0;
    last_is = -1;
    return;
  }

  if (PointingData[i_point].at_float && !CommandData.at_float) {
    bputs(info, "Scheduler: *** Executing initial float commands. ***\n");
    /* el on */
    event.command = el_on;
    event.is_multi = 0;
    ScheduledCommand(&event);
    /* unlock */
    event.command = unlock;
    ScheduledCommand(&event);
    /* az on */
    event.command = az_on;
    ScheduledCommand(&event);
    /* point antisolar */
    event.command = antisun;
    ScheduledCommand(&event);
    /* enable hwpr autostepping */
    event.command = hwpr_step_on;
    ScheduledCommand(&event);
    /* potvalve_open */
    event.command = potvalve_open;
    ScheduledCommand(&event);
    event.command = potvalve_on;
    ScheduledCommand(&event);
    /* turn off lock motor hold current */
    event.command = lock_i;
    event.is_multi = 1;
    event.ivalues[0] = 50;
    event.ivalues[1] = 0;
    ScheduledCommand(&event);
    /* activate fridge autocycle system */
    event.command = allow_cycle;
    event.is_multi = 0;
    ScheduledCommand(&event);

    // out of sched mode for a while
    CommandData.pointing_mode.t = t + 30;
    doing_schedule = 0;

    /* Remember we're at float */
    CommandData.at_float = 1;
    bputs(info, "Scheduler: *** Initial float commands complete. ***\n");
    return;
  }

  /******************/
  /** find i_sched **/
  i_sched = last_is;
  if (i_sched < 0)
    i_sched = 0;
  while ((i_sched < S->n_sched - 1) && (dt > S->event[i_sched].t))
    i_sched++;
  while ((i_sched > 0) && (dt < S->event[i_sched].t))
    i_sched--;

  /******************************/
  /** Execute initial controls **/
  if (!doing_schedule) {
    bputs(info, "Scheduler: *** Entering schedule file mode.  ***\n"
        "Scheduler: *** Running initial schedule controls.  ***\n");
    /* bias fixed */
    event.command = fixed;
    event.is_multi = 0;
    // ScheduledCommand(&event);
    /* cal repeat */
    bputs(info, "Scheduler: *** Scheduling a cal_repeat...disabled in mcp for now ***\n");
    // TODO(laura): Why did we start with a cal repeater?  Is there an equivalent command for TNG?
//     event.command = cal_repeat;
//     event.is_multi = 1;
//     event.ivalues[0] = 130; /* ms */
//     event.ivalues[1] = 600; /* s */
    // ScheduledCommand(&event);

    bputs(info, "Scheduler: *** Searching for current pointing mode. ***\n");

    /* find last pointing command */
    for (i = i_sched; i >= 0; --i)
      if (S->event[i].is_multi) {
        index = MIndex(S->event[i].command);
        if (mcommands[index].group & GR_POINT)
          break;
      } else {
        index = SIndex(S->event[i].command);
        if (scommands[index].group & GR_POINT)
          break;
      }

    if (i == -1) {
      bputs(warning,
          "Scheduler: *** No previous pointing mode, turning antisolar. ***\n");
      event.command = antisun;
      event.is_multi = 0;
      ScheduledCommand(&event);
    } else if (i == i_sched) {
      bputs(info, "Scheduler: *** Pointing mode change imminent. ***\n");
    } else {
      blast_info("Scheduler: *** Recovering scheduled pointing mode (event %i). ***\n", i);
      ScheduledCommand(&S->event[i]);
    }
  }
  doing_schedule = 1;

  /*******************************/
  /** Execute scheduled command **/
  dt /= 3600;
  if (i_sched != last_is) {
    blast_info("Scheduler: Submitting event %i from %s to command "
        "subsystem (LST = %i/%f)", i_sched,
        filename[CommandData.sucks][CommandData.lat_range], (int)(dt / 24),
        fmod(dt, 24));
    ScheduledCommand(&S->event[i_sched]);
  }
  last_is = i_sched;
  first_time = 0;
}
