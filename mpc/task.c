/* fake.c: MAS-like stubs for fake MAS operation
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include "mpc.h"
#include "mputs.h"
#include <stdlib.h>
#include <unistd.h>

static const char *dt_name[] = { DT_STRINGS };
enum dtask data_tk = dt_idle;
int dt_error = 0;
int comms_lost = 0;

/* request a data tasklet and wait for it's completion.  returns dt_error */
static int dt_wait(enum dtask dt)
{
  dt_error = 0;
  bprintf(info, "DTreq: %s", dt_name[dt]);
  data_tk = dt;
  while (data_tk != dt_idle)
    usleep(10000);

  bprintf(info, "DTerr: %i", dt_error);

  return dt_error;
}

/* wait times after power cycle request (in seconds) */
static const int power_wait[] = { 10, 10, 20, 40, 60, 0 };

/* run generic tasks */
void *task(void *dummy)
{
  int cl_count = 0;

  nameThread("Task");

  /* wait for a task */
  for (;;) {
    if (comms_lost) { /* deal with no MCE communication */
      bprintf(warning, "Comms lost.");
      start_tk = stop_tk = 0xFFFF; /* interrupt! */

      /* kill acquisition, if it's running, but ignore errors */
      if (state & st_retdat) {
        dt_wait(dt_killacq);
        state &= ~st_retdat;
      }

      /* dsp reset */
      if (dt_wait(dt_dsprs)) {
        power_cycle_cmp = 1;
        sleep(60);
      }

      /* MCE reset */
      if (!dt_wait(dt_mcers)) {
        /* problem solved, I guess.  Get meta to return us to our scheduled
         * program */
        comms_lost = 0;
        cl_count = 0;
        state &= ~st_config; /* we're unconfigured */
        state |= st_mcecom; /* but communicating */
        start_tk = stop_tk = st_idle;
        continue;
      }

      /* ask for a MCE power cycle if this is the first time, and then at most
       * once per minute (the 40 seconds works here because we've already 
       * waited 20 seconds without asking) */
      if (cl_count == 0 || power_wait[cl_count] >= 40)
        power_cycle_mce = 1;

      /* wait */
      sleep(power_wait[cl_count]);
      if (power_wait[cl_count + 1])
        cl_count++;
      comms_lost = 0;
      state &= ~(st_config | st_mcecom); /* need complete MCE restart */
      start_tk = stop_tk = st_idle; /* try again */
    } else if (start_tk != st_idle) { /* handle start task requests */
      switch (start_tk) {
        case st_idle:
        case st_drive1: /* XXX */
        case st_drive2: /* XXX */
          break;
        case st_drive0:
          /* set directory */
          dt_wait(dt_setdir);

          /* do nothing for now */
          state |= st_drive0;
          start_tk = st_idle;
          break;
        case st_mcecom:
          /* DSP reset */
          if (dt_wait(dt_dsprs)) {
            power_cycle_cmp = 1;
            sleep(60);
          }
          /* MCE reset */
          if (dt_wait(dt_mcers)) {
            comms_lost = 1;
          } else {
            cl_count = 0;
            state |= st_mcecom;
            start_tk = st_idle;
          }
          break;
        case st_mcecmd:
          /* start mce_cmd */
          if (dt_wait(dt_mcecmd_init)) {
            ; /* is this even meaningful? */
          } else {
            state |= st_mcecmd;
            start_tk = st_idle;
          }
          break;
        case st_config:
          /* MCE reconfig */
          if (dt_wait(dt_reconfig)) {
            comms_lost = 1;
          } else {
            state |= st_config;
            start_tk = st_idle;
          }
          break;
        case st_retdat:
          /* start acq */
          if (dt_wait(dt_startacq)) {
            comms_lost = 1;
          } else {
            state |= st_retdat;
            start_tk = st_idle;
          }
      }
    } else if (stop_tk != st_idle) { /* handle stop task requests */
      /* XXX */
    }
    usleep(10000);
  }
}
