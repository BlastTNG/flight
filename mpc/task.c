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
#include <sys/time.h>

static const char *dt_name[] = { DT_STRINGS };
enum dtask data_tk = dt_idle;
int dt_error = 0;
int comms_lost = 0;
int leech_veto = 0;
static int cl_count = 0;


/* request a data tasklet and wait for it's completion.  returns dt_error */
static int dt_wait(enum dtask dt)
{
  dt_error = 0;
  bprintf(info, "DTreq: %s", dt_name[dt]);
  data_tk = dt;
  while (data_tk != dt_idle)
    usleep(10000);

  if (dt_error)
    bprintf(warning, "DTerr: %i", dt_error);

  return dt_error;
}

/* wait times after power cycle request (in seconds) */
static const int power_wait[] = { 10, 10, 20, 40, 60, 0 };

/* do a DSP reset, MCE reset, fake stop and empty the data buffer */
static int task_reset_mce()
{
  /* DSP reset */
  if (dt_wait(dt_dsprs)) {
    power_cycle_cmp = 1;
    sleep(60);
    return 1;
  }
  /* MCE reset */
  if (dt_wait(dt_mcers)) {
    comms_lost = 1;
    return 1;
  }
  /* Fake stop */
  if (dt_wait(dt_fakestop)) {
    comms_lost = 1;
    return 1;
  }
  /* Empty the buffer */
  if (dt_wait(dt_empty)) {
    comms_lost = 1;
    return 1;
  }

  cl_count = 0;
  state |= st_mcecom;
  state &= ~(st_config | st_acqcnf | st_retdat | st_tuning);
  return 0;
}

/* run generic tasks */
void *task(void *dummy)
{
  int repc = 0;
  int check_acq = 0;
  nameThread("Task");

  /* wait for a task */
  for (;;) {
    if (comms_lost) { /* deal with no MCE communication */
      bprintf(warning, "Comms lost.");
      start_tk = stop_tk = 0xFFFF; /* interrupt! */

      /* stop the MCE and reset it */
      if (task_reset_mce() == 0) {
        /* problem solved, I guess.  Get meta to return us to our scheduled
         * program */
        comms_lost = 0;
        check_acq = 0;
        cl_count = 0;
        start_tk = stop_tk = st_idle;
        state &= ~(st_config | st_acqcnf | st_mcecom | st_retdat);
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
      check_acq = 0;
      /* need complete MCE restart */
      state &= ~(st_config | st_acqcnf | st_mcecom | st_retdat);
      start_tk = stop_tk = st_idle; /* try again */
    } else if (req_dm != cur_dm && state & st_acqcnf) {
      /* change of data mode while acquiring -- restart acq */
      task_reset_mce();
    } else if (start_tk != st_idle) { /* handle start task requests */
      switch (start_tk) {
        case st_idle:
          break;
        case st_drive0:
          /* set directory */
          dt_wait(dt_setdir);

          /* do nothing for now */
          state |= st_drive0;
          start_tk = st_idle;
          break;
        case st_mcecom:
          task_reset_mce();
          start_tk = st_idle;
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
        case st_acqcnf:
          /* "mce status" */
          if (dt_wait(dt_status)) {
            comms_lost = 1;
          } else {
            state |= st_acqcnf;
            start_tk = st_idle;
          }
          /* acq config */
          if (dt_wait(dt_acqcnf)) {
            comms_lost = 1;
          } else {
            state |= st_acqcnf;
            start_tk = st_idle;
          }
          break;
        case st_retdat:
          if (check_acq) { /* probably means an acquisition immediately
                            terminated -- try a reset */
            comms_lost = 1;
            check_acq = 0;
            continue;
          } 
          check_acq = 1;
          /* start acq */
          if (dt_wait(dt_startacq)) {
            comms_lost = 1;
          } else {
            state |= st_retdat;
            start_tk = st_idle;
          }
          break;
        case st_tuning:
          /* auto_setup */
          state |= st_tuning;
          if (dt_wait(dt_autosetup)) {
            comms_lost = 1; /* hmm... */
          } else {
            /* done with tuning; back to acq */
            goal = op_acq;
            start_tk = st_idle;
          }
      }
    } else if (stop_tk != st_idle) { /* handle stop task requests */
      switch (stop_tk) {
        case st_idle:
          break;
        case st_acqcnf:
        case st_retdat:
        case st_tuning:
          /* sledgehammer based stop acq */
          task_reset_mce();
          stop_tk = st_idle;
          break;
      }
    }
    usleep(10000);

    if (repc++ > 100) {
      if (state & st_retdat) {
        if (check_acq && rd_count > 20) {
          bprintf(info, "start of acquisition detected");
          check_acq = 0;
        } else if (!check_acq) {
          if (rd_count < 1) {
            bprintf(info, "loss of acquisition detected");
            comms_lost = 1;
          }
        }
        rd_count = 0;
      }
      repc = 0;
    }
  }
}
