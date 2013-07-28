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
/* kill switch */
int kill_special = 0;
int dt_going = 0;
static int cl_count = 0;

/* request a data tasklet */
static void dt_req(enum dtask dt)
{
  dt_error = 0;
  kill_special = 0;
  bprintf(info, "DTreq: %s", dt_name[dt]);
  data_tk = dt;
  dt_going = 1;
}

/* check whether a data task has completed */
static int dt_done(void)
{
  if (dt_going && data_tk == dt_idle) {

    if (dt_error)
      bprintf(warning, "DTerr: %i", dt_error);

    dt_going = 0;
    kill_special = 0;
    return 1;
  }
  return 0;
}

/* request a data tasklet and wait for it's completion.  returns dt_error */
static int dt_wait(enum dtask dt)
{
  dt_req(dt);
  while (!dt_done())
    usleep(10000);

  return dt_error;
}

/* wait times after power cycle request (in seconds) */
static const int power_wait[] = { 10, 10, 20, 40, 60, 0 };

/* determine drive priorities */
static unsigned task_set_drives(void)
{
  uint8_t map = 0;
  int new_data_drive[3];
  char data_root[] = "/data#/mce";

  int best_spinner, second_spinner, the_solid;

  /* rate the drives */
  if (slow_dat.df[2] > slow_dat.df[3]) {
    best_spinner = 2;
    second_spinner = (slow_dat.df[3] > 0) ? 3 : -1;
  } else if (slow_dat.df[3] > 0) {
    best_spinner = 3;
    second_spinner = (slow_dat.df[2] > 0) ? 2 : -1;
  } else
    best_spinner = second_spinner = -1;

  if (slow_dat.df[0] > slow_dat.df[1])
    the_solid = 0;
  else if (slow_dat.df[1] > 0)
    the_solid = 1;
  else
    the_solid = -1;

  /* choose drive0 */
  if (best_spinner == 3) {
    map = DRIVE0_DATA3;
    new_data_drive[0] = 3;
  } else if (best_spinner == 2) {
    map = DRIVE0_DATA2;
    new_data_drive[0] = 2;
  } else if (the_solid == 1) {
    map = DRIVE0_DATA1;
    the_solid = -1;
    new_data_drive[0] = 1;
  } else if (the_solid == 0) {
    map = DRIVE0_DATA0;
    the_solid = -1;
    new_data_drive[0] = 0;
  } else {
    drive_map = (DRIVE0_UNMAP | DRIVE1_UNMAP | DRIVE2_UNMAP);
    bprintf(err, "No mappable drives!");
    return 1;
  }

  /* choose drive1 */
  if (second_spinner == 3) {
    map |= DRIVE1_DATA3;
    new_data_drive[1] = 3;
  } else if (second_spinner == 2) {
    map |= DRIVE1_DATA2;
    new_data_drive[1] = 2;
  } else if (the_solid == 1) {
    map |= DRIVE1_DATA1;
    new_data_drive[1] = 1;
    the_solid = -1;
  } else if (the_solid == 0) {
    map |= DRIVE1_DATA0;
    new_data_drive[1] = 0;
    the_solid = -1;
  } else {
    new_data_drive[1] = -1;
    map |= DRIVE1_UNMAP;
  }

  /* choose drive2 */
  if (the_solid == 1) {
    new_data_drive[2] = 1;
    map |= DRIVE2_DATA1;
  } else if (the_solid == 0) {
    new_data_drive[2] = 0;
    map |= DRIVE2_DATA0;
  } else {
    new_data_drive[2] = -1;
    map |= DRIVE2_UNMAP;
  }

  /* print mapping */
  bprintf(info, "  primary drive: /data%i", new_data_drive[0]);
  if (new_data_drive[1] == -1)
    bprintf(info, "secondary drive: unmapped");
  else
    bprintf(info, "secondary drive: /data%i", new_data_drive[1]);

  if (new_data_drive[2] == -1)
    bprintf(info, " tertiary drive: unmapped");
  else
    bprintf(info, " tertiary drive: /data%i", new_data_drive[2]);

  /* record */
  data_drive[0] = new_data_drive[0];
  data_drive[1] = new_data_drive[1];
  data_drive[2] = new_data_drive[2];
  drive_map = map;

  /* update the environment for scripts */
  data_root[5] = data_drive[0] + '0';
  setenv("MAS_DATA_ROOT", data_root, 1);

  return 0;
}

/* do a fake stop and empty the data buffer */
static int task_stop_acq(int fake)
{
  int i;
  int tries = 0;

  for (;;) {
    tries++;
    /* Stop */
    if (dt_wait(fake ? dt_fakestop : dt_stop)) {
      comms_lost = 1;
      return 1;
    }

    /* wait for acq termination */
    for (i = 0; i < 1000; ++i) {
      if (!(state & st_retdat))
        goto STOPPED;
      usleep(10000);
    }

    if (tries > 2)
      fake = 1;
    if (tries > 10)
      comms_lost = 1;
  }
STOPPED:

  /* Empty the buffer */
  if (dt_wait(dt_empty)) {
    comms_lost = 1;
    return 1;
  }

  /* clean up */
  dt_wait(dt_delacq);

  cl_count = 0;
  state |= st_mcecom;
  state &= ~(st_retdat | st_acqcnf);

  return 0;
}

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
  if (task_stop_acq(1))
    return 1;

  state |= st_mcecom;
  state &= ~st_config;
  return 0;
}

/* handle running a moda */
static enum modas task_off_moda;
static int task_off_reconfig;
static void task_run_moda(void)
{
  if (dt_done()) {
    /* an off_moda of md_none indicates that the goal is completed and
     * we should switch back to our default */
    if (task_off_moda == md_none) {
      bprintf(info, "Goal complete.");
      meta_goal_complete(task_off_reconfig);
    } else {
      bprintf(info, "Moda switch.");
      moda = task_off_moda;
    }
  }
}

/* run a dt script without blocking, switching to on_moda while it is running
 * and then to off_moda when it completes */
static void task_dt_script(enum dtask dt, enum modas on_moda,
    enum modas off_moda, int reconfig)
{
  dt_req(dt);
  moda = on_moda;
  task_off_moda = off_moda;
  task_off_reconfig = reconfig;
  meta_tk = 0;
}

/* run generic tasks */
void *task(void *dummy)
{
  int repc = 0;
  int check_acq = 0;
  nameThread("Task");

  /* wait for a task */
  for (;;) {
    if (terminate) { /* crash stop */
      bprintf(info, "Terminate.");
      return NULL;
    } else if (comms_lost) { /* deal with no MCE communication */
      bprintf(warning, "Comms lost.");
      meta_tk = 0xFFFFFFFF; /* interrupt! */

      /* stop mce comms */
      state &= ~st_active;
      mceveto = 1;

      /* stop the MCE and reset it */
      if (task_reset_mce() == 0 || memory.squidveto) {
        /* problem solved, I guess.  Get meta to return us to our scheduled
         * program */
        comms_lost = 0;
        check_acq = 0;
        cl_count = 0;
        meta_tk = 0;
        state &= ~(st_config | st_mcecom | st_acqcnf | st_retdat);
        state |= st_active;
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
      state &= ~(st_config | st_mcecom | st_retdat);
      meta_tk = 0; /* try again */
    } else if (req_dm != cur_dm) {
      if ((goal.goal >= gl_acq) && !memory.squidveto) {
        bprintf(info, "data mode change detected %i -> %i", cur_dm, req_dm);
        /* change of data mode while acquiring -- restart acq */
        task_reset_mce();
      }
      cur_dm = req_dm;
    } else if (meta_tk) {
      int stop = meta_tk & STOP_TK;
      enum status status_tk = st_idle;
      enum modas  moda_tk = md_none;

      if ((meta_tk & ~STOP_TK) >= (1U << MODA_SHIFT))
        moda_tk = (meta_tk & ~STOP_TK) >> MODA_SHIFT;
      else 
        status_tk = (meta_tk & ~STOP_TK);

      if (!stop) { /* handle start task requests */
        if (status_tk != st_idle)
          switch (status_tk) {
            case st_idle:
              break;
            case st_drives:
              /* choose drives */
              if (task_set_drives()) {
                /* no useable drives -- probably means some one should power
                 * cycle a hard drive can, but we'll just power cycle oursevles
                 */
                power_cycle_cmp = 1;
                sleep(60);
              } else {
                state |= st_drives;

                /* set directory */
                dt_wait(dt_setdir);

                /* parse experiment.cfg */
                if (load_experiment_cfg()) {
                  /* um ... */
                } else
                  send_mceparam = 1;
              }

              /* done */
              meta_tk = 0;
              break;
            case st_active:
              mceveto = 0;
              state |= st_active;
              meta_tk = 0;
              break;
            case st_mcecom:
              task_reset_mce();
              meta_tk = 0;
              break;
            case st_config:
              /* MCE reconfig */
              if (dt_wait(dt_reconfig)) {
                comms_lost = 1;
              } else {
                state |= st_config;
                meta_tk = 0;
              }
              break;
            case st_acqcnf:
              /* "mce status" */
              if (dt_wait(dt_status))
                comms_lost = 1;
              else if (dt_wait(dt_acqcnf)) /* acq config */
                comms_lost = 1;
              else {
                state |= st_acqcnf;
                meta_tk = 0;
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
                meta_tk = 0;
              }
              break;
          }
        else /* moda start */
          switch (moda_tk) {
            case md_none: /* just to shut CC up */
              break;
            case md_running: /* nop mode */
              moda = md_running;
              meta_tk = 0;
              break;
            case md_tuning:
              /* auto_setup */
              task_dt_script(dt_autosetup, moda_tk, md_none, 1);
              break;
            case md_iv_curve:
              /* iv curve */
              task_dt_script(dt_ivcurve, moda_tk, md_none, 1);
              break;
            case md_lcloop:
              /* lcloop script */
              task_dt_script(dt_lcloop, moda_tk, md_none, 1);
              break;
            case md_bstep: /* bias step */
              task_dt_script(dt_bstep, moda_tk, md_none, 0);
              break;
            case md_bramp: /* bias ramp */
              task_dt_script(dt_bramp, moda_tk, md_none, 0);
              break;
          }
      } else { /* stop task */
        if (status_tk != st_idle)
          switch (status_tk) {
            case st_idle:
              break;
            case st_retdat:
            case st_acqcnf:
            case st_drives:
            case st_mcecom:
              task_stop_acq(0);
              meta_tk = 0;
              break;
            case st_config:
              /* sledgehammer based stop acq */
              task_reset_mce();
              meta_tk = 0;
              break;
            case st_active:
              dt_error = dt_wait(dt_stopmce);
              mceveto = 1;
              state &= ~(st_config | st_active);
              meta_tk = 0;
              break;
          }
        else /* moda stop */
          switch (moda_tk) {
            case md_none: /* just to shut CC up */
              break;
            case md_running: /* nop mode */
              meta_tk = 0;
              moda = md_none;
              break;
            case md_tuning:
            case md_iv_curve:
            case md_lcloop:
              kill_special = 1;
              while (!dt_done())
                usleep(10000);
              kill_special = 0;
              task_reset_mce();
              moda = md_none;
              meta_tk = 0;
              break;
            case md_bstep:
            case md_bramp:
              kill_special = 1;
              while (!dt_done())
                usleep(10000);
              kill_special = 0;
              moda = md_none;
              meta_tk = 0;
              break;
          }
      }
    } else /* no new task, run the current moda */
      task_run_moda();
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
