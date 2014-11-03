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
#include "sys.h"
#include "mpc.h"
#include "mputs.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

int wait_state = st_idle;
static int wait_time = 0;
static const char *dt_name[] = { DT_STRINGS };
enum dtask data_tk = dt_idle;
int try_mount = 1; /* attempt to mount data drives */
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

/* kill a data task, if one is running */
static int dt_kill(void)
{
  if (dt_going) {
    kill_special = 1;
    while (!dt_done())
      usleep(10000);
    kill_special = 0;
  }

  return dt_error;
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

/* try mounting drives -- under linux, if a drive is screwed up, sometimes
 * attempt to mount it results in a zombie process, so we do this asynchronously
 * to try and prevent screwing up.
 */
static void task_mount_drives(void)
{
  int is_mounted[4] = {0, 0, 0, 0};
  int dont_mount[4] = {0, 0, 0, 0};
  int phy_map[100];
  int nerr[100];
  int last_phy = 0;
  char buffer[1024], l, n;
  int d, i;
  FILE *stream;

  memset(nerr, 0, sizeof(int) * 100);
  memset(phy_map, 0, sizeof(int) * 100);

  /* first figure out what is already mounted */
  stream = fopen("/proc/mounts", "r");
  if (stream == NULL) {
    berror(err, "Unable to read /proc/mount");
    return;
  }

  while (fgets(buffer, 1024, stream)) {
    /* looking for interesting lines */
    if (sscanf(buffer, "/dev/sd%c1 /data%c ext4", &l, &n) == 2)
      if (n >= '0' && n <= '3') {
        phy_map[(int)l] = n - '0';
        bprintf(info, "Found /dev/sd%c1 mounted on /data%c", l, n);
        is_mounted[n - '0'] = 1;
      }
  }

  fclose(stream);

  stream = open_dmesg(); 

  /* search dmesg for warning signs */
  while (fgets(buffer, 1024, stream)) {
    double offset = strtod(buffer + 1, NULL);
    if (btime + offset < memory.dmesg_lookback)
      continue;
    if (sscanf(buffer + 15, "end_request: critical target error, dev sd%c", &l)
        == 1)
    {
      if (last_phy < l)
        last_phy = l;
      nerr[(int)l]++;
    } else if (sscanf(buffer + 15, "end_request: I/O error, dev sd%c", &l) == 1)
    {
      if (last_phy < l)
        last_phy = l;
      nerr[(int)l]++;
    } else if (sscanf(buffer + 15, "Buffer I/O error on device sd%c1", &l) == 1)
    {
      if (last_phy < l)
        last_phy = l;
      nerr[(int)l]++;
    }
    usleep(1000);
  }
  close_dmesg(stream);

  /* deal with bad drives */
  for (i = 'a'; i <= last_phy; ++i)
    if (nerr[i] > 10) {
      if (phy_map[i] > 0) {
        bprintf(info, "%i kernel errors on device sd%c.  Disabling.",
            nerr[i], i);
        disk_bad[phy_map[i]] = 1;
        dont_mount[phy_map[i]] = 1;
      } else {
        /* try to find an unmounted drive associated with this device */
        for (d = 0; d < 4; ++d)
          if (!is_mounted[d]) {
            char link[1024];
            sprintf(link, "/dev/disk/by-uuid/%s", uuid[d]);
            if (readlink(link, buffer, 1024) > 0) {
              /* this will be of the form ../../sd?1 */
              if (buffer[8] == i) {
                bprintf(info, "%i kernel errors on device sd%c "
                    "intended as /data%i.  Disabling.", nerr[i], i, d);
                dont_mount[d] = 1;
                disk_bad[d] = 1;
              }
            }
          }
      }
    }

  /* unmount bad drives */
  for (d = 0; d < 4; ++d)
    if (is_mounted[d] && disk_bad[d]) {
      bprintf(info, "Lazy unmounting /data%i", d);
      sprintf(buffer, "/data%i", d);
      do_umount(buffer);
      is_mounted[d] = 0;
    }

  /* mount good drives */
  for (d = 0; d < 4; ++d)
    if (!is_mounted[d] && !dont_mount[d]) {
      bprintf(info, "Attempting to mount /data%i", d);
      sprintf(buffer, "/data%i", d);
      do_mount(buffer, 100);
      drive_error[d] = 0;
    }

  /* reset list of bad drives */
  memset(disk_bad, 0, sizeof(int) * 4); 
}

static void task_fix_data(void)
{
  struct stat stat_buf;

  /* try to stat our stored self -- this will tell us whether the
   * /data symlink has been screwed up
   */
  if (stat("/data/mas/bin/mpc", &stat_buf) < 0) {
    char mount[] = "/data#";

    /* need a new /data partition; remove the old symlink */
    unlink("/run/data");

    /* link it to our primary */
    mount[5] = data_drive[0] + '0';
    if (symlink(mount, "/run/data") == 0)
      bprintf(info, "linked %s to /data", mount);
  }
}

/* determine drive priorities */
static unsigned task_set_drives(void)
{
  uint8_t map = 0;
  int i, new_data_drive[3];
  char data_root[] = "/data#/mce";
  uint16_t df[4];

  /* wait for a slow data update */
  drives_checked = 0;
  while (!drives_checked)
    usleep(100000);

  for (i = 0; i < 4; ++i)
    df[i] = disk_bad[i] ? 0 : slow_dat.df[i];

  int best_spinner, second_spinner, the_solid;

  /* rate the drives */
  if (df[2] > df[3]) {
    best_spinner = 2;
    second_spinner = (df[3] > 0) ? 3 : -1;
  } else if (df[3] > 0) {
    best_spinner = 3;
    second_spinner = (df[2] > 0) ? 2 : -1;
  } else
    best_spinner = second_spinner = -1;

  if (df[0] > df[1])
    the_solid = 0;
  else if (df[1] > 0)
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
    if (dt_wait(dt_stop)) {
      comms_lost = 1;
      return 1;
    }

    /* fakestop */
    if (fake)
      dt_wait(dt_fakestop);

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
  state &= ~st_mcecom;
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
  state &= ~(st_config | st_biased);
  return 0;
}

static int rst_tune;
static int rst_iv;
static int rst_wait;
static int off_trans_wait;

void reset_array_health(void)
{
  rst_tune = 0;
  rst_iv = 0;
  rst_wait = memory.rst_wait;
  off_trans_wait = memory.off_trans_wait;
}

static void array_health(void)
{
  if (rst_wait == 0) {
    if (memory.ramp_max > 0 &&
        (slow_dat.ramp_count + slow_dat.clamp_count) > memory.ramp_max)
    {
      rst_tune++; /* kick it up a notch */
    } else
      rst_tune = 0; /* we're totally fine */

    /* wait a bit before rechecking */
    rst_wait = memory.rst_wait;

    /* do something abut the levels */
    if (rst_tune > memory.rst_tries) {
      /* try a tune */
      new_goal.force = 0; /* don't force-tune biases */
      new_goal.apply = 1; /* apply tuning */
      new_goal.goal = gl_tune;
      change_goal = 1; /* do it */
    } else if (rst_tune > 0) /* flux loop init */
      dt_wait(dt_rstsrvo);
  } else
    rst_wait--;

  if (off_trans_wait == 0) {
    if (memory.off_trans_max > 0 && slow_dat.off_trans > memory.off_trans_max)
      rst_iv++;
    else
      rst_iv = 0;

    /* wait a bit before rechecking */
    rst_wait = memory.rst_wait;

    /* do something abut the levels */
    if (rst_iv > 0) {
      new_goal.kick = memory.auto_iv_kick;
      new_goal.kicktime = memory.auto_iv_kicktime;
      new_goal.kickwait = memory.auto_iv_kickwait;
      new_goal.start = memory.auto_iv_start;
      new_goal.stop = memory.auto_iv_stop;
      new_goal.step = memory.auto_iv_step;
      new_goal.wait = memory.auto_iv_wait;
      new_goal.apply = 1;
      new_goal.goal = gl_iv;
      change_goal = 1;
    }
  } else
    off_trans_wait--;
}

/* handle running a moda */
static enum modas task_off_moda;
static int task_off_reconfig;
static void task_run_moda(void)
{
  /* do things while acquiring data */
  if (moda == md_running)
    array_health();

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
}

/* run generic tasks */
void *task(void *dummy)
{
  int repc = 0;
  int check_acq = 0;
  int no_dat = 0;
  nameThread("Task");

  /* wait for a task */
  for (;;) {
    if (terminate) { /* crash stop */
      bprintf(info, "Terminate.");
      return NULL;
    } else if (comms_lost) { /* deal with no MCE communication */
      bprintf(warning, "Comms lost.");
      meta_tk = 0xFFFFFFFF; /* interrupt! */
      wait_state = 0;

      /* stop mce comms */
      state &= ~st_active;

      /* stop the MCE and reset it */
      if (task_reset_mce() == 0 || memory.squidveto) {
        /* problem solved, I guess.  Get meta to return us to our scheduled
         * program */
        comms_lost = 0;
        check_acq = 0;
        cl_count = 0;
        meta_tk = 0;
        state &= ~(st_config | st_mcecom | st_acqcnf | st_biased | st_retdat);
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
      state &= ~(st_config | st_mcecom | st_retdat | st_biased);
      meta_tk = 0; /* try again */
    } else if (wait_state) {
      /* waiting */
      if (meta_tk & STOP_TK) {
        wait_time = 0;
        wait_state = 0;
        meta_tk = 0;
      } else if (wait_time == 0) {
        state |= wait_state;
        wait_state = 0;
      } else
        wait_time--;
    } else if (req_dm != cur_dm) {
      if ((goal.goal >= gl_acq) && !memory.squidveto && (state & st_config)) {
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
            case st_syncon:
              break;
            case st_drives:
              /* mount shennanigans */
              if (try_mount)
                task_mount_drives();

              /* choose drives */
              if (task_set_drives()) {
                /* no useable drives -- probably means some one should power
                 * cycle a hard drive can, but we'll just power cycle oursevles
                 */
                power_cycle_cmp = 1;
                try_mount = 1;
                sleep(60);
              } else {
                /* check that /data exists */
                task_fix_data();

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
              break;
            case st_active:
              state |= st_active;
              break;
            case st_mcecom:
              task_reset_mce();
              break;
            case st_config:
              /* MCE reconfig */
              if (dt_wait(dt_reconfig) == 0)
                state |= st_config;
              break;
            case st_biased:
              if (dt_wait(dt_kick) == 0) {
                wait_state = st_biased;
                wait_time = memory.bias_kick_wait * 100; /* convert to units
                                                            of 10 msec */
              }
              break;
            case st_acqcnf:
              /* "mce status" */
              if (dt_wait(dt_status))
                comms_lost = 1;
              else if (dt_wait(dt_acqcnf)) /* acq config */
                comms_lost = 1;
              else
                state |= st_acqcnf;
              break;
            case st_retdat:
              if (check_acq) { /* probably means an acquisition immediately
                                  terminated -- try a reset */
                comms_lost = 1;
                check_acq = 0;
                continue;
              } 
              check_acq = 1;
              no_dat = 0;
              /* start acq */
              if (dt_wait(dt_startacq))
                comms_lost = 1;
              else
                state |= st_retdat;
              break;
          }
        else /* moda start */
          switch (moda_tk) {
            case md_none: /* just to shut CC up */
              break;
            case md_running: /* nop mode */
              reset_array_health();
              moda = md_running;
              break;
            case md_tuning: /* auto_setup */
              task_dt_script(dt_autosetup, moda_tk, md_none, 1);
              break;
            case md_iv_curve: /* iv curve */
              task_dt_script(dt_ivcurve, moda_tk, md_none, 1);
              break;
            case md_lcloop: /* lcloop script */
              task_dt_script(dt_lcloop, moda_tk, md_none, 1);
              break;
            case md_bstep: /* bias step */
              if (check_acq == 0)
                task_dt_script(dt_bstep, moda_tk, md_none, 0);
              break;
            case md_bramp: /* bias ramp */
              if (check_acq == 0)
                task_dt_script(dt_bramp, moda_tk, md_none, 0);
              break;
            case md_partial: /* partial load curve */
              if (check_acq == 0)
                task_dt_script(dt_partial, moda_tk, md_none, 0);
              break;
          }
      } else { /* stop task */
        if (status_tk != st_idle)
          switch (status_tk) {
            case st_idle:
              break;
            case st_syncon:
            case st_biased:
              dt_kill();
              state &= ~status_tk;
              break;
            case st_retdat:
            case st_acqcnf:
            case st_drives:
            case st_mcecom:
              task_stop_acq(0);
              break;
            case st_config:
              /* sledgehammer based stop acq */
              task_reset_mce();
              break;
            case st_active:
              dt_error = dt_wait(dt_stopmce);
              state &= ~(st_biased | st_config | st_active);
              break;
          }
        else /* moda stop */
          switch (moda_tk) {
            case md_none: /* just to shut CC up */
              break;
            case md_running: /* nop mode */
              moda = md_none;
              break;
            case md_iv_curve:
              if (dt_kill())
                task_reset_mce();
              else
                state &= ~st_config;
              moda = md_none;
              break;
            case md_tuning:
            case md_lcloop:
              dt_kill();
              task_reset_mce();
              moda = md_none;
              break;
            case md_bstep:
            case md_bramp:
            case md_partial:
              dt_kill();
              moda = md_none;
              break;
          }
      }
      meta_tk = 0;
    } else /* no new task, run the current moda */
      task_run_moda();
    usleep(10000);

    if (repc++ > 100) { /* ie. this occurs roughly once a second */
      if (state & st_retdat) {
        if (check_acq) {
          if (rd_count > 20) {
            bprintf(info, "start of acquisition detected");
            check_acq = 0;
          } else if (no_dat++ > 2) {
            bprintf(info, "acquisition start error");
            comms_lost = 1;
          }
        } else if (!check_acq) {
          if (rd_count < 1) { /* no data in a second */
            bprintf(info, "loss of acquisition detected");
            state &= ~st_drives;
            comms_lost = 1;
          }
        }
        rd_count = 0;
      }
      repc = 0;
    }
  }
}
