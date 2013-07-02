/* mas.c: MAS connectivity in MPC
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
#include "blast.h"
#include "sys.h"
#include "mce_struct.h"
#include <mce_library.h>
#include <mce/data_ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>

static int cmd_err = 0;

/* ask PCM to reboot the MCE if we get too many commanding errors */
#define MAS_CMD_ERR 1
#define CHECK_COMMAND_ERR() \
  do { \
    if (++cmd_err >= MAS_CMD_ERR) { \
      comms_lost = 1; \
    } \
  } while(0)

/* mce_cmd stuff */
struct mcp_proc *mcecmd = NULL;
int mcecmd_i, mcecmd_o, mcecmd_e;

/* mas */
mce_context_t *mas;

/* Write a block to the MCE; returns non-zero on error */
/* I guess MAS needs to be able to write to it's input data buffer...? */
static int mas_write_block(const char *card, const char *block, uint32_t *data,
    size_t num)
{
  int ret;

  mce_param_t param;
  memset(&param, 0, sizeof(param));

  ret = mcecmd_load_param(mas, &param, card, block);

  if (ret) {
    bprintf(warning, "parameter load error, block %s/%s: %s", card, block,
        mcelib_error_string(ret));
    return 1;
  }

  ret = mcecmd_write_block(mas, &param, num, data);
  if (ret) {
    bprintf(warning, "write error, block %s/%s: %s", card, block,
        mcelib_error_string(ret));
    CHECK_COMMAND_ERR();
    return 1;
  }
  cmd_err = 0;

  return 0;
}

/* read a block by number from the MCE; returns non-zero on error */
static int mas_read_block_by_id(int card, int block, uint32_t *data, size_t num)
{
  return 1;
}

/* read a block by name from the MCE; returns non-zero on error */
static int mas_read_block(const char *card, const char *block, uint32_t *data,
    size_t num)
{
  int ret;

  mce_param_t param;
  memset(&param, 0, sizeof(param));

  ret = mcecmd_load_param(mas, &param, card, block);

  if (ret) {
    bprintf(warning, "parameter load error, block %s/%s: %s", card, block,
        mcelib_error_string(ret));
    return 1;
  }

  ret = mcecmd_read_block(mas, &param, num, data);
  if (ret) {
    bprintf(warning, "read error, block %s/%s: %s", card, block,
        mcelib_error_string(ret));
    CHECK_COMMAND_ERR();
    return 1;
  }
  cmd_err = 0;

  return 0;
}

static void get_acq_metadata(void)
{
  uint32_t buffer[2];

  /* Reading from virtual cards is weird: the count parameter indicates the
   * number of elements per card to read, ergo, 1 here
   */
  if (!mas_read_block("rca", "data_mode", buffer, 1)) {
    /* sanity check */
    if (buffer[0] != buffer[1]) {
      bprintf(warning, "Data mode mismatch on RCs: %i, %i", buffer[0],
          buffer[1]);
      /* Might as well try fixing it. */
      mas_write_block("rc2", "data_mode", buffer, 1);
    }
    if (data_mode != buffer[0])
      bprintf(info, "Data mode: %i", buffer[0]);
    data_mode = buffer[0];
  }
}

static int mpc_termio(int severity, const char *message)
{
  buos_t level = info;
  if (severity == MCELIB_WARN)
    level = warning;
  else if (severity == MCELIB_ERR)
    level = err;

  bputs(level, message);

  return 0;
}

static int set_frame(char **frame)
{
  int fs = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_DATASIZE);
  bprintf(info, "framesize: %i", fs);

  if (*frame)
    free(*frame);
  *frame = malloc(fs);

  return fs;
}


/* Check that we have all the cards */
static int mce_check_cards(void)
{
  const char *card[] = { "cc", "rc1", "rc2", "bc1", "bc2", "ac", NULL };
  int i, ret = 0;
  uint32_t datum;

  /* Verify that everyone's alive */
  for (i = 0; card[i]; ++i)
    if (!mas_read_block(card[i], "fw_rev", &datum, 1))
      bprintf(info, "%3s firmware: 0x%X", card[i], datum);
    else
      ret = 1;

  return ret;
}

/* formatted write to MCE_CMD */
static int __attribute__((format(printf,1,2))) mcecmd_write(const char *fmt,
    ...)
{
  char buffer[4096];
  int n, ret;
  va_list ap;
  
  va_start(ap, fmt);
  n = vsnprintf(buffer, 4096, fmt, ap);

  bprintf(info, "MCECMD: %s", buffer);

  buffer[n++] = '\n';
  buffer[n] = 0;

  if (write(mcecmd_i, buffer, n) < 0)
    ret = 1;
  else
    ret = 0;

  va_end(ap);

  return ret;
}

/* a big old array of block data */
static uint16_t *mce_stat[0xE0][0xB];

static long acq_time; /* for filenames */

static int mce_status(void)
{
  int i;
  acq_time = (long)time(NULL);

  /* mce status fun */
  for (i = 0; mstat_phys[i].c[0]; ++i) {

  }
  return 1;
}
static int acq_conf(void)
{
  /* restart the servo */
  uint32_t one = 1;
  mas_write_block("rca", "flx_lp_init", &one, 1);

  /* start a multiacq */
  mcecmd_write("acq_multi_begin");
  mcecmd_write("acq_config_fs /data0/mce/current_data/mpc_%li rcs 100000",
      acq_time);

  if (state & st_drive1)
    mcecmd_write("acq_config_fs /data1/mce/current_data/mpc_%li rcs 100000",
        acq_time);

  if (state & st_drive2)
    mcecmd_write("acq_config_fs /data2/mce/current_data/mpc_%li rcs 100000",
        acq_time);

  return 0;
}

static int start_mcecmd(void)
{
  char *argv[] = { "mce_cmd", "-ipr", NULL };
  mcecmd = start_proc("mce_cmd", argv, 0, 1, &mcecmd_i, &mcecmd_o,
      &mcecmd_e);
  if (mcecmd == NULL)
    return 1;

  return mcecmd_write("acq_link /data/mas/etc/mpc.lnk\n");
}

void *mas_data(void *dummy)
{
  char *frame = NULL;
  int frame_size = 0;
  int max = 0;
  int ret;

  nameThread("MAS");

  /* Initialise MAS */
  mas = mcelib_create_termio(-1, NULL, 0, mpc_termio);

  if (mas == NULL)
    bputs(tfatal, "Unable to initialise MAS");
  bprintf(info, "libmce version: %s", mcelib_version());

  /* open the subsystems */
  if (mcedata_open(mas))
    bputs(fatal, "Unable to attach to MAS data subsystem");

  if (mcecmd_open(mas))
    bputs(fatal, "Unable to attach to MAS command subsystem");

  if (mceconfig_open(mas, NULL, NULL))
    bputs(fatal, "Unable to attach to MAS config subsystem");

  /* leech mode */
  mcedata_ioctl(mas, DATADEV_IOCT_SET, DATA_LEECH);
  bputs(info, "Leeching data from MAS data device");

  /* main loop */
  for (;;) {

    /* deal with tasks */
    switch (data_tk) {
      case dt_idle:
        break;
      case dt_setdir:
        if (exec_and_wait(sched, MAS_SCRIPT "/set_directory", NULL, 20, 0)) 
          dt_error = 1;
        else
          dt_error = 0;
        data_tk = dt_idle;
        break;
      case dt_dsprs:
        ret = mcecmd_interface_reset(mas);
        if (ret) {
          bprintf(err, "Error resetting DSP: error #%i", ret);
          dt_error = 1;
        } else
          dt_error = 0;
        data_tk = dt_idle;
        break;
      case dt_mcers:
        ret = mcecmd_hardware_reset(mas);
        if (ret) {
          bprintf(err, "Error resetting MCE: error #%i", ret);
          dt_error = 1;
        } else if (mce_check_cards()) {
          bprintf(err, "Card check failed");
          dt_error = 1;
        } else
          dt_error = 0;

        frame_size = set_frame(&frame);
        get_acq_metadata();
        max = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_MAX);

        data_tk = dt_idle;
        break;
      case dt_mcecmd_init:
        dt_error = start_mcecmd();
        data_tk = dt_idle;
        break;
      case dt_reconfig:
        if (exec_and_wait(sched, MAS_SCRIPT "/mce_reconfig", NULL, 100, 0))
          comms_lost = 1;
        data_tk = dt_idle;
        break;
      case dt_status:
        dt_error = mce_status();
        break;
      case dt_acqcnf:
        dt_error = acq_conf();
        data_tk = dt_idle;
        break;
      case dt_startacq:
        dt_error = mcecmd_write("acq_go 1000000000"); /* 100 days */
        data_tk = dt_idle;
        break;
      case dt_fakestop:
        ret = mcedata_fake_stopframe(mas);
        if (ret) {
          bprintf(err, "Error stopping acquisition: error #%i", ret);
          dt_error = 1;
        } else
          dt_error = 0;
        data_tk = dt_idle;
        break;
      case dt_empty:
        ret = mcedata_empty_data(mas);
        if (ret) {
          bprintf(err, "Error emptying data queue: error #%i", ret);
          dt_error = 1;
        } else
          dt_error = 0;
        data_tk = dt_idle;
        break;
      case dt_multiend:
        dt_error = mcecmd_write("acq_multi_end");
        data_tk = dt_idle;
        break;
    }

    if (state & st_mcecmd) {
      /* check for change in leeched acq */
      int leech_valid = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_LVALID);
      if (!leech_valid) {
        mcedata_ioctl(mas, DATADEV_IOCT_RESET, 0);
        bputs(info, "New acquisition");
        /* reset frequency halving */
        pcm_strobe = 0;
        frame_size = set_frame(&frame);
        get_acq_metadata();
      }

      if (frame_size > 0) {
        int head;
        head = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_HEAD);
        int tail, ltail;
        tail = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_TAIL);
        ltail = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_LTAIL);
    
        ssize_t total;
        if (head != ltail) {
          while (head != ltail) {
            total = mcedata_read(mas, frame, frame_size);
            do_frame((const uint32_t*)frame, frame_size);
            ltail = (ltail + 1) % max;
          }
        }
      }
    }
    usleep(10000);
  }

  return NULL;
}
