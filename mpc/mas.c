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
#include <mce_library.h>
#include <mce/data_ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

static int need_reconfig = 1;
static int cmd_err = 0;

/* ask PCM to reboot the MCE if we get too many commanding errors */
#define MAS_CMD_ERR 1
#define CHECK_COMMAND_ERR() \
  do { \
    if (!command_veto) \
      if (++cmd_err >= MAS_CMD_ERR) { \
        bputs(err, "Too many commanding errors; MCE power cycle requested"); \
        power_cycle_mce = 1; \
      } \
  } while(0)

/* Write a block to the MCE; returns non-zero on error */
/* I guess MAS needs to be able to write to it's input data buffer...? */
static int mas_write_block(mce_context_t *mas, const char *card,
    const char *block, uint32_t *data, size_t num)
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

/* read a block from the MCE; returns non-zero on error */
static int mas_read_block(mce_context_t *mas, const char *card,
    const char *block, uint32_t *data, size_t num)
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

static void get_acq_metadata(mce_context_t *mas)
{
  uint32_t buffer[2];

  /* Reading from virtual cards is weird: the count parameter indicates the
   * number of elements per card to read, ergo, 1 here
   */
  if (!mas_read_block(mas, "rca", "data_mode", buffer, 1)) {
    /* sanity check */
    if (buffer[0] != buffer[1]) {
      bprintf(warning, "Data mode mismatch on RCs: %i, %i", buffer[0],
          buffer[1]);
      /* Might as well try fixing it. */
      mas_write_block(mas, "rc2", "data_mode", buffer, 1);
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

static int set_frame(mce_context_t *mas, char **frame)
{
  int fs = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_DATASIZE);
  bprintf(info, "framesize: %i", fs);

  if (*frame)
    free(*frame);
  *frame = malloc(fs);

  return fs;
}

/* Do a reset/reconfig on the MCE */
static int mce_reconfig(mce_context_t *mas)
{
  const char *card[] = { "cc", "rc1", "rc2", "bc1", "bc2", "ac", NULL };
  int ret, i;
  uint32_t datum;

  bputs(info, "MCE reset/reconfig");
  ret = mcecmd_interface_reset(mas);

  if (ret) {
    bprintf(err, "Error resetting DSP: error #%i", ret);
    return 1;
  }

  ret = mcecmd_hardware_reset(mas);
  if (ret) {
    bprintf(err, "Error resetting MCE: error #%i", ret);
    return 1;
  }

  if (run_simple_script(MAS_SCRIPT "/mce_reconfig", NULL)) {
    /* reset the mce on reconfig error...? */
    power_cycle_mce = 1;
    return 1;
  }

  /* Verify that everyone's alive */
  for (i = 0; card[i]; ++i)
    if (!mas_read_block(mas, card[i], "fw_rev", &datum, 1))
      bprintf(info, "%3s firmware: 0x%X", card[i], datum);

  return 0;
}

void *mas_data(void *dummy)
{
  char *frame = NULL;
  mce_context_t *mas;
  int frame_size;

  nameThread("MAS");

  /* Initialise MAS */
  mas = mcelib_create_termio(-1, NULL, 0, mpc_termio);

  if (mas == NULL)
    bputs(tfatal, "Unable to initialise MAS");
  bprintf(info, "libmce version: %s", mcelib_version());

  /* open the subsystems */
  if (mcedata_open(mas))
    bputs(tfatal, "Unable to attach to MAS data subsystem");

  if (mcecmd_open(mas))
    bputs(tfatal, "Unable to attach to MAS command subsystem");

  if (mceconfig_open(mas, NULL, NULL))
    bputs(tfatal, "Unable to attach to MAS config subsystem");

  /* leech mode */
  mcedata_ioctl(mas, DATADEV_IOCT_SET, DATA_LEECH);
  bputs(info, "Leeching data from MAS data device");

  mce_reconfig(mas);

  frame_size = set_frame(mas, &frame);
  get_acq_metadata(mas);
  
  /* main loop */
  for (;;) {
    /* check for change in leeched acq */
    int leech_valid = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_LVALID);
    if (!leech_valid) {
      mcedata_ioctl(mas, DATADEV_IOCT_RESET, 0);
      bputs(info, "New acquisition");
      frame_size = set_frame(mas, &frame);
      get_acq_metadata(mas);
    }

    int head;
    head = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_HEAD);
    int tail, ltail;
    tail = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_TAIL);
    ltail = mcedata_ioctl(mas, DATADEV_IOCT_QUERY, QUERY_LTAIL);
    
    ssize_t total;
    if (head > ltail) {
      bprintf(info, "buffer = %i -> %i/%i\n", head, tail, ltail);
      while (head > ltail) {
        total = mcedata_read(mas, frame, frame_size);
        do_frame((const uint32_t*)frame, frame_size);
        ltail++;
      }
    } else
      usleep(10000);
  }

  return NULL;
}
