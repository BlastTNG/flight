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

#include "mpc_proto.h"
#include "mpc.h"
#include "mputs.h"
#include "blast.h"
#include "sys.h"
#include "mce_struct.h"
#include <mce_library.h>
#include <mce/data_ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>

static unsigned cmd_err = 0;

/* ask PCM to reboot the MCE if we get too many commanding errors */
#define MAS_CMD_ERR 1
#define CHECK_COMMAND_ERR() \
  do { \
    if (++cmd_err >= MAS_CMD_ERR) { \
      comms_lost = 1; \
    } \
  } while(0)

/* a big old array of block data */
uint32_t mce_stat[N_MCE_STAT];

const char *const all_cards[] = {"cc", "rc1", "rc2", "bc1", "bc2", "ac", NULL};

/* mas */
mce_context_t *mas;

/* acq */
mce_acq_t *acq = NULL;
static volatile int acq_going = 0;
static volatile int acq_stopped = 0;
int acq_init = 0;

/* the data frame buffer */
size_t frame_size;
int fb_top = 0;
int pb_last = 0;
uint32_t *frame[FB_SIZE];
static uint32_t *fb = NULL;

/* number of frames in an acq_go */
#define ACQ_FRAMECOUNT 1000000000L /* a billion frames = 105 days */

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
    if (cur_dm != buffer[0]) {
      bprintf(info, "Data mode: %i", buffer[0]);
      cur_dm = buffer[0];
    }
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

/* Check that we have all the cards */
static int mce_check_cards(void)
{
  int i, ret = 0;
  uint32_t datum;

  /* Verify that everyone's alive */
  for (i = 0; all_cards[i]; ++i)
    if (!mas_read_block(all_cards[i], "fw_rev", &datum, 1))
      bprintf(info, "%3s firmware: 0x%X", all_cards[i], datum);
    else
      ret = 1;

  return ret;
}

static int param_index(const char *card, const char *param)
{
  int first = 0, last = 0, i;

  /* card offset */
  if (card[0] == 'a') {
    first = FIRST_AC_PARAM;
    last  =  LAST_AC_PARAM;
  } else if (card[0] == 'b') {
    first = (card[2] == '1') ? FIRST_BC1_PARAM : FIRST_BC2_PARAM;
    last  = (card[2] == '1') ?  LAST_BC1_PARAM :  LAST_BC2_PARAM;
  } else if (card[0] == 'c') {
    first = FIRST_CC_PARAM;
    last  =  LAST_CC_PARAM;
  } else if (card[0] == 'r') {
    first = (card[2] == '1') ? FIRST_RC1_PARAM : FIRST_RC2_PARAM;
    last  = (card[2] == '1') ?  LAST_RC1_PARAM :  LAST_RC2_PARAM;
  } else /* Be unforgiving */
    bprintf(fatal, "Parameter look-up error: %s/%s\n", card, param);

  /* parameter offset */
  for (i = first; i <= last; i++) {
    if (strcmp(param, mstat_phys[i].p) == 0) {
      return i;
    }
  }

  /* Be unforgiving */
  bprintf(fatal, "Parameter look-up error: %s/%s\n", card, param);
  return -1; /* can't get here */
}

/* update and return data in the mce_stat array */
static void update_param(const char *card, const char *param, uint32_t *data,
    int count)
{
  int i = param_index(card, param);

  if (count > mstat_phys[i].nw)
    bprintf(fatal, "Request for too much data: %i from %s/%s\n", count, card,
        param);

  /* fetch data */
  if (mas_read_block(card, param, data, count) == 0)
    /* update the array */
    memcpy(mce_stat + mstat_phys[i].cd, data, sizeof(uint32_t) * count);
  else 
    /* read error: just return old data */
    memcpy(data, mce_stat + mstat_phys[i].cd, sizeof(uint32_t) * count);
}

/* read a value from the mce_stat array */
static void mce_param(const char *card, const char *param, int offset,
    uint32_t *data, int count)
{
  int i = param_index(card, param);

  if (count + offset > mstat_phys[i].nw)
    bprintf(fatal, "Request for too much data: %i+%i from %s/%s\n", offset,
        count, card, param);

  memcpy(data, mce_stat + mstat_phys[i].cd + offset,
      sizeof(uint32_t) * count);
}

static long acq_time; /* for filenames */

/* write the runfile */
static int dump_runfile(FILE *stream)
{
  int i, j, nr, nw;
  uint32_t datum;
  uint32_t vdata[33];

  fprintf(stream, "<HEADER>\n");
  /* physical cards */
  for (i = 0; i < N_MCE_PHYS; ++i) {
    fprintf(stream, "<RB %s %s>", mstat_phys[i].c, mstat_phys[i].p);
    for (j = 0; j < mstat_phys[i].nw; ++j)
      fprintf(stream, " %08i", mce_stat[mstat_phys[i].cd + j]);
    if (mstat_phys[i].nr > mstat_phys[i].nw) /* pad */
      for (; j < mstat_phys[i].nr; ++j)
        fprintf(stream, " %08i", 0);
    fprintf(stream, "\n");
  }
  /* sys */
  for (i = 0; i < N_MCE_SYS; ++i) {
    fprintf(stream, "<RB sys %s>", mstat_sys[i]);
    for (j = 0; all_cards[j]; ++j) {
      mce_param(all_cards[j], mstat_sys[i], 0, &datum, 1);
      fprintf(stream, " %08i", datum);
    }
    fprintf(stream, "\n");
  }
  /* virt */
  for (i = 0 ; i < N_MCE_VIRT; ++i) {
    fprintf(stream, "<RB %s %s>", mstat_virt[i].c, mstat_virt[i].p);

    /* the first map */
    nw = nr = mstat_virt[i].m[0].e - mstat_virt[i].m[0].s;
    if (nr == 41)
      nw = 33;

    mce_param(mstat_virt[i].m[0].c, mstat_virt[i].m[0].p, mstat_virt[i].m[0].o,
        vdata, nw);
    for (j = 0; j < nw; ++j)
      fprintf(stream, " %08i", vdata[j]);
    if (nr > nw) /* pad */
      for (; j < nr; ++j)
        fprintf(stream, " %08i", 0);

    /* the second map */
    if (mstat_virt[i].m[1].c[0]) {
      nw = nr = mstat_virt[i].m[1].e - mstat_virt[i].m[1].s;
      mce_param(mstat_virt[i].m[1].c, mstat_virt[i].m[1].p,
          mstat_virt[i].m[1].o, vdata, nw);
      for (j = 0; j < nw; ++j)
        fprintf(stream, " %08i", vdata[j]);
    }
    fprintf(stream, "\n");
  }
  fprintf(stream, "</HEADER>\n");

  /* frameacq block */
  fprintf(stream, "<FRAMEACQ>\n"
      "  <RUNFILE_VERSION> 2\n"
      "  <MAS_VERSION> " MAS_VERSION "\n"
      "  <RC> rcs\n"
      "  <DATA_FILENAME> mpc_%li\n"
      "  <DATA_FRAMEACQ> %li\n"
      "  <CTIME> %li\n"
      "  <HOSTNAME> x%i.spider\n"
      "</FRAMEACQ>\n",
      acq_time, ACQ_FRAMECOUNT, acq_time, nmce + 1);

  /* mpc block */
  fprintf(stream, "<MPC>\n"
      "  <PROTO> %i/%i\n"
      "  <STATE> %08x\n"
      "</MPC>\n",
      mpc_proto_rev, mpc_cmd_rev, state);

  return 0;
}

static int mce_status(void)
{
  int i;
  FILE *stream;
  uint32_t buffer[41];
  char filename[100];

  acq_time = (long)time(NULL);

  /* read all the things */
  for (i = 0; i < N_MCE_PHYS; ++i) {
    if (mstat_phys[i].nr != mstat_phys[i].nw) {
      if (mas_read_block(mstat_phys[i].c, mstat_phys[i].p, buffer,
          mstat_phys[i].nr))
      {
        goto MCE_STATUS_ERR;
      }
      memcpy(mce_stat + mstat_phys[i].cd, buffer, sizeof(uint32_t) *
          mstat_phys[i].nw);
    } else if (mas_read_block(mstat_phys[i].c, mstat_phys[i].p, mce_stat +
          mstat_phys[i].cd, mstat_phys[i].nr))
    {
      goto MCE_STATUS_ERR;
    }
  }

  /* trigger send to PCM */
  send_mcestat = 1;

  /* now create the files */
  sprintf(filename, "/data%c/mce/current_data/mpc_%li.run", data_drive[0] + '0',
      acq_time);
  stream = fopen(filename, "w");
  if (stream == NULL)
    berror(err, "Unable to create runfile %s", filename);
  else {
    bprintf(info, "Writing runfile: %s\n", filename);
    dump_runfile(stream);
    fclose(stream);
  }

  if (state & st_drive1) {
    filename[5] = data_drive[1] + '0';
    stream = fopen(filename, "w");
    if (stream == NULL)
      berror(err, "Unable to create runfile %s", filename);
    else {
      bprintf(info, "Writing runfile: %s\n", filename);
      dump_runfile(stream);
      fclose(stream);
    }
  }

  if (state & st_drive2) {
    filename[5] = data_drive[2] + '0';
    stream = fopen(filename, "w");
    if (stream == NULL)
      berror(err, "Unable to create runfile %s", filename);
    else {
      bprintf(info, "Writing runfile: %s\n", filename);
      dump_runfile(stream);
      fclose(stream);
    }
  }

MCE_STATUS_ERR:
  return 0;
}

#define ACQ_INTERVAL 50000 /* number of frames in a chunk */
static int acq_conf(void)
{
  int r, i;
  mcedata_storage_t *st;
  uint32_t n = 1;
  char filename[100];

  /* stop the pushback */
  fb_top = pb_last = 0;

  /* restart the servo */
  n = 1;
  mas_write_block("rca", "flx_lp_init", &n, 1);

  /* set data mode */
  cur_dm = n = req_dm;
  mas_write_block("rca", "data_mode", &n, 1);


  /* start a multiacq -- this follows prepare_outfile in mce_cmd somewhat */
  acq = malloc(sizeof(mce_acq_t));
  if ((st = mcedata_multisync_create(0)) == NULL) {
    bprintf(err, "Failed to create multisync");
    goto ACQ_CONFIG_ERR;
  }

  if ((r = mcedata_acq_create(acq, mas, 0, 0 /* rcs */, -1, st))) {
    bprintf(err, "Multisync acquisition failed: %s", mcelib_error_string(r));
    goto ACQ_CONFIG_ERR;
  }

  /* create the rambuff callback */
  st = mcedata_rambuff_create(frame_acq, 0);
  if (st == NULL) {
    bprintf(err, "Couldn't set up frame callback");
    goto ACQ_CONFIG_ERR;
  }
  if (mcedata_multisync_add(acq, st)) {
    bprintf(err, "Couldn't append frame callback");
    goto ACQ_CONFIG_ERR;
  }

  /* now create a flatfile sequencer for each configured drive */

  /* drive0 */
  sprintf(filename, "/data%c/mce/current_data/mpc_%li", data_drive[0] + '0',
      acq_time);
  st = mcedata_fileseq_create(filename, ACQ_INTERVAL, 3 /* sequence digits */,
      "/data/mas/mpc.lnk");
  if (st == NULL) {
    bprintf(err, "Couldn't set up file sequencer for data0");
    goto ACQ_CONFIG_ERR;
  }
  if (mcedata_multisync_add(acq, st)) {
    bprintf(err, "Couldn't append file sequencer for data0");
    goto ACQ_CONFIG_ERR;
  }

  if (state & st_drive1) {
    filename[5] = data_drive[1] + '0';
    st = mcedata_fileseq_create(filename, ACQ_INTERVAL, 3 /* sequence digits */,
        NULL);
    if (st == NULL) {
      bprintf(err, "Couldn't set up file sequencer for data1");
      goto ACQ_CONFIG_ERR;
    }
    if (mcedata_multisync_add(acq, st)) {
      bprintf(err, "Couldn't append file sequencer for data1");
      goto ACQ_CONFIG_ERR;
    }
  }

  if (state & st_drive2) {
    filename[5] = data_drive[2] + '0';
    st = mcedata_fileseq_create(filename, ACQ_INTERVAL, 3 /* sequence digits */,
        NULL);
    if (st == NULL) {
      bprintf(err, "Couldn't set up file sequencer for data2");
      goto ACQ_CONFIG_ERR;
    }
    if (mcedata_multisync_add(acq, st)) {
      bprintf(err, "Couldn't append file sequencer for data2");
      goto ACQ_CONFIG_ERR;
    }
  }

  /* rebuild the frame buffer */
  free(fb);
  frame_size = acq->frame_size * sizeof(uint32_t);
  fb = malloc(FB_SIZE * frame_size);
  for (i = 0; i < FB_SIZE; ++i)
    frame[i] = fb + i * acq->frame_size;

  get_acq_metadata();

  /* done */
  return 0;

ACQ_CONFIG_ERR:
  mcedata_acq_destroy(acq);
  return 1;
}

static char *data_root(int n)
{
  static char data_root[] = "/data#/mce";
  data_root[5] = data_drive[n] + '0';
  return data_root;
}

static int write_array_id(int d)
{
  int n;

  char file[1024];
  sprintf(file, "/data%c/mce/array_id", data_drive[d] + '0');
  FILE *stream = fopen(file, "w");

  if (stream == NULL) {
    berror(err, "Unable to open array ID file: %s", file);
    goto BAD_ARRAY_ID;
  }

  n = fprintf(stream, "%s\n", array_id);

  if (n < 0) {
    berror(err, "Can't write array id!");
BAD_ARRAY_ID:
    fclose(stream);
    if (nmce == -1)
      nmce = 0;
    return 1;
  }
  fclose(stream);
  return 0;
}

static int set_directory(void)
{
  char *argv[] = {"set_directory", data_root(0), NULL};
  write_array_id(0);
  if (exec_and_wait(sched, none, MAS_SCRIPT "/set_directory", argv, 20, 1))
    return 1;

  if (state & st_drive1) {
    write_array_id(1);
    argv[1] = data_root(1);
    if (exec_and_wait(sched, none, MAS_SCRIPT "/set_directory", argv, 20, 1))
      return 1;
  }

  if (state & st_drive2) {
    write_array_id(2);
    argv[2] = data_root(2);
    if (exec_and_wait(sched, none, MAS_SCRIPT "/set_directory", argv, 20, 1))
      return 1;
  }

  return 0;
}

void *mas_data(void *dummy)
{
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

  /* main loop */
  for (;;) {

    /* deal with tasks */
    switch (data_tk) {
      case dt_idle:
        break;
      case dt_setdir:
        dt_error = set_directory();
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

        data_tk = dt_idle;
        break;
      case dt_reconfig:
        if (exec_and_wait(sched, none, MAS_SCRIPT "/mce_reconfig", NULL, 100,
              0))
        {
          comms_lost = 1;
        }
        data_tk = dt_idle;
        break;
      case dt_status:
        dt_error = mce_status();
        data_tk = dt_idle;
        break;
      case dt_acqcnf:
        dt_error = acq_conf();
        data_tk = dt_idle;
        break;
      case dt_startacq:
        acq_going = 1;
        dt_error = 0;
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
      case dt_autosetup:
        if (exec_and_wait(sched, startup, MAS_SCRIPT "/auto_setup", NULL, 0, 0))
          dt_error = 1;
        else
          dt_error = 0;
        data_tk = dt_idle;
        break;
      case dt_delacq:
        if (acq)
          mcedata_acq_destroy(acq);
        acq = NULL;
        dt_error = 0;
        data_tk = dt_idle;
        break;
    }

    /* temperature poll */
    if (mas_get_temp) {
      update_param("cc", "box_temp", (uint32_t*)&box_temp, 1);
      mas_get_temp = 0;
    }

    /* check for acq termination */
    if (acq_going && acq_stopped) {
      acq_going = 0;
      acq_stopped = 0;
      state &= ~st_retdat;
    }
    usleep(10000);
  }

  return NULL;
}

/* the acquisition thread */
void *acquer(void* dummy)
{
  int r;
  nameThread("Acq");

  for (;;) {
    usleep(100000);
    if (acq_going && !acq_stopped) {
      acq_init = 1;
      r = mcedata_acq_go(acq, ACQ_FRAMECOUNT); /* blocks */
      if (r)
        bprintf(err, "Acquisition error: %s", mcelib_error_string(r));
      else
        bprintf(info, "Acquisition stopped");
      acq_stopped = 1;
      acq_init = 0; /* just in case */
    }
  }

  return NULL;
}
