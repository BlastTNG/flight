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

#define SHOW_WRITE_PARAM

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
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <libgen.h>
#include <string.h>
#include <time.h>
#include <signal.h>

uint16_t tuning_status = 0;

static unsigned cmd_err = 0;

static uint32_t ref_biases_dark[16];
static uint32_t ref_biases_lite[16];
int ref_biases_dark_ok = 0;
int ref_biases_lite_ok = 0;

#define MAS_CMD_ERR 2
#define CHECK_COMMAND_ERR() \
  do { \
    if (++cmd_err >= MAS_CMD_ERR) { \
      comms_lost = 1; \
    } else \
    cmd_err = 0; \
  } while(0)

/* a big old array of block data */
uint32_t mce_stat[N_MCE_STAT];

const char *const all_cards[] = {"cc", "rc1", "rc2", "bc1", "bc2", "ac", NULL};

/* mas */
mce_context_t *mas = NULL;

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

uint32_t iclamp[2];

/* number of frames in an acq_go */
#define ACQ_FRAMECOUNT 1000000000 /* a billion frames = 105 days */

#define MPC_LNK "/data/mas/etc/mpc.lnk"

/* veto stat computation */
int stat_veto = 0;

static int pop_block(void);

/* Write a (parital) block to the MCE; returns non-zero on error */
/* I guess MAS needs to be able to write to it's input data buffer...? */
static int mas_write_range(const char *card, const char *block, int offset,
    uint32_t *data, size_t num)
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

  /* why does mcecmd_..._range() have a different parameter order than 
   * mcecmd_..._block()?
   */
  ret = mcecmd_write_range(mas, &param, offset, data, num);
  if (ret) {
    bprintf(warning, "write error, block %s/%s: %s", card, block,
        mcelib_error_string(ret));
    CHECK_COMMAND_ERR();
    return 1;
  }
  cmd_err = 0;

  return 0;
}

static int mas_write_block(const char *card, const char *block, uint32_t *data,
    size_t num)
{
  /* ho hum .. */
  return mas_write_range(card, block, 0, data, num);
}

static inline int drive_ready(int i)
{
  if (i == 0)
    return ((drive_map & DRIVE0_MASK) != DRIVE0_UNMAP);
  if (i == 1)
    return ((drive_map & DRIVE1_MASK) != DRIVE1_UNMAP);
  if (i == 2)
    return ((drive_map & DRIVE2_MASK) != DRIVE2_UNMAP);
  return 0;
}

/* read a (partial) block by name from the MCE; returns non-zero on error */
static int mas_read_range(const char *card, const char *block, int offset,
    uint32_t *data, size_t num)
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

  ret = mcecmd_read_range(mas, &param, offset, data, num);
  if (ret) {
    bprintf(warning, "read error, block %s/%s: %s", card, block,
        mcelib_error_string(ret));
    CHECK_COMMAND_ERR();
    return 1;
  }
  cmd_err = 0;

  return 0;
}

static int mas_read_block(const char *card, const char *block, uint32_t *data,
    size_t num)
{
  /* Boring ... */
  return mas_read_range(card, block, 0, data, num);
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
  int virt = 0, first = 0, last = 0, i;

  /* card offset */
  if (card[1] == 'c') { /* physical card */
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
    }
  } else { /* maybe a virtual card */
    virt = 1;
    if (strcmp(card, "sq1") == 0) {
      first = FIRST_SQ1_MAP;
      last = LAST_SQ1_MAP;
    } else if (strcmp(card, "sq2") == 0) {
      first = FIRST_SQ2_MAP;
      last = LAST_SQ2_MAP;
    } else if (strcmp(card, "sa") == 0) {
      first = FIRST_SA_MAP;
      last = LAST_SA_MAP;
    } else if (strcmp(card, "tes") == 0) {
      first = FIRST_TES_MAP;
      last = LAST_TES_MAP;
    } else if (strcmp(card, "heater") == 0) {
      first = FIRST_HEATER_MAP;
      last = LAST_HEATER_MAP;
    } else {
      /* Be unforgiving */
      bprintf(fatal, "Parameter look-up error: %s/%s\n", card, param);
    }
  }

  /* parameter offset */
  if (virt) {
    for (i = first; i <= last; i++)
      if (strcmp(param, mstat_virt[i].p) == 0)
        return -1 - i;
  } else 
    for (i = first; i <= last; i++)
      if (strcmp(param, mstat_phys[i].p) == 0)
        return i;

  /* Be unforgiving */
  bprintf(fatal, "Parameter look-up error: %s/%s\n", card, param);
  return -1; /* can't get here */
}

/* send a parameter to the MCE, updating it in the mce_stat array */
static void write_param(const char *card, const char *param, int offset,
    uint32_t *data, int count)
{
  int n, new = 0;
  int i = param_index(card, param);

#ifdef SHOW_WRITE_PARAM
  {
    int i;
    char *ptr, params[1000];
    ptr = params;
    for (i = 0; i < count; ++i)
      ptr += sprintf(ptr, "%u ", data[i]);
    bprintf(info, "write_param: %s/%s+%i(%i) [ %s]", card, param, offset,
        count, params);
  }
#endif

  if (i < 0) { /* virtual parameters */
    int n = 0;
    i = -i - 1;
    if (count + offset > mstat_virt[i].n)
      bprintf(fatal, "Request for too much data: %i+%i from %s/%s\n", offset,
          count, card, param);

    /* do we have data in the first map? */
    if (offset < mstat_virt[i].m[0].e) {
      n = mstat_virt[i].m[0].e - offset;
      if (n > count)
        n = count;

      write_param(mstat_virt[i].m[0].c, mstat_virt[i].m[0].p,
          mstat_virt[i].m[0].o + offset, data, n);
    }

    /* do we have data in the second map? */
    if (count > n)
      write_param(mstat_virt[i].m[1].c, mstat_virt[i].m[1].p,
          mstat_virt[i].m[1].o, data + n, count - n);
  } else {
    if (count + offset > mstat_phys[i].nw)
      bprintf(fatal, "Write of too much data: %i+%i from %s/%s\n", count,
          offset, card, param);

    /* check whether an update is needed */
    for (n = 0; n < count; ++n)
      if (mce_stat[mstat_phys[i].cd + offset + n] != data[n]) {
        new = 1;
        break;
      }

    if (!new)
      return;

    /* update MCE */
    if (mas_write_range(card, param, offset, data, count) == 0)
      /* update the array */
      memcpy(mce_stat + mstat_phys[i].cd + offset, data,
          sizeof(uint32_t) * count);
  }
}

/* fetch a parameter from the MCE and save it to the mce_stat array */
static void fetch_param(const char *card, const char *param, int offset,
    uint32_t *data, int count)
{
  int i = param_index(card, param);

  if (i < 0) { /* virtual parameters */
    int n = 0;
    i = -i - 1;
    if (count + offset > mstat_virt[i].n)
      bprintf(fatal, "Request for too much data: %i+%i from %s/%s\n", offset,
          count, card, param);

    /* do we have data in the first map? */
    if (offset < mstat_virt[i].m[0].e) {
      n = mstat_virt[i].m[0].e - offset;
      if (n > count)
        n = count;

      fetch_param(mstat_virt[i].m[0].c, mstat_virt[i].m[0].p,
          mstat_virt[i].m[0].o + offset, data, n);
    }

    /* do we have data in the second map? */
    if (count > n)
      fetch_param(mstat_virt[i].m[0].c, mstat_virt[i].m[0].p,
          mstat_virt[i].m[0].o, data + n, count - n);
  } else {
    if (count + offset > mstat_phys[i].nw)
      bprintf(fatal, "Request for too much data: %i+%i from %s/%s\n", count,
          offset, card, param);

    /* fetch data */
    if (mas_read_range(card, param, offset, data, count) == 0)
      /* update the array */
      memcpy(mce_stat + mstat_phys[i].cd + offset, data,
          sizeof(uint32_t) * count);
    else 
      /* read error: just return old data */
      memcpy(data, mce_stat + mstat_phys[i].cd + offset,
          sizeof(uint32_t) * count);
  }
}

/* read a value from the mce_stat array */
void read_param(const char *card, const char *param, int offset, uint32_t *data,
    int count)
{
  int i = param_index(card, param);

  if (i < 0) { /* virtual parameters */
    int n = 0;
    i = -i - 1;
    if (count + offset > mstat_virt[i].n)
      bprintf(fatal, "Request for too much data: %i+%i from %s/%s\n", offset,
          count, card, param);

    /* do we have data in the first map? */
    if (offset < mstat_virt[i].m[0].e) {
      n = mstat_virt[i].m[0].e - offset;
      if (n > count)
        n = count;

      read_param(mstat_virt[i].m[0].c, mstat_virt[i].m[0].p,
          mstat_virt[i].m[0].o + offset, data, n);
    }

    /* do we have data in the second map? */
    if (count > n)
      read_param(mstat_virt[i].m[0].c, mstat_virt[i].m[0].p,
          mstat_virt[i].m[0].o, data + n, count - n);
  } else {
    if (count + offset > mstat_phys[i].nw)
      bprintf(fatal, "Request for too much data: %i+%i from %s/%s\n", offset,
          count, card, param);

    memcpy(data, mce_stat + mstat_phys[i].cd + offset,
        sizeof(uint32_t) * count);
  }
}

static long acq_time; /* for filenames */

/* write the runfile */
static int dump_runfile(FILE *stream, int n)
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
      read_param(all_cards[j], mstat_sys[i], 0, &datum, 1);
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

    read_param(mstat_virt[i].m[0].c, mstat_virt[i].m[0].p, mstat_virt[i].m[0].o,
        vdata, nw);
    for (j = 0; j < nw; ++j)
      fprintf(stream, " %08i", vdata[j]);
    if (nr > nw) /* pad */
      for (; j < nr; ++j)
        fprintf(stream, " %08i", 0);

    /* the second map */
    if (mstat_virt[i].m[1].c[0]) {
      nw = nr = mstat_virt[i].m[1].e - mstat_virt[i].m[1].s;
      read_param(mstat_virt[i].m[1].c, mstat_virt[i].m[1].p,
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
      "  <RC> 1 2\n"
      "  <DATA_FILENAME> mpc_%li\n"
      "  <DATA_FRAMEACQ> %i\n"
      "  <CTIME> %li\n"
      "  <ARRAY_ID> x%i\n"
      "  <HOSTNAME> x%i.spider\n"
      "</FRAMEACQ>\n",
      acq_time, ACQ_FRAMECOUNT, acq_time, nmce + 1, nmce + 1);

  /* mpc block */
  fprintf(stream, "<MPC>\n"
      "  <PROTO> %i/%i\n"
      "  <USED_TUNE> %i\n"
      "  <LAST_TUNE> %i\n"
      "  <LAST_IV> %i\n"
      "</MPC>\n",
      mpc_proto_rev, mpc_cmd_rev, memory.used_tune, memory.last_tune,
      memory.last_iv);

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
  send_mceparam = 1;

  sprintf(filename, "/data%c/mce/current_data/mpc_%li.run", data_drive[0] + '0',
      acq_time);

  /* now create the files */
  if ((drive_map & DRIVE0_MASK) != DRIVE0_UNMAP) {
    stream = fopen(filename, "w");
    if (stream == NULL)
      berror(err, "Unable to create runfile %s", filename);
    else {
      bprintf(info, "Writing runfile: %s\n", filename);
      dump_runfile(stream, ACQ_FRAMECOUNT);
      fclose(stream);
    }
  }

  if ((drive_map & DRIVE1_MASK) != DRIVE1_UNMAP) {
    filename[5] = data_drive[1] + '0';
    stream = fopen(filename, "w");
    if (stream == NULL)
      berror(err, "Unable to create runfile %s", filename);
    else {
      bprintf(info, "Writing runfile: %s\n", filename);
      dump_runfile(stream, ACQ_FRAMECOUNT);
      fclose(stream);
    }
  }

  if ((drive_map & DRIVE2_MASK) != DRIVE2_UNMAP) {
    filename[5] = data_drive[2] + '0';
    stream = fopen(filename, "w");
    if (stream == NULL)
      berror(err, "Unable to create runfile %s", filename);
    else {
      bprintf(info, "Writing runfile: %s\n", filename);
      dump_runfile(stream, ACQ_FRAMECOUNT);
      fclose(stream);
    }
  }

MCE_STATUS_ERR:
  return 0;
}

/* multisync error callback */
static int acq_err(void *user_data, int sync_num, int err,
    mcedata_stage_t stage)
{
  switch (sync_num) {
    case 0:
      /* the rambuff -- ignore */
      return 0;
    case 1: /* primary */
      bprintf(info, "primary drive failed; stopping acq");
      drive_error[data_drive[0]] = 1;
      state &= ~st_drives;
      break;
    case 2: /* secondary */
      bprintf(info, "secondary drive failed; stopping acq");
      drive_error[data_drive[1]] = 1;
      state &= ~st_drives;
      break;
    case 3: /* teritary */
      bprintf(info, "tertiary drive failed; stopping acq");
      drive_error[data_drive[1]] = 2;
      state &= ~st_drives;
      break;
  }
  if (drive_map == (DRIVE0_UNMAP | DRIVE1_UNMAP | DRIVE2_UNMAP)) {
    /* uh-oh */
    state &= ~st_drives;
  }
  return 1; /* stop this one */
}

/* send a command, if pending */
static void deblock(void)
{
  if ((state & st_active) && (state & st_mcecom)) {
    /* pop a block from the queue, if there are any,
     * otherwise, poll the temperature, if requested */
    if (!pop_block() && mas_get_temp) {
      fetch_param("cc", "box_temp", 0, (uint32_t*)&box_temp, 1);
      mas_get_temp = 0;
    }
  }
}

/* wait with kill detection */
static int check_wait(double wait)
{
  double i;

  for (i = 0; i < wait; i += 0.01) {
    if (kill_special)
      return 1;

    deblock();
    usleep(10000);
  }
  return kill_special ? 1 : 0;
}

/* restart the servo */
static int flux_loop_init(double wait)
{
  uint32_t one = 1;
  bprintf(info, "Flux Loop Init");
  mas_write_block("rca", "flx_lp_init", &one, 1);
  stat_reset = 1;
  return check_wait(wait);
}

#define ACQ_INTERVAL 50000 /* number of frames in a chunk */
static int acq_conf(void)
{
  int r, i;
  mcedata_storage_t *st;
  uint32_t n;
  char filename[100];

  /* stop the pushback */
  fb_top = pb_last = 0;

  /* restart the servo */
  flux_loop_init(0);

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

  /* register the error handler */
  mcedata_multisync_errcallback(acq, acq_err, NULL);

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

  if ((drive_map & DRIVE0_MASK) != DRIVE0_UNMAP) {
    st = mcedata_fileseq_create(filename, ACQ_INTERVAL, 3 /* sequence digits */,
        MPC_LNK);
    if (st == NULL) {
      bprintf(err, "Couldn't set up file sequencer for primary drive");
      goto ACQ_CONFIG_ERR;
    }
    if (mcedata_multisync_add(acq, st)) {
      bprintf(err, "Couldn't append file sequencer for primary drive");
      goto ACQ_CONFIG_ERR;
    }
  }

  if ((drive_map & DRIVE1_MASK) != DRIVE1_UNMAP) {
    filename[5] = data_drive[1] + '0';
    st = mcedata_fileseq_create(filename, ACQ_INTERVAL, 3 /* sequence digits */,
        NULL);
    if (st == NULL) {
      bprintf(err, "Couldn't set up file sequencer for secondary drive");
      goto ACQ_CONFIG_ERR;
    }
    if (mcedata_multisync_add(acq, st)) {
      bprintf(err, "Couldn't append file sequencer for secondary drive");
      goto ACQ_CONFIG_ERR;
    }
  }

  if ((drive_map & DRIVE2_MASK) != DRIVE2_UNMAP) {
    filename[5] = data_drive[2] + '0';
    st = mcedata_fileseq_create(filename, ACQ_INTERVAL, 3 /* sequence digits */,
        NULL);
    if (st == NULL) {
      bprintf(err, "Couldn't set up file sequencer for tertiary drive");
      goto ACQ_CONFIG_ERR;
    }
    if (mcedata_multisync_add(acq, st)) {
      bprintf(err, "Couldn't append file sequencer for tertiary drive");
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

static int write_array_id(int d)
{
  int n;

  char file[1024];
  sprintf(file, "/data%c/mce/array_id", d + '0');
  FILE *stream = fopen(file, "w");

  if (stream == NULL) {
    berror(err, "Unable to open array ID file: %s", file);
    goto BAD_ARRAY_ID;
  }

  n = fprintf(stream, "%s\n", array_id);

  if (n < 0) {
    berror(err, "Can't write array id!");
    fclose(stream);
BAD_ARRAY_ID:
    if (nmce == -1)
      nmce = 0;
    return 1;
  }
  fclose(stream);
  return 0;
}

static int set_directory(void)
{
  int i, r = 1;
  char data_root[] = "/data#/mce";
  char *argv[] = {"set_directory", data_root, NULL};
  for (i = 0; i < 4; ++i) {
    if (disk_bad[i])
      continue;

    data_root[5] = i + '0';
    write_array_id(i);
    if (exec_and_wait(sched, none, MAS_SCRIPT "/set_directory", argv, 20, 0,
          NULL))
    {
      drive_error[i] = 1;
      state &= ~st_drives;
    } else
      r = 0; /* at least one good drive */
  }

  return r;
}

#define KICK_DONT_BIAS 4000000000U
static int kick(uint32_t tes_bias, uint32_t kick_bias, uint32_t heater_bias,
    double time, int wait)
{
  int i, need_rebias = 0;
  uint32_t zero = 0;
  uint32_t data[16];
  uint32_t saved_bias[16];
  int kick_time;

  if (heater_bias == 0)
    return 0;

  /* convert to microseconds, and set default if necessary */
  if (time == 0)
    kick_time = memory.bias_kick_time;
  else
    kick_time = time * 1000000;

  if (kick_bias > 0) {
    /* remember biases */
    if (tes_bias == KICK_DONT_BIAS)
      fetch_param("tes", "bias", 0, saved_bias, 16);
    else
      for (i = 0; i < 16; ++i)
        saved_bias[i] = tes_bias;

    for (i = 0; i < 16; ++i)
      data[i] = kick_bias;
    write_param("tes", "bias", 0, data, 16);
    need_rebias = 1;
  } else if (tes_bias != KICK_DONT_BIAS) {
    for (i = 0; i < 16; ++i)
      data[i] = tes_bias;
    write_param("tes", "bias", 0, data, 16);
  }

  bprintf(info, "Kick %i counts for %i msec", heater_bias, kick_time / 1000);

  write_param("heater", "bias", 0, &heater_bias, 1);
  slow_dat.tile_heater = heater_bias;
  usleep(kick_time);
  write_param("heater", "bias", 0, &zero, 1);
  slow_dat.tile_heater = 0;

  /* rebias, if necessary */
  if (need_rebias)
    write_param("tes", "bias", 0, saved_bias, 16);

  /* fast exit? */
  if (wait == 0)
    return 0;

  /* wait */
  if (check_wait(wait))
    return 1;

  /* flx_lp_init */
  return flux_loop_init(2);
}

static void pick_biases(int iv_num, int dark, int set_ref)
{
  int good = 1;
  struct mcp_proc *p;
  int p_stdout = 0;
  char biases[1024];
  uint32_t data[256];
  char ivname[256];
  char *darks = "--darks";
  char *argv[] = { "/data/mas/bin/choose_tes_bias", ivname, NULL /* darks */,
    NULL };

  /* read the reference tuning, if needed */
  if (!set_ref) {
    if (dark) {
      if (!ref_biases_dark_ok)
        pick_biases(memory.ref_iv, 1, 1);
    } else
      if (!ref_biases_lite_ok)
        pick_biases(memory.ref_iv, 0, 1);
  }

  if (dark)
    argv[2] = darks;

  if (iv_num == 0)
    iv_num = memory.last_iv;

  sprintf(ivname, "iv_%04i", iv_num);

  p = start_proc(argv[0], argv, 0, 1, NULL, &p_stdout, NULL,
      &kill_special);
  if (p == NULL)
    return;

  /* capture the output */
  if (read(p_stdout, &biases, 1024) < 0)
    good = 0;

  stop_proc(p, 0, 0, 0, 0);

  if (good)
    if (sscanf(biases, "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
          data + 0, data + 1, data + 2, data + 3, data + 4, data + 5, data + 6,
          data + 7, data + 8, data + 9, data + 10, data + 11, data + 12,
          data + 13, data + 14, data + 15) == 16)
    {
      if (set_ref) {
        memory.ref_iv = iv_num;
        if (dark) {
          ref_biases_dark_ok = 1;
          memcpy(ref_biases_dark, data, sizeof(uint32_t) * 16);
        } else {
          ref_biases_lite_ok = 1;
          memcpy(ref_biases_lite, data, sizeof(uint32_t) * 16);
        }
      } else {
        /* apply and record, then kick */
        write_param("tes", "bias", 0, data, 16);
        cfg_set_intarr("tes_bias", 0, data, 16);
        kick(KICK_DONT_BIAS, memory.bias_kick_bias, memory.bias_kick_val, 0,
            memory.bias_kick_wait);
        state |= st_biased;
      }
    }
}

static void pop_kick(const struct block_q *req)
{
  const double *time = (const double*)(req->d);
  kick(KICK_DONT_BIAS, req->d[2], req->o, *time, memory.bias_kick_wait);
}

/* returns non-zero if something was popped */
static int pop_block(void)
{
  static int dont_pop = 0;

  if (dont_pop)
    return 0;

  int new_tail = (blockq_tail + 1) % BLOCKQ_SIZE;

  /* no pending requests */
  if (blockq_head == blockq_tail)
    return 0;

  dont_pop = 1;

  switch (blockq[new_tail].raw) {
    case 3:
      pick_biases(blockq[new_tail].o, blockq[new_tail].d[0], 0);
      break;
    case 2: /* asynchronous kick request */
      pop_kick(blockq + new_tail);
      break;
    case 1:
      mas_write_range(blockq[new_tail].c, blockq[new_tail].p,
          blockq[new_tail].o, blockq[new_tail].d, blockq[new_tail].n);
      break;
    case 0:
      write_param(blockq[new_tail].c, blockq[new_tail].p, blockq[new_tail].o,
          blockq[new_tail].d, blockq[new_tail].n);
      break;
  }

  free(blockq[new_tail].c);
  free(blockq[new_tail].p);

  blockq_tail = new_tail;

  dont_pop = 0;
  return 1;
}

/* zero biases and shut down muxing */
static void stop_mce(void)
{
  uint32_t one = 1;
  uint32_t zeroes[33] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int32_t number[8] = {-8192, -8192, -8192, -8192, -8192, -8192, -8192, -8192};

  /* these are row-wise */
  bprintf(info, "Stopping MCE.");
  write_param("ac", "on_bias", 0, zeroes, 33);
  write_param("ac", "off_bias", 0, zeroes, 33);
  write_param("ac", "enbl_mux", 0, &one, 1);

  /* these are column-wise */
  write_param("sq2", "fb", 0, zeroes, 16);
  write_param("sq2", "bias", 0, zeroes, 16);
  write_param("sa", "fb", 0, zeroes, 16);
  write_param("sa", "bias", 0, zeroes, 16);
  write_param("sa", "offset", 0, zeroes, 16);
  write_param("tes", "bias", 0, zeroes, 16);

  /* wait and then turn off muxing */
  usleep(200000);
  write_param("ac", "enbl_mux", 0, zeroes, 1);

  /* adc offset per pixel */
  write_param("rc1", "adc_offset0", 0, zeroes, 33);
  write_param("rc1", "adc_offset1", 0, zeroes, 33);
  write_param("rc1", "adc_offset2", 0, zeroes, 33);
  write_param("rc1", "adc_offset3", 0, zeroes, 33);
  write_param("rc1", "adc_offset4", 0, zeroes, 33);
  write_param("rc1", "adc_offset5", 0, zeroes, 33);
  write_param("rc1", "adc_offset6", 0, zeroes, 33);
  write_param("rc1", "adc_offset7", 0, zeroes, 33);
  write_param("rc2", "adc_offset0", 0, zeroes, 33);
  write_param("rc2", "adc_offset1", 0, zeroes, 33);
  write_param("rc2", "adc_offset2", 0, zeroes, 33);
  write_param("rc2", "adc_offset3", 0, zeroes, 33);
  write_param("rc2", "adc_offset4", 0, zeroes, 33);
  write_param("rc2", "adc_offset5", 0, zeroes, 33);
  write_param("rc2", "adc_offset6", 0, zeroes, 33);
  write_param("rc2", "adc_offset7", 0, zeroes, 33);

  /* stop the servo */
  write_param("rc1", "servo_mode", 0, zeroes, 8);
  write_param("rc2", "servo_mode", 0, zeroes, 8);

  /* sq1 fb to 0V = -8192 */
  write_param("rc1", "fb_const", 0, (void*)number, 8);
  write_param("rc2", "fb_const", 0, (void*)number, 8);
  mas_write_block("rca", "flx_lp_init", &one, 1);

  /* stop acq */
  kill_special = 1;
}

/* run a load curve */
static int do_ivcurve(uint32_t kickvalue, double kicktime, int kickwait,
    int start, int last, int step, double wait, const char *filename)
{
  mcedata_storage_t *st;
  FILE *stream;
  uint32_t data[16];
  char *auxfile;
  int bias, i, r = 0;

  /* no longer properly biased */
  state &= ~st_biased;

  /* kick */
  if (kick(start, memory.bias_kick_bias, kickvalue, kicktime, kickwait))
    return 1;

  /* runfile */
  auxfile = malloc(strlen(filename) + 6);
  sprintf(auxfile, "%s.run", filename);
  stream = fopen(auxfile, "w");
  if (stream == NULL) {
    bprintf(err, "Failed to open runfile");
    return 1;
  }
  dump_runfile(stream, (last - start) / step + 1);
  fclose(stream);

  /* bias file */
  sprintf(auxfile, "%s.bias", filename);
  stream = fopen(auxfile, "w");
  if (stream == NULL) {
    bprintf(err, "Failed to open bias file");
    return 1;
  }
  fputs("<tes_bias>\n", stream); /* why...? */

  /* set up the acq for the ramp */
  acq = malloc(sizeof(mce_acq_t));
  if ((st = mcedata_flatfile_create(filename, NULL)) == NULL) {
    bprintf(err, "Failed to create frameacq");
    r = 1;
    goto IV_DONE;
  }

  if ((r = mcedata_acq_create(acq, mas, 0, 0 /* rcs */, -1, st))) {
    bprintf(err, "Loadcurve acquisition failed: %s", mcelib_error_string(r));
    r = 1;
    goto IV_DONE;
  }

  /* don't screw up the stats */
  stat_veto = 1;

  /* bias loop */
  for (bias = start; bias >= last; bias += step) {
    /* set and record bias */
    if (bias < last)
      bias = last;
    for (i = 0; i < 16; ++i)
      data[i] = bias;
    write_param("tes", "bias", 0, data, 16);
    fprintf(stream, "%i\n", bias); 

    /* wait */
    if (check_wait(wait)) {
      r = 1;
      goto IV_DONE;
    }

    /* take a frame of data */
    acq_going = 1;

    while (!acq_stopped)
      usleep(1000);

    acq_going = 0;
    acq_stopped = 0;
  }

IV_DONE:
  stat_veto = 0;
  fclose(stream);
  free(auxfile);
  mcedata_acq_destroy(acq);
  free(acq);
  acq = NULL;
  return r;
}

static int ivcurve(void)
{
  char filename1[1024];
  uint32_t zero = 0;

  /* we burn the index number whether-or-not things are successful */
  sprintf(filename1, "/data%i/mce/current_data/iv_%04i", data_drive[0],
      ++memory.last_iv);
  mem_dirty = 1;

  /* disable integral clamping */
  write_param("rc1", "integral_clamp", 0, &zero, 1);
  write_param("rc2", "integral_clamp", 0, &zero, 1);

  int r = do_ivcurve(goal.kick, goal.kicktime, goal.kickwait, goal.start,
      goal.stop, goal.step, goal.wait, filename1);

  if (r == 0) { /* archive it */
    int d;
    char filename2[90], filename3[90];
    char basedir[] = "/data#/mce/ivcurves";

    char *argv[] = {"/bin/cp", filename1, filename2, filename3, basedir, NULL};

    sprintf(filename1, "/data%c/mce/current_data/iv_%04i", data_drive[0] + '0',
        memory.last_iv);
    sprintf(filename2, "/data%c/mce/current_data/iv_%04i.run",
        data_drive[0] + '0', memory.last_iv);
    sprintf(filename3, "/data%c/mce/current_data/iv_%04i.bias",
        data_drive[0] + '0', memory.last_iv);

    for (d = 0; d < 4; ++d)
      if (!disk_bad[d]) {
        basedir[5] = d + '0';
        exec_and_wait(sched, none, argv[0], (char**)argv, 100, 0, NULL);
        bprintf(info, "Archived IV curve as %s/iv_%04i", basedir,
            memory.last_iv);
      }

    /* analyse */
    if (goal.apply)
      pick_biases(0, goal.apply == 2, 0);
  }

  return 0;
}

static int lcloop(void)
{
  int r;
  char filename[290];
  uint32_t zero = 0;

  /* disable integral clamping */
  write_param("rc1", "integral_clamp", 0, &zero, 1);
  write_param("rc2", "integral_clamp", 0, &zero, 1);

  /* this just alternates between Al and Ti load curves forever */
  for (;;) {
    /* AL */
    sprintf(filename, "/data%i/mce/current_data/loadcurve_Al_MPC_%li",
        data_drive[0], (long)time(NULL));

    bprintf(info, "LCLOOP: %s", filename);
    r = do_ivcurve(6554, 0, 300, 32000, 0, -50, 0.06, filename);

    if (r)
      return 1;

    /* wait */
    if (check_wait(900))
      return 1;

    /* Ti */
    sprintf(filename, "/data%i/mce/current_data/loadcurve_Ti_MPC_%li",
        data_drive[0], (long)time(NULL));

    bprintf(info, "LCLOOP: %s", filename);
    r = do_ivcurve(6554, 0, 300, 32000, 0, -50, 0.06, filename);

    if (r)
      return 1;

    /* wait */
    if (check_wait(900))
      return 1;
  }

  return 0;
}

/* if experiment.cfg doesn't exist in current_data, force a flush to disk
 * of the one we have */
static void ensure_experiment_cfg(void)
{
  struct stat buf;
  char file[] = "/data#/mce/current_data/experiment.cfg";
  file[5] = data_drive[0] + '0';

  /* if stat fails, force a flush of experiment.cfg */
  if (stat(file, &buf))
    flush_experiment_cfg(1);
}

/* tes bias step via internal commanding */
static int bias_step(void)
{
  uint32_t data[32] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
  };
  uint32_t bc2 = 0x08;
  uint32_t mod_val = 0x27;
  uint32_t thirty_two = 32;
  uint32_t zero = 0;
  uint32_t two = 2;
  uint32_t step = goal.step;
  uint32_t period, phase;

  /* set up for the internal ramp */

  /* turn off internal commanding */
  mas_write_range("cc", "internal_cmd_mode", 0, &zero, 1);

  /* set card and parameter (bc2 mod_val) */
  mas_write_range("cc", "ramp_card_addr", 0, &bc2, 1);
  mas_write_range("cc", "ramp_param_id", 0, &mod_val, 1);

  /* we have to ramp all 32 mod_vals, but only the top 16 of them will
   * be used because of enbl_flux_fb_mod */
  mas_write_range("cc", "ramp_step_data_num", 0, &thirty_two, 1);

  /* min val is always zero */
  mas_write_range("cc", "ramp_min_val", 0, &zero, 1);

  /* max val and step size are both the commanded step value */
  mas_write_range("cc", "ramp_max_val", 0, &step, 1);
  mas_write_range("cc", "ramp_step_size", 0, &step, 1);

  /* the width of the thing in ARZs -- goal.wait is the whole cycle frequency in
   * hertz
   *
   * we calculate the period in multiples of data frame rate so we're
   * synchronous with DVs and can phase shift away from the internal command
   * collision */
  period = (uint32_t)(50e6 / row_len / num_rows / data_rate / goal.wait / 2
      + 0.5) * data_rate;
  mas_write_range("cc", "ramp_step_period", 0, &period, 1);

  /* phase shift by 120 degrees from the DV pulse to avoid collision */
  phase = data_rate / 3;
  mas_write_range("cc", "ramp_step_phase", 0, &phase, 1);

  /* mod_val-ify the upper 16 elements of bc2/flux_fb = tes/bias */
  mas_write_range("bc2", "enbl_flux_fb_mod", 0, data, 32);

  /* start the ramp */
  bprintf(info, "Starting bias step");
  mas_write_range("cc", "internal_cmd_mode", 0, &two, 1);

  /* wait for completion */
  check_wait(goal.total);

  /* stop ramping */
  bprintf(info, "Stopping bias step");
  mas_write_range("cc", "internal_cmd_mode", 0, &zero, 1);

  /* disable the bias offset */
  memset(data, 0, sizeof(uint32_t) * 32);
  mas_write_range("bc2", "enbl_flux_fb_mod", 0, data, 32);

  return 0;
}

static int partial_iv(void)
{
  int offset;
  uint32_t biases[16];
  uint32_t saved_bias[16];
  int i, r;

  uint32_t saved_user_word;

  /* remember userword */
  fetch_param("cc", "user_word", 0, &saved_user_word, 1);

  /* remember biases */
  fetch_param("tes", "bias", 0, saved_bias, 16);

  /* kick */
  if (kick(goal.start, memory.bias_kick_bias, goal.kick, goal.kicktime,
        goal.kickwait))
  {
    return 1;
  }

  /* ramp down from offset -- goal.step is always negative */
  for (offset = goal.start; offset > 0; offset += goal.step)
  {
    uint32_t user_word = offset;
    /* set bias */
    for (i = 0; i < 16; ++i)
      biases[i] = saved_bias[i] + offset;
    write_param("cc", "user_word", 0, &user_word, 1);
    write_param("tes", "bias", 0, biases, 16);

    /* wait */
    if (check_wait(goal.wait)) {
      r = 1;
      break;
    }
  }

  /* return to normal biases; also user_word */
  write_param("cc", "user_word", 0, &saved_user_word, 1);
  write_param("tes", "bias", 0, saved_bias, 16);

  if (r == 0 && check_wait(0.1))
    return 1;

  /* reset servo */
  flux_loop_init(0);

  return r;
}

static int bias_ramp(void)
{
  uint32_t bias;
  uint32_t biases[16];
  uint32_t saved_bias[16];
  int i, r;

  /* remember biases */
  fetch_param("tes", "bias", 0, saved_bias, 16);

  /* kick */
  if (kick(goal.start, memory.bias_kick_bias, goal.kick, goal.kicktime,
        goal.kickwait))
  {
    return 1;
  }

  /* ramp -- goal.step is always negative */
  for (bias = goal.start; bias > goal.stop; bias += goal.step)
  {
    /* set bias */
    for (i = 0; i < 16; ++i)
      biases[i] = bias;
    write_param("tes", "bias", 0, biases, 16);

    if (check_wait(0.1))
      return 1;

    /* reset servo */
    flux_loop_init(0);

    /* wait */
    if (check_wait(goal.wait)) {
      /* return to normal biases */
      write_param("tes", "bias", 0, saved_bias, 16);
      return 1;
    }
  }

  /* return to normal biases */
  write_param("tes", "bias", 0, saved_bias, 16);

  if (check_wait(0.1))
    return 1;

  /* reset servo */
  flux_loop_init(0);

  return r;
}

static int get_tune_dir(char buffer[100])
{
  char linkval[100];
  char link[] = "/data#/mce/last_squid_tune";
  link[5] = data_drive[0] + '0';

  if (readlink(link, linkval, 100) < 0) {
    bprintf(warning, "Bad link: %s", link);
    return 1;
  }

  strcpy(buffer, dirname(linkval));
  return 0;
}

/* update an experiment.cfg parameter based on output from the tuning evaluation
 * script
 */
static void check_tune_update(char *line)
{
  char *ptr;
  char *endptr;
  uint32_t data[NUM_COL * NUM_ROW];
  int offset;
  int n;
  /* this should have the form: <param_name> <offset> <data>... */
  bprintf(info, "cfg-do: %s", line);

  /* find the first whitespace character */
  for (ptr = line; *ptr && !isblank(*ptr); ++ptr)
    ;

  /* zero it */
  *ptr = 0;

  /* offset */
  offset = (int)strtoul(ptr + 1, &endptr, 10);

  /* get the data */
  n = 0;
  do {
    /* strip leading whitespace */
    for (ptr = endptr; *ptr && !isblank(*ptr); ++ptr)
      ;

    if (!*ptr)
      break;

    /* convert */
    data[n++] = (uint32_t)strtoul(ptr, &endptr, 10);

    if (n == NUM_COL * NUM_ROW)
      break;
  } while (*endptr);

  /* run it */
  if (n == 1)
    cfg_set_int(line, offset, data[0]);
  else if (n > 0)
    cfg_set_intarr(line, offset, data, n);
  else
    bprintf(warning, "ignoring malformed eval_squid_tune output starting %s",
        line);
}

#define CHECK_TUNE_ERR (1 << 8)
#define FLOAT_TO_STRING(n) sprintf(n, "%.9f", memory.n[stage])
#define INT_TO_STRING(n) sprintf(n, "%i", memory.n[stage])
static int check_tune(const char *lst_dir, const char *ref_tune_dir,
    const char *stage_name, int stage)
{
  struct mcp_proc *p;
  int p_stdout = -1;
  int p_stderr = -1;
  char obuffer[8192], ebuffer[8192];
  char p2p_abs_thresh[40];
  char p2p_rel_thresh[40];
  char slope_abs_thresh[40];
  char slope_rel_thresh[40];
  char range_abs_thresh[40];
  char range_rel_thresh[40];
  char count_thresh[40];
  char fail_thresh[40];
  char ramp_shift[40];
  char ramp_buffer[40];
  char *opos = obuffer;
  char *epos = ebuffer;
  int argc = 25;
  const char *argv[30] = { "/data/mas/bin/eval_squid_tune",
    lst_dir, ref_tune_dir, stage_name, "--p2p-abs-thresh", p2p_abs_thresh,
    "--p2p-rel-thresh", p2p_rel_thresh, "--slope-abs-thresh", slope_abs_thresh,
    "--slope-rel-thresh", slope_rel_thresh, "--range-abs-thresh",
    range_abs_thresh, "--range-rel-thresh", range_rel_thresh, "--count-thresh",
    count_thresh, "--fail-thresh", fail_thresh, "--ramp-shift", ramp_shift,
    "--ramp-buffer", ramp_buffer, "--criteria"
  };
  int nfds = 0;
  int proc_state;
  struct timeval seltime;
  fd_set fdset;
  ssize_t n;

  /* stringify numbers */
  FLOAT_TO_STRING(p2p_abs_thresh);
  FLOAT_TO_STRING(p2p_rel_thresh);
  FLOAT_TO_STRING(slope_abs_thresh);
  FLOAT_TO_STRING(slope_rel_thresh);
  FLOAT_TO_STRING(range_abs_thresh);
  FLOAT_TO_STRING(range_rel_thresh);
  INT_TO_STRING(count_thresh);
  INT_TO_STRING(fail_thresh);
  FLOAT_TO_STRING(ramp_shift);
  INT_TO_STRING(ramp_buffer);

  /* add criteria */
  if (memory.check_count[stage])
    argv[argc++] = "count";
  if (memory.check_range[stage])
    argv[argc++] = "range";
  if (memory.check_slope[stage])
    argv[argc++] = "slope";
  if (memory.check_p2p[stage])
    argv[argc++] = "p2p";

  /* terminate */
  argv[argc] = 0;

  p = start_proc(argv[0], (char**)argv, 0, 1, NULL, &p_stdout, &p_stderr,
      &kill_special);
  if (p == NULL)
    return CHECK_TUNE_ERR;

  /* capture the output */
  while ((proc_state = check_proc(p)) == 0) {
    FD_ZERO(&fdset);
    FD_SET(p_stderr, &fdset);
    FD_SET(p_stdout, &fdset);

    seltime.tv_sec = 1;
    seltime.tv_usec = 0;

    n = select(nfds, &fdset, NULL, NULL, &seltime);

    /* parrot the child's standard error */
    if (FD_ISSET(p_stderr, &fdset)) {
      n = read(p_stderr, epos, 1);
      if (n < 0)
        break;
      else if (n > 0) {
        if (*epos == '\n' || (epos - ebuffer) >= 8192) {
          *epos = 0;
          epos = ebuffer;
          bprintf(sched, "%s", ebuffer);
        } else
          epos++;
      }
    }

    /* collect stdout and do stuff with it when we have a line */
    if (FD_ISSET(p_stdout, &fdset)) {
      n = read(p_stdout, opos, 1);
      if (n < 0)
        break;
      else if (n > 0) {
        if (*opos == '\n' || (opos - obuffer) >= 8192) {
          *opos = 0;
          opos = obuffer;
          check_tune_update(obuffer);
        } else
          opos++;
      }
    }
  }

  if (proc_state == 1) {
    /* clear the buffers */
    for (;;) {
      n = read(p_stderr, epos, 1);
      if (n <= 0)
        break;
      else if (n > 0) {
        if (*epos == '\n' || (epos - ebuffer) >= 8192) {
          *epos = 0;
          epos = ebuffer;
          bprintf(sched, "%s", ebuffer);
        } else
          epos++;
      }
    }

    for (;;) {
      n = read(p_stdout, opos, 1);
      if (n <= 0)
        break;
      else if (n > 0) {
        if (*opos == '\n' || (opos - obuffer) >= 8192) {
          *opos = 0;
          opos = obuffer;
          check_tune_update(obuffer);
        } else
          opos++;
      }
    }
  }

  return stop_proc(p, 0, 0, 0, 0);
}

/* run a tuning */
#define MAX_STAGE_TRIES 7
static int tune(void)
{
  const char *first_stage_tune[] = {
    "--first-stage=sa_ramp", "--first-stage=sq2_servo",
    "--first-stage=sq1_servo", "--first-stage=sq1_ramp"
  };

  const char *last_stage_tune[] = {
    "--last-stage=sa_ramp", "--last-stage=sq2_servo",
    "--last-stage=sq1_servo", "--last-stage=sq1_ramp"
  };

  const char *stage_name[] = {"sa_ramp", "sq2_servo", "sq1_servo", "sq1_ramp",
    NULL};

  int old_sa_ramp_bias = 0, old_sq2_servo_bias_ramp = 0;
  int old_sq1_servo_bias_ramp = 0;
  int local_tune_force_biases = goal.force;
  char ref_tune_dir[100];
  char stage_dirs[4][MAX_STAGE_TRIES][100];
  int stage, r = 0;
  int stage_tries[4];
  int nt;

  const char *argv[5] = { MAS_SCRIPT "/auto_setup", "--set-directory=0",
    NULL /* first stage */, NULL /* last stage */, NULL };

  ensure_experiment_cfg();

  if (local_tune_force_biases) {
    old_sa_ramp_bias = cfg_get_int("sa_ramp_bias", 0);
    cfg_set_int("sa_ramp_bias", 0, 1);

    old_sq2_servo_bias_ramp = cfg_get_int("sq2_servo_bias_ramp", 0);
    cfg_set_int("sq2_servo_bias_ramp", 0, 1);

    old_sq1_servo_bias_ramp = cfg_get_int("sq1_servo_bias_ramp", 0);
    cfg_set_int("sq1_servo_bias_ramp", 0, 1);
  }

  /* global tuning attempt counter */
  for (nt = 0; nt < memory.tune_global_tries; ++nt) {
    /* run each stage in turn */
    for (stage = 0; stage < 4; ++stage) {
      stage_tries[stage] = 0;
      flush_experiment_cfg(0);

      argv[2] = first_stage_tune[stage];
      argv[3] = last_stage_tune[stage];
      r = exec_and_wait(sched, none, argv[0], (char**)argv, 0, 1,
          &kill_special);

      stage_tries[stage]++;

      if (r)
        break;

      if (get_tune_dir(stage_dirs[stage][stage_tries[stage] - 1])) {
        r = 1;
        break;
      }

      /* skip check */
      if (memory.tune_check_off)
        continue;

      sprintf(ref_tune_dir, "/data%c/mce/tuning/%04i/%s", data_drive[0] + '0',
          memory.ref_tune, stage_name[stage]);

      int check = check_tune(stage_dirs[stage][stage_tries[stage] - 1],
          ref_tune_dir, stage_name[stage], stage);

      if (WIFEXITED(check)) {
        switch (WEXITSTATUS(check)) {
          case 2: /* redo */
            if (stage_tries[stage] < memory.tune_tries[stage])
              stage--;
            break;
          case 0: /* success */
            break;
          default: /* irrecoverable error */
            r = 1;
            break;
        }
      } else
        r = 1;

      if (r)
        break;
    }

    /* abort */
    if (kill_special) {
      r = 1;
      break;
    }

    /* done with global tries */
    if (r == 0)
      break;
  }

  if (r == 0) { /* archive it */
    int d, i;
    char tuning_dir[100];
    char *stage_part = tuning_dir + 23;

    /* increment tuning */
    memory.last_tune++;
    mem_dirty = 1;

    for (d = 0; d < 4; ++d) 
      if (!disk_bad[d]) {
        sprintf(tuning_dir, "/data#/mce/tuning/%04i/", memory.last_tune);
        tuning_dir[5] = '0' + d;
        if (mkdir(tuning_dir, 0777) == 0) {
          /* copy all the things to all the places */
          for (stage = 0; stage <= 4; ++stage)
            for (i = 0; i < stage_tries[stage]; ++i) {
              if (i == stage_tries[stage] - 1) /* the good one */
                strcpy(stage_part, stage_name[stage]);
              else 
                sprintf(stage_part, "%s_try%i", stage_name[stage], i + 1);
              argv[0] = "/bin/cp";
              argv[1] = "-r";
              argv[2] = stage_dirs[stage][i];
              argv[3] = tuning_dir;
              argv[4] = NULL;
              exec_and_wait(sched, none, argv[0], (char**)argv, 100, 0, NULL);
              bprintf(info, "Archived tuning %s as %s\n", stage_dirs[stage][i],
                  tuning_dir);
            }
        }
      }
  }

  if (goal.force) {
    cfg_set_int("sa_ramp_bias", 0, old_sa_ramp_bias);
    cfg_set_int("sq2_servo_bias_ramp", 0, old_sq2_servo_bias_ramp);
    cfg_set_int("sq1_servo_bias_ramp", 0, old_sq1_servo_bias_ramp);
  }

  /* reset */
  flush_experiment_cfg(1);

  /* apply, if requested */
  if (goal.apply)
    cfg_apply_tuning(memory.last_tune, 0);

  tuning_status = (r << 15) | ((nt + 1) << 12) | (stage_tries[3] << 9) |
    (stage_tries[2] << 6) | (stage_tries[1] << 3) | (stage_tries[0] << 0);

  return r ? 1 : 0;
}

static int check_set_sync(void)
{
  uint32_t v = 1;

  if (memory.sync_veto) {
    v = 0; /* vetoed off */
  } else {
    /* ignore the return value here: if the sync box isn't on, this will usually
     * time out as the CC waits for lock */
    mas_write_range("cc", "select_clk", 0, &v, 1);

    sleep(1);

    if (mas_read_range("cc", "select_clk", 0, &v, 1))
      return 1;

    /* now select_clk is one if the sync box is working, zero otherwise
     * co-incidentally we set config_sync to those same values to configure
     * the sync box */
  }

  cfg_set_int("config_sync", 0, v);
  if (v)
    state |= st_syncon;
  else {
    state &= ~st_syncon;
    if (memory.sync_veto)
      bprintf(warning, "Sync box vetoed on CC.");
    else
      bprintf(warning, "CC reports sync box unresponsive.");
  }

  return 0;
}

static int reconfig(void)
{
  char gaini[] = "gaini0";
  int c;
  uint32_t u32;

  /* check whether the sync box is useable */
  if (check_set_sync()) {
    comms_lost = 1;
    return 1;
  }

  /* ensure we're synced */
  flush_experiment_cfg(0);

  if (exec_and_wait(sched, none, MAS_SCRIPT "/mce_make_config", NULL, 100, 0,
        NULL))
  {
    drive_error[data_drive[0]] = 1; /* must be a drive error */
    state &= ~st_drives;
    return 1;
  }

  if (exec_and_wait(sched, none, MAS_SCRIPT "/mce_reconfig", NULL, 100, 0,
        NULL))
  {
    comms_lost = 1;
    return 1;
  }

  /* update tile heater data */
  fetch_param("heater", "bias", 0, &u32, 1);
  slow_dat.tile_heater = u32;

  /* update the clamp values */
  fetch_param("rc1", "integral_clamp", 0, iclamp + 0, 1);
  fetch_param("rc2", "integral_clamp", 0, iclamp + 1, 1);

  /* update igains */
  for (c = 0; c < NUM_COL; ++c) {
    gaini[5] = (c % 8) + '0';
    fetch_param((c < 8) ? "rc1" : "rc2", gaini, 0, igain + c * NUM_ROW,
        NUM_ROW);
  }

  return 0;
}

/* stop rcs ret_dat */
static int stopacq(void)
{
  mce_param_t p;
  memset(&p, 0, sizeof(p));

  p.card.id[0] = 0x0b; /* rcs */
  p.card.card_count = 1;
  p.param.id = 0x16; /* ret_dat */
  p.param.count = 1;

  int e = mcecmd_stop_application(mas, &p);

  if (e)
    bprintf(err, "Error stopping acquisition: error #%i", e);

  return e ? 1 : 0;
}

void crash_stop(int sig)
{
  terminate = 1;
  bprintf(err, "Crash stop.");
  /* stop acq and zero bias */
  if (mas && (state & st_active)) {
    stopacq();
  }
  signal(sig, SIG_DFL);
  raise(sig);
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
        break;
      case dt_dsprs:
        slow_veto++; /* watchdog */
        ret = mcecmd_interface_reset(mas);
        slow_veto--;
        if (ret) {
          bprintf(err, "Error resetting DSP: error #%i", ret);
          dt_error = 1;
        } else
          dt_error = 0;
        break;
      case dt_mcers:
        slow_veto++; /* watchdog */
        ret = mcecmd_hardware_reset(mas);
        slow_veto--;
        if (ret) {
          bprintf(err, "Error resetting MCE: error #%i", ret);
          dt_error = 1;
        } else if (mce_check_cards()) {
          bprintf(err, "Card check failed");
          dt_error = 1;
        } else
          dt_error = 0;
        break;
      case dt_reconfig:
        dt_error = reconfig();
        break;
      case dt_kick:
        dt_error = kick(KICK_DONT_BIAS, memory.bias_kick_bias,
            memory.bias_kick_val, 0, 0);
        break;
      case dt_status:
        dt_error = mce_status();
        break;
      case dt_acqcnf:
        dt_error = acq_conf();
        break;
      case dt_startacq:
        acq_going = ACQ_FRAMECOUNT;
        stat_reset = 1;
        dt_error = 0;
        break;
      case dt_stop:
        dt_error = stopacq();
        break;
      case dt_fakestop:
        ret = mcedata_fake_stopframe(mas);
        if (ret) {
          bprintf(err, "Error during fake stop: error #%i", ret);
          dt_error = 1;
        } else
          dt_error = 0;
        break;
      case dt_empty:
        ret = mcedata_empty_data(mas);
        if (ret) {
          bprintf(err, "Error emptying data queue: error #%i", ret);
          dt_error = 1;
        } else
          dt_error = 0;
        break;
      case dt_delacq:
        if (acq) {
          mcedata_acq_destroy(acq);
          free(acq);
          acq = NULL;
        }
        dt_error = 0;
        break;
      case dt_autosetup:
        dt_error = tune();
        break;
      case dt_ivcurve:
        dt_error = ivcurve();
        break;
      case dt_lcloop:
        dt_error = lcloop();
        break;
      case dt_stopmce:
        stop_mce();
        break;
      case dt_bstep:
        bias_step();
        break;
      case dt_bramp:
        bias_ramp();
        break;
      case dt_partial:
        partial_iv();
        break;
      case dt_rstsrvo:
        dt_error = flux_loop_init(2);
        break;
    }
    data_tk = dt_idle;

    deblock();

    /* check for acq termination */
    if (acq_going && acq_stopped) {
      acq_going = 0;
      acq_stopped = 0;
      state &= ~st_retdat;
    }
    usleep(10000); /* a frame */
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
      r = mcedata_acq_go(acq, acq_going); /* this blocks */
      if (r)
        bprintf(err, "Acquisition error: %s", mcelib_error_string(r));
      acq_stopped = 1;
      acq_init = 0; /* just in case */
    }
  }

  return NULL;
}
