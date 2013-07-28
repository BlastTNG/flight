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
#include <sys/stat.h>
#include <libgen.h>
#include <string.h>
#include <time.h>
#include <signal.h>

static unsigned cmd_err = 0;

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

/* check drives when acq finishes */
int acq_check_drives = 0;

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

/* veto on mce communication */
int mceveto = 1;

/* number of frames in an acq_go */
#define ACQ_FRAMECOUNT 1000000000L /* a billion frames = 105 days */

/* wait with kill detection */
static int check_wait(double wait)
{
  double i;

  for (i = 0; i < wait; i += 0.01) {
    if (kill_special)
      return 1;
    usleep(10000);
  }
  return kill_special ? 1 : 0;
}

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
static int mas_read_range(const char *card, const char *block, uint32_t *data,
    size_t num, int offset)
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
  return mas_read_range(card, block, data, num, 0);
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

  {
    int i;
    char *ptr, params[1000];
    ptr = params;
    for (i = 0; i < count; ++i)
      ptr += sprintf(ptr, "%u ", data[i]);
    bprintf(info, "write_param: %s/%s+%i(%i) [ %s]", card, param, offset,
        count, params);
  }

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
    if (mas_read_range(card, param, data, count, offset) == 0)
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
static void read_param(const char *card, const char *param, int offset,
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
      "  <DATA_FRAMEACQ> %li\n"
      "  <CTIME> %li\n"
      "  <ARRAY_ID> x%i"
      "  <HOSTNAME> x%i.spider\n"
      "</FRAMEACQ>\n",
      acq_time, ACQ_FRAMECOUNT, acq_time, nmce + 1, nmce + 1);

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
      dump_runfile(stream);
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
      dump_runfile(stream);
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
      dump_runfile(stream);
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
      drive_map = (drive_map & ~DRIVE0_MASK) | DRIVE0_UNMAP;
      acq_check_drives = 1;
      break;
    case 2: /* secondary */
      bprintf(info, "secondary drive failed; stopping acq");
      drive_map = (drive_map & ~DRIVE1_MASK) | DRIVE1_UNMAP;
      acq_check_drives = 1;
      break;
    case 3: /* teritary */
      bprintf(info, "tertiary drive failed; stopping acq");
      drive_map = (drive_map & ~DRIVE2_MASK) | DRIVE2_UNMAP;
      acq_check_drives = 1;
      break;
  }
  if (drive_map == (DRIVE0_UNMAP | DRIVE1_UNMAP | DRIVE2_UNMAP)) {
    /* uh-oh */
    state &= ~st_drives;
  }
  return 1; /* stop this one */
}

/* restart the servo */
static int flux_loop_init(double wait)
{
  uint32_t one = 1;
  bprintf(info, "Flux Loop Init");
  mas_write_block("rca", "flx_lp_init", &one, 1);
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
        "/data/mas/etc/mpc.lnk");
    if (st == NULL) {
      bprintf(err, "Couldn't set up file sequencer for data0");
      goto ACQ_CONFIG_ERR;
    }
    if (mcedata_multisync_add(acq, st)) {
      bprintf(err, "Couldn't append file sequencer for data0");
      goto ACQ_CONFIG_ERR;
    }
  }

  if ((drive_map & DRIVE1_MASK) != DRIVE1_UNMAP) {
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

  if ((drive_map & DRIVE2_MASK) != DRIVE2_UNMAP) {
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

  bprintf(info, "Starting acquisition #%li", acq_time);

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
  int i;
  char data_root[] = "/data#/mce";
  char *argv[] = {"set_directory", data_root, NULL};
  for (i = 0; i < 4; ++i) {
    if (slow_dat.df[i] == 0)
      continue;

    data_root[5] = i + '0';
    write_array_id(i);
    if (exec_and_wait(sched, none, MAS_SCRIPT "/set_directory", argv, 20, 1,
        NULL))
    {
      return 1;
    }
  }

  return 0;
}

#define KICK_DONT_BIAS 4000000000U
static int kick(uint32_t bias, uint32_t value, int wait)
{
  if (value > 0) {
    int i;
    uint32_t zero = 0;
    uint32_t data[16];

    if (bias != KICK_DONT_BIAS) {
      /* bias to the start */
      for (i = 0; i < 16; ++i)
        data[i] = bias;
      write_param("tes", "bias", 0, data, 16);
    }

    write_param("heater", "bias", 0, &value, 1);
    sleep(2);
    write_param("heater", "bias", 0, &zero, 1);

    /* wait */
    if (check_wait(wait))
      return 1;
  }
  /* flx_lp_init */
  return flux_loop_init(2);
}

/* returns non-zero if something was popped */
static int pop_block(void)
{
  int new_tail = (blockq_tail + 1) % BLOCKQ_SIZE;

  /* no pending requests */
  if (blockq_head == blockq_tail)
    return 0;

  if (blockq[new_tail].raw == 2) { /* asynchronous kick request */
    kick(KICK_DONT_BIAS, blockq[new_tail].o, 30);
  } else if (blockq[new_tail].raw)
    mas_write_range(blockq[new_tail].c, blockq[new_tail].p, blockq[new_tail].o,
        blockq[new_tail].d, blockq[new_tail].n);
  else
    write_param(blockq[new_tail].c, blockq[new_tail].p, blockq[new_tail].o,
        blockq[new_tail].d, blockq[new_tail].n);

  free(blockq[new_tail].c);
  free(blockq[new_tail].p);

  blockq_tail = new_tail;

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
static int do_ivcurve(uint32_t kickbias, int kickwait, int start, int last,
    int step, double wait, const char *filename)
{
  char start_arg[30], step_arg[30], count_arg[30], step_wait[30], last_arg[30];
  const char *argv[] = { MAS_SCRIPT "/ramp_tes_bias", filename, "s", start_arg,
    step_arg, count_arg, step_wait, step_wait, start_arg, "0", last_arg, NULL };

  sprintf(start_arg, "%i", start);
  sprintf(last_arg, "%i", last);
  sprintf(step_arg, "%i", step);
  sprintf(count_arg, "%i", (last - start) / step + 1);
  sprintf(step_wait, "%f", wait);

  /* kick */
  if (kick(start, kickbias, kickwait))
    return 1;

  /* do the ramp */
  return exec_and_wait(sched, none, argv[0], (char**)argv, 0, 1, &kill_special);
}

/* run and archive an iv curve */
static int ivcurve(void)
{
  char filename1[90];

  /* we burn the index number whether-or-not things are successful */
  sprintf(filename1, "iv_%04i", ++memory.last_iv);
  mem_dirty = 1;

  int r = do_ivcurve(goal.kick, goal.kickwait, goal.start, goal.stop, goal.step,
      goal.wait, filename1);

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
      if (slow_dat.df[d]) {
        basedir[5] = d + '0';
        exec_and_wait(sched, none, argv[0], (char**)argv, 100, 0, NULL);
        bprintf(info, "Archived IV curve as %s/iv_%04i", basedir,
            memory.last_iv);
      }
  }

  return 0;
}

static int lcloop(void)
{
  int r;
  char filename[90];

  /* this just alternates between Al and Ti load curves forever */
  for (;;) {
    /* AL */
    sprintf(filename, "loadcurve_Al_MPC_%li", (long)time(NULL));

    bprintf(info, "LCLOOP: %s", filename);
    r = do_ivcurve(6554, 300, 32000, 0, -50, 0.06, filename);

    if (r)
      return 1;

    /* wait */
    if (check_wait(900))
      return 1;

    /* Ti */
    sprintf(filename, "loadcurve_Ti_MPC_%li", (long)time(NULL));

    bprintf(info, "LCLOOP: %s", filename);
    r = do_ivcurve(6554, 300, 32000, 0, -50, 0.06, filename);

    if (r)
      return 1;

    /* wait */
    if (check_wait(900))
      return 1;
  }

  return 0;
}

static const char *first_stage_tune[] = {
  "--first-stage=sa_ramp", "--first-stage=sq2_servo",
  "--first-stage=sq1_servo", "--first-stage=sq1_ramp",
  "--first-stage=sq1_ramp_tes", "--first-stage=operate"
};

static const char *last_stage_tune[] = {
  "--last-stage=sa_ramp", "--last-stage=sq2_servo",
  "--last-stage=sq1_servo", "--last-stage=sq1_ramp",
  "--last-stage=sq1_ramp_tes", "--last-stage=operate"
};

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

/* bias steppy */
static int bias_step(void)
{
  int i, j;
  uint32_t bias[16];

  /* get biases */
  fetch_param("tes", "bias", 0, bias, 16);

  /* step up */
  for (j = 0; j < goal.stop; ++j) {
    bprintf(info, "Bias Step up");
    for (i = 0; i < 16; ++i)
      bias[i] += goal.step;
    write_param("tes", "bias", 0, bias, 16);

    if (check_wait(goal.wait))
      return 1;

    /* step down */
    bprintf(info, "Bias Step down");
    for (i = 0; i < 16; ++i)
      bias[i] -= 2 * goal.step;
    write_param("tes", "bias", 0, bias, 16);

    if (check_wait(goal.wait))
      return 1;

    /* back to normal */
    for (i = 0; i < 16; ++i)
      bias[i] += goal.step;
  }
  bprintf(info, "Bias Step finished");
  write_param("tes", "bias", 0, bias, 16);

  return 0;
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
  if (kick(goal.start, goal.kick, goal.kickwait))
    return 1;

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

/* run a tuning */
static int tune(void)
{
  int old_sa_ramp_bias = 0, old_sq2_servo_bias_ramp = 0;
  int old_sq1_servo_bias_ramp = 0;
  const char *argv[] = { MAS_SCRIPT "/auto_setup", "--set-directory=0",
    first_stage_tune[goal.start], last_stage_tune[goal.stop], NULL };
  int local_tune_force_biases = goal.force;

  ensure_experiment_cfg();

  if (local_tune_force_biases) {
    old_sa_ramp_bias = cfg_get_int("sa_ramp_bias", 0);
    cfg_set_int("sa_ramp_bias", 0, 1);

    old_sq2_servo_bias_ramp = cfg_get_int("sq2_servo_bias_ramp", 0);
    cfg_set_int("sq2_servo_bias_ramp", 0, 1);

    old_sq1_servo_bias_ramp = cfg_get_int("sq1_servo_bias_ramp", 0);
    cfg_set_int("sq1_servo_bias_ramp", 0, 1);
  }
  flush_experiment_cfg(0);

  int r = exec_and_wait(sched, none, MAS_SCRIPT "/auto_setup", (char**)argv,
      0, 1, &kill_special);

  if (r == 0) { /* archive it */
    int d;
    char *dir;
    char lst[100];
    char link[] = "/data#/mce/last_squid_tune";
    link[5] = data_drive[0] + '0';

    if (readlink(link, lst, 100) < 0) {
      bprintf(warning, "Bad link: %s", link);
      r = 1;
    } else {
      dir = dirname(lst);
      char tuning_dir[100];

      /* increment tuning */
      memory.last_tune++;
      mem_dirty = 1;

      /* copy to all available drives */
      sprintf(tuning_dir, "/data#/mce/tuning/%04i", memory.last_tune);
      argv[0] = "/bin/cp";
      argv[1] = "-r";
      argv[2] = dir;
      argv[3] = tuning_dir;
      argv[4] = NULL;
      for (d = 0; d < 4; ++d)
        if (slow_dat.df[d]) {
          tuning_dir[5] = '0' + d;
          exec_and_wait(sched, none, argv[0], (char**)argv, 100, 0, NULL);
          bprintf(info, "Archived tuning as %s\n", tuning_dir);
        }
    }
  }

  if (goal.force) {
    cfg_set_int("sa_ramp_bias", 0, old_sa_ramp_bias);
    cfg_set_int("sq2_servo_bias_ramp", 0, old_sq2_servo_bias_ramp);
    cfg_set_int("sq1_servo_bias_ramp", 0, old_sq1_servo_bias_ramp);
  }

  /* don't apply this tuning, I guess ... ? */
  flush_experiment_cfg(1);

  return r ? 1 : 0;
}

static int reconfig(void)
{
  uint32_t u32;

  if (exec_and_wait(sched, none, MAS_SCRIPT "/mce_reconfig", NULL, 100, 1,
        NULL))
  {
    comms_lost = 1;
    return 1;
  }

  /* update tile heater data */
  read_param("heater", "bias", 0, &u32, 1);
  slow_dat.tile_heater = u32;

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
  if (mas && !mceveto) {
    stopacq();
    stop_mce();
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
      case dt_status:
        dt_error = mce_status();
        break;
      case dt_acqcnf:
        dt_error = acq_conf();
        break;
      case dt_startacq:
        acq_going = 1;
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
        if (acq)
          mcedata_acq_destroy(acq);
        acq = NULL;
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
    }
    data_tk = dt_idle;

    if (!mceveto)
    /* pop a block from the queue, if there are any,
     * otherwise, poll the temperature, if requested */
      if (!pop_block() && mas_get_temp) {
        fetch_param("cc", "box_temp", 0, (uint32_t*)&box_temp, 1);
        mas_get_temp = 0;
      }

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
      r = mcedata_acq_go(acq, ACQ_FRAMECOUNT); /* blocks */
      if (r)
        bprintf(err, "Acquisition error: %s", mcelib_error_string(r));
      else
        bprintf(info, "Acquisition stopped");
      acq_stopped = 1;
      acq_init = 0; /* just in case */
      if (acq_check_drives) {
        state &= ~st_drives;
        acq_check_drives = 0;
      }
    }
  }

  return NULL;
}
