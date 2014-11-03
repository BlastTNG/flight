/* MPC: MCE-PCM communicator
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include "bset.h"
#include "mpc.h"
#include "mce_frame.h"
#include "mputs.h"

int new_blob_type = BLOB_NONE;
int blob_type = BLOB_NONE;
int blob_size;

/* these can be used to pass data to the blobber */
char blob_source[1024];
int blob_data[N_BLOB_DATA];

/* dictionary-compresses the given stream into the blob buffer */
#define DC_IGNORE_SPACE 1
int dict_compress(unsigned flags)
{
  int c, i;
  uint16_t nd = 0;
  uint64_t acc = 0;
  uint64_t fac = 1;
  int cpw, ciw = 0;
  char d[256], rd[256];

  memset(d, 0, 256);

  /* open the file */
  bprintf(info, "DC-blobbing %s", blob_source);
  FILE *stream = fopen(blob_source, "r");
  if (stream == NULL) {
    berror(warning, "Unable to open %s", blob_source);
    return -1;
  }

  /* first pass: compute the dictionary */
  while ((c = fgetc(stream)) != EOF)
    d[c & 0xFF] = 1;
  for (i = 0; i < 256; ++i)
    if (d[i] && (i != 0x20 || ~flags & DC_IGNORE_SPACE))
      rd[(int)(d[i] = nd++)] = i;
    else
      d[i] = -1;

  /* can't deal with a small dictionary */
  if (nd == 0) {
    rd[0] = 'x';
    rd[1] = 'y';
    d[(int)'x'] = 0;
    d[(int)'y'] = 1;
    nd = 2;
  } else if (nd == 1) {
    rd[1] = (~rd[0]) & 0xFF;
    d[(int)rd[1]] = 1;
    nd = 2;
  }

  /* write the header */
  blob[0] = nd;
  for (i = 0; i < nd; i += 2)
    blob[i / 2 + 1] = (rd[i + 1] << 8) | rd[i];
  blob_size = i / 2 + 1;

  /* second pass: encode file */
  cpw = 64 * M_LN2 / log(nd);
  rewind(stream);
  while ((c = fgetc(stream)) != EOF) {
    if ((flags & DC_IGNORE_SPACE) && c == 0x20)
      continue;

    if (ciw == cpw) {
      /* ship out the accumulator */
      for (i = 0; i < 4; ++i) {
        blob[blob_size++] = acc & 0xFFFF;
        acc >>= 16;

        if (blob_size == MCE_BLOB_MAX) {
          bprintf(warning, "File too big; truncating");
          fclose(stream);
          return 0;
        }
      }
      acc = ciw = 0;
      fac = 1;
    }

    /* add */
    acc += d[c] * fac;
    fac *= nd;
    ciw++;
  }
  /* last word(s) */
  if (ciw)
    for (i = 0; i < 4; ++i) {
      blob[blob_size++] = acc & 0xFFFF;
      acc >>= 16;

      if (blob_size == MCE_BLOB_MAX) {
        bprintf(warning, "File too big; truncating");
        fclose(stream);
        return 0;
      }
    }

  fclose(stream);
  return 0;
}

/* compress IV curve */
int iv_compress()
{
  uint32_t ivframe[NUM_ROW * NUM_COL];
  uint32_t ivc[MAX_BSET][FB_SIZE];
  int ii, jj, count, bset_offset, n_chan;
  uint32_t iv_min[MAX_BSET], iv_max[MAX_BSET];
  double iv_scale;
  uint16_t bias_start, bias_step, n_bias;
  char bias_file[1024];
  char bias[1024];
  uint16_t ivb[MCE_BLOB_MAX];
  int iv_bset;
  int chan[MAX_BSET];

  bprintf(info, "reading IV curves from %s", blob_source);
  
  /* open the bias file */
  sprintf(bias_file, "%s.bias", blob_source);
  FILE *bstream = fopen(bias_file, "r");
  if (bstream == NULL) {
    bprintf(warning, "Unable to open %s", bias_file);
    return -1;
  }
  
  /* extract bias values */
  count = 0;
  while (!feof(bstream)) {
    if (fgets(bias,1024,bstream) == NULL) break;
    if (bias[0] == '<') continue;
    ivb[count++] = atoi(bias);
  }
  fclose(bstream);
  n_bias = count;
  bias_start = ivb[0];
  bias_step = (bias_start-ivb[n_bias-1])/(n_bias-1);
  bprintf(info, "Found %d biases in range %d to %d", 
      n_bias, bias_start, ivb[n_bias-1]);
  
  /* check TES selection */
  iv_bset = blob_data[0];
  bset_offset = blob_data[1];
  count = blob_data[2];
  if (count > MAX_BSET)
    count = MAX_BSET;

  /* use current PCM bset on out of range bset */
  if (iv_bset <= 0 || iv_bset > 255)
    iv_bset = bset_num & 0xFF;

  /* load the bset -- on error this will make an empty bset */
  iv_bset = change_bset(iv_bset);

  /* extract a channel list from the bset */
  n_chan = 0;
  for (ii = bset_offset; ii < curr_bset.n; ++ii) {
    if (TES_MCE(curr_bset.v[ii]) == nmce) {
      chan[n_chan++] = TES_OFFSET(curr_bset.v[ii]);
      if (n_chan == count)
        break;
    }
  }

  /* nothing to send */
  if (n_chan == 0)
    return -1;

  if (n_chan * (n_bias + 6) > MCE_BLOB_MAX)
    n_chan = (uint16_t) MCE_BLOB_MAX / (n_bias + 6);
  bprintf(info, "Blobbing %d channels", n_chan);
  
  /* open the data file */
  FILE *stream = fopen(blob_source, "rb");
  if (stream == NULL) {
    bprintf(warning, "Unable to open %s", blob_source);
    return -1;
  }
  
  count = 0;
  for (ii = 0; ii < n_chan; ii++) {
    iv_min[ii] = -1;
    iv_max[ii] = 0;
  }
  
  /* extract data */
  while (!feof(stream)) {
    /* skip header */
    if (fseek(stream, sizeof(uint32_t)*MCE_HEADER_SIZE, SEEK_CUR)) {
      bprintf(warning, "Truncated frame %d header", count);
      break;
    }
    
    /* read data */
    if (fread(ivframe, sizeof(uint32_t), NUM_ROW * NUM_COL, stream) 
        != NUM_ROW * NUM_COL){
      bprintf(warning, "Truncated frame %d data", count);
      break;
    }
    
    /* skip footer */
    if (fseek(stream, sizeof(uint32_t), SEEK_CUR)) {
      bprintf(warning, "Truncated frame %d footer", count);
      break;
    }
    
    /* extract desired channels */
    for (ii = 0; ii < n_chan; ii++) {
      ivc[ii][count] = (ivframe[chan[ii]] >> 7);
      if (ivc[ii][count] < iv_min[ii]) iv_min[ii] = ivc[ii][count];
      if (ivc[ii][count] > iv_max[ii]) iv_max[ii] = ivc[ii][count];
    }
    count++;
  }
  fclose(stream);
  
  if (count > n_bias) {
    bprintf(warning, "Bias file mismatch");
    return -1;
  } else if (count < n_bias) n_bias = count;
  
  /* fill blob */
  blob_size = 0;
  // blob[blob_size++] = n_tes; // number of IV curves expected
  for (ii = 0; ii < n_chan; ii++) {
    // rescale to 16-bits
    iv_scale = 65536. / (double)(iv_max[ii] - iv_min[ii]);
  
    /* iv header */
    blob[blob_size++] = chan[ii]; // TES ID
    bprintf(info,"Blobbing IV curve for TES %d",blob[blob_size-1]);
    blob[blob_size++] = bias_start;          // starting bias
    blob[blob_size++] = n_bias;              // number of bias steps
    blob[blob_size++] = bias_step;           // bias step size
    blob[blob_size++] = (uint16_t) (iv_min[ii] * iv_scale); // TES current offset
    blob[blob_size++] = (uint16_t) iv_scale; // TES current rescaling factor
    
    /* iv curve */
    for (jj = 0; jj < n_bias; jj++) {
      blob[blob_size++] = (uint16_t) ((ivc[ii][jj]-iv_min[ii]) * iv_scale);
    }
  }
  bprintf(info,"IV blob size %d",blob_size);
  
  return 0;
}

/* prepare data blobs */
void *blobber(void *dummy)
{
  int r;
  nameThread("Blob");

  /* wait for a blob-making request */
  for (;;) {
    if (new_blob_type != BLOB_NONE) {
      blob_type = new_blob_type;
      new_blob_type = BLOB_NONE;
      bprintf(info, "New blob: %i\n", blob_type);

      switch (blob_type) {
        case BLOB_EXPCFG:
          strcpy(blob_source, "/data#/mce/current_data/experiment.cfg");
          blob_source[5] = '0' + data_drive[0];
          r = dict_compress(DC_IGNORE_SPACE);
          break;
        case BLOB_IV:
          sprintf(blob_source, "/data/mce/ivcurves/iv_%04i", memory.last_iv);
          r = iv_compress();
          break;
        case BLOB_TUNECFG:
          tuning_filename("sq1_ramp", "experiment.cfg", blob_data[0],
              blob_source);
          r = dict_compress(DC_IGNORE_SPACE);
          break;
        case BLOB_TUNESQ_SA:
          tuning_filename("sa_ramp", "*.sqtune", blob_data[0], blob_source);
          r = dict_compress(0);
          break;
        case BLOB_TUNESQ_SQ2:
          tuning_filename("sq2_servo", "*.sqtune", blob_data[0], blob_source);
          r = dict_compress(0);
          break;
        case BLOB_TUNESQ_SQ1S:
          tuning_filename("sq1_servo", "*.sqtune", blob_data[0], blob_source);
          r = dict_compress(0);
          break;
        case BLOB_TUNESQ_SQ1R:
          tuning_filename("sq1_ramp", "*.sqtune", blob_data[0], blob_source);
          r = dict_compress(0);
          break;
        default:
          bprintf(warning, "Ignoring unknown method: %i", blob_type);
          r = 1;
          break;
      }

      /* trigger a send */
      if (!r && new_blob_type == BLOB_NONE)
        send_blob = 1;
    } else
      sleep(1);
  }
}
