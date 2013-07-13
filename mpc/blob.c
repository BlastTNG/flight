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
#include "mpc.h"
#include "mputs.h"

int new_blob_type = BLOB_NONE;
int blob_type = BLOB_NONE;
int blob_size;
char blob_source[1024];

/* dictionary-compresses the given stream into the blob buffer */
#define DC_IGNORE_SPACE 1
int dict_compress(unsigned flags)
{
  int c, i;
  uint16_t nd = 0;
  uint32_t acc = 0;
  uint32_t env = 1;
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
    if (d[i] && (!DC_IGNORE_SPACE || i != 0x20))
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
  bprintf(info, "Dictionary size: %i", nd);
  blob[0] = nd;
  for (i = 0; i < nd; i += 2)
    blob[i / 2 + 1] = (rd[i + 1] << 8) | rd[i];
  blob_size = i / 2 + 1;

  /* second pass: encode file */
  rewind(stream);
  while ((c = fgetc(stream)) != EOF) {
    if (DC_IGNORE_SPACE && c == 0x20)
      continue;

    if (env > 0xFFFF) {
      /* ship out the low bits */
      blob[blob_size++] = acc & 0xFFFF;
      acc >>= 16;

      if (blob_size == MCE_BLOB_MAX) {
        bprintf(warning, "File too big; truncating");
        fclose(stream);
        return 0;
      }

      /* recompute the envelope.  If it's not a power of two, we burn part of
       * a bit here */
      if (env & (env - 1)) {
        env = (env >> 16) + 1;
      } else
        env >>= 16;
    }

    /* add */
    acc = d[c] * env;
    env *= nd;
  }
  /* last word(s) */
  if (env > 0xFFFF) {
    blob[blob_size++] = (acc & 0xFFFF);
    acc >>= 16;
  }

  if (blob_size == MCE_BLOB_MAX)
    bprintf(warning, "File too big; truncating");
  else
    blob[blob_size++] = (acc & 0xFFFF);

  fclose(stream);
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
