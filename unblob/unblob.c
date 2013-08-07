/* unblob: reconstitute mce_blob data
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * unblob is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * unblob is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with unblob; if not, write to the Free Software Foundation.
 */
#include "mce_blob.h"
#include "crc.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define GD_64BIT_API
#include <getdata.h>

#define DEFAULT_DIRFILE "/data/etc/defile.lnk"

static void dict_decompress(uint16_t *payload, uint16_t len)
{
  int i;
  int cpw, pos;
  int nd = payload[0];
  int d[256];

  for (i = 0; i < nd; i += 2) {
    d[i] = payload[i / 2 + 1] & 0xFF;
    d[i + 1] = payload[i / 2 + 1] >> 8;
  }

  pos = (nd / 2) + 1 + (nd % 2);
  cpw = 64 * M_LN2 / log(nd);

  for (; pos < len; pos += 4) {
    uint64_t v = *((uint64_t*)(payload + pos));
    for (i = 0; i < cpw; ++i) {
      char c = d[v % nd];
      write(1, &c, 1);
      v /= nd;
    }
  }
}

static void usage(void)
{
  printf("Usage:\n  unblob [OPTION]...\n\n"
    "where:\n\n"
    " -d DIRFILE    read from DIRFILE instead of " DEFAULT_DIRFILE "\n"
    " -o OFFSET     start reading at frame OFFSET\n"
#if 0
    " -m MCE        unblob a blob from MCC number MCE (1-6)\n"
    " -n BLOB_NUM   only unblob blob number BLOB_NUM (otherwise, unblob the "
    "first\n"
    "                 blob found matching the criteria (after frame OFFSET)\n"
    " -t BLOB_TYPE  unblob a blob of numeric type BLOB_TYPE.\n"
#endif
    );
  exit(0);
}

int main(int argc, char **argv)
{
  int blob_num_req = -1;
  int mce_num_req = -1;
  int blob_type_req = -1;
  int match = 0;
  const uint16_t leadin[] = BLOB_LEADIN;
  const uint16_t leadout[] = BLOB_LEADOUT;
  uint16_t *payload = NULL;

  uint64_t blob_sample = 0;
  uint16_t blob_num = 0, blob_mce = 0, blob_type = 0, blob_crc1 = 0;
  uint16_t blob_crc2 = 0, payload_crc = 0;
  uint16_t payload_len = 0;

  uint16_t blob_left = 0;

  size_t n, i;
  uint16_t *data = malloc(30000 * sizeof(uint16_t));
  
  const char *dirfile = NULL;
  uint64_t offset = 0;

  /* parse command line */
  while ((n = getopt(argc, argv, "d:m:n:o:t:")) != -1) {
    switch (n) {
      case 'd':
        dirfile = strdup(optarg);
        break;
      case 'm':
        mce_num_req = atoi(optarg);
        break;
      case 'n':
        blob_num_req = atoi(optarg);
        break;
      case 'o':
        offset = atoi(optarg);
        break;
      case 't':
        blob_type_req = atoi(optarg);
        break;
      case '?':
      default:
        printf("unrecognised: '%c'\n", (char)n);
        usage();
    }
  }

  if (dirfile == NULL)
    dirfile = DEFAULT_DIRFILE;

  /* get data */
  DIRFILE *D = gd_open(dirfile, GD_RDONLY | GD_VERBOSE);
  if (!D || gd_error(D))
    return 1;

  /* find a blob */
  for (;;) {
    n = gd_getdata64(D, "mce_blob", offset, 0, 1500, 0, GD_UINT16, data);
    if (gd_error(D))
      return 1;

    if (n == 0) {
      fprintf(stderr, "Out of data after frame %llu\n", offset);
      return 1;
    }

    /* look for the lead in */
    for (i = 0; i < n; ++i) {
      if (match == BLOB_LEADIN_LEN) {
        blob_num = data[i];
        match++;
      } else if (match == BLOB_LEADIN_LEN + 1) {
        blob_crc1 = data[i];
        match++;
      } else if (match == BLOB_LEADIN_LEN + 2) {
        blob_mce = data[i] + 1;
        match++;
      } else if (match == BLOB_LEADIN_LEN + 3) {
        blob_type = data[i];
        match++;
      } else if (match == BLOB_LEADIN_LEN + 4) {
        blob_left = payload_len = data[i] + 3;
        match++;
        break;
      } else {
        if (data[i]) {
          if (data[i] == leadin[match])
            match++;
          else
            match = 0;
        } else if (match < 8)
          match++;
      }
    }

    if (match == BLOB_LEADIN_LEN + 5) {
      i++;
      blob_sample = offset * 20 + i - match;
      size_t n_to_copy = n - i + 3;
      if (n_to_copy > blob_left)
        n_to_copy = blob_left;
      payload = malloc(sizeof(uint16_t) * payload_len);
      memcpy(payload, data + i - 3, sizeof(uint16_t) * n_to_copy);
      blob_left -= n_to_copy;
      /* get the rest */
      if (blob_left > 0) {
        n = gd_getdata(D, "mce_blob", GD_HERE, 0, 0, blob_left, GD_UINT16,
            payload + n_to_copy);
        if (gd_error(D))
          exit(1);
        if (n < blob_left) {
          fprintf(stderr, "Out of data reading blob.\n");
          exit(1);
        }
      }
      
      /* get the leadout */
      n = gd_getdata(D, "mce_blob", 0,
          blob_sample + payload_len + BLOB_LEADIN_LEN + 2, 0,
          1 + BLOB_LEADOUT_LEN, GD_UINT16, data);
      if (n < 1 + BLOB_LEADOUT_LEN) {
        fprintf(stderr, "Out of data reading leadout.\n");
        exit(1);
      }
      blob_crc2 = data[0];
      for (i = 0; i < BLOB_LEADOUT_LEN; ++i) {
        if (data[i + 1] != leadout[i]) {
          fprintf(stderr, "Bad block lead-out.\n");
          break;
        }
      }
      break;
    }

    offset += n / 20;
  }

  /* calcualte CRC */
  payload_crc = CalculateCRC(CRC_SEED, payload, payload_len);

  fprintf(stderr, "Found blob #%i at frame %.02f from X%i, type %i, "
      "len = %i.\n", blob_num, blob_sample / 20., blob_mce, blob_type,
      payload_len);
  fprintf(stderr, "  CRC check: lead-in: %s; lead-out %s\n",
      (blob_crc1 == payload_crc) ? "ok" : "BAD",
      (blob_crc2 == payload_crc) ? "ok" : "BAD");

  /* strip the MPC header */
  payload += 3;
  payload_len -= 3;

  /* do something with the blob */
  switch (blob_type) {
    case BLOB_NONE:
      fprintf(stderr, "Bad blob type zero.\n");
      return 1;
    case BLOB_EXPCFG:
    case BLOB_TUNECFG:
    case BLOB_TUNESQ:
      dict_decompress(payload, payload_len);
      break;
  }

  return 0;
}
