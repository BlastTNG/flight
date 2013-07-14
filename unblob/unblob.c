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

#define GD_64BIT_API
#include <getdata.h>

static void usage(void)
{
  printf("Usage:\n  unblob [OPTION]... DIRFILE [OFFSET]\n\n"
    "where:\n\n"
    "  DIRFILE      the path to the dirfile to read\n"
    "  OFFSET       an optional frame offset\n"
    " -m MCE        unblob a blob from MCC number MCE (1-6)\n"
    " -n BLOB_NUM   only unblob blob number BLOB_NUM (otherwise, unblob the "
    "first\n"
    "                 blob found matching the criteria (after frame OFFSET)\n"
    " -t BLOB_TYPE  unblob a blob of numeric type BLOB_TYPE.\n");
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
  uint16_t blob_len = 0;

  uint16_t blob_left = 0;

  size_t n, i;
  uint16_t *data = malloc(30000 * sizeof(uint16_t));
  
  const char *dirfile;
  uint64_t offset = 0;

  /* parse command line */
  while ((n = getopt(argc, argv, "m:n:t:") != -1)) {
    switch (n) {
      case 'm':
        mce_num_req = atoi(optarg);
        break;
      case 'n':
        blob_num_req = atoi(optarg);
        break;
      case 't':
        blob_type_req = atoi(optarg);
        break;
      case '?':
      default:
        usage();
    }
  }

  /* look for non-option stuff */
  argc -= optind;
  argv += optind;

  if (argc < 1)
    usage();

  dirfile = argv[0];

  if (argc > 1)
    offset = strtoull(argv[1], NULL, 0);

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
        blob_left = blob_len = data[i] + 3;
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
      payload = malloc(sizeof(uint16_t) * blob_len);
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
          blob_sample + blob_len + BLOB_LEADIN_LEN + 2, 0,
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
  payload_crc = CalculateCRC(CRC_SEED, payload, blob_len);

  printf("Found blob #%i at frame %.02f from X%i, type %i, len = %i.\n",
      blob_num, blob_sample / 20., blob_mce, blob_type, blob_len);
  printf("  CRC check: lead-in: %s; lead-out %s\n",
      (blob_crc1 == payload_crc) ? "ok" : "BAD",
      (blob_crc2 == payload_crc) ? "ok" : "BAD");

  return 0;
}
