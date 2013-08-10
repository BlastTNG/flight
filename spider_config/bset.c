/* pcm: the Spider master control program
 *
 * bset.c: handle routines for field set manipulation
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * There are 255 sets available of (001.bset -> 255.bset).  bset files may
 * contain comment lines (with a # in column one, and also blank lines.  These
 * are ignored.
 *
 * The first (non-blank, non-comment) line must comprise an integer indicating
 * the number of entries in the set.  Excess elements are ignored.
 */

#include "channels.h"
#include "bset_dir.h"
#include "bset.h"
#include "tes.h"
#include "blast.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>

/* serial number for bsets */
static uint8_t bset_serial = 0xF9;

/* currently loaded set in PCM */
struct bset curr_bset;

/* set the global sets along with their numbers; also sets CommandData for
 * consistency */
static int set_bset(const struct bset *local_set, int num)
{
  memcpy(&curr_bset, local_set, sizeof(curr_bset));
  curr_bset.num = num;
  return num;
}

/* parse bset file number 'i' and store it in 'set'.  Returns -1 on error */
static int read_bset(int i, struct bset *set)
{
  struct bset new_set = { .n = -1 };
  int lineno = 1, c = 0;
  FILE *stream;
  char line[1024];
  char name[sizeof(SET_DIR) + sizeof(".bset") + 5];

  sprintf(name, SET_DIR "/%03i.bset", i);

  /* open */
  if ((stream = fopen(name, "r")) == NULL) {
    /* a missing file isn't interesting */
    if (errno != ENOENT)
      berror(err, "Unable to open %s as BSET%03i", name, i);
    return -1;
  }

  /* parse lines */
  while (fgets(line, 1024, stream)) {
    size_t len = strlen(line);
    /* check for line length */
    if (line[len - 1] != '\n') {
      bprintf(err, "Error reading BSET%03i: line %i too long.", i,
          lineno);
      goto LOAD_BSET_ERROR;
    }

    /* skip comments and blank lines */
    if (line[0] == '\n' || line[0] == '#') {
      lineno++;
      continue;
    }

    if (new_set.n == -1) {
      /* first line is the number */
      char *endptr;
      new_set.n = strtol(line, &endptr, 10);
      /* check for trailing garbage and/or crazy numbers */
      if (*endptr != '\n' || new_set.n <= 0) {
        bprintf(err, "Error reading BSET%03i: bad count on line %i", i,
            lineno);
        goto LOAD_BSET_ERROR;
      } else if (new_set.n > MAX_BSET) {
        bprintf(warning, "BSET%03i too long; dropping %i %s", i,
            new_set.n - MAX_BSET,
            (new_set.n - MAX_BSET == 1) ? "entry" : "entries");
        new_set.n = MAX_BSET;
      }
    } else if (c == new_set.n) {
      /* done -- ignore the rest of the file and return success */
      break;
    } else { /* bolo */
      int mce;

      /* parse "x#c##r##\n" */
      if (line[0] != 'x' || line[2] != 'r' || line[5] != 'c' || line[8] != '\n')
      {
        bprintf(err, "Bad syntax line %i of BSET%03i", lineno, i);
        goto LOAD_BSET_ERROR;
      }

      /* lame number parsing.
       * yes ",m" produces the same number as "21" ... DON'T DO THAT */
      mce = line[1] - '1';
      new_set.v[c] = TESNumber(mce,
          (line[3] - '0') * 10 + line[4] - '0',
          (line[6] - '0') * 10 + line[7] - '0');

      if (new_set.v[c] < 0) {
        bprintf(err, "Bad bolometer number on line %i of BSET%03i",
            lineno, i);
        goto LOAD_BSET_ERROR;
      }

      /* update per-mce stuff */
      new_set.im[mce][new_set.nm[mce]++] = c++;
    }
    lineno++;
  }

  if (c != new_set.n) {
    bprintf(err, "Unexpected EOF reading BSET%03i", i);
    goto LOAD_BSET_ERROR;
  }

  /* calculate empties */
  for (i = 0; i < NUM_MCE; ++i)
    if (new_set.nm[i] == 0)
      new_set.empties |= (1 << i);

  /* done */
  fclose(stream);
  memcpy(set, &new_set, sizeof(new_set));
  return new_set.n;

LOAD_BSET_ERROR:
  fclose(stream);
  return -1;
}

/* (re-)load the bset number 'i' into the local buffer -- no change on error;
 * 'init'=1 occurs at start up, when there's no fallback initialised.
 */
int change_bset(int j)
{
  static int init = 1;
  struct bset new_bset;
  int i = j & 0xFF;

  /* range checking */
  if (i < 0 || i > 255) {
    bprintf(warning, "Ignoring out-of-range BSET index %i\n", i);
    return curr_bset.num;
  }

  /* special empty sets -- always succeeds */
  if (i == 0) {
    memset(&new_bset, 0, sizeof(new_bset));
    return set_bset(&new_bset, new_bset_num(0));
  }

  /* try to load the bset */
  if (read_bset(i, &new_bset) == -1) {
    /* no bset loaded -- load the empty default */
    if (init) {
      init = 0;
      return change_bset(new_bset_num(0));
    }

    bprintf(warning, "Unable to read BSET%03i; still using BSET%03i", i,
        (curr_bset.num & 0xFF));
    return curr_bset.num;
  }

  /* update the current bset */
  init = 0;
  return set_bset(&new_bset, j);
}

int new_bset_num(int i)
{
  /* avoid the forbidden serial number */
  if (bset_serial == 0xFF)
    bset_serial++;

  return (i | (bset_serial++ << 8));
}
