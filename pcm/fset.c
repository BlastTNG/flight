/* pcm: the Spider master control program
 *
 * fset.c: handle routines for field set manipulation
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

#include "command_struct.h"
#include "fset.h"
#include "tes.h"
#include "blast.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>

#define SET_DIR "/data/etc/spider"

/* currently loaded set in PCM */
static struct bset curr_bset;

static pthread_mutex_t set_mex = PTHREAD_MUTEX_INITIALIZER;

/* get/set the global sets along with their numbers */
int get_bset(struct bset *local_set)
{
  int num;
  pthread_mutex_lock(&set_mex);
  num = CommandData.bset_num;
  memcpy(local_set, &curr_bset, sizeof(curr_bset));
  pthread_mutex_unlock(&set_mex);
  return num;
}

void set_bset(const struct bset *local_set, int num)
{
  pthread_mutex_lock(&set_mex);
  CommandData.bset_num = num;
  memcpy(&curr_bset, local_set, sizeof(curr_bset));
  pthread_mutex_unlock(&set_mex);
}

/* parse bset file number 'i' and store it in 'set'.  Returns 'set' or NULL
 * on error
 */
int read_bset(int i, struct bset *set)
{
  struct bset new_set = { .n = -1 };
  int lineno = 0, c = 0;
  FILE *stream;
  char line[1024];
  char name[sizeof(SET_DIR) + sizeof(".bset") + 5];

  sprintf(name, SET_DIR "/%03i.bset", i);

  /* open */
  if ((stream = fopen(name, "r")) == NULL) {
    /* a missing file isn't interesting */
    if (errno != ENOENT)
      berror(err, "Set: unable to open %s as BSET%03i", name, i);
    return -1;
  }

  /* parse lines */
  while (fgets(line, 1024, stream)) {
    size_t len = strlen(line);
    /* check for line length */
    if (line[len - 1] != '\n') {
      bprintf(err, "Set: Error reading BSET%03i: line %i too long.", i,
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
      if (*endptr != '\n' || new_set.n <= 0 || new_set.n >= MAX_BSET) {
        bprintf(err, "Set: Error reading BSET%03i: bad count on line %i", i,
            lineno);
        goto LOAD_BSET_ERROR;
      } else if (new_set.n > MAX_BSET) {
        bprintf(warning, "Set: BSET%03i too long; dropping %i %s", i,
            new_set.n - MAX_BSET,
            (new_set.n - MAX_BSET == 1) ? "entry" : "entries");
        new_set.n = MAX_BSET;
      }
    } else if (c == new_set.n) {
      /* done -- ignore the rest of the file and return success */
      break;
    } else { /* bolo */
      int mce;

      /* parse "m#c##r##\n" */
      if (line[0] != 'm' || line[2] != 'c' || line[5] != 'r' || line[8] != '\n')
      {
        bprintf(err, "Set: Bad bolometer number on line %i of BSET%03i",
            lineno, i);
        goto LOAD_BSET_ERROR;
      }
      /* lame number parsing.
       * yes ",m" produces the same number as "21" ... DON'T DO THAT */

      mce = line[1] - '0';
      new_set.v[c] = TESNumber(mce,
          (line[3] - '0') * 10 + line[4] - '0',
          (line[6] - '0') * 10 + line[7] - '0');

      if (new_set.v[c] < 0) {
        bprintf(err, "Set: Bad bolometer number on line %i of BSET%03i",
            lineno, i);
        goto LOAD_BSET_ERROR;
      }

      /* update per-mce stuff */
      new_set.im[mce][new_set.nm[mce]++] = c++;
    }
    lineno++;
  }

  if (c != new_set.n) {
    bprintf(err, "Set: Unexpected EOF reading BSET%03i", i);
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
