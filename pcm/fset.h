/* pcm: the Spider master control program
 *
 * fset.h: field set prototypes
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
#ifndef FSET_H
#define FSET_H

#include "channels.h" /* for FIELD_LEN */
#include "tes.h" /* for NUM_MCE */
#include "tx.h" /* for NUM_MCE_FIELDS */
#include <stdint.h>

/* maximum number of fields in the sets */
#define MAX_FSET 200

/* if this is more than 255, the type of .im in the struct bset must also be
 * changed to accomodate */
#define MAX_BSET NUM_MCE_FIELDS

/* set buffers */
struct bset {
  int n;
  int16_t v[MAX_BSET];

  int nm[NUM_MCE]; /* per-MCE counts */
  int8_t im[NUM_MCE][MAX_BSET]; /* per-MCE reverse lookups */
  int empties; /* indicating MCEs for which do data will be returned */
};

struct fset {
  int n;
  char *v[MAX_FSET][FIELD_LEN];
};

int read_bset(int i, struct bset *set);
int read_fset(int i, struct fset *set);

int get_bset(struct bset *local_set);
void set_bset(const struct bset *local_set, int num);
int get_fset(struct fset *local_set);
void set_fset(const struct fset *local_set, int num);

#endif
