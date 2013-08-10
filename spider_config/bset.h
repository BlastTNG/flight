/* pcm: the Spider master control program
 *
 * bset.h: field set prototypes
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
#include <stdint.h>

/* if this is more than 255, the type of .im in the struct bset must also be
 * changed to accomodate */
#define MAX_BSET NUM_MCE_FIELDS

/* set buffers */
struct bset {
  int num; /* bset num */
  int n; /* bset count */
  int16_t v[MAX_BSET];

  int nm[NUM_MCE]; /* per-MCE counts */
  int8_t im[NUM_MCE][MAX_BSET]; /* per-MCE reverse lookups */
  int empties; /* indicating MCEs for which no data will be returned */
};
extern struct bset curr_bset;

int new_bset_num(int);
int change_bset(int j);

#endif
