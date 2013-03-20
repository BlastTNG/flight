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
#include <stdint.h>

/* maximum number of fields in a field set */
#define MAX_FSET 100

/* describes an item in a field set */
struct fset_item {
  int16_t bolo; /* < 0 indicates non-bolo */
  char name[FIELD_LEN]; /* for non-bolos */
};

/* a field set */
struct fset {
  int n;
  struct fset_item *f;
};

struct fset *read_fset(int i, struct fset *set);

#endif
