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

#include "mce_counts.h"

struct ms_phys {
  char c[4];
  char p[22];
  int nr, nw, id, cd;
};
extern const struct ms_phys mstat_phys[N_MCE_PHYS];

extern const char *const mstat_sys[N_MCE_SYS];

struct ms_map {
  int s, e;
  char c[4];
  char p[22];
  int o;
};

struct ms_virt {
  char c[8];
  char p[22];
  int n;
  struct ms_map m[2];
};
extern const struct ms_virt mstat_virt[N_MCE_VIRT];
