/* sss
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * 
 * This file is originally part of mcp, of BLAST.
 * 
 * sss is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * sss is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

struct LutType {
  char filename[1000];
  int n;
  double *y;
  double *x;
  int last_n;
};

double LutCal(struct LutType *L, double x);
void LutInit(struct LutType *L);
