/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2005-2006 University of Toronto
 *
 * This file is part of mcp.
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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "blast.h"
#include "fir.h"

void initFir(struct FirStruct *fs, int ns) {
  int i;
  double x, sw = 0.0;

  for (i=0; i<NSTAGE; i++) {
    fs->sum[i] = 0;
    fs->i_w[i] = i*ns/NSTAGE;
  }

  fs->out = 0;
  fs->ns = ns;

  fs->w = (double *)balloc(fatal, ns * sizeof(double));

  for (i=0; i<ns; i++) {
    x = i-ns/2;
    x /= ((double)ns/4.5);

    fs->w[i] = exp(-x*x);
    sw += fs->w[i];
  }
  for (i=0; i<ns; i++) {
    fs->w[i] /= sw;
  }
}

double filter(double x, struct FirStruct *fs) {
  int i, i_stage;

  for (i_stage = 0; i_stage<NSTAGE; i_stage++) {
    i = fs->i_w[i_stage];

    fs->sum[i_stage] += fs->w[i]*x;

    i++;
    if (i>=fs->ns) {
      fs->out = fs->sum[i_stage];
      i = 0;
      fs->sum[i_stage] = 0;
    }
    fs->i_w[i_stage] = i;
  }
  return (fs->out);
}

#if 0
int main() {
  double x;
  struct FirStruct fs;
  int i;

  initFir(&fs, 100.0*30.0*60.0);

  for (i=0; i<1000000; i++) {
    x = sin(2.0*M_PI*(double)i/4000) + sin(2.0*M_PI*(double)i/8000); //random()/12345.0;
    printf("%g %g\n", x, filter(x, &fs));
  }
  return(0);
}
#endif
