/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2003-2005 University of Toronto
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "blast.h"
#include "lut.h"
#include "mcp.h"

#define MAXITER 10
#define TOLERANCE (10.0/3600.*M_PI/180.)

/***************************************************************************/
/*    GetLine: read non-comment line from file                             */
/*        The line is placed   in *line.                                   */
/*        Returns 1 if succesful, 0 if unsuccesful                         */
/***************************************************************************/
int GetLine(FILE *fp, char *line)
{
  char buffer[MAX_LINE_LENGTH];
  char *ret_val;
  int first_char;
  
  do {
    ret_val = fgets(buffer, MAX_LINE_LENGTH, fp);
    if (ret_val != NULL) {
      first_char = 0;
      while ((buffer[first_char] == ' ') || (buffer[first_char] == '\t'))
	first_char++;
      strncpy(line, &buffer[first_char], MAX_LINE_LENGTH - first_char);
    }
  } while ((ret_val != NULL) && ((line[0] == '#') || (strlen(line) < 2)));

  if (ret_val != NULL)
    return 1; /* a line was read */
  else
    return 0; /* there were no valid lines */
}

void LutInit(struct LutType *L)
{
  int i;

  FILE *fp;
  char line[1024];

  fp = fopen(L->filename, "r");
  if (fp == NULL) {
    blast_err("LUT: error reading LUT file %s: no calibration\n",
        L->filename);
    L->n = 1;
    return;
  }

  /* first read the file to see how big it is */
  i = 0;
  while (GetLine(fp, line))
    i++;

  if (i < 2) {
    blast_err("LUT: error reading LUT file %s: no calibration\n",
        L->filename);
    L->n = 1;
    return;
  }

  //blast_info("LUT: Reading lut `%s' with %i lines\n", L->filename, i);
  L->n = i;
  L->x = (double *)balloc(fatal, i * sizeof(double));
  L->y = (double *)balloc(fatal, i * sizeof(double));

  /* now read in the data */
  rewind(fp);
  for (i = 0; i < L->n; i++) {
    GetLine(fp, line);
    sscanf(line, "%lg %lg",&(L->x[i]), &(L->y[i]));
  }
  L->last_n = L->n / 2;

  /* Is the LUT index ascending or descending? */
  if(L->x[1] <= L->x[(L->n)-1]) {
    L->dir = 1;
  } else {
    L->dir = -1;
  }
}

double LutCal(struct LutType *L, double x)
{
  int i, n;
  double y;

  n = L->n;

  if (n == 1) {
    return(x); // no LUT, not cal
  }
  /* find index */
  i = L->last_n;

  if (L->dir >= 0) { 
    while ((i < n - 2) && (x > L->x[i]))	//i can't be over n-2 since i+1 is used
      i++;
    while ((i > 0) && (x < L->x[i]))
      i--;
  } else {
    while ((i < n - 2) && (x < L->x[i]))
      i++;
    while ((i > 0) && (x > L->x[i]))
      i--;
  }

  L->last_n = i;

  y = L->y[i] + (L->y[i + 1] - L->y[i]) / (L->x[i + 1] - L->x[i]) *
    (x - L->x[i]);

  return(y);
}

#if 0	//this appears unused
	//if revived, check for array overflows on interpolation (like above)
double MagLutCal(struct LutType *L, double mag_x, double mag_y, double x)
{
  int i, n, iter;
  double mod;
  double mag_c, mag_s;
  double tsin, tcos;
  double dx;

  if (L->n == 1)
    return(x); // no LUT, not cal

  n = L->n >> 1;

  mag_x -= L->x[0];
  mag_y -= L->y[0];
  mod = sqrt(mag_x*mag_x + mag_y*mag_y);
  mag_x /= mod;
  mag_y /= mod;

  iter = 0;
  x *= M_PI/180.0;
  mag_s = mag_c = 0.0;
  while(1) {
    for (i = 1; i <= n; i++) {
      tsin = sin(i*x); tcos = cos(i*x);
      mag_s += L->y[2*i-1]*tsin + L->y[2*i]*tcos;
      mag_c += L->x[2*i-1]*tsin + L->x[2*i]*tcos;
    }
    mod = sqrt(mag_s*mag_s + mag_c*mag_c);
    mag_s /= mod; mag_c /= mod;

    dx = mag_s*mag_x - mag_c*mag_y;
    x += dx;
    iter++;
    if(fabs(dx) < TOLERANCE) return (x*180.0/M_PI);

    if(iter == MAXITER) {
      blast_err("LUT: Error: MagLutCal doesn't converge.\n");
      return (x*180.0/M_PI);
    }
  }

  return 0.0;
}
#endif

