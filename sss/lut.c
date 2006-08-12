/* sss: The SSS control program
 *
 * This software is copyright (C) 2003-2004 University of Toronto
 * 
 * This file is originally from mcp, the BLAST master control program,
 * which is a GPL'ed app.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "lut.h"

#define MAXITER 10
#define TOLERANCE (10.0/3600.*M_PI/180.)
#define MAX_LINE_LENGTH 1000

int GetLine(FILE *fp, char *line)
{
  char buffer[MAX_LINE_LENGTH];
  char *ret_val;
  int first_char;

  do {
    ret_val = fgets(buffer, MAX_LINE_LENGTH, fp);
    first_char = 0;
    while ((buffer[first_char] == ' ') || (buffer[first_char] == '\t'))
      first_char++;
    strncpy(line, &buffer[first_char], MAX_LINE_LENGTH);
  } while (((line[0] == '#') || (strlen(line) < 2)) && (ret_val != NULL));

  if (ret_val != NULL)
    return 1; /* a line was read */
  else
    return 0; /* there were no valid lines */
}

void LutInit(struct LutType *L) {
  int i;

  FILE *fp;
  char line[1024];

  fp = fopen(L->filename, "r");
  if (fp == NULL) {
    fprintf(stderr, "LUT: error reading LUT file %s: no calibration\n",
        L->filename);
    L->n = 1;
    return;
  }

  /* first read the file to see how big it is */
  i = 0;
  while (GetLine(fp, line))
    i++; 

  if (i < 2) {
    fprintf(stderr, "LUT: error reading LUT file %s: no calibration\n",
        L->filename);
    L->n = 1;
    return;
  }

  fprintf(stderr, "LUT: Reading lut `%s' with %i lines\n", L->filename, i);
  L->n = i;
  L->x = (double *)malloc(i * sizeof(double));
  L->y = (double *)malloc(i * sizeof(double));

  /* now read in the data */
  rewind(fp);
  for (i = 0; i < L->n; i++) {
    GetLine(fp, line);
    sscanf(line, "%lg %lg",&(L->x[i]), &(L->y[i]));
  }
  L->last_n = L->n / 2;
}

double LutCal(struct LutType *L, double x) {
  int i, n;
  double y;

  n = L->n;

  if (n == 1)
    return(x); // no LUT, not cal

  /* find index */
  i = L->last_n;
  while ((i < n - 1) && (x > L->x[i]))
    i++;
  while ((i > 0) && (x < L->x[i]))
    i--;

  L->last_n = i;

  y = L->y[i] + (L->y[i + 1] - L->y[i]) / (L->x[i + 1] - L->x[i]) *
    (x - L->x[i]);

  return(y);
}

