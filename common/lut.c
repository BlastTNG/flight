#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "lut.h"
#include "mcp.h"

#define MAXITER 10
#define TOLERANCE (10.0/3600.*M_PI/180.)

int GetLine(FILE *fp, char *line); // defined in sched.c

void LutInit(struct LutType *L) {
  int i;

  FILE *fp;
  char line[255];

  fp = fopen(L->filename, "r");
  if (fp == NULL) {
    mprintf(MCP_ERROR, "error reading LUT file %s: no calibration\n",
        L->filename);
    L->n = 1;
    return;
  }

  /* first read the file to see how big it is */
  i = 0;
  while (GetLine(fp, line)) {
    i++;
  }
  if (i < 2) {
    mprintf(MCP_ERROR, "error reading LUT file %s: no calibration\n",
        L->filename);
    L->n = 1;
    return;
  }

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
  while ((i >= 0) && (x < L->x[i]))
    i--;
  L->last_n = i;

  y = L->y[i] + (L->y[i + 1] - L->y[i]) / (L->x[i + 1] - L->x[i]) *
    (x - L->x[i]);

  return(y);
}

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
      mprintf(MCP_ERROR, "Error MagLutCal: don't converge.\n");
      return (x*180.0/M_PI);
    }
  }

  return 0.0;
}
