#include <stdio.h>
#include <stdlib.h>

#include "lut.h"
#include "mcp.h"

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
