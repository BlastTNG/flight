#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "fir.h"

void initFir(struct FirStruct *fs, int ns) {
  int i;
  double x, sw = 0.0;

  for (i=0; i<NSTAGE; i++) {
    fs->sum[NSTAGE] = 0;
    fs->i_w[i] = i*ns/NSTAGE;
  }
    
  fs->out = 0;
  fs->ns = ns;
  
  fs->w = (double *)malloc(ns*sizeof(double));

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


/* int main() { */
/*   double x; */
/*   struct FirStruct fs; */
/*   int i; */

/*   initFir(&fs, 10000); */
  
/*   for (i=0; i<1000000; i++) { */
/*     x = sin(2.0*M_PI*(double)i/4000) + sin(2.0*M_PI*(double)i/8000); //random()/12345.0; */
/*     printf("%g %g\n", x, filter(x, &fs)); */
/*   } */
/*   return(0); */
/* } */
