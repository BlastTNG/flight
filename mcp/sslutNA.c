#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "sslutNA.h"
void SSLut_Normalize(SSLut_t *SSLut)
{
  int k;

  for(k = 0; k < SSLut->size; k++) {
    SSLut->LUT[k].CCDx = (SSLut->LUT[k].CCDx - NPIX/2)/(D*PIXD);
  }
}

int SSLut_GetLut(SSLut_t *SSLut, const char *filename)
{
  FILE *fd;

  if ( (fd = fopen(filename, "r")) == NULL ) {
    fprintf(stderr, "Unable to load Sun Sensor LUT: file %s\n", filename);
    return SS_LUTNOTFOUND;
  }

  fread(&SSLut->nx, sizeof(SSLut->nx), 1, fd);
  fread(&SSLut->ny, sizeof(SSLut->ny), 1, fd);
  fread(&SSLut->az_min, sizeof(SSLut->az_min), 1, fd);
  fread(&SSLut->az_max, sizeof(SSLut->az_max), 1, fd);
  fread(&SSLut->el_min, sizeof(SSLut->el_min), 1, fd);
  fread(&SSLut->el_max, sizeof(SSLut->el_max), 1, fd);
  SSLut->size = SSLut->nx*SSLut->ny;

  SSLut->LUT = malloc(SSLut->size * sizeof(*SSLut->LUT));
  memset(SSLut->LUT, 0, SSLut->size * sizeof(*SSLut->LUT));
  fread(SSLut->LUT, sizeof(*SSLut->LUT), SSLut->size, fd);
  fclose(fd);

  SSLut_Normalize(SSLut);
  
  return SS_OK;
}    

int SSLut_find(double ccd_x, double *azi, double ele,
	       SSLut_t *SSLut, int *iter) {
  double dx;
  int k0;
  double aa;

  
  if((ccd_x < 10.0) || (ccd_x > (NPIX - 10.))) 
    return SS_OUTOFRANGE;
  ccd_x = (ccd_x - NPIX/2)/(D*PIXD);
  
  SSLut->y0 = rint((ele - SSLut->el_min)/(SSLut->el_max-SSLut->el_min) * SSLut->ny);
  if(SSLut->y0 < 0 || SSLut->y0 > SSLut->ny)
    return SS_OUTOFRANGE;
  
  *iter = 0;
  do {
    
    k0 = SSLut->x0 + SSLut->nx*SSLut->y0;
    dx = (SSLut->LUT[k0].CCDx - ccd_x)/(1.0 + SSLut->LUT[k0].CCDx*ccd_x); 
    dx = dx * SSLut->nx / (SSLut->az_max - SSLut->az_min); 
  
    SSLut->x0 = rint(SSLut->x0 + dx);
    
    while (SSLut->x0 < 0) SSLut->x0 += SSLut->nx;
    while (SSLut->x0 > SSLut->nx) SSLut->x0 -= SSLut->nx;
    
    (*iter)++;       
    
  } while ( (fabs(dx) > 1.0) && (*iter < MAXITER));
  
  if(*iter >= MAXITER) return SS_NOTCONVERGING;
  
  // Do a linear interpolation: this is good to <1 arc minute 
  
  k0 = SSLut->x0 + SSLut->nx*SSLut->y0;
  aa = (SSLut->LUT[k0].CCDx - ccd_x)/(SSLut->LUT[k0].CCDx - SSLut->LUT[k0+1].CCDx);

  *azi = ((double)SSLut->x0 + aa)*(SSLut->az_max - SSLut->az_min)/SSLut->nx + SSLut->az_min;
  
  return SS_OK;
}
