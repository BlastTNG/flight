#ifndef SSLUT_H
#define SSLUT_H

#define SS_OK 0
#define SS_OUTOFRANGE 1
#define SS_NOTCONVERGING 2
#define SS_LUTNOTFOUND 3

#define NPIX 4096             // Number of pixels
#define D 8.28                // Slit to ccd distance (mm)
#define PIXD 142.857          // Pixel density (Pix/mm)

#define MAXITER 10            // Maximum number of allowed iteration for the LUT convergence 

typedef struct {
  double CCDx;   
}  CCD_t;

typedef struct {
  CCD_t *LUT;
  double az_min, az_max;
  double el_min, el_max;
  int *hits;
  int nx;
  int ny;
  int size;
  int nfill;
  int x0, y0;
} SSLut_t;


/* Intialize Sun Sensor LUT */
int SSLut_GetLut(SSLut_t *SSLut, const char *filename);

/* Browse the LUT:
   given ccd_x, ccd_y in pixels
   returns current azi and ele in radiants
   iter: number of iterations  */
int SSLut_find(double ccd_x, double *azi, double ele, SSLut_t *SSLut, int *iter);

#endif
