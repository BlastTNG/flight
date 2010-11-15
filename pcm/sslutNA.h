/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2004 University of Toronto
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

#ifndef SSLUT_H
#define SSLUT_H

#define SS_OK 0
#define SS_OUTOFRANGE 1
#define SS_NOTCONVERGING 2
#define SS_LUTNOTFOUND 3

#define NPIX 4096             // Number of pixels
#define D 8.28                // Slit to ccd distance (mm)
#define PIXD 142.857          // Pixel density (Pix/mm)

#define MAXITER 10            // Maximum number of allowed iteration
                              // for the LUT convergence 

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
int SSLut_find(double, double*, double, SSLut_t*, int*);

#endif
