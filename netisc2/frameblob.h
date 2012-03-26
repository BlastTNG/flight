/* 
        frameblob.h

        class which takes a bitmapped frame and locates blobs (where
        blobs are stored as a linked list)

        Depends on bloblist class, some definitions in mapglobals.
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bloblist.h"
#include "mapglobals.h"

#ifndef FRAMEBLOB_H
#define FRAMEBLOB_H

#define D 19       // Size of the box used by findCentre
#define R 2        // Radius of the parabolic smoothing kernel for findCentre

class frameblob {
 public:
  // --- Constructors / Destructor ---
  frameblob( void );     // Constructors
  frameblob( MAPTYPE *map_in, unsigned int xpix_in, unsigned int ypix_in,
             unsigned int bits_in, double platescale_in );
  void commonconstructor(MAPTYPE *map_in, unsigned int xpix_in, 
                         unsigned int ypix_in,
                         unsigned int bits_in, double platescale_in );
  ~frameblob();                 // Destructor
  
  void set_map( MAPTYPE *in_map );
  MAPTYPE *get_map(void) { return map; };
  
  // --- Add blob, delete blob, return blob list ---
  void addblob( int flux, double x, double y );  // Add a blob to the list
  void deleteblob( bloblist *killblob );   // Remove blob from the list
  void clearblobs();                       // Clear the whole blob list  
  bloblist *getblobs();            // return pointer to 1st blob in list
  void sortblobs();                // sort blobs by descending order of flux
  void set_maxblobs(unsigned int in_maxblobs);        // set maximum number of blobs
  
  // --- Bad pixel map ---
  int load_badpix(char *fname); // Load bad pixel map from file
  void fix_badpix(MAPTYPE val); // set bad pixels to val
  
  // --- Set search parameters ---
  void set_satval( MAPTYPE in_satval );      // set numeric value of sat pixels
  void set_grid( unsigned int in_grid );     // coarse grid pixsize
  void set_threshold( double in_threshold ); // n sigma source threshold
  void set_disttol( int in_disttol );        // adjacent source dist tolerance
  
  // --- Get search parameters ---
  MAPTYPE get_satval(void) { return satval; };
  unsigned int get_grid(void) { return grid; };
  double get_threshold(void) { return threshold; };
  int get_disttol(void);
  
  // --- Source find / map statistics ---
  void calc_mapstat();   // calc map mean + count # saturated pixels each col.
  double get_mapmean();  // return the map mean - calls mapstat
  double get_sigma();    // return estimates standard deviation of the image
  void calc_searchgrid();// find cells in search grid with sources
  void fix_multiple();   // check for multiple sources w/i disttol and fix
  
  void set_gain( double in_gain );
  void set_readout_offset( double in_readout_offset );
  void set_readout_noise( double in_readout_noise );
  
  double get_gain(void) { return gain; };
  double get_readout_offset(void) { return readout_offset; };
  double get_readout_noise(void) { return readout_noise; };
  int get_numblobs(void) { return numblobs; };
  
 private:
  // better centroid
  void findCentre(double *x, double *y, double *z, double M[D][D]); 
  double convolve(double x, double y, double *noise, double M[D][D]);  
  
  MAPTYPE *map;             // Pointer to the map buffer
  unsigned int bits;        // number of bits per pixel
  unsigned int xpix;        // # pixels wide
  unsigned int ypix;        // # pixels high
  unsigned int npix;        // # pixels in total map
  
  double *searchgrid;       // coarse grid cells - store S/N brightest pixel
  double *meangrid;         // array of mean values for each grid cell
  unsigned int xgrid;       // dimensions of the search grid array
  unsigned int ygrid;       //    "
  unsigned int ngrid;       //    "
  
  double platescale;        // number of degrees/pixel in the image
  double gain;              // gain applied to N_photons going to measured val
  double readout_offset;    // constant offset in the frame 
  double readout_noise;     // constant RMS noise added to frame due to readout
  
  bloblist *firstblob;      // blob linked list pointers (sources)
  bloblist *lastblob;
  bloblist *thisblob;       // multi-purpose blob pointer for list traversal
  unsigned int numblobs;
  unsigned int maxblobs;    // won't exceed this number of blobs if non-zero
  
  unsigned int *badpixels;  // array of indices of bad pixels in CCD frames
  unsigned int numbadpix;   // # of bad pixels
  
  double mapmean;           // the mean value of the total map
  double sigma;             // estimated standard deviation of the map
  MAPTYPE satval;           // numerical value of a saturated pixel
  int *satcol;              // # saturated pixels in each column of the map
  double convweight;        // weight of the convolution filter

  unsigned int grid;        // # pixels/side in coarse grid
  double threshold;         // N sigma threshold for detection    
  int disttol;              // distance tolerance adjacent sources in pixels
};

#endif FRAMEBLOB_H
