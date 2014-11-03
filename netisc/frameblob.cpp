/* 
   frameblob.cpp

   class for locating blobs in a bitmap
        
   Depends on bloblist class.
*/

#include "frameblob.h"

// ---------- Class constructor -----------------------------------------

frameblob::frameblob( void ) {
}

frameblob::frameblob( MAPTYPE *map_in, unsigned int xpix_in, unsigned 
                      int ypix_in, unsigned int bits_in, 
                      double platescale_in ) {
  commonconstructor( map_in, xpix_in, ypix_in, bits_in, platescale_in );
}

void frameblob::commonconstructor( MAPTYPE *map_in, 
                                   unsigned int xpix_in, 
                                   unsigned int ypix_in, 
                                   unsigned int bits_in, 
                                   double platescale_in ) {
  // set up map parameters
  map = map_in;
  xpix = xpix_in;
  ypix = ypix_in;
  bits = bits_in;
  platescale = platescale_in;
  npix = xpix*ypix;
  
  // Blob list pointers initialized to 0
  numblobs = 0;
  maxblobs = 0;
  
  firstblob = NULL;
  lastblob = NULL;
  thisblob = NULL;
  
  // Set the gridsearch array to NULL to check at the end if allocated
  searchgrid = NULL;
  meangrid = NULL;
  
  // Map statistics defaults
  mapmean=0;
  sigma=1;
  satval = 16383;             // numerical value of a saturated pixel
  satcol = new int[xpix];     // array contains # sat pixels each column
  
  gain=1;
  readout_offset=0;
  readout_noise=0;

  // Defaults for search parameters
  grid = 32;         // pixels/side in coarse grid search
  threshold = 5;     // N sigma threshold criterion
  disttol = 30*30;   // distance^2 tolerance for adjacent sources
  
  badpixels = NULL;

}

// set the pointer to the map
void frameblob::set_map( MAPTYPE *in_map ) {
  map = in_map;
}


// ---------- Class Destructor ------------------------------------------

frameblob::~frameblob() {
  clearblobs();
  
  delete[] searchgrid;
  delete[] meangrid;
  delete[] badpixels;
  delete[] satcol;
}

// ---------- Add blob to list ------------------------------------------

void frameblob::addblob( int flux, double x, double y ) {
  // Only add a new blob if we haven't exceeded the maximum number, or if
  // we can have an unlimited number (maxblob=0)
  
  //printf("nblobs: %i maxblobs: %i\n",numblobs,maxblobs);
  
  if( (maxblobs == 0) || (numblobs < maxblobs) ) {
    thisblob = new bloblist(flux, x, y);
    
    if( lastblob == NULL ) {  // make new starting blob if empty list
      firstblob = thisblob;
      lastblob = thisblob;              
      numblobs = 1;
    } else {                  // otherwise add to the end
      lastblob->setnextblob( thisblob );
      thisblob->setprevblob( lastblob );
      numblobs++;
    }
    
    lastblob = thisblob;      // new end of list
  }
}

// ---------- Delete blob from list -------------------------------------

void frameblob::deleteblob( bloblist *killblob ) {
  bloblist *prev, *next;

  prev = killblob->getprevblob();
  next = killblob->getnextblob();

  // Set the head/tail pointers for surrounding blobs to skip this one

  if( prev == NULL ) {      // if 1st blob in list

    if( next == NULL ) {    // if was only blob in list
      firstblob = NULL;
      lastblob = NULL;
    } else {                // otherwise next is new start
      firstblob = next;
      next->setprevblob(NULL);
    }
  } else {
    if( next == NULL ) {    // was last element in multi-blob list
      prev->setnextblob(NULL);
      lastblob = prev;
    } else {                // somewhere in the middle of a list
      prev->setnextblob(next);
      next->setprevblob(prev);
    }
  }
  
  // Delete this blob

  delete killblob;
  
  // Decrement the blob counter
  
  numblobs --;
}

// ---------- Clear whole blob list -------------------------------------

void frameblob::clearblobs() {
  bloblist *nextblob;
  thisblob = firstblob;  // start at the beginning of the list
  while( thisblob != NULL ) {
    nextblob = thisblob->getnextblob(); // get next blob
    delete thisblob;                    // delete the current one
    thisblob = nextblob;                // move along
  }

  firstblob=NULL;
  lastblob=NULL;
  thisblob=NULL;
  numblobs = 0;
}

// ---------- Return start of blob list ---------------------------------

bloblist *frameblob::getblobs() { // pointer to first blob
  return firstblob;
}

// ---------- Sort the blob list in descending flux order ---------------

void frameblob::sortblobs() {
  bloblist *nextblob, *startblob, *endblob;
  int flag=1;   // if set, list is still out of order
  
  if( numblobs > 0 ) {
    while( flag == 1 ) {
      flag=0;
      thisblob = firstblob;
      nextblob = thisblob->getnextblob();
      
      while( nextblob != NULL ) {  // Traverse list, swap if out of order
        if( nextblob->getflux() > thisblob->getflux() ) {   
          flag=1;    // list still out of order
          
          startblob = thisblob->getprevblob();
          endblob = nextblob->getnextblob();
          
          // Change the forward pointers
          if( startblob != NULL ) startblob->setnextblob(nextblob);
          else firstblob = nextblob;   // since start of list changed
          nextblob->setnextblob(thisblob);
          thisblob->setnextblob(endblob);
          
          // Change the backward pointers
          if( endblob != NULL ) endblob->setprevblob(thisblob);
          else lastblob = thisblob;
          thisblob->setprevblob(nextblob);
          nextblob->setprevblob(startblob);
          
          nextblob = endblob;
        } else thisblob = nextblob;
        
        nextblob = thisblob->getnextblob();
      }
    }
  }
}

// set a maximum number of blobs
void frameblob::set_maxblobs(unsigned int in_maxblobs) {
  maxblobs = in_maxblobs;
}

// ---------- Deal with bad (extra noisey) pixels -----------------------

// Load bad pixel map from file. returns -1 on error, 1 for success

int frameblob::load_badpix(char *fname) {
  FILE *badfile;
  bloblist *firstbad, *thisbad, *nextbad, *lastbad;  // bad pixel list
  char thisline[81];
  int count;     // Number of elements read from the current line
  int i, x, y;   // coordinates of the bad pixel in this line
  
  if( (badfile = fopen( fname, "r" )) == NULL )
    return -1;
  
  // Read through file line-by-line reading in pixel coordinates.
  // Use the blob linked-list to dynamically add coordinates as we
  // read them in. At the end, put the array indices calculated from the
  // pixel coordinates into an array for quick access when we process
  // frames.
  
  numbadpix = 0;   
  firstbad = NULL; 
  
  while( fgets(thisline,80,badfile) != NULL ) {
    count = sscanf(thisline,"%i %i",&x, &y);
    //printf("badpix: %i %i\n",x, y);
    if( count == 2 ) { // If we read in a coordinate pair
      numbadpix ++;
      thisbad = new bloblist(0, (double)x, (double)(ypix - y - 1));
      
      if( firstbad == NULL ) { // If first bad pixel in the list
        firstbad = thisbad;
      } else {                 // adding on new bad pixel to list
        lastbad->setnextblob(thisbad);
        thisbad->setprevblob(lastbad);
      }
      
      lastbad = thisbad;
    }
  }
  
  fclose(badfile);
  
  // Now traverse (and delete) linked list, calculating pixel coordinates
  
  badpixels = new unsigned int[numbadpix];
  thisbad = firstbad;  // start at the beginning of the list
  
  i = 0;
  
  //printf("badpixels: %i\n",numbadpix);
  while( thisbad != NULL ) {
    badpixels[i] = (unsigned int) thisbad->gety()*xpix + 
      (unsigned int) thisbad->getx();
    
    i++;
    nextbad = thisbad->getnextblob();  // get next blob
    delete thisbad;                    // delete the current one
    thisbad = nextbad;                 // move along
  }
  
  printf("Opened bad pixel file: %s with %i bad pixels.\n",
         fname,numbadpix);

  return 1;
}


// Set bad pixels to a value

void frameblob::fix_badpix(MAPTYPE val) {
  int i;
  
  if( badpixels != NULL ) 
    for( i=0; i<(int)numbadpix; i++ ) { 
      map[badpixels[i]] = val;
    }
}

// ---------- Search Parameters -----------------------------------------

void frameblob::set_satval( MAPTYPE in_satval ) {
  satval = in_satval;
}

void frameblob::set_grid( unsigned int in_grid ) {
  grid = in_grid;
}

void frameblob::set_threshold( double in_threshold ) {
  threshold = in_threshold;
  
  if( threshold < 0.2 ) threshold = 0.2;  // place a hard limit
}

void frameblob::set_disttol( int in_disttol ) {
  disttol = in_disttol*in_disttol;
}

int frameblob::get_disttol(void) { 
  return (int) sqrt((double) disttol); 
}


// ---------- Calculate map statistics: mean/# saturated pixels/column --

void frameblob::calc_mapstat() {
  unsigned int i, j, count,index;
  
  // initialize variables
  
  mapmean = 0;
  count = 0;
  memset(satcol,0,xpix*sizeof(int));
  
  for( j=0; j<ypix; j++ ) { // j=row
    index = j*xpix;         // inner loop columns to speed up addressing
    for( i=0; i<xpix; i++ ) {
      //printf("%i ",map[index]);
      
      if( map[index] > satval ) satcol[i]++;   // if bad pixel
      else {                // otherwise contributes to mean
        if( map[index] != satval ) { // pixel isn't bad
          mapmean += map[index];
          count ++;
        }
      }
      index++;
    }
  }
  
  if( count > 0 ) mapmean = mapmean / (double) count; // prevent  / zero
  if( mapmean < 1 ) mapmean = 1;                      // prevent 0 means
}

double frameblob::get_mapmean() {
  if( mapmean == 0 ) calc_mapstat();
  return mapmean;
}

double frameblob::get_sigma() {
  return sigma;
}


// ---------- Search grid for coarse cells with sources -----------------

void frameblob::calc_searchgrid() {
  // First calculate dimensions of the searchgrid array
  
  unsigned int xfull, yfull, xextra, yextra, xdim, ydim;
  
  xfull  = xpix / grid;       // # of full-sized cells
  yfull  = ypix / grid; 
  xextra = xpix % grid;       // # extra pixels in small edge cells
  yextra = ypix % grid;
  
  xgrid = (xfull + (xextra>0));        // total # grid cells 
  ygrid = (yfull + (yextra>0));
  ngrid = (xgrid+xextra) * ygrid;
  
  // unallocate searchgrid and meangrid before we start in case we have 
  // run this function several times 
  
  delete searchgrid;
  delete meangrid;
  
  searchgrid = new double[ngrid];   // S/N of brightest pixel/grid cell
  meangrid = new double[ngrid];     // array of grid cell means 
  
  // Allocate an array to contain all the GRID*GRID elements within this
  // grid cell of our input map
  
  MAPTYPE *cell;
  cell = new MAPTYPE[grid*grid];
  
  MAPTYPE pix;           // current pixel values
  MAPTYPE max;           // the maximum value pixel in the cell
  int total;             // running total elements in cell (big number)
  
  double meancell ;      // cell mean
  double level;          // max adjusted by centre value
  double sn;             // signal to noise level of brightest pixel
  double x;              // x and y pixel positions for brightest pixels
  double y;              //   for the large input map  
  
  unsigned int i, j, k, l, mapindex, cellindex;
  int startx, endx, starty, endy;
  double thisflux;
  double M[D][D];  // small map centred over blob candidate
  double xcen, ycen, nap;
  int clip_startx, clip_endx, clip_starty, clip_endy;
  int count=0;
  
  clearblobs();    // Make sure we don't have old blobs sitting around
  
  for( i=0; i<xgrid; i++ )            
    for( j=0; j<ygrid; j++ ) {  
      // Sum all the elements within this cell of the input map.
      // Also make a list of the elements in the cell
      
      cellindex = 0;
      total = 0;
      max = 0;
      
      // Pixel dimensions of grid cell. grix*grid unless edge
      if( i==xfull ) xdim = xextra;
      else xdim = grid;
      if( j==yfull ) ydim = yextra;
      else ydim = grid;
      
      
      for( k=0; k<xdim; k++ )     
        for( l=0; l<ydim; l++ ) {
          mapindex = (j*grid+l)*xpix + (i*grid+k);
          pix = map[mapindex];
          
          cell[cellindex] = pix;
          total += pix;
          
          // Check for greatest pixel brightness
          if( pix > max ) {
            x = (double) (i*grid + k);  // store pixel coordinates
            y = (double) (j*grid + l);  // of brightest pixel    
            max = pix; 
          }                                     
          cellindex ++; // end of loop = # pixels in cell
        }
      
      // Get the mean for this cell
      meancell = (double) total / (cellindex);
      
      // Level is the brightness of a pixel - the mean level for the cell
      level = (double) max - meancell; 
      
      // Calculate the sample variance about the central value
      if( mapmean == 0 ) calc_mapstat();   // Calculate the map mean
      
      if( mapmean < readout_offset ) sigma = readout_noise;
      else sigma = (double)sqrt(gain*(mapmean-readout_offset) + 
                                readout_noise*readout_noise); // Poisson
      if( sigma < 1 ) sigma = 1;   // prevent 0 sigmas
      
      sn = level/sigma;
      
      // Store information about this grid cell
      searchgrid[j*xgrid+i] = sn;     // s/n brightest pixel in cell
      meangrid[j*xgrid+i] = meancell; // mean of this cell
      
      // If we got a pixel > threshold sigma, possibly a source
      if( sn >= threshold ) {
        //printf("%i ",count);
        //count ++;
        
        // --------------------------------------------------------------
        // Decide if this is a single-point spike or a real source
        
        startx = (int) x - 1;  // Aperture boundaries
        starty = (int) y - 1;
        endx = (int) x + 1;
        endy = (int) y + 1;
        
        if( startx < 0 ) startx = 0;          // clipping map boundaries
        if( endx >= (int) xpix ) endx = xpix-1;
        if( starty < 0 ) starty = 0;
        if( endy >= (int) ypix ) endy = ypix-1;
        
        thisflux = 0.;
        
        nap = (double) (endx-startx+1)*(endy-starty+1); // pixels in ap.

        // add up flux centred over bright pixel
        for( k=startx; k<=endx; k++ )
          for( l=starty; l<=endy; l++ )
            thisflux += (double) map[l*xpix + k];
        
        // remove flux centre pixel and check remaining flux 
        // exceeds the theoretical flux from pure noise 
        // (check extendedness)
        thisflux -= max;

        // remove the baseline for the remaining pixels
        thisflux -= ((nap-1)*meancell);

        // Extended case
        if( (thisflux/(sqrt(nap-1)*sigma)) >= threshold ) {       
          // ------------------------------------------------------------
          // Re-calculate the baseline over perimeter of a larger box
          
          startx = (int) x - D/2;  // box boundary
          starty = (int) y - D/2;
          endx = startx + D-1;
          endy = starty + D-1;
          
          if( startx < 0 ) startx = 0;    // clipping for boundaries
          if( endx >= (int) xpix ) endx = xpix-1;
          if( starty < 0 ) starty = 0;
          if( endy >= (int) ypix ) endy = ypix-1;
          
          meancell = 0;
          for( k=startx; k<=endx; k++ )
            meancell += (double) map[starty*xpix + k] + 
              (double) map[endy*xpix + k];
          
          for( l=starty+1; l<=endy-1; l++ )
            meancell += (double) map[l*xpix + startx] + 
              (double) map[l*xpix + endx];
          
          meancell /= ( 2*(endx-startx) + 2*(endy-starty) );
          
          // ------------------------------------------------------------
          // Centroid the blob
          
          startx = (int) x - D/2;  
          starty = (int) y - D/2;
          endx = startx + D-1;
          endy = starty + D-1;
          
          if( startx < 0 ) clip_startx = 0;  // clipping for boundaries
          else clip_startx = startx;
         
          if( endx >= (int) xpix ) clip_endx = xpix-1;
          else clip_endx = endx;
          
          if( starty < 0 ) clip_starty = 0;
          else clip_starty = starty;
          
          if( endy >= (int) ypix ) clip_endy = ypix-1;
          else clip_endy = endy;
          
          // boundary case
          if( (startx != clip_startx) || (starty != clip_starty) ||
              (endx != clip_endx) || (endy != clip_endy) ) {
          
            // fill the sub-map with the mean value
            for( k=0; k<D; k++ )
              for( l=0; l<D; l++ )
                M[k][l] = (double) meancell;
            
            // paste in the useful part of the map
            for( k=clip_startx-startx; k<clip_endx-startx; k++ )
              for( l=clip_starty-starty; l<clip_endy-starty; l++ )
                M[k][l] = (double) map[(l+clip_starty)*xpix + 
                                       k+clip_startx];
          }
          
          // normal case
          else {
            for( k=0; k<D; k++ )
              for( l=0; l<D; l++ )
                M[k][l] = (double) map[(l+starty)*xpix + k+startx];
          }
          
          // Find the centroid in this small map
          findCentre( &xcen, &ycen, &thisflux, M );
          
          // Correct for the baseline and final check realness
          thisflux = thisflux - meancell;
          
          //printf("Flux: %lf Sigma: %lf\n",
          //thisflux,sqrt(convweight)*sigma);

          // Add to the list if significant
          if( thisflux/(sqrt(convweight)*sigma) >= threshold ) {
            addblob( (int)thisflux, x+xcen+0.5, y+ycen+0.5 ); 
            thisblob->setmean(meancell);
            thisblob->setsnr( (double)thisflux / 
                              (sqrt(convweight)*sigma) );
          }
        }
        
        // --------------------------------------------------------------
      }
    }
  
  // clean up
  delete cell;
}

// ---------- Barth's improved convolution centroid finder --------------

double frameblob::convolve(double x, double y, double *noise, 
                           double M[D][D]) {
  int i, j;
  int xi, yi;
  double dx2, dy2, r2;
  double T=0, Tw = 0, w;
  
  *noise = 0; // Added by EC - calculate expected noise in the aperture
  
  for (i=-R-1; i<=R+1; i++) {
    xi = (int)floor(x + i+0.5);
    dx2 = x-(double)xi;
    dx2*=dx2;

    if (xi<0) xi = 0;
    if (xi>D-1) xi = D-1;

    for (j=-R-1; j<=R+1; j++) {
      yi = (int)floor(y + j+0.5);
      dy2 = y-(double)yi;
      dy2*=dy2;
      
      if (yi<0) yi = 0;
      if (yi>D-1) yi = D-1;
      
      r2 = dx2 + dy2;
      
      if (r2>R*R) {
        w = 0;
      } else {
        w = (-r2 + R*R)/R*R;
      }
      
      Tw+=w;
      T+=w*M[xi][yi];
      
      *noise += w*w;
    }
  }
  
  *noise /= Tw*Tw; // multiple by sigma^2 in a pixel to get 
                   // sigma^2 in aperture
  return (T/Tw);
}

void frameblob::findCentre(double *x, double *y, double *z, 
                           double M[D][D]) {
  double z1, z2, z3;
  double C;
  int j;
  double dp, sx, sy;
  double w;
  
  *x = *y = 0.0;
  
  C = floor( (double) D/2); // centre of map - our starting guess
  
  sx = sy = 1.0;
  
  for (j=0; j<10; j++) {
    // last find best x along best y row
    z1 = convolve(C-sx+*x, C+*y, &w, M);
    z2 = convolve(C+*x, C+*y, &convweight, M);
    z3 = convolve(C+sx+*x, C+*y, &w, M);
    
    if (z2 > (z1 + z3)/2) {
      dp = (z1-z2)/(z1+z3-z2-z2) - 0.5;
      
      /* only believe if we bounded the max */
      if (dp>1) {
        dp = sx;
      } else if (dp<-1) {
        dp = -sx;
      } else {
        dp *= sx;
        sx*=0.7;
      }
    } else {
      if (z1>z3) dp = -sx;
      else dp = sx;
    }
    *x += dp;
    
    //fprintf(stderr, "x: %g %g %g %g\n", *x, z1, z2, z3);
    
    // now find best y along best x row
    z1 = convolve(C+*x, C-sy+*y, &w, M);
    z2 = convolve(C+*x, C+*y, &convweight, M);
    z3 = convolve(C+*x, C+sy+*y, &w, M);
    
    if (z2 > (z1 + z3)/2) {
      dp = (z1-z2)/(z1+z3-z2-z2) - 0.5;
      
      /* only believe if we bounded the max */
      if (dp>1) {
        dp = sy;
      } else if (dp<-1) {
        dp = -sy;
      } else {
        dp *= sy;
        sy*=0.7;
      }
    } else {
      if (z1>z3) dp = -sy;
      else dp = sy;
    }
    
    *y += dp;
    
    //fprintf(stderr, "x: %g y: %g %g | %g %g %g\n",
    // *x, *y, dp, z1, z2, z3);
    if (sy<0.2 && sx < 0.2) break;
  }
  
  *z = z2;
}

// ---------- Set the gain and other noise statistics parameters --------

void frameblob::set_gain( double in_gain ) {
  gain = in_gain;
}

void frameblob::set_readout_offset( double in_readout_offset ) {
  readout_offset = in_readout_offset;
}

void frameblob::set_readout_noise( double in_readout_noise ) {
  readout_noise = in_readout_noise;
}

// ---------- Check for multiple sources --------------------------------

void frameblob::fix_multiple() {
  int ref_x, ref_y, ref_dist, ref_flux;
  //int maxflux;
  
  bloblist *refblob, *compareblob;
  bloblist *nextcomp;
  
  // Traverse blob list
  refblob = firstblob;
  while( refblob != NULL ) {
    ref_x = (int) refblob->getx();
    ref_y = (int) refblob->gety();
    ref_flux = refblob->getflux();
    
    // Compare each blob to the reference
    compareblob = firstblob;
    while( compareblob != NULL ) {
      nextcomp = compareblob->getnextblob();
      if( compareblob != refblob ) {
        ref_dist = (ref_x-(int)compareblob->getx()) *
          (ref_x-(int)compareblob->getx()) + 
          (ref_y-(int) compareblob->gety()) *
          (ref_y-(int)compareblob->gety());
        
        // if blob is close to the reference and fainter, delete it
        if( (ref_dist <= disttol) && (compareblob->getflux() <= 
                                      ref_flux) )
          deleteblob(compareblob);
      }
      compareblob = nextcomp;
    }
    refblob = refblob->getnextblob();
  }
}
