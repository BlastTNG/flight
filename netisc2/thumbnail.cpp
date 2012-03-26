#include "thumbnail.h"

// Save thumbnails of sources from a bitmapped image
//
// buf      = pointer to image
// size     = size of an element in the image
// xdim     = x dimension of the image
// ydim     = y dim "
// thumbsize  = size of the thumbnail on a side in pixels
// nsource  = # of sources in the image
// xsource  = array of x coordinates for thumbnails
// ysource  = array of y "
// filename = name of the file in which to save the thumbnails
//
// Return: 1 for success, 0 for failure

int thumbnail( unsigned short *buf, int xdim, int ydim, int thumbsize,
                int nsource, int *xsource, int *ysource, char *filename ) {
  FILE *thumbfile;
  unsigned short *thisthumb;
  int i, j, k;
  int count;
  int *left, *bottom;
  
  // Allocate arrays for coordinates of the top left corner of each thumbnail
  left = new int[nsource];
  bottom = new int[nsource];    
  thisthumb = new unsigned short[thumbsize*thumbsize];
  
  // Calculate the coordinates for the bottom-left of each thumbnail
  for( k=0; k<nsource; k++ ) {
    left[k] = xsource[k] - thumbsize/2;
    bottom[k] = ysource[k] - thumbsize/2;
    
    if( left[k] < 0 ) left[k] = 0;
    if( (left[k]+thumbsize-1) >= xdim ) left[k] = xdim - thumbsize;
    
    if( bottom[k] < 0 ) bottom[k] = 0;
    if( (bottom[k]+thumbsize-1) >= ydim ) bottom[k] = ydim - thumbsize;
  }
  
  // Open the file and write out the header info:
  // xdim, ydim, thumbsize, nsource, left (array), bottom (array)
  if( (thumbfile = fopen( filename, "wb" )) == NULL )
    return 0;
  
  fwrite( &xdim, sizeof(xdim), 1, thumbfile );
  fwrite( &ydim, sizeof(ydim), 1, thumbfile );
  fwrite( &thumbsize, sizeof(thumbsize), 1, thumbfile );
  fwrite( &nsource, sizeof(nsource), 1, thumbfile );
  
  fwrite( left, sizeof(int), nsource, thumbfile );
  fwrite( bottom, sizeof(int), nsource, thumbfile );
  
  // Write out the individual thumbnails
  for( k=0; k<nsource; k++ ) {
    // reset we are in the thumbnail buffer
    count = 0;
    
    for( j=bottom[k]; j<bottom[k]+thumbsize; j++ )
      for( i=left[k]; i<left[k]+thumbsize; i++ ) {
	thisthumb[count] = buf[j*xdim + i];
	count ++;
      }
        
    fwrite( thisthumb, sizeof(unsigned short), thumbsize*thumbsize, 
	    thumbfile );
  }
  
  // clean up
  
  fclose(thumbfile);
  
  delete[] left;
  delete[] bottom;
  delete[] thisthumb;
  
  return 1;
}
