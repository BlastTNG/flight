/*
	bloblist.h

	linked list of blobs
*/

#include <stdio.h>

#ifndef BLOBLIST_H
#define BLOBLIST_H

class bloblist {
 public:
  bloblist(int flux_in, double x_in, double y_in);       // Node creator
  ~bloblist();                                           // Node destructor
  
  // Set blob properties
  void setflux( int flux_in );
  void setx( double x_in );
  void sety( double y_in );
  void settype( int type_in );
  void setsnr( double snr_in );
  void setmean( double mean_in );
  
  // Retrieve blob properties 
  int getflux();
  double getx();
  double gety();
  int gettype();
  double getsnr();
  double getmean();
  
  // Pointers to next and previous blobs (for traversal purposes)
  void setnextblob( bloblist *nextblob_in );
  void setprevblob( bloblist *prevblob_in );
  bloblist *getnextblob();
  bloblist *getprevblob();
  
 private:
  bloblist *nextblob;    // head/tail pointers for
  bloblist *prevblob;    // the list.
  
  int type;	         // 0=undefined, 1=point, 2=extended
  
  int flux;              // Flux in this blob
  double x;              // x pixel coordinate of centroid 
  double y;              // y pixel coordinate of centroid
  double snr;            // the SNR of this blob
  double mean;           // mean level in the vicinity of the blob 
};

#endif BLOBLIST_H
